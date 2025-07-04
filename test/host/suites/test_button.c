/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_button.c
 * @brief The front panel button's press timing.
 *
 * The button is the last way back when a link is wedged, so what matters is
 * that a hold fires exactly once and a brush against it fires not at all.
 *
 * The firmware source is included rather than linked, for the usual reason:
 * the state machine is static and the only public entry point starts a task
 * that never returns. `button_poll()` is that task's body, and calling it
 * BUTTON_POLL_MS at a time is what the scheduler would have done.
 */

#include "td_test.h"

#include "fake_gpio.h"
#include "pinout.h"

#include "button.c"

/** Samples the real task would take to reach the hold threshold. */
#define SAMPLES_TO_HOLD (BUTTON_HOLD_MS / BUTTON_POLL_MS)

static int g_fired_count;

static void on_hold(void)
{
    g_fired_count++;
}

static void press(void)
{
    fake_gpio_set_level(PIN_BUTTON, 0);   /* active low */
}

static void release(void)
{
    fake_gpio_set_level(PIN_BUTTON, 1);
}

/** Runs @p n samples of the task body. */
static void poll_times(int n)
{
    for (int i = 0; i < n; i++) {
        button_poll();
    }
}

void td_setup(void)
{
    fake_gpio_reset();
    idf_stub_reset_task_starts();

    g_fired_count = 0;
    g_held_ms = 0;
    g_fired = false;
    g_on_hold = on_hold;
}

void td_teardown(void)
{
}

/* ------------------------------------------------------------------ *
 * Setup
 * ------------------------------------------------------------------ */

TEST(init_takes_the_pin_and_configures_it_as_an_input)
{
    fake_gpio_reset();

    button_init();

    /* The button owns GPIO 0 outright; board.c does not touch it. */
    TEST_ASSERT_TRUE(fake_gpio_was_reset(PIN_BUTTON));
    TEST_ASSERT_EQUAL_INT(GPIO_MODE_INPUT, fake_gpio_direction(PIN_BUTTON));
}

/* ------------------------------------------------------------------ *
 * Press timing
 * ------------------------------------------------------------------ */

TEST(an_untouched_button_reads_as_released)
{
    poll_times(SAMPLES_TO_HOLD * 4);

    TEST_ASSERT_EQUAL_INT(0, g_fired_count);
}

TEST(a_hold_fires_once_it_reaches_the_threshold)
{
    press();

    poll_times(SAMPLES_TO_HOLD - 1);
    TEST_ASSERT_MSG(g_fired_count == 0, "fired after only %d ms",
                    (int)g_held_ms);

    button_poll();
    TEST_ASSERT_EQUAL_INT(1, g_fired_count);
    TEST_ASSERT_EQUAL_INT(BUTTON_HOLD_MS, (int)g_held_ms);
}

TEST(holding_it_down_does_not_fire_again)
{
    press();
    poll_times(SAMPLES_TO_HOLD * 5);

    /* Everything is released and both links reset; doing that repeatedly for
     * as long as a thumb stays on the button would be its own fault. */
    TEST_ASSERT_EQUAL_INT(1, g_fired_count);
}

TEST(a_short_press_fires_nothing)
{
    for (int i = 0; i < 5; i++) {
        press();
        poll_times(SAMPLES_TO_HOLD - 1);
        release();
        button_poll();
    }

    TEST_ASSERT_EQUAL_INT(0, g_fired_count);
}

TEST(a_release_forgets_how_long_it_had_been_held)
{
    /* Two nearly-long presses must not add up to one long one. */
    press();
    poll_times(SAMPLES_TO_HOLD - 1);

    release();
    button_poll();
    TEST_ASSERT_EQUAL_INT(0, (int)g_held_ms);

    press();
    poll_times(SAMPLES_TO_HOLD - 1);
    TEST_ASSERT_EQUAL_INT(0, g_fired_count);

    button_poll();
    TEST_ASSERT_EQUAL_INT(1, g_fired_count);
}

TEST(a_second_hold_fires_again)
{
    press();
    poll_times(SAMPLES_TO_HOLD);
    TEST_ASSERT_EQUAL_INT(1, g_fired_count);

    release();
    button_poll();

    press();
    poll_times(SAMPLES_TO_HOLD);
    TEST_ASSERT_EQUAL_INT(2, g_fired_count);
}

/* ------------------------------------------------------------------ *
 * Wiring
 * ------------------------------------------------------------------ */

TEST(a_hold_with_no_handler_installed_does_nothing)
{
    g_on_hold = NULL;

    press();
    poll_times(SAMPLES_TO_HOLD * 2);

    TEST_ASSERT_EQUAL_INT(0, g_fired_count);
}

TEST(installing_the_handler_starts_exactly_one_watcher)
{
    g_watching = false;

    button_set_hold_callback(on_hold);
    TEST_ASSERT_TRUE(g_watching);
    TEST_ASSERT_EQUAL_INT(1, idf_stub_task_starts());

    /* Installing it twice must not leave two tasks polling the same pin. */
    button_set_hold_callback(on_hold);
    TEST_ASSERT_EQUAL_INT(1, idf_stub_task_starts());
}
