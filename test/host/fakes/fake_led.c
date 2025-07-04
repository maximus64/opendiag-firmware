/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_led.c
 * @brief Test double for main/ws2812_led.c.
 */

#include "fake_led.h"

static led_state_t g_state = LED_STATE_IDLE;
static int g_changes;

void fake_led_reset(void)
{
    g_state = LED_STATE_IDLE;
    g_changes = 0;
}

led_state_t fake_led_state(void)
{
    return g_state;
}

int fake_led_change_count(void)
{
    return g_changes;
}

/* ------------------------------------------------------------------ *
 * ws2812_led.h implementation
 * ------------------------------------------------------------------ */

void ws2812_led_init(void)
{
    fake_led_reset();
}

void ws2812_led_set_state(led_state_t new_state)
{
    if (new_state != g_state) {
        g_changes++;
    }
    g_state = new_state;
}
