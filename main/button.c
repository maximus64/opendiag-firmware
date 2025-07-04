/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file button.c
 * @brief The front panel button. See button.h for why it lives outside vif.
 */

#include "button.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pinout.h"

#define TAG "button"

/** How often the button is sampled. Also the debounce, in effect: a hold has
 *  to survive many consecutive samples before it counts. */
#define BUTTON_POLL_MS 50

/** How long it has to be held before the handler runs. */
#define BUTTON_HOLD_MS 2000

/** Stack for the watcher. It calls one callback and formats no strings. */
#define BUTTON_TASK_STACK 2048

static button_hold_cb_t g_on_hold;

/* Whether the watcher is up. Not the task handle: nothing here needs to
 * address the task, and a handle would only be as trustworthy as whatever
 * xTaskCreate() chose to write into it. */
static bool g_watching;

/* Per-press state. Held across samples, cleared the moment the button is up. */
static uint32_t g_held_ms;
static bool g_fired;

/** GPIO 0 is active low. It is also the download mode strapping pin, which is
 *  a reset-time function and does not collide with reading it here. */
static bool button_is_down(void)
{
    return gpio_get_level(PIN_BUTTON) == 0;
}

/**
 * @brief Take one sample. The task's body, without the loop around it.
 *
 * Split out so the host tests can drive the press timing without a scheduler,
 * the same way the session task's body is driven in the other suites.
 */
static void button_poll(void)
{
    if (!button_is_down()) {
        g_held_ms = 0;
        g_fired = false;
        return;
    }

    g_held_ms += BUTTON_POLL_MS;

    if (g_fired || g_held_ms < BUTTON_HOLD_MS) {
        return;
    }

    /* Once per press, not once per sample while it stays down. */
    g_fired = true;

    if (g_on_hold) {
        g_on_hold();
    }
}

static void button_task(void *param)
{
    for (;;) {
        button_poll();
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

void button_init(void)
{
    gpio_reset_pin(PIN_BUTTON);
    gpio_set_direction(PIN_BUTTON, GPIO_MODE_INPUT);
}

void button_set_hold_callback(button_hold_cb_t cb)
{
    g_on_hold = cb;

    if (g_watching) {
        return;
    }

    if (xTaskCreate(button_task, "button", BUTTON_TASK_STACK, NULL, 4, NULL)
        != pdPASS) {
        ESP_LOGE(TAG, "failed to start the button task");
        return;
    }

    g_watching = true;
}
