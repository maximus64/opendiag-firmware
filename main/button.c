/* SPDX-License-Identifier: GPL-3.0-only */
#include "button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "pinout.h"

#define TAG "BUTTON"

/** Contact settle time before trusting a level. */
#define BUTTON_DEBOUNCE_MS 20

/** How long it has to be held before the handler runs. */
#define BUTTON_HOLD_MS 2000
#define BUTTON_LONG_HOLD_MS 10000
#define BUTTON_CLICK_MIN_MS 100

#define BUTTON_TASK_STACK 2048

static button_hold_cb_t g_on_hold;
static button_hold_cb_t g_on_long_hold;
static button_click_cb_t g_on_click;

static TaskHandle_t g_task;

/* -1 while idle; otherwise esp_timer_get_time() at the debounced press. */
static int64_t g_press_start_us = -1;
static bool g_fired;
static bool g_fired_long;

/** GPIO 0 is active low. It is also the download mode strapping pin, which is
 *  a reset-time function and does not collide with reading it here. */
static bool button_is_down(void) { return gpio_get_level(PIN_BUTTON) == 0; }

/** Wakes button_step() on every edge; the task re-reads the level itself. */
static void IRAM_ATTR button_isr(void *arg) {
    BaseType_t woken = pdFALSE;

    (void)arg;
    vTaskNotifyGiveFromISR(g_task, &woken);
    if (woken) {
        portYIELD_FROM_ISR();
    }
}

/** Blocks up to @p ms for the next edge. */
static bool wait_for_edge(uint32_t ms) {
    return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ms)) != 0;
}

/**
 * @brief One idle-wait, or one press-to-threshold-or-release step.
 *
 * Called in a loop by button_task(); driven call-by-call by host tests, the
 * same way the old poll body was.
 */
static void button_step(void) {
    uint32_t elapsed_ms;
    bool released;

    if (g_press_start_us < 0) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* idle: wait for a press */
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
        if (!button_is_down()) {
            return; /* bounce */
        }

        g_press_start_us = esp_timer_get_time();
        g_fired = false;
        g_fired_long = false;
        return;
    }

    elapsed_ms = (uint32_t)((esp_timer_get_time() - g_press_start_us) / 1000);

    if (g_fired && g_fired_long) {
        ulTaskNotifyTake(pdTRUE,
                         portMAX_DELAY); /* both fired; just wait it out */
        released = true;
    } else {
        uint32_t deadline_ms = g_fired ? BUTTON_LONG_HOLD_MS : BUTTON_HOLD_MS;
        uint32_t wait_ms =
            deadline_ms > elapsed_ms ? deadline_ms - elapsed_ms : 0;

        released = wait_for_edge(wait_ms);
        if (released) {
            /* Measure at the edge, not after the debounce settle below. */
            elapsed_ms =
                (uint32_t)((esp_timer_get_time() - g_press_start_us) / 1000);
        }
    }

    if (released) {
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
        if (button_is_down()) {
            return; /* bounce on the way up; still mid-press */
        }

        if (!g_fired && elapsed_ms >= BUTTON_CLICK_MIN_MS && g_on_click) {
            g_on_click();
        }
        g_press_start_us = -1;
        return;
    }

    /* Timed out: whichever threshold was pending has been reached. */
    if (!g_fired) {
        g_fired = true;
        if (g_on_hold) {
            g_on_hold();
        }
    } else {
        g_fired_long = true;
        if (g_on_long_hold) {
            g_on_long_hold();
        }
    }
}

static void button_task(void *param) {
    (void)param;

    for (;;) {
        button_step();
    }
}

void button_init(void) {
    gpio_reset_pin(PIN_BUTTON);
    gpio_set_direction(PIN_BUTTON, GPIO_MODE_INPUT);

    if (xTaskCreate(button_task, "button", BUTTON_TASK_STACK, NULL, 4,
                    &g_task) != pdPASS) {
        ESP_LOGE(TAG, "failed to start the button task");
        return;
    }

    if (gpio_set_intr_type(PIN_BUTTON, GPIO_INTR_ANYEDGE) != ESP_OK ||
        gpio_isr_handler_add(PIN_BUTTON, button_isr, NULL) != ESP_OK ||
        gpio_intr_enable(PIN_BUTTON) != ESP_OK) {
        ESP_LOGE(TAG, "failed to attach the button ISR");
    }
}

void button_set_hold_callback(button_hold_cb_t cb) { g_on_hold = cb; }

void button_set_long_hold_callback(button_hold_cb_t cb) { g_on_long_hold = cb; }

void button_set_click_callback(button_click_cb_t cb) { g_on_click = cb; }
