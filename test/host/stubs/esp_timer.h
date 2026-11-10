/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file esp_timer.h
 * @brief Host stand-in for the ESP-IDF high resolution timer.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/** Microseconds on the fake clock. */
int64_t esp_timer_get_time(void);

/* One-shot timers on the fake clock. A blocking ulTaskNotifyTake() whose wait
 * covers an armed timer's expiry advances the clock to it and fires it, the
 * way the real timer task would cut the sleep short. */
typedef struct esp_timer *esp_timer_handle_t;
typedef void (*esp_timer_cb_t)(void *arg);

typedef struct {
    esp_timer_cb_t callback;
    void *arg;
    const char *name;
} esp_timer_create_args_t;

esp_err_t esp_timer_create(const esp_timer_create_args_t *args,
                           esp_timer_handle_t *out);
esp_err_t esp_timer_start_once(esp_timer_handle_t timer, uint64_t timeout_us);
esp_err_t esp_timer_stop(esp_timer_handle_t timer);

/** True while @p timer is armed; its expiry in fake-clock microseconds. */
bool fake_esp_timer_armed(esp_timer_handle_t timer, uint64_t *due_us);
