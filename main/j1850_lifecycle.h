/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_attr.h"
#include "driver/gptimer.h"
#include "driver/rmt_tx.h"

typedef struct {
    bool stopping;
    uint32_t active;
} j1850_callback_guard_t;

/* Count before checking the gate so shutdown cannot miss an admitted ISR. */
static inline __attribute__((always_inline)) bool IRAM_ATTR
j1850_callback_enter(j1850_callback_guard_t *guard) {
    __atomic_add_fetch(&guard->active, 1, __ATOMIC_SEQ_CST);
    if (__atomic_load_n(&guard->stopping, __ATOMIC_SEQ_CST)) {
        __atomic_sub_fetch(&guard->active, 1, __ATOMIC_SEQ_CST);
        return false;
    }
    return true;
}

static inline __attribute__((always_inline)) void IRAM_ATTR
j1850_callback_exit(j1850_callback_guard_t *guard) {
    __atomic_sub_fetch(&guard->active, 1, __ATOMIC_SEQ_CST);
}

static inline void j1850_callbacks_start(j1850_callback_guard_t *guard) {
    __atomic_store_n(&guard->stopping, false, __ATOMIC_SEQ_CST);
}

esp_err_t j1850_callbacks_stop(j1850_callback_guard_t *guard);
esp_err_t j1850_rmt_enable(rmt_channel_handle_t channel, bool *enabled);
esp_err_t j1850_rmt_delete(rmt_channel_handle_t *channel, bool *enabled);
esp_err_t j1850_rmt_recover(rmt_channel_handle_t channel, bool *enabled);
esp_err_t j1850_timer_enable(gptimer_handle_t timer, bool *enabled);
esp_err_t j1850_timer_start(gptimer_handle_t timer, bool *running);
esp_err_t j1850_timer_delete(gptimer_handle_t *timer, bool *enabled,
                             bool *running);
