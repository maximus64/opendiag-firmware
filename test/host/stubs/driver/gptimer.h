/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include <stdint.h>
#include "esp_err.h"

typedef struct gptimer_t *gptimer_handle_t;
typedef struct {
    uint64_t alarm_count;
} gptimer_alarm_config_t;
esp_err_t gptimer_enable(gptimer_handle_t timer);
esp_err_t gptimer_start(gptimer_handle_t timer);
esp_err_t gptimer_stop(gptimer_handle_t timer);
esp_err_t gptimer_disable(gptimer_handle_t timer);
esp_err_t gptimer_del_timer(gptimer_handle_t timer);
esp_err_t gptimer_set_alarm_action(gptimer_handle_t timer,
                                   const gptimer_alarm_config_t *config);
