/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include "esp_err.h"
#include "driver/rmt_types.h"

typedef struct rmt_channel_t *rmt_channel_handle_t;
esp_err_t rmt_enable(rmt_channel_handle_t channel);
esp_err_t rmt_disable(rmt_channel_handle_t channel);
esp_err_t rmt_del_channel(rmt_channel_handle_t channel);
esp_err_t rmt_tx_wait_all_done(rmt_channel_handle_t channel, int timeout_ms);
