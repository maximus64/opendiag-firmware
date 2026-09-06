/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include "esp_twai.h"
typedef struct {
    struct {
        int tx, rx, quanta_clk_out, bus_off_indicator;
    } io_cfg;
    struct {
        uint32_t bitrate;
        uint16_t sp_permill;
    } bit_timing;
    unsigned tx_queue_depth;
    int fail_retry_cnt;
} twai_onchip_node_config_t;
esp_err_t twai_new_node_onchip(const twai_onchip_node_config_t *cfg,
                               twai_node_handle_t *node);
