/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
typedef struct driver_twai_node *twai_node_handle_t;
typedef enum {
    TWAI_ERROR_ACTIVE,
    TWAI_ERROR_WARNING,
    TWAI_ERROR_PASSIVE,
    TWAI_ERROR_BUS_OFF
} twai_error_state_t;
typedef struct {
    struct {
        uint32_t id;
        uint8_t dlc;
        bool ide, rtr;
    } header;
    uint8_t *buffer;
    size_t buffer_len;
} twai_frame_t;
typedef struct {
    twai_error_state_t state;
    unsigned tx_error_count, rx_error_count;
    unsigned long tx_queue_remaining;
} twai_node_status_t;
typedef struct {
    unsigned long bus_err_num;
} twai_node_record_t;
typedef struct {
    int unused;
} twai_rx_done_event_data_t;
typedef struct {
    twai_error_state_t old_sta, new_sta;
} twai_state_change_event_data_t;
typedef struct {
    bool is_tx_success;
} twai_tx_done_event_data_t;
typedef union {
    struct {
        uint32_t arb_lost : 1;
        uint32_t bit_err : 1;
        uint32_t form_err : 1;
        uint32_t stuff_err : 1;
        uint32_t ack_err : 1;
    };
    uint32_t val;
} twai_error_flags_t;
typedef struct {
    twai_error_flags_t err_flags;
} twai_error_event_data_t;
typedef struct {
    bool (*on_error)(twai_node_handle_t, const twai_error_event_data_t *,
                     void *);
    bool (*on_tx_done)(twai_node_handle_t, const twai_tx_done_event_data_t *,
                       void *);
    bool (*on_rx_done)(twai_node_handle_t, const twai_rx_done_event_data_t *,
                       void *);
    bool (*on_state_change)(twai_node_handle_t,
                            const twai_state_change_event_data_t *, void *);
} twai_event_callbacks_t;
esp_err_t twai_node_register_event_callbacks(twai_node_handle_t node,
                                             const twai_event_callbacks_t *cbs,
                                             void *arg);
esp_err_t twai_node_enable(twai_node_handle_t node);
esp_err_t twai_node_disable(twai_node_handle_t node);
esp_err_t twai_node_delete(twai_node_handle_t node);
esp_err_t twai_node_recover(twai_node_handle_t node);
esp_err_t twai_node_transmit(twai_node_handle_t node, const twai_frame_t *frame,
                             int timeout_ms);
esp_err_t twai_node_receive_from_isr(twai_node_handle_t node,
                                     twai_frame_t *frame);
esp_err_t twai_node_get_info(twai_node_handle_t node,
                             twai_node_status_t *status,
                             twai_node_record_t *record);

esp_err_t twai_node_transmit_wait_all_done(twai_node_handle_t node,
                                           int timeout_ms);
