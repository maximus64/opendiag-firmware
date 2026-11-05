/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "bus.h"
#include "can_frame.h"

void can_print_stat(void);
esp_err_t can_bus_setup(int baud_rate);
/** Failure retains resources; retry teardown before setup. */
esp_err_t can_bus_teardown(void);
/**
 * Route the next CAN open to the OBD socket (pins 6/14, false) or to the
 * AUX_CAN header J3 (true). AUX always opens listen-only. Refused with
 * ESP_ERR_INVALID_STATE while a bus is open. Not kept across a reset.
 */
esp_err_t can_bus_use_aux(bool aux);
bool can_bus_aux(void);
int can_send(const struct can_frame *frame);
int can_receive(struct can_frame *frame, TickType_t ticks_to_wait);

/**
 * @brief CAN behind the same vtable as every other bus.
 *
 * Frames cross it as bytes; can_frame.h has the format and the two calls
 * that apply it.
 */
extern const bus_ops_t can_bus_ops;
