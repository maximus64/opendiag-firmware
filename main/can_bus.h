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
int can_send(const struct can_frame *frame);
int can_receive(struct can_frame *frame, TickType_t ticks_to_wait);

/**
 * @brief CAN behind the same vtable as every other bus.
 *
 * Frames cross it as bytes; can_frame.h has the format and the two calls
 * that apply it.
 */
extern const bus_ops_t can_bus_ops;
