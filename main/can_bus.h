/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "bus.h"
#include "can_frame.h"

void can_print_stat(void);
/** TEC/REC of the open bus. ESP_ERR_INVALID_STATE when no bus is open: the
 *  counters only mean anything while the controller is on the bus. */
esp_err_t can_bus_error_counters(uint8_t *tec, uint8_t *rec);
/**
 * Valid frames and bus errors since the bus was opened. Both zero when it is
 * not open. What tells a bit rate that matches the bus from one that does
 * not, without transmitting: see canbaud in the shell.
 */
void can_bus_activity(uint32_t *frames, uint32_t *errors);
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
