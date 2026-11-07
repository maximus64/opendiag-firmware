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
 * Where the next CAN open goes. Refused with ESP_ERR_INVALID_STATE while a
 * bus is open. Not kept across a reset.
 *
 *   OBD  CAN0 on OBD-II pins 6 (H) and 14 (L), the default.
 *   AUX  CAN1 on the AUX_CAN header J3. Always listen-only: it is for tapping
 *        a bus that is not on the OBD socket.
 *   MS   CAN1 on OBD-II pins 3 (H) and 11 (L), the medium speed CAN of Ford,
 *        Mazda and Volvo. Only with R18 and R19 fitted (0 ohm), which the
 *        stock board leaves out; it transmits like the OBD port. Pin 11 then
 *        also carries CAN-L, so vif refuses programming voltage on it.
 */
typedef enum {
    CAN_PORT_OBD = 0,
    CAN_PORT_AUX,
    CAN_PORT_MS,
} can_port_t;

esp_err_t can_bus_set_port(can_port_t port);
can_port_t can_bus_port(void);
const char *can_bus_port_name(can_port_t port);
/** The AUX port as a yes/no, for callers that only care about that. */
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
