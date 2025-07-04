/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdint.h>
#include "freertos/portmacro.h"
#include "can_frame.h"

void can_print_stat(void);
esp_err_t can_bus_setup(int baud_rate);
void can_bus_teardown(void);
int can_send(const struct can_frame *frame);
int can_receive(struct can_frame *frame, TickType_t ticks_to_wait);
