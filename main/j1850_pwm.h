/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h" // for TickType_t

void j1850_pwm_setup(void);
void j1850_pwm_teardown(void);
int j1850_pwm_send(const uint8_t *data, uint8_t len);
int j1850_pwm_receive(uint8_t *data, uint8_t len, TickType_t xTicksToWait);
