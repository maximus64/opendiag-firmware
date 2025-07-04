/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include <stdint.h>
#include "freertos/FreeRTOS.h" // for TickType_t

void kline_setup(void);
void kline_teardown(void);
int kline_sync(void);
int kline_send(const uint8_t *data, uint8_t len);
int kline_recieve(uint8_t *data, uint8_t len, TickType_t xTicksToWait);
