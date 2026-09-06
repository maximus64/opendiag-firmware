/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file FreeRTOS.h
 * @brief Host stand-in for the FreeRTOS kernel header.
 *
 * The tick rate is fixed at 1 kHz so that one tick is one millisecond and
 * pdMS_TO_TICKS() is the identity. Firmware code converts between the two in
 * several places (timer.c divides by configTICK_RATE_HZ); keeping the ratio at
 * one keeps those conversions exact and the tests readable.
 *
 * The target runs at 1 kHz too (CONFIG_FREERTOS_HZ in sdkconfig.defaults), so
 * a millisecond means the same thing here and on the device. It did not always
 * - at the IDF default of 100 Hz every timeout under 10 ms silently became
 * "do not wait", which a host test with this stub could never have caught.
 */

#pragma once

#include <assert.h>
#include "freertos/portmacro.h"

#define configTICK_RATE_HZ 1000

#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define pdTICKS_TO_MS(t) ((uint32_t)(t))

#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdFAIL 0
