/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file portmacro.h
 * @brief Host stand-in for the FreeRTOS port types.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

typedef uint32_t TickType_t;
typedef int BaseType_t;
typedef unsigned UBaseType_t;

#define portTICK_PERIOD_MS 1
#define portMAX_DELAY ((TickType_t)0xFFFFFFFFUL)
