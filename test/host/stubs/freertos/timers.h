/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file timers.h
 * @brief Host stand-in for the FreeRTOS software timer header.
 *
 * timer.c includes this for xTaskGetTickCount(), which the real header pulls
 * in transitively, so this one does too.
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
