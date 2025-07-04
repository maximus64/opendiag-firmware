/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file semphr.h
 * @brief Host stand-in for FreeRTOS semaphores.
 *
 * Tests are single threaded, so a mutex has nothing to protect. Take and give
 * are counted anyway, which is enough to catch an unbalanced lock in code that
 * has several early-return paths under LOCK().
 */

#pragma once

#include "freertos/FreeRTOS.h"

typedef void *SemaphoreHandle_t;

SemaphoreHandle_t xSemaphoreCreateMutex(void);
void vSemaphoreDelete(SemaphoreHandle_t sem);
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout);
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem);

/** Net take/give balance across every mutex. Non-zero means a leaked lock. */
int idf_stub_lock_balance(void);
