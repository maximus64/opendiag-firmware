/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#define xSemaphoreCreateMutex driver_create_mutex
#define xSemaphoreTake driver_sem_take
#define xSemaphoreGive driver_sem_give
#define vSemaphoreDelete driver_sem_delete
#include "../../stubs/freertos/semphr.h"
SemaphoreHandle_t xSemaphoreCreateBinary(void);
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t sem, BaseType_t *woken);
