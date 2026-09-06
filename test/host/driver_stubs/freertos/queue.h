/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include "freertos/FreeRTOS.h"
typedef struct driver_queue *QueueHandle_t;
typedef QueueHandle_t QueueSetHandle_t;
typedef QueueHandle_t QueueSetMemberHandle_t;
QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size);
QueueSetHandle_t xQueueCreateSet(UBaseType_t length);
BaseType_t xQueueAddToSet(QueueSetMemberHandle_t q, QueueSetHandle_t set);
BaseType_t xQueueRemoveFromSet(QueueSetMemberHandle_t q, QueueSetHandle_t set);
QueueSetMemberHandle_t xQueueSelectFromSet(QueueSetHandle_t set,
                                           TickType_t wait);
BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t wait);
BaseType_t xQueueSendFromISR(QueueHandle_t q, const void *item,
                             BaseType_t *woken);
BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t wait);
BaseType_t xQueueReset(QueueHandle_t q);
void vQueueDelete(QueueHandle_t q);
