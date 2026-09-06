/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#define xTaskCreate driver_task_create
#define vTaskDelete driver_task_delete
#define vTaskNotifyGiveFromISR driver_task_notify
#include "../../stubs/freertos/task.h"
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(lock) ((void)(lock))
#define portEXIT_CRITICAL(lock) ((void)(lock))
#define portENTER_CRITICAL_ISR(lock) ((void)(lock))
#define portEXIT_CRITICAL_ISR(lock) ((void)(lock))
