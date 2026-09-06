/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file task.h
 * @brief Host stand-in for the FreeRTOS task API.
 *
 * Tests are single threaded: xTaskCreate() records the request and returns
 * without starting anything. Code that would run in a task is driven directly
 * by the test instead.
 */

#pragma once

#include "freertos/FreeRTOS.h"

typedef void *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);

/** @brief How many tasks have been asked for since the last reset. */
int idf_stub_task_starts(void);
void idf_stub_reset_task_starts(void);
void idf_stub_set_current_task(TaskHandle_t task);
void idf_stub_set_created_task(TaskHandle_t task);

BaseType_t xTaskCreate(TaskFunction_t fn, const char *name,
                       uint32_t stack_depth, void *param, UBaseType_t priority,
                       TaskHandle_t *created);

TickType_t xTaskGetTickCount(void);

/**
 * @brief The thread vif's ownership rule is asked about.
 *
 * Defaults to a non-NULL identity; tests can simulate another caller.
 */
TaskHandle_t xTaskGetCurrentTaskHandle(void);

/** Advances the fake clock, exactly as a real delay would let it advance. */
void vTaskDelay(TickType_t ticks);

/** No task was ever started, so there is nothing to delete. */
void vTaskDelete(TaskHandle_t task);

void vPortYield(void);

/* Task notifications: a single pending flag, since nothing here needs a
 * counting one. fake_gpio.c posts to it on a simulated edge. */
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken);
uint32_t ulTaskNotifyTake(BaseType_t clear_on_exit, TickType_t ticks_to_wait);

#define portYIELD_FROM_ISR() ((void)0)
