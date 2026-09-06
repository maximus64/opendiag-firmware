/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
extern int driver_allocations, driver_live_objects, driver_fail_allocation;
extern int driver_set_joins, driver_fail_set_join;
extern bool driver_fail_task, driver_exit_on_wait;
extern int driver_task_starts, driver_task_exits, driver_notifications;
void driver_rtos_reset(void);
void driver_run_worker(void);
