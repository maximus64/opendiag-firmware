/* SPDX-License-Identifier: GPL-3.0-only */
#include "driver_rtos.h"
#include <stdlib.h>
#include <string.h>
#include "fake_clock.h"
#include "td_test.h"

struct driver_queue {
    struct driver_queue *next;
    bool deleted, mutex;
    unsigned count, capacity, item_size;
    void *data;
};
static struct driver_queue *objects;
static TaskFunction_t worker;
static void *worker_arg;
static bool worker_alive;
int driver_allocations, driver_live_objects, driver_fail_allocation;
bool driver_fail_task, driver_exit_on_wait;
int driver_set_joins, driver_fail_set_join;
int driver_task_starts, driver_task_exits, driver_notifications;

void driver_rtos_reset(void) {
    TEST_ASSERT_EQUAL_INT(0, driver_live_objects);
    TEST_ASSERT_FALSE(worker_alive);
    while (objects) {
        struct driver_queue *next = objects->next;
        free(objects);
        objects = next;
    }
    driver_allocations = driver_fail_allocation = 0;
    driver_set_joins = driver_fail_set_join = 0;
    driver_task_starts = driver_task_exits = driver_notifications = 0;
    driver_fail_task = driver_exit_on_wait = false;
    worker = NULL;
}

QueueHandle_t xQueueCreate(UBaseType_t count, UBaseType_t size) {
    driver_allocations++;
    if (driver_allocations == driver_fail_allocation)
        return NULL;
    struct driver_queue *q = calloc(1, sizeof(*q));
    TEST_ASSERT_NOT_NULL(q);
    q->capacity = count;
    q->item_size = size;
    q->data = calloc(count, size ? size : 1);
    TEST_ASSERT_NOT_NULL(q->data);
    q->next = objects;
    objects = q;
    driver_live_objects++;
    return q;
}
void vQueueDelete(QueueHandle_t q) {
    TEST_ASSERT_NOT_NULL(q);
    TEST_ASSERT_FALSE(q->deleted);
    q->deleted = true;
    free(q->data);
    q->data = NULL;
    driver_live_objects--;
}
BaseType_t xQueueReset(QueueHandle_t q) {
    TEST_ASSERT_FALSE(q->deleted);
    q->count = 0;
    return pdPASS;
}
BaseType_t xQueueSend(QueueHandle_t q, const void *item, TickType_t wait) {
    TEST_ASSERT_FALSE(q->deleted);
    if (q->count == q->capacity)
        return pdFALSE;
    if (q->item_size)
        memcpy((char *)q->data + q->count * q->item_size, item, q->item_size);
    q->count++;
    return pdTRUE;
}
BaseType_t xQueueSendFromISR(QueueHandle_t q, const void *item,
                             BaseType_t *woken) {
    return xQueueSend(q, item, 0);
}
BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t wait) {
    TEST_ASSERT_FALSE(q->deleted);
    if (!q->count)
        return pdFALSE;
    if (q->item_size) {
        memcpy(item, q->data, q->item_size);
        memmove(q->data, (char *)q->data + q->item_size,
                (q->count - 1) * q->item_size);
    }
    q->count--;
    return pdTRUE;
}
QueueSetHandle_t xQueueCreateSet(UBaseType_t n) {
    return xQueueCreate(n, sizeof(void *));
}
BaseType_t xQueueAddToSet(QueueSetMemberHandle_t q, QueueSetHandle_t set) {
    driver_set_joins++;
    return driver_set_joins == driver_fail_set_join ? pdFAIL : pdPASS;
}
BaseType_t xQueueRemoveFromSet(QueueSetMemberHandle_t q, QueueSetHandle_t set) {
    return pdPASS;
}
QueueSetMemberHandle_t xQueueSelectFromSet(QueueSetHandle_t set,
                                           TickType_t wait) {
    return NULL;
}
SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    struct driver_queue *q = xQueueCreate(1, 0);
    if (q) {
        q->mutex = true;
        q->count = 1;
    }
    return q;
}
SemaphoreHandle_t xSemaphoreCreateBinary(void) { return xQueueCreate(1, 0); }
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem) {
    return xQueueSend(sem, NULL, 0);
}
BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t sem, BaseType_t *woken) {
    return xSemaphoreGive(sem);
}
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t wait) {
    struct driver_queue *q = sem;
    TEST_ASSERT_NOT_NULL(q);
    TEST_ASSERT_FALSE(q->deleted);
    if (!q->count && wait && driver_exit_on_wait && worker_alive && !q->mutex) {
        driver_run_worker();
    }
    if (!q->count && wait) {
        TEST_ASSERT_FALSE(q->mutex);
        TEST_ASSERT_TRUE(wait != portMAX_DELAY);
        fake_clock_advance_ms(wait);
    }
    return xQueueReceive(q, NULL, 0);
}
void vSemaphoreDelete(SemaphoreHandle_t sem) { vQueueDelete(sem); }
BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint32_t stack,
                       void *arg, UBaseType_t priority, TaskHandle_t *created) {
    if (driver_fail_task)
        return pdFAIL;
    TEST_ASSERT_FALSE(worker_alive);
    worker = fn;
    worker_arg = arg;
    worker_alive = true;
    *created = &worker;
    driver_task_starts++;
    return pdPASS;
}
void driver_run_worker(void) {
    TEST_ASSERT_TRUE(worker_alive);
    worker(worker_arg);
    TEST_ASSERT_FALSE(worker_alive);
}
void vTaskDelete(TaskHandle_t task) {
    TEST_ASSERT_NULL(task); /* Never force-delete a live worker. */
    TEST_ASSERT_TRUE(worker_alive);
    worker_alive = false;
    driver_task_exits++;
}
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken) {
    TEST_ASSERT_TRUE(worker_alive);
    TEST_ASSERT_NOT_NULL(task);
    driver_notifications++;
}
