/* SPDX-License-Identifier: GPL-3.0-only */
#include "bus_tx.h"

#include <stdlib.h>
#include <string.h>
#include "freertos/semphr.h"
#include "freertos/task.h"

#define TX_DATA_MAX 260u
#define TX_WAIT_MS 4000u

enum { WORKER_IDLE, WORKER_PENDING, WORKER_DONE };

struct bus_tx_worker {
    const bus_ops_t *ops;
    SemaphoreHandle_t wake;
    SemaphoreHandle_t done;
    SemaphoreHandle_t exited;
    TaskHandle_t task;
    bool stopping;
    uint32_t state;
    bus_tx_control_t control;
    bus_msg_t message;
    uint32_t flags;
    int result;
    bool timing_reported;
    uint8_t data[TX_DATA_MAX];
};

static void transmit_task(void *argument) {
    bus_tx_worker_t *worker = argument;

    for (;;) {
        xSemaphoreTake(worker->wake, portMAX_DELAY);
        if (__atomic_load_n(&worker->state, __ATOMIC_ACQUIRE) ==
            WORKER_PENDING) {
            worker->result =
                bus_tx_cancelled(&worker->control)
                    ? BUS_ERR_CANCELLED
                    : worker->ops->send_controlled(
                          &worker->message, worker->flags, &worker->control);
            __atomic_store_n(&worker->state, WORKER_DONE, __ATOMIC_RELEASE);
            xSemaphoreGive(worker->done);
        }
        if (__atomic_load_n(&worker->stopping, __ATOMIC_ACQUIRE))
            break;
    }
    xSemaphoreGive(worker->exited);
    vTaskDelete(NULL);
}

static void release_worker(bus_tx_worker_t *worker) {
    if (worker->wake)
        vSemaphoreDelete(worker->wake);
    if (worker->done)
        vSemaphoreDelete(worker->done);
    if (worker->exited)
        vSemaphoreDelete(worker->exited);
    free(worker);
}

bus_tx_worker_t *bus_tx_worker_create(const bus_ops_t *ops) {
    if (!ops || !ops->send_controlled)
        return NULL;
    bus_tx_worker_t *worker = calloc(1, sizeof(*worker));
    if (!worker)
        return NULL;
    worker->ops = ops;
    worker->wake = xSemaphoreCreateBinary();
    worker->done = xSemaphoreCreateBinary();
    worker->exited = xSemaphoreCreateBinary();
    if (!worker->wake || !worker->done || !worker->exited ||
        xTaskCreate(transmit_task, "bus-tx", 3072, worker, 10, &worker->task) !=
            pdPASS) {
        release_worker(worker);
        return NULL;
    }
    return worker;
}

int bus_tx_worker_submit(bus_tx_worker_t *worker, const bus_msg_t *msg,
                         uint32_t flags, uint32_t rx_sequence) {
    if (!worker || !msg || !msg->data || msg->len > TX_DATA_MAX)
        return BUS_ERR_BAD_ARG;
    if (__atomic_load_n(&worker->stopping, __ATOMIC_ACQUIRE))
        return BUS_ERR_NOT_READY;
    if (__atomic_load_n(&worker->state, __ATOMIC_ACQUIRE) != WORKER_IDLE)
        return BUS_ERR_NO_SPACE;

    xSemaphoreTake(worker->done, 0);
    memcpy(worker->data, msg->data, msg->len);
    worker->message = *msg;
    worker->message.data = worker->data;
    worker->flags = flags;
    worker->timing_reported = false;
    bus_tx_prepare(&worker->control, rx_sequence);
    worker->control.check_rx = !!(flags & BUS_TX_CHECK_RX);
    __atomic_store_n(&worker->state, WORKER_PENDING, __ATOMIC_RELEASE);
    xSemaphoreGive(worker->wake);
    return 0;
}

bool bus_tx_worker_poll(bus_tx_worker_t *worker, bus_tx_result_t *result) {
    if (!worker)
        return false;
    uint32_t state = __atomic_load_n(&worker->state, __ATOMIC_ACQUIRE);
    bool complete = state == WORKER_DONE;
    bool ended = __atomic_load_n(&worker->control.ended, __ATOMIC_ACQUIRE);
    if (state == WORKER_IDLE ||
        (!complete && (!ended || worker->timing_reported)))
        return false;
    *result = (bus_tx_result_t){
        .status = complete ? worker->result : 0,
        .complete = complete,
        .started = __atomic_load_n(&worker->control.state, __ATOMIC_ACQUIRE) ==
                   BUS_TX_ACTIVE,
        .started_us = worker->control.started_us,
        .ended_us =
            __atomic_load_n(&worker->control.ended_us, __ATOMIC_RELAXED),
    };
    worker->timing_reported = true;
    if (complete)
        __atomic_store_n(&worker->state, WORKER_IDLE, __ATOMIC_RELEASE);
    return true;
}

void bus_tx_worker_cancel(bus_tx_worker_t *worker) {
    if (worker &&
        __atomic_load_n(&worker->state, __ATOMIC_ACQUIRE) == WORKER_PENDING)
        bus_tx_cancel(&worker->control);
}

int bus_tx_worker_wait(bus_tx_worker_t *worker) {
    if (!worker)
        return 0;
    TickType_t started = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(TX_WAIT_MS);
    while (__atomic_load_n(&worker->state, __ATOMIC_ACQUIRE) ==
           WORKER_PENDING) {
        TickType_t elapsed = xTaskGetTickCount() - started;
        if (elapsed >= timeout ||
            xSemaphoreTake(worker->done, timeout - elapsed) != pdTRUE)
            return BUS_ERR_TIMEOUT;
    }
    return 0;
}

esp_err_t bus_tx_worker_close(bus_tx_worker_t *worker) {
    if (!worker)
        return ESP_OK;
    __atomic_store_n(&worker->stopping, true, __ATOMIC_RELEASE);
    bus_tx_worker_cancel(worker);
    xSemaphoreGive(worker->wake);
    if (xSemaphoreTake(worker->exited, pdMS_TO_TICKS(TX_WAIT_MS)) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    release_worker(worker);
    return ESP_OK;
}
