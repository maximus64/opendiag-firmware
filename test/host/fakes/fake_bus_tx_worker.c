/* SPDX-License-Identifier: GPL-3.0-only */
#include "fake_bus_tx_worker.h"
#include <stdlib.h>
#include <string.h>
#include "esp_timer.h"
#include "bus_tx.h"

struct bus_tx_worker {
    const bus_ops_t *ops;
    bus_msg_t message;
    uint8_t data[260];
    uint32_t flags;
    bus_tx_control_t control;
    bool pending;
    bool executed;
    bool timing_reported;
    int result;
};

static bool held;
static bool completion_held;
void fake_bus_tx_hold_completion(bool hold) { completion_held = hold; }
void fake_bus_tx_hold(bool hold) { held = hold; }

bus_tx_worker_t *bus_tx_worker_create(const bus_ops_t *ops) {
    bus_tx_worker_t *worker = calloc(1, sizeof(*worker));
    if (worker)
        worker->ops = ops;
    return worker;
}
int bus_tx_worker_submit(bus_tx_worker_t *worker, const bus_msg_t *msg,
                         uint32_t flags, uint32_t rx_sequence) {
    if (worker->pending)
        return BUS_ERR_NO_SPACE;
    if (msg->len > sizeof(worker->data))
        return BUS_ERR_TOO_LONG;
    memcpy(worker->data, msg->data, msg->len);
    worker->message = *msg;
    worker->message.data = worker->data;
    worker->flags = flags;
    bus_tx_prepare(&worker->control, rx_sequence);
    worker->control.check_rx = !!(flags & BUS_TX_CHECK_RX);
    worker->pending = true;
    worker->executed = worker->timing_reported = false;
    return 0;
}
bool bus_tx_worker_poll(bus_tx_worker_t *worker, bus_tx_result_t *result) {
    if (!worker || !worker->pending || held)
        return false;
    if (!worker->executed) {
        worker->result = worker->ops->send_controlled(
            &worker->message, worker->flags, &worker->control);
        worker->executed = true;
    }
    bool complete = !completion_held;
    if (!complete && (!worker->control.ended || worker->timing_reported))
        return false;
    *result = (bus_tx_result_t){
        .status = worker->result,
        .complete = complete,
        .started = worker->control.state == BUS_TX_ACTIVE,
        .started_us = worker->control.started_us,
        .ended_us = worker->control.ended_us,
    };
    worker->pending = !complete;
    worker->timing_reported = true;
    return true;
}
void bus_tx_worker_cancel(bus_tx_worker_t *worker) {
    if (worker && worker->pending)
        bus_tx_cancel(&worker->control);
}
int bus_tx_worker_wait(bus_tx_worker_t *worker) {
    if (!worker || !worker->pending)
        return 0;
    if (held || completion_held)
        return BUS_ERR_TIMEOUT;
    if (!worker->executed) {
        worker->result = worker->ops->send_controlled(
            &worker->message, worker->flags, &worker->control);
        worker->executed = true;
    }
    return 0;
}
esp_err_t bus_tx_worker_close(bus_tx_worker_t *worker) {
    free(worker);
    return ESP_OK;
}
