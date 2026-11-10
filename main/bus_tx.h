/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bus.h"

typedef enum {
    BUS_TX_QUEUED,
    BUS_TX_ACTIVE,
    BUS_TX_CANCELLED,
} bus_tx_state_t;

/* The caller may cancel until the driver admits the first byte/frame. */
struct bus_tx_control {
    uint32_t state;
    bool check_rx;
    uint32_t rx_sequence;
    uint32_t started_us;
    uint32_t ended_us;
    bool ended;
    bool stop; /* Cancelled after admission: a train sends no further frame. */
};

typedef struct {
    int status;
    /* TX end can precede completion of native echo checking. */
    bool complete;
    bool started;
    bool ended; /* ended_us is when the last bit left, not a placeholder. */
    uint32_t started_us;
    uint32_t ended_us;
    /* Set for a train: how many of its frames went out and when. */
    const bus_tx_train_t *train;
} bus_tx_result_t;

typedef struct {
    uint32_t sequence;
    uint32_t lost_sequence;
} bus_rx_epoch_t;

void bus_tx_prepare(bus_tx_control_t *control, uint32_t rx_sequence);
bool bus_tx_cancel(bus_tx_control_t *control);
bool bus_tx_cancelled(const bus_tx_control_t *control);
/* Cancelled before or after admission. */
bool bus_tx_stopped(const bus_tx_control_t *control);
int bus_tx_admit(bus_tx_control_t *control, uint32_t rx_sequence,
                 uint32_t timestamp_us);
void bus_tx_end(bus_tx_control_t *control, uint32_t timestamp_us);

uint32_t bus_rx_next(bus_rx_epoch_t *epoch);
/* Each driver has one listener, which vif binds to the task holding the
 * bus's claim if that task waits on news, and to nobody otherwise. Drivers
 * notify it after queueing received data or losing the link, and the TX
 * worker after a transmission completes. NULL listeners are ignored. */
struct bus_listener {
    TaskHandle_t task;
};
void bus_listen(bus_listener_t *listener, TaskHandle_t task);
void bus_notify(bus_listener_t *listener);
void bus_notify_from_isr(bus_listener_t *listener, BaseType_t *woken);
uint32_t bus_rx_sequence(const bus_rx_epoch_t *epoch);
void bus_rx_lost(bus_rx_epoch_t *epoch, uint32_t sequence);
/* Called only after the driver's receive queue has been drained. */
bool bus_rx_take_loss(bus_rx_epoch_t *epoch, bus_msg_t *msg);

typedef struct bus_tx_worker bus_tx_worker_t;

bus_tx_worker_t *bus_tx_worker_create(const bus_ops_t *ops);
int bus_tx_worker_submit(bus_tx_worker_t *worker, const bus_msg_t *msg,
                         uint32_t flags, uint32_t rx_sequence);
/* Frames of at most eight bytes, sent through ops->send_train. */
int bus_tx_worker_submit_train(bus_tx_worker_t *worker, const bus_msg_t *msgs,
                               unsigned count, uint32_t flags,
                               uint32_t rx_sequence);
bool bus_tx_worker_poll(bus_tx_worker_t *worker, bus_tx_result_t *result);
void bus_tx_worker_cancel(bus_tx_worker_t *worker);
int bus_tx_worker_wait(bus_tx_worker_t *worker);
/* A failed close retains the worker and its request storage for a retry. */
esp_err_t bus_tx_worker_close(bus_tx_worker_t *worker);
