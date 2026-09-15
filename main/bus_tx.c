/* SPDX-License-Identifier: GPL-3.0-only */
#include "bus_tx.h"

#include <string.h>
#include "esp_attr.h"

void bus_tx_prepare(bus_tx_control_t *control, uint32_t rx_sequence) {
    memset(control, 0, sizeof(*control));
    control->rx_sequence = rx_sequence;
    __atomic_store_n(&control->state, BUS_TX_QUEUED, __ATOMIC_RELEASE);
}

bool bus_tx_cancel(bus_tx_control_t *control) {
    uint32_t queued = BUS_TX_QUEUED;

    return __atomic_compare_exchange_n(&control->state, &queued,
                                       BUS_TX_CANCELLED, false,
                                       __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

bool bus_tx_cancelled(const bus_tx_control_t *control) {
    return control && __atomic_load_n(&control->state, __ATOMIC_ACQUIRE) ==
                          BUS_TX_CANCELLED;
}

int bus_tx_admit(bus_tx_control_t *control, uint32_t rx_sequence,
                 uint32_t timestamp_us) {
    if (!control)
        return 0;
    uint32_t state = __atomic_load_n(&control->state, __ATOMIC_ACQUIRE);

    if (state == BUS_TX_CANCELLED)
        return BUS_ERR_CANCELLED;
    if (state == BUS_TX_ACTIVE)
        return 0;
    if (control->check_rx && control->rx_sequence != rx_sequence)
        return BUS_ERR_RX_PENDING;

    control->started_us = timestamp_us;
    if (!__atomic_compare_exchange_n(&control->state, &state, BUS_TX_ACTIVE,
                                     false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
        return BUS_ERR_CANCELLED;
    return 0;
}

void bus_tx_end(bus_tx_control_t *control, uint32_t timestamp_us) {
    if (control) {
        __atomic_store_n(&control->ended_us, timestamp_us, __ATOMIC_RELAXED);
        __atomic_store_n(&control->ended, true, __ATOMIC_RELEASE);
    }
}

uint32_t IRAM_ATTR bus_rx_next(bus_rx_epoch_t *epoch) {
    uint32_t sequence =
        __atomic_add_fetch(&epoch->sequence, 1, __ATOMIC_ACQ_REL);
    if (!sequence)
        sequence = __atomic_add_fetch(&epoch->sequence, 1, __ATOMIC_ACQ_REL);
    return sequence;
}

uint32_t bus_rx_sequence(const bus_rx_epoch_t *epoch) {
    return __atomic_load_n(&epoch->sequence, __ATOMIC_ACQUIRE);
}

void IRAM_ATTR bus_rx_lost(bus_rx_epoch_t *epoch, uint32_t sequence) {
    uint32_t previous =
        __atomic_load_n(&epoch->lost_sequence, __ATOMIC_ACQUIRE);
    while (!previous || (int32_t)(sequence - previous) > 0) {
        if (__atomic_compare_exchange_n(&epoch->lost_sequence, &previous,
                                        sequence, false, __ATOMIC_ACQ_REL,
                                        __ATOMIC_ACQUIRE))
            break;
    }
}

bool bus_rx_take_loss(bus_rx_epoch_t *epoch, bus_msg_t *msg) {
    uint32_t sequence =
        __atomic_exchange_n(&epoch->lost_sequence, 0, __ATOMIC_ACQ_REL);
    if (!sequence)
        return false;
    msg->len = 0;
    msg->status = BUS_RX_BUFFER_OVERFLOW;
    msg->sequence = sequence;
    return true;
}
