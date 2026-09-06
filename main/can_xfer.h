/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

/**
 * @file can_xfer.h
 * @brief CAN frames over a vif session.
 *
 * vif carries messages, not frames: one set of calls over four buses, with
 * the arbitration id riding in bus_msg_t like any other field. A CAN client
 * still wants to work in frames, so the field moves that turn one into the
 * other live here rather than bulging vif.h with a bus-shaped pair of calls
 * the other three buses have no answer to.
 *
 * The frame stays the front-end's type on purpose. struct can_frame owns its
 * eight bytes; bus_msg_t points at somebody else's buffer. ISO-TP holds a
 * frame across a flow control exchange, and a borrowed pointer would turn
 * that into a question of buffer lifetime.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"

#include "bus.h"
#include "can_frame.h"
#include "vif.h"

/** @brief Send one frame. 0, VIF_ERR_NO_CLAIM, or a BUS_ERR_* code. */
static inline int can_frame_send(vif_session_t *s, const struct can_frame *f) {
    bus_msg_t msg;

    if (!f) {
        return VIF_ERR_NO_CLAIM;
    }

    bus_msg_tx(&msg, f->data, f->dlc, f->id);

    return vif_bus_send(s, VIF_BUS_CAN, &msg, 0);
}

/** @brief Receive one frame. 0 when one arrived, negative otherwise. */
static inline int can_frame_recv(vif_session_t *s, struct can_frame *f,
                                 TickType_t wait) {
    bus_msg_t msg;
    int rc;

    if (!f) {
        return VIF_ERR_NO_CLAIM;
    }

    /* Straight into the caller's frame; the driver's copy is the only one. */
    memset(f, 0, sizeof(*f));
    bus_msg_init(&msg, f->data, sizeof(f->data));

    rc = vif_bus_recv(s, VIF_BUS_CAN, &msg, wait);
    if (rc < 0) {
        return rc;
    }

    f->id = msg.id;
    f->dlc = (uint8_t)msg.len;

    return 0;
}
