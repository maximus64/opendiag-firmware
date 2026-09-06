/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

/**
 * @file can_xfer.h
 * @brief CAN frames over the link's vif claim.
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

/** @brief Send one frame. 0, BUS_ERR_BAD_ARG, or a BUS_ERR_* code. */
static inline int can_frame_send(const struct can_frame *f) {
    bus_msg_t msg;

    if (!f) {
        return BUS_ERR_BAD_ARG;
    }

    bus_msg_tx(&msg, f->data, f->dlc, f->id);

    return vif_bus_send(VIF_OWNER_LINK, VIF_BUS_CAN, &msg, 0);
}

/* Only BUS_ERR_TX_ABORTED closes the claim and requires reopening. */
static inline int can_frame_send_confirmed(const struct can_frame *f) {
    if (!f)
        return BUS_ERR_BAD_ARG;
    bus_msg_t msg;
    bus_msg_tx(&msg, f->data, f->dlc, f->id);
    int ret = vif_bus_send(VIF_OWNER_LINK, VIF_BUS_CAN, &msg, BUS_TX_WAIT_DONE);
    if (ret == BUS_ERR_TX_ABORTED)
        vif_bus_close(VIF_OWNER_LINK, VIF_BUS_CAN);
    return ret;
}

/** @brief Receive one frame. 0 when one arrived, negative otherwise. */
static inline int can_frame_recv(struct can_frame *f, TickType_t wait) {
    bus_msg_t msg;
    int rc;

    if (!f) {
        return BUS_ERR_BAD_ARG;
    }

    /* Straight into the caller's frame; the driver's copy is the only one. */
    memset(f, 0, sizeof(*f));
    bus_msg_init(&msg, f->data, sizeof(f->data));

    rc = vif_bus_recv(VIF_OWNER_LINK, VIF_BUS_CAN, &msg, wait);
    if (rc < 0) {
        return rc;
    }

    f->id = msg.id;
    f->dlc = (uint8_t)msg.len;

    return 0;
}
