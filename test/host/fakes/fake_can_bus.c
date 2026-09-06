/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_can_bus.c
 * @brief Test double for main/can_bus.c.
 */

#include "fake_can_bus.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "fake_clock.h"

/* Large enough to stage a full multi-frame transfer: a 400 byte ISO-TP
 * reply is 58 frames, and long replies are exactly what the tx path
 * regressions are about. */
#define MAX_FRAMES 128

typedef struct {
    struct can_frame frame;
    uint32_t delay_ms;
    int release_at; /* Transmit count that frees this frame */
} staged_t;

static struct {
    bool up;
    int baud;
    int setup_count;
    int teardown_count;
    bool fail_next_send;
    bool fail_setup;
    int confirmed_send_error;

    staged_t live[MAX_FRAMES]; /* Readable now */
    int live_head, live_count;

    staged_t pending[MAX_FRAMES]; /* Waiting for their release_at transmit */
    int pending_count;

    struct can_frame sent[MAX_FRAMES];
    int sent_count;
    int sent_total; /* Includes transmits past the log's end */
} g;

void fake_can_reset(void) {
    memset(&g, 0, sizeof(g));
    g.baud = -1;
}

/* ------------------------------------------------------------------ *
 * Test-facing accessors
 * ------------------------------------------------------------------ */

static void stage(staged_t *slot, uint32_t id, uint8_t dlc, const uint8_t *data,
                  uint32_t delay_ms) {
    memset(slot, 0, sizeof(*slot));

    slot->frame.id = id;
    slot->frame.dlc = dlc;
    if (data && dlc) {
        memcpy(slot->frame.data, data, dlc > 8 ? 8 : dlc);
    }
    slot->delay_ms = delay_ms;
}

void fake_can_stage_response_at(uint32_t id, uint8_t dlc, const uint8_t *data,
                                uint32_t delay_ms, int after_sends) {
    staged_t *slot;

    if (g.pending_count >= MAX_FRAMES) {
        return;
    }

    slot = &g.pending[g.pending_count++];
    stage(slot, id, dlc, data, delay_ms);
    slot->release_at = g.sent_total + (after_sends < 1 ? 1 : after_sends);
}

void fake_can_stage_response(uint32_t id, uint8_t dlc, const uint8_t *data,
                             uint32_t delay_ms) {
    fake_can_stage_response_at(id, dlc, data, delay_ms, 1);
}

void fake_can_stage_stale(uint32_t id, uint8_t dlc, const uint8_t *data) {
    int slot;

    if (g.live_count >= MAX_FRAMES) {
        return;
    }

    slot = (g.live_head + g.live_count) % MAX_FRAMES;
    stage(&g.live[slot], id, dlc, data, 0);
    g.live_count++;
}

int fake_can_setup_count(void) { return g.setup_count; }
int fake_can_teardown_count(void) { return g.teardown_count; }
int fake_can_last_baud(void) { return g.baud; }
bool fake_can_is_up(void) { return g.up; }
int fake_can_sent_count(void) { return g.sent_count; }

void fake_can_fail_confirmed_send(void) {
    g.confirmed_send_error = BUS_ERR_TX_FAILED;
}

void fake_can_abort_confirmed_send(void) {
    g.confirmed_send_error = BUS_ERR_TX_ABORTED;
}

void fake_can_fail_next_send(void) { g.fail_next_send = true; }

const struct can_frame *fake_can_sent(int idx) {
    if (idx < 0 || idx >= g.sent_count) {
        return NULL;
    }
    return &g.sent[idx];
}

/* ------------------------------------------------------------------ *
 * can_bus.h implementation
 * ------------------------------------------------------------------ */

esp_err_t can_bus_setup(int baud_rate) {
    g.setup_count++;

    if (g.fail_setup) {
        return ESP_FAIL;
    }

    g.up = true;
    g.baud = baud_rate;

    return ESP_OK;
}

void fake_can_fail_setup(bool fail) { g.fail_setup = fail; }

esp_err_t can_bus_teardown(void) {
    g.up = false;
    g.teardown_count++;
    return ESP_OK;
}

void can_print_stat(void) {}

int can_send(const struct can_frame *frame) {
    if (!frame) {
        return -1;
    }

    if (g.fail_next_send) {
        g.fail_next_send = false;
        return -1;
    }

    if (g.sent_count < MAX_FRAMES) {
        g.sent[g.sent_count++] = *frame;
    }
    g.sent_total++;

    /* Release whatever the ECU was staged to answer with by this transmit,
     * keeping the rest waiting for a later one. */
    int kept = 0;
    for (int i = 0; i < g.pending_count; i++) {
        if (g.pending[i].release_at > g.sent_total ||
            g.live_count >= MAX_FRAMES) {
            g.pending[kept++] = g.pending[i];
            continue;
        }

        g.live[(g.live_head + g.live_count) % MAX_FRAMES] = g.pending[i];
        g.live_count++;
    }
    g.pending_count = kept;

    return 0;
}

int can_receive(struct can_frame *frame, TickType_t ticks_to_wait) {
    staged_t *s;

    if (g.live_count == 0) {
        /* Nothing to hand over: charge the caller for the wait it asked for,
         * so its timeout loop terminates. */
        fake_clock_advance_ms((uint32_t)ticks_to_wait);
        return -1;
    }

    s = &g.live[g.live_head];

    if (s->delay_ms > (uint32_t)ticks_to_wait) {
        /* The ECU has not answered yet. The caller waited as long as it asked
         * to and times out, exactly as the real driver would; the frame stays
         * queued with the rest of its delay still to run. */
        s->delay_ms -= (uint32_t)ticks_to_wait;
        fake_clock_advance_ms((uint32_t)ticks_to_wait);
        return -1;
    }

    if (frame) {
        *frame = s->frame;
    }

    fake_clock_advance_ms(s->delay_ms);

    g.live_head = (g.live_head + 1) % MAX_FRAMES;
    g.live_count--;

    return 0;
}

/* ------------------------------------------------------------------ *
 * The bus interface
 *
 * The same shape as can_bus.c's, over the same marshalling, so vif.c reaches
 * the fake exactly as it reaches the driver.
 * ------------------------------------------------------------------ */

static esp_err_t can_ops_open(const bus_cfg_t *cfg) {
    if (!cfg || cfg->bitrate == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    return can_bus_setup((int)cfg->bitrate);
}

static esp_err_t can_ops_close(void) { return can_bus_teardown(); }

static int can_ops_send(const bus_msg_t *msg, uint32_t flags) {
    struct can_frame f = {0};

    if ((flags & BUS_TX_WAIT_DONE) && g.confirmed_send_error) {
        int error = g.confirmed_send_error;
        g.confirmed_send_error = 0;
        return error;
    }

    f.id = msg->id;
    f.dlc = (uint8_t)msg->len;

    if (!(f.id & CAN_RTR_FLAG)) {
        memcpy(f.data, msg->data, msg->len > 8 ? 8 : msg->len);
    }

    return can_send(&f) == 0 ? 0 : BUS_ERR_TX_FAILED;
}

static int can_ops_recv(bus_msg_t *msg, TickType_t wait) {
    struct can_frame f;
    size_t len;

    if (can_receive(&f, wait) != 0) {
        return BUS_ERR_TIMEOUT;
    }

    len = (f.dlc > 8) ? 8 : f.dlc;
    if (f.id & CAN_RTR_FLAG) {
        len = 0;
    }
    if (msg->cap < len) {
        return BUS_ERR_NO_SPACE;
    }

    memcpy(msg->data, f.data, len);
    msg->id = f.id;
    msg->len = f.dlc;

    return (int)len;
}

const bus_ops_t can_bus_ops = {
    .name = "CAN",
    .open = can_ops_open,
    .close = can_ops_close,
    .send = can_ops_send,
    .recv = can_ops_recv,
};
