/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_can_bus.c
 * @brief Test double for main/can_bus.c.
 */

#include "fake_can_bus.h"
#include "esp_timer.h"
#include "bus_tx.h"

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
    uint64_t ready_ms;
    uint32_t timestamp_us;
    int release_at; /* Transmit count that frees this frame */
} staged_t;

static struct {
    bool up;
    bool listen_only; /* most recent open asked for listen-only */
    bool listen_first; /* most recent open asked to listen first */
    bool foreign_rate; /* a bus at another bit rate is running */
    uint32_t rx_timestamp_us;
    uint32_t rx_sequence;
    int baud;
    int setup_count;
    int teardown_count;
    bool fail_next_send;
    int fail_countdown; /* Fail this many sends from now; 0 for none. */
    bool fail_setup;
    void (*lifecycle_hook)(void);
    int confirmed_send_error;
    uint32_t tx_end_lag_us; /* How long before the send returns a frame ends. */

    staged_t live[MAX_FRAMES]; /* Readable now */
    int live_head, live_count;

    staged_t pending[MAX_FRAMES]; /* Waiting for their release_at transmit */
    int pending_count;

    struct can_frame sent[MAX_FRAMES];
    int sent_count;
    int sent_total; /* Includes transmits past the log's end */
} g;

static uint8_t fake_tec, fake_rec;
static uint32_t fake_frames, fake_errors;
static can_port_t fake_port;

void fake_can_reset(void) {
    memset(&g, 0, sizeof(g));
    fake_tec = fake_rec = 0;
    fake_frames = fake_errors = 0;
    fake_port = CAN_PORT_OBD;
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
    slot->ready_ms = fake_clock_ms();
    slot->timestamp_us = (uint32_t)esp_timer_get_time();
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
bool fake_can_last_listen_only(void) { return g.listen_only; }
bool fake_can_last_listen_first(void) { return g.listen_first; }
void fake_can_set_foreign_rate(bool on) { g.foreign_rate = on; }
int fake_can_sent_count(void) { return g.sent_count; }

void fake_can_fail_confirmed_send(void) {
    g.confirmed_send_error = BUS_ERR_TX_FAILED;
}

void fake_can_abort_confirmed_send(void) {
    g.confirmed_send_error = BUS_ERR_TX_ABORTED;
}

void fake_can_fail_next_send(void) { g.fail_next_send = true; }

void fake_can_fail_send_after(int sends) { g.fail_countdown = sends + 1; }

void fake_can_tx_end_lag_us(uint32_t lag_us) { g.tx_end_lag_us = lag_us; }

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
    if (g.lifecycle_hook)
        g.lifecycle_hook();

    if (g.fail_setup) {
        return ESP_FAIL;
    }

    g.up = true;
    g.baud = baud_rate;

    return ESP_OK;
}

void fake_can_fail_setup(bool fail) { g.fail_setup = fail; }

void fake_can_on_lifecycle(void (*hook)(void)) { g.lifecycle_hook = hook; }

esp_err_t can_bus_teardown(void) {
    if (g.lifecycle_hook)
        g.lifecycle_hook();
    g.up = false;
    g.teardown_count++;
    return ESP_OK;
}

void can_print_stat(void) {}

void fake_can_set_error_counters(uint8_t tec, uint8_t rec) {
    fake_tec = tec;
    fake_rec = rec;
}
esp_err_t can_bus_set_port(can_port_t port) {
    if (g.up)
        return ESP_ERR_INVALID_STATE;
    fake_port = port;
    return ESP_OK;
}

can_port_t can_bus_port(void) { return fake_port; }

void can_bus_activity(uint32_t *frames, uint32_t *errors) {
    *frames = g.up ? fake_frames : 0;
    *errors = g.up ? fake_errors : 0;
}

void fake_can_set_activity(uint32_t frames, uint32_t errors) {
    fake_frames = frames;
    fake_errors = errors;
}

esp_err_t can_bus_error_counters(uint8_t *tec, uint8_t *rec) {
    if (!g.up)
        return ESP_ERR_INVALID_STATE;
    *tec = fake_tec;
    *rec = fake_rec;
    return ESP_OK;
}

int can_send(const struct can_frame *frame) {
    if (!frame) {
        return -1;
    }

    if (g.fail_next_send) {
        g.fail_next_send = false;
        return -1;
    }
    if (g.fail_countdown && --g.fail_countdown == 0)
        return -1;

    if (g.sent_count < MAX_FRAMES) {
        g.sent[g.sent_count++] = *frame;
    }
    g.sent_total++;

    /* Release whatever the ECU was staged to answer with by this transmit,
     * keeping the rest waiting for a later one. */
    uint64_t ready_ms = fake_clock_ms();
    if (g.live_count) {
        uint64_t tail =
            g.live[(g.live_head + g.live_count - 1) % MAX_FRAMES].ready_ms;
        if (tail > ready_ms)
            ready_ms = tail;
    }
    int kept = 0;
    for (int i = 0; i < g.pending_count; i++) {
        if (g.pending[i].release_at > g.sent_total ||
            g.live_count >= MAX_FRAMES) {
            g.pending[kept++] = g.pending[i];
            continue;
        }

        ready_ms += g.pending[i].delay_ms;
        g.pending[i].ready_ms = ready_ms;
        g.pending[i].timestamp_us =
            (uint32_t)esp_timer_get_time() +
            (uint32_t)(ready_ms - fake_clock_ms()) * 1000u;
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

    uint32_t delay_ms =
        s->ready_ms > fake_clock_ms() ? s->ready_ms - fake_clock_ms() : 0;
    if (delay_ms > (uint32_t)ticks_to_wait) {
        fake_clock_advance_ms((uint32_t)ticks_to_wait);
        return -1;
    }

    if (frame) {
        *frame = s->frame;
    }

    fake_clock_advance_ms(delay_ms);
    g.rx_timestamp_us = s->timestamp_us;

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

    g.listen_only = cfg->listen_only;
    g.listen_first = cfg->listen_first;
    /* What the driver's listen-first check concludes on such a bus. */
    if (cfg->listen_first && !cfg->listen_only && g.foreign_rate)
        return ESP_ERR_INVALID_STATE;
    return can_bus_setup((int)cfg->bitrate);
}

static esp_err_t can_ops_close(void) { return can_bus_teardown(); }

static int can_ops_send(const bus_msg_t *msg, uint32_t flags) {
    struct can_frame f = {0};

    /* Like the driver: a listen-only node has nothing to transmit with. */
    if (g.listen_only)
        return BUS_ERR_TX_FAILED;

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

static int can_ops_controlled(const bus_msg_t *msg, uint32_t flags,
                              bus_tx_control_t *control) {
    uint32_t sequence = g.rx_sequence;
    for (int i = 0; i < g.live_count; i++)
        if (g.live[(g.live_head + i) % MAX_FRAMES].ready_ms <= fake_clock_ms())
            sequence++;
    int rc = bus_tx_admit(control, sequence, (uint32_t)esp_timer_get_time());
    if (rc)
        return rc;
    rc = can_ops_send(msg, flags);
    bus_tx_end(control, (uint32_t)esp_timer_get_time() - g.tx_end_lag_us);
    return rc;
}

/* One frame after another through the same send, stopping where a frame
 * fails or the train is cancelled, as the driver's does. */
static int can_ops_train(const bus_msg_t *msgs, unsigned count,
                         bus_tx_control_t *control, bus_tx_train_t *train) {
    train->sent = 0;
    int rc =
        bus_tx_admit(control, g.rx_sequence, (uint32_t)esp_timer_get_time());
    if (rc)
        return rc;
    for (unsigned i = 0; i < count; i++) {
        if (bus_tx_stopped(control)) {
            rc = BUS_ERR_CANCELLED;
            break;
        }
        rc = can_ops_send(&msgs[i], BUS_TX_WAIT_DONE);
        if (rc)
            break;
        train->ended_us[train->sent++] = (uint32_t)esp_timer_get_time();
    }
    if (train->sent)
        bus_tx_end(control, train->ended_us[train->sent - 1]);
    return rc;
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
    msg->timestamp_us = g.rx_timestamp_us;
    msg->sequence = ++g.rx_sequence;
    msg->status = 0;

    return (int)len;
}

static bus_listener_t listener;

const bus_ops_t can_bus_ops = {
    .name = "CAN",
    .open = can_ops_open,
    .close = can_ops_close,
    .send = can_ops_send,
    .send_controlled = can_ops_controlled,
    .send_train = can_ops_train,
    .recv = can_ops_recv,
    .listener = &listener,
};
