/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_bytebus.c
 * @brief Test double for kline.c, j1850_pwm.c and j1850_vpw.c.
 */

#include "fake_bytebus.h"

#include <string.h>

#include "fake_clock.h"
#include "freertos/FreeRTOS.h"
#include "j1850_pwm.h"
#include "j1850_vpw.h"
#include "kline.h"

#define MAX_FRAMES     16
#define MAX_FRAME_LEN  32

typedef struct {
    uint8_t data[MAX_FRAME_LEN];
    size_t len;
    uint32_t delay_ms;
} staged_t;

typedef struct {
    bool up;
    int setup_count;
    int teardown_count;
    bool fail_next_send;

    staged_t live[MAX_FRAMES];
    int live_head, live_count;

    staged_t pending[MAX_FRAMES];
    int pending_count;

    staged_t sent[MAX_FRAMES];
    int sent_count;
} bus_t;

static bus_t g_bus[FAKE_BUS_COUNT];
static int g_sync_count;

void fake_bytebus_reset_all(void)
{
    memset(g_bus, 0, sizeof(g_bus));
    g_sync_count = 0;
}

/* ------------------------------------------------------------------ *
 * Test-facing accessors
 * ------------------------------------------------------------------ */

static bus_t *bus_of(fake_bus_id_t id)
{
    if (id < 0 || id >= FAKE_BUS_COUNT) {
        return &g_bus[0];
    }
    return &g_bus[id];
}

static void stage(staged_t *slot, const uint8_t *data, size_t len,
                  uint32_t delay_ms)
{
    memset(slot, 0, sizeof(*slot));

    if (len > MAX_FRAME_LEN) {
        len = MAX_FRAME_LEN;
    }
    if (data && len) {
        memcpy(slot->data, data, len);
    }

    slot->len = len;
    slot->delay_ms = delay_ms;
}

void fake_bytebus_stage_response(fake_bus_id_t id, const uint8_t *data,
                                 size_t len, uint32_t delay_ms)
{
    bus_t *b = bus_of(id);

    if (b->pending_count >= MAX_FRAMES) {
        return;
    }
    stage(&b->pending[b->pending_count++], data, len, delay_ms);
}

void fake_bytebus_stage_stale(fake_bus_id_t id, const uint8_t *data, size_t len)
{
    bus_t *b = bus_of(id);

    if (b->live_count >= MAX_FRAMES) {
        return;
    }
    stage(&b->live[(b->live_head + b->live_count) % MAX_FRAMES], data, len, 0);
    b->live_count++;
}

int fake_bytebus_setup_count(fake_bus_id_t id)    { return bus_of(id)->setup_count; }
int fake_bytebus_teardown_count(fake_bus_id_t id) { return bus_of(id)->teardown_count; }
bool fake_bytebus_is_up(fake_bus_id_t id)         { return bus_of(id)->up; }
int fake_bytebus_sent_count(fake_bus_id_t id)     { return bus_of(id)->sent_count; }
int fake_bytebus_sync_count(void)                 { return g_sync_count; }

void fake_bytebus_fail_next_send(fake_bus_id_t id)
{
    bus_of(id)->fail_next_send = true;
}

const uint8_t *fake_bytebus_sent(fake_bus_id_t id, int idx, size_t *out_len)
{
    bus_t *b = bus_of(id);

    if (idx < 0 || idx >= b->sent_count) {
        if (out_len) {
            *out_len = 0;
        }
        return NULL;
    }

    if (out_len) {
        *out_len = b->sent[idx].len;
    }
    return b->sent[idx].data;
}

/* ------------------------------------------------------------------ *
 * Shared driver behaviour
 * ------------------------------------------------------------------ */

static void bus_setup(fake_bus_id_t id)
{
    bus_t *b = bus_of(id);

    b->up = true;
    b->setup_count++;
}

static void bus_teardown(fake_bus_id_t id)
{
    bus_t *b = bus_of(id);

    b->up = false;
    b->teardown_count++;
}

static int bus_send(fake_bus_id_t id, const uint8_t *data, uint8_t len)
{
    bus_t *b = bus_of(id);

    if (!data) {
        return -1;
    }

    if (b->fail_next_send) {
        b->fail_next_send = false;
        return -1;
    }

    if (b->sent_count < MAX_FRAMES) {
        stage(&b->sent[b->sent_count++], data, len, 0);
    }

    /* The ECU's answer becomes readable now that the request has gone out. */
    for (int i = 0; i < b->pending_count && b->live_count < MAX_FRAMES; i++) {
        b->live[(b->live_head + b->live_count) % MAX_FRAMES] = b->pending[i];
        b->live_count++;
    }
    b->pending_count = 0;

    return 0;
}

/**
 * @brief Common receive.
 *
 * Returns the frame length, or -1 when nothing is waiting. Callers treat -1 as
 * "queue drained" and anything <= 0 as "keep waiting", so the empty case has
 * to charge the requested timeout to the fake clock.
 */
static int bus_receive(fake_bus_id_t id, uint8_t *data, uint8_t len,
                       TickType_t ticks_to_wait)
{
    bus_t *b = bus_of(id);
    staged_t *s;
    size_t n;

    if (b->live_count == 0) {
        fake_clock_advance_ms((uint32_t)ticks_to_wait);
        return -1;
    }

    s = &b->live[b->live_head];

    if (s->delay_ms > (uint32_t)ticks_to_wait) {
        /* Not on the wire yet. See fake_can_bus.c for the reasoning. */
        s->delay_ms -= (uint32_t)ticks_to_wait;
        fake_clock_advance_ms((uint32_t)ticks_to_wait);
        return -1;
    }

    n = s->len < len ? s->len : len;

    if (data && n) {
        memcpy(data, s->data, n);
    }

    fake_clock_advance_ms(s->delay_ms);

    b->live_head = (b->live_head + 1) % MAX_FRAMES;
    b->live_count--;

    return (int)n;
}

/* ------------------------------------------------------------------ *
 * kline.h implementation
 * ------------------------------------------------------------------ */

void kline_setup(void)    { bus_setup(FAKE_BUS_KLINE); }
void kline_teardown(void) { bus_teardown(FAKE_BUS_KLINE); }

int kline_sync(void)
{
    g_sync_count++;
    return 0;
}

int kline_send(const uint8_t *data, uint8_t len)
{
    return bus_send(FAKE_BUS_KLINE, data, len);
}

int kline_recieve(uint8_t *data, uint8_t len, TickType_t xTicksToWait)
{
    return bus_receive(FAKE_BUS_KLINE, data, len, xTicksToWait);
}

/* ------------------------------------------------------------------ *
 * j1850_pwm.h implementation
 * ------------------------------------------------------------------ */

void j1850_pwm_setup(void)    { bus_setup(FAKE_BUS_J1850_PWM); }
void j1850_pwm_teardown(void) { bus_teardown(FAKE_BUS_J1850_PWM); }

int j1850_pwm_send(const uint8_t *data, uint8_t len)
{
    return bus_send(FAKE_BUS_J1850_PWM, data, len);
}

int j1850_pwm_receive(uint8_t *data, uint8_t len, TickType_t xTicksToWait)
{
    return bus_receive(FAKE_BUS_J1850_PWM, data, len, xTicksToWait);
}

/* ------------------------------------------------------------------ *
 * j1850_vpw.h implementation
 * ------------------------------------------------------------------ */

void j1850_vpw_setup(void)    { bus_setup(FAKE_BUS_J1850_VPW); }
void j1850_vpw_teardown(void) { bus_teardown(FAKE_BUS_J1850_VPW); }

int j1850_vpw_send(const uint8_t *data, uint8_t len)
{
    return bus_send(FAKE_BUS_J1850_VPW, data, len);
}

int j1850_vpw_receive(uint8_t *data, uint8_t len, TickType_t xTicksToWait)
{
    return bus_receive(FAKE_BUS_J1850_VPW, data, len, xTicksToWait);
}
