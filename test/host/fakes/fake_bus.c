/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_bus.c
 * @brief Test double for kline.c, j1850_pwm.c and j1850_vpw.c.
 */

#include "fake_bus.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "fake_clock.h"
#include "j1850_pwm.h"
#include "j1850_vpw.h"
#include "kline.h"
#include "kline_codec.h"

#define MAX_FRAMES 16
#define MAX_FRAME_LEN 32

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
    void (*lifecycle_hook)(void *);
    void *lifecycle_arg;
    esp_err_t open_result;
    esp_err_t close_result;
    void (*receive_hook)(void *);
    void *receive_arg;

    staged_t live[MAX_FRAMES];
    int live_head, live_count;

    staged_t pending[MAX_FRAMES];
    int pending_count;

    staged_t sent[MAX_FRAMES];
    int sent_count;
} bus_t;

static bus_t g_bus[FAKE_BUS_COUNT];
static int g_sync_count;
static int g_stop_comm_count;
static int g_connect_result;
static bus_link_t g_link;
static bus_stats_t g_stats[FAKE_BUS_COUNT];
static uint32_t g_kline_baud;
static uint32_t g_kline_only;
static void (*g_kline_only_hook)(void);
static int g_kline_only_result;
static uint32_t g_wakeup_ms;
static uint32_t g_ifr_enabled;
static uint32_t g_ifr_byte;
static uint8_t g_wakeup[KLINE_WAKEUP_MAX];
static uint8_t g_wakeup_len;

void fake_bus_reset_all(void) {
    memset(g_bus, 0, sizeof(g_bus));
    memset(&g_link, 0, sizeof(g_link));
    g_sync_count = 0;
    g_stop_comm_count = 0;
    g_connect_result = 0;
    g_kline_baud = KLINE_BAUD_DEFAULT;
    g_kline_only = 1;
    g_kline_only_hook = NULL;
    g_kline_only_result = 0;
    g_wakeup_ms = 0;
    g_ifr_enabled = 0;
    g_ifr_byte = 0xF1;
    g_wakeup_len = 0;
    memset(g_stats, 0, sizeof(g_stats));
}

/* ------------------------------------------------------------------ *
 * Test-facing accessors
 * ------------------------------------------------------------------ */

static bus_t *bus_of(fake_bus_id_t id) {
    if (id < 0 || id >= FAKE_BUS_COUNT) {
        return &g_bus[0];
    }
    return &g_bus[id];
}

void fake_bus_on_receive(fake_bus_id_t id, void (*hook)(void *), void *arg) {
    bus_of(id)->receive_hook = hook;
    bus_of(id)->receive_arg = arg;
}

void fake_bus_on_lifecycle(fake_bus_id_t id, void (*hook)(void *), void *arg) {
    bus_of(id)->lifecycle_hook = hook;
    bus_of(id)->lifecycle_arg = arg;
}

static void lifecycle_hook(fake_bus_id_t id) {
    bus_t *b = bus_of(id);
    if (b->lifecycle_hook)
        b->lifecycle_hook(b->lifecycle_arg);
}

void fake_bus_open_result(fake_bus_id_t id, esp_err_t result) {
    bus_of(id)->open_result = result;
}

void fake_bus_close_result(fake_bus_id_t id, esp_err_t result) {
    bus_of(id)->close_result = result;
}

static void stage(staged_t *slot, const uint8_t *data, size_t len,
                  uint32_t delay_ms) {
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

void fake_bus_stage_response(fake_bus_id_t id, const uint8_t *data, size_t len,
                             uint32_t delay_ms) {
    bus_t *b = bus_of(id);

    if (b->pending_count >= MAX_FRAMES) {
        return;
    }
    stage(&b->pending[b->pending_count++], data, len, delay_ms);
}

void fake_bus_stage_stale(fake_bus_id_t id, const uint8_t *data, size_t len) {
    bus_t *b = bus_of(id);

    if (b->live_count >= MAX_FRAMES) {
        return;
    }
    stage(&b->live[(b->live_head + b->live_count) % MAX_FRAMES], data, len, 0);
    b->live_count++;
}

int fake_bus_setup_count(fake_bus_id_t id) { return bus_of(id)->setup_count; }
int fake_bus_teardown_count(fake_bus_id_t id) {
    return bus_of(id)->teardown_count;
}
bool fake_bus_is_up(fake_bus_id_t id) { return bus_of(id)->up; }
int fake_bus_sent_count(fake_bus_id_t id) { return bus_of(id)->sent_count; }
int fake_bus_sync_count(void) { return g_sync_count; }
int fake_bus_stop_comm_count(void) { return g_stop_comm_count; }

void fake_bus_kline_connect_result(int rc) { g_connect_result = rc; }
uint32_t fake_bus_kline_baud(void) { return g_kline_baud; }
uint32_t fake_bus_kline_wakeup_ms(void) { return g_wakeup_ms; }

void fake_bus_kline_set_variant(kline_variant_t v) {
    g_link.variant = (uint8_t)v;
}

const uint8_t *fake_bus_kline_wakeup(size_t *out_len) {
    if (out_len) {
        *out_len = g_wakeup_len;
    }
    return g_wakeup_len ? g_wakeup : NULL;
}

void fake_bus_fail_next_send(fake_bus_id_t id) {
    bus_of(id)->fail_next_send = true;
}

const uint8_t *fake_bus_sent(fake_bus_id_t id, int idx, size_t *out_len) {
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

static void bus_setup(fake_bus_id_t id) {
    bus_t *b = bus_of(id);

    b->up = true;
    b->setup_count++;
}

static void bus_teardown(fake_bus_id_t id) {
    bus_t *b = bus_of(id);

    b->up = false;
    b->teardown_count++;
}

static int bus_send(fake_bus_id_t id, const uint8_t *data, uint8_t len) {
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
                       TickType_t ticks_to_wait) {
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
 * The bus interface, three times over
 * ------------------------------------------------------------------ *
 *
 * One body per call, parameterised by bus id, and three vtables pointing at
 * thin wrappers around them. The wrappers exist because a bus_ops_t entry
 * takes no bus argument - the real drivers each own one peripheral - so the
 * fake has to bind the id at the vtable rather than at the call.
 *
 * The K-Line entries carry a little more: the initialisation IOCTLs and the
 * link they establish, because the ELM327 suites drive AT SI, AT FI and AT BI
 * through them and there is nothing else to stand in for a handshake.
 */

static esp_err_t fake_open(fake_bus_id_t id) {
    lifecycle_hook(id);
    bus_setup(id);
    return bus_of(id)->open_result;
}

static int fake_send(fake_bus_id_t id, const uint8_t *data, size_t len,
                     uint32_t flags) {
    (void)flags;

    if (len > UINT8_MAX) {
        return BUS_ERR_TOO_LONG;
    }

    g_stats[id].tx_msgs++;
    return bus_send(id, data, (uint8_t)len);
}

static int fake_recv(fake_bus_id_t id, bus_msg_t *msg, TickType_t wait) {
    bus_t *b = bus_of(id);
    if (b->receive_hook) {
        void (*hook)(void *) = b->receive_hook;
        b->receive_hook = NULL;
        hook(b->receive_arg);
    }
    int rc;

    if (!msg || !msg->data) {
        return BUS_ERR_BAD_ARG;
    }

    rc =
        bus_receive(id, msg->data,
                    msg->cap > UINT8_MAX ? UINT8_MAX : (uint8_t)msg->cap, wait);
    if (rc <= 0) {
        return rc < 0 ? BUS_ERR_TIMEOUT : BUS_ERR_TIMEOUT;
    }

    msg->len = (uint16_t)rc;
    msg->status = 0;
    msg->timestamp_us = fake_clock_ms() * 1000u;
    g_stats[id].rx_msgs++;
    return rc;
}

/**
 * @brief The parameters the fake actually remembers.
 *
 * Only the ones a test asserts on. Everything else is accepted and dropped,
 * because a fake that refuses a parameter the real driver accepts would make
 * the ELM327 layer's policy calls look like failures.
 */
void fake_bus_on_kline_only(void (*hook)(void), int result) {
    g_kline_only_hook = hook;
    g_kline_only_result = result;
}

static int fake_set_param(bus_param_t p, uint32_t value) {
    switch (p) {
    case BUS_P_IFR_ENABLED:
        g_ifr_enabled = value;
        return 0;
    case BUS_P_IFR_BYTE:
        g_ifr_byte = value;
        return 0;
    case BUS_P_K_LINE_ONLY:
        if (g_kline_only_hook) {
            g_kline_only_hook();
        }
        if (g_kline_only_result != 0) {
            return g_kline_only_result;
        }
        g_kline_only = value;
        return 0;
    case BUS_P_DATA_RATE:
        g_kline_baud = value;
        return 0;
    case BUS_P_PERIODIC_MS:
        g_wakeup_ms = value;
        return 0;
    default:
        return 0;
    }
}

static int fake_get_param(bus_param_t p, uint32_t *out) {
    if (!out) {
        return BUS_ERR_BAD_ARG;
    }

    switch (p) {
    case BUS_P_IFR_ENABLED:
        *out = g_ifr_enabled;
        return 0;
    case BUS_P_IFR_BYTE:
        *out = g_ifr_byte;
        return 0;
    case BUS_P_K_LINE_ONLY:
        *out = g_kline_only;
        return 0;
    case BUS_P_DATA_RATE:
        *out = g_kline_baud;
        return 0;
    case BUS_P_PERIODIC_MS:
        *out = g_wakeup_ms;
        return 0;
    default:
        return BUS_ERR_UNSUPPORTED;
    }
}

static int fake_kline_ioctl(bus_ioctl_t id, const void *in, void *out) {
    switch (id) {
    case BUS_IOCTL_FIVE_BAUD_INIT:
    case BUS_IOCTL_FAST_INIT: {
        const bus_init_t *req = in;

        g_sync_count++;

        if (g_connect_result != 0) {
            g_link.connected = false;
            return g_connect_result;
        }

        g_link.connected = true;
        g_link.baud = g_kline_baud;
        g_link.address =
            req && req->address ? req->address : KLINE_INIT_ADDR_OBD;

        /* A variant staged with fake_bus_kline_set_variant() stands;
         * otherwise a slow init lands on ISO 9141-2 and a fast one on KWP,
         * which is what the two handshakes can actually produce. */
        if (g_link.variant == KLINE_VARIANT_UNKNOWN) {
            g_link.variant = (id == BUS_IOCTL_FAST_INIT)
                                 ? KLINE_VARIANT_KWP
                                 : KLINE_VARIANT_ISO9141;
        }

        /* Key bytes that decode to the variant, so a front-end that checks
         * them - AT KW1 does - sees what the handshake claims to have found. */
        if (g_link.variant == KLINE_VARIANT_KWP) {
            g_link.key[0] = 0xE9;
            g_link.key[1] = 0x8F;
        } else {
            g_link.key[0] = 0x08;
            g_link.key[1] = 0x08;
        }

        if (out) {
            bus_init_t *rsp = out;

            if (req) {
                *rsp = *req;
            }
            rsp->key[0] = g_link.key[0];
            rsp->key[1] = g_link.key[1];
        }
        return 0;
    }

    case BUS_IOCTL_STOP_COMM:
        g_stop_comm_count++;
        g_link.connected = false;
        return 0;

    case BUS_IOCTL_GET_LINK:
        if (!out) {
            return BUS_ERR_BAD_ARG;
        }
        *(bus_link_t *)out = g_link;
        return 0;

    case BUS_IOCTL_ASSUME_LINK:
        g_link.connected = true;
        if (in) {
            g_link.variant = (uint8_t)*(const kline_variant_t *)in;
        }
        return 0;

    case BUS_IOCTL_SET_PERIODIC: {
        const bus_msg_t *m = in;

        g_wakeup_len = 0;
        if (m && m->len && m->len <= BUS_PERIODIC_MAX) {
            memcpy(g_wakeup, m->data, m->len);
            g_wakeup_len = (uint8_t)m->len;
        }
        return 0;
    }

    case BUS_IOCTL_CLEAR_PERIODIC:
        g_wakeup_ms = 0;
        return 0;

    default:
        return BUS_ERR_UNSUPPORTED;
    }
}

static void fake_get_stats(fake_bus_id_t id, bus_stats_t *out) {
    if (out) {
        *out = g_stats[id];
    }
}

static void fake_reset_stats(fake_bus_id_t id) {
    memset(&g_stats[id], 0, sizeof(g_stats[id]));
}

#define FAKE_BUS_VTABLE(prefix, id, label)                                     \
    static esp_err_t prefix##_open(const bus_cfg_t *cfg) {                     \
        (void)cfg;                                                             \
        return fake_open(id);                                                  \
    }                                                                          \
    static esp_err_t prefix##_close(void) {                                    \
        lifecycle_hook(id);                                                    \
        esp_err_t err = bus_of(id)->close_result;                              \
        if (err == ESP_OK) {                                                   \
            bus_teardown(id);                                                  \
        }                                                                      \
        return err;                                                            \
    }                                                                          \
    static int prefix##_send(const bus_msg_t *m, uint32_t f) {                 \
        return fake_send(id, m->data, m->len, f);                              \
    }                                                                          \
    static int prefix##_recv(bus_msg_t *m, TickType_t w) {                     \
        return fake_recv(id, m, w);                                            \
    }                                                                          \
    static void prefix##_stats(bus_stats_t *o) { fake_get_stats(id, o); }      \
    static void prefix##_clear(void) { fake_reset_stats(id); }

FAKE_BUS_VTABLE(fk_kline, FAKE_BUS_KLINE, "K-Line")
FAKE_BUS_VTABLE(fk_pwm, FAKE_BUS_J1850_PWM, "J1850 PWM")
FAKE_BUS_VTABLE(fk_vpw, FAKE_BUS_J1850_VPW, "J1850 VPW")

const bus_ops_t kline_bus_ops = {
    .name = "K-Line",
    .open = fk_kline_open,
    .close = fk_kline_close,
    .send = fk_kline_send,
    .recv = fk_kline_recv,
    .set_param = fake_set_param,
    .get_param = fake_get_param,
    .ioctl = fake_kline_ioctl,
    .get_stats = fk_kline_stats,
    .reset_stats = fk_kline_clear,
};

const bus_ops_t j1850_pwm_bus_ops = {
    .name = "J1850 PWM",
    .open = fk_pwm_open,
    .close = fk_pwm_close,
    .send = fk_pwm_send,
    .recv = fk_pwm_recv,
    .set_param = fake_set_param,
    .get_param = fake_get_param,
    .ioctl = NULL,
    .get_stats = fk_pwm_stats,
    .reset_stats = fk_pwm_clear,
};

const bus_ops_t j1850_vpw_bus_ops = {
    .name = "J1850 VPW",
    .open = fk_vpw_open,
    .close = fk_vpw_close,
    .send = fk_vpw_send,
    .recv = fk_vpw_recv,
    .set_param = fake_set_param,
    .get_param = fake_get_param,
    .ioctl = NULL,
    .get_stats = fk_vpw_stats,
    .reset_stats = fk_vpw_clear,
};
