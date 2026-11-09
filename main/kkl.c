/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file kkl.c
 * @brief The KKL cable front-end. See kkl.h.
 *
 * The USB stack reports line coding and break from its own task, and only the
 * link's task may touch the bus it holds, so both are queued and applied on
 * the link's side.
 *
 * They are queued with a position in the data port's byte stream, because a
 * setting reaches the device over the control pipe and the data over the bulk
 * pipe, and either may be ahead of the other by the time the link reads them.
 * A tool that writes a 5 baud address word and then sets 9600 baud must have
 * its byte go out at 5 baud, however promptly the 9600 arrived. So a chunk of
 * data is cut at every event that came after part of it, and an event is held
 * until every byte that preceded it has been sent.
 */

#include "kkl.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "bus.h"
#include "kline_codec.h"

#define TAG "KKL"

#define KKL_EVENTS 16
#define KKL_BREAK_HOLD 0xFFFF

typedef struct {
    bool is_break;
    uint16_t break_ms;
    uint32_t baud;
    uint8_t data_bits;
    uint8_t parity;
    comm_port_id_t port; /**< Where it came from. */
    uint32_t at;         /**< That port's stream position when it came. */
} kkl_event_t;

static SemaphoreHandle_t lock;
static kkl_event_t queue[KKL_EVENTS];
static size_t q_head, q_count;

/* The last line coding a host set, for a link that becomes KKL afterwards. */
static kkl_event_t line = {
    .baud = KLINE_BAUD_DEFAULT,
    .data_bits = 8,
    .parity = KLINE_PARITY_NONE,
};

/* ------------------------------------------------------------------ *
 * Applying settings to the bus
 * ------------------------------------------------------------------ */

static void set_param(bus_param_t p, uint32_t v, const char *what) {
    int rc = vif_bus_param_set(VIF_OWNER_LINK, VIF_BUS_KLINE, p, v);

    if (rc) {
        ESP_LOGW(TAG, "%s %u refused (%d)", what, (unsigned)v, rc);
    }
}

static void set_break(bool on) {
    int rc = vif_bus_ioctl(VIF_OWNER_LINK, VIF_BUS_KLINE, BUS_IOCTL_KLINE_BREAK,
                           &on, NULL);

    if (rc) {
        ESP_LOGW(TAG, "break %s refused (%d)", on ? "on" : "off", rc);
    }
}

static void apply_line(const kkl_event_t *ev) {
    /* ponytail: 5 and 6 data bits go out as 8; no KKL tool uses them. */
    set_param(BUS_P_DATA_BITS,
              ev->data_bits == 7 ? BUS_DATA_BITS_7 : BUS_DATA_BITS_8,
              "data bits");
    set_param(BUS_P_PARITY, ev->parity, "parity");
    set_param(BUS_P_DATA_RATE, ev->baud, "baud");
}

static void apply(const kkl_event_t *ev) {
    if (!ev->is_break) {
        apply_line(ev);
        return;
    }

    if (ev->break_ms == 0 || ev->break_ms == KKL_BREAK_HOLD) {
        set_break(ev->break_ms != 0);
        return;
    }

    /* A timed break holds the link for its length, as it holds the wire. */
    set_break(true);
    vTaskDelay(pdMS_TO_TICKS(ev->break_ms));
    set_break(false);
}

static void send(const uint8_t *data, size_t len) {
    bus_msg_t msg;
    int rc;

    if (len == 0) {
        return;
    }

    bus_msg_tx(&msg, data, len, 0);
    rc = vif_bus_send(VIF_OWNER_LINK, VIF_BUS_KLINE, &msg, BUS_TX_NO_CHECKSUM);
    if (rc) {
        /* A cable has no way to say so either; the host sees no echo. */
        ESP_LOGD(TAG, "%u bytes not sent (%d)", (unsigned)len, rc);
    }
}

/* ------------------------------------------------------------------ *
 * The event queue
 * ------------------------------------------------------------------ */

/**
 * @brief Take the oldest event if it is due before stream position @p pos of
 *        @p port. An event from another port has no place in this stream and
 *        is due at once.
 */
static bool take_due(comm_port_id_t port, uint32_t pos, kkl_event_t *out) {
    bool due = false;

    xSemaphoreTake(lock, portMAX_DELAY);
    if (q_count) {
        const kkl_event_t *ev = &queue[q_head];

        due = ev->port != port || (int32_t)(ev->at - pos) <= 0;
        if (due) {
            *out = *ev;
            q_head = (q_head + 1) % KKL_EVENTS;
            q_count--;
        }
    }
    xSemaphoreGive(lock);

    return due;
}

/** @brief Where in the chunk at @p pos the oldest event falls, if inside it. */
static bool next_cut(comm_port_id_t port, uint32_t pos, size_t len,
                     size_t *cut) {
    bool inside = false;

    xSemaphoreTake(lock, portMAX_DELAY);
    if (q_count && queue[q_head].port == port) {
        uint32_t off = queue[q_head].at - pos;

        inside = off < len;
        *cut = off;
    }
    xSemaphoreGive(lock);

    return inside;
}

static void apply_due(comm_port_id_t port, uint32_t pos) {
    kkl_event_t ev;

    while (take_due(port, pos, &ev)) {
        apply(&ev);
    }
}

static void post(kkl_event_t *ev) {
    if (!lock) {
        return; /* before kkl_register(): nothing to apply it to yet */
    }

    ev->at = comm_port_rx_position(ev->port);

    xSemaphoreTake(lock, portMAX_DELAY);
    if (!ev->is_break) {
        line = *ev;
    }
    if (q_count < KKL_EVENTS) {
        queue[(q_head + q_count) % KKL_EVENTS] = *ev;
        q_count++;
        ev = NULL;
    }
    xSemaphoreGive(lock);

    if (ev) {
        ESP_LOGW(TAG, "line event dropped");
    }
}

void kkl_line_coding(comm_port_id_t port, uint32_t baud, uint8_t data_bits,
                     uint8_t parity) {
    kkl_event_t ev = {
        .baud = baud,
        .data_bits = data_bits,
        .parity = parity,
        .port = port,
    };

    post(&ev);
}

void kkl_break(comm_port_id_t port, uint16_t duration_ms) {
    kkl_event_t ev = {.is_break = true, .break_ms = duration_ms, .port = port};

    post(&ev);
}

/* ------------------------------------------------------------------ *
 * The front-end
 * ------------------------------------------------------------------ */

static bool kkl_start(void) {
    kkl_event_t now;

    if (vif_bus_open(VIF_OWNER_LINK, VIF_BUS_KLINE, NULL) != ESP_OK) {
        ESP_LOGE(TAG, "K-Line is held elsewhere");
        return false;
    }
    if (vif_bus_param_set(VIF_OWNER_LINK, VIF_BUS_KLINE, BUS_P_KLINE_RAW, 1)) {
        /* The grammar vif falls back to must not inherit a half set up bus. */
        vif_bus_close(VIF_OWNER_LINK, VIF_BUS_KLINE);
        ESP_LOGE(TAG, "K-Line refused raw mode");
        return false;
    }

    /* Whatever was queued is history; the latest coding is all that counts,
     * and a break from before this grammar ran was never meant for it. */
    xSemaphoreTake(lock, portMAX_DELAY);
    q_head = q_count = 0;
    now = line;
    xSemaphoreGive(lock);
    apply_line(&now);

    ESP_LOGI(TAG, "KKL cable on pin 7 at %u baud", (unsigned)now.baud);
    return true;
}

static void kkl_feed(const uint8_t *data, size_t len) {
    comm_port_id_t port = vif_link_port();
    /* The link task has just read these, so they end at the read position. */
    uint32_t pos = comm_port_read_position(port) - (uint32_t)len;
    size_t cut;

    for (;;) {
        apply_due(port, pos);
        if (!next_cut(port, pos, len, &cut)) {
            break;
        }
        send(data, cut);
        data += cut;
        len -= cut;
        pos += (uint32_t)cut;
    }

    send(data, len);
}

static void kkl_poll(void) {
    comm_port_id_t port = vif_link_port();
    uint8_t buf[KLINE_MAX_MSG];
    bool wrote = false;

    /* Everything read so far has been sent, so anything that came after it
     * is due. Anything after bytes still unread waits for them. */
    apply_due(port, comm_port_read_position(port));

    for (;;) {
        bus_msg_t msg;

        bus_msg_init(&msg, buf, sizeof(buf));
        if (vif_bus_recv(VIF_OWNER_LINK, VIF_BUS_KLINE, &msg, 0) < 0) {
            break;
        }
        /* Zero is a loss report with nothing in it; keep draining. */
        if (msg.len) {
            vif_link_write(buf, msg.len);
            wrote = true;
        }
    }

    if (wrote) {
        vif_link_flush();
    }
}

const vif_frontend_t kkl_frontend = {
    .name = "kkl",
    .start = kkl_start,
    .feed = kkl_feed,
    .poll = kkl_poll,
    /* No stop: vif closes the bus, and nothing else is ours to undo. Polled at
     * vif's default streaming cadence, as SLCAN is: well inside the time a
     * KW1281 ECU allows for each complement. */
};

void kkl_register(void) {
    if (!lock) {
        lock = xSemaphoreCreateMutex();
    }
    if (!lock || vif_frontend_register(&kkl_frontend) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register the KKL front-end");
    }
}
