/* SPDX-License-Identifier: GPL-3.0-only */
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "comm_iface.h"
#include "slcan.h"
#include "utility.h"
#include "vif.h"

#define TAG "SLCAN"

/** Longest command is an extended frame: 'T' + 8 id + 1 dlc + 16 data + CR. */
#define SLCAN_CMD_MAX 32

/** Frames forwarded per poll(). Enough to empty the driver's receive queue in
 *  one pass, bounded so that input still gets a look in. */
#define SLCAN_RX_BURST 32

static const char hexval[] = "0123456789ABCDEF";

typedef struct {
    bool live;
    vif_session_t *session;
    comm_port_id_t port;

    uint32_t bitrate;

    char cmdbuf[SLCAN_CMD_MAX];
    int cmdidx;
    bool discard; /* dropping the tail of an over-long command */
} slcan_ctx_t;

/**
 * @brief The instance pool.
 *
 * One per data link, like the ELM327 interpreter: a client that switches its
 * own link to SLCAN must not disturb another client already speaking it.
 */
#define SLCAN_MAX_INSTANCES 2

static slcan_ctx_t g_slcan[SLCAN_MAX_INSTANCES];

/* ------------------------------------------------------------------ *
 * Output
 * ------------------------------------------------------------------ */

static void slcan_write(slcan_ctx_t *c, const char *s, size_t len)
{
    comm_port_write(c->port, s, len);
}

static void slcan_ack(slcan_ctx_t *c)
{
    slcan_write(c, "\r", 1);
}

static void slcan_nack(slcan_ctx_t *c)
{
    slcan_write(c, "\a", 1);
}

static void slcan_reply_str(slcan_ctx_t *c, const char *s)
{
    slcan_write(c, s, strlen(s));
}

/* ------------------------------------------------------------------ *
 * Commands
 * ------------------------------------------------------------------ */

/**
 * @brief Is the channel open?
 *
 * Asked of the session rather than tracked here. The claim is the truth: it
 * outlives this front-end, so a channel opened before a protocol switch is
 * still open after it, and a cached flag would only be a second answer waiting
 * to disagree with the first.
 */
static bool slcan_is_open(const slcan_ctx_t *c)
{
    return vif_bus_current(c->session) == VIF_BUS_CAN;
}

/**
 * @brief LAWICEL bit rate digit to bits per second.
 *
 * The four rates the TWAI timing tables here do not carry - 10k, 20k, 100k and
 * 800k - return 0 and are refused, rather than quietly running at some other
 * speed on a live vehicle bus.
 */
static uint32_t slcan_bitrate(char digit)
{
    switch (digit) {
    case '2': return 50000;
    case '4': return 125000;
    case '5': return 250000;
    case '6': return 500000;
    case '8': return 1000000;
    default:  return 0;
    }
}

/** @brief Parse a t/T/r/R command into @p f. 0 on success. */
static int slcan_parse_frame(const char *buf, bool rtr, bool ext, struct can_frame *f)
{
    const int id_len = ext ? 8 : 3;
    size_t len = strlen(buf);
    uint32_t id;
    int dlc;

    if (len < (size_t)id_len + 2) {
        return -1;
    }
    if (hex_string_to_u32_be(&buf[1], id_len, &id)) {
        return -1;
    }

    dlc = hex_char_to_int(buf[1 + id_len]);
    if (dlc < 0 || dlc > 8) {
        return -1;
    }

    memset(f, 0, sizeof(*f));
    f->id = ext ? ((id & CAN_EFF_MASK) | CAN_EFF_FLAG) : (id & CAN_SFF_MASK);
    if (rtr) {
        f->id |= CAN_RTR_FLAG;
    }
    f->dlc = (uint8_t)dlc;

    /* A remote request asks for data rather than carrying any. */
    if (!rtr) {
        const char *payload = &buf[2 + id_len];

        if (strlen(payload) < (size_t)dlc * 2) {
            return -1;
        }
        if (hex_string_to_u8_array(payload, (size_t)dlc * 2, f->data,
                                   sizeof(f->data)) != dlc) {
            return -1;
        }
    }

    return 0;
}

static void slcan_send_frame(slcan_ctx_t *c, const char *buf, bool rtr, bool ext)
{
    struct can_frame frame;

    if (!slcan_is_open(c)) {
        slcan_nack(c);
        return;
    }

    if (slcan_parse_frame(buf, rtr, ext, &frame)) {
        slcan_nack(c);
        return;
    }

    if (vif_can_send(c->session, &frame) != 0) {
        ESP_LOGE(TAG, "transmit failed");
        slcan_nack(c);
        return;
    }

    slcan_ack(c);
}

static void slcan_open_bus(slcan_ctx_t *c)
{
    vif_bus_cfg_t cfg = { .bitrate = c->bitrate };

    if (slcan_is_open(c)) {
        slcan_ack(c);
        return;
    }

    if (vif_bus_open(c->session, VIF_BUS_CAN, &cfg) != ESP_OK) {
        /* Either the bit rate is unusable or another session holds the bus. */
        slcan_nack(c);
        return;
    }

    slcan_ack(c);
}

static void slcan_close_bus(slcan_ctx_t *c)
{
    if (slcan_is_open(c)) {
        vif_bus_close(c->session);
    }

    slcan_ack(c);
}

static void slcan_parse_command(slcan_ctx_t *c, const char *buf)
{
    switch (buf[0]) {
    case 'O': /* open the channel */
        slcan_open_bus(c);
        break;
    case 'C': /* close the channel */
        slcan_close_bus(c);
        break;
    case 't': /* standard frame */
        slcan_send_frame(c, buf, false, false);
        break;
    case 'T': /* extended frame */
        slcan_send_frame(c, buf, false, true);
        break;
    case 'r': /* standard remote request */
        slcan_send_frame(c, buf, true, false);
        break;
    case 'R': /* extended remote request */
        slcan_send_frame(c, buf, true, true);
        break;
    case 'S': { /* bit rate */
        uint32_t rate = slcan_bitrate(buf[1]);

        if (rate == 0) {
            /* A rate the timing tables do not carry is refused rather than
             * silently substituted. */
            slcan_nack(c);
            break;
        }

        c->bitrate = rate;

        /*
         * A channel this session already holds is reconfigured rather than
         * refused. That case is normal now: switching to this front-end from
         * ELM327 inherits a CAN claim opened at whatever protocol number the
         * previous grammar selected, and slcand's opening S<n> has to be able
         * to correct it. vif_bus_open() takes the old driver down and brings
         * the new one up, and the claim never leaves this session.
         */
        if (slcan_is_open(c)) {
            vif_bus_cfg_t cfg = { .bitrate = rate };

            if (vif_bus_open(c->session, VIF_BUS_CAN, &cfg) != ESP_OK) {
                slcan_nack(c);
                break;
            }
        }

        ESP_LOGI(TAG, "bit rate %" PRIu32, rate);
        slcan_ack(c);
        break;
    }
    case 'M': /* acceptance code, no filtering implemented */
    case 'm': /* acceptance mask */
        slcan_ack(c);
        break;
    case 'F': /* status flags */
        slcan_reply_str(c, "F00");
        slcan_ack(c);
        break;
    case 'V': /* version */
        slcan_reply_str(c, "V-082021");
        slcan_ack(c);
        break;
    case 'N': /* serial number */
        slcan_reply_str(c, "N2208");
        slcan_ack(c);
        break;
    default:
        slcan_nack(c);
        break;
    }
}

/* ------------------------------------------------------------------ *
 * Received frames
 * ------------------------------------------------------------------ */

static void slcan_print_frame(slcan_ctx_t *c, const struct can_frame *f)
{
    char buf[28];
    int pos = 0;

    if (f->id & CAN_EFF_FLAG) {
        uint32_t id = f->id & CAN_EFF_MASK;

        buf[pos++] = (f->id & CAN_RTR_FLAG) ? 'R' : 'T';
        for (int shift = 28; shift >= 0; shift -= 4) {
            buf[pos++] = hexval[(id >> shift) & 0xf];
        }
    }
    else {
        uint32_t id = f->id & CAN_SFF_MASK;

        buf[pos++] = (f->id & CAN_RTR_FLAG) ? 'r' : 't';
        for (int shift = 8; shift >= 0; shift -= 4) {
            buf[pos++] = hexval[(id >> shift) & 0xf];
        }
    }

    buf[pos++] = hexval[f->dlc & 0xf];

    if (!(f->id & CAN_RTR_FLAG)) {
        for (int i = 0; i < f->dlc && i < 8; i++) {
            buf[pos++] = hexval[f->data[i] >> 4];
            buf[pos++] = hexval[f->data[i] & 0xf];
        }
    }

    buf[pos++] = '\r';

    comm_port_write(c->port, buf, pos);
}

/* ------------------------------------------------------------------ *
 * Front-end
 * ------------------------------------------------------------------ */

/**
 * @brief Claim a free instance from the pool.
 *
 * A session already speaking SLCAN keeps the instance it has, so re-installing
 * the front-end that is already running is not an error.
 */
static slcan_ctx_t *slcan_instance_claim(vif_session_t *s)
{
    for (int i = 0; i < SLCAN_MAX_INSTANCES; i++) {
        if (g_slcan[i].live && g_slcan[i].session == s) {
            return &g_slcan[i];
        }
    }

    for (int i = 0; i < SLCAN_MAX_INSTANCES; i++) {
        if (!g_slcan[i].live) {
            return &g_slcan[i];
        }
    }

    return NULL;
}

static void *slcan_fe_create(vif_session_t *s, comm_port_id_t port)
{
    slcan_ctx_t *c = slcan_instance_claim(s);

    if (!c) {
        ESP_LOGE(TAG, "no free SLCAN instance");
        return NULL;
    }

    memset(c, 0, sizeof(*c));
    c->live = true;
    c->session = s;
    c->port = port;
    c->bitrate = 500000;

    return c;
}

static void slcan_fe_feed(void *ctx, const uint8_t *data, size_t len)
{
    slcan_ctx_t *c = ctx;

    for (size_t i = 0; i < len; i++) {
        char val = (char)data[i];

        if (val == '\r') {
            if (c->discard) {
                c->discard = false;
                slcan_nack(c);
            }
            else if (c->cmdidx > 0) {
                c->cmdbuf[c->cmdidx] = '\0';
                slcan_parse_command(c, c->cmdbuf);
            }
            c->cmdidx = 0;
            comm_port_flush(c->port);
            continue;
        }

        if (c->discard) {
            continue;
        }

        c->cmdbuf[c->cmdidx++] = val;
        if (c->cmdidx == SLCAN_CMD_MAX - 1) {
            /* Answer once, when the line finally ends. */
            c->discard = true;
            c->cmdidx = 0;
        }
    }
}

static void slcan_fe_poll(void *ctx)
{
    slcan_ctx_t *c = ctx;
    struct can_frame frame;
    int forwarded = 0;

    if (!slcan_is_open(c)) {
        return;
    }

    while (forwarded < SLCAN_RX_BURST &&
           vif_can_recv(c->session, &frame, 0) == 0) {
        slcan_print_frame(c, &frame);
        forwarded++;
    }

    if (forwarded) {
        comm_port_flush(c->port);
    }
}

static void slcan_fe_destroy(void *ctx)
{
    slcan_ctx_t *c = ctx;

    /* The CAN claim belongs to the session and outlives the front-end, so a
     * switch to another protocol does not interrupt a transfer in progress. */
    c->live = false;
    c->session = NULL;
    c->port = COMM_INVALID_PORT_ID;
}

const vif_frontend_t slcan_frontend = {
    .name = "slcan",
    .create = slcan_fe_create,
    .feed = slcan_fe_feed,
    .poll = slcan_fe_poll,
    .destroy = slcan_fe_destroy,
};

void slcan_register(void)
{
    if (vif_frontend_register(&slcan_frontend) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register the SLCAN front-end");
    }
}
