/* SPDX-License-Identifier: GPL-3.0-only */
#include "isotp.h"
#include <limits.h>
#include <string.h>

static bool expired(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}

static bool valid_id(uint32_t id) {
    uint32_t mask = (id & CAN_EFF_FLAG) ? CAN_EFF_MASK : CAN_SFF_MASK;
    return !(id & ~(mask | CAN_EFF_FLAG));
}

static bool valid_config(const isotp_config_t *c) {
    return c && valid_id(c->tx_id) && valid_id(c->rx_id) &&
           (c->stmin <= 0x7f || (c->stmin >= 0xf1 && c->stmin <= 0xf9)) &&
           c->timeout_a_us && c->timeout_a_us <= INT32_MAX &&
           c->timeout_bs_us && c->timeout_bs_us <= INT32_MAX &&
           c->timeout_cr_us && c->timeout_cr_us <= INT32_MAX;
}

static bool matches(const isotp_config_t *c, const struct can_frame *f) {
    return f && valid_id(f->id) && f->dlc <= 8 &&
           f->dlc > (unsigned)c->extended_address &&
           !((f->id ^ c->rx_id) & (c->rx_mask | CAN_EFF_FLAG)) &&
           (!c->extended_address || f->data[0] == c->rx_address);
}

static unsigned prepare(const isotp_config_t *c, struct can_frame *f) {
    memset(f, 0, sizeof(*f));
    f->id = c->tx_id;
    memset(f->data, c->pad_value, sizeof(f->data));
    if (c->extended_address)
        f->data[0] = c->tx_address;
    return c->extended_address;
}

isotp_config_t isotp_config_default(uint32_t tx_id, uint32_t rx_id) {
    return (isotp_config_t){
        .tx_id = tx_id,
        .rx_id = rx_id,
        .rx_mask = UINT32_MAX,
        .pad = true,
        .wait_limit = 255,
        .timeout_a_us = ISOTP_DEFAULT_TIMEOUT_US,
        .timeout_bs_us = ISOTP_DEFAULT_TIMEOUT_US,
        .timeout_cr_us = ISOTP_DEFAULT_TIMEOUT_US,
    };
}

uint32_t isotp_stmin_us(uint8_t value) {
    if (value <= 0x7f)
        return value * 1000u;
    if (value >= 0xf1 && value <= 0xf9)
        return (value - 0xf0u) * 100u;
    /* ISO 15765-2:2016 9.6.5.5 mandates 127 ms for reserved values. */
    return 127000;
}

int isotp_rx_init(isotp_rx_t *rx, const isotp_config_t *config, uint8_t *buffer,
                  size_t capacity) {
    if (!rx || !valid_config(config))
        return ISOTP_ERR_ARGUMENT;
    *rx =
        (isotp_rx_t){.config = *config, .buffer = buffer, .capacity = capacity};
    return 0;
}

int isotp_rx_check_timeout(isotp_rx_t *rx, uint32_t now_us) {
    if (rx->state == ISOTP_RX_IDLE || !expired(now_us, rx->deadline_us))
        return 0;
    int rc =
        rx->state == ISOTP_RX_CF ? ISOTP_ERR_TIMEOUT_CR : ISOTP_ERR_TIMEOUT_A;
    rx->state = ISOTP_RX_IDLE;
    return rc;
}

static void segment_copy(isotp_rx_t *rx, isotp_segment_t *s,
                         const uint8_t *data, unsigned length) {
    s->data = data;
    s->length = length;
    s->offset = rx->offset;
    s->total = rx->total;
    s->padding_error = rx->padding_error;
    if (rx->buffer)
        memcpy(rx->buffer + rx->offset, data, length);
    rx->offset += length;
    s->complete = rx->offset == rx->total;
}

int isotp_rx_feed(isotp_rx_t *rx, const struct can_frame *f, uint32_t now_us,
                  isotp_segment_t *s) {
    if (!rx || !s)
        return ISOTP_ERR_ARGUMENT;
    *s = (isotp_segment_t){0};
    int rc = isotp_rx_check_timeout(rx, now_us);
    if (rc < 0)
        return rc;
    if (!matches(&rx->config, f))
        return ISOTP_IGNORED;
    unsigned a = rx->config.extended_address;
    unsigned type = f->data[a] >> 4;
    unsigned size;
    if (type == 0 || type == 1) {
        if (type == 0) {
            size = f->data[a] & 0x0f;
            if (!size || size > f->dlc - a - 1)
                return ISOTP_IGNORED;
        } else {
            if (rx->config.functional || f->dlc != 8)
                return ISOTP_IGNORED;
            size = ((f->data[a] & 0x0f) << 8) | f->data[a + 1];
            if (size <= 7 - a)
                return ISOTP_IGNORED;
        }
        s->replaced = rx->state != ISOTP_RX_IDLE;
        rx->state = ISOTP_RX_IDLE;
        rx->offset = 0;
        rx->total = size;
        rx->padding_error = f->dlc < 8;
        if (size > rx->capacity) {
            if (type == 1) {
                rx->flow_status = 2;
                rx->state = ISOTP_RX_FC_READY;
                rx->deadline_us = now_us + rx->config.timeout_a_us;
            }
            return ISOTP_ERR_OVERFLOW;
        }
        s->started = true;
        if (type == 0) {
            segment_copy(rx, s, f->data + a + 1, size);
            return ISOTP_COMPLETE;
        }
        rx->sequence = 1;
        rx->block_count = 0;
        rx->flow_status = 0;
        rx->state = ISOTP_RX_FC_READY;
        rx->deadline_us = now_us + rx->config.timeout_a_us;
        segment_copy(rx, s, f->data + a + 2, 6 - a);
        return ISOTP_FRAME;
    }
    if (type != 2 || rx->state != ISOTP_RX_CF)
        return ISOTP_IGNORED;
    unsigned remaining = rx->total - rx->offset;
    size = remaining < 7 - a ? remaining : 7 - a;
    if (f->dlc < a + 1 + size)
        return ISOTP_IGNORED;
    if ((f->data[a] & 0x0f) != rx->sequence) {
        rx->state = ISOTP_RX_IDLE;
        return ISOTP_ERR_SEQUENCE;
    }
    rx->padding_error |= f->dlc < 8;
    segment_copy(rx, s, f->data + a + 1, size);
    rx->sequence = (rx->sequence + 1) & 0x0f;
    if (s->complete) {
        rx->state = ISOTP_RX_IDLE;
        return ISOTP_COMPLETE;
    }
    rx->deadline_us = now_us + rx->config.timeout_cr_us;
    if (rx->config.block_size && ++rx->block_count == rx->config.block_size) {
        rx->block_count = 0;
        rx->state = ISOTP_RX_FC_READY;
        rx->deadline_us = now_us + rx->config.timeout_a_us;
    }
    return ISOTP_FRAME;
}

int isotp_rx_flow_control(isotp_rx_t *rx, struct can_frame *f,
                          uint32_t now_us) {
    int rc = isotp_rx_check_timeout(rx, now_us);
    if (rc < 0 || rx->state != ISOTP_RX_FC_READY)
        return rc;
    unsigned a = prepare(&rx->config, f);
    f->data[a] = 0x30 | rx->flow_status;
    f->data[a + 1] = rx->config.block_size;
    f->data[a + 2] = rx->config.stmin;
    f->dlc = rx->config.pad ? 8 : a + 3;
    rx->state = ISOTP_RX_FC_CONFIRM;
    rx->deadline_us = now_us + rx->config.timeout_a_us;
    return ISOTP_FRAME;
}

int isotp_rx_confirm(isotp_rx_t *rx, bool success, uint32_t now_us) {
    if (rx->state != ISOTP_RX_FC_CONFIRM)
        return ISOTP_ERR_ARGUMENT;
    int rc = isotp_rx_check_timeout(rx, now_us);
    if (rc < 0)
        return rc;
    if (!success || rx->flow_status == 2) {
        rx->state = ISOTP_RX_IDLE;
        return success ? 0 : ISOTP_ERR_SEND;
    }
    rx->state = ISOTP_RX_CF;
    rx->deadline_us = now_us + rx->config.timeout_cr_us;
    return 0;
}

int isotp_tx_init(isotp_tx_t *tx, const isotp_config_t *config) {
    if (!tx || !valid_config(config))
        return ISOTP_ERR_ARGUMENT;
    *tx = (isotp_tx_t){.config = *config};
    return 0;
}

int isotp_tx_start(isotp_tx_t *tx, const uint8_t *data, size_t length,
                   uint32_t now_us) {
    if (!tx || !data || !length || length > ISOTP_MAX_PAYLOAD ||
        (tx->config.functional && length > 7u - tx->config.extended_address))
        return ISOTP_ERR_ARGUMENT;
    if (tx->state != ISOTP_TX_IDLE)
        return ISOTP_ERR_BUSY;
    tx->buffer = data;
    tx->total = length;
    tx->offset = 0;
    tx->sequence = 1;
    tx->wait_count = 0;
    tx->block_count = 0;
    tx->sent_cf = false;
    tx->reserved_stmin = false;
    tx->padding_error = false;
    tx->state = ISOTP_TX_READY;
    tx->deadline_us = now_us + tx->config.timeout_a_us;
    return 0;
}

int isotp_tx_check_timeout(isotp_tx_t *tx, uint32_t now_us) {
    if (tx->state == ISOTP_TX_IDLE || tx->state == ISOTP_TX_CF ||
        !expired(now_us, tx->deadline_us))
        return 0;
    int rc =
        tx->state == ISOTP_TX_FC ? ISOTP_ERR_TIMEOUT_BS : ISOTP_ERR_TIMEOUT_A;
    tx->state = ISOTP_TX_IDLE;
    return rc;
}

int isotp_tx_flow_control(isotp_tx_t *tx, const struct can_frame *f,
                          uint32_t now_us) {
    int rc = isotp_tx_check_timeout(tx, now_us);
    if (rc < 0)
        return rc;
    unsigned a = tx->config.extended_address;
    if (tx->state != ISOTP_TX_FC || !matches(&tx->config, f) ||
        f->dlc < a + 3 || f->data[a] >> 4 != 3)
        return ISOTP_IGNORED;
    unsigned fs = f->data[a] & 0x0f;
    tx->padding_error |= f->dlc < 8;
    if (fs == 0) {
        tx->block_size = f->data[a + 1];
        uint8_t stmin = f->data[a + 2];
        tx->reserved_stmin |= stmin > 0x7f && (stmin < 0xf1 || stmin > 0xf9);
        tx->separation_us = tx->reserved_stmin ? 127000 : isotp_stmin_us(stmin);
        tx->block_count = 0;
        tx->wait_count = 0;
        tx->next_us = now_us;
        /* Separation applies across block boundaries too. */
        if (tx->sent_cf && !expired(now_us, tx->last_cf_us + tx->separation_us))
            tx->next_us = tx->last_cf_us + tx->separation_us;
        tx->state = ISOTP_TX_CF;
        return ISOTP_FRAME;
    }
    if (fs == 1 && ++tx->wait_count <= tx->config.wait_limit) {
        tx->deadline_us = now_us + tx->config.timeout_bs_us;
        return ISOTP_FRAME;
    }
    tx->state = ISOTP_TX_IDLE;
    if (fs == 1)
        return ISOTP_ERR_WAIT_LIMIT;
    return fs == 2 ? ISOTP_ERR_OVERFLOW : ISOTP_ERR_FLOW_STATUS;
}

int isotp_tx_next(isotp_tx_t *tx, struct can_frame *f, uint32_t now_us) {
    int rc = isotp_tx_check_timeout(tx, now_us);
    if (rc < 0)
        return rc;
    if (tx->state != ISOTP_TX_READY && tx->state != ISOTP_TX_CF)
        return ISOTP_IGNORED;
    if (tx->state == ISOTP_TX_CF && !expired(now_us, tx->next_us))
        return ISOTP_IGNORED;
    unsigned a = prepare(&tx->config, f);
    unsigned start = a + 1;
    unsigned size;
    tx->pending_cf = tx->offset != 0;
    if (!tx->offset && tx->total <= 7 - a) {
        f->data[a] = tx->total;
        size = tx->total;
    } else if (!tx->offset) {
        f->data[a] = 0x10 | (tx->total >> 8);
        f->data[a + 1] = tx->total & 0xff;
        start++;
        size = 6 - a;
    } else {
        f->data[a] = 0x20 | tx->sequence;
        tx->sequence = (tx->sequence + 1) & 0x0f;
        size = tx->total - tx->offset;
        if (size > 7 - a)
            size = 7 - a;
    }
    memcpy(f->data + start, tx->buffer + tx->offset, size);
    tx->offset += size;
    f->dlc = tx->config.pad ? 8 : start + size;
    tx->state = ISOTP_TX_CONFIRM;
    tx->deadline_us = now_us + tx->config.timeout_a_us;
    return ISOTP_FRAME;
}

int isotp_tx_confirm(isotp_tx_t *tx, bool success, uint32_t now_us) {
    if (tx->state != ISOTP_TX_CONFIRM)
        return ISOTP_ERR_ARGUMENT;
    int rc = isotp_tx_check_timeout(tx, now_us);
    if (rc < 0)
        return rc;
    if (!success) {
        tx->state = ISOTP_TX_IDLE;
        return ISOTP_ERR_SEND;
    }
    if (tx->offset == tx->total) {
        tx->state = ISOTP_TX_IDLE;
        return ISOTP_COMPLETE;
    }
    if (tx->pending_cf) {
        tx->sent_cf = true;
        tx->last_cf_us = now_us;
        tx->block_count++;
    }
    if (!tx->pending_cf ||
        (tx->block_size && tx->block_count == tx->block_size)) {
        tx->state = ISOTP_TX_FC;
        tx->deadline_us = now_us + tx->config.timeout_bs_us;
    } else {
        tx->state = ISOTP_TX_CF;
        tx->next_us = now_us + tx->separation_us;
    }
    return 0;
}
