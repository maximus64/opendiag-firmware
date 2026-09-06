/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file kline_codec.c
 * @brief The ISO 9141-2 / ISO 14230 grammar. See kline_codec.h.
 */

#include <string.h>

#include "kline_codec.h"

/* A real ECU oscillator and a microsecond timestamp are not exact. Six per
 * cent admits the usual 10400/10417 distinction without accepting 12500 baud,
 * which belongs to the manufacturer-specific parameter surface, not to the
 * ISO 5-baud synchronisation range. */
#define SYNC_RATE_MARGIN_PCT 6u
#define SYNC_INTERVAL_TOLERANCE_PCT 20u

uint32_t kline_sync_baud_from_edges(const uint32_t *edge_us, size_t count) {
    uint32_t span, mean, baud;
    uint32_t min_baud = KLINE_BAUD_MIN * (100u - SYNC_RATE_MARGIN_PCT) / 100u;
    uint32_t max_baud = KLINE_BAUD_MAX * (100u + SYNC_RATE_MARGIN_PCT) / 100u;

    if (!edge_us || count < KLINE_SYNC_EDGE_COUNT) {
        return 0;
    }

    /* Unsigned subtraction deliberately handles one wrap of the 32-bit
     * microsecond counter. There are nine complete bit periods between the
     * first and last of the ten transitions. */
    span = edge_us[KLINE_SYNC_EDGE_COUNT - 1] - edge_us[0];
    if (span == 0) {
        return 0;
    }
    mean =
        (span + (KLINE_SYNC_EDGE_COUNT - 1) / 2u) / (KLINE_SYNC_EDGE_COUNT - 1);
    if (mean == 0) {
        return 0;
    }

    /* One missed edge produces a two-bit interval. One noise edge produces
     * two short ones. Reject both rather than silently selecting a plausible
     * rate from the total span alone. */
    for (size_t i = 1; i < KLINE_SYNC_EDGE_COUNT; i++) {
        uint32_t interval = edge_us[i] - edge_us[i - 1];
        uint64_t scaled = (uint64_t)interval * 100u;

        if (scaled < (uint64_t)mean * (100u - SYNC_INTERVAL_TOLERANCE_PCT) ||
            scaled > (uint64_t)mean * (100u + SYNC_INTERVAL_TOLERANCE_PCT)) {
            return 0;
        }
    }

    baud = (uint32_t)((9000000ull + span / 2u) / span);
    return baud >= min_baud && baud <= max_baud ? baud : 0;
}

size_t kline_header_len(uint8_t fmt) {
    switch (kline_addr_mode(fmt)) {
    case KLINE_ADDR_PHYSICAL:
    case KLINE_ADDR_FUNCTIONAL:
        return 3; /* format, target, source */
    case KLINE_ADDR_CARB:
        /* ISO 9141-2 always sends three, but the length is not encoded, so
         * this only tells the caller where the data starts. */
        return 3;
    default:
        return 1;
    }
}

size_t kline_msg_len(const uint8_t *buf, size_t len) {
    size_t hdr, data;

    if (!buf || len < 1) {
        return 0;
    }

    /* Clause 4.1.1: the CARB exception mode puts no length anywhere. Only
     * the inter-byte gap ends one of those. */
    if (kline_is_carb_fmt(buf[0])) {
        return 0;
    }

    hdr = (kline_addr_mode(buf[0]) == KLINE_ADDR_NONE) ? 1 : 3;
    data = buf[0] & 0x3Fu;

    if (data == 0) {
        /* Clause 4.1.4: the six length bits are zero, so a separate length
         * byte follows the addresses. */
        if (len < hdr + 1) {
            return 0;
        }
        data = buf[hdr];
        if (data == 0) {
            /* A message with no data field at all is not a message. */
            return 0;
        }
        hdr += 1;
    }

    return hdr + data + 1; /* + checksum */
}

const char *kline_variant_name(kline_variant_t v) {
    switch (v) {
    case KLINE_VARIANT_ISO9141:
        return "ISO 9141-2";
    case KLINE_VARIANT_KWP:
        return "ISO 14230-4";
    default:
        return "unknown";
    }
}

uint8_t kline_checksum(const uint8_t *data, size_t len) {
    uint8_t sum = 0;

    for (size_t i = 0; i < len; i++) {
        sum = (uint8_t)(sum + data[i]);
    }

    return sum;
}

bool kline_frame_ok(const uint8_t *buf, size_t len) {
    if (!buf || len < KLINE_MIN_MSG || len > KLINE_MAX_MSG) {
        return false;
    }

    return kline_checksum(buf, len - 1) == buf[len - 1];
}

bool kline_odd_parity(uint8_t b) {
    uint8_t ones = 0;

    for (int i = 0; i < 8; i++) {
        ones = (uint8_t)(ones + ((b >> i) & 1u));
    }

    return (ones & 1u) != 0;
}

/** @brief Every KWP key byte pair puts this in key byte 2, Table 5. */
#define KLINE_KB_KWP 0x8F

/** Bit positions inside the capability key byte, derived from Table 5. */
#define KB_AL0 0x01u
#define KB_AL1 0x02u
#define KB_HB0 0x04u
#define KB_HB1 0x08u
#define KB_TP0 0x10u
#define KB_TP1 0x20u

bool kline_decode_keybytes(uint8_t b1, uint8_t b2, kline_keybytes_t *out) {
    kline_keybytes_t k;
    uint8_t cap;

    if (!out) {
        return false;
    }

    memset(&k, 0, sizeof(k));
    k.kb1 = b1;
    k.kb2 = b2;
    k.parity_ok = kline_odd_parity(b1) && kline_odd_parity(b2);

    if ((b1 == 0x08 && b2 == 0x08) || (b1 == 0x94 && b2 == 0x94)) {
        /* ISO 9141-2. The pair also encodes P2min, which this driver does not
         * shorten, so nothing else is taken from it. */
        k.variant = KLINE_VARIANT_ISO9141;
        k.hdr_address = true;
        *out = k;
        return true;
    }

    if (b2 == KLINE_KB_KWP) {
        cap = b1;
    } else if (b1 == KLINE_KB_KWP) {
        cap = b2;
    } else {
        *out = k;
        return false;
    }

    k.variant = KLINE_VARIANT_KWP;
    k.len_in_format = (cap & KB_AL0) != 0;
    k.extra_len_byte = (cap & KB_AL1) != 0;
    k.hdr_1byte = (cap & KB_HB0) != 0;
    k.hdr_address = (cap & KB_HB1) != 0;
    /* Table 4: TP0,TP1 of 1,0 selects the extended set, 0,1 the normal one.
     * Any other combination is undefined, and reading it as normal timing is
     * the conservative half of the choice. */
    k.extended_timing = (cap & KB_TP0) != 0 && (cap & KB_TP1) == 0;

    /* Table 5's decimal identifier: clear both parity bits, then key byte 2
     * weighs 2^7. */
    k.code = (uint16_t)(((KLINE_KB_KWP & 0x7Fu) << 7) | (cap & 0x7Fu));

    *out = k;
    return true;
}

/* ------------------------------------------------------------------ *
 * Timing
 * ------------------------------------------------------------------ */

void kline_timing_normal(kline_timing_t *out) {
    if (!out) {
        return;
    }

    /* Table 1. */
    out->p1_max = 20;
    out->p2_min = 25;
    out->p2_max = 50;
    out->p3_min = 55;
    out->p3_max = 5000;
    out->p4_min = 5;
}

void kline_timing_extended(kline_timing_t *out) {
    if (!out) {
        return;
    }

    /* Table 2. Only reachable on a physically addressed link whose key bytes
     * asked for it. */
    out->p1_max = 20;
    out->p2_min = 0;
    out->p2_max = 1000;
    out->p3_min = 0;
    out->p3_max = 5000;
    out->p4_min = 5;
}

uint32_t kline_p2max_from_byte(uint8_t v) {
    if (v == 0) {
        return 0;
    }

    if (v <= 0xF0) {
        return (uint32_t)v * 25u;
    }

    /* Table 3: past $F0 the low nibble counts 256 units of 25 ms. */
    return (uint32_t)(v & 0x0Fu) * 256u * 25u;
}

/* ------------------------------------------------------------------ *
 * Initialisation
 * ------------------------------------------------------------------ */

bool kline_is_response_pending(const uint8_t *buf, size_t len) {
    size_t hdr;

    if (!buf || len < KLINE_MIN_MSG) {
        return false;
    }

    hdr = kline_header_len(buf[0]);
    if (kline_addr_mode(buf[0]) != KLINE_ADDR_CARB && (buf[0] & 0x3Fu) == 0 &&
        len > hdr) {
        hdr += 1; /* step over the additional length byte */
    }

    /* 7F <service> 78, checksum after it. */
    if (len < hdr + 3 + 1) {
        return false;
    }

    return buf[hdr] == KLINE_SVC_NEG_RESP &&
           buf[hdr + 2] == KLINE_NRC_RESPONSE_PENDING;
}

size_t kline_default_wakeup(kline_variant_t v, uint8_t *out, size_t cap) {
    /* The two the ELM327 datasheet documents under AT WM. */
    static const uint8_t iso9141[] = {0x68, 0x6A, 0xF1, 0x01, 0x00};
    static const uint8_t kwp[] = {0xC1, 0x33, 0xF1, KLINE_SVC_TESTER_PRESENT};

    const uint8_t *src = (v == KLINE_VARIANT_KWP) ? kwp : iso9141;
    size_t len = (v == KLINE_VARIANT_KWP) ? sizeof(kwp) : sizeof(iso9141);

    if (!out || cap < len) {
        return 0;
    }

    memcpy(out, src, len);
    return len;
}
