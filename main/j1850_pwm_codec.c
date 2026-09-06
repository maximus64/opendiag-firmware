/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file j1850_pwm_codec.c
 * @brief SAE J1850 PWM symbol coding, framing and CRC. No hardware.
 */

#include <string.h>

#include "j1850_pwm_codec.h"

/* ------------------------------------------------------------------ *
 * Encoding
 * ------------------------------------------------------------------ */

/*
 * The trim must not push an emitted width outside the window it is aiming at
 * even before the driver stage adds its delay back on, and a trim large
 * enough to invert a pulse would be a silent disaster on the wire.
 */
_Static_assert(J1850_PWM_TX_TP1 >= J1850_PWM_TP1_TX_MIN - 1 &&
                   J1850_PWM_TX_TP1 <= J1850_PWM_TP1_TX_MAX,
               "J1850_PWM_TX_TRIM_US puts the emitted Tp1 out of range");
_Static_assert(J1850_PWM_TX_TP2 >= J1850_PWM_TP2_TX_MIN - 1 &&
                   J1850_PWM_TX_TP2 <= J1850_PWM_TP2_TX_MAX,
               "J1850_PWM_TX_TRIM_US puts the emitted Tp2 out of range");
_Static_assert(J1850_PWM_TX_TP7 >= J1850_PWM_TP7_TX_MIN - 1 &&
                   J1850_PWM_TX_TP7 <= J1850_PWM_TP7_TX_MAX,
               "J1850_PWM_TX_TRIM_US puts the emitted Tp7 out of range");

/** @brief One bit cell: active for @p active_us, passive for the rest of Tp3.
 */
static void put_cell(rmt_symbol_word_t *s, uint16_t active_us,
                     uint16_t cell_us) {
    s->level0 = J1850_PWM_ACTIVE;
    s->duration0 = active_us;
    s->level1 = J1850_PWM_PASSIVE;
    s->duration1 = (uint16_t)(cell_us - active_us);
}

size_t j1850_pwm_encode(const uint8_t *data, size_t len, bool with_sof,
                        rmt_symbol_word_t *out, size_t cap) {
    size_t need = len * 8 + (with_sof ? 1u : 0u);
    size_t n = 0;

    if (!data || !out || len == 0 || len > J1850_PWM_MAX_FRAME || cap < need) {
        return 0;
    }

    if (with_sof) {
        /* Active for Tp7, then passive until the first data bit's rising
         * edge falls Tp4 after the SOF's own - clause 6.6.1.2 c. Only the
         * active phase carries the trim; the cell length is what sets the
         * next rising edge and the driver stage does not move that. */
        put_cell(&out[n++], J1850_PWM_TX_TP7, J1850_PWM_TP4_NOM);
    }

    for (size_t i = 0; i < len; i++) {
        for (int bit = 7; bit >= 0; bit--) { /* MSB first, clause 5.3.2 */
            bool one = (data[i] >> bit) & 1u;

            put_cell(&out[n++], one ? J1850_PWM_TX_TP1 : J1850_PWM_TX_TP2,
                     J1850_PWM_TP3_NOM);
        }
    }

    return n;
}

/* ------------------------------------------------------------------ *
 * Decoding
 * ------------------------------------------------------------------ */

const char *j1850_pwm_rx_status_str(j1850_pwm_rx_status_t st) {
    switch (st) {
    case J1850_PWM_RX_OK:
        return "ok";
    case J1850_PWM_RX_IFR_ONLY:
        return "ifr";
    case J1850_PWM_RX_BREAK:
        return "break";
    case J1850_PWM_RX_EMPTY:
        return "empty";
    case J1850_PWM_RX_NO_SOF:
        return "no-sof";
    case J1850_PWM_RX_BAD_SYMBOL:
        return "bad-symbol";
    case J1850_PWM_RX_BAD_TIMING:
        return "bad-timing";
    case J1850_PWM_RX_FRAMING:
        return "framing";
    case J1850_PWM_RX_TOO_LONG:
        return "too-long";
    case J1850_PWM_RX_SHORT:
        return "short";
    case J1850_PWM_RX_BAD_CRC:
        return "bad-crc";
    default:
        return "?";
    }
}

/**
 * @brief Walks an RMT capture one timed pulse at a time.
 *
 * Each symbol word holds two of them, and the peripheral marks the end of the
 * capture with a zero duration, so a pulse index runs to twice the symbol
 * count but usually stops earlier.
 */
typedef struct {
    const rmt_symbol_word_t *sym;
    size_t pulses; /* 2 per symbol */
    size_t idx;
    uint8_t level;
    uint32_t dur;
} pulse_iter_t;

static void IRAM_ATTR pulse_begin(pulse_iter_t *it,
                                  const rmt_symbol_word_t *sym, size_t n) {
    it->sym = sym;
    it->pulses = n * 2;
    it->idx = 0;
    it->level = J1850_PWM_PASSIVE;
    it->dur = 0;
}

/** @brief Loads the next pulse into @p it. False at the end of the capture. */
static bool IRAM_ATTR pulse_next(pulse_iter_t *it) {
    rmt_symbol_word_t s;

    if (it->idx >= it->pulses) {
        return false;
    }

    s = it->sym[it->idx >> 1];

    if (it->idx & 1u) {
        it->level = (uint8_t)s.level1;
        it->dur = s.duration1;
    } else {
        it->level = (uint8_t)s.level0;
        it->dur = s.duration0;
    }

    it->idx++;

    /* Zero duration is the peripheral's end marker, not a pulse. */
    return it->dur != 0;
}

/** What an active pulse width means, before any context is applied. */
typedef enum {
    SYM_ONE,
    SYM_ZERO,
    SYM_SOF,
    SYM_BRK,
    SYM_INVALID,
} active_sym_t;

static active_sym_t IRAM_ATTR classify_active(uint32_t us) {
    if (us >= J1850_PWM_TP1_RX_MIN && us <= J1850_PWM_TP2_RX_MAX) {
        /* Inside the merged bit window. Table 3 leaves the gap between
         * Tp1(max) and Tp2(min) to the implementer; split at the midpoint. */
        return (us < J1850_PWM_BIT_SPLIT) ? SYM_ONE : SYM_ZERO;
    }
    if (us >= J1850_PWM_TP7_RX_MIN && us <= J1850_PWM_TP7_RX_MAX) {
        return SYM_SOF;
    }
    if (us >= J1850_PWM_TP8_RX_MIN && us <= J1850_PWM_TP8_RX_MAX) {
        return SYM_BRK;
    }

    return SYM_INVALID;
}

/** @brief Appends one bit to the data or IFR field. False when it is full. */
static bool IRAM_ATTR push_bit(j1850_pwm_rx_t *out, bool in_ifr, bool one,
                               uint8_t *nbits) {
    uint8_t *buf = in_ifr ? out->ifr : out->data;
    uint8_t *len = in_ifr ? &out->ifr_len : &out->len;

    if (*nbits == 0) {
        if (*len >= J1850_PWM_MAX_FRAME) {
            return false;
        }
        buf[*len] = 0;
    }

    if (one) {
        buf[*len] |= (uint8_t)(1u << (7 - *nbits));
    }

    if (++(*nbits) == 8) {
        *nbits = 0;
        (*len)++;
    }

    return true;
}

j1850_pwm_rx_status_t IRAM_ATTR j1850_pwm_decode(const rmt_symbol_word_t *sym,
                                                 size_t n,
                                                 j1850_pwm_rx_t *out) {
    pulse_iter_t it;
    uint32_t prev_active;
    uint8_t nbits = 0;
    bool in_ifr = false;
    bool after_sof;
    bool ended = false;

    if (!out) {
        return J1850_PWM_RX_EMPTY;
    }

    memset(out, 0, sizeof(*out));
    out->status = J1850_PWM_RX_EMPTY;

    if (!sym || n == 0) {
        return out->status;
    }

    pulse_begin(&it, sym, n);

    /* Leading idle, however long the capture happened to start before the
     * first edge. */
    do {
        if (!pulse_next(&it)) {
            return out->status; /* nothing but idle */
        }
    } while (it.level != J1850_PWM_ACTIVE);

    /* The opening active pulse says what this capture is. */
    switch (classify_active(it.dur)) {
    case SYM_SOF:
        after_sof = true;
        out->status = J1850_PWM_RX_OK;
        break;

    case SYM_BRK:
        out->status = J1850_PWM_RX_BREAK;
        return out->status;

    case SYM_ONE:
    case SYM_ZERO:
        /* No SOF: in-frame response bytes, which follow another node's EOD.
         * Reception of them starts here because the EOD gap that precedes
         * them is long enough to close the previous capture. */
        after_sof = false;
        in_ifr = true;
        out->status = J1850_PWM_RX_IFR_ONLY;
        (void)push_bit(out, true, classify_active(it.dur) == SYM_ONE, &nbits);
        break;

    default:
        out->status = J1850_PWM_RX_NO_SOF;
        return out->status;
    }

    prev_active = it.dur;
    out->last_active_us = (uint16_t)it.dur;

    /* Body. Each turn consumes the passive phase then the next rising edge,
     * so the loop always holds a complete rising-to-rising interval. */
    while (!ended) {
        uint32_t passive, edge_gap;
        active_sym_t s;

        if (!pulse_next(&it)) {
            break; /* capture ended in the passive phase: EOF */
        }
        if (it.level != J1850_PWM_PASSIVE) {
            out->status = J1850_PWM_RX_BAD_SYMBOL; /* levels must alternate */
            return out->status;
        }
        passive = it.dur;

        if (!pulse_next(&it)) {
            break; /* no further rising edge: EOF */
        }

        /* Clause 6.6.1.8: cell boundaries are measured rising edge to rising
         * edge, never from the fall, which network capacitance smears. */
        edge_gap = prev_active + passive;

        if (edge_gap >= J1850_PWM_TP5_RX_MIN) {
            /* EOF expired before this edge, so it opens a new frame. Ours
             * is complete; leave the rest to the next capture. */
            break;
        }

        if (after_sof) {
            /* The first data bit's rising edge is Tp4 after the SOF's. */
            if (edge_gap < J1850_PWM_TP4_RX_MIN ||
                edge_gap > J1850_PWM_TP4_RX_MAX) {
                out->status = J1850_PWM_RX_BAD_TIMING;
                return out->status;
            }
            after_sof = false;
        } else if (edge_gap >= J1850_PWM_TP4_RX_MIN) {
            /* EOD. Clause 5.3.4.2: what follows is the in-frame response.
             * A second one is not a thing, so treat it as the end. */
            if (in_ifr) {
                break;
            }
            if (nbits != 0) {
                out->status = J1850_PWM_RX_FRAMING;
                return out->status;
            }
            in_ifr = true;
            out->eod = true;
            out->eod_gap_us = (uint16_t)edge_gap;
        } else if (edge_gap < J1850_PWM_TP3_RX_MIN ||
                   edge_gap > J1850_PWM_TP3_RX_MAX) {
            out->status = J1850_PWM_RX_BAD_TIMING;
            return out->status;
        }

        s = classify_active(it.dur);
        if (s != SYM_ONE && s != SYM_ZERO) {
            /* A SOF or BRK this far into a frame means the frame was cut
             * short; report what caused it rather than the bytes so far. */
            out->status = (s == SYM_INVALID) ? J1850_PWM_RX_BAD_SYMBOL
                                             : J1850_PWM_RX_BAD_TIMING;
            return out->status;
        }

        if (!push_bit(out, in_ifr, s == SYM_ONE, &nbits)) {
            out->status = J1850_PWM_RX_TOO_LONG;
            return out->status;
        }

        prev_active = it.dur;
        out->last_active_us = (uint16_t)it.dur;
    }

    if (nbits != 0) {
        out->status = J1850_PWM_RX_FRAMING;
        return out->status;
    }

    if (out->status == J1850_PWM_RX_IFR_ONLY) {
        /* IFR bytes carry no CRC unless the responder chose to append one,
         * and there is no way to tell from the bytes alone. Hand them up as
         * they are; only a complete byte is required. */
        return (out->ifr_len == 0) ? (out->status = J1850_PWM_RX_EMPTY)
                                   : out->status;
    }

    if (out->len < 2) {
        out->status = J1850_PWM_RX_SHORT;
    } else if (!j1850_crc_check(out->data, out->len)) {
        out->status = J1850_PWM_RX_BAD_CRC;
    }

    return out->status;
}

size_t IRAM_ATTR j1850_pwm_quick_decode(const rmt_symbol_word_t *sym, size_t n,
                                        uint8_t *out, size_t cap) {
    size_t i = 0;
    size_t len = 0;
    uint8_t nbits = 0;
    uint8_t acc = 0;

    if (!sym || !out || cap == 0) {
        return 0;
    }

    /* Leading idle, then the SOF. Checked because without it there is no way
     * to tell a frame from another node's in-frame response, and answering
     * one of those would be answering a frame that was never addressed. */
    while (i < n && sym[i].level0 != J1850_PWM_ACTIVE && sym[i].duration0) {
        i++;
    }
    if (i >= n || classify_active(sym[i].duration0) != SYM_SOF) {
        return 0;
    }

    for (i++; i < n; i++) {
        uint32_t active = sym[i].duration0;

        if (active == 0) {
            break; /* the peripheral's end marker */
        }

        acc = (uint8_t)((acc << 1) | (active < J1850_PWM_BIT_SPLIT));

        if (++nbits == 8) {
            if (len >= cap) {
                return 0; /* longer than the caller can hold */
            }
            out[len++] = acc;
            nbits = 0;
            acc = 0;
        }

        /* Stop at an end of data. One add and one compare, and without it a
         * frame that carries an in-frame response comes back with somebody
         * else's byte glued to the end of it - which fails its CRC, and,
         * worse, stops this node recognising the echo of its own frame. */
        if (active + sym[i].duration1 >= J1850_PWM_TP4_RX_MIN) {
            break;
        }
    }

    return nbits ? 0 : len;
}

/* ------------------------------------------------------------------ *
 * In-frame response
 * ------------------------------------------------------------------ */

bool IRAM_ATTR j1850_pwm_stream_pulse(j1850_pwm_stream_t *s, uint32_t active_us,
                                      uint32_t edge_us) {
    if (active_us >= J1850_PWM_TP7_RX_MIN &&
        active_us <= J1850_PWM_TP7_RX_MAX) {
        s->len = s->bits = s->byte = 0;
        s->crc = 0xFF;
        s->valid = s->first = true;
        return false;
    }
    if (!s->valid)
        return false;
    if (active_us < J1850_PWM_TP1_RX_MIN || active_us > J1850_PWM_TP2_RX_MAX ||
        edge_us < (s->first ? J1850_PWM_TP4_RX_MIN : J1850_PWM_TP3_RX_MIN) ||
        edge_us > (s->first ? J1850_PWM_TP4_RX_MAX : J1850_PWM_TP3_RX_MAX) ||
        s->len >= J1850_PWM_MAX_FRAME) {
        s->valid = false;
        return false;
    }
    s->first = false;
    s->byte = (uint8_t)((s->byte << 1) | (active_us < J1850_PWM_BIT_SPLIT));
    if (++s->bits != 8)
        return false;
    s->data[s->len++] = s->byte;
    s->crc ^= s->byte;
    for (unsigned i = 0; i < 8; i++)
        s->crc = (s->crc & 0x80) ? (uint8_t)((s->crc << 1) ^ 0x1D)
                                 : (uint8_t)(s->crc << 1);
    s->byte = s->bits = 0;
    /* Leave room for our IFR in the twelve-byte message limit. */
    return s->len >= 4 && s->len < J1850_PWM_MAX_FRAME &&
           s->crc == J1850_CRC_RESIDUE;
}

void j1850_pwm_ifr_cfg_default(j1850_pwm_ifr_cfg_t *cfg) {
    if (!cfg) {
        return;
    }

    memset(cfg, 0, sizeof(*cfg));
    cfg->enabled = true;
    cfg->node_address = 0xF1; /* the tester's physical address */
    cfg->targets[0] = 0x6B;   /* modules answer a functional request here */
    cfg->targets[1] = 0xF1;   /* and a physically addressed reply, here */
    cfg->target_count = 2;
}

bool IRAM_ATTR j1850_pwm_ifr_wanted(const uint8_t *frame, size_t len,
                                    const j1850_pwm_ifr_cfg_t *cfg) {
    if (!frame || !cfg || !cfg->enabled) {
        return false;
    }

    /* Header, target, and at least a CRC under them. */
    if (len < 3) {
        return false;
    }

    if (!j1850_hdr_wants_ifr(frame[0])) {
        return false;
    }

    /* A single byte header has no target field, so nothing says the frame
     * was for us. Staying quiet is the safe half of that ambiguity. */
    if (!j1850_hdr_is_three_byte(frame[0])) {
        return false;
    }

    if ((frame[0] & J1850_HDR_Y_BIT) && frame[1] == cfg->node_address)
        return true;

    for (uint8_t i = 0; i < cfg->target_count && i < J1850_PWM_MAX_IFR_TARGETS;
         i++) {
        if (frame[1] == cfg->targets[i]) {
            return true;
        }
    }

    return false;
}
