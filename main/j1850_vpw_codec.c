/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file j1850_vpw_codec.c
 * @brief SAE J1850 VPW symbol coding and framing. No hardware.
 */

#include <string.h>

#include "j1850_vpw_codec.h"

/* ------------------------------------------------------------------ *
 * Encoding
 * ------------------------------------------------------------------ */

/*
 * Every emitted width, both levels of it, has to sit inside the transmit
 * windows of Table 5 - and a trim large enough to walk one out of its window
 * would be a silent disaster on the wire, so it is caught here rather than on
 * a vehicle.
 */
_Static_assert(J1850_VPW_TX_TV1_ACTIVE >= J1850_VPW_TV1_TX_MIN &&
                   J1850_VPW_TX_TV1_PASSIVE <= J1850_VPW_TV1_TX_MAX,
               "J1850_VPW_TX_TRIM_US puts the emitted short pulse outside Tv1");
_Static_assert(J1850_VPW_TX_TV2_ACTIVE >= J1850_VPW_TV2_TX_MIN &&
                   J1850_VPW_TX_TV2_PASSIVE <= J1850_VPW_TV2_TX_MAX,
               "J1850_VPW_TX_TRIM_US puts the emitted long pulse outside Tv2");
_Static_assert(J1850_VPW_TX_TV3_ACTIVE >= J1850_VPW_TV3_TX_MIN &&
                   J1850_VPW_TX_TV3_PASSIVE <= J1850_VPW_TV3_TX_MAX,
               "J1850_VPW_TX_TRIM_US puts the emitted SOF outside Tv3");

/**
 * @brief Write pulse @p i of the train into the symbol array.
 *
 * Pulse 0 is the SOF and the levels alternate from there, so an even index is
 * always the active half of a symbol word and an odd one always the passive
 * half. That is the whole reason this mapping is arithmetic rather than
 * stateful: the RMT's two-pulses-per-word layout happens to line up exactly
 * with VPW's active-then-passive alternation.
 */
static void put_pulse(rmt_symbol_word_t *out, size_t i, uint16_t us) {
    rmt_symbol_word_t *s = &out[i >> 1];

    if (i & 1u) {
        s->level1 = J1850_VPW_PASSIVE;
        s->duration1 = us;
    } else {
        s->level0 = J1850_VPW_ACTIVE;
        s->duration0 = us;
    }
}

size_t j1850_vpw_encode(const uint8_t *data, size_t len, rmt_symbol_word_t *out,
                        size_t cap) {
    size_t need = 4 * len + 1;
    size_t i = 0;

    if (!data || !out || len == 0 || len > J1850_VPW_MAX_FRAME || cap < need) {
        return 0;
    }

    put_pulse(out, i++, J1850_VPW_TX_TV3_ACTIVE); /* SOF, clause 6.6.2.2 */

    for (size_t b = 0; b < len; b++) {
        for (int bit = 7; bit >= 0; bit--) { /* MSB first, clause 5.3.2 */
            bool one = (data[b] >> bit) & 1u;
            bool active = (i & 1u) == 0;
            uint16_t us;

            /* Figure 14. The level is set by where in the alternation this
             * bit falls; the value picks the width against it. */
            if (active) {
                us = one ? J1850_VPW_TX_TV1_ACTIVE : J1850_VPW_TX_TV2_ACTIVE;
            } else {
                us = one ? J1850_VPW_TX_TV2_PASSIVE : J1850_VPW_TX_TV1_PASSIVE;
            }

            put_pulse(out, i++, us);
        }
    }

    /* The frame ended on an active pulse - it always does, because SOF is
     * active and 8 bits a byte keeps the parity - so this passive EOD fills
     * the second half of the last symbol word and the count comes out exact. */
    put_pulse(out, i++, J1850_VPW_TX_TV3_PASSIVE);

    return i / 2;
}

/* ------------------------------------------------------------------ *
 * Contention
 * ------------------------------------------------------------------ */

bool IRAM_ATTR j1850_vpw_pulse_at(const uint8_t *data, size_t len, size_t k,
                                  j1850_vpw_pulse_t *out) {
    size_t bit;
    bool one;

    if (!data || !out || len == 0 || len > J1850_VPW_MAX_FRAME ||
        k >= J1850_VPW_PULSE_COUNT(len)) {
        return false;
    }

    if (k == 0) {
        out->level = J1850_VPW_ACTIVE;
        out->us = J1850_VPW_TX_TV3_ACTIVE; /* SOF */
        return true;
    }

    if (k == J1850_VPW_PULSE_COUNT(len) - 1) {
        out->level = J1850_VPW_PASSIVE;
        out->us = J1850_VPW_TX_TV3_PASSIVE; /* the trailing EOD */
        return true;
    }

    /* Pulse 0 is the SOF and the levels alternate from there, so an even
     * index is active - the same arithmetic the encoder uses. */
    bit = k - 1;
    one = (data[bit >> 3] >> (7 - (bit & 7u))) & 1u;

    if ((k & 1u) == 0) {
        out->level = J1850_VPW_ACTIVE;
        out->us = one ? J1850_VPW_TX_TV1_ACTIVE : J1850_VPW_TX_TV2_ACTIVE;
    } else {
        out->level = J1850_VPW_PASSIVE;
        out->us = one ? J1850_VPW_TX_TV2_PASSIVE : J1850_VPW_TX_TV1_PASSIVE;
    }

    return true;
}

size_t IRAM_ATTR j1850_vpw_watch_offsets(const j1850_vpw_pulse_t *p,
                                         uint16_t head_us, uint16_t tail_us,
                                         uint16_t *out, size_t cap) {
    size_t n = 0;
    uint32_t last;

    if (!p || !out || p->level != J1850_VPW_PASSIVE) {
        return 0;
    }

    last = (uint32_t)p->us;
    if (last <= tail_us) {
        return 0; /* nothing of this pulse can be trusted */
    }
    last -= tail_us;

    /*
     * The guard is charged at *both* ends of the pulse, and the second one is
     * the easier of the two to forget.
     *
     * At the start it covers this board's own edge settling - the falling
     * edge that opens a passive phase arrives a few microseconds after the
     * driver asked for it, and a sample any sooner would read the tail of our
     * own active phase.
     *
     * At the end it covers the opposite error, which is subtler and cost a
     * frame in about every twenty-five before it was allowed for: a passive
     * pulse is *shorter* on the wire than nominal by the same skew, because
     * the active phase before it ran long. Measured here, a 64 us passive
     * bit occupies 59. A sample landing in that missing tail - a delayed
     * interrupt is all it takes - reads the node's own next active phase and
     * calls it somebody else's.
     *
     * The caller's reference carries error of its own, so the tail margin is
     * its parameter rather than a copy of the head. A check is only offered
     * when both fit, and j1850_vpw.c refuses any sample arriving past the
     * same tail.
     */
    if (n < cap && head_us < last) {
        out[n++] = head_us;
    }

    /*
     * The second instant exists only when this pulse outlasts a short one -
     * that is, when we are driving the long form and a competitor driving the
     * short form would already have gone active. Written against the pulse
     * rather than against "is it Tv2" so that a trimmed or rescaled width
     * cannot quietly produce a check with no room at the end of it.
     */
    if (n < cap && (uint32_t)J1850_VPW_TV1_NOM + head_us < last) {
        out[n++] = (uint16_t)(J1850_VPW_TV1_NOM + head_us);
    }

    return n;
}

/* ------------------------------------------------------------------ *
 * Decoding
 * ------------------------------------------------------------------ */

const char *j1850_vpw_rx_status_str(j1850_vpw_rx_status_t st) {
    switch (st) {
    case J1850_VPW_RX_OK:
        return "ok";
    case J1850_VPW_RX_IFR_ONLY:
        return "ifr";
    case J1850_VPW_RX_BREAK:
        return "break";
    case J1850_VPW_RX_EMPTY:
        return "empty";
    case J1850_VPW_RX_NO_SOF:
        return "no-sof";
    case J1850_VPW_RX_BAD_SYMBOL:
        return "bad-symbol";
    case J1850_VPW_RX_FRAMING:
        return "framing";
    case J1850_VPW_RX_TOO_LONG:
        return "too-long";
    case J1850_VPW_RX_SHORT:
        return "short";
    case J1850_VPW_RX_BAD_CRC:
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
    it->level = J1850_VPW_PASSIVE;
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

/**
 * @brief Which of Table 5's windows a pulse width falls in.
 *
 * Level free on purpose: in VPW the width says which of the four symbol
 * classes a pulse belongs to, and the level then says which member of that
 * class it is - a Tv3 is a SOF driven active and an EOD released passive, and
 * a Tv1 is a "1" or a "0" the same way. Splitting the decision in two is what
 * keeps the alternation rule enforceable separately from the widths.
 */
typedef enum {
    W_INVALID, /**< Under Tv1(rx,min): noise, or a pulse the filter let past. */
    W_SHORT,   /**< Tv1. */
    W_LONG,    /**< Tv2. */
    W_TV3,     /**< SOF or EOD. */
    W_TV4,     /**< EOF or BRK, and the end of a capture either way. */
} width_t;

static width_t IRAM_ATTR classify(uint32_t us) {
    if (us < J1850_VPW_TV1_RX_MIN) {
        return W_INVALID;
    }
    if (us <= J1850_VPW_TV1_RX_MAX) {
        return W_SHORT;
    }
    if (us <= J1850_VPW_TV2_RX_MAX) {
        return W_LONG;
    }
    if (us <= J1850_VPW_TV3_RX_MAX) {
        return W_TV3;
    }

    return W_TV4;
}

/** @brief The bit a pulse of this width, at this level, carries. Figure 14. */
static inline bool IRAM_ATTR bit_value(uint8_t level, width_t w) {
    return (level == J1850_VPW_ACTIVE) ? (w == W_SHORT) : (w == W_LONG);
}

/** @brief Appends one bit to the data or IFR field. False when it is full. */
static bool IRAM_ATTR push_bit(j1850_vpw_rx_t *out, bool in_ifr, bool one,
                               uint8_t *nbits) {
    uint8_t *buf = in_ifr ? out->ifr : out->data;
    uint8_t *len = in_ifr ? &out->ifr_len : &out->len;

    if (*nbits == 0) {
        if (*len >= J1850_VPW_MAX_FRAME) {
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

j1850_vpw_rx_status_t IRAM_ATTR j1850_vpw_decode(const rmt_symbol_word_t *sym,
                                                 size_t n,
                                                 j1850_vpw_rx_t *out) {
    pulse_iter_t it;
    uint8_t nbits = 0;
    uint8_t expect;
    bool in_ifr = false;
    bool need_nb = false;

    if (!out) {
        return J1850_VPW_RX_EMPTY;
    }

    memset(out, 0, sizeof(*out));
    out->status = J1850_VPW_RX_EMPTY;

    if (!sym || n == 0) {
        return out->status;
    }

    pulse_begin(&it, sym, n);

    /* Leading idle, however long the capture happened to start before the
     * first edge. Every symbol that opens something - SOF, BRK, a
     * normalization bit - is active, so the first active pulse is where the
     * decision gets made. */
    do {
        if (!pulse_next(&it)) {
            return out->status; /* nothing but idle */
        }
    } while (it.level != J1850_VPW_ACTIVE);

    switch (classify(it.dur)) {
    case W_TV3:
        /* Active Tv3 is a start of frame. */
        out->status = J1850_VPW_RX_OK;
        break;

    case W_TV4:
        /* Active and longer than an EOF: a break, clause 6.6.2.7. */
        out->status = J1850_VPW_RX_BREAK;
        return out->status;

    case W_SHORT:
    case W_LONG:
        /* Bits with no SOF in front of them. On this bus that shape is a
         * responder's in-frame response: its normalization bit is active and
         * follows the originator's EOD, and 200 us of passive is long enough
         * to have closed the capture the frame itself arrived in. */
        out->status = J1850_VPW_RX_IFR_ONLY;
        in_ifr = true;
        out->nb = true;
        out->nb_long = classify(it.dur) == W_LONG;
        break;

    default:
        out->status = J1850_VPW_RX_NO_SOF;
        return out->status;
    }

    /* SOF and NB are both active, and the levels alternate from there. */
    expect = J1850_VPW_PASSIVE;

    for (;;) {
        width_t w;

        if (!pulse_next(&it)) {
            break; /* the capture ran out: whatever is here is what there is */
        }

        /* The alternation rule. Two pulses at one level cannot both be
         * symbols, and there is no way to tell which of them to believe -
         * so this is reported rather than guessed at. */
        if (it.level != expect) {
            out->status = J1850_VPW_RX_BAD_SYMBOL;
            return out->status;
        }
        expect = !expect;

        w = classify(it.dur);

        switch (w) {
        case W_INVALID:
            out->status = J1850_VPW_RX_BAD_SYMBOL;
            return out->status;

        case W_TV4:
            if (it.level == J1850_VPW_ACTIVE) {
                out->status = J1850_VPW_RX_BREAK;
                return out->status;
            }
            goto ended; /* EOF: clause 5.3.4.3, the frame is complete */

        case W_TV3:
            if (it.level == J1850_VPW_ACTIVE) {
                /* A SOF cannot appear inside a frame. Reaching one means the
                 * frame this capture opened with never ended, because an EOF
                 * would have closed the capture before this edge. */
                out->status = J1850_VPW_RX_BAD_SYMBOL;
                return out->status;
            }
            /* EOD, clause 5.3.4.2: the originator has stopped and what
             * follows, if anything, belongs to a responder. */
            if (in_ifr) {
                goto ended; /* a second EOD; nothing more is defined after it */
            }
            if (nbits != 0) {
                out->status = J1850_VPW_RX_FRAMING;
                return out->status;
            }
            in_ifr = true;
            need_nb = true;
            out->eod = true;
            continue;

        default: /* W_SHORT or W_LONG */
            break;
        }

        if (need_nb) {
            /* Clause 6.6.2.5: the responder opens with an active
             * normalization bit, which carries no data of its own. */
            need_nb = false;
            out->nb = true;
            out->nb_long = w == W_LONG;
            continue;
        }

        if (!push_bit(out, in_ifr, bit_value(it.level, w), &nbits)) {
            out->status = J1850_VPW_RX_TOO_LONG;
            return out->status;
        }
    }

ended:
    if (nbits != 0) {
        out->status = J1850_VPW_RX_FRAMING;
        return out->status;
    }

    if (out->status == J1850_VPW_RX_IFR_ONLY) {
        /* Clause 5.3.7 d leaves it to the responder whether an IFR carries a
         * CRC, and the normalization bit that would have said so is only a
         * "preferred method". So these bytes are handed up as they are; only
         * a whole number of them is required. */
        return (out->ifr_len == 0) ? (out->status = J1850_VPW_RX_EMPTY)
                                   : out->status;
    }

    if (out->len < 2) {
        out->status = J1850_VPW_RX_SHORT;
    } else if (!j1850_crc_check(out->data, out->len)) {
        out->status = J1850_VPW_RX_BAD_CRC;
    }

    return out->status;
}
