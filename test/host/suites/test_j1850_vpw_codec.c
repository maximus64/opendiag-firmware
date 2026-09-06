/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_j1850_vpw_codec.c
 * @brief SAE J1850 VPW symbol coding and framing.
 *
 * The driver's protocol half runs here in full: every timing window in
 * Table 5 is walked to its edges, both bit encodings of Figure 14 are checked
 * against each other, the in-frame response rules of clause 5.3.7 are
 * exercised, and a pseudo-random sweep pushes jittered, truncated and
 * corrupted captures through the decoder to show it always terminates with a
 * verdict instead of reading past its input.
 *
 * Clause references are to SAE J1850 rev. FEB1994.
 */

#include <string.h>

#include "j1850_vpw_codec.h"
#include "td_test.h"

/* ------------------------------------------------------------------ *
 * Building captures
 *
 * A VPW capture is a run of single pulses whose levels alternate, starting
 * active. The RMT packs two per symbol word, so appending is by pulse index
 * and the level falls out of its parity - which is the same arithmetic the
 * encoder uses, and is why cap_pulse_at() exists separately for the cases
 * that need to break the alternation on purpose.
 * ------------------------------------------------------------------ */

typedef struct {
    rmt_symbol_word_t sym[256];
    size_t n;           /**< Symbol words used. */
    size_t pulses;      /**< Pulses appended. */
    uint8_t next_level; /**< Where the alternation stands. */
} cap_t;

static void cap_reset(cap_t *c) {
    memset(c, 0, sizeof(*c));
    /* Everything that opens a capture - SOF, BRK, a normalization bit - is
     * driven active. */
    c->next_level = J1850_VPW_ACTIVE;
}

/** Overwrites the width of the pulse most recently appended. */
static void cap_restretch_last(cap_t *c, uint16_t us) {
    size_t last = c->pulses - 1;

    if (last & 1u) {
        c->sym[last >> 1].duration1 = us;
    } else {
        c->sym[last >> 1].duration0 = us;
    }
}

/**
 * Appends one pulse at an explicit level, wherever the alternation is.
 *
 * The alternation is tracked rather than derived from the pulse index,
 * because a capture that opened while the bus was already quiet has a passive
 * pulse in front of everything and shifts every symbol into the other half of
 * its word - which is exactly the shape the decoder has to cope with.
 */
static void cap_pulse_at(cap_t *c, uint8_t level, uint16_t us) {
    rmt_symbol_word_t *s = &c->sym[c->pulses >> 1];

    if (c->pulses & 1u) {
        s->level1 = level;
        s->duration1 = us;
    } else {
        s->level0 = level;
        s->duration0 = us;
    }

    c->pulses++;
    c->n = (c->pulses + 1) / 2;
    c->next_level = level ? J1850_VPW_PASSIVE : J1850_VPW_ACTIVE;
}

/** Appends one pulse at the level the alternation calls for. */
static void cap_pulse(cap_t *c, uint16_t us) {
    cap_pulse_at(c, c->next_level, us);
}

/** True when the next pulse appended would be an active one. */
static bool cap_next_is_active(const cap_t *c) {
    return c->next_level == J1850_VPW_ACTIVE;
}

static void cap_sof(cap_t *c) { cap_pulse(c, J1850_VPW_TV3_NOM); }

/** One bit at the current level, Figure 14. */
static void cap_bit(cap_t *c, bool one) {
    bool active = cap_next_is_active(c);

    cap_pulse(c, (active == one) ? J1850_VPW_TV1_NOM : J1850_VPW_TV2_NOM);
}

static void cap_byte(cap_t *c, uint8_t b) {
    for (int i = 7; i >= 0; i--) {
        cap_bit(c, (b >> i) & 1u);
    }
}

/**
 * Appends bits until the next pulse would be an active one.
 *
 * SOF is active and a byte is an even number of pulses, so after any whole
 * number of bytes the alternation is sitting on a passive slot - which is
 * exactly why EOD is a passive symbol. Reaching an active one, to place a
 * break or a spurious start of frame, therefore costs one bit.
 */
static void cap_to_active_slot(cap_t *c) {
    while (!cap_next_is_active(c)) {
        cap_bit(c, true);
    }
}

/** SOF, the bytes, then the CRC over them. */
static void cap_frame(cap_t *c, const uint8_t *data, size_t len) {
    cap_sof(c);
    for (size_t i = 0; i < len; i++) {
        cap_byte(c, data[i]);
    }
    cap_byte(c, j1850_crc(data, len));
}

/** End of data: passive Tv3, clause 6.6.2.3. */
static void cap_eod(cap_t *c) { cap_pulse(c, J1850_VPW_TV3_NOM); }

/** The responder's normalization bit: active Tv1, or Tv2 to announce a CRC. */
static void cap_nb(cap_t *c, bool with_crc) {
    cap_pulse(c, with_crc ? J1850_VPW_TV2_NOM : J1850_VPW_TV1_NOM);
}

/**
 * How the peripheral really ends a capture: the bus goes quiet, the arming
 * threshold expires, and a zero duration follows what it recorded.
 *
 * A frame ends on an active pulse, so that quiet is normally a passive pulse
 * of its own. When the last thing appended was already passive - a capture cut
 * off mid-byte, say - there is no new pulse: the silence merges into the one
 * already there and the peripheral records it as long. Both shapes appear on
 * a real bus and the difference matters to the decoder, so it is reproduced
 * here rather than papered over.
 */
static void cap_end(cap_t *c) {
    if (c->next_level == J1850_VPW_PASSIVE) {
        cap_pulse(c, J1850_VPW_TV4_NOM);
    } else if (c->pulses) {
        cap_restretch_last(c, J1850_VPW_TV4_NOM);
    }

    /* Room for the zero marker. cap_pulse_at() would count it as a pulse. */
    c->n = (c->pulses + 1) / 2 + 1;
}

/* A reply of the shape a GM module puts on this bus: three header bytes,
 * a mode 01 response, and four data bytes under a CRC. */
static const uint8_t live_reply[] = {0x48, 0x6B, 0x10, 0x41, 0x00,
                                     0xBF, 0xBF, 0xB9, 0x94};

static j1850_vpw_rx_status_t decode(const cap_t *c, j1850_vpw_rx_t *rx) {
    return j1850_vpw_decode(c->sym, c->n, rx);
}

/* ------------------------------------------------------------------ *
 * Encoding
 * ------------------------------------------------------------------ */

TEST(encode_opens_with_a_start_of_frame) {
    rmt_symbol_word_t sym[J1850_VPW_MAX_SYMBOLS];
    const uint8_t data[] = {0x68, 0x6A, 0xF1, 0x01, 0x00, 0x00};

    TEST_ASSERT(j1850_vpw_encode(data, sizeof(data), sym,
                                 sizeof(sym) / sizeof(sym[0])));
    TEST_ASSERT_EQUAL_INT(J1850_VPW_ACTIVE, sym[0].level0);
    TEST_ASSERT_EQUAL_INT(J1850_VPW_TV3_NOM, sym[0].duration0);
}

/** Clause 6.6.2: every symbol is one pulse and the levels strictly alternate,
 *  which in the RMT's layout means active always lands in the first half of a
 *  symbol word and passive always in the second. */
TEST(encode_alternates_active_and_passive_throughout) {
    rmt_symbol_word_t sym[J1850_VPW_MAX_SYMBOLS];
    const uint8_t data[] = {0x00, 0xFF, 0x55, 0xAA};
    size_t n =
        j1850_vpw_encode(data, sizeof(data), sym, sizeof(sym) / sizeof(sym[0]));

    TEST_ASSERT(n > 0);
    for (size_t i = 0; i < n; i++) {
        TEST_ASSERT_MSG(sym[i].level0 == J1850_VPW_ACTIVE,
                        "symbol %u opens passive", (unsigned)i);
        TEST_ASSERT_MSG(sym[i].level1 == J1850_VPW_PASSIVE,
                        "symbol %u closes active", (unsigned)i);
        TEST_ASSERT_MSG(sym[i].duration0 && sym[i].duration1,
                        "symbol %u has a zero length pulse", (unsigned)i);
    }
}

/**
 * Figure 14, both halves of it.
 *
 * 0x55 is 01010101 and its bits fall on alternating levels starting passive,
 * so every one of them is a short pulse; 0xAA is the same argument the other
 * way round and makes every bit a long one. Between them they pin the
 * encoding without a table of expected widths to mistype.
 */
TEST(encode_uses_the_bit_widths_of_figure_14) {
    rmt_symbol_word_t sym[J1850_VPW_MAX_SYMBOLS];
    const uint8_t shorts[] = {0x55, 0x55};
    const uint8_t longs[] = {0xAA, 0xAA};
    size_t n;

    n = j1850_vpw_encode(shorts, sizeof(shorts), sym,
                         sizeof(sym) / sizeof(sym[0]));
    TEST_ASSERT(n > 0);
    /* Pulse 0 is the SOF; pulses 1..16 are the bits, then the trailing EOD. */
    for (size_t p = 1; p <= 8 * sizeof(shorts); p++) {
        uint16_t us = (p & 1u) ? sym[p / 2].duration1 : sym[p / 2].duration0;

        TEST_ASSERT_MSG(us == J1850_VPW_TV1_NOM,
                        "0x55 bit %u is %u us, expected a short pulse",
                        (unsigned)p, us);
    }

    n = j1850_vpw_encode(longs, sizeof(longs), sym,
                         sizeof(sym) / sizeof(sym[0]));
    TEST_ASSERT(n > 0);
    for (size_t p = 1; p <= 8 * sizeof(longs); p++) {
        uint16_t us = (p & 1u) ? sym[p / 2].duration1 : sym[p / 2].duration0;

        TEST_ASSERT_MSG(us == J1850_VPW_TV2_NOM,
                        "0xAA bit %u is %u us, expected a long pulse",
                        (unsigned)p, us);
    }
}

/** The frame ends on an active pulse, so the EOD fills the last symbol word
 *  and the symbol count is exact rather than rounded up. */
TEST(encode_closes_with_an_end_of_data_and_an_exact_symbol_count) {
    rmt_symbol_word_t sym[J1850_VPW_MAX_SYMBOLS];

    for (size_t len = 1; len <= J1850_VPW_MAX_FRAME; len++) {
        uint8_t data[J1850_VPW_MAX_FRAME];
        size_t n;

        memset(data, 0x5A, len);
        n = j1850_vpw_encode(data, len, sym, sizeof(sym) / sizeof(sym[0]));

        TEST_ASSERT_MSG(n == 4 * len + 1, "%u bytes encoded to %u symbols",
                        (unsigned)len, (unsigned)n);
        TEST_ASSERT_EQUAL_INT(J1850_VPW_TV3_NOM, sym[n - 1].duration1);
        TEST_ASSERT_EQUAL_INT(J1850_VPW_PASSIVE, sym[n - 1].level1);
    }
}

TEST(encode_refuses_what_it_cannot_lay_out) {
    rmt_symbol_word_t sym[J1850_VPW_MAX_SYMBOLS];
    uint8_t data[J1850_VPW_MAX_FRAME + 1] = {0};

    TEST_ASSERT_EQUAL_INT(
        0, j1850_vpw_encode(NULL, 4, sym, J1850_VPW_MAX_SYMBOLS));
    TEST_ASSERT_EQUAL_INT(
        0, j1850_vpw_encode(data, 4, NULL, J1850_VPW_MAX_SYMBOLS));
    TEST_ASSERT_EQUAL_INT(
        0, j1850_vpw_encode(data, 0, sym, J1850_VPW_MAX_SYMBOLS));
    /* Clause 7.2.2 caps a message at twelve bytes. */
    TEST_ASSERT_EQUAL_INT(
        0, j1850_vpw_encode(data, sizeof(data), sym, J1850_VPW_MAX_SYMBOLS));
    /* One symbol short of what four bytes need. */
    TEST_ASSERT_EQUAL_INT(0, j1850_vpw_encode(data, 4, sym, 4 * 4));
    TEST_ASSERT(j1850_vpw_encode(data, 4, sym, 4 * 4 + 1) > 0);
}

/* ------------------------------------------------------------------ *
 * Encode and decode agree
 * ------------------------------------------------------------------ */

/**
 * The one test that would catch an inverted bit table: the encoder and the
 * decoder are written from Figure 14 independently, and anything that flipped
 * the sense of a level or a width in one of them shows up here.
 */
TEST(everything_encoded_decodes_back_to_itself) {
    uint32_t seed = 0x1850;

    for (int trial = 0; trial < 400; trial++) {
        rmt_symbol_word_t sym[J1850_VPW_MAX_SYMBOLS + 1];
        uint8_t data[J1850_VPW_MAX_FRAME];
        j1850_vpw_rx_t rx;
        size_t len, n;

        seed = seed * 1103515245u + 12345u;
        len = 2 + (seed >> 16) % (J1850_VPW_MAX_FRAME - 2);

        for (size_t i = 0; i < len - 1; i++) {
            seed = seed * 1103515245u + 12345u;
            data[i] = (uint8_t)(seed >> 16);
        }
        data[len - 1] = j1850_crc(data, len - 1);

        memset(sym, 0, sizeof(sym));
        n = j1850_vpw_encode(data, len, sym, J1850_VPW_MAX_SYMBOLS);
        TEST_ASSERT(n > 0);

        /* The encoder stops at the EOD; a real capture then sees the bus stay
         * passive into an end of frame, which is what closes it. */
        sym[n - 1].duration1 = J1850_VPW_TV4_NOM;

        TEST_ASSERT_MSG(j1850_vpw_decode(sym, n, &rx) == J1850_VPW_RX_OK,
                        "trial %d: %s", trial,
                        j1850_vpw_rx_status_str(rx.status));
        TEST_ASSERT_EQUAL_INT(len, rx.len);
        TEST_ASSERT_EQUAL_MEM(data, rx.data, len);
    }
}

/* ------------------------------------------------------------------ *
 * Decoding a good frame
 * ------------------------------------------------------------------ */

TEST(decode_reads_a_frame_and_checks_its_crc) {
    cap_t c;
    j1850_vpw_rx_t rx;

    cap_reset(&c);
    cap_frame(&c, live_reply, sizeof(live_reply));
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_OK, decode(&c, &rx));
    TEST_ASSERT_EQUAL_INT(sizeof(live_reply) + 1, rx.len);
    TEST_ASSERT_EQUAL_MEM(live_reply, rx.data, sizeof(live_reply));
    TEST_ASSERT_FALSE(rx.eod);
    TEST_ASSERT_EQUAL_INT(0, rx.ifr_len);
}

/** A capture that simply stops - a full buffer, or a truncated copy - is an
 *  end of frame like any other, and what was read stands. */
TEST(decode_treats_a_capture_that_simply_stops_as_an_end_of_frame) {
    cap_t c;
    j1850_vpw_rx_t rx;

    cap_reset(&c);
    cap_frame(&c, live_reply, sizeof(live_reply));
    /* No cap_end(): the frame's last active pulse is the last thing here. */

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_OK, decode(&c, &rx));
    TEST_ASSERT_EQUAL_MEM(live_reply, rx.data, sizeof(live_reply));
}

TEST(decode_skips_whatever_idle_the_capture_opened_with) {
    cap_t c;
    j1850_vpw_rx_t rx;

    cap_reset(&c);
    /* Armed while the bus was already quiet. */
    cap_pulse_at(&c, J1850_VPW_PASSIVE, 900);
    cap_frame(&c, live_reply, sizeof(live_reply));
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_OK, decode(&c, &rx));
    TEST_ASSERT_EQUAL_MEM(live_reply, rx.data, sizeof(live_reply));
}

TEST(decode_rejects_a_frame_whose_crc_does_not_add_up) {
    cap_t c;
    j1850_vpw_rx_t rx;
    uint8_t data[3] = {0x48, 0x6B, 0x10};

    cap_reset(&c);
    cap_sof(&c);
    for (size_t i = 0; i < sizeof(data); i++) {
        cap_byte(&c, data[i]);
    }
    cap_byte(&c, (uint8_t)(j1850_crc(data, sizeof(data)) ^ 0xFF));
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_BAD_CRC, decode(&c, &rx));
    /* The bytes are still handed over: whoever is diagnosing the bus wants to
     * see what arrived, not just that something did not add up. */
    TEST_ASSERT_EQUAL_INT(sizeof(data) + 1, rx.len);
    TEST_ASSERT_EQUAL_MEM(data, rx.data, sizeof(data));
}

TEST(decode_reports_a_frame_with_no_message_under_its_crc) {
    cap_t c;
    j1850_vpw_rx_t rx;

    cap_reset(&c);
    cap_sof(&c);
    cap_byte(&c, 0x92);
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_SHORT, decode(&c, &rx));
}

TEST(decode_reports_a_partial_byte_as_a_framing_error) {
    cap_t c;
    j1850_vpw_rx_t rx;

    cap_reset(&c);
    cap_frame(&c, live_reply, sizeof(live_reply));
    cap_bit(&c, true);
    cap_bit(&c, false);
    cap_bit(&c, true);
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_FRAMING, decode(&c, &rx));
}

/** Clause 7.2.2: twelve message bytes, and the thirteenth has nowhere to go. */
TEST(decode_reports_a_message_past_the_twelve_byte_limit) {
    cap_t c;
    j1850_vpw_rx_t rx;

    cap_reset(&c);
    cap_sof(&c);
    for (int i = 0; i < J1850_VPW_MAX_FRAME + 1; i++) {
        cap_byte(&c, (uint8_t)i);
    }
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_TOO_LONG, decode(&c, &rx));
    TEST_ASSERT_EQUAL_INT(J1850_VPW_MAX_FRAME, rx.len);
}

/** Exactly twelve is not too long. The boundary is worth its own case. */
TEST(decode_accepts_a_message_of_exactly_twelve_bytes) {
    cap_t c;
    j1850_vpw_rx_t rx;
    uint8_t data[J1850_VPW_MAX_FRAME];

    for (size_t i = 0; i < sizeof(data) - 1; i++) {
        data[i] = (uint8_t)(0x30 + i);
    }
    data[sizeof(data) - 1] = j1850_crc(data, sizeof(data) - 1);

    cap_reset(&c);
    cap_sof(&c);
    for (size_t i = 0; i < sizeof(data); i++) {
        cap_byte(&c, data[i]);
    }
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_OK, decode(&c, &rx));
    TEST_ASSERT_EQUAL_INT(J1850_VPW_MAX_FRAME, rx.len);
}

/* ------------------------------------------------------------------ *
 * Symbols that are not bits
 * ------------------------------------------------------------------ */

/** Clause 6.6.2.7: a long active period terminates everything. */
TEST(decode_reports_a_break_that_opens_a_capture) {
    cap_t c;
    j1850_vpw_rx_t rx;

    cap_reset(&c);
    cap_pulse(&c, J1850_VPW_TV5_NOM);
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_BREAK, decode(&c, &rx));
}

TEST(decode_reports_a_break_that_interrupts_a_frame) {
    cap_t c;
    j1850_vpw_rx_t rx;

    cap_reset(&c);
    cap_sof(&c);
    cap_byte(&c, 0x48);
    cap_byte(&c, 0x6B);
    /* A break is active, and a whole byte leaves the alternation passive. */
    cap_to_active_slot(&c);
    cap_pulse(&c, J1850_VPW_TV5_NOM);
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_BREAK, decode(&c, &rx));
}

/** An end of frame closes the capture; a start of frame inside one cannot
 *  happen, because the end of frame would have closed it first. */
TEST(decode_rejects_a_start_of_frame_inside_a_frame) {
    cap_t c;
    j1850_vpw_rx_t rx;

    cap_reset(&c);
    cap_sof(&c);
    cap_byte(&c, 0x48);
    cap_to_active_slot(&c);
    cap_pulse(&c, J1850_VPW_TV3_NOM);
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_BAD_SYMBOL, decode(&c, &rx));
}

TEST(decode_rejects_a_capture_that_never_opened_with_a_start_of_frame) {
    cap_t c;
    j1850_vpw_rx_t rx;

    /* Shorter than Tv1 accepts: the tail of a pulse the capture caught the
     * end of, or noise the glitch filter was not wide enough to stop. */
    cap_reset(&c);
    cap_pulse(&c, J1850_VPW_TV1_RX_MIN - 1);
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_NO_SOF, decode(&c, &rx));
}

TEST(decode_rejects_a_pulse_too_short_to_be_a_symbol) {
    cap_t c;
    j1850_vpw_rx_t rx;

    cap_reset(&c);
    cap_sof(&c);
    cap_byte(&c, 0x48);
    cap_pulse(&c, J1850_VPW_TV1_RX_MIN - 1);
    /* A bit after it, so the capture ends on the pulse after the bad one
     * rather than swallowing it into the trailing silence. */
    cap_pulse(&c, J1850_VPW_TV1_NOM);
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_BAD_SYMBOL, decode(&c, &rx));
}

/**
 * The alternation rule.
 *
 * Two pulses at the same level cannot both be symbols on this bus, and there
 * is no way to tell which of them to believe - a decoder that picked one
 * would invert every bit after it rather than fail where the fault is.
 */
TEST(decode_rejects_two_pulses_at_the_same_level) {
    cap_t c;
    j1850_vpw_rx_t rx;

    cap_reset(&c);
    cap_sof(&c);
    cap_byte(&c, 0x48);
    /* A whole byte leaves the alternation on a passive slot. */
    TEST_ASSERT_FALSE(cap_next_is_active(&c));
    cap_pulse_at(&c, J1850_VPW_ACTIVE, J1850_VPW_TV1_NOM);
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_BAD_SYMBOL, decode(&c, &rx));
}

TEST(decode_reports_nothing_at_all_as_empty) {
    cap_t c;
    j1850_vpw_rx_t rx;

    cap_reset(&c);
    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_EMPTY, j1850_vpw_decode(c.sym, 0, &rx));
    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_EMPTY, j1850_vpw_decode(NULL, 4, &rx));
    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_EMPTY, j1850_vpw_decode(c.sym, 4, NULL));

    /* A capture holding one long passive stretch and nothing else. */
    cap_pulse_at(&c, J1850_VPW_PASSIVE, 900);
    cap_end(&c);
    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_EMPTY, decode(&c, &rx));
}

/* ------------------------------------------------------------------ *
 * In-frame response, clause 5.3.7
 * ------------------------------------------------------------------ */

TEST(decode_reads_an_in_frame_response_after_the_end_of_data) {
    cap_t c;
    j1850_vpw_rx_t rx;
    const uint8_t data[] = {0x48, 0x6B, 0x10};

    cap_reset(&c);
    cap_frame(&c, data, sizeof(data));
    cap_eod(&c);
    cap_nb(&c, false);
    cap_byte(&c, 0xF1);
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_OK, decode(&c, &rx));
    TEST_ASSERT_EQUAL_INT(sizeof(data) + 1, rx.len);
    TEST_ASSERT_EQUAL_MEM(data, rx.data, sizeof(data));
    TEST_ASSERT_TRUE(rx.eod);
    TEST_ASSERT_TRUE(rx.nb);
    TEST_ASSERT_FALSE(rx.nb_long);
    TEST_ASSERT_EQUAL_INT(1, rx.ifr_len);
    TEST_ASSERT_EQUAL_INT(0xF1, rx.ifr[0]);
}

/**
 * Clause 6.6.2.5: a long normalization bit is the responder saying its
 * response carries a CRC. Reported, not acted on - the clause calls it a
 * "preferred method", so a decoder that stripped a byte on the strength of it
 * would corrupt any manufacturer who did it differently.
 */
TEST(decode_reports_a_long_normalization_bit_without_acting_on_it) {
    cap_t c;
    j1850_vpw_rx_t rx;
    const uint8_t data[] = {0x48, 0x6B, 0x10};
    const uint8_t ifr[] = {0xF1, 0x22};

    cap_reset(&c);
    cap_frame(&c, data, sizeof(data));
    cap_eod(&c);
    cap_nb(&c, true);
    cap_byte(&c, ifr[0]);
    cap_byte(&c, ifr[1]);
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_OK, decode(&c, &rx));
    TEST_ASSERT_TRUE(rx.nb_long);
    TEST_ASSERT_EQUAL_INT(sizeof(ifr), rx.ifr_len);
    TEST_ASSERT_EQUAL_MEM(ifr, rx.ifr, sizeof(ifr));
}

/** Several responders each add a byte, clause 5.3.7 c. */
TEST(decode_reads_a_multiple_responder_response_stream) {
    cap_t c;
    j1850_vpw_rx_t rx;
    const uint8_t data[] = {0x48, 0x6B, 0x10};
    const uint8_t ifr[] = {0x10, 0x18, 0x28, 0xF1};

    cap_reset(&c);
    cap_frame(&c, data, sizeof(data));
    cap_eod(&c);
    cap_nb(&c, false);
    for (size_t i = 0; i < sizeof(ifr); i++) {
        cap_byte(&c, ifr[i]);
    }
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_OK, decode(&c, &rx));
    TEST_ASSERT_EQUAL_INT(sizeof(ifr), rx.ifr_len);
    TEST_ASSERT_EQUAL_MEM(ifr, rx.ifr, sizeof(ifr));
}

/**
 * A capture that is nothing but a response.
 *
 * The frame it answers ended with a 200 us end of data, which is long enough
 * to close the capture it arrived in - so on this hardware the response very
 * often arrives on its own, opening with its normalization bit.
 */
TEST(decode_reads_a_capture_that_is_only_a_response) {
    cap_t c;
    j1850_vpw_rx_t rx;

    cap_reset(&c);
    cap_nb(&c, false);
    cap_byte(&c, 0xF1);
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_IFR_ONLY, decode(&c, &rx));
    TEST_ASSERT_EQUAL_INT(0, rx.len);
    TEST_ASSERT_EQUAL_INT(1, rx.ifr_len);
    TEST_ASSERT_EQUAL_INT(0xF1, rx.ifr[0]);
    TEST_ASSERT_TRUE(rx.nb);
}

TEST(decode_reports_a_response_of_no_whole_bytes_as_empty) {
    cap_t c;
    j1850_vpw_rx_t rx;

    cap_reset(&c);
    cap_nb(&c, false);
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_EMPTY, decode(&c, &rx));
}

/** There is no such thing as a response to a response. */
TEST(decode_stops_at_a_second_end_of_data) {
    cap_t c;
    j1850_vpw_rx_t rx;
    const uint8_t data[] = {0x48, 0x6B, 0x10};

    cap_reset(&c);
    cap_frame(&c, data, sizeof(data));
    cap_eod(&c);
    cap_nb(&c, false);
    cap_byte(&c, 0xF1);
    cap_eod(&c);
    cap_nb(&c, false);
    cap_byte(&c, 0x10);
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_OK, decode(&c, &rx));
    TEST_ASSERT_EQUAL_INT(1, rx.ifr_len);
    TEST_ASSERT_EQUAL_INT(0xF1, rx.ifr[0]);
}

TEST(decode_reports_an_end_of_data_mid_byte_as_a_framing_error) {
    cap_t c;
    j1850_vpw_rx_t rx;

    cap_reset(&c);
    cap_sof(&c);
    cap_byte(&c, 0x48);
    cap_bit(&c, true);
    cap_bit(&c, false);
    /* The alternation has to be passive for this to be an end of data. */
    while (cap_next_is_active(&c)) {
        cap_bit(&c, true);
    }
    cap_eod(&c);
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_VPW_RX_FRAMING, decode(&c, &rx));
}

/* ------------------------------------------------------------------ *
 * Table 5, walked to its edges
 * ------------------------------------------------------------------ */

/**
 * Every width a receiver must accept, decoded.
 *
 * Appendix C: "VPW bus symbols allow no forbidden zones between symbols", so
 * every microsecond from Tv1(rx,min) upwards belongs to exactly one window
 * and the windows meet. A frame built entirely of short pulses reads as 0x55
 * bytes and one built entirely of long pulses reads as 0xAA - which follows
 * from Figure 14 alone and needs no table of expected bits - so sweeping the
 * width across a whole window and requiring the same bytes out checks both
 * ends of it at once.
 */
TEST(every_width_inside_tv1_decodes_as_a_short_pulse) {
    for (uint16_t us = J1850_VPW_TV1_RX_MIN; us <= J1850_VPW_TV1_RX_MAX; us++) {
        cap_t c;
        j1850_vpw_rx_t rx;

        cap_reset(&c);
        cap_sof(&c);
        for (int i = 0; i < 16; i++) {
            cap_pulse(&c, us);
        }
        cap_end(&c);

        TEST_ASSERT_MSG(decode(&c, &rx) == J1850_VPW_RX_BAD_CRC, "%u us: %s",
                        us, j1850_vpw_rx_status_str(rx.status));
        TEST_ASSERT_MSG(rx.len == 2 && rx.data[0] == 0x55 && rx.data[1] == 0x55,
                        "%u us decoded to %s", us, td_hex(rx.data, rx.len));
    }
}

TEST(every_width_inside_tv2_decodes_as_a_long_pulse) {
    for (uint16_t us = J1850_VPW_TV2_RX_MIN; us <= J1850_VPW_TV2_RX_MAX; us++) {
        cap_t c;
        j1850_vpw_rx_t rx;

        cap_reset(&c);
        cap_sof(&c);
        for (int i = 0; i < 16; i++) {
            cap_pulse(&c, us);
        }
        cap_end(&c);

        TEST_ASSERT_MSG(decode(&c, &rx) == J1850_VPW_RX_BAD_CRC, "%u us: %s",
                        us, j1850_vpw_rx_status_str(rx.status));
        TEST_ASSERT_MSG(rx.len == 2 && rx.data[0] == 0xAA && rx.data[1] == 0xAA,
                        "%u us decoded to %s", us, td_hex(rx.data, rx.len));
    }
}

TEST(every_width_inside_tv3_opens_a_frame_when_it_is_active) {
    for (uint16_t us = J1850_VPW_TV3_RX_MIN; us <= J1850_VPW_TV3_RX_MAX; us++) {
        cap_t c;
        j1850_vpw_rx_t rx;

        cap_reset(&c);
        cap_pulse(&c, us);
        cap_byte(&c, 0x92);
        cap_byte(&c, j1850_crc((const uint8_t[]){0x92}, 1));
        cap_end(&c);

        TEST_ASSERT_MSG(decode(&c, &rx) == J1850_VPW_RX_OK, "%u us: %s", us,
                        j1850_vpw_rx_status_str(rx.status));
    }
}

TEST(every_width_inside_tv3_ends_the_data_when_it_is_passive) {
    const uint8_t data[] = {0x48, 0x6B, 0x10};

    for (uint16_t us = J1850_VPW_TV3_RX_MIN; us <= J1850_VPW_TV3_RX_MAX; us++) {
        cap_t c;
        j1850_vpw_rx_t rx;

        cap_reset(&c);
        cap_frame(&c, data, sizeof(data));
        cap_pulse(&c, us); /* passive by alternation: an end of data */
        cap_nb(&c, false);
        cap_byte(&c, 0xF1);
        cap_end(&c);

        TEST_ASSERT_MSG(decode(&c, &rx) == J1850_VPW_RX_OK, "%u us: %s", us,
                        j1850_vpw_rx_status_str(rx.status));
        TEST_ASSERT_MSG(rx.eod && rx.ifr_len == 1, "%u us: eod %d, ifr %u", us,
                        rx.eod, rx.ifr_len);
    }
}

/** Above Tv3 the same width is an end of frame when passive and a break when
 *  active - the one place on this bus where the level alone decides. */
TEST(above_tv3_the_level_alone_separates_an_end_of_frame_from_a_break) {
    for (uint16_t us = J1850_VPW_TV4_RX_MIN; us <= J1850_VPW_TV4_RX_MIN + 60;
         us++) {
        cap_t c;
        j1850_vpw_rx_t rx;

        /* Passive: the frame is complete. */
        cap_reset(&c);
        cap_frame(&c, live_reply, sizeof(live_reply));
        cap_pulse(&c, us);
        TEST_ASSERT_MSG(decode(&c, &rx) == J1850_VPW_RX_OK, "%u us passive: %s",
                        us, j1850_vpw_rx_status_str(rx.status));

        /* Active: a break. */
        cap_reset(&c);
        cap_sof(&c);
        cap_byte(&c, 0x48);
        cap_to_active_slot(&c);
        cap_pulse(&c, us);
        TEST_ASSERT_MSG(decode(&c, &rx) == J1850_VPW_RX_BREAK,
                        "%u us active: %s", us,
                        j1850_vpw_rx_status_str(rx.status));
    }
}

/* ------------------------------------------------------------------ *
 * Jitter and noise
 * ------------------------------------------------------------------ */

/**
 * A frame whose every pulse is pushed to the edge of its window still decodes.
 *
 * This is the case a real bus produces: a module at one end of its oscillator
 * tolerance, a network that stretches every edge, and a receiver that has to
 * accept the result.
 */
TEST(a_frame_at_the_edges_of_every_window_still_decodes) {
    uint32_t seed = 0xBEEF;

    for (int trial = 0; trial < 200; trial++) {
        rmt_symbol_word_t sym[J1850_VPW_MAX_SYMBOLS];
        uint8_t data[J1850_VPW_MAX_FRAME];
        j1850_vpw_rx_t rx;
        size_t len, n;

        seed = seed * 1103515245u + 12345u;
        len = 2 + (seed >> 16) % (J1850_VPW_MAX_FRAME - 2);
        for (size_t i = 0; i < len - 1; i++) {
            seed = seed * 1103515245u + 12345u;
            data[i] = (uint8_t)(seed >> 16);
        }
        data[len - 1] = j1850_crc(data, len - 1);

        memset(sym, 0, sizeof(sym));
        n = j1850_vpw_encode(data, len, sym, J1850_VPW_MAX_SYMBOLS);
        TEST_ASSERT(n > 0);

        /* Slam every pulse to whichever end of its window this trial picks.
         * Read and written back rather than aliased: these are bitfields. */
        for (size_t p = 0; p < 2 * n; p++) {
            uint16_t d = (p & 1u) ? sym[p / 2].duration1 : sym[p / 2].duration0;
            bool high;

            seed = seed * 1103515245u + 12345u;
            high = (seed >> 20) & 1u;

            if (d == J1850_VPW_TV1_NOM) {
                d = high ? J1850_VPW_TV1_RX_MAX : J1850_VPW_TV1_RX_MIN;
            } else if (d == J1850_VPW_TV2_NOM) {
                d = high ? J1850_VPW_TV2_RX_MAX : J1850_VPW_TV2_RX_MIN;
            } else if (d == J1850_VPW_TV3_NOM) {
                d = high ? J1850_VPW_TV3_RX_MAX : J1850_VPW_TV3_RX_MIN;
            }

            if (p & 1u) {
                sym[p / 2].duration1 = d;
            } else {
                sym[p / 2].duration0 = d;
            }
        }
        /* The trailing end of data becomes the end of frame that closes it. */
        sym[n - 1].duration1 = J1850_VPW_TV4_NOM;

        TEST_ASSERT_MSG(j1850_vpw_decode(sym, n, &rx) == J1850_VPW_RX_OK,
                        "trial %d: %s", trial,
                        j1850_vpw_rx_status_str(rx.status));
        TEST_ASSERT_EQUAL_MEM(data, rx.data, len);
    }
}

/**
 * Anything at all, and the decoder still terminates with a verdict.
 *
 * The receive interrupt runs this on whatever the peripheral hands over,
 * noise included, so the contract is that it never reads past its input and
 * never returns without having filled the result in.
 */
TEST(arbitrary_captures_always_terminate_with_a_verdict) {
    uint32_t seed = 0xC0FFEE;

    for (int trial = 0; trial < 4000; trial++) {
        rmt_symbol_word_t sym[64];
        j1850_vpw_rx_t rx;
        size_t n;

        seed = seed * 1103515245u + 12345u;
        n = 1 + (seed >> 16) % (sizeof(sym) / sizeof(sym[0]));

        for (size_t i = 0; i < n; i++) {
            seed = seed * 1103515245u + 12345u;
            sym[i].level0 = (seed >> 13) & 1u;
            sym[i].duration0 = (seed >> 16) % 400;
            seed = seed * 1103515245u + 12345u;
            sym[i].level1 = (seed >> 13) & 1u;
            sym[i].duration1 = (seed >> 16) % 400;
        }

        (void)j1850_vpw_decode(sym, n, &rx);

        TEST_ASSERT_MSG(rx.len <= J1850_VPW_MAX_FRAME,
                        "trial %d produced %u data bytes", trial, rx.len);
        TEST_ASSERT_MSG(rx.ifr_len <= J1850_VPW_MAX_FRAME,
                        "trial %d produced %u response bytes", trial,
                        rx.ifr_len);
        /* A frame is only ever delivered when its CRC checks out. */
        if (rx.status == J1850_VPW_RX_OK) {
            TEST_ASSERT_MSG(rx.len >= 2 && j1850_crc_check(rx.data, rx.len),
                            "trial %d delivered %s", trial,
                            td_hex(rx.data, rx.len));
        }
        TEST_ASSERT_NOT_NULL(j1850_vpw_rx_status_str(rx.status));
    }
}

/** Every status has a name; the shell and the target suite print them. */
TEST(every_status_has_a_name) {
    static const j1850_vpw_rx_status_t all[] = {
        J1850_VPW_RX_OK,      J1850_VPW_RX_IFR_ONLY, J1850_VPW_RX_BREAK,
        J1850_VPW_RX_EMPTY,   J1850_VPW_RX_NO_SOF,   J1850_VPW_RX_BAD_SYMBOL,
        J1850_VPW_RX_FRAMING, J1850_VPW_RX_TOO_LONG, J1850_VPW_RX_SHORT,
        J1850_VPW_RX_BAD_CRC,
    };

    for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); i++) {
        const char *name = j1850_vpw_rx_status_str(all[i]);

        TEST_ASSERT_NOT_NULL(name);
        TEST_ASSERT_MSG(strcmp(name, "?") != 0, "status %d has no name",
                        all[i]);
    }
}

/* ------------------------------------------------------------------ *
 * Contention model
 *
 * The pulse model is what the driver watches a transmission against, and its
 * whole value rests on agreeing with the encoder. These are the tests that
 * would catch it drifting.
 * ------------------------------------------------------------------ */

/** The one invariant that matters: the model and the encoder describe the
 *  same waveform, pulse for pulse, for every frame. */
TEST(the_pulse_model_agrees_with_the_encoder) {
    uint32_t seed = 0x6A5F;

    for (int trial = 0; trial < 300; trial++) {
        rmt_symbol_word_t sym[J1850_VPW_MAX_SYMBOLS];
        uint8_t data[J1850_VPW_MAX_FRAME];
        size_t len, n;

        seed = seed * 1103515245u + 12345u;
        len = 1 + (seed >> 16) % J1850_VPW_MAX_FRAME;
        for (size_t i = 0; i < len; i++) {
            seed = seed * 1103515245u + 12345u;
            data[i] = (uint8_t)(seed >> 16);
        }

        n = j1850_vpw_encode(data, len, sym, J1850_VPW_MAX_SYMBOLS);
        TEST_ASSERT(n > 0);
        TEST_ASSERT_EQUAL_INT(2 * n, J1850_VPW_PULSE_COUNT(len));

        for (size_t k = 0; k < J1850_VPW_PULSE_COUNT(len); k++) {
            j1850_vpw_pulse_t p;
            uint8_t level = (k & 1u) ? sym[k / 2].level1 : sym[k / 2].level0;
            uint16_t us =
                (k & 1u) ? sym[k / 2].duration1 : sym[k / 2].duration0;

            TEST_ASSERT_MSG(j1850_vpw_pulse_at(data, len, k, &p),
                            "trial %d: pulse %u of %u refused", trial,
                            (unsigned)k, (unsigned)J1850_VPW_PULSE_COUNT(len));
            TEST_ASSERT_MSG(p.level == level && p.us == us,
                            "trial %d pulse %u: model says %s %u us, "
                            "encoder says %s %u us",
                            trial, (unsigned)k, p.level ? "act" : "psv", p.us,
                            level ? "act" : "psv", us);
        }

        /* And nothing past the end. */
        {
            j1850_vpw_pulse_t p;

            TEST_ASSERT_FALSE(
                j1850_vpw_pulse_at(data, len, J1850_VPW_PULSE_COUNT(len), &p));
        }
    }
}

TEST(the_pulse_model_refuses_what_it_cannot_describe) {
    j1850_vpw_pulse_t p;
    uint8_t data[J1850_VPW_MAX_FRAME + 1] = {0};

    TEST_ASSERT_FALSE(j1850_vpw_pulse_at(NULL, 4, 0, &p));
    TEST_ASSERT_FALSE(j1850_vpw_pulse_at(data, 4, 0, NULL));
    TEST_ASSERT_FALSE(j1850_vpw_pulse_at(data, 0, 0, &p));
    TEST_ASSERT_FALSE(j1850_vpw_pulse_at(data, sizeof(data), 0, &p));
}

/** An active pulse says nothing about contention: a second node driving
 *  active underneath us is indistinguishable from us driving alone. */
TEST(active_pulses_are_never_watched) {
    const uint8_t data[] = {0x68, 0x6A, 0xF1, 0x01, 0x00};

    for (size_t k = 0; k < J1850_VPW_PULSE_COUNT(sizeof(data)); k++) {
        j1850_vpw_pulse_t p;
        uint16_t offs[J1850_VPW_MAX_WATCH];

        TEST_ASSERT(j1850_vpw_pulse_at(data, sizeof(data), k, &p));
        if (p.level == J1850_VPW_ACTIVE) {
            TEST_ASSERT_MSG(j1850_vpw_watch_offsets(&p, 12, 12, offs,
                                                    J1850_VPW_MAX_WATCH) == 0,
                            "pulse %u is active and was given a check",
                            (unsigned)k);
        }
    }
}

/**
 * A short passive pulse is watched once and a long one twice, and the second
 * look falls where a competitor driving the short form would already have
 * taken the bus.
 */
/** A pulse with no room for a guarded sample is not sampled at all. */
TEST(a_pulse_too_short_to_sample_safely_is_left_alone) {
    j1850_vpw_pulse_t shortp = {J1850_VPW_PASSIVE, J1850_VPW_TV1_NOM};
    uint16_t offs[J1850_VPW_MAX_WATCH];

    /* 32 us of guard at each end does not fit in a 64 us pulse. */
    TEST_ASSERT_EQUAL_INT(
        0, j1850_vpw_watch_offsets(&shortp, 32, 32, offs, J1850_VPW_MAX_WATCH));
    TEST_ASSERT_EQUAL_INT(
        1, j1850_vpw_watch_offsets(&shortp, 31, 31, offs, J1850_VPW_MAX_WATCH));
}

TEST(a_long_passive_pulse_is_watched_twice_and_a_short_one_once) {
    j1850_vpw_pulse_t shortp = {J1850_VPW_PASSIVE, J1850_VPW_TV1_NOM};
    j1850_vpw_pulse_t longp = {J1850_VPW_PASSIVE, J1850_VPW_TV2_NOM};
    uint16_t offs[J1850_VPW_MAX_WATCH];

    TEST_ASSERT_EQUAL_INT(
        1, j1850_vpw_watch_offsets(&shortp, 12, 12, offs, J1850_VPW_MAX_WATCH));
    TEST_ASSERT_EQUAL_INT(12, offs[0]);

    TEST_ASSERT_EQUAL_INT(
        2, j1850_vpw_watch_offsets(&longp, 12, 12, offs, J1850_VPW_MAX_WATCH));
    TEST_ASSERT_EQUAL_INT(12, offs[0]);
    TEST_ASSERT_EQUAL_INT(J1850_VPW_TV1_NOM + 12, offs[1]);
}

/** Every check has to land inside the pulse it belongs to. A check past the
 *  end would sample a phase nobody asked about and invent a collision. */
TEST(no_check_ever_falls_outside_its_own_pulse) {
    const uint8_t data[] = {0x00, 0xFF, 0x55, 0xAA, 0x5A};

    for (uint16_t guard = 0; guard <= 40; guard++) {
        for (size_t k = 0; k < J1850_VPW_PULSE_COUNT(sizeof(data)); k++) {
            j1850_vpw_pulse_t p;
            uint16_t offs[J1850_VPW_MAX_WATCH];
            size_t n;

            TEST_ASSERT(j1850_vpw_pulse_at(data, sizeof(data), k, &p));
            n = j1850_vpw_watch_offsets(&p, guard, guard, offs,
                                        J1850_VPW_MAX_WATCH);

            for (size_t i = 0; i < n; i++) {
                /* Clear of both edges, not merely inside the pulse: the far
                 * end is where the wire falls short of the nominal width. */
                TEST_ASSERT_MSG(offs[i] >= guard && offs[i] + guard <= p.us,
                                "guard %u, pulse %u (%u us): check at %u us "
                                "does not clear both edges",
                                guard, (unsigned)k, p.us, offs[i]);
                if (i) {
                    TEST_ASSERT_MSG(offs[i] > offs[i - 1],
                                    "guard %u pulse %u: checks out of order",
                                    guard, (unsigned)k);
                }
            }
        }
    }
}

TEST(watch_offsets_respects_the_caller_s_capacity) {
    j1850_vpw_pulse_t longp = {J1850_VPW_PASSIVE, J1850_VPW_TV2_NOM};
    uint16_t one[1];

    TEST_ASSERT_EQUAL_INT(1, j1850_vpw_watch_offsets(&longp, 12, 12, one, 1));
    TEST_ASSERT_EQUAL_INT(0, j1850_vpw_watch_offsets(&longp, 12, 12, one, 0));
    TEST_ASSERT_EQUAL_INT(0, j1850_vpw_watch_offsets(NULL, 12, 12, one, 1));
}
