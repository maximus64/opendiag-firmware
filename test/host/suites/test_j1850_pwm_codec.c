/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_j1850_pwm_codec.c
 * @brief SAE J1850 PWM symbol coding, framing and CRC.
 *
 * The driver's protocol half runs here in full: every timing window in
 * Table 3 is walked to its edges, the CRC is checked against the vectors
 * printed in the standard, and a pseudo-random sweep pushes jittered,
 * truncated and corrupted captures through the decoder to show it always
 * terminates with a verdict instead of reading past its input.
 *
 * Clause references are to SAE J1850 rev. FEB1994.
 */

#include <string.h>

#include "j1850_pwm_codec.h"
#include "td_test.h"

/* ------------------------------------------------------------------ *
 * Building captures
 * ------------------------------------------------------------------ */

/** A capture under construction, in the peripheral's own symbol layout. */
typedef struct {
    rmt_symbol_word_t sym[256];
    size_t n;
} cap_t;

static void cap_reset(cap_t *c) { memset(c, 0, sizeof(*c)); }

/** Appends one active/passive pair, the shape every J1850 PWM symbol has. */
static void cap_cell(cap_t *c, uint16_t active_us, uint16_t passive_us) {
    c->sym[c->n].level0 = J1850_PWM_ACTIVE;
    c->sym[c->n].duration0 = active_us;
    c->sym[c->n].level1 = J1850_PWM_PASSIVE;
    c->sym[c->n].duration1 = passive_us;
    c->n++;
}

/** SOF: active Tp7, next rising edge Tp4 after this one. */
static void cap_sof(cap_t *c) {
    cap_cell(c, J1850_PWM_TP7_NOM, J1850_PWM_TP4_NOM - J1850_PWM_TP7_NOM);
}

static void cap_bit(cap_t *c, bool one) {
    uint16_t active = one ? J1850_PWM_TP1_NOM : J1850_PWM_TP2_NOM;

    cap_cell(c, active, (uint16_t)(J1850_PWM_TP3_NOM - active));
}

static void cap_byte(cap_t *c, uint8_t b) {
    for (int i = 7; i >= 0; i--) {
        cap_bit(c, (b >> i) & 1u);
    }
}

/** Bytes plus their CRC, preceded by a SOF: a whole well formed frame. */
static void cap_frame(cap_t *c, const uint8_t *data, size_t len) {
    cap_sof(c);
    for (size_t i = 0; i < len; i++) {
        cap_byte(c, data[i]);
    }
    cap_byte(c, j1850_crc(data, len));
}

/**
 * @brief Stretches the last cell's passive phase so the next edge lands at
 *        Tp4 rather than Tp3 - the EOD that opens an in-frame response.
 */
static void cap_eod(cap_t *c) {
    rmt_symbol_word_t *last = &c->sym[c->n - 1];

    last->duration1 = (uint16_t)(J1850_PWM_TP4_NOM - last->duration0);
}

/** Closes a capture the way the peripheral does, with a zero duration. */
static void cap_end(cap_t *c) {
    c->sym[c->n].level0 = J1850_PWM_PASSIVE;
    c->sym[c->n].duration0 = 0;
    c->sym[c->n].duration1 = 0;
    c->n++;
}

/**
 * @brief Closes a capture the way the hardware actually does.
 *
 * The RMT ends a capture on the pulse that outlasted its arming threshold and
 * writes a zero where that pulse's other half would go, so a real frame's
 * final symbol is an active phase followed by nothing - not a symbol of its
 * own. Captures taken from a module on the bench end exactly like this.
 */
static void cap_end_on_hardware_boundary(cap_t *c) {
    c->sym[c->n - 1].duration1 = 0;
}

static j1850_pwm_rx_status_t decode(cap_t *c, j1850_pwm_rx_t *out) {
    cap_end(c);
    return j1850_pwm_decode(c->sym, c->n, out);
}

/* A mode 01 PID 00 reply, as captured from a Ford module on the bench. */
static const uint8_t live_reply[] = {0x41, 0x6B, 0x10, 0x41, 0x00,
                                     0xBF, 0x9F, 0xB9, 0x90};

/* ------------------------------------------------------------------ *
 * CRC, clause 5.4.1
 * ------------------------------------------------------------------ */

/**
 * Table 1 of the standard prints seven worked examples. They are the only
 * external check that exists on this polynomial, seed and final inversion, so
 * all seven are here rather than a representative one.
 */
TEST(crc_matches_every_vector_in_table_1) {
    static const struct {
        uint8_t data[9];
        size_t len;
        uint8_t crc;
    } vectors[] = {
        {{0x00, 0x00, 0x00, 0x00}, 4, 0x59},
        {{0xF2, 0x01, 0x83}, 3, 0x37},
        {{0x0F, 0xAA, 0x00, 0x55}, 4, 0x79},
        {{0x00, 0xFF, 0x55, 0x11}, 4, 0xB8},
        {{0x33, 0x22, 0x55, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}, 9, 0xCB},
        {{0x92, 0x6B, 0x55}, 3, 0x8C},
        {{0xFF, 0xFF, 0xFF, 0xFF}, 4, 0x74},
    };

    for (size_t i = 0; i < sizeof(vectors) / sizeof(vectors[0]); i++) {
        uint8_t got = j1850_crc(vectors[i].data, vectors[i].len);

        TEST_ASSERT_MSG(got == vectors[i].crc,
                        "vector %u: expected CRC 0x%02X, got 0x%02X",
                        (unsigned)i, vectors[i].crc, got);
    }
}

/**
 * Clause 5.4.1 g: running the checker over a frame *including* its CRC byte
 * leaves the constant 0xC4 whatever the frame said. That is the property a
 * hardware checker relies on, and it is a stronger statement than any single
 * vector - it has to hold for every frame the encoder can produce.
 */
TEST(crc_over_a_whole_frame_leaves_the_c4_residue) {
    uint8_t frame[6] = {0x61, 0x6A, 0xF1, 0x01, 0x00, 0x00};

    for (unsigned v = 0; v < 256; v++) {
        uint8_t crc = 0xFF;

        frame[4] = (uint8_t)v;
        frame[5] = j1850_crc(frame, 5);

        /* The checker is the generator without the final inversion. */
        for (size_t i = 0; i < sizeof(frame); i++) {
            crc ^= frame[i];
            for (int b = 0; b < 8; b++) {
                crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D)
                                   : (uint8_t)(crc << 1);
            }
        }

        TEST_ASSERT_MSG(crc == J1850_CRC_RESIDUE,
                        "byte 0x%02X: residue 0x%02X, expected 0x%02X", v, crc,
                        J1850_CRC_RESIDUE);
    }
}

TEST(crc_check_rejects_a_frame_with_no_room_for_a_message) {
    uint8_t one_byte[1] = {0x59};

    TEST_ASSERT_FALSE(j1850_crc_check(one_byte, 1));
    TEST_ASSERT_FALSE(j1850_crc_check(one_byte, 0));
    TEST_ASSERT_FALSE(j1850_crc_check(NULL, 4));
}

TEST(crc_check_accepts_the_frame_the_bench_module_sent) {
    uint8_t frame[sizeof(live_reply) + 1];

    memcpy(frame, live_reply, sizeof(live_reply));
    frame[sizeof(live_reply)] = 0x05;

    TEST_ASSERT_TRUE(j1850_crc_check(frame, sizeof(frame)));
}

/* ------------------------------------------------------------------ *
 * Encoding
 * ------------------------------------------------------------------ */

TEST(encode_emits_a_sof_then_one_symbol_per_bit) {
    uint8_t data[3] = {0x61, 0x6A, 0xF1};
    rmt_symbol_word_t sym[J1850_PWM_MAX_SYMBOLS];
    size_t n = j1850_pwm_encode(data, sizeof(data), true, sym,
                                sizeof(sym) / sizeof(sym[0]));

    TEST_ASSERT_EQUAL_INT(1 + 3 * 8, n);

    /* SOF: active Tp7 less the driver stage's stretch, and the next rising
     * edge Tp4 after this one - the cell length carries no trim. */
    TEST_ASSERT_EQUAL_INT(J1850_PWM_ACTIVE, sym[0].level0);
    TEST_ASSERT_EQUAL_INT(J1850_PWM_TX_TP7, sym[0].duration0);
    TEST_ASSERT_EQUAL_INT(J1850_PWM_PASSIVE, sym[0].level1);
    TEST_ASSERT_EQUAL_INT(J1850_PWM_TP4_NOM,
                          sym[0].duration0 + sym[0].duration1);
}

TEST(encode_sends_the_most_significant_bit_first) {
    uint8_t data[1] = {0x80}; /* one, then seven zeroes */
    rmt_symbol_word_t sym[J1850_PWM_MAX_SYMBOLS];

    TEST_ASSERT_EQUAL_INT(8, j1850_pwm_encode(data, 1, false, sym, 8));

    TEST_ASSERT_EQUAL_INT(J1850_PWM_TX_TP1, sym[0].duration0);
    for (int i = 1; i < 8; i++) {
        TEST_ASSERT_EQUAL_INT(J1850_PWM_TX_TP2, sym[i].duration0);
    }
}

TEST(encode_holds_every_bit_cell_to_the_nominal_bit_time) {
    uint8_t data[2] = {0xA5, 0x5A};
    rmt_symbol_word_t sym[J1850_PWM_MAX_SYMBOLS];
    size_t n =
        j1850_pwm_encode(data, sizeof(data), true, sym, J1850_PWM_MAX_SYMBOLS);

    for (size_t i = 1; i < n; i++) {
        TEST_ASSERT_MSG(sym[i].duration0 + sym[i].duration1 ==
                            J1850_PWM_TP3_NOM,
                        "symbol %u spans %u us, expected %u", (unsigned)i,
                        sym[i].duration0 + sym[i].duration1, J1850_PWM_TP3_NOM);
        TEST_ASSERT_EQUAL_INT(J1850_PWM_ACTIVE, sym[i].level0);
        TEST_ASSERT_EQUAL_INT(J1850_PWM_PASSIVE, sym[i].level1);
    }
}

/**
 * Table 3's transmit column constrains what appears *on the bus*: the note
 * under it says the tolerances already include "physical layer delays (i.e.,
 * turn-on and turn-off delays)". So the widths to check are the emitted ones
 * plus what this board's driver stage adds to them, not the emitted ones
 * alone - and they should land on the nominal, not against a window edge,
 * because a transmitter with no margin left is one bus load away from being
 * out of spec.
 */
TEST(what_reaches_the_bus_is_the_nominal_width_of_every_symbol) {
    uint8_t data[J1850_PWM_MAX_FRAME];
    rmt_symbol_word_t sym[J1850_PWM_MAX_SYMBOLS];
    size_t n;

    for (size_t i = 0; i < sizeof(data); i++) {
        data[i] = (uint8_t)(i * 37 + 1);
    }

    n = j1850_pwm_encode(data, sizeof(data), true, sym, J1850_PWM_MAX_SYMBOLS);
    TEST_ASSERT_EQUAL_INT(1 + 8 * J1850_PWM_MAX_FRAME, n);

    /* SOF: Tp7 active, and the next rising edge Tp4 later. */
    TEST_ASSERT_EQUAL_INT(J1850_PWM_TP7_NOM,
                          sym[0].duration0 + J1850_PWM_TX_TRIM_US);
    TEST_ASSERT_EQUAL_INT(J1850_PWM_TP4_NOM,
                          sym[0].duration0 + sym[0].duration1);

    for (size_t i = 1; i < n; i++) {
        unsigned wire = sym[i].duration0 + J1850_PWM_TX_TRIM_US;
        unsigned cell = sym[i].duration0 + sym[i].duration1;

        TEST_ASSERT_MSG(wire == J1850_PWM_TP1_NOM || wire == J1850_PWM_TP2_NOM,
                        "symbol %u puts %u us on the bus, expected Tp1 (%u) or "
                        "Tp2 (%u)",
                        (unsigned)i, wire, J1850_PWM_TP1_NOM,
                        J1850_PWM_TP2_NOM);
        TEST_ASSERT_MSG(cell == J1850_PWM_TP3_NOM,
                        "symbol %u spans %u us, expected Tp3 (%u)", (unsigned)i,
                        cell, J1850_PWM_TP3_NOM);
    }
}

/**
 * The same widths against the windows themselves, which is the check that
 * survives someone changing the trim for a different driver stage. Stated as
 * the standard states it, so a future board only has to make this pass.
 */
TEST(what_reaches_the_bus_stays_inside_the_transmit_windows_of_table_3) {
    uint8_t data[J1850_PWM_MAX_FRAME];
    rmt_symbol_word_t sym[J1850_PWM_MAX_SYMBOLS];
    unsigned sof, sof_cell;
    size_t n;

    for (size_t i = 0; i < sizeof(data); i++) {
        data[i] = (uint8_t)(i * 91 + 7);
    }
    n = j1850_pwm_encode(data, sizeof(data), true, sym, J1850_PWM_MAX_SYMBOLS);

    sof = sym[0].duration0 + J1850_PWM_TX_TRIM_US;
    sof_cell = sym[0].duration0 + sym[0].duration1;

    TEST_ASSERT_MSG(sof >= J1850_PWM_TP7_TX_MIN && sof <= J1850_PWM_TP7_TX_MAX,
                    "SOF puts %u us on the bus, Tp7 is %u..%u", sof,
                    J1850_PWM_TP7_TX_MIN, J1850_PWM_TP7_TX_MAX);
    TEST_ASSERT_MSG(sof_cell >= J1850_PWM_TP4_TX_MIN &&
                        sof_cell <= J1850_PWM_TP4_TX_MAX,
                    "SOF to first bit is %u us, Tp4 is %u..%u", sof_cell,
                    J1850_PWM_TP4_TX_MIN, J1850_PWM_TP4_TX_MAX);

    for (size_t i = 1; i < n; i++) {
        unsigned wire = sym[i].duration0 + J1850_PWM_TX_TRIM_US;
        unsigned cell = sym[i].duration0 + sym[i].duration1;
        bool one = wire < J1850_PWM_BIT_SPLIT;
        unsigned lo = one ? J1850_PWM_TP1_TX_MIN : J1850_PWM_TP2_TX_MIN;
        unsigned hi = one ? J1850_PWM_TP1_TX_MAX : J1850_PWM_TP2_TX_MAX;

        TEST_ASSERT_MSG(wire >= lo && wire <= hi,
                        "symbol %u puts %u us on the bus, window is %u..%u",
                        (unsigned)i, wire, lo, hi);
        TEST_ASSERT_MSG(cell >= J1850_PWM_TP3_TX_MIN &&
                            cell <= J1850_PWM_TP3_TX_MAX,
                        "symbol %u spans %u us, Tp3 is %u..%u", (unsigned)i,
                        cell, J1850_PWM_TP3_TX_MIN, J1850_PWM_TP3_TX_MAX);
    }
}

/**
 * Whatever the trim, two rising edges are never closer than Tp3 - clause
 * 6.6.1.1 says they "shall never" be - and the trim must never eat a pulse.
 */
TEST(the_transmit_trim_cannot_break_the_rising_edge_cadence) {
    uint8_t data[2] = {0x00, 0xFF};
    rmt_symbol_word_t sym[J1850_PWM_MAX_SYMBOLS];
    size_t n =
        j1850_pwm_encode(data, sizeof(data), true, sym, J1850_PWM_MAX_SYMBOLS);

    for (size_t i = 0; i < n; i++) {
        TEST_ASSERT_MSG(sym[i].duration0 > 0,
                        "symbol %u has no active phase at all", (unsigned)i);
        TEST_ASSERT_MSG(sym[i].duration1 > 0,
                        "symbol %u has no passive phase at all", (unsigned)i);
        TEST_ASSERT_MSG(
            sym[i].duration0 + sym[i].duration1 >= J1850_PWM_TP3_TX_MIN,
            "symbol %u puts two rising edges %u us apart, under Tp3",
            (unsigned)i, sym[i].duration0 + sym[i].duration1);
    }
}

TEST(encode_refuses_what_it_cannot_fit_or_send) {
    uint8_t data[J1850_PWM_MAX_FRAME + 1] = {0};
    rmt_symbol_word_t sym[J1850_PWM_MAX_SYMBOLS];

    /* Past the twelve byte message limit of clause 7.2.1. */
    TEST_ASSERT_EQUAL_INT(0,
                          j1850_pwm_encode(data, J1850_PWM_MAX_FRAME + 1, true,
                                           sym, J1850_PWM_MAX_SYMBOLS));
    /* Nothing to send. */
    TEST_ASSERT_EQUAL_INT(
        0, j1850_pwm_encode(data, 0, true, sym, J1850_PWM_MAX_SYMBOLS));
    /* One symbol short of what a SOF plus a byte needs. */
    TEST_ASSERT_EQUAL_INT(0, j1850_pwm_encode(data, 1, true, sym, 8));
    TEST_ASSERT_EQUAL_INT(9, j1850_pwm_encode(data, 1, true, sym, 9));
    /* Without a SOF the same byte fits in eight. */
    TEST_ASSERT_EQUAL_INT(8, j1850_pwm_encode(data, 1, false, sym, 8));
    TEST_ASSERT_EQUAL_INT(0, j1850_pwm_encode(NULL, 1, true, sym, 32));
    TEST_ASSERT_EQUAL_INT(0, j1850_pwm_encode(data, 1, true, NULL, 32));
}

/* ------------------------------------------------------------------ *
 * Decoding: the happy path
 * ------------------------------------------------------------------ */

TEST(decode_reads_back_what_encode_wrote) {
    uint8_t data[6] = {0x61, 0x6A, 0xF1, 0x01, 0x00, 0x00};
    rmt_symbol_word_t sym[J1850_PWM_MAX_SYMBOLS + 1];
    j1850_pwm_rx_t rx;
    size_t n;

    data[5] = j1850_crc(data, 5);

    n = j1850_pwm_encode(data, sizeof(data), true, sym, J1850_PWM_MAX_SYMBOLS);
    memset(&sym[n], 0, sizeof(sym[0])); /* the peripheral's end marker */

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_OK, j1850_pwm_decode(sym, n + 1, &rx));
    TEST_ASSERT_EQUAL_INT(sizeof(data), rx.len);
    TEST_ASSERT_EQUAL_MEM(data, rx.data, sizeof(data));
    TEST_ASSERT_EQUAL_INT(0, rx.ifr_len);
    TEST_ASSERT_FALSE(rx.eod);
}

/** Every byte value, through both halves of the codec. */
TEST(decode_round_trips_all_256_byte_values) {
    for (unsigned v = 0; v < 256; v++) {
        uint8_t data[3] = {0x41, (uint8_t)v, 0};
        rmt_symbol_word_t sym[J1850_PWM_MAX_SYMBOLS + 1];
        j1850_pwm_rx_t rx;
        size_t n;

        data[2] = j1850_crc(data, 2);
        n = j1850_pwm_encode(data, sizeof(data), true, sym,
                             J1850_PWM_MAX_SYMBOLS);
        memset(&sym[n], 0, sizeof(sym[0]));

        TEST_ASSERT_MSG(j1850_pwm_decode(sym, n + 1, &rx) == J1850_PWM_RX_OK,
                        "byte 0x%02X did not round trip: %s", v,
                        j1850_pwm_rx_status_str(rx.status));
        TEST_ASSERT_EQUAL_MEM(data, rx.data, sizeof(data));
    }
}

TEST(decode_handles_a_full_length_twelve_byte_frame) {
    uint8_t data[J1850_PWM_MAX_FRAME];
    cap_t c;
    j1850_pwm_rx_t rx;

    for (size_t i = 0; i < sizeof(data) - 1; i++) {
        data[i] = (uint8_t)(0x11 * i);
    }
    data[sizeof(data) - 1] = j1850_crc(data, sizeof(data) - 1);

    cap_reset(&c);
    cap_sof(&c);
    for (size_t i = 0; i < sizeof(data); i++) {
        cap_byte(&c, data[i]);
    }

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_OK, decode(&c, &rx));
    TEST_ASSERT_EQUAL_INT(J1850_PWM_MAX_FRAME, rx.len);
    TEST_ASSERT_EQUAL_MEM(data, rx.data, sizeof(data));
}

TEST(decode_ignores_idle_before_the_start_of_frame) {
    cap_t c;
    j1850_pwm_rx_t rx;

    cap_reset(&c);
    /* A capture that opened mid-idle: a passive stretch with no rising edge. */
    c.sym[c.n].level0 = J1850_PWM_PASSIVE;
    c.sym[c.n].duration0 = 5000;
    c.sym[c.n].level1 = J1850_PWM_PASSIVE;
    c.sym[c.n].duration1 = 5000;
    c.n++;
    cap_frame(&c, live_reply, sizeof(live_reply));

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_OK, decode(&c, &rx));
    TEST_ASSERT_EQUAL_INT(sizeof(live_reply) + 1, rx.len);
    TEST_ASSERT_EQUAL_MEM(live_reply, rx.data, sizeof(live_reply));
}

/* ------------------------------------------------------------------ *
 * Decoding: timing windows
 * ------------------------------------------------------------------ */

/**
 * Table 3 gives the receiver a window per symbol. Walk each one edge to edge,
 * and one microsecond outside each edge, and check the verdict flips exactly
 * where the standard says it should.
 */
TEST(decode_accepts_every_active_width_inside_the_bit_windows) {
    for (unsigned us = J1850_PWM_TP1_RX_MIN; us <= J1850_PWM_TP2_RX_MAX; us++) {
        cap_t c;
        j1850_pwm_rx_t rx;
        bool expect_one = us < J1850_PWM_BIT_SPLIT;
        uint8_t expect = expect_one ? 0x80 : 0x00;
        uint8_t data[2];

        data[0] = expect;
        data[1] = j1850_crc(data, 1);

        cap_reset(&c);
        cap_sof(&c);
        /* First bit at the width under test, the rest nominal. */
        cap_cell(&c, (uint16_t)us, (uint16_t)(J1850_PWM_TP3_NOM - us));
        for (int i = 6; i >= 0; i--) {
            cap_bit(&c, (expect >> i) & 1u);
        }
        cap_byte(&c, data[1]);

        TEST_ASSERT_MSG(decode(&c, &rx) == J1850_PWM_RX_OK, "active %u us: %s",
                        us, j1850_pwm_rx_status_str(rx.status));
        TEST_ASSERT_MSG(rx.data[0] == expect,
                        "active %u us decoded to 0x%02X, expected 0x%02X", us,
                        rx.data[0], expect);
    }
}

TEST(decode_rejects_active_widths_between_the_bit_and_sof_windows) {
    /* Tp2(max) is 18 and Tp7(min) is 27; the gap belongs to no symbol. */
    for (unsigned us = J1850_PWM_TP2_RX_MAX + 1; us < J1850_PWM_TP7_RX_MIN;
         us++) {
        cap_t c;
        j1850_pwm_rx_t rx;

        cap_reset(&c);
        cap_sof(&c);
        cap_cell(&c, (uint16_t)us, (uint16_t)2);
        cap_byte(&c, 0x00);

        TEST_ASSERT_MSG(decode(&c, &rx) == J1850_PWM_RX_BAD_SYMBOL,
                        "active %u us should be no symbol, got %s", us,
                        j1850_pwm_rx_status_str(rx.status));
    }
}

TEST(decode_rejects_an_active_pulse_shorter_than_tp1_min) {
    for (unsigned us = 1; us < J1850_PWM_TP1_RX_MIN; us++) {
        cap_t c;
        j1850_pwm_rx_t rx;

        cap_reset(&c);
        cap_sof(&c);
        cap_cell(&c, (uint16_t)us, (uint16_t)(J1850_PWM_TP3_NOM - us));
        cap_byte(&c, 0x00);

        TEST_ASSERT_MSG(decode(&c, &rx) == J1850_PWM_RX_BAD_SYMBOL,
                        "active %u us should be too short, got %s", us,
                        j1850_pwm_rx_status_str(rx.status));
    }
}

TEST(decode_accepts_a_start_of_frame_across_its_whole_window) {
    for (unsigned us = J1850_PWM_TP7_RX_MIN; us <= J1850_PWM_TP7_RX_MAX; us++) {
        cap_t c;
        j1850_pwm_rx_t rx;

        cap_reset(&c);
        /* Whatever Tp7 is, the first data bit still starts Tp4 in. */
        cap_cell(&c, (uint16_t)us, (uint16_t)(J1850_PWM_TP4_NOM - us));
        cap_byte(&c, 0x92);
        cap_byte(&c, j1850_crc((const uint8_t[]){0x92}, 1));

        TEST_ASSERT_MSG(decode(&c, &rx) == J1850_PWM_RX_OK, "SOF of %u us: %s",
                        us, j1850_pwm_rx_status_str(rx.status));
    }
}

TEST(decode_reports_a_break_symbol) {
    for (unsigned us = J1850_PWM_TP8_RX_MIN; us <= J1850_PWM_TP8_RX_MAX; us++) {
        cap_t c;
        j1850_pwm_rx_t rx;

        cap_reset(&c);
        cap_cell(&c, (uint16_t)us, (uint16_t)J1850_PWM_TP9_NOM);

        TEST_ASSERT_MSG(decode(&c, &rx) == J1850_PWM_RX_BREAK,
                        "BRK of %u us: %s", us,
                        j1850_pwm_rx_status_str(rx.status));
    }
}

TEST(decode_rejects_an_opening_pulse_that_matches_no_symbol) {
    cap_t c;
    j1850_pwm_rx_t rx;

    cap_reset(&c);
    cap_cell(&c, 200, 300); /* a VPW start of frame, on the wrong bus */

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_NO_SOF, decode(&c, &rx));
}

/**
 * Clause 6.6.1.8: a cell is measured rising edge to rising edge, so a cell
 * whose *total* falls outside Tp3 is bad even when both its pulses look fine
 * on their own. This is the case a decoder that only measures active widths
 * gets wrong.
 */
TEST(decode_rejects_a_cell_whose_edges_are_spaced_wrong) {
    static const unsigned bad_cells[] = {15, 20, 28, 41};

    for (size_t i = 0; i < sizeof(bad_cells) / sizeof(bad_cells[0]); i++) {
        unsigned cell = bad_cells[i];
        cap_t c;
        j1850_pwm_rx_t rx;

        cap_reset(&c);
        cap_sof(&c);
        cap_bit(&c, false);
        /* Active is a perfectly good Tp1; only the spacing is wrong. */
        cap_cell(&c, J1850_PWM_TP1_NOM, (uint16_t)(cell - J1850_PWM_TP1_NOM));
        cap_byte(&c, 0x00);

        TEST_ASSERT_MSG(decode(&c, &rx) == J1850_PWM_RX_BAD_TIMING,
                        "%u us between rising edges: %s", cell,
                        j1850_pwm_rx_status_str(rx.status));
    }
}

TEST(decode_accepts_bit_cells_across_the_whole_tp3_window) {
    for (unsigned cell = J1850_PWM_TP3_RX_MIN; cell <= J1850_PWM_TP3_RX_MAX;
         cell++) {
        cap_t c;
        j1850_pwm_rx_t rx;
        uint8_t data[2] = {0xC3, 0};

        data[1] = j1850_crc(data, 1);

        cap_reset(&c);
        cap_sof(&c);
        for (int i = 7; i >= 0; i--) {
            uint16_t a =
                ((data[0] >> i) & 1u) ? J1850_PWM_TP1_NOM : J1850_PWM_TP2_NOM;

            cap_cell(&c, a, (uint16_t)(cell - a));
        }
        cap_byte(&c, data[1]);

        TEST_ASSERT_MSG(decode(&c, &rx) == J1850_PWM_RX_OK,
                        "bit cell of %u us: %s", cell,
                        j1850_pwm_rx_status_str(rx.status));
    }
}

/** The first data bit follows the SOF's rising edge by Tp4, not Tp3. */
TEST(decode_requires_the_first_data_bit_at_tp4_after_the_sof) {
    for (unsigned gap = J1850_PWM_TP4_RX_MIN; gap <= J1850_PWM_TP4_RX_MAX;
         gap++) {
        cap_t c;
        j1850_pwm_rx_t rx;

        cap_reset(&c);
        cap_cell(&c, J1850_PWM_TP7_NOM, (uint16_t)(gap - J1850_PWM_TP7_NOM));
        cap_byte(&c, 0x92);
        cap_byte(&c, 0xEA); /* CRC of 0x92 */

        TEST_ASSERT_MSG(decode(&c, &rx) == J1850_PWM_RX_OK,
                        "SOF to first bit %u us: %s", gap,
                        j1850_pwm_rx_status_str(rx.status));
    }

    /* Tp3 spacing after a SOF is not a frame. */
    {
        cap_t c;
        j1850_pwm_rx_t rx;

        cap_reset(&c);
        cap_cell(&c, J1850_PWM_TP7_NOM, 4);
        cap_byte(&c, 0x92);

        TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_BAD_TIMING, decode(&c, &rx));
    }
}

/* ------------------------------------------------------------------ *
 * Decoding: framing rules
 * ------------------------------------------------------------------ */

TEST(decode_rejects_a_frame_that_does_not_end_on_a_byte_boundary) {
    cap_t c;
    j1850_pwm_rx_t rx;

    cap_reset(&c);
    cap_sof(&c);
    cap_byte(&c, 0x61);
    cap_bit(&c, true);
    cap_bit(&c, false);
    cap_bit(&c, true); /* three bits into the second byte, then nothing */

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_FRAMING, decode(&c, &rx));
}

TEST(decode_rejects_a_frame_past_the_twelve_byte_message_limit) {
    cap_t c;
    j1850_pwm_rx_t rx;

    cap_reset(&c);
    cap_sof(&c);
    for (int i = 0; i <= J1850_PWM_MAX_FRAME; i++) {
        cap_byte(&c, (uint8_t)i);
    }

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_TOO_LONG, decode(&c, &rx));
}

TEST(decode_rejects_a_single_byte_frame_as_having_no_message) {
    cap_t c;
    j1850_pwm_rx_t rx;

    cap_reset(&c);
    cap_sof(&c);
    cap_byte(&c, 0x59);

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_SHORT, decode(&c, &rx));
}

TEST(decode_reports_a_bad_crc_but_still_shows_the_bytes) {
    cap_t c;
    j1850_pwm_rx_t rx;
    uint8_t data[3] = {0x61, 0x6A, 0x00};

    data[2] = (uint8_t)(j1850_crc(data, 2) ^ 0xFF); /* deliberately wrong */

    cap_reset(&c);
    cap_sof(&c);
    for (size_t i = 0; i < sizeof(data); i++) {
        cap_byte(&c, data[i]);
    }

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_BAD_CRC, decode(&c, &rx));
    /* A dongle diagnosing a marginal bus needs to see what arrived. */
    TEST_ASSERT_EQUAL_INT(sizeof(data), rx.len);
    TEST_ASSERT_EQUAL_MEM(data, rx.data, sizeof(data));
}

TEST(decode_reports_an_empty_capture) {
    j1850_pwm_rx_t rx;
    rmt_symbol_word_t none = {.val = 0};

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_EMPTY, j1850_pwm_decode(&none, 1, &rx));
    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_EMPTY, j1850_pwm_decode(NULL, 0, &rx));
    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_EMPTY, j1850_pwm_decode(&none, 0, &rx));
}

/* ------------------------------------------------------------------ *
 * Decoding: end of data and in-frame response
 * ------------------------------------------------------------------ */

TEST(decode_splits_the_response_bytes_off_at_the_eod_gap) {
    cap_t c;
    j1850_pwm_rx_t rx;
    uint8_t data[3] = {0x41, 0x6B, 0};

    data[2] = j1850_crc(data, 2);

    cap_reset(&c);
    cap_sof(&c);
    cap_byte(&c, data[0]);
    cap_byte(&c, data[1]);
    cap_byte(&c, data[2]);
    cap_eod(&c); /* clause 5.3.4.2: what follows is the response */
    cap_byte(&c, 0xF1);

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_OK, decode(&c, &rx));
    TEST_ASSERT_EQUAL_INT(sizeof(data), rx.len);
    TEST_ASSERT_EQUAL_MEM(data, rx.data, sizeof(data));
    TEST_ASSERT_TRUE(rx.eod);
    TEST_ASSERT_EQUAL_INT(1, rx.ifr_len);
    TEST_ASSERT_EQUAL_INT(0xF1, rx.ifr[0]);
}

/**
 * The receive path arms on a short idle so it can answer an IFR in time,
 * which means somebody else's response arrives as its own capture, with bits
 * but no SOF. It has to be recognised rather than counted as noise.
 */
TEST(decode_reads_a_capture_of_response_bytes_with_no_sof) {
    cap_t c;
    j1850_pwm_rx_t rx;

    cap_reset(&c);
    cap_byte(&c, 0xF1);

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_IFR_ONLY, decode(&c, &rx));
    TEST_ASSERT_EQUAL_INT(0, rx.len);
    TEST_ASSERT_EQUAL_INT(1, rx.ifr_len);
    TEST_ASSERT_EQUAL_INT(0xF1, rx.ifr[0]);
}

TEST(decode_rejects_a_partial_byte_of_response) {
    cap_t c;
    j1850_pwm_rx_t rx;

    cap_reset(&c);
    cap_bit(&c, true);
    cap_bit(&c, true);

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_FRAMING, decode(&c, &rx));
}

TEST(decode_rejects_an_eod_that_lands_mid_byte) {
    cap_t c;
    j1850_pwm_rx_t rx;

    cap_reset(&c);
    cap_sof(&c);
    cap_byte(&c, 0x41);
    cap_bit(&c, true);
    cap_bit(&c, false);
    cap_eod(&c);
    cap_byte(&c, 0xF1);

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_FRAMING, decode(&c, &rx));
}

/**
 * Clause 6.6.1.5: a rising edge more than an EOF after the last one opens a
 * new frame. The capture in hand is complete and the rest is not ours.
 */
TEST(decode_stops_at_a_frame_that_starts_after_an_end_of_frame) {
    cap_t c;
    j1850_pwm_rx_t rx;
    rmt_symbol_word_t *last;

    cap_reset(&c);
    cap_frame(&c, live_reply, sizeof(live_reply));

    /* Hold the bus passive past Tp5, then start a second frame. */
    last = &c.sym[c.n - 1];
    last->duration1 = (uint16_t)(J1850_PWM_TP6_NOM - last->duration0);
    cap_sof(&c);
    cap_byte(&c, 0x00);

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_OK, decode(&c, &rx));
    TEST_ASSERT_EQUAL_INT(sizeof(live_reply) + 1, rx.len);
    TEST_ASSERT_EQUAL_MEM(live_reply, rx.data, sizeof(live_reply));
    TEST_ASSERT_EQUAL_INT(0, rx.ifr_len);
}

/* ------------------------------------------------------------------ *
 * In-frame response policy
 * ------------------------------------------------------------------ */

TEST(ifr_is_wanted_for_a_module_reply_addressed_to_the_tester) {
    j1850_pwm_ifr_cfg_t cfg;
    /* 0x41: three byte header, K = 0, functional. Target 0x6B is the tester. */
    const uint8_t reply[] = {0x41, 0x6B, 0x10, 0x41, 0x00, 0x05};

    j1850_pwm_ifr_cfg_default(&cfg);
    cfg.enabled = true;

    TEST_ASSERT_TRUE(j1850_pwm_ifr_wanted(reply, sizeof(reply), &cfg));
    TEST_ASSERT_EQUAL_INT(0xF1, cfg.node_address);
}

TEST(ifr_is_enabled_by_default) {
    j1850_pwm_ifr_cfg_t cfg;
    const uint8_t reply[] = {0x41, 0x6B, 0x10, 0x41, 0x00, 0x05};

    j1850_pwm_ifr_cfg_default(&cfg);

    TEST_ASSERT_TRUE(cfg.enabled);
    TEST_ASSERT_TRUE(j1850_pwm_ifr_wanted(reply, sizeof(reply), &cfg));
}

TEST(ifr_is_not_wanted_when_the_k_bit_says_none_is_expected) {
    j1850_pwm_ifr_cfg_t cfg;
    uint8_t reply[] = {0x41 | J1850_HDR_K_BIT, 0x6B, 0x10, 0x05};

    j1850_pwm_ifr_cfg_default(&cfg);
    cfg.enabled = true;

    TEST_ASSERT_FALSE(j1850_pwm_ifr_wanted(reply, sizeof(reply), &cfg));
}

TEST(ifr_is_not_wanted_for_a_frame_addressed_to_somebody_else) {
    j1850_pwm_ifr_cfg_t cfg;
    uint8_t reply[] = {0x41, 0x10, 0x6B, 0x05};

    j1850_pwm_ifr_cfg_default(&cfg);
    cfg.enabled = true;

    TEST_ASSERT_FALSE(j1850_pwm_ifr_wanted(reply, sizeof(reply), &cfg));
}

/**
 * A single byte header has no target field. Acknowledging one would put our
 * address on the wire during somebody else's response window.
 */
TEST(ifr_is_not_wanted_for_a_single_byte_header) {
    j1850_pwm_ifr_cfg_t cfg;
    uint8_t reply[] = {0x41 | J1850_HDR_H_BIT, 0x6B, 0x05};

    j1850_pwm_ifr_cfg_default(&cfg);
    cfg.enabled = true;

    TEST_ASSERT_FALSE(j1850_pwm_ifr_wanted(reply, sizeof(reply), &cfg));
}

TEST(ifr_is_not_wanted_when_it_is_switched_off_or_the_frame_is_a_runt) {
    j1850_pwm_ifr_cfg_t cfg;
    uint8_t reply[] = {0x41, 0x6B, 0x10, 0x05};

    j1850_pwm_ifr_cfg_default(&cfg);
    cfg.enabled = false;
    TEST_ASSERT_FALSE(j1850_pwm_ifr_wanted(reply, sizeof(reply), &cfg));

    j1850_pwm_ifr_cfg_default(&cfg);
    cfg.enabled = true;
    TEST_ASSERT_FALSE(j1850_pwm_ifr_wanted(reply, 2, &cfg));
    TEST_ASSERT_FALSE(j1850_pwm_ifr_wanted(NULL, sizeof(reply), &cfg));
    TEST_ASSERT_FALSE(j1850_pwm_ifr_wanted(reply, sizeof(reply), NULL));
}

TEST(ifr_answers_only_the_configured_targets) {
    j1850_pwm_ifr_cfg_t cfg;
    uint8_t reply[] = {0x41, 0x33, 0x10, 0x05};

    j1850_pwm_ifr_cfg_default(&cfg);
    cfg.enabled = true;
    TEST_ASSERT_FALSE(j1850_pwm_ifr_wanted(reply, sizeof(reply), &cfg));

    cfg.targets[cfg.target_count++] = 0x33;
    TEST_ASSERT_TRUE(j1850_pwm_ifr_wanted(reply, sizeof(reply), &cfg));
}

TEST(ifr_answers_the_current_physical_node_address) {
    j1850_pwm_ifr_cfg_t cfg;
    uint8_t frame[] = {0x45, 0xF2, 0x10, 0x41, 0};
    j1850_pwm_ifr_cfg_default(&cfg);
    cfg.node_address = 0xF2;
    cfg.target_count = 0;
    TEST_ASSERT_TRUE(j1850_pwm_ifr_wanted(frame, sizeof(frame), &cfg));
    frame[1] = 0xF1;
    TEST_ASSERT_FALSE(j1850_pwm_ifr_wanted(frame, sizeof(frame), &cfg));
}

/* ------------------------------------------------------------------ *
 * Stress
 * ------------------------------------------------------------------ */

/** xorshift32: same sequence on every host, so a failure is reproducible. */
static uint32_t rnd_state = 0x12345678u;

static uint32_t rnd(void) {
    rnd_state ^= rnd_state << 13;
    rnd_state ^= rnd_state >> 17;
    rnd_state ^= rnd_state << 5;
    return rnd_state;
}

/**
 * Jitter every edge to the limit of its receive window at once, which is
 * worse than any real bus, and require the frame to survive intact. This is
 * the test that would fail if the decoder ever drifted towards measuring
 * cells from the falling edge.
 */
TEST(decode_survives_jitter_to_the_edges_of_every_receive_window) {
    rnd_state = 0xC0FFEEu;

    for (int trial = 0; trial < 2000; trial++) {
        uint8_t data[8];
        size_t len = 2 + (rnd() % (sizeof(data) - 1));
        cap_t c;
        j1850_pwm_rx_t rx;
        unsigned sof, gap;

        for (size_t i = 0; i < len - 1; i++) {
            data[i] = (uint8_t)rnd();
        }
        data[len - 1] = j1850_crc(data, len - 1);

        cap_reset(&c);

        sof = J1850_PWM_TP7_RX_MIN +
              rnd() % (J1850_PWM_TP7_RX_MAX - J1850_PWM_TP7_RX_MIN + 1);
        gap = J1850_PWM_TP4_RX_MIN +
              rnd() % (J1850_PWM_TP4_RX_MAX - J1850_PWM_TP4_RX_MIN + 1);
        cap_cell(&c, (uint16_t)sof, (uint16_t)(gap - sof));

        for (size_t i = 0; i < len; i++) {
            for (int b = 7; b >= 0; b--) {
                bool one = (data[i] >> b) & 1u;
                unsigned a, cell;

                /* Widths anywhere in the symbol's window, cells anywhere in
                 * Tp3, chosen independently so they can conspire. */
                if (one) {
                    a = J1850_PWM_TP1_RX_MIN +
                        rnd() % (J1850_PWM_BIT_SPLIT - J1850_PWM_TP1_RX_MIN);
                } else {
                    a = J1850_PWM_BIT_SPLIT + rnd() % (J1850_PWM_TP2_RX_MAX -
                                                       J1850_PWM_BIT_SPLIT + 1);
                }
                cell =
                    J1850_PWM_TP3_RX_MIN +
                    rnd() % (J1850_PWM_TP3_RX_MAX - J1850_PWM_TP3_RX_MIN + 1);
                if (cell <= a) {
                    cell = a + 1;
                }

                cap_cell(&c, (uint16_t)a, (uint16_t)(cell - a));
            }
        }

        TEST_ASSERT_MSG(decode(&c, &rx) == J1850_PWM_RX_OK, "trial %d: %s",
                        trial, j1850_pwm_rx_status_str(rx.status));
        TEST_ASSERT_MSG(rx.len == len, "trial %d: %u bytes, expected %u", trial,
                        rx.len, (unsigned)len);
        TEST_ASSERT_EQUAL_MEM(data, rx.data, len);
    }
}

/**
 * Arbitrary garbage, including widths and levels the peripheral would never
 * produce. Nothing here should be accepted as a frame by luck alone, and
 * nothing should read past the capture - which the address sanitiser in the
 * default build is watching for.
 */
TEST(decode_terminates_with_a_verdict_on_arbitrary_noise) {
    rnd_state = 0xDEADBEEFu;

    for (int trial = 0; trial < 20000; trial++) {
        rmt_symbol_word_t sym[64];
        size_t n = 1 + rnd() % (sizeof(sym) / sizeof(sym[0]));
        j1850_pwm_rx_t rx;
        j1850_pwm_rx_status_t st;

        for (size_t i = 0; i < n; i++) {
            sym[i].val = rnd();
        }

        st = j1850_pwm_decode(sym, n, &rx);

        TEST_ASSERT_MSG(rx.len <= J1850_PWM_MAX_FRAME,
                        "trial %d: len %u overflowed", trial, rx.len);
        TEST_ASSERT_MSG(rx.ifr_len <= J1850_PWM_MAX_FRAME,
                        "trial %d: ifr_len %u overflowed", trial, rx.ifr_len);
        /* Whatever it decided, a frame it calls good must actually be good. */
        if (st == J1850_PWM_RX_OK) {
            TEST_ASSERT_MSG(rx.len >= 2 && j1850_crc_check(rx.data, rx.len),
                            "trial %d: accepted a frame that fails its own CRC",
                            trial);
        }
    }
}

/**
 * Corrupt one bit of a good frame's waveform and require the decoder to
 * notice - by a timing verdict, or by the CRC. This is the guarantee the
 * standard leans on when it lets ambiguous pulse widths be guessed at.
 */
TEST(decode_never_silently_accepts_a_corrupted_frame) {
    rnd_state = 0xA5A5A5A5u;

    for (int trial = 0; trial < 5000; trial++) {
        uint8_t data[6];
        cap_t c;
        j1850_pwm_rx_t rx;
        size_t victim;
        j1850_pwm_rx_status_t st;

        for (size_t i = 0; i < sizeof(data) - 1; i++) {
            data[i] = (uint8_t)rnd();
        }
        data[sizeof(data) - 1] = j1850_crc(data, sizeof(data) - 1);

        cap_reset(&c);
        cap_frame(&c, data, sizeof(data) - 1);

        /* Flip one cell between a one and a zero, keeping the cell length. */
        victim = 1 + rnd() % (c.n - 1);
        if (c.sym[victim].duration0 == J1850_PWM_TP1_NOM) {
            c.sym[victim].duration0 = J1850_PWM_TP2_NOM;
            c.sym[victim].duration1 = J1850_PWM_TP3_NOM - J1850_PWM_TP2_NOM;
        } else {
            c.sym[victim].duration0 = J1850_PWM_TP1_NOM;
            c.sym[victim].duration1 = J1850_PWM_TP3_NOM - J1850_PWM_TP1_NOM;
        }

        st = decode(&c, &rx);

        TEST_ASSERT_MSG(st != J1850_PWM_RX_OK,
                        "trial %d: a flipped bit was accepted as a good frame",
                        trial);
    }
}

/* ------------------------------------------------------------------ *
 * The acknowledgement fast path
 *
 * j1850_pwm_quick_decode() exists because the full decoder cannot run inside
 * the 48 us an in-frame response has to start within. Two things have to hold
 * for that shortcut to be safe: it must agree with the full decoder on every
 * frame the full decoder accepts, and it must not turn something the full
 * decoder rejects into a frame that passes its CRC.
 * ------------------------------------------------------------------ */

TEST(quick_decode_agrees_with_the_full_decoder_on_valid_frames) {
    rnd_state = 0x5EED1234u;

    for (int trial = 0; trial < 4000; trial++) {
        uint8_t data[J1850_PWM_MAX_FRAME];
        size_t len = 2 + (rnd() % (J1850_PWM_MAX_FRAME - 1));
        cap_t c;
        j1850_pwm_rx_t rx;
        uint8_t quick[J1850_PWM_MAX_FRAME];
        size_t qlen;

        for (size_t i = 0; i < len - 1; i++) {
            data[i] = (uint8_t)rnd();
        }
        data[len - 1] = j1850_crc(data, len - 1);

        cap_reset(&c);
        cap_frame(&c, data, len - 1);
        cap_end(&c);

        TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_OK,
                              j1850_pwm_decode(c.sym, c.n, &rx));

        qlen = j1850_pwm_quick_decode(c.sym, c.n, quick, sizeof(quick));
        TEST_ASSERT_MSG(qlen == rx.len,
                        "trial %d: fast path read %u bytes, full %u", trial,
                        (unsigned)qlen, rx.len);
        TEST_ASSERT_EQUAL_MEM(rx.data, quick, qlen);
        TEST_ASSERT_TRUE(j1850_crc_check(quick, qlen));
    }
}

TEST(quick_decode_refuses_a_capture_with_no_start_of_frame) {
    cap_t c;
    uint8_t quick[J1850_PWM_MAX_FRAME];

    /* In-frame response bytes from another node: bits, but no SOF. Answering
     * these would be answering a frame that was never addressed to us. */
    cap_reset(&c);
    cap_byte(&c, 0xF1);
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(
        0, j1850_pwm_quick_decode(c.sym, c.n, quick, sizeof(quick)));
}

TEST(quick_decode_refuses_a_partial_byte_or_an_oversized_frame) {
    cap_t c;
    uint8_t quick[J1850_PWM_MAX_FRAME];

    cap_reset(&c);
    cap_sof(&c);
    cap_byte(&c, 0x41);
    cap_bit(&c, true);
    cap_end(&c);
    TEST_ASSERT_EQUAL_INT(
        0, j1850_pwm_quick_decode(c.sym, c.n, quick, sizeof(quick)));

    cap_reset(&c);
    cap_sof(&c);
    for (int i = 0; i <= J1850_PWM_MAX_FRAME; i++) {
        cap_byte(&c, (uint8_t)i);
    }
    cap_end(&c);
    TEST_ASSERT_EQUAL_INT(
        0, j1850_pwm_quick_decode(c.sym, c.n, quick, sizeof(quick)));
}

/**
 * The fast path skips every timing check, so the CRC is its only safety net -
 * which is what clause 7.3.2.1 says that CRC is for: "an incorrect decision
 * will be detected in the CRC error detection byte". The property to hold is
 * therefore about *content*, not about waveforms: a flipped bit must never
 * survive into a frame this would acknowledge.
 */
TEST(quick_decode_never_acknowledges_a_frame_with_a_flipped_bit) {
    rnd_state = 0x0BADF00Du;

    for (int trial = 0; trial < 5000; trial++) {
        uint8_t data[6];
        cap_t c;
        uint8_t quick[J1850_PWM_MAX_FRAME];
        size_t qlen, victim;

        for (size_t i = 0; i < sizeof(data) - 1; i++) {
            data[i] = (uint8_t)rnd();
        }
        data[sizeof(data) - 1] = j1850_crc(data, sizeof(data) - 1);

        cap_reset(&c);
        cap_frame(&c, data, sizeof(data) - 1);

        victim = 1 + rnd() % (c.n - 1);
        if (c.sym[victim].duration0 == J1850_PWM_TP1_NOM) {
            c.sym[victim].duration0 = J1850_PWM_TP2_NOM;
            c.sym[victim].duration1 = J1850_PWM_TP3_NOM - J1850_PWM_TP2_NOM;
        } else {
            c.sym[victim].duration0 = J1850_PWM_TP1_NOM;
            c.sym[victim].duration1 = J1850_PWM_TP3_NOM - J1850_PWM_TP1_NOM;
        }
        cap_end(&c);

        qlen = j1850_pwm_quick_decode(c.sym, c.n, quick, sizeof(quick));

        TEST_ASSERT_MSG(qlen == 0 || !j1850_crc_check(quick, qlen),
                        "trial %d: a flipped bit would have been acknowledged",
                        trial);
    }
}

/**
 * Widths that match no symbol are a different matter from a flipped bit. The
 * full decoder rejects them outright; the fast path resolves them to the
 * nearer bit and leans on the CRC. That is allowed - but only as long as the
 * two never disagree about the *bytes*, because a fast path reading one frame
 * while the queue receives another would be a bug nothing else could catch.
 */
TEST(quick_decode_and_the_full_decoder_never_disagree_about_the_bytes) {
    rnd_state = 0xFEEDFACEu;

    for (int trial = 0; trial < 5000; trial++) {
        uint8_t data[6];
        cap_t c;
        j1850_pwm_rx_t rx;
        uint8_t quick[J1850_PWM_MAX_FRAME];
        size_t qlen, victim;

        for (size_t i = 0; i < sizeof(data) - 1; i++) {
            data[i] = (uint8_t)rnd();
        }
        data[sizeof(data) - 1] = j1850_crc(data, sizeof(data) - 1);

        cap_reset(&c);
        cap_frame(&c, data, sizeof(data) - 1);

        victim = 1 + rnd() % (c.n - 1);
        c.sym[victim].duration0 = (uint16_t)(1 + rnd() % 40);
        cap_end(&c);

        j1850_pwm_decode(c.sym, c.n, &rx);
        qlen = j1850_pwm_quick_decode(c.sym, c.n, quick, sizeof(quick));

        if (rx.status == J1850_PWM_RX_OK && qlen != 0) {
            TEST_ASSERT_MSG(qlen == rx.len && memcmp(rx.data, quick, qlen) == 0,
                            "trial %d: the two paths read different bytes",
                            trial);
        }
    }
}

/**
 * A frame that invited an in-frame response is followed by the responder's
 * byte inside the same capture. The fast path has to end where the frame
 * does, exactly as the full decoder does - both because the response is not
 * part of the frame's CRC, and because this node recognises the echo of its
 * own transmissions by comparing these bytes.
 */
TEST(quick_decode_stops_at_the_end_of_data_like_the_full_decoder) {
    cap_t c;
    j1850_pwm_rx_t rx;
    uint8_t quick[J1850_PWM_MAX_FRAME];
    uint8_t data[6] = {0x61, 0x6A, 0xF1, 0x01, 0x00, 0};
    size_t qlen;

    data[5] = j1850_crc(data, 5);

    cap_reset(&c);
    cap_frame(&c, data, 5);
    cap_eod(&c);
    cap_byte(&c, 0x10); /* the module acknowledging with its address */
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_OK, j1850_pwm_decode(c.sym, c.n, &rx));
    TEST_ASSERT_EQUAL_INT(1, rx.ifr_len);

    qlen = j1850_pwm_quick_decode(c.sym, c.n, quick, sizeof(quick));
    TEST_ASSERT_EQUAL_INT(rx.len, qlen);
    TEST_ASSERT_EQUAL_MEM(rx.data, quick, qlen);
    TEST_ASSERT_TRUE(j1850_crc_check(quick, qlen));
}

/** The same, swept over every frame length and both in-frame response sizes. */
TEST(quick_decode_matches_the_full_decoder_on_frames_with_a_response) {
    rnd_state = 0x1234ABCDu;

    for (int trial = 0; trial < 2000; trial++) {
        uint8_t data[8];
        size_t len = 2 + (rnd() % (sizeof(data) - 1));
        size_t ifr_len = 1 + (rnd() % 2);
        cap_t c;
        j1850_pwm_rx_t rx;
        uint8_t quick[J1850_PWM_MAX_FRAME];
        size_t qlen;

        for (size_t i = 0; i < len - 1; i++) {
            data[i] = (uint8_t)rnd();
        }
        data[len - 1] = j1850_crc(data, len - 1);

        cap_reset(&c);
        cap_frame(&c, data, len - 1);
        cap_eod(&c);
        for (size_t i = 0; i < ifr_len; i++) {
            cap_byte(&c, (uint8_t)rnd());
        }
        cap_end(&c);

        TEST_ASSERT_MSG(j1850_pwm_decode(c.sym, c.n, &rx) == J1850_PWM_RX_OK,
                        "trial %d: %s", trial,
                        j1850_pwm_rx_status_str(rx.status));
        TEST_ASSERT_EQUAL_INT(ifr_len, rx.ifr_len);

        qlen = j1850_pwm_quick_decode(c.sym, c.n, quick, sizeof(quick));
        TEST_ASSERT_MSG(qlen == rx.len, "trial %d: fast path %u bytes, full %u",
                        trial, (unsigned)qlen, rx.len);
        TEST_ASSERT_EQUAL_MEM(rx.data, quick, qlen);
    }
}

/* ------------------------------------------------------------------ *
 * Edges the interrupt can hit but a tidy capture never shows
 * ------------------------------------------------------------------ */

/**
 * The peripheral usually closes a capture with a zero duration, but a capture
 * cut off by a full buffer simply stops. Ending on a passive phase is an end
 * of frame either way.
 */
TEST(decode_treats_a_capture_that_simply_stops_as_an_end_of_frame) {
    cap_t c;
    j1850_pwm_rx_t rx;

    cap_reset(&c);
    cap_frame(&c, live_reply, sizeof(live_reply));
    /* No cap_end(): the last symbol's passive phase is the last thing here. */

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_OK, j1850_pwm_decode(c.sym, c.n, &rx));
    TEST_ASSERT_EQUAL_INT(sizeof(live_reply) + 1, rx.len);
    TEST_ASSERT_EQUAL_MEM(live_reply, rx.data, sizeof(live_reply));
}

/**
 * The shape the peripheral really hands over: a final symbol that is an
 * active phase and a zero. Worth its own case because it is what every frame
 * off a real bus looks like, and because the width in that final symbol is
 * also what the driver uses to work out where the frame's last rising edge
 * was.
 */
TEST(decode_reads_a_capture_ending_the_way_the_hardware_ends_one) {
    cap_t c;
    j1850_pwm_rx_t rx;

    cap_reset(&c);
    cap_frame(&c, live_reply, sizeof(live_reply));
    cap_end_on_hardware_boundary(&c);

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_OK, j1850_pwm_decode(c.sym, c.n, &rx));
    TEST_ASSERT_EQUAL_INT(sizeof(live_reply) + 1, rx.len);
    TEST_ASSERT_EQUAL_MEM(live_reply, rx.data, sizeof(live_reply));

    /* 0x05 ends in a "1" bit, so the last active phase is Tp1. */
    TEST_ASSERT_EQUAL_INT(J1850_PWM_TP1_NOM, rx.last_active_us);
}

/** A second end of data inside one capture ends the frame; there is no
 *  such thing as a response to a response. */
TEST(decode_stops_at_a_second_end_of_data) {
    cap_t c;
    j1850_pwm_rx_t rx;
    uint8_t data[3] = {0x41, 0x6B, 0};

    data[2] = j1850_crc(data, 2);

    cap_reset(&c);
    cap_sof(&c);
    for (size_t i = 0; i < sizeof(data); i++) {
        cap_byte(&c, data[i]);
    }
    cap_eod(&c);
    cap_byte(&c, 0xF1);
    cap_eod(&c);
    cap_byte(&c, 0x10);

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_OK, decode(&c, &rx));
    TEST_ASSERT_EQUAL_MEM(data, rx.data, sizeof(data));
    TEST_ASSERT_EQUAL_INT(1, rx.ifr_len);
    TEST_ASSERT_EQUAL_INT(0xF1, rx.ifr[0]);
}

TEST(quick_decode_skips_leading_idle_and_gives_up_on_a_silent_capture) {
    cap_t c;
    uint8_t quick[J1850_PWM_MAX_FRAME];
    size_t qlen;

    /* Idle first, then a frame: what a capture armed before the bus woke up
     * looks like. */
    cap_reset(&c);
    c.sym[c.n].level0 = J1850_PWM_PASSIVE;
    c.sym[c.n].duration0 = 900;
    c.sym[c.n].level1 = J1850_PWM_PASSIVE;
    c.sym[c.n].duration1 = 900;
    c.n++;
    cap_frame(&c, live_reply, sizeof(live_reply));
    cap_end(&c);

    qlen = j1850_pwm_quick_decode(c.sym, c.n, quick, sizeof(quick));
    TEST_ASSERT_EQUAL_INT(sizeof(live_reply) + 1, qlen);
    TEST_ASSERT_EQUAL_MEM(live_reply, quick, sizeof(live_reply));

    /* Idle and nothing else. */
    cap_reset(&c);
    c.sym[c.n].level0 = J1850_PWM_PASSIVE;
    c.sym[c.n].duration0 = 900;
    c.sym[c.n].level1 = J1850_PWM_PASSIVE;
    c.sym[c.n].duration1 = 900;
    c.n++;
    cap_end(&c);

    TEST_ASSERT_EQUAL_INT(
        0, j1850_pwm_quick_decode(c.sym, c.n, quick, sizeof(quick)));
}

TEST(the_decoders_tolerate_being_handed_nothing) {
    uint8_t quick[J1850_PWM_MAX_FRAME];
    rmt_symbol_word_t sym = {.val = 0};

    TEST_ASSERT_EQUAL_INT(J1850_PWM_RX_EMPTY, j1850_pwm_decode(&sym, 1, NULL));
    TEST_ASSERT_EQUAL_INT(
        0, j1850_pwm_quick_decode(NULL, 4, quick, sizeof(quick)));
    TEST_ASSERT_EQUAL_INT(0, j1850_pwm_quick_decode(&sym, 1, NULL, 4));
    TEST_ASSERT_EQUAL_INT(0, j1850_pwm_quick_decode(&sym, 1, quick, 0));
    TEST_ASSERT_EQUAL_INT(0xFF ^ 0xFF, j1850_crc(NULL, 4));
    j1850_pwm_ifr_cfg_default(NULL); /* must not fault */
}

/** Every status has a name; the shell prints them and "?" helps nobody. */
TEST(every_receive_status_has_a_distinct_name) {
    static const j1850_pwm_rx_status_t all[] = {
        J1850_PWM_RX_OK,         J1850_PWM_RX_IFR_ONLY, J1850_PWM_RX_BREAK,
        J1850_PWM_RX_EMPTY,      J1850_PWM_RX_NO_SOF,   J1850_PWM_RX_BAD_SYMBOL,
        J1850_PWM_RX_BAD_TIMING, J1850_PWM_RX_FRAMING,  J1850_PWM_RX_TOO_LONG,
        J1850_PWM_RX_SHORT,      J1850_PWM_RX_BAD_CRC,
    };
    size_t n = sizeof(all) / sizeof(all[0]);

    for (size_t i = 0; i < n; i++) {
        const char *a = j1850_pwm_rx_status_str(all[i]);

        TEST_ASSERT_NOT_NULL(a);
        TEST_ASSERT_MSG(a[0] != '?', "status %d has no name", (int)all[i]);

        for (size_t j = i + 1; j < n; j++) {
            TEST_ASSERT_MSG(
                !td_string_equal(a, j1850_pwm_rx_status_str(all[j])),
                "statuses %d and %d share the name \"%s\"", (int)all[i],
                (int)all[j], a);
        }
    }

    TEST_ASSERT_EQUAL_STRING(
        "?", j1850_pwm_rx_status_str((j1850_pwm_rx_status_t)99));
}

static bool stream_capture(const cap_t *c, j1850_pwm_stream_t *stream) {
    uint32_t gap = 0;
    bool ready = false;
    for (size_t i = 0; i < c->n; i++) {
        ready = j1850_pwm_stream_pulse(stream, c->sym[i].duration0, gap);
        gap = c->sym[i].duration0 + c->sym[i].duration1;
    }
    return ready;
}

TEST(streaming_ifr_checks_crc_for_both_final_bit_values) {
    bool final_one = false, final_zero = false;
    for (unsigned value = 0; value < 256; value++) {
        uint8_t data[] = {0x41, 0x6B, 0x10, 0x41, 0x00, (uint8_t)value};
        cap_t c = {0};
        j1850_pwm_stream_t stream = {0};
        cap_frame(&c, data, sizeof(data));
        TEST_ASSERT_TRUE(stream_capture(&c, &stream));
        TEST_ASSERT_EQUAL_INT(sizeof(data) + 1, stream.len);
        TEST_ASSERT_EQUAL_MEM(data, stream.data, sizeof(data));
        if (stream.data[stream.len - 1] & 1)
            final_one = true;
        else
            final_zero = true;
        c.sym[c.n - 1].duration0 = c.sym[c.n - 1].duration0 == 7 ? 15 : 7;
        stream = (j1850_pwm_stream_t){0};
        TEST_ASSERT_FALSE(stream_capture(&c, &stream));
    }
    TEST_ASSERT_TRUE(final_one && final_zero);
}

TEST(streaming_ifr_rejects_bad_pulses_and_requires_sof) {
    const uint8_t data[] = {0x41, 0x6B, 0x10, 0x41, 0};
    cap_t good = {0};
    cap_frame(&good, data, sizeof(data));
    for (size_t i = 1; i < good.n; i++) {
        cap_t bad = good;
        j1850_pwm_stream_t stream = {0};
        bad.sym[i].duration0 = 2;
        TEST_ASSERT_FALSE(stream_capture(&bad, &stream));
        bad = good;
        bad.sym[i - 1].duration1 = 60;
        stream = (j1850_pwm_stream_t){0};
        TEST_ASSERT_FALSE(stream_capture(&bad, &stream));
    }
    j1850_pwm_stream_t stream = {0};
    for (size_t i = 1; i < good.n; i++)
        TEST_ASSERT_FALSE(
            j1850_pwm_stream_pulse(&stream, good.sym[i].duration0, 24));
}

TEST(streaming_ifr_leaves_room_for_the_response_byte) {
    uint8_t data[J1850_PWM_MAX_FRAME - 1] = {0x41, 0x6B, 0x10};
    cap_t c = {0};
    j1850_pwm_stream_t stream = {0};
    cap_frame(&c, data, sizeof(data));
    TEST_ASSERT_FALSE(stream_capture(&c, &stream));
    TEST_ASSERT_EQUAL_INT(J1850_PWM_MAX_FRAME, stream.len);
    TEST_ASSERT_FALSE(j1850_pwm_stream_pulse(&stream, 7, 24));
    TEST_ASSERT_FALSE(stream.valid);
}
