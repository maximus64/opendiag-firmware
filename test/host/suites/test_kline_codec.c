/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_kline_codec.c
 * @brief The ISO 9141-2 / ISO 14230 message grammar, off target.
 *
 * Every table the driver depends on is walked here against the document
 * rather than against a vehicle: the format byte forms of clause 4.1.5, the
 * checksum of clause 4.3, all nineteen key byte sets of Table 5, the timing
 * defaults of Tables 1 and 2, and the P2max arithmetic of Table 3. A sweep at
 * the end pushes arbitrary bytes through the length calculation to show it
 * never claims a message longer than the standard allows and never reads past
 * what it was given.
 *
 * Clause references are to ISO/DIS 14230-2.
 */

#include <string.h>

#include "kline_codec.h"
#include "td_test.h"

/* ------------------------------------------------------------------ *
 * $55 synchronisation baud detection, clause 5.1.5.2.2
 * ------------------------------------------------------------------ */

TEST(the_sync_edges_measure_10400_baud) {
    /* Integer microseconds necessarily alternate around the 96.15 us bit
     * period; the estimator must not require a fractional timer. */
    static const uint32_t edge[] = {
        1000, 1096, 1192, 1288, 1385, 1481, 1577, 1673, 1769, 1865,
    };
    uint32_t baud =
        kline_sync_baud_from_edges(edge, sizeof(edge) / sizeof(edge[0]));

    TEST_ASSERT_MSG(baud >= 10350 && baud <= 10450, "measured %u baud",
                    (unsigned)baud);
}

TEST(the_sync_edges_cover_9600_and_the_1200_baud_lower_limit) {
    static const uint32_t edge_9600[] = {
        50, 154, 258, 363, 467, 571, 675, 779, 883, 988,
    };
    static const uint32_t edge_1200[] = {
        0, 833, 1667, 2500, 3333, 4167, 5000, 5833, 6667, 7500,
    };
    uint32_t baud;

    baud = kline_sync_baud_from_edges(edge_9600,
                                      sizeof(edge_9600) / sizeof(edge_9600[0]));
    TEST_ASSERT_MSG(baud >= 9550 && baud <= 9650, "measured %u baud",
                    (unsigned)baud);

    baud = kline_sync_baud_from_edges(edge_1200,
                                      sizeof(edge_1200) / sizeof(edge_1200[0]));
    TEST_ASSERT_EQUAL_INT(1200, baud);
}

TEST(the_sync_estimator_tolerates_edge_jitter) {
    static const uint32_t edge[] = {
        5000, 5092, 5192, 5287, 5384, 5474, 5577, 5673, 5767, 5867,
    };
    uint32_t baud =
        kline_sync_baud_from_edges(edge, sizeof(edge) / sizeof(edge[0]));

    TEST_ASSERT_MSG(baud >= 10300 && baud <= 10500,
                    "jittered capture measured %u baud", (unsigned)baud);
}

TEST(the_sync_estimator_handles_the_microsecond_counter_wrapping) {
    static const uint32_t edge[] = {
        UINT32_MAX - 400u,
        UINT32_MAX - 296u,
        UINT32_MAX - 192u,
        UINT32_MAX - 88u,
        15u,
        119u,
        223u,
        327u,
        431u,
        535u,
    };
    uint32_t baud =
        kline_sync_baud_from_edges(edge, sizeof(edge) / sizeof(edge[0]));

    TEST_ASSERT_MSG(baud >= 9550 && baud <= 9650,
                    "wrapped capture measured %u baud", (unsigned)baud);
}

TEST(a_missing_or_spurious_sync_edge_is_rejected) {
    static const uint32_t missed[] = {
        0, 96, 192, 384, 480, 576, 672, 768, 864, 960,
    };
    static const uint32_t spurious[] = {
        0, 96, 126, 192, 288, 384, 480, 576, 672, 768,
    };

    TEST_ASSERT_EQUAL_INT(0, kline_sync_baud_from_edges(
                                 missed, sizeof(missed) / sizeof(missed[0])));
    TEST_ASSERT_EQUAL_INT(
        0, kline_sync_baud_from_edges(spurious,
                                      sizeof(spurious) / sizeof(spurious[0])));
}

TEST(an_incomplete_sync_or_a_rate_outside_the_iso_range_is_rejected) {
    static const uint32_t too_slow[] = {
        0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000,
    };
    static const uint32_t too_fast[] = {
        0, 80, 160, 240, 320, 400, 480, 560, 640, 720,
    };

    TEST_ASSERT_EQUAL_INT(
        0, kline_sync_baud_from_edges(NULL, KLINE_SYNC_EDGE_COUNT));
    TEST_ASSERT_EQUAL_INT(
        0, kline_sync_baud_from_edges(too_slow, KLINE_SYNC_EDGE_COUNT - 1));
    TEST_ASSERT_EQUAL_INT(
        0, kline_sync_baud_from_edges(too_slow,
                                      sizeof(too_slow) / sizeof(too_slow[0])));
    TEST_ASSERT_EQUAL_INT(
        0, kline_sync_baud_from_edges(too_fast,
                                      sizeof(too_fast) / sizeof(too_fast[0])));
}

/* ------------------------------------------------------------------ *
 * Checksum, clause 4.3
 * ------------------------------------------------------------------ */

TEST(the_checksum_is_the_eight_bit_sum_of_the_message) {
    /* The whitepaper's ISO 9141-2 mode 01 PID 00 request, checksum included:
     * 68 6A F1 01 00 C4. */
    static const uint8_t req[] = {0x68, 0x6A, 0xF1, 0x01, 0x00};

    TEST_ASSERT_EQUAL_INT(0xC4, kline_checksum(req, sizeof(req)));
}

TEST(the_checksum_of_the_kwp_request_matches_the_whitepaper) {
    /* C2 33 F1 01 00 E7 for KWP, and C1 33 F1 81 66 for StartCommunication. */
    static const uint8_t kwp_req[] = {0xC2, 0x33, 0xF1, 0x01, 0x00};
    static const uint8_t start[] = {0xC1, 0x33, 0xF1, 0x81};

    TEST_ASSERT_EQUAL_INT(0xE7, kline_checksum(kwp_req, sizeof(kwp_req)));
    TEST_ASSERT_EQUAL_INT(0x66, kline_checksum(start, sizeof(start)));
}

TEST(the_checksum_wraps_at_one_byte) {
    static const uint8_t d[] = {0xFF, 0xFF, 0x02};

    TEST_ASSERT_EQUAL_INT(0x00, kline_checksum(d, sizeof(d)));
}

TEST(a_frame_is_accepted_only_when_its_checksum_adds_up) {
    uint8_t f[] = {0x48, 0x6B, 0x09, 0x41, 0x00, 0xBE, 0x3F, 0xA8, 0x11, 0xB3};

    TEST_ASSERT_TRUE(kline_frame_ok(f, sizeof(f)));

    f[5] ^= 0x01;
    TEST_ASSERT_FALSE(kline_frame_ok(f, sizeof(f)));
}

TEST(a_frame_shorter_than_a_message_is_rejected) {
    /* Format byte, service id, checksum is the shortest thing clause 4.1.5
     * describes. Two bytes cannot be a message however they add up. */
    static const uint8_t two[] = {0x00, 0x00};
    static const uint8_t three[] = {0x01, 0x3E, 0x3F};

    TEST_ASSERT_FALSE(kline_frame_ok(two, sizeof(two)));
    TEST_ASSERT_TRUE(kline_frame_ok(three, sizeof(three)));
    TEST_ASSERT_FALSE(kline_frame_ok(NULL, 5));
}

/* ------------------------------------------------------------------ *
 * Format byte, clause 4.1.1 and 4.1.4
 * ------------------------------------------------------------------ */

TEST(the_address_mode_comes_from_the_top_two_bits) {
    TEST_ASSERT_EQUAL_INT(KLINE_ADDR_NONE, kline_addr_mode(0x02));
    TEST_ASSERT_EQUAL_INT(KLINE_ADDR_CARB, kline_addr_mode(0x68));
    TEST_ASSERT_EQUAL_INT(KLINE_ADDR_CARB, kline_addr_mode(0x48));
    TEST_ASSERT_EQUAL_INT(KLINE_ADDR_PHYSICAL, kline_addr_mode(0x83));
    TEST_ASSERT_EQUAL_INT(KLINE_ADDR_FUNCTIONAL, kline_addr_mode(0xC2));
}

TEST(a_header_without_addresses_is_one_byte_long) {
    TEST_ASSERT_EQUAL_INT(1, kline_header_len(0x02));
    TEST_ASSERT_EQUAL_INT(3, kline_header_len(0x82));
    TEST_ASSERT_EQUAL_INT(3, kline_header_len(0xC2));
    TEST_ASSERT_EQUAL_INT(3, kline_header_len(0x68));
}

TEST(the_length_in_the_format_byte_gives_the_whole_message) {
    /* 83 F1 10 C1 KB1 KB2 CS: three header, three data, one checksum. */
    static const uint8_t resp[] = {0x83, 0xF1, 0x10, 0xC1, 0xE9, 0x8F, 0x00};

    TEST_ASSERT_EQUAL_INT(7, kline_msg_len(resp, sizeof(resp)));
    /* The answer must not need the whole message to be there yet - that is
     * the entire point of a length field. */
    TEST_ASSERT_EQUAL_INT(7, kline_msg_len(resp, 1));
}

TEST(a_header_with_no_addresses_still_carries_its_length) {
    static const uint8_t m[] = {0x02, 0x3E, 0x01, 0x41};

    TEST_ASSERT_EQUAL_INT(4, kline_msg_len(m, sizeof(m)));
}

TEST(a_zero_length_field_moves_the_length_into_its_own_byte) {
    /* Clause 4.1.4: format, target, source, length, then the data. */
    static const uint8_t m[] = {0x80, 0xF1, 0x10, 0x05, 0x41,
                                0x00, 0x11, 0x22, 0x33, 0x00};

    TEST_ASSERT_EQUAL_INT(10, kline_msg_len(m, sizeof(m)));

    /* Not knowable until the length byte itself has arrived. */
    TEST_ASSERT_EQUAL_INT(0, kline_msg_len(m, 3));
    TEST_ASSERT_EQUAL_INT(10, kline_msg_len(m, 4));
}

TEST(a_length_byte_of_zero_names_no_message) {
    static const uint8_t m[] = {0x80, 0xF1, 0x10, 0x00, 0x00};

    TEST_ASSERT_EQUAL_INT(0, kline_msg_len(m, sizeof(m)));
}

TEST(a_carb_message_declares_no_length_at_all) {
    /* This is what forces the receiver to fall back on the inter-byte gap for
     * ISO 9141-2, and the reason KLINE_CARB_MAX_MSG exists. */
    static const uint8_t req[] = {0x68, 0x6A, 0xF1, 0x01, 0x00, 0xC4};
    static const uint8_t rsp[] = {0x48, 0x6B, 0x09, 0x41, 0x00, 0xBE};

    TEST_ASSERT_EQUAL_INT(0, kline_msg_len(req, sizeof(req)));
    TEST_ASSERT_EQUAL_INT(0, kline_msg_len(rsp, sizeof(rsp)));
    TEST_ASSERT_TRUE(kline_is_carb_fmt(0x68));
    TEST_ASSERT_TRUE(kline_is_carb_fmt(0x48));
    TEST_ASSERT_FALSE(kline_is_carb_fmt(0xC2));
}

TEST(the_longest_message_the_length_byte_can_name_fits_the_buffer) {
    static const uint8_t m[] = {0x80, 0xF1, 0x10, 0xFF};

    /* Clause 4.1.4: "The longest message consists of a maximum of 260 byte." */
    TEST_ASSERT_EQUAL_INT(KLINE_MAX_MSG, kline_msg_len(m, sizeof(m)));
}

TEST(no_format_byte_can_name_a_message_longer_than_the_standard_allows) {
    /* The receive buffer is sized on this promise, so it is worth proving
     * over the whole input space rather than at a few points. */
    for (unsigned fmt = 0; fmt < 256; fmt++) {
        for (unsigned lenb = 0; lenb < 256; lenb++) {
            uint8_t m[4] = {(uint8_t)fmt, 0xF1, 0x10, (uint8_t)lenb};
            size_t n = kline_msg_len(m, sizeof(m));

            TEST_ASSERT_MSG(n <= KLINE_MAX_MSG,
                            "fmt %02X len %02X claimed %u bytes", fmt, lenb,
                            (unsigned)n);
            TEST_ASSERT_MSG(n == 0 || n >= KLINE_MIN_MSG,
                            "fmt %02X len %02X claimed %u bytes", fmt, lenb,
                            (unsigned)n);
        }
    }
}

TEST(the_length_calculation_never_reads_past_what_it_was_given) {
    /* Every prefix of a maximal header, so a short read cannot reach the
     * length byte before it exists. Run under a sanitizer this is the test
     * that catches an off-by-one in the additional length byte handling. */
    static const uint8_t m[] = {0x80, 0xF1, 0x10, 0x05};

    TEST_ASSERT_EQUAL_INT(0, kline_msg_len(m, 0));
    TEST_ASSERT_EQUAL_INT(0, kline_msg_len(NULL, 4));

    for (size_t n = 1; n <= sizeof(m); n++) {
        uint8_t copy[sizeof(m)];

        memcpy(copy, m, n);
        (void)kline_msg_len(copy, n);
    }
}

/* ------------------------------------------------------------------ *
 * Key bytes, clause 5.1.5.1 and Table 5
 * ------------------------------------------------------------------ */

TEST(the_iso_9141_key_byte_pairs_are_recognised) {
    kline_keybytes_t k;

    TEST_ASSERT_TRUE(kline_decode_keybytes(0x08, 0x08, &k));
    TEST_ASSERT_EQUAL_INT(KLINE_VARIANT_ISO9141, k.variant);
    TEST_ASSERT_TRUE(k.parity_ok);

    TEST_ASSERT_TRUE(kline_decode_keybytes(0x94, 0x94, &k));
    TEST_ASSERT_EQUAL_INT(KLINE_VARIANT_ISO9141, k.variant);
    TEST_ASSERT_TRUE(k.parity_ok);
}

TEST(the_four_key_byte_sets_iso_14230_4_allows_are_kwp) {
    /* ISO 14230-4 clause 4.4 permits 8F E9, 8F 6B, 8F 6D and 8F EF for
     * legislated OBD, and all four are to behave as 8F E9. */
    static const uint8_t caps[] = {0xE9, 0x6B, 0x6D, 0xEF};

    for (size_t i = 0; i < sizeof(caps); i++) {
        kline_keybytes_t k;

        TEST_ASSERT_TRUE(kline_decode_keybytes(caps[i], 0x8F, &k));
        TEST_ASSERT_EQUAL_INT(KLINE_VARIANT_KWP, k.variant);
        TEST_ASSERT_MSG(k.hdr_address, "cap %02X should allow addresses",
                        caps[i]);
        TEST_ASSERT_MSG(!k.extended_timing, "cap %02X should be normal timing",
                        caps[i]);
        TEST_ASSERT_TRUE(k.parity_ok);
    }
}

TEST(the_key_bytes_are_read_whichever_order_they_arrive_in) {
    kline_keybytes_t a, b;

    /* Figure 8 transmits KB1 then KB2; Table 5 and the field literature write
     * the pair the other way round. Both have to work. */
    TEST_ASSERT_TRUE(kline_decode_keybytes(0xE9, 0x8F, &a));
    TEST_ASSERT_TRUE(kline_decode_keybytes(0x8F, 0xE9, &b));

    TEST_ASSERT_EQUAL_INT(a.variant, b.variant);
    TEST_ASSERT_EQUAL_INT(a.code, b.code);
    TEST_ASSERT_EQUAL_INT(a.hdr_address, b.hdr_address);
    TEST_ASSERT_EQUAL_INT(a.len_in_format, b.len_in_format);
}

/** Table 5 in full: capability byte, decimal code, and the four columns. */
TEST(every_key_byte_set_in_table_5_decodes_to_its_row) {
    struct row {
        uint8_t cap;
        uint16_t code;
        bool len_fmt, len_byte, hdr1, hdra, ext;
    };
    static const struct row table[] = {
        /* cap   code  AL0    AL1    HB0    HB1    ext   */
        {0xD0, 2000, false, false, false, false, true},
        {0xD5, 2005, true, false, true, false, true},
        {0xD6, 2006, false, true, true, false, true},
        {0x57, 2007, true, true, true, false, true},
        {0xD9, 2009, true, false, false, true, true},
        {0xDA, 2010, false, true, false, true, true},
        {0x5B, 2011, true, true, false, true, true},
        {0x5D, 2013, true, false, true, true, true},
        {0x5E, 2014, false, true, true, true, true},
        {0xDF, 2015, true, true, true, true, true},
        {0xE5, 2021, true, false, true, false, false},
        {0xE6, 2022, false, true, true, false, false},
        {0x67, 2023, true, true, true, false, false},
        {0xE9, 2025, true, false, false, true, false},
        {0xEA, 2026, false, true, false, true, false},
        {0x6B, 2027, true, true, false, true, false},
        {0x6D, 2029, true, false, true, true, false},
        {0x6E, 2030, false, true, true, true, false},
        {0xEF, 2031, true, true, true, true, false},
    };

    for (size_t i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
        const struct row *r = &table[i];
        kline_keybytes_t k;

        TEST_ASSERT_MSG(kline_decode_keybytes(r->cap, 0x8F, &k),
                        "key byte %02X 8F was not recognised", r->cap);
        TEST_ASSERT_MSG(k.code == r->code, "%02X: code %u, expected %u", r->cap,
                        k.code, r->code);
        TEST_ASSERT_MSG(k.len_in_format == r->len_fmt,
                        "%02X: length in format byte", r->cap);
        TEST_ASSERT_MSG(k.extra_len_byte == r->len_byte,
                        "%02X: additional length byte", r->cap);
        TEST_ASSERT_MSG(k.hdr_1byte == r->hdr1, "%02X: one byte header",
                        r->cap);
        TEST_ASSERT_MSG(k.hdr_address == r->hdra, "%02X: address header",
                        r->cap);
        TEST_ASSERT_MSG(k.extended_timing == r->ext, "%02X: timing set",
                        r->cap);
        TEST_ASSERT_MSG(k.parity_ok, "%02X: odd parity", r->cap);
    }
}

TEST(key_bytes_that_name_no_protocol_are_refused) {
    kline_keybytes_t k;

    TEST_ASSERT_FALSE(kline_decode_keybytes(0x00, 0x00, &k));
    TEST_ASSERT_EQUAL_INT(KLINE_VARIANT_UNKNOWN, k.variant);
    /* The raw bytes still come back, because a tester with AT KW0 wants to
     * report them even when it cannot classify them. */
    TEST_ASSERT_EQUAL_INT(0x00, k.kb1);

    TEST_ASSERT_FALSE(kline_decode_keybytes(0x08, 0x94, &k));
    TEST_ASSERT_FALSE(kline_decode_keybytes(0x12, 0x34, &k));
    TEST_ASSERT_FALSE(kline_decode_keybytes(0x00, 0x00, NULL));
}

TEST(odd_parity_is_checked_over_all_eight_bits) {
    TEST_ASSERT_TRUE(kline_odd_parity(0x08));
    TEST_ASSERT_TRUE(kline_odd_parity(0x94));
    TEST_ASSERT_TRUE(kline_odd_parity(0x8F));
    TEST_ASSERT_TRUE(kline_odd_parity(0xE9));
    TEST_ASSERT_FALSE(kline_odd_parity(0x00));
    TEST_ASSERT_FALSE(kline_odd_parity(0x33));
}

TEST(bad_parity_is_reported_without_rejecting_the_pair) {
    kline_keybytes_t k;

    /* $E8 is $E9 with a bit knocked out: still a KWP shaped pair, but the
     * line is doing something it should not be. */
    TEST_ASSERT_TRUE(kline_decode_keybytes(0xE8, 0x8F, &k));
    TEST_ASSERT_EQUAL_INT(KLINE_VARIANT_KWP, k.variant);
    TEST_ASSERT_FALSE(k.parity_ok);
}

/* ------------------------------------------------------------------ *
 * Timing, clause 4.4
 * ------------------------------------------------------------------ */

TEST(the_normal_timing_set_is_table_1) {
    kline_timing_t t;

    kline_timing_normal(&t);

    TEST_ASSERT_EQUAL_INT(20, t.p1_max);
    TEST_ASSERT_EQUAL_INT(25, t.p2_min);
    TEST_ASSERT_EQUAL_INT(50, t.p2_max);
    TEST_ASSERT_EQUAL_INT(55, t.p3_min);
    TEST_ASSERT_EQUAL_INT(5000, t.p3_max);
    TEST_ASSERT_EQUAL_INT(5, t.p4_min);
}

TEST(the_extended_timing_set_is_table_2) {
    kline_timing_t t;

    kline_timing_extended(&t);

    TEST_ASSERT_EQUAL_INT(20, t.p1_max);
    TEST_ASSERT_EQUAL_INT(0, t.p2_min);
    TEST_ASSERT_EQUAL_INT(1000, t.p2_max);
    TEST_ASSERT_EQUAL_INT(0, t.p3_min);
    TEST_ASSERT_EQUAL_INT(5, t.p4_min);
}

TEST(both_timing_sets_satisfy_the_restrictions_the_standard_lists) {
    kline_timing_t sets[2];

    kline_timing_normal(&sets[0]);
    kline_timing_extended(&sets[1]);

    for (int i = 0; i < 2; i++) {
        const kline_timing_t *t = &sets[i];

        /* Clause 4.4: Pimin < Pimax, and where a receiver ends a message by
         * time-out, P2min > P4max and P2min > P1max. The extended set is
         * explicitly for physical addressing where that does not apply, so
         * only the normal one is held to the last two. */
        TEST_ASSERT_MSG(t->p2_min < t->p2_max, "set %d: P2min < P2max", i);
        TEST_ASSERT_MSG(t->p3_min < t->p3_max, "set %d: P3min < P3max", i);

        if (i == 0) {
            TEST_ASSERT_MSG(t->p3_min > t->p4_min, "set %d: P3min > P4min", i);
            TEST_ASSERT_MSG(t->p2_min > t->p1_max, "set %d: P2min > P1max", i);
        }
    }
}

TEST(p2max_decodes_by_table_3) {
    /* $01 to $F0 counts 25 ms. */
    TEST_ASSERT_EQUAL_INT(25, kline_p2max_from_byte(0x01));
    TEST_ASSERT_EQUAL_INT(50, kline_p2max_from_byte(0x02));
    TEST_ASSERT_EQUAL_INT(6000, kline_p2max_from_byte(0xF0));

    /* Past that the low nibble counts 256 of them. */
    TEST_ASSERT_EQUAL_INT(1 * 256 * 25, kline_p2max_from_byte(0xF1));
    TEST_ASSERT_EQUAL_INT(15 * 256 * 25, kline_p2max_from_byte(0xFF));

    TEST_ASSERT_EQUAL_INT(0, kline_p2max_from_byte(0x00));
}

/* ------------------------------------------------------------------ *
 * Initialisation
 * ------------------------------------------------------------------ */

/*
 * The parameters of an initialisation attempt and what it established are
 * bus_init_t and bus_link_t now - they are the same shape on every bus that
 * has a handshake, and their contents are driver behaviour rather than
 * grammar. What stays here is the grammar those sequences are built from: the
 * StartCommunication request's own bytes, and the wakeup messages.
 */

TEST(the_start_communication_request_is_the_one_iso_14230_4_allows) {
    /* C1 33 F1 81 66: functional addressing, every emission related ECU, this
     * tester, StartCommunication - and the checksum the driver appends. */
    static const uint8_t req[] = {0xC1, KLINE_INIT_ADDR_OBD, 0xF1,
                                  KLINE_SVC_START_COMM};
    uint8_t msg[5];

    memcpy(msg, req, sizeof(req));
    msg[4] = kline_checksum(msg, 4);

    TEST_ASSERT_EQUAL_MEM("\xC1\x33\xF1\x81\x66", msg, 5);
    TEST_ASSERT_TRUE(kline_frame_ok(msg, sizeof(msg)));
    TEST_ASSERT_EQUAL_INT(5, kline_msg_len(msg, sizeof(msg)));
}

TEST(the_obd_initialisation_address_and_default_rate_are_the_legislated_ones) {
    TEST_ASSERT_EQUAL_INT(0x33, KLINE_INIT_ADDR_OBD);
    TEST_ASSERT_EQUAL_INT(10400, KLINE_BAUD_DEFAULT);
    /* Clause 5.1.5.2.2 allows 1200 to 10400; AT IB and J2534 both offer
     * faster rates for the manufacturer specific systems that use them. */
    TEST_ASSERT_EQUAL_INT(1200, KLINE_BAUD_MIN);
    TEST_ASSERT(KLINE_BAUD_HW_MAX >= KLINE_BAUD_MAX);
}

TEST(the_default_wakeup_messages_are_the_ones_the_datasheet_names) {
    uint8_t buf[KLINE_WAKEUP_MAX];
    size_t n;

    n = kline_default_wakeup(KLINE_VARIANT_ISO9141, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_INT(5, n);
    TEST_ASSERT_EQUAL_MEM("\x68\x6A\xF1\x01\x00", buf, 5);

    n = kline_default_wakeup(KLINE_VARIANT_KWP, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_INT(4, n);
    TEST_ASSERT_EQUAL_MEM("\xC1\x33\xF1\x3E", buf, 4);

    /* An unclassified link is treated as ISO 9141-2: a mode 01 request keeps
     * any OBD ECU awake, where a TesterPresent only keeps a KWP one awake. */
    n = kline_default_wakeup(KLINE_VARIANT_UNKNOWN, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_INT(5, n);

    TEST_ASSERT_EQUAL_INT(0,
                          kline_default_wakeup(KLINE_VARIANT_ISO9141, buf, 2));
    TEST_ASSERT_EQUAL_INT(0,
                          kline_default_wakeup(KLINE_VARIANT_ISO9141, NULL, 8));
}

TEST(a_wakeup_message_always_fits_the_at_wm_limit) {
    uint8_t buf[KLINE_WAKEUP_MAX];

    TEST_ASSERT(kline_default_wakeup(KLINE_VARIANT_ISO9141, buf, sizeof(buf)) <=
                KLINE_WAKEUP_MAX);
    TEST_ASSERT(kline_default_wakeup(KLINE_VARIANT_KWP, buf, sizeof(buf)) <=
                KLINE_WAKEUP_MAX);
}

/* ------------------------------------------------------------------ *
 * Response pending, clause 4.4.1
 * ------------------------------------------------------------------ */

TEST(a_response_pending_negative_response_is_recognised) {
    /* 83 F1 10 7F 01 78 CS: the ECU asking for the P3max window instead of
     * P2max. A receiver that misses this reports a timeout on every slow
     * service the vehicle offers. */
    uint8_t m[] = {0x83, 0xF1, 0x10, 0x7F, 0x01, 0x78, 0x00};

    m[6] = kline_checksum(m, 6);
    TEST_ASSERT_TRUE(kline_frame_ok(m, sizeof(m)));
    TEST_ASSERT_TRUE(kline_is_response_pending(m, sizeof(m)));
}

TEST(a_response_pending_is_found_behind_an_additional_length_byte) {
    uint8_t m[] = {0x80, 0xF1, 0x10, 0x03, 0x7F, 0x01, 0x78, 0x00};

    m[7] = kline_checksum(m, 7);
    TEST_ASSERT_EQUAL_INT(8, kline_msg_len(m, sizeof(m)));
    TEST_ASSERT_TRUE(kline_is_response_pending(m, sizeof(m)));
}

TEST(other_negative_responses_are_not_response_pending) {
    /* $11 is serviceNotSupported: a final answer, not a request for time. */
    uint8_t m[] = {0x83, 0xF1, 0x10, 0x7F, 0x01, 0x11, 0x00};
    uint8_t ok[] = {0x83, 0xF1, 0x10, 0x41, 0x00, 0xBE, 0x00};

    TEST_ASSERT_FALSE(kline_is_response_pending(m, sizeof(m)));
    TEST_ASSERT_FALSE(kline_is_response_pending(ok, sizeof(ok)));
    TEST_ASSERT_FALSE(kline_is_response_pending(m, 2));
    TEST_ASSERT_FALSE(kline_is_response_pending(NULL, 7));
}

/* ------------------------------------------------------------------ *
 * Sweep
 * ------------------------------------------------------------------ */

/** xorshift32, so the sweep is the same run to run and the same everywhere. */
static uint32_t rnd(uint32_t *s) {
    uint32_t x = *s;

    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *s = x;
    return x;
}

TEST(arbitrary_bytes_never_make_the_grammar_misbehave) {
    uint32_t seed = 0x5EED1234u;

    for (int iter = 0; iter < 20000; iter++) {
        uint8_t buf[KLINE_MAX_MSG];
        size_t len = 1 + rnd(&seed) % 24;
        size_t n;

        for (size_t i = 0; i < len; i++) {
            buf[i] = (uint8_t)rnd(&seed);
        }

        n = kline_msg_len(buf, len);
        TEST_ASSERT_MSG(n == 0 || (n >= KLINE_MIN_MSG && n <= KLINE_MAX_MSG),
                        "claimed %u bytes for [%s]", (unsigned)n,
                        td_hex(buf, len));

        /* Both of these have to reach a verdict on any input at all, because
         * the receive path calls them on every byte that arrives. */
        (void)kline_frame_ok(buf, len);
        (void)kline_is_response_pending(buf, len);
    }
}

TEST(a_well_formed_message_is_always_recognised_at_its_own_length) {
    uint32_t seed = 0x1234ABCDu;

    /* Build messages the way the driver builds them and confirm the length
     * calculation and the checksum agree - this is the pair the receiver's
     * fast path relies on, and disagreement between them would show up as a
     * bus that mostly works. */
    for (int iter = 0; iter < 5000; iter++) {
        uint8_t buf[KLINE_MAX_MSG];
        size_t data = 1 + rnd(&seed) % 60;
        bool addresses = (rnd(&seed) & 1) != 0;
        bool len_byte = (rnd(&seed) & 1) != 0;
        size_t n = 0;

        buf[n++] = (uint8_t)((addresses ? 0x80u : 0x00u) |
                             (len_byte ? 0u : (data & 0x3Fu)));
        if (addresses) {
            buf[n++] = 0x10;
            buf[n++] = 0xF1;
        }
        if (len_byte) {
            buf[n++] = (uint8_t)data;
        }
        for (size_t i = 0; i < data; i++) {
            buf[n++] = (uint8_t)rnd(&seed);
        }
        buf[n] = kline_checksum(buf, n);
        n++;

        TEST_ASSERT_MSG(kline_msg_len(buf, n) == n,
                        "message of %u bytes measured %u", (unsigned)n,
                        (unsigned)kline_msg_len(buf, n));
        TEST_ASSERT_TRUE(kline_frame_ok(buf, n));
    }
}
