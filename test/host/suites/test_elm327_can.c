/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_elm327_can.c
 * @brief ISO 15765-4 request framing, reply filtering and output formatting.
 */

#include "elm327_harness.h"

#define CAN_11BIT_500K "ATSP6\r"
#define CAN_29BIT_500K "ATSP7\r"
#define CAN_11BIT_250K "ATSP8\r"

/** Puts the adapter on 11 bit 500 kbaud CAN with echo off. */
static void on_can_11bit(void)
{
    elm_echo_off();
    elm_ok(CAN_11BIT_500K);
}

/** A typical mode 01 PID 00 reply from engine ECU 0x7E8. */
static const uint8_t supported_pids[8] = {
    0x06, 0x41, 0x00, 0xBE, 0x3F, 0xB8, 0x13, 0x00
};

/* ------------------------------------------------------------------ *
 * Request framing
 * ------------------------------------------------------------------ */

TEST(a_request_is_wrapped_in_a_single_frame_pci_byte)
{
    on_can_11bit();

    elm_ask("0100\r");

    const struct can_frame *tx = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(tx);
    TEST_ASSERT_EQUAL_HEX32(0x7DF, tx->id);
    TEST_ASSERT_EQUAL_INT(3, tx->dlc);
    TEST_ASSERT_EQUAL_MEM("\x02\x01\x00", tx->data, 3);
}

TEST(auto_formatting_off_sends_the_bytes_untouched)
{
    on_can_11bit();
    elm_ok("ATCAF0\r");

    elm_ask("02010D\r");

    const struct can_frame *tx = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(tx);
    TEST_ASSERT_EQUAL_INT(3, tx->dlc);
    TEST_ASSERT_EQUAL_MEM("\x02\x01\x0D", tx->data, 3);
}

TEST(eight_bytes_will_not_fit_a_formatted_frame_and_are_refused)
{
    on_can_11bit();

    /*
     * Auto formatting is on by default and spends one of the eight data bytes
     * on the ISO-TP length byte, so eight bytes of payload cannot go out. This
     * used to trip an assert and take the adapter down - reachable by anyone
     * who could open the port, or connect over BLE, which needs no pairing.
     */
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("1122334455667788\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}

TEST(seven_bytes_is_the_most_a_formatted_frame_carries)
{
    on_can_11bit();

    elm_ask("11223344556677\r");

    const struct can_frame *tx = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(tx);
    TEST_ASSERT_EQUAL_INT(8, tx->dlc);
    TEST_ASSERT_EQUAL_MEM("\x07\x11\x22\x33\x44\x55\x66\x77", tx->data, 8);
}

TEST(eight_bytes_go_out_untouched_with_formatting_off)
{
    on_can_11bit();
    elm_ok("ATCAF0\r");

    /* The same eight bytes the formatted path has to refuse: AT CAF0 is how
     * a client sends a raw frame, and it must still work. */
    elm_ask("1122334455667788\r");

    const struct can_frame *tx = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(tx);
    TEST_ASSERT_EQUAL_INT(8, tx->dlc);
    TEST_ASSERT_EQUAL_MEM("\x11\x22\x33\x44\x55\x66\x77\x88", tx->data, 8);
}

TEST(a_twenty_nine_bit_protocol_flags_the_transmitted_id_as_extended)
{
    elm_echo_off();
    elm_ok(CAN_29BIT_500K);

    elm_ask("0100\r");

    const struct can_frame *tx = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(tx);
    TEST_ASSERT_EQUAL_HEX32(0x18DB33F1 | CAN_EFF_FLAG, tx->id);
}

TEST(at_sh_overrides_the_transmitted_id)
{
    on_can_11bit();
    elm_ok("ATSH7E0\r");

    elm_ask("0100\r");

    const struct can_frame *tx = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(tx);
    TEST_ASSERT_EQUAL_HEX32(0x7E0, tx->id);
}

TEST(the_250_kbaud_protocols_configure_the_slower_bus)
{
    elm_echo_off();
    elm_ok(CAN_11BIT_250K);

    TEST_ASSERT_EQUAL_INT(250000, fake_can_last_baud());
}

TEST(a_failed_transmit_is_reported)
{
    on_can_11bit();
    fake_can_fail_next_send();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("0100\r"));
}

/* ------------------------------------------------------------------ *
 * Single frame replies
 * ------------------------------------------------------------------ */

TEST(a_single_frame_reply_is_printed_with_its_id)
{
    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(spacing_off_packs_the_reply)
{
    on_can_11bit();
    elm_ok("ATS0\r");
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    TEST_ASSERT_EQUAL_STRING("7E8064100BE3FB81300\r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(a_twenty_nine_bit_reply_is_printed_as_eight_hex_digits)
{
    static const uint8_t reply[8] = {
        0x06, 0x41, 0x00, 0xBE, 0x3F, 0xB8, 0x13, 0x00
    };

    elm_echo_off();
    elm_ok(CAN_29BIT_500K);
    fake_can_stage_response(0x18DAF110 | CAN_EFF_FLAG, 8, reply, 5);

    TEST_ASSERT_EQUAL_STRING("18DAF110 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(spacing_off_packs_a_twenty_nine_bit_reply)
{
    elm_echo_off();
    elm_ok(CAN_29BIT_500K);
    elm_ok("ATS0\r");
    fake_can_stage_response(0x18DAF110 | CAN_EFF_FLAG, 8, supported_pids, 5);

    TEST_ASSERT_EQUAL_STRING("18DAF110064100BE3FB81300\r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(a_silent_bus_reports_no_data)
{
    on_can_11bit();

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(replies_from_other_ids_are_filtered_out)
{
    static const uint8_t noise[8] = { 0x08, 0xAA, 0, 0, 0, 0, 0, 0 };

    on_can_11bit();
    fake_can_stage_response(0x123, 8, noise, 1);
    fake_can_stage_response(0x7E8, 8, supported_pids, 1);

    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(traffic_on_the_request_ids_is_not_mistaken_for_a_reply)
{
    static const uint8_t another_tester[8] = { 0x02, 0x01, 0x00, 0, 0, 0, 0, 0 };

    on_can_11bit();

    /* 0x7E0 to 0x7E7 are the diagnostic request addresses. Seeing one means
     * another tester is talking, not that an ECU answered us. */
    fake_can_stage_response(0x7E0, 8, another_tester, 1);
    fake_can_stage_response(0x7E8, 8, supported_pids, 1);

    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(replies_from_the_higher_ecu_addresses_are_accepted)
{
    static const uint8_t from_tcm[8] = { 0x03, 0x41, 0x00, 0x80, 0, 0, 0, 0 };

    on_can_11bit();

    /* A broadcast request is answered by every module: 0x7E8 through 0x7EF. */
    fake_can_stage_response(0x7E8, 8, supported_pids, 1);
    fake_can_stage_response(0x7EF, 8, from_tcm, 1);

    TEST_ASSERT_EQUAL_STRING(
        "7E8 06 41 00 BE 3F B8 13 00 \r"
        "7EF 03 41 00 80 00 00 00 00 \r" ELM_PROMPT,
        elm_ask("0100\r"));
}

TEST(frames_already_queued_are_discarded_before_the_request_goes_out)
{
    static const uint8_t leftover[8] = { 0x03, 0x41, 0x0C, 0x1A, 0, 0, 0, 0 };

    on_can_11bit();

    /* A reply to somebody else's request, still sitting in the driver. */
    fake_can_stage_stale(0x7E8, 8, leftover);
    fake_can_stage_response(0x7E8, 8, supported_pids, 1);

    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(a_directed_request_returns_as_soon_as_its_ecu_answers)
{
    on_can_11bit();
    elm_ok("ATSH7E0\r");
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    uint32_t before = fake_clock_ms();
    elm_ask("0100\r");
    uint32_t waited = fake_clock_ms() - before;

    /* Addressed to one ECU, so there is nothing to wait around for. */
    TEST_ASSERT_MSG(waited < 200, "waited %u ms after the reply arrived", waited);
}

TEST(a_broadcast_request_keeps_listening_until_the_timeout)
{
    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    uint32_t before = fake_clock_ms();
    elm_ask("0100\r");
    uint32_t waited = fake_clock_ms() - before;

    /* 0x7DF may be answered by several ECUs, so the window stays open. */
    TEST_ASSERT_MSG(waited >= 200, "gave other ECUs only %u ms", waited);
}

/* ------------------------------------------------------------------ *
 * Frame count hint
 * ------------------------------------------------------------------ */

TEST(a_frame_count_hint_stops_the_wait_early)
{
    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    uint32_t before = fake_clock_ms();
    /* The odd trailing digit says how many frames to expect. */
    const char *out = elm_ask("01001\r");
    uint32_t waited = fake_clock_ms() - before;

    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT, out);
    TEST_ASSERT_MSG(waited < 200, "waited %u ms despite the hint", waited);
}

TEST(a_zero_frame_hint_sends_without_waiting_for_an_answer)
{
    on_can_11bit();

    TEST_ASSERT_EQUAL_STRING(ELM_PROMPT, elm_ask("01000\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
}

/* ------------------------------------------------------------------ *
 * Multi frame replies
 * ------------------------------------------------------------------ */

/* Mode 09 PID 02, a VIN, split over a first frame and two consecutive ones. */
static const uint8_t vin_ff[8]  = { 0x10, 0x14, 0x49, 0x02, 0x01, 0x31, 0x32, 0x33 };
static const uint8_t vin_cf1[8] = { 0x21, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x41 };
static const uint8_t vin_cf2[8] = { 0x22, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48 };

static void stage_vin_reply(void)
{
    fake_can_stage_response(0x7E8, 8, vin_ff, 5);
    /* A real ECU holds the rest back until it sees the flow control frame. */
    fake_can_stage_response_at(0x7E8, 8, vin_cf1, 5, 2);
    fake_can_stage_response_at(0x7E8, 8, vin_cf2, 5, 2);
}

TEST(a_first_frame_is_answered_with_a_clear_to_send)
{
    on_can_11bit();
    stage_vin_reply();

    elm_ask("0902\r");

    const struct can_frame *fc = fake_can_sent(1);
    TEST_ASSERT_NOT_NULL(fc);
    TEST_ASSERT_EQUAL_INT(3, fc->dlc);
    TEST_ASSERT_EQUAL_MEM("\x30\x00\x00", fc->data, 3);
}

TEST(every_frame_of_a_multi_frame_reply_is_printed_in_order)
{
    on_can_11bit();
    stage_vin_reply();

    TEST_ASSERT_EQUAL_STRING(
        "7E8 10 14 49 02 01 31 32 33 \r"
        "7E8 21 34 35 36 37 38 39 41 \r"
        "7E8 22 42 43 44 45 46 47 48 \r" ELM_PROMPT,
        elm_ask("0902\r"));
}

TEST(a_complete_multi_frame_reply_ends_the_exchange_immediately)
{
    on_can_11bit();
    stage_vin_reply();

    uint32_t before = fake_clock_ms();
    elm_ask("0902\r");
    uint32_t waited = fake_clock_ms() - before;

    /* The first frame declared the length, so there is nothing left to wait
     * for once that many bytes have arrived. */
    TEST_ASSERT_MSG(waited < 200, "waited %u ms after the last frame", waited);
}

TEST(a_gap_in_the_consecutive_frame_sequence_is_an_error)
{
    static const uint8_t out_of_order[8] = {
        0x22, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x41
    };

    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, vin_ff, 5);
    fake_can_stage_response_at(0x7E8, 8, out_of_order, 5, 2);

    /* Index 2 where index 1 was due: a frame went missing. */
    TEST_ASSERT_EQUAL_STRING("7E8 10 14 49 02 01 31 32 33 \r?\r" ELM_PROMPT,
                             elm_ask("0902\r"));
}

TEST(a_second_first_frame_before_the_transfer_finishes_is_an_error)
{
    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, vin_ff, 5);
    fake_can_stage_response_at(0x7E8, 8, vin_ff, 5, 2);

    /* Two multi-frame transfers cannot be reassembled at once. */
    TEST_ASSERT_EQUAL_STRING("7E8 10 14 49 02 01 31 32 33 \r?\r" ELM_PROMPT,
                             elm_ask("0902\r"));
}

TEST(an_unrecognised_pci_nibble_is_an_error)
{
    static const uint8_t bad_pci[8] = { 0x40, 0x01, 0, 0, 0, 0, 0, 0 };

    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, bad_pci, 5);

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_flow_control_frame_from_the_ecu_is_handed_to_the_client)
{
    static const uint8_t fc[8] = { 0x30, 0x00, 0x00, 0, 0, 0, 0, 0 };

    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, fc, 5);

    TEST_ASSERT_EQUAL_STRING("7E8 30 00 00 00 00 00 00 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

/* ------------------------------------------------------------------ *
 * Long multi-frame replies
 *
 * Two limits used to sit about seven payload bytes apart here: the 4 bit
 * consecutive frame index was compared against a counter that did not wrap,
 * capping a transfer at 111 bytes, and behind it the 512 byte tx ring buffer
 * would deadlock the ELM327 task once a response outgrew it. Both are fixed;
 * these tests hold that ground.
 * ------------------------------------------------------------------ */

/** Stages an ISO-TP reply of @p payload bytes: one first frame, then CFs. */
static void stage_isotp_reply(int payload)
{
    uint8_t f[8];
    int carried = 6, idx = 1;

    f[0] = 0x10 | ((payload >> 8) & 0x0f);
    f[1] = payload & 0xff;
    for (int i = 2; i < 8; i++) {
        f[i] = (uint8_t)(0xA0 + i);
    }
    fake_can_stage_response(0x7E8, 8, f, 1);

    while (carried < payload) {
        f[0] = (uint8_t)(0x20 | (idx & 0x0f));
        for (int i = 1; i < 8; i++) {
            f[i] = (uint8_t)(idx * 16 + i);
        }
        fake_can_stage_response_at(0x7E8, 8, f, 1, 2);
        carried += 7;
        idx++;
    }
}

/** Lines the adapter printed, excluding the trailing prompt. */
static int lines_printed(const char *s)
{
    int n = 0;

    for (const char *p = s; *p; p++) {
        if (*p == '\r') {
            n++;
        }
    }

    return n - 1;
}

TEST(a_reply_of_fifteen_consecutive_frames_is_reassembled)
{
    on_can_11bit();
    stage_isotp_reply(111);

    /* 6 bytes in the first frame plus 15 x 7 is the largest that works. */
    TEST_ASSERT_EQUAL_INT(16, lines_printed(elm_ask("22F190\r")));
}

TEST(a_reply_crossing_the_sequence_number_rollover_is_reassembled)
{
    on_can_11bit();
    stage_isotp_reply(118);

    /* 16 consecutive frames, so the sequence number runs 1..15 then back to
     * 0. Comparing it against an unmasked counter used to reject this. */
    const char *out = elm_ask("22F190\r");

    TEST_ASSERT_MSG(strstr(out, "?") == NULL, "transfer was rejected: \"%s\"",
                    td_escape(out, fake_port_len(0)));
    TEST_ASSERT_EQUAL_INT(17, lines_printed(out));
}

TEST(a_reply_larger_than_the_tx_buffer_streams_out_in_full)
{
    on_can_11bit();
    stage_isotp_reply(400);

    /* 58 frames is about 1.7 kB of text through a 512 byte tx ring buffer, so
     * it only arrives if the buffer is drained as it fills rather than once
     * at the end of the command. */
    const char *out = elm_ask("22F190\r");

    TEST_ASSERT_MSG(strstr(out, "?") == NULL, "transfer was rejected: \"%s\"",
                    td_escape(out, fake_port_len(0)));
    TEST_ASSERT_EQUAL_INT(58, lines_printed(out));

    /* The tail has to be intact, not just the frame count: a mid-transfer
     * flush must not lose or reorder what it was holding. */
    TEST_ASSERT_MSG(strstr(out, "7E8 29 91 92 93 94 95 96 97 \r") != NULL,
                    "the last consecutive frame is missing or corrupted");
}

TEST(a_long_reply_arrives_in_order)
{
    on_can_11bit();
    stage_isotp_reply(200);

    const char *out = elm_ask("22F190\r");
    const char *first = strstr(out, "7E8 21 ");
    const char *second = strstr(out, "7E8 22 ");
    const char *wrapped = strstr(out, "7E8 20 ");

    TEST_ASSERT_NOT_NULL(first);
    TEST_ASSERT_NOT_NULL(second);
    TEST_ASSERT_MSG(first < second, "consecutive frames came out reordered");

    /* 200 bytes needs 28 consecutive frames, so the sequence number wraps
     * past 15 and back to 0 once. */
    TEST_ASSERT_NOT_NULL(wrapped);
    TEST_ASSERT_MSG(second < wrapped, "the rollover frame came out too early");
}

/* ------------------------------------------------------------------ *
 * Response pending
 * ------------------------------------------------------------------ */

TEST(a_response_pending_reply_extends_the_wait)
{
    /* Negative response 0x7F, code 0x78: "busy, answer coming". */
    static const uint8_t pending[8] = { 0x03, 0x7F, 0x01, 0x78, 0, 0, 0, 0 };
    static const uint8_t answer[8]  = { 0x06, 0x41, 0x00, 0xBE, 0x3F, 0xB8, 0x13, 0x00 };

    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, pending, 5);
    /* Well past the 200 ms default, but inside the extended window. */
    fake_can_stage_response(0x7E8, 8, answer, 900);

    TEST_ASSERT_EQUAL_STRING(
        "7E8 03 7F 01 78 00 00 00 00 \r"
        "7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
        elm_ask("0100\r"));
}

TEST(a_reply_after_the_timeout_is_missed)
{
    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, supported_pids, 5000);

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}
