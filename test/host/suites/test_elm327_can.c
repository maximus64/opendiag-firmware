/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_elm327_can.c
 * @brief ISO 15765-4 request framing, reply filtering and output formatting.
 */

#include "elm327_harness.h"

#define CAN_11BIT_500K "ATSP6\r"
#define CAN_29BIT_500K "ATSP7\r"
#define CAN_11BIT_250K "ATSP8\r"

/**
 * @brief Puts the adapter on 11 bit 500 kbaud CAN, showing frames as received.
 *
 * Headers on, which is not the power-on default. Almost everything below is
 * about the transport - which identifier a reply came from, what its PCI byte
 * said, whether a consecutive frame was in sequence - and AT H1 is the mode
 * that shows a frame the way it arrived: "turning the display of headers on
 * (with AT H1) will override some of the CAF1 formatting of the received
 * data, so that the received bytes will appear much like in the CAF0 mode".
 *
 * The default view, where the transport is stripped back out again, has its
 * own section at the end of this file.
 */
static void on_can_11bit(void) {
    elm_echo_off();
    elm_ok(CAN_11BIT_500K);
    elm_ok("ATH1\r");
}

/** The same, on 29 bit 500 kbaud CAN. */
static void on_can_29bit(void) {
    elm_echo_off();
    elm_ok(CAN_29BIT_500K);
    elm_ok("ATH1\r");
}

/** A typical mode 01 PID 00 reply from engine ECU 0x7E8. */
static const uint8_t supported_pids[8] = {0x06, 0x41, 0x00, 0xBE,
                                          0x3F, 0xB8, 0x13, 0x00};

/* ------------------------------------------------------------------ *
 * Request framing
 * ------------------------------------------------------------------ */

TEST(a_request_is_wrapped_in_a_single_frame_pci_byte) {
    on_can_11bit();

    elm_ask("0100\r");

    const struct can_frame *tx = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(tx);
    TEST_ASSERT_EQUAL_HEX32(0x7DF, tx->id);
    TEST_ASSERT_EQUAL_INT(8, tx->dlc);
    TEST_ASSERT_EQUAL_MEM("\x02\x01\x00\x00\x00\x00\x00\x00", tx->data, 8);
}

TEST(auto_formatting_off_sends_the_bytes_untouched) {
    on_can_11bit();
    elm_ok("ATCAF0\r");

    elm_ask("02010D\r");

    const struct can_frame *tx = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(tx);
    TEST_ASSERT_EQUAL_INT(8, tx->dlc);
    TEST_ASSERT_EQUAL_MEM("\x02\x01\x0D\x00\x00\x00\x00\x00", tx->data, 8);
}

TEST(eight_bytes_will_not_fit_a_formatted_frame_and_are_refused) {
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

TEST(seven_bytes_is_the_most_a_formatted_frame_carries) {
    on_can_11bit();

    elm_ask("11223344556677\r");

    const struct can_frame *tx = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(tx);
    TEST_ASSERT_EQUAL_INT(8, tx->dlc);
    TEST_ASSERT_EQUAL_MEM("\x07\x11\x22\x33\x44\x55\x66\x77", tx->data, 8);
}

TEST(eight_bytes_go_out_untouched_with_formatting_off) {
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

TEST(a_twenty_nine_bit_protocol_flags_the_transmitted_id_as_extended) {
    elm_echo_off();
    elm_ok(CAN_29BIT_500K);

    elm_ask("0100\r");

    const struct can_frame *tx = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(tx);
    TEST_ASSERT_EQUAL_HEX32(0x18DB33F1 | CAN_EFF_FLAG, tx->id);
    TEST_ASSERT_EQUAL_INT(8, tx->dlc);
}

TEST(at_sh_overrides_the_transmitted_id) {
    on_can_11bit();
    elm_ok("ATSH7E0\r");

    elm_ask("0100\r");

    const struct can_frame *tx = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(tx);
    TEST_ASSERT_EQUAL_HEX32(0x7E0, tx->id);
    TEST_ASSERT_EQUAL_INT(8, tx->dlc);
}

TEST(unused_bytes_are_padded_to_make_a_full_length_frame) {
    on_can_11bit();

    elm_ask("0100\r");

    const struct can_frame *tx = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(tx);
    TEST_ASSERT_EQUAL_INT(8, tx->dlc);
    TEST_ASSERT_EQUAL_MEM("\x00\x00\x00\x00\x00", &tx->data[3], 5);
}

TEST(the_250_kbaud_protocols_configure_the_slower_bus) {
    elm_echo_off();
    elm_ok(CAN_11BIT_250K);

    TEST_ASSERT_EQUAL_INT(250000, fake_can_last_baud());
}

TEST(a_failed_transmit_is_reported) {
    on_can_11bit();
    fake_can_fail_next_send();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("0100\r"));
}

/* ------------------------------------------------------------------ *
 * Single frame replies
 * ------------------------------------------------------------------ */

TEST(a_single_frame_reply_is_printed_with_its_id) {
    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(spacing_off_packs_the_reply) {
    on_can_11bit();
    elm_ok("ATS0\r");
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    TEST_ASSERT_EQUAL_STRING("7E8064100BE3FB81300\r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(a_twenty_nine_bit_reply_is_printed_as_eight_hex_digits) {
    static const uint8_t reply[8] = {0x06, 0x41, 0x00, 0xBE,
                                     0x3F, 0xB8, 0x13, 0x00};

    on_can_29bit();
    fake_can_stage_response(0x18DAF110 | CAN_EFF_FLAG, 8, reply, 5);

    TEST_ASSERT_EQUAL_STRING("18DAF110 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(spacing_off_packs_a_twenty_nine_bit_reply) {
    on_can_29bit();
    elm_ok("ATS0\r");
    fake_can_stage_response(0x18DAF110 | CAN_EFF_FLAG, 8, supported_pids, 5);

    TEST_ASSERT_EQUAL_STRING("18DAF110064100BE3FB81300\r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(a_silent_bus_reports_no_data) {
    on_can_11bit();

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(replies_from_other_ids_are_filtered_out) {
    static const uint8_t noise[8] = {0x08, 0xAA, 0, 0, 0, 0, 0, 0};

    on_can_11bit();
    fake_can_stage_response(0x123, 8, noise, 1);
    fake_can_stage_response(0x7E8, 8, supported_pids, 1);

    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(traffic_on_the_request_ids_is_not_mistaken_for_a_reply) {
    static const uint8_t another_tester[8] = {0x02, 0x01, 0x00, 0, 0, 0, 0, 0};

    on_can_11bit();

    /* 0x7E0 to 0x7E7 are the diagnostic request addresses. Seeing one means
     * another tester is talking, not that an ECU answered us. */
    fake_can_stage_response(0x7E0, 8, another_tester, 1);
    fake_can_stage_response(0x7E8, 8, supported_pids, 1);

    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(replies_from_the_higher_ecu_addresses_are_accepted) {
    static const uint8_t from_tcm[8] = {0x03, 0x41, 0x00, 0x80, 0, 0, 0, 0};

    on_can_11bit();

    /* A broadcast request is answered by every module: 0x7E8 through 0x7EF. */
    fake_can_stage_response(0x7E8, 8, supported_pids, 1);
    fake_can_stage_response(0x7EF, 8, from_tcm, 1);

    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r"
                             "7EF 03 41 00 80 00 00 00 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(frames_already_queued_are_discarded_before_the_request_goes_out) {
    static const uint8_t leftover[8] = {0x03, 0x41, 0x0C, 0x1A, 0, 0, 0, 0};

    on_can_11bit();

    /* A reply to somebody else's request, still sitting in the driver. */
    fake_can_stage_stale(0x7E8, 8, leftover);
    fake_can_stage_response(0x7E8, 8, supported_pids, 1);

    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(a_directed_request_returns_as_soon_as_its_ecu_answers) {
    on_can_11bit();
    elm_ok("ATSH7E0\r");
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    uint32_t before = fake_clock_ms();
    elm_ask("0100\r");
    uint32_t waited = fake_clock_ms() - before;

    /* Addressed to one ECU, so there is nothing to wait around for. */
    TEST_ASSERT_MSG(waited < 200, "waited %u ms after the reply arrived",
                    waited);
}

TEST(a_broadcast_request_keeps_listening_until_the_timeout) {
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

TEST(a_frame_count_hint_stops_the_wait_early) {
    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    uint32_t before = fake_clock_ms();
    /* The odd trailing digit says how many frames to expect. */
    const char *out = elm_ask("01001\r");
    uint32_t waited = fake_clock_ms() - before;

    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT, out);
    TEST_ASSERT_MSG(waited < 200, "waited %u ms despite the hint", waited);
}

TEST(a_zero_frame_hint_sends_without_waiting_for_an_answer) {
    on_can_11bit();

    TEST_ASSERT_EQUAL_STRING(ELM_PROMPT, elm_ask("01000\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
}

/* ------------------------------------------------------------------ *
 * Multi frame replies
 * ------------------------------------------------------------------ */

/* Mode 09 PID 02, a VIN, split over a first frame and two consecutive ones. */
static const uint8_t vin_ff[8] = {0x10, 0x14, 0x49, 0x02,
                                  0x01, 0x31, 0x32, 0x33};
static const uint8_t vin_cf1[8] = {0x21, 0x34, 0x35, 0x36,
                                   0x37, 0x38, 0x39, 0x41};
static const uint8_t vin_cf2[8] = {0x22, 0x42, 0x43, 0x44,
                                   0x45, 0x46, 0x47, 0x48};

static void stage_vin_reply(void) {
    fake_can_stage_response(0x7E8, 8, vin_ff, 5);
    /* A real ECU holds the rest back until it sees the flow control frame. */
    fake_can_stage_response_at(0x7E8, 8, vin_cf1, 5, 2);
    fake_can_stage_response_at(0x7E8, 8, vin_cf2, 5, 2);
}

TEST(a_first_frame_is_answered_with_a_clear_to_send) {
    on_can_11bit();
    stage_vin_reply();

    elm_ask("0902\r");

    const struct can_frame *fc = fake_can_sent(1);
    TEST_ASSERT_NOT_NULL(fc);
    TEST_ASSERT_EQUAL_INT(8, fc->dlc);
    TEST_ASSERT_EQUAL_MEM("\x30\x00\x00\x00\x00\x00\x00\x00", fc->data, 8);
}

TEST(flow_control_is_addressed_to_the_ecu_that_sent_the_first_frame) {
    on_can_11bit();
    stage_vin_reply();

    elm_ask("0902\r");

    /* ISO 15765-2 clause 6.4: flow control belongs to the connection the
     * first frame opened, which is the physical channel to the ECU that sent
     * it. This request went out on 0x7DF, the functional address every
     * emission related ECU listens to, and sending the flow control back
     * there asks all of them to act on one ECU's window. */
    const struct can_frame *fc = fake_can_sent(1);
    TEST_ASSERT_NOT_NULL(fc);
    TEST_ASSERT_EQUAL_HEX32(0x7E0, fc->id);
}

TEST(flow_control_on_a_29_bit_bus_is_addressed_to_the_responding_ecu) {
    static const uint8_t ff[8] = {0x10, 0x14, 0x49, 0x02,
                                  0x01, 0x31, 0x32, 0x33};

    on_can_29bit();
    fake_can_stage_response(0x18DAF110 | CAN_EFF_FLAG, 8, ff, 5);

    elm_ask("0902\r");

    /* 0x18DAF110 is ECU 0x10 answering tester 0xF1, so its own address is
     * 0x18DA10F1 - target and source the other way round. */
    const struct can_frame *fc = fake_can_sent(1);
    TEST_ASSERT_NOT_NULL(fc);
    TEST_ASSERT_EQUAL_HEX32(0x18DA10F1 | CAN_EFF_FLAG, fc->id);
}

/* ------------------------------------------------------------------ *
 * The receive filter
 * ------------------------------------------------------------------ */

TEST(an_identifier_just_past_the_diagnostic_range_is_not_a_reply) {
    static const uint8_t noise[8] = {0x06, 0x41, 0x00, 0, 0, 0, 0, 0};

    on_can_11bit();
    fake_can_stage_response(0x7F8, 8, noise, 1);

    /* ISO 15765-4 clause 8 stops at 0x7EF. The filter used to ask whether the
     * bits of 0x7E8 were set rather than whether the identifier was in range,
     * which let all of 0x7F8 to 0x7FF through - and those carry traffic that
     * is nobody's answer to a diagnostic request. */
    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(an_11_bit_protocol_does_not_accept_a_29_bit_frame) {
    static const uint8_t reply[8] = {0x06, 0x41, 0x00, 0, 0, 0, 0, 0};

    on_can_11bit();
    fake_can_stage_response(0x18DAF110 | CAN_EFF_FLAG, 8, reply, 1);

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_29_bit_protocol_does_not_accept_an_11_bit_frame) {
    on_can_29bit();
    fake_can_stage_response(0x7E8, 8, supported_pids, 1);

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

/* ------------------------------------------------------------------ *
 * The default view: AT H0 with auto formatting on
 * ------------------------------------------------------------------ *
 *
 * Both are the power-on state, so this is what a client that sends no display
 * commands at all sees. "the formatting (PCI) bytes will be automatically
 * generated for you when sending, and will be removed when receiving. This
 * means that you can continue to issue OBD requests (01 00, etc.) as usual,
 * without regard to the extra bytes that CAN diagnostics systems require."
 *
 * On CAN none of that was happening: the identifier and the PCI byte were
 * printed whatever AT H said, so a client relying on the documented default
 * had four bytes of transport in front of every reply.
 */

TEST(the_default_view_shows_the_data_without_the_transport) {
    elm_echo_off();
    elm_ok(CAN_11BIT_500K);
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    /* The PCI byte said six, so the padding byte behind the data goes too:
     * "any extra (unused) data bytes that are received in the frame will be
     * removed". */
    TEST_ASSERT_EQUAL_STRING("41 00 BE 3F B8 13 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(the_default_view_packs_the_data_with_spacing_off) {
    elm_echo_off();
    elm_ok(CAN_11BIT_500K);
    elm_ok("ATS0\r");
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    TEST_ASSERT_EQUAL_STRING("4100BE3FB813\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(auto_formatting_off_shows_the_pci_byte_without_the_identifier) {
    elm_echo_off();
    elm_ok(CAN_11BIT_500K);
    elm_ok("ATCAF0\r");
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    /* AT CAF0 hands the frame over as it arrived. AT H0 still hides the
     * identifier: the two settings are independent. */
    TEST_ASSERT_EQUAL_STRING("06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(a_multi_frame_reply_in_the_default_view_is_a_length_and_segments) {
    elm_echo_off();
    elm_ok(CAN_11BIT_500K);
    stage_vin_reply();

    /* The datasheet's worked 0902 example, exactly: the total length once,
     * then each frame behind its sequence number. Nothing is reassembled -
     * "CAN systems add this single hex digit ... to aid in reassembling", so
     * the client is the one that assembles. */
    TEST_ASSERT_EQUAL_STRING("014\r"
                             "0: 49 02 01 31 32 33 \r"
                             "1: 34 35 36 37 38 39 41 \r"
                             "2: 42 43 44 45 46 47 48 \r" ELM_PROMPT,
                             elm_ask("0902\r"));
}

TEST(a_multi_frame_reply_in_the_default_view_packs_with_spacing_off) {
    elm_echo_off();
    elm_ok(CAN_11BIT_500K);
    elm_ok("ATS0\r");
    stage_vin_reply();

    TEST_ASSERT_EQUAL_STRING("014\r"
                             "0:490201313233\r"
                             "1:34353637383941\r"
                             "2:42434445464748\r" ELM_PROMPT,
                             elm_ask("0902\r"));
}

/* ------------------------------------------------------------------ *
 * AT R0
 * ------------------------------------------------------------------ */

TEST(responses_off_sends_without_printing_a_reply) {
    on_can_11bit();
    elm_ok("ATR0\r");
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    /* Honoured on the byte buses all along, ignored on CAN: the request went
     * out, the full reply window was waited out anyway, and the reply the
     * client had said it did not want was printed at the end of it. */
    TEST_ASSERT_EQUAL_STRING(ELM_PROMPT, elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
}

TEST(every_frame_of_a_multi_frame_reply_is_printed_in_order) {
    on_can_11bit();
    stage_vin_reply();

    TEST_ASSERT_EQUAL_STRING("7E8 10 14 49 02 01 31 32 33 \r"
                             "7E8 21 34 35 36 37 38 39 41 \r"
                             "7E8 22 42 43 44 45 46 47 48 \r" ELM_PROMPT,
                             elm_ask("0902\r"));
}

TEST(a_complete_directed_multi_frame_reply_ends_the_exchange_immediately) {
    on_can_11bit();
    elm_ok("ATSH7E0\r");
    stage_vin_reply();

    uint32_t before = fake_clock_ms();
    elm_ask("0902\r");
    uint32_t waited = fake_clock_ms() - before;

    /* The first frame declared the length, so there is nothing left to wait
     * for once that many bytes have arrived. */
    TEST_ASSERT_MSG(waited < 200, "waited %u ms after the last frame", waited);
}

TEST(a_gap_in_the_consecutive_frame_sequence_is_an_error) {
    static const uint8_t out_of_order[8] = {0x22, 0x34, 0x35, 0x36,
                                            0x37, 0x38, 0x39, 0x41};

    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, vin_ff, 5);
    fake_can_stage_response_at(0x7E8, 8, out_of_order, 5, 2);

    /* Index 2 where index 1 was due: a frame went missing. */
    TEST_ASSERT_EQUAL_STRING("7E8 10 14 49 02 01 31 32 33 \r?\r" ELM_PROMPT,
                             elm_ask("0902\r"));
}

TEST(a_new_first_frame_restarts_reception_and_reports_the_old_one_aborted) {
    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, vin_ff, 5);
    fake_can_stage_response_at(0x7E8, 8, vin_ff, 5, 2);
    fake_can_stage_response_at(0x7E8, 8, vin_cf1, 5, 3);
    fake_can_stage_response_at(0x7E8, 8, vin_cf2, 5, 3);

    TEST_ASSERT_EQUAL_STRING("7E8 10 14 49 02 01 31 32 33 \r?\r"
                             "7E8 10 14 49 02 01 31 32 33 \r"
                             "7E8 21 34 35 36 37 38 39 41 \r"
                             "7E8 22 42 43 44 45 46 47 48 \r" ELM_PROMPT,
                             elm_ask("0902\r"));
    TEST_ASSERT_EQUAL_INT(3, fake_can_sent_count());
}

TEST(an_unrecognised_pci_nibble_is_ignored) {
    static const uint8_t bad_pci[8] = {0x40, 0x01, 0, 0, 0, 0, 0, 0};

    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, bad_pci, 5);

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_flow_control_frame_from_the_ecu_is_handed_to_the_raw_client) {
    static const uint8_t fc[8] = {0x30, 0x00, 0x00, 0, 0, 0, 0, 0};

    on_can_11bit();
    elm_ok("ATCAF0\r");
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
static void stage_isotp_reply(int payload) {
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
static int lines_printed(const char *s) {
    int n = 0;

    for (const char *p = s; *p; p++) {
        if (*p == '\r') {
            n++;
        }
    }

    return n - 1;
}

TEST(a_reply_of_fifteen_consecutive_frames_is_reassembled) {
    on_can_11bit();
    stage_isotp_reply(111);

    /* 6 bytes in the first frame plus 15 x 7 is the largest that works. */
    TEST_ASSERT_EQUAL_INT(16, lines_printed(elm_ask("22F190\r")));
}

TEST(a_reply_crossing_the_sequence_number_rollover_is_reassembled) {
    on_can_11bit();
    stage_isotp_reply(118);

    /* 16 consecutive frames, so the sequence number runs 1..15 then back to
     * 0. Comparing it against an unmasked counter used to reject this. */
    const char *out = elm_ask("22F190\r");

    TEST_ASSERT_MSG(strstr(out, "?") == NULL, "transfer was rejected: \"%s\"",
                    td_escape(out, fake_port_len(0)));
    TEST_ASSERT_EQUAL_INT(17, lines_printed(out));
}

TEST(a_reply_larger_than_the_tx_buffer_streams_out_in_full) {
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

TEST(a_long_reply_arrives_in_order) {
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

TEST(a_response_pending_reply_extends_the_wait) {
    /* Negative response 0x7F, code 0x78: "busy, answer coming". */
    static const uint8_t pending[8] = {0x03, 0x7F, 0x01, 0x78, 0, 0, 0, 0};
    static const uint8_t answer[8] = {0x06, 0x41, 0x00, 0xBE,
                                      0x3F, 0xB8, 0x13, 0x00};

    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, pending, 5);
    /* Well past the 200 ms default, but inside the extended window. */
    fake_can_stage_response(0x7E8, 8, answer, 900);

    TEST_ASSERT_EQUAL_STRING("7E8 03 7F 01 78 00 00 00 00 \r"
                             "7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(a_reply_after_the_timeout_is_missed) {
    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, supported_pids, 5000);

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

/* ------------------------------------------------------------------ *
 * AT D0 and AT D1 - the data length code
 * ------------------------------------------------------------------ */

TEST(the_data_length_is_hidden_by_default) {
    on_can_11bit();
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    /* PP 29's factory value, which the command summary marks as the default
     * with its asterisk. ISO 15765-4 requires eight data bytes in every
     * message, so the digit says the same thing on every line of an OBD
     * session and is only worth the room when experimenting. */
    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(at_d1_shows_the_data_length_between_the_header_and_the_data) {
    on_can_11bit();
    elm_ok("ATD1\r");
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    /* "the single DLC digit will appear between the ID (header) bytes and the
     * data bytes". */
    TEST_ASSERT_EQUAL_STRING("7E8 8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(at_d1_reports_a_short_frames_real_length) {
    static const uint8_t shortie[3] = {0x02, 0x41, 0x00};

    on_can_11bit();
    elm_ok("ATD1\r");
    fake_can_stage_response(0x7E8, 3, shortie, 5);

    /* The whole point of the command: on a bus that does not pad to eight,
     * this is the only place the frame's real length is visible. */
    TEST_ASSERT_EQUAL_STRING("7E8 3 02 41 00 \r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(at_d0_hides_it_again) {
    on_can_11bit();
    elm_ok("ATD1\r");
    elm_ok("ATD0\r");
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(the_data_length_needs_headers_to_be_on) {
    elm_echo_off();
    elm_ok(CAN_11BIT_500K);
    elm_ok("ATD1\r");
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    /* "the headers must also be on in order to see this digit". With them off
     * there is no identifier for the digit to sit behind, and a bare hex
     * digit in front of the data would read as another data byte. */
    TEST_ASSERT_EQUAL_STRING("41 00 BE 3F B8 13 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(at_d1_packs_with_spacing_off) {
    on_can_11bit();
    elm_ok("ATD1\r");
    elm_ok("ATS0\r");
    fake_can_stage_response(0x7E8, 8, supported_pids, 5);

    TEST_ASSERT_EQUAL_STRING("7E88064100BE3FB81300\r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(at_d_still_sets_everything_to_defaults) {
    on_can_11bit();

    /*
     * The reason all three matches are exact. AT D is a different command -
     * set all to defaults - that happens to be a prefix of these two, and a
     * loose match either way round would have one swallow the other. Car
     * Scanner sends AT D and AT D0 one after the other in its opening
     * sequence, so getting this wrong is not hypothetical.
     */
    elm_ok("ATD\r");

    /* Echo back on is itself the evidence: it is a power-on default that
     * on_can_11bit() turned off, so seeing the command come back means the
     * reset reached settings this test never touched. */
    TEST_ASSERT_EQUAL_STRING("ATDPN\r0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_TRUE(g_elm->settings.echo);
    TEST_ASSERT_FALSE(g_elm->settings.show_header);
    TEST_ASSERT_FALSE(g_elm->settings.can_show_dlc);
}

TEST(interleaved_ecu_transfers_have_independent_sequence_numbers) {
    on_can_11bit();
    fake_can_stage_response(0x7e8, 8, vin_ff, 1);
    fake_can_stage_response(0x7e9, 8, vin_ff, 1);
    fake_can_stage_response_at(0x7e8, 8, vin_cf1, 1, 3);
    fake_can_stage_response_at(0x7e9, 8, vin_cf1, 1, 3);
    fake_can_stage_response_at(0x7e8, 8, vin_cf2, 1, 3);
    fake_can_stage_response_at(0x7e9, 8, vin_cf2, 1, 3);
    const char *out = elm_ask("0902\r");
    TEST_ASSERT_NULL(strchr(out, '?'));
    TEST_ASSERT_NOT_NULL(strstr(out, "7E8 22 "));
    TEST_ASSERT_NOT_NULL(strstr(out, "7E9 22 "));
    TEST_ASSERT_EQUAL_INT(0x7e0, fake_can_sent(1)->id);
    TEST_ASSERT_EQUAL_INT(0x7e1, fake_can_sent(2)->id);
}

TEST(single_frame_from_another_ecu_does_not_change_cf_sequence) {
    on_can_11bit();
    fake_can_stage_response(0x7e9, 8, supported_pids, 1);
    stage_vin_reply();
    const char *out = elm_ask("0902\r");
    TEST_ASSERT_NULL(strchr(out, '?'));
    TEST_ASSERT_NOT_NULL(strstr(out, "7E8 22 "));
}

TEST(a_cf_from_another_ecu_cannot_complete_the_active_transfer) {
    on_can_11bit();
    fake_can_stage_response(0x7e8, 8, vin_ff, 1);
    fake_can_stage_response_at(0x7e9, 8, vin_cf1, 1, 2);
    TEST_ASSERT_EQUAL_STRING("7E8 10 14 49 02 01 31 32 33 \r?\r" ELM_PROMPT,
                             elm_ask("0902\r"));
}

TEST(a_missing_final_frame_is_a_transport_error) {
    on_can_11bit();
    fake_can_stage_response(0x7e8, 8, vin_ff, 1);
    uint32_t before = fake_clock_ms();
    const char *out = elm_ask("0902\r");
    TEST_ASSERT_NOT_NULL(strstr(out, "?\r"));
    TEST_ASSERT_EQUAL_INT(1001, fake_clock_ms() - before);
}

TEST(consecutive_frames_use_transport_timeout_instead_of_at_st) {
    on_can_11bit();
    fake_can_stage_response(0x7e8, 8, vin_ff, 1);
    fake_can_stage_response_at(0x7e8, 8, vin_cf1, 900, 2);
    fake_can_stage_response_at(0x7e8, 8, vin_cf2, 900, 2);
    const char *out = elm_ask("0902\r");
    TEST_ASSERT_NULL(strchr(out, '?'));
    TEST_ASSERT_NOT_NULL(strstr(out, "7E8 22 "));
}

TEST(final_cf_padding_is_removed_only_in_the_formatted_view) {
    elm_echo_off();
    elm_ok(CAN_11BIT_500K);
    uint8_t ff[8] = {0x10, 8, 1, 2, 3, 4, 5, 6};
    uint8_t cf[8] = {0x21, 7, 8, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa};
    fake_can_stage_response(0x7e8, 8, ff, 1);
    fake_can_stage_response_at(0x7e8, 8, cf, 1, 2);
    TEST_ASSERT_EQUAL_STRING(
        "008\r0: 01 02 03 04 05 06 \r1: 07 08 \r" ELM_PROMPT,
        elm_ask("0902\r"));
}

TEST(malformed_and_unsolicited_frames_do_not_count_as_replies) {
    on_can_11bit();
    uint8_t bad_sf[8] = {0x07};
    uint8_t fc[8] = {0x30};
    fake_can_stage_response(0x7e8, 2, bad_sf, 1);
    fake_can_stage_response(0x7e8, 7, vin_ff, 1);
    fake_can_stage_response(0x7e8, 8, vin_cf1, 1);
    fake_can_stage_response(0x7e8, 8, fc, 1);
    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
}

TEST(directed_request_ignores_other_ecus) {
    on_can_11bit();
    elm_ok("ATSH7E0\r");
    fake_can_stage_response(0x7e9, 8, supported_pids, 1);
    fake_can_stage_response(0x7e8, 8, supported_pids, 1);
    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(broadcast_reception_keeps_listening_after_a_complete_segmented_reply) {
    on_can_11bit();
    stage_vin_reply();
    fake_can_stage_response_at(0x7e9, 8, supported_pids, 100, 2);
    const char *out = elm_ask("0902\r");
    TEST_ASSERT_NOT_NULL(strstr(out, "7E8 22 "));
    TEST_ASSERT_NOT_NULL(strstr(out, "7E9 06 "));
}

TEST(failed_flow_control_send_is_reported) {
    on_can_11bit();
    fake_can_fail_confirmed_send();
    stage_vin_reply();
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("0902\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
}

TEST(custom_priority_request_accepts_default_priority_response) {
    on_can_29bit();
    elm_ok("ATSH0CDA10F1\r");
    fake_can_stage_response(CAN_EFF_FLAG | 0x18DAF111, 8, supported_pids, 1);
    fake_can_stage_response(CAN_EFF_FLAG | 0x18DAF110, 8, supported_pids, 1);
    TEST_ASSERT_EQUAL_STRING("18DAF110 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(normal_fixed_reply_priority_can_change_between_segments) {
    on_can_29bit();
    elm_ok("ATSH0CDA10F1\r");
    fake_can_stage_response(CAN_EFF_FLAG | 0x04DAF110, 8, vin_ff, 1);
    fake_can_stage_response_at(CAN_EFF_FLAG | 0x08DAF110, 8, vin_cf1, 1, 2);
    fake_can_stage_response_at(CAN_EFF_FLAG | 0x18DAF110, 8, vin_cf2, 1, 2);
    const char *out = elm_ask("0902\r");
    TEST_ASSERT_NULL(strchr(out, '?'));
    TEST_ASSERT_NOT_NULL(strstr(out, "04DAF110 10 14 "));
    TEST_ASSERT_NOT_NULL(strstr(out, "08DAF110 21 "));
    TEST_ASSERT_NOT_NULL(strstr(out, "18DAF110 22 "));
    TEST_ASSERT_EQUAL_INT(2, fake_can_sent_count());
    TEST_ASSERT_EQUAL_HEX32(CAN_EFF_FLAG | 0x18DA10F1, fake_can_sent(1)->id);
}

TEST(custom_priority_functional_request_accepts_multiple_responders) {
    on_can_29bit();
    elm_ok("ATSH0CDB33F1\r");
    fake_can_stage_response(CAN_EFF_FLAG | 0x18DAF110, 8, supported_pids, 1);
    fake_can_stage_response(CAN_EFF_FLAG | 0x04DAF111, 8, supported_pids, 1);
    const char *out = elm_ask("0100\r");
    TEST_ASSERT_NOT_NULL(strstr(out, "18DAF110 06 "));
    TEST_ASSERT_NOT_NULL(strstr(out, "04DAF111 06 "));
}

TEST(
    aborted_flow_control_closes_can_and_next_request_reopens_with_same_header) {
    on_can_29bit();
    elm_ok("ATSH0CDA10F1\r");
    fake_can_abort_confirmed_send();
    fake_can_stage_response(CAN_EFF_FLAG | 0x18DAF110, 8, vin_ff, 1);
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("0902\r"));
    TEST_ASSERT_FALSE(vif_bus_is_open(g_elm_session, VIF_BUS_CAN));
    TEST_ASSERT_FALSE(fake_can_is_up());
    int setups = fake_can_setup_count();
    fake_can_stage_response(CAN_EFF_FLAG | 0x18DAF110, 8, supported_pids, 1);
    TEST_ASSERT_EQUAL_STRING("18DAF110 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_INT(setups + 1, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(500000, fake_can_last_baud());
    TEST_ASSERT_EQUAL_HEX32(CAN_EFF_FLAG | 0x0CDA10F1, fake_can_sent(1)->id);
    TEST_ASSERT_EQUAL_STRING("7\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(failed_flow_control_keeps_can_open_for_the_next_request) {
    on_can_29bit();
    elm_ok("ATSH0CDA10F1\r");
    int setups = fake_can_setup_count();
    int teardowns = fake_can_teardown_count();
    fake_can_fail_confirmed_send();
    fake_can_stage_response(CAN_EFF_FLAG | 0x18DAF110, 8, vin_ff, 1);
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("0902\r"));
    TEST_ASSERT_TRUE(vif_bus_is_open(g_elm_session, VIF_BUS_CAN));
    TEST_ASSERT_TRUE(fake_can_is_up());
    fake_can_stage_response(CAN_EFF_FLAG | 0x18DAF110, 8, supported_pids, 1);
    TEST_ASSERT_EQUAL_STRING("18DAF110 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_INT(setups, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(teardowns, fake_can_teardown_count());
    TEST_ASSERT_EQUAL_HEX32(CAN_EFF_FLAG | 0x0CDA10F1, fake_can_sent(1)->id);
}
