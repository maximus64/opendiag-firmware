/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_elm327_bus.c
 * @brief J1850 PWM, J1850 VPW and ISO 9141 framing and output formatting.
 *
 * The three share one code shape in elm327_at.c: a three byte header in front
 * of the request, then frames printed until the timeout runs out.
 */

#include "elm327_harness.h"

/* A mode 01 PID 00 reply: three header bytes, six data bytes, one checksum. */
static const uint8_t reply[10] = {0x41, 0x6B, 0x10, 0x41, 0x00,
                                  0xBE, 0x3F, 0xB8, 0x13, 0xC4};

#define REPLY_WITH_HEADER "41 6B 10 41 00 BE 3F B8 13 C4 \r"
#define REPLY_STRIPPED "41 00 BE 3F B8 13 \r"

/* ------------------------------------------------------------------ *
 * Request framing
 * ------------------------------------------------------------------ */

static void assert_sent(fake_bus_id_t bus, const char *expect, size_t len) {
    size_t sent_len = 0;
    const uint8_t *sent = fake_bus_sent(bus, 0, &sent_len);

    TEST_ASSERT_MSG(sent != NULL, "nothing was transmitted");
    TEST_ASSERT_EQUAL_INT(len, sent_len);
    TEST_ASSERT_EQUAL_MEM(expect, sent, len);
}

TEST(j1850_pwm_prepends_its_default_header) {
    elm_echo_off();
    elm_ok("ATSP1\r");

    elm_ask("0100\r");

    /* 0x61 0x6A 0xF1: priority, gateway, tester. */
    assert_sent(FAKE_BUS_J1850_PWM, "\x61\x6A\xF1\x01\x00", 5);
}

static uint32_t pwm_param(bus_param_t param) {
    uint32_t value = 0;
    TEST_ASSERT_EQUAL_INT(
        0, vif_bus_param_get(g_elm->session, VIF_BUS_J1850_PWM, param, &value));
    return value;
}

TEST(pwm_ifr_commands_reach_the_driver) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ask("0100\r");
    TEST_ASSERT_EQUAL_INT(1, pwm_param(BUS_P_IFR_ENABLED));
    TEST_ASSERT_EQUAL_INT(0xF1, pwm_param(BUS_P_IFR_BYTE));
    elm_ok("ATIFR0\r");
    TEST_ASSERT_EQUAL_INT(0, pwm_param(BUS_P_IFR_ENABLED));
    elm_ask("0100\r");
    TEST_ASSERT_EQUAL_INT(0, pwm_param(BUS_P_IFR_ENABLED));
    elm_ok("ATIFR1\r");
    TEST_ASSERT_EQUAL_INT(1, pwm_param(BUS_P_IFR_ENABLED));
    elm_ok("ATSH616AF2\r");
    elm_ask("0100\r");
    TEST_ASSERT_EQUAL_INT(0xF2, pwm_param(BUS_P_IFR_BYTE));
    elm_ok("ATTAF3\r");
    elm_ok("ATIFRS\r");
    TEST_ASSERT_EQUAL_INT(0xF3, pwm_param(BUS_P_IFR_BYTE));
    elm_ok("ATIFRH\r");
    TEST_ASSERT_EQUAL_INT(0xF2, pwm_param(BUS_P_IFR_BYTE));
}

TEST(j1850_vpw_prepends_its_default_header) {
    elm_echo_off();
    elm_ok("ATSP2\r");

    elm_ask("0100\r");

    assert_sent(FAKE_BUS_J1850_VPW, "\x68\x6A\xF1\x01\x00", 5);
}

TEST(iso9141_prepends_its_default_header) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    elm_ask("0100\r");

    assert_sent(FAKE_BUS_KLINE, "\x68\x6A\xF1\x01\x00", 5);
}

TEST(at_sh_overrides_the_header_bytes) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATSH81F110\r");

    elm_ask("0100\r");

    assert_sent(FAKE_BUS_J1850_PWM, "\x81\xF1\x10\x01\x00", 5);
}

TEST(selecting_iso9141_brings_the_bus_up_but_does_not_initialise_it) {
    elm_echo_off();

    elm_ok("ATSP3\r");

    /* Selecting a protocol is not opening a session. A 5 baud handshake takes
     * two and a half seconds and can fail on a vehicle that is reachable a
     * moment later, so a client that has only chosen a protocol has not yet
     * spent that time. */
    TEST_ASSERT_EQUAL_INT(1, fake_bus_setup_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sync_count());
}

TEST(the_five_baud_init_runs_on_the_first_request) {
    static const uint8_t reply[] = {0x48, 0x6B, 0x10, 0x41, 0x00, 0xBE, 0x22};

    elm_echo_off();
    elm_ok("ATSP3\r");
    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);

    /* "generally not until a request needs to be sent". */
    TEST_ASSERT_EQUAL_STRING(ELM_BUS_INIT "41 00 BE \r" ELM_PROMPT,
                             elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sync_count());
}

TEST(the_link_is_initialised_once_and_then_left_alone) {
    static const uint8_t reply[] = {0x48, 0x6B, 0x10, 0x41, 0x00, 0xBE, 0x22};

    elm_echo_off();
    elm_ok("ATSP3\r");

    for (int i = 0; i < 3; i++) {
        fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);
        elm_ask("0100\r");
    }

    TEST_ASSERT_EQUAL_INT(1, fake_bus_sync_count());
}

TEST(a_failed_initiation_is_reported_and_stops_the_request) {
    elm_echo_off();
    elm_ok("ATSP3\r");
    fake_bus_kline_connect_result(-1);

    /* One line, not two: the client must not see BUS INIT: ...ERROR followed
     * by a bare "?" for the same failure. */
    TEST_ASSERT_EQUAL_STRING("BUS INIT: ...ERROR\r" ELM_PROMPT,
                             elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sent_count(FAKE_BUS_KLINE));
}

TEST(switching_away_tears_the_previous_bus_down) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATSP2\r");

    TEST_ASSERT_EQUAL_INT(1, fake_bus_teardown_count(FAKE_BUS_J1850_PWM));
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_J1850_PWM));
    TEST_ASSERT_TRUE(fake_bus_is_up(FAKE_BUS_J1850_VPW));
}

TEST(a_failed_pwm_transmit_is_reported) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    fake_bus_fail_next_send(FAKE_BUS_J1850_PWM);

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_failed_vpw_transmit_is_reported) {
    elm_echo_off();
    elm_ok("ATSP2\r");
    fake_bus_fail_next_send(FAKE_BUS_J1850_VPW);

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_failed_kline_transmit_is_reported) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");
    fake_bus_fail_next_send(FAKE_BUS_KLINE);

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("0100\r"));
}

/* ------------------------------------------------------------------ *
 * Reply formatting
 * ------------------------------------------------------------------ */

TEST(a_reply_is_printed_with_its_header_bytes) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, elm_ask("0100\r"));
}

TEST(headers_off_strips_the_header_and_the_checksum) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH0\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(REPLY_STRIPPED ELM_PROMPT, elm_ask("0100\r"));
}

TEST(headers_off_leaves_a_short_frame_alone) {
    static const uint8_t runt[4] = {0x41, 0x6B, 0x10, 0xC4};

    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH0\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, runt, sizeof(runt), 5);

    /* Four bytes are all header and checksum; there is nothing to strip down
     * to, so the frame is passed through rather than emptied. */
    TEST_ASSERT_EQUAL_STRING("41 6B 10 C4 \r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(spacing_off_packs_the_reply) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH1\r");
    elm_ok("ATS0\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING("416B104100BE3FB813C4\r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(every_reply_inside_the_window_is_printed) {
    static const uint8_t second[10] = {0x41, 0x6B, 0x18, 0x41, 0x00,
                                       0x80, 0x00, 0x00, 0x00, 0x9F};

    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, second, sizeof(second), 5);

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER
                             "41 6B 18 41 00 80 00 00 00 9F \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(a_frame_count_hint_stops_the_wait_early) {
    static const uint8_t second[10] = {0x41, 0x6B, 0x18, 0x41, 0x00,
                                       0x80, 0x00, 0x00, 0x00, 0x9F};

    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, second, sizeof(second), 5);

    uint32_t before = fake_clock_ms();
    const char *out = elm_ask("01001\r");
    uint32_t waited = fake_clock_ms() - before;

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, out);
    TEST_ASSERT_MSG(waited < 200, "waited %u ms despite the hint", waited);
}

TEST(a_zero_frame_hint_sends_without_waiting_for_an_answer) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(ELM_PROMPT, elm_ask("01000\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sent_count(FAKE_BUS_J1850_PWM));
}

TEST(a_zero_frame_hint_on_vpw_sends_without_waiting) {
    elm_echo_off();
    elm_ok("ATSP2\r");
    fake_bus_stage_response(FAKE_BUS_J1850_VPW, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(ELM_PROMPT, elm_ask("01000\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sent_count(FAKE_BUS_J1850_VPW));
}

TEST(a_zero_frame_hint_on_kline_sends_without_waiting) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");
    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(ELM_PROMPT, elm_ask("01000\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sent_count(FAKE_BUS_KLINE));
}

TEST(a_vpw_reply_is_printed_like_a_pwm_one) {
    elm_echo_off();
    elm_ok("ATSP2\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_VPW, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_vpw_frame_count_hint_stops_the_wait_early) {
    static const uint8_t second[10] = {0x41, 0x6B, 0x18, 0x41, 0x00,
                                       0x80, 0x00, 0x00, 0x00, 0x9F};

    elm_echo_off();
    elm_ok("ATSP2\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_VPW, reply, sizeof(reply), 5);
    fake_bus_stage_response(FAKE_BUS_J1850_VPW, second, sizeof(second), 5);

    uint32_t before = fake_clock_ms();
    const char *out = elm_ask("01001\r");

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, out);
    TEST_ASSERT_MSG(fake_clock_ms() - before < 200, "waited despite the hint");
}

TEST(a_kline_frame_count_hint_stops_the_wait_early) {
    static const uint8_t second[10] = {0x48, 0x6B, 0x18, 0x41, 0x00,
                                       0x80, 0x00, 0x00, 0x00, 0x9F};

    elm_echo_off();
    elm_kline_select("ATSP3\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);
    fake_bus_stage_response(FAKE_BUS_KLINE, second, sizeof(second), 5);

    uint32_t before = fake_clock_ms();
    const char *out = elm_ask("01001\r");

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, out);
    TEST_ASSERT_MSG(fake_clock_ms() - before < 200, "waited despite the hint");
}

/* ------------------------------------------------------------------ *
 * Silence and stale traffic
 * ------------------------------------------------------------------ */

TEST(a_silent_pwm_bus_reports_no_data) {
    elm_echo_off();
    elm_ok("ATSP1\r");

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_silent_vpw_bus_reports_no_data) {
    elm_echo_off();
    elm_ok("ATSP2\r");

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_silent_kline_bus_reports_no_data) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_reply_arriving_after_the_timeout_is_missed) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5000);

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(j1850_discards_traffic_queued_before_the_request) {
    static const uint8_t leftover[10] = {0x41, 0x6B, 0x10, 0x41, 0x0C,
                                         0x1A, 0xF8, 0x00, 0x00, 0x77};

    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH1\r");
    fake_bus_stage_stale(FAKE_BUS_J1850_PWM, leftover, sizeof(leftover));
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, elm_ask("0100\r"));
}

TEST(iso9141_still_reports_traffic_queued_before_the_request) {
    static const uint8_t leftover[10] = {0x48, 0x6B, 0x10, 0x41, 0x0C,
                                         0x1A, 0xF8, 0x00, 0x00, 0x77};

    elm_echo_off();
    elm_kline_select("ATSP3\r");
    elm_ok("ATH1\r");
    fake_bus_stage_stale(FAKE_BUS_KLINE, leftover, sizeof(leftover));
    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);

    /* Unlike the J1850 paths, the K-Line transfer does not drain the driver
     * first: that loop is commented out in elm327_iso9141_xfer(). A reply to
     * somebody else's request is therefore attributed to this one. Pinned so
     * the difference is visible rather than surprising. */
    TEST_ASSERT_EQUAL_STRING(
        "48 6B 10 41 0C 1A F8 00 00 77 \r" REPLY_WITH_HEADER ELM_PROMPT,
        elm_ask("0100\r"));
}

/* ------------------------------------------------------------------ *
 * Receive addressing
 *
 * The datasheet displays a reply only when the address it was sent to is one
 * this tool answers to. Without that, every message a network carries between
 * the diagnostic exchanges is printed as though it answered the outstanding
 * request.
 * ------------------------------------------------------------------ */

/* Same reply, addressed to module 0x18 rather than to the tester. */
static const uint8_t not_for_us[10] = {0x41, 0x18, 0x10, 0x41, 0x00,
                                       0xBE, 0x3F, 0xB8, 0x13, 0x2A};

/* And one sent to the tester's own address rather than the functional one. */
static const uint8_t to_tester[10] = {0x41, 0xF1, 0x10, 0x41, 0x00,
                                      0xBE, 0x3F, 0xB8, 0x13, 0x5C};

TEST(a_reply_to_the_functional_response_address_is_accepted) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    /* The request went to 0x6A; SAE J2178 pairs that with 0x6B for the
     * answer, which is the target byte this reply carries. */
    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_reply_to_the_tester_address_is_accepted) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, to_tester, sizeof(to_tester),
                            5);

    TEST_ASSERT_EQUAL_STRING("41 F1 10 41 00 BE 3F B8 13 5C \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(a_message_addressed_to_another_module_is_not_printed) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, not_for_us, sizeof(not_for_us),
                            5);

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(traffic_for_others_does_not_hold_the_window_open) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, not_for_us, sizeof(not_for_us),
                            5);
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    /* The foreign frame is dropped without restarting the AT ST timer, so the
     * reply behind it is still inside the original window. */
    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, elm_ask("0100\r"));
}

TEST(at_ra_narrows_the_accepted_address_to_one_byte) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATRA18\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, not_for_us, sizeof(not_for_us),
                            5);
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    /* 0x18 is now the only address accepted, so the functional reply that
     * would otherwise have been printed is the one dropped. */
    TEST_ASSERT_EQUAL_STRING("41 18 10 41 00 BE 3F B8 13 2A \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(at_sr_is_the_same_command_as_at_ra) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATSR18\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(at_ar_restores_the_automatic_addresses) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATRA18\r");
    elm_ok("ATAR\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, elm_ask("0100\r"));
}

TEST(at_ta_adds_the_new_tester_address_to_the_accepted_set) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATTA18\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, not_for_us, sizeof(not_for_us),
                            5);

    TEST_ASSERT_EQUAL_STRING("41 18 10 41 00 BE 3F B8 13 2A \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(at_ft_filters_on_the_transmitter) {
    static const uint8_t from_18[10] = {0x41, 0x6B, 0x18, 0x41, 0x00,
                                        0x80, 0x00, 0x00, 0x00, 0xB7};

    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH1\r");
    elm_ok("ATFT10\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, from_18, sizeof(from_18), 5);
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    /* Both are addressed to us; only the one module 0x10 sent is wanted. */
    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, elm_ask("0100\r"));
}

TEST(at_ft_with_no_address_turns_the_filter_off) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH1\r");
    elm_ok("ATFT18\r");
    elm_ok("ATFT\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_single_byte_header_frame_carries_no_address_to_judge) {
    /* Bit 4 of the header byte set: clause 5.3.1's one byte header, which has
     * no target and so cannot be attributed to anyone. */
    static const uint8_t one_byte_header[5] = {0x90, 0x41, 0x00, 0xBE, 0x77};

    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, one_byte_header,
                            sizeof(one_byte_header), 5);

    TEST_ASSERT_EQUAL_STRING("90 41 00 BE 77 \r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(headers_off_strips_one_byte_from_a_single_byte_header) {
    static const uint8_t one_byte_header[5] = {0x90, 0x41, 0x00, 0xBE, 0x77};

    elm_echo_off();
    elm_ok("ATSP1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, one_byte_header,
                            sizeof(one_byte_header), 5);

    TEST_ASSERT_EQUAL_STRING("41 00 BE \r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(iso9141_keeps_three_header_bytes_whatever_bit_four_says) {
    /* The one byte header is a J1850 idea. On K-Line an 0x90 first byte is a
     * length field, and stripping one byte off it would eat an address. */
    static const uint8_t kwp_like[6] = {0x90, 0x6B, 0x10, 0x41, 0x00, 0x77};

    elm_echo_off();
    elm_kline_select("ATSP3\r");
    fake_bus_stage_response(FAKE_BUS_KLINE, kwp_like, sizeof(kwp_like), 5);

    TEST_ASSERT_EQUAL_STRING("41 00 \r" ELM_PROMPT, elm_ask("0100\r"));
}

/* ------------------------------------------------------------------ *
 * Retransmission settling
 * ------------------------------------------------------------------ */

TEST(a_frame_count_hint_still_waits_for_the_bus_to_go_quiet) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);
    /* The retransmission the module sends because nothing acknowledged it. */
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    uint32_t before = fake_clock_ms();
    const char *out = elm_ask("01001\r");
    uint32_t waited = fake_clock_ms() - before;

    /* One line, as asked - but the prompt is not handed back until the repeat
     * has come and gone, so the next request cannot collide with it. */
    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, out);
    TEST_ASSERT_MSG(waited >= J1850_SETTLE_MS,
                    "returned after %u ms without letting the bus settle",
                    waited);
    TEST_ASSERT_MSG(waited < 200, "waited %u ms, the hint bought nothing",
                    waited);
}

TEST(a_zero_frame_hint_settles_the_bus_before_the_prompt) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    uint32_t before = fake_clock_ms();

    TEST_ASSERT_EQUAL_STRING(ELM_PROMPT, elm_ask("01000\r"));
    TEST_ASSERT_MSG(fake_clock_ms() - before >= J1850_SETTLE_MS,
                    "the request went out and the prompt came straight back");
}

TEST(a_window_that_ran_out_has_nothing_left_to_settle) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    uint32_t before = fake_clock_ms();

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, elm_ask("0100\r"));

    /* The AT ST window closed on its own, so the bus has already been quiet
     * for 200 ms and no further wait is added on top. */
    TEST_ASSERT_MSG(fake_clock_ms() - before < 200 + J1850_SETTLE_MS,
                    "settled again after the window had already expired");
}

TEST(kline_has_no_retransmissions_to_wait_out) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");
    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);

    uint32_t before = fake_clock_ms();

    elm_ask("01001\r");

    TEST_ASSERT_MSG(fake_clock_ms() - before < J1850_SETTLE_MS,
                    "K-Line waited for a burst that cannot happen");
}

/* ------------------------------------------------------------------ *
 * ISO 14230-4, protocols 4 and 5
 * ------------------------------------------------------------------ */

TEST(kwp_prepends_the_functional_header_iso_14230_4_requires) {
    elm_echo_off();
    elm_kline_select("ATSP4\r");

    elm_ask("0100\r");

    /* C2 33 F1 01 00: functional addressing, two data bytes, every emission
     * related ECU. The whitepaper calls this "the only correct and valid
     * Service 1 PID 00 request message" for KWP key bytes. */
    assert_sent(FAKE_BUS_KLINE, "\xC2\x33\xF1\x01\x00", 5);
}

TEST(kwp_recomputes_the_length_in_the_format_byte) {
    elm_echo_off();
    elm_kline_select("ATSP4\r");

    /* Three data bytes where the header was set up for two: the length bits
     * follow the request, not the header the client typed. */
    elm_ask("010C0D\r");

    assert_sent(FAKE_BUS_KLINE, "\xC3\x33\xF1\x01\x0C\x0D", 6);
}

TEST(a_zero_length_nibble_asks_for_an_additional_length_byte) {
    elm_echo_off();
    elm_kline_select("ATSP4\r");
    elm_ok("ATSH80F110\r");

    elm_ask("0100\r");

    /* "If you provide a value of 0 for the second digit of the first header
     * byte... you want to have a fourth header (length) byte inserted." */
    assert_sent(FAKE_BUS_KLINE, "\x80\xF1\x10\x02\x01\x00", 6);
}

TEST(a_kwp_reply_is_stripped_of_a_three_byte_header) {
    static const uint8_t reply[] = {0x83, 0xF1, 0x10, 0x41, 0x00, 0xBE, 0x00};

    elm_echo_off();
    elm_kline_select("ATSP4\r");
    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING("41 00 BE \r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_kwp_reply_is_stripped_of_its_additional_length_byte_too) {
    /* Clause 4.1.4's fourth header byte. Counting it as data would print the
     * reply one byte out, which reads as a vehicle answering nonsense. */
    static const uint8_t reply[] = {0x80, 0xF1, 0x10, 0x03,
                                    0x41, 0x00, 0xBE, 0x00};

    elm_echo_off();
    elm_kline_select("ATSP4\r");
    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING("41 00 BE \r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_kwp_reply_keeps_its_whole_header_when_headers_are_on) {
    static const uint8_t reply[] = {0x83, 0xF1, 0x10, 0x41, 0x00, 0xBE, 0x00};

    elm_echo_off();
    elm_kline_select("ATSP4\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING("83 F1 10 41 00 BE 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(a_kwp_message_with_no_addresses_is_passed_through_whole) {
    /* Clause 4.1.1 mode 00: nothing to judge the target against, so the
     * address filter must not silently drop it. */
    static const uint8_t reply[] = {0x03, 0x41, 0x00, 0xBE, 0x02};

    elm_echo_off();
    elm_kline_select("ATSP4\r");
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING("03 41 00 BE 02 \r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(fast_init_is_what_protocol_5_performs) {
    static const uint8_t reply[] = {0x83, 0xF1, 0x10, 0x41, 0x00, 0xBE, 0x00};

    elm_echo_off();
    elm_ok("ATSP5\r");
    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);

    /* "a fast initiation does not show the dots". */
    TEST_ASSERT_EQUAL_STRING("BUS INIT: OK\r41 00 BE \r" ELM_PROMPT,
                             elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sync_count());
}

TEST(key_bytes_that_say_kwp_move_a_protocol_3_client_to_protocol_4) {
    static const uint8_t reply[] = {0x83, 0xF1, 0x10, 0x41, 0x00, 0xBE, 0x00};

    elm_echo_off();
    elm_ok("ATSP3\r");
    fake_bus_kline_set_variant(KLINE_VARIANT_KWP);
    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);

    elm_ask("0100\r");

    /* One handshake tells the two 5 baud protocols apart, and the answer is
     * the vehicle's rather than the client's. */
    TEST_ASSERT_EQUAL_STRING("4\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    assert_sent(FAKE_BUS_KLINE, "\xC2\x33\xF1\x01\x00", 5);
}

TEST(key_bytes_that_say_iso_9141_move_a_protocol_4_client_to_protocol_3) {
    elm_echo_off();
    elm_ok("ATSP4\r");
    fake_bus_kline_set_variant(KLINE_VARIANT_ISO9141);

    elm_ask("0100\r");

    TEST_ASSERT_EQUAL_STRING("3\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    assert_sent(FAKE_BUS_KLINE, "\x68\x6A\xF1\x01\x00", 5);
}

TEST(moving_between_the_two_five_baud_protocols_keeps_the_session) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    elm_ok("ATSP4\r");

    /* Both run over the same transceiver. Cycling the driver to renumber the
     * protocol would throw away a session that took two and a half seconds
     * to establish. */
    TEST_ASSERT_EQUAL_INT(1, fake_bus_setup_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_teardown_count(FAKE_BUS_KLINE));
}

/* ------------------------------------------------------------------ *
 * The ISO AT commands
 * ------------------------------------------------------------------ */

TEST(at_bi_makes_the_protocol_active_without_touching_the_bus) {
    elm_echo_off();
    elm_ok("ATSP3\r");

    TEST_ASSERT_EQUAL_STRING("N\r" ELM_PROMPT, elm_ask("ATIA\r"));
    elm_ok("ATBI\r");
    TEST_ASSERT_EQUAL_STRING("Y\r" ELM_PROMPT, elm_ask("ATIA\r"));

    TEST_ASSERT_EQUAL_INT(0, fake_bus_sync_count());
}

TEST(at_si_runs_a_slow_init_on_its_own) {
    elm_echo_off();
    elm_ok("ATSP3\r");

    /* "Simply send AT SI, wait a little, then send the message." */
    TEST_ASSERT_EQUAL_STRING("BUS INIT: ...OK\r" ELM_PROMPT, elm_ask("ATSI\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sync_count());
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sent_count(FAKE_BUS_KLINE));
}

TEST(at_si_is_refused_on_a_protocol_that_does_not_use_it) {
    elm_echo_off();
    elm_ok("ATSP5\r");

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSI\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sync_count());
}

TEST(at_fi_runs_a_fast_init_and_only_on_protocol_5) {
    elm_echo_off();
    elm_ok("ATSP3\r");
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATFI\r"));

    elm_ok("ATSP5\r");
    TEST_ASSERT_EQUAL_STRING("BUS INIT: OK\r" ELM_PROMPT, elm_ask("ATFI\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sync_count());
}

TEST(at_kw_reports_the_key_bytes_the_vehicle_sent) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    /* AT BI puts no handshake on the wire, so there are no key bytes to
     * report yet - and reporting that honestly is the point of the command. */
    TEST_ASSERT_EQUAL_STRING("00 00\r" ELM_PROMPT, elm_ask("ATKW\r"));
}

TEST(at_kw_needs_a_k_line_protocol) {
    elm_echo_off();
    elm_ok("ATSP1\r");

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATKW\r"));
}

TEST(at_iia_changes_the_address_the_slow_init_is_directed_to) {
    elm_echo_off();
    elm_ok("ATSP3\r");

    elm_ok("ATIIA7A\r");
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATIIAZZ\r"));

    /* It survives a protocol close, and only a full reset puts $33 back. */
    elm_ok("ATPC\r");
    TEST_ASSERT_EQUAL_INT(0x7A, g_elm->settings.iso_init_address);
    elm327_reset(g_elm);
    TEST_ASSERT_EQUAL_INT(0x33, g_elm->settings.iso_init_address);
}

TEST(at_ib_selects_one_of_the_five_documented_rates) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    elm_ok("ATIB96\r");
    TEST_ASSERT_EQUAL_INT(9600, fake_bus_kline_baud());

    elm_ok("ATIB48\r");
    TEST_ASSERT_EQUAL_INT(4800, fake_bus_kline_baud());

    elm_ok("ATIB10\r");
    TEST_ASSERT_EQUAL_INT(10400, fake_bus_kline_baud());

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATIB99\r"));
    TEST_ASSERT_EQUAL_INT(10400, fake_bus_kline_baud());
}

TEST(at_sw_sets_the_wakeup_interval_in_twenty_millisecond_steps) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    elm_ok("ATSW92\r");
    TEST_ASSERT_EQUAL_INT(0x92 * 20, fake_bus_kline_wakeup_ms());

    /* "the value 00 (zero) is special, as it will stop the periodic messages"
     * without forgetting the interval that was set. */
    elm_ok("ATSW00\r");
    TEST_ASSERT_EQUAL_INT(0, fake_bus_kline_wakeup_ms());
    TEST_ASSERT_EQUAL_INT(0x92 * 20, g_elm->settings.wakeup_ms);

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSWZZ\r"));
}

TEST(at_wm_replaces_the_wakeup_message) {
    const uint8_t *wm;
    size_t n = 0;

    elm_echo_off();
    elm_kline_select("ATSP3\r");

    elm_ok("ATWM 68 6A F1 01 00\r");

    wm = fake_bus_kline_wakeup(&n);
    TEST_ASSERT_EQUAL_INT(5, n);
    TEST_ASSERT_EQUAL_MEM("\x68\x6A\xF1\x01\x00", wm, 5);
}

TEST(at_wm_refuses_a_message_it_cannot_carry) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    /* "one to six bytes total, not including the checksum". */
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT,
                             elm_ask("ATWM112233445566778899\r"));
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATWM6\r"));
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATWM\r"));
}

TEST(at_pc_ends_the_session_before_the_driver_goes_down) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    elm_ok("ATPC\r");

    /* Clause 5.2: a KWP ECU is entitled to be told rather than left to time
     * out at P3max, and this is the last moment anyone knows to say so. */
    /* The claim is gone with the driver, so there is no session left to ask
     * about - which is the point: AT PC closes the protocol, not just the
     * link on it. */
    TEST_ASSERT_EQUAL_INT(VIF_ERR_NO_CLAIM,
                          vif_bus_ioctl(g_elm_session, VIF_BUS_KLINE,
                                        BUS_IOCTL_GET_LINK, NULL, NULL));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE,
                          vif_bus_current(g_elm_session, VIF_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_teardown_count(FAKE_BUS_KLINE));
}

/* ------------------------------------------------------------------ *
 * Adaptive timing - AT AT0, AT1, AT2
 *
 * Every rule below is quoted from the ELM327 datasheet, because the whole
 * point of the feature is compatibility: an application written against a
 * real ELM327 has to see the same behaviour from this one.
 * ------------------------------------------------------------------ */

/** @brief Drive one exchange whose reply arrives @p delay_ms after the request.
 */
static void elm_exchange_at(uint32_t delay_ms) {
    static const uint8_t reply[] = {0x48, 0x6B, 0x10, 0x41, 0x00, 0xBE, 0x22};

    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), delay_ms);
    elm_ask("0100\r");
}

/**
 * The failure this exists to prevent, reproduced from the bench.
 *
 * A J1850 PWM module answers anywhere between 6 and 21 ms depending on where
 * the request lands in its own cycle. A run of quick replies pulls the
 * adaptive window down to the floor; the next slow reply misses it. Before
 * the window learned from its own timeouts, what happened next was the bug:
 * the timeout discarded everything learned, the following request got the
 * full window and was answered quickly, and that one quick answer pulled the
 * window straight back under the slow replies. Every other request failed,
 * indefinitely, on a bus with no errors of any kind on it.
 */
TEST(a_window_that_timed_out_is_never_chosen_again) {
    uint16_t floor_after_failure;

    elm_echo_off();
    elm_kline_select("ATSP3\r");

    /* Quick replies pull the window down as far as it will go. */
    for (int i = 0; i < 6; i++) {
        elm_exchange_at(6);
    }
    TEST_ASSERT_MSG(elm327_reply_window(g_elm) < 40,
                    "six quick replies left the window at %u ms",
                    elm327_reply_window(g_elm));

    /* Now one that arrives after that window: nothing comes back. */
    elm_exchange_at(elm327_reply_window(g_elm) + 10);
    floor_after_failure = g_elm->adaptive_floor_ms;

    TEST_ASSERT_MSG(floor_after_failure > 0,
                    "a timeout taught the algorithm nothing");

    /* The next request gets the full window, and is answered quickly. That
     * single quick answer must not undo what the timeout established. */
    elm_exchange_at(6);

    TEST_ASSERT_MSG(elm327_reply_window(g_elm) >= floor_after_failure,
                    "one quick reply pulled the window back to %u ms, under "
                    "the %u ms that had already failed",
                    elm327_reply_window(g_elm), floor_after_failure);
}

/**
 * The floor is evidence about one vehicle on one protocol, so a reset clears
 * it along with everything else learned. AT SP 0 deliberately does not: it
 * keeps whichever bus is working rather than tearing it down, so there is
 * nothing to forget.
 */
TEST(the_learned_floor_is_forgotten_on_a_reset) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    for (int i = 0; i < 6; i++) {
        elm_exchange_at(6);
    }
    elm_exchange_at(elm327_reply_window(g_elm) + 10);
    TEST_ASSERT(g_elm->adaptive_floor_ms > 0);

    elm_ask("ATZ\r");
    TEST_ASSERT_EQUAL_INT(0, g_elm->adaptive_floor_ms);
}

TEST(adaptive_timing_is_on_by_default) {
    /* "By default, Adaptive Timing option 1 (AT1) is enabled, and is the
     * recommended setting." */
    TEST_ASSERT_EQUAL_INT(1, g_elm->settings.adaptive_timing);
}

TEST(nothing_is_assumed_before_a_reply_has_been_timed) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    /* With no evidence there is nothing to shorten, so the full AT ST window
     * stands. A window guessed at from nothing would be the one thing worse
     * than a window that is too long. */
    TEST_ASSERT_EQUAL_INT(0, g_elm->adaptive_ms);
    TEST_ASSERT_EQUAL_INT(g_elm->settings.timeout, elm327_reply_window(g_elm));
}

TEST(the_window_shrinks_towards_what_the_vehicle_actually_does) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    elm_exchange_at(40);

    /* "automatically sets the timeout value for you, to a value that is based
     * on the actual response times that your vehicle is responding in" - and
     * the datasheet's own example turns a 58 ms response into a window "in the
     * range of 90 msec", a little over half again. */
    TEST_ASSERT_EQUAL_INT(60, g_elm->adaptive_ms);
    TEST_ASSERT_EQUAL_INT(60, elm327_reply_window(g_elm));
}

TEST(the_datasheets_own_worked_example_lands_where_it_says) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    /* "The engine controller responds very quickly, but the transmission
     * takes considerably longer... the adaptive timing algorithm measures the
     * longer transmission response times and will use them to set the
     * timeout, likely to a value in the range of 90 msec." */
    elm_exchange_at(58);

    TEST_ASSERT_MSG(g_elm->adaptive_ms >= 80 && g_elm->adaptive_ms <= 100,
                    "a 58 ms response should give a window near 90 ms, got %u",
                    g_elm->adaptive_ms);
}

TEST(at_st_is_the_ceiling_the_algorithm_never_passes) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    /* AT ST 05 is 20 ms. "it always uses your AT ST hh setting as the maximum
     * setting, and will never choose one which is longer." */
    elm_ok("ATST05\r");
    elm_exchange_at(60);

    TEST_ASSERT_EQUAL_INT(20, elm327_reply_window(g_elm));
}

TEST(a_slower_reply_widens_the_window_at_once) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    elm_exchange_at(60);
    uint16_t was = g_elm->adaptive_ms;

    /* Still inside the 90 ms the last one taught it, so this reply is heard -
     * and being one exchange late is a slow reading where being one exchange
     * short is a reading that never arrives, so widening is immediate rather
     * than gradual. */
    elm_exchange_at(80);

    TEST_ASSERT_MSG(g_elm->adaptive_ms > was,
                    "an 80 ms reply after a 60 ms one should widen the window "
                    "at once, went from %u to %u",
                    was, g_elm->adaptive_ms);
    TEST_ASSERT_MSG(g_elm->adaptive_ms >= 110,
                    "and should clear the next reply of that length: %u",
                    g_elm->adaptive_ms);
}

TEST(a_reply_slower_than_the_learned_window_costs_one_exchange_and_no_more) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    /* Twenty consistent quick replies, so the window is well down. */
    for (int i = 0; i < 10; i++) {
        elm_exchange_at(20);
    }
    TEST_ASSERT(g_elm->adaptive_ms < 60);

    /* Now the vehicle takes 120 ms - bus loading, a slower PID, a module that
     * woke up. The reply falls outside the learned window and is missed. This
     * is the cost of adaptive timing, and the datasheet's AT0 exists for
     * anyone unwilling to pay it. */
    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT,
                             (elm_exchange_at(120), fake_port_text(0)));

    /* What matters is that it costs exactly one exchange: the miss drops
     * everything learned, so the retry runs on the full AT ST window and
     * succeeds. A window that stayed too short would cut off the very reply
     * that would have corrected it. */
    TEST_ASSERT_EQUAL_INT(0, g_elm->adaptive_ms);

    fake_bus_reset_all();
    elm_exchange_at(120);
    TEST_ASSERT_MSG(g_elm->adaptive_ms >= 150,
                    "the retry should have seen the 120 ms reply and learned "
                    "from it, got %u",
                    g_elm->adaptive_ms);
}

TEST(a_faster_vehicle_narrows_the_window_gradually) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    elm_exchange_at(80);
    uint16_t wide = g_elm->adaptive_ms;

    elm_exchange_at(20);
    uint16_t after_one = g_elm->adaptive_ms;

    /* "As conditions such as bus loading, etc. change, the algorithm learns
     * from them, and makes appropriate adjustments" - learns, rather than
     * jumps: one quick reply on a busy bus must not commit the next request
     * to a window that only suited that one. */
    TEST_ASSERT_MSG(after_one < wide, "it should come down");
    TEST_ASSERT_MSG(after_one > 30, "but not all at once: %u -> %u", wide,
                    after_one);

    for (int i = 0; i < 12; i++) {
        elm_exchange_at(20);
    }

    TEST_ASSERT_MSG(g_elm->adaptive_ms <= 40,
                    "and should settle near 30 ms after a dozen consistent "
                    "exchanges, got %u",
                    g_elm->adaptive_ms);
}

TEST(an_exchange_that_drew_nothing_forgets_what_was_learned) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    elm_exchange_at(20);
    TEST_ASSERT(g_elm->adaptive_ms != 0);

    /* Nothing staged, so nothing answers. Without this the window that was
     * too short would stay too short: the reply that would have corrected it
     * is the one being cut off. */
    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));

    TEST_ASSERT_EQUAL_INT(0, g_elm->adaptive_ms);
    TEST_ASSERT_EQUAL_INT(g_elm->settings.timeout, elm327_reply_window(g_elm));
}

TEST(at_at0_turns_the_whole_thing_off) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    elm_exchange_at(20);

    /* "AT0 is used to disable Adaptive Timing (so the timeout is always as
     * set by AT ST)". */
    elm_ok("ATAT0\r");
    TEST_ASSERT_EQUAL_INT(g_elm->settings.timeout, elm327_reply_window(g_elm));

    elm_exchange_at(20);
    TEST_ASSERT_EQUAL_INT(g_elm->settings.timeout, elm327_reply_window(g_elm));
}

TEST(at_at2_is_more_aggressive_than_at_at1) {
    uint16_t at1, at2;

    elm_echo_off();
    elm_kline_select("ATSP3\r");
    elm_exchange_at(40);
    at1 = g_elm->adaptive_ms;

    elm327_reset(g_elm);
    fake_bus_reset_all();
    elm_echo_off();
    elm_kline_select("ATSP3\r");
    elm_ok("ATAT2\r");
    elm_exchange_at(40);
    at2 = g_elm->adaptive_ms;

    /* "AT2 [is] a more aggressive version of AT1". */
    TEST_ASSERT_MSG(at2 < at1, "AT2 gave %u, AT1 gave %u", at2, at1);
}

TEST(changing_the_mode_drops_what_the_other_one_learned) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    elm_exchange_at(40);
    TEST_ASSERT(g_elm->adaptive_ms != 0);

    /* The two modes weigh the same measurement differently, so a window
     * learned under one is not the window the other would have chosen. */
    elm_ok("ATAT2\r");
    TEST_ASSERT_EQUAL_INT(0, g_elm->adaptive_ms);
}

TEST(switching_protocol_drops_what_was_learned) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    elm_exchange_at(40);
    TEST_ASSERT(g_elm->adaptive_ms != 0);

    /* A different bus answers in a different time. */
    elm_ok("ATSP1\r");
    TEST_ASSERT_EQUAL_INT(0, g_elm->adaptive_ms);
}

TEST(the_window_never_drops_below_what_a_bus_can_answer_in) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    /* An immediate reply must not teach the adapter to stop listening: ISO
     * 14230-2 alone allows an ECU 50 ms, so a window under 20 could only ever
     * cut off a reply that was on its way. */
    for (int i = 0; i < 20; i++) {
        elm_exchange_at(0);
    }

    TEST_ASSERT_MSG(elm327_reply_window(g_elm) >= 20,
                    "window collapsed to %u ms", elm327_reply_window(g_elm));
}

TEST(a_protocol_search_keeps_a_floor_under_the_window) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    /* "during protocol searches, an internally set minimum time is used - you
     * may select longer times with AT ST, but not shorter ones." A search
     * that gives up on a protocol early reports the wrong protocol, which is
     * a far worse failure than a slow one. */
    elm_ok("ATST05\r"); /* 20 ms */
    elm_exchange_at(10);

    TEST_ASSERT_EQUAL_INT(20, elm327_reply_window(g_elm));

    g_elm->in_search = true;
    TEST_ASSERT_MSG(elm327_reply_window(g_elm) >= 200,
                    "a search should not run with a %u ms window",
                    elm327_reply_window(g_elm));
    g_elm->in_search = false;
}

/* ------------------------------------------------------------------ *
 * Physically addressed requests, the shape a DTC scan uses
 * ------------------------------------------------------------------ */

TEST(a_reply_reaches_a_client_that_addressed_one_module_physically) {
    /* AT SH 68 09 F1 then mode 03 is how a scan tool reads trouble codes from
     * one ECU rather than from the whole functional group. SAE J1979 has the
     * module answer in a 48 6B <ECU> header regardless, so a tester that
     * expects the reply at the address it asked at discards every one of
     * them - which reads as a vehicle with no fault codes. */
    static const uint8_t dtcs[] = {0x48, 0x6B, 0x09, 0x43, 0x09, 0x74,
                                   0x09, 0x77, 0x09, 0x80, 0x85};

    elm_echo_off();
    elm_kline_select("ATSP3\r");
    elm_ok("ATSH6809F1\r");

    fake_bus_stage_response(FAKE_BUS_KLINE, dtcs, sizeof(dtcs), 5);

    TEST_ASSERT_EQUAL_STRING("43 09 74 09 77 09 80 \r" ELM_PROMPT,
                             elm_ask("03\r"));
}

TEST(a_physically_addressed_request_goes_out_with_the_header_it_was_given) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");
    elm_ok("ATSH6809F1\r");

    elm_ask("03\r");

    assert_sent(FAKE_BUS_KLINE, "\x68\x09\xF1\x03", 4);
}

TEST(the_functional_header_still_reaches_the_same_reply) {
    static const uint8_t dtcs[] = {0x48, 0x6B, 0x09, 0x43, 0x09, 0x74,
                                   0x09, 0x77, 0x09, 0x80, 0x85};

    elm_echo_off();
    elm_kline_select("ATSP3\r");

    fake_bus_stage_response(FAKE_BUS_KLINE, dtcs, sizeof(dtcs), 5);

    /* The default 68 6A F1 accepted $6B all along, because $6A | 1 is $6B.
     * That is why this only ever broke for clients that set a header. */
    TEST_ASSERT_EQUAL_STRING("43 09 74 09 77 09 80 \r" ELM_PROMPT,
                             elm_ask("03\r"));
}

TEST(a_message_for_another_tester_is_still_not_ours) {
    /* $6B is accepted because the protocol fixes it, not because addressing
     * stopped being checked. A reply aimed somewhere else stays filtered. */
    static const uint8_t other[] = {0x48, 0x33, 0x09, 0x43, 0x09, 0x74, 0x00};

    elm_echo_off();
    elm_kline_select("ATSP3\r");
    elm_ok("ATSH6809F1\r");

    fake_bus_stage_response(FAKE_BUS_KLINE, other, sizeof(other), 5);

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("03\r"));
}

TEST(the_iso_9141_reply_address_rule_does_not_leak_onto_j1850) {
    /* A J1850 priority byte of $41 has the same top two bits as an ISO 9141-2
     * CARB format byte and means something else entirely. The exception that
     * lets $6B through on K-Line must not follow it here: on this bus a
     * physically addressed request is answered to the tester, and a message
     * for the functional group is somebody else's. */
    static const uint8_t functional[] = {0x41, 0x6B, 0x10, 0x41,
                                         0x00, 0xBE, 0x00};

    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATSH4110F1\r");

    fake_bus_stage_response(FAKE_BUS_J1850_PWM, functional, sizeof(functional),
                            5);

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}
