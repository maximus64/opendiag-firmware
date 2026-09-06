/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_elm327_search.c
 * @brief Automatic protocol search, the path taken when no ATSP was issued.
 *
 * The search tries all four ISO 15765-4 CAN protocols, then ISO 9141, then
 * KWP fast init, then J1850 PWM, then J1850 VPW, keeping whichever answers
 * first.
 *
 * The regression here is a_search_stops_at_can_11_bit_when_an_ecu_answers().
 * The order used to hold protocol 7 - CAN 29 bit, 500 kbaud - and not
 * protocol 6, so the one combination nearly every vehicle built this century
 * uses was the one the search could never arrive at: a healthy 11 bit bus was
 * walked past, every other protocol was tried, and the answer was NO DATA.
 */

#include "elm327_harness.h"

static const uint8_t can_reply[8] = {0x06, 0x41, 0x00, 0xBE,
                                     0x3F, 0xB8, 0x13, 0x00};

/** Reply identifiers for the two addressing schemes, per ISO 15765-4 clause 8.
 */
#define CAN_11BIT_REPLY_ID 0x7E8
#define CAN_29BIT_REPLY_ID (0x18DAF110 | CAN_EFF_FLAG)

/** How many CAN protocols one full search brings the driver up for. */
#define CAN_PROTOCOLS_IN_A_SEARCH 4

static const uint8_t bus_reply[10] = {0x48, 0x6B, 0x10, 0x41, 0x00,
                                      0xBE, 0x3F, 0xB8, 0x13, 0xC4};

#define SEARCHING "SEARCHING...\r"

TEST(a_request_without_a_protocol_starts_a_search) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING(SEARCHING "NO DATA\r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(a_search_tries_every_bus_before_giving_up) {
    elm_echo_off();

    elm_ask("0100\r");

    /* All four ISO 15765-4 protocols, not just one: two bit rates times two
     * identifier widths, and a vehicle answers exactly one of them. */
    TEST_ASSERT_MSG(fake_can_setup_count() == CAN_PROTOCOLS_IN_A_SEARCH,
                    "not every CAN protocol was tried");
    TEST_ASSERT_MSG(fake_bus_setup_count(FAKE_BUS_KLINE) == 1,
                    "K-Line was not tried");
    TEST_ASSERT_MSG(fake_bus_setup_count(FAKE_BUS_J1850_PWM) == 1,
                    "J1850 PWM was not tried");
    TEST_ASSERT_MSG(fake_bus_setup_count(FAKE_BUS_J1850_VPW) == 1,
                    "J1850 VPW was not tried");
}

TEST(a_failed_search_leaves_no_bus_running) {
    elm_echo_off();

    elm_ask("0100\r");

    TEST_ASSERT_FALSE(fake_can_is_up());
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_KLINE));
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_J1850_PWM));
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_J1850_VPW));

    /* Not "A0": AT DP and AT DPN prefix with A when a search arrived
     * somewhere, and this one arrived nowhere. */
    TEST_ASSERT_EQUAL_STRING("0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(a_second_request_after_a_failed_search_searches_again) {
    elm_echo_off();
    elm_ask("0100\r");

    /* Another four bus bring-ups is the price, and it is worth paying. A
     * search comes back empty most often because the vehicle was asleep or
     * the key was off, and an adapter that answers NO DATA for the rest of
     * the session because of one such moment is one the user has to power
     * cycle. The datasheet expects to see it try again: "Subsequent OBD
     * requests may show 'SEARCHING'". */
    TEST_ASSERT_EQUAL_STRING(SEARCHING "NO DATA\r" ELM_PROMPT,
                             elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_INT(2 * CAN_PROTOCOLS_IN_A_SEARCH,
                          fake_can_setup_count());
}

TEST(a_search_stops_at_can_11_bit_when_an_ecu_answers) {
    elm_echo_off();
    elm_ok("ATH1\r");
    fake_can_stage_response(CAN_11BIT_REPLY_ID, 8, can_reply, 5);

    TEST_ASSERT_EQUAL_STRING(SEARCHING
                             "7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));

    /* Protocol 6 is CAN 11 bit 500 kbaud, and the A marks it as found. It is
     * tried first because it is what most vehicles answer to, so a bus that
     * speaks it costs one bring-up rather than four. */
    TEST_ASSERT_EQUAL_STRING("A6\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_TRUE(fake_can_is_up());
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(0, fake_bus_setup_count(FAKE_BUS_KLINE));
}

TEST(a_search_reaches_can_29_bit_when_the_11_bit_protocols_are_silent) {
    elm_echo_off();
    elm_ok("ATH1\r");

    /* Released by the second transmit: the first is protocol 6's request,
     * which this ECU does not answer because it is not addressed to it. */
    fake_can_stage_response_at(CAN_29BIT_REPLY_ID, 8, can_reply, 5, 2);

    TEST_ASSERT_EQUAL_STRING(SEARCHING
                             "18DAF110 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));

    TEST_ASSERT_EQUAL_STRING("A7\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_EQUAL_INT(2, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(0, fake_bus_setup_count(FAKE_BUS_KLINE));
}

TEST(a_search_reaches_the_250_kbaud_protocols) {
    elm_echo_off();
    elm_ok("ATH1\r");

    /* The third request out is protocol 8's: 11 bit at 500 kbaud, then
     * 29 bit at 500 kbaud, then 11 bit at 250 kbaud. */
    fake_can_stage_response_at(CAN_11BIT_REPLY_ID, 8, can_reply, 5, 3);

    TEST_ASSERT_EQUAL_STRING(SEARCHING
                             "7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));

    TEST_ASSERT_EQUAL_STRING("A8\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_EQUAL_INT(250000, fake_can_last_baud());
}

TEST(a_search_falls_through_to_kline_when_can_is_silent) {
    elm_echo_off();
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_KLINE, bus_reply, sizeof(bus_reply), 5);

    TEST_ASSERT_EQUAL_STRING(SEARCHING
                             "48 6B 10 41 00 BE 3F B8 13 C4 \r" ELM_PROMPT,
                             elm_ask("0100\r"));

    TEST_ASSERT_EQUAL_STRING("A3\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_FALSE(fake_can_is_up());
    TEST_ASSERT_TRUE(fake_bus_is_up(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_setup_count(FAKE_BUS_J1850_PWM));
}

TEST(a_search_falls_through_to_j1850_pwm) {
    elm_echo_off();
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, bus_reply, sizeof(bus_reply),
                            5);

    TEST_ASSERT_EQUAL_STRING(SEARCHING
                             "48 6B 10 41 00 BE 3F B8 13 C4 \r" ELM_PROMPT,
                             elm_ask("0100\r"));

    TEST_ASSERT_EQUAL_STRING("A1\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_setup_count(FAKE_BUS_J1850_VPW));
}

TEST(a_search_falls_through_to_j1850_vpw_last) {
    elm_echo_off();
    elm_ok("ATH1\r");
    fake_bus_stage_response(FAKE_BUS_J1850_VPW, bus_reply, sizeof(bus_reply),
                            5);

    TEST_ASSERT_EQUAL_STRING(SEARCHING
                             "48 6B 10 41 00 BE 3F B8 13 C4 \r" ELM_PROMPT,
                             elm_ask("0100\r"));

    TEST_ASSERT_EQUAL_STRING("A2\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_TRUE(fake_bus_is_up(FAKE_BUS_J1850_VPW));
}

TEST(a_protocol_found_by_search_is_used_for_later_requests_without_searching) {
    elm_echo_off();
    elm_ok("ATH1\r");
    fake_can_stage_response(CAN_11BIT_REPLY_ID, 8, can_reply, 5);
    elm_ask("0100\r");

    fake_can_stage_response(CAN_11BIT_REPLY_ID, 8, can_reply, 5);

    /* Protocol 6 is live now, so this goes straight out on CAN. */
    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
}

/* ------------------------------------------------------------------ *
 * AT SP 0 and re-searching
 * ------------------------------------------------------------------ */

TEST(a_search_that_found_nothing_is_tried_again_on_the_next_request) {
    static const uint8_t reply[] = {0x48, 0x6B, 0x10, 0x41, 0x00, 0xBE, 0x22};

    elm_echo_off();
    elm_ok("ATSP0\r");

    /* Nothing staged anywhere, so the first search finds nothing. */
    TEST_ASSERT_EQUAL_STRING("SEARCHING...\rNO DATA\r" ELM_PROMPT,
                             elm_ask("0100\r"));

    /* The vehicle wakes up. A failed search must not latch the adapter into
     * NO DATA - the datasheet expects "Subsequent OBD requests may show
     * 'SEARCHING'", and a module that was asleep a moment ago is the ordinary
     * reason a first search comes back empty. */
    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING("SEARCHING...\r41 00 BE \r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(at_sp_0_keeps_the_bus_that_is_already_working) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");

    elm_ok("ATSP0\r");

    /* AT SP 0 says the protocol is unknown, not that this one is wrong. The
     * session on it took two and a half seconds to establish and is still
     * the best guess there is. */
    TEST_ASSERT_EQUAL_INT(0, fake_bus_teardown_count(FAKE_BUS_KLINE));
    TEST_ASSERT_TRUE(fake_bus_is_up(FAKE_BUS_KLINE));
}

TEST(a_search_after_at_sp_0_tries_the_working_protocol_first) {
    static const uint8_t reply[] = {0x48, 0x6B, 0x10, 0x41, 0x00, 0xBE, 0x22};

    elm_echo_off();
    elm_kline_select("ATSP3\r");
    elm_ok("ATSP0\r");

    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);
    TEST_ASSERT_EQUAL_STRING("SEARCHING...\r41 00 BE \r" ELM_PROMPT,
                             elm_ask("0100\r"));

    /* Straight back to K-Line without going near CAN, and without the driver
     * being cycled - which is what would have cost the session on it. */
    TEST_ASSERT_EQUAL_INT(0, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(1, fake_bus_setup_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_teardown_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_STRING("A3\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(a_search_that_found_nothing_reports_no_protocol) {
    elm_echo_off();
    elm_ok("ATSP0\r");

    elm_ask("0100\r");

    /* AT DPN prefixes with A only when a search actually arrived somewhere.
     * A search that found nothing arrived at nothing. */
    TEST_ASSERT_EQUAL_STRING("0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_KLINE));
}

TEST(naming_a_protocol_after_at_sp_0_closes_whatever_was_open) {
    elm_echo_off();
    elm_kline_select("ATSP3\r");
    elm_ok("ATSP0\r");

    /* Automatic is the only selection that keeps the bus. Naming a different
     * protocol is a switch, and the datasheet is explicit that the ELM327
     * "automatically performs [a protocol close] when you switch protocols". */
    elm_ok("ATSP1\r");

    TEST_ASSERT_EQUAL_INT(1, fake_bus_teardown_count(FAKE_BUS_KLINE));
    TEST_ASSERT_TRUE(fake_bus_is_up(FAKE_BUS_J1850_PWM));
}

/* ------------------------------------------------------------------ *
 * Naming a protocol: AT SP h, AT SP Ah, AT TP h, AT TP Ah
 * ------------------------------------------------------------------ */

TEST(at_tp_selects_a_protocol) {
    elm_echo_off();

    /* "This command is identical to the SP command, except that the protocol
     * that you select is not immediately saved in internal EEPROM memory."
     * There is no EEPROM here, so the two are the same command - but AT TP
     * used to be answered with '?', and python-obd reaches for it whenever
     * the adapter's own search has not settled on a protocol. */
    elm_ok("ATTP6\r");

    TEST_ASSERT_EQUAL_STRING("6\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_TRUE(fake_can_is_up());
    TEST_ASSERT_EQUAL_INT(500000, fake_can_last_baud());
}

TEST(at_sp_with_the_automatic_marker_selects_the_protocol_beside_it) {
    elm_echo_off();

    /* Not protocol 0xA. hex_char_to_int() read the 'A' as a digit, so AT SP A6
     * selected SAE J1939 and left the client on a bus its vehicle does not
     * speak - with an OK to say it had worked. */
    elm_ok("ATSPA6\r");

    TEST_ASSERT_EQUAL_STRING("A6\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_EQUAL_INT(500000, fake_can_last_baud());
}

TEST(the_automatic_marker_may_follow_the_protocol) {
    elm_echo_off();

    /* "Note that the 'A' can come before or after the h, so AT SP A3 can also
     * be entered as AT SP 3A." */
    elm_ok("ATSP6A\r");

    TEST_ASSERT_EQUAL_STRING("A6\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_EQUAL_INT(500000, fake_can_last_baud());
}

TEST(a_bare_a_is_protocol_ten) {
    elm_echo_off();

    /* SAE J1939, which is a protocol in its own right. The marker form needs
     * a protocol after it to mark, so one character cannot be a marker. */
    elm_ok("ATSPA\r");

    TEST_ASSERT_EQUAL_STRING("A\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(at_sp_00_is_protocol_zero) {
    elm_echo_off();
    elm_ok("ATSP6\r");

    /* "If you really want to store the value '0' in the internal EEPROM, you
     * must use the AT SP 00 command." No EEPROM here, so what is left of it
     * is the protocol selection itself. */
    elm_ok("ATSP00\r");

    TEST_ASSERT_EQUAL_STRING("0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(a_protocol_argument_that_means_nothing_is_refused) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSPZ\r"));
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSP16\r"));
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSP\r"));
}

/* ------------------------------------------------------------------ *
 * The automatic fallback
 * ------------------------------------------------------------------ */

TEST(a_named_protocol_that_answers_never_reaches_the_fallback) {
    elm_echo_off();
    elm_ok("ATH1\r");
    elm_ok("ATSPA6\r");
    fake_can_stage_response(CAN_11BIT_REPLY_ID, 8, can_reply, 5);

    /* No SEARCHING line: the protocol the client named was right. */
    TEST_ASSERT_EQUAL_STRING("7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
}

TEST(a_named_protocol_that_stays_silent_falls_back_to_a_search) {
    elm_echo_off();
    elm_ok("ATH1\r");
    elm_ok("ATSPA7\r");

    /* Released by the second transmit: the first is protocol 7's request,
     * which this 11 bit ECU does not answer. */
    fake_can_stage_response_at(CAN_11BIT_REPLY_ID, 8, can_reply, 5, 2);

    /* "if the protocol that is tried should fail to initialize, the ELM327
     * will then automatically sequence through the other protocols,
     * attempting to connect to one of them." */
    TEST_ASSERT_EQUAL_STRING(SEARCHING
                             "7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_STRING("A6\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(the_fallback_does_not_retry_the_protocol_that_was_named) {
    elm_echo_off();
    elm_ok("ATSPA7\r");

    elm_ask("0100\r");

    /* Four CAN bring-ups, not five: protocol 7 was the named attempt, and the
     * search that follows skips it rather than asking a second time. */
    TEST_ASSERT_EQUAL_INT(CAN_PROTOCOLS_IN_A_SEARCH, fake_can_setup_count());
}

TEST(a_protocol_named_without_the_marker_does_not_fall_back) {
    elm_echo_off();
    elm_ok("ATSP7\r");

    /* "that protocol will become the default, and will be the only protocol
     * used by the ELM327 ... no other protocols will be attempted." */
    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
    TEST_ASSERT_EQUAL_STRING("7\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(a_fallback_search_runs_once_for_the_request_that_triggered_it) {
    elm_echo_off();
    elm_ok("ATSPA7\r");

    elm_ask("0100\r");

    /* One named attempt on protocol 7, then one search over the other three
     * CAN protocols. Four bring-ups, not eight: the marker arms the search
     * that follows the failure, and does not also leave the request looking
     * like one that never had a protocol at all. */
    TEST_ASSERT_EQUAL_INT(CAN_PROTOCOLS_IN_A_SEARCH, fake_can_setup_count());

    /* And the marker itself is gone. What is left is an ordinary unknown
     * protocol, which is a different thing: a client that reads AT DPN here
     * must not be told the adapter is still holding protocol 7 for it. */
    TEST_ASSERT_EQUAL_STRING("0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(a_vehicle_that_wakes_up_after_a_failed_fallback_is_still_found) {
    elm_echo_off();
    elm_ok("ATH1\r");
    elm_ok("ATSPA7\r");

    /* Nothing staged, so the named protocol and the search behind it both
     * come back empty. */
    TEST_ASSERT_EQUAL_STRING(SEARCHING "NO DATA\r" ELM_PROMPT,
                             elm_ask("0100\r"));

    /* The vehicle wakes up. A failed fallback leaves the session knowing no
     * protocol, which is the ordinary state a request searches from - so this
     * is found, and found on 6 rather than on the 7 that was named and
     * failed. */
    fake_can_stage_response(CAN_11BIT_REPLY_ID, 8, can_reply, 5);

    TEST_ASSERT_EQUAL_STRING(SEARCHING
                             "7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_STRING("A6\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(at_pc_clears_a_pending_fallback) {
    elm_echo_off();
    elm_ok("ATSPA6\r");

    elm_ok("ATPC\r");

    /* Letting go of the bus lets go of what was going to happen on it. */
    TEST_ASSERT_EQUAL_STRING("0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

/* ------------------------------------------------------------------ *
 * AT SS
 * ------------------------------------------------------------------ */

TEST(at_ss_searches_in_the_j1978_order) {
    static const uint8_t reply[] = {0x48, 0x6B, 0x10, 0x41, 0x00, 0xBE, 0x22};

    elm_echo_off();
    elm_ok("ATSS\r");
    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);

    /* "SAE standard J1978 specifies a protocol search order that scan tools
     * should use. It follows the number order that we have assigned to the
     * ELM327 protocols." J1850 first, then ISO 9141 - so a K-Line vehicle is
     * reached without the CAN driver being brought up at all, which is the
     * reverse of the fast order. */
    TEST_ASSERT_EQUAL_STRING(SEARCHING "41 00 BE \r" ELM_PROMPT,
                             elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_setup_count());
    TEST_ASSERT_EQUAL_STRING("A3\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(the_j1978_order_still_reaches_can) {
    elm_echo_off();
    elm_ok("ATH1\r");
    elm_ok("ATSS\r");
    fake_can_stage_response(CAN_11BIT_REPLY_ID, 8, can_reply, 5);

    TEST_ASSERT_EQUAL_STRING(SEARCHING
                             "7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_STRING("A6\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(the_search_after_a_failed_one_still_opens_on_can) {
    elm_echo_off();
    elm_ok("ATH1\r");

    /* Nothing staged anywhere, so this walks the whole order and gives up on
     * J1850 VPW, which is last in it. */
    elm_ask("0100\r");

    int can_before = fake_can_setup_count();
    int vpw_before = fake_bus_setup_count(FAKE_BUS_J1850_VPW);

    /* The vehicle wakes up on protocol 6, which the order starts with, so
     * finding it has to cost exactly one CAN bring-up and touch nothing else.
     *
     * Selecting automatic on the way out of a failed search remembers
     * whatever was current as the protocol to try first next time. That is
     * right after AT SP 0 from a working protocol and precisely wrong here:
     * what is current at that point is the last protocol of the search that
     * just failed. The order drifted because of it - the second search opened
     * on J1850 VPW, the third on J1850 PWM - and each one spent a bus
     * bring-up on a protocol that had just been shown not to answer. */
    fake_can_stage_response(CAN_11BIT_REPLY_ID, 8, can_reply, 5);

    TEST_ASSERT_EQUAL_STRING(SEARCHING
                             "7E8 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));

    TEST_ASSERT_EQUAL_INT(can_before + 1, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(vpw_before, fake_bus_setup_count(FAKE_BUS_J1850_VPW));
}

TEST(protocol_switch_stops_when_old_bus_cannot_close) {
    elm_echo_off();
    elm_ok("ATSP1\r");
    fake_bus_close_result(FAKE_BUS_J1850_PWM, ESP_ERR_TIMEOUT);
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSP6\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(VIF_BUS_J1850_PWM,
                          vif_bus_current(VIF_OWNER_LINK, VIF_BUS_J1850_PWM));
    fake_bus_close_result(FAKE_BUS_J1850_PWM, ESP_OK);
    elm_ok("ATSP6\r");
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
}

TEST(protocol_close_reports_failure_and_allows_retry) {
    elm_echo_off();
    elm_ok("ATSP2\r");
    fake_bus_close_result(FAKE_BUS_J1850_VPW, ESP_ERR_TIMEOUT);
    TEST_ASSERT_EQUAL_STRING("ERROR\r" ELM_PROMPT, elm_ask("ATPC\r"));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_J1850_VPW,
                          vif_bus_current(VIF_OWNER_LINK, VIF_BUS_J1850_VPW));
    TEST_ASSERT_EQUAL_STRING("N\r" ELM_PROMPT, elm_ask("ATIA\r"));
    fake_bus_close_result(FAKE_BUS_J1850_VPW, ESP_OK);
    elm_ok("ATPC\r");
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE,
                          vif_bus_current(VIF_OWNER_LINK, VIF_BUS_J1850_VPW));
}

TEST(retained_failed_open_is_not_reused_as_a_working_bus) {
    elm_echo_off();
    fake_bus_open_result(FAKE_BUS_J1850_PWM, ESP_FAIL);
    fake_bus_close_result(FAKE_BUS_J1850_PWM, ESP_ERR_TIMEOUT);
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSP1\r"));
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSP1\r"));
    fake_bus_open_result(FAKE_BUS_J1850_PWM, ESP_OK);
    fake_bus_close_result(FAKE_BUS_J1850_PWM, ESP_OK);
    elm_ok("ATSP1\r");
    TEST_ASSERT_EQUAL_INT(2, fake_bus_setup_count(FAKE_BUS_J1850_PWM));
}

TEST(reset_commands_report_failed_close_and_can_be_retried) {
    const char *commands[] = {"ATD\r", "ATZ\r", "ATWS\r"};
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
        elm_echo_off();
        elm_ok("ATSP1\r");
        fake_bus_close_result(FAKE_BUS_J1850_PWM, ESP_ERR_TIMEOUT);
        TEST_ASSERT_EQUAL_STRING("ERROR\r" ELM_PROMPT, elm_ask(commands[i]));
        TEST_ASSERT_EQUAL_INT(
            VIF_BUS_J1850_PWM,
            vif_bus_current(VIF_OWNER_LINK, VIF_BUS_J1850_PWM));
        fake_bus_close_result(FAKE_BUS_J1850_PWM, ESP_OK);
        elm_ask(commands[i]);
        TEST_ASSERT_EQUAL_INT(
            VIF_BUS_NONE, vif_bus_current(VIF_OWNER_LINK, VIF_BUS_J1850_PWM));
    }
}

TEST(selecting_same_protocol_after_failed_close_reopens_driver) {
    elm_echo_off();
    elm_ok("ATSP2\r");
    fake_bus_close_result(FAKE_BUS_J1850_VPW, ESP_ERR_TIMEOUT);
    TEST_ASSERT_EQUAL_STRING("ERROR\r" ELM_PROMPT, elm_ask("ATPC\r"));
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSP2\r"));
    fake_bus_close_result(FAKE_BUS_J1850_VPW, ESP_OK);
    elm_ok("ATSP2\r");
    TEST_ASSERT_EQUAL_INT(2, fake_bus_setup_count(FAKE_BUS_J1850_VPW));
    TEST_ASSERT_EQUAL_STRING("Y\r" ELM_PROMPT, elm_ask("ATIA\r"));
}
