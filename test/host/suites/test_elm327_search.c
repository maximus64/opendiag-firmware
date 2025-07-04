/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_elm327_search.c
 * @brief Automatic protocol search, the path taken when no ATSP was issued.
 *
 * The search tries CAN 29 bit, then ISO 9141, then J1850 PWM, then J1850 VPW,
 * keeping whichever answers first.
 */

#include "elm327_harness.h"

static const uint8_t can_reply[8] = {
    0x06, 0x41, 0x00, 0xBE, 0x3F, 0xB8, 0x13, 0x00
};

static const uint8_t bus_reply[10] = {
    0x48, 0x6B, 0x10, 0x41, 0x00, 0xBE, 0x3F, 0xB8, 0x13, 0xC4
};

#define SEARCHING "SEARCHING...\r"

TEST(a_request_without_a_protocol_starts_a_search)
{
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING(SEARCHING "NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_search_tries_every_bus_before_giving_up)
{
    elm_echo_off();

    elm_ask("0100\r");

    TEST_ASSERT_MSG(fake_can_setup_count() == 1, "CAN was not tried");
    TEST_ASSERT_MSG(fake_bytebus_setup_count(FAKE_BUS_KLINE) == 1,
                    "K-Line was not tried");
    TEST_ASSERT_MSG(fake_bytebus_setup_count(FAKE_BUS_J1850_PWM) == 1,
                    "J1850 PWM was not tried");
    TEST_ASSERT_MSG(fake_bytebus_setup_count(FAKE_BUS_J1850_VPW) == 1,
                    "J1850 VPW was not tried");
}

TEST(a_failed_search_leaves_no_bus_running)
{
    elm_echo_off();

    elm_ask("0100\r");

    TEST_ASSERT_FALSE(fake_can_is_up());
    TEST_ASSERT_FALSE(fake_bytebus_is_up(FAKE_BUS_KLINE));
    TEST_ASSERT_FALSE(fake_bytebus_is_up(FAKE_BUS_J1850_PWM));
    TEST_ASSERT_FALSE(fake_bytebus_is_up(FAKE_BUS_J1850_VPW));

    TEST_ASSERT_EQUAL_STRING("A0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(a_second_request_after_a_failed_search_does_not_search_again)
{
    elm_echo_off();
    elm_ask("0100\r");

    /* Already searched once and found nothing; saying so again is quicker
     * than another four bus bring-ups. */
    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
}

TEST(a_search_stops_at_can_when_an_ecu_answers)
{
    elm_echo_off();
    fake_can_stage_response(0x18DAF110 | CAN_EFF_FLAG, 8, can_reply, 5);

    TEST_ASSERT_EQUAL_STRING(
        SEARCHING "18DAF110 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
        elm_ask("0100\r"));

    /* Protocol 7 is CAN 29 bit 500 kbaud, and the A marks it as found. */
    TEST_ASSERT_EQUAL_STRING("A7\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_TRUE(fake_can_is_up());
    TEST_ASSERT_EQUAL_INT(0, fake_bytebus_setup_count(FAKE_BUS_KLINE));
}

TEST(a_search_falls_through_to_kline_when_can_is_silent)
{
    elm_echo_off();
    fake_bytebus_stage_response(FAKE_BUS_KLINE, bus_reply, sizeof(bus_reply), 5);

    TEST_ASSERT_EQUAL_STRING(
        SEARCHING "48 6B 10 41 00 BE 3F B8 13 C4 \r" ELM_PROMPT,
        elm_ask("0100\r"));

    TEST_ASSERT_EQUAL_STRING("A3\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_FALSE(fake_can_is_up());
    TEST_ASSERT_TRUE(fake_bytebus_is_up(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(0, fake_bytebus_setup_count(FAKE_BUS_J1850_PWM));
}

TEST(a_search_falls_through_to_j1850_pwm)
{
    elm_echo_off();
    fake_bytebus_stage_response(FAKE_BUS_J1850_PWM, bus_reply,
                                sizeof(bus_reply), 5);

    TEST_ASSERT_EQUAL_STRING(
        SEARCHING "48 6B 10 41 00 BE 3F B8 13 C4 \r" ELM_PROMPT,
        elm_ask("0100\r"));

    TEST_ASSERT_EQUAL_STRING("A1\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_bytebus_setup_count(FAKE_BUS_J1850_VPW));
}

TEST(a_search_falls_through_to_j1850_vpw_last)
{
    elm_echo_off();
    fake_bytebus_stage_response(FAKE_BUS_J1850_VPW, bus_reply,
                                sizeof(bus_reply), 5);

    TEST_ASSERT_EQUAL_STRING(
        SEARCHING "48 6B 10 41 00 BE 3F B8 13 C4 \r" ELM_PROMPT,
        elm_ask("0100\r"));

    TEST_ASSERT_EQUAL_STRING("A2\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_TRUE(fake_bytebus_is_up(FAKE_BUS_J1850_VPW));
}

TEST(a_protocol_found_by_search_is_used_for_later_requests_without_searching)
{
    elm_echo_off();
    fake_can_stage_response(0x18DAF110 | CAN_EFF_FLAG, 8, can_reply, 5);
    elm_ask("0100\r");

    fake_can_stage_response(0x18DAF110 | CAN_EFF_FLAG, 8, can_reply, 5);

    /* Protocol 7 is live now, so this goes straight out on CAN. */
    TEST_ASSERT_EQUAL_STRING("18DAF110 06 41 00 BE 3F B8 13 00 \r" ELM_PROMPT,
                             elm_ask("0100\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
}
