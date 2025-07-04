/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_elm327_bytebus.c
 * @brief J1850 PWM, J1850 VPW and ISO 9141 framing and output formatting.
 *
 * The three share one code shape in elm327_at.c: a three byte header in front
 * of the request, then frames printed until the timeout runs out.
 */

#include "elm327_harness.h"

/* A mode 01 PID 00 reply: three header bytes, six data bytes, one checksum. */
static const uint8_t reply[10] = {
    0x41, 0x6B, 0x10, 0x41, 0x00, 0xBE, 0x3F, 0xB8, 0x13, 0xC4
};

#define REPLY_WITH_HEADER "41 6B 10 41 00 BE 3F B8 13 C4 \r"
#define REPLY_STRIPPED    "41 00 BE 3F B8 13 \r"

/* ------------------------------------------------------------------ *
 * Request framing
 * ------------------------------------------------------------------ */

static void assert_sent(fake_bus_id_t bus, const char *expect, size_t len)
{
    size_t sent_len = 0;
    const uint8_t *sent = fake_bytebus_sent(bus, 0, &sent_len);

    TEST_ASSERT_MSG(sent != NULL, "nothing was transmitted");
    TEST_ASSERT_EQUAL_INT(len, sent_len);
    TEST_ASSERT_EQUAL_MEM(expect, sent, len);
}

TEST(j1850_pwm_prepends_its_default_header)
{
    elm_echo_off();
    elm_ok("ATSP1\r");

    elm_ask("0100\r");

    /* 0x61 0x6A 0xF1: priority, gateway, tester. */
    assert_sent(FAKE_BUS_J1850_PWM, "\x61\x6A\xF1\x01\x00", 5);
}

TEST(j1850_vpw_prepends_its_default_header)
{
    elm_echo_off();
    elm_ok("ATSP2\r");

    elm_ask("0100\r");

    assert_sent(FAKE_BUS_J1850_VPW, "\x68\x6A\xF1\x01\x00", 5);
}

TEST(iso9141_prepends_its_default_header)
{
    elm_echo_off();
    elm_ok("ATSP3\r");

    elm_ask("0100\r");

    assert_sent(FAKE_BUS_KLINE, "\x68\x6A\xF1\x01\x00", 5);
}

TEST(at_sh_overrides_the_header_bytes)
{
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATSH81F110\r");

    elm_ask("0100\r");

    assert_sent(FAKE_BUS_J1850_PWM, "\x81\xF1\x10\x01\x00", 5);
}

TEST(iso9141_runs_the_five_baud_init_when_selected)
{
    elm_echo_off();

    elm_ok("ATSP3\r");

    TEST_ASSERT_EQUAL_INT(1, fake_bytebus_setup_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(1, fake_bytebus_sync_count());
}

TEST(switching_away_tears_the_previous_bus_down)
{
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATSP2\r");

    TEST_ASSERT_EQUAL_INT(1, fake_bytebus_teardown_count(FAKE_BUS_J1850_PWM));
    TEST_ASSERT_FALSE(fake_bytebus_is_up(FAKE_BUS_J1850_PWM));
    TEST_ASSERT_TRUE(fake_bytebus_is_up(FAKE_BUS_J1850_VPW));
}

TEST(a_failed_pwm_transmit_is_reported)
{
    elm_echo_off();
    elm_ok("ATSP1\r");
    fake_bytebus_fail_next_send(FAKE_BUS_J1850_PWM);

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_failed_vpw_transmit_is_reported)
{
    elm_echo_off();
    elm_ok("ATSP2\r");
    fake_bytebus_fail_next_send(FAKE_BUS_J1850_VPW);

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_failed_kline_transmit_is_reported)
{
    elm_echo_off();
    elm_ok("ATSP3\r");
    fake_bytebus_fail_next_send(FAKE_BUS_KLINE);

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("0100\r"));
}

/* ------------------------------------------------------------------ *
 * Reply formatting
 * ------------------------------------------------------------------ */

TEST(a_reply_is_printed_with_its_header_bytes)
{
    elm_echo_off();
    elm_ok("ATSP1\r");
    fake_bytebus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, elm_ask("0100\r"));
}

TEST(headers_off_strips_the_header_and_the_checksum)
{
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH0\r");
    fake_bytebus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(REPLY_STRIPPED ELM_PROMPT, elm_ask("0100\r"));
}

TEST(headers_off_leaves_a_short_frame_alone)
{
    static const uint8_t runt[4] = { 0x41, 0x6B, 0x10, 0xC4 };

    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATH0\r");
    fake_bytebus_stage_response(FAKE_BUS_J1850_PWM, runt, sizeof(runt), 5);

    /* Four bytes are all header and checksum; there is nothing to strip down
     * to, so the frame is passed through rather than emptied. */
    TEST_ASSERT_EQUAL_STRING("41 6B 10 C4 \r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(spacing_off_packs_the_reply)
{
    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATS0\r");
    fake_bytebus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING("416B104100BE3FB813C4\r" ELM_PROMPT,
                             elm_ask("0100\r"));
}

TEST(every_reply_inside_the_window_is_printed)
{
    static const uint8_t second[10] = {
        0x41, 0x6B, 0x18, 0x41, 0x00, 0x80, 0x00, 0x00, 0x00, 0x9F
    };

    elm_echo_off();
    elm_ok("ATSP1\r");
    fake_bytebus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);
    fake_bytebus_stage_response(FAKE_BUS_J1850_PWM, second, sizeof(second), 5);

    TEST_ASSERT_EQUAL_STRING(
        REPLY_WITH_HEADER "41 6B 18 41 00 80 00 00 00 9F \r" ELM_PROMPT,
        elm_ask("0100\r"));
}

TEST(a_frame_count_hint_stops_the_wait_early)
{
    static const uint8_t second[10] = {
        0x41, 0x6B, 0x18, 0x41, 0x00, 0x80, 0x00, 0x00, 0x00, 0x9F
    };

    elm_echo_off();
    elm_ok("ATSP1\r");
    fake_bytebus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);
    fake_bytebus_stage_response(FAKE_BUS_J1850_PWM, second, sizeof(second), 5);

    uint32_t before = fake_clock_ms();
    const char *out = elm_ask("01001\r");
    uint32_t waited = fake_clock_ms() - before;

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, out);
    TEST_ASSERT_MSG(waited < 200, "waited %u ms despite the hint", waited);
}

TEST(a_zero_frame_hint_sends_without_waiting_for_an_answer)
{
    elm_echo_off();
    elm_ok("ATSP1\r");
    fake_bytebus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(ELM_PROMPT, elm_ask("01000\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_bytebus_sent_count(FAKE_BUS_J1850_PWM));
}

TEST(a_zero_frame_hint_on_vpw_sends_without_waiting)
{
    elm_echo_off();
    elm_ok("ATSP2\r");
    fake_bytebus_stage_response(FAKE_BUS_J1850_VPW, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(ELM_PROMPT, elm_ask("01000\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_bytebus_sent_count(FAKE_BUS_J1850_VPW));
}

TEST(a_zero_frame_hint_on_kline_sends_without_waiting)
{
    elm_echo_off();
    elm_ok("ATSP3\r");
    fake_bytebus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(ELM_PROMPT, elm_ask("01000\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_bytebus_sent_count(FAKE_BUS_KLINE));
}

TEST(a_vpw_reply_is_printed_like_a_pwm_one)
{
    elm_echo_off();
    elm_ok("ATSP2\r");
    fake_bytebus_stage_response(FAKE_BUS_J1850_VPW, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_vpw_frame_count_hint_stops_the_wait_early)
{
    static const uint8_t second[10] = {
        0x41, 0x6B, 0x18, 0x41, 0x00, 0x80, 0x00, 0x00, 0x00, 0x9F
    };

    elm_echo_off();
    elm_ok("ATSP2\r");
    fake_bytebus_stage_response(FAKE_BUS_J1850_VPW, reply, sizeof(reply), 5);
    fake_bytebus_stage_response(FAKE_BUS_J1850_VPW, second, sizeof(second), 5);

    uint32_t before = fake_clock_ms();
    const char *out = elm_ask("01001\r");

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, out);
    TEST_ASSERT_MSG(fake_clock_ms() - before < 200, "waited despite the hint");
}

TEST(a_kline_frame_count_hint_stops_the_wait_early)
{
    static const uint8_t second[10] = {
        0x48, 0x6B, 0x18, 0x41, 0x00, 0x80, 0x00, 0x00, 0x00, 0x9F
    };

    elm_echo_off();
    elm_ok("ATSP3\r");
    fake_bytebus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);
    fake_bytebus_stage_response(FAKE_BUS_KLINE, second, sizeof(second), 5);

    uint32_t before = fake_clock_ms();
    const char *out = elm_ask("01001\r");

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, out);
    TEST_ASSERT_MSG(fake_clock_ms() - before < 200, "waited despite the hint");
}

/* ------------------------------------------------------------------ *
 * Silence and stale traffic
 * ------------------------------------------------------------------ */

TEST(a_silent_pwm_bus_reports_no_data)
{
    elm_echo_off();
    elm_ok("ATSP1\r");

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_silent_vpw_bus_reports_no_data)
{
    elm_echo_off();
    elm_ok("ATSP2\r");

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_silent_kline_bus_reports_no_data)
{
    elm_echo_off();
    elm_ok("ATSP3\r");

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(a_reply_arriving_after_the_timeout_is_missed)
{
    elm_echo_off();
    elm_ok("ATSP1\r");
    fake_bytebus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5000);

    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
}

TEST(j1850_discards_traffic_queued_before_the_request)
{
    static const uint8_t leftover[10] = {
        0x41, 0x6B, 0x10, 0x41, 0x0C, 0x1A, 0xF8, 0x00, 0x00, 0x77
    };

    elm_echo_off();
    elm_ok("ATSP1\r");
    fake_bytebus_stage_stale(FAKE_BUS_J1850_PWM, leftover, sizeof(leftover));
    fake_bytebus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 5);

    TEST_ASSERT_EQUAL_STRING(REPLY_WITH_HEADER ELM_PROMPT, elm_ask("0100\r"));
}

TEST(iso9141_still_reports_traffic_queued_before_the_request)
{
    static const uint8_t leftover[10] = {
        0x48, 0x6B, 0x10, 0x41, 0x0C, 0x1A, 0xF8, 0x00, 0x00, 0x77
    };

    elm_echo_off();
    elm_ok("ATSP3\r");
    fake_bytebus_stage_stale(FAKE_BUS_KLINE, leftover, sizeof(leftover));
    fake_bytebus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);

    /* Unlike the J1850 paths, the K-Line transfer does not drain the driver
     * first: that loop is commented out in elm327_iso9141_xfer(). A reply to
     * somebody else's request is therefore attributed to this one. Pinned so
     * the difference is visible rather than surprising. */
    TEST_ASSERT_EQUAL_STRING(
        "48 6B 10 41 0C 1A F8 00 00 77 \r" REPLY_WITH_HEADER ELM_PROMPT,
        elm_ask("0100\r"));
}
