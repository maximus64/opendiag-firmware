/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_elm327_at.c
 * @brief The AT command grammar and the line assembler behind it.
 *
 * Expectations are written as the exact byte stream a client sees, control
 * characters and all, because that is the contract scan tools depend on.
 */

#include "elm327_harness.h"

/* ------------------------------------------------------------------ *
 * Line assembly
 * ------------------------------------------------------------------ */

TEST(echo_is_on_by_default)
{
    TEST_ASSERT_EQUAL_STRING("ATI\r" ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r"));
}

TEST(echo_off_suppresses_the_command)
{
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r"));
}

TEST(echo_on_restores_it)
{
    elm_echo_off();
    TEST_ASSERT_EQUAL_STRING("OK\r" ELM_PROMPT, elm_ask("ATE1\r"));

    TEST_ASSERT_EQUAL_STRING("ATI\r" ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r"));
}

TEST(lower_case_is_folded_to_upper)
{
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("ati\r"));
}

TEST(spaces_are_stripped)
{
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("A T   I\r"));
}

TEST(echo_reflects_the_raw_bytes_not_the_stripped_ones)
{
    /* Spacing and case are normalised for parsing only. What comes back is
     * what was typed, which is what a terminal user expects to see. */
    TEST_ASSERT_EQUAL_STRING("at i\r" ELM_ID "\r" ELM_PROMPT, elm_ask("at i\r"));
}

TEST(a_one_character_line_is_ignored)
{
    elm_echo_off();

    /* Too short to be a command, and too short to mean "repeat". */
    TEST_ASSERT_EQUAL_STRING("", elm_ask("A\r"));
}

TEST(an_empty_line_repeats_the_previous_command)
{
    elm_echo_off();
    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r"));

    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("\r"));
}

TEST(repeating_a_short_command_does_not_replay_the_longer_one_before_it)
{
    /* The stored command is only as long as the command itself. If its
     * terminator were dropped, the tail of the previous, longer command would
     * still be there and the repeat would run "ATI" plus that tail. */
    elm_echo_off();
    elm_ok("ATSH123456\r");
    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r"));

    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("\r"));
}

TEST(repeating_before_any_command_is_rejected)
{
    /* Echo is left on: turning it off would itself become the command to
     * repeat. The leading carriage return is the echoed empty line. */
    TEST_ASSERT_EQUAL_STRING("\r?\r" ELM_PROMPT, elm_ask("\r"));
}

TEST(a_line_longer_than_the_buffer_wraps_without_running_off_the_end)
{
    elm_echo_off();

    /* 32 characters fill the line buffer, which then restarts. Only the tail
     * is parsed, and it must be parsed as an ordinary command. */
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT,
        elm_ask("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" "ATBOGUS1" "\r"));
}

/* ------------------------------------------------------------------ *
 * Identity and reset
 * ------------------------------------------------------------------ */

TEST(at_at1_reports_the_device_name)
{
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("Dalalogic OpenDiag 1.0\r" ELM_PROMPT,
                             elm_ask("AT@1\r"));
}

TEST(an_unknown_at_command_is_rejected)
{
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATNOSUCH\r"));
}

TEST(at_z_identifies_after_the_reset_delay)
{
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("\r\r" ELM_ID "\r" ELM_PROMPT, elm_ask("ATZ\r"));

    /* Some scan tools time the reset, so the wait has to be real. */
    TEST_ASSERT_MSG(fake_clock_ms() >= 1000,
                    "reset returned after only %u ms", fake_clock_ms());
}

TEST(at_z_returns_settings_to_their_defaults)
{
    elm_echo_off();
    elm_ok("ATSP6\r");

    elm_ask("ATZ\r");

    /* Echo back on and the protocol back to automatic. */
    TEST_ASSERT_EQUAL_STRING("ATI\r" ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r"));
    TEST_ASSERT_EQUAL_STRING("ATDPN\r0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(at_z_releases_only_the_pins_this_client_holds)
{
    elm_echo_off();
    elm_ok("ATPROGV6000003E8\r");
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(6));

    elm_ask("ATZ\r");

    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_state(6));
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_boost_en());
    TEST_ASSERT_FALSE(vif_any_pin_active());

    /* A reset here used to call board_hs_ls_reset_state(), which cut the
     * programming voltage for every client on the adapter, whoever raised it.
     * AT Z now releases this session's claims and nobody else's. */
    TEST_ASSERT_EQUAL_INT(0, fake_board_reset_count());
}

TEST(at_d_restores_defaults_without_the_reset_delay)
{
    elm_echo_off();
    elm_ok("ATSP6\r");

    TEST_ASSERT_EQUAL_STRING("OK\r" ELM_PROMPT, elm_ask("ATD\r"));
    TEST_ASSERT_MSG(fake_clock_ms() < 1000, "ATD should not wait a second");

    TEST_ASSERT_EQUAL_STRING("ATDPN\r0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

/* ------------------------------------------------------------------ *
 * Protocol selection
 * ------------------------------------------------------------------ */

TEST(at_sp_brings_up_the_selected_bus)
{
    elm_echo_off();

    elm_ok("ATSP6\r");

    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(500000, fake_can_last_baud());
    TEST_ASSERT_TRUE(fake_can_is_up());
}

TEST(at_sp_accepts_every_documented_protocol_number)
{
    static const struct { const char *cmd; const char *dpn; } cases[] = {
        { "ATSP0\r", "0\r" }, { "ATSP1\r", "1\r" }, { "ATSP2\r", "2\r" },
        { "ATSP3\r", "3\r" }, { "ATSP4\r", "4\r" }, { "ATSP5\r", "5\r" },
        { "ATSP6\r", "6\r" }, { "ATSP7\r", "7\r" }, { "ATSP8\r", "8\r" },
        { "ATSP9\r", "9\r" }, { "ATSPA\r", "10\r" }, { "ATSPB\r", "11\r" },
        { "ATSPC\r", "12\r" },
    };

    elm_echo_off();

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        char expect[16];

        elm_ok(cases[i].cmd);

        snprintf(expect, sizeof(expect), "%s" ELM_PROMPT, cases[i].dpn);
        TEST_ASSERT_EQUAL_STRING(expect, elm_ask("ATDPN\r"));
    }
}

TEST(at_sp_is_refused_while_another_client_holds_the_bus)
{
    const vif_bus_cfg_t cfg = { .bitrate = 500000 };
    vif_session_t *other = vif_session_open("other", COMM_INVALID_PORT_ID);

    elm_echo_off();
    TEST_ASSERT_NOT_NULL(other);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(other, VIF_BUS_CAN, &cfg));

    /* One bus, one holder. Answering OK here would leave the client believing
     * it had CAN while every request came back NO DATA. */
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSP6\r"));
    TEST_ASSERT_EQUAL_STRING("0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());

    vif_session_close(other);

    elm_ok("ATSP6\r");
}

TEST(at_sp_rejects_a_protocol_above_the_range)
{
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSPD\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_setup_count());
}

TEST(at_sp_rejects_a_non_hex_protocol)
{
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSPZ\r"));
}

TEST(selecting_the_active_protocol_again_does_not_restart_the_bus)
{
    elm_echo_off();

    elm_ok("ATSP6\r");
    elm_ok("ATSP6\r");

    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(0, fake_can_teardown_count());
}

TEST(switching_protocols_tears_the_previous_bus_down_first)
{
    elm_echo_off();

    elm_ok("ATSP6\r");
    elm_ok("ATSP2\r");

    TEST_ASSERT_EQUAL_INT(1, fake_can_teardown_count());
    TEST_ASSERT_FALSE(fake_can_is_up());
    TEST_ASSERT_EQUAL_INT(1, fake_bytebus_setup_count(FAKE_BUS_J1850_VPW));
}

TEST(returning_to_automatic_leaves_no_bus_running)
{
    elm_echo_off();

    elm_ok("ATSP6\r");
    elm_ok("ATSP0\r");

    TEST_ASSERT_EQUAL_INT(1, fake_can_teardown_count());
    TEST_ASSERT_FALSE(fake_can_is_up());
}

TEST(at_dpn_marks_an_automatically_found_protocol)
{
    elm_echo_off();

    /* An unanswered request on the automatic protocol runs the search, which
     * leaves the auto flag set even though nothing replied. */
    TEST_ASSERT_EQUAL_STRING("SEARCHING...\rNO DATA\r" ELM_PROMPT,
                             elm_ask("0100\r"));

    TEST_ASSERT_EQUAL_STRING("A0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

/* ------------------------------------------------------------------ *
 * Timeout
 * ------------------------------------------------------------------ */

TEST(at_st_is_silent_on_success)
{
    elm_echo_off();

    /* A deviation from the ELM327 datasheet, which answers OK. Pinned here so
     * that changing it is a deliberate decision. */
    TEST_ASSERT_EQUAL_STRING(ELM_PROMPT, elm_ask("ATST19\r"));
}

TEST(at_st_rejects_non_hex_digits)
{
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSTZZ\r"));
}

TEST(at_st_scales_the_timeout_by_four_milliseconds)
{
    elm_echo_off();
    elm_ok("ATSP6\r");

    /* 0x19 is 25 units of 4 ms, so a silent bus should be given 100 ms. */
    elm_ask("ATST19\r");

    uint32_t before = fake_clock_ms();
    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
    uint32_t waited = fake_clock_ms() - before;

    TEST_ASSERT_MSG(waited >= 100 && waited < 200,
                    "waited %u ms, expected about 100", waited);
}

TEST(at_st_zero_restores_the_default_timeout)
{
    elm_echo_off();
    elm_ok("ATSP6\r");
    elm_ask("ATST19\r");

    elm_ask("ATST00\r");

    uint32_t before = fake_clock_ms();
    elm_ask("0100\r");
    uint32_t waited = fake_clock_ms() - before;

    TEST_ASSERT_MSG(waited >= 200, "waited %u ms, expected at least 200", waited);
}

/* ------------------------------------------------------------------ *
 * Header
 * ------------------------------------------------------------------ */

TEST(at_sh_accepts_an_eleven_bit_header)
{
    elm_echo_off();

    elm_ok("ATSH7E0\r");
}

TEST(at_sh_accepts_a_twenty_nine_bit_header)
{
    elm_echo_off();

    elm_ok("ATSH18DB33F1\r");
}

TEST(at_sh_rejects_a_non_hex_header)
{
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSHZZZ\r"));
}

TEST(at_sh_rejects_a_header_wider_than_thirty_two_bits)
{
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSH123456789\r"));
}

/* ------------------------------------------------------------------ *
 * Battery voltage
 * ------------------------------------------------------------------ */

TEST(at_rv_reports_the_battery_voltage_to_one_decimal)
{
    elm_echo_off();

    fake_board_set_vbatt_mv(12600);
    TEST_ASSERT_EQUAL_STRING("12.6V\r" ELM_PROMPT, elm_ask("ATRV\r"));

    fake_board_set_vbatt_mv(9040);
    TEST_ASSERT_EQUAL_STRING("9.0V\r" ELM_PROMPT, elm_ask("ATRV\r"));
}

/* ------------------------------------------------------------------ *
 * Programming voltage
 * ------------------------------------------------------------------ */

TEST(at_progv_drives_a_high_side_pin)
{
    elm_echo_off();

    elm_ok("ATPROGV6000003E8\r");

    TEST_ASSERT_EQUAL_INT(1000, fake_board_hs_voltage_mv());
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_boost_en());
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(6));
}

TEST(at_progv_switches_a_high_side_pin_back_off)
{
    elm_echo_off();
    elm_ok("ATPROGV6000003E8\r");

    elm_ok("ATPROGV6FFFFFFFF\r");

    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_state(6));
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_boost_en());
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_voltage_mv());
}

TEST(at_progv_shorts_the_low_side_pin_to_ground)
{
    elm_echo_off();

    /* Pin 15, voltage 0xFFFFFFFE, the short-to-ground request. */
    elm_ok("ATPROGVFFFFFFFFE\r");

    TEST_ASSERT_EQUAL_INT(1, fake_board_ls_state(15));
}

TEST(at_progv_releases_the_low_side_pin)
{
    elm_echo_off();
    elm_ok("ATPROGVFFFFFFFFE\r");
    TEST_ASSERT_EQUAL_INT(1, fake_board_ls_state(15));

    elm_ok("ATPROGVFFFFFFFFF\r");

    TEST_ASSERT_EQUAL_INT(0, fake_board_ls_state(15));
}

TEST(switching_off_an_idle_low_side_pin_is_accepted_and_does_nothing)
{
    elm_echo_off();

    elm_ok("ATPROGVFFFFFFFFF\r");

    TEST_ASSERT_EQUAL_INT(-1, fake_board_ls_state(15));
}

TEST(at_progv_refuses_a_voltage_on_the_low_side_pin)
{
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATPROGVF000003E8\r"));
    TEST_ASSERT_EQUAL_INT(-1, fake_board_ls_state(15));
}

TEST(at_progv_refuses_a_pin_the_board_cannot_drive)
{
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATPROGV7000003E8\r"));
}

TEST(at_progv_allows_only_one_high_side_pin_at_a_time)
{
    elm_echo_off();
    elm_ok("ATPROGV6000003E8\r");

    /* J2534 permits one high side and one low side driver at a time. */
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATPROGV9000003E8\r"));
    TEST_ASSERT_EQUAL_INT(-1, fake_board_hs_state(9));
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(6));
}

TEST(at_progv_rejects_a_malformed_voltage)
{
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATPROGV6ZZ\r"));
}

TEST(switching_off_an_idle_pin_is_accepted_and_does_nothing)
{
    elm_echo_off();

    elm_ok("ATPROGV6FFFFFFFF\r");

    TEST_ASSERT_EQUAL_INT(-1, fake_board_hs_state(6));
}

/* ------------------------------------------------------------------ *
 * Display switches
 * ------------------------------------------------------------------ */

TEST(the_display_switches_are_all_accepted)
{
    static const char *const cmds[] = {
        "ATH0\r", "ATH1\r", "ATS0\r", "ATS1\r", "ATL0\r", "ATL1\r",
        "ATM0\r", "ATM1\r", "ATCAF0\r", "ATCAF1\r", "ATE1\r",
    };

    elm_echo_off();

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        TEST_ASSERT_MSG(td_string_equal("OK\r" ELM_PROMPT, elm_ask(cmds[i])),
                        "%s was not accepted, got \"%s\"", cmds[i],
                        td_escape(fake_port_text(0), fake_port_len(0)));

        /* ATE1 puts echo back on; the rest of the loop needs it off. */
        if (i + 1 < sizeof(cmds) / sizeof(cmds[0])) {
            elm_ask("ATE0\r");
        }
    }
}

/* ------------------------------------------------------------------ *
 * Request framing
 * ------------------------------------------------------------------ */

TEST(a_request_longer_than_a_can_frame_is_rejected)
{
    elm_echo_off();
    elm_ok("ATSP6\r");

    /* Nine bytes of hex cannot fit the eight byte staging buffer. Rejecting it
     * here is what keeps it off the stack. */
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("010203040506070809\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}

TEST(a_request_with_a_bad_hex_digit_is_rejected)
{
    elm_echo_off();
    elm_ok("ATSP6\r");

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("01ZZ\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}
