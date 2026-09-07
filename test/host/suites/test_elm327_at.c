/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_elm327_at.c
 * @brief The AT command grammar and the line assembler behind it.
 *
 * Expectations are written as the exact byte stream a client sees, control
 * characters and all, because that is the contract scan tools depend on.
 */

#include "elm327_harness.h"

static const vif_frontend_t passthru_test_frontend = {.name = "j2534"};
static const vif_frontend_t slcan_test_frontend = {.name = "slcan"};

TEST(passthru_command_switches_after_reply_and_discards_pipelined_commands) {
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_frontend_register(&passthru_test_frontend));
    elm_echo_off();
    elm_ok("ATSP6\r");
    TEST_ASSERT_EQUAL_STRING("OK\r" ELM_PROMPT,
                             elm_ask("at vif passthru\rATSP7\r"));
    TEST_ASSERT_EQUAL_STRING("elm327", vif_link_frontend_name());
    TEST_ASSERT_EQUAL_INT(ELM327_PROTO_CAN_11BIT_500K,
                          g_elm->settings.current_protocol);
    TEST_ASSERT_TRUE(vif_link_service());
    TEST_ASSERT_EQUAL_STRING("j2534", vif_link_frontend_name());
    elm_link_down();
    TEST_ASSERT_EQUAL_STRING("elm327", vif_link_frontend_name());
}

TEST(slcan_command_selects_registered_frontend) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_frontend_register(&slcan_test_frontend));
    elm_echo_off();
    elm_ok("AT VIF SLCAN\r");
    TEST_ASSERT_TRUE(vif_link_service());
    TEST_ASSERT_EQUAL_STRING("slcan", vif_link_frontend_name());
    elm_link_down();
}

TEST(frontend_command_requires_an_exact_supported_name) {
    elm_echo_off();
    const char *commands[] = {"AT VIF\r", "AT VIF J2534\r",
                              "AT VIF PASSTHRUX\r", "AT VIF SLCANX\r"};
    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
        TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask(commands[i]));
        TEST_ASSERT_EQUAL_STRING("elm327", vif_link_frontend_name());
    }
}

/* ------------------------------------------------------------------ *
 * Line assembly
 * ------------------------------------------------------------------ */

TEST(echo_is_on_by_default) {
    TEST_ASSERT_EQUAL_STRING("ATI\r" ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r"));
}

TEST(echo_off_suppresses_the_command) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r"));
}

TEST(echo_on_restores_it) {
    elm_echo_off();
    TEST_ASSERT_EQUAL_STRING("OK\r" ELM_PROMPT, elm_ask("ATE1\r"));

    TEST_ASSERT_EQUAL_STRING("ATI\r" ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r"));
}

TEST(lower_case_is_folded_to_upper) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("ati\r"));
}

TEST(spaces_are_stripped) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("A T   I\r"));
}

TEST(echo_reflects_the_raw_bytes_not_the_stripped_ones) {
    /* Spacing and case are normalised for parsing only. What comes back is
     * what was typed, which is what a terminal user expects to see. */
    TEST_ASSERT_EQUAL_STRING("at i\r" ELM_ID "\r" ELM_PROMPT,
                             elm_ask("at i\r"));
}

TEST(a_one_character_line_is_answered_rather_than_ignored) {
    elm_echo_off();

    /* Too short to be a command, and too short to mean "repeat" - but it is
     * still a line, and a line gets a prompt. Answering nothing leaves the
     * client waiting for a prompt that never comes, and one behind for the
     * rest of the session once it gives up. */
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("A\r"));
}

TEST(an_empty_line_repeats_the_previous_command) {
    elm_echo_off();
    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r"));

    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("\r"));
}

TEST(repeating_a_short_command_does_not_replay_the_longer_one_before_it) {
    /* The stored command is only as long as the command itself. If its
     * terminator were dropped, the tail of the previous, longer command would
     * still be there and the repeat would run "ATI" plus that tail. */
    elm_echo_off();
    elm_ok("ATSH123456\r");
    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r"));

    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("\r"));
}

TEST(repeating_before_any_command_is_rejected) {
    /* Echo is left on: turning it off would itself become the command to
     * repeat. The leading carriage return is the echoed empty line. */
    TEST_ASSERT_EQUAL_STRING("\r?\r" ELM_PROMPT, elm_ask("\r"));
}

TEST(a_line_longer_than_the_buffer_wraps_without_running_off_the_end) {
    elm_echo_off();

    /* 32 characters fill the line buffer, which then restarts. Only the tail
     * is parsed, and it must be parsed as an ordinary command. */
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT,
                             elm_ask("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX"
                                     "ATBOGUS1"
                                     "\r"));
}

/* ------------------------------------------------------------------ *
 * Identity and reset
 * ------------------------------------------------------------------ */

TEST(at_at1_reports_the_device_name) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("Dalalogic OpenDiag (host-test)\r" ELM_PROMPT,
                             elm_ask("AT@1\r"));
}

TEST(an_unknown_at_command_is_rejected) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATNOSUCH\r"));
}

TEST(at_z_identifies_after_the_reset_delay) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("\r\r" ELM_ID "\r" ELM_PROMPT, elm_ask("ATZ\r"));

    /* Some scan tools time the reset, so the wait has to be real. */
    TEST_ASSERT_MSG(fake_clock_ms() >= 1000, "reset returned after only %u ms",
                    fake_clock_ms());
}

TEST(at_z_returns_settings_to_their_defaults) {
    elm_echo_off();
    elm_ok("ATSP6\r");

    elm_ask("ATZ\r");

    /* Echo back on and the protocol back to automatic. */
    TEST_ASSERT_EQUAL_STRING("ATI\r" ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r"));
    TEST_ASSERT_EQUAL_STRING("ATDPN\r0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(at_z_releases_only_the_pins_this_client_holds) {
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

TEST(at_d_restores_defaults_without_the_reset_delay) {
    elm_echo_off();
    elm_ok("ATSP6\r");

    TEST_ASSERT_EQUAL_STRING("OK\r" ELM_PROMPT, elm_ask("ATD\r"));
    TEST_ASSERT_MSG(fake_clock_ms() < 1000, "ATD should not wait a second");

    TEST_ASSERT_EQUAL_STRING("ATDPN\r0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

/* ------------------------------------------------------------------ *
 * Protocol selection
 * ------------------------------------------------------------------ */

TEST(at_sp_brings_up_the_selected_bus) {
    elm_echo_off();

    elm_ok("ATSP6\r");

    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(500000, fake_can_last_baud());
    TEST_ASSERT_TRUE(fake_can_is_up());
}

TEST(at_sp_accepts_every_documented_protocol_number) {
    static const struct {
        const char *cmd;
        const char *dpn;
    } cases[] = {
        {"ATSP0\r", "0\r"}, {"ATSP1\r", "1\r"}, {"ATSP2\r", "2\r"},
        {"ATSP3\r", "3\r"}, {"ATSP4\r", "4\r"}, {"ATSP5\r", "5\r"},
        {"ATSP6\r", "6\r"}, {"ATSP7\r", "7\r"}, {"ATSP8\r", "8\r"},
        {"ATSP9\r", "9\r"}, {"ATSPA\r", "A\r"}, {"ATSPB\r", "B\r"},
        {"ATSPC\r", "C\r"},
    };

    elm_echo_off();

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        char expect[16];

        elm_ok(cases[i].cmd);

        snprintf(expect, sizeof(expect), "%s" ELM_PROMPT, cases[i].dpn);
        TEST_ASSERT_EQUAL_STRING(expect, elm_ask("ATDPN\r"));
    }
}

TEST(at_sp_is_refused_while_the_shell_holds_the_bus) {
    const vif_bus_cfg_t cfg = {.bitrate = 500000};

    elm_echo_off();

    /* This thread stands in for the link task; it is also the only thread
     * there is, so it can be the shell's for as long as the shell holds
     * something. */
    vif_shell_bind();
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_bus_open(VIF_OWNER_SHELL, VIF_BUS_CAN, &cfg));

    /* One bus, one holder. Answering OK here would leave the client believing
     * it had CAN while every request came back NO DATA. */
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSP6\r"));
    TEST_ASSERT_EQUAL_STRING("0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_release_all(VIF_OWNER_SHELL));

    elm_ok("ATSP6\r");
}

/* ------------------------------------------------------------------ *
 * The transmit buffer
 *
 * Filled and drained by the one task, so it is a plain array. What matters is
 * the discipline around it: a response too long to hold is pushed out as it
 * is composed rather than truncated, and nothing is ever written past the
 * end. No command on this bench composes 512 bytes in one reply - that takes
 * a segmented response from a real ECU - so it is exercised directly here.
 * ------------------------------------------------------------------ */

TEST(a_response_too_long_to_hold_is_pushed_out_as_it_is_composed) {
    uint8_t chunk[200];

    elm_echo_off();
    fake_port_reset();

    memset(chunk, 'A', sizeof(chunk));

    /* Two fit; nothing has been said yet. */
    TEST_ASSERT_EQUAL_INT(sizeof(chunk),
                          elm327_uart_send_bytes(g_elm, chunk, sizeof(chunk)));
    TEST_ASSERT_EQUAL_INT(sizeof(chunk),
                          elm327_uart_send_bytes(g_elm, chunk, sizeof(chunk)));
    TEST_ASSERT_EQUAL_INT(0, (int)fake_port_len(0));

    /* The third does not, so the first two go out to make room for it. */
    TEST_ASSERT_EQUAL_INT(sizeof(chunk),
                          elm327_uart_send_bytes(g_elm, chunk, sizeof(chunk)));
    TEST_ASSERT_EQUAL_INT(400, (int)fake_port_len(0));

    elm327_uart_flush(g_elm);
    TEST_ASSERT_EQUAL_INT(600, (int)fake_port_len(0));

    /* Every byte arrived, and only those bytes. */
    for (size_t i = 0; i < 600; i++) {
        TEST_ASSERT_MSG(fake_port_text(0)[i] == 'A', "byte %zu is not ours", i);
    }
}

TEST(a_single_write_larger_than_the_whole_buffer_is_refused) {
    static uint8_t huge[ELM327_TX_BUF + 1];

    elm_echo_off();
    fake_port_reset();

    memset(huge, 'B', sizeof(huge));

    /* Refused rather than written past the end, and the caller is told. */
    TEST_ASSERT_EQUAL_INT(0, elm327_uart_send_bytes(g_elm, huge, sizeof(huge)));

    elm327_uart_flush(g_elm);
    TEST_ASSERT_EQUAL_INT(0, (int)fake_port_len(0));
}

TEST(at_sp_rejects_a_protocol_above_the_range) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSPD\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_setup_count());
}

TEST(at_sp_rejects_a_non_hex_protocol) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSPZ\r"));
}

TEST(selecting_the_active_protocol_again_does_not_restart_the_bus) {
    elm_echo_off();

    elm_ok("ATSP6\r");
    elm_ok("ATSP6\r");

    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(0, fake_can_teardown_count());
}

TEST(switching_protocols_tears_the_previous_bus_down_first) {
    elm_echo_off();

    elm_ok("ATSP6\r");
    elm_ok("ATSP2\r");

    TEST_ASSERT_EQUAL_INT(1, fake_can_teardown_count());
    TEST_ASSERT_FALSE(fake_can_is_up());
    TEST_ASSERT_EQUAL_INT(1, fake_bus_setup_count(FAKE_BUS_J1850_VPW));
}

TEST(returning_to_automatic_keeps_the_bus_as_the_searchs_first_guess) {
    elm_echo_off();

    elm_ok("ATSP6\r");
    elm_ok("ATSP0\r");

    /* AT SP 0 says the protocol is unknown, not that this one is wrong, so
     * the bus stays up as the search's first guess. AT PC is the command that
     * lets go. */
    TEST_ASSERT_EQUAL_INT(0, fake_can_teardown_count());
    TEST_ASSERT_TRUE(fake_can_is_up());

    elm_ok("ATPC\r");

    TEST_ASSERT_EQUAL_INT(1, fake_can_teardown_count());
    TEST_ASSERT_FALSE(fake_can_is_up());
}

TEST(at_dpn_marks_an_automatically_found_protocol) {
    static const uint8_t reply[] = {0x48, 0x6B, 0x10, 0x41, 0x00, 0xBE, 0x22};

    elm_echo_off();
    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 5);

    elm_ask("0100\r");

    /* The A prefix says the protocol was arrived at by searching rather than
     * named by the client. */
    TEST_ASSERT_EQUAL_STRING("A3\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(at_dpn_does_not_mark_a_search_that_found_nothing) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("SEARCHING...\rNO DATA\r" ELM_PROMPT,
                             elm_ask("0100\r"));

    /* Nothing was found, so there is no automatically found protocol to
     * report. "A0" would be claiming the search settled on protocol 0. */
    TEST_ASSERT_EQUAL_STRING("0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

/* ------------------------------------------------------------------ *
 * Timeout
 * ------------------------------------------------------------------ */

TEST(at_st_acknowledges_the_setting) {
    elm_echo_off();

    /* Setting changes answer OK; the datasheet states the rule once and for
     * all of them. A client that reads until OK used to be left waiting. */
    TEST_ASSERT_EQUAL_STRING("OK\r" ELM_PROMPT, elm_ask("ATST19\r"));
    TEST_ASSERT_EQUAL_STRING("OK\r" ELM_PROMPT, elm_ask("ATST00\r"));
    TEST_ASSERT_EQUAL_STRING("OK\r" ELM_PROMPT, elm_ask("ATSTFF\r"));
}

TEST(at_st_rejects_an_argument_that_is_not_two_hex_digits) {
    static const char *const cmds[] = {
        "ATSTZZ\r",
        "ATST1\r",
        "ATST190\r",
        "ATST\r",
    };

    elm_echo_off();

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        TEST_ASSERT_MSG(td_string_equal("?\r" ELM_PROMPT, elm_ask(cmds[i])),
                        "%s was accepted, got \"%s\"", cmds[i],
                        td_escape(fake_port_text(0), fake_port_len(0)));
    }
}

TEST(at_st_scales_the_timeout_by_four_milliseconds) {
    elm_echo_off();
    elm_ok("ATSP6\r");

    /* 0x19 is 25 units of 4 ms, so a silent bus should be given 100 ms. */
    elm_ok("ATST19\r");

    uint32_t before = fake_clock_ms();
    TEST_ASSERT_EQUAL_STRING("NO DATA\r" ELM_PROMPT, elm_ask("0100\r"));
    uint32_t waited = fake_clock_ms() - before;

    TEST_ASSERT_MSG(waited >= 100 && waited < 200,
                    "waited %u ms, expected about 100", waited);
}

TEST(at_st_zero_restores_the_default_timeout) {
    elm_echo_off();
    elm_ok("ATSP6\r");
    elm_ok("ATST19\r");

    elm_ok("ATST00\r");

    uint32_t before = fake_clock_ms();
    elm_ask("0100\r");
    uint32_t waited = fake_clock_ms() - before;

    TEST_ASSERT_MSG(waited >= 200, "waited %u ms, expected at least 200",
                    waited);
}

/* ------------------------------------------------------------------ *
 * Header
 * ------------------------------------------------------------------ */

TEST(at_sh_accepts_an_eleven_bit_header) {
    elm_echo_off();

    elm_ok("ATSH7E0\r");
}

TEST(at_sh_accepts_a_twenty_nine_bit_header) {
    elm_echo_off();

    elm_ok("ATSH18DB33F1\r");
}

TEST(at_sh_rejects_a_non_hex_header) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSHZZZ\r"));
}

TEST(at_sh_rejects_a_header_wider_than_thirty_two_bits) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATSH123456789\r"));
}

/* ------------------------------------------------------------------ *
 * Battery voltage
 * ------------------------------------------------------------------ */

TEST(at_rv_reports_the_battery_voltage_to_one_decimal) {
    elm_echo_off();

    fake_board_set_vbatt_mv(12600);
    TEST_ASSERT_EQUAL_STRING("12.6V\r" ELM_PROMPT, elm_ask("ATRV\r"));

    fake_board_set_vbatt_mv(9040);
    TEST_ASSERT_EQUAL_STRING("9.0V\r" ELM_PROMPT, elm_ask("ATRV\r"));
}

/* ------------------------------------------------------------------ *
 * Programming voltage
 * ------------------------------------------------------------------ */

TEST(at_progv_drives_a_high_side_pin) {
    elm_echo_off();

    elm_ok("ATPROGV6000003E8\r");

    TEST_ASSERT_EQUAL_INT(1000, fake_board_hs_voltage_mv());
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_boost_en());
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(6));
}

TEST(at_progv_switches_a_high_side_pin_back_off) {
    elm_echo_off();
    elm_ok("ATPROGV6000003E8\r");

    elm_ok("ATPROGV6FFFFFFFF\r");

    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_state(6));
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_boost_en());
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_voltage_mv());
}

TEST(at_progv_shorts_the_low_side_pin_to_ground) {
    elm_echo_off();

    /* Pin 15, voltage 0xFFFFFFFE, the short-to-ground request. */
    elm_ok("ATPROGVFFFFFFFFE\r");

    TEST_ASSERT_EQUAL_INT(1, fake_board_ls_state(15));
}

TEST(at_progv_releases_the_low_side_pin) {
    elm_echo_off();
    elm_ok("ATPROGVFFFFFFFFE\r");
    TEST_ASSERT_EQUAL_INT(1, fake_board_ls_state(15));

    elm_ok("ATPROGVFFFFFFFFF\r");

    TEST_ASSERT_EQUAL_INT(0, fake_board_ls_state(15));
}

TEST(switching_off_an_idle_low_side_pin_is_accepted_and_does_nothing) {
    elm_echo_off();

    elm_ok("ATPROGVFFFFFFFFF\r");

    TEST_ASSERT_EQUAL_INT(-1, fake_board_ls_state(15));
}

TEST(at_progv_refuses_a_voltage_on_the_low_side_pin) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATPROGVF000003E8\r"));
    TEST_ASSERT_EQUAL_INT(-1, fake_board_ls_state(15));
}

TEST(at_progv_refuses_a_pin_the_board_cannot_drive) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATPROGV7000003E8\r"));
}

TEST(at_progv_allows_only_one_high_side_pin_at_a_time) {
    elm_echo_off();
    elm_ok("ATPROGV6000003E8\r");

    /* J2534 permits one high side and one low side driver at a time. */
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATPROGV9000003E8\r"));
    TEST_ASSERT_EQUAL_INT(-1, fake_board_hs_state(9));
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(6));
}

TEST(at_progv_rejects_a_malformed_voltage) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("ATPROGV6ZZ\r"));
}

TEST(switching_off_an_idle_pin_is_accepted_and_does_nothing) {
    elm_echo_off();

    elm_ok("ATPROGV6FFFFFFFF\r");

    TEST_ASSERT_EQUAL_INT(-1, fake_board_hs_state(6));
}

/* ------------------------------------------------------------------ *
 * Display switches
 * ------------------------------------------------------------------ */

TEST(the_display_switches_are_all_accepted) {
    static const char *const cmds[] = {
        "ATH0\r", "ATH1\r", "ATS0\r",   "ATS1\r",   "ATL0\r",
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

TEST(at_l1_puts_a_linefeed_after_every_carriage_return) {
    elm_echo_off();

    /* The acknowledgement for the command that turns linefeeds on already
     * carries them: the setting takes effect before the reply is written. */
    TEST_ASSERT_EQUAL_STRING("OK\r\n\r\n>", elm_ask("ATL1\r"));
    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r\n\r\n>", elm_ask("ATI\r"));

    TEST_ASSERT_EQUAL_STRING("OK\r" ELM_PROMPT, elm_ask("ATL0\r"));
    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r"));
}

TEST(at_l1_linefeeds_the_echo_too) {
    elm_echo_off();
    elm_ask("ATL1\r");
    elm_ask("ATE1\r");

    TEST_ASSERT_EQUAL_STRING("ATI\r\n" ELM_ID "\r\n\r\n>", elm_ask("ATI\r"));
}

typedef enum {
    LINE_ENDING_UNCHANGED,
    LINE_ENDING_FORCED_OFF,
    LINE_ENDING_FORCED_ON,
} line_ending_effect_t;

typedef struct {
    const char *command;
    line_ending_effect_t effect;
} line_ending_case_t;

/** Assert that the entire reply uses one ELM327 line-ending mode consistently.
 */
static void assert_line_endings(const char *command, bool start_with_lf,
                                line_ending_effect_t effect) {
    bool expect_lf = start_with_lf;
    const char *out;
    const char *suffix;
    size_t out_len;
    size_t suffix_len;

    /* Each row is independent. Some commands open a bus, drive a pin, turn
     * echo on, or restore all defaults, and none of that state may influence
     * the row after it. Set echo directly because this test is about the
     * adapter's response framing; echo framing has its own test above. */
    elm327_reset(g_elm);
    g_elm->settings.echo = false;
    g_elm->settings.linefeed = start_with_lf;

    out = elm_ask(command);

    if (effect == LINE_ENDING_FORCED_OFF) {
        expect_lf = false;
    } else if (effect == LINE_ENDING_FORCED_ON) {
        expect_lf = true;
    }

    out_len = strlen(out);
    for (size_t i = 0; i < out_len; i++) {
        if (out[i] == '\r') {
            bool followed_by_lf = i + 1 < out_len && out[i + 1] == '\n';

            TEST_ASSERT_MSG(followed_by_lf == expect_lf,
                            "%s in L%d mode returned inconsistent endings: %s",
                            command, expect_lf ? 1 : 0,
                            td_escape(out, out_len));
        } else if (out[i] == '\n') {
            TEST_ASSERT_MSG(expect_lf && i > 0 && out[i - 1] == '\r',
                            "%s returned a bare LF: %s", command,
                            td_escape(out, out_len));
        }
    }

    /* A normal ELM327 exchange ends the response line, emits one empty line,
     * then prints the prompt. This is the CR CR > shape in the iCar Pro
     * capture, expanded to CR LF CR LF > by AT L1. */
    suffix = expect_lf ? "\r\n\r\n>" : "\r\r>";
    suffix_len = strlen(suffix);
    TEST_ASSERT_MSG(out_len >= suffix_len && memcmp(out + out_len - suffix_len,
                                                    suffix, suffix_len) == 0,
                    "%s did not end with %s: %s", command,
                    td_escape(suffix, suffix_len), td_escape(out, out_len));
}

TEST(every_command_uses_the_selected_line_ending) {
    /* One or more representatives for every recognized branch in
     * elm327_at_command_handler(), including each documented spelling where
     * two spellings share a branch. Argument variants that select distinct
     * settings are included as well. The OBD row covers the non-AT dispatcher
     * and its multi-line SEARCHING / NO DATA response. */
    static const line_ending_case_t cases[] = {
        {"ATZ\r", LINE_ENDING_FORCED_OFF},
        {"ATD\r", LINE_ENDING_FORCED_OFF},
        {"ATD0\r", LINE_ENDING_UNCHANGED},
        {"ATD1\r", LINE_ENDING_UNCHANGED},
        {"ATSP0\r", LINE_ENDING_UNCHANGED},
        {"ATSPA6\r", LINE_ENDING_UNCHANGED},
        {"ATSP6A\r", LINE_ENDING_UNCHANGED},
        {"ATTP6\r", LINE_ENDING_UNCHANGED},
        {"ATST00\r", LINE_ENDING_UNCHANGED},
        {"ATSS\r", LINE_ENDING_UNCHANGED},
        {"ATDPN\r", LINE_ENDING_UNCHANGED},
        {"ATDP\r", LINE_ENDING_UNCHANGED},
        {"ATE0\r", LINE_ENDING_UNCHANGED},
        {"ATE1\r", LINE_ENDING_UNCHANGED},
        {"ATS0\r", LINE_ENDING_UNCHANGED},
        {"ATS1\r", LINE_ENDING_UNCHANGED},
        {"ATH0\r", LINE_ENDING_UNCHANGED},
        {"ATH1\r", LINE_ENDING_UNCHANGED},
        {"ATI\r", LINE_ENDING_UNCHANGED},
        {"ATL0\r", LINE_ENDING_FORCED_OFF},
        {"ATL1\r", LINE_ENDING_FORCED_ON},
        {"ATM0\r", LINE_ENDING_UNCHANGED},
        {"ATM1\r", LINE_ENDING_UNCHANGED},
        {"ATCAF0\r", LINE_ENDING_UNCHANGED},
        {"ATCAF1\r", LINE_ENDING_UNCHANGED},
        {"ATRV\r", LINE_ENDING_UNCHANGED},
        {"ATPROGV6FFFFFFFF\r", LINE_ENDING_UNCHANGED},
        {"ATSH7DF\r", LINE_ENDING_UNCHANGED},
        {"AT@1\r", LINE_ENDING_UNCHANGED},
        {"ATR0\r", LINE_ENDING_UNCHANGED},
        {"ATR1\r", LINE_ENDING_UNCHANGED},
        {"ATAT0\r", LINE_ENDING_UNCHANGED},
        {"ATAT1\r", LINE_ENDING_UNCHANGED},
        {"ATAT2\r", LINE_ENDING_UNCHANGED},
        {"ATAR\r", LINE_ENDING_UNCHANGED},
        {"ATRA10\r", LINE_ENDING_UNCHANGED},
        {"ATSR10\r", LINE_ENDING_UNCHANGED},
        {"ATTAF1\r", LINE_ENDING_UNCHANGED},
        {"ATFT\r", LINE_ENDING_UNCHANGED},
        {"ATFT10\r", LINE_ENDING_UNCHANGED},
        {"ATIFR0\r", LINE_ENDING_UNCHANGED},
        {"ATIFR1\r", LINE_ENDING_UNCHANGED},
        {"ATIFR2\r", LINE_ENDING_UNCHANGED},
        {"ATIFR4\r", LINE_ENDING_UNCHANGED},
        {"ATIFR5\r", LINE_ENDING_UNCHANGED},
        {"ATIFR6\r", LINE_ENDING_UNCHANGED},
        {"ATIFRH\r", LINE_ENDING_UNCHANGED},
        {"ATIFRS\r", LINE_ENDING_UNCHANGED},
        {"ATWS\r", LINE_ENDING_FORCED_OFF},
        {"ATPC\r", LINE_ENDING_UNCHANGED},
        {"ATIIA33\r", LINE_ENDING_UNCHANGED},
        {"ATKW\r", LINE_ENDING_UNCHANGED},
        {"ATKW0\r", LINE_ENDING_UNCHANGED},
        {"ATKW1\r", LINE_ENDING_UNCHANGED},
        {"ATIB10\r", LINE_ENDING_UNCHANGED},
        {"ATIB12\r", LINE_ENDING_UNCHANGED},
        {"ATIB15\r", LINE_ENDING_UNCHANGED},
        {"ATIB48\r", LINE_ENDING_UNCHANGED},
        {"ATIB96\r", LINE_ENDING_UNCHANGED},
        {"ATSW00\r", LINE_ENDING_UNCHANGED},
        {"ATWM686AF10100\r", LINE_ENDING_UNCHANGED},
        {"ATSI\r", LINE_ENDING_UNCHANGED},
        {"ATFI\r", LINE_ENDING_UNCHANGED},
        {"ATBI\r", LINE_ENDING_UNCHANGED},
        {"ATIA\r", LINE_ENDING_UNCHANGED},
        {"ATAL\r", LINE_ENDING_UNCHANGED},
        {"ATNL\r", LINE_ENDING_UNCHANGED},
        {"ATBOGUS\r", LINE_ENDING_UNCHANGED},
        {"0100\r", LINE_ENDING_UNCHANGED},
    };

    for (int start_with_lf = 0; start_with_lf <= 1; start_with_lf++) {
        for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            assert_line_endings(cases[i].command, start_with_lf != 0,
                                cases[i].effect);
        }
    }
}

/* ------------------------------------------------------------------ *
 * Request framing
 * ------------------------------------------------------------------ */

TEST(a_request_longer_than_a_can_frame_is_rejected) {
    elm_echo_off();
    elm_ok("ATSP6\r");

    /* Nine bytes of hex cannot fit the eight byte staging buffer. Rejecting it
     * here is what keeps it off the stack. */
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("010203040506070809\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}

TEST(a_request_with_a_bad_hex_digit_is_rejected) {
    elm_echo_off();
    elm_ok("ATSP6\r");

    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("01ZZ\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}

/* ------------------------------------------------------------------ *
 * Defaults the datasheet marks with an asterisk
 * ------------------------------------------------------------------ */

TEST(headers_are_off_until_asked_for) {
    elm_echo_off();

    /* AT H0 is the documented power-on state. A client relying on it was
     * being handed three header bytes and a checksum on every reply. */
    TEST_ASSERT_FALSE(g_elm->settings.show_header);
    TEST_ASSERT_TRUE(g_elm->settings.responses);
    TEST_ASSERT_TRUE(g_elm->settings.auto_receive);
    TEST_ASSERT_FALSE(g_elm->settings.filter_tx);
    TEST_ASSERT_EQUAL_INT(1, g_elm->settings.adaptive_timing);
    TEST_ASSERT_EQUAL_INT(0xF1, g_elm->settings.tester_address);
    TEST_ASSERT_EQUAL_INT(ELM327_IFR_AUTO, g_elm->settings.ifr_mode);
}

/* ------------------------------------------------------------------ *
 * Line endings
 * ------------------------------------------------------------------ */

TEST(a_crlf_line_ending_does_not_corrupt_the_next_command) {
    elm_echo_off();

    /* The carriage return dispatches; the linefeed used to be kept and ended
     * up at the front of the next line, so every command after the first was
     * read as "\n..." and rejected. */
    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r\n"));
    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r\n"));
    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r\n"));
}

TEST(a_stray_linefeed_does_not_repeat_the_last_command) {
    elm_echo_off();

    elm_ask("ATI\r");

    /* A lone linefeed is not a line. Only a bare carriage return repeats. */
    TEST_ASSERT_EQUAL_STRING("", elm_ask("\n"));
    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("\r"));
}

/* ------------------------------------------------------------------ *
 * Commands added for the J1850 protocols and for client compatibility
 * ------------------------------------------------------------------ */

TEST(the_obd_setting_commands_are_all_accepted) {
    static const char *const cmds[] = {
        "ATR0\r",   "ATR1\r",   "ATAT0\r",   "ATAT1\r",  "ATAT2\r",  "ATAR\r",
        "ATRA10\r", "ATSR10\r", "ATTA F1\r", "ATFT10\r", "ATFT\r",   "ATIFR0\r",
        "ATIFR1\r", "ATIFR2\r", "ATIFR4\r",  "ATIFR5\r", "ATIFR6\r", "ATIFRH\r",
        "ATIFRS\r", "ATPC\r",   "ATAL\r",    "ATNL\r",
    };

    elm_echo_off();

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        TEST_ASSERT_MSG(td_string_equal("OK\r" ELM_PROMPT, elm_ask(cmds[i])),
                        "%s was not accepted, got \"%s\"", cmds[i],
                        td_escape(fake_port_text(0), fake_port_len(0)));
    }
}

TEST(a_malformed_address_argument_is_rejected) {
    static const char *const cmds[] = {
        "ATRA1\r", "ATRAZZ\r", "ATSR123\r", "ATTAG0\r",
        "ATFT1\r", "ATIFR3\r", "ATIFR7\r",  "ATIFRX\r",
    };

    elm_echo_off();

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        TEST_ASSERT_MSG(td_string_equal("?\r" ELM_PROMPT, elm_ask(cmds[i])),
                        "%s was accepted, got \"%s\"", cmds[i],
                        td_escape(fake_port_text(0), fake_port_len(0)));
    }
}

TEST(at_dp_names_the_protocol) {
    elm_echo_off();

    elm_ok("ATSP1\r");
    TEST_ASSERT_EQUAL_STRING("SAE J1850 PWM\r" ELM_PROMPT, elm_ask("ATDP\r"));

    elm_ok("ATSP3\r");
    TEST_ASSERT_EQUAL_STRING("ISO 9141-2\r" ELM_PROMPT, elm_ask("ATDP\r"));
}

TEST(at_dp_marks_a_protocol_that_was_searched_for) {
    elm_echo_off();

    g_elm->settings.current_protocol = ELM327_PROTO_J1850_VPW;
    g_elm->settings.auto_search = true;

    TEST_ASSERT_EQUAL_STRING("AUTO, SAE J1850 VPW\r" ELM_PROMPT,
                             elm_ask("ATDP\r"));
}

TEST(at_ws_identifies_without_the_reset_delay) {
    elm_echo_off();
    elm_ok("ATH1\r");

    uint32_t before = fake_clock_ms();

    TEST_ASSERT_EQUAL_STRING("\r\r" ELM_ID "\r" ELM_PROMPT, elm_ask("ATWS\r"));
    TEST_ASSERT_MSG(fake_clock_ms() - before < 1000, "AT WS waited like AT Z");

    /* And it restores the same settings AT Z does. */
    TEST_ASSERT_FALSE(g_elm->settings.show_header);
}

TEST(at_pc_closes_the_protocol) {
    elm_echo_off();
    elm_ok("ATSP1\r");

    elm_ok("ATPC\r");

    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_J1850_PWM));
    TEST_ASSERT_EQUAL_STRING("0\r" ELM_PROMPT, elm_ask("ATDPN\r"));
}

TEST(at_r0_sends_the_request_and_does_not_wait) {
    static const uint8_t pwm_reply[10] = {0x41, 0x6B, 0x10, 0x41, 0x00,
                                          0xBE, 0x3F, 0xB8, 0x13, 0xC4};

    elm_echo_off();
    elm_ok("ATSP1\r");
    elm_ok("ATR0\r");
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, pwm_reply, sizeof(pwm_reply),
                            5);

    /* Responses off overrides the frame count digit, as the datasheet says. */
    TEST_ASSERT_EQUAL_STRING(ELM_PROMPT, elm_ask("01001\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sent_count(FAKE_BUS_J1850_PWM));
}

TEST(at_sh_rejects_a_width_no_protocol_uses) {
    static const char *const cmds[] = {
        "ATSH1\r", "ATSH12\r", "ATSH1234\r", "ATSH12345\r", "ATSH1234567\r",
    };

    elm_echo_off();

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        TEST_ASSERT_MSG(td_string_equal("?\r" ELM_PROMPT, elm_ask(cmds[i])),
                        "%s was accepted, got \"%s\"", cmds[i],
                        td_escape(fake_port_text(0), fake_port_len(0)));
    }
}

/* ------------------------------------------------------------------ *
 * One prompt per line
 *
 * An ELM327 conversation is strictly one prompt per line. A line that
 * produces no prompt leaves the client waiting for one that never comes; it
 * times out, sends the next command, and reads the previous reply against it
 * - and stays exactly one behind for the rest of the session. Every request
 * from then on is answered with the answer to the one before it, which is
 * indistinguishable from a flaky bus and is not recoverable without a reset.
 * ------------------------------------------------------------------ */

TEST(a_single_character_line_is_answered) {
    elm_echo_off();

    /* One character is not a command, but it is a line, and a line gets a
     * prompt. This used to be logged and dropped in silence. */
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask("1\r"));
}

TEST(an_overlong_line_is_answered_once) {
    char cmd[80];

    elm_echo_off();

    memset(cmd, 'A', sizeof(cmd) - 2);
    cmd[sizeof(cmd) - 2] = '\r';
    cmd[sizeof(cmd) - 1] = '\0';

    /* Exactly one prompt for one line. Resetting the buffer mid-line instead
     * would make the tail look like a command of its own and produce two. */
    TEST_ASSERT_EQUAL_STRING("?\r" ELM_PROMPT, elm_ask(cmd));
}

TEST(the_adapter_stays_in_step_after_a_line_it_could_not_use) {
    char overlong[80];

    elm_echo_off();

    memset(overlong, 'A', sizeof(overlong) - 2);
    overlong[sizeof(overlong) - 2] = '\r';
    overlong[sizeof(overlong) - 1] = '\0';

    elm_ask("1\r");
    elm_ask(overlong);

    /* The next command must be answered by its own reply, not by the one
     * before it. This is the whole point. */
    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r"));
    TEST_ASSERT_EQUAL_STRING("OK\r" ELM_PROMPT, elm_ask("ATE0\r"));
}

TEST(a_bare_carriage_return_repeats_the_last_command) {
    elm_echo_off();

    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("ATI\r"));
    TEST_ASSERT_EQUAL_STRING(ELM_ID "\r" ELM_PROMPT, elm_ask("\r"));
}

TEST(every_line_produces_exactly_one_prompt) {
    static const char *lines[] = {
        "ATI\r", "1\r", "\r", "ATE0\r", "XX\r", "ATZZTOP\r", "0\r", "ATRV\r",
    };
    const char *out;
    int prompts;

    elm_echo_off();

    for (size_t i = 0; i < sizeof(lines) / sizeof(lines[0]); i++) {
        out = elm_ask(lines[i]);
        prompts = 0;

        for (const char *p = out; *p; p++) {
            if (*p == '>') {
                prompts++;
            }
        }

        TEST_ASSERT_MSG(prompts == 1, "line %s produced %d prompts: %s",
                        td_escape(lines[i], __builtin_strlen(lines[i])),
                        prompts, td_escape(out, __builtin_strlen(out)));
    }
}

/* ------------------------------------------------------------------ *
 * One client's conversation does not leak into the next one's
 *
 * A BLE central drops and another connects seconds later, which for an OBD-II
 * app is not an edge case but the shape of every session it opens. What the
 * departed client left - a line it was halfway through sending, a command
 * nobody had read yet, the settings it chose - belongs to that client, and
 * every scrap of it that survives costs the next client a prompt.
 * ------------------------------------------------------------------ */

/** @brief A client's bytes arrive but nobody reads them; then it vanishes. */
static void elm_deliver_unread(const char *s) {
    TEST_ASSERT_MSG(
        comm_port_rx(fake_port_id(0), (const uint8_t *)s, strlen(s)) == ESP_OK,
        "transport refused %zu bytes", strlen(s));
}

TEST(a_half_sent_line_is_not_glued_to_the_next_clients_first_command) {
    elm_echo_off();

    /* Mid-line when the connection dropped. */
    elm_send("01");

    elm_link_down();

    /*
     * "ATZ" and nothing else. This used to come out as "010ATZ", one line the
     * adapter could only answer with "?" where the client had sent two - and
     * a client an answer short of its commands reads every later reply
     * against the wrong request for the rest of the session.
     *
     * Echo is back on because the reset restored the power-on settings, which
     * is the other half of the same guarantee: the next client gets an
     * adapter in the state it is entitled to assume.
     */
    fake_port_reset();
    TEST_ASSERT_EQUAL_STRING("ATZ\r\r\r" ELM_ID "\r" ELM_PROMPT,
                             elm_ask("ATZ\r"));
}

TEST(a_command_nobody_read_is_not_run_for_the_next_client) {
    elm_echo_off();

    /* Queued behind a request the adapter was still working on, and never
     * read, when the client went away. */
    elm_deliver_unread("ATI\r010");

    elm_link_down();

    fake_port_reset();
    elm_pump();

    /* Not one byte: an unasked-for reply is an extra prompt, and an extra
     * prompt puts the client out of step exactly as surely as a missing one.
     */
    TEST_ASSERT_EQUAL_STRING("", fake_port_text(0));

    TEST_ASSERT_EQUAL_STRING("ATZ\r\r\r" ELM_ID "\r" ELM_PROMPT,
                             elm_ask("ATZ\r"));
}

TEST(the_settings_a_client_chose_leave_with_it) {
    elm_echo_off();
    elm_ok("ATH1\r");

    /* Its own reply already carries the linefeeds it just asked for. */
    TEST_ASSERT_EQUAL_STRING("OK\r\n\r\n>", elm_ask("ATL1\r"));

    elm_link_down();

    /* Echo on, headers off, no linefeeds: what a client that sends nothing
     * but "0100" is entitled to find. */
    TEST_ASSERT_EQUAL_INT(1, (int)g_elm->settings.echo);
    TEST_ASSERT_EQUAL_INT(0, (int)g_elm->settings.show_header);
    TEST_ASSERT_EQUAL_INT(0, (int)g_elm->settings.linefeed);
}

TEST(a_kwp_session_ends_when_the_client_that_opened_it_leaves) {
    elm_echo_off();
    elm_kline_select("ATSP5\r");

    elm_link_down();

    /*
     * ISO 14230-2 clause 5.2. The ECU was put into a diagnostic session on
     * this client's behalf; the client has gone, so the session is over and
     * the ECU is told rather than left to time out at P3max.
     *
     * The bus claim deliberately outlives a client - a programming voltage
     * raised for a flash has to survive the tool reconnecting - and the
     * session used to ride along with it. That is what made a K-Line adapter
     * work exactly once: close the app, open it again, and the next client's
     * initialisation went to an ECU that was still mid-session with the last
     * one and would not answer it. A claim is a resource; a session is a
     * conversation, and only the second one belongs to the client.
     */
    TEST_ASSERT_EQUAL_INT(1, fake_bus_stop_comm_count());
}

TEST(a_client_leaving_a_protocol_with_no_session_says_nothing) {
    elm_echo_off();
    elm_ok("ATSP6\r");

    elm_link_down();

    /* CAN has no session to end, and neither has a K-Line protocol that was
     * selected but never initialised. */
    TEST_ASSERT_EQUAL_INT(0, fake_bus_stop_comm_count());
}

TEST(a_client_leaving_ends_the_session_and_lets_the_bus_go) {
    elm_echo_off();
    elm_kline_select("ATSP5\r");

    elm_link_down();

    /* Two things have to happen, and they are separate. The ECU is told the
     * session is over rather than left mid-session with no tester - that is
     * the StopCommunication above - and the driver is released, so the next
     * client, on this link or another one, can have the bus at all. */
    TEST_ASSERT_EQUAL_INT(1, fake_bus_stop_comm_count());
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE,
                          vif_bus_current(VIF_OWNER_LINK, VIF_BUS_KLINE));
}

/* ------------------------------------------------------------------ *
 * A reset loses what was in flight when it ran
 * ------------------------------------------------------------------ */

TEST(a_reset_swallows_the_rest_of_the_write_it_arrived_in) {
    /*
     * Car Scanner opens every connection with exactly this, in one write: a
     * reset, echo off, and a row of bare carriage returns meant to sweep the
     * line clean. On an ELM327 a bare carriage return repeats the last
     * command, so those eight would be eight more ATE0s - and eight replies
     * the client is not waiting for leave it eight behind for the rest of the
     * session, reading every answer against a command eight places later.
     *
     * A real chip loses the lot: the whole burst reaches it during the second
     * its reset takes, and the reset clears the buffer it was read into.
     */
    fake_port_reset();
    elm_send("ATZ\rATE0\r\r\r\r\r\r\r\r\r");

    TEST_ASSERT_EQUAL_STRING("ATZ\r\r\r" ELM_ID "\r" ELM_PROMPT,
                             fake_port_text(0));
}

TEST(a_reset_does_not_swallow_what_comes_after_it) {
    /* Only the write the reset was part of. The next one is a client that
     * waited for its prompt, and is owed an answer. */
    elm_send("ATZ\rATE0\r");
    fake_port_reset();

    elm_send("ATI\r");

    /* Echo is on: the reset put the settings back, and the ATE0 that would
     * have turned it off went with the rest of that write. */
    TEST_ASSERT_EQUAL_STRING("ATI\r" ELM_ID "\r" ELM_PROMPT, fake_port_text(0));
}

TEST(one_write_of_several_commands_is_still_answered_in_full) {
    /* Nothing here resets, so nothing is discarded: three lines, three
     * answers. The discard is a property of AT Z, not of batching. */
    elm_echo_off();
    fake_port_reset();

    elm_send("ATH1\rATS0\rATL0\r");

    TEST_ASSERT_EQUAL_STRING("OK\r" ELM_PROMPT "OK\r" ELM_PROMPT
                             "OK\r" ELM_PROMPT,
                             fake_port_text(0));
}
