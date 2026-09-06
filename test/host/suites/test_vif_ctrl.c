/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_vif_ctrl.c
 * @brief The control plane's grammar.
 *
 * This is what both the shell's "mode" command and the BLE control
 * characteristic run, so the answers here are the ones a client actually sees.
 *
 * The front-ends are stand-ins rather than the real ELM327 and SLCAN: what is
 * under test is the command set and what it does to the link, not either
 * grammar. vif.c itself is real, so a switch really does go through the claim
 * rules - which means it lands on the link task's next pass, and this thread
 * stands in for that task.
 */

#include <string.h>

#include "td_test.h"

#include "freertos/semphr.h"
#include "freertos/task.h"

#include "fake_board.h"
#include "fake_bus.h"
#include "fake_can_bus.h"
#include "fake_clock.h"
#include "fake_led.h"
#include "fake_port.h"

#include "vif.h"
#include "vif_ctrl.h"

#define OBD_PIN_HS 6

static const vif_bus_cfg_t can500 = {.bitrate = 500000};

static const vif_frontend_t frontend_a = {.name = "elm327"};
static const vif_frontend_t frontend_b = {.name = "slcan"};

static char g_out[VIF_CTRL_REPLY_MAX];

/** Runs one control command and lets the link task apply what it asked for. */
static const char *ctrl(const char *cmd) {
    memset(g_out, 0, sizeof(g_out));
    vif_ctrl_exec(cmd, strlen(cmd), g_out, sizeof(g_out));
    vif_link_service();

    return g_out;
}

/** Runs one and reports whether it was understood. */
static bool ctrl_ok(const char *cmd) {
    bool ok;

    memset(g_out, 0, sizeof(g_out));
    ok = vif_ctrl_exec(cmd, strlen(cmd), g_out, sizeof(g_out));
    vif_link_service();

    return ok;
}

static void ports_init_once(void) {
    static bool done;

    if (done) {
        return;
    }
    done = true;

    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_iface_init());
    fake_port_register_all();
}

void td_setup(void) {
    /* A real port, because the control plane's answers go out on one. */
    ports_init_once();

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_init());

    fake_clock_reset();
    fake_board_reset();
    fake_led_reset();
    fake_can_reset();
    fake_bus_reset_all();

    vif_frontend_register(&frontend_a);
    vif_frontend_register(&frontend_b);

    /* This thread is the link task, and the shell's, so a claim taken on
     * either owner's behalf reaches the driver. */
    idf_stub_set_created_task(xTaskGetCurrentTaskHandle());
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_link_start(&frontend_a));
    vif_shell_bind();

    vif_link_set_port(fake_port_id(0));
    vif_link_service();
}

void td_teardown(void) {
    TEST_ASSERT_MSG(idf_stub_lock_balance() == 0, "left %d locks held",
                    idf_stub_lock_balance());
}

/* ------------------------------------------------------------------ *
 * Identity
 * ------------------------------------------------------------------ */

TEST(id_reports_the_link_its_mode_and_every_mode) {
    const char *out = ctrl("ID");

    /* This is the whole discovery story: an OpenDiag-aware client asks once
     * and configures itself; anything else never sends a control command and
     * sees a plain ELM327. */
    TEST_ASSERT_MSG(strstr(out, "link=link") != NULL, "no link name in '%s'",
                    out);
    TEST_ASSERT_MSG(strstr(out, "mode=elm327") != NULL, "no mode in '%s'", out);
    TEST_ASSERT_MSG(strstr(out, "modes=elm327,slcan") != NULL,
                    "no mode list in '%s'", out);
    TEST_ASSERT_MSG(strstr(out, "fw=host-test ") != NULL,
                    "wrong firmware version in '%s'", out);
}

TEST(an_answer_never_overruns_the_buffer_it_was_given) {
    char small[16];

    memset(small, 'x', sizeof(small));
    vif_ctrl_exec("ID", 2, small, sizeof(small));

    TEST_ASSERT_EQUAL_INT(0, small[sizeof(small) - 1]);
}

/* ------------------------------------------------------------------ *
 * Mode
 * ------------------------------------------------------------------ */

TEST(mode_reports_the_grammar_running_now) {
    TEST_ASSERT_EQUAL_STRING("MODE elm327", ctrl("MODE"));
}

TEST(mode_switches_the_link) {
    TEST_ASSERT_TRUE(ctrl_ok("MODE=slcan"));
    TEST_ASSERT_EQUAL_STRING("slcan", vif_link_frontend_name());
    TEST_ASSERT_EQUAL_STRING("MODE slcan", ctrl("MODE"));
}

TEST(a_command_is_case_insensitive) {
    /* A phone app writes what it likes, and the ELM327 grammar would have
     * upper-cased it anyway. */
    TEST_ASSERT_TRUE(ctrl_ok("mode=SLCAN"));
    TEST_ASSERT_EQUAL_STRING("slcan", vif_link_frontend_name());
}

TEST(a_terminated_command_is_accepted) {
    /* A terminal sends CR, a phone app sends nothing. Both are one command. */
    TEST_ASSERT_TRUE(ctrl_ok("MODE=slcan\r\n"));
    TEST_ASSERT_EQUAL_STRING("slcan", vif_link_frontend_name());
}

TEST(an_unknown_mode_is_refused_and_says_what_there_is) {
    TEST_ASSERT_FALSE(ctrl_ok("MODE=j1939"));

    TEST_ASSERT_MSG(strstr(g_out, "elm327,slcan") != NULL,
                    "the refusal did not list the modes: '%s'", g_out);

    /* Refused means unchanged, not left with no grammar at all. */
    TEST_ASSERT_EQUAL_STRING("elm327", vif_link_frontend_name());
}

TEST(switching_releases_every_claim) {
    vif_pin_set(VIF_OWNER_LINK, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);
    vif_bus_open(VIF_OWNER_LINK, VIF_BUS_CAN, &can500);
    vif_bus_open(VIF_OWNER_LINK, VIF_BUS_KLINE, NULL);

    TEST_ASSERT_TRUE(ctrl_ok("MODE=slcan"));

    /* A client asking for another grammar is starting again, so it starts
     * with the hardware it would have had on connecting: none of it. */
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_state(OBD_PIN_HS));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE,
                          vif_bus_current(VIF_OWNER_LINK, VIF_BUS_CAN));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE,
                          vif_bus_current(VIF_OWNER_LINK, VIF_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(1, fake_can_teardown_count());
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_KLINE));
}

/* ------------------------------------------------------------------ *
 * Reset
 * ------------------------------------------------------------------ */

TEST(reset_releases_the_claims_and_restores_the_default) {
    vif_pin_set(VIF_OWNER_LINK, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);
    vif_bus_open(VIF_OWNER_LINK, VIF_BUS_CAN, &can500);
    ctrl_ok("MODE=slcan");

    TEST_ASSERT_EQUAL_STRING("RESET elm327", ctrl("RESET"));

    /* Unlike a client merely going away, this is an explicit request to let
     * go of the hardware as well as the grammar. */
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_state(OBD_PIN_HS));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE,
                          vif_bus_current(VIF_OWNER_LINK, VIF_BUS_CAN));
    TEST_ASSERT_FALSE(fake_can_is_up());
    TEST_ASSERT_EQUAL_STRING("elm327", vif_link_frontend_name());
}

TEST(reset_leaves_the_shells_claims_alone) {
    vif_bus_open(VIF_OWNER_SHELL, VIF_BUS_CAN, &can500);

    ctrl_ok("RESET");

    /* The control plane is the link's, and only the link's: a phone must not
     * be able to pull a bus out from under a bring-up session on the console.
     */
    TEST_ASSERT_EQUAL_INT(VIF_BUS_CAN,
                          vif_bus_current(VIF_OWNER_SHELL, VIF_BUS_CAN));
    TEST_ASSERT_TRUE(fake_can_is_up());
}

/* ------------------------------------------------------------------ *
 * Malformed input
 * ------------------------------------------------------------------ */

TEST(an_unknown_command_is_refused) {
    TEST_ASSERT_FALSE(ctrl_ok("WHAT"));
    TEST_ASSERT_MSG(strstr(g_out, "ERR") != NULL, "no error in '%s'", g_out);
}

TEST(an_empty_command_is_refused) {
    TEST_ASSERT_FALSE(ctrl_ok(""));
    TEST_ASSERT_FALSE(ctrl_ok("   \r\n"));
}

TEST(a_command_longer_than_the_parser_takes_is_refused_not_overrun) {
    char big[256];

    memset(big, 'A', sizeof(big) - 1);
    big[sizeof(big) - 1] = '\0';

    TEST_ASSERT_FALSE(ctrl_ok(big));
}

/* ------------------------------------------------------------------ *
 * Reporting what is live
 * ------------------------------------------------------------------ */

TEST(bus_reports_every_live_bus_and_its_holder) {
    TEST_ASSERT_EQUAL_STRING("BUS", ctrl("BUS"));

    vif_bus_open(VIF_OWNER_LINK, VIF_BUS_CAN, &can500);
    vif_bus_open(VIF_OWNER_LINK, VIF_BUS_KLINE, NULL);

    /* A phone has no shell to ask, so this is the only way a client that was
     * refused a bus finds out who has it. */
    TEST_ASSERT_MSG(strstr(ctrl("BUS"), "CAN@500000/link") != NULL,
                    "no CAN claim in '%s'", g_out);
    TEST_ASSERT_MSG(strstr(g_out, "K-Line/link") != NULL,
                    "no K-Line claim in '%s'", g_out);
}

TEST(bus_names_the_shell_when_the_shell_is_the_holder) {
    vif_bus_open(VIF_OWNER_SHELL, VIF_BUS_KLINE, NULL);

    TEST_ASSERT_MSG(strstr(ctrl("BUS"), "K-Line/shell") != NULL,
                    "the shell was not named in '%s'", g_out);
}

/* ------------------------------------------------------------------ *
 * Cleanup that could not finish
 * ------------------------------------------------------------------ */

TEST(reset_reports_failed_cleanup_and_preserves_claim_for_retry) {
    TEST_ASSERT_EQUAL_INT(
        ESP_OK, vif_bus_open(VIF_OWNER_LINK, VIF_BUS_J1850_PWM, NULL));
    fake_bus_close_result(FAKE_BUS_J1850_PWM, ESP_ERR_TIMEOUT);

    TEST_ASSERT_FALSE(ctrl_ok("RESET"));
    TEST_ASSERT_EQUAL_STRING("ERR bus close failed: ESP_ERR_TIMEOUT", g_out);
    TEST_ASSERT_EQUAL_INT(VIF_BUS_J1850_PWM,
                          vif_bus_current(VIF_OWNER_LINK, VIF_BUS_J1850_PWM));

    fake_bus_close_result(FAKE_BUS_J1850_PWM, ESP_OK);
    TEST_ASSERT_TRUE(ctrl_ok("RESET"));
    TEST_ASSERT_EQUAL_STRING("RESET elm327", g_out);
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE,
                          vif_bus_current(VIF_OWNER_LINK, VIF_BUS_J1850_PWM));
}
