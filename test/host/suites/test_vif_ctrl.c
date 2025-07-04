/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_vif_ctrl.c
 * @brief The control plane's grammar.
 *
 * This is what both the shell's "mode" command and the BLE control
 * characteristic run, so the answers here are the ones a client actually sees.
 *
 * The front-ends are stand-ins rather than the real ELM327 and SLCAN: what is
 * under test is the command set and what it does to a session, not either
 * grammar. vif.c itself is real, so a switch really does go through the claim
 * rules.
 */

#include <string.h>

#include "td_test.h"

#include "freertos/semphr.h"

#include "fake_board.h"
#include "fake_bytebus.h"
#include "fake_can_bus.h"
#include "fake_clock.h"
#include "fake_led.h"
#include "fake_port.h"

#include "vif_ctrl.h"

#define OBD_PIN_HS 6

static const vif_bus_cfg_t can500 = { .bitrate = 500000 };

static int a_ctx, b_ctx;

static void *fe_a_create(vif_session_t *s, comm_port_id_t port) { return &a_ctx; }
static void *fe_b_create(vif_session_t *s, comm_port_id_t port) { return &b_ctx; }

static const vif_frontend_t frontend_a = { .name = "elm327", .create = fe_a_create };
static const vif_frontend_t frontend_b = { .name = "slcan",  .create = fe_b_create };

static vif_session_t *g_link;
static char g_out[VIF_CTRL_REPLY_MAX];

/** Runs one control command against the link and returns the answer. */
static const char *ctrl(const char *cmd)
{
    memset(g_out, 0, sizeof(g_out));
    vif_ctrl_exec(g_link, cmd, strlen(cmd), g_out, sizeof(g_out));

    return g_out;
}

/** Runs one and asserts it was understood. */
static bool ctrl_ok(const char *cmd)
{
    memset(g_out, 0, sizeof(g_out));
    return vif_ctrl_exec(g_link, cmd, strlen(cmd), g_out, sizeof(g_out));
}

static void ports_init_once(void)
{
    static bool done;

    if (done) {
        return;
    }
    done = true;

    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_iface_init());
    fake_port_register_all();
}

void td_setup(void)
{
    /* A real port, because a session without one is not a link and the
     * control plane refuses to put a grammar on it. */
    ports_init_once();

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_init());

    fake_clock_reset();
    fake_board_reset();
    fake_led_reset();
    fake_can_reset();
    fake_bytebus_reset_all();

    vif_frontend_register(&frontend_a);
    vif_frontend_register(&frontend_b);

    g_link = vif_session_open("usb", fake_port_id(0));
    TEST_ASSERT_NOT_NULL(g_link);

    vif_session_set_default_frontend(g_link, &frontend_a);
    vif_session_set_frontend(g_link, &frontend_a);
}

void td_teardown(void)
{
    TEST_ASSERT_MSG(idf_stub_lock_balance() == 0,
                    "left %d locks held", idf_stub_lock_balance());
}

/* ------------------------------------------------------------------ *
 * Identity
 * ------------------------------------------------------------------ */

TEST(id_reports_the_link_its_mode_and_every_mode)
{
    const char *out = ctrl("ID");

    /* This is the whole discovery story: an OpenDiag-aware client asks once
     * and configures itself; anything else never sends a control command and
     * sees a plain ELM327. */
    TEST_ASSERT_MSG(strstr(out, "link=usb") != NULL, "no link name in '%s'", out);
    TEST_ASSERT_MSG(strstr(out, "mode=elm327") != NULL, "no mode in '%s'", out);
    TEST_ASSERT_MSG(strstr(out, "modes=elm327,slcan") != NULL,
                    "no mode list in '%s'", out);
    TEST_ASSERT_MSG(strstr(out, "fw=") != NULL, "no version in '%s'", out);
}

TEST(an_answer_never_overruns_the_buffer_it_was_given)
{
    char small[16];

    memset(small, 'x', sizeof(small));
    vif_ctrl_exec(g_link, "ID", 2, small, sizeof(small));

    TEST_ASSERT_EQUAL_INT(0, small[sizeof(small) - 1]);
}

/* ------------------------------------------------------------------ *
 * Mode
 * ------------------------------------------------------------------ */

TEST(mode_reports_the_grammar_running_now)
{
    TEST_ASSERT_EQUAL_STRING("MODE elm327", ctrl("MODE"));
}

TEST(mode_switches_the_link)
{
    TEST_ASSERT_TRUE(ctrl_ok("MODE=slcan"));
    TEST_ASSERT_EQUAL_STRING("slcan", vif_session_frontend_name(g_link));
    TEST_ASSERT_EQUAL_STRING("MODE slcan", ctrl("MODE"));
}

TEST(a_command_is_case_insensitive)
{
    /* A phone app writes what it likes, and the ELM327 grammar would have
     * upper-cased it anyway. */
    TEST_ASSERT_TRUE(ctrl_ok("mode=SLCAN"));
    TEST_ASSERT_EQUAL_STRING("slcan", vif_session_frontend_name(g_link));
}

TEST(a_terminated_command_is_accepted)
{
    /* A terminal sends CR, a phone app sends nothing. Both are one command. */
    TEST_ASSERT_TRUE(ctrl_ok("MODE=slcan\r\n"));
    TEST_ASSERT_EQUAL_STRING("slcan", vif_session_frontend_name(g_link));
}

TEST(an_unknown_mode_is_refused_and_says_what_there_is)
{
    TEST_ASSERT_FALSE(ctrl_ok("MODE=j1939"));

    TEST_ASSERT_MSG(strstr(g_out, "elm327,slcan") != NULL,
                    "the refusal did not list the modes: '%s'", g_out);

    /* Refused means unchanged, not left with no grammar at all. */
    TEST_ASSERT_EQUAL_STRING("elm327", vif_session_frontend_name(g_link));
}

TEST(switching_keeps_the_claims)
{
    vif_pin_set(g_link, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);
    vif_bus_open(g_link, VIF_BUS_CAN, &can500);

    TEST_ASSERT_TRUE(ctrl_ok("MODE=slcan"));

    /* Raise a programming voltage under ELM327, flash under SLCAN. This is
     * the reason a switch happens inside the session. */
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(OBD_PIN_HS));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_CAN, vif_bus_current(g_link));
    TEST_ASSERT_EQUAL_INT(0, fake_can_teardown_count());
}

/* ------------------------------------------------------------------ *
 * Bus
 * ------------------------------------------------------------------ */

TEST(bus_says_nothing_is_live_when_nothing_is)
{
    TEST_ASSERT_EQUAL_STRING("BUS none", ctrl("BUS"));
}

TEST(bus_names_the_holder_so_a_refusal_can_be_explained)
{
    vif_session_t *other = vif_session_open("ble", fake_port_id(1));

    TEST_ASSERT_NOT_NULL(other);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(other, VIF_BUS_CAN, &can500));

    /* One bus, one holder. Before this the loser of that race got a bare '?'
     * and no way to find out why. */
    TEST_ASSERT_EQUAL_STRING("BUS CAN 500000 held=ble", ctrl("BUS"));

    vif_session_close(other);
}

/* ------------------------------------------------------------------ *
 * Reset
 * ------------------------------------------------------------------ */

TEST(reset_releases_the_claims_and_restores_the_default)
{
    vif_pin_set(g_link, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);
    vif_bus_open(g_link, VIF_BUS_CAN, &can500);
    ctrl_ok("MODE=slcan");

    TEST_ASSERT_EQUAL_STRING("RESET elm327", ctrl("RESET"));

    /* Unlike a client merely going away, this is an explicit request to let
     * go of the hardware as well as the grammar. */
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_state(OBD_PIN_HS));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE, vif_bus_current(g_link));
    TEST_ASSERT_FALSE(fake_can_is_up());
    TEST_ASSERT_EQUAL_STRING("elm327", vif_session_frontend_name(g_link));
}

TEST(reset_leaves_another_links_claims_alone)
{
    vif_session_t *other = vif_session_open("ble", fake_port_id(1));

    TEST_ASSERT_NOT_NULL(other);
    vif_bus_open(other, VIF_BUS_CAN, &can500);

    ctrl_ok("RESET");

    TEST_ASSERT_EQUAL_INT(VIF_BUS_CAN, vif_bus_current(other));
    TEST_ASSERT_TRUE(fake_can_is_up());

    vif_session_close(other);
}

/* ------------------------------------------------------------------ *
 * Malformed input
 * ------------------------------------------------------------------ */

TEST(an_unknown_command_is_refused)
{
    TEST_ASSERT_FALSE(ctrl_ok("WHAT"));
    TEST_ASSERT_MSG(strstr(g_out, "ERR") != NULL, "no error in '%s'", g_out);
}

TEST(an_empty_command_is_refused)
{
    TEST_ASSERT_FALSE(ctrl_ok(""));
    TEST_ASSERT_FALSE(ctrl_ok("   \r\n"));
}

TEST(a_command_longer_than_the_parser_takes_is_refused_not_overrun)
{
    char big[256];

    memset(big, 'A', sizeof(big) - 1);
    big[sizeof(big) - 1] = '\0';

    TEST_ASSERT_FALSE(ctrl_ok(big));
}

TEST(a_command_against_no_link_is_answered_rather_than_crashing)
{
    char out[VIF_CTRL_REPLY_MAX];

    TEST_ASSERT_FALSE(vif_ctrl_exec(NULL, "ID", 2, out, sizeof(out)));
    TEST_ASSERT_MSG(strstr(out, "ERR") != NULL, "no error in '%s'", out);
}

/* ------------------------------------------------------------------ *
 * Sessions that are not links
 * ------------------------------------------------------------------ */

TEST(a_session_with_no_transport_has_no_mode)
{
    char out[VIF_CTRL_REPLY_MAX];
    vif_session_t *shell = vif_session_open("shell", COMM_INVALID_PORT_ID);

    TEST_ASSERT_NOT_NULL(shell);
    TEST_ASSERT_FALSE(vif_session_is_link(shell));

    /* The debug shell holds claims so its hardware commands are arbitrated,
     * but it parses no protocol. A grammar here would never be fed or polled
     * and would hold a pool slot for nothing. */
    TEST_ASSERT_FALSE(vif_ctrl_exec(shell, "MODE", 4, out, sizeof(out)));
    TEST_ASSERT_MSG(strstr(out, "shell") != NULL, "unhelpful refusal: '%s'", out);

    TEST_ASSERT_FALSE(vif_ctrl_exec(shell, "MODE=slcan", 10, out, sizeof(out)));
    TEST_ASSERT_NULL(vif_session_frontend_name(shell));

    vif_session_close(shell);
}

TEST(a_session_with_no_transport_can_still_be_reset)
{
    char out[VIF_CTRL_REPLY_MAX];
    vif_session_t *shell = vif_session_open("shell", COMM_INVALID_PORT_ID);

    TEST_ASSERT_NOT_NULL(shell);
    vif_pin_set(shell, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);

    /* Releasing what the console left energised is exactly what it is for. */
    TEST_ASSERT_TRUE(vif_ctrl_exec(shell, "RESET", 5, out, sizeof(out)));
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_state(OBD_PIN_HS));

    vif_session_close(shell);
}
