/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_slcan.c
 * @brief The SLCAN front-end: the LAWICEL grammar, and the claims behind it.
 *
 * SLCAN had no tests at all while it was unreachable at runtime. It is the
 * transport ECU flashing will ride on, so the exchanges a host tool depends on
 * - the bare CR acknowledgement, the BELL refusal, a channel that is really
 * open on the CAN driver - are pinned here byte for byte.
 *
 * The front-end is driven through its own vtable, the same calls the vif
 * session task makes, because the host stub never starts that task.
 */

#include <string.h>

#include "td_test.h"

#include "freertos/semphr.h"

#include "fake_can_bus.h"
#include "fake_clock.h"
#include "fake_led.h"
#include "fake_port.h"

#include "slcan.h"
#include "vif.h"

#define SLCAN_ACK "\r"
#define SLCAN_NACK "\a"

static comm_port_id_t g_port = COMM_INVALID_PORT_ID;
static vif_session_t *g_session;
static void *g_ctx;

/* ------------------------------------------------------------------ *
 * Driving the front-end
 * ------------------------------------------------------------------ */

/** Delivers @p s the way the transport would, and returns what came back. */
static const char *slcan_ask(const char *cmd) {
    uint8_t buf[64];
    size_t n;

    fake_port_reset();

    TEST_ASSERT_EQUAL_INT(
        ESP_OK,
        comm_port_rx(fake_port_id(0), (const uint8_t *)cmd, strlen(cmd)));

    /* The session task's loop body: hand over what arrived, then let the
     * front-end push whatever the bus gave it. */
    while ((n = comm_port_read(g_port, buf, sizeof(buf), 0)) > 0) {
        slcan_frontend.feed(g_ctx, buf, n);
    }
    slcan_frontend.poll(g_ctx);

    return fake_port_text(0);
}

/** Runs poll() alone, for traffic that arrives with no command to prompt it. */
static const char *slcan_listen(void) {
    fake_port_reset();
    slcan_frontend.poll(g_ctx);

    return fake_port_text(0);
}

/** Opens the channel, which every frame test needs first. */
static void slcan_open(void) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("O\r"));
}

/* ------------------------------------------------------------------ *
 * Fixture
 * ------------------------------------------------------------------ */

static void slcan_harness_init_once(void) {
    static bool done;

    if (done) {
        return;
    }
    done = true;

    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_iface_init());

    fake_port_register_all();
    g_port = fake_port_id(0);
    TEST_ASSERT_MSG(g_port != COMM_INVALID_PORT_ID, "no SLCAN port");
}

void td_setup(void) {
    uint8_t drain[64];

    /* comm_iface has no teardown by design, so the port is set up once; vif is
     * returned to its power-on state for every test. */
    slcan_harness_init_once();

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_init());

    fake_clock_reset();
    fake_led_reset();
    fake_can_reset();

    while (comm_port_read(g_port, drain, sizeof(drain), 0) > 0) {
        ;
    }
    fake_port_reset();

    g_session = vif_session_open("slcan", VIF_SESSION_LINK);
    TEST_ASSERT_NOT_NULL(g_session);
    vif_session_set_port(g_session, g_port);

    g_ctx = slcan_frontend.create(g_session);
    TEST_ASSERT_NOT_NULL(g_ctx);
}

void td_teardown(void) {
    slcan_frontend.destroy(g_ctx);
    g_ctx = NULL;

    TEST_ASSERT_MSG(idf_stub_lock_balance() == 0, "vif left %d locks held",
                    idf_stub_lock_balance());
}

/* ------------------------------------------------------------------ *
 * Opening and closing the channel
 * ------------------------------------------------------------------ */

TEST(open_starts_the_can_driver_at_the_selected_bit_rate) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("S5\r"));

    slcan_open();

    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(250000, fake_can_last_baud());
    TEST_ASSERT_TRUE(fake_can_is_up());
}

TEST(the_default_bit_rate_is_five_hundred_kilobit) {
    slcan_open();

    TEST_ASSERT_EQUAL_INT(500000, fake_can_last_baud());
}

TEST(a_bit_rate_the_driver_cannot_produce_is_refused) {
    /* S7 is 800 kbit/s, which the TWAI timing tables here do not carry.
     * Accepting it would run a vehicle bus at the wrong speed. */
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("S7\r"));

    slcan_open();
    TEST_ASSERT_EQUAL_INT(500000, fake_can_last_baud());
}

TEST(the_bit_rate_reconfigures_a_channel_this_session_holds) {
    /*
     * Not merely allowed - required. Switching to this front-end from ELM327
     * inherits a CAN claim already open at whatever protocol number that
     * grammar picked, so slcand's opening S<n> is the only chance to correct
     * the rate. The driver is restarted, and the claim never leaves us.
     */
    slcan_open();

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("S4\r"));

    TEST_ASSERT_EQUAL_INT(125000, fake_can_last_baud());
    TEST_ASSERT_EQUAL_INT(2, fake_can_setup_count());
    TEST_ASSERT_TRUE(fake_can_is_up());
}

TEST(a_reconfigure_the_driver_refuses_is_reported) {
    slcan_open();

    fake_can_fail_setup(true);
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("S4\r"));
}

TEST(close_stops_the_driver) {
    slcan_open();

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("C\r"));

    TEST_ASSERT_EQUAL_INT(1, fake_can_teardown_count());
    TEST_ASSERT_FALSE(fake_can_is_up());
}

TEST(open_is_refused_while_another_session_holds_the_bus) {
    vif_session_t *other = vif_session_open("elm", VIF_SESSION_LOCAL);
    const vif_bus_cfg_t cfg = {.bitrate = 500000};

    TEST_ASSERT_NOT_NULL(other);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(other, VIF_BUS_CAN, &cfg));

    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("O\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());

    /* And it works as soon as the other client lets go. */
    vif_bus_close(other, VIF_BUS_CAN);
    slcan_open();
}

/* ------------------------------------------------------------------ *
 * Transmitting
 * ------------------------------------------------------------------ */

TEST(a_standard_frame_is_transmitted) {
    slcan_open();

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("t7DF3010203\r"));

    const struct can_frame *f = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(f);
    TEST_ASSERT_EQUAL_HEX32(0x7DF, f->id);
    TEST_ASSERT_EQUAL_INT(3, f->dlc);
    TEST_ASSERT_EQUAL_MEM("\x01\x02\x03", f->data, 3);
}

TEST(an_extended_frame_is_transmitted) {
    slcan_open();

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("T18DB33F12AABB\r"));

    const struct can_frame *f = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(f);
    TEST_ASSERT_EQUAL_HEX32(0x18DB33F1 | CAN_EFF_FLAG, f->id);
    TEST_ASSERT_EQUAL_INT(2, f->dlc);
    TEST_ASSERT_EQUAL_MEM("\xAA\xBB", f->data, 2);
}

TEST(a_remote_request_carries_no_data) {
    slcan_open();

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("r1238\r"));

    const struct can_frame *f = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(f);
    TEST_ASSERT_EQUAL_HEX32(0x123 | CAN_RTR_FLAG, f->id);
    TEST_ASSERT_EQUAL_INT(8, f->dlc);
}

TEST(a_frame_sent_on_a_closed_channel_is_refused) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("t7DF3010203\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}

TEST(a_malformed_frame_is_refused) {
    slcan_open();

    /* Too short for its length byte, a bad hex digit, and a length of 9. */
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("t7DF3\r"));
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("t7DF1ZZ\r"));
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK,
                             slcan_ask("t7DF9010203040506070809\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}

TEST(a_failed_transmit_is_reported) {
    slcan_open();
    fake_can_fail_next_send();

    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("t7DF1AA\r"));
}

/* ------------------------------------------------------------------ *
 * Receiving
 * ------------------------------------------------------------------ */

TEST(a_received_standard_frame_is_forwarded) {
    const uint8_t data[3] = {0x11, 0x22, 0x33};

    slcan_open();
    fake_can_stage_stale(0x123, sizeof(data), data);

    TEST_ASSERT_EQUAL_STRING("t1233112233\r", slcan_listen());
}

TEST(a_received_extended_frame_is_forwarded) {
    const uint8_t data[1] = {0xAB};

    slcan_open();
    fake_can_stage_stale(0x18DAF110 | CAN_EFF_FLAG, sizeof(data), data);

    TEST_ASSERT_EQUAL_STRING("T18DAF1101AB\r", slcan_listen());
}

TEST(several_frames_are_forwarded_in_one_pass) {
    const uint8_t data[1] = {0x01};

    slcan_open();
    fake_can_stage_stale(0x100, sizeof(data), data);
    fake_can_stage_stale(0x101, sizeof(data), data);

    TEST_ASSERT_EQUAL_STRING("t100101\rt101101\r", slcan_listen());
}

TEST(nothing_is_forwarded_while_the_channel_is_closed) {
    const uint8_t data[1] = {0x01};

    fake_can_stage_stale(0x100, sizeof(data), data);

    TEST_ASSERT_EQUAL_STRING("", slcan_listen());
}

/* ------------------------------------------------------------------ *
 * The rest of the grammar
 * ------------------------------------------------------------------ */

TEST(status_version_and_serial_are_answered) {
    TEST_ASSERT_EQUAL_STRING("F00" SLCAN_ACK, slcan_ask("F\r"));
    TEST_ASSERT_EQUAL_STRING("V-082021" SLCAN_ACK, slcan_ask("V\r"));
    TEST_ASSERT_EQUAL_STRING("N2208" SLCAN_ACK, slcan_ask("N\r"));
}

TEST(the_acceptance_filter_commands_are_accepted_and_ignored) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("M00000000\r"));
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("mFFFFFFFF\r"));
}

TEST(an_unknown_command_is_refused) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("X\r"));
}

TEST(an_empty_line_says_nothing) {
    TEST_ASSERT_EQUAL_STRING("", slcan_ask("\r"));
}

TEST(an_over_long_command_is_refused_once) {
    char line[80];

    memset(line, 't', sizeof(line));
    line[sizeof(line) - 2] = '\r';
    line[sizeof(line) - 1] = '\0';

    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask(line));

    /* And the line after it is understood normally. */
    TEST_ASSERT_EQUAL_STRING("V-082021" SLCAN_ACK, slcan_ask("V\r"));
}

TEST(a_command_split_across_two_reads_is_still_understood) {
    slcan_open();

    TEST_ASSERT_EQUAL_STRING("", slcan_ask("t7DF3"));
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("010203\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
}

TEST(the_channel_follows_the_claim_rather_than_a_cached_flag) {
    slcan_open();

    /* Whatever closes the session's bus - another front-end, a shell command -
     * closes the channel, and SLCAN has to notice. */
    vif_bus_close(g_session, VIF_BUS_CAN);

    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("t7DF1AA\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}

TEST(the_channel_survives_a_front_end_switch) {
    slcan_open();

    /* Reinstalling the front-end is what a protocol switch does to it. The
     * session keeps the CAN claim, so the channel comes back up open. */
    slcan_frontend.destroy(g_ctx);
    g_ctx = slcan_frontend.create(g_session);

    TEST_ASSERT_NOT_NULL(g_ctx);
    TEST_ASSERT_EQUAL_INT(0, fake_can_teardown_count());
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("t7DF1AA\r"));
}

TEST(close_without_a_claim_leaves_another_owners_bus_running) {
    vif_session_t *other = vif_session_open("elm", VIF_SESSION_LOCAL);
    vif_bus_cfg_t cfg = {.bitrate = 500000};
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(other, VIF_BUS_CAN, &cfg));
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("C\r"));
    TEST_ASSERT_TRUE(vif_bus_is_open(other, VIF_BUS_CAN));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_close(other, VIF_BUS_CAN));
}
