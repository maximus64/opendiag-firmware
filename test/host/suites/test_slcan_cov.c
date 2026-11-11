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
 * The front-end is driven through its own vtable, the same calls the link
 * task makes, because the host stub never starts that task.
 */

#include <string.h>

#include "td_test.h"

#include "freertos/semphr.h"
#include "freertos/task.h"

#include "fake_can_bus.h"
#include "fake_clock.h"
#include "fake_led.h"
#include "fake_port.h"

#include "can_xfer.h"
#include "slcan.h"
#include "vif.h"

#define SLCAN_ACK "\r"
#define SLCAN_NACK "\a"

static comm_port_id_t g_port = COMM_INVALID_PORT_ID;

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

    /* The link task's loop body: hand over what arrived, then let the
     * front-end push whatever the bus gave it. */
    while ((n = comm_port_read(g_port, buf, sizeof(buf), 0)) > 0) {
        slcan_frontend.feed(buf, n);
    }
    slcan_frontend.poll();

    return fake_port_text(0);
}

/** Runs poll() alone, for traffic that arrives with no command to prompt it. */
static const char *slcan_listen(void) {
    fake_port_reset();
    slcan_frontend.poll();

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

    /* This thread is the link task, and the shell's too, so both owners can
     * reach the driver from here. */
    idf_stub_set_created_task(xTaskGetCurrentTaskHandle());
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_link_start(&slcan_frontend));
    vif_shell_bind();

    vif_link_set_port(g_port);

    /* The queued default is left queued: these tests drive the front-end's
     * own vtable rather than the task that would install it. */
    TEST_ASSERT_TRUE(slcan_frontend.start());
}

void td_teardown(void) {
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

TEST(open_is_refused_while_the_shell_holds_the_bus) {
    const vif_bus_cfg_t cfg = {.bitrate = 500000};

    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_bus_open(VIF_OWNER_SHELL, VIF_BUS_CAN, &cfg));

    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("O\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());

    /* And it works as soon as the shell lets go. */
    vif_bus_close(VIF_OWNER_SHELL, VIF_BUS_CAN);
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

TEST(a_whole_burst_costs_one_transport_write) {
    const uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    const char *out;
    int lines = 0;

    slcan_open();
    fake_port_reset();

    /* Twelve extended frames: 27 bytes each, 324 in all, inside the one
     * batch. A saturated bus delivers about this many per poll. */
    for (int i = 0; i < 12; i++) {
        fake_can_stage_stale((uint32_t)(0x18DA0000 + i) | CAN_EFF_FLAG,
                             sizeof(data), data);
    }

    out = slcan_listen();

    for (const char *p = out; *p; p++) {
        lines += (*p == '\r');
    }
    TEST_ASSERT_EQUAL_INT(12, lines);
    TEST_ASSERT_EQUAL_INT(12 * 27, (int)fake_port_len(0));

    /* The point of the change: one write and one flush for the burst, not one
     * of each per frame. Every comm_port_write() is a lock, a connection
     * check and a USB FIFO queue call. */
    TEST_ASSERT_EQUAL_INT(1, fake_port_write_count(0));
    TEST_ASSERT_EQUAL_INT(1, fake_port_flush_count(0));
}

TEST(a_burst_too_big_for_one_batch_is_split_rather_than_dropped) {
    const uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    /* 648 bytes of longest-line frames against a 512 byte batch: enough to
     * spill, and comfortably inside one poll's forwarding limit. */
    const int frames = 24;
    int lines = 0;

    slcan_open();
    fake_port_reset();

    for (int i = 0; i < frames; i++) {
        fake_can_stage_stale((uint32_t)(0x18DA0000 + i) | CAN_EFF_FLAG,
                             sizeof(data), data);
    }

    for (const char *p = slcan_listen(); *p; p++) {
        lines += (*p == '\r');
    }

    /* Split across two writes, and not one frame short. */
    TEST_ASSERT_EQUAL_INT(frames, lines);
    TEST_ASSERT_EQUAL_INT(frames * 27, (int)fake_port_len(0));
    TEST_ASSERT_EQUAL_INT(2, fake_port_write_count(0));
}

TEST(a_reply_lands_behind_the_frames_that_came_before_it) {
    const uint8_t data[1] = {0x01};
    uint8_t buf[64];
    size_t n;

    slcan_open();
    fake_port_reset();
    fake_can_stage_stale(0x100, sizeof(data), data);

    /* Forward what the bus had, then answer the command that arrived after
     * it - not through slcan_ask(), which clears the capture first. Frames and
     * replies share the one buffer, so the order they were composed in is the
     * order the client sees. */
    slcan_frontend.poll();

    TEST_ASSERT_EQUAL_INT(
        ESP_OK, comm_port_rx(fake_port_id(0), (const uint8_t *)"V\r", 2));
    while ((n = comm_port_read(g_port, buf, sizeof(buf), 0)) > 0) {
        slcan_frontend.feed(buf, n);
    }

    TEST_ASSERT_EQUAL_STRING("t100101\rV-082021\r", fake_port_text(0));
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

    /* Whatever closes the link's bus - another front-end, a shell command -
     * closes the channel, and SLCAN has to notice. */
    vif_bus_close(VIF_OWNER_LINK, VIF_BUS_CAN);

    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("t7DF1AA\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}

TEST(the_channel_survives_a_front_end_switch) {
    slcan_open();

    /* Restarting the front-end is what a protocol switch does to it. The
     * claim outlives the grammar, so the channel comes back up open. */
    slcan_frontend.stop();
    TEST_ASSERT_TRUE(slcan_frontend.start());

    TEST_ASSERT_EQUAL_INT(0, fake_can_teardown_count());
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("t7DF1AA\r"));
}

TEST(close_without_a_claim_leaves_the_other_owners_bus_running) {
    vif_bus_cfg_t cfg = {.bitrate = 500000};

    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_bus_open(VIF_OWNER_SHELL, VIF_BUS_CAN, &cfg));
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("C\r"));
    TEST_ASSERT_TRUE(vif_bus_is_open(VIF_OWNER_SHELL, VIF_BUS_CAN));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_close(VIF_OWNER_SHELL, VIF_BUS_CAN));
}

/* ------------------------------------------------------------------ *
 * Coverage: the other bit rate digits, bad input, and registration
 * ------------------------------------------------------------------ */

TEST(the_lawicel_digits_two_six_and_eight_select_their_rates) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("S2\r"));
    slcan_open();
    TEST_ASSERT_EQUAL_INT(50000, fake_can_last_baud());
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("C\r"));

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("S6\r"));
    slcan_open();
    TEST_ASSERT_EQUAL_INT(500000, fake_can_last_baud());
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("C\r"));

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("S8\r"));
    slcan_open();
    TEST_ASSERT_EQUAL_INT(1000000, fake_can_last_baud());
}

TEST(opening_an_open_channel_is_acknowledged_without_restarting_the_driver) {
    slcan_open();

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("O\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
    TEST_ASSERT_TRUE(fake_can_is_up());
}

/* ------------------------------------------------------------------ *
 * Listen-only: 'L'
 * ------------------------------------------------------------------ */

TEST(l_opens_the_channel_listen_only) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("L\r"));

    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(500000, fake_can_last_baud());
    TEST_ASSERT_TRUE(fake_can_is_up());
    TEST_ASSERT_TRUE(fake_can_last_listen_only());
    TEST_ASSERT_TRUE(vif_bus_is_open(VIF_OWNER_LINK, VIF_BUS_CAN));
}

TEST(o_still_opens_normally) {
    slcan_open();

    TEST_ASSERT_FALSE(fake_can_last_listen_only());
}

TEST(l_is_refused_while_the_shell_holds_the_bus) {
    const vif_bus_cfg_t cfg = {.bitrate = 500000};

    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_bus_open(VIF_OWNER_SHELL, VIF_BUS_CAN, &cfg));

    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("L\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());

    vif_bus_close(VIF_OWNER_SHELL, VIF_BUS_CAN);
}

TEST(l_on_a_driver_that_will_not_start_is_refused) {
    fake_can_fail_setup(true);

    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("L\r"));
    TEST_ASSERT_FALSE(vif_bus_is_open(VIF_OWNER_LINK, VIF_BUS_CAN));
}

TEST(l_again_is_acknowledged_without_reopening) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("L\r"));

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("L\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(0, fake_can_teardown_count());
}

TEST(l_on_a_normal_channel_is_refused_until_it_is_closed) {
    slcan_open();

    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("L\r"));

    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
    TEST_ASSERT_FALSE(fake_can_last_listen_only());

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("C\r"));
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("L\r"));

    TEST_ASSERT_EQUAL_INT(2, fake_can_setup_count());
    TEST_ASSERT_TRUE(fake_can_last_listen_only());
    TEST_ASSERT_TRUE(fake_can_is_up());
}

TEST(o_on_a_listen_only_channel_is_refused_until_it_is_closed) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("L\r"));

    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("O\r"));

    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());
    TEST_ASSERT_TRUE(fake_can_last_listen_only());

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("C\r"));
    slcan_open();

    TEST_ASSERT_EQUAL_INT(2, fake_can_setup_count());
    TEST_ASSERT_FALSE(fake_can_last_listen_only());
    TEST_ASSERT_TRUE(fake_can_is_up());

    /* And it transmits again. */
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("t7DF1AA\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
}

TEST(a_failed_reopen_leaves_the_channel_closed_and_sends_nothing) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("L\r"));
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("C\r"));

    fake_can_fail_setup(true);
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("O\r"));
    fake_can_fail_setup(false);

    /* vif takes the old driver down first, so the claim is gone. */
    TEST_ASSERT_FALSE(vif_bus_is_open(VIF_OWNER_LINK, VIF_BUS_CAN));
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("t7DF1AA\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}

TEST(listen_only_refuses_every_frame_command_and_sends_nothing) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("L\r"));

    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("t7DF3010203\r"));
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("T18DB33F12AABB\r"));
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("r1238\r"));
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("R18DB33F14\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}

TEST(listen_only_still_forwards_received_frames) {
    const uint8_t data[] = {0xAA, 0xBB};

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("L\r"));
    fake_can_stage_stale(0x7E8, 2, data);

    TEST_ASSERT_EQUAL_STRING("t7E82AABB\r", slcan_listen());
}

TEST(the_bit_rate_change_keeps_listen_only) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("L\r"));

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("S4\r"));

    TEST_ASSERT_EQUAL_INT(2, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(125000, fake_can_last_baud());
    TEST_ASSERT_TRUE(fake_can_last_listen_only());
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("t7DF1AA\r"));
}

TEST(the_bit_rate_change_keeps_a_normal_channel_normal) {
    slcan_open();

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("S4\r"));

    TEST_ASSERT_FALSE(fake_can_last_listen_only());
}

TEST(a_bit_rate_set_before_l_applies_to_the_listen_only_open) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("S5\r"));

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("L\r"));

    TEST_ASSERT_EQUAL_INT(250000, fake_can_last_baud());
    TEST_ASSERT_TRUE(fake_can_last_listen_only());
}

TEST(a_channel_closed_and_reopened_with_o_is_normal_again) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("L\r"));
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("C\r"));

    slcan_open();

    TEST_ASSERT_FALSE(fake_can_last_listen_only());
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("t7DF1AA\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
}

TEST(a_frame_command_on_a_closed_channel_is_refused_after_listen_only_too) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("L\r"));
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("C\r"));

    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("t7DF1AA\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}

TEST(a_frame_too_short_for_its_identifier_is_refused) {
    slcan_open();

    /* A standard identifier needs three digits and a length, an extended one
     * eight digits and a length. */
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("t7DF\r"));
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("T18DB33F1\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}

TEST(a_frame_with_a_non_hex_identifier_is_refused) {
    slcan_open();

    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("t7GG1AA\r"));
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("T18DB3ZF12AA\r"));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());
}

TEST(an_extended_remote_request_carries_no_data) {
    slcan_open();

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("R18DB33F14\r"));

    const struct can_frame *f = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(f);
    TEST_ASSERT_EQUAL_HEX32(0x18DB33F1 | CAN_EFF_FLAG | CAN_RTR_FLAG, f->id);
    TEST_ASSERT_EQUAL_INT(4, f->dlc);
}

static int g_other_thread;

TEST(a_close_from_another_thread_is_refused_and_the_channel_stays_open) {
    slcan_open();

    /* vif tears a driver down only on the thread that owns it. Running the
     * command as another thread makes the close refused, and the front-end
     * has to say so rather than report a channel it has not closed. */
    idf_stub_set_current_task((TaskHandle_t)&g_other_thread);
    TEST_ASSERT_EQUAL_STRING(SLCAN_NACK, slcan_ask("C\r"));
    idf_stub_set_current_task(NULL);

    TEST_ASSERT_EQUAL_INT(0, fake_can_teardown_count());
    TEST_ASSERT_TRUE(fake_can_is_up());
    TEST_ASSERT_TRUE(vif_bus_is_open(VIF_OWNER_LINK, VIF_BUS_CAN));

    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK, slcan_ask("C\r"));
    TEST_ASSERT_EQUAL_INT(1, fake_can_teardown_count());
}

TEST(registering_over_a_taken_name_is_refused_and_the_taken_name_stays) {
    /* The front-end table has no way out, so this is the only test that puts
     * anything under the SLCAN name. The squatter's name differs in case only,
     * which the table's name comparison ignores. */
    static const vif_frontend_t squatter = {.name = "SLCAN"};

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_frontend_register(&squatter));

    slcan_register();

    TEST_ASSERT_TRUE(vif_frontend_find("slcan") == &squatter);
}

TEST(an_overrun_is_reported_by_f_until_the_channel_reopens) {
    const uint8_t data[1] = {0x01};

    slcan_open();
    TEST_ASSERT_EQUAL_STRING("F00" SLCAN_ACK, slcan_ask("F\r"));

    fake_can_stage_stale_status(0x100, sizeof(data), data, BUS_RX_BUFFER_OVERFLOW);
    TEST_ASSERT_EQUAL_STRING("t100101\r", slcan_listen());
    TEST_ASSERT_EQUAL_STRING("F08" SLCAN_ACK, slcan_ask("F\r"));

    /* Sticky: a frame without the overflow mark does not clear it. */
    fake_can_stage_stale(0x101, sizeof(data), data);
    TEST_ASSERT_EQUAL_STRING("t101101\r", slcan_listen());
    TEST_ASSERT_EQUAL_STRING("F08" SLCAN_ACK, slcan_ask("F\r"));

    slcan_ask("C\r");
    slcan_open();
    TEST_ASSERT_EQUAL_STRING("F00" SLCAN_ACK, slcan_ask("F\r"));
}

TEST(a_crlf_line_end_does_not_break_the_next_command) {
    /* The LF after the CR is an empty line, not the start of the next command. */
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK "F00" SLCAN_ACK, slcan_ask("O\r\nF\r"));
}

TEST(a_bare_lf_also_ends_a_command) {
    TEST_ASSERT_EQUAL_STRING(SLCAN_ACK "F00" SLCAN_ACK, slcan_ask("O\nF\n"));
}

TEST(can_xfer_helpers_refuse_a_null_frame) {
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BAD_ARG, can_frame_send(NULL));
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BAD_ARG, can_frame_send_confirmed(NULL));
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BAD_ARG, can_frame_recv(NULL, 0));
}
