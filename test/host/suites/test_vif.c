/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_vif.c
 * @brief The vehicle interface: sessions, claims and arbitration.
 *
 * These tests drive vif.c through its public API with the board and the four
 * bus drivers faked underneath, which is where the rules the whole design
 * rests on actually live: one bus at a time, one high side and one low side
 * pin, and a claim that belongs to the session that made it.
 *
 * Sessions here are opened without a comm_iface service. That is not a
 * shortcut - the debug shell runs exactly like this - and it keeps the session
 * task out of the way, since a task the host stub never starts could not pump
 * anything anyway.
 */

#include <string.h>

#include "td_test.h"

#include "freertos/semphr.h"

#include "fake_board.h"
#include "fake_bytebus.h"
#include "fake_can_bus.h"
#include "fake_clock.h"
#include "fake_led.h"

#include "vif.h"

#define OBD_PIN_HS 6
#define OBD_PIN_HS_OTHER 9
#define OBD_PIN_LS 15

static const vif_bus_cfg_t can500 = { .bitrate = 500000 };

/* ------------------------------------------------------------------ *
 * A front-end that only records what it was asked to do
 * ------------------------------------------------------------------ */

static struct {
    int created;
    int destroyed;
    int fed;
    bool refuse_create;
} fe_a, fe_b;

static int a_ctx, b_ctx;

static void *fe_a_create(vif_session_t *s, comm_port_id_t port)
{
    fe_a.created++;
    return fe_a.refuse_create ? NULL : &a_ctx;
}

static void fe_a_feed(void *ctx, const uint8_t *data, size_t len)
{
    fe_a.fed += (int)len;
}

static void fe_a_destroy(void *ctx)
{
    fe_a.destroyed++;
}

static void *fe_b_create(vif_session_t *s, comm_port_id_t port)
{
    fe_b.created++;
    return fe_b.refuse_create ? NULL : &b_ctx;
}

static void fe_b_destroy(void *ctx)
{
    fe_b.destroyed++;
}

static const vif_frontend_t frontend_a = {
    .name = "fe_a",
    .create = fe_a_create,
    .feed = fe_a_feed,
    .destroy = fe_a_destroy,
};

static const vif_frontend_t frontend_b = {
    .name = "fe_b",
    .create = fe_b_create,
    .destroy = fe_b_destroy,
};

/* ------------------------------------------------------------------ *
 * Fixture
 * ------------------------------------------------------------------ */

static vif_session_t *open_session(const char *name)
{
    vif_session_t *s = vif_session_open(name, COMM_INVALID_PORT_ID);

    TEST_ASSERT_MSG(s != NULL, "no session for '%s'", name);
    return s;
}

void td_setup(void)
{
    /* vif_init() returns the session pool and every claim to their power-on
     * state, which is what gives each test a clean adapter. */
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_init());

    fake_clock_reset();
    fake_board_reset();
    fake_led_reset();
    fake_can_reset();
    fake_bytebus_reset_all();

    memset(&fe_a, 0, sizeof(fe_a));
    memset(&fe_b, 0, sizeof(fe_b));
}

void td_teardown(void)
{
    TEST_ASSERT_MSG(idf_stub_lock_balance() == 0,
                    "vif left %d locks held", idf_stub_lock_balance());
}

/* ------------------------------------------------------------------ *
 * Sessions
 * ------------------------------------------------------------------ */

TEST(the_session_pool_is_finite_and_reusable)
{
    vif_session_t *held[VIF_MAX_SESSIONS];

    for (int i = 0; i < VIF_MAX_SESSIONS; i++) {
        held[i] = open_session("s");
    }

    TEST_ASSERT_NULL(vif_session_open("one_too_many", COMM_INVALID_PORT_ID));

    vif_session_close(held[1]);
    TEST_ASSERT_NOT_NULL(vif_session_open("reuse", COMM_INVALID_PORT_ID));
}

TEST(a_session_can_be_found_by_name)
{
    vif_session_t *s = open_session("elm");

    TEST_ASSERT_TRUE(vif_session_find("elm") == s);
    TEST_ASSERT_NULL(vif_session_find("nobody"));
}

/* ------------------------------------------------------------------ *
 * Bus arbitration
 * ------------------------------------------------------------------ */

TEST(one_session_holds_the_bus_at_a_time)
{
    vif_session_t *first = open_session("first");
    vif_session_t *second = open_session("second");

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(first, VIF_BUS_CAN, &can500));

    /* The second client is refused rather than stealing the driver. */
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_bus_open(second, VIF_BUS_CAN, &can500));
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_close(first));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(second, VIF_BUS_CAN, &can500));
}

TEST(a_session_that_does_not_hold_the_bus_cannot_close_it)
{
    vif_session_t *owner = open_session("owner");
    vif_session_t *other = open_session("other");

    vif_bus_open(owner, VIF_BUS_CAN, &can500);

    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, vif_bus_close(other));
    TEST_ASSERT_TRUE(fake_can_is_up());
}

TEST(opening_another_bus_in_the_same_session_swaps_it)
{
    vif_session_t *s = open_session("s");

    vif_bus_open(s, VIF_BUS_CAN, &can500);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(s, VIF_BUS_J1850_VPW, NULL));

    TEST_ASSERT_EQUAL_INT(1, fake_can_teardown_count());
    TEST_ASSERT_FALSE(fake_can_is_up());
    TEST_ASSERT_TRUE(fake_bytebus_is_up(FAKE_BUS_J1850_VPW));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_J1850_VPW, vif_bus_current(s));
}

TEST(can_will_not_start_without_a_bit_rate)
{
    vif_session_t *s = open_session("s");

    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, vif_bus_open(s, VIF_BUS_CAN, NULL));
    TEST_ASSERT_EQUAL_INT(0, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE, vif_bus_current(s));
}

TEST(opening_k_line_runs_the_five_baud_init)
{
    vif_session_t *s = open_session("s");

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(s, VIF_BUS_KLINE, NULL));

    TEST_ASSERT_EQUAL_INT(1, fake_bytebus_setup_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(1, fake_bytebus_sync_count());
}

/* ------------------------------------------------------------------ *
 * Transfers follow the claim
 * ------------------------------------------------------------------ */

TEST(a_transfer_without_the_claim_is_refused)
{
    vif_session_t *owner = open_session("owner");
    vif_session_t *other = open_session("other");
    struct can_frame frame = { .id = 0x7DF, .dlc = 1 };

    vif_bus_open(owner, VIF_BUS_CAN, &can500);

    TEST_ASSERT_EQUAL_INT(VIF_ERR_NO_CLAIM, vif_can_send(other, &frame));
    TEST_ASSERT_EQUAL_INT(VIF_ERR_NO_CLAIM, vif_can_recv(other, &frame, 0));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());

    TEST_ASSERT_EQUAL_INT(0, vif_can_send(owner, &frame));
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
}

TEST(a_can_claim_does_not_carry_over_to_the_byte_buses)
{
    vif_session_t *s = open_session("s");
    uint8_t data[4] = { 1, 2, 3, 4 };

    vif_bus_open(s, VIF_BUS_CAN, &can500);

    TEST_ASSERT_EQUAL_INT(VIF_ERR_NO_CLAIM, vif_raw_send(s, data, sizeof(data)));
    TEST_ASSERT_EQUAL_INT(VIF_ERR_NO_CLAIM, vif_raw_recv(s, data, sizeof(data), 0));
}

TEST(a_byte_bus_transfer_reaches_the_bus_the_session_holds)
{
    vif_session_t *s = open_session("s");
    const uint8_t request[3] = { 0x68, 0x6A, 0xF1 };
    const uint8_t reply[2] = { 0x41, 0x00 };
    uint8_t buf[8] = {0};
    size_t sent_len = 0;

    vif_bus_open(s, VIF_BUS_J1850_PWM, NULL);
    fake_bytebus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 0);

    TEST_ASSERT_EQUAL_INT(0, vif_raw_send(s, request, sizeof(request)));
    TEST_ASSERT_EQUAL_INT(1, fake_bytebus_sent_count(FAKE_BUS_J1850_PWM));
    TEST_ASSERT_NOT_NULL(fake_bytebus_sent(FAKE_BUS_J1850_PWM, 0, &sent_len));
    TEST_ASSERT_EQUAL_INT(sizeof(request), sent_len);

    TEST_ASSERT_EQUAL_INT(sizeof(reply), vif_raw_recv(s, buf, sizeof(buf), 100));
    TEST_ASSERT_EQUAL_INT(0x41, buf[0]);

    /* K-Line and VPW are idle: one bus is live, and it is the one asked for. */
    TEST_ASSERT_EQUAL_INT(0, fake_bytebus_sent_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(0, fake_bytebus_sent_count(FAKE_BUS_J1850_VPW));
}

/* ------------------------------------------------------------------ *
 * Pins
 * ------------------------------------------------------------------ */

TEST(a_high_side_pin_is_driven_and_released)
{
    vif_session_t *s = open_session("s");

    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_pin_set(s, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000));
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(OBD_PIN_HS));
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_boost_en());
    TEST_ASSERT_EQUAL_INT(12000, fake_board_hs_voltage_mv());
    TEST_ASSERT_TRUE(vif_any_pin_active());

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_pin_set(s, OBD_PIN_HS, VIF_PIN_OFF, 0));
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_state(OBD_PIN_HS));
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_boost_en());
    TEST_ASSERT_FALSE(vif_any_pin_active());
}

TEST(one_high_side_pin_at_a_time_whoever_asks)
{
    vif_session_t *first = open_session("first");
    vif_session_t *second = open_session("second");

    vif_pin_set(first, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);

    /* SAE J2534 allows one high side driver, so both the holder's second pin
     * and another client's are refused. */
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_pin_set(first, OBD_PIN_HS_OTHER, VIF_PIN_VOLTAGE, 12000));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_pin_set(second, OBD_PIN_HS_OTHER, VIF_PIN_VOLTAGE, 12000));
    TEST_ASSERT_EQUAL_INT(-1, fake_board_hs_state(OBD_PIN_HS_OTHER));
}

TEST(the_holder_may_change_the_voltage_on_its_own_pin)
{
    vif_session_t *s = open_session("s");

    vif_pin_set(s, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_pin_set(s, OBD_PIN_HS, VIF_PIN_VOLTAGE, 18000));

    TEST_ASSERT_EQUAL_INT(18000, fake_board_hs_voltage_mv());
}

TEST(a_pin_one_session_holds_is_not_switched_off_by_another)
{
    vif_session_t *owner = open_session("owner");
    vif_session_t *other = open_session("other");

    vif_pin_set(owner, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);

    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_pin_set(other, OBD_PIN_HS, VIF_PIN_OFF, 0));
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(OBD_PIN_HS));

    /* Nor by a release sweep the other session runs on its own claims. */
    vif_pin_release_all(other);
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(OBD_PIN_HS));
}

TEST(releasing_a_pin_nobody_drives_is_accepted)
{
    vif_session_t *s = open_session("s");

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_pin_set(s, OBD_PIN_HS, VIF_PIN_OFF, 0));
    TEST_ASSERT_EQUAL_INT(-1, fake_board_hs_state(OBD_PIN_HS));
}

TEST(the_low_side_pin_grounds_and_releases_independently)
{
    vif_session_t *s = open_session("s");

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_pin_set(s, OBD_PIN_LS, VIF_PIN_GROUND, 0));
    TEST_ASSERT_EQUAL_INT(1, fake_board_ls_state(OBD_PIN_LS));

    /* One high side and one low side may be held at the same time. */
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_pin_set(s, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000));

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_pin_set(s, OBD_PIN_LS, VIF_PIN_OFF, 0));
    TEST_ASSERT_EQUAL_INT(0, fake_board_ls_state(OBD_PIN_LS));
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(OBD_PIN_HS));
}

TEST(a_pin_group_cannot_do_the_other_groups_job)
{
    vif_session_t *s = open_session("s");

    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG,
                          vif_pin_set(s, OBD_PIN_LS, VIF_PIN_VOLTAGE, 12000));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG,
                          vif_pin_set(s, OBD_PIN_HS, VIF_PIN_GROUND, 0));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG,
                          vif_pin_set(s, 7, VIF_PIN_VOLTAGE, 12000));
}

TEST(closing_a_session_releases_everything_it_held)
{
    vif_session_t *s = open_session("s");

    vif_pin_set(s, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);
    vif_pin_set(s, OBD_PIN_LS, VIF_PIN_GROUND, 0);
    vif_bus_open(s, VIF_BUS_CAN, &can500);

    vif_session_close(s);

    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_state(OBD_PIN_HS));
    TEST_ASSERT_EQUAL_INT(0, fake_board_ls_state(OBD_PIN_LS));
    TEST_ASSERT_FALSE(fake_can_is_up());
    TEST_ASSERT_FALSE(vif_any_pin_active());
}

TEST(the_status_led_warns_while_a_connector_pin_is_live)
{
    vif_session_t *s = open_session("s");

    vif_pin_set(s, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);
    TEST_ASSERT_EQUAL_INT(LED_STATE_PIN_LIVE, fake_led_state());

    /* Still live: the low side pin is grounded even after the high side goes. */
    vif_pin_set(s, OBD_PIN_LS, VIF_PIN_GROUND, 0);
    vif_pin_set(s, OBD_PIN_HS, VIF_PIN_OFF, 0);
    TEST_ASSERT_EQUAL_INT(LED_STATE_PIN_LIVE, fake_led_state());

    vif_pin_release_all(s);
    TEST_ASSERT_EQUAL_INT(LED_STATE_IDLE, fake_led_state());
}

/* ------------------------------------------------------------------ *
 * Analog and calibration
 * ------------------------------------------------------------------ */

TEST(the_analog_readings_need_no_claim)
{
    vif_session_t *s = open_session("s");

    /* Reading a voltage disturbs nothing, so it is not arbitrated. */
    fake_board_set_vbatt_mv(13400);
    TEST_ASSERT_EQUAL_INT(13400, vif_vbatt_mv());
    TEST_ASSERT_EQUAL_INT(0, vif_hs_vsense_mv());
    TEST_ASSERT_EQUAL_INT(1, vif_board_id());

    /* Battery calibration drives no pin either, so it takes no claim. */
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_calibrate_vbatt(s));
    TEST_ASSERT_FALSE(vif_any_pin_active());
}

TEST(calibration_takes_over_the_callers_own_pins)
{
    vif_session_t *shell = open_session("shell");

    vif_pin_set(shell, OBD_PIN_LS, VIF_PIN_GROUND, 0);
    vif_pin_set(shell, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_calibrate_hs(shell));

    /* The procedure resets every driver itself, so a claim left standing here
     * would describe pins that are no longer driven. */
    TEST_ASSERT_FALSE(vif_any_pin_active());
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_state(OBD_PIN_HS));
    TEST_ASSERT_EQUAL_INT(0, fake_board_ls_state(OBD_PIN_LS));
    TEST_ASSERT_EQUAL_INT(LED_STATE_IDLE, fake_led_state());
}

TEST(calibration_is_refused_while_another_session_holds_a_pin)
{
    vif_session_t *shell = open_session("shell");
    vif_session_t *client = open_session("client");

    vif_pin_set(client, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);

    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, vif_calibrate_hs(shell));

    vif_pin_release_all(client);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_calibrate_hs(shell));

    /* The procedure puts the drivers back, so nothing is left claimed. */
    TEST_ASSERT_FALSE(vif_any_pin_active());
}

/* ------------------------------------------------------------------ *
 * Front-ends
 * ------------------------------------------------------------------ */

TEST(installing_a_front_end_creates_it)
{
    vif_session_t *s = open_session("s");

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_session_set_frontend(s, &frontend_a));

    TEST_ASSERT_EQUAL_INT(1, fe_a.created);
    TEST_ASSERT_EQUAL_STRING("fe_a", vif_session_frontend_name(s));
}

TEST(a_front_end_that_refuses_to_start_is_reported)
{
    vif_session_t *s = open_session("s");

    fe_a.refuse_create = true;

    TEST_ASSERT_EQUAL_INT(ESP_FAIL, vif_session_set_frontend(s, &frontend_a));
    TEST_ASSERT_NULL(vif_session_frontend_name(s));
}

TEST(a_front_end_that_refuses_to_start_gives_the_port_back_to_the_old_one)
{
    vif_session_t *s = open_session("s");

    vif_session_set_frontend(s, &frontend_a);
    fe_b.refuse_create = true;

    TEST_ASSERT_EQUAL_INT(ESP_FAIL, vif_session_set_frontend(s, &frontend_b));

    /* A mistyped switch on the console must not leave the port mute. */
    TEST_ASSERT_EQUAL_STRING("fe_a", vif_session_frontend_name(s));
    TEST_ASSERT_EQUAL_INT(2, fe_a.created);
    TEST_ASSERT_EQUAL_INT(1, fe_a.destroyed);
}

TEST(switching_front_ends_leaves_the_claims_standing)
{
    vif_session_t *s = open_session("s");

    vif_session_set_frontend(s, &frontend_a);
    vif_pin_set(s, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);
    vif_bus_open(s, VIF_BUS_CAN, &can500);

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_session_set_frontend(s, &frontend_b));

    TEST_ASSERT_EQUAL_INT(1, fe_a.destroyed);
    TEST_ASSERT_EQUAL_INT(1, fe_b.created);
    TEST_ASSERT_EQUAL_STRING("fe_b", vif_session_frontend_name(s));

    /* This is the whole reason the vtable exists: raise a programming voltage
     * under one protocol, flash under the next, without the pin dropping. */
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(OBD_PIN_HS));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_CAN, vif_bus_current(s));
    TEST_ASSERT_TRUE(fake_can_is_up());
    TEST_ASSERT_EQUAL_INT(0, fake_can_teardown_count());
}

/* ------------------------------------------------------------------ *
 * The front-end table
 * ------------------------------------------------------------------ */

TEST(a_front_end_can_be_found_by_the_name_a_client_types)
{
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_frontend_register(&frontend_a));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_frontend_register(&frontend_b));

    TEST_ASSERT(vif_frontend_find("fe_a") == &frontend_a);

    /* A client types what it likes; "MODE=SLCAN" and "mode=slcan" are one
     * command. */
    TEST_ASSERT(vif_frontend_find("FE_B") == &frontend_b);

    TEST_ASSERT_NULL(vif_frontend_find("nothing"));
    TEST_ASSERT_NULL(vif_frontend_find(NULL));
}

TEST(registering_the_same_front_end_twice_is_harmless)
{
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_frontend_register(&frontend_a));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_frontend_register(&frontend_a));
}

TEST(two_front_ends_cannot_share_a_name)
{
    static const vif_frontend_t impostor = { .name = "FE_A" };

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_frontend_register(&frontend_a));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_frontend_register(&impostor));
}

TEST(the_table_can_be_walked_and_ends)
{
    size_t count = 0;

    vif_frontend_register(&frontend_a);
    vif_frontend_register(&frontend_b);

    while (vif_frontend_at(count) != NULL) {
        count++;
        TEST_ASSERT_MSG(count <= VIF_MAX_FRONTENDS, "the walk did not end");
    }

    TEST_ASSERT_EQUAL_INT(2, (int)count);
}

/* ------------------------------------------------------------------ *
 * Two links at once
 * ------------------------------------------------------------------ */

TEST(each_link_runs_its_own_front_end)
{
    vif_session_t *usb = open_session("usb");
    vif_session_t *ble = open_session("ble");

    /* The reason each link gets its own session: one client switching its own
     * grammar must not change what the other is speaking. */
    vif_session_set_frontend(usb, &frontend_a);
    vif_session_set_frontend(ble, &frontend_a);

    TEST_ASSERT_EQUAL_INT(2, fe_a.created);

    vif_session_set_frontend(ble, &frontend_b);

    TEST_ASSERT_EQUAL_STRING("fe_a", vif_session_frontend_name(usb));
    TEST_ASSERT_EQUAL_STRING("fe_b", vif_session_frontend_name(ble));
}

/* ------------------------------------------------------------------ *
 * Defaults and reverting
 * ------------------------------------------------------------------ */

TEST(a_link_goes_back_to_its_default_when_the_client_leaves)
{
    vif_session_t *s = open_session("usb");

    vif_session_set_default_frontend(s, &frontend_a);
    vif_session_set_frontend(s, &frontend_a);
    vif_session_set_frontend(s, &frontend_b);

    vif_session_link_down(s);

    /* An off-the-shelf ELM327 app has to find ELM327 however the link was
     * last used. */
    TEST_ASSERT_EQUAL_STRING("fe_a", vif_session_frontend_name(s));
    TEST_ASSERT_EQUAL_INT(1, fe_b.destroyed);
}

TEST(a_client_leaving_does_not_release_the_claims)
{
    vif_session_t *s = open_session("usb");

    vif_session_set_default_frontend(s, &frontend_a);
    vif_session_set_frontend(s, &frontend_b);
    vif_pin_set(s, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);
    vif_bus_open(s, VIF_BUS_CAN, &can500);

    vif_session_link_down(s);

    /* Claims persist across a disconnect; the grammar does not. A programming
     * voltage has to survive a replugged cable. */
    TEST_ASSERT_EQUAL_STRING("fe_a", vif_session_frontend_name(s));
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(OBD_PIN_HS));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_CAN, vif_bus_current(s));
    TEST_ASSERT_EQUAL_INT(0, fake_can_teardown_count());
}

TEST(a_link_already_on_its_default_is_left_alone)
{
    vif_session_t *s = open_session("usb");

    vif_session_set_default_frontend(s, &frontend_a);
    vif_session_set_frontend(s, &frontend_a);

    vif_session_link_down(s);
    vif_session_link_down(s);

    /* A tool that merely toggles DTR must not have its interpreter rebuilt,
     * and its settings reset, underneath it. */
    TEST_ASSERT_EQUAL_INT(1, fe_a.created);
    TEST_ASSERT_EQUAL_INT(0, fe_a.destroyed);
}

TEST(a_link_with_no_default_keeps_what_it_is_running)
{
    vif_session_t *s = open_session("usb");

    vif_session_set_frontend(s, &frontend_b);
    vif_session_link_down(s);

    TEST_ASSERT_EQUAL_STRING("fe_b", vif_session_frontend_name(s));
    TEST_ASSERT_EQUAL_INT(0, fe_b.destroyed);
}

/* ------------------------------------------------------------------ *
 * Recovery
 * ------------------------------------------------------------------ */

TEST(recovering_releases_every_claim_and_restores_every_default)
{
    vif_session_t *usb = open_session("usb");
    vif_session_t *ble = open_session("ble");

    vif_session_set_default_frontend(usb, &frontend_a);
    vif_session_set_frontend(usb, &frontend_b);
    vif_pin_set(usb, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);

    vif_session_set_default_frontend(ble, &frontend_a);
    vif_session_set_frontend(ble, &frontend_b);
    vif_bus_open(ble, VIF_BUS_CAN, &can500);

    /*
     * The button only raises the request; it touches no driver and so needs
     * no stack to speak of. Each session lets go on its own thread, which is
     * the only place it is provably not inside a driver call.
     */
    vif_recover_all();

    TEST_ASSERT_EQUAL_INT(VIF_BUS_CAN, vif_bus_current(ble));
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(OBD_PIN_HS));

    TEST_ASSERT_TRUE(vif_session_service(usb));
    TEST_ASSERT_TRUE(vif_session_service(ble));

    /* Asking again once it is done finds nothing to do. */
    TEST_ASSERT_FALSE(vif_session_service(usb));

    TEST_ASSERT_EQUAL_STRING("fe_a", vif_session_frontend_name(usb));
    TEST_ASSERT_EQUAL_STRING("fe_a", vif_session_frontend_name(ble));
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_state(OBD_PIN_HS));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE, vif_bus_current(ble));
    TEST_ASSERT_FALSE(fake_can_is_up());
}

/* ------------------------------------------------------------------ *
 * Reporting what is live
 * ------------------------------------------------------------------ */

TEST(the_bus_report_names_the_bus_its_rate_and_its_holder)
{
    vif_session_t *s = open_session("usb");
    vif_bus_info_t info;

    vif_bus_info(&info);
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE, info.bus);
    TEST_ASSERT_EQUAL_STRING("", info.owner);

    vif_bus_open(s, VIF_BUS_CAN, &can500);
    vif_bus_info(&info);

    /* Without this a client refused the bus learns only that it was refused,
     * not who has it. */
    TEST_ASSERT_EQUAL_INT(VIF_BUS_CAN, info.bus);
    TEST_ASSERT_EQUAL_INT(500000, (int)info.bitrate);
    TEST_ASSERT_EQUAL_STRING("usb", info.owner);
    TEST_ASSERT_EQUAL_STRING("CAN", vif_bus_name(info.bus));

    vif_bus_close(s);
    vif_bus_info(&info);
    TEST_ASSERT_EQUAL_INT(0, (int)info.bitrate);
}
