/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_vif.c
 * @brief The vehicle interface: sessions, claims and arbitration.
 *
 * These tests drive vif.c through its public API with the board and the four
 * bus drivers faked underneath, which is where the rules the whole design
 * rests on actually live: one bus per group of shared wiring, one high side
 * and one low side pin, and a claim that belongs to the session that made it.
 *
 * Sessions here are opened as VIF_SESSION_LOCAL. That is not a shortcut - the
 * debug shell runs exactly like this - and it keeps the session task out of
 * the way, since a task the host stub never starts could not pump anything
 * anyway.
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

#include "can_frame.h"
#include "vif.h"

#define OBD_PIN_HS 6
#define OBD_PIN_HS_OTHER 9
#define OBD_PIN_LS 15

static const vif_bus_cfg_t can500 = {.bitrate = 500000};

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

static void *fe_a_create(vif_session_t *s) {
    fe_a.created++;
    return fe_a.refuse_create ? NULL : &a_ctx;
}

static void fe_a_feed(void *ctx, const uint8_t *data, size_t len) {
    fe_a.fed += (int)len;
}

static void fe_a_destroy(void *ctx) { fe_a.destroyed++; }

static void *fe_b_create(vif_session_t *s) {
    fe_b.created++;
    return fe_b.refuse_create ? NULL : &b_ctx;
}

static void fe_b_destroy(void *ctx) { fe_b.destroyed++; }

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

static vif_session_t *open_session(const char *name) {
    vif_session_t *s = vif_session_open(name, VIF_SESSION_LOCAL);

    TEST_ASSERT_MSG(s != NULL, "no session for '%s'", name);
    return s;
}

void td_setup(void) {
    idf_stub_set_current_task(NULL);
    idf_stub_set_created_task(NULL);
    /* vif_init() returns the session pool and every claim to their power-on
     * state, which is what gives each test a clean adapter. */
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_init());

    fake_clock_reset();
    fake_board_reset();
    fake_led_reset();
    fake_can_reset();
    fake_bus_reset_all();

    memset(&fe_a, 0, sizeof(fe_a));
    memset(&fe_b, 0, sizeof(fe_b));
}

void td_teardown(void) {
    idf_stub_set_current_task(NULL);
    idf_stub_set_created_task(NULL);
    TEST_ASSERT_MSG(idf_stub_lock_balance() == 0, "vif left %d locks held",
                    idf_stub_lock_balance());
}

/* ------------------------------------------------------------------ *
 * Sessions
 * ------------------------------------------------------------------ */

static int other_task, link_task;

static void check_wrong_task_bus_access(vif_session_t *s) {
    TaskHandle_t owner = xTaskGetCurrentTaskHandle();
    uint8_t data[] = {0x55};
    bus_msg_t msg;
    uint32_t value = 0;
    bus_msg_init(&msg, data, sizeof(data));
    msg.len = sizeof(data);
    fake_bus_stage_stale(FAKE_BUS_KLINE, data, sizeof(data));

    idf_stub_set_current_task(&other_task);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_bus_open(s, VIF_BUS_KLINE, NULL));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_bus_close(s, VIF_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(VIF_ERR_NO_CLAIM,
                          vif_bus_recv(s, VIF_BUS_KLINE, &msg, 100));
    TEST_ASSERT_EQUAL_INT(VIF_ERR_NO_CLAIM,
                          vif_bus_send(s, VIF_BUS_KLINE, &msg, 0));
    TEST_ASSERT_EQUAL_INT(
        VIF_ERR_NO_CLAIM,
        vif_bus_param_set(s, VIF_BUS_KLINE, BUS_P_DATA_RATE, 4800));
    TEST_ASSERT_EQUAL_INT(
        VIF_ERR_NO_CLAIM,
        vif_bus_param_get(s, VIF_BUS_KLINE, BUS_P_DATA_RATE, &value));
    TEST_ASSERT_EQUAL_INT(
        VIF_ERR_NO_CLAIM,
        vif_bus_ioctl(s, VIF_BUS_KLINE, BUS_IOCTL_STOP_COMM, NULL, NULL));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_setup_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_teardown_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sent_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_stop_comm_count());

    idf_stub_set_current_task(owner);
    TEST_ASSERT_EQUAL_INT(1, vif_bus_recv(s, VIF_BUS_KLINE, &msg, 0));
    TEST_ASSERT_EQUAL_INT(0x55, data[0]);
}

TEST(taskless_sessions_reject_foreign_driver_and_lifecycle_calls) {
    vif_session_t *s = open_session("local");
    TaskHandle_t owner = xTaskGetCurrentTaskHandle();
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(s, VIF_BUS_KLINE, NULL));
    check_wrong_task_bus_access(s);

    idf_stub_set_current_task(&other_task);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_session_set_frontend(s, &frontend_a));
    vif_session_close(s);
    TEST_ASSERT_TRUE(vif_session_find("local") == s);
    TEST_ASSERT_TRUE(fake_bus_is_up(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(0, fe_a.created);

    idf_stub_set_current_task(owner);
    vif_session_close(s);
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_KLINE));
    TEST_ASSERT_NULL(vif_session_find("local"));
}

TEST(link_task_takes_bus_ownership_from_creator) {
    vif_session_t *s = vif_session_open("link", VIF_SESSION_LINK);
    idf_stub_set_created_task(&link_task);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_session_set_frontend(s, &frontend_a));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_bus_open(s, VIF_BUS_KLINE, NULL));

    idf_stub_set_current_task(&link_task);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(s, VIF_BUS_KLINE, NULL));
    check_wrong_task_bus_access(s);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_close(s, VIF_BUS_KLINE));
}

static void release_during_receive(void *arg) {
    vif_session_t *s = arg;
    TaskHandle_t owner = xTaskGetCurrentTaskHandle();
    idf_stub_set_current_task(&other_task);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_release_all(s));
    vif_recover_all();
    TEST_ASSERT_FALSE(vif_session_service(s));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_teardown_count(FAKE_BUS_KLINE));
    idf_stub_set_current_task(owner);
}

TEST(release_during_receive_waits_for_owner_service) {
    for (int link = 0; link < 2; link++) {
        vif_session_t *s =
            vif_session_open(link ? "link" : "local",
                             link ? VIF_SESSION_LINK : VIF_SESSION_LOCAL);
        if (link) {
            idf_stub_set_created_task(&link_task);
            TEST_ASSERT_EQUAL_INT(ESP_OK,
                                  vif_session_set_frontend(s, &frontend_a));
            idf_stub_set_current_task(&link_task);
        }
        fake_bus_reset_all();
        TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(s, VIF_BUS_KLINE, NULL));
        uint8_t data[] = {0x55};
        bus_msg_t msg;
        bus_msg_init(&msg, data, sizeof(data));
        fake_bus_stage_stale(FAKE_BUS_KLINE, data, sizeof(data));
        fake_bus_on_receive(FAKE_BUS_KLINE, release_during_receive, s);
        TEST_ASSERT_EQUAL_INT(1, vif_bus_recv(s, VIF_BUS_KLINE, &msg, 100));
        TEST_ASSERT_TRUE(fake_bus_is_up(FAKE_BUS_KLINE));
        TEST_ASSERT_TRUE(vif_session_service(s));
        TEST_ASSERT_EQUAL_INT(1, fake_bus_teardown_count(FAKE_BUS_KLINE));
        TEST_ASSERT_FALSE(vif_session_service(s));
    }
}

TEST(failed_close_retains_claim_and_blocks_reopen) {
    vif_session_t *s = open_session("owner");
    vif_session_t *other = open_session("other");
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(s, VIF_BUS_KLINE, NULL));
    fake_bus_close_result(FAKE_BUS_KLINE, ESP_ERR_TIMEOUT);

    TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT, vif_bus_close(s, VIF_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_KLINE, vif_bus_current(s, VIF_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT,
                          vif_bus_open(s, VIF_BUS_KLINE, NULL));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_bus_open(other, VIF_BUS_KLINE, NULL));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_setup_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_teardown_count(FAKE_BUS_KLINE));

    fake_bus_close_result(FAKE_BUS_KLINE, ESP_OK);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_close(s, VIF_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE, vif_bus_current(s, VIF_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(other, VIF_BUS_KLINE, NULL));
}

TEST(release_all_keeps_only_failed_claims) {
    vif_session_t *s = open_session("owner");
    vif_bus_open(s, VIF_BUS_KLINE, NULL);
    vif_bus_open(s, VIF_BUS_CAN, &can500);
    fake_bus_close_result(FAKE_BUS_KLINE, ESP_ERR_TIMEOUT);

    TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT, vif_bus_release_all(s));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_KLINE, vif_bus_current(s, VIF_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE, vif_bus_current(s, VIF_BUS_CAN));
    fake_bus_close_result(FAKE_BUS_KLINE, ESP_OK);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_release_all(s));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE, vif_bus_current(s, VIF_BUS_KLINE));
}

TEST(failed_shutdown_keeps_session_slot_until_retry) {
    vif_session_t *s = open_session("owner");
    vif_session_set_frontend(s, &frontend_a);
    vif_bus_open(s, VIF_BUS_KLINE, NULL);
    fake_bus_close_result(FAKE_BUS_KLINE, ESP_ERR_TIMEOUT);

    vif_session_close(s);
    TEST_ASSERT_TRUE(vif_session_find("owner") == s);
    TEST_ASSERT_EQUAL_INT(VIF_BUS_KLINE, vif_bus_current(s, VIF_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(1, fe_a.destroyed);
    for (int i = 1; i < VIF_MAX_SESSIONS; i++) {
        TEST_ASSERT_TRUE(open_session("other") != s);
    }
    TEST_ASSERT_NULL(vif_session_open("full", VIF_SESSION_LOCAL));

    fake_bus_close_result(FAKE_BUS_KLINE, ESP_OK);
    vif_session_close(s);
    TEST_ASSERT_NULL(vif_session_find("owner"));
    TEST_ASSERT_EQUAL_INT(1, fe_a.destroyed);
    TEST_ASSERT_TRUE(open_session("reused") == s);
}

TEST(frontend_switch_waits_for_failed_close_to_recover) {
    vif_session_t *s = open_session("owner");
    vif_session_set_frontend(s, &frontend_a);
    vif_bus_open(s, VIF_BUS_KLINE, NULL);
    fake_bus_close_result(FAKE_BUS_KLINE, ESP_ERR_TIMEOUT);

    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_session_set_frontend(s, &frontend_b));
    TEST_ASSERT_EQUAL_INT(1, fe_a.destroyed);
    TEST_ASSERT_EQUAL_INT(0, fe_b.created);
    TEST_ASSERT_EQUAL_INT(VIF_BUS_KLINE, vif_bus_current(s, VIF_BUS_KLINE));

    fake_bus_close_result(FAKE_BUS_KLINE, ESP_OK);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_session_set_frontend(s, &frontend_b));
    TEST_ASSERT_EQUAL_INT(1, fe_b.created);
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE, vif_bus_current(s, VIF_BUS_KLINE));
}

TEST(recovery_request_survives_close_timeout) {
    vif_session_t *s = open_session("owner");
    vif_session_set_default_frontend(s, &frontend_a);
    vif_session_set_frontend(s, &frontend_a);
    vif_bus_open(s, VIF_BUS_KLINE, NULL);
    fake_bus_close_result(FAKE_BUS_KLINE, ESP_ERR_TIMEOUT);
    vif_recover_all();

    TEST_ASSERT_TRUE(vif_session_service(s));
    TEST_ASSERT_EQUAL_INT(1, fe_a.created);
    TEST_ASSERT_EQUAL_INT(0, fe_a.destroyed);
    TEST_ASSERT_EQUAL_INT(VIF_BUS_KLINE, vif_bus_current(s, VIF_BUS_KLINE));
    fake_bus_close_result(FAKE_BUS_KLINE, ESP_OK);
    TEST_ASSERT_TRUE(vif_session_service(s));
    TEST_ASSERT_EQUAL_INT(2, fe_a.created);
    TEST_ASSERT_EQUAL_INT(1, fe_a.destroyed);
    TEST_ASSERT_FALSE(vif_session_service(s));
}

TEST(the_session_pool_is_finite_and_reusable) {
    vif_session_t *held[VIF_MAX_SESSIONS];

    for (int i = 0; i < VIF_MAX_SESSIONS; i++) {
        held[i] = open_session("s");
    }

    TEST_ASSERT_NULL(vif_session_open("one_too_many", VIF_SESSION_LOCAL));

    vif_session_close(held[1]);
    TEST_ASSERT_NOT_NULL(vif_session_open("reuse", VIF_SESSION_LOCAL));
}

TEST(a_session_can_be_found_by_name) {
    vif_session_t *s = open_session("elm");

    TEST_ASSERT_TRUE(vif_session_find("elm") == s);
    TEST_ASSERT_NULL(vif_session_find("nobody"));
}

/* ------------------------------------------------------------------ *
 * Bus arbitration
 * ------------------------------------------------------------------ */

TEST(one_session_holds_a_bus_at_a_time) {
    vif_session_t *first = open_session("first");
    vif_session_t *second = open_session("second");

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(first, VIF_BUS_CAN, &can500));

    /* The second client is refused rather than stealing the driver. */
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_bus_open(second, VIF_BUS_CAN, &can500));
    TEST_ASSERT_EQUAL_INT(1, fake_can_setup_count());

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_close(first, VIF_BUS_CAN));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(second, VIF_BUS_CAN, &can500));
}

TEST(buses_that_are_wired_apart_run_at_the_same_time) {
    vif_session_t *s = open_session("s");

    /* The whole point of arbitrating per group: CAN, K-Line and one J1850
     * modulation have nothing in common electrically, so a session that wants
     * all three gets all three. */
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(s, VIF_BUS_CAN, &can500));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(s, VIF_BUS_KLINE, NULL));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(s, VIF_BUS_J1850_PWM, NULL));

    TEST_ASSERT_TRUE(fake_can_is_up());
    TEST_ASSERT_TRUE(fake_bus_is_up(FAKE_BUS_KLINE));
    TEST_ASSERT_TRUE(fake_bus_is_up(FAKE_BUS_J1850_PWM));

    TEST_ASSERT_EQUAL_INT(VIF_BUS_CAN, vif_bus_current(s, VIF_BUS_CAN));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_KLINE, vif_bus_current(s, VIF_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_J1850_PWM,
                          vif_bus_current(s, VIF_BUS_J1850_PWM));

    /* Nothing was torn down to make room. */
    TEST_ASSERT_EQUAL_INT(0, fake_can_teardown_count());
}

TEST(different_sessions_can_hold_different_buses) {
    vif_session_t *a = open_session("a");
    vif_session_t *b = open_session("b");

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(a, VIF_BUS_CAN, &can500));

    /* A refusal is about the wiring, not about somebody else being busy. */
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(b, VIF_BUS_KLINE, NULL));
    TEST_ASSERT_TRUE(fake_can_is_up());
    TEST_ASSERT_TRUE(fake_bus_is_up(FAKE_BUS_KLINE));
}

TEST(pwm_and_vpw_cannot_both_be_on_the_connector) {
    vif_session_t *a = open_session("a");
    vif_session_t *b = open_session("b");

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(a, VIF_BUS_J1850_PWM, NULL));

    /* They share the transceiver's TX pins and its mode select, so this is a
     * refusal even though no other session asked for PWM itself. */
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_bus_open(b, VIF_BUS_J1850_VPW, NULL));
    TEST_ASSERT_TRUE(fake_bus_is_up(FAKE_BUS_J1850_PWM));
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_J1850_VPW));
}

TEST(closing_one_bus_leaves_the_others_alone) {
    vif_session_t *s = open_session("s");

    vif_bus_open(s, VIF_BUS_CAN, &can500);
    vif_bus_open(s, VIF_BUS_KLINE, NULL);

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_close(s, VIF_BUS_KLINE));

    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_KLINE));
    TEST_ASSERT_TRUE(fake_can_is_up());
    TEST_ASSERT_EQUAL_INT(VIF_BUS_CAN, vif_bus_current(s, VIF_BUS_CAN));
}

TEST(releasing_everything_lets_go_of_every_group) {
    vif_session_t *s = open_session("s");

    vif_bus_open(s, VIF_BUS_CAN, &can500);
    vif_bus_open(s, VIF_BUS_KLINE, NULL);
    vif_bus_open(s, VIF_BUS_J1850_VPW, NULL);

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_release_all(s));

    TEST_ASSERT_FALSE(fake_can_is_up());
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_KLINE));
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_J1850_VPW));
}

TEST(a_session_that_does_not_hold_the_bus_cannot_close_it) {
    vif_session_t *owner = open_session("owner");
    vif_session_t *other = open_session("other");

    vif_bus_open(owner, VIF_BUS_CAN, &can500);

    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_bus_close(other, VIF_BUS_CAN));
    TEST_ASSERT_TRUE(fake_can_is_up());
}

TEST(opening_the_other_bus_in_a_group_swaps_it) {
    vif_session_t *s = open_session("s");

    vif_bus_open(s, VIF_BUS_J1850_PWM, NULL);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(s, VIF_BUS_J1850_VPW, NULL));

    /* One call covers the change of modulation, because the two cannot be
     * live together on the one transceiver. */
    TEST_ASSERT_EQUAL_INT(1, fake_bus_teardown_count(FAKE_BUS_J1850_PWM));
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_J1850_PWM));
    TEST_ASSERT_TRUE(fake_bus_is_up(FAKE_BUS_J1850_VPW));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_J1850_VPW,
                          vif_bus_current(s, VIF_BUS_J1850_PWM));
}

TEST(reopening_can_at_another_rate_restarts_it) {
    vif_session_t *s = open_session("s");
    const vif_bus_cfg_t can250 = {.bitrate = 250000};

    vif_bus_open(s, VIF_BUS_CAN, &can500);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(s, VIF_BUS_CAN, &can250));

    /* The controller takes its rate at install time, so a new rate is a new
     * driver. */
    TEST_ASSERT_EQUAL_INT(1, fake_can_teardown_count());
    TEST_ASSERT_EQUAL_INT(2, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(250000, fake_can_last_baud());
}

TEST(can_will_not_start_without_a_bit_rate) {
    vif_session_t *s = open_session("s");

    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG,
                          vif_bus_open(s, VIF_BUS_CAN, NULL));
    TEST_ASSERT_EQUAL_INT(0, fake_can_setup_count());
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE, vif_bus_current(s, VIF_BUS_CAN));
}

/** @brief Does @p s hold a byte bus with a session established on it? */
static bool session_has_link(vif_session_t *s) {
    bus_link_t link;

    if (vif_bus_ioctl(s, VIF_BUS_KLINE, BUS_IOCTL_GET_LINK, NULL, &link) != 0) {
        return false;
    }

    return link.connected;
}

TEST(opening_k_line_brings_the_driver_up_without_initialising_it) {
    vif_session_t *s = open_session("s");

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(s, VIF_BUS_KLINE, NULL));

    /* Claiming the bus and establishing a session are separate on K-Line:
     * the handshake takes seconds and is entitled to fail, and a client that
     * only wants to watch the bus should not pay for one. */
    TEST_ASSERT_EQUAL_INT(1, fake_bus_setup_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sync_count());
    TEST_ASSERT_FALSE(session_has_link(s));
}

TEST(the_k_line_session_is_established_on_request) {
    vif_session_t *s = open_session("s");
    bus_init_t io = {.address = KLINE_INIT_ADDR_OBD};
    bus_link_t link;

    vif_bus_open(s, VIF_BUS_KLINE, NULL);

    TEST_ASSERT_EQUAL_INT(
        0, vif_bus_ioctl(s, VIF_BUS_KLINE, BUS_IOCTL_FIVE_BAUD_INIT, &io, &io));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sync_count());
    TEST_ASSERT_EQUAL_INT(
        0, vif_bus_ioctl(s, VIF_BUS_KLINE, BUS_IOCTL_GET_LINK, NULL, &link));
    TEST_ASSERT_TRUE(link.connected);
}

TEST(a_k_line_session_belongs_to_whoever_holds_the_bus) {
    vif_session_t *owner = open_session("owner");
    vif_session_t *other = open_session("other");
    bus_init_t io = {.address = KLINE_INIT_ADDR_OBD};
    bus_msg_t tx;

    vif_bus_open(owner, VIF_BUS_KLINE, NULL);

    /* Every one of these reaches the same driver through the same vtable, so
     * the claim has to be checked once, in vif.c, rather than in each of
     * them. This is what proves it is. */
    TEST_ASSERT_EQUAL_INT(VIF_ERR_NO_CLAIM,
                          vif_bus_ioctl(other, VIF_BUS_KLINE,
                                        BUS_IOCTL_FIVE_BAUD_INIT, &io, &io));
    TEST_ASSERT_EQUAL_INT(
        VIF_ERR_NO_CLAIM,
        vif_bus_ioctl(other, VIF_BUS_KLINE, BUS_IOCTL_STOP_COMM, NULL, NULL));
    TEST_ASSERT_EQUAL_INT(
        VIF_ERR_NO_CLAIM,
        vif_bus_param_set(other, VIF_BUS_KLINE, BUS_P_DATA_RATE, 9600));
    bus_msg_tx(&tx, (const uint8_t *)"\x01", 1, 0);
    TEST_ASSERT_EQUAL_INT(VIF_ERR_NO_CLAIM,
                          vif_bus_send(other, VIF_BUS_KLINE, &tx, 0));
    TEST_ASSERT_FALSE(session_has_link(other));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sync_count());
}

TEST(the_k_line_session_ends_when_the_client_asks) {
    vif_session_t *s = open_session("s");
    bus_init_t io = {.address = KLINE_INIT_ADDR_OBD};

    vif_bus_open(s, VIF_BUS_KLINE, NULL);
    vif_bus_ioctl(s, VIF_BUS_KLINE, BUS_IOCTL_FIVE_BAUD_INIT, &io, &io);

    TEST_ASSERT_EQUAL_INT(
        0, vif_bus_ioctl(s, VIF_BUS_KLINE, BUS_IOCTL_STOP_COMM, NULL, NULL));
    TEST_ASSERT_FALSE(session_has_link(s));
}

TEST(a_parameter_a_bus_does_not_have_is_reported_rather_than_refused) {
    vif_session_t *s = open_session("s");
    uint32_t value = 0;

    vif_bus_open(s, VIF_BUS_KLINE, NULL);

    /* The whole point of one interface over three buses: a front-end offers
     * the entire parameter set and the driver says which of them it has. */
    TEST_ASSERT_EQUAL_INT(
        0, vif_bus_param_get(s, VIF_BUS_KLINE, BUS_P_DATA_RATE, &value));
    TEST_ASSERT_EQUAL_INT(
        BUS_ERR_UNSUPPORTED,
        vif_bus_param_get(s, VIF_BUS_KLINE, BUS_P_NETWORK_LINE, &value));
}

/* ------------------------------------------------------------------ *
 * Transfers follow the claim
 * ------------------------------------------------------------------ */

TEST(a_transfer_without_the_claim_is_refused) {
    vif_session_t *owner = open_session("owner");
    vif_session_t *other = open_session("other");
    const uint8_t payload[1] = {0x00};
    uint8_t buf[8];
    bus_msg_t tx;
    bus_msg_t rx;

    vif_bus_open(owner, VIF_BUS_CAN, &can500);
    bus_msg_tx(&tx, payload, sizeof(payload), 0x7DF);
    bus_msg_init(&rx, buf, sizeof(buf));

    TEST_ASSERT_EQUAL_INT(VIF_ERR_NO_CLAIM,
                          vif_bus_send(other, VIF_BUS_CAN, &tx, 0));
    TEST_ASSERT_EQUAL_INT(VIF_ERR_NO_CLAIM,
                          vif_bus_recv(other, VIF_BUS_CAN, &rx, 0));
    TEST_ASSERT_EQUAL_INT(0, fake_can_sent_count());

    TEST_ASSERT_EQUAL_INT(0, vif_bus_send(owner, VIF_BUS_CAN, &tx, 0));
    TEST_ASSERT_EQUAL_INT(1, fake_can_sent_count());
}

TEST(an_extended_id_and_its_flags_reach_the_driver_intact) {
    vif_session_t *s = open_session("s");
    const uint8_t payload[3] = {0x02, 0x01, 0x00};
    const struct can_frame *sent;
    bus_msg_t tx;

    vif_bus_open(s, VIF_BUS_CAN, &can500);
    bus_msg_tx(&tx, payload, sizeof(payload), 0x18DB33F1u | CAN_EFF_FLAG);

    TEST_ASSERT_EQUAL_INT(0, vif_bus_send(s, VIF_BUS_CAN, &tx, 0));

    /* The id rides in the message rather than in a format of its own, so
     * there is nothing in between that could round trip it wrongly. */
    sent = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(sent);
    /* Compared as unsigned: an extended id with its flag set does not fit
     * in an int. */
    TEST_ASSERT_TRUE(sent->id == (0x18DB33F1u | CAN_EFF_FLAG));
    TEST_ASSERT_EQUAL_INT(3, sent->dlc);
    TEST_ASSERT_EQUAL_INT(0, memcmp(sent->data, payload, sizeof(payload)));
}

TEST(a_received_frame_brings_its_id_back_with_it) {
    vif_session_t *s = open_session("s");
    const uint8_t payload[3] = {0x41, 0x00, 0xFF};
    uint8_t buf[8] = {0};
    bus_msg_t rx;

    vif_bus_open(s, VIF_BUS_CAN, &can500);
    bus_msg_init(&rx, buf, sizeof(buf));
    fake_can_stage_stale(0x7E8, 3, payload);

    TEST_ASSERT_EQUAL_INT(3, vif_bus_recv(s, VIF_BUS_CAN, &rx, 0));
    TEST_ASSERT_EQUAL_INT(0x7E8, (int)rx.id);
    TEST_ASSERT_EQUAL_INT(3, rx.len);
    TEST_ASSERT_EQUAL_INT(0, memcmp(buf, payload, sizeof(payload)));
}

TEST(a_remote_frame_carries_its_length_code_and_no_payload) {
    vif_session_t *s = open_session("s");
    const uint8_t none[8] = {0};
    const struct can_frame *sent;
    bus_msg_t tx;

    vif_bus_open(s, VIF_BUS_CAN, &can500);

    /* A remote frame asks for eight bytes and sends none, so the length code
     * has to survive on its own. The buffer is along for the ride: the
     * driver skips the copy once it sees the flag. */
    bus_msg_tx(&tx, none, 8, 0x123 | CAN_RTR_FLAG);

    TEST_ASSERT_EQUAL_INT(0, vif_bus_send(s, VIF_BUS_CAN, &tx, 0));

    sent = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(sent);
    TEST_ASSERT_TRUE((sent->id & CAN_RTR_FLAG) != 0);
    TEST_ASSERT_EQUAL_INT(8, sent->dlc);
}

TEST(can_can_be_driven_through_the_common_interface) {
    vif_session_t *s = open_session("s");
    const uint8_t payload[2] = {0xAA, 0xBB};
    const struct can_frame *sent;
    bus_msg_t tx;

    vif_bus_open(s, VIF_BUS_CAN, &can500);

    /* No CAN shaped call anywhere: a client that knows only bus.h puts the
     * arbitration id in the message and sends it like any other bus. This is
     * what the J2534 layer will do. */
    bus_msg_tx(&tx, payload, sizeof(payload), 0x7DF);
    TEST_ASSERT_EQUAL_INT(0, vif_bus_send(s, VIF_BUS_CAN, &tx, 0));

    sent = fake_can_sent(0);
    TEST_ASSERT_NOT_NULL(sent);
    TEST_ASSERT_EQUAL_INT(0x7DF, (int)sent->id);
    TEST_ASSERT_EQUAL_INT(2, sent->dlc);
    TEST_ASSERT_EQUAL_INT(0, memcmp(sent->data, payload, sizeof(payload)));
}

TEST(a_can_claim_does_not_carry_over_to_the_byte_buses) {
    vif_session_t *s = open_session("s");
    uint8_t data[4] = {1, 2, 3, 4};
    bus_msg_t msg;
    bus_msg_t tx;

    vif_bus_open(s, VIF_BUS_CAN, &can500);
    bus_msg_init(&msg, data, sizeof(data));
    bus_msg_tx(&tx, data, sizeof(data), 0);

    TEST_ASSERT_EQUAL_INT(VIF_ERR_NO_CLAIM,
                          vif_bus_send(s, VIF_BUS_KLINE, &tx, 0));
    TEST_ASSERT_EQUAL_INT(VIF_ERR_NO_CLAIM,
                          vif_bus_recv(s, VIF_BUS_KLINE, &msg, 0));
}

TEST(a_byte_bus_transfer_reaches_the_bus_the_session_holds) {
    vif_session_t *s = open_session("s");
    const uint8_t request[3] = {0x68, 0x6A, 0xF1};
    const uint8_t reply[2] = {0x41, 0x00};
    uint8_t buf[8] = {0};
    size_t sent_len = 0;
    bus_msg_t msg;
    bus_msg_t tx;

    vif_bus_open(s, VIF_BUS_J1850_PWM, NULL);
    bus_msg_init(&msg, buf, sizeof(buf));
    bus_msg_tx(&tx, request, sizeof(request), 0);
    fake_bus_stage_response(FAKE_BUS_J1850_PWM, reply, sizeof(reply), 0);

    TEST_ASSERT_EQUAL_INT(0, vif_bus_send(s, VIF_BUS_J1850_PWM, &tx, 0));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_sent_count(FAKE_BUS_J1850_PWM));
    TEST_ASSERT_NOT_NULL(fake_bus_sent(FAKE_BUS_J1850_PWM, 0, &sent_len));
    TEST_ASSERT_EQUAL_INT(sizeof(request), sent_len);

    TEST_ASSERT_EQUAL_INT(sizeof(reply),
                          vif_bus_recv(s, VIF_BUS_J1850_PWM, &msg, 100));
    TEST_ASSERT_EQUAL_INT(0x41, buf[0]);

    /* K-Line and VPW are idle: only the bus asked for was touched. */
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sent_count(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sent_count(FAKE_BUS_J1850_VPW));
}

/* ------------------------------------------------------------------ *
 * Pins
 * ------------------------------------------------------------------ */

TEST(a_high_side_pin_is_driven_and_released) {
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

TEST(one_high_side_pin_at_a_time_whoever_asks) {
    vif_session_t *first = open_session("first");
    vif_session_t *second = open_session("second");

    vif_pin_set(first, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);

    /* SAE J2534 allows one high side driver, so both the holder's second pin
     * and another client's are refused. */
    TEST_ASSERT_EQUAL_INT(
        ESP_ERR_INVALID_STATE,
        vif_pin_set(first, OBD_PIN_HS_OTHER, VIF_PIN_VOLTAGE, 12000));
    TEST_ASSERT_EQUAL_INT(
        ESP_ERR_INVALID_STATE,
        vif_pin_set(second, OBD_PIN_HS_OTHER, VIF_PIN_VOLTAGE, 12000));
    TEST_ASSERT_EQUAL_INT(-1, fake_board_hs_state(OBD_PIN_HS_OTHER));
}

TEST(the_holder_may_change_the_voltage_on_its_own_pin) {
    vif_session_t *s = open_session("s");

    vif_pin_set(s, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_pin_set(s, OBD_PIN_HS, VIF_PIN_VOLTAGE, 18000));

    TEST_ASSERT_EQUAL_INT(18000, fake_board_hs_voltage_mv());
}

TEST(a_pin_one_session_holds_is_not_switched_off_by_another) {
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

TEST(releasing_a_pin_nobody_drives_is_accepted) {
    vif_session_t *s = open_session("s");

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_pin_set(s, OBD_PIN_HS, VIF_PIN_OFF, 0));
    TEST_ASSERT_EQUAL_INT(-1, fake_board_hs_state(OBD_PIN_HS));
}

TEST(the_low_side_pin_grounds_and_releases_independently) {
    vif_session_t *s = open_session("s");

    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_pin_set(s, OBD_PIN_LS, VIF_PIN_GROUND, 0));
    TEST_ASSERT_EQUAL_INT(1, fake_board_ls_state(OBD_PIN_LS));

    /* One high side and one low side may be held at the same time. */
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_pin_set(s, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000));

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_pin_set(s, OBD_PIN_LS, VIF_PIN_OFF, 0));
    TEST_ASSERT_EQUAL_INT(0, fake_board_ls_state(OBD_PIN_LS));
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(OBD_PIN_HS));
}

TEST(a_pin_group_cannot_do_the_other_groups_job) {
    vif_session_t *s = open_session("s");

    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG,
                          vif_pin_set(s, OBD_PIN_LS, VIF_PIN_VOLTAGE, 12000));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG,
                          vif_pin_set(s, OBD_PIN_HS, VIF_PIN_GROUND, 0));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG,
                          vif_pin_set(s, 7, VIF_PIN_VOLTAGE, 12000));
}

TEST(closing_a_session_releases_everything_it_held) {
    vif_session_t *s = open_session("s");

    vif_pin_set(s, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);
    vif_pin_set(s, OBD_PIN_LS, VIF_PIN_GROUND, 0);
    vif_bus_open(s, VIF_BUS_CAN, &can500);
    vif_bus_open(s, VIF_BUS_KLINE, NULL);

    vif_session_close(s);

    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_state(OBD_PIN_HS));
    TEST_ASSERT_EQUAL_INT(0, fake_board_ls_state(OBD_PIN_LS));
    TEST_ASSERT_FALSE(fake_can_is_up());
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_KLINE));
    TEST_ASSERT_FALSE(vif_any_pin_active());
}

TEST(the_status_led_warns_while_a_connector_pin_is_live) {
    vif_session_t *s = open_session("s");

    vif_pin_set(s, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);
    TEST_ASSERT_EQUAL_INT(LED_STATE_PIN_LIVE, fake_led_state());

    /* Still live: the low side pin is grounded even after the high side goes.
     */
    vif_pin_set(s, OBD_PIN_LS, VIF_PIN_GROUND, 0);
    vif_pin_set(s, OBD_PIN_HS, VIF_PIN_OFF, 0);
    TEST_ASSERT_EQUAL_INT(LED_STATE_PIN_LIVE, fake_led_state());

    vif_pin_release_all(s);
    TEST_ASSERT_EQUAL_INT(LED_STATE_IDLE, fake_led_state());
}

/* ------------------------------------------------------------------ *
 * Analog and calibration
 * ------------------------------------------------------------------ */

TEST(the_analog_readings_need_no_claim) {
    vif_session_t *s = open_session("s");

    /* Reading a voltage disturbs nothing, so it is not arbitrated. */
    fake_board_set_vbatt_mv(13400);
    TEST_ASSERT_EQUAL_INT(13400, vif_vbatt_mv());
    TEST_ASSERT_EQUAL_INT(0, vif_hs_vsense_mv());

    /* Battery calibration drives no pin either, so it takes no claim. */
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_calibrate_vbatt(s));
    TEST_ASSERT_FALSE(vif_any_pin_active());
}

TEST(calibration_takes_over_the_callers_own_pins) {
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

TEST(calibration_is_refused_while_another_session_holds_a_pin) {
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

TEST(installing_a_front_end_creates_it) {
    vif_session_t *s = open_session("s");

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_session_set_frontend(s, &frontend_a));

    TEST_ASSERT_EQUAL_INT(1, fe_a.created);
    TEST_ASSERT_EQUAL_STRING("fe_a", vif_session_frontend_name(s));
}

TEST(a_front_end_that_refuses_to_start_is_reported) {
    vif_session_t *s = open_session("s");

    fe_a.refuse_create = true;

    TEST_ASSERT_EQUAL_INT(ESP_FAIL, vif_session_set_frontend(s, &frontend_a));
    TEST_ASSERT_NULL(vif_session_frontend_name(s));
}

TEST(a_front_end_that_refuses_to_start_gives_the_port_back_to_the_old_one) {
    vif_session_t *s = open_session("s");

    vif_session_set_frontend(s, &frontend_a);
    fe_b.refuse_create = true;

    TEST_ASSERT_EQUAL_INT(ESP_FAIL, vif_session_set_frontend(s, &frontend_b));

    /* A mistyped switch on the console must not leave the port mute. */
    TEST_ASSERT_EQUAL_STRING("fe_a", vif_session_frontend_name(s));
    TEST_ASSERT_EQUAL_INT(2, fe_a.created);
    TEST_ASSERT_EQUAL_INT(1, fe_a.destroyed);
}

TEST(switching_front_ends_releases_every_claim) {
    vif_session_t *s = open_session("s");

    vif_session_set_frontend(s, &frontend_a);
    vif_pin_set(s, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);
    vif_bus_open(s, VIF_BUS_CAN, &can500);
    vif_bus_open(s, VIF_BUS_KLINE, NULL);

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_session_set_frontend(s, &frontend_b));

    TEST_ASSERT_EQUAL_INT(1, fe_a.destroyed);
    TEST_ASSERT_EQUAL_INT(1, fe_b.created);
    TEST_ASSERT_EQUAL_STRING("fe_b", vif_session_frontend_name(s));

    /* A grammar change is a fresh start: the buses the last one claimed were
     * claimed for its protocol, and every one of them goes, not just the one
     * the new grammar happens to want. */
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_state(OBD_PIN_HS));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE, vif_bus_current(s, VIF_BUS_CAN));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE, vif_bus_current(s, VIF_BUS_KLINE));
    TEST_ASSERT_FALSE(fake_can_is_up());
    TEST_ASSERT_EQUAL_INT(1, fake_can_teardown_count());
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_KLINE));
}

/* ------------------------------------------------------------------ *
 * The front-end table
 * ------------------------------------------------------------------ */

TEST(a_front_end_can_be_found_by_the_name_a_client_types) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_frontend_register(&frontend_a));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_frontend_register(&frontend_b));

    TEST_ASSERT(vif_frontend_find("fe_a") == &frontend_a);

    /* A client types what it likes; "MODE=SLCAN" and "mode=slcan" are one
     * command. */
    TEST_ASSERT(vif_frontend_find("FE_B") == &frontend_b);

    TEST_ASSERT_NULL(vif_frontend_find("nothing"));
    TEST_ASSERT_NULL(vif_frontend_find(NULL));
}

TEST(registering_the_same_front_end_twice_is_harmless) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_frontend_register(&frontend_a));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_frontend_register(&frontend_a));
}

TEST(two_front_ends_cannot_share_a_name) {
    static const vif_frontend_t impostor = {.name = "FE_A"};

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_frontend_register(&frontend_a));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_frontend_register(&impostor));
}

TEST(the_table_can_be_walked_and_ends) {
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

TEST(each_link_runs_its_own_front_end) {
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

TEST(a_link_goes_back_to_its_default_when_the_client_leaves) {
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

TEST(a_client_leaving_gives_up_its_pins_and_gives_up_its_bus) {
    vif_session_t *s = open_session("usb");

    vif_session_set_default_frontend(s, &frontend_a);
    vif_session_set_frontend(s, &frontend_b);
    vif_pin_set(s, OBD_PIN_HS, VIF_PIN_VOLTAGE, 12000);
    vif_bus_open(s, VIF_BUS_CAN, &can500);

    vif_session_link_down(s);

    /* The grammar goes back to the default, and the hardware goes with the
     * client: held past its departure it locks every other client out, and
     * nothing is lost by letting go, because the front-end just restarted at
     * protocol 0 and whatever reconnects reopens what it actually wants. */
    TEST_ASSERT_EQUAL_STRING("fe_a", vif_session_frontend_name(s));
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_state(OBD_PIN_HS));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE, vif_bus_current(s, VIF_BUS_CAN));
    TEST_ASSERT_EQUAL_INT(1, fake_can_teardown_count());
}

TEST(the_bus_a_departed_client_held_can_be_taken_by_another_link) {
    vif_session_t *a = open_session("ble");
    vif_session_t *b = open_session("usb");

    vif_session_set_default_frontend(a, &frontend_a);
    vif_session_set_frontend(a, &frontend_a);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(a, VIF_BUS_CAN, &can500));

    /* The BLE client walks away. */
    vif_session_link_down(a);

    /* The USB link asks for the same bus. This is the case that looked like a
     * dead vehicle: refused here, the ELM327 front-end answers '?' to AT SP
     * and then NO DATA to everything after it. */
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(b, VIF_BUS_CAN, &can500));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_CAN, vif_bus_current(b, VIF_BUS_CAN));
}

TEST(a_link_already_on_its_default_is_still_rebuilt) {
    vif_session_t *s = open_session("usb");

    vif_session_set_default_frontend(s, &frontend_a);
    vif_session_set_frontend(s, &frontend_a);

    vif_session_link_down(s);
    vif_session_link_down(s);

    /*
     * Once per departing client, and on a link that never switched grammar at
     * all - which is every link an ordinary OBD-II app ever touches.
     *
     * This used to return early whenever the default was already installed,
     * on the grounds that a tool toggling DTR should not have its settings
     * reset underneath it. That is the smaller of the two harms. A front-end
     * holds the state of one conversation: the line half assembled when the
     * client vanished, the echo and header settings it chose, the protocol it
     * found. Leaving that standing hands the next client a parser mid
     * sentence - its first command arrives appended to the last one's
     * fragment, is rejected as one line rather than answered as two, and an
     * ELM327 client that is one prompt short never catches up.
     */
    TEST_ASSERT_EQUAL_INT(3, fe_a.created);
    TEST_ASSERT_EQUAL_INT(2, fe_a.destroyed);
}

TEST(a_link_with_no_default_keeps_what_it_is_running) {
    vif_session_t *s = open_session("usb");

    vif_session_set_frontend(s, &frontend_b);
    vif_session_link_down(s);

    TEST_ASSERT_EQUAL_STRING("fe_b", vif_session_frontend_name(s));
    TEST_ASSERT_EQUAL_INT(0, fe_b.destroyed);
}

/* ------------------------------------------------------------------ *
 * Recovery
 * ------------------------------------------------------------------ */

TEST(recovering_releases_every_claim_and_restores_every_default) {
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

    TEST_ASSERT_EQUAL_INT(VIF_BUS_CAN, vif_bus_current(ble, VIF_BUS_CAN));
    TEST_ASSERT_EQUAL_INT(1, fake_board_hs_state(OBD_PIN_HS));

    TEST_ASSERT_TRUE(vif_session_service(usb));
    TEST_ASSERT_TRUE(vif_session_service(ble));

    /* Asking again once it is done finds nothing to do. */
    TEST_ASSERT_FALSE(vif_session_service(usb));

    TEST_ASSERT_EQUAL_STRING("fe_a", vif_session_frontend_name(usb));
    TEST_ASSERT_EQUAL_STRING("fe_a", vif_session_frontend_name(ble));
    TEST_ASSERT_EQUAL_INT(0, fake_board_hs_state(OBD_PIN_HS));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE, vif_bus_current(ble, VIF_BUS_CAN));
    TEST_ASSERT_FALSE(fake_can_is_up());
}

/* ------------------------------------------------------------------ *
 * Reporting what is live
 * ------------------------------------------------------------------ */

/** @brief The reported claim for @p bus, or NULL when its group is idle. */
static const vif_bus_claim_t *claim_for(const vif_bus_claim_t *claims,
                                        vif_bus_t bus) {
    for (size_t i = 0; i < VIF_BUS_GROUPS; i++) {
        if (claims[i].bus == bus) {
            return &claims[i];
        }
    }

    return NULL;
}

TEST(the_bus_report_names_each_bus_its_rate_and_its_holder) {
    vif_session_t *s = open_session("usb");
    vif_bus_claim_t claims[VIF_BUS_GROUPS];
    const vif_bus_claim_t *can;

    vif_bus_info(claims);
    for (size_t i = 0; i < VIF_BUS_GROUPS; i++) {
        TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE, claims[i].bus);
        TEST_ASSERT_EQUAL_STRING("", claims[i].owner);
    }

    vif_bus_open(s, VIF_BUS_CAN, &can500);
    vif_bus_open(s, VIF_BUS_KLINE, NULL);
    vif_bus_info(claims);

    /* Without this a client refused a bus learns only that it was refused,
     * not who has it. */
    can = claim_for(claims, VIF_BUS_CAN);
    TEST_ASSERT_NOT_NULL(can);
    TEST_ASSERT_EQUAL_INT(500000, (int)can->bitrate);
    TEST_ASSERT_EQUAL_STRING("usb", can->owner);
    TEST_ASSERT_EQUAL_STRING("CAN", vif_bus_name(can->bus));

    /* Both are reported, because both are live. */
    TEST_ASSERT_NOT_NULL(claim_for(claims, VIF_BUS_KLINE));

    vif_bus_close(s, VIF_BUS_CAN);
    vif_bus_info(claims);
    TEST_ASSERT_NULL(claim_for(claims, VIF_BUS_CAN));
    TEST_ASSERT_NOT_NULL(claim_for(claims, VIF_BUS_KLINE));
}

TEST(failed_j1850_open_retains_claim_when_cleanup_fails) {
    const vif_bus_t buses[] = {VIF_BUS_J1850_PWM, VIF_BUS_J1850_VPW};
    const fake_bus_id_t ids[] = {FAKE_BUS_J1850_PWM, FAKE_BUS_J1850_VPW};
    vif_session_t *owner = open_session("owner");
    vif_session_t *other = open_session("other");
    for (unsigned i = 0; i < 2; i++) {
        fake_bus_open_result(ids[i], ESP_FAIL);
        fake_bus_close_result(ids[i], ESP_ERR_TIMEOUT);
        TEST_ASSERT_EQUAL_INT(ESP_FAIL, vif_bus_open(owner, buses[i], NULL));
        TEST_ASSERT_EQUAL_INT(buses[i], vif_bus_current(owner, buses[i]));
        TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                              vif_bus_open(other, buses[1 - i], NULL));
        TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT,
                              vif_bus_open(owner, buses[1 - i], NULL));
        fake_bus_close_result(ids[i], ESP_OK);
        TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_close(owner, buses[i]));
        TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE, vif_bus_current(owner, buses[i]));
        fake_bus_open_result(ids[i], ESP_OK);
    }
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(other, VIF_BUS_J1850_PWM, NULL));
}

TEST(failed_j1850_open_releases_claim_after_successful_cleanup) {
    vif_session_t *owner = open_session("owner");
    vif_session_t *other = open_session("other");
    fake_bus_open_result(FAKE_BUS_J1850_PWM, ESP_FAIL);
    TEST_ASSERT_EQUAL_INT(ESP_FAIL,
                          vif_bus_open(owner, VIF_BUS_J1850_PWM, NULL));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE,
                          vif_bus_current(owner, VIF_BUS_J1850_PWM));
    TEST_ASSERT_EQUAL_INT(1, fake_bus_teardown_count(FAKE_BUS_J1850_PWM));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(other, VIF_BUS_J1850_VPW, NULL));
}

typedef struct {
    vif_session_t *owner;
    vif_session_t *other;
    int calls;
} lifecycle_probe_t;

static void probe_lifecycle(void *arg) {
    lifecycle_probe_t *p = arg;
    TEST_ASSERT_EQUAL_INT(0, idf_stub_lock_balance());
    TEST_ASSERT_EQUAL_INT(VIF_BUS_J1850_PWM,
                          vif_bus_current(p->owner, VIF_BUS_J1850_PWM));
    TEST_ASSERT_FALSE(vif_bus_is_open(p->owner, VIF_BUS_J1850_PWM));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          vif_bus_open(p->other, VIF_BUS_J1850_VPW, NULL));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(p->other, VIF_BUS_CAN, &can500));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_close(p->other, VIF_BUS_CAN));
    p->calls++;
}

TEST(lifecycle_waits_keep_reservation_without_holding_global_mutex) {
    lifecycle_probe_t p = {.owner = open_session("owner"),
                           .other = open_session("other")};
    fake_bus_on_lifecycle(FAKE_BUS_J1850_PWM, probe_lifecycle, &p);
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_bus_open(p.owner, VIF_BUS_J1850_PWM, NULL));
    TEST_ASSERT_TRUE(vif_bus_is_open(p.owner, VIF_BUS_J1850_PWM));
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_bus_open(p.owner, VIF_BUS_J1850_PWM, NULL));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_close(p.owner, VIF_BUS_J1850_PWM));
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_bus_open(p.owner, VIF_BUS_J1850_PWM, NULL));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_release_all(p.owner));
    TEST_ASSERT_EQUAL_INT(6, p.calls);
    fake_bus_on_lifecycle(FAKE_BUS_J1850_PWM, NULL, NULL);
}

TEST(failed_startup_cleanup_keeps_reservation_without_global_mutex) {
    lifecycle_probe_t p = {.owner = open_session("owner"),
                           .other = open_session("other")};
    fake_bus_on_lifecycle(FAKE_BUS_J1850_PWM, probe_lifecycle, &p);
    fake_bus_open_result(FAKE_BUS_J1850_PWM, ESP_FAIL);
    fake_bus_close_result(FAKE_BUS_J1850_PWM, ESP_ERR_TIMEOUT);
    TEST_ASSERT_EQUAL_INT(ESP_FAIL,
                          vif_bus_open(p.owner, VIF_BUS_J1850_PWM, NULL));
    TEST_ASSERT_EQUAL_INT(2, p.calls);
    TEST_ASSERT_FALSE(vif_bus_is_open(p.owner, VIF_BUS_J1850_PWM));
    fake_bus_close_result(FAKE_BUS_J1850_PWM, ESP_OK);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_release_all(p.owner));
    TEST_ASSERT_EQUAL_INT(3, p.calls);
    fake_bus_on_lifecycle(FAKE_BUS_J1850_PWM, NULL, NULL);
}

TEST(retained_close_failure_rejects_io_until_reopened) {
    vif_session_t *s = open_session("owner");
    uint8_t data[] = {0x81, 0x01};
    bus_msg_t msg = {.data = data, .len = sizeof(data), .cap = sizeof(data)};
    uint32_t value;
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(s, VIF_BUS_J1850_PWM, NULL));
    fake_bus_close_result(FAKE_BUS_J1850_PWM, ESP_ERR_TIMEOUT);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT, vif_bus_close(s, VIF_BUS_J1850_PWM));
    TEST_ASSERT_FALSE(vif_bus_is_open(s, VIF_BUS_J1850_PWM));
    TEST_ASSERT_EQUAL_INT(VIF_ERR_NO_CLAIM,
                          vif_bus_send(s, VIF_BUS_J1850_PWM, &msg, 0));
    TEST_ASSERT_EQUAL_INT(VIF_ERR_NO_CLAIM,
                          vif_bus_recv(s, VIF_BUS_J1850_PWM, &msg, 0));
    TEST_ASSERT_EQUAL_INT(
        VIF_ERR_NO_CLAIM,
        vif_bus_param_get(s, VIF_BUS_J1850_PWM, BUS_P_DATA_RATE, &value));
    TEST_ASSERT_EQUAL_INT(
        VIF_ERR_NO_CLAIM,
        vif_bus_param_set(s, VIF_BUS_J1850_PWM, BUS_P_DATA_RATE, 41600));
    TEST_ASSERT_EQUAL_INT(0, fake_bus_sent_count(FAKE_BUS_J1850_PWM));
    fake_bus_close_result(FAKE_BUS_J1850_PWM, ESP_OK);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_open(s, VIF_BUS_J1850_PWM, NULL));
    TEST_ASSERT_TRUE(vif_bus_is_open(s, VIF_BUS_J1850_PWM));
    TEST_ASSERT_EQUAL_INT(0, vif_bus_send(s, VIF_BUS_J1850_PWM, &msg, 0));
}
