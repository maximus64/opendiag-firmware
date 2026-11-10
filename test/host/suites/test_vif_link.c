/* SPDX-License-Identifier: GPL-3.0-only */
#include <setjmp.h>
#include <string.h>

#include "fake_bus.h"
#include "fake_can_bus.h"
#include "fake_clock.h"
#include "fake_port.h"
#include "td_test.h"

/* Run the actual loop; poll() returns control to the test via longjmp. */
#include "vif.c"

static jmp_buf loop_done;
static comm_port_id_t port;
static void (*request_during_feed)(void);
static int old_chunks, new_bytes, old_stops, new_starts, old_polls;
static bool second_chunk_bus_open;
static int other_task;

static bool old_start(void) {
    const vif_bus_cfg_t cfg = {.bitrate = 500000};
    return vif_bus_open(VIF_OWNER_LINK, VIF_BUS_CAN, &cfg) == ESP_OK;
}

static void old_feed(const uint8_t *data, size_t len) {
    old_chunks++;
    if (old_chunks == 1) {
        TaskHandle_t owner = xTaskGetCurrentTaskHandle();
        idf_stub_set_current_task(&other_task);
        request_during_feed();
        idf_stub_set_current_task(owner);
    } else {
        second_chunk_bus_open = vif_bus_is_open(VIF_OWNER_LINK, VIF_BUS_CAN);
    }
}

static void old_stop(void) { old_stops++; }

static void old_poll(void) {
    old_polls++;
    longjmp(loop_done, 1);
}

static bool new_start(void) {
    new_starts++;
    return true;
}

static void new_feed(const uint8_t *data, size_t len) { new_bytes += (int)len; }

static void new_poll(void) { longjmp(loop_done, 1); }

static const vif_frontend_t old_frontend = {
    .name = "old",
    .start = old_start,
    .feed = old_feed,
    .poll = old_poll,
    .stop = old_stop,
};

static const vif_frontend_t new_frontend = {
    .name = "new",
    .start = new_start,
    .feed = new_feed,
    .poll = new_poll,
};

void td_setup(void) {
    static bool ports_registered;

    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_iface_init());
    if (!ports_registered) {
        fake_port_register_all();
        ports_registered = true;
    }
    port = fake_port_id(0);
    TEST_ASSERT_TRUE(port != COMM_INVALID_PORT_ID);
    comm_port_client_gone(port);

    idf_stub_set_current_task(NULL);
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_init());
    fake_bus_reset_all();
    fake_can_reset();
    fake_clock_reset();
    fake_port_reset();
    old_chunks = new_bytes = old_stops = new_starts = old_polls = 0;
    second_chunk_bus_open = false;

    idf_stub_set_created_task(xTaskGetCurrentTaskHandle());
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_link_start(&new_frontend));
    vif_link_set_frontend(&old_frontend);
    vif_link_set_port(port);
    /* Those requests woke the task; each test starts from an idle link. */
    idf_stub_notify_reset();
}

void td_teardown(void) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_release_all(VIF_OWNER_LINK));
    TEST_ASSERT_EQUAL_INT(0, idf_stub_lock_balance());
    idf_stub_set_current_task(NULL);
    idf_stub_set_created_task(NULL);
}

static void run_with_request(void (*request)(void)) {
    uint8_t input[65];
    memset(input, 'x', sizeof(input));
    request_during_feed = request;
    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_rx(port, input, sizeof(input)));
    if (setjmp(loop_done) == 0) {
        link_task(NULL);
    }
}

static void switch_frontend(void) { vif_link_set_frontend(&new_frontend); }

static void reconnect(void) {
    vif_link_down();
    vif_link_set_port(COMM_INVALID_PORT_ID);
    vif_link_set_port(port);
    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_rx(port, (const uint8_t *)"N", 1));
}

static void release_bus(void) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_release_all(VIF_OWNER_LINK));
}

static void check_switched(void) {
    TEST_ASSERT_EQUAL_INT(1, old_chunks);
    TEST_ASSERT_EQUAL_INT(1, old_stops);
    TEST_ASSERT_EQUAL_INT(1, new_starts);
    TEST_ASSERT_EQUAL_INT(1, new_bytes);
    TEST_ASSERT_EQUAL_INT(0, old_polls);
    TEST_ASSERT_FALSE(vif_bus_is_open(VIF_OWNER_LINK, VIF_BUS_CAN));
}

TEST(a_switch_during_feed_applies_before_the_next_chunk_or_poll) {
    run_with_request(switch_frontend);
    check_switched();
}

TEST(a_recovery_during_feed_applies_before_the_next_chunk_or_poll) {
    run_with_request(vif_recover_all);
    check_switched();
}

TEST(a_reconnected_clients_input_reaches_the_default_frontend) {
    run_with_request(reconnect);
    check_switched();
}

TEST(a_deferred_release_finishes_before_the_next_chunk) {
    run_with_request(release_bus);
    TEST_ASSERT_EQUAL_INT(2, old_chunks);
    TEST_ASSERT_FALSE(second_chunk_bus_open);
    TEST_ASSERT_EQUAL_INT(0, old_stops);
    TEST_ASSERT_EQUAL_INT(0, new_bytes);
}

TEST(a_frontend_can_observe_late_replies_at_the_idle_poll_cadence) {
    static const vif_frontend_t observer = {
        .name = "observer",
        .poll = old_poll,
        .poll_interval_ms = 50,
    };
    vif_link_set_frontend(&observer);
    idf_stub_notify_reset();
    if (setjmp(loop_done) == 0)
        link_task(NULL);
    TEST_ASSERT_EQUAL_INT(1, old_polls);
    TEST_ASSERT_EQUAL_INT(50, fake_clock_ms());
}

TEST(streaming_frontends_keep_the_default_poll_cadence) {
    if (setjmp(loop_done) == 0)
        link_task(NULL);
    TEST_ASSERT_EQUAL_INT(1, old_polls);
    TEST_ASSERT_EQUAL_INT(2, fake_clock_ms());
}

/* ------------------------------------------------------------------ *
 * What ends the link task's sleep
 * ------------------------------------------------------------------ */

TEST(input_on_the_link_port_wakes_the_task) {
    const uint8_t byte = 'x';
    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_rx(fake_port_id(1), &byte, 1));
    TEST_ASSERT_FALSE(idf_stub_notify_pending());
    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_rx(port, &byte, 1));
    TEST_ASSERT_TRUE(idf_stub_notify_pending());
}

TEST(input_dropped_for_a_full_buffer_still_wakes_the_task) {
    uint8_t chunk[FAKE_PORT_RX_BUF + 1];
    memset(chunk, 'x', sizeof(chunk));
    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_rx(port, chunk, sizeof(chunk)));
    TEST_ASSERT_EQUAL_INT(1, comm_port_rx_dropped(port));
    TEST_ASSERT_TRUE(idf_stub_notify_pending());
}

static const vif_bus_cfg_t can_500k = {.bitrate = 500000};

TEST(news_from_a_bus_the_link_holds_wakes_the_link_task) {
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_bus_open(VIF_OWNER_LINK, VIF_BUS_CAN, &can_500k));
    idf_stub_notify_reset();
    bus_notify(can_bus_ops.listener);
    TEST_ASSERT_TRUE(idf_stub_notify_pending());
    TEST_ASSERT_TRUE(idf_stub_notified_task() == g.task[VIF_OWNER_LINK]);
    idf_stub_notify_reset();
    BaseType_t woken = pdFALSE;
    bus_notify_from_isr(can_bus_ops.listener, &woken);
    TEST_ASSERT_TRUE(idf_stub_notified_task() == g.task[VIF_OWNER_LINK]);
}

TEST(a_closed_bus_wakes_nobody) {
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_bus_open(VIF_OWNER_LINK, VIF_BUS_CAN, &can_500k));
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_close(VIF_OWNER_LINK, VIF_BUS_CAN));
    idf_stub_notify_reset();
    bus_notify(can_bus_ops.listener);
    TEST_ASSERT_FALSE(idf_stub_notify_pending());
}

TEST(news_from_a_bus_the_shell_holds_leaves_the_link_asleep) {
    static int shell_thread;
    idf_stub_set_current_task((TaskHandle_t)&shell_thread);
    vif_shell_bind();
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          vif_bus_open(VIF_OWNER_SHELL, VIF_BUS_KLINE, NULL));
    idf_stub_notify_reset();
    bus_notify(kline_bus_ops.listener);
    TEST_ASSERT_FALSE(idf_stub_notify_pending());
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_bus_release_all(VIF_OWNER_SHELL));
    idf_stub_set_current_task(NULL);
}

TEST(control_requests_from_other_tasks_wake_the_task) {
    vif_link_set_frontend(&new_frontend);
    TEST_ASSERT_TRUE(idf_stub_notify_pending());
    idf_stub_notify_reset();
    vif_recover_all();
    TEST_ASSERT_TRUE(idf_stub_notify_pending());
}

static uint32_t wake_due_us;

static bool wake_at_due(uint32_t *due_us) {
    *due_us = wake_due_us;
    return true;
}

/* Runs the loop under a front-end that wants polling at @p in_us from now. */
static void run_with_deadline(int32_t in_us) {
    static const vif_frontend_t timed = {
        .name = "timed",
        .poll = old_poll,
        .next_wake = wake_at_due,
    };
    vif_link_set_frontend(&timed);
    TEST_ASSERT_TRUE(vif_link_service());
    idf_stub_notify_reset();
    wake_due_us = (uint32_t)esp_timer_get_time() + (uint32_t)in_us;
    if (setjmp(loop_done) == 0)
        link_task(NULL);
    TEST_ASSERT_EQUAL_INT(1, old_polls);
    TEST_ASSERT_FALSE(fake_esp_timer_armed(link_timer, NULL));
}

TEST(a_frontend_deadline_ends_the_wait_between_ticks) {
    run_with_deadline(300);
    TEST_ASSERT_EQUAL_INT(0, fake_clock_ms());
    TEST_ASSERT_TRUE((int32_t)((uint32_t)esp_timer_get_time() - wake_due_us) >=
                     0);
}

TEST(a_deadline_already_due_skips_the_wait) {
    run_with_deadline(-1);
    TEST_ASSERT_EQUAL_INT(0, fake_clock_ms());
    TEST_ASSERT_TRUE((int32_t)((uint32_t)esp_timer_get_time() - wake_due_us) <
                     1000);
}

TEST(a_deadline_inside_the_backstop_is_met_on_time) {
    run_with_deadline(10000);
    TEST_ASSERT_EQUAL_INT(10, fake_clock_ms());
}

TEST(a_deadline_beyond_the_backstop_waits_only_the_backstop) {
    run_with_deadline(500000);
    TEST_ASSERT_EQUAL_INT(VIF_BACKSTOP_MS, fake_clock_ms());
}

/* ------------------------------------------------------------------ *
 * The backstop: what slips past next_wake and the events
 * ------------------------------------------------------------------ */

static uint32_t work_done;
static bool poll_works;
static int counted_polls;

static uint32_t count_progress(void) { return work_done; }

/* Does its pass, then ends the loop on the next one. */
static void counted_poll(void) {
    if (counted_polls++)
        longjmp(loop_done, 1);
    if (poll_works)
        work_done++;
}

static bool nothing_due(uint32_t *due_us) {
    (void)due_us;
    return false;
}

static void run_counted(bool works, bool woken) {
    static const vif_frontend_t counted = {
        .name = "counted",
        .poll = counted_poll,
        .next_wake = nothing_due,
        .progress = count_progress,
    };
    vif_link_set_frontend(&counted);
    TEST_ASSERT_TRUE(vif_link_service());
    idf_stub_notify_reset();
    counted_polls = 0;
    poll_works = works;
    if (woken)
        xTaskNotifyGive(g.task[VIF_OWNER_LINK]);
    if (setjmp(loop_done) == 0)
        link_task(NULL);
}

TEST(a_frontend_with_nothing_due_sleeps_until_the_backstop) {
    run_counted(false, false);
    TEST_ASSERT_EQUAL_INT(2 * VIF_BACKSTOP_MS, fake_clock_ms());
    TEST_ASSERT_EQUAL_INT(1, g.link.backstop_polls);
    TEST_ASSERT_EQUAL_INT(0, g.link.backstop_work);
}

TEST(work_found_by_a_backstop_poll_is_counted) {
    run_counted(true, false);
    TEST_ASSERT_EQUAL_INT(1, g.link.backstop_polls);
    TEST_ASSERT_EQUAL_INT(1, g.link.backstop_work);
}

TEST(work_after_an_event_is_not_the_backstops) {
    run_counted(true, true);
    TEST_ASSERT_EQUAL_INT(0, g.link.backstop_polls);
    TEST_ASSERT_EQUAL_INT(0, g.link.backstop_work);
}
