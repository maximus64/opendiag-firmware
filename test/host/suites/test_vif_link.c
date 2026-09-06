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
