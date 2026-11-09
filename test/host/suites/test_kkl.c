/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_kkl.c
 * @brief The KKL cable front-end: what reaches the K-Line, and in what order.
 *
 * A host's serial settings and its data reach the adapter by different USB
 * pipes, so the order in which the link sees them is not the order the host
 * sent them in. These tests pin the order the K-Line sees: each setting lands
 * between the two bytes it was sent between.
 *
 * The front-end is driven through its own vtable, the calls the link task
 * makes, because the host stub never starts that task.
 */

#include <string.h>

#include "td_test.h"

#include "freertos/semphr.h"
#include "freertos/task.h"

#include "fake_bus.h"
#include "fake_can_bus.h"
#include "fake_clock.h"
#include "fake_led.h"
#include "fake_port.h"

#include "kkl.h"
#include "vif.h"

static comm_port_id_t g_port = COMM_INVALID_PORT_ID;

/** @brief The host wrote @p s; the USB task hands it to the port. */
static void host_writes(const char *s) {
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          comm_port_rx(g_port, (const uint8_t *)s, strlen(s)));
}

/** @brief The link task's loop body: read what arrived, feed it, poll. */
static void link_runs(void) {
    uint8_t buf[64];
    size_t n;

    while ((n = comm_port_read(g_port, buf, sizeof(buf), 0)) > 0) {
        kkl_frontend.feed(buf, n);
    }
    kkl_frontend.poll();
}

/** @brief What reached the K-Line since kkl_frontend.start(). */
static const char *wire(void) {
    const char *log = fake_bus_kline_log();
    const char *raw = strstr(log, "raw=1 ");

    TEST_ASSERT_MSG(raw != NULL, "raw mode was never set: '%s'", log);
    /* Skip start()'s own settings: "raw=1 bits=0 parity=0 baud=10400 ". */
    const char *after = strstr(raw, "baud=");
    after = strchr(after, ' ') + 1;
    return after;
}

static void harness_init_once(void) {
    static bool done;

    if (done) {
        return;
    }
    done = true;

    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_iface_init());
    fake_port_register_all();
    g_port = fake_port_id(0);
    kkl_register();
}

void td_setup(void) {
    uint8_t drain[64];

    harness_init_once();

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_init());
    fake_clock_reset();
    fake_led_reset();
    fake_can_reset();
    fake_bus_reset_all();

    while (comm_port_read(g_port, drain, sizeof(drain), 0) > 0) {
        ;
    }
    fake_port_reset();

    idf_stub_set_created_task(xTaskGetCurrentTaskHandle());
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_link_start(&kkl_frontend));
    vif_shell_bind();
    vif_link_set_port(g_port);

    /* Every test starts from the default coding a fresh host would see. */
    kkl_line_coding(g_port, 10400, 8, 0);
}

void td_teardown(void) {
    TEST_ASSERT_MSG(idf_stub_lock_balance() == 0, "left %d locks held",
                    idf_stub_lock_balance());
}

/* ------------------------------------------------------------------ *
 * Starting
 * ------------------------------------------------------------------ */

TEST(start_puts_the_k_line_in_raw_mode_at_the_last_coding) {
    kkl_line_coding(g_port, 9600, 7, 1);
    kkl_break(g_port, 0xFFFF); /* before the grammar ran: not for it */

    TEST_ASSERT_TRUE(kkl_frontend.start());

    TEST_ASSERT_EQUAL_STRING("raw=1 bits=1 parity=1 baud=9600 ",
                             fake_bus_kline_log());
    link_runs();
    TEST_ASSERT_EQUAL_STRING("", wire());
}

TEST(a_k_line_that_refuses_raw_mode_is_closed_again) {
    fake_bus_kline_raw_result(BUS_ERR_BUS_BUSY);

    TEST_ASSERT_FALSE(kkl_frontend.start());

    /* The grammar vif falls back to must open it afresh. */
    TEST_ASSERT_FALSE(fake_bus_is_up(FAKE_BUS_KLINE));
    TEST_ASSERT_EQUAL_INT(VIF_BUS_NONE,
                          vif_bus_current(VIF_OWNER_LINK, VIF_BUS_KLINE));
}

/* ------------------------------------------------------------------ *
 * Settings land between the bytes they were sent between
 * ------------------------------------------------------------------ */

TEST(a_5_baud_address_goes_out_at_5_baud_however_fast_9600_follows) {
    TEST_ASSERT_TRUE(kkl_frontend.start());

    /* cfsetspeed(5); write(0x01); tcdrain(); cfsetspeed(9600) - and the 9600
     * reaches the adapter before the link has read the byte. */
    kkl_line_coding(g_port, 5, 7, 1);
    host_writes("\x01");
    kkl_line_coding(g_port, 9600, 8, 0);

    link_runs();

    TEST_ASSERT_EQUAL_STRING(
        "bits=1 parity=1 baud=5 tx:01 bits=0 parity=0 baud=9600 ", wire());
}

TEST(a_break_between_two_bytes_of_one_read_splits_the_send) {
    TEST_ASSERT_TRUE(kkl_frontend.start());

    host_writes("AB");
    kkl_break(g_port, 0xFFFF);
    host_writes("C");
    kkl_break(g_port, 0);

    link_runs();

    TEST_ASSERT_EQUAL_STRING("tx:4142 break=1 tx:43 break=0 ", wire());
}

TEST(a_setting_waits_for_the_bytes_written_before_it) {
    TEST_ASSERT_TRUE(kkl_frontend.start());

    host_writes("\x55");
    kkl_line_coding(g_port, 9600, 8, 0);

    /* A poll before the link has read the byte must not jump the queue. */
    kkl_frontend.poll();
    TEST_ASSERT_EQUAL_STRING("", wire());

    link_runs();
    TEST_ASSERT_EQUAL_STRING("tx:55 bits=0 parity=0 baud=9600 ", wire());
}

TEST(a_setting_from_another_port_applies_at_once) {
    TEST_ASSERT_TRUE(kkl_frontend.start());

    host_writes("\x55"); /* unread on the link's port */
    kkl_line_coding(fake_port_id(1), 4800, 8, 0);

    kkl_frontend.poll();
    TEST_ASSERT_EQUAL_STRING("bits=0 parity=0 baud=4800 ", wire());
}

TEST(a_timed_break_holds_then_releases_the_line) {
    TEST_ASSERT_TRUE(kkl_frontend.start());
    uint64_t t0 = fake_clock_ms();

    kkl_break(g_port, 25);
    link_runs();

    TEST_ASSERT_EQUAL_STRING("break=1 break=0 ", wire());
    TEST_ASSERT_TRUE(fake_clock_ms() - t0 >= 25);
}

/* ------------------------------------------------------------------ *
 * The other direction
 * ------------------------------------------------------------------ */

TEST(what_the_k_line_carries_goes_back_to_the_host_as_is) {
    static const uint8_t reply[] = {0x55, 0x01, 0x8A};

    TEST_ASSERT_TRUE(kkl_frontend.start());
    fake_bus_stage_response(FAKE_BUS_KLINE, reply, sizeof(reply), 0);

    host_writes("\x01");
    link_runs();

    TEST_ASSERT_EQUAL_INT(sizeof(reply), fake_port_len(0));
    TEST_ASSERT_EQUAL_MEM(reply, fake_port_text(0), sizeof(reply));
}
