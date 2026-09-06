/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file elm327_harness.h
 * @brief Shared fixture for the ELM327 suites.
 *
 * Include this instead of elm327_at.c. It pulls the firmware source into the
 * test translation unit, which is what gives the tests reach into the file's
 * statics: everything worth testing there is static, and the only public entry
 * point starts a task that never returns.
 *
 * Because the source is included rather than linked, each suite is its own
 * executable with its own copy of the module state. That also means a suite
 * cannot leak settings into another suite.
 *
 * The path a command travels here is the production one:
 *
 *   comm_port_rx()  ->  port rx ring buffer  ->  comm_port_read()
 *     ->  elm327_feed_byte()  ->  elm327_parse_command()  ->  real vif.c
 *     ->  bus fakes
 *     ->  tx ring buffer  ->  elm327_uart_flush()  ->  comm_port_write()
 *     ->  fake port capture
 *
 * The only thing standing in for the vif session task is the pump loop below:
 * read what the transport delivered, hand it to the front-end a byte at a
 * time. vif.c itself is real, so the claims the tests exercise - the bus, the
 * connector pins - are arbitrated exactly as they are on the device.
 */

#pragma once

#include <string.h>

#include "td_test.h"

#include "freertos/semphr.h"

#include "fake_board.h"
#include "fake_bus.h"
#include "fake_can_bus.h"
#include "fake_clock.h"
#include "fake_led.h"
#include "fake_port.h"

#include "elm327_at.c"

/** The ELM327 identity string the firmware reports, for use in expectations. */
#define ELM_ID ELM_IDENTIFY

/** Every response ends with a blank line and the prompt. */
#define ELM_PROMPT "\r>"

/* Not every suite reaches for every helper below. */
#define TD_MAYBE_UNUSED __attribute__((unused))

/** The interpreter. One instance, so this is just a shorter name for it. */
static elm327_ctx_t *const g_elm = &g_elm327;

/* ------------------------------------------------------------------ *
 * Driving the adapter
 * ------------------------------------------------------------------ */

/**
 * @brief Runs the task body over whatever the transport has delivered.
 *
 * In chunks of the size the link task reads, and through the front-end's own
 * feed(), rather than a byte at a time into elm327_feed_byte(). Where the
 * chunk boundaries fall is not a detail: a reset discards the rest of the
 * chunk it arrived in, so a harness that fed one byte per call would have
 * nothing left to discard and would quietly pass a test about it.
 */
static TD_MAYBE_UNUSED void elm_pump(void) {
    uint8_t buf[64];
    size_t n;

    while ((n = vif_link_read(buf, sizeof(buf), 0)) > 0) {
        elm327_fe_feed(buf, n);
    }
}

/** Delivers @p s on fake port @p idx and lets the adapter process it. */
static TD_MAYBE_UNUSED void elm_send_on(int idx, const char *s) {
    TEST_ASSERT_MSG(comm_port_rx(fake_port_id(idx), (const uint8_t *)s,
                                 strlen(s)) == ESP_OK,
                    "transport refused %zu bytes", strlen(s));
    elm_pump();
}

/** Delivers @p s on the first fake port, standing in for USB CDC 0. */
static TD_MAYBE_UNUSED void elm_send(const char *s) { elm_send_on(0, s); }

/** The client went away: what main.c asks for, plus the link task's next pass.
 */
static TD_MAYBE_UNUSED void elm_link_down(void) {
    vif_link_down();
    vif_link_service();
}

/**
 * @brief Sends one command line and returns everything said back.
 *
 * Earlier output is discarded first, so an expectation covers exactly this
 * exchange. Echo is on by default, so the reply starts with the command
 * itself unless the test turned echo off.
 */
static TD_MAYBE_UNUSED const char *elm_ask(const char *cmd) {
    fake_port_reset();
    elm_send(cmd);

    return fake_port_text(0);
}

/**
 * @brief Turns echo off so later expectations hold only the adapter's words.
 *
 * Echo is still on while this command itself is being read, so the reply to it
 * carries the command back.
 */
static TD_MAYBE_UNUSED void elm_echo_off(void) {
    TEST_ASSERT_EQUAL_STRING("ATE0\rOK\r" ELM_PROMPT, elm_ask("ATE0\r"));
}

/** Sends @p cmd and asserts the adapter accepted it. Echo must be off. */
static TD_MAYBE_UNUSED void elm_ok(const char *cmd) {
    TEST_ASSERT_EQUAL_STRING("OK\r" ELM_PROMPT, elm_ask(cmd));
}

/** What a slow initiation prints before the exchange it precedes. */
#define ELM_BUS_INIT "BUS INIT: ...OK\r"

/**
 * @brief Select a K-Line protocol and mark the link active without a handshake.
 *
 * AT BI is the datasheet's own way of doing this, and it keeps the initiation
 * out of expectations that are about the exchange rather than about how the
 * session was opened. The tests that *are* about the initiation let it run.
 */
static TD_MAYBE_UNUSED void elm_kline_select(const char *sp_cmd) {
    elm_ok(sp_cmd);
    elm_ok("ATBI\r");
}

/* ------------------------------------------------------------------ *
 * Fixture
 * ------------------------------------------------------------------ */

static void elm_harness_init_once(void) {
    static bool done;

    if (done) {
        return;
    }
    done = true;

    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_init());
    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_iface_init());

    fake_port_register_all();

    /* The same wiring main.c does: the link over one port, with ELM327 as the
     * grammar it starts on and reverts to. The link task is recorded by the
     * stub and never started, and this thread stands in for it - so it is this
     * thread the stub hands back as the created one. elm_pump() is its body.
     */
    elm327_register();

    idf_stub_set_created_task(xTaskGetCurrentTaskHandle());
    TEST_ASSERT_EQUAL_INT(ESP_OK, vif_link_start(&elm327_frontend));

    vif_link_set_port(fake_port_id(0));

    /* The default is queued for the link task, and this thread is that task. */
    vif_link_service();

    TEST_ASSERT_MSG(vif_link_frontend_name() != NULL,
                    "the ELM327 front-end did not start");
}

void td_setup(void) {
    uint8_t drain[64];

    elm_harness_init_once();

    /* Return the settings to power-on defaults before clearing the fakes, so
     * any bus teardown this triggers is not mistaken for the test's own. */
    elm327_reset(g_elm);

    fake_clock_reset();
    fake_board_reset();
    fake_led_reset();
    fake_can_reset();
    fake_bus_reset_all();
    fake_port_reset();

    /* Discard anything a previous test left in flight. */
    while (vif_link_read(drain, sizeof(drain), 0) > 0) {
        ;
    }
    elm327_uart_flush(g_elm);
    fake_port_reset();

    memset(&g_elm->line, 0, sizeof(g_elm->line));
}

void td_teardown(void) {
    TEST_ASSERT_MSG(idf_stub_lock_balance() == 0, "%d locks were left held",
                    idf_stub_lock_balance());
}
