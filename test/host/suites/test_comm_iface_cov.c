/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_comm_iface.c
 * @brief The transport table.
 *
 * Originally ported from main/comm_iface_test.c, which ran from the debug
 * shell and could not actually pass: comm_iface holds COMM_MAX_PORTS (4) port
 * slots, main.c permanently filled three of them, and ports are never
 * unregistered, so two mock ports never fit. Here the table starts empty for
 * every test, which also makes the slot exhaustion path testable.
 *
 * The service layer this suite used to cover is gone. It existed to map
 * several ports onto one protocol handler, and once every link got its own
 * session and its own front-end instance the mapping was one-to-one and the
 * layer carried nothing. The cases for binding, last-writer-wins reply
 * routing and broadcast were deleted rather than rewritten: there is no
 * behaviour left behind them.
 *
 * The firmware source is included rather than linked so the fixture can reset
 * g_comm between tests. comm_iface has no teardown API by design - ports are
 * registered once at startup and never removed - so a test that wants a clean
 * table has to reach in and clear it.
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "td_test.h"

#include "freertos/semphr.h"
#include "fake_port.h"

/* comm_iface_init() takes its lock from xSemaphoreCreateMutex(). That one call
 * is routed through test_create_mutex() so a test can refuse it. */
static bool refuse_mutex;
static char mutex_token;
static SemaphoreHandle_t test_create_mutex(void) {
    return refuse_mutex ? NULL : (SemaphoreHandle_t)&mutex_token;
}
#define xSemaphoreCreateMutex test_create_mutex
#include "comm_iface.c"
#undef xSemaphoreCreateMutex

#define P0 (fake_port_id(0))
#define P1 (fake_port_id(1))

/** Delivers @p s to @p port and drains it into @p buf. Returns bytes read. */
static size_t round_trip(comm_port_id_t port, const char *s, uint8_t *buf,
                         size_t cap) {
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          comm_port_rx(port, (const uint8_t *)s, strlen(s)));

    return comm_port_read(port, buf, cap, pdMS_TO_TICKS(100));
}

void td_setup(void) {
    /* Release the buffers the previous test's ports owned, then start from an
     * empty table. */
    for (int i = 0; i < COMM_MAX_PORTS; i++) {
        if (g_comm.ports[i].active) {
            vRingbufferDeleteWithCaps(g_comm.ports[i].rx_stream);
        }
    }
    memset(&g_comm, 0, sizeof(g_comm));

    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_iface_init());

    fake_port_register_all();
    TEST_ASSERT_MSG(P0 != COMM_INVALID_PORT_ID && P1 != COMM_INVALID_PORT_ID,
                    "the fake ports did not register");
}

void td_teardown(void) {
    TEST_ASSERT_MSG(idf_stub_lock_balance() == 0, "left %d locks held",
                    idf_stub_lock_balance());
}

/* ------------------------------------------------------------------ *
 * Receiving
 * ------------------------------------------------------------------ */

TEST(received_bytes_reach_the_reader) {
    uint8_t buf[32] = {0};
    size_t n = round_trip(P0, "ATZ\r", buf, sizeof(buf));

    TEST_ASSERT_EQUAL_INT(4, (int)n);
    TEST_ASSERT_EQUAL_STRING("ATZ\r", (const char *)buf);
}

TEST(a_read_returns_nothing_when_the_transport_is_idle) {
    uint8_t buf[8];

    TEST_ASSERT_EQUAL_INT(
        0, comm_port_read(P0, buf, sizeof(buf), pdMS_TO_TICKS(10)));
}

TEST(a_port_does_not_see_traffic_from_another_port) {
    uint8_t buf[16];

    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          comm_port_rx(P1, (const uint8_t *)"hello", 5));

    TEST_ASSERT_EQUAL_INT(0, comm_port_read(P0, buf, sizeof(buf), 0));
    TEST_ASSERT_EQUAL_INT(5, (int)comm_port_read(P1, buf, sizeof(buf), 0));
}

TEST(an_overflowing_receive_buffer_drops_and_counts_instead_of_blocking) {
    uint8_t big[FAKE_PORT_RX_BUF];
    uint8_t buf[64];

    memset(big, 'A', sizeof(big));

    /* Fill it, then push one more chunk that cannot fit. The transport's task
     * must come straight back rather than wait for the reader. */
    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_rx(P0, big, sizeof(big) - 8));
    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_rx(P0, big, sizeof(big) - 8));

    TEST_ASSERT_EQUAL_INT(1, (int)comm_port_rx_dropped(P0));

    /* What did fit is still readable: the drop costs the new chunk, not the
     * buffer. */
    TEST_ASSERT_MSG(comm_port_read(P0, buf, sizeof(buf), 0) > 0,
                    "the buffered bytes were lost with the dropped chunk");
}

/* ------------------------------------------------------------------ *
 * Writing
 * ------------------------------------------------------------------ */

TEST(a_write_goes_out_the_port_it_names) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_write(P0, "X\r", 2));

    TEST_ASSERT_EQUAL_STRING("X\r", fake_port_text(0));
    TEST_ASSERT_EQUAL_INT(0, (int)fake_port_len(1));
}

TEST(a_flush_reaches_the_transport) {
    comm_port_flush(P0);

    TEST_ASSERT_EQUAL_INT(1, fake_port_flush_count(0));
    TEST_ASSERT_EQUAL_INT(0, fake_port_flush_count(1));
}

TEST(a_short_write_is_completed_through_a_flush) {
    /* A slow host leaves room for only part of the response in the FIFO. */
    fake_port_set_write_limit(0, 4);

    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_write(P0, "ABCDEFGH", 8));

    TEST_ASSERT_EQUAL_STRING("ABCDEFGH", fake_port_text(0));
    TEST_ASSERT_MSG(fake_port_flush_count(0) > 0,
                    "the remainder went out without draining the FIFO first");
}

TEST(a_disconnected_port_is_not_written_to) {
    fake_port_set_connected(0, false);

    /* Not merely pointless: the USB FIFO would accept the bytes and nobody
     * would ever drain them. */
    TEST_ASSERT_EQUAL_INT(ESP_ERR_NOT_FOUND, comm_port_write(P0, "Y\r", 2));
    TEST_ASSERT_EQUAL_INT(0, (int)fake_port_len(0));

    fake_port_set_connected(0, true);
    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_write(P0, "Y\r", 2));
    TEST_ASSERT_EQUAL_STRING("Y\r", fake_port_text(0));
}

/* ------------------------------------------------------------------ *
 * Registration
 * ------------------------------------------------------------------ */

TEST(registering_more_ports_than_there_are_slots_fails) {
    static const comm_port_ops_t ops = {.write = NULL};
    comm_port_ops_t writable = ops;
    int registered = 2; /* The two fake ports already hold slots. */

    /* Any non-NULL write is enough; nothing is transmitted here. */
    writable.write = (int (*)(const void *, uint32_t))(void *)&registered;

    while (registered < COMM_MAX_PORTS) {
        TEST_ASSERT_MSG(comm_port_register("X", &writable, 32) !=
                            COMM_INVALID_PORT_ID,
                        "slot %d should still have been free", registered);
        registered++;
    }

    /* This is what the on-target commtest hit, with three of the four slots
     * permanently held. */
    TEST_ASSERT_EQUAL_INT(COMM_INVALID_PORT_ID,
                          comm_port_register("OVER", &writable, 32));
}

TEST(a_port_needs_a_write_function) {
    static const comm_port_ops_t no_write = {.write = NULL};

    TEST_ASSERT_EQUAL_INT(COMM_INVALID_PORT_ID,
                          comm_port_register("BAD", &no_write, 32));
    TEST_ASSERT_EQUAL_INT(COMM_INVALID_PORT_ID,
                          comm_port_register("BAD", NULL, 32));
}

TEST(a_port_needs_a_receive_buffer) {
    static const comm_port_ops_t ops = {.write = NULL};
    comm_port_ops_t writable = ops;
    int dummy = 0;

    writable.write = (int (*)(const void *, uint32_t))(void *)&dummy;

    TEST_ASSERT_EQUAL_INT(COMM_INVALID_PORT_ID,
                          comm_port_register("BAD", &writable, 0));
}

TEST(a_name_longer_than_the_limit_is_truncated_not_overrun) {
    static const comm_port_ops_t ops = {.write = NULL};
    comm_port_ops_t writable = ops;
    comm_port_id_t id;
    int dummy = 0;

    writable.write = (int (*)(const void *, uint32_t))(void *)&dummy;

    id = comm_port_register("VERYLONGNAME", &writable, 32);

    TEST_ASSERT_MSG(id != COMM_INVALID_PORT_ID, "registration failed");
    TEST_ASSERT_EQUAL_STRING("VERYLON", g_comm.ports[id].name);
}

TEST(invalid_ids_are_rejected_everywhere) {
    uint8_t buf[8];

    TEST_ASSERT_MSG(
        comm_port_rx(COMM_INVALID_PORT_ID, (const uint8_t *)"x", 1) != ESP_OK,
        "rx accepted an invalid port");
    TEST_ASSERT_EQUAL_INT(
        0, comm_port_read(COMM_INVALID_PORT_ID, buf, sizeof(buf), 0));
    TEST_ASSERT_MSG(comm_port_write(COMM_INVALID_PORT_ID, "x", 1) != ESP_OK,
                    "write accepted an invalid port");
    TEST_ASSERT_EQUAL_INT(0, comm_port_rx_dropped(COMM_INVALID_PORT_ID));

    /* A slot nobody registered is as invalid as the sentinel. */
    TEST_ASSERT_MSG(comm_port_write(COMM_MAX_PORTS - 1, "x", 1) != ESP_OK,
                    "write accepted an unregistered port slot");
}

/* ------------------------------------------------------------------ *
 * A client going away
 *
 * The transport is the only layer that learns about this at the moment it
 * happens. The session task can be several seconds deep in one request when a
 * BLE central drops, and the next central can be connected and sending before
 * that request has even finished - so what the departing client left behind
 * has to be dealt with here rather than whenever the session notices.
 * ------------------------------------------------------------------ */

TEST(a_departing_client_takes_its_unread_input_with_it) {
    uint8_t buf[64] = {0};

    /* A command line the session never got round to reading, and the front of
     * a second one that the client was still sending when it vanished. */
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          comm_port_rx(P0, (const uint8_t *)"ATI\r010", 7));

    comm_port_client_gone(P0);
    comm_port_client_ready(P0);

    /*
     * Nothing of it survives. "010" prepended to the next client's first line
     * turns "ATZ" into "010ATZ", which is one unanswerable line where the
     * client sent two - and an ELM327 client that is a prompt short stays a
     * reply behind for the rest of its session.
     */
    TEST_ASSERT_EQUAL_INT(0, (int)comm_port_read(P0, buf, sizeof(buf), 0));

    TEST_ASSERT_EQUAL_INT(4, (int)round_trip(P0, "ATZ\r", buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_INT(0, memcmp(buf, "ATZ\r", 4));
}

TEST(the_answer_to_a_departed_client_is_not_given_to_the_next_one) {
    fake_port_reset();

    comm_port_client_gone(P0);

    /* Still being composed when the client left, and addressed to nobody. */
    TEST_ASSERT_MSG(comm_port_write(P0, "OK\r\r>", 5) != ESP_OK,
                    "a muted port accepted a write");
    TEST_ASSERT_EQUAL_INT(0, (int)fake_port_len(0));

    /* The front-end that answers the next client says so, and the port talks
     * again. */
    comm_port_client_ready(P0);

    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_write(P0, "ELM327 v2.3\r\r>", 14));
    TEST_ASSERT_EQUAL_STRING("ELM327 v2.3\r\r>", fake_port_text(0));
}

TEST(one_client_leaving_does_not_mute_another_port) {
    fake_port_reset();

    comm_port_client_gone(P0);

    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_write(P1, "OK\r\r>", 5));
    TEST_ASSERT_EQUAL_STRING("OK\r\r>", fake_port_text(1));
}

TEST(what_a_departing_client_left_is_counted) {
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          comm_port_rx(P0, (const uint8_t *)"0100\r", 5));

    comm_port_client_gone(P0);

    /* Silent discards are how a desync stays a mystery; this is the number to
     * look at when one is suspected. */
    TEST_ASSERT_EQUAL_INT(5, (int)g_comm.ports[P0].rx_discarded);
}

TEST(the_backlog_is_what_the_client_is_ahead_by) {
    uint8_t buf[8] = {0};

    TEST_ASSERT_EQUAL_INT(0, (int)comm_port_rx_pending(P0));

    /* Two command lines sent while the session is busy with an earlier one. */
    TEST_ASSERT_EQUAL_INT(
        ESP_OK, comm_port_rx(P0, (const uint8_t *)"0100\r010C\r", 10));
    TEST_ASSERT_EQUAL_INT(10, (int)comm_port_rx_pending(P0));

    /* Reading draws it down, byte for byte. */
    TEST_ASSERT_EQUAL_INT(5, (int)comm_port_read(P0, buf, 5, 0));
    TEST_ASSERT_EQUAL_INT(5, (int)comm_port_rx_pending(P0));

    /* And a client that leaves takes its backlog with it. */
    comm_port_client_gone(P0);
    TEST_ASSERT_EQUAL_INT(0, (int)comm_port_rx_pending(P0));
}

/* ------------------------------------------------------------------ *
 * Coverage: the paths the cases above do not reach
 * ------------------------------------------------------------------ */

/** Captures stdout for the one function whose output is its contract. */
static FILE *g_capture;
static int g_saved_stdout = -1;

static void capture_begin(void) {
    fflush(stdout);
    g_capture = tmpfile();
    TEST_ASSERT_NOT_NULL(g_capture);
    g_saved_stdout = dup(STDOUT_FILENO);
    TEST_ASSERT_TRUE(g_saved_stdout >= 0);
    TEST_ASSERT_TRUE(dup2(fileno(g_capture), STDOUT_FILENO) >= 0);
}

static const char *capture_end(void) {
    static char text[4096];
    size_t length;
    int restored;

    fflush(stdout);
    restored = dup2(g_saved_stdout, STDOUT_FILENO);
    close(g_saved_stdout);
    g_saved_stdout = -1;
    rewind(g_capture);
    length = fread(text, 1, sizeof(text) - 1, g_capture);
    text[length] = '\0';
    fclose(g_capture);
    g_capture = NULL;
    TEST_ASSERT_TRUE(restored >= 0);
    return text;
}

/* A transport with no is_connected: always treated as attached. */
static int plain_write(const void *buf, uint32_t length) {
    (void)buf;
    return (int)length;
}
static const comm_port_ops_t g_plain_ops = {.write = plain_write};

TEST(a_receive_buffer_the_heap_cannot_give_takes_no_slot) {
    /* No heap can supply SIZE_MAX bytes, so the ring buffer create fails. */
    comm_port_id_t id = comm_port_register("HUGE", &g_plain_ops, SIZE_MAX);

    TEST_ASSERT_EQUAL_INT(COMM_INVALID_PORT_ID, id);
    TEST_ASSERT_FALSE(g_comm.ports[2].active);

    /* The slot it would have used is still the next one handed out. */
    TEST_ASSERT_EQUAL_INT(2, comm_port_register("NEXT", &g_plain_ops, 32));
}

TEST(an_uninitialised_table_refuses_io_and_keeps_what_was_received) {
    uint8_t buf[8] = {0};

    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          comm_port_rx(P0, (const uint8_t *)"ABC", 3));

    g_comm.initialized = false;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE,
                          comm_port_rx(P0, (const uint8_t *)"Z", 1));
    TEST_ASSERT_EQUAL_INT(0, (int)comm_port_read(P0, buf, sizeof(buf), 0));
    TEST_ASSERT_EQUAL_INT(0, (int)comm_port_rx_pending(P0));
    TEST_ASSERT_EQUAL_INT(0, (int)comm_port_rx_dropped(P0));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, comm_port_write(P0, "x", 1));
    comm_port_flush(P0);
    comm_port_client_gone(P0);
    TEST_ASSERT_EQUAL_INT(0, fake_port_flush_count(0));
    TEST_ASSERT_EQUAL_INT(0, (int)fake_port_len(0));

    g_comm.initialized = true;

    /* Nothing that happened while the table was down touched the buffer. */
    TEST_ASSERT_EQUAL_INT(3, (int)comm_port_rx_pending(P0));
    TEST_ASSERT_EQUAL_INT(3, (int)comm_port_read(P0, buf, sizeof(buf), 0));
    TEST_ASSERT_EQUAL_INT(0, memcmp(buf, "ABC", 3));
}

TEST(debug_info_says_so_before_the_table_is_up) {
    g_comm.initialized = false;
    capture_begin();
    comm_print_debug_info();
    const char *out = capture_end();
    g_comm.initialized = true;

    TEST_ASSERT_EQUAL_STRING("comm_iface not initialized\n", out);
}

TEST(a_receive_needs_data_and_at_least_one_byte) {
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, comm_port_rx(P0, NULL, 4));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG,
                          comm_port_rx(P0, (const uint8_t *)"ABC", 0));

    TEST_ASSERT_EQUAL_INT(0, (int)comm_port_rx_pending(P0));
    TEST_ASSERT_EQUAL_INT(0, (int)comm_port_rx_dropped(P0));
}

TEST(a_read_with_no_buffer_or_no_room_takes_nothing) {
    uint8_t buf[8] = {0};

    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          comm_port_rx(P0, (const uint8_t *)"ABC", 3));

    TEST_ASSERT_EQUAL_INT(0, (int)comm_port_read(P0, NULL, sizeof(buf), 0));
    TEST_ASSERT_EQUAL_INT(0, (int)comm_port_read(P0, buf, 0, 0));
    TEST_ASSERT_EQUAL_INT(3, (int)comm_port_rx_pending(P0));
    TEST_ASSERT_EQUAL_INT(3, (int)comm_port_read(P0, buf, sizeof(buf), 0));
}

TEST(client_gone_on_an_unknown_port_touches_nothing) {
    uint8_t buf[8] = {0};

    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          comm_port_rx(P0, (const uint8_t *)"ABC", 3));

    comm_port_client_gone(COMM_INVALID_PORT_ID);
    comm_port_client_gone(COMM_MAX_PORTS - 1); /* a slot nobody registered */

    TEST_ASSERT_EQUAL_INT(3, (int)comm_port_rx_pending(P0));
    TEST_ASSERT_EQUAL_INT(0, (int)g_comm.ports[P0].rx_discarded);
    TEST_ASSERT_FALSE(g_comm.ports[P0].muted);
    TEST_ASSERT_EQUAL_INT(3, (int)comm_port_read(P0, buf, sizeof(buf), 0));
}

TEST(a_write_needs_data_and_at_least_one_byte) {
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, comm_port_write(P0, NULL, 2));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_ARG, comm_port_write(P0, "X", 0));

    TEST_ASSERT_EQUAL_INT(0, (int)fake_port_len(0));
    TEST_ASSERT_EQUAL_INT(0, fake_port_write_count(0));
}

/** One full transport's worth of bytes, for filling the fake's FIFO. */
static uint8_t g_fill[FAKE_PORT_CAP];

TEST(a_transport_that_accepts_nothing_is_flushed_then_fails_the_write) {
    memset(g_fill, 'F', sizeof(g_fill));

    /* The fake takes exactly FAKE_PORT_CAP bytes; once they are in, it
     * accepts none, which is a refusal rather than a short write. */
    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_write(P0, g_fill, sizeof(g_fill)));
    TEST_ASSERT_EQUAL_INT(0, fake_port_flush_count(0));

    /* Each refused write is followed by a flush, until the stall limit. */
    TEST_ASSERT_EQUAL_INT(ESP_FAIL, comm_port_write(P0, "X", 1));
    TEST_ASSERT_EQUAL_INT(PORT_WRITE_STALL_TICKS + 1, fake_port_flush_count(0));
    TEST_ASSERT_EQUAL_INT(FAKE_PORT_CAP, (int)fake_port_len(0));
}

TEST(a_write_larger_than_the_fifo_goes_out_whole) {
    /* Four bytes per call: the transport drains in three writes, flushing
     * between them, so the whole twelve bytes go out. */
    fake_port_set_write_limit(0, 4);

    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          comm_port_write(P0, "ABCDEFGHIJKL", 12));

    TEST_ASSERT_EQUAL_STRING("ABCDEFGHIJKL", fake_port_text(0));
    TEST_ASSERT_EQUAL_INT(2, fake_port_flush_count(0));
    TEST_ASSERT_EQUAL_INT(3, fake_port_write_count(0));
}

TEST(a_flush_on_an_unknown_port_does_nothing) {
    comm_port_flush(COMM_INVALID_PORT_ID);
    comm_port_flush(COMM_MAX_PORTS - 1);

    TEST_ASSERT_EQUAL_INT(0, fake_port_flush_count(0));
    TEST_ASSERT_EQUAL_INT(0, fake_port_flush_count(1));
}

TEST(an_unknown_port_has_no_drops_to_report) {
    TEST_ASSERT_EQUAL_INT(0, (int)comm_port_rx_dropped(COMM_MAX_PORTS - 1));
}

TEST(debug_info_reports_each_port_and_leaves_empty_slots_out) {
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          comm_port_rx(P0, (const uint8_t *)"0100\r", 5));
    comm_port_client_gone(P0); /* 5 bytes discarded, port muted */
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          comm_port_rx(P1, (const uint8_t *)"ABC", 3));
    fake_port_set_connected(1, false);

    capture_begin();
    comm_print_debug_info();
    const char *out = capture_end();

    TEST_ASSERT_NOT_NULL(strstr(out, "Ports:\n"));
    TEST_ASSERT_NOT_NULL(strstr(
        out, "[0] FAKE0    connected:yes  muted:yes  rx_dropped:0 "
             "rx_discarded:5 backlog:0\n"));
    TEST_ASSERT_NOT_NULL(strstr(
        out, "[1] FAKE1    connected:no   muted:no   rx_dropped:0 "
             "rx_discarded:0 backlog:3\n"));
    /* The two slots nobody registered are not listed. */
    TEST_ASSERT_NULL(strstr(out, "[2]"));
    TEST_ASSERT_NULL(strstr(out, "[3]"));
    /* And only the BLE port gets the extra link counters. */
    TEST_ASSERT_NULL(strstr(out, "notify_dropped"));
}

TEST(the_ble_port_also_reports_its_link_counters) {
    comm_port_id_t ble = comm_port_register("BLE", &g_plain_ops, 32);

    TEST_ASSERT_EQUAL_INT(2, ble);

    capture_begin();
    comm_print_debug_info();
    const char *out = capture_end();

    /* No is_connected in its ops, so it is always shown as attached. */
    TEST_ASSERT_NOT_NULL(strstr(
        out, "[2] BLE      connected:yes  muted:no   rx_dropped:0 "
             "rx_discarded:0 backlog:0\n"));
    TEST_ASSERT_NOT_NULL(strstr(out, "notify_dropped:0 notify_retries:0\n"));
    TEST_ASSERT_NOT_NULL(strstr(out, "rx_bytes:0 widest_write:0 refused:0\n"));
}

TEST(init_when_already_up_keeps_the_ports_it_has) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_iface_init());

    /* The table was not rebuilt: the port is still there and still writes. */
    TEST_ASSERT_TRUE(g_comm.ports[P0].active);
    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_write(P0, "A", 1));
    TEST_ASSERT_EQUAL_STRING("A", fake_port_text(0));
}

TEST(client_ready_while_the_table_is_down_leaves_the_port_muted) {
    comm_port_client_gone(P0);

    g_comm.initialized = false;
    comm_port_client_ready(P0);
    g_comm.initialized = true;

    TEST_ASSERT_TRUE(g_comm.ports[P0].muted);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_NOT_FOUND, comm_port_write(P0, "x", 1));
    TEST_ASSERT_EQUAL_INT(0, (int)fake_port_len(0));
}

TEST(init_reports_a_refused_lock_and_is_not_marked_up) {
    SemaphoreHandle_t lock = g_comm.lock;

    g_comm.initialized = false;
    refuse_mutex = true;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_NO_MEM, comm_iface_init());
    refuse_mutex = false;
    TEST_ASSERT_FALSE(g_comm.initialized);

    /* Put back what the fixture set up, so later calls still have a lock. */
    g_comm.lock = lock;
    g_comm.initialized = true;
}

static int refuse_write(const void *buf, uint32_t length) {
    (void)buf;
    (void)length;
    return -1;
}

static int take_one_byte(const void *buf, uint32_t length) {
    (void)buf;
    (void)length;
    return 1;
}

TEST(a_transport_that_refuses_a_write_fails_the_port_write) {
    static const comm_port_ops_t ops = {.write = refuse_write};
    comm_port_id_t id = comm_port_register("REFUSE", &ops, 32);

    TEST_ASSERT_MSG(id != COMM_INVALID_PORT_ID,
                    "the refusing port did not register");
    TEST_ASSERT_EQUAL_INT(ESP_FAIL, comm_port_write(id, "abc", 3));
}

TEST(a_partial_write_to_a_transport_without_flush_gives_up) {
    static const comm_port_ops_t ops = {.write = take_one_byte};
    comm_port_id_t id = comm_port_register("PARTIAL", &ops, 32);

    TEST_ASSERT_MSG(id != COMM_INVALID_PORT_ID,
                    "the partial port did not register");
    TEST_ASSERT_EQUAL_INT(ESP_FAIL, comm_port_write(id, "abc", 3));
}
