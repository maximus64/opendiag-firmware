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

#include <string.h>

#include "td_test.h"

#include "fake_port.h"
#include "freertos/semphr.h"

#include "comm_iface.c"

#define P0 (fake_port_id(0))
#define P1 (fake_port_id(1))

/** Delivers @p s to @p port and drains it into @p buf. Returns bytes read. */
static size_t round_trip(comm_port_id_t port, const char *s,
                         uint8_t *buf, size_t cap)
{
    TEST_ASSERT_EQUAL_INT(ESP_OK,
        comm_port_rx(port, (const uint8_t *)s, strlen(s)));

    return comm_port_read(port, buf, cap, pdMS_TO_TICKS(100));
}

void td_setup(void)
{
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

void td_teardown(void)
{
    TEST_ASSERT_MSG(idf_stub_lock_balance() == 0,
                    "left %d locks held", idf_stub_lock_balance());
}

/* ------------------------------------------------------------------ *
 * Receiving
 * ------------------------------------------------------------------ */

TEST(received_bytes_reach_the_reader)
{
    uint8_t buf[32] = {0};
    size_t n = round_trip(P0, "ATZ\r", buf, sizeof(buf));

    TEST_ASSERT_EQUAL_INT(4, (int)n);
    TEST_ASSERT_EQUAL_STRING("ATZ\r", (const char *)buf);
}

TEST(a_read_returns_nothing_when_the_transport_is_idle)
{
    uint8_t buf[8];

    TEST_ASSERT_EQUAL_INT(0, comm_port_read(P0, buf, sizeof(buf),
                                            pdMS_TO_TICKS(10)));
}

TEST(a_port_does_not_see_traffic_from_another_port)
{
    uint8_t buf[16];

    TEST_ASSERT_EQUAL_INT(ESP_OK,
        comm_port_rx(P1, (const uint8_t *)"hello", 5));

    TEST_ASSERT_EQUAL_INT(0, comm_port_read(P0, buf, sizeof(buf), 0));
    TEST_ASSERT_EQUAL_INT(5, (int)comm_port_read(P1, buf, sizeof(buf), 0));
}

TEST(an_overflowing_receive_buffer_drops_and_counts_instead_of_blocking)
{
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

TEST(a_write_goes_out_the_port_it_names)
{
    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_write(P0, "X\r", 2));

    TEST_ASSERT_EQUAL_STRING("X\r", fake_port_text(0));
    TEST_ASSERT_EQUAL_INT(0, (int)fake_port_len(1));
}

TEST(a_flush_reaches_the_transport)
{
    comm_port_flush(P0);

    TEST_ASSERT_EQUAL_INT(1, fake_port_flush_count(0));
    TEST_ASSERT_EQUAL_INT(0, fake_port_flush_count(1));
}

TEST(a_short_write_is_completed_through_a_flush)
{
    /* A slow host leaves room for only part of the response in the FIFO. */
    fake_port_set_write_limit(0, 4);

    TEST_ASSERT_EQUAL_INT(ESP_OK, comm_port_write(P0, "ABCDEFGH", 8));

    TEST_ASSERT_EQUAL_STRING("ABCDEFGH", fake_port_text(0));
    TEST_ASSERT_MSG(fake_port_flush_count(0) > 0,
                    "the remainder went out without draining the FIFO first");
}

TEST(a_disconnected_port_is_not_written_to)
{
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

TEST(registering_more_ports_than_there_are_slots_fails)
{
    static const comm_port_ops_t ops = { .write = NULL };
    comm_port_ops_t writable = ops;
    int registered = 2;     /* The two fake ports already hold slots. */

    /* Any non-NULL write is enough; nothing is transmitted here. */
    writable.write = (int (*)(const void *, uint32_t))(void *)&registered;

    while (registered < COMM_MAX_PORTS) {
        TEST_ASSERT_MSG(comm_port_register("X", &writable, 32)
                        != COMM_INVALID_PORT_ID,
                        "slot %d should still have been free", registered);
        registered++;
    }

    /* This is what the on-target commtest hit, with three of the four slots
     * permanently held. */
    TEST_ASSERT_EQUAL_INT(COMM_INVALID_PORT_ID,
                          comm_port_register("OVER", &writable, 32));
}

TEST(a_port_needs_a_write_function)
{
    static const comm_port_ops_t no_write = { .write = NULL };

    TEST_ASSERT_EQUAL_INT(COMM_INVALID_PORT_ID,
                          comm_port_register("BAD", &no_write, 32));
    TEST_ASSERT_EQUAL_INT(COMM_INVALID_PORT_ID,
                          comm_port_register("BAD", NULL, 32));
}

TEST(a_port_needs_a_receive_buffer)
{
    static const comm_port_ops_t ops = { .write = NULL };
    comm_port_ops_t writable = ops;
    int dummy = 0;

    writable.write = (int (*)(const void *, uint32_t))(void *)&dummy;

    TEST_ASSERT_EQUAL_INT(COMM_INVALID_PORT_ID,
                          comm_port_register("BAD", &writable, 0));
}

TEST(a_name_longer_than_the_limit_is_truncated_not_overrun)
{
    static const comm_port_ops_t ops = { .write = NULL };
    comm_port_ops_t writable = ops;
    comm_port_id_t id;
    int dummy = 0;

    writable.write = (int (*)(const void *, uint32_t))(void *)&dummy;

    id = comm_port_register("VERYLONGNAME", &writable, 32);

    TEST_ASSERT_MSG(id != COMM_INVALID_PORT_ID, "registration failed");
    TEST_ASSERT_EQUAL_STRING("VERYLON", g_comm.ports[id].name);
}

TEST(invalid_ids_are_rejected_everywhere)
{
    uint8_t buf[8];

    TEST_ASSERT_MSG(comm_port_rx(COMM_INVALID_PORT_ID, (const uint8_t *)"x", 1)
                    != ESP_OK, "rx accepted an invalid port");
    TEST_ASSERT_EQUAL_INT(0, comm_port_read(COMM_INVALID_PORT_ID, buf,
                                            sizeof(buf), 0));
    TEST_ASSERT_MSG(comm_port_write(COMM_INVALID_PORT_ID, "x", 1) != ESP_OK,
                    "write accepted an invalid port");
    TEST_ASSERT_EQUAL_INT(0, comm_port_rx_dropped(COMM_INVALID_PORT_ID));

    /* A slot nobody registered is as invalid as the sentinel. */
    TEST_ASSERT_MSG(comm_port_write(COMM_MAX_PORTS - 1, "x", 1) != ESP_OK,
                    "write accepted an unregistered port slot");
}
