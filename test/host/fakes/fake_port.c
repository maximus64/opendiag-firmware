/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_port.c
 * @brief A comm_iface transport that captures everything written to it.
 */

#include "fake_port.h"

#include <string.h>

#include "esp_err.h"

typedef struct {
    char tx[FAKE_PORT_CAP + 1];
    size_t tx_len;
    int flushes;
    bool connected;
    size_t write_limit; /* 0 means accept everything */
    comm_port_id_t id;
} fake_port_t;

static fake_port_t g_port[FAKE_PORT_COUNT];

static int port_write(int idx, const void *buf, uint32_t length) {
    fake_port_t *p = &g_port[idx];
    size_t n = length;

    if (p->write_limit && n > p->write_limit) {
        n = p->write_limit;
    }
    if (n > FAKE_PORT_CAP - p->tx_len) {
        n = FAKE_PORT_CAP - p->tx_len;
    }

    memcpy(p->tx + p->tx_len, buf, n);
    p->tx_len += n;
    p->tx[p->tx_len] = '\0';

    return (int)n;
}

static int port0_write(const void *b, uint32_t n) {
    return port_write(0, b, n);
}
static int port1_write(const void *b, uint32_t n) {
    return port_write(1, b, n);
}
static void port0_flush(void) { g_port[0].flushes++; }
static void port1_flush(void) { g_port[1].flushes++; }
static bool port0_connected(void) { return g_port[0].connected; }
static bool port1_connected(void) { return g_port[1].connected; }

static const comm_port_ops_t g_ops[FAKE_PORT_COUNT] = {
    {.write = port0_write,
     .flush = port0_flush,
     .is_connected = port0_connected},
    {.write = port1_write,
     .flush = port1_flush,
     .is_connected = port1_connected},
};

static const char *const g_names[FAKE_PORT_COUNT] = {"FAKE0", "FAKE1"};

void fake_port_register_all(void) {
    fake_port_reset();

    for (int i = 0; i < FAKE_PORT_COUNT; i++) {
        g_port[i].id =
            comm_port_register(g_names[i], &g_ops[i], FAKE_PORT_RX_BUF);
    }
}

comm_port_id_t fake_port_id(int idx) {
    if (idx < 0 || idx >= FAKE_PORT_COUNT) {
        return COMM_INVALID_PORT_ID;
    }
    return g_port[idx].id;
}

void fake_port_reset(void) {
    for (int i = 0; i < FAKE_PORT_COUNT; i++) {
        comm_port_id_t saved = g_port[i].id;

        memset(&g_port[i], 0, sizeof(g_port[i]));
        g_port[i].id = saved;
        g_port[i].connected = true;
    }
}

const char *fake_port_text(int idx) {
    if (idx < 0 || idx >= FAKE_PORT_COUNT) {
        return "";
    }
    return g_port[idx].tx;
}

size_t fake_port_len(int idx) {
    if (idx < 0 || idx >= FAKE_PORT_COUNT) {
        return 0;
    }
    return g_port[idx].tx_len;
}

int fake_port_flush_count(int idx) {
    if (idx < 0 || idx >= FAKE_PORT_COUNT) {
        return 0;
    }
    return g_port[idx].flushes;
}

void fake_port_set_connected(int idx, bool connected) {
    if (idx >= 0 && idx < FAKE_PORT_COUNT) {
        g_port[idx].connected = connected;
    }
}

void fake_port_set_write_limit(int idx, size_t limit) {
    if (idx >= 0 && idx < FAKE_PORT_COUNT) {
        g_port[idx].write_limit = limit;
    }
}
