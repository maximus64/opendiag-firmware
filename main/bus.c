/* SPDX-License-Identifier: GPL-3.0-only */
#include "bus.h"
#include <string.h>
#include <strings.h>

static const struct {
    bus_param_t p;
    const char *name;
} g_params[] = {
    {BUS_P_DATA_RATE, "data-rate"},
    {BUS_P_NODE_ADDRESS, "node-address"},
    {BUS_P_NETWORK_LINE, "network-line"},
    {BUS_P_P1_MAX, "p1-max"},
    {BUS_P_P2_MAX, "p2-max"},
    {BUS_P_P3_MIN, "p3-min"},
    {BUS_P_P4_MIN, "p4-min"},
    {BUS_P_W0_MIN, "w0-min"},
    {BUS_P_W1_MAX, "w1-max"},
    {BUS_P_W2_MAX, "w2-max"},
    {BUS_P_W3_MAX, "w3-max"},
    {BUS_P_W4_MIN, "w4-min"},
    {BUS_P_W4_MAX, "w4-max"},
    {BUS_P_W5_MIN, "w5-min"},
    {BUS_P_TIDLE, "tidle"},
    {BUS_P_TINIL, "tinil"},
    {BUS_P_TWUP, "twup"},
    {BUS_P_PARITY, "parity"},
    {BUS_P_DATA_BITS, "data-bits"},
    {BUS_P_FIVE_BAUD_MOD, "five-baud-mod"},
    {BUS_P_LOOPBACK, "loopback"},

    {BUS_P_CHECKSUM_RX, "checksum-rx"},
    {BUS_P_CHECKSUM_TX, "checksum-tx"},
    {BUS_P_TX_RETRIES, "tx-retries"},
    {BUS_P_DUPLICATE_MS, "duplicate-ms"},
    {BUS_P_PERIODIC_MS, "periodic-ms"},
    {BUS_P_PERIODIC_QUIET, "periodic-quiet"},
    {BUS_P_TIMING_FROM_KEYBYTES, "timing-from-keybytes"},
    {BUS_P_IFR_ENABLED, "ifr-enabled"},
    {BUS_P_IFR_BYTE, "ifr-byte"},
    {BUS_P_ARBITRATION, "arbitration"},
    {BUS_P_K_LINE_ONLY, "k-line-only"},
};

#define PARAM_COUNT (sizeof(g_params) / sizeof(g_params[0]))

const char *bus_param_name(bus_param_t p) {
    for (size_t i = 0; i < PARAM_COUNT; i++) {
        if (g_params[i].p == p) {
            return g_params[i].name;
        }
    }

    return NULL;
}

bool bus_param_from_name(const char *name, bus_param_t *out) {
    if (!name || !out) {
        return false;
    }

    for (size_t i = 0; i < PARAM_COUNT; i++) {
        if (strcasecmp(name, g_params[i].name) == 0) {
            *out = g_params[i].p;
            return true;
        }
    }

    return false;
}

const char *bus_param_at(size_t idx, bus_param_t *out) {
    if (idx >= PARAM_COUNT) {
        return NULL;
    }

    if (out) {
        *out = g_params[idx].p;
    }

    return g_params[idx].name;
}
