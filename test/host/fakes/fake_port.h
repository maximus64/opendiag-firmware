/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_port.h
 * @brief A comm_iface transport that captures everything written to it.
 *
 * The real comm_iface.c is linked into the tests, so responses travel the
 * production path: elm327 tx ring buffer, comm_port_write(), the port's write
 * vtable. This is where they land.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "comm_iface.h"

/** Receive buffer each fake port is registered with. */
#define FAKE_PORT_RX_BUF 256

#define FAKE_PORT_COUNT 2
#define FAKE_PORT_CAP   2048

/** Registers FAKE_PORT_COUNT ports with comm_iface. Call once per process. */
void fake_port_register_all(void);

/** Port ID handed out by comm_iface for @p idx. */
comm_port_id_t fake_port_id(int idx);

/** Clears the captured bytes and counters on every port. */
void fake_port_reset(void);

/** Captured bytes as a NUL-terminated string. Never NULL. */
const char *fake_port_text(int idx);

size_t fake_port_len(int idx);

int fake_port_flush_count(int idx);

/** Marks a port disconnected, so comm_iface should skip it. */
void fake_port_set_connected(int idx, bool connected);

/** Caps how much one write() accepts, to exercise the short write retry. */
void fake_port_set_write_limit(int idx, size_t limit);
