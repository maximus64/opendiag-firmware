/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_bytebus.h
 * @brief Test double for the three byte-oriented bus drivers.
 *
 * kline.c, j1850_pwm.c and j1850_vpw.c expose the same four calls with the
 * same semantics, so one fake serves all three. Each gets its own independent
 * instance, selected by fake_bus_id_t.
 *
 * The staging model matches fake_can_bus: a response is delivered only after
 * the next send, so the pre-transmit drain the ELM327 layer performs cannot
 * accidentally consume it.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    FAKE_BUS_KLINE = 0,
    FAKE_BUS_J1850_PWM,
    FAKE_BUS_J1850_VPW,
    FAKE_BUS_COUNT,
} fake_bus_id_t;

/** Clears every instance. */
void fake_bytebus_reset_all(void);

/* --- Staging received traffic --- */

/** Queue a frame the ECU sends @p delay_ms after the next request goes out. */
void fake_bytebus_stage_response(fake_bus_id_t bus, const uint8_t *data,
                                 size_t len, uint32_t delay_ms);

/** Queue a frame already sitting in the driver before any request. */
void fake_bytebus_stage_stale(fake_bus_id_t bus, const uint8_t *data, size_t len);

/* --- Inspecting the driver's lifecycle --- */

int fake_bytebus_setup_count(fake_bus_id_t bus);
int fake_bytebus_teardown_count(fake_bus_id_t bus);
bool fake_bytebus_is_up(fake_bus_id_t bus);

/** Times kline_sync() ran the 5 baud init. K-Line only. */
int fake_bytebus_sync_count(void);

/** Makes the next send on @p bus fail. */
void fake_bytebus_fail_next_send(fake_bus_id_t bus);

/* --- Inspecting transmitted traffic --- */

int fake_bytebus_sent_count(fake_bus_id_t bus);

/** Bytes of transmit @p idx, with its length in @p out_len. NULL if absent. */
const uint8_t *fake_bytebus_sent(fake_bus_id_t bus, int idx, size_t *out_len);
