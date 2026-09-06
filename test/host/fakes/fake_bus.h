/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_bus.h
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

#include "bus.h"
#include "kline_codec.h"

typedef enum {
    FAKE_BUS_KLINE = 0,
    FAKE_BUS_J1850_PWM,
    FAKE_BUS_J1850_VPW,
    FAKE_BUS_COUNT,
} fake_bus_id_t;

/** Clears every instance. */
void fake_bus_reset_all(void);

/* --- Staging received traffic --- */

/** Queue a frame the ECU sends @p delay_ms after the next request goes out. */
void fake_bus_stage_response(fake_bus_id_t bus, const uint8_t *data, size_t len,
                             uint32_t delay_ms);

/** Queue a frame already sitting in the driver before any request. */
void fake_bus_stage_stale(fake_bus_id_t bus, const uint8_t *data, size_t len);

/* --- Inspecting the driver's lifecycle --- */

int fake_bus_setup_count(fake_bus_id_t bus);
int fake_bus_teardown_count(fake_bus_id_t bus);
bool fake_bus_is_up(fake_bus_id_t bus);
/** Run once inside the next receive, to simulate another task. */
void fake_bus_on_receive(fake_bus_id_t bus, void (*hook)(void *), void *arg);
void fake_bus_on_lifecycle(fake_bus_id_t id, void (*hook)(void *), void *arg);
void fake_bus_open_result(fake_bus_id_t id, esp_err_t result);
void fake_bus_close_result(fake_bus_id_t bus, esp_err_t result);

/** Times an initialisation IOCTL ran a handshake. K-Line only. */
int fake_bus_sync_count(void);

/** Times BUS_IOCTL_STOP_COMM ended a session. K-Line only. */
int fake_bus_stop_comm_count(void);

/** What the next and every later initialisation IOCTL returns. K-Line only. */
void fake_bus_kline_connect_result(int rc);

/** The variant a successful kline_connect() reports having negotiated. */
void fake_bus_kline_set_variant(kline_variant_t v);

/** The rate AT IB pushed down, and the wakeup AT SW / AT WM did. */
uint32_t fake_bus_kline_baud(void);
uint32_t fake_bus_kline_wakeup_ms(void);
const uint8_t *fake_bus_kline_wakeup(size_t *out_len);

/** Makes the next send on @p bus fail. */
void fake_bus_fail_next_send(fake_bus_id_t bus);

/* --- Inspecting transmitted traffic --- */

int fake_bus_sent_count(fake_bus_id_t bus);

/** Bytes of transmit @p idx, with its length in @p out_len. NULL if absent. */
const uint8_t *fake_bus_sent(fake_bus_id_t bus, int idx, size_t *out_len);
