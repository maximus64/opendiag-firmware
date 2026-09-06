/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_can_bus.h
 * @brief Test double for main/can_bus.c.
 *
 * Two ways to stage a received frame, because the ELM327 layer drains the
 * receive queue before every transmit:
 *
 *   fake_can_stage_response() - delivered only after the next can_send(), the
 *                               way an ECU answers a request.
 *   fake_can_stage_stale()    - available immediately, so a test can prove the
 *                               pre-send drain really discards it.
 *
 * Each staged frame carries a delivery delay in milliseconds that is charged
 * to the fake clock when the frame is handed over. That is how a test makes a
 * reply arrive after the AT ST timeout, or inside an extended response-pending
 * window.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "can_bus.h"

void fake_can_reset(void);

/* --- Staging received traffic --- */

/** Queue a frame the ECU sends @p delay_ms after the next request goes out. */
void fake_can_stage_response(uint32_t id, uint8_t dlc, const uint8_t *data,
                             uint32_t delay_ms);

/**
 * @brief Queue a frame released only after @p after_sends further transmits.
 *
 * Use 2 for the consecutive frames of a multi-frame reply, which a real ECU
 * only sends once it has seen the flow control frame.
 */
void fake_can_stage_response_at(uint32_t id, uint8_t dlc, const uint8_t *data,
                                uint32_t delay_ms, int after_sends);

/** Queue a frame that is already sitting in the driver before any request. */
void fake_can_stage_stale(uint32_t id, uint8_t dlc, const uint8_t *data);

/* --- Inspecting the driver's lifecycle --- */

int fake_can_setup_count(void);
int fake_can_teardown_count(void);

/** Baud rate of the most recent can_bus_setup(), or -1 if never set up. */
int fake_can_last_baud(void);

bool fake_can_is_up(void);

/** Makes the next can_send() fail, to exercise the error path. */
void fake_can_fail_next_send(void);

/** Makes can_bus_setup() refuse, to exercise a driver that will not start. */
void fake_can_fail_setup(bool fail);

/* --- Inspecting transmitted traffic --- */

int fake_can_sent_count(void);

/** Frame at @p idx of the transmit log, or NULL. */
const struct can_frame *fake_can_sent(int idx);

void fake_can_fail_confirmed_send(void);
void fake_can_abort_confirmed_send(void);
