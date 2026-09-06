/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_clock.h
 * @brief The single source of time for host tests.
 *
 * Nothing in a host test ever sleeps. Time moves only when something that
 * would have blocked on target says how long it waited: a ring buffer receive
 * that timed out, a faked bus driver delivering a frame, vTaskDelay().
 *
 * That is what makes the ELM327 timeout loops testable. ATZ waits a second,
 * elm327_can_protocol_xfer() spins on Timer_is_expired() for the AT ST
 * timeout; both terminate in microseconds here, deterministically, and still
 * observe the elapsed milliseconds they were written against.
 */

#pragma once

#include <stdint.h>

/** Milliseconds since the clock was reset. */
uint32_t fake_clock_ms(void);

void fake_clock_advance_ms(uint32_t ms);

/** Rewinds to zero. Call from the per-test fixture. */
void fake_clock_reset(void);
