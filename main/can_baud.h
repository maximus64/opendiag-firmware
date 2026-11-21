/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stddef.h>
#include <stdint.h>

/**
 * @file can_baud.h
 * @brief Finding a CAN bus's bit rate without ever transmitting.
 *
 * The caller opens the bus listen-only at each rate in turn and counts what
 * it heard. At the right rate frames decode and pass their CRC; at a wrong
 * one the controller sees bit and form errors and next to no valid frame.
 * Listen-only means no acknowledgement and no error frame, so a wrong guess
 * costs the vehicle nothing.
 */

typedef struct {
    uint32_t rate;
    uint32_t frames; /**< Valid frames heard. */
    uint32_t errors; /**< Bus errors heard. */
} can_baud_sample_t;

/** @brief Rates to try, most likely first. */
const uint32_t *can_baud_order(size_t *count);

/**
 * @brief The sample whose rate matches the bus, or -1.
 *
 * The one with most valid frames, provided errors stay under a quarter of
 * them; a silent bus, or one where every rate errs, has no answer.
 */
int can_baud_pick(const can_baud_sample_t *s, size_t n);
