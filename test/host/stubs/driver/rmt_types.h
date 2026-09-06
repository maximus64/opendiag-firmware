/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file rmt_types.h
 * @brief Host stand-in for the ESP-IDF RMT symbol layout.
 *
 * j1850_pwm_codec.c speaks the peripheral's own symbol format so the receive
 * ISR does not have to copy a capture before decoding it. That one type is
 * all the codec needs from the RMT driver, and it is a plain bitfield, so the
 * host build reproduces it exactly rather than adapting around it.
 *
 * Field order and widths must match hal/rmt_types.h. They have been stable
 * since the peripheral exists: the hardware reads these words directly.
 */

#pragma once

#include <stdint.h>

typedef union {
    struct {
        uint16_t duration0 : 15; /*!< Duration of level0 */
        uint16_t level0 : 1;     /*!< Level of the first part */
        uint16_t duration1 : 15; /*!< Duration of level1 */
        uint16_t level1 : 1;     /*!< Level of the second part */
    };
    uint32_t val;
} rmt_symbol_word_t;
