/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file rmt_types.h
 * @brief hal/ alias for the driver/ stub, matching ESP-IDF's own layout.
 *
 * ESP-IDF defines rmt_symbol_word_t in hal/rmt_types.h and re-exports it
 * through driver/rmt_types.h. j1850_pwm_codec.h includes the hal/ path
 * directly, so the host build needs it under both.
 */

#pragma once

#include "driver/rmt_types.h"
