/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file esp_timer.h
 * @brief Host stand-in for the ESP-IDF high resolution timer.
 */

#pragma once

#include <stdint.h>

/** Microseconds on the fake clock. */
int64_t esp_timer_get_time(void);
