/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_led.h
 * @brief Test double for main/ws2812_led.c.
 *
 * Only the state matters here: vif drives the LED to warn that a connector pin
 * is energised, and that is the behaviour under test, not the SPI timing of a
 * WS2812.
 */

#pragma once

#include "ws2812_led.h"

void fake_led_reset(void);

/** Latest state passed to ws2812_led_set_state(). */
led_state_t fake_led_state(void);

/** Times the state actually changed, ignoring repeats of the same state. */
int fake_led_change_count(void);
