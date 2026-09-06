/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include <stdint.h>

typedef enum {
    LED_STATE_OFF,
    LED_STATE_IDLE,
    LED_STATE_RUNNING,
    LED_STATE_ERROR,
    LED_STATE_PIN_LIVE,
    LED_STATE_PAIRING,
} led_state_t;

void ws2812_led_init(void);
void ws2812_led_set_state(led_state_t new_state);
