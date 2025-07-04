/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include <stdint.h>

#include <stdint.h>

/**
 * @brief Defines the different states the LED animation can be in.
 */
typedef enum {
    LED_STATE_OFF,     // LED off
    LED_STATE_IDLE,    // Breathing cyan color
    LED_STATE_RUNNING, // Solid green color
    LED_STATE_ERROR,   // Flashing red at 2Hz
    LED_STATE_PIN_LIVE, // Pulsing amber: an OBD connector pin is energised
} led_state_t;

/**
 * @brief Initializes the WS2812B LED driver and starts the animation task.
 * * This function sets up the SPI peripheral required to drive the LED and
 * creates a dedicated FreeRTOS task to handle animations.
 */
void ws2812_led_init(void);

/**
 * @brief Sets the current animation state for the LED.
 * * This function is thread-safe and can be called from any task to change
 * the LED's behavior.
 * * @param new_state The new state to set from the led_state_t enum.
 */
void ws2812_led_set_state(led_state_t new_state);
