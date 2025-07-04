/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_gpio.h
 * @brief A GPIO a test can drive, standing in for the real pins.
 *
 * Levels default to high, which is a released button: PIN_BUTTON is active
 * low, so a test that never touches a pin sees nothing pressed.
 */

#pragma once

#include <stdbool.h>

#include "driver/gpio.h"

#define FAKE_GPIO_MAX_PINS 64

void fake_gpio_reset(void);

/** @brief Drive a pin, as the outside world would. */
void fake_gpio_set_level(gpio_num_t pin, int level);

/** @brief True once gpio_reset_pin() has been called for this pin. */
bool fake_gpio_was_reset(gpio_num_t pin);

/** @brief The mode gpio_set_direction() last set, or GPIO_MODE_DISABLE. */
gpio_mode_t fake_gpio_direction(gpio_num_t pin);
