/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file gpio.h
 * @brief Enough of driver/gpio.h for the host build.
 *
 * Shadows the real IDF header on the include path. Only button.c reaches for
 * GPIO directly on the host - board.c is replaced wholesale by fake_board.c -
 * so this carries the three calls it makes and nothing else.
 *
 * The implementation is fakes/fake_gpio.c, which lets a test drive a pin.
 */

#pragma once

#include "esp_err.h"

typedef int gpio_num_t;

typedef enum {
    GPIO_MODE_DISABLE = 0,
    GPIO_MODE_INPUT = 1,
    GPIO_MODE_OUTPUT = 2,
    GPIO_MODE_INPUT_OUTPUT = 3,
} gpio_mode_t;

esp_err_t gpio_reset_pin(gpio_num_t gpio_num);
esp_err_t gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode);
int gpio_get_level(gpio_num_t gpio_num);
