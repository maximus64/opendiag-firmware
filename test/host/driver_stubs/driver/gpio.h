/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include "esp_attr.h"
#include "../../stubs/driver/gpio.h"
#define GPIO_NUM_NC (-1)
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t value);
