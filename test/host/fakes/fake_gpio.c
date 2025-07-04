/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_gpio.c
 * @brief The driver/gpio.h stub's implementation.
 */

#include "fake_gpio.h"

#include <string.h>

static struct {
    int level;
    bool was_reset;
    gpio_mode_t mode;
} g_pin[FAKE_GPIO_MAX_PINS];

static bool pin_valid(gpio_num_t pin)
{
    return pin >= 0 && pin < FAKE_GPIO_MAX_PINS;
}

void fake_gpio_reset(void)
{
    memset(g_pin, 0, sizeof(g_pin));

    /* Idle high, so an untouched active low button reads as released. */
    for (int i = 0; i < FAKE_GPIO_MAX_PINS; i++) {
        g_pin[i].level = 1;
    }
}

void fake_gpio_set_level(gpio_num_t pin, int level)
{
    if (pin_valid(pin)) {
        g_pin[pin].level = level ? 1 : 0;
    }
}

bool fake_gpio_was_reset(gpio_num_t pin)
{
    return pin_valid(pin) ? g_pin[pin].was_reset : false;
}

gpio_mode_t fake_gpio_direction(gpio_num_t pin)
{
    return pin_valid(pin) ? g_pin[pin].mode : GPIO_MODE_DISABLE;
}

/* ------------------------------------------------------------------ *
 * The driver API itself
 * ------------------------------------------------------------------ */

esp_err_t gpio_reset_pin(gpio_num_t gpio_num)
{
    if (!pin_valid(gpio_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    g_pin[gpio_num].was_reset = true;
    g_pin[gpio_num].mode = GPIO_MODE_DISABLE;

    return ESP_OK;
}

esp_err_t gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode)
{
    if (!pin_valid(gpio_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    g_pin[gpio_num].mode = mode;

    return ESP_OK;
}

int gpio_get_level(gpio_num_t gpio_num)
{
    return pin_valid(gpio_num) ? g_pin[gpio_num].level : 0;
}
