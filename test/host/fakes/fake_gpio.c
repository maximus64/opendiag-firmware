/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_gpio.c
 * @brief The driver/gpio.h stub's implementation.
 */

#include "fake_gpio.h"

#include <string.h>

#include "freertos/task.h"

static struct {
    int level;
    bool was_reset;
    gpio_mode_t mode;
    gpio_int_type_t intr_type;
    bool isr_attached;
    bool isr_enabled;
} g_pin[FAKE_GPIO_MAX_PINS];

static bool pin_valid(gpio_num_t pin) {
    return pin >= 0 && pin < FAKE_GPIO_MAX_PINS;
}

static bool edge_matches(gpio_int_type_t type, int old_level, int new_level) {
    switch (type) {
    case GPIO_INTR_POSEDGE:
        return old_level == 0 && new_level == 1;
    case GPIO_INTR_NEGEDGE:
        return old_level == 1 && new_level == 0;
    case GPIO_INTR_ANYEDGE:
        return old_level != new_level;
    default:
        return false;
    }
}

void fake_gpio_reset(void) {
    memset(g_pin, 0, sizeof(g_pin));

    /* Idle high, so an untouched active low button reads as released. */
    for (int i = 0; i < FAKE_GPIO_MAX_PINS; i++) {
        g_pin[i].level = 1;
    }
}

/** Mimics real hardware: an armed ISR gets notified on a matching edge. */
void fake_gpio_set_level(gpio_num_t pin, int level) {
    int new_level;
    int old_level;

    if (!pin_valid(pin)) {
        return;
    }

    new_level = level ? 1 : 0;
    old_level = g_pin[pin].level;
    g_pin[pin].level = new_level;

    if (g_pin[pin].isr_attached && g_pin[pin].isr_enabled &&
        edge_matches(g_pin[pin].intr_type, old_level, new_level)) {
        BaseType_t woken = pdFALSE;

        vTaskNotifyGiveFromISR(NULL, &woken);
    }
}

bool fake_gpio_was_reset(gpio_num_t pin) {
    return pin_valid(pin) ? g_pin[pin].was_reset : false;
}

gpio_mode_t fake_gpio_direction(gpio_num_t pin) {
    return pin_valid(pin) ? g_pin[pin].mode : GPIO_MODE_DISABLE;
}

/* ------------------------------------------------------------------ *
 * The driver API itself
 * ------------------------------------------------------------------ */

esp_err_t gpio_reset_pin(gpio_num_t gpio_num) {
    if (!pin_valid(gpio_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    g_pin[gpio_num].was_reset = true;
    g_pin[gpio_num].mode = GPIO_MODE_DISABLE;

    return ESP_OK;
}

esp_err_t gpio_set_direction(gpio_num_t gpio_num, gpio_mode_t mode) {
    if (!pin_valid(gpio_num)) {
        return ESP_ERR_INVALID_ARG;
    }

    g_pin[gpio_num].mode = mode;

    return ESP_OK;
}

int gpio_get_level(gpio_num_t gpio_num) {
    return pin_valid(gpio_num) ? g_pin[gpio_num].level : 0;
}

esp_err_t gpio_install_isr_service(int intr_alloc_flags) {
    (void)intr_alloc_flags;
    return ESP_OK;
}

esp_err_t gpio_set_intr_type(gpio_num_t gpio_num, gpio_int_type_t type) {
    if (!pin_valid(gpio_num)) {
        return ESP_ERR_INVALID_ARG;
    }
    g_pin[gpio_num].intr_type = type;
    return ESP_OK;
}

esp_err_t gpio_isr_handler_add(gpio_num_t gpio_num, gpio_isr_t isr, void *arg) {
    (void)arg;
    if (!pin_valid(gpio_num)) {
        return ESP_ERR_INVALID_ARG;
    }
    g_pin[gpio_num].isr_attached = (isr != NULL);
    return ESP_OK;
}

esp_err_t gpio_isr_handler_remove(gpio_num_t gpio_num) {
    if (!pin_valid(gpio_num)) {
        return ESP_ERR_INVALID_ARG;
    }
    g_pin[gpio_num].isr_attached = false;
    return ESP_OK;
}

esp_err_t gpio_intr_enable(gpio_num_t gpio_num) {
    if (!pin_valid(gpio_num)) {
        return ESP_ERR_INVALID_ARG;
    }
    g_pin[gpio_num].isr_enabled = true;
    return ESP_OK;
}

esp_err_t gpio_intr_disable(gpio_num_t gpio_num) {
    if (!pin_valid(gpio_num)) {
        return ESP_ERR_INVALID_ARG;
    }
    g_pin[gpio_num].isr_enabled = false;
    return ESP_OK;
}
