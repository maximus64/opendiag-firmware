/* SPDX-License-Identifier: GPL-3.0-only */
#include "ws2812_led.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "pinout.h"

/*
 * This value scales the brightness of all colors to prevent the LED from
 * being too bright and to reduce power consumption.
 */
#define LED_MAX_BRIGHTNESS 128

// SPI timing bits for WS2812B protocol.
// Each 4 bits of color data are encoded into a 16-bit SPI word.
static const uint16_t timing_bits[16] = {
    0x1111, 0x7111, 0x1711, 0x7711, 0x1171, 0x7171, 0x1771, 0x7771,
    0x1117, 0x7117, 0x1717, 0x7717, 0x1177, 0x7177, 0x1777, 0x7777};

static spi_device_handle_t spi_handle;
static QueueHandle_t led_state_queue;
static led_state_t current_state = LED_STATE_IDLE;

// SPI buffer: 6 words for color data + 2 for reset pulse
static uint16_t led_buf[8] = {0};

#define COLOR_CYAN 0x00FFFF
#define COLOR_GREEN 0x00FF00
#define COLOR_RED 0xFF0000
#define COLOR_AMBER 0xFF6000
#define COLOR_BLUE 0x0000FF
#define COLOR_OFF 0x000000

static const char *TAG = "RGB_LED";

static uint32_t scale_color(uint32_t color, uint8_t brightness) {
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    r = ((uint16_t)r * brightness) / 255;
    g = ((uint16_t)g * brightness) / 255;
    b = ((uint16_t)b * brightness) / 255;

    return (r << 16) | (g << 8) | b;
}

static void ws2812_write_led(uint32_t rgb) {
    int n = 1; // Start at index 1 to leave room for reset pulse start

    // The WS2812B expects colors in GRB order.
    // Green
    led_buf[n++] = timing_bits[0x0f & (rgb >> 12)];
    led_buf[n++] = timing_bits[0x0f & (rgb >> 8)];
    // Red
    led_buf[n++] = timing_bits[0x0f & (rgb >> 20)];
    led_buf[n++] = timing_bits[0x0f & (rgb >> 16)];
    // Blue
    led_buf[n++] = timing_bits[0x0f & (rgb >> 4)];
    led_buf[n++] = timing_bits[0x0f & (rgb)];

    spi_transaction_t t = {
        .length = sizeof(led_buf) * 8,
        .tx_buffer = led_buf,
    };

    esp_err_t ret = spi_device_transmit(spi_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transmission failed");
    }
}

static void led_animation_task(void *pvParameters) {
    led_state_t new_state;

    for (;;) {
        // Check for a new state from the queue without blocking.
        if (xQueueReceive(led_state_queue, &new_state, 0) == pdPASS) {
            if (current_state != new_state) {
                ESP_LOGI(TAG, "Changing LED state to %d", new_state);
                current_state = new_state;
            }
        }

        switch (current_state) {
        case LED_STATE_IDLE: {
            // Breathing effect.
            const uint32_t BREATHING_PERIOD_MS = 3000;
            uint32_t time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

            float angle = (float)(time_ms % BREATHING_PERIOD_MS) /
                          (float)BREATHING_PERIOD_MS * 2.0f * M_PI;

            float brightness_factor = (sinf(angle) + 1.0f) / 2.0f;

            // Apply the brightness factor to the maximum configured brightness.
            uint8_t brightness =
                (uint8_t)(brightness_factor * LED_MAX_BRIGHTNESS);

            ws2812_write_led(scale_color(COLOR_CYAN, brightness));
            vTaskDelay(pdMS_TO_TICKS(20)); // Update rate
            break;
        }
        case LED_STATE_RUNNING: {
            // Solid green
            ws2812_write_led(scale_color(COLOR_GREEN, LED_MAX_BRIGHTNESS));
            vTaskDelay(pdMS_TO_TICKS(250)); // No need to update frequently
            break;
        }
        case LED_STATE_PIN_LIVE: {
            // Pulsing amber at 1Hz. A connector pin stays energised with
            // no client attached, so the board has to say so on its own.
            ws2812_write_led(scale_color(COLOR_AMBER, LED_MAX_BRIGHTNESS));
            vTaskDelay(pdMS_TO_TICKS(750));
            ws2812_write_led(scale_color(COLOR_AMBER, LED_MAX_BRIGHTNESS / 8));
            vTaskDelay(pdMS_TO_TICKS(250));
            break;
        }
        case LED_STATE_PAIRING: {
            // Fast blue blink at 2.5Hz. Bluetooth pairing state
            ws2812_write_led(scale_color(COLOR_BLUE, LED_MAX_BRIGHTNESS));
            vTaskDelay(pdMS_TO_TICKS(200));
            ws2812_write_led(COLOR_OFF);
            vTaskDelay(pdMS_TO_TICKS(200));
            break;
        }
        case LED_STATE_ERROR: {
            // Flashing red at 2Hz (500ms period: 250ms on, 250ms off)
            ws2812_write_led(scale_color(COLOR_RED, LED_MAX_BRIGHTNESS));
            vTaskDelay(pdMS_TO_TICKS(250));
            ws2812_write_led(COLOR_OFF);
            vTaskDelay(pdMS_TO_TICKS(250));
            break;
        }
        case LED_STATE_OFF: {
            ws2812_write_led(COLOR_OFF);
            vTaskDelay(pdMS_TO_TICKS(250));
            break;
        }
        default: { // Solid red
            ws2812_write_led(COLOR_RED);
            vTaskDelay(pdMS_TO_TICKS(250));
            break;
        }
        }
    }
}

void ws2812_led_init(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_WS2812_LED,
        .miso_io_num = -1,
        .sclk_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = sizeof(led_buf),
    };

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = 3200000, // 3.2 MHz clock for WS2812
        .mode = 0,
        .spics_io_num = -1,
        .queue_size = 1,
        .flags = SPI_DEVICE_TXBIT_LSBFIRST,
    };

    // Initialize the SPI bus and add the device
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle));

    // Create the queue for state changes
    led_state_queue = xQueueCreate(5, sizeof(led_state_t));
    if (led_state_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create LED state queue");
        return;
    }

    xTaskCreate(led_animation_task, "led_animation_task", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "Status RGB LED initialized");
}

void ws2812_led_set_state(led_state_t new_state) {
    if (!led_state_queue) {
        return;
    }

    xQueueSend(led_state_queue, &new_state, 0);
}
