/* SPDX-License-Identifier: GPL-3.0-only */
#include "ws2812_led.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "pinout.h"
#include <math.h>

/**
 * @brief Maximum brightness for the LED (0-255).
 * * This value scales the brightness of all colors to prevent the LED from being too bright
 * and to reduce power consumption. A value of 128 is a good starting point.
 */
#define LED_MAX_BRIGHTNESS 128

// --- Private Declarations ---

// SPI timing bits for WS2812B protocol.
// Each 4 bits of color data are encoded into a 16-bit SPI word.
static const uint16_t timing_bits[16] = {
    0x1111, 0x7111, 0x1711, 0x7711, 0x1171, 0x7171, 0x1771, 0x7771,
    0x1117, 0x7117, 0x1717, 0x7717, 0x1177, 0x7177, 0x1777, 0x7777};

// SPI device handle
static spi_device_handle_t spi_handle;

// FreeRTOS queue to send state changes to the animation task
static QueueHandle_t led_state_queue;

// Current state of the LED
static led_state_t current_state = LED_STATE_IDLE;

// SPI buffer: 6 words for color data + 2 for reset pulse
static uint16_t led_buf[8] = {0};

// Color definitions
#define COLOR_CYAN  0x00FFFF
#define COLOR_GREEN 0x00FF00
#define COLOR_RED   0xFF0000
#define COLOR_AMBER 0xFF6000
#define COLOR_OFF   0x000000

static const char *TAG = "WS2812_LED";

// --- Private Functions ---

/**
 * @brief Scales a 24-bit RGB color by a brightness value.
 * @param color The input color (0x00RRGGBB).
 * @param brightness The brightness value (0-255).
 * @return The scaled 24-bit RGB color.
 */
static uint32_t scale_color(uint32_t color, uint8_t brightness) {
    uint8_t r = (color >> 16) & 0xFF;
    uint8_t g = (color >> 8) & 0xFF;
    uint8_t b = color & 0xFF;

    r = ((uint16_t)r * brightness) / 255;
    g = ((uint16_t)g * brightness) / 255;
    b = ((uint16_t)b * brightness) / 255;

    return (r << 16) | (g << 8) | b;
}

/**
 * @brief Sends a 24-bit RGB color to the WS2812B LED.
 * This is the low-level function that transmits data over SPI.
 * @param rgb The 24-bit color to set (format: 0x00RRGGBB).
 */
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

/**
 * @brief The main FreeRTOS task for handling LED animations.
 */
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
                // The brightness is modulated using a sine wave for a smooth effect.
                const uint32_t BREATHING_PERIOD_MS = 3000;
                uint32_t time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

                // Calculate the angle for the sine function. The angle completes a full 2*PI cycle
                // every BREATHING_PERIOD_MS. The modulo operator prevents time_ms from overflowing
                // or losing precision over long uptimes.
                float angle = (float)(time_ms % BREATHING_PERIOD_MS) / (float)BREATHING_PERIOD_MS * 2.0f * M_PI;

                // The sine wave gives a value from -1 to 1. We shift and scale it to get a
                // brightness factor from 0.0 to 1.0.
                float brightness_factor = (sinf(angle) + 1.0f) / 2.0f;

                // Apply the brightness factor to the maximum configured brightness.
                uint8_t brightness = (uint8_t)(brightness_factor * LED_MAX_BRIGHTNESS);

                ws2812_write_led(scale_color(COLOR_CYAN, brightness));
                vTaskDelay(pdMS_TO_TICKS(20)); // Update rate for smooth animation
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
            case LED_STATE_ERROR: {
                // Flashing red at 2Hz (500ms period: 250ms on, 250ms off)
                ws2812_write_led(scale_color(COLOR_RED, LED_MAX_BRIGHTNESS));
                vTaskDelay(pdMS_TO_TICKS(250));
                ws2812_write_led(COLOR_OFF);
                vTaskDelay(pdMS_TO_TICKS(250));
                break;
            }
            case LED_STATE_OFF: // Fall through
            default:
                // Turn off LED for any unknown state
                ws2812_write_led(COLOR_OFF);
                vTaskDelay(pdMS_TO_TICKS(250));
                break;
        }
    }
}

// --- Public Functions ---

void ws2812_led_init(void) {
    // Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_WS2812_LED,
        .miso_io_num = -1,
        .sclk_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = sizeof(led_buf),
    };

    // Configuration for the SPI device
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

    // Create the animation task
    xTaskCreate(led_animation_task, "led_animation_task", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "WS2812B LED initialized");
}

void ws2812_led_set_state(led_state_t new_state) {
    // Send the new state to the animation task via the queue
    if (led_state_queue != NULL) {
        xQueueSend(led_state_queue, &new_state, 0);
    }
}
