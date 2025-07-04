/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "utility.h"
#include "pinout.h"

#include "kline.h"

#define TAG "KLINE"

#define UART_PORT_NUM      UART_NUM_1

static bool g_started;
static int64_t g_time_last_packet;

static void send_5baud_sync(void) {
    // ESP32 S3 UART peripheral cannot do 5 baud so here we bitbang it instead.
    // Baud rate is 5, so the time for one bit is 1/5s = 200ms
    const TickType_t xDelay = pdMS_TO_TICKS(200);

    // Configure GPIO for Output
    gpio_reset_pin(PIN_KLINE_TX);
    gpio_set_direction(PIN_KLINE_TX, GPIO_MODE_OUTPUT);

    // Start Bit (LOW)
    gpio_set_level(PIN_KLINE_TX, 0);
    vTaskDelay(xDelay);

    const uint8_t b = 0x33;
    for (int i = 0; i < 8; i++) {
        uint8_t lv = ((b >> i) & 1);
        gpio_set_level(PIN_KLINE_TX, lv);
        vTaskDelay(xDelay);
    }

    // Stop Bit (HIGH)
    gpio_set_level(PIN_KLINE_TX, 1);
    vTaskDelay(xDelay);

    // Reconfigure GPIO for UART mode
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, PIN_KLINE_TX, PIN_KLINE_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void send_bytes_inter_delay(const uint8_t * buf, size_t len) {
    int rc;

    for (size_t i = 0; i < len; i++) {
        rc = uart_write_bytes(UART_PORT_NUM, &buf[i], 1);
        assert(rc == 1);

        ESP_ERROR_CHECK(uart_wait_tx_done(UART_PORT_NUM, portMAX_DELAY));

        // P4 - inter byte time min 5ms, max 20ms
        delay_us(5000);
    }
}


void kline_setup(void) {
    if (g_started) {
        ESP_LOGI(TAG, "K-Line already started");
        return;
    }

    // Un-mute K-Line driver
    // TODO: we should only unmute when TX
    gpio_set_level(PIN_KLINE_nSILENT, 1);

    ESP_LOGI(TAG, "Initializing UART configuration.");

    //TODO: we need to able to detect baud rate.
    //Spec: Baud rates from 1200 to 10400 Baud are allowed for communication.

    // Configure UART parameters
    uart_config_t uart_config = {
        .baud_rate = 10400,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_XTAL,
    };

    // Install UART driver, and get the queue.
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));

    // Set UART pins (TX, RX, RTS, CTS)
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, PIN_KLINE_TX, PIN_KLINE_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    g_time_last_packet = esp_timer_get_time();

    g_started = true;
}


void kline_teardown(void) {
    if (!g_started) {
        ESP_LOGE(TAG, "K-Line not started yet. Cannot teardown");
        return;
    }

    // Mute K-Line driver
    gpio_set_level(PIN_KLINE_nSILENT, 0);

    ESP_ERROR_CHECK(uart_driver_delete(UART_PORT_NUM));

    g_started = false;
}

int kline_sync(void) {
    uint8_t data[3] = {0};

    ESP_LOGI(TAG, "Send 5 baud init.");
    send_5baud_sync();

    ESP_ERROR_CHECK(uart_flush_input(UART_PORT_NUM));
    int bytes_read = uart_read_bytes(UART_PORT_NUM, data, 3, pdMS_TO_TICKS(500));
    if (bytes_read != 3) {
        ESP_LOGE(TAG, "Synchronisation short read %d", bytes_read);
        return -1;
    }

    if (data[0] != 0x55) {
        ESP_LOGE(TAG, "Synchronisation pattern mismatch %02x", data[0]);
        return -2;
    }

    // data[1] - KB1 - Key Byte 1
    // data[2] - KB2 - Key Byte 2
    const uint8_t kb2 = data[2];

    // Timing W4 - min 25ms
    vTaskDelay(pdMS_TO_TICKS(25));

    // Now we need to send inverse if KB2 for handshake
    data[0] = ~kb2;
    int rc = uart_write_bytes(UART_PORT_NUM, data, 1);
    assert(rc == 1);

    ESP_ERROR_CHECK(uart_wait_tx_done(UART_PORT_NUM, portMAX_DELAY));
    ESP_ERROR_CHECK(uart_flush_input(UART_PORT_NUM));

    // Listen for ECU address inverse
    bytes_read = uart_read_bytes(UART_PORT_NUM, data, 1, pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Read %d bytes: 0x%02X", bytes_read, data[0]);
    const uint8_t ecu_address = ~data[0];
    ESP_LOGI(TAG, "ECU Address: %02x", ecu_address);

    g_time_last_packet = esp_timer_get_time();

    return 0;
}

static uint8_t kline_cal_checksum(const uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum;
}

int kline_send(const uint8_t *data, uint8_t len) {
    // P3 timing - timming between response and next request min 55ms
    int64_t delta, delay_ms;

    while ((delta = esp_timer_get_time() - g_time_last_packet) < 55000) {
        delay_ms = (55000 - delta) / 1000;
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    ESP_LOGI(TAG, "Transmit packet %d bytes.", len);
    ESP_LOG_BUFFER_HEX(TAG, data, len );

    uint8_t cksum = kline_cal_checksum(data, len);
    send_bytes_inter_delay(data, len);
    send_bytes_inter_delay(&cksum, 1);

    g_time_last_packet = esp_timer_get_time();

    return 0;
}


int kline_recieve(uint8_t *data, uint8_t len, TickType_t xTicksToWait) {
    int rc, bytes_read = 0;

    ESP_ERROR_CHECK(uart_wait_tx_done(UART_PORT_NUM, portMAX_DELAY));
    ESP_ERROR_CHECK(uart_flush_input(UART_PORT_NUM));

    for (uint8_t i = 0; i < len; i++) {
        rc = uart_read_bytes(UART_PORT_NUM, &data[i], 1, (i == 0) ? xTicksToWait : pdMS_TO_TICKS(20));
        if (rc != 1) {
            break;
        }
        bytes_read += 1;
    }

    g_time_last_packet = esp_timer_get_time();
    g_time_last_packet -= 20000; //We already spend 20ms for timeout above.

    ESP_LOGI(TAG, "Received %d bytes.", bytes_read);
    ESP_LOG_BUFFER_HEX(TAG, data, bytes_read );

    return bytes_read;
}
