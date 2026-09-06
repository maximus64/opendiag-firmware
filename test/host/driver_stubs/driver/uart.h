/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include "freertos/queue.h"
#include "esp_err.h"
typedef int uart_port_t;
typedef int uart_word_length_t;
typedef int uart_parity_t;
enum {
    UART_NUM_1 = 1,
    UART_DATA_7_BITS,
    UART_DATA_8_BITS,
    UART_PARITY_DISABLE,
    UART_PARITY_ODD,
    UART_PARITY_EVEN,
    UART_STOP_BITS_1,
    UART_HW_FLOWCTRL_DISABLE,
    UART_SCLK_XTAL,
    UART_FIFO_OVF,
    UART_BUFFER_FULL,
    UART_BREAK,
    UART_FRAME_ERR,
    UART_PARITY_ERR,
    UART_DATA
};
#define UART_PIN_NO_CHANGE (-1)
typedef struct {
    uart_word_length_t data_bits;
    uart_parity_t parity;
    int stop_bits, flow_ctrl, source_clk, baud_rate;
} uart_config_t;
typedef struct {
    int type;
    size_t size;
    bool timeout_flag;
} uart_event_t;
esp_err_t uart_driver_install(uart_port_t port, int rx_size, int tx_size,
                              int queue_size, QueueHandle_t *queue, int flags);
esp_err_t uart_driver_delete(uart_port_t port);
esp_err_t uart_param_config(uart_port_t port, const uart_config_t *cfg);
esp_err_t uart_set_pin(uart_port_t port, int tx, int rx, int rts, int cts);
esp_err_t uart_set_rx_timeout(uart_port_t port, uint8_t threshold);
esp_err_t uart_set_baudrate(uart_port_t port, uint32_t baud);
esp_err_t uart_set_word_length(uart_port_t port, uart_word_length_t bits);
esp_err_t uart_set_parity(uart_port_t port, uart_parity_t parity);
esp_err_t uart_flush_input(uart_port_t port);
int uart_read_bytes(uart_port_t port, void *buf, uint32_t length,
                    TickType_t wait);
int uart_write_bytes(uart_port_t port, const void *buf, size_t length);
esp_err_t uart_wait_tx_done(uart_port_t port, TickType_t wait);
