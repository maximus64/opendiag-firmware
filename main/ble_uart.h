/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "common.h"

/**
 * @brief One control command, as written to the control characteristic.
 *
 * GATT frames each write as a whole message, so @p cmd is one complete
 * command and is not NUL terminated.
 */
typedef void (*ble_uart_ctrl_cb_t)(const char *cmd, size_t len);

/** @brief A central connected or went away. */
typedef void (*ble_uart_conn_cb_t)(bool connected);

void ble_uart_setup(void);
void ble_uart_send (const char *buffer, size_t size);
void ble_uart_rx_set_callback(interface_rx_cb_t callback);
bool ble_uart_is_connected(void);

/** @brief Answer a control command. Split to fit the negotiated ATT MTU. */
void ble_uart_ctrl_reply(const char *text);

void ble_uart_ctrl_set_callback(ble_uart_ctrl_cb_t cb);
void ble_uart_set_conn_callback(ble_uart_conn_cb_t cb);
