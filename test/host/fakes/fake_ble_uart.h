/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_ble_uart.h
 * @brief What a test may ask of the BLE transport stand-in.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#define FAKE_BLE_MAX_NOTES 64
#define FAKE_BLE_NOTE_LEN 40

/** @brief Forget every recorded dispatch. */
void fake_ble_uart_reset(void);

/** @brief How many command lines the parser has dispatched. */
size_t fake_ble_uart_note_count(void);

/** @brief The @p idx'th dispatched command line, or NULL past the end. */
const char *fake_ble_uart_note(size_t idx);
