/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file sdkconfig.h
 * @brief Host stand-in for the generated Kconfig header.
 *
 * Add entries here only when firmware code under test starts reading them, and
 * keep the values matching sdkconfig.defaults.
 */

#pragma once

#define CONFIG_IDF_TARGET_ESP32S3 1
#define CONFIG_FREERTOS_HZ 1000

/* On rather than following sdkconfig.defaults, which leaves it to the
 * compiler optimisation level. The host has no BLE, but it does have
 * fake_ble_uart.c, and building the traced path here is what keeps the
 * dispatch markers a suite can assert on from rotting. */
#define CONFIG_OPENDIAG_BLE_TRACE 1
