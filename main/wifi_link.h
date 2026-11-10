/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * @file wifi_link.h
 * @brief The data link over WiFi, the way WiFi ELM327 adapters offer it.
 *
 * The adapter runs its own access point and listens on TCP port 35000 at
 * 192.168.0.10, which is what OBD-II apps and tools expect of a WiFi
 * adapter. One client at a time; it gets the same data link as USB CDC 0 or
 * BLE, so ELM327, SLCAN (AT VIF SLCAN) and J2534 all work over it.
 *
 * Off by default, and it will not start without a WPA2 password: an open
 * access point in a parked vehicle would hand its buses to anyone in range.
 */

typedef void (*wifi_link_rx_cb_t)(const uint8_t *data, size_t len);
typedef void (*wifi_link_conn_cb_t)(bool connected);

#define WIFI_LINK_PORT 35000
#define WIFI_LINK_PASS_MIN 8
#define WIFI_LINK_PASS_MAX 63

/** @brief Load the settings and start the access point if it is enabled. */
void wifi_link_setup(void);

void wifi_link_set_callbacks(wifi_link_rx_cb_t rx, wifi_link_conn_cb_t conn);

/** @brief Turn the access point on or off now, and remember it. */
esp_err_t wifi_link_set_enabled(bool on);
/** @brief 1-32 characters. Applied at once if running. */
esp_err_t wifi_link_set_ssid(const char *ssid);
/** @brief WIFI_LINK_PASS_MIN-WIFI_LINK_PASS_MAX characters. */
esp_err_t wifi_link_set_password(const char *pass);

/** @brief Settings and state, never the password. */
void wifi_link_print_status(void);

bool wifi_link_is_connected(void);
int wifi_link_write(const void *buf, uint32_t len);
