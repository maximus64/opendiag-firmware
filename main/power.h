/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "power_policy.h"

/**
 * @file power.h
 * @brief Deep sleep for an adapter left plugged into a parked vehicle.
 *
 * With nobody connected, nothing claimed, no CAN traffic and the engine off
 * for the idle time, the adapter goes into deep sleep with every output held
 * at its idle level. CAN activity on either transceiver or the button wakes
 * it, which is a full restart. A timer also wakes it now and then to look at
 * the battery: a charging voltage means the engine is running, and it stays
 * up; otherwise it goes straight back to sleep before USB or BLE start.
 */

/**
 * @brief Call once, right after vif_init().
 *
 * Releases the pins held through the last sleep, and after a timer wake-up
 * with the engine still off, goes back to sleep without returning.
 */
void power_boot(void);

/** @brief Start watching for idle. Call once the transports are up. */
esp_err_t power_start(void);

/** @brief The settings in force. */
power_cfg_t power_cfg(void);

/** @brief Change and store the settings. */
esp_err_t power_set_enabled(bool on);
esp_err_t power_set_idle_minutes(uint32_t minutes);
esp_err_t power_set_run_mv(int32_t mv);

/** @brief Print the settings, the last tick and the last wake-up cause. */
void power_print_status(void);

/**
 * @brief Sleep now, unless a client, a claim or a pin is active.
 * @return ESP_ERR_INVALID_STATE when something is in use; otherwise it does
 *         not return.
 */
esp_err_t power_sleep_now(void);
