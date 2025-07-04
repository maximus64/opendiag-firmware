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
