/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file esp_log.h
 * @brief Host stand-in for the ESP-IDF logging macros.
 *
 * Logs are suppressed by default so a passing run is quiet. Set TD_VERBOSE=1
 * in the environment to see them, which is usually the fastest way to find out
 * what a failing firmware path decided to do.
 */

#pragma once

#include <stdio.h>

/** True when TD_VERBOSE is set in the environment. */
int idf_stub_log_enabled(void);

#define IDF_STUB_LOG(level, tag, fmt, ...)                                     \
    do {                                                                       \
        if (idf_stub_log_enabled()) {                                          \
            fprintf(stderr, "%c (%s) " fmt "\n", level, tag, ##__VA_ARGS__);   \
        }                                                                      \
    } while (0)

#define ESP_LOGE(tag, fmt, ...) IDF_STUB_LOG('E', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) IDF_STUB_LOG('W', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) IDF_STUB_LOG('I', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) IDF_STUB_LOG('D', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGV(tag, fmt, ...) IDF_STUB_LOG('V', tag, fmt, ##__VA_ARGS__)
