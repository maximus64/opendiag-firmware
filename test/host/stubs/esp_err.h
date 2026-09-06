/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file esp_err.h
 * @brief Host stand-in for the ESP-IDF error type.
 *
 * Only the codes the firmware actually returns are defined; the numeric values
 * match ESP-IDF so a test may compare against either the name or the number.
 */

#pragma once

typedef int esp_err_t;

#define ESP_OK 0
#define ESP_FAIL -1

#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_SIZE 0x104
#define ESP_ERR_NOT_FOUND 0x105
#define ESP_ERR_NOT_SUPPORTED 0x106
#define ESP_ERR_TIMEOUT 0x107

const char *esp_err_to_name(esp_err_t code);

#define ESP_ERROR_CHECK(x)                                                     \
    do {                                                                       \
        esp_err_t err_rc_ = (x);                                               \
        if (err_rc_ != ESP_OK) {                                               \
            idf_stub_abort(__FILE__, __LINE__, "ESP_ERROR_CHECK failed: %s",   \
                           esp_err_to_name(err_rc_));                          \
        }                                                                      \
    } while (0)

/** Reports a fatal stub-level failure and terminates the test binary. */
void idf_stub_abort(const char *file, int line, const char *fmt, ...)
    __attribute__((noreturn, format(printf, 3, 4)));
