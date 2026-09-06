/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file j1850_common.c
 * @brief The CRC both SAE J1850 modulations share. No hardware.
 */

#include "j1850_common.h"

uint8_t IRAM_ATTR j1850_crc(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF; /* "all ones state during SOF" */

    if (!data) {
        return 0xFF ^ 0xFF;
    }

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            /* x^8 + x^4 + x^3 + x^2 + 1, the x^8 term being the shift out. */
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x1D)
                               : (uint8_t)(crc << 1);
        }
    }

    return (uint8_t)~crc;
}

bool IRAM_ATTR j1850_crc_check(const uint8_t *data, size_t len) {
    if (!data || len < 2) {
        return false;
    }

    return data[len - 1] == j1850_crc(data, len - 1);
}
