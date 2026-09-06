/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_attr.h"

#define J1850_MAX_FRAME 12

uint8_t j1850_crc(const uint8_t *data, size_t len);

/** @brief The residue an error free frame leaves, clause 5.4.1 g. */
#define J1850_CRC_RESIDUE 0xC4

/**
 * @brief True when @p len bytes ending in a CRC byte check out.
 *
 * @param data Frame including its trailing CRC byte.
 * @param len  Byte count including the CRC. Fewer than two is always false:
 *             there is no room for a message under the CRC.
 */
bool j1850_crc_check(const uint8_t *data, size_t len);

#define J1850_HDR_H_BIT 0x10 /**< 0: three byte header. 1: single byte. */
#define J1850_HDR_K_BIT 0x08 /**< 0: in-frame response required. 1: none. */
#define J1850_HDR_Y_BIT 0x04 /**< 0: functional addressing. 1: physical. */

/** @brief True when @p hdr announces a three byte header (H = 0). */
static inline bool j1850_hdr_is_three_byte(uint8_t hdr) {
    return (hdr & J1850_HDR_H_BIT) == 0;
}

/** @brief True when @p hdr asks recipients for an in-frame response (K = 0). */
static inline bool j1850_hdr_wants_ifr(uint8_t hdr) {
    return (hdr & J1850_HDR_K_BIT) == 0;
}
