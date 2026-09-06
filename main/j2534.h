/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include "vif.h"

#define J2534_FRAGMENT_MAX 192u
#define J2534_RPC_MAX 4608u
#define J2534_DATA_MAX 4128u
#define J2534_PHYSICAL_MAX 3u
#define J2534_LOGICAL_MAX 2u
#define J2534_FILTER_MAX 10u
#define J2534_PERIODIC_MAX 10u

enum {
    J2534_OK = 0,
    J2534_NOT_SUPPORTED = 1,
    J2534_CHANNEL = 2,
    J2534_PROTOCOL = 3,
    J2534_VALUE = 5,
    J2534_FLAGS = 6,
    J2534_FAILED = 7,
    J2534_TIMEOUT = 9,
    J2534_MSG = 10,
    J2534_INTERVAL = 11,
    J2534_LIMIT = 12,
    J2534_MSG_ID = 13,
    J2534_IN_USE = 14,
    J2534_IOCTL = 15,
    J2534_EMPTY = 16,
    J2534_FULL = 17,
    J2534_OVERFLOW = 18,
    J2534_PIN = 19,
    J2534_CONFLICT = 20,
    J2534_MSG_PROTOCOL = 21,
    J2534_FILTER_ID = 22,
    J2534_MSG_NOT_ALLOWED = 23,
    J2534_NOT_UNIQUE = 24,
    J2534_BAUD = 25,
    J2534_DEVICE = 26,
    J2534_NOT_OPEN = 27,
    J2534_FILTER_TYPE = 29,
    J2534_PARAM = 30,
    J2534_VOLTAGE_IN_USE = 31,
    J2534_PIN_IN_USE = 32,
    J2534_INIT = 33,
    J2534_SMALL = 35,
    J2534_LOGICAL = 36,
    J2534_SELECT = 37,
    J2534_DESCRIPTOR = 39,
};

enum {
    J2534_VPW = 1,
    J2534_PWM = 2,
    J2534_ISO9141 = 3,
    J2534_ISO14230 = 4,
    J2534_CAN = 5,
    J2534_ISOTP = 0x200,
};

enum {
    J2534_TX_MSG = 1,
    J2534_START = 2,
    J2534_BREAK = 4,
    J2534_TX_SUCCESS = 8,
    J2534_PADDING = 16,
    J2534_ERROR = 32,
    J2534_RX_OVERFLOW = 64,
    J2534_ADDR = 128,
    J2534_29BIT = 256,
    J2534_TX_FAILED = 512,
    J2534_PAD = 64,
    J2534_WAIT_P3 = 512,
    J2534_CHECKSUM_DISABLED = 512,
    J2534_CAN_BOTH = 2048,
    J2534_K_ONLY = 4096,
};

extern const vif_frontend_t j2534_frontend;
void j2534_register(void);

/* Internal core/transport boundary; all calls run on the VIF link task. */
bool j2534_core_start(void);
void j2534_core_stop(void);
void j2534_core_poll(void);
size_t j2534_core_request(uint8_t op, const uint8_t *in, size_t len,
                          uint8_t out[J2534_RPC_MAX]);

static inline uint32_t j2534_u32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 |
           (uint32_t)p[3] << 24;
}

static inline void j2534_put32(uint8_t *p, uint32_t v) {
    for (unsigned i = 0; i < 4; i++)
        p[i] = (uint8_t)(v >> (8 * i));
}
