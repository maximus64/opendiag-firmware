/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include "vif.h"

/* Payload per wire frame: a 258-byte flash block, with its message header,
 * fits one frame. The host sends a frame only once the last is answered, so
 * a link's receive buffer must hold one whole frame: this, 16 bytes of header
 * and a 4-byte CRC. */
#define J2534_FRAGMENT_MAX 1024u
#define J2534_WIRE_FRAME_MAX (J2534_FRAGMENT_MAX + 20u)
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

#define J2534_REPEAT_MAX 10u

typedef struct {
    uint32_t protocol;
    uint32_t flags;
    uint32_t handle;
    uint32_t len;
    uint8_t data[12];
} j2534_repeat_message_t;

typedef struct {
    uint32_t interval_ms;
    uint32_t condition; /* 0: until matching RX; 1: while matching RX. */
    j2534_repeat_message_t message;
    j2534_repeat_message_t mask;
    j2534_repeat_message_t pattern;
} j2534_repeat_setup_t;

/* Internal firmware API; the wire and DLL entry points are added in Phase 2. */
uint32_t j2534_core_repeat_start(uint32_t channel,
                                 const j2534_repeat_setup_t *setup,
                                 uint32_t *id);
uint32_t j2534_core_repeat_query(uint32_t channel, uint32_t id, bool *active);
uint32_t j2534_core_repeat_stop(uint32_t channel, uint32_t id);

extern const vif_frontend_t j2534_frontend;
void j2534_register(void);

/* Internal core/transport boundary; all calls run on the VIF link task. */
bool j2534_core_start(void);
void j2534_core_stop(void);
void j2534_core_poll(void);
/* The next instant poll() has work for without input or bus events. */
bool j2534_core_next_wake(uint32_t *due_us);
/* Rises whenever poll() acts; see vif_frontend_t.progress. */
uint32_t j2534_core_progress(void);
/* RPC contract revision, Capabilities.wire_version. 2: Read and Select wait
 * on the device. 3: a WAKE frame ends that wait early. */
#define J2534_PROTOCOL_REVISION 3u
/* Longest a Read or Select may wait on the device, as J2534 allows. */
#define J2534_WAIT_MAX_MS 30000u
/* j2534_core_request() result for a request answered later, from
 * j2534_core_take_held(). */
#define J2534_CORE_HELD SIZE_MAX
size_t j2534_core_request(uint8_t op, const uint8_t *in, size_t len,
                          uint8_t out[J2534_RPC_MAX]);
/* After each poll: the held request's answer once it has one or times out. */
bool j2534_core_take_held(uint8_t out[J2534_RPC_MAX], size_t *len);
/* The host has moved on, so a held request must not be answered. */
void j2534_core_cancel_held(void);
/* Another caller needs the link: answer the held request on the next poll,
 * as though its timeout had run out. */
void j2534_core_wake_held(void);

static inline uint32_t j2534_u32(const uint8_t *p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 |
           (uint32_t)p[3] << 24;
}

static inline void j2534_put32(uint8_t *p, uint32_t v) {
    for (unsigned i = 0; i < 4; i++)
        p[i] = (uint8_t)(v >> (8 * i));
}
