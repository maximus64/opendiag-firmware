/* SPDX-License-Identifier: GPL-3.0-only */
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "j2534.h"

#define WIRE_MAGIC "J253"
#define WIRE_MAGIC_SIZE 4u
#define WIRE_VERSION 1u
#define WIRE_HEADER_SIZE 16u
#define WIRE_CRC_SIZE 4u
#define WIRE_FRAME_MAX (WIRE_HEADER_SIZE + J2534_FRAGMENT_MAX + WIRE_CRC_SIZE)
#define WIRE_TIMEOUT_US 2000000u

/* Header fields after the four-byte magic; multibyte fields are little-endian.
 */
enum {
    WIRE_VERSION_OFFSET = 4,
    WIRE_OPERATION_OFFSET = 5,
    WIRE_FLAGS_OFFSET = 6,
    WIRE_SEQUENCE_OFFSET = 8,
    WIRE_FRAGMENT_OFFSET = 12,
    WIRE_LENGTH_OFFSET = 14,
};

enum {
    WIRE_MORE = 1,
    WIRE_FETCH = 2,
    WIRE_ACK = 4,
    WIRE_RESPONSE = 128,
};

static struct {
    uint8_t frame[WIRE_FRAME_MAX];
    uint8_t request[J2534_RPC_MAX];
    uint8_t response[J2534_RPC_MAX];
    size_t frame_used;
    size_t request_len;
    size_t response_len;
    uint32_t sequence;
    uint32_t last_input_us;
    uint8_t operation;
    bool assembling_request;
    bool response_ready;
} wire;

static uint16_t get_le16(const uint8_t *data) {
    return data[0] | (uint16_t)data[1] << 8;
}

static void put_le16(uint8_t *data, unsigned value) {
    data[0] = value;
    data[1] = value >> 8;
}

static uint32_t crc32(const uint8_t *data, size_t len) {
    uint32_t crc = UINT32_MAX;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; bit++)
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
    }

    return ~crc;
}

static uint32_t frame_crc(const uint8_t *frame, size_t payload_len) {
    return crc32(frame + WIRE_MAGIC_SIZE,
                 WIRE_HEADER_SIZE - WIRE_MAGIC_SIZE + payload_len);
}

static void send_fragment(uint8_t flags, size_t offset, const uint8_t *data,
                          size_t len) {
    uint8_t frame[WIRE_FRAME_MAX] = {'J', '2',          '5',
                                     '3', WIRE_VERSION, wire.operation};

    put_le16(frame + WIRE_FLAGS_OFFSET, WIRE_RESPONSE | flags);
    j2534_put32(frame + WIRE_SEQUENCE_OFFSET, wire.sequence);
    put_le16(frame + WIRE_FRAGMENT_OFFSET, offset);
    put_le16(frame + WIRE_LENGTH_OFFSET, len);
    if (len)
        memcpy(frame + WIRE_HEADER_SIZE, data, len);

    j2534_put32(frame + WIRE_HEADER_SIZE + len, frame_crc(frame, len));
    vif_link_write(frame, WIRE_HEADER_SIZE + len + WIRE_CRC_SIZE);
    vif_link_flush();
}

static void send_response_fragment(size_t offset) {
    size_t len;
    uint8_t flags;

    if (offset > wire.response_len)
        return;

    len = wire.response_len - offset;
    if (len > J2534_FRAGMENT_MAX)
        len = J2534_FRAGMENT_MAX;

    flags = offset + len < wire.response_len ? WIRE_MORE : 0;
    send_fragment(flags, offset, wire.response + offset, len);
}

static void reject_request(void) {
    wire.assembling_request = false;
    /* Protobuf Response.status: field 1, varint, followed by the error code. */
    wire.response[0] = 8;
    wire.response[1] = J2534_MSG;
    wire.response_len = 2;
    wire.response_ready = true;
    send_response_fragment(0);
}

static void process_frame(void) {
    const uint8_t *frame = wire.frame;
    unsigned flags = get_le16(frame + WIRE_FLAGS_OFFSET);
    unsigned offset = get_le16(frame + WIRE_FRAGMENT_OFFSET);
    unsigned len = get_le16(frame + WIRE_LENGTH_OFFSET);
    uint32_t sequence = j2534_u32(frame + WIRE_SEQUENCE_OFFSET);
    uint8_t operation = frame[WIRE_OPERATION_OFFSET];

    if (j2534_u32(frame + WIRE_HEADER_SIZE + len) != frame_crc(frame, len))
        return;

    if (flags == WIRE_FETCH) {
        if (!len && wire.response_ready && sequence == wire.sequence &&
            operation == wire.operation)
            send_response_fragment(offset);
        return;
    }

    if (flags & ~(unsigned)WIRE_MORE)
        return;

    if (!offset) {
        wire.request_len = 0;
        wire.sequence = sequence;
        wire.operation = operation;
        wire.assembling_request = true;
        wire.response_ready = false;
    }

    if (!wire.assembling_request || sequence != wire.sequence ||
        operation != wire.operation || offset != wire.request_len)
        return;

    if (len > J2534_RPC_MAX - wire.request_len ||
        ((flags & WIRE_MORE) && !len)) {
        reject_request();
        return;
    }

    memcpy(wire.request + offset, frame + WIRE_HEADER_SIZE, len);
    wire.request_len += len;
    if (flags & WIRE_MORE) {
        send_fragment(WIRE_ACK, wire.request_len, NULL, 0);
        return;
    }

    wire.assembling_request = false;
    wire.response_len = j2534_core_request(wire.operation, wire.request,
                                           wire.request_len, wire.response);
    wire.response_ready = true;
    send_response_fragment(0);
}

static void feed(const uint8_t *data, size_t len) {
    uint32_t current_us = (uint32_t)esp_timer_get_time();

    if ((uint32_t)(current_us - wire.last_input_us) > WIRE_TIMEOUT_US) {
        wire.frame_used = 0;
        wire.assembling_request = false;
    }
    wire.last_input_us = current_us;

    for (size_t i = 0; i < len; i++) {
        unsigned payload_len;

        wire.frame[wire.frame_used++] = data[i];
        while (wire.frame_used &&
               memcmp(wire.frame, WIRE_MAGIC,
                      wire.frame_used < WIRE_MAGIC_SIZE ? wire.frame_used
                                                        : WIRE_MAGIC_SIZE)) {
            wire.frame_used--;
            memmove(wire.frame, wire.frame + 1, wire.frame_used);
        }

        if (wire.frame_used < WIRE_HEADER_SIZE)
            continue;

        payload_len = get_le16(wire.frame + WIRE_LENGTH_OFFSET);
        if (wire.frame[WIRE_VERSION_OFFSET] != WIRE_VERSION ||
            payload_len > J2534_FRAGMENT_MAX) {
            wire.frame_used = 0;
            continue;
        }

        if (wire.frame_used == WIRE_HEADER_SIZE + payload_len + WIRE_CRC_SIZE) {
            process_frame();
            wire.frame_used = 0;
        }
    }
}

static bool start(void) {
    memset(&wire, 0, sizeof(wire));
    return j2534_core_start();
}

static void stop(void) {
    j2534_core_stop();
    memset(&wire, 0, sizeof(wire));
}

const vif_frontend_t j2534_frontend = {
    .name = "j2534",
    .start = start,
    .feed = feed,
    .poll = j2534_core_poll,
    .stop = stop,
};

void j2534_register(void) {
    if (vif_frontend_register(&j2534_frontend) != ESP_OK)
        ESP_LOGE("J2534", "front-end registration failed");
}
