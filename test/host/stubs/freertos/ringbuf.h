/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file ringbuf.h
 * @brief Host stand-in for the ESP-IDF byte-mode ring buffer.
 *
 * Only RINGBUF_TYPE_BYTEBUF is implemented, which is all the firmware uses.
 * The receive side deliberately keeps the real API's quirk of returning a
 * pointer into the buffer that must be handed back with
 * vRingbufferReturnItem(), and of returning less than was asked for when the
 * data wraps. Code that assumes one receive drains everything will fail here
 * the same way it would on target.
 *
 * A receive that finds the buffer empty advances the fake clock by the
 * requested timeout instead of sleeping, so firmware timeout loops terminate
 * in real time while still seeing the elapsed milliseconds they expect.
 */

#pragma once

/* The real header exposes the capability flags used by the WithCaps calls. */
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"

typedef enum {
    RINGBUF_TYPE_NOSPLIT = 0,
    RINGBUF_TYPE_ALLOWSPLIT,
    RINGBUF_TYPE_BYTEBUF,
} RingbufferType_t;

typedef struct ringbuf_stub *RingbufHandle_t;

RingbufHandle_t xRingbufferCreateWithCaps(size_t size, RingbufferType_t type,
                                          uint32_t caps);
void vRingbufferDeleteWithCaps(RingbufHandle_t rb);

BaseType_t xRingbufferSend(RingbufHandle_t rb, const void *data, size_t size,
                           TickType_t timeout);
void *xRingbufferReceiveUpTo(RingbufHandle_t rb, size_t *item_size,
                             TickType_t timeout, size_t wanted);
void vRingbufferReturnItem(RingbufHandle_t rb, void *item);
