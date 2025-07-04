/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file idf_stub.c
 * @brief Host implementations behind the ESP-IDF stub headers.
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "fake_clock.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

/* ------------------------------------------------------------------ *
 * Failure reporting
 * ------------------------------------------------------------------ */

void idf_stub_abort(const char *file, int line, const char *fmt, ...)
{
    va_list ap;

    fprintf(stderr, "\nIDF STUB FATAL %s:%d: ", file, line);
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fprintf(stderr, "\n");
    fflush(stderr);

    abort();
}

const char *esp_err_to_name(esp_err_t code)
{
    switch (code) {
    case ESP_OK:                  return "ESP_OK";
    case ESP_FAIL:                return "ESP_FAIL";
    case ESP_ERR_NO_MEM:          return "ESP_ERR_NO_MEM";
    case ESP_ERR_INVALID_ARG:     return "ESP_ERR_INVALID_ARG";
    case ESP_ERR_INVALID_STATE:   return "ESP_ERR_INVALID_STATE";
    case ESP_ERR_INVALID_SIZE:    return "ESP_ERR_INVALID_SIZE";
    case ESP_ERR_NOT_FOUND:       return "ESP_ERR_NOT_FOUND";
    case ESP_ERR_NOT_SUPPORTED:   return "ESP_ERR_NOT_SUPPORTED";
    case ESP_ERR_TIMEOUT:         return "ESP_ERR_TIMEOUT";
    default:                      return "ESP_ERR_UNKNOWN";
    }
}

int idf_stub_log_enabled(void)
{
    static int cached = -1;

    if (cached < 0) {
        const char *v = getenv("TD_VERBOSE");
        cached = (v && *v && strcmp(v, "0") != 0) ? 1 : 0;
    }

    return cached;
}

/* ------------------------------------------------------------------ *
 * Fake clock
 * ------------------------------------------------------------------ */

static uint32_t g_now_ms;
static uint32_t g_now_sub_us;

uint32_t fake_clock_ms(void)
{
    return g_now_ms;
}

void fake_clock_advance_ms(uint32_t ms)
{
    g_now_ms += ms;
}

void fake_clock_reset(void)
{
    g_now_ms = 0;
    g_now_sub_us = 0;
}

int64_t esp_timer_get_time(void)
{
    /* delay_us() busy-waits on this, so every read has to move forward or the
     * caller never leaves the loop. One microsecond per read is enough, and it
     * keeps the microsecond clock consistent with the millisecond one. */
    g_now_sub_us++;
    return (int64_t)g_now_ms * 1000 + g_now_sub_us;
}

/* ------------------------------------------------------------------ *
 * Tasks
 * ------------------------------------------------------------------ */

static int g_task_starts;

BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint32_t stack_depth,
                       void *param, UBaseType_t priority, TaskHandle_t *created)
{
    (void)fn;
    (void)name;
    (void)param;
    (void)stack_depth;
    (void)priority;

    if (created) {
        *created = NULL;
    }

    g_task_starts++;

    /* Not started. Tests drive the code a task would run. */
    return pdPASS;
}

int idf_stub_task_starts(void)
{
    return g_task_starts;
}

void idf_stub_reset_task_starts(void)
{
    g_task_starts = 0;
}

TickType_t xTaskGetTickCount(void)
{
    return (TickType_t)g_now_ms;
}

void vTaskDelay(TickType_t ticks)
{
    fake_clock_advance_ms((uint32_t)ticks);
}

void vTaskDelete(TaskHandle_t task)
{
    /* xTaskCreate() never started anything, so a task deleting itself at the
     * end of its loop has nothing to unwind here. */
    (void)task;
}

void vPortYield(void)
{
}

/* ------------------------------------------------------------------ *
 * Mutexes
 * ------------------------------------------------------------------ */

static int g_lock_balance;
static int g_mutex_token;

SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    /* Any distinct non-NULL pointer will do; nothing dereferences it. */
    return (SemaphoreHandle_t)&g_mutex_token;
}

void vSemaphoreDelete(SemaphoreHandle_t sem)
{
    (void)sem;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout)
{
    (void)timeout;

    if (!sem) {
        idf_stub_abort(__FILE__, __LINE__, "take on a NULL semaphore");
    }

    g_lock_balance++;
    return pdTRUE;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t sem)
{
    if (!sem) {
        idf_stub_abort(__FILE__, __LINE__, "give on a NULL semaphore");
    }

    g_lock_balance--;
    return pdTRUE;
}

int idf_stub_lock_balance(void)
{
    return g_lock_balance;
}

/* ------------------------------------------------------------------ *
 * Byte-mode ring buffer
 * ------------------------------------------------------------------ */

struct ringbuf_stub {
    uint8_t *buf;
    size_t cap;
    size_t head;     /* Write cursor */
    size_t tail;     /* Read cursor */
    size_t count;    /* Bytes held */
    size_t lent;     /* Bytes handed out and not yet returned */
};

RingbufHandle_t xRingbufferCreateWithCaps(size_t size, RingbufferType_t type,
                                          uint32_t caps)
{
    struct ringbuf_stub *rb;

    (void)caps;

    if (type != RINGBUF_TYPE_BYTEBUF) {
        idf_stub_abort(__FILE__, __LINE__,
                       "only RINGBUF_TYPE_BYTEBUF is stubbed (got %d)", type);
    }
    if (size == 0) {
        return NULL;
    }

    rb = calloc(1, sizeof(*rb));
    if (!rb) {
        return NULL;
    }

    rb->buf = calloc(1, size);
    if (!rb->buf) {
        free(rb);
        return NULL;
    }

    rb->cap = size;
    return rb;
}

void vRingbufferDeleteWithCaps(RingbufHandle_t rb)
{
    if (!rb) {
        return;
    }
    free(rb->buf);
    free(rb);
}

BaseType_t xRingbufferSend(RingbufHandle_t rb, const void *data, size_t size,
                           TickType_t timeout)
{
    const uint8_t *src = data;

    (void)timeout;

    if (!rb || !data || size == 0) {
        return pdFALSE;
    }

    if (rb->count + size > rb->cap) {
        /* A blocking send into a full buffer is a deadlock on target unless
         * some *other* task drains it. Tests are single threaded, so if the
         * caller is also the drainer it would wait on itself forever. Say so
         * loudly rather than quietly dropping the data, which would hide a
         * target hang behind a merely truncated host result. */
        if (timeout == portMAX_DELAY) {
            idf_stub_abort(__FILE__, __LINE__,
                           "blocking send of %zu bytes into a full %zu byte "
                           "ring buffer: this deadlocks on target",
                           size, rb->cap);
        }
        return pdFALSE;
    }

    for (size_t i = 0; i < size; i++) {
        rb->buf[rb->head] = src[i];
        rb->head = (rb->head + 1) % rb->cap;
    }
    rb->count += size;

    return pdTRUE;
}

void *xRingbufferReceiveUpTo(RingbufHandle_t rb, size_t *item_size,
                             TickType_t timeout, size_t wanted)
{
    size_t n;

    if (item_size) {
        *item_size = 0;
    }

    if (!rb || wanted == 0) {
        return NULL;
    }

    if (rb->lent) {
        idf_stub_abort(__FILE__, __LINE__,
                       "receive while %zu bytes are still lent out; the "
                       "previous item was never returned", rb->lent);
    }

    if (rb->count == 0) {
        /* This is where a real task would have blocked. Charge the wait to the
         * fake clock so the caller's timeout loop makes progress. */
        if (timeout == portMAX_DELAY) {
            idf_stub_abort(__FILE__, __LINE__,
                           "blocking receive with portMAX_DELAY on an empty "
                           "buffer would never return under test");
        }
        fake_clock_advance_ms((uint32_t)timeout);
        return NULL;
    }

    n = rb->count < wanted ? rb->count : wanted;

    /* Return a contiguous run, as the real byte buffer does. A caller that
     * assumes one receive drains everything breaks here too. */
    if (rb->tail + n > rb->cap) {
        n = rb->cap - rb->tail;
    }

    rb->lent = n;
    if (item_size) {
        *item_size = n;
    }

    return &rb->buf[rb->tail];
}

void vRingbufferReturnItem(RingbufHandle_t rb, void *item)
{
    (void)item;

    if (!rb || rb->lent == 0) {
        return;
    }

    rb->tail = (rb->tail + rb->lent) % rb->cap;
    rb->count -= rb->lent;
    rb->lent = 0;
}

