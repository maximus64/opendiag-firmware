/* SPDX-License-Identifier: GPL-3.0-only */
#include "comm_iface.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "ble_uart.h"

static const char *TAG = "comm_iface";

typedef struct {
    bool active;
    char name[COMM_NAME_LEN];
    comm_port_ops_t ops;
    RingbufHandle_t rx_stream; /* Bytes received, waiting for the session */
    uint32_t rx_dropped;       /* Chunks lost to a full rx_stream */
    uint32_t rx_discarded;     /* Bytes dropped when a client went away */
    size_t rx_pending;         /* Received and not yet read */
    bool muted;                /* Client gone; nothing may be written yet */
} port_t;

static struct {
    bool initialized;
    SemaphoreHandle_t lock; /* Protects the table below */
    port_t ports[COMM_MAX_PORTS];
} g_comm;

#define LOCK() xSemaphoreTake(g_comm.lock, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(g_comm.lock)

static bool port_valid(comm_port_id_t id) {
    return id < COMM_MAX_PORTS && g_comm.ports[id].active;
}

static void copy_name(char *dst, const char *src) {
    strncpy(dst, src ? src : "?", COMM_NAME_LEN - 1);
    dst[COMM_NAME_LEN - 1] = '\0';
}

esp_err_t comm_iface_init(void) {
    if (g_comm.initialized) {
        return ESP_OK;
    }

    g_comm.lock = xSemaphoreCreateMutex();
    if (!g_comm.lock) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    memset(g_comm.ports, 0, sizeof(g_comm.ports));

    g_comm.initialized = true;

    return ESP_OK;
}

comm_port_id_t comm_port_register(const char *name, const comm_port_ops_t *ops,
                                  size_t rx_buf_size) {
    comm_port_id_t id = COMM_INVALID_PORT_ID;
    RingbufHandle_t stream;

    if (!g_comm.initialized || !ops || !ops->write || rx_buf_size == 0) {
        ESP_LOGE(TAG, "port '%s': bad arguments", name ? name : "?");
        return COMM_INVALID_PORT_ID;
    }

    /* Allocate before taking the lock so a failure costs nothing. */
    stream = xRingbufferCreateWithCaps(rx_buf_size, RINGBUF_TYPE_BYTEBUF,
                                       MALLOC_CAP_DEFAULT);
    if (!stream) {
        ESP_LOGE(TAG, "port '%s': cannot allocate %u byte rx buffer",
                 name ? name : "?", (unsigned)rx_buf_size);
        return COMM_INVALID_PORT_ID;
    }

    LOCK();
    for (comm_port_id_t i = 0; i < COMM_MAX_PORTS; i++) {
        if (!g_comm.ports[i].active) {
            id = i;
            break;
        }
    }

    if (id != COMM_INVALID_PORT_ID) {
        port_t *port = &g_comm.ports[id];
        port->active = true;
        port->ops = *ops;
        port->rx_stream = stream;
        port->rx_dropped = 0;
        port->rx_discarded = 0;
        port->rx_pending = 0;
        port->muted = false;
        copy_name(port->name, name);
    }
    UNLOCK();

    if (id == COMM_INVALID_PORT_ID) {
        vRingbufferDeleteWithCaps(stream);
        ESP_LOGE(TAG, "port '%s': no free slots", name ? name : "?");
    } else {
        ESP_LOGI(TAG, "port '%s' registered as %d (%u byte rx buffer)",
                 g_comm.ports[id].name, id, (unsigned)rx_buf_size);
    }

    return id;
}

esp_err_t comm_port_rx(comm_port_id_t port_id, const uint8_t *data,
                       size_t length) {
    port_t *port;

    if (!g_comm.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!data || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    LOCK();
    if (!port_valid(port_id)) {
        UNLOCK();
        return ESP_ERR_INVALID_ARG;
    }

    port = &g_comm.ports[port_id];

    /* Non-blocking on purpose. We are running in the transport's task, so
     * waiting here would stall USB or BLE for every other consumer. */
    if (xRingbufferSend(port->rx_stream, data, length, 0) == pdTRUE) {
        port->rx_pending += length;
    } else {
        port->rx_dropped++;
        ESP_LOGW(TAG,
                 "port '%s': rx buffer full, dropped %u bytes (%" PRIu32
                 " total)",
                 port->name, (unsigned)length, port->rx_dropped);
    }
    UNLOCK();

    return ESP_OK;
}

size_t comm_port_read(comm_port_id_t port_id, uint8_t *buf, size_t length,
                      TickType_t timeout) {
    RingbufHandle_t stream;
    size_t size = 0;
    void *item;

    if (!g_comm.initialized || !buf || length == 0) {
        return 0;
    }

    LOCK();
    stream = port_valid(port_id) ? g_comm.ports[port_id].rx_stream : NULL;
    UNLOCK();

    if (!stream) {
        return 0;
    }

    /* Block outside the lock so a waiting session never holds up a transport */
    item = xRingbufferReceiveUpTo(stream, &size, timeout, length);
    if (!item) {
        return 0;
    }

    memcpy(buf, item, size);
    vRingbufferReturnItem(stream, item);

    LOCK();
    if (port_valid(port_id)) {
        port_t *port = &g_comm.ports[port_id];

        port->rx_pending -= size < port->rx_pending ? size : port->rx_pending;
    }
    UNLOCK();

    return size;
}

size_t comm_port_rx_pending(comm_port_id_t port_id) {
    size_t pending = 0;

    if (!g_comm.initialized) {
        return 0;
    }

    LOCK();
    if (port_valid(port_id)) {
        pending = g_comm.ports[port_id].rx_pending;
    }
    UNLOCK();

    return pending;
}

/** @brief Throw away everything the port has received and nobody has read. */
static uint32_t rx_stream_drain(RingbufHandle_t stream) {
    uint32_t dropped = 0;
    size_t size;
    void *item;

    /* The byte buffer hands back a pointer into itself and stops at the wrap,
     * so one receive is not one drain. SIZE_MAX rather than the ring buffer's
     * own "0 means everything contiguous": zero is indistinguishable from a
     * caller asking for nothing, and this asks for as much as there is. */
    while ((item = xRingbufferReceiveUpTo(stream, &size, 0, SIZE_MAX)) !=
           NULL) {
        dropped += (uint32_t)size;
        vRingbufferReturnItem(stream, item);
    }

    return dropped;
}

void comm_port_client_gone(comm_port_id_t port_id) {
    port_t *port;
    uint32_t dropped;

    if (!g_comm.initialized) {
        return;
    }

    LOCK();
    if (!port_valid(port_id)) {
        UNLOCK();
        return;
    }

    port = &g_comm.ports[port_id];
    dropped = rx_stream_drain(port->rx_stream);
    port->rx_discarded += dropped;
    port->rx_pending = 0;
    port->muted = true;
    UNLOCK();

    if (dropped) {
        ESP_LOGI(TAG,
                 "port '%s': client gone, %" PRIu32 " unread byte(s) dropped",
                 g_comm.ports[port_id].name, dropped);
    }
}

void comm_port_client_ready(comm_port_id_t port_id) {
    if (!g_comm.initialized) {
        return;
    }

    LOCK();
    if (port_valid(port_id)) {
        g_comm.ports[port_id].muted = false;
    }
    UNLOCK();
}

/**
 * @brief Write to one port, retrying the remainder once through a flush.
 *
 * tinyusb_cdcacm_write_queue() accepts only what fits in its FIFO, so a large
 * response would otherwise be silently truncated when the host is slow.
 */
static bool port_write_all(const comm_port_ops_t *ops, const uint8_t *data,
                           size_t length) {
    int written = ops->write(data, length);

    if (written <= 0) {
        return false;
    }

    if ((size_t)written < length && ops->flush) {
        ops->flush();
        int more = ops->write(data + written, length - written);
        if (more > 0) {
            written += more;
        }
    }

    if ((size_t)written < length) {
        return false;
    }

    return true;
}

esp_err_t comm_port_write(comm_port_id_t port_id, const void *data,
                          size_t length) {
    comm_port_ops_t ops;
    bool muted;

    if (!g_comm.initialized || !data || length == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Snapshot the vtable under the lock, write with it released: a transport
     * that blocks must not hold every other port up behind it. */
    LOCK();
    if (!port_valid(port_id)) {
        UNLOCK();
        return ESP_ERR_INVALID_ARG;
    }
    ops = g_comm.ports[port_id].ops;
    muted = g_comm.ports[port_id].muted;
    UNLOCK();

    /*
     * The client this was composed for has gone. It is the tail of that
     * client's conversation - very often the prompt that ends it - and the
     * one who has connected since did not ask for it and cannot make sense of
     * it. See comm_port_client_gone().
     */
    if (muted) {
        return ESP_ERR_NOT_FOUND;
    }

    /*
     * Nothing is written to a port with no client. tinyusb_cdcacm_write_queue()
     * would happily fill a FIFO that nobody is draining, and the answer to a
     * request that arrived before the client left is not worth that.
     */
    if (ops.is_connected && !ops.is_connected()) {
        return ESP_ERR_NOT_FOUND;
    }

    return port_write_all(&ops, data, length) ? ESP_OK : ESP_FAIL;
}

void comm_port_flush(comm_port_id_t port_id) {
    void (*flush)(void) = NULL;

    if (!g_comm.initialized) {
        return;
    }

    LOCK();
    if (port_valid(port_id)) {
        flush = g_comm.ports[port_id].ops.flush;
    }
    UNLOCK();

    if (flush) {
        flush();
    }
}

uint32_t comm_port_rx_dropped(comm_port_id_t port_id) {
    uint32_t dropped = 0;

    if (!g_comm.initialized) {
        return 0;
    }

    LOCK();
    if (port_valid(port_id)) {
        dropped = g_comm.ports[port_id].rx_dropped;
    }
    UNLOCK();

    return dropped;
}

void comm_print_debug_info(void) {
    if (!g_comm.initialized) {
        printf("comm_iface not initialized\n");
        return;
    }

    LOCK();

    printf("Ports:\n");
    for (int i = 0; i < COMM_MAX_PORTS; i++) {
        port_t *port = &g_comm.ports[i];
        bool connected;

        if (!port->active) {
            continue;
        }

        connected = port->ops.is_connected ? port->ops.is_connected() : true;
        printf("  [%d] %-8s connected:%-4s muted:%-4s rx_dropped:%" PRIu32
               " rx_discarded:%" PRIu32 " backlog:%u\n",
               i, port->name, connected ? "yes" : "no",
               port->muted ? "yes" : "no", port->rx_dropped, port->rx_discarded,
               (unsigned)port->rx_pending);

        /* BLE is the one transport that can lose part of a response rather
         * than all of it, and a truncated ELM327 reply is the hardest fault
         * on this device to diagnose from the client's end - it looks like a
         * flaky bus. So the count is here rather than nowhere. */
        if (strcasecmp(port->name, "ble") == 0) {
            uint32_t dropped = 0, retries = 0;

            uint32_t rx_bytes = 0, widest = 0, refused = 0;

            ble_uart_get_notify_stats(&dropped, &retries);
            ble_uart_get_rx_stats(&rx_bytes, &widest, &refused);
            printf("           notify_dropped:%" PRIu32
                   " notify_retries:%" PRIu32 "\n",
                   dropped, retries);
            printf("           rx_bytes:%" PRIu32 " widest_write:%" PRIu32
                   " refused:%" PRIu32 "\n",
                   rx_bytes, widest, refused);
        }
    }

    UNLOCK();
}
