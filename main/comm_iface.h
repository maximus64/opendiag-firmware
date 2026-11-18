/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"

#define COMM_MAX_PORTS 4
#define COMM_NAME_LEN 8

#define COMM_INVALID_PORT_ID 0xFF

typedef uint8_t comm_port_id_t;

/**
 * @brief Operations a physical transport must provide.
 */
typedef struct {
    /** Write to the transport. Returns bytes accepted, or negative on error. */
    int (*write)(const void *buf, uint32_t length);
    /** Push any buffered data out. May be NULL. */
    void (*flush)(void);
    /** True when a client is attached. May be NULL, meaning always connected.
     */
    bool (*is_connected)(void);
} comm_port_ops_t;

/**
 * @brief Initialize the transport table. Call before any other comm_* function.
 */
esp_err_t comm_iface_init(void);

/**
 * @brief Register a physical transport.
 *
 * @param name        Short name, truncated to COMM_NAME_LEN - 1 characters.
 * @param ops         Operations table; copied, so it need not outlive the call.
 * @param rx_buf_size Size in bytes of this port's receive buffer.
 * @return Port ID, or COMM_INVALID_PORT_ID on failure.
 */
comm_port_id_t comm_port_register(const char *name, const comm_port_ops_t *ops,
                                  size_t rx_buf_size);

/**
 * @brief Deliver received bytes from a transport.
 *
 * Never blocks. If the port's receive buffer is full the whole chunk is
 * dropped and counted; see comm_port_rx_dropped(). Safe to call from a
 * transport's task context.
 */
esp_err_t comm_port_rx(comm_port_id_t port_id, const uint8_t *data,
                       size_t length);

/**
 * @brief Be told when a port has received bytes.
 *
 * @p hook runs in the transport's task after the bytes are readable, or were
 * dropped for a full buffer, outside the lock, so the session can sleep until
 * there is input instead of polling.
 */
void comm_iface_set_rx_hook(void (*hook)(comm_port_id_t port_id));

/**
 * @brief Read what a port received, blocking up to @p timeout.
 *
 * @return Number of bytes read; 0 on timeout.
 */
size_t comm_port_read(comm_port_id_t port_id, uint8_t *buf, size_t length,
                      TickType_t timeout);

/**
 * @brief Write to a port.
 *
 * @return ESP_OK when the transport accepted the data.
 */
esp_err_t comm_port_write(comm_port_id_t port_id, const void *data,
                          size_t length);

/**
 * @brief Push whatever comm_port_write() buffered out to the wire.
 */
void comm_port_flush(comm_port_id_t port_id);

/**
 * @brief How much the client has sent that nobody has read yet.

 */
size_t comm_port_rx_pending(comm_port_id_t port_id);

/**
 * @brief Where in the port's byte stream the transport has got to.
 *
 * Bytes ever accepted by comm_port_rx(), modulo 2^32. A chunk dropped for want
 * of room is not counted. Taken alongside an out-of-band event - a serial
 * line setting, say - it says which received byte the event came after.
 */
uint32_t comm_port_rx_position(comm_port_id_t port_id);

/**
 * @brief Bytes ever read from the port, or discarded with its client.
 *
 * Equal to comm_port_rx_position() when nothing is pending. A reader holding
 * @p n bytes it just read owns positions [read - n, read).
 */
uint32_t comm_port_read_position(comm_port_id_t port_id);

/**
 * @brief The port's client has gone away.
 */
void comm_port_client_gone(comm_port_id_t port_id);

/**
 * @brief A front-end has come up on the port; it may talk again.
 *
 * The counterpart to comm_port_client_gone(). Called when a front-end starts,
 * because that is the point at which everything the previous one might still
 * have emitted has been and gone.
 */
void comm_port_client_ready(comm_port_id_t port_id);

/**
 * @brief Number of receive chunks dropped because the port's buffer was full.
 */
uint32_t comm_port_rx_dropped(comm_port_id_t port_id);

/**
 * @brief Print the ports and their state to the console.
 */
void comm_print_debug_info(void);
