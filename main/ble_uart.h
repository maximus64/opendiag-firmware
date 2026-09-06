/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "common.h"
#include "sdkconfig.h"

typedef void (*ble_uart_rx_cb_t)(const uint8_t *rx_buf, size_t rx_size);

void ble_uart_setup(void);
void ble_uart_send(const char *buffer, size_t size);
void ble_uart_rx_set_callback(ble_uart_rx_cb_t callback);
bool ble_uart_is_connected(void);

/**
 * @brief How much the link has struggled to deliver.
 *
 * @param dropped Notifications abandoned outright. Any non-zero value here
 *                means a client saw a truncated response, which on an ELM327
 *                link is unrecoverable: the missing prompt puts it one reply
 *                behind for the rest of the session.
 * @param retries Notifications that needed the controller to catch up first.
 *                Ordinary in small numbers on a long connection interval.
 */
void ble_uart_get_notify_stats(uint32_t *dropped, uint32_t *retries);

/**
 * @brief What the receive side has seen.
 *
 * @param bytes   Total accepted from GATT writes.
 * @param widest  The largest single write. A client that puts several
 *                commands in one write shows up here, and that is the case
 *                that used to lose all but the first of them.
 * @param refused Writes rejected as too long or unreadable.
 */
void ble_uart_get_rx_stats(uint32_t *bytes, uint32_t *widest,
                           uint32_t *refused);

void ble_uart_print_trace(void);

#if CONFIG_OPENDIAG_BLE_TRACE

/**
 * @brief Put a marker in the trace, so it shows decisions as well as bytes.
 *
 * The bytes alone cannot distinguish "the client sent that twice" from "we
 * ran it twice"; a marker at the point of dispatch can. That distinction is
 * the whole of some faults: a write of "ATZ<CR>ATE0<CR>" followed by bare
 * carriage returns looks innocent until the markers show eight dispatches
 * behind it.
 */
void ble_uart_trace_note(char tag, const char *text, size_t len);
void ble_uart_trace_reset(void);

#else

static inline void ble_uart_trace_note(char tag, const char *text, size_t len) {
    (void)tag;
    (void)text;
    (void)len;
}

static inline void ble_uart_trace_reset(void) {}

#endif /* CONFIG_OPENDIAG_BLE_TRACE */

/** @brief Answer a control command. Split to fit the negotiated ATT MTU. */
void ble_uart_ctrl_reply(const char *text);

/**
 * One control command, as written to the control characteristic.
 *
 * GATT frames each write as a whole message, so @p cmd is one complete
 * command and is not NUL terminated.
 */
typedef void (*ble_uart_ctrl_cb_t)(const char *cmd, size_t len);

/* A central connected or went away. */
typedef void (*ble_uart_conn_cb_t)(bool connected);

void ble_uart_ctrl_set_callback(ble_uart_ctrl_cb_t cb);
void ble_uart_set_conn_callback(ble_uart_conn_cb_t cb);

void ble_uart_open_pairing_window(void);
bool ble_uart_pairing_window_open(void);

/**
 * @brief Forget every paired phone. The next one to pair has to be invited.
 */
void ble_uart_forget_bonds(void);

/** @brief How many phones are bonded to this adapter. */
int ble_uart_bond_count(void);

typedef enum {
    /** Nothing may connect and nothing is bonded: do not advertise at all.
     *  A new unit is off the air until somebody presses the button. */
    BLE_ADV_SILENT,

    /** The window is open. Named, unfiltered: any phone may see it and bond. */
    BLE_ADV_PAIRABLE,

    /** Bonded phones only. The name moves to the scan response, which the
     *  accept list withholds from everybody else, so a stranger's scan turns
     *  up a nameless device it cannot connect to. */
    BLE_ADV_LOCKED,

    /** A bond exists that cannot be matched through the address a phone
     *  actually connects from - no identity key on file, or the controller
     *  refused the accept list.*/
    BLE_ADV_DEGRADED,
} ble_adv_state_t;

ble_adv_state_t ble_uart_adv_state(void);
const char *ble_adv_state_name(ble_adv_state_t state);