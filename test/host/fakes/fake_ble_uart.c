/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_ble_uart.c
 * @brief Host stand-in for the BLE transport's diagnostic side.
 *
 * comm_iface reports the BLE link's notify and receive counters, and the
 * ELM327 front-end drops a marker in the BLE trace at every dispatch. Neither
 * is worth a NimBLE stack on the host, but both are real symbols that the
 * firmware sources reference, so they are answered here.
 *
 * The trace notes are kept rather than thrown away: they are the record of
 * which command lines the parser actually dispatched, which is exactly what a
 * test about a client falling out of step wants to assert on.
 */

#include "fake_ble_uart.h"

#include <string.h>

static char g_notes[FAKE_BLE_MAX_NOTES][FAKE_BLE_NOTE_LEN];
static size_t g_note_count;

void fake_ble_uart_reset(void) {
    memset(g_notes, 0, sizeof(g_notes));
    g_note_count = 0;
}

size_t fake_ble_uart_note_count(void) { return g_note_count; }

const char *fake_ble_uart_note(size_t idx) {
    return idx < g_note_count ? g_notes[idx] : NULL;
}

/* ---- the firmware's view ---------------------------------------------- */

void ble_uart_trace_note(char tag, const char *text, size_t len) {
    (void)tag;

    if (g_note_count >= FAKE_BLE_MAX_NOTES || !text) {
        return;
    }

    if (len >= FAKE_BLE_NOTE_LEN) {
        len = FAKE_BLE_NOTE_LEN - 1;
    }

    memcpy(g_notes[g_note_count], text, len);
    g_notes[g_note_count][len] = '\0';
    g_note_count++;
}

void ble_uart_get_notify_stats(uint32_t *dropped, uint32_t *retries) {
    if (dropped) {
        *dropped = 0;
    }
    if (retries) {
        *retries = 0;
    }
}

void ble_uart_get_rx_stats(uint32_t *bytes, uint32_t *widest,
                           uint32_t *refused) {
    if (bytes) {
        *bytes = 0;
    }
    if (widest) {
        *widest = 0;
    }
    if (refused) {
        *refused = 0;
    }
}
