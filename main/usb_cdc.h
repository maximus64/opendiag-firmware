/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include "common.h"

/** The one USB data link: a protocol front-end lives here. */
#define USB_CDC_INTERFACE_DATA 0

/** The console. The IDF console and the debug shell are redirected onto it,
 *  so it is not a comm_iface port and takes no RX callback. */
#define USB_CDC_INTERFACE_SHELL 1

/**
 * @brief A host opened or closed one of the CDC interfaces.
 *
 * esp_tinyusb reports "connected" as DTR and RTS both asserted, so a tool that
 * raises DTR alone never reports either edge. That is the safe direction to
 * fail in: nothing reverts underneath a client that is still there.
 */
typedef void (*usb_cdc_line_cb_t)(int itf, bool connected);

void usb_cdc_setup(void);
void usb_cdc_teardown(void);
void usb_cdc_rx_set_callback(int itf, interface_rx_cb_t callback);
void usb_cdc_set_line_callback(usb_cdc_line_cb_t callback);
int usb_cdc_tx_write(int itf, const void* buf, uint32_t length);
void usb_cdc_tx_flush(int itf);
bool usb_cdc_get_line_status(int itf);
