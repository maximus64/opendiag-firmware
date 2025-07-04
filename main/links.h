/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

/**
 * @file links.h
 * @brief Names of the data links.
 *
 * A link's name is its vif session name, and that is how both a client and the
 * shell address it: "mode usb slcan". Keeping the spellings in one header is
 * what stops main.c, the shell and the control plane from drifting apart.
 *
 * The shell is a link only in the sense that it holds claims; it has no
 * transport of its own and cannot be switched to another grammar.
 */

#define LINK_NAME_USB   "usb"
#define LINK_NAME_BLE   "ble"
#define LINK_NAME_SHELL "shell"
