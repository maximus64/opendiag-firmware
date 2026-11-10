/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

/**
 * @file kkl.h
 * @brief The link as a dumb KKL cable on OBD-II pin 7.
 *
 * Bytes from the host go onto the K-Line as they are, and everything the
 * K-Line carries - our own echo included - comes back as it arrives. The
 * host's serial port settings drive the line: the baud rate, data bits and
 * parity it sets reach the UART (or the bit banger, below 40 baud), and its
 * break holds the line low. That is all a VAG KW1281 tool, VCDS-Lite style,
 * expects of an FTDI KKL cable, and the 5 baud address word and the timing
 * stay the host's business.
 *
 * Over BLE or WiFi, which have no line settings, it is a raw bridge at
 * whatever rate the last USB host left, 10400 by default.
 */

#include <stdbool.h>
#include <stdint.h>

#include "comm_iface.h"
#include "vif.h"

extern const vif_frontend_t kkl_frontend;

/** @brief Make the KKL grammar available to the link by name. */
void kkl_register(void);

/**
 * @brief The host set @p port 's line coding. Call from the task that feeds
 *        @p port 's received bytes to comm_port_rx(), so the setting lands
 *        between the right two of them.
 *
 * @param parity USB CDC encoding: 0 none, 1 odd, 2 even, 3 mark, 4 space.
 */
void kkl_line_coding(comm_port_id_t port, uint32_t baud, uint8_t data_bits,
                     uint8_t parity);

/**
 * @brief The host sent a break on @p port. Same task rule as above.
 *
 * @param duration_ms 0 releases the line, 0xFFFF holds it until released,
 *                    anything else is a break of that many milliseconds.
 */
void kkl_break(comm_port_id_t port, uint16_t duration_ms);
