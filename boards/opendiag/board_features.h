/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

/* OpenDIAG hardware v1 has every feature below. A board that lacks one sets it
 * to 0 in its own board_features.h, and the firmware leaves that code out. */

/* CAN1 transceiver on the AUX_CAN header (CAN bus with its own pins). */
#define OPENDIAG_HAS_CAN1 1
/* K-Line transceiver. */
#define OPENDIAG_HAS_KLINE 1
/* J1850 PWM and VPW transceiver (shared MODE, TX and RX pins). */
#define OPENDIAG_HAS_J1850 1
/* High side drivers on OBD pins 6, 9, 11, 12, 13 and 14, with boost and the
 * programming voltage adjust PWM. */
#define OPENDIAG_HAS_OBD_HS 1
/* Low side drivers on OBD pin 15 (and pin 9 with the rework). */
#define OPENDIAG_HAS_OBD_LS 1
/* ADC inputs for programming voltage sense, battery voltage and board ID. */
#define OPENDIAG_HAS_OBD_SENSE 1
/* WS2812 status LED. */
#define OPENDIAG_HAS_WS2812_LED 1
