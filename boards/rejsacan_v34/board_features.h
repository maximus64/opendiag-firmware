/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

/* RejsaCAN v3.4 has CAN0 (always present, see pinout.h) and none of the
 * features below. The firmware leaves that code out, and a request for it
 * fails with an explicit error. */

/* CAN1 transceiver on the AUX_CAN header. Not fitted. */
#define OPENDIAG_HAS_CAN1 0
/* K-Line transceiver. Not fitted. */
#define OPENDIAG_HAS_KLINE 0
/* J1850 PWM and VPW transceiver. Not fitted. */
#define OPENDIAG_HAS_J1850 0
/* High side OBD drivers, boost and programming voltage. Not fitted. */
#define OPENDIAG_HAS_OBD_HS 0
/* Low side OBD drivers. Not fitted. */
#define OPENDIAG_HAS_OBD_LS 0
/* ADC inputs for programming voltage sense, battery voltage and board ID. The
 * v3.4 battery sense is on GPIO 9, which is not wired to the firmware yet. */
#define OPENDIAG_HAS_OBD_SENSE 0
/* WS2812 status LED. The v3.4 has discrete LEDs and no WS2812. */
#define OPENDIAG_HAS_WS2812_LED 0
