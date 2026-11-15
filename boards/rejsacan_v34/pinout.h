/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include "board_features.h"

/* RejsaCAN v3.4 (ESP32-S3). Pin assignments from the RejsaCAN project's
 * REV 3.4 schematic (MagnusThome/RejsaCAN-ESP32, Schematics/RejsaCAN v3.x).
 *
 * Only the signals this board has and the firmware can use are defined. The
 * board has no CAN1, K-Line, J1850, OBD high side or low side drivers, boost,
 * WS2812 LED, or OBD voltage and board ID sense. Those macros stay undefined,
 * so any code that needs them fails to compile on this board until the feature
 * gating removes it. Nothing is left pointing at a pin with a different role. */

/* CAN0 transceiver (SN65HVD230). */
#define PIN_CAN0_TX 14
#define PIN_CAN0_RX 13
/* CAN_RS. High turns the driver off and leaves the receiver on, so the
 * firmware's listen-only state holds. */
#define PIN_CAN0_SILENT 38 /* Active High */

/* Boot strap pin, used as the user button as on OpenDIAG hardware v1. */
#define PIN_BUTTON 0
