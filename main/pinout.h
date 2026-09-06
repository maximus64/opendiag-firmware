/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

/* CAN0 Transceiver */
#define PIN_CAN0_TX 4
#define PIN_CAN0_RX 5
#define PIN_CAN0_SILENT 8 /* Active High */

/* CAN1 Transceiver */
#define PIN_CAN1_TX 6
#define PIN_CAN1_RX 7
#define PIN_CAN1_SILENT 47 /* Active Low */

/* K-Line Transceiver */
#define PIN_KLINE_TX 17
#define PIN_KLINE_RX 18
#define PIN_KLINE_nSILENT 21

/* J1850 IO */
#define PIN_J1850_MODE 11
#define PIN_J1850_PWM_RX 12
#define PIN_J1850_VPW_RX 13
#define PIN_J1850_TX_N 14
#define PIN_J1850_TX_P 15

/* ADC Inputs */
#define PIN_HS_VOLTAGE_SENSE 1
#define PIN_OBD_VOLTAGE_SENSE 2
#define PIN_BOARDID 9

/* High Side Driver */
#define PIN_HS_OBD_6 38
#define PIN_HS_OBD_9 39
#define PIN_HS_OBD_11 40
#define PIN_HS_OBD_12 41
#define PIN_HS_OBD_13 42
#define PIN_HS_OBD_14 45
#define PIN_HS_BOOST_EN 46
#define PIN_HS_VOLTAGE_ADJUST 10 /* PWM signal */

/* Low Side Driver */
#define PIN_LS_OBD_15 3

/* Status LED + Button */
#define PIN_WS2812_LED 48
#define PIN_BUTTON 0
