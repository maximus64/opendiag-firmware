/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

enum hs_pin {
    HS_OBD_PIN_6 = 6,
    HS_OBD_PIN_9 = 9,
    HS_OBD_PIN_11 = 11,
    HS_OBD_PIN_12 = 12,
    HS_OBD_PIN_13 = 13,
    HS_OBD_PIN_14 = 14,
};

enum ls_pin {
    LS_OBD_PIN_15 = 15,
};

void board_setup(void);


void board_hs_ls_reset_state(void);
void board_set_hs_boost_en(int state);
void board_set_hs_voltage(uint32_t mvolt);
void board_set_hs_state(enum hs_pin pin, int state);
void board_set_ls_state(enum ls_pin pin, int state);

uint8_t board_get_boardid(void);

/* return voltage value in millivolt */
int32_t board_get_vbatt(void);
int32_t board_get_hs_vsense(void);

/* factory calibration */
void board_calibrate_hs(void);
void board_calibrate_vbatt(void);
