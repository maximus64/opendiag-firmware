/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_board.h
 * @brief Test double for main/board.c.
 *
 * Records every output the ELM327 layer drives and lets a test dictate what
 * the analog inputs read back.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board.h"

void fake_board_reset(void);

/* --- Inputs the test controls --- */

/* Only the battery reading is settable: it is the one analog input the ELM327
 * layer reads, via AT RV. The rest of board.h is implemented below but has no
 * caller under test, so it has no knob. */
void fake_board_set_vbatt_mv(int32_t mv);

/* --- Outputs the test inspects --- */

/** Last voltage requested through board_set_hs_voltage(), in millivolts. */
uint32_t fake_board_hs_voltage_mv(void);

/** Latest boost enable state. */
int fake_board_hs_boost_en(void);

/** Drive state of an OBD-II connector pin, or -1 if it was never touched. */
int fake_board_hs_state(int obd_pin);
int fake_board_ls_state(int obd_pin);

/** Times board_hs_ls_reset_state() was called. */
int fake_board_reset_count(void);
