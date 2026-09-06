/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file fake_board.c
 * @brief Test double for main/board.c.
 */

#include "fake_board.h"

#include <string.h>

#define MAX_OBD_PIN 16

static struct {
    int32_t vbatt_mv;

    uint32_t hs_voltage_mv;
    int hs_boost_en;
    int hs_state[MAX_OBD_PIN + 1];
    int ls_state[MAX_OBD_PIN + 1];
    int reset_count;
} g;

void fake_board_reset(void) {
    memset(&g, 0, sizeof(g));

    for (int i = 0; i <= MAX_OBD_PIN; i++) {
        g.hs_state[i] = -1;
        g.ls_state[i] = -1;
    }

    /* A plausible healthy vehicle, so tests that do not care read something
     * sensible rather than 0.0V. */
    g.vbatt_mv = 12600;
}

/* ------------------------------------------------------------------ *
 * Test-facing accessors
 * ------------------------------------------------------------------ */

void fake_board_set_vbatt_mv(int32_t mv) { g.vbatt_mv = mv; }

uint32_t fake_board_hs_voltage_mv(void) { return g.hs_voltage_mv; }
int fake_board_hs_boost_en(void) { return g.hs_boost_en; }
int fake_board_reset_count(void) { return g.reset_count; }

int fake_board_hs_state(int obd_pin) {
    if (obd_pin < 0 || obd_pin > MAX_OBD_PIN) {
        return -1;
    }
    return g.hs_state[obd_pin];
}

int fake_board_ls_state(int obd_pin) {
    if (obd_pin < 0 || obd_pin > MAX_OBD_PIN) {
        return -1;
    }
    return g.ls_state[obd_pin];
}

/* ------------------------------------------------------------------ *
 * board.h implementation
 * ------------------------------------------------------------------ */

void board_setup(void) { fake_board_reset(); }

/* No caller under test; present so the fake implements all of board.h. */
uint8_t board_get_button_state(void) { return 0; }

uint8_t board_get_boardid(void) { return 1; }

int32_t board_get_vbatt(void) { return g.vbatt_mv; }

int32_t board_get_hs_vsense(void) { return 0; }

void board_hs_ls_reset_state(void) {
    g.reset_count++;

    for (int i = 0; i <= MAX_OBD_PIN; i++) {
        g.hs_state[i] = -1;
        g.ls_state[i] = -1;
    }

    g.hs_boost_en = 0;
    g.hs_voltage_mv = 0;
}

void board_set_hs_boost_en(int state) { g.hs_boost_en = state; }

void board_set_hs_voltage(uint32_t mvolt) { g.hs_voltage_mv = mvolt; }

void board_set_hs_state(enum hs_pin pin, int state) {
    if ((int)pin >= 0 && (int)pin <= MAX_OBD_PIN) {
        g.hs_state[(int)pin] = state;
    }
}

void board_set_ls_state(enum ls_pin pin, int state) {
    if ((int)pin >= 0 && (int)pin <= MAX_OBD_PIN) {
        g.ls_state[(int)pin] = state;
    }
}

void board_calibrate_hs(void) {}

void board_calibrate_vbatt(void) {}

esp_err_t board_calibration_set(const char *name, int32_t value) {
    return ESP_OK;
}
