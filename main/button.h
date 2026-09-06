/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

void button_init(void);

typedef void (*button_hold_cb_t)(void);
typedef void (*button_click_cb_t)(void);

void button_set_hold_callback(button_hold_cb_t cb);
void button_set_long_hold_callback(button_hold_cb_t cb);
void button_set_click_callback(button_click_cb_t cb);
