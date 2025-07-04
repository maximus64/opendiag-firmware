/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

void button_init(void);

/** @brief Called once per press, when the button has been held long enough. */
typedef void (*button_hold_cb_t)(void);

/**
 * @brief Install the long press handler and start watching the button.
 */
void button_set_hold_callback(button_hold_cb_t cb);
