/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include "vif.h"

extern const vif_frontend_t elm327_frontend;

/** Copy every request and reply to the console log ("ELMTRC"). */
void elm327_trace_set(bool on);
bool elm327_trace_get(void);

void elm327_register(void);

/**
 * Listen before the first transmit on a CAN protocol, and refuse it when a
 * bus at another bit rate is running on the pins. On by default.
 */
void elm327_can_guard_set(bool on);
bool elm327_can_guard_get(void);
