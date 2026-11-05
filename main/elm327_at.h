/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include "vif.h"

extern const vif_frontend_t elm327_frontend;

/** Copy every request and reply to the console log ("ELMTRC"). */
void elm327_trace_set(bool on);
bool elm327_trace_get(void);

void elm327_register(void);
