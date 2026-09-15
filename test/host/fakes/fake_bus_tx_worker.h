/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include <stdbool.h>
/* Hold physical transmission admission while the real core keeps polling. */
void fake_bus_tx_hold(bool hold);

void fake_bus_tx_hold_completion(bool hold);
