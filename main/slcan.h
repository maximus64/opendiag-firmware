/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include "vif.h"

/** LAWICEL/SLCAN front-end, for slcand and anything else speaking it. */
extern const vif_frontend_t slcan_frontend;

/** @brief Make the SLCAN grammar available to sessions by name. */
void slcan_register(void);
