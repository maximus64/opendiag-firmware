/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include "vif.h"

/** ELM327 AT command front-end: the default grammar on every data link. */
extern const vif_frontend_t elm327_frontend;

/** @brief Make the ELM327 grammar available to sessions by name. */
void elm327_register(void);
