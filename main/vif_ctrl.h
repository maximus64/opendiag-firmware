/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

/**
 * @file vif_ctrl.h
 * @brief The control plane: switching a link's grammar, and asking what it is.
 *
 * Control never rides in a data stream. There is no escape character and no
 * front-end parses a control command, because the first binary front-end -
 * which is what the ECU flashing path wants - would have no character to
 * spare. Instead every link has an out-of-band way in:
 *
 *   USB   the debug shell on USB CDC 1, through its "mode" command
 *   BLE   the control characteristic, alongside the UART pair
 *
 * Both funnel into vif_ctrl_exec(). The command set is deliberately small and
 * carries nothing that drives hardware: the shell keeps the bring-up commands
 * that can put 20 V on a connector pin, and the radio never sees them.
 *
 * Commands, case insensitive:
 *
 *   ID            product, firmware, this link, its grammar, and every grammar
 *   MODE          the grammar running now
 *   MODE=<name>   switch to it, releasing every claim the session held
 *   BUS           which buses are live, how fast, and which link holds each
 *   RESET         release this link's claims and go back to its default
 */

#include <stdbool.h>
#include <stddef.h>

#include "vif.h"

/** Enough for the longest answer, which is ID with every grammar listed. */
#define VIF_CTRL_REPLY_MAX 128

/**
 * @brief Run one control command against @p s.
 *
 * @param s   the link the command acts on; NULL is answered, not crashed
 * @param cmd command text, not necessarily NUL terminated
 * @param len its length
 * @param out answer, always NUL terminated and never longer than @p cap - 1
 * @param cap size of @p out
 *
 * @return true when the command was understood. A false return still fills
 *         @p out with something worth showing the client.
 */
bool vif_ctrl_exec(vif_session_t *s, const char *cmd, size_t len, char *out,
                   size_t cap);
