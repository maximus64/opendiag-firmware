/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include <stddef.h>
#include <stdint.h>

/* Product identity, reported by AT @1 and by the control plane's ID. */
#define OPENDIAG_MANUFACTURE "Dalalogic"
#define OPENDIAG_PRODUCT "OpenDiag"
#define OPENDIAG_HARDWARE "rev1"

/* Session names. One data link, whichever transport is carrying it. */
#define LINK_NAME_DATA "link"
#define LINK_NAME_SHELL "shell"
