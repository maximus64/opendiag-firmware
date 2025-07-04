/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once
#include <stdint.h>
#include <stddef.h>

typedef void (*interface_rx_cb_t)(const uint8_t *rx_buf, size_t rx_size);

/** Product identity, reported by AT @1 and by the control plane's ID. */
#define OPENDIAG_PRODUCT  "Dalalogic OpenDiag"
#define OPENDIAG_HARDWARE "rev1"
#define OPENDIAG_VERSION  "1.0"
