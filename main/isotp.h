/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "can_frame.h"

#define ISOTP_MAX_PAYLOAD 4095u
#define ISOTP_DEFAULT_TIMEOUT_US 1000000u

typedef enum {
    ISOTP_IGNORED = 0,
    ISOTP_FRAME = 1,
    ISOTP_COMPLETE = 2,
    ISOTP_ERR_ARGUMENT = -1,
    ISOTP_ERR_BUSY = -2,
    ISOTP_ERR_SEQUENCE = -3,
    ISOTP_ERR_OVERFLOW = -4,
    ISOTP_ERR_TIMEOUT_A = -5,
    ISOTP_ERR_TIMEOUT_BS = -6,
    ISOTP_ERR_TIMEOUT_CR = -7,
    ISOTP_ERR_FLOW_STATUS = -8,
    ISOTP_ERR_WAIT_LIMIT = -9,
    ISOTP_ERR_SEND = -10,
} isotp_result_t;

/* IDs include CAN_EFF_FLAG. A mask may exclude normal-fixed priority bits.
 * Extended/mixed addressing adds one byte; CAN ID width is independent. */
typedef struct {
    uint32_t tx_id;
    uint32_t rx_id;
    uint32_t rx_mask;
    bool extended_address;
    uint8_t tx_address;
    uint8_t rx_address;
    bool functional;
    bool pad;
    uint8_t pad_value;
    uint8_t block_size;
    uint8_t stmin;
    uint8_t wait_limit;
    uint32_t timeout_a_us;
    uint32_t timeout_bs_us;
    uint32_t timeout_cr_us;
} isotp_config_t;

/* Defaults: exact ID match, padding on, BS/STmin=0, WAIT limit=255, 1 s timers.
 */
isotp_config_t isotp_config_default(uint32_t tx_id, uint32_t rx_id);
uint32_t isotp_stmin_us(uint8_t value);

typedef struct {
    const uint8_t *data; /* Borrowed from the input frame until it is reused. */
    uint16_t length;
    uint16_t offset;
    uint16_t total;
    bool started;
    bool complete;
    bool replaced;      /* Previous reception aborted with N_UNEXP_PDU. */
    bool padding_error; /* Cumulative DLC < 8; padding byte values are ignored.
                         */
} isotp_segment_t;

typedef enum {
    ISOTP_RX_IDLE,
    ISOTP_RX_FC_READY,
    ISOTP_RX_FC_CONFIRM,
    ISOTP_RX_CF,
} isotp_rx_state_t;

typedef struct {
    isotp_config_t config;
    uint8_t *buffer;
    size_t capacity;
    isotp_rx_state_t state;
    uint32_t deadline_us;
    uint16_t total;
    uint16_t offset;
    uint8_t sequence;
    uint8_t block_count;
    uint8_t flow_status;
    bool padding_error;
} isotp_rx_t;

/* NULL buffer selects streaming; capacity still bounds accepted message size.
 * All times are monotonic uint32_t microseconds; poll within 2^31 us. */
int isotp_rx_init(isotp_rx_t *rx, const isotp_config_t *config, uint8_t *buffer,
                  size_t capacity);
int isotp_rx_feed(isotp_rx_t *rx, const struct can_frame *frame,
                  uint32_t now_us, isotp_segment_t *segment);
/* Take a pending FC exactly once, then confirm its actual CAN completion. */
int isotp_rx_flow_control(isotp_rx_t *rx, struct can_frame *frame,
                          uint32_t now_us);
int isotp_rx_confirm(isotp_rx_t *rx, bool success, uint32_t now_us);
int isotp_rx_check_timeout(isotp_rx_t *rx, uint32_t now_us);

typedef enum {
    ISOTP_TX_IDLE,
    ISOTP_TX_READY,
    ISOTP_TX_CONFIRM,
    ISOTP_TX_FC,
    ISOTP_TX_CF,
} isotp_tx_state_t;

typedef struct {
    isotp_config_t config;
    const uint8_t *buffer;
    isotp_tx_state_t state;
    uint32_t deadline_us;
    uint32_t next_us;
    uint32_t last_cf_us;
    uint32_t separation_us;
    uint16_t total;
    uint16_t offset;
    uint16_t wait_count;
    uint8_t sequence;
    uint8_t block_size;
    uint8_t block_count;
    bool sent_cf;
    bool pending_cf;
    bool reserved_stmin;
    bool padding_error; /* Includes received FC frames for J2534 RxStatus. */
} isotp_tx_t;

int isotp_tx_init(isotp_tx_t *tx, const isotp_config_t *config);
/* Payload must remain valid until completion/error or reinitialization. */
int isotp_tx_start(isotp_tx_t *tx, const uint8_t *data, size_t length,
                   uint32_t now_us);
int isotp_tx_flow_control(isotp_tx_t *tx, const struct can_frame *frame,
                          uint32_t now_us);
/* Returns FRAME only when due. Confirm each returned frame before taking more.
 */
int isotp_tx_next(isotp_tx_t *tx, struct can_frame *frame, uint32_t now_us);
int isotp_tx_confirm(isotp_tx_t *tx, bool success, uint32_t now_us);
int isotp_tx_check_timeout(isotp_tx_t *tx, uint32_t now_us);
