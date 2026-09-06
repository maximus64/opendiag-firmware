/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"

/**
 * @brief What a bus needs before its peripheral can start.
 *
 * Only CAN has anything here: the TWAI controller takes its bit rate at
 * install time and cannot be retimed while up. The byte buses ignore it and
 * are configured after open(), through set_param() and ioctl().
 */
typedef struct {
    uint32_t bitrate; /* Bits per second. */
} bus_cfg_t;

typedef struct {
    uint8_t *data;   /* Caller's buffer. */
    size_t cap;      /* In: how much of it there is. */
    uint16_t len;    /* Out on receive, in on send. */
    uint16_t status; /* Out: BUS_RX_* flags. */
    /*
     * Out: when the last bit of the message was on the wire, microseconds.
     */
    uint32_t timestamp_us;
    /*
     * The CAN arbitration id, carrying CAN_EFF_FLAG and CAN_RTR_FLAG, and
     * on a remote frame @c len is the length code with no payload behind it.
     *
     * Zero on the byte buses: K-Line and J1850 carry their addressing in the
     * message itself, so there is nothing to put here. It lives in the
     * message rather than in a CAN shaped call because that is the only
     * thing separating CAN from the others - without it a frame would have
     * to be flattened into bytes and parsed straight back out again.
     */
    uint32_t id;
} bus_msg_t;

/* Receive status flags */
#define BUS_RX_TX_MSG_TYPE (1u << 0)     /* Loopback of our own transmit. */
#define BUS_RX_BREAK (1u << 1)           /* A break, not a message. */
#define BUS_RX_START_OF_MSG (1u << 2)    /* First bytes of a long one. */
#define BUS_RX_BUFFER_OVERFLOW (1u << 3) /* Data was lost before this. */
#define BUS_RX_BAD_CHECKSUM (1u << 4)    /* Checksum or CRC did not match. */
/* An answer to a message this driver sent on its own account. */
#define BUS_RX_PERIODIC_REPLY (1u << 5)
/* A repeat of the message before it, inside the duplicate window. */
#define BUS_RX_DUPLICATE (1u << 6)

/* Transmit flags */
/*
 * The caller's bytes already end in their checksum or CRC; append nothing.
 */
#define BUS_TX_NO_CHECKSUM (1u << 0)
/* J2534 WAIT_P3_MIN_ONLY: ignore P2max between requests, wait P3min only. */
#define BUS_TX_WAIT_P3_MIN_ONLY (1u << 1)

#define BUS_ERR_TIMEOUT (-1)   /* Timeout error */
#define BUS_ERR_NO_SPACE (-2)  /* Message is longer than the buffer. */
#define BUS_ERR_NOT_READY (-3) /* The driver is not up. */
#define BUS_ERR_TOO_LONG (-4)  /* Past what this bus can carry. */
#define BUS_ERR_TX_FAILED (-5) /* The peripheral refused it. */
#define BUS_ERR_ECHO (-6)      /* What came back is not what went out. */
#define BUS_ERR_INIT (-7)      /* An initialisation sequence failed. */
#define BUS_ERR_BAD_ARG (-8)
#define BUS_ERR_UNSUPPORTED (-9) /* Not a parameter this bus has. */
#define BUS_ERR_BUS_BUSY (-10)   /* Never went idle long enough to start. */
#define BUS_ERR_ARBITRATION (-11)

/* ------------------------------------------------------------------ *
 * Parameters
 * ------------------------------------------------------------------ */

/*
 * Everything about a bus that an upper layer may set or read.
 *
 * The values below BUS_P_VENDOR_BASE are J2534-1 Table 85 verbatim, units
 * included: the W and T parameters count milliseconds, the P parameters count
 * half-milliseconds. Keeping J2534's units as well as its numbers is what
 * makes SET_CONFIG a pass-through; a driver that quietly worked in
 * milliseconds throughout would need a conversion table that only the reader
 * of both files could check.
 *
 * A driver returns BUS_ERR_UNSUPPORTED for anything that does not apply
 * to it, which is exactly J2534's ERR_NOT_SUPPORTED and lets one front-end
 * offer the whole set without knowing which bus it is talking to.
 */
typedef enum {
    /* J2534-1 Table 85 */
    BUS_P_DATA_RATE = 0x01,    /* Bits per second. */
    BUS_P_NODE_ADDRESS = 0x04, /* J1850 PWM, this node's own address. */
    BUS_P_NETWORK_LINE = 0x05, /* J1850 PWM: normal, BUS+ only, BUS-. */

    BUS_P_P1_MAX = 0x07, /* Inter-byte time inside an ECU response. */
    BUS_P_P2_MAX = 0x09, /* Longest wait for a response. */
    BUS_P_P3_MIN = 0x0A, /* Quiet owed after a response. */
    BUS_P_P4_MIN = 0x0C, /* Inter-byte time inside our own request. */

    BUS_P_W1_MAX = 0x0E, /* Address word to synchronisation pattern. */
    BUS_P_W2_MAX = 0x0F, /* Synchronisation pattern to key byte 1. */
    BUS_P_W3_MAX = 0x10, /* Between the key bytes. */
    BUS_P_W4_MIN = 0x11, /* Key byte 2 to our inversion of it. */
    BUS_P_W5_MIN = 0x12, /* Bus idle before a 5 baud address, ISO 14230. */
    BUS_P_TIDLE = 0x13,  /* Bus idle before a fast initialisation. */
    BUS_P_TINIL = 0x14,  /* The low pulse of a fast initialisation. */
    BUS_P_TWUP = 0x15,   /* The whole wake up pulse. */
    BUS_P_PARITY = 0x16, /* BUS_PARITY_*. */
    BUS_P_W0_MIN = 0x19, /* Bus idle before a 5 baud address, ISO 9141. */

    BUS_P_DATA_BITS = 0x20,     /* BUS_DATA_BITS_*. */
    BUS_P_FIVE_BAUD_MOD = 0x21, /* BUS_FIVE_BAUD_*. */
    BUS_P_W4_MAX = 0x29,        /* Our key byte to the ECU's address. */
    BUS_P_LOOPBACK = 0x31,      /* J2534 ECHO_PHYSICAL_CHANNEL_TX. */

    /*
     * Where this firmware's own parameters start.
     *
     * Past everything J2534-1 numbers and past its reserved range, in the
     * space J2534-2 leaves to the interface vendor.
     */
    BUS_P_VENDOR_BASE = 0x10000,

    /*
     * Verify the checksum or CRC on receive. Default on.
     */
    BUS_P_CHECKSUM_RX = BUS_P_VENDOR_BASE,
    /*
     * Append the checksum or CRC on transmit. Default on.
     */
    BUS_P_CHECKSUM_TX,

    /*
     * Retransmissions after a transmission that did not survive the wire.
     */
    BUS_P_TX_RETRIES,

    /*
     * Collapse an identical message repeated inside this many milliseconds.
     */
    BUS_P_DUPLICATE_MS,

    /*
     * Send @c BUS_IOCTL_SET_PERIODIC 's message every this many ms of
     * silence. Zero switches it off.
     */
    BUS_P_PERIODIC_MS,

    /*
     * Hide the answers to this driver's own periodic messages. Default on.
     *
     * The ELM327 is explicit that a client never sees them: a reply to a
     * message the client did not send.
     */
    BUS_P_PERIODIC_QUIET,

    /*
     * Let the key bytes choose the timing set. Default off.
     */
    BUS_P_TIMING_FROM_KEYBYTES,

    /* Answer J1850 messages whose K bit asks for an in-frame response. */
    BUS_P_IFR_ENABLED,

    /* The byte this node sends as its in-frame response. */
    BUS_P_IFR_BYTE,

    /*
     * Watch the bus during our own transmission and stop on losing it.
     */
    BUS_P_ARBITRATION,
} bus_param_t;

/* J2534 Table 86 values for BUS_P_PARITY. */
#define BUS_PARITY_NONE 0
#define BUS_PARITY_ODD 1
#define BUS_PARITY_EVEN 2

/* J2534 Table 86 values for BUS_P_DATA_BITS. */
#define BUS_DATA_BITS_8 0
#define BUS_DATA_BITS_7 1

/* J2534 Table 86 values for BUS_P_NETWORK_LINE. */
#define BUS_NORMAL 0
#define BUS_PLUS 1
#define BUS_MINUS 2

/*
 * J2534 Table 86 values for BUS_P_FIVE_BAUD_MOD: how much of the
 * handshake after the key bytes actually happens.
 *
 * The standard's own note is worth repeating - these are described against
 * the original ISO 9141 sequence, which stops at the key bytes, not against
 * ISO 9141-2 or ISO 14230, which do not.
 */
#define BUS_FIVE_BAUD_STD_INIT 0 /* Both halves: our KB2, then its address. */
#define BUS_FIVE_BAUD_INV_KB2 1  /* We send the inverted key byte, and stop. */
#define BUS_FIVE_BAUD_INV_ADDR                                                 \
    2                            /* We send nothing; we wait for its address.  \
                                  */
#define BUS_FIVE_BAUD_9141_STD 3 /* Key bytes and nothing after them. */

typedef enum {
    /**
     * The 5 baud address word of ISO 14230-2
     *
     * in:  bus_init_t, whose @c address is the byte to send.
     * out: bus_init_t, filled with the key bytes and what they meant.
     */
    BUS_IOCTL_FIVE_BAUD_INIT = 0x04,

    /**
     * The wake up pattern
     *
     * in:  bus_init_t carrying the StartCommunication request to send,
     *      or a zero length one to drive the pattern and send nothing.
     * out: bus_init_t, whose @c msg holds the response.
     */
    BUS_IOCTL_FAST_INIT = 0x05,

    BUS_IOCTL_CLEAR_TX_QUEUE = 0x07,
    BUS_IOCTL_CLEAR_RX_QUEUE = 0x08,

    /** Stop the periodic message. J2534 CLEAR_PERIODIC_MSGS. */
    BUS_IOCTL_CLEAR_PERIODIC = 0x09,

    /** Empty the functional address table. J1850. */
    BUS_IOCTL_CLEAR_FUNCT_TABLE = 0x0B,
    /** Add one address to it. in: const uint8_t *. J1850. */
    BUS_IOCTL_ADD_FUNCT_ADDR = 0x0C,
    /** Remove one. in: const uint8_t *. J1850. */
    BUS_IOCTL_DEL_FUNCT_ADDR = 0x0D,

    /* --- Past J2534-1, in the vendor space --- */

    /**
     * Set the message the driver sends on its own account when the bus has
     * been quiet for BUS_P_PERIODIC_MS.
     *
     * in: bus_msg_t, without a checksum. NULL restores the protocol's own
     * default, which is a mode 01 PID 00 request on ISO 9141-2 and a
     * TesterPresent on ISO 14230.
     */
    BUS_IOCTL_SET_PERIODIC = BUS_P_VENDOR_BASE,

    /**
     * End the session, telling the ECU where the protocol has a way to.
     *
     * out: optional bus_link_t.
     */
    BUS_IOCTL_STOP_COMM,

    /**
     * What the last initialisation established, and whether one is up.
     *
     * out: bus_link_t.
     */
    BUS_IOCTL_GET_LINK,

    /**
     * Declare the link up without putting anything on the wire.
     *
     * The ELM327's AT BI, for a bench ECU or a simulator that answers
     * requests but performs no handshake. in: optional const uint8_t * naming
     * a protocol variant.
     */
    BUS_IOCTL_ASSUME_LINK,
} bus_ioctl_t;

/** Longest message a periodic transmission may carry. The ELM327's AT WM
 *  limit, and comfortably more than a TesterPresent needs. */
#define BUS_PERIODIC_MAX 6

/** Longest request a fast initialisation may carry, before its checksum. */
#define BUS_INIT_MSG_MAX 8

/**
 * @brief In and out for the two initialisation IOCTLs.
 *
 * One structure for both directions because that is what the sequence is: an
 * address or a request goes down, key bytes or a response come back, and the
 * measured intervals come back with them. J2534 splits it across SBYTE_ARRAY
 * and PASSTHRU_MSG; keeping the measurements is this firmware's addition, and
 * the reason is in bus_link_t.
 */
typedef struct {
    /* In. */
    uint8_t address;               /**< The 5 baud address word. */
    uint8_t msg[BUS_INIT_MSG_MAX]; /**< Fast init request, no checksum. */
    uint8_t msg_len;               /**< Zero sends no message. */

    /* Out. */
    uint8_t key[2];                      /**< Key bytes, in arrival order. */
    uint8_t reply[BUS_INIT_MSG_MAX + 8]; /**< Fast init response. */
    uint8_t reply_len;
} bus_init_t;

/**
 * @brief What a session negotiated, and how the vehicle performed.
 *
 * The measured intervals are the reason this exists rather than a bare "are
 * we connected". Every window in ISO 14230-2 Table 6 is wide - W1 spans 240
 * milliseconds - and an ECU sitting at the edge of one is a vehicle that
 * initialises in the workshop and not on the driveway. A counter of failed
 * initialisations cannot say that; these numbers can.
 */
typedef struct {
    bool connected;
    uint8_t variant; /**< Protocol-specific; K-Line uses kline_variant_t. */
    uint8_t key[2];
    uint16_t key_code; /**< ISO 14230-2 Table 5 identifier, 2000..2031. */
    uint8_t address;   /**< The address the handshake was directed to. */
    uint32_t baud;

    uint32_t w1_us; /**< Address word to synchronisation pattern. */
    uint32_t w2_us; /**< Synchronisation pattern to key byte 1. */
    uint32_t w3_us; /**< Between the key bytes. */
    uint32_t w4_us; /**< Our inverted key byte to the ECU's address. */
} bus_link_t;

/*
 * What a bus has done, split by failure mode rather than totalled.
 */
typedef struct {
    uint32_t rx_msgs; /* Complete messages handed to a caller. */
    uint32_t rx_bytes;
    uint32_t rx_bad_checksum;
    uint32_t rx_short;    /* Too few bytes to be a message. */
    uint32_t rx_too_long; /* Past what the protocol allows. */
    uint32_t rx_dropped;  /* Good messages lost: the queue was full. */
    uint32_t rx_overrun;  /* Bytes lost before the driver read them. */
    uint32_t rx_break;
    uint32_t rx_frame_err; /* Bad stop bit or symbol: usually the wrong rate. */
    uint32_t rx_suppressed; /* Answers to our own periodic messages. */
    uint32_t rx_duplicate;  /* Retransmissions collapsed into one. */

    uint32_t tx_msgs;
    uint32_t tx_bytes;
    uint32_t tx_echo_ok;      /* Read back off the wire unchanged. */
    uint32_t tx_echo_missing; /* Never came back at all. */
    uint32_t tx_echo_bad;     /* Came back different: a collision. */
    uint32_t tx_retries;
    uint32_t tx_bus_busy; /* Refused: the bus never went idle. */

    uint32_t init_attempts;
    uint32_t init_ok;
    uint32_t init_no_sync;  /* No synchronisation pattern inside W1. */
    uint32_t init_no_keys;  /* Sync arrived, key bytes did not. */
    uint32_t init_bad_keys; /* Key bytes named no protocol. */
    uint32_t init_no_addr;  /* The ECU never echoed the inverted address. */
    uint32_t periodic_sent;

    /* The most recent value of each, microseconds. Zero when never measured. */
    uint32_t last_w1_us;
    uint32_t last_w2_us;
    uint32_t last_w3_us;
    uint32_t last_w4_us;
    uint32_t last_p2_us;     /* Request to the first byte of its answer. */
    uint32_t last_p1_max_us; /* Widest inter-byte gap inside one message. */
} bus_stats_t;

/**
 * @brief What a bus driver has to provide.
 *
 * VIF serializes lifecycle and I/O on the owning session task. Direct users
 * must also finish all I/O before close; drivers do not cancel blocked reads.
 *
 * Every bus this firmware carries - CAN included - is reached through this
 * one vtable, so vif.c arbitrates all four the same way and no caller learns
 * which peripheral is underneath. CAN's frames cross it as bytes: a 4 byte
 * id carrying the EFF and RTR flags, a length, then the payload. See
 * can_bus_ops.
 *
 * Every entry except @p open, @p close, @p send and @p recv may be NULL; the
 * helpers below answer for a driver that does not implement one, so a bus
 * with no initialisation sequence does not carry an empty ioctl().
 */
typedef struct bus_ops {
    const char *name;

    /** Bring the peripheral up. @p cfg may be NULL for a bus that needs
     *  none. Idempotent while up. Failed startup still requires close. */
    esp_err_t (*open)(const bus_cfg_t *cfg);

    /** Release resources. Failure retains the claim; close may be retried. */
    esp_err_t (*close)(void);

    /**
     * Put one message on the wire.
     *
     * Waits out whatever quiet time the protocol owes first, so a caller
     * never has to. @p flags carries the per-message BUS_TX_* bits.
     *
     * @return 0, or a BUS_ERR_* code.
     */
    int (*send)(const bus_msg_t *msg, uint32_t flags);

    /**
     * Take the next message off the receive queue.
     *
     * @return Bytes written to @c msg->data, or a BUS_ERR_* code.
     */
    int (*recv)(bus_msg_t *msg, TickType_t wait);

    int (*set_param)(bus_param_t p, uint32_t value);
    int (*get_param)(bus_param_t p, uint32_t *out);

    /** @p in and @p out are the types the IOCTL's comment names. */
    int (*ioctl)(bus_ioctl_t id, const void *in, void *out);

    void (*get_stats)(bus_stats_t *out);
    void (*reset_stats)(void);
} bus_ops_t;

/** @brief Fill @p msg in for a receive into @p buf. */
static inline void bus_msg_init(bus_msg_t *msg, uint8_t *buf, size_t cap) {
    msg->data = buf;
    msg->cap = cap;
    msg->len = 0;
    msg->status = 0;
    msg->timestamp_us = 0;
    msg->id = 0;
}

/**
 * @brief Fill @p msg in for a transmit of @p len bytes from @p data.
 *
 * @param id CAN arbitration id with its flags, or 0 on the byte buses.
 *
 * The cast is here and nowhere else: send() takes the message by const
 * pointer and no driver writes through @c data on the way out, but the one
 * structure has to serve a receive as well, where the buffer is written.
 */
static inline void bus_msg_tx(bus_msg_t *msg, const uint8_t *data, size_t len,
                              uint32_t id) {
    msg->data = (uint8_t *)data;
    msg->cap = len;
    msg->len = (uint16_t)len;
    msg->status = 0;
    msg->timestamp_us = 0;
    msg->id = id;
}

/** @brief Name for a parameter, for the shell and for logs. NULL if unknown. */
const char *bus_param_name(bus_param_t p);

/** @brief Look a parameter up by the name bus_param_name() prints.
 *  @return true when @p out was set. Case insensitive. */
bool bus_param_from_name(const char *name, bus_param_t *out);

/** @brief Walk every parameter this firmware knows. NULL past the end. */
const char *bus_param_at(size_t idx, bus_param_t *out);
