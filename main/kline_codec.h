/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file kline_codec.h
 * @brief ISO 9141-2 and ISO 14230 (KWP 2000) message grammar, off the wire.
 *
 * Everything in here is arithmetic on bytes: checksums, how long a message is
 * going to be, what the key bytes said the ECU can do, which of the three
 * K-Line protocols we ended up speaking. None of it touches a peripheral, so
 * it is compiled into the host test suite and exercised there against the
 * tables in ISO 14230-2 rather than against a vehicle.
 *
 * kline.c is the other half: the UART, the timing, and the initialisation
 * sequences that this file only describes.
 *
 * References are to ISO/DIS 14230-2 (Keyword Protocol 2000, Part 2: Data Link
 * Layer) unless noted.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ------------------------------------------------------------------ *
 * Sizes
 * ------------------------------------------------------------------ */

/**
 * @brief Longest message the standard allows, checksum included.
 *
 * Clause 4.1.4: four header bytes, 255 data bytes, one checksum. An OBD-II
 * exchange never comes close - ISO 9141-2 caps at eleven - but a driver that
 * silently truncates at the length it expected is a driver that turns a
 * manufacturer specific upload into a checksum error.
 */
#define KLINE_MAX_MSG 260

/**
 * @brief Longest ISO 9141-2 / SAE J1979 message: three header, seven data,
 *        one checksum.
 *
 * This is a real limit rather than a buffer size, and the receiver uses it:
 * a CARB format byte carries no length, so eleven bytes with a good checksum
 * is a complete message and there is no point waiting out the inter-byte gap
 * to find out.
 */
#define KLINE_CARB_MAX_MSG 11

/** @brief Shortest legal message: format byte, service id, checksum. */
#define KLINE_MIN_MSG 3

/* ------------------------------------------------------------------ *
 * Format byte
 * ------------------------------------------------------------------ */

/** @brief Clause 4.1.1, bits A1 and A0 of the format byte. */
typedef enum {
    KLINE_ADDR_NONE = 0,     /**< No address information in the header. */
    KLINE_ADDR_CARB = 1,     /**< The ISO 9141-2 exception mode. */
    KLINE_ADDR_PHYSICAL = 2, /**< Target and source, physically addressed. */
    KLINE_ADDR_FUNCTIONAL =
        3, /**< Target and source, functionally addressed. */
} kline_addr_mode_t;

/** @brief The two format bytes CARB mode uses, clause 4.1.1. */
#define KLINE_FMT_CARB_REQ 0x68  /**< Tester to vehicle. */
#define KLINE_FMT_CARB_RESP 0x48 /**< Vehicle to tester. */

/**
 * @brief The address an ISO 9141-2 module answers to, whoever asked.
 *
 * SAE J1979 fixes the CARB response header at 48 6B <ECU>: the target byte
 * is this value on every reply, and it does not depend on how the request was
 * addressed. A tester that derives the expected reply address from its own
 * request header is right only by accident - it works while the request goes
 * to the functional address $6A, because $6A | 1 is $6B, and stops working
 * the moment a client addresses a module physically.
 */
#define KLINE_CARB_TESTER_ADDR 0x6B

/** @brief Address mode carried by a format byte. */
static inline kline_addr_mode_t kline_addr_mode(uint8_t fmt) {
    return (kline_addr_mode_t)((fmt >> 6) & 0x03u);
}

/**
 * @brief Header bytes a format byte implies, the format byte itself included.
 *
 * One on its own, or three once target and source are present. The additional
 * length byte is not counted here; kline_msg_len() folds it in.
 */
size_t kline_header_len(uint8_t fmt);

/**
 * @brief How long the whole message is, checksum included.
 *
 * @param buf Bytes received so far, starting at the format byte.
 * @param len How many of them there are.
 * @return The total on-wire length, or 0 when @p buf does not determine it -
 *         either because too few bytes have arrived, or because the message
 *         is in CARB mode, where the length is not encoded anywhere and the
 *         end of the message can only be found by the inter-byte gap.
 *
 * Clause 4.1.4: six bits of length in the format byte, and when those are
 * zero, a separate length byte after the addresses.
 */
size_t kline_msg_len(const uint8_t *buf, size_t len);

/** @brief True for the two format bytes of clause 4.1.1's CARB exception. */
static inline bool kline_is_carb_fmt(uint8_t fmt) {
    return kline_addr_mode(fmt) == KLINE_ADDR_CARB;
}

/* ------------------------------------------------------------------ *
 * Checksum
 * ------------------------------------------------------------------ */

/** @brief Clause 4.3: the 8 bit sum of every byte, checksum excluded. */
uint8_t kline_checksum(const uint8_t *data, size_t len);

/**
 * @brief Does @p buf hold one complete, correctly summed message?
 *
 * Length and checksum are the two things clause 6.4 requires the tester to
 * check, and the only two it can check without knowing the service.
 */
bool kline_frame_ok(const uint8_t *buf, size_t len);

/* ------------------------------------------------------------------ *
 * Key bytes
 * ------------------------------------------------------------------ */

/** @brief Which of the three K-Line protocols the key bytes chose. */
typedef enum {
    KLINE_VARIANT_UNKNOWN = 0,
    KLINE_VARIANT_ISO9141, /**< Key bytes 08 08 or 94 94. */
    KLINE_VARIANT_KWP,     /**< Key bytes 8F xx, ISO 14230-4. */
} kline_variant_t;

/** @brief Name for a variant, for logs and AT DP style reporting. */
const char *kline_variant_name(kline_variant_t v);

/**
 * @brief What an ECU said about itself in the key bytes, clause 5.1.5.1.
 *
 * The capability bits are the reason to decode these rather than just check
 * them: an ECU that does not support a header with addresses has to be
 * spoken to differently, and the only announcement of that is here.
 */
typedef struct {
    kline_variant_t variant;
    uint8_t kb1; /**< First byte off the wire. */
    uint8_t kb2; /**< Second byte off the wire. */

    /* Table 4. Meaningless unless variant is KLINE_VARIANT_KWP. */
    bool len_in_format;   /**< AL0: length may live in the format byte. */
    bool extra_len_byte;  /**< AL1: the separate length byte is supported. */
    bool hdr_1byte;       /**< HB0: a header without addresses is supported. */
    bool hdr_address;     /**< HB1: target/source in the header is supported. */
    bool extended_timing; /**< TP0,TP1 = 1,0: the Table 2 parameter set. */

    /** Table 5's decimal identifier, 2000 to 2031. Zero when not KWP. */
    uint16_t code;

    /** Both bytes carry odd parity, as ISO 9141:1989 requires. */
    bool parity_ok;
} kline_keybytes_t;

/**
 * @brief Read the two key bytes an ECU answered a 5 baud init with.
 *
 * Table 5 puts $8F in key byte 2 for every KWP set, and the two ISO 9141-2
 * pairs are symmetric, so which byte arrived first does not change the
 * answer. That is deliberate: the transmission order in figure 8 is KB1 then
 * KB2, but the tables and most of the field literature write the pair the
 * other way round, and a driver that picks the wrong convention fails to
 * recognise a perfectly good vehicle.
 *
 * @return true when the pair names a protocol. False leaves @p out zeroed
 *         apart from the raw bytes and the parity verdict, which is what a
 *         tester with key byte checking turned off wants to see anyway.
 */
bool kline_decode_keybytes(uint8_t b1, uint8_t b2, kline_keybytes_t *out);

/** @brief Odd parity over all eight bits, as the key bytes are defined. */
bool kline_odd_parity(uint8_t b);

/* ------------------------------------------------------------------ *
 * Timing
 * ------------------------------------------------------------------ */

/**
 * @brief The clause 4.4 parameters, in milliseconds.
 *
 * Table 1 is the default set and the only one an OBD-II tester needs; Table 2
 * is reachable when the key bytes said extended timing, and either can be
 * moved by the AccessTimingParameter service. The driver keeps a copy so both
 * are expressible without a second code path.
 *
 * @p p1_max doubles as the inter-byte gap that ends a received message. The
 * standard does not name a frame separator - it does not have to, because an
 * ECU that respects P1max and P2min has left a gap wider than any inter-byte
 * one by the time it stops talking.
 */
typedef struct {
    uint16_t p1_max; /**< Inter byte time within an ECU response. */
    uint16_t p2_min; /**< Earliest an ECU may answer. */
    uint16_t p2_max; /**< Latest it may, before the request is lost. */
    uint16_t p3_min; /**< Quiet time owed before the next request. */
    uint16_t p3_max; /**< Idle time after which the ECU drops the session. */
    uint16_t p4_min; /**< Inter byte time within a tester request. */
} kline_timing_t;

/** @brief Table 1, the normal set: functional and physical addressing. */
void kline_timing_normal(kline_timing_t *out);

/** @brief Table 2, the extended set: physical addressing only. */
void kline_timing_extended(kline_timing_t *out);

/**
 * @brief Table 3: decode a P2max byte from AccessTimingParameter.
 *
 * $01 to $F0 is a count of 25 ms. Above that the low nibble is multiplied by
 * 256 as well, which is how the service reaches past six seconds.
 *
 * @return Milliseconds, or 0 for the reserved value $00.
 */
uint32_t kline_p2max_from_byte(uint8_t v);

/* ------------------------------------------------------------------ *
 * Initialisation
 * ------------------------------------------------------------------ */

/**
 * @brief How the link is to be brought up.
 *
 * The parameters of an attempt and what it established live in bus.h, as
 * bus_init_t and bus_link_t: they are the same shape on every bus that has
 * an initialisation sequence, and J2534 reaches them through one IOCTL. This
 * enum stays here because naming the two sequences is protocol knowledge -
 * it is what tells ELM327 whether AT SI or AT FI applies.
 */
typedef enum {
    /**
     * No initialisation at all: bytes in, bytes out.
     *
     * Not a protocol - it is what a bus trace and the target test suite need,
     * and what lets a client drive an unusual sequence by hand.
     */
    KLINE_INIT_NONE = 0,
    /** The 5 baud address word of clause 5.1.5.2. ISO 9141-2 or KWP; which
     *  one it turned out to be is in the key bytes afterwards. */
    KLINE_INIT_5BAUD,
    /** The wake up pattern of clause 5.1.5.3. KWP only, 10400 baud only. */
    KLINE_INIT_FAST,
} kline_init_mode_t;

/** @brief The address every emission related ECU listens for, ISO 14230-4. */
#define KLINE_INIT_ADDR_OBD 0x33

/** @brief The baud rate fast init is defined at, and 5 baud init defaults to.
 */
#define KLINE_BAUD_DEFAULT 10400

/** @brief Clause 5.1.5.2.2: the range a 5 baud init may negotiate. */
#define KLINE_BAUD_MIN 1200
#define KLINE_BAUD_MAX 10400

/**
 * @brief The fastest rate the driver will accept.
 *
 * Above what the standard allows, because AT IB offers 12500 and 15625 and
 * the systems that want them are not OBD systems - they are the manufacturer
 * specific ones a scan tool also has to reach.
 */
#define KLINE_BAUD_HW_MAX 15625

/**
 * @brief Number of transitions in an 8N1 $55 synchronisation character.
 *
 * Idle-to-start, one at every alternating data bit, then data-to-stop: ten
 * edges separated by exactly one bit time. This is the pattern clause
 * 5.1.5.2.2 gives the tester so it can determine the ECU's baud rate.
 */
#define KLINE_SYNC_EDGE_COUNT 10

/**
 * @brief Derive the baud rate from timestamped edges of the $55 sync byte.
 *
 * @param edge_us At least @ref KLINE_SYNC_EDGE_COUNT monotonically sampled
 *        32-bit microsecond timestamps. Counter wrap is supported.
 * @param count Number of timestamps available.
 * @return Measured baud in the ISO 5-baud-init range (with clock tolerance),
 *         or zero when the intervals are not a credible $55 pattern.
 *
 * This is kept independent of the ESP UART/GPIO drivers so the estimator can
 * be tested on the host. The caller is responsible for confirming that the
 * first transition is high-to-low.
 */
uint32_t kline_sync_baud_from_edges(const uint32_t *edge_us, size_t count);

/**
 * @brief Longest wakeup message.
 *
 * Not an ISO limit: it is the ELM327's AT WM, "one to six bytes total, not
 * including the checksum", and this firmware answers to that interface.
 */
#define KLINE_WAKEUP_MAX 6

/**
 * @brief The request that opens a fast init, clause 5.1.5.3.
 *
 * Functional addressing, target $33, tester $F1, StartCommunication. The
 * whitepaper's C1 33 F1 81, and the only form ISO 14230-4 allows for
 * legislated OBD.
 */
#define KLINE_SVC_START_COMM 0x81
#define KLINE_SVC_START_COMM_RESP 0xC1
#define KLINE_SVC_STOP_COMM 0x82
#define KLINE_SVC_STOP_COMM_RESP 0xC2
#define KLINE_SVC_TESTER_PRESENT 0x3E
#define KLINE_SVC_NEG_RESP 0x7F

/** @brief Negative response code $78: the ECU is still working on it. */
#define KLINE_NRC_RESPONSE_PENDING 0x78

/**
 * @brief Is this a "response pending" negative response?
 *
 * Clause 4.4.1: it moves P2max out to P3max for as long as the ECU keeps
 * sending them, so a receiver that does not recognise it reports a timeout
 * on every slow service.
 */
bool kline_is_response_pending(const uint8_t *buf, size_t len);

/**
 * @brief The default wakeup message for a variant, checksum excluded.
 *
 * 68 6A F1 01 00 for ISO 9141-2 and C1 33 F1 3E for KWP - a mode 01 PID 00
 * request and a TesterPresent respectively, which is what a real ELM327
 * sends and what AT WM overrides.
 *
 * @return Bytes written to @p out, or 0 if @p cap is too small.
 */
size_t kline_default_wakeup(kline_variant_t v, uint8_t *out, size_t cap);
