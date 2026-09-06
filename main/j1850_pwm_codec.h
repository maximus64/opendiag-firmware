/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file j1850_pwm_codec.h
 * @brief SAE J1850 PWM (41.6 kbps) symbol coding, framing and CRC.
 *
 * This is the protocol half of the J1850 PWM driver, kept free of ESP-IDF so
 * that every timing window, every framing rule and every error path can be
 * exercised on the host. j1850_pwm.c owns the RMT peripheral and calls in
 * here; nothing in this file touches hardware, allocates, blocks or logs.
 *
 * The only ESP-IDF type it uses is rmt_symbol_word_t, which is a plain
 * bitfield describing two timed pulses. Speaking the peripheral's own layout
 * avoids copying a hundred symbols in the receive ISR, and the host build
 * shadows the header with an identical definition.
 *
 * Clause references are to SAE J1850 rev. FEB1994; the timing windows are
 * Table 3, "PWM Pulse Width Times".
 *
 * Wire model
 * ----------
 * PWM is a two-wire differential bus that is *active* (BUS+ high, BUS- low)
 * or *passive*. Every symbol starts with a rising edge - passive to active -
 * and the width of the active phase carries the value:
 *
 *      "1"  active Tp1 (7 us)      of a 24 us bit cell
 *      "0"  active Tp2 (15 us)     of a 24 us bit cell
 *      SOF  active Tp7 (31 us),    the next rising edge Tp4 (48 us) later
 *      BRK  active Tp8 (39 us)
 *
 * Clause 6.6.1.8 is explicit that only the rising edge is a timing reference:
 * the fall is slow and ambiguous because it is set by network capacitance. So
 * the decoder here measures rising edge to rising edge for cell boundaries and
 * uses the active width only to pick the bit value - which is also what makes
 * EOD distinguishable from a bit cell, since both end in a passive period.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_attr.h"
#include "hal/rmt_types.h"

#include "j1850_common.h"

/* ------------------------------------------------------------------ *
 * Timing, SAE J1850 Table 3 (microseconds)
 *
 * Two sets: what a transmitter must produce, and what a receiver must
 * accept. They are deliberately separate - transmitting at the edge of the
 * receive window would leave nothing for the next node's tolerance.
 * ------------------------------------------------------------------ */

#define J1850_PWM_TP1_NOM 7   /**< Active phase of a "1" bit. */
#define J1850_PWM_TP2_NOM 15  /**< Active phase of a "0" bit. */
#define J1850_PWM_TP3_NOM 24  /**< Bit time, rising edge to rising edge. */
#define J1850_PWM_TP4_NOM 48  /**< SOF / EOD, rising edge to rising edge. */
#define J1850_PWM_TP5_NOM 72  /**< EOF. */
#define J1850_PWM_TP6_NOM 96  /**< IFS. */
#define J1850_PWM_TP7_NOM 31  /**< Active phase of SOF. */
#define J1850_PWM_TP8_NOM 39  /**< Active phase of BRK. */
#define J1850_PWM_TP9_NOM 120 /**< BRK to IFS. */

/*
 * Transmit windows. What the standard requires to appear *on the bus* - the
 * note under Table 3 is explicit that these tolerances already include
 * "physical layer delays (i.e., turn-on and turn-off delays)". They are
 * therefore a constraint on the waveform after the driver stage, not on what
 * the microcontroller asks for; see J1850_PWM_TX_TRIM_US.
 */
#define J1850_PWM_TP1_TX_MIN 6
#define J1850_PWM_TP1_TX_MAX 8
#define J1850_PWM_TP2_TX_MIN 14
#define J1850_PWM_TP2_TX_MAX 16
#define J1850_PWM_TP3_TX_MIN 23
#define J1850_PWM_TP3_TX_MAX 25
#define J1850_PWM_TP4_TX_MIN 47
#define J1850_PWM_TP4_TX_MAX 49
#define J1850_PWM_TP5_TX_MIN 70
#define J1850_PWM_TP6_TX_MIN 93
#define J1850_PWM_TP7_TX_MIN 29
#define J1850_PWM_TP7_TX_MAX 32
#define J1850_PWM_TP8_TX_MIN 37
#define J1850_PWM_TP8_TX_MAX 41
#define J1850_PWM_TP9_TX_MIN 116

/* Receive acceptance windows. */
#define J1850_PWM_TP1_RX_MIN 4
#define J1850_PWM_TP1_RX_MAX 10
#define J1850_PWM_TP2_RX_MIN 12
#define J1850_PWM_TP2_RX_MAX 18
#define J1850_PWM_TP3_RX_MIN 21
#define J1850_PWM_TP3_RX_MAX 27
#define J1850_PWM_TP4_RX_MIN 42
#define J1850_PWM_TP4_RX_MAX 54
#define J1850_PWM_TP5_RX_MIN 63
#define J1850_PWM_TP6_RX_MIN 84
#define J1850_PWM_TP7_RX_MIN 27
#define J1850_PWM_TP7_RX_MAX 34
#define J1850_PWM_TP8_RX_MIN 35
#define J1850_PWM_TP8_RX_MAX 43

/**
 * @brief Where an ambiguous active pulse is split between "1" and "0".
 *
 * Table 3 leaves Tp1(max)..Tp2(min) - 11 us - undefined on purpose: "a pulse
 * width detected between Tp1(max) and Tp2(min) can be decoded as either an
 * active phase 1 or active phase 0", because the CRC will catch a wrong
 * guess. Splitting at the midpoint of the two nominals is the guess that is
 * wrong least often.
 */
#define J1850_PWM_BIT_SPLIT ((J1850_PWM_TP1_NOM + J1850_PWM_TP2_NOM) / 2)

/**
 * @brief Microseconds to shorten every emitted active phase by.
 *
 * A J1850 PWM transmitter is judged on what reaches the bus, and the path
 * from pin to bus is not symmetric: the transistor turns on faster than it
 * turns off, so the rising edge lands on time and the falling edge lands
 * late. Clause 6.6.1.8 describes exactly this, and it is why the protocol
 * takes all of its timing from rising edges.
 *
 * On this board that stretch measures +0.4 us, with the cell length untouched
 * because both rising edges shift equally. Emitting the nominal widths
 * therefore puts 7.4, 15.4 and 31.4 us on the wire - inside every transmit
 * window of Table 3 with 0.6 us to spare above, and 2.6 us to spare against
 * what any receiver must accept. So the correction is zero: the widths are
 * already right, and a whole microsecond of trim would cost more margin at
 * the bottom of Tp1 and Tp2 than it buys at the top.
 *
 * The knob is here because 0.4 us is a property of this driver stage, not of
 * the protocol. A slower one - a couple of microseconds, say - would put Tp1
 * out of its window, and there the correction is both needed and expressible.
 *
 * Measuring it takes care. The RMT records in whole microseconds, and this
 * node's own transmissions are synchronous with the very clock doing the
 * recording, so a self-measurement quantises: emitting 7 read back as 8 and
 * emitting 6 read back as 6, which looks like a 1 us stretch and is not one.
 * The number above came from temporarily raising the receive channel's
 * resolution to 10 MHz, which resolves tenths and settled it. Do that rather
 * than trusting "j1850 dump tx" to a microsecond.
 */
#ifndef J1850_PWM_TX_TRIM_US
#define J1850_PWM_TX_TRIM_US 0
#endif

/** Active phases the encoder emits, so the bus sees the nominal widths. */
#define J1850_PWM_TX_TP1 (J1850_PWM_TP1_NOM - J1850_PWM_TX_TRIM_US)
#define J1850_PWM_TX_TP2 (J1850_PWM_TP2_NOM - J1850_PWM_TX_TRIM_US)
#define J1850_PWM_TX_TP7 (J1850_PWM_TP7_NOM - J1850_PWM_TX_TRIM_US)

/**
 * @brief Bus levels as this board's transceiver presents them.
 *
 * The transistor driver is non-inverting on both TX pins and the receiver
 * comparator idles low, so active is a logic 1 on PIN_J1850_PWM_RX and on
 * PIN_J1850_TX_P / PIN_J1850_TX_N. Named rather than spelled 1 and 0 so a
 * board that inverts either side has one place to change.
 */
#define J1850_PWM_ACTIVE 1
#define J1850_PWM_PASSIVE 0

/**
 * @brief Longest message, clause 7.2.1: 12 bytes excluding the delimiters.
 *
 * The same limit as VPW, so the number itself lives in j1850_common.h. The
 * companion limit in the same clause - 101 bit times from SOF to EOF - works
 * out to the same 12 bytes once the delimiters are added, which is the part
 * that is specific to this modulation and the reason the alias is kept.
 */
#define J1850_PWM_MAX_FRAME J1850_MAX_FRAME

/** Symbols one encoded frame can need: SOF plus eight bits per byte. */
#define J1850_PWM_MAX_SYMBOLS (1 + 8 * J1850_PWM_MAX_FRAME)

/* ------------------------------------------------------------------ *
 * Encoding
 * ------------------------------------------------------------------ */

/**
 * @brief Lay out a frame as RMT symbols.
 *
 * One symbol per bit - active phase then passive phase - optionally preceded
 * by a SOF symbol. Nominal transmit timings throughout, which sit in the
 * middle of every receiver's window.
 *
 * What this does *not* emit is EOD, EOF or IFS. Those are absences of signal,
 * not pulses: the transmitter simply stops and leaves the bus passive, and
 * the caller is responsible for staying off it long enough afterwards.
 *
 * @param data     Bytes to send, CRC included. Sent MSB first (clause 5.3.2).
 * @param len      Byte count, at most J1850_PWM_MAX_FRAME.
 * @param with_sof True for a frame, false for in-frame response bytes, which
 *                 follow the originator's EOD and carry no SOF of their own.
 * @param out      Destination symbol array.
 * @param cap      Symbols @p out can hold.
 * @return Symbols written, or 0 if @p len or @p cap makes that impossible.
 */
size_t j1850_pwm_encode(const uint8_t *data, size_t len, bool with_sof,
                        rmt_symbol_word_t *out, size_t cap);

/* ------------------------------------------------------------------ *
 * Decoding
 * ------------------------------------------------------------------ */

/**
 * @brief What a captured pulse train turned out to be.
 *
 * Everything but J1850_PWM_RX_OK and J1850_PWM_RX_IFR_ONLY is counted rather
 * than delivered; the split matters because a dongle on an unfamiliar vehicle
 * needs to tell "quiet bus" from "wrong protocol" from "marginal wiring", and
 * one lumped error count cannot.
 */
typedef enum {
    J1850_PWM_RX_OK = 0,     /**< SOF, whole bytes, CRC good. */
    J1850_PWM_RX_IFR_ONLY,   /**< Bits with no SOF: someone's in-frame response.
                              */
    J1850_PWM_RX_BREAK,      /**< A BRK symbol, clause 6.6.1.6. */
    J1850_PWM_RX_EMPTY,      /**< Idle, or too little to judge. */
    J1850_PWM_RX_NO_SOF,     /**< Opening pulse matches no symbol at all. */
    J1850_PWM_RX_BAD_SYMBOL, /**< An active pulse outside every window. */
    J1850_PWM_RX_BAD_TIMING, /**< Rising edges spaced outside every window. */
    J1850_PWM_RX_FRAMING,    /**< Bit count is not a whole number of bytes. */
    J1850_PWM_RX_TOO_LONG,   /**< Past the 12 byte message limit. */
    J1850_PWM_RX_SHORT,   /**< Under two bytes: no message beneath the CRC. */
    J1850_PWM_RX_BAD_CRC, /**< Well formed, wrong CRC. */
} j1850_pwm_rx_status_t;

/** @brief Short name for logs and the shell. Never NULL. */
const char *j1850_pwm_rx_status_str(j1850_pwm_rx_status_t st);

/**
 * @brief One decoded capture.
 *
 * @p data and @p ifr are filled as far as the decoder got, so a frame that
 * fails on its CRC or runs long still shows its bytes to whoever is
 * diagnosing the bus.
 */
typedef struct {
    j1850_pwm_rx_status_t status;
    uint8_t data[J1850_PWM_MAX_FRAME]; /**< Message bytes, CRC included. */
    uint8_t len;                       /**< Bytes in @p data. */
    uint8_t ifr[J1850_PWM_MAX_FRAME];  /**< Response bytes after EOD. */
    uint8_t ifr_len;
    bool eod;            /**< An EOD gap separated data from IFR. */
    uint16_t eod_gap_us; /**< Captured rising-edge gap before the IFR. */
    /**
     * Width of the last active pulse in the capture, microseconds.
     *
     * The receive path needs it to place an in-frame response. A capture ends
     * when the bus has been passive for the arming threshold, so the last
     * rising edge was that threshold plus this width ago - which is the only
     * way back to the Tp4 reference the response has to start at.
     */
    uint16_t last_active_us;
} j1850_pwm_rx_t;

/**
 * @brief Turn a captured pulse train back into bytes.
 *
 * Never fails destructively: any input, including noise or a truncated
 * capture, yields a status and whatever was recoverable. That is what lets
 * the receive ISR run it unconditionally on whatever the RMT hands over.
 *
 * @param sym Captured symbols, in RMT order. A zero duration ends the train,
 *            which is how the peripheral marks the trailing idle.
 * @param n   Symbols in @p sym.
 * @param out Result, always fully initialised.
 * @return @p out->status, for convenience.
 */
j1850_pwm_rx_status_t j1850_pwm_decode(const rmt_symbol_word_t *sym, size_t n,
                                       j1850_pwm_rx_t *out);

/**
 * @brief Read the bytes out of a capture and nothing else.
 *
 * Used only to recognize our TX echo before rearming RMT. Pulse widths are
 * split without timing validation, and decoding stops at EOD. Received
 * messages use the full decoder; acknowledgment uses the streaming decoder.
 *
 * @param sym Captured symbols, in RMT order.
 * @param n   Symbols in @p sym.
 * @param out Destination for the message bytes, CRC included.
 * @param cap Bytes @p out can hold.
 * @return Bytes written, or 0 if the capture does not open with a SOF or does
 *         not end on a byte boundary.
 */
size_t j1850_pwm_quick_decode(const rmt_symbol_word_t *sym, size_t n,
                              uint8_t *out, size_t cap);

typedef struct {
    uint8_t data[J1850_PWM_MAX_FRAME];
    uint8_t len;
    uint8_t bits;
    uint8_t byte;
    uint8_t crc;
    bool valid;
    bool first;
} j1850_pwm_stream_t;

/* Feed a completed active pulse and its rising-edge spacing. True means a
 * CRC-valid byte boundary; the caller must still verify EOD and addressing. */
bool j1850_pwm_stream_pulse(j1850_pwm_stream_t *s, uint32_t active_us,
                            uint32_t edge_us);

/* ------------------------------------------------------------------ *
 * In-frame response
 * ------------------------------------------------------------------ */

/** Addresses one node answers for. Two covers a tester: its own and the
 *  functional address replies to it are sent to. */
#define J1850_PWM_MAX_IFR_TARGETS 4

/**
 * @brief Which frames this node acknowledges, and with what.
 *
 * A tester on Ford SCP sends its requests to functional address 0x6A and the
 * modules answer to 0x6B with K = 0, meaning "acknowledge me". The answer is
 * the tester's own physical address, 0xF1 by convention: a type 1 IFR,
 * clause 5.3.7 b.
 *
 * Enabled by default. GPIO edge decoding checks CRC and EOD before sending
 * the tester address. Table 3 requires a 47..49 us transmitted gap and
 * accepts 42..54 us at the receiver; 63 us is EOF, not the IFR deadline.
 */
typedef struct {
    bool enabled;                               /**< Off answers nothing. */
    uint8_t node_address;                       /**< Byte sent as the IFR. */
    uint8_t targets[J1850_PWM_MAX_IFR_TARGETS]; /**< Target bytes we answer. */
    uint8_t target_count;
} j1850_pwm_ifr_cfg_t;

/** @brief The tester defaults: 0xF1 answering 0x6B and 0xF1, enabled. */
void j1850_pwm_ifr_cfg_default(j1850_pwm_ifr_cfg_t *cfg);

/**
 * @brief Whether @p frame calls for an in-frame response from this node.
 *
 * True only for a three byte header frame whose K bit asks for one and whose
 * target byte is one of ours. A single byte header carries no target, so
 * there is no way to tell it was meant for us and we stay quiet - answering
 * a frame that was not addressed to us corrupts somebody else's IFR.
 *
 * The caller is expected to have checked the CRC first: an acknowledgement
 * for a frame we did not correctly receive is worse than silence, because it
 * tells the sender not to retransmit.
 */
bool j1850_pwm_ifr_wanted(const uint8_t *frame, size_t len,
                          const j1850_pwm_ifr_cfg_t *cfg);
