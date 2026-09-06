/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file j1850_vpw_codec.h
 * @brief SAE J1850 VPW (10.4 kbps) symbol coding and framing.
 *
 * This is the protocol half of the J1850 VPW driver, kept free of ESP-IDF so
 * that every timing window, every framing rule and every error path can be
 * exercised on the host. j1850_vpw.c owns the RMT peripheral and calls in
 * here; nothing in this file touches hardware, allocates, blocks or logs.
 *
 * The only ESP-IDF type it uses is rmt_symbol_word_t, which is a plain
 * bitfield describing two timed pulses. Speaking the peripheral's own layout
 * avoids copying a hundred symbols in the receive ISR, and the host build
 * shadows the header with an identical definition.
 *
 * Clause references are to SAE J1850 rev. FEB1994; the timing windows are
 * Table 5, "VPW Pulse Width Times".
 *
 * Wire model
 * ----------
 * VPW is a single wire, wired-OR bus that is *active* (driven high) or
 * *passive* (released). Where PWM gives every bit a fixed cell and encodes
 * the value in the width of its active phase, VPW gives every bit exactly one
 * pulse and encodes the value in that pulse's width *and* its level - which
 * is what "variable pulse width" means and why the levels strictly alternate:
 *
 *      "1"   active short (Tv1, 64 us)   or  passive long  (Tv2, 128 us)
 *      "0"   active long  (Tv2, 128 us)  or  passive short (Tv1, 64 us)
 *      SOF   active Tv3 (200 us)
 *      EOD   passive Tv3 (200 us)
 *      NB    active Tv1 or Tv2, the first symbol of an in-frame response
 *      EOF   passive Tv4 (280 us)
 *      BRK   active Tv5 (>= 280 us)
 *
 * Clause 6.6.2 (Figure 14) is the authority on the two bit encodings, and the
 * arbitration diagram of Figure 21 is a second statement of the same thing.
 * The consequence worth keeping in mind while reading the decoder is that a
 * pulse width on its own says nothing: the level it was driven at is half the
 * symbol, and a decoder that loses track of the alternation inverts every bit
 * it goes on to produce rather than failing visibly.
 *
 * Because a bit is one pulse and not a cell, there is no rising-edge-to-
 * rising-edge interval to validate the way j1850_pwm_codec.c does. What
 * replaces it is the alternation rule: two pulses at the same level cannot
 * both be symbols, and the decoder treats that as a bad symbol rather than
 * guessing which of them to believe.
 *
 * Frame shape
 * -----------
 *      SOF  data... CRC  [ EOD  NB  IFR... ]  EOF  IFS
 *
 * Since SOF is active and the levels alternate from there, the first data bit
 * is always passive, every byte is a whole number of pulses, and a frame ends
 * on an active pulse - which is exactly why EOD is defined as a passive
 * symbol and can never be confused with a bit.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_attr.h"
#include "driver/rmt_types.h"

#include "j1850_common.h"

/* ------------------------------------------------------------------ *
 * Timing, SAE J1850 Table 5 (microseconds)
 *
 * Two sets: what a transmitter must produce, and what a receiver must
 * accept. They are deliberately separate - transmitting at the edge of the
 * receive window would leave nothing for the next node's tolerance.
 * ------------------------------------------------------------------ */

#define J1850_VPW_TV1_NOM 64  /**< Short pulse. */
#define J1850_VPW_TV2_NOM 128 /**< Long pulse. */
#define J1850_VPW_TV3_NOM 200 /**< SOF when active, EOD when passive. */
#define J1850_VPW_TV4_NOM 280 /**< EOF. */
#define J1850_VPW_TV5_NOM 300 /**< BRK. */
#define J1850_VPW_TV6_NOM 300 /**< IFS. */

/* Transmit windows, Table 5 columns Tx,min and Tx,max. */
#define J1850_VPW_TV1_TX_MIN 49
#define J1850_VPW_TV1_TX_MAX 79
#define J1850_VPW_TV2_TX_MIN 112
#define J1850_VPW_TV2_TX_MAX 145
#define J1850_VPW_TV3_TX_MIN 182
#define J1850_VPW_TV3_TX_MAX 218
#define J1850_VPW_TV4_TX_MIN 261
#define J1850_VPW_TV5_TX_MIN 280
#define J1850_VPW_TV6_TX_MIN 280

/*
 * Receive acceptance windows, Table 5 columns Rx,min and Rx,max.
 *
 * Appendix C is explicit that "VPW bus symbols allow no forbidden zones
 * between symbols": each window begins one microsecond above the one below
 * it, so every width from Tv1(rx,min) upwards decodes as something. The
 * bounds are written here exactly as the table states them - strictly greater
 * than the minimum, less than or equal to the maximum - because at 64 and
 * 128 us nominal the boundaries are 30 us from either neighbour and there is
 * no reason to paraphrase them into off-by-one risk.
 */
#define J1850_VPW_TV1_RX_MIN 35 /**< "> 34" */
#define J1850_VPW_TV1_RX_MAX 96
#define J1850_VPW_TV2_RX_MIN 97 /**< "> 96" */
#define J1850_VPW_TV2_RX_MAX 163
#define J1850_VPW_TV3_RX_MIN 164 /**< "> 163" */
#define J1850_VPW_TV3_RX_MAX 239
#define J1850_VPW_TV4_RX_MIN 240 /**< "> 239", and the same floor as BRK. */

/**
 * @brief Bus levels as this board's transceiver presents them.
 *
 * The transistor driver is non-inverting and the receiver comparator idles
 * low, so an active bus is a logic 1 on PIN_J1850_VPW_RX and on
 * PIN_J1850_TX_P. Named rather than spelled 1 and 0 so a board that inverts
 * either side has one place to change.
 */
#define J1850_VPW_ACTIVE 1
#define J1850_VPW_PASSIVE 0

/**
 * @brief Microseconds to shorten every active pulse by, and lengthen every
 *        passive one by.
 *
 * The path from pin to bus is not symmetric: the transistor turns on faster
 * than it turns off, so a rising edge lands on time and the falling edge
 * lands late. Every active pulse comes out that much longer and every passive
 * pulse that much shorter, with the cell length untouched because the two
 * errors cancel across an edge.
 *
 * On this board, measured through this node's own receiver with "vpw dump
 * tx", the skew is 4 us: emitting the nominal widths puts 204, 68 and 132 us
 * of active on the wire and 60 and 124 us of passive. The correction is
 * nonetheless zero, and the numbers are why. Table 5 allows a short pulse
 * anywhere from 49 to 79 us, so the worst of those sits 11 us inside its
 * transmit window and 28 us inside what any receiver must accept - and unlike
 * PWM, VPW resynchronises on every edge, so nothing accumulates across a
 * frame.
 *
 * There is also a reason not to trust the measurement far enough to act on
 * it. It was taken through this board's own receiver, which has turn-on and
 * turn-off delays of its own; some unknown part of that 4 us is the receiver
 * rather than the driver, and trimming would move the real waveform off
 * centre while making the measurement look perfect. Correcting a skew that
 * costs nothing, on the strength of an instrument that cannot separate the
 * two halves of it, is how a working transmitter becomes a marginal one.
 *
 * The knob is here because the skew is a property of a driver stage, not of
 * the protocol. A slower one - fifteen microseconds, say - would push Tv1 out
 * of its window, and there the correction is both needed and expressible.
 * Settling it properly takes a scope on the bus, not this node's receiver.
 */
#ifndef J1850_VPW_TX_TRIM_US
#define J1850_VPW_TX_TRIM_US 0
#endif

/** Widths the encoder emits, so the bus sees the nominal ones. */
#define J1850_VPW_TX_TV1_ACTIVE (J1850_VPW_TV1_NOM - J1850_VPW_TX_TRIM_US)
#define J1850_VPW_TX_TV2_ACTIVE (J1850_VPW_TV2_NOM - J1850_VPW_TX_TRIM_US)
#define J1850_VPW_TX_TV3_ACTIVE (J1850_VPW_TV3_NOM - J1850_VPW_TX_TRIM_US)
#define J1850_VPW_TX_TV1_PASSIVE (J1850_VPW_TV1_NOM + J1850_VPW_TX_TRIM_US)
#define J1850_VPW_TX_TV2_PASSIVE (J1850_VPW_TV2_NOM + J1850_VPW_TX_TRIM_US)
#define J1850_VPW_TX_TV3_PASSIVE (J1850_VPW_TV3_NOM + J1850_VPW_TX_TRIM_US)

/** @brief Longest message, clause 7.2.2. The same 12 bytes PWM carries. */
#define J1850_VPW_MAX_FRAME J1850_MAX_FRAME

/**
 * @brief Symbols one encoded frame needs.
 *
 * A frame is 1 + 8*len pulses - an odd number, because SOF is active and the
 * frame therefore ends on one - and an RMT symbol word holds two pulses. The
 * odd pulse out is paired with the trailing EOD, so the count is exact rather
 * than rounded up.
 */
#define J1850_VPW_MAX_SYMBOLS (4 * J1850_VPW_MAX_FRAME + 1)

/* ------------------------------------------------------------------ *
 * Encoding
 * ------------------------------------------------------------------ */

/**
 * @brief Lay out a frame as RMT symbols: SOF, the bytes, and a trailing EOD.
 *
 * Nominal transmit timings throughout, plus J1850_VPW_TX_TRIM_US.
 *
 * The EOD at the end is emitted rather than merely waited out. It costs
 * nothing - the bus is passive either way - and it means the peripheral, not
 * the calling task's scheduling, is what holds the line for the 200 us in
 * which a responder is entitled to start an in-frame response.
 *
 * What this does *not* emit is EOF or IFS. Those are absences of signal, not
 * pulses: the transmitter stops and leaves the bus passive, and the caller is
 * responsible for staying off it long enough afterwards.
 *
 * @param data Bytes to send, CRC included. Sent MSB first (clause 5.3.2).
 * @param len  Byte count, at most J1850_VPW_MAX_FRAME.
 * @param out  Destination symbol array.
 * @param cap  Symbols @p out can hold.
 * @return Symbols written, or 0 if @p len or @p cap makes that impossible.
 */
size_t j1850_vpw_encode(const uint8_t *data, size_t len, rmt_symbol_word_t *out,
                        size_t cap);

/* ------------------------------------------------------------------ *
 * Contention
 *
 * Clause 6.7.2 requires a transmitter to compare the bus against what it is
 * driving and drop out on the first difference. What follows is the model
 * j1850_vpw.c watches a transmission with; it is here rather than there
 * because it is protocol reasoning and belongs somewhere it can be tested.
 * ------------------------------------------------------------------ */

/** @brief One pulse of an encoded frame: a level and a width. */
typedef struct {
    uint8_t level; /**< J1850_VPW_ACTIVE or J1850_VPW_PASSIVE. */
    uint16_t us;
} j1850_vpw_pulse_t;

/** @brief Pulses in a frame of @p len bytes: SOF, the bits, the trailing EOD.
 */
#define J1850_VPW_PULSE_COUNT(len) (2u + 8u * (size_t)(len))

/**
 * @brief The @p k th pulse of the frame @p data, worked out rather than stored.
 *
 * Deliberately derived from the bytes on every call instead of read out of the
 * encoded symbol array. The transmit path will eventually stream symbols
 * straight from the byte buffer rather than materialise them - a 4128 byte
 * message is 16,500 symbols and has no business existing in RAM - and a
 * contention monitor written against that array would have to be rewritten
 * when it goes. This form does not care.
 *
 * @return false when @p k is past the end of the frame, or the arguments make
 *         no sense.
 */
bool j1850_vpw_pulse_at(const uint8_t *data, size_t len, size_t k,
                        j1850_vpw_pulse_t *out);

/** @brief Most offsets j1850_vpw_watch_offsets() can produce for one pulse. */
#define J1850_VPW_MAX_WATCH 2

/**
 * @brief When to look, during @p p, for a node that has taken the bus off us.
 *
 * The whole of contention detection on this bus reduces to one question asked
 * at the right moments: *is the bus active while we intend it passive?* The
 * reverse cannot happen - active dominates, so a node that has released the
 * bus can never see it go passive against its will - and our own active
 * phases are therefore uninformative, because a second node driving active
 * underneath us is indistinguishable from us driving alone.
 *
 * So only passive pulses are watched, and only at the instants another node
 * could legally have started driving. Clause 6.6.2.9 is what makes those
 * instants countable: "all transmitting nodes reference their transmit timing
 * from their receiver's perception of the previous edge", so every node is on
 * the same symbol grid and a competitor can only go active at a symbol
 * boundary. For a passive pulse that leaves at most two:
 *
 *  - at its start, which catches the case where *our own* active phase was
 *    the short form and somebody drove the long one over the top of it;
 *  - one short pulse in, which catches the case where we drove the long form
 *    of a passive bit and somebody drove the short one and went active first.
 *
 * Both are "0" beating "1", clause 7.1, arriving by the two different routes
 * the encoding allows.
 *
 * Two margins, and they are not the same size, because the two ends of a
 * pulse go wrong for different reasons. @p head_us covers the leading edge
 * arriving late - this board's driver turns off slower than it turns on. @p
 * tail_us covers the pulse being *shorter* on the wire than nominal by that
 * same skew, plus whatever error the caller's own time reference carries. An
 * offset is offered only when both margins fit.
 *
 * @param p       The pulse being driven.
 * @param head_us Margin at the start, for the leading edge to settle.
 * @param tail_us Margin at the end, for the wire falling short of nominal.
 * @param out     Offsets from the start of @p p, microseconds, ascending.
 * @param cap     Entries @p out can hold.
 * @return Offsets written. Zero for an active pulse, which says nothing, and
 *         zero for a pulse too short to hold a sample inside both margins.
 */
size_t j1850_vpw_watch_offsets(const j1850_vpw_pulse_t *p, uint16_t head_us,
                               uint16_t tail_us, uint16_t *out, size_t cap);

/* ------------------------------------------------------------------ *
 * Decoding
 * ------------------------------------------------------------------ */

/**
 * @brief What a captured pulse train turned out to be.
 *
 * Everything but J1850_VPW_RX_OK and J1850_VPW_RX_IFR_ONLY is counted rather
 * than delivered; the split matters because a dongle on an unfamiliar vehicle
 * needs to tell "quiet bus" from "wrong protocol" from "marginal wiring", and
 * one lumped error count cannot.
 *
 * There is no bad-timing verdict here, unlike the PWM decoder. On that bus a
 * pulse can be a legal width in an illegal place, because the bit cell is a
 * separate constraint from the pulse inside it. On this one a symbol *is* a
 * pulse, so a width that decodes at all is in the only place it could be, and
 * what is left to go wrong is the level - which is J1850_VPW_RX_BAD_SYMBOL.
 */
typedef enum {
    J1850_VPW_RX_OK = 0,     /**< SOF, whole bytes, CRC good. */
    J1850_VPW_RX_IFR_ONLY,   /**< Response bytes with no frame in front. */
    J1850_VPW_RX_BREAK,      /**< A BRK symbol, clause 6.6.2.7. */
    J1850_VPW_RX_EMPTY,      /**< Idle, or too little to judge. */
    J1850_VPW_RX_NO_SOF,     /**< Activity that never opened with a SOF. */
    J1850_VPW_RX_BAD_SYMBOL, /**< A width, or a level, that cannot be a symbol.
                              */
    J1850_VPW_RX_FRAMING,    /**< Bit count was not a whole number of bytes. */
    J1850_VPW_RX_TOO_LONG,   /**< Past the 12 byte message limit. */
    J1850_VPW_RX_SHORT,   /**< Under two bytes: no message beneath the CRC. */
    J1850_VPW_RX_BAD_CRC, /**< Well formed, wrong CRC. */
} j1850_vpw_rx_status_t;

/** @brief Short name for logs and the shell. Never NULL. */
const char *j1850_vpw_rx_status_str(j1850_vpw_rx_status_t st);

/**
 * @brief One decoded capture.
 *
 * @p data and @p ifr are filled as far as the decoder got, so a frame that
 * fails on its CRC or runs long still shows its bytes to whoever is
 * diagnosing the bus.
 */
typedef struct {
    j1850_vpw_rx_status_t status;
    uint8_t data[J1850_VPW_MAX_FRAME]; /**< Message bytes, CRC included. */
    uint8_t len;                       /**< Bytes in @p data. */
    uint8_t ifr[J1850_VPW_MAX_FRAME];  /**< Response bytes after EOD. */
    uint8_t ifr_len;
    bool eod; /**< An EOD symbol closed the data portion. */
    bool nb;  /**< A normalization bit followed that EOD. */

    /**
     * The normalization bit was long (Tv2) rather than short (Tv1).
     *
     * Clause 6.6.2.5 makes this the responder's declaration of what follows:
     * short for an IFR with no CRC (types 1 and 2), long for one that carries
     * a CRC (type 3). It is a "preferred method" rather than a requirement,
     * so it is reported rather than acted on - a decoder that trusted it
     * would strip a data byte from a manufacturer who did not follow it.
     */
    bool nb_long;
} j1850_vpw_rx_t;

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
j1850_vpw_rx_status_t j1850_vpw_decode(const rmt_symbol_word_t *sym, size_t n,
                                       j1850_vpw_rx_t *out);
