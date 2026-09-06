/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file j1850_pwm.h
 * @brief SAE J1850 PWM (41.6 kbps) bus driver - the Ford SCP network.
 *
 * The first four calls are the byte bus shape vif.c dispatches through, the
 * same one kline.c and j1850_vpw.c present. Everything below them is for the
 * shell and for bring-up: an OBD-II dongle meeting an unfamiliar vehicle has
 * to be able to say *why* a bus looks dead, and a single error count cannot.
 *
 * The protocol itself - symbol timing, framing, CRC and the in-frame response
 * rules - lives in j1850_pwm_codec.h, where it is testable off target. This
 * file is the RMT peripheral, the queues and the transmit policy.
 */

#pragma once

#include <stdint.h>

#include "bus.h"
#include "j1850_pwm_codec.h"

/* ------------------------------------------------------------------ *
 * The bus
 * ------------------------------------------------------------------ */

/**
 * @brief The J1850 PWM bus, as vif.c and the front-ends see it.
 *
 * Transfers, the acknowledgement policy, the functional message lookup table
 * and the retransmission window all go through bus.h, so nothing above
 * this file has to know which bus it holds. Protocol specific values that
 * travel through the generic interface:
 *
 *  - BUS_P_NODE_ADDRESS and BUS_P_IFR_BYTE are the same byte: the one
 *    this node drives as an in-frame response. J2534 calls it the node
 *    address, SAE J1850 calls it the IFR.
 *  - BUS_IOCTL_ADD_FUNCT_ADDR and its siblings maintain the eight-address
 *    functional message lookup table of J2534 clause 6.5.3, which here is the
 *    list of target bytes this node acknowledges.
 *  - BUS_P_DATA_RATE reads 41600 and accepts nothing else. J2534 allows
 *    83.3 kbps as well; the symbol tables for it are not built, and reporting
 *    a rate that is not being driven would be worse than refusing it.
 *
 * The two calls below are outside that interface on purpose. Both expose
 * pulse widths and per-symbol decode failures, which exist on no other bus,
 * and giving them shared names would produce counters that read the same and
 * mean different things depending on which driver answered.
 */
extern const bus_ops_t j1850_pwm_bus_ops;

/**
 * @brief The duplicate window an ELM327 client wants, milliseconds.
 *
 * A module that asked for an in-frame response and did not get one sends its
 * frame again, twice over, a few milliseconds apart - it has no way
 * of knowing the frame arrived. Those repeats are one reply as far as an
 * OBD-II client is concerned, and delivering three copies of a mode 01
 * response is not what any of them expect.
 *
 * So an identical frame arriving inside this window of the last one is
 * counted and dropped. The window is short by diagnostic standards and long
 * by retransmission standards: a genuinely repeated broadcast is tens of
 * milliseconds apart; this bench's ten-byte retransmissions are 2.04 ms apart.
 *
 * It is not the driver's default. BUS_P_DUPLICATE_MS starts at zero,
 * because "deliver everything" is what a bus trace wants and what J2534
 * clause 6.5.1 requires of a Pass-Thru channel; the ELM327 front-end asks for
 * this value when it takes the bus.
 */
#define J1850_PWM_DUPLICATE_MS 5

/* ------------------------------------------------------------------ *
 * Diagnostics
 * ------------------------------------------------------------------ */

/**
 * @brief What the bus has done since the counters were last cleared.
 *
 * The receive counters are split by failure mode on purpose. A quiet bus, a
 * vehicle speaking VPW instead of PWM, and a marginal connection each look
 * completely different here, and identical in a single "errors" number.
 */
typedef struct {
    uint32_t rx_frames; /**< Valid captures, including TX echoes. */
    uint32_t rx_ifr;    /**< In-frame responses seen from other nodes. */
    uint32_t rx_break;  /**< BRK symbols, clause 6.6.1.6. */
    uint32_t rx_bad_crc;
    uint32_t rx_framing;    /**< Bit count was not a whole number of bytes. */
    uint32_t rx_bad_symbol; /**< Pulse widths that match no J1850 PWM symbol. */
    uint32_t rx_bad_timing; /**< Edges spaced outside every window. */
    uint32_t rx_no_sof;     /**< Activity that never looked like a frame. */
    uint32_t rx_too_long;   /**< Past the 12 byte message limit. */
    uint32_t rx_short;      /**< Under two bytes. */
    uint32_t rx_empty;      /**< A capture with no edges in it. */
    uint32_t rx_dropped;    /**< Good frames lost because the queue was full. */
    uint32_t rx_duplicate;  /**< Retransmissions collapsed into one frame. */
    uint32_t tx_frames;     /**< Frames handed to the peripheral. */
    uint32_t tx_retries;    /**< Retransmissions after a silent bus. */
    uint32_t tx_bus_busy;   /**< Sends refused: the bus never went idle. */
    uint32_t tx_echo_ok;    /**< Own frames read back off the wire intact. */
    uint32_t tx_echo_missing; /**< Transmits the receiver never saw. */
    uint32_t ifr_sent;        /**< In-frame responses this node drove. */
    uint32_t ifr_sof;
    uint32_t ifr_candidates;
    uint32_t ifr_bad_pulse;
    uint32_t ifr_bad_gap;
    uint32_t ifr_late;
    uint32_t ifr_lost;
    uint32_t ifr_observed;
    uint32_t ifr_gap_min_us;
    uint32_t ifr_gap_max_us;

    /** Estimated IFR start after the GPIO rising-edge timestamp, excluding ISR
     * latency. Table 3: TX 47..49 us, RX 42..54 us. Verify on the wire. */
    uint32_t ifr_start_us;
    uint32_t rx_isr_us_max; /**< Longest receive interrupt, microseconds. */
} j1850_pwm_stats_t;

/**
 * @brief This driver's own counters, past what bus_stats_t can name.
 *
 * The receive counters are split by failure mode on purpose. A quiet bus, a
 * vehicle speaking VPW instead of PWM, and a marginal connection each look
 * completely different here, and identical in a single "errors" number.
 */
void j1850_pwm_get_native_stats(j1850_pwm_stats_t *out);

/**
 * @brief Symbols one capture can hold.
 *
 * A maximum length frame is 97 and its in-frame response adds eight, so this
 * is headroom rather than a limit. The same buffer is the diagnostic dump and
 * the staging copy the receive interrupt decodes from once it has put the
 * peripheral back on the air, so it has to be able to hold anything the
 * receiver might capture, noise included.
 */
#define J1850_PWM_CAPTURE_MAX 192

/** Which stored capture to read back. */
typedef enum {
    J1850_PWM_CAP_LAST, /**< The most recent activity of any kind. */
    /**
     * The last frame this node transmitted, as its own receiver saw it.
     *
     * Kept separately because it is the one capture that is always available
     * and always attributable: whatever else is on the bus, this is our own
     * driver stage measured through our own front end. It is what the trim in
     * J1850_PWM_TX_TRIM_US is derived from, and what the target test suite
     * checks against the transmit windows of Table 3.
     */
    J1850_PWM_CAP_TX_ECHO,
} j1850_pwm_capture_sel_t;

/**
 * @brief The raw pulse train behind a stored capture.
 *
 * The point of exposing timings rather than a decode is calibration. A
 * transceiver whose turn-on and turn-off delays differ stretches every active
 * pulse by the same amount, and that shows up here as a fleet of Tp1 pulses
 * sitting at 9 us instead of 7 - long before it starts costing frames, and
 * invisible in any error counter until it does.
 *
 * @param which  Which capture to read.
 * @param out    Destination for the symbols.
 * @param cap    Symbols @p out can hold.
 * @param status Filled with the decoder's verdict on that capture.
 * @return Symbols copied, 0 if nothing of that kind has been captured yet.
 */
size_t j1850_pwm_get_capture(j1850_pwm_capture_sel_t which,
                             rmt_symbol_word_t *out, size_t cap,
                             j1850_pwm_rx_status_t *status);
