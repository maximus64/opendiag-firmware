/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file j1850_vpw.h
 * @brief SAE J1850 VPW (10.4 kbps) bus driver - the GM class 2 network.
 *
 * The protocol itself - symbol timing, framing and the in-frame response
 * rules - lives in j1850_vpw_codec.h, where it is testable off target. This
 * file is the RMT peripheral, the queues and the transmit policy.
 */

#pragma once

#include <stdint.h>

#include "bus.h"
#include "j1850_vpw_codec.h"

/* ------------------------------------------------------------------ *
 * The bus
 * ------------------------------------------------------------------ */

/**
 * @brief The J1850 VPW bus, as vif.c and the front-ends see it.
 *
 * Transfers, the retransmission window and the checksum policy all go through
 * bus.h, so nothing above this file has to know which bus it holds. Protocol
 * specific values that travel through the generic interface:
 *
 *  - BUS_P_DATA_RATE reads 10400 and accepts nothing else. GM's 4X mode runs
 *    the same symbols at 41.6 kbps, but it is not in SAE J1850 and none of
 *    its timings are built here; reporting a rate that is not being driven
 *    would be worse than refusing it.
 *  - BUS_P_IFR_ENABLED accepts 0 and refuses 1. The mechanism for answering
 *    another node's frame is deliberately absent rather than present and
 *    broken; the comment on j1850_vpw_stats_t::rx_ifr has the arithmetic.
 *  - BUS_IOCTL_ADD_FUNCT_ADDR and its siblings answer BUS_ERR_UNSUPPORTED for
 *    the same reason: the functional address table exists to decide which
 *    frames this node acknowledges, and this node acknowledges none.
 *
 * The two calls below are outside that interface on purpose. Both expose
 * pulse widths and per-symbol decode failures, which exist on no other bus,
 * and giving them shared names would produce counters that read the same and
 * mean different things depending on which driver answered.
 */
extern const bus_ops_t j1850_vpw_bus_ops;

/**
 * @brief The duplicate window an ELM327 client wants, milliseconds.
 *
 * A module that asked for an in-frame response and did not get one sends its
 * frame again, and an OBD-II client expects one reply per request rather than
 * a running commentary. This is the same idea as J1850_PWM_DUPLICATE_MS and a
 * different number, because a VPW frame is four times as long in the air: a
 * retransmission cannot arrive sooner than an inter-frame separation after
 * the one before it, and the shortest useful frame already occupies about
 * three milliseconds.
 *
 * It is not the driver's default. BUS_P_DUPLICATE_MS starts at zero, because
 * "deliver everything" is what a bus trace wants and what J2534 clause 6.5.1
 * requires of a Pass-Thru channel; the ELM327 front-end asks for this value
 * when it takes the bus.
 */
#define J1850_VPW_DUPLICATE_MS 20

/* ------------------------------------------------------------------ *
 * Diagnostics
 * ------------------------------------------------------------------ */

/**
 * @brief What the bus has done since the counters were last cleared.
 *
 * The receive counters are split by failure mode on purpose. A quiet bus, a
 * vehicle speaking PWM instead of VPW, and a marginal connection each look
 * completely different here, and identical in a single "errors" number.
 */
typedef struct {
    uint32_t rx_frames; /**< Good frames handed to the caller. */

    /**
     * Captures that held only an in-frame response, clause 5.3.7.
     *
     * These are other nodes answering each other, and seeing them is the
     * whole of this driver's involvement with the feature. Answering would
     * mean driving a normalization bit inside the originator's 200 us end of
     * data, and the capture that would say a response is owed does not reach
     * software until the arming threshold has expired - which is after that
     * window has closed. So an IFR is decoded, counted and reported, and
     * never sent. J1850 OBD-II requests carry K = 1 and ask for none.
     */
    uint32_t rx_ifr;
    uint32_t rx_break; /**< BRK symbols, clause 6.6.2.7. */
    uint32_t rx_bad_crc;
    uint32_t rx_framing;    /**< Bit count was not a whole number of bytes. */
    uint32_t rx_bad_symbol; /**< A width, or a level, that is not a symbol. */
    uint32_t rx_no_sof;     /**< Activity that never looked like a frame. */
    uint32_t rx_too_long;   /**< Past the 12 byte message limit. */
    uint32_t rx_short;      /**< Under two bytes. */
    uint32_t rx_empty;      /**< A capture with no symbols in it. */
    uint32_t rx_dropped;    /**< Good frames lost because the queue was full. */
    uint32_t rx_duplicate;  /**< Retransmissions collapsed into one frame. */
    uint32_t tx_frames;     /**< Frames handed to the peripheral. */
    uint32_t tx_retries;    /**< Retransmissions after a silent bus. */
    uint32_t tx_bus_busy;   /**< Sends refused: the bus never went idle. */
    uint32_t tx_echo_ok;    /**< Own frames read back off the wire intact. */
    uint32_t tx_echo_missing; /**< Transmits the receiver never saw. */

    /**
     * Transmissions stopped part way because another node had the bus.
     *
     * SAE J1850 clause 6.7.2's contention detection. A frame counted here was
     * taken off the wire before it could corrupt the one that won, and - up
     * to BUS_P_TX_RETRIES - sent again once the bus went quiet. On a healthy
     * two-node bench this stays at zero; on a vehicle it is the number that
     * says how contended the network actually is.
     */
    uint32_t tx_arb_lost;
    uint32_t tx_arb_checks; /**< Bus samples the monitor took. */

    /**
     * Samples abandoned because the interrupt arrived too late to trust them.
     *
     * Every check is a question about one named phase of one named bit, and
     * the answer is only worth having while that phase is still on the wire.
     * An interrupt delayed past the end of it - by a receive interrupt, by
     * the radio, by anything - would be reading the *next* phase, which this
     * node is driving active, and would report a collision that never
     * happened. Skipped rather than believed, and counted here so that
     * "never fires" and "fires but is always late" stay distinguishable.
     */
    uint32_t tx_arb_late;

    /**
     * Frames sent without a monitor because the reference could not be timed.
     *
     * The whole schedule hangs off one measurement - the falling edge that
     * ends this node's own start of frame - and that measurement is taken by
     * polling a pin from a task, with interrupts enabled. A receive interrupt
     * landing on it can push the reference tens of microseconds late, and
     * every check after it inherits the error. So the edge is bracketed and a
     * reference that cannot be placed closely enough is discarded, taking the
     * monitor off for that frame rather than letting it accuse the node of
     * colliding with itself. The transmission still goes out and the echo
     * check still covers it.
     */
    uint32_t tx_arb_unsynced;

    /** Bit position the last loss was detected at, for diagnosing one. */
    uint32_t tx_arb_pulse;
    uint32_t rx_isr_us_max; /**< Longest receive interrupt, microseconds. */
} j1850_vpw_stats_t;

/**
 * @brief This driver's own counters, past what bus_stats_t can name.
 */
void j1850_vpw_get_native_stats(j1850_vpw_stats_t *out);

/**
 * @brief Symbols one capture can hold.
 *
 * A maximum length frame is 49 symbols and an in-frame response adds a few,
 * so this is headroom rather than a limit. The same buffer is the diagnostic
 * dump and the staging copy the receive interrupt decodes from once it has
 * put the peripheral back on the air, so it has to be able to hold anything
 * the receiver might capture, noise included.
 */
#define J1850_VPW_CAPTURE_MAX 128

/** Which stored capture to read back. */
typedef enum {
    J1850_VPW_CAP_LAST, /**< The most recent activity of any kind. */
    /**
     * The last frame this node transmitted, as its own receiver saw it.
     *
     * Kept separately because it is the one capture that is always available
     * and always attributable: whatever else is on the bus, this is our own
     * driver stage measured through our own front end. It is what the target
     * test suite checks against the transmit windows of Table 5.
     */
    J1850_VPW_CAP_TX_ECHO,
} j1850_vpw_capture_sel_t;

/**
 * @brief The raw pulse train behind a stored capture.
 *
 * The point of exposing timings rather than a decode is calibration. A
 * transceiver whose turn-on and turn-off delays differ stretches every active
 * pulse by the same amount, and that shows up here long before it starts
 * costing frames - invisible in any error counter until it does.
 *
 * @param which  Which capture to read.
 * @param out    Destination for the symbols.
 * @param cap    Symbols @p out can hold.
 * @param status Filled with the decoder's verdict on that capture.
 * @return Symbols copied, 0 if nothing of that kind has been captured yet.
 */
size_t j1850_vpw_get_capture(j1850_vpw_capture_sel_t which,
                             rmt_symbol_word_t *out, size_t cap,
                             j1850_vpw_rx_status_t *status);

/**
 * @brief Transmit deliberately on top of a frame already in progress.
 *
 * A test hook, and a deliberate violation of clause 5.3.4.4 - which is the
 * point. Contention cannot be tested by waiting for it: a bench with one
 * other module produces a collision perhaps never, and a driver whose
 * arbitration has quietly stopped working looks exactly like a bus that
 * happened to be quiet.
 *
 * So this skips the inter-frame separation and starts inside a passive phase
 * of somebody else's frame, which is the same shape as a real collision - two
 * nodes bit-synchronised on one grid, one of them recessive - and forces the
 * monitor to do its job while a real module is watching.
 *
 * Never called by the front-ends. It is reachable from the debug console
 * only, and it is expected to fail: BUS_ERR_ARBITRATION is the successful
 * outcome.
 *
 * @return BUS_ERR_ARBITRATION when the monitor did its job, 0 if this node
 *         won the bus outright, or another BUS_ERR_* code.
 */
int j1850_vpw_force_collision(const uint8_t *data, size_t len);

/**
 * @brief Make the @p nth contention check of the next frame report a loss.
 *
 * The other half of testing arbitration, and the half a bench cannot supply.
 * j1850_vpw_force_collision() puts real interference on a real bus, but what
 * it demonstrates is that the *module* gives up first: a compliant node meeting
 * a 200 us start of frame where it expected a bit abandons its own message
 * almost immediately, after which this node is alone and correctly detects
 * nothing. Producing a contention that outlives its own detection needs a
 * second transmitter that does not back off, which the bench does not have.
 *
 * So the detection and the response are tested separately. Whether the checks
 * fall in the right places is settled off target, against the encoder, in
 * test_j1850_vpw_codec.c. What is left is everything that happens *after* a
 * check says yes - the pins coming off the peripheral inside a microsecond,
 * the RMT being recovered, the retry, and the bus still working afterwards -
 * and that is what this exercises, on real hardware, deterministically.
 *
 * @param nth Check ordinal within the frame, counting from one. Zero disables.
 *            Spent on the next transmission and not repeated.
 */
void j1850_vpw_inject_arbitration_loss(uint32_t nth);
