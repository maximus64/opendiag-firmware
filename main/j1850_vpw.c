/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file j1850_vpw.c
 * @brief SAE J1850 VPW bus driver: the RMT peripheral, the queue, the policy.
 *
 * Receiving
 * ---------
 * The RMT captures pulse widths at 1 us and closes a capture once the input
 * has held one level for J1850_VPW_RX_IDLE_US. "One level", not "idle": the
 * peripheral counts an active phase the same way it counts a passive one, so
 * the threshold has to clear every symbol that is a single steady level - the
 * 200 us of a start of frame and the 200 us of an end of data alike, both of
 * which may run to 239 us and still be legal. Above that there is exactly one
 * thing left, which is the end of frame, and that is what the threshold is
 * set to catch.
 *
 * That the end of data sits *under* the threshold is the point rather than an
 * accident: it keeps a frame and the in-frame response that follows it in one
 * capture, which is the only way to know the two belong together.
 *
 * Transmitting
 * ------------
 * The receiver is left armed during transmission rather than muted, so this
 * node reads its own frames back off the wire. That is not a side effect to
 * be tolerated, it is the transmit check: a frame that comes back intact
 * confirms the driver stage, and one that never appears says the transceiver
 * is not driving, which is otherwise indistinguishable from a quiet vehicle.
 * Echoes are filtered out before the receive queue, so callers never see them.
 *
 * The send waits for that echo before returning. On PWM the equivalent check
 * is made after the fact, because a frame there is over in two milliseconds;
 * here a frame is four times as long and the echo lands a quarter of a
 * millisecond after the last edge, so waiting for it costs nothing measurable
 * and turns "we handed it to the peripheral" into "it was on the wire".
 *
 * Arbitration
 * -----------
 * Clause 6.7.2 has each node compare the bus against what it is driving and
 * drop out on the first difference, and this driver does. An RMT transmission
 * cannot be steered once it starts, but it can be taken off the pin - so the
 * bus is sampled at the instants a competitor could have taken it, and losing
 * one costs two register writes to get off the wire. The reasoning, the
 * timing budget and the way it is tested are in the "Contention monitor"
 * section below.
 *
 * What that buys is not this node's own frame. A transmitter that never drops
 * out goes on to corrupt, at the next bit where it is the dominant one, a
 * frame that legitimately won the bus - so a collision costs two messages
 * instead of one, and one of them belonged to somebody else.
 */

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/soc.h"

#include "bus.h"
#include "j1850_lifecycle.h"
#include "j1850_vpw.h"
#include "pinout.h"

#define TAG "J1850_VPW"

/* ------------------------------------------------------------------ *
 * Tuning
 * ------------------------------------------------------------------ */

/** 1 tick = 1 us, which is the unit every J1850 timing is written in. */
#define RMT_RESOLUTION_HZ 1000000

/**
 * @brief The timebase every symbol on this bus is measured against.
 *
 * Named rather than left as RMT_CLK_SRC_DEFAULT, because what it must not be
 * matters: the internal RC oscillator is uncalibrated and drifts with
 * temperature. VPW's windows are wide enough to survive a percent or two of
 * that, unlike PWM's, but a clock that moves with the die temperature would
 * turn a marginal vehicle into an intermittent one. APB is crystal derived,
 * so 80 MHz divided by 80 is exactly 1 MHz.
 */
#define RMT_CLK_SRC RMT_CLK_SRC_APB

/**
 * @brief How long one level may persist before the capture is closed.
 *
 * The floor is Tv3(rx,max), 239 us: a start of frame and an end of data are
 * both single steady levels that may legally run that long, and a threshold
 * under either chops the capture in the middle of the symbol that identifies
 * it. The ceiling is the end of frame this is meant to detect, Tv4 nominal at
 * 280 us.
 *
 * 250 us sits between them with 11 us of margin over the longest legal symbol
 * and 30 us under the shortest end of frame a conforming node produces. Above
 * the floor, lower is better: this threshold is dead time before the receive
 * interrupt can react to anything, and it is the whole of the latency between
 * a module finishing its reply and this driver having the bytes.
 */
#define J1850_VPW_RX_IDLE_US 250

/**
 * @brief Glitch filter. Clause 6.6.2.9 expects a receiver to have one.
 *
 * A pulse under Tv1(rx,min) is not a symbol, so in principle anything up to
 * 34 us could be filtered. The peripheral sets the ceiling instead: the
 * filter counts source clock ticks in an eight bit field, which at 80 MHz is
 * 3.19 us and no more. That is comfortably enough for the ringing an edge
 * produces on a 40 metre network, and the decoder rejects what gets past it.
 */
#define J1850_VPW_RX_FILTER_NS 3000

/** Symbols the receive buffer holds. The staging copy has to match it. */
#define RX_BUFFER_SYMBOLS J1850_VPW_CAPTURE_MAX

/**
 * @brief RMT memory the receive channel is given, in symbols.
 *
 * Two of the peripheral's blocks, which is 96 symbols and so more than the 49
 * a maximum length frame needs: a whole frame lands without the driver having
 * to copy a half block out mid-capture.
 *
 * Deliberately *not* DMA. Re-arming a DMA receive means a cache writeback
 * over the whole buffer, and on the sibling PWM driver the measured cost of
 * that put the receiver back on the air too late to catch the start of a
 * module's retransmission. Programmed I/O re-arms in a fraction of the time,
 * and a J1850 frame is far too small to need DMA.
 */
#define RX_MEM_SYMBOLS (SOC_RMT_MEM_WORDS_PER_CHANNEL * 2)

/** Frames buffered for the caller. Deep enough for a burst of module replies.
 */
#define RX_QUEUE_DEPTH 8

/** How long j1850_vpw_tx() waits for an idle bus before giving up. */
#define TX_BUS_IDLE_TIMEOUT_MS 100

/**
 * @brief How long to wait for this node's own frame to come back.
 *
 * The capture closes J1850_VPW_RX_IDLE_US after the frame's last edge, and
 * the transmission itself is already over by then - the encoder emits the end
 * of data, so rmt_tx_wait_all_done() returns 200 us into that quiet period.
 * What is left is the remaining threshold plus one interrupt, which is tens
 * of microseconds. Ten milliseconds is therefore not a timeout that a working
 * bus ever approaches; it is the point at which "no echo" becomes the answer.
 */
#define TX_ECHO_MS 10

/**
 * @brief How long a retry waits to be convinced the bus really is silent.
 *
 * Longer than any legal response latency on this bus: a module's reply frame
 * is itself several milliseconds of air time, and the ones on the bench land
 * inside twenty-five.
 */
#define TX_SILENCE_MS 40

/**
 * @brief Extra attempts when nothing at all answers.
 *
 * Two, matching the PWM driver rather than the zero bus.h names as its
 * default: an ELM327 client has no other recovery, and the retry only ever
 * happens into a bus that stayed completely silent, which cannot duplicate a
 * request that was heard.
 *
 * A J2534 channel does *not* want this at zero, which is the opposite of what
 * an earlier version of this comment claimed. Clause 6.5.1's "shall not
 * resend" is in the ISO 9141 section and is about UART byte echo; for this
 * bus, Table 8 puts loss of arbitration under the controller retry strategy -
 * retried automatically, with nothing reported to the application.
 */
#define TX_DEFAULT_RETRIES 2

/**
 * @brief How long to let an edge settle before believing the bus, in us.
 *
 * Every contention check reads the pin a fixed distance into a phase we are
 * driving passive, and the fixed distance has to clear this board's own
 * skew: "vpw dump tx" puts the falling edge about 4 us late, so a check any
 * sooner than that would read our own transmission as somebody else's.
 *
 * It is charged at *both* ends of a pulse, and the far end is the one that
 * matters more. A passive phase is shorter on the wire than nominal by the
 * same skew - a 64 us bit measures 59 - so a sample in that missing tail
 * reads this node's own next active phase and reports a collision against
 * itself. That cost about one frame in twenty-five until it was allowed for.
 *
 * Twelve leaves a 64 us pulse a window of [12, 52] to be sampled in, against
 * a phase that really ends at 59, and the deadline the abort has to beat -
 * the next time this node drives active - is a further 52 us past that. At
 * 41.6 kbps the guard has to become a function of the bit rate rather than a
 * constant.
 */
#define TX_WATCH_GUARD_US 12

/** How long a frame may take before the peripheral is presumed wedged. */
#define TX_COMPLETE_TIMEOUT_MS 100

/**
 * @brief How closely the monitor's reference edge has to be placed, in us.
 *
 * The falling edge that ends our start of frame is found by polling a pin
 * from a task, so the answer is only as good as the time between the poll
 * that still saw the bus active and the one that saw it passive. Uninterrupted
 * that is a couple of microseconds; with a receive interrupt landing in the
 * middle of it, it can be a hundred, and a reference that late walks the very
 * first check into the node's own next active phase.
 *
 * So the edge is bracketed and the bracket is checked. Eight microseconds is
 * comfortably more than the loop costs and comfortably less than the margin
 * the guard band leaves, and a frame that cannot be placed inside it is sent
 * unwatched rather than watched wrongly.
 */
#define TX_SYNC_SLACK_US 8

/**
 * @brief Margin at the far end of a passive pulse, microseconds.
 *
 * Bigger than the head guard, because two errors accumulate there rather than
 * one. The pulse is about 4 us shorter on the wire than nominal - the active
 * phase before it ran long by the same amount - and the reference the whole
 * schedule hangs off may itself be TX_SYNC_SLACK_US late. A sample past this
 * point is reading the node's own next active phase.
 *
 * With it, a 64 us passive bit is sampled somewhere in [12, 44] against a
 * phase that really ends no earlier than 51. Without it the window closed
 * exactly on the edge, and cost about one frame in sixty.
 */
#define TX_WATCH_TAIL_US (TX_WATCH_GUARD_US + TX_SYNC_SLACK_US)

/* ------------------------------------------------------------------ *
 * State
 * ------------------------------------------------------------------ */

/**
 * @brief One received frame on its way to the caller.
 *
 * The timestamp is taken in the receive interrupt, which is the only place
 * that knows when the frame was actually on the wire; anything read later
 * would be the time the caller got round to asking.
 */
struct rx_frame {
    uint8_t len;
    uint16_t status; /**< BUS_RX_* flags. */
    uint32_t timestamp_us;
    uint8_t data[J1850_VPW_MAX_FRAME];
};

static struct {
    bool started;
    bool initialized;
    bool tx_enabled;
    bool rx_enabled;
    j1850_callback_guard_t rx_guard;
    rmt_symbol_word_t tx_symbols[J1850_VPW_MAX_SYMBOLS];
    rmt_channel_handle_t tx_chan;
    rmt_channel_handle_t rx_chan;
    rmt_encoder_handle_t copy_encoder;
    rmt_symbol_word_t *rx_buf; /**< RX_BUFFER_SYMBOLS. */
    QueueHandle_t rx_queue;
    SemaphoreHandle_t tx_lock;

    /** Given by the receive interrupt when this node's own frame comes back. */
    SemaphoreHandle_t echo_sem;

    /**
     * Given by the transmit interrupt when the peripheral is done.
     *
     * Waited on in preference to rmt_tx_wait_all_done() because a
     * transmission here has two ways to end - it finishes, or the contention
     * monitor takes the pin off the wire - and only the first is an event the
     * RMT knows about. Polling that call with a short timeout works, but it
     * logs an error on every timeout, which turns a normal frame into two
     * lines of driver noise.
     */
    SemaphoreHandle_t tx_done_sem;

    uint8_t retries;

    /**
     * The routing the RMT driver gave the transmit pin, saved so the
     * contention monitor can take the pin over and hand it back without
     * having to know which channel was allocated.
     */
    uint32_t tx_func_sel;

    /**
     * Watching our own transmission for a node that has taken the bus.
     *
     * Everything up to @c lost is written under tx_lock before a frame starts
     * and only read by the alarm interrupt afterwards, so no ordering is
     * needed between them; @c lost crosses back the other way and is the one
     * field the transmitting task polls.
     */
    struct {
        gptimer_handle_t timer;
        bool timer_enabled;
        bool timer_running;
        j1850_callback_guard_t guard;
        bool enabled;
        uint8_t frame[J1850_VPW_MAX_FRAME];
        uint8_t len;
        size_t pulse;         /**< Pulse being watched. */
        uint64_t pulse_start; /**< Its start, in timer ticks. */
        uint64_t pulse_end;   /**< And the end, past which a sample
                               *  is about the wrong phase. */
        uint16_t offs[J1850_VPW_MAX_WATCH];
        uint8_t n_offs;
        uint8_t next_off;
        volatile bool lost;

        /* Modified only while the callback gate is closed. */
        volatile bool running;

        /* Fault injection, for the target suite. Zero is off. */
        uint32_t inject_at;
        uint32_t checks;
    } mon;

    /**
     * Policy the front-end chooses; see bus.h. The CRC flags exist because
     * J2534's CHECKSUM_DISABLED lets an application drive its own error
     * detection, and loopback because ECHO_PHYSICAL_CHANNEL_TX lets it see
     * its own transmissions in the receive stream.
     */
    bool crc_tx;
    bool crc_rx;
    bool loopback;

    /** Flags owed to the next frame delivered: J2534 clause 6.10 puts a
     *  buffer overflow on the first message after the loss, not on the one
     *  that was lost. */
    volatile uint16_t pending_status;

    /**
     * Echo suppression. The frame is written under tx_lock and read in the
     * receive interrupt, which can be running on the other core, so the flag
     * is published with a release store once the bytes beside it are in
     * place and read with an acquire load. Without that ordering the
     * interrupt could see the flag set over the previous frame's bytes and
     * pass this node's own transmission up to the caller.
     */
    bool echo_armed;
    uint8_t echo[J1850_VPW_MAX_FRAME];
    uint8_t echo_len;

    /* Retransmission collapsing. Only the receive interrupt touches these. */
    uint32_t dup_window_us;
    uint8_t last_frame[J1850_VPW_MAX_FRAME];
    uint8_t last_frame_len;
    uint32_t last_frame_us;

    /**
     * Captures that were not this node's own echo, of any kind at all.
     *
     * A transmit that leaves this unchanged is one nothing on the bus
     * reacted to, which is the only condition under which a retry can be
     * sent without risking a duplicate request. Our own echo is excluded
     * precisely because it is not somebody reacting.
     */
    volatile uint32_t rx_foreign;

    j1850_vpw_stats_t stats;

    /* Last capture, for the shell's calibration dump. */
    rmt_symbol_word_t cap[J1850_VPW_CAPTURE_MAX];
    volatile uint16_t cap_len;
    volatile uint8_t cap_status;

    /* And the last one that turned out to be our own transmission, kept so
     * the driver stage can be measured whatever else the bus is doing. */
    rmt_symbol_word_t tx_cap[J1850_VPW_CAPTURE_MAX];
    volatile uint16_t tx_cap_len;
    volatile uint8_t tx_cap_status;
} g;

static const rmt_receive_config_t g_rx_config = {
    .signal_range_min_ns = J1850_VPW_RX_FILTER_NS,
    .signal_range_max_ns = J1850_VPW_RX_IDLE_US * 1000,
};

/* ------------------------------------------------------------------ *
 * Receive
 * ------------------------------------------------------------------ */

static void IRAM_ATTR count_status(j1850_vpw_rx_status_t st) {
    switch (st) {
    case J1850_VPW_RX_OK:
        g.stats.rx_frames++;
        break;
    case J1850_VPW_RX_IFR_ONLY:
        g.stats.rx_ifr++;
        break;
    case J1850_VPW_RX_BREAK:
        g.stats.rx_break++;
        break;
    case J1850_VPW_RX_BAD_CRC:
        g.stats.rx_bad_crc++;
        break;
    case J1850_VPW_RX_FRAMING:
        g.stats.rx_framing++;
        break;
    case J1850_VPW_RX_BAD_SYMBOL:
        g.stats.rx_bad_symbol++;
        break;
    case J1850_VPW_RX_NO_SOF:
        g.stats.rx_no_sof++;
        break;
    case J1850_VPW_RX_TOO_LONG:
        g.stats.rx_too_long++;
        break;
    case J1850_VPW_RX_SHORT:
        g.stats.rx_short++;
        break;
    default:
        g.stats.rx_empty++;
        break;
    }
}

/** @brief True when these bytes are the frame we just put on the wire. */
static bool IRAM_ATTR is_own_echo(const uint8_t *data, size_t len) {
    if (!__atomic_load_n(&g.echo_armed, __ATOMIC_ACQUIRE) ||
        len != g.echo_len) {
        return false;
    }

    return memcmp(data, g.echo, len) == 0;
}

/**
 * @brief True when this frame is the previous one sent again.
 *
 * A J1850 frame carries nothing to tell a retransmission from a repeat, so
 * this is a judgement about time: the same bytes, this soon, from a bus whose
 * retransmission interval is an inter-frame separation.
 */
static bool IRAM_ATTR is_retransmission(const j1850_vpw_rx_t *rx,
                                        uint32_t now) {
    if (g.dup_window_us == 0 || g.last_frame_len != rx->len) {
        return false;
    }
    if ((uint32_t)(now - g.last_frame_us) > g.dup_window_us) {
        return false;
    }

    return memcmp(g.last_frame, rx->data, rx->len) == 0;
}

/** @brief Hand one frame to the caller, or count it lost. */
static void IRAM_ATTR queue_frame(const j1850_vpw_rx_t *rx, uint16_t status,
                                  uint32_t now, BaseType_t *woken) {
    struct rx_frame f;

    f.len = rx->len;
    f.status = (uint16_t)(g.pending_status | status);
    f.timestamp_us = now;
    g.pending_status = 0;
    memcpy(f.data, rx->data, rx->len);

    if (xQueueSendFromISR(g.rx_queue, &f, woken) != pdTRUE) {
        g.stats.rx_dropped++;
        g.pending_status |= BUS_RX_BUFFER_OVERFLOW;
    }
}

static bool receive_done(rmt_channel_handle_t channel,
                         const rmt_rx_done_event_data_t *edata, void *user_ctx);

static bool IRAM_ATTR on_rx_done(rmt_channel_handle_t channel,
                                 const rmt_rx_done_event_data_t *edata,
                                 void *user_ctx) {
    if (!j1850_callback_enter(&g.rx_guard))
        return false;
    bool woken = receive_done(channel, edata, user_ctx);
    j1850_callback_exit(&g.rx_guard);
    return woken;
}

static bool IRAM_ATTR receive_done(rmt_channel_handle_t channel,
                                   const rmt_rx_done_event_data_t *edata,
                                   void *user_ctx) {
    uint32_t entry_us = (uint32_t)esp_timer_get_time();
    BaseType_t woken = pdFALSE;
    j1850_vpw_rx_t rx;
    size_t n;
    bool echo;

    (void)user_ctx;

    /* Take a copy and put the receiver straight back on the air. Everything
     * below works from the copy, so the bus is unwatched for the length of a
     * memcpy rather than for the length of a decode - which matters, because
     * a module that did not get an answer starts retransmitting one
     * inter-frame separation from now. */
    n = edata->num_symbols;
    if (n > J1850_VPW_CAPTURE_MAX) {
        n = J1850_VPW_CAPTURE_MAX;
    }
    memcpy(g.cap, edata->received_symbols, n * sizeof(rmt_symbol_word_t));

    if (rmt_receive(channel, g.rx_buf,
                    RX_BUFFER_SYMBOLS * sizeof(rmt_symbol_word_t),
                    &g_rx_config) != ESP_OK) {
        esp_rom_printf("J1850 VPW: receiver would not re-arm\n");
    }

    j1850_vpw_decode(g.cap, n, &rx);
    g.cap_len = (uint16_t)n;
    g.cap_status = (uint8_t)rx.status;

    count_status(rx.status);

    /* A frame that failed its CRC is still ours if it is byte-for-byte what
     * we sent, which is what makes "the transceiver is not driving" and "the
     * bus corrupted it" different answers rather than one. */
    echo =
        (rx.status == J1850_VPW_RX_OK || rx.status == J1850_VPW_RX_BAD_CRC) &&
        is_own_echo(rx.data, rx.len);

    if (echo) {
        __atomic_store_n(&g.echo_armed, false, __ATOMIC_RELAXED);
        g.stats.tx_echo_ok++;

        /* Off the critical path deliberately: the receiver is already back on
         * the air by here, so keeping a second copy costs nothing that
         * matters. */
        memcpy(g.tx_cap, g.cap, n * sizeof(rmt_symbol_word_t));
        g.tx_cap_len = (uint16_t)n;
        g.tx_cap_status = (uint8_t)rx.status;

        xSemaphoreGiveFromISR(g.echo_sem, &woken);
    } else {
        /* Anything at all that was not us: the bus is alive and whoever we
         * were talking to has had their say. */
        g.rx_foreign++;
    }

    if (!echo && rx.status == J1850_VPW_RX_OK) {
        if (is_retransmission(&rx, entry_us)) {
            g.stats.rx_duplicate++;
        } else {
            queue_frame(&rx, 0, entry_us, &woken);
        }

        memcpy(g.last_frame, rx.data, rx.len);
        g.last_frame_len = rx.len;
        /* Restarted on every copy, so a module making three attempts has all
         * three collapsed rather than the last one leaking through. */
        g.last_frame_us = entry_us;
    } else if (!echo && rx.status == J1850_VPW_RX_BAD_CRC && !g.crc_rx) {
        /* Verification off: the frame goes up flagged rather than discarded,
         * which is what a bus trace and a manufacturer specific error check
         * both need. bus.h, BUS_P_CHECKSUM_RX. */
        queue_frame(&rx, BUS_RX_BAD_CHECKSUM, entry_us, &woken);
    }

    {
        uint32_t took = (uint32_t)esp_timer_get_time() - entry_us;

        if (took > g.stats.rx_isr_us_max) {
            g.stats.rx_isr_us_max = took;
        }
    }

    return woken == pdTRUE;
}

/* ------------------------------------------------------------------ *
 * Contention monitor
 *
 * Clause 6.7.2 wants a transmitter to compare the bus with what it is driving
 * and stop on the first difference. The comparison itself is one question -
 * "is the bus active while we intend it passive?" - and j1850_vpw_codec.c
 * works out the instants worth asking it at. What is left here is a timer to
 * ask at those instants and a way to get off the wire immediately when the
 * answer is yes.
 *
 * The deadline is more generous than the clause's wording suggests. Releasing
 * the bus cannot disturb anybody, so the only thing that has to happen before
 * the next bit is that we stop *driving* - and the next active phase is a
 * whole bit time away, 64 us, or some fifteen thousand cycles. What follows
 * needs a small fraction of that.
 * ------------------------------------------------------------------ */

/* The transmit pin has to live in the low GPIO bank for the single register
 * write below to reach it. */
_Static_assert(PIN_J1850_TX_P < 32,
               "J1850 transmit pin must be in GPIO_OUT's first bank");

/** @brief Output routing register for one pin. They are four bytes apart. */
#define GPIO_FUNC_OUT_SEL(pin) (GPIO_FUNC0_OUT_SEL_CFG_REG + 4 * (pin))

/**
 * @brief Get off the wire, now.
 *
 * Two register writes, and it does not touch the RMT at all: the output latch
 * is parked passive and then the pin is taken off the peripheral's signal, so
 * the transmission carries on into a pin that is no longer listening. Stopping
 * the RMT properly needs a call that cannot be made from an interrupt, and it
 * is the *wire* that has a deadline, not the driver's bookkeeping - the task
 * tidies up afterwards in mon_release().
 *
 * The order matters. Writing the latch second would put whatever it happened
 * to be holding onto the bus for as long as it took to reach the second store.
 */
/**
 * @brief Is the bus active? Best of three, a microsecond apart.
 *
 * Clause 6.6 expects a receiver to use "a simple clock-driven digital filter
 * and digital integrator or majority vote sampling circuit", and the receive
 * path has the RMT's own glitch filter doing that job. This one had a single
 * unfiltered read deciding whether to abandon a frame, which makes one noise
 * spike indistinguishable from a node taking the bus - an expensive way to
 * lose a message, and worse than useless during a programming session.
 *
 * Three samples spanning two microseconds cost nothing here: the deadline is
 * a whole bit time away, and every sample is taken a dozen microseconds clear
 * of the nearest edge by construction.
 */
static bool IRAM_ATTR bus_is_active(void) {
    int votes = 0;

    for (int i = 0; i < 3; i++) {
        if (i) {
            esp_rom_delay_us(1);
        }
        votes += gpio_get_level(PIN_J1850_VPW_RX) == J1850_VPW_ACTIVE;
    }

    return votes >= 2;
}

static void IRAM_ATTR mon_abort(void) {
    REG_WRITE(GPIO_OUT_W1TC_REG, BIT(PIN_J1850_TX_P));
    REG_WRITE(GPIO_FUNC_OUT_SEL(PIN_J1850_TX_P), SIG_GPIO_OUT_IDX);

    g.mon.lost = true;
}

/** @brief Give the transmit pin back to the RMT. Task context. */
static void mon_release(void) {
    REG_WRITE(GPIO_FUNC_OUT_SEL(PIN_J1850_TX_P), g.tx_func_sel);
}

/**
 * @brief Arm the alarm for the next check, or report that there are none.
 *
 * Walks forward through the frame's pulses - derived from the bytes, never
 * from an encoded array - until it finds one with a check left in it. A check
 * whose moment has already passed is skipped rather than fired late: a late
 * sample reads a phase it was not asked about, and would report contention
 * that never happened.
 */
static bool IRAM_ATTR mon_schedule(void) {
    for (;;) {
        if (g.mon.next_off < g.mon.n_offs) {
            uint64_t at = g.mon.pulse_start + g.mon.offs[g.mon.next_off++];
            uint64_t now = 0;
            gptimer_alarm_config_t alarm = {.alarm_count = at};

            if (gptimer_get_raw_count(g.mon.timer, &now) == ESP_OK &&
                now >= at) {
                continue; /* missed it */
            }

            return gptimer_set_alarm_action(g.mon.timer, &alarm) == ESP_OK;
        }

        {
            j1850_vpw_pulse_t p;

            if (!j1850_vpw_pulse_at(g.mon.frame, g.mon.len, g.mon.pulse, &p)) {
                return false; /* the frame is done */
            }
            g.mon.pulse_start += p.us;
            g.mon.pulse++;

            if (!j1850_vpw_pulse_at(g.mon.frame, g.mon.len, g.mon.pulse, &p)) {
                return false;
            }
            g.mon.n_offs = (uint8_t)j1850_vpw_watch_offsets(
                &p, TX_WATCH_GUARD_US, TX_WATCH_TAIL_US, g.mon.offs,
                J1850_VPW_MAX_WATCH);
            g.mon.next_off = 0;
            /* Short of the nominal end by the guard: the pulse is that much
             * shorter on the wire, and a sample in the difference reads this
             * node's own next phase. */
            g.mon.pulse_end = g.mon.pulse_start + p.us - TX_WATCH_TAIL_US;
        }
    }
}

static bool watch_alarm(gptimer_handle_t timer,
                        const gptimer_alarm_event_data_t *edata,
                        void *user_ctx);

static bool IRAM_ATTR on_watch_alarm(gptimer_handle_t timer,
                                     const gptimer_alarm_event_data_t *edata,
                                     void *user_ctx) {
    if (!j1850_callback_enter(&g.mon.guard))
        return false;
    bool woken = watch_alarm(timer, edata, user_ctx);
    j1850_callback_exit(&g.mon.guard);
    return woken;
}

static bool IRAM_ATTR watch_alarm(gptimer_handle_t timer,
                                  const gptimer_alarm_event_data_t *edata,
                                  void *user_ctx) {
    uint64_t now = 0;
    bool active;

    (void)timer;
    (void)edata;
    (void)user_ctx;

    if (!g.mon.running) {
        /* The frame this belonged to is over. Aborting now would park a pin
         * that nothing is going to hand back. */
        return false;
    }

    g.stats.tx_arb_checks++;
    g.mon.checks++;

    /*
     * Only believe a sample that is still inside the phase it was asked
     * about. An interrupt held off past the end of this pulse would be
     * reading the next one - which this node drives active - and would report
     * a collision against itself.
     */
    active = bus_is_active();

    if (g.mon.inject_at && g.mon.checks == g.mon.inject_at) {
        g.mon.inject_at = 0; /* spent */
        active = true;
    } else if (gptimer_get_raw_count(g.mon.timer, &now) != ESP_OK ||
               now >= g.mon.pulse_end) {
        g.stats.tx_arb_late++;
        (void)mon_schedule();
        return false;
    }

    if (active) {
        /* We are driving nothing at this instant and the bus is active, so
         * somebody else is. Clause 7.1: their "0" beat our "1". */
        mon_abort();
        g.stats.tx_arb_lost++;
        g.stats.tx_arb_pulse = (uint32_t)g.mon.pulse;
        return false;
    }

    (void)mon_schedule();
    return false;
}

/* Drain callbacks before disarming: an admitted callback may rearm. */
static esp_err_t mon_stop(void) {
    esp_err_t err = j1850_callbacks_stop(&g.mon.guard);
    if (err != ESP_OK)
        return err;
    g.mon.running = false;
    return g.mon.timer ? gptimer_set_alarm_action(g.mon.timer, NULL) : ESP_OK;
}

/**
 * @brief Start watching a frame that is already on its way out.
 *
 * Synchronised on the *falling* edge that ends our own start of frame, which
 * is also the start of the first data bit and so needs no arithmetic to turn
 * into a reference. The rising edge would be the obvious choice and is not a
 * reliable one: rmt_transmit() may have put the frame on the wire before it
 * returned, and syncing part way into a 200 us symbol would skew every check
 * after it. The fall is 200 us further on, which is time enough to be waiting.
 *
 * A transmission that never appears leaves the monitor switched off and is
 * reported by the echo check instead - that is its job, and duplicating it
 * here would only add a second opinion about the same silence.
 */
static void mon_start(const uint8_t *frame, size_t len) {
    int64_t deadline;

    g.mon.lost = false;

    if (!g.mon.enabled || !g.mon.timer || len == 0 ||
        len > J1850_VPW_MAX_FRAME) {
        return;
    }

    memcpy(g.mon.frame, frame, len);
    g.mon.len = (uint8_t)len;

    deadline = esp_timer_get_time() + 2000;
    while (gpio_get_level(PIN_J1850_VPW_RX) != J1850_VPW_ACTIVE) {
        if (esp_timer_get_time() > deadline) {
            return;
        }
    }

    /*
     * Bracket the falling edge rather than merely wait for it. `active_at` is
     * the last moment the bus was known to still be active and `pulse_start`
     * the first it was known to be passive, so the edge lies between them -
     * and the width of that gap is exactly how much this reference can be
     * trusted. Uninterrupted it is a couple of microseconds. Interrupted, it
     * is however long the interrupt took, and the frame is better sent
     * unwatched than watched against a reference that is tens of
     * microseconds out.
     */
    {
        uint64_t active_at = 0;

        if (gptimer_get_raw_count(g.mon.timer, &active_at) != ESP_OK) {
            return;
        }

        for (;;) {
            if (gpio_get_level(PIN_J1850_VPW_RX) == J1850_VPW_PASSIVE) {
                break;
            }
            if (gptimer_get_raw_count(g.mon.timer, &active_at) != ESP_OK ||
                esp_timer_get_time() > deadline) {
                return;
            }
        }

        if (gptimer_get_raw_count(g.mon.timer, &g.mon.pulse_start) != ESP_OK) {
            return;
        }

        if (g.mon.pulse_start - active_at > TX_SYNC_SLACK_US) {
            g.stats.tx_arb_unsynced++;
            return;
        }
    }

    /* The fall ended pulse 0, so the reference above is the start of pulse 1
     * and there is nothing left to watch in the start of frame. */
    g.mon.pulse = 1;
    g.mon.n_offs = 0;
    g.mon.next_off = 0;
    g.mon.checks = 0;
    g.mon.running = true;

    {
        j1850_vpw_pulse_t p;

        if (j1850_vpw_pulse_at(g.mon.frame, g.mon.len, 1, &p)) {
            g.mon.n_offs = (uint8_t)j1850_vpw_watch_offsets(
                &p, TX_WATCH_GUARD_US, TX_WATCH_TAIL_US, g.mon.offs,
                J1850_VPW_MAX_WATCH);
            g.mon.pulse_end = g.mon.pulse_start + p.us - TX_WATCH_TAIL_US;
        }
    }

    j1850_callbacks_start(&g.mon.guard);
    (void)mon_schedule();
}

/* ------------------------------------------------------------------ *
 * Bus access
 * ------------------------------------------------------------------ */

/**
 * @brief Wait for the bus to be passive for a whole inter-frame separation.
 *
 * Clause 5.3.4.4: a transmitter may start once IFS has expired. The shorter
 * alternative the clause allows - an end of frame plus a rising edge - is not
 * used here; a dongle is never the node that has to win a race for the bus.
 *
 * Counted as continuous passive time, which is stricter than the standard's
 * "after the end of the last data bit": that instant is one end of data and
 * one end of frame further back, so this waits if anything too long.
 *
 * @return true when the bus went idle, false if @p timeout_ms passed first.
 */
static bool wait_bus_idle(uint32_t timeout_ms) {
    const int64_t start = esp_timer_get_time();
    const int64_t deadline = start + (int64_t)timeout_ms * 1000;

    for (;;) {
        int64_t now = esp_timer_get_time();
        int64_t quiet_since;

        if (now > deadline) {
            return false;
        }

        if (gpio_get_level(PIN_J1850_VPW_RX) != J1850_VPW_PASSIVE) {
            /* An idle bus answers within an inter-frame separation, so
             * anything past a couple of milliseconds means real traffic and
             * this thread has no business spinning through it. */
            if (now - start > 2000) {
                vTaskDelay(1);
            }
            continue;
        }

        quiet_since = esp_timer_get_time();
        while (esp_timer_get_time() - quiet_since < J1850_VPW_TV6_NOM) {
            if (gpio_get_level(PIN_J1850_VPW_RX) != J1850_VPW_PASSIVE) {
                goto not_idle;
            }
        }

        return true;

    not_idle:
        continue;
    }
}

/**
 * @brief Wait for a frame to be in progress and jump into one of its gaps.
 *
 * The opposite of wait_bus_idle(), and only ever reached from
 * j1850_vpw_force_collision(). Starting inside another node's passive phase
 * puts two transmitters on one bit grid, which is what a real collision is.
 *
 * Spun rather than slept through on purpose: a frame here is a few
 * milliseconds and a scheduler tick is ten, so yielding would step straight
 * over the thing being waited for.
 */
static bool wait_to_barge_in(uint32_t timeout_ms) {
    const int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    int64_t quiet_since;

    /*
     * Wait out a gap first. Without it this lands wherever the bus happened
     * to be when it was called, which is usually most of the way through
     * somebody's frame - and colliding with the last two bits of a message
     * proves very little. A millisecond of silence is several inter-frame
     * separations, so the next rising edge after it is a start of frame.
     */
    for (;;) {
        if (esp_timer_get_time() > deadline) {
            return false;
        }
        if (gpio_get_level(PIN_J1850_VPW_RX) != J1850_VPW_PASSIVE) {
            continue;
        }
        quiet_since = esp_timer_get_time();
        while (esp_timer_get_time() - quiet_since < 1000) {
            if (gpio_get_level(PIN_J1850_VPW_RX) != J1850_VPW_PASSIVE) {
                goto not_quiet;
            }
        }
        break;
    not_quiet:
        continue;
    }

    /* Their start of frame. */
    while (gpio_get_level(PIN_J1850_VPW_RX) != J1850_VPW_ACTIVE) {
        if (esp_timer_get_time() > deadline) {
            return false;
        }
    }
    /* And the end of it, which is the start of their first data bit. */
    while (gpio_get_level(PIN_J1850_VPW_RX) != J1850_VPW_PASSIVE) {
        if (esp_timer_get_time() > deadline) {
            return false;
        }
    }

    return true;
}

static bool IRAM_ATTR on_tx_done(rmt_channel_handle_t channel,
                                 const rmt_tx_done_event_data_t *edata,
                                 void *user_ctx) {
    BaseType_t woken = pdFALSE;

    (void)channel;
    (void)edata;
    (void)user_ctx;

    xSemaphoreGiveFromISR(g.tx_done_sem, &woken);
    return woken == pdTRUE;
}

/**
 * @brief Wait for the frame to finish, or for the monitor to say we lost.
 *
 * @return 0 when the transmission completed, BUS_ERR_ARBITRATION when it was
 *         stopped part way, BUS_ERR_TX_FAILED if neither happened in time.
 */
static int tx_await(void) {
    int64_t deadline = esp_timer_get_time() + TX_COMPLETE_TIMEOUT_MS * 1000;

    for (;;) {
        if (g.mon.lost) {
            return BUS_ERR_ARBITRATION;
        }
        if (xSemaphoreTake(g.tx_done_sem, 1) == pdTRUE) {
            /* A delayed callback may belong to an aborted transfer. */
            int64_t remaining_us = deadline - esp_timer_get_time();
            int wait_ms =
                remaining_us > 0 ? (int)((remaining_us + 999) / 1000) : 0;
            esp_err_t err = rmt_tx_wait_all_done(g.tx_chan, wait_ms);
            if (g.mon.lost)
                return BUS_ERR_ARBITRATION;
            return err == ESP_OK ? 0 : BUS_ERR_TX_FAILED;
        }
        if (esp_timer_get_time() > deadline) {
            ESP_LOGE(TAG, "transmit did not complete");
            return BUS_ERR_TX_FAILED;
        }
    }
}

/* Keep the pin detached until the aborted transfer has been drained. */
static int tx_finish(int rc) {
    esp_err_t err = mon_stop();
    if (err != ESP_OK)
        goto fault;
    if (rc != 0) {
        err = j1850_rmt_recover(g.tx_chan, &g.tx_enabled);
        if (err != ESP_OK)
            goto fault;
    }
    mon_release();
    return rc;

fault:
    g.started = false;
    ESP_LOGE(TAG, "TX recovery failed: %s", esp_err_to_name(err));
    return BUS_ERR_TX_FAILED;
}

/** @brief One transmit attempt: idle wait, hand to the peripheral, wait out. */
static int tx_once(const rmt_symbol_word_t *sym, size_t n, const uint8_t *frame,
                   size_t len) {
    static const rmt_transmit_config_t cfg = {
        .loop_count = 0,
        .flags.eot_level = J1850_VPW_PASSIVE, /* leave the bus released */
    };
    esp_err_t err;
    int rc;

    if (!wait_bus_idle(TX_BUS_IDLE_TIMEOUT_MS)) {
        g.stats.tx_bus_busy++;
        return BUS_ERR_BUS_BUSY;
    }

    /* Whatever happened last time, this pin belongs to the peripheral before
     * a frame starts. Writing the routing it already has is a no-op; writing
     * it after a stray abort is the difference between a working transmitter
     * and a silent one. */
    mon_release();

    err = rmt_transmit(g.tx_chan, g.copy_encoder, sym,
                       n * sizeof(rmt_symbol_word_t), &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "transmit refused: %s", esp_err_to_name(err));
        return BUS_ERR_TX_FAILED;
    }

    mon_start(frame, len);
    rc = tx_await();

    rc = tx_finish(rc);

    if (rc != 0) {
        return rc;
    }

    g.stats.tx_frames++;
    return 0;
}

/**
 * @brief Wait for this node's own frame to come back off the wire.
 *
 * @return true when it did. False means the receiver never saw it, which is
 *         a transceiver that is not driving, or a collision that destroyed
 *         the frame - both of which leave the request unsent.
 */
static bool await_echo(void) {
    if (xSemaphoreTake(g.echo_sem, pdMS_TO_TICKS(TX_ECHO_MS)) == pdTRUE) {
        return true;
    }

    /* Disarm before returning, or a late echo would be matched against the
     * next frame's expectations. */
    if (__atomic_exchange_n(&g.echo_armed, false, __ATOMIC_ACQ_REL)) {
        g.stats.tx_echo_missing++;
    }

    return false;
}

static int j1850_vpw_tx(const bus_msg_t *msg, uint32_t flags) {
    /* Addressing rides in the bytes on this bus, so msg->id is not
     * ours to look at. */
    const uint8_t *data = msg->data;
    size_t len = msg->len;

    uint8_t frame[J1850_VPW_MAX_FRAME];
    bool add_crc = g.crc_tx && !(flags & BUS_TX_NO_CHECKSUM);
    size_t n;
    int rc;

    if (!g.started) {
        return BUS_ERR_NOT_READY;
    }
    if (!data || len == 0) {
        return BUS_ERR_BAD_ARG;
    }
    if (len > (add_crc ? J1850_VPW_MAX_FRAME - 1u : J1850_VPW_MAX_FRAME)) {
        /* One byte of the message budget belongs to the CRC. */
        return BUS_ERR_TOO_LONG;
    }
    if (!add_crc && len < 2) {
        /* The caller said its bytes already end in a CRC, so a single byte is
         * a CRC with no message in front of it. */
        return BUS_ERR_BAD_ARG;
    }

    memcpy(frame, data, len);
    if (add_crc) {
        frame[len] = j1850_crc(data, len);
        len += 1;
    }

    if (xSemaphoreTake(g.tx_lock, portMAX_DELAY) != pdTRUE) {
        return BUS_ERR_NOT_READY;
    }

    /* Re-read under the lock: close() may have taken the bus down while this
     * call was waiting for it, and everything below touches the peripheral. */
    if (!g.started) {
        xSemaphoreGive(g.tx_lock);
        return BUS_ERR_NOT_READY;
    }

    n = j1850_vpw_encode(frame, len, g.tx_symbols, J1850_VPW_MAX_SYMBOLS);
    if (n == 0) {
        xSemaphoreGive(g.tx_lock);
        return BUS_ERR_TOO_LONG;
    }

    for (uint8_t attempt = 0;; attempt++) {
        uint32_t foreign_before = g.rx_foreign;

        /* Arm echo suppression before the frame goes out: the receive ISR
         * can run before rmt_transmit() has even returned. Draining the
         * semaphore first keeps a previous frame's echo from answering for
         * this one. */
        while (xSemaphoreTake(g.echo_sem, 0) == pdTRUE) {
            ;
        }
        while (xSemaphoreTake(g.tx_done_sem, 0) == pdTRUE) {
            ;
        }
        memcpy(g.echo, frame, len);
        g.echo_len = (uint8_t)len;
        __atomic_store_n(&g.echo_armed, true, __ATOMIC_RELEASE);

        rc = tx_once(g.tx_symbols, n, frame, len);
        if (rc != 0) {
            __atomic_store_n(&g.echo_armed, false, __ATOMIC_RELEASE);

            /*
             * Losing the bus is the one failure worth trying again, and the
             * only one this driver can retry without guessing. J2534-1
             * Table 8 puts it under the controller retry strategy - retried
             * automatically, with nothing reported to the application - and
             * clause 6.11.4.1 permits that only to a controller that can
             * detect the loss, which is now this one. The next attempt waits
             * out an inter-frame separation on its way in, so the node that
             * won gets to finish.
             */
            if (rc == BUS_ERR_ARBITRATION && attempt < g.retries) {
                g.stats.tx_retries++;
                continue;
            }
            break;
        }

        (void)await_echo();

        if (attempt >= g.retries) {
            break;
        }

        /* Retry only into total silence. Anything at all on the bus - a
         * module's answering frame, another node's broadcast, even a capture
         * that failed to decode - means the frame was heard, and asking a
         * second time would duplicate the request.
         *
         * Watched rather than slept through. Sleeping the whole window and
         * then asking whether anything arrived charges every exchange the
         * full TX_SILENCE_MS even when the answer was already on the wire -
         * which is most of them, and which would make this call return long
         * after the reply it was waiting for had been timestamped. */
        {
            int64_t deadline = esp_timer_get_time() + TX_SILENCE_MS * 1000;

            while (g.rx_foreign == foreign_before &&
                   esp_timer_get_time() < deadline) {
                vTaskDelay(1);
            }
        }

        if (g.rx_foreign != foreign_before) {
            break;
        }

        g.stats.tx_retries++;
    }

    /* J2534's ECHO_PHYSICAL_CHANNEL_TX. The receiver already sees this
     * node's own transmissions - that is what the echo check is built on - so
     * a loopback copy is a matter of not discarding one rather than of
     * generating one. It is still queued from here rather than from the
     * interrupt, because only here is it known whether the frame finished. */
    if (rc == 0 && g.loopback) {
        struct rx_frame f;

        f.len = (uint8_t)len;
        f.status = BUS_RX_TX_MSG_TYPE;
        f.timestamp_us = (uint32_t)esp_timer_get_time();
        memcpy(f.data, frame, f.len);

        if (xQueueSend(g.rx_queue, &f, 0) != pdTRUE) {
            g.stats.rx_dropped++;
        }
    }

    xSemaphoreGive(g.tx_lock);
    return rc;
}

static int j1850_vpw_rx(bus_msg_t *msg, TickType_t xTicksToWait) {
    struct rx_frame f;

    if (!g.started || !g.rx_queue) {
        return BUS_ERR_NOT_READY;
    }
    if (!msg || !msg->data || msg->cap == 0) {
        return BUS_ERR_BAD_ARG;
    }

    if (xQueueReceive(g.rx_queue, &f, xTicksToWait) != pdPASS) {
        return BUS_ERR_TIMEOUT;
    }

    if (f.len > msg->cap) {
        return BUS_ERR_NO_SPACE;
    }

    memcpy(msg->data, f.data, f.len);
    msg->len = f.len;
    msg->status = f.status;
    msg->timestamp_us = f.timestamp_us;
    return f.len;
}

void j1850_vpw_inject_arbitration_loss(uint32_t nth) { g.mon.inject_at = nth; }

int j1850_vpw_force_collision(const uint8_t *data, size_t len) {
    uint8_t frame[J1850_VPW_MAX_FRAME];
    size_t n;
    int rc;

    if (!g.started) {
        return BUS_ERR_NOT_READY;
    }
    if (!data || len == 0 || len > J1850_VPW_MAX_FRAME - 1u) {
        return BUS_ERR_BAD_ARG;
    }

    memcpy(frame, data, len);
    frame[len] = j1850_crc(data, len);
    len += 1;

    if (xSemaphoreTake(g.tx_lock, portMAX_DELAY) != pdTRUE) {
        return BUS_ERR_NOT_READY;
    }
    if (!g.started) {
        xSemaphoreGive(g.tx_lock);
        return BUS_ERR_NOT_READY;
    }

    n = j1850_vpw_encode(frame, len, g.tx_symbols, J1850_VPW_MAX_SYMBOLS);
    if (n == 0) {
        xSemaphoreGive(g.tx_lock);
        return BUS_ERR_TOO_LONG;
    }

    while (xSemaphoreTake(g.tx_done_sem, 0) == pdTRUE) {
        ;
    }

    if (!wait_to_barge_in(1500)) {
        xSemaphoreGive(g.tx_lock);
        return BUS_ERR_TIMEOUT; /* nothing was talking to collide with */
    }

    /* Straight past wait_bus_idle(), which is the whole point. */
    {
        static const rmt_transmit_config_t cfg = {
            .loop_count = 0,
            .flags.eot_level = J1850_VPW_PASSIVE,
        };
        if (rmt_transmit(g.tx_chan, g.copy_encoder, g.tx_symbols,
                         n * sizeof(rmt_symbol_word_t), &cfg) != ESP_OK) {
            xSemaphoreGive(g.tx_lock);
            return BUS_ERR_TX_FAILED;
        }

        mon_start(frame, len);
        rc = tx_await();
        rc = tx_finish(rc);

        if (rc == 0) {
            g.stats.tx_frames++;
        }
    }

    xSemaphoreGive(g.tx_lock);
    return rc;
}

/* ------------------------------------------------------------------ *
 * Lifecycle
 * ------------------------------------------------------------------ */

static esp_err_t j1850_vpw_close(void);

static esp_err_t j1850_vpw_open(const bus_cfg_t *cfg) {
    /* Fixed 10.4 kbit/s. */
    (void)cfg;

    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = PIN_J1850_TX_P,
        .clk_src = RMT_CLK_SRC,
        .resolution_hz = RMT_RESOLUTION_HZ,
        /* A maximum frame is 49 symbols, so one block holds a whole one and
         * the driver never has to refill mid-frame. */
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = PIN_J1850_VPW_RX,
        .clk_src = RMT_CLK_SRC,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = RX_MEM_SYMBOLS,
    };
    const rmt_rx_event_callbacks_t rx_cbs = {.on_recv_done = on_rx_done};
    rmt_copy_encoder_config_t copy_cfg = {};

    if (g.started) {
        ESP_LOGI(TAG, "already up");
        return ESP_OK;
    }

    if (g.initialized)
        return ESP_ERR_INVALID_STATE;

    memset(&g, 0, sizeof(g));
    g.initialized = true;
    g.mon.guard.stopping = true;
    g.retries = TX_DEFAULT_RETRIES;
    g.crc_tx = true;
    g.crc_rx = true;
    g.loopback = false;
    /* On by default: it is a correctness fix, and a switch that defaults off
     * is a switch nothing ever exercises. bus.h, BUS_P_ARBITRATION. */
    g.mon.enabled = true;
    /* Zero: deliver every frame. Collapsing retransmissions is a client's
     * choice, made through BUS_P_DUPLICATE_MS - see j1850_vpw.h. */
    g.dup_window_us = 0;

    /* The transceiver is dual mode; 0 selects VPW and its 7 V signalling. */
    gpio_set_level(PIN_J1850_MODE, 0);

    g.rx_queue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(struct rx_frame));
    g.tx_lock = xSemaphoreCreateMutex();
    g.echo_sem = xSemaphoreCreateBinary();
    g.tx_done_sem = xSemaphoreCreateBinary();
    g.rx_buf = heap_caps_aligned_calloc(4, RX_BUFFER_SYMBOLS,
                                        sizeof(rmt_symbol_word_t),
                                        MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!g.rx_queue || !g.tx_lock || !g.echo_sem || !g.tx_done_sem ||
        !g.rx_buf) {
        ESP_LOGE(TAG, "out of memory bringing the bus up");
        goto fail;
    }

    /* Checked, not ESP_ERROR_CHECK'd: a bus that cannot be opened must refuse
     * and say why, not abort the firmware. See j1850_pwm.c for what walking
     * into that costs. */
    {
        const rmt_tx_event_callbacks_t tx_cbs = {.on_trans_done = on_tx_done};

        if (rmt_new_tx_channel(&tx_cfg, &g.tx_chan) != ESP_OK ||
            rmt_new_copy_encoder(&copy_cfg, &g.copy_encoder) != ESP_OK ||
            rmt_tx_register_event_callbacks(g.tx_chan, &tx_cbs, NULL) !=
                ESP_OK ||
            j1850_rmt_enable(g.tx_chan, &g.tx_enabled) != ESP_OK) {
            ESP_LOGE(TAG, "could not bring the transmitter up");
            goto fail;
        }
    }

    /*
     * VPW is a single wire bus: only BUS+ is driven, and the low side takes
     * no part in it. It still has to be parked, because the PWM driver routes
     * it to an RMT channel and closing that driver leaves the pin holding
     * whatever the matrix last put there. The latch is written before the
     * routing is changed, so the pin never presents an old level.
     */
    gpio_set_level(PIN_J1850_TX_N, J1850_VPW_PASSIVE);
    gpio_set_direction(PIN_J1850_TX_N, GPIO_MODE_OUTPUT);

    /*
     * The routing the RMT driver just gave the high side, read back off the
     * pin rather than assumed: which signal it is depends on the channel that
     * was allocated. The contention monitor swaps the pin to plain GPIO to
     * get off the wire and puts this back afterwards, and the latch has to be
     * enabled as an output for that to drive anything.
     */
    REG_WRITE(GPIO_ENABLE_W1TS_REG, BIT(PIN_J1850_TX_P));
    g.tx_func_sel = REG_READ(GPIO_FUNC_OUT_SEL(PIN_J1850_TX_P));

    {
        /* Free running at 1 us, the unit every J1850 timing is written in.
         * Only the alarm is armed and disarmed; the count never stops, so a
         * check scheduled across a frame boundary cannot land on a counter
         * that has been reset underneath it. */
        gptimer_config_t tcfg = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000,
        };
        const gptimer_event_callbacks_t tcbs = {.on_alarm = on_watch_alarm};

        /* Refuse startup if arbitration monitoring cannot be initialized. */
        esp_err_t terr = gptimer_new_timer(&tcfg, &g.mon.timer);

        if (terr == ESP_OK) {
            terr = gptimer_register_event_callbacks(g.mon.timer, &tcbs, NULL);
        }
        if (terr == ESP_OK) {
            terr = j1850_timer_enable(g.mon.timer, &g.mon.timer_enabled);
        }
        if (terr == ESP_OK) {
            terr = j1850_timer_start(g.mon.timer, &g.mon.timer_running);
        }
        if (terr != ESP_OK) {
            ESP_LOGE(TAG, "contention monitor failed: %s",
                     esp_err_to_name(terr));
            goto fail;
        }
    }

    if (rmt_new_rx_channel(&rx_cfg, &g.rx_chan) != ESP_OK ||
        rmt_rx_register_event_callbacks(g.rx_chan, &rx_cbs, NULL) != ESP_OK ||
        j1850_rmt_enable(g.rx_chan, &g.rx_enabled) != ESP_OK ||
        rmt_receive(g.rx_chan, g.rx_buf,
                    RX_BUFFER_SYMBOLS * sizeof(rmt_symbol_word_t),
                    &g_rx_config) != ESP_OK) {
        ESP_LOGE(TAG, "could not bring the receiver up");
        goto fail;
    }

    g.started = true;
    ESP_LOGI(TAG,
             "up: 10.4 kbps, %u us arming threshold, %u us glitch filter, "
             "arbitration %s",
             (unsigned)J1850_VPW_RX_IDLE_US,
             (unsigned)(J1850_VPW_RX_FILTER_NS / 1000),
             g.mon.enabled ? "watched" : "off");
    return ESP_OK;

fail:
    j1850_vpw_close();
    return ESP_ERR_INVALID_STATE;
}

static esp_err_t j1850_vpw_close(void) {
    esp_err_t err;
    if (!g.initialized)
        return ESP_OK;
    if (g.tx_lock)
        xSemaphoreTake(g.tx_lock, portMAX_DELAY);
    g.started = false;

    err = j1850_callbacks_stop(&g.rx_guard);
    if (err != ESP_OK)
        goto out;
    err = mon_stop();
    if (err != ESP_OK)
        goto out;
    err = j1850_timer_delete(&g.mon.timer, &g.mon.timer_enabled,
                             &g.mon.timer_running);
    if (err != ESP_OK)
        goto out;
    err = j1850_rmt_delete(&g.rx_chan, &g.rx_enabled);
    if (err != ESP_OK)
        goto out;
    err = j1850_rmt_delete(&g.tx_chan, &g.tx_enabled);
    if (err != ESP_OK)
        goto out;
    if (g.copy_encoder) {
        err = rmt_del_encoder(g.copy_encoder);
        if (err != ESP_OK)
            goto out;
        g.copy_encoder = NULL;
    }

    gpio_set_level(PIN_J1850_TX_P, J1850_VPW_PASSIVE);
    gpio_set_direction(PIN_J1850_TX_P, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_J1850_TX_N, J1850_VPW_PASSIVE);
    gpio_set_direction(PIN_J1850_TX_N, GPIO_MODE_OUTPUT);
    free(g.rx_buf);
    g.rx_buf = NULL;
    if (g.rx_queue) {
        vQueueDelete(g.rx_queue);
        g.rx_queue = NULL;
    }
    if (g.echo_sem) {
        vSemaphoreDelete(g.echo_sem);
        g.echo_sem = NULL;
    }
    if (g.tx_done_sem) {
        vSemaphoreDelete(g.tx_done_sem);
        g.tx_done_sem = NULL;
    }
    if (g.tx_lock) {
        xSemaphoreGive(g.tx_lock);
        vSemaphoreDelete(g.tx_lock);
        g.tx_lock = NULL;
    }
    g.initialized = false;
    ESP_LOGI(TAG, "down");
    return ESP_OK;

out:
    if (g.tx_lock)
        xSemaphoreGive(g.tx_lock);
    ESP_LOGE(TAG, "close incomplete: %s", esp_err_to_name(err));
    return err;
}

/* ------------------------------------------------------------------ *
 * Configuration
 * ------------------------------------------------------------------ */

/**
 * @brief The parameters of bus.h that mean something on this bus.
 *
 * Fewer than K-Line has, because almost all of ISO 14230's timing is
 * negotiated where J1850's is fixed by the symbol coding, and one fewer than
 * PWM has, because this driver does not answer frames.
 */
static int j1850_vpw_set_param(bus_param_t p, uint32_t value) {
    if (!g.started) {
        return BUS_ERR_NOT_READY;
    }

    switch (p) {
    case BUS_P_DATA_RATE:
        /* GM's 4X mode runs these symbols at 41.6 kbps. It is not in SAE
         * J1850 and none of its timings are built here. */
        return value == 10400 ? 0 : BUS_ERR_BAD_ARG;

    case BUS_P_NETWORK_LINE:
        /* VPW is a single wire bus, so there is no line to select. J2534
         * calls the ability optional. */
        return value == BUS_NORMAL ? 0 : BUS_ERR_UNSUPPORTED;

    case BUS_P_IFR_ENABLED:
        /* Refused rather than accepted and ignored: a client that asked to
         * acknowledge frames and was told yes would wait for retransmissions
         * that never stop. j1850_vpw_stats_t::rx_ifr has the reason. */
        return value ? BUS_ERR_UNSUPPORTED : 0;

    case BUS_P_TX_RETRIES:
        if (value > 8) {
            return BUS_ERR_BAD_ARG;
        }
        g.retries = (uint8_t)value;
        return 0;

    case BUS_P_DUPLICATE_MS:
        g.dup_window_us = value * 1000u;
        return 0;

    case BUS_P_CHECKSUM_TX:
        g.crc_tx = value != 0;
        return 0;

    case BUS_P_CHECKSUM_RX:
        g.crc_rx = value != 0;
        return 0;

    case BUS_P_LOOPBACK:
        g.loopback = value != 0;
        return 0;

    case BUS_P_ARBITRATION:
        g.mon.enabled = value != 0;
        return 0;

    default:
        return BUS_ERR_UNSUPPORTED;
    }
}

static int j1850_vpw_get_param(bus_param_t p, uint32_t *out) {
    if (!out) {
        return BUS_ERR_BAD_ARG;
    }
    if (!g.started) {
        return BUS_ERR_NOT_READY;
    }

    switch (p) {
    case BUS_P_DATA_RATE:
        *out = 10400;
        return 0;
    case BUS_P_NETWORK_LINE:
        *out = BUS_NORMAL;
        return 0;
    case BUS_P_IFR_ENABLED:
        *out = 0;
        return 0;
    case BUS_P_TX_RETRIES:
        *out = g.retries;
        return 0;
    case BUS_P_DUPLICATE_MS:
        *out = g.dup_window_us / 1000u;
        return 0;
    case BUS_P_CHECKSUM_TX:
        *out = g.crc_tx;
        return 0;
    case BUS_P_CHECKSUM_RX:
        *out = g.crc_rx;
        return 0;
    case BUS_P_LOOPBACK:
        *out = g.loopback;
        return 0;
    case BUS_P_ARBITRATION:
        *out = g.mon.enabled;
        return 0;
    default:
        return BUS_ERR_UNSUPPORTED;
    }
}

static int j1850_vpw_ioctl(bus_ioctl_t id, const void *in, void *out) {
    (void)in;
    (void)out;

    if (!g.started) {
        return BUS_ERR_NOT_READY;
    }

    switch (id) {
    case BUS_IOCTL_CLEAR_RX_QUEUE:
        if (g.rx_queue) {
            xQueueReset(g.rx_queue);
        }
        return 0;

    case BUS_IOCTL_CLEAR_TX_QUEUE:
        /* Transmissions are synchronous: j1850_vpw_tx() does not return until
         * the frame is on the wire, so there is never a queue to clear.
         * J2534 requires success when the queue is already empty. */
        return 0;

    default:
        /* Including the functional address table, which exists to decide
         * which frames this node acknowledges - and it acknowledges none. */
        return BUS_ERR_UNSUPPORTED;
    }
}

/* ------------------------------------------------------------------ *
 * Diagnostics
 * ------------------------------------------------------------------ */

/**
 * @brief The shared counters, filled from this driver's own.
 *
 * Several of J1850's failure modes have no shared name and are not given one:
 * a symbol that matches no pulse width means nothing on a UART bus, and
 * folding it into rx_frame_err would make a counter that reads the same and
 * means something different depending on which bus you asked. Those stay in
 * j1850_vpw_get_native_stats(), which the shell and the target suite use.
 */
static void j1850_vpw_bus_stats(bus_stats_t *out) {
    if (!out) {
        return;
    }

    memset(out, 0, sizeof(*out));
    out->rx_msgs = g.stats.rx_frames;
    out->rx_bad_checksum = g.stats.rx_bad_crc;
    out->rx_short = g.stats.rx_short;
    out->rx_too_long = g.stats.rx_too_long;
    out->rx_dropped = g.stats.rx_dropped;
    out->rx_break = g.stats.rx_break;
    /* Every way a capture can fail to be a frame, which is what a UART bus
     * would call a framing error. */
    out->rx_frame_err = g.stats.rx_framing + g.stats.rx_bad_symbol +
                        g.stats.rx_no_sof + g.stats.rx_empty;
    out->rx_duplicate = g.stats.rx_duplicate;
    out->tx_msgs = g.stats.tx_frames;
    out->tx_echo_ok = g.stats.tx_echo_ok;
    out->tx_echo_missing = g.stats.tx_echo_missing;
    out->tx_retries = g.stats.tx_retries;
    out->tx_bus_busy = g.stats.tx_bus_busy;
    /* A frame that lost the bus did go out and did come back wrong - as far
     * as this node's own receiver is concerned it is a collision, which is
     * exactly what tx_echo_bad names. The reason it lost is in
     * j1850_vpw_get_native_stats(). */
    out->tx_echo_bad = g.stats.tx_arb_lost;
}

static void j1850_vpw_bus_reset_stats(void) {
    memset(&g.stats, 0, sizeof(g.stats));
}

void j1850_vpw_get_native_stats(j1850_vpw_stats_t *out) {
    if (out) {
        *out = g.stats;
    }
}

size_t j1850_vpw_get_capture(j1850_vpw_capture_sel_t which,
                             rmt_symbol_word_t *out, size_t cap,
                             j1850_vpw_rx_status_t *status) {
    bool tx = (which == J1850_VPW_CAP_TX_ECHO);
    const rmt_symbol_word_t *src = tx ? g.tx_cap : g.cap;
    size_t n = tx ? g.tx_cap_len : g.cap_len;

    if (status) {
        *status = (j1850_vpw_rx_status_t)(tx ? g.tx_cap_status : g.cap_status);
    }
    if (!out || cap == 0) {
        return 0;
    }
    if (n > cap) {
        n = cap;
    }

    memcpy(out, src, n * sizeof(rmt_symbol_word_t));
    return n;
}

const bus_ops_t j1850_vpw_bus_ops = {
    .name = "J1850 VPW",
    .open = j1850_vpw_open,
    .close = j1850_vpw_close,
    .send = j1850_vpw_tx,
    .recv = j1850_vpw_rx,
    .set_param = j1850_vpw_set_param,
    .get_param = j1850_vpw_get_param,
    .ioctl = j1850_vpw_ioctl,
    .get_stats = j1850_vpw_bus_stats,
    .reset_stats = j1850_vpw_bus_reset_stats,
};
