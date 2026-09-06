/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file j1850_pwm.c
 * @brief SAE J1850 PWM bus driver: the RMT peripheral, the queue, the policy.
 *
 * All the protocol knowledge is next door in j1850_pwm_codec.c, which has no
 * hardware in it and is exercised on the host. What is left here is the part
 * that can only be judged on a real bus, and three of its decisions were
 * settled by measurement rather than by reading the standard.
 *
 * Receiving
 * ---------
 * The RMT captures pulse widths at 1 us and closes a capture once the input
 * has held one level for J1850_PWM_RX_IDLE_US. "One level", not "idle": the
 * peripheral counts an active phase the same way it counts a passive one, so
 * the threshold has to clear the 31 us a start of frame occupies as well as
 * the 23 us worst case gap between two bits. Set below that, frames arrive
 * with their heads cut off - which is exactly what happened at 30 us.
 *
 * Answering
 * ---------
 * A reply with K clear requires an IFR starting Tp4 after the last data
 * rising edge: 47..49 us to transmit, 42..54 us to receive (Table 3).
 * Tp5 is EOF detection, not an extension of that response window.
 *
 * An IRAM GPIO handler on core 1 validates pulses and CRC as they arrive.
 * At a valid byte boundary it checks for EOD and sends the IFR against the
 * last rising-edge timestamp. RMT independently captures the frame and IFR
 * for delivery and diagnostics; its completion interrupt does not drive ACK.
 *
 * Transmitting
 * ------------
 * The receiver is left armed during transmission rather than muted, so this
 * node reads its own frames back off the wire. That is not a side effect to
 * be tolerated, it is the transmit check: a frame that comes back intact
 * confirms the driver stage, and one that never appears says the transceiver
 * is not driving, which is otherwise indistinguishable from a quiet vehicle.
 * Echoes are filtered out before the receive queue, so callers never see them.
 */

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/soc.h"

#include "bus.h"
#include "j1850_lifecycle.h"
#include "j1850_pwm.h"
#include "pinout.h"

#define TAG "J1850_PWM"

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
 * temperature, and a few percent of error on a 7 us pulse walks Tp1 out of a
 * window that is only +-1 us wide. APB is crystal derived, so 80 MHz divided
 * by 80 is exactly 1 MHz and the error is the crystal's few parts per million.
 * It is also the current default, which is the point - this pins it.
 */
#define RMT_CLK_SRC RMT_CLK_SRC_APB

/* Capture the entire IFR: its longest legal passive gap is Tp4(max) minus
 * Tp1(min), 50 us. GPIO edge timing handles acknowledgment independently. */
#define J1850_PWM_RX_IDLE_US 55

/**
 * @brief Glitch filter. Clause 6.6 expects a receiver to have one.
 *
 * Under Tp1(min) of 4 us so a legitimately short "1" survives, and generous
 * enough to swallow the ringing a 40 metre network produces on an edge.
 */
#define J1850_PWM_RX_FILTER_NS 1500

/* Compensate core-1 level-3 dispatch and output delay; center captured EOD
 * at 48 us on this 240 MHz board, including the observed 1 us jitter. */
#define J1850_PWM_GPIO_LATENCY_US 3

/** Symbols the receive buffer holds. The staging copy has to match it. */
#define RX_BUFFER_SYMBOLS J1850_PWM_CAPTURE_MAX

/**
 * @brief RMT memory the receive channel is given, in symbols.
 *
 * Two of the peripheral's blocks. The driver copies each half out as it
 * fills, so this only sets how often that happens during a frame, not how
 * long a frame may be.
 *
 * Deliberately *not* DMA. Re-arming a DMA receive means a cache writeback
 * over the whole buffer, and the measured cost of that put the receiver back
 * on the air too late to catch the start of a module's retransmission - the
 * frame arrived headless, one bit into its first byte. Programmed I/O re-arms
 * in a fraction of the time, and a J1850 frame is far too small to need DMA.
 */
#define RX_MEM_SYMBOLS (SOC_RMT_MEM_WORDS_PER_CHANNEL * 2)

/** Frames buffered for the caller. Deep enough for a burst of module replies.
 */
#define RX_QUEUE_DEPTH 8

/** How long j1850_pwm_send() waits for an idle bus before giving up. */
#define TX_BUS_IDLE_TIMEOUT_MS 100

/**
 * @brief How long a retry waits to be convinced the bus really is silent.
 *
 * Longer than any legal response latency: a module's in-frame response is due
 * within Tp5 and its answering frame within a few milliseconds.
 */
#define TX_SILENCE_MS 20

/** Extra attempts when nothing at all answers. */
#define TX_DEFAULT_RETRIES 2

/**
 * @brief Interrupt priority for both RMT channels. Zero lets the driver pick.
 *
 * It has to be the same number in both channel configurations, because the
 * priority belongs to the RMT *group* and not to a channel: whichever channel
 * is created first fixes it, and a second channel asking for anything else is
 * refused with ESP_ERR_INVALID_ARG and the message "intr_priority conflict".
 * Setting it on the receiver alone - which is the one with a deadline - looks
 * reasonable and aborts the firmware on the next bus open.
 *
 * Left at zero. Raising it was tried against the headless-capture problem
 * described above and does not address it: the delay that matters is this
 * handler waiting for a previous instance of *itself* to finish, and two
 * interrupts at the same priority serialise whatever that priority is.
 */
#define J1850_PWM_INTR_PRIORITY 0

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
    uint8_t data[J1850_PWM_MAX_FRAME];
};

static struct {
    bool started;
    bool initialized;
    bool tx_enabled;
    bool rx_enabled;
    j1850_callback_guard_t rx_guard;
    j1850_callback_guard_t edge_guard;
    bool edge_installed;
    j1850_pwm_stream_t stream;
    uint32_t edge_cycle;
    uint32_t edge_gap_cycles;
    bool edge_active;
    /* Policy changes invalidate a frame already being decoded. */
    uint32_t ifr_generation;
    uint32_t stream_generation;
    rmt_symbol_word_t tx_symbols[J1850_PWM_MAX_SYMBOLS];
    rmt_channel_handle_t tx_chan;
    rmt_channel_handle_t rx_chan;
    rmt_encoder_handle_t copy_encoder;
    rmt_symbol_word_t *rx_buf; /**< DMA capable, RX_BUFFER_SYMBOLS. */
    QueueHandle_t rx_queue;
    SemaphoreHandle_t tx_lock;

    /** Cycles per microsecond, read once: the in-ISR response is clocked
     *  from the CPU counter and must not call into the clock tree. */
    uint32_t cpu_ticks_per_us;

    /**
     * The GPIO matrix routing the RMT driver set up for the two transmit
     * pins, saved verbatim so the bit banged response can take the pins over
     * and hand them back without having to know which RMT channel was
     * allocated or how the rest of the register is laid out.
     */
    uint32_t tx_func_sel[2];

    j1850_pwm_ifr_cfg_t ifr;
    uint8_t retries;

    /**
     * Policy the front-end chooses; see bus.h. The CRC flags exist
     * because J2534's CHECKSUM_DISABLED lets an application drive its own
     * error detection, and loopback because ECHO_PHYSICAL_CHANNEL_TX lets it
     * see its own transmissions in the receive stream.
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
    uint8_t echo[J1850_PWM_MAX_FRAME];
    uint8_t echo_len;

    /**
     * A BRK was seen and the longer resynchronisation period it calls for has
     * not been served yet. Set by the receive interrupt, cleared by whoever
     * waits it out.
     */
    volatile bool break_pending;

    /* Retransmission collapsing. Only the receive interrupt touches these. */
    uint32_t dup_window_cycles;
    uint8_t last_frame[J1850_PWM_MAX_FRAME];
    uint8_t last_frame_len;
    uint32_t last_frame_cycle;

    /** Bumped for every capture, whatever it decoded to. A transmit that
     *  leaves this unchanged is one nothing on the bus reacted to. */
    volatile uint32_t rx_events;

    j1850_pwm_stats_t stats;

    /* Last capture, for the shell's calibration dump. */
    rmt_symbol_word_t cap[J1850_PWM_CAPTURE_MAX];
    volatile uint16_t cap_len;
    volatile uint8_t cap_status;

    /* And the last one that turned out to be our own transmission, kept so
     * the driver stage can be measured whatever else the bus is doing. */
    rmt_symbol_word_t tx_cap[J1850_PWM_CAPTURE_MAX];
    volatile uint16_t tx_cap_len;
    volatile uint8_t tx_cap_status;
} g;

static const rmt_receive_config_t g_rx_config = {
    .signal_range_min_ns = J1850_PWM_RX_FILTER_NS,
    .signal_range_max_ns = J1850_PWM_RX_IDLE_US * 1000,
};

/* ------------------------------------------------------------------ *
 * In-frame response
 *
 * Bit banged, from the GPIO edge ISR, against the CPU cycle counter. Everything
 * it touches is in IRAM. No allocation or blocking calls; the waveform runs
 * inside an ISR critical section on a microsecond deadline.
 * ------------------------------------------------------------------ */

static portMUX_TYPE g_ifr_spinlock = portMUX_INITIALIZER_UNLOCKED;

/* Both transmit pins have to live in the low GPIO bank for the single
 * register write below to reach them. */
_Static_assert(PIN_J1850_TX_P < 32 && PIN_J1850_TX_N < 32,
               "J1850 transmit pins must be in GPIO_OUT's first bank");

#define TX_PIN_MASK (BIT(PIN_J1850_TX_P) | BIT(PIN_J1850_TX_N))

/** @brief Output routing register for one pin. They are four bytes apart. */
#define GPIO_FUNC_OUT_SEL(pin) (GPIO_FUNC0_OUT_SEL_CFG_REG + 4 * (pin))

/** @brief Both bus drivers, in one register write each. */
static inline void IRAM_ATTR bus_drive(int level) {
    /* TX_P and TX_N are the two halves of the differential pair. The board
     * inverts the low side in hardware, so both pins take the same level. */
    REG_WRITE(level ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, TX_PIN_MASK);
}

/** @brief Spins until the cycle counter reaches @p deadline. */
static inline void IRAM_ATTR wait_until(uint32_t deadline) {
    /* Signed comparison so the counter wrapping mid-wait - it is 32 bits, so
     * every half minute or so - reads as "already past" rather than "another
     * half minute to go". */
    while ((int32_t)(deadline - esp_cpu_get_cycle_count()) > 0) {
        ;
    }
}

/**
 * @brief Drive one in-frame response byte, starting at @p start_cycle.
 *
 * A type 1 IFR (clause 5.3.7 b): one byte, no SOF, no CRC. Bits are MSB first
 * like any J1850 byte, each one an active phase inside a Tp3 cell, and every
 * edge is placed against an absolute deadline rather than by accumulating
 * delays, so a cell that starts late does not push the ones after it.
 */
static bool IRAM_ATTR ifr_transmit(uint8_t byte, uint32_t start_cycle,
                                   uint32_t reference_cycle) {
    const uint32_t per_us = g.cpu_ticks_per_us;
    const uint32_t cell = J1850_PWM_TP3_NOM * per_us;
    uint32_t rise = start_cycle;

    if ((int32_t)(esp_cpu_get_cycle_count() - start_cycle) > 0) {
        g.stats.ifr_late++;
        return false;
    }

    /* Park the output latch passive *before* taking the pins off the RMT
     * matrix, or whatever the latch happened to hold becomes a pulse on the
     * bus the moment the routing changes. */
    bus_drive(0);
    REG_WRITE(GPIO_FUNC_OUT_SEL(PIN_J1850_TX_P), SIG_GPIO_OUT_IDX);
    REG_WRITE(GPIO_FUNC_OUT_SEL(PIN_J1850_TX_N), SIG_GPIO_OUT_IDX);

    bool sent = true;
    if ((int32_t)(esp_cpu_get_cycle_count() - start_cycle) > 0) {
        g.stats.ifr_late++;
        sent = false;
        goto restore;
    }
    while ((int32_t)(start_cycle - esp_cpu_get_cycle_count()) > 0) {
        if (REG_READ(GPIO_IN_REG) & BIT(PIN_J1850_PWM_RX)) {
            g.stats.ifr_lost++;
            sent = false;
            goto restore;
        }
    }
    for (int bit = 7; bit >= 0; bit--) {
        uint32_t active =
            ((byte >> bit) & 1u) ? J1850_PWM_TP1_NOM : J1850_PWM_TP2_NOM;

        wait_until(rise);
        bus_drive(1);
        if (bit == 7) {
            /* Relative to the GPIO edge ISR timestamp, not RMT completion.
             * A captured EOD gap also includes GPIO interrupt latency. */
            g.stats.ifr_start_us =
                (esp_cpu_get_cycle_count() - reference_cycle) /
                g.cpu_ticks_per_us;
        }
        wait_until(rise + active * per_us);
        bus_drive(0);

        /* A dominant zero beats our recessive one. Relinquish the bus. */
        if ((byte >> bit) & 1u) {
            wait_until(rise + J1850_PWM_BIT_SPLIT * per_us);
            if (REG_READ(GPIO_IN_REG) & BIT(PIN_J1850_PWM_RX)) {
                sent = false;
                g.stats.ifr_lost++;
                break;
            }
        }

        rise += cell;
    }

    /* Hold passive to the end of the last cell before handing the pins back,
     * so the trailing bit gets its full width. */
    wait_until(rise);

restore:
    REG_WRITE(GPIO_FUNC_OUT_SEL(PIN_J1850_TX_P), g.tx_func_sel[0]);
    REG_WRITE(GPIO_FUNC_OUT_SEL(PIN_J1850_TX_N), g.tx_func_sel[1]);
    return sent;
}

/* ------------------------------------------------------------------ *
 * Receive
 * ------------------------------------------------------------------ */

static void IRAM_ATTR count_status(j1850_pwm_rx_status_t st) {
    switch (st) {
    case J1850_PWM_RX_OK:
        g.stats.rx_frames++;
        break;
    case J1850_PWM_RX_IFR_ONLY:
        g.stats.rx_ifr++;
        break;
    case J1850_PWM_RX_BREAK:
        g.stats.rx_break++;
        /* Clause 6.6.1.6: a break resets every node to ready-to-receive, and
         * the bus needs Tp9 rather than an ordinary IFS to resynchronise
         * before anyone transmits again. */
        g.break_pending = true;
        break;
    case J1850_PWM_RX_BAD_CRC:
        g.stats.rx_bad_crc++;
        break;
    case J1850_PWM_RX_FRAMING:
        g.stats.rx_framing++;
        break;
    case J1850_PWM_RX_BAD_SYMBOL:
        g.stats.rx_bad_symbol++;
        break;
    case J1850_PWM_RX_BAD_TIMING:
        g.stats.rx_bad_timing++;
        break;
    case J1850_PWM_RX_NO_SOF:
        g.stats.rx_no_sof++;
        break;
    case J1850_PWM_RX_TOO_LONG:
        g.stats.rx_too_long++;
        break;
    case J1850_PWM_RX_SHORT:
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

static void IRAM_ATTR pwm_edge_isr(void *arg) {
    uint32_t now = esp_cpu_get_cycle_count();
    bool active = (REG_READ(GPIO_IN_REG) & BIT(PIN_J1850_PWM_RX)) != 0;
    (void)arg;
    if (!j1850_callback_enter(&g.edge_guard))
        return;
    uint32_t generation = __atomic_load_n(&g.ifr_generation, __ATOMIC_ACQUIRE);
    if (generation != g.stream_generation) {
        g.stream.valid = false;
        g.edge_active = false;
        g.stream_generation = generation;
    }
    if (!__atomic_load_n(&g.ifr.enabled, __ATOMIC_ACQUIRE))
        goto out;
    if (active) {
        g.edge_gap_cycles = now - g.edge_cycle;
        g.edge_cycle = now;
        g.edge_active = true;
        goto out;
    }
    if (!g.edge_active)
        goto out;
    g.edge_active = false;
    uint32_t per_us = g.cpu_ticks_per_us;
    uint32_t active_us = (now - g.edge_cycle + per_us / 2) / per_us;
    uint32_t gap_us = (g.edge_gap_cycles + per_us / 2) / per_us;
    if (active_us >= J1850_PWM_TP7_RX_MIN && active_us <= J1850_PWM_TP7_RX_MAX)
        g.stats.ifr_sof++;
    else if (g.stream.valid) {
        if (active_us < J1850_PWM_TP1_RX_MIN ||
            active_us > J1850_PWM_TP2_RX_MAX)
            g.stats.ifr_bad_pulse++;
        bool eod = !g.stream.first && !g.stream.bits && g.stream.len >= 4 &&
                   gap_us >= J1850_PWM_TP4_RX_MIN &&
                   gap_us <= J1850_PWM_TP4_RX_MAX;
        if (!eod && (gap_us < (g.stream.first ? J1850_PWM_TP4_RX_MIN
                                              : J1850_PWM_TP3_RX_MIN) ||
                     gap_us > (g.stream.first ? J1850_PWM_TP4_RX_MAX
                                              : J1850_PWM_TP3_RX_MAX)))
            g.stats.ifr_bad_gap++;
    }
    if (!j1850_pwm_stream_pulse(&g.stream, active_us, gap_us))
        goto out;

    /* A CRC-valid prefix is not necessarily the end of a message. Watch
     * for the next data edge before committing to the EOD response. */
    portENTER_CRITICAL_ISR(&g_ifr_spinlock);
    if (generation != __atomic_load_n(&g.ifr_generation, __ATOMIC_RELAXED) ||
        !j1850_pwm_ifr_wanted(g.stream.data, g.stream.len, &g.ifr) ||
        g.stream.data[2] == g.ifr.node_address ||
        is_own_echo(g.stream.data, g.stream.len)) {
        portEXIT_CRITICAL_ISR(&g_ifr_spinlock);
        goto out;
    }
    g.stats.ifr_candidates++;
    uint32_t edge = g.edge_cycle - J1850_PWM_GPIO_LATENCY_US * per_us;
    uint32_t eod = edge + J1850_PWM_TP4_RX_MIN * per_us;
    uint32_t start = edge + J1850_PWM_TP4_NOM * per_us;
    while ((int32_t)(eod - esp_cpu_get_cycle_count()) > 0) {
        if (REG_READ(GPIO_IN_REG) & BIT(PIN_J1850_PWM_RX)) {
            portEXIT_CRITICAL_ISR(&g_ifr_spinlock);
            goto out;
        }
    }
    g.stream.valid = false;
    if (!(REG_READ(GPIO_IN_REG) & BIT(PIN_J1850_PWM_RX)) &&
        ifr_transmit(g.ifr.node_address, start, edge))
        g.stats.ifr_sent++;
    portEXIT_CRITICAL_ISR(&g_ifr_spinlock);
out:
    j1850_callback_exit(&g.edge_guard);
}

/**
 * @brief True when this frame is the previous one sent again.
 *
 * A J1850 frame carries nothing to tell a retransmission from a repeat, so
 * this is a judgement about time: the same bytes, this soon, from a bus whose
 * retransmission interval is measured in hundreds of microseconds.
 */
static bool IRAM_ATTR is_retransmission(const j1850_pwm_rx_t *rx,
                                        uint32_t now) {
    if (g.dup_window_cycles == 0 || g.last_frame_len != rx->len) {
        return false;
    }
    if ((uint32_t)(now - g.last_frame_cycle) > g.dup_window_cycles) {
        return false;
    }

    return memcmp(g.last_frame, rx->data, rx->len) == 0;
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
    uint32_t isr_cycle = esp_cpu_get_cycle_count();
    BaseType_t woken = pdFALSE;
    j1850_pwm_rx_t rx;
    uint8_t quick[J1850_PWM_MAX_FRAME];
    size_t quick_len;
    size_t n;
    bool echo;

    (void)user_ctx;

    quick_len =
        __atomic_load_n(&g.echo_armed, __ATOMIC_ACQUIRE)
            ? j1850_pwm_quick_decode(edata->received_symbols,
                                     edata->num_symbols, quick, sizeof(quick))
            : 0;
    echo = quick_len && is_own_echo(quick, quick_len);

    /* Take a copy and put the receiver straight back on the air. Everything
     * below works from the copy, so the bus is unwatched for the length of a
     * memcpy rather than for the length of a decode - which matters, because
     * a module that did not like the answer starts retransmitting one
     * inter-frame separation from now. */
    n = edata->num_symbols;
    if (n > J1850_PWM_CAPTURE_MAX) {
        n = J1850_PWM_CAPTURE_MAX;
    }
    memcpy(g.cap, edata->received_symbols, n * sizeof(rmt_symbol_word_t));

    if (rmt_receive(channel, g.rx_buf,
                    RX_BUFFER_SYMBOLS * sizeof(rmt_symbol_word_t),
                    &g_rx_config) != ESP_OK) {
        esp_rom_printf("J1850 PWM: receiver would not re-arm\n");
    }

    /* Off the deadline now. The full decoder is what the caller's frames come
     * from and what the error counters mean: it validates every pulse width
     * and every edge interval, which the fast path above does not. */
    j1850_pwm_decode(g.cap, n, &rx);
    g.cap_len = (uint16_t)n;
    g.cap_status = (uint8_t)rx.status;

    g.rx_events++;
    count_status(rx.status);

    if (!echo && rx.status == J1850_PWM_RX_OK && rx.eod && rx.ifr_len == 1 &&
        rx.ifr[0] == g.ifr.node_address) {
        g.stats.ifr_observed++;
        if (!g.stats.ifr_gap_min_us || rx.eod_gap_us < g.stats.ifr_gap_min_us)
            g.stats.ifr_gap_min_us = rx.eod_gap_us;
        if (rx.eod_gap_us > g.stats.ifr_gap_max_us)
            g.stats.ifr_gap_max_us = rx.eod_gap_us;
    }

    if (echo) {
        __atomic_store_n(&g.echo_armed, false, __ATOMIC_RELAXED);
        g.stats.tx_echo_ok++;

        /* Off the critical path deliberately: the receiver is already back on
         * the air by here, so keeping a second copy costs nothing that
         * matters. */
        memcpy(g.tx_cap, g.cap, n * sizeof(rmt_symbol_word_t));
        g.tx_cap_len = (uint16_t)n;
        g.tx_cap_status = (uint8_t)rx.status;
    }

    if (rx.status == J1850_PWM_RX_OK && !echo) {
        struct rx_frame f;

        if (is_retransmission(&rx, isr_cycle)) {
            g.stats.rx_duplicate++;
        } else {
            f.len = rx.len;
            f.status = g.pending_status;
            f.timestamp_us = (uint32_t)esp_timer_get_time();
            g.pending_status = 0;
            memcpy(f.data, rx.data, rx.len);

            if (xQueueSendFromISR(g.rx_queue, &f, &woken) != pdTRUE) {
                g.stats.rx_dropped++;
                g.pending_status |= BUS_RX_BUFFER_OVERFLOW;
            }
        }

        memcpy(g.last_frame, rx.data, rx.len);
        g.last_frame_len = rx.len;
        /* Restarted on every copy, so a module making three attempts has all
         * three collapsed rather than the last one leaking through. */
        g.last_frame_cycle = isr_cycle;
    }

    {
        uint32_t took =
            (esp_cpu_get_cycle_count() - isr_cycle) / g.cpu_ticks_per_us;

        if (took > g.stats.rx_isr_us_max) {
            g.stats.rx_isr_us_max = took;
        }
    }

    return woken == pdTRUE;
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
 * @return true when the bus went idle, false if @p timeout_ms passed first.
 */
static bool wait_bus_idle(uint32_t timeout_ms) {
    const int64_t start = esp_timer_get_time();
    const int64_t deadline = start + (int64_t)timeout_ms * 1000;
    /* Tp6 normally; Tp9 while a break is still being resynchronised from.
     * Both are counted as passive time, which is stricter than the standard's
     * "after the rising edge of the last bit" - that edge is one active phase
     * further back, so this waits if anything too long. */
    const int64_t needed =
        g.break_pending ? J1850_PWM_TP9_NOM : J1850_PWM_TP6_NOM;

    for (;;) {
        int64_t now = esp_timer_get_time();
        int64_t quiet_since;

        if (now > deadline) {
            return false;
        }

        if (gpio_get_level(PIN_J1850_PWM_RX) != J1850_PWM_PASSIVE) {
            /* An idle bus answers within an inter-frame separation, so
             * anything past a couple of milliseconds means real traffic and
             * this thread has no business spinning through it. */
            if (now - start > 2000) {
                vTaskDelay(1);
            }
            continue;
        }

        quiet_since = esp_timer_get_time();
        while (esp_timer_get_time() - quiet_since < needed) {
            if (gpio_get_level(PIN_J1850_PWM_RX) != J1850_PWM_PASSIVE) {
                goto not_idle;
            }
        }

        g.break_pending = false;
        return true;

    not_idle:
        continue;
    }
}

/** @brief One transmit attempt: idle wait, encode, hand to the peripheral. */
static int tx_once(const rmt_symbol_word_t *sym, size_t n) {
    static const rmt_transmit_config_t cfg = {
        .loop_count = 0,
        .flags.eot_level = J1850_PWM_PASSIVE, /* leave the bus released */
    };
    esp_err_t err;

    if (!wait_bus_idle(TX_BUS_IDLE_TIMEOUT_MS)) {
        g.stats.tx_bus_busy++;
        return BUS_ERR_BUS_BUSY;
    }

    err = rmt_transmit(g.tx_chan, g.copy_encoder, sym,
                       n * sizeof(rmt_symbol_word_t), &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "transmit refused: %s", esp_err_to_name(err));
        return BUS_ERR_TX_FAILED;
    }

    err = rmt_tx_wait_all_done(g.tx_chan, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "transmit did not complete: %s", esp_err_to_name(err));
        err = j1850_rmt_recover(g.tx_chan, &g.tx_enabled);
        if (err != ESP_OK) {
            g.started = false;
            ESP_LOGE(TAG, "TX recovery failed: %s", esp_err_to_name(err));
        }
        return BUS_ERR_TX_FAILED;
    }

    g.stats.tx_frames++;
    return 0;
}

static int j1850_pwm_tx(const bus_msg_t *msg, uint32_t flags) {
    /* Addressing rides in the bytes on this bus, so msg->id is not
     * ours to look at. */
    const uint8_t *data = msg->data;
    size_t len = msg->len;

    uint8_t frame[J1850_PWM_MAX_FRAME];
    bool add_crc = g.crc_tx && !(flags & BUS_TX_NO_CHECKSUM);
    size_t n;
    int rc;

    if (!g.started) {
        return BUS_ERR_NOT_READY;
    }
    if (!data || len == 0) {
        return BUS_ERR_BAD_ARG;
    }
    if (len > (add_crc ? J1850_PWM_MAX_FRAME - 1u : J1850_PWM_MAX_FRAME)) {
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
    } else {
        /* The caller's bytes already end in their own error check. J2534's
         * CHECKSUM_DISABLED, which exists so a manufacturer specific scheme
         * can be driven from the application. */
        len -= 1;
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

    n = j1850_pwm_encode(frame, len + 1u, true, g.tx_symbols,
                         J1850_PWM_MAX_SYMBOLS);
    if (n == 0) {
        xSemaphoreGive(g.tx_lock);
        return BUS_ERR_TOO_LONG;
    }

    for (uint8_t attempt = 0;; attempt++) {
        uint32_t events_before = g.rx_events;

        /* Arm echo suppression before the frame goes out: the receive ISR
         * can run before rmt_transmit() has even returned. */
        memcpy(g.echo, frame, len + 1u);
        g.echo_len = len + 1u;
        __atomic_store_n(&g.echo_armed, true, __ATOMIC_RELEASE);

        rc = tx_once(g.tx_symbols, n);
        if (rc != 0) {
            break;
        }

        /* Retry only into total silence. Anything at all on the bus - a
         * module's in-frame response, its answering frame, even a capture
         * that failed to decode - means the frame was heard, and asking a
         * second time would duplicate the request.
         *
         * Watched rather than slept through. Sleeping the whole window and
         * then asking whether anything arrived charges every exchange the
         * full TX_SILENCE_MS even when the answer was already on the wire
         * after three - which is most of them, and which made this call
         * return long after the reply it was waiting for had been
         * timestamped. */
        {
            int64_t deadline = esp_timer_get_time() + TX_SILENCE_MS * 1000;

            while (g.rx_events == events_before &&
                   esp_timer_get_time() < deadline) {
                vTaskDelay(1);
            }
        }

        /* RX closes after TX finishes its final bit cell. Keep echo
         * suppression armed for that capture even when retries are off. */
        if (g.rx_events != events_before || attempt >= g.retries) {
            break;
        }

        g.stats.tx_retries++;
    }

    /* However it ended, an echo that never arrived is worth knowing about:
     * it means the receiver could not hear this node's own driver. */
    if (__atomic_exchange_n(&g.echo_armed, false, __ATOMIC_ACQ_REL) &&
        rc == 0) {
        g.stats.tx_echo_missing++;
    }

    /* J2534's ECHO_PHYSICAL_CHANNEL_TX. The receiver already sees this
     * node's own transmissions - that is what makes arbitration work and what
     * the echo counters are built on - so a loopback copy is a matter of not
     * discarding one rather than of generating one. It is still queued from
     * here rather than from the interrupt, because only here is it known
     * whether the transmission finished. */
    if (rc == 0 && g.loopback) {
        struct rx_frame f;

        f.len = (uint8_t)(len + 1u);
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

static int j1850_pwm_rx(bus_msg_t *msg, TickType_t xTicksToWait) {
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

/* ------------------------------------------------------------------ *
 * Lifecycle
 * ------------------------------------------------------------------ */

static esp_err_t j1850_pwm_close(void);

static esp_err_t j1850_pwm_open(const bus_cfg_t *cfg) {
    (void)cfg;

    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = PIN_J1850_TX_P,
        .clk_src = RMT_CLK_SRC,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .intr_priority = J1850_PWM_INTR_PRIORITY,
        /* A maximum frame is 97 symbols and this block holds 64, so the
         * driver refills it as it drains. There is no risk in that: the
         * refill interrupt fires with half a block still to send, which at
         * 24 us a symbol is three quarters of a millisecond of slack. */
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    rmt_rx_channel_config_t rx_cfg = {
        .gpio_num = PIN_J1850_PWM_RX,
        .clk_src = RMT_CLK_SRC,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = RX_MEM_SYMBOLS,
        .intr_priority = J1850_PWM_INTR_PRIORITY,
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
    j1850_pwm_ifr_cfg_default(&g.ifr);
    g.retries = TX_DEFAULT_RETRIES;
    g.crc_tx = true;
    g.crc_rx = true;
    g.loopback = false;
    g.cpu_ticks_per_us = (uint32_t)esp_clk_cpu_freq() / 1000000u;
    /* Zero: deliver every frame. Collapsing retransmissions is a client's
     * choice, made through BUS_P_DUPLICATE_MS - see j1850_pwm.h. */
    g.dup_window_cycles = 0;

    /* The transceiver is dual mode; 1 selects PWM and its 5 V logic. */
    gpio_set_level(PIN_J1850_MODE, 1);

    g.rx_queue = xQueueCreate(RX_QUEUE_DEPTH, sizeof(struct rx_frame));
    g.tx_lock = xSemaphoreCreateMutex();
    g.rx_buf = heap_caps_aligned_calloc(4, RX_BUFFER_SYMBOLS,
                                        sizeof(rmt_symbol_word_t),
                                        MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!g.rx_queue || !g.tx_lock || !g.rx_buf) {
        ESP_LOGE(TAG, "out of memory bringing the bus up");
        goto fail;
    }

    /*
     * Checked rather than ESP_ERROR_CHECK'd, all the way down.
     *
     * Every one of these can fail for a reason that is a configuration
     * mistake rather than a broken chip - a channel already taken, a clock
     * source the group has settled on, an interrupt priority another channel
     * fixed first - and ESP_ERROR_CHECK turns each of them into abort(). A
     * diagnostic adapter that panics when a bus cannot be opened is one that
     * panics with a programming session half finished, and it takes the
     * console down with it, so the reason is lost as well. Refusing the open
     * and saying why leaves the device answerable.
     */
    if (rmt_new_tx_channel(&tx_cfg, &g.tx_chan) != ESP_OK ||
        rmt_new_copy_encoder(&copy_cfg, &g.copy_encoder) != ESP_OK ||
        j1850_rmt_enable(g.tx_chan, &g.tx_enabled) != ESP_OK) {
        ESP_LOGE(TAG, "could not bring the transmitter up");
        goto fail;
    }

    /* The low side of the pair follows the same RMT signal. The board's
     * driver inverts it, so no inversion is asked for here. Which signal
     * that is depends on the channel the driver just allocated, so it is
     * read back off the pin the driver did bind rather than assumed. */
    gpio_reset_pin(PIN_J1850_TX_N);
    gpio_set_direction(PIN_J1850_TX_N, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(
        PIN_J1850_TX_N,
        REG_READ(GPIO_FUNC_OUT_SEL(PIN_J1850_TX_P)) & GPIO_FUNC0_OUT_SEL_V,
        false, false);

    /* Both latches must be enabled as outputs for the bit banged in-frame
     * response, which drives them directly rather than through the matrix. */
    REG_WRITE(GPIO_ENABLE_W1TS_REG, TX_PIN_MASK);
    g.tx_func_sel[0] = REG_READ(GPIO_FUNC_OUT_SEL(PIN_J1850_TX_P));
    g.tx_func_sel[1] = REG_READ(GPIO_FUNC_OUT_SEL(PIN_J1850_TX_N));

    if (rmt_new_rx_channel(&rx_cfg, &g.rx_chan) != ESP_OK ||
        rmt_rx_register_event_callbacks(g.rx_chan, &rx_cbs, NULL) != ESP_OK ||
        j1850_rmt_enable(g.rx_chan, &g.rx_enabled) != ESP_OK ||
        rmt_receive(g.rx_chan, g.rx_buf,
                    RX_BUFFER_SYMBOLS * sizeof(rmt_symbol_word_t),
                    &g_rx_config) != ESP_OK) {
        ESP_LOGE(TAG, "could not bring the receiver up");
        goto fail;
    }

    if (gpio_set_intr_type(PIN_J1850_PWM_RX, GPIO_INTR_ANYEDGE) != ESP_OK ||
        gpio_isr_handler_add(PIN_J1850_PWM_RX, pwm_edge_isr, NULL) != ESP_OK)
        goto fail;
    g.edge_installed = true;
    if (gpio_intr_enable(PIN_J1850_PWM_RX) != ESP_OK)
        goto fail;

    g.started = true;
    ESP_LOGI(TAG,
             "up: 41.6 kbps, %u us arming threshold, %" PRIu32
             " cycles/us (ROM says %" PRIu32 "), IFR %s as 0x%02X",
             (unsigned)J1850_PWM_RX_IDLE_US, g.cpu_ticks_per_us,
             esp_rom_get_cpu_ticks_per_us(), g.ifr.enabled ? "on" : "off",
             g.ifr.node_address);
    return ESP_OK;

fail:
    j1850_pwm_close();
    return ESP_ERR_INVALID_STATE;
}

static esp_err_t j1850_pwm_close(void) {
    esp_err_t err;
    if (!g.initialized)
        return ESP_OK;
    if (g.tx_lock)
        xSemaphoreTake(g.tx_lock, portMAX_DELAY);
    g.started = false;

    if (g.edge_installed) {
        gpio_intr_disable(PIN_J1850_PWM_RX);
        err = j1850_callbacks_stop(&g.edge_guard);
        if (err != ESP_OK)
            goto out;
        gpio_isr_handler_remove(PIN_J1850_PWM_RX);
        gpio_set_intr_type(PIN_J1850_PWM_RX, GPIO_INTR_DISABLE);
        g.edge_installed = false;
    }
    err = j1850_callbacks_stop(&g.rx_guard);
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

    gpio_set_level(PIN_J1850_TX_P, J1850_PWM_PASSIVE);
    gpio_set_direction(PIN_J1850_TX_P, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_J1850_TX_N, J1850_PWM_PASSIVE);
    gpio_set_direction(PIN_J1850_TX_N, GPIO_MODE_OUTPUT);
    free(g.rx_buf);
    g.rx_buf = NULL;
    if (g.rx_queue) {
        vQueueDelete(g.rx_queue);
        g.rx_queue = NULL;
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
 * J1850 has far fewer than K-Line does, because almost all of ISO 14230's
 * timing is negotiated where J1850's is fixed by the symbol coding. What is
 * left is the node's own identity, the acknowledgement policy, and the two
 * pieces of front-end policy every byte bus carries.
 */
static int j1850_pwm_set_param_locked(bus_param_t p, uint32_t value) {
    if (!g.started) {
        return BUS_ERR_NOT_READY;
    }

    switch (p) {
    case BUS_P_DATA_RATE:
        /* Clause 6.5.3 of J2534 allows 41.6 and 83.3 kbps. Only the first is
         * built: 83.3 kbps needs a different symbol table throughout the
         * codec, and claiming it here would be worse than refusing it. */
        return value == 41600 ? 0 : BUS_ERR_BAD_ARG;

    case BUS_P_NODE_ADDRESS:
        if (value > 0xFF) {
            return BUS_ERR_BAD_ARG;
        }
        g.ifr.node_address = (uint8_t)value;
        return 0;

    case BUS_P_NETWORK_LINE:
        /* This board drives BUS+ and BUS- from one pair of latches, so there
         * is no line to select. J2534 calls the ability optional. */
        return value == BUS_NORMAL ? 0 : BUS_ERR_UNSUPPORTED;

    case BUS_P_IFR_ENABLED:
        __atomic_store_n(&g.ifr.enabled, value != 0, __ATOMIC_RELEASE);
        return 0;

    case BUS_P_IFR_BYTE:
        if (value > 0xFF) {
            return BUS_ERR_BAD_ARG;
        }
        g.ifr.node_address = (uint8_t)value;
        return 0;

    case BUS_P_TX_RETRIES:
        if (value > 8) {
            return BUS_ERR_BAD_ARG;
        }
        g.retries = (uint8_t)value;
        return 0;

    case BUS_P_DUPLICATE_MS:
        g.dup_window_cycles = value * 1000u * g.cpu_ticks_per_us;
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

    default:
        return BUS_ERR_UNSUPPORTED;
    }
}

static int j1850_pwm_set_param(bus_param_t p, uint32_t value) {
    portENTER_CRITICAL(&g_ifr_spinlock);
    int rc = j1850_pwm_set_param_locked(p, value);
    if (rc == 0 && (p == BUS_P_IFR_ENABLED || p == BUS_P_IFR_BYTE ||
                    p == BUS_P_NODE_ADDRESS))
        __atomic_add_fetch(&g.ifr_generation, 1, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&g_ifr_spinlock);
    return rc;
}

static int j1850_pwm_get_param(bus_param_t p, uint32_t *out) {
    if (!out) {
        return BUS_ERR_BAD_ARG;
    }
    if (!g.started) {
        return BUS_ERR_NOT_READY;
    }

    switch (p) {
    case BUS_P_DATA_RATE:
        *out = 41600;
        return 0;
    case BUS_P_NODE_ADDRESS:
    case BUS_P_IFR_BYTE:
        *out = g.ifr.node_address;
        return 0;
    case BUS_P_NETWORK_LINE:
        *out = BUS_NORMAL;
        return 0;
    case BUS_P_IFR_ENABLED:
        *out = g.ifr.enabled;
        return 0;
    case BUS_P_TX_RETRIES:
        *out = g.retries;
        return 0;
    case BUS_P_DUPLICATE_MS:
        *out = g.cpu_ticks_per_us
                   ? g.dup_window_cycles / (1000u * g.cpu_ticks_per_us)
                   : 0;
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
    default:
        return BUS_ERR_UNSUPPORTED;
    }
}

/**
 * @brief The functional message lookup table, J2534 clause 6.5.3.
 *
 * "The size of the functional message lookup table shall be eight addresses."
 * These are the target bytes this node answers with an in-frame response,
 * which is exactly what the table is for: a frame addressed to a functional
 * group this tester belongs to is one this tester must acknowledge.
 */
static int j1850_pwm_ioctl_impl(bus_ioctl_t id, const void *in, void *out) {
    (void)out;

    if (!g.started) {
        return BUS_ERR_NOT_READY;
    }

    switch (id) {
    case BUS_IOCTL_CLEAR_FUNCT_TABLE:
        g.ifr.target_count = 0;
        return 0;

    case BUS_IOCTL_ADD_FUNCT_ADDR: {
        uint8_t addr;

        if (!in) {
            return BUS_ERR_BAD_ARG;
        }
        addr = *(const uint8_t *)in;

        for (uint8_t i = 0; i < g.ifr.target_count; i++) {
            if (g.ifr.targets[i] == addr) {
                return 0; /* already there; J2534 calls this success */
            }
        }
        if (g.ifr.target_count >= J1850_PWM_MAX_IFR_TARGETS) {
            return BUS_ERR_NO_SPACE;
        }
        g.ifr.targets[g.ifr.target_count++] = addr;
        return 0;
    }

    case BUS_IOCTL_DEL_FUNCT_ADDR: {
        uint8_t addr;

        if (!in) {
            return BUS_ERR_BAD_ARG;
        }
        addr = *(const uint8_t *)in;

        for (uint8_t i = 0; i < g.ifr.target_count; i++) {
            if (g.ifr.targets[i] == addr) {
                g.ifr.targets[i] = g.ifr.targets[--g.ifr.target_count];
                return 0;
            }
        }
        return BUS_ERR_BAD_ARG;
    }

    case BUS_IOCTL_CLEAR_RX_QUEUE:
        if (g.rx_queue) {
            xQueueReset(g.rx_queue);
        }
        return 0;

    case BUS_IOCTL_CLEAR_TX_QUEUE:
        /* Transmissions are synchronous: j1850_pwm_tx() does not return until
         * the frame is on the wire, so there is never a queue to clear.
         * J2534 requires success when the queue is already empty. */
        return 0;

    default:
        return BUS_ERR_UNSUPPORTED;
    }
}

/* ------------------------------------------------------------------ *
 * Diagnostics
 * ------------------------------------------------------------------ */

static int j1850_pwm_ioctl(bus_ioctl_t id, const void *in, void *out) {
    bool table = id == BUS_IOCTL_CLEAR_FUNCT_TABLE ||
                 id == BUS_IOCTL_ADD_FUNCT_ADDR ||
                 id == BUS_IOCTL_DEL_FUNCT_ADDR;
    if (!table)
        return j1850_pwm_ioctl_impl(id, in, out);
    portENTER_CRITICAL(&g_ifr_spinlock);
    int rc = j1850_pwm_ioctl_impl(id, in, out);
    if (rc == 0)
        __atomic_add_fetch(&g.ifr_generation, 1, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&g_ifr_spinlock);
    return rc;
}

/**
 * @brief The shared counters, filled from this driver's own.
 *
 * Several of J1850's failure modes have no shared name and are not given one:
 * a symbol that matches no pulse width means nothing on a UART bus, and
 * folding it into rx_frame_err would make a counter that reads the same and
 * means something different depending on which bus you asked. Those stay in
 * j1850_pwm_get_native_stats(), which the shell and the target suite use.
 */
static void j1850_pwm_bus_stats(bus_stats_t *out) {
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
                        g.stats.rx_bad_timing + g.stats.rx_no_sof +
                        g.stats.rx_empty;
    out->rx_duplicate = g.stats.rx_duplicate;
    out->tx_msgs = g.stats.tx_frames;
    out->tx_echo_ok = g.stats.tx_echo_ok;
    out->tx_echo_missing = g.stats.tx_echo_missing;
    out->tx_retries = g.stats.tx_retries;
    out->tx_bus_busy = g.stats.tx_bus_busy;
}

static void j1850_pwm_bus_reset_stats(void) {
    memset(&g.stats, 0, sizeof(g.stats));
}

void j1850_pwm_get_native_stats(j1850_pwm_stats_t *out) {
    if (out) {
        *out = g.stats;
    }
}

size_t j1850_pwm_get_capture(j1850_pwm_capture_sel_t which,
                             rmt_symbol_word_t *out, size_t cap,
                             j1850_pwm_rx_status_t *status) {
    bool tx = (which == J1850_PWM_CAP_TX_ECHO);
    const rmt_symbol_word_t *src = tx ? g.tx_cap : g.cap;
    size_t n = tx ? g.tx_cap_len : g.cap_len;

    if (status) {
        *status = (j1850_pwm_rx_status_t)(tx ? g.tx_cap_status : g.cap_status);
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

const bus_ops_t j1850_pwm_bus_ops = {
    .name = "J1850 PWM",
    .open = j1850_pwm_open,
    .close = j1850_pwm_close,
    .send = j1850_pwm_tx,
    .recv = j1850_pwm_rx,
    .set_param = j1850_pwm_set_param,
    .get_param = j1850_pwm_get_param,
    .ioctl = j1850_pwm_ioctl,
    .get_stats = j1850_pwm_bus_stats,
    .reset_stats = j1850_pwm_bus_reset_stats,
};
