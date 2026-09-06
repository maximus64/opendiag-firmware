/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file kline.c
 * @brief The K-Line ISO 9141 and KWP protocols
 *
 * One task owns UART1 and does everything on the wire. That is not an
 * arbitrary choice of shape - it falls out of two properties of this bus:
 *
 *   - An ISO 9141-2 message carries no length anywhere, so the only thing
 *     that ends one is the bus going quiet for P1max. Measuring that needs a
 *     reader in place at the moment the last byte arrives, not one that turns
 *     up later and finds four bytes sitting in a FIFO with no timestamps.
 *
 *   - The transceiver loops back what this node drives. Every byte
 *     transmitted arrives on the receiver a moment later, and something has
 *     to consume it before it is mistaken for an answer. Only the writer
 *     knows what to expect, so the writer and the reader have to be the same
 *     thread - and once they are, comparing the echo is free.
 *
 * Callers hand work to that task through a one-slot command queue and wait
 * for it; a mutex upstream of the queue means there is only ever one command
 * outstanding, which is why the request buffer can be a single static one.
 */

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "bus.h"
#include "kline.h"
#include "pinout.h"
#include "utility.h"

#define TAG "KLINE"

#define KLINE_UART UART_NUM_1

/*
 * Room for a maximum length message and then some, so a burst of replies
 * cannot overrun the receiver while the task is busy framing.
 */
#define KLINE_RX_RINGBUF 1024

/*
 * UART events in flight. Losing one costs nothing - every path drains the
 * ring buffer rather than trusting the event to say how much is there.
 */
#define KLINE_UART_EVENTS 16

#define KLINE_RX_QUEUE 8

#define KLINE_TASK_STACK 4096

/*
 * Above the session tasks: framing by inter-byte gap is time sensitive and
 * a client's parser is not.
 */
#define KLINE_TASK_PRIO 6

/* Enough room to slide past settling/noise before the ten edges of $55. */
#define KLINE_SYNC_CAPTURE_MAX 32

/* One 5 baud bit. Clause 5.1.5.2.2: ten of them, two seconds in total. */
#define KLINE_5BAUD_BIT_MS 200

/*
 * Slack added to every window before it is treated as expired.
 * Some ECUs might not strictly follow the timing specification, so give them
 * some slack to ensure proper operation.
 */
#define KLINE_WINDOW_SLACK_MS 50

/*
 * How long to wait for the StartCommunication response of a fast init.
 * P2max by the book; this is deliberately more forgiving, for the reason
 * above.
 */
#define KLINE_FAST_RESP_MS 300

/* How long after a periodic message its answers are still ours to swallow. */
#define KLINE_PERIODIC_QUIET_MS 60

/*
 * J2534 Table 57 counts P1 to P4 in half milliseconds and W and T in whole
 * ones. Keeping its units means SET_CONFIG needs no conversion table.
 */
#define P_TO_US(v) ((int64_t)(v) * 500)
#define W_TO_US(v) ((int64_t)(v) * 1000)
#define P_FROM_MS(ms) ((uint32_t)((ms) * 2))

/* ------------------------------------------------------------------ *
 * State
 * ------------------------------------------------------------------ */

typedef struct {
    uint8_t data[KLINE_MAX_MSG];
    uint16_t len;
    uint16_t status;
    int64_t start_us; /* When its first byte landed. */
    int64_t end_us;   /* And its last: the J2534 timestamp. */
} kline_frame_t;

typedef enum {
    CMD_TX,
    CMD_FIVE_BAUD,
    CMD_FAST_INIT,
    CMD_STOP_COMM,
} kline_cmd_t;

/* Whether the transceiver loops our own transmissions back at us. */
typedef enum {
    ECHO_UNKNOWN = 0,
    ECHO_PRESENT,
    ECHO_ABSENT,
} kline_echo_t;

/**
 * @brief Every parameter of bus.h that applies to this bus.
 *
 * Units are J2534's: P1 to P4 in half milliseconds, W and T in whole ones.
 * The defaults are the ones both standards agree on, except where they do
 * not - and those four are called out, because a default that silently picks
 * a side is how a driver ends up only working under one front-end.
 */
typedef struct {
    uint32_t baud;

    /* Clause 4.4 Table 1, and J2534 Table 57. */
    uint32_t p1_max; /* 40 = 20 ms */
    uint32_t p2_max; /* 100 = 50 ms */
    uint32_t p3_min; /* 110 = 55 ms */
    uint32_t p4_min; /* 10 = 5 ms */

    /* Table 6, and J2534 Table 57. Whole milliseconds. */
    uint32_t w0_min; /* 300 - ISO 9141 bus idle before the address word */
    uint32_t w1_max; /* 300 */
    uint32_t w2_max; /* 20 */
    uint32_t w3_max; /* 20 */
    uint32_t w4_min; /* 25 */
    uint32_t w4_max; /* 50 */
    uint32_t w5_min; /* 300 - ISO 14230 bus idle before the address word */
    uint32_t tidle;  /* 300 */
    uint32_t tinil;  /* 25 */
    uint32_t twup;   /* 50 */

    uint32_t parity;        /* BUS_PARITY_* */
    uint32_t data_bits;     /* BUS_DATA_BITS_* */
    uint32_t five_baud_mod; /* BUS_FIVE_BAUD_* */

    /** J2534 ECHO_PHYSICAL_CHANNEL_TX: put our own transmissions in the
     *  receive queue, flagged. Off, because a client that did not ask to see
     *  its own requests should not have to filter them out. */
    bool loopback;

    /** Discard a message whose checksum does not add up. On: both standards
     *  agree, and the flag exists for the manufacturer specific checks that
     *  do not use an ISO checksum. */
    bool checksum_rx;

    /** Append the checksum on transmit. On, for the same reason. */
    bool checksum_tx;

    /**
     * Retransmissions after a corrupted transmission. Zero.
     *
     * Clause 6.6 has the tester retransmit; J2534 clause 6.5.1 says "shall
     * detect corrupted transmissions and not attempt to resend". The stricter
     * of the two is the safer default, and ELM327 asks for one.
     */
    uint32_t tx_retries;

    /** Milliseconds of silence before the periodic message goes out. Zero
     *  switches it off, which is the default: a driver that talks to the
     *  vehicle on its own account before anybody asked is a surprise. */
    uint32_t periodic_ms;

    /** Swallow the answers to it. On - see bus.h. */
    bool periodic_quiet;

    /**
     * Let the key bytes select the timing set. Off.
     *
     * Clause 5.1.5.1 says an ECU announces normal or extended timing this
     * way; J2534 clause 7.4.5 says the interface "shall not alter timing
     * parameters based on key bytes". Off means the timers stay where the
     * front-end put them, which is the reading that cannot surprise anybody.
     */
    bool timing_from_keybytes;

    uint8_t periodic[BUS_PERIODIC_MAX];
    uint8_t periodic_len;
} kline_cfg_t;

static struct {
    bool started;
    bool uart_installed;
    bool stop_requested;

    TaskHandle_t task;
    QueueHandle_t uart_q;
    QueueHandle_t cmd_q;
    QueueHandle_t rx_q;
    QueueSetHandle_t set;
    SemaphoreHandle_t cmd_done;
    SemaphoreHandle_t exited;
    SemaphoreHandle_t api_lock;  /**< One outstanding command at a time. */
    SemaphoreHandle_t data_lock; /**< Guards the fields below from readers. */

    /* Request payload. Written by a caller holding api_lock, read by the
     * task while that caller is blocked on cmd_done. */
    kline_cmd_t cmd;
    uint8_t req[KLINE_MAX_MSG];
    size_t req_len;
    uint32_t req_flags;
    bus_init_t req_init;
    int result;

    kline_cfg_t cfg;

    /* Live state, task-owned. */
    uint32_t active_baud; /**< UART rate; slow init may detect a new one. */
    bus_link_t link;
    kline_keybytes_t keys;
    kline_echo_t echo;
    int64_t last_bus_us; /**< Last byte seen or driven, either direction. */
    int64_t tx_end_us;   /**< End of the most recent transmission. */
    bool p2_pending;     /**< The next message completes a P2 measurement. */
    bool tx_answered;    /**< Something replied to the last transmission. */
    int64_t suppress_until_us;
    uint16_t pending_status; /**< Flags owed to the next delivered message. */

    /* Receive assembly. */
    uint8_t asm_buf[KLINE_MAX_MSG];
    size_t asm_len;
    int64_t asm_start_us;
    int64_t asm_last_us;

    /* The UART cannot determine all ISO rates on ESP32-S3: its hardware
     * auto-baud period counter saturates below about 9.8 kbaud on the 40 MHz
     * XTAL clock. A GPIO any-edge ISR therefore timestamps the $55 pattern.
     * The release-store to sync_edge_count publishes each completed entry to
     * the K-Line task. */
    uint32_t sync_edge_us[KLINE_SYNC_CAPTURE_MAX];
    uint32_t sync_edge_count;

    bus_stats_t stats;
} g;

/* The mutex is created on the first open. Reading the counters of a bus that
 * has never been up is legitimate - the shell does it - and there is nothing
 * to race with when no task exists, so the guard degrades to nothing. */
#define LOCK()                                                                 \
    do {                                                                       \
        if (g.data_lock)                                                       \
            xSemaphoreTake(g.data_lock, portMAX_DELAY);                        \
    } while (0)
#define UNLOCK()                                                               \
    do {                                                                       \
        if (g.data_lock)                                                       \
            xSemaphoreGive(g.data_lock);                                       \
    } while (0)

/* ------------------------------------------------------------------ *
 * Small helpers
 * ------------------------------------------------------------------ */

static inline int64_t now_us(void) { return esp_timer_get_time(); }

/** @brief Microseconds one character occupies at @p baud. */
static uint32_t byte_time_at_baud(uint32_t baud) {
    /* Start, data, an optional parity bit, stop. */
    uint32_t bits = 1 + (g.cfg.data_bits == BUS_DATA_BITS_7 ? 7 : 8) + 1;

    if (baud == 0) {
        baud = KLINE_BAUD_DEFAULT;
    }
    if (g.cfg.parity != BUS_PARITY_NONE) {
        bits += 1;
    }

    return (bits * 1000000u + baud - 1) / baud;
}

/** @brief Microseconds one character occupies at the current settings. */
static uint32_t byte_time_us(void) { return byte_time_at_baud(g.active_baud); }

/**
 * @brief The interval between two timestamps, less the character that ended
 *        it, clamped at zero.
 *
 * Every interval in Table 6 is defined between the *edges* of bytes, and what
 * this driver can observe is when a byte finished arriving. Subtracting one
 * character time turns one into the other.
 */
static uint32_t span_us(int64_t from, int64_t to, uint32_t bt) {
    int64_t d = to - from - (int64_t)bt;

    return d > 0 ? (uint32_t)d : 0;
}

/** @brief Sleep until @p deadline, giving the CPU up for all but the last
 *         millisecond so the wait is neither a busy loop nor a tick rounding
 *         error on a 25 ms pulse the standard specifies to within one. */
static void delay_until_us(int64_t deadline) {
    int64_t left = deadline - now_us();

    if (left <= 0) {
        return;
    }

    if (left > 2000) {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)((left - 1000) / 1000)));
        left = deadline - now_us();
    }

    if (left > 0) {
        delay_us((uint32_t)left);
    }
}

/* ------------------------------------------------------------------ *
 * The line
 * ------------------------------------------------------------------ */

/**
 * @brief Apply the baud rate, parity and data bits to the peripheral.
 *
 * J2534 clause 6.5.1 requires odd and even parity with seven or eight data
 * bits, so these are three parameters rather than a hard-coded 8N1. Nothing
 * an OBD-II vehicle does needs them; the manufacturer specific systems a scan
 * tool also has to reach do.
 */
static esp_err_t apply_line_cfg(void) {
    uart_word_length_t bits = (g.cfg.data_bits == BUS_DATA_BITS_7)
                                  ? UART_DATA_7_BITS
                                  : UART_DATA_8_BITS;
    uart_parity_t par = UART_PARITY_DISABLE;
    esp_err_t err;

    if (g.cfg.parity == BUS_PARITY_ODD) {
        par = UART_PARITY_ODD;
    } else if (g.cfg.parity == BUS_PARITY_EVEN) {
        par = UART_PARITY_EVEN;
    }

    err = uart_set_baudrate(KLINE_UART, (int)g.active_baud);
    if (err != ESP_OK) {
        return err;
    }
    err = uart_set_word_length(KLINE_UART, bits);
    if (err != ESP_OK) {
        return err;
    }

    return uart_set_parity(KLINE_UART, par);
}

/**
 * @brief Take the transmit pin away from the UART to drive it by hand.
 *
 * Both initialisation patterns are far slower than any baud rate the
 * peripheral can generate - 5 bits per second for the address word, a 25 ms
 * pulse for the wake up - so they are bit banged. K-Line idles high, and so
 * does the pin under its pull-up between the reset and the first drive, so
 * the handover puts nothing on the bus.
 */
static void tx_pin_to_gpio(void) {
    gpio_reset_pin(PIN_KLINE_TX);
    gpio_set_level(PIN_KLINE_TX, 1);
    gpio_set_direction(PIN_KLINE_TX, GPIO_MODE_OUTPUT);
}

static void tx_pin_to_uart(void) {
    ESP_ERROR_CHECK(uart_set_pin(KLINE_UART, PIN_KLINE_TX, PIN_KLINE_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

/** @brief Forget everything not yet framed: the FIFO, the ring buffer and
 *         the partial message. Used where what is in flight is known to
 *         predate what is about to happen. */
static void rx_discard(void) {
    uart_flush_input(KLINE_UART);
    g.asm_len = 0;
}

/* ------------------------------------------------------------------ *
 * Slow-init synchronisation rate detection
 * ------------------------------------------------------------------ */

/**
 * @brief Timestamp K-Line RX transitions while waiting for the $55 byte.
 *
 * ESP-IDF places both esp_timer_get_time() and the FreeRTOS ISR notification
 * path in IRAM in this build. Keeping the complete handler there matters:
 * the 10400-baud edges are only 96 us apart, and a flash-cache stall must not
 * turn two transitions into one GPIO interrupt. Decoding and UART changes
 * stay in task context.
 */
static void IRAM_ATTR sync_edge_isr(void *arg) {
    uint32_t n = __atomic_load_n(&g.sync_edge_count, __ATOMIC_RELAXED);

    (void)arg;
    if (n >= KLINE_SYNC_CAPTURE_MAX) {
        return;
    }

    g.sync_edge_us[n] = (uint32_t)esp_timer_get_time();
    __atomic_store_n(&g.sync_edge_count, n + 1, __ATOMIC_RELEASE);

    if (n + 1 >= KLINE_SYNC_EDGE_COUNT && g.task) {
        BaseType_t higher_woken = pdFALSE;

        vTaskNotifyGiveFromISR(g.task, &higher_woken);
        if (higher_woken) {
            portYIELD_FROM_ISR();
        }
    }
}

static esp_err_t sync_capture_start(void) {
    esp_err_t err;

    gpio_intr_disable(PIN_KLINE_RX);
    __atomic_store_n(&g.sync_edge_count, 0, __ATOMIC_RELEASE);
    (void)ulTaskNotifyTake(pdTRUE, 0);

    err = gpio_set_intr_type(PIN_KLINE_RX, GPIO_INTR_ANYEDGE);
    if (err != ESP_OK) {
        return err;
    }
    err = gpio_isr_handler_add(PIN_KLINE_RX, sync_edge_isr, NULL);
    if (err != ESP_OK) {
        gpio_set_intr_type(PIN_KLINE_RX, GPIO_INTR_DISABLE);
        return err;
    }

    err = gpio_intr_enable(PIN_KLINE_RX);
    if (err != ESP_OK) {
        gpio_isr_handler_remove(PIN_KLINE_RX);
        gpio_set_intr_type(PIN_KLINE_RX, GPIO_INTR_DISABLE);
    }
    return err;
}

static void sync_capture_stop(void) {
    gpio_intr_disable(PIN_KLINE_RX);
    gpio_isr_handler_remove(PIN_KLINE_RX);
    gpio_set_intr_type(PIN_KLINE_RX, GPIO_INTR_DISABLE);
    (void)ulTaskNotifyTake(pdTRUE, 0);
}

/** Expand a captured 32-bit timestamp next to a recent 64-bit timestamp. */
static int64_t sync_time64(uint32_t stamp, int64_t near_us) {
    return near_us + (int32_t)(stamp - (uint32_t)near_us);
}

/**
 * @brief Find and time the $55 pattern inside W1.
 *
 * A UART frame containing $55 in 8N1 changes level at every bit boundary:
 * ten transitions, nine equal intervals. Sliding over the capture makes a
 * harmless settling edge before the byte tolerable, while the interval
 * checks reject a missed edge or a noise spike. The first edge must leave
 * the idle-high line low, i.e. it must be a UART start bit.
 */
static bool detect_sync_baud(int64_t addr_end, uint32_t slack_ms,
                             uint32_t *baud_out, int64_t *sync_start_out) {
    int64_t start_deadline = addr_end + W_TO_US(g.cfg.w1_max + slack_ms);
    int64_t capture_deadline =
        start_deadline + (10000000ll + KLINE_BAUD_MIN - 1) / KLINE_BAUD_MIN;
    size_t next = 0;

    if (sync_capture_start() != ESP_OK) {
        ESP_LOGE(TAG, "cannot arm K-Line sync edge capture");
        return false;
    }

    for (;;) {
        uint32_t count = __atomic_load_n(&g.sync_edge_count, __ATOMIC_ACQUIRE);

        if (count > KLINE_SYNC_CAPTURE_MAX) {
            count = KLINE_SYNC_CAPTURE_MAX;
        }

        while (next + KLINE_SYNC_EDGE_COUNT <= count) {
            uint32_t edges[KLINE_SYNC_EDGE_COUNT];
            uint32_t baud;
            int64_t start;

            for (size_t i = 0; i < KLINE_SYNC_EDGE_COUNT; i++) {
                edges[i] = g.sync_edge_us[next + i];
            }

            start = sync_time64(edges[0], addr_end);
            /* Capture begins while the line is idle high, so even-numbered
             * transitions are falling edges. */
            baud =
                (next & 1u) == 0 && start <= start_deadline
                    ? kline_sync_baud_from_edges(edges, KLINE_SYNC_EDGE_COUNT)
                    : 0;
            next++;

            if (baud != 0) {
                sync_capture_stop();
                *baud_out = baud;
                *sync_start_out = start;
                return true;
            }
        }

        if (count == KLINE_SYNC_CAPTURE_MAX || now_us() >= capture_deadline) {
            break;
        }

        {
            int64_t left = capture_deadline - now_us();
            TickType_t ticks =
                pdMS_TO_TICKS((uint32_t)((left > 0 ? left : 1) + 999) / 1000);

            if (ticks == 0) {
                ticks = 1;
            }
            (void)ulTaskNotifyTake(pdTRUE, ticks);
        }
    }

    sync_capture_stop();
    return false;
}

/* ------------------------------------------------------------------ *
 * Receive framing
 * ------------------------------------------------------------------ */

static void asm_push(uint8_t b, int64_t when) {
    if (g.asm_len == 0) {
        g.asm_start_us = when;
    } else {
        uint32_t gap = (uint32_t)(when - g.asm_last_us);

        /* Bytes read out of the ring buffer in one batch share a timestamp,
         * so this only ever sees a gap the task actually waited through -
         * which is the one worth reporting. */
        if (gap > g.stats.last_p1_max_us) {
            g.stats.last_p1_max_us = gap;
        }
    }

    if (g.asm_len < KLINE_MAX_MSG) {
        g.asm_buf[g.asm_len++] = b;
    } else {
        /* Past the longest message the standard defines and still no gap.
         * Whatever this is, it is not one message; start over rather than
         * hold a buffer full of it. */
        g.stats.rx_too_long++;
        g.asm_len = 0;
        g.asm_start_us = when;
        g.asm_buf[g.asm_len++] = b;
    }

    g.asm_last_us = when;
}

/**
 * @brief Decide whether the bytes gathered so far are a whole message.
 *
 * Three ways one can end, in order of how quickly they settle it:
 *
 *  - The format byte said how long the message would be and the checksum
 *    agrees at exactly that length (clause 4.1.4). This is the KWP fast path,
 *    and J2534 clause 6.5.2 names it: "shall use the header length field and
 *    P1_MAX, whichever occurs first". It costs nothing to be wrong about - a
 *    coincidence still has to pass the checksum, and if the length hint was
 *    nonsense the gap catches it.
 *
 *  - The format byte is one of the two CARB ones, which encode no length at
 *    all, but ISO 9141-2 caps a message at eleven bytes. Eleven bytes that
 *    sum correctly cannot be the front of anything longer.
 *
 *  - The bus went quiet for P1max. This is the only thing that ends a short
 *    ISO 9141-2 message, and the backstop under everything else.
 *
 * With checksum verification off, only the gap ends a message: without a
 * checksum to confirm it, a length field is a guess, and guessing wrong
 * splits one message into two.
 *
 * @return true when @p out was filled.
 */
static bool asm_complete(bool gap_elapsed, kline_frame_t *out) {
    size_t want;
    uint16_t status = 0;

    if (g.asm_len == 0) {
        return false;
    }

    if (g.cfg.checksum_rx) {
        want = kline_msg_len(g.asm_buf, g.asm_len);

        if (want && g.asm_len == want && kline_frame_ok(g.asm_buf, g.asm_len)) {
            goto deliver;
        }

        if (kline_is_carb_fmt(g.asm_buf[0]) &&
            g.asm_len == KLINE_CARB_MAX_MSG &&
            kline_frame_ok(g.asm_buf, g.asm_len)) {
            goto deliver;
        }
    }

    if (!gap_elapsed) {
        return false;
    }

    if (g.asm_len < KLINE_MIN_MSG) {
        /* One or two bytes and then silence. A line settling after an
         * initialisation, or an ECU that gave up mid-message. */
        g.stats.rx_short++;
        g.asm_len = 0;
        return false;
    }

    if (!kline_frame_ok(g.asm_buf, g.asm_len)) {
        g.stats.rx_bad_checksum++;
        if (g.cfg.checksum_rx) {
            g.asm_len = 0;
            return false;
        }
        status |= BUS_RX_BAD_CHECKSUM;
    }

deliver:
    memcpy(out->data, g.asm_buf, g.asm_len);
    out->len = (uint16_t)g.asm_len;
    out->status = status;
    out->start_us = g.asm_start_us;
    out->end_us = g.asm_last_us;
    g.asm_len = 0;
    return true;
}

/** @brief Read everything the UART has, without waiting. */
static void rx_drain(void) {
    uint8_t buf[64];
    int n;

    while ((n = uart_read_bytes(KLINE_UART, buf, sizeof(buf), 0)) > 0) {
        int64_t when = now_us();

        for (int i = 0; i < n; i++) {
            asm_push(buf[i], when);
        }

        g.stats.rx_bytes += (uint32_t)n;
        g.last_bus_us = when;
    }
}

static void uart_event(const uart_event_t *ev) {
    switch (ev->type) {
    case UART_FIFO_OVF:
    case UART_BUFFER_FULL:
        /* Bytes were lost, so whatever is half assembled is a fragment. The
         * next message delivered carries the flag, which is where J2534
         * clause 6.10 puts a buffer overflow: on the first message after it,
         * so the application knows a gap preceded what it is holding. */
        g.stats.rx_overrun++;
        g.pending_status |= BUS_RX_BUFFER_OVERFLOW;
        rx_discard();
        break;
    case UART_BREAK:
        g.stats.rx_break++;
        g.pending_status |= BUS_RX_BREAK;
        break;
    case UART_FRAME_ERR:
    case UART_PARITY_ERR:
        /* Nearly always the wrong baud rate, which is what makes this worth
         * a counter of its own rather than a share of the checksum errors. */
        g.stats.rx_frame_err++;
        break;
    default:
        break;
    }
}

/**
 * @brief Wait for one complete message, a command, or @p deadline.
 *
 * The single place the task blocks. It sleeps on the queue set - the UART's
 * event queue and the command queue together - so a request handed over while
 * the bus is quiet is picked up at once rather than at the end of some poll
 * interval, and a message being assembled still wakes the task in time to
 * measure its closing gap.
 *
 * @return 1 with @p out filled, 0 when @p deadline passed, -1 when a command
 *         is waiting in g.cmd.
 */
static int rx_service(int64_t deadline_us, kline_frame_t *out) {
    for (;;) {
        int64_t now = now_us();
        int64_t wake = deadline_us;
        int64_t gap_at = 0;
        QueueSetMemberHandle_t member;
        TickType_t ticks;

        if (g.asm_len) {
            gap_at = g.asm_last_us + P_TO_US(g.cfg.p1_max);
            if (gap_at < wake) {
                wake = gap_at;
            }
        }

        if (now >= wake) {
            if (gap_at && now >= gap_at && asm_complete(true, out)) {
                return 1;
            }
            if (now >= deadline_us) {
                return 0;
            }
            continue;
        }

        ticks = pdMS_TO_TICKS((uint32_t)((wake - now + 999) / 1000));
        if (ticks == 0) {
            ticks = 1;
        }

        member = xQueueSelectFromSet(g.set, ticks);

        if (member == (QueueSetMemberHandle_t)g.cmd_q) {
            if (xQueueReceive(g.cmd_q, &g.cmd, 0) == pdTRUE) {
                return -1;
            }
            continue;
        }

        if (member == (QueueSetMemberHandle_t)g.uart_q) {
            uart_event_t ev;

            /* uart_flush_input() resets this queue behind the set's back, so
             * a handle can be handed over with nothing behind it. */
            if (xQueueReceive(g.uart_q, &ev, 0) == pdTRUE) {
                uart_event(&ev);
            }
        }

        rx_drain();

        if (asm_complete(false, out)) {
            return 1;
        }
    }
}

/**
 * @brief Read exactly one byte, un-framed, before @p deadline.
 *
 * The handshake bytes of a 5 baud init are not messages - no format byte, no
 * checksum, nothing the framer could make sense of - so they are read
 * straight off the wire.
 */
static bool rx_raw_byte(int64_t deadline_us, uint8_t *out) {
    for (;;) {
        int64_t left = deadline_us - now_us();
        TickType_t ticks;

        if (left <= 0) {
            return false;
        }

        ticks = pdMS_TO_TICKS((uint32_t)((left + 999) / 1000));
        if (ticks == 0) {
            ticks = 1;
        }

        if (uart_read_bytes(KLINE_UART, out, 1, ticks) == 1) {
            /* Not counted in rx_bytes: this path also swallows our own echo,
             * and a receive counter that includes what we transmitted is a
             * counter nobody can read. */
            g.last_bus_us = now_us();
            return true;
        }
    }
}

/** @brief rx_service() for the initialisation sequences, which are already
 *         inside a command and cannot be interrupted by another one. */
static bool wait_frame(int64_t deadline_us, kline_frame_t *out);

/* ------------------------------------------------------------------ *
 * Delivery
 * ------------------------------------------------------------------ */

static void publish(kline_frame_t *f) {
    if (g.p2_pending) {
        g.stats.last_p2_us = (uint32_t)(f->start_us - g.tx_end_us);
        g.p2_pending = false;
    }

    g.tx_answered = true;

    f->status |= g.pending_status;
    g.pending_status = 0;

    if (now_us() < g.suppress_until_us) {
        /* An answer to a message this driver sent on its own account. */
        g.stats.rx_suppressed++;
        if (g.cfg.periodic_quiet) {
            return;
        }
        f->status |= BUS_RX_PERIODIC_REPLY;
    }

    if (xQueueSend(g.rx_q, f, 0) != pdTRUE) {
        g.stats.rx_dropped++;
        /* The next one that does fit says data was lost before it. */
        g.pending_status |= BUS_RX_BUFFER_OVERFLOW;
        return;
    }

    g.stats.rx_msgs++;
}

/**
 * @brief Put a copy of what we transmitted into the receive queue.
 *
 * J2534's ECHO_PHYSICAL_CHANNEL_TX, with TX_MSG_TYPE set so the application
 * can tell it from a reply. Off by default; a front-end that wants a complete
 * trace of the wire turns it on.
 */
static void publish_loopback(const uint8_t *data, size_t len) {
    kline_frame_t f;

    if (!g.cfg.loopback || len > KLINE_MAX_MSG) {
        return;
    }

    memcpy(f.data, data, len);
    f.len = (uint16_t)len;
    f.status = BUS_RX_TX_MSG_TYPE;
    f.start_us = g.tx_end_us;
    f.end_us = g.tx_end_us;

    if (xQueueSend(g.rx_q, &f, 0) != pdTRUE) {
        g.stats.rx_dropped++;
    }
}

/**
 * @brief Wait out the quiet time owed before a request, without going deaf.
 *
 * J2534 clause 6.5.1 states the rule precisely: after a request that drew a
 * response, wait P3min; after one that drew nothing, wait P2max. The second
 * half is what stops a silent bus from costing 55 ms a request when the
 * standard only asks for 50, and it is why this takes the answered flag into
 * account rather than always waiting the larger figure.
 *
 * A reply that turns up during the wait restarts it - and, more to the point,
 * still belongs to whoever asked for it. Sleeping through the window instead
 * would leave those bytes to be thrown away by the flush at the top of the
 * transmission, which is how a late second module comes to look like a module
 * that never answered.
 *
 * Only ever called from inside a command, and only one command is ever
 * outstanding, so rx_service() cannot report another one here.
 *
 * @param p3_only J2534 WAIT_P3_MIN_ONLY: wait P3min whatever happened.
 */
static void wait_before_tx(bool p3_only) {
    for (;;) {
        int64_t owed = (g.tx_answered || p3_only) ? P_TO_US(g.cfg.p3_min)
                                                  : P_TO_US(g.cfg.p2_max);
        int64_t deadline = g.last_bus_us + owed;
        kline_frame_t f;
        int r;

        if (now_us() >= deadline) {
            return;
        }

        r = rx_service(deadline, &f);

        if (r == 1) {
            publish(&f);
        } else if (r < 0) {
            ESP_LOGE(TAG, "command arrived while one was running");
            return;
        }
    }
}

/* ------------------------------------------------------------------ *
 * Transmit
 * ------------------------------------------------------------------ */

/**
 * @brief Put one message on the wire and watch it go.
 *
 * @param buf     Message with its checksum already appended, if it is to have
 *                one.
 * @param wait    Honour the inter-request quiet time first. False only for
 *                the StartCommunication request of a fast init, which clause
 *                5.1.5.3 requires to follow the wake up pattern immediately.
 * @param p3_only See wait_before_tx().
 *
 * Each byte is read back as it is driven. The transceiver's loopback makes
 * that free, and it is the only way to tell a message that was transmitted
 * from one that was transmitted into somebody else. A byte that comes back
 * changed is a corrupted transmission, which both standards ask a tester to
 * detect; a byte that does not come back at all on the very first attempt is
 * a transceiver without loopback, which is a property of the board rather
 * than a fault, so it is noted once and the checking is dropped for the rest
 * of the session.
 */
static int tx_once(const uint8_t *buf, size_t len, bool wait, bool p3_only) {
    uint32_t echo_us = byte_time_us() * 2 + 2000;
    bool collision = false;
    int rc = 0;

    if (wait) {
        wait_before_tx(p3_only);
    }

    /* Anything unframed at this point predates the request, and so does
     * anything the status flags were describing: they say what happened to
     * the bytes that were just thrown away. Carrying a break raised by a
     * 5 baud address word into the first reply of the session that followed
     * it is not a warning, it is noise. */
    rx_discard();
    g.pending_status = 0;

    for (size_t i = 0; i < len; i++) {
        int64_t sent_us;

        if (uart_write_bytes(KLINE_UART, &buf[i], 1) != 1) {
            return BUS_ERR_TX_FAILED;
        }
        if (uart_wait_tx_done(KLINE_UART, pdMS_TO_TICKS(100)) != ESP_OK) {
            return BUS_ERR_TX_FAILED;
        }

        sent_us = now_us();
        g.last_bus_us = sent_us;

        if (g.echo != ECHO_ABSENT) {
            uint8_t back;

            /* The echo window is a couple of character times: well inside
             * P2min, so a byte arriving in it cannot be an answer. */
            if (rx_raw_byte(sent_us + echo_us, &back)) {
                g.echo = ECHO_PRESENT;
                if (back == buf[i]) {
                    g.stats.tx_echo_ok++;
                } else {
                    g.stats.tx_echo_bad++;
                    collision = true;
                }
            } else {
                g.stats.tx_echo_missing++;
                if (g.echo == ECHO_UNKNOWN) {
                    ESP_LOGW(TAG, "transceiver does not echo; corrupted "
                                  "transmissions will go unnoticed");
                    g.echo = ECHO_ABSENT;
                }
            }
        }

        g.stats.tx_bytes++;

        if (i + 1 < len) {
            /* P4min, measured from the end of the byte just sent. */
            delay_until_us(sent_us + P_TO_US(g.cfg.p4_min));
        }
    }

    g.tx_end_us = now_us();
    g.last_bus_us = g.tx_end_us;
    g.p2_pending = true;
    g.tx_answered = false;

    if (collision) {
        rc = BUS_ERR_ECHO;
    }

    return rc;
}

/** @brief tx_once() with whatever retransmission policy is configured. */
static int tx_message(const uint8_t *buf, size_t len, bool wait, bool p3_only) {
    int rc = tx_once(buf, len, wait, p3_only);

    for (uint32_t i = 0; rc == BUS_ERR_ECHO && i < g.cfg.tx_retries; i++) {
        ESP_LOGW(TAG, "corrupted transmission, retransmitting");
        g.stats.tx_retries++;
        /* Whoever we collided with is entitled to finish. Clause 6.6 puts the
         * retransmission after P2max. */
        delay_until_us(g.last_bus_us + P_TO_US(g.cfg.p2_max));
        rc = tx_once(buf, len, true, p3_only);
    }

    if (rc == 0 || rc == BUS_ERR_ECHO) {
        g.stats.tx_msgs++;
        publish_loopback(buf, len);
    }

    return rc;
}

/**
 * @brief Build the on-wire message: the caller's bytes, plus a checksum.
 *
 * @return Total length, or 0 when it will not fit.
 */
static size_t frame_for_tx(const uint8_t *data, size_t len, uint32_t flags,
                           uint8_t *out) {
    bool add = g.cfg.checksum_tx && !(flags & BUS_TX_NO_CHECKSUM);

    if (len == 0 || len + (add ? 1u : 0u) > KLINE_MAX_MSG) {
        return 0;
    }

    memcpy(out, data, len);
    if (add) {
        out[len] = kline_checksum(data, len);
        len += 1;
    }

    return len;
}

/* ------------------------------------------------------------------ *
 * Initialisation
 * ------------------------------------------------------------------ */

/**
 * @brief The 5 baud address word, clause 5.1.5.2.2.
 *
 * Start bit, eight data bits least significant first, stop bit, 200 ms each:
 * two seconds exactly. The address is driven as given, with no parity added -
 * $33 is a functional address and does not carry the odd parity a physical
 * one would, which is also why the ELM327's AT IIA documents itself as using
 * the byte "exactly as provided".
 */
static void send_5baud_address(uint8_t addr) {
    const TickType_t bit = pdMS_TO_TICKS(KLINE_5BAUD_BIT_MS);

    tx_pin_to_gpio();

    gpio_set_level(PIN_KLINE_TX, 0);
    vTaskDelay(bit);

    for (int i = 0; i < 8; i++) {
        gpio_set_level(PIN_KLINE_TX, (addr >> i) & 1u);
        vTaskDelay(bit);
    }

    gpio_set_level(PIN_KLINE_TX, 1);
    vTaskDelay(bit);

    tx_pin_to_uart();
}

/** @brief Bus idle owed before an initialisation pattern, microseconds.
 *
 *  W0 and W5 are the same 300 ms from two documents - ISO 9141 names one,
 *  ISO 14230 the other - and J2534 exposes both. Honouring the larger is the
 *  only reading that satisfies whichever the vehicle is built to. */
static int64_t init_idle_us(void) {
    uint32_t ms = g.cfg.w0_min > g.cfg.w5_min ? g.cfg.w0_min : g.cfg.w5_min;

    return W_TO_US(ms);
}

/**
 * @brief Clause 5.1.5.2.2 and Table 6, in order.
 *
 * Address word, $55, two key bytes, our inverted key byte 2, the ECU's
 * inverted address. How much of the tail actually happens is
 * BUS_P_FIVE_BAUD_MOD, because J2534 lets an application ask for any of
 * four truncations of it and some pre-OBD ECUs need one.
 *
 * Every wait is the parameter's value plus a little slack, and every interval
 * is measured on the way past, because an ECU that answers inside its window
 * and an ECU that answers at the edge of it are the same vehicle in summer
 * and different vehicles in winter.
 */
static int init_5baud(const bus_init_t *in) {
    kline_keybytes_t keys;
    int64_t addr_end, sync_start, sync_at, kb1_at, kb2_at;
    uint8_t b, kb1, kb2;
    uint32_t configured_baud, detected_baud, bt;
    uint32_t slack = KLINE_WINDOW_SLACK_MS;

    /* DATA_RATE is the configured starting point, not a cached result from a
     * previous ECU. The sync pattern may replace active_baud for this link. */
    g.active_baud = g.cfg.baud;
    if (apply_line_cfg() != ESP_OK) {
        return BUS_ERR_INIT;
    }

    /* W0 / W5. An ECU part way through validating an address word ignores
     * everything until it has the whole two seconds, so starting early is how
     * a tester makes a working vehicle look dead. J2534 lets an application
     * set this to zero and take that risk deliberately. */
    delay_until_us(g.last_bus_us + init_idle_us());
    rx_discard();

    ESP_LOGI(TAG, "5 baud init, address %02X", in->address);
    send_5baud_address(in->address);
    addr_end = now_us();

    /* Our own address word came back through the loopback as two seconds of
     * framing errors. None of it is a message. */
    rx_discard();

    /* W1: $55 is not merely a marker. Clause 5.1.5.2.2 requires the tester
     * to recognise the communication rate from its alternating bits. Capture
     * those physical transitions before asking the UART for the key bytes. */
    if (!detect_sync_baud(addr_end, slack, &detected_baud, &sync_start)) {
        ESP_LOGW(TAG, "no valid synchronisation pattern within W1");
        g.stats.init_no_sync++;
        return BUS_ERR_INIT;
    }

    configured_baud = g.active_baud;
    bt = byte_time_at_baud(detected_baud);
    sync_at = sync_start + bt;
    delay_until_us(sync_at); /* do not retune in the sync byte's stop bit */

    g.active_baud = detected_baud;
    if (apply_line_cfg() != ESP_OK) {
        ESP_LOGE(TAG, "cannot apply detected rate %" PRIu32, detected_baud);
        g.active_baud = configured_baud;
        (void)apply_line_cfg();
        g.stats.init_no_sync++;
        return BUS_ERR_INIT;
    }

    rx_discard(); /* discard any old-rate UART interpretation */

    ESP_LOGI(TAG, "sync 55 selected %" PRIu32 " baud (configured %" PRIu32 ")",
             detected_baud, configured_baud);

    /* Both timestamps now name byte ends, so subtracting one character time
     * reports W1 between the address stop bit and sync start bit. */
    g.stats.last_w1_us = span_us(addr_end, sync_at, bt);

    /* W2: to key byte 1. W3: between the key bytes. */
    if (!rx_raw_byte(sync_at + W_TO_US(g.cfg.w2_max + slack), &kb1)) {
        ESP_LOGW(TAG, "key byte 1 missing");
        g.stats.init_no_keys++;
        return BUS_ERR_INIT;
    }
    kb1_at = now_us();
    g.stats.last_w2_us = span_us(sync_at, kb1_at, bt);

    if (!rx_raw_byte(kb1_at + W_TO_US(g.cfg.w3_max + slack), &kb2)) {
        ESP_LOGW(TAG, "key byte 2 missing");
        g.stats.init_no_keys++;
        return BUS_ERR_INIT;
    }
    kb2_at = now_us();
    g.stats.last_w3_us = span_us(kb1_at, kb2_at, bt);

    if (!kline_decode_keybytes(kb1, kb2, &keys)) {
        /* Not a pair this tester recognises. Reported, not refused: J2534
         * hands the key bytes to the application and lets it judge, and the
         * ELM327's AT KW0 exists to do the same. A front-end that wants them
         * checked reads them back and decides. */
        ESP_LOGW(TAG, "key bytes %02X %02X name no protocol", kb1, kb2);
        g.stats.init_bad_keys++;
    }

    ESP_LOGI(TAG, "key bytes %02X %02X - %s%s", kb1, kb2,
             kline_variant_name(keys.variant),
             keys.parity_ok ? "" : " (bad parity)");

    /* W4: before the inverted key byte 2 goes back. */
    if (g.cfg.five_baud_mod == BUS_FIVE_BAUD_STD_INIT ||
        g.cfg.five_baud_mod == BUS_FIVE_BAUD_INV_KB2) {
        delay_until_us(kb2_at + W_TO_US(g.cfg.w4_min));

        b = (uint8_t)~kb2;
        if (tx_once(&b, 1, false, false) == BUS_ERR_TX_FAILED) {
            return BUS_ERR_TX_FAILED;
        }
    }

    /* W4 again, this time for the ECU's inverted address. */
    if (g.cfg.five_baud_mod == BUS_FIVE_BAUD_STD_INIT ||
        g.cfg.five_baud_mod == BUS_FIVE_BAUD_INV_ADDR) {
        int64_t from = (g.cfg.five_baud_mod == BUS_FIVE_BAUD_STD_INIT)
                           ? g.tx_end_us
                           : kb2_at;

        if (!rx_raw_byte(from + W_TO_US(g.cfg.w4_max + slack), &b)) {
            ESP_LOGW(TAG, "no inverted address from the ECU");
            g.stats.init_no_addr++;
        } else {
            g.stats.last_w4_us = span_us(from, now_us(), bt);

            if ((uint8_t)~b != in->address) {
                ESP_LOGW(TAG, "ECU answered address %02X, asked for %02X",
                         (uint8_t)~b, in->address);
                g.stats.init_no_addr++;
            }
        }
    }

    g.keys = keys;
    g.link.variant = (uint8_t)keys.variant;
    g.link.key[0] = kb1;
    g.link.key[1] = kb2;
    g.link.key_code = keys.code;
    g.link.address = in->address;
    g.link.w1_us = g.stats.last_w1_us;
    g.link.w2_us = g.stats.last_w2_us;
    g.link.w3_us = g.stats.last_w3_us;
    g.link.w4_us = g.stats.last_w4_us;

    return 0;
}

/**
 * @brief The wake up pattern and StartCommunication of clause 5.1.5.3.
 *
 * TiniL low then high to TWuP, and the request straight after with no gap the
 * ECU could mistake for anything else. Both are specified to within a
 * millisecond, which is why they are timed against the clock rather than
 * counted in ticks.
 *
 * J2534 clause 7.4.6 allows the request to be any message the application
 * likes, or none at all - so @c msg_len of zero drives the pattern and stops.
 */
static int init_fast(bus_init_t *io) {
    uint8_t msg[BUS_INIT_MSG_MAX + 1];
    kline_frame_t f;
    kline_keybytes_t keys;
    int64_t t0;
    size_t hdr, n;
    int rc;

    if (io->msg_len > BUS_INIT_MSG_MAX) {
        return BUS_ERR_BAD_ARG;
    }

    /* Unlike a 5-baud init, fast init has no synchronisation byte and uses
     * exactly the rate selected by DATA_RATE (10400 for legislated OBD). */
    g.active_baud = g.cfg.baud;
    if (apply_line_cfg() != ESP_OK) {
        return BUS_ERR_INIT;
    }

    delay_until_us(g.last_bus_us + W_TO_US(g.cfg.tidle));
    rx_discard();

    ESP_LOGI(TAG, "fast init");

    tx_pin_to_gpio();
    t0 = now_us();
    gpio_set_level(PIN_KLINE_TX, 0);
    delay_until_us(t0 + W_TO_US(g.cfg.tinil));
    gpio_set_level(PIN_KLINE_TX, 1);
    delay_until_us(t0 + W_TO_US(g.cfg.twup));
    tx_pin_to_uart();

    /* The low pulse reads as a break, and the release as a framing error. */
    rx_discard();
    g.pending_status = 0;

    if (io->msg_len == 0) {
        /* Pattern only. The application will send its own request. */
        g.link.address = io->address;
        return 0;
    }

    n = frame_for_tx(io->msg, io->msg_len, 0, msg);
    if (n == 0) {
        return BUS_ERR_BAD_ARG;
    }

    rc = tx_message(msg, n, false, false);
    if (rc == BUS_ERR_TX_FAILED) {
        return rc;
    }

    if (!wait_frame(now_us() + W_TO_US(KLINE_FAST_RESP_MS), &f)) {
        ESP_LOGW(TAG, "no StartCommunication response");
        g.stats.init_no_keys++;
        return BUS_ERR_INIT;
    }

    if (f.len > sizeof(io->reply)) {
        return BUS_ERR_NO_SPACE;
    }
    memcpy(io->reply, f.data, f.len);
    io->reply_len = (uint8_t)f.len;

    hdr = kline_header_len(f.data[0]);
    if ((f.data[0] & 0x3Fu) == 0 && f.len > hdr) {
        hdr += 1; /* additional length byte */
    }

    /* Service id, two key bytes, checksum. */
    if (f.len < hdr + 4 || f.data[hdr] != KLINE_SVC_START_COMM_RESP) {
        ESP_LOGW(TAG, "StartCommunication answered with something else");
        g.stats.init_bad_keys++;
        return BUS_ERR_INIT;
    }

    if (!kline_decode_keybytes(f.data[hdr + 1], f.data[hdr + 2], &keys)) {
        ESP_LOGW(TAG, "key bytes %02X %02X name no protocol", f.data[hdr + 1],
                 f.data[hdr + 2]);
        g.stats.init_bad_keys++;
    }

    /* A vehicle that answers StartCommunication at all is speaking KWP,
     * whatever its key bytes claimed. */
    g.keys = keys;
    g.link.variant = KLINE_VARIANT_KWP;
    g.link.key[0] = keys.kb1;
    g.link.key[1] = keys.kb2;
    g.link.key_code = keys.code;
    g.link.address = io->address;

    ESP_LOGI(TAG, "fast init ok, key bytes %02X %02X", keys.kb1, keys.kb2);
    return 0;
}

/** @brief Adopt the periodic message the link's protocol calls for. */
static void link_established(void) {
    uint8_t def[BUS_PERIODIC_MAX];
    size_t n;

    g.link.connected = true;
    g.link.baud = g.active_baud;
    g.link.key[0] = g.keys.kb1;
    g.link.key[1] = g.keys.kb2;

    /* Clause 5.1.5.1 says the key bytes select Table 1 or Table 2; J2534
     * clause 7.4.5 says they must not. Off by default, so an application that
     * set its own timers keeps them. */
    if (g.cfg.timing_from_keybytes && g.keys.variant == KLINE_VARIANT_KWP) {
        kline_timing_t t;

        if (g.keys.extended_timing) {
            kline_timing_extended(&t);
        } else {
            kline_timing_normal(&t);
        }
        g.cfg.p1_max = P_FROM_MS(t.p1_max);
        g.cfg.p2_max = P_FROM_MS(t.p2_max);
        g.cfg.p3_min = P_FROM_MS(t.p3_min);
        g.cfg.p4_min = P_FROM_MS(t.p4_min);
    }

    /* The default periodic message follows the protocol that was negotiated:
     * a mode 01 request keeps any OBD ECU awake, a TesterPresent only keeps a
     * KWP one awake. A message set explicitly is left alone. */
    if (g.cfg.periodic_len == 0) {
        n = kline_default_wakeup((kline_variant_t)g.link.variant, def,
                                 sizeof(def));
        if (n) {
            memcpy(g.cfg.periodic, def, n);
            g.cfg.periodic_len = (uint8_t)n;
        }
    }

    g.stats.init_ok++;
}

static int do_init(kline_cmd_t which, bus_init_t *io) {
    int rc;

    g.stats.init_attempts++;
    memset(&g.link, 0, sizeof(g.link));
    memset(&g.keys, 0, sizeof(g.keys));
    g.echo = ECHO_UNKNOWN;

    rc = (which == CMD_FIVE_BAUD) ? init_5baud(io) : init_fast(io);

    if (rc != 0) {
        /* Clause 6.1: after a failed StartCommunication the tester waits W5
         * before trying again. Charging that to the failure rather than to
         * the next attempt keeps the caller from having to know about it. */
        g.last_bus_us = now_us();
        rx_discard();
        return rc;
    }

    io->key[0] = g.link.key[0];
    io->key[1] = g.link.key[1];

    link_established();
    rx_discard();
    g.pending_status = 0;
    return 0;
}

static int do_stop_comm(void) {
    uint8_t msg[8];
    size_t n = 0;

    if (!g.link.connected) {
        return 0;
    }

    if (g.link.variant == KLINE_VARIANT_KWP) {
        /* Clause 5.2. ISO 9141-2 has no equivalent; there the ECU simply
         * times out at P3max. */
        msg[n++] = 0xC1;
        msg[n++] = g.link.address ? g.link.address : KLINE_INIT_ADDR_OBD;
        msg[n++] = 0xF1;
        msg[n++] = KLINE_SVC_STOP_COMM;
        msg[n] = kline_checksum(msg, n);
        n++;

        tx_message(msg, n, true, false);
        /* Whatever it answers is the end of the session, not data. */
        g.suppress_until_us = now_us() + P_TO_US(g.cfg.p2_max);
    }

    memset(&g.link, 0, sizeof(g.link));
    memset(&g.keys, 0, sizeof(g.keys));
    return 0;
}

/* ------------------------------------------------------------------ *
 * Periodic messages
 * ------------------------------------------------------------------ */

static bool periodic_due(int64_t now) {
    return g.cfg.periodic_ms && g.cfg.periodic_len &&
           now - g.last_bus_us >= (int64_t)g.cfg.periodic_ms * 1000;
}

/**
 * @brief Hold the session open while nothing else is being asked of it.
 *
 * The ECU drops the link after P3max, five seconds by default. Answers to a
 * message the client never sent would be worse than the timeout this is
 * preventing, so by default everything that arrives in the window afterwards
 * is counted and thrown away - which is what a real ELM327 does with its idle
 * messages. A J2534 channel turns BUS_P_PERIODIC_QUIET off, because there
 * the application asked for the message and is entitled to its answer.
 */
static void send_periodic(void) {
    uint8_t msg[BUS_PERIODIC_MAX + 1];
    size_t n = frame_for_tx(g.cfg.periodic, g.cfg.periodic_len, 0, msg);

    if (n == 0) {
        return;
    }

    ESP_LOGD(TAG, "periodic message");
    tx_message(msg, n, true, false);

    g.stats.periodic_sent++;
    g.suppress_until_us =
        now_us() + P_TO_US(g.cfg.p2_max) + W_TO_US(KLINE_PERIODIC_QUIET_MS);
}

/* ------------------------------------------------------------------ *
 * The task
 * ------------------------------------------------------------------ */

static bool wait_frame(int64_t deadline_us, kline_frame_t *out) {
    return rx_service(deadline_us, out) == 1;
}

static void kline_task(void *arg) {
    (void)arg;

    for (;;) {
        kline_frame_t f;
        int64_t deadline;
        int r;

        if (__atomic_load_n(&g.stop_requested, __ATOMIC_ACQUIRE)) {
            do_stop_comm();
            xSemaphoreGive(g.exited);
            vTaskDelete(NULL);
            return;
        }

        if (periodic_due(now_us())) {
            send_periodic();
        }

        deadline = now_us() + 200000;
        if (g.cfg.periodic_ms && g.cfg.periodic_len) {
            int64_t at = g.last_bus_us + (int64_t)g.cfg.periodic_ms * 1000;
            if (at < deadline) {
                deadline = at;
            }
        }

        r = rx_service(deadline, &f);

        if (r == 1) {
            publish(&f);
            continue;
        }
        if (r == 0) {
            continue;
        }

        /* A command. The caller is blocked on cmd_done until we answer. */
        switch (g.cmd) {
        case CMD_TX:
            g.result = tx_message(g.req, g.req_len, true,
                                  (g.req_flags & BUS_TX_WAIT_P3_MIN_ONLY) != 0);
            break;
        case CMD_FIVE_BAUD:
        case CMD_FAST_INIT:
            g.result = do_init(g.cmd, &g.req_init);
            break;
        case CMD_STOP_COMM:
            g.result = do_stop_comm();
            break;
        }

        xSemaphoreGive(g.cmd_done);
    }
}

/** @brief Hand one command to the task and wait for its answer. */
static int run_cmd(kline_cmd_t c, TickType_t wait) {
    if (xQueueSend(g.cmd_q, &c, 0) != pdTRUE) {
        return BUS_ERR_NOT_READY;
    }

    if (xSemaphoreTake(g.cmd_done, wait) != pdTRUE) {
        ESP_LOGE(TAG, "command %d did not complete", (int)c);
        return BUS_ERR_TIMEOUT;
    }

    return g.result;
}

/* ------------------------------------------------------------------ *
 * Defaults
 * ------------------------------------------------------------------ */

/**
 * @brief The state the bus comes up in.
 *
 * Every timing value is J2534 Table 57's default, which is also ISO 14230-2's
 * Table 1 and Table 6 expressed in J2534's units. The policy flags underneath
 * them are argued for at their declarations.
 */
static void cfg_defaults(kline_cfg_t *c) {
    memset(c, 0, sizeof(*c));

    c->baud = KLINE_BAUD_DEFAULT;

    c->p1_max = 40;  /* 20 ms */
    c->p2_max = 100; /* 50 ms */
    c->p3_min = 110; /* 55 ms */
    c->p4_min = 10;  /* 5 ms */

    c->w0_min = 300;
    c->w1_max = 300;
    c->w2_max = 20;
    c->w3_max = 20;
    c->w4_min = 25;
    c->w4_max = 50;
    c->w5_min = 300;
    c->tidle = 300;
    c->tinil = 25;
    c->twup = 50;

    c->parity = BUS_PARITY_NONE;
    c->data_bits = BUS_DATA_BITS_8;
    c->five_baud_mod = BUS_FIVE_BAUD_STD_INIT;

    c->loopback = false;
    c->checksum_rx = true;
    c->checksum_tx = true;
    c->tx_retries = 0;
    c->periodic_ms = 0;
    c->periodic_quiet = true;
    c->timing_from_keybytes = false;
}

/* ------------------------------------------------------------------ *
 * The bus interface
 * ------------------------------------------------------------------ */

/* api_lock held; no worker may still use these resources. */
static esp_err_t kline_release(void) {
    gpio_set_level(PIN_KLINE_nSILENT, 0);
    if (g.uart_installed) {
        /* UART owns its event queue; stop it before deleting the queue set. */
        esp_err_t err = uart_driver_delete(KLINE_UART);
        if (err != ESP_OK)
            return err;
        g.uart_installed = false;
        g.uart_q = NULL;
    }
    if (g.set) {
        vQueueDelete(g.set);
        g.set = NULL;
    }
    if (g.cmd_q) {
        vQueueDelete(g.cmd_q);
        g.cmd_q = NULL;
    }
    if (g.rx_q) {
        vQueueDelete(g.rx_q);
        g.rx_q = NULL;
    }
    if (g.cmd_done) {
        vSemaphoreDelete(g.cmd_done);
        g.cmd_done = NULL;
    }
    if (g.exited) {
        vSemaphoreDelete(g.exited);
        g.exited = NULL;
    }
    memset(&g.link, 0, sizeof(g.link));
    return ESP_OK;
}

static esp_err_t kline_open(const bus_cfg_t *cfg) {
    /* Rate arrives later, through BUS_P_DATA_RATE. */
    (void)cfg;
    esp_err_t err;
    uart_config_t uart_config = {
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_XTAL,
        .baud_rate = KLINE_BAUD_DEFAULT,
    };

    if (g.started) {
        return ESP_OK;
    }

    if (!g.api_lock)
        g.api_lock = xSemaphoreCreateMutex();
    if (!g.data_lock)
        g.data_lock = xSemaphoreCreateMutex();
    if (!g.api_lock || !g.data_lock)
        return ESP_ERR_NO_MEM;

    xSemaphoreTake(g.api_lock, portMAX_DELAY);

    if (g.task || g.uart_installed) {
        xSemaphoreGive(g.api_lock);
        return ESP_ERR_INVALID_STATE;
    }
    __atomic_store_n(&g.stop_requested, false, __ATOMIC_RELEASE);

    cfg_defaults(&g.cfg);
    g.active_baud = g.cfg.baud;
    g.echo = ECHO_UNKNOWN;
    g.asm_len = 0;
    g.suppress_until_us = 0;
    g.pending_status = 0;
    g.tx_answered = true;
    memset(&g.link, 0, sizeof(g.link));
    memset(&g.keys, 0, sizeof(g.keys));

    err = uart_driver_install(KLINE_UART, KLINE_RX_RINGBUF, 0,
                              KLINE_UART_EVENTS, &g.uart_q, 0);
    if (err != ESP_OK)
        goto fail;
    g.uart_installed = true;
    err = uart_param_config(KLINE_UART, &uart_config);
    if (err != ESP_OK)
        goto fail;
    tx_pin_to_uart();

    /* Report a lull after two character times rather than after the default
     * ten, so the framer learns about the end of a message while its P1max
     * window is still open. */
    err = uart_set_rx_timeout(KLINE_UART, 2);
    if (err != ESP_OK)
        goto fail;

    g.cmd_q = xQueueCreate(1, sizeof(kline_cmd_t));
    g.rx_q = xQueueCreate(KLINE_RX_QUEUE, sizeof(kline_frame_t));
    g.cmd_done = xSemaphoreCreateBinary();
    g.exited = xSemaphoreCreateBinary();
    g.set = xQueueCreateSet(KLINE_UART_EVENTS + 1);

    if (!g.cmd_q || !g.rx_q || !g.cmd_done || !g.exited || !g.set) {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    if (xQueueAddToSet(g.uart_q, g.set) != pdPASS ||
        xQueueAddToSet(g.cmd_q, g.set) != pdPASS) {
        err = ESP_FAIL;
        goto fail;
    }

    /* Un-mute the transceiver. It is a low side driver with the line pulled
     * up, and the transmit pin idles high, so enabling it puts nothing on the
     * bus - it only makes this node able to pull the line down when asked. */
    gpio_set_level(PIN_KLINE_nSILENT, 1);

    g.last_bus_us = now_us();

    if (xTaskCreate(kline_task, "kline", KLINE_TASK_STACK, NULL,
                    KLINE_TASK_PRIO, &g.task) != pdPASS) {
        ESP_LOGE(TAG, "cannot start the receive task");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    g.started = true;
    ESP_LOGI(TAG, "up at %" PRIu32 " baud", g.active_baud);
    xSemaphoreGive(g.api_lock);
    return ESP_OK;

fail: {
    esp_err_t cleanup = kline_release();
    if (cleanup != ESP_OK) {
        ESP_LOGE(TAG, "startup cleanup incomplete: %s",
                 esp_err_to_name(cleanup));
    }
}
    xSemaphoreGive(g.api_lock);
    return err;
}

static esp_err_t kline_close(void) {
    if (!g.api_lock) {
        return ESP_OK;
    }

    xSemaphoreTake(g.api_lock, portMAX_DELAY);
    /*
     * ISO 14230-2 clause 5.2: end the session before the driver goes.
     *
     * This is the backstop, and it is the one that matters, because the
     * layers above do not all know they are about to drop the bus. A protocol
     * search that moves from KWP to CAN calls vif_bus_open(), which takes the
     * K-Line driver down underneath it; so does a client that simply goes
     * away. Neither is AT PC, which was the only path that said
     * StopCommunication.
     *
     * An ECU left in a diagnostic session answers no new initialisation - not
     * the 5 baud address, not a fast init wake-up - so the symptom is a
     * K-Line that worked once and never again: the first client's session was
     * still open, and every attempt after it was talking to an ECU that was
     * not listening for one.
     *
     * do_stop_comm() is a no-op on a link that was never established and
     * clears the link once it has run, so this costs nothing when there is no
     * session and cannot send a second StopCommunication after AT PC.
     */
    g.started = false;
    if (g.task) {
        /* Out-of-band: a timed-out command may still occupy the queue. */
        __atomic_store_n(&g.stop_requested, true, __ATOMIC_RELEASE);
        if (xSemaphoreTake(g.exited, pdMS_TO_TICKS(4000)) != pdTRUE) {
            ESP_LOGE(TAG, "worker did not exit; resources retained");
            xSemaphoreGive(g.api_lock);
            return ESP_ERR_TIMEOUT;
        }
        g.task = NULL;
    }

    esp_err_t err = kline_release();
    if (err != ESP_OK) {
        xSemaphoreGive(g.api_lock);
        return err;
    }

    ESP_LOGI(TAG, "down");
    xSemaphoreGive(g.api_lock);
    return ESP_OK;
}

static int kline_tx(const bus_msg_t *msg, uint32_t flags) {
    /* Addressing rides in the bytes on this bus, so msg->id is not
     * ours to look at. */
    const uint8_t *data = msg->data;
    size_t len = msg->len;

    size_t n;
    int rc;

    if (!data || len == 0) {
        return BUS_ERR_BAD_ARG;
    }
    if (!g.started) {
        return BUS_ERR_NOT_READY;
    }

    xSemaphoreTake(g.api_lock, portMAX_DELAY);

    n = frame_for_tx(data, len, flags, g.req);
    if (n == 0) {
        xSemaphoreGive(g.api_lock);
        return BUS_ERR_TOO_LONG;
    }

    g.req_len = n;
    g.req_flags = flags;

    /* The quiet time owed plus the message itself, with room for a
     * retransmission and the collision wait in front of it. */
    rc = run_cmd(CMD_TX, pdMS_TO_TICKS(3000));

    xSemaphoreGive(g.api_lock);
    return rc;
}

static int kline_rx(bus_msg_t *msg, TickType_t wait) {
    kline_frame_t f;

    if (!msg || !msg->data) {
        return BUS_ERR_BAD_ARG;
    }
    if (!g.started) {
        return BUS_ERR_NOT_READY;
    }

    if (xQueueReceive(g.rx_q, &f, wait) != pdTRUE) {
        return BUS_ERR_TIMEOUT;
    }

    if (f.len > msg->cap) {
        return BUS_ERR_NO_SPACE;
    }

    memcpy(msg->data, f.data, f.len);
    msg->len = f.len;
    msg->status = f.status;
    msg->timestamp_us = (uint32_t)f.end_us;

    return (int)f.len;
}

/* ------------------------------------------------------------------ *
 * Parameters
 * ------------------------------------------------------------------ */

/** @brief The parameter table, so get and set cannot drift apart.
 *
 *  Every entry is a field of g.cfg plus its limits; there is no per-parameter
 *  code to disagree with itself. */
static uint32_t *cfg_slot(bus_param_t p, uint32_t *lo, uint32_t *hi) {
    *lo = 0;
    *hi = 0xFFFF;

    switch (p) {
    case BUS_P_DATA_RATE:
        *lo = KLINE_BAUD_MIN;
        *hi = KLINE_BAUD_HW_MAX;
        return &g.cfg.baud;
    case BUS_P_P1_MAX:
        *lo = 1;
        return &g.cfg.p1_max;
    case BUS_P_P2_MAX:
        return &g.cfg.p2_max;
    case BUS_P_P3_MIN:
        return &g.cfg.p3_min;
    case BUS_P_P4_MIN:
        return &g.cfg.p4_min;
    case BUS_P_W0_MIN:
        return &g.cfg.w0_min;
    case BUS_P_W1_MAX:
        return &g.cfg.w1_max;
    case BUS_P_W2_MAX:
        return &g.cfg.w2_max;
    case BUS_P_W3_MAX:
        return &g.cfg.w3_max;
    case BUS_P_W4_MIN:
        return &g.cfg.w4_min;
    case BUS_P_W4_MAX:
        return &g.cfg.w4_max;
    case BUS_P_W5_MIN:
        return &g.cfg.w5_min;
    case BUS_P_TIDLE:
        return &g.cfg.tidle;
    case BUS_P_TINIL:
        return &g.cfg.tinil;
    case BUS_P_TWUP:
        return &g.cfg.twup;
    case BUS_P_PARITY:
        *hi = BUS_PARITY_EVEN;
        return &g.cfg.parity;
    case BUS_P_DATA_BITS:
        *hi = BUS_DATA_BITS_7;
        return &g.cfg.data_bits;
    case BUS_P_FIVE_BAUD_MOD:
        *hi = BUS_FIVE_BAUD_9141_STD;
        return &g.cfg.five_baud_mod;
    case BUS_P_TX_RETRIES:
        *hi = 8;
        return &g.cfg.tx_retries;
    case BUS_P_PERIODIC_MS:
        *hi = 0xFFFFFF;
        return &g.cfg.periodic_ms;
    default:
        return NULL;
    }
}

/** @brief The flags, kept apart because a bool is not a uint32_t slot. */
static bool *cfg_flag(bus_param_t p) {
    switch (p) {
    case BUS_P_LOOPBACK:
        return &g.cfg.loopback;
    case BUS_P_CHECKSUM_RX:
        return &g.cfg.checksum_rx;
    case BUS_P_CHECKSUM_TX:
        return &g.cfg.checksum_tx;
    case BUS_P_PERIODIC_QUIET:
        return &g.cfg.periodic_quiet;
    case BUS_P_TIMING_FROM_KEYBYTES:
        return &g.cfg.timing_from_keybytes;
    default:
        return NULL;
    }
}

static int kline_set_param(bus_param_t p, uint32_t value) {
    uint32_t lo, hi, *slot;
    bool *flag;
    int rc = 0;

    if (!g.started) {
        return BUS_ERR_NOT_READY;
    }

    xSemaphoreTake(g.api_lock, portMAX_DELAY);
    LOCK();

    flag = cfg_flag(p);
    if (flag) {
        *flag = value != 0;
        goto done;
    }

    slot = cfg_slot(p, &lo, &hi);
    if (!slot) {
        rc = BUS_ERR_UNSUPPORTED;
        goto done;
    }
    if (value < lo || value > hi) {
        rc = BUS_ERR_BAD_ARG;
        goto done;
    }

    *slot = value;

    /* The three that reach the peripheral rather than only the timers. */
    if (p == BUS_P_DATA_RATE || p == BUS_P_PARITY || p == BUS_P_DATA_BITS) {
        if (p == BUS_P_DATA_RATE) {
            g.active_baud = value;
        }
        if (apply_line_cfg() != ESP_OK) {
            rc = BUS_ERR_BAD_ARG;
        }
    }

done:
    UNLOCK();
    xSemaphoreGive(g.api_lock);
    return rc;
}

static int kline_get_param(bus_param_t p, uint32_t *out) {
    uint32_t lo, hi, *slot;
    bool *flag;
    int rc = 0;

    if (!out) {
        return BUS_ERR_BAD_ARG;
    }
    if (!g.started) {
        return BUS_ERR_NOT_READY;
    }

    LOCK();

    flag = cfg_flag(p);
    if (flag) {
        *out = *flag ? 1u : 0u;
    } else if ((slot = cfg_slot(p, &lo, &hi)) != NULL) {
        *out = *slot;
    } else {
        rc = BUS_ERR_UNSUPPORTED;
    }

    UNLOCK();
    return rc;
}

static int kline_ioctl(bus_ioctl_t id, const void *in, void *out) {
    int rc;

    if (!g.started) {
        return BUS_ERR_NOT_READY;
    }

    switch (id) {
    case BUS_IOCTL_FIVE_BAUD_INIT:
    case BUS_IOCTL_FAST_INIT: {
        bus_init_t io;

        if (in) {
            io = *(const bus_init_t *)in;
        } else {
            memset(&io, 0, sizeof(io));
        }
        if (io.address == 0) {
            io.address = KLINE_INIT_ADDR_OBD;
        }

        xSemaphoreTake(g.api_lock, portMAX_DELAY);
        g.req_init = io;
        /* Two seconds of address word, W0 before it, W1 after, and margin. */
        rc = run_cmd(id == BUS_IOCTL_FIVE_BAUD_INIT ? CMD_FIVE_BAUD
                                                    : CMD_FAST_INIT,
                     pdMS_TO_TICKS(8000));
        if (out) {
            *(bus_init_t *)out = g.req_init;
        }
        xSemaphoreGive(g.api_lock);
        return rc;
    }

    case BUS_IOCTL_STOP_COMM:
        xSemaphoreTake(g.api_lock, portMAX_DELAY);
        rc = run_cmd(CMD_STOP_COMM, pdMS_TO_TICKS(2000));
        xSemaphoreGive(g.api_lock);
        return rc;

    case BUS_IOCTL_GET_LINK:
        if (!out) {
            return BUS_ERR_BAD_ARG;
        }
        LOCK();
        *(bus_link_t *)out = g.link;
        UNLOCK();
        return 0;

    case BUS_IOCTL_ASSUME_LINK: {
        kline_variant_t v =
            in ? *(const kline_variant_t *)in : KLINE_VARIANT_UNKNOWN;
        uint8_t def[BUS_PERIODIC_MAX];
        size_t n;

        LOCK();
        memset(&g.link, 0, sizeof(g.link));
        memset(&g.keys, 0, sizeof(g.keys));
        g.keys.variant = v;
        g.link.variant = (uint8_t)v;
        g.link.address = KLINE_INIT_ADDR_OBD;
        g.link.baud = g.active_baud;
        g.link.connected = true;
        n = kline_default_wakeup(v, def, sizeof(def));
        if (n) {
            memcpy(g.cfg.periodic, def, n);
            g.cfg.periodic_len = (uint8_t)n;
        }
        UNLOCK();
        return 0;
    }

    case BUS_IOCTL_SET_PERIODIC: {
        const bus_msg_t *m = in;

        LOCK();
        if (!m || m->len == 0) {
            /* Back to the default for whatever protocol is up. */
            g.cfg.periodic_len = (uint8_t)kline_default_wakeup(
                (kline_variant_t)g.link.variant, g.cfg.periodic,
                sizeof(g.cfg.periodic));
        } else if (m->len > BUS_PERIODIC_MAX) {
            UNLOCK();
            return BUS_ERR_TOO_LONG;
        } else {
            memcpy(g.cfg.periodic, m->data, m->len);
            g.cfg.periodic_len = (uint8_t)m->len;
        }
        UNLOCK();
        return 0;
    }

    case BUS_IOCTL_CLEAR_PERIODIC:
        LOCK();
        g.cfg.periodic_ms = 0;
        UNLOCK();
        return 0;

    case BUS_IOCTL_CLEAR_RX_QUEUE:
        if (g.rx_q) {
            xQueueReset(g.rx_q);
        }
        return 0;

    case BUS_IOCTL_CLEAR_TX_QUEUE:
        /* Transmissions are synchronous here - kline_tx() does not return
         * until the message is on the wire - so there is never a queue to
         * clear. Answered rather than refused, because J2534 requires
         * STATUS_NOERROR when the queue is already empty. */
        return 0;

    default:
        return BUS_ERR_UNSUPPORTED;
    }
}

static void kline_get_stats(bus_stats_t *out) {
    if (!out) {
        return;
    }

    LOCK();
    *out = g.stats;
    UNLOCK();
}

static void kline_reset_stats(void) {
    LOCK();
    memset(&g.stats, 0, sizeof(g.stats));
    UNLOCK();
}

const bus_ops_t kline_bus_ops = {
    .name = "K-Line",
    .open = kline_open,
    .close = kline_close,
    .send = kline_tx,
    .recv = kline_rx,
    .set_param = kline_set_param,
    .get_param = kline_get_param,
    .ioctl = kline_ioctl,
    .get_stats = kline_get_stats,
    .reset_stats = kline_reset_stats,
};
