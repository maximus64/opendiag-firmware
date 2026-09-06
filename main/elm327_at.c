/* SPDX-License-Identifier: GPL-3.0-only */
#include "elm327_at.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "ble_uart.h"
#include "can_xfer.h"
#include "comm_iface.h"
#include "common.h"
#include "j1850_pwm.h"
#include "j1850_vpw.h"
#include "kline_codec.h"
#include "sdkconfig.h"
#include "timer.h"
#include "utility.h"
#include "vif.h"

#define DEFAULT_TIMEOUT_MS 200

#define TAG "ELM327_AT"

#define ELM_IDENTIFY "ELM327 v2.3"

/*
 * The address a scan tool answers to, PP 06 on a real ELM327.
 *
 * It is the third byte of every header this layer sends, and - unless AT RA
 * or AT SR overrides it - one of the target addresses a reply may carry for
 * this tool to accept it.
 */
#define ELM327_TESTER_ADDRESS 0xF1

/* J1850 carries 12 payload bytes at most, so 15 with the header. */
#define J1850_FRAME_CAP 15

/*
 * Largest K-Line message this layer will carry.
 *
 * ISO 9141-2 never exceeds eleven, but ISO 14230 allows 260 and the
 * manufacturer specific services that use the additional length byte make
 * real use of the room. This is a compromise: enough for every diagnostic
 * exchange an OBD-II client performs and for a comfortable margin past it,
 * without putting a quarter of a kilobyte of message on a session task's
 * stack twice over. A longer message is reported rather than truncated.
 */
#define KLINE_FRAME_CAP 64

/* Buffers sized for the largest of the byte buses, plus the terminator. */
#define ELM327_BUS_CAP KLINE_FRAME_CAP

/**
 * @brief State of the incoming command line.
 *
 * Kept separate from the rest of the instance so the line assembly can be
 * driven a byte at a time, whatever hands the bytes over.
 */
typedef struct {
    char cmdbuf[32];
    char lastcmd[32];
    int cmdidx;

    /** The line being read is longer than cmdbuf. Its remainder is discarded
     *  and it is answered once, at the carriage return. */
    bool overflow;

    /** A reset ran part way through a chunk of input. Whatever else that
     *  chunk held reached us before the reset finished, so it is gone with
     *  everything else the reset threw away. */
    bool reset_discard;
} elm327_line_t;

/**
 * @brief AT IFR0 to IFR6: when this node acknowledges a J1850 message.
 *
 * The two halves of the command differ only in whether they also apply while
 * monitoring, which this firmware has no mode for yet, so IFR4..6 land on the
 * same three values as IFR0..2.
 */
typedef enum {
    ELM327_IFR_OFF = 0, /**< AT IFR0 / IFR4: never acknowledge. */
    ELM327_IFR_AUTO,    /**< AT IFR1 / IFR5: acknowledge if the K bit asks. */
    ELM327_IFR_ON,      /**< AT IFR2 / IFR6: acknowledge regardless. */
} elm327_ifr_mode_t;

/** The AT-settable knobs. Reset to defaults whenever the front-end starts. */
typedef struct {
    int timeout;
    bool echo;
    bool show_header;
    bool linefeed;
    uint8_t current_protocol;
    bool auto_search;

    /**
     * AT SP Ah and AT TP Ah: search the rest when this protocol goes
     * unanswered.
     *
     * "if the protocol that is tried should fail to initialize, the ELM327
     * will then automatically sequence through the other protocols". It is
     * separate from @p auto_search, which records how the protocol in use was
     * arrived at rather than what to do when it stops working.
     */
    bool auto_fallback;

    /**
     * AT SS: search in the J1978 order rather than the fast one.
     *
     * "SAE standard J1978 specifies a protocol search order that scan tools
     * should use... In order to provide a faster search, the ELM327 does not
     * normally follow this order, but it will if you command it to."
     */
    bool std_search_order;

    bool can_auto_format;

    /**
     * AT D0 / AT D1: show the CAN data length code with the header.
     *
     * "Standard CAN (ISO 15765-4) OBD requires that all messages have 8 data
     * bytes, so displaying the number of data bytes (the DLC) is not normally
     * very useful. When experimenting with other protocols, however, it may
     * be useful to be able to see what the data lengths are."
     *
     * Off by default, which is what PP 29 selects on a factory ELM327.
     */
    bool can_show_dlc;

    uint32_t header_id;
    bool hex_spacing;

    /** AT R0 / R1. Off sends the request and does not wait for an answer. */
    bool responses;

    /**
     * AT AT0 / AT1 / AT2: how the reply window is chosen.
     *
     * AT0 always waits the full AT ST window. AT1, the ELM327 default,
     * shortens it towards what this vehicle has actually been answering in.
     * AT2 is "a little more aggressive". See elm327_reply_window().
     */
    uint8_t adaptive_timing;

    /** AT TA: the address this tool answers to, third header byte by default.
     */
    uint8_t tester_address;

    /**
     * AT AR versus AT RA / AT SR. Automatic derives the addresses a reply may
     * be sent to from the header; a fixed one accepts nothing else.
     */
    bool auto_receive;
    uint8_t recv_address;

    /** AT FT hh: only accept frames from this transmitter. */
    bool filter_tx;
    uint8_t tx_filter;

    /** AT IFR0..6 and AT IFR H / S, protocols 1 and 2 only. */
    elm327_ifr_mode_t ifr_mode;
    bool ifr_from_source;

    /* ---- K-Line, protocols 3, 4 and 5 ---- */

    /** AT IIA: the 5 baud address word, used exactly as given. */
    uint8_t iso_init_address;

    /** AT KW0 / KW1. Off still performs the handshake and still needs the
     *  key bytes to arrive; it just stops caring what they say. */
    bool key_word_check;

    /** AT IB10 and friends. 10400 for anything legislated. */
    uint32_t iso_baud;

    /** AT SW, in milliseconds. Zero stops the wakeup messages. */
    uint32_t wakeup_ms;

    /** AT WM. Empty means the default for whichever protocol was negotiated. */
    uint8_t wakeup_msg[BUS_PERIODIC_MAX];
    uint8_t wakeup_len;
} elm327_settings_t;

/**
 * @brief One ELM327 instance: a grammar, and the client it speaks to.
 *
 * There is one of these per data link, not one per firmware. USB CDC 0 and
 * BLE each get their own, because two clients sharing a single set of echo,
 * header and protocol settings - and racing over which of them a reply is
 * addressed to - is a bug rather than a feature.
 *
 * @p session is the hardware half. Every bus, pin and analog reading goes
 * through it, so a claim is attributed to this client.
 */
typedef struct {
    bool live;
    vif_session_t *session;
    RingbufHandle_t tx_ringbuf;
    elm327_settings_t settings;
    elm327_line_t line;

    /**
     * What adaptive timing has learned about this vehicle, milliseconds.
     *
     * Zero until a reply has been timed, and returned to zero whenever an
     * exchange draws nothing at all - see elm327_note_latency(). It lives on
     * the instance rather than in the settings because it is an observation
     * rather than a setting: AT Z clears it, but so does changing protocol,
     * and no client can set it directly.
     */
    uint16_t adaptive_ms;

    /**
     * The shortest window this vehicle has already proved is not enough.
     *
     * Without it the algorithm cannot learn from a timeout, only from a
     * reply, and on a module whose response latency varies it oscillates:
     * a run of quick answers pulls the window down to the floor, the next
     * slow answer misses it, the timeout throws away everything learned, the
     * full window then succeeds with a quick answer, and the window is pulled
     * straight back down to where it just failed. Every other request fails,
     * indefinitely, and the counters show a perfectly healthy bus.
     *
     * That is not hypothetical - it is what the J1850 PWM module on the bench
     * does. Its reply lands anywhere between 5.7 and 21.1 ms, and
     * ELM327_ADAPTIVE_MIN_MS is 20, so roughly one reply in ten arrives just
     * after the window the previous nine taught the algorithm to use.
     *
     * So a window that expired empty raises this floor above itself, and no
     * amount of subsequent good luck lets the window back under it. Cleared
     * with adaptive_ms, which means AT Z and any change of protocol.
     */
    uint16_t adaptive_floor_ms;

    /**
     * How long after the request the most recent reply arrived, milliseconds.
     *
     * Written by whichever transfer path is running and read once by the
     * dispatcher. It lives here rather than being returned because the CAN
     * path alone has five exits, and a measurement that has to be repeated at
     * each of them is a measurement that will be missed at one.
     */
    uint32_t last_latency_ms;

    /**
     * The protocol an automatic search should try before any other.
     *
     * Set by AT SP 0 to whatever was working at the time. A search that
     * starts with the protocol already established finishes in one attempt
     * and, because selecting a byte bus that is already open does not cycle
     * its driver, without dropping the session on it.
     */
    uint8_t search_first;

    /**
     * True while the automatic search is trying protocols in turn.
     *
     * The datasheet is explicit that a bus initiation performed during a
     * search reports nothing - the client asked which protocol the vehicle
     * speaks, not to watch four of them fail.
     */
    bool in_search;
} elm327_ctx_t;

/**
 * @brief The instance pool.
 *
 * Static, because nothing here is allocated at run time and a session lives
 * for the lifetime of the firmware. Two links carry ELM327 today; a third
 * would need only a bigger pool.
 */
#define ELM327_MAX_INSTANCES 2

static elm327_ctx_t g_elm327[ELM327_MAX_INSTANCES];

enum elm327_protocol {
    ELM327_PROTO_AUTO = 0,      // Automatic
    ELM327_PROTO_J1850_PWM = 1, // SAE J1850 PWM (41.6 kbaud)
    ELM327_PROTO_J1850_VPW = 2, // SAE J1850 VPW (10.4 kbaud)
    ELM327_PROTO_ISO9141 = 3,   // ISO 9141-2 (5 baud init, 10.4 kbaud)
    ELM327_PROTO_ISO14230_KWP_5BAUD =
        4, // ISO 14230-4 KWP (5 baud init, 10.4 kbaud)
    ELM327_PROTO_ISO14230_KWP_FAST =
        5,                           // ISO 14230-4 KWP (fast init, 10.4 kbaud)
    ELM327_PROTO_CAN_11BIT_500K = 6, // ISO 15765-4 CAN (11 bit ID, 500 kbaud)
    ELM327_PROTO_CAN_29BIT_500K = 7, // ISO 15765-4 CAN (29 bit ID, 500 kbaud)
    ELM327_PROTO_CAN_11BIT_250K = 8, // ISO 15765-4 CAN (11 bit ID, 250 kbaud)
    ELM327_PROTO_CAN_29BIT_250K = 9, // ISO 15765-4 CAN (29 bit ID, 250 kbaud)
    ELM327_PROTO_CAN_SAE_J1939 = 10, // SAE J1939 CAN (29 bit ID, 250* kbaud)
    ELM327_PROTO_CAN_USER1 = 11,     // USER1 CAN (11* bit ID, 125* kbaud)
    ELM327_PROTO_CAN_USER2 = 12,     // USER2 CAN (11* bit ID, 50* kbaud)
};

static int elm327_set_protocol(elm327_ctx_t *e, int proto);
static void elm327_uart_flush(elm327_ctx_t *e);

/* The K-Line helpers live down beside the transfer path they belong to, but
 * the AT command table above needs them. */
static bool elm327_is_kline(const elm327_ctx_t *e);
static bool elm327_is_kwp(const elm327_ctx_t *e);
static bool elm327_kline_link(elm327_ctx_t *e, bus_link_t *out);
static int elm327_kline_init(elm327_ctx_t *e, kline_init_mode_t mode);
static void elm327_kline_reconcile(elm327_ctx_t *e);
static void elm327_kline_apply_wakeup(elm327_ctx_t *e);

static size_t elm327_uart_read_bytes(elm327_ctx_t *e, uint8_t *buffer,
                                     size_t len, TickType_t ticks_to_wait) {
    return comm_port_read(vif_session_port(e->session), buffer, len,
                          ticks_to_wait);
}

static size_t elm327_uart_send_bytes(elm327_ctx_t *e, const uint8_t *buffer,
                                     size_t size) {
    /*
     * Never wait for room. The only thing that drains e->tx_ringbuf is
     * elm327_uart_flush(e), which runs in this same task, so blocking here
     * would be waiting on ourselves.
     *
     * A long multi frame response is formatted one frame at a time and can
     * easily exceed the buffer, so when it fills, push what has accumulated
     * out to the client and carry on. Short responses still go out in a
     * single flush at the end of the command.
     */
    if (xRingbufferSend(e->tx_ringbuf, buffer, size, 0) == pdTRUE) {
        return size;
    }

    elm327_uart_flush(e);

    if (xRingbufferSend(e->tx_ringbuf, buffer, size, 0) == pdTRUE) {
        return size;
    }

    /* Only reachable if one write is larger than the whole buffer */
    ESP_LOGE(TAG, "tx buffer cannot take %u bytes", (unsigned)size);
    return 0;
}

static void elm327_uart_flush(elm327_ctx_t *e) {
    size_t size = 0;
    uint8_t *data;

    while (1) {
        data = (uint8_t *)xRingbufferReceiveUpTo(e->tx_ringbuf, &size, 0, 64);
        if (!data) {
            break;
        }

        /* Answer whoever sent the request, not every attached client */
        comm_port_write(vif_session_port(e->session), data, size);

        vRingbufferReturnItem(e->tx_ringbuf, data);
    }

    /* One push per flush, rather than per 64 byte chunk. A short response is
     * flushed once, at the end of the command; a long one is flushed again
     * each time it fills the buffer. */
    comm_port_flush(vif_session_port(e->session));
}

/**
 * @brief Write @p buffer out, expanding every CR to CR LF when AT L1 is set.
 *
 * Linefeeds are a display option, so they belong at the one place every byte
 * this layer emits passes through rather than in each of the several dozen
 * literals that end in a carriage return. The echo goes through here too,
 * which is what a real ELM327 does: with L1 on, the command you typed comes
 * back with a linefeed after it, and so does the prompt.
 */
static void elm327_uart_send_text(elm327_ctx_t *e, const uint8_t *buffer,
                                  size_t size) {
    size_t start = 0;

    if (!e->settings.linefeed) {
        elm327_uart_send_bytes(e, buffer, size);
        return;
    }

    for (size_t i = 0; i < size; i++) {
        if (buffer[i] != '\r') {
            continue;
        }

        elm327_uart_send_bytes(e, &buffer[start], i + 1 - start);
        elm327_uart_send_bytes(e, (const uint8_t *)"\n", 1);
        start = i + 1;
    }

    if (start < size) {
        elm327_uart_send_bytes(e, &buffer[start], size - start);
    }
}

static void elm327_send_string(elm327_ctx_t *e, const char *s) {
    elm327_uart_send_text(e, (const uint8_t *)s, strlen(s));
}

/**
 * @brief Put the command grammar back to its power-on defaults.
 *
 * Settings only. Installing this front-end must not disturb the session's
 * claims: a client that raised a programming voltage, switched to SLCAN to
 * flash, and came back expects to find the voltage still up.
 */
static void elm327_load_defaults(elm327_ctx_t *e) {
    e->settings.timeout = DEFAULT_TIMEOUT_MS;
    e->settings.echo = true;
    /* Headers off is the documented power-on state - the AT command summary
     * marks H0 with the default asterisk, and the datasheet says the extra
     * bytes are "not normally shown". A client that never sends AT H0 because
     * it is relying on that default was being handed three header bytes and a
     * checksum in front of every reply. */
    e->settings.show_header = 0;
    e->settings.linefeed = 0;
    e->settings.current_protocol = ELM327_PROTO_AUTO;
    e->settings.auto_search = false;
    e->settings.auto_fallback = false;
    e->settings.std_search_order = false;
    e->settings.can_auto_format = true;
    e->settings.can_show_dlc = false;
    e->settings.header_id = 0;
    e->settings.hex_spacing = true;
    e->settings.responses = true;
    e->settings.adaptive_timing = 1;
    e->settings.tester_address = ELM327_TESTER_ADDRESS;
    e->settings.auto_receive = true;
    e->settings.recv_address = 0;
    e->settings.filter_tx = false;
    e->settings.tx_filter = 0;
    e->settings.ifr_mode = ELM327_IFR_AUTO;
    e->settings.ifr_from_source = false;
    e->adaptive_ms = 0;
    e->adaptive_floor_ms = 0;
    e->last_latency_ms = 0;
    e->search_first = 0;
    e->settings.iso_init_address = KLINE_INIT_ADDR_OBD;
    e->settings.key_word_check = true;
    e->settings.iso_baud = KLINE_BAUD_DEFAULT;
    /* AT SW defaults to $92, "a nominal delay of 3 seconds". */
    e->settings.wakeup_ms = 0x92u * 20u;
    e->settings.wakeup_len = 0;
}

/**
 * @brief AT Z / AT D: defaults, bus down, and this client's pins released.
 *
 * Only the claims held by this session are dropped. A reset here used to call
 * board_hs_ls_reset_state(), which cut the programming voltage for every
 * client on the adapter, whoever had raised it.
 */
static esp_err_t elm327_reset(elm327_ctx_t *e) {
    ESP_LOGI(TAG, "Reset setting to defaults");

    esp_err_t err = vif_bus_release_all(e->session);
    vif_pin_release_all(e->session);
    if (err != ESP_OK)
        return err;

    elm327_set_protocol(e, ELM327_PROTO_AUTO);
    elm327_load_defaults(e);
    return ESP_OK;
}

#define VOLTAGE_PIN_OFF 0xffffffff
#define VOLTAGE_SHORT_TO_GROUND 0xfffffffe

/**
 * @brief AT PROGV: drive an OBD-II pin, or release it.
 *
 * The two sentinel "voltages" are an ELM327 quirk, so they are translated
 * here; the one-high-side-one-low-side rule they run into lives in vif.
 */
static int at_set_programing_voltage(elm327_ctx_t *e, const char *arg) {
    int rc;
    int8_t pin;
    uint32_t voltage = 0;
    vif_pin_mode_t mode;

    pin = hex_char_to_int(arg[0]);
    if (pin < 0) {
        return -1;
    }

    rc = hex_string_to_u32_be(&arg[1], 8, &voltage);
    if (rc) {
        return -2;
    }

    ESP_LOGI(TAG, "Set Programming Voltage: pin=%d voltage=0x%08" PRIx32, pin,
             voltage);

    if (voltage == VOLTAGE_PIN_OFF) {
        mode = VIF_PIN_OFF;
    } else if (voltage == VOLTAGE_SHORT_TO_GROUND) {
        mode = VIF_PIN_GROUND;
    } else {
        mode = VIF_PIN_VOLTAGE;
    }

    if (vif_pin_set(e->session, pin, mode, voltage) != ESP_OK) {
        return -3;
    }

    return 0;
}

/** @brief Parse a two digit hex argument. Returns -1 when it is not one. */
static int at_parse_byte(const char *arg, uint8_t *out) {
    int8_t hi, lo;

    if (strlen(arg) != 2) {
        return -1;
    }

    hi = hex_char_to_int(arg[0]);
    lo = hex_char_to_int(arg[1]);
    if (hi < 0 || lo < 0) {
        return -1;
    }

    *out = (uint8_t)(hi << 4 | lo);
    return 0;
}

static int at_set_header(elm327_ctx_t *e, const char *arg) {
    int rc;
    size_t len = strlen(arg);
    uint32_t id = 0;

    /* Three digits for an 11 bit CAN ID, six for the three header bytes every
     * other protocol uses, eight for a 29 bit CAN ID with its priority byte.
     * Anything else is a typo, and one the ELM327 answers with a '?'. */
    if (len != 3 && len != 6 && len != 8) {
        return -1;
    }

    rc = hex_string_to_u32_be(arg, len, &id);
    if (rc) {
        return -2;
    }

    ESP_LOGI(TAG, "Set Header to 0x%08" PRIx32, id);

    e->settings.header_id = id;

    return 0;
}

/**
 * @brief Tell the bus which of the two paradigms it is serving.
 *
 * The drivers default to what the standards require of a J2534 Pass-Thru
 * interface, because that is the stricter reading wherever the two disagree.
 * An ELM327 client expects something different in three places, and this is
 * where those three are asked for rather than assumed - see bus.h.
 *
 * Every call is allowed to fail: a bus that has no such parameter answers
 * BUS_ERR_UNSUPPORTED, which is the whole point of a shared interface.
 */
static void elm327_apply_bus_policy(elm327_ctx_t *e, vif_bus_t bus,
                                    uint8_t proto) {
    /* ISO 14230-2 clause 6.6 and SAE J1850 both have the sender retry a
     * corrupted transmission. J2534 forbids it; an ELM327 client is used to
     * the standards' own behaviour and has no other recovery. */
    vif_bus_param_set(e->session, bus, BUS_P_TX_RETRIES, 1);

    /* "You will never see the responses to these." */
    vif_bus_param_set(e->session, bus, BUS_P_PERIODIC_QUIET, 1);

    /* One reply per request. A J1850 module retransmits an unacknowledged
     * answer two or three times over, and no OBD-II client expects to be
     * shown all of them.
     *
     * The window is per bus rather than one number, because it is a statement
     * about air time: a retransmission arrives an inter-frame separation
     * behind the frame it repeats, and at 10.4 kbps that is four times as far
     * behind as it is at 41.6. The PWM figure applied to VPW is not
     * conservative, it is inert - no VPW repeat can arrive inside five
     * milliseconds, because the frame itself takes longer than that to send.
     * Buses with no retransmission of their own are left at zero. */
    vif_bus_param_set(e->session, bus, BUS_P_DUPLICATE_MS,
                      proto == ELM327_PROTO_J1850_VPW   ? J1850_VPW_DUPLICATE_MS
                      : proto == ELM327_PROTO_J1850_PWM ? J1850_PWM_DUPLICATE_MS
                                                        : 0);

    /* A client that did not ask to see its own requests should not have to
     * filter them out of the replies. */
    vif_bus_param_set(e->session, bus, BUS_P_LOOPBACK, 0);
}

/**
 * @brief The bus a protocol runs on, what it needs to start, and its header.
 *
 * One table, because two kinds of caller need it: the switch below, which
 * opens the bus, and everything afterwards, which has to name the bus it is
 * already talking to. VIF_BUS_NONE for the protocol numbers with no driver
 * behind them yet - KWP, J1939, the two USER slots.
 *
 * @param cfg    Filled with what the bus needs to start. Optional.
 * @param header Filled with the protocol's request header. Optional.
 */
static vif_bus_t elm327_bus_for_protocol(int proto, vif_bus_cfg_t *cfg,
                                         uint32_t *header) {
    vif_bus_t bus = VIF_BUS_NONE;
    uint32_t header_id = 0;
    uint32_t bitrate = 0;

    switch (proto) {
    case ELM327_PROTO_J1850_PWM:
        /* SAE J1850 PWM (41.6 kbaud) */
        bus = VIF_BUS_J1850_PWM;
        header_id = 0x616af1;
        break;
    case ELM327_PROTO_J1850_VPW:
        /* SAE J1850 VPW (10.4 kbaud) */
        bus = VIF_BUS_J1850_VPW;
        header_id = 0x686af1;
        break;
    case ELM327_PROTO_ISO9141:
        /* ISO 9141-2 (5 baud init, 10.4 kbaud). The CARB request header, and
         * with 68 6A F1 01 00 the only mode 01 PID 00 request ISO 9141-2
         * accepts. */
        bus = VIF_BUS_KLINE;
        header_id = 0x686af1;
        break;
    case ELM327_PROTO_ISO14230_KWP_5BAUD:
    case ELM327_PROTO_ISO14230_KWP_FAST:
        /* ISO 14230-4 KWP. C2 33 F1: functional addressing, length in the
         * format byte, every emission related ECU. ISO 14230-4 clause 4.4
         * allows nothing else for legislated OBD, and the length nibble is
         * recomputed for each request - see elm327_build_bus_request(). */
        bus = VIF_BUS_KLINE;
        header_id = 0xc233f1;
        break;
    case ELM327_PROTO_CAN_11BIT_250K:
        bus = VIF_BUS_CAN;
        bitrate = 250000;
        header_id = 0x7DF;
        break;
    case ELM327_PROTO_CAN_11BIT_500K:
        bus = VIF_BUS_CAN;
        bitrate = 500000;
        header_id = 0x7DF;
        break;
    case ELM327_PROTO_CAN_29BIT_250K:
        bus = VIF_BUS_CAN;
        bitrate = 250000;
        header_id = 0x18DB33F1;
        break;
    case ELM327_PROTO_CAN_29BIT_500K:
        bus = VIF_BUS_CAN;
        bitrate = 500000;
        header_id = 0x18DB33F1;
        break;
    default:
        break;
    }

    if (cfg) {
        cfg->bitrate = bitrate;
    }
    if (header) {
        *header = header_id;
    }

    return bus;
}

/** @brief The bus the protocol in use runs on, or VIF_BUS_NONE. */
static vif_bus_t elm327_bus(const elm327_ctx_t *e) {
    return elm327_bus_for_protocol(e->settings.current_protocol, NULL, NULL);
}

/**
 * @brief AT SP: select the protocol, which is also the bus this client holds.
 *
 * An ELM327 client talks to one bus at a time, so whatever this session held
 * is released before the new one opens: vif on its own would only take down a
 * bus sharing the new one's wiring, and leaving the others up would hold
 * hardware this client has stopped using. Protocol numbers with no driver
 * behind them leave the session with no bus at all, as before.
 *
 * @return 0, or -1 when the bus could not be opened, which now includes
 *         another session already holding it.
 */
static int elm327_set_protocol(elm327_ctx_t *e, int proto) {
    vif_bus_cfg_t cfg = {0};
    vif_bus_t bus = VIF_BUS_NONE;
    uint32_t header_id = 0;

    if (proto == ELM327_PROTO_AUTO) {
        /* Automatic is a search in its own right, so there is nothing left
         * for an AT SP Ah marker to fall back from. Cleared ahead of the
         * unchanged-protocol shortcut below so that AT PC and a search that
         * found nothing both leave it off. */
        e->settings.auto_fallback = false;
    }

    bus = elm327_bus_for_protocol(proto, &cfg, &header_id);
    if (e->settings.current_protocol == proto &&
        (bus == VIF_BUS_NONE || vif_bus_is_open(e->session, bus))) {
        ESP_LOGW(TAG, "Already using protocol: %d", proto);
        return 0;
    }

    if (e->settings.current_protocol == ELM327_PROTO_AUTO) {
        /* leaving automatic: any search result is stale */
        e->settings.auto_search = false;
    }

    if (bus == VIF_BUS_NONE) {
        if (proto == ELM327_PROTO_AUTO) {
            /* "the SP 0 command sets the protocol to automatic". Automatic is
             * not another protocol to switch to, it is an admission that the
             * right one is not known - so the one that is already working is
             * the best guess available and the bus stays up while the search
             * confirms it.
             *
             * Tearing it down here is what made AT SP 0 expensive: a K-Line
             * session that had just cost two and a half seconds to establish
             * was dropped, the search then started at CAN, and the same
             * session had to be built again from nothing. The datasheet's own
             * justification for not searching in J1978 order is that the
             * ELM327 searches in whatever order is fastest.
             *
             * A client that wants the bus released has AT PC, which is what
             * "Protocol Close" is for. */
            e->search_first = e->settings.current_protocol;
        } else {
            if (vif_bus_release_all(e->session) != ESP_OK)
                return -1;
            e->search_first = 0;
        }

        e->settings.current_protocol = proto;
        return 0;
    }

    /* Several protocols share one bus - ISO 9141-2 and both KWP variants all
     * run over the K-Line transceiver - and cycling the driver to move
     * between them would throw away the session that was just established on
     * it. Only the numbering and the header change. CAN is excluded because
     * two CAN protocols on the same wire can still want different bit rates. */
    if (bus != VIF_BUS_CAN && vif_bus_is_open(e->session, bus)) {
        e->settings.header_id = header_id;
        e->settings.current_protocol = proto;
        return 0;
    }

    e->adaptive_ms = 0;
    e->adaptive_floor_ms = 0;

    if (vif_bus_release_all(e->session) != ESP_OK)
        return -1;

    if (vif_bus_open(e->session, bus, &cfg) != ESP_OK) {
        /* Startup failed; VIF may retain a claim until cleanup succeeds. */
        ESP_LOGE(TAG, "Protocol %d unavailable", proto);
        e->settings.current_protocol = ELM327_PROTO_AUTO;
        return -1;
    }

    elm327_apply_bus_policy(e, bus, (uint8_t)proto);

    /* A different bus answers in a different time. What was learned about the
     * last one says nothing about this one. */
    e->adaptive_ms = 0;
    e->adaptive_floor_ms = 0;

    e->settings.header_id = header_id;
    e->settings.current_protocol = proto;
    return 0;
}

/** @brief The name AT DP reports a protocol under, as the datasheet spells it.
 */
static const char *elm327_protocol_name(uint8_t proto) {
    switch (proto) {
    case ELM327_PROTO_AUTO:
        return "AUTO";
    case ELM327_PROTO_J1850_PWM:
        return "SAE J1850 PWM";
    case ELM327_PROTO_J1850_VPW:
        return "SAE J1850 VPW";
    case ELM327_PROTO_ISO9141:
        return "ISO 9141-2";
    case ELM327_PROTO_ISO14230_KWP_5BAUD:
        return "ISO 14230-4 (KWP 5BAUD)";
    case ELM327_PROTO_ISO14230_KWP_FAST:
        return "ISO 14230-4 (KWP FAST)";
    case ELM327_PROTO_CAN_11BIT_500K:
        return "ISO 15765-4 (CAN 11/500)";
    case ELM327_PROTO_CAN_29BIT_500K:
        return "ISO 15765-4 (CAN 29/500)";
    case ELM327_PROTO_CAN_11BIT_250K:
        return "ISO 15765-4 (CAN 11/250)";
    case ELM327_PROTO_CAN_29BIT_250K:
        return "ISO 15765-4 (CAN 29/250)";
    case ELM327_PROTO_CAN_SAE_J1939:
        return "SAE J1939 (CAN 29/250)";
    case ELM327_PROTO_CAN_USER1:
        return "USER1 (CAN 11/125)";
    case ELM327_PROTO_CAN_USER2:
        return "USER2 (CAN 11/50)";
    default:
        return "UNKNOWN";
    }
}

/**
 * @brief AT IFR0 to IFR6 and AT IFR H / S, protocols 1 and 2.
 *
 * Recorded, and deliberately not pushed down to the bus driver. The driver
 * keeps its in-frame response switched off on this hardware because it cannot
 * put the byte on the wire inside the window the standard gives it - Tp4 to
 * Tp5, 48 to 63 us after the frame's last rising edge - and answering late
 * measured worse than not answering at all. j1850_pwm_codec.h has the
 * arithmetic and the bench results.
 *
 * So the setting is accepted rather than rejected, because a client that sets
 * it should not have to treat a documented command as unrecognised, and the
 * behaviour it asks for is the behaviour a real ELM327 would also fail to
 * deliver on a link that could not carry the response in time. What the
 * adapter does about the retransmissions this costs is J1850_SETTLE_MS.
 *
 * @return 0 when @p arg named a mode, -1 otherwise.
 */
static int at_set_ifr(elm327_ctx_t *e, const char *arg) {
    if (strcmp(arg, "H") == 0) {
        e->settings.ifr_from_source = false;
        return 0;
    }
    if (strcmp(arg, "S") == 0) {
        e->settings.ifr_from_source = true;
        return 0;
    }

    if (strlen(arg) != 1) {
        return -1;
    }

    /* IFR4..6 differ from IFR0..2 only in applying while monitoring as well,
     * and there is no monitoring mode here yet, so they fold together. */
    switch (arg[0]) {
    case '0':
    case '4':
        e->settings.ifr_mode = ELM327_IFR_OFF;
        return 0;
    case '1':
    case '5':
        e->settings.ifr_mode = ELM327_IFR_AUTO;
        return 0;
    case '2':
    case '6':
        e->settings.ifr_mode = ELM327_IFR_ON;
        return 0;
    default:
        return -1;
    }
}

/**
 * @brief Should AT DP and AT DPN prefix the protocol with "A"?
 *
 * Two ways to earn it, and the datasheet's examples cover both: a protocol a
 * search arrived at on its own, and one named with AT SP Ah, which "allows
 * you to choose a starting (default) protocol, while still retaining the
 * ability to automatically search". Either way the number the client is being
 * shown is not the only one this session might end up using, and that is what
 * the prefix tells it.
 */
static bool elm327_protocol_is_auto(const elm327_ctx_t *e) {
    return e->settings.auto_search || e->settings.auto_fallback;
}

/**
 * @brief Parse the argument shared by AT SP and AT TP.
 *
 * Four forms, all of them documented:
 *
 * - "h"  - protocol h, and only protocol h. "Failure to initiate a connection
 *          in this situation will result in a response such as 'BUS INIT:
 *          ...ERROR', and no other protocols will be attempted."
 * - "Ah" - protocol h first, then a search if it goes unanswered.
 * - "hA" - the same thing. "Note that the 'A' can come before or after the h,
 *          so AT SP A3 can also be entered as AT SP 3A."
 * - "00" - protocol zero, spelled out. AT SP 00 exists only to write the zero
 *          to EEPROM, which this firmware does not have, so it lands on the
 *          same place as AT SP 0.
 *
 * @p arg has already been stripped of spaces and upper cased by
 * elm327_feed_byte(), so "AT SP A6" and "atspa6" both arrive here as "A6".
 *
 * @return 0 on success, -1 if @p arg is not one of those forms.
 */
static int elm327_parse_proto_arg(const char *arg, uint8_t *proto,
                                  bool *fallback) {
    size_t len = strlen(arg);
    int8_t digit;

    if (len == 1) {
        /* A bare "A" is protocol 0xA, SAE J1939 - the marker form needs a
         * protocol after it to mark. */
        digit = hex_char_to_int(arg[0]);
        if (digit < 0) {
            return -1;
        }
        *proto = (uint8_t)digit;
        *fallback = false;
        return 0;
    }

    if (len != 2) {
        return -1;
    }

    if (arg[0] == 'A') {
        /* "AA" reads as the marker plus protocol A. Reading it the other way
         * round gives the same protocol and no fallback, which is what a
         * client that wrote the marker was asking to avoid. */
        digit = hex_char_to_int(arg[1]);
    } else if (arg[1] == 'A') {
        digit = hex_char_to_int(arg[0]);
    } else if (arg[0] == '0' && arg[1] == '0') {
        *proto = ELM327_PROTO_AUTO;
        *fallback = false;
        return 0;
    } else {
        return -1;
    }

    if (digit < 0) {
        return -1;
    }

    *proto = (uint8_t)digit;
    *fallback = true;
    return 0;
}

/** @brief AT SP and AT TP, which differ only in a memory this firmware has not
 * got. */
static void elm327_select_protocol_cmd(elm327_ctx_t *e, const char *arg) {
    uint8_t proto;
    bool fallback;

    if (elm327_parse_proto_arg(arg, &proto, &fallback) != 0 ||
        proto > ELM327_PROTO_CAN_USER2) {
        ESP_LOGE(TAG, "Bad protocol argument: %s", arg);
        elm327_send_string(e, "?\r");
        return;
    }

    ESP_LOGI(TAG, "Set protocol: %d%s", proto,
             fallback ? " (auto fallback)" : "");

    if (elm327_set_protocol(e, proto) != 0) {
        elm327_send_string(e, "?\r");
        return;
    }

    /* After elm327_set_protocol(), which clears it on the way out of
     * automatic. Protocol 0 is a search already, so the marker adds nothing
     * there and would only make AT DP claim a fallback that is the whole
     * mechanism. */
    e->settings.auto_fallback = fallback && proto != ELM327_PROTO_AUTO;

    elm327_send_string(e, "OK\r");
}

static void elm327_at_command_handler(elm327_ctx_t *e, const char *cmd) {
    int rc;

    if (strcmp(cmd, "Z") == 0) {
        ESP_LOGI(TAG, "Reset all command");

        if (elm327_reset(e) != ESP_OK) {
            elm327_send_string(e, "ERROR\r");
            return;
        }

        /*
         * A reset takes about a second and loses every character that arrives
         * while it runs, because the real chip is reading them into a buffer
         * it is about to clear. Apps rely on that far more than they know:
         * this one opens a connection with "ATZ<CR>ATE0<CR>" followed by a
         * row of bare carriage returns, meaning to sweep the line clean. A
         * bare carriage return repeats the last command, so on an adapter
         * that keeps them those become eight more ATE0s and eight replies
         * nobody is waiting for - and the client reads every answer for the
         * rest of the session against a command eight places later.
         *
         * Two halves, because input on its way here sits in two places. What
         * has not been read off the transport yet is drained below. What has
         * already been read is in the chunk this very command arrived in,
         * being fed a byte at a time, out of reach of any drain - so the flag
         * tells elm327_fe_feed() to abandon the rest of it.
         */
        e->line.reset_discard = true;

        Timer timer;
        Timer_init(&timer);
        Timer_start(&timer, 1000);

        while (!Timer_is_expired(&timer)) {
            uint8_t val;
            elm327_uart_read_bytes(e, (uint8_t *)&val, 1, pdMS_TO_TICKS(100));
        }

        ESP_LOGI(TAG, "Reset DONE");
        elm327_send_string(e, "\r\r" ELM_IDENTIFY "\r");
    } else if (strcmp(cmd, "D") == 0) {
        if (elm327_reset(e) != ESP_OK) {
            elm327_send_string(e, "ERROR\r");
            return;
        }
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "D0") == 0 || strcmp(cmd, "D1") == 0) {
        /* Exact matches, and deliberately just below AT D: "D" on its own is
         * a different command entirely - set everything to defaults - and a
         * prefix match on either would swallow the other. */
        e->settings.can_show_dlc = (cmd[1] == '1');
        ESP_LOGI(TAG, "CAN DLC display %s",
                 e->settings.can_show_dlc ? "on" : "off");
        elm327_send_string(e, "OK\r");
    } else if (strncmp(cmd, "SP", 2) == 0 || strncmp(cmd, "TP", 2) == 0) {
        /* "This command is identical to the SP command, except that the
         * protocol that you select is not immediately saved in internal
         * EEPROM memory". There is no EEPROM behind either of them here, so
         * the two really are the same command. */
        elm327_select_protocol_cmd(e, &cmd[2]);
    } else if (strncmp(cmd, "ST", 2) == 0) {
        uint8_t val;

        if (at_parse_byte(&cmd[2], &val)) {
            ESP_LOGE(TAG, "Set timeout invalid hex");
            elm327_send_string(e, "?\r");
        } else {
            /* "Note that a value of 00 does not result in a time of 0 msec -
             * it will restore the timer to the default value." */
            e->settings.timeout = val ? val * 4 : DEFAULT_TIMEOUT_MS;
            ESP_LOGI(TAG, "Timeout set to %d ms", e->settings.timeout);
            elm327_send_string(e, "OK\r");
        }
    } else if (strcmp(cmd, "SS") == 0) {
        ESP_LOGI(TAG, "Standard J1978 search order");
        e->settings.std_search_order = true;
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "DPN") == 0) {
        /* One hex digit, so protocols A, B and C report as letters rather
         * than as 10, 11 and 12. */
        char pn[6];
        ESP_LOGI(TAG, "Describe the Protocol by Number");

        if (elm327_protocol_is_auto(e)) {
            snprintf(pn, sizeof(pn), "A%X\r", e->settings.current_protocol);
        } else {
            snprintf(pn, sizeof(pn), "%X\r", e->settings.current_protocol);
        }
        elm327_send_string(e, pn);
    } else if (strcmp(cmd, "DP") == 0) {
        char dp[40];

        if (elm327_protocol_is_auto(e)) {
            snprintf(dp, sizeof(dp), "AUTO, %s\r",
                     elm327_protocol_name(e->settings.current_protocol));
        } else {
            snprintf(dp, sizeof(dp), "%s\r",
                     elm327_protocol_name(e->settings.current_protocol));
        }
        elm327_send_string(e, dp);
    } else if (strcmp(cmd, "E0") == 0) {
        ESP_LOGI(TAG, "Echo off");
        e->settings.echo = 0;
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "E1") == 0) {
        ESP_LOGI(TAG, "Echo on");
        e->settings.echo = 1;
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "S0") == 0) {
        ESP_LOGI(TAG, "Spacing off");
        e->settings.hex_spacing = false;
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "S1") == 0) {
        ESP_LOGI(TAG, "Spacing on");
        e->settings.hex_spacing = true;
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "H0") == 0) {
        ESP_LOGI(TAG, "Headers OFF");
        e->settings.show_header = 0;
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "H1") == 0) {
        ESP_LOGI(TAG, "Headers ON");
        e->settings.show_header = 1;
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "I") == 0) {
        elm327_send_string(e, ELM_IDENTIFY "\r");
    } else if (strcmp(cmd, "L0") == 0) {
        ESP_LOGI(TAG, "Linefeed OFF");
        e->settings.linefeed = 0;
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "L1") == 0) {
        ESP_LOGI(TAG, "Linefeed ON");
        e->settings.linefeed = 1;
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "M0") == 0) {
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "M1") == 0) {
        // TODO: for Memory on, we support to remeber the last use protocol
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "CAF0") == 0) {
        ESP_LOGI(TAG, "CAN auto format OFF");
        e->settings.can_auto_format = false;
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "CAF1") == 0) {
        ESP_LOGI(TAG, "CAN auto format ON");
        e->settings.can_auto_format = true;
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "RV") == 0) {
        char voltstr[8];
        float volt = vif_vbatt_mv() / 1000.0f;
        snprintf(voltstr, sizeof(voltstr), "%0.1fV\r", volt);
        elm327_send_string(e, voltstr);
    } else if (strncmp(cmd, "PROGV", 5) == 0) {
        rc = at_set_programing_voltage(e, &cmd[5]);
        if (rc) {
            elm327_send_string(e, "?\r");
        } else {
            elm327_send_string(e, "OK\r");
        }
    } else if (strncmp(cmd, "SH", 2) == 0) {
        rc = at_set_header(e, &cmd[2]);
        if (rc) {
            elm327_send_string(e, "?\r");
        } else {
            elm327_send_string(e, "OK\r");
        }
    } else if (strcmp(cmd, "@1") == 0) {
        const esp_app_desc_t *desc = esp_app_get_description();
        elm327_send_string(e, OPENDIAG_MANUFACTURE " " OPENDIAG_PRODUCT " (");
        elm327_send_string(e, desc->version);
        elm327_send_string(e, ")\r");
    } else if (strcmp(cmd, "R0") == 0) {
        ESP_LOGI(TAG, "Responses off");
        e->settings.responses = false;
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "R1") == 0) {
        ESP_LOGI(TAG, "Responses on");
        e->settings.responses = true;
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "AT0") == 0 || strcmp(cmd, "AT1") == 0 ||
               strcmp(cmd, "AT2") == 0) {
        e->settings.adaptive_timing = (uint8_t)(cmd[2] - '0');
        /* The two modes weigh the same measurement differently, so what was
         * learned under one is not what the other would have concluded. */
        e->adaptive_ms = 0;
        e->adaptive_floor_ms = 0;
        ESP_LOGI(TAG, "Adaptive timing %s",
                 e->settings.adaptive_timing ? "on" : "off");
        elm327_send_string(e, "OK\r");
    } else if (strcmp(cmd, "AR") == 0) {
        ESP_LOGI(TAG, "Auto receive address");
        e->settings.auto_receive = true;
        elm327_send_string(e, "OK\r");
    } else if (strncmp(cmd, "RA", 2) == 0 || strncmp(cmd, "SR", 2) == 0) {
        /* The datasheet says these two are interchangeable, so they are. */
        uint8_t addr;

        if (at_parse_byte(&cmd[2], &addr)) {
            elm327_send_string(e, "?\r");
        } else {
            ESP_LOGI(TAG, "Receive address fixed at 0x%02x", addr);
            e->settings.recv_address = addr;
            e->settings.auto_receive = false;
            elm327_send_string(e, "OK\r");
        }
    } else if (strncmp(cmd, "TA", 2) == 0) {
        uint8_t addr;

        if (at_parse_byte(&cmd[2], &addr)) {
            elm327_send_string(e, "?\r");
        } else {
            ESP_LOGI(TAG, "Tester address 0x%02x", addr);
            e->settings.tester_address = addr;
            elm327_send_string(e, "OK\r");
        }
    } else if (strcmp(cmd, "FT") == 0) {
        ESP_LOGI(TAG, "Transmitter filter off");
        e->settings.filter_tx = false;
        elm327_send_string(e, "OK\r");
    } else if (strncmp(cmd, "FT", 2) == 0) {
        uint8_t addr;

        if (at_parse_byte(&cmd[2], &addr)) {
            elm327_send_string(e, "?\r");
        } else {
            ESP_LOGI(TAG, "Only accept frames from 0x%02x", addr);
            e->settings.tx_filter = addr;
            e->settings.filter_tx = true;
            elm327_send_string(e, "OK\r");
        }
    } else if (strncmp(cmd, "IFR", 3) == 0) {
        if (at_set_ifr(e, &cmd[3])) {
            elm327_send_string(e, "?\r");
        } else {
            elm327_send_string(e, "OK\r");
        }
    } else if (strcmp(cmd, "WS") == 0) {
        /* A warm start restores exactly what AT Z does, without the wait or
         * the lamp test, so a client resetting in software is not made to
         * spend a second on it. */
        if (elm327_reset(e) != ESP_OK) {
            elm327_send_string(e, "ERROR\r");
            return;
        }
        elm327_send_string(e, "\r\r" ELM_IDENTIFY "\r");
    } else if (strcmp(cmd, "PC") == 0) {
        ESP_LOGI(TAG, "Protocol close");
        /* Clause 5.2: a KWP ECU is entitled to be told the session is over
         * rather than left to time out at P3max, and the only moment this
         * layer knows that is here, before the driver goes down. */
        if (elm327_is_kline(e)) {
            vif_bus_ioctl(e->session, VIF_BUS_KLINE, BUS_IOCTL_STOP_COMM, NULL,
                          NULL);
        }
        /* Explicitly, because selecting automatic no longer does it: AT SP 0
         * keeps a working bus so the search can confirm it in one attempt,
         * where AT PC is the command whose entire job is to let go. */
        if (vif_bus_release_all(e->session) != ESP_OK) {
            elm327_send_string(e, "ERROR\r");
            return;
        }
        elm327_set_protocol(e, ELM327_PROTO_AUTO);
        e->search_first = 0;

        elm327_send_string(e, "OK\r");
    } else if (strncmp(cmd, "IIA", 3) == 0) {
        uint8_t addr;

        /* "The full eight bit value is used exactly as provided - no changes
         * are made to it (ie no adding of parity bits, etc.)" */
        if (at_parse_byte(&cmd[3], &addr)) {
            elm327_send_string(e, "?\r");
        } else {
            ESP_LOGI(TAG, "ISO init address 0x%02x", addr);
            e->settings.iso_init_address = addr;
            elm327_send_string(e, "OK\r");
        }
    } else if (strcmp(cmd, "KW") == 0) {
        bus_link_t link;
        char out[16];

        /* The two bytes as they arrived. A client uses these to tell an
         * ISO 9141-2 vehicle from an ISO 14230-4 one without trusting our
         * classification of them. */
        if (elm327_is_kline(e) && elm327_kline_link(e, &link)) {
            snprintf(out, sizeof(out), "%02X %02X\r", link.key[0], link.key[1]);
            elm327_send_string(e, out);
        } else {
            elm327_send_string(e, "?\r");
        }
    } else if (strcmp(cmd, "KW0") == 0 || strcmp(cmd, "KW1") == 0) {
        e->settings.key_word_check = cmd[2] == '1';
        ESP_LOGI(TAG, "Key word checking %s",
                 e->settings.key_word_check ? "on" : "off");
        elm327_send_string(e, "OK\r");
    } else if (strncmp(cmd, "IB", 2) == 0) {
        /* The five rates the datasheet offers. The two above 10400 are past
         * what ISO 14230-2 allows and exist for the manufacturer specific
         * systems that use them anyway. */
        uint32_t baud = 0;

        if (strcmp(&cmd[2], "10") == 0)
            baud = 10400;
        else if (strcmp(&cmd[2], "12") == 0)
            baud = 12500;
        else if (strcmp(&cmd[2], "15") == 0)
            baud = 15625;
        else if (strcmp(&cmd[2], "48") == 0)
            baud = 4800;
        else if (strcmp(&cmd[2], "96") == 0)
            baud = 9600;

        if (baud == 0) {
            elm327_send_string(e, "?\r");
        } else {
            ESP_LOGI(TAG, "ISO baud rate %" PRIu32, baud);
            e->settings.iso_baud = baud;
            if (elm327_is_kline(e)) {
                vif_bus_param_set(e->session, VIF_BUS_KLINE, BUS_P_DATA_RATE,
                                  baud);
            }
            elm327_send_string(e, "OK\r");
        }
    } else if (strncmp(cmd, "SW", 2) == 0) {
        uint8_t val;

        if (at_parse_byte(&cmd[2], &val)) {
            elm327_send_string(e, "?\r");
        } else {
            /* "any hexadecimal value from 00 to FF" in 20 ms steps, and zero
             * stops them without forgetting the interval. */
            if (val) {
                e->settings.wakeup_ms = (uint32_t)val * 20u;
                elm327_kline_apply_wakeup(e);
            } else {
                /* "will stop the periodic (wakeup) messages... will not change
                 * a prior setting for the time between wakeup messages". */
                vif_bus_param_set(e->session, elm327_bus(e), BUS_P_PERIODIC_MS,
                                  0);
            }
            ESP_LOGI(TAG, "Wakeup interval %" PRIu32 " ms",
                     val ? e->settings.wakeup_ms : 0);
            elm327_send_string(e, "OK\r");
        }
    } else if (strncmp(cmd, "WM", 2) == 0) {
        uint8_t msg[BUS_PERIODIC_MAX];
        size_t arglen = strlen(&cmd[2]);
        int n;

        /* Spaces are already gone by the time a command reaches here, so
         * "AT WM 68 6A F1 01 00" and "AT WM686AF10100" are the same thing. */
        n = (arglen && (arglen % 2) == 0)
                ? hex_string_to_u8_array(&cmd[2], arglen, msg, sizeof(msg))
                : -1;

        if (n <= 0) {
            elm327_send_string(e, "?\r");
        } else {
            memcpy(e->settings.wakeup_msg, msg, (size_t)n);
            e->settings.wakeup_len = (uint8_t)n;
            elm327_kline_apply_wakeup(e);
            ESP_LOGI(TAG, "Wakeup message set, %d bytes", n);
            elm327_send_string(e, "OK\r");
        }
    } else if (strcmp(cmd, "SI") == 0) {
        /* "Protocol 3 or 4 must be selected to use the AT SI command, or an
         * error will result." */
        if (e->settings.current_protocol != ELM327_PROTO_ISO9141 &&
            e->settings.current_protocol != ELM327_PROTO_ISO14230_KWP_5BAUD) {
            elm327_send_string(e, "?\r");
        } else if (elm327_kline_init(e, KLINE_INIT_5BAUD) == 0) {
            elm327_kline_reconcile(e);
        }
    } else if (strcmp(cmd, "FI") == 0) {
        /* "Protocol 5 must be selected to use the AT FI command." */
        if (e->settings.current_protocol != ELM327_PROTO_ISO14230_KWP_FAST) {
            elm327_send_string(e, "?\r");
        } else {
            elm327_kline_init(e, KLINE_INIT_FAST);
        }
    } else if (strcmp(cmd, "BI") == 0) {
        /* "allows an OBD protocol to be made active without requiring any
         * sort of initiation or handshaking to occur." Nothing goes on the
         * wire; only our idea of the link changes. */
        if (!elm327_is_kline(e)) {
            elm327_send_string(e, "?\r");
        } else {
            kline_variant_t v =
                elm327_is_kwp(e) ? KLINE_VARIANT_KWP : KLINE_VARIANT_ISO9141;

            vif_bus_ioctl(e->session, VIF_BUS_KLINE, BUS_IOCTL_ASSUME_LINK, &v,
                          NULL);
            elm327_kline_apply_wakeup(e);
            elm327_send_string(e, "OK\r");
        }
    } else if (strcmp(cmd, "IA") == 0) {
        /* A protocol is active once a handshake succeeded or AT BI said so.
         * For the buses with no handshake, having one open is the whole of
         * what "active" can mean. */
        bool active = elm327_is_kline(e)
                          ? elm327_kline_link(e, NULL)
                          : vif_bus_is_open(e->session, elm327_bus(e));

        elm327_send_string(e, active ? "Y\r" : "N\r");
    } else if (strcmp(cmd, "AL") == 0 || strcmp(cmd, "NL") == 0) {
        /* Long message support is decided by the frame cap of the bus in use,
         * so there is nothing to switch; accepted so a client that asks is
         * not turned away. */
        elm327_send_string(e, "OK\r");
    } else {
        elm327_send_string(e, "?\r");
    }
}

/** @brief Drop whatever the bus queued before this request went out. */
static void read_all_can(elm327_ctx_t *e) {
    struct can_frame frame;

    while (1) {
        int ret = can_frame_recv(e->session, &frame, 0);
        if (ret != 0)
            return;
    }
}

/** @brief Append @p len bytes as hex, spaced when AT S1 is on. */
static char *elm327_append_hex(const elm327_ctx_t *e, char *p,
                               const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        p += sprintf(p, e->settings.hex_spacing ? "%02X " : "%02X", data[i]);
    }

    return p;
}

/**
 * @brief Print one received CAN frame the way the client asked to see it.
 *
 * Three shapes, chosen by what the client has turned on rather than by the
 * frame:
 *
 * - AT CAF0, or AT H1: as received. Every byte the frame carried, PCI byte
 *   included, behind the identifier when headers are on. "Note that turning
 *   the display of headers on (with AT H1) will override some of the CAF1
 *   formatting of the received data, so that the received bytes will appear
 *   much like in the CAF0 mode (ie. as received)."
 *
 * - AT CAF1 with headers off, which is the power-on state of both: the
 *   transport is the adapter's business, not the client's. "the formatting
 *   (PCI) bytes will be ... removed when receiving ... Also, with formatting
 *   on, any extra (unused) data bytes that are received in the frame will be
 *   removed." A single frame is therefore just its data, which is what lets a
 *   client "continue to issue OBD requests (01 00, etc.) as usual, without
 *   regard to the extra bytes that CAN diagnostics systems require".
 *
 * - The same, for a reply too long for one frame. The datasheet's worked 0902
 *   example is the entire specification:
 *
 *       >0902
 *       014
 *       0: 49 02 01 31 44 34
 *       1: 47 50 30 30 52 35 35
 *       2: 42 31 32 33 34 35 36
 *
 *   the total length once, then each frame behind its sequence number. Note
 *   what is not happening: nothing is reassembled or reordered here. "CAN
 *   systems add this single hex digit (it goes from 0 to F then repeats), to
 *   aid in reassembling" - the client assembles, and the sequence numbers are
 *   printed so that it can, because on a bus where two ECUs answer at once
 *   the frames genuinely do interleave.
 *
 * Printing the identifier unconditionally - which is what this did - meant a
 * client that never sent AT H1, and so was entitled to the documented
 * default, had to strip three bytes of address and a PCI byte off the front
 * of every reply before it could read one.
 */
static void elm327_print_can_frame(elm327_ctx_t *e, uint32_t id, uint8_t dlc,
                                   const uint8_t *data) {
    /* Longest line this can produce is a 29 bit identifier, a space, and
     * eight spaced bytes: 9 + 24, plus the carriage return and terminator. */
    char outstr[48];
    char *p = &outstr[0];
    uint8_t type;

    if (dlc > 8) {
        dlc = 8;
    }

    if (e->settings.can_auto_format && !e->settings.show_header && dlc >= 1) {
        type = data[0] >> 4;

        if (type == 0) {
            /* Single frame. The low nibble is the count, and it is what
             * separates data from the padding a CAN frame is made up to
             * length with. */
            uint8_t n = data[0] & 0x0F;

            if (n > (uint8_t)(dlc - 1)) {
                n = dlc - 1;
            }
            p = elm327_append_hex(e, p, &data[1], n);
        } else if (type == 1 && dlc >= 2) {
            /* First frame: the total to come, then this frame's six bytes
             * as segment zero. */
            char len_line[8];

            snprintf(len_line, sizeof(len_line), "%03X\r",
                     (unsigned)(((data[0] & 0x0F) << 8) | data[1]));
            elm327_send_string(e, len_line);

            p += sprintf(p, e->settings.hex_spacing ? "0: " : "0:");
            p = elm327_append_hex(e, p, &data[2], dlc - 2);
        } else if (type == 2) {
            p += sprintf(
                p, e->settings.hex_spacing ? "%X: " : "%X:", data[0] & 0x0F);
            p = elm327_append_hex(e, p, &data[1], dlc - 1);
        } else {
            /* Flow control, or a PCI byte that means nothing. Shown as it
             * arrived rather than swallowed: a line that produces no output
             * would leave the client with a bare prompt and no way to tell
             * that from a reply it failed to read. */
            p = elm327_append_hex(e, p, data, dlc);
        }
    } else {
        if (e->settings.show_header) {
            if (id & CAN_EFF_FLAG) {
                p += sprintf(p,
                             e->settings.hex_spacing ? "%08" PRIX32 " "
                                                     : "%08" PRIX32,
                             id & CAN_EFF_MASK);
            } else {
                p += sprintf(p,
                             e->settings.hex_spacing ? "%03" PRIX32 " "
                                                     : "%03" PRIX32,
                             id & CAN_SFF_MASK);
            }

            /* AT D1: "the single DLC digit will appear between the ID
             * (header) bytes and the data bytes". Inside the headers-on
             * branch because the datasheet ties the two together - "the
             * headers must also be on in order to see this digit" - and
             * because a bare digit in front of the data, with no identifier
             * to separate it from, would be indistinguishable from a data
             * byte. */
            if (e->settings.can_show_dlc) {
                p += sprintf(p, e->settings.hex_spacing ? "%X " : "%X", dlc);
            }
        }

        p = elm327_append_hex(e, p, data, dlc);
    }

    *p++ = '\r';
    *p = '\0';

    elm327_send_string(e, outstr);
}

/* ------------------------------------------------------------------ *
 * Adaptive timing - AT AT0, AT1, AT2
 * ------------------------------------------------------------------ *
 *
 * The datasheet describes the problem exactly: "The ELM327 sends a request
 * then waits up to 200 msec for a reply... After each reply has been
 * received, the ELM327 must wait to see if any more replies are coming."
 * With a vehicle answering in 50 ms and a 200 ms window, most of every
 * exchange is spent waiting for a reply that already arrived.
 *
 * Adaptive timing "automatically sets the timeout value for you, to a value
 * that is based on the actual response times that your vehicle is responding
 * in", and "always uses your AT ST hh setting as the maximum setting, and
 * will never choose one which is longer".
 *
 * The datasheet's worked example is what fixes the margin: a J1850 VPW
 * vehicle answering at 4 ms and 58 ms is said to settle "likely to a value in
 * the range of 90 msec" - a little over one and a half times the slowest
 * response. AT2 is documented only as "a little more aggressive", so it takes
 * a smaller margin over the same measurement.
 */

/** AT1: the datasheet's 58 ms observation becoming a 90 ms window. */
#define ELM327_ADAPTIVE_NUM_AT1 3
#define ELM327_ADAPTIVE_DEN_AT1 2

/** AT2, "a little more aggressive". */
#define ELM327_ADAPTIVE_NUM_AT2 5
#define ELM327_ADAPTIVE_DEN_AT2 4

/**
 * @brief Shortest window adaptive timing will choose.
 *
 * Nothing on these buses can answer faster than this and still be answering
 * the request that was just sent - ISO 14230-2 alone allows an ECU 50 ms - so
 * a window below it could only ever cut off a reply that was on its way.
 */
#define ELM327_ADAPTIVE_MIN_MS 20

/**
 * @brief Floor on the window while the automatic search is running.
 *
 * "Also, during protocol searches, an internally set minimum time is used -
 * you may select longer times with AT ST, but not shorter ones." A search
 * that gives up on a protocol early reports the wrong protocol, not a slow
 * one, so this floor applies whatever AT ST and adaptive timing say.
 */
#define ELM327_SEARCH_MIN_MS 200

/**
 * @brief How long to wait for the next reply, in milliseconds.
 *
 * AT ST is the ceiling in every case; the datasheet is explicit that the
 * algorithm "will never choose one which is longer".
 */
static uint32_t elm327_reply_window(const elm327_ctx_t *e) {
    uint32_t window = (uint32_t)e->settings.timeout;

    if (e->settings.adaptive_timing != 0 && e->adaptive_ms != 0) {
        uint32_t learned = e->adaptive_ms;

        if (learned < ELM327_ADAPTIVE_MIN_MS) {
            learned = ELM327_ADAPTIVE_MIN_MS;
        }
        if (learned < window) {
            window = learned;
        }
    }

    if (e->in_search && window < ELM327_SEARCH_MIN_MS) {
        window = ELM327_SEARCH_MIN_MS;
    }

    return window;
}

/**
 * @brief Feed the algorithm one exchange's worth of evidence.
 *
 * @param latency_ms How long after the request the last reply arrived, or
 *                   zero when the exchange drew nothing at all.
 *
 * Two rules, and the asymmetry between them is the whole safety argument:
 *
 * - A slower vehicle than expected raises the window immediately. Being one
 *   exchange late is a slow reading; being one exchange short is a reading
 *   that never arrives.
 * - A faster one lowers it gradually, which is what "as conditions such as
 *   bus loading, etc. change, the algorithm learns from them" asks for -
 *   a single quick reply on a busy bus should not commit the next request to
 *   a window that only suited that one.
 *
 * An exchange that drew nothing forgets everything learned, so the next
 * request waits the full AT ST window. Without that, one window that turned
 * out to be too short would keep being too short: the reply that would have
 * corrected it is the one being cut off.
 */
static void elm327_note_latency(elm327_ctx_t *e, uint32_t latency_ms) {
    uint32_t target;

    if (latency_ms == 0) {
        /*
         * The window just expired with nothing in it, so whatever it was is
         * known to be too short for this vehicle. Recording that is the whole
         * difference between an algorithm that converges and one that
         * oscillates - see adaptive_floor_ms. It grows by the same margin a
         * measured reply would be given, and is capped by AT ST because the
         * datasheet is explicit that the algorithm "will never choose one
         * which is longer".
         */
        uint32_t floor = (uint32_t)e->adaptive_ms * ELM327_ADAPTIVE_NUM_AT1 /
                         ELM327_ADAPTIVE_DEN_AT1;

        if (floor > (uint32_t)e->settings.timeout) {
            floor = (uint32_t)e->settings.timeout;
        }
        if (floor > e->adaptive_floor_ms) {
            e->adaptive_floor_ms = (uint16_t)floor;
        }

        /* Still zero, so the next request gets the full window: the reply
         * that would calibrate the new floor is the one being cut off. */
        e->adaptive_ms = 0;
        return;
    }

    if (e->settings.adaptive_timing >= 2) {
        target = latency_ms * ELM327_ADAPTIVE_NUM_AT2 / ELM327_ADAPTIVE_DEN_AT2;
    } else {
        target = latency_ms * ELM327_ADAPTIVE_NUM_AT1 / ELM327_ADAPTIVE_DEN_AT1;
    }

    if (target < ELM327_ADAPTIVE_MIN_MS) {
        target = ELM327_ADAPTIVE_MIN_MS;
    }
    /* Never back under a window this vehicle has already failed. */
    if (target < e->adaptive_floor_ms) {
        target = e->adaptive_floor_ms;
    }
    if (target > UINT16_MAX) {
        target = UINT16_MAX;
    }

    if (e->adaptive_ms == 0 || target > e->adaptive_ms) {
        e->adaptive_ms = (uint16_t)target;
    } else {
        /* A quarter of the way down, so it takes a few consistent exchanges
         * to commit to a shorter window. */
        e->adaptive_ms -= (uint16_t)((e->adaptive_ms - target) / 4);
    }
}

/** @brief True for the two ISO 15765-4 protocols that use 11 bit identifiers.
 */
static bool elm327_can_is_11bit(const elm327_ctx_t *e) {
    return e->settings.current_protocol == ELM327_PROTO_CAN_11BIT_250K ||
           e->settings.current_protocol == ELM327_PROTO_CAN_11BIT_500K;
}

/**
 * @brief Is @p id one this request could be answered on?
 *
 * ISO 15765-4 clause 8 fixes both ranges: 0x7E8 to 0x7EF for the 11 bit
 * addressing scheme, and 0x18DAF100 to 0x18DAF1FF - a physical reply to the
 * tester, F1 - for the 29 bit one. The low bits are the ECU that answered, so
 * they are masked off rather than compared.
 *
 * The mask for the 11 bit range has to be 0x7F8, not 0x7E8: an identifier is
 * only in the range when the bits *outside* it are clear, and testing
 * (id & 0x7E8) == 0x7E8 asks the opposite question. It let 0x7F8 to 0x7FF
 * through as well, which is where several manufacturers put their
 * non-diagnostic traffic.
 *
 * The frame format is checked too. An 11 bit protocol has no business
 * accepting a 29 bit frame just because the bottom of its identifier happens
 * to land in range, and on a bus carrying both that is not a hypothetical.
 */
static bool elm327_can_reply_id_ok(const elm327_ctx_t *e, uint32_t id) {
    bool extended = (id & CAN_EFF_FLAG) != 0;

    if (id & CAN_RTR_FLAG) {
        /* A remote frame carries no data to be a reply with. */
        return false;
    }

    if (elm327_can_is_11bit(e)) {
        return !extended && (id & 0x7F8u) == 0x7E8u;
    }

    return extended && (id & CAN_EFF_MASK & 0x1FFFFF00u) == 0x18DAF100u;
}

/**
 * @brief Where a flow control frame for @p reply_id has to be addressed.
 *
 * ISO 15765-2 clause 6.4: flow control belongs to the connection the first
 * frame opened, which is the physical channel to the ECU that sent it - not
 * the functional address the request went out on. Sending it back to 0x7DF
 * asks every ECU on the bus to interpret one ECU's flow control, and a
 * multi-frame reply to a functional request - mode 09, every VIN read - is
 * exactly where that happens.
 *
 * The two schemes are laid out so this is arithmetic rather than a table:
 * 0x7E8+n answers 0x7E0+n, and 0x18DAF1nn answers 0x18DAnnF1.
 */
static uint32_t elm327_can_flow_control_id(const elm327_ctx_t *e,
                                           uint32_t reply_id) {
    if (elm327_can_is_11bit(e)) {
        return (reply_id & CAN_SFF_MASK) - 8u;
    }

    return 0x18DA0000u | ((reply_id & 0xFFu) << 8) | 0xF1u | CAN_EFF_FLAG;
}

static int elm327_can_protocol_xfer(elm327_ctx_t *e, const uint8_t *frame,
                                    size_t len, int num_frame) {
    int ret;
    struct can_frame tx_frame = {0};

    /* UDS framing. Single frame */
    tx_frame.id = e->settings.header_id;

    bool multicast = false;
    if (tx_frame.id == 0x7DF || tx_frame.id == 0x18DB33F1) {
        multicast = true;
    }

    // Set extended ID flag for 29 bit ID protocols
    if (e->settings.current_protocol == ELM327_PROTO_CAN_29BIT_250K ||
        e->settings.current_protocol == ELM327_PROTO_CAN_29BIT_500K ||
        e->settings.current_protocol == ELM327_PROTO_CAN_SAE_J1939) {
        tx_frame.id |= CAN_EFF_FLAG;
    }

    if (e->settings.can_auto_format) {
        if (len > sizeof(tx_frame.data) - 1) {
            ESP_LOGE(TAG, "%u bytes do not fit a formatted frame; try AT CAF0",
                     (unsigned)len);
            return -2;
        }

        tx_frame.data[0] = len; /* single frame transfer */
        memcpy(&tx_frame.data[1], frame, len);
        tx_frame.dlc = len + 1;
    } else {
        if (len > sizeof(tx_frame.data)) {
            ESP_LOGE(TAG, "%u bytes do not fit a CAN frame", (unsigned)len);
            return -2;
        }

        memcpy(&tx_frame.data[0], frame, len);
        tx_frame.dlc = len;
    }

    /*
     * ISO 15765-4 fixes DLC at 8 for every diagnostic frame on
     * these protocols. SAE J1939 is the one exception - its frames
     * are genuinely variable length.
     */
    if (e->settings.current_protocol != ELM327_PROTO_CAN_SAE_J1939) {
        tx_frame.dlc = sizeof(tx_frame.data);
    }

    read_all_can(e);

    ret = can_frame_send(e->session, &tx_frame);
    if (ret != 0) {
        ESP_LOGE(TAG, "Fail to send CAN frame ret=%d\n", ret);
        goto fail;
    }

    /* AT R0 overrides the frame count hint, as the datasheet requires. The
     * byte buses have honoured it in elm327_bus_xfer() all along; CAN was
     * waiting out the full reply window and printing the answer to a request
     * whose client had said it did not want one. */
    if (num_frame == 0 || !e->settings.responses) {
        // We done. 0 frame to read.
        return 0;
    }

    uint16_t pkt_size = 0;
    uint16_t curr_size = 0;
    uint8_t frame_counter = 0;
    uint32_t window = elm327_reply_window(e);

    /* Started once and never restarted, so it measures each reply against the
     * request rather than against the reply before it. That is what the
     * adaptive algorithm is defined on: "the actual response times that your
     * vehicle is responding in". */
    Timer since_request;
    Timer_init(&since_request);
    Timer_start(&since_request, 0);

    Timer timer;
    Timer_init(&timer);
    Timer_start(&timer, window);
    while (!Timer_is_expired(&timer)) {
        struct can_frame rx = {0};

        ret = can_frame_recv(e->session, &rx, pdMS_TO_TICKS(window));

        // Filter incoming frames
        // TODO: add support for user defined CAN ID filtering
        if (ret || rx.dlc < 1 || !elm327_can_reply_id_ok(e, rx.id)) {
            continue;
        }

        if (((rx.data[0] >> 4) & 0xf) == 0) {
            /* Single Frame */
            elm327_print_can_frame(e, rx.id, rx.dlc, rx.data);

            e->last_latency_ms = (uint32_t)Timer_elapsed_ms(&since_request);

            // Response pending frames
            if (rx.data[0] == 0x03 && rx.data[1] == 0x7f &&
                rx.data[3] == 0x78) {
                /* restart timer */
                Timer_start(&timer, 3000); // FIX ME
                continue;
            }

            if (!multicast) {
                return 0;
            }
            frame_counter++;
        } else if (((rx.data[0] >> 4) & 0xf) == 1) {
            /* Multi frame: First Frame */
            if (pkt_size) {
                ESP_LOGE(TAG, "Pending multi frame transfer\n");
                goto fail;
            }
            pkt_size = (((uint16_t)rx.data[0] & 0xf) << 8) | rx.data[1];

            curr_size = rx.dlc - 2;

            elm327_print_can_frame(e, rx.id, rx.dlc, rx.data);

            /* Send flow control, to the ECU that sent the first frame rather
             * than to whatever the request went out on. Clear to send, no
             * block size limit, no separation time: get all remaining frames.
             * A copy, because tx_frame still holds the request that a later
             * retry would want. */
            struct can_frame fc = {
                .id = elm327_can_flow_control_id(e, rx.id),
                .dlc = 8, /* ISO 15765-4: DLC 8, always. */
                .data = {0x30, 0x00, 0x00},
            };
            can_frame_send(e->session, &fc);

            /* restart timer */
            Timer_start(&timer, window);
            frame_counter++;
        } else if (((rx.data[0] >> 4) & 0xf) == 2) {
            uint8_t index = rx.data[0] & 0xf;

            /* The sequence number is 4 bits and rolls over after 15, so the
             * counter has to be masked to match. Comparing it unmasked broke
             * every transfer longer than 15 consecutive frames, which is any
             * payload over 111 bytes. */
            e->last_latency_ms = (uint32_t)Timer_elapsed_ms(&since_request);

            if (index != (frame_counter & 0x0f)) {
                ESP_LOGE(TAG, "Frame drop\n");
                goto fail;
            }

            elm327_print_can_frame(e, rx.id, rx.dlc, rx.data);
            frame_counter++;
            curr_size += 7;

            if (curr_size >= pkt_size) {
                /* got all frame. We are done here */
                return 0;
            }

            /* restart timer */
            Timer_start(&timer, window);
        } else if (((rx.data[0] >> 4) & 0xf) == 3) {
            /* Flow control frame - let host handle */
            elm327_print_can_frame(e, rx.id, rx.dlc, rx.data);
            return 0;
        } else {
            ESP_LOGE(TAG, "Unknow code: %02x", rx.data[0]);
            goto fail;
        }

        if (num_frame > 0 && frame_counter >= num_frame) {
            // reach number of frame needed. We done
            return 0;
        }
    }

    if (frame_counter == 0) {
        ESP_LOGE(TAG, "NO DATA / TIMEOUT");
        return -1;
    }
    return 0;

fail:
    return -2;
}

/**
 * @brief Clause 5.3.1: header byte bit 4 clear means three header bytes.
 *
 * Set, the message carries a single byte header and no addresses at all. The
 * distinction only exists on J1850; ISO 9141 always sends three.
 */
#define J1850_HDR_H_BIT 0x10

/** @brief True for the two KWP protocols, which share every rule but the
 *         initialisation that starts them. */
static bool elm327_is_kwp(const elm327_ctx_t *e) {
    return e->settings.current_protocol == ELM327_PROTO_ISO14230_KWP_5BAUD ||
           e->settings.current_protocol == ELM327_PROTO_ISO14230_KWP_FAST;
}

/** @brief True for every protocol that runs over the K-Line transceiver. */
static bool elm327_is_kline(const elm327_ctx_t *e) {
    return e->settings.current_protocol == ELM327_PROTO_ISO9141 ||
           elm327_is_kwp(e);
}

/**
 * @brief Header bytes in front of the data on @p frame.
 *
 * Three for J1850 with the H bit clear and for ISO 9141-2, which has no other
 * form. KWP is the one that varies: ISO 14230-2 clause 4.1.1 puts the address
 * mode in the top two bits of the format byte, and clause 4.1.4 adds a fourth
 * header byte when the six length bits are zero. Getting this wrong does not
 * corrupt anything - it decides how many bytes AT H0 strips off the front, so
 * it shows up as a reply displayed one byte out.
 */
static size_t elm327_header_len(const elm327_ctx_t *e, const uint8_t *frame,
                                size_t len) {
    bool j1850 = e->settings.current_protocol == ELM327_PROTO_J1850_PWM ||
                 e->settings.current_protocol == ELM327_PROTO_J1850_VPW;

    if (len >= 1 && j1850 && (frame[0] & J1850_HDR_H_BIT)) {
        return 1;
    }

    if (len >= 1 && elm327_is_kwp(e)) {
        size_t hdr = kline_header_len(frame[0]);

        if ((frame[0] & 0x3Fu) == 0 && len > hdr) {
            hdr += 1;
        }
        return hdr;
    }

    return 3;
}

/** @brief Does @p frame carry a target and a source address to judge? */
static bool elm327_frame_has_addresses(const elm327_ctx_t *e,
                                       const uint8_t *frame, size_t len) {
    if (len < 4) {
        return false;
    }

    if (e->settings.current_protocol == ELM327_PROTO_J1850_PWM ||
        e->settings.current_protocol == ELM327_PROTO_J1850_VPW) {
        return (frame[0] & J1850_HDR_H_BIT) == 0;
    }

    if (elm327_is_kwp(e)) {
        kline_addr_mode_t m = kline_addr_mode(frame[0]);

        return m == KLINE_ADDR_PHYSICAL || m == KLINE_ADDR_FUNCTIONAL;
    }

    /* ISO 9141-2 always sends three. */
    return true;
}

/**
 * @brief Is this frame addressed to us, and from whom we asked?
 *
 * The datasheet is explicit that a reply is displayed only "if the internally
 * stored receive address matches the address that the message is being sent
 * to", and that with Auto Receive - the default - that address comes from the
 * current header bytes. Without it, every message on the bus is printed as
 * though it answered the request that happens to be outstanding, which on a
 * network that carries ordinary vehicle traffic between the diagnostic
 * exchanges is most of them.
 *
 * Automatic accepts three targets, because a request can be answered at any
 * of them:
 *
 * - the source address the request went out with, which is where a module
 *   addressed physically sends its reply;
 * - the tester address from AT TA, normally the same byte and PP 06 on a real
 *   ELM327, kept separate so changing one does not silently change the other;
 * - the functional response address paired with the request's target. SAE
 *   J2178 pairs these as an even request address and the odd one above it -
 *   a tool asks 0x6A and the modules answer 0x6B - which is what a Ford SCP
 *   bus does and what the bench module here does.
 *
 * AT RA / AT SR replace all three with one fixed byte; AT FT adds a
 * transmitter to match on top, whatever else is set.
 *
 * A single byte header carries no addresses, so there is nothing to judge and
 * the frame is passed through. So is anything too short to hold a header and
 * a checksum: a runt is worth showing to whoever is looking at the bus.
 */
static bool elm327_frame_is_addressed_to_us(const elm327_ctx_t *e,
                                            const uint8_t *frame, size_t len) {
    uint8_t target, source, req_target, req_source;

    if (!elm327_frame_has_addresses(e, frame, len)) {
        return true;
    }

    target = frame[1];
    source = frame[2];
    req_target = (e->settings.header_id >> 8) & 0xff;
    req_source = e->settings.header_id & 0xff;

    if (e->settings.filter_tx && source != e->settings.tx_filter) {
        return false;
    }

    if (!e->settings.auto_receive) {
        return target == e->settings.recv_address;
    }

    /*
     * ISO 9141-2 fixes the reply address rather than deriving it. SAE J1979
     * puts every CARB response in a 48 6B <ECU> header, whatever address the
     * request carried, so $6B has to be accepted on its own account.
     *
     * Deriving it from the request works only while the request goes to the
     * functional address $6A - $6A | 1 is $6B - and fails the moment a client
     * addresses a module physically. AT SH 68 09 F1 followed by mode 03 is
     * the ordinary way to read trouble codes from one ECU, and every reply to
     * it was being discarded as somebody else's.
     *
     * Scoped to the K-Line protocols on purpose. A J1850 priority byte such
     * as $41 has the same two top bits as a CARB format byte and means
     * something entirely different, so testing the byte alone would quietly
     * apply an ISO 9141-2 rule to a bus that never agreed to it.
     */
    if (elm327_is_kline(e) && kline_is_carb_fmt(frame[0]) &&
        target == KLINE_CARB_TESTER_ADDR) {
        return true;
    }

    return target == req_source || target == e->settings.tester_address ||
           target == (uint8_t)(req_target | 1u);
}

static void elm327_print_j1850_kline_frame(elm327_ctx_t *e, const uint8_t *data,
                                           size_t len) {
    /* Three characters per byte with spacing on, for a frame as long as
     * KLINE_FRAME_CAP, plus the carriage return and the terminator. */
    char outstr[KLINE_FRAME_CAP * 3 + 2] = {0};
    char *p = &outstr[0];
    size_t hdr = elm327_header_len(e, data, len);

    /* Headers off strips what the standard wraps the data in: the header at
     * the front and the checksum at the back. A frame with nothing left under
     * them is printed whole rather than emptied. */
    if (!e->settings.show_header && len > hdr + 1) {
        data += hdr;
        len -= hdr + 1;
    }

    for (size_t i = 0; i < len; i++) {
        if (e->settings.hex_spacing) {
            sprintf(p, "%02X ", data[i]);
            p += 3;
        } else {
            sprintf(p, "%02X", data[i]);
            p += 2;
        }
    }
    *p++ = '\r';
    *p++ = '\0';

    elm327_send_string(e, outstr);
}

/**
 * @brief How long the bus must be quiet before the prompt goes back, J1850.
 *
 * A J1850 module asks to be acknowledged - the K bit in its header - and
 * retransmits its reply when nobody does. This adapter cannot answer inside
 * the window the standard gives an in-frame response (Tp4 to Tp5, 48 to 63 us
 * after the last rising edge); j1850_pwm_codec.h has the arithmetic and the
 * bench measurements behind that. So every reply arrives two or three times
 * over, a few milliseconds apart.
 *
 * The bus driver collapses those repeats, so a caller sees one reply. What it
 * cannot do is stop the *next* request from being transmitted into the middle
 * of the burst, and that is what goes wrong when a client asks for its PIDs
 * back to back with the frame count hint: the exchange returns as soon as the
 * first copy is printed, the client sends the next request a millisecond
 * later, and it collides with a retransmission of the answer to the previous
 * one. Measured on the bench, one request in five was lost that way; with the
 * driver's duplicate window shortened so the repeats reach this layer, the
 * loss turns into the worse failure instead - the retransmission is read back
 * as the answer to the request that followed it, and every value a client
 * displays is one request stale.
 *
 * So after an exchange that ended early, hold the prompt until the bus has
 * been quiet this long. It is the same figure the driver uses to decide a
 * transmission went unanswered, and the reasoning is the same: longer than
 * any legal response latency, short enough to be worth having over waiting
 * out the whole AT ST window.
 */
#define J1850_SETTLE_MS 20

/** But never hold the prompt longer than this, whatever the bus is doing. */
#define J1850_SETTLE_CAP_MS 100

/**
 * @brief Wait for the bus to go quiet, discarding whatever arrives meanwhile.
 *
 * @param cap       Largest frame this bus carries.
 * @param settle_ms Quiet period required. Zero disables the wait entirely,
 *                  which is what the buses with no in-frame response want.
 * @param quiet_ms  How long the bus has already been quiet. An exchange that
 *                  ran out its own AT ST window has nothing left to wait for.
 */
static void elm327_bus_settle(elm327_ctx_t *e, size_t cap, uint32_t settle_ms,
                              uint32_t quiet_ms) {
    uint8_t discard[32];
    bus_msg_t msg;
    Timer limit;

    if (settle_ms == 0 || quiet_ms >= settle_ms) {
        return;
    }

    bus_msg_init(&msg, discard, cap > sizeof(discard) ? sizeof(discard) : cap);

    Timer_init(&limit);
    Timer_start(&limit, J1850_SETTLE_CAP_MS);

    while (!Timer_is_expired(&limit)) {
        if (vif_bus_recv(e->session, elm327_bus(e), &msg,
                         pdMS_TO_TICKS(settle_ms)) <= 0) {
            /* Nothing for a whole settling period: the burst is over. */
            return;
        }
    }

    ESP_LOGD(TAG, "bus still busy after %d ms", J1850_SETTLE_CAP_MS);
}

/* ------------------------------------------------------------------ *
 * K-Line
 * ------------------------------------------------------------------ */

/**
 * @brief Build the on-wire request for one of the byte buses.
 *
 * J1850 and ISO 9141-2 are a straight copy: the three CARB header bytes from AT
 * SH and then the data. KWP is not, because its format byte carries the length
 * of the message, and a client that types "01 00" has no way to know that.
 *
 * The datasheet describes the rule the ELM327 uses, and this follows it
 * exactly so that software written against one works against the other: if
 * the low digit of the first header byte is anything but zero, the length
 * goes into the format byte and the header stays three bytes long; if it is
 * zero, a fourth header byte carries the length instead. The second form is
 * outside ISO 14230-4 but in wide use for manufacturer specific transfers,
 * which is exactly why AT SH is allowed to select it.
 *
 * @return Bytes written to @p out, or 0 when it will not fit.
 */
static size_t elm327_build_bus_request(elm327_ctx_t *e, const uint8_t *data,
                                       size_t len, uint8_t *out, size_t cap) {
    uint32_t id = e->settings.header_id;
    uint8_t fmt = (uint8_t)(id >> 16);
    size_t n = 0;

    if (len == 0 || len > 0x3F) {
        return 0;
    }

    if (!elm327_is_kwp(e)) {
        if (len + 3 > cap) {
            return 0;
        }
        out[n++] = fmt;
        out[n++] = (uint8_t)(id >> 8);
        out[n++] = (uint8_t)id;
        memcpy(&out[n], data, len);
        return n + len;
    }

    if (len + 4 > cap) {
        return 0;
    }

    if ((fmt & 0x3Fu) != 0) {
        out[n++] = (uint8_t)((fmt & 0xC0u) | (len & 0x3Fu));
        out[n++] = (uint8_t)(id >> 8);
        out[n++] = (uint8_t)id;
    } else {
        out[n++] = (uint8_t)(fmt & 0xC0u);
        out[n++] = (uint8_t)(id >> 8);
        out[n++] = (uint8_t)id;
        out[n++] = (uint8_t)len;
    }

    memcpy(&out[n], data, len);
    return n + len;
}

/** @brief The initialisation the selected protocol calls for. */
static kline_init_mode_t elm327_kline_init_mode(const elm327_ctx_t *e) {
    switch (e->settings.current_protocol) {
    case ELM327_PROTO_ISO9141:
    case ELM327_PROTO_ISO14230_KWP_5BAUD:
        return KLINE_INIT_5BAUD;
    case ELM327_PROTO_ISO14230_KWP_FAST:
        return KLINE_INIT_FAST;
    default:
        return KLINE_INIT_NONE;
    }
}

/** @brief Is a K-Line session up, and what did it negotiate? */
static bool elm327_kline_link(elm327_ctx_t *e, bus_link_t *out) {
    bus_link_t link;

    if (vif_bus_ioctl(e->session, VIF_BUS_KLINE, BUS_IOCTL_GET_LINK, NULL,
                      &link) != 0) {
        return false;
    }

    if (out) {
        *out = link;
    }

    return link.connected;
}

/** @brief Fill in an initialisation request from the AT settings. */
static void elm327_kline_init_args(const elm327_ctx_t *e,
                                   kline_init_mode_t mode, bus_init_t *io) {
    memset(io, 0, sizeof(*io));
    io->address = e->settings.iso_init_address;

    if (mode != KLINE_INIT_FAST) {
        return;
    }

    /* C1 33 F1 81: functional addressing, every emission related ECU, this
     * tester, StartCommunication. ISO 14230-2 clause 5.1.5.3, and the only
     * form ISO 14230-4 allows for legislated OBD. The address word is what a
     * functional StartCommunication targets too, so AT IIA moves both. */
    io->msg[0] = 0xC1;
    io->msg[1] = e->settings.iso_init_address;
    io->msg[2] = e->settings.tester_address;
    io->msg[3] = KLINE_SVC_START_COMM;
    io->msg_len = 4;
}

/** @brief Push AT SW and AT WM down to the driver, once there is a link. */
static void elm327_kline_apply_wakeup(elm327_ctx_t *e) {
    bus_msg_t msg;

    if (e->settings.wakeup_len) {
        bus_msg_init(&msg, e->settings.wakeup_msg, e->settings.wakeup_len);
        msg.len = e->settings.wakeup_len;
        vif_bus_ioctl(e->session, VIF_BUS_KLINE, BUS_IOCTL_SET_PERIODIC, &msg,
                      NULL);
    }

    vif_bus_param_set(e->session, VIF_BUS_KLINE, BUS_P_PERIODIC_MS,
                      e->settings.wakeup_ms);
}

/**
 * @brief Run an initialisation and report it the way the datasheet does.
 *
 * "BUS INIT: " then dots for the seconds a slow init spends on its address
 * word, then OK or ERROR. The dots are not decoration - they are the only
 * sign a client has that the adapter is working rather than hung, and a slow
 * init is long enough for that to matter.
 *
 * @return 0 on success, -1 when the sequence failed.
 */
static int elm327_kline_init(elm327_ctx_t *e, kline_init_mode_t mode) {
    bus_init_t io;
    int rc;

    elm327_kline_init_args(e, mode, &io);

    /* Fast initialisation is defined at one rate and one only. */
    vif_bus_param_set(e->session, VIF_BUS_KLINE, BUS_P_DATA_RATE,
                      mode == KLINE_INIT_FAST ? KLINE_BAUD_DEFAULT
                                              : e->settings.iso_baud);

    if (!e->in_search) {
        elm327_send_string(e, "BUS INIT: ");
        if (mode == KLINE_INIT_5BAUD) {
            elm327_send_string(e, "...");
        }
        elm327_uart_flush(e);
    }

    rc = vif_bus_ioctl(e->session, VIF_BUS_KLINE,
                       mode == KLINE_INIT_FAST ? BUS_IOCTL_FAST_INIT
                                               : BUS_IOCTL_FIVE_BAUD_INIT,
                       &io, &io);

    /* AT KW1 is the default: an initialisation whose key bytes name no
     * protocol this tester knows has not established anything. AT KW0 says to
     * accept them anyway, which is what lets a pre-OBD ECU be reached. */
    if (rc == 0 && e->settings.key_word_check) {
        kline_keybytes_t k;

        if (!kline_decode_keybytes(io.key[0], io.key[1], &k)) {
            ESP_LOGW(TAG, "key bytes %02X %02X rejected", io.key[0], io.key[1]);
            rc = -1;
        }
    }

    if (rc != 0) {
        ESP_LOGW(TAG, "K-Line init failed: %d", rc);
        if (!e->in_search) {
            elm327_send_string(e, "ERROR\r");
        }
        return -1;
    }

    elm327_kline_apply_wakeup(e);
    if (!e->in_search) {
        elm327_send_string(e, "OK\r");
    }
    return 0;
}

/**
 * @brief Make sure there is a session before a request goes out.
 *
 * The ELM327 initialises lazily, "generally not until a request needs to be
 * sent", and this does the same. A client that wants the two separated has
 * AT SI and AT FI; one that wants neither has AT BI.
 */
/**
 * @brief Settle on the protocol the key bytes named, not the one asked for.
 *
 * Protocols 3 and 4 are the same handshake to the same address at the same
 * rate; the only thing that separates them is what comes back. That is why
 * the whitepaper points out one 5 baud initialisation is enough to tell them
 * apart, and why the answer belongs to the vehicle rather than to the client.
 *
 * So both directions are corrected here. A vehicle that answered $8F xx is an
 * ISO 14230-4 vehicle even if AT SP 3 was typed, and wants the C2 33 F1
 * header; one that answered 08 08 is ISO 9141-2 even if AT SP 4 was, and
 * wants 68 6A F1. Sending the wrong one produces a link that initialised
 * perfectly and then answers nothing, which is the least diagnosable failure
 * this bus has.
 *
 * A fast initialised link is left alone: it can only ever be KWP.
 */
static void elm327_kline_reconcile(elm327_ctx_t *e) {
    bus_link_t link;
    int want;

    if (e->settings.current_protocol != ELM327_PROTO_ISO9141 &&
        e->settings.current_protocol != ELM327_PROTO_ISO14230_KWP_5BAUD) {
        return;
    }

    if (!elm327_kline_link(e, &link)) {
        return;
    }

    switch ((kline_variant_t)link.variant) {
    case KLINE_VARIANT_KWP:
        want = ELM327_PROTO_ISO14230_KWP_5BAUD;
        break;
    case KLINE_VARIANT_ISO9141:
        want = ELM327_PROTO_ISO9141;
        break;
    default:
        /* AT KW0 let an unrecognised pair through. The client asked for this,
         * so it keeps the protocol it asked for. */
        return;
    }

    if (want == e->settings.current_protocol) {
        return;
    }

    ESP_LOGI(TAG, "key bytes say %s; moving to protocol %d",
             kline_variant_name((kline_variant_t)link.variant), want);
    elm327_set_protocol(e, want);
}

static int elm327_kline_ensure_link(elm327_ctx_t *e) {
    kline_init_mode_t mode;

    if (elm327_kline_link(e, NULL)) {
        return 0;
    }

    mode = elm327_kline_init_mode(e);
    if (mode == KLINE_INIT_NONE) {
        return -1;
    }

    if (elm327_kline_init(e, mode) != 0) {
        return -1;
    }

    elm327_kline_reconcile(e);
    return 0;
}

/**
 * @brief One request/response exchange on a byte-oriented bus.
 *
 * J1850 PWM, J1850 VPW and ISO 9141 differ in their electrical layer, not in
 * how the ELM327 frames them: three header bytes from AT SH, then the payload,
 * then replies printed until the AT ST window closes or the frame count hint
 * is met. The places they do differ are parameters:
 *
 * - @p cap  is the largest frame the bus carries. J1850 is 12 bytes plus the
 *   three header bytes; K-Line messages run longer.
 * - @p drain_first discards traffic queued before the request. J1850 does;
 *   ISO 9141 does not, so a reply that arrived early still gets printed.
 * - @p settle_ms holds the prompt until the bus is quiet, for the buses whose
 *   modules retransmit unacknowledged replies. See J1850_SETTLE_MS.
 */
static int elm327_bus_xfer(elm327_ctx_t *e, const uint8_t *frame, size_t len,
                           int num_frame, size_t cap, bool drain_first,
                           uint32_t settle_ms) {
    int ret;
    uint8_t tx_frame[ELM327_BUS_CAP] = {0};
    uint8_t rx_frame[ELM327_BUS_CAP] = {0};
    bus_msg_t rx;
    bus_msg_t tx_msg;
    size_t tx_len;

    if (cap > sizeof(tx_frame)) {
        cap = sizeof(tx_frame);
    }

    bus_msg_init(&rx, rx_frame, cap);

    if (drain_first) {
        while (vif_bus_recv(e->session, elm327_bus(e), &rx, 0) > 0) {
            ;
        }
    }

    tx_len = elm327_build_bus_request(e, frame, len, tx_frame, cap);
    if (tx_len == 0) {
        ESP_LOGE(TAG, "Frame too large for this bus");
        return -2;
    }

    bus_msg_tx(&tx_msg, tx_frame, tx_len, 0);
    ret = vif_bus_send(e->session, elm327_bus(e), &tx_msg, 0);
    if (ret != 0) {
        ESP_LOGE(TAG, "Fail to send frame ret=%d", ret);
        return -2;
    }

    /* AT R0 overrides the frame count hint, as the datasheet requires. */
    if (num_frame == 0 || !e->settings.responses) {
        /* Nothing to read - but the module has not necessarily stopped
         * talking, so the bus still has to be handed over quiet. */
        elm327_bus_settle(e, cap, settle_ms, 0);
        return 0;
    }

    uint8_t frame_counter = 0;
    uint32_t window = elm327_reply_window(e);

    /* Started once and never restarted, so each reply is measured against the
     * request rather than against the reply before it - which is what the
     * adaptive algorithm is defined on. */
    Timer since_request;
    Timer_init(&since_request);
    Timer_start(&since_request, 0);

    Timer timer;
    Timer_init(&timer);
    Timer_start(&timer, window);
    while (!Timer_is_expired(&timer)) {

        ret =
            vif_bus_recv(e->session, elm327_bus(e), &rx, pdMS_TO_TICKS(window));

        if (ret <= 0) {
            continue;
        }

        /* Our own transmission coming back, or an answer to a message this
         * client never sent. Neither is a reply to the request in flight, and
         * neither should hold the window open. */
        if (rx.status & (BUS_RX_TX_MSG_TYPE | BUS_RX_PERIODIC_REPLY)) {
            continue;
        }

        if (!elm327_frame_is_addressed_to_us(e, rx_frame, ret)) {
            /* Somebody else's message. Not ours to print, and not ours to
             * hold the window open with either - a bus carrying ordinary
             * vehicle traffic would never let the prompt come back. */
            continue;
        }

        elm327_print_j1850_kline_frame(e, rx_frame, ret);
        frame_counter++;
        e->last_latency_ms = (uint32_t)Timer_elapsed_ms(&since_request);

        /* restart timer */
        Timer_start(&timer, window);

        if (num_frame > 0 && frame_counter >= num_frame) {
            // reach number of frame needed. We done
            elm327_bus_settle(e, cap, settle_ms, 0);
            return 0;
        }
    }

    /* The window ran out, so the bus has been quiet for exactly that long. */
    elm327_bus_settle(e, cap, settle_ms, window);

    if (frame_counter == 0) {
        ESP_LOGE(TAG, "NO DATA / TIMEOUT");
        return -1;
    }
    return 0;
}

static int elm327_j1850_pwm_xfer(elm327_ctx_t *e, const uint8_t *frame,
                                 size_t len, int num_frame) {
    return elm327_bus_xfer(e, frame, len, num_frame, J1850_FRAME_CAP, true,
                           J1850_SETTLE_MS);
}

static int elm327_j1850_vpw_xfer(elm327_ctx_t *e, const uint8_t *frame,
                                 size_t len, int num_frame) {
    return elm327_bus_xfer(e, frame, len, num_frame, J1850_FRAME_CAP, true,
                           J1850_SETTLE_MS);
}

/**
 * @brief ISO 9141-2 and both KWP variants, which differ only in how they start.
 *
 * Once the link is up the three are the same exchange: a header, a request,
 * and replies until the AT ST window closes. K-Line has no in-frame response
 * and nothing retransmits behind the tester's back, so unlike J1850 there is
 * no burst to wait out before the prompt can go back.
 *
 * @return -3 when the bus could not be initialised. The failure has already
 *         been reported to the client as BUS INIT: ...ERROR, so the caller
 *         must not add anything to it.
 */
static int elm327_kline_xfer(elm327_ctx_t *e, const uint8_t *frame, size_t len,
                             int num_frame) {
    if (elm327_kline_ensure_link(e) != 0) {
        return -3;
    }

    return elm327_bus_xfer(e, frame, len, num_frame, KLINE_FRAME_CAP, false, 0);
}

/** @brief Run one protocol's transfer, whichever kind of bus it is. */
static int elm327_try_protocol(elm327_ctx_t *e, uint8_t proto,
                               const uint8_t *frame, size_t len,
                               int num_frame) {
    if (elm327_set_protocol(e, proto) != 0) {
        return -2;
    }

    switch (proto) {
    case ELM327_PROTO_ISO9141:
    case ELM327_PROTO_ISO14230_KWP_5BAUD:
    case ELM327_PROTO_ISO14230_KWP_FAST:
        return elm327_kline_xfer(e, frame, len, num_frame);
    case ELM327_PROTO_J1850_PWM:
        return elm327_j1850_pwm_xfer(e, frame, len, num_frame);
    case ELM327_PROTO_J1850_VPW:
        return elm327_j1850_vpw_xfer(e, frame, len, num_frame);
    default:
        return elm327_can_protocol_xfer(e, frame, len, num_frame);
    }
}

#define J1850_CROSS_MODULATION_MS 750

/** @brief True for the two protocols that share the J1850 wire. */
static bool elm327_is_j1850(uint8_t proto) {
    return proto == ELM327_PROTO_J1850_PWM || proto == ELM327_PROTO_J1850_VPW;
}

/**
 * @brief One attempt in a search, with whatever the bus is owed beforehand.
 *
 * @param prev In and out: the protocol the previous attempt used, so that
 *             going from one J1850 modulation to the other can be spotted.
 */
static int elm327_search_attempt(elm327_ctx_t *e, uint8_t proto, uint8_t *prev,
                                 const uint8_t *frame, size_t len,
                                 int num_frame) {
    if (elm327_is_j1850(proto) && elm327_is_j1850(*prev) && proto != *prev) {
        vTaskDelay(pdMS_TO_TICKS(J1850_CROSS_MODULATION_MS));
    }

    *prev = proto;
    return elm327_try_protocol(e, proto, frame, len, num_frame);
}

/**
 * @brief Find the protocol this vehicle speaks, by trying each in turn.
 *
 * "SAE standard J1978 specifies a protocol search order that scan tools
 * should use... In order to provide a faster search, the ELM327 does not
 * normally follow this order." Neither does this: CAN is first because it is
 * what a vehicle built this century answers to, and the 5 baud handshake is
 * left as late as possible because it alone costs two and a half seconds
 * whether it works or not.
 *
 * Ahead of all of them comes whatever AT SP 0 was holding when it was asked
 * to go automatic. A client that has just been talking to the vehicle and
 * then resets the search - which is the common shape, and what AT SP 0 is
 * documented as being used for - gets that protocol confirmed in one attempt
 * rather than rediscovered from the far end of the list.
 *
 * ISO 14230-4 with a 5 baud init is deliberately absent: it is the same
 * handshake as ISO 9141-2 and differs only in the key bytes that come back,
 * so trying protocol 3 covers both and elm327_kline_reconcile() settles which
 * of them it turned out to be.
 */
static int elm327_search_protocol_xfer(elm327_ctx_t *e, const uint8_t *frame,
                                       size_t len, int num_frame,
                                       uint8_t skip) {
    /* Every protocol a vehicle can answer on, cheapest first.
     *
     * All four ISO 15765-4 combinations are here, and leaving any of them out
     * is not a slow search but a search that cannot succeed: an 11 bit /
     * 500 kbaud vehicle - the common case, and what most of this century's
     * cars are - answers nothing else, so a list holding only the 29 bit
     * variant reported NO DATA on a bus that was working perfectly.
     *
     * 500 kbaud comes before 250 kbaud because a wrong bit rate is the
     * expensive kind of wrong: the controller cannot even acknowledge a
     * frame, so it errors its way to bus-off and the teardown has to wait out
     * a recovery, where a wrong ID at the right bit rate merely goes
     * unanswered.
     *
     * ISO 14230-4 with a 5 baud init is deliberately absent: it is the same
     * handshake as ISO 9141-2 and differs only in the key bytes that come
     * back, so trying protocol 3 covers both and elm327_kline_reconcile()
     * settles which of them it turned out to be. */
    static const uint8_t order_fast[] = {
        ELM327_PROTO_CAN_11BIT_500K, ELM327_PROTO_CAN_29BIT_500K,
        ELM327_PROTO_CAN_11BIT_250K, ELM327_PROTO_CAN_29BIT_250K,
        ELM327_PROTO_ISO9141,        ELM327_PROTO_ISO14230_KWP_FAST,
        ELM327_PROTO_J1850_PWM,      ELM327_PROTO_J1850_VPW,
    };

    /* AT SS: "SAE standard J1978 specifies a protocol search order that scan
     * tools should use. It follows the number order that we have assigned to
     * the ELM327 protocols." */
    static const uint8_t order_std[] = {
        ELM327_PROTO_J1850_PWM,      ELM327_PROTO_J1850_VPW,
        ELM327_PROTO_ISO9141,        ELM327_PROTO_ISO14230_KWP_FAST,
        ELM327_PROTO_CAN_11BIT_500K, ELM327_PROTO_CAN_29BIT_500K,
        ELM327_PROTO_CAN_11BIT_250K, ELM327_PROTO_CAN_29BIT_250K,
    };

    const uint8_t *order =
        e->settings.std_search_order ? order_std : order_fast;
    size_t order_len =
        e->settings.std_search_order ? sizeof(order_std) : sizeof(order_fast);
    uint8_t first = e->search_first;
    /* The protocol the last attempt used, which is what says whether the next
     * one needs the J1850 recovery pause. @p skip counts: it was tried, on
     * this bus, immediately before this search started. */
    uint8_t prev = skip;
    int ret = -1;

    e->in_search = true;

    if (first != ELM327_PROTO_AUTO) {
        ret = elm327_search_attempt(e, first, &prev, frame, len, num_frame);
        if (ret >= 0) {
            ret = 0;
            goto done;
        }
    }

    for (size_t i = 0; i < order_len; i++) {
        if (order[i] == first || order[i] == skip) {
            continue; /* already tried, and it did not answer */
        }

        ret = elm327_search_attempt(e, order[i], &prev, frame, len, num_frame);
        if (ret >= 0) {
            ret = 0;
            goto done;
        }
    }

    /* Nothing answered. Release the bus and go back to knowing nothing, so
     * the next request searches again rather than reporting NO DATA forever
     * off the back of one failure. */
    e->in_search = false;
    elm327_set_protocol(e, ELM327_PROTO_AUTO);

    /* After that call and not before it. Selecting automatic remembers
     * whatever was current as the protocol to try first next time, which is
     * the right thing when a client sends AT SP 0 from a working protocol and
     * exactly the wrong thing here: what is current at this point is the last
     * protocol of the search that just failed. Clearing it first left every
     * subsequent search opening with the protocol least likely to answer -
     * J1850 VPW after one failed search, J1850 PWM after the next - which is
     * a wasted bus bring-up per request and a search order that no longer
     * resembled the one written down. */
    e->search_first = 0;

    vif_bus_release_all(e->session);
    e->settings.auto_search = false;
    return ret;

done:
    e->in_search = false;
    e->search_first = 0;
    /* Only now: AT DP and AT DPN prefix the protocol with A when it was
     * arrived at automatically, and a search that found nothing arrived at
     * nothing. */
    e->settings.auto_search = true;
    return ret;
}

static void elm327_parse_command(elm327_ctx_t *e, const char *cmd) {
    /* One marker per dispatch. Bytes on the wire cannot tell "the client sent
     * it twice" from "we ran it twice"; this can. */
    ble_uart_trace_note('!', cmd, strlen(cmd));

    int rc;
    uint8_t frame[8] = {0};

    if (cmd[0] == 'A' && cmd[1] == 'T') {
        /* AT commands */
        ESP_LOGI(TAG, "Command: %s", cmd);

        elm327_at_command_handler(e, &cmd[2]);
    } else {
        size_t len = strlen(cmd);
        int num_frame = -1;

        if (len & 1) {
            /* Extra hex digit is for frame count */
            num_frame = hex_char_to_int(cmd[len - 1]);
            len -= 1;
        }

        if (len > sizeof(frame) * 2) {
            ESP_LOGE(TAG, "buffer too small");
            elm327_send_string(e, "?\r");
            goto done;
        }

        int frame_len =
            hex_string_to_u8_array(cmd, len, &frame[0], sizeof(frame));

        if (frame_len <= 0) {
            ESP_LOGE(TAG, "Fail to parse the hex data: %s rc = %d", cmd,
                     frame_len);
            elm327_send_string(e, "?\r");
        } else {
            rc = -2;

            /* Every transfer path reports the latency of its last reply
             * through this, and the algorithm is fed once here rather than at
             * each of their exits. Zero means nothing answered, which is
             * itself evidence - see elm327_note_latency(). */
            e->last_latency_ms = 0;

            if (e->settings.current_protocol == 0) {
                /* Search every time the protocol is still unknown. A search
                 * that failed must not latch: the vehicle may simply have
                 * been asleep, and the datasheet expects to see it again -
                 * "Subsequent OBD requests may show 'SEARCHING'". A successful
                 * one leaves current_protocol set to what it found, so it
                 * does not come back through here at all. */
                elm327_send_string(e, "SEARCHING...\r");
                rc = elm327_search_protocol_xfer(e, frame, frame_len, num_frame,
                                                 ELM327_PROTO_AUTO);
            } else if (e->settings.current_protocol == ELM327_PROTO_J1850_PWM) {
                rc = elm327_j1850_pwm_xfer(e, frame, frame_len, num_frame);
            } else if (e->settings.current_protocol == ELM327_PROTO_J1850_VPW) {
                rc = elm327_j1850_vpw_xfer(e, frame, frame_len, num_frame);
            } else if (elm327_is_kline(e)) {
                rc = elm327_kline_xfer(e, frame, frame_len, num_frame);
            } else if (e->settings.current_protocol ==
                           ELM327_PROTO_CAN_29BIT_500K ||
                       e->settings.current_protocol ==
                           ELM327_PROTO_CAN_29BIT_250K ||
                       e->settings.current_protocol ==
                           ELM327_PROTO_CAN_11BIT_500K ||
                       e->settings.current_protocol ==
                           ELM327_PROTO_CAN_11BIT_250K) {
                rc = elm327_can_protocol_xfer(e, frame, frame_len, num_frame);
            }

            /* AT SP Ah / AT TP Ah: "if the protocol that is tried should fail
             * to initialize, the ELM327 will then automatically sequence
             * through the other protocols". The one just tried is skipped -
             * it is what did not answer - and the marker is spent either way,
             * so a vehicle that stays silent reports NO DATA on the next
             * request rather than searching every protocol again. */
            if (rc < 0 && rc != -3 && e->settings.auto_fallback) {
                uint8_t tried = e->settings.current_protocol;

                e->settings.auto_fallback = false;
                e->search_first = 0;
                elm327_send_string(e, "SEARCHING...\r");
                rc = elm327_search_protocol_xfer(e, frame, frame_len, num_frame,
                                                 tried);
            }

            elm327_note_latency(e, e->last_latency_ms);

            if (rc == -1) {
                elm327_send_string(e, "NO DATA\r");
            } else if (rc == -3) {
                /* Already reported in full - a second line here would have
                 * the client seeing "BUS INIT: ...ERROR" followed by "?". */
            } else if (rc < 0) {
                elm327_send_string(e, "?\r");
            }
        }
    }

done:
    elm327_send_string(e, "\r>");
    elm327_uart_flush(e);
}

/**
 * @brief Feed one received byte into the command line assembler.
 *
 * Echoes, strips spaces, folds to upper case, and dispatches on carriage
 * return. An empty line repeats the previous command, as the ELM327 does.
 */
static void elm327_feed_byte(elm327_ctx_t *e, char val) {
    /* Echo back */
    if (e->settings.echo) {
        elm327_uart_send_text(e, (const uint8_t *)&val, 1);
    }

    /*
     * Strip whitespace, and with it the linefeed half of a CR LF line ending.
     * A client that terminates its lines that way is entitled to: the ELM327
     * dispatches on the carriage return and ignores the rest. Keeping the
     * linefeed put it at the front of the *next* command, so every command
     * after the first was read as "\n0105" and rejected - which looks to the
     * client exactly like the adapter answering the wrong request.
     */
    if (val == ' ' || val == '\n' || val == '\t' || val == '\0') {
        return;
    }

    /* convert to upper case */
    if (val >= 'a' && val <= 'z') {
        val -= 0x20;
    }

    e->line.cmdbuf[e->line.cmdidx] = val;

    if (val == '\r') {
        e->line.cmdbuf[e->line.cmdidx] = '\0';

        /*
         * Every line that arrives gets an answer and a prompt. That is not
         * politeness, it is the only thing keeping the client in step: an
         * ELM327 conversation is strictly one prompt per line, so a line that
         * produces nothing leaves the client waiting for a prompt that will
         * never come. It times out, sends the next command, and reads the
         * previous reply against it - and stays exactly one behind for the
         * rest of the session, answering every request with the answer to the
         * one before it.
         *
         * Two lines used to produce nothing: a single character, and anything
         * that overran the buffer. Both now answer '?' like any other line
         * that means nothing.
         */
        if (e->line.overflow) {
            ESP_LOGE(TAG, "Command too long");
            elm327_send_string(e, "?\r\r>");
            elm327_uart_flush(e);
        } else if (e->line.cmdidx == 0) {
            /* A bare carriage return repeats the last command. */
            elm327_parse_command(e, e->line.lastcmd);
        } else {
            elm327_parse_command(e, e->line.cmdbuf);

            /* store last command, including the terminator at [cmdidx] */
            memcpy(e->line.lastcmd, e->line.cmdbuf, e->line.cmdidx + 1);
        }

        e->line.cmdidx = 0;
        e->line.overflow = false;
        return;
    }

    e->line.cmdidx++;
    if (e->line.cmdidx == (int)sizeof(e->line.cmdbuf) - 1) {
        /* Keep swallowing until the carriage return, then answer once. A
         * buffer reset here instead would make the tail of the line look like
         * a command of its own and produce a second prompt for one line,
         * which puts the client out of step just as surely. */
        e->line.overflow = true;
        e->line.cmdidx = 0;
    }
}

/* ------------------------------------------------------------------ *
 * Front-end
 * ------------------------------------------------------------------ */

/**
 * @brief Claim a free instance from the pool.
 *
 * A session that is already running ELM327 keeps the instance it has, so a
 * redundant switch to the front-end already installed is not an error.
 */
static elm327_ctx_t *elm327_instance_claim(vif_session_t *s) {
    for (int i = 0; i < ELM327_MAX_INSTANCES; i++) {
        if (g_elm327[i].live && g_elm327[i].session == s) {
            return &g_elm327[i];
        }
    }

    for (int i = 0; i < ELM327_MAX_INSTANCES; i++) {
        if (!g_elm327[i].live) {
            return &g_elm327[i];
        }
    }

    return NULL;
}

static void *elm327_fe_create(vif_session_t *s) {
    elm327_ctx_t *e = elm327_instance_claim(s);

    if (!e) {
        ESP_LOGE(TAG, "no free ELM327 instance");
        return NULL;
    }

    /* destroy() normally releases this first; a reclaimed instance that still
     * holds one would otherwise leak it under the memset. */
    if (e->tx_ringbuf) {
        vRingbufferDeleteWithCaps(e->tx_ringbuf);
    }

    memset(e, 0, sizeof(*e));

    e->tx_ringbuf = xRingbufferCreateWithCaps(512, RINGBUF_TYPE_BYTEBUF,
                                              MALLOC_CAP_DEFAULT);
    if (!e->tx_ringbuf) {
        ESP_LOGE(TAG, "Failed to allocate tx ring buffer");
        return NULL;
    }

    e->live = true;
    e->session = s;

    /*
     * Settings only, and the protocol starts back at automatic. vif released
     * every claim on the way in, so there is no bus here to inherit and
     * nothing to reconcile: the first request selects a protocol and opens
     * one.
     */
    elm327_load_defaults(e);

    return e;
}

static void elm327_fe_feed(void *ctx, const uint8_t *data, size_t len) {
    elm327_ctx_t *e = ctx;

    for (size_t i = 0; i < len; i++) {
        elm327_feed_byte(e, (char)data[i]);

        /* A reset in the middle of this chunk. The rest of it was sent before
         * the reset finished, so it goes the same way as everything else that
         * arrived during one. See the AT Z handler. */
        if (e->line.reset_discard) {
            e->line.reset_discard = false;
            e->line.cmdidx = 0;
            e->line.overflow = false;
            return;
        }
    }
}

static void elm327_fe_destroy(void *ctx) {
    elm327_ctx_t *e = ctx;

    /* Whatever the last command answered is still in the tx buffer; the client
     * should see it before the grammar changes underneath it. */
    elm327_uart_flush(e);

    /*
     * The client that owned this conversation has gone, so the conversation
     * is over - and on KWP that is something the ECU has to be told. It was
     * put into a diagnostic session on this client's behalf, and ISO 14230-2
     * clause 5.2 gives it a StopCommunication rather than leaving it to time
     * out at P3max.
     *
     * This has to happen here, before the claim goes: vif tears the bus
     * down as it swaps the grammar, and an ECU left mid-session would not
     * answer the next client's initialisation. Closing an app and reopening
     * it was enough to reproduce that.
     */
    if (e->session && elm327_is_kline(e)) {
        vif_bus_ioctl(e->session, VIF_BUS_KLINE, BUS_IOCTL_STOP_COMM, NULL,
                      NULL);
    }

    if (e->tx_ringbuf) {
        vRingbufferDeleteWithCaps(e->tx_ringbuf);
        e->tx_ringbuf = NULL;
    }

    e->live = false;
    e->session = NULL;
}

const vif_frontend_t elm327_frontend = {
    .name = "elm327",
    .create = elm327_fe_create,
    .feed = elm327_fe_feed,
    .destroy = elm327_fe_destroy,
};

void elm327_register(void) {
    if (vif_frontend_register(&elm327_frontend) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register the ELM327 front-end");
    }
}
