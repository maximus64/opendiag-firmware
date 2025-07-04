/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "common.h"
#include "elm327_at.h"
#include "utility.h"
#include "timer.h"
#include "comm_iface.h"
#include "vif.h"

#define DEFAULT_TIMEOUT_MS 200

#define TAG "ELM327_AT"


#define ELM_IDENTIFY "ELM327 v2.3"

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
} elm327_line_t;

/** The AT-settable knobs. Reset to defaults whenever the front-end starts. */
typedef struct {
    int timeout;
    bool echo;
    bool show_header;
    bool linefeed;
    uint8_t current_protocol;
    bool auto_search;
    bool can_auto_format;
    uint32_t header_id;
    bool hex_spacing;
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
 * through it, so a claim is attributed to this client and survives a switch
 * to another protocol.
 */
typedef struct {
    bool live;
    vif_session_t *session;
    comm_port_id_t port;
    RingbufHandle_t tx_ringbuf;
    elm327_settings_t settings;
    elm327_line_t line;
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
    ELM327_PROTO_AUTO = 0,               // Automatic
    ELM327_PROTO_J1850_PWM = 1,          // SAE J1850 PWM (41.6 kbaud)
    ELM327_PROTO_J1850_VPW = 2,          // SAE J1850 VPW (10.4 kbaud)
    ELM327_PROTO_ISO9141 = 3,            // ISO 9141-2 (5 baud init, 10.4 kbaud)
    ELM327_PROTO_ISO14230_KWP_5BAUD = 4, // ISO 14230-4 KWP (5 baud init, 10.4 kbaud)
    ELM327_PROTO_ISO14230_KWP_FAST = 5,  // ISO 14230-4 KWP (fast init, 10.4 kbaud)
    ELM327_PROTO_CAN_11BIT_500K = 6,     // ISO 15765-4 CAN (11 bit ID, 500 kbaud)
    ELM327_PROTO_CAN_29BIT_500K = 7,     // ISO 15765-4 CAN (29 bit ID, 500 kbaud)
    ELM327_PROTO_CAN_11BIT_250K = 8,     // ISO 15765-4 CAN (11 bit ID, 250 kbaud)
    ELM327_PROTO_CAN_29BIT_250K = 9,     // ISO 15765-4 CAN (29 bit ID, 250 kbaud)
    ELM327_PROTO_CAN_SAE_J1939 = 10,     // SAE J1939 CAN (29 bit ID, 250* kbaud)
    ELM327_PROTO_CAN_USER1 = 11,         // USER1 CAN (11* bit ID, 125* kbaud)
    ELM327_PROTO_CAN_USER2 = 12,         // USER2 CAN (11* bit ID, 50* kbaud)
};

static int elm327_set_protocol(elm327_ctx_t *e, int proto);
static void elm327_uart_flush(elm327_ctx_t *e);


static size_t elm327_uart_read_bytes(elm327_ctx_t *e, uint8_t *buffer, size_t len, TickType_t ticks_to_wait)
{
    return comm_port_read(e->port, buffer, len, ticks_to_wait);
}

static size_t elm327_uart_send_bytes(elm327_ctx_t *e, const uint8_t *buffer, size_t size)
{
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

static void elm327_uart_flush(elm327_ctx_t *e)
{
    size_t size = 0;
    uint8_t *data;

    while (1) {
        data = (uint8_t *) xRingbufferReceiveUpTo(e->tx_ringbuf, &size, 0, 64);
        if (!data) {
            break;
        }

        /* Answer whoever sent the request, not every attached client */
        comm_port_write(e->port, data, size);

        vRingbufferReturnItem(e->tx_ringbuf, data);
    }

    /* One push per flush, rather than per 64 byte chunk. A short response is
     * flushed once, at the end of the command; a long one is flushed again
     * each time it fills the buffer. */
    comm_port_flush(e->port);
}

static void elm327_send_string(elm327_ctx_t *e, const char *s)
{
    elm327_uart_send_bytes(e, (const uint8_t *)s, strlen(s));
}

/**
 * @brief Put the command grammar back to its power-on defaults.
 *
 * Settings only. Installing this front-end must not disturb the session's
 * claims: a client that raised a programming voltage, switched to SLCAN to
 * flash, and came back expects to find the voltage still up.
 */
static void elm327_load_defaults(elm327_ctx_t *e)
{
    e->settings.timeout = DEFAULT_TIMEOUT_MS;
    e->settings.echo = true;
    e->settings.show_header = 1;
    e->settings.linefeed = 0;
    e->settings.current_protocol = ELM327_PROTO_AUTO;
    e->settings.auto_search = false;
    e->settings.can_auto_format = true;
    e->settings.header_id = 0;
    e->settings.hex_spacing = true;
}

/**
 * @brief AT Z / AT D: defaults, bus down, and this client's pins released.
 *
 * Only the claims held by this session are dropped. A reset here used to call
 * board_hs_ls_reset_state(), which cut the programming voltage for every
 * client on the adapter, whoever had raised it.
 */
static void elm327_reset(elm327_ctx_t *e) {
    ESP_LOGI(TAG, "Reset setting to defaults");

    elm327_set_protocol(e, ELM327_PROTO_AUTO);
    elm327_load_defaults(e);

    vif_pin_release_all(e->session);
}

#define VOLTAGE_PIN_OFF 0xffffffff
#define VOLTAGE_SHORT_TO_GROUND 0xfffffffe

/**
 * @brief AT PROGV: drive an OBD-II pin, or release it.
 *
 * The two sentinel "voltages" are an ELM327 quirk, so they are translated
 * here; the one-high-side-one-low-side rule they run into lives in vif.
 */
static int at_set_programing_voltage(elm327_ctx_t *e, const char* arg)
{
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

    ESP_LOGI(TAG, "Set Programming Voltage: pin=%d voltage=0x%08" PRIx32, pin, voltage);

    if (voltage == VOLTAGE_PIN_OFF) {
        mode = VIF_PIN_OFF;
    }
    else if (voltage == VOLTAGE_SHORT_TO_GROUND) {
        mode = VIF_PIN_GROUND;
    }
    else {
        mode = VIF_PIN_VOLTAGE;
    }

    if (vif_pin_set(e->session, pin, mode, voltage) != ESP_OK) {
        return -3;
    }

    return 0;
}

static int at_set_header(elm327_ctx_t *e, const char* arg)
{
    int rc;
    uint32_t id = 0;

    rc = hex_string_to_u32_be(arg, strlen(arg), &id);
    if (rc) {
        return -2;
    }

    ESP_LOGI(TAG, "Set Header to 0x%08" PRIx32, id);

    e->settings.header_id = id;

    return 0;
}

/**
 * @brief AT SP: select the protocol, which is also the bus this client holds.
 *
 * One vif_bus_open() covers the switch: the session's previous bus goes down
 * inside it, because two bus drivers cannot be live at once. Protocol numbers
 * with no driver behind them yet - KWP, J1939, the two USER slots - leave the
 * session with no bus at all, which is what selecting them did before.
 *
 * @return 0, or -1 when the bus could not be opened, which now includes
 *         another session already holding it.
 */
static int elm327_set_protocol(elm327_ctx_t *e, int proto) {
    vif_bus_cfg_t cfg = {0};
    vif_bus_t bus = VIF_BUS_NONE;
    uint32_t header_id = 0;

    if (e->settings.current_protocol == proto) {
        ESP_LOGW(TAG, "Already using protocol: %d", proto);
        return 0;
    }

    if (e->settings.current_protocol == ELM327_PROTO_AUTO) {
        /* leaving automatic: any search result is stale */
        e->settings.auto_search = false;
    }

    switch (proto)
    {
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
        /* ISO 9141-2 (5 baud init, 10.4 kbaud) */
        bus = VIF_BUS_KLINE;
        header_id = 0x686af1;
        break;
    case ELM327_PROTO_CAN_11BIT_250K:
        bus = VIF_BUS_CAN;
        cfg.bitrate = 250000;
        header_id = 0x7DF;
        break;
    case ELM327_PROTO_CAN_11BIT_500K:
        bus = VIF_BUS_CAN;
        cfg.bitrate = 500000;
        header_id = 0x7DF;
        break;
    case ELM327_PROTO_CAN_29BIT_250K:
        bus = VIF_BUS_CAN;
        cfg.bitrate = 250000;
        header_id = 0x18DB33F1;
        break;
    case ELM327_PROTO_CAN_29BIT_500K:
        bus = VIF_BUS_CAN;
        cfg.bitrate = 500000;
        header_id = 0x18DB33F1;
        break;
    default:
        break;
    }

    if (bus == VIF_BUS_NONE) {
        vif_bus_close(e->session);
        e->settings.current_protocol = proto;
        return 0;
    }

    if (vif_bus_open(e->session, bus, &cfg) != ESP_OK) {
        /* The bus is held by another client, or its driver refused to start.
         * Either way this session has no bus now, and says so. */
        ESP_LOGE(TAG, "Protocol %d unavailable", proto);
        e->settings.current_protocol = ELM327_PROTO_AUTO;
        return -1;
    }

    e->settings.header_id = header_id;
    e->settings.current_protocol = proto;
    return 0;
}


static void elm327_at_command_handler(elm327_ctx_t *e, const char *cmd)
{
    int rc;

    if (strcmp(cmd, "Z") == 0) {
        ESP_LOGI(TAG, "Reset all command");

        elm327_reset(e);

        /* Simulate ELM327 reset behavior. Some apps rely on this */
        Timer timer;
        Timer_init(&timer);
        Timer_start(&timer, 1000);

        while (!Timer_is_expired(&timer)) {
            uint8_t val;
            elm327_uart_read_bytes(e, (uint8_t*)&val, 1, pdMS_TO_TICKS(100));
        }

        ESP_LOGI(TAG, "Reset DONE");
        elm327_send_string(e, "\r\r" ELM_IDENTIFY "\r");
    }
    else if (strcmp(cmd, "D") == 0) {
        elm327_reset(e);
        elm327_send_string(e, "OK\r");
    }
    else if (strncmp(cmd, "SP", 2) == 0) {
        int8_t protocol = hex_char_to_int(cmd[2]);

        ESP_LOGI(TAG, "Set protocol: %d", protocol);
        if (protocol >= 0 && protocol <= 0xc && elm327_set_protocol(e, protocol) == 0) {
            elm327_send_string(e, "OK\r");
        }
        else {
            elm327_send_string(e, "?\r");
        }
    }
    else if (strncmp(cmd, "ST", 2) == 0) {
        int8_t high_nibble = hex_char_to_int(cmd[2]);
        int8_t low_nibble = hex_char_to_int(cmd[3]);

        if (high_nibble == -1 || low_nibble == -1) {
            ESP_LOGE(TAG, "Set timeout invalid hex");
            elm327_send_string(e, "?\r");
        }
        else {
            int val = (int)high_nibble << 4 | low_nibble;
            if (val) {
                e->settings.timeout = val * 4;
                ESP_LOGI(TAG, "Timeout set to %d ms", e->settings.timeout);
            }
            else {
                e->settings.timeout = DEFAULT_TIMEOUT_MS;
            }
        }
    }
    else if (strcmp(cmd, "DPN") == 0) {
        char pn[6];
        ESP_LOGI(TAG, "Describe the Protocol by Number");

        if (e->settings.auto_search) {
            snprintf(pn, sizeof(pn), "A%d\r", e->settings.current_protocol);
        }
        else {
            snprintf(pn, sizeof(pn), "%d\r", e->settings.current_protocol);
        }
        elm327_send_string(e, pn);
    }
    else if (strcmp(cmd, "E0") == 0) {
        ESP_LOGI(TAG, "Echo off");
        e->settings.echo = 0;
        elm327_send_string(e, "OK\r");
    }
    else if (strcmp(cmd, "E1") == 0) {
        ESP_LOGI(TAG, "Echo on");
        e->settings.echo = 1;
        elm327_send_string(e, "OK\r");
    }
    else if (strcmp(cmd, "S0") == 0) {
        ESP_LOGI(TAG, "Spacing off");
        e->settings.hex_spacing = false;
        elm327_send_string(e, "OK\r");
    }
    else if (strcmp(cmd, "S1") == 0) {
        ESP_LOGI(TAG, "Spacing on");
        e->settings.hex_spacing = true;
        elm327_send_string(e, "OK\r");
    }
    else if (strcmp(cmd, "H0") == 0) {
        ESP_LOGI(TAG, "Headers OFF");
        e->settings.show_header = 0;
        elm327_send_string(e, "OK\r");
    }
    else if (strcmp(cmd, "H1") == 0) {
        ESP_LOGI(TAG, "Headers ON");
        e->settings.show_header = 1;
        elm327_send_string(e, "OK\r");
    }
    else if (strcmp(cmd, "I") == 0) {
        elm327_send_string(e, ELM_IDENTIFY "\r");
    }
    else if (strcmp(cmd, "L0") == 0) {
        ESP_LOGI(TAG, "Linefeed OFF");
        e->settings.linefeed = 0;
        elm327_send_string(e, "OK\r");
    }
    else if (strcmp(cmd, "L1") == 0) {
        ESP_LOGI(TAG, "Linefeed ON");
        e->settings.linefeed = 1;
        elm327_send_string(e, "OK\r");
    }
    else if (strcmp(cmd, "M0") == 0) {
        elm327_send_string(e, "OK\r");
    }
    else if (strcmp(cmd, "M1") == 0) {
        // TODO: for Memory on, we support to remeber the last use protocol
        elm327_send_string(e, "OK\r");
    }
    else if (strcmp(cmd, "CAF0") == 0) {
        ESP_LOGI(TAG, "CAN auto format OFF");
        e->settings.can_auto_format = false;
        elm327_send_string(e, "OK\r");
    }
    else if (strcmp(cmd, "CAF1") == 0) {
        ESP_LOGI(TAG, "CAN auto format ON");
        e->settings.can_auto_format = true;
        elm327_send_string(e, "OK\r");
    }
    else if (strcmp(cmd, "RV") == 0) {
        char voltstr[8];

        ESP_LOGI(TAG, "Read Voltage");
        float volt = vif_vbatt_mv() / 1000.0f;
        snprintf(voltstr, sizeof(voltstr), "%0.1fV\r", volt);
        elm327_send_string(e, voltstr);
    }
    else if (strncmp(cmd, "PROGV", 5) == 0) {
        rc = at_set_programing_voltage(e, &cmd[5]);
        if (rc) {
            elm327_send_string(e, "?\r");
        }
        else {
            elm327_send_string(e, "OK\r");
        }
    }
    else if (strncmp(cmd, "SH", 2) == 0) {
        rc = at_set_header(e, &cmd[2]);
        if (rc) {
            elm327_send_string(e, "?\r");
        }
        else {
            elm327_send_string(e, "OK\r");
        }
    }
    else if (strcmp(cmd, "@1") == 0) {
        elm327_send_string(e, OPENDIAG_PRODUCT " " OPENDIAG_VERSION "\r");
    }
    else {
        elm327_send_string(e, "?\r");
    }
}

/** @brief Drop whatever the bus queued before this request went out. */
static void read_all_can(elm327_ctx_t *e)
{
    struct can_frame frame;

    while (1) {
        int ret = vif_can_recv(e->session, &frame, 0);
        if (ret != 0)
         return;
    }
}

static void elm327_print_can_frame(elm327_ctx_t *e, uint32_t id, uint8_t dlc, const uint8_t *data)
{
    char outstr[64] = {0};
    char *p = &outstr[0];

    if (id & CAN_EFF_FLAG) {
        if (e->settings.hex_spacing) {
            sprintf(p, "%08" PRIX32 " ", id & CAN_EFF_MASK);
            p += 9;
        }
        else {
            sprintf(p, "%08" PRIX32, id & CAN_EFF_MASK);
            p += 8;
        }
    }
    else {
        if (e->settings.hex_spacing) {
            sprintf(p, "%03" PRIX32 " ", id & CAN_SFF_MASK);
            p += 4;
        }
        else {
            sprintf(p, "%03" PRIX32, id & CAN_SFF_MASK);
            p += 3;
        }
    }
    for (int i = 0; i < dlc; i++) {
        if (e->settings.hex_spacing) {
            sprintf(p, "%02X ", data[i]);
            p += 3;
        }
        else {
            sprintf(p, "%02X", data[i]);
            p += 2;
        }
    }
    *p++ = '\r';
    *p++ = '\0';

    elm327_send_string(e, outstr);
}

static int elm327_can_protocol_xfer(elm327_ctx_t *e, const uint8_t *frame, size_t len, int num_frame)
{
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
    }
    else {
        if (len > sizeof(tx_frame.data)) {
            ESP_LOGE(TAG, "%u bytes do not fit a CAN frame", (unsigned)len);
            return -2;
        }

        memcpy(&tx_frame.data[0], frame, len);
        tx_frame.dlc = len;
    }

    read_all_can(e);

    ret = vif_can_send(e->session, &tx_frame);
    if (ret != 0) {
        ESP_LOGE(TAG, "Fail to send CAN frame ret=%d\n", ret);
        goto fail;
    }

    if (num_frame == 0) {
        // We done. 0 frame to read.
        return 0;
    }

    uint16_t pkt_size = 0;
    uint16_t curr_size = 0;
    uint8_t frame_counter = 0;

    Timer timer;
    Timer_init(&timer);
    Timer_start(&timer, e->settings.timeout);
    while (!Timer_is_expired(&timer)) {
        struct can_frame rx = {0};

        ret = vif_can_recv(e->session, &rx, pdMS_TO_TICKS(e->settings.timeout));

        // Filter incoming frames
        // TODO: add support for user defined CAN ID filtering
        if (e->settings.current_protocol == ELM327_PROTO_CAN_11BIT_250K ||
            e->settings.current_protocol == ELM327_PROTO_CAN_11BIT_500K) {
            if (ret || (rx.id & 0x7E8) != 0x7E8 || rx.dlc < 1) {
                continue;
            }
        }
        else {
            if (ret || (rx.id & 0x1FFFFF00) != 0x18DAF100 || rx.dlc < 1) {
                continue;
            }
        }

        if (((rx.data[0] >> 4) & 0xf) == 0) {
            /* Single Frame */
            elm327_print_can_frame(e, rx.id, rx.dlc, rx.data);

            // Response pending frames
            if (rx.data[0] == 0x03 && rx.data[1] == 0x7f && rx.data[3] == 0x78) {
                /* restart timer */
                Timer_start(&timer, 3000); // FIX ME
                continue;
            }

            if (!multicast) {
                return 0;
            }
            frame_counter++;
        }
        else if (((rx.data[0] >> 4) & 0xf) == 1) {
            /* Multi frame: First Frame */
            if (pkt_size) {
                ESP_LOGE(TAG, "Pending multi frame transfer\n");
                goto fail;
            }
            pkt_size = (((uint16_t)rx.data[0] & 0xf) << 8) | rx.data[1];

            curr_size = rx.dlc - 2;

            elm327_print_can_frame(e, rx.id, rx.dlc, rx.data);

            /* send flow control. Get all remaining frames */
            tx_frame.data[0] = 0x30;
            tx_frame.data[1] = 0x00;
            tx_frame.data[2] = 0x00;
            tx_frame.dlc = 3;
            vif_can_send(e->session, &tx_frame);

            /* restart timer */
            Timer_start(&timer, e->settings.timeout);
            frame_counter++;
        }
        else if (((rx.data[0] >> 4) & 0xf) == 2) {
            uint8_t index = rx.data[0] & 0xf;

            /* The sequence number is 4 bits and rolls over after 15, so the
             * counter has to be masked to match. Comparing it unmasked broke
             * every transfer longer than 15 consecutive frames, which is any
             * payload over 111 bytes. */
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
            Timer_start(&timer, e->settings.timeout);
        }
        else if (((rx.data[0] >> 4) & 0xf) == 3) {
            /* Flow control frame - let host handle */
            elm327_print_can_frame(e, rx.id, rx.dlc, rx.data);
            return 0;
        }
        else {
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

static void elm327_print_j1850_kline_frame(elm327_ctx_t *e, const uint8_t *data, size_t len)
{
    /* Three characters per byte with spacing on, for a frame as long as
     * KLINE_FRAME_CAP, plus the terminator. */
    char outstr[104] = {0};
    char *p = &outstr[0];

    if (!e->settings.show_header && len > 4) {
        data += 3;
        len -= 3; //Skip 3 bytes header

        len -= 1; //Skip last CRC
    }

    for (size_t i = 0; i < len; i++) {
        if (e->settings.hex_spacing) {
            sprintf(p, "%02X ", data[i]);
            p += 3;
        }
        else {
            sprintf(p, "%02X", data[i]);
            p += 2;
        }
    }
    *p++ = '\r';
    *p++ = '\0';

    elm327_send_string(e, outstr);
}

/**
 * @brief One request/response exchange on a byte-oriented bus.
 *
 * J1850 PWM, J1850 VPW and ISO 9141 differ in their electrical layer, not in
 * how the ELM327 frames them: three header bytes from AT SH, then the payload,
 * then replies printed until the AT ST window closes or the frame count hint
 * is met. The two places they do differ are parameters:
 *
 * - @p cap  is the largest frame the bus carries. J1850 is 12 bytes plus the
 *   three header bytes; K-Line messages run longer.
 * - @p drain_first discards traffic queued before the request. J1850 does;
 *   ISO 9141 does not, so a reply that arrived early still gets printed.
 */
static int elm327_bytebus_xfer(elm327_ctx_t *e, const uint8_t *frame, size_t len, int num_frame,
                               size_t cap, bool drain_first)
{
    int ret;
    uint8_t tx_frame[32] = {0};
    uint8_t rx_frame[32] = {0};
    uint32_t id;

    if (cap > sizeof(tx_frame)) {
        cap = sizeof(tx_frame);
    }

    id = e->settings.header_id;

    if (len + 3 > cap) {
        ESP_LOGE(TAG, "Frame too large for this bus\n");
        return -2;
    }

    if (drain_first) {
        while (1) {
            ret = vif_raw_recv(e->session, rx_frame, cap, 0);
            if (ret <= 0) {
                break;
            }
        }
    }

    // contruct frame
    tx_frame[0] = (id >> 16) & 0xff; // header
    tx_frame[1] = (id >> 8) & 0xff; // dest address
    tx_frame[2] = (id) & 0xff; // src address
    memcpy(&tx_frame[3], frame, len);

    ret = vif_raw_send(e->session, tx_frame, len + 3);
    if (ret != 0) {
        ESP_LOGE(TAG, "Fail to send frame ret=%d\n", ret);
        return -2;
    }

    if (num_frame == 0) {
        // We done. 0 frame to read.
        return 0;
    }

    uint8_t frame_counter = 0;

    Timer timer;
    Timer_init(&timer);
    Timer_start(&timer, e->settings.timeout);
    while (!Timer_is_expired(&timer)) {

        ret = vif_raw_recv(e->session, rx_frame, cap, pdMS_TO_TICKS(e->settings.timeout));

        if (ret <= 0) {
            continue;
        }

        elm327_print_j1850_kline_frame(e, rx_frame, ret);
        frame_counter++;

        /* restart timer */
        Timer_start(&timer, e->settings.timeout);

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
}

/** J1850 carries 12 payload bytes at most, so 15 with the header. */
#define J1850_FRAME_CAP 15
#define KLINE_FRAME_CAP 32

static int elm327_j1850_pwm_xfer(elm327_ctx_t *e, const uint8_t *frame, size_t len, int num_frame)
{
    return elm327_bytebus_xfer(e, frame, len, num_frame, J1850_FRAME_CAP, true);
}

static int elm327_j1850_vpw_xfer(elm327_ctx_t *e, const uint8_t *frame, size_t len, int num_frame)
{
    return elm327_bytebus_xfer(e, frame, len, num_frame, J1850_FRAME_CAP, true);
}

static int elm327_iso9141_xfer(elm327_ctx_t *e, const uint8_t *frame, size_t len, int num_frame)
{
    return elm327_bytebus_xfer(e, frame, len, num_frame, KLINE_FRAME_CAP, false);
}

static int elm327_search_protocol_xfer(elm327_ctx_t *e, const uint8_t *frame, size_t len, int num_frame)
{
    int ret = -1;
    // uint32_t save_id = e->settings.header_id;

    // // Use default header when searching protocol
    // e->settings.header_id = 0;
    // Try can bus first
    elm327_set_protocol(e, ELM327_PROTO_CAN_29BIT_500K);
    ret = elm327_can_protocol_xfer(e, frame, len, num_frame);
    if (ret >= 0) {
        ret = 0;
        goto done;
    }

    // Try K-Line ISO 9141
    elm327_set_protocol(e, ELM327_PROTO_ISO9141);
    ret = elm327_iso9141_xfer(e, frame, len, num_frame);
    if (ret >= 0) {
        ret = 0;
        goto done;
    }

    // Try J1850 PWM
    elm327_set_protocol(e, ELM327_PROTO_J1850_PWM);
    ret = elm327_j1850_pwm_xfer(e, frame, len, num_frame);
    if (ret >= 0) {
        ret = 0;
        goto done;
    }

    // Try J1850 VPW
    elm327_set_protocol(e, ELM327_PROTO_J1850_VPW);
    ret = elm327_j1850_vpw_xfer(e, frame, len, num_frame);
    if (ret >= 0) {
        ret = 0;
        goto done;
    }

    if (ret != 0) {
        //Auto search fail.
        elm327_set_protocol(e, 0); //teardown
    }

done:
    // e->settings.header_id = save_id;
    e->settings.auto_search = true;
    return ret;
}


static void elm327_parse_command(elm327_ctx_t *e, const char *cmd)
{
    int rc;
    uint8_t frame[8] = {0};

    if (cmd[0] == 'A' && cmd[1] == 'T') {
        /* AT commands */
        ESP_LOGI(TAG, "Command: %s", cmd);

        elm327_at_command_handler(e, &cmd[2]);
    }
    else {
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

        int frame_len = hex_string_to_u8_array(cmd, len, &frame[0], sizeof(frame));

        if (frame_len <= 0) {
            ESP_LOGE(TAG, "Fail to parse the hex data: %s rc = %d", cmd, frame_len);
            elm327_send_string(e, "?\r");
        }
        else {
            rc = -2;

            if (e->settings.current_protocol == 0) {
                // Check if we already auto search.
                if (e->settings.auto_search) {
                    rc = -1;
                }
                else {
                    elm327_send_string(e, "SEARCHING...\r");
                    rc = elm327_search_protocol_xfer(e, frame, frame_len, num_frame);
                }
            }
            else if (e->settings.current_protocol == ELM327_PROTO_J1850_PWM) {
                rc = elm327_j1850_pwm_xfer(e, frame, frame_len, num_frame);
            }
            else if (e->settings.current_protocol == ELM327_PROTO_J1850_VPW) {
                rc = elm327_j1850_vpw_xfer(e, frame, frame_len, num_frame);
            }
            else if (e->settings.current_protocol == ELM327_PROTO_ISO9141) {
                rc = elm327_iso9141_xfer(e, frame, frame_len, num_frame);
            }
            else if (e->settings.current_protocol == ELM327_PROTO_CAN_29BIT_500K ||
                     e->settings.current_protocol == ELM327_PROTO_CAN_29BIT_250K ||
                     e->settings.current_protocol == ELM327_PROTO_CAN_11BIT_500K ||
                     e->settings.current_protocol == ELM327_PROTO_CAN_11BIT_250K) {
                rc = elm327_can_protocol_xfer(e, frame, frame_len, num_frame);
            }


            if (rc == -1) {
                elm327_send_string(e, "NO DATA\r");
            }
            else if (rc < 0) {
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
static void elm327_feed_byte(elm327_ctx_t *e, char val)
{
    /* Echo back */
    if (e->settings.echo) {
        elm327_uart_send_bytes(e, (const uint8_t *)&val, 1);
    }

    /* strip white space */
    if (val == ' ') {
        return;
    }

    /* convert to upper case */
    if (val >= 'a' && val <= 'z') {
        val -= 0x20;
    }

    e->line.cmdbuf[e->line.cmdidx] = val;

    if (val == '\r') {
        e->line.cmdbuf[e->line.cmdidx] = '\0';

        if (e->line.cmdidx >= 2) {
            elm327_parse_command(e, e->line.cmdbuf);

            /* store last command, including the terminator at [cmdidx] */
            memcpy(e->line.lastcmd, e->line.cmdbuf, e->line.cmdidx + 1);
        }
        else if (e->line.cmdidx == 0) {
            /* repeat last command */
            elm327_parse_command(e, e->line.lastcmd);
        }
        else {
            ESP_LOGE(TAG, "Command too short");
        }

        e->line.cmdidx = 0;
        return;
    }

    e->line.cmdidx++;
    if (e->line.cmdidx == (int)sizeof(e->line.cmdbuf)) {
        ESP_LOGE(TAG, "Command buffer overflow");
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
static elm327_ctx_t *elm327_instance_claim(vif_session_t *s)
{
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

static void *elm327_fe_create(vif_session_t *s, comm_port_id_t port)
{
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

    e->tx_ringbuf = xRingbufferCreateWithCaps(512, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_DEFAULT);
    if (!e->tx_ringbuf) {
        ESP_LOGE(TAG, "Failed to allocate tx ring buffer");
        return NULL;
    }

    e->live = true;
    e->session = s;
    e->port = port;

    /*
     * Settings only. Whatever the session already holds - a bus, a programming
     * voltage raised under a different front-end - stays exactly as it is.
     * The protocol number does start back at automatic, because a CAN claim
     * does not say whether it was 11 or 29 bit; the first request re-selects
     * one, which reopens the bus rather than losing it.
     */
    elm327_load_defaults(e);

    return e;
}

static void elm327_fe_feed(void *ctx, const uint8_t *data, size_t len)
{
    elm327_ctx_t *e = ctx;

    for (size_t i = 0; i < len; i++) {
        elm327_feed_byte(e, (char)data[i]);
    }
}

static void elm327_fe_destroy(void *ctx)
{
    elm327_ctx_t *e = ctx;

    /* Whatever the last command answered is still in the tx buffer; the client
     * should see it before the grammar changes underneath it. */
    elm327_uart_flush(e);

    /* Claims are the session's, not the front-end's, and outlive it. */
    if (e->tx_ringbuf) {
        vRingbufferDeleteWithCaps(e->tx_ringbuf);
        e->tx_ringbuf = NULL;
    }

    e->live = false;
    e->session = NULL;
    e->port = COMM_INVALID_PORT_ID;
}

const vif_frontend_t elm327_frontend = {
    .name = "elm327",
    .create = elm327_fe_create,
    .feed = elm327_fe_feed,
    .destroy = elm327_fe_destroy,
};

void elm327_register(void)
{
    if (vif_frontend_register(&elm327_frontend) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register the ELM327 front-end");
    }
}
