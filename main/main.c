/* SPDX-License-Identifier: GPL-3.0-only */
#include <inttypes.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "ble_uart.h"
#include "button.h"
#include "comm_iface.h"
#include "common.h"
#include "elm327_at.h"
#include "link.h"
#include "nvs_flash.h"
#include "pinout.h"
#include "sdkconfig.h"
#include "shell.h"
#include "slcan.h"
#include "tusb_console.h"
#include "usb_cdc.h"
#include "utility.h"
#include "vif.h"
#include "vif_ctrl.h"
#include "ws2812_led.h"

#define TAG "app_main"

/*
 * Port IDs. Only the data links are comm_iface ports: USB CDC 1 carries the
 * console and the debug shell, which are driven through stdio rather than the
 * multiplexer.
 */
comm_port_id_t g_port_usb_cdc0 = COMM_INVALID_PORT_ID;
comm_port_id_t g_port_ble = COMM_INVALID_PORT_ID;

/** Room for a full command line and then some; overflow is dropped, not
 * blocked. */
#define LINK_RX_BUF 256

/* USB CDC 0: the USB data link */
static int port_usb_cdc0_write(const void *buf, uint32_t length) {
    return usb_cdc_tx_write(USB_CDC_INTERFACE_DATA, buf, length);
}

static void port_usb_cdc0_flush(void) {
    usb_cdc_tx_flush(USB_CDC_INTERFACE_DATA);
}

static bool port_usb_cdc0_is_connected(void) {
    return usb_cdc_get_line_status(USB_CDC_INTERFACE_DATA);
}

static void usb_cdc0_rx_callback(const uint8_t *rx_buf, size_t rx_size) {
    comm_port_rx(g_port_usb_cdc0, rx_buf, rx_size);
}

/* BLE UART Wrapper */
static int port_ble_write(const void *buf, uint32_t length) {
    ble_uart_send(buf, length);
    return length; // Assume success
}

static void port_ble_flush(void) {
    // BLE doesn't have explicit flush
}

static bool port_ble_is_connected(void) { return ble_uart_is_connected(); }

static void ble_rx_callback(const uint8_t *rx_buf, size_t rx_size) {
    comm_port_rx(g_port_ble, rx_buf, rx_size);
}

static void comm_ports_setup(void) {
    static const comm_port_ops_t cdc0_ops = {
        .write = port_usb_cdc0_write,
        .flush = port_usb_cdc0_flush,
        .is_connected = port_usb_cdc0_is_connected,
    };
    static const comm_port_ops_t ble_ops = {
        .write = port_ble_write,
        .flush = port_ble_flush,
        .is_connected = port_ble_is_connected,
    };

    g_port_usb_cdc0 = comm_port_register("CDC0", &cdc0_ops, LINK_RX_BUF);
    g_port_ble = comm_port_register("BLE", &ble_ops, LINK_RX_BUF);

    if (g_port_usb_cdc0 == COMM_INVALID_PORT_ID ||
        g_port_ble == COMM_INVALID_PORT_ID) {
        ESP_LOGE(TAG, "Failed to register one or more comm ports");
    }

    /*
     * Connect port drivers to our RX callbacks. USB CDC 1 deliberately gets
     * none: esp_tusb_init_console() puts a VFS reader on that interface, and
     * two consumers of the same CDC would race for every byte.
     */
    usb_cdc_rx_set_callback(USB_CDC_INTERFACE_DATA, usb_cdc0_rx_callback);
    ble_uart_rx_set_callback(ble_rx_callback);
}

/**
 * @brief The one data link, and the transport currently carrying it.
 *
 * A user talks to this adapter over USB or over BLE, never both at once, so
 * there is one session and it follows whichever transport has a client. That
 * is what keeps the grammar, the settings and the claims in one place: two
 * standing sessions would arbitrate against each other over hardware only one
 * of them was ever going to use.
 *
 * First connected wins. The loser is still connected as far as its transport
 * is concerned - refusing that is not vif's business - it simply does not get
 * the link until the first one lets go.
 *
 * The link starts on ELM327 and returns to it when its client goes away, so
 * an off-the-shelf OBD-II app finds what it expects however it was last used.
 */
static vif_session_t *g_link;

static void link_try_bind(comm_port_id_t port) {
    comm_port_id_t held = vif_session_port(g_link);

    if (held == port || held != COMM_INVALID_PORT_ID) {
        return; /* already ours, or the other transport got here first */
    }

    vif_session_set_port(g_link, port);
}

/**
 * @brief Offer the free link to a transport other than @p just_freed.
 *
 * That one was turned away while the link was held, and nothing else would
 * come along to ask it again.
 */
static void link_offer_elsewhere(comm_port_id_t just_freed) {
    if (just_freed != g_port_usb_cdc0 && port_usb_cdc0_is_connected()) {
        link_try_bind(g_port_usb_cdc0);
    } else if (just_freed != g_port_ble && ble_uart_is_connected()) {
        link_try_bind(g_port_ble);
    }
}

/** Take the link off @p port and pass it on. The client is treated as gone:
 *  its grammar and its claims must not reach whoever comes next. */
static void link_hand_back(comm_port_id_t port) {
    vif_session_link_down(g_link);
    vif_session_set_port(g_link, COMM_INVALID_PORT_ID);
    link_offer_elsewhere(port);
}

static void link_release(comm_port_id_t port) {
    if (vif_session_port(g_link) != port) {
        return; /* never held the link */
    }

    link_hand_back(port);
}

/* ------------------------------------------------------------------ *
 * link.h: the manual override
 * ------------------------------------------------------------------ */

static comm_port_id_t transport_port(link_transport_t t) {
    switch (t) {
    case LINK_USB:
        return g_port_usb_cdc0;
    case LINK_BLE:
        return g_port_ble;
    default:
        return COMM_INVALID_PORT_ID;
    }
}

link_transport_t link_holder(void) {
    comm_port_id_t held = vif_session_port(g_link);

    if (held == COMM_INVALID_PORT_ID) {
        return LINK_NONE;
    }

    return (held == g_port_ble) ? LINK_BLE : LINK_USB;
}

const char *link_transport_name(link_transport_t t) {
    switch (t) {
    case LINK_USB:
        return "usb";
    case LINK_BLE:
        return "ble";
    default:
        return "none";
    }
}

bool link_transport_connected(link_transport_t t) {
    switch (t) {
    case LINK_USB:
        return port_usb_cdc0_is_connected();
    case LINK_BLE:
        return ble_uart_is_connected();
    default:
        return false;
    }
}

void link_set(link_transport_t t) {
    comm_port_id_t want = transport_port(t);
    comm_port_id_t held = vif_session_port(g_link);

    if (held == want) {
        return;
    }

    if (held != COMM_INVALID_PORT_ID) {
        vif_session_link_down(g_link);
        vif_session_set_port(g_link, COMM_INVALID_PORT_ID);
    }

    if (want != COMM_INVALID_PORT_ID) {
        /* Named outright, so it wins over first-connected. */
        vif_session_set_port(g_link, want);
        ESP_LOGI(TAG, "link: %s by request", link_transport_name(t));
        return;
    }

    ESP_LOGI(TAG, "link: let go by request");
    link_offer_elsewhere(held);
}

/**
 * @brief A host opened or closed the USB data link.
 *
 * The falling edge hands the link back; it never reverts the grammar on
 * *connect*, which would be the obvious thing and the wrong one: the shell
 * sets the link's mode before the tool that needs it opens the port - slcand
 * cannot ask for SLCAN itself - so flipping back when that tool attaches
 * would make the whole feature unusable.
 */
static void usb_line_state_changed(int itf, bool connected) {
    if (itf != USB_CDC_INTERFACE_DATA) {
        return;
    }

    if (connected) {
        link_try_bind(g_port_usb_cdc0);
    } else {
        link_release(g_port_usb_cdc0);
    }
}

/**
 * @brief The front panel button was held down.
 *
 * Opens the BLE pairing window. A device with no screen and no keypad cannot
 * prove who is asking to connect to it any other way: the one thing a
 * stranger driving past cannot do is stand at the car and press a button, so
 * that press is what a new phone needs before it is allowed to bond.
 */
static void button_held(void) { ble_uart_open_pairing_window(); }

/**
 * @brief The front panel button was held down for a long time.
 *
 * Forgets every phone. The way back when a phone and this adapter disagree
 * about what key they share - which the phone alone can cause, by being told
 * to forget the adapter - and the way to hand a unit on to somebody else.
 *
 * The pairing window opened on the way past the shorter threshold, and is
 * left open on purpose: with nothing bonded, pairing again is the only thing
 * anybody would want to do next.
 */
static void button_held_long(void) { ble_uart_forget_bonds(); }

static void ble_conn_changed(bool connected) {
    if (connected) {
        link_try_bind(g_port_ble);
    } else {
        link_release(g_port_ble);
    }
}

/**
 * @brief One control command written to the BLE control characteristic.
 *
 * There is one link and this acts on it. A phone that has the link gets the
 * answers it expects; one that was turned away can still ask what is running
 * and who holds what, which is the only way it would find out.
 */
static void ble_ctrl_command(const char *cmd, size_t len) {
    char out[VIF_CTRL_REPLY_MAX];

    vif_ctrl_exec(g_link, cmd, len, out, sizeof(out));
    ble_uart_ctrl_reply(out);
}

static void links_setup(void) {
    elm327_register();
    slcan_register();

    g_link = vif_session_open(LINK_NAME_DATA, VIF_SESSION_LINK);
    if (!g_link) {
        ESP_LOGE(TAG, "failed to open the data link session");
        return;
    }

    vif_session_set_default_frontend(g_link, &elm327_frontend);

    if (vif_session_set_frontend(g_link, &elm327_frontend) != ESP_OK) {
        ESP_LOGE(TAG, "failed to start the default front-end");
    }

    /*
     * The link binds to whichever transport connects first and is handed back
     * when that one leaves. Both edges are reported by the transports
     * themselves, so nothing has to poll.
     */
    usb_cdc_set_line_callback(usb_line_state_changed);
    ble_uart_set_conn_callback(ble_conn_changed);

    /* A host that opened the port before the callback existed reports no
     * edge, so the state is read once here rather than waited for. */
    if (port_usb_cdc0_is_connected()) {
        link_try_bind(g_port_usb_cdc0);
    } else if (ble_uart_is_connected()) {
        link_try_bind(g_port_ble);
    }

    /* Hold down button to enter BLE pairing; keep holding to forget every
     * phone that has ever paired. */
    button_set_hold_callback(button_held);
    button_set_long_hold_callback(button_held_long);

    /* The control plane for a BLE client. USB reaches it through the shell. */
    ble_uart_ctrl_set_callback(ble_ctrl_command);
}

/**
 * @brief Application main entry point
 */
void app_main(void) {
    esp_err_t err;
    /*
     * NVS flash initialization
     * Dependency of BLE stack to store configurations and board voltage senses
     * calibration data
     */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nvs flash, error code: %d ", err);
        abort();
    }

    /*
     * Vehicle interface first: it owns the board - gpio, adc, pwm - and every
     * bus, and a session cannot be opened before it is up.
     */
    err = vif_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize the vehicle interface: %s",
                 esp_err_to_name(err));
        abort();
    }

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ", CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154)
               ? ", 802.15.4 (Zigbee/Thread)"
               : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        abort();
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded"
                                                         : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n",
           esp_get_minimum_free_heap_size());

    /* The front panel button. Its handler is installed with the transports'. */
    button_init();

    /* Setup WS2812 status led */
    ws2812_led_init();
    ws2812_led_set_state(LED_STATE_IDLE);

    /*
     * Bring the multiplexer and its ports up before the transports, so that a
     * BLE central cannot connect and write before the RX callbacks exist.
     */
    err = comm_iface_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize comm_iface: %s",
                 esp_err_to_name(err));
    }
    comm_ports_setup();

    /* setup devices */
    usb_cdc_setup();
    ble_uart_setup();

    /*
     * Hand the console to USB CDC 1. Everything up to here has gone to UART0,
     * which is what it is for now: early debug, before USB exists. From this
     * point the shell and every ESP_LOG line come out on the second CDC
     * interface, which is where a user can actually reach them.
     */
    err = esp_tusb_init_console(USB_CDC_INTERFACE_SHELL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Console stays on UART0: %s", esp_err_to_name(err));
    }

    /* The data link, on whichever of USB CDC 0 and BLE has a client. */
    links_setup();

    /* start shell */
    shell_loop();

    /* shouldn't be here */
    printf("Shell loop returned. Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
