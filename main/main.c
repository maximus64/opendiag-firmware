/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "pinout.h"
#include "utility.h"
#include "elm327_at.h"
#include "links.h"
#include "ws2812_led.h"
#include "shell.h"
#include "slcan.h"
#include "usb_cdc.h"
#include "tusb_console.h"
#include "ble_uart.h"
#include "button.h"
#include "comm_iface.h"
#include "vif.h"
#include "vif_ctrl.h"

#define TAG "app_main"

/*
 * Port IDs. Only the data links are comm_iface ports: USB CDC 1 carries the
 * console and the debug shell, which are driven through stdio rather than the
 * multiplexer.
 */
comm_port_id_t g_port_usb_cdc0 = COMM_INVALID_PORT_ID;
comm_port_id_t g_port_ble = COMM_INVALID_PORT_ID;

/** Room for a full command line and then some; overflow is dropped, not blocked. */
#define LINK_RX_BUF 256

/* ========================================================================
 * Port Wrapper Functions
 *
 * These adapt the port drivers to the comm_iface API.
 * ======================================================================== */

/* USB CDC 0: the USB data link */
static int port_usb_cdc0_write(const void *buf, uint32_t length)
{
    return usb_cdc_tx_write(USB_CDC_INTERFACE_DATA, buf, length);
}

static void port_usb_cdc0_flush(void)
{
    usb_cdc_tx_flush(USB_CDC_INTERFACE_DATA);
}

static bool port_usb_cdc0_is_connected(void)
{
    return usb_cdc_get_line_status(USB_CDC_INTERFACE_DATA);
}

static void usb_cdc0_rx_callback(const uint8_t *rx_buf, size_t rx_size)
{
    comm_port_rx(g_port_usb_cdc0, rx_buf, rx_size);
}

/* BLE UART Wrapper */
static int port_ble_write(const void *buf, uint32_t length)
{
    ble_uart_send(buf, length);
    return length; // Assume success
}

static void port_ble_flush(void)
{
    // BLE doesn't have explicit flush
}

static bool port_ble_is_connected(void)
{
    return ble_uart_is_connected();
}

static void ble_rx_callback(const uint8_t *rx_buf, size_t rx_size)
{
    comm_port_rx(g_port_ble, rx_buf, rx_size);
}

/**
 * @brief Register the data link transports.
 */
static void comm_ports_setup(void)
{
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
    g_port_ble      = comm_port_register("BLE", &ble_ops, LINK_RX_BUF);

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
 * @brief Bring one data link up: a transport, a session, and a grammar.
 *
 * One session per link, not per protocol. Two clients sharing a single
 * front-end would share its settings and race over which of them a reply was
 * addressed to; separate sessions also mean vif arbitrates their claims
 * against each other, which is the honest answer when both want the one bus.
 *
 * Every link starts on ELM327 and returns to it when its client goes away, so
 * an off-the-shelf OBD-II app finds what it expects however the link was last
 * used.
 */
/* The two data links, kept so the transport callbacks can name them. */
static vif_session_t *g_link_usb;
static vif_session_t *g_link_ble;

static vif_session_t *link_setup(const char *name, comm_port_id_t port)
{
    vif_session_t *session;

    if (port == COMM_INVALID_PORT_ID) {
        ESP_LOGE(TAG, "%s: no transport", name);
        return NULL;
    }

    session = vif_session_open(name, port);
    if (!session) {
        ESP_LOGE(TAG, "%s: failed to open the session", name);
        return NULL;
    }

    vif_session_set_default_frontend(session, &elm327_frontend);

    if (vif_session_set_frontend(session, &elm327_frontend) != ESP_OK) {
        ESP_LOGE(TAG, "%s: failed to start the default front-end", name);
        vif_session_close(session);
        return NULL;
    }

    return session;
}

/**
 * @brief A host opened or closed the USB data link.
 *
 * Only the falling edge matters, and only for the data interface. Reverting
 * on *connect* would be the obvious thing and the wrong one: the shell sets a
 * link's mode before the tool that needs it opens the port - slcand cannot ask
 * for SLCAN itself - so flipping back when that tool attaches would make the
 * whole feature unusable.
 */
static void usb_line_state_changed(int itf, bool connected)
{
    if (itf == USB_CDC_INTERFACE_DATA && !connected) {
        vif_session_link_down(g_link_usb);
    }
}

/**
 * @brief The front panel button was held down.
 *
 * The last way back when a link is wedged or USB never came up: every claim
 * released, every link on its default grammar again.
 */
static void button_held(void)
{
    vif_recover_all();
}

static void ble_conn_changed(bool connected)
{
    if (!connected) {
        vif_session_link_down(g_link_ble);
    }
}

/**
 * @brief One control command written to the BLE control characteristic.
 *
 * It acts on the BLE link implicitly: unlike the shell, a client here has a
 * data link of its own and can only mean that one.
 */
static void ble_ctrl_command(const char *cmd, size_t len)
{
    char out[VIF_CTRL_REPLY_MAX];

    vif_ctrl_exec(g_link_ble, cmd, len, out, sizeof(out));
    ble_uart_ctrl_reply(out);
}

/**
 * @brief Every grammar a link can be switched to, and the links themselves.
 */
static void links_setup(void)
{
    elm327_register();
    slcan_register();

    g_link_usb = link_setup(LINK_NAME_USB, g_port_usb_cdc0);
    g_link_ble = link_setup(LINK_NAME_BLE, g_port_ble);

    /*
     * Claims outlive a disconnect; the grammar does not. Both edges are
     * reported by the transports themselves, so nothing has to poll.
     */
    usb_cdc_set_line_callback(usb_line_state_changed);
    ble_uart_set_conn_callback(ble_conn_changed);

    /* The recovery of last resort, wired the same way and in the same place. */
    button_set_hold_callback(button_held);

    /* The control plane for the BLE link. USB reaches it through the shell. */
    ble_uart_ctrl_set_callback(ble_ctrl_command);
}

/**
 * @brief Application main entry point
 */
void app_main(void)
{
    esp_err_t err;
    /*
     * NVS flash initialization
     * Dependency of BLE stack to store configurations
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
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        abort();
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

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
        ESP_LOGE(TAG, "Failed to initialize comm_iface: %s", esp_err_to_name(err));
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

    /* Data links: USB CDC 0 and BLE, both starting on ELM327. */
    links_setup();

    /* start shell */
    shell_loop();

    /* shouldn't be here */
    printf("Shell loop returned. Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
