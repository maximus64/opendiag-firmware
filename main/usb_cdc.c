/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"

#include "usb_cdc.h"

#define TAG "USB_CDC"

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN * 2)
#define USB_CDC_INTERFACE_NUM 2

static volatile bool line_status[USB_CDC_INTERFACE_NUM];
static interface_rx_cb_t tx_callback[USB_CDC_INTERFACE_NUM];
static usb_cdc_line_cb_t line_callback;

enum {
    ITF_NUM_CDC_0 = 0,
    ITF_NUM_CDC_0_DATA,
    ITF_NUM_CDC_1,
    ITF_NUM_CDC_1_DATA,
    ITF_NUM_TOTAL
};

enum {
    EPNUM_CDC_0_NOTIF = 0x81,
    EPNUM_CDC_0_OUT   = 0x02,
    EPNUM_CDC_0_IN    = 0x82,
    EPNUM_CDC_1_NOTIF = 0x83,
    EPNUM_CDC_1_OUT   = 0x04,
    EPNUM_CDC_1_IN    = 0x84,
};

static const tusb_desc_device_t cdc_device_descriptor = {
    .bLength = sizeof(cdc_device_descriptor),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_ESPRESSIF_VID,
    .idProduct = 0x4002,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

static const uint8_t desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // 1st CDC: Interface number, string index, EP notification, EP notification size, EP data out, EP data in, EP data packet size
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_0, 4, EPNUM_CDC_0_NOTIF, 8, EPNUM_CDC_0_OUT, EPNUM_CDC_0_IN, 64),

    // 2nd CDC: Interface number, string index, EP notification, EP notification size, EP data out, EP data in, EP data packet size
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 5, EPNUM_CDC_1_NOTIF, 8, EPNUM_CDC_1_OUT, EPNUM_CDC_1_IN, 64),
};

static char serial_str[13] = "000000000000";

static const char *string_desc_arr[] = {
    (static const char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
    "Dalalogic",          // 1: Manufacturer
    "OpenDiag",           // 2: Product
    serial_str,           // 3: Serials,
    "OpenDiag Data",      // 4: CDC 0 - the protocol data link
    "Console",            // 5: CDC 1 - console and debug shell
};

static const tinyusb_config_t tusb_cfg = {
    .device_descriptor = NULL,
    .string_descriptor = string_desc_arr,
    .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
    .external_phy = false, // In the most cases you need to use a `false` value
    .configuration_descriptor = desc_configuration,
};


/**
 * @brief Invoked when a line state change event occurs
 *
 * This callback is used to detect when a serial terminal is connected or disconnected.
 */
static void tinyusb_cdc_line_state_cb(int itf, cdcacm_event_t *event)
{
    bool open = false;

    if (event->line_state_changed_data.dtr && event->line_state_changed_data.rts) {
        ESP_LOGI(TAG, "Serial terminal connected on CDC interface %d", itf);
        open = true;
    } else {
        ESP_LOGI(TAG, "Serial terminal disconnected on CDC interface %d", itf);
    }

    line_status[itf] = open;

    if (line_callback) {
        line_callback(itf, open);
    }
}

/**
 * @brief Invoked when received new data
 *
 * This callback is triggered when the host sends data to either of the CDC interfaces.
 * The `itf` parameter is used to distinguish between the two interfaces.
 */
static void tinyusb_cdc_rx_cb(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;
    uint8_t rx_buf[64];

    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(itf, rx_buf, sizeof(rx_buf), &rx_size);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CDC RX read error. ret = %d\n", ret);
        return;
    }

    if (tx_callback[itf]) {
        tx_callback[itf](rx_buf, rx_size);
    }
}

void usb_cdc_set_line_callback(usb_cdc_line_cb_t callback)
{
    line_callback = callback;
}

void usb_cdc_rx_set_callback(int itf, interface_rx_cb_t callback)
{
    assert(itf < USB_CDC_INTERFACE_NUM);
    tx_callback[itf] = callback;
}

int usb_cdc_tx_write(int itf, const void* buf, uint32_t length)
{
    assert(itf < USB_CDC_INTERFACE_NUM);
    return tinyusb_cdcacm_write_queue(itf, buf, length);
}

void usb_cdc_tx_flush(int itf)
{
    assert(itf < USB_CDC_INTERFACE_NUM);
    tinyusb_cdcacm_write_flush(itf, 0);
}

bool usb_cdc_get_line_status(int itf)
{
    assert(itf < USB_CDC_INTERFACE_NUM);
    return line_status[itf];
}

void usb_cdc_setup(void)
{
    ESP_LOGI(TAG, "USB initialization");

    // Populate serial string with MAC address
    uint8_t base_mac[6] = {0};
    if (esp_efuse_mac_get_default(base_mac) == ESP_OK) {
        snprintf(serial_str, sizeof(serial_str), "%02X%02X%02X%02X%02X%02X",
                 base_mac[0], base_mac[1], base_mac[2],
                 base_mac[3], base_mac[4], base_mac[5]);
        ESP_LOGI(TAG, "USB serial (eFuse MAC): %s", serial_str);
    } else {
        ESP_LOGW(TAG, "Failed to get MAC address for USB serial number");
    }

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    ESP_ERROR_CHECK(esp_register_shutdown_handler(usb_cdc_teardown));

    /* CDC 0 is the data link: its receive callback drains the FIFO and hands
     * the bytes to comm_iface. */
    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = tinyusb_cdc_rx_cb,
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = tinyusb_cdc_line_state_cb,
        .callback_line_coding_changed = NULL
    };
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

    /*
     * CDC 1 is the console, and gets NO receive callback.
     *
     * esp_tusb_init_console() puts a VFS reader on this interface, and that
     * reader takes bytes straight out of the TinyUSB FIFO with
     * tud_cdc_n_read_char(). A callback here would run first, drain the same
     * FIFO through tinyusb_cdcacm_read(), and - having nothing to hand the
     * bytes to - discard them. The console would print but never read a key.
     */
    acm_cfg.cdc_port = TINYUSB_CDC_ACM_1;
    acm_cfg.callback_rx = NULL;
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));

    ESP_LOGI(TAG, "USB initialization finished.");
}

void usb_cdc_teardown(void)
{
    ESP_LOGI(TAG, "USB taredown...");
    tinyusb_driver_uninstall();
}
