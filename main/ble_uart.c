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
#include "nimble/nimble_port.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nvs_flash.h"
#include "common.h"
#include "ble_uart.h"

#define TAG "BLE_UART"

#define DEVICE_NAME "OpenDIAG"
#define BLE_GAP_APPEARANCE_GENERIC_TAG 0x0200
#define BLE_GAP_URI_PREFIX_HTTPS 0x17
#define BLE_GAP_LE_ROLE_PERIPHERAL 0x00

extern void ble_store_config_init(void);


static interface_rx_cb_t ble_rx_callback;


/* {6E400001-B5A3-F393-E0A9-E50E24DCCA9E} */
static const ble_uuid128_t gatt_svr_svc_uart_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

/* {6E400002-B5A3-F393-E0A9-E50E24DCCA9E} */
static const ble_uuid128_t gatt_svr_chr_uart_write_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);


/* {6E400003-B5A3-F393-E0A9-E50E24DCCA9E} */
static const ble_uuid128_t gatt_svr_chr_uart_read_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

/*
 * {6E400004-B5A3-F393-E0A9-E50E24DCCA9E}: the control plane.
 *
 * A sibling of the UART pair rather than a second service, because it is not
 * a second data pipe: it has no session and no front-end of its own, which is
 * what keeps it reachable whatever grammar the data characteristics are
 * carrying. One command per write - GATT already frames each write as a whole
 * message, so there is no line assembly here and no terminator to agree on.
 */
static const ble_uuid128_t gatt_svr_chr_ctrl_uuid =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
                     0x93, 0xf3, 0xa3, 0xb5, 0x04, 0x00, 0x40, 0x6e);

// Connection handle for the current connection
static uint16_t conn_handle;

/* ble uart attr read handle */
static uint16_t g_bleuart_attr_read_handle;

/* ble uart attr write handle */
static uint16_t g_bleuart_attr_write_handle;

/* control characteristic handle */
static uint16_t g_blectrl_attr_handle;

static ble_uart_ctrl_cb_t ble_ctrl_callback;
static ble_uart_conn_cb_t ble_conn_callback;

static uint8_t own_addr_type;
static uint8_t addr_val[6] = {0};
static volatile uint8_t connected;

static void start_advertising(void);


// GATT server access callback function
static int gatt_svr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        ESP_LOGI(TAG, "Characteristic write, conn_handle %d, attr_handle %d", conn_handle, attr_handle);

        if (attr_handle == g_blectrl_attr_handle) {
            if (ble_ctrl_callback) {
                ble_ctrl_callback((const char *)ctxt->om->om_data,
                                  ctxt->om->om_len);
            }
            return 0;
        }

        if (ble_rx_callback) {
            ble_rx_callback(ctxt->om->om_data, ctxt->om->om_len);
        }
        return 0;
    default:
        return BLE_ATT_ERR_UNLIKELY;
    }
}


/* GATT services table */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /* Heart rate service */
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uart_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) { 
            {
                .uuid = &gatt_svr_chr_uart_read_uuid.u,
                .val_handle = &g_bleuart_attr_read_handle,
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            {
                /* Characteristic: Write */
                .uuid = &gatt_svr_chr_uart_write_uuid.u,
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &g_bleuart_attr_write_handle,
            },
            {
                /* Characteristic: control plane, in and out */
                .uuid = &gatt_svr_chr_ctrl_uuid.u,
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP |
                         BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &g_blectrl_attr_handle,
            },
            { 0 } /* No more characteristics in this service */
        },
    },
    { 0 } /* No more services */
};


static void nimble_host_task(void *param) {
    /* Task entry log */
    ESP_LOGI(TAG, "nimble host task has been started!");

    /* This function won't return until nimble_port_stop() is executed */
    nimble_port_run();

    /* Clean up at exit */
    vTaskDelete(NULL);
}

static int gap_init(void) {
    /* Local variables */
    int rc = 0;

    /* Initialize GAP service */
    ble_svc_gap_init();

    /* Set GAP device name */
    rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set device name to %s, error code: %d",
                 DEVICE_NAME, rc);
        return rc;
    }

    /* Set GAP device appearance */
    rc = ble_svc_gap_device_appearance_set(BLE_GAP_APPEARANCE_GENERIC_TAG);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set device appearance, error code: %d", rc);
        return rc;
    }
    return rc;
}

/*
 *  GATT server initialization
 *      1. Initialize GATT service
 *      2. Update NimBLE host GATT services counter
 *      3. Add GATT services to server
 */
 int gatt_svc_init(void) {
    /* Local variables */
    int rc;

    /* 1. GATT service initialization */
    ble_svc_gatt_init();

    /* 2. Update GATT services counter */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    /* 3. Add GATT services */
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

inline static void format_addr(char *addr_str, uint8_t addr[]) {
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1],
            addr[2], addr[3], addr[4], addr[5]);
}

static void print_conn_desc(struct ble_gap_conn_desc *desc) {
    /* Local variables */
    char addr_str[18] = {0};

    /* Connection handle */
    ESP_LOGI(TAG, "connection handle: %d", desc->conn_handle);

    /* Local ID address */
    format_addr(addr_str, desc->our_id_addr.val);
    ESP_LOGI(TAG, "device id address: type=%d, value=%s",
             desc->our_id_addr.type, addr_str);

    /* Peer ID address */
    format_addr(addr_str, desc->peer_id_addr.val);
    ESP_LOGI(TAG, "peer id address: type=%d, value=%s", desc->peer_id_addr.type,
             addr_str);

    /* Connection info */
    ESP_LOGI(TAG,
             "conn_itvl=%d, conn_latency=%d, supervision_timeout=%d, "
             "encrypted=%d, authenticated=%d, bonded=%d\n",
             desc->conn_itvl, desc->conn_latency, desc->supervision_timeout,
             desc->sec_state.encrypted, desc->sec_state.authenticated,
             desc->sec_state.bonded);
}



/*
 * NimBLE applies an event-driven model to keep GAP service going
 * gap_event_handler is a callback function registered when calling
 * ble_gap_adv_start API and called when a GAP event arrives
 */
 static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    /* Local variables */
    int rc = 0;
    struct ble_gap_conn_desc desc;

    /* Handle different GAP event */
    switch (event->type) {

    /* Connect event */
    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        ESP_LOGI(TAG, "connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);

        /* Connection succeeded */
        if (event->connect.status == 0) {
            /* Check connection handle */
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc != 0) {
                ESP_LOGE(TAG,
                         "failed to find connection by handle, error code: %d",
                         rc);
                return rc;
            }

            /* Print connection descriptor and turn on the LED */
            print_conn_desc(&desc);

            connected = 1;
            conn_handle = event->connect.conn_handle;

            if (ble_conn_callback) {
                ble_conn_callback(true);
            }

            // /* Try to update connection parameters */
            // struct ble_gap_upd_params params = {.itvl_min = desc.conn_itvl,
            //                                     .itvl_max = desc.conn_itvl,
            //                                     .latency = 3,
            //                                     .supervision_timeout =
            //                                         desc.supervision_timeout};
            // rc = ble_gap_update_params(event->connect.conn_handle, &params);
            // if (rc != 0) {
            //     ESP_LOGE(
            //         TAG,
            //         "failed to update connection parameters, error code: %d",
            //         rc);
            //     return rc;
            // }

            // Sets the client's BLE connection behaviours 
            // https://mynewt.apache.org/latest/network/ble_hs/ble_gap.html#c.ble_gap_update_params
            // ITVL uses 1.25 ms units
            // Timout is in 10ms units
            // CE LEN uses 0.625 ms units
            // BLE specifies minimum 7.5ms connection interval
            struct ble_gap_upd_params conn_parameters = { 0 };
            conn_parameters.itvl_min = 6;   // 7.5ms
            conn_parameters.itvl_max = 24;  // 30ms
            conn_parameters.latency = 0;
            conn_parameters.supervision_timeout = 20; 
            // https://github.com/apache/mynewt-nimble/issues/793#issuecomment-616022898
            conn_parameters.min_ce_len = 0x00;
            conn_parameters.max_ce_len = 0x00;

            ble_gap_update_params(event->connect.conn_handle, &conn_parameters);
            if (rc != 0) {
                ESP_LOGE(TAG, "failed to ble_gap_update_params, error code: %d", rc);
                abort();
            }
        }
        /* Connection failed, restart advertising */
        else {
            start_advertising();
        }
        return rc;

    /* Disconnect event */
    case BLE_GAP_EVENT_DISCONNECT:
        /* A connection was terminated, print connection descriptor */
        ESP_LOGI(TAG, "disconnected from peer; reason=%d",
                 event->disconnect.reason);

        connected = 0;

        /*
         * The link's client is gone. vif puts the grammar back to the link
         * default and leaves every claim standing - a programming voltage has
         * to survive a dropped connection, a parser the next client did not
         * ask for must not.
         */
        if (ble_conn_callback) {
            ble_conn_callback(false);
        }

        /* Restart advertising */
        start_advertising();
        return rc;

    /* Connection parameters update event */
    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        ESP_LOGI(TAG, "connection updated; status=%d",
                 event->conn_update.status);

        /* Print connection descriptor */
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (rc != 0) {
            ESP_LOGE(TAG, "failed to find connection by handle, error code: %d",
                     rc);
            return rc;
        }
        print_conn_desc(&desc);
        return rc;
    }

    return rc;
}

static void start_advertising(void) {
    /* Local variables */
    int rc = 0;
    const char *name;
    struct ble_hs_adv_fields adv_fields = {0};
    struct ble_hs_adv_fields rsp_fields = {0};
    struct ble_gap_adv_params adv_params = {0};
    static uint8_t esp_uri[] = {BLE_GAP_URI_PREFIX_HTTPS, '/', '/', 'e', 's', 'p', 'r', 'e', 's', 's', 'i', 'f', '.', 'c', 'o', 'm'};


    /* Set advertising flags */
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Set device name */
    name = ble_svc_gap_device_name();
    adv_fields.name = (uint8_t *)name;
    adv_fields.name_len = strlen(name);
    adv_fields.name_is_complete = 1;

    /* Set device tx power */
    adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    adv_fields.tx_pwr_lvl_is_present = 1;

    /* Set device appearance */
    adv_fields.appearance = BLE_GAP_APPEARANCE_GENERIC_TAG;
    adv_fields.appearance_is_present = 1;

    /* Set device LE role */
    adv_fields.le_role = BLE_GAP_LE_ROLE_PERIPHERAL;
    adv_fields.le_role_is_present = 1;

    /* Set advertiement fields */
    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set advertising data, error code: %d", rc);
        return;
    }

    /* Set device address */
    rsp_fields.device_addr = addr_val;
    rsp_fields.device_addr_type = own_addr_type;
    rsp_fields.device_addr_is_present = 1;

    /* Set URI */
    rsp_fields.uri = esp_uri;
    rsp_fields.uri_len = sizeof(esp_uri);

    /* Set advertising interval */
    rsp_fields.adv_itvl = BLE_GAP_ADV_ITVL_MS(500);
    rsp_fields.adv_itvl_is_present = 1;

    /* Set scan response fields */
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to set scan response data, error code: %d", rc);
        return;
    }

    /* Set non-connetable and general discoverable mode to be a beacon */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    /* Set advertising interval */
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(500);
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(510);

    /* Start advertising */
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params,
                           gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to start advertising, error code: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "advertising started!");
}

/* Public functions */
void adv_init(void) {
    /* Local variables */
    int rc = 0;
    char addr_str[18] = {0};

    /* Make sure we have proper BT identity address set */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "device does not have any available bt address!");
        return;
    }

    /* Figure out BT address to use while advertising */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to infer address type, error code: %d", rc);
        return;
    }

    /* Copy device address to addr_val */
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to copy device address, error code: %d", rc);
        return;
    }
    format_addr(addr_str, addr_val);
    ESP_LOGI(TAG, "device address: %s", addr_str);

    /* Start advertising. */
    start_advertising();
}

/* Private functions */
/*
 *  Stack event callback functions
 *      - on_stack_reset is called when host resets BLE stack due to errors
 *      - on_stack_sync is called when host has synced with controller
 */
static void on_stack_reset(int reason) {
    /* On reset, print reset reason to console */
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_stack_sync(void) {
    /* On stack sync, do advertising initialization */
    adv_init();
}


static void nimble_host_config_init(void) {
    /* Set host callbacks */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Store host configuration */
    ble_store_config_init();
}

void ble_uart_rx_set_callback(interface_rx_cb_t callback)
{
    ble_rx_callback = callback;
}


bool ble_uart_is_connected(void)
{
    return connected != 0;
}


/**
 * @brief Notify @p len bytes on @p handle, split to fit the negotiated MTU.
 *
 * ble_gatts_notify_custom() truncates anything past ATT_MTU - 3 rather than
 * failing, so a single notification per response silently lost everything
 * beyond 20 bytes on a central that never negotiated a larger MTU. Every
 * caller here can be longer than that: a multi frame ELM327 answer certainly
 * is, and so is the control plane's ID.
 */
static void ble_notify_chunked(uint16_t handle, const char *buffer, size_t size)
{
    uint16_t mtu;
    size_t chunk;

    if (!connected || !handle || !buffer || !size) {
        return;
    }

    mtu = ble_att_mtu(conn_handle);

    /* Before the exchange the stack reports the default 23. Three bytes go to
     * the notification header. */
    chunk = (mtu > 3) ? (size_t)(mtu - 3) : 20;

    while (size) {
        size_t n = size < chunk ? size : chunk;
        struct os_mbuf *om = ble_hs_mbuf_from_flat(buffer, n);
        int rc;

        if (!om) {
            ESP_LOGE(TAG, "out of mbufs; %u bytes dropped", (unsigned)size);
            return;
        }

        rc = ble_gatts_notify_custom(conn_handle, handle, om);
        if (rc) {
            /* The mbuf is consumed either way, so there is nothing to free. */
            ESP_LOGE(TAG, "Error sending notification; rc=%d", rc);
            return;
        }

        buffer += n;
        size -= n;
    }
}

void ble_uart_send (const char *buffer, size_t size) {
    ble_notify_chunked(g_bleuart_attr_read_handle, buffer, size);
}

void ble_uart_ctrl_reply(const char *text)
{
    if (text) {
        ble_notify_chunked(g_blectrl_attr_handle, text, strlen(text));
    }
}

void ble_uart_ctrl_set_callback(ble_uart_ctrl_cb_t cb)
{
    ble_ctrl_callback = cb;
}

void ble_uart_set_conn_callback(ble_uart_conn_cb_t cb)
{
    ble_conn_callback = cb;
}

void ble_uart_setup(void) {
    esp_err_t ret;
    int rc;

    /* NimBLE stack initialization */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to initialize nimble stack, error code: %d ",
                 ret);
        return;
    }

    /* GAP service initialization */
    rc = gap_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GAP service, error code: %d", rc);
        return;
    }

    /* GATT server initialization */
    rc = gatt_svc_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "failed to initialize GATT server, error code: %d", rc);
        return;
    }

    /* NimBLE host configuration initialization */
    nimble_host_config_init();

    /* Start NimBLE host task thread */
    xTaskCreate(nimble_host_task, "NimBLE Host", 4*1024, NULL, 5, NULL);
}
