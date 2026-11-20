/* SPDX-License-Identifier: GPL-3.0-only */
#include "wifi_link.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "nvs.h"

#define TAG "WIFI"
#define WIFI_NVS "wifi"
/* The address WiFi ELM327 adapters use; apps default to it. */
#define WIFI_IP "192.168.0.10"
#define WIFI_CHANNEL 6
#define WIFI_MAX_STATIONS 2
#define WIFI_RX_BUF 256

static bool enabled;
static bool running;
static char ssid[33];
static char pass[WIFI_LINK_PASS_MAX + 1];
static esp_netif_t *ap_netif;
static SemaphoreHandle_t tx_lock;
static volatile int client = -1;
static char client_ip[16];
static wifi_link_rx_cb_t rx_cb;
static wifi_link_conn_cb_t conn_cb;

static void wifi_load(void) {
    nvs_handle_t h;
    uint8_t on = 0;
    size_t n;

    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(ssid, sizeof(ssid), "OpenDiag-%02X%02X", mac[4], mac[5]);

    if (nvs_open(WIFI_NVS, NVS_READONLY, &h) != ESP_OK)
        return;
    if (nvs_get_u8(h, "on", &on) == ESP_OK)
        enabled = on;
    n = sizeof(ssid);
    if (nvs_get_str(h, "ssid", ssid, &n) != ESP_OK)
        snprintf(ssid, sizeof(ssid), "OpenDiag-%02X%02X", mac[4], mac[5]);
    n = sizeof(pass);
    if (nvs_get_str(h, "pass", pass, &n) != ESP_OK)
        pass[0] = '\0';
    nvs_close(h);
}

static esp_err_t wifi_store(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(WIFI_NVS, NVS_READWRITE, &h);

    if (err != ESP_OK)
        return err;
    err = nvs_set_u8(h, "on", enabled);
    if (err == ESP_OK)
        err = nvs_set_str(h, "ssid", ssid);
    if (err == ESP_OK)
        err = nvs_set_str(h, "pass", pass);
    if (err == ESP_OK)
        err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static void wifi_drop_client(void) {
    int s = client;

    if (s >= 0)
        shutdown(s, SHUT_RDWR); /* the server task closes it */
}

static void wifi_serve(int s) {
    uint8_t buf[WIFI_RX_BUF];
    int one = 1, idle = 5, intvl = 2, cnt = 3;

    /* Small writes are the whole protocol: send them as they come. Keepalive
     * notices a laptop that went away without closing. */
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(s, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    setsockopt(s, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(s, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
    setsockopt(s, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));

    client = s;
    if (conn_cb)
        conn_cb(true);
    for (;;) {
        int n = recv(s, buf, sizeof(buf), 0);
        if (n <= 0)
            break;
        if (rx_cb)
            rx_cb(buf, (size_t)n);
    }
    xSemaphoreTake(tx_lock, portMAX_DELAY);
    client = -1;
    xSemaphoreGive(tx_lock);
    close(s);
    ESP_LOGI(TAG, "client %s gone", client_ip);
    client_ip[0] = '\0';
    if (conn_cb)
        conn_cb(false);
}

/* One client at a time: the next one is accepted when this one leaves. */
static void wifi_server_task(void *arg) {
    struct sockaddr_in addr = {.sin_family = AF_INET,
                               .sin_port = htons(WIFI_LINK_PORT),
                               .sin_addr.s_addr = htonl(INADDR_ANY)};
    int one = 1;
    int ls = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    if (ls < 0 ||
        setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one)) != 0 ||
        bind(ls, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(ls, 1) != 0) {
        ESP_LOGE(TAG, "cannot listen on %d", WIFI_LINK_PORT);
        vTaskDelete(NULL);
        return;
    }
    for (;;) {
        struct sockaddr_in peer;
        socklen_t len = sizeof(peer);
        int s = accept(ls, (struct sockaddr *)&peer, &len);

        if (s < 0)
            continue;
        inet_ntoa_r(peer.sin_addr, client_ip, sizeof(client_ip));
        ESP_LOGI(TAG, "client %s", client_ip);
        wifi_serve(s);
    }
}

static esp_err_t wifi_once(void) {
    esp_netif_ip_info_t ip = {0};
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err;

    if (ap_netif)
        return ESP_OK;
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif");
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        return err;
    ap_netif = esp_netif_create_default_wifi_ap();
    ESP_RETURN_ON_FALSE(ap_netif, ESP_FAIL, TAG, "ap netif");

    ip.ip.addr = esp_ip4addr_aton(WIFI_IP);
    ip.gw.addr = ip.ip.addr;
    ip.netmask.addr = esp_ip4addr_aton("255.255.255.0");
    esp_netif_dhcps_stop(ap_netif);
    ESP_RETURN_ON_ERROR(esp_netif_set_ip_info(ap_netif, &ip), TAG, "ip");
    ESP_RETURN_ON_ERROR(esp_netif_dhcps_start(ap_netif), TAG, "dhcps");

    ESP_RETURN_ON_ERROR(esp_wifi_init(&init), TAG, "init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "storage");
    tx_lock = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(tx_lock, ESP_ERR_NO_MEM, TAG, "lock");
    ESP_RETURN_ON_FALSE(xTaskCreate(wifi_server_task, "wifi_link", 4096, NULL,
                                    5, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");
    return ESP_OK;
}

static esp_err_t wifi_start(void) {
    wifi_config_t wc = {
        .ap = {.channel = WIFI_CHANNEL,
               .max_connection = WIFI_MAX_STATIONS,
               .authmode = WIFI_AUTH_WPA2_PSK},
    };

    if (strlen(pass) < WIFI_LINK_PASS_MIN)
        return ESP_ERR_INVALID_STATE; /* never an open access point */
    ESP_RETURN_ON_ERROR(wifi_once(), TAG, "setup");
    strlcpy((char *)wc.ap.ssid, ssid, sizeof(wc.ap.ssid));
    wc.ap.ssid_len = strlen(ssid);
    strlcpy((char *)wc.ap.password, pass, sizeof(wc.ap.password));
    if (running)
        esp_wifi_stop();
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "mode");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &wc), TAG, "config");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "start");
    running = true;
    ESP_LOGI(TAG, "access point %s, %s:%d", ssid, WIFI_IP, WIFI_LINK_PORT);
    return ESP_OK;
}

static void wifi_stop(void) {
    if (!running)
        return;
    wifi_drop_client();
    esp_wifi_stop();
    running = false;
}

void wifi_link_setup(void) {
    wifi_load();
    if (enabled && wifi_start() != ESP_OK)
        ESP_LOGE(TAG, "not started: set a password with 'wifi pass'");
}

void wifi_link_set_callbacks(wifi_link_rx_cb_t rx, wifi_link_conn_cb_t conn) {
    rx_cb = rx;
    conn_cb = conn;
}

esp_err_t wifi_link_set_enabled(bool on) {
    esp_err_t err = ESP_OK;

    if (on)
        err = wifi_start();
    else
        wifi_stop();
    if (err != ESP_OK)
        return err;
    enabled = on;
    return wifi_store();
}

esp_err_t wifi_link_set_ssid(const char *s) {
    size_t n = strlen(s);

    if (n < 1 || n >= sizeof(ssid))
        return ESP_ERR_INVALID_ARG;
    strlcpy(ssid, s, sizeof(ssid));
    if (running)
        wifi_start();
    return wifi_store();
}

esp_err_t wifi_link_set_password(const char *p) {
    size_t n = strlen(p);

    if (n < WIFI_LINK_PASS_MIN || n > WIFI_LINK_PASS_MAX)
        return ESP_ERR_INVALID_ARG;
    strlcpy(pass, p, sizeof(pass));
    if (running)
        wifi_start(); /* stations must join again with the new one */
    return wifi_store();
}

void wifi_link_print_status(void) {
    wifi_sta_list_t sta = {0};

    if (running)
        esp_wifi_ap_get_sta_list(&sta);
    printf("wifi: %s, ssid %s, password %s\n",
           running ? "on" : (enabled ? "enabled, not running" : "off"), ssid,
           strlen(pass) >= WIFI_LINK_PASS_MIN ? "set" : "NOT SET");
    if (running)
        printf("  %s:%d, %d station(s), client %s\n", WIFI_IP, WIFI_LINK_PORT,
               sta.num, client >= 0 ? client_ip : "none");
}

bool wifi_link_is_connected(void) { return client >= 0; }

int wifi_link_write(const void *buf, uint32_t len) {
    const uint8_t *p = buf;
    uint32_t done = 0;

    if (!tx_lock)
        return -1;
    xSemaphoreTake(tx_lock, portMAX_DELAY);
    while (client >= 0 && done < len) {
        int n = send(client, p + done, len - done, 0);
        if (n <= 0) {
            wifi_drop_client();
            break;
        }
        done += (uint32_t)n;
    }
    xSemaphoreGive(tx_lock);
    return done ? (int)done : -1;
}
