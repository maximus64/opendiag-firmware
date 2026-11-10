/* SPDX-License-Identifier: GPL-3.0-only */
#include "power.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "ble_uart.h"
#include "comm_iface.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/rtc_io.h"
#include "nvs.h"
#include "pinout.h"
#include "usb_cdc.h"
#include "vif.h"
#include "ws2812_led.h"

#define TAG "POWER"

#define POWER_TICK_MS 1000
/* How often a sleeping adapter looks at the battery. Each look is a short
 * boot; once a minute keeps the average draw well under a milliamp while
 * still catching an engine that started without any CAN traffic. */
#define POWER_TIMER_WAKE_S 60
#define POWER_NVS "power"

#define POWER_IDLE_MIN_MAX 240
#define POWER_RUN_MV_MIN 12000
#define POWER_RUN_MV_MAX 15500

/*
 * ponytail: 13.2 V suits an ordinary alternator. A "smart" alternator can
 * sit at 12.5 V with the engine running; 'power vrun' is the knob for that.
 */
static power_cfg_t cfg = {.enabled = true, .idle_s = 10 * 60, .run_mv = 13200};
static power_state_t state;
static pcnt_unit_handle_t edges;
static const char *wake_cause = "power-on";

/*
 * Every output, at the idle level board_setup() gives it. They are held
 * through deep sleep, where the pads would otherwise float: a floating gate
 * on the low side switch could ground pin 15, a floating driver input could
 * light a high side pin.
 */
static const gpio_num_t held_outputs[] = {
    PIN_J1850_MODE,  PIN_J1850_TX_P, PIN_J1850_TX_N,        PIN_CAN0_SILENT,
    PIN_CAN1_SILENT, PIN_KLINE_TX,   PIN_KLINE_nSILENT,     PIN_HS_BOOST_EN,
    PIN_HS_OBD_6,    PIN_HS_OBD_9,   PIN_HS_OBD_11,         PIN_HS_OBD_12,
    PIN_HS_OBD_13,   PIN_HS_OBD_14,  PIN_HS_VOLTAGE_ADJUST, PIN_LS_OBD_15,
    PIN_WS2812_LED,
#if OPENDIAG_HW_LS_OBD_9
    PIN_LS_OBD_9,
#endif
};

#define WAKE_PINS (BIT64(PIN_CAN0_RX) | BIT64(PIN_CAN1_RX) | BIT64(PIN_BUTTON))

static void power_load(void) {
    nvs_handle_t h;
    uint8_t on;
    uint32_t idle;
    int32_t run;

    if (nvs_open(POWER_NVS, NVS_READONLY, &h) != ESP_OK)
        return; /* never stored: the defaults */
    if (nvs_get_u8(h, "auto", &on) == ESP_OK)
        cfg.enabled = on;
    if (nvs_get_u32(h, "idle_s", &idle) == ESP_OK)
        cfg.idle_s = idle;
    if (nvs_get_i32(h, "run_mv", &run) == ESP_OK)
        cfg.run_mv = run;
    nvs_close(h);
}

static esp_err_t power_store(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(POWER_NVS, NVS_READWRITE, &h);

    if (err != ESP_OK)
        return err;
    err = nvs_set_u8(h, "auto", cfg.enabled);
    if (err == ESP_OK)
        err = nvs_set_u32(h, "idle_s", cfg.idle_s);
    if (err == ESP_OK)
        err = nvs_set_i32(h, "run_mv", cfg.run_mv);
    if (err == ESP_OK)
        err = nvs_commit(h);
    nvs_close(h);
    return err;
}

static bool power_client(void) {
    return vif_link_port() != COMM_INVALID_PORT_ID || usb_cdc_host_present() ||
           ble_uart_is_connected();
}

static bool power_busy(void) {
    vif_bus_claim_t claims[VIF_BUS_GROUPS];

    vif_bus_info(claims);
    for (size_t i = 0; i < VIF_BUS_GROUPS; i++) {
        if (claims[i].bus != VIF_BUS_NONE)
            return true;
    }
    return vif_any_pin_active();
}

/* Falling edges on either CAN receiver since the last call. The counter
 * listens through the GPIO matrix, beside the TWAI controller, so it sees
 * the bus whether or not anybody has it open. */
static bool power_traffic(void) {
    int n = 0;

    if (!edges)
        return false;
    pcnt_unit_get_count(edges, &n);
    pcnt_unit_clear_count(edges);
    return n != 0;
}

static void power_enter_sleep(bool led_running) {
    if (led_running) {
        ws2812_led_set_state(LED_STATE_OFF);
        vTaskDelay(pdMS_TO_TICKS(50)); /* let the LED task send it */
    }
    for (size_t i = 0; i < sizeof(held_outputs) / sizeof(held_outputs[0]); i++)
        gpio_hold_en(held_outputs[i]);
    gpio_deep_sleep_hold_en();

    /* The CAN receivers are driven by their transceivers; the button pin
     * needs a pull-up, which needs the RTC peripherals powered. */
    rtc_gpio_pullup_en(PIN_BUTTON);
    rtc_gpio_pulldown_dis(PIN_BUTTON);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_sleep_enable_ext1_wakeup_io(WAKE_PINS, ESP_EXT1_WAKEUP_ANY_LOW);
    esp_sleep_enable_timer_wakeup((uint64_t)POWER_TIMER_WAKE_S * 1000000u);

    ESP_LOGW(TAG, "deep sleep, vbatt %ld mV", (long)vif_vbatt_mv());
    fflush(stdout);
    esp_deep_sleep_start();
}

void power_boot(void) {
    uint32_t causes = esp_sleep_get_wakeup_causes();

    power_load();

    /* board_setup() has already put every pin where it was held, so letting
     * go here changes nothing on the connector. */
    gpio_deep_sleep_hold_dis();
    for (size_t i = 0; i < sizeof(held_outputs) / sizeof(held_outputs[0]); i++)
        gpio_hold_dis(held_outputs[i]);

    if (causes & BIT(ESP_SLEEP_WAKEUP_EXT1)) {
        wake_cause = (esp_sleep_get_ext1_wakeup_status() & BIT64(PIN_BUTTON))
                         ? "button"
                         : "CAN traffic";
    } else if (causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) {
        wake_cause = "timer";
        if (power_policy_back_to_sleep(&cfg, vif_vbatt_mv()))
            power_enter_sleep(false); /* no LED task yet; it is already off */
        ESP_LOGI(TAG, "woke on the timer with %ld mV: staying up",
                 (long)vif_vbatt_mv());
    }
}

static void power_task(void *arg) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(POWER_TICK_MS));

        power_sample_t s = {
            .client = power_client(),
            .busy = power_busy(),
            .bus_traffic = power_traffic(),
            .vbatt_mv = vif_vbatt_mv(),
        };
        if (!power_policy_tick(&state, &cfg, &s, POWER_TICK_MS / 1000))
            continue;
        /* Last look: a client may have arrived since the sample. */
        if (power_client() || power_busy()) {
            state.quiet_s = 0;
            continue;
        }
        power_enter_sleep(true);
    }
}

static esp_err_t power_edges_setup(void) {
    const pcnt_unit_config_t unit = {.low_limit = -1, .high_limit = 32767};
    const pcnt_glitch_filter_config_t filter = {.max_glitch_ns = 200};
    const int pins[] = {PIN_CAN0_RX, PIN_CAN1_RX};
    esp_err_t err = pcnt_new_unit(&unit, &edges);

    if (err == ESP_OK)
        err = pcnt_unit_set_glitch_filter(edges, &filter);
    for (size_t i = 0; err == ESP_OK && i < 2; i++) {
        const pcnt_chan_config_t chan = {.edge_gpio_num = pins[i],
                                         .level_gpio_num = -1};
        pcnt_channel_handle_t ch;

        err = pcnt_new_channel(edges, &chan, &ch);
        if (err == ESP_OK)
            err =
                pcnt_channel_set_edge_action(ch, PCNT_CHANNEL_EDGE_ACTION_HOLD,
                                             PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    }
    if (err == ESP_OK)
        err = pcnt_unit_enable(edges);
    if (err == ESP_OK)
        err = pcnt_unit_start(edges);
    return err;
}

esp_err_t power_start(void) {
    esp_err_t err = power_edges_setup();

    if (err != ESP_OK) {
        /* Without it a busy bus looks quiet, so do not sleep at all. */
        ESP_LOGE(TAG, "no CAN edge counter (%s): sleep disabled",
                 esp_err_to_name(err));
        edges = NULL;
        cfg.enabled = false;
        return err;
    }
    return xTaskCreate(power_task, "power", 3072, NULL, 2, NULL) == pdPASS
               ? ESP_OK
               : ESP_ERR_NO_MEM;
}

power_cfg_t power_cfg(void) { return cfg; }

esp_err_t power_set_enabled(bool on) {
    cfg.enabled = on;
    state.quiet_s = 0;
    return power_store();
}

esp_err_t power_set_idle_minutes(uint32_t minutes) {
    if (minutes < 1 || minutes > POWER_IDLE_MIN_MAX)
        return ESP_ERR_INVALID_ARG;
    cfg.idle_s = minutes * 60;
    state.quiet_s = 0;
    return power_store();
}

esp_err_t power_set_run_mv(int32_t mv) {
    if (mv < POWER_RUN_MV_MIN || mv > POWER_RUN_MV_MAX)
        return ESP_ERR_INVALID_ARG;
    cfg.run_mv = mv;
    return power_store();
}

void power_print_status(void) {
    printf("power: sleep %s, after %lu min idle, engine running at >= %ld "
           "mV\n",
           cfg.enabled ? "on" : "off", (unsigned long)(cfg.idle_s / 60),
           (long)cfg.run_mv);
    printf("  vbatt %ld mV, quiet %lu s, staying up for: %s\n",
           (long)vif_vbatt_mv(), (unsigned long)state.quiet_s,
           power_hold_name(state.hold));
    printf("  last wake-up: %s\n", wake_cause);
}

esp_err_t power_sleep_now(void) {
    /* The console asking is itself a client, so only claims refuse. */
    if (power_busy())
        return ESP_ERR_INVALID_STATE;
    power_enter_sleep(true);
    return ESP_OK; /* not reached */
}
