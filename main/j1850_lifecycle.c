/* SPDX-License-Identifier: GPL-3.0-only */
#include "j1850_lifecycle.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

esp_err_t j1850_callbacks_stop(j1850_callback_guard_t *guard) {
    __atomic_store_n(&guard->stopping, true, __ATOMIC_SEQ_CST);
    TickType_t start = xTaskGetTickCount();
    while (__atomic_load_n(&guard->active, __ATOMIC_SEQ_CST)) {
        if (xTaskGetTickCount() - start >= pdMS_TO_TICKS(100)) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }
    return ESP_OK;
}

esp_err_t j1850_rmt_enable(rmt_channel_handle_t channel, bool *enabled) {
    esp_err_t err = rmt_enable(channel);
    if (err == ESP_OK)
        *enabled = true;
    return err;
}

static esp_err_t rmt_stop(rmt_channel_handle_t channel, bool *enabled) {
    if (!*enabled)
        return ESP_OK;
    esp_err_t err = rmt_disable(channel);
    if (err == ESP_OK)
        *enabled = false;
    return err;
}

esp_err_t j1850_rmt_delete(rmt_channel_handle_t *channel, bool *enabled) {
    if (!*channel)
        return ESP_OK;
    esp_err_t err = rmt_stop(*channel, enabled);
    if (err != ESP_OK)
        return err;
    err = rmt_del_channel(*channel);
    if (err == ESP_OK)
        *channel = NULL;
    return err;
}

esp_err_t j1850_rmt_recover(rmt_channel_handle_t channel, bool *enabled) {
    esp_err_t err = rmt_stop(channel, enabled);
    if (err != ESP_OK)
        return err;
    /* Drain the aborted transaction before accepting another payload. */
    err = rmt_tx_wait_all_done(channel, 100);
    if (err != ESP_OK)
        return err;
    return j1850_rmt_enable(channel, enabled);
}

esp_err_t j1850_timer_enable(gptimer_handle_t timer, bool *enabled) {
    esp_err_t err = gptimer_enable(timer);
    if (err == ESP_OK)
        *enabled = true;
    return err;
}

esp_err_t j1850_timer_start(gptimer_handle_t timer, bool *running) {
    esp_err_t err = gptimer_start(timer);
    if (err == ESP_OK)
        *running = true;
    return err;
}

esp_err_t j1850_timer_delete(gptimer_handle_t *timer, bool *enabled,
                             bool *running) {
    if (!*timer)
        return ESP_OK;
    esp_err_t err = gptimer_set_alarm_action(*timer, NULL);
    if (err != ESP_OK)
        return err;
    if (*running) {
        err = gptimer_stop(*timer);
        if (err != ESP_OK)
            return err;
        *running = false;
    }
    if (*enabled) {
        err = gptimer_disable(*timer);
        if (err != ESP_OK)
            return err;
        *enabled = false;
    }
    err = gptimer_del_timer(*timer);
    if (err == ESP_OK)
        *timer = NULL;
    return err;
}
