/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "pinout.h"
#include "can_bus.h"

#define TAG "CANBUS"

#define CAN_RX_QUEUE_LEN 32

static twai_handle_t can_bus_handle;
static QueueHandle_t rx_frame_queue;
static volatile TaskHandle_t can_bus_task_handle = NULL;
static volatile bool task_should_exit = false;
static SemaphoreHandle_t task_exit_semaphore = NULL;

static const char* can_get_state_str(int state)
{
    switch (state) {
        case TWAI_STATE_STOPPED:
            return "TWAI_STATE_STOPPED";
            break;
        case TWAI_STATE_RUNNING:
            return "TWAI_STATE_RUNNING";
        case TWAI_STATE_BUS_OFF:
            return "TWAI_STATE_BUS_OFF";
        case TWAI_STATE_RECOVERING:
            return "TWAI_STATE_RECOVERING";
            break;
        default:
            return "UNKNOWN";
    }
}

int can_receive(struct can_frame *frame, TickType_t ticks_to_wait)
{
    if (frame == NULL || rx_frame_queue == NULL) {
        return -1;
    }

    if (xQueueReceive(rx_frame_queue, frame, ticks_to_wait) != pdTRUE) {
        return -1;
    }

    return 0;
}

int can_send(const struct can_frame *frame)
{
    if (frame == NULL || can_bus_handle == NULL) {
        return -1;
    }

    uint32_t canid;

    if (frame->id & CAN_EFF_FLAG) {
        canid = frame->id & CAN_EFF_MASK;
    }
    else {
        canid = frame->id & CAN_SFF_MASK;
    }

    twai_message_t req_msg = {
        .identifier = canid,
        .data_length_code = frame->dlc,
        .self = false, // True: Transmitted message will also received by the same node 
        .extd = (frame->id & CAN_EFF_FLAG) ? true : false, // Extended Frame Format (29bit ID)
        .rtr = (frame->id & CAN_RTR_FLAG) ? true : false, // remote transmission request
    };

    /* copy data over with bounds checking */
    size_t copy_len = (frame->dlc > 8) ? 8 : frame->dlc;
    memcpy(req_msg.data, frame->data, copy_len);

    /* transmit can frame */
    // TODO: do something smarter with the timeout.
    esp_err_t err = twai_transmit_v2(can_bus_handle, &req_msg, pdMS_TO_TICKS(1000));
    if (err == ESP_OK) {
        return 0;
    } else {
        printf("Failed to transmit err=%d\n", err);
        return -1;
    }
}

void can_print_stat(void) {
    if (can_bus_handle == NULL) {
        ESP_LOGE(TAG, "CAN bus not initialized");
        return;
    }

    twai_status_info_t status_info = {0};
    esp_err_t err = twai_get_status_info_v2(can_bus_handle, &status_info);
    ESP_ERROR_CHECK(err);

    printf("CAN bus status info:\n");
    printf("state: %s\n", can_get_state_str(status_info.state));
    printf("msgs_to_tx: %ld\n", status_info.msgs_to_tx);
    printf("msgs_to_rx: %ld\n", status_info.msgs_to_rx);
    printf("tx_error_counter: %ld\n", status_info.tx_error_counter);
    printf("rx_error_counter: %ld\n", status_info.rx_error_counter);
    printf("tx_failed_count: %ld\n", status_info.tx_failed_count);
    printf("rx_missed_count: %ld\n", status_info.rx_missed_count);
    printf("rx_overrun_count: %ld\n", status_info.rx_overrun_count);
    printf("arb_lost_count: %ld\n", status_info.arb_lost_count);
    printf("bus_error_count: %ld\n", status_info.bus_error_count);
}


static void can_bus_task(void *param)
{
    esp_err_t err;
    twai_message_t msg = {0};
    uint32_t alerts = 0;
    struct can_frame frame = {0};

    while (!task_should_exit) {
        err = twai_read_alerts_v2(can_bus_handle, &alerts, 0);

        if (err == ESP_OK) {
            if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
                ESP_LOGE(TAG, "Surpassed Error Warning Limit");
            }
            if (alerts & TWAI_ALERT_ERR_PASS) {
                ESP_LOGE(TAG, "Entered Error Passive state");
            }
            if (alerts & TWAI_ALERT_BUS_OFF) {
                ESP_LOGE(TAG, "Bus Off state");
                //Prepare to initiate bus recovery, reconfigure alerts to detect bus recovery completion
                ESP_LOGI(TAG, "Initiate bus recovery");
                twai_initiate_recovery_v2(can_bus_handle);
            }
            if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                //Bus recovery was successful
                ESP_LOGI(TAG, "Bus Recovered");
                ESP_LOGI(TAG, "Restart CAN bus");
                twai_start_v2(can_bus_handle);
            }
        }

        //receive next CAN frame from queue
        err = twai_receive_v2(can_bus_handle, &msg, pdMS_TO_TICKS(100));
        if (err != ESP_OK) {
            continue;
        }

        frame.id = msg.identifier;
        frame.id |= msg.extd ? CAN_EFF_FLAG : 0;
        frame.id |= msg.rtr ? CAN_RTR_FLAG : 0;
        frame.dlc = msg.data_length_code;
        
        /* copy data with bounds checking */
        size_t copy_len = (msg.data_length_code > 8) ? 8 : msg.data_length_code;
        memset(frame.data, 0, 8); // Clear buffer first
        memcpy(frame.data, msg.data, copy_len);

        while (xQueueSend(rx_frame_queue, &frame, pdMS_TO_TICKS(100)) != pdTRUE) {
            if (task_should_exit) {
                break;
            }
        }
    }
    
    ESP_LOGI(TAG, "CAN bus task exiting gracefully");
    
    // Signal that the task is about to exit
    xSemaphoreGive(task_exit_semaphore);
    
    // Clear the task handle from within the task before self-deletion
    // This is safe because we're about to delete ourselves
    can_bus_task_handle = NULL;
    
    vTaskDelete(NULL); // Delete self
}

esp_err_t can_bus_setup(int baud_rate)
{
    // Prevent double initialization
    if (can_bus_task_handle != NULL) {
        ESP_LOGW(TAG, "CAN bus already initialized, call teardown first");
        return ESP_ERR_INVALID_STATE;
    }

    static const twai_timing_config_t t_config_1mbits = TWAI_TIMING_CONFIG_1MBITS();
    static const twai_timing_config_t t_config_500kbits = TWAI_TIMING_CONFIG_500KBITS();
    static const twai_timing_config_t t_config_250kbits = TWAI_TIMING_CONFIG_250KBITS();
    static const twai_timing_config_t t_config_125kbits = TWAI_TIMING_CONFIG_125KBITS();
    static const twai_timing_config_t t_config_50kbits = TWAI_TIMING_CONFIG_50KBITS();
    const twai_timing_config_t *t_config = NULL;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(
        PIN_CAN0_TX, PIN_CAN0_RX, TWAI_MODE_NORMAL);
    general_config.controller_id = 0;

    if (baud_rate == 1000000) {
        t_config = &t_config_1mbits;
    }
    else if (baud_rate == 500000) {
        t_config = &t_config_500kbits;
    }
    else if (baud_rate == 250000) {
        t_config = &t_config_250kbits;
    }
    else if (baud_rate == 125000) {
        t_config = &t_config_125kbits;
    }
    else if (baud_rate == 50000) {
        t_config = &t_config_50kbits;
    }
    else {
        ESP_LOGE(TAG, "Unsupported CAN baud rate %d\n", baud_rate);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Init CAN driver\n");

    /* Deep enough to hold what a saturated 500 kbit/s bus delivers between two
     * drains by the client above: the SLCAN front-end empties this every 2 ms,
     * and a bus of back-to-back minimum length frames produces about 21 in
     * that time. */
    rx_frame_queue = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(struct can_frame));
    if (rx_frame_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create RX frame queue");
        abort();
    }

    // Create semaphore for task synchronization
    task_exit_semaphore = xSemaphoreCreateBinary();
    if (task_exit_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create task exit semaphore");
        abort();
    }

    // Reset task exit flag
    task_should_exit = false;

    ESP_ERROR_CHECK(twai_driver_install_v2(&general_config, t_config, &f_config, &can_bus_handle));

    //Start TWAI driver
    ESP_ERROR_CHECK(twai_start_v2(can_bus_handle));

    /* Let the transceiver drive the bus. Active high, so low is not silent. */
    gpio_set_level(PIN_CAN0_SILENT, 0);

    //Prepare to trigger errors, reconfigure alerts to detect change in error state
    twai_reconfigure_alerts_v2(can_bus_handle,
        TWAI_ALERT_ABOVE_ERR_WARN | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED, NULL);

    BaseType_t task_result = xTaskCreate(can_bus_task, "canbus", 4096, NULL, 5, (TaskHandle_t *)&can_bus_task_handle);
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CAN bus task");
        abort();
    }

    ESP_LOGI(TAG, "Init CAN driver - Done!\n");

    return ESP_OK;
}

void can_bus_teardown(void)
{
    ESP_LOGI(TAG, "Teardown CAN driver\n");

    /* Back to listening only, so nothing this adapter does can reach the bus
     * while no channel is open. */
    gpio_set_level(PIN_CAN0_SILENT, 1);

    // Signal the task to exit gracefully
    if (can_bus_task_handle != NULL) {
        task_should_exit = true;
        
        // Wait for the task to signal completion (with timeout)
        if (task_exit_semaphore != NULL) {
            if (xSemaphoreTake(task_exit_semaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
                ESP_LOGI(TAG, "CAN bus task exited gracefully");
                // Task has signaled completion, handle should already be NULL
            } else {
                ESP_LOGW(TAG, "Timeout waiting for task to exit gracefully");
                // If task still exists after timeout, force delete it
                if (can_bus_task_handle != NULL) {
                    ESP_LOGW(TAG, "Force deleting CAN bus task");
                    vTaskDelete(can_bus_task_handle);
                    can_bus_task_handle = NULL;
                }
            }
        } else {
            // No semaphore available, force delete
            ESP_LOGW(TAG, "No semaphore available, force deleting task");
            vTaskDelete(can_bus_task_handle);
            can_bus_task_handle = NULL;
        }
    }

    if (can_bus_handle) {
        //Stop TWAI driver
        ESP_ERROR_CHECK(twai_stop_v2(can_bus_handle));

        //Uninstall TWAI driver
        ESP_ERROR_CHECK(twai_driver_uninstall_v2(can_bus_handle));

        can_bus_handle = NULL;
    }


    // Clean up synchronization objects
    if (task_exit_semaphore != NULL) {
        vSemaphoreDelete(task_exit_semaphore);
        task_exit_semaphore = NULL;
    }

    if (rx_frame_queue != NULL) {
        vQueueDelete(rx_frame_queue);
        rx_frame_queue = NULL;
    }

    ESP_LOGI(TAG, "Teardown CAN driver - Done!\n");
}