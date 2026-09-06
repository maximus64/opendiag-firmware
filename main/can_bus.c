/* SPDX-License-Identifier: GPL-3.0-only */
#include "can_bus.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "driver/gpio.h"
#include "pinout.h"

#define TAG "CANBUS"

#define CAN_RX_QUEUE_LEN 32

/** How many frames the driver will hold for us behind the one in hardware. */
#define CAN_TX_QUEUE_LEN 4

/**
 * @brief Frames we own on the driver's behalf.
 *
 * twai_node_transmit() stores the pointer it is given; the driver reads
 * through it when the hardware frees up, and again if a bus-off recovery has
 * to restart the transmission. A frame built on the caller's stack would be
 * gone by then. So the frames live here, and a send takes the next slot.
 *
 * Two more slots than the driver can be holding at once (CAN_TX_QUEUE_LEN
 * queued plus the one in hardware) is what makes reuse safe: by the time the
 * round trip comes back to a slot, every one of those has been handed over
 * since, so the frame in it cannot still be in flight.
 */
#define CAN_TX_POOL_LEN (CAN_TX_QUEUE_LEN + 2)

/** How long a send waits for room when the driver is still busy. */
#define CAN_TX_TIMEOUT_MS 1000

static twai_node_handle_t can_node;
static QueueHandle_t rx_frame_queue;
static TaskHandle_t can_bus_task_handle;
static bool task_should_exit;
static bool can_started;
static portMUX_TYPE task_notify_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t task_exit_semaphore = NULL;

static twai_frame_t tx_pool[CAN_TX_POOL_LEN];
static uint8_t tx_pool_data[CAN_TX_POOL_LEN][8];
static unsigned tx_pool_next;

/** Frames received with nowhere to put them. Nothing else counts these. */
static volatile uint32_t rx_dropped;
static uint32_t tx_failed;
static uint32_t bus_errors;
static uint32_t arbitration_lost;
static uint32_t bit_errors;
static uint32_t form_errors;
static uint32_t stuff_errors;
static uint32_t ack_errors;
static uint32_t unclassified_errors;

static bool can_error(twai_node_handle_t node,
                      const twai_error_event_data_t *event, void *ctx) {
    twai_error_flags_t errors = event->err_flags;
    if (errors.arb_lost)
        __atomic_fetch_add(&arbitration_lost, 1, __ATOMIC_RELAXED);
    errors.arb_lost = 0;
    /* IDF combines bus errors and normal arbitration losses in bus_err_num.
     * Empty flags can denote an unclassified bus error (e.g. CRC on S3). */
    if (errors.val || !event->err_flags.arb_lost)
        __atomic_fetch_add(&bus_errors, 1, __ATOMIC_RELAXED);
    if (errors.bit_err)
        __atomic_fetch_add(&bit_errors, 1, __ATOMIC_RELAXED);
    if (errors.form_err)
        __atomic_fetch_add(&form_errors, 1, __ATOMIC_RELAXED);
    if (errors.stuff_err)
        __atomic_fetch_add(&stuff_errors, 1, __ATOMIC_RELAXED);
    if (errors.ack_err)
        __atomic_fetch_add(&ack_errors, 1, __ATOMIC_RELAXED);
    if (!errors.bit_err && !errors.form_err && !errors.stuff_err &&
        !errors.ack_err && (errors.val || !event->err_flags.arb_lost))
        __atomic_fetch_add(&unclassified_errors, 1, __ATOMIC_RELAXED);
    return false;
}

static bool can_tx_done(twai_node_handle_t node,
                        const twai_tx_done_event_data_t *event, void *ctx) {
    if (!event->is_tx_success)
        __atomic_fetch_add(&tx_failed, 1, __ATOMIC_RELAXED);
    return false;
}

static const char *can_get_state_str(twai_error_state_t state) {
    switch (state) {
    case TWAI_ERROR_ACTIVE:
        return "error active";
    case TWAI_ERROR_WARNING:
        return "error warning";
    case TWAI_ERROR_PASSIVE:
        return "error passive";
    case TWAI_ERROR_BUS_OFF:
        /* The driver uses this for a node that was never started as well
         * as one the bus threw off, so it cannot be reported as either. */
        return "bus off or stopped";
    default:
        return "UNKNOWN";
    }
}

int can_receive(struct can_frame *frame, TickType_t ticks_to_wait) {
    if (!can_started || frame == NULL || rx_frame_queue == NULL) {
        return -1;
    }

    if (xQueueReceive(rx_frame_queue, frame, ticks_to_wait) != pdTRUE) {
        return -1;
    }

    return 0;
}

int can_send(const struct can_frame *frame) {
    if (!can_started || frame == NULL || can_node == NULL) {
        return -1;
    }

    uint32_t canid;

    if (frame->id & CAN_EFF_FLAG) {
        canid = frame->id & CAN_EFF_MASK;
    } else {
        canid = frame->id & CAN_SFF_MASK;
    }

    /* Only one link may hold the CAN bus at a time, so there is one sender and
     * this index needs no lock. */
    twai_frame_t *msg = &tx_pool[tx_pool_next];
    uint8_t *data = tx_pool_data[tx_pool_next];
    size_t copy_len = (frame->dlc > 8) ? 8 : frame->dlc;

    memset(msg, 0, sizeof(*msg));
    msg->header.id = canid;
    msg->header.ide =
        (frame->id & CAN_EFF_FLAG) ? 1 : 0; // Extended Frame Format (29bit ID)
    msg->header.rtr =
        (frame->id & CAN_RTR_FLAG) ? 1 : 0; // remote transmission request

    if (msg->header.rtr) {
        /* A remote frame carries no data, but its length code still asks for
         * that many bytes back, so it has to be set rather than derived. */
        msg->header.dlc = copy_len;
        msg->buffer = data;
        msg->buffer_len = 0;
    } else {
        memcpy(data, frame->data, copy_len);
        msg->buffer = data;
        msg->buffer_len = copy_len; /* the driver derives the length code */
    }

    // TODO: do something smarter with the timeout.
    esp_err_t err = twai_node_transmit(can_node, msg, CAN_TX_TIMEOUT_MS);
    if (err != ESP_OK) {
        printf("Failed to transmit err=%d\n", err);
        return -1;
    }

    tx_pool_next = (tx_pool_next + 1) % CAN_TX_POOL_LEN;
    return 0;
}

void can_print_stat(void) {
    if (!can_started || can_node == NULL) {
        ESP_LOGE(TAG, "CAN bus not initialized");
        return;
    }

    twai_node_status_t status = {0};
    twai_node_record_t record = {0};
    esp_err_t err = twai_node_get_info(can_node, &status, &record);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_get_info failed: %s", esp_err_to_name(err));
        return;
    }

    printf("CAN bus status info:\n");
    printf("state: %s\n", can_get_state_str(status.state));
    printf("tx_error_counter: %u\n", status.tx_error_count);
    printf("rx_error_counter: %u\n", status.rx_error_count);
    printf("tx_queue_free: %lu\n", status.tx_queue_remaining);
    printf("driver_error_events: %lu\n", record.bus_err_num);
    printf("bus_error_count: %lu\n",
           (unsigned long)__atomic_load_n(&bus_errors, __ATOMIC_RELAXED));
    printf("arb_lost_count: %lu\n",
           (unsigned long)__atomic_load_n(&arbitration_lost, __ATOMIC_RELAXED));
    printf("bit_error_count: %lu\n",
           (unsigned long)__atomic_load_n(&bit_errors, __ATOMIC_RELAXED));
    printf("form_error_count: %lu\n",
           (unsigned long)__atomic_load_n(&form_errors, __ATOMIC_RELAXED));
    printf("stuff_error_count: %lu\n",
           (unsigned long)__atomic_load_n(&stuff_errors, __ATOMIC_RELAXED));
    printf("ack_error_count: %lu\n",
           (unsigned long)__atomic_load_n(&ack_errors, __ATOMIC_RELAXED));
    printf(
        "unclassified_error_count: %lu\n",
        (unsigned long)__atomic_load_n(&unclassified_errors, __ATOMIC_RELAXED));
    printf("tx_failed_count: %lu\n",
           (unsigned long)__atomic_load_n(&tx_failed, __ATOMIC_RELAXED));
    printf("rx_dropped: %lu\n", (unsigned long)rx_dropped);
}

/**
 * @brief A frame arrived. Runs in the driver's interrupt.
 *
 * The only place a received frame can be read from - the driver hands it over
 * here or not at all, so this cannot defer the read to the worker task.
 */
static bool can_rx_done(twai_node_handle_t node,
                        const twai_rx_done_event_data_t *edata, void *ctx) {
    BaseType_t task_woken = pdFALSE;
    uint8_t buffer[8];
    twai_frame_t rx = {
        .buffer = buffer,
        .buffer_len = sizeof(buffer),
    };
    struct can_frame frame = {0};

    if (twai_node_receive_from_isr(node, &rx) != ESP_OK) {
        return false;
    }

    frame.id = rx.header.id;
    frame.id |= rx.header.ide ? CAN_EFF_FLAG : 0;
    frame.id |= rx.header.rtr ? CAN_RTR_FLAG : 0;
    frame.dlc = rx.header.dlc;

    /* copy data with bounds checking */
    size_t copy_len = (rx.header.dlc > 8) ? 8 : rx.header.dlc;
    memcpy(frame.data, buffer, copy_len);

    /* No waiting here, and nothing that could wait: a client that has stopped
     * draining the queue must cost frames, not the interrupt. */
    if (xQueueSendFromISR(rx_frame_queue, &frame, &task_woken) != pdTRUE) {
        rx_dropped++;
    }

    return task_woken == pdTRUE;
}

/**
 * @brief The controller changed error state. Runs in the driver's interrupt.
 *
 * Both edges of a bus-off are worth waking the worker for: the way in, because
 * recovery has to be asked for and asking is not an interrupt-safe call, and
 * the way out, because that is the only notice anybody gets that the bus came
 * back. Recovery finishing needs nothing from us - the controller returns to
 * error-active on its own after 128 runs of 11 recessive bits, and the driver
 * restarts whatever transmissions were queued when it does.
 *
 * The worker reads the state for itself rather than being told which edge this
 * was, so two notifications collapsing into one wakeup cannot lose a recovery.
 */
static bool can_state_change(twai_node_handle_t node,
                             const twai_state_change_event_data_t *edata,
                             void *ctx) {
    BaseType_t task_woken = pdFALSE;
    bool crossed = (edata->new_sta == TWAI_ERROR_BUS_OFF) ||
                   (edata->old_sta == TWAI_ERROR_BUS_OFF);

    portENTER_CRITICAL_ISR(&task_notify_lock);
    if (crossed && !__atomic_load_n(&task_should_exit, __ATOMIC_ACQUIRE) &&
        can_bus_task_handle != NULL) {
        vTaskNotifyGiveFromISR(can_bus_task_handle, &task_woken);
    }
    portEXIT_CRITICAL_ISR(&task_notify_lock);

    return task_woken == pdTRUE;
}

/**
 * @brief Recovers the controller when the bus has thrown it off.
 *
 * All that is left of what used to be a polling loop: reception and the error
 * states both arrive as interrupts now. The wait has a timeout only so that a
 * teardown does not have to wake it.
 */
static void can_bus_task(void *param) {
    while (!__atomic_load_n(&task_should_exit, __ATOMIC_ACQUIRE)) {
        twai_node_status_t status = {0};
        esp_err_t err;

        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) == 0) {
            continue;
        }

        if (__atomic_load_n(&task_should_exit, __ATOMIC_ACQUIRE)) {
            break;
        }

        if (twai_node_get_info(can_node, &status, NULL) != ESP_OK) {
            continue;
        }

        if (status.state != TWAI_ERROR_BUS_OFF) {
            ESP_LOGI(TAG, "Bus Recovered");
            continue;
        }

        ESP_LOGE(TAG, "Bus Off state");
        ESP_LOGI(TAG, "Initiate bus recovery");

        err = twai_node_recover(can_node);
        if (err != ESP_OK) {
            /* INVALID_STATE here means the node was disabled rather than
             * thrown off, which a teardown does on its way past. */
            ESP_LOGW(TAG, "twai_node_recover failed: %s", esp_err_to_name(err));
        }
    }

    ESP_LOGI(TAG, "CAN bus task exiting gracefully");

    xSemaphoreGive(task_exit_semaphore);
    vTaskDelete(NULL);
}

/**
 * @brief Release everything can_bus_setup() claimed, in reverse.
 *
 * Shared with the setup failure path, so it has to tolerate a half built
 * driver: every step is guarded by whether that step ever happened.
 *
 * The caller must have stopped can_bus_task() first - it is the only other
 * thing that touches the controller.
 */
static esp_err_t can_bus_release(void) {
    if (can_node != NULL) {
        esp_err_t err;
        twai_node_status_t status = {0};

        err = twai_node_get_info(can_node, &status, NULL);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "twai_node_get_info failed: %s",
                     esp_err_to_name(err));
            return err;
        }

        if (status.state != TWAI_ERROR_BUS_OFF) {
            err = twai_node_disable(can_node);
            /* Bus-off can occur after the status read. */
            if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "twai_node_disable failed: %s",
                         esp_err_to_name(err));
                return err;
            }
        }

        err = twai_node_delete(can_node);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "twai_node_delete failed: %s", esp_err_to_name(err));
            return err;
        }
        can_node = NULL;
    }

    if (task_exit_semaphore != NULL) {
        vSemaphoreDelete(task_exit_semaphore);
        task_exit_semaphore = NULL;
    }

    if (rx_frame_queue != NULL) {
        vQueueDelete(rx_frame_queue);
        rx_frame_queue = NULL;
    }
    return ESP_OK;
}

/**
 * @brief Ask can_bus_task() to leave, and wait for it to say that it has.
 */
static esp_err_t can_bus_stop_task(void) {
    /* Drain notifications before the worker can delete its task handle. */
    portENTER_CRITICAL(&task_notify_lock);
    __atomic_store_n(&task_should_exit, true, __ATOMIC_RELEASE);
    portEXIT_CRITICAL(&task_notify_lock);

    if (can_bus_task_handle == NULL)
        return ESP_OK;
    if (xSemaphoreTake(task_exit_semaphore, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "worker did not exit; resources retained");
        return ESP_ERR_TIMEOUT;
    }
    can_bus_task_handle = NULL;
    return ESP_OK;
}

/**
 * @brief Where the bit is sampled, in permille of the bit time.
 *
 * The driver picks its own if this is left at zero, and would pick differently
 * from the legacy timing tables this firmware was tested against: 80% at
 * 500 kbit/s and 87.5% below it, where the old macros used 75% everywhere
 * except 50 kbit/s. Both are defensible and the difference only shows on a
 * long or noisy bus, but a port is the wrong place to change how the adapter
 * sits on somebody's vehicle. These are the old numbers.
 */
static uint16_t can_sample_point_permille(int baud_rate) {
    return (baud_rate == 50000) ? 800 : 750;
}

esp_err_t can_bus_setup(int baud_rate) {
    esp_err_t err;

    if (can_bus_task_handle || can_node || rx_frame_queue ||
        task_exit_semaphore) {
        ESP_LOGW(TAG, "CAN bus already initialized, call teardown first");
        return ESP_ERR_INVALID_STATE;
    }

    switch (baud_rate) {
    case 1000000:
    case 500000:
    case 250000:
    case 125000:
    case 50000:
        break;
    default:
        ESP_LOGE(TAG, "Unsupported CAN baud rate %d\n", baud_rate);
        return ESP_ERR_INVALID_ARG;
    }

    twai_onchip_node_config_t node_config = {
        .io_cfg =
            {
                .tx = PIN_CAN0_TX,
                .rx = PIN_CAN0_RX,
                .quanta_clk_out = GPIO_NUM_NC,
                .bus_off_indicator = GPIO_NUM_NC,
            },
        .bit_timing =
            {
                .bitrate = (uint32_t)baud_rate,
                .sp_permill = can_sample_point_permille(baud_rate),
            },
        .tx_queue_depth = CAN_TX_QUEUE_LEN,
        /* Retry forever, which is what the old driver did and what the
         * teardown path is written around: transmitting into a bus with no ECU
         * answering is what drives the controller to bus-off, and the ELM327
         * protocol search depends on getting there rather than on being told
         * about a single failed frame. */
        .fail_retry_cnt = -1,
    };

    ESP_LOGI(TAG, "Init CAN driver\n");

    /* Deep enough to hold what a saturated 500 kbit/s bus delivers between two
     * drains by the client above: the SLCAN front-end empties this every 2 ms,
     * and a bus of back-to-back minimum length frames produces about 21 in
     * that time. */
    rx_frame_queue = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(struct can_frame));
    task_exit_semaphore = xSemaphoreCreateBinary();
    if (rx_frame_queue == NULL || task_exit_semaphore == NULL) {
        ESP_LOGE(
            TAG,
            "Failed to create the RX frame queue or the task exit semaphore");
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    // Reset task exit flag
    __atomic_store_n(&task_should_exit, false, __ATOMIC_RELEASE);
    tx_pool_next = 0;
    rx_dropped = 0;
    __atomic_store_n(&tx_failed, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&bus_errors, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&arbitration_lost, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&bit_errors, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&form_errors, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&stuff_errors, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&ack_errors, 0, __ATOMIC_RELAXED);
    __atomic_store_n(&unclassified_errors, 0, __ATOMIC_RELAXED);

    /* A bus that will not come up is a protocol the client cannot have, not a
     * reason to restart the adapter: vif_bus_open() turns any error here into
     * an ELM327 protocol failure, and the search moves on to the next one. */
    err = twai_new_node_onchip(&node_config, &can_node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_new_node_onchip failed: %s", esp_err_to_name(err));
        can_node = NULL;
        goto fail;
    }

    /* No filter is configured: the hardware accepts every ID until one is,
     * which is what an OBD adapter wants. */

    const twai_event_callbacks_t callbacks = {
        .on_rx_done = can_rx_done,
        .on_tx_done = can_tx_done,
        .on_error = can_error,
        .on_state_change = can_state_change,
    };
    err = twai_node_register_event_callbacks(can_node, &callbacks, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_register_event_callbacks failed: %s",
                 esp_err_to_name(err));
        goto fail;
    }

    /* The worker has to exist before the node is enabled: a bus-off can arrive
     * with the first frame, and the interrupt has nothing to wake without it.
     */
    BaseType_t task_result = xTaskCreate(can_bus_task, "canbus", 4096, NULL, 5,
                                         &can_bus_task_handle);
    if (task_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CAN bus task");
        can_bus_task_handle = NULL;
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    err = twai_node_enable(can_node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_enable failed: %s", esp_err_to_name(err));
        goto fail;
    }

    /* Let the transceiver drive the bus. Active high, so low is not silent. */
    gpio_set_level(PIN_CAN0_SILENT, 0);
    can_started = true;

    ESP_LOGI(TAG, "Init CAN driver - Done!\n");

    return ESP_OK;

fail: {
    esp_err_t cleanup = can_bus_teardown();
    if (cleanup != ESP_OK) {
        ESP_LOGE(TAG, "startup cleanup incomplete: %s",
                 esp_err_to_name(cleanup));
    }
}
    return err;
}

esp_err_t can_bus_teardown(void) {
    can_started = false;
    gpio_set_level(PIN_CAN0_SILENT, 1);

    esp_err_t err = can_bus_stop_task();
    if (err != ESP_OK)
        return err;
    err = can_bus_release();
    if (err == ESP_OK)
        ESP_LOGI(TAG, "Teardown CAN driver - Done!");
    return err;
}

/* ------------------------------------------------------------------ *
 * The bus interface
 *
 * CAN behind the same vtable as the byte buses. A frame is an id and a
 * payload, which is exactly what bus_msg_t carries, so there is nothing to
 * translate: the length code goes in len, the id and its flags in id.
 * ------------------------------------------------------------------ */

static esp_err_t can_ops_open(const bus_cfg_t *cfg) {
    /* The controller takes its rate at install time and cannot be retimed
     * while up, so a claim without one is refused here rather than later. */
    if (!cfg || cfg->bitrate == 0) {
        ESP_LOGE(TAG, "CAN needs a bit rate");
        return ESP_ERR_INVALID_ARG;
    }

    return can_bus_setup((int)cfg->bitrate);
}

static esp_err_t can_ops_close(void) { return can_bus_teardown(); }

static int can_ops_send(const bus_msg_t *msg, uint32_t flags) {
    struct can_frame f = {0};

    f.id = msg->id;
    f.dlc = (uint8_t)msg->len;

    /* A remote frame asks for len bytes and carries none. The copy is
     * clamped because the length code may legally exceed the eight bytes a
     * classic frame can hold. */
    if (!(f.id & CAN_RTR_FLAG)) {
        memcpy(f.data, msg->data, msg->len > 8 ? 8 : msg->len);
    }

    uint32_t failures = __atomic_load_n(&tx_failed, __ATOMIC_RELAXED);
    if (can_send(&f) != 0)
        return BUS_ERR_TX_FAILED;
    if (flags & BUS_TX_WAIT_DONE) {
        esp_err_t err =
            twai_node_transmit_wait_all_done(can_node, CAN_TX_TIMEOUT_MS);
        /* Completion may race the end of the blocking wait. */
        if (err != ESP_OK)
            err = twai_node_transmit_wait_all_done(can_node, 0);
        if (err != ESP_OK) {
            /* Disable alone retains queued frames. Silence now, then delete
             * the node; failed cleanup keeps sends blocked until close. */
            esp_err_t cleanup = can_bus_teardown();
            if (cleanup != ESP_OK)
                ESP_LOGE(TAG, "TX abort cleanup failed: %s",
                         esp_err_to_name(cleanup));
            return BUS_ERR_TX_ABORTED;
        }
        if (__atomic_load_n(&tx_failed, __ATOMIC_RELAXED) != failures)
            return BUS_ERR_TX_FAILED;
    }
    return 0;
}

static int can_ops_recv(bus_msg_t *msg, TickType_t wait) {
    struct can_frame f;
    size_t len;

    if (can_receive(&f, wait) != 0) {
        return BUS_ERR_TIMEOUT;
    }

    len = (f.dlc > 8) ? 8 : f.dlc;
    if (f.id & CAN_RTR_FLAG) {
        len = 0;
    }
    if (msg->cap < len) {
        return BUS_ERR_NO_SPACE;
    }

    memcpy(msg->data, f.data, len);
    msg->id = f.id;
    msg->len = f.dlc;

    return (int)len;
}

/* set_param, get_param and ioctl stay NULL: the rate is fixed at open and
 * the rest of bus_param_t is byte bus timing. The generic calls answer
 * BUS_ERR_UNSUPPORTED for them, which is the honest answer. Statistics come
 * from the controller instead, through can_print_stat(). */
const bus_ops_t can_bus_ops = {
    .name = "CAN",
    .open = can_ops_open,
    .close = can_ops_close,
    .send = can_ops_send,
    .recv = can_ops_recv,
};
