/* SPDX-License-Identifier: GPL-3.0-only */
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/rmt_rx.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "sdkconfig.h"
#include "pinout.h"
#include "j1850_vpw.h"

// shell test command: vpw 686af10100

// We use RMT RX instead of a software ISR to capture VPW pulse widths accurately

// RMT RX configuration
#define RMT_RESOLUTION_HZ   1000000 // 1MHz -> 1 tick = 1us
#define BUFFER_SIZE_SYMBOLS 512
// RMT input pulse acceptance window (nanoseconds)
#define RX_RANGE_MIN_NS      3000
#define RX_RANGE_MAX_NS      300000

// --- J1850 VPW Timings (in microseconds) ---
#define VPW_T_SHORT         64   // Short pulse (active low) for logic '1'
#define VPW_T_LONG          128  // Long pulse (active low) for logic '0'
#define VPW_T_SOF           200  // Start of Frame pulse (active low)
#define VPW_T_EOD           200  // End of Data - minimum passive (high) period
#define VPW_T_IFS           300  // Inter-Frame Space (minimum idle time before new frame)
#define VPW_T_PROPAGATION   10   // Estimated signal propagation delay

// --- Tolerances for Receiving ---
#define T_PULSE_TOLERANCE   20   // 20us tolerance for active pulses
#define T_SHORT_MIN         (VPW_T_SHORT - T_PULSE_TOLERANCE)
#define T_SHORT_MAX         (VPW_T_SHORT + T_PULSE_TOLERANCE)
#define T_LONG_MIN          (VPW_T_LONG - T_PULSE_TOLERANCE)
#define T_LONG_MAX          (VPW_T_LONG + T_PULSE_TOLERANCE)
#define T_SOF_MIN           (VPW_T_SOF - T_PULSE_TOLERANCE)
#define T_SOF_MAX           (VPW_T_SOF + T_PULSE_TOLERANCE)
#define T_EOD_MIN           (VPW_T_EOD - T_PULSE_TOLERANCE)

#define TAG "J1850_VPW"

struct j1850_frame {
    uint8_t len;
    uint8_t data[16]; // Max frame size 12 bytes + CRC
};

// --- Static variables ---
static QueueHandle_t s_rx_frame_queue;
static SemaphoreHandle_t s_tx_mutex;      // Ensures only one task can transmit at a time
static portMUX_TYPE s_tx_spinlock = portMUX_INITIALIZER_UNLOCKED;
static rmt_channel_handle_t s_rmt_rx = NULL;
static rmt_symbol_word_t *s_rmt_buffer = NULL;
static bool s_rx_enabled = false;


// Standard CRC-8/SAE-J1850
static uint8_t j1850_cal_checksum(const uint8_t *data, size_t length) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x1D;
            } else {
                crc <<= 1;
            }
        }
    }
    return ~crc;
}

// Helper to (re)arm RMT RX with consistent range and buffer
static inline esp_err_t arm_rmt_rx(rmt_channel_handle_t ch) {
    if (!ch || !s_rmt_buffer) {
        return ESP_ERR_INVALID_STATE;
    }
    const rmt_receive_config_t rx_range = {
        .signal_range_min_ns = RX_RANGE_MIN_NS,
        .signal_range_max_ns = RX_RANGE_MAX_NS,
    };
    return rmt_receive(ch, s_rmt_buffer, BUFFER_SIZE_SYMBOLS * sizeof(rmt_symbol_word_t), &rx_range);
}


/**
 * @brief Wait until a GPIO stays at the target level for a continuous period.
 * This is useful to ensure the bus is idle (or busy) for at least a given window.
 * @param gpio_num The GPIO to monitor.
 * @param target_level The level that must be maintained.
 * @param min_duration_us Minimum continuous duration to accept (in microseconds).
 * @param timeout_us Overall timeout for the wait (in microseconds).
 * @return true if level was held continuously for min_duration_us before timeout, else false.
 */
static bool IRAM_ATTR wait_level_stable(int gpio_num, int target_level, uint32_t min_duration_us, uint32_t timeout_us) {
    const int64_t deadline = esp_timer_get_time() + timeout_us;
    while (esp_timer_get_time() < deadline) {
        if (gpio_get_level(gpio_num) == target_level) {
            const int64_t t0 = esp_timer_get_time();
            while ((esp_timer_get_time() - t0) < min_duration_us) {
                if (gpio_get_level(gpio_num) != target_level) {
                    // Level changed before the window elapsed -> restart outer wait
                    goto continue_wait;
                }
            }
            // Held stable for the required duration
            return true;
        }
continue_wait:
        ;
    }
    return false;
}


/**
 * @brief Internal function that sends a frame with precise timing.
 * Interrupts are disabled during transmission.
 * @return 0 on success, -1 on arbitration loss.
 */
static int j1850_send_frame_blocking(const uint8_t *data, uint8_t len) {
    // Wait until bus is idle (logic low on RX) for IFS period
    if (!wait_level_stable(PIN_J1850_VPW_RX, 0, VPW_T_IFS, VPW_T_IFS + 100)) {
        ESP_LOGW(TAG, "Bus is not idle for IFS, cannot send");
        return -2; // Bus busy
    }

    int64_t tstart = esp_timer_get_time();

    portENTER_CRITICAL(&s_tx_spinlock);

    // --- Send SOF ---
    gpio_set_level(PIN_J1850_TX_P, 1); // Active
    esp_rom_delay_us(VPW_T_SOF);

    // --- Send Data Bytes ---
    for (int i = 0; i < len; i++) {
        uint8_t byte = data[i];

        uint8_t nbits = 8;
        while( nbits-- ) {
            bool bit = (byte >> nbits) & 1; // MSB first
            uint32_t width;

            tstart = esp_timer_get_time();

            if (nbits & 1) // start allways with passive symbol
            {
                width = bit ? VPW_T_LONG : VPW_T_SHORT;
                gpio_set_level(PIN_J1850_TX_P, 0); // Passive
                esp_rom_delay_us(VPW_T_PROPAGATION);
                while ((esp_timer_get_time() - tstart) < width) {
                    if (gpio_get_level(PIN_J1850_VPW_RX) == 1) {
                        portEXIT_CRITICAL(&s_tx_spinlock);
                        ESP_LOGW(TAG, "Arbitration lost!");
                        return -1;
                    }
                }
            }
            else {
                width = bit ? VPW_T_SHORT : VPW_T_LONG;
                gpio_set_level(PIN_J1850_TX_P, 1); // Active
                esp_rom_delay_us(VPW_T_PROPAGATION);
                while ((esp_timer_get_time() - tstart) < width) {
                    if (gpio_get_level(PIN_J1850_VPW_RX) ==  0) {
                        portEXIT_CRITICAL(&s_tx_spinlock);
                        ESP_LOGW(TAG, "Arbitration lost!");
                        return -1;
                    }
                }
            }
        }
    }

    //EOF
    gpio_set_level(PIN_J1850_TX_P, 0); // Passive
    tstart = esp_timer_get_time();
    esp_rom_delay_us(VPW_T_PROPAGATION);
    while ((esp_timer_get_time() - tstart) < VPW_T_EOD) {
        if (gpio_get_level(PIN_J1850_VPW_RX) ==  1) {
            portEXIT_CRITICAL(&s_tx_spinlock);
            ESP_LOGW(TAG, "Arbitration lost!");
            return -1;
        }
    }

    portEXIT_CRITICAL(&s_tx_spinlock);
    return 0; // Success
}


// --- VPW RX decoding ---
static inline bool is_in_range(uint32_t v, uint32_t vmin, uint32_t vmax) {
    return (v >= vmin) && (v <= vmax);
}

static int j1850_vpw_decode_symbols(const rmt_symbol_word_t *symbols, size_t num_symbols,
                                    uint8_t *out_bytes, size_t out_size) {
    if (!symbols || num_symbols == 0) return 0;

    bool in_frame = false;
    uint8_t cur_byte = 0;
    int bit_idx = 0; // 0..7 (MSB first)
    int byte_idx = 0;

    for (size_t i = 0; i < num_symbols * 2; i++) {
        const rmt_symbol_word_t s = symbols[i >> 1];
        uint32_t level, duration;

        if (i & 1) {
            level = s.level1;
            duration = s.duration1;
        }
        else {
            level = s.level0;
            duration = s.duration0;
        }

        //esp_rom_printf("%d/%d Level %u, Duration %u\n", i, num_symbols,level, duration);

        if (!in_frame) {
            // Look for SOF as a ~200us high pulse
            if (level && is_in_range(duration, T_SOF_MIN, T_SOF_MAX)) {
                in_frame = true;
                cur_byte = 0;
                bit_idx = 0;
                byte_idx = 0;
            }
            continue;
        }

        // In frame: each low pulse encodes a bit
        if (is_in_range(duration, T_SHORT_MIN, T_SHORT_MAX)) {
            if (level) {
                // Bit '1'
                cur_byte |= (1u << (7 - bit_idx));
            }
            // Bit '0'
            bit_idx++;
        } else if (is_in_range(duration, T_LONG_MIN, T_LONG_MAX)) {
            if (!level) {
                // Bit '1'
                cur_byte |= (1u << (7 - bit_idx));
            }
            // Bit '0'
            bit_idx++;
        } else if (level && is_in_range(duration, T_SOF_MIN, T_SOF_MAX)) {
            // Unexpected new SOF inside frame: finalize what we have
            //esp_rom_printf("Unexpected SOF inside frame\n");
            break;
        } else {
            // Invalid pulse -> abort this frame
            //esp_rom_printf("Invalid pulse, aborting frame\n");
            break;
        }

        if (bit_idx == 8) {
            if (byte_idx < (int)out_size) {
                out_bytes[byte_idx++] = cur_byte;
            }
            cur_byte = 0;
            bit_idx = 0;
        }
    }

    //esp_rom_printf("Decoded %d bytes\n", byte_idx);

    // Only accept if we have at least one full byte and a valid CRC
    if (byte_idx >= 2) {
        uint8_t calc = j1850_cal_checksum(out_bytes, byte_idx - 1);
        if (calc == out_bytes[byte_idx - 1]) {
            return byte_idx; // includes CRC
        }
        //esp_rom_printf("CRC mismatch: calculated 0x%02X, received 0x%02X\n", calc, out_bytes[byte_idx - 1]);
    }
    return 0; // decode failure
}

static bool IRAM_ATTR on_vpw_receive_callback(rmt_channel_handle_t channel,
                                              const rmt_rx_done_event_data_t *event_data,
                                              void *user_data) {
    BaseType_t high_task_woken = pdFALSE;
    struct j1850_frame rx;

    (void) user_data;

    rx.len = 0;

    if (event_data && event_data->num_symbols > 4) {
        int n = j1850_vpw_decode_symbols(event_data->received_symbols,
                                         event_data->num_symbols,
                                         rx.data, sizeof(rx.data));
        if (n > 0) {
            rx.len = (uint8_t)n; // includes CRC
            xQueueSendFromISR(s_rx_frame_queue, &rx, &high_task_woken);
        }
    }

    // Re-arm RX
    (void)arm_rmt_rx(channel);

    return high_task_woken == pdTRUE;
}

/**
 * @brief Initialize the J1850 VPW driver.
 */
void j1850_vpw_setup(void) {
    if (!s_rx_frame_queue) {
        s_rx_frame_queue = xQueueCreate(8, sizeof(struct j1850_frame));
        if (!s_rx_frame_queue) {
            ESP_LOGE(TAG, "Failed to create RX frame queue");
        }
    }
    if (!s_tx_mutex) {
        s_tx_mutex = xSemaphoreCreateMutex();
        if (!s_tx_mutex) {
            ESP_LOGE(TAG, "Failed to create TX mutex");
        }
    }

    // Switch regulator to VPW mode
    gpio_set_level(PIN_J1850_MODE, 0);

    // --- Configure TX Pin ---
    gpio_config_t tx_conf = {
        .pin_bit_mask = (1ULL << PIN_J1850_TX_P),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&tx_conf);
    gpio_set_level(PIN_J1850_TX_P, 0); // Set bus to passive

    // --- Set up RMT RX on VPW RX pin ---
    if (!s_rmt_rx) {
        rmt_rx_channel_config_t rx_cfg = {
            .gpio_num = PIN_J1850_VPW_RX,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = RMT_RESOLUTION_HZ,
            .mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL,
            .flags.with_dma = 1,
        };
        ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_cfg, &s_rmt_rx));

        const rmt_rx_event_callbacks_t cbs = {
            .on_recv_done = on_vpw_receive_callback,
        };
        ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(s_rmt_rx, &cbs, NULL));

        s_rmt_buffer = heap_caps_aligned_calloc(64, BUFFER_SIZE_SYMBOLS, sizeof(rmt_symbol_word_t),
                                                MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
        if (!s_rmt_buffer) {
            ESP_LOGE(TAG, "Failed to allocate RMT RX buffer");
            return;
        }

        ESP_ERROR_CHECK(rmt_enable(s_rmt_rx));
        ESP_ERROR_CHECK(arm_rmt_rx(s_rmt_rx));
        s_rx_enabled = true;
    }

    ESP_LOGI(TAG, "J1850 VPW driver initialized (TX bit-bang, RX via RMT).");
}

void j1850_vpw_teardown(void) {
    if (s_rx_frame_queue) {
        vQueueDelete(s_rx_frame_queue);
        s_rx_frame_queue = NULL;
    }
    if (s_tx_mutex) {
        vSemaphoreDelete(s_tx_mutex);
        s_tx_mutex = NULL;
    }

    if (s_rmt_rx) {
        rmt_disable(s_rmt_rx);
        rmt_del_channel(s_rmt_rx);
        s_rmt_rx = NULL;
    }
    if (s_rmt_buffer) {
        free(s_rmt_buffer);
        s_rmt_buffer = NULL;
    }
    s_rx_enabled = false;
}


/**
 * @brief Send a J1850 VPW frame.
 * @param data Pointer to the data buffer to send.
 * @param len Length of the data.
 * @return 0 on success, negative on error.
 */
int j1850_vpw_send(const uint8_t *data, uint8_t len) {
    struct j1850_frame frame;

    if (len > sizeof(frame.data) - 1) {
        return -10; // Invalid length
    }

    memcpy(frame.data, data, len);
    frame.data[len] = j1850_cal_checksum(data, len);
    frame.len = len + 1; // Data + CRC

    int ret = -1;
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        // Pause RX while transmitting to prevent receiving our own echo
        if (s_rmt_rx && s_rx_enabled) {
            rmt_disable(s_rmt_rx);
            s_rx_enabled = false;
        }
        
        ret = j1850_send_frame_blocking(frame.data, frame.len);

        // Re-enable RX
        if (s_rmt_rx && !s_rx_enabled) {
            ESP_ERROR_CHECK(rmt_enable(s_rmt_rx));
            ESP_ERROR_CHECK(arm_rmt_rx(s_rmt_rx));
            s_rx_enabled = true;
        }
        xSemaphoreGive(s_tx_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to acquire TX mutex");
        ret = -11; // Mutex timeout
    }
    return ret;
}

/**
 * @brief Receive a J1850 VPW frame.
 * @param data Buffer to store the received data (without CRC).
 * @param max_len Maximum size of the buffer.
 * @param xTicksToWait Ticks to wait for a frame.
 * @return Length of received data, or negative on error/timeout.
 */
int j1850_vpw_receive(uint8_t *data, uint8_t max_len, TickType_t xTicksToWait) {
    if (!s_rx_frame_queue) {
        return -2;
    }

    struct j1850_frame frame;
    if (xQueueReceive(s_rx_frame_queue, &frame, xTicksToWait) == pdTRUE) {
        if (frame.len == 0) {
            return -3; // decode error
        }
        // Expect frame contains data + CRC; return without CRC if space allows
        if (frame.len < 2) {
            return -3;
        }
        uint8_t data_len = frame.len - 1; // strip CRC
        if (data_len > max_len) {
            data_len = max_len;
        }
        memcpy(data, frame.data, data_len);
        return data_len;
    }
    return -1; // timeout
}
