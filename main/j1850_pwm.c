/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_log.h"

#include "soc/gpio_sig_map.h"
#include "soc/gpio_reg.h"
#include "pinout.h"
#include "j1850_pwm.h"

#define TAG "J1850_PWM"


/*
Physical Node Addresses:
ID Module

$00-$1F Powertrain controllers
$00-$0F Integration/Manufacturer Expansion
$10-$17 Engine controllers
$18-$1F Transmission controllers
$20-$3F Chassis controllers
$20-$27 Integration/Manufacturer Expansion
$28-$2F Brake controllers
$30-$37 Sterring controllers
$38-$3F Suspension controllers
$40-$C7 Body controllers
$40-$57 Integration/Manufacturer Expansion
$48-$5F Restraints
$60-$6F Driver information/Diplays
$70-$7F Lighting
$80-$8F Enterntainment
$90-$97 Personal communications
$98-$9F Climate control (HVAC)
$A0-$BF Convenience (doos, Seats, Windows, etc.)
$C0-$C7 Security
$C8-$CB Electric Vehicle Energy Transfer System (EV-ETS)
$C8 Utility connection services
$C9 AC to AC conversion
$CA AC to DC conversion
$CB Energy storage management
$CC-$CF Future expansion
$D0-$EF Manufacturer specific
$F0-$FD Off-Board Testers/Diagnostic scan tools
$FE All nodes
$FF Null node


Max Length 12 bytes MSB

*/


// --- Configuration ---
#define RMT_RESOLUTION_HZ   1000000 // 1MHz resolution, 1 tick = 1 microsecond

// --- J1850 PWM Timings (in microseconds) ---
#define PWM_T_BIT           24  /* Total time for one bit cell */
#define PWM_T_SHORT         8   /* Active pulse for logic '1' */
#define PWM_T_LONG          16  /* Active pulse for logic '0' */
#define PWM_T_SOF_HIGH      31  /* SOF high pulse */
#define PWM_T_SOF_LOW       17  /* SOF low pulse */

#define REG32(addr) (*(volatile uint32_t *)(addr))

#define BUFFER_SIZE_SYMBOLS 256

// Tolerance for receiving pulses
#define PWM_PULSE_TOLERANCE 4

// RMT Channel Handles
static rmt_channel_handle_t g_tx_channel = NULL;
static rmt_channel_handle_t g_rx_channel = NULL;

static QueueHandle_t rx_frame_queue;

static rmt_symbol_word_t *g_rmt_buffer;

static const rmt_receive_config_t g_rx_config = {
    .signal_range_min_ns = 3000,
    .signal_range_max_ns = 24000,
};

struct j1850_frame {
    uint8_t len;
    uint8_t data[16];
};

// Custom encoder struct
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_handle_t copy_encoder;
} rmt_j1850_pwm_encoder_t;


// --- Custom RMT Encoder for J1850 PWM ---
size_t rmt_encode_j1850_pwm(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state) {
    rmt_j1850_pwm_encoder_t *pwm_encoder = __containerof(encoder, rmt_j1850_pwm_encoder_t, base);
    rmt_encoder_handle_t copy_encoder = pwm_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    const uint8_t *data = (const uint8_t *)primary_data;

    // Start of Frame (SOF) symbol
    rmt_symbol_word_t sof_symbol = {
        .level0 = 1, .duration0 = PWM_T_SOF_HIGH,
        .level1 = 0, .duration1 = PWM_T_SOF_LOW
    };
    encoded_symbols += copy_encoder->encode(copy_encoder, channel, &sof_symbol, sizeof(sof_symbol), &session_state);

    // Encode each byte
    for (int i = 0; i < data_size; i++) {
        uint8_t current_byte = data[i];
        for (int j = 0; j < 8; j++) {
            bool bit = (current_byte >> (7 - j)) & 1; // MSB first
            uint32_t high_duration = bit ? PWM_T_SHORT : PWM_T_LONG;
            
            rmt_symbol_word_t bit_symbol = {
                .level0 = 1, .duration0 = high_duration,
                .level1 = 0, .duration1 = PWM_T_BIT - high_duration
            };
            encoded_symbols += copy_encoder->encode(copy_encoder, channel, &bit_symbol, sizeof(bit_symbol), &session_state);
        }
    }

    *ret_state = session_state;
    return encoded_symbols;
}

static esp_err_t rmt_j1850_encoder_reset(rmt_encoder_t *encoder)
{
    // Our simple encoder is stateless, so there's nothing to reset.
    // We just need to implement the function to fulfill the RMT encoder interface.
    return ESP_OK;
}

static rmt_j1850_pwm_encoder_t pwm_encoder = {
    .base.encode = rmt_encode_j1850_pwm,
    .base.reset = rmt_j1850_encoder_reset,
};


static int IRAM_ATTR j1850_pwm_decode(const rmt_symbol_word_t *remote_codes, size_t num_symbols, uint8_t *decoded_bytes, size_t decoded_size) {
    const uint32_t TSHORT_MIN    = PWM_T_SHORT - PWM_PULSE_TOLERANCE;
    const uint32_t TSHORT_MAX    = PWM_T_SHORT + PWM_PULSE_TOLERANCE;
    const uint32_t TLONG_MIN     = PWM_T_LONG - PWM_PULSE_TOLERANCE;
    const uint32_t TLONG_MAX     = PWM_T_LONG + PWM_PULSE_TOLERANCE;

    int byte_idx = 0;
    int bit_idx = 0;
    uint8_t current_byte = 0;

    //bool flip = remote_codes[0].level0 ? false : true;

    for (size_t i = 0; i < num_symbols; i++) {
        uint32_t d0;

        // if (flip) {
        //     d0 = remote_codes[i].duration1;
        // }
        // else {
            d0 = remote_codes[i].duration0;
        // }

        // Decode '1' (Short Pulse)
        if (d0 > TSHORT_MIN && d0 < TSHORT_MAX) {
            current_byte |= (1 << (7 - bit_idx));
            bit_idx++;
        }
        // Decode '0' (Long Pulse)
        else if (d0 > TLONG_MIN && d0 < TLONG_MAX) {
            // No need to set the bit, just advance the index
            bit_idx++;
        }

        // If a full byte has been assembled, store it
        if (bit_idx == 8) {
            if (byte_idx < decoded_size) {
                decoded_bytes[byte_idx++] = current_byte;
            }
            bit_idx = 0;
            current_byte = 0;

            //TODO: hacky to skip ack for now;
            if (remote_codes[i].duration0 > 24 || remote_codes[i].duration1 > 24) {
                break;
            }
        }
    }

    return byte_idx;
}

// --- ISR Callback ---
static bool IRAM_ATTR on_receive_callback(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *event_data, void *user_data) {
    BaseType_t high_task_woken = pdFALSE;
    struct j1850_frame rx_frame;
    rmt_symbol_word_t *buffer_just_filled = event_data->received_symbols;
    esp_err_t ret;

    if (event_data->num_symbols > 8) {
        rx_frame.len = j1850_pwm_decode(buffer_just_filled, event_data->num_symbols, rx_frame.data, sizeof(rx_frame.data));

        /* ACK if frame is for us*/
        if (rx_frame.len > 2 && rx_frame.data[1] == 0x6b)
        {
            const uint8_t ack_id = 0xf1;
            uint32_t old_func_sel = REG32(GPIO_FUNC14_OUT_SEL_CFG_REG);

            REG32(GPIO_FUNC14_OUT_SEL_CFG_REG) = 0x100;
            REG32(GPIO_FUNC15_OUT_SEL_CFG_REG) = 0x100;

            // Clear output value
            REG32(GPIO_OUT_W1TS_REG) = (BIT(14) | BIT(15));
            // Output Enable
            REG32(GPIO_ENABLE_W1TS_REG) = (BIT(14) | BIT(15));

            // Clock out bits
            for (uint8_t b = 0; b < 8; b++) {
                if (ack_id & BIT(7 - b)) {
                    // Bit 1
                    REG32(GPIO_OUT_W1TS_REG) = (BIT(14) | BIT(15));
                    for (int i = 0; i < 212; i++) { //8.64
                        asm volatile("nop");
                    }
                    REG32(GPIO_OUT_W1TC_REG) = (BIT(14) | BIT(15));
                    for (int i = 0; i < 420; i++) { //15.36 Good
                        asm volatile("nop");
                    }
                }
                else {
                    // Bit 0
                    REG32(GPIO_OUT_W1TS_REG) = (BIT(14) | BIT(15));
                    for (int i = 0; i < 420; i++) { //16.64 Good
                        asm volatile("nop");
                    }
                    REG32(GPIO_OUT_W1TC_REG) = (BIT(14) | BIT(15));
                    for (int i = 0; i < 212; i++) { //7.36
                        asm volatile("nop");
                    }
                }
            }

            // restore original mode
            REG32(GPIO_FUNC14_OUT_SEL_CFG_REG) = old_func_sel;
            REG32(GPIO_FUNC15_OUT_SEL_CFG_REG) = old_func_sel;
        }

        // Send the buffer that was just filled to the processing task.
        xQueueSendFromISR(rx_frame_queue, &rx_frame, &high_task_woken);
    }
    
    // Re-arm the receiver
    ret = rmt_receive(channel, g_rmt_buffer, BUFFER_SIZE_SYMBOLS * sizeof(rmt_symbol_word_t), &g_rx_config);
    if (unlikely(ret != ESP_OK)) {
        ESP_LOGE(TAG, "Critical error in RMT driver! ret = 0x%x", ret);
    }

    return high_task_woken == pdTRUE;
}

static bool IRAM_ATTR on_trans_done_callback(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx) {
    // Transmit done so trigger RX again
    ESP_ERROR_CHECK(rmt_enable(g_rx_channel));
    ESP_ERROR_CHECK(rmt_receive(g_rx_channel, g_rmt_buffer, BUFFER_SIZE_SYMBOLS * sizeof(rmt_symbol_word_t), &g_rx_config));
    return false;
}

static uint8_t j1850_cal_checksum(const uint8_t *data, size_t length) {
    // The CRC-8 polynomial (x^8 + x^4 + x^3 + x^2 + 1)
    const uint8_t polynomial = 0x1D;
    uint8_t crc = 0xFF;

    for (size_t i = 0; i < length; ++i) {
        // XOR the current CRC value with the current data byte.
        crc ^= data[i];

        // Process each of the 8 bits in the current byte.
        for (int j = 0; j < 8; ++j) {
            // Check if the most significant bit (MSB) of the CRC is 1.
            if (crc & 0x80) {
                // If the MSB is 1, shift the CRC left by one bit and then
                // XOR it with the polynomial.
                crc = (crc << 1) ^ polynomial;
            } else {
                // If the MSB is 0, just shift the CRC left by one bit.
                crc <<= 1;
            }
        }
    }

    // The final step is to invert the result by XORing with 0xFF.
    return crc ^ 0xFF;
}

static bool g_started;

void j1850_pwm_setup(void) {

    if (g_started) {
        ESP_LOGI(TAG, "J1850 PWM already started");
        return;
    }

    ESP_LOGI(TAG, "Initializing RMT for J1850 PWM");

    // --- TX Channel Setup ---
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = PIN_J1850_TX_P,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 128,
        .trans_queue_depth = 4,
    };

    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &g_tx_channel));

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_encoder_config, &pwm_encoder.copy_encoder));

    ESP_LOGI(TAG, "register TX done callback");
    rmt_tx_event_callbacks_t cbs = {
        .on_trans_done = on_trans_done_callback,
    };
    ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(g_tx_channel, &cbs, NULL));

    ESP_ERROR_CHECK(rmt_enable(g_tx_channel));

    gpio_reset_pin(PIN_J1850_TX_N);
    esp_rom_gpio_connect_out_signal(PIN_J1850_TX_N, RMT_SIG_OUT0_IDX, false, false);


    // Create the queues.
    rx_frame_queue = xQueueCreate(8, sizeof(struct j1850_frame));
    assert(rx_frame_queue);

    g_rmt_buffer = heap_caps_aligned_calloc(64, BUFFER_SIZE_SYMBOLS, sizeof(rmt_symbol_word_t),
                                                MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    assert(g_rmt_buffer);

    // --- RX Channel Setup ---
    rmt_rx_channel_config_t rx_chan_config = {
        .gpio_num = PIN_J1850_PWM_RX,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RESOLUTION_HZ,
        .mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL,
        .flags.with_dma = 1,
    };
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &g_rx_channel));


    ESP_LOGI(TAG, "register RX done callback");
    rmt_rx_event_callbacks_t tx_cbs = {
        .on_recv_done = on_receive_callback,
    };
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(g_rx_channel, &tx_cbs, NULL));

    ESP_ERROR_CHECK(rmt_enable(g_rx_channel));
    ESP_LOGI(TAG, "RMT RX enabled.");

    ESP_ERROR_CHECK(rmt_receive(g_rx_channel, g_rmt_buffer, BUFFER_SIZE_SYMBOLS * sizeof(rmt_symbol_word_t), &g_rx_config));
    ESP_LOGI(TAG, "RMT started, listening into the first buffer.");

    g_started = true;
}

void j1850_pwm_teardown(void) {
    if (!g_started) {
        ESP_LOGE(TAG, "J1850 not started yet. Cannot teardown");
        return;
    }

    ESP_LOGI(TAG, "Tearing down J1850");

    rmt_disable(g_rx_channel);
    rmt_disable(g_tx_channel);

    rmt_del_channel(g_rx_channel);
    rmt_del_channel(g_tx_channel);

    rmt_del_encoder(pwm_encoder.copy_encoder);

    if (g_rmt_buffer) {
        free(g_rmt_buffer);
        g_rmt_buffer = NULL;
    }

    if (rx_frame_queue) {
        vQueueDelete(rx_frame_queue);
        rx_frame_queue = NULL;
    }

    g_started = false;
}

int j1850_pwm_send(const uint8_t *data, uint8_t len) {
    esp_err_t ret;
    static const rmt_transmit_config_t tx_config = {
        .loop_count = 0
    };
    struct j1850_frame frame = {
        .len = len + 1,
    };

    if (len > sizeof(frame.data) - 1) {
        ESP_LOGE(TAG, "frame of %u bytes is too long for J1850", (unsigned)len);
        return -10;
    }

    // Make sure all previous messages transmited
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(g_tx_channel, -1));

    // Disable RX
    ESP_ERROR_CHECK(rmt_disable(g_rx_channel));

    memcpy(frame.data, data, len);

    frame.data[len] = j1850_cal_checksum(frame.data, len);

    // TODO: need to check for ack and re-transmit if needed (up to 3 times)
    ret = rmt_transmit(g_tx_channel, &pwm_encoder.base, frame.data, frame.len, &tx_config);
    if (ret == ESP_OK) {
        return 0;
    }
    else {
        return -1;
    }
}

int j1850_pwm_receive(uint8_t *data, uint8_t len, TickType_t xTicksToWait) {
    struct j1850_frame rx_frame;

    if (xQueueReceive(rx_frame_queue, &rx_frame, xTicksToWait) == pdPASS) {
        assert(rx_frame.len); // should be no empty frame

        if (rx_frame.len > len) {
            return -2;
        }
        uint8_t cksum = j1850_cal_checksum(rx_frame.data, rx_frame.len - 1);
        if (rx_frame.data[rx_frame.len - 1] != cksum) {
            ESP_LOG_BUFFER_HEX(TAG, rx_frame.data, rx_frame.len );
            return -3;
        }

        memcpy(data, rx_frame.data, rx_frame.len);
        return rx_frame.len;
    }
    else {
        return -1;
    }
}
