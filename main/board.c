/* SPDX-License-Identifier: GPL-3.0-only */
#include "board.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_ipc.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "nvs_flash.h"
#include "pinout.h"
#include "shell.h"

#define TAG "board"

#define CALIBRATION_SCALE_FACTOR 10000.0f
#define ADC_NUM_SAMPLES 10

/* Maximum duty for the LEDC_TIMER_12_BIT high side PWM timer */
#define HS_DUTY_MAX ((1 << 12) - 1)

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cal_handle;
static uint8_t board_id;
static struct {
    int32_t vbatt_gain;
    int32_t vbatt_offset;
    int32_t hs_sense_gain;
    int32_t hs_sense_offset;
    int32_t hs_duty_gain;
    int32_t hs_duty_offset;
    bool calibrated;
} pc_calibration;

static int board_adc_read_boardid(void);

static void board_gpio_isr_install(void *arg) {
    *(esp_err_t *)arg =
        gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
}

void board_setup(void) {
    /* J1850 driver */
    gpio_reset_pin(PIN_J1850_MODE);
    gpio_set_level(PIN_J1850_MODE,
                   1); // Default to lower drive voltage - PWM mode - 5V
    gpio_set_direction(PIN_J1850_MODE, GPIO_MODE_OUTPUT);

    // gpio_reset_pin(PIN_J1850_TX_P); // calling this cause glitch on J1850 bus
    // on boot up.
    gpio_set_level(PIN_J1850_TX_P, 0);
    gpio_set_direction(PIN_J1850_TX_P, GPIO_MODE_OUTPUT);

    // gpio_reset_pin(PIN_J1850_TX_N);
    gpio_set_level(PIN_J1850_TX_N, 0);
    gpio_set_direction(PIN_J1850_TX_N, GPIO_MODE_OUTPUT);

    gpio_reset_pin(PIN_J1850_PWM_RX);
    gpio_set_direction(PIN_J1850_PWM_RX, GPIO_MODE_INPUT);

    gpio_reset_pin(PIN_J1850_VPW_RX);
    gpio_set_direction(PIN_J1850_VPW_RX, GPIO_MODE_INPUT);

    /* CAN0 transceiver: silent until a bus is actually opened, so the
     * adapter cannot drive a vehicle bus it was only plugged into. */
    gpio_reset_pin(PIN_CAN0_SILENT);
    gpio_set_level(PIN_CAN0_SILENT, 1);
    gpio_set_direction(PIN_CAN0_SILENT, GPIO_MODE_OUTPUT);

    /* K-Line driver */
    gpio_reset_pin(PIN_KLINE_TX);
    gpio_set_level(PIN_KLINE_TX, 1);
    gpio_set_direction(PIN_KLINE_TX, GPIO_MODE_OUTPUT);

    gpio_reset_pin(PIN_KLINE_nSILENT);
    gpio_set_level(PIN_KLINE_nSILENT, 0); // Default to silent
    gpio_set_direction(PIN_KLINE_nSILENT, GPIO_MODE_OUTPUT);

    gpio_reset_pin(PIN_KLINE_RX);
    gpio_pullup_en(
        PIN_KLINE_RX); // Enable pull since there are no external pull up
    gpio_set_direction(PIN_KLINE_RX, GPIO_MODE_INPUT);

    /* High side driver */
    gpio_reset_pin(PIN_HS_BOOST_EN);
    gpio_set_level(PIN_HS_BOOST_EN, 0);
    gpio_set_direction(PIN_HS_BOOST_EN, GPIO_MODE_OUTPUT);

    gpio_reset_pin(PIN_HS_OBD_6);
    gpio_set_level(PIN_HS_OBD_6, 0);
    gpio_set_direction(PIN_HS_OBD_6, GPIO_MODE_OUTPUT);

    gpio_reset_pin(PIN_HS_OBD_9);
    gpio_set_level(PIN_HS_OBD_9, 0);
    gpio_set_direction(PIN_HS_OBD_9, GPIO_MODE_OUTPUT);

    gpio_reset_pin(PIN_HS_OBD_11);
    gpio_set_level(PIN_HS_OBD_11, 0);
    gpio_set_direction(PIN_HS_OBD_11, GPIO_MODE_OUTPUT);

    gpio_reset_pin(PIN_HS_OBD_12);
    gpio_set_level(PIN_HS_OBD_12, 0);
    gpio_set_direction(PIN_HS_OBD_12, GPIO_MODE_OUTPUT);

    gpio_reset_pin(PIN_HS_OBD_13);
    gpio_set_level(PIN_HS_OBD_13, 0);
    gpio_set_direction(PIN_HS_OBD_13, GPIO_MODE_OUTPUT);

    gpio_reset_pin(PIN_HS_OBD_14);
    gpio_set_level(PIN_HS_OBD_14, 0);
    gpio_set_direction(PIN_HS_OBD_14, GPIO_MODE_OUTPUT);

    /* High side voltage adjust PWM */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_12_BIT, // resolution of PWM duty
        .freq_hz = (80000000 / (1 << 12)),    // frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE,    // timer mode
        .timer_num = LEDC_TIMER_1,            // timer index
        .clk_cfg = LEDC_USE_APB_CLK,          // Auto select the source clock
    };
    ESP_LOGI(TAG, "HS PWM Config: ledc_timer.freq_hz = %ld\n",
             ledc_timer.freq_hz);
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t hs_vadj_channel = {.speed_mode = LEDC_LOW_SPEED_MODE,
                                             .channel = LEDC_CHANNEL_0,
                                             .timer_sel = LEDC_TIMER_1,
                                             .intr_type = LEDC_INTR_DISABLE,
                                             .gpio_num = PIN_HS_VOLTAGE_ADJUST,
                                             .duty = 0, // Set duty to 0%
                                             .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&hs_vadj_channel));

    /* Low side driver */
    gpio_reset_pin(PIN_LS_OBD_15);
    gpio_set_level(PIN_LS_OBD_15, 0);
    gpio_set_direction(PIN_LS_OBD_15, GPIO_MODE_OUTPUT);

    esp_err_t gpio_isr_err = ESP_OK;
    /* Keep shared GPIO timing interrupts away from Bluetooth on core 0. */
    ESP_ERROR_CHECK(
        esp_ipc_call_blocking(1, board_gpio_isr_install, &gpio_isr_err));
    ESP_ERROR_CHECK(gpio_isr_err);

    /* ADC inputs signals */
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_1, &config));
    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_8, &config));

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(
        adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cal_handle));

    /* Read board id from register strap */
    board_id = (board_adc_read_boardid() >> 8) & 0xff;

    /* Retrieve calibration data from persistent config nvs */
    nvs_handle_t pc_handle;
    esp_err_t err;

    err = nvs_open("pc", NVS_READONLY, &pc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        goto out;
    }

    err = nvs_get_i32(pc_handle, "vbatt_gain", &pc_calibration.vbatt_gain);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) failed to get vbatt_gain!",
                 esp_err_to_name(err));
        goto out_nvs;
    }

    err = nvs_get_i32(pc_handle, "vbatt_offset", &pc_calibration.vbatt_offset);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) failed to get vbatt_offset!",
                 esp_err_to_name(err));
        goto out_nvs;
    }

    err =
        nvs_get_i32(pc_handle, "hs_sense_gain", &pc_calibration.hs_sense_gain);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) failed to get hs_sense_gain!",
                 esp_err_to_name(err));
        goto out_nvs;
    }

    err = nvs_get_i32(pc_handle, "hs_sense_offset",
                      &pc_calibration.hs_sense_offset);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) failed to get hs_sense_offset!",
                 esp_err_to_name(err));
        goto out_nvs;
    }

    err = nvs_get_i32(pc_handle, "hs_duty_gain", &pc_calibration.hs_duty_gain);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) failed to get hs_duty_gain!",
                 esp_err_to_name(err));
        goto out_nvs;
    }

    err = nvs_get_i32(pc_handle, "hs_duty_offset",
                      &pc_calibration.hs_duty_offset);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) failed to get hs_duty_offset!",
                 esp_err_to_name(err));
        goto out;
    }

    pc_calibration.calibrated = true;

out_nvs:
    nvs_close(pc_handle);
out:

    if (!pc_calibration.calibrated) {
        ESP_LOGE(TAG, "Voltage sense calibration missing. Fallback to default");
        pc_calibration.vbatt_gain = 308982;
        pc_calibration.vbatt_offset = 2361515;
        pc_calibration.hs_sense_gain = 229142;
        pc_calibration.hs_sense_offset = 601008;
        pc_calibration.hs_duty_gain = 42840;
        pc_calibration.hs_duty_offset = 36930392;
    } else {
        ESP_LOGI(TAG, "cal: vbatt_gain     : %" PRId32,
                 pc_calibration.vbatt_gain);
        ESP_LOGI(TAG, "cal: vbatt_offset   : %" PRId32,
                 pc_calibration.vbatt_offset);
        ESP_LOGI(TAG, "cal: hs_sense_gain  : %" PRId32,
                 pc_calibration.hs_sense_gain);
        ESP_LOGI(TAG, "cal: hs_sense_offset: %" PRId32,
                 pc_calibration.hs_sense_offset);
        ESP_LOGI(TAG, "cal: hs_duty_gain   : %" PRId32,
                 pc_calibration.hs_duty_gain);
        ESP_LOGI(TAG, "cal: hs_duty_offset : %" PRId32,
                 pc_calibration.hs_duty_offset);
    }
}

void board_print_info(void) {
    printf("\tcalibrated     : %d\n", pc_calibration.calibrated);
    printf("\tvbatt_gain     : %" PRId32 "\n", pc_calibration.vbatt_gain);
    printf("\tvbatt_offset   : %" PRId32 "\n", pc_calibration.vbatt_offset);
    printf("\ths_sense_gain  : %" PRId32 "\n", pc_calibration.hs_sense_gain);
    printf("\ths_sense_offset: %" PRId32 "\n", pc_calibration.hs_sense_offset);
    printf("\ths_duty_gain   : %" PRId32 "\n", pc_calibration.hs_duty_gain);
    printf("\ths_duty_offset : %" PRId32 "\n", pc_calibration.hs_duty_offset);
}

/* Calibration constants addressable by name. The name doubles as the NVS key,
 * and the order matches board_print_info(), so a dump taken from a working
 * unit can be typed straight back into a board whose NVS was erased. */
static const struct {
    const char *name;
    int32_t *value;
} pc_cal_fields[] = {
    {"vbatt_gain", &pc_calibration.vbatt_gain},
    {"vbatt_offset", &pc_calibration.vbatt_offset},
    {"hs_sense_gain", &pc_calibration.hs_sense_gain},
    {"hs_sense_offset", &pc_calibration.hs_sense_offset},
    {"hs_duty_gain", &pc_calibration.hs_duty_gain},
    {"hs_duty_offset", &pc_calibration.hs_duty_offset},
};

#define PC_CAL_NUM_FIELDS (sizeof(pc_cal_fields) / sizeof(pc_cal_fields[0]))

const char *board_calibration_field(int index) {
    if (index < 0 || index >= (int)PC_CAL_NUM_FIELDS) {
        return NULL;
    }

    return pc_cal_fields[index].name;
}

/** @brief True once every calibration key is present in the open namespace. */
static bool pc_cal_all_stored(nvs_handle_t pc_handle) {
    for (size_t i = 0; i < PC_CAL_NUM_FIELDS; i++) {
        int32_t val;

        if (nvs_get_i32(pc_handle, pc_cal_fields[i].name, &val) != ESP_OK) {
            return false;
        }
    }

    return true;
}

esp_err_t board_calibration_set(const char *name, int32_t value) {
    nvs_handle_t pc_handle;
    esp_err_t err;
    size_t i;

    for (i = 0; i < PC_CAL_NUM_FIELDS; i++) {
        if (strcmp(name, pc_cal_fields[i].name) == 0) {
            break;
        }
    }
    if (i == PC_CAL_NUM_FIELDS) {
        return ESP_ERR_NOT_FOUND;
    }

    err = nvs_open("pc", NVS_READWRITE, &pc_handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_i32(pc_handle, name, value);
    if (err == ESP_OK) {
        err = nvs_commit(pc_handle);
    }
    if (err != ESP_OK) {
        nvs_close(pc_handle);
        return err;
    }

    /* Apply it live as well, so the value is in use before the next reboot. */
    *pc_cal_fields[i].value = value;
    pc_calibration.calibrated = pc_cal_all_stored(pc_handle);

    nvs_close(pc_handle);

    ESP_LOGI(TAG, "cal: %s set to %" PRId32, name, value);

    return ESP_OK;
}

static int board_adc_read_ch(int ch) {
    int val;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ch, &val));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cal_handle, val, &val));
    return val;
}

static int board_adc_read_boardid(void) {
    return board_adc_read_ch(ADC_CHANNEL_8);
}

static int board_adc_read_vbatt(void) {
    return board_adc_read_ch(ADC_CHANNEL_1);
}

static int board_adc_read_hs_vsense(void) {
    return board_adc_read_ch(ADC_CHANNEL_0);
}

int32_t board_get_vbatt(void) {
    int i;
    int adc_val = 0;

    for (i = 0; i < ADC_NUM_SAMPLES; i++) {
        adc_val += board_adc_read_vbatt();
    }
    adc_val = adc_val / ADC_NUM_SAMPLES;

    float adc_gain = pc_calibration.vbatt_gain / CALIBRATION_SCALE_FACTOR;
    float adc_offset = pc_calibration.vbatt_offset / CALIBRATION_SCALE_FACTOR;

    float volt = (adc_val * adc_gain) + adc_offset;

    return (int32_t)volt;
}

int32_t board_get_hs_vsense(void) {
    int i;
    int adc_val = 0;

    for (i = 0; i < ADC_NUM_SAMPLES; i++) {
        adc_val += board_adc_read_hs_vsense();
    }
    adc_val = adc_val / ADC_NUM_SAMPLES;

    float adc_gain = pc_calibration.hs_sense_gain / CALIBRATION_SCALE_FACTOR;
    float adc_offset =
        pc_calibration.hs_sense_offset / CALIBRATION_SCALE_FACTOR;

    float volt = (adc_val * adc_gain) + adc_offset;

    return (int32_t)volt;
}

void board_set_hs_boost_en(int state) {
    gpio_set_level(PIN_HS_BOOST_EN, state);
}

static void board_set_hs_duty(uint32_t duty) {
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

void board_set_hs_voltage(uint32_t millivolt) {
    /* 0 - Switch off PWM output */
    if (millivolt == 0) {
        board_set_hs_duty(0);
        return;
    }

    /* clamp value to 5V - 20V*/
    if (millivolt < 5000) {
        millivolt = 5000;
    }
    if (millivolt > 20000) {
        millivolt = 20000;
    }

    float pwm_gain = pc_calibration.hs_duty_gain / CALIBRATION_SCALE_FACTOR;
    float pwm_offset = pc_calibration.hs_duty_offset / CALIBRATION_SCALE_FACTOR;

    /* An uncalibrated board reads back a zero gain from NVS. Dividing by it
     * yields inf/NaN, and the resulting duty would drive an arbitrary voltage
     * onto the OBD connector, so refuse to output anything instead. */
    if (pwm_gain == 0.0f) {
        ESP_LOGE(
            TAG,
            "HS voltage not calibrated (hs_duty_gain is 0), output disabled");
        board_set_hs_duty(0);
        return;
    }

    float duty = (millivolt - pwm_offset) / pwm_gain;

    /* Clamp to the PWM resolution. Out of range values make ledc_set_duty()
     * fail, which trips ESP_ERROR_CHECK() and panics the device. */
    int32_t iduty = (int32_t)duty;
    if (iduty < 0) {
        iduty = 0;
    }
    if (iduty > HS_DUTY_MAX) {
        iduty = HS_DUTY_MAX;
    }

    board_set_hs_duty((uint32_t)iduty);
}

void board_set_hs_state(enum hs_pin pin, int state) {
    switch (pin) {
    case HS_OBD_PIN_6:
        gpio_set_level(PIN_HS_OBD_6, state);
        break;
    case HS_OBD_PIN_9:
        gpio_set_level(PIN_HS_OBD_9, state);
        break;
    case HS_OBD_PIN_11:
        gpio_set_level(PIN_HS_OBD_11, state);
        break;
    case HS_OBD_PIN_12:
        gpio_set_level(PIN_HS_OBD_12, state);
        break;
    case HS_OBD_PIN_13:
        gpio_set_level(PIN_HS_OBD_13, state);
        break;
    case HS_OBD_PIN_14:
        gpio_set_level(PIN_HS_OBD_14, state);
        break;
    default:
        ESP_LOGE(TAG, "Unknow pin: %d\n", pin);
        abort();
        break;
    }
}

void board_set_ls_state(enum ls_pin pin, int state) {
    switch (pin) {
    case LS_OBD_PIN_15:
        gpio_set_level(PIN_LS_OBD_15, state);
        break;
    default:
        ESP_LOGE(TAG, "Unknow pin: %d\n", pin);
        abort();
        break;
    }
}

void board_hs_ls_reset_state(void) {
    board_set_hs_boost_en(0);
    board_set_hs_duty(0);
    gpio_set_level(PIN_HS_OBD_6, 0);
    gpio_set_level(PIN_HS_OBD_9, 0);
    gpio_set_level(PIN_HS_OBD_11, 0);
    gpio_set_level(PIN_HS_OBD_12, 0);
    gpio_set_level(PIN_HS_OBD_13, 0);
    gpio_set_level(PIN_HS_OBD_14, 0);
    gpio_set_level(PIN_LS_OBD_15, 0);
}

#define HS_CAL_LOW_DUTY 400
#define HS_CAL_HIGH_DUTY 3800

void board_calibrate_hs(void) {
    char str[50] = {0};
    float voltl = 0.0f, volth = 0.0f;
    int i = 0, adcl = 0, adch = 0;

    /* reset to know state */
    board_hs_ls_reset_state();

    printf("Set HS duty cycle to %d\n", HS_CAL_LOW_DUTY);
    board_set_hs_duty(HS_CAL_LOW_DUTY);
    board_set_hs_boost_en(1);
    board_set_hs_state(HS_OBD_PIN_12, 1);

    printf("Wait for voltage to settle\n");
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("Please enter voltage reading in mV:\n");
    shell_getline(str, sizeof(str));
    sscanf(str, "%f", &voltl);
    printf("You entered: %.4f mV\n", voltl);

    adcl = 0;
    for (i = 0; i < 100; i++) {
        adcl += board_adc_read_hs_vsense();
    }
    adcl = adcl / 100;
    printf("ADC HS sense: %d\n", adcl);

    printf("Set HS duty cycle to %d\n", HS_CAL_HIGH_DUTY);
    board_set_hs_duty(HS_CAL_HIGH_DUTY);

    printf("Wait for voltage to settle\n");
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("Please enter voltage reading in mV:\n");
    shell_getline(str, sizeof(str));
    sscanf(str, "%f", &volth);
    printf("You entered: %.4f mV\n", volth);

    adch = 0;
    for (i = 0; i < 100; i++) {
        adch += board_adc_read_hs_vsense();
    }
    adch = adch / 100;
    printf("ADC HS sense: %d\n", adch);

    float adc_gain_f = (volth - voltl) / (adch - adcl);
    float adc_offset_f = voltl - (adc_gain_f * adcl);

    printf("ADC gain: %.4f\n", adc_gain_f);
    printf("ADC offset: %.4f\n", adc_offset_f);

    float pwm_gain_f = (volth - voltl) / (HS_CAL_HIGH_DUTY - HS_CAL_LOW_DUTY);
    float pwm_offset_f = voltl - (pwm_gain_f * HS_CAL_LOW_DUTY);

    printf("PWM gain: %.4f\n", pwm_gain_f);
    printf("PWM offset: %.4f\n", pwm_offset_f);

    board_hs_ls_reset_state();

    int32_t adc_gain_i32 = (int32_t)(adc_gain_f * CALIBRATION_SCALE_FACTOR);
    int32_t adc_offset_i32 = (int32_t)(adc_offset_f * CALIBRATION_SCALE_FACTOR);
    int32_t pwm_gain_i32 = (int32_t)(pwm_gain_f * CALIBRATION_SCALE_FACTOR);
    int32_t pwm_offset_i32 = (int32_t)(pwm_offset_f * CALIBRATION_SCALE_FACTOR);

    printf("Calibration complete! Commit result to flash (y/N)?\n");
    shell_getline(str, sizeof(str));

    if (strcmp(str, "y") == 0) {
        nvs_handle_t pc_handle;
        esp_err_t err;

        err = nvs_open("pc", NVS_READWRITE, &pc_handle);
        if (err != ESP_OK) {
            printf("Error (%s) opening NVS handle!", esp_err_to_name(err));
            nvs_close(pc_handle);
            return;
        }

        err = nvs_set_i32(pc_handle, "hs_sense_gain", adc_gain_i32);
        if (err != ESP_OK) {
            printf("Error (%s) failed to set hs_sense_gain!",
                   esp_err_to_name(err));
            nvs_close(pc_handle);
            return;
        }

        err = nvs_set_i32(pc_handle, "hs_sense_offset", adc_offset_i32);
        if (err != ESP_OK) {
            printf("Error (%s) failed to set hs_sense_offset!",
                   esp_err_to_name(err));
            nvs_close(pc_handle);
            return;
        }

        err = nvs_set_i32(pc_handle, "hs_duty_gain", pwm_gain_i32);
        if (err != ESP_OK) {
            printf("Error (%s) failed to set hs_duty_gain!",
                   esp_err_to_name(err));
            nvs_close(pc_handle);
            return;
        }

        err = nvs_set_i32(pc_handle, "hs_duty_offset", pwm_offset_i32);
        if (err != ESP_OK) {
            printf("Error (%s) failed to set hs_duty_offset!",
                   esp_err_to_name(err));
            nvs_close(pc_handle);
            return;
        }

        err = nvs_commit(pc_handle);
        if (err != ESP_OK) {
            printf("Error (%s) failed to commit NVS!", esp_err_to_name(err));
            nvs_close(pc_handle);
            return;
        }

        nvs_close(pc_handle);
        printf("Commit to flash successfully.\n");
    } else {
        printf("Cancelled.\n");
    }
}

#define VBATT_CAL_LOW_V 10.0f
#define VBATT_CAL_HIGH_V 26.0f

void board_calibrate_vbatt(void) {
    char str[2] = {0};
    int i = 0, adcl = 0, adch = 0;
    printf("Set VBATT power supply to %.1f V\n", VBATT_CAL_LOW_V);
    printf("Press enter to continue...\n");
    shell_getline(str, sizeof(str));

    adcl = 0;
    for (i = 0; i < 100; i++) {
        adcl += board_adc_read_vbatt();
    }
    adcl = adcl / 100;
    printf("ADC VBATT sense: %d\n", adcl);

    printf("Set VBATT power supply to %.1f V\n", VBATT_CAL_HIGH_V);
    printf("Press enter to continue...\n");
    shell_getline(str, sizeof(str));

    adch = 0;
    for (i = 0; i < 100; i++) {
        adch += board_adc_read_vbatt();
    }
    adch = adch / 100;
    printf("ADC VBATT sense: %d\n", adch);

    float mv_low = VBATT_CAL_LOW_V * 1000.0f;
    float mv_high = VBATT_CAL_HIGH_V * 1000.0f;

    float adc_gain_f = (mv_high - mv_low) / (adch - adcl);
    float adc_offset_f = mv_low - (adc_gain_f * adcl);

    printf("ADC gain: %.4f\n", adc_gain_f);
    printf("ADC offset: %.4f\n", adc_offset_f);

    int32_t adc_gain_i32 = (int32_t)(adc_gain_f * CALIBRATION_SCALE_FACTOR);
    int32_t adc_offset_i32 = (int32_t)(adc_offset_f * CALIBRATION_SCALE_FACTOR);

    printf("Calibration complete! Commit result to flash (y/N)?\n");
    shell_getline(str, sizeof(str));

    if (strcmp(str, "y") == 0) {
        nvs_handle_t pc_handle;
        esp_err_t err;

        err = nvs_open("pc", NVS_READWRITE, &pc_handle);
        if (err != ESP_OK) {
            printf("Error (%s) opening NVS handle!", esp_err_to_name(err));
            nvs_close(pc_handle);
            return;
        }

        err = nvs_set_i32(pc_handle, "vbatt_gain", adc_gain_i32);
        if (err != ESP_OK) {
            printf("Error (%s) failed to set hs_sense_gain!",
                   esp_err_to_name(err));
            nvs_close(pc_handle);
            return;
        }

        err = nvs_set_i32(pc_handle, "vbatt_offset", adc_offset_i32);
        if (err != ESP_OK) {
            printf("Error (%s) failed to set hs_sense_offset!",
                   esp_err_to_name(err));
            nvs_close(pc_handle);
            return;
        }

        err = nvs_commit(pc_handle);
        if (err != ESP_OK) {
            printf("Error (%s) failed to commit NVS!", esp_err_to_name(err));
            nvs_close(pc_handle);
            return;
        }

        nvs_close(pc_handle);
        printf("Commit to flash successfully.\n");
    } else {
        printf("Cancelled.\n");
    }
}
