/* SPDX-License-Identifier: GPL-3.0-only */
#include <string.h>
#include "esp_twai_onchip.h"
#include "can_bus.h"
#include "driver/gpio.h"
#include "driver_rtos.h"
#include "pinout.h"
#include "td_test.h"

enum operation { NEW, REGISTER, ENABLE, GET_INFO, DISABLE, DELETE, OP_COUNT };
static esp_err_t result[OP_COUNT];
static int calls[OP_COUNT];
static bool node_live, node_enabled;
static bool bus_off_on_disable;
static int silent;
static twai_event_callbacks_t callbacks;
static int node_token;
#define NODE ((twai_node_handle_t) & node_token)
#define CALL(op)                                                               \
    do {                                                                       \
        calls[op]++;                                                           \
        if (result[op])                                                        \
            return result[op];                                                 \
    } while (0)

esp_err_t gpio_set_level(gpio_num_t pin, uint32_t value) {
    if (pin == PIN_CAN0_SILENT)
        silent = value;
    return ESP_OK;
}
esp_err_t twai_new_node_onchip(const twai_onchip_node_config_t *cfg,
                               twai_node_handle_t *node) {
    CALL(NEW);
    TEST_ASSERT_FALSE(node_live);
    node_live = true;
    *node = NODE;
    return ESP_OK;
}
esp_err_t twai_node_register_event_callbacks(twai_node_handle_t node,
                                             const twai_event_callbacks_t *cb,
                                             void *arg) {
    CALL(REGISTER);
    callbacks = *cb;
    return ESP_OK;
}
esp_err_t twai_node_enable(twai_node_handle_t node) {
    CALL(ENABLE);
    node_enabled = true;
    return ESP_OK;
}
esp_err_t twai_node_disable(twai_node_handle_t node) {
    CALL(DISABLE);
    TEST_ASSERT_TRUE(node_live);
    if (bus_off_on_disable)
        node_enabled = false;
    if (!node_enabled)
        return ESP_ERR_INVALID_STATE;
    node_enabled = false;
    return ESP_OK;
}
esp_err_t twai_node_delete(twai_node_handle_t node) {
    CALL(DELETE);
    TEST_ASSERT_TRUE(node_live);
    TEST_ASSERT_FALSE(node_enabled);
    TEST_ASSERT_EQUAL_INT(driver_task_starts, driver_task_exits);
    node_live = false;
    return ESP_OK;
}
esp_err_t twai_node_recover(twai_node_handle_t node) { return ESP_OK; }
esp_err_t twai_node_transmit(twai_node_handle_t node, const twai_frame_t *frame,
                             int ms) {
    TEST_ASSERT_TRUE(node_live && node_enabled);
    return ESP_OK;
}
esp_err_t twai_node_receive_from_isr(twai_node_handle_t node,
                                     twai_frame_t *frame) {
    frame->header.id = 0x123;
    frame->header.dlc = 1;
    frame->buffer[0] = 0x55;
    return ESP_OK;
}
esp_err_t twai_node_get_info(twai_node_handle_t node,
                             twai_node_status_t *status,
                             twai_node_record_t *record) {
    CALL(GET_INFO);
    if (status) {
        memset(status, 0, sizeof(*status));
        status->state = node_enabled ? TWAI_ERROR_ACTIVE : TWAI_ERROR_BUS_OFF;
    }
    if (record)
        memset(record, 0, sizeof(*record));
    return ESP_OK;
}

void td_setup(void) {
    driver_rtos_reset();
    memset(result, 0, sizeof(result));
    memset(calls, 0, sizeof(calls));
    memset(&callbacks, 0, sizeof(callbacks));
    TEST_ASSERT_FALSE(node_live);
    bus_off_on_disable = false;
    driver_exit_on_wait = true;
}
void td_teardown(void) {
    memset(result, 0, sizeof(result));
    driver_exit_on_wait = true;
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_teardown());
    TEST_ASSERT_EQUAL_INT(0, driver_live_objects);
    TEST_ASSERT_FALSE(node_live);
}

static void assert_io_stopped(void) {
    struct can_frame frame = {.id = 0x123, .dlc = 1};
    TEST_ASSERT_EQUAL_INT(-1, can_send(&frame));
    TEST_ASSERT_EQUAL_INT(-1, can_receive(&frame, 0));
    TEST_ASSERT_EQUAL_INT(1, silent);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, can_bus_setup(500000));
}

TEST(can_worker_timeout_retains_resources_until_late_acknowledgement) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
    twai_state_change_event_data_t state = {.new_sta = TWAI_ERROR_BUS_OFF};
    callbacks.on_state_change(NODE, &state, NULL);
    TEST_ASSERT_EQUAL_INT(1, driver_notifications);
    driver_exit_on_wait = false;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT, can_bus_teardown());
    TEST_ASSERT_EQUAL_INT(0, calls[DISABLE]);
    TEST_ASSERT_EQUAL_INT(0, calls[DELETE]);
    TEST_ASSERT_EQUAL_INT(2, driver_live_objects);
    TEST_ASSERT_EQUAL_INT(0, driver_task_exits);
    assert_io_stopped();

    callbacks.on_state_change(NODE, &state, NULL);
    TEST_ASSERT_EQUAL_INT(1, driver_notifications);
    driver_run_worker();
    callbacks.on_state_change(NODE, &state, NULL);
    callbacks.on_rx_done(NODE, NULL, NULL);
    TEST_ASSERT_EQUAL_INT(1, driver_notifications);
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_teardown());
    TEST_ASSERT_EQUAL_INT(0, driver_live_objects);
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
}

TEST(can_peripheral_failures_retain_queues_and_allow_cleanup_retry) {
    const int failures[] = {GET_INFO, DISABLE, DELETE};
    for (size_t i = 0; i < sizeof(failures) / sizeof(failures[0]); i++) {
        TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
        int fail = failures[i];
        result[fail] = ESP_FAIL;
        int deletes = calls[DELETE];
        int disables = calls[DISABLE];
        TEST_ASSERT_EQUAL_INT(ESP_FAIL, can_bus_ops.close());
        TEST_ASSERT_TRUE(node_live);
        TEST_ASSERT_EQUAL_INT(2, driver_live_objects);
        assert_io_stopped();
        if (fail != DELETE)
            TEST_ASSERT_EQUAL_INT(deletes, calls[DELETE]);
        callbacks.on_rx_done(NODE, NULL, NULL);
        twai_state_change_event_data_t state = {.new_sta = TWAI_ERROR_BUS_OFF};
        callbacks.on_state_change(NODE, &state, NULL);
        result[fail] = ESP_OK;
        TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_teardown());
        TEST_ASSERT_EQUAL_INT(disables + (fail == DISABLE ? 2 : 1),
                              calls[DISABLE]);
        TEST_ASSERT_EQUAL_INT(0, driver_live_objects);
    }
}

TEST(can_bus_off_node_can_be_deleted) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
    node_enabled = false;
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_teardown());
    TEST_ASSERT_EQUAL_INT(0, calls[DISABLE]);
    TEST_ASSERT_FALSE(node_live);
}

TEST(can_bus_off_between_status_and_disable_can_be_deleted) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
    bus_off_on_disable = true;
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_teardown());
    TEST_ASSERT_EQUAL_INT(1, calls[DISABLE]);
    TEST_ASSERT_FALSE(node_live);
}

TEST(can_failed_startup_waits_for_worker_before_releasing_node) {
    result[ENABLE] = ESP_FAIL;
    driver_exit_on_wait = false;
    TEST_ASSERT_EQUAL_INT(ESP_FAIL, can_bus_setup(500000));
    TEST_ASSERT_EQUAL_INT(0, calls[DELETE]);
    TEST_ASSERT_TRUE(node_live);
    assert_io_stopped();
    result[ENABLE] = ESP_OK;
    driver_exit_on_wait = true;
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_teardown());
    TEST_ASSERT_EQUAL_INT(0, calls[DISABLE]);
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
}

TEST(can_partial_startup_failure_releases_all_owned_resources) {
    const int failures[] = {NEW, REGISTER, ENABLE};
    for (size_t i = 0; i < sizeof(failures) / sizeof(failures[0]); i++) {
        result[failures[i]] = ESP_FAIL;
        TEST_ASSERT_EQUAL_INT(ESP_FAIL, can_bus_setup(500000));
        TEST_ASSERT_EQUAL_INT(0, driver_live_objects);
        TEST_ASSERT_FALSE(node_live);
        result[failures[i]] = ESP_OK;
    }
    driver_fail_task = true;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_NO_MEM, can_bus_setup(500000));
    TEST_ASSERT_EQUAL_INT(0, driver_live_objects);
    TEST_ASSERT_FALSE(node_live);
    driver_fail_task = false;
    TEST_ASSERT_EQUAL_INT(0, calls[DISABLE]);
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
}

TEST(can_partial_allocation_failure_is_recoverable) {
    for (int allocation = 1; allocation <= 2; allocation++) {
        driver_fail_allocation = driver_allocations + allocation;
        TEST_ASSERT_EQUAL_INT(ESP_ERR_NO_MEM, can_bus_setup(500000));
        TEST_ASSERT_EQUAL_INT(0, driver_live_objects);
        TEST_ASSERT_FALSE(node_live);
    }
    driver_fail_allocation = 0;
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
}
