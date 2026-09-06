/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "esp_twai_onchip.h"
#include "can_bus.h"
#include "driver/gpio.h"
#include "driver_rtos.h"
#include "pinout.h"
#include "td_test.h"

enum operation {
    NEW,
    REGISTER,
    ENABLE,
    GET_INFO,
    DISABLE,
    DELETE,
    TRANSMIT,
    WAIT_TX,
    OP_COUNT
};
static esp_err_t result[OP_COUNT];
static int calls[OP_COUNT];
static bool node_live, node_enabled;
static bool bus_off_on_disable;
static bool tx_unsuccessful;
static bool complete_on_poll;
static int silent;
static twai_event_callbacks_t callbacks;
static int node_token;
static const twai_frame_t *pending_tx[8];
static unsigned pending_count, wire_count;
static uint32_t last_wire_id;
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
    pending_count = 0;
    return ESP_OK;
}
esp_err_t twai_node_recover(twai_node_handle_t node) { return ESP_OK; }
esp_err_t twai_node_transmit(twai_node_handle_t node, const twai_frame_t *frame,
                             int ms) {
    CALL(TRANSMIT);
    TEST_ASSERT_TRUE(node_live && node_enabled);
    TEST_ASSERT_TRUE(pending_count < 8);
    pending_tx[pending_count++] = frame;
    return ESP_OK;
}
static void complete_pending_tx(void) {
    if (!node_live || !node_enabled || silent)
        return;
    for (unsigned i = 0; i < pending_count; i++) {
        if (!tx_unsuccessful) {
            wire_count++;
            last_wire_id = pending_tx[i]->header.id;
        }
        twai_tx_done_event_data_t event = {.is_tx_success = !tx_unsuccessful};
        callbacks.on_tx_done(NODE, &event, NULL);
    }
    pending_count = 0;
}
esp_err_t twai_node_transmit_wait_all_done(twai_node_handle_t node, int ms) {
    calls[WAIT_TX]++;
    if (result[WAIT_TX] && !(ms == 0 && complete_on_poll))
        return result[WAIT_TX];
    TEST_ASSERT_TRUE(node_live && node_enabled);
    complete_pending_tx();
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
    tx_unsuccessful = false;
    complete_on_poll = false;
    TEST_ASSERT_EQUAL_INT(0, pending_count);
    wire_count = 0;
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

TEST(confirmed_can_send_waits_for_wire_completion_and_reports_failure) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
    uint8_t data[] = {0x30, 0, 0};
    bus_msg_t msg;
    bus_msg_tx(&msg, data, sizeof(data), 0x7e0);
    TEST_ASSERT_EQUAL_INT(0, can_bus_ops.send(&msg, 0));
    TEST_ASSERT_EQUAL_INT(0, calls[WAIT_TX]);
    TEST_ASSERT_EQUAL_INT(0, can_bus_ops.send(&msg, BUS_TX_WAIT_DONE));
    TEST_ASSERT_EQUAL_INT(1, calls[WAIT_TX]);
    tx_unsuccessful = true;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_TX_FAILED,
                          can_bus_ops.send(&msg, BUS_TX_WAIT_DONE));
    TEST_ASSERT_TRUE(node_live && node_enabled);
    TEST_ASSERT_EQUAL_INT(0, silent);
    TEST_ASSERT_EQUAL_INT(0, calls[DISABLE]);
    TEST_ASSERT_EQUAL_INT(0, calls[DELETE]);
    tx_unsuccessful = false;
    TEST_ASSERT_EQUAL_INT(0, can_bus_ops.send(&msg, BUS_TX_WAIT_DONE));
    TEST_ASSERT_EQUAL_INT(1, calls[NEW]);
    result[WAIT_TX] = ESP_ERR_TIMEOUT;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_TX_ABORTED,
                          can_bus_ops.send(&msg, BUS_TX_WAIT_DONE));
    TEST_ASSERT_FALSE(node_live);
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
    result[WAIT_TX] = ESP_ERR_INVALID_STATE;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_TX_ABORTED,
                          can_bus_ops.send(&msg, BUS_TX_WAIT_DONE));
    TEST_ASSERT_FALSE(node_live);
}

TEST(confirmed_send_timeout_discards_frames_before_reopen) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
    uint8_t data[] = {0x30, 0, 0};
    bus_msg_t msg;
    bus_msg_tx(&msg, data, sizeof(data), 0x7e0);
    TEST_ASSERT_EQUAL_INT(0, can_bus_ops.send(&msg, 0));
    result[WAIT_TX] = ESP_ERR_TIMEOUT;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_TX_ABORTED,
                          can_bus_ops.send(&msg, BUS_TX_WAIT_DONE));
    TEST_ASSERT_EQUAL_INT(0, pending_count);
    TEST_ASSERT_FALSE(node_live);
    complete_pending_tx();
    TEST_ASSERT_EQUAL_INT(0, wire_count);

    result[WAIT_TX] = ESP_OK;
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
    complete_pending_tx();
    TEST_ASSERT_EQUAL_INT(0, wire_count);
    msg.id = 0x7e1;
    TEST_ASSERT_EQUAL_INT(0, can_bus_ops.send(&msg, BUS_TX_WAIT_DONE));
    TEST_ASSERT_EQUAL_INT(1, wire_count);
    TEST_ASSERT_EQUAL_HEX32(0x7e1, last_wire_id);
}

TEST(confirmed_send_enqueue_failure_keeps_can_and_older_frames_running) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
    uint8_t data[] = {0x30, 0, 0};
    bus_msg_t msg;
    bus_msg_tx(&msg, data, sizeof(data), 0x7e0);
    TEST_ASSERT_EQUAL_INT(0, can_bus_ops.send(&msg, 0));
    msg.id = 0x7e1;
    result[TRANSMIT] = ESP_ERR_TIMEOUT;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_TX_FAILED,
                          can_bus_ops.send(&msg, BUS_TX_WAIT_DONE));
    TEST_ASSERT_EQUAL_INT(1, pending_count);
    TEST_ASSERT_TRUE(node_live && node_enabled);
    TEST_ASSERT_EQUAL_INT(0, silent);
    TEST_ASSERT_EQUAL_INT(0, calls[WAIT_TX]);
    TEST_ASSERT_EQUAL_INT(0, calls[DISABLE]);
    TEST_ASSERT_EQUAL_INT(0, calls[DELETE]);
    complete_pending_tx();
    TEST_ASSERT_EQUAL_INT(1, wire_count);
    TEST_ASSERT_EQUAL_HEX32(0x7e0, last_wire_id);
    result[TRANSMIT] = ESP_OK;
    TEST_ASSERT_EQUAL_INT(0, can_bus_ops.send(&msg, BUS_TX_WAIT_DONE));
    TEST_ASSERT_EQUAL_INT(2, wire_count);
    TEST_ASSERT_EQUAL_HEX32(0x7e1, last_wire_id);
    TEST_ASSERT_EQUAL_INT(1, calls[NEW]);
}

TEST(completion_racing_the_timeout_keeps_can_open) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
    uint8_t data[] = {0x30, 0, 0};
    bus_msg_t msg;
    bus_msg_tx(&msg, data, sizeof(data), 0x7e0);
    result[WAIT_TX] = ESP_ERR_TIMEOUT;
    complete_on_poll = true;
    TEST_ASSERT_EQUAL_INT(0, can_bus_ops.send(&msg, BUS_TX_WAIT_DONE));
    TEST_ASSERT_EQUAL_INT(1, wire_count);
    tx_unsuccessful = true;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_TX_FAILED,
                          can_bus_ops.send(&msg, BUS_TX_WAIT_DONE));
    TEST_ASSERT_EQUAL_INT(4, calls[WAIT_TX]);
    TEST_ASSERT_EQUAL_INT(0, pending_count);
    TEST_ASSERT_TRUE(node_live && node_enabled);
    TEST_ASSERT_EQUAL_INT(0, silent);
    TEST_ASSERT_EQUAL_INT(0, calls[DISABLE]);
    TEST_ASSERT_EQUAL_INT(0, calls[DELETE]);
}

TEST(failed_tx_abort_keeps_the_transceiver_silent_until_cleanup_succeeds) {
    const int failures[] = {GET_INFO, DISABLE, DELETE, -1};
    for (unsigned i = 0; i < sizeof(failures) / sizeof(failures[0]); i++) {
        TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
        uint8_t data[] = {0x30, 0, 0};
        bus_msg_t msg;
        bus_msg_tx(&msg, data, sizeof(data), 0x7e0);
        result[WAIT_TX] = ESP_ERR_TIMEOUT;
        if (failures[i] < 0)
            driver_exit_on_wait = false;
        else
            result[failures[i]] = ESP_FAIL;
        TEST_ASSERT_EQUAL_INT(BUS_ERR_TX_ABORTED,
                              can_bus_ops.send(&msg, BUS_TX_WAIT_DONE));
        TEST_ASSERT_TRUE(node_live);
        assert_io_stopped();
        node_enabled = true;
        complete_pending_tx();
        TEST_ASSERT_EQUAL_INT(0, wire_count);
        memset(result, 0, sizeof(result));
        driver_exit_on_wait = true;
        TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_teardown());
        TEST_ASSERT_EQUAL_INT(0, pending_count);
    }
}

static const char *printed_stats(void) {
    static char text[2048];
    FILE *capture = tmpfile();
    TEST_ASSERT_NOT_NULL(capture);
    fflush(stdout);
    int saved = dup(STDOUT_FILENO);
    TEST_ASSERT_TRUE(saved >= 0);
    TEST_ASSERT_TRUE(dup2(fileno(capture), STDOUT_FILENO) >= 0);
    can_print_stat();
    fflush(stdout);
    int restored = dup2(saved, STDOUT_FILENO);
    close(saved);
    rewind(capture);
    size_t length = fread(text, 1, sizeof(text) - 1, capture);
    text[length] = 0;
    fclose(capture);
    TEST_ASSERT_TRUE(restored >= 0);
    return text;
}

TEST(arbitration_losses_are_not_counted_as_bus_errors) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
    twai_error_event_data_t event = {.err_flags = {.arb_lost = 1}};
    for (unsigned i = 0; i < 9; i++)
        callbacks.on_error(NODE, &event, NULL);
    const char *stats = printed_stats();
    TEST_ASSERT_NOT_NULL(strstr(stats, "bus_error_count: 0\n"));
    TEST_ASSERT_NOT_NULL(strstr(stats, "arb_lost_count: 9\n"));
    TEST_ASSERT_NOT_NULL(strstr(stats, "unclassified_error_count: 0\n"));
}

TEST(real_errors_are_counted_even_when_arbitration_is_also_reported) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
    const twai_error_flags_t flags[] = {
        {.bit_err = 1},
        {.form_err = 1},
        {.stuff_err = 1},
        {.ack_err = 1},
        {.arb_lost = 1, .bit_err = 1},
        {.val = 0},
        {.val = 1u << 8},
    };
    for (unsigned i = 0; i < sizeof(flags) / sizeof(flags[0]); i++) {
        twai_error_event_data_t event = {.err_flags = flags[i]};
        callbacks.on_error(NODE, &event, NULL);
    }
    const char *stats = printed_stats();
    TEST_ASSERT_NOT_NULL(strstr(stats, "bus_error_count: 7\n"));
    TEST_ASSERT_NOT_NULL(strstr(stats, "arb_lost_count: 1\n"));
    TEST_ASSERT_NOT_NULL(strstr(stats, "bit_error_count: 2\n"));
    TEST_ASSERT_NOT_NULL(strstr(stats, "form_error_count: 1\n"));
    TEST_ASSERT_NOT_NULL(strstr(stats, "stuff_error_count: 1\n"));
    TEST_ASSERT_NOT_NULL(strstr(stats, "ack_error_count: 1\n"));
    TEST_ASSERT_NOT_NULL(strstr(stats, "unclassified_error_count: 2\n"));
}

TEST(diagnostic_counters_reset_when_a_new_can_session_opens) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
    twai_error_event_data_t event = {
        .err_flags = {.arb_lost = 1, .ack_err = 1}};
    callbacks.on_error(NODE, &event, NULL);
    twai_tx_done_event_data_t tx_event = {.is_tx_success = false};
    callbacks.on_tx_done(NODE, &tx_event, NULL);
    TEST_ASSERT_NOT_NULL(strstr(printed_stats(), "tx_failed_count: 1\n"));
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_teardown());
    TEST_ASSERT_EQUAL_INT(ESP_OK, can_bus_setup(500000));
    const char *stats = printed_stats();
    TEST_ASSERT_NOT_NULL(strstr(stats, "bus_error_count: 0\n"));
    TEST_ASSERT_NOT_NULL(strstr(stats, "arb_lost_count: 0\n"));
    TEST_ASSERT_NOT_NULL(strstr(stats, "ack_error_count: 0\n"));
    TEST_ASSERT_NOT_NULL(strstr(stats, "tx_failed_count: 0\n"));
}
