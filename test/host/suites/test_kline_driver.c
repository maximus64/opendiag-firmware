/* SPDX-License-Identifier: GPL-3.0-only */
#include "driver_rtos.h"
#include "kline.c"
#include "td_test.h"

static bool uart_live;
static QueueHandle_t uart_queue;
static esp_err_t install_result, config_result, delete_result;
static int uart_deletes, silent;

esp_err_t gpio_set_level(gpio_num_t pin, uint32_t value) {
    if (pin == PIN_KLINE_nSILENT)
        silent = value;
    return ESP_OK;
}
esp_err_t uart_driver_install(uart_port_t port, int rx, int tx, int queue_size,
                              QueueHandle_t *queue, int flags) {
    if (install_result != ESP_OK)
        return install_result;
    TEST_ASSERT_FALSE(uart_live);
    uart_queue = xQueueCreate(queue_size, sizeof(uart_event_t));
    if (!uart_queue)
        return ESP_ERR_NO_MEM;
    *queue = uart_queue;
    uart_live = true;
    return ESP_OK;
}
esp_err_t uart_driver_delete(uart_port_t port) {
    TEST_ASSERT_TRUE(uart_live);
    uart_deletes++;
    if (delete_result != ESP_OK)
        return delete_result;
    TEST_ASSERT_EQUAL_INT(driver_task_starts, driver_task_exits);
    vQueueDelete(uart_queue);
    uart_queue = NULL;
    uart_live = false;
    return ESP_OK;
}
esp_err_t uart_param_config(uart_port_t port, const uart_config_t *cfg) {
    return config_result;
}
esp_err_t uart_set_pin(uart_port_t port, int tx, int rx, int rts, int cts) {
    return ESP_OK;
}
esp_err_t uart_set_rx_timeout(uart_port_t port, uint8_t threshold) {
    return ESP_OK;
}
esp_err_t uart_set_baudrate(uart_port_t port, uint32_t baud) { return ESP_OK; }
esp_err_t uart_set_word_length(uart_port_t port, uart_word_length_t bits) {
    return ESP_OK;
}
esp_err_t uart_set_parity(uart_port_t port, uart_parity_t parity) {
    return ESP_OK;
}
esp_err_t uart_flush_input(uart_port_t port) { return ESP_OK; }
int uart_read_bytes(uart_port_t port, void *buf, uint32_t length,
                    TickType_t wait) {
    return 0;
}
int uart_write_bytes(uart_port_t port, const void *buf, size_t length) {
    return length;
}
esp_err_t uart_wait_tx_done(uart_port_t port, TickType_t wait) {
    return ESP_OK;
}

void td_setup(void) {
    driver_rtos_reset();
    memset(&g, 0, sizeof(g));
    install_result = config_result = delete_result = ESP_OK;
    uart_deletes = 0;
    TEST_ASSERT_FALSE(uart_live);
    driver_exit_on_wait = true;
}
void td_teardown(void) {
    delete_result = ESP_OK;
    driver_exit_on_wait = true;
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_close());
    if (g.api_lock)
        vSemaphoreDelete(g.api_lock);
    if (g.data_lock)
        vSemaphoreDelete(g.data_lock);
    memset(&g, 0, sizeof(g));
    TEST_ASSERT_EQUAL_INT(0, driver_live_objects);
    TEST_ASSERT_FALSE(uart_live);
}

static void assert_released(void) {
    TEST_ASSERT_FALSE(uart_live);
    TEST_ASSERT_FALSE(g.started);
    TEST_ASSERT_NULL(g.task);
    TEST_ASSERT_NULL(g.uart_q);
    TEST_ASSERT_NULL(g.cmd_q);
    TEST_ASSERT_NULL(g.rx_q);
    TEST_ASSERT_NULL(g.set);
    TEST_ASSERT_NULL(g.cmd_done);
    TEST_ASSERT_NULL(g.exited);
    TEST_ASSERT_EQUAL_INT(2,
                          driver_live_objects); /* Process-lifetime mutexes. */
    TEST_ASSERT_EQUAL_INT(0, silent);
}

TEST(kline_worker_creation_failure_does_not_leak_across_retries) {
    driver_fail_task = true;
    for (int i = 0; i < 20; i++) {
        TEST_ASSERT_EQUAL_INT(ESP_ERR_NO_MEM, kline_open(NULL));
        assert_released();
        TEST_ASSERT_EQUAL_INT(ESP_OK, kline_close());
    }
    driver_fail_task = false;
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_close());
    assert_released();
}

TEST(kline_failed_startup_cleanup_can_be_retried_without_worker) {
    driver_fail_task = true;
    delete_result = ESP_FAIL;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_NO_MEM, kline_open(NULL));
    TEST_ASSERT_TRUE(uart_live);
    TEST_ASSERT_NULL(g.task);
    int live = driver_live_objects;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, kline_open(NULL));
    TEST_ASSERT_EQUAL_INT(live, driver_live_objects);
    TEST_ASSERT_EQUAL_INT(ESP_FAIL, kline_close());
    TEST_ASSERT_EQUAL_INT(live, driver_live_objects);
    delete_result = ESP_OK;
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_close());
    assert_released();
    driver_fail_task = false;
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
}

TEST(kline_worker_timeout_retains_resources_until_late_acknowledgement) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    driver_exit_on_wait = false;
    int live = driver_live_objects;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT, kline_close());
    TEST_ASSERT_EQUAL_INT(0, uart_deletes);
    TEST_ASSERT_EQUAL_INT(live, driver_live_objects);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, kline_open(NULL));
    driver_run_worker();
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_close());
    assert_released();
}

TEST(kline_uart_delete_failure_after_worker_exit_can_be_retried) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    delete_result = ESP_FAIL;
    TEST_ASSERT_EQUAL_INT(ESP_FAIL, kline_close());
    TEST_ASSERT_NULL(g.task);
    TEST_ASSERT_TRUE(uart_live);
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, kline_open(NULL));
    delete_result = ESP_OK;
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_close());
    assert_released();
}

TEST(kline_each_allocation_failure_allows_a_later_successful_open) {
    for (int allocation = 1; allocation <= 8; allocation++) {
        driver_fail_allocation = driver_allocations + allocation;
        TEST_ASSERT_EQUAL_INT(ESP_ERR_NO_MEM, kline_open(NULL));
        TEST_ASSERT_FALSE(uart_live);
        TEST_ASSERT_NULL(g.task);
        driver_fail_allocation = 0;
        TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
        TEST_ASSERT_EQUAL_INT(ESP_OK, kline_close());
        assert_released();
        vSemaphoreDelete(g.api_lock);
        vSemaphoreDelete(g.data_lock);
        g.api_lock = g.data_lock = NULL;
    }
}

TEST(kline_install_failure_does_not_delete_an_unowned_uart) {
    install_result = ESP_ERR_INVALID_STATE;
    TEST_ASSERT_EQUAL_INT(ESP_ERR_INVALID_STATE, kline_open(NULL));
    TEST_ASSERT_EQUAL_INT(0, uart_deletes);
    assert_released();
    install_result = ESP_OK;
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
}

TEST(kline_uart_configuration_failure_releases_partial_startup) {
    config_result = ESP_FAIL;
    TEST_ASSERT_EQUAL_INT(ESP_FAIL, kline_open(NULL));
    assert_released();
    config_result = ESP_OK;
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
}

TEST(kline_queue_set_failure_releases_partial_startup) {
    for (int join = 1; join <= 2; join++) {
        driver_fail_set_join = driver_set_joins + join;
        TEST_ASSERT_EQUAL_INT(ESP_FAIL, kline_open(NULL));
        assert_released();
    }
    driver_fail_set_join = 0;
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
}
