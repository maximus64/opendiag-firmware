/* SPDX-License-Identifier: GPL-3.0-only */
#include "driver_rtos.h"
#include "fake_clock.h"
#include "kline.c"
#include "td_test.h"

static bool uart_live;
static QueueHandle_t uart_queue;
static esp_err_t install_result, config_result, delete_result;
static int uart_deletes, silent;
static int l_line_level;
static struct {
    int pin;
    uint32_t level;
    int64_t time;
} edges[64];
static unsigned edge_count;
static bool capture_edges;

esp_err_t gpio_set_level(gpio_num_t pin, uint32_t value) {
    if (pin == PIN_KLINE_nSILENT)
        silent = value;
    if (pin == PIN_LS_OBD_15) {
        l_line_level = value;
    }
    if (capture_edges && (pin == PIN_KLINE_TX || pin == PIN_LS_OBD_15)) {
        TEST_ASSERT_TRUE(edge_count < sizeof(edges) / sizeof(edges[0]));
        edges[edge_count].pin = pin;
        edges[edge_count].level = value;
        edges[edge_count].time = esp_timer_get_time();
        edge_count++;
    }
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
    fake_clock_reset();
    edge_count = 0;
    capture_edges = false;
    l_line_level = 0;
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
    TEST_ASSERT_EQUAL_INT(0, l_line_level);
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

TEST(slow_init_mirrors_address_on_l_line_then_releases_it) {
    cfg_defaults(&g.cfg);
    g.cfg.k_line_only = false;
    capture_edges = true;
    send_5baud_address(0x33);
    capture_edges = false;
    TEST_ASSERT_EQUAL_INT(21, edge_count);
    const unsigned expected[] = {0, 1, 1, 0, 0, 1, 1, 0, 0, 1};
    for (unsigned bit = 0; bit < 10; bit++) {
        unsigned k = 1 + bit * 2;
        TEST_ASSERT_EQUAL_INT(PIN_KLINE_TX, edges[k].pin);
        TEST_ASSERT_EQUAL_INT(expected[bit], edges[k].level);
        TEST_ASSERT_EQUAL_INT(PIN_LS_OBD_15, edges[k + 1].pin);
        TEST_ASSERT_EQUAL_INT(!expected[bit], edges[k + 1].level);
        TEST_ASSERT_TRUE(edges[k + 1].time - edges[k].time <= 2);
        if (bit) {
            int64_t duration = edges[k].time - edges[k - 2].time;
            TEST_ASSERT_TRUE(duration >= 200000 && duration <= 200010);
        }
    }
    TEST_ASSERT_EQUAL_INT(0, l_line_level);
}

TEST(k_only_initialization_never_asserts_l_line) {
    cfg_defaults(&g.cfg);
    capture_edges = true;
    send_5baud_address(0x33);
    bus_init_t init = {0};
    TEST_ASSERT_EQUAL_INT(0, init_fast(&init));
    capture_edges = false;
    for (unsigned index = 0; index < edge_count; index++) {
        TEST_ASSERT_EQUAL_INT(PIN_KLINE_TX, edges[index].pin);
    }
    TEST_ASSERT_EQUAL_INT(0, l_line_level);
}

TEST(fast_init_mirrors_only_the_wakeup_pulse_on_l_line) {
    cfg_defaults(&g.cfg);
    g.cfg.k_line_only = false;
    bus_init_t init = {0};
    capture_edges = true;
    TEST_ASSERT_EQUAL_INT(0, init_fast(&init));
    capture_edges = false;
    TEST_ASSERT_EQUAL_INT(5, edge_count);
    TEST_ASSERT_EQUAL_INT(PIN_LS_OBD_15, edges[2].pin);
    TEST_ASSERT_EQUAL_INT(1, edges[2].level);
    TEST_ASSERT_EQUAL_INT(PIN_LS_OBD_15, edges[4].pin);
    TEST_ASSERT_EQUAL_INT(0, edges[4].level);
    int64_t pulse = edges[4].time - edges[2].time;
    TEST_ASSERT_TRUE(pulse >= 24990 && pulse <= 25010);
    TEST_ASSERT_EQUAL_INT(0, l_line_level);
}

TEST(l_line_configuration_resets_and_teardown_releases_driver) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    uint32_t value = 0;
    TEST_ASSERT_EQUAL_INT(0, kline_get_param(BUS_P_K_LINE_ONLY, &value));
    TEST_ASSERT_EQUAL_INT(1, value);
    TEST_ASSERT_EQUAL_INT(0, kline_set_param(BUS_P_K_LINE_ONLY, 0));
    TEST_ASSERT_EQUAL_INT(0, kline_get_param(BUS_P_K_LINE_ONLY, &value));
    TEST_ASSERT_EQUAL_INT(0, value);
    gpio_set_level(PIN_LS_OBD_15, 1);
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_close());
    TEST_ASSERT_EQUAL_INT(0, l_line_level);
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    TEST_ASSERT_EQUAL_INT(0, kline_get_param(BUS_P_K_LINE_ONLY, &value));
    TEST_ASSERT_EQUAL_INT(1, value);
}

TEST(k_only_open_and_close_do_not_touch_pin_15) {
    l_line_level = 1;
    capture_edges = true;
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    TEST_ASSERT_EQUAL_INT(0, kline_set_param(BUS_P_K_LINE_ONLY, 1));
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_close());
    capture_edges = false;
    TEST_ASSERT_EQUAL_INT(1, l_line_level);
    for (unsigned index = 0; index < edge_count; index++) {
        TEST_ASSERT_TRUE(edges[index].pin != PIN_LS_OBD_15);
    }
}

TEST(disabling_l_line_releases_it_before_k_only_cleanup) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    TEST_ASSERT_EQUAL_INT(0, kline_set_param(BUS_P_K_LINE_ONLY, 0));
    gpio_set_level(PIN_LS_OBD_15, 1);
    TEST_ASSERT_EQUAL_INT(0, kline_set_param(BUS_P_K_LINE_ONLY, 1));
    TEST_ASSERT_EQUAL_INT(0, l_line_level);

    /* Another owner may ground pin 15 after the mode change. */
    gpio_set_level(PIN_LS_OBD_15, 1);
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_close());
    TEST_ASSERT_EQUAL_INT(1, l_line_level);
}

TEST(pending_initialization_prevents_releasing_l_line_after_timeout) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    TEST_ASSERT_EQUAL_INT(0, kline_set_param(BUS_P_K_LINE_ONLY, 0));
    TEST_ASSERT_EQUAL_INT(BUS_ERR_TIMEOUT, run_cmd(CMD_FIVE_BAUD, 0));
    TEST_ASSERT_EQUAL_INT(1, g.pending_inits);
    gpio_set_level(PIN_LS_OBD_15, 1);
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BUS_BUSY,
                          kline_set_param(BUS_P_K_LINE_ONLY, 1));
    TEST_ASSERT_FALSE(g.cfg.k_line_only);
    TEST_ASSERT_EQUAL_INT(1, l_line_level);
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_close());
    TEST_ASSERT_EQUAL_INT(0, l_line_level);
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    TEST_ASSERT_EQUAL_INT(0, g.pending_inits);
}
