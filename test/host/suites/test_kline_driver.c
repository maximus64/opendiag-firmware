/* SPDX-License-Identifier: GPL-3.0-only */
#include "driver_rtos.h"
#include "fake_clock.h"
#include "kline.c"
#include "td_test.h"

static bool uart_live;
static QueueHandle_t uart_queue;
static esp_err_t install_result, config_result, delete_result, tx_done_result;
static int uart_deletes, silent;
static int l_line_level;
static struct {
    int pin;
    uint32_t level;
    int64_t time;
} edges[64];
static unsigned edge_count;
static bool capture_edges;
static uint8_t uart_rx_data[KLINE_RX_RINGBUF];
static size_t uart_rx_len;
static void (*uart_write_hook)(void);
static bool init_test_echo, init_test_corrupt_echo;
static uint8_t init_test_sent[BUS_INIT_MSG_MAX];
static size_t init_test_sent_len, init_test_request_len;
static const uint8_t *init_test_reply;
static size_t init_test_reply_len;
static int64_t init_test_first_tx_us;
static int uart_rx_threshold;
static unsigned uart_batch_delay_us;
static int64_t uart_batch_ready_us;
static bool uart_hold_all_echoes;
static bool uart_immediate_echo;
static unsigned uart_tx_count, uart_omit_echo_at;
static int64_t uart_tx_times[BUS_INIT_MSG_MAX];
static size_t uart_read_limit;
static int uart_injected_event;
static int uart_error_after_data;
static bool uart_omit_data_events;
static void (*uart_read_hook)(TickType_t wait);

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
esp_err_t uart_set_rx_full_threshold(uart_port_t port, int threshold) {
    uart_rx_threshold = threshold;
    return config_result;
}
esp_err_t uart_set_baudrate(uart_port_t port, uint32_t baud) { return ESP_OK; }
esp_err_t uart_set_word_length(uart_port_t port, uart_word_length_t bits) {
    return ESP_OK;
}
esp_err_t uart_set_parity(uart_port_t port, uart_parity_t parity) {
    return ESP_OK;
}
esp_err_t uart_flush_input(uart_port_t port) {
    uart_rx_len = 0;
    uart_batch_ready_us = 0;
    return ESP_OK;
}
int uart_read_bytes(uart_port_t port, void *buf, uint32_t length,
                    TickType_t wait) {
    if (uart_read_hook)
        uart_read_hook(wait);
    if (uart_batch_ready_us && now_us() < uart_batch_ready_us)
        return 0;
    size_t n = length < uart_rx_len ? length : uart_rx_len;
    if (uart_read_limit && n > uart_read_limit)
        n = uart_read_limit;
    memcpy(buf, uart_rx_data, n);
    uart_rx_len -= n;
    memmove(uart_rx_data, uart_rx_data + n, uart_rx_len);
    return (int)n;
}
int uart_write_bytes(uart_port_t port, const void *buf, size_t length) {
    TEST_ASSERT_EQUAL_INT(1, length);
    TEST_ASSERT_TRUE(uart_tx_count < BUS_INIT_MSG_MAX);
    uart_tx_times[uart_tx_count++] = now_us();
    if (uart_immediate_echo && uart_tx_count != uart_omit_echo_at)
        uart_rx_data[uart_rx_len++] = *(const uint8_t *)buf;
    if (init_test_echo) {
        TEST_ASSERT_EQUAL_INT(1, length);
        if (!init_test_sent_len)
            init_test_first_tx_us = now_us();
        init_test_sent[init_test_sent_len++] = *(const uint8_t *)buf;
        if (uart_tx_count != uart_omit_echo_at)
            uart_rx_data[uart_rx_len++] =
                *(const uint8_t *)buf ^ init_test_corrupt_echo;
        if (uart_hold_all_echoes)
            uart_batch_ready_us = now_us() + 1000000;
        if (init_test_sent_len == init_test_request_len) {
            if (init_test_reply_len)
                memcpy(uart_rx_data + uart_rx_len, init_test_reply,
                       init_test_reply_len);
            uart_rx_len += init_test_reply_len;
            uart_batch_ready_us = now_us() + uart_batch_delay_us;
            if (uart_injected_event) {
                uart_event_t event = {.type = uart_injected_event};
                xQueueSend(g.uart_q, &event, 0);
            }
        }
    }
    if ((uart_immediate_echo || init_test_echo) && !uart_omit_data_events) {
        uart_event_t event = {
            .type = UART_DATA,
            .size = uart_tx_count == uart_omit_echo_at ? 0 : 1,
        };
        if (init_test_echo && init_test_sent_len == init_test_request_len)
            event.size += init_test_reply_len;
        TEST_ASSERT_EQUAL_INT(pdTRUE, xQueueSend(g.uart_q, &event, 0));
    }
    if (uart_error_after_data && init_test_sent_len == init_test_request_len) {
        uart_event_t event = {.type = uart_error_after_data};
        TEST_ASSERT_EQUAL_INT(pdTRUE, xQueueSend(g.uart_q, &event, 0));
    }
    if (uart_write_hook) {
        uart_write_hook();
    }
    return length;
}
esp_err_t uart_wait_tx_done(uart_port_t port, TickType_t wait) {
    return tx_done_result;
}

void td_setup(void) {
    driver_rtos_reset();
    fake_clock_reset();
    edge_count = 0;
    capture_edges = false;
    uart_rx_len = 0;
    uart_write_hook = NULL;
    init_test_echo = init_test_corrupt_echo = false;
    init_test_sent_len = init_test_request_len = init_test_reply_len = 0;
    init_test_reply = NULL;
    uart_rx_threshold = 120;
    uart_batch_delay_us = 0;
    uart_batch_ready_us = 0;
    uart_hold_all_echoes = false;
    uart_immediate_echo = false;
    uart_tx_count = uart_omit_echo_at = 0;
    uart_read_limit = 0;
    uart_injected_event = 0;
    uart_error_after_data = 0;
    uart_omit_data_events = false;
    uart_read_hook = NULL;
    l_line_level = 0;
    memset(&g, 0, sizeof(g));
    install_result = config_result = delete_result = ESP_OK;
    tx_done_result = ESP_OK;
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

static const uint8_t previous_reply[] = {0x48, 0x6B, 0x10, 0x41, 0x00,
                                         0xBE, 0x3F, 0xB8, 0x13, 0xCC};
static const uint8_t current_reply[] = {0x48, 0x6B, 0x10, 0x41, 0x01,
                                        0x00, 0x00, 0x00, 0x00, 0x05};
static const uint8_t current_request[] = {0x68, 0x6A, 0xF1, 0x01, 0x01, 0xC5};

static void stage_uart_reply(const uint8_t *data, size_t len) {
    TEST_ASSERT_EQUAL_INT(0, uart_rx_len);
    TEST_ASSERT_TRUE(len <= sizeof(uart_rx_data));
    memcpy(uart_rx_data, data, len);
    uart_rx_len = len;
}

static void queue_previous_reply(void) {
    kline_frame_t f = {.len = sizeof(previous_reply)};
    memcpy(f.data, previous_reply, sizeof(previous_reply));
    publish(&f);
}

TEST(kline_request_boundary_discards_replies_queued_during_the_quiet_wait) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    uart_immediate_echo = true;
    queue_previous_reply();
    stage_uart_reply(previous_reply, sizeof(previous_reply));
    g.tx_answered = false;

    TEST_ASSERT_EQUAL_INT(0,
                          tx_message(current_request, sizeof(current_request),
                                     true, BUS_TX_CLEAR_RX_QUEUE));
    /* One reply was queued before the call; the UART reply was framed and
     * published by wait_before_tx(), after the call began. */
    TEST_ASSERT_EQUAL_INT(2, g.stats.rx_msgs);
    TEST_ASSERT_TRUE(g.tx_end_us >= P_TO_US(g.cfg.p3_min));
    kline_frame_t f;
    TEST_ASSERT_EQUAL_INT(pdFALSE, xQueueReceive(g.rx_q, &f, 0));
}

TEST(kline_request_boundary_discards_replies_after_an_idle_pause) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    uart_immediate_echo = true;
    queue_previous_reply();
    fake_clock_advance_ms(250);

    TEST_ASSERT_EQUAL_INT(0,
                          tx_message(current_request, sizeof(current_request),
                                     true, BUS_TX_CLEAR_RX_QUEUE));
    kline_frame_t f;
    TEST_ASSERT_EQUAL_INT(pdFALSE, xQueueReceive(g.rx_q, &f, 0));
}

TEST(kline_raw_transmit_preserves_queued_and_late_replies) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    uart_immediate_echo = true;
    queue_previous_reply();
    stage_uart_reply(previous_reply, sizeof(previous_reply));

    TEST_ASSERT_EQUAL_INT(
        0, tx_message(current_request, sizeof(current_request), true, 0));
    TEST_ASSERT_EQUAL_INT(2, g.stats.rx_msgs);
    kline_frame_t f;
    for (int i = 0; i < 2; i++) {
        TEST_ASSERT_EQUAL_INT(pdTRUE, xQueueReceive(g.rx_q, &f, 0));
        TEST_ASSERT_EQUAL_INT(sizeof(previous_reply), f.len);
        TEST_ASSERT_EQUAL_MEM(previous_reply, f.data, f.len);
    }
    TEST_ASSERT_EQUAL_INT(pdFALSE, xQueueReceive(g.rx_q, &f, 0));
}

static unsigned request_bytes_sent;

static void receive_reply_at_tx_completion(void) {
    request_bytes_sent++;
    if (request_bytes_sent == sizeof(current_request)) {
        memcpy(uart_rx_data + uart_rx_len, current_reply,
               sizeof(current_reply));
        uart_rx_len += sizeof(current_reply);
    }
}

TEST(kline_request_boundary_preserves_the_new_reply_and_tx_loopback) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    uart_immediate_echo = true;
    g.cfg.loopback = true;
    queue_previous_reply();
    request_bytes_sent = 0;
    uart_write_hook = receive_reply_at_tx_completion;

    TEST_ASSERT_EQUAL_INT(0,
                          tx_message(current_request, sizeof(current_request),
                                     true, BUS_TX_CLEAR_RX_QUEUE));
    TEST_ASSERT_EQUAL_INT(sizeof(current_request), request_bytes_sent);

    uint8_t data[KLINE_MAX_MSG];
    bus_msg_t msg;
    bus_msg_init(&msg, data, sizeof(data));
    TEST_ASSERT_EQUAL_INT(sizeof(current_request), kline_rx(&msg, 0));
    TEST_ASSERT_EQUAL_INT(BUS_RX_TX_MSG_TYPE, msg.status);
    TEST_ASSERT_EQUAL_MEM(current_request, data, msg.len);

    kline_frame_t f;
    TEST_ASSERT_EQUAL_INT(1, rx_service(now_us() + 100000, &f, true));
    publish(&f);
    TEST_ASSERT_EQUAL_INT(sizeof(current_reply), kline_rx(&msg, 0));
    TEST_ASSERT_EQUAL_INT(0, msg.status);
    TEST_ASSERT_EQUAL_MEM(current_reply, data, msg.len);
    TEST_ASSERT_EQUAL_INT(BUS_ERR_TIMEOUT, kline_rx(&msg, 0));
}

static bus_init_t raw_init(bool checksum, const uint8_t *reply,
                           size_t reply_len) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    g.cfg.frame_mode = 1;
    g.cfg.checksum_rx = g.cfg.checksum_tx = checksum;
    g.cfg.loopback = true;
    g.cfg.tidle = 30;
    g.cfg.tinil = 70;
    g.cfg.twup = 210;
    g.cfg.p4_min = 2;
    init_test_echo = true;
    init_test_reply = reply;
    init_test_reply_len = reply_len;
    bus_init_t init = {.raw_response = true, .msg_len = 6};
    /* Captured PID 00 bytes exercise checksum ownership, not an inferred
     * HDS FAST_INIT payload: XP logging did not include that payload. */
    memcpy(init.msg, "\x68\x6a\xf1\x01\x00\xc4", 6);
    if (checksum)
        init.msg_len--;
    init_test_request_len = 6;
    return init;
}

TEST(fast_init_iso9141_returns_raw_bytes_and_honors_hds_timing) {
    const uint8_t reply[] = {0x10, 0x02};
    bus_init_t init = raw_init(false, reply, sizeof(reply));
    g.cfg.k_line_only = false;
    capture_edges = true;
    TEST_ASSERT_EQUAL_INT(0, do_init(CMD_FAST_INIT, &init));
    capture_edges = false;
    TEST_ASSERT_EQUAL_INT(sizeof(reply), init.reply_len);
    TEST_ASSERT_EQUAL_MEM(reply, init.reply, sizeof(reply));
    TEST_ASSERT_TRUE(init.reply_timestamp_us > 0);
    TEST_ASSERT_EQUAL_MEM(init.msg, init_test_sent, 6);
    TEST_ASSERT_TRUE(edges[3].time - edges[1].time >= 70000);
    TEST_ASSERT_TRUE(edges[3].time - edges[1].time <= 70020);
    TEST_ASSERT_TRUE(init_test_first_tx_us - edges[1].time >= 210000);
    TEST_ASSERT_TRUE(init_test_first_tx_us - edges[1].time <= 210050);
    TEST_ASSERT_EQUAL_INT(0, l_line_level);
    kline_frame_t frame;
    TEST_ASSERT_EQUAL_INT(pdFALSE, xQueueReceive(g.rx_q, &frame, 0));
}

TEST(fast_init_automatic_checksum_is_appended_once) {
    bus_init_t init = raw_init(true, previous_reply, sizeof(previous_reply));
    TEST_ASSERT_EQUAL_INT(0, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(6, init_test_sent_len);
    TEST_ASSERT_EQUAL_INT(0xc4, init_test_sent[5]);
    TEST_ASSERT_EQUAL_MEM(previous_reply, init.reply, sizeof(previous_reply));
}

TEST(fast_init_no_response_preserves_uart_reply_for_normal_receive) {
    bus_init_t init = raw_init(false, previous_reply, sizeof(previous_reply));
    init.no_response = true;
    TEST_ASSERT_EQUAL_INT(0, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(0, init.reply_len);
    kline_frame_t frame;
    TEST_ASSERT_EQUAL_INT(1, rx_service(now_us() + 100000, &frame, true));
    TEST_ASSERT_EQUAL_MEM(previous_reply, frame.data, frame.len);
}

TEST(fast_init_kwp_preserves_a_coalesced_following_message) {
    const uint8_t replies[] = {0x81, 0xf1, 0x10, 0x50, 0xd2,
                               0x81, 0xf1, 0x10, 0x51, 0xd3};
    bus_init_t init = raw_init(false, replies, sizeof(replies));
    g.cfg.frame_mode = 2;
    TEST_ASSERT_EQUAL_INT(0, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(5, init.reply_len);
    TEST_ASSERT_EQUAL_MEM(replies, init.reply, 5);
    kline_frame_t frame;
    TEST_ASSERT_EQUAL_INT(1, rx_service(now_us() + 100000, &frame, true));
    TEST_ASSERT_EQUAL_INT(5, frame.len);
    TEST_ASSERT_EQUAL_MEM(replies + 5, frame.data, 5);
}

TEST(fast_init_echo_failure_is_not_accepted_as_initialization) {
    bus_init_t init = raw_init(false, NULL, 0);
    init_test_corrupt_echo = true;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_ECHO, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(0, g.stats.init_ok);
    TEST_ASSERT_EQUAL_INT(1, init_test_sent_len);
}

TEST(fast_init_absent_response_fails_and_oversize_request_has_no_pulse) {
    bus_init_t init = raw_init(false, NULL, 0);
    TEST_ASSERT_EQUAL_INT(BUS_ERR_INIT, do_init(CMD_FAST_INIT, &init));
    init.msg_len = BUS_INIT_MSG_MAX + 1;
    capture_edges = true;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BAD_ARG, do_init(CMD_FAST_INIT, &init));
    capture_edges = false;
    TEST_ASSERT_EQUAL_INT(0, edge_count);
    TEST_ASSERT_EQUAL_INT(0, g.stats.init_ok);
}

TEST(fast_init_accepts_current_capacity_without_byte_length_truncation) {
    uint8_t reply[BUS_INIT_MSG_MAX];
    memset(reply, 0x55, sizeof(reply));
    bus_init_t init = raw_init(false, reply, sizeof(reply));
    TEST_ASSERT_EQUAL_INT(0, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(BUS_INIT_MSG_MAX, init.reply_len);
    TEST_ASSERT_EQUAL_MEM(reply, init.reply, sizeof(reply));
    init_test_sent_len = 0;
    uart_tx_count = 0;
    init_test_reply_len = 0;
    init.no_response = true;
    init.msg_len = BUS_INIT_MSG_MAX;
    memset(init.msg, 0x55, sizeof(init.msg));
    init_test_request_len = BUS_INIT_MSG_MAX;
    TEST_ASSERT_EQUAL_INT(0, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(BUS_INIT_MSG_MAX, init_test_sent_len);
    stage_uart_reply(reply, sizeof(reply));
    kline_frame_t frame;
    TEST_ASSERT_EQUAL_INT(1, rx_service(now_us() + 100000, &frame, true));
    TEST_ASSERT_EQUAL_INT(sizeof(reply), frame.len);
}

TEST(timed_out_init_cannot_be_overwritten_by_a_later_command) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    driver_exit_on_wait = false;
    bus_init_t init = {.raw_response = true, .msg_len = 1, .msg = {0x55}};
    TEST_ASSERT_EQUAL_INT(BUS_ERR_TIMEOUT,
                          kline_ioctl(BUS_IOCTL_FAST_INIT, &init, NULL));
    init.msg[0] = 0xaa;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_NOT_READY,
                          kline_ioctl(BUS_IOCTL_FAST_INIT, &init, NULL));
    TEST_ASSERT_EQUAL_INT(0x55, g.req_init.msg[0]);
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BUS_BUSY, kline_set_param(BUS_P_TINIL, 25));
}

TEST(fast_init_one_byte_payload_with_automatic_checksum) {
    const uint8_t reply[] = {0x55, 0x55};
    bus_init_t init = raw_init(true, reply, sizeof(reply));
    TEST_ASSERT_EQUAL_INT(0, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(2, init.reply_len);
    TEST_ASSERT_EQUAL_MEM(reply, init.reply, 2);
}

TEST(kline_does_not_wait_for_another_uart_event_at_a_batch_boundary) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    g.cfg.frame_mode = BUS_KLINE_FRAME_ISO14230;
    g.cfg.checksum_rx = false;
    uint8_t replies[69] = {0xfc, 0xf1, 0x10};
    memcpy(replies + 64, "\x81\xf1\x10\x50\xd2", 5);
    stage_uart_reply(replies, sizeof(replies));
    kline_frame_t frame;
    TEST_ASSERT_EQUAL_INT(1, rx_service(now_us() + 100000, &frame, true));
    TEST_ASSERT_EQUAL_INT(64, frame.len);
    int waits = driver_queue_set_waits;
    TEST_ASSERT_EQUAL_INT(1, rx_service(now_us() + 100000, &frame, true));
    TEST_ASSERT_EQUAL_INT(5, frame.len);
    TEST_ASSERT_EQUAL_MEM(replies + 64, frame.data, 5);
    TEST_ASSERT_EQUAL_INT(waits, driver_queue_set_waits);
}

static bus_init_t hds_init(const uint8_t *reply, size_t len) {
    bus_init_t init = raw_init(false, reply, len);
    init.msg_len = 4;
    memcpy(init.msg, "\xfe\x04\x36\xc8", 4);
    init_test_request_len = 4;
    return init;
}

TEST(hds_delayed_echo_and_reply_share_one_uart_batch) {
    const uint8_t reply[] = {0x0e, 0x04, 0x36, 0xb8};
    bus_init_t init = hds_init(reply, sizeof(reply));
    uart_batch_delay_us = 6000;
    TEST_ASSERT_EQUAL_INT(0, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(sizeof(reply), init.reply_len);
    TEST_ASSERT_EQUAL_MEM(reply, init.reply, sizeof(reply));
    TEST_ASSERT_EQUAL_INT(4, g.stats.tx_echo_ok);
    TEST_ASSERT_EQUAL_INT(0, g.stats.tx_echo_missing);
}

TEST(hds_reply_can_start_with_the_final_transmitted_byte) {
    const uint8_t reply[] = {0xc8, 0x04, 0x36, 0xfe};
    bus_init_t init = hds_init(reply, sizeof(reply));
    uart_batch_delay_us = 6000;
    TEST_ASSERT_EQUAL_INT(0, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(sizeof(reply), init.reply_len);
    TEST_ASSERT_EQUAL_MEM(reply, init.reply, sizeof(reply));
}

TEST(hds_missing_echo_is_not_inferred_to_be_a_different_transceiver) {
    bus_init_t init = hds_init(NULL, 0);
    uart_omit_echo_at = 4;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_ECHO, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(0, init.reply_len);
    TEST_ASSERT_EQUAL_INT(0, g.stats.init_ok);
    TEST_ASSERT_EQUAL_INT(1, g.stats.tx_echo_missing);
}

TEST(hds_tx_pacing_does_not_wait_for_uart_batch_delivery) {
    const uint8_t reply[] = {0x0e, 0x04, 0x36, 0xb8};
    bus_init_t init = hds_init(reply, sizeof(reply));
    /* Hold every echo until after the complete request. */
    uart_hold_all_echoes = true;
    uart_batch_delay_us = 6000;
    TEST_ASSERT_EQUAL_INT(0, do_init(CMD_FAST_INIT, &init));
    for (unsigned i = 1; i < uart_tx_count; i++) {
        int64_t gap = uart_tx_times[i] - uart_tx_times[i - 1];
        TEST_ASSERT_TRUE(gap >= 1000 && gap <= 1100);
    }
    TEST_ASSERT_EQUAL_MEM(reply, init.reply, sizeof(reply));
}

TEST(hds_echo_and_response_survive_single_byte_uart_reads) {
    const uint8_t reply[] = {0x0e, 0x04, 0x36, 0xb8};
    bus_init_t init = hds_init(reply, sizeof(reply));
    uart_hold_all_echoes = true;
    uart_batch_delay_us = 6000;
    uart_read_limit = 1;
    TEST_ASSERT_EQUAL_INT(0, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(sizeof(reply), init.reply_len);
    TEST_ASSERT_EQUAL_MEM(reply, init.reply, sizeof(reply));
}

TEST(hds_full_request_echo_can_arrive_with_a_full_response) {
    uint8_t reply[BUS_INIT_MSG_MAX];
    memset(reply, 0x55, sizeof(reply));
    bus_init_t init = raw_init(false, reply, sizeof(reply));
    init.msg_len = BUS_INIT_MSG_MAX;
    memset(init.msg, 0x55, sizeof(init.msg));
    init_test_request_len = BUS_INIT_MSG_MAX;
    uart_hold_all_echoes = true;
    uart_batch_delay_us = 6000;
    TEST_ASSERT_EQUAL_INT(0, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(BUS_INIT_MSG_MAX, g.stats.tx_echo_ok);
    TEST_ASSERT_EQUAL_INT(sizeof(reply), init.reply_len);
    TEST_ASSERT_EQUAL_MEM(reply, init.reply, sizeof(reply));
}

TEST(hds_uart_loss_cannot_be_hidden_by_matching_echo_bytes) {
    const uint8_t reply[] = {0x0e, 0x04, 0x36, 0xb8};
    bus_init_t init = hds_init(reply, sizeof(reply));
    uart_injected_event = UART_BUFFER_FULL;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_ECHO, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(1, g.stats.rx_overrun);
    TEST_ASSERT_EQUAL_INT(0, init.reply_len);
    TEST_ASSERT_EQUAL_INT(0, g.stats.init_ok);
}

TEST(kline_ordinary_tx_preserves_a_reply_delivered_with_its_echo) {
    const uint8_t reply[] = {0xc8, 0x04, 0x36, 0xfe};
    bus_init_t init = hds_init(reply, sizeof(reply));
    uart_batch_delay_us = 6000;
    g.cfg.loopback = false;
    TEST_ASSERT_EQUAL_INT(0, tx_message(init.msg, init.msg_len, true, 0));
    kline_frame_t frame;
    TEST_ASSERT_EQUAL_INT(1, rx_service(now_us() + 100000, &frame, true));
    TEST_ASSERT_EQUAL_INT(sizeof(reply), frame.len);
    TEST_ASSERT_EQUAL_MEM(reply, frame.data, sizeof(reply));
}

TEST(kline_uart_completion_timeout_requires_physical_close) {
    bus_init_t init = hds_init(NULL, 0);
    tx_done_result = ESP_ERR_TIMEOUT;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_TX_FAILED, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_NULL(g.tx);
    TEST_ASSERT_EQUAL_INT(BUS_ERR_NOT_READY, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(BUS_ERR_NOT_READY,
                          tx_message(init.msg, init.msg_len, false, 0));
    TEST_ASSERT_EQUAL_INT(1, uart_tx_count);
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BUS_BUSY,
                          kline_set_param(BUS_P_DATA_RATE, 9600));
    kline_frame_t frame;
    TEST_ASSERT_FALSE(rx_drain(&frame));
    delete_result = ESP_FAIL;
    TEST_ASSERT_EQUAL_INT(ESP_FAIL, kline_close());
    TEST_ASSERT_TRUE(command_blocked());
    delete_result = ESP_OK;
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_close());
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    TEST_ASSERT_FALSE(command_blocked());
}

TEST(kline_command_during_periodic_tx_keeps_its_quiet_wait_and_reply) {
    const uint8_t reply[] = {0x0e, 0x04, 0x36, 0xb8};
    bus_init_t init = hds_init(reply, sizeof(reply));
    g.cfg.loopback = false;
    int64_t earliest_tx = g.last_bus_us + P_TO_US(g.cfg.p3_min);
    kline_cmd_t command = CMD_STOP_COMM;
    TEST_ASSERT_EQUAL_INT(pdTRUE, xQueueSend(g.cmd_q, &command, 0));
    TEST_ASSERT_EQUAL_INT(0, tx_message(init.msg, init.msg_len, true, 0));
    TEST_ASSERT_TRUE(init_test_first_tx_us >= earliest_tx);
    kline_frame_t frame;
    TEST_ASSERT_EQUAL_INT(1, rx_service(now_us() + 100000, &frame, false));
    TEST_ASSERT_EQUAL_INT(sizeof(reply), frame.len);
    TEST_ASSERT_EQUAL_MEM(reply, frame.data, sizeof(reply));
    TEST_ASSERT_EQUAL_INT(-1, rx_service(now_us() + 100000, &frame, true));
    TEST_ASSERT_EQUAL_INT(CMD_STOP_COMM, g.cmd);
}

TEST(kline_error_before_tx_is_not_attributed_to_the_new_echo) {
    const uint8_t reply[] = {0x0e, 0x04, 0x36, 0xb8};
    bus_init_t init = hds_init(reply, sizeof(reply));
    uart_event_t event = {.type = UART_FRAME_ERR};
    TEST_ASSERT_EQUAL_INT(pdTRUE, xQueueSend(g.uart_q, &event, 0));
    TEST_ASSERT_EQUAL_INT(0, tx_message(init.msg, init.msg_len, false, 0));
    TEST_ASSERT_EQUAL_INT(1, g.stats.rx_frame_err);
    TEST_ASSERT_EQUAL_INT(4, g.stats.tx_echo_ok);
}

TEST(kline_failed_echo_does_not_publish_a_successful_loopback) {
    bus_init_t init = hds_init(NULL, 0);
    init_test_corrupt_echo = true;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_ECHO,
                          tx_message(init.msg, init.msg_len, false, 0));
    kline_frame_t frame;
    TEST_ASSERT_EQUAL_INT(pdFALSE, xQueueReceive(g.rx_q, &frame, 0));
    TEST_ASSERT_EQUAL_INT(1, g.stats.tx_echo_bad);
}

static void clear_stats_after_first_tx_byte(void) {
    if (uart_tx_count == 1)
        kline_reset_stats();
}

TEST(kline_stats_reset_does_not_abort_valid_tx) {
    bus_init_t init = hds_init(NULL, 0);
    g.stats.rx_frame_err = 1;
    g.stats.rx_overrun = 1;
    uart_write_hook = clear_stats_after_first_tx_byte;
    TEST_ASSERT_EQUAL_INT(0, tx_message(init.msg, init.msg_len, false, 0));
}

TEST(kline_response_frame_error_does_not_fail_or_retry_good_tx) {
    const uint8_t reply[] = {0x0e, 0x04, 0x36, 0xb8};
    bus_init_t init = hds_init(reply, sizeof(reply));
    uart_error_after_data = UART_FRAME_ERR;
    g.cfg.tx_retries = 1;
    TEST_ASSERT_EQUAL_INT(0, tx_message(init.msg, init.msg_len, false, 0));
    TEST_ASSERT_EQUAL_INT(4, uart_tx_count);
    TEST_ASSERT_EQUAL_INT(4, g.stats.tx_echo_ok);
    TEST_ASSERT_EQUAL_INT(0, g.stats.tx_retries);
    TEST_ASSERT_EQUAL_INT(1, g.stats.rx_frame_err);
    kline_frame_t frame;
    TEST_ASSERT_EQUAL_INT(pdTRUE, xQueueReceive(g.rx_q, &frame, 0));
    TEST_ASSERT_EQUAL_INT(BUS_RX_TX_MSG_TYPE, frame.status);
    TEST_ASSERT_EQUAL_INT(0, rx_service(now_us() + 100000, &frame, true));

    /* A later intact frame must survive the error recovery gap. */
    memcpy(uart_rx_data, reply, sizeof(reply));
    uart_rx_len = sizeof(reply);
    TEST_ASSERT_EQUAL_INT(1, rx_service(now_us() + 100000, &frame, true));
    TEST_ASSERT_EQUAL_INT(sizeof(reply), frame.len);
    TEST_ASSERT_EQUAL_MEM(reply, frame.data, sizeof(reply));
}

TEST(kline_response_parity_error_does_not_fail_good_tx) {
    bus_init_t init = hds_init(NULL, 0);
    uart_error_after_data = UART_PARITY_ERR;
    TEST_ASSERT_EQUAL_INT(0, tx_message(init.msg, init.msg_len, false, 0));
    TEST_ASSERT_EQUAL_INT(1, g.stats.rx_frame_err);
}

TEST(kline_response_overflow_does_not_fail_confirmed_echo_prefix) {
    bus_init_t init = hds_init(NULL, 0);
    uart_error_after_data = UART_BUFFER_FULL;
    TEST_ASSERT_EQUAL_INT(0, tx_message(init.msg, init.msg_len, false, 0));
    TEST_ASSERT_EQUAL_INT(1, g.stats.rx_overrun);
}

TEST(kline_parity_error_before_echo_confirmation_fails_tx) {
    bus_init_t init = hds_init(NULL, 0);
    uart_injected_event = UART_PARITY_ERR;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_ECHO,
                          tx_message(init.msg, init.msg_len, false, 0));
}

TEST(kline_stats_reset_cannot_hide_a_new_uart_error) {
    bus_init_t init = hds_init(NULL, 0);
    uart_write_hook = clear_stats_after_first_tx_byte;
    uart_injected_event = UART_FRAME_ERR;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_ECHO,
                          tx_message(init.msg, init.msg_len, false, 0));
}

TEST(kline_lost_data_notifications_do_not_lose_echo_or_reply_bytes) {
    const uint8_t reply[] = {0x0e, 0x04, 0x36, 0xb8};
    bus_init_t init = hds_init(reply, sizeof(reply));
    uart_omit_data_events = true;
    TEST_ASSERT_EQUAL_INT(0, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(sizeof(reply), init.reply_len);
    TEST_ASSERT_EQUAL_MEM(reply, init.reply, sizeof(reply));
    TEST_ASSERT_EQUAL_INT(4, g.stats.tx_echo_ok);
}

static bool error_before_blocked_data;

static void deliver_error_and_echo_during_read(TickType_t wait) {
    if (!wait)
        return;
    uart_read_hook = NULL;
    uart_batch_ready_us = 0;
    uart_event_t data = {.type = UART_DATA, .size = init_test_request_len};
    uart_event_t error = {.type = UART_FRAME_ERR};
    TEST_ASSERT_EQUAL_INT(
        pdTRUE,
        xQueueSend(g.uart_q, error_before_blocked_data ? &error : &data, 0));
    TEST_ASSERT_EQUAL_INT(
        pdTRUE,
        xQueueSend(g.uart_q, error_before_blocked_data ? &data : &error, 0));
}

TEST(kline_error_during_blocked_read_is_checked_before_echo) {
    bus_init_t init = hds_init(NULL, 0);
    uart_omit_data_events = true;
    uart_hold_all_echoes = true;
    uart_batch_delay_us = 6000;
    error_before_blocked_data = true;
    uart_read_hook = deliver_error_and_echo_during_read;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_ECHO,
                          tx_message(init.msg, init.msg_len, false, 0));
    TEST_ASSERT_NULL(uart_read_hook);
}

TEST(kline_response_error_during_blocked_read_keeps_good_tx) {
    bus_init_t init = hds_init(NULL, 0);
    uart_omit_data_events = true;
    uart_hold_all_echoes = true;
    uart_batch_delay_us = 6000;
    error_before_blocked_data = false;
    uart_read_hook = deliver_error_and_echo_during_read;
    TEST_ASSERT_EQUAL_INT(0, tx_message(init.msg, init.msg_len, false, 0));
    TEST_ASSERT_NULL(uart_read_hook);
    TEST_ASSERT_EQUAL_INT(4, g.stats.tx_echo_ok);
}

static void clear_stats_after_error_during_read(TickType_t wait) {
    if (!wait)
        return;
    uart_read_hook = NULL;
    uart_event_t error = {.type = UART_FRAME_ERR};
    TEST_ASSERT_EQUAL_INT(pdTRUE, xQueueSend(g.uart_q, &error, 0));
    uart_events();
    kline_reset_stats();
    uart_batch_ready_us = 0;
}

TEST(kline_stats_reset_preserves_an_already_latched_tx_fault) {
    bus_init_t init = hds_init(NULL, 0);
    uart_omit_data_events = true;
    uart_hold_all_echoes = true;
    uart_batch_delay_us = 6000;
    uart_read_hook = clear_stats_after_error_during_read;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_ECHO,
                          tx_message(init.msg, init.msg_len, false, 0));
    TEST_ASSERT_EQUAL_INT(0, g.stats.rx_frame_err);
    TEST_ASSERT_NULL(g.tx);
}

TEST(kline_fast_init_rejects_damaged_response_after_good_tx) {
    const uint8_t reply[] = {0x0e, 0x04, 0x36, 0xb8};
    bus_init_t init = hds_init(reply, sizeof(reply));
    uart_error_after_data = UART_FRAME_ERR;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_INIT, do_init(CMD_FAST_INIT, &init));
    TEST_ASSERT_EQUAL_INT(4, g.stats.tx_echo_ok);
    TEST_ASSERT_EQUAL_INT(0, g.stats.tx_retries);
    TEST_ASSERT_EQUAL_INT(0, init.reply_len);
    TEST_ASSERT_EQUAL_INT(0, g.stats.init_ok);
    TEST_ASSERT_NULL(g.tx);
}

TEST(kline_controlled_send_defers_for_reply_received_during_quiet_wait) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    uart_immediate_echo = true;
    stage_uart_reply(previous_reply, sizeof(previous_reply));
    g.tx_answered = false;
    bus_tx_control_t control;
    bus_tx_prepare(&control, 0);
    control.check_rx = true;
    g.active_control = &control;
    TEST_ASSERT_EQUAL_INT(
        BUS_ERR_RX_PENDING,
        tx_message(current_request, sizeof(current_request), true, 0));
    g.active_control = NULL;
    TEST_ASSERT_EQUAL_INT(0, uart_tx_count);
    uint8_t data[16];
    bus_msg_t message;
    bus_msg_init(&message, data, sizeof(data));
    TEST_ASSERT_EQUAL_INT(sizeof(previous_reply), kline_rx(&message, 0));
    TEST_ASSERT_EQUAL_MEM(previous_reply, data, sizeof(previous_reply));
    TEST_ASSERT_EQUAL_INT(1, message.sequence);

    bus_tx_prepare(&control, message.sequence);
    control.check_rx = true;
    g.active_control = &control;
    TEST_ASSERT_EQUAL_INT(
        0, tx_message(current_request, sizeof(current_request), true, 0));
    g.active_control = NULL;
    TEST_ASSERT_EQUAL_INT(sizeof(current_request), uart_tx_count);
    TEST_ASSERT_TRUE(control.started_us >=
                     message.timestamp_us + P_TO_US(g.cfg.p3_min));
    TEST_ASSERT_EQUAL_HEX32(g.tx_end_us, control.ended_us);
}

TEST(kline_controlled_send_frames_buffered_input_even_after_long_idle) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    fake_clock_advance_ms(250);
    uart_immediate_echo = true;
    stage_uart_reply(previous_reply, sizeof(previous_reply));
    bus_tx_control_t control;
    bus_tx_prepare(&control, 0);
    control.check_rx = true;
    g.active_control = &control;
    TEST_ASSERT_EQUAL_INT(
        BUS_ERR_RX_PENDING,
        tx_message(current_request, sizeof(current_request), true, 0));
    g.active_control = NULL;
    TEST_ASSERT_EQUAL_INT(0, uart_tx_count);
    TEST_ASSERT_EQUAL_INT(1, g.stats.rx_msgs);
}

static int64_t cancel_tx_at;

static void cancel_waiting_tx(TickType_t wait) {
    if (now_us() >= cancel_tx_at) {
        uart_read_hook = NULL;
        TEST_ASSERT_TRUE(bus_tx_cancel(g.active_control));
    }
}

TEST(kline_cancellation_during_quiet_wait_sends_no_bytes) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    bus_tx_control_t control;
    bus_tx_prepare(&control, 0);
    g.active_control = &control;
    cancel_tx_at = now_us() + 5000;
    uart_read_hook = cancel_waiting_tx;
    TEST_ASSERT_EQUAL_INT(
        BUS_ERR_CANCELLED,
        tx_message(current_request, sizeof(current_request), true, 0));
    g.active_control = NULL;
    TEST_ASSERT_EQUAL_INT(0, uart_tx_count);
}

static void cancel_started_tx(void) {
    uart_write_hook = NULL;
    TEST_ASSERT_FALSE(bus_tx_cancel(g.active_control));
}

TEST(kline_transmission_already_started_finishes_when_stop_arrives) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    uart_immediate_echo = true;
    bus_tx_control_t control;
    bus_tx_prepare(&control, 0);
    g.active_control = &control;
    uart_write_hook = cancel_started_tx;
    TEST_ASSERT_EQUAL_INT(
        0, tx_message(current_request, sizeof(current_request), true, 0));
    g.active_control = NULL;
    TEST_ASSERT_EQUAL_INT(sizeof(current_request), uart_tx_count);
    TEST_ASSERT_TRUE(control.ended_us >= control.started_us);
}

TEST(kline_uart_overflow_blocks_repeat_admission_without_a_later_response) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, kline_open(NULL));
    uart_event_t event = {.type = UART_BUFFER_FULL};
    uart_event(&event);
    bus_tx_control_t control;
    bus_tx_prepare(&control, 0);
    control.check_rx = true;
    g.active_control = &control;
    TEST_ASSERT_EQUAL_INT(
        BUS_ERR_RX_PENDING,
        tx_message(current_request, sizeof(current_request), true, 0));
    g.active_control = NULL;
    uint8_t buffer[16];
    bus_msg_t message;
    bus_msg_init(&message, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_INT(0, kline_rx(&message, 0));
    TEST_ASSERT_EQUAL_INT(BUS_RX_BUFFER_OVERFLOW, message.status);
    TEST_ASSERT_EQUAL_INT(1, message.sequence);
    TEST_ASSERT_EQUAL_INT(0, uart_tx_count);
}
