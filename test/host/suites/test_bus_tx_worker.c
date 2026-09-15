/* SPDX-License-Identifier: GPL-3.0-only */
#include "bus_tx_worker.c"
#include "driver_rtos.h"
#include "fake_clock.h"
#include "td_test.h"

static bus_tx_worker_t *worker;
static int sends;
static uint8_t captured;

static int send_controlled(const bus_msg_t *message, uint32_t flags,
                           bus_tx_control_t *control) {
    int rc = bus_tx_admit(control, 0, 1000);
    if (rc)
        return rc;
    sends++;
    captured = message->data[0];
    TEST_ASSERT_FALSE(bus_tx_cancel(control));
    bus_tx_end(control, 8000);
    bus_tx_result_t timing;
    TEST_ASSERT_TRUE(bus_tx_worker_poll(worker, &timing));
    TEST_ASSERT_FALSE(timing.complete);
    TEST_ASSERT_EQUAL_INT(8000, timing.ended_us);
    TEST_ASSERT_FALSE(bus_tx_worker_poll(worker, &timing));
    /* End the deterministic task invocation after this completed send. */
    __atomic_store_n(&worker->stopping, true, __ATOMIC_RELEASE);
    return 0;
}
static const bus_ops_t ops = {.send_controlled = send_controlled};

void td_setup(void) {
    driver_rtos_reset();
    fake_clock_reset();
    worker = NULL;
    sends = 0;
}
void td_teardown(void) {
    driver_exit_on_wait = true;
    if (worker)
        TEST_ASSERT_EQUAL_INT(ESP_OK, bus_tx_worker_close(worker));
    TEST_ASSERT_EQUAL_INT(0, driver_live_objects);
}

TEST(worker_owns_the_packet_and_reports_physical_end_before_final_completion) {
    worker = bus_tx_worker_create(&ops);
    TEST_ASSERT_NOT_NULL(worker);
    uint8_t data[] = {0x41};
    bus_msg_t message;
    bus_msg_tx(&message, data, sizeof(data), 0);
    TEST_ASSERT_EQUAL_INT(0, bus_tx_worker_submit(worker, &message, 0, 0));
    data[0] = 0;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_NO_SPACE,
                          bus_tx_worker_submit(worker, &message, 0, 0));
    driver_run_worker();
    TEST_ASSERT_EQUAL_INT(1, sends);
    TEST_ASSERT_EQUAL_INT(0x41, captured);
    bus_tx_result_t result;
    TEST_ASSERT_TRUE(bus_tx_worker_poll(worker, &result));
    TEST_ASSERT_TRUE(result.complete);
    TEST_ASSERT_TRUE(result.started);
    TEST_ASSERT_EQUAL_INT(0, result.status);
    TEST_ASSERT_EQUAL_INT(1000, result.started_us);
    TEST_ASSERT_EQUAL_INT(8000, result.ended_us);
}

TEST(worker_close_cancels_queued_send_and_joins_before_freeing_storage) {
    worker = bus_tx_worker_create(&ops);
    TEST_ASSERT_NOT_NULL(worker);
    uint8_t data[] = {1};
    bus_msg_t message;
    bus_msg_tx(&message, data, sizeof(data), 0);
    TEST_ASSERT_EQUAL_INT(0, bus_tx_worker_submit(worker, &message, 0, 0));
    driver_exit_on_wait = true;
    TEST_ASSERT_EQUAL_INT(ESP_OK, bus_tx_worker_close(worker));
    worker = NULL;
    TEST_ASSERT_EQUAL_INT(0, sends);
    TEST_ASSERT_EQUAL_INT(1, driver_task_exits);
}

TEST(worker_failed_close_retains_request_until_late_exit_acknowledgement) {
    worker = bus_tx_worker_create(&ops);
    TEST_ASSERT_NOT_NULL(worker);
    uint8_t data[] = {1};
    bus_msg_t message;
    bus_msg_tx(&message, data, sizeof(data), 0);
    TEST_ASSERT_EQUAL_INT(0, bus_tx_worker_submit(worker, &message, 0, 0));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT, bus_tx_worker_close(worker));
    TEST_ASSERT_EQUAL_INT(3, driver_live_objects);
    TEST_ASSERT_EQUAL_INT(BUS_ERR_NOT_READY,
                          bus_tx_worker_submit(worker, &message, 0, 0));
    driver_run_worker();
    TEST_ASSERT_EQUAL_INT(ESP_OK, bus_tx_worker_close(worker));
    worker = NULL;
    TEST_ASSERT_EQUAL_INT(0, sends);
}

TEST(worker_partial_creation_failures_release_all_semaphores) {
    for (int i = 1; i <= 3; i++) {
        driver_fail_allocation = driver_allocations + i;
        TEST_ASSERT_NULL(bus_tx_worker_create(&ops));
        TEST_ASSERT_EQUAL_INT(0, driver_live_objects);
    }
    driver_fail_allocation = 0;
    driver_fail_task = true;
    TEST_ASSERT_NULL(bus_tx_worker_create(&ops));
    TEST_ASSERT_EQUAL_INT(0, driver_live_objects);
}
