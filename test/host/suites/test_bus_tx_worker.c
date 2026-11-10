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

static uint8_t train_bytes[BUS_TX_TRAIN_MAX];
static unsigned train_cancel_after;

/* Sends frame by frame; cancelling from "another task" after train_cancel_after
 * frames must stop it there. */
static int send_train(const bus_msg_t *msgs, unsigned count,
                      bus_tx_control_t *control, bus_tx_train_t *train) {
    train->sent = 0;
    int rc = bus_tx_admit(control, 0, 1000);
    if (rc)
        return rc;
    for (unsigned i = 0; i < count; i++) {
        if (train_cancel_after && i == train_cancel_after)
            bus_tx_worker_cancel(worker);
        if (bus_tx_stopped(control)) {
            rc = BUS_ERR_CANCELLED;
            break;
        }
        train_bytes[i] = msgs[i].data[0];
        train->ended_us[train->sent++] = 2000 + i;
    }
    bus_tx_end(control, 2000 + train->sent);
    __atomic_store_n(&worker->stopping, true, __ATOMIC_RELEASE);
    return rc;
}
static const bus_ops_t train_ops = {.send_controlled = send_controlled,
                                    .send_train = send_train};

void td_setup(void) {
    driver_rtos_reset();
    fake_clock_reset();
    worker = NULL;
    sends = 0;
    train_cancel_after = 0;
    memset(train_bytes, 0, sizeof(train_bytes));
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

TEST(a_finished_send_notifies_the_bus_listener) {
    static int holder;
    static bus_listener_t listener;
    static const bus_ops_t heard = {.send_controlled = send_controlled,
                                    .listener = &listener};
    bus_listen(&listener, (TaskHandle_t)&holder);
    worker = bus_tx_worker_create(&heard);
    TEST_ASSERT_NOT_NULL(worker);
    uint8_t data[] = {0x41};
    bus_msg_t message;
    bus_msg_tx(&message, data, sizeof(data), 0);
    TEST_ASSERT_EQUAL_INT(0, bus_tx_worker_submit(worker, &message, 0, 0));
    idf_stub_notify_reset();
    driver_run_worker();
    TEST_ASSERT_TRUE(idf_stub_notified_task() == (TaskHandle_t)&holder);
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

static void make_train(bus_msg_t *msgs, uint8_t *data, unsigned count) {
    for (unsigned i = 0; i < count; i++) {
        data[i] = (uint8_t)(0x21 + i);
        bus_msg_tx(&msgs[i], &data[i], 1, 0x7e0);
    }
}

TEST(worker_owns_a_trains_frames_and_reports_each_ones_end) {
    worker = bus_tx_worker_create(&train_ops);
    TEST_ASSERT_NOT_NULL(worker);
    bus_msg_t msgs[3];
    uint8_t data[3];
    make_train(msgs, data, 3);
    TEST_ASSERT_EQUAL_INT(0, bus_tx_worker_submit_train(worker, msgs, 3, 0, 0));
    memset(data, 0, sizeof(data));
    driver_run_worker();
    bus_tx_result_t result;
    TEST_ASSERT_TRUE(bus_tx_worker_poll(worker, &result));
    TEST_ASSERT_TRUE(result.complete);
    TEST_ASSERT_EQUAL_INT(0, result.status);
    TEST_ASSERT_NOT_NULL(result.train);
    TEST_ASSERT_EQUAL_INT(3, result.train->sent);
    for (unsigned i = 0; i < 3; i++) {
        TEST_ASSERT_EQUAL_INT(0x21 + i, train_bytes[i]);
        TEST_ASSERT_EQUAL_INT(2000 + i, result.train->ended_us[i]);
    }
}

TEST(worker_rejects_trains_it_cannot_carry) {
    bus_msg_t msgs[BUS_TX_TRAIN_MAX + 1];
    uint8_t data[BUS_TX_TRAIN_MAX + 1];
    make_train(msgs, data, BUS_TX_TRAIN_MAX + 1);
    worker = bus_tx_worker_create(&ops);
    TEST_ASSERT_EQUAL_INT(BUS_ERR_UNSUPPORTED,
                          bus_tx_worker_submit_train(worker, msgs, 1, 0, 0));
    driver_exit_on_wait = true;
    TEST_ASSERT_EQUAL_INT(ESP_OK, bus_tx_worker_close(worker));
    worker = bus_tx_worker_create(&train_ops);
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BAD_ARG,
                          bus_tx_worker_submit_train(worker, msgs, 0, 0, 0));
    TEST_ASSERT_EQUAL_INT(
        BUS_ERR_BAD_ARG,
        bus_tx_worker_submit_train(worker, msgs, BUS_TX_TRAIN_MAX + 1, 0, 0));
    uint8_t long_frame[9] = {0};
    bus_msg_tx(&msgs[1], long_frame, sizeof(long_frame), 0x7e0);
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BAD_ARG,
                          bus_tx_worker_submit_train(worker, msgs, 2, 0, 0));
}

TEST(cancelling_an_admitted_train_stops_it_between_frames) {
    worker = bus_tx_worker_create(&train_ops);
    bus_msg_t msgs[5];
    uint8_t data[5];
    make_train(msgs, data, 5);
    train_cancel_after = 2;
    TEST_ASSERT_EQUAL_INT(0, bus_tx_worker_submit_train(worker, msgs, 5, 0, 0));
    driver_run_worker();
    bus_tx_result_t result;
    TEST_ASSERT_TRUE(bus_tx_worker_poll(worker, &result));
    TEST_ASSERT_EQUAL_INT(BUS_ERR_CANCELLED, result.status);
    TEST_ASSERT_EQUAL_INT(2, result.train->sent);
    TEST_ASSERT_EQUAL_INT(0, train_bytes[2]);
}
