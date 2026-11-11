/* SPDX-License-Identifier: GPL-3.0-only */
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/* bus_tx_worker_create() takes its context from calloc(). That one call is
 * routed through test_calloc() so a test can refuse it; the rest of the file
 * is unchanged. */
static bool refuse_calloc;
static void *test_calloc(size_t count, size_t size) {
    void *p;

    if (refuse_calloc)
        return NULL;
    p = malloc(count * size);
    if (p)
        memset(p, 0, count * size);
    return p;
}
#define calloc test_calloc
#include "bus_tx_worker.c"
#undef calloc

#include "driver_rtos.h"
#include "fake_clock.h"
#include "td_test.h"

static bus_tx_worker_t *worker;
static int sends;
static uint8_t captured;
static int plain_sends;
static uint8_t plain_captured;

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
    plain_sends = 0;
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

/* ------------------------------------------------------------------ *
 * Coverage: the paths above do not reach
 *
 * plain_ops completes each send without asserting on the way, and ends the
 * task after one send (stopping is set), so a test can let the worker run from
 * inside bus_tx_worker_wait() without the task looping forever.
 * ------------------------------------------------------------------ */

static int plain_send(const bus_msg_t *message, uint32_t flags,
                      bus_tx_control_t *control) {
    (void)flags;
    int rc = bus_tx_admit(control, 0, 1000);
    if (rc)
        return rc;
    plain_sends++;
    plain_captured = message->data[0];
    bus_tx_end(control, 8000);
    __atomic_store_n(&worker->stopping, true, __ATOMIC_RELEASE);
    return 0;
}
static const bus_ops_t plain_ops = {.send_controlled = plain_send};

static void submit_one(uint8_t value) {
    uint8_t data[] = {value};
    bus_msg_t message;
    bus_msg_tx(&message, data, sizeof(data), 0);
    TEST_ASSERT_EQUAL_INT(0, bus_tx_worker_submit(worker, &message, 0, 0));
}

TEST(create_refuses_a_missing_ops_table_or_send_function_without_a_task) {
    static const bus_ops_t no_send = {0};

    TEST_ASSERT_NULL(bus_tx_worker_create(NULL));
    TEST_ASSERT_NULL(bus_tx_worker_create(&no_send));
    TEST_ASSERT_EQUAL_INT(0, driver_allocations);
    TEST_ASSERT_EQUAL_INT(0, driver_task_starts);
    TEST_ASSERT_EQUAL_INT(0, driver_live_objects);
}

TEST(submit_refuses_bad_messages_and_queues_nothing) {
    uint8_t data[TX_DATA_MAX + 1] = {0};
    bus_msg_t message;

    worker = bus_tx_worker_create(&plain_ops);
    TEST_ASSERT_NOT_NULL(worker);

    bus_msg_tx(&message, data, sizeof(data), 0); /* one byte past the limit */
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BAD_ARG,
                          bus_tx_worker_submit(worker, &message, 0, 0));
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BAD_ARG,
                          bus_tx_worker_submit(NULL, &message, 0, 0));

    bus_msg_tx(&message, data, 1, 0);
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BAD_ARG,
                          bus_tx_worker_submit(worker, NULL, 0, 0));
    message.data = NULL;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BAD_ARG,
                          bus_tx_worker_submit(worker, &message, 0, 0));

    /* None of that took the slot: nothing is pending and a good one is taken. */
    bus_tx_result_t result = {.status = 77};
    TEST_ASSERT_FALSE(bus_tx_worker_poll(worker, &result));
    TEST_ASSERT_EQUAL_INT(77, result.status);
    TEST_ASSERT_EQUAL_INT(0, plain_sends);
    submit_one(0x33);
}

TEST(poll_on_no_worker_reports_nothing) {
    bus_tx_result_t result = {.status = 77};

    TEST_ASSERT_FALSE(bus_tx_worker_poll(NULL, &result));
    TEST_ASSERT_EQUAL_INT(77, result.status);
}

TEST(close_of_no_worker_is_a_no_op) {
    TEST_ASSERT_EQUAL_INT(ESP_OK, bus_tx_worker_close(NULL));
    TEST_ASSERT_EQUAL_INT(0, driver_live_objects);
}

TEST(wait_returns_at_once_when_nothing_is_pending) {
    TEST_ASSERT_EQUAL_INT(0, bus_tx_worker_wait(NULL));

    worker = bus_tx_worker_create(&plain_ops);
    TEST_ASSERT_NOT_NULL(worker);
    TEST_ASSERT_EQUAL_INT(0, bus_tx_worker_wait(worker));

    /* The loop was never entered, so no time was spent waiting. */
    TEST_ASSERT_EQUAL_INT(0, (int)fake_clock_ms());
    TEST_ASSERT_EQUAL_INT(0, plain_sends);
}

TEST(wait_lets_a_pending_send_run_and_returns_once_it_completes) {
    worker = bus_tx_worker_create(&plain_ops);
    TEST_ASSERT_NOT_NULL(worker);
    submit_one(0x5A);

    /* The wait's own take on the done semaphore lets the worker run. */
    driver_exit_on_wait = true;
    TEST_ASSERT_EQUAL_INT(0, bus_tx_worker_wait(worker));

    TEST_ASSERT_EQUAL_INT(1, plain_sends);
    TEST_ASSERT_EQUAL_INT(0x5A, plain_captured);

    bus_tx_result_t result;
    TEST_ASSERT_TRUE(bus_tx_worker_poll(worker, &result));
    TEST_ASSERT_TRUE(result.complete);
    TEST_ASSERT_TRUE(result.started);
    TEST_ASSERT_EQUAL_INT(0, result.status);
    TEST_ASSERT_EQUAL_INT(1000, result.started_us);
    TEST_ASSERT_EQUAL_INT(8000, result.ended_us);
}

TEST(wait_gives_up_after_the_timeout_when_the_send_never_finishes) {
    worker = bus_tx_worker_create(&plain_ops);
    TEST_ASSERT_NOT_NULL(worker);
    submit_one(0x01);

    /* Nothing runs the worker, so the wait sees an empty done semaphore and
     * the whole TX_WAIT_MS passes on the fake clock. */
    TEST_ASSERT_EQUAL_INT(BUS_ERR_TIMEOUT, bus_tx_worker_wait(worker));
    TEST_ASSERT_EQUAL_INT(4000, (int)fake_clock_ms());
    TEST_ASSERT_EQUAL_INT(0, plain_sends);

    /* Still pending: the timeout does not invent a completion. */
    bus_tx_result_t result;
    TEST_ASSERT_FALSE(bus_tx_worker_poll(worker, &result));
}

/* ------------------------------------------------------------------ *
 * Coverage: frame trains
 * ------------------------------------------------------------------ */

static int train_sends;
static unsigned train_count_seen;
static uint8_t train_first[3];
static uint8_t train_second[3];

static int train_send(const bus_msg_t *msgs, unsigned count,
                      bus_tx_control_t *control, bus_tx_train_t *train) {
    int rc = bus_tx_admit(control, 0, 1000);
    if (rc)
        return rc;
    train_sends++;
    train_count_seen = count;
    train_first[0] = msgs[0].data[0];
    train_second[0] = msgs[1].data[0];
    train->sent = count;
    train->ended_us[0] = 2000;
    train->ended_us[1] = 3000;
    bus_tx_end(control, 3000);
    __atomic_store_n(&worker->stopping, true, __ATOMIC_RELEASE);
    return 0;
}
static const bus_ops_t train_ops = {.send_controlled = plain_send,
                                    .send_train = train_send};

TEST(train_is_copied_sent_as_one_and_reported_with_per_frame_end_times) {
    uint8_t first[] = {0x11};
    uint8_t second[] = {0x22};
    bus_msg_t msgs[2];

    train_sends = 0;
    worker = bus_tx_worker_create(&train_ops);
    TEST_ASSERT_NOT_NULL(worker);
    bus_msg_tx(&msgs[0], first, sizeof(first), 0);
    bus_msg_tx(&msgs[1], second, sizeof(second), 0);
    TEST_ASSERT_EQUAL_INT(0, bus_tx_worker_submit_train(worker, msgs, 2, 0, 0));
    /* The worker owns its copy: changing the caller's bytes changes nothing. */
    first[0] = 0;
    second[0] = 0;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_NO_SPACE,
                          bus_tx_worker_submit_train(worker, msgs, 2, 0, 0));

    driver_run_worker();
    TEST_ASSERT_EQUAL_INT(1, train_sends);
    TEST_ASSERT_EQUAL_INT(0, plain_sends);
    TEST_ASSERT_EQUAL_INT(2, (int)train_count_seen);
    TEST_ASSERT_EQUAL_INT(0x11, train_first[0]);
    TEST_ASSERT_EQUAL_INT(0x22, train_second[0]);

    bus_tx_result_t result;
    TEST_ASSERT_TRUE(bus_tx_worker_poll(worker, &result));
    TEST_ASSERT_TRUE(result.complete);
    TEST_ASSERT_EQUAL_INT(0, result.status);
    TEST_ASSERT_NOT_NULL(result.train);
    TEST_ASSERT_EQUAL_INT(2, (int)result.train->sent);
    TEST_ASSERT_EQUAL_INT(2000, result.train->ended_us[0]);
    TEST_ASSERT_EQUAL_INT(3000, result.train->ended_us[1]);
}

TEST(train_submit_refuses_bad_arguments_and_queues_nothing) {
    uint8_t data[9] = {0};
    bus_msg_t msgs[BUS_TX_TRAIN_MAX + 1];

    worker = bus_tx_worker_create(&train_ops);
    TEST_ASSERT_NOT_NULL(worker);
    for (unsigned i = 0; i < BUS_TX_TRAIN_MAX + 1; i++)
        bus_msg_tx(&msgs[i], data, 1, 0);

    TEST_ASSERT_EQUAL_INT(BUS_ERR_BAD_ARG,
                          bus_tx_worker_submit_train(NULL, msgs, 1, 0, 0));
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BAD_ARG,
                          bus_tx_worker_submit_train(worker, NULL, 1, 0, 0));
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BAD_ARG,
                          bus_tx_worker_submit_train(worker, msgs, 0, 0, 0));
    TEST_ASSERT_EQUAL_INT(
        BUS_ERR_BAD_ARG,
        bus_tx_worker_submit_train(worker, msgs, BUS_TX_TRAIN_MAX + 1, 0, 0));
    /* Nine bytes does not fit a frame of the train. */
    bus_msg_tx(&msgs[1], data, 9, 0);
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BAD_ARG,
                          bus_tx_worker_submit_train(worker, msgs, 2, 0, 0));
    bus_msg_tx(&msgs[1], data, 1, 0);
    msgs[1].data = NULL;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_BAD_ARG,
                          bus_tx_worker_submit_train(worker, msgs, 2, 0, 0));

    bus_tx_result_t result;
    TEST_ASSERT_FALSE(bus_tx_worker_poll(worker, &result));
}

TEST(train_submit_is_refused_without_driver_support_or_while_stopping) {
    uint8_t data[] = {1};
    bus_msg_t message;

    bus_msg_tx(&message, data, sizeof(data), 0);
    worker = bus_tx_worker_create(&plain_ops);
    TEST_ASSERT_NOT_NULL(worker);
    TEST_ASSERT_EQUAL_INT(BUS_ERR_UNSUPPORTED,
                          bus_tx_worker_submit_train(worker, &message, 1, 0, 0));
    driver_exit_on_wait = true;
    TEST_ASSERT_EQUAL_INT(ESP_OK, bus_tx_worker_close(worker));
    worker = NULL;
    driver_exit_on_wait = false;

    worker = bus_tx_worker_create(&train_ops);
    TEST_ASSERT_NOT_NULL(worker);
    __atomic_store_n(&worker->stopping, true, __ATOMIC_RELEASE);
    TEST_ASSERT_EQUAL_INT(BUS_ERR_NOT_READY,
                          bus_tx_worker_submit_train(worker, &message, 1, 0, 0));
}

/* ------------------------------------------------------------------ *
 * Coverage: the context allocation, the bus notify and the RX epoch
 * ------------------------------------------------------------------ */

TEST(a_refused_context_allocation_creates_no_worker) {
    refuse_calloc = true;
    TEST_ASSERT_NULL(bus_tx_worker_create(&ops));
    refuse_calloc = false;

    worker = bus_tx_worker_create(&ops);
    TEST_ASSERT_NOT_NULL(worker);
}

TEST(a_listener_is_notified_only_while_a_task_is_bound) {
    static int task_token;
    TaskHandle_t task = (TaskHandle_t)&task_token;
    bus_listener_t listener = {0};

    idf_stub_notify_reset();
    bus_notify(&listener);
    TEST_ASSERT_FALSE(idf_stub_notify_pending());

    bus_listen(&listener, task);
    bus_notify(&listener);
    TEST_ASSERT_TRUE(idf_stub_notify_pending());
    TEST_ASSERT_TRUE(idf_stub_notified_task() == task);
    idf_stub_notify_reset();
}

TEST(a_receive_sequence_that_wraps_skips_zero) {
    bus_rx_epoch_t epoch = {.sequence = UINT32_MAX};

    TEST_ASSERT_EQUAL_HEX32(1, bus_rx_next(&epoch));
    TEST_ASSERT_EQUAL_HEX32(2, bus_rx_next(&epoch));
}
