/* SPDX-License-Identifier: GPL-3.0-only */
#include <string.h>
#include "bus_tx.h"
#include "j2534_scheduler.h"
#include "td_test.h"

static j2534_scheduler_t scheduler;
void td_setup(void) { memset(&scheduler, 0, sizeof(scheduler)); }
void td_teardown(void) {}

static j2534_schedule_job_t *create(j2534_schedule_kind_t kind, uint32_t id) {
    j2534_schedule_job_t *job =
        j2534_schedule_allocate(&scheduler, kind != J2534_SCHEDULE_PERIODIC);
    TEST_ASSERT_NOT_NULL(job);
    *job = (j2534_schedule_job_t){.id = id,
                                  .kind = kind,
                                  .interval_us = 20000,
                                  .active = true,
                                  .match_len = 2,
                                  .mask = {0xff, 0xf0},
                                  .pattern = {0x48, 0x60}};
    return job;
}

TEST(periodic_interval_starts_at_tx_start_repeat_at_tx_end) {
    j2534_schedule_job_t *periodic = create(J2534_SCHEDULE_PERIODIC, 1);
    j2534_schedule_job_t *repeat = create(J2534_SCHEDULE_UNTIL, 2);
    j2534_schedule_complete(periodic, true, 1000, 8000);
    j2534_schedule_complete(repeat, true, 1000, 8000);
    TEST_ASSERT_EQUAL_HEX32(21000, periodic->due_us);
    TEST_ASSERT_EQUAL_HEX32(28000, repeat->due_us);
}

TEST(deferred_admission_preserves_the_original_due_copy) {
    j2534_schedule_job_t *job = create(J2534_SCHEDULE_UNTIL, 1);
    job->due_us = 1234;
    job->pending = true;
    TEST_ASSERT_NULL(j2534_schedule_next(&scheduler, 2000, true));
    j2534_schedule_defer(job);
    TEST_ASSERT_TRUE(job->active);
    TEST_ASSERT_EQUAL_HEX32(1234, job->due_us);
    TEST_ASSERT_TRUE(j2534_schedule_next(&scheduler, 2000, true) == job);
}

TEST(earliest_due_order_prevents_a_slow_low_index_job_starving_others) {
    j2534_schedule_job_t *first = create(J2534_SCHEDULE_PERIODIC, 1);
    j2534_schedule_job_t *second = create(J2534_SCHEDULE_UNTIL, 2);
    TEST_ASSERT_TRUE(j2534_schedule_next(&scheduler, 0, true) == first);
    j2534_schedule_complete(first, true, 0, 50000);
    TEST_ASSERT_TRUE(j2534_schedule_next(&scheduler, 50000, true) == second);
}

TEST(prefix_matching_ignores_extra_bytes_but_not_short_messages_or_flags) {
    j2534_schedule_job_t *job = create(J2534_SCHEDULE_UNTIL, 1);
    const uint8_t data[] = {0x48, 0x6b, 0x10};
    j2534_schedule_receive(&scheduler, data, 1, 0, 1000);
    TEST_ASSERT_TRUE(job->active);
    j2534_schedule_receive(&scheduler, data, 3, 256, 1000);
    TEST_ASSERT_TRUE(job->active);
    j2534_schedule_receive(&scheduler, data, 3, 0, 1000);
    TEST_ASSERT_FALSE(job->active);
    TEST_ASSERT_TRUE(j2534_schedule_find(&scheduler, 1) == job);
}

TEST(pattern_bits_outside_mask_do_not_match) {
    j2534_schedule_job_t *job = create(J2534_SCHEDULE_UNTIL, 1);
    job->pattern[1] = 0x6b;
    const uint8_t data[] = {0x48, 0x6b};
    j2534_schedule_receive(&scheduler, data, 2, 0, 1000);
    TEST_ASSERT_TRUE(job->active);
}

TEST(until_continues_in_silence_while_stops_at_end_relative_deadline) {
    j2534_schedule_job_t *until = create(J2534_SCHEDULE_UNTIL, 1);
    j2534_schedule_job_t *while_job = create(J2534_SCHEDULE_WHILE, 2);
    j2534_schedule_complete(until, true, 1000, 8000);
    j2534_schedule_complete(while_job, true, 1000, 8000);
    j2534_schedule_expire(&scheduler, 27999);
    TEST_ASSERT_TRUE(while_job->active);
    j2534_schedule_expire(&scheduler, 28000);
    TEST_ASSERT_FALSE(while_job->active);
    TEST_ASSERT_TRUE(until->active);
}

TEST(while_requires_a_new_match_each_interval_and_stops_on_any_nonmatch) {
    j2534_schedule_job_t *job = create(J2534_SCHEDULE_WHILE, 1);
    const uint8_t matching[] = {0x48, 0x6b};
    j2534_schedule_complete(job, true, 1000, 8000);
    j2534_schedule_receive(&scheduler, matching, 2, 0, 20000);
    j2534_schedule_expire(&scheduler, 28000);
    TEST_ASSERT_TRUE(job->active);
    j2534_schedule_complete(job, true, 28000, 35000);
    j2534_schedule_expire(&scheduler, 55000);
    TEST_ASSERT_FALSE(job->active);
    job->active = true;
    j2534_schedule_receive(&scheduler, matching, 1, 0, 55001);
    TEST_ASSERT_FALSE(job->active);
}

TEST(late_matching_message_cannot_rescue_a_silent_interval) {
    j2534_schedule_job_t *job = create(J2534_SCHEDULE_WHILE, 1);
    const uint8_t matching[] = {0x48, 0x6b};
    j2534_schedule_complete(job, true, 1000, 8000);
    j2534_schedule_receive(&scheduler, matching, 2, 0, 28001);
    j2534_schedule_expire(&scheduler, 28002);
    TEST_ASSERT_FALSE(job->active);
}

TEST(response_before_sender_returns_counts_in_the_new_window) {
    j2534_schedule_job_t *job = create(J2534_SCHEDULE_WHILE, 1);
    const uint8_t matching[] = {0x48, 0x6b};
    job->pending = true;
    j2534_schedule_receive(&scheduler, matching, 2, 0, 9000);
    j2534_schedule_complete(job, true, 1000, 8000);
    j2534_schedule_expire(&scheduler, 28000);
    TEST_ASSERT_TRUE(job->active);
}

TEST(stopping_during_transmission_is_not_undone_by_completion) {
    j2534_schedule_job_t *job = create(J2534_SCHEDULE_UNTIL, 1);
    const uint8_t matching[] = {0x48, 0x6b};
    job->pending = true;
    j2534_schedule_receive(&scheduler, matching, 2, 0, 9000);
    j2534_schedule_complete(job, true, 1000, 8000);
    TEST_ASSERT_FALSE(job->active);
}

TEST(timer_wrap_keeps_due_order_and_silence_deadline) {
    j2534_schedule_job_t *job = create(J2534_SCHEDULE_WHILE, 1);
    j2534_schedule_complete(job, true, UINT32_MAX - 10000, UINT32_MAX - 5000);
    j2534_schedule_expire(&scheduler, 14998);
    TEST_ASSERT_TRUE(job->active);
    j2534_schedule_expire(&scheduler, 14999);
    TEST_ASSERT_FALSE(job->active);
}

TEST(completed_repeat_ids_still_consume_the_independent_repeat_quota) {
    for (unsigned i = 0; i < J2534_SCHEDULE_SLOTS; i++) {
        j2534_schedule_job_t *job = create(J2534_SCHEDULE_UNTIL, i + 1);
        job->active = false;
    }
    TEST_ASSERT_NULL(j2534_schedule_allocate(&scheduler, true));
    TEST_ASSERT_NOT_NULL(j2534_schedule_allocate(&scheduler, false));
}

TEST(receive_loss_stops_repeat_conditions_and_preserves_periodics) {
    j2534_schedule_job_t *periodic = create(J2534_SCHEDULE_PERIODIC, 1);
    j2534_schedule_job_t *repeat = create(J2534_SCHEDULE_UNTIL, 2);
    j2534_schedule_receive_lost(&scheduler);
    TEST_ASSERT_TRUE(periodic->active);
    TEST_ASSERT_FALSE(repeat->active);
}

TEST(cancellation_and_receive_guard_apply_only_before_admission) {
    bus_tx_control_t control;
    bus_tx_prepare(&control, 5);
    control.check_rx = true;
    TEST_ASSERT_EQUAL_INT(BUS_ERR_RX_PENDING, bus_tx_admit(&control, 6, 1000));
    TEST_ASSERT_TRUE(bus_tx_cancel(&control));
    TEST_ASSERT_EQUAL_INT(BUS_ERR_CANCELLED, bus_tx_admit(&control, 5, 1000));
    bus_tx_prepare(&control, 6);
    control.check_rx = true;
    TEST_ASSERT_EQUAL_INT(0, bus_tx_admit(&control, 6, 2000));
    TEST_ASSERT_FALSE(bus_tx_cancel(&control));
    TEST_ASSERT_EQUAL_INT(0, bus_tx_admit(&control, 7, 3000));
    TEST_ASSERT_EQUAL_HEX32(2000, control.started_us);
    bus_tx_end(&control, 4000);
    TEST_ASSERT_EQUAL_HEX32(4000, control.ended_us);
}

TEST(receive_loss_remains_visible_after_the_queue_is_drained) {
    bus_rx_epoch_t epoch = {0};
    bus_msg_t message = {0};
    TEST_ASSERT_EQUAL_HEX32(1, bus_rx_next(&epoch));
    bus_rx_lost(&epoch, 1);
    TEST_ASSERT_EQUAL_HEX32(2, bus_rx_next(&epoch));
    bus_rx_lost(&epoch, 2);
    TEST_ASSERT_TRUE(bus_rx_take_loss(&epoch, &message));
    TEST_ASSERT_EQUAL_HEX32(2, message.sequence);
    TEST_ASSERT_EQUAL_INT(BUS_RX_BUFFER_OVERFLOW, message.status);
    TEST_ASSERT_FALSE(bus_rx_take_loss(&epoch, &message));
}

TEST(periodic_transmission_failure_does_not_delete_the_definition_or_spin) {
    j2534_schedule_job_t *job = create(J2534_SCHEDULE_PERIODIC, 1);
    j2534_schedule_complete(job, false, 10000, 10000);
    TEST_ASSERT_TRUE(job->active);
    TEST_ASSERT_NULL(j2534_schedule_next(&scheduler, 29999, true));
    TEST_ASSERT_TRUE(j2534_schedule_next(&scheduler, 30000, true) == job);
}

TEST(receive_backlog_defers_repeats_without_blocking_periodic_progress) {
    j2534_schedule_job_t *repeat = create(J2534_SCHEDULE_UNTIL, 1);
    j2534_schedule_job_t *periodic = create(J2534_SCHEDULE_PERIODIC, 2);
    TEST_ASSERT_TRUE(j2534_schedule_next(&scheduler, 0, true) == repeat);
    TEST_ASSERT_TRUE(j2534_schedule_next(&scheduler, 0, false) == periodic);
}
