/* SPDX-License-Identifier: GPL-3.0-only */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define J2534_SCHEDULE_SLOTS 10u
#define J2534_SCHEDULE_DATA_MAX 12u

typedef enum {
    J2534_SCHEDULE_PERIODIC,
    J2534_SCHEDULE_UNTIL,
    J2534_SCHEDULE_WHILE,
} j2534_schedule_kind_t;

typedef struct {
    uint32_t flags;
    uint32_t handle;
    uint8_t len;
    uint8_t data[J2534_SCHEDULE_DATA_MAX];
} j2534_scheduled_message_t;

typedef struct {
    uint32_t id;
    j2534_schedule_kind_t kind;
    uint32_t interval_us;
    uint32_t due_us;
    uint32_t ended_us;
    uint32_t created_us;
    uint32_t match_us;
    bool active;
    bool pending;
    bool transmission_ended;
    bool sent;
    bool matched;
    bool have_match;
    j2534_scheduled_message_t message;
    uint32_t match_flags;
    uint8_t match_len;
    uint8_t mask[J2534_SCHEDULE_DATA_MAX];
    uint8_t pattern[J2534_SCHEDULE_DATA_MAX];
} j2534_schedule_job_t;

typedef struct {
    j2534_schedule_job_t jobs[2 * J2534_SCHEDULE_SLOTS];
} j2534_scheduler_t;

j2534_schedule_job_t *j2534_schedule_find(j2534_scheduler_t *scheduler,
                                          uint32_t id);
/* Returns an unused slot in the independent periodic/repeat quota. */
j2534_schedule_job_t *j2534_schedule_allocate(j2534_scheduler_t *scheduler,
                                              bool repeat);
j2534_schedule_job_t *j2534_schedule_next(j2534_scheduler_t *scheduler,
                                          uint32_t now_us, bool allow_repeat);
void j2534_schedule_receive(j2534_scheduler_t *scheduler, const uint8_t *data,
                            size_t len, uint32_t flags, uint32_t timestamp_us);
void j2534_schedule_expire(j2534_scheduler_t *scheduler, uint32_t now_us);
void j2534_schedule_transmitted(j2534_schedule_job_t *job, uint32_t started_us,
                                uint32_t ended_us);
void j2534_schedule_defer(j2534_schedule_job_t *job);
void j2534_schedule_complete(j2534_schedule_job_t *job, bool success,
                             uint32_t started_us, uint32_t ended_us);
void j2534_schedule_receive_lost(j2534_scheduler_t *scheduler);
