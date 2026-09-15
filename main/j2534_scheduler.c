/* SPDX-License-Identifier: GPL-3.0-only */
#include "j2534_scheduler.h"

static bool at_or_after(uint32_t time, uint32_t boundary) {
    return (int32_t)(time - boundary) >= 0;
}

j2534_schedule_job_t *j2534_schedule_find(j2534_scheduler_t *scheduler,
                                          uint32_t id) {
    for (unsigned i = 0; i < 2 * J2534_SCHEDULE_SLOTS; i++)
        if (id && scheduler->jobs[i].id == id)
            return &scheduler->jobs[i];
    return NULL;
}

j2534_schedule_job_t *j2534_schedule_allocate(j2534_scheduler_t *scheduler,
                                              bool repeat) {
    unsigned first = repeat ? J2534_SCHEDULE_SLOTS : 0;
    for (unsigned i = first; i < first + J2534_SCHEDULE_SLOTS; i++)
        if (!scheduler->jobs[i].id)
            return &scheduler->jobs[i];
    return NULL;
}

j2534_schedule_job_t *j2534_schedule_next(j2534_scheduler_t *scheduler,
                                          uint32_t now_us, bool allow_repeat) {
    j2534_schedule_job_t *next = NULL;
    for (unsigned i = 0; i < 2 * J2534_SCHEDULE_SLOTS; i++) {
        j2534_schedule_job_t *job = &scheduler->jobs[i];
        if (!job->active || job->pending || !at_or_after(now_us, job->due_us) ||
            (!allow_repeat && job->kind != J2534_SCHEDULE_PERIODIC))
            continue;
        if (!next || (int32_t)(job->due_us - next->due_us) < 0 ||
            (job->due_us == next->due_us && (int32_t)(job->id - next->id) < 0))
            next = job;
    }
    return next;
}

static bool matches(const j2534_schedule_job_t *job, const uint8_t *data,
                    size_t len, uint32_t flags) {
    if (len < job->match_len || flags != job->match_flags)
        return false;
    for (unsigned i = 0; i < job->match_len; i++)
        if ((data[i] & job->mask[i]) != job->pattern[i])
            return false;
    return true;
}

void j2534_schedule_receive(j2534_scheduler_t *scheduler, const uint8_t *data,
                            size_t len, uint32_t flags, uint32_t timestamp_us) {
    for (unsigned i = J2534_SCHEDULE_SLOTS; i < 2 * J2534_SCHEDULE_SLOTS; i++) {
        j2534_schedule_job_t *job = &scheduler->jobs[i];
        if (!job->active || !at_or_after(timestamp_us, job->created_us))
            continue;
        bool match = matches(job, data, len, flags);
        if ((job->kind == J2534_SCHEDULE_UNTIL && match) ||
            (job->kind == J2534_SCHEDULE_WHILE && !match)) {
            job->active = false;
        } else if (match) {
            job->have_match = true;
            job->match_us = timestamp_us;
            if (job->sent && at_or_after(timestamp_us, job->ended_us) &&
                at_or_after(job->due_us, timestamp_us))
                job->matched = true;
        }
    }
}

void j2534_schedule_expire(j2534_scheduler_t *scheduler, uint32_t now_us) {
    for (unsigned i = J2534_SCHEDULE_SLOTS; i < 2 * J2534_SCHEDULE_SLOTS; i++) {
        j2534_schedule_job_t *job = &scheduler->jobs[i];
        if (job->active && job->kind == J2534_SCHEDULE_WHILE && job->sent &&
            (!job->pending || job->transmission_ended) && !job->matched &&
            at_or_after(now_us, job->due_us))
            job->active = false;
    }
}

void j2534_schedule_transmitted(j2534_schedule_job_t *job, uint32_t started_us,
                                uint32_t ended_us) {
    if (job->transmission_ended && job->ended_us == ended_us)
        return;
    job->transmission_ended = true;
    job->sent = true;
    job->ended_us = ended_us;
    job->due_us =
        (job->kind == J2534_SCHEDULE_PERIODIC ? started_us : ended_us) +
        job->interval_us;
    job->matched = job->have_match && at_or_after(job->match_us, ended_us) &&
                   at_or_after(job->due_us, job->match_us);
}

void j2534_schedule_defer(j2534_schedule_job_t *job) { job->pending = false; }

void j2534_schedule_complete(j2534_schedule_job_t *job, bool success,
                             uint32_t started_us, uint32_t ended_us) {
    job->pending = false;
    if (!success && job->kind != J2534_SCHEDULE_PERIODIC) {
        job->active = false;
        return;
    }
    j2534_schedule_transmitted(job, started_us, ended_us);
}

void j2534_schedule_receive_lost(j2534_scheduler_t *scheduler) {
    for (unsigned i = J2534_SCHEDULE_SLOTS; i < 2 * J2534_SCHEDULE_SLOTS; i++)
        scheduler->jobs[i].active = false;
}
