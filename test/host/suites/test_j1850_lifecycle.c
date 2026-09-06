/* SPDX-License-Identifier: GPL-3.0-only */
#include <string.h>
#include "fake_clock.h"
#include "j1850_lifecycle.h"
#include "td_test.h"

struct rmt_channel_t {
    bool enabled;
    bool pending;
    bool deleted;
};
struct gptimer_t {
    bool enabled;
    bool running;
    bool deleted;
};

enum operation {
    RMT_ENABLE,
    RMT_DISABLE,
    RMT_DELETE,
    RMT_DRAIN,
    TIMER_ENABLE,
    TIMER_START,
    TIMER_DISARM,
    TIMER_STOP,
    TIMER_DISABLE,
    TIMER_DELETE,
    OP_COUNT
};
static esp_err_t result[OP_COUNT];
static int calls[OP_COUNT];

void td_setup(void) {
    memset(result, 0, sizeof(result));
    memset(calls, 0, sizeof(calls));
}

#define CALL(op)                                                               \
    do {                                                                       \
        calls[op]++;                                                           \
        if (result[op])                                                        \
            return result[op];                                                 \
    } while (0)

esp_err_t rmt_enable(rmt_channel_handle_t ch) {
    CALL(RMT_ENABLE);
    TEST_ASSERT_FALSE(ch->enabled);
    TEST_ASSERT_FALSE(ch->pending);
    ch->enabled = true;
    return ESP_OK;
}
esp_err_t rmt_disable(rmt_channel_handle_t ch) {
    CALL(RMT_DISABLE);
    TEST_ASSERT_TRUE(ch->enabled);
    ch->enabled = false;
    return ESP_OK;
}
esp_err_t rmt_del_channel(rmt_channel_handle_t ch) {
    CALL(RMT_DELETE);
    TEST_ASSERT_FALSE(ch->enabled);
    ch->deleted = true;
    return ESP_OK;
}
esp_err_t rmt_tx_wait_all_done(rmt_channel_handle_t ch, int ms) {
    CALL(RMT_DRAIN);
    TEST_ASSERT_FALSE(ch->enabled);
    TEST_ASSERT_EQUAL_INT(100, ms);
    ch->pending = false;
    return ESP_OK;
}
esp_err_t gptimer_enable(gptimer_handle_t timer) {
    CALL(TIMER_ENABLE);
    TEST_ASSERT_FALSE(timer->enabled);
    timer->enabled = true;
    return ESP_OK;
}
esp_err_t gptimer_start(gptimer_handle_t timer) {
    CALL(TIMER_START);
    TEST_ASSERT_TRUE(timer->enabled);
    TEST_ASSERT_FALSE(timer->running);
    timer->running = true;
    return ESP_OK;
}
esp_err_t gptimer_set_alarm_action(gptimer_handle_t timer,
                                   const gptimer_alarm_config_t *cfg) {
    CALL(TIMER_DISARM);
    TEST_ASSERT_NULL(cfg);
    return ESP_OK;
}
esp_err_t gptimer_stop(gptimer_handle_t timer) {
    CALL(TIMER_STOP);
    TEST_ASSERT_TRUE(timer->running);
    timer->running = false;
    return ESP_OK;
}
esp_err_t gptimer_disable(gptimer_handle_t timer) {
    CALL(TIMER_DISABLE);
    TEST_ASSERT_TRUE(timer->enabled);
    TEST_ASSERT_FALSE(timer->running);
    timer->enabled = false;
    return ESP_OK;
}
esp_err_t gptimer_del_timer(gptimer_handle_t timer) {
    CALL(TIMER_DELETE);
    TEST_ASSERT_FALSE(timer->enabled);
    TEST_ASSERT_FALSE(timer->running);
    timer->deleted = true;
    return ESP_OK;
}

TEST(rmt_cleanup_retains_failed_handle_and_retries_completed_steps_safely) {
    for (int fail = RMT_DISABLE; fail <= RMT_DELETE; fail++) {
        td_setup();
        struct rmt_channel_t channel = {.enabled = true};
        rmt_channel_handle_t handle = &channel;
        bool enabled = true;
        result[fail] = ESP_FAIL;
        TEST_ASSERT_EQUAL_INT(ESP_FAIL, j1850_rmt_delete(&handle, &enabled));
        TEST_ASSERT_TRUE(handle == &channel);
        TEST_ASSERT_FALSE(channel.deleted);
        TEST_ASSERT_EQUAL_INT(channel.enabled, enabled);
        TEST_ASSERT_EQUAL_INT(fail == RMT_DELETE, calls[RMT_DELETE]);
        result[fail] = ESP_OK;
        TEST_ASSERT_EQUAL_INT(ESP_OK, j1850_rmt_delete(&handle, &enabled));
        TEST_ASSERT_NULL(handle);
        TEST_ASSERT_TRUE(channel.deleted);
        TEST_ASSERT_EQUAL_INT(fail == RMT_DISABLE ? 2 : 1, calls[RMT_DISABLE]);
        TEST_ASSERT_EQUAL_INT(ESP_OK, j1850_rmt_delete(&handle, &enabled));
    }
}

TEST(rmt_failed_enable_can_be_deleted_without_disable) {
    struct rmt_channel_t channel = {0};
    rmt_channel_handle_t handle = &channel;
    bool enabled = false;
    result[RMT_ENABLE] = ESP_FAIL;
    TEST_ASSERT_EQUAL_INT(ESP_FAIL, j1850_rmt_enable(handle, &enabled));
    TEST_ASSERT_FALSE(enabled);
    TEST_ASSERT_EQUAL_INT(ESP_OK, j1850_rmt_delete(&handle, &enabled));
    TEST_ASSERT_EQUAL_INT(0, calls[RMT_DISABLE]);
}

TEST(tx_recovery_stops_and_drains_before_enabling) {
    struct rmt_channel_t channel = {.enabled = true, .pending = true};
    bool enabled = true;
    TEST_ASSERT_EQUAL_INT(ESP_OK, j1850_rmt_recover(&channel, &enabled));
    TEST_ASSERT_TRUE(enabled);
    TEST_ASSERT_FALSE(channel.pending);
    TEST_ASSERT_EQUAL_INT(1, calls[RMT_DISABLE]);
    TEST_ASSERT_EQUAL_INT(1, calls[RMT_DRAIN]);
    TEST_ASSERT_EQUAL_INT(1, calls[RMT_ENABLE]);
}

TEST(tx_failed_recovery_retains_state_for_close) {
    const int failures[] = {RMT_DISABLE, RMT_DRAIN, RMT_ENABLE};
    for (size_t i = 0; i < sizeof(failures) / sizeof(failures[0]); i++) {
        td_setup();
        int fail = failures[i];
        struct rmt_channel_t channel = {.enabled = true, .pending = true};
        rmt_channel_handle_t handle = &channel;
        bool enabled = true;
        result[fail] = ESP_ERR_TIMEOUT;
        TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT,
                              j1850_rmt_recover(handle, &enabled));
        TEST_ASSERT_EQUAL_INT(channel.enabled, enabled);
        TEST_ASSERT_EQUAL_INT(fail == RMT_ENABLE, calls[RMT_ENABLE]);
        if (fail == RMT_DISABLE)
            TEST_ASSERT_EQUAL_INT(0, calls[RMT_DRAIN]);
        result[fail] = ESP_OK;
        TEST_ASSERT_EQUAL_INT(ESP_OK, j1850_rmt_delete(&handle, &enabled));
        TEST_ASSERT_NULL(handle);
    }
}

TEST(timer_partial_startup_can_be_unwound) {
    const int failures[] = {TIMER_ENABLE, TIMER_START};
    for (size_t i = 0; i < sizeof(failures) / sizeof(failures[0]); i++) {
        td_setup();
        struct gptimer_t timer = {0};
        gptimer_handle_t handle = &timer;
        bool enabled = false, running = false;
        result[failures[i]] = ESP_FAIL;
        esp_err_t err = j1850_timer_enable(handle, &enabled);
        if (err == ESP_OK)
            err = j1850_timer_start(handle, &running);
        TEST_ASSERT_EQUAL_INT(ESP_FAIL, err);
        TEST_ASSERT_EQUAL_INT(ESP_OK,
                              j1850_timer_delete(&handle, &enabled, &running));
        TEST_ASSERT_NULL(handle);
        TEST_ASSERT_EQUAL_INT(0, calls[TIMER_STOP]);
        TEST_ASSERT_EQUAL_INT(failures[i] == TIMER_START, calls[TIMER_DISABLE]);
    }
}

TEST(timer_failed_cleanup_retains_handle_and_retries_remaining_steps) {
    for (int fail = TIMER_DISARM; fail <= TIMER_DELETE; fail++) {
        td_setup();
        struct gptimer_t timer = {.enabled = true, .running = true};
        gptimer_handle_t handle = &timer;
        bool enabled = true, running = true;
        result[fail] = ESP_FAIL;
        TEST_ASSERT_EQUAL_INT(ESP_FAIL,
                              j1850_timer_delete(&handle, &enabled, &running));
        TEST_ASSERT_TRUE(handle == &timer);
        TEST_ASSERT_FALSE(timer.deleted);
        TEST_ASSERT_EQUAL_INT(timer.enabled, enabled);
        TEST_ASSERT_EQUAL_INT(timer.running, running);
        for (int op = fail + 1; op <= TIMER_DELETE; op++) {
            TEST_ASSERT_EQUAL_INT(0, calls[op]);
        }
        result[fail] = ESP_OK;
        TEST_ASSERT_EQUAL_INT(ESP_OK,
                              j1850_timer_delete(&handle, &enabled, &running));
        TEST_ASSERT_NULL(handle);
        TEST_ASSERT_TRUE(timer.deleted);
        TEST_ASSERT_EQUAL_INT(fail == TIMER_STOP ? 2 : 1, calls[TIMER_STOP]);
        TEST_ASSERT_EQUAL_INT(fail == TIMER_DISABLE ? 2 : 1,
                              calls[TIMER_DISABLE]);
    }
}

TEST(callback_shutdown_waits_for_admitted_isr_and_rejects_late_entries) {
    j1850_callback_guard_t guard = {0};
    TEST_ASSERT_TRUE(j1850_callback_enter(&guard));
    TEST_ASSERT_EQUAL_INT(ESP_ERR_TIMEOUT, j1850_callbacks_stop(&guard));
    TEST_ASSERT_FALSE(j1850_callback_enter(&guard));
    TEST_ASSERT_EQUAL_INT(1, guard.active);
    j1850_callback_exit(&guard);
    TEST_ASSERT_EQUAL_INT(ESP_OK, j1850_callbacks_stop(&guard));
    TEST_ASSERT_FALSE(j1850_callback_enter(&guard));
    TEST_ASSERT_EQUAL_INT(0, guard.active);
    j1850_callbacks_start(&guard);
    TEST_ASSERT_TRUE(j1850_callback_enter(&guard));
    j1850_callback_exit(&guard);
    TEST_ASSERT_EQUAL_INT(ESP_OK, j1850_callbacks_stop(&guard));
}

TEST(timer_lifecycle_tracks_successful_start_and_close) {
    struct gptimer_t timer = {0};
    gptimer_handle_t handle = &timer;
    bool enabled = false, running = false;
    TEST_ASSERT_EQUAL_INT(ESP_OK, j1850_timer_enable(handle, &enabled));
    TEST_ASSERT_TRUE(enabled);
    TEST_ASSERT_EQUAL_INT(ESP_OK, j1850_timer_start(handle, &running));
    TEST_ASSERT_TRUE(running);
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          j1850_timer_delete(&handle, &enabled, &running));
    TEST_ASSERT_NULL(handle);
    TEST_ASSERT_FALSE(enabled);
    TEST_ASSERT_FALSE(running);
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          j1850_timer_delete(&handle, &enabled, &running));
    TEST_ASSERT_EQUAL_INT(1, calls[TIMER_DELETE]);
}

TEST(timer_created_but_never_enabled_can_be_deleted) {
    struct gptimer_t timer = {0};
    gptimer_handle_t handle = &timer;
    bool enabled = false, running = false;
    TEST_ASSERT_EQUAL_INT(ESP_OK,
                          j1850_timer_delete(&handle, &enabled, &running));
    TEST_ASSERT_NULL(handle);
    TEST_ASSERT_EQUAL_INT(0, calls[TIMER_STOP]);
    TEST_ASSERT_EQUAL_INT(0, calls[TIMER_DISABLE]);
}
