/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_power_policy.c
 * @brief When the adapter sleeps to spare a parked vehicle's battery.
 */
#include "power_policy.h"
#include "td_test.h"

static const power_cfg_t cfg = {
    .enabled = true, .idle_s = 600, .run_mv = 13200};

/* Parked, nobody connected, bus asleep: 12.4 V. */
static power_sample_t parked(void) {
    return (power_sample_t){.vbatt_mv = 12400};
}

TEST(a_parked_quiet_vehicle_sleeps_after_the_idle_time) {
    power_state_t st = {0};
    power_sample_t s = parked();

    for (int i = 0; i < 599; i++)
        TEST_ASSERT_FALSE(power_policy_tick(&st, &cfg, &s, 1));
    TEST_ASSERT_EQUAL_INT(POWER_HOLD_NONE, st.hold);
    TEST_ASSERT_TRUE(power_policy_tick(&st, &cfg, &s, 1));
}

TEST(any_reason_to_stay_up_restarts_the_count) {
    power_state_t st = {0};
    power_sample_t s = parked();

    for (int i = 0; i < 500; i++)
        power_policy_tick(&st, &cfg, &s, 1);
    s.bus_traffic = true;
    TEST_ASSERT_FALSE(power_policy_tick(&st, &cfg, &s, 1));
    TEST_ASSERT_EQUAL_INT(POWER_HOLD_TRAFFIC, st.hold);
    TEST_ASSERT_EQUAL_INT(0, st.quiet_s);
    s.bus_traffic = false;
    for (int i = 0; i < 599; i++)
        TEST_ASSERT_FALSE(power_policy_tick(&st, &cfg, &s, 1));
    TEST_ASSERT_TRUE(power_policy_tick(&st, &cfg, &s, 1));
}

TEST(never_sleeps_with_a_client_a_claim_or_the_engine_running) {
    power_state_t st = {0};
    power_sample_t s;

    s = parked();
    s.client = true;
    TEST_ASSERT_FALSE(power_policy_tick(&st, &cfg, &s, 10000));
    TEST_ASSERT_EQUAL_INT(POWER_HOLD_CLIENT, st.hold);

    s = parked();
    s.busy = true;
    TEST_ASSERT_FALSE(power_policy_tick(&st, &cfg, &s, 10000));
    TEST_ASSERT_EQUAL_INT(POWER_HOLD_BUSY, st.hold);

    s = parked();
    s.vbatt_mv = 13900;
    TEST_ASSERT_FALSE(power_policy_tick(&st, &cfg, &s, 10000));
    TEST_ASSERT_EQUAL_INT(POWER_HOLD_RUNNING, st.hold);
}

TEST(never_sleeps_on_usb_power_alone_or_when_disabled) {
    power_state_t st = {0};
    power_sample_t s = parked();
    power_cfg_t off = cfg;

    s.vbatt_mv = 300;
    TEST_ASSERT_FALSE(power_policy_tick(&st, &cfg, &s, 10000));
    TEST_ASSERT_EQUAL_INT(POWER_HOLD_NO_VEHICLE, st.hold);

    off.enabled = false;
    s = parked();
    TEST_ASSERT_FALSE(power_policy_tick(&st, &off, &s, 10000));
    TEST_ASSERT_EQUAL_INT(POWER_HOLD_DISABLED, st.hold);
}

TEST(a_timer_wake_goes_back_to_sleep_unless_the_engine_runs) {
    power_cfg_t off = cfg;

    TEST_ASSERT_TRUE(power_policy_back_to_sleep(&cfg, 12300));
    TEST_ASSERT_FALSE(power_policy_back_to_sleep(&cfg, 13200));
    TEST_ASSERT_FALSE(power_policy_back_to_sleep(&cfg, 500));
    off.enabled = false;
    TEST_ASSERT_FALSE(power_policy_back_to_sleep(&off, 12300));
}

TEST(every_hold_has_a_name) {
    for (int h = POWER_HOLD_NONE; h <= POWER_HOLD_TRAFFIC; h++)
        TEST_ASSERT_TRUE(power_hold_name((power_hold_t)h)[0] != '?');
}
