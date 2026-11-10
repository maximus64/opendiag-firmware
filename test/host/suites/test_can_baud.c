/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_can_baud.c
 * @brief Picking a bus's bit rate from what listen-only opens heard.
 */
#include "can_baud.h"
#include "can_frame.h"
#include "td_test.h"

TEST(the_rate_with_clean_frames_wins) {
    const can_baud_sample_t s[] = {
        {500000, 0, 900}, {250000, 2, 300}, {50000, 120, 0}, {125000, 0, 50}};

    TEST_ASSERT_EQUAL_INT(2, can_baud_pick(s, 4));
}

TEST(a_silent_bus_has_no_answer) {
    const can_baud_sample_t s[] = {{500000, 0, 0}, {50000, 0, 0}};

    TEST_ASSERT_EQUAL_INT(-1, can_baud_pick(s, 2));
}

TEST(frames_drowned_in_errors_are_not_an_answer) {
    const can_baud_sample_t s[] = {{500000, 10, 40}, {250000, 8, 1}};

    TEST_ASSERT_EQUAL_INT(1, can_baud_pick(s, 2));
    TEST_ASSERT_EQUAL_INT(-1, can_baud_pick(s, 1));
}

TEST(every_rate_tried_is_one_the_adapter_offers) {
    size_t n;
    const uint32_t *order = can_baud_order(&n);

    TEST_ASSERT_TRUE(n >= 12);
    for (size_t i = 0; i < n; i++)
        TEST_ASSERT_EQUAL_INT(order[i], can_bitrate_canonical(order[i]));
}
