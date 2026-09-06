/* SPDX-License-Identifier: GPL-3.0-only */
#include <ctype.h>
#include <string.h>

#include "bus.h"
#include "td_test.h"

TEST(parameter_names_round_trip_in_both_cases) {
    bus_param_t expected, actual;
    const char *name;

    for (size_t i = 0; (name = bus_param_at(i, &expected)); i++) {
        char upper[64];
        TEST_ASSERT_TRUE(bus_param_from_name(name, &actual));
        TEST_ASSERT_EQUAL_INT(expected, actual);
        TEST_ASSERT_EQUAL_STRING(name, bus_param_name(actual));
        TEST_ASSERT_TRUE(strlen(name) < sizeof(upper));
        for (size_t j = 0; j <= strlen(name); j++)
            upper[j] = (char)toupper((unsigned char)name[j]);
        TEST_ASSERT_TRUE(bus_param_from_name(upper, &actual));
        TEST_ASSERT_EQUAL_INT(expected, actual);
    }
}

TEST(unknown_parameter_names_do_not_select_another_setting) {
    const char *invalid[] = {"", "ifr", "ifr-enabled-extra", "not-a-param"};
    bus_param_t actual = BUS_P_IFR_ENABLED;

    for (size_t i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++) {
        TEST_ASSERT_FALSE(bus_param_from_name(invalid[i], &actual));
        TEST_ASSERT_EQUAL_INT(BUS_P_IFR_ENABLED, actual);
    }
    TEST_ASSERT_FALSE(bus_param_from_name(NULL, &actual));
    TEST_ASSERT_FALSE(bus_param_from_name("ifr-enabled", NULL));
}
