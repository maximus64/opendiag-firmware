/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file test_utility_cov.c
 * @brief The shared helpers: the hex parsers, the hex dump and the delay.
 *
 * utility.c is included rather than linked, so this suite builds with no extra
 * source list. Every helper here is pure except hexdump(), which writes to
 * stdout; that output is captured through a temporary file so the rows can be
 * checked byte for byte.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "td_test.h"

#include "fake_clock.h"

#include "utility.c"

/* ------------------------------------------------------------------ *
 * Capturing stdout
 * ------------------------------------------------------------------ */

static FILE *g_capture;
static int g_saved_stdout = -1;

static void capture_begin(void) {
    fflush(stdout);
    g_capture = tmpfile();
    TEST_ASSERT_NOT_NULL(g_capture);
    g_saved_stdout = dup(STDOUT_FILENO);
    TEST_ASSERT_TRUE(g_saved_stdout >= 0);
    TEST_ASSERT_TRUE(dup2(fileno(g_capture), STDOUT_FILENO) >= 0);
}

static const char *capture_end(void) {
    static char text[4096];
    size_t length;
    int restored;

    fflush(stdout);
    restored = dup2(g_saved_stdout, STDOUT_FILENO);
    close(g_saved_stdout);
    g_saved_stdout = -1;
    rewind(g_capture);
    length = fread(text, 1, sizeof(text) - 1, g_capture);
    text[length] = '\0';
    fclose(g_capture);
    g_capture = NULL;
    TEST_ASSERT_TRUE(restored >= 0);
    return text;
}

void td_setup(void) { fake_clock_reset(); }

/* ------------------------------------------------------------------ *
 * hexdump
 * ------------------------------------------------------------------ */

TEST(hexdump_prints_a_full_row_in_hex_then_ascii) {
    uint8_t data[16];

    for (int i = 0; i < 16; i++) {
        data[i] = (uint8_t)('A' + i);
    }

    capture_begin();
    hexdump(data, sizeof(data));
    const char *out = capture_end();

    /* An extra space after the eighth byte splits the row in two halves. */
    TEST_ASSERT_EQUAL_STRING("41 42 43 44 45 46 47 48  49 4A 4B 4C 4D 4E 4F 50 "
                             "|  ABCDEFGHIJKLMNOP\n",
                             out);
}

TEST(hexdump_pads_a_short_row_and_shows_unprintables_as_dots) {
    static const uint8_t data[] = {0x00, 0x7F, 0x41};

    capture_begin();
    hexdump(data, sizeof(data));
    const char *out = capture_end();

    TEST_ASSERT_EQUAL_INT(0, strncmp(out, "00 7F 41 ", 9));

    /* The gutter sits in the same column as on a full row. */
    const char *bar = strchr(out, '|');
    TEST_ASSERT_NOT_NULL(bar);
    TEST_ASSERT_EQUAL_INT(49, (int)(bar - out));
    TEST_ASSERT_EQUAL_STRING("|  ..A\n", bar);
}

TEST(hexdump_starts_a_new_row_after_sixteen_bytes) {
    uint8_t data[17];

    for (int i = 0; i < 17; i++) {
        data[i] = (uint8_t)('A' + i);
    }

    capture_begin();
    hexdump(data, sizeof(data));
    const char *out = capture_end();

    const char *second = strchr(out, '\n');
    TEST_ASSERT_NOT_NULL(second);
    second++;

    TEST_ASSERT_EQUAL_INT(0, strncmp(second, "51 ", 3));
    const char *bar = strchr(second, '|');
    TEST_ASSERT_NOT_NULL(bar);
    TEST_ASSERT_EQUAL_INT(49, (int)(bar - second));
    TEST_ASSERT_EQUAL_STRING("|  Q\n", bar);
}

TEST(hexdump_of_zero_bytes_prints_nothing) {
    static const uint8_t data[1] = {0};

    capture_begin();
    hexdump(data, 0);
    const char *out = capture_end();

    TEST_ASSERT_EQUAL_STRING("", out);
}

/* ------------------------------------------------------------------ *
 * hex_char_to_int
 * ------------------------------------------------------------------ */

TEST(hex_char_to_int_maps_digits_and_both_cases_of_a_to_f) {
    TEST_ASSERT_EQUAL_INT(0, hex_char_to_int('0'));
    TEST_ASSERT_EQUAL_INT(9, hex_char_to_int('9'));
    TEST_ASSERT_EQUAL_INT(10, hex_char_to_int('a'));
    TEST_ASSERT_EQUAL_INT(12, hex_char_to_int('c'));
    TEST_ASSERT_EQUAL_INT(15, hex_char_to_int('f'));
    TEST_ASSERT_EQUAL_INT(10, hex_char_to_int('A'));
    TEST_ASSERT_EQUAL_INT(15, hex_char_to_int('F'));
}

TEST(hex_char_to_int_rejects_the_characters_either_side_of_each_range) {
    TEST_ASSERT_EQUAL_INT(-1, hex_char_to_int('/')); /* just below '0' */
    TEST_ASSERT_EQUAL_INT(-1, hex_char_to_int(':')); /* just above '9' */
    TEST_ASSERT_EQUAL_INT(-1, hex_char_to_int('@')); /* just below 'A' */
    TEST_ASSERT_EQUAL_INT(-1, hex_char_to_int('G')); /* just above 'F' */
    TEST_ASSERT_EQUAL_INT(-1, hex_char_to_int('`')); /* just below 'a' */
    TEST_ASSERT_EQUAL_INT(-1, hex_char_to_int('g')); /* just above 'f' */
    TEST_ASSERT_EQUAL_INT(-1, hex_char_to_int(' '));
    TEST_ASSERT_EQUAL_INT(-1, hex_char_to_int('\0'));
}

/* ------------------------------------------------------------------ *
 * hex_string_to_u8_array
 * ------------------------------------------------------------------ */

TEST(hex_string_to_u8_array_decodes_both_cases_of_each_digit) {
    static const uint8_t expect[4] = {0x0A, 0xFF, 0x1B, 0x2C};
    uint8_t out[4] = {0};

    TEST_ASSERT_EQUAL_INT(4,
                          hex_string_to_u8_array("0aFf1B2c", 8, out, sizeof(out)));
    TEST_ASSERT_EQUAL_MEM(expect, out, sizeof(expect));
}

TEST(hex_string_to_u8_array_of_nothing_is_zero_bytes) {
    uint8_t out[4] = {0xEE, 0xEE, 0xEE, 0xEE};

    TEST_ASSERT_EQUAL_INT(0, hex_string_to_u8_array("", 0, out, sizeof(out)));
    TEST_ASSERT_EQUAL_INT(0xEE, out[0]);
}

TEST(hex_string_to_u8_array_refuses_more_bytes_than_fit_and_writes_nothing) {
    uint8_t out[2] = {0xEE, 0xEE};

    TEST_ASSERT_EQUAL_INT(-2,
                          hex_string_to_u8_array("010203", 6, out, sizeof(out)));
    TEST_ASSERT_EQUAL_INT(0xEE, out[0]);
    TEST_ASSERT_EQUAL_INT(0xEE, out[1]);
}

TEST(hex_string_to_u8_array_stops_at_a_bad_digit_in_either_position) {
    uint8_t out[4] = {0};

    /* Bad high nibble, then bad low nibble. */
    TEST_ASSERT_EQUAL_INT(-1,
                          hex_string_to_u8_array("G0", 2, out, sizeof(out)));
    TEST_ASSERT_EQUAL_INT(-1,
                          hex_string_to_u8_array("0G", 2, out, sizeof(out)));
}

/* ------------------------------------------------------------------ *
 * hex_string_to_u32_be
 * ------------------------------------------------------------------ */

TEST(hex_string_to_u32_be_reads_up_to_eight_digits_big_endian) {
    uint32_t value = 0;

    TEST_ASSERT_EQUAL_INT(0, hex_string_to_u32_be("12345678", 8, &value));
    TEST_ASSERT_EQUAL_HEX32(0x12345678u, value);

    TEST_ASSERT_EQUAL_INT(0, hex_string_to_u32_be("1aB", 3, &value));
    TEST_ASSERT_EQUAL_HEX32(0x1ABu, value);

    TEST_ASSERT_EQUAL_INT(0, hex_string_to_u32_be("", 0, &value));
    TEST_ASSERT_EQUAL_HEX32(0u, value);
}

TEST(hex_string_to_u32_be_refuses_more_than_eight_digits_and_keeps_output) {
    uint32_t value = 0xDEADBEEFu;

    TEST_ASSERT_EQUAL_INT(-1, hex_string_to_u32_be("123456789", 9, &value));
    TEST_ASSERT_EQUAL_HEX32(0xDEADBEEFu, value);
}

TEST(hex_string_to_u32_be_refuses_a_bad_digit_and_keeps_output) {
    uint32_t value = 0xDEADBEEFu;

    TEST_ASSERT_EQUAL_INT(-1, hex_string_to_u32_be("12Z4", 4, &value));
    TEST_ASSERT_EQUAL_HEX32(0xDEADBEEFu, value);
}

/* ------------------------------------------------------------------ *
 * delay_us
 *
 * The timer is the fake one: every read advances it by a microsecond, so the
 * busy-wait ends after a deterministic number of reads and never sleeps.
 * ------------------------------------------------------------------ */

TEST(delay_us_returns_once_the_timer_has_moved_on_by_the_request) {
    int64_t start = esp_timer_get_time();

    delay_us(0); /* one read, and it is already due */
    delay_us(25);

    TEST_ASSERT_TRUE(esp_timer_get_time() - start >= 25);
}
