/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file td_test.h
 * @brief Minimal test framework for the OpenDiag host tests.
 *
 * No external dependencies, one header and one source file. The assertion
 * macro names match Unity's, so if these tests ever want to move under
 * ESP-IDF's Unity runner the bodies do not have to change.
 *
 * Writing a test:
 *
 *     #include "td_test.h"
 *
 *     TEST(the_thing_does_the_thing)
 *     {
 *         TEST_ASSERT_EQUAL_INT(42, the_thing());
 *     }
 *
 * Registration is automatic. Define td_setup() and td_teardown() in the suite
 * to get a fresh fixture around every test; both are optional.
 *
 * A failed assertion aborts the current test and moves to the next one, so one
 * broken case does not cascade into a page of noise.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

typedef void (*td_test_fn)(void);

void td_register(const char *name, td_test_fn fn, const char *file);

/**
 * @brief Define and register a test case.
 */
#define TEST(name)                                                             \
    static void name(void);                                                    \
    static void __attribute__((constructor)) td_register_##name(void) {        \
        td_register(#name, name, __FILE__);                                    \
    }                                                                          \
    static void name(void)

/* Optional per-test fixture hooks, supplied by the suite. */
void td_setup(void) __attribute__((weak));
void td_teardown(void) __attribute__((weak));

/* ------------------------------------------------------------------ *
 * Assertions
 * ------------------------------------------------------------------ */

/** Records a failure and aborts the current test. Does not return. */
void td_fail(const char *file, int line, const char *fmt, ...)
    __attribute__((noreturn, format(printf, 3, 4)));

/**
 * @brief Renders control characters visibly, e.g. "OK\r" -> "OK\\r".
 *
 * Returns one of a few rotating static buffers, so several calls may appear in
 * the same printf without clobbering each other.
 */
const char *td_escape(const void *data, size_t len);

/** Renders bytes as space separated hex, using the same rotating buffers. */
const char *td_hex(const void *data, size_t len);

#define TEST_FAIL(msg) td_fail(__FILE__, __LINE__, "%s", (msg))

#define TEST_ASSERT(cond)                                                      \
    do {                                                                       \
        if (!(cond)) {                                                         \
            td_fail(__FILE__, __LINE__, "expected true: %s", #cond);           \
        }                                                                      \
    } while (0)

#define TEST_ASSERT_MSG(cond, ...)                                             \
    do {                                                                       \
        if (!(cond)) {                                                         \
            td_fail(__FILE__, __LINE__, __VA_ARGS__);                          \
        }                                                                      \
    } while (0)

#define TEST_ASSERT_FALSE(cond) TEST_ASSERT(!(cond))
#define TEST_ASSERT_TRUE(cond) TEST_ASSERT(cond)

#define TEST_ASSERT_NULL(ptr) TEST_ASSERT((ptr) == NULL)
#define TEST_ASSERT_NOT_NULL(ptr) TEST_ASSERT((ptr) != NULL)

#define TEST_ASSERT_EQUAL_INT(expected, actual)                                \
    do {                                                                       \
        long long td_e_ = (long long)(expected);                               \
        long long td_a_ = (long long)(actual);                                 \
        if (td_e_ != td_a_) {                                                  \
            td_fail(__FILE__, __LINE__, "%s: expected %lld, got %lld",         \
                    #actual, td_e_, td_a_);                                    \
        }                                                                      \
    } while (0)

#define TEST_ASSERT_EQUAL_HEX32(expected, actual)                              \
    do {                                                                       \
        uint32_t td_e_ = (uint32_t)(expected);                                 \
        uint32_t td_a_ = (uint32_t)(actual);                                   \
        if (td_e_ != td_a_) {                                                  \
            td_fail(__FILE__, __LINE__, "%s: expected 0x%08X, got 0x%08X",     \
                    #actual, td_e_, td_a_);                                    \
        }                                                                      \
    } while (0)

#define TEST_ASSERT_EQUAL_STRING(expected, actual)                             \
    do {                                                                       \
        const char *td_e_ = (expected);                                        \
        const char *td_a_ = (actual);                                          \
        if (!td_string_equal(td_e_, td_a_)) {                                  \
            td_fail(__FILE__, __LINE__,                                        \
                    "%s:\n    expected \"%s\"\n"                               \
                    "    actual   \"%s\"",                                     \
                    #actual,                                                   \
                    td_escape(td_e_, td_e_ ? __builtin_strlen(td_e_) : 0),     \
                    td_escape(td_a_, td_a_ ? __builtin_strlen(td_a_) : 0));    \
        }                                                                      \
    } while (0)

#define TEST_ASSERT_EQUAL_MEM(expected, actual, len)                           \
    do {                                                                       \
        const void *td_e_ = (expected);                                        \
        const void *td_a_ = (actual);                                          \
        size_t td_n_ = (size_t)(len);                                          \
        if (__builtin_memcmp(td_e_, td_a_, td_n_) != 0) {                      \
            td_fail(__FILE__, __LINE__,                                        \
                    "%s:\n    expected [%s]\n"                                 \
                    "    actual   [%s]",                                       \
                    #actual, td_hex(td_e_, td_n_), td_hex(td_a_, td_n_));      \
        }                                                                      \
    } while (0)

int td_string_equal(const char *a, const char *b);
