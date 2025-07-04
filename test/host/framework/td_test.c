/* SPDX-License-Identifier: GPL-3.0-only */
/**
 * @file td_test.c
 * @brief Registration, assertion reporting and the runner's main().
 *
 * Every test runs under an alarm. A firmware timeout loop that fails to make
 * progress against the fake clock would otherwise hang the whole run, and a
 * hang in CI is far more expensive to diagnose than a failure.
 */

#include "td_test.h"

#include <setjmp.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define TD_MAX_TESTS   256
#define TD_TIMEOUT_SEC 10

typedef struct {
    const char *name;
    td_test_fn fn;
    const char *file;
} td_case_t;

static td_case_t g_cases[TD_MAX_TESTS];
static int g_case_count;

static jmp_buf g_abort_test;
static int g_in_test;
static const char *g_current_name = "<none>";

void td_register(const char *name, td_test_fn fn, const char *file)
{
    if (g_case_count >= TD_MAX_TESTS) {
        fprintf(stderr, "td_test: more than %d tests in one binary\n", TD_MAX_TESTS);
        exit(2);
    }

    g_cases[g_case_count].name = name;
    g_cases[g_case_count].fn = fn;
    g_cases[g_case_count].file = file;
    g_case_count++;
}

/* ------------------------------------------------------------------ *
 * Rendering helpers
 * ------------------------------------------------------------------ */

#define TD_SCRATCH_SLOTS 4
#define TD_SCRATCH_SIZE  1024

static char g_scratch[TD_SCRATCH_SLOTS][TD_SCRATCH_SIZE];
static int g_scratch_next;

static char *scratch_take(void)
{
    char *p = g_scratch[g_scratch_next];

    g_scratch_next = (g_scratch_next + 1) % TD_SCRATCH_SLOTS;
    p[0] = '\0';

    return p;
}

const char *td_escape(const void *data, size_t len)
{
    const unsigned char *src = data;
    char *out = scratch_take();
    size_t o = 0;

    if (!data) {
        return "(null)";
    }

    for (size_t i = 0; i < len && o + 5 < TD_SCRATCH_SIZE; i++) {
        unsigned char c = src[i];

        switch (c) {
        case '\r': o += (size_t)sprintf(out + o, "\\r"); break;
        case '\n': o += (size_t)sprintf(out + o, "\\n"); break;
        case '\t': o += (size_t)sprintf(out + o, "\\t"); break;
        case '\\': o += (size_t)sprintf(out + o, "\\\\"); break;
        default:
            if (c < 0x20 || c >= 0x7f) {
                o += (size_t)sprintf(out + o, "\\x%02X", c);
            } else {
                out[o++] = (char)c;
            }
            break;
        }
    }

    out[o] = '\0';
    return out;
}

const char *td_hex(const void *data, size_t len)
{
    const unsigned char *src = data;
    char *out = scratch_take();
    size_t o = 0;

    if (!data) {
        return "(null)";
    }

    for (size_t i = 0; i < len && o + 4 < TD_SCRATCH_SIZE; i++) {
        o += (size_t)sprintf(out + o, i ? " %02X" : "%02X", src[i]);
    }

    out[o] = '\0';
    return out;
}

int td_string_equal(const char *a, const char *b)
{
    if (a == b) {
        return 1;
    }
    if (!a || !b) {
        return 0;
    }
    return strcmp(a, b) == 0;
}

/* ------------------------------------------------------------------ *
 * Failure path
 * ------------------------------------------------------------------ */

static int g_failures;

void td_fail(const char *file, int line, const char *fmt, ...)
{
    va_list ap;

    printf("  FAIL %s\n    at %s:%d\n    ", g_current_name, file, line);
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    printf("\n");
    fflush(stdout);

    g_failures++;

    if (g_in_test) {
        longjmp(g_abort_test, 1);
    }

    exit(1);
}

static void on_alarm(int sig)
{
    (void)sig;

    /* Cannot longjmp safely out of a signal handler into an unknown state, so
     * report and leave. The exit code marks the run as failed. */
    const char msg[] = "\n  TIMEOUT: test exceeded its time budget: ";
    ssize_t ignored;

    ignored = write(STDOUT_FILENO, msg, sizeof(msg) - 1);
    ignored = write(STDOUT_FILENO, g_current_name, strlen(g_current_name));
    ignored = write(STDOUT_FILENO, "\n", 1);
    (void)ignored;

    _exit(1);
}

/* ------------------------------------------------------------------ *
 * Runner
 * ------------------------------------------------------------------ */

int main(int argc, char **argv)
{
    const char *filter = NULL;
    int run = 0, passed = 0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--list") == 0) {
            for (int j = 0; j < g_case_count; j++) {
                printf("%s\n", g_cases[j].name);
            }
            return 0;
        }
        filter = argv[i];
    }

    signal(SIGALRM, on_alarm);

    for (int i = 0; i < g_case_count; i++) {
        if (filter && !strstr(g_cases[i].name, filter)) {
            continue;
        }

        g_current_name = g_cases[i].name;
        run++;

        int before = g_failures;

        alarm(TD_TIMEOUT_SEC);

        g_in_test = 1;
        if (setjmp(g_abort_test) == 0) {
            if (td_setup) {
                td_setup();
            }
            g_cases[i].fn();
        }
        g_in_test = 0;

        /* Runs even when the body aborted, so a fixture that owns resources
         * still gets to release them. */
        if (td_teardown) {
            td_teardown();
        }

        alarm(0);

        if (g_failures == before) {
            passed++;
            printf("  ok   %s\n", g_cases[i].name);
        }
    }

    printf("\n%d run, %d passed, %d failed\n", run, passed, run - passed);

    if (filter && run == 0) {
        printf("no test matched \"%s\"\n", filter);
        return 2;
    }

    return g_failures ? 1 : 0;
}
