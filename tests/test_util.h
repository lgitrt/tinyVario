/*
 * test_util.h — Minimal header-only assertion helper for the host-side unit tests.
 *
 * Author: Luca Obwegs
 *
 * No external test framework dependency: each test executable is a plain
 * `main()` that calls CHECK()/CHECK_NEAR() and reports a pass/fail summary
 * via its exit code, so it plugs directly into `make test` and CI.
 */
#ifndef TEST_UTIL_H_
#define TEST_UTIL_H_

#include <math.h>
#include <stdio.h>

static int g_checks_run    = 0;
static int g_checks_failed = 0;

#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        g_checks_run++;                                                      \
        if (!(cond)) {                                                       \
            g_checks_failed++;                                               \
            fprintf(stderr, "  FAIL (%s:%d): %s\n", __FILE__, __LINE__, msg); \
        }                                                                     \
    } while (0)

#define CHECK_NEAR(actual, expected, tol, msg)                               \
    CHECK(fabsf((float)(actual) - (float)(expected)) <= (float)(tol), msg)

#define TEST_SUMMARY()                                                       \
    do {                                                                     \
        printf("%s: %d/%d checks passed\n", __FILE__,                       \
               g_checks_run - g_checks_failed, g_checks_run);                \
        return g_checks_failed ? 1 : 0;                                      \
    } while (0)

#endif /* TEST_UTIL_H_ */
