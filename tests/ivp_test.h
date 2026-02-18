/* ivp_test.h — Minimal unit test framework for IVP C17
 *
 * Usage:
 *   IVP_TEST(my_test) { IVP_ASSERT_TRUE(1 + 1 == 2); }
 *   int main(void) { IVP_RUN_ALL(); }
 *
 * Place IVP_TEST_LIST at the end of each test file to register all tests.
 */
#ifndef IVP_TEST_H
#define IVP_TEST_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ── Internal bookkeeping ───────────────────────────────────────────────── */

typedef void (*ivp_test_fn_t)(int *_fail_count);

typedef struct ivp_test_entry {
    const char    *name;
    ivp_test_fn_t  fn;
} ivp_test_entry_t;

#define IVP_MAX_TESTS 256
static ivp_test_entry_t s_tests[IVP_MAX_TESTS];
static int s_test_count = 0;

static inline void ivp_register_test(const char *name, ivp_test_fn_t fn) {
    if (s_test_count < IVP_MAX_TESTS) {
        s_tests[s_test_count].name = name;
        s_tests[s_test_count].fn   = fn;
        s_test_count++;
    }
}

/* ── Test definition macro ──────────────────────────────────────────────── */

/* Forward-declare and register via constructor attribute / init section. */
#define IVP_TEST(name)                                                     \
    static void ivp_test_##name(int *_fail_count);                         \
    static void ivp_test_##name(int *_fail_count)

/* ── Assertion macros ───────────────────────────────────────────────────── */

#define IVP_FAIL(msg)                                                      \
    do {                                                                   \
        fprintf(stderr, "  FAIL: %s (%s:%d)\n", (msg), __FILE__, __LINE__);\
        (*_fail_count)++;                                                  \
        return;                                                            \
    } while (0)

#define IVP_ASSERT_TRUE(expr)                                              \
    do {                                                                   \
        if (!(expr)) {                                                     \
            fprintf(stderr, "  ASSERT_TRUE failed: %s (%s:%d)\n",          \
                    #expr, __FILE__, __LINE__);                            \
            (*_fail_count)++;                                              \
            return;                                                        \
        }                                                                  \
    } while (0)

#define IVP_ASSERT_FALSE(expr)                                             \
    do {                                                                   \
        if ((expr)) {                                                      \
            fprintf(stderr, "  ASSERT_FALSE failed: %s (%s:%d)\n",         \
                    #expr, __FILE__, __LINE__);                            \
            (*_fail_count)++;                                              \
            return;                                                        \
        }                                                                  \
    } while (0)

#define IVP_ASSERT_EQ(a, b)                                                \
    do {                                                                   \
        if ((a) != (b)) {                                                  \
            fprintf(stderr, "  ASSERT_EQ failed: %s != %s (%s:%d)\n",     \
                    #a, #b, __FILE__, __LINE__);                           \
            (*_fail_count)++;                                              \
            return;                                                        \
        }                                                                  \
    } while (0)

#define IVP_ASSERT_NE(a, b)                                                \
    do {                                                                   \
        if ((a) == (b)) {                                                  \
            fprintf(stderr, "  ASSERT_NE failed: %s == %s (%s:%d)\n",     \
                    #a, #b, __FILE__, __LINE__);                           \
            (*_fail_count)++;                                              \
            return;                                                        \
        }                                                                  \
    } while (0)

#define IVP_ASSERT_NULL(ptr)                                               \
    do {                                                                   \
        if ((ptr) != NULL) {                                               \
            fprintf(stderr, "  ASSERT_NULL failed: %s (%s:%d)\n",          \
                    #ptr, __FILE__, __LINE__);                             \
            (*_fail_count)++;                                              \
            return;                                                        \
        }                                                                  \
    } while (0)

#define IVP_ASSERT_NOT_NULL(ptr)                                           \
    do {                                                                   \
        if ((ptr) == NULL) {                                               \
            fprintf(stderr, "  ASSERT_NOT_NULL failed: %s (%s:%d)\n",      \
                    #ptr, __FILE__, __LINE__);                             \
            (*_fail_count)++;                                              \
            return;                                                        \
        }                                                                  \
    } while (0)

/* Float comparison with tolerance */
#define IVP_ASSERT_NEAR(a, b, tol)                                         \
    do {                                                                   \
        double _a = (double)(a);                                           \
        double _b = (double)(b);                                           \
        double _t = (double)(tol);                                         \
        if (fabs(_a - _b) > _t) {                                         \
            fprintf(stderr,                                                \
                    "  ASSERT_NEAR failed: %s=%g, %s=%g, tol=%g (%s:%d)\n",\
                    #a, _a, #b, _b, _t, __FILE__, __LINE__);              \
            (*_fail_count)++;                                              \
            return;                                                        \
        }                                                                  \
    } while (0)

/* ── Test runner ────────────────────────────────────────────────────────── */

#define IVP_RUN_ALL()                                                      \
    do {                                                                   \
        int _total_pass = 0, _total_fail = 0;                              \
        printf("Running %d tests...\n", s_test_count);                     \
        for (int _i = 0; _i < s_test_count; _i++) {                       \
            int _fc = 0;                                                   \
            printf("[%d/%d] %s ... ", _i + 1, s_test_count,                \
                   s_tests[_i].name);                                      \
            fflush(stdout);                                                \
            s_tests[_i].fn(&_fc);                                          \
            if (_fc == 0) {                                                \
                printf("OK\n");                                            \
                _total_pass++;                                             \
            } else {                                                       \
                printf("FAILED (%d failures)\n", _fc);                     \
                _total_fail++;                                             \
            }                                                              \
        }                                                                  \
        printf("\n%d passed, %d failed, %d total\n",                       \
               _total_pass, _total_fail, s_test_count);                    \
        return _total_fail > 0 ? 1 : 0;                                   \
    } while (0)

/* ── Test registration macro ────────────────────────────────────────────── */

/* Call this in a function to register a test by name. */
#define IVP_REGISTER(name) ivp_register_test(#name, ivp_test_##name)

#endif /* IVP_TEST_H */
