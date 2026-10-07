/* ivp_assert.c — Assert handler implementation */
#include <ivp/base/ivp_assert.h>
#include <stdio.h>
#include <stdatomic.h>

/* ── Global handler state ───────────────────────────────────────────────── */

static ivp_assert_handler_t s_handler  = NULL;
static void                *s_userdata = NULL;
static atomic_flag          s_handler_lock = ATOMIC_FLAG_INIT;

static void handler_lock(void) {
    while (atomic_flag_test_and_set_explicit(&s_handler_lock,
                                              memory_order_acquire)) {
    }
}

static void handler_unlock(void) {
    atomic_flag_clear_explicit(&s_handler_lock, memory_order_release);
}

static ivp_assert_handler_t handler_snapshot(void **userdata) {
    ivp_assert_handler_t handler;
    handler_lock();
    handler = s_handler;
    *userdata = s_userdata;
    handler_unlock();
    return handler;
}

void ivp_set_assert_handler(ivp_assert_handler_t handler, void *userdata) {
    handler_lock();
    s_handler  = handler;
    s_userdata = userdata;
    handler_unlock();
}

void ivp_assert_report(const char *expr, const char *file, int line,
                       const char *func) {
    void *userdata;
    ivp_assert_handler_t handler = handler_snapshot(&userdata);
    if (handler) handler(expr, file, line, func, NULL, userdata);
}

bool ivp_assert_fail(const char *expr, const char *file, int line,
                     const char *func, const char *msg) {
    void *userdata;
    ivp_assert_handler_t handler = handler_snapshot(&userdata);
    if (handler) {
        handler(expr, file, line, func, msg, userdata);
        return false; /* custom handler consumed the assert */
    }

    /* Default: print to stderr and request trap */
    fprintf(stderr, "IVP ASSERT FAILED: %s\n", expr);
    if (msg) {
        fprintf(stderr, "  Message: %s\n", msg);
    }
    fprintf(stderr, "  File: %s:%d\n", file, line);
    fprintf(stderr, "  Function: %s\n", func);
    return true; /* request debug trap */
}
