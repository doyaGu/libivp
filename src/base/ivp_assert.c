/* ivp_assert.c — Assert handler implementation */
#include <ivp/base/ivp_assert.h>
#include <stdio.h>

/* ── Global handler state ───────────────────────────────────────────────── */

static ivp_assert_handler_t s_handler  = NULL;
static void                *s_userdata = NULL;

void ivp_set_assert_handler(ivp_assert_handler_t handler, void *userdata) {
    s_handler  = handler;
    s_userdata = userdata;
}

bool ivp_assert_fail(const char *expr, const char *file, int line,
                     const char *func, const char *msg) {
    if (s_handler) {
        s_handler(expr, file, line, func, msg, s_userdata);
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
