/* ivp_assert.h — Debug assertions for IVP C17
 *
 * Controlled by IVP_ENABLE_ASSERTS (set in Debug builds or explicitly).
 * Custom handler can be installed at runtime via ivp_set_assert_handler().
 */
#ifndef IVP_ASSERT_H
#define IVP_ASSERT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Assert handler callback type ───────────────────────────────────────── */

typedef void (*ivp_assert_handler_t)(
    const char *expr,
    const char *file,
    int         line,
    const char *func,
    const char *msg,
    void       *userdata
);

/* Set a custom assert handler. Pass NULL to restore default (stderr + trap). */
void ivp_set_assert_handler(ivp_assert_handler_t handler, void *userdata);

/* Internal: called by IVP_ASSERT macro. Returns true if should trap. */
bool ivp_assert_fail(const char *expr, const char *file, int line,
                     const char *func, const char *msg);

/* Internal: called by IVP_ASSERT_NONFATAL.  Passes the failure to a custom
 * handler (if one is installed) and never traps. */
void ivp_assert_report(const char *expr, const char *file, int line,
                       const char *func);

#ifdef __cplusplus
}
#endif

/* ── Assert macros ──────────────────────────────────────────────────────── */

/* Platform-specific debug trap */
#if defined(_MSC_VER)
    #define IVP_DEBUG_TRAP() __debugbreak()
#elif defined(__GNUC__) || defined(__clang__)
    #if defined(__has_builtin) && __has_builtin(__builtin_trap)
        #define IVP_DEBUG_TRAP() __builtin_trap()
    #else
        #include <signal.h>
        #define IVP_DEBUG_TRAP() raise(SIGTRAP)
    #endif
#else
    #include <stdlib.h>
    #define IVP_DEBUG_TRAP() abort()
#endif

/* CORE (reference ivu_types.hxx: raise(SIGINT), in every build): report
 * through the assert handler (expression "CORE") and trap unless a custom
 * handler consumed it. */
#define IVP_CORE()                                                         \
    do {                                                                   \
        if (ivp_assert_fail("CORE", __FILE__, __LINE__,                    \
                            __func__, (const char*)0)) {                   \
            IVP_DEBUG_TRAP();                                              \
        }                                                                  \
    } while (0)

#ifdef IVP_ENABLE_ASSERTS

    #define IVP_ASSERT(expr)                                               \
        do {                                                               \
            if (!(expr)) {                                                 \
                if (ivp_assert_fail(#expr, __FILE__, __LINE__,             \
                                    __func__, (const char*)0)) {           \
                    IVP_DEBUG_TRAP();                                       \
                }                                                          \
            }                                                              \
        } while (0)

    #define IVP_ASSERT_MSG(expr, msg)                                      \
        do {                                                               \
            if (!(expr)) {                                                 \
                if (ivp_assert_fail(#expr, __FILE__, __LINE__,             \
                                    __func__, (msg))) {                    \
                    IVP_DEBUG_TRAP();                                       \
                }                                                          \
            }                                                              \
        } while (0)

    /* Reference IVP_ASSERTs that also fail on valid data (the reference debug
     * build raises SIGINT there, e.g. the 16-byte alignment check on compact
     * grid ledge allocations): reported to a custom assert handler like any
     * assert, but never trap and print nothing by default. */
    #define IVP_ASSERT_NONFATAL(expr)                                      \
        do {                                                               \
            if (!(expr)) {                                                 \
                ivp_assert_report(#expr, __FILE__, __LINE__, __func__);    \
            }                                                              \
        } while (0)

    /* IVP_IF (reference ivu_types.hxx): the debug-only code of the reference
     * engine.  Active exactly when IVP_ASSERT is (Debug builds or
     * IVP_ENABLE_ASSERTS), like the reference IVP_IF / IVP_ASSERT pair
     * (NDEBUG not defined).  The reference debug build also defines DEBUG:
     * its "#ifdef DEBUG" code is gated on IVP_ENABLE_ASSERTS as well
     * (IVP_DEBUG_BUILD). */
    #define IVP_IF(flag) if (flag)
    #define IVP_DEBUG_BUILD 1

#else /* !IVP_ENABLE_ASSERTS */

    #define IVP_ASSERT(expr)          ((void)0)
    #define IVP_ASSERT_MSG(expr, msg) ((void)0)
    #define IVP_ASSERT_NONFATAL(expr) ((void)0)

    /* reference release build: "if (0 == 1)".  The flag is still referenced
     * (never evaluated) so that variables only used as IVP_IF conditions do
     * not trigger unused warnings; the block is compiled but dead. */
    #define IVP_IF(flag) if ((0 == 1) && (flag))

#endif /* IVP_ENABLE_ASSERTS */

/* IVP_USE (reference: "a = a" in debug builds, nothing in release) */
#define IVP_USE(a) ((void)(a))

#endif /* IVP_ASSERT_H */
