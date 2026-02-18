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

#ifdef __cplusplus
}
#endif

/* ── Assert macros ──────────────────────────────────────────────────────── */

#ifdef IVP_ENABLE_ASSERTS

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

#else /* !IVP_ENABLE_ASSERTS */

    #define IVP_ASSERT(expr)          ((void)0)
    #define IVP_ASSERT_MSG(expr, msg) ((void)0)

#endif /* IVP_ENABLE_ASSERTS */

#endif /* IVP_ASSERT_H */
