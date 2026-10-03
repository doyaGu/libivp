/* ivp_message.h — text output of the engine's diagnostics
 *
 * The reference engine prints its debug and statistic output with printf
 * (stdout) and ivp_message (ivp_utility/ivu_string.cxx: "[havok] " + text,
 * printf on every platform but WIN32).  libivp routes both through one
 * overridable output function so applications can redirect it; the default
 * writes the text unchanged to stdout like the reference.
 *
 * Nothing prints in release builds except where the reference prints in
 * its release build too: the debug output sits in IVP_IF blocks
 * (ivp/base/ivp_assert.h), active only in Debug / IVP_ENABLE_ASSERTS builds.
 */
#ifndef IVP_MESSAGE_H
#define IVP_MESSAGE_H

#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__GNUC__) || defined(__clang__)
    #define IVP_PRINTF_FMT(a, b) __attribute__((format(printf, a, b)))
#else
    #define IVP_PRINTF_FMT(a, b)
#endif

/* Receives every piece of text (already formatted, not necessarily a whole
 * line).  NULL restores the default (fputs to stdout). */
typedef void (*ivp_message_output_fn_t)(const char *text, void *userdata);
void ivp_set_message_output(ivp_message_output_fn_t fn, void *userdata);

/* The reference's printf(...) in debug code: the formatted text, unchanged. */
void ivp_debug_printf(const char *fmt, ...) IVP_PRINTF_FMT(1, 2);
void ivp_debug_vprintf(const char *fmt, va_list args) IVP_PRINTF_FMT(1, 0);

/* ivp_message (ivu_string.cxx): "[havok] " + formatted text (the reference
 * truncates the text to MAX_ERROR_BUFFER_LEN - 1 = 9999 characters). */
void ivp_message(const char *fmt, ...) IVP_PRINTF_FMT(1, 2);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MESSAGE_H */
