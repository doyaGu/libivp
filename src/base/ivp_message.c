/* ivp_message.c — diagnostics text output (see ivp_message.h) */
#include <ivp/base/ivp_message.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static ivp_message_output_fn_t s_output   = NULL;
static void                   *s_userdata = NULL;

void ivp_set_message_output(ivp_message_output_fn_t fn, void *userdata)
{
    s_output   = fn;
    s_userdata = userdata;
}

static void ivp_message_emit(const char *text)
{
    if (s_output) {
        s_output(text, s_userdata);
    } else {
        fputs(text, stdout);
    }
}

void ivp_debug_vprintf(const char *fmt, va_list args)
{
    char    buffer[4096];
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(buffer, sizeof(buffer), fmt, copy);
    va_end(copy);
    if (n < 0) return;
    if ((size_t)n < sizeof(buffer)) {
        ivp_message_emit(buffer);
        return;
    }
    /* longer than the stack buffer: printf has no limit */
    char *big = (char *)malloc((size_t)n + 1);
    if (!big) return;
    vsnprintf(big, (size_t)n + 1, fmt, args);
    ivp_message_emit(big);
    free(big);
}

void ivp_debug_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    ivp_debug_vprintf(fmt, args);
    va_end(args);
}

#define IVP_MAX_ERROR_BUFFER_LEN 10000 /* ivu_string.cxx MAX_ERROR_BUFFER_LEN */

void ivp_message(const char *fmt, ...)
{
    char    buffer_tmp[IVP_MAX_ERROR_BUFFER_LEN];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer_tmp, sizeof(buffer_tmp), fmt, args);
    va_end(args);

    static const char prefix[] = "[havok] ";
    char buffer[IVP_MAX_ERROR_BUFFER_LEN];
    const size_t prefix_len = sizeof(prefix) - 1;
    const size_t max_text_len = sizeof(buffer) - sizeof(prefix);
    size_t text_len = strlen(buffer_tmp);
    if (text_len > max_text_len) text_len = max_text_len;
    memcpy(buffer, prefix, prefix_len);
    memcpy(buffer + prefix_len, buffer_tmp, text_len);
    buffer[prefix_len + text_len] = '\0';
    ivp_message_emit(buffer);
}
