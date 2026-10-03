/* ivp_statistics_console_callback.c — port of ivp_physics/ivp_stat_manager_cback_con.cxx
 * (IVP_Statisticsmanager_Console_Callback)
 */
#include <ivp/diagnostics/ivp_better_statistics_manager.h>
#include <ivp/base/ivp_message.h>

#include <stdarg.h>

/* reference: printf.  stdout (the default) or NULL: the engine output
 * (ivp_debug_printf); any other FILE: fprintf to it */
static void console_printf(const ivp_statistics_console_callback_t *cb, const char *fmt, ...) IVP_PRINTF_FMT(2, 3);
static void console_printf(const ivp_statistics_console_callback_t *cb, const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    if (!cb->out || cb->out == stdout) {
        ivp_debug_vprintf(fmt, args);
    } else {
        vfprintf(cb->out, fmt, args);
    }
    va_end(args);
}

static void console_output_request(ivp_better_statistics_callback_t *base, ivp_better_statistics_data_entity_t *entity)
{
    ivp_statistics_console_callback_t *cb = (ivp_statistics_console_callback_t *)base;
    const char *text = entity->text ? entity->text : "";

    switch (entity->type) {
    case IVP_BSM_INT_VALUE:
        console_printf(cb, "%s%d\n", text, entity->data.int_value);
        break;
    case IVP_BSM_DOUBLE_VALUE:
        console_printf(cb, "%s%f\n", text, (double)entity->data.double_value);
        break;
    case IVP_BSM_INT_ARRAY:
        console_printf(cb, "%s\n", text);
        for (int i = 0; i < entity->data.int_array.size; i++) {
            int value = entity->data.int_array.array[i];
            console_printf(cb, "%d\n", value);
        }
        break;
    case IVP_BSM_DOUBLE_ARRAY:
        console_printf(cb, "%s\n", text);
        for (int i = 0; i < entity->data.double_array.size; i++) {
            ivp_real_t value = entity->data.double_array.array[i];
            console_printf(cb, "%f\n", (double)value);
        }
        break;
    case IVP_BSM_STRING:
        console_printf(cb, "%s\n", text);
        break;
    }
}

static void console_enable(ivp_better_statistics_callback_t *base)
{
    (void)base;
}

static void console_disable(ivp_better_statistics_callback_t *base)
{
    (void)base;
}

static const ivp_better_statistics_callback_vtable_t s_console_vtable = {
    .output_request = console_output_request,
    .enable = console_enable,
    .disable = console_disable,
};

void ivp_statistics_console_callback_init(ivp_statistics_console_callback_t *cb)
{
    if (!cb) return;
    cb->base.vt = &s_console_vtable;
    cb->out = stdout;
}
