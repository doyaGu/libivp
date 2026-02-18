#include "inspect.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

static int parse_interval_from_env(void)
{
    const char *value = getenv("IVP_DEBUG_EVERY");
    if (!value || !value[0]) return 30;

    int parsed = atoi(value);
    if (parsed < 1) return 30;
    return parsed;
}

void ivp_example_inspector_init(ivp_example_inspector_t *inspector,
                                const char *tag)
{
    if (!inspector) return;

    const char *debug = getenv("IVP_DEBUG");
    inspector->enabled = (debug && debug[0] && debug[0] != '0');
    inspector->frame_counter = 0;
    inspector->interval = parse_interval_from_env();
    inspector->tag = (tag && tag[0]) ? tag : "example";

    if (inspector->enabled) {
        setvbuf(stdout, NULL, _IONBF, 0);
    }
}

bool ivp_example_inspector_tick(ivp_example_inspector_t *inspector)
{
    if (!inspector || !inspector->enabled) return false;
    inspector->frame_counter++;
    return (inspector->frame_counter % inspector->interval) == 0;
}

void ivp_example_inspector_log(ivp_example_inspector_t *inspector,
                               const char *fmt, ...)
{
    if (!inspector || !inspector->enabled || !fmt) return;

    printf("[%s dbg] ", inspector->tag);

    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);

    putchar('\n');
}
