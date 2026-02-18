#ifndef IVP_EXAMPLE_INSPECT_H
#define IVP_EXAMPLE_INSPECT_H

#include <stdbool.h>

typedef struct ivp_example_inspector {
    const char *tag;
    bool enabled;
    int frame_counter;
    int interval;
} ivp_example_inspector_t;

void ivp_example_inspector_init(ivp_example_inspector_t *inspector,
                                const char *tag);

bool ivp_example_inspector_tick(ivp_example_inspector_t *inspector);

void ivp_example_inspector_log(ivp_example_inspector_t *inspector,
                               const char *fmt, ...);

#endif /* IVP_EXAMPLE_INSPECT_H */
