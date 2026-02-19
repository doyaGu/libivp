#ifndef IVP_HANDLE_H
#define IVP_HANDLE_H

#include <ivp/base/ivp_types.h>

typedef struct ivp_handle {
    uint32_t slot;
    uint32_t generation;
} ivp_handle_t;

typedef ivp_handle_t ivp_object_handle_t;
typedef ivp_handle_t ivp_controller_handle_t;
typedef ivp_handle_t ivp_event_listener_handle_t;

/* Generic handle operations (all handle type aliases share the same layout). */
static inline ivp_handle_t ivp_handle_invalid(void) {
    ivp_handle_t h;
    h.slot = UINT32_MAX;
    h.generation = 0;
    return h;
}

static inline bool ivp_handle_is_valid(ivp_handle_t h) {
    return h.slot != UINT32_MAX && h.generation != 0;
}

/* Type-specific convenience wrappers (thin aliases). */
static inline ivp_object_handle_t ivp_object_handle_invalid(void) {
    return ivp_handle_invalid();
}
static inline bool ivp_object_handle_is_valid(ivp_object_handle_t h) {
    return ivp_handle_is_valid(h);
}
static inline ivp_controller_handle_t ivp_controller_handle_invalid(void) {
    return ivp_handle_invalid();
}
static inline bool ivp_controller_handle_is_valid(ivp_controller_handle_t h) {
    return ivp_handle_is_valid(h);
}
static inline ivp_event_listener_handle_t ivp_event_listener_handle_invalid(void) {
    return ivp_handle_invalid();
}
static inline bool ivp_event_listener_handle_is_valid(ivp_event_listener_handle_t h) {
    return ivp_handle_is_valid(h);
}

#endif
