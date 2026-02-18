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

static inline ivp_object_handle_t ivp_object_handle_invalid(void) {
    ivp_object_handle_t h;
    h.slot = UINT32_MAX;
    h.generation = 0;
    return h;
}

static inline bool ivp_object_handle_is_valid(ivp_object_handle_t h) {
    return h.slot != UINT32_MAX && h.generation != 0;
}

static inline ivp_controller_handle_t ivp_controller_handle_invalid(void) {
    ivp_controller_handle_t h;
    h.slot = UINT32_MAX;
    h.generation = 0;
    return h;
}

static inline bool ivp_controller_handle_is_valid(ivp_controller_handle_t h) {
    return h.slot != UINT32_MAX && h.generation != 0;
}

static inline ivp_event_listener_handle_t ivp_event_listener_handle_invalid(void) {
    ivp_event_listener_handle_t h;
    h.slot = UINT32_MAX;
    h.generation = 0;
    return h;
}

static inline bool ivp_event_listener_handle_is_valid(ivp_event_listener_handle_t h) {
    return h.slot != UINT32_MAX && h.generation != 0;
}

#endif
