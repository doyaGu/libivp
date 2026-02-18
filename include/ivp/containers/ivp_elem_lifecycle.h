/* ivp_elem_lifecycle.h — Unified element lifecycle hooks for containers */
#ifndef IVP_ELEM_LIFECYCLE_H
#define IVP_ELEM_LIFECYCLE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*ivp_elem_init_func_t)(void *elem, void *user_data);
typedef void (*ivp_elem_reset_func_t)(void *elem, void *user_data);
typedef void (*ivp_elem_copy_func_t)(void *elem, int src_index, int dst_index,
                                      void *user_data);
typedef void (*ivp_elem_move_func_t)(void *elem, int src_index, int dst_index,
                                      void *user_data);
typedef void (*ivp_elem_dispose_func_t)(void *elem, void *user_data);

typedef struct ivp_elem_lifecycle {
    ivp_elem_init_func_t    init;
    ivp_elem_reset_func_t   reset;
    ivp_elem_copy_func_t    copy;
    ivp_elem_move_func_t    move;
    ivp_elem_dispose_func_t dispose;
} ivp_elem_lifecycle_t;

#ifdef __cplusplus
}
#endif

#endif /* IVP_ELEM_LIFECYCLE_H */
