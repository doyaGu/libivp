/* ivp_actuator_check_dist.h -- Distance checking actuator
 *
 * Monitors distance between two objects and fires events when
 * distance crosses a threshold.  The result is exposed via an
 * integer state value: 1 = inside range, 0 = outside range.
 *
 * Ported from IVP_Actuator_Check_Dist (ivp_actuator.hxx).
 */
#ifndef IVP_ACTUATOR_CHECK_DIST_H
#define IVP_ACTUATOR_CHECK_DIST_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Callback when distance status changes */
typedef void (*ivp_check_dist_event_fn_t)(void *ctx,
                                           ivp_object_t *obj_a,
                                           ivp_object_t *obj_b,
                                           ivp_real_t distance,
                                           bool is_inside_range);

/* ── Check distance state ───────────────────────────────────────────────── */

typedef struct ivp_check_dist_state {
    /* Objects to monitor */
    ivp_object_t *obj_a;
    ivp_object_t *obj_b;

    /* Positions in world space (set at creation time, can be updated) */
    ivp_vec3r_t position_a_ws;
    ivp_vec3r_t position_b_ws;

    /* Configuration */
    ivp_real_t range;                   /* distance threshold */
    bool       use_object_centers;      /* if true, always use object CoM */

    /* Result state */
    ivp_real_t current_distance;
    int        is_outside;              /* 1 = outside range, 0 = inside */
    int        prev_is_outside;         /* for change detection */

    /* Optional event callback */
    ivp_check_dist_event_fn_t event_fn;
    void                     *event_ctx;

    bool       active;
} ivp_check_dist_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_check_dist_state_init(ivp_check_dist_state_t *state);

void ivp_check_dist_state_setup(ivp_check_dist_state_t *state,
                                  ivp_object_t *obj_a,
                                  const ivp_vec3r_t *pos_a_ws,
                                  ivp_object_t *obj_b,
                                  const ivp_vec3r_t *pos_b_ws,
                                  ivp_real_t range);

/* Set event callback for distance threshold crossing */
void ivp_check_dist_set_callback(ivp_check_dist_state_t *state,
                                   ivp_check_dist_event_fn_t fn,
                                   void *ctx);

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_check_dist(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTUATOR_CHECK_DIST_H */
