/* ivp_actuator_check_dist.c -- Distance checking actuator implementation
 *
 * Port of IVP_Actuator_Check_Dist from the legacy SDK.
 *
 * Monitors the distance between two points (optionally bound to objects)
 * and reports when the distance crosses a threshold.
 */
#include <ivp/controllers/ivp_actuator_check_dist.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>

#include <string.h>

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_check_dist_state_init(ivp_check_dist_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->active = true;
    state->is_outside = 1;
    state->prev_is_outside = 1;
    state->use_object_centers = true;
}

void ivp_check_dist_state_setup(ivp_check_dist_state_t *state,
                                  ivp_object_t *obj_a,
                                  const ivp_vec3r_t *pos_a_ws,
                                  ivp_object_t *obj_b,
                                  const ivp_vec3r_t *pos_b_ws,
                                  ivp_real_t range)
{
    memset(state, 0, sizeof(*state));
    state->obj_a = obj_a;
    state->obj_b = obj_b;
    state->position_a_ws = *pos_a_ws;
    state->position_b_ws = *pos_b_ws;
    state->range = range;
    state->active = true;
    state->is_outside = 1;
    state->prev_is_outside = 1;
    state->use_object_centers = false;
}

void ivp_check_dist_set_callback(ivp_check_dist_state_t *state,
                                   ivp_check_dist_event_fn_t fn,
                                   void *ctx)
{
    state->event_fn = fn;
    state->event_ctx = ctx;
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_check_dist(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx)
{
    (void)obj;

    ivp_check_dist_state_t *state = (ivp_check_dist_state_t *)ctx;
    if (!state->active) return;

    /* Determine positions */
    ivp_vec3r_t pa, pb;

    if (state->use_object_centers) {
        /* Use object centers of mass (world space) */
        if (state->obj_a) {
            pa = state->obj_a->core.pos_world_f_core_last_psi;
        } else {
            pa = state->position_a_ws;
        }
        if (state->obj_b) {
            pb = state->obj_b->core.pos_world_f_core_last_psi;
        } else {
            pb = state->position_b_ws;
        }
    } else {
        /* Use supplied world-space positions.
         * If objects are set, offset by object movement since setup.
         * For simplicity, we use the stored world-space positions
         * which should have been set based on object transforms. */
        pa = state->position_a_ws;
        pb = state->position_b_ws;

        /* If bound to moving objects, recompute from centers */
        if (state->obj_a) {
            pa = state->obj_a->core.pos_world_f_core_last_psi;
        }
        if (state->obj_b) {
            pb = state->obj_b->core.pos_world_f_core_last_psi;
        }
    }

    /* Compute distance */
    ivp_vec3r_t diff;
    ivp_vec3r_sub(&diff, &pb, &pa);
    state->current_distance = ivp_vec3r_length(&diff);

    /* Update status */
    state->prev_is_outside = state->is_outside;
    state->is_outside = (state->current_distance > state->range) ? 1 : 0;

    /* Fire event on state change */
    if (state->is_outside != state->prev_is_outside && state->event_fn) {
        state->event_fn(state->event_ctx,
                          state->obj_a,
                          state->obj_b,
                          state->current_distance,
                          state->is_outside == 0);
    }
}
