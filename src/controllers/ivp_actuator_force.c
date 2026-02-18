/* ivp_actuator_force.c -- Anchor-based force actuator implementation
 *
 * Port of IVP_Actuator_Force from the legacy SDK.
 *
 * Applies a constant force along the line connecting two anchor points.
 * The force direction is from anchor_a toward anchor_b (attraction).
 * Negative force values push objects apart (repulsion).
 *
 * Force is applied at the anchor points, producing both linear and angular
 * effects when anchors are offset from the center of mass.
 */
#include <ivp/controllers/ivp_actuator_force.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>

#include <string.h>

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_force_actuator_state_init(ivp_force_actuator_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->active = true;
    state->push_first_object = true;
    state->push_second_object = true;
}

void ivp_force_actuator_state_setup(ivp_force_actuator_state_t *state,
                                     ivp_object_t *obj_a,
                                     const ivp_vec3r_t *anchor_a_os,
                                     ivp_object_t *obj_b,
                                     const ivp_vec3r_t *anchor_b_os,
                                     ivp_real_t force,
                                     bool push_first,
                                     bool push_second)
{
    memset(state, 0, sizeof(*state));
    state->obj_a = obj_a;
    state->obj_b = obj_b;
    state->anchor_a_os = *anchor_a_os;
    state->anchor_b_os = *anchor_b_os;
    state->force = force;
    state->push_first_object = push_first;
    state->push_second_object = push_second;
    state->active = true;
}

void ivp_force_actuator_set_force(ivp_force_actuator_state_t *state,
                                    ivp_real_t force)
{
    state->force = force;
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_force_actuator(const ivp_event_sim_t *event_sim,
                                     ivp_object_t *obj,
                                     void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;

    ivp_force_actuator_state_t *state = (ivp_force_actuator_state_t *)ctx;
    if (!state->active) return;
    if (ivp_fabs(state->force) < IVP_REAL_EPS) return;
    if (dt < IVP_REAL_EPS) return;

    /* Environment dispatches controllers once per active object.
     * Execute only once per PSI on the first dynamic endpoint. */
    ivp_object_t *exec_obj = NULL;
    if (state->obj_a && !state->obj_a->core.physical_unmoveable) {
        exec_obj = state->obj_a;
    } else if (state->obj_b && !state->obj_b->core.physical_unmoveable) {
        exec_obj = state->obj_b;
    }
    if (!exec_obj || obj != exec_obj) return;

    /* Compute world-space anchor positions */
    ivp_vec3r_t pa_ws, pb_ws;
    ivp_controller_get_anchor_ws(state->obj_a, &state->anchor_a_os, &pa_ws);
    ivp_controller_get_anchor_ws(state->obj_b, &state->anchor_b_os, &pb_ws);

    /* Direction from A to B */
    ivp_vec3r_t dir;
    ivp_vec3r_sub(&dir, &pb_ws, &pa_ws);
    ivp_real_t dist = ivp_vec3r_length(&dir);
    state->current_distance = dist;

    if (dist < IVP_REAL_EPS) return;

    ivp_vec3r_scale(&dir, &dir, IVP_REAL(1.0) / dist);

    /* Positive force: attract (A pushed toward B, B pushed toward A).
     * Apply at anchor points for both linear + angular effects. */

    if (state->push_first_object && state->obj_a) {
        ivp_vec3r_t force_on_a;
        ivp_vec3r_scale(&force_on_a, &dir, state->force);
        ivp_controller_apply_force_at_anchor(state->obj_a, &pa_ws, &force_on_a, dt);
    }

    if (state->push_second_object && state->obj_b) {
        ivp_vec3r_t force_on_b;
        ivp_vec3r_scale(&force_on_b, &dir, -state->force);
        ivp_controller_apply_force_at_anchor(state->obj_b, &pb_ws, &force_on_b, dt);
    }
}
