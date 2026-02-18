/* ivp_actuator_suspension.c -- Suspension spring implementation
 *
 * Spring with asymmetric damping (compression vs extension) and
 * optional body force clamping. Wheel receives unclamped force.
 */

#include <ivp/controllers/ivp_actuator_suspension.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>

/* ── Initialization ────────────────────────────────────────────────────── */

void ivp_suspension_state_init(ivp_suspension_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->stiffness           = IVP_REAL(100.0);
    state->damping             = IVP_REAL(1.0);
    state->damping_compression = IVP_REAL(1.0);
    state->rest_length         = IVP_REAL(1.0);
    state->max_body_force      = IVP_REAL(0.0); /* unlimited */
}

void ivp_suspension_state_setup(ivp_suspension_state_t *state,
                                 ivp_object_t *body,
                                 ivp_object_t *wheel,
                                 const ivp_vec3r_t *anchor_body_os,
                                 const ivp_vec3r_t *anchor_wheel_os,
                                 ivp_real_t stiffness,
                                 ivp_real_t damping,
                                 ivp_real_t damping_compression,
                                 ivp_real_t rest_length)
{
    state->obj_a              = body;
    state->obj_b              = wheel;
    state->anchor_a_os        = *anchor_body_os;
    state->anchor_b_os        = *anchor_wheel_os;
    state->stiffness          = stiffness;
    state->damping            = damping;
    state->damping_compression = damping_compression;
    state->rest_length        = rest_length;
}

/* ── Runtime adjustment ────────────────────────────────────────────────── */

void ivp_suspension_set_damping_compression(ivp_suspension_state_t *state,
                                             ivp_real_t value)
{
    state->damping_compression = value;
}

void ivp_suspension_set_max_body_force(ivp_suspension_state_t *state,
                                        ivp_real_t value)
{
    state->max_body_force = value;
}

/* ── Controller callback ───────────────────────────────────────────────── */

void ivp_controller_suspension(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;

    ivp_suspension_state_t *state = (ivp_suspension_state_t *)ctx;
    if (!state) return;

    /* Execute-once guard on first dynamic endpoint */
    ivp_object_t *exec_obj = NULL;
    if (state->obj_a && !state->obj_a->core.physical_unmoveable) {
        exec_obj = state->obj_a;
    } else if (state->obj_b && !state->obj_b->core.physical_unmoveable) {
        exec_obj = state->obj_b;
    }
    if (!exec_obj || obj != exec_obj) return;

    /* Get anchor world positions */
    ivp_vec3r_t pos_a_ws, pos_b_ws;
    ivp_controller_get_anchor_ws(state->obj_a, &state->anchor_a_os, &pos_a_ws);
    ivp_controller_get_anchor_ws(state->obj_b, &state->anchor_b_os, &pos_b_ws);

    /* Spring direction: body(A) → wheel(B), same convention as ivp_actuator_spring */
    ivp_vec3r_t diff;
    ivp_vec3r_sub(&diff, &pos_b_ws, &pos_a_ws);

    ivp_real_t length = ivp_vec3r_length(&diff);
    state->current_length = length;

    if (length < IVP_REAL_EPS) {
        state->current_force = IVP_REAL(0.0);
        state->current_body_force = IVP_REAL(0.0);
        return;
    }

    /* Normalized direction (body toward wheel) */
    ivp_vec3r_t dir;
    ivp_real_t inv_len = IVP_REAL(1.0) / length;
    ivp_vec3r_scale(&dir, &diff, inv_len);

    /* Hooke's law: F = -k * (length - rest_length) */
    ivp_real_t stretch = length - state->rest_length;
    ivp_real_t spring_force = -state->stiffness * stretch;

    /* Relative velocity along spring axis: v_rel = vel_wheel - vel_body */
    ivp_vec3r_t vel_a_ws, vel_b_ws;
    ivp_controller_get_point_velocity_ws(state->obj_a, &pos_a_ws, &vel_a_ws);
    ivp_controller_get_point_velocity_ws(state->obj_b, &pos_b_ws, &vel_b_ws);

    ivp_vec3r_t v_rel;
    ivp_vec3r_sub(&v_rel, &vel_b_ws, &vel_a_ws);
    ivp_real_t closing_velocity = ivp_vec3r_dot(&v_rel, &dir);

    /* Select damping based on compression vs extension:
     * closing_velocity < 0 means wheel approaching body = compression
     * closing_velocity > 0 means wheel separating from body = extension */
    ivp_real_t damp = (closing_velocity < IVP_REAL(0.0))
                    ? state->damping_compression
                    : state->damping;

    ivp_real_t damp_force = -damp * closing_velocity;
    ivp_real_t total_force = spring_force + damp_force;

    state->current_force = total_force;

    /* Force on body: dir * (-total_force).
     * When extended, total_force < 0, so -total_force > 0, body pushed toward wheel. */
    ivp_real_t body_force_mag = -total_force;

    /* Clamp body force magnitude */
    if (state->max_body_force > IVP_REAL(0.0)) {
        if (body_force_mag > state->max_body_force)
            body_force_mag = state->max_body_force;
        if (body_force_mag < -state->max_body_force)
            body_force_mag = -state->max_body_force;
    }

    state->current_body_force = body_force_mag;

    {
        ivp_vec3r_t force_on_body;
        ivp_vec3r_scale(&force_on_body, &dir, body_force_mag);
        ivp_controller_apply_force_at_anchor(state->obj_a, &pos_a_ws, &force_on_body, dt);
    }

    /* Force on wheel: opposite of body force but unclamped.
     * force_on_wheel = dir * total_force (negated body direction, unclamped). */
    {
        ivp_vec3r_t force_on_wheel;
        ivp_vec3r_scale(&force_on_wheel, &dir, total_force);
        ivp_controller_apply_force_at_anchor(state->obj_b, &pos_b_ws, &force_on_wheel, dt);
    }
}
