/* ivp_actuator_spring.c -- Spring force actuator implementation */

#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_spring_state_init(ivp_spring_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->stiffness   = IVP_REAL(100.0);
    state->damping     = IVP_REAL(1.0);
    state->rest_length = IVP_REAL(1.0);
}

void ivp_spring_state_setup(ivp_spring_state_t *state,
                              ivp_object_t *obj_a,
                              const ivp_vec3r_t *anchor_a_os,
                              ivp_object_t *obj_b,
                              const ivp_vec3r_t *anchor_b_os,
                              ivp_real_t stiffness,
                              ivp_real_t damping,
                              ivp_real_t rest_length)
{
    state->obj_a       = obj_a;
    state->obj_b       = obj_b;
    state->anchor_a_os = *anchor_a_os;
    state->anchor_b_os = *anchor_b_os;
    state->stiffness   = stiffness;
    state->damping     = damping;
    state->rest_length = rest_length;
    state->max_force   = IVP_REAL(0.0);
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_spring(const ivp_event_sim_t *event_sim,
                             ivp_object_t *obj,
                             void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;
    (void)obj; /* Spring acts on its own two objects, not the iteration object */

    ivp_spring_state_t *state = (ivp_spring_state_t *)ctx;
    if (!state) return;

    /* Only execute once per PSI on the first dynamic endpoint.
     * This keeps behavior deterministic and still works when obj_a is static. */
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

    /* Spring direction: A → B */
    ivp_vec3r_t diff;
    ivp_vec3r_sub(&diff, &pos_b_ws, &pos_a_ws);

    ivp_real_t length = ivp_vec3r_length(&diff);
    state->current_length = length;

    if (length < IVP_REAL_EPS) {
        state->current_force = IVP_REAL(0.0);
        return; /* degenerate: anchors coincide */
    }

    /* Normalized direction */
    ivp_vec3r_t dir;
    ivp_real_t inv_len = IVP_REAL(1.0) / length;
    ivp_vec3r_scale(&dir, &diff, inv_len);

    /* Hooke's law: F = -k * (length - rest_length) */
    ivp_real_t stretch = length - state->rest_length;
    ivp_real_t spring_force = -state->stiffness * stretch;

    /* Damping: F_damp = -c * relative_velocity_along_spring */
    ivp_vec3r_t vel_a_ws, vel_b_ws;
    ivp_controller_get_point_velocity_ws(state->obj_a, &pos_a_ws, &vel_a_ws);
    ivp_controller_get_point_velocity_ws(state->obj_b, &pos_b_ws, &vel_b_ws);

    ivp_vec3r_t v_rel;
    ivp_vec3r_sub(&v_rel, &vel_b_ws, &vel_a_ws);
    ivp_real_t closing_velocity = ivp_vec3r_dot(&v_rel, &dir);

    ivp_real_t damp_force = -state->damping * closing_velocity;

    /* Total force magnitude (positive = pull toward rest length) */
    ivp_real_t total_force = spring_force + damp_force;

    /* Clamp force */
    if (state->max_force > IVP_REAL(0.0)) {
        if (total_force > state->max_force)  total_force = state->max_force;
        if (total_force < -state->max_force) total_force = -state->max_force;
    }

    state->current_force = total_force;

    /* Force vector: positive total_force means stretch needs to contract,
     * so force on A is toward B (+dir) and force on B is toward A (-dir).
     * But our spring_force = -k*stretch where stretch>0 means extended,
     * so negative spring_force means pull together (force on A = +dir * |F|). */
    ivp_vec3r_t force_on_a;
    ivp_vec3r_scale(&force_on_a, &dir, -total_force);

    /* Apply at anchor points (includes linear + angular effects) */
    ivp_controller_apply_force_at_anchor(state->obj_a, &pos_a_ws, &force_on_a, dt);

    ivp_vec3r_t force_on_b;
    ivp_vec3r_scale(&force_on_b, &force_on_a, IVP_REAL(-1.0));
    ivp_controller_apply_force_at_anchor(state->obj_b, &pos_b_ws, &force_on_b, dt);
}
