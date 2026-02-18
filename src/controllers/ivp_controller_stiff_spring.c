/* ivp_controller_stiff_spring.c -- Stiff spring controller implementation
 *
 * Uses implicit (backward) Euler to solve the spring equation:
 *   m * dv/dt = -k * x - c * v
 *
 * Implicit Euler rearranges to:
 *   dv = -(k * x * dt + c * v_rel) / (m_eff + c * dt + k * dt^2)
 *
 * This is unconditionally stable for any stiffness k.
 */

#include <ivp/controllers/ivp_controller_stiff_spring.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_stiff_spring_state_init(ivp_stiff_spring_state_t *state)
{
    memset(state, 0, sizeof(*state));
    state->stiffness   = IVP_REAL(1000.0);
    state->damping     = IVP_REAL(10.0);
    state->rest_length = IVP_REAL(1.0);
    state->active      = true;
}

void ivp_stiff_spring_state_setup(ivp_stiff_spring_state_t *state,
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
    state->max_dv      = IVP_REAL(0.0);
    state->active      = true;
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_stiff_spring(const ivp_event_sim_t *event_sim,
                                   ivp_object_t *obj,
                                   void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;
    ivp_stiff_spring_state_t *state = (ivp_stiff_spring_state_t *)ctx;
    if (!state || !state->active) return;

    /* Only execute once per PSI step */
    if (state->obj_a && obj != state->obj_a) return;
    if (!state->obj_a && state->obj_b && obj != state->obj_b) return;

    /* Get anchor world positions */
    ivp_vec3r_t pos_a_ws, pos_b_ws;
    ivp_controller_get_anchor_ws(state->obj_a, &state->anchor_a_os, &pos_a_ws);
    ivp_controller_get_anchor_ws(state->obj_b, &state->anchor_b_os, &pos_b_ws);

    /* Spring vector: A → B */
    ivp_vec3r_t diff;
    ivp_vec3r_sub(&diff, &pos_b_ws, &pos_a_ws);

    ivp_real_t length = ivp_vec3r_length(&diff);
    state->current_length = length;

    if (length < IVP_REAL_EPS) return;

    /* Normalized direction */
    ivp_vec3r_t dir;
    ivp_real_t inv_len = IVP_REAL(1.0) / length;
    ivp_vec3r_scale(&dir, &diff, inv_len);

    /* Displacement from rest length */
    ivp_real_t x = length - state->rest_length;

    /* Relative velocity along spring direction */
    ivp_real_t v_rel = IVP_REAL(0.0);
    if (state->obj_a && state->obj_b) {
        ivp_vec3r_t dv;
        ivp_vec3r_sub(&dv, &state->obj_b->core.speed,
                      &state->obj_a->core.speed);
        v_rel = ivp_vec3r_dot(&dv, &dir);
    } else if (state->obj_a) {
        v_rel = -ivp_vec3r_dot(&state->obj_a->core.speed, &dir);
    } else if (state->obj_b) {
        v_rel = ivp_vec3r_dot(&state->obj_b->core.speed, &dir);
    }

    /* Effective inverse mass */
    ivp_real_t inv_m_eff = IVP_REAL(0.0);
    bool a_moveable = state->obj_a && !state->obj_a->core.physical_unmoveable;
    bool b_moveable = state->obj_b && !state->obj_b->core.physical_unmoveable;

    if (a_moveable) inv_m_eff += state->obj_a->core.inv_mass;
    if (b_moveable) inv_m_eff += state->obj_b->core.inv_mass;

    if (inv_m_eff < IVP_REAL_EPS) return;

    /* Implicit (backward) Euler velocity update:
     *
     *   v' = v + (dt/m) * (-k*(x + dt*v') - c*v')
     *
     * Solving for dv = v' - v:
     *   dv = -(k*x*dt + (c + k*dt)*dt*v) * inv_m / (1 + (c + k*dt)*dt*inv_m)
     *
     * Let gamma = (c + k*dt)*dt, then:
     *   softness = 1 / (1 + gamma * inv_m_eff)
     *   dv = -(k*x*dt + gamma*v_rel) * inv_m_eff * softness
     */
    ivp_real_t gamma = (state->damping + state->stiffness * dt) * dt;
    ivp_real_t denom = IVP_REAL(1.0) + gamma * inv_m_eff;

    ivp_real_t softness = IVP_REAL(1.0) / denom;

    /* Total velocity change along spring direction (for the effective mass) */
    ivp_real_t impulse = -(state->stiffness * x * dt + gamma * v_rel)
                         * inv_m_eff * softness;

    /* Clamp velocity change */
    if (state->max_dv > IVP_REAL(0.0)) {
        if (impulse >  state->max_dv) impulse =  state->max_dv;
        if (impulse < -state->max_dv) impulse = -state->max_dv;
    }

    /* Apply velocity changes directly (implicit integration = velocity level) */
    if (a_moveable) {
        ivp_core_t *ca = &state->obj_a->core;
        ivp_real_t ratio_a = ca->inv_mass * (IVP_REAL(1.0) / inv_m_eff);
        ivp_real_t dva = -impulse * ratio_a;
        ca->speed_change.k[0] += dir.k[0] * dva;
        ca->speed_change.k[1] += dir.k[1] * dva;
        ca->speed_change.k[2] += dir.k[2] * dva;
    }

    if (b_moveable) {
        ivp_core_t *cb = &state->obj_b->core;
        ivp_real_t ratio_b = cb->inv_mass * (IVP_REAL(1.0) / inv_m_eff);
        ivp_real_t dvb = impulse * ratio_b;
        cb->speed_change.k[0] += dir.k[0] * dvb;
        cb->speed_change.k[1] += dir.k[1] * dvb;
        cb->speed_change.k[2] += dir.k[2] * dvb;
    }
}
