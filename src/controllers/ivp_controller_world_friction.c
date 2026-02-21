/* ivp_controller_world_friction.c -- World friction controller */

#include <ivp/controllers/ivp_controller_world_friction.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>
#include <string.h>

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_world_friction_init_ground(ivp_world_friction_state_t *state,
                                      const ivp_vec3r_t *normal,
                                      ivp_real_t height,
                                      ivp_real_t friction_coeff)
{
    memset(state, 0, sizeof(*state));
    state->mode             = IVP_WORLD_FRICTION_GROUND;
    if (normal) {
        state->ground_normal = *normal;
    } else {
        ivp_vec3r_set(&state->ground_normal, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    }
    if (ivp_vec3r_normalize(&state->ground_normal) != IVP_OK) {
        ivp_vec3r_set(&state->ground_normal, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    }
    state->ground_dist      = height;
    state->ground_tolerance = IVP_REAL(0.5);
    state->friction_coeff   = friction_coeff;
    state->angular_friction = IVP_REAL(0.1);
    state->active           = true;
}

void ivp_world_friction_init_global(ivp_world_friction_state_t *state,
                                      ivp_real_t friction_coeff)
{
    memset(state, 0, sizeof(*state));
    state->mode             = IVP_WORLD_FRICTION_GLOBAL;
    state->friction_coeff   = friction_coeff;
    state->angular_friction = IVP_REAL(0.1);
    state->active           = true;
}

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_world_friction(const ivp_event_sim_t *event_sim,
                                     ivp_object_t *obj,
                                     void *ctx)
{
    ivp_environment_t *env = event_sim->environment;
    const ivp_real_t dt = event_sim->delta_time;
    ivp_world_friction_state_t *state = (ivp_world_friction_state_t *)ctx;
    if (!state || !state->active) return;

    ivp_core_t *core = &obj->core;
    if (core->physical_unmoveable || core->pinned) return;

    bool apply = false;

    if (state->mode == IVP_WORLD_FRICTION_GLOBAL) {
        apply = true;
    } else {
        /* Ground mode: check distance from ground plane */
        ivp_real_t dist = ivp_vec3r_dot(&core->pos_world_f_core_last_psi,
                                         &state->ground_normal)
                        - state->ground_dist;

        /* Apply friction when within tolerance (considering object radius) */
        if (dist - core->upper_limit_radius <= state->ground_tolerance) {
            apply = true;
        }
    }

    if (!apply) return;

    ivp_vec3r_t friction_vel = core->speed;
    if (state->mode == IVP_WORLD_FRICTION_GROUND) {
        /* Ground mode only damps tangential velocity, not normal approach speed. */
        ivp_real_t vn = ivp_vec3r_dot(&friction_vel, &state->ground_normal);
        friction_vel.k[0] -= state->ground_normal.k[0] * vn;
        friction_vel.k[1] -= state->ground_normal.k[1] * vn;
        friction_vel.k[2] -= state->ground_normal.k[2] * vn;
    }

    ivp_real_t speed_sq = ivp_vec3r_dot(&friction_vel, &friction_vel);
    if (speed_sq >= IVP_REAL_EPS) {
        ivp_real_t speed = ivp_sqrt(speed_sq);

        /* Coulomb friction force: F = mu * m * g
         * dv = mu * g * dt, clamped to not exceed current speed.
         * Use environment gravity if set, otherwise assume standard Earth gravity. */
        ivp_real_t g = ivp_vec3r_length(&env->gravity);
        if (g < IVP_REAL_EPS) g = IVP_REAL(9.81);
        ivp_real_t max_dv = state->friction_coeff * g * dt;

        if (max_dv > speed) max_dv = speed;

        /* Apply deceleration along friction velocity direction (sync push). */
        ivp_real_t factor = -max_dv / speed;
        core->speed.k[0] += friction_vel.k[0] * factor;
        core->speed.k[1] += friction_vel.k[1] * factor;
        core->speed.k[2] += friction_vel.k[2] * factor;
    }

    /* Angular friction: simple damping */
    if (state->angular_friction > IVP_REAL(0.0)) {
        ivp_real_t ang_factor = IVP_REAL(1.0) - state->angular_friction * dt;
        if (ang_factor < IVP_REAL(0.0)) ang_factor = IVP_REAL(0.0);

        core->rot_speed.k[0] *= (float)ang_factor;
        core->rot_speed.k[1] *= (float)ang_factor;
        core->rot_speed.k[2] *= (float)ang_factor;
    }
}
