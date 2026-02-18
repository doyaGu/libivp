/* ivp_controller_floating.c — Floating/levitation controller
 *
 * Applies a 1D impulse along a ray direction to maintain an object
 * at a target distance from a surface.
 */
#include <ivp/controllers/ivp_controller_floating.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>

/* ── Initialization ────────────────────────────────────────────────────── */

void ivp_floating_init(ivp_floating_state_t *state,
                       ivp_real_t target_dist,
                       ivp_real_t max_repulsive,
                       ivp_real_t max_adhesive)
{
    memset(state, 0, sizeof(*state));

    /* Default: ray points downward (Y-down gravity → push up) */
    state->ray_direction_ws.k[1] = IVP_REAL(1.0);  /* +Y = down */

    state->target_distance     = target_dist;
    state->current_distance    = target_dist;
    state->max_repulsive_force = max_repulsive;
    state->max_adhesive_force  = max_adhesive;
    state->damping             = IVP_REAL(0.5);
    state->active              = true;
}

void ivp_floating_set_raycast(ivp_floating_state_t *state,
                              ivp_floating_raycast_fn_t fn,
                              void *ctx)
{
    state->raycast_fn  = fn;
    state->raycast_ctx = ctx;
}

void ivp_floating_set_position_os(ivp_floating_state_t *state,
                                  const ivp_vec3r_t *pos_os)
{
    state->position_os = *pos_os;
}

void ivp_floating_set_ray_direction_ws(ivp_floating_state_t *state,
                                       const ivp_vec3r_t *dir_ws)
{
    state->ray_direction_ws = *dir_ws;
    ivp_real_t len = ivp_vec3r_length(&state->ray_direction_ws);
    if (len > IVP_REAL_EPS) {
        ivp_real_t inv = IVP_REAL(1.0) / len;
        state->ray_direction_ws.k[0] *= inv;
        state->ray_direction_ws.k[1] *= inv;
        state->ray_direction_ws.k[2] *= inv;
    }
}

/* ── Controller callback ───────────────────────────────────────────────── */

void ivp_controller_floating(const ivp_event_sim_t *event_sim,
                             ivp_object_t *obj,
                             void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;
    ivp_floating_state_t *state = (ivp_floating_state_t *)ctx;
    if (!state || !state->active) return;
    if (!obj || obj->core.physical_unmoveable) return;
    if (dt <= IVP_REAL_EPS) return;

    ivp_core_t *core = &obj->core;

    /* Compute ray start in world space */
    ivp_vec3r_t ray_start_ws;
    ivp_transform_point(&ray_start_ws, &core->m_world_f_core_last_psi,
                        &state->position_os);

    /* Update current distance via raycast */
    if (state->raycast_fn) {
        ivp_real_t dist;
        bool hit = state->raycast_fn(state->raycast_ctx, obj,
                                     &ray_start_ws,
                                     &state->ray_direction_ws,
                                     &dist);
        if (hit) {
            state->current_distance = dist;
        } else {
            /* No surface found: no force applied */
            return;
        }
    }

    /* Distance error: positive means we're too far (need to pull toward) */
    ivp_real_t error = state->current_distance - state->target_distance;

    /* Velocity along ray direction */
    ivp_real_t vel_along_ray = ivp_vec3r_dot(&core->speed,
                                              &state->ray_direction_ws);

    /* Virtual mass along ray direction:
     * For a center-of-mass push along ray, vm = mass */
    ivp_real_t vm = (core->inv_mass > IVP_REAL_EPS)
                  ? (IVP_REAL(1.0) / core->inv_mass)
                  : IVP_REAL(0.0);
    if (vm < IVP_REAL_EPS) return;

    /* Desired impulse:
     * - Spring-like: push proportional to error
     * - Damper: oppose velocity along ray
     * Impulse = mass * (kp * error - kd * vel_along_ray) * dt
     *
     * Use tau as the spring gain and damping as the velocity gain. */
    ivp_real_t kp = vm / (dt * dt);    /* spring stiffness */
    ivp_real_t kd = vm * state->damping / dt;  /* damping coefficient */

    /* Force along ray (positive = push in ray direction = toward surface) */
    ivp_real_t force = kp * error - kd * vel_along_ray;

    /* Clamp force */
    if (force > IVP_REAL(0.0)) {
        /* Adhesive: pulling toward surface */
        if (force > state->max_adhesive_force) {
            force = state->max_adhesive_force;
        }
    } else {
        /* Repulsive: pushing away from surface */
        if (force < -state->max_repulsive_force) {
            force = -state->max_repulsive_force;
        }
    }

    /* Convert force to impulse and apply along ray direction */
    ivp_real_t impulse = force * dt;
    ivp_real_t dv = impulse * core->inv_mass;

    core->speed.k[0] += state->ray_direction_ws.k[0] * dv;
    core->speed.k[1] += state->ray_direction_ws.k[1] * dv;
    core->speed.k[2] += state->ray_direction_ws.k[2] * dv;
}
