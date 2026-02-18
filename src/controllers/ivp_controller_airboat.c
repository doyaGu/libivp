/* ivp_controller_airboat.c — Raycast airboat/hovercraft controller
 *
 * Simulates a watercraft with pontoon contacts, buoyancy, drag,
 * turbine thrust, and steering torque.
 */
#include <ivp/controllers/ivp_controller_airboat.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>

/* ── Initialization ────────────────────────────────────────────────────── */

void ivp_airboat_init(ivp_airboat_state_t *state,
                      ivp_object_t *chassis,
                      ivp_real_t length,
                      ivp_real_t width,
                      ivp_real_t suspension_length)
{
    memset(state, 0, sizeof(*state));

    state->chassis = chassis;
    state->n_pontoons = 4;

    /* Default physics parameters */
    state->water_gravity     = IVP_REAL(7.0);
    state->drag_coefficient  = IVP_REAL(0.003);
    state->drag_area         = IVP_REAL(3.5);
    state->buoyancy_factor   = IVP_REAL(0.25);
    state->steering_gain     = IVP_REAL(1.0);
    state->active            = true;

    ivp_real_t hl = length * IVP_REAL(0.5);
    ivp_real_t hw = width * IVP_REAL(0.5);

    /* Layout: front-left, front-right, rear-left, rear-right */
    ivp_vec3r_t positions[4] = {
        {{-hw, IVP_REAL(0.0), -hl}},  /* FL */
        {{ hw, IVP_REAL(0.0), -hl}},  /* FR */
        {{-hw, IVP_REAL(0.0),  hl}},  /* RL */
        {{ hw, IVP_REAL(0.0),  hl}}   /* RR */
    };

    for (int i = 0; i < 4; i++) {
        ivp_airboat_pontoon_t *p = &state->pontoons[i];
        p->hardpoint_cs = positions[i];
        ivp_vec3r_set(&p->direction_cs, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
        p->suspension_length = suspension_length;
        p->spring_constant   = IVP_REAL(5000.0);
        p->damping           = IVP_REAL(500.0);
    }
}

/* ── Pontoon simulation ────────────────────────────────────────────────── */

static void simulate_pontoons(ivp_airboat_state_t *state, ivp_real_t dt)
{
    ivp_core_t *core = &state->chassis->core;
    ivp_real_t mass = (core->inv_mass > IVP_REAL_EPS)
                    ? (IVP_REAL(1.0) / core->inv_mass)
                    : IVP_REAL(0.0);
    if (mass < IVP_REAL_EPS) return;

    for (int i = 0; i < state->n_pontoons; i++) {
        ivp_airboat_pontoon_t *p = &state->pontoons[i];
        p->ground_contact = false;
        p->water_contact  = false;

        if (!state->raycast_fn) continue;

        /* Transform hardpoint and direction to world space */
        ivp_vec3r_t start_ws, dir_ws;
        ivp_transform_point(&start_ws, &core->m_world_f_core_last_psi,
                            &p->hardpoint_cs);
        ivp_transform_dir(&dir_ws, &core->m_world_f_core_last_psi,
                          &p->direction_cs);

        /* Raycast */
        ivp_raycast_result_t hit;
        memset(&hit, 0, sizeof(hit));
        state->raycast_fn(state->raycast_ctx, &start_ws, &dir_ws,
                          p->suspension_length, &hit);

        if (!hit.hit) {
            p->compression = IVP_REAL(0.0);
            continue;
        }

        p->contact_point_ws  = hit.hit_point;
        p->contact_normal_ws = hit.hit_normal;
        p->contact_distance  = hit.distance;
        p->compression = IVP_REAL(1.0) - hit.distance / p->suspension_length;
        if (p->compression < IVP_REAL(0.0)) p->compression = IVP_REAL(0.0);
        if (p->compression > IVP_REAL(1.0)) p->compression = IVP_REAL(1.0);

        if (hit.surface_type == IVP_SURFACE_GROUND) {
            p->ground_contact = true;

            /* Spring-damper suspension */
            ivp_real_t spring_force = p->spring_constant * p->compression *
                                     p->suspension_length;

            /* Compute velocity at contact point along normal */
            ivp_vec3r_t r_ws;
            ivp_vec3r_sub(&r_ws, &start_ws, &core->pos_world_f_core_last_psi);
            ivp_vec3r_t vel_at_point = core->speed;
            /* Add rotational contribution: v += omega x r */
            ivp_vec3r_t rot_speed_ws, omega_cross_r;
            ivp_vec3r_t rot_speed_r;
            rot_speed_r.k[0] = (ivp_real_t)core->rot_speed.k[0];
            rot_speed_r.k[1] = (ivp_real_t)core->rot_speed.k[1];
            rot_speed_r.k[2] = (ivp_real_t)core->rot_speed.k[2];
            ivp_transform_dir(&rot_speed_ws, &core->m_world_f_core_last_psi, &rot_speed_r);
            ivp_vec3r_cross(&omega_cross_r, &rot_speed_ws, &r_ws);
            ivp_vec3r_add(&vel_at_point, &vel_at_point, &omega_cross_r);

            ivp_real_t vel_normal = ivp_vec3r_dot(&vel_at_point,
                                                   &p->contact_normal_ws);

            ivp_real_t damp_force = -p->damping * vel_normal;
            ivp_real_t total_force = spring_force + damp_force;
            if (total_force < IVP_REAL(0.0)) total_force = IVP_REAL(0.0);

            ivp_real_t impulse = total_force * dt;
            ivp_real_t dv = impulse * core->inv_mass;
            core->speed.k[0] -= p->contact_normal_ws.k[0] * dv;
            core->speed.k[1] -= p->contact_normal_ws.k[1] * dv;
            core->speed.k[2] -= p->contact_normal_ws.k[2] * dv;

        } else if (hit.surface_type == IVP_SURFACE_WATER) {
            p->water_contact = true;

            /* Buoyancy: fraction of mass * reduced_gravity per pontoon */
            ivp_real_t buoyancy_force = state->buoyancy_factor * mass *
                                       state->water_gravity * p->compression;

            ivp_real_t impulse = buoyancy_force * dt;
            ivp_real_t dv = impulse * core->inv_mass;
            /* Push opposite to ray direction (upward) */
            core->speed.k[0] -= dir_ws.k[0] * dv;
            core->speed.k[1] -= dir_ws.k[1] * dv;
            core->speed.k[2] -= dir_ws.k[2] * dv;
        }
    }
}

/* ── Drag simulation ───────────────────────────────────────────────────── */

static void simulate_drag(ivp_airboat_state_t *state, ivp_real_t dt)
{
    ivp_core_t *core = &state->chassis->core;

    /* Check if any pontoon is in water */
    bool in_water = false;
    for (int i = 0; i < state->n_pontoons; i++) {
        if (state->pontoons[i].water_contact) {
            in_water = true;
            break;
        }
    }
    if (!in_water) return;

    /* Quadratic drag: F = -0.5 * rho * Cd * A * v^2 * v_hat
     * Simplified: F = -drag_coefficient * drag_area * |v| * v */
    ivp_real_t speed = ivp_vec3r_length(&core->speed);
    if (speed < IVP_REAL_EPS) return;

    ivp_real_t drag_mag = state->drag_coefficient * state->drag_area *
                          speed * speed;
    ivp_real_t drag_impulse = drag_mag * dt;

    /* Don't overdamp: limit impulse to current momentum */
    ivp_real_t max_impulse = speed / (core->inv_mass > IVP_REAL_EPS
                                      ? core->inv_mass : IVP_REAL(1.0));
    if (drag_impulse > max_impulse * IVP_REAL(0.5)) {
        drag_impulse = max_impulse * IVP_REAL(0.5);
    }

    ivp_real_t factor = -drag_impulse / speed * core->inv_mass;
    core->speed.k[0] += core->speed.k[0] * factor;
    core->speed.k[1] += core->speed.k[1] * factor;
    core->speed.k[2] += core->speed.k[2] * factor;
}

/* ── Thrust simulation ─────────────────────────────────────────────────── */

static void simulate_thrust(ivp_airboat_state_t *state, ivp_real_t dt)
{
    ivp_core_t *core = &state->chassis->core;

    ivp_real_t total_thrust = state->thrust;

    /* Booster */
    if (state->booster_remaining > IVP_REAL(0.0)) {
        total_thrust += state->booster_force;
        state->booster_remaining -= dt;
        if (state->booster_remaining < IVP_REAL(0.0))
            state->booster_remaining = IVP_REAL(0.0);
    } else if (state->booster_cooldown > IVP_REAL(0.0)) {
        state->booster_cooldown -= dt;
        if (state->booster_cooldown < IVP_REAL(0.0))
            state->booster_cooldown = IVP_REAL(0.0);
    }

    if (ivp_fabs(total_thrust) < IVP_REAL_EPS) return;

    /* Forward direction: -Z axis in core space (or configurable) */
    ivp_vec3r_t forward_cs = {{IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(-1.0)}};
    ivp_vec3r_t forward_ws;
    ivp_transform_dir(&forward_ws, &core->m_world_f_core_last_psi, &forward_cs);

    ivp_real_t dv = total_thrust * dt * core->inv_mass;
    core->speed.k[0] += forward_ws.k[0] * dv;
    core->speed.k[1] += forward_ws.k[1] * dv;
    core->speed.k[2] += forward_ws.k[2] * dv;
}

/* ── Steering simulation ───────────────────────────────────────────────── */

static void simulate_steering(ivp_airboat_state_t *state, ivp_real_t dt)
{
    if (ivp_fabs(state->steering_angle) < IVP_REAL_EPS) return;

    ivp_core_t *core = &state->chassis->core;

    /* Apply yaw torque based on steering angle.
     * Torque axis: up direction in world space (Y-up or from normal). */
    ivp_vec3r_t up_cs = {{IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0)}};
    ivp_vec3r_t up_ws;
    ivp_transform_dir(&up_ws, &core->m_world_f_core_last_psi, &up_cs);

    /* Scale steering torque by speed (more speed = more steering authority) */
    ivp_real_t speed = ivp_vec3r_length(&core->speed);
    ivp_real_t authority = speed * state->steering_gain;
    if (authority > IVP_REAL(10.0)) authority = IVP_REAL(10.0);

    ivp_real_t torque = state->steering_angle * authority;

    /* Apply angular impulse in core space */
    ivp_vec3r_t torque_ws;
    ivp_vec3r_scale(&torque_ws, &up_ws, torque);

    ivp_vec3r_t torque_cs;
    ivp_mat3r_vimult(&torque_cs, &core->m_world_f_core_last_psi.rot, &torque_ws);

    core->rot_speed.k[0] += (float)(torque_cs.k[0] * core->inv_rot_inertia.k[0] * dt);
    core->rot_speed.k[1] += (float)(torque_cs.k[1] * core->inv_rot_inertia.k[1] * dt);
    core->rot_speed.k[2] += (float)(torque_cs.k[2] * core->inv_rot_inertia.k[2] * dt);

    /* Angular damping to prevent excessive spin */
    ivp_real_t ang_damp = IVP_REAL(0.98);
    core->rot_speed.k[0] *= (float)ang_damp;
    core->rot_speed.k[1] *= (float)ang_damp;
    core->rot_speed.k[2] *= (float)ang_damp;
}

/* ── Controller callback ───────────────────────────────────────────────── */

void ivp_controller_airboat(const ivp_event_sim_t *event_sim,
                            ivp_object_t *obj,
                            void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;
    ivp_airboat_state_t *state = (ivp_airboat_state_t *)ctx;
    if (!state || !state->active) return;
    if (!state->chassis || obj != state->chassis) return;
    if (state->chassis->core.physical_unmoveable) return;
    if (dt <= IVP_REAL_EPS) return;

    simulate_pontoons(state, dt);
    simulate_drag(state, dt);
    simulate_thrust(state, dt);
    simulate_steering(state, dt);
}
