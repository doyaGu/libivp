/* ivp_controller_buoyancy.c -- Buoyancy controller implementation */

#include <ivp/controllers/ivp_controller_buoyancy.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_mat3.h>
#include <string.h>

/* ── Initialization ────────────────────────────────────────────────────── */

void ivp_buoyancy_init(ivp_buoyancy_state_t *state,
                         ivp_real_t surface_y,
                         ivp_real_t fluid_density)
{
    memset(state, 0, sizeof(*state));
    /* For Y-down gravity, surface normal points up (-Y) */
    ivp_vec3r_set(&state->surface_normal,
                  IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    state->surface_height = -surface_y; /* plane eq: -1*y = -surface_y */
    state->fluid_density = fluid_density;
    state->linear_drag = IVP_REAL(0.5);
    state->angular_drag = IVP_REAL(0.3);
    ivp_vec3r_set(&state->flow_velocity,
                  IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    state->active = true;
}

void ivp_buoyancy_init_plane(ivp_buoyancy_state_t *state,
                               const ivp_vec3r_t *normal,
                               ivp_real_t height,
                               ivp_real_t fluid_density)
{
    memset(state, 0, sizeof(*state));
    state->surface_normal = *normal;
    state->surface_height = height;
    state->fluid_density = fluid_density;
    state->linear_drag = IVP_REAL(0.5);
    state->angular_drag = IVP_REAL(0.3);
    ivp_vec3r_set(&state->flow_velocity,
                  IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    state->active = true;
}

/* ── Controller callback ───────────────────────────────────────────────── */

void ivp_controller_buoyancy(const ivp_event_sim_t *event_sim,
                               ivp_object_t *obj,
                               void *ctx)
{
    ivp_environment_t *env = event_sim->environment;
    const ivp_real_t dt = event_sim->delta_time;
    ivp_buoyancy_state_t *state = (ivp_buoyancy_state_t *)ctx;
    if (!state || !state->active) return;

    ivp_core_t *core = &obj->core;
    if (core->physical_unmoveable || core->pinned) return;

    /* Compute signed distance from surface plane to object center.
     * dist > 0 means above surface (in air), dist < 0 means submerged. */
    ivp_real_t dist = ivp_vec3r_dot(&core->pos_world_f_core_last_psi,
                                     &state->surface_normal)
                    - state->surface_height;

    ivp_real_t radius = core->upper_limit_radius;
    if (radius < IVP_REAL_EPS) return;

    /* Proper sphere cap volume computation (matching legacy IVP_Buoyancy_Solver).
     * h = height of submerged spherical cap = radius - dist  (when dist < radius)
     * The submerged height is clamped to [0, 2*radius]. */
    ivp_real_t h = radius - dist;  /* distance from bottom of sphere to surface */
    if (h <= IVP_REAL(0.0)) return;  /* fully above surface */
    if (h > IVP_REAL(2.0) * radius) h = IVP_REAL(2.0) * radius;

    /* Sphere cap volume: V = pi * h^2 * (3r - h) / 3
     * (Legacy: compute_buoyancy_values_for_one_ball, case 1-4) */
    ivp_real_t pi = IVP_PI;
    ivp_real_t volume_submerged = (pi * h * h * (IVP_REAL(3.0) * radius - h))
                                / IVP_REAL(3.0);

    /* Displacement center: the center of buoyancy is offset from the
     * geometric center along the surface normal.
     * Legacy formula: center_offset = 0.75 * (4r^2 - 4rh + h^2) / (3r - h)
     * This gives the distance from geometric center to the centroid of
     * the submerged cap, measured along the surface normal direction. */
    ivp_real_t denom_vc = IVP_REAL(3.0) * radius - h;
    ivp_vec3r_t volume_center;
    if (denom_vc > IVP_REAL_EPS) {
        ivp_real_t center_offset = IVP_REAL(0.75) *
            (IVP_REAL(4.0) * radius * radius
           - IVP_REAL(4.0) * radius * h
           + h * h) / denom_vc;

        /* Volume center = object center + center_offset * surface_normal
         * (surface normal points out of fluid, so the center of submerged
         *  volume is offset IN the fluid direction) */
        volume_center.k[0] = core->pos_world_f_core_last_psi.k[0]
                           + state->surface_normal.k[0] * center_offset;
        volume_center.k[1] = core->pos_world_f_core_last_psi.k[1]
                           + state->surface_normal.k[1] * center_offset;
        volume_center.k[2] = core->pos_world_f_core_last_psi.k[2]
                           + state->surface_normal.k[2] * center_offset;
    } else {
        volume_center = core->pos_world_f_core_last_psi;
    }

    /* ── Buoyancy force ──────────────────────────────────────────────── */

    /* F_buoyancy = rho_fluid * V_submerged * g
     * Applied at the volume center, direction opposing surface normal
     * (i.e., into the fluid = upward for typical water surface). */
    ivp_real_t gravity_mag = ivp_vec3r_length(&env->gravity);
    ivp_real_t buoyancy_force = state->fluid_density * volume_submerged * gravity_mag;
    ivp_real_t buoyancy_accel = buoyancy_force * core->inv_mass;

    /* Apply buoyancy impulse: dv = (F/m) * dt = buoyancy_accel * dt
     * Direction: along surface normal (surface normal points OUT of fluid = upward).
     * Legacy: impulse = gravity_dir * volume * (-density) * dt, which gives upward push.
     * Here surface_normal already points upward, so += gives the same result. */
    core->speed_change.k[0] += state->surface_normal.k[0] * buoyancy_accel * dt;
    core->speed_change.k[1] += state->surface_normal.k[1] * buoyancy_accel * dt;
    core->speed_change.k[2] += state->surface_normal.k[2] * buoyancy_accel * dt;

    /* ── Torque from displaced volume ────────────────────────────────── */

    /* The buoyancy force acts at the volume center, not the center of mass.
     * This creates a torque: tau = r_vc x F_buoyancy
     * where r_vc = volume_center - center_of_mass */
    ivp_vec3r_t r_vc;
    ivp_vec3r_sub(&r_vc, &volume_center, &core->pos_world_f_core_last_psi);
    ivp_real_t r_vc_mag = ivp_vec3r_length(&r_vc);
    if (r_vc_mag > IVP_REAL_EPS) {
        ivp_vec3r_t buoy_dir;
        buoy_dir.k[0] = -state->surface_normal.k[0];
        buoy_dir.k[1] = -state->surface_normal.k[1];
        buoy_dir.k[2] = -state->surface_normal.k[2];

        ivp_vec3r_t force_ws;
        ivp_vec3r_scale(&force_ws, &buoy_dir, buoyancy_force);

        ivp_vec3r_t torque_ws;
        ivp_vec3r_cross(&torque_ws, &r_vc, &force_ws);

        ivp_vec3r_t torque_cs;
        ivp_mat3r_vimult(&torque_cs, &core->m_world_f_core_last_psi.rot, &torque_ws);

        core->rot_speed.k[0] += (float)(torque_cs.k[0] * core->inv_rot_inertia.k[0] * dt);
        core->rot_speed.k[1] += (float)(torque_cs.k[1] * core->inv_rot_inertia.k[1] * dt);
        core->rot_speed.k[2] += (float)(torque_cs.k[2] * core->inv_rot_inertia.k[2] * dt);
    }

    /* ── Pressure drag (legacy: projected surface area based) ────────── */

    /* The projected submerged surface area for a sphere cap.
     * For a sphere cap with height h:
     *   If fully submerged (h >= 2r): projected area = pi * r^2
     *   Partial: projected area = pi * h * (2r - h)  (circle from cap projection)
     * However the legacy code computes this differently depending on velocity
     * direction relative to the surface. For simplicity, use the cap projection. */
    ivp_real_t proj_area;
    if (h >= IVP_REAL(2.0) * radius) {
        proj_area = pi * radius * radius;
    } else {
        /* Projected circle area of the sphere cap: pi * R_cap^2
         * where R_cap^2 = h * (2r - h)  (from spherical cap geometry) */
        proj_area = pi * h * (IVP_REAL(2.0) * radius - h);
    }

    /* Relative velocity of object w.r.t. fluid */
    ivp_vec3r_t rel_vel;
    rel_vel.k[0] = core->speed.k[0] - state->flow_velocity.k[0];
    rel_vel.k[1] = core->speed.k[1] - state->flow_velocity.k[1];
    rel_vel.k[2] = core->speed.k[2] - state->flow_velocity.k[2];

    ivp_real_t rel_speed_sq = ivp_vec3r_quad_length(&rel_vel);

    /* Pressure drag: F_drag = 0.5 * rho * Cd * A * v^2 (in direction opposing velocity)
     * Legacy uses pressure_damp_factor = input.pressure_damp_factor * 0.5 * rho.
     * We use linear_drag as the Cd-like coefficient. */
    if (state->linear_drag > IVP_REAL(0.0) && rel_speed_sq > IVP_REAL_EPS) {
        ivp_real_t drag_coeff = IVP_REAL(0.5) * state->fluid_density *
                                state->linear_drag * proj_area;
        ivp_real_t rel_speed = ivp_sqrt(rel_speed_sq);
        ivp_real_t drag_force = drag_coeff * rel_speed;
        ivp_real_t drag_accel = drag_force * core->inv_mass;

        /* Cap to prevent velocity reversal: drag_decel * dt <= |v| */
        if (drag_accel * dt > rel_speed) {
            drag_accel = rel_speed / dt;
        }

        ivp_real_t inv_rel_speed = IVP_REAL(1.0) / rel_speed;
        core->speed_change.k[0] -= rel_vel.k[0] * inv_rel_speed * drag_accel * dt;
        core->speed_change.k[1] -= rel_vel.k[1] * inv_rel_speed * drag_accel * dt;
        core->speed_change.k[2] -= rel_vel.k[2] * inv_rel_speed * drag_accel * dt;
    }

    /* ── Angular drag ────────────────────────────────────────────────── */

    if (state->angular_drag > IVP_REAL(0.0)) {
        /* Immersion fraction for angular drag scaling */
        ivp_real_t immersion = h / (IVP_REAL(2.0) * radius);
        if (immersion > IVP_REAL(1.0)) immersion = IVP_REAL(1.0);

        ivp_real_t ang_damp = IVP_REAL(1.0) - immersion * state->angular_drag * dt;
        if (ang_damp < IVP_REAL(0.0)) ang_damp = IVP_REAL(0.0);

        core->rot_speed.k[0] *= (float)ang_damp;
        core->rot_speed.k[1] *= (float)ang_damp;
        core->rot_speed.k[2] *= (float)ang_damp;
    }
}
