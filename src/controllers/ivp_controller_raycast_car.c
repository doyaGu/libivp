/* ivp_controller_raycast_car.c -- Raycast vehicle implementation */

#include <ivp/controllers/ivp_controller_raycast_car.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>
#include <math.h>

/* ── Initialization ────────────────────────────────────────────────────── */

void ivp_car_init_4wheel(ivp_car_state_t *state,
                           ivp_object_t *chassis,
                           ivp_real_t wheelbase,
                           ivp_real_t track_width,
                           ivp_real_t wheel_radius,
                           ivp_real_t suspension_length)
{
    memset(state, 0, sizeof(*state));
    state->chassis = chassis;
    state->steer_front = true;
    state->drive_rear = true;
    state->active = true;

    /* Default ground plane: Y=0, normal pointing up (-Y for Y-down) */
    ivp_vec3r_set(&state->ground_normal,
                  IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    state->ground_height = IVP_REAL(0.0);

    ivp_real_t half_wb = wheelbase * IVP_REAL(0.5);
    ivp_real_t half_tw = track_width * IVP_REAL(0.5);

    /* Wheel layout in chassis core space (Y-down):
     * Front-left [0], Front-right [1], Rear-left [2], Rear-right [3] */
    struct { ivp_real_t x, z; } positions[4] = {
        { -half_tw, -half_wb }, /* FL */
        {  half_tw, -half_wb }, /* FR */
        { -half_tw,  half_wb }, /* RL */
        {  half_tw,  half_wb }  /* RR */
    };

    for (int i = 0; i < 4; i++) {
        ivp_car_wheel_t w;
        memset(&w, 0, sizeof(w));
        ivp_vec3r_set(&w.hardpoint_cs, positions[i].x, IVP_REAL(0.0), positions[i].z);
        ivp_vec3r_set(&w.direction_cs, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
        w.suspension_length = suspension_length;
        w.radius = wheel_radius;
        w.spring_constant = IVP_REAL(50000.0);  /* 50 kN/m default */
        w.damping = IVP_REAL(3000.0);           /* 3 kNs/m default */
        w.friction_forward = IVP_REAL(1.0);
        w.friction_side = IVP_REAL(1.0);
        ivp_car_add_wheel(state, &w);
    }
}

int ivp_car_add_wheel(ivp_car_state_t *state, const ivp_car_wheel_t *wheel)
{
    if (!state || !wheel || state->n_wheels >= IVP_CAR_MAX_WHEELS) return -1;
    int idx = state->n_wheels++;
    state->wheels[idx] = *wheel;
    state->wheels[idx].compression = IVP_REAL(0.0);
    state->wheels[idx].angular_velocity = IVP_REAL(0.0);
    state->wheels[idx].grounded = false;
    return idx;
}

/* ── Controller callback ───────────────────────────────────────────────── */

void ivp_controller_raycast_car(const ivp_event_sim_t *event_sim,
                                  ivp_object_t *obj,
                                  void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;
    ivp_car_state_t *car = (ivp_car_state_t *)ctx;
    if (!car || !car->active || obj != car->chassis) return;

    ivp_core_t *core = &obj->core;
    if (core->physical_unmoveable) return;
    if (car->n_wheels <= 0) return;

    for (int wi = 0; wi < car->n_wheels; wi++) {
        ivp_car_wheel_t *w = &car->wheels[wi];

        /* Transform hardpoint and direction to world space */
        ivp_vec3r_t hp_ws, dir_ws;
        ivp_transform_point(&hp_ws, &core->m_world_f_core_last_psi, &w->hardpoint_cs);
        ivp_transform_dir(&dir_ws, &core->m_world_f_core_last_psi, &w->direction_cs);

        /* Raycast against ground plane.
         * ray: P = hp_ws + t * dir_ws
         * plane: normal · P = ground_height
         * t = (ground_height - normal · hp_ws) / (normal · dir_ws) */
        ivp_real_t denom = ivp_vec3r_dot(&car->ground_normal, &dir_ws);
        if (ivp_fabs(denom) < IVP_REAL_EPS) {
            w->grounded = false;
            continue;
        }

        ivp_real_t t_hit = (car->ground_height -
                            ivp_vec3r_dot(&car->ground_normal, &hp_ws)) / denom;

        /* Check if hit is within suspension range */
        ivp_real_t max_ray = w->suspension_length + w->radius;
        if (t_hit < IVP_REAL(0.0) || t_hit > max_ray) {
            w->grounded = false;
            w->compression = IVP_REAL(0.0);
            continue;
        }

        w->grounded = true;
        w->contact_point_ws.k[0] = hp_ws.k[0] + dir_ws.k[0] * t_hit;
        w->contact_point_ws.k[1] = hp_ws.k[1] + dir_ws.k[1] * t_hit;
        w->contact_point_ws.k[2] = hp_ws.k[2] + dir_ws.k[2] * t_hit;
        w->contact_normal_ws = car->ground_normal;

        /* Suspension compression */
        ivp_real_t spring_length = t_hit - w->radius;
        w->compression = IVP_REAL(1.0) - spring_length / w->suspension_length;
        if (w->compression < IVP_REAL(0.0)) w->compression = IVP_REAL(0.0);
        if (w->compression > IVP_REAL(1.0)) w->compression = IVP_REAL(1.0);

        /* Suspension force (spring + damper) */
        ivp_real_t spring_force = w->compression * w->suspension_length *
                                   w->spring_constant;

        /* Compute velocity at wheel contact point relative to ground */
        ivp_vec3r_t r_ws;
        ivp_vec3r_sub(&r_ws, &w->contact_point_ws,
                      &core->pos_world_f_core_last_psi);

        /* velocity at point = v + omega x r */
        ivp_vec3r_t omega_ws;
        ivp_transform_dir(&omega_ws, &core->m_world_f_core_last_psi,
                          &(ivp_vec3r_t){{ (ivp_real_t)core->rot_speed.k[0],
                                           (ivp_real_t)core->rot_speed.k[1],
                                           (ivp_real_t)core->rot_speed.k[2] }});
        ivp_vec3r_t v_point;
        ivp_vec3r_cross(&v_point, &omega_ws, &r_ws);
        v_point.k[0] += core->speed.k[0];
        v_point.k[1] += core->speed.k[1];
        v_point.k[2] += core->speed.k[2];

        /* Damping: velocity component along suspension direction */
        ivp_real_t v_susp = ivp_vec3r_dot(&v_point, &dir_ws);
        ivp_real_t damp_force = -v_susp * w->damping;

        ivp_real_t total_susp_force = spring_force + damp_force;
        if (total_susp_force < IVP_REAL(0.0)) total_susp_force = IVP_REAL(0.0);

        /* Apply suspension force along -dir_ws (opposing compression) */
        ivp_real_t susp_impulse = total_susp_force * dt;
        core->speed_change.k[0] -= dir_ws.k[0] * susp_impulse * core->inv_mass;
        core->speed_change.k[1] -= dir_ws.k[1] * susp_impulse * core->inv_mass;
        core->speed_change.k[2] -= dir_ws.k[2] * susp_impulse * core->inv_mass;

        /* Compute forward and side directions at contact */
        /* Forward = chassis forward rotated by steering (for front wheels) */
        ivp_vec3r_t fwd_cs;
        ivp_vec3r_set(&fwd_cs, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(-1.0));

        bool is_front = (wi < 2);
        bool is_steered = is_front && car->steer_front;

        if (is_steered) {
            ivp_real_t cs_angle = ivp_cos(car->steering_angle);
            ivp_real_t sn_angle = ivp_sin(car->steering_angle);
            ivp_vec3r_t steered;
            steered.k[0] = fwd_cs.k[0] * cs_angle - fwd_cs.k[2] * sn_angle;
            steered.k[1] = fwd_cs.k[1];
            steered.k[2] = fwd_cs.k[0] * sn_angle + fwd_cs.k[2] * cs_angle;
            fwd_cs = steered;
        }

        ivp_vec3r_t fwd_ws;
        ivp_transform_dir(&fwd_ws, &core->m_world_f_core_last_psi, &fwd_cs);

        /* Project onto ground plane */
        ivp_real_t fwd_dot_n = ivp_vec3r_dot(&fwd_ws, &car->ground_normal);
        fwd_ws.k[0] -= fwd_dot_n * car->ground_normal.k[0];
        fwd_ws.k[1] -= fwd_dot_n * car->ground_normal.k[1];
        fwd_ws.k[2] -= fwd_dot_n * car->ground_normal.k[2];
        ivp_real_t fwd_len = ivp_vec3r_length(&fwd_ws);
        if (fwd_len > IVP_REAL_EPS) {
            ivp_real_t inv_len = IVP_REAL(1.0) / fwd_len;
            fwd_ws.k[0] *= inv_len; fwd_ws.k[1] *= inv_len; fwd_ws.k[2] *= inv_len;
        }

        /* Side direction = ground_normal x forward */
        ivp_vec3r_t side_ws;
        ivp_vec3r_cross(&side_ws, &car->ground_normal, &fwd_ws);

        /* Tire friction forces */
        ivp_real_t v_fwd = ivp_vec3r_dot(&v_point, &fwd_ws);
        ivp_real_t v_side = ivp_vec3r_dot(&v_point, &side_ws);

        /* Normal load for Coulomb friction */
        ivp_real_t normal_force = total_susp_force;

        /* Lateral friction: oppose sliding */
        ivp_real_t max_lat = normal_force * w->friction_side;
        ivp_real_t lat_impulse = -v_side * core->mass * IVP_REAL(0.25); /* per-wheel share */
        if (lat_impulse > max_lat * dt) lat_impulse = max_lat * dt;
        if (lat_impulse < -max_lat * dt) lat_impulse = -max_lat * dt;
        core->speed_change.k[0] += side_ws.k[0] * lat_impulse * core->inv_mass;
        core->speed_change.k[1] += side_ws.k[1] * lat_impulse * core->inv_mass;
        core->speed_change.k[2] += side_ws.k[2] * lat_impulse * core->inv_mass;

        /* Engine / brake force (longitudinal) */
        bool is_driven = (!is_front && car->drive_rear) ||
                         (is_front && !car->drive_rear);

        ivp_real_t long_force = IVP_REAL(0.0);
        if (is_driven) {
            long_force += car->engine_force;
        }

        /* Brake all wheels */
        if (car->brake_force > IVP_REAL(0.0)) {
            ivp_real_t brake = car->brake_force;
            if (v_fwd > IVP_REAL(0.0)) {
                long_force -= brake;
                if (long_force < -normal_force * w->friction_forward)
                    long_force = -normal_force * w->friction_forward;
            } else {
                long_force += brake;
                if (long_force > normal_force * w->friction_forward)
                    long_force = normal_force * w->friction_forward;
            }
        }

        ivp_real_t fwd_impulse = long_force * dt / (ivp_real_t)car->n_wheels;
        core->speed_change.k[0] += fwd_ws.k[0] * fwd_impulse * core->inv_mass;
        core->speed_change.k[1] += fwd_ws.k[1] * fwd_impulse * core->inv_mass;
        core->speed_change.k[2] += fwd_ws.k[2] * fwd_impulse * core->inv_mass;

        /* Update wheel spin */
        if (w->radius > IVP_REAL_EPS) {
            w->angular_velocity = v_fwd / w->radius;
        }
    }
}
