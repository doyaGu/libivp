/* ivp_car_system.c — Real-wheel vehicle system
 *
 * Manages suspension, steering, and engine forces between a chassis
 * body and physical wheel objects.
 */
#include <ivp/controllers/ivp_car_system.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>

/* ── Template defaults ────────────────────────────────────────────────── */

void ivp_car_system_template_default(ivp_car_system_template_t *tmpl)
{
    memset(tmpl, 0, sizeof(*tmpl));
    tmpl->n_wheels = 4;
    tmpl->n_axes = 2;

    for (int i = 0; i < IVP_CAR_SYSTEM_MAX_WHEELS; i++) {
        tmpl->wheel_radius[i] = IVP_REAL(0.3);
        tmpl->wheel_friction[i] = IVP_REAL(1.0);
        tmpl->wheel_reversed_sign[i] = IVP_REAL(1.0);
        tmpl->spring_constant[i] = IVP_REAL(50000.0);
        tmpl->spring_damping[i] = IVP_REAL(3000.0);
        tmpl->spring_damping_compression[i] = IVP_REAL(3000.0);
        tmpl->max_body_force[i] = IVP_REAL(1e6);
    }

    /* Default: front wheels = axis 0, rear wheels = axis 1 */
    tmpl->wheel_axis_index[0] = 0;
    tmpl->wheel_axis_index[1] = 0;
    tmpl->wheel_axis_index[2] = 1;
    tmpl->wheel_axis_index[3] = 1;

    tmpl->stabilizer_constant[0] = IVP_REAL(500.0);
    tmpl->stabilizer_constant[1] = IVP_REAL(500.0);

    tmpl->extra_gravity_force = IVP_REAL(0.0);
    tmpl->body_counter_torque_factor = IVP_REAL(0.0);
    tmpl->fast_turn_factor = IVP_REAL(1.0);
}

/* ── Initialization ───────────────────────────────────────────────────── */

void ivp_car_system_init(ivp_car_system_t *car,
                           const ivp_car_system_template_t *tmpl)
{
    memset(car, 0, sizeof(*car));

    car->body = tmpl->car_body;
    car->n_wheels = tmpl->n_wheels;
    car->n_axes = tmpl->n_axes;
    car->active = true;

    car->extra_gravity_force = tmpl->extra_gravity_force;
    car->extra_gravity_height_offset = tmpl->extra_gravity_height_offset;
    car->body_counter_torque_factor = tmpl->body_counter_torque_factor;
    car->fast_turn_factor = tmpl->fast_turn_factor;

    for (int i = 0; i < car->n_wheels && i < IVP_CAR_SYSTEM_MAX_WHEELS; i++) {
        ivp_car_system_wheel_t *w = &car->wheels[i];
        w->wheel_obj = tmpl->wheel_objects[i];
        w->radius = tmpl->wheel_radius[i];
        w->friction = tmpl->wheel_friction[i];
        w->reversed_sign = tmpl->wheel_reversed_sign[i];
        w->axis_index = tmpl->wheel_axis_index[i];
        w->spring_constant = tmpl->spring_constant[i];
        w->spring_damping = tmpl->spring_damping[i];
        w->spring_damping_compression = tmpl->spring_damping_compression[i];
        w->spring_pre_tension = tmpl->spring_pre_tension[i];
        w->max_body_force = tmpl->max_body_force[i];

        /* Default suspension axis: chassis Y (down) */
        ivp_vec3r_set(&w->axis_cs,
                      IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

        /* Compute hardpoint from wheel position relative to body */
        if (w->wheel_obj && car->body) {
            ivp_vec3r_t wheel_pos = w->wheel_obj->core.pos_world_f_core_last_psi;
            ivp_vec3r_t body_pos = car->body->core.pos_world_f_core_last_psi;
            ivp_vec3r_t offset_ws;
            ivp_vec3r_sub(&offset_ws, &wheel_pos, &body_pos);

            /* Transform to body core space */
            ivp_mat3r_vimult(&w->hardpoint_cs,
                              &car->body->core.m_world_f_core_last_psi.rot,
                              &offset_ws);
            w->suspension_travel = IVP_REAL(0.3);
        }
    }

    /* Set up axle-to-wheel mapping */
    for (int ax = 0; ax < car->n_axes && ax < IVP_CAR_SYSTEM_MAX_AXES; ax++) {
        car->axles[ax].stabilizer_constant = tmpl->stabilizer_constant[ax];
        car->axles[ax].left_wheel = -1;
        car->axles[ax].right_wheel = -1;
    }

    /* Find left/right wheels per axle */
    for (int i = 0; i < car->n_wheels; i++) {
        int ax = car->wheels[i].axis_index;
        if (ax < 0 || ax >= car->n_axes) continue;
        if (car->axles[ax].left_wheel < 0) {
            car->axles[ax].left_wheel = i;
        } else if (car->axles[ax].right_wheel < 0) {
            car->axles[ax].right_wheel = i;
        }
    }
}

/* ── Steering ─────────────────────────────────────────────────────────── */

void ivp_car_system_steer_wheel(ivp_car_system_t *car,
                                  int wheel_index,
                                  ivp_real_t angle_rad)
{
    if (wheel_index < 0 || wheel_index >= car->n_wheels) return;
    car->wheels[wheel_index].steering_angle = angle_rad;
}

void ivp_car_system_set_steering(ivp_car_system_t *car,
                                   ivp_real_t steering_angle)
{
    car->steering_angle = steering_angle;

    /* Apply to front wheels (axis 0) with fast_turn_factor */
    ivp_real_t effective = steering_angle * car->fast_turn_factor;
    for (int i = 0; i < car->n_wheels; i++) {
        if (car->wheels[i].axis_index == 0) {
            car->wheels[i].steering_angle = effective *
                car->wheels[i].reversed_sign;
        }
    }
}

/* ── Dynamic adjustment ───────────────────────────────────────────────── */

void ivp_car_system_set_spring_constant(ivp_car_system_t *car,
                                          int wheel_index, ivp_real_t k)
{
    if (wheel_index >= 0 && wheel_index < car->n_wheels)
        car->wheels[wheel_index].spring_constant = k;
}

void ivp_car_system_set_spring_damping(ivp_car_system_t *car,
                                         int wheel_index, ivp_real_t d)
{
    if (wheel_index >= 0 && wheel_index < car->n_wheels)
        car->wheels[wheel_index].spring_damping = d;
}

void ivp_car_system_set_stabilizer(ivp_car_system_t *car,
                                     int axis_index, ivp_real_t k)
{
    if (axis_index >= 0 && axis_index < car->n_axes)
        car->axles[axis_index].stabilizer_constant = k;
}

void ivp_car_system_set_wheel_torque(ivp_car_system_t *car,
                                       int wheel_index, ivp_real_t torque)
{
    if (wheel_index >= 0 && wheel_index < car->n_wheels)
        car->wheels[wheel_index].torque = torque;
}

void ivp_car_system_fix_wheel(ivp_car_system_t *car,
                                int wheel_index, bool fixed)
{
    if (wheel_index >= 0 && wheel_index < car->n_wheels)
        car->wheels[wheel_index].fixed = fixed;
}

/* ── Booster ──────────────────────────────────────────────────────────── */

void ivp_car_system_activate_booster(ivp_car_system_t *car,
                                       ivp_real_t force,
                                       ivp_real_t duration,
                                       ivp_real_t recharge_time)
{
    if (car->booster_cooldown > IVP_REAL(0.0)) return;
    car->booster_force = force;
    car->booster_remaining = duration;
    car->booster_recharge = recharge_time;
}

/* ── Queries ──────────────────────────────────────────────────────────── */

ivp_real_t ivp_car_system_get_body_speed(const ivp_car_system_t *car)
{
    if (!car || !car->body) return IVP_REAL(0.0);
    const ivp_core_t *core = &car->body->core;

    ivp_vec3r_t forward_cs = {{IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(-1.0)}};
    ivp_vec3r_t forward_ws;
    ivp_transform_dir(&forward_ws, &core->m_world_f_core_last_psi, &forward_cs);
    return ivp_vec3r_dot(&core->speed, &forward_ws);
}

ivp_real_t ivp_car_system_get_wheel_angular_velocity(
    const ivp_car_system_t *car, int wheel_index)
{
    if (!car || wheel_index < 0 || wheel_index >= car->n_wheels)
        return IVP_REAL(0.0);
    return car->wheels[wheel_index].angular_velocity;
}

/* ── Simulation subroutines ───────────────────────────────────────────── */

static void simulate_suspension(ivp_car_system_t *car, ivp_real_t dt)
{
    ivp_core_t *body = &car->body->core;

    for (int i = 0; i < car->n_wheels; i++) {
        ivp_car_system_wheel_t *w = &car->wheels[i];
        if (!w->wheel_obj) continue;
        if (w->wheel_obj->core.physical_unmoveable) continue;

        ivp_core_t *wheel = &w->wheel_obj->core;

        /* Compute hardpoint in world space */
        ivp_vec3r_t hp_ws;
        ivp_transform_point(&hp_ws, &body->m_world_f_core_last_psi,
                            &w->hardpoint_cs);

        /* Suspension direction in world space (body Y axis = down) */
        ivp_vec3r_t susp_dir_cs = {{IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0)}};
        ivp_vec3r_t susp_dir_ws;
        ivp_transform_dir(&susp_dir_ws, &body->m_world_f_core_last_psi,
                          &susp_dir_cs);

        /* Current wheel position relative to hardpoint */
        ivp_vec3r_t wheel_offset;
        ivp_vec3r_sub(&wheel_offset, &wheel->pos_world_f_core_last_psi, &hp_ws);
        ivp_real_t displacement = ivp_vec3r_dot(&wheel_offset, &susp_dir_ws);

        /* Compression: how far wheel has moved into suspension travel */
        w->compression = (w->suspension_travel - displacement) /
                          w->suspension_travel;
        if (w->compression < IVP_REAL(0.0)) w->compression = IVP_REAL(0.0);
        if (w->compression > IVP_REAL(1.0)) w->compression = IVP_REAL(1.0);

        /* Spring force = k * (compression * travel + pre_tension) */
        ivp_real_t spring_disp = w->compression * w->suspension_travel +
                                  w->spring_pre_tension;
        ivp_real_t spring_force = w->spring_constant * spring_disp;

        /* Velocity difference along suspension axis */
        ivp_vec3r_t body_vel_at_hp;
        ivp_core_get_surface_speed_ws(body, &hp_ws, &body_vel_at_hp);

        ivp_vec3r_t rel_vel;
        ivp_vec3r_sub(&rel_vel, &wheel->speed, &body_vel_at_hp);
        ivp_real_t vel_susp = ivp_vec3r_dot(&rel_vel, &susp_dir_ws);

        /* Damping: different for compression vs extension */
        ivp_real_t damp_coeff = (vel_susp > IVP_REAL(0.0))
            ? w->spring_damping_compression : w->spring_damping;
        ivp_real_t damp_force = -damp_coeff * vel_susp;

        /* Total suspension force (only push apart, never pull together) */
        ivp_real_t total_force = spring_force + damp_force;
        if (total_force < IVP_REAL(0.0)) total_force = IVP_REAL(0.0);
        if (total_force > w->max_body_force) total_force = w->max_body_force;

        /* Apply equal and opposite impulses to body and wheel */
        ivp_real_t impulse = total_force * dt;
        ivp_vec3r_t push_up;
        ivp_vec3r_scale(&push_up, &susp_dir_ws, -impulse);

        ivp_core_push_ws(body, &hp_ws, &push_up);

        /* Push wheel down (opposite direction) */
        ivp_vec3r_t push_down;
        ivp_vec3r_scale(&push_down, &susp_dir_ws, impulse);
        ivp_core_center_push(wheel, &push_down);

        /* Lateral constraint: keep wheel aligned with hardpoint horizontally */
        ivp_vec3r_t lateral_error;
        ivp_vec3r_sub(&lateral_error, &hp_ws, &wheel->pos_world_f_core_last_psi);
        /* Remove suspension-axis component */
        ivp_real_t along_susp = ivp_vec3r_dot(&lateral_error, &susp_dir_ws);
        lateral_error.k[0] -= susp_dir_ws.k[0] * along_susp;
        lateral_error.k[1] -= susp_dir_ws.k[1] * along_susp;
        lateral_error.k[2] -= susp_dir_ws.k[2] * along_susp;

        /* Apply position correction as velocity impulse */
        ivp_real_t inv_dt = IVP_REAL(1.0) / dt;
        ivp_real_t correction_factor = IVP_REAL(0.5);
        ivp_vec3r_t lat_impulse;
        for (int a = 0; a < 3; a++) {
            ivp_real_t desired_vel = lateral_error.k[a] * inv_dt * correction_factor;
            ivp_real_t current_vel = wheel->speed.k[a] - body_vel_at_hp.k[a];
            /* Remove suspension component from current_vel */
            ivp_real_t susp_component = susp_dir_ws.k[a] *
                ivp_vec3r_dot(&rel_vel, &susp_dir_ws);
            current_vel -= susp_component;

            lat_impulse.k[a] = (desired_vel - current_vel) *
                                (IVP_REAL(1.0) / wheel->inv_mass);
        }
        ivp_core_center_push(wheel, &lat_impulse);
    }
}

static void simulate_stabilizers(ivp_car_system_t *car, ivp_real_t dt)
{
    ivp_core_t *body = &car->body->core;

    for (int ax = 0; ax < car->n_axes; ax++) {
        ivp_car_system_axle_t *axle = &car->axles[ax];
        if (axle->stabilizer_constant < IVP_REAL_EPS) continue;
        if (axle->left_wheel < 0 || axle->right_wheel < 0) continue;

        ivp_car_system_wheel_t *left = &car->wheels[axle->left_wheel];
        ivp_car_system_wheel_t *right = &car->wheels[axle->right_wheel];

        ivp_real_t diff = left->compression - right->compression;
        ivp_real_t force = axle->stabilizer_constant * diff;

        /* Apply to body at hardpoints */
        ivp_vec3r_t hp_left_ws, hp_right_ws;
        ivp_transform_point(&hp_left_ws, &body->m_world_f_core_last_psi,
                            &left->hardpoint_cs);
        ivp_transform_point(&hp_right_ws, &body->m_world_f_core_last_psi,
                            &right->hardpoint_cs);

        ivp_vec3r_t susp_dir_cs = {{IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0)}};
        ivp_vec3r_t susp_dir_ws;
        ivp_transform_dir(&susp_dir_ws, &body->m_world_f_core_last_psi,
                          &susp_dir_cs);

        ivp_vec3r_t push;
        ivp_vec3r_scale(&push, &susp_dir_ws, -force * dt);
        ivp_core_push_ws(body, &hp_left_ws, &push);

        ivp_vec3r_scale(&push, &susp_dir_ws, force * dt);
        ivp_core_push_ws(body, &hp_right_ws, &push);
    }
}

static void wheel_axis_with_steering_cs(const ivp_car_system_wheel_t *w,
                                        ivp_vec3r_t *axis_cs_out)
{
    if (ivp_fabs(w->steering_angle) <= IVP_REAL_EPS) {
        *axis_cs_out = w->axis_cs;
        return;
    }

    /* Steering rotates wheel spin axis around body up (Y) in chassis space. */
    ivp_real_t s = ivp_sin(w->steering_angle);
    ivp_real_t c = ivp_cos(w->steering_angle);
    const ivp_vec3r_t *a = &w->axis_cs;

    axis_cs_out->k[0] = a->k[0] * c + a->k[2] * s;
    axis_cs_out->k[1] = a->k[1];
    axis_cs_out->k[2] = -a->k[0] * s + a->k[2] * c;
}

static void simulate_wheel_forces(ivp_car_system_t *car, ivp_real_t dt)
{
    ivp_core_t *body = &car->body->core;

    for (int i = 0; i < car->n_wheels; i++) {
        ivp_car_system_wheel_t *w = &car->wheels[i];
        if (!w->wheel_obj) continue;
        ivp_core_t *wheel = &w->wheel_obj->core;

        ivp_vec3r_t axis_cs;
        wheel_axis_with_steering_cs(w, &axis_cs);
        ivp_vec3r_t axis_ws;
        ivp_transform_dir(&axis_ws, &body->m_world_f_core_last_psi, &axis_cs);

        /* Engine torque: apply as angular impulse on wheel rotation axis */
        if (ivp_fabs(w->torque) > IVP_REAL_EPS && !w->fixed) {
            /* Angular impulse on wheel */
            ivp_vec3r_t ang_impulse_ws;
            ivp_vec3r_scale(&ang_impulse_ws, &axis_ws,
                            w->torque * dt * w->reversed_sign);

            /* Transform to wheel core space */
            ivp_vec3r_t ang_impulse_cs;
            ivp_mat3r_vimult(&ang_impulse_cs,
                              &wheel->m_world_f_core_last_psi.rot,
                              &ang_impulse_ws);

            ivp_core_rot_push_cs(wheel, &ang_impulse_cs);

            /* Reaction torque on body */
            ivp_vec3r_t body_torque_ws;
            ivp_vec3r_scale(&body_torque_ws, &axis_ws,
                            -w->torque * dt * car->body_counter_torque_factor);

            ivp_vec3r_t body_torque_cs;
            ivp_mat3r_vimult(&body_torque_cs,
                              &body->m_world_f_core_last_psi.rot,
                              &body_torque_ws);
            ivp_core_rot_push_cs(body, &body_torque_cs);
        }

        /* Brake: add angular damping to wheel */
        if (w->fixed) {
            ivp_real_t damp = IVP_REAL(0.9);
            wheel->rot_speed.k[0] *= (float)(IVP_REAL(1.0) - damp);
            wheel->rot_speed.k[1] *= (float)(IVP_REAL(1.0) - damp);
            wheel->rot_speed.k[2] *= (float)(IVP_REAL(1.0) - damp);
        }

        /* Track angular velocity for queries */
        ivp_vec3r_t omega_ws;
        omega_ws.k[0] = (ivp_real_t)wheel->rot_speed.k[0];
        omega_ws.k[1] = (ivp_real_t)wheel->rot_speed.k[1];
        omega_ws.k[2] = (ivp_real_t)wheel->rot_speed.k[2];
        ivp_vec3r_t omega_ws2;
        ivp_mat3r_vmult(&omega_ws2, &wheel->m_world_f_core_last_psi.rot,
                         &omega_ws);
        w->angular_velocity = ivp_vec3r_dot(&omega_ws2, &axis_ws);
    }
}

static void simulate_extra_forces(ivp_car_system_t *car, ivp_real_t dt)
{
    ivp_core_t *body = &car->body->core;

    /* Extra gravity */
    if (ivp_fabs(car->extra_gravity_force) > IVP_REAL_EPS) {
        ivp_vec3r_t grav = {{
            IVP_REAL(0.0),
            car->extra_gravity_force * dt,
            IVP_REAL(0.0)
        }};
        ivp_core_center_push(body, &grav);
    }

    /* Booster */
    if (car->booster_remaining > IVP_REAL(0.0)) {
        ivp_vec3r_t forward_cs = {{IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(-1.0)}};
        ivp_vec3r_t forward_ws;
        ivp_transform_dir(&forward_ws, &body->m_world_f_core_last_psi,
                          &forward_cs);

        ivp_real_t dv = car->booster_force * dt * body->inv_mass;
        body->speed.k[0] += forward_ws.k[0] * dv;
        body->speed.k[1] += forward_ws.k[1] * dv;
        body->speed.k[2] += forward_ws.k[2] * dv;

        car->booster_remaining -= dt;
        if (car->booster_remaining <= IVP_REAL(0.0)) {
            car->booster_remaining = IVP_REAL(0.0);
            car->booster_cooldown = car->booster_recharge;
        }
    } else if (car->booster_cooldown > IVP_REAL(0.0)) {
        car->booster_cooldown -= dt;
        if (car->booster_cooldown < IVP_REAL(0.0))
            car->booster_cooldown = IVP_REAL(0.0);
    }
}

/* ── Controller callback ──────────────────────────────────────────────── */

void ivp_controller_car_system(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;
    ivp_car_system_t *car = (ivp_car_system_t *)ctx;
    if (!car || !car->active) return;
    if (!car->body || obj != car->body) return;
    if (car->body->core.physical_unmoveable) return;
    if (dt <= IVP_REAL_EPS) return;

    simulate_suspension(car, dt);
    simulate_stabilizers(car, dt);
    simulate_wheel_forces(car, dt);
    simulate_extra_forces(car, dt);
}
