/* ivp_controller_fake_jetski.c - Legacy-style fake jetski raycast controller */

#include <ivp/controllers/ivp_controller_fake_jetski.h>

#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_transform.h>

#include <string.h>

typedef struct ivp_jetski_wheel_temp {
    ivp_real_t  friction_value;
    ivp_real_t  stabilizer_force;
    ivp_object_t *moveable_object_hit_by_ray;
    ivp_vec3r_t ground_normal_ws;
    ivp_vec3r_t ground_hit_ws;
    ivp_vec3r_t spring_direction_ws;
    ivp_vec3r_t surface_speed_wheel_ws;
    ivp_vec3r_t projected_surface_speed_wheel_ws;
    ivp_vec3r_t axis_direction_ws;
    ivp_vec3r_t projected_axis_direction_ws;
    ivp_real_t  inv_normal_dot_dir;
    bool        hit;
} ivp_jetski_wheel_temp_t;

typedef struct ivp_jetski_ray_data {
    ivp_vec3r_t start_ws;
    ivp_real_t  ray_length;
} ivp_jetski_ray_data_t;

static int jetski_sanitize_axis_index(int idx, int fallback) {
    if (idx < 0 || idx > 2) {
        return fallback;
    }
    return idx;
}

static bool jetski_valid_wheel_index(const ivp_jetski_state_t *state, ivp_jetski_wheel_pos_t pos) {
    const int idx = (int)pos;
    return idx >= 0 && idx < (int)state->n_wheels;
}

static bool jetski_valid_axis_index(const ivp_jetski_state_t *state, ivp_jetski_axis_pos_t pos) {
    const int idx = (int)pos;
    return idx >= 0 && idx < (int)state->n_axis;
}

static ivp_real_t jetski_inv_or_zero(ivp_real_t x) {
    if (ivp_fabs(x) <= IVP_REAL_EPS) {
        return IVP_REAL(0.0);
    }
    return IVP_REAL(1.0) / x;
}

static bool jetski_normalize_or_zero(ivp_vec3r_t *v) {
    if (ivp_vec3r_normalize(v) != IVP_OK) {
        ivp_vec3r_set_zero(v);
        return false;
    }
    return true;
}

static void jetski_set_local_axis_unit(ivp_vec3r_t *axis_cs, int axis_index) {
    ivp_vec3r_set_zero(axis_cs);
    if (axis_index >= 0 && axis_index < 3) {
        axis_cs->k[axis_index] = IVP_REAL(1.0);
    }
}

static void jetski_orthogonal_part(ivp_vec3r_t *out,
                                   const ivp_vec3r_t *v,
                                   const ivp_vec3r_t *normal) {
    const ivp_real_t d = ivp_vec3r_dot(v, normal);
    ivp_vec3r_t proj;
    ivp_vec3r_scale(&proj, normal, d);
    ivp_vec3r_sub(out, v, &proj);
}

static void jetski_inv_inertia_world_mul(const ivp_core_t *core,
                                         const ivp_vec3r_t *v_ws,
                                         ivp_vec3r_t *out_ws) {
    ivp_vec3r_t v_cs;
    ivp_vec3r_t out_cs;

    ivp_mat3r_vimult(&v_cs, &core->m_world_f_core_last_psi.rot, v_ws);
    out_cs.k[0] = v_cs.k[0] * core->inv_rot_inertia.k[0];
    out_cs.k[1] = v_cs.k[1] * core->inv_rot_inertia.k[1];
    out_cs.k[2] = v_cs.k[2] * core->inv_rot_inertia.k[2];
    ivp_mat3r_vmult(out_ws, &core->m_world_f_core_last_psi.rot, &out_cs);
}

static ivp_real_t jetski_constraint_diag_term(const ivp_core_t *core,
                                              const ivp_vec3r_t *point_ws,
                                              const ivp_vec3r_t *dir_ws,
                                              ivp_vec3r_t *cross_out) {
    ivp_vec3r_t r_ws;
    ivp_vec3r_t c_ws;
    ivp_vec3r_t inv_i_c_ws;

    ivp_vec3r_sub(&r_ws, point_ws, &core->pos_world_f_core_last_psi);
    ivp_vec3r_cross(&c_ws, &r_ws, dir_ws);
    if (cross_out) {
        *cross_out = c_ws;
    }

    jetski_inv_inertia_world_mul(core, &c_ws, &inv_i_c_ws);

    return core->inv_mass * ivp_vec3r_dot(dir_ws, dir_ws) + ivp_vec3r_dot(&c_ws, &inv_i_c_ws);
}

static ivp_real_t jetski_constraint_offdiag_term(const ivp_core_t *core,
                                                 const ivp_vec3r_t *dir_a_ws,
                                                 const ivp_vec3r_t *dir_b_ws,
                                                 const ivp_vec3r_t *cross_a_ws,
                                                 const ivp_vec3r_t *cross_b_ws) {
    ivp_vec3r_t inv_i_cross_b_ws;
    jetski_inv_inertia_world_mul(core, cross_b_ws, &inv_i_cross_b_ws);

    return core->inv_mass * ivp_vec3r_dot(dir_a_ws, dir_b_ws) +
           ivp_vec3r_dot(cross_a_ws, &inv_i_cross_b_ws);
}

static bool jetski_solve_2x2(ivp_real_t m00, ivp_real_t m01,
                             ivp_real_t m10, ivp_real_t m11,
                             ivp_real_t b0, ivp_real_t b1,
                             ivp_real_t *x0, ivp_real_t *x1) {
    const ivp_real_t det = m00 * m11 - m01 * m10;
    if (ivp_fabs(det) < IVP_REAL(1e-7)) {
        *x0 = IVP_REAL(0.0);
        *x1 = IVP_REAL(0.0);
        return false;
    }

    const ivp_real_t inv_det = IVP_REAL(1.0) / det;
    *x0 = (b0 * m11 - b1 * m01) * inv_det;
    *x1 = (b1 * m00 - b0 * m10) * inv_det;
    return true;
}
static void jetski_setup_wheel_raycasts(ivp_jetski_state_t *state,
                                        const ivp_core_t *core,
                                        ivp_jetski_wheel_temp_t *temp_wheels,
                                        ivp_jetski_ray_data_t *ray_data,
                                        ivp_jetski_raycast_result_t *ray_hits) {
    for (int i = 0; i < (int)state->n_wheels; ++i) {
        ivp_jetski_wheel_t *wheel = &state->wheels[i];
        ivp_jetski_wheel_temp_t *temp = &temp_wheels[i];
        ivp_jetski_ray_data_t *ray = &ray_data[i];
        ivp_jetski_raycast_result_t *hit = &ray_hits[i];

        memset(temp, 0, sizeof(*temp));
        memset(hit, 0, sizeof(*hit));

        ivp_transform_point(&ray->start_ws, &core->m_world_f_core_last_psi, &wheel->raycast_start_cs);
        ivp_transform_dir(&temp->spring_direction_ws, &core->m_world_f_core_last_psi, &wheel->spring_direction_cs);

        if (!jetski_normalize_or_zero(&temp->spring_direction_ws)) {
            jetski_set_local_axis_unit(&temp->spring_direction_ws, state->index_y);
        }

        const ivp_real_t height_delta =
            wheel->hp_cs.k[state->index_y] - wheel->raycast_start_cs.k[state->index_y];
        ray->ray_length = wheel->spring_len + wheel->wheel_radius + height_delta;
        if (ray->ray_length < IVP_REAL(0.0)) {
            ray->ray_length = IVP_REAL(0.0);
        }

        hit->friction = IVP_REAL(1.0);
        hit->distance = ray->ray_length;
        ivp_vec3r_negate(&hit->hit_normal, &temp->spring_direction_ws);

        if (state->raycast_fn) {
            state->raycast_fn(state->raycast_ctx,
                              &ray->start_ws,
                              &temp->spring_direction_ws,
                              ray->ray_length,
                              hit);

            if (!hit->hit) {
                hit->distance = ray->ray_length;
            }

            if (hit->distance < IVP_REAL(0.0)) {
                hit->distance = IVP_REAL(0.0);
            }
            if (hit->distance > ray->ray_length) {
                hit->distance = ray->ray_length;
            }
            if (hit->friction <= IVP_REAL_EPS) {
                hit->friction = IVP_REAL(1.0);
            }
        }
    }
}

static bool jetski_do_simulation_wheels(ivp_jetski_state_t *state,
                                        const ivp_core_t *core,
                                        ivp_jetski_wheel_temp_t *temp_wheels,
                                        const ivp_jetski_ray_data_t *ray_data,
                                        const ivp_jetski_raycast_result_t *ray_hits) {
    for (int i = 0; i < (int)state->n_wheels; ++i) {
        ivp_jetski_wheel_t *wheel = &state->wheels[i];
        ivp_jetski_wheel_temp_t *temp = &temp_wheels[i];
        const ivp_jetski_ray_data_t *ray = &ray_data[i];
        const ivp_jetski_raycast_result_t *hit = &ray_hits[i];

        const ivp_real_t height_delta =
            wheel->hp_cs.k[state->index_y] - wheel->raycast_start_cs.k[state->index_y];

        temp->hit = hit->hit;

        if (hit->hit) {
            temp->ground_normal_ws = hit->hit_normal;
            if (!jetski_normalize_or_zero(&temp->ground_normal_ws)) {
                ivp_vec3r_negate(&temp->ground_normal_ws, &temp->spring_direction_ws);
            }

            wheel->raycast_dist = hit->distance - height_delta;
            temp->inv_normal_dot_dir = IVP_REAL(1.1) /
                (ivp_fabs(ivp_vec3r_dot(&temp->spring_direction_ws, &temp->ground_normal_ws)) + IVP_REAL(0.1));

            if (hit->hit_object && !hit->hit_object->core.physical_unmoveable) {
                temp->moveable_object_hit_by_ray = hit->hit_object;
            }
        } else {
            wheel->pressure = IVP_REAL(0.0);
            wheel->raycast_dist = wheel->spring_len + wheel->wheel_radius - height_delta;
            temp->inv_normal_dot_dir = IVP_REAL(1.0);
            temp->moveable_object_hit_by_ray = NULL;
            ivp_vec3r_negate(&temp->ground_normal_ws, &temp->spring_direction_ws);
        }

        temp->friction_value = hit->friction * wheel->friction_of_wheel;

        ivp_vec3r_t wheel_pos_ws;
        ivp_transform_point(&wheel_pos_ws, &core->m_world_f_core_last_psi, &wheel->hp_cs);
        ivp_vec3r_scale(&temp->ground_hit_ws, &temp->spring_direction_ws, wheel->raycast_dist);
        ivp_vec3r_add(&temp->ground_hit_ws, &temp->ground_hit_ws, &wheel_pos_ws);

        if (hit->hit) {
            temp->ground_hit_ws.k[state->index_y] =
                ray->start_ws.k[state->index_y] + temp->spring_direction_ws.k[state->index_y] * hit->distance;
        }

        ivp_core_get_surface_speed_ws(core, &temp->ground_hit_ws, &temp->surface_speed_wheel_ws);
        jetski_orthogonal_part(&temp->projected_surface_speed_wheel_ws,
                               &temp->surface_speed_wheel_ws,
                               &temp->ground_normal_ws);

        ivp_transform_dir(&temp->axis_direction_ws, &core->m_world_f_core_last_psi, &wheel->axis_direction_cs);
        jetski_orthogonal_part(&temp->projected_axis_direction_ws,
                               &temp->axis_direction_ws,
                               &temp->ground_normal_ws);

        if (!jetski_normalize_or_zero(&temp->projected_axis_direction_ws)) {
            return false;
        }
    }

    return true;
}

static void jetski_do_simulation_stabilizers(ivp_jetski_state_t *state,
                                             ivp_jetski_wheel_temp_t *temp_wheels) {
    if (state->wheels_per_axis == 2) {
        for (int axis = 0; axis < (int)state->n_axis; ++axis) {
            ivp_jetski_wheel_t *wheel0 = &state->wheels[axis * state->wheels_per_axis + 0];
            ivp_jetski_wheel_t *wheel1 = &state->wheels[axis * state->wheels_per_axis + 1];

            const ivp_real_t diff0 = wheel0->raycast_dist - wheel0->spring_len - wheel0->wheel_radius;
            const ivp_real_t diff1 = wheel1->raycast_dist - wheel1->spring_len - wheel1->wheel_radius;

            temp_wheels[2 * axis].stabilizer_force =
                (diff1 - diff0) * (state->axles[axis].stabilizer_constant * IVP_REAL(0.5));
            temp_wheels[2 * axis + 1].stabilizer_force = -temp_wheels[2 * axis].stabilizer_force;
        }
    } else {
        for (int i = 0; i < (int)state->n_wheels; ++i) {
            temp_wheels[i].stabilizer_force = IVP_REAL(0.0);
        }
    }
}

static void jetski_do_simulation_shocks(ivp_jetski_state_t *state,
                                        ivp_jetski_wheel_temp_t *temp_wheels,
                                        const ivp_jetski_raycast_result_t *ray_hits,
                                        ivp_real_t dt,
                                        ivp_core_t *core) {
    for (int i = 0; i < (int)state->n_wheels; ++i) {
        ivp_jetski_wheel_t *wheel = &state->wheels[i];
        ivp_jetski_wheel_temp_t *temp = &temp_wheels[i];

        if (!ray_hits[i].hit) {
            continue;
        }

        const ivp_real_t diff = wheel->raycast_dist - wheel->spring_len - wheel->wheel_radius;
        if (diff >= IVP_REAL(0.0)) {
            continue;
        }

        ivp_real_t force = -diff * wheel->spring_constant + temp->stabilizer_force;

        ivp_real_t inv_normal_dot_dir = ivp_clamp(temp->inv_normal_dot_dir, IVP_REAL(0.0), IVP_REAL(3.0));
        force *= inv_normal_dot_dir;

        ivp_vec3r_t diff_speed;
        ivp_vec3r_sub(&diff_speed,
                      &temp->projected_surface_speed_wheel_ws,
                      &temp->surface_speed_wheel_ws);

        const ivp_real_t speed = ivp_vec3r_dot(&diff_speed, &temp->spring_direction_ws);
        if (speed > IVP_REAL(0.0)) {
            force -= wheel->spring_damp_relax * speed;
        } else {
            force -= wheel->spring_damp_compress * speed;
        }

        if (force < IVP_REAL(0.0)) {
            force = IVP_REAL(0.0);
        }

        wheel->pressure = force;

        ivp_vec3r_t impulse_ws;
        ivp_vec3r_scale(&impulse_ws, &temp->ground_normal_ws, force * dt);
        ivp_core_push_ws(core, &temp->ground_hit_ws, &impulse_ws);
    }
}
static void jetski_do_simulation_booster(ivp_jetski_state_t *state,
                                         ivp_real_t dt,
                                         ivp_core_t *core) {
    if (state->booster_seconds_until_ready > IVP_REAL(0.0)) {
        state->booster_seconds_until_ready -= dt;
        if (state->booster_seconds_until_ready < IVP_REAL(0.0)) {
            state->booster_seconds_until_ready = IVP_REAL(0.0);
        }
    }

    if (state->booster_seconds_to_go > IVP_REAL(0.0)) {
        state->booster_seconds_to_go -= dt;
        if (state->booster_seconds_to_go <= IVP_REAL(0.0)) {
            state->booster_seconds_to_go = IVP_REAL(0.0);
            state->booster_force = IVP_REAL(0.0);
        }
    }

    if (ivp_fabs(state->booster_force) > IVP_REAL_EPS) {
        ivp_vec3r_t booster_dir_cs;
        ivp_vec3r_t booster_dir_ws;
        jetski_set_local_axis_unit(&booster_dir_cs, state->index_z);
        ivp_transform_dir(&booster_dir_ws, &core->m_world_f_core_last_psi, &booster_dir_cs);
        ivp_vec3r_add_scaled(&core->speed, &booster_dir_ws, state->booster_force * dt);
    }
}

static void jetski_calc_steering_forces(ivp_jetski_state_t *state,
                                        ivp_jetski_wheel_temp_t *temp_wheels,
                                        const ivp_core_t *core,
                                        ivp_real_t dt,
                                        ivp_real_t *forces_needed_to_drive_straight) {
    for (int i = 0; i < IVP_JETSKI_MAX_AXES; ++i) {
        forces_needed_to_drive_straight[i] = IVP_REAL(0.0);
    }

    if (state->n_axis <= 0) {
        return;
    }

    ivp_vec3r_t front_pos_ws;
    ivp_vec3r_t back_pos_ws;
    ivp_vec3r_t front_axis_ws;
    ivp_vec3r_t back_axis_ws;

    if (state->wheels_per_axis == 2 && state->n_wheels >= 4) {
        ivp_vec3r_lerp(&front_pos_ws,
                       &temp_wheels[IVP_JETSKI_FRONT_LEFT].ground_hit_ws,
                       &temp_wheels[IVP_JETSKI_FRONT_RIGHT].ground_hit_ws,
                       IVP_REAL(0.5));
        ivp_vec3r_lerp(&back_pos_ws,
                       &temp_wheels[IVP_JETSKI_REAR_LEFT].ground_hit_ws,
                       &temp_wheels[IVP_JETSKI_REAR_RIGHT].ground_hit_ws,
                       IVP_REAL(0.5));
        front_axis_ws = temp_wheels[IVP_JETSKI_FRONT_LEFT].axis_direction_ws;
        back_axis_ws = temp_wheels[IVP_JETSKI_REAR_LEFT].axis_direction_ws;
    } else {
        front_pos_ws = temp_wheels[0].ground_hit_ws;
        back_pos_ws = temp_wheels[1].ground_hit_ws;
        front_axis_ws = temp_wheels[0].axis_direction_ws;
        back_axis_ws = temp_wheels[1].axis_direction_ws;
    }

    ivp_vec3r_t front_vel_ws;
    ivp_vec3r_t back_vel_ws;
    ivp_core_get_surface_speed_ws(core, &front_pos_ws, &front_vel_ws);
    ivp_core_get_surface_speed_ws(core, &back_pos_ws, &back_vel_ws);

    const ivp_real_t front_speed = ivp_vec3r_dot(&front_vel_ws, &front_axis_ws);
    const ivp_real_t back_speed = ivp_vec3r_dot(&back_vel_ws, &back_axis_ws);

    ivp_vec3r_t cross_front_ws;
    ivp_vec3r_t cross_back_ws;
    const ivp_real_t m00 = jetski_constraint_diag_term(core, &front_pos_ws, &front_axis_ws, &cross_front_ws);
    const ivp_real_t m11 = jetski_constraint_diag_term(core, &back_pos_ws, &back_axis_ws, &cross_back_ws);
    const ivp_real_t m01 = jetski_constraint_offdiag_term(core,
                                                          &front_axis_ws,
                                                          &back_axis_ws,
                                                          &cross_front_ws,
                                                          &cross_back_ws);

    const ivp_real_t relaxation_rate = IVP_REAL(1.2);
    const ivp_real_t inv_dt = jetski_inv_or_zero(dt);
    const ivp_real_t a = -front_speed * inv_dt * relaxation_rate;
    const ivp_real_t b = -back_speed * inv_dt * relaxation_rate;

    ivp_real_t force_front = IVP_REAL(0.0);
    ivp_real_t force_back = IVP_REAL(0.0);
    (void)jetski_solve_2x2(m00, m01, m01, m11, a, b, &force_front, &force_back);

    forces_needed_to_drive_straight[0] = force_front;
    if (state->n_axis > 1) {
        forces_needed_to_drive_straight[1] = force_back;
    }
}

static void jetski_turn_in_place_hack(ivp_jetski_state_t *state,
                                      ivp_jetski_wheel_temp_t *temp_wheels,
                                      ivp_core_t *core,
                                      ivp_real_t dt) {
    for (int i = 0; i < (int)state->n_wheels; ++i) {
        const ivp_jetski_wheel_t *wheel = &state->wheels[i];
        if (wheel->wheel_is_fixed || ivp_fabs(wheel->torque) > IVP_REAL_EPS) {
            return;
        }
    }

    const ivp_real_t speed = ivp_vec3r_length(&core->speed);
    if (speed >= IVP_REAL(1.0)) {
        return;
    }

    const ivp_real_t inv_dt = jetski_inv_or_zero(dt);

    for (int i = 0; i < (int)state->n_wheels; ++i) {
        ivp_jetski_wheel_t *wheel = &state->wheels[i];
        ivp_jetski_wheel_temp_t *temp = &temp_wheels[i];

        ivp_vec3r_t projected_front_dir_ws;
        ivp_vec3r_cross(&projected_front_dir_ws,
                        &temp->projected_axis_direction_ws,
                        &temp->spring_direction_ws);
        if (!jetski_normalize_or_zero(&projected_front_dir_ws)) {
            continue;
        }

        const ivp_real_t body_speed = ivp_vec3r_dot(&core->speed, &projected_front_dir_ws);
        const ivp_real_t max_force = wheel->pressure * temp->friction_value;

        ivp_real_t force = IVP_REAL(0.0);
        if (ivp_fabs(state->steering_angle) > IVP_REAL(0.1)) {
            const ivp_real_t sign = (body_speed >= IVP_REAL(0.0)) ? IVP_REAL(1.0) : IVP_REAL(-1.0);
            const ivp_real_t friction_force = temp->friction_value * (IVP_REAL(0.25) * core->mass) * sign;
            force = -IVP_REAL(2.5) * friction_force + IVP_REAL(750.0);

            const ivp_real_t force_sq = force * force;
            const ivp_real_t max_force_sq = max_force * max_force;
            if (force_sq > max_force_sq && force_sq > IVP_REAL_EPS) {
                force *= ivp_sqrt(max_force_sq / force_sq);
            }
        } else {
            force = -IVP_REAL(0.5) * body_speed * core->mass * inv_dt;
        }

        ivp_vec3r_t impulse_ws;
        ivp_vec3r_scale(&impulse_ws, &projected_front_dir_ws, force * dt);
        ivp_core_push_ws(core, &temp->ground_hit_ws, &impulse_ws);
    }
}

static void jetski_apply_steering_forces(ivp_jetski_state_t *state,
                                         ivp_jetski_wheel_temp_t *temp_wheels,
                                         ivp_core_t *core,
                                         ivp_real_t dt,
                                         ivp_real_t *forces_needed_to_drive_straight) {
    const ivp_real_t inv_dt = jetski_inv_or_zero(dt);

    for (int i = 0; i < (int)state->n_wheels; ++i) {
        ivp_jetski_wheel_t *wheel = &state->wheels[i];
        ivp_jetski_wheel_temp_t *temp = &temp_wheels[i];

        const ivp_real_t max_force = wheel->pressure * temp->friction_value;

        ivp_vec3r_t projected_front_dir_ws;
        ivp_vec3r_cross(&projected_front_dir_ws,
                        &temp->projected_axis_direction_ws,
                        &temp->spring_direction_ws);
        if (!jetski_normalize_or_zero(&projected_front_dir_ws)) {
            continue;
        }

        const ivp_real_t body_speed = ivp_vec3r_dot(&core->speed, &projected_front_dir_ws);

        ivp_real_t fr_force;
        if (wheel->wheel_is_fixed) {
            fr_force = -IVP_REAL(0.5) * body_speed * core->mass * inv_dt;
            wheel->wheel_angular_velocity = IVP_REAL(0.0);
        } else {
            wheel->wheel_angular_velocity = body_speed * wheel->inv_wheel_radius;
            const ivp_real_t sign = (body_speed >= IVP_REAL(0.0)) ? IVP_REAL(1.0) : IVP_REAL(-1.0);
            const ivp_real_t friction_force = temp->friction_value * (IVP_REAL(0.25) * core->mass) * sign;
            fr_force = -IVP_REAL(2.5) * friction_force;
            fr_force += wheel->torque * wheel->inv_wheel_radius;
        }

        int axis_index = 0;
        if (state->wheels_per_axis > 0) {
            axis_index = i / state->wheels_per_axis;
        }
        if (axis_index >= IVP_JETSKI_MAX_AXES) {
            axis_index = IVP_JETSKI_MAX_AXES - 1;
        }

        ivp_real_t force_rotational = forces_needed_to_drive_straight[axis_index];
        ivp_real_t quad_sum = fr_force * fr_force + force_rotational * force_rotational;
        ivp_real_t max_force_sq = max_force * max_force;
        if (quad_sum > max_force_sq && quad_sum > IVP_REAL_EPS) {
            const ivp_real_t factor = ivp_sqrt(max_force_sq / quad_sum);
            fr_force *= factor;
            force_rotational *= factor;
        }

        forces_needed_to_drive_straight[axis_index] -= force_rotational;

        wheel->angle_wheel -= wheel->wheel_angular_velocity * dt;

        ivp_vec3r_t impulse_ws;
        ivp_vec3r_scale(&impulse_ws, &temp->projected_axis_direction_ws, force_rotational * dt);
        ivp_core_push_ws(core, &temp->ground_hit_ws, &impulse_ws);
        ivp_vec3r_scale(&impulse_ws, &projected_front_dir_ws, fr_force * dt);
        ivp_core_push_ws(core, &temp->ground_hit_ws, &impulse_ws);
    }

    jetski_turn_in_place_hack(state, temp_wheels, core, dt);
}

static void jetski_do_simulation_steering(ivp_jetski_state_t *state,
                                          ivp_jetski_wheel_temp_t *temp_wheels,
                                          ivp_core_t *core,
                                          ivp_real_t dt) {
    ivp_real_t forces_needed_to_drive_straight[IVP_JETSKI_MAX_AXES] = {
        IVP_REAL(0.0), IVP_REAL(0.0)
    };

    jetski_calc_steering_forces(state, temp_wheels, core, dt, forces_needed_to_drive_straight);
    jetski_apply_steering_forces(state, temp_wheels, core, dt, forces_needed_to_drive_straight);
}
void ivp_jetski_set_axes(ivp_jetski_state_t *state,
                         int index_x,
                         int index_y,
                         int index_z,
                         bool is_left_handed) {
    if (!state) {
        return;
    }

    state->index_x = jetski_sanitize_axis_index(index_x, 0);
    state->index_y = jetski_sanitize_axis_index(index_y, 1);
    state->index_z = jetski_sanitize_axis_index(index_z, 2);
    state->is_left_handed = is_left_handed;
}

void ivp_jetski_set_gravity(ivp_jetski_state_t *state, const ivp_vec3r_t *gravity_ws) {
    if (!state || !gravity_ws) {
        return;
    }

    state->normized_gravity_ws = *gravity_ws;
    if (!jetski_normalize_or_zero(&state->normized_gravity_ws)) {
        jetski_set_local_axis_unit(&state->normized_gravity_ws, state->index_y);
    }
    state->gravity_y_direction = (gravity_ws->k[state->index_y] > IVP_REAL(0.0))
        ? IVP_REAL(1.0)
        : IVP_REAL(-1.0);
}

void ivp_jetski_init(ivp_jetski_state_t *state,
                     ivp_object_t *body,
                     ivp_real_t length,
                     ivp_real_t width,
                     ivp_real_t suspension_length) {
    memset(state, 0, sizeof(*state));

    state->body = body;
    state->n_wheels = 4;
    state->n_axis = 2;
    state->wheels_per_axis = 2;
    state->active = true;

    ivp_jetski_set_axes(state, 0, 1, 2, false);

    ivp_vec3r_t default_gravity;
    ivp_vec3r_set(&default_gravity, IVP_REAL(0.0), IVP_REAL(9.81), IVP_REAL(0.0));
    ivp_jetski_set_gravity(state, &default_gravity);

    state->extra_gravity = IVP_REAL(0.0);
    state->down_force = IVP_REAL(0.0);
    state->down_force_vertical_offset = IVP_REAL(0.0);
    state->booster_force = IVP_REAL(0.0);
    state->booster_seconds_to_go = IVP_REAL(0.0);
    state->booster_seconds_until_ready = IVP_REAL(0.0);
    state->steering_angle = IVP_REAL(-1.0);
    state->steering_rate = IVP_REAL(0.005);

    const ivp_real_t half_length = length * IVP_REAL(0.5);
    const ivp_real_t half_width = width * IVP_REAL(0.5);

    const ivp_vec3r_t positions[IVP_JETSKI_MAX_WHEELS] = {
        {{-half_width, IVP_REAL(0.0), -half_length}},
        {{ half_width, IVP_REAL(0.0), -half_length}},
        {{-half_width, IVP_REAL(0.0),  half_length}},
        {{ half_width, IVP_REAL(0.0),  half_length}}
    };

    for (int i = 0; i < (int)state->n_wheels; ++i) {
        ivp_jetski_wheel_t *wheel = &state->wheels[i];
        memset(wheel, 0, sizeof(*wheel));

        wheel->hp_cs = positions[i];
        wheel->raycast_start_cs = positions[i];
        ivp_vec3r_set(&wheel->spring_direction_cs, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
        wheel->distance_orig_hp_to_hp = suspension_length;
        wheel->spring_len = suspension_length;
        wheel->spring_constant = IVP_REAL(50000.0);
        wheel->spring_damp_relax = IVP_REAL(3000.0);
        wheel->spring_damp_compress = IVP_REAL(3000.0);
        wheel->max_rotation_speed = IVP_REAL(100.0);
        wheel->wheel_radius = IVP_REAL(0.3);
        wheel->inv_wheel_radius = jetski_inv_or_zero(wheel->wheel_radius);
        wheel->friction_of_wheel = IVP_REAL(1.0);
        wheel->wheel_is_fixed = true;
        wheel->pressure = IVP_REAL(0.0);
        ivp_vec3r_set_zero(&wheel->axis_direction_cs);
        wheel->axis_direction_cs.k[state->index_x] = IVP_REAL(1.0);
    }

    state->axles[0].stabilizer_constant = IVP_REAL(500.0);
    state->axles[1].stabilizer_constant = IVP_REAL(500.0);

    ivp_jetski_do_steering(state, IVP_REAL(0.0));
}

void ivp_jetski_do_steering_wheel(ivp_jetski_state_t *state,
                                  ivp_jetski_wheel_pos_t wheel_pos,
                                  ivp_real_t steering_angle) {
    if (!state || !jetski_valid_wheel_index(state, wheel_pos)) {
        return;
    }

    ivp_jetski_wheel_t *wheel = &state->wheels[(int)wheel_pos];
    ivp_vec3r_set_zero(&wheel->axis_direction_cs);
    wheel->axis_direction_cs.k[state->index_x] = IVP_REAL(1.0);

    ivp_mat3r_t steer_matrix;
    ivp_mat3r_init_rotation(&steer_matrix, (ivp_coord_index_t)state->index_y, steering_angle);
    ivp_mat3r_vmult(&wheel->axis_direction_cs, &steer_matrix, &wheel->axis_direction_cs);
}

void ivp_jetski_do_steering(ivp_jetski_state_t *state, ivp_real_t steering_angle) {
    if (!state) {
        return;
    }
    if (state->steering_angle == steering_angle) {
        return;
    }

    state->steering_angle = steering_angle;
    for (int i = 0; i < state->wheels_per_axis && i < (int)state->n_wheels; ++i) {
        ivp_jetski_do_steering_wheel(state, (ivp_jetski_wheel_pos_t)i, state->steering_angle);
    }
}

void ivp_jetski_change_spring_constant(ivp_jetski_state_t *state,
                                       ivp_jetski_wheel_pos_t pos,
                                       ivp_real_t spring_constant) {
    if (!state || !jetski_valid_wheel_index(state, pos)) {
        return;
    }
    state->wheels[(int)pos].spring_constant = spring_constant;
}

void ivp_jetski_change_spring_dampening(ivp_jetski_state_t *state,
                                        ivp_jetski_wheel_pos_t pos,
                                        ivp_real_t spring_dampening) {
    if (!state || !jetski_valid_wheel_index(state, pos)) {
        return;
    }
    state->wheels[(int)pos].spring_damp_relax = spring_dampening;
}

void ivp_jetski_change_spring_dampening_compression(ivp_jetski_state_t *state,
                                                    ivp_jetski_wheel_pos_t pos,
                                                    ivp_real_t spring_dampening) {
    if (!state || !jetski_valid_wheel_index(state, pos)) {
        return;
    }
    state->wheels[(int)pos].spring_damp_compress = spring_dampening;
}

void ivp_jetski_change_spring_pre_tension(ivp_jetski_state_t *state,
                                          ivp_jetski_wheel_pos_t pos,
                                          ivp_real_t pre_tension_length) {
    if (!state || !jetski_valid_wheel_index(state, pos)) {
        return;
    }

    ivp_jetski_wheel_t *wheel = &state->wheels[(int)pos];
    wheel->spring_len = state->gravity_y_direction *
                        (wheel->distance_orig_hp_to_hp - pre_tension_length);
}

void ivp_jetski_change_spring_length(ivp_jetski_state_t *state,
                                     ivp_jetski_wheel_pos_t pos,
                                     ivp_real_t spring_length) {
    if (!state || !jetski_valid_wheel_index(state, pos)) {
        return;
    }
    state->wheels[(int)pos].spring_len = spring_length;
}

void ivp_jetski_change_wheel_torque(ivp_jetski_state_t *state,
                                    ivp_jetski_wheel_pos_t pos,
                                    ivp_real_t torque) {
    if (!state || !jetski_valid_wheel_index(state, pos)) {
        return;
    }
    state->wheels[(int)pos].torque = torque;
}

void ivp_jetski_fix_wheel(ivp_jetski_state_t *state,
                          ivp_jetski_wheel_pos_t pos,
                          bool fixed) {
    if (!state || !jetski_valid_wheel_index(state, pos)) {
        return;
    }
    state->wheels[(int)pos].wheel_is_fixed = fixed;
}

void ivp_jetski_change_friction_of_wheel(ivp_jetski_state_t *state,
                                         ivp_jetski_wheel_pos_t pos,
                                         ivp_real_t friction) {
    if (!state || !jetski_valid_wheel_index(state, pos)) {
        return;
    }
    state->wheels[(int)pos].friction_of_wheel = friction;
}

void ivp_jetski_change_stabilizer_constant(ivp_jetski_state_t *state,
                                           ivp_jetski_axis_pos_t pos,
                                           ivp_real_t stabi_constant) {
    if (!state || !jetski_valid_axis_index(state, pos)) {
        return;
    }
    state->axles[(int)pos].stabilizer_constant = stabi_constant;
}

void ivp_jetski_change_fast_turn_factor(ivp_jetski_state_t *state,
                                        ivp_real_t fast_turn_factor) {
    (void)state;
    (void)fast_turn_factor;
}

void ivp_jetski_change_body_downforce(ivp_jetski_state_t *state, ivp_real_t force) {
    if (!state) {
        return;
    }
    state->down_force = force;
}
void ivp_jetski_set_booster_acceleration(ivp_jetski_state_t *state, ivp_real_t acceleration) {
    if (!state) {
        return;
    }
    state->booster_force = acceleration;
}

void ivp_jetski_activate_booster(ivp_jetski_state_t *state,
                                 ivp_real_t thrust,
                                 ivp_real_t duration,
                                 ivp_real_t delay) {
    if (!state) {
        return;
    }
    if (ivp_fabs(state->booster_force) > IVP_REAL_EPS) {
        return;
    }
    if (state->booster_seconds_until_ready > IVP_REAL(0.0)) {
        return;
    }

    state->booster_force = thrust;
    state->booster_seconds_to_go = duration;
    state->booster_seconds_until_ready = duration + delay;
}

ivp_real_t ivp_jetski_get_booster_delay(const ivp_jetski_state_t *state) {
    if (!state) {
        return IVP_REAL(0.0);
    }
    return state->booster_seconds_until_ready;
}

ivp_real_t ivp_jetski_get_wheel_angular_velocity(const ivp_jetski_state_t *state,
                                                 ivp_jetski_wheel_pos_t pos) {
    if (!state || !jetski_valid_wheel_index(state, pos)) {
        return IVP_REAL(0.0);
    }
    return state->wheels[(int)pos].wheel_angular_velocity;
}

ivp_real_t ivp_jetski_get_body_speed(const ivp_jetski_state_t *state, int axis_index) {
    if (!state || !state->body) {
        return IVP_REAL(0.0);
    }

    axis_index = jetski_sanitize_axis_index(axis_index, state->index_z);
    ivp_vec3r_t orientation_cs;
    ivp_vec3r_t orientation_ws;
    jetski_set_local_axis_unit(&orientation_cs, axis_index);
    ivp_transform_dir(&orientation_ws,
                      &state->body->core.m_world_f_core_last_psi,
                      &orientation_cs);
    return ivp_vec3r_dot(&orientation_ws, &state->body->core.speed);
}

ivp_real_t ivp_jetski_get_orig_front_wheel_distance(const ivp_jetski_state_t *state) {
    if (!state || state->n_wheels < 2) {
        return IVP_REAL(0.0);
    }
    const ivp_real_t dist =
        state->wheels[IVP_JETSKI_FRONT_LEFT].hp_cs.k[state->index_x] -
        state->wheels[IVP_JETSKI_FRONT_RIGHT].hp_cs.k[state->index_x];
    return ivp_fabs(dist);
}

ivp_real_t ivp_jetski_get_orig_axles_distance(const ivp_jetski_state_t *state) {
    if (!state || state->n_wheels < 3) {
        return IVP_REAL(0.0);
    }
    const ivp_real_t dist =
        state->wheels[IVP_JETSKI_FRONT_LEFT].hp_cs.k[state->index_z] -
        state->wheels[IVP_JETSKI_REAR_LEFT].hp_cs.k[state->index_z];
    return ivp_fabs(dist);
}

void ivp_jetski_get_skid_info(const ivp_jetski_state_t *state,
                              ivp_jetski_wheel_skid_info_t *array_of_skid_info_out) {
    if (!state || !array_of_skid_info_out) {
        return;
    }

    for (int i = 0; i < (int)state->n_wheels; ++i) {
        ivp_jetski_wheel_skid_info_t *info = &array_of_skid_info_out[i];
        ivp_vec3r_set_zero(&info->last_contact_position_ws);
        info->last_skid_value = IVP_REAL(0.0);
        info->last_skid_time = IVP_REAL(0.0);
    }
}

void ivp_controller_fake_jetski(const ivp_event_sim_t *event_sim,
                                ivp_object_t *obj,
                                void *ctx) {
    ivp_environment_t *env = event_sim->environment;
    const ivp_real_t dt = event_sim->delta_time;
    ivp_jetski_state_t *state = (ivp_jetski_state_t *)ctx;
    if (!state || !state->active) {
        return;
    }
    if (!state->body || obj != state->body) {
        return;
    }
    if (state->body->core.physical_unmoveable) {
        return;
    }
    if (dt <= IVP_REAL_EPS) {
        return;
    }

    if (env) {
        ivp_jetski_set_gravity(state, &env->gravity);
    }

    ivp_core_t *core = &state->body->core;

    ivp_jetski_wheel_temp_t temp_wheels[IVP_JETSKI_MAX_WHEELS];
    ivp_jetski_ray_data_t ray_data[IVP_JETSKI_MAX_WHEELS];
    ivp_jetski_raycast_result_t ray_hits[IVP_JETSKI_MAX_WHEELS];

    jetski_setup_wheel_raycasts(state, core, temp_wheels, ray_data, ray_hits);

    if (!jetski_do_simulation_wheels(state, core, temp_wheels, ray_data, ray_hits)) {
        return;
    }

    jetski_do_simulation_stabilizers(state, temp_wheels);
    ivp_vec3r_add_scaled(&core->speed,
                         &state->normized_gravity_ws,
                         state->extra_gravity * core->inv_mass * dt);
    jetski_do_simulation_shocks(state, temp_wheels, ray_hits, dt, core);
    jetski_do_simulation_booster(state, dt, core);
    jetski_do_simulation_steering(state, temp_wheels, core, dt);
}
