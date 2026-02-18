/* ivp_controller_golem.c — Animation-driven physics controller
 *
 * Computes velocity corrections each PSI to track target position
 * and orientation set by the animation system.
 */

#include <ivp/controllers/ivp_controller_golem.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <math.h>

void ivp_golem_state_init(ivp_golem_state_t *state)
{
    state->target_position    = ivp_vec3r_zero();
    state->target_orientation = ivp_quatr_identity();
    state->position_gain      = IVP_REAL(0.5);
    state->rotation_gain      = IVP_REAL(0.5);
    state->max_linear_speed   = IVP_REAL(100.0);
    state->max_angular_speed  = IVP_REAL(10.0);
    state->max_position_error = IVP_REAL(1.0);
    state->max_rotation_error = IVP_REAL(0.5);
    state->problem_fn         = NULL;
    state->problem_ctx        = NULL;
    state->target_position_set    = false;
    state->target_orientation_set = false;
}

void ivp_golem_set_target_position(ivp_golem_state_t *state,
                                   const ivp_vec3r_t *pos)
{
    state->target_position     = *pos;
    state->target_position_set = true;
}

void ivp_golem_set_target_orientation(ivp_golem_state_t *state,
                                      const ivp_quatr_t *orientation)
{
    state->target_orientation     = *orientation;
    state->target_orientation_set = true;
}

void ivp_controller_golem(const ivp_event_sim_t *event_sim,
                          ivp_object_t *obj,
                          void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;
    ivp_golem_state_t *state = (ivp_golem_state_t *)ctx;
    if (!state || !obj) return;
    if (dt < IVP_REAL_EPS) return;

    ivp_core_t *core = ivp_object_get_core(obj);
    if (!core || core->physical_unmoveable) return;

    ivp_real_t inv_dt = IVP_REAL(1.0) / dt;
    ivp_real_t pos_error = IVP_REAL(0.0);
    ivp_real_t rot_error = IVP_REAL(0.0);

    /* ── Position tracking ─────────────────────────────────────────────── */
    if (state->target_position_set) {
        ivp_vec3r_t cur_pos;
        ivp_core_get_position(core, &cur_pos);

        ivp_vec3r_t diff;
        ivp_vec3r_sub(&state->target_position, &cur_pos, &diff);
        pos_error = (ivp_real_t)sqrt((double)ivp_vec3r_quad_length(&diff));

        /* Compute desired velocity = gain * (target - current) / dt */
        ivp_vec3r_t desired_speed;
        ivp_real_t speed_scale = state->position_gain * inv_dt;
        ivp_vec3r_set(&desired_speed,
                      diff.k[0] * speed_scale,
                      diff.k[1] * speed_scale,
                      diff.k[2] * speed_scale);

        /* Clamp speed */
        ivp_real_t speed_sq = ivp_vec3r_quad_length(&desired_speed);
        ivp_real_t max_sq = state->max_linear_speed * state->max_linear_speed;
        if (speed_sq > max_sq && speed_sq > IVP_REAL_EPS) {
            ivp_real_t scale = state->max_linear_speed / (ivp_real_t)sqrt((double)speed_sq);
            ivp_vec3r_set(&desired_speed,
                          desired_speed.k[0] * scale,
                          desired_speed.k[1] * scale,
                          desired_speed.k[2] * scale);
        }

        /* Apply as impulse: delta_v = desired_speed - current_speed */
        ivp_vec3r_t cur_speed;
        ivp_core_get_speed(core, &cur_speed);

        ivp_vec3r_t impulse;
        ivp_vec3r_sub(&desired_speed, &cur_speed, &impulse);
        impulse.k[0] *= core->mass;
        impulse.k[1] *= core->mass;
        impulse.k[2] *= core->mass;

        ivp_core_center_push(core, &impulse);
    }

    /* ── Orientation tracking ──────────────────────────────────────────── */
    if (state->target_orientation_set) {
        ivp_quatr_t cur_orient;
        ivp_core_get_orientation(core, &cur_orient);

        rot_error = ivp_quat_angle_diff(&cur_orient, &state->target_orientation);

        if (rot_error > IVP_REAL_EPS) {
            /* Compute the rotation from current to target:
             * q_diff = q_target * conjugate(q_current) */
            ivp_quatr_t cur_conj = ivp_quat_conjugate(&cur_orient);
            ivp_quatr_t q_diff;
            ivp_quat_mult(&q_diff, &state->target_orientation, &cur_conj);

            /* Ensure shortest path */
            if (q_diff.w < IVP_REAL(0.0)) {
                q_diff.x = -q_diff.x;
                q_diff.y = -q_diff.y;
                q_diff.z = -q_diff.z;
                q_diff.w = -q_diff.w;
            }

            /* Extract axis-angle: axis = normalized(x,y,z), angle = 2*acos(w) */
            ivp_real_t sin_half = (ivp_real_t)sqrt((double)(
                q_diff.x * q_diff.x + q_diff.y * q_diff.y + q_diff.z * q_diff.z));

            if (sin_half > IVP_REAL_EPS) {
                ivp_real_t angle = IVP_REAL(2.0) * (ivp_real_t)acos(
                    (double)(q_diff.w > IVP_REAL(1.0) ? IVP_REAL(1.0) : q_diff.w));
                ivp_real_t inv_sin = IVP_REAL(1.0) / sin_half;

                /* Desired angular velocity = gain * angle * axis / dt */
                ivp_real_t ang_scale = state->rotation_gain * angle * inv_dt * inv_sin;

                ivp_vec3r_t desired_rot;
                ivp_vec3r_set(&desired_rot,
                              q_diff.x * ang_scale,
                              q_diff.y * ang_scale,
                              q_diff.z * ang_scale);

                /* Clamp angular speed */
                ivp_real_t rot_speed_sq = ivp_vec3r_quad_length(&desired_rot);
                ivp_real_t max_rot_sq = state->max_angular_speed * state->max_angular_speed;
                if (rot_speed_sq > max_rot_sq && rot_speed_sq > IVP_REAL_EPS) {
                    ivp_real_t scale = state->max_angular_speed /
                                       (ivp_real_t)sqrt((double)rot_speed_sq);
                    ivp_vec3r_set(&desired_rot,
                                  desired_rot.k[0] * scale,
                                  desired_rot.k[1] * scale,
                                  desired_rot.k[2] * scale);
                }

                /* Apply torque impulse (approximate) */
                ivp_vec3r_t torque_impulse;
                ivp_vec3r_set(&torque_impulse,
                              desired_rot.k[0] * core->rot_inertia.k[0],
                              desired_rot.k[1] * core->rot_inertia.k[1],
                              desired_rot.k[2] * core->rot_inertia.k[2]);

                ivp_core_rot_push_cs(core, &torque_impulse);
            }
        }
    }

    /* ── Problem callback ──────────────────────────────────────────────── */
    if (state->problem_fn) {
        if (pos_error > state->max_position_error ||
            rot_error > state->max_rotation_error) {
            state->problem_fn(state->problem_ctx, obj, pos_error, rot_error);
        }
    }
}
