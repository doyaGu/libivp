/* ivp_car_wheel_physics.c -- port of ivp_controller/ivp_car_wheel_physics.cxx
 *
 *   IVP_Calc_Next_PSI_Solver::calc_fast_wheel_rotation
 *   IVP_Contact_Point::friction_force_local_constraint_2d_wheel
 *
 * Both are reached only through core->car_wheel, which the car constraint
 * solver sets (ivp_constraint_car.c).  Reference types and evaluation order
 * (see ivp_constraint_car_internal.h).
 */
#include <math.h>

#include "controllers/ivp_car_wheel_physics.h"
#include "controllers/ivp_constraint_car_internal.h"
#include "solver/ivp_friction_internal.h"

/* IVP_Inline_Math::sind / sqrtd / fabsd (IVP_DOUBLE) */
static inline D cw_sind(D x) { return ivp_sin(x); }
static inline D cw_sqrtd(D x) { return ivp_sqrt(x); }
static inline D cw_fabsd(D x) { return ivp_fabs(x); }

void ivp_car_wheel_calc_fast_wheel_rotation(ivp_core_t *pc, float delta_sim_time, ivp_quatr_t *q_core_f_core)
{
    const ivp_constraint_car_object_t *wheel = ivp_car_object_of_core(pc);
    int axis = wheel->solver_car->x_idx;
    D rot_speed_wheel_axis = (D)pc->rot_speed.k[axis];
    ivp_vec3f_t p = pc->rot_speed;
    p.k[axis] = 0.0f;

    ivp_quatr_t gyro_rotation;
    ivp_quat_set_fast_multiple_with_clip(&gyro_rotation, &p, (ivp_real_t)delta_sim_time);

    ivp_quatr_t wheel_rotation;
    lvf_zero(&p);
    D sinus = cw_sind(rot_speed_wheel_axis * 0.5f * (D)delta_sim_time);
    p.k[axis] = (float)sinus;

    wheel_rotation.x = (ivp_real_t)p.k[0];
    wheel_rotation.y = (ivp_real_t)p.k[1];
    wheel_rotation.z = (ivp_real_t)p.k[2];
    wheel_rotation.w = (ivp_real_t)cw_sqrtd(1.0f - sinus * sinus);

    lvq_mult(q_core_f_core, &gyro_rotation, &wheel_rotation);
}

bool ivp_car_wheel_friction_force_local_constraint_2d_wheel(ivp_contact_pt_t *cp, ivp_core_t *core_a,
                                                            ivp_impact_solver_lt_t *info, const ivp_sim_es_t *es,
                                                            float *flEnergy)
{
    ivp_constraint_car_object_t *wheel = ivp_car_object_of_core(core_a);
    ivp_constraint_solver_car_t *solver = wheel->solver_car;
    ivp_core_t *body_core = solver->body_object->real_object->physical_core;

    D maximum_impulse_force = (D)(cp->now_friction_pressure * cp->real_friction_factor) * (D)es->delta_time;

    ivp_vec3f_t axis_bs;
    axis_bs.k[0] = (float)LVM_E(&wheel->target_position_bs, 0, solver->x_idx);
    axis_bs.k[1] = (float)LVM_E(&wheel->target_position_bs, 1, solver->x_idx);
    axis_bs.k[2] = (float)LVM_E(&wheel->target_position_bs, 2, solver->x_idx);
    ivp_vec3f_t axis_ws;
    const ivp_transform_r_t *m_world_f_core = &body_core->m_world_f_core_last_psi;
    lvm_vmult3_f(m_world_f_core, &axis_bs, &axis_ws);

    /* new span system for which one axis.dot(axis_ws) == 0 */
    ivp_vec3f_t span_v_0; /* orthogonal to any car body influences */
    ivp_vec3f_t span_v_1; /* influenced by car body weight */

    lvf_cross(&span_v_0, &axis_ws, &info->surf_normal);
    if (lvf_quad_length(&span_v_0) < (D)0.001f) return false;

    (void)lvf_normize(&span_v_0);
    lvf_cross(&span_v_1, &span_v_0, &info->surf_normal);

    D dot_old0_new0 = lvf_dot(&info->span_friction_v[0], &span_v_0);
    D dot_old0_new1 = lvf_dot(&info->span_friction_v[0], &span_v_1);
    D dot_old1_new0 = lvf_dot(&info->span_friction_v[1], &span_v_0);
    D dot_old1_new1 = lvf_dot(&info->span_friction_v[1], &span_v_1);

    D span_s_0 = (D)cp->span_friction_s[0] * dot_old0_new0 + (D)cp->span_friction_s[1] * dot_old1_new0;
    D span_s_1 = (D)cp->span_friction_s[0] * dot_old0_new1 + (D)cp->span_friction_s[1] * dot_old1_new1;

    /* pushing behaviour between wheel and car in wheel axis direction */
    D p_wheel, p_body, wheel_vel, body_vel;

    ivp_car_scr_t tcb;
    ivp_car_scr_init_translation_ws(&tcb, core_a, NULL, &info->contact_point_ws, &span_v_0, &span_v_1, NULL);
    p_wheel = tcb.m[1][1];
    wheel_vel = (D)tcb.delta_velocity_ds.k[1];
    {
        ivp_car_scr_t sc_body;
        ivp_car_scr_init_translation_ws(&sc_body, body_core, NULL, &info->contact_point_ws, &span_v_1, NULL, NULL);
        p_body = sc_body.m[0][0];
        body_vel = (D)sc_body.delta_velocity_ds.k[0];
    }

    D es_i_delta_time = (D)es->i_delta_time;
    D a = span_s_0 * es_i_delta_time - (D)tcb.delta_velocity_ds.k[0];
    D b = span_s_1 * es_i_delta_time - 1.0f * (wheel_vel * 1.0f + body_vel * 0.0f);

    D inv_mat2x2[4];
    const D tpm00 = tcb.m[0][0];
    const D tpm01 = tcb.m[0][1];
    const D tpm10 = tcb.m[1][0];
    const D tpm11 = tcb.m[1][1];
    /* reference (dimhotepus) argument order */
    if (!ivp_car_invert_2x2_matrix(tpm00, tpm01, tpm10, tpm11, &inv_mat2x2[0], &inv_mat2x2[1], &inv_mat2x2[2],
                                   &inv_mat2x2[3])) {
        *flEnergy = 0.0f;
        return false; /* fall back to the generic 2D friction solver */
    }

    ivp_vec3f_t impulses;
    impulses.k[0] = (float)(inv_mat2x2[0] * a + inv_mat2x2[1] * b);
    impulses.k[1] = (float)(inv_mat2x2[2] * a + inv_mat2x2[3] * b);
    impulses.k[2] = 0.0f;

    D project_span_v1 = cw_fabsd(lvf_dot(&span_v_1, &axis_ws));

    D relaxation_coefficient = 0.3f; /* ~1.0f tends to oscillate */
    if (cw_fabsd(p_body) < (D)P_FLOAT_RES) {
        *flEnergy = 0.0f;
        return false; /* avoid unstable wheel-body scaling */
    }
    D body_impulse_factor = (project_span_v1 * p_wheel / p_body + 1.0f) * relaxation_coefficient;

    D imp0_sqrd = (D)(impulses.k[0] * impulses.k[0]);
    D imp1_sqrd = (D)(impulses.k[1] * impulses.k[1]);

    /* check for sliding with body push */
    lvf_set_p(&wheel->last_contact_position_ws, &info->contact_point_ws);
    wheel->last_skid_time = ivp_env_get_current_time(es->environment);

    float body_inv_mass = ivp_core_get_inv_mass_f(body_core);
    D max_sq = maximum_impulse_force * maximum_impulse_force;

    D square_impulse;
    if (wheel->fix_wheel_constraint) {
        /* handbrake */
        square_impulse = (imp0_sqrd + imp1_sqrd) * body_impulse_factor * body_impulse_factor;
        if (square_impulse > max_sq) {
            /* sliding handbrake */
            D isum_impulse = (D)lv_isqrt_float((float)square_impulse); /* body push */
            D f = maximum_impulse_force * isum_impulse * body_impulse_factor;
            impulses.k[0] = (float)((D)impulses.k[0] * f);
            impulses.k[1] = (float)((D)impulses.k[1] * f);

            f /= body_impulse_factor;

            cp->span_friction_s[0] = (float)((D)cp->span_friction_s[0] * f); /* update reference point */
            cp->span_friction_s[1] = (float)((D)cp->span_friction_s[1] * f);

            wheel->last_skid_value = (float)((1.0f - f) * (D)cp->now_friction_pressure * (D)body_inv_mass);
        } else {
            /* still handbrake */
            D impulse_factor = .5f; /* reduce real values to avoid jitter effects */
            impulses.k[0] = (float)((D)impulses.k[0] * (body_impulse_factor * impulse_factor));
            impulses.k[1] = (float)((D)impulses.k[1] * (body_impulse_factor * impulse_factor));
            wheel->last_skid_value = 0.0f;
        }
    } else {
        square_impulse = imp0_sqrd + imp1_sqrd * body_impulse_factor * body_impulse_factor;
        if (square_impulse > max_sq) {
            /* check for sliding with no body push */
            D square2 = imp0_sqrd + imp1_sqrd;
            if (square2 > max_sq) {
                /* clip impulse on a circle */
                D isum_impulse = (D)lv_isqrt_float((float)square2); /* no body push */
                D f = maximum_impulse_force * isum_impulse;
                impulses.k[0] = (float)((D)impulses.k[0] * f);
                impulses.k[1] = (float)((D)impulses.k[1] * f);

                cp->span_friction_s[0] = (float)((D)cp->span_friction_s[0] * f); /* update reference point */
                cp->span_friction_s[1] = (float)((D)cp->span_friction_s[1] * f);
                wheel->last_skid_value = (float)((1.0f - f) * (D)cp->now_friction_pressure * (D)body_inv_mass);
            } else {
                /* reduce body push until no sliding (clip impulse on an ellipse) */
                D y = cw_sqrtd(max_sq - imp0_sqrd);
                D old1 = (D)impulses.k[1];
                if (impulses.k[1] > 0) {
                    impulses.k[1] = (float)y;
                } else {
                    impulses.k[1] = (float)-y;
                }

                /* update reference point */
                static const D extra_factor = .9f;
                D s_factor = extra_factor * (D)impulses.k[1] / (old1 * body_impulse_factor);
                wheel->last_skid_value =
                    (float)(0.3f * (1.0f - s_factor) * (D)cp->now_friction_pressure * (D)body_inv_mass);
                if (s_factor < 1.0f) {
                    span_s_1 *= s_factor;
                    D s0 = span_s_0 * dot_old0_new0 + span_s_1 * dot_old0_new1;
                    D s1 = span_s_0 * dot_old1_new0 + span_s_1 * dot_old1_new1;
                    cp->span_friction_s[0] = (float)s0;
                    cp->span_friction_s[1] = (float)s1;
                }
            }
            square_impulse = max_sq;
        }
        /* else: no sliding */
    }
    (void)square_impulse;

    ivp_car_scr_exert_impulse_dim2(&tcb, core_a, NULL, &impulses);

    *flEnergy = 0.0f;
    return true;
}
