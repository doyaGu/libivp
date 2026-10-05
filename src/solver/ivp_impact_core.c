/* ivp_impact_core.c — port of IVP_Impact_Solver / IVP_Impact_Solver_Long_Term /
 * IVP_Impact_System and IVP_Mindist::do_impact / try_to_generate_managed_friction
 *
 * Reference: ivp_intern/ivp_impact.{hxx,cxx}, ivp_physics/ivp_mindist_impact.cxx,
 * ivp_physics/ivp_mindist.cxx (create_cp_in_advance_pretension),
 * ivp_physics/ivp_mindist_minimize.cxx (mindist_rescue_push),
 * ivp_physics/ivp_anomaly_manager.cxx (inter_penetration).
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>

#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <ivp/diagnostics/ivp_debug_manager.h>
#include <ivp/diagnostics/ivp_statistic_manager.h>
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include "collision/ivp_lvec.h"
#include "solver/ivp_friction_internal.h"
#include "dynamics/ivp_env_hot.h"

#define IVP_SAFETY_FACTOR_FOR_DELAY      1.2f
#define IVP_INV_SAFETY_FACTOR_FOR_DELAY  (1.0f / IVP_SAFETY_FACTOR_FOR_DELAY)
#define IVP_ROT_MAX_UNCERTAINTY          0.5f
#define IVP_COLL_DETECT_MIN_INV_TIME     200.0f
#define IVP_COLL_DETECT_MIN_TIME         (1.0f / IVP_COLL_DETECT_MIN_INV_TIME)
#define IVP_SPEED_ADDON_SYSTEM_IMPACT    0.01f
#define IVP_INV_HALF_CONSERVATION_STEPS  0.5f
#define MAXIMUM_SYSTEM_PUSH_NUMBER       5000
#define IMPACT_EPS                       10E-5f

bool ivp_mindist_recursive_do_impact(ivp_mindist_t *md);   /* ivp_mindist_core.c */

/* env->debug_information / get_debug_manager() and get_statistic_manager() */
static inline ivp_debug_manager_t *imp_dm(const ivp_core_t *c) { return ivp_env_get_debug_manager(ivp_core_env(c)); }
static inline ivp_statistic_manager_t *imp_sm(const ivp_core_t *c) { return ivp_env_get_statistic_manager(ivp_core_env(c)); }

/* IVP_Impact_Solver::get_total_energy (with its IVP_IF(debug_impact) "total_energies" print) is not
 * ported: the reference never calls it (only from commented-out code). */

/* ── IVP_Impact_Solver ─────────────────────────────────────────────────── */

typedef struct impact_solver {
    float        rescue_speed_impact_solver;
    double       virt_mass[2];
    bool         delaying_is_allowed;
    double       energy_deformation;
    const ivp_transform_r_t *m_world_f_core[2];
    ivp_vec3f_t  rot_speed[2];
    ivp_vec3f_t  trans_speed[2];
    ivp_vec3f_t  rot_speed_change[2];
    ivp_vec3f_t  trans_speed_change[2];
    ivp_vec3f_t  relative_world_speed;
    ivp_vec3f_t  world_push_direction;
    ivp_vec3f_t  integral_pushes_world;
    /* public */
    bool         two_friction_values;
    float        sin_second_friction;
    ivp_vec3f_t  world_direction_second_friction;
    ivp_core_t  *core[2];
    ivp_vec3f_t *obj_point[2];
    float        percent_energy_conservation;
    float        cos_friction;
    float        sin_friction;
    ivp_vec3f_t *surf_normal;
    ivp_vec3f_t *speed;
} impact_solver_t;

static void is_get_relative_speed_vector(impact_solver_t *s)
{
    ivp_vec3f_t world_speed0, world_speed1;
    ivp_core_get_surface_speed_on_test(s->core[0], s->obj_point[0], &s->trans_speed[0], &s->rot_speed[0], &world_speed0);
    ivp_core_get_surface_speed_on_test(s->core[1], s->obj_point[1], &s->trans_speed[1], &s->rot_speed[1], &world_speed1);
    lvf_subtract2(&s->relative_world_speed, &world_speed1, &world_speed0);
}

static void is_get_world_push_direction_two_friction(impact_solver_t *s, double part_direction_surf_normal)
{
    ivp_vec3f_t part_in_direction_surf;
    double mangle = -part_direction_surf_normal;
    lvf_add_multiple2(&part_in_direction_surf, &s->world_push_direction, s->surf_normal, (D)mangle);
    float length_direction_surf = (float)lvf_real_length_plus_normize(&part_in_direction_surf);
    double part_second_friction = (double)lvf_dot(&part_in_direction_surf, &s->world_direction_second_friction);
    float abs_second_part = (float)fabs(part_second_friction);
    float angle_2d = (float)asin((double)abs_second_part);
    float axis_first_friction = lv_approx5_cos(angle_2d);
    float axis_second_friction = abs_second_part;
    float quad_len_allowed = axis_first_friction * axis_first_friction * s->sin_friction * s->sin_friction +
                             axis_second_friction * axis_second_friction * s->sin_second_friction * s->sin_second_friction;
    if (length_direction_surf * length_direction_surf > quad_len_allowed) {
        float allowed_len = (float)sqrt((double)quad_len_allowed);
        float length_angle = (float)asin((double)allowed_len);
        float cos_part = lv_approx5_cos(length_angle);
        ivp_vec3f_t world_push_dir_fl;
        lvf_set_multiple(&world_push_dir_fl, s->surf_normal, (D)-cos_part);
        s->world_push_direction = world_push_dir_fl;
        lvf_add_multiple(&s->world_push_direction, &part_in_direction_surf, (D)allowed_len);
    }
}

static void is_get_world_push_direction(impact_solver_t *s)
{
    ivp_vec3f_t part_in_direction_surf;
    s->world_push_direction = s->relative_world_speed;
    (void)lvf_fast_normize(&s->world_push_direction);
    double angle = (double)lvf_dot(&s->world_push_direction, s->surf_normal);
    IVP_IF(imp_dm(s->core[0])->debug_impact) {
        ivp_debug_printf("wpd %.2f %.2f %.2f  ", (double)s->world_push_direction.k[0],
                         (double)s->world_push_direction.k[1], (double)s->world_push_direction.k[2]);
    }
    if (angle > 0.0) {
        IVP_IF(imp_dm(s->core[0])->debug_impact) {
            ivp_debug_printf(" impact_speed_now_reversed ");
        }
        s->world_push_direction = s->integral_pushes_world;
        (void)lvf_fast_normize(&s->world_push_direction);
        return;
    } else {
        IVP_IF(imp_dm(s->core[0])->debug_impact) {
            ivp_debug_printf(" not_reversed ");
        }
    }
    if (!s->two_friction_values) {
        if (angle > (double)-s->cos_friction) {
            double mangle = -angle;
            lvf_add_multiple2(&part_in_direction_surf, &s->world_push_direction, s->surf_normal, (D)mangle);
            (void)lvf_normize(&part_in_direction_surf);
            lvf_mult(&part_in_direction_surf, (D)s->sin_friction);
            lvf_add_multiple2(&s->world_push_direction, &part_in_direction_surf, s->surf_normal, (D)-s->cos_friction);
        }
    } else {
        is_get_world_push_direction_two_friction(s, angle);
    }
}

static void is_confirm_impact(impact_solver_t *s, int core_nr)
{
    ivp_core_t *c = s->core[core_nr];
    ivp_core_set_speed_f(c, &s->trans_speed[core_nr]);
    c->rot_speed = s->rot_speed[core_nr];
    struct ivp_environment *env = ivp_core_env(c);
    if (c->impacts_since_last_PSI > ivp_env_anomaly_get_max_collisions_per_psi(env)) {
        c->temporarily_unmovable = ivp_env_anomaly_max_collisions_exceeded_check_freezing(env, c);
    }
}

static void is_undo_push(impact_solver_t *s)
{
    IVP_IF(imp_dm(s->core[0])->debug_impact) {
        ivp_debug_printf("impact_undo_push\n");
    }
    if (!s->core[0]->physical_unmoveable) {
        lvf_subtract(&s->rot_speed[0], &s->rot_speed_change[0]);
        lvf_subtract(&s->trans_speed[0], &s->trans_speed_change[0]);
    }
    if (!s->core[1]->physical_unmoveable) {
        lvf_subtract(&s->rot_speed[1], &s->rot_speed_change[1]);
        lvf_subtract(&s->trans_speed[1], &s->trans_speed_change[1]);
        IVP_IF(imp_dm(s->core[0])->debug_impact) {
            ivp_debug_printf("undoing trans %.3f %.3f %.3f\n", (double)s->trans_speed_change[1].k[0],
                             (double)s->trans_speed_change[1].k[1], (double)s->trans_speed_change[1].k[2]);
        }
    }
}

static void is_do_rescue_push(impact_solver_t *s, const ivp_vec3f_t *push_dir_norm, bool panic_mode)
{
    ivp_vec3f_t diff_vec_world;
    lvf_set_multiple(&diff_vec_world, push_dir_norm, -1.0f);
    ivp_vec3f_t world_speed0, world_speed1;
    ivp_core_get_surface_speed_on_test(s->core[0], s->obj_point[0], &s->trans_speed[0], &s->rot_speed[0], &world_speed0);
    ivp_core_get_surface_speed_on_test(s->core[1], s->obj_point[1], &s->trans_speed[1], &s->rot_speed[1], &world_speed1);
    ivp_vec3f_t rel_speed_world;
    lvf_subtract2(&rel_speed_world, &world_speed1, &world_speed0);
    double dist_velocity = (double)lvf_dot(&rel_speed_world, &diff_vec_world);
    if (panic_mode) {
        if (dist_velocity > 0.0) dist_velocity = 0.0;
    }
    double desired_velo_change = (double)s->rescue_speed_impact_solver - dist_velocity;
    if (desired_velo_change < 0.0) return;
    /* ivp_impact.cxx:281 "#if 0" translation-only rescue push (with its IVP_IF(1) rescue counter block):
     * off in the reference debug build too, not ported. */

    ivp_vec3f_t push_vec_obj, push_vec_world, rotation_vec, translation_vec, speed_change_vec;
    double speed_change_scalar = 0.0;
    push_vec_world = diff_vec_world;
    if (!s->core[1]->physical_unmoveable) {
        lvm_vimult3_f(s->m_world_f_core[1], &push_vec_world, &push_vec_obj);
        ivp_core_test_push_core(s->core[1], s->obj_point[1], &push_vec_obj, &push_vec_world, &translation_vec, &rotation_vec);
        ivp_core_get_surface_speed_on_test(s->core[1], s->obj_point[1], &translation_vec, &rotation_vec, &speed_change_vec);
        speed_change_scalar += (double)lvf_dot(&push_vec_world, &speed_change_vec);
    }
    if (!s->core[0]->physical_unmoveable) {
        lvf_mult(&push_vec_world, -1.0f);
        lvm_vimult3_f(s->m_world_f_core[0], &push_vec_world, &push_vec_obj);
        ivp_core_test_push_core(s->core[0], s->obj_point[0], &push_vec_obj, &push_vec_world, &translation_vec, &rotation_vec);
        ivp_core_get_surface_speed_on_test(s->core[0], s->obj_point[0], &translation_vec, &rotation_vec, &speed_change_vec);
        speed_change_scalar += (double)lvf_dot(&push_vec_world, &speed_change_vec);
    }
    double resulting_push = desired_velo_change / (speed_change_scalar + (double)1e-15f);
    if (resulting_push < 0.0) return;
    IVP_IF(1) {
        if (panic_mode) {
            imp_sm(s->core[0])->impact_hard_rescue_counter++;
        } else {
            imp_sm(s->core[0])->impact_rescue_after_counter++;
        }
    }
    IVP_IF(imp_dm(s->core[0])->debug_impact || 0) {
        ivp_debug_printf("doing_res_push %f dvc %f\n", resulting_push, desired_velo_change);
    }
    ivp_vec3f_t obj_push, world_push;
    if (!s->core[0]->physical_unmoveable) {
        lvf_set_multiple(&world_push, &diff_vec_world, (D)-resulting_push);
        lvm_vimult3_f(s->m_world_f_core[0], &world_push, &obj_push);
        ivp_core_test_push_core(s->core[0], s->obj_point[0], &obj_push, &world_push, &s->trans_speed_change[0],
                                &s->rot_speed_change[0]);
        lvf_add(&s->rot_speed[0], &s->rot_speed_change[0]);
        lvf_add(&s->trans_speed[0], &s->trans_speed_change[0]);
    }
    if (!s->core[1]->physical_unmoveable) {
        lvf_set_multiple(&world_push, &diff_vec_world, (D)resulting_push);
        lvm_vimult3_f(s->m_world_f_core[1], &world_push, &obj_push);
        ivp_core_test_push_core(s->core[1], s->obj_point[1], &obj_push, &world_push, &s->trans_speed_change[1],
                                &s->rot_speed_change[1]);
        lvf_add(&s->rot_speed[1], &s->rot_speed_change[1]);
        lvf_add(&s->trans_speed[1], &s->trans_speed_change[1]);
    }
}

static void is_do_push(impact_solver_t *s, double impulse_val)
{
    ivp_vec3f_t obj_push, world_push;
    world_push = s->world_push_direction;
    IVP_IF(imp_dm(s->core[0])->debug_impact) {
        ivp_vec3f_t surf_dir;
        lvf_set3(&surf_dir, 0.0f, 1.0f, 0.0f);
        double surf_cos = (double)lvf_dot(&surf_dir, &s->world_push_direction);
        ivp_debug_printf("push_dir_cos %f\n", surf_cos);
    }
    lvf_mult(&world_push, (D)impulse_val);
    if (!s->core[0]->physical_unmoveable) {
        IVP_IF(imp_dm(s->core[0])->debug_impact) {
            /* printf("p %.2f %.2f %.2f ",world_push_direction...) (commented out in the reference) */
        }
        lvm_vimult3_f(s->m_world_f_core[0], &world_push, &obj_push);
        ivp_core_test_push_core(s->core[0], s->obj_point[0], &obj_push, &world_push, &s->trans_speed_change[0],
                                &s->rot_speed_change[0]);
        lvf_add(&s->rot_speed[0], &s->rot_speed_change[0]);
        lvf_add(&s->trans_speed[0], &s->trans_speed_change[0]);
    }
    if (!s->core[1]->physical_unmoveable) {
        lvf_mult(&world_push, -1.0f);
        IVP_IF(imp_dm(s->core[0])->debug_impact) {
            /* printf("p %.2f %.2f %.2f ",world_push_direction...) (commented out in the reference) */
        }
        lvm_vimult3_f(s->m_world_f_core[1], &world_push, &obj_push);
        ivp_core_test_push_core(s->core[1], s->obj_point[1], &obj_push, &world_push, &s->trans_speed_change[1],
                                &s->rot_speed_change[1]);
        IVP_IF(imp_dm(s->core[0])->debug_impact) {
            /* printf("did_push trans ...") (commented out in the reference) */
        }
        lvf_add(&s->rot_speed[1], &s->rot_speed_change[1]);
        lvf_add(&s->trans_speed[1], &s->trans_speed_change[1]);
    }
}

static void is_calc_virt_masses_impact_solver(impact_solver_t *s, const ivp_vec3f_t *world_direction_normal)
{
    ivp_vec3f_t direction_world = *world_direction_normal;
    for (int i = 1; i >= 0; i--) {
        ivp_vec3f_t direction_obj;
        lvm_vimult3_f(s->m_world_f_core[i], &direction_world, &direction_obj);
        s->virt_mass[i] = ivp_core_calc_correct_virt_mass(s->core[i], s->obj_point[i], &direction_obj, &direction_world);
        lvf_mult(&direction_world, -1.0f);
    }
    if (s->core[0]->physical_unmoveable) s->virt_mass[0] = s->virt_mass[1] * 100000.0f;
    if (s->core[1]->physical_unmoveable) s->virt_mass[1] = s->virt_mass[0] * 100000.0f;
}

static double is_estimate_push_impulse(impact_solver_t *s)
{
    is_calc_virt_masses_impact_solver(s, s->surf_normal);
    double push_v0 = 0.0;
    double push_v1 = -(double)lvf_dot(&s->relative_world_speed, s->surf_normal);
    double div_masses = 1.0f / (s->virt_mass[0] + s->virt_mass[1]);
    double u0 = (2.0f * s->virt_mass[1] * push_v1 + push_v0 * (s->virt_mass[0] - s->virt_mass[1]));
    u0 *= div_masses;
    double diff_v0 = u0 - push_v0;
    IVP_IF(imp_dm(s->core[0])->debug_impact) {
        ivp_debug_printf("start_impulse %.2f\n", diff_v0 * s->virt_mass[0]);
    }
    return diff_v0 * s->virt_mass[0];
}

static void is_clear_change_values_cores(impact_solver_t *s)
{
    for (int i = 0; i < 2; i++) {
        ivp_vec3r_set_zero(&s->core[i]->rot_speed_change);
        ivp_vec3r_set_zero(&s->core[i]->speed_change);
    }
}

static void is_delay_of_impact(impact_solver_t *s, int delayed_core_nr)
{
    ivp_core_t *c = s->core[delayed_core_nr];
    ivp_vec3f_t sp = ivp_core_speed_f(c);
    ivp_vec3f_t d;
    lvf_subtract2(&d, &s->trans_speed[delayed_core_nr], &sp);
    lvp_set_f(&c->speed_change, &d);
    lvf_subtract2(&d, &s->rot_speed[delayed_core_nr], &c->rot_speed);
    lvp_set_f(&c->rot_speed_change, &d);
}

static void is_delay_decision(impact_solver_t *s, ivp_core_t *pushed_cores[2])
{
    s->core[0]->impacts_since_last_PSI = (short)(s->core[0]->impacts_since_last_PSI + (1 - (int)s->core[0]->physical_unmoveable));
    s->core[1]->impacts_since_last_PSI = (short)(s->core[1]->impacts_since_last_PSI + (1 - (int)s->core[1]->physical_unmoveable));
    if (s->delaying_is_allowed) {
        int try_to_delay_core, other_core;
        if (s->virt_mass[0] > s->virt_mass[1]) {
            try_to_delay_core = 0;
            other_core = 1;
        } else {
            try_to_delay_core = 1;
            other_core = 0;
        }
        if (pushed_cores[try_to_delay_core]->physical_unmoveable) goto no_delaying;
        ivp_vec3f_t delayer_trans_speed, delayer_rot_speed, other_trans_speed, other_rot_speed;
        other_rot_speed = s->rot_speed[other_core];
        other_trans_speed = s->trans_speed[other_core];
        delayer_rot_speed = s->core[try_to_delay_core]->rot_speed;
        delayer_trans_speed = ivp_core_speed_f(s->core[try_to_delay_core]);
        double closing_speed;
        {
            ivp_vec3f_t rel_world_speed;
            ivp_core_get_diff_surface_speed_of_two_cores_on_test(
                s->core[try_to_delay_core], s->core[other_core], s->obj_point[try_to_delay_core],
                s->obj_point[other_core], &delayer_trans_speed, &delayer_rot_speed, &other_trans_speed,
                &other_rot_speed, &rel_world_speed);
            closing_speed = (double)lvf_dot(&rel_world_speed, s->surf_normal);
            if (try_to_delay_core) closing_speed *= -1.0;
        }
        if (closing_speed < (double)(-s->rescue_speed_impact_solver * IVP_INV_SAFETY_FACTOR_FOR_DELAY)) {
            imp_sm(s->core[0])->impact_delayed_counter++;
            pushed_cores[try_to_delay_core] = NULL;
            is_clear_change_values_cores(s);
            is_confirm_impact(s, 1 - try_to_delay_core);
            is_delay_of_impact(s, try_to_delay_core);
            s->core[try_to_delay_core]->impacts_since_last_PSI--;
        } else {
            goto no_delaying;
        }
        return;
    }
no_delaying:
    is_clear_change_values_cores(s);
    is_confirm_impact(s, 0);
    is_confirm_impact(s, 1);
}

static void is_do_impact(impact_solver_t *s, ivp_core_t *pushed_cores[2], bool allow_delaying, int pushes_while_system,
                         float rescue_speed_addon)
{
    double virtual_speed, give_back_speed = 0.0, speed_before = 0.0;
    double relative_trans_speed_before = 0.0;
    double used_conservation;
    IVP_IF(1) {
        imp_dm(s->core[0])->all_time_impacts += (ivp_real_t)1.0f;
    }
    used_conservation = (double)(1.0f - (1.0f / ((float)pushes_while_system * IVP_INV_HALF_CONSERVATION_STEPS + 1.0f)) *
                                            (1.0f - s->percent_energy_conservation));
    s->rescue_speed_impact_solver =
        (float)(ivp_mindist_settings.min_vertical_speed_at_collision + rescue_speed_addon) * IVP_SAFETY_FACTOR_FOR_DELAY;
    s->delaying_is_allowed = allow_delaying;
    pushed_cores[0] = s->core[0];
    pushed_cores[1] = s->core[1];
    imp_sm(s->core[0])->impact_counter++;
    s->m_world_f_core[0] = &s->core[0]->m_world_f_core_last_psi;
    s->m_world_f_core[1] = &s->core[1]->m_world_f_core_last_psi;
    IVP_IF(0) {
        s->rot_speed[0] = s->core[0]->rot_speed;
        s->trans_speed[0] = ivp_core_speed_f(s->core[0]);
        s->rot_speed[1] = s->core[1]->rot_speed;
        s->trans_speed[1] = ivp_core_speed_f(s->core[1]);
        is_get_relative_speed_vector(s);
        double speed_parallel_mindist_dbg = (double)lvf_dot(&s->relative_world_speed, s->surf_normal);
        ivp_debug_printf("impact %.20G\n", speed_parallel_mindist_dbg);
    }
    for (int i = 0; i < 2; i++) {
        ivp_core_t *c = s->core[i];
        s->rot_speed[i].k[0] = (float)((ivp_real_t)c->rot_speed.k[0] + c->rot_speed_change.k[0]);
        s->rot_speed[i].k[1] = (float)((ivp_real_t)c->rot_speed.k[1] + c->rot_speed_change.k[1]);
        s->rot_speed[i].k[2] = (float)((ivp_real_t)c->rot_speed.k[2] + c->rot_speed_change.k[2]);
        s->trans_speed[i].k[0] = (float)(c->speed.k[0] + c->speed_change.k[0]);
        s->trans_speed[i].k[1] = (float)(c->speed.k[1] + c->speed_change.k[1]);
        s->trans_speed[i].k[2] = (float)(c->speed.k[2] + c->speed_change.k[2]);
    }
    IVP_IF(1) {
        ivp_vec3f_t trans_vec;
        lvf_subtract2(&trans_vec, &s->trans_speed[0], &s->trans_speed[1]);
        relative_trans_speed_before = (double)lvf_real_length(&trans_vec);
    }
    IVP_USE(relative_trans_speed_before);
    s->energy_deformation = 0.0;
    is_get_relative_speed_vector(s);
    *s->speed = s->relative_world_speed;
    int push_counter = 0;
    double impulse = is_estimate_push_impulse(s) * 0.1f;
    double speed_parallel_mindist = (double)lvf_dot(&s->relative_world_speed, s->surf_normal);
    if (speed_parallel_mindist > (double)-IMPACT_EPS) {
        ivp_vec3f_t push_dir;
        lvf_set_negative(&push_dir, s->surf_normal);
        is_do_rescue_push(s, &push_dir, true);
        if (pushes_while_system > 10) s->delaying_is_allowed = false;
        goto delaying_test;
    }
    IVP_IF(imp_dm(s->core[0])->debug_impact) {
        ivp_debug_printf("speed %.4f %.4f %.4f rot %.4f %.4f %.4f\n", (double)s->trans_speed[1].k[0],
                         (double)s->trans_speed[1].k[1], (double)s->trans_speed[1].k[2], (double)s->rot_speed[1].k[0],
                         (double)s->rot_speed[1].k[1], (double)s->rot_speed[1].k[2]);
    }
    is_get_world_push_direction(s);
    lvf_zero(&s->integral_pushes_world);
    virtual_speed = -(double)lvf_dot(s->surf_normal, &s->relative_world_speed);
    speed_before = virtual_speed;
    IVP_ASSERT(virtual_speed > 0.0);
    give_back_speed = virtual_speed * sqrt(used_conservation);
    give_back_speed += (double)IVP_SPEED_ADDON_SYSTEM_IMPACT * sqrt((double)pushes_while_system);
    while ((virtual_speed > 0.0) && (push_counter < 100)) {
        push_counter++;
        is_do_push(s, impulse);
        is_get_relative_speed_vector(s);
        virtual_speed = -(double)lvf_dot(s->surf_normal, &s->relative_world_speed);
        is_get_world_push_direction(s);
    }
    give_back_speed += virtual_speed;
    lvf_set_multiple(&s->world_push_direction, s->surf_normal, -1.0f);
    if ((give_back_speed > 0.0) && (push_counter != 100)) {
        is_do_push(s, 1);
        is_get_relative_speed_vector(s);
        double speed_test = (double)lvf_dot(s->surf_normal, &s->relative_world_speed);
        IVP_ASSERT((speed_test > -virtual_speed));
        double speed_diff = speed_test + virtual_speed;
        double target_push;
        if (fabs(speed_diff) > (double)IMPACT_EPS) {
            target_push = give_back_speed / speed_diff;
        } else {
            target_push = 0.0;
        }
        is_undo_push(s);
        is_do_push(s, target_push);
    }
    is_do_rescue_push(s, &s->world_push_direction, false);
    IVP_IF(1 == 1) {
        ivp_vec3f_t rel_speed_now;
        ivp_core_get_diff_surface_speed_of_two_cores_on_test(s->core[0], s->core[1], s->obj_point[0], s->obj_point[1],
                                                             &s->trans_speed[0], &s->rot_speed[0], &s->trans_speed[1],
                                                             &s->rot_speed[1], &rel_speed_now);
        if ((double)lvf_dot(&rel_speed_now, s->surf_normal) > 0.0) {
            IVP_IF(1) { ivp_debug_printf("impact_objects_have_illegal_speed_afterwards\n"); }
        }
    }
delaying_test:
    IVP_IF(0) {
        is_get_relative_speed_vector(s);
        double testtest = (double)lvf_dot(s->surf_normal, &s->relative_world_speed);
        IVP_USE(testtest);
        double speed_gain = testtest - speed_before; /* speed before not initialized when hard rescue */
        if (speed_gain > 0.0) {
            if (speed_gain > (double)imp_sm(s->core[0])->max_speed_gain) {
                imp_sm(s->core[0])->max_speed_gain = (float)speed_gain;
            }
        }
    }
    ivp_core_clip_velocity(s->core[0], &s->trans_speed[0], &s->rot_speed[0]);
    ivp_core_clip_velocity(s->core[1], &s->trans_speed[1], &s->rot_speed[1]);
    is_delay_decision(s, pushed_cores);
    if (s->core[0]->temporarily_unmovable | s->core[1]->temporarily_unmovable) {
        IVP_IF(0) {
            ivp_debug_printf("switched_to_temporarily_unmov %p %p %f\n", (void *)s->core[0], (void *)s->core[1],
                             ivp_env_get_current_time(ivp_core_env(s->core[0])));
        }
        imp_sm(s->core[0])->impact_unmov++;
        for (int i = 0; i < 2; i++) {
            ivp_core_t *c = s->core[i];
            c->temporarily_unmovable = (bool)(1 - (int)c->physical_unmoveable);
            /* IVP_U_Float_Point::add: float results */
            for (int k = 0; k < 3; k++) {
                c->speed_change.k[k] = (ivp_real_t)((float)c->speed_change.k[k] + (float)c->speed.k[k]);
                c->rot_speed_change.k[k] = (ivp_real_t)((float)c->rot_speed_change.k[k] + c->rot_speed.k[k]);
            }
            ivp_vec3r_set_zero(&c->speed);
            lvf_zero(&c->rot_speed);
            pushed_cores[i] = c;
        }
    }
}

/* IVP_Impact_Solver::get_cos_sin_for_impact */
static void is_get_cos_sin_for_impact(float friction_val, float percent_energy_conservation, float *cos_val, float *sin_val)
{
    double impact_fri_fact = (double)friction_val * (1.0f + sqrt((double)percent_energy_conservation));
    double fri_angle = atan(impact_fri_fact);
    double c = (double)lv_approx5_cos((float)fri_angle);
    *cos_val = (float)c;
    *sin_val = (float)(c * impact_fri_fact);
}

static void is_get_world_direction_second_friction(impact_solver_t *s, ivp_contact_pt_t *cp)
{
    const ivp_material_t *mtl[2];
    ivp_contact_pt_get_material_info(cp, mtl);
    ivp_vec3f_t *ret_second_friction_vec = &s->world_direction_second_friction;
    bool use_second_friction = false;
    for (int i = 0; i < 2; i++) {
        if (!mtl[i]->second_friction_x_enabled) continue;
        ivp_vec3f_t x_direction;
        ivp_core_t *i_core = cp->synapse[i].l_obj->friction_core;
        const ivp_transform_r_t *mat = &i_core->m_world_f_core_last_psi;
        x_direction.k[0] = (float)mat->rot.rows[0].k[0];
        x_direction.k[1] = (float)mat->rot.rows[1].k[0];
        x_direction.k[2] = (float)mat->rot.rows[2].k[0];
        ivp_vec3f_t *surf_normal = &cp->tmp_contact_info->surf_normal;
        ivp_vec3f_t x_in_surface;
        lvf_set_orthogonal_part(&x_in_surface, &x_direction, surf_normal);
        double relevance_factor = (double)lvf_real_length(&x_in_surface);
        double second_friction_val = (double)mtl[i]->second_friction_factor;
        double other_factor = (double)mtl[1 - i]->friction;
        double using_friction = other_factor * second_friction_val;
        double difference_in_friction = (double)cp->real_friction_factor - using_friction;
        float effective_friction_second = (float)((double)cp->real_friction_factor - difference_in_friction * relevance_factor);
        if (relevance_factor < P_DOUBLE_EPS) continue;
        use_second_friction = true;
        (void)lvf_normize(&x_in_surface);
        *ret_second_friction_vec = x_in_surface;
        float dummy_cos;
        is_get_cos_sin_for_impact(effective_friction_second, s->percent_energy_conservation, &dummy_cos,
                                  &s->sin_second_friction);
    }
    s->two_friction_values = use_second_friction;
}

/* ── IVP_Impact_Solver_Long_Term::do_impact_long_term ──────────────────── */

void ivp_impact_lt_do_impact_long_term(ivp_impact_solver_lt_t *info, ivp_core_t *pushed_cores[2], float rescue_speed_val,
                                       ivp_contact_pt_t *cp)
{
    impact_solver_t s;
    memset(&s, 0, sizeof(s));
    ivp_vec3f_t surf_normal_stack;
    if (!info->contact_core[1]) {
        s.core[0] = info->objects[1]->physical_core;
        s.core[1] = info->contact_core[0];
        s.obj_point[0] = &info->contact_point_cs[1];
        s.obj_point[1] = &info->contact_point_cs[0];
        lvf_set_negative(&surf_normal_stack, &info->surf_normal);
        s.surf_normal = &surf_normal_stack;
    } else {
        s.core[0] = info->contact_core[0];
        if (!info->contact_core[0]) s.core[0] = info->objects[0]->physical_core;
        s.core[1] = info->contact_core[1];
        s.obj_point[0] = &info->contact_point_cs[0];
        s.obj_point[1] = &info->contact_point_cs[1];
        s.surf_normal = &info->surf_normal;
    }
    s.speed = &info->speed;
    s.percent_energy_conservation = info->impact.percent_energy_conservation;
    s.two_friction_values = false;
    if (s.core[0]->car_wheel || s.core[1]->car_wheel) {
        is_get_cos_sin_for_impact(0.0f, info->impact.percent_energy_conservation, &s.cos_friction, &s.sin_friction);
    } else {
        is_get_cos_sin_for_impact(cp->real_friction_factor, info->impact.percent_energy_conservation, &s.cos_friction,
                                  &s.sin_friction);
        if (cp->two_friction_values) is_get_world_direction_second_friction(&s, cp);
    }
    IVP_IF(1) {
        if (info->contact_core[0]) ivp_core_core_plausible_check(info->contact_core[0]);
        if (info->contact_core[1]) ivp_core_core_plausible_check(info->contact_core[1]);
    }
    is_do_impact(&s, pushed_cores, true, info->impacts_while_system, rescue_speed_val);
    IVP_IF(1) {
        if (info->contact_core[0]) ivp_core_core_plausible_check(info->contact_core[0]);
        if (info->contact_core[1]) ivp_core_core_plausible_check(info->contact_core[1]);
    }
}

/* ── IVP_Contact_Point rescue / coll distance (ivp_impact.cxx) ────────── */

static float cp_get_rot_speed_uncertainty(ivp_contact_pt_t *cp)
{
    float rot_uncertainty = 0.0f;
    for (int i = 0; i < 2; i++) {
        ivp_core_t *my_core = cp->tmp_contact_info->contact_core[i];
        if (!my_core) continue;
        if (cp->synapse[i].status == IVP_ST_BALL) continue;
        double tan_quad = (double)lvf_quad_length(&my_core->rot_speed) * (double)IVP_COLL_DETECT_MIN_TIME *
                          (double)IVP_COLL_DETECT_MIN_TIME;
        double max_tan_quad = (double)(IVP_ROT_MAX_UNCERTAINTY * IVP_ROT_MAX_UNCERTAINTY);
        if (tan_quad > max_tan_quad) tan_quad = max_tan_quad;
        double alpha = sqrt(tan_quad);
        double uncertain_way = (1.0f - cos(alpha)) * (double)my_core->upper_limit_radius;
        double uncertain_speed = uncertain_way * (double)ivp_env_get_inv_delta_psi_time(ivp_core_env(my_core));
        rot_uncertainty = (float)((double)rot_uncertainty + uncertain_speed);
    }
    return rot_uncertainty;
}

float ivp_contact_pt_get_rescue_speed_impact(ivp_contact_pt_t *cp, struct ivp_environment *env)
{
    double resc_val = 0.0;
    double penetration_len = (double)ivp_cp_get_gap_length(cp);
    if (penetration_len < (double)ivp_mindist_settings.min_coll_dists) {
        resc_val = ((double)ivp_mindist_settings.min_coll_dists - penetration_len) * (double)ivp_env_get_inv_delta_psi_time(env);
    } else {
        cp->tmp_contact_info->impact.rescue_speed_addon = 0.0f;
    }
    double rot_uncertainty = (double)cp_get_rot_speed_uncertainty(cp);
    resc_val += rot_uncertainty;
    IVP_ASSERT(resc_val < 1000.0);
    IVP_IF(resc_val > (double)ivp_env_get_statistic_manager(env)->max_rescue_speed) {
        ivp_env_get_statistic_manager(env)->max_rescue_speed = (float)resc_val;
    }
    return (float)(2.0f * resc_val);
}

void ivp_contact_pt_calc_coll_distance(ivp_contact_pt_t *cp)
{
    ivp_impact_solver_lt_t *info = cp->tmp_contact_info;
    info->coll_time_is_valid = true;
    if (ivp_cp_get_gap_length(cp) > ivp_mindist_settings.max_distance_for_impact_system) {
        info->impact.distance_reached_in_time = 1E20f;
        return;
    }
    struct ivp_environment *env = cp->synapse[0].l_obj->owner;
    info->impact.rescue_speed_addon = ivp_contact_pt_get_rescue_speed_impact(cp, env);
    double closing_speed = ivp_impact_lt_get_closing_speed(info);
    info->impact.distance_reached_in_time =
        (float)((double)ivp_cp_get_gap_length(cp) -
                (closing_speed + (double)info->impact.rescue_speed_addon * 0.5f) * (double)ivp_env_get_delta_psi_time(env));
    IVP_IF(1) {
        ivp_debug_manager_t *dm = ivp_env_get_debug_manager(env);
        IVP_IF(dm->file_out_impacts) {
            FILE *fp = dm->out_deb_file;
            ivp_core_t *core0 = cp->synapse[0].l_obj->friction_core;
            ivp_core_t *core1 = cp->synapse[1].l_obj->friction_core;
            fprintf(fp, "  %u %u-%u: ", (unsigned)((uintptr_t)cp & 0x0000ffffU), (unsigned)((uintptr_t)core0 & 0x0000ffffU),
                    (unsigned)((uintptr_t)core1 & 0x0000ffffU));
            fprintf(fp, "di %.4f  ", (double)ivp_cp_get_gap_length(cp));
            double debug_cs = closing_speed + (double)(info->impact.rescue_speed_addon * 0.5f);
            fprintf(fp, "cs %.4f  ", debug_cs);
            fprintf(fp, "dr %.4f  ", (double)info->impact.distance_reached_in_time);
        }
        IVP_IF(dm->debug_imp_sys) {
            ivp_debug_printf("tested_frdist %u di %.4f cs %.4f dr %.4f\n", (unsigned)((uintptr_t)cp & 0x0000ffffU),
                             (double)ivp_cp_get_gap_length(cp),
                             closing_speed + (double)(info->impact.rescue_speed_addon * 0.5f),
                             (double)info->impact.distance_reached_in_time);
        }
    }
}

/* ── IVP_Impact_System ─────────────────────────────────────────────────── */

typedef struct impact_system {
    struct ivp_environment *l_environment;
    int        sum_of_pushes;
    ivp_pvec_t i_s_pushed_cores;
    ivp_pvec_t i_s_known_cores;
    ivp_pvec_t i_s_pairs;
    ivp_fr_system_t *associated_fs_system;
} impact_system_t;

static void isys_add_known_core(impact_system_t *is, ivp_core_t *c) { ivp_pvec_add(&is->i_s_known_cores, c); }

static void isys_add_pushed_core(impact_system_t *is, ivp_core_t *c)
{
    ivp_pvec_add(&is->i_s_pushed_cores, c);
    IVP_ASSERT(c->old_sync_rot && !c->old_sync_rot->was_pushed_during_i_s);
    if (c->old_sync_rot) c->old_sync_rot->was_pushed_during_i_s = true;
}

static bool isys_core_was_already_pushed(ivp_core_t *c) { return c->old_sync_rot && c->old_sync_rot->was_pushed_during_i_s; }
static bool isys_core_is_already_known(ivp_core_t *c) { return c->old_sync_rot != NULL; }

static bool isys_pair_is_already_in_system(impact_system_t *is, ivp_fr_core_pair_t *test_pair)
{
    for (int i = ivp_pvec_len(&is->i_s_pairs) - 1; i >= 0; i--) {
        if (ivp_pvec_at(&is->i_s_pairs, i) == test_pair) return true;
    }
    return false;
}

static void isys_add_pair(impact_system_t *is, ivp_fr_core_pair_t *p) { ivp_pvec_add(&is->i_s_pairs, p); }

static void isys_synchronize_core(ivp_core_t *new_core)
{
    if (IVP_MTIS_SIMULATED(new_core->movement_state)) ivp_core_synchronize_with_rot_z(new_core);
}

static void isys_add_pushed_core_with_pairs_except(impact_system_t *is, ivp_core_t *new_core, ivp_fr_core_pair_t *start_pair)
{
    isys_add_pushed_core(is, new_core);
    ivp_fr_system_t *fs = is->associated_fs_system;
    for (int i = ivp_pvec_len(&fs->fr_pairs_of_objs) - 1; i >= 0; i--) {
        if (i >= ivp_pvec_len(&fs->fr_pairs_of_objs)) continue;
        ivp_fr_core_pair_t *fr_pair = (ivp_fr_core_pair_t *)ivp_pvec_at(&fs->fr_pairs_of_objs, i);
        if ((fr_pair->objs[0] == new_core) || (fr_pair->objs[1] == new_core)) {
            if (fr_pair != start_pair) {
                if (!isys_pair_is_already_in_system(is, fr_pair)) {
                    ivp_core_t *pair_core0 = fr_pair->objs[0];
                    ivp_core_t *pair_core1 = fr_pair->objs[1];
                    if (ivp_fr_core_pair_check_all_fr_mindists_to_be_valid(fr_pair, fs) > 0) {
                        ivp_fr_core_pair_t *validated_pair = ivp_fr_system_find_pair_of_cores(fs, pair_core0, pair_core1);
                        if (validated_pair && !isys_pair_is_already_in_system(is, validated_pair)) {
                            isys_add_pair(is, validated_pair);
                        }
                    }
                }
            }
        }
    }
}

static void isys_check_start_pair(impact_system_t *is, ivp_fr_core_pair_t *start_pair, ivp_contact_pt_t *mdfr)
{
    for (int i = ivp_pvec_len(&start_pair->fr_dists) - 1; i >= 0; i--) {
        ivp_contact_pt_t *my_dist = (ivp_contact_pt_t *)ivp_pvec_at(&start_pair->fr_dists, i);
        if (mdfr != my_dist) {
            ivp_contact_pt_recalc_friction_s_vals(my_dist);
            ivp_impact_solver_lt_t *info = my_dist->tmp_contact_info;
            ivp_contact_pt_read_materials_for_contact_situation(my_dist, info);
            if (info->friction_is_broken) ivp_fr_system_delete_friction_distance(is->associated_fs_system, my_dist);
        }
    }
}

static void isys_invalidate_impact_mindists(impact_system_t *is, ivp_core_t *my_core)
{
    ivp_fr_info_for_core_t *my_info = ivp_core_get_friction_info(my_core, is->associated_fs_system);
    if (!my_info) return;
    for (int i = ivp_pvec_len(&my_info->friction_springs) - 1; i >= 0; i--) {
        ivp_contact_pt_t *my_dist = (ivp_contact_pt_t *)ivp_pvec_at(&my_info->friction_springs, i);
        my_dist->tmp_contact_info->coll_time_is_valid = false;
    }
}

static bool isys_test_loop_all_pairs(impact_system_t *is)
{
    ivp_contact_pt_t *worst_impact_dist = NULL;
    ivp_fr_core_pair_t *associated_pair = NULL;
    double smallest_distance = (double)ivp_mindist_settings.minimum_friction_dist;
    double rescue_speed_addon = 0.0;
    IVP_IF(ivp_env_get_debug_manager(is->l_environment)->file_out_impacts) {
        fprintf(ivp_env_get_debug_manager(is->l_environment)->out_deb_file, "calc_new_coll_distances\n");
    }
    for (int i = ivp_pvec_len(&is->i_s_pairs) - 1; i >= 0; i--) {
        ivp_fr_core_pair_t *my_pair = (ivp_fr_core_pair_t *)ivp_pvec_at(&is->i_s_pairs, i);
        int unmov0 = (int)my_pair->objs[0]->physical_unmoveable | (int)my_pair->objs[0]->temporarily_unmovable;
        int unmov1 = (int)my_pair->objs[1]->physical_unmoveable | (int)my_pair->objs[1]->temporarily_unmovable;
        if (unmov0 & unmov1) continue;
        for (int k = ivp_pvec_len(&my_pair->fr_dists) - 1; k >= 0; k--) {
            ivp_contact_pt_t *my_fr = (ivp_contact_pt_t *)ivp_pvec_at(&my_pair->fr_dists, k);
            if (my_fr->tmp_contact_info->coll_time_is_valid) {
                IVP_IF(ivp_env_get_debug_manager(is->l_environment)->debug_imp_sys) {
                    ivp_debug_printf("did_not_test %u\n", (unsigned)((uintptr_t)my_fr & 0x0000ffffU));
                }
            } else {
                ivp_env_get_statistic_manager(is->l_environment)->impact_coll_checks++;
                ivp_contact_pt_calc_coll_distance(my_fr);
            }
            if ((double)my_fr->tmp_contact_info->impact.distance_reached_in_time < smallest_distance) {
                rescue_speed_addon = (double)my_fr->tmp_contact_info->impact.rescue_speed_addon;
                smallest_distance = (double)my_fr->tmp_contact_info->impact.distance_reached_in_time;
                worst_impact_dist = my_fr;
                associated_pair = my_pair;
            }
        }
    }
    IVP_IF(ivp_env_get_debug_manager(is->l_environment)->file_out_impacts) {
        fprintf(ivp_env_get_debug_manager(is->l_environment)->out_deb_file, "\n");
    }
    if (!worst_impact_dist) return false;
    IVP_ASSERT((double)worst_impact_dist->tmp_contact_info->impact.distance_reached_in_time <
               (double)ivp_mindist_settings.minimum_friction_dist);
    for (int i = 1; i >= 0; i--) {
        ivp_core_t *core = worst_impact_dist->tmp_contact_info->contact_core[i];
        if (core && !isys_core_is_already_known(core)) {
            isys_add_known_core(is, core);
            isys_synchronize_core(core);
        }
    }
    ivp_core_t *impacting_cores[2];
    worst_impact_dist->tmp_contact_info->impacts_while_system++;
    ivp_impact_lt_do_impact_long_term(worst_impact_dist->tmp_contact_info, impacting_cores, (float)rescue_speed_addon,
                                      worst_impact_dist);
    for (int i = 1; i >= 0; i--) {
        ivp_core_t *core = impacting_cores[i];
        if (core && !core->physical_unmoveable && !core->pinned) {
            if (!isys_core_was_already_pushed(core)) {
                isys_add_pushed_core_with_pairs_except(is, core, associated_pair);
            } else {
                isys_invalidate_impact_mindists(is, core);
            }
        }
    }
    return true;
}

static void isys_recalc_all_affected_cores(impact_system_t *is)
{
    IVP_IF(1) {
        for (int k = ivp_pvec_len(&is->i_s_pushed_cores) - 1; k >= 0; k--) {
            ivp_core_t *my_core = (ivp_core_t *)ivp_pvec_at(&is->i_s_pushed_cores, k);
            /* every pushed core has to be known */
            bool found = false;
            for (int j = ivp_pvec_len(&is->i_s_known_cores) - 1; j >= 0; j--) {
                ivp_core_t *test_core = (ivp_core_t *)ivp_pvec_at(&is->i_s_known_cores, j);
                if (my_core == test_core) found = true;
            }
            IVP_ASSERT(found == true);
            IVP_ASSERT(my_core->old_sync_rot && my_core->old_sync_rot->was_pushed_during_i_s == true);
            IVP_USE(found);
        }
    }
    for (int j = ivp_pvec_len(&is->i_s_known_cores) - 1; j >= 0; j--) {
        ivp_core_t *my_core2 = (ivp_core_t *)ivp_pvec_at(&is->i_s_known_cores, j);
        if (!isys_core_was_already_pushed(my_core2)) {
            ivp_core_undo_synchronize_rot_z(my_core2);
        }
        my_core2->old_sync_rot = NULL;
    }
    {
        double delta_time_till_next_psi =
            ivp_time_diff(ivp_env_get_next_psi_time(is->l_environment), ivp_env_get_current_time(is->l_environment));
        ivp_pvec_t active_hull_managers;
        ivp_pvec_init(&active_hull_managers);
        for (int k = ivp_pvec_len(&is->i_s_pushed_cores) - 1; k >= 0; k--) {
            ivp_core_t *my_core = (ivp_core_t *)ivp_pvec_at(&is->i_s_pushed_cores, k);
            IVP_ASSERT(!my_core->physical_unmoveable); /* core merge? */
            if (!my_core->physical_unmoveable) {
                ivp_sim_es_t es;
                ivp_sim_es_init_dt(&es, is->l_environment, delta_time_till_next_psi);
                ivp_cnps_calc_next_psi_matrix(my_core, &es, &active_hull_managers);
                my_core->old_sync_rot = NULL;
                isys_invalidate_impact_mindists(is, my_core);
            }
        }
        ivp_cnps_commit_all_hull_managers(is->l_environment, &active_hull_managers);
        ivp_pvec_clear(&active_hull_managers);
    }
    for (int k = ivp_pvec_len(&is->i_s_pushed_cores) - 1; k >= 0; k--) {
        ivp_core_t *my_core = (ivp_core_t *)ivp_pvec_at(&is->i_s_pushed_cores, k);
        if (!my_core->physical_unmoveable) ivp_core_update_exact_mindist_events_of_core(my_core);
    }
}

/* IVP_Impact_System::debug_check_all_dists_at_end.  Called in every debug build (IVP_IF(1)): its
 * recalc_friction_s_vals / calc_coll_distance on every friction dist of the system change the
 * simulation of the reference debug build (span_friction_s, last_gap_len, ...). */
static void isys_debug_check_all_dists_at_end(impact_system_t *is)
{
    ivp_fr_system_t *fs = is->associated_fs_system;
    for (ivp_contact_pt_t *my_dist = fs->first_friction_dist; my_dist; my_dist = my_dist->next_dist_in_friction) {
        ivp_contact_pt_recalc_friction_s_vals(my_dist);
        ivp_contact_pt_calc_coll_distance(my_dist);
        my_dist->tmp_contact_info->coll_time_is_valid = false;
        if (my_dist->tmp_contact_info->impact.distance_reached_in_time < ivp_mindist_settings.minimum_friction_dist) {
            ivp_core_t *core0 = my_dist->synapse[0].l_obj->physical_core;
            ivp_core_t *core1 = my_dist->synapse[1].l_obj->physical_core;
            int unmov0 = (int)core0->physical_unmoveable | (int)core0->temporarily_unmovable;
            int unmov1 = (int)core1->physical_unmoveable | (int)core1->temporarily_unmovable;
            if (unmov0 & unmov1) continue;
            if (isys_core_is_already_known(core0) && isys_core_is_already_known(core1)) {
                ivp_debug_printf("error_in_impact_system_missed_impact\n");
            }
        }
    }
}

static void isys_init_and_solve(ivp_fr_system_t *fs_system, ivp_fr_core_pair_t *start_pair, ivp_contact_pt_t *start_fr_dist)
{
    if (!fs_system || !start_pair || !start_fr_dist || !start_pair->objs[0] || !start_pair->objs[1]) {
        IVP_ASSERT(fs_system && start_pair && start_fr_dist && start_pair->objs[0] && start_pair->objs[1]);
        return;
    }
    impact_system_t is;
    memset(&is, 0, sizeof(is));
    ivp_pvec_init(&is.i_s_pushed_cores);
    ivp_pvec_init(&is.i_s_known_cores);
    ivp_pvec_init(&is.i_s_pairs);
    ivp_core_t *start_core0 = start_fr_dist->synapse[0].l_obj->physical_core;
    ivp_core_t *start_core1 = start_fr_dist->synapse[1].l_obj->physical_core;
    is.sum_of_pushes = 0;
    is.associated_fs_system = fs_system;
    is.l_environment = ivp_core_env(start_pair->objs[0]);
    if (!start_core0->physical_unmoveable && !start_core0->pinned) {
        isys_add_pushed_core_with_pairs_except(&is, start_core0, start_pair);
    }
    ivp_core_t *core0 = start_pair->objs[0];
    if (!core0->physical_unmoveable && !core0->pinned) isys_add_known_core(&is, core0);
    if (!start_core1->physical_unmoveable && !start_core1->pinned) {
        isys_add_pushed_core_with_pairs_except(&is, start_core1, start_pair);
    }
    ivp_core_t *core1 = start_pair->objs[1];
    if (!core1->physical_unmoveable && !core1->pinned) isys_add_known_core(&is, core1);
    isys_add_pair(&is, start_pair);
    isys_check_start_pair(&is, start_pair, start_fr_dist);
    ivp_debug_manager_t *dm = ivp_env_get_debug_manager(is.l_environment);
    IVP_IF(1) {
        dm->debug_imp_sys = false;
    }
    IVP_IF(dm->file_out_impacts) {
        if (ivp_env_get_current_time(is.l_environment) > (double)10000.86f) {
            ivp_debug_printf("critical_imp_sys\n");
            dm->debug_imp_sys = true;
        }
        fprintf(dm->out_deb_file, "starting_impact_system at time %f\n", ivp_env_get_current_time(is.l_environment));
    }
    int impact_sys_counter = 0;
    while (isys_test_loop_all_pairs(&is)) {
        impact_sys_counter++;
        is.sum_of_pushes++;
        if (is.sum_of_pushes >= MAXIMUM_SYSTEM_PUSH_NUMBER) {
            IVP_IF(1) {
                ivp_debug_printf("system_push_number exceeded\n");
            }
            break;
        }
    }
    ivp_env_get_statistic_manager(is.l_environment)->impact_sum_sys += impact_sys_counter + 1;
    IVP_IF(is.sum_of_pushes > 1) {
        /* printf("did_impact_sys ...") (commented out) and an "#if 0" pair dump: not ported */
    }
    IVP_IF(1) {
        isys_debug_check_all_dists_at_end(&is);
    }
    isys_recalc_all_affected_cores(&is);
    IVP_IF(1) {
        ivp_fr_system_debug_clean_tmp_info(is.associated_fs_system);
    }
    IVP_IF(dm->file_out_impacts) {
        dm->debug_imp_sys = false;
        fprintf(dm->out_deb_file, "\n");
    }
    ivp_pvec_clear(&is.i_s_pushed_cores);
    ivp_pvec_clear(&is.i_s_known_cores);
    ivp_pvec_clear(&is.i_s_pairs);
}

/* ── IVP_Impact_Solver_Long_Term::do_impact_of_two_objects ─────────────── */

void ivp_impact_lt_do_impact_of_two_objects(ivp_mindist_t *mindist, ivp_object_t *obj0, ivp_object_t *obj1)
{
    struct ivp_fr_system *affected_friction_system = NULL;
    bool having_new_dist;
    ivp_core_t *core0 = obj0->physical_core;
    ivp_core_t *core1 = obj1->physical_core;
    struct ivp_sim_unit *sim_u;
    if (obj0->object_movement_state >= IVP_MT_NOT_SIM) {
        sim_u = core0->sim_unit;
    } else {
        sim_u = core1->sim_unit;
    }
    ivp_contact_pt_t *my_fr_dist = ivp_mindist_try_to_generate_managed_friction(mindist, &affected_friction_system,
                                                                                &having_new_dist, sim_u, true);
    if (!my_fr_dist || !affected_friction_system) {
        IVP_ASSERT(my_fr_dist && affected_friction_system);
        return;
    }
    ivp_impact_solver_lt_t *imp_solve = my_fr_dist->tmp_contact_info;
    if (!imp_solve) {
        IVP_ASSERT(imp_solve);
        return;
    }
    ivp_fr_core_pair_t *pair = ivp_fr_system_find_pair_of_cores(affected_friction_system, core0, core1);
    if (!pair) {
        IVP_ASSERT(pair);
        return;
    }
    struct ivp_environment *env = mindist->env;
    ivp_time_t now_time = ivp_env_get_current_time(env);
    ivp_time_t last_time = pair->last_impact_time_pair;
    pair->last_impact_time_pair = now_time;
    float rescue_val_addon = ivp_contact_pt_get_rescue_speed_impact(my_fr_dist, env);
    double d_time_since_last = ivp_time_diff(now_time, last_time);
    ivp_env_fire_pre_collision(env, imp_solve, d_time_since_last);
    if (ivp_env_object_deletion_is_deferred(env, obj0) || ivp_env_object_deletion_is_deferred(env, obj1)) return;
    ivp_core_t *impacteers[2];
    ivp_impact_lt_do_impact_long_term(imp_solve, impacteers, rescue_val_addon, my_fr_dist);
    ivp_vec3f_t speed_for_callback = imp_solve->speed;
    if (core1->physical_unmoveable) lvf_mult(&speed_for_callback, -1.0f);
    isys_init_and_solve(affected_friction_system, pair, my_fr_dist);
    imp_solve->speed = speed_for_callback;
    ivp_env_fire_post_collision(env, imp_solve, d_time_since_last);
}

/* ── debug helpers of ivp_mindist_impact.cxx ───────────────────────────── */

/* ── IVP_Mindist::try_to_generate_managed_friction ─────────────────────── */

ivp_contact_pt_t *ivp_mindist_try_to_generate_managed_friction(ivp_mindist_t *my_dist, struct ivp_fr_system **associated_fs,
                                                               bool *having_new, struct ivp_sim_unit *sim_unit_not_destroy,
                                                               bool call_recalc_svals)
{
    ivp_object_t *obj0 = ivp_mindist_get_sorted_synapse(my_dist, 0)->l_obj;
    ivp_object_t *obj1 = ivp_mindist_get_sorted_synapse(my_dist, 1)->l_obj;
    IVP_IF(ivp_env_get_debug_manager(obj0->owner)->debug_friction) {
        ivp_debug_printf("new_fri_mindist\n");
    }
    ivp_core_t *core0 = obj0->friction_core;
    ivp_core_t *core1 = obj1->friction_core;
    if (core0->physical_unmoveable) {
        ivp_core_t *t = core0;
        core0 = core1;
        core1 = t;
    }
    ivp_fr_info_for_core_t *fr_info0 = ivp_core_moveable_core_has_friction_info(core0);
    ivp_fr_info_for_core_t *fr_info1;
    struct ivp_environment *my_env = ivp_core_env(core0);
    bool gen_success;
    *associated_fs = NULL;
    *having_new = false;
    ivp_contact_pt_t *friction_dist = ivp_friction_mgr_generate_contact_point(my_dist, &gen_success);
    if (!friction_dist) {
        if (fr_info0) *associated_fs = fr_info0->l_friction_system;
        return NULL;
    }
    if (!gen_success) {
        *associated_fs = friction_dist->l_friction_system;
        if (!*associated_fs && fr_info0) *associated_fs = fr_info0->l_friction_system;
        *having_new = false;
        if (call_recalc_svals) {
            ivp_contact_pt_recalc_friction_s_vals(friction_dist);
            ivp_contact_pt_read_materials_for_contact_situation(friction_dist, friction_dist->tmp_contact_info);
        }
        return friction_dist;
    }
    if (call_recalc_svals) {
        ivp_contact_pt_recalc_friction_s_vals(friction_dist);
        ivp_contact_pt_read_materials_for_contact_situation(friction_dist, friction_dist->tmp_contact_info);
    }
    ivp_env_fire_friction_created(obj0->owner, friction_dist);
    *having_new = true;
    ivp_fr_system_t *affected_fs;
    if (fr_info0) {
        ivp_fr_system_t *fr_sys0 = fr_info0->l_friction_system;
        fr_info1 = ivp_core_get_friction_info(core1, fr_sys0);
        if (fr_info1) {
            ivp_fr_system_add_dist_to_system(fr_sys0, friction_dist);
            ivp_fr_system_dist_added_update_pair_info(fr_sys0, friction_dist);
            affected_fs = fr_sys0;
        } else {
            if (!core1->physical_unmoveable) {
                fr_info1 = ivp_core_moveable_core_has_friction_info(core1);
                if (fr_info1) {
                    ivp_fr_system_fusion_friction_systems(fr_sys0, fr_info1->l_friction_system);
                    goto add_second_to_first_dist_only;
                }
            }
            fr_info1 = ivp_fr_info_create(ivp_env_get_allocator(my_env));
            fr_info1->l_friction_system = fr_sys0;
            ivp_core_add_friction_info(core1, fr_info1);
            ivp_fr_system_add_core_to_system(fr_sys0, core1);
        add_second_to_first_dist_only:
            ivp_fr_system_add_dist_to_system(fr_sys0, friction_dist);
            ivp_fr_system_dist_added_update_pair_info(fr_sys0, friction_dist);
            affected_fs = fr_sys0;
        }
    } else {
        fr_info0 = ivp_fr_info_create(ivp_env_get_allocator(my_env));
        if (!core1->physical_unmoveable && (fr_info1 = ivp_core_moveable_core_has_friction_info(core1)) != NULL) {
            ivp_fr_system_t *fr_sys1 = fr_info1->l_friction_system;
            fr_info0->l_friction_system = fr_sys1;
            ivp_core_add_friction_info(core0, fr_info0);
            ivp_fr_system_add_core_to_system(fr_sys1, core0);
            ivp_fr_system_add_dist_to_system(fr_sys1, friction_dist);
            ivp_fr_system_dist_added_update_pair_info(fr_sys1, friction_dist);
            affected_fs = fr_sys1;
        } else {
            ivp_fr_system_t *new_system = ivp_fr_system_create(my_env);
            fr_info1 = ivp_fr_info_create(ivp_env_get_allocator(my_env));
            fr_info0->l_friction_system = new_system;
            fr_info1->l_friction_system = new_system;
            ivp_core_add_friction_info(core1, fr_info1);
            ivp_core_add_friction_info(core0, fr_info0);
            ivp_fr_system_add_dist_to_system(new_system, friction_dist);
            ivp_fr_system_dist_added_update_pair_info(new_system, friction_dist);
            ivp_fr_system_add_core_to_system(new_system, core0);
            ivp_fr_system_add_core_to_system(new_system, core1);
            affected_fs = new_system;
        }
    }
    ivp_fr_info_insert_friction_dist(fr_info0, friction_dist);
    ivp_fr_info_insert_friction_dist(fr_info1, friction_dist);
    *associated_fs = affected_fs;
#ifdef IVP_DEBUG_BUILD
    IVP_IF(0) {
        ivp_fr_system_t *fs0 = NULL;
        ivp_fr_system_t *fs1 = NULL;
        if (!core0->physical_unmoveable) {
            ivp_fr_info_for_core_t *info_friction = ivp_core_moveable_core_has_friction_info(core0);
            if (info_friction) fs0 = info_friction->l_friction_system;
        }
        if (!core1->physical_unmoveable) {
            ivp_fr_info_for_core_t *info_for_core = ivp_core_moveable_core_has_friction_info(core1);
            if (info_for_core) fs1 = info_for_core->l_friction_system;
        }
        ivp_debug_printf("\nmaking_new_frdist for cores %p %d  %p %d  fss %p %p  with sim_units: %p %p\n",
                         (void *)core0, (int)core0->physical_unmoveable, (void *)core1, (int)core1->physical_unmoveable,
                         (void *)fs0, (void *)fs1, (void *)core0->sim_unit, (void *)core1->sim_unit);
        ivp_sim_unit_sim_unit_debug_out(core0->sim_unit);
        ivp_sim_unit_sim_unit_debug_out(core1->sim_unit);
    }
#endif
    ivp_contact_pt_calc_virtual_mass_of_mindist(friction_dist);

    IVP_IF(ivp_env_get_debug_manager(obj0->owner)->check_fs) {
        ivp_fr_system_test_hole_fr_system_data(affected_fs);
    }

    ivp_sim_unit_t *sim0 = core0->sim_unit;
    ivp_sim_unit_t *sim1 = core1->sim_unit;
    if (!((int)core1->physical_unmoveable | (int)core0->physical_unmoveable)) {
        if (sim0 != sim1) {
            if (sim1 == sim_unit_not_destroy) {
                if (ivp_sim_unit_fusion_simulation_unities(sim1, sim0)) ivp_sim_unit_delete(sim0);
            } else {
                if (ivp_sim_unit_fusion_simulation_unities(sim0, sim1)) ivp_sim_unit_delete(sim1);
            }
        }
    }
#ifdef IVP_DEBUG_BUILD
    IVP_IF(1) {
        if (!core0->physical_unmoveable) ivp_sim_unit_sim_unit_debug_consistency(core0->sim_unit);
        if (!core1->physical_unmoveable) ivp_sim_unit_sim_unit_debug_consistency(core1->sim_unit);
    }
#endif
    return friction_dist;
}

/* ── IVP_Mindist::do_impact ───────────────────────────────────────────── */

static void mindist_base_do_impact(ivp_mindist_t *md)
{
    ivp_mindist_t *previous_mindist = ivp_g_current_mindist;
    bool previous_defer_delete = ivp_g_defer_delete_mindist;
    ivp_g_defer_delete_mindist = false;
    ivp_g_current_mindist = md;
    struct ivp_environment *env = md->env;
    IVP_ASSERT(md->mindist_status == IVP_MD_EXACT);
    ivp_object_t *objects[2];
    for (int i = 0; i < 2; i++) {
        ivp_object_t *obj = md->synapse[i].l_obj;
        objects[i] = obj;
        ivp_object_revive_object_for_simulation(obj);
    }
    if (ivp_g_defer_delete_mindist) {
        IVP_ASSERT(0);
        ivp_g_current_mindist = previous_mindist;
        ivp_g_defer_delete_mindist = previous_defer_delete;
        ivp_collision_delete(&md->coll);
        return;
    }
    ivp_env_sim_mem_begin(env);
    for (int j = 0; j < 2; j++) {
        ivp_core_t *core = objects[j]->physical_core;
        if (IVP_MTIS_SIMULATED(core->movement_state) && !core->pinned) {
            ivp_core_synchronize_with_rot_z(core);
        }
    }
    ivp_env_inc_mindist_event_timestamp_reference(env);
    ivp_env_begin_deferred_deletion(env);
    ivp_impact_lt_do_impact_of_two_objects(md, objects[0], objects[1]);
    ivp_env_sim_mem_end(env);
    bool delete_current = ivp_g_defer_delete_mindist;
    ivp_g_current_mindist = previous_mindist;
    ivp_g_defer_delete_mindist = previous_defer_delete;
    if (delete_current) ivp_collision_delete(&md->coll);
    ivp_env_end_deferred_deletion(env);
}

void ivp_mindist_do_impact(ivp_mindist_t *md)
{
    if (md->is_recursive && ivp_mindist_recursive_do_impact(md)) return;
    mindist_base_do_impact(md);
}

/* ── IVP_Mindist::mindist_rescue_push / IVP_Anomaly_Manager::inter_penetration ── */

static void anomaly_solve_inter_penetration_simple(ivp_object_t *obj0, ivp_object_t *obj1, double speed_change)
{
    struct ivp_environment *env = obj0->owner;
    double d_time = (double)ivp_env_get_delta_psi_time(env);
    ivp_core_t *core0 = obj0->physical_core;
    ivp_core_t *core1 = obj1->physical_core;
    const ivp_transform_r_t *m0 = &core0->m_world_f_core_last_psi;
    const ivp_transform_r_t *m1 = &core1->m_world_f_core_last_psi;
    ivp_vec3f_t vec01;
    lvf_subtract2_p(&vec01, &m1->pos, &m0->pos);
    float delta_length = (float)lvf_real_length_plus_normize(&vec01);
    if (delta_length <= 0.01) lvf_set3(&vec01, 1.0f, 0.0f, 0.0f);
    if (IVP_MTIS_SIMULATED(core0->movement_state) && !core0->pinned) {
        ivp_vec3f_t p0;
        lvf_set_multiple(&p0, &vec01, (D)(-speed_change * (double)ivp_core_get_mass_f(core0)));
        ivp_object_async_push_object_ws(obj0, &m0->pos, &p0);
        ivp_vec3f_t rot_speed_object;
        lvf_setd3(&rot_speed_object, (D)-d_time, 0, 0);
        ivp_object_async_add_rot_speed_object_cs(obj0, &rot_speed_object);
    }
    if (IVP_MTIS_SIMULATED(core1->movement_state) && !core1->pinned) {
        ivp_vec3f_t p1;
        lvf_set_multiple(&p1, &vec01, (D)(speed_change * (double)ivp_core_get_mass_f(core1)));
        ivp_object_async_push_object_ws(obj1, &m1->pos, &p1);
        ivp_vec3f_t rot_speed_object;
        lvf_setd3(&rot_speed_object, (D)d_time, 0, 0);
        ivp_object_async_add_rot_speed_object_cs(obj1, &rot_speed_object);
    }

    IVP_IF(1) /* ivp_anomaly_manager.cxx:104 */
    {
        const char *name0 = obj0->name;
        if (!name0) name0 = "(null)";
        const char *name1 = obj1->name;
        if (!name1) name1 = "(null)";
        ivp_debug_printf("Mindist rescue push between '%s' and '%s' at %f\n", name0, name1,
                         ivp_env_get_current_time(env));
    }
}

static void anomaly_inter_penetration(ivp_mindist_t *mindist, ivp_object_t *obj0, ivp_object_t *obj1, double speed_change)
{
    ivp_object_t *my_objects[2] = { obj0, obj1 };
    int swapped = 0;
    if (obj1->physical_core->physical_unmoveable) swapped = 1;
    if (!my_objects[swapped]->physical_core->physical_unmoveable) {
        anomaly_solve_inter_penetration_simple(obj0, obj1, speed_change);
        return;
    }
    if (mindist->synapse[swapped].status > IVP_ST_TRIANGLE) {
        anomaly_solve_inter_penetration_simple(obj0, obj1, speed_change);
        return;
    }
    ivp_object_t *moving_obj = my_objects[1 - swapped];
    ivp_vec3r_t world_pos_moving = moving_obj->physical_core->m_world_f_core_last_psi.pos;
    ivp_cache_obj_t *unmovable_cache = ivp_object_get_cache_object_no_lock(my_objects[swapped]);
    ivp_vec3r_t obj_pos_moving;
    lvm_vimult4(&unmovable_cache->m_world_f_object, &world_pos_moving, &obj_pos_moving);
    const ivp_compact_ledge_t *my_compact_ledge = ivp_md_synapse_get_ledge(&mindist->synapse[swapped]);
    const ivp_compact_triangle_t *my_compact_triangle = ivp_ledge_get_first_triangle(my_compact_ledge);
    ivp_vec3r_t push_direction;
    lvp_set3(&push_direction, 1.0f, 0.0f, 0.0f);
    D biggest_dist = -(D)P_DOUBLE_MAX;
    const ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array(my_compact_ledge);
    for (int i = 0; i < ivp_ledge_get_n_triangles(my_compact_ledge); i++) {
        ivp_vec3r_t three_points[3];
        for (int e = 0; e < 3; e++) {
            const ivp_compact_poly_point_t *p = &pts[ivp_edge_get_start_point_index(ivp_tri_get_edge(my_compact_triangle, e))];
            lvp_set3(&three_points[e], (D)p->k[0], (D)p->k[1], (D)p->k[2]);
        }
        lv_hesse_t hesse_plane;
        lv_hesse_calc_ppp(&hesse_plane, &three_points[0], &three_points[2], &three_points[1]);
        lv_hesse_normize(&hesse_plane);
        D distance = lv_hesse_get_dist(&hesse_plane, &obj_pos_moving);
        if (distance > biggest_dist) {
            biggest_dist = distance;
            push_direction = hesse_plane.n;
        }
        my_compact_triangle = my_compact_triangle + 1;
    }
    ivp_vec3r_t world_push_dir;
    lvm_vmult3(&unmovable_cache->m_world_f_object, &push_direction, &world_push_dir);
    ivp_vec3f_t world_push_vec;
    lvf_set_p(&world_push_vec, &world_push_dir);
    ivp_core_t *moving_core = moving_obj->physical_core;
    if (IVP_MTIS_SIMULATED(moving_core->movement_state)) {
        lvf_mult(&world_push_vec, (D)speed_change);
        ivp_object_async_add_speed_object_ws(moving_obj, &world_push_vec);
    }
}

void ivp_mindist_mindist_rescue_push(ivp_mindist_t *md)
{
    if (md->is_recursive) return;   /* IVP_Mindist_Recursive::mindist_rescue_push: "will be solved later" */
    ivp_object_t *obj0 = md->synapse[0].l_obj;
    ivp_object_t *obj1 = md->synapse[1].l_obj;
    struct ivp_environment *env = obj0->owner;
    /* IVP_Anomaly_Manager::get_push_speed_penetration */
    double d_time = (double)ivp_env_get_delta_psi_time(env);
    double grav = ivp_env_get_gravity_scalar(env);
    double speed_change = (grav * 4.0f) * d_time;
    anomaly_inter_penetration(md, obj0, obj1, (double)(float)speed_change);
}

/* ── IVP_Mindist::create_cp_in_advance_pretension ──────────────────────── */

void ivp_mindist_create_cp_in_advance_pretension(ivp_mindist_t *md, ivp_object_t *robject, float gap_len)
{
    ivp_fr_synapse_t *old_fr_syn = robject->fr_synapses;
    if (!old_fr_syn) return;
    struct ivp_environment *env = robject->owner;
    ivp_env_sim_mem_begin(env);
    struct ivp_fr_system *associated_fs;
    bool success;
    ivp_contact_pt_t *new_cp = ivp_mindist_try_to_generate_managed_friction(md, &associated_fs, &success, NULL, true);
    if (success && new_cp) {
        ivp_contact_pt_t *old_cp = old_fr_syn->contact_point;
        ivp_time_t old_time = old_cp->last_time_of_recalc_friction_s_vals;
        old_cp->last_time_of_recalc_friction_s_vals = ivp_env_get_current_time(env);
        ivp_contact_pt_recalc_friction_s_vals(old_cp);
        new_cp->now_friction_pressure = old_cp->now_friction_pressure;
        old_cp->now_friction_pressure = 0.0f;
        new_cp->last_gap_len = gap_len;
        new_cp->last_time_of_recalc_friction_s_vals = old_time;
        old_cp->last_time_of_recalc_friction_s_vals = old_time;
        ivp_vec3f_t world_friction_vec;
        lvf_set_multiple(&world_friction_vec, &old_cp->tmp_contact_info->span_friction_v[0], (D)old_cp->span_friction_s[0]);
        lvf_add_multiple(&world_friction_vec, &old_cp->tmp_contact_info->span_friction_v[1], (D)old_cp->span_friction_s[1]);
        float s1 = (float)lvf_dot(&new_cp->tmp_contact_info->span_friction_v[0], &world_friction_vec);
        float s2 = (float)lvf_dot(&new_cp->tmp_contact_info->span_friction_v[1], &world_friction_vec);
        new_cp->span_friction_s[0] = s1;
        new_cp->span_friction_s[1] = s2;
    }
    ivp_env_sim_mem_end(env);
}
