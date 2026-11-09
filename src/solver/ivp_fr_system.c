/* ivp_fr_system.c — port of IVP_Friction_System / IVP_Friction_Core_Pair /
 * IVP_Friction_Solver / IVP_Mutual_Energizer / IVP_Friction_Sys_Static / _Energy
 *
 * Reference: ivp_intern/ivp_friction.cxx, ivp_intern/ivp_friction_gaps.cxx,
 * ivp_intern/ivp_friction_solver.hxx, ivp_intern/ivp_physic_private.cxx
 * (friction system ctor, dist/core bookkeeping).
 *
 * Reference build: IVP_DISTANCE_KEEPERS / IVP_PANELTY_SOLVER undefined, easing and
 * mutual energy destruction enabled, IVP_FAST_WHEELS_ENABLED defined.  The
 * reference IVP_IF blocks and its "#ifdef DEBUG" code are ported as IVP_IF /
 * IVP_DEBUG_BUILD (dead in release builds, live in debug builds).
 */
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>

#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <ivp/diagnostics/ivp_debug_manager.h>
#include <ivp/diagnostics/ivp_statistic_manager.h>
#include "base/ivp_arena_internal.h"
#include <ivp/controllers/ivp_controller.h>
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/solver/ivp_great_matrix.h>
#include <ivp/solver/ivp_linear_constraint_solver.h>
#include "solver/ivp_scr_internal.h"
#include "collision/ivp_lvec.h"
#include "solver/ivp_friction_internal.h"
#include "controllers/ivp_car_wheel_physics.h"
#include "dynamics/ivp_env_internal.h"
#include "dynamics/ivp_env_hot.h"
#include "dynamics/ivp_env_parallel_tasks.h"

/* sizeof(IVP_Contact_Point *) of the 64-bit reference (arena allocation size) */
#define IVP_FR_REF_POINTER_SIZE 8u
#define IVP_FR_PARALLEL_MIN_INTERACTIONS 16384u
#define IVP_FR_PARALLEL_MAX_WORKERS 4
/* Adjacent columns share cache lines in every matrix row.  Giving one lane a
 * short contiguous run cuts both task claims and cross-lane row boundaries. */
#define IVP_FR_PARALLEL_COLUMN_BLOCK 16u

#define FS_FROM(ptr, member) ((ivp_fr_system_t *)(void *)((char *)(ptr) - offsetof(ivp_fr_system_t, member)))

static inline ivp_contact_pt_t *cp_at(const ivp_pvec_t *v, int i) { return (ivp_contact_pt_t *)ivp_pvec_at(v, i); }
static inline ivp_core_t *core_at(const ivp_pvec_t *v, int i) { return (ivp_core_t *)ivp_pvec_at(v, i); }
static inline ivp_fr_core_pair_t *pair_at(const ivp_pvec_t *v, int i) { return (ivp_fr_core_pair_t *)ivp_pvec_at(v, i); }

/* synapse[i].l_obj->physical_core (get_core()) / ->friction_core */
static ivp_core_t *cp_core(ivp_contact_pt_t *cp, int i) { return cp->synapse[i].l_obj->physical_core; }
static ivp_core_t *cp_fcore(ivp_contact_pt_t *cp, int i) { return cp->synapse[i].l_obj->friction_core; }

/* ── debug support (reference IVP_IF code) ─────────────────────────────── */

/* env->get_debug_manager() (== env->debug_information) */
static inline ivp_debug_manager_t *fr_dm(struct ivp_environment *env) { return ivp_env_get_debug_manager(env); }

/* (uint)((uintp)ptr & 0x0000ffffU) of the reference debug prints */
static inline unsigned fr_p16(const void *p) { return (unsigned)((uintptr_t)p & 0x0000ffffU); }

/* IVP_Environment::add_draw_vector(IVP_U_Point *, IVP_U_Float_Point *, text, color) */
static void fr_add_draw_vector(struct ivp_environment *env, const ivp_vec3r_t *start_p, const ivp_vec3f_t *dir_vec,
                               const char *debug_text, int color)
{
    const double s[3] = { (double)start_p->k[0], (double)start_p->k[1], (double)start_p->k[2] };
    const float d[3] = { dir_vec->k[0], dir_vec->k[1], dir_vec->k[2] };
    ivp_env_add_draw_vector(env, s, d, debug_text, color);
}

/* object name for the reference's printf("%s", get_name()) (NULL: "(null)") */
static inline const char *fr_obj_name(const ivp_object_t *obj) { return obj->name ? obj->name : "(null)"; }

/* The reference counters are static class members.  libivp keeps their
 * single-world ordering but stores them on the environment so worlds do not
 * race or influence one another's debug text. */

/* ── IVP_Impact_Solver_Long_Term::get_closing_speed ─────────────────────── */

double ivp_impact_lt_get_closing_speed(const ivp_impact_solver_lt_t *info)
{
    double result = 0.0;
    ivp_core_t *c0 = info->contact_core[0];
    if (c0) {
        ivp_vec3f_t sp = ivp_core_speed_f(c0);
        result = (double)(lvf_dot(&sp, &info->surf_normal) + lvf_dot(&c0->rot_speed, &info->contact_cross_nomal_cs[0]));
    }
    ivp_core_t *c1 = info->contact_core[1];
    if (c1) {
        ivp_vec3f_t sp = ivp_core_speed_f(c1);
        result -= (double)(lvf_dot(&sp, &info->surf_normal) + lvf_dot(&c1->rot_speed, &info->contact_cross_nomal_cs[1]));
    }
    return result;
}

/* ── IVP_Friction_Solver statics ────────────────────────────────────────── */

/* IVP_U_Float_Point::add_multiple on the core's float speed vectors
 * (speed_change / rot_speed_change are IVP_U_Float_Point in the reference) */
static inline void vr_add_multiple_f(ivp_vec3r_t *a, const ivp_vec3f_t *v, double f)
{
    a->k[0] = (ivp_real_t)(float)((double)a->k[0] + (double)v->k[0] * f);
    a->k[1] = (ivp_real_t)(float)((double)a->k[1] + (double)v->k[1] * f);
    a->k[2] = (ivp_real_t)(float)((double)a->k[2] + (double)v->k[2] * f);
}


static inline void vr_rot_add_multiple(ivp_vec3f_t *a, const ivp_vec3f_t *v, double f) {
    lvf_add_multiple(a, v, (D)f);
}

static inline void vrr_add_multiple_f(ivp_vec3r_t *a, const ivp_vec3f_t *v, double f) { vr_add_multiple_f(a, v, f); }

void ivp_frs_apply_impulse(const ivp_impact_solver_lt_t *info, double impulse)
{
    ivp_core_t *c0 = info->contact_core[0];
    if (c0) {
        ivp_vec3f_t hp, iri = ivp_core_inv_rot_inertia_f(c0);
        lvf_set_pairwise_mult(&hp, &info->contact_cross_nomal_cs[0], &iri);
        vr_rot_add_multiple(&c0->rot_speed, &hp, -impulse);
        vr_add_multiple_f(&c0->speed, &info->surf_normal, -impulse * (double)ivp_core_get_inv_mass_f(c0));
    }
    ivp_core_t *c1 = info->contact_core[1];
    if (c1) {
        ivp_vec3f_t hp, iri = ivp_core_inv_rot_inertia_f(c1);
        lvf_set_pairwise_mult(&hp, &info->contact_cross_nomal_cs[1], &iri);
        vr_rot_add_multiple(&c1->rot_speed, &hp, impulse);
        vr_add_multiple_f(&c1->speed, &info->surf_normal, impulse * (double)ivp_core_get_inv_mass_f(c1));
    }
}

void ivp_frs_async_apply_impulse(const ivp_impact_solver_lt_t *info, double impulse)
{
    ivp_core_t *c0 = info->contact_core[0];
    if (c0) {
        ivp_vec3f_t hp, iri = ivp_core_inv_rot_inertia_f(c0);
        lvf_set_pairwise_mult(&hp, &info->contact_cross_nomal_cs[0], &iri);
        vrr_add_multiple_f(&c0->rot_speed_change, &hp, -impulse);
        vr_add_multiple_f(&c0->speed_change, &info->surf_normal, -impulse * (double)ivp_core_get_inv_mass_f(c0));
    }
    ivp_core_t *c1 = info->contact_core[1];
    if (c1) {
        ivp_vec3f_t hp, iri = ivp_core_inv_rot_inertia_f(c1);
        lvf_set_pairwise_mult(&hp, &info->contact_cross_nomal_cs[1], &iri);
        /* retail RVA 0x364C0 keeps the second core's X inertia product
         * wide; only Y/Z pass through the float hp */
        c1->rot_speed_change.k[0] = (ivp_real_t)(float)((double)c1->rot_speed_change.k[0] +
            ((double)info->contact_cross_nomal_cs[1].k[0] * (double)iri.k[0]) * impulse);
        c1->rot_speed_change.k[1] = (ivp_real_t)(float)((double)c1->rot_speed_change.k[1] + (double)hp.k[1] * impulse);
        c1->rot_speed_change.k[2] = (ivp_real_t)(float)((double)c1->rot_speed_change.k[2] + (double)hp.k[2] * impulse);
        vr_add_multiple_f(&c1->speed_change, &info->surf_normal, impulse * (double)ivp_core_get_inv_mass_f(c1));
    }
}

static void frs_test_push_core_i(const ivp_impact_solver_lt_t *info, int core_index,
                                 ivp_vec3f_t *rot_change, ivp_vec3f_t *speed_change, float factor)
{
    ivp_core_t *c0 = info->contact_core[core_index];
    ivp_vec3f_t iri = ivp_core_inv_rot_inertia_f(c0);
    lvf_set_pairwise_mult(rot_change, &info->contact_cross_nomal_cs[core_index], &iri);
    if (factor < 0.0f) lvf_set_negative(rot_change, rot_change);
    lvf_set_multiple(speed_change, &info->surf_normal, (D)(ivp_core_get_inv_mass_f(c0) * factor));
}

static double frs_get_closing_speed_core_i(const ivp_impact_solver_lt_t *info, int core_index_i,
                                           const ivp_vec3f_t *rot_sped_cs, const ivp_vec3f_t *speed_ws)
{
    return (double)(lvf_dot(speed_ws, &info->surf_normal) +
                    lvf_dot(rot_sped_cs, &info->contact_cross_nomal_cs[core_index_i]));
}

double ivp_frs_calc_desired_gap_speed(double closing_speed, double gap_diff, float speedup_factor)
{
    double use_delay = (double)speedup_factor;
    if (gap_diff < 0.0) use_delay *= (double)IVP_FR_DESCENDING_SPEEDUP;
    return gap_diff * use_delay + closing_speed;
}

/* ── IVP_Contact_Point friction forces ─────────────────────────────────── */

static double cp_two_values_friction(ivp_contact_pt_t *cp, ivp_vec3f_t *real_world_friction_vec)
{
    const ivp_material_t *mtl[2];
    ivp_contact_pt_get_material_info(cp, mtl);
    ivp_impact_solver_lt_t *info = cp->tmp_contact_info;
    for (int i = 0; i < 2; i++) {
        if (!mtl[i]->second_friction_x_enabled) continue;
        ivp_vec3f_t x_direction;
        ivp_core_t *core = cp_fcore(cp, i);
        const ivp_transform_r_t *mat = &core->m_world_f_core_last_psi;
        x_direction.k[0] = (float)mat->rot.rows[0].k[0];
        x_direction.k[1] = (float)mat->rot.rows[1].k[0];
        x_direction.k[2] = (float)mat->rot.rows[2].k[0];
        IVP_IF(1) /* ivp_friction.cxx:141 */
        {
            ivp_vec3r_t base_p = mat->pos;
            ivp_vec3f_t vec_p = x_direction;
            fr_add_draw_vector(ivp_core_env(core), &base_p, &vec_p, "x_vec", 1);
            lvf_set3(&vec_p, 0.0f, -1.0f, 0.0f);
            fr_add_draw_vector(ivp_core_env(core), &base_p, &vec_p, "grav", 3);
        }
        ivp_vec3f_t x_in_surface;
        lvf_set_orthogonal_part(&x_in_surface, &x_direction, &info->surf_normal);
        double relevance_factor = (double)lvf_real_length(&x_in_surface);
        double second_friction_val = (double)mtl[i]->second_friction_factor;
        double other_factor = (double)mtl[1 - i]->friction;
        double using_friction = other_factor * second_friction_val;
        double difference_in_friction = (double)cp->real_friction_factor - using_friction;
        double effective_friction = (double)cp->real_friction_factor - difference_in_friction * relevance_factor;
        if ((double)cp->real_friction_factor < P_DOUBLE_EPS) return 0.0;
        double max_len_in_x_direction = (double)IVP_FR_MINIMAL_REAL_FRICTION_LEN * effective_friction / (double)cp->real_friction_factor;
        ivp_vec3f_t friction_vec;
        lvf_set_multiple(&friction_vec, &info->span_friction_v[0], (D)cp->span_friction_s[0]);
        lvf_add_multiple(&friction_vec, &info->span_friction_v[1], (D)cp->span_friction_s[1]);
        IVP_IF(1) /* ivp_friction.cxx:175 */
        {
            ivp_vec3r_t base_p = cp->tmp_contact_info->contact_point_ws;
            ivp_vec3f_t vec_p = friction_vec;
            lvf_mult(&vec_p, 10.0f);
            fr_add_draw_vector(ivp_core_env(core), &base_p, &vec_p, "f_vec", 1);
        }
        (void)lvf_normize(&x_in_surface);
        ivp_vec3f_t x_in_surface_float = x_in_surface;
        double len_direction_x = (double)lvf_dot(&friction_vec, &x_in_surface_float);
        double abs_len = fabs(len_direction_x);
        if (abs_len > max_len_in_x_direction) {
            ivp_vec3f_t vec_full_friction = friction_vec;
            lvf_add_multiple(&vec_full_friction, &x_in_surface, (D)-len_direction_x);
            ivp_vec3f_t vec_second_dir;
            lvf_set_multiple(&vec_second_dir, &x_in_surface, (D)max_len_in_x_direction);
            if (len_direction_x < 0.0) lvf_mult(&vec_second_dir, -1.0f);
            ivp_vec3f_t resulting_vec = vec_full_friction;
            lvf_add(&resulting_vec, &vec_second_dir);
            *real_world_friction_vec = resulting_vec;
            cp->span_friction_s[0] = (float)lvf_dot(&resulting_vec, &info->span_friction_v[0]);
            cp->span_friction_s[1] = (float)lvf_dot(&resulting_vec, &info->span_friction_v[1]);
        }
    }
    ivp_vec3f_t friction_vec;
    lvf_set_multiple(&friction_vec, &info->span_friction_v[0], (D)cp->span_friction_s[0]);
    lvf_add_multiple(&friction_vec, &info->span_friction_v[1], (D)cp->span_friction_s[1]);
    *real_world_friction_vec = friction_vec;
    return (double)lvf_real_length_plus_normize(real_world_friction_vec);
}

static double cp_get_and_set_real_friction_len(ivp_contact_pt_t *cp, ivp_vec3f_t *real_world_friction_vec)
{
    ivp_impact_solver_lt_t *info = cp->tmp_contact_info;
    lvf_set_multiple(real_world_friction_vec, &info->span_friction_v[0], (D)cp->span_friction_s[0]);
    lvf_add_multiple(real_world_friction_vec, &info->span_friction_v[1], (D)cp->span_friction_s[1]);
    double len_of_spring = (double)lvf_real_length_plus_normize(real_world_friction_vec);
    if (cp->two_friction_values) return cp_two_values_friction(cp, real_world_friction_vec);
    return len_of_spring;
}

static void cp_static_friction_single(ivp_contact_pt_t *cp, const ivp_sim_es_t *es, float desired_gap,
                                      float speedup_factor)
{
    ivp_impact_solver_lt_t *info = cp->tmp_contact_info;
    double closing_speed = ivp_impact_lt_get_closing_speed(info);
    double gap_diff = (double)desired_gap - (double)ivp_cp_get_gap_length(cp); /* retail: double */
    double a = ivp_frs_calc_desired_gap_speed(closing_speed, gap_diff, speedup_factor);
    double b = (double)info->virtual_mass;
    double impulse = a * b;
    if (impulse > 0.0) {
        cp->now_friction_pressure = (float)(impulse * es->i_delta_time);
        ivp_frs_apply_impulse(info, impulse);
    } else {
        cp->now_friction_pressure = 0.0f;
    }
}

/* IVP_Contact_Point::friction_force_local_constraint_2d_wheel lives in the
 * car-wheel physics (controllers/ivp_car_wheel_physics.c), like in the
 * reference (ivp_controller/ivp_car_wheel_physics.cxx). */
static float cp_friction_force_local_constraint_2d(ivp_contact_pt_t *cp, const ivp_sim_es_t *es)
{
    ivp_impact_solver_lt_t *info = cp->tmp_contact_info;
    /* retail RVA 0x1B080 keeps the pressure product wide until the step */
    double maximum_impulse_force = (double)cp->now_friction_pressure * (double)cp->real_friction_factor * es->delta_time;
    if (maximum_impulse_force < (double)P_FLOAT_RES) return 0.0f;
    ivp_core_t *core_a = info->contact_core[0];
    ivp_core_t *core_b = info->contact_core[1];

    if (core_a && core_a->car_wheel && !core_b &&
        (cp->next_dist_in_friction == NULL || cp->next_dist_in_friction->now_friction_pressure == 0.0f)) {
        float flEnergy = 0.0f;
        if (ivp_car_wheel_friction_force_local_constraint_2d_wheel(cp, core_a, info, es, &flEnergy)) return flEnergy;
    }

    ivp_scr_f_t tcb;   /* IVP_Solver_Core_Reaction (float-exact) */
    ivp_scr_f_init_translation_ws(&tcb, core_a, core_b, &info->contact_point_ws, &info->span_friction_v[0], &info->span_friction_v[1], NULL);
    double a = (double)cp->span_friction_s[0] * es->i_delta_time - (double)tcb.delta_velocity_ds.k[0];
    double b = (double)cp->span_friction_s[1] * es->i_delta_time - (double)tcb.delta_velocity_ds.k[1];
    /* the reaction solver fills the upper triangle; retail RVA 0x1B080
     * treats the matrix as symmetric, using tpm01 in both off-diagonal
     * slots, with the squared determinant cutoff 1e-38 */
    const double tpm00 = tcb.m[0][0];
    const double tpm01 = tcb.m[0][1];
    const double tpm10 = tpm01;
    const double tpm11 = tcb.m[1][1];
    double Dm = tpm00 * tpm11 - tpm01 * tpm10;
    if (Dm * Dm < 1e-38) return 0.0f;
    double DI = 1.0 / Dm;
    double inv0 = tpm11 * DI;
    double inv1 = -tpm01 * DI;
    double inv2 = inv1;
    double inv3 = tpm00 * DI;
    ivp_vec3f_t impulses;
    impulses.k[0] = (float)(inv0 * a + inv1 * b);
    impulses.k[1] = (float)(inv2 * a + inv3 * b);
    impulses.k[2] = 0.0f;
    double square_impulse = (double)impulses.k[1] * (double)impulses.k[1] + (double)impulses.k[0] * (double)impulses.k[0];
    if (square_impulse > maximum_impulse_force * maximum_impulse_force) {
        double isum_impulse = (double)lv_isqrt_float((float)square_impulse);
        double f = maximum_impulse_force * isum_impulse;
        impulses.k[0] = (float)((double)impulses.k[0] * f);
        impulses.k[1] = (float)((double)impulses.k[1] * f);
    }
    ivp_scr_f_exert(&tcb, core_a, core_b, &impulses, 2, true); /* exert_impulse_dim2 */
    /* retail: the energy is a double expression */
    double e = 0.5 * sqrt(((double)cp->span_friction_s[0] * (double)cp->span_friction_s[0] +
                           (double)cp->span_friction_s[1] * (double)cp->span_friction_s[1]) *
                          (es->delta_time * es->delta_time * square_impulse));
    float diff_e = (float)(e - (double)cp->old_energy_dynamic_fr);
    cp->old_energy_dynamic_fr = (float)e;
    return diff_e;
}

static void cp_friction_force_local_constraint_1d(ivp_contact_pt_t *cp, const ivp_sim_es_t *es)
{
    ivp_vec3f_t world_offset_contact;
    double spring_len = cp_get_and_set_real_friction_len(cp, &world_offset_contact);
    ivp_core_t *core[2];
    core[0] = cp_fcore(cp, 0);
    core[1] = cp_fcore(cp, 1);
    IVP_IF(0) /* ivp_friction.cxx:336 */
    {
        ivp_vec3f_t debug_way_world;
        lvf_set_multiple(&debug_way_world, &world_offset_contact, (D)spring_len);
        IVP_USE(debug_way_world);
        ivp_vec3r_t base_p = cp->tmp_contact_info->contact_point_ws;
        ivp_vec3f_t vec_p = world_offset_contact;
        lvf_mult(&vec_p, (D)(spring_len * 10000.0));
        fr_add_draw_vector(ivp_core_env(core[0]), &base_p, &vec_p, "f_vec", 3);
    }
    /* IVP_FLOAT now_friction_pressure * IVP_FLOAT real_friction_factor is a float product */
    double maximum_impulse_force = (double)(cp->now_friction_pressure * cp->real_friction_factor) * es->delta_time;
    ivp_vec3f_t world_speed[2];
    for (int i = 0; i < 2; i++) {
        ivp_core_t *c = core[i];
        if (!c->physical_unmoveable) {
            ivp_core_get_surface_speed(c, &cp->tmp_contact_info->contact_point_cs[i], &world_speed[i]);
        } else {
            lvf_zero(&world_speed[i]);
        }
    }
    ivp_vec3f_t rel_world_speed;
    lvf_subtract2(&rel_world_speed, &world_speed[0], &world_speed[1]);
    double now_speed_direction = (double)lvf_dot(&world_offset_contact, &rel_world_speed);
    ivp_vec3f_t translation_vec[2], rotation_vec[2], push_core_vec[2];
    double speed_change_on_push = 0.0;
    double sign = 1.0;
    for (int i = 0; i < 2; i++) {
        ivp_core_t *c = core[i];
        lvf_zero(&translation_vec[i]);
        lvf_zero(&rotation_vec[i]);
        if (!c->physical_unmoveable) {
            const ivp_transform_r_t *mat = &c->m_world_f_core_last_psi;
            ivp_vec3f_t push_world_vec = world_offset_contact;
            lvf_mult(&push_world_vec, (D)sign);
            lvm_vimult3_f(mat, &push_world_vec, &push_core_vec[i]);
            ivp_vec3f_t test_trans, test_rot;
            ivp_core_test_push_core(c, &cp->tmp_contact_info->contact_point_cs[i], &push_core_vec[i], &push_world_vec,
                                    &test_trans, &test_rot);
            translation_vec[i] = test_trans;
            rotation_vec[i] = test_rot;
            ivp_vec3f_t test_world;
            ivp_core_get_surface_speed_on_test(c, &cp->tmp_contact_info->contact_point_cs[i], &test_trans, &test_rot,
                                               &test_world);
            speed_change_on_push += (double)lvf_dot(&test_world, &push_world_vec);
        }
        sign *= -1.0;
    }
    double desired_speed = spring_len * es->delta_time;
    double diff_speed = desired_speed - now_speed_direction;
    double push_val;
    if (speed_change_on_push > P_DOUBLE_EPS) {
        push_val = diff_speed / speed_change_on_push;
    } else {
        return;
    }
    if (fabs(push_val) > maximum_impulse_force) push_val = maximum_impulse_force;
    for (int i = 0; i < 2; i++) {
        ivp_core_t *c = core[i];
        if (!c->physical_unmoveable) {
            vr_add_multiple_f(&c->speed, &translation_vec[i], push_val);
            lvf_add_multiple(&c->rot_speed, &rotation_vec[i], (D)push_val);
        }
    }
}

static void cp_calc_pretension(ivp_contact_pt_t *cp, float len)
{
    /* retail: double products and a double bound */
    double qlen_of_spring = (double)cp->span_friction_s[0] * (double)cp->span_friction_s[0] +
                            (double)cp->span_friction_s[1] * (double)cp->span_friction_s[1];
    if (qlen_of_spring > (double)len * (double)len + (double)P_FLOAT_RES) {
        double ilen_of_spring = lv_isqrt_float((float)qlen_of_spring);
        cp->integrated_destroyed_energy = (float)((double)cp->integrated_destroyed_energy +
                                                  (qlen_of_spring * ilen_of_spring - (double)len) *
                                                      (double)cp->now_friction_pressure * (double)cp->real_friction_factor);
        double percent_of_old = (double)len * ilen_of_spring;
        cp->span_friction_s[0] = (float)((double)cp->span_friction_s[0] * percent_of_old);
        cp->span_friction_s[1] = (float)((double)cp->span_friction_s[1] * percent_of_old);
        cp->cp_status = IVP_CPBS_NEEDS_RECHECK;
    }
}

/* ── IVP_Friction_Core_Pair ─────────────────────────────────────────────── */

static ivp_fr_core_pair_t *pair_create(ivp_fr_system_t *fs)
{
    const ivp_allocator_t *allocator = ivp_env_get_allocator(fs->l_environment);
    ivp_fr_core_pair_t *p = (ivp_fr_core_pair_t *)ivp_alloc(allocator, sizeof(*p));
    if (!p) return NULL;
    memset(p, 0, sizeof(*p));
    p->allocator = allocator;
    ivp_pvec_init_with_allocator(&p->fr_dists, allocator);
    p->next_ease_nr_psi = 1;
    p->last_impact_time_pair = -1000.0;
    p->integrated_anti_energy = 0.0f;
    p->objs[0] = NULL;
    p->objs[1] = NULL;
    return p;
}

static void pair_destroy(ivp_fr_core_pair_t *p)
{
    if (!p) return;
    ivp_pvec_clear(&p->fr_dists);
    ivp_free(p->allocator, p, sizeof(*p));
}

/* retail RVA 0x1BC20 keeps each product and the pair sum wide until both
 * double time factors are applied */
static float pair_get_sum_slide_way(ivp_fr_core_pair_t *pair, const ivp_sim_es_t *es)
{
    double sum = 0.0;
    for (int i = ivp_pvec_len(&pair->fr_dists) - 1; i >= 0; i--) {
        const ivp_contact_pt_t *cp = cp_at(&pair->fr_dists, i);
        sum += (double)cp->now_friction_pressure * (double)cp->real_friction_factor *
               (double)cp->inv_virt_mass_mindist_no_dir;
    }
    return (float)(sum * es->delta_time * es->delta_time);
}

/* IVP_Friction_Core_Pair::get_average_friction_vector */
static void pair_get_average_friction_vector(ivp_fr_core_pair_t *pair, ivp_vec3f_t *average_friction)
{
    lvf_zero(average_friction);
    int numbers = ivp_pvec_len(&pair->fr_dists);
    if (numbers > 0) {
        double factor = (double)(1.0f / (float)numbers); /* 1.0f / int: float division */
        for (int i = ivp_pvec_len(&pair->fr_dists) - 1; i >= 0; i--) {
            ivp_contact_pt_t *fr_dist = cp_at(&pair->fr_dists, i);
            ivp_vec3f_t temp_v;
            lvf_zero(&temp_v);
            ivp_impact_solver_lt_t *info = fr_dist->tmp_contact_info;
            lvf_add_multiple(&temp_v, &info->span_friction_v[0], (D)fr_dist->span_friction_s[0]);
            lvf_add_multiple(&temp_v, &info->span_friction_v[1], (D)fr_dist->span_friction_s[1]);
            double sign;
            if (cp_core(fr_dist, 0) == pair->objs[0]) {
                sign = 1.0;
            } else {
                sign = -1.0;
            }
            lvf_add_multiple(average_friction, &temp_v, (D)sign);
        }
        lvf_mult(average_friction, (D)factor);
    }
}

/* IVP_Friction_Core_Pair::debug_store_vector_before_ease */
static void pair_debug_store_vector_before_ease(ivp_fr_core_pair_t *pair)
{
    lvf_zero(&pair->span_vector_sum);
    pair_get_average_friction_vector(pair, &pair->span_vector_sum);
}

/* IVP_Friction_Core_Pair::debug_read_vector_after_ease */
static void pair_debug_read_vector_after_ease(ivp_fr_core_pair_t *pair)
{
    ivp_vec3f_t test_vec;
    lvf_zero(&test_vec);
    pair_get_average_friction_vector(pair, &test_vec);
    ivp_vec3f_t diff;
    lvf_subtract2(&diff, &test_vec, &pair->span_vector_sum);
    if ((double)lvf_real_length(&diff) > (double)0.01f) {
        ivp_debug_printf("easingerror: %f %f %f should equal %f %f %f\n", (double)test_vec.k[0], (double)test_vec.k[1],
                         (double)test_vec.k[2], (double)pair->span_vector_sum.k[0],
                         (double)pair->span_vector_sum.k[1], (double)pair->span_vector_sum.k[2]);
    }
}

/* IVP_Friction_Core_Pair::debug_printf_pair */
void ivp_fr_core_pair_debug_printf_pair(ivp_fr_core_pair_t *pair)
{
    IVP_IF(1) /* ivp_friction.cxx:1831 */
    {
        ivp_object_t *obj1 = ivp_core_object_at(pair->objs[0], 0);
        ivp_object_t *obj2 = ivp_core_object_at(pair->objs[1], 0);
        ivp_debug_printf("frpair_names %s %s ", fr_obj_name(obj1), fr_obj_name(obj2));
    }
}

static void pair_calc_friction_forces(ivp_fr_core_pair_t *pair, const ivp_sim_es_t *es)
{
    float max_local_spring_len = pair_get_sum_slide_way(pair, es);
    float sum_delta_energy = 0.0f;
    for (int k = ivp_pvec_len(&pair->fr_dists) - 1; k >= 0; k--) {
        ivp_contact_pt_t *my_fr = cp_at(&pair->fr_dists, k);
        cp_calc_pretension(my_fr, max_local_spring_len);
        IVP_ASSERT(my_fr->tmp_contact_info->coll_time_is_valid == false);
        if (!my_fr->two_friction_values) {
            sum_delta_energy += cp_friction_force_local_constraint_2d(my_fr, es);
        } else {
            cp_friction_force_local_constraint_1d(my_fr, es);
        }
    }
    if (sum_delta_energy > 0.0f) pair->integrated_anti_energy += sum_delta_energy;
}

/* IVP_Mutual_Energizer */
typedef struct mutual_energizer {
    ivp_vec3f_t trans_vec_world;
    ivp_vec3f_t rot_vec_obj[2];
    double trans_speed_potential;
    double trans_inertia[2];
    double inv_trans_inertia[2];
    double rot_speed_potential;
    double rot_inertia[2];
    double inv_rot_inertia[2];
    double rot_energy_potential;
    double trans_energy_potential;
    ivp_core_t *core[2];
    double whole_mutual_energy;
} mutual_energizer_t;

/* IVP_Mutual_Energizer (retail: no zero-mass guard, no clamp) */
static double me_calc_impulse_to_reduce_energy_level(double speed_pot, double inv_mass0, double inv_mass1, double delta_e)
{
    double inv_mass_sum = inv_mass0 + inv_mass1;
    double root = sqrt(fabs(speed_pot * speed_pot - (delta_e + delta_e) * inv_mass_sum));
    return (speed_pot - root) * (1.0 / inv_mass_sum);
}

static double me_calc_energy_potential(double speed_pot, double mass0, double mass1, double inv_mass0, double inv_mass1)
{
    double x = speed_pot / (inv_mass0 + inv_mass1);
    double speed1 = speed_pot - x * inv_mass1;
    double E1 = speed1 * speed1 * mass1;
    double speed0 = x * inv_mass0;
    double E0 = speed0 * speed0 * mass0;
    return 0.5 * (speed_pot * speed_pot * mass1 - E0 - E1);
}

static void me_get_rot_inertia(ivp_core_t *core, ivp_vec3f_t *rot_vec_obj, double *rot_inertia, double *inv_rot_inertia)
{
    ivp_vec3f_t rot_vec_abs, ri = ivp_core_rot_inertia_f(core);
    lvf_set_pairwise_mult(&rot_vec_abs, rot_vec_obj, &ri);
    *rot_inertia = (double)lvf_real_length(&rot_vec_abs);
    if (*rot_inertia < P_DOUBLE_EPS) {
        *rot_inertia = 1.0;
        *inv_rot_inertia = 1.0;
    } else {
        *inv_rot_inertia = 1.0f / (*rot_inertia);
    }
}

static void me_init(mutual_energizer_t *me, ivp_core_t *core0, ivp_core_t *core1)
{
    if (core1->physical_unmoveable || core1->pinned) {
        me->core[0] = core1;
        me->core[1] = core0;
    } else {
        me->core[1] = core1;
        me->core[0] = core0;
    }
    {
        ivp_vec3f_t s1 = ivp_core_speed_f(me->core[1]), s0 = ivp_core_speed_f(me->core[0]);
        lvf_subtract2(&me->trans_vec_world, &s1, &s0);
        me->trans_speed_potential = (double)lvf_real_length_plus_normize(&me->trans_vec_world);
    }
    {
        ivp_vec3f_t rot0_world, rot1_world;
        lvm_vmult3_f(&me->core[0]->m_world_f_core_last_psi, &me->core[0]->rot_speed, &rot0_world);
        lvm_vmult3_f(&me->core[1]->m_world_f_core_last_psi, &me->core[1]->rot_speed, &rot1_world);
        ivp_vec3f_t rot_vec_world;
        lvf_subtract2(&rot_vec_world, &rot1_world, &rot0_world);
        me->rot_speed_potential = (double)lvf_real_length_plus_normize(&rot_vec_world);
        lvm_vimult3_f(&me->core[0]->m_world_f_core_last_psi, &rot_vec_world, &me->rot_vec_obj[0]);
        lvf_mult(&rot_vec_world, -1.0f);
        lvm_vimult3_f(&me->core[1]->m_world_f_core_last_psi, &rot_vec_world, &me->rot_vec_obj[1]);
        me_get_rot_inertia(me->core[0], &me->rot_vec_obj[0], &me->rot_inertia[0], &me->inv_rot_inertia[0]);
        me_get_rot_inertia(me->core[1], &me->rot_vec_obj[1], &me->rot_inertia[1], &me->inv_rot_inertia[1]);
    }
    {
        me->trans_inertia[1] = (double)ivp_core_get_mass_f(me->core[1]);
        me->inv_trans_inertia[1] = (double)ivp_core_get_inv_mass_f(me->core[1]);
        if (me->core[0]->physical_unmoveable || me->core[0]->pinned) {
            me->trans_inertia[0] = 10000 * me->trans_inertia[1];
            me->inv_trans_inertia[0] = me->inv_trans_inertia[1] * 0.0001;
            me->rot_inertia[0] = 10000 * me->rot_inertia[1];
            me->inv_rot_inertia[0] = me->inv_rot_inertia[1] * 0.0001;
        } else {
            me->trans_inertia[0] = (double)ivp_core_get_mass_f(me->core[0]);
            me->inv_trans_inertia[0] = (double)ivp_core_get_inv_mass_f(me->core[0]);
        }
    }
}

static void me_calc_energy_potential_all(mutual_energizer_t *me)
{
    me->rot_energy_potential = me_calc_energy_potential(me->rot_speed_potential, me->rot_inertia[0], me->rot_inertia[1],
                                                        me->inv_rot_inertia[0], me->inv_rot_inertia[1]);
    me->trans_energy_potential = me_calc_energy_potential(me->trans_speed_potential, me->trans_inertia[0],
                                                          me->trans_inertia[1], me->inv_trans_inertia[0],
                                                          me->inv_trans_inertia[1]);
    me->whole_mutual_energy = me->trans_energy_potential + me->rot_energy_potential;
}

static void me_destroy_percent_energy(mutual_energizer_t *me, double percent_energy_to_destroy)
{
    double rot_impulse = me_calc_impulse_to_reduce_energy_level(me->rot_speed_potential, me->inv_rot_inertia[0],
                                                                me->inv_rot_inertia[1],
                                                                percent_energy_to_destroy * me->rot_energy_potential);
    double trans_impulse = me_calc_impulse_to_reduce_energy_level(me->trans_speed_potential, me->inv_trans_inertia[0],
                                                                  me->inv_trans_inertia[1],
                                                                  percent_energy_to_destroy * me->trans_energy_potential);
    if (!me->core[0]->physical_unmoveable && !me->core[0]->pinned) {
        vr_add_multiple_f(&me->core[0]->speed_change, &me->trans_vec_world, me->inv_trans_inertia[0] * trans_impulse);
        vr_add_multiple_f(&me->core[0]->rot_speed_change, &me->rot_vec_obj[0], me->inv_rot_inertia[0] * rot_impulse);
    }
    lvf_mult(&me->trans_vec_world, -1.0f);
    vr_add_multiple_f(&me->core[1]->speed_change, &me->trans_vec_world, me->inv_trans_inertia[1] * trans_impulse);
    vr_add_multiple_f(&me->core[1]->rot_speed_change, &me->rot_vec_obj[1], me->inv_rot_inertia[1] * rot_impulse);
}

static double pair_destroy_mutual_energy(ivp_fr_core_pair_t *pair, double d_e)
{
    mutual_energizer_t me;
    me_init(&me, pair->objs[0], pair->objs[1]);
    me_calc_energy_potential_all(&me);
    double max_energy_to_destroy = (double)IVP_FR_MAX_ENERGY_DESTROY * me.whole_mutual_energy;
    if (d_e > max_energy_to_destroy) d_e = max_energy_to_destroy;
    if (me.whole_mutual_energy < P_DOUBLE_EPS) return 0.0;
    double percent_energy_to_destroy = d_e / me.whole_mutual_energy;
    me_destroy_percent_energy(&me, percent_energy_to_destroy);
    return d_e;
}

/* retail RVA 0x1CCD0 stores a float cache but reduces by the unrounded
 * product; zero energy skips the reduction, car wheels do not */
static void pair_remove_energy_gained_by_real_friction(ivp_fr_core_pair_t *pair)
{
    const double damp = ivp_env_get_integrated_energy_damp(ivp_core_env(pair->objs[0]));
    const double energy_to_remove = (double)pair->integrated_anti_energy * damp;
    pair->integrated_anti_energy = (float)energy_to_remove;
    if (!(energy_to_remove > 0.0)) return;
    double amount_energy_destr = pair_destroy_mutual_energy(pair, energy_to_remove);
    /* statistic counter, every build (ivp_friction.cxx:1598) */
    ivp_env_get_statistic_manager(ivp_core_env(pair->objs[0]))->sum_energy_destr += amount_energy_destr;
    pair->integrated_anti_energy = (float)((double)pair->integrated_anti_energy - amount_energy_destr);
}

int ivp_fr_core_pair_check_all_fr_mindists_to_be_valid(ivp_fr_core_pair_t *pair, ivp_fr_system_t *my_fs)
{
    int total_number_remaining = ivp_pvec_len(&pair->fr_dists);
    for (int k = ivp_pvec_len(&pair->fr_dists) - 1; k >= 0; k--) {
        ivp_contact_pt_t *my_dist = cp_at(&pair->fr_dists, k);
        ivp_contact_pt_recalc_friction_s_vals(my_dist);
        ivp_impact_solver_lt_t *info = my_dist->tmp_contact_info;
        ivp_contact_pt_read_materials_for_contact_situation(my_dist, info);
        if (info->friction_is_broken) {
            total_number_remaining--;
            ivp_fr_system_delete_friction_distance(my_fs, my_dist);
        }
    }
    return total_number_remaining;
}

/* ── easing (IVP_Friction_Solver::ease_*) ──────────────────────────────── */

static void cp_ease_the_friction_force(ivp_contact_pt_t *cp, const ivp_vec3f_t *ease_diff_vec)
{
    ivp_impact_solver_lt_t *info = cp->tmp_contact_info;
    cp->span_friction_s[0] = (float)((double)cp->span_friction_s[0] + (double)lvf_dot(ease_diff_vec, &info->span_friction_v[0]));
    cp->span_friction_s[1] = (float)((double)cp->span_friction_s[1] + (double)lvf_dot(ease_diff_vec, &info->span_friction_v[1]));
}

static void frs_ease_two_mindists(ivp_contact_pt_t *dist0, ivp_contact_pt_t *dist1, ivp_vec3f_t *ease_diff_vec0,
                                  ivp_vec3f_t *ease_diff_vec1, double ease_factor)
{
    ivp_core_t *rev_core = cp_core(dist0, 0);
    float reverse_factor;
    if (rev_core != cp_core(dist1, 0)) {
        reverse_factor = -1.0f;
    } else {
        reverse_factor = 1.0f;
    }
    ivp_impact_solver_lt_t *info0 = dist0->tmp_contact_info;
    ivp_impact_solver_lt_t *info1 = dist1->tmp_contact_info;
    ivp_vec3f_t world_vec0, world_vec1;
    lvf_set_multiple(&world_vec0, &info0->span_friction_v[0], (D)dist0->span_friction_s[0]);
    lvf_add_multiple(&world_vec0, &info0->span_friction_v[1], (D)dist0->span_friction_s[1]);
    lvf_set_multiple(&world_vec1, &info1->span_friction_v[0], (D)dist1->span_friction_s[0]);
    lvf_add_multiple(&world_vec1, &info1->span_friction_v[1], (D)dist1->span_friction_s[1]);
    ivp_vec3f_t world_connection;
    lvf_subtract2_p(&world_connection, &info0->contact_point_ws, &info1->contact_point_ws);
    IVP_IF(0) /* ivp_friction.cxx:645 */
    {
        fr_add_draw_vector(ivp_core_env(rev_core), &info1->contact_point_ws, &world_connection, "ease", 3);
    }
    (void)lvf_fast_normize(&world_connection);
    ivp_vec3f_t ease_part0, ease_part1;
    double part_val0 = (double)lvf_dot(&world_connection, &world_vec0);
    double part_val1 = (double)lvf_dot(&world_connection, &world_vec1);
    lvf_set_multiple(&ease_part0, &world_connection, (D)part_val0);
    lvf_set_multiple(&ease_part1, &world_connection, (D)(part_val1 * (double)reverse_factor));
    ivp_vec3f_t optimal_vec;
    lvf_add2(&optimal_vec, &ease_part0, &ease_part1);
    lvf_mult(&optimal_vec, 0.5f);
    ivp_vec3f_t diff_vec0, diff_vec1;
    lvf_subtract_and_mult(&diff_vec0, &optimal_vec, &ease_part0, (D)ease_factor);
    lvf_subtract_and_mult(&diff_vec1, &optimal_vec, &ease_part1, (D)(ease_factor * (double)reverse_factor));
    lvf_add(ease_diff_vec0, &diff_vec0);
    lvf_add(ease_diff_vec1, &diff_vec1);
}

/* IVP_Friction_Solver::ease_test_two_mindists: does nothing in the reference
 * (its body is "#if 0"); only called from IVP_IF(0) blocks */
static void frs_ease_test_two_mindists(ivp_contact_pt_t *dist0, ivp_contact_pt_t *dist1,
                                       const ivp_vec3f_t *world_surf_normal)
{
    IVP_USE(dist1);
    IVP_USE(dist0);
    IVP_USE(world_surf_normal);
}

/* the IVP_IF(0) "ease_testt" blocks of IVP_Friction_Solver::ease_friction_pair */
static void frs_debug_ease_testt(ivp_contact_pt_t **all_my_dists, int total_n)
{
    ivp_debug_printf("ease_testt\n");
    for (int i = 0; i < total_n - 1; i++) {
        ivp_contact_pt_t *fr_dist = all_my_dists[i];
        if (((double)fr_dist->now_friction_pressure > P_DOUBLE_EPS) || 1) {
            for (int j = i + 1; j < total_n; j++) {
                ivp_contact_pt_t *fr_dist2 = all_my_dists[j];
                if (((double)fr_dist2->now_friction_pressure > P_DOUBLE_EPS) || 1) {
                    double dir_conform = (double)lvf_dot(&fr_dist->tmp_contact_info->surf_normal,
                                                         &fr_dist2->tmp_contact_info->surf_normal);
                    dir_conform = fabs(dir_conform) - 1.0;
                    dir_conform = fabs(dir_conform);
                    if ((dir_conform < (double)1E-3f) || 0) {
                        frs_ease_test_two_mindists(fr_dist2, fr_dist, &fr_dist->tmp_contact_info->surf_normal);
                    }
                }
            }
        }
    }
}

/* IVP_Friction_Solver::ease_friction_pair.  The reference takes the pointer
 * array all_my_dists from the environment's short-term arena (outside of any
 * transaction: the arena pointer just advances until the next friction
 * transaction ends) and the force vectors from alloca.  The arena bytes are
 * reserved the same way here; they are left holding zeros instead of the
 * reference's (address dependent) pointer bits. */
static void frs_ease_friction_pair(ivp_fr_core_pair_t *my_pair,
                                   ivp_arena_t *arena,
                                   struct ivp_environment *env)
{
    int total_n = ivp_pvec_len(&my_pair->fr_dists);
    size_t arena_bytes = (size_t)(total_n > 0 ? total_n : 0) * IVP_FR_REF_POINTER_SIZE;
    void *arena_slot = ivp_arena_alloc(arena, arena_bytes);
    if (arena_slot) memset(arena_slot, 0, arena_bytes);
    if (total_n <= 0) return;
    if ((size_t)total_n > SIZE_MAX / sizeof(ivp_contact_pt_t *) ||
        (size_t)total_n > SIZE_MAX / sizeof(ivp_vec3f_t)) return;
    ivp_contact_pt_t **all_my_dists = (ivp_contact_pt_t **)ivp_env_sim_mem_alloc(
        env, (size_t)total_n * sizeof(ivp_contact_pt_t *));
    ivp_vec3f_t *ease_diff = (ivp_vec3f_t *)ivp_env_sim_mem_alloc(
        env, (size_t)total_n * sizeof(ivp_vec3f_t));
    if (!all_my_dists || !ease_diff) return;
    double easing_factor = 1.0f / ((double)total_n + P_DOUBLE_EPS);
    for (int i = ivp_pvec_len(&my_pair->fr_dists) - 1; i >= 0; i--) {
        all_my_dists[i] = cp_at(&my_pair->fr_dists, i);
        lvf_zero(&ease_diff[i]);
    }
    IVP_IF(0) /* ivp_friction.cxx:699 */
    {
        frs_debug_ease_testt(all_my_dists, total_n);
    }
    IVP_IF(1) /* ivp_friction.cxx:736 */
    {
        pair_debug_store_vector_before_ease(my_pair);
    }
    for (int i = 0; i < total_n - 1; i++) {
        ivp_contact_pt_t *fr_dist = all_my_dists[i];
        for (int j = i + 1; j < total_n; j++) {
            ivp_contact_pt_t *fr_dist2 = all_my_dists[j];
            double dir_conform = (double)lvf_dot(&fr_dist->tmp_contact_info->surf_normal,
                                                 &fr_dist2->tmp_contact_info->surf_normal);
            dir_conform = fabs(dir_conform) - 1.0;
            dir_conform = fabs(dir_conform);
            if (dir_conform < (double)1E-3f) {
                frs_ease_two_mindists(fr_dist2, fr_dist, &ease_diff[j], &ease_diff[i], easing_factor);
            }
        }
    }
    for (int i = 0; i < total_n; i++) {
        cp_ease_the_friction_force(all_my_dists[i], &ease_diff[i]);
    }
    IVP_IF(1) /* ivp_friction.cxx:787 */
    {
        pair_debug_read_vector_after_ease(my_pair);
    }
    IVP_IF(0) /* ivp_friction.cxx:792 */
    {
        frs_debug_ease_testt(all_my_dists, total_n);
    }
}

/* ── IVP_Friction_System bookkeeping (ivp_physic_private.cxx) ──────────── */

static int  fs_dyn_prio(ivp_sim_ctrl_t *c) { (void)c; return IVP_CP_DYNAMIC_FRICTION; }
static int  fs_static_prio(ivp_sim_ctrl_t *c) { (void)c; return IVP_CP_STATIC_FRICTION; }
static int  fs_energy_prio(ivp_sim_ctrl_t *c) { (void)c; return IVP_CP_ENERGY_FRICTION; }
static void fs_dyn_do_simulation(ivp_sim_ctrl_t *c, ivp_sim_es_t *es, ivp_pvec_t *cores);
static void fs_static_do_simulation(ivp_sim_ctrl_t *c, ivp_sim_es_t *es, ivp_pvec_t *cores);
static void fs_energy_do_simulation(ivp_sim_ctrl_t *c, ivp_sim_es_t *es, ivp_pvec_t *cores);
static ivp_pvec_t *fs_dyn_assoc(ivp_sim_ctrl_t *c) { return &FS_FROM(c, dyn_ctrl)->moveable_cores_of_friction_system; }
static ivp_pvec_t *fs_indep_assoc(ivp_sim_ctrl_t *c) { (void)c; return NULL; }
static void fs_noop_core_deleted(ivp_sim_ctrl_t *c, ivp_core_t *core) { (void)c; (void)core; }
static void fs_static_core_deleted(ivp_sim_ctrl_t *c, ivp_core_t *core);
static void fs_dyn_reset_time(ivp_sim_ctrl_t *c, double offset) { ivp_fr_system_reset_time(FS_FROM(c, dyn_ctrl), offset); }

static const ivp_sim_ctrl_vt_t fs_dyn_vt = {
    fs_dyn_prio, fs_dyn_do_simulation, fs_dyn_assoc, fs_noop_core_deleted, fs_dyn_reset_time
};
static const ivp_sim_ctrl_vt_t fs_static_vt = {
    fs_static_prio, fs_static_do_simulation, fs_indep_assoc, fs_static_core_deleted, NULL
};
static const ivp_sim_ctrl_vt_t fs_energy_vt = {
    fs_energy_prio, fs_energy_do_simulation, fs_indep_assoc, fs_noop_core_deleted, NULL
};

ivp_fr_system_t *ivp_fr_system_create(struct ivp_environment *env)
{
    const ivp_allocator_t *allocator = ivp_env_get_allocator(env);
    ivp_fr_system_t *fs = (ivp_fr_system_t *)ivp_alloc(allocator, sizeof(*fs));
    if (!fs) return NULL;
    memset(fs, 0, sizeof(*fs));
    IVP_IF(fr_dm(env)->check_fs) /* ivp_physic_private.cxx:659 */
    {
        fprintf(fr_dm(env)->out_deb_file, "create_fs %f %p\n", ivp_env_get_current_time(env), (void *)fs);
    }
    fs->dyn_ctrl.vt = &fs_dyn_vt;
    fs->static_ctrl.vt = &fs_static_vt;
    fs->energy_ctrl.vt = &fs_energy_vt;
    fs->l_environment = env;
    fs->union_find_necessary = false;
    fs->first_friction_dist = NULL;
    fs->friction_obj_number = 0;
    fs->friction_dist_number = 0;
    fs->complex_not_necessary_number = 0;
    ivp_pvec_init_with_allocator(&fs->cores_of_friction_system, allocator);
    ivp_pvec_init_with_allocator(&fs->moveable_cores_of_friction_system, allocator);
    ivp_pvec_init_with_allocator(&fs->fr_pairs_of_objs, allocator);
    return fs;
}

void ivp_fr_system_destroy(ivp_fr_system_t *fs)
{
    if (!fs) return;
    IVP_IF(fr_dm(fs->l_environment)->check_fs) /* ivp_physic_private.cxx:679 */
    {
        fprintf(fr_dm(fs->l_environment)->out_deb_file, "delete_fs %f %p\n",
                ivp_env_get_current_time(fs->l_environment), (void *)fs);
    }
    ivp_pvec_clear(&fs->cores_of_friction_system);
    ivp_pvec_clear(&fs->moveable_cores_of_friction_system);
    ivp_pvec_clear(&fs->fr_pairs_of_objs);
    ivp_free(ivp_env_get_allocator(fs->l_environment), fs, sizeof(*fs));
}

static void fs_recalc_all_contact_points(ivp_fr_system_t *fs)
{
    for (int i = ivp_pvec_len(&fs->fr_pairs_of_objs) - 1; i >= 0; i--) {
        ivp_fr_core_pair_t *my_pair = pair_at(&fs->fr_pairs_of_objs, i);
        float energy_diff_sum = 0.0f;
        for (int j = ivp_pvec_len(&my_pair->fr_dists) - 1; j >= 0; j--) {
            ivp_contact_pt_t *fr_mindist = cp_at(&my_pair->fr_dists, j);
            float old_gap_len = ivp_cp_get_gap_length(fr_mindist);
            ivp_contact_pt_recalc_friction_s_vals(fr_mindist);
            float gap_diff = old_gap_len - ivp_cp_get_gap_length(fr_mindist);
            energy_diff_sum += fr_mindist->now_friction_pressure * gap_diff;
        }
        if (energy_diff_sum > 0.0f) my_pair->integrated_anti_energy += energy_diff_sum;
    }
}

static bool fs_add_fr_pair(ivp_fr_system_t *fs, ivp_fr_core_pair_t *pair)
{
    if (ivp_pvec_add(&fs->fr_pairs_of_objs, pair) < 0) return false;
    ivp_env_fire_friction_pair_created(ivp_core_env(pair->objs[0]), pair);
    return true;
}

static void fs_del_fr_pair(ivp_fr_system_t *fs, ivp_fr_core_pair_t *pair)
{
    ivp_env_fire_friction_pair_deleted(ivp_core_env(pair->objs[0]), pair);
    ivp_pvec_remove(&fs->fr_pairs_of_objs, pair);
}

ivp_fr_core_pair_t *ivp_fr_system_get_pair_info_for_objs(ivp_fr_system_t *fs, ivp_core_t *core0, ivp_core_t *core1)
{
    for (int i = ivp_pvec_len(&fs->fr_pairs_of_objs) - 1; i >= 0; i--) {
        ivp_fr_core_pair_t *my_pair = pair_at(&fs->fr_pairs_of_objs, i);
        if ((my_pair->objs[0] == core0) || (my_pair->objs[1] == core0)) {
            if ((my_pair->objs[0] == core1) || (my_pair->objs[1] == core1)) return my_pair;
        }
    }
    return NULL;
}

ivp_fr_core_pair_t *ivp_fr_system_find_pair_of_cores(ivp_fr_system_t *fs, ivp_core_t *core0, ivp_core_t *core1)
{
    for (int m = ivp_pvec_len(&fs->fr_pairs_of_objs) - 1; m >= 0; m--) {
        ivp_fr_core_pair_t *my_pair = pair_at(&fs->fr_pairs_of_objs, m);
        if (((my_pair->objs[0] == core0) && (my_pair->objs[1] == core1)) ||
            ((my_pair->objs[0] == core1) && (my_pair->objs[1] == core0))) {
            return my_pair;
        }
    }
    return NULL;
}

bool ivp_fr_system_dist_removed_update_pair_info(ivp_fr_system_t *fs, ivp_contact_pt_t *old_dist)
{
    ivp_core_t *core0 = cp_core(old_dist, 0);
    ivp_core_t *core1 = cp_core(old_dist, 1);
    ivp_fr_core_pair_t *my_pair_info = ivp_fr_system_get_pair_info_for_objs(fs, core0, core1);
    if (my_pair_info) {
        ivp_pvec_remove(&my_pair_info->fr_dists, old_dist);
        if (ivp_pvec_len(&my_pair_info->fr_dists) == 0) {
            fs_del_fr_pair(fs, my_pair_info);
            pair_destroy(my_pair_info);
            return true;
        }
        return false;
    }
    IVP_CORE();   /* ivp_physic_private.cxx:735: every build */
    return false;
}

void ivp_fr_system_remove_dist_from_system(ivp_fr_system_t *fs, ivp_contact_pt_t *old_dist)
{
    IVP_IF(fr_dm(fs->l_environment)->check_fs) /* ivp_physic_private.cxx:741 */
    {
        fprintf(fr_dm(fs->l_environment)->out_deb_file, "rem_dist_from_fs %f %p from %p\n",
                ivp_env_get_current_time(fs->l_environment), (void *)old_dist, (void *)fs);
    }
    ivp_contact_pt_t *previous = old_dist->prev_dist_in_friction;
    ivp_contact_pt_t *following = old_dist->next_dist_in_friction;
    if (following) following->prev_dist_in_friction = previous;
    if (previous) {
        previous->next_dist_in_friction = following;
    } else {
        fs->first_friction_dist = following;
    }
    fs->friction_dist_number--;
}

bool ivp_fr_system_dist_added_update_pair_info(ivp_fr_system_t *fs, ivp_contact_pt_t *new_dist)
{
    ivp_core_t *core0 = cp_core(new_dist, 0);
    ivp_core_t *core1 = cp_core(new_dist, 1);
    ivp_fr_core_pair_t *my_pair_info = ivp_fr_system_get_pair_info_for_objs(fs, core0, core1);
    if (my_pair_info) {
        return ivp_pvec_add(&my_pair_info->fr_dists, new_dist) >= 0;
    } else {
        my_pair_info = pair_create(fs);
        if (!my_pair_info) return false;
        my_pair_info->objs[0] = core0;
        my_pair_info->objs[1] = core1;
        if (!ivp_pvec_reserve(&my_pair_info->fr_dists, 1) ||
            !ivp_pvec_ensure(&fs->fr_pairs_of_objs)) {
            pair_destroy(my_pair_info);
            return false;
        }
        int added = ivp_pvec_add(&my_pair_info->fr_dists, new_dist);
        IVP_ASSERT(added >= 0);
        IVP_USE(added);
        if (!fs_add_fr_pair(fs, my_pair_info)) {
            pair_destroy(my_pair_info);
            return false;
        }
        return true;
    }
}

void ivp_fr_system_add_dist_to_system(ivp_fr_system_t *fs, ivp_contact_pt_t *new_dist)
{
    IVP_IF(fr_dm(fs->l_environment)->check_fs) /* ivp_physic_private.cxx:792 */
    {
        fprintf(fr_dm(fs->l_environment)->out_deb_file, "add_dist_to_fs %f %p to %p\n",
                ivp_env_get_current_time(fs->l_environment), (void *)new_dist, (void *)fs);
    }
    new_dist->l_friction_system = fs;
    ivp_contact_pt_t *first = fs->first_friction_dist;
    new_dist->prev_dist_in_friction = NULL;
    new_dist->next_dist_in_friction = first;
    fs->first_friction_dist = new_dist;
    if (first) first->prev_dist_in_friction = new_dist;
    IVP_IF(fr_dm(fs->l_environment)->debug_mindist) /* ivp_physic_private.cxx:811 */
    {
        ivp_debug_printf("added_dist %d\n", (int)fs->friction_dist_number);
    }
    fs->friction_dist_number++;
}

bool ivp_fr_system_add_core_to_system(ivp_fr_system_t *fs, ivp_core_t *new_obj)
{
    IVP_IF(fr_dm(fs->l_environment)->check_fs) /* ivp_physic_private.cxx:822 */
    {
        fprintf(fr_dm(fs->l_environment)->out_deb_file, "add_core_to_fs %f %p to %p\n",
                ivp_env_get_current_time(fs->l_environment), (void *)new_obj, (void *)fs);
    }
    if (!ivp_pvec_ensure(&fs->cores_of_friction_system)) {
        return false;
    }
    if (!new_obj->physical_unmoveable) {
        if (!ivp_pvec_ensure(&fs->moveable_cores_of_friction_system)) {
            return false;
        }
        if (!ivp_core_add_core_controller(new_obj, &fs->static_ctrl)) return false;
        if (!ivp_core_add_core_controller(new_obj, &fs->dyn_ctrl)) {
            ivp_core_rem_core_controller(new_obj, &fs->static_ctrl);
            return false;
        }
        if (!ivp_core_add_core_controller(new_obj, &fs->energy_ctrl)) {
            ivp_core_rem_core_controller(new_obj, &fs->dyn_ctrl);
            ivp_core_rem_core_controller(new_obj, &fs->static_ctrl);
            return false;
        }
    }
    int core_added = ivp_pvec_add(&fs->cores_of_friction_system, new_obj);
    IVP_ASSERT(core_added >= 0);
    IVP_USE(core_added);
    if (!new_obj->physical_unmoveable) {
        int moveable_added = ivp_pvec_add(&fs->moveable_cores_of_friction_system,
                                          new_obj);
        IVP_ASSERT(moveable_added >= 0);
        IVP_USE(moveable_added);
    }
    fs->friction_obj_number++;
    return true;
}

void ivp_fr_system_remove_core_from_system(ivp_fr_system_t *fs, ivp_core_t *old_obj)
{
    IVP_IF(fr_dm(fs->l_environment)->check_fs) /* ivp_physic_private.cxx:844 */
    {
        fprintf(fr_dm(fs->l_environment)->out_deb_file, "remove_core_from_fs %f %p from %p\n",
                ivp_env_get_current_time(fs->l_environment), (void *)old_obj, (void *)fs);
    }
    if (!old_obj->physical_unmoveable) {
        ivp_pvec_remove(&fs->moveable_cores_of_friction_system, old_obj);
        ivp_core_rem_core_controller(old_obj, &fs->energy_ctrl);
        ivp_core_rem_core_controller(old_obj, &fs->dyn_ctrl);
        ivp_core_rem_core_controller(old_obj, &fs->static_ctrl);
    }
    ivp_pvec_remove(&fs->cores_of_friction_system, old_obj);
    fs->friction_obj_number--;
}

void ivp_fr_system_delete_friction_distance(ivp_fr_system_t *fs, ivp_contact_pt_t *old_dist)
{
    ivp_core_t *core0 = cp_core(old_dist, 0);
    ivp_core_t *core1 = cp_core(old_dist, 1);
    ivp_core_reset_freeze_check_values(core0);
    ivp_core_reset_freeze_check_values(core1);
    ivp_fr_system_remove_dist_from_system(fs, old_dist);
    if (ivp_fr_system_dist_removed_update_pair_info(fs, old_dist)) {
        fs->union_find_necessary = true;
    }
    ivp_fr_info_for_core_t *fr_info0 = ivp_core_get_friction_info(core0, fs);
    ivp_fr_info_for_core_t *fr_info1 = ivp_core_get_friction_info(core1, fs);
    ivp_fr_info_delete_friction_dist(fr_info0, old_dist);
    if (ivp_pvec_len(&fr_info0->friction_springs) == 0) {
        ivp_core_delete_friction_info(core0, fr_info0);
        ivp_fr_system_remove_core_from_system(fs, core0);
        if (core0->sim_unit) ivp_sim_unit_request_union_find(core0->sim_unit);
    }
    ivp_fr_info_delete_friction_dist(fr_info1, old_dist);
    if (ivp_pvec_len(&fr_info1->friction_springs) == 0) {
        ivp_core_delete_friction_info(core1, fr_info1);
        ivp_fr_system_remove_core_from_system(fs, core1);
        if (core1->sim_unit) ivp_sim_unit_request_union_find(core1->sim_unit);
    }
    ivp_contact_pt_destroy(old_dist);
}

typedef struct fs_fusion_pair_plan {
    ivp_fr_core_pair_t *source;
    ivp_fr_core_pair_t *destination;
    ivp_core_t *cores[2];
    bool destination_is_new;
    bool destination_is_published;
} fs_fusion_pair_plan_t;

static fs_fusion_pair_plan_t *fs_fusion_find_pair_plan(
    fs_fusion_pair_plan_t *plans, int plan_count,
    ivp_core_t *core0, ivp_core_t *core1)
{
    for (int i = 0; i < plan_count; i++) {
        if ((plans[i].cores[0] == core0 && plans[i].cores[1] == core1) ||
            (plans[i].cores[0] == core1 && plans[i].cores[1] == core0)) {
            return &plans[i];
        }
    }
    return NULL;
}

bool ivp_fr_system_fusion_friction_systems(ivp_fr_system_t *first_sys, ivp_fr_system_t *second_sys)
{
    if (!first_sys || !second_sys || first_sys == second_sys) return false;
    const ivp_allocator_t *allocator =
        ivp_env_get_allocator(first_sys->l_environment);
    const int plan_count = ivp_pvec_len(&second_sys->fr_pairs_of_objs);
    if (plan_count < 0 ||
        (size_t)plan_count > SIZE_MAX / sizeof(fs_fusion_pair_plan_t)) {
        return false;
    }
    fs_fusion_pair_plan_t *plans = NULL;
    if (plan_count > 0) {
        plans = (fs_fusion_pair_plan_t *)ivp_alloc(
            allocator, (size_t)plan_count * sizeof(*plans));
        if (!plans) return false;
        memset(plans, 0, (size_t)plan_count * sizeof(*plans));
    }

    int new_pair_count = 0;
    for (int i = 0; i < plan_count; i++) {
        ivp_fr_core_pair_t *source = pair_at(&second_sys->fr_pairs_of_objs, i);
        fs_fusion_pair_plan_t *plan = &plans[i];
        plan->source = source;
        plan->cores[0] = source->objs[0];
        plan->cores[1] = source->objs[1];
        plan->destination = ivp_fr_system_find_pair_of_cores(
            first_sys, source->objs[0], source->objs[1]);
        if (!plan->destination) {
            plan->destination = pair_create(first_sys);
            if (!plan->destination) goto prepare_failed;
            plan->destination->objs[0] = source->objs[0];
            plan->destination->objs[1] = source->objs[1];
            plan->destination_is_new = true;
            new_pair_count++;
        }
        int destination_count = ivp_pvec_len(&plan->destination->fr_dists);
        int source_count = ivp_pvec_len(&source->fr_dists);
        if (source_count > INT_MAX - destination_count ||
            !ivp_pvec_reserve(&plan->destination->fr_dists,
                              destination_count + source_count)) {
            goto prepare_failed;
        }
    }

    if (new_pair_count > INT_MAX - ivp_pvec_len(&first_sys->fr_pairs_of_objs) ||
        !ivp_pvec_reserve(&first_sys->fr_pairs_of_objs,
                          ivp_pvec_len(&first_sys->fr_pairs_of_objs) +
                              new_pair_count)) {
        goto prepare_failed;
    }
    int second_core_count = ivp_pvec_len(&second_sys->cores_of_friction_system);
    if (second_core_count > INT_MAX - ivp_pvec_len(&first_sys->cores_of_friction_system) ||
        !ivp_pvec_reserve(&first_sys->cores_of_friction_system,
                          ivp_pvec_len(&first_sys->cores_of_friction_system) +
                              second_core_count) ||
        second_core_count > INT_MAX - ivp_pvec_len(&first_sys->moveable_cores_of_friction_system) ||
        !ivp_pvec_reserve(&first_sys->moveable_cores_of_friction_system,
                          ivp_pvec_len(&first_sys->moveable_cores_of_friction_system) +
                              second_core_count)) {
        goto prepare_failed;
    }
    for (int i = 0; i < second_core_count; i++) {
        ivp_core_t *core = core_at(&second_sys->cores_of_friction_system, i);
        ivp_fr_info_for_core_t *destination_info =
            ivp_core_get_friction_info(core, first_sys);
        if (!destination_info) continue;
        ivp_fr_info_for_core_t *source_info =
            ivp_core_get_friction_info(core, second_sys);
        int destination_count = ivp_pvec_len(&destination_info->friction_springs);
        int source_count = ivp_pvec_len(&source_info->friction_springs);
        if (source_count > INT_MAX - destination_count ||
            !ivp_pvec_reserve(&destination_info->friction_springs,
                              destination_count + source_count)) {
            goto prepare_failed;
        }
    }
    int first_core_count_before = ivp_pvec_len(&first_sys->cores_of_friction_system);
    IVP_IF(1) /* ivp_friction.cxx:1077 */
    {
        struct ivp_environment *env = first_sys->l_environment;
        IVP_IF(fr_dm(env)->check_fs)
        {
            ivp_debug_printf("fusion_fs %f %p %p  cores ", ivp_env_get_current_time(env), (void *)first_sys,
                             (void *)second_sys);
            for (int k = ivp_pvec_len(&first_sys->cores_of_friction_system) - 1; k >= 0; k--) {
                ivp_core_t *my_core = core_at(&first_sys->cores_of_friction_system, k);
                ivp_debug_printf("%p ", (void *)my_core);
            }
            ivp_debug_printf(" ");
            for (int l = ivp_pvec_len(&second_sys->cores_of_friction_system) - 1; l >= 0; l--) {
                ivp_core_t *my_core = core_at(&second_sys->cores_of_friction_system, l);
                ivp_debug_printf("%p ", (void *)my_core);
            }
            ivp_debug_printf("\n");
        }
    }
    /* Register every core which only belongs to the source system before
     * moving a contact.  Controller registration is the last operation in
     * this transaction that may allocate. */
    for (int n = second_core_count - 1; n >= 0; n--) {
        ivp_core_t *core = core_at(&second_sys->cores_of_friction_system, n);
        if (ivp_core_get_friction_info(core, first_sys)) continue;
        if (!ivp_fr_system_add_core_to_system(first_sys, core)) {
            while (ivp_pvec_len(&first_sys->cores_of_friction_system) >
                   first_core_count_before) {
                ivp_core_t *added_core = core_at(
                    &first_sys->cores_of_friction_system,
                    ivp_pvec_len(&first_sys->cores_of_friction_system) - 1);
                ivp_fr_system_remove_core_from_system(first_sys, added_core);
            }
            goto prepare_failed;
        }
    }
    ivp_contact_pt_t *my_dist = second_sys->first_friction_dist;
    while (my_dist) {
        ivp_contact_pt_t *next_dist = my_dist->next_dist_in_friction;
        ivp_core_t *core0 = cp_core(my_dist, 0);
        ivp_core_t *core1 = cp_core(my_dist, 1);
        fs_fusion_pair_plan_t *plan = fs_fusion_find_pair_plan(
            plans, plan_count, core0, core1);
        IVP_ASSERT(plan);
        ivp_fr_system_remove_dist_from_system(second_sys, my_dist);
        (void)ivp_fr_system_dist_removed_update_pair_info(second_sys, my_dist);
        ivp_fr_system_add_dist_to_system(first_sys, my_dist);
        int added = ivp_pvec_add(&plan->destination->fr_dists, my_dist);
        IVP_ASSERT(added >= 0);
        IVP_USE(added);
        if (plan->destination_is_new && !plan->destination_is_published) {
            /* the reference re-adds the contacts one by one
             * (dist_added_update_pair_info): a new pair takes the core
             * order of the first contact moved into it */
            plan->destination->objs[0] = core0;
            plan->destination->objs[1] = core1;
            bool published = fs_add_fr_pair(first_sys, plan->destination);
            IVP_ASSERT(published);
            IVP_USE(published);
            plan->destination_is_published = true;
        }
        my_dist = next_dist;
    }
    for (int n = ivp_pvec_len(&second_sys->cores_of_friction_system) - 1; n >= 0; n--) {
        ivp_core_t *obj1 = core_at(&second_sys->cores_of_friction_system, n);
        ivp_fr_info_for_core_t *fr_info0, *fr_info1;
        fr_info1 = ivp_core_get_friction_info(obj1, second_sys);
        if ((fr_info0 = ivp_core_get_friction_info(obj1, first_sys)) != NULL) {
            for (int m = ivp_pvec_len(&fr_info1->friction_springs) - 1; m >= 0; m--) {
                ivp_contact_pt_t *d = cp_at(&fr_info1->friction_springs, m);
                bool inserted = ivp_fr_info_insert_friction_dist(fr_info0, d);
                IVP_ASSERT(inserted);
                IVP_USE(inserted);
                ivp_fr_info_delete_friction_dist(fr_info1, d);
            }
            ivp_core_delete_friction_info(obj1, fr_info1);
        } else {
            ivp_core_unlink_friction_info(obj1, fr_info1);
            fr_info1->l_friction_system = first_sys;
            bool info_added = ivp_core_add_friction_info(obj1, fr_info1);
            IVP_ASSERT(info_added);
            IVP_USE(info_added);
            ivp_fr_system_remove_core_from_system(second_sys, obj1);
        }
    }
    ivp_fr_system_destroy(second_sys);
    IVP_IF(1) /* ivp_friction.cxx:1154 */
    {
        ivp_fr_system_test_hole_fr_system_data(first_sys);
    }
    if (plans) {
        ivp_free(allocator, plans, (size_t)plan_count * sizeof(*plans));
    }
    return true;

prepare_failed:
    for (int i = 0; i < plan_count; i++) {
        if (plans[i].destination_is_new && plans[i].destination) {
            pair_destroy(plans[i].destination);
        }
    }
    if (plans) {
        ivp_free(allocator, plans, (size_t)plan_count * sizeof(*plans));
    }
    return false;
}

/* IVP_Core::union_find_get_father (ivp_friction.cxx) */
ivp_core_t *ivp_core_union_find_get_father(ivp_core_t *core)
{
    ivp_core_t *obj = core;
    ivp_core_t *on_top = obj;
    while (obj) {
        on_top = obj;
        obj = obj->union_find_father;
    }
    return on_top;
}

static ivp_core_t *fs_union_find_fr_sys(ivp_fr_system_t *fr_sys)
{
    for (int k = ivp_pvec_len(&fr_sys->cores_of_friction_system) - 1; k >= 0; k--) {
        core_at(&fr_sys->cores_of_friction_system, k)->union_find_father = NULL;
    }
    for (int l = ivp_pvec_len(&fr_sys->fr_pairs_of_objs) - 1; l >= 0; l--) {
        ivp_fr_core_pair_t *fr_pair = pair_at(&fr_sys->fr_pairs_of_objs, l);
        ivp_core_t *obj0 = fr_pair->objs[0];
        ivp_core_t *obj1 = fr_pair->objs[1];
        if (obj0->physical_unmoveable || obj1->physical_unmoveable) continue;
        obj0 = ivp_core_union_find_get_father(obj0);
        obj1 = ivp_core_union_find_get_father(obj1);
        if (obj0 != obj1) obj1->union_find_father = obj0;
    }
    IVP_IF(0) /* ivp_friction.cxx:1275 */
    {
        for (int k = ivp_pvec_len(&fr_sys->cores_of_friction_system) - 1; k >= 0; k--) {
            ivp_core_t *objj = core_at(&fr_sys->cores_of_friction_system, k);
            ivp_core_t *of = ivp_core_union_find_get_father(objj);
            ivp_debug_printf("uff of %u : %u\n", fr_p16(objj), fr_p16(of));
        }
    }
    ivp_core_t *first_father = NULL, *second_father = NULL;
    for (int i1 = ivp_pvec_len(&fr_sys->cores_of_friction_system) - 1; i1 >= 0; i1--) {
        ivp_core_t *obj = core_at(&fr_sys->cores_of_friction_system, i1);
        if (!obj->physical_unmoveable) first_father = ivp_core_union_find_get_father(obj);
    }
    IVP_ASSERT(first_father);
    for (int i2 = ivp_pvec_len(&fr_sys->cores_of_friction_system) - 1; i2 >= 0; i2--) {
        ivp_core_t *obj = core_at(&fr_sys->cores_of_friction_system, i2);
        if (obj->physical_unmoveable) continue;
        ivp_core_t *test_father = ivp_core_union_find_get_father(obj);
        if (test_father != first_father) second_father = test_father;
    }
    return second_father;
}

typedef struct fs_split_static_plan {
    ivp_core_t *core;
    ivp_fr_info_for_core_t *info;
    bool linked;
} fs_split_static_plan_t;

static bool fs_pair_belongs_to_split(const ivp_fr_core_pair_t *pair,
                                     const ivp_core_t *split_father)
{
    ivp_core_t *moveable = pair->objs[0];
    if (moveable->physical_unmoveable) moveable = pair->objs[1];
    return ivp_core_union_find_get_father(moveable) == split_father;
}

static fs_split_static_plan_t *fs_split_find_static_plan(
    fs_split_static_plan_t *plans, int count, const ivp_core_t *core)
{
    for (int i = 0; i < count; i++) {
        if (plans[i].core == core) return &plans[i];
    }
    return NULL;
}

static void fs_split_friction_system(ivp_fr_system_t *fr_sys, ivp_core_t *split_father)
{
    const ivp_allocator_t *allocator = ivp_env_get_allocator(fr_sys->l_environment);
    ivp_fr_system_t *new_fr_sys = ivp_fr_system_create(fr_sys->l_environment);
    if (!new_fr_sys) return;
    int core_count = ivp_pvec_len(&fr_sys->cores_of_friction_system);
    if (core_count < 0 ||
        (size_t)core_count > SIZE_MAX / sizeof(fs_split_static_plan_t)) {
        ivp_fr_system_destroy(new_fr_sys);
        return;
    }
    fs_split_static_plan_t *static_plans = NULL;
    if (core_count > 0) {
        static_plans = (fs_split_static_plan_t *)ivp_alloc(
            allocator, (size_t)core_count * sizeof(*static_plans));
        if (!static_plans) {
            ivp_fr_system_destroy(new_fr_sys);
            return;
        }
        memset(static_plans, 0, (size_t)core_count * sizeof(*static_plans));
    }

    int static_count = 0;
    int moved_core_count = 0;
    int moved_moveable_count = 0;
    for (int i = 0; i < core_count; i++) {
        ivp_core_t *core = core_at(&fr_sys->cores_of_friction_system, i);
        if (core->physical_unmoveable) {
            static_plans[static_count++].core = core;
            moved_core_count++;
        } else if (ivp_core_union_find_get_father(core) == split_father) {
            moved_core_count++;
            moved_moveable_count++;
        }
    }
    int moved_pair_count = 0;
    for (int i = 0; i < ivp_pvec_len(&fr_sys->fr_pairs_of_objs); i++) {
        if (fs_pair_belongs_to_split(pair_at(&fr_sys->fr_pairs_of_objs, i),
                                     split_father)) {
            moved_pair_count++;
        }
    }
    if (!ivp_pvec_reserve(&new_fr_sys->cores_of_friction_system, moved_core_count) ||
        !ivp_pvec_reserve(&new_fr_sys->moveable_cores_of_friction_system,
                          moved_moveable_count) ||
        !ivp_pvec_reserve(&new_fr_sys->fr_pairs_of_objs, moved_pair_count)) {
        goto prepare_failed;
    }
    for (int i = 0; i < static_count; i++) {
        fs_split_static_plan_t *plan = &static_plans[i];
        plan->info = ivp_fr_info_create(allocator);
        if (!plan->info) goto prepare_failed;
        plan->info->l_friction_system = new_fr_sys;
        int spring_count = 0;
        for (int j = 0; j < ivp_pvec_len(&fr_sys->fr_pairs_of_objs); j++) {
            ivp_fr_core_pair_t *pair = pair_at(&fr_sys->fr_pairs_of_objs, j);
            if (!fs_pair_belongs_to_split(pair, split_father)) continue;
            if (pair->objs[0] == plan->core || pair->objs[1] == plan->core) {
                int pair_count = ivp_pvec_len(&pair->fr_dists);
                if (pair_count > INT_MAX - spring_count) goto prepare_failed;
                spring_count += pair_count;
            }
        }
        if (!ivp_pvec_reserve(&plan->info->friction_springs, spring_count)) {
            goto prepare_failed;
        }
    }
    for (int i = 0; i < static_count; i++) {
        if (!ivp_core_add_friction_info(static_plans[i].core,
                                        static_plans[i].info)) {
            goto prepare_failed;
        }
        static_plans[i].linked = true;
    }
    /* Core/controller registration is the final allocating step.  Install it
     * while the original system is still untouched so failure can simply
     * remove the new registrations. */
    for (int j = core_count - 1; j >= 0; j--) {
        ivp_core_t *core = core_at(&fr_sys->cores_of_friction_system, j);
        if (!core->physical_unmoveable &&
            ivp_core_union_find_get_father(core) != split_father) {
            continue;
        }
        if (!ivp_fr_system_add_core_to_system(new_fr_sys, core)) {
            goto prepare_failed;
        }
    }
    IVP_IF(fr_dm(fr_sys->l_environment)->check_fs) /* ivp_friction.cxx:1322 */
    {
        ivp_debug_printf("split_fs %f %p %p  cores ", ivp_env_get_current_time(fr_sys->l_environment),
                         (void *)fr_sys, (void *)new_fr_sys);
        for (int i = ivp_pvec_len(&fr_sys->cores_of_friction_system) - 1; i >= 0; i--) {
            ivp_core_t *my_core = core_at(&fr_sys->cores_of_friction_system, i);
            ivp_debug_printf("%p ", (void *)my_core);
        }
        ivp_debug_printf("\n");
    }
    for (int j = ivp_pvec_len(&fr_sys->cores_of_friction_system) - 1; j >= 0; j--) {
        ivp_core_t *obj = core_at(&fr_sys->cores_of_friction_system, j);
        if (obj->physical_unmoveable) {
            fs_split_static_plan_t *plan = fs_split_find_static_plan(
                static_plans, static_count, obj);
            IVP_ASSERT(plan && plan->linked);
            IVP_USE(plan);
        } else if (ivp_core_union_find_get_father(obj) == split_father) {
            ivp_fr_system_remove_core_from_system(fr_sys, obj);
            ivp_fr_info_for_core_t *fr_i = ivp_core_get_friction_info(obj, fr_sys);
            fr_i->l_friction_system = new_fr_sys;
        }
    }
    for (int n = ivp_pvec_len(&fr_sys->fr_pairs_of_objs) - 1; n >= 0; n--) {
        ivp_fr_core_pair_t *fr_pair = pair_at(&fr_sys->fr_pairs_of_objs, n);
        ivp_core_t *robj = fr_pair->objs[0];
        ivp_core_t *robj2 = fr_pair->objs[1];
        ivp_fr_info_for_core_t *fr_i_old = NULL, *fr_i_new = NULL;
        if (robj->physical_unmoveable) {
            ivp_core_t *ct = robj2;
            robj2 = robj;
            robj = ct;
            fr_i_old = ivp_core_get_friction_info(robj2, fr_sys);
            fr_i_new = ivp_core_get_friction_info(robj2, new_fr_sys);
        } else if (robj2->physical_unmoveable) {
            fr_i_old = ivp_core_get_friction_info(robj2, fr_sys);
            fr_i_new = ivp_core_get_friction_info(robj2, new_fr_sys);
        }
        if (ivp_core_union_find_get_father(robj) == split_father) {
            fs_del_fr_pair(fr_sys, fr_pair);
            bool pair_added = fs_add_fr_pair(new_fr_sys, fr_pair);
            IVP_ASSERT(pair_added);
            IVP_USE(pair_added);
            for (int k = ivp_pvec_len(&fr_pair->fr_dists) - 1; k >= 0; k--) {
                ivp_contact_pt_t *fr_dist = cp_at(&fr_pair->fr_dists, k);
                {
                    int found_mine = 0;
                    for (ivp_contact_pt_t *mindist = fr_sys->first_friction_dist; mindist;
                         mindist = mindist->next_dist_in_friction) {
                        if (mindist == fr_dist) found_mine = 1;
                    }
                    if (!found_mine) {
                        IVP_IF(1) { ivp_debug_printf("removing_dist that doesnt belong to sys\n"); }
                        IVP_CORE();
                    }
                }
                ivp_fr_system_remove_dist_from_system(fr_sys, fr_dist);
                ivp_fr_system_add_dist_to_system(new_fr_sys, fr_dist);
                if (fr_i_old) {
                    ivp_fr_info_delete_friction_dist(fr_i_old, fr_dist);
                    bool inserted = ivp_fr_info_insert_friction_dist(fr_i_new, fr_dist);
                    IVP_ASSERT(inserted);
                    IVP_USE(inserted);
                }
            }
        }
    }
    if (static_plans) {
        ivp_free(allocator, static_plans,
                 (size_t)core_count * sizeof(*static_plans));
    }
    for (int l = ivp_pvec_len(&fr_sys->cores_of_friction_system) - 1; l >= 0; l--) {
        ivp_core_t *obj = core_at(&fr_sys->cores_of_friction_system, l);
        if (obj->physical_unmoveable) {
            ivp_fr_info_for_core_t *fr_i_new = ivp_core_get_friction_info(obj, new_fr_sys);
            ivp_fr_info_for_core_t *fr_i_old = ivp_core_get_friction_info(obj, fr_sys);
            if (ivp_pvec_len(&fr_i_new->friction_springs) == 0) {
                ivp_core_delete_friction_info(obj, fr_i_new);
                ivp_fr_system_remove_core_from_system(new_fr_sys, obj);
            }
            if (ivp_pvec_len(&fr_i_old->friction_springs) == 0) {
                ivp_core_delete_friction_info(obj, fr_i_old);
                ivp_fr_system_remove_core_from_system(fr_sys, obj);
            }
        }
    }
    if (new_fr_sys->friction_obj_number < 2) {
        ivp_core_t *obj = core_at(&new_fr_sys->cores_of_friction_system, 0);
        ivp_fr_info_for_core_t *fr_i = ivp_core_get_friction_info(obj, new_fr_sys);
        ivp_core_delete_friction_info(obj, fr_i);
        ivp_fr_system_destroy(new_fr_sys);
        return;
    }
    if (fr_sys->friction_obj_number < 2) {
        ivp_core_t *obj = core_at(&fr_sys->cores_of_friction_system, 0);
        ivp_fr_info_for_core_t *fr_i = ivp_core_get_friction_info(obj, fr_sys);
        ivp_core_delete_friction_info(obj, fr_i);
        ivp_fr_system_destroy(fr_sys);
        return;
    }
    IVP_IF(1 == 1) /* ivp_friction.cxx:1479, "expensive but important" */
    {
        ivp_fr_system_test_hole_fr_system_data(new_fr_sys);
        ivp_fr_system_test_hole_fr_system_data(fr_sys);
    }
    ivp_core_t *obj = fs_union_find_fr_sys(fr_sys);
    if (obj) fs_split_friction_system(fr_sys, obj);
    return;

prepare_failed:
    while (ivp_pvec_len(&new_fr_sys->cores_of_friction_system) > 0) {
        ivp_core_t *added_core = core_at(
            &new_fr_sys->cores_of_friction_system,
            ivp_pvec_len(&new_fr_sys->cores_of_friction_system) - 1);
        ivp_fr_system_remove_core_from_system(new_fr_sys, added_core);
    }
    for (int i = static_count - 1; i >= 0; i--) {
        if (static_plans[i].linked) {
            ivp_core_unlink_friction_info(static_plans[i].core,
                                          static_plans[i].info);
        }
        if (static_plans[i].info) ivp_fr_info_destroy(static_plans[i].info);
    }
    if (static_plans) {
        ivp_free(allocator, static_plans,
                 (size_t)core_count * sizeof(*static_plans));
    }
    ivp_fr_system_destroy(new_fr_sys);
}

void ivp_fr_system_reset_time(ivp_fr_system_t *fs, double offset)
{
    for (int i = ivp_pvec_len(&fs->fr_pairs_of_objs) - 1; i >= 0; i--) {
        ivp_fr_core_pair_t *fr_pair = pair_at(&fs->fr_pairs_of_objs, i);
        for (int c = ivp_pvec_len(&fr_pair->fr_dists) - 1; c >= 0; c--) {
            ivp_contact_pt_reset_time(cp_at(&fr_pair->fr_dists, c), offset);
        }
    }
}

static void fs_remove_energy_gained_by_real_friction(ivp_fr_system_t *fs)
{
    for (int i = ivp_pvec_len(&fs->fr_pairs_of_objs) - 1; i >= 0; i--) {
        pair_remove_energy_gained_by_real_friction(pair_at(&fs->fr_pairs_of_objs, i));
    }
}

static void fs_clear_integrated_anti_energy(ivp_fr_system_t *fs)
{
    for (int k = ivp_pvec_len(&fs->fr_pairs_of_objs) - 1; k >= 0; k--) {
        pair_at(&fs->fr_pairs_of_objs, k)->integrated_anti_energy = 0.0f;
    }
}

static void fs_calc_friction_forces(ivp_fr_system_t *fs, const ivp_sim_es_t *es)
{
    for (int i = ivp_pvec_len(&fs->fr_pairs_of_objs) - 1; i >= 0; i--) {
        ivp_fr_core_pair_t *my_pair = pair_at(&fs->fr_pairs_of_objs, i);
        IVP_IF(ivp_pvec_len(&my_pair->fr_dists) == 0) /* ivp_friction.cxx:516 */
        {
            ivp_debug_printf("\n\n\n warngin: empty core pair\n");
        }
        pair_calc_friction_forces(my_pair, es);
    }
}

static void fs_ease_friction_forces(ivp_fr_system_t *fs)
{
    for (int i = ivp_pvec_len(&fs->fr_pairs_of_objs) - 1; i >= 0; i--) {
        ivp_fr_core_pair_t *my_pairs = pair_at(&fs->fr_pairs_of_objs, i);
        my_pairs->next_ease_nr_psi--;
        if (my_pairs->next_ease_nr_psi == 0) {
            frs_ease_friction_pair(my_pairs, ivp_env_get_short_term_mem(fs->l_environment),
                                   fs->l_environment);
            my_pairs->next_ease_nr_psi = IVP_FR_EASE_EVERY_NTH_PSI;
        }
    }
}

static void fs_apply_real_friction(ivp_fr_system_t *fs, const ivp_sim_es_t *es)
{
    fs_calc_friction_forces(fs, es);
    fs_ease_friction_forces(fs);
}

/* ── IVP_Friction_Solver (ivp_friction_gaps.cxx) ───────────────────────── */

typedef struct fr_solver {
    ivp_gmmz_t  dist_change_mat;
    double      correct_x_factor;
    struct ivp_environment *l_environment;
    const ivp_sim_es_t *es;
    ivp_pvec_t  contact_info_vector;   /* ivp_impact_solver_lt_t* */
    int         gauss_succed;          /* for perf test (debug builds count it) */
} fr_solver_t;

/* IVP_Great_Matrix_Many_Zero::align_matrix_values: its IVP_IF(1) block
 * (ivp_great_matrix.cxx:471) zeroes the row padding of matrix_values and of
 * desired_vector.  Debug builds perform this explicit fill; Release normally
 * clears arena spans, while reference-internals builds preserve reuse. */
static void frs_align_matrix_values_debug_fill(ivp_gmmz_t *m)
{
    for (int i = 0; i < m->columns; i++) {
        double *x = &m->matrix_values[i * m->aligned_row_len];
        for (int j = m->columns; j < m->aligned_row_len; j++) {
            x[j] = 0.0;
        }
    }
    for (int j = m->columns; j < m->aligned_row_len; j++) {
        m->desired_vector[j] = 0.0;
    }
}

/* IVP_Friction_Solver::IVP_Friction_Solver: dist_change_mat from the
 * short-term arena and not initialized here (calc_solver_PSI clears the
 * matrix; the IVP_IF(1) padding fill of align_matrix_values runs in debug
 * builds only).  Release padding follows the selected arena behavior. */
static bool frs_init(fr_solver_t *s, ivp_fr_system_t *fs,
                     const ivp_sim_es_t *es, ivp_arena_t *arena)
{
    memset(s, 0, sizeof(*s));
    s->correct_x_factor = 0;
    s->l_environment = fs->l_environment;
    s->es = es;
#ifdef IVP_DEBUG_BUILD
    s->gauss_succed = 0;
#endif
    ivp_pvec_init_with_allocator(&s->contact_info_vector,
                                 ivp_env_get_allocator(s->l_environment));
    ivp_gmmz_t *m = &s->dist_change_mat;
    m->matrix_eps = IVP_GMMZ_DEFAULT_EPS;
    m->columns = fs->friction_dist_number - fs->complex_not_necessary_number;
    if (!ivp_pvec_reserve(&s->contact_info_vector, m->columns)) return false;
    ivp_gmmz_calc_aligned_row_len(m);
    size_t arl = (size_t)m->aligned_row_len;
    m->matrix_values = (double *)ivp_arena_alloc(
        arena, (arl * (size_t)m->columns +
                (size_t)(IVP_GM_VECFPU_SIZE - 1)) * sizeof(double));
    m->desired_vector = (double *)ivp_arena_alloc(
        arena, arl * sizeof(double));
    m->result_vector = (double *)ivp_arena_alloc(
        arena, arl * sizeof(double));
    /* align_matrix_values(): arena memory is 32-byte aligned, nothing moves */
    if (!(m->matrix_values && m->desired_vector && m->result_vector)) return false;
    IVP_IF(1)
    {
        frs_align_matrix_values_debug_fill(m);
    }
    return true;
}

static void frs_destroy(fr_solver_t *s)
{
    ivp_pvec_clear(&s->contact_info_vector);
}

static ivp_impact_solver_lt_t *frs_info_at(fr_solver_t *s, int i) {
    return (ivp_impact_solver_lt_t *)ivp_pvec_at(&s->contact_info_vector, i);
}

static void frs_setup_coords_mindists(fr_solver_t *s, ivp_fr_system_t *fs)
{
    int index_of_contact_point = 0;
    int total_dist_counter = 0;
    for (ivp_contact_pt_t *my_dist = fs->first_friction_dist; my_dist; my_dist = my_dist->next_dist_in_friction) {
        ivp_impact_solver_lt_t *info = my_dist->tmp_contact_info;
        if (total_dist_counter++ < fs->complex_not_necessary_number) {
            info->index_in_fs = -1;
            continue;
        }
        info->friction.friction_infos[0] = ivp_core_get_friction_info(cp_core(my_dist, 0), fs);
        info->friction.friction_infos[1] = ivp_core_get_friction_info(cp_core(my_dist, 1), fs);
        info->friction.dist_len = ivp_cp_get_gap_length(my_dist);
        info->friction.has_negative_pull_since = my_dist->has_negative_pull_since;
        int added = ivp_pvec_add(&s->contact_info_vector, info);
        IVP_ASSERT(added >= 0);
        IVP_ASSERT(added == index_of_contact_point);
        IVP_USE(added);
        info->index_in_fs = (short)index_of_contact_point++;
    }
}

static void frs_calc_distance_matrix_column(fr_solver_t *s, int current_contact_point_index, ivp_core_t *core,
                                            ivp_fr_info_for_core_t *fr_info, const ivp_vec3f_t *rotation_vec,
                                            const ivp_vec3f_t *translation_vec)
{
    IVP_ASSERT(!core->physical_unmoveable);
    ivp_gmmz_t *m = &s->dist_change_mat;
    /* The loop does not resize either container.  Keep their bases and
     * dimensions out of the dependent contact-row load chain. */
    void **springs = fr_info->friction_springs.elems;
    int spring_count = fr_info->friction_springs.n;
    double *matrix_values = m->matrix_values;
    int aligned_row_len = m->aligned_row_len;
    for (int col = 0; col < spring_count; col++) {
        ivp_contact_pt_t *distance = (ivp_contact_pt_t *)springs[col];
        ivp_impact_solver_lt_t *info = distance->tmp_contact_info;
        int line_num = info->index_in_fs;
        if (line_num < 0) continue;
        /* frs_setup_coords_mindists installs this exact info pointer at
         * line_num.  Reuse it instead of round-tripping through the vector. */
        int which_obj;
        double factor;
        if (info->contact_core[0] == core) {
            which_obj = 0;
            factor = -1.0;
        } else {
            factor = +1.0;
            which_obj = 1;
        }
        if (info->contact_core[which_obj]) {
            double speed_change_scalar = factor * frs_get_closing_speed_core_i(info, which_obj, rotation_vec, translation_vec);
            matrix_values[line_num * aligned_row_len + current_contact_point_index] += speed_change_scalar;
        }
    }
}

static void frs_calc_solver_desired(fr_solver_t *s, int i,
                                    ivp_impact_solver_lt_t *info)
{
    ivp_gmmz_t *m = &s->dist_change_mat;
    double dist_velocity = ivp_impact_lt_get_closing_speed(info);
    double distance_off =
        (double)(ivp_env_get_mindist_settings(s->l_environment)->friction_dist -
                 info->friction.dist_len); /* IVP_FLOAT - IVP_FLOAT */
    m->desired_vector[i] =
        ivp_frs_calc_desired_gap_speed(dist_velocity, distance_off, 1.0f);
}

static void frs_calc_solver_column_effect(fr_solver_t *s, int i,
                                          ivp_impact_solver_lt_t *info)
{
    if (info->contact_core[0]) {
        ivp_vec3f_t rotation_vec, translation_vec;
        frs_test_push_core_i(info, 0, &rotation_vec, &translation_vec, -1.0f);
        frs_calc_distance_matrix_column(
            s, i, info->contact_core[0], info->friction.friction_infos[0],
            &rotation_vec, &translation_vec);
    }
    if (info->contact_core[1]) {
        ivp_vec3f_t rotation_vec, translation_vec;
        frs_test_push_core_i(info, 1, &rotation_vec, &translation_vec, +1.0f);
        frs_calc_distance_matrix_column(
            s, i, info->contact_core[1], info->friction.friction_infos[1],
            &rotation_vec, &translation_vec);
    }
}

typedef struct frs_calc_solver_columns_context {
    fr_solver_t *solver;
    int contact_count;
} frs_calc_solver_columns_context_t;

static void frs_calc_solver_column_block_task(void *opaque, size_t task_index)
{
    frs_calc_solver_columns_context_t *context =
        (frs_calc_solver_columns_context_t *)opaque;
    fr_solver_t *s = context->solver;
    size_t first = task_index * IVP_FR_PARALLEL_COLUMN_BLOCK;
    size_t end = first + IVP_FR_PARALLEL_COLUMN_BLOCK;
    ivp_gmmz_t *m = &s->dist_change_mat;
    if (end > (size_t)m->aligned_row_len) {
        end = (size_t)m->aligned_row_len;
    }
    /* Each task owns these columns across every row, including padding.
     * Clearing here adds work to the existing barrier without changing any
     * matrix cell's writer or the arithmetic order within a real column. */
    for (int row = 0; row < m->columns; row++) {
        double *values =
            &m->matrix_values[row * m->aligned_row_len];
        for (size_t column = first; column < end; column++) {
            values[column] = 0.0;
        }
    }
    size_t contact_end = end;
    if (contact_end > (size_t)context->contact_count) {
        contact_end = (size_t)context->contact_count;
    }
    for (size_t column = first; column < contact_end; column++) {
        int i = (int)column;
        ivp_impact_solver_lt_t *info = frs_info_at(s, i);
        frs_calc_solver_desired(s, i, info);
        frs_calc_solver_column_effect(s, i, info);
    }
}

static int frs_calc_solver_psi(fr_solver_t *s, ivp_fr_system_t *fs,
                               int *active_is_at_pos)
{
    ivp_gmmz_t *m = &s->dist_change_mat;
    IVP_IF((fr_dm(s->l_environment)->debug_friction) || 0) /* ivp_friction_gaps.cxx:1017 */
    {
        int counter = ivp_env_next_global_friction_debug_counter(s->l_environment);
        ivp_debug_printf("\ndoing_friction_calc %d\n", counter);
    }
    const int contact_count = ivp_pvec_len(&s->contact_info_vector);
    size_t minimum_tasks = 8u;
    int worker_count = ivp_env_parallel_worker_count(s->l_environment);
    if (worker_count > 0 && minimum_tasks < (size_t)worker_count) {
        minimum_tasks = (size_t)worker_count;
    }
    size_t matrix_interactions = 0;
    /* A friction matrix has at most 150 columns.  On current desktop CPUs,
     * smaller matrices do not amortize a pool barrier, and using more than
     * four lanes only adds wakeup cost at that fixed maximum size. */
    if (worker_count > 1 && worker_count <= IVP_FR_PARALLEL_MAX_WORKERS &&
        (size_t)contact_count >= minimum_tasks) {
        for (int i = 0; i < contact_count; i++) {
            ivp_impact_solver_lt_t *info = frs_info_at(s, i);
            for (int core_index = 0; core_index < 2; core_index++) {
                if (!info->contact_core[core_index]) continue;
                int springs = ivp_pvec_len(
                    &info->friction.friction_infos[core_index]
                         ->friction_springs);
                if ((size_t)springs > SIZE_MAX - matrix_interactions) {
                    matrix_interactions = SIZE_MAX;
                    break;
                }
                matrix_interactions += (size_t)springs;
            }
            if (matrix_interactions == SIZE_MAX) break;
        }
    }
    size_t column_block_count =
        ((size_t)m->aligned_row_len + IVP_FR_PARALLEL_COLUMN_BLOCK - 1u) /
        IVP_FR_PARALLEL_COLUMN_BLOCK;
    frs_calc_solver_columns_context_t task_context = { s, contact_count };
    bool dispatched =
        matrix_interactions >= IVP_FR_PARALLEL_MIN_INTERACTIONS &&
        ivp_env_parallel_for(s->l_environment, IVP_PARALLEL_WORK_SOLVER,
                             column_block_count, minimum_tasks,
                             frs_calc_solver_column_block_task,
                             &task_context);
    int active_dists_so_far = 0;
    if (dispatched) {
        for (int i = 0; i < contact_count; i++) {
            ivp_impact_solver_lt_t *info = frs_info_at(s, i);
            if (info->friction.has_negative_pull_since != 0) {
                *active_is_at_pos = i;
                active_is_at_pos++;
                active_dists_so_far++;
            }
        }
    } else {
        int mat_width = (m->aligned_row_len * m->columns);
        for (int d = mat_width - 1; d >= 0; d--) {
            m->matrix_values[d] = 0.0;
        }
        /* Preserve the reference loop order on the default serial path:
         * desired value, active-set append, then matrix-column effects. */
        for (int i = 0; i < contact_count; i++) {
            ivp_impact_solver_lt_t *info = frs_info_at(s, i);
            frs_calc_solver_desired(s, i, info);
            if (info->friction.has_negative_pull_since != 0) {
                *active_is_at_pos = i;
                active_is_at_pos++;
                active_dists_so_far++;
            }
            frs_calc_solver_column_effect(s, i, info);
        }
    }
    /* ivp_friction_gaps.cxx:1077: index check, CORE in every build */
    int k = 0;
    for (ivp_contact_pt_t *my_dist = fs->first_friction_dist; my_dist;
         k++, my_dist = my_dist->next_dist_in_friction) {
        if (my_dist->tmp_contact_info->index_in_fs < 0) continue;
        if (my_dist->tmp_contact_info->index_in_fs != k) {
            IVP_CORE();
        }
    }
    return active_dists_so_far;
}

static void frs_normize_constraint_equ(fr_solver_t *s)
{
    ivp_gmmz_t *m = &s->dist_change_mat;
    double max_val = 0.0;
    for (int i = m->columns - 1; i >= 0; i--) {
        double test_val = m->matrix_values[i * m->aligned_row_len + i];
        if (test_val > max_val) max_val = test_val;
    }
    double a_norm = (max_val > P_DOUBLE_EPS) ? 1.0f / max_val : 1.0f;
    s->correct_x_factor = a_norm;
    max_val = 0.0;
    for (int i = m->columns - 1; i >= 0; i--) {
        double test_val = fabs(m->desired_vector[i]);
        if (test_val > max_val) max_val = test_val;
    }
    double b_norm;
    if (max_val > P_DOUBLE_EPS) {
        b_norm = 1.0f / max_val;
    } else {
        b_norm = 1.0f;
        max_val = 1.0f;
    }
    s->correct_x_factor *= max_val;
    for (int i = 0; i < m->columns; i++) {
        double *base = &m->matrix_values[i * m->aligned_row_len];
        for (int j = 0; j < m->aligned_row_len; j++) base[j] *= a_norm;
    }
    for (int i = m->columns - 1; i >= 0; i--) m->desired_vector[i] *= b_norm;
}

static void frs_factor_result_vec(fr_solver_t *s)
{
    for (int i = s->dist_change_mat.columns - 1; i >= 0; i--) {
        s->dist_change_mat.result_vector[i] *= s->correct_x_factor;
    }
}

static bool frs_test_gauss_solution_suggestion(fr_solver_t *s, const double *push_results, const int *active_is_at_pos,
                                               int total_actives, ivp_arena_t *arena)
{
    ivp_gmmz_t *m = &s->dist_change_mat;
    /* ivp_u_bool (int) array from the arena, cleared */
    int *was_already_tested = (int *)ivp_arena_alloc(
        arena, (size_t)m->columns * sizeof(int));
    if (!was_already_tested) return false;
    memset(was_already_tested, 0, sizeof(int) * (size_t)m->columns);
    int gauss_failed = 0;
    for (int i = 0; i < total_actives; i++) {
        int orig_index = active_is_at_pos[i];
        ivp_impact_solver_lt_t *info = frs_info_at(s, orig_index);
        double push_val = push_results[i];
        /* IVP_MAX_ADHESION_GAUSS (float) * IVP_FLOAT gravity_scalar */
        double allowed_negative = (double)(IVP_FR_MAX_ADHESION_GAUSS * (float)ivp_env_get_gravity_scalar(s->l_environment));
        if (allowed_negative > push_val * s->correct_x_factor * (double)info->inv_virtual_mass) {
            IVP_IF((fr_dm(s->l_environment)->debug_friction) || 0) /* ivp_friction_gaps.cxx:47 */
            {
                ivp_debug_printf("gauss_fault_pull %f %f\n", allowed_negative, push_val);
            }
            gauss_failed = 1;
        }
        was_already_tested[orig_index] = 1; /* IVP_TRUE */
        m->result_vector[orig_index] = push_val;
    }
    for (int i = 0; i < m->columns; i++) {
        if (!was_already_tested[i]) {
            if (ivp_gmmz_check_unequation_line(m, i) != IVP_OK) {
                return false;
            }
        }
    }
    return gauss_failed ? false : true;
}

static int fs_get_num_supposed_active_frdists(ivp_fr_system_t *fs)
{
    int sum = 0;
    for (ivp_contact_pt_t *fr_dist = fs->first_friction_dist; fr_dist; fr_dist = fr_dist->next_dist_in_friction) {
        if (fr_dist->has_negative_pull_since < 0) {
            sum++;
        } else {
            return sum;
        }
    }
    return sum;
}

static double fs_kinetic_energy_of_hole_frs(ivp_fr_system_t *fs)
{
    double sum = 0.0;
    for (int i = ivp_pvec_len(&fs->cores_of_friction_system) - 1; i >= 0; i--) {
        ivp_core_t *my_obj = core_at(&fs->cores_of_friction_system, i);
        ivp_vec3f_t speed_vec, rot_vec;
        speed_vec.k[0] = (float)(my_obj->speed.k[0] + my_obj->speed_change.k[0]);
        speed_vec.k[1] = (float)(my_obj->speed.k[1] + my_obj->speed_change.k[1]);
        speed_vec.k[2] = (float)(my_obj->speed.k[2] + my_obj->speed_change.k[2]);
        rot_vec.k[0] = (float)((ivp_real_t)my_obj->rot_speed.k[0] + my_obj->rot_speed_change.k[0]);
        rot_vec.k[1] = (float)((ivp_real_t)my_obj->rot_speed.k[1] + my_obj->rot_speed_change.k[1]);
        rot_vec.k[2] = (float)((ivp_real_t)my_obj->rot_speed.k[2] + my_obj->rot_speed_change.k[2]);
        sum += ivp_core_get_energy_on_test(my_obj, &speed_vec, &rot_vec);
    }
    return sum;
}

static void fs_confirm_complex_pushes(ivp_fr_system_t *fs)
{
    for (int k = ivp_pvec_len(&fs->moveable_cores_of_friction_system) - 1; k >= 0; k--) {
        ivp_core_commit_all_async_pushes_ref(core_at(&fs->moveable_cores_of_friction_system, k));
    }
}

static void fs_undo_complex_pushes(ivp_fr_system_t *fs)
{
    for (int k = ivp_pvec_len(&fs->moveable_cores_of_friction_system) - 1; k >= 0; k--) {
        ivp_core_abort_all_async_pushes(core_at(&fs->moveable_cores_of_friction_system, k));
    }
}

static double fs_get_max_energy_gain(ivp_fr_system_t *fs)
{
    double sum_gain = 0.0;
    for (int k = ivp_pvec_len(&fs->cores_of_friction_system) - 1; k >= 0; k--) {
        ivp_core_t *my_core = core_at(&fs->cores_of_friction_system, k);
        if (!my_core->physical_unmoveable) {
            double force_gravity = (double)(ivp_core_get_mass_f(my_core) * 9.81f); /* IVP_FLOAT * float */
            sum_gain += force_gravity * (double)0.1f;
        }
    }
    return sum_gain;
}

/* IVP_Friction_Solver::debug_distance_after_push: its body is "#if 0" in the
 * reference ("muss noch umgeschrieben werden") */
static void frs_debug_distance_after_push(fr_solver_t *s, int counter)
{
    IVP_USE(s);
    IVP_USE(counter);
}

#ifdef IVP_DEBUG_BUILD
/* IVP_Friction_Solver::print_dist_velocity (reference "#ifdef DEBUG") */
static void frs_print_dist_velocity(fr_solver_t *s, ivp_fr_system_t *fs)
{
    IVP_USE(s);
    int dist_counter_two = 0;
    int total_dist_counter = -1;
    ivp_contact_pt_t *my_dist, *next_dist = NULL;
    my_dist = fs->first_friction_dist;
    while (my_dist) {
        total_dist_counter++;
        next_dist = my_dist->next_dist_in_friction;
        if (total_dist_counter < fs->complex_not_necessary_number) {
            my_dist = next_dist;
            continue;
        }
        ivp_core_t *core0 = cp_core(my_dist, 0);
        ivp_core_t *core1 = cp_core(my_dist, 1);
        ivp_vec3f_t w_speed0, w_speed1;
        ivp_vec3f_t speed_var_trans, speed_var_rot;
        /* IVP_U_Float_Point::add(speed, speed_change) etc. (float sums) */
        for (int k = 0; k < 3; k++) {
            speed_var_trans.k[k] = (float)(core0->speed.k[k] + core0->speed_change.k[k]);
            speed_var_rot.k[k] = (float)((ivp_real_t)core0->rot_speed.k[k] + core0->rot_speed_change.k[k]);
        }
        ivp_core_get_surface_speed_on_test(core0, &my_dist->tmp_contact_info->contact_point_cs[0], &speed_var_trans,
                                           &speed_var_rot, &w_speed0);
        for (int k = 0; k < 3; k++) {
            speed_var_trans.k[k] = (float)(core1->speed.k[k] + core1->speed_change.k[k]);
            speed_var_rot.k[k] = (float)((ivp_real_t)core1->rot_speed.k[k] + core1->rot_speed_change.k[k]);
        }
        ivp_core_get_surface_speed_on_test(core1, &my_dist->tmp_contact_info->contact_point_cs[1], &speed_var_trans,
                                           &speed_var_rot, &w_speed1);
        ivp_vec3f_t rel_world_speed;
        lvf_subtract2(&rel_world_speed, &w_speed1, &w_speed0);
        IVP_IF(1) /* ivp_friction_gaps.cxx:667 */
        {
            ivp_debug_printf("dvelo_ %.4f   ", (double)lvf_dot(&rel_world_speed, &my_dist->tmp_contact_info->surf_normal));
        }
        dist_counter_two++;
        my_dist = next_dist;
    }
    IVP_USE(dist_counter_two);
    IVP_IF(1) /* ivp_friction_gaps.cxx:675 */
    {
        ivp_debug_printf("\n");
    }
}
#endif /* IVP_DEBUG_BUILD */

static void frs_do_resulting_pushes(fr_solver_t *s, ivp_fr_system_t *fs)
{
    int m_index = 0;
    for (ivp_contact_pt_t *my_dist = fs->first_friction_dist; my_dist;
         m_index++, my_dist = my_dist->next_dist_in_friction) {
        double impulse_now;
        if (m_index < fs->complex_not_necessary_number) {
            impulse_now = 0.0;
            goto no_complex_done;
        }
        impulse_now = s->dist_change_mat.result_vector[m_index];
        if (impulse_now > 0.0) {
            if (my_dist->has_negative_pull_since >= 0) {
                my_dist->has_negative_pull_since = -1;
            } else {
                my_dist->has_negative_pull_since--;
            }
        } else {
            if (impulse_now == 0.0) {
                my_dist->has_negative_pull_since = 0;
                impulse_now = 0.0;
                goto cont_for_next;
            } else {
                if (my_dist->has_negative_pull_since < 0) my_dist->has_negative_pull_since = 0;
                my_dist->has_negative_pull_since++;
                if (my_dist->has_negative_pull_since > 9) my_dist->has_negative_pull_since = 0;
            }
        }
        ivp_frs_async_apply_impulse(my_dist->tmp_contact_info, impulse_now);
    cont_for_next:
        if (impulse_now < 0.0) impulse_now = 0.0;
    no_complex_done:
        my_dist->now_friction_pressure = (float)(impulse_now * s->es->i_delta_time);
    }
    IVP_IF(fr_dm(s->l_environment)->debug_friction) /* ivp_friction_gaps.cxx:601 */
    {
        for (int j = 0; j < fs->friction_dist_number; j++) {
            IVP_IF(j >= fs->complex_not_necessary_number)
            {
                frs_debug_distance_after_push(s, j - fs->complex_not_necessary_number);
            }
        }
    }
#ifdef IVP_DEBUG_BUILD
    IVP_IF(fr_dm(s->l_environment)->debug_friction) /* ivp_friction_gaps.cxx:614 */
    {
        ivp_debug_printf("pushes_ok. resulting_distance_velos:\n");
        frs_print_dist_velocity(s, fs);
    }
#endif /* IVP_DEBUG_BUILD */
}

static void frs_solve_linear_equation_and_push(fr_solver_t *s, ivp_fr_system_t *my_fs, int *active_is_at_pos,
                                               int total_actives, ivp_arena_t *arena)
{
    ivp_gmmz_t *big = &s->dist_change_mat;
    ivp_gmmz_t actives_matrix;
    memset(&actives_matrix, 0, sizeof(actives_matrix));
    actives_matrix.matrix_eps = IVP_GMMZ_DEFAULT_EPS;
    actives_matrix.columns = total_actives;
    ivp_gmmz_calc_aligned_row_len(&actives_matrix);
    frs_normize_constraint_equ(s);
    /* arena arrays, uninitialised, in the reference order */
    size_t arl = (size_t)actives_matrix.aligned_row_len;
    actives_matrix.desired_vector = (double *)ivp_arena_alloc(
        arena, arl * sizeof(double));
    actives_matrix.result_vector = (double *)ivp_arena_alloc(
        arena, arl * sizeof(double));
    actives_matrix.matrix_values = (double *)ivp_arena_alloc(
        arena, ((size_t)total_actives * arl +
                (size_t)(IVP_GM_VECFPU_SIZE - 1)) * sizeof(double));
    /* align_matrix_values(): arena memory is 32-byte aligned, nothing moves */
    if (!actives_matrix.desired_vector || !actives_matrix.result_vector || !actives_matrix.matrix_values) return;
    IVP_IF(1)
    {
        frs_align_matrix_values_debug_fill(&actives_matrix);
    }
    memset(big->result_vector, 0, (size_t)big->aligned_row_len * sizeof(double));
    bool ok = false;
    ivp_gmmz_fill_from_bigger_matrix(&actives_matrix, big, active_is_at_pos, total_actives);
    if (ivp_gmmz_solve(&actives_matrix) == IVP_OK) {
        if (frs_test_gauss_solution_suggestion(s, actives_matrix.result_vector, active_is_at_pos, total_actives,
                                               arena)) {
            IVP_IF(1) /* ivp_friction_gaps.cxx:209 */
            {
                s->gauss_succed++;
            }
            ok = true;
        }
    }
    if (!ok) {
        int supposed_actives = fs_get_num_supposed_active_frdists(my_fs);
        if (ivp_lcs_init_and_solve_in_memory(big->matrix_values, big->desired_vector, big->result_vector,
                                             big->columns, supposed_actives,
                                             arena) == IVP_OK) {
            ok = true;
        } else {
            IVP_IF(1) /* ivp_friction_gaps.cxx:249 */
            {
                ivp_debug_printf("static_friction_unequation failed\n");
            }
        }
    }
    if (ok) {
        frs_factor_result_vec(s);
        double e_before = fs_kinetic_energy_of_hole_frs(my_fs);
        frs_do_resulting_pushes(s, my_fs);
        double e_after = fs_kinetic_energy_of_hole_frs(my_fs);
        double e_max_grown = fs_get_max_energy_gain(my_fs);
        if (e_after <= e_before + e_max_grown) {
            fs_confirm_complex_pushes(my_fs);
        } else {
            IVP_IF(1) /* ivp_friction_gaps.cxx:240 */
            {
                ivp_debug_printf("too_much_energy_gained\n");
            }
            fs_undo_complex_pushes(my_fs);
        }
    }
}

static void fs_exchange_friction_dists(ivp_fr_system_t *fs, ivp_contact_pt_t *first_frd, ivp_contact_pt_t *second_frd)
{
    if (fs->first_friction_dist == first_frd) fs->first_friction_dist = second_frd;
    if (first_frd->prev_dist_in_friction) first_frd->prev_dist_in_friction->next_dist_in_friction = second_frd;
    if (second_frd->next_dist_in_friction) second_frd->next_dist_in_friction->prev_dist_in_friction = first_frd;
    first_frd->next_dist_in_friction = second_frd->next_dist_in_friction;
    second_frd->prev_dist_in_friction = first_frd->prev_dist_in_friction;
    first_frd->prev_dist_in_friction = second_frd;
    second_frd->next_dist_in_friction = first_frd;
}

static void fs_bubble_sort_dists_importance(ivp_fr_system_t *fs)
{
    /* Normal ordering extends the reference's partial importance order to a
     * total order with the contact's world-local identity. */
    const bool use_reference_order =
        ivp_build_uses_reference_internal_state();
    ivp_contact_pt_t *first_dist = fs->first_friction_dist;
    if (!first_dist) return;
    for (;;) {
        ivp_contact_pt_t *second_dist = first_dist->next_dist_in_friction;
        if (!second_dist) break;
        bool should_shift =
            first_dist->has_negative_pull_since >
                second_dist->has_negative_pull_since;
        if (!use_reference_order &&
            first_dist->has_negative_pull_since ==
                second_dist->has_negative_pull_since) {
            IVP_ASSERT(first_dist->creation_sequence !=
                       second_dist->creation_sequence);
            should_shift = first_dist->creation_sequence >
                           second_dist->creation_sequence;
        }
        if (should_shift) {
            fs_exchange_friction_dists(fs, first_dist, second_dist);
            ivp_contact_pt_t *test_dist;
            while ((test_dist = second_dist->prev_dist_in_friction) != NULL) {
                bool test_should_shift =
                    test_dist->has_negative_pull_since >
                        second_dist->has_negative_pull_since;
                if (!use_reference_order &&
                    test_dist->has_negative_pull_since ==
                        second_dist->has_negative_pull_since) {
                    IVP_ASSERT(test_dist->creation_sequence !=
                               second_dist->creation_sequence);
                    test_should_shift =
                        test_dist->creation_sequence >
                        second_dist->creation_sequence;
                }
                if (!test_should_shift) break;
                fs_exchange_friction_dists(fs, test_dist, second_dist);
            }
            second_dist = first_dist;
        }
        first_dist = second_dist;
    }
    IVP_IF(1) /* ivp_friction_gaps.cxx:1246 */
    {
        for (first_dist = fs->first_friction_dist; first_dist; first_dist = first_dist->next_dist_in_friction) {
            ivp_contact_pt_t *second_dist = first_dist->next_dist_in_friction;
            if (second_dist) {
                IVP_ASSERT(!(first_dist->has_negative_pull_since > second_dist->has_negative_pull_since));
                if (!use_reference_order &&
                    first_dist->has_negative_pull_since ==
                        second_dist->has_negative_pull_since) {
                    IVP_ASSERT(first_dist->creation_sequence <
                               second_dist->creation_sequence);
                }
            }
        }
    }
}

typedef int (*fs_order_compare_fn)(const void *left,
                                   const void *right);

static void fs_sort_vector(ivp_pvec_t *vector,
                           fs_order_compare_fn compare)
{
    for (int i = 1; i < ivp_pvec_len(vector); i++) {
        void *value = vector->elems[i];
        int position = i;
        while (position > 0 &&
               compare(vector->elems[position - 1], value) > 0) {
            vector->elems[position] = vector->elems[position - 1];
            position--;
        }
        vector->elems[position] = value;
    }
}

static int fs_compare_cores(const void *left, const void *right)
{
    uint64_t left_key =
        ivp_core_stable_order_key((const ivp_core_t *)left);
    uint64_t right_key =
        ivp_core_stable_order_key((const ivp_core_t *)right);
    IVP_ASSERT(left_key != right_key || left == right);
    return left_key < right_key ? -1 : left_key > right_key ? 1 : 0;
}

static int fs_compare_contacts(const void *left, const void *right)
{
    const ivp_contact_pt_t *left_contact =
        (const ivp_contact_pt_t *)left;
    const ivp_contact_pt_t *right_contact =
        (const ivp_contact_pt_t *)right;
    uint64_t left_key = left_contact->creation_sequence;
    uint64_t right_key = right_contact->creation_sequence;
    IVP_ASSERT(left_key != right_key || left == right);
    return left_key < right_key ? -1 : left_key > right_key ? 1 : 0;
}

static void fs_pair_order_key(const ivp_fr_core_pair_t *pair,
                              uint64_t *first, uint64_t *second)
{
    /* A core pair is unordered; its two stable core identities are the key. */
    uint64_t key0 = ivp_core_stable_order_key(pair->objs[0]);
    uint64_t key1 = ivp_core_stable_order_key(pair->objs[1]);
    if (key0 <= key1) {
        *first = key0;
        *second = key1;
    } else {
        *first = key1;
        *second = key0;
    }
}

static int fs_compare_pairs(const void *left, const void *right)
{
    uint64_t left_first, left_second;
    uint64_t right_first, right_second;
    fs_pair_order_key((const ivp_fr_core_pair_t *)left,
                      &left_first, &left_second);
    fs_pair_order_key((const ivp_fr_core_pair_t *)right,
                      &right_first, &right_second);
    if (left_first != right_first) {
        return left_first < right_first ? -1 : 1;
    }
    IVP_ASSERT(left_second != right_second || left == right);
    return left_second < right_second
               ? -1
               : left_second > right_second ? 1 : 0;
}

static void fs_restore_internal_order(ivp_fr_system_t *fs)
{
    /* Reconstruct every order from domain identities, never from allocation
     * addresses, physical slots, or a shadow of earlier container layout. */
    fs_bubble_sort_dists_importance(fs);
    fs_sort_vector(&fs->cores_of_friction_system, fs_compare_cores);
    fs_sort_vector(&fs->moveable_cores_of_friction_system,
                   fs_compare_cores);
    for (int i = 0; i < ivp_pvec_len(&fs->fr_pairs_of_objs); i++) {
        ivp_fr_core_pair_t *pair = pair_at(&fs->fr_pairs_of_objs, i);
        fs_sort_vector(&pair->fr_dists, fs_compare_contacts);
    }
    fs_sort_vector(&fs->fr_pairs_of_objs, fs_compare_pairs);
    for (int i = 0;
         i < ivp_pvec_len(&fs->cores_of_friction_system);
         i++) {
        ivp_core_t *core = core_at(&fs->cores_of_friction_system, i);
        for (int j = 0; j < ivp_pvec_len(&core->objects); j++) {
            ivp_friction_mgr_restore_object_contact_order(
                (ivp_object_t *)ivp_pvec_at(&core->objects, j));
        }
        ivp_fr_info_for_core_t *info =
            ivp_core_get_friction_info(core, fs);
        if (info) {
            fs_sort_vector(&info->friction_springs,
                           fs_compare_contacts);
        }
    }
}

static void fs_reorder_mindists_for_complex(ivp_fr_system_t *fs)
{
    int not_in_complex = 0;
    ivp_contact_pt_t *my_frdist = fs->first_friction_dist;
    while (my_frdist) {
        ivp_contact_pt_t *next_frdist = my_frdist->next_dist_in_friction;
        const ivp_mindist_settings_t *settings =
            ivp_env_get_mindist_settings(fs->l_environment);
        if ((ivp_cp_get_gap_length(my_frdist) >= settings->max_distance_for_friction) ||
            (my_frdist->tmp_contact_info->friction_is_broken)) {
            ivp_fr_system_delete_friction_distance(fs, my_frdist);
            my_frdist = next_frdist;
            continue;
        }
        if (ivp_cp_get_gap_length(my_frdist) >
            settings->friction_dist + settings->distance_keepers_safety) {
            ivp_core_t *core0 = cp_fcore(my_frdist, 0);
            ivp_core_t *core1 = cp_fcore(my_frdist, 1);
            bool allow0 = core0->fast_piling_allowed;
            bool allow1 = core1->fast_piling_allowed;
            if (allow0 & allow1) {
                ivp_fr_system_remove_dist_from_system(fs, my_frdist);
                ivp_fr_system_add_dist_to_system(fs, my_frdist);
            }
        }
        my_frdist = next_frdist;
    }
    fs->complex_not_necessary_number = (short)not_in_complex;
}

static bool fs_core_is_terminal_in_fs(ivp_fr_system_t *fs, ivp_core_t *test_core)
{
    int total_number_of_partners = 0;
    IVP_ASSERT(test_core->physical_unmoveable == false);
    for (int i = ivp_pvec_len(&fs->fr_pairs_of_objs) - 1; i >= 0; i--) {
        ivp_fr_core_pair_t *fcp = pair_at(&fs->fr_pairs_of_objs, i);
        if ((fcp->objs[0] == test_core) || (fcp->objs[1] == test_core)) {
            int unmov = (int)fcp->objs[0]->physical_unmoveable | (int)fcp->objs[1]->physical_unmoveable;
            if (!unmov) total_number_of_partners++;
        }
    }
    return total_number_of_partners > 1 ? false : true;
}

static void fs_static_fr_oversized_matrix_panic(ivp_fr_system_t *fs)
{
    for (int i = ivp_pvec_len(&fs->moveable_cores_of_friction_system) - 1; i >= 0; i--) {
        ivp_core_t *my_core = core_at(&fs->moveable_cores_of_friction_system, i);
        my_core->fast_piling_allowed = true;
        if (!fs_core_is_terminal_in_fs(fs, my_core)) {
            ivp_vec3r_set_zero(&my_core->speed);
            lvf_zero(&my_core->rot_speed);
        }
    }
}

/* IVP_Friction_System::do_friction_system: one transaction of the
 * environment's short-term arena */
static void fs_do_friction_system(ivp_fr_system_t *fs, const ivp_sim_es_t *es_in)
{
    ivp_arena_t *arena = ivp_env_get_short_term_mem(fs->l_environment);
    ivp_arena_begin_transaction(arena);
    int friction_debug_counter = 0;
#ifdef IVP_DEBUG_BUILD
    friction_debug_counter = ivp_env_next_friction_debug_counter(fs->l_environment);
#endif
    IVP_IF(fr_dm(fs->l_environment)->debug_friction) /* ivp_friction_gaps.cxx:1367 */
    {
        ivp_debug_printf("doing_friction_nr %d\n", friction_debug_counter);
        if (friction_debug_counter == 14) {
            ivp_debug_printf("losgehts\n");
        }
        ivp_debug_printf("friction_system_simulating %d distances\n", (int)fs->friction_dist_number);
    }
    if (!ivp_build_uses_reference_internal_state()) {
        fs_restore_internal_order(fs);
    } else {
        fs_bubble_sort_dists_importance(fs);
    }
    fs_reorder_mindists_for_complex(fs);
    if ((fs->friction_dist_number - fs->complex_not_necessary_number) > IVP_FR_MAX_FRICTION_MATRIX) {
        fs_static_fr_oversized_matrix_panic(fs);
        ivp_arena_end_transaction(arena);
        return;
    }
    fr_solver_t fr_solver;
    if (frs_init(&fr_solver, fs, es_in, arena)) {
        frs_setup_coords_mindists(&fr_solver, fs);
        int *original_pos_of_active =
            (int *)ivp_arena_alloc(
                arena, (size_t)fs->friction_dist_number * sizeof(int));
        if (original_pos_of_active) {
            do { /* while (IVP_TRUE) { ... break; } */
                int num_actives = frs_calc_solver_psi(&fr_solver, fs, original_pos_of_active);
                IVP_IF(fr_dm(fs->l_environment)->disable_friction) /* ivp_friction_gaps.cxx:1401 */
                {
                    break;
                }
                IVP_IF(1 == 0) /* ivp_friction_gaps.cxx:1410 */
                {
                    for (int j = 0; j < num_actives; j++) {
                        IVP_ASSERT(original_pos_of_active[j] >= 0);
                        IVP_ASSERT(original_pos_of_active[j] < fs->friction_dist_number);
                    }
                }
                frs_solve_linear_equation_and_push(
                    &fr_solver, fs, original_pos_of_active, num_actives,
                    arena);
            } while (0);
        }
    }
    frs_destroy(&fr_solver);
    ivp_arena_end_transaction(arena);
}

/* ── controllers ───────────────────────────────────────────────────────── */

/* IVP_Friction_Sys_Energy::do_simulation_controller */
static void fs_energy_do_simulation(ivp_sim_ctrl_t *c, ivp_sim_es_t *es, ivp_pvec_t *cores)
{
    (void)cores;
    ivp_fr_system_t *fs = FS_FROM(c, energy_ctrl);
    if (!ivp_build_uses_reference_internal_state()) {
        fs_restore_internal_order(fs);
    }
    if (fs->friction_dist_number <= 1) {
        ivp_contact_pt_t *cp = fs->first_friction_dist;
        if (cp) ivp_contact_pt_recalc_friction_s_vals(cp);
        return;
    }
    fs_recalc_all_contact_points(fs);
    if (es->sim_unit && es->sim_unit->sim_unit_just_slowed_down) {
        fs_clear_integrated_anti_energy(fs);
    }
    if (es->sim_unit && !es->sim_unit->sim_unit_has_fast_objects) {
        fs_remove_energy_gained_by_real_friction(fs);
    }
}

/* IVP_Friction_System::do_simulation_controller */
static void fs_dyn_do_simulation(ivp_sim_ctrl_t *c, ivp_sim_es_t *es, ivp_pvec_t *cores)
{
    (void)cores;
    ivp_fr_system_t *fs = FS_FROM(c, dyn_ctrl);
    if (!ivp_build_uses_reference_internal_state()) {
        fs_restore_internal_order(fs);
    }
    if (fs->friction_dist_number <= 1) {
        ivp_contact_pt_t *cp = fs->first_friction_dist;
        if (!cp) return;
        /* retail RVA 0x1D750 narrows only after both time factors */
        float max_local_spring_len = (float)((double)cp->inv_virt_mass_mindist_no_dir * (double)cp->now_friction_pressure *
                                             (double)cp->real_friction_factor * es->delta_time * es->delta_time);
        cp_calc_pretension(cp, max_local_spring_len);
        if (!cp->two_friction_values) {
            (void)cp_friction_force_local_constraint_2d(cp, es);
        } else {
            cp_friction_force_local_constraint_1d(cp, es);
        }
        return;
    }
    fs_apply_real_friction(fs, es);
}

/* IVP_Friction_Sys_Static::do_simulation_single_friction */
static void fs_static_do_single(ivp_fr_system_t *fs, ivp_sim_es_t *es)
{
    ivp_contact_pt_t *cp = fs->first_friction_dist;
    if (!cp) return;
    const ivp_mindist_settings_t *settings =
        ivp_env_get_mindist_settings(fs->l_environment);
    cp_static_friction_single(cp, es, settings->friction_dist, 1.0f);
    if ((ivp_cp_get_gap_length(cp) >= settings->max_distance_for_friction) ||
        cp->tmp_contact_info->friction_is_broken) {
        ivp_fr_system_delete_friction_distance(fs, cp);
    }
}

/* IVP_Friction_Sys_Static::do_simulation_controller */
static void fs_static_do_simulation(ivp_sim_ctrl_t *c, ivp_sim_es_t *es, ivp_pvec_t *cores)
{
    (void)cores;
    ivp_fr_system_t *fs = FS_FROM(c, static_ctrl);
    if (fs->friction_dist_number <= 1) {
        fs_static_do_single(fs, es);
    } else {
        fs_do_friction_system(fs, es);
    }
    if (fs->friction_dist_number == 0) {
        ivp_fr_system_destroy(fs);
        if (es->sim_unit) ivp_sim_unit_request_union_find(es->sim_unit);
        return;
    }
    if (fs->union_find_necessary) {
        fs->union_find_necessary = false;
        ivp_core_t *obj = fs_union_find_fr_sys(fs);
        if (obj) {
            fs_split_friction_system(fs, obj);
            if (obj->sim_unit) ivp_sim_unit_request_union_find(obj->sim_unit);
        }
    }
    IVP_IF(1) /* ivp_friction.cxx:2184 */
    {
        ivp_fr_system_debug_clean_tmp_info(fs);
    }
}

/* IVP_Friction_Sys_Static::core_is_going_to_be_deleted_event */
static void fs_static_core_deleted(ivp_sim_ctrl_t *c, ivp_core_t *del_core)
{
    ivp_fr_system_t *fs = FS_FROM(c, static_ctrl);
    IVP_ASSERT(!del_core->physical_unmoveable);
    IVP_ASSERT(ivp_env_get_state(del_core->environment) == IVP_ES_AT);
    for (int j = ivp_pvec_len(&fs->fr_pairs_of_objs) - 1; j >= 0; j--) {
        if (j >= ivp_pvec_len(&fs->fr_pairs_of_objs)) continue;
        ivp_fr_core_pair_t *my_pair = pair_at(&fs->fr_pairs_of_objs, j);
        if ((my_pair->objs[0] == del_core) || (my_pair->objs[1] == del_core)) {
            for (int k = ivp_pvec_len(&my_pair->fr_dists) - 1; k >= 0; k--) {
                ivp_contact_pt_t *fr_mindist = cp_at(&my_pair->fr_dists, k);
                bool last = (ivp_pvec_len(&my_pair->fr_dists) == 1);
                ivp_fr_system_delete_friction_distance(fs, fr_mindist);
                if (last) break;   /* pair struct was freed with its last dist */
            }
        }
    }
    if (fs->friction_dist_number == 0) {
        ivp_fr_system_destroy(fs);
    }
}

/* ── IVP_Core::grow_friction_system (ivp_friction.cxx) ─────────────────── */

bool ivp_core_grow_friction_system(ivp_core_t *core)
{
    IVP_IF(fr_dm(ivp_core_env(core))->check_fs) /* ivp_friction.cxx:2008 */
    {
        ivp_debug_printf("growing_fs %f core %p\n", ivp_env_get_current_time(ivp_core_env(core)), (void *)core);
    }
    bool grew_new_contact_point = false;
    ivp_core_t *start_core = core;
    for (int c = ivp_core_n_objects(core) - 1; c >= 0; c--) {
        ivp_object_t *r_obj = ivp_core_object_at(core, c);
        ivp_md_synapse_t *my_synapse, *next_synapse;
        for (my_synapse = r_obj->exact_synapses; my_synapse; my_synapse = next_synapse) {
            next_synapse = my_synapse->next;
            ivp_mindist_t *my_mindist = my_synapse->mindist;
            if (my_mindist->mindist_function != IVP_MF_COLLISION) continue;
            ivp_core_t *other_core = my_mindist->synapse[0].l_obj->physical_core;   /* get_core() */
            if (other_core == start_core) other_core = my_mindist->synapse[1].l_obj->physical_core;
            if (other_core->physical_unmoveable) {
                if (core->physical_unmoveable) {
                    ivp_collision_t *coll = &my_mindist->coll;
                    coll->cvt->destroy(coll);
                }
                continue;
            }
            ivp_fr_info_for_core_t *fr_info = ivp_core_moveable_core_has_friction_info(other_core);
            if (fr_info) continue;
            (void)ivp_mindist_recalc_mindist(my_mindist);
            if (my_mindist->recalc_result != IVP_MDRR_OK) continue;
            float mindist_distance = ivp_mindist_get_length(my_mindist);
            if (mindist_distance <
                ivp_env_get_mindist_settings(my_mindist->env)->max_distance_for_friction) {
                struct ivp_fr_system *affected_friction_system;
                bool having_new_dist;
                struct ivp_environment *env = ivp_core_env(core);
                ivp_env_sim_mem_begin(env);
                (void)ivp_mindist_try_to_generate_managed_friction(my_mindist, &affected_friction_system, &having_new_dist,
                                                                   core->sim_unit, true);
                ivp_env_sim_mem_end(env);
                if (having_new_dist) {
                    grew_new_contact_point = true;
                    ivp_core_reset_freeze_check_values(other_core);
                }
            }
        }
    }
    return grew_new_contact_point;
}

/* ── IVP_Friction_System debug methods ─────────────────────────────────── */

/* IVP_Friction_System::test_hole_fr_system_data (ivp_friction_gaps.cxx:682) */
void ivp_fr_system_test_hole_fr_system_data(ivp_fr_system_t *fs)
{
    IVP_IF(1)
    {
        ivp_contact_pt_t *mindist = fs->first_friction_dist;
        if (!mindist) return;
        /* "#if 0" checks of number_in_friction: not ported (off in the reference) */
        int two_count = 0;
        while (mindist) {
            for (int i = 0; i < 2; i++) {
                ivp_core_t *r_obj = mindist->synapse[i].l_obj->friction_core;
                ivp_object_t *obj_obj = ivp_core_object_at(r_obj, 0);
                ivp_time_t time = ivp_env_get_current_time(r_obj->environment);
                ivp_fr_info_for_core_t *fr_info = ivp_core_get_friction_info(r_obj, fs);
                if (fr_info->l_friction_system != fs) {
                    ivp_debug_printf("test_fr l_friction_system %f %p\n", time, (void *)obj_obj);
                    IVP_CORE();
                }
                int found_mine = 0;
                for (int j = ivp_pvec_len(&fr_info->friction_springs) - 1; j >= 0; j--) {
                    ivp_contact_pt_t *all_dists = cp_at(&fr_info->friction_springs, j);
                    if (all_dists == mindist) {
                        found_mine = 1;
                    } else {
                        int is_in_collection = 0;
                        ivp_contact_pt_t *temp_dist = fs->first_friction_dist;
                        while (temp_dist) {
                            if (temp_dist == all_dists) is_in_collection = 1;
                            temp_dist = temp_dist->next_dist_in_friction;
                        }
                        if (!is_in_collection) {
                            ivp_debug_printf("test_fr there was mindist %p in obj which is not in system\n",
                                             (void *)all_dists);
                            IVP_CORE();
                        }
                    }
                }
                if (!found_mine) {
                    ivp_debug_printf("test_fr mindist %p missing in obj\n", (void *)mindist);
                    IVP_CORE();
                }
            }
            {
                /* test if mindist is found in pairs */
                int found_mine = 0;
                for (int k = ivp_pvec_len(&fs->fr_pairs_of_objs) - 1; k >= 0; k--) {
                    ivp_fr_core_pair_t *fr_pair = pair_at(&fs->fr_pairs_of_objs, k);
                    for (int n = ivp_pvec_len(&fr_pair->fr_dists) - 1; n >= 0; n--) {
                        if (cp_at(&fr_pair->fr_dists, n) == mindist) found_mine = 1;
                    }
                }
                if (!found_mine) {
                    ivp_debug_printf("test_fr dist %p not found in pairs\n", (void *)mindist);
                    IVP_CORE();
                }
            }
            two_count += 2;
            mindist = mindist->next_dist_in_friction;
        }
        IVP_USE(two_count);
        {
            /* test if all dists in pairs are in system */
            for (int n = ivp_pvec_len(&fs->fr_pairs_of_objs) - 1; n >= 0; n--) {
                ivp_fr_core_pair_t *fr_pair = pair_at(&fs->fr_pairs_of_objs, n);
                for (int m = ivp_pvec_len(&fr_pair->fr_dists) - 1; m >= 0; m--) {
                    ivp_contact_pt_t *fr_dist = cp_at(&fr_pair->fr_dists, m);
                    int found_mine = 0;
                    for (ivp_contact_pt_t *mindist2 = fs->first_friction_dist; mindist2;
                         mindist2 = mindist2->next_dist_in_friction) {
                        if (mindist2 == fr_dist) found_mine = 1;
                    }
                    if (!found_mine) {
                        ivp_debug_printf("test_fr pair %p containing dist %p not in system\n", (void *)fr_pair,
                                         (void *)fr_dist);
                        IVP_CORE();
                    }
                }
            }
        }
    }
}

/* IVP_Friction_System::debug_clean_tmp_info (ivp_impact.cxx:1411) */
void ivp_fr_system_debug_clean_tmp_info(ivp_fr_system_t *fs)
{
    for (ivp_contact_pt_t *my_dist = fs->first_friction_dist; my_dist; my_dist = my_dist->next_dist_in_friction) {
        my_dist->tmp_contact_info = NULL;
    }
}

/* IVP_Friction_System::debug_check_system_consistency (ivp_friction.cxx:1217):
 * all cores of a simulated friction system have to be simulated */
void ivp_fr_system_debug_check_system_consistency(ivp_fr_system_t *fs)
{
    for (int i = ivp_pvec_len(&fs->cores_of_friction_system) - 1; i >= 0; i--) {
        ivp_core_t *core = core_at(&fs->cores_of_friction_system, i);
        if (!core->physical_unmoveable) {
            IVP_ASSERT(IVP_MTIS_SIMULATED(core->movement_state));
        }
    }
}

/* IVP_Friction_System::print_all_dists (ivp_friction.cxx:1498) */
void ivp_fr_system_print_all_dists(ivp_fr_system_t *fs)
{
    IVP_IF(1)
    {
        ivp_debug_printf("fs %u  ", fr_p16(fs));
        for (ivp_contact_pt_t *mindist = fs->first_friction_dist; mindist; mindist = mindist->next_dist_in_friction) {
            ivp_debug_printf("%u ", fr_p16(mindist));
        }
        ivp_debug_printf("\n");
        ivp_debug_printf("      ");
        for (int i = ivp_pvec_len(&fs->fr_pairs_of_objs) - 1; i >= 0; i--) {
            ivp_fr_core_pair_t *fr_pair = pair_at(&fs->fr_pairs_of_objs, i);
            ivp_debug_printf("p %u %u  ", fr_p16(fr_pair->objs[0]), fr_p16(fr_pair->objs[1]));
            for (int c = ivp_pvec_len(&fr_pair->fr_dists) - 1; c >= 0; c--) {
                ivp_debug_printf("%u ", fr_p16(cp_at(&fr_pair->fr_dists, c)));
            }
        }
        ivp_debug_printf("\n");
    }
}

/* IVP_Friction_System::debug_fs_out_ascii (ivp_friction.cxx:1877) */
void ivp_fr_system_debug_fs_out_ascii(ivp_fr_system_t *fs)
{
    IVP_IF(1)
    {
        ivp_debug_printf("fs %u  ", fr_p16(fs));
        for (ivp_contact_pt_t *mindist = fs->first_friction_dist; mindist; mindist = mindist->next_dist_in_friction) {
            ivp_debug_printf("%u ", fr_p16(mindist));
        }
        ivp_debug_printf("\n");
        for (int k = ivp_pvec_len(&fs->cores_of_friction_system) - 1; k >= 0; k--) {
            ivp_core_t *my_core = core_at(&fs->cores_of_friction_system, k);
            ivp_debug_printf("    core %u  ", fr_p16(my_core));
            ivp_fr_info_for_core_t *inf = ivp_core_get_friction_info(my_core, fs);
            ivp_debug_printf("lfs %u  ", fr_p16(inf->l_friction_system));
            for (int i = ivp_pvec_len(&inf->friction_springs) - 1; i >= 0; i--) {
                ivp_debug_printf("%u  ", fr_p16(cp_at(&inf->friction_springs, i)));
            }
        }
        ivp_debug_printf("\n");
        for (int m = ivp_pvec_len(&fs->fr_pairs_of_objs) - 1; m >= 0; m--) {
            ivp_fr_core_pair_t *fr_pair = pair_at(&fs->fr_pairs_of_objs, m);
            ivp_debug_printf("    p %u %u  ", fr_p16(fr_pair->objs[0]), fr_p16(fr_pair->objs[1]));
            for (int c = ivp_pvec_len(&fr_pair->fr_dists) - 1; c >= 0; c--) {
                ivp_debug_printf("%u ", fr_p16(cp_at(&fr_pair->fr_dists, c)));
            }
        }
        ivp_debug_printf("\n");
    }
}

/* IVP_Friction_System::ivp_debug_fs_pointers (ivp_friction_gaps.cxx:1200) */
void ivp_fr_system_ivp_debug_fs_pointers(ivp_fr_system_t *fs)
{
    IVP_IF(1)
    {
        ivp_debug_printf("%u  ", fr_p16(fs->first_friction_dist));
        for (ivp_contact_pt_t *fr_d = fs->first_friction_dist; fr_d; fr_d = fr_d->next_dist_in_friction) {
            ivp_debug_printf("%u %u %d %u  ", fr_p16(fr_d->prev_dist_in_friction), fr_p16(fr_d),
                             fr_d->has_negative_pull_since, fr_p16(fr_d->next_dist_in_friction));
        }
        ivp_debug_printf("\n");
    }
}

/* out_friction_info_obj (ivp_friction_gaps.cxx:927) returns at once in the
 * reference ("absturz").  Its dead diagnostic block is intentionally omitted. */
void ivp_fr_out_friction_info_obj(ivp_core_t *obj)
{
    (void)obj;
}

/* debug_out_friction_system (ivp_friction_gaps.cxx:1084) is not ported: an
 * unused release function that hits CORE before its IVP_IF(0) draw-vector
 * block (:1118), which needs the removed calc_object_coords. */
