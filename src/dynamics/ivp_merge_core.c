/* ivp_merge_core.c -- port of IVP_Core_Merged / IVP_Merge_Core
 * (ivp_intern/ivp_merge_core.{hxx,cxx}), IVP_Core_Merged's constructors,
 * IVP_Core::set_matrizes_and_speed / create_collision_merged_core_with /
 * IVP_Core_Collision::split_collision_merged_core_next_PSI
 * (ivp_physics/ivp_core.cxx) and IVP_Environment::merge_objects
 * (ivp_physics/ivp_environment.cxx).
 *
 * Precision follows the reference types: IVP_Core::speed / rot_speed /
 * rot_inertia / damp factors are IVP_FLOAT (float values in libivp's
 * fields), matrices and positions IVP_DOUBLE.
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <ivp/base/ivp_assert.h>
#include <ivp/dynamics/ivp_merge_core.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_transform.h>
#include "collision/ivp_lvec.h"
#include "dynamics/ivp_env_pipeline.h"
#include "dynamics/ivp_env_hot.h"

/* ── IVP_Merge_Core ────────────────────────────────────────────────────── */

typedef struct merge_core {
    int n_cores;
    ivp_core_t **cores;
    ivp_core_t *unmovable_core;        /* an unmoveable core, if there is one */
    ivp_movement_type_t movement_type; /* ored movement */
    ivp_core_merged_t *mcore;          /* destination core */
} merge_core_t;

static inline float core_mass_f(const ivp_core_t *c) { return (float)c->mass; }   /* get_mass() */

/* sets m_world_f_core_last_psi.vv (pos_world_f_core_last_psi keeps the
 * position copied by find_main_axis) */
static void mc_find_mass_center(merge_core_t *m)
{
    ivp_core_t *mcore = &m->mcore->core;
    if (m->unmovable_core) {
        mcore->m_world_f_core_last_psi.pos = m->unmovable_core->m_world_f_core_last_psi.pos;
    } else {
        ivp_vec3r_t center;
        lvp_zero(&center);
        D mass_sum = (D)P_DOUBLE_EPS;
        for (int i = 0; i < m->n_cores; i++) {
            ivp_core_t *core = m->cores[i];
            D mass = (D)core_mass_f(core);
            lvp_add_multiple(&center, &core->m_world_f_core_last_psi.pos, mass);
            mass_sum += mass;
        }
        lvp_set_multiple(&mcore->m_world_f_core_last_psi.pos, &center, (D)1.0f / mass_sum);
    }
}

static void mc_find_damp_factors(merge_core_t *m)
{
    if (m->unmovable_core) return;
    ivp_core_t *mcore = &m->mcore->core;
    float speed_damp_factor = (float)mcore->speed_damp_factor;   /* IVP_FLOAT */
    ivp_vec3f_t rot_speed_damp_factor;
    lvf_set_p(&rot_speed_damp_factor, &mcore->rot_speed_damp_factor);
    D mass_sum = (D)P_DOUBLE_EPS;
    for (int i = 0; i < m->n_cores; i++) {
        ivp_core_t *core = m->cores[i];
        D mass = (D)core_mass_f(core);
        speed_damp_factor += (float)((D)(float)core->speed_damp_factor * mass);
        ivp_vec3f_t f;
        lvf_set_p(&f, &core->rot_speed_damp_factor);
        lvf_add_multiple(&rot_speed_damp_factor, &f, mass);
        mass_sum += (D)core_mass_f(core);
    }
    D inv_mass = (D)1.0f / mass_sum;
    speed_damp_factor *= (float)inv_mass;
    lvf_mult(&rot_speed_damp_factor, inv_mass);
    mcore->speed_damp_factor = (ivp_real_t)speed_damp_factor;
    lvp_set_f(&mcore->rot_speed_damp_factor, &rot_speed_damp_factor);
}

/* sets m_world_f_core_last_psi (and q / pos of the last PSI) */
static void mc_find_main_axis(merge_core_t *m)
{
    ivp_core_t *mcore = &m->mcore->core;
    ivp_core_t *c;
    if (m->unmovable_core) {
        c = m->unmovable_core;
    } else {
        int max_i = 0;
        D max_mass = (D)0.0f;
        for (int i = 0; i < m->n_cores; i++) {   /* simply search maximum mass */
            ivp_core_t *core = m->cores[i];
            if ((D)core_mass_f(core) > max_mass) {
                max_mass = (D)core_mass_f(core);
                max_i = i;
            }
        }
        c = m->cores[max_i];
    }
    mcore->m_world_f_core_last_psi = c->m_world_f_core_last_psi;
    mcore->q_world_f_core_last_psi = c->q_world_f_core_last_psi;
    mcore->pos_world_f_core_last_psi = c->pos_world_f_core_last_psi;
    ivp_vec3r_set_zero(&mcore->delta_world_f_core_psis);
}

static void mc_set_radius(merge_core_t *m)
{
    ivp_core_t *mcore = &m->mcore->core;
    D max_radius = (D)0.0f;
    for (int i = 0; i < m->n_cores; i++) {
        ivp_core_t *core = m->cores[i];
        D radius = lvp_quad_distance(&core->m_world_f_core_last_psi.pos, &mcore->m_world_f_core_last_psi.pos);
        radius = (D)sqrt((double)radius);                       /* IVP_Inline_Math::sqrtd */
        radius += (D)(float)core->upper_limit_radius;
        if (radius > max_radius) max_radius = radius;
    }
    mcore->upper_limit_radius = (ivp_real_t)(float)max_radius;
    mcore->max_surface_deviation = (ivp_real_t)(float)max_radius;
}

static void mc_set_speed(merge_core_t *m)
{
    ivp_core_t *mcore = &m->mcore->core;
    if (m->unmovable_core) {
        lvf_zero(&mcore->rot_speed);
        ivp_vec3r_set(&mcore->rot_inertia, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
        ivp_vec3r_set_zero(&mcore->speed);
        return;
    }
    ivp_vec3f_t sum_moment, sum_rot_inertia, sum_rot_inertia_speed;
    D mass_sum = (D)0.0f;
    lvf_zero(&sum_moment);
    lvf_zero(&sum_rot_inertia);
    lvf_zero(&sum_rot_inertia_speed);
    const ivp_transform_r_t *m_mcore = &mcore->m_world_f_core_last_psi;
    for (int i = 0; i < m->n_cores; i++) {
        ivp_core_t *core = m->cores[i];
        mass_sum += (D)core_mass_f(core);
        ivp_vec3f_t speed = ivp_core_speed_f(core);
        lvf_add_multiple(&sum_moment, &speed, (D)core_mass_f(core));
        ivp_vec3f_t ri = ivp_core_rot_inertia_f(core);
        {
            ivp_vec3f_t quad_sum_rot_inertia;
            lvf_zero(&quad_sum_rot_inertia);
            const ivp_transform_r_t *mm = &core->m_world_f_core_last_psi;
            /* add spin momentum by original spin momentum */
            for (int s = 0; s < 3; s++) {   /* s for source */
                ivp_vec3f_t world_achse;
                lvf_setd3(&world_achse, LVM_E(mm, 0, s), LVM_E(mm, 1, s), LVM_E(mm, 2, s));   /* get_col */
                ivp_vec3f_t projected_achse;   /* spin of old axis projected to new axis */
                lvm_vimult3_f(m_mcore, &world_achse, &projected_achse);
                lvf_add_multiple(&sum_rot_inertia_speed, &projected_achse, (D)(ri.k[s] * core->rot_speed.k[s]));
                lvf_set_pairwise_mult(&projected_achse, &projected_achse, &projected_achse);
                D rot_inertia_for_this_axis = (D)ri.k[s];
                lvf_add_multiple(&quad_sum_rot_inertia, &projected_achse, rot_inertia_for_this_axis * rot_inertia_for_this_axis);
            }
            for (int k = 0; k < 3; k++) {   /* line_sqrt: ivp_sqrtf */
                quad_sum_rot_inertia.k[k] = (float)sqrt((double)quad_sum_rot_inertia.k[k]);
            }
            lvf_add(&sum_rot_inertia, &quad_sum_rot_inertia);
        }
        /* add spin momentum by different mass centers */
        {
            ivp_vec3f_t m_center_in_mcore, speed_in_mcore, distances_2d;
            lvm_vimult4_pf(m_mcore, &core->m_world_f_core_last_psi.pos, &m_center_in_mcore);
            lvm_vimult3_f(m_mcore, &speed, &speed_in_mcore);
            lvf_cross(&distances_2d, &m_center_in_mcore, &speed_in_mcore);   /* see test_push_core */
            lvf_add_multiple(&sum_rot_inertia_speed, &distances_2d, (D)core_mass_f(core));
            ivp_vec3f_t quad_center;
            lvf_set_pairwise_mult(&quad_center, &m_center_in_mcore, &m_center_in_mcore);
            /* IVP_FLOAT += sqrtd(float + float) * mass (double) */
            sum_rot_inertia.k[0] = (float)((double)sum_rot_inertia.k[0] +
                                           sqrt((double)(quad_center.k[1] + quad_center.k[2])) * (double)core_mass_f(core));
            sum_rot_inertia.k[1] = (float)((double)sum_rot_inertia.k[1] +
                                           sqrt((double)(quad_center.k[0] + quad_center.k[2])) * (double)core_mass_f(core));
            sum_rot_inertia.k[2] = (float)((double)sum_rot_inertia.k[2] +
                                           sqrt((double)(quad_center.k[0] + quad_center.k[1])) * (double)core_mass_f(core));
        }
    }
    ivp_vec3f_t mspeed;
    lvf_set_multiple(&mspeed, &sum_moment, (D)1.0f / mass_sum);
    ivp_core_set_speed_f(mcore, &mspeed);
    ivp_vec3f_t inv_rot_inertia;
    for (int d = 0; d < 3; d++) inv_rot_inertia.k[d] = 1.0f / sum_rot_inertia.k[d];   /* d for destination */
    lvp_set_f(&mcore->rot_inertia, &sum_rot_inertia);
    mcore->mass = (ivp_real_t)(float)mass_sum;
    lvf_set_pairwise_mult(&mcore->rot_speed, &sum_rot_inertia_speed, &inv_rot_inertia);
}

static void mc_synchronize_motion(merge_core_t *m)
{
    struct ivp_environment *env = ivp_core_env(&m->mcore->core);
    double d_time = ivp_time_diff(ivp_env_get_current_time(env), ivp_env_get_old_time_of_last_psi(env));
    if (d_time > 0.0) {   /* not within PSI */
        for (int i = 0; i < m->n_cores; i++) {
            ivp_core_t *core = m->cores[i];
            if (IVP_MTIS_SIMULATED(core->movement_state)) ivp_core_synchronize_with_rot_z(core);
        }
    }
}

static void mc_check_for_unmovable_core(merge_core_t *m)
{
    ivp_core_t *mcore = &m->mcore->core;
    m->unmovable_core = NULL;
    mcore->fast_piling_allowed = true;
    m->movement_type = IVP_MT_NOT_SIM;
    for (int i = 0; i < m->n_cores; i++) {
        ivp_core_t *core = m->cores[i];
        if (!core->fast_piling_allowed) mcore->fast_piling_allowed = false;
        if (core->physical_unmoveable) {
            m->unmovable_core = core;
        } else if (core->movement_state < IVP_MT_NOT_SIM) {
            m->movement_type = core->movement_state;
        }
    }
    mcore->movement_state = IVP_MT_NOT_SIM;
    if (m->unmovable_core) {
        mcore->movement_state = IVP_MT_STATIC;
        mcore->physical_unmoveable = true;
    } else {
        mcore->physical_unmoveable = false;
    }
}

/* set_new_m_object_f_core of all objects of the merged cores */
static void mc_place_objects(merge_core_t *m)
{
    ivp_core_t *mcore = &m->mcore->core;
    for (int i = 0; i < m->n_cores; i++) {
        ivp_core_t *core = m->cores[i];
        ivp_transform_r_t m_ncore_f_core;
        ivp_transform_inv_compose(&m_ncore_f_core, &mcore->m_world_f_core_last_psi, &core->m_world_f_core_last_psi);
        for (int o = ivp_core_n_objects(core) - 1; o >= 0; o--) {
            ivp_object_t *obj = ivp_core_object_at(core, o);
            ivp_transform_r_t m_core_f_object, m_ncore_f_object, m_object_f_ncore;
            ivp_object_calc_m_core_f_object(obj, &m_core_f_object);
            ivp_transform_compose(&m_ncore_f_object, &m_ncore_f_core, &m_core_f_object);
            ivp_transform_invert(&m_object_f_ncore, &m_ncore_f_object);   /* set_transpose */
            ivp_object_set_new_m_object_f_core(obj, &m_object_f_ncore);
        }
    }
}

/* IVP_Merge_Core::calc_calc */
static void mc_calc_calc(merge_core_t *m)
{
    mc_check_for_unmovable_core(m);
    mc_synchronize_motion(m);
    mc_find_main_axis(m);   /* sets m_world_f_core_last_psi.mm */
    mc_find_mass_center(m); /* sets m_world_f_core_last_psi.vv */
    mc_place_objects(m);    /* set new_m_object_f_core of all objects */
    m->mcore->m_world_f_core_when_created = m->mcore->core.m_world_f_core_last_psi;
    mc_find_damp_factors(m);
    mc_set_speed(m);
    mc_set_radius(m);
    ivp_core_calc_calc(&m->mcore->core);
}

/* ── IVP_Core_Merged ───────────────────────────────────────────────────── */

/* IVP_Core_Merged::IVP_Core_Merged(IVP_Real_Object *real_obj): an IVP_Core
 * initialized from the object (sim unit, gravity controller), without the
 * object in its object list */
static ivp_core_merged_t *core_merged_create_from_object(ivp_object_t *real_obj)
{
    ivp_core_merged_t *mc = (ivp_core_merged_t *)malloc(sizeof(ivp_core_merged_t));
    if (!mc) return NULL;
    if (!ivp_core_init_from_object(&mc->core, real_obj)) {
        free(mc);
        return NULL;
    }
    memset(&mc->m_world_f_core_when_created, 0, sizeof(mc->m_world_f_core_when_created));
    ivp_pvec_remove(&mc->core.objects, real_obj);
    return mc;
}

/* IVP_Core_Merged::set_by_merge(c0, c1) (note: does not call calc_next_PSI_matrix) */
void ivp_core_merged_set_by_merge(ivp_core_merged_t *mcore, ivp_core_t *c0, ivp_core_t *c1)
{
    ivp_core_t *cores[2] = { c0, c1 };
    merge_core_t merge;
    merge.n_cores = 2;
    merge.cores = cores;
    merge.unmovable_core = NULL;
    merge.movement_type = IVP_MT_UNDEFINED;
    merge.mcore = mcore;
    mc_calc_calc(&merge);
}

/* IVP_Core_Merged::IVP_Core_Merged(IVP_Core *core0, IVP_Core *core1).  Only
 * IVP_Core_Collision uses it, which the reference never creates
 * (create_collision_merged_core_with returns first).  The next-PSI
 * quaternion starts from q_world_f_core_last_psi before the first
 * calc_next_PSI_matrix (reference 8ca7bb2). */
ivp_core_merged_t *ivp_core_merged_create_from_cores(ivp_core_t *core0, ivp_core_t *core1)
{
    ivp_object_t *io = ivp_core_object_at(core0, 0);
    ivp_core_merged_t *mc = (ivp_core_merged_t *)malloc(sizeof(ivp_core_merged_t));
    if (!mc) return NULL;
    if (!ivp_core_init_from_object(&mc->core, io)) {
        free(mc);
        return NULL;
    }
    memset(&mc->m_world_f_core_when_created, 0, sizeof(mc->m_world_f_core_when_created));
    /* clear the objects added by the base constructor (set_by_merge does not re-add them) */
    ivp_pvec_remove_all(&mc->core.objects);
    mc->core.physical_unmoveable = core0->physical_unmoveable || core1->physical_unmoveable;
    mc->core.environment = core0->environment;
    mc->core.movement_state = IVP_MT_MOVING;
    ivp_core_merged_set_by_merge(mc, core0, core1);
    mc->core.q_world_f_core_next_psi = mc->core.q_world_f_core_last_psi;
    ivp_sim_es_t es;
    ivp_sim_es_init(&es, mc->core.environment);
    ivp_cnps_calc_next_psi_matrix(&mc->core, &es, NULL);
    return mc;
}

/* ── IVP_Core splitting / collision cores (ivp_core.cxx) ───────────────── */

/* IVP_Core::create_collision_merged_core_with: the reference returns right
 * after its assertion (the IVP_Core_Collision code below the return is
 * unreachable), so collision cores are never created */
void ivp_core_create_collision_merged_core_with(ivp_core_t *core, ivp_core_t *other_core)
{
    IVP_ASSERT(other_core != core);
    (void)core;
    (void)other_core;
}

void ivp_core_set_matrizes_and_speed(ivp_core_t *core, ivp_core_merged_t *template_core,
                                     ivp_transform_r_t *m_CORE_f_core_out)
{
    /* Convention: all matrix names for the current time are in capital letters */
    IVP_ASSERT(template_core->core.merged_core_which_replace_this_core == NULL);
    /* sum up m_world_now_f_world_when_merged */
    ivp_transform_r_t m_WORLD_f_world;
    if (template_core == core->merged_core_which_replace_this_core) {
        ivp_transform_mi2mult(&m_WORLD_f_world, &template_core->core.m_world_f_core_last_psi,
                              &template_core->m_world_f_core_when_created);
    } else {
        ivp_transform_identity(&m_WORLD_f_world);
        for (ivp_core_merged_t *mc = core->merged_core_which_replace_this_core; mc;
             mc = mc->core.merged_core_which_replace_this_core) {
            ivp_transform_r_t m_WORLD_f_world_one_core;
            ivp_transform_mi2mult(&m_WORLD_f_world_one_core, &mc->core.m_world_f_core_last_psi,
                                  &mc->m_world_f_core_when_created);
            ivp_transform_compose(&m_WORLD_f_world, &m_WORLD_f_world_one_core, &m_WORLD_f_world);
        }
    }
    (void)ivp_mat3r_orthonormalize(&m_WORLD_f_world.rot);
    /* calc m_CORE_f_core */
    ivp_transform_r_t m_CORE_f_core;
    {
        ivp_transform_r_t m_WORLD_f_core;
        ivp_transform_compose(&m_WORLD_f_core, &m_WORLD_f_world, &core->m_world_f_core_last_psi);
        ivp_transform_inv_compose(&m_CORE_f_core, &core->m_world_f_core_last_psi, &m_WORLD_f_core);
    }
    *m_CORE_f_core_out = m_CORE_f_core;
    /* set matrices */
    ivp_transform_compose(&core->m_world_f_core_last_psi, &m_WORLD_f_world, &core->m_world_f_core_last_psi);
    /* set speed + rot_speed */
    core->speed = template_core->core.speed;
    ivp_vec3f_t rot_speed_world;
    lvm_vmult3_f(&template_core->core.m_world_f_core_last_psi, &template_core->core.rot_speed, &rot_speed_world);
    lvm_vimult3_f(&core->m_world_f_core_last_psi, &rot_speed_world, &core->rot_speed);
}

/* IVP_Core_Collision::split_collision_merged_core_next_PSI: returns before
 * doing anything in the reference ("return; //@@@@@", the old split code is
 * #if 0) */
void ivp_core_split_collision_merged_core_next_psi(ivp_core_merged_t *ccore)
{
    (void)ccore;
}

/* ── IVP_Environment::merge_objects ────────────────────────────────────── */

bool ivp_environment_merge_objects(ivp_environment_t *env, ivp_object_t *const *objs_to_merge, int n_objs)
{
    if (!env || !objs_to_merge || n_objs <= 0) return false;
    /* input contract (reference 8ad306e, see ivp_merge_core.h) */
    for (int i = 0; i < n_objs; i++) {
        ivp_object_t *obj = objs_to_merge[i];
        if (!obj || obj->owner != env || !obj->physical_core) return false;
        ivp_core_t *core = obj->physical_core;
        if (ivp_core_n_objects(core) != 1 || obj->friction_core != core || obj->original_core != core) return false;
        if (IVP_MTIS_SIMULATED(core->movement_state)) return false;
        if (obj->fr_synapses) return false;
        for (int k = 0; k < i; k++) {
            if (objs_to_merge[k] == obj) return false;   /* the same core twice */
        }
    }

    merge_core_t merge_all_objs;   /* IVP_Merge_Core() */
    merge_all_objs.unmovable_core = NULL;
    merge_all_objs.movement_type = IVP_MT_UNDEFINED;
    merge_all_objs.n_cores = n_objs;
    ivp_object_t *my_obj = objs_to_merge[0];
    ivp_core_merged_t *merged_core_for_all = core_merged_create_from_object(my_obj);
    if (!merged_core_for_all) return false;
    merge_all_objs.mcore = merged_core_for_all;
    merge_all_objs.cores = (ivp_core_t **)malloc((size_t)n_objs * sizeof(ivp_core_t *));
    if (!merge_all_objs.cores) {
        ivp_core_delete(&merged_core_for_all->core);
        return false;
    }
    for (int i = 0; i < n_objs; i++) {
        my_obj = objs_to_merge[i];
        /* if (IVP_MTIS_SIMULATED(old_core->movement_state)): the reference has
         * the stop_physical_movement / remove_sim_core calls commented out */
        merge_all_objs.cores[i] = my_obj->physical_core;
        my_obj->physical_core = &merged_core_for_all->core;
        my_obj->friction_core = &merged_core_for_all->core;
        my_obj->original_core = &merged_core_for_all->core;
        ivp_core_core_add_link_to_obj(&merged_core_for_all->core, my_obj);
    }

    mc_calc_calc(&merge_all_objs);   /* now the merged core is valid */

    ivp_sim_es_t es;
    ivp_sim_es_init_dt(&es, env, ivp_time_diff(ivp_env_get_next_psi_time(env), ivp_env_get_current_time(env)));
    /* the movement type is never simulated here (simulated cores are
     * rejected above) */
    IVP_ASSERT(!IVP_MTIS_SIMULATED(merge_all_objs.movement_type));
    ivp_core_calc_next_psi_matrix_zero_speed(&merged_core_for_all->core, &es);

    for (int i = 0; i < n_objs; i++) ivp_core_delete(merge_all_objs.cores[i]);   /* delete cores[i] */
    free(merge_all_objs.cores);

    /* libivp "all cores" controllers are attached to every new core */
    ivp_env_attach_default_controllers(env, &merged_core_for_all->core);
    return true;
}
