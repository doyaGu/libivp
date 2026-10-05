/* ivp_object_attach.c -- port of IVP_Object_Attach
 * (ivp_physics/ivp_object_attach.cxx)
 */
#include <math.h>
#include <limits.h>
#include <string.h>

#include <ivp/base/ivp_assert.h>
#include <ivp/collision/ivp_hull_mgr.h>
#include <ivp/dynamics/ivp_object_attach.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_transform.h>
#include "collision/ivp_lvec.h"
#include "dynamics/ivp_env_pipeline.h"
#include "dynamics/ivp_env_hot.h"

void ivp_object_attach_object(ivp_object_t *parent, ivp_object_t *attached_object,
                              double max_distance_attached_object_to_parent)
{
    if (!parent || !attached_object || !parent->physical_core || !attached_object->physical_core) return;
    /* NULL and self attachment are ignored (reference f105953) */
    if (parent == attached_object) return;
    ivp_core_t *old_core = attached_object->physical_core;   /* get_core() */
    ivp_core_t *ncore = parent->physical_core;
    if (old_core != ncore) {
        int n_objects = ivp_core_n_objects(ncore);
        if (n_objects == INT_MAX || !ivp_pvec_reserve(&ncore->objects, n_objects + 1)) return;
    }
    struct ivp_environment *env = parent->owner;
    D max_dist = (D)max_distance_attached_object_to_parent;
    if (max_dist < (D)0.0f) {
        /* take current distance */
        ivp_time_t time = ivp_env_get_current_time(env);
        ivp_vec3r_t parents_mass_center, attached_mass_center;
        ivp_core_inline_calc_at_position(parent->physical_core, time, &parents_mass_center);
        ivp_core_inline_calc_at_position(attached_object->physical_core, time, &attached_mass_center);
        max_dist = (D)sqrt((double)lvp_quad_distance(&parents_mass_center, &attached_mass_center));
    }

    /* make sure both objects share the same state */
    if (IVP_MTIS_SIMULATED(parent->object_movement_state)) {
        if (!IVP_MTIS_SIMULATED(attached_object->object_movement_state)) {
            ivp_object_revive_object_for_simulation(attached_object);
        }
    } else {
        if (IVP_MTIS_SIMULATED(attached_object->object_movement_state)) {
            ivp_object_revive_object_for_simulation(parent);
        }
    }

    ivp_quatr_t q_world_f_object;
    ivp_vec3r_t shift_world_f_object;
    ivp_object_get_quat_world_f_object_AT(attached_object, &q_world_f_object, &shift_world_f_object);

    /* update radius (IVP_DOUBLE sums, IVP_FLOAT core fields) */
    D radius = max_dist + (D)(float)old_core->upper_limit_radius;
    if (radius > (D)(float)ncore->upper_limit_radius) ncore->upper_limit_radius = (ivp_real_t)(float)radius;
    D max_surface_deviation = max_dist + (D)(float)old_core->max_surface_deviation;
    if (max_surface_deviation > (D)(float)ncore->max_surface_deviation) {
        ncore->max_surface_deviation = (ivp_real_t)(float)max_surface_deviation;
    }

    /* exchange cores */
    ivp_core_unlink_obj_from_core_and_maybe_destroy(old_core, attached_object);
    attached_object->physical_core = ncore;
    attached_object->friction_core = ncore;
    attached_object->original_core = ncore;
    bool linked = ivp_core_core_add_link_to_obj(ncore, attached_object);
    IVP_ASSERT(linked);
    if (!linked) return;   /* capacity was reserved above (or released by unlinking this core) */
    attached_object->object_movement_state = parent->object_movement_state;

    /* position object */
    (void)ivp_object_reposition_object_Ros(NULL, attached_object, &q_world_f_object, &shift_world_f_object, false);

    /* now update radius */
    ivp_core_values_changed_recalc_redundants(ncore);
}

void ivp_object_detach_object(ivp_object_t *attached_object, const ivp_object_template_t *t)
{
    if (!attached_object || !t || !attached_object->physical_core) return;
    struct ivp_environment *env = attached_object->owner;
    ivp_core_t *old_core = attached_object->physical_core;

    /* orientation */
    ivp_quatr_t q_world_f_object;
    ivp_vec3r_t position_ws;
    ivp_object_get_quat_world_f_object_AT(attached_object, &q_world_f_object, &position_ws);

    /* velocity (IVP_U_Float_Point) */
    ivp_vec3r_t speed_ws;
    ivp_core_get_surface_speed_ws(old_core, &position_ws, &speed_ws);

    /* angular velocity */
    ivp_vec3f_t rot_speed_cs = old_core->rot_speed;

    /* get the old center offset */
    ivp_vec3f_t shift;
    lvf_set_p(&shift, &attached_object->shift_core_f_object);
    D moved_distance_val = lvf_real_length(&shift);

    /* Allocate the replacement before changing the old core.  The new core
     * is not observable through the object until the old links are gone. */
    ivp_core_t *core = ivp_core_create(attached_object, &q_world_f_object, &position_ws,
                                       false /* physical_unmoveable */, t->enable_piling_optimization);
    if (!core) return;
    if (!ivp_env_attach_default_controllers(env, core)) {
        ivp_core_delete(core);
        return;
    }

    /* get rid of all contact points as they reference the old core */
    ivp_object_unlink_contact_points(attached_object, true);

    /* reset object values */
    if (t->nocoll_group_ident && t->nocoll_group_ident[0]) {
        /* IVP_Template_Real_Object::set_nocoll_group_ident: CORE for an id
         * that is too long (see ivp_object_pipeline_create) */
        if (strlen(t->nocoll_group_ident) >= sizeof(attached_object->nocoll_group_ident)) IVP_CORE();
        strncpy(attached_object->nocoll_group_ident, t->nocoll_group_ident, sizeof(attached_object->nocoll_group_ident));
        attached_object->nocoll_group_ident[sizeof(attached_object->nocoll_group_ident) - 1] = '\0';
    }
    {   /* get rid of old core */
        int n_objs = ivp_core_n_objects(old_core);
        ivp_core_unlink_obj_from_core_and_maybe_destroy(old_core, attached_object);
        if (n_objs <= 1) old_core = NULL;   /* mark core as deleted */
        if (old_core) ivp_core_values_changed_recalc_redundants(old_core);
    }
    {   /* build new core */
        attached_object->object_movement_state = IVP_MT_NOT_SIM;
        attached_object->physical_core = core;
        attached_object->friction_core = core;
        attached_object->original_core = core;
        /* if (t->material): a libivp template always carries a material */
        attached_object->material = t->material;
        if (t->client_data) attached_object->client_data = t->client_data;
        attached_object->extra_radius = (float)t->extra_radius;
        ivp_object_init_object_core(attached_object, t);
    }

    /* set the new speed for next psi and wakeup object */
    if (old_core && IVP_MTIS_SIMULATED(old_core->movement_state)) {
        ivp_core_fire_event_object_frozen(attached_object->physical_core);   /* tell them that we went to sleep */
        ivp_object_ensure_in_simulation_now(attached_object);               /* wakeup and tell everybody */

        ivp_vec3f_t sp;
        lvf_set_p(&sp, &speed_ws);
        ivp_core_set_speed_f(attached_object->physical_core, &sp);
        attached_object->physical_core->rot_speed = rot_speed_cs;

        double delta_time_till_next_psi = ivp_time_diff(ivp_env_get_next_psi_time(env), ivp_env_get_current_time(env));
        ivp_core_t *my_core = attached_object->physical_core;
        if (!my_core->physical_unmoveable) {
            ivp_pvec_t active_hull_managers;
            ivp_pvec_init_with_allocator(&active_hull_managers,
                                         ivp_env_get_allocator(env));
            if (attached_object->hull_manager) {
                ivp_hull_mgr_jump_add_hull(attached_object->hull_manager, (float)moved_distance_val, 0.0f);
            }
            ivp_sim_es_t es;
            ivp_sim_es_init_dt(&es, env, delta_time_till_next_psi);
            ivp_cnps_calc_next_psi_matrix(my_core, &es, &active_hull_managers);
            my_core->old_sync_rot = NULL;   /* tmp_null.old_sync_info */
            ivp_cnps_commit_all_hull_managers(env, &active_hull_managers);
            ivp_pvec_clear(&active_hull_managers);
            ivp_object_recalc_exact_mindists_of_object(attached_object);
            ivp_object_update_exact_mindist_events_of_object(attached_object);
        }
    }
}

ivp_result_t ivp_object_reposition_object_Ros(ivp_object_t *parent, ivp_object_t *attached_object,
                                              const ivp_quatr_t *q_Ros_f_Aos, const ivp_vec3r_t *shift_Ros_f_Aos,
                                              bool check_before_moving)
{
    (void)check_before_moving;
    if (!attached_object || !q_Ros_f_Aos || !shift_Ros_f_Aos) return IVP_ERR_INVALID_ARG;
    ivp_transform_r_t m_world_f_Ros;
    if (parent) {
        ivp_object_get_m_world_f_object_AT(parent, &m_world_f_Ros);
    } else {
        ivp_transform_identity(&m_world_f_Ros);
    }

    ivp_transform_r_t m_Ros_f_Aos;
    ivp_quat_to_mat3(q_Ros_f_Aos, &m_Ros_f_Aos.rot);
    m_Ros_f_Aos.pos = *shift_Ros_f_Aos;

    ivp_transform_r_t m_ws_f_Aos;
    ivp_transform_compose(&m_ws_f_Aos, &m_world_f_Ros, &m_Ros_f_Aos);

    ivp_transform_r_t m_world_f_core;
    {   /* get current m_world_f_core */
        ivp_time_t current_time = ivp_env_get_current_time(attached_object->owner);
        ivp_quatr_t q_world_f_core;
        ivp_core_inline_calc_at_quaternion(attached_object->physical_core, current_time, &q_world_f_core);
        ivp_quat_to_mat3(&q_world_f_core, &m_world_f_core.rot);
        ivp_core_inline_calc_at_position(attached_object->physical_core, current_time, &m_world_f_core.pos);
    }

    ivp_transform_r_t m_Aos_f_Rcs;
    ivp_transform_inv_compose(&m_Aos_f_Rcs, &m_ws_f_Aos, &m_world_f_core);

    ivp_object_set_new_m_object_f_core(attached_object, &m_Aos_f_Rcs);

    D diff = (D)0.01f;                 /* hopefully we wont get an endless loop */
    D moved_distance_val = (D)0.0f;    /* center movement */

    ivp_hull_mgr_t *h_manager = attached_object->hull_manager;
    if (h_manager) ivp_hull_mgr_jump_add_hull(h_manager, (float)diff, (float)moved_distance_val);

    ivp_object_invalid_cache_object(attached_object);   /* invalidate_cache_object */
    ivp_object_recalc_exact_mindists_of_object(attached_object);
    ivp_object_recalc_invalid_mindists_of_object(attached_object);

    if (h_manager) {
        ivp_hull_mgr_check_hull_synapses(h_manager, attached_object->owner);
        ivp_hull_mgr_check_for_reset(h_manager);
    }
    return IVP_OK;
}
