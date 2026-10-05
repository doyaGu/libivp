/* ivp_object_pipeline.c — port of IVP_Real_Object / IVP_Polygon / IVP_Ball simulation
 * parts and IVP_Cache_Object_Manager.
 *
 * Reference: ivp_physics/ivp_object.cxx, ivp_physics/ivp_polygon.cxx, ivp_physics/ivp_ball.cxx,
 * ivp_intern/ivp_cache_object.cxx, ivp_physics/ivp_core.cxx (IVP_Core ctor / dtor).
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/collision/ivp_hull_mgr.h>
#include <ivp/geometry/ivp_surman.h>
#include "collision/ivp_lvec.h"
#include "dynamics/ivp_env_pipeline.h"
#include "dynamics/ivp_env_hot.h"
#include "solver/ivp_friction_internal.h"
#include <ivp/dynamics/ivp_cache_object.h>
#include <ivp/dynamics/ivp_anchor.h>

void ivp_core_free_friction_info_hash(ivp_core_t *core);   /* ivp_contact_point.c */

static inline ivp_env_pipeline_t *PL(const ivp_object_t *obj) { return obj->owner->pl; }

/* ── IVP_Cache_Object_Manager ──────────────────────────────────────────── */

ivp_result_t ivp_cache_obj_mgr_init(ivp_cache_obj_mgr_t *m,
                                    const ivp_allocator_t *allocator,
                                    int n)
{
    if (!m) return IVP_ERR_INVALID_ARG;
    IVP_ASSERT(n > 0);   /* number_of_cache_elements > 0 */
    if (n <= 0) n = 1;
    allocator = allocator ? allocator : ivp_default_allocator();
    if ((size_t)n > SIZE_MAX / sizeof(ivp_cache_obj_t)) {
        return IVP_ERR_CAPACITY_EXCEEDED;
    }
    m->n_cache_objects = n;
    m->allocator = allocator;
    const size_t bytes = (size_t)n * sizeof(ivp_cache_obj_t);
    m->cache_objects = (ivp_cache_obj_t *)ivp_alloc(allocator, bytes);
    if (!m->cache_objects) {
        m->n_cache_objects = 0;
        return IVP_ERR_OUT_OF_MEMORY;
    }
    memset(m->cache_objects, 0, bytes);
    m->reuse_loop_index = 0;
    return IVP_OK;
}

void ivp_cache_obj_mgr_destroy(ivp_cache_obj_mgr_t *m)
{
    if (!m->cache_objects) return;
    for (int i = 0; i < m->n_cache_objects; i++) {
        ivp_cache_obj_t *co = &m->cache_objects[i];
        if (co->object) ivp_object_invalid_cache_object(co->object);
    }
    ivp_free(m->allocator, m->cache_objects,
             (size_t)m->n_cache_objects * sizeof(ivp_cache_obj_t));
    m->cache_objects = NULL;
    m->n_cache_objects = 0;
    m->allocator = NULL;
}

void ivp_object_invalid_cache_object(ivp_object_t *object)
{
    if (object->cache_object) {
        IVP_ASSERT(object->cache_object->reference_count == 0);
        IVP_ASSERT(object->cache_object->object == object);
        object->cache_object->object = NULL;
        object->cache_object = NULL;
    }
}

#ifdef IVP_DEBUG_BUILD
/* IVP_Cache_Object_Manager::check (reference: #ifdef DEBUG; check when no
 * locks are set) */
void ivp_cache_obj_mgr_check(ivp_cache_obj_mgr_t *m)
{
    IVP_ASSERT(m->reuse_loop_index < m->n_cache_objects);
    for (int i = 0; i < m->n_cache_objects; i++) {
        ivp_cache_obj_t *co = &m->cache_objects[i];
        IVP_ASSERT(co->reference_count == 0);
        IVP_IF(co->object)
        {
            IVP_ASSERT(co->object->cache_object == co);
        }
    }
}
#endif

/* IVP_Cache_Object::update_cache_object */
void ivp_cache_obj_update(ivp_cache_obj_t *co)
{
    ivp_object_t *object = co->object;
    struct ivp_environment *env = object->owner;
    ivp_time_t current_time = ivp_env_get_current_time(env);
    co->valid_until_time_code = ivp_env_get_current_time_code(env);
    ivp_core_t *core = object->physical_core;   /* get_core() */
    if (ivp_time_diff(current_time, core->time_of_last_psi) == 0.0) {
        co->q_world_f_object = core->q_world_f_core_last_psi;
        co->m_world_f_object.pos = core->pos_world_f_core_last_psi;
        co->core_pos = core->pos_world_f_core_last_psi;
    } else {
        ivp_core_inline_calc_at_quaternion(core, current_time, &co->q_world_f_object);
        ivp_core_inline_calc_at_position(core, current_time, &co->m_world_f_object.pos);
        co->core_pos = co->m_world_f_object.pos;
    }
    ivp_quat_to_mat3(&co->q_world_f_object, &co->m_world_f_object.rot);
    if (!object->shift_core_f_object_is_zero) {
        ivp_vec3r_t p;
        lvm_vmult4(&co->m_world_f_object, &object->shift_core_f_object, &p);
        co->m_world_f_object.pos = p;
    }
    if (object->q_core_f_object) {
        ivp_quat_mult(&co->q_world_f_object, &co->q_world_f_object, object->q_core_f_object);
        ivp_quat_to_mat3(&co->q_world_f_object, &co->m_world_f_object.rot);
    }
}

static ivp_cache_obj_t *cache_mgr_get_cache_object(ivp_cache_obj_mgr_t *m, ivp_object_t *object)
{
    ivp_cache_obj_t *co;
    int attempts = 0;
    while ((co = &m->cache_objects[m->reuse_loop_index])->reference_count) {
        m->reuse_loop_index++;
        if (m->reuse_loop_index >= m->n_cache_objects) m->reuse_loop_index = 0;
        if (++attempts >= m->n_cache_objects) {
            IVP_ASSERT(0 && "All cache objects are referenced - cannot evict");
            ivp_message("IVP_Cache_Object_Manager::get_cache_object: all %d cache objects are referenced\n",
                        m->n_cache_objects);
            IVP_CORE();
            return NULL;
        }
    }
    m->reuse_loop_index++;
    if (m->reuse_loop_index >= m->n_cache_objects) m->reuse_loop_index = 0;
    ivp_object_t *obj = co->object;
    if (obj) ivp_object_invalid_cache_object(obj);
    co->object = object;
    object->cache_object = co;
    co->valid_until_time_code = 0;
    if (!IVP_MTIS_SIMULATED(object->object_movement_state)) ivp_cache_obj_update(co);
    return co;
}

ivp_cache_obj_t *ivp_object_get_cache_object_no_lock(ivp_object_t *obj)
{
    if (!obj->cache_object) obj->cache_object = cache_mgr_get_cache_object(&PL(obj)->cache_mgr, obj);
    if (IVP_MTIS_SIMULATED(obj->object_movement_state)) {
        if (ivp_env_get_current_time_code(obj->owner) > obj->cache_object->valid_until_time_code) {
            ivp_cache_obj_update(obj->cache_object);
        }
    }
    return obj->cache_object;
}

ivp_cache_obj_t *ivp_object_get_cache_object(ivp_object_t *obj)
{
    if (!obj->cache_object) obj->cache_object = cache_mgr_get_cache_object(&PL(obj)->cache_mgr, obj);
    obj->cache_object->reference_count++;
    if (IVP_MTIS_SIMULATED(obj->object_movement_state)) {
        if (ivp_env_get_current_time_code(obj->owner) > obj->cache_object->valid_until_time_code) {
            ivp_cache_obj_update(obj->cache_object);
        }
    }
    return obj->cache_object;
}

/* ── IVP_Real_Object simulation helpers ────────────────────────────────── */

void ivp_object_set_movement_state(ivp_object_t *obj, ivp_movement_type_t mt)
{
    obj->object_movement_state = (uint8_t)mt;
}

void ivp_object_ensure_in_simulation(ivp_object_t *obj)
{
    if (obj->object_movement_state != IVP_MT_NOT_SIM) {
        ivp_core_reset_freeze_check_values(obj->physical_core);
    } else {
        IVP_ASSERT(!obj->physical_core->physical_unmoveable);
        ivp_env_add_revive_core(obj->owner, obj->friction_core);
    }
}

void ivp_object_revive_object_for_simulation(ivp_object_t *obj)
{
    if (obj->object_movement_state == IVP_MT_NOT_SIM) {
        ivp_sim_unit_revive_for_simulation(obj->friction_core->sim_unit, ivp_core_env(obj->friction_core));
    }
}

void ivp_object_ensure_in_simulation_now(ivp_object_t *obj)
{
    IVP_ASSERT(!obj->physical_core->physical_unmoveable);
    if (obj->physical_core->movement_state == IVP_MT_NOT_SIM) ivp_object_revive_object_for_simulation(obj);
}

void ivp_object_recalc_invalid_mindists_of_object(ivp_object_t *obj)
{
    ivp_md_synapse_t *first_syn, *next_syn;
    ivp_mindist_mgr_t *mm = &PL(obj)->mindist_mgr;
    for (first_syn = obj->invalid_synapses; first_syn; first_syn = next_syn) {
        next_syn = first_syn->next;
        ivp_mindist_t *mdist = first_syn->mindist;
        IVP_ASSERT(mdist->mindist_function == IVP_MF_COLLISION);
        (void)ivp_mindist_recalc_invalid_mindist(mdist);
        if (mdist->recalc_result == IVP_MDRR_INTRUSION) continue;
        ivp_mindist_mgr_remove_invalid_mindist(mm, mdist);
        ivp_mindist_mgr_insert_exact_mindist(mm, mdist);
    }
}

void ivp_object_recalc_exact_mindists_of_object(ivp_object_t *obj)
{
    ivp_md_synapse_t *first_syn, *next_syn;
    ivp_mindist_mgr_t *mm = &PL(obj)->mindist_mgr;
    for (first_syn = obj->exact_synapses; first_syn; first_syn = next_syn) {
        next_syn = first_syn->next;
        ivp_mindist_mgr_recalc_exact_mindist(mm, first_syn->mindist);
    }
}

void ivp_object_update_exact_mindist_events_of_object(ivp_object_t *obj)
{
    ivp_md_synapse_t *syn, *syn_next;
    for (syn = obj->exact_synapses; syn; syn = syn_next) {
        syn_next = syn->next;
        ivp_mindist_t *mindist = syn->mindist;
        ivp_core_t *core0 = mindist->synapse[0].l_obj->physical_core;
        ivp_core_t *core1 = mindist->synapse[1].l_obj->physical_core;
        if (core0->mindist_event_already_done != core1->mindist_event_already_done) {
            (void)ivp_mindist_recalc_mindist(mindist);
            if (mindist->recalc_result == IVP_MDRR_OK) {
                ivp_mindist_update_exact_mindist_events(mindist, false, IVP_EH_BIG_DELAY);
            }
        }
    }
}

void ivp_object_get_all_near_mindists(ivp_object_t *obj)
{
    ivp_movement_type_t temp_movement = obj->physical_core->movement_state;
    obj->physical_core->movement_state = IVP_MT_GET_MINDIST;
    ivp_mindist_mgr_recheck_ov_element(&PL(obj)->mindist_mgr, obj);
    obj->physical_core->movement_state = temp_movement;
}

void ivp_object_recheck_collision_filter(ivp_object_t *obj)
{
    if (obj->ov_element) ivp_mindist_mgr_recheck_ov_element(&PL(obj)->mindist_mgr, obj);
}

void ivp_object_reset_time(ivp_object_t *obj, double offset)
{
    if (obj->hull_manager) ivp_hull_mgr_reset_time(obj->hull_manager, offset);
}

/* IVP_Real_Object::calc_at_matrix */
void ivp_object_calc_at_matrix(const ivp_object_t *obj, ivp_time_t current_time, ivp_transform_r_t *out)
{
    ivp_quatr_t q_world_f_object;
    const ivp_core_t *core = obj->physical_core;   /* get_core() */
    ivp_core_inline_calc_at_quaternion(core, current_time, &q_world_f_object);
    ivp_core_inline_calc_at_position(core, current_time, &out->pos);
    ivp_quat_to_mat3(&q_world_f_object, &out->rot);
    if (!obj->shift_core_f_object_is_zero) {
        ivp_vec3r_t p;
        lvm_vmult4(out, &obj->shift_core_f_object, &p);
        out->pos = p;
    }
    if (obj->q_core_f_object) {
        ivp_quat_mult(&q_world_f_object, &q_world_f_object, obj->q_core_f_object);
        ivp_quat_to_mat3(&q_world_f_object, &out->rot);
    }
}

void ivp_object_async_add_speed_object_ws(ivp_object_t *obj, const ivp_vec3f_t *speed_vec)
{
    ivp_object_ensure_in_simulation(obj);
    ivp_core_t *c = obj->physical_core;
    ivp_vec3f_t sc;
    lvf_set_p(&sc, &c->speed_change);
    lvf_add(&sc, speed_vec);
    lvp_set_f(&c->speed_change, &sc);
}

void ivp_object_async_add_rot_speed_object_cs(ivp_object_t *obj, const ivp_vec3f_t *rotation_vec)
{
    ivp_object_ensure_in_simulation(obj);
    ivp_core_t *c = obj->physical_core;   /* get_core() */
    ivp_vec3f_t rsc;
    lvf_set_p(&rsc, &c->rot_speed_change);
    lvf_add(&rsc, rotation_vec);
    lvp_set_f(&c->rot_speed_change, &rsc);
}

void ivp_object_async_push_object_ws(ivp_object_t *obj, const ivp_vec3r_t *position_ws, const ivp_vec3f_t *impulse_ws)
{
    ivp_object_ensure_in_simulation(obj);
    ivp_core_t *core = obj->physical_core;   /* get_core() */
    ivp_transform_r_t m_world_f_core;
    ivp_core_calc_at_matrix_ref(core, ivp_env_get_current_time(obj->owner), &m_world_f_core);
    ivp_vec3f_t impulse_cs, position_cs;
    lvm_vimult4_pf(&m_world_f_core, position_ws, &position_cs);
    lvm_vimult3_f(&m_world_f_core, impulse_ws, &impulse_cs);
    ivp_core_async_push_core(core, &position_cs, &impulse_cs, impulse_ws);
}

/* IVP_Real_Object::unlink_contact_points */
void ivp_object_unlink_contact_points(ivp_object_t *obj, bool silent)
{
    /* then delete static friction datas @@@OS, loop only once + flag */
    int debug_once = 0;
    ivp_fr_synapse_t *fr_synapse;
    while ((fr_synapse = obj->fr_synapses) != NULL) {
        ivp_contact_pt_t *fr_mindist = fr_synapse->contact_point;
        ivp_fr_system_t *fr_sys = fr_mindist->l_friction_system;
        IVP_IF(debug_once) /* never true: debug_once starts at 0 */
        {
            debug_once = 1;
            ivp_fr_synapse_t *debug_syn = obj->fr_synapses;
            ivp_debug_printf("still_left_fd ");
            while (debug_syn) {
                ivp_debug_printf("fd %p sys %p  ", (void *)debug_syn->contact_point,
                                 (void *)debug_syn->contact_point->l_friction_system);
                debug_syn = debug_syn->next;
            }
            ivp_debug_printf("\n");
            ivp_fr_system_debug_fs_out_ascii(fr_sys);
            ivp_debug_printf("newl\n");
        }
        ivp_core_t *core0 = fr_mindist->synapse[0].l_obj->friction_core;
        ivp_core_t *core1 = fr_mindist->synapse[1].l_obj->friction_core;
        if (!silent) {
            ivp_core_ensure_core_to_be_in_simulation(core0);
            ivp_core_ensure_core_to_be_in_simulation(core1);
        }
        if (!fr_sys) {
            ivp_contact_pt_destroy(fr_mindist);
            continue;
        }
        ivp_fr_system_delete_friction_distance(fr_sys, fr_mindist);
        if (fr_sys->friction_dist_number == 0) ivp_fr_system_destroy(fr_sys);
    }
}

/* IVP_Real_Object::enable_collision_detection */
void ivp_object_enable_collision_detection(ivp_object_t *obj, bool enable)
{
    if (enable) {
        if (!obj->collision_detection_enabled) {
            ivp_mindist_mgr_enable_collision_detection_for_object(&PL(obj)->mindist_mgr, obj);
            obj->collision_detection_enabled = true;
        }
    } else {
        if (obj->collision_detection_enabled) {
            if (obj->ov_element) {
                ivp_ov_element_destroy(obj->ov_element);
                obj->ov_element = NULL;
            }
            obj->collision_detection_enabled = false;
            ivp_object_unlink_contact_points(obj, true);
        }
    }
}

/* IVP_Real_Object::set_pinned */
void ivp_object_set_pinned_ref(ivp_object_t *obj, bool is_pinned)
{
    ivp_core_t *core = obj->physical_core;   /* get_core() */
    core->pinned = is_pinned;
    ivp_vec3r_set_zero(&core->speed);
    lvf_zero(&core->rot_speed);
    if (is_pinned) {
        bool c_enabled = obj->collision_detection_enabled;
        ivp_object_enable_collision_detection(obj, false);
        ivp_vec3r_set_zero(&core->inv_rot_inertia);
        core->inv_mass = IVP_REAL(0.0);
        ivp_vec3r_set_zero(&core->speed_change);
        ivp_vec3r_set_zero(&core->rot_speed_change);
        ivp_object_enable_collision_detection(obj, c_enabled);
    } else {
        core->inv_rot_inertia.k[0] = (ivp_real_t)(1.0f / (float)core->rot_inertia.k[0]);
        core->inv_rot_inertia.k[1] = (ivp_real_t)(1.0f / (float)core->rot_inertia.k[1]);
        core->inv_rot_inertia.k[2] = (ivp_real_t)(1.0f / (float)core->rot_inertia.k[2]);
        core->inv_mass = (ivp_real_t)(1.0f / (float)core->mass);
        ivp_mindist_mgr_recheck_ov_element(&PL(obj)->mindist_mgr, obj);
    }
}

/* IVP_Real_Object::revive_nearest_objects_grow_fs */
void ivp_object_revive_nearest_objects_grow_fs(ivp_object_t *obj)
{
    if (obj->physical_core->physical_unmoveable) {
        ivp_object_get_all_near_mindists(obj);
        (void)ivp_core_grow_friction_system(obj->physical_core);
        ivp_core_revive_adjacent_to_unmoveable(obj->physical_core);
    } else {
        ivp_core_ensure_core_to_be_in_simulation(obj->physical_core);
    }
}

/* IVP_Real_Object::change_unmovable_flag */
void ivp_object_change_unmovable_flag(ivp_object_t *obj, bool flag)
{
    ivp_core_t *my_core = obj->physical_core;   /* get_core() */
    if (my_core->physical_unmoveable == flag) return;
    ivp_sim_unit_t *replacement_unit = NULL;
    ivp_sim_ctrl_t *gravity = NULL;
    if (flag && my_core->sim_unit) {
        replacement_unit = ivp_sim_unit_new();
        gravity = ivp_env_get_standard_gravity_controller(ivp_core_env(my_core));
        if (!replacement_unit || !ivp_sim_unit_add_sim_unit_core(replacement_unit, my_core) ||
            !ivp_sim_unit_add_controller_of_core(replacement_unit, my_core, gravity) ||
            !ivp_pvec_reserve(&my_core->controllers_of_core, 1)) {
            ivp_sim_unit_delete(replacement_unit);
            return;
        }
        replacement_unit->sim_unit_movement_type = IVP_MT_NOT_SIM;
    }
    for (int d = ivp_core_n_objects(my_core) - 1; d >= 0; d--) {
        ivp_object_unlink_contact_points(ivp_core_object_at(my_core, d), false);
    }
    if (my_core->physical_unmoveable) {
        ivp_core_free_friction_info_hash(my_core);
    } else {
        my_core->core_friction_info.for_moveables.moveable_core_friction_info = NULL;
    }
    if (my_core->movement_state < IVP_MT_NOT_SIM) ivp_core_freeze_simulation_core(my_core);
    my_core->physical_unmoveable = flag;
    if (flag) {
        if (my_core->sim_unit) {
            struct ivp_environment *env = ivp_core_env(my_core);
            ivp_sim_unit_sim_unit_remove_core(my_core->sim_unit, my_core);
            my_core->sim_unit = replacement_unit;
            for (int ic = ivp_pvec_len(&my_core->controllers_of_core) - 1; ic >= 0; ic--) {
                ivp_pvec_remove_at(&my_core->controllers_of_core, ic);
            }
            ivp_sim_units_mgr_add_sim_unit(&env->pl->sim_units_manager, my_core->sim_unit);
            int added = ivp_pvec_add(&my_core->controllers_of_core, gravity);
            IVP_ASSERT(added >= 0);
            if (added < 0) IVP_CORE();   /* core controller capacity was reserved above */
        }
    }
    ivp_core_values_changed_recalc_redundants(my_core);
}

/* IVP_Real_Object::beam_object_to_new_position */
void ivp_object_beam_object_to_new_position(ivp_object_t *obj, const ivp_quatr_t *rotation_world_f_object,
                                            const ivp_vec3r_t *position_w_f_o, bool optimize_for_repeated_calls)
{
    ivp_core_t *core = obj->physical_core;   /* get_core() */
    ivp_quatr_t rotation_world_f_core = *rotation_world_f_object;
    ivp_vec3r_t position_w_f_c = *position_w_f_o;
    if (!obj->shift_core_f_object_is_zero) {
        ivp_transform_r_t m_world_f_object;
        ivp_quat_to_mat3(rotation_world_f_object, &m_world_f_object.rot);
        m_world_f_object.pos = *position_w_f_o;
        ivp_vec3r_t inv_shift;
        ivp_vec3f_t s;
        lvf_set_p(&s, &obj->shift_core_f_object);
        lvf_set_negative(&s, &s);
        lvp_set_f(&inv_shift, &s);
        lvm_vmult4(&m_world_f_object, &inv_shift, &position_w_f_c);
    }
    if (obj->q_core_f_object) {
        ivp_quat_set_div_unit_quat(&rotation_world_f_core, rotation_world_f_object, obj->q_core_f_object);
    }
    ivp_cnps_set_transformation(core, &rotation_world_f_core, &position_w_f_c, optimize_for_repeated_calls);
}

/* IVP_Real_Object::disable_simulation */
bool ivp_object_disable_simulation(ivp_object_t *obj)
{
    ivp_core_t *core = obj->physical_core;   /* get_core() */
    if (core->physical_unmoveable) return true;
    if (core->is_in_wakeup_vec) ivp_env_remove_revive_core(ivp_core_env(core), core);
    if (core->movement_state < IVP_MT_NOT_SIM) {
        for (int c = ivp_core_n_objects(core) - 1; c >= 0; c--) {
            /* trick to prevent any friction systems to wake up the object again */
            ivp_object_unlink_contact_points(ivp_core_object_at(core, c), true);
        }
        ivp_quatf_set(&core->q_world_f_core_calm_reference[0], &core->q_world_f_core_next_psi);
        lvf_set_p(&core->position_world_f_core_calm_reference[0], &core->pos_world_f_core_last_psi);
        /* IVP_Time ref_time = now_time - 20 (IVP_Time::operator- rounds to float) */
        core->time_of_calm_reference[0] = ivp_time_diff(ivp_env_get_current_time(ivp_core_env(core)), 20.0);
        ivp_sim_unit_do_sim_unit_union_find(core->sim_unit);
        return ivp_sim_unit_sim_unit_calc_movement_state(core->sim_unit, ivp_core_env(core));
    }
    return true;
}

/* IVP_Real_Object::insert_anchor / remove_anchor (ivp_object.cxx:443-471) */
void ivp_anchor_link(ivp_anchor_t *anchor, ivp_vec3r_t *core_pos)
{
    ivp_object_t *obj = anchor->l_anchor_object;
    if (!obj || anchor->linked) return;
    anchor->core_pos = core_pos;
    anchor->anchor_prev_in_object = NULL;
    anchor->anchor_next_in_object = obj->anchors;
    if (obj->anchors) obj->anchors->anchor_prev_in_object = anchor;
    obj->anchors = anchor;
    anchor->linked = true;
}

void ivp_anchor_unlink(ivp_anchor_t *anchor)
{
    if (!anchor->linked) return;
    ivp_anchor_t *previous = anchor->anchor_prev_in_object;
    ivp_anchor_t *following = anchor->anchor_next_in_object;
    if (previous) {
        previous->anchor_next_in_object = following;
    } else {
        anchor->l_anchor_object->anchors = following;
    }
    if (following) following->anchor_prev_in_object = previous;
    anchor->anchor_next_in_object = anchor->anchor_prev_in_object = NULL;
    anchor->core_pos = NULL;
    anchor->linked = false;
}

/* IVP_Real_Object::set_new_m_object_f_core */
void ivp_object_set_new_m_object_f_core(ivp_object_t *obj, const ivp_transform_r_t *new_m_object_f_core)
{
    const bool needs_quaternion =
        new_m_object_f_core->rot.rows[0].k[0] != IVP_REAL(1.0) ||
        new_m_object_f_core->rot.rows[1].k[1] != IVP_REAL(1.0) ||
        new_m_object_f_core->rot.rows[2].k[2] != IVP_REAL(1.0);
    ivp_quatr_t *q_core_f_object = obj->q_core_f_object;
    if (needs_quaternion && !q_core_f_object) {
        q_core_f_object = (ivp_quatr_t *)calloc(1, sizeof(*q_core_f_object));
        if (!q_core_f_object) return;
    }

    /* new_m_core_f_object = transpose(new_m_object_f_core) */
    ivp_transform_r_t m;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) m.rot.rows[r].k[c] = new_m_object_f_core->rot.rows[c].k[r];
    }
    ivp_vec3r_t neg;
    lvp_set_negative(&neg, &new_m_object_f_core->pos);
    lvm_vmult3(&m, &neg, &m.pos);
    /* new_m_core_f_object.vmult4(&a->object_pos, &a->core_pos) */
    for (ivp_anchor_t *a = obj->anchors; a; a = a->anchor_next_in_object) {
        ivp_vec3f_t core_pos;
        lvm_vmult4_ff(&m, &a->object_pos, &core_pos);
        lvp_set_f(a->core_pos, &core_pos);
    }
    if (lvp_quad_length(&m.pos) < (D)(P_RES_EPS * P_RES_EPS)) {
        obj->shift_core_f_object_is_zero = true;
        ivp_vec3r_set_zero(&obj->shift_core_f_object);
    } else {
        ivp_vec3f_t f;
        lvf_set_p(&f, &m.pos);
        lvp_set_f(&obj->shift_core_f_object, &f);   /* IVP_U_Float_Point shift_core_f_object */
        obj->shift_core_f_object_is_zero = false;
    }
    if (!needs_quaternion) {
        if (obj->q_core_f_object) {
            free(obj->q_core_f_object);
            obj->q_core_f_object = NULL;
        }
    } else {
        obj->q_core_f_object = q_core_f_object;
        ivp_quat_from_mat3(obj->q_core_f_object, &m.rot);
        (void)ivp_quat_normalize(obj->q_core_f_object);
    }
    if (obj->cache_object) ivp_object_invalid_cache_object(obj);
}

/* IVP_Core::transform_PSI_matrizes_core */
void ivp_core_transform_psi_matrizes_core(ivp_core_t *core, const ivp_transform_r_t *m_core_f_core)
{
    ivp_transform_r_t res;
    /* m_world_f_core_last_psi.mmult4(m_core_f_core, &m_world_f_core_last_psi) */
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            res.rot.rows[r].k[c] = core->m_world_f_core_last_psi.rot.rows[r].k[0] * m_core_f_core->rot.rows[0].k[c] +
                                   core->m_world_f_core_last_psi.rot.rows[r].k[1] * m_core_f_core->rot.rows[1].k[c] +
                                   core->m_world_f_core_last_psi.rot.rows[r].k[2] * m_core_f_core->rot.rows[2].k[c];
        }
    }
    lvm_vmult4(&core->m_world_f_core_last_psi, &m_core_f_core->pos, &res.pos);
    core->m_world_f_core_last_psi = res;
    core->pos_world_f_core_last_psi = res.pos;
    ivp_quat_from_mat3(&core->q_world_f_core_last_psi, &res.rot);
    core->q_world_f_core_next_psi = core->q_world_f_core_last_psi;
}

/* IVP_Real_Object::init_object_core */
void ivp_object_init_object_core(ivp_object_t *obj, const ivp_object_template_t *templ)
{
    bool is_ball = (obj->type == IVP_OBJECT_BALL);
    struct ivp_environment *env = obj->owner;
    ivp_vec3f_t center;
    lvf_zero(&center);
    if (templ->use_mass_center_override) {
        lvf_set_p(&center, &templ->mass_center_override);
    } else if (obj->surman && !is_ball) {
        float c[3];
        ivp_surman_get_mass_center(obj->surman, c);
        lvf_set3(&center, c[0], c[1], c[2]);
    }
    ivp_transform_r_t m_object_f_core;
    memset(&m_object_f_core, 0, sizeof(m_object_f_core));
    m_object_f_core.rot.rows[0].k[0] = m_object_f_core.rot.rows[1].k[1] = m_object_f_core.rot.rows[2].k[2] = IVP_REAL(1.0);
    lvp_set_f(&m_object_f_core.pos, &center);
    ivp_object_set_new_m_object_f_core(obj, &m_object_f_core);
    ivp_core_t *core = obj->physical_core;   /* get_core() */
    float rad = 0.0f, rad_dev = 0.0f;
    if (!is_ball && obj->surman) {
        float c[3] = { center.k[0], center.k[1], center.k[2] };
        ivp_surman_get_radius_and_deviation(obj->surman, c, &rad, &rad_dev);
    }
    rad += obj->extra_radius;
    core->upper_limit_radius = (ivp_real_t)rad;     /* set_radius */
    core->max_surface_deviation = (ivp_real_t)rad_dev;
    double mass = (double)templ->mass;
    if ((double)templ->mass < P_RES_EPS) mass = 1.0f;
    ivp_vec3f_t rot_inertia;
    if (templ->rot_inertia_is_factor) {
        ivp_vec3f_t rot_in;
        if (!is_ball && obj->surman) {
            float ri[3];
            ivp_surman_get_rotation_inertia(obj->surman, ri);
            lvf_set3(&rot_in, ri[0], ri[1], ri[2]);
        } else {
            float rot = (2.0f / 5.0f) * rad * rad; /* IVP_FLOAT rad: float products */
            lvf_set3(&rot_in, rot, rot, rot);
        }
        ivp_vec3f_t fac;
        lvf_set_p(&fac, &templ->rot_inertia);
        lvf_set_pairwise_mult(&rot_inertia, &rot_in, &fac);
        lvf_mult(&rot_inertia, (D)mass);
    } else {
        lvf_set_p(&rot_inertia, &templ->rot_inertia);
    }
    if (templ->auto_check_rot_inertia != IVP_REAL(0.0)) {
        float min_inertia_value = (float)((double)lvf_real_length(&rot_inertia) * (double)(float)templ->auto_check_rot_inertia);
        for (int hh = 0; hh < 3; hh++) {
            if (rot_inertia.k[hh] < min_inertia_value) rot_inertia.k[hh] = min_inertia_value;
        }
    }
    lvp_set_f(&core->rot_inertia, &rot_inertia);
    core->mass = (ivp_real_t)(float)mass;
    core->speed_damp_factor = (ivp_real_t)(float)templ->speed_damp_factor;
    ivp_vec3f_t rdf;
    lvf_set_p(&rdf, &templ->rot_speed_damp_factor);
    lvp_set_f(&core->rot_speed_damp_factor, &rdf);
    lvf_zero(&core->rot_speed);
    ivp_vec3r_set_zero(&core->speed);
    ivp_core_calc_calc(core);
    ivp_core_transform_psi_matrizes_core(core, &m_object_f_core);
    ivp_sim_es_t es;
    ivp_sim_es_init_dt(&es, env, ivp_time_diff(ivp_env_get_next_psi_time(env), ivp_env_get_current_time(env)));
    ivp_core_calc_next_psi_matrix_zero_speed(core, &es);
    if (templ->pinned) ivp_object_set_pinned_ref(obj, true);
}

/* IVP_Real_Object ctor + IVP_Core ctor + IVP_Polygon / IVP_Ball ctor */
bool ivp_object_pipeline_create(ivp_environment_t *env, ivp_object_t *obj, const ivp_object_template_t *tmpl)
{
    bool is_ball = (tmpl->type == IVP_OBJECT_BALL) ||
                   (obj->surman && ivp_surman_get_type(obj->surman) == IVP_SURMAN_BALL);
    obj->type = is_ball ? IVP_OBJECT_BALL : IVP_OBJECT_POLYGON;
    obj->exact_synapses = NULL;
    obj->invalid_synapses = NULL;
    obj->fr_synapses = NULL;
    obj->ov_element = NULL;
    obj->cache_object = NULL;
    obj->controller_phantom = NULL;
    obj->q_core_f_object = NULL;
    ivp_vec3r_set_zero(&obj->shift_core_f_object);
    obj->shift_core_f_object_is_zero = true;
    memset(obj->nocoll_group_ident, 0, sizeof(obj->nocoll_group_ident));
    if (tmpl->nocoll_group_ident) {
        /* IVP_Template_Real_Object::set_nocoll_group_ident: CORE for an id
         * longer than IVP_NO_COLL_GROUP_STRING_LEN - 1 (libivp's template
         * holds the string itself, checked where it is used) */
        if (strlen(tmpl->nocoll_group_ident) >= sizeof(obj->nocoll_group_ident)) IVP_CORE();
        strncpy(obj->nocoll_group_ident, tmpl->nocoll_group_ident, sizeof(obj->nocoll_group_ident));
        obj->nocoll_group_ident[sizeof(obj->nocoll_group_ident) - 1] = '\0';
    }

    /* physical_core = new IVP_Core(this, q, pos, unmoveable, piling);
     * friction_core = original_core = physical_core */
    ivp_core_t *core = ivp_core_create(obj, &tmpl->orientation, &tmpl->position, tmpl->physical_unmoveable,
                                       tmpl->enable_piling_optimization);
    if (!core) return false;
    obj->physical_core = core;
    obj->friction_core = core;
    obj->original_core = core;

    ivp_object_set_movement_state(obj, tmpl->physical_unmoveable ? IVP_MT_STATIC : IVP_MT_NOT_SIM);

    /* Attach all-core controllers before allocating the remaining object
     * state so failure can discard the new core without publishing it. */
    if (!ivp_env_attach_default_controllers(env, core)) {
        ivp_core_unlink_obj_from_core_and_maybe_destroy(core, obj);
        obj->physical_core = NULL;
        obj->friction_core = NULL;
        obj->original_core = NULL;
        return false;
    }

    /* hull manager */
    obj->hull_manager =
        (struct ivp_hull_mgr *)ivp_alloc(env->allocator, sizeof(ivp_hull_mgr_t));
    if (!obj->hull_manager) {
        ivp_core_unlink_obj_from_core_and_maybe_destroy(core, obj);
        obj->physical_core = NULL;
        obj->friction_core = NULL;
        obj->original_core = NULL;
        return false;
    }
    memset(obj->hull_manager, 0, sizeof(ivp_hull_mgr_t));
    if (ivp_hull_mgr_init(obj->hull_manager, env->allocator) != IVP_OK) {
        ivp_free(env->allocator, obj->hull_manager, sizeof(ivp_hull_mgr_t));
        obj->hull_manager = NULL;
        ivp_core_unlink_obj_from_core_and_maybe_destroy(core, obj);
        obj->physical_core = NULL;
        obj->friction_core = NULL;
        obj->original_core = NULL;
        return false;
    }

    /* IVP_Polygon / IVP_Ball */
    if (is_ball) {
        float r = (float)tmpl->ball_radius;
        if (r <= 0.0f && obj->surman) r = ivp_surman_get_ball_radius(obj->surman);
        obj->extra_radius = r + (float)tmpl->extra_radius;
    } else {
        obj->extra_radius = (float)tmpl->extra_radius;
    }
    ivp_object_init_object_core(obj, tmpl);

    return true;
}

/* ~IVP_Real_Object: unlink the object from its cores (each core is deleted
 * with its last object) */
static void obj_unlink_cores(ivp_object_t *obj)
{
    ivp_core_t *physical_core = obj->physical_core;
    ivp_core_t *original_core = obj->original_core;
    ivp_core_t *friction_core = obj->friction_core;
    if (!physical_core) return;
    ivp_core_unlink_obj_from_core_and_maybe_destroy(physical_core, obj);
    if (original_core != physical_core) ivp_core_unlink_obj_from_core_and_maybe_destroy(original_core, obj);
    if (friction_core != original_core && friction_core != physical_core) {
        ivp_core_unlink_obj_from_core_and_maybe_destroy(friction_core, obj);
    }
    obj->physical_core = NULL;
    obj->friction_core = NULL;
    obj->original_core = NULL;
}

/* IVP_Real_Object::~IVP_Real_Object (anchors and phantoms are not used here) */
/* first half of ~IVP_Real_Object: phantom, hull manager, clear_internal_references */
void ivp_object_pipeline_clear_references(ivp_environment_t *env, ivp_object_t *obj)
{
    (void)env;
    if (obj->controller_phantom) ivp_controller_phantom_destroy((struct ivp_controller_phantom *)obj->controller_phantom);
    if (obj->hull_manager) ivp_hull_mgr_delete_listeners(obj->hull_manager);
    /* clear_internal_references */
    ivp_object_unlink_contact_points(obj, true);
    ivp_md_synapse_t *syn;
    while ((syn = obj->exact_synapses) != NULL) {
        ivp_collision_delete(&syn->mindist->coll);
    }
    if (obj->ov_element) {
        ivp_ov_element_destroy(obj->ov_element);
        obj->ov_element = NULL;
    }
    while ((syn = obj->invalid_synapses) != NULL) {
        ivp_collision_delete(&syn->mindist->coll);
    }
}

/* second half of ~IVP_Real_Object (after the deleted events): callback free
 * memory: cache object, unlink_obj_from_core_and_maybe_destroy */
void ivp_object_pipeline_destroy(ivp_environment_t *env, ivp_object_t *obj)
{
    (void)env;
    if (obj->cache_object) ivp_object_invalid_cache_object(obj);
    obj_unlink_cores(obj);
    if (obj->hull_manager) {
        ivp_hull_mgr_destroy(obj->hull_manager);
        ivp_free(env->allocator, obj->hull_manager, sizeof(ivp_hull_mgr_t));
        obj->hull_manager = NULL;
    }
    if (obj->q_core_f_object) {
        free(obj->q_core_f_object);
        obj->q_core_f_object = NULL;
    }
}

/* ── public AT helpers (ivp_cache_object.h) ────────────────────────────── */

void ivp_object_get_at_matrix(const ivp_object_t *obj, ivp_time_t t, ivp_transform_r_t *out)
{
    ivp_object_calc_at_matrix(obj, t, out);
}

/* IVP_Real_Object::calc_at_quaternion */
void ivp_object_get_at_position(const ivp_object_t *obj, ivp_time_t t, ivp_vec3r_t *position_out)
{
    const ivp_core_t *core = obj->physical_core;   /* get_core() */
    ivp_core_inline_calc_at_position(core, t, position_out);
    if (!obj->shift_core_f_object_is_zero) {
        ivp_quatr_t q;
        ivp_core_inline_calc_at_quaternion(core, t, &q);
        ivp_transform_r_t m;
        ivp_quat_to_mat3(&q, &m.rot);
        m.pos = *position_out;
        lvm_vmult4(&m, &obj->shift_core_f_object, position_out);
    }
}

void ivp_object_get_at_quat(const ivp_object_t *obj, ivp_time_t t, ivp_quatr_t *out)
{
    ivp_core_inline_calc_at_quaternion(obj->physical_core, t, out);
    if (obj->q_core_f_object) ivp_quat_mult(out, out, obj->q_core_f_object);
}

/* IVP_Real_Object::calc_m_core_f_object */
void ivp_object_calc_m_core_f_object(const ivp_object_t *obj, ivp_transform_r_t *m_core_f_object)
{
    memset(m_core_f_object, 0, sizeof(*m_core_f_object));
    m_core_f_object->rot.rows[0].k[0] = IVP_REAL(1.0);
    m_core_f_object->rot.rows[1].k[1] = IVP_REAL(1.0);
    m_core_f_object->rot.rows[2].k[2] = IVP_REAL(1.0);
    if (obj->q_core_f_object) ivp_quat_to_mat3(obj->q_core_f_object, &m_core_f_object->rot);
    /* shift_core_f_object is an IVP_U_Float_Point */
    m_core_f_object->pos.k[0] = (D)(float)obj->shift_core_f_object.k[0];
    m_core_f_object->pos.k[1] = (D)(float)obj->shift_core_f_object.k[1];
    m_core_f_object->pos.k[2] = (D)(float)obj->shift_core_f_object.k[2];
}

/* IVP_Real_Object::get_quat_world_f_object_AT (calc_at_quaternion at the
 * environment's current time) */
void ivp_object_get_quat_world_f_object_AT(const ivp_object_t *obj, ivp_quatr_t *q_world_f_object, ivp_vec3r_t *position)
{
    ivp_time_t t = ivp_env_get_current_time(obj->owner);
    ivp_object_get_at_position(obj, t, position);
    ivp_object_get_at_quat(obj, t, q_world_f_object);
}
