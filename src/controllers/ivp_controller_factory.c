/* ivp_controller_factory.c -- IVP_Controller_Factory
 *
 * Port of ivp_controller/ivp_environment_factories.cxx (create_rotmot,
 * create_torque, create_controller_motion, create_force, create_spring,
 * create_suspension, create_stabilizer, create_check_dist) and
 * IVP_Controller_Factory::create_constraint (ivp_template_constraint.cxx),
 * with the template constructors (ivp_actuator.cxx, ivp_actuator_spring.cxx,
 * ivp_controller_motion.cxx) and the actuator constructors' anchor setup
 * (IVP_Anchor::init_anchor, IVP_Actuator_Two_Point / _Four_Point).
 *
 * Ownership: the factory keeps the created states of an environment in a
 * registry attached to it through a global object listener.  The listener
 * deletes a factory controller when one of its objects is deleted (the
 * reference IVP_Actuator::anchor_will_be_deleted_event,
 * IVP_Anchor_Check_Dist::hull_manager_is_going_to_be_deleted_event,
 * IVP_Controller_Motion / IVP_Constraint_Local::core_is_going_to_be_deleted_event)
 * and all of them when the environment is deleted.
 */
#include <ivp/controllers/ivp_controller_factory.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/events/ivp_listener_object.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>

#include <stdio.h>
#include <string.h>
#include "controllers/ivp_controller_ref.h"
#include "dynamics/ivp_env_pipeline.h"

/* ── Template anchors ───────────────────────────────────────────────────── */

void ivp_template_anchor_set_position_ws(ivp_template_anchor_t *anchor, ivp_object_t *obj,
                                         const ivp_vec3r_t *coords_ws)
{
    anchor->object = obj;
    anchor->coords_world = *coords_ws;
}

void ivp_template_anchor_set_position_os(ivp_template_anchor_t *anchor, ivp_object_t *obj,
                                         const ivp_vec3r_t *coords_os)
{
    anchor->object = obj;
    /* IVP_U_Float_Point coords_os; transform_position_to_world_coords */
    ivp_vec3f_t c;
    lvf_set_p(&c, coords_os);
    if (!obj || !obj->owner) {
        lvp_set_f(&anchor->coords_world, &c); /* static object frame == world frame */
        return;
    }
    ivp_transform_r_t tmp;
    lvm_vmult4_fp(ivp_ctl_cache_m_world_f_object(obj, &tmp), &c, &anchor->coords_world);
}

void ivp_template_anchor_set_position_cs(ivp_template_anchor_t *anchor, ivp_object_t *obj,
                                         const ivp_vec3r_t *coords_cs)
{
    ivp_transform_r_t mat;
    ivp_ctl_calc_m_core_f_object(obj, &mat);
    ivp_vec3f_t c, coords_os;
    lvf_set_p(&c, coords_cs);
    lvm_vimult4_ff(&mat, &c, &coords_os);
    ivp_vec3r_t os;
    lvp_set_f(&os, &coords_os);
    ivp_template_anchor_set_position_os(anchor, obj, &os);
}

/* ── Template constructors ──────────────────────────────────────────────── */

void ivp_template_force_init(ivp_template_force_t *t)
{
    memset(t, 0, sizeof(*t));
    t->push_first_object = true;
    t->push_second_object = false;
}

void ivp_template_torque_init(ivp_template_torque_t *t) { memset(t, 0, sizeof(*t)); }

void ivp_template_rot_mot_init(ivp_template_rot_mot_t *t) { memset(t, 0, sizeof(*t)); }

void ivp_template_spring_init(ivp_template_spring_t *t)
{
    memset(t, 0, sizeof(*t));
    t->max_len_exceed_type = IVP_SFE_NONE;
    t->break_max_len = IVP_FLOAT_MAX; /* P_FLOAT_MAX */
}

void ivp_template_suspension_init(ivp_template_suspension_t *t)
{
    memset(t, 0, sizeof(*t));
    ivp_template_spring_init(&t->spring);
    t->spring.break_max_len = 10e8f;
    t->spring_dampening_compression = 0.0f;
    t->max_body_force = 0.0f;
}

void ivp_template_stabilizer_init(ivp_template_stabilizer_t *t) { memset(t, 0, sizeof(*t)); }

void ivp_template_check_dist_init(ivp_template_check_dist_t *t) { memset(t, 0, sizeof(*t)); }

void ivp_template_controller_motion_init(ivp_template_controller_motion_t *t)
{
    memset(t, 0, sizeof(*t));
    t->max_translation_force.k[0] = 1e6f;
    t->max_translation_force.k[1] = 1e6f;
    t->max_translation_force.k[2] = 1e6f;
    t->max_torque = 1e6f;
    t->force_factor = 0.8f;
    t->damp_factor = 1.0f;
    t->torque_factor = 0.8f;
    t->angular_damp_factor = 1.0f;
}

/* ── Registry ───────────────────────────────────────────────────────────── */

typedef enum fac_kind {
    FAC_FORCE, FAC_TORQUE, FAC_ROT_MOT, FAC_SPRING, FAC_SUSPENSION, FAC_STABILIZER,
    FAC_CHECK_DIST, FAC_MOTION, FAC_CONSTRAINT
} fac_kind_t;

typedef struct fac_entry {
    fac_kind_t              kind;
    void                   *state;
    size_t                  state_size;
    ivp_controller_handle_t handle;
    void                   *client_data;
    ivp_object_t           *objects[4];   /* objects whose deletion deletes the controller */
    int                     n_objects;
} fac_entry_t;

typedef struct fac_registry {
    ivp_listener_object_t listener;      /* first member */
    ivp_environment_t    *env;
    fac_entry_t          *entries;
    int                   n, cap;
} fac_registry_t;

static void fac_object_deleted(ivp_listener_object_t *self, ivp_event_object_t *event);
static void fac_environment_deleted(ivp_listener_object_t *self, struct ivp_environment *env);

static fac_registry_t *fac_find_registry(ivp_environment_t *env)
{
    if (!env || !env->pl) return NULL;
    ivp_pvec_t *v = &env->pl->global_object_listeners;
    for (int i = 0; i < ivp_pvec_len(v); i++) {
        ivp_listener_object_t *l = (ivp_listener_object_t *)ivp_pvec_at(v, i);
        if (l && l->event_object_deleted == fac_object_deleted) return (fac_registry_t *)l;
    }
    return NULL;
}

static fac_registry_t *fac_registry(ivp_environment_t *env)
{
    fac_registry_t *r = fac_find_registry(env);
    if (r) return r;
    r = (fac_registry_t *)ivp_alloc(env->allocator, sizeof(*r));
    if (!r) return NULL;
    memset(r, 0, sizeof(*r));
    r->listener.event_object_deleted = fac_object_deleted;
    r->listener.event_environment_deleted = fac_environment_deleted;
    r->listener.user_data = r;
    r->env = env;
    ivp_environment_add_listener_object_global(env, &r->listener);
    return r;
}

static fac_entry_t *fac_find_entry(fac_registry_t *r, const void *state)
{
    if (!r || !state) return NULL;
    for (int i = 0; i < r->n; i++) {
        if (r->entries[i].state == state) return &r->entries[i];
    }
    return NULL;
}

/* allocate a zeroed state and its registry entry */
static void *fac_new(ivp_environment_t *env, fac_kind_t kind, size_t size, void *client_data)
{
    fac_registry_t *r = fac_registry(env);
    if (!r) return NULL;
    if (r->n == r->cap) {
        int nc = r->cap ? r->cap * 2 : 16;
        fac_entry_t *ne = (fac_entry_t *)ivp_alloc(env->allocator, (size_t)nc * sizeof(*ne));
        if (!ne) return NULL;
        if (r->entries) {
            memcpy(ne, r->entries, (size_t)r->n * sizeof(*ne));
            ivp_free(env->allocator, r->entries, (size_t)r->cap * sizeof(*ne));
        }
        r->entries = ne;
        r->cap = nc;
    }
    void *state = ivp_alloc(env->allocator, size);
    if (!state) return NULL;
    memset(state, 0, size);
    fac_entry_t *e = &r->entries[r->n++];
    memset(e, 0, sizeof(*e));
    e->kind = kind;
    e->state = state;
    e->state_size = size;
    e->handle = ivp_controller_handle_invalid();
    e->client_data = client_data;
    return state;
}

static void fac_add_object(fac_entry_t *e, ivp_object_t *obj)
{
    if (!obj || e->n_objects >= 4) return;
    for (int i = 0; i < e->n_objects; i++) {
        if (e->objects[i] == obj) return;
    }
    e->objects[e->n_objects++] = obj;
}

/* the reference destructor: silent removal, active values, free */
static void fac_destroy_entry(fac_registry_t *r, int index)
{
    fac_entry_t e = r->entries[index];
    r->entries[index] = r->entries[--r->n];
    ivp_environment_t *env = r->env;
    if (ivp_controller_handle_is_valid(e.handle)) {
        /* ~IVP_Actuator_Two_Point: remove_controller_from_environment(this, TRUE);
         * ~IVP_Controller_Motion: remove_controller_from_core (a stale handle
         * of a controller the environment already removed is ignored) */
        (void)ivp_environment_remove_dependent_controller(env, e.handle, true);
    }
    switch (e.kind) {
    case FAC_FORCE:  ivp_force_actuator_unbind_active((ivp_force_actuator_state_t *)e.state); break;
    case FAC_TORQUE: ivp_torque_actuator_unbind_active((ivp_torque_actuator_state_t *)e.state); break;
    case FAC_ROT_MOT: ivp_rot_mot_unbind_active((ivp_rot_mot_state_t *)e.state); break;
    case FAC_SPRING: ivp_spring_unbind_active((ivp_spring_state_t *)e.state); break;
    case FAC_CHECK_DIST: ivp_check_dist_detach((ivp_check_dist_state_t *)e.state); break;
    case FAC_SUSPENSION:
    case FAC_STABILIZER:
    case FAC_MOTION:
    case FAC_CONSTRAINT:
    default:
        break;
    }
    ivp_free(env->allocator, e.state, e.state_size);
}

/* drop a state whose construction failed */
static void fac_abort(ivp_environment_t *env, void *state)
{
    fac_registry_t *r = fac_find_registry(env);
    fac_entry_t *e = fac_find_entry(r, state);
    if (e) fac_destroy_entry(r, (int)(e - r->entries));
}

static void fac_object_deleted(ivp_listener_object_t *self, ivp_event_object_t *event)
{
    fac_registry_t *r = (fac_registry_t *)self;
    ivp_object_t *obj = event->real_object;
    for (int i = r->n - 1; i >= 0; i--) {
        if (i >= r->n) continue;
        fac_entry_t *e = &r->entries[i];
        bool uses = false;
        for (int k = 0; k < e->n_objects; k++) uses = uses || e->objects[k] == obj;
        if (uses) fac_destroy_entry(r, i);
    }
}

static void fac_environment_deleted(ivp_listener_object_t *self, struct ivp_environment *env)
{
    fac_registry_t *r = (fac_registry_t *)self;
    while (r->n > 0) fac_destroy_entry(r, r->n - 1);
    ivp_environment_remove_listener_object_global(env, &r->listener);
    if (r->entries) ivp_free(env->allocator, r->entries, (size_t)r->cap * sizeof(*r->entries));
    ivp_free(env->allocator, r, sizeof(*r));
}

/* ── Anchors (IVP_Anchor::init_anchor) ──────────────────────────────────── */

/* The anchor object as stored in libivp actuator states (NULL for the
 * static object) and the anchor's core_pos (float exact). */
static ivp_object_t *fac_anchor(ivp_environment_t *env, const ivp_template_anchor_t *ta, ivp_vec3r_t *core_pos,
                                ivp_anchor_t *anchor)
{
    ivp_object_t *obj = ta->object;
    if (obj == ivp_environment_get_static_object(env)) obj = NULL;
    ivp_vec3f_t cp, object_pos;
    ivp_ctl_anchor_core_pos_ws(obj, &ta->coords_world, &cp, &object_pos);
    lvp_set_f(core_pos, &cp);
    ivp_ctl_anchor_set(anchor, obj, &object_pos);
    return obj;
}

static bool fac_anchors_valid(ivp_template_anchor_t *const *anchors, int n)
{
    for (int i = 0; i < n; i++) {
        if (!anchors[i]) return false;
    }
    return true;
}

/* anchor->l_anchor_object->get_original_core() */
static ivp_core_t *fac_original_core(ivp_environment_t *env, ivp_object_t *obj)
{
    return ivp_ctl_anchor_original_core(obj, env);
}

/* ── create_force ───────────────────────────────────────────────────────── */

ivp_force_actuator_state_t *ivp_controller_factory_create_force(ivp_environment_t *env,
                                                                const ivp_template_force_t *templ)
{
    if (!env || !templ || !fac_anchors_valid(templ->anchors, 2)) return NULL;
    ivp_force_actuator_state_t *st = (ivp_force_actuator_state_t *)fac_new(env, FAC_FORCE, sizeof(*st), templ->client_data);
    if (!st) return NULL;
    fac_entry_t *e = fac_find_entry(fac_find_registry(env), st);
    ivp_force_actuator_state_init(st);
    /* IVP_Actuator_Two_Point: anchors, controlled cores, announce */
    st->obj_a = fac_anchor(env, templ->anchors[0], &st->anchor_a_os, &st->anchors[0]);
    st->obj_b = fac_anchor(env, templ->anchors[1], &st->anchor_b_os, &st->anchors[1]);
    fac_add_object(e, st->obj_a);
    fac_add_object(e, st->obj_b);
    /* IVP_Actuator_Force */
    st->force = (ivp_real_t)templ->force;
    st->push_first_object = templ->push_first_object;
    st->push_second_object = templ->push_second_object;
    st->active = true;
    e->handle = ivp_environment_add_controller(env, IVP_CP_ACTUATOR, ivp_controller_force_actuator, st);
    if (!ivp_controller_handle_is_valid(e->handle)) {
        fac_abort(env, st);
        return NULL;
    }
    /* IVP_Actuator_Force_Active */
    if (templ->active_float_force) ivp_force_actuator_bind_active(st, templ->active_float_force);
    return st;
}

/* ── create_torque ──────────────────────────────────────────────────────── */

ivp_torque_actuator_state_t *ivp_controller_factory_create_torque(ivp_environment_t *env,
                                                                  const ivp_template_torque_t *templ)
{
    if (!env || !templ || !fac_anchors_valid(templ->anchors, 2)) return NULL;
    ivp_torque_actuator_state_t *st = (ivp_torque_actuator_state_t *)fac_new(env, FAC_TORQUE, sizeof(*st), templ->client_data);
    if (!st) return NULL;
    fac_entry_t *e = fac_find_entry(fac_find_registry(env), st);
    ivp_torque_actuator_state_init(st);
    ivp_vec3r_t cp0, cp1;
    ivp_object_t *obj0 = fac_anchor(env, templ->anchors[0], &cp0, NULL);
    ivp_object_t *obj1 = fac_anchor(env, templ->anchors[1], &cp1, NULL);
    fac_add_object(e, obj0);
    fac_add_object(e, obj1);
    st->obj = obj0; /* the torque acts on anchor 0's object */
    st->max_rotation_speed = (ivp_real_t)templ->max_rotation_speed;
    st->current_speed = IVP_REAL(0.0); /* rot_speed_out */
    e->handle = ivp_environment_add_controller(env, IVP_CP_ACTUATOR, ivp_controller_torque_actuator, st);
    if (!ivp_controller_handle_is_valid(e->handle)) {
        fac_abort(env, st);
        return NULL;
    }
    if (templ->active_float_rotation_speed_out) {
        ivp_torque_actuator_set_rotation_speed_out(st, templ->active_float_rotation_speed_out);
    }
    if (fac_original_core(env, obj0) != fac_original_core(env, obj1)) {
        ivp_debug_printf("Both Anchors of a Torque_Actuator must be attached to just one object.\n");
        IVP_CORE(); /* ivp_actuator.cxx:695 */
    }
    /* axis_in_core_coord_system.subtract(core_pos1, core_pos0); normize() */
    {
        ivp_vec3f_t c0 = ivp_ctl_f(&cp0), c1 = ivp_ctl_f(&cp1), axis_cs;
        lvf_subtract2(&axis_cs, &c1, &c0);
        (void)lvf_normize(&axis_cs);
        st->axis_os = ivp_ctl_r(&axis_cs);
    }
    st->torque = (ivp_real_t)templ->torque;
    st->active = true;
    if (templ->torque != 0.0f) (void)ivp_environment_ensure_controller_in_simulation(env, e->handle);
    /* IVP_Actuator_Torque_Active: max_rotation_speed, then torque */
    if (templ->active_float_torque || templ->active_float_max_rotation_speed) {
        ivp_torque_actuator_bind_active(st, templ->active_float_torque, templ->active_float_max_rotation_speed);
    }
    return st;
}

/* ── create_rotmot ──────────────────────────────────────────────────────── */

ivp_rot_mot_state_t *ivp_controller_factory_create_rotmot(ivp_environment_t *env,
                                                          const ivp_template_rot_mot_t *templ)
{
    if (!env || !templ || !fac_anchors_valid(templ->anchors, 2)) return NULL;
    ivp_rot_mot_state_t *st = (ivp_rot_mot_state_t *)fac_new(env, FAC_ROT_MOT, sizeof(*st), templ->client_data);
    if (!st) return NULL;
    fac_entry_t *e = fac_find_entry(fac_find_registry(env), st);
    ivp_rot_mot_state_init(st);
    st->obj_a = fac_anchor(env, templ->anchors[0], &st->anchor_a_os, &st->anchors[0]);
    st->obj_b = fac_anchor(env, templ->anchors[1], &st->anchor_b_os, &st->anchors[1]);
    fac_add_object(e, st->obj_a);
    fac_add_object(e, st->obj_b);
    st->max_rotation_speed = (ivp_real_t)templ->max_rotation_speed;
    st->power = (ivp_real_t)templ->power;
    st->max_torque = (ivp_real_t)templ->max_torque;
    st->current_speed = IVP_REAL(0.0);
    st->active = true;
    e->handle = ivp_environment_add_controller(env, IVP_CP_ACTUATOR, ivp_controller_rot_mot, st);
    if (!ivp_controller_handle_is_valid(e->handle)) {
        fac_abort(env, st);
        return NULL;
    }
    if (templ->active_float_rotation_speed_out) {
        ivp_rot_mot_set_rotation_speed_out(st, templ->active_float_rotation_speed_out);
    }
    if (fac_original_core(env, st->obj_a) != fac_original_core(env, st->obj_b)) {
        ivp_debug_printf("Rot Mot Actuation has to be fixed on one object\n");
    }
    /* IVP_Actuator_Rot_Mot_Active: max_rotation_speed, power, max_torque */
    if (templ->active_float_max_torque || templ->active_float_max_rotation_speed || templ->active_float_power) {
        ivp_rot_mot_bind_active(st, templ->active_float_max_rotation_speed, templ->active_float_power,
                                templ->active_float_max_torque);
    }
    return st;
}

/* ── create_spring / create_suspension ──────────────────────────────────── */

/* IVP_Actuator_Spring constructor: spring_values_factor of the anchors */
static float fac_spring_values_factor(ivp_object_t *obj_a, const ivp_vec3r_t *cp_a,
                                      ivp_object_t *obj_b, const ivp_vec3r_t *cp_b)
{
    ivp_spring_state_t tmp;
    ivp_spring_state_init(&tmp);
    tmp.obj_a = obj_a;
    tmp.obj_b = obj_b;
    tmp.anchor_a_os = *cp_a;
    tmp.anchor_b_os = *cp_b;
    ivp_spring_state_make_relative(&tmp);
    return (float)tmp.spring_values_factor;
}

ivp_spring_state_t *ivp_controller_factory_create_spring(ivp_environment_t *env,
                                                         const ivp_template_spring_t *templ)
{
    if (!env || !templ || !fac_anchors_valid(templ->anchors, 2)) return NULL;
    ivp_spring_state_t *st = (ivp_spring_state_t *)fac_new(env, FAC_SPRING, sizeof(*st), templ->client_data);
    if (!st) return NULL;
    fac_entry_t *e = fac_find_entry(fac_find_registry(env), st);
    ivp_spring_state_init(st);
    st->obj_a = fac_anchor(env, templ->anchors[0], &st->anchor_a_os, &st->anchors[0]);
    st->obj_b = fac_anchor(env, templ->anchors[1], &st->anchor_b_os, &st->anchors[1]);
    fac_add_object(e, st->obj_a);
    fac_add_object(e, st->obj_b);
    st->rest_length = (ivp_real_t)templ->spring_len;
    st->break_max_len = (ivp_real_t)templ->break_max_len;
    st->max_len_exceed_type = templ->max_len_exceed_type;
    /* spring constants (multiplied with the relative factor) */
    st->stiffness = (ivp_real_t)templ->spring_constant;
    st->damping = (ivp_real_t)templ->spring_damp;
    st->rel_pos_damp = (ivp_real_t)templ->rel_pos_damp;
    if (templ->spring_values_are_relative) ivp_spring_state_make_relative(st);
    st->spring_force_only_on_stretch = templ->spring_force_only_on_stretch;
    e->handle = ivp_environment_add_controller(env, IVP_CP_ACTUATOR, ivp_controller_spring, st);
    if (!ivp_controller_handle_is_valid(e->handle)) {
        fac_abort(env, st);
        return NULL;
    }
    /* IVP_Actuator_Spring_Active: len, constant, damp, rel_pos_damp */
    if (templ->active_float_spring_len || templ->active_float_spring_constant || templ->active_float_spring_damp ||
        templ->active_float_spring_rel_pos_damp) {
        ivp_spring_bind_active(st, templ->active_float_spring_len, templ->active_float_spring_constant,
                               templ->active_float_spring_damp, templ->active_float_spring_rel_pos_damp);
    }
    return st;
}

ivp_suspension_state_t *ivp_controller_factory_create_suspension(ivp_environment_t *env,
                                                                 const ivp_template_suspension_t *templ)
{
    if (!env || !templ || !fac_anchors_valid(templ->spring.anchors, 2)) return NULL;
    ivp_suspension_state_t *st =
        (ivp_suspension_state_t *)fac_new(env, FAC_SUSPENSION, sizeof(*st), templ->spring.client_data);
    if (!st) return NULL;
    fac_entry_t *e = fac_find_entry(fac_find_registry(env), st);
    ivp_suspension_state_init(st);
    st->obj_a = fac_anchor(env, templ->spring.anchors[0], &st->anchor_a_os, &st->anchors[0]);
    st->obj_b = fac_anchor(env, templ->spring.anchors[1], &st->anchor_b_os, &st->anchors[1]);
    fac_add_object(e, st->obj_a);
    fac_add_object(e, st->obj_b);
    float factor = 1.0f;
    if (templ->spring.spring_values_are_relative) {
        factor = fac_spring_values_factor(st->obj_a, &st->anchor_a_os, st->obj_b, &st->anchor_b_os);
    }
    st->spring_values_factor = (ivp_real_t)factor;
    st->rest_length = (ivp_real_t)templ->spring.spring_len;
    st->stiffness = (ivp_real_t)(templ->spring.spring_constant * factor);
    st->damping = (ivp_real_t)(templ->spring.spring_damp * factor);
    /* IVP_Actuator_Suspension */
    st->damping_compression = (ivp_real_t)(templ->spring_dampening_compression * factor);
    st->max_body_force = (ivp_real_t)templ->max_body_force;
    e->handle = ivp_environment_add_controller(env, IVP_CP_ACTUATOR, ivp_controller_suspension, st);
    if (!ivp_controller_handle_is_valid(e->handle)) {
        fac_abort(env, st);
        return NULL;
    }
    return st;
}

/* ── create_stabilizer ──────────────────────────────────────────────────── */

ivp_stabilizer_state_t *ivp_controller_factory_create_stabilizer(ivp_environment_t *env,
                                                                 const ivp_template_stabilizer_t *templ)
{
    if (!env || !templ || !fac_anchors_valid(templ->anchors, 4)) return NULL;
    ivp_stabilizer_state_t *st =
        (ivp_stabilizer_state_t *)fac_new(env, FAC_STABILIZER, sizeof(*st), templ->client_data);
    if (!st) return NULL;
    fac_entry_t *e = fac_find_entry(fac_find_registry(env), st);
    ivp_stabilizer_state_init(st);
    /* IVP_Actuator_Four_Point: anchors only, the controller is not announced */
    for (int i = 0; i < 4; i++) {
        st->obj[i] = fac_anchor(env, templ->anchors[i], &st->anchor_os[i], &st->anchors[i]);
        fac_add_object(e, st->obj[i]);
    }
    st->stabi_constant = (ivp_real_t)templ->stabi_constant;
    st->active = true;
    return st;
}

ivp_controller_handle_t ivp_controller_factory_announce_stabilizer(ivp_environment_t *env,
                                                                   ivp_stabilizer_state_t *stabilizer)
{
    fac_entry_t *e = fac_find_entry(fac_find_registry(env), stabilizer);
    if (!e || e->kind != FAC_STABILIZER) return ivp_controller_handle_invalid();
    if (!ivp_controller_handle_is_valid(e->handle)) {
        e->handle = ivp_environment_add_controller(env, IVP_CP_ACTUATOR, ivp_controller_stabilizer, stabilizer);
    }
    return e->handle;
}

/* ── create_check_dist ──────────────────────────────────────────────────── */

ivp_check_dist_state_t *ivp_controller_factory_create_check_dist(ivp_environment_t *env,
                                                                 const ivp_template_check_dist_t *templ)
{
    if (!env || !templ || !templ->objects[0] || !templ->objects[1]) return NULL;
    ivp_check_dist_state_t *st =
        (ivp_check_dist_state_t *)fac_new(env, FAC_CHECK_DIST, sizeof(*st), templ->client_data);
    if (!st) return NULL;
    fac_entry_t *e = fac_find_entry(fac_find_registry(env), st);
    ivp_check_dist_state_setup(st, templ->objects[0], &templ->position_world_space[0], templ->objects[1],
                               &templ->position_world_space[1], (ivp_real_t)templ->range);
    st->mod_is_outside = templ->mod_is_outside;
    fac_add_object(e, templ->objects[0]);
    fac_add_object(e, templ->objects[1]);
    /* anchors into the hull managers, then hull_limit_exceeded_event() */
    if (!ivp_check_dist_attach(st, env)) {
        fac_abort(env, st);
        return NULL;
    }
    return st;
}

/* ── create_controller_motion ───────────────────────────────────────────── */

ivp_motion_state_t *ivp_controller_factory_create_controller_motion(ivp_environment_t *env, ivp_object_t *obj,
                                                                    const ivp_template_controller_motion_t *templ)
{
    if (!obj || !templ) return NULL;
    env = obj->owner; /* the reference takes l_environment from the object */
    if (!env) return NULL;
    ivp_motion_state_t *st = (ivp_motion_state_t *)fac_new(env, FAC_MOTION, sizeof(*st), NULL);
    if (!st) return NULL;
    fac_entry_t *e = fac_find_entry(fac_find_registry(env), st);
    /* target at the origin / identity, IVP_MOTION_TARGET on both channels */
    ivp_motion_state_init(st);
    for (int k = 0; k < 3; k++) {
        st->max_translation_force.k[k] = (ivp_real_t)templ->max_translation_force.k[k];
        st->max_torque.k[k] = (ivp_real_t)templ->max_torque;
    }
    st->force_factor = (ivp_real_t)templ->force_factor;
    st->damp_factor = (ivp_real_t)templ->damp_factor;
    st->torque_factor = (ivp_real_t)templ->torque_factor;
    st->angular_damp_factor = (ivp_real_t)templ->angular_damp_factor;
    ivp_motion_state_setup(st, obj);
    fac_add_object(e, obj);
    /* l_environment->get_controller_manager()->add_controller_to_core(this, core) */
    e->handle = ivp_environment_create_core_controller(env, IVP_CP_MOTION, ivp_controller_motion, st);
    if (!ivp_controller_handle_is_valid(e->handle) || !ivp_environment_add_controller_to_core(env, e->handle, obj)) {
        fac_abort(env, st);
        return NULL;
    }
    return st;
}

/* ── create_constraint ──────────────────────────────────────────────────── */

ivp_constraint_state_t *ivp_controller_factory_create_constraint(ivp_environment_t *env,
                                                                 const ivp_constraint_template_t *templ)
{
    if (!env || !templ) return NULL;
    if (!templ->obj_r && !templ->obj_a) return NULL;
    ivp_constraint_template_t checked_tmpl = *templ;
    if (checked_tmpl.force_factor <= (ivp_real_t)P_FLOAT_EPS) checked_tmpl.force_factor = (ivp_real_t)P_FLOAT_EPS;
    ivp_constraint_state_t *st = (ivp_constraint_state_t *)fac_new(env, FAC_CONSTRAINT, sizeof(*st), NULL);
    if (!st) return NULL;
    fac_entry_t *e = fac_find_entry(fac_find_registry(env), st);
    /* new IVP_Constraint_Local(checked_tmpl) */
    ivp_constraint_init_from_template(st, &checked_tmpl);
    fac_add_object(e, checked_tmpl.obj_r);
    fac_add_object(e, checked_tmpl.obj_a);
    e->handle = ivp_environment_add_controller(env, IVP_CP_CONSTRAINTS, ivp_controller_constraint, st);
    if (!ivp_controller_handle_is_valid(e->handle)) {
        fac_abort(env, st);
        return NULL;
    }
    return st;
}

/* ── Factory controllers ────────────────────────────────────────────────── */

bool ivp_controller_factory_delete(ivp_environment_t *env, void *controller)
{
    fac_registry_t *r = fac_find_registry(env);
    fac_entry_t *e = fac_find_entry(r, controller);
    if (!e) return false;
    fac_destroy_entry(r, (int)(e - r->entries));
    return true;
}

ivp_controller_handle_t ivp_controller_factory_get_handle(ivp_environment_t *env, const void *controller)
{
    fac_entry_t *e = fac_find_entry(fac_find_registry(env), controller);
    return e ? e->handle : ivp_controller_handle_invalid();
}

void *ivp_controller_factory_get_client_data(ivp_environment_t *env, const void *controller)
{
    fac_entry_t *e = fac_find_entry(fac_find_registry(env), controller);
    return e ? e->client_data : NULL;
}
