/* ivp_forcefield.c -- Force field controller implementation
 *
 * Set-driven part: port of IVP_Forcefield (ivp_controller/ivp_forcefield.cxx).
 */

#include <ivp/controllers/ivp_forcefield.h>
#include <ivp/controllers/ivp_attacher_to_cores.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include "controllers/ivp_controller_ref.h"
#include <string.h>

#include "dynamics/ivp_env_internal.h"
#include "dynamics/ivp_sim_internal.h"

void ivp_forcefield_state_init(ivp_forcefield_state_t *state,
                                ivp_forcefield_mode_t mode,
                                const ivp_vec3r_t *value)
{
    memset(state, 0, sizeof(*state));
    state->mode   = mode;
    if (value) state->value = *value;
    state->active = true;
}

void ivp_forcefield_set_objects(ivp_forcefield_state_t *state,
                                ivp_object_t *const *objects, int count)
{
    state->objects = objects;
    state->object_count = objects ? count : 0;
    for (int i = 0; i < state->object_count; i++) {
        ivp_object_t *o = objects[i];
        if (o && !o->physical_core->physical_unmoveable && o->owner) {
            (void)ivp_environment_ensure_core_in_simulation(o->owner, o);
        }
    }
}

void ivp_forcefield_set_custom(ivp_forcefield_state_t *state,
                               ivp_forcefield_custom_fn_t fn, void *ctx)
{
    state->mode = IVP_FORCEFIELD_CUSTOM;
    state->custom_fn = fn;
    state->custom_ctx = ctx;
}

/* ── Reference set-driven IVP_Forcefield ───────────────────────────────── */

static ivp_forcefield_state_t *ff_of_listener(ivp_listener_object_t *l)
{
    return (ivp_forcefield_state_t *)l->user_data;
}

static void ff_restore_core_order(ivp_forcefield_state_t *state)
{
    if (ivp_build_uses_reference_internal_state()) return;
    ivp_core_vector_sort_by_identity(&state->attached_cores);
}

static bool ff_attach_core(ivp_forcefield_state_t *state,
                           ivp_core_t *core)
{
    int index = ivp_pvec_index_of(&state->attached_cores, core);
    bool added = index < 0;
    if (added) {
        index = ivp_pvec_add(&state->attached_cores, core);
        if (index < 0) return false;
    }
    if (!ivp_environment_add_controller_to_core(
            state->environment, state->controller,
            ivp_attacher_core_object(core))) {
        if (added) {
            ivp_pvec_remove_at(&state->attached_cores, index);
        }
        return false;
    }
    ff_restore_core_order(state);
    return true;
}

/* element_added: attached_cores.install(elem); add_controller_to_core */
static void ff_element_added(void *ctx, ivp_set_active_t *set, void *elem)
{
    (void)set;
    ivp_forcefield_state_t *state = (ivp_forcefield_state_t *)ctx;
    ivp_core_t *core = (ivp_core_t *)elem;
    if (!core || !state->environment) return;
    (void)ff_attach_core(state, core);
}

typedef struct ff_attach_set_context {
    ivp_forcefield_state_t *state;
    bool success;
} ff_attach_set_context_t;

static void ff_attach_set_core(void *context, ivp_core_t *core)
{
    ff_attach_set_context_t *attach =
        (ff_attach_set_context_t *)context;
    if (attach->success && !ff_attach_core(attach->state, core)) {
        attach->success = false;
    }
}

/* element_removed: remove_controller_from_core; attached_cores.remove.
 * Cores the field is not attached to are ignored (never dereferenced). */
static void ff_element_removed(void *ctx, ivp_set_active_t *set, void *elem)
{
    (void)set;
    ivp_forcefield_state_t *state = (ivp_forcefield_state_t *)ctx;
    ivp_core_t *core = (ivp_core_t *)elem;
    if (!core) return;
    int idx = ivp_pvec_index_of(&state->attached_cores, core);
    if (idx < 0) return;
    if (state->environment) {
        (void)ivp_environment_remove_controller_from_core(state->environment, state->controller,
                                                          ivp_attacher_core_object(core));
    }
    ivp_pvec_remove_at(&state->attached_cores, idx);
}

/* core_is_going_to_be_deleted_event */
static void ff_core_is_going_to_be_deleted(ivp_forcefield_state_t *state, ivp_core_t *core)
{
    if (ivp_pvec_index_of(&state->attached_cores, core) < 0) return; /* not a core of the field */
    if (!state->set_is_going_to_be_deleted && state->set_of_cores &&
        ivp_set_find(&state->set_of_cores->set, core)) {
        ivp_set_active_remove(state->set_of_cores, core); /* notifies element_removed */
        return;
    }
    ff_element_removed(state, NULL, core);
}

static void ff_object_deleted(ivp_listener_object_t *l, ivp_event_object_t *ev)
{
    ivp_forcefield_state_t *state = ff_of_listener(l);
    if (!state->environment || !ev->real_object) return;
    ivp_core_t *core = ev->real_object->physical_core;
    if (!core || ivp_pvec_len(&core->objects) > 1) return;   /* the core outlives the object */
    ff_core_is_going_to_be_deleted(state, core);
}

/* The environment deletes all its cores: handle them now, while the
 * environment is still valid, then forget the environment (its controller
 * slots are removed by the environment). */
static void ff_environment_deleted(ivp_listener_object_t *l, struct ivp_environment *env)
{
    (void)env;
    ivp_forcefield_state_t *state = ff_of_listener(l);
    if (!state->environment) return;
    ff_restore_core_order(state);
    while (ivp_pvec_len(&state->attached_cores) > 0) {
        int n = ivp_pvec_len(&state->attached_cores);
        ff_core_is_going_to_be_deleted(state, (ivp_core_t *)ivp_pvec_at(&state->attached_cores, n - 1));
        if (ivp_pvec_len(&state->attached_cores) == n) ivp_pvec_remove_at(&state->attached_cores, n - 1);
    }
    ivp_environment_remove_listener_object_global(state->environment, &state->object_listener);
    state->environment = NULL;
}

/* ~IVP_Forcefield (+ libivp: unregister the controller slot) */
static void ff_release(ivp_forcefield_state_t *state)
{
    ff_restore_core_order(state);
    for (int i = ivp_pvec_len(&state->attached_cores) - 1; i >= 0; --i) {
        ivp_core_t *my_core = (ivp_core_t *)ivp_pvec_at(&state->attached_cores, i);
        if (my_core && state->environment) {
            (void)ivp_environment_remove_controller_from_core(state->environment, state->controller,
                                                              ivp_attacher_core_object(my_core));
        }
    }
    ivp_pvec_clear(&state->attached_cores);
    if (!state->set_is_going_to_be_deleted && state->set_of_cores) {
        ivp_set_active_remove_listener(state->set_of_cores, &state->set_listener);
    }
    state->set_of_cores = NULL;
    ivp_environment_t *env = state->environment;
    ivp_controller_handle_t controller = state->controller;
    state->environment = NULL;
    state->controller = ivp_controller_handle_invalid();
    state->set_driven = false;
    if (env) {
        ivp_environment_remove_listener_object_global(env, &state->object_listener);
        (void)ivp_environment_remove_controller(env, controller);
    }
}

/* pset_is_going_to_be_deleted: the set does not remove its elements */
static void ff_pset_is_going_to_be_deleted(void *ctx, ivp_set_active_t *set)
{
    (void)set;
    ivp_forcefield_state_t *state = (ivp_forcefield_state_t *)ctx;
    state->set_is_going_to_be_deleted = true;
    state->set_of_cores = NULL;
    ff_release(state); /* P_DELETE_THIS(this) */
    if (state->deleted_fn) state->deleted_fn(state->deleted_ctx, state);
}

bool ivp_forcefield_attach_set(ivp_forcefield_state_t *state, ivp_environment_t *env,
                               ivp_set_active_t *set_of_cores, bool owner_of_set)
{
    if (!state || !env || !set_of_cores) return false;
    ivp_forcefield_destroy(state);
    state->allocator = *ivp_env_get_allocator(env);
    state->allocator_bound = true;
    ivp_pvec_init_with_allocator(&state->attached_cores, &state->allocator);
    state->controller = ivp_environment_create_core_controller(
        env, IVP_CP_ACTUATOR, ivp_controller_forcefield, state);
    if (!ivp_controller_handle_is_valid(state->controller)) return false;
    state->environment = env;
    state->set_of_cores = set_of_cores;
    state->i_am_owner_of_set_of_cores = owner_of_set;
    state->set_is_going_to_be_deleted = false;
    state->set_driven = true;
    state->objects = NULL;
    state->object_count = 0;

    state->set_listener.element_added = ff_element_added;
    state->set_listener.element_removed = ff_element_removed;
    state->set_listener.pset_is_going_to_be_deleted = ff_pset_is_going_to_be_deleted;
    state->set_listener.ctx = state;
    memset(&state->object_listener, 0, sizeof(state->object_listener));
    state->object_listener.event_object_deleted = ff_object_deleted;
    state->object_listener.event_environment_deleted = ff_environment_deleted;
    state->object_listener.user_data = state;
    ivp_environment_add_listener_object_global(env, &state->object_listener);

    if (!ivp_build_uses_reference_internal_state()) {
        ff_attach_set_context_t attach = { state, true };
        ivp_core_set_visit_by_identity(
            &set_of_cores->set, &state->allocator, NULL,
            ff_attach_set_core, &attach);
        if (!attach.success) {
            state->set_is_going_to_be_deleted = true;
            ff_release(state);
            return false;
        }
    } else {
        ivp_set_enumerator_t enumerator;
        ivp_set_enumerator_init(&enumerator, &set_of_cores->set);
        ivp_core_t *core;
        while ((core = (ivp_core_t *)ivp_set_enumerator_next(
                    &enumerator, &set_of_cores->set)) != NULL) {
            ff_element_added(state, set_of_cores, core);
        }
    }
    if (!ivp_set_active_add_listener(set_of_cores, &state->set_listener)) {
        state->set_is_going_to_be_deleted = true; /* not listening */
        ff_release(state);
        return false;
    }
    return true;
}

void ivp_forcefield_set_deleted_listener(ivp_forcefield_state_t *state,
                                         ivp_forcefield_deleted_fn_t fn, void *ctx)
{
    if (!state) return;
    state->deleted_fn = fn;
    state->deleted_ctx = ctx;
}

void ivp_forcefield_destroy(ivp_forcefield_state_t *state)
{
    if (!state || !state->set_driven) return;
    ff_release(state);
}

static bool forcefield_has_object(const ivp_forcefield_state_t *state, const ivp_object_t *obj)
{
    if (!state->objects) return true;
    for (int i = 0; i < state->object_count; i++) {
        if (state->objects[i] == obj) return true;
    }
    return false;
}

void ivp_controller_forcefield(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx)
{
    const ivp_real_t dt = event_sim->delta_time;
    ivp_forcefield_state_t *state = (ivp_forcefield_state_t *)ctx;
    if (!state || !state->active) return;

    if (!forcefield_has_object(state, obj)) return;

    ivp_core_t *core = obj->physical_core;
    if (core->physical_unmoveable || core->pinned) return;

    /* libivp force modes.  speed_change is an IVP_U_Float_Point in the
     * reference: accumulate with IVP_U_Float_Point::add_multiple. */
    ivp_vec3f_t sc = ivp_ctl_f(&core->speed_change);
    switch (state->mode) {
    case IVP_FORCEFIELD_CONSTANT: {
        /* Constant acceleration: dv = accel * dt (async push, priority IVP_CP_ACTUATOR = 1500) */
        ivp_vec3f_t accel = ivp_ctl_f(&state->value);
        lvf_add_multiple(&sc, &accel, (D)dt);
        core->speed_change = ivp_ctl_r(&sc);
        break;
    }
    case IVP_FORCEFIELD_FORCE: {
        /* Constant force: dv = F/m * dt (async_center_push_core_multiple_ws) */
        ivp_vec3f_t force = ivp_ctl_f(&state->value);
        lvf_add_multiple(&sc, &force, (D)((double)ivp_core_get_inv_mass_f(core) * (double)dt));
        core->speed_change = ivp_ctl_r(&sc);
        break;
    }
    case IVP_FORCEFIELD_DRAG: {
        /* Velocity-proportional drag: F = -drag * v
         * dv = -drag * v / m * dt (async push, priority IVP_CP_ACTUATOR = 1500) */
        ivp_vec3f_t drag, speed = ivp_core_speed_f(core), coeff = ivp_ctl_f(&state->value);
        lvf_set_pairwise_mult(&drag, &coeff, &speed);
        lvf_add_multiple(&sc, &drag, (D)(-(double)ivp_core_get_inv_mass_f(core) * (double)dt));
        core->speed_change = ivp_ctl_r(&sc);
        break;
    }
    case IVP_FORCEFIELD_CUSTOM:
        if (state->custom_fn) state->custom_fn(state->custom_ctx, event_sim, obj);
        break;
    }
}
