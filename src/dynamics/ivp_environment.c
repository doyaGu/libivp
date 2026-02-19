#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_environment_collision.h>
#include <ivp/dynamics/ivp_universe_manager.h>
#include <ivp/collision/ivp_collision_tolerance.h>
#include <ivp/collision/ivp_hull_manager.h>
#include <ivp/collision/ivp_mindist_persistent.h>
#include <ivp/collision/ivp_time_event_queue.h>
#include <ivp/base/ivp_assert.h>
#include <string.h>

#include "ivp_object_system.h"
#include "dynamics/ivp_environment_events.h"
#include "controllers/ivp_controller_system.h"
#include "events/ivp_event_system.h"

static bool ivp_environment_core_is_active(const ivp_core_t *core)
{
    return !core->physical_unmoveable && core->movement_state != IVP_MT_NOT_SIM;
}

static bool ivp_environment_core_has_pending_push(const ivp_core_t *core)
{
    const ivp_real_t lin_q = ivp_vec3r_quad_length(&core->speed_change);
    const ivp_real_t ang_q = ivp_vec3r_quad_length(&core->rot_speed_change);
    return lin_q > IVP_REAL_EPS || ang_q > IVP_REAL_EPS;
}

static void ivp_environment_emit_phase(ivp_environment_t *env, ivp_psi_phase_t phase, ivp_real_t dt)
{
    if (env->phase_hook) {
        env->phase_hook(env->phase_hook_ctx, env, phase, env->current_time, dt);
    }
}

static ivp_event_system_t *ivp_environment_get_event_system(const ivp_environment_t *env)
{
    return (ivp_event_system_t *)env->event_system;
}

static ivp_controller_system_t *ivp_environment_get_controller_system(const ivp_environment_t *env)
{
    return (ivp_controller_system_t *)env->controller_system;
}

static ivp_object_system_t *ivp_environment_get_object_system(const ivp_environment_t *env)
{
    return (ivp_object_system_t *)env->object_system;
}

static bool ivp_environment_delta_time_is_valid(ivp_real_t dt)
{
    return dt > IVP_REAL_EPS && dt < IVP_REAL_MAX;
}

static ivp_event_listener_slot_t *ivp_environment_get_event_listener_by_handle(
    ivp_event_system_t *es,
    ivp_event_listener_handle_t handle)
{
    if (!es || !ivp_event_listener_handle_is_valid(handle)) return NULL;
    ivp_handle_t h;
    h.slot = handle.slot;
    h.generation = handle.generation;
    return (ivp_event_listener_slot_t *)ivp_registry_get_by_handle(&es->registry, h);
}

void ivp_environment_emit_event_internal(ivp_environment_t *env,
                                         ivp_event_type_t type,
                                         ivp_real_t dt,
                                         int object_index,
                                         int controller_id)
{
    ivp_event_system_t *es = ivp_environment_get_event_system(env);
    if (!es) return;

    const int n = ivp_event_system_count(es);
    if (n <= 0) return;

    ivp_event_t evt;
    evt.type = type;
    evt.time = env->current_time;
    evt.dt = dt;
    evt.sim_unit_index = env->current_sim_unit_index;
    evt.object_index = object_index;
    evt.controller_id = controller_id;

    ivp_event_listener_handle_t local_handles[32];
    ivp_event_listener_handle_t *handles = local_handles;
    bool allocated = false;
    if (n > (int)(sizeof(local_handles) / sizeof(local_handles[0]))) {
        handles = (ivp_event_listener_handle_t *)ivp_alloc(
            env->allocator, (size_t)n * sizeof(ivp_event_listener_handle_t));
        if (!handles) {
            return;
        }
        allocated = true;
    }

    int handle_count = 0;
    for (int i = 0; i < n; i++) {
        ivp_event_listener_slot_t *slot = ivp_event_system_get_at(es, i);
        if (!slot) continue;
        ivp_event_listener_handle_t h;
        h.slot = slot->handle.slot;
        h.generation = slot->handle.generation;
        handles[handle_count++] = h;
    }

    for (int i = 0; i < handle_count; i++) {
        ivp_event_listener_slot_t *slot =
            ivp_environment_get_event_listener_by_handle(es, handles[i]);
        if (!slot || !slot->enabled || !slot->fn) continue;
        ivp_event_listener_fn_t fn = slot->fn;
        void *ctx = slot->ctx;
        fn(ctx, env, &evt);
    }

    if (allocated) {
        ivp_free(env->allocator, handles,
                 (size_t)n * sizeof(ivp_event_listener_handle_t));
    }
}

static void ivp_environment_commit_async_pushes_for_active_cores(ivp_environment_t *env)
{
    ivp_object_system_t *os = ivp_environment_get_object_system(env);
    const int n = ivp_object_system_count(os);
    for (int i = 0; i < n; i++) {
        ivp_core_t *core = &ivp_object_system_get_at(os, i)->core;
        if (!ivp_environment_core_is_active(core)) continue;
        ivp_core_commit_async_pushes(core);
    }
}

static void ivp_environment_revive_cores_psi(ivp_environment_t *env, ivp_real_t dt)
{
    ivp_object_system_t *os = ivp_environment_get_object_system(env);
    const int n = ivp_object_system_count(os);
    for (int i = 0; i < n; i++) {
        ivp_object_t *obj = ivp_object_system_get_at(os, i);
        ivp_core_t *core = &obj->core;
        if (core->physical_unmoveable || core->movement_state != IVP_MT_NOT_SIM) {
            continue;
        }

        if (ivp_environment_core_has_pending_push(core)) {
            core->movement_state = IVP_MT_MOVING;
            core->movement_check_counter = 0;
            ivp_core_commit_async_pushes(core);
            ivp_environment_emit_event_internal(env, IVP_EVENT_CORE_REVIVED, dt, obj->dense_index, 0);
            ivp_environment_emit_event_internal(env, IVP_EVENT_OBJECT_UNFROZEN, dt, obj->dense_index, 0);
        }
    }
}

static void ivp_environment_init_active_cores_for_psi(ivp_environment_t *env, ivp_real_t dt)
{
    ivp_object_system_t *os = ivp_environment_get_object_system(env);
    const int n = ivp_object_system_count(os);
    for (int i = 0; i < n; i++) {
        ivp_core_t *core = &ivp_object_system_get_at(os, i)->core;
        if (!ivp_environment_core_is_active(core)) continue;
        ivp_core_init_for_psi_ex(core, dt, env->use_forward_projection);

        /* Legacy: init_moving_core_for_psi calls commit_all_async_pushes
         * only for temporarily_unmovable cores.  Normal cores get their
         * async pushes committed inside the gravity controller at
         * priority 1000.  Since C17 doesn't yet have the temporarily_
         * unmovable flag, commit for any core that already has pending
         * pushes at this point (e.g., from external API calls between
         * PSIs). */
        if (ivp_environment_core_has_pending_push(core)) {
            ivp_core_commit_async_pushes(core);
        }
    }
}

static void ivp_environment_sort_controllers(ivp_environment_t *env)
{
    ivp_controller_system_t *cs = ivp_environment_get_controller_system(env);
    ivp_controller_system_sort(cs);
}

/* Forward declarations of sub-phase runners (defined below). */
static void ivp_environment_run_friction_subphase_static(ivp_environment_t *env,
                                                          ivp_real_t dt);
static void ivp_environment_run_friction_subphase_dynamic(ivp_environment_t *env,
                                                          ivp_real_t dt);
static void ivp_environment_run_friction_subphase_energy(ivp_environment_t *env,
                                                         ivp_real_t dt);

/* ── Friction sub-phase controller callbacks ────────────────────────────
 * In the reference IVP implementation, the friction solver runs at three
 * priority points within the controller pipeline (static, dynamic, energy).
 * Each callback delegates to the corresponding monolithic sub-phase. */

static void ivp_friction_controller_static(const ivp_event_sim_t *event_sim,
                                           ivp_object_t *obj, void *ctx)
{
    (void)obj; (void)ctx;
    ivp_environment_run_friction_subphase_static(event_sim->environment,
                                                 event_sim->delta_time);
}

static void ivp_friction_controller_dynamic(const ivp_event_sim_t *event_sim,
                                            ivp_object_t *obj, void *ctx)
{
    (void)obj; (void)ctx;
    ivp_environment_run_friction_subphase_dynamic(event_sim->environment,
                                                  event_sim->delta_time);
}

static void ivp_friction_controller_energy(const ivp_event_sim_t *event_sim,
                                           ivp_object_t *obj, void *ctx)
{
    (void)obj; (void)ctx;
    ivp_environment_run_friction_subphase_energy(event_sim->environment,
                                                 event_sim->delta_time);
}

static void ivp_environment_run_friction_subphase_static(ivp_environment_t *env,
                                                          ivp_real_t dt)
{
    ivp_friction_manager_t *fm = &env->friction_manager;

    if (fm->system_count == 0) return;

    ivp_collision_settings_t coll_settings;
    ivp_collision_settings_init_default(&coll_settings);
    coll_settings.gravity_magnitude = ivp_vec3r_length(&env->gravity);

    for (int s = 0; s < fm->system_count; s++) {
        ivp_friction_system_t *fs = fm->systems[s];
        if (fs->core_count >= 2) {
            ivp_friction_system_phase_static_normal(fs, &coll_settings, dt);
        }
    }
}

static void ivp_environment_run_friction_subphase_dynamic(ivp_environment_t *env,
                                                          ivp_real_t dt)
{
    ivp_friction_manager_t *fm = &env->friction_manager;
    if (fm->system_count == 0) return;

    ivp_collision_settings_t coll_settings;
    ivp_collision_settings_init_default(&coll_settings);
    coll_settings.gravity_magnitude = ivp_vec3r_length(&env->gravity);

    for (int s = 0; s < fm->system_count; s++) {
        ivp_friction_system_t *fs = fm->systems[s];
        if (fs->core_count >= 2) {
            ivp_friction_system_phase_dynamic_tangential(fs, &coll_settings, dt);
        }
    }
}

static void ivp_environment_run_friction_subphase_energy(ivp_environment_t *env,
                                                         ivp_real_t dt)
{
    ivp_friction_manager_t *fm = &env->friction_manager;
    if (fm->system_count == 0) return;

    ivp_collision_settings_t coll_settings;
    ivp_collision_settings_init_default(&coll_settings);
    coll_settings.gravity_magnitude = ivp_vec3r_length(&env->gravity);

    for (int s = 0; s < fm->system_count; s++) {
        ivp_friction_system_phase_energy(fm->systems[s], &coll_settings, dt);
    }
}

static int ivp_environment_island_identity_from_object(const ivp_environment_t *env,
                                                       const ivp_object_t *obj)
{
    if (!env || !obj) return -1;
    if (obj->dense_index < 0) return -1;
    return ivp_island_manager_find_island(&env->island_manager, obj->dense_index);
}

static void ivp_environment_invoke_controller_slot(ivp_environment_t *env,
                                                   ivp_controller_slot_t *slot,
                                                   ivp_object_t *obj,
                                                   ivp_real_t dt,
                                                   const ivp_object_handle_t *associated_handles,
                                                   int associated_count)
{
    if (!env || !slot || !slot->event_fn) return;
    ivp_event_sim_t event_sim;
    event_sim.delta_time = dt;
    event_sim.inverse_delta_time = (dt > IVP_REAL_EPS)
        ? (IVP_REAL(1.0) / dt) : IVP_REAL(0.0);
    event_sim.environment = env;
    event_sim.sim_unit_index = env->current_sim_unit_index;
    event_sim.island_identity = ivp_environment_island_identity_from_object(env, obj);
    event_sim.associated_core_handles = associated_handles;
    event_sim.associated_core_count = associated_count;
    slot->event_fn(&event_sim, obj, slot->ctx);
}

static void ivp_environment_run_controllers(ivp_environment_t *env, ivp_real_t dt)
{
    ivp_controller_system_t *cs = ivp_environment_get_controller_system(env);
    ivp_object_system_t *os = ivp_environment_get_object_system(env);

    const int cn = ivp_controller_system_count(cs);
    if (cn <= 0) {
        ivp_environment_commit_async_pushes_for_active_cores(env);
        return;
    }

    ivp_controller_handle_t local_controller_handles[32];
    ivp_controller_handle_t *controller_handles = local_controller_handles;
    bool controller_handles_allocated = false;
    if (cn > (int)(sizeof(local_controller_handles) / sizeof(local_controller_handles[0]))) {
        controller_handles = (ivp_controller_handle_t *)ivp_alloc(
            env->allocator, (size_t)cn * sizeof(ivp_controller_handle_t));
        if (!controller_handles) {
            return;
        }
        controller_handles_allocated = true;
    }

    int controller_handle_count = 0;
    for (int i = 0; i < cn; i++) {
        ivp_controller_slot_t *slot = ivp_controller_system_get_at(cs, i);
        if (!slot) continue;
        ivp_controller_handle_t h;
        h.slot = slot->handle.slot;
        h.generation = slot->handle.generation;
        controller_handles[controller_handle_count++] = h;
    }

    /* Run controllers in descending priority order (reference parity).
     *
     * Reference IVP semantics:
     *   - priority > IVP_CP_GRAVITY: async pushes
     *   - priority < IVP_CP_GRAVITY: real/sync pushes
     *
     * C17 controllers commonly write to speed_change for both stages.
     * To preserve reference-visible behavior, the dispatcher enforces an
     * explicit async→sync divider at gravity/boundary and commits after each
     * low-priority controller so low-stage pushes are immediately visible. */

    bool saw_above_gravity_controller = false;
    bool committed_at_gravity_boundary = false;

    for (int ci = 0; ci < controller_handle_count; ci++) {
        ivp_controller_slot_t *slot =
            ivp_controller_system_get_by_handle(cs, controller_handles[ci]);
        if (!slot || !slot->enabled) continue;
        if (!slot->event_fn) continue;

        if (!committed_at_gravity_boundary &&
            saw_above_gravity_controller &&
            slot->priority < IVP_CP_GRAVITY) {
            ivp_environment_commit_async_pushes_for_active_cores(env);
            committed_at_gravity_boundary = true;
        }

        if (slot->mode == IVP_CONTROLLER_DEPENDENT) {
            ivp_object_handle_t associated_handles[IVP_CONTROLLER_DISCOVERY_STACK_MAX];
            int associated_count = ivp_controller_slot_collect_associated_handles(
                slot, env, associated_handles, IVP_CONTROLLER_DISCOVERY_STACK_MAX);
            if (associated_count <= 0) continue;

            ivp_object_t *dispatch_obj = NULL;
            for (int ai = 0; ai < associated_count; ai++) {
                ivp_object_t *obj = ivp_object_system_get_by_handle(os, associated_handles[ai]);
                if (!obj) continue;
                if (!ivp_environment_core_is_active(&obj->core)) continue;
                dispatch_obj = obj;
                break;
            }
            if (!dispatch_obj) continue;

            /* Dependent parity: dispatch once with discovered associated set. */
            ivp_environment_invoke_controller_slot(env,
                                                   slot,
                                                   dispatch_obj,
                                                   dt,
                                                   associated_handles,
                                                   associated_count);
        } else {
            if (!slot->run_for_all_cores && slot->associated_count > 0) {
                for (int ai = 0; ai < slot->associated_count; ai++) {
                    ivp_object_handle_t h = slot->associated_handles[ai];
                    if (!ivp_object_handle_is_valid(h)) continue;
                    ivp_object_t *obj = ivp_object_system_get_by_handle(os, h);
                    if (!obj) continue;
                    if (!ivp_environment_core_is_active(&obj->core)) continue;
                    ivp_environment_invoke_controller_slot(env, slot, obj, dt, &h, 1);
                }
            } else if (slot->run_for_all_cores) {
                const int on = ivp_object_system_count(os);
                ivp_object_handle_t local_object_handles[64];
                ivp_object_handle_t *object_handles = local_object_handles;
                bool object_handles_allocated = false;
                if (on > (int)(sizeof(local_object_handles) / sizeof(local_object_handles[0]))) {
                    object_handles = (ivp_object_handle_t *)ivp_alloc(
                        env->allocator, (size_t)on * sizeof(ivp_object_handle_t));
                    if (!object_handles) {
                        continue;
                    }
                    object_handles_allocated = true;
                }

                int object_handle_count = 0;
                for (int oi = 0; oi < on; oi++) {
                    ivp_object_t *obj = ivp_object_system_get_at(os, oi);
                    if (!obj) continue;
                    ivp_object_handle_t h;
                    h.slot = obj->handle.slot;
                    h.generation = obj->handle.generation;
                    object_handles[object_handle_count++] = h;
                }

                for (int oi = 0; oi < object_handle_count; oi++) {
                    ivp_object_t *obj = ivp_object_system_get_by_handle(os, object_handles[oi]);
                    if (!obj) continue;
                    if (!ivp_environment_core_is_active(&obj->core)) continue;
                    ivp_environment_invoke_controller_slot(env,
                                                           slot,
                                                           obj,
                                                           dt,
                                                           &object_handles[oi],
                                                           1);
                }

                if (object_handles_allocated) {
                    ivp_free(env->allocator, object_handles,
                             (size_t)on * sizeof(ivp_object_handle_t));
                }
            } else if (slot->is_global) {
                /* Global controller: independent, not per-core, no
                 * associated handles.  Dispatch once with NULL object.
                 * Used by friction sub-phase controllers. */
                ivp_environment_invoke_controller_slot(env, slot, NULL, dt, NULL, 0);
            }
        }

        if (slot->priority > IVP_CP_GRAVITY) {
            saw_above_gravity_controller = true;
        } else if (slot->priority == IVP_CP_GRAVITY) {
            ivp_environment_commit_async_pushes_for_active_cores(env);
            committed_at_gravity_boundary = true;
        } else {
            ivp_environment_commit_async_pushes_for_active_cores(env);
        }
    }

    ivp_environment_commit_async_pushes_for_active_cores(env);

    if (controller_handles_allocated) {
        ivp_free(env->allocator, controller_handles,
                 (size_t)cn * sizeof(ivp_controller_handle_t));
    }
}

static void ivp_environment_integrate_active_cores(ivp_environment_t *env,
                                                   ivp_real_t dt,
                                                   ivp_time_t step_end_time)
{
    ivp_object_system_t *os = ivp_environment_get_object_system(env);
    const int n = ivp_object_system_count(os);

    for (int i = 0; i < n; i++) {
        ivp_core_t *core = &ivp_object_system_get_at(os, i)->core;
        if (!ivp_environment_core_is_active(core)) continue;
        ivp_core_calc_next_psi_matrix(core, dt);
        core->time_of_last_psi = step_end_time;
    }
}

/* Friction sub-phases are now interleaved with controllers at the correct
 * priority boundaries in ivp_environment_run_controllers().
 * The old monolithic ivp_environment_run_friction_phase() has been removed.
 *
 * Legacy ordering:
 *   static friction  (prio 0)   — begin_psi + distance_keepers + coupled_solve
 *   dynamic friction (prio 600) — easing
 *   gravity+damping  (prio 1000) — gravity controller
 *   energy friction  (prio 2000) — energy removal
 */

static void ivp_environment_update_movement_state(ivp_environment_t *env, ivp_real_t dt)
{
    ivp_object_system_t *os = ivp_environment_get_object_system(env);
    const int n = ivp_object_system_count(os);

    for (int i = 0; i < n; i++) {
        ivp_object_t *obj = ivp_object_system_get_at(os, i);
        ivp_core_t *core = &obj->core;
        if (!ivp_environment_core_is_active(core)) continue;

        core->movement_check_counter++;
        if (core->movement_check_counter >= IVP_MOVEMENT_CHECK_COUNT) {
            core->movement_check_counter = 0;
            ivp_movement_type_t old_state = core->movement_state;
            ivp_core_calc_movement_state(core, &env->freeze_params);
            if (old_state != IVP_MT_NOT_SIM && core->movement_state == IVP_MT_NOT_SIM) {
                ivp_environment_emit_event_internal(env, IVP_EVENT_OBJECT_FROZEN, dt,
                                                    obj->dense_index, 0);
            }
        }
    }
}

static void ivp_environment_default_gravity_controller(const ivp_event_sim_t *event_sim,
                                               ivp_object_t *obj,
                                               void *ctx)
{
    (void)ctx;
    ivp_environment_t *env = event_sim->environment;
    const ivp_real_t dt = event_sim->delta_time;
    ivp_core_t *core = &obj->core;
    if (core->pinned) return;

    /* Legacy IVP_Standard_Gravity_Controller::do_simulation_controller():
     *   1. global_damp_core(core)
     *   2. commit_all_async_pushes(core)
     *   3. speed += gravity * dt
     *
     * Bundling damping + commit + gravity in a single controller at
     * priority 1000 matches the legacy ordering where these three
     * operations happen atomically before lower-priority controllers
     * (springs, motors, etc.) see the results. */
    ivp_core_damp(core, dt);
    ivp_core_commit_async_pushes(core);
    ivp_vec3r_add_scaled(&core->speed, &env->gravity, dt);
}

/* Damping is now handled inside the gravity controller (priority 1000),
 * matching legacy IVP_Standard_Gravity_Controller::do_simulation_controller().
 * The old ivp_environment_damp_active_cores() has been removed. */

void ivp_environment_config_init(ivp_environment_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    ivp_vec3r_set(&cfg->gravity, IVP_REAL(0.0), IVP_REAL(9.81), IVP_REAL(0.0));
    cfg->delta_psi_time = (ivp_real_t)IVP_DEFAULT_DELTA_TIME;
    cfg->freeze_params = ivp_freeze_params_default();
    cfg->allocator = NULL;
    cfg->initial_object_capacity = IVP_OBJECTS_INIT_CAPACITY;
    cfg->initial_controller_capacity = IVP_CONTROLLERS_INIT_CAPACITY;
    cfg->initial_event_listener_capacity = IVP_EVENT_LISTENERS_INIT_CAPACITY;
    cfg->sim_unit_count = IVP_SIM_UNIT_COUNT_DEFAULT;
    ivp_anomaly_config_init(&cfg->anomaly_config);
    cfg->collision_filter = ivp_collision_filter_allow_all();
    cfg->ccd_enabled = false;
    ivp_solver_config_init_default(&cfg->solver_config);
}

/* Forward declarations for controller registration within create(). */
static ivp_controller_handle_t ivp_environment_add_controller_internal(
    ivp_environment_t *env, int priority,
    ivp_controller_event_fn_t event_fn, void *ctx,
    ivp_controller_mode_t mode, ivp_controller_discovery_fn_t discovery_fn);
static ivp_controller_slot_t *ivp_environment_get_controller_slot(
    ivp_environment_t *env, ivp_controller_handle_t controller_handle);

/* Add a "global" controller: dispatched once per PSI with NULL object,
 * rather than per active core.  Used for friction sub-phase controllers. */
static ivp_controller_handle_t ivp_environment_add_global_controller(
    ivp_environment_t *env, int priority,
    ivp_controller_event_fn_t event_fn, void *ctx)
{
    ivp_controller_handle_t h = ivp_environment_add_controller_internal(
        env, priority, event_fn, ctx, IVP_CONTROLLER_INDEPENDENT, NULL);
    if (ivp_controller_handle_is_valid(h)) {
        ivp_controller_slot_t *slot = ivp_environment_get_controller_slot(env, h);
        if (slot) {
            slot->run_for_all_cores = false;
            slot->is_global = true;
        }
    }
    return h;
}

ivp_environment_t *ivp_environment_create(const ivp_environment_config_t *config)
{
    ivp_environment_config_t cfg;
    if (config) cfg = *config;
    else ivp_environment_config_init(&cfg);

    if (!ivp_environment_delta_time_is_valid(cfg.delta_psi_time)) {
        cfg.delta_psi_time = (ivp_real_t)IVP_DEFAULT_DELTA_TIME;
    }

    const ivp_allocator_t *alloc = cfg.allocator ? cfg.allocator : ivp_default_allocator();

    ivp_environment_t *env = (ivp_environment_t *)ivp_alloc(alloc, sizeof(ivp_environment_t));
    if (!env) return NULL;
    memset(env, 0, sizeof(*env));

    env->gravity = cfg.gravity;
    env->delta_psi_time = cfg.delta_psi_time;
    env->freeze_params = cfg.freeze_params;
    env->current_time = 0.0;
    env->residual_time = 0.0;
    env->sim_unit_count = cfg.sim_unit_count > 0 ? cfg.sim_unit_count : 1;
    env->current_sim_unit_index = -1;

    if (cfg.allocator) {
        env->allocator = cfg.allocator;
        env->owns_allocator = false;
    } else {
        ivp_allocator_t *a = (ivp_allocator_t *)ivp_alloc(alloc, sizeof(ivp_allocator_t));
        if (!a) {
            ivp_free(alloc, env, sizeof(ivp_environment_t));
            return NULL;
        }
        *a = *alloc;
        env->allocator = a;
        env->owns_allocator = true;
    }

    env->object_system = (struct ivp_object_system *)ivp_alloc(env->allocator, sizeof(ivp_object_system_t));
    env->controller_system = (struct ivp_controller_system *)ivp_alloc(env->allocator, sizeof(ivp_controller_system_t));
    env->event_system = (struct ivp_event_system *)ivp_alloc(env->allocator, sizeof(ivp_event_system_t));
    if (!env->object_system || !env->controller_system || !env->event_system) {
        ivp_environment_destroy(env);
        return NULL;
    }

    if (ivp_object_system_init((ivp_object_system_t *)env->object_system,
                               env->allocator,
                               cfg.initial_object_capacity) != IVP_OK ||
        ivp_controller_system_init((ivp_controller_system_t *)env->controller_system,
                                   env->allocator,
                                   cfg.initial_controller_capacity) != IVP_OK ||
        ivp_event_system_init((ivp_event_system_t *)env->event_system,
                              env->allocator,
                              cfg.initial_event_listener_capacity) != IVP_OK) {
        ivp_environment_destroy(env);
        return NULL;
    }

    /* Initialize anomaly manager */
    ivp_anomaly_manager_init(&env->anomaly_manager, &cfg.anomaly_config);

    /* Collision filter */
    env->collision_filter = cfg.collision_filter;

    /* Friction persistence */
    if (ivp_friction_manager_init(&env->friction_manager, env->allocator) != IVP_OK) {
        ivp_environment_destroy(env);
        return NULL;
    }
    env->friction_manager.owner = env;

    /* Register friction sub-phases as global controllers at their priority
     * levels.  This replaces the old inline threshold-based dispatch and
     * aligns with the reference IVP controller pipeline. */
    env->friction_manager.static_fs_handle = ivp_environment_add_global_controller(
        env, IVP_CP_STATIC_FRICTION, ivp_friction_controller_static, &env->friction_manager);
    env->friction_manager.dynamic_fs_handle = ivp_environment_add_global_controller(
        env, IVP_CP_DYNAMIC_FRICTION, ivp_friction_controller_dynamic, &env->friction_manager);
    env->friction_manager.energy_fs_handle = ivp_environment_add_global_controller(
        env, IVP_CP_ENERGY_FRICTION, ivp_friction_controller_energy, &env->friction_manager);

    ivp_mindist_manager_init(&env->mindist_manager, env->allocator);
    ivp_hull_manager_init(&env->hull_manager, env->allocator);
    ivp_time_event_queue_init(&env->time_event_queue, env->allocator);

    /* CCD */
    env->ccd_enabled = cfg.ccd_enabled;

    /* Forward projection (legacy default) */
    env->use_forward_projection = true;

    /* Solver configuration */
    env->solver_config = cfg.solver_config;

    /* Sweep-and-prune broadphase */
    ivp_broadphase_sap_init(&env->broadphase_sap, env->allocator);

    /* Island manager */
    ivp_island_manager_init(&env->island_manager, env->allocator);

    ivp_controller_handle_t gravity_h = ivp_environment_add_controller(
        env, IVP_CP_GRAVITY, ivp_environment_default_gravity_controller, NULL);
    if (!ivp_controller_handle_is_valid(gravity_h)) {
        ivp_environment_destroy(env);
        return NULL;
    }

    return env;
}

void ivp_environment_destroy(ivp_environment_t *env)
{
    if (!env) return;

    ivp_allocator_t *alloc = env->allocator;
    const bool owns = env->owns_allocator;

    /* If we own the allocator, snapshot it to a stack copy so we can use it
     * to free itself after freeing everything else. This avoids the subtle
     * self-referential free where alloc->free is called with alloc as the
     * pointer being freed while still reading from alloc's vtable. */
    ivp_allocator_t alloc_copy;
    if (owns) {
        alloc_copy = *alloc;
    }

    if (env->universe_manager) {
        ivp_free(alloc, env->universe_manager, sizeof(*env->universe_manager));
        env->universe_manager = NULL;
    }

    ivp_time_event_queue_destroy(&env->time_event_queue);
    ivp_hull_manager_destroy(&env->hull_manager);
    ivp_mindist_manager_destroy(&env->mindist_manager);
    ivp_friction_manager_destroy(&env->friction_manager);
    ivp_broadphase_sap_destroy(&env->broadphase_sap);
    ivp_island_manager_destroy(&env->island_manager);

    if (env->event_system) {
        ivp_event_system_destroy((ivp_event_system_t *)env->event_system, alloc);
        ivp_free(alloc, env->event_system, sizeof(ivp_event_system_t));
    }

    if (env->controller_system) {
        ivp_controller_system_destroy((ivp_controller_system_t *)env->controller_system, alloc);
        ivp_free(alloc, env->controller_system, sizeof(ivp_controller_system_t));
    }

    if (env->object_system) {
        ivp_object_system_destroy((ivp_object_system_t *)env->object_system, alloc);
        ivp_free(alloc, env->object_system, sizeof(ivp_object_system_t));
    }

    ivp_free(alloc, env, sizeof(ivp_environment_t));
    if (owns) {
        ivp_free(&alloc_copy, alloc, sizeof(ivp_allocator_t));
    }
}

static ivp_controller_handle_t ivp_environment_add_controller_internal(
    ivp_environment_t *env,
    int priority,
    ivp_controller_event_fn_t event_fn,
    void *ctx,
    ivp_controller_mode_t mode,
    ivp_controller_discovery_fn_t discovery_fn);

ivp_controller_handle_t ivp_environment_add_controller(ivp_environment_t *env,
                                               int priority,
                                               ivp_controller_fn_t fn,
                                               void *ctx)
{
    return ivp_environment_add_controller_internal(env, priority, fn, ctx,
                                                   IVP_CONTROLLER_INDEPENDENT, NULL);
}

static ivp_controller_slot_t *ivp_environment_get_controller_slot(
    ivp_environment_t *env, ivp_controller_handle_t controller_handle)
{
    if (!env || !ivp_controller_handle_is_valid(controller_handle)) return NULL;
    return ivp_controller_system_get_by_handle(
        (const ivp_controller_system_t *)env->controller_system,
        controller_handle);
}

static int ivp_environment_normalize_priority(int priority)
{
    if (priority < IVP_CP_STATIC_FRICTION) return IVP_CP_STATIC_FRICTION;
    if (priority > IVP_CP_ENERGY_FRICTION) return IVP_CP_ENERGY_FRICTION;
    return priority;
}

static ivp_controller_handle_t ivp_environment_add_controller_internal(
    ivp_environment_t *env,
    int priority,
    ivp_controller_event_fn_t event_fn,
    void *ctx,
    ivp_controller_mode_t mode,
    ivp_controller_discovery_fn_t discovery_fn)
{
    ivp_controller_handle_t invalid = ivp_controller_handle_invalid();
    if (!env) return invalid;
    if (!event_fn) return invalid;

    const int normalized_priority = ivp_environment_normalize_priority(priority);

    ivp_controller_handle_t out =
        ivp_controller_system_add((ivp_controller_system_t *)env->controller_system,
                                  env->allocator,
                                  normalized_priority,
                                  event_fn,
                                  ctx);
    if (!ivp_controller_handle_is_valid(out)) return invalid;

    ivp_controller_slot_t *slot = ivp_environment_get_controller_slot(env, out);
    if (!slot) {
        (void)ivp_controller_system_remove((ivp_controller_system_t *)env->controller_system,
                                           out,
                                           env->allocator);
        return invalid;
    }
    slot->mode = mode;
    slot->run_for_all_cores = (mode == IVP_CONTROLLER_INDEPENDENT);
    slot->discovery_fn = discovery_fn;

    ivp_environment_sort_controllers(env);
    ivp_environment_emit_event_internal(env, IVP_EVENT_CONTROLLER_ADDED, IVP_REAL(0.0), -1, (int)out.slot);
    return out;
}

ivp_controller_handle_t ivp_environment_announce_dependent_controller(
    ivp_environment_t *env,
    int priority,
    ivp_controller_fn_t fn,
    void *ctx,
    ivp_controller_discovery_fn_t discovery_fn)
{
    return ivp_environment_add_controller_internal(env, priority, fn, ctx,
                                                   IVP_CONTROLLER_DEPENDENT, discovery_fn);
}

bool ivp_environment_remove_dependent_controller(ivp_environment_t *env,
                                                 ivp_controller_handle_t controller_handle,
                                                 bool silently)
{
    if (!env || !ivp_controller_handle_is_valid(controller_handle)) return false;

    if (!ivp_controller_system_remove((ivp_controller_system_t *)env->controller_system,
                                      controller_handle,
                                      env->allocator)) {
        return false;
    }

    if (!silently) {
        ivp_environment_emit_event_internal(env, IVP_EVENT_CONTROLLER_REMOVED, IVP_REAL(0.0), -1, (int)controller_handle.slot);
    }
    return true;
}

bool ivp_environment_remove_controller(ivp_environment_t *env, ivp_controller_handle_t controller_handle)
{
    return ivp_environment_remove_dependent_controller(env, controller_handle, false);
}

bool ivp_environment_set_controller_discovery(ivp_environment_t *env,
                                              ivp_controller_handle_t controller_handle,
                                              ivp_controller_discovery_fn_t discovery_fn)
{
    if (!env || !ivp_controller_handle_is_valid(controller_handle) || !discovery_fn) return false;
    ivp_controller_slot_t *slot = ivp_environment_get_controller_slot(env, controller_handle);
    if (!slot) return false;
    slot->mode = IVP_CONTROLLER_DEPENDENT;
    slot->run_for_all_cores = false;
    slot->discovery_fn = discovery_fn;
    return true;
}

bool ivp_environment_add_controller_to_core(ivp_environment_t *env,
                                            ivp_controller_handle_t controller_handle,
                                            ivp_object_t *object)
{
    if (!env || !object || object->owner != env) return false;
    ivp_controller_slot_t *slot = ivp_environment_get_controller_slot(env, controller_handle);
    if (!slot) return false;
    slot->mode = IVP_CONTROLLER_INDEPENDENT;
    slot->run_for_all_cores = false;
    slot->discovery_fn = NULL;
    return ivp_controller_slot_add_associated_handle(
        slot, ivp_environment_get_object_handle(object), env->allocator);
}

bool ivp_environment_remove_controller_from_core(ivp_environment_t *env,
                                                 ivp_controller_handle_t controller_handle,
                                                 ivp_object_t *object)
{
    if (!env || !object || object->owner != env) return false;
    ivp_controller_slot_t *slot = ivp_environment_get_controller_slot(env, controller_handle);
    if (!slot) return false;
    return ivp_controller_slot_remove_associated_handle(
        slot, ivp_environment_get_object_handle(object));
}

bool ivp_environment_ensure_core_in_simulation(ivp_environment_t *env,
                                               ivp_object_t *object)
{
    if (!env || !object || object->owner != env) return false;
    ivp_core_t *core = &object->core;
    if (core->physical_unmoveable) return false;
    if (core->movement_state == IVP_MT_NOT_SIM) {
        core->movement_state = IVP_MT_MOVING;
        core->movement_check_counter = 0;
        ivp_environment_emit_event_internal(env, IVP_EVENT_CORE_REVIVED, IVP_REAL(0.0),
                                            object->dense_index, 0);
        ivp_environment_emit_event_internal(env, IVP_EVENT_OBJECT_UNFROZEN, IVP_REAL(0.0),
                                            object->dense_index, 0);
    }
    return true;
}

bool ivp_environment_ensure_controller_in_simulation(ivp_environment_t *env,
                                                     ivp_controller_handle_t controller_handle)
{
    if (!env || !ivp_controller_handle_is_valid(controller_handle)) return false;
    ivp_controller_slot_t *slot = ivp_environment_get_controller_slot(env, controller_handle);
    if (!slot) return false;

    ivp_object_system_t *os = ivp_environment_get_object_system(env);
    bool woke_any = false;

    if (slot->mode == IVP_CONTROLLER_DEPENDENT) {
        ivp_object_handle_t handles[IVP_CONTROLLER_DISCOVERY_STACK_MAX];
        int count = ivp_controller_slot_collect_associated_handles(
            slot, env, handles, IVP_CONTROLLER_DISCOVERY_STACK_MAX);
        for (int i = 0; i < count; i++) {
            ivp_object_t *obj = ivp_object_system_get_by_handle(os, handles[i]);
            if (!obj) continue;
            if (ivp_environment_ensure_core_in_simulation(env, obj)) woke_any = true;
        }
    } else if (slot->run_for_all_cores) {
        const int n = ivp_object_system_count(os);
        for (int i = 0; i < n; i++) {
            ivp_object_t *obj = ivp_object_system_get_at(os, i);
            if (!obj) continue;
            if (ivp_environment_ensure_core_in_simulation(env, obj)) woke_any = true;
        }
    } else {
        for (int i = 0; i < slot->associated_count; i++) {
            ivp_object_t *obj = ivp_object_system_get_by_handle(os, slot->associated_handles[i]);
            if (!obj) continue;
            if (ivp_environment_ensure_core_in_simulation(env, obj)) woke_any = true;
        }
    }
    return woke_any;
}

void ivp_environment_clear_controllers(ivp_environment_t *env)
{
    if (!env) return;

    ivp_controller_system_clear((ivp_controller_system_t *)env->controller_system, env->allocator);
}

ivp_event_listener_handle_t ivp_environment_add_event_listener(ivp_environment_t *env,
                                                       ivp_event_listener_fn_t fn,
                                                       void *ctx)
{
    ivp_event_listener_handle_t invalid = ivp_event_listener_handle_invalid();
    if (!env || !fn) return invalid;

    return ivp_event_system_add_listener((ivp_event_system_t *)env->event_system,
                                         env->allocator,
                                         fn,
                                         ctx);
}

bool ivp_environment_remove_event_listener(ivp_environment_t *env,
                                   ivp_event_listener_handle_t listener_handle)
{
    if (!env || !ivp_event_listener_handle_is_valid(listener_handle)) return false;

    if (!ivp_event_system_remove_listener((ivp_event_system_t *)env->event_system,
                                          listener_handle,
                                          env->allocator)) {
        return false;
    }
    return true;
}

void ivp_environment_clear_event_listeners(ivp_environment_t *env)
{
    if (!env) return;

    ivp_event_system_clear((ivp_event_system_t *)env->event_system, env->allocator);
}

void ivp_environment_set_sim_unit_count(ivp_environment_t *env, int sim_unit_count)
{
    if (!env) return;
    env->sim_unit_count = sim_unit_count > 0 ? sim_unit_count : 1;
}

int ivp_environment_get_sim_unit_count(const ivp_environment_t *env)
{
    if (!env) return 1;
    return env->sim_unit_count;
}

int ivp_environment_get_current_sim_unit_index(const ivp_environment_t *env)
{
    if (!env) return -1;
    return env->current_sim_unit_index;
}

ivp_object_t *ivp_environment_create_object_with_handle(ivp_environment_t *env,
                                                const ivp_object_template_t *tmpl,
                                                const ivp_surman_t *surman,
                                                ivp_object_handle_t *out_handle)
{
    if (!env || !tmpl) return NULL;
    ivp_object_t *obj = ivp_object_system_create((ivp_object_system_t *)env->object_system,
                                                 env,
                                                 tmpl,
                                                 surman,
                                                 out_handle);
    if (!obj) return NULL;
    ivp_hull_manager_add_object(&env->hull_manager, obj);
    return obj;
}

ivp_object_t *ivp_environment_create_object(ivp_environment_t *env,
                                    const ivp_object_template_t *tmpl,
                                    const ivp_surman_t *surman)
{
    return ivp_environment_create_object_with_handle(env, tmpl, surman, NULL);
}

ivp_object_t *ivp_environment_get_object_by_handle(const ivp_environment_t *env, ivp_object_handle_t handle)
{
    if (!env || !ivp_object_handle_is_valid(handle)) return NULL;
    return ivp_object_system_get_by_handle((const ivp_object_system_t *)env->object_system, handle);
}

ivp_object_handle_t ivp_environment_get_object_handle(const ivp_object_t *obj)
{
    if (!obj) return ivp_object_handle_invalid();
    ivp_object_handle_t h;
    h.slot = obj->handle.slot;
    h.generation = obj->handle.generation;
    return h;
}

bool ivp_environment_destroy_object_by_handle(ivp_environment_t *env, ivp_object_handle_t handle)
{
    if (!env || !ivp_object_handle_is_valid(handle)) return false;

    ivp_object_t *obj = ivp_object_system_get_by_handle(
        (const ivp_object_system_t *)env->object_system, handle);
    if (obj) {
        ivp_hull_manager_remove_object(&env->hull_manager, obj);
        ivp_mindist_manager_remove_object(&env->mindist_manager, obj);
        ivp_time_event_queue_remove_object(&env->time_event_queue, handle);
    }

    return ivp_object_system_destroy_by_handle((ivp_object_system_t *)env->object_system,
                                               handle,
                                               env->allocator);
}

void ivp_environment_destroy_object(ivp_environment_t *env, ivp_object_t *obj)
{
    if (!env || !obj) return;
    if (obj->owner != env) return;

    /* Fire object-deleted notification to collision listeners (reference parity) */
    ivp_collision_listeners_fire_object_deleted(obj);

    /* Notify collision delegator roots */
    for (int i = 0; i < env->collision_delegator_root_count; i++) {
        ivp_collision_delegator_root_t *root = env->collision_delegator_roots[i];
        if (root && root->object_removed) {
            root->object_removed(root, obj);
        }
    }

    /* Detach from parent cluster if any */
    if (obj->parent_cluster) {
        ivp_cluster_remove_child(obj->parent_cluster, obj);
    }

    /* Notify universe manager before destruction */
    if (env->universe_manager && env->universe_manager->deleted_fn) {
        env->universe_manager->deleted_fn(env->universe_manager->ctx, obj);
    }

    (void)ivp_environment_destroy_object_by_handle(env, ivp_environment_get_object_handle(obj));
}

/* ── Cluster API ───────────────────────────────────────────────────────── */

ivp_cluster_t *ivp_environment_create_cluster(ivp_environment_t *env,
                                              ivp_cluster_t *parent,
                                              const ivp_cluster_template_t *tmpl)
{
    if (!env) return NULL;
    return ivp_cluster_create(env, parent, tmpl);
}

void ivp_environment_destroy_cluster(ivp_environment_t *env, ivp_cluster_t *cluster)
{
    if (!env || !cluster) return;
    ivp_cluster_destroy(cluster);
}

int ivp_environment_get_object_count(const ivp_environment_t *env)
{
    if (!env) return 0;
    return ivp_object_system_count((const ivp_object_system_t *)env->object_system);
}

ivp_object_t *ivp_environment_get_object(const ivp_environment_t *env, int index)
{
    if (!env) return NULL;
    if (index < 0 || index >= ivp_environment_get_object_count(env)) return NULL;
    return ivp_object_system_get_at((const ivp_object_system_t *)env->object_system, index);
}

void ivp_environment_simulate_one_psi(ivp_environment_t *env)
{
    if (!env) return;
    if (!ivp_environment_delta_time_is_valid(env->delta_psi_time)) {
        env->delta_psi_time = (ivp_real_t)IVP_DEFAULT_DELTA_TIME;
    }

    const ivp_real_t dt = env->delta_psi_time;
    const int sim_units = env->sim_unit_count > 0 ? env->sim_unit_count : 1;
    const ivp_real_t unit_dt = dt / (ivp_real_t)sim_units;

    /* Set active solver config for this PSI so solver functions can read it.
     * Cache normalized gravity direction for shock propagation. */
    {
        ivp_real_t grav_len = ivp_vec3r_length(&env->gravity);
        if (grav_len > IVP_REAL_EPS) {
            ivp_real_t inv = IVP_REAL(1.0) / grav_len;
            env->solver_config.gravity_dir.k[0] = env->gravity.k[0] * inv;
            env->solver_config.gravity_dir.k[1] = env->gravity.k[1] * inv;
            env->solver_config.gravity_dir.k[2] = env->gravity.k[2] * inv;
        } else {
            env->solver_config.gravity_dir = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
        }
    }
    ivp_solver_config_set_active(&env->solver_config);

    ivp_environment_emit_phase(env, IVP_PSI_PHASE_PRE_PSI, dt);

    for (int unit = 0; unit < sim_units; unit++) {
        env->current_sim_unit_index = unit;

        ivp_environment_emit_phase(env, IVP_PSI_PHASE_SIM_UNIT_BEGIN, unit_dt);
        ivp_environment_emit_event_internal(env, IVP_EVENT_SIM_UNIT_BEGIN, unit_dt, -1, 0);

        /* Reset per-PSI anomaly counters */
        ivp_anomaly_manager_begin_psi(&env->anomaly_manager);

        ivp_environment_emit_phase(env, IVP_PSI_PHASE_REVIVE, unit_dt);
        ivp_environment_revive_cores_psi(env, unit_dt);

        ivp_environment_emit_phase(env, IVP_PSI_PHASE_MANAGER_ACTIVE_VALUES, unit_dt);
        ivp_environment_emit_phase(env, IVP_PSI_PHASE_MANAGER_CONSTRAINTS, unit_dt);

        ivp_environment_emit_phase(env, IVP_PSI_PHASE_PRE_CONTROLLERS, unit_dt);
        ivp_environment_init_active_cores_for_psi(env, unit_dt);
        ivp_island_manager_rebuild(&env->island_manager, env);
        ivp_friction_manager_begin_psi(&env->friction_manager);

        /* Legacy PSI order: friction sub-phases are interleaved with
         * controllers at the correct priority boundaries.  Damping and
         * commit_async are bundled inside the gravity controller at
         * priority 1000.  The controller runner handles all ordering. */
        ivp_environment_emit_phase(env, IVP_PSI_PHASE_CONTROLLERS, unit_dt);
        ivp_environment_run_controllers(env, unit_dt);

        ivp_environment_emit_phase(env, IVP_PSI_PHASE_INTEGRATE, unit_dt);
        const ivp_time_t unit_end_time =
            env->current_time + (ivp_time_t)((ivp_real_t)(unit + 1) * unit_dt);
        ivp_environment_integrate_active_cores(env, unit_dt, unit_end_time);

        ivp_environment_emit_phase(env, IVP_PSI_PHASE_COLLISION_BROADPHASE, unit_dt);
        ivp_environment_emit_phase(env, IVP_PSI_PHASE_COLLISION_NARROWPHASE, unit_dt);
        ivp_environment_emit_phase(env, IVP_PSI_PHASE_COLLISION_SOLVE, unit_dt);
        ivp_environment_resolve_collisions(env);

        ivp_environment_emit_phase(env, IVP_PSI_PHASE_MANAGER_EVENTS, unit_dt);

        ivp_environment_emit_phase(env, IVP_PSI_PHASE_MOVEMENT_STATE, unit_dt);
        ivp_environment_update_movement_state(env, unit_dt);

        ivp_environment_emit_phase(env, IVP_PSI_PHASE_SIM_UNIT_END, unit_dt);
        ivp_environment_emit_event_internal(env, IVP_EVENT_SIM_UNIT_END, unit_dt, -1, 0);
    }

    env->current_sim_unit_index = -1;

    ivp_environment_emit_phase(env, IVP_PSI_PHASE_POST_PSI, dt);
    env->current_time += (ivp_time_t)dt;

    /* Clear active solver config after PSI */
    ivp_solver_config_set_active(NULL);
}

void ivp_environment_simulate_dtime(ivp_environment_t *env, ivp_real_t elapsed_time)
{
    if (!env || elapsed_time <= IVP_REAL(0.0)) return;
    if (!ivp_environment_delta_time_is_valid(env->delta_psi_time)) {
        env->delta_psi_time = (ivp_real_t)IVP_DEFAULT_DELTA_TIME;
    }

    env->residual_time += (ivp_time_t)elapsed_time;

    /* Clamp accumulated residual per-step (not total elapsed),
     * matching original IVP. This allows catching up after frame spikes
     * rather than losing simulation time. Cap total steps to prevent
     * spiral-of-death when update rate can't keep up. */
    const int max_steps = (int)((ivp_real_t)IVP_MAX_DELTA_PSI_TIME / env->delta_psi_time) + 1;
    int steps = 0;

    while (env->residual_time >= (ivp_time_t)env->delta_psi_time && steps < max_steps) {
        ivp_environment_simulate_one_psi(env);
        env->residual_time -= (ivp_time_t)env->delta_psi_time;
        steps++;
    }
}

/* ── Collision Delegator API ───────────────────────────────────────────── */

bool ivp_environment_add_collision_delegator(ivp_environment_t *env,
                                             ivp_collision_delegator_root_t *root)
{
    if (!env || !root) return false;
    if (env->collision_delegator_root_count >= IVP_MAX_COLLISION_DELEGATOR_ROOTS) return false;
    env->collision_delegator_roots[env->collision_delegator_root_count++] = root;
    return true;
}

bool ivp_environment_remove_collision_delegator(ivp_environment_t *env,
                                                ivp_collision_delegator_root_t *root)
{
    if (!env || !root) return false;
    for (int i = 0; i < env->collision_delegator_root_count; i++) {
        if (env->collision_delegator_roots[i] == root) {
            for (int j = i; j < env->collision_delegator_root_count - 1; j++) {
                env->collision_delegator_roots[j] = env->collision_delegator_roots[j + 1];
            }
            env->collision_delegator_root_count--;
            return true;
        }
    }
    return false;
}

/* ── Global Collision Listener API ─────────────────────────────────────── */

int ivp_environment_add_collision_listener_global(ivp_environment_t *env,
                                                  ivp_collision_listener_t *listener)
{
    if (!env || !listener) return -1;
    if (env->global_collision_listener_count >= IVP_MAX_GLOBAL_COLLISION_LISTENERS) return -1;
    env->global_collision_listeners[env->global_collision_listener_count++] = listener;
    return 0;
}

int ivp_environment_remove_collision_listener_global(ivp_environment_t *env,
                                                     ivp_collision_listener_t *listener)
{
    if (!env || !listener) return -1;
    for (int i = 0; i < env->global_collision_listener_count; i++) {
        if (env->global_collision_listeners[i] == listener) {
            for (int j = i; j < env->global_collision_listener_count - 1; j++) {
                env->global_collision_listeners[j] = env->global_collision_listeners[j + 1];
            }
            env->global_collision_listener_count--;
            return 0;
        }
    }
    return -1;
}
