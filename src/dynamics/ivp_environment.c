/* ivp_environment.c — port of IVP_Environment (ivp_physics/ivp_environment.cxx,
 * ivp_physics/ivp_time.cxx PSI event, ivp_intern/ivp_physic_private.cxx revive list,
 * ivp_physics/ivp_range_manager.cxx, ivp_physics/ivp_anomaly_manager.cxx,
 * ivp_physics/ivp_collision_filter.cxx, ivp_physics/ivp_material.cxx)
 * plus the libivp public environment / controller / event APIs.
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_universe_manager.h>
#include <ivp/collision/ivp_range_manager.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/collision/ivp_hull_mgr.h>
#include <ivp/controllers/ivp_actuator_check_dist.h>
#include <ivp/controllers/ivp_actuator_extra.h>
#include <ivp/dynamics/ivp_anchor.h>
#include <ivp/controllers/ivp_actuator_force.h>
#include <ivp/controllers/ivp_actuator_rot_mot.h>
#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/controllers/ivp_actuator_stabilizer.h>
#include <ivp/controllers/ivp_actuator_suspension.h>
#include <ivp/controllers/ivp_actuator_torque.h>
#include <ivp/controllers/ivp_car_system.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/controllers/ivp_constraint_fixed_keyed.h>
#include <ivp/controllers/ivp_constraint_local.h>
#include <ivp/controllers/ivp_controller_airboat.h>
#include <ivp/controllers/ivp_controller_buoyancy.h>
#include <ivp/controllers/ivp_controller_fake_jetski.h>
#include <ivp/controllers/ivp_controller_floating.h>
#include <ivp/controllers/ivp_controller_golem.h>
#include <ivp/controllers/ivp_controller_motion.h>
#include <ivp/controllers/ivp_controller_phantom.h>
#include <ivp/events/ivp_listener_object.h>
#include <ivp/controllers/ivp_controller_raycast_car.h>
#include <ivp/controllers/ivp_controller_stiff_spring.h>
#include <ivp/controllers/ivp_controller_world_friction.h>
#include <ivp/controllers/ivp_forcefield.h>
#include <ivp/dynamics/ivp_object_attach.h>
#include <ivp/diagnostics/ivp_performance_counter.h>
#include <ivp/diagnostics/ivp_better_statistics_manager.h>
#include <ivp/diagnostics/ivp_debug_manager.h>
#include <ivp/diagnostics/ivp_statistic_manager.h>
#include <ivp/utility/ivp_string.h>

#include "collision/ivp_lvec.h"
#include "ivp_object_system.h"
#include "dynamics/ivp_environment_events.h"
#include "dynamics/ivp_env_pipeline.h"
#include "dynamics/ivp_environment_parallel.h"
#include "dynamics/ivp_env_parallel_tasks.h"
#include "base/ivp_task_pool.h"
#if IVP_ENABLE_CHECKPOINTS
#include <ivp/base/ivp_memory_region.h>
#endif
#include "controllers/ivp_controller_system.h"
#include "events/ivp_event_system.h"
#include "solver/ivp_friction_internal.h"


#define IVP_MOVEMENT_CHECK_COUNT 10
/* the mean of the reference interval 15 + (short)(ivp_rand() * 5), which is
 * 15 to 19 PSIs */
#define IVP_MOVEMENT_CHECK_PERIOD 17
#define IVP_REF_PI 3.14159265358979323846

/* ── small helpers ─────────────────────────────────────────────────────── */

static ivp_event_system_t *env_event_system(const ivp_environment_t *env) { return (ivp_event_system_t *)env->event_system; }
static ivp_controller_system_t *env_controller_system(const ivp_environment_t *env) { return (ivp_controller_system_t *)env->controller_system; }
static ivp_object_system_t *env_object_system(const ivp_environment_t *env) { return (ivp_object_system_t *)env->object_system; }

int ivp_env_parallel_worker_count(const ivp_environment_t *env)
{
    if (!env || !env->pl || !env->pl->task_pool) return 1;
    return ivp_task_pool_worker_count(env->pl->task_pool);
}

bool ivp_env_parallel_for(ivp_environment_t *env,
                          ivp_parallel_work_kind_t kind,
                          size_t task_count,
                          size_t min_tasks,
                          ivp_env_parallel_task_fn function,
                          void *context)
{
    if (!env || !env->pl || !env->pl->task_pool || !function ||
        task_count < min_tasks) {
        return false;
    }
    if (!ivp_task_pool_parallel_for(env->pl->task_pool, task_count, function,
                                    context)) {
        return false;
    }
    ivp_parallel_stats_t *stats = &env->pl->parallel_stats;
    if (kind == IVP_PARALLEL_WORK_ISLAND) {
        stats->island_dispatches++;
        stats->island_tasks += (uint64_t)task_count;
    } else if (kind == IVP_PARALLEL_WORK_BROADPHASE) {
        stats->broadphase_dispatches++;
        stats->broadphase_tasks += (uint64_t)task_count;
    } else {
        stats->solver_dispatches++;
        stats->solver_tasks += (uint64_t)task_count;
    }
    int workers_used = ivp_task_pool_worker_count(env->pl->task_pool);
    if ((size_t)workers_used > task_count) workers_used = (int)task_count;
    if (workers_used > stats->max_workers_used) {
        stats->max_workers_used = workers_used;
    }
    return true;
}

int ivp_environment_get_parallel_worker_count(const ivp_environment_t *env)
{
    return ivp_env_parallel_worker_count(env);
}

void ivp_environment_get_parallel_stats(const ivp_environment_t *env,
                                        ivp_parallel_stats_t *stats_out)
{
    if (!stats_out) return;
    memset(stats_out, 0, sizeof(*stats_out));
    if (env && env->pl) *stats_out = env->pl->parallel_stats;
}

static void env_emit_phase(ivp_environment_t *env, ivp_psi_phase_t phase, ivp_real_t dt)
{
    if (env->phase_hook) env->phase_hook(env->phase_hook_ctx, env, phase, env->current_time, dt);
}

static ivp_event_listener_slot_t *env_get_event_listener_by_handle(ivp_event_system_t *es, ivp_event_listener_handle_t handle)
{
    if (!es || !ivp_event_listener_handle_is_valid(handle)) return NULL;
    ivp_handle_t h;
    h.slot = handle.slot;
    h.generation = handle.generation;
    return (ivp_event_listener_slot_t *)ivp_registry_get_by_handle(&es->registry, h);
}

typedef struct env_event_listener_snapshot {
    ivp_event_listener_handle_t handle;
    uint64_t registration_sequence;
} env_event_listener_snapshot_t;

static void env_sort_event_listener_snapshot(
    env_event_listener_snapshot_t *listeners, int count)
{
    for (int i = 1; i < count; i++) {
        env_event_listener_snapshot_t listener = listeners[i];
        int position = i;
        while (position > 0 &&
               listeners[position - 1].registration_sequence >
                   listener.registration_sequence) {
            listeners[position] = listeners[position - 1];
            position--;
        }
        listeners[position] = listener;
    }
}

static void env_emit_event_by_registration(ivp_environment_t *env,
                                           ivp_event_system_t *es,
                                           const ivp_event_t *event,
                                           uint64_t last_registration)
{
    while (last_registration > 0u) {
        ivp_event_listener_handle_t handle =
            ivp_event_listener_handle_invalid();
        uint64_t selected_registration = 0u;
        const int count = ivp_event_system_count(es);
        for (int i = 0; i < count; i++) {
            ivp_event_listener_slot_t *slot =
                ivp_event_system_get_at(es, i);
            if (!slot || slot->registration_sequence > last_registration ||
                slot->registration_sequence <= selected_registration) {
                continue;
            }
            selected_registration = slot->registration_sequence;
            handle.slot = slot->handle.slot;
            handle.generation = slot->handle.generation;
        }
        if (selected_registration == 0u) return;
        last_registration = selected_registration - 1u;

        ivp_event_listener_slot_t *slot =
            env_get_event_listener_by_handle(es, handle);
        if (!slot || slot->registration_sequence != selected_registration ||
            !slot->enabled || !slot->fn) {
            continue;
        }
        ivp_event_listener_fn_t fn = slot->fn;
        void *ctx = slot->ctx;
        fn(ctx, env, event);
    }
}

void ivp_environment_emit_event_internal(ivp_environment_t *env, ivp_event_type_t type, ivp_real_t dt,
                                         int object_index, int controller_id)
{
    ivp_event_system_t *es = env_event_system(env);
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
    env_event_listener_snapshot_t local_listeners[32];
    env_event_listener_snapshot_t *listeners = local_listeners;
    bool allocated = false;
    if (n > (int)(sizeof(local_listeners) /
                  sizeof(local_listeners[0]))) {
        listeners = (env_event_listener_snapshot_t *)ivp_alloc(
            env->allocator, (size_t)n * sizeof(*listeners));
        if (!listeners) {
            const uint64_t last_registration =
                es->next_registration_sequence > 0u
                    ? es->next_registration_sequence - 1u
                    : 0u;
            env_emit_event_by_registration(
                env, es, &evt, last_registration);
            return;
        }
        allocated = true;
    }
    int listener_count = 0;
    for (int i = 0; i < n; i++) {
        ivp_event_listener_slot_t *slot = ivp_event_system_get_at(es, i);
        if (!slot) continue;
        env_event_listener_snapshot_t *snapshot =
            &listeners[listener_count++];
        snapshot->handle.slot = slot->handle.slot;
        snapshot->handle.generation = slot->handle.generation;
        snapshot->registration_sequence = slot->registration_sequence;
    }
    env_sort_event_listener_snapshot(listeners, listener_count);
    /* reference listener vectors fire from the last added to the first
     * (e.g. IVP_Environment::fire_event_PSI) */
    for (int i = listener_count - 1; i >= 0; i--) {
        ivp_event_listener_slot_t *slot =
            env_get_event_listener_by_handle(es, listeners[i].handle);
        if (!slot || !slot->enabled || !slot->fn) continue;
        slot->fn(slot->ctx, env, &evt);
    }
    if (allocated) {
        ivp_free(env->allocator, listeners,
                 (size_t)n * sizeof(*listeners));
    }
}

/* ── sim unit transaction arena ─────────────────────────────────────── */

/* Sim-unit scratch does not depend on the reference short-term arena's exact
 * 0x7fe0-byte layout.  Keep its resident block modest for small worlds;
 * overflow blocks still use IVP_ARENA_BLOCK_SIZE for large contact systems. */
#define IVP_SIM_MEM_FIRST_BLOCK_SIZE 0x1800u

struct ivp_arena *ivp_env_get_short_term_mem(struct ivp_environment *env)
{
    return &env->pl->short_term_mem;
}

static uint64_t env_take_sequence(uint64_t *next_sequence)
{
    if (*next_sequence == 0u) *next_sequence = 1u;
    IVP_ASSERT(*next_sequence != UINT64_MAX);
    if (*next_sequence == UINT64_MAX) return 0u;
    return (*next_sequence)++;
}

void ivp_env_register_sim_controller(
    struct ivp_environment *env, struct ivp_sim_ctrl *controller)
{
    if (!env || !env->pl || !controller ||
        ivp_build_uses_reference_internal_state() ||
        controller->registration_sequence != 0u) {
        return;
    }
    controller->registration_sequence =
        env_take_sequence(&env->pl->next_controller_sequence);
}

uint64_t ivp_env_next_contact_sequence(struct ivp_environment *env)
{
    if (!env || !env->pl ||
        ivp_build_uses_reference_internal_state()) {
        return 0u;
    }
    return env_take_sequence(&env->pl->next_contact_sequence);
}

uint64_t ivp_env_register_cluster(struct ivp_environment *env)
{
#if IVP_STABLE_INTERNAL_STATE
    if (!env || !env->pl) return 0u;
    uint64_t sequence = env_take_sequence(&env->pl->next_object_sequence);
    if (sequence != 0u) env->pl->live_cluster_count++;
    return sequence;
#else
    (void)env;
    return 0u;
#endif
}

void ivp_env_unregister_cluster(struct ivp_environment *env,
                                uint64_t creation_sequence)
{
#if IVP_STABLE_INTERNAL_STATE
    if (!env || !env->pl || creation_sequence == 0u) return;
    IVP_ASSERT(env->pl->live_cluster_count > 0);
    if (env->pl->live_cluster_count > 0) {
        env->pl->live_cluster_count--;
    }
    if (env->pl->live_cluster_count == 0 &&
        ivp_object_system_count(env_object_system(env)) == 0) {
        env->pl->next_object_sequence = 1u;
        env->pl->next_contact_sequence = 1u;
    }
#else
    (void)env;
    (void)creation_sequence;
#endif
}

void *ivp_env_sim_mem_alloc(struct ivp_environment *env, size_t size)
{
    /* IVP_U_Memory::get_mem_transaction (ivu_memory.hxx:121) */
    IVP_ASSERT(env->pl->sim_unit_mem.transaction_depth == 1);
    const size_t request = size ? size : 1;
    return ivp_arena_alloc(&env->pl->sim_unit_mem, request);
}

void ivp_env_sim_mem_begin(struct ivp_environment *env)
{
    ivp_arena_begin_transaction(&env->pl->sim_unit_mem);
}

void ivp_env_sim_mem_end(struct ivp_environment *env)
{
    /* Inner transactions share the outer allocation lifetime because
     * tmp_contact_info may still reference their memory. */
    ivp_arena_end_transaction(&env->pl->sim_unit_mem);
}

/* ── IVP_Environment accessors (ivp_env_internal.h) ───────────────────── */

ivp_time_t ivp_env_get_current_time(const struct ivp_environment *env) { return env->current_time; }
ivp_time_t ivp_env_get_next_psi_time(const struct ivp_environment *env) { return env->pl->time_of_next_psi; }
ivp_time_t ivp_env_get_old_time_of_last_psi(const struct ivp_environment *env) { return env->pl->time_of_last_psi; }
float ivp_env_get_delta_psi_time(const struct ivp_environment *env) { return (float)env->pl->delta_psi_time; }
float ivp_env_get_inv_delta_psi_time(const struct ivp_environment *env) { return (float)env->pl->inv_delta_psi_time; }
double ivp_env_get_integrated_energy_damp(const struct ivp_environment *env) { return env->pl->integrated_energy_damp; }
ivp_time_code_t ivp_env_get_current_time_code(const struct ivp_environment *env) { return env->pl->current_time_code; }
#ifdef IVP_DEBUG_BUILD
/* IVP_Environment::invalid_time_code (ivp_environment.hxx, #ifdef DEBUG) */
void ivp_env_invalid_time_code(struct ivp_environment *env) { env->pl->current_time_code++; }
#endif
ivp_env_state_t ivp_env_get_state(const struct ivp_environment *env) { return env->pl->state; }
void ivp_env_set_state(struct ivp_environment *env, ivp_env_state_t s) { env->pl->state = s; }
ivp_time_code_t ivp_env_get_mindist_event_timestamp_reference(const struct ivp_environment *env)
{
    return env->pl->mindist_event_timestamp_reference;
}
void ivp_env_inc_mindist_event_timestamp_reference(struct ivp_environment *env) { env->pl->mindist_event_timestamp_reference++; }
void ivp_env_get_gravity_vec(const struct ivp_environment *env, double out[3])
{
    out[0] = (double)env->gravity.k[0];
    out[1] = (double)env->gravity.k[1];
    out[2] = (double)env->gravity.k[2];
}
double ivp_env_get_gravity_scalar(const struct ivp_environment *env) { return env->pl->gravity_scalar; }
const ivp_allocator_t *ivp_env_get_allocator(const struct ivp_environment *env) { return env->allocator; }
struct ivp_time_manager *ivp_env_get_time_manager(struct ivp_environment *env) { return &env->pl->time_manager; }
struct ivp_time_manager *ivp_environment_get_time_manager(ivp_environment_t *env) { return env ? &env->pl->time_manager : NULL; }
struct ivp_mindist_mgr *ivp_env_get_mindist_mgr(struct ivp_environment *env) { return &env->pl->mindist_mgr; }
struct ivp_ov_tree *ivp_env_get_ov_tree(struct ivp_environment *env) { return &env->pl->ov_tree; }
ivp_sim_units_manager_t *ivp_env_get_sim_units_manager(struct ivp_environment *env) { return &env->pl->sim_units_manager; }
ivp_sim_ctrl_t *ivp_env_get_standard_gravity_controller(struct ivp_environment *env) { return &env->pl->gravity_ctrl; }
const ivp_mindist_settings_t *ivp_env_get_mindist_settings(const struct ivp_environment *env)
{
    return &env->pl->mindist_settings;
}
ivp_mindist_t *ivp_env_get_current_mindist(const struct ivp_environment *env)
{
    return env->pl->current_mindist;
}
void ivp_env_set_current_mindist(struct ivp_environment *env, ivp_mindist_t *mindist)
{
    env->pl->current_mindist = mindist;
}
bool ivp_env_get_defer_delete_mindist(const struct ivp_environment *env)
{
    return env->pl->defer_delete_mindist;
}
void ivp_env_set_defer_delete_mindist(struct ivp_environment *env, bool defer_delete)
{
    env->pl->defer_delete_mindist = defer_delete;
}
int ivp_env_next_friction_debug_counter(struct ivp_environment *env)
{
    return ++env->pl->friction_debug_counter;
}
int ivp_env_next_global_friction_debug_counter(struct ivp_environment *env)
{
    return ++env->pl->global_friction_debug_counter;
}

/* IVP_Environment::set_current_time */
void ivp_environment_set_current_time_internal(struct ivp_environment *env, ivp_time_t t)
{
    env->pl->current_time_code++;
    env->current_time = t;
}

/* ivp_rand (ivu_os_dep.cxx), one generator per environment */
float ivp_env_rand(struct ivp_environment *env)
{
    unsigned int next_seed = (unsigned int)env->pl->rand_seed * 75U;
    memcpy(&env->pl->rand_seed, &next_seed, sizeof(int));
    return (float)(env->pl->rand_seed & 0xffff) / (float)0x10000;
}

/* ── diagnostics (IVP_PerformanceCounter) ──────────────────────────────── */

void ivp_env_start_pcount(struct ivp_environment *env)
{
    if (env && env->performance_counter) ivp_performance_counter_start_pcount(env->performance_counter);
}

void ivp_env_pcount(struct ivp_environment *env, ivp_performance_element_t el)
{
    if (env && env->performance_counter) ivp_performance_counter_pcount(env->performance_counter, el);
}

void ivp_env_stop_pcount(struct ivp_environment *env)
{
    if (env && env->performance_counter) ivp_performance_counter_stop_pcount(env->performance_counter);
}

ivp_performance_counter_t *ivp_environment_get_performance_counter(const ivp_environment_t *env)
{
    return env ? env->performance_counter : NULL;
}

ivp_better_statistics_manager_t *ivp_environment_get_better_statistics_manager(const ivp_environment_t *env)
{
    return env ? env->better_statistics_manager : NULL;
}

ivp_debug_manager_t *ivp_environment_get_debug_manager(const ivp_environment_t *env)
{
    return env ? env->debug_manager : NULL;
}

/* IVP_Environment::add_draw_vector */
void ivp_env_add_draw_vector(struct ivp_environment *env, const double start_p[3], const float dir_vec[3],
                             const char *debug_text_in, int v_color)
{
    IVP_IF(1)
    {
        ivp_env_pipeline_t *pl = env->pl;
        /* new IVP_Draw_Vector_Debug() */
        ivp_draw_vector_debug_t *draw_vector =
            (ivp_draw_vector_debug_t *)ivp_alloc(env->allocator, sizeof(ivp_draw_vector_debug_t));
        if (!draw_vector) return;
        for (int i = 0; i < 3; i++) {
            draw_vector->first_point.k[i] = start_p[i];
            draw_vector->direction_vec.k[i] = (double)dir_vec[i];
        }
        draw_vector->debug_text = ivp_string_dup(env->allocator, debug_text_in);
        draw_vector->color = v_color;
        draw_vector->next = pl->draw_vectors;
        pl->draw_vectors = draw_vector;
    }
}

/* IVP_Environment::delete_draw_vector_debug */
void ivp_env_delete_draw_vector_debug(struct ivp_environment *env)
{
    ivp_env_pipeline_t *pl = env->pl;
    ivp_draw_vector_debug_t *dv = pl->draw_vectors;
    while (dv != NULL) {
        ivp_draw_vector_debug_t *next_dv = dv->next;
        ivp_string_free(env->allocator, dv->debug_text);
        ivp_free(env->allocator, dv, sizeof(ivp_draw_vector_debug_t));
        dv = next_dv;
    }
    pl->draw_vectors = NULL;
}

const ivp_draw_vector_debug_t *ivp_environment_get_draw_vectors(const ivp_environment_t *env)
{
    return (env && env->pl) ? env->pl->draw_vectors : NULL;
}

ivp_statistic_manager_t *ivp_environment_get_statistic_manager(const ivp_environment_t *env)
{
    return (env && env->pl) ? &env->pl->statistic_manager : NULL;
}

ivp_statistic_manager_t *ivp_env_get_statistic_manager(struct ivp_environment *env)
{
    return &env->pl->statistic_manager;
}

ivp_debug_manager_t *ivp_env_get_debug_manager(struct ivp_environment *env)
{
    return env->debug_manager;
}

#if IVP_EDIT_LOCALITY
/* SplitMix64 finalizer */
static uint64_t env_mix_salt(uint64_t value)
{
    value ^= value >> 30;
    value *= UINT64_C(0xbf58476d1ce4e5b9);
    value ^= value >> 27;
    value *= UINT64_C(0x94d049bb133111eb);
    value ^= value >> 31;
    return value;
}

/* The creation sequence orders objects but its value shifts whenever an
 * earlier object is added or removed, so the salt uses only the PSI the
 * object was created in and its initial position. */
static uint64_t env_movement_check_salt(const ivp_environment_t *env,
                                        const ivp_object_template_t *tmpl)
{
    uint64_t salt = env_mix_salt(env->pl->psi_index);
    for (int axis = 0; axis < 3; axis++) {
        uint64_t bits = 0u;
        memcpy(&bits, &tmpl->position.k[axis], sizeof(tmpl->position.k[axis]));
        salt = env_mix_salt(salt ^ bits);
    }
    return salt;
}
#endif

/* IVP_Environment::must_perform_movement_check
 *
 * The reference counts one countdown down across every simulated unit and
 * draws each interval from the environment RNG, so adding or removing an
 * object anywhere in the world shifts which units are checked and when they
 * freeze.  With IVP_EDIT_LOCALITY each unit is checked every
 * IVP_MOVEMENT_CHECK_PERIOD PSIs at a phase taken from the salt of its first
 * object in stable order; the unit's membership, and with it the phase,
 * changes only through its own interactions. */
bool ivp_env_must_perform_movement_check(struct ivp_environment *env,
                                         ivp_sim_unit_t *su)
{
    ivp_env_pipeline_t *pl = env->pl;
#if IVP_EDIT_LOCALITY
    uint64_t phase = ivp_sim_unit_movement_check_salt(su) %
                     (uint64_t)IVP_MOVEMENT_CHECK_PERIOD;
    return (pl->psi_index + phase) %
               (uint64_t)IVP_MOVEMENT_CHECK_PERIOD == 0u;
#else
    (void)su;
    pl->next_movement_check--;
    if (pl->next_movement_check == 0) {
        pl->next_movement_check =
            (short)(IVP_MOVEMENT_CHECK_COUNT * 3 / 2 + (short)(ivp_env_rand(env) * (IVP_MOVEMENT_CHECK_COUNT / 2)));
        return true;
    }
    return false;
#endif
}

double ivp_env_get_freeze_check_dtime(struct ivp_environment *env)
{
    return (double)(float)env->freeze_params.freeze_check_dtime;
}

/* IVP_Environment::range_manager */
void ivp_env_get_coll_range_intra_objects(struct ivp_environment *env, const struct ivp_object *obj0,
                                          const struct ivp_object *obj1, double *range0, double *range1)
{
    /* max_dist -= sum_speed * environment->get_delta_PSI_time(): IVP_FLOAT getter */
    ivp_range_manager_coll_range_intra_objects(env->pl->range_manager, (double)ivp_env_get_delta_psi_time(env), obj0, obj1,
                                               range0, range1);
}

double ivp_env_get_coll_range_in_world(struct ivp_environment *env, const struct ivp_object *obj0)
{
    return ivp_range_manager_coll_range_in_world(env->pl->range_manager, (double)ivp_env_get_delta_psi_time(env), obj0);
}

ivp_range_manager_t *ivp_environment_get_range_manager(const ivp_environment_t *env)
{
    return (env && env->pl) ? env->pl->range_manager : NULL;
}

/* IVP_Collision_Filter_Coll_Group_Ident + libivp filters */
bool ivp_env_check_objects_for_collision_detection(struct ivp_environment *env, struct ivp_object *o0, struct ivp_object *o1)
{
    if (o0->nocoll_group_ident[0] != 0 && o1->nocoll_group_ident[0] != 0) {
        if (strncmp(o0->nocoll_group_ident, o1->nocoll_group_ident, sizeof(o0->nocoll_group_ident)) == 0) return false;
    }
    if (ivp_objects_share_no_collision_group(o0, o1)) return false;
    if (o0->collision_group != 0 && o0->collision_group == o1->collision_group) return false;
    if ((o0->layer_mask & o1->layer_mask) == 0) return false;
    return ivp_collision_filter_check(&env->collision_filter, o0, o1);
}

int ivp_env_get_collision_delegator_root_count(const struct ivp_environment *env) { return env->collision_delegator_root_count; }
struct ivp_collision_delegator_root *ivp_env_get_collision_delegator_root(struct ivp_environment *env, int i)
{
    return env->collision_delegator_roots[i];
}

/* IVP_Environment::fire_object_is_removed_from_collision_detection */
void ivp_env_fire_object_is_removed_from_collision_detection(struct ivp_environment *env, struct ivp_object *obj)
{
    for (int k = env->collision_delegator_root_count - 1; k >= 0; k--) {
        ivp_collision_delegator_root_t *cdr = env->collision_delegator_roots[k];
        cdr->rvt->object_is_removed_from_collision_detection(cdr, obj);
    }
}

bool ivp_env_has_universe_manager(const struct ivp_environment *env)
{
    return env->universe_manager && env->universe_manager->ensure_fn;
}

void ivp_env_universe_ensure_objects(struct ivp_environment *env, struct ivp_object *obj, const float center[3], double radius)
{
    if (ivp_env_has_universe_manager(env)) {
        env->universe_manager->ensure_fn(env->universe_manager->ctx, obj, center, radius);
    }
}

/* IVP_Environment::add_revive_core / remove_revive_core */
bool ivp_env_add_revive_core(struct ivp_environment *env, struct ivp_core *c)
{
    if (c->is_in_wakeup_vec) return true;
    IVP_IF(1)
    {
        IVP_ASSERT(ivp_pvec_index_of(&env->pl->core_revive_list, c) < 0);
    }
    if (ivp_pvec_add(&env->pl->core_revive_list, c) < 0) return false;
    c->is_in_wakeup_vec = true;
    return true;
}

void ivp_env_remove_revive_core(struct ivp_environment *env, struct ivp_core *c)
{
    int index = ivp_pvec_index_of(&env->pl->core_revive_list, c);
    if (index >= 0) {
        IVP_ASSERT(c->is_in_wakeup_vec == true);
        ivp_pvec_remove_at(&env->pl->core_revive_list, index);
        c->is_in_wakeup_vec = false;
    }
}

/* IVP_Cluster_Manager::number_of_real_objects: the objects plus the static
 * object (both in the root cluster) */
static int env_number_of_real_objects(const ivp_environment_t *env)
{
    return ivp_environment_get_object_count(env) + (env->pl->static_object ? 1 : 0);
}

/* IVP_Cluster_Manager::get_next_real_object_in_cluster_tree (NULL -> first
 * object, last -> NULL) */
static ivp_object_t *env_next_object_to_check(ivp_environment_t *env, ivp_object_t *object)
{
    if (env_number_of_real_objects(env) == 0) return NULL;
    return ivp_cluster_get_next_real_object_in_tree(&env->pl->root_cluster, object);
}

/* IVP_Cluster_Manager::check_for_unused_objects */
static void env_check_for_unused_objects(ivp_environment_t *env)
{
    ivp_universe_manager_t *um = env->universe_manager;
    ivp_env_pipeline_t *pl = env->pl;
    if (!um->provide_settings) return;
    if (env_number_of_real_objects(env) < um->num_objects_in_environment_threshold_0) return;
    if (!ivp_build_uses_reference_internal_state()) {
        ivp_cluster_restore_child_order(&pl->root_cluster);
    }
    /* int(int * IVP_FLOAT get_delta_PSI_time()): float product */
    for (int i = (int)((float)um->check_objects_per_second_threshold_0 * ivp_env_get_delta_psi_time(env)); i >= 0; i--) {
        ivp_object_t *an_object = pl->an_object_to_be_checked;
        pl->an_object_to_be_checked = env_next_object_to_check(env, an_object);
        if (!an_object) return; /* no objects available */
        if (ivp_object_get_collision_check_reference_count(an_object) != 0) continue;
        if (um->evict_fn) um->evict_fn(um->ctx, an_object);
        if (env_number_of_real_objects(env) < um->num_objects_in_environment_threshold_0) return;
    }
    if (env_number_of_real_objects(env) < um->num_objects_in_environment_threshold_1) return;
    for (int i = (int)((float)um->check_objects_per_second_threshold_1 * ivp_env_get_delta_psi_time(env)); i >= 0; i--) {
        ivp_object_t *an_object = pl->an_object_to_be_checked;
        if (!an_object) return;
        pl->an_object_to_be_checked = env_next_object_to_check(env, an_object);
        if (ivp_object_get_collision_check_reference_count(an_object) != 1) continue;
        if (um->evict_fn) um->evict_fn(um->ctx, an_object);
        if (env_number_of_real_objects(env) < um->num_objects_in_environment_threshold_1) return;
    }
}

/* IVP_Environment::revive_cores_PSI */
static void env_restore_revive_core_order(ivp_pvec_t *cores)
{
    if (ivp_build_uses_reference_internal_state()) return;
    ivp_core_vector_sort_by_identity(cores);
}

static void env_revive_cores_psi(ivp_environment_t *env)
{
    ivp_pvec_t *l = &env->pl->core_revive_list;
    /* Revival fires callbacks and can fuse simulation units.  Normalize the
     * queue before either effect becomes observable; allocator and enqueue
     * history must not select the order. */
    env_restore_revive_core_order(l);
    for (int i = ivp_pvec_len(l) - 1; i >= 0; i--) {
        if (i >= ivp_pvec_len(l)) continue;
        ivp_core_t *my_core = (ivp_core_t *)ivp_pvec_at(l, i);
        IVP_ASSERT(!my_core->physical_unmoveable);
        ivp_core_ensure_core_to_be_in_simulation(my_core);
        my_core->is_in_wakeup_vec = false;
    }
    ivp_pvec_remove_all(l);
}

/* IVP_Anomaly_Limits / IVP_Anomaly_Manager */
int ivp_env_get_max_collision_checks_per_psi(const struct ivp_environment *env) { return env->anomaly_manager.config.max_collision_checks_per_psi; }
float ivp_env_anomaly_get_max_velocity(struct ivp_environment *env) { return (float)env->anomaly_manager.config.max_speed; }
float ivp_env_anomaly_get_max_angular_velocity_per_psi(struct ivp_environment *env)
{
    return env->anomaly_manager.config.max_angular_velocity_per_psi;
}
int ivp_env_anomaly_get_max_collisions_per_psi(struct ivp_environment *env) { return env->anomaly_manager.config.max_collisions_per_psi; }

void ivp_env_anomaly_max_velocity_exceeded(struct ivp_environment *env, ivp_core_t *core, ivp_vec3f_t *velocity)
{
    (void)core;
    double square_speed = (double)lvf_quad_length(velocity);
    double max_speed = (double)ivp_env_anomaly_get_max_velocity(env);
    lvf_mult(velocity, (D)((double)0.99f * max_speed / sqrt(square_speed)));
    env->anomaly_manager.anomalies_detected++;
    env->anomaly_manager.anomalies_corrected++;
}

void ivp_env_anomaly_max_angular_velocity_exceeded(struct ivp_environment *env, ivp_core_t *core, ivp_vec3f_t *rot)
{
    (void)core;
    double square_rot_speed = (double)lvf_quad_length(rot);
    double max_rot_speed = (double)(ivp_env_anomaly_get_max_angular_velocity_per_psi(env) * ivp_env_get_inv_delta_psi_time(env));
    lvf_mult(rot, (D)((double)0.9f * max_rot_speed / sqrt(square_rot_speed)));
    env->anomaly_manager.anomalies_detected++;
    env->anomaly_manager.anomalies_corrected++;
}

bool ivp_env_anomaly_max_collisions_exceeded_check_freezing(struct ivp_environment *env, ivp_core_t *core)
{
    env->anomaly_manager.anomalies_detected++;
    if (env->anomaly_manager.config.callback) {
        env->anomaly_manager.config.callback(env->anomaly_manager.config.callback_ctx,
                                             ivp_core_n_objects(core) > 0 ? ivp_core_object_at(core, 0) : NULL,
                                             IVP_ANOMALY_MAX_COLLISIONS, "max collisions per PSI exceeded");
    }
    return true;
}

/* ── materials (IVP_Material_Manager) ──────────────────────────────────── */

static void env_material_situation(ivp_impact_solver_lt_t *info, ivp_contact_situation_t *sit)
{
    sit->obj_a = info->objects[0];
    sit->obj_b = info->objects[1];
    sit->contact_point_ws = info->contact_point_ws;
    lvp_set_f(&sit->contact_normal_ws, &info->surf_normal);
}

float ivp_env_material_get_elasticity(struct ivp_environment *env, struct ivp_impact_solver_lt *info)
{
    if (env->material_manager) {
        ivp_contact_situation_t sit;
        env_material_situation(info, &sit);
        ivp_real_t f = info->materials[0]->friction * info->materials[1]->friction;
        ivp_real_t e = info->materials[0]->elasticity * info->materials[1]->elasticity;
        ivp_real_t a = info->materials[0]->adhesion + info->materials[1]->adhesion;
        env->material_manager(env->material_manager_ctx, &sit, &f, &e, &a);
        return (float)e;
    }
    return (float)((double)info->materials[0]->elasticity * (double)info->materials[1]->elasticity);
}

float ivp_env_material_get_friction_factor(struct ivp_environment *env, struct ivp_impact_solver_lt *info)
{
    if (env->material_manager) {
        ivp_contact_situation_t sit;
        env_material_situation(info, &sit);
        ivp_real_t f = info->materials[0]->friction * info->materials[1]->friction;
        ivp_real_t e = info->materials[0]->elasticity * info->materials[1]->elasticity;
        ivp_real_t a = info->materials[0]->adhesion + info->materials[1]->adhesion;
        env->material_manager(env->material_manager_ctx, &sit, &f, &e, &a);
        return (float)f;
    }
    return (float)((double)info->materials[0]->friction * (double)info->materials[1]->friction);
}

/* ── collision / friction / object events ──────────────────────────────── */

/* callback tables per object (IVP_Object_Callback_Table / IVP_Collision_Callback_Table) */
typedef struct env_obj_cb_table {
    ivp_object_t *real_object;
    ivp_pvec_t    listeners;
} env_obj_cb_table_t;

static env_obj_cb_table_t *cb_find_table(const ivp_pvec_t *tables, const ivp_object_t *obj)
{
    for (int i = 0; i < ivp_pvec_len(tables); i++) {
        env_obj_cb_table_t *t = (env_obj_cb_table_t *)ivp_pvec_at(tables, i);
        if (t->real_object == obj) return t;
    }
    return NULL;
}

static env_obj_cb_table_t *cb_create_table(ivp_pvec_t *tables, ivp_object_t *obj)
{
    const ivp_allocator_t *allocator = ivp_pvec_allocator(tables);
    env_obj_cb_table_t *t = (env_obj_cb_table_t *)ivp_alloc(allocator, sizeof(*t));
    if (!t) return NULL;
    memset(t, 0, sizeof(*t));
    t->real_object = obj;
    ivp_pvec_init_with_allocator(&t->listeners, allocator);
    if (ivp_pvec_add(tables, t) < 0) {
        ivp_free(allocator, t, sizeof(*t));
        return NULL;
    }
    return t;
}

static void cb_remove_table(ivp_pvec_t *tables, env_obj_cb_table_t *t)
{
    const ivp_allocator_t *allocator = ivp_pvec_allocator(tables);
    (void)ivp_pvec_remove_if_present(tables, t);
    ivp_pvec_clear(&t->listeners);
    ivp_free(allocator, t, sizeof(*t));
}

static void cb_clear_tables(ivp_pvec_t *tables)
{
    while (ivp_pvec_len(tables) > 0) cb_remove_table(tables, (env_obj_cb_table_t *)ivp_pvec_at(tables, 0));
    ivp_pvec_clear(tables);
}

/* ── IVP_Listener_Collision ─────────────────────────────────────────────── */

enum { COLL_PRE, COLL_POST, COLL_FR_CREATED, COLL_FR_DELETED, COLL_OBJ_DELETED, COLL_PAIR_CREATED, COLL_PAIR_DELETED };

static int coll_kind_flag(int kind)
{
    switch (kind) {
    case COLL_PRE: return IVP_COLLISION_CB_PRE_COLLISION;
    case COLL_POST: return IVP_COLLISION_CB_POST_COLLISION;
    case COLL_FR_CREATED:
    case COLL_PAIR_CREATED: return IVP_COLLISION_CB_FRICTION_CREATED;
    case COLL_FR_DELETED:
    case COLL_PAIR_DELETED: return IVP_COLLISION_CB_FRICTION_DELETED;
    default: return IVP_COLLISION_CB_OBJECT_DELETED;
    }
}

static void coll_listener_call(ivp_collision_listener_t *l, int kind, const void *ev)
{
    if (!l->active || !(l->enabled_callbacks & coll_kind_flag(kind))) return;
    switch (kind) {
    case COLL_PRE: if (l->pre_collision_fn) l->pre_collision_fn(l->ctx, (const ivp_collision_event_t *)ev); break;
    case COLL_POST: if (l->post_collision_fn) l->post_collision_fn(l->ctx, (const ivp_collision_event_t *)ev); break;
    case COLL_FR_CREATED: if (l->friction_created_fn) l->friction_created_fn(l->ctx, (const ivp_friction_event_t *)ev); break;
    case COLL_FR_DELETED: if (l->friction_deleted_fn) l->friction_deleted_fn(l->ctx, (const ivp_friction_event_t *)ev); break;
    case COLL_OBJ_DELETED: if (l->object_deleted_fn) l->object_deleted_fn(l->ctx, (ivp_object_t *)(void *)(uintptr_t)ev); break;
    case COLL_PAIR_CREATED: if (l->friction_pair_created_fn) l->friction_pair_created_fn(l->ctx, (struct ivp_fr_core_pair *)(uintptr_t)ev); break;
    case COLL_PAIR_DELETED: if (l->friction_pair_deleted_fn) l->friction_pair_deleted_fn(l->ctx, (struct ivp_fr_core_pair *)(uintptr_t)ev); break;
    default: break;
    }
}

/* IVP_Environment::fire_event_* (global listeners, last added first) */
static void fire_collision_global(ivp_environment_t *env, int kind, const void *ev)
{
    ivp_pvec_t *v = &env->pl->global_collision_listeners;
    for (int i = ivp_pvec_len(v) - 1; i >= 0; i--) {
        if (i >= ivp_pvec_len(v)) continue;
        coll_listener_call((ivp_collision_listener_t *)ivp_pvec_at(v, i), kind, ev);
    }
}

/* IVP_Cluster_Manager::fire_event_* (private listeners of one object) */
void ivp_env_fire_collision_private(ivp_environment_t *env, ivp_object_t *obj, int kind, const void *ev)
{
    if (!obj->collision_listener_exists) return;
    ivp_pvec_t *tables = &env->pl->coll_callback_tables;
    env_obj_cb_table_t *t = cb_find_table(tables, obj);
    if (!t) return;
    for (int i = ivp_pvec_len(&t->listeners) - 1; i >= 0; i--) {
        if (i >= ivp_pvec_len(&t->listeners)) continue;
        coll_listener_call((ivp_collision_listener_t *)ivp_pvec_at(&t->listeners, i), kind, ev);
        if (i > 0 && cb_find_table(tables, obj) != t) break; /* object deleted */
    }
}

int ivp_env_add_listener_collision_private(ivp_environment_t *env, ivp_object_t *obj, ivp_collision_listener_t *l)
{
    ivp_pvec_t *tables = &env->pl->coll_callback_tables;
    env_obj_cb_table_t *t = cb_find_table(tables, obj);
    bool created = false;
    if (!t) {
        t = cb_create_table(tables, obj);
        if (!t) return -1;
        created = true;
    }
    if (ivp_pvec_add(&t->listeners, l) < 0) {
        if (created) {
            cb_remove_table(tables, t);
            obj->collision_listener_exists = false;
        }
        return -1;
    }
    if (created) obj->collision_listener_exists = true;
    return 0;
}

int ivp_env_remove_listener_collision_private(ivp_environment_t *env, ivp_object_t *obj, ivp_collision_listener_t *l)
{
    ivp_pvec_t *tables = &env->pl->coll_callback_tables;
    env_obj_cb_table_t *t = cb_find_table(tables, obj);
    if (!t) return -1;
    bool found = ivp_pvec_remove_if_present(&t->listeners, l);
    if (ivp_pvec_len(&t->listeners) == 0) {
        cb_remove_table(tables, t);
        obj->collision_listener_exists = false;
    }
    return found ? 0 : -1;
}

static void make_collision_event(ivp_environment_t *env, ivp_impact_solver_lt_t *info, double d_time_since_last,
                                 ivp_collision_event_t *ev)
{
    memset(ev, 0, sizeof(*ev));
    ev->obj_a = info->objects[0];
    ev->obj_b = info->objects[1];
    ev->contact_point = info->contact_point_ws;
    lvp_set_f(&ev->contact_normal, &info->surf_normal);
    ev->closing_speed = (ivp_real_t)lvf_dot(&info->speed, &info->surf_normal);
    ev->d_time_since_last_collision = (ivp_real_t)(float)d_time_since_last; /* IVP_FLOAT */
    ev->environment = env;
    lvp_set_f(&ev->speed, &info->speed);
    ev->compact_edges[0] = info->compact_edges[0];
    ev->compact_edges[1] = info->compact_edges[1];
}

/* IVP_Mindist::do_impact_of_two_objects: pre collision */
void ivp_env_fire_pre_collision(struct ivp_environment *env, struct ivp_impact_solver_lt *info, double d_time_since_last)
{
    ivp_collision_event_t ev;
    make_collision_event(env, info, d_time_since_last, &ev);
    ivp_object_t *obj0 = info->objects[0], *obj1 = info->objects[1];
    fire_collision_global(env, COLL_PRE, &ev);
    ivp_env_fire_collision_private(env, obj0, COLL_PRE, &ev);
    ivp_env_fire_collision_private(env, obj1, COLL_PRE, &ev);
}

/* post collision (contact situation speed already restored by the caller) */
void ivp_env_fire_post_collision(struct ivp_environment *env, struct ivp_impact_solver_lt *info, double d_time_since_last)
{
    ivp_collision_event_t ev;
    make_collision_event(env, info, d_time_since_last, &ev);
    ivp_object_t *obj0 = info->objects[0], *obj1 = info->objects[1];
    int i0 = obj0->dense_index, i1 = obj1->dense_index;
    fire_collision_global(env, COLL_POST, &ev);
    ivp_env_fire_collision_private(env, obj0, COLL_POST, &ev);
    ivp_env_fire_collision_private(env, obj1, COLL_POST, &ev);
    ivp_environment_emit_event_internal(env, IVP_EVENT_COLLISION_DETECTED, IVP_REAL(0.0), i0, i1);
}

static void fire_friction_listeners(ivp_environment_t *env, ivp_contact_pt_t *cp, bool deleted)
{
    ivp_friction_event_t ev;
    memset(&ev, 0, sizeof(ev));
    ivp_object_t *obj0 = cp->synapse[0].l_obj;
    ivp_object_t *obj1 = cp->synapse[1].l_obj;
    ev.obj_a = obj0;
    ev.obj_b = obj1;
    if (deleted || !cp->tmp_contact_info) {
        /* friction deleted: surf_normal_ws / last_contact_point_ws */
        lvp_set_f(&ev.contact_point, &cp->last_contact_point_ws);
        lvp_set_f(&ev.contact_normal, &cp->surf_normal_ws);
        ev.compact_edges[0] = cp->synapse[0].edge;
        ev.compact_edges[1] = cp->synapse[1].edge;
    } else {
        /* friction created: contact_situation = tmp_contact_info */
        ev.contact_point = cp->tmp_contact_info->contact_point_ws;
        lvp_set_f(&ev.contact_normal, &cp->tmp_contact_info->surf_normal);
        ev.compact_edges[0] = cp->tmp_contact_info->compact_edges[0];
        ev.compact_edges[1] = cp->tmp_contact_info->compact_edges[1];
    }
    ev.normal_force = (ivp_real_t)cp->now_friction_pressure;
    ev.environment = env;
    ev.friction_handle = cp;
    int kind = deleted ? COLL_FR_DELETED : COLL_FR_CREATED;
    fire_collision_global(env, kind, &ev);
    ivp_env_fire_collision_private(env, obj0, kind, &ev);
    ivp_env_fire_collision_private(env, obj1, kind, &ev);
}

void ivp_env_fire_friction_created(struct ivp_environment *env, struct ivp_contact_pt *cp) { fire_friction_listeners(env, cp, false); }
void ivp_env_fire_friction_deleted(struct ivp_environment *env, struct ivp_contact_pt *cp) { fire_friction_listeners(env, cp, true); }

/* IVP_Friction_System::add_fr_pair / del_fr_pair */
void ivp_env_fire_friction_pair_created(struct ivp_environment *env, struct ivp_fr_core_pair *pair)
{
    fire_collision_global(env, COLL_PAIR_CREATED, (const void *)pair);
}
void ivp_env_fire_friction_pair_deleted(struct ivp_environment *env, struct ivp_fr_core_pair *pair)
{
    fire_collision_global(env, COLL_PAIR_DELETED, (const void *)pair);
}

/* IVP_Contact_Point_API */
float ivp_contact_point_get_eliminated_energy(const struct ivp_contact_pt *fh) { return fh->integrated_destroyed_energy; }
void ivp_contact_point_reset_eliminated_energy(struct ivp_contact_pt *fh) { fh->integrated_destroyed_energy = 0.0f; }
float ivp_contact_point_get_vert_force(const struct ivp_contact_pt *fh) { return fh->now_friction_pressure; }
void ivp_contact_point_get_surface_normal_ws(const struct ivp_contact_pt *fh, ivp_vec3r_t *normal)
{
    lvp_set_f(normal, &fh->surf_normal_ws);
}

/* ── IVP_Listener_Object (global list + IVP_Cluster_Manager::obj_callback_hash) ── */

static env_obj_cb_table_t *obj_cb_find_table(ivp_env_pipeline_t *pl, const ivp_object_t *obj)
{
    return cb_find_table(&pl->obj_callback_tables, obj);
}

static void obj_cb_remove_table(ivp_env_pipeline_t *pl, env_obj_cb_table_t *t)
{
    cb_remove_table(&pl->obj_callback_tables, t);
}

void ivp_environment_add_listener_object_global(ivp_environment_t *env, ivp_listener_object_t *listener)
{
    if (!env || !listener) return;
    ivp_pvec_add(&env->pl->global_object_listeners, listener);
}

void ivp_environment_install_listener_object_global(ivp_environment_t *env, ivp_listener_object_t *listener)
{
    if (!env || !listener) return;
    (void)ivp_pvec_install(&env->pl->global_object_listeners, listener);
}

void ivp_environment_remove_listener_object_global(ivp_environment_t *env, ivp_listener_object_t *listener)
{
    if (!env || !listener) return;
    (void)ivp_pvec_remove_if_present(&env->pl->global_object_listeners, listener);
}

void ivp_object_add_listener_object(ivp_object_t *obj, ivp_listener_object_t *listener)
{
    if (!obj || !listener || !obj->owner) return;
    ivp_env_pipeline_t *pl = obj->owner->pl;
    env_obj_cb_table_t *t = obj_cb_find_table(pl, obj);
    bool created = false;
    if (!t) {
        t = cb_create_table(&pl->obj_callback_tables, obj);
        if (!t) return;
        created = true;
    }
    if (ivp_pvec_add(&t->listeners, listener) < 0 && created) {
        obj_cb_remove_table(pl, t);
        return;
    }
    if (created) obj->object_listener_exists = true;
}

void ivp_object_remove_listener_object(ivp_object_t *obj, ivp_listener_object_t *listener)
{
    if (!obj || !listener || !obj->owner) return;
    ivp_env_pipeline_t *pl = obj->owner->pl;
    env_obj_cb_table_t *t = obj_cb_find_table(pl, obj);
    if (!t) return;
    (void)ivp_pvec_remove_if_present(&t->listeners, listener);
    if (ivp_pvec_len(&t->listeners) == 0) {
        obj_cb_remove_table(pl, t);
        obj->object_listener_exists = false;
    }
}

typedef enum { OBJ_EV_DELETED, OBJ_EV_CREATED, OBJ_EV_REVIVED, OBJ_EV_FROZEN } obj_ev_kind_t;

static void obj_listener_call(ivp_listener_object_t *l, obj_ev_kind_t kind, ivp_event_object_t *ev)
{
    switch (kind) {
    case OBJ_EV_DELETED: if (l->event_object_deleted) l->event_object_deleted(l, ev); break;
    case OBJ_EV_CREATED: if (l->event_object_created) l->event_object_created(l, ev); break;
    case OBJ_EV_REVIVED: if (l->event_object_revived) l->event_object_revived(l, ev); break;
    case OBJ_EV_FROZEN:  if (l->event_object_frozen) l->event_object_frozen(l, ev); break;
    }
}

/* IVP_Environment::fire_event_object_* */
static void fire_object_event_global(ivp_environment_t *env, ivp_object_t *obj, obj_ev_kind_t kind)
{
    ivp_pvec_t *v = &env->pl->global_object_listeners;
    for (int i = ivp_pvec_len(v) - 1; i >= 0; i--) {
        if (i >= ivp_pvec_len(v)) continue;
        ivp_event_object_t ev;
        ev.environment = env;
        ev.real_object = obj;
        obj_listener_call((ivp_listener_object_t *)ivp_pvec_at(v, i), kind, &ev);
    }
}

/* IVP_Cluster_Manager::fire_event_object_* */
static void fire_object_event_private(ivp_environment_t *env, ivp_object_t *obj, obj_ev_kind_t kind)
{
    ivp_env_pipeline_t *pl = env->pl;
    env_obj_cb_table_t *t = obj_cb_find_table(pl, obj);
    if (!t) return;
    for (int i = ivp_pvec_len(&t->listeners) - 1; i >= 0; i--) {
        if (i >= ivp_pvec_len(&t->listeners)) continue;
        ivp_event_object_t ev;
        ev.environment = env;
        ev.real_object = obj;
        obj_listener_call((ivp_listener_object_t *)ivp_pvec_at(&t->listeners, i), kind, &ev);
        if (i > 0 && obj_cb_find_table(pl, obj) != t) break; /* object deleted */
    }
}

/* IVP_Core::revive_simulation_core (event part) */
void ivp_env_fire_object_revived(struct ivp_environment *env, struct ivp_object *obj)
{
    if (obj->object_listener_exists) fire_object_event_private(env, obj, OBJ_EV_REVIVED);
    fire_object_event_global(env, obj, OBJ_EV_REVIVED);
    ivp_environment_emit_event_internal(env, IVP_EVENT_CORE_REVIVED, IVP_REAL(0.0), obj->dense_index, 0);
    ivp_environment_emit_event_internal(env, IVP_EVENT_OBJECT_UNFROZEN, IVP_REAL(0.0), obj->dense_index, 0);
}

/* IVP_Core::fire_event_object_frozen */
void ivp_env_fire_object_frozen(struct ivp_environment *env, struct ivp_object *obj)
{
    if (obj->object_listener_exists) fire_object_event_private(env, obj, OBJ_EV_FROZEN);
    fire_object_event_global(env, obj, OBJ_EV_FROZEN);
    ivp_environment_emit_event_internal(env, IVP_EVENT_OBJECT_FROZEN, IVP_REAL(0.0), obj->dense_index, 0);
}

/* ── deferred deletion ─────────────────────────────────────────────────── */

void ivp_env_begin_deferred_deletion(struct ivp_environment *env) { env->pl->deferred_deletion_depth++; }

bool ivp_env_object_deletion_is_deferred(struct ivp_environment *env, const struct ivp_object *obj)
{
    (void)env;
    return obj->object_deletion_deferred;
}

static void env_delete_object_now(ivp_environment_t *env, ivp_object_t *obj, bool check_vicinity);

/* The queue stays deferred while it is drained (reference c90899a): a
 * deletion callback that deletes another object appends it to the queue,
 * and one that deletes the object being deleted is a no-op.  The state is
 * per environment (the reference keeps it in process-wide statics). */
void ivp_env_end_deferred_deletion(struct ivp_environment *env)
{
    ivp_env_pipeline_t *pl = env->pl;
    if (pl->deferred_deletion_depth <= 0) {
        IVP_ASSERT(pl->deferred_deletion_depth > 0);
        return;
    }
    if (--pl->deferred_deletion_depth != 0 || pl->deferred_deletion_draining) return;
    pl->deferred_deletion_draining = true;
    while (ivp_pvec_len(&pl->deferred_deletions) > 0) {
        ivp_object_t *object = (ivp_object_t *)ivp_pvec_at(&pl->deferred_deletions, 0);
        ivp_pvec_remove_at(&pl->deferred_deletions, 0);
        bool check_vicinity = object->check_vicinity_on_deferred_deletion;
        object->object_deletion_deferred = false;
        object->check_vicinity_on_deferred_deletion = false;
        env_delete_object_now(env, object, check_vicinity);
    }
    pl->deferred_deletion_draining = false;
}

/* phantom hooks: src/dynamics/ivp_phantom.c */

/* ── IVP_Standard_Gravity_Controller ───────────────────────────────────── */

static int gravity_prio(ivp_sim_ctrl_t *c) { (void)c; return IVP_CP_GRAVITY; }
static ivp_pvec_t *gravity_assoc(ivp_sim_ctrl_t *c) { (void)c; return NULL; }
static void gravity_core_deleted(ivp_sim_ctrl_t *c, ivp_core_t *core) { (void)c; (void)core; }

static void gravity_do_simulation(ivp_sim_ctrl_t *c, ivp_sim_es_t *es, ivp_pvec_t *core_list)
{
    ivp_env_pipeline_t *pl = (ivp_env_pipeline_t *)(void *)((char *)c - offsetof(ivp_env_pipeline_t, gravity_ctrl));
    for (int i = ivp_pvec_len(core_list) - 1; i >= 0; i--) {
        ivp_core_t *my_core = (ivp_core_t *)ivp_pvec_at(core_list, i);
        if (!my_core->pinned) {
            ivp_core_global_damp_core(my_core, es->delta_time);
            ivp_core_commit_all_async_pushes_ref(my_core);
            ivp_vec3f_t sp = ivp_core_speed_f(my_core);
            lvf_add_multiple(&sp, &pl->grav_vec, (D)es->delta_time);
            ivp_core_set_speed_f(my_core, &sp);
        }
    }
}

static const ivp_sim_ctrl_vt_t gravity_vt = { gravity_prio, gravity_do_simulation, gravity_assoc, gravity_core_deleted, NULL };

/* ── libivp controller slots as sim-unit controllers ──────────────────── */

typedef struct ivp_controller_descriptor {
    ivp_controller_event_fn_t     fn;
    int                           priority;
    ivp_controller_discovery_fn_t discovery; /* NULL: attached to every core */
    bool (*on_add)(void *ctx, ivp_environment_t *env);
    void (*on_remove)(void *ctx);
    void (*reset_time)(void *ctx, ivp_time_t offset);
    void (*anchors)(void *ctx, bool link);
    bool (*parallel_safe)(const void *ctx);
} ivp_controller_descriptor_t;

static int slot_prio(ivp_sim_ctrl_t *c) { return ((ivp_controller_slot_t *)c)->priority; }

static ivp_pvec_t *slot_assoc(ivp_sim_ctrl_t *c)
{
    ivp_controller_slot_t *slot = (ivp_controller_slot_t *)c;
    return slot->mode == IVP_CONTROLLER_DEPENDENT
               ? &slot->controlled_cores
               : NULL;
}

static void slot_ctrl_detach_all(ivp_environment_t *env,
                                 ivp_controller_slot_t *slot,
                                 bool silently);

/* IVP_Controller::core_is_going_to_be_deleted_event.  A dependent controller
 * (actuators, constraints ...) is deleted with any of its cores
 * (IVP_Actuator::core_is_going_to_be_deleted_event: P_DELETE_THIS): it is
 * detached from all its cores here (the deleted core included, like the
 * reference destructor's remove_controller_from_environment) and stays
 * registered without cores until it is removed.  Other controllers just
 * forget the core. */
static void env_unlink_controller_anchors(ivp_environment_t *env, ivp_controller_handle_t h);

static void slot_core_deleted(ivp_sim_ctrl_t *c, ivp_core_t *core)
{
    ivp_controller_slot_t *slot = (ivp_controller_slot_t *)c;
    if (slot->mode == IVP_CONTROLLER_DEPENDENT &&
        ivp_pvec_index_of(&slot->controlled_cores, core) >= 0) {
        /* ~IVP_Actuator_Two_Point: the anchors leave their objects */
        env_unlink_controller_anchors(slot->env, slot->handle);
        ivp_ctrl_mgr_remove_controller_from_environment(
            slot->env, &slot->base, true);
        ivp_pvec_remove_all(&slot->controlled_cores);
        slot->deleted_with_core = true;
        return;
    }
    (void)ivp_pvec_remove_if_present(&slot->controlled_cores, core);
}

static void slot_fill_event_sim(ivp_controller_slot_t *slot,
                                ivp_sim_es_t *es,
                                ivp_event_sim_t *pub)
{
    pub->delta_time = (ivp_real_t)es->delta_time;
    pub->inverse_delta_time = (ivp_real_t)es->i_delta_time;
    pub->environment = slot->env;
    pub->sim_unit_index = 0;
    if (es->sim_unit &&
        !ivp_build_uses_reference_internal_state()) {
        pub->island_identity = (int)(
            ivp_sim_unit_stable_order_key(es->sim_unit) &
            UINT64_C(0x7fffffff));
    } else {
        pub->island_identity =
            es->sim_unit
                ? (int)((uintptr_t)es->sim_unit & 0x7fffffff)
                : -1;
    }
    pub->associated_core_handles = slot->dispatch_handles;
    pub->associated_core_count = slot->dispatch_count;
}

static void slot_do_simulation(ivp_sim_ctrl_t *c, ivp_sim_es_t *es, ivp_pvec_t *core_list)
{
    ivp_controller_slot_t *slot = (ivp_controller_slot_t *)c;
    if (!slot || !slot->enabled || !slot->event_fn) return;
    ivp_environment_t *env = slot->env;
    ivp_event_sim_t pub;
    slot_fill_event_sim(slot, es, &pub);
    ivp_object_system_t *os = env_object_system(env);
    if (slot->mode == IVP_CONTROLLER_DEPENDENT) {
        /* the first associated object with a movable core */
        ivp_object_t *dispatch_obj = NULL;
        for (int i = 0; i < slot->dispatch_count && !dispatch_obj; i++) {
            ivp_object_t *o = slot->descriptor
                                  ? slot->dispatch_objects[i]
                                  : ivp_object_system_get_by_handle(
                                        os, slot->dispatch_handles[i]);
            if (o && o->physical_core && !o->physical_core->physical_unmoveable) dispatch_obj = o;
        }
        if (dispatch_obj) slot->event_fn(&pub, dispatch_obj, slot->ctx);
        return;
    }
    for (int i = ivp_pvec_len(core_list) - 1; i >= 0; i--) {
        ivp_core_t *core = (ivp_core_t *)ivp_pvec_at(core_list, i);
        if (core->physical_unmoveable || ivp_core_n_objects(core) == 0) continue;
        /* once per core: the associated object of an independent core
         * controller on this core, else the core's first object */
        ivp_object_t *obj = NULL;
        for (int k = 0; k < slot->associated_count && !obj; k++) {
            ivp_object_t *o = ivp_object_system_get_by_handle(os, slot->associated_handles[k]);
            if (o && o->physical_core == core) obj = o;
        }
        if (!obj) obj = ivp_core_object_at(core, 0);
        ivp_object_handle_t h;
        h.slot = obj->handle.slot;
        h.generation = obj->handle.generation;
        pub.associated_core_handles = &h;
        pub.associated_core_count = 1;
        slot->event_fn(&pub, obj, slot->ctx);
    }
}

static const ivp_sim_ctrl_vt_t slot_vt = { slot_prio, slot_do_simulation, slot_assoc, slot_core_deleted, NULL };

static bool env_constraint_parallel_safe(const void *ctx)
{
    const ivp_constraint_state_t *state =
        (const ivp_constraint_state_t *)ctx;
    if (!state) return false;
    for (int i = 0; i < 6; i++) {
        if (state->core.maxforce_type[i] == IVP_CFE_BREAK) return false;
    }
    return state->force_exceed_action != IVP_CFE_BREAK;
}

static bool env_constraint_local_parallel_safe(const void *ctx)
{
    const ivp_constraint_local_state_t *state =
        (const ivp_constraint_local_state_t *)ctx;
    if (!state) return false;
    for (int i = 0; i < 6; i++) {
        if (state->axes[i].exceed_mode == IVP_CFE_BREAK ||
            state->core.maxforce_type[i] == IVP_CFE_BREAK) {
            return false;
        }
    }
    return true;
}

bool ivp_env_sim_ctrl_is_parallel_safe(ivp_sim_ctrl_t *c)
{
#ifdef IVP_DEBUG_BUILD
    /* Constraint debug drawing appends to an environment-wide vector. */
    (void)c;
    return false;
#else
    if (!c) return false;
    if (c->vt == &gravity_vt) return true;
    if (c->vt != &slot_vt) return false;

    ivp_controller_slot_t *slot = (ivp_controller_slot_t *)c;
    if (!slot->enabled || !slot->event_fn) return true;
    return slot->descriptor && slot->descriptor->parallel_safe
               ? slot->descriptor->parallel_safe(slot->ctx)
               : false;
#endif
}

/* Detach a slot from every core it is attached to. */
static void slot_ctrl_detach_all(ivp_environment_t *env,
                                 ivp_controller_slot_t *slot,
                                 bool silently)
{
    if (slot->mode == IVP_CONTROLLER_DEPENDENT) {
        ivp_ctrl_mgr_remove_controller_from_environment(
            env, &slot->base, silently);
        ivp_pvec_remove_all(&slot->controlled_cores);
    }
    ivp_object_system_t *os = env_object_system(env);
    for (int i = 0; i < ivp_object_system_count(os); i++) {
        ivp_object_t *obj = ivp_object_system_get_at(os, i);
        if (obj &&
            ivp_ctrl_mgr_core_has_controller(obj->physical_core, &slot->base)) {
            ivp_core_rem_core_controller(obj->physical_core, &slot->base);
            if (obj->physical_core->sim_unit) ivp_sim_unit_request_union_find(obj->physical_core->sim_unit);
        }
    }
    (void)ivp_pvec_remove_if_present(
        &env->pl->all_core_ctrls, &slot->base);
}

static void slot_ctrl_release_runtime(ivp_environment_t *env,
                                      ivp_controller_slot_t *slot)
{
    (void)ivp_pvec_remove_if_present(&env->pl->slot_ctrls, slot);
    ivp_pvec_clear(&slot->controlled_cores);
    if (slot->dispatch_handles) {
        ivp_free(env->allocator, slot->dispatch_handles,
                 (size_t)slot->dispatch_count *
                     sizeof(*slot->dispatch_handles));
    }
    if (slot->dispatch_objects) {
        ivp_free(env->allocator, slot->dispatch_objects,
                 (size_t)slot->dispatch_count *
                     sizeof(*slot->dispatch_objects));
    }
    slot->dispatch_handles = NULL;
    slot->dispatch_objects = NULL;
    slot->dispatch_count = 0;
}

static bool slot_ctrl_collect_attached_cores(ivp_environment_t *env,
                                             ivp_controller_slot_t *slot,
                                             ivp_pvec_t *cores)
{
    ivp_pvec_init_with_allocator(cores, env->allocator);
    ivp_object_system_t *objects = env_object_system(env);
    for (int i = 0; i < ivp_object_system_count(objects); i++) {
        ivp_object_t *object = ivp_object_system_get_at(objects, i);
        if (!object || !ivp_ctrl_mgr_core_has_controller(
                           object->physical_core, &slot->base)) {
            continue;
        }
        if (ivp_pvec_install(cores, object->physical_core) < 0) {
            ivp_pvec_clear(cores);
            return false;
        }
    }
    return true;
}

/* refresh the associated cores of a dependent controller (announce on change) */
static bool slot_ctrl_refresh_dependent(ivp_environment_t *env,
                                        ivp_controller_slot_t *slot,
                                        const ivp_pvec_t *attached_cores)
{
    if (slot->deleted_with_core) return true; /* deleted with one of its cores */
    ivp_object_handle_t handles[IVP_CONTROLLER_DISCOVERY_STACK_MAX];
    int n = ivp_controller_slot_collect_associated_handles(slot, env, handles, IVP_CONTROLLER_DISCOVERY_STACK_MAX);
    ivp_object_system_t *os = env_object_system(env);
    ivp_pvec_t cores;
    ivp_pvec_init_with_allocator(&cores, env->allocator);
    for (int i = 0; i < n; i++) {
        ivp_object_t *obj = ivp_object_system_get_by_handle(os, handles[i]);
        if (obj && ivp_pvec_install(&cores, obj->physical_core) < 0) {   /* a shared core once */
            ivp_pvec_clear(&cores);
            return false;
        }
    }
    bool same_cores =
        ivp_pvec_len(&cores) == ivp_pvec_len(&slot->controlled_cores);
    for (int i = 0; same_cores && i < ivp_pvec_len(&cores); i++) {
        if (ivp_pvec_at(&cores, i) !=
            ivp_pvec_at(&slot->controlled_cores, i)) {
            same_cores = false;
        }
    }
    bool same_handles = n == slot->dispatch_count;
    for (int i = 0; same_handles && i < n; i++) {
        if (handles[i].slot != slot->dispatch_handles[i].slot ||
            handles[i].generation != slot->dispatch_handles[i].generation) {
            same_handles = false;
        }
    }
    if (!same_cores || !same_handles || attached_cores) {
        ivp_object_handle_t *new_handles = NULL;
        ivp_object_t **new_objects = NULL;
        if (n > 0) {
            new_handles = (ivp_object_handle_t *)ivp_alloc(
                env->allocator, (size_t)n * sizeof(*new_handles));
            if (!new_handles) {
                ivp_pvec_clear(&cores);
                return false;
            }
            if (slot->descriptor) {
                new_objects = (ivp_object_t **)ivp_alloc(
                    env->allocator, (size_t)n * sizeof(*new_objects));
                if (!new_objects) {
                    ivp_free(env->allocator, new_handles,
                             (size_t)n * sizeof(*new_handles));
                    ivp_pvec_clear(&cores);
                    return false;
                }
            }
            memcpy(new_handles, handles, (size_t)n * sizeof(*new_handles));
            if (new_objects) {
                for (int i = 0; i < n; i++) {
                    new_objects[i] = ivp_object_system_get_by_handle(os, handles[i]);
                }
            }
        }

        ivp_pvec_t previous_cores = {0};
        bool cores_replaced = false;
        if (!same_cores) {
            previous_cores = slot->controlled_cores;
            slot->controlled_cores = cores;
            ivp_pvec_init_with_allocator(&cores, env->allocator);
            cores_replaced = true;
        }
        const ivp_pvec_t *previous_attached = attached_cores;
        if (!previous_attached && cores_replaced) {
            previous_attached = &previous_cores;
        }
        if (slot->mode == IVP_CONTROLLER_DEPENDENT && previous_attached &&
            !ivp_ctrl_mgr_replace_controller_cores(
                env, &slot->base, previous_attached)) {
            if (cores_replaced) {
                ivp_pvec_clear(&slot->controlled_cores);
                slot->controlled_cores = previous_cores;
            }
            if (new_objects) {
                ivp_free(env->allocator, new_objects,
                         (size_t)n * sizeof(*new_objects));
            }
            if (new_handles) {
                ivp_free(env->allocator, new_handles,
                         (size_t)n * sizeof(*new_handles));
            }
            ivp_pvec_clear(&cores);
            return false;
        }
        if (cores_replaced) {
            ivp_pvec_clear(&previous_cores);
        }
        if (slot->dispatch_handles) {
            ivp_free(env->allocator, slot->dispatch_handles,
                     (size_t)slot->dispatch_count *
                         sizeof(*slot->dispatch_handles));
        }
        if (slot->dispatch_objects) {
            ivp_free(env->allocator, slot->dispatch_objects,
                     (size_t)slot->dispatch_count *
                         sizeof(*slot->dispatch_objects));
        }
        slot->dispatch_handles = new_handles;
        slot->dispatch_objects = new_objects;
        slot->dispatch_count = n;
    }
    ivp_pvec_clear(&cores);
    return true;
}

static const ivp_controller_descriptor_t *env_find_builtin_controller(
    ivp_controller_event_fn_t event_fn);

/* Dependent user controllers follow the cores of the objects their
 * discovery function reports.  Built-in controllers keep the cores they
 * were announced with, like the reference (IVP_Actuator::
 * actuator_controlled_cores is set in the constructor): an object that
 * moves to another core (detach_object) keeps its actuators on the old
 * core, and a deleted core deletes them (slot_core_deleted). */
static void env_refresh_controller_associations(ivp_environment_t *env)
{
    ivp_controller_system_t *cs = env_controller_system(env);
    for (int i = 0; i < ivp_controller_system_count(cs); i++) {
        ivp_controller_slot_t *slot = ivp_controller_system_get_at(cs, i);
        if (!slot || slot->mode != IVP_CONTROLLER_DEPENDENT) continue;
        if (slot->descriptor) continue;
        (void)slot_ctrl_refresh_dependent(env, slot, NULL);
    }
}

/* attach libivp "all cores" controllers to a new core */
bool ivp_env_attach_default_controllers(ivp_environment_t *env, ivp_core_t *core)
{
    ivp_pvec_t *l = &env->pl->all_core_ctrls;
    for (int i = 0; i < ivp_pvec_len(l); i++) {
        if (!ivp_ctrl_mgr_add_controller_to_core(
                (ivp_sim_ctrl_t *)ivp_pvec_at(l, i), core)) {
            for (int k = i - 1; k >= 0; k--) {
                ivp_ctrl_mgr_remove_controller_from_core(
                    (ivp_sim_ctrl_t *)ivp_pvec_at(l, k), core);
            }
            return false;
        }
    }
    return true;
}

static bool slot_ctrl_set_all_cores(ivp_environment_t *env,
                                    ivp_controller_slot_t *slot)
{
    if (slot->run_for_all_cores) return true;
    if (ivp_pvec_add(&env->pl->all_core_ctrls, &slot->base) < 0) {
        return false;
    }
    slot->run_for_all_cores = true;
    ivp_object_system_t *os = env_object_system(env);
    for (int i = 0; i < ivp_object_system_count(os); i++) {
        ivp_object_t *obj = ivp_object_system_get_at(os, i);
        if (obj && !ivp_ctrl_mgr_add_controller_to_core(
                       &slot->base, obj->physical_core)) {
            slot_ctrl_detach_all(env, slot, true);
            slot->run_for_all_cores = false;
            return false;
        }
    }
    return true;
}

/* ── controller API ────────────────────────────────────────────────────── */

static ivp_controller_slot_t *env_get_controller_slot(ivp_environment_t *env, ivp_controller_handle_t h)
{
    if (!env || !ivp_controller_handle_is_valid(h)) return NULL;
    return ivp_controller_system_get_by_handle(env_controller_system(env), h);
}

/* Built-in descriptors provide the reference priority and lifecycle hooks.
 * Function lookup is confined to registration; live slots retain the
 * descriptor explicitly. */

/* IVP_Anchor::init_anchor (insert_anchor) while the actuator is registered,
 * remove_anchor when it is removed or deleted with a core */
static void env_link_anchor(ivp_anchor_t *a, ivp_vec3r_t *core_pos, bool link)
{
    if (link) {
        ivp_anchor_link(a, core_pos);
    } else {
        ivp_anchor_unlink(a);
    }
}
static void env_force_anchors(void *ctx, bool link)
{
    ivp_force_actuator_state_t *st = (ivp_force_actuator_state_t *)ctx;
    env_link_anchor(&st->anchors[0], &st->anchor_a_os, link);
    env_link_anchor(&st->anchors[1], &st->anchor_b_os, link);
}
static void env_rot_mot_anchors(void *ctx, bool link)
{
    ivp_rot_mot_state_t *st = (ivp_rot_mot_state_t *)ctx;
    env_link_anchor(&st->anchors[0], &st->anchor_a_os, link);
    env_link_anchor(&st->anchors[1], &st->anchor_b_os, link);
}
static void env_extra_anchors(void *ctx, bool link)
{
    ivp_actuator_extra_state_t *st = (ivp_actuator_extra_state_t *)ctx;
    env_link_anchor(&st->anchors[0], &st->anchor_a_os, link);
    env_link_anchor(&st->anchors[1], &st->anchor_b_os, link);
}
static void env_stabilizer_anchors(void *ctx, bool link)
{
    ivp_stabilizer_state_t *st = (ivp_stabilizer_state_t *)ctx;
    for (int i = 0; i < 4; i++) env_link_anchor(&st->anchors[i], &st->anchor_os[i], link);
}
static void env_spring_anchors(void *ctx, bool link)
{
    ivp_spring_state_t *st = (ivp_spring_state_t *)ctx;
    env_link_anchor(&st->anchors[0], &st->anchor_a_os, link);
    env_link_anchor(&st->anchors[1], &st->anchor_b_os, link);
}
static void env_suspension_anchors(void *ctx, bool link)
{
    ivp_suspension_state_t *st = (ivp_suspension_state_t *)ctx;
    env_link_anchor(&st->anchors[0], &st->anchor_a_os, link);
    env_link_anchor(&st->anchors[1], &st->anchor_b_os, link);
}
static void env_stiff_spring_anchors(void *ctx, bool link)
{
    ivp_stiff_spring_state_t *st = (ivp_stiff_spring_state_t *)ctx;
    env_link_anchor(&st->anchors[0], &st->anchor_a_os, link);
    env_link_anchor(&st->anchors[1], &st->anchor_b_os, link);
}

static bool env_check_dist_on_add(void *ctx, ivp_environment_t *env)
{
    return ivp_check_dist_attach((ivp_check_dist_state_t *)ctx, env);
}
static void env_check_dist_on_remove(void *ctx)
{
    ivp_check_dist_detach((ivp_check_dist_state_t *)ctx);
}
/* the reference destructors drop the active value dependencies */
static void env_force_on_remove(void *ctx) { ivp_force_actuator_unbind_active((ivp_force_actuator_state_t *)ctx); }
static void env_torque_on_remove(void *ctx) { ivp_torque_actuator_unbind_active((ivp_torque_actuator_state_t *)ctx); }
static void env_rot_mot_on_remove(void *ctx) { ivp_rot_mot_unbind_active((ivp_rot_mot_state_t *)ctx); }
static void env_spring_on_remove(void *ctx) { ivp_spring_unbind_active((ivp_spring_state_t *)ctx); }
static void env_stiff_spring_on_remove(void *ctx) { ivp_stiff_spring_unbind_active((ivp_stiff_spring_state_t *)ctx); }
static void env_extra_on_remove(void *ctx) { ivp_actuator_extra_destroy((ivp_actuator_extra_state_t *)ctx); }
static void env_golem_reset_time(void *ctx, ivp_time_t offset) { ivp_golem_reset_time((ivp_golem_state_t *)ctx, offset); }

static const ivp_controller_descriptor_t *env_find_builtin_controller(
    ivp_controller_event_fn_t event_fn)
{
    static const ivp_controller_descriptor_t table[] = {
        { ivp_controller_forcefield, IVP_CP_ACTUATOR, NULL, NULL, NULL, NULL, NULL, NULL },
        { ivp_controller_buoyancy, IVP_CP_FORCEFIELDS, NULL, NULL, NULL, NULL, NULL, NULL },
        { ivp_controller_force_actuator, IVP_CP_ACTUATOR, ivp_force_actuator_discovery, NULL, env_force_on_remove, NULL, env_force_anchors, NULL },
        { ivp_controller_torque_actuator, IVP_CP_ACTUATOR, ivp_torque_actuator_discovery, NULL, env_torque_on_remove, NULL, NULL, NULL },
        { ivp_controller_rot_mot, IVP_CP_ACTUATOR, ivp_rot_mot_discovery, NULL, env_rot_mot_on_remove, NULL, env_rot_mot_anchors, NULL },
        { ivp_controller_actuator_extra, IVP_CP_ACTUATOR, ivp_actuator_extra_discovery, NULL, env_extra_on_remove, NULL, env_extra_anchors, NULL },
        { ivp_controller_check_dist, IVP_CP_ACTUATOR, NULL, env_check_dist_on_add, env_check_dist_on_remove, NULL, NULL, NULL },
        { ivp_controller_stabilizer, IVP_CP_ACTUATOR, ivp_stabilizer_discovery, NULL, NULL, NULL, env_stabilizer_anchors, NULL },
        { ivp_controller_spring, IVP_CP_ACTUATOR, ivp_spring_discovery, NULL, env_spring_on_remove, NULL, env_spring_anchors, NULL }, /* IVP_Actuator::get_controller_priority */
        { ivp_controller_suspension, IVP_CP_ACTUATOR, ivp_suspension_discovery, NULL, NULL, NULL, env_suspension_anchors, NULL },
        { ivp_controller_stiff_spring, IVP_CP_STIFF_SPRINGS, ivp_stiff_spring_discovery, NULL, env_stiff_spring_on_remove, NULL, env_stiff_spring_anchors, NULL },
        { ivp_controller_floating, IVP_CP_FLOATING, NULL, NULL, NULL, NULL, NULL, NULL },
        { ivp_controller_motion, IVP_CP_MOTION, NULL, NULL, NULL, NULL, NULL, NULL },
        { ivp_controller_golem, IVP_CP_MOTION, NULL, NULL, NULL, env_golem_reset_time, NULL, NULL },
        { ivp_controller_constraint, IVP_CP_CONSTRAINTS, ivp_constraint_discovery, NULL, NULL, NULL, NULL, env_constraint_parallel_safe },
        { ivp_controller_constraint_local, IVP_CP_CONSTRAINTS, ivp_constraint_local_discovery, NULL, NULL, NULL, NULL, env_constraint_local_parallel_safe },
        { ivp_controller_fixed_keyed, IVP_CP_CONSTRAINTS, ivp_fixed_keyed_discovery, NULL, NULL, NULL, NULL, NULL },
        { ivp_controller_car_system, IVP_CP_CONSTRAINTS, NULL, NULL, NULL, NULL, NULL, NULL }, /* ivp_constraint_car.hxx:88 */
        { ivp_controller_world_friction, IVP_CP_CONSTRAINTS_MAX, NULL, NULL, NULL, NULL, NULL, NULL },
        { ivp_controller_raycast_car, IVP_CP_CONSTRAINTS_MAX, ivp_car_discovery, NULL, NULL, NULL, NULL, NULL },
        { ivp_controller_fake_jetski, IVP_CP_CONSTRAINTS_MAX, NULL, NULL, NULL, NULL, NULL, NULL },
        { ivp_controller_airboat, IVP_CP_CONSTRAINTS_MAX, NULL, NULL, NULL, NULL, NULL, NULL },
        { ivp_controller_phantom, IVP_CP_CONSTRAINTS_MAX, NULL, NULL, NULL, NULL, NULL, NULL },
    };
    if (!event_fn) return NULL;
    for (size_t i = 0; i < sizeof(table) / sizeof(table[0]); i++) {
        if (table[i].fn == event_fn) return &table[i];
    }
    return NULL;
}

static void env_unlink_controller_anchors(ivp_environment_t *env, ivp_controller_handle_t h)
{
    ivp_controller_slot_t *slot = env_get_controller_slot(env, h);
    if (!slot) return;
    const ivp_controller_descriptor_t *b = slot->descriptor;
    if (b && b->anchors) b->anchors(slot->ctx, false);
}

static ivp_controller_handle_t env_add_controller_internal(ivp_environment_t *env, int priority,
                                                           ivp_controller_event_fn_t event_fn, void *ctx,
                                                           ivp_controller_mode_t mode,
                                                           ivp_controller_discovery_fn_t discovery_fn,
                                                           bool attach_all_cores,
                                                           const ivp_controller_descriptor_t *descriptor)
{
    ivp_controller_handle_t invalid = ivp_controller_handle_invalid();
    if (!env || !event_fn) return invalid;
    const int normalized_priority = priority; /* IVP_Controller::get_priority() */
    ivp_controller_handle_t out = ivp_controller_system_add(
        env_controller_system(env), normalized_priority, event_fn, ctx);
    if (!ivp_controller_handle_is_valid(out)) return invalid;
    ivp_controller_slot_t *slot = env_get_controller_slot(env, out);
    if (!slot) {
        (void)ivp_controller_system_remove(env_controller_system(env), out);
        return invalid;
    }
    slot->mode = mode;
    slot->run_for_all_cores = false;
    slot->discovery_fn = discovery_fn;
    slot->env = env;
    slot->descriptor = descriptor;
    slot->base.vt = &slot_vt;
    ivp_env_register_sim_controller(env, &slot->base);
    ivp_pvec_init_with_allocator(&slot->controlled_cores, env->allocator);
    if (ivp_pvec_add(&env->pl->slot_ctrls, slot) < 0) {
        slot_ctrl_release_runtime(env, slot);
        (void)ivp_controller_system_remove(env_controller_system(env), out);
        return invalid;
    }
    if (slot->mode == IVP_CONTROLLER_DEPENDENT) {
        if (!slot_ctrl_refresh_dependent(env, slot, NULL)) {
            slot_ctrl_release_runtime(env, slot);
            (void)ivp_controller_system_remove(env_controller_system(env), out);
            return invalid;
        }
    } else if (attach_all_cores && !slot_ctrl_set_all_cores(env, slot)) {
        slot_ctrl_release_runtime(env, slot);
        (void)ivp_controller_system_remove(env_controller_system(env), out);
        return invalid;
    }
    ivp_controller_system_sort(
        env_controller_system(env),
        ivp_build_uses_reference_internal_state());
    ivp_environment_emit_event_internal(env, IVP_EVENT_CONTROLLER_ADDED, IVP_REAL(0.0), -1, (int)out.slot);
    return out;
}

ivp_controller_handle_t ivp_environment_add_controller(ivp_environment_t *env, int priority, ivp_controller_fn_t fn, void *ctx)
{
    if (!env || !fn) return ivp_controller_handle_invalid();
    const ivp_controller_descriptor_t *b = env_find_builtin_controller(fn);
    if (!b) {
        /* user controller (IVP_Controller subclass): its own priority,
         * attached to every core */
        return env_add_controller_internal(env, priority, fn, ctx,
                                           IVP_CONTROLLER_INDEPENDENT, NULL,
                                           true, NULL);
    }
    ivp_controller_handle_t h;
    if (b->discovery) {
        /* IVP_Controller_Dependent: announce_controller_to_environment */
        h = env_add_controller_internal(env, b->priority, fn, ctx,
                                        IVP_CONTROLLER_DEPENDENT, b->discovery,
                                        false, b);
    } else {
        h = env_add_controller_internal(env, b->priority, fn, ctx,
                                        IVP_CONTROLLER_INDEPENDENT, NULL,
                                        true, b);
    }
    if (ivp_controller_handle_is_valid(h)) {
        if (b->anchors) b->anchors(ctx, true);
        if (b->on_add && !b->on_add(ctx, env)) {
            (void)ivp_environment_remove_dependent_controller(env, h, true);
            return ivp_controller_handle_invalid();
        }
    }
    return h;
}

#ifdef IVP_ENABLE_TEST_CUSTOM_CONTROLLER_API
ivp_controller_handle_t ivp_test_environment_add_custom_controller(ivp_environment_t *env, int priority,
                                                                   ivp_controller_fn_t fn, void *ctx)
{
    return env_add_controller_internal(env, priority, fn, ctx,
                                       IVP_CONTROLLER_INDEPENDENT, NULL,
                                       true, NULL);
}

ivp_controller_handle_t ivp_test_environment_announce_custom_dependent_controller(
    ivp_environment_t *env, int priority, ivp_controller_fn_t fn, void *ctx, ivp_controller_discovery_fn_t discovery_fn)
{
    if (!discovery_fn) return ivp_controller_handle_invalid();
    return env_add_controller_internal(env, priority, fn, ctx, IVP_CONTROLLER_DEPENDENT,
                                       discovery_fn, false, NULL);
}
#endif

ivp_controller_handle_t ivp_environment_announce_dependent_controller(ivp_environment_t *env, int priority,
                                                                      ivp_controller_fn_t fn, void *ctx,
                                                                      ivp_controller_discovery_fn_t discovery_fn)
{
    if (!env || !fn || !discovery_fn) return ivp_controller_handle_invalid();
    const ivp_controller_descriptor_t *b = env_find_builtin_controller(fn);
    ivp_controller_handle_t h =
        env_add_controller_internal(env, b ? b->priority : priority, fn, ctx,
                                    IVP_CONTROLLER_DEPENDENT, discovery_fn,
                                    false, b);
    if (ivp_controller_handle_is_valid(h) && b && b->anchors) b->anchors(ctx, true);
    return h;
}

bool ivp_environment_remove_dependent_controller(ivp_environment_t *env, ivp_controller_handle_t controller_handle,
                                                 bool silently)
{
    if (!env || !ivp_controller_handle_is_valid(controller_handle)) return false;
    ivp_controller_slot_t *slot = env_get_controller_slot(env, controller_handle);
    if (!slot) return false;
    const ivp_controller_descriptor_t *descriptor = slot->descriptor;
    if (descriptor && descriptor->anchors) {
        descriptor->anchors(slot->ctx, false);
    }
    if (descriptor && descriptor->on_remove) {
        descriptor->on_remove(slot->ctx);
    }
    slot_ctrl_detach_all(env, slot, silently);
    slot_ctrl_release_runtime(env, slot);
    if (!ivp_controller_system_remove(env_controller_system(env), controller_handle)) return false;
    if (!silently) {
        ivp_environment_emit_event_internal(env, IVP_EVENT_CONTROLLER_REMOVED, IVP_REAL(0.0), -1, (int)controller_handle.slot);
    }
    return true;
}

bool ivp_environment_remove_controller(ivp_environment_t *env, ivp_controller_handle_t controller_handle)
{
    return ivp_environment_remove_dependent_controller(env, controller_handle, false);
}

bool ivp_environment_set_controller_discovery(ivp_environment_t *env, ivp_controller_handle_t controller_handle,
                                              ivp_controller_discovery_fn_t discovery_fn)
{
    if (!env || !discovery_fn) return false;
    ivp_controller_slot_t *slot = env_get_controller_slot(env, controller_handle);
    if (!slot) return false;
    ivp_controller_mode_t old_mode = slot->mode;
    ivp_controller_discovery_fn_t old_discovery_fn = slot->discovery_fn;
    if (slot->mode != IVP_CONTROLLER_DEPENDENT) {
        ivp_pvec_t attached_cores;
        if (!slot_ctrl_collect_attached_cores(env, slot, &attached_cores)) {
            return false;
        }
        slot->discovery_fn = discovery_fn;
        slot->mode = IVP_CONTROLLER_DEPENDENT;
        if (!slot_ctrl_refresh_dependent(
                env, slot, &attached_cores)) {
            slot->mode = old_mode;
            slot->discovery_fn = old_discovery_fn;
            ivp_pvec_clear(&attached_cores);
            return false;
        }
        ivp_pvec_clear(&attached_cores);
        (void)ivp_pvec_remove_if_present(
            &env->pl->all_core_ctrls, &slot->base);
    } else {
        slot->discovery_fn = discovery_fn;
        if (!slot_ctrl_refresh_dependent(env, slot, NULL)) {
            slot->discovery_fn = old_discovery_fn;
            return false;
        }
    }
    slot->mode = IVP_CONTROLLER_DEPENDENT;
    slot->run_for_all_cores = false;
    return true;
}

ivp_controller_handle_t ivp_environment_create_core_controller(ivp_environment_t *env, int priority,
                                                               ivp_controller_fn_t fn, void *ctx)
{
    if (!env || !fn) return ivp_controller_handle_invalid();
    const ivp_controller_descriptor_t *b = env_find_builtin_controller(fn);
    /* IVP_Controller_Independent attached to no core yet */
    return env_add_controller_internal(env, b ? b->priority : priority, fn, ctx,
                                       IVP_CONTROLLER_INDEPENDENT, NULL,
                                       false, b);
}

bool ivp_environment_add_controller_to_core(ivp_environment_t *env, ivp_controller_handle_t controller_handle,
                                            ivp_object_t *object)
{
    if (!env || !object || object->owner != env) return false;
    ivp_controller_slot_t *slot = env_get_controller_slot(env, controller_handle);
    if (!slot) return false;
    const ivp_object_handle_t object_handle = ivp_environment_get_object_handle(object);
    bool handle_was_present = false;
    for (int i = 0; i < slot->associated_count; i++) {
        if (slot->associated_handles[i].slot == object_handle.slot &&
            slot->associated_handles[i].generation == object_handle.generation) {
            handle_was_present = true;
            break;
        }
    }
    if (!ivp_controller_slot_add_associated_handle(slot, object_handle, env->allocator)) return false;

    const bool was_dependent = slot->mode == IVP_CONTROLLER_DEPENDENT;
    const bool was_all_cores = slot->run_for_all_cores;
    if (was_dependent || was_all_cores) {
        ivp_pvec_t attached_cores;
        if (!slot_ctrl_collect_attached_cores(
                env, slot, &attached_cores)) {
            if (!handle_was_present) {
                (void)ivp_controller_slot_remove_associated_handle(
                    slot, object_handle);
            }
            return false;
        }

        ivp_pvec_t target_core;
        ivp_pvec_init_with_allocator(&target_core, env->allocator);
        if (ivp_pvec_add(&target_core, object->physical_core) < 0) {
            ivp_pvec_clear(&attached_cores);
            if (!handle_was_present) {
                (void)ivp_controller_slot_remove_associated_handle(
                    slot, object_handle);
            }
            return false;
        }

        const ivp_controller_mode_t old_mode = slot->mode;
        ivp_pvec_t old_controlled_cores = slot->controlled_cores;
        slot->mode = IVP_CONTROLLER_DEPENDENT;
        slot->controlled_cores = target_core;
        if (!ivp_ctrl_mgr_replace_controller_cores(
                env, &slot->base, &attached_cores)) {
            ivp_pvec_clear(&slot->controlled_cores);
            slot->controlled_cores = old_controlled_cores;
            slot->mode = old_mode;
            ivp_pvec_clear(&attached_cores);
            if (!handle_was_present) {
                (void)ivp_controller_slot_remove_associated_handle(
                    slot, object_handle);
            }
            return false;
        }

        ivp_pvec_clear(&attached_cores);
        ivp_pvec_clear(&old_controlled_cores);
        (void)ivp_pvec_remove_if_present(
            &env->pl->all_core_ctrls, &slot->base);
        slot->mode = IVP_CONTROLLER_INDEPENDENT;
        slot->run_for_all_cores = false;
        slot->discovery_fn = NULL;
        ivp_pvec_remove_all(&slot->controlled_cores);
        return true;
    }

    if (!ivp_ctrl_mgr_add_controller_to_core(
            &slot->base, object->physical_core)) {
        if (!handle_was_present) {
            (void)ivp_controller_slot_remove_associated_handle(slot, object_handle);
        }
        return false;
    }
    return true;
}

bool ivp_environment_remove_controller_from_core(ivp_environment_t *env, ivp_controller_handle_t controller_handle,
                                                 ivp_object_t *object)
{
    if (!env || !object || object->owner != env) return false;
    ivp_controller_slot_t *slot = env_get_controller_slot(env, controller_handle);
    if (!slot) return false;
    if (!ivp_controller_slot_remove_associated_handle(
            slot, ivp_environment_get_object_handle(object))) return false;
    bool core_still_associated = false;
    ivp_object_system_t *os = env_object_system(env);
    for (int i = 0; i < slot->associated_count; i++) {
        ivp_object_t *remaining = ivp_object_system_get_by_handle(
            os, slot->associated_handles[i]);
        if (remaining && remaining->physical_core == object->physical_core) {
            core_still_associated = true;
            break;
        }
    }
    if (!core_still_associated) {
        ivp_ctrl_mgr_remove_controller_from_core(
            &slot->base, object->physical_core);
    }
    return true;
}

bool ivp_environment_ensure_core_in_simulation(ivp_environment_t *env, ivp_object_t *object)
{
    if (!env || !object || object->owner != env) return false;
    if (object->physical_core->physical_unmoveable) return false;
    if (object->object_movement_state == IVP_MT_NOT_SIM) {
        return ivp_env_add_revive_core(env, object->friction_core);
    }
    ivp_object_ensure_in_simulation(object);
    return true;
}

/* IVP_Actuator_Two_Point::ensure_actuator_in_simulation ->
 * IVP_Controller_Manager::ensure_controller_in_simulation over the
 * actuator_controlled_cores (the movable anchor cores, anchor order): the
 * sim unit of the controlled cores is revived at once (or its cores' freeze
 * check values are reset); without any sim unit the first movable core is
 * revived at the next PSI (ensure_core_in_simulation_delayed).  The reference
 * stops after the first sim unit, which holds all controlled cores of an
 * announced actuator; a libivp actuator state used without a registered
 * controller can leave its cores in separate sim units, so every distinct
 * sim unit is handled (a no-op difference for a shared unit). */
void ivp_environment_ensure_actuator_in_simulation(ivp_environment_t *env, ivp_object_t *const *objects, int count)
{
    if (!env || !objects) return;
    ivp_core_t *fallback_core = NULL;
    struct ivp_sim_unit *done[8];
    int n_done = 0;
    for (int i = 0; i < count; i++) {
        ivp_object_t *obj = objects[i];
        if (!obj || obj->owner != env) continue;
        ivp_core_t *core = obj->physical_core;
        if (core->physical_unmoveable) continue;
        if (!fallback_core) fallback_core = core;
        struct ivp_sim_unit *su = core->sim_unit;
        if (!su) continue;
        bool seen = false;
        for (int k = 0; k < n_done; k++) seen = seen || done[k] == su;
        if (seen) continue;
        if (n_done < (int)(sizeof(done) / sizeof(done[0]))) done[n_done++] = su;
        ivp_sim_unit_ensure_in_simulation(su);
    }
    if (n_done == 0 && fallback_core) ivp_core_ensure_core_in_simulation_delayed(fallback_core);
}

bool ivp_environment_ensure_controller_in_simulation(ivp_environment_t *env, ivp_controller_handle_t controller_handle)
{
    ivp_controller_slot_t *slot = env_get_controller_slot(env, controller_handle);
    if (!slot) return false;
    if (slot->mode == IVP_CONTROLLER_DEPENDENT) {
        ivp_ctrl_mgr_ensure_controller_in_simulation(env, &slot->base);
        return ivp_pvec_len(&slot->controlled_cores) > 0;
    }
    bool woke_any = false;
    ivp_object_system_t *os = env_object_system(env);
    for (int i = 0; i < ivp_object_system_count(os); i++) {
        ivp_object_t *obj = ivp_object_system_get_at(os, i);
        if (!obj || !ivp_ctrl_mgr_core_has_controller(
                        obj->physical_core, &slot->base)) {
            continue;
        }
        if (ivp_environment_ensure_core_in_simulation(env, obj)) woke_any = true;
    }
    return woke_any;
}

void ivp_environment_clear_controllers(ivp_environment_t *env)
{
    if (!env) return;
    ivp_controller_system_t *cs = env_controller_system(env);
    while (ivp_controller_system_count(cs) > 0) {
        ivp_controller_slot_t *slot = ivp_controller_system_get_at(cs, ivp_controller_system_count(cs) - 1);
        ivp_controller_handle_t h;
        h.slot = slot->handle.slot;
        h.generation = slot->handle.generation;
        if (!ivp_environment_remove_dependent_controller(env, h, true)) break;
    }
}

/* ── event listener API ────────────────────────────────────────────────── */

ivp_event_listener_handle_t ivp_environment_add_event_listener(ivp_environment_t *env, ivp_event_listener_fn_t fn, void *ctx)
{
    if (!env || !fn) return ivp_event_listener_handle_invalid();
    return ivp_event_system_add_listener(env_event_system(env), fn, ctx);
}

bool ivp_environment_remove_event_listener(ivp_environment_t *env, ivp_event_listener_handle_t listener_handle)
{
    if (!env || !ivp_event_listener_handle_is_valid(listener_handle)) return false;
    return ivp_event_system_remove_listener(env_event_system(env), listener_handle);
}

void ivp_environment_clear_event_listeners(ivp_environment_t *env)
{
    if (!env) return;
    ivp_event_system_clear(env_event_system(env));
}

void ivp_environment_set_sim_unit_count(ivp_environment_t *env, int sim_unit_count)
{
    if (!env) return;
    env->sim_unit_count = sim_unit_count > 0 ? sim_unit_count : 1;
}

int ivp_environment_get_sim_unit_count(const ivp_environment_t *env) { return env ? env->sim_unit_count : 1; }
int ivp_environment_get_current_sim_unit_index(const ivp_environment_t *env) { return env ? env->current_sim_unit_index : -1; }

/* ── configuration ─────────────────────────────────────────────────────── */

void ivp_environment_config_init(ivp_environment_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    /* IVP_Environment constructor: IVP_U_Point temp_grav(0.0f, 9.83f, 0.0f) */
    ivp_vec3r_set(&cfg->gravity, IVP_REAL(0.0), (ivp_real_t)9.83f, IVP_REAL(0.0));
    cfg->delta_psi_time = (ivp_real_t)(1.0f / 66.0f);
    cfg->freeze_params = ivp_freeze_params_default();
    cfg->allocator = NULL;
    cfg->initial_object_capacity = IVP_OBJECTS_INIT_CAPACITY;
    cfg->initial_controller_capacity = IVP_CONTROLLERS_INIT_CAPACITY;
    cfg->initial_event_listener_capacity = IVP_EVENT_LISTENERS_INIT_CAPACITY;
    cfg->sim_unit_count = IVP_SIM_UNIT_COUNT_DEFAULT;
    cfg->parallel_worker_count = IVP_PARALLEL_WORKER_COUNT_DEFAULT;
    ivp_anomaly_config_init(&cfg->anomaly_config);
    cfg->collision_filter = ivp_collision_filter_allow_all();
    cfg->ccd_enabled = false;
    cfg->n_cache_objects = 256;
    cfg->disable_performance_counter = false;
}

/* IVP_Environment::set_gravity */
void ivp_environment_set_gravity(ivp_environment_t *env, const ivp_vec3r_t *g)
{
    env->gravity = *g;
    env->pl->gravity_scalar = (double)lvp_real_length(g);
    lvf_set_p(&env->pl->grav_vec, g);
}

/* IVP_Environment::set_delta_PSI_time */
static bool env_set_delta_psi_time_d(ivp_environment_t *env, double psi_time)
{
    if (!(psi_time >= (double)IVP_MIN_DELTA_PSI_TIME && psi_time <= (double)IVP_MAX_DELTA_PSI_TIME)) return false;
    ivp_env_pipeline_t *pl = env->pl;
    pl->delta_psi_time = psi_time;
    pl->inv_delta_psi_time = 1.0f / pl->delta_psi_time;
    pl->integrated_energy_damp = exp((double)(float)log(0.9f) * pl->delta_psi_time);
    env->delta_psi_time = (ivp_real_t)psi_time;
    return true;
}

void ivp_environment_set_delta_psi_time_internal(struct ivp_environment *env, double psi_time)
{
    if (!env_set_delta_psi_time_d(env, psi_time)) {
        /* out of range: nothing changed */
        IVP_ASSERT(psi_time >= (double)IVP_MIN_DELTA_PSI_TIME);
        IVP_ASSERT(psi_time <= (double)IVP_MAX_DELTA_PSI_TIME);
    }
}

void ivp_environment_set_delta_psi_time(ivp_environment_t *env, ivp_real_t dt)
{
    if (!env) return;
    ivp_environment_set_delta_psi_time_internal(env, (double)dt);
}

void ivp_environment_set_global_collision_tolerance(ivp_environment_t *env, ivp_real_t tolerance)
{
    if (!env) return;
    ivp_mindist_settings_set_collision_tolerance(&env->pl->mindist_settings,
                                                  (double)tolerance);
}

/* ── PSI (IVP_Environment::simulate_psi + IVP_Time_Event_PSI) ─────────── */

static void env_restore_contact_order(ivp_environment_t *env)
{
    if (ivp_build_uses_reference_internal_state()) return;
    ivp_object_system_t *objects = env_object_system(env);
    for (int i = 0; i < ivp_object_system_count(objects); i++) {
        ivp_friction_mgr_restore_object_contact_order(
            ivp_object_system_get_at(objects, i));
    }
}

static void env_simulate_psi(ivp_environment_t *env)
{
    ivp_env_pipeline_t *pl = env->pl;
    const ivp_real_t dt = (ivp_real_t)pl->delta_psi_time;
    IVP_IF(1 || env->debug_manager->display_statistic)
    {
        /* get_current_time() - last_statistic_output: IVP_Time::operator-
         * rounds the difference to float */
        if ((double)(float)(env->current_time - pl->statistic_manager.last_statistic_output) >= 1.0) {
            ivp_statistic_manager_output(&pl->statistic_manager);
            pl->statistic_manager.last_statistic_output = env->current_time;
        }
    }
    IVP_IF(1)   /* ivp_environment.cxx:1050: debug builds only */
    {
        if (env->debug_manager) env->debug_manager->psi_counter += IVP_REAL(1.0);
        ivp_env_delete_draw_vector_debug(env);
    }
    ivp_env_pcount(env, IVP_PE_PSI_UNIVERSE);
    pl->in_psi = true;
#if IVP_EDIT_LOCALITY
    pl->psi_index++;
#endif
    env->current_sim_unit_index = 0;
    pl->state = IVP_ES_PSI;
    env_restore_contact_order(env);
    env_emit_phase(env, IVP_PSI_PHASE_PRE_PSI, dt);
    ivp_anomaly_manager_begin_psi(&env->anomaly_manager);
    env_emit_phase(env, IVP_PSI_PHASE_REVIVE, dt);
    if (ivp_pvec_len(&pl->core_revive_list)) env_revive_cores_psi(env);
    if (env->universe_manager) env_check_for_unused_objects(env);
    env_emit_phase(env, IVP_PSI_PHASE_MANAGER_ACTIVE_VALUES, dt);
    /* l_active_value_manager->refresh_psi_active_values(current_time) */
    if (pl->active_value_manager) ivp_active_value_manager_refresh_psi(pl->active_value_manager, env->current_time);
    /* fire_event_PSI */
    env_emit_phase(env, IVP_PSI_PHASE_SIM_UNIT_BEGIN, dt);
    ivp_environment_emit_event_internal(env, IVP_EVENT_SIM_UNIT_BEGIN, dt, -1, 0);
    ivp_mindist_mgr_recalc_all_exact_wheel_mindist(&pl->mindist_mgr);
    ivp_env_pcount(env, IVP_PE_PSI_CONTROLLERS);
    env_emit_phase(env, IVP_PSI_PHASE_MANAGER_CONSTRAINTS, dt);
    env_refresh_controller_associations(env);
    env_emit_phase(env, IVP_PSI_PHASE_PRE_CONTROLLERS, dt);
    env_emit_phase(env, IVP_PSI_PHASE_CONTROLLERS, dt);
    ivp_pvec_remove_all(&pl->touched_cores);
    ivp_sim_units_mgr_simulate_sim_units_psi(&pl->sim_units_manager, &pl->touched_cores);
    env_emit_phase(env, IVP_PSI_PHASE_INTEGRATE, dt);
    ivp_pvec_remove_all(&pl->active_hulls);
    ivp_env_pcount(env, IVP_PE_PSI_INTEGRATORS);
    ivp_cnps_commit_all_calc_next_psi_matrix(env, &pl->touched_cores, &pl->active_hulls);
    env_emit_phase(env, IVP_PSI_PHASE_COLLISION_BROADPHASE, dt);
    ivp_env_pcount(env, IVP_PE_PSI_HULL);
    pl->state = IVP_ES_PSI_HULL;
    ivp_cnps_commit_all_hull_managers(env, &pl->active_hulls);
    env_emit_phase(env, IVP_PSI_PHASE_COLLISION_NARROWPHASE, dt);
    ivp_env_pcount(env, IVP_PE_PSI_SHORT_MINDISTS);
    pl->state = IVP_ES_PSI_SHORT;
    ivp_mindist_mgr_recalc_all_exact_mindists(&pl->mindist_mgr);
    env_emit_phase(env, IVP_PSI_PHASE_COLLISION_SOLVE, dt);
    ivp_env_pcount(env, IVP_PE_PSI_CRITICAL_MINDISTS);
    pl->state = IVP_ES_PSI_CRITIC;
    ivp_mindist_mgr_recalc_all_exact_mindists_events(&pl->mindist_mgr);
    ivp_env_pcount(env, IVP_PE_PSI_END);
    env_emit_phase(env, IVP_PSI_PHASE_MANAGER_EVENTS, dt);
    env_emit_phase(env, IVP_PSI_PHASE_MOVEMENT_STATE, dt);
    env_emit_phase(env, IVP_PSI_PHASE_SIM_UNIT_END, dt);
    ivp_environment_emit_event_internal(env, IVP_EVENT_SIM_UNIT_END, dt, -1, 0);
    env_emit_phase(env, IVP_PSI_PHASE_POST_PSI, dt);
    pl->state = IVP_ES_AT;
    env->current_sim_unit_index = -1;
    pl->in_psi = false;
}

static void env_psi_event(ivp_time_event_t *ev, struct ivp_environment *env)
{
    ivp_env_pipeline_t *pl = env->pl;
    double delta_psi_time = (double)ivp_env_get_delta_psi_time(env);
    pl->time_of_last_psi = env->current_time;
    pl->time_of_next_psi = pl->time_of_last_psi + delta_psi_time;
    ivp_time_manager_rebase_for_psi(&pl->time_manager, pl->time_of_last_psi);
    env_simulate_psi(env);
    ivp_time_manager_insert_event(&pl->time_manager, ev, pl->time_of_next_psi);
}

/* ── simulate API ──────────────────────────────────────────────────────── */

/* get_betterstatisticsmanager()->set_simulation_time(time) */
static void env_set_statistics_simulation_time(ivp_environment_t *env, ivp_time_t time)
{
    if (env->better_statistics_manager) {
        ivp_better_statistics_manager_set_simulation_time(env->better_statistics_manager, (ivp_real_t)time);
    }
}

#if defined(IVP_HOME_BUILD) || defined(IVP_BENCHMARK_DIAGNOSTICS)
/* These opt-in legacy modes intentionally keep process-wide mutable
 * instrumentation state.  Serialize them instead of exposing a data race;
 * ordinary builds retain independent per-environment ownership. */
static atomic_bool s_legacy_global_simulation_running = false;
#endif

bool ivp_environment_try_begin_simulation(ivp_environment_t *env)
{
    if (!env) return false;
    bool expected = false;
    if (!atomic_compare_exchange_strong_explicit(&env->pl->simulation_running,
                                                 &expected, true,
                                                 memory_order_acquire,
                                                 memory_order_relaxed)) {
        return false;
    }
#if defined(IVP_HOME_BUILD) || defined(IVP_BENCHMARK_DIAGNOSTICS)
    expected = false;
    if (!atomic_compare_exchange_strong_explicit(
            &s_legacy_global_simulation_running, &expected, true,
            memory_order_acquire, memory_order_relaxed)) {
        atomic_store_explicit(&env->pl->simulation_running, false,
                              memory_order_release);
        return false;
    }
#endif
    return true;
}

void ivp_environment_end_simulation(ivp_environment_t *env)
{
#if defined(IVP_HOME_BUILD) || defined(IVP_BENCHMARK_DIAGNOSTICS)
    atomic_store_explicit(&s_legacy_global_simulation_running, false,
                          memory_order_release);
#endif
    atomic_store_explicit(&env->pl->simulation_running, false,
                          memory_order_release);
}

static void env_simulate_until_owned(ivp_environment_t *env,
                                     ivp_time_t until_time)
{
    double delta_time = until_time - env->current_time;
    if (delta_time < -(double)P_FLOAT_RES) {
        IVP_ASSERT(delta_time >= -(double)P_FLOAT_RES);
        env_set_statistics_simulation_time(env, env->current_time);
        return;
    }
    if (delta_time <= (double)P_FLOAT_RES) {
        env_set_statistics_simulation_time(env, env->current_time);
        return;
    }
    env_set_statistics_simulation_time(env, until_time);
    ivp_time_manager_event_loop(&env->pl->time_manager, env, until_time);
}

void ivp_environment_simulate_until(ivp_environment_t *env, ivp_time_t until_time)
{
    if (!env) return;
    bool acquired = ivp_environment_try_begin_simulation(env);
    IVP_ASSERT_NONFATAL(acquired);
    if (!acquired) return;
    env_simulate_until_owned(env, until_time);
    ivp_environment_end_simulation(env);
}

void ivp_environment_simulate_dtime_owned(ivp_environment_t *env,
                                          ivp_real_t elapsed_time)
{
    double dtime = (double)elapsed_time;
    if (dtime < -(double)P_FLOAT_RES) {
        IVP_ASSERT(dtime >= -(double)P_FLOAT_RES);
        env_set_statistics_simulation_time(env, env->current_time);
        return;
    }
    if (dtime <= (double)P_FLOAT_RES) {
        env_set_statistics_simulation_time(env, env->current_time);
        return;
    }
    ivp_time_t new_time = env->current_time + dtime;
    env_set_statistics_simulation_time(env, new_time);
    ivp_time_manager_event_loop(&env->pl->time_manager, env, new_time);
}

void ivp_environment_simulate_dtime(ivp_environment_t *env, ivp_real_t elapsed_time)
{
    if (!env) return;
    bool acquired = ivp_environment_try_begin_simulation(env);
    IVP_ASSERT_NONFATAL(acquired);
    if (!acquired) return;
    ivp_environment_simulate_dtime_owned(env, elapsed_time);
    ivp_environment_end_simulation(env);
}

void ivp_environment_simulate_one_psi_owned(ivp_environment_t *env)
{
    env_simulate_until_owned(env, env->current_time + env->pl->delta_psi_time);
}

void ivp_environment_simulate_one_psi(ivp_environment_t *env)
{
    if (!env) return;
    bool acquired = ivp_environment_try_begin_simulation(env);
    IVP_ASSERT_NONFATAL(acquired);
    if (!acquired) return;
    ivp_environment_simulate_one_psi_owned(env);
    ivp_environment_end_simulation(env);
}

void ivp_environment_simulate_variable_time_step(ivp_environment_t *env, ivp_real_t delta_time)
{
    if (!env) return;
    bool acquired = ivp_environment_try_begin_simulation(env);
    IVP_ASSERT_NONFATAL(acquired);
    if (!acquired) return;
    float dt = (float)delta_time;
    if (dt < (float)IVP_MIN_DELTA_PSI_TIME) dt = (float)IVP_MIN_DELTA_PSI_TIME;
    if (dt > (float)IVP_MAX_DELTA_PSI_TIME) dt = (float)IVP_MAX_DELTA_PSI_TIME;
    env_set_statistics_simulation_time(env, env->current_time + (double)dt);
    ivp_time_manager_simulate_variable_time_step(&env->pl->time_manager, env, dt);
    ivp_environment_end_simulation(env);
}

/* IVP_Environment::reset_time */
void ivp_environment_reset_time(ivp_environment_t *env)
{
    if (!env) return;
    ivp_env_pipeline_t *pl = env->pl;
    ivp_time_t old = pl->time_of_last_psi;
    ivp_time_manager_reset_time(&pl->time_manager, old);
    ivp_sim_units_mgr_reset_time(&pl->sim_units_manager, old);
    {   /* IVP_Controller::reset_time of libivp controller slots (once each) */
        ivp_controller_system_t *cs = env_controller_system(env);
        for (int i = 0; i < ivp_controller_system_count(cs); i++) {
            ivp_controller_slot_t *slot = ivp_controller_system_get_at(cs, i);
            const ivp_controller_descriptor_t *b =
                slot ? slot->descriptor : NULL;
            if (b && b->reset_time) b->reset_time(slot->ctx, old);
        }
    }
    /* IVP_Time(get_current_time() - get_old_time_of_last_PSI()): IVP_Time::operator- is float */
    ivp_environment_set_current_time_internal(env, ivp_time_diff(env->current_time, old));
    pl->time_of_last_psi = 0.0;
    pl->time_of_next_psi = 0.0 + (double)ivp_env_get_delta_psi_time(env);
}

/* ── environment lifecycle ─────────────────────────────────────────────── */

static bool env_create_static_object(ivp_environment_t *env);

ivp_environment_t *ivp_environment_create(const ivp_environment_config_t *config)
{
    ivp_environment_config_t cfg;
    if (config) {
        cfg = *config;
    } else {
        ivp_environment_config_init(&cfg);
    }
    const ivp_allocator_t *alloc = cfg.allocator ? cfg.allocator : ivp_default_allocator();
    ivp_environment_t *env = (ivp_environment_t *)ivp_alloc(alloc, sizeof(ivp_environment_t));
    if (!env) return NULL;
    memset(env, 0, sizeof(*env));
    env->freeze_params = cfg.freeze_params;
    env->current_time = 0.0;
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
    env->pl = (struct ivp_env_pipeline *)ivp_alloc(env->allocator, sizeof(ivp_env_pipeline_t));
    if (env->object_system) memset(env->object_system, 0, sizeof(ivp_object_system_t));
    if (env->controller_system) memset(env->controller_system, 0, sizeof(ivp_controller_system_t));
    if (env->event_system) memset(env->event_system, 0, sizeof(ivp_event_system_t));
    if (env->pl) memset(env->pl, 0, sizeof(ivp_env_pipeline_t));
    if (!env->object_system || !env->controller_system || !env->event_system || !env->pl) {
        ivp_environment_destroy(env);
        return NULL;
    }
    if (ivp_object_system_init(env_object_system(env), env->allocator, cfg.initial_object_capacity) != IVP_OK ||
        ivp_controller_system_init(env_controller_system(env), env->allocator, cfg.initial_controller_capacity) != IVP_OK ||
        ivp_event_system_init(env_event_system(env), env->allocator, cfg.initial_event_listener_capacity) != IVP_OK) {
        ivp_environment_destroy(env);
        return NULL;
    }
    ivp_anomaly_manager_init(&env->anomaly_manager, &cfg.anomaly_config);

    env->collision_filter = cfg.collision_filter;
    env->ccd_enabled = cfg.ccd_enabled;

    ivp_env_pipeline_t *pl = env->pl;
    ivp_mindist_settings_init(&pl->mindist_settings);
    atomic_init(&pl->simulation_running, false);
    if (cfg.parallel_worker_count > 1) {
        /* threads and their synchronization must stay out of a memory
         * region, whose checkpoints would write old bytes over them */
        const ivp_allocator_t *pool_allocator = env->allocator;
#if IVP_ENABLE_CHECKPOINTS
        if (ivp_memory_region_is_allocator(pool_allocator)) {
            pool_allocator = ivp_default_allocator();
        }
#endif
        pl->task_pool = ivp_task_pool_create(cfg.parallel_worker_count,
                                             pool_allocator);
        if (!pl->task_pool) {
            ivp_environment_destroy(env);
            return NULL;
        }
    }
    if (cfg.range_manager) {
        pl->range_manager = cfg.range_manager;
    } else {
        ivp_range_manager_init(&pl->own_range_manager, cfg.delta_psi_time);
        pl->range_manager = &pl->own_range_manager;
    }
    if (cfg.active_value_manager) {
        pl->active_value_manager = cfg.active_value_manager;
        pl->owns_active_value_manager = false;
    } else {
        pl->active_value_manager = ivp_active_value_manager_create(env->allocator);
        pl->owns_active_value_manager = true;
        if (!pl->active_value_manager) {
            ivp_environment_destroy(env);
            return NULL;
        }
    }
    /* the manager's "current_time" terminal (borrowed) */
    env->mod_current_time = pl->active_value_manager ? ivp_active_value_manager_current_time(pl->active_value_manager) : NULL;
    pl->state = IVP_ES_PSI;
    pl->rand_seed = 1;
    ivp_pvec_init_with_allocator(&pl->core_revive_list, env->allocator);
    ivp_pvec_init_with_allocator(&pl->touched_cores, env->allocator);
    ivp_pvec_init_with_allocator(&pl->active_hulls, env->allocator);
    ivp_pvec_init_with_allocator(&pl->parallel_sim_units, env->allocator);
    ivp_pvec_init_with_allocator(&pl->deferred_deletions, env->allocator);
    ivp_pvec_init_with_allocator(&pl->all_core_ctrls, env->allocator);
    ivp_pvec_init_with_allocator(&pl->slot_ctrls, env->allocator);
    ivp_pvec_init_with_allocator(&pl->global_object_listeners, env->allocator);
    ivp_pvec_init_with_allocator(&pl->obj_callback_tables, env->allocator);
    ivp_pvec_init_with_allocator(&pl->global_collision_listeners, env->allocator);
    ivp_pvec_init_with_allocator(&pl->coll_callback_tables, env->allocator);
    if (ivp_arena_init_sized(&pl->sim_unit_mem, env->allocator,
                             IVP_SIM_MEM_FIRST_BLOCK_SIZE) != IVP_OK) {
        ivp_environment_destroy(env);
        return NULL;
    }
    ivp_arena_set_clear_on_allocate(
        &pl->sim_unit_mem,
        !ivp_build_uses_reference_internal_state());
    /* short_term_mem = new IVP_U_Memory(); init_mem_transaction_usage() */
    if (ivp_arena_init(&pl->short_term_mem, env->allocator) != IVP_OK) {
        ivp_environment_destroy(env);
        return NULL;
    }
    ivp_arena_set_clear_on_allocate(
        &pl->short_term_mem,
        !ivp_build_uses_reference_internal_state());
    pl->state = IVP_ES_AT;
    /* standard gravity controller + set_gravity */
    pl->gravity_ctrl.vt = &gravity_vt;
    ivp_env_register_sim_controller(env, &pl->gravity_ctrl);
    ivp_environment_set_gravity(env, &cfg.gravity);
    /* set_delta_PSI_time */
    double dt = (double)cfg.delta_psi_time;
    if (!env_set_delta_psi_time_d(env, dt)) (void)env_set_delta_psi_time_d(env, (double)(1.0f / 66.0f));
    ivp_sim_units_mgr_init(&pl->sim_units_manager, env);
    if (ivp_time_manager_init(&pl->time_manager, env->allocator, env_psi_event) != IVP_OK) {
        ivp_environment_destroy(env);
        return NULL;
    }
    ivp_mindist_mgr_init(&pl->mindist_mgr, env);
    if (ivp_ov_tree_init_with_allocator(&pl->ov_tree, env->allocator) != IVP_OK) {
        ivp_environment_destroy(env);
        return NULL;
    }
    /* statistic_manager.l_environment = this */
    ivp_statistic_manager_init(&pl->statistic_manager);
    pl->statistic_manager.l_environment = env;
    /* better_statisticsmanager = new IVP_BetterStatisticsmanager() */
    env->better_statistics_manager =
        (ivp_better_statistics_manager_t *)ivp_alloc(env->allocator, sizeof(ivp_better_statistics_manager_t));
    if (!env->better_statistics_manager) {
        ivp_environment_destroy(env);
        return NULL;
    }
    ivp_better_statistics_manager_init_with_allocator(env->better_statistics_manager,
                                                      env->allocator);
    /* performancecounter: the application's, else new IVP_PerformanceCounter_Simple() */
    if (cfg.performance_counter) {
        env->performance_counter = cfg.performance_counter;
    } else if (!cfg.disable_performance_counter) {
        ivp_performance_counter_simple_t *simple = ivp_performance_counter_simple_create(env->allocator);
        if (!simple) {
            ivp_environment_destroy(env);
            return NULL;
        }
        env->performance_counter = &simple->base;
    }
    if (ivp_cache_obj_mgr_init(&pl->cache_mgr, env->allocator,
                               cfg.n_cache_objects > 0 ? cfg.n_cache_objects : 256) != IVP_OK) {
        ivp_environment_destroy(env);
        return NULL;
    }
    pl->current_time_code = 1;
    env->current_time = 0.0;
    pl->time_of_next_psi = (double)ivp_env_get_delta_psi_time(env);
    pl->next_movement_check = IVP_MOVEMENT_CHECK_COUNT;
    /* debug_information = new IVP_Debug_Manager() (reference: debug builds only) */
    env->debug_manager = (ivp_debug_manager_t *)ivp_alloc(env->allocator, sizeof(ivp_debug_manager_t));
    if (!env->debug_manager) {
        ivp_environment_destroy(env);
        return NULL;
    }
    ivp_debug_manager_init(env->debug_manager);
    /* default collision delegator root */
    pl->root_mindist = ivp_collision_delegator_root_mindist_create(env);
    if (!pl->root_mindist) {
        ivp_environment_destroy(env);
        return NULL;
    }
    env->collision_delegator_roots[env->collision_delegator_root_count++] = pl->root_mindist;
    ivp_cluster_init(&pl->root_cluster, env, NULL); /* IVP_Cluster_Manager::root_cluster */
    if (!env_create_static_object(env)) {
        ivp_environment_destroy(env);
        return NULL;
    }
    return env;
}

static void env_remove_controllers_of_object(ivp_environment_t *env, ivp_object_t *obj);

/* IVP_Environment ctor: create a static ball (not in the object list) */
static bool env_create_static_object(ivp_environment_t *env)
{
    ivp_object_t *obj = (ivp_object_t *)ivp_alloc(env->allocator, sizeof(ivp_object_t));
    if (!obj) return false;
    memset(obj, 0, sizeof(*obj));
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.ball_radius = IVP_REAL(1.0);
    t.name = "static_object";
    t.physical_unmoveable = true;
    t.use_mass_center_override = true;
    ivp_vec3r_set(&t.mass_center_override, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    t.enable_collision_detection = false;
    t.start_in_simulation = false;
    obj->type = t.type;
    obj->material = t.material;
    obj->layer_mask = 0xFFFFFFFFu;
    obj->name = t.name;
    obj->owner = env;
    obj->dense_index = -1;
    obj->handle.slot = 0xFFFFFFFFu;
    obj->handle.generation = 0;
    obj->creation_sequence = env->pl->next_object_sequence++;
    obj->shift_core_f_object_is_zero = true;
    if (!ivp_object_pipeline_create(env, obj, &t)) {
        ivp_free(env->allocator, obj, sizeof(ivp_object_t));
        return false;
    }
    env->pl->static_object = obj;
    ivp_cluster_add_child(&env->pl->root_cluster, obj); /* create_ball(root cluster) */
    return true;
}


/* ~IVP_Environment: static_object->delete_silently(); static_object = NULL */
static void env_delete_static_object(ivp_environment_t *env)
{
    ivp_object_t *obj = env->pl->static_object;
    if (!obj) return;
    obj->object_deletion_deferred = false;
    env_delete_object_now(env, obj, false);   /* frees the static object */
    env->pl->static_object = NULL;
}

/* ~IVP_Cluster (P_DELETE(cluster_manager) -> P_DELETE(root_cluster)): delete
 * the first object of the cluster until none is left; sub-clusters delete
 * their objects first and are deleted with the environment */
static void env_delete_cluster_objects(ivp_environment_t *env, ivp_cluster_t *cluster)
{
    while (cluster->first_child) {
        ivp_object_t *o = cluster->first_child;
        if (o->type == IVP_OBJECT_CLUSTER) {
            ivp_cluster_t *sub = (ivp_cluster_t *)o;
            env_delete_cluster_objects(env, sub);
            ivp_cluster_remove_child(cluster, o);   /* ~IVP_Object */
            ivp_free(env->allocator, sub, sizeof(ivp_cluster_t));
        } else {
            o->object_deletion_deferred = false;
            env_delete_object_now(env, o, false);
        }
    }
}

struct ivp_active_value_manager *ivp_environment_get_active_value_manager(const ivp_environment_t *env)
{
    return (env && env->pl) ? env->pl->active_value_manager : NULL;
}

ivp_object_t *ivp_environment_get_static_object(const ivp_environment_t *env)
{
    return (env && env->pl) ? env->pl->static_object : NULL;
}

void ivp_environment_destroy(ivp_environment_t *env)
{
    if (!env) return;
    ivp_allocator_t *alloc = env->allocator;
    const bool owns = env->owns_allocator;
    ivp_allocator_t alloc_copy;
    memset(&alloc_copy, 0, sizeof(alloc_copy));
    if (owns && alloc) alloc_copy = *alloc;
    ivp_env_pipeline_t *pl = env->pl;

    if (pl && env->event_system) {
        /* ~IVP_Environment: IVP_Listener_PSI::environment_will_be_deleted */
        ivp_environment_emit_event_internal(env, IVP_EVENT_ENVIRONMENT_DELETED, IVP_REAL(0.0), -1, 0);
    }
    if (pl) {
        /* IVP_Listener_Object::event_environment_deleted */
        for (int i = ivp_pvec_len(&pl->global_object_listeners) - 1; i >= 0; i--) {
            if (i >= ivp_pvec_len(&pl->global_object_listeners)) continue;
            ivp_listener_object_t *l = (ivp_listener_object_t *)ivp_pvec_at(&pl->global_object_listeners, i);
            if (l->event_environment_deleted) l->event_environment_deleted(l, env);
        }
        /* IVP_Listener_Collision::event_environment_deleted */
        for (int i = ivp_pvec_len(&pl->global_collision_listeners) - 1; i >= 0; i--) {
            if (i >= ivp_pvec_len(&pl->global_collision_listeners)) continue;
            ivp_collision_listener_t *l = (ivp_collision_listener_t *)ivp_pvec_at(&pl->global_collision_listeners, i);
            if (l->active && l->environment_deleted_fn) l->environment_deleted_fn(l->ctx, env);
        }
    }
    if (pl && env->object_system) {
        /* ~IVP_Environment: the static object first (delete_silently), then
         * P_DELETE(cluster_manager) deletes the objects in cluster order: the
         * root cluster deletes its first object (the newest one) until it is
         * empty, sub-clusters recursively */
        if (!ivp_build_uses_reference_internal_state()) {
            ivp_cluster_restore_child_order(&pl->root_cluster);
        }
        env_delete_static_object(env);
        env_delete_cluster_objects(env, &pl->root_cluster);
        /* objects outside the root cluster tree, newest first */
        ivp_object_system_t *os = env_object_system(env);
        while (ivp_object_system_count(os) > 0) {
            ivp_object_t *obj = ivp_object_system_get_at(os, ivp_object_system_count(os) - 1);
            obj->object_deletion_deferred = false;
            env_delete_object_now(env, obj, false);
        }
    }
    if (pl) env_delete_static_object(env);
    if (pl && env->controller_system) ivp_environment_clear_controllers(env);
    if (env->universe_manager) {
        ivp_free(alloc, env->universe_manager, sizeof(*env->universe_manager));
        env->universe_manager = NULL;
    }
    if (pl) {
        for (int k = env->collision_delegator_root_count - 1; k >= 0; k--) {
            ivp_collision_delegator_root_t *cdr = env->collision_delegator_roots[k];
            if (cdr && cdr->rvt->environment_is_going_to_be_deleted_event) {
                cdr->rvt->environment_is_going_to_be_deleted_event(cdr, env);
            }
        }
        env->collision_delegator_root_count = 0;
        ivp_time_manager_destroy(&pl->time_manager);
        ivp_mindist_mgr_destroy(&pl->mindist_mgr);
        ivp_ov_tree_destroy(&pl->ov_tree);
        /* P_DELETE(better_statisticsmanager); P_DELETE(debug_information);
         * performancecounter->environment_is_going_to_be_deleted(this) */
        if (env->better_statistics_manager) {
            ivp_better_statistics_manager_destroy(env->better_statistics_manager);
            ivp_free(alloc, env->better_statistics_manager, sizeof(ivp_better_statistics_manager_t));
            env->better_statistics_manager = NULL;
        }
        if (env->debug_manager) {
            ivp_debug_manager_clear(env->debug_manager);
            ivp_free(alloc, env->debug_manager, sizeof(ivp_debug_manager_t));
            env->debug_manager = NULL;
        }
        if (env->performance_counter) {
            ivp_performance_counter_t *pc = env->performance_counter;
            env->performance_counter = NULL;
            ivp_performance_counter_environment_is_going_to_be_deleted(pc, env);
        }
        ivp_cache_obj_mgr_destroy(&pl->cache_mgr);
        ivp_task_pool_destroy(pl->task_pool);
        pl->task_pool = NULL;
        ivp_arena_destroy(&pl->sim_unit_mem);
        ivp_env_delete_draw_vector_debug(env);
        /* remaining (empty) sim units */
        for (ivp_sim_unit_t *s = pl->sim_units_manager.sim_units_slot0; s;) {
            ivp_sim_unit_t *n = s->next_sim_unit;
            ivp_sim_unit_delete(s);
            s = n;
        }
        for (ivp_sim_unit_t *s = pl->sim_units_manager.still_slot; s;) {
            ivp_sim_unit_t *n = s->next_sim_unit;
            ivp_sim_unit_delete(s);
            s = n;
        }
        ivp_pvec_clear(&pl->core_revive_list);
        ivp_pvec_clear(&pl->touched_cores);
        ivp_pvec_clear(&pl->active_hulls);
        ivp_pvec_clear(&pl->parallel_sim_units);
        ivp_pvec_clear(&pl->deferred_deletions);
        ivp_pvec_clear(&pl->all_core_ctrls);
        ivp_pvec_clear(&pl->slot_ctrls);
        if (pl->active_value_manager && pl->owns_active_value_manager) ivp_active_value_manager_destroy(pl->active_value_manager);
        pl->active_value_manager = NULL;
        cb_clear_tables(&pl->obj_callback_tables);
        cb_clear_tables(&pl->coll_callback_tables);
        ivp_pvec_clear(&pl->global_object_listeners);
        ivp_pvec_clear(&pl->global_collision_listeners);
        ivp_arena_destroy(&pl->short_term_mem);
        /* (the root cluster tree is empty: its objects and sub-clusters were deleted) */
        while (pl->root_cluster.first_child) ivp_cluster_remove_child(&pl->root_cluster, pl->root_cluster.first_child);
        ivp_free(alloc, pl, sizeof(ivp_env_pipeline_t));
        env->pl = NULL;
    }
    env->mod_current_time = NULL; /* borrowed from the active value manager */
    if (env->event_system) {
        ivp_event_system_destroy(env_event_system(env));
        ivp_free(alloc, env->event_system, sizeof(ivp_event_system_t));
    }
    if (env->controller_system) {
        ivp_controller_system_destroy(env_controller_system(env));
        ivp_free(alloc, env->controller_system, sizeof(ivp_controller_system_t));
    }
    if (env->object_system) {
        ivp_object_system_destroy(env_object_system(env));
        ivp_free(alloc, env->object_system, sizeof(ivp_object_system_t));
    }
    ivp_free(alloc, env, sizeof(ivp_environment_t));
    if (owns) ivp_free(&alloc_copy, alloc, sizeof(ivp_allocator_t));
}

/* ── objects ───────────────────────────────────────────────────────────── */

static void env_reset_object_sequence_if_empty(ivp_environment_t *env)
{
    bool has_live_clusters = false;
#if IVP_STABLE_INTERNAL_STATE
    has_live_clusters = env->pl->live_cluster_count != 0;
#endif
    if (ivp_object_system_count(env_object_system(env)) == 0 &&
        !has_live_clusters) {
        /* The static object permanently owns sequence zero.  Once no public
         * objects remain, a new scene can start from the same stable
         * order even if registry slots and allocator pools are still warm. */
        env->pl->next_object_sequence = 1u;
        env->pl->next_contact_sequence = 1u;
    }
}

ivp_object_t *ivp_environment_create_object_with_handle(ivp_environment_t *env, const ivp_object_template_t *tmpl,
                                                        const ivp_surman_t *surman, ivp_object_handle_t *out_handle)
{
    if (out_handle) *out_handle = ivp_object_handle_invalid();
    if (!env || !tmpl) return NULL;
    ivp_object_t *obj = ivp_object_system_create(env_object_system(env), env, tmpl, surman, out_handle);
    if (!obj) return NULL;
    const int object_count = ivp_object_system_count(env_object_system(env));
    /* A PSI can touch at most one core and one hull manager per object.  Grow
     * the reusable scratch here so the simulation hot path never allocates. */
    if (!ivp_pvec_reserve(&env->pl->touched_cores, object_count) ||
        !ivp_pvec_reserve(&env->pl->active_hulls, object_count)) {
        (void)ivp_object_system_destroy_by_handle(
            env_object_system(env), ivp_environment_get_object_handle(obj));
        env_reset_object_sequence_if_empty(env);
        if (out_handle) *out_handle = ivp_object_handle_invalid();
        return NULL;
    }
    if (!ivp_object_pipeline_create(env, obj, tmpl)) {
        (void)ivp_object_system_destroy_by_handle(
            env_object_system(env), ivp_environment_get_object_handle(obj));
        env_reset_object_sequence_if_empty(env);
        if (out_handle) *out_handle = ivp_object_handle_invalid();
        return NULL;
    }
    obj->creation_sequence = env->pl->next_object_sequence++;
#if IVP_EDIT_LOCALITY
    obj->movement_check_salt = env_movement_check_salt(env, tmpl);
#endif
    ivp_cluster_add_child(&env->pl->root_cluster, obj); /* IVP_Object(father = root cluster) */
    fire_object_event_global(env, obj, OBJ_EV_CREATED); /* end of IVP_Polygon / IVP_Ball ctor */
    ivp_environment_emit_event_internal(env, IVP_EVENT_OBJECT_CREATED, IVP_REAL(0.0), obj->dense_index, 0);
    if (tmpl->enable_collision_detection) ivp_object_enable_collision_detection(obj, true);
    if (tmpl->start_in_simulation && !obj->physical_core->physical_unmoveable) ivp_object_ensure_in_simulation_now(obj);
    return obj;
}

ivp_object_t *ivp_environment_create_object(ivp_environment_t *env, const ivp_object_template_t *tmpl,
                                            const ivp_surman_t *surman)
{
    return ivp_environment_create_object_with_handle(env, tmpl, surman, NULL);
}

ivp_object_t *ivp_environment_get_object_by_handle(const ivp_environment_t *env, ivp_object_handle_t handle)
{
    if (!env || !ivp_object_handle_is_valid(handle)) return NULL;
    return ivp_object_system_get_by_handle(env_object_system(env), handle);
}

ivp_object_handle_t ivp_environment_get_object_handle(const ivp_object_t *obj)
{
    if (!obj) return ivp_object_handle_invalid();
    ivp_object_handle_t h;
    h.slot = obj->handle.slot;
    h.generation = obj->handle.generation;
    return h;
}

/* clear_internal_references: an actuator / constraint is deleted together
 * with an object it is anchored to (IVP_Anchor::object_is_going_to_be_deleted
 * -> anchor_will_be_deleted_event).  libivp controllers that act on a fixed
 * set of objects (dependent controllers) are unregistered when one of their
 * objects is deleted; their state structs belong to the caller. */
static void env_remove_controllers_of_object(ivp_environment_t *env, ivp_object_t *obj)
{
    ivp_controller_system_t *cs = env_controller_system(env);
    ivp_object_handle_t oh = ivp_environment_get_object_handle(obj);
    ivp_controller_handle_t doomed[64];
    int n_doomed = 0;
    const bool core_dies = obj->physical_core && ivp_core_n_objects(obj->physical_core) <= 1;
    for (int i = 0; i < ivp_controller_system_count(cs) && n_doomed < 64; i++) {
        ivp_controller_slot_t *slot = ivp_controller_system_get_at(cs, i);
        if (!slot || slot->mode != IVP_CONTROLLER_DEPENDENT) continue;
        /* the object's core is deleted with it (last object of the core) */
        bool uses = core_dies &&
                    ivp_pvec_index_of(
                        &slot->controlled_cores, obj->physical_core) >= 0;
        if (!uses) {
            ivp_object_handle_t handles[IVP_CONTROLLER_DISCOVERY_STACK_MAX];
            /* every anchor object, unmoveable ones included */
            env->pl->discovery_all_anchors = true;
            int n = ivp_controller_slot_collect_associated_handles(slot, env, handles, IVP_CONTROLLER_DISCOVERY_STACK_MAX);
            env->pl->discovery_all_anchors = false;
            for (int k = 0; k < n; k++) {
                if (handles[k].slot == oh.slot && handles[k].generation == oh.generation) {
                    uses = true;
                    break;
                }
            }
        }
        if (uses) doomed[n_doomed++] = slot->handle;
    }
    /* actuators / constraints remove themselves silently (ivp_actuator.cxx:209) */
    for (int i = 0; i < n_doomed; i++) (void)ivp_environment_remove_dependent_controller(env, doomed[i], true);
}

bool ivp_environment_discovery_reports_all_anchors(const ivp_environment_t *env)
{
    return env && env->pl && env->pl->discovery_all_anchors;
}

/* ~IVP_Real_Object (+ delete_and_check_vicinity when check_vicinity) */
static void env_delete_object_now(ivp_environment_t *env, ivp_object_t *obj, bool check_vicinity)
{
    ivp_object_t *previous_being_deleted = env->pl->object_being_deleted;
    env->pl->object_being_deleted = obj;
    if (check_vicinity) {
        ivp_core_t *my_core = obj->physical_core;   /* get_core() */
        if (!my_core->physical_unmoveable && my_core->sim_unit) ivp_sim_unit_ensure_in_simulation(my_core->sim_unit);
        ivp_object_revive_nearest_objects_grow_fs(obj);
    }
    /* ~IVP_Real_Object */
    env_remove_controllers_of_object(env, obj);
    /* clear_internal_references: anchors of actuators that stay registered
     * (more than 64 controllers of the object) leave the object */
    while (obj->anchors) ivp_anchor_unlink(obj->anchors);
    ivp_object_pipeline_clear_references(env, obj);
    fire_object_event_global(env, obj, OBJ_EV_DELETED);     /* global 'object deleted' */
    ivp_env_fire_collision_private(env, obj, COLL_OBJ_DELETED, (const void *)obj); /* local collision listeners */
    if (obj->object_listener_exists) {                      /* object-dependent 'object deleted' */
        fire_object_event_private(env, obj, OBJ_EV_DELETED);
    }
    {   /* IVP_Cluster_Manager::remove_object */
        env_obj_cb_table_t *t = obj_cb_find_table(env->pl, obj);
        if (t) obj_cb_remove_table(env->pl, t);
        obj->object_listener_exists = false;
        env_obj_cb_table_t *ct = cb_find_table(&env->pl->coll_callback_tables, obj);
        if (ct) cb_remove_table(&env->pl->coll_callback_tables, ct);
        obj->collision_listener_exists = false;
    }
    if (obj == env->pl->an_object_to_be_checked) {
        env->pl->an_object_to_be_checked = env_next_object_to_check(env, obj);
        if (env->pl->an_object_to_be_checked == obj) env->pl->an_object_to_be_checked = NULL;
    }
    if (env->universe_manager && env->universe_manager->deleted_fn) {
        env->universe_manager->deleted_fn(env->universe_manager->ctx, obj);
    }
    /* controllers attached through slots lose this core (when it is deleted
     * with its last object) */
    if (obj->physical_core && ivp_core_n_objects(obj->physical_core) <= 1) {
        for (int i = 0; i < ivp_pvec_len(&env->pl->slot_ctrls); i++) {
            ivp_controller_slot_t *slot =
                (ivp_controller_slot_t *)ivp_pvec_at(
                    &env->pl->slot_ctrls, i);
            (void)ivp_pvec_remove_if_present(
                &slot->controlled_cores, obj->physical_core);
        }
    }
    ivp_object_pipeline_destroy(env, obj);
    if (obj->parent_cluster) ivp_cluster_remove_child(obj->parent_cluster, obj); /* ~IVP_Object */
    IVP_ASSERT(obj->owner);   /* ~IVP_Object: environment */
    env->pl->object_being_deleted = previous_being_deleted;
    if (obj == env->pl->static_object) {   /* not in the object system */
        ivp_free(env->allocator, obj, sizeof(ivp_object_t));
        return;
    }
    const int object_index = obj->dense_index;
    if (ivp_object_system_destroy_by_handle(
            env_object_system(env), ivp_environment_get_object_handle(obj))) {
        ivp_environment_emit_event_internal(
            env, IVP_EVENT_OBJECT_DESTROYED, IVP_REAL(0.0), object_index, 0);
    }
    env_reset_object_sequence_if_empty(env);
}

bool ivp_environment_destroy_object_by_handle(ivp_environment_t *env, ivp_object_handle_t handle)
{
    if (!env || !ivp_object_handle_is_valid(handle)) return false;
    ivp_object_t *obj = ivp_object_system_get_by_handle(env_object_system(env), handle);
    if (!obj) return false;
    ivp_env_pipeline_t *pl = env->pl;
    /* a deletion callback that deletes the object being deleted: the
     * running deletion owns it */
    if (obj == pl->object_being_deleted) return true;
    if (pl->deferred_deletion_depth > 0 || pl->deferred_deletion_draining) {
        /* ivp_defer_object_deletion */
        if (!obj->object_deletion_deferred) {
            if (ivp_pvec_add(&pl->deferred_deletions, obj) < 0) return false;
            obj->object_deletion_deferred = true;
            obj->check_vicinity_on_deferred_deletion = true;
        } else {
            obj->check_vicinity_on_deferred_deletion = true;
        }
        return true;
    }
    env_delete_object_now(env, obj, true);
    return true;
}

void ivp_environment_destroy_object(ivp_environment_t *env, ivp_object_t *obj)
{
    if (!env || !obj || obj->owner != env) return;
    (void)ivp_environment_destroy_object_by_handle(env, ivp_environment_get_object_handle(obj));
}

int ivp_environment_get_object_count(const ivp_environment_t *env)
{
    return env ? ivp_object_system_count(env_object_system(env)) : 0;
}

ivp_object_t *ivp_environment_get_object(const ivp_environment_t *env, int index)
{
    if (!env || index < 0 || index >= ivp_environment_get_object_count(env)) return NULL;
    return ivp_object_system_get_at(env_object_system(env), index);
}

/* ── clusters ──────────────────────────────────────────────────────────── */

ivp_cluster_t *ivp_environment_get_root_cluster(ivp_environment_t *env)
{
    return (env && env->pl) ? &env->pl->root_cluster : NULL;
}

ivp_cluster_t *ivp_environment_create_cluster(ivp_environment_t *env, ivp_cluster_t *parent, const ivp_cluster_template_t *tmpl)
{
    if (!env) return NULL;
    return ivp_cluster_create(env, parent ? parent : &env->pl->root_cluster, tmpl);
}

void ivp_environment_destroy_cluster(ivp_environment_t *env, ivp_cluster_t *cluster)
{
    if (!env || !cluster) return;
    ivp_cluster_destroy(cluster);
}

/* ── collision delegators / global collision listeners ────────────────── */

bool ivp_environment_add_collision_delegator(ivp_environment_t *env, struct ivp_collision_delegator_root *root)
{
    if (!env || !root) return false;
    if (env->collision_delegator_root_count >= IVP_MAX_COLLISION_DELEGATOR_ROOTS) return false;
    env->collision_delegator_roots[env->collision_delegator_root_count++] = root;
    return true;
}

bool ivp_environment_remove_collision_delegator(ivp_environment_t *env, struct ivp_collision_delegator_root *root)
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

int ivp_environment_add_collision_listener_global(ivp_environment_t *env, ivp_collision_listener_t *listener)
{
    if (!env || !listener) return -1;
    /* IVP_Environment::add_listener_collision_global */
    return ivp_pvec_add(&env->pl->global_collision_listeners, listener) >= 0 ? 0 : -1;
}

int ivp_environment_remove_collision_listener_global(ivp_environment_t *env, ivp_collision_listener_t *listener)
{
    if (!env || !listener) return -1;
    return ivp_pvec_remove_if_present(&env->pl->global_collision_listeners, listener) ? 0 : -1;
}
