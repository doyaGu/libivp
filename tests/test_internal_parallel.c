/* test_internal_parallel.c -- deterministic parallel kernels inside one world. */
#include "ivp_test.h"

#include "base/ivp_task_pool.h"

#include <ivp/collision/ivp_collision_core.h>
#include <ivp/containers/ivp_pvec.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/controllers/ivp_constraint_local.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>

#include <math.h>
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#if defined(_WIN32)
#include <windows.h>
static void internal_parallel_yield(void) { Sleep(0); }
#else
#include <sched.h>
static void internal_parallel_yield(void) { (void)sched_yield(); }
#endif

#define INTERNAL_PARALLEL_WORKERS 4
#define TASK_EXACT_COUNT 1031
#define BALLISTIC_OBJECT_COUNT 2048
#define BROADPHASE_ELEMENT_COUNT 8192
#define SOLVER_CONTACT_COUNT 128
#define INDEPENDENT_CONSTRAINT_COUNT 512

typedef struct pool_overlap_probe {
    atomic_int entered;
    atomic_int active;
    atomic_int max_active;
    atomic_bool release;
} pool_overlap_probe_t;

static void update_atomic_max(atomic_int *value, int candidate)
{
    int old = atomic_load_explicit(value, memory_order_relaxed);
    while (candidate > old &&
           !atomic_compare_exchange_weak_explicit(
               value, &old, candidate, memory_order_relaxed,
               memory_order_relaxed)) {
    }
}

static void pool_overlap_task(void *opaque, size_t task_index)
{
    (void)task_index;
    pool_overlap_probe_t *probe = (pool_overlap_probe_t *)opaque;
    int active = atomic_fetch_add_explicit(
                     &probe->active, 1, memory_order_acq_rel) + 1;
    update_atomic_max(&probe->max_active, active);
    int entered = atomic_fetch_add_explicit(
                      &probe->entered, 1, memory_order_acq_rel) + 1;
    if (entered == INTERNAL_PARALLEL_WORKERS) {
        atomic_store_explicit(&probe->release, true, memory_order_release);
    }
    for (int spin = 0; spin < 1000000; spin++) {
        if (atomic_load_explicit(&probe->release, memory_order_acquire)) break;
        internal_parallel_yield();
    }
    atomic_fetch_sub_explicit(&probe->active, 1, memory_order_acq_rel);
}

IVP_TEST(task_pool_runs_concurrently)
{
    ivp_task_pool_t *pool =
        ivp_task_pool_create(INTERNAL_PARALLEL_WORKERS, NULL);
    bool ok = pool != NULL;
    pool_overlap_probe_t probe;
    atomic_init(&probe.entered, 0);
    atomic_init(&probe.active, 0);
    atomic_init(&probe.max_active, 0);
    atomic_init(&probe.release, false);
    if (pool) {
        ivp_task_pool_parallel_for(pool, INTERNAL_PARALLEL_WORKERS,
                                   pool_overlap_task, &probe);
        ok = ok &&
             ivp_task_pool_worker_count(pool) == INTERNAL_PARALLEL_WORKERS &&
             atomic_load_explicit(&probe.entered, memory_order_acquire) ==
                 INTERNAL_PARALLEL_WORKERS &&
             atomic_load_explicit(&probe.max_active, memory_order_acquire) >= 2;
    }
    ivp_task_pool_destroy(pool);
    IVP_ASSERT_TRUE(ok);
}

typedef struct task_exact_probe {
    atomic_int hits[TASK_EXACT_COUNT];
} task_exact_probe_t;

static void task_exact_once(void *opaque, size_t task_index)
{
    task_exact_probe_t *probe = (task_exact_probe_t *)opaque;
    atomic_fetch_add_explicit(&probe->hits[task_index], 1,
                              memory_order_relaxed);
}

IVP_TEST(task_pool_claims_every_task_exactly_once)
{
    ivp_task_pool_t *pool =
        ivp_task_pool_create(INTERNAL_PARALLEL_WORKERS, NULL);
    task_exact_probe_t probe;
    for (int i = 0; i < TASK_EXACT_COUNT; i++) {
        atomic_init(&probe.hits[i], 0);
    }
    bool ok = pool != NULL;
    if (pool) {
        ivp_task_pool_parallel_for(pool, TASK_EXACT_COUNT, task_exact_once,
                                   &probe);
        ivp_task_pool_parallel_for(pool, 17u, task_exact_once, &probe);
        for (int i = 0; i < TASK_EXACT_COUNT; i++) {
            int expected = i < 17 ? 2 : 1;
            if (atomic_load_explicit(&probe.hits[i],
                                     memory_order_relaxed) != expected) {
                ok = false;
                break;
            }
        }
    }
    ivp_task_pool_destroy(pool);
    IVP_ASSERT_TRUE(ok);
}

static ivp_environment_t *create_zero_gravity_environment(
    int worker_count, ivp_collision_filter_t collision_filter)
{
    ivp_environment_config_t config;
    ivp_environment_config_init(&config);
    config.parallel_worker_count = worker_count;
    config.disable_performance_counter = true;
    config.collision_filter = collision_filter;
    ivp_vec3r_set(&config.gravity, IVP_REAL(0.0), IVP_REAL(0.0),
                  IVP_REAL(0.0));
    return ivp_environment_create(&config);
}

IVP_TEST(broadphase_preserves_candidate_order)
{
    ivp_environment_t *serial = create_zero_gravity_environment(
        1, ivp_collision_filter_allow_all());
    ivp_environment_t *parallel = create_zero_gravity_environment(
        INTERNAL_PARALLEL_WORKERS, ivp_collision_filter_allow_all());

    ivp_ov_tree_t serial_tree;
    ivp_ov_tree_t parallel_tree;
    memset(&serial_tree, 0, sizeof(serial_tree));
    memset(&parallel_tree, 0, sizeof(parallel_tree));
    bool serial_tree_initialized =
        ivp_ov_tree_init_with_allocator(&serial_tree, NULL) == IVP_OK;
    bool parallel_tree_initialized =
        ivp_ov_tree_init_with_allocator(&parallel_tree, NULL) == IVP_OK;
    bool trees_initialized =
        serial_tree_initialized && parallel_tree_initialized;
    ivp_ov_element_t *serial_elements = (ivp_ov_element_t *)calloc(
        BROADPHASE_ELEMENT_COUNT + 1u, sizeof(*serial_elements));
    ivp_ov_element_t *parallel_elements = (ivp_ov_element_t *)calloc(
        BROADPHASE_ELEMENT_COUNT + 1u, sizeof(*parallel_elements));
    ivp_pvec_t serial_candidates;
    ivp_pvec_t parallel_candidates;
    ivp_pvec_init(&serial_candidates);
    ivp_pvec_init(&parallel_candidates);

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.5);
    object_template.mass = IVP_REAL(1.0);
    object_template.speed_damp_factor = IVP_REAL(0.0);
    object_template.enable_collision_detection = false;
    object_template.start_in_simulation = false;
    ivp_vec3r_set(&object_template.rot_speed_damp_factor,
                  IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *serial_owner = serial ? ivp_environment_create_object(
                                             serial, &object_template, NULL)
                                        : NULL;
    ivp_object_t *parallel_owner = parallel ? ivp_environment_create_object(
                                                 parallel, &object_template,
                                                 NULL)
                                            : NULL;
    bool ok = serial && parallel && trees_initialized && serial_elements &&
              parallel_elements && serial_owner && parallel_owner;

    for (int i = 0; ok && i < BROADPHASE_ELEMENT_COUNT; i++) {
        serial_elements[i].real_object = serial_owner;
        parallel_elements[i].real_object = parallel_owner;
        if (ivp_ov_tree_insert_ov_element_ex(
                &serial_tree, &serial_elements[i], 1.0, 1.0, NULL, NULL) !=
                IVP_OK ||
            ivp_ov_tree_insert_ov_element_ex(
                &parallel_tree, &parallel_elements[i], 1.0, 1.0, NULL,
                NULL) != IVP_OK) {
            ok = false;
        }
    }
    if (ok) {
        serial_elements[BROADPHASE_ELEMENT_COUNT].real_object = serial_owner;
        parallel_elements[BROADPHASE_ELEMENT_COUNT].real_object =
            parallel_owner;
        ok = ivp_ov_tree_insert_ov_element_ex(
                 &serial_tree, &serial_elements[BROADPHASE_ELEMENT_COUNT],
                 1.0, 1.0, &serial_candidates, NULL) == IVP_OK &&
             ivp_ov_tree_insert_ov_element_ex(
                 &parallel_tree,
                 &parallel_elements[BROADPHASE_ELEMENT_COUNT], 1.0, 1.0,
                 &parallel_candidates, NULL) == IVP_OK;
    }
    ok = ok && ivp_pvec_len(&serial_candidates) ==
                   BROADPHASE_ELEMENT_COUNT + 1 &&
         ivp_pvec_len(&parallel_candidates) ==
                   BROADPHASE_ELEMENT_COUNT + 1;
    for (int i = 0; ok && i < ivp_pvec_len(&serial_candidates); i++) {
        const ivp_ov_element_t *serial_element =
            (const ivp_ov_element_t *)ivp_pvec_at(&serial_candidates, i);
        const ivp_ov_element_t *parallel_element =
            (const ivp_ov_element_t *)ivp_pvec_at(&parallel_candidates, i);
        if (serial_element - serial_elements !=
            parallel_element - parallel_elements) {
            ok = false;
        }
    }

    ivp_parallel_stats_t stats;
    ivp_environment_get_parallel_stats(parallel, &stats);
    ok = ok && stats.broadphase_dispatches == 2u &&
         stats.broadphase_tasks >=
             (uint64_t)(2 * INTERNAL_PARALLEL_WORKERS) &&
         stats.max_workers_used == INTERNAL_PARALLEL_WORKERS;
    if (!ok) {
        fprintf(stderr,
                "broadphase: serial=%d parallel=%d "
                "dispatches=%llu tasks=%llu max_workers=%d\n",
                ivp_pvec_len(&serial_candidates),
                ivp_pvec_len(&parallel_candidates),
                (unsigned long long)stats.broadphase_dispatches,
                (unsigned long long)stats.broadphase_tasks,
                stats.max_workers_used);
    }

    ivp_pvec_clear(&parallel_candidates);
    ivp_pvec_clear(&serial_candidates);
    if (parallel_tree_initialized) ivp_ov_tree_destroy(&parallel_tree);
    if (serial_tree_initialized) ivp_ov_tree_destroy(&serial_tree);
    free(parallel_elements);
    free(serial_elements);
    ivp_environment_destroy(parallel);
    ivp_environment_destroy(serial);
    IVP_ASSERT_TRUE(ok);
}

typedef struct ballistic_scene {
    ivp_environment_t *environment;
    ivp_object_t *objects[BALLISTIC_OBJECT_COUNT];
} ballistic_scene_t;

static bool ballistic_scene_init(ballistic_scene_t *scene, int worker_count)
{
    memset(scene, 0, sizeof(*scene));
    scene->environment = create_zero_gravity_environment(
        worker_count, ivp_collision_filter_allow_all());
    if (!scene->environment) return false;

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.25);
    object_template.mass = IVP_REAL(1.0);
    object_template.speed_damp_factor = IVP_REAL(0.0);
    object_template.enable_collision_detection = false;
    ivp_vec3r_set(&object_template.rot_speed_damp_factor,
                  IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    for (int i = 0; i < BALLISTIC_OBJECT_COUNT; i++) {
        int x = i % 8;
        int z = i / 8;
        ivp_vec3r_set(&object_template.position,
                      (ivp_real_t)(x * 4),
                      (ivp_real_t)(-z * 3),
                      (ivp_real_t)(z * 5));
        ivp_object_t *object = ivp_environment_create_object(
            scene->environment, &object_template, NULL);
        scene->objects[i] = object;
        if (!object) return false;
        ivp_core_t *core = object->physical_core;
        ivp_vec3r_set(&core->speed,
                      (ivp_real_t)(float)(0.25f + 0.01f * (float)x),
                      (ivp_real_t)(float)(-0.15f - 0.005f * (float)z),
                      (ivp_real_t)(float)(0.05f + 0.002f * (float)i));
        core->rot_speed.k[0] = 0.01f * (float)(1 + (i % 3));
        core->rot_speed.k[1] = -0.008f * (float)(1 + (i % 5));
        core->rot_speed.k[2] = 0.006f * (float)(1 + (i % 7));
    }
    return true;
}

static bool scalar_bits_equal(const void *a, const void *b, size_t size)
{
    return memcmp(a, b, size) == 0;
}

static bool vec3r_bits_equal(const ivp_vec3r_t *a, const ivp_vec3r_t *b)
{
    for (int i = 0; i < 3; i++) {
        if (!scalar_bits_equal(&a->k[i], &b->k[i], sizeof(a->k[i]))) {
            return false;
        }
    }
    return true;
}

static bool vec3f_bits_equal(const ivp_vec3f_t *a, const ivp_vec3f_t *b)
{
    for (int i = 0; i < 3; i++) {
        if (!scalar_bits_equal(&a->k[i], &b->k[i], sizeof(a->k[i]))) {
            return false;
        }
    }
    return true;
}

static bool quatr_bits_equal(const ivp_quatr_t *a, const ivp_quatr_t *b)
{
    return scalar_bits_equal(&a->x, &b->x, sizeof(a->x)) &&
           scalar_bits_equal(&a->y, &b->y, sizeof(a->y)) &&
           scalar_bits_equal(&a->z, &b->z, sizeof(a->z)) &&
           scalar_bits_equal(&a->w, &b->w, sizeof(a->w));
}

static bool quatf_bits_equal(const ivp_quatf_t *a, const ivp_quatf_t *b)
{
    return scalar_bits_equal(&a->x, &b->x, sizeof(a->x)) &&
           scalar_bits_equal(&a->y, &b->y, sizeof(a->y)) &&
           scalar_bits_equal(&a->z, &b->z, sizeof(a->z)) &&
           scalar_bits_equal(&a->w, &b->w, sizeof(a->w));
}

static bool transform_bits_equal(const ivp_transform_r_t *a,
                                 const ivp_transform_r_t *b)
{
    for (int row = 0; row < 3; row++) {
        if (!vec3r_bits_equal(&a->rot.rows[row], &b->rot.rows[row])) {
            return false;
        }
    }
    return vec3r_bits_equal(&a->pos, &b->pos);
}

static const char *exact_core_state_difference(const ivp_core_t *a,
                                               const ivp_core_t *b)
{
#define CORE_SCALAR_DIFFERENCE(field) \
    do { if (a->field != b->field) return #field; } while (0)
#define CORE_BITS_DIFFERENCE(field) \
    do { \
        if (!scalar_bits_equal(&a->field, &b->field, sizeof(a->field))) \
            return #field; \
    } while (0)
#define CORE_VEC3R_DIFFERENCE(field) \
    do { if (!vec3r_bits_equal(&a->field, &b->field)) return #field; } while (0)
#define CORE_VEC3F_DIFFERENCE(field) \
    do { if (!vec3f_bits_equal(&a->field, &b->field)) return #field; } while (0)
#define CORE_QUATR_DIFFERENCE(field) \
    do { if (!quatr_bits_equal(&a->field, &b->field)) return #field; } while (0)
    CORE_SCALAR_DIFFERENCE(fast_piling_allowed);
    CORE_SCALAR_DIFFERENCE(is_in_wakeup_vec);
    CORE_SCALAR_DIFFERENCE(movement_state);
    CORE_SCALAR_DIFFERENCE(temporarily_unmovable);
    CORE_SCALAR_DIFFERENCE(impacts_since_last_PSI);
    CORE_BITS_DIFFERENCE(time_of_last_psi);
    CORE_BITS_DIFFERENCE(i_delta_time);
    CORE_VEC3R_DIFFERENCE(rot_speed_change);
    CORE_VEC3R_DIFFERENCE(speed_change);
    CORE_VEC3F_DIFFERENCE(rot_speed);
    CORE_VEC3R_DIFFERENCE(speed);
    CORE_VEC3R_DIFFERENCE(pos_world_f_core_last_psi);
    CORE_VEC3R_DIFFERENCE(delta_world_f_core_psis);
    CORE_QUATR_DIFFERENCE(q_world_f_core_last_psi);
    CORE_QUATR_DIFFERENCE(q_world_f_core_next_psi);
    if (!transform_bits_equal(&a->m_world_f_core_last_psi,
                              &b->m_world_f_core_last_psi)) {
        return "m_world_f_core_last_psi";
    }
    CORE_VEC3F_DIFFERENCE(rotation_axis_world_space);
    CORE_BITS_DIFFERENCE(current_speed);
    CORE_BITS_DIFFERENCE(abs_omega);
    CORE_BITS_DIFFERENCE(max_surface_rot_speed);
    for (int i = 0; i < 2; i++) {
        if (!scalar_bits_equal(&a->time_of_calm_reference[i],
                               &b->time_of_calm_reference[i],
                               sizeof(a->time_of_calm_reference[i]))) {
            return "time_of_calm_reference";
        }
        if (!quatf_bits_equal(&a->q_world_f_core_calm_reference[i],
                              &b->q_world_f_core_calm_reference[i])) {
            return "q_world_f_core_calm_reference";
        }
        if (!vec3f_bits_equal(&a->position_world_f_core_calm_reference[i],
                              &b->position_world_f_core_calm_reference[i])) {
            return "position_world_f_core_calm_reference";
        }
    }
    CORE_SCALAR_DIFFERENCE(mindist_event_already_done);
#undef CORE_QUATR_DIFFERENCE
#undef CORE_VEC3F_DIFFERENCE
#undef CORE_VEC3R_DIFFERENCE
#undef CORE_BITS_DIFFERENCE
#undef CORE_SCALAR_DIFFERENCE
    return NULL;
}

static bool exact_core_state_equal(const ivp_core_t *a, const ivp_core_t *b)
{
    return exact_core_state_difference(a, b) == NULL;
}

static bool ballistic_scenes_equal(const ballistic_scene_t *a,
                                   const ballistic_scene_t *b)
{
    if (ivp_environment_get_time(a->environment) !=
        ivp_environment_get_time(b->environment)) {
        return false;
    }
    for (int i = 0; i < BALLISTIC_OBJECT_COUNT; i++) {
        if (!exact_core_state_equal(a->objects[i]->physical_core,
                                    b->objects[i]->physical_core) ||
            a->objects[i]->object_movement_state !=
                b->objects[i]->object_movement_state) {
            return false;
        }
    }
    return true;
}

IVP_TEST(core_preparation_is_bit_exact)
{
    ballistic_scene_t serial;
    ballistic_scene_t parallel;
    memset(&serial, 0, sizeof(serial));
    memset(&parallel, 0, sizeof(parallel));
    bool initialized = ballistic_scene_init(&serial, 1) &&
                       ballistic_scene_init(&parallel,
                                            INTERNAL_PARALLEL_WORKERS);
    bool ok = initialized;
    int mismatch_step = -1;
    for (int step = 0; ok && step < 120; step++) {
        ivp_environment_simulate_one_psi(serial.environment);
        ivp_environment_simulate_one_psi(parallel.environment);
        if (!ballistic_scenes_equal(&serial, &parallel)) {
            mismatch_step = step;
            ok = false;
        }
    }

    ivp_parallel_stats_t stats;
    ivp_environment_get_parallel_stats(parallel.environment, &stats);
    bool island_stats_ok;
#ifdef IVP_DEBUG_BUILD
    island_stats_ok = stats.island_dispatches == 0u &&
                      stats.island_tasks == 0u;
#else
    island_stats_ok = stats.island_dispatches >= 120u &&
                      stats.island_tasks >=
                          (uint64_t)(120 * BALLISTIC_OBJECT_COUNT);
#endif
    bool stats_ok = island_stats_ok &&
                    stats.broadphase_dispatches >= 120u &&
                    stats.broadphase_tasks >=
                        (uint64_t)(120 * BALLISTIC_OBJECT_COUNT) &&
                    stats.max_workers_used == INTERNAL_PARALLEL_WORKERS;
    ok = ok && stats_ok;
    if (!ok) {
        int object_index = -1;
        if (initialized && mismatch_step >= 0) {
            for (int i = 0; i < BALLISTIC_OBJECT_COUNT; i++) {
                if (!exact_core_state_equal(
                        serial.objects[i]->physical_core,
                        parallel.objects[i]->physical_core)) {
                    object_index = i;
                    break;
                }
            }
        }
        fprintf(stderr,
                "core prep: initialized=%d mismatch_step=%d object=%d "
                "island_dispatches=%llu island_tasks=%llu "
                "broadphase_dispatches=%llu broadphase_tasks=%llu "
                "max_workers=%d stats_ok=%d\n",
                initialized, mismatch_step, object_index,
                (unsigned long long)stats.island_dispatches,
                (unsigned long long)stats.island_tasks,
                (unsigned long long)stats.broadphase_dispatches,
                (unsigned long long)stats.broadphase_tasks,
                stats.max_workers_used, stats_ok);
        if (object_index >= 0) {
            const ivp_core_t *a = serial.objects[object_index]->physical_core;
            const ivp_core_t *b = parallel.objects[object_index]->physical_core;
            fprintf(stderr,
                    "  pos=(%a,%a,%a)/(%a,%a,%a) "
                    "speed=(%a,%a,%a)/(%a,%a,%a) "
                    "current=%a/%a omega=%a/%a difference=%s\n",
                    (double)a->pos_world_f_core_last_psi.k[0],
                    (double)a->pos_world_f_core_last_psi.k[1],
                    (double)a->pos_world_f_core_last_psi.k[2],
                    (double)b->pos_world_f_core_last_psi.k[0],
                    (double)b->pos_world_f_core_last_psi.k[1],
                    (double)b->pos_world_f_core_last_psi.k[2],
                    (double)a->speed.k[0], (double)a->speed.k[1],
                    (double)a->speed.k[2], (double)b->speed.k[0],
                    (double)b->speed.k[1], (double)b->speed.k[2],
                    (double)a->current_speed, (double)b->current_speed,
                    (double)a->abs_omega, (double)b->abs_omega,
                    exact_core_state_difference(a, b));
            fprintf(stderr, "  calm_time=(%a,%a)/(%a,%a) movement=%u/%u\n",
                    a->time_of_calm_reference[0],
                    a->time_of_calm_reference[1],
                    b->time_of_calm_reference[0],
                    b->time_of_calm_reference[1],
                    (unsigned)a->movement_state,
                    (unsigned)b->movement_state);
        }
    }

    ivp_environment_destroy(parallel.environment);
    ivp_environment_destroy(serial.environment);
    IVP_ASSERT_TRUE(ok);
}

typedef struct independent_constraint_scene {
    ivp_environment_t *environment;
    ivp_object_t **objects;
    ivp_constraint_state_t *constraints;
    ivp_constraint_local_state_t *local_constraints;
} independent_constraint_scene_t;

static bool independent_constraint_scene_init(
    independent_constraint_scene_t *scene, int worker_count)
{
    memset(scene, 0, sizeof(*scene));
    scene->environment = create_zero_gravity_environment(
        worker_count, ivp_collision_filter_allow_all());
    scene->objects = (ivp_object_t **)calloc(
        INDEPENDENT_CONSTRAINT_COUNT, sizeof(*scene->objects));
    scene->constraints = (ivp_constraint_state_t *)calloc(
        INDEPENDENT_CONSTRAINT_COUNT, sizeof(*scene->constraints));
    scene->local_constraints = (ivp_constraint_local_state_t *)calloc(
        INDEPENDENT_CONSTRAINT_COUNT, sizeof(*scene->local_constraints));
    if (!scene->environment || !scene->objects || !scene->constraints ||
        !scene->local_constraints) {
        return false;
    }

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.25);
    object_template.mass = IVP_REAL(1.0);
    object_template.speed_damp_factor = IVP_REAL(0.0);
    object_template.enable_collision_detection = false;
    ivp_vec3r_set_zero(&object_template.rot_speed_damp_factor);
    for (int i = 0; i < INDEPENDENT_CONSTRAINT_COUNT; i++) {
        ivp_vec3r_t anchor = ivp_vec3r(
            (ivp_real_t)(i % 32) * IVP_REAL(2.0),
            (ivp_real_t)(i / 32) * IVP_REAL(2.0), IVP_REAL(0.0));
        object_template.position = anchor;
        scene->objects[i] = ivp_environment_create_object(
            scene->environment, &object_template, NULL);
        if (!scene->objects[i]) return false;
        ivp_controller_handle_t handle;
        if ((i & 1) == 0) {
            ivp_constraint_init_ball_socket_ws(
                &scene->constraints[i], NULL, &anchor, scene->objects[i]);
            handle = ivp_environment_announce_dependent_controller(
                scene->environment, IVP_CP_CONSTRAINTS,
                ivp_controller_constraint, &scene->constraints[i],
                ivp_constraint_discovery);
        } else {
            ivp_vec3r_t object_anchor = ivp_vec3r(
                IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
            ivp_constraint_local_init(
                &scene->local_constraints[i], NULL, &anchor,
                scene->objects[i], &object_anchor);
            for (int axis = 0; axis < 3; axis++) {
                ivp_constraint_local_free_rotation(
                    &scene->local_constraints[i], axis);
            }
            handle = ivp_environment_announce_dependent_controller(
                scene->environment, IVP_CP_CONSTRAINTS,
                ivp_controller_constraint_local,
                &scene->local_constraints[i],
                ivp_constraint_local_discovery);
        }
        if (!ivp_controller_handle_is_valid(handle)) return false;
    }
    return true;
}

static void independent_constraint_scene_drive(
    independent_constraint_scene_t *scene)
{
    for (int i = 0; i < INDEPENDENT_CONSTRAINT_COUNT; i++) {
        ivp_vec3r_set(&scene->objects[i]->physical_core->speed,
                      IVP_REAL(0.25), IVP_REAL(-0.125), IVP_REAL(0.0625));
    }
}

static bool independent_constraint_scenes_equal(
    const independent_constraint_scene_t *a,
    const independent_constraint_scene_t *b)
{
    if (ivp_environment_get_time(a->environment) !=
        ivp_environment_get_time(b->environment)) {
        return false;
    }
    for (int i = 0; i < INDEPENDENT_CONSTRAINT_COUNT; i++) {
        if (!exact_core_state_equal(a->objects[i]->physical_core,
                                    b->objects[i]->physical_core)) {
            return false;
        }
        if ((i & 1) == 0) {
            if (!scalar_bits_equal(
                    &a->constraints[i].accumulated_impulse,
                    &b->constraints[i].accumulated_impulse,
                    sizeof(a->constraints[i].accumulated_impulse)) ||
                a->constraints[i].active != b->constraints[i].active ||
                a->constraints[i].is_broken != b->constraints[i].is_broken) {
                return false;
            }
            for (int axis = 0; axis < 6; axis++) {
                if (!scalar_bits_equal(
                        &a->constraints[i].core.last_impulse[axis],
                        &b->constraints[i].core.last_impulse[axis],
                        sizeof(a->constraints[i].core.last_impulse[axis]))) {
                    return false;
                }
            }
        } else {
            if (!scalar_bits_equal(
                    a->local_constraints[i].accumulated_impulse,
                    b->local_constraints[i].accumulated_impulse,
                    sizeof(a->local_constraints[i].accumulated_impulse)) ||
                a->local_constraints[i].active !=
                    b->local_constraints[i].active ||
                a->local_constraints[i].broken !=
                    b->local_constraints[i].broken) {
                return false;
            }
            for (int axis = 0; axis < 6; axis++) {
                if (!scalar_bits_equal(
                        &a->local_constraints[i].core.last_impulse[axis],
                        &b->local_constraints[i].core.last_impulse[axis],
                        sizeof(a->local_constraints[i]
                                   .core.last_impulse[axis]))) {
                    return false;
                }
            }
        }
    }
    return true;
}

static void independent_constraint_scene_destroy(
    independent_constraint_scene_t *scene)
{
    ivp_environment_destroy(scene->environment);
    free(scene->local_constraints);
    free(scene->constraints);
    free(scene->objects);
}

IVP_TEST(independent_constraint_islands_are_bit_exact)
{
    independent_constraint_scene_t serial;
    independent_constraint_scene_t parallel;
    memset(&serial, 0, sizeof(serial));
    memset(&parallel, 0, sizeof(parallel));
    bool ok = independent_constraint_scene_init(&serial, 1) &&
              independent_constraint_scene_init(
                  &parallel, INTERNAL_PARALLEL_WORKERS);
    int mismatch_step = -1;
    if (ok) {
        independent_constraint_scene_drive(&serial);
        independent_constraint_scene_drive(&parallel);
        ivp_environment_simulate_one_psi(serial.environment);
        ivp_environment_simulate_one_psi(parallel.environment);
        if (!independent_constraint_scenes_equal(&serial, &parallel)) {
            mismatch_step = 0;
            ok = false;
        }
    }
    ivp_parallel_stats_t stats;
    ivp_environment_get_parallel_stats(parallel.environment, &stats);
    uint64_t island_dispatches = stats.island_dispatches;
#ifdef IVP_DEBUG_BUILD
    ok = ok && stats.island_dispatches == 0u && stats.island_tasks == 0u;
#else
    ok = ok && stats.island_dispatches > 0u &&
         stats.island_tasks >= INDEPENDENT_CONSTRAINT_COUNT;
#endif
    if (ok) {
        serial.constraints[0].force_exceed_action = IVP_CFE_BREAK;
        serial.constraints[0].break_force_threshold = IVP_REAL(1.0e10);
        parallel.constraints[0].force_exceed_action = IVP_CFE_BREAK;
        parallel.constraints[0].break_force_threshold = IVP_REAL(1.0e10);
        for (int step = 1; ok && step < 60; step++) {
            independent_constraint_scene_drive(&serial);
            independent_constraint_scene_drive(&parallel);
            ivp_environment_simulate_one_psi(serial.environment);
            ivp_environment_simulate_one_psi(parallel.environment);
            if (!independent_constraint_scenes_equal(&serial, &parallel)) {
                mismatch_step = step;
                ok = false;
            }
        }
        ivp_environment_get_parallel_stats(parallel.environment, &stats);
#ifdef IVP_DEBUG_BUILD
        ok = ok && stats.island_dispatches == island_dispatches;
#else
        ok = ok && stats.island_dispatches > island_dispatches;
#endif
    }
    if (!ok) {
        fprintf(stderr,
                "independent constraints: mismatch_step=%d "
                "dispatches=%llu tasks=%llu max_workers=%d\n",
                mismatch_step,
                (unsigned long long)stats.island_dispatches,
                (unsigned long long)stats.island_tasks,
                stats.max_workers_used);
    }
    independent_constraint_scene_destroy(&parallel);
    independent_constraint_scene_destroy(&serial);
    IVP_ASSERT_TRUE(ok);
}

typedef struct contact_scene {
    ivp_environment_t *environment;
    ivp_object_t *center;
    ivp_object_t *balls[SOLVER_CONTACT_COUNT];
    ivp_vec3r_t inward_speeds[SOLVER_CONTACT_COUNT];
} contact_scene_t;

static bool contact_scene_init(contact_scene_t *scene, int worker_count)
{
    memset(scene, 0, sizeof(*scene));
    scene->environment = create_zero_gravity_environment(
        worker_count, ivp_collision_filter_allow_all());
    if (!scene->environment) return false;

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(5.0);
    object_template.mass = IVP_REAL(1000.0);
    object_template.material.friction = IVP_REAL(0.8);
    object_template.material.elasticity = IVP_REAL(0.0);
    object_template.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set_zero(&object_template.rot_speed_damp_factor);
    scene->center = ivp_environment_create_object(
        scene->environment, &object_template, NULL);
    if (!scene->center) return false;

    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.25);
    object_template.mass = IVP_REAL(1.0);
    object_template.material.friction = IVP_REAL(0.8);
    object_template.material.elasticity = IVP_REAL(0.0);
    object_template.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set_zero(&object_template.rot_speed_damp_factor);
    const double golden_angle = 2.39996322972865332;
    for (int i = 0; i < SOLVER_CONTACT_COUNT; i++) {
        double y = 1.0 - 2.0 * ((double)i + 0.5) /
                             (double)SOLVER_CONTACT_COUNT;
        double radial = sqrt(1.0 - y * y);
        double angle = golden_angle * (double)i;
        double x = cos(angle) * radial;
        double z = sin(angle) * radial;
        ivp_vec3r_set(&object_template.position,
                      (ivp_real_t)(5.30 * x),
                      (ivp_real_t)(5.30 * y),
                      (ivp_real_t)(5.30 * z));
        ivp_vec3r_set(&scene->inward_speeds[i],
                      (ivp_real_t)(-2.0 * x),
                      (ivp_real_t)(-2.0 * y),
                      (ivp_real_t)(-2.0 * z));
        scene->balls[i] = ivp_environment_create_object(
            scene->environment, &object_template, NULL);
        if (!scene->balls[i]) return false;
        scene->balls[i]->physical_core->speed = scene->inward_speeds[i];
    }
    return true;
}

static void contact_scene_drive_inward(contact_scene_t *scene)
{
    for (int i = 0; i < SOLVER_CONTACT_COUNT; i++) {
        scene->balls[i]->physical_core->speed = scene->inward_speeds[i];
    }
}

static bool contact_scenes_equal(const contact_scene_t *a,
                                 const contact_scene_t *b)
{
    if (ivp_environment_get_time(a->environment) !=
        ivp_environment_get_time(b->environment)) {
        return false;
    }
    if (!exact_core_state_equal(a->center->physical_core,
                                b->center->physical_core) ||
        a->center->object_movement_state !=
            b->center->object_movement_state) {
        return false;
    }
    for (int i = 0; i < SOLVER_CONTACT_COUNT; i++) {
        if (!exact_core_state_equal(a->balls[i]->physical_core,
                                    b->balls[i]->physical_core) ||
            a->balls[i]->object_movement_state !=
                b->balls[i]->object_movement_state) {
            return false;
        }
    }
    return true;
}

IVP_TEST(solver_matrix_is_bit_exact)
{
    contact_scene_t serial;
    contact_scene_t parallel;
    memset(&serial, 0, sizeof(serial));
    memset(&parallel, 0, sizeof(parallel));
    bool ok = contact_scene_init(&serial, 1) &&
              contact_scene_init(&parallel, INTERNAL_PARALLEL_WORKERS);
    for (int step = 0; ok && step < 60; step++) {
        contact_scene_drive_inward(&serial);
        contact_scene_drive_inward(&parallel);
        ivp_environment_simulate_one_psi(serial.environment);
        ivp_environment_simulate_one_psi(parallel.environment);
        if (!contact_scenes_equal(&serial, &parallel)) ok = false;
    }

    ivp_parallel_stats_t stats;
    ivp_environment_get_parallel_stats(parallel.environment, &stats);
    ok = ok && stats.solver_dispatches > 0u &&
         stats.solver_tasks >= SOLVER_CONTACT_COUNT &&
         stats.max_workers_used == INTERNAL_PARALLEL_WORKERS;
    if (!ok) {
        fprintf(stderr,
                "solver: dispatches=%llu tasks=%llu max_workers=%d\n",
                (unsigned long long)stats.solver_dispatches,
                (unsigned long long)stats.solver_tasks,
                stats.max_workers_used);
    }

    ivp_environment_destroy(parallel.environment);
    ivp_environment_destroy(serial.environment);
    IVP_ASSERT_TRUE(ok);
}

int main(void)
{
    IVP_REGISTER(task_pool_runs_concurrently);
    IVP_REGISTER(task_pool_claims_every_task_exactly_once);
    IVP_REGISTER(broadphase_preserves_candidate_order);
    IVP_REGISTER(core_preparation_is_bit_exact);
    IVP_REGISTER(independent_constraint_islands_are_bit_exact);
    IVP_REGISTER(solver_matrix_is_bit_exact);
    IVP_RUN_ALL();
}
