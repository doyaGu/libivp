/* test_internal_state_stability.c -- internal-layout perturbation regressions. */
#include "ivp_test.h"

#include "dynamics/ivp_env_pipeline.h"
#include "dynamics/ivp_sim_internal.h"

#include <ivp/builder/ivp_build_surface.h>
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/geometry/ivp_surman.h>

#include <stdint.h>
#include <string.h>

#define STRONG_ISLAND_COUNT 6
#define STRONG_OBJECTS_PER_ISLAND 4
#define STRONG_OBJECT_COUNT \
    (STRONG_ISLAND_COUNT * STRONG_OBJECTS_PER_ISLAND)
#define STRONG_POOL_WARMUP_COUNT STRONG_OBJECT_COUNT
#define STRONG_PARALLEL_OBJECT_COUNT 256
#define STRONG_PARALLEL_WORKERS 4

enum strong_perturbation {
    STRONG_PERTURB_CLIENT_POINTERS = 1u << 0,
    STRONG_PERTURB_OV_TREE = 1u << 1,
    STRONG_PERTURB_SIM_UNITS = 1u << 2,
    STRONG_PERTURB_ARENAS = 1u << 3,
    STRONG_PERTURB_ALL = (1u << 4) - 1u
};

typedef struct strong_scene {
    ivp_environment_t *environment;
    ivp_object_t *objects[STRONG_OBJECT_COUNT];
    uintptr_t client_tokens[STRONG_OBJECT_COUNT];
} strong_scene_t;

#define STRONG_CUBE_COUNT 16
#define STRONG_CUBE_OBJECT_COUNT (STRONG_CUBE_COUNT + 1)

typedef struct strong_cube_scene {
    ivp_environment_t *environment;
    ivp_object_t *objects[STRONG_CUBE_OBJECT_COUNT];
    ivp_compact_surface_t *ground_surface;
    ivp_surman_polygon_t ground_surman;
    ivp_compact_surface_t *cube_surfaces[STRONG_CUBE_COUNT];
    ivp_surman_polygon_t cube_surmans[STRONG_CUBE_COUNT];
    bool shared_surface;
} strong_cube_scene_t;

static ivp_environment_t *strong_environment_create(int worker_count)
{
    ivp_environment_config_t config;
    ivp_environment_config_init(&config);
    config.canonical_internal_state = true;
    config.parallel_worker_count = worker_count;
    config.disable_performance_counter = true;
    ivp_vec3r_set_zero(&config.gravity);
    return ivp_environment_create(&config);
}

static ivp_environment_t *strong_gravity_environment_create(void)
{
    ivp_environment_config_t config;
    ivp_environment_config_init(&config);
    config.canonical_internal_state = true;
    config.disable_performance_counter = true;
    return ivp_environment_create(&config);
}

static void reverse_pvec(ivp_pvec_t *vector)
{
    int first = 0;
    int last = ivp_pvec_len(vector) - 1;
    while (first < last) {
        ivp_pvec_swap(vector, first, last);
        first++;
        last--;
    }
}

static void scramble_ov_node(ivp_ov_node_t *node)
{
    if (!node) return;
    reverse_pvec(&node->elements);
    reverse_pvec(&node->children);
    for (int i = 0; i < ivp_pvec_len(&node->children); i++) {
        scramble_ov_node((ivp_ov_node_t *)ivp_pvec_at(&node->children, i));
    }
}

static void reverse_sim_unit_slot(ivp_sim_unit_t **head)
{
    ivp_sim_unit_t *node = *head;
    ivp_sim_unit_t *new_head = NULL;
    while (node) {
        ivp_sim_unit_t *next = node->next_sim_unit;
        node->next_sim_unit = node->prev_sim_unit;
        node->prev_sim_unit = next;
        new_head = node;
        node = next;
    }
    *head = new_head;
}

static void scramble_sim_unit_lists(ivp_environment_t *environment)
{
    ivp_sim_units_manager_t *manager =
        &environment->pl->sim_units_manager;
    reverse_sim_unit_slot(&manager->sim_units_slot0);
    reverse_sim_unit_slot(&manager->still_slot);
}

static void reset_arena_without_writing(ivp_u_memory_t *arena)
{
    ivp_u_memory_start_transaction(arena);
    ivp_u_memory_end_transaction(arena);
}

static bool poison_arena(ivp_u_memory_t *arena, size_t bytes,
                         unsigned char value)
{
    ivp_u_memory_start_transaction(arena);
    void *storage = ivp_u_memory_get_mem(arena, bytes);
    if (storage) memset(storage, value, bytes);
    ivp_u_memory_end_transaction(arena);
    return storage != NULL;
}

static bool strong_scene_init(strong_scene_t *scene, unsigned perturbations,
                              int pool_warmup_count)
{
    memset(scene, 0, sizeof(*scene));
    scene->environment = strong_environment_create(1);
    if (!scene->environment) return false;

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.65);
    object_template.mass = IVP_REAL(1.0);
    object_template.material.friction = IVP_REAL(0.8);
    object_template.material.elasticity = IVP_REAL(0.0);
    object_template.speed_damp_factor = IVP_REAL(0.0);
    object_template.enable_collision_detection = false;
    ivp_vec3r_set_zero(&object_template.rot_speed_damp_factor);

    ivp_object_t *warmup_objects[STRONG_POOL_WARMUP_COUNT] = {0};
    if (pool_warmup_count < 0 ||
        pool_warmup_count > STRONG_POOL_WARMUP_COUNT) {
        return false;
    }
    object_template.start_in_simulation = false;
    for (int i = 0; i < pool_warmup_count; i++) {
        warmup_objects[i] = ivp_environment_create_object(
            scene->environment, &object_template, NULL);
        if (!warmup_objects[i]) return false;
    }
    for (int i = 0; i < pool_warmup_count; i++) {
        ivp_environment_destroy_object(scene->environment,
                                       warmup_objects[i]);
    }
    object_template.start_in_simulation = true;

    static const double offsets[STRONG_OBJECTS_PER_ISLAND][2] = {
        { -0.5, -0.5 }, { 0.5, -0.5 },
        { 0.5, 0.5 }, { -0.5, 0.5 }
    };
    for (int island = 0; island < STRONG_ISLAND_COUNT; island++) {
        for (int local = 0; local < STRONG_OBJECTS_PER_ISLAND; local++) {
            int index = island * STRONG_OBJECTS_PER_ISLAND + local;
            ivp_vec3r_set(&object_template.position,
                          (ivp_real_t)((double)island * 8.0 +
                                       offsets[local][0]),
                          (ivp_real_t)(0.125 * (double)(local & 1)),
                          (ivp_real_t)offsets[local][1]);
            int token_index =
                (perturbations & STRONG_PERTURB_CLIENT_POINTERS)
                    ? STRONG_OBJECT_COUNT - 1 - index
                    : index;
            object_template.client_data = &scene->client_tokens[token_index];
            scene->objects[index] = ivp_environment_create_object(
                scene->environment, &object_template, NULL);
            if (!scene->objects[index]) return false;
            ivp_vec3r_set(&scene->objects[index]->physical_core->speed,
                          (ivp_real_t)(-0.15 * offsets[local][0]),
                          (ivp_real_t)(0.01 * (double)(local - 1)),
                          (ivp_real_t)(-0.15 * offsets[local][1]));
        }
    }

    for (int i = 0; i < STRONG_OBJECT_COUNT; i++) {
        if (perturbations & STRONG_PERTURB_OV_TREE) {
            scramble_ov_node(scene->environment->pl->ov_tree.root);
        }
        ivp_object_enable_collision_detection(scene->objects[i], true);
    }
    return true;
}

static bool real_bits_equal(ivp_real_t a, ivp_real_t b)
{
    return memcmp(&a, &b, sizeof(a)) == 0;
}

static bool float_bits_equal(float a, float b)
{
    return memcmp(&a, &b, sizeof(a)) == 0;
}

static bool vec3r_bits_equal(const ivp_vec3r_t *a,
                             const ivp_vec3r_t *b)
{
    return real_bits_equal(a->k[0], b->k[0]) &&
           real_bits_equal(a->k[1], b->k[1]) &&
           real_bits_equal(a->k[2], b->k[2]);
}

static bool vec3f_bits_equal(const ivp_vec3f_t *a,
                             const ivp_vec3f_t *b)
{
    return float_bits_equal(a->k[0], b->k[0]) &&
           float_bits_equal(a->k[1], b->k[1]) &&
           float_bits_equal(a->k[2], b->k[2]);
}

static bool quatr_bits_equal(const ivp_quatr_t *a,
                             const ivp_quatr_t *b)
{
    return real_bits_equal(a->x, b->x) &&
           real_bits_equal(a->y, b->y) &&
           real_bits_equal(a->z, b->z) &&
           real_bits_equal(a->w, b->w);
}

static bool visible_core_state_equal(const ivp_core_t *a,
                                     const ivp_core_t *b)
{
    return vec3r_bits_equal(&a->pos_world_f_core_last_psi,
                            &b->pos_world_f_core_last_psi) &&
           quatr_bits_equal(&a->q_world_f_core_last_psi,
                            &b->q_world_f_core_last_psi) &&
           quatr_bits_equal(&a->q_world_f_core_next_psi,
                            &b->q_world_f_core_next_psi) &&
           vec3r_bits_equal(&a->speed, &b->speed) &&
           vec3f_bits_equal(&a->rot_speed, &b->rot_speed) &&
           a->movement_state == b->movement_state;
}

static bool strong_scenes_equal(const strong_scene_t *a,
                                const strong_scene_t *b)
{
    if (ivp_environment_get_time(a->environment) !=
        ivp_environment_get_time(b->environment)) {
        return false;
    }
    for (int i = 0; i < STRONG_OBJECT_COUNT; i++) {
        if (!visible_core_state_equal(a->objects[i]->physical_core,
                                      b->objects[i]->physical_core) ||
            a->objects[i]->object_movement_state !=
                b->objects[i]->object_movement_state) {
            return false;
        }
    }
    return true;
}

static void report_strong_scene_difference(const strong_scene_t *a,
                                           const strong_scene_t *b)
{
    for (int i = 0; i < STRONG_OBJECT_COUNT; i++) {
        const ivp_core_t *ca = a->objects[i]->physical_core;
        const ivp_core_t *cb = b->objects[i]->physical_core;
        if (visible_core_state_equal(ca, cb) &&
            a->objects[i]->object_movement_state ==
                b->objects[i]->object_movement_state) {
            continue;
        }
        fprintf(stderr,
                "object %d pos=(%a,%a,%a)/(%a,%a,%a) "
                "speed=(%a,%a,%a)/(%a,%a,%a) movement=%u/%u\n",
                i,
                (double)ca->pos_world_f_core_last_psi.k[0],
                (double)ca->pos_world_f_core_last_psi.k[1],
                (double)ca->pos_world_f_core_last_psi.k[2],
                (double)cb->pos_world_f_core_last_psi.k[0],
                (double)cb->pos_world_f_core_last_psi.k[1],
                (double)cb->pos_world_f_core_last_psi.k[2],
                (double)ca->speed.k[0], (double)ca->speed.k[1],
                (double)ca->speed.k[2], (double)cb->speed.k[0],
                (double)cb->speed.k[1], (double)cb->speed.k[2],
                (unsigned)ca->movement_state,
                (unsigned)cb->movement_state);
        return;
    }
}

static bool strong_cube_scene_init(strong_cube_scene_t *scene,
                                   bool shared_surface)
{
    memset(scene, 0, sizeof(*scene));
    scene->shared_surface = shared_surface;
    scene->environment = strong_gravity_environment_create();
    if (!scene->environment) return false;

    scene->ground_surface =
        ivp_build_box_surface(NULL, 20.0f, 0.5f, 20.0f);
    if (!scene->ground_surface) return false;
    ivp_surman_polygon_init(&scene->ground_surman,
                            scene->ground_surface);

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_POLYGON;
    object_template.mass = IVP_REAL(1.0e6);
    ivp_vec3r_set(&object_template.rot_inertia,
                  IVP_REAL(1.0e6), IVP_REAL(1.0e6), IVP_REAL(1.0e6));
    object_template.rot_inertia_is_factor = false;
    object_template.material.friction = IVP_REAL(0.8);
    object_template.material.elasticity = IVP_REAL(0.0);
    object_template.physical_unmoveable = true;
    ivp_vec3r_set(&object_template.position,
                  IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    scene->objects[0] = ivp_environment_create_object(
        scene->environment, &object_template,
        &scene->ground_surman.base);
    if (!scene->objects[0]) return false;

    if (shared_surface) {
        scene->cube_surfaces[0] =
            ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
        if (!scene->cube_surfaces[0]) return false;
        ivp_surman_polygon_init(&scene->cube_surmans[0],
                                scene->cube_surfaces[0]);
    }

    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_POLYGON;
    object_template.mass = IVP_REAL(0.6);
    ivp_vec3r_set(&object_template.rot_inertia,
                  IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    object_template.rot_inertia_is_factor = false;
    object_template.material.friction = IVP_REAL(0.6);
    object_template.material.elasticity = IVP_REAL(0.05);
    object_template.speed_damp_factor = IVP_REAL(0.02);
    ivp_vec3r_set_zero(&object_template.rot_speed_damp_factor);

    int object_index = 1;
    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
            int cube_index = object_index - 1;
            if (!shared_surface) {
                scene->cube_surfaces[cube_index] =
                    ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
                if (!scene->cube_surfaces[cube_index]) return false;
                ivp_surman_polygon_init(
                    &scene->cube_surmans[cube_index],
                    scene->cube_surfaces[cube_index]);
            }
            ivp_vec3r_set(&object_template.position,
                          (ivp_real_t)(-2.25 + (double)x * 1.5),
                          (ivp_real_t)(-2.0 - (double)y * 1.1),
                          IVP_REAL(0.0));
            int surman_index = shared_surface ? 0 : cube_index;
            scene->objects[object_index] =
                ivp_environment_create_object(
                    scene->environment, &object_template,
                    &scene->cube_surmans[surman_index].base);
            if (!scene->objects[object_index]) return false;
            object_index++;
        }
    }
    return true;
}

static void strong_cube_scene_destroy(strong_cube_scene_t *scene)
{
    if (!scene) return;
    ivp_environment_destroy(scene->environment);
    if (scene->shared_surface) {
        if (scene->cube_surfaces[0]) {
            ivp_compact_surface_free(NULL, scene->cube_surfaces[0]);
        }
    } else {
        for (int i = 0; i < STRONG_CUBE_COUNT; i++) {
            if (scene->cube_surfaces[i]) {
                ivp_compact_surface_free(NULL, scene->cube_surfaces[i]);
            }
        }
    }
    if (scene->ground_surface) {
        ivp_compact_surface_free(NULL, scene->ground_surface);
    }
}

static bool strong_cube_scenes_equal(const strong_cube_scene_t *a,
                                     const strong_cube_scene_t *b)
{
    if (ivp_environment_get_time(a->environment) !=
        ivp_environment_get_time(b->environment)) {
        return false;
    }
    for (int i = 0; i < STRONG_CUBE_OBJECT_COUNT; i++) {
        if (!visible_core_state_equal(a->objects[i]->physical_core,
                                      b->objects[i]->physical_core) ||
            a->objects[i]->object_movement_state !=
                b->objects[i]->object_movement_state) {
            return false;
        }
    }
    return true;
}

static bool run_perturbation(unsigned perturbations)
{
    strong_scene_t canonical = {0};
    strong_scene_t perturbed = {0};
    bool ok = strong_scene_init(&canonical, 0u, 0) &&
              strong_scene_init(&perturbed, perturbations, 0);

    for (int step = 0; ok && step < 32; step++) {
        if (perturbations & STRONG_PERTURB_ARENAS) {
            reset_arena_without_writing(
                &canonical.environment->pl->short_term_mem);
            reset_arena_without_writing(
                &canonical.environment->pl->sim_unit_mem.arena);
            ok = poison_arena(&perturbed.environment->pl->short_term_mem,
                              0x6000u,
                              (unsigned char)(0x5au ^ (unsigned char)step)) &&
                 poison_arena(
                     &perturbed.environment->pl->sim_unit_mem.arena,
                     0x1000u,
                     (unsigned char)(0xa5u ^ (unsigned char)step));
        }
        if (perturbations & STRONG_PERTURB_SIM_UNITS) {
            scramble_sim_unit_lists(perturbed.environment);
        }
        if (perturbations & STRONG_PERTURB_OV_TREE) {
            scramble_ov_node(perturbed.environment->pl->ov_tree.root);
        }

        ivp_environment_simulate_one_psi(canonical.environment);
        ivp_environment_simulate_one_psi(perturbed.environment);
        if (!strong_scenes_equal(&canonical, &perturbed)) {
            fprintf(stderr,
                    "visible state diverged at step %d (perturbations=%u)\n",
                    step, perturbations);
            report_strong_scene_difference(&canonical, &perturbed);
            ok = false;
        }
    }

    ivp_environment_destroy(perturbed.environment);
    ivp_environment_destroy(canonical.environment);
    return ok;
}

IVP_TEST(pool_history_does_not_change_visible_state)
{
    strong_scene_t canonical = {0};
    strong_scene_t warmed = {0};
    bool ok = strong_scene_init(&canonical, 0u, 0) &&
              strong_scene_init(&warmed, 0u,
                                STRONG_POOL_WARMUP_COUNT);
    bool handle_slots_changed = false;
    for (int i = 0; ok && i < STRONG_OBJECT_COUNT; i++) {
        handle_slots_changed = handle_slots_changed ||
            canonical.objects[i]->handle.slot !=
                warmed.objects[i]->handle.slot;
        if (canonical.objects[i]->creation_sequence !=
            warmed.objects[i]->creation_sequence) {
            ok = false;
        }
    }
    ok = ok && handle_slots_changed;
    for (int step = 0; ok && step < 32; step++) {
        ivp_environment_simulate_one_psi(canonical.environment);
        ivp_environment_simulate_one_psi(warmed.environment);
        if (!strong_scenes_equal(&canonical, &warmed)) {
            fprintf(stderr,
                    "pool history changed visible state at step %d\n",
                    step);
            ok = false;
        }
    }
    ivp_environment_destroy(warmed.environment);
    ivp_environment_destroy(canonical.environment);
    IVP_ASSERT_TRUE(ok);
}

typedef struct strong_parallel_scene {
    ivp_environment_t *environment;
    ivp_object_t *objects[STRONG_PARALLEL_OBJECT_COUNT];
} strong_parallel_scene_t;

static bool strong_parallel_scene_init(strong_parallel_scene_t *scene,
                                       int worker_count)
{
    memset(scene, 0, sizeof(*scene));
    scene->environment = strong_environment_create(worker_count);
    if (!scene->environment) return false;

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.25);
    object_template.mass = IVP_REAL(1.0);
    object_template.speed_damp_factor = IVP_REAL(0.0);
    object_template.enable_collision_detection = false;
    ivp_vec3r_set_zero(&object_template.rot_speed_damp_factor);
    for (int i = 0; i < STRONG_PARALLEL_OBJECT_COUNT; i++) {
        ivp_vec3r_set(&object_template.position,
                      (ivp_real_t)((i % 16) * 4),
                      (ivp_real_t)(-(i / 16) * 4),
                      (ivp_real_t)((i % 7) * 3));
        scene->objects[i] = ivp_environment_create_object(
            scene->environment, &object_template, NULL);
        if (!scene->objects[i]) return false;
        ivp_vec3r_set(&scene->objects[i]->physical_core->speed,
                      (ivp_real_t)(0.1 + (double)(i % 5) * 0.01),
                      (ivp_real_t)(-0.03 - (double)(i % 3) * 0.01),
                      (ivp_real_t)(0.02 + (double)(i % 11) * 0.002));
    }
    return true;
}

static bool strong_parallel_scenes_equal(
    const strong_parallel_scene_t *a,
    const strong_parallel_scene_t *b)
{
    if (ivp_environment_get_time(a->environment) !=
        ivp_environment_get_time(b->environment)) {
        return false;
    }
    for (int i = 0; i < STRONG_PARALLEL_OBJECT_COUNT; i++) {
        if (!visible_core_state_equal(a->objects[i]->physical_core,
                                      b->objects[i]->physical_core) ||
            a->objects[i]->object_movement_state !=
                b->objects[i]->object_movement_state) {
            return false;
        }
    }
    return true;
}

IVP_TEST(worker_count_does_not_change_canonical_state)
{
    strong_parallel_scene_t serial = {0};
    strong_parallel_scene_t parallel = {0};
    bool ok = strong_parallel_scene_init(&serial, 1) &&
              strong_parallel_scene_init(&parallel,
                                         STRONG_PARALLEL_WORKERS);
    for (int step = 0; ok && step < 8; step++) {
        ivp_environment_simulate_one_psi(serial.environment);
        ivp_environment_simulate_one_psi(parallel.environment);
        if (!strong_parallel_scenes_equal(&serial, &parallel)) {
            fprintf(stderr,
                    "worker count changed canonical state at step %d\n",
                    step);
            ok = false;
        }
    }
    ivp_parallel_stats_t stats;
    ivp_environment_get_parallel_stats(parallel.environment, &stats);
#ifdef IVP_DEBUG_BUILD
    /* Debug consistency and draw hooks keep every controller on the owning
     * thread by design; the state comparison still covers 1-vs-4 config. */
    bool stats_ok = stats.island_dispatches == 0u &&
                    stats.island_tasks == 0u &&
                    stats.max_workers_used == 0;
#else
    bool stats_ok = stats.island_dispatches > 0u &&
                    stats.island_tasks >= STRONG_PARALLEL_OBJECT_COUNT &&
                    stats.max_workers_used == STRONG_PARALLEL_WORKERS;
#endif
    if (!stats_ok) {
        fprintf(stderr,
                "parallel stats: dispatches=%llu tasks=%llu max_workers=%d\n",
                (unsigned long long)stats.island_dispatches,
                (unsigned long long)stats.island_tasks,
                stats.max_workers_used);
    }
    ok = ok && stats_ok;
    ivp_environment_destroy(parallel.environment);
    ivp_environment_destroy(serial.environment);
    IVP_ASSERT_TRUE(ok);
}

IVP_TEST(arena_history_does_not_change_visible_state)
{
    IVP_ASSERT_TRUE(run_perturbation(STRONG_PERTURB_ARENAS));
}

IVP_TEST(sim_unit_list_order_does_not_change_visible_state)
{
    IVP_ASSERT_TRUE(run_perturbation(STRONG_PERTURB_SIM_UNITS));
}

IVP_TEST(ov_tree_order_does_not_change_visible_state)
{
    IVP_ASSERT_TRUE(run_perturbation(STRONG_PERTURB_OV_TREE));
}

IVP_TEST(client_pointer_order_does_not_change_visible_state)
{
    IVP_ASSERT_TRUE(run_perturbation(STRONG_PERTURB_CLIENT_POINTERS));
}

IVP_TEST(combined_internal_perturbations_do_not_change_visible_state)
{
    bool ok = run_perturbation(STRONG_PERTURB_ALL);
    IVP_ASSERT_TRUE(ok);
}

IVP_TEST(relocated_and_shared_geometry_do_not_change_visible_state)
{
    strong_cube_scene_t relocated = {0};
    strong_cube_scene_t shared = {0};
    bool ok = strong_cube_scene_init(&relocated, false) &&
              strong_cube_scene_init(&shared, true);
    for (int step = 0; ok && step < 180; step++) {
        ivp_environment_simulate_one_psi(relocated.environment);
        ivp_environment_simulate_one_psi(shared.environment);
        if (!strong_cube_scenes_equal(&relocated, &shared)) {
            fprintf(stderr,
                    "geometry relocation changed visible state at step %d\n",
                    step);
            ok = false;
        }
    }
    strong_cube_scene_destroy(&shared);
    strong_cube_scene_destroy(&relocated);
    IVP_ASSERT_TRUE(ok);
}

int main(void)
{
    IVP_REGISTER(arena_history_does_not_change_visible_state);
    IVP_REGISTER(sim_unit_list_order_does_not_change_visible_state);
    IVP_REGISTER(ov_tree_order_does_not_change_visible_state);
    IVP_REGISTER(client_pointer_order_does_not_change_visible_state);
    IVP_REGISTER(combined_internal_perturbations_do_not_change_visible_state);
    IVP_REGISTER(pool_history_does_not_change_visible_state);
    IVP_REGISTER(worker_count_does_not_change_canonical_state);
    IVP_REGISTER(relocated_and_shared_geometry_do_not_change_visible_state);
    IVP_RUN_ALL();
}
