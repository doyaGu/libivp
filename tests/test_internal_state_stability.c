/* test_internal_state_stability.c -- internal-layout perturbation regressions. */
#include "ivp_test.h"

#include "dynamics/ivp_env_pipeline.h"
#include "dynamics/ivp_sim_internal.h"
#include "solver/ivp_friction_internal.h"

#include <ivp/builder/ivp_build_surface.h>
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_merge_core.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_phantom.h>
#include <ivp/dynamics/ivp_universe_manager.h>
#include <ivp/events/ivp_listener_object.h>
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
#define STRONG_REVIVE_OBJECT_COUNT 8
#define STRONG_CLUSTER_OBJECT_COUNT 6
#define STRONG_CLUSTER_VISIT_CAPACITY 128
#define STRONG_PHANTOM_CORE_COUNT 8

enum strong_perturbation {
    STRONG_PERTURB_CLIENT_POINTERS = 1u << 0,
    STRONG_PERTURB_OV_TREE = 1u << 1,
    STRONG_PERTURB_SIM_UNITS = 1u << 2,
    STRONG_PERTURB_ARENAS = 1u << 3,
    STRONG_PERTURB_CONTACT_LISTS = 1u << 4,
    STRONG_PERTURB_MINDIST_MANAGER = 1u << 5,
    STRONG_PERTURB_OBJECT_MINDISTS = 1u << 6,
    STRONG_PERTURB_ALL = (1u << 7) - 1u
};

typedef struct strong_scene {
    ivp_environment_t *environment;
    ivp_object_t *objects[STRONG_OBJECT_COUNT];
    uintptr_t client_tokens[STRONG_OBJECT_COUNT];
    int controller_operations[2];
    ivp_controller_handle_t controller_handles[2];
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

#define STRONG_COMPOUND_PART_COUNT 3

typedef struct strong_compound_scene {
    ivp_environment_t *environment;
    ivp_object_t *ground;
    ivp_object_t *parts[STRONG_COMPOUND_PART_COUNT];
} strong_compound_scene_t;

static ivp_environment_t *strong_environment_create(int worker_count)
{
    ivp_environment_config_t config;
    ivp_environment_config_init(&config);
    config.parallel_worker_count = worker_count;
    config.disable_performance_counter = true;
    ivp_vec3r_set_zero(&config.gravity);
    return ivp_environment_create(&config);
}

static ivp_environment_t *strong_gravity_environment_create(void)
{
    ivp_environment_config_t config;
    ivp_environment_config_init(&config);
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

static void reverse_cluster_tree(ivp_cluster_t *cluster)
{
    for (ivp_object_t *child = cluster->first_child; child;) {
        ivp_object_t *next = child->next_in_cluster;
        if (child->type == IVP_OBJECT_CLUSTER) {
            reverse_cluster_tree((ivp_cluster_t *)child);
        }
        child = next;
    }

    ivp_object_t *child = cluster->first_child;
    ivp_object_t *new_head = NULL;
    while (child) {
        ivp_object_t *next = child->next_in_cluster;
        child->next_in_cluster = child->prev_in_cluster;
        child->prev_in_cluster = next;
        new_head = child;
        child = next;
    }
    cluster->first_child = new_head;
}

static void set_phantom_hash_order(
    ivp_set_t *set,
    ivp_object_t *const objects[STRONG_PHANTOM_CORE_COUNT],
    bool reverse)
{
    ivp_vhash_t *hash = &set->hash;
    int capacity = ivp_vhash_capacity(hash);
    memset(hash->elems, 0,
           (size_t)capacity * sizeof(hash->elems[0]));
    for (int i = 0; i < STRONG_PHANTOM_CORE_COUNT; i++) {
        int object_index = reverse
                               ? STRONG_PHANTOM_CORE_COUNT - 1 - i
                               : i;
        hash->elems[i].elem = objects[object_index]->physical_core;
    }
    hash->nelems = STRONG_PHANTOM_CORE_COUNT;
}

static void scramble_sim_unit_contents(ivp_sim_unit_t *unit)
{
    reverse_pvec(&unit->sim_unit_cores);
    reverse_pvec(&unit->controller_cores);
    for (int i = 0; i < ivp_pvec_len(&unit->controller_cores); i++) {
        ivp_su_ctrl_list_t *controller =
            (ivp_su_ctrl_list_t *)ivp_pvec_at(&unit->controller_cores, i);
        reverse_pvec(&controller->cores_controlled_by);
    }
    for (int i = 0; i < ivp_pvec_len(&unit->sim_unit_cores); i++) {
        ivp_core_t *core =
            (ivp_core_t *)ivp_pvec_at(&unit->sim_unit_cores, i);
        reverse_pvec(&core->controllers_of_core);
    }
}

static void scramble_sim_unit_slot_contents(ivp_sim_unit_t *head)
{
    for (ivp_sim_unit_t *unit = head; unit;
         unit = unit->next_sim_unit) {
        scramble_sim_unit_contents(unit);
    }
}

static void scramble_sim_unit_lists(ivp_environment_t *environment)
{
    ivp_sim_units_manager_t *manager =
        &environment->pl->sim_units_manager;
    reverse_sim_unit_slot(&manager->sim_units_slot0);
    reverse_sim_unit_slot(&manager->still_slot);
    scramble_sim_unit_slot_contents(manager->sim_units_slot0);
    scramble_sim_unit_slot_contents(manager->still_slot);
}

static void reverse_contact_list(ivp_fr_system_t *system)
{
    ivp_contact_pt_t *contact = system->first_friction_dist;
    ivp_contact_pt_t *new_head = NULL;
    while (contact) {
        ivp_contact_pt_t *next = contact->next_dist_in_friction;
        contact->next_dist_in_friction = contact->prev_dist_in_friction;
        contact->prev_dist_in_friction = next;
        new_head = contact;
        contact = next;
    }
    system->first_friction_dist = new_head;
}

static void reverse_mindist_list(ivp_mindist_t **head)
{
    ivp_mindist_t *mindist = *head;
    ivp_mindist_t *new_head = NULL;
    while (mindist) {
        ivp_mindist_t *next = mindist->next;
        mindist->next = mindist->prev;
        mindist->prev = next;
        new_head = mindist;
        mindist = next;
    }
    *head = new_head;
}

static void reverse_object_mindist_list(ivp_md_synapse_t **head)
{
    ivp_md_synapse_t *synapse = *head;
    ivp_md_synapse_t *new_head = NULL;
    while (synapse) {
        ivp_md_synapse_t *next = synapse->next;
        synapse->next = synapse->prev;
        synapse->prev = next;
        new_head = synapse;
        synapse = next;
    }
    *head = new_head;
}

static void scramble_mindist_manager(ivp_environment_t *environment)
{
    ivp_mindist_mgr_t *manager = &environment->pl->mindist_mgr;
    reverse_mindist_list(&manager->exact_mindists);
    reverse_mindist_list(&manager->invalid_mindists);
    reverse_pvec(&manager->wheel_look_ahead_mindists);
}

static void scramble_object_mindists(ivp_object_t *const *objects,
                                     int object_count)
{
    for (int i = 0; i < object_count; i++) {
        reverse_object_mindist_list(&objects[i]->exact_synapses);
        reverse_object_mindist_list(&objects[i]->invalid_synapses);
    }
}

static void scramble_contact_lists(ivp_object_t *const *objects,
                                   int object_count)
{
    ivp_fr_system_t *seen[128];
    int seen_count = 0;
    for (int i = 0; i < object_count; i++) {
        for (ivp_fr_synapse_t *synapse = objects[i]->fr_synapses;
             synapse; synapse = synapse->next) {
            ivp_fr_system_t *system =
                synapse->contact_point->l_friction_system;
            bool already_seen = false;
            for (int j = 0; j < seen_count; j++) {
                if (seen[j] == system) {
                    already_seen = true;
                    break;
                }
            }
            if (already_seen) continue;
            IVP_ASSERT(seen_count <
                       (int)(sizeof(seen) / sizeof(seen[0])));
            if (seen_count >=
                (int)(sizeof(seen) / sizeof(seen[0]))) {
                return;
            }
            seen[seen_count++] = system;
            reverse_contact_list(system);
            for (int j = 0;
                 j < ivp_pvec_len(&system->fr_pairs_of_objs);
                 j++) {
                ivp_fr_core_pair_t *pair =
                    (ivp_fr_core_pair_t *)ivp_pvec_at(
                        &system->fr_pairs_of_objs, j);
                reverse_pvec(&pair->fr_dists);
            }
            for (int j = 0;
                 j < ivp_pvec_len(&system->cores_of_friction_system);
                 j++) {
                ivp_core_t *core =
                    (ivp_core_t *)ivp_pvec_at(
                        &system->cores_of_friction_system, j);
                ivp_fr_info_for_core_t *info =
                    ivp_core_get_friction_info(core, system);
                if (info) reverse_pvec(&info->friction_springs);
            }
            reverse_pvec(&system->cores_of_friction_system);
            reverse_pvec(&system->moveable_cores_of_friction_system);
            reverse_pvec(&system->fr_pairs_of_objs);
        }
    }
    for (int i = 0; i < object_count; i++) {
        ivp_fr_synapse_t *synapse = objects[i]->fr_synapses;
        ivp_fr_synapse_t *new_head = NULL;
        while (synapse) {
            ivp_fr_synapse_t *next = synapse->next;
            synapse->next = synapse->prev;
            synapse->prev = next;
            new_head = synapse;
            synapse = next;
        }
        objects[i]->fr_synapses = new_head;
    }
}

static bool contact_lists_follow_creation_order(
    ivp_object_t *const *objects, int object_count)
{
    for (int i = 0; i < object_count; i++) {
        ivp_fr_synapse_t *previous = NULL;
        uint64_t previous_key = 0u;
        for (ivp_fr_synapse_t *synapse = objects[i]->fr_synapses;
             synapse; synapse = synapse->next) {
            uint64_t key = synapse->contact_point->creation_sequence;
            if (synapse->prev != previous || key == 0u ||
                (previous && previous_key >= key)) {
                return false;
            }
            previous = synapse;
            previous_key = key;
        }
    }
    return true;
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

static void strong_ordered_controller(
    const ivp_event_sim_t *event_sim, ivp_object_t *object, void *ctx)
{
    (void)event_sim;
    int operation = *(const int *)ctx;
    if (operation == 0) {
        object->physical_core->speed.k[0] += IVP_REAL(0.125);
    } else {
        object->physical_core->speed.k[0] *= IVP_REAL(1.0009765625);
    }
}

static void strong_noop_controller(
    const ivp_event_sim_t *event_sim, ivp_object_t *object, void *ctx)
{
    (void)event_sim;
    (void)object;
    (void)ctx;
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
    if (pool_warmup_count > 0) {
        ivp_controller_handle_t warmup_controllers[2];
        for (int i = 0; i < 2; i++) {
            warmup_controllers[i] = ivp_environment_add_controller(
                scene->environment, 1300, strong_noop_controller, NULL);
            if (!ivp_controller_handle_is_valid(warmup_controllers[i])) {
                return false;
            }
        }
        for (int i = 0; i < 2; i++) {
            if (!ivp_environment_remove_controller(
                    scene->environment, warmup_controllers[i])) {
                return false;
            }
        }
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
    for (int i = 0; i < 2; i++) {
        scene->controller_operations[i] = i;
        scene->controller_handles[i] = ivp_environment_add_controller(
            scene->environment, 1300, strong_ordered_controller,
            &scene->controller_operations[i]);
        if (!ivp_controller_handle_is_valid(
                scene->controller_handles[i])) {
            return false;
        }
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

static bool strong_compound_scene_init(strong_compound_scene_t *scene)
{
    memset(scene, 0, sizeof(*scene));
    scene->environment = strong_gravity_environment_create();
    if (!scene->environment) return false;

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(2.0);
    object_template.physical_unmoveable = true;
    object_template.mass = IVP_REAL(1000000.0);
    ivp_vec3r_set(&object_template.position,
                  IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0));
    scene->ground = ivp_environment_create_object(
        scene->environment, &object_template, NULL);
    if (!scene->ground) return false;

    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.55);
    object_template.mass = IVP_REAL(1.0);
    object_template.material.friction = IVP_REAL(0.8);
    object_template.material.elasticity = IVP_REAL(0.0);
    object_template.start_in_simulation = false;
    object_template.enable_collision_detection = false;
    for (int i = 0; i < STRONG_COMPOUND_PART_COUNT; i++) {
        ivp_vec3r_set(&object_template.position,
                      (ivp_real_t)(-0.9 + 0.9 * (double)i),
                      IVP_REAL(-3.0), IVP_REAL(0.0));
        scene->parts[i] = ivp_environment_create_object(
            scene->environment, &object_template, NULL);
        if (!scene->parts[i]) return false;
    }
    if (!ivp_environment_merge_objects(
            scene->environment, scene->parts,
            STRONG_COMPOUND_PART_COUNT)) {
        return false;
    }
    for (int i = 0; i < STRONG_COMPOUND_PART_COUNT; i++) {
        ivp_object_enable_collision_detection(scene->parts[i], true);
    }
    ivp_object_ensure_in_simulation_now(scene->parts[0]);
    scene->parts[0]->physical_core->speed.k[0] = IVP_REAL(0.05);
    return true;
}

static bool strong_compound_scenes_equal(
    const strong_compound_scene_t *a,
    const strong_compound_scene_t *b)
{
    if (ivp_environment_get_time(a->environment) !=
        ivp_environment_get_time(b->environment)) {
        return false;
    }
    if (!visible_core_state_equal(a->ground->physical_core,
                                  b->ground->physical_core)) {
        return false;
    }
    for (int i = 0; i < STRONG_COMPOUND_PART_COUNT; i++) {
        if (!visible_core_state_equal(a->parts[i]->physical_core,
                                      b->parts[i]->physical_core) ||
            a->parts[i]->object_movement_state !=
                b->parts[i]->object_movement_state) {
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
        if (perturbations & STRONG_PERTURB_CONTACT_LISTS) {
            scramble_contact_lists(perturbed.objects,
                                   STRONG_OBJECT_COUNT);
        }
        if (perturbations & STRONG_PERTURB_MINDIST_MANAGER) {
            scramble_mindist_manager(perturbed.environment);
        }
        if (perturbations & STRONG_PERTURB_OBJECT_MINDISTS) {
            scramble_object_mindists(perturbed.objects,
                                     STRONG_OBJECT_COUNT);
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
    bool object_slots_changed = false;
    bool controller_slots_changed = false;
    for (int i = 0; ok && i < STRONG_OBJECT_COUNT; i++) {
        object_slots_changed = object_slots_changed ||
            canonical.objects[i]->handle.slot !=
                warmed.objects[i]->handle.slot;
        if (canonical.objects[i]->creation_sequence !=
            warmed.objects[i]->creation_sequence) {
            ok = false;
        }
    }
    for (int i = 0; ok && i < 2; i++) {
        controller_slots_changed = controller_slots_changed ||
            canonical.controller_handles[i].slot !=
                warmed.controller_handles[i].slot;
    }
    ok = ok && object_slots_changed && controller_slots_changed;
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

IVP_TEST(contact_list_order_does_not_change_visible_state)
{
    IVP_ASSERT_TRUE(run_perturbation(STRONG_PERTURB_CONTACT_LISTS));
}

IVP_TEST(mindist_manager_order_does_not_change_visible_state)
{
    IVP_ASSERT_TRUE(run_perturbation(STRONG_PERTURB_MINDIST_MANAGER));
}

IVP_TEST(object_mindist_order_does_not_change_visible_state)
{
    IVP_ASSERT_TRUE(run_perturbation(STRONG_PERTURB_OBJECT_MINDISTS));
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

IVP_TEST(contact_stack_order_does_not_change_visible_state)
{
    strong_cube_scene_t canonical = {0};
    strong_cube_scene_t perturbed = {0};
    bool ok = strong_cube_scene_init(&canonical, true) &&
              strong_cube_scene_init(&perturbed, true);
    bool reversed_multiple_contacts = false;
    for (int step = 0; ok && step < 180; step++) {
        for (int i = 0; i < STRONG_CUBE_OBJECT_COUNT; i++) {
            for (ivp_fr_synapse_t *synapse =
                     perturbed.objects[i]->fr_synapses;
                 synapse; synapse = synapse->next) {
                ivp_fr_system_t *system =
                    synapse->contact_point->l_friction_system;
                if (system->friction_dist_number > 1) {
                    reversed_multiple_contacts = true;
                    break;
                }
            }
            if (reversed_multiple_contacts) break;
        }
        scramble_contact_lists(perturbed.objects,
                               STRONG_CUBE_OBJECT_COUNT);
        ivp_environment_simulate_one_psi(canonical.environment);
        ivp_environment_simulate_one_psi(perturbed.environment);
        if (!contact_lists_follow_creation_order(
                perturbed.objects, STRONG_CUBE_OBJECT_COUNT)) {
            fprintf(stderr,
                    "contact stack order was not restored at step %d\n",
                    step);
            ok = false;
        }
        if (!strong_cube_scenes_equal(&canonical, &perturbed)) {
            fprintf(stderr,
                    "contact stack order changed visible state at step %d\n",
                    step);
            ok = false;
        }
    }
    ok = ok && reversed_multiple_contacts;
    strong_cube_scene_destroy(&perturbed);
    strong_cube_scene_destroy(&canonical);
    IVP_ASSERT_TRUE(ok);
}

IVP_TEST(mindist_stack_order_does_not_change_visible_state)
{
    strong_cube_scene_t canonical = {0};
    strong_cube_scene_t perturbed = {0};
    bool ok = strong_cube_scene_init(&canonical, true) &&
              strong_cube_scene_init(&perturbed, true);
    bool reversed_multiple_mindists = false;
    for (int step = 0; ok && step < 180; step++) {
        int mindist_count = 0;
        for (ivp_mindist_t *mindist =
                 perturbed.environment->pl->mindist_mgr.exact_mindists;
             mindist; mindist = mindist->next) {
            mindist_count++;
        }
        reversed_multiple_mindists =
            reversed_multiple_mindists || mindist_count > 1;
        scramble_mindist_manager(perturbed.environment);
        scramble_object_mindists(perturbed.objects,
                                 STRONG_CUBE_OBJECT_COUNT);
        ivp_environment_simulate_one_psi(canonical.environment);
        ivp_environment_simulate_one_psi(perturbed.environment);
        if (!strong_cube_scenes_equal(&canonical, &perturbed)) {
            fprintf(stderr,
                    "mindist stack order changed visible state at step %d\n",
                    step);
            ok = false;
        }
    }
    ok = ok && reversed_multiple_mindists;
    strong_cube_scene_destroy(&perturbed);
    strong_cube_scene_destroy(&canonical);
    IVP_ASSERT_TRUE(ok);
}

IVP_TEST(compound_object_order_does_not_change_visible_state)
{
    strong_compound_scene_t canonical = {0};
    strong_compound_scene_t perturbed = {0};
    bool ok = strong_compound_scene_init(&canonical) &&
              strong_compound_scene_init(&perturbed);
    for (int step = 0; ok && step < 180; step++) {
        reverse_pvec(&perturbed.parts[0]->physical_core->objects);
        ivp_environment_simulate_one_psi(canonical.environment);
        ivp_environment_simulate_one_psi(perturbed.environment);
        if (!strong_compound_scenes_equal(&canonical, &perturbed)) {
            fprintf(stderr,
                    "compound object order changed visible state at step %d\n",
                    step);
            ok = false;
        }
    }
    ivp_environment_destroy(perturbed.environment);
    ivp_environment_destroy(canonical.environment);
    IVP_ASSERT_TRUE(ok);
}

typedef struct strong_revive_capture {
    uint64_t keys[STRONG_REVIVE_OBJECT_COUNT];
    int count;
} strong_revive_capture_t;

static void strong_capture_revived(ivp_listener_object_t *listener,
                                   ivp_event_object_t *event)
{
    strong_revive_capture_t *capture =
        (strong_revive_capture_t *)listener->user_data;
    IVP_ASSERT(capture->count < STRONG_REVIVE_OBJECT_COUNT);
    if (capture->count < STRONG_REVIVE_OBJECT_COUNT) {
        capture->keys[capture->count++] =
            event->real_object->creation_sequence;
    }
}

static bool strong_queue_revive_scene(
    ivp_environment_t **environment_out,
    ivp_object_t *objects[STRONG_REVIVE_OBJECT_COUNT],
    ivp_listener_object_t *listener,
    strong_revive_capture_t *capture)
{
    ivp_environment_t *environment = strong_environment_create(1);
    if (!environment) return false;

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.25);
    object_template.mass = IVP_REAL(1.0);
    object_template.start_in_simulation = false;
    object_template.enable_collision_detection = false;

    for (int i = 0; i < STRONG_REVIVE_OBJECT_COUNT; i++) {
        ivp_vec3r_set(&object_template.position,
                      (ivp_real_t)(4.0 * (double)i),
                      IVP_REAL(0.0), IVP_REAL(0.0));
        objects[i] = ivp_environment_create_object(
            environment, &object_template, NULL);
        if (!objects[i]) {
            ivp_environment_destroy(environment);
            return false;
        }
    }

    memset(capture, 0, sizeof(*capture));
    memset(listener, 0, sizeof(*listener));
    listener->event_object_revived = strong_capture_revived;
    listener->user_data = capture;
    ivp_environment_add_listener_object_global(environment, listener);
    for (int i = 0; i < STRONG_REVIVE_OBJECT_COUNT; i++) {
        ivp_object_ensure_in_simulation(objects[i]);
    }
    *environment_out = environment;
    return true;
}

IVP_TEST(revive_queue_order_does_not_change_observable_order)
{
    ivp_environment_t *canonical = NULL;
    ivp_environment_t *perturbed = NULL;
    ivp_object_t *canonical_objects[STRONG_REVIVE_OBJECT_COUNT] = {0};
    ivp_object_t *perturbed_objects[STRONG_REVIVE_OBJECT_COUNT] = {0};
    ivp_listener_object_t canonical_listener;
    ivp_listener_object_t perturbed_listener;
    strong_revive_capture_t canonical_capture;
    strong_revive_capture_t perturbed_capture;
    bool ok = strong_queue_revive_scene(
                  &canonical, canonical_objects, &canonical_listener,
                  &canonical_capture) &&
              strong_queue_revive_scene(
                  &perturbed, perturbed_objects, &perturbed_listener,
                  &perturbed_capture);
    if (ok) {
        reverse_pvec(&perturbed->pl->core_revive_list);
        ivp_environment_simulate_one_psi(canonical);
        ivp_environment_simulate_one_psi(perturbed);
        ok = canonical_capture.count == STRONG_REVIVE_OBJECT_COUNT &&
             perturbed_capture.count == STRONG_REVIVE_OBJECT_COUNT &&
             memcmp(canonical_capture.keys, perturbed_capture.keys,
                    sizeof(canonical_capture.keys)) == 0;
        for (int i = 0; ok && i < STRONG_REVIVE_OBJECT_COUNT; i++) {
            ok = visible_core_state_equal(
                     canonical_objects[i]->physical_core,
                     perturbed_objects[i]->physical_core) &&
                 canonical_objects[i]->object_movement_state ==
                     perturbed_objects[i]->object_movement_state;
        }
    }
    if (perturbed) {
        ivp_environment_remove_listener_object_global(
            perturbed, &perturbed_listener);
    }
    if (canonical) {
        ivp_environment_remove_listener_object_global(
            canonical, &canonical_listener);
    }
    ivp_environment_destroy(perturbed);
    ivp_environment_destroy(canonical);
    IVP_ASSERT_TRUE(ok);
}

typedef struct strong_cluster_capture {
    uint64_t keys[STRONG_CLUSTER_VISIT_CAPACITY];
    int count;
} strong_cluster_capture_t;

typedef struct strong_cluster_scene {
    ivp_environment_t *environment;
    ivp_object_t *objects[STRONG_CLUSTER_OBJECT_COUNT];
    ivp_cluster_t *outer_cluster;
    ivp_cluster_t *inner_cluster;
    strong_cluster_capture_t capture;
} strong_cluster_scene_t;

static void strong_capture_cluster_visit(void *context, ivp_object_t *object)
{
    strong_cluster_capture_t *capture =
        (strong_cluster_capture_t *)context;
    IVP_ASSERT(capture->count < STRONG_CLUSTER_VISIT_CAPACITY);
    if (capture->count < STRONG_CLUSTER_VISIT_CAPACITY) {
        capture->keys[capture->count++] = object->creation_sequence;
    }
}

static bool strong_cluster_scene_init(strong_cluster_scene_t *scene)
{
    memset(scene, 0, sizeof(*scene));
    ivp_environment_config_t config;
    ivp_environment_config_init(&config);
    config.disable_performance_counter = true;
    config.delta_psi_time = IVP_REAL(0.01);
    ivp_vec3r_set_zero(&config.gravity);
    scene->environment = ivp_environment_create(&config);
    if (!scene->environment) return false;

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.25);
    object_template.physical_unmoveable = true;
    object_template.start_in_simulation = false;
    object_template.enable_collision_detection = true;
    for (int i = 0; i < STRONG_CLUSTER_OBJECT_COUNT; i++) {
        ivp_vec3r_set(&object_template.position,
                      (ivp_real_t)(8.0 * (double)i),
                      IVP_REAL(0.0), IVP_REAL(0.0));
        scene->objects[i] = ivp_environment_create_object(
            scene->environment, &object_template, NULL);
        if (!scene->objects[i]) return false;
    }

    scene->outer_cluster = ivp_environment_create_cluster(
        scene->environment, NULL, NULL);
    scene->inner_cluster = ivp_environment_create_cluster(
        scene->environment, scene->outer_cluster, NULL);
    if (!scene->outer_cluster || !scene->inner_cluster ||
        scene->outer_cluster->base.creation_sequence == 0u ||
        scene->inner_cluster->base.creation_sequence == 0u ||
        scene->outer_cluster->base.creation_sequence ==
            scene->inner_cluster->base.creation_sequence) {
        return false;
    }
    ivp_cluster_add_child(scene->outer_cluster, scene->objects[0]);
    ivp_cluster_add_child(scene->outer_cluster, scene->objects[3]);
    ivp_cluster_add_child(scene->inner_cluster, scene->objects[1]);
    ivp_cluster_add_child(scene->inner_cluster, scene->objects[4]);

    ivp_universe_manager_t manager;
    ivp_universe_manager_init(&manager);
    manager.evict_fn = strong_capture_cluster_visit;
    manager.ctx = &scene->capture;
    manager.num_objects_in_environment_threshold_0 = 1;
    manager.check_objects_per_second_threshold_0 = 600;
    manager.num_objects_in_environment_threshold_1 = INT_MAX;
    ivp_environment_set_universe_manager(scene->environment, &manager);
    return true;
}

static void strong_cluster_scene_destroy(strong_cluster_scene_t *scene)
{
    if (!scene->environment) return;
    ivp_environment_set_universe_manager(scene->environment, NULL);
    ivp_environment_destroy(scene->environment);
    scene->environment = NULL;
}

IVP_TEST(cluster_link_order_does_not_change_observable_order)
{
    strong_cluster_scene_t canonical;
    strong_cluster_scene_t perturbed;
    bool ok = strong_cluster_scene_init(&canonical) &&
              strong_cluster_scene_init(&perturbed);
    for (int step = 0; ok && step < 8; step++) {
        reverse_cluster_tree(
            ivp_environment_get_root_cluster(perturbed.environment));
        ivp_environment_simulate_one_psi(canonical.environment);
        ivp_environment_simulate_one_psi(perturbed.environment);
        ok = canonical.capture.count == perturbed.capture.count &&
             memcmp(canonical.capture.keys, perturbed.capture.keys,
                    (size_t)canonical.capture.count *
                        sizeof(canonical.capture.keys[0])) == 0;
        for (int i = 0; ok && i < STRONG_CLUSTER_OBJECT_COUNT; i++) {
            ok = visible_core_state_equal(
                canonical.objects[i]->physical_core,
                perturbed.objects[i]->physical_core);
        }
    }
    ok = ok && canonical.capture.count >= STRONG_CLUSTER_OBJECT_COUNT;
    strong_cluster_scene_destroy(&perturbed);
    strong_cluster_scene_destroy(&canonical);
    IVP_ASSERT_TRUE(ok);
}

IVP_TEST(cluster_identity_blocks_sequence_reuse)
{
    ivp_environment_t *environment = strong_environment_create(1);
    IVP_ASSERT_NOT_NULL(environment);
    ivp_cluster_t *cluster = ivp_environment_create_cluster(
        environment, NULL, NULL);
    IVP_ASSERT_NOT_NULL(cluster);

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.25);
    object_template.start_in_simulation = false;
    object_template.enable_collision_detection = false;
    ivp_object_t *first = ivp_environment_create_object(
        environment, &object_template, NULL);
    IVP_ASSERT_NOT_NULL(first);
    uint64_t cluster_key = cluster->base.creation_sequence;
    uint64_t first_key = first->creation_sequence;
    ivp_environment_destroy_object(environment, first);

    ivp_object_t *second = ivp_environment_create_object(
        environment, &object_template, NULL);
    IVP_ASSERT_NOT_NULL(second);
    IVP_ASSERT_TRUE(cluster_key < first_key);
    IVP_ASSERT_TRUE(first_key < second->creation_sequence);
    ivp_environment_destroy_object(environment, second);
    ivp_environment_destroy_cluster(environment, cluster);

    ivp_object_t *fresh = ivp_environment_create_object(
        environment, &object_template, NULL);
    IVP_ASSERT_NOT_NULL(fresh);
    IVP_ASSERT_EQ(fresh->creation_sequence, 1u);
    ivp_environment_destroy(environment);
}

typedef struct strong_phantom_capture {
    ivp_listener_phantom_t listener;
    uint64_t keys[STRONG_PHANTOM_CORE_COUNT];
    int count;
} strong_phantom_capture_t;

typedef struct strong_phantom_scene {
    ivp_environment_t *environment;
    ivp_object_t *owner;
    ivp_object_t *objects[STRONG_PHANTOM_CORE_COUNT];
    ivp_controller_phantom_t *phantom;
    strong_phantom_capture_t capture;
} strong_phantom_scene_t;

static void strong_capture_phantom_left(
    ivp_listener_phantom_t *listener,
    ivp_controller_phantom_t *phantom,
    ivp_core_t *core)
{
    (void)phantom;
    strong_phantom_capture_t *capture =
        (strong_phantom_capture_t *)listener;
    IVP_ASSERT(capture->count < STRONG_PHANTOM_CORE_COUNT);
    if (capture->count < STRONG_PHANTOM_CORE_COUNT) {
        capture->keys[capture->count++] =
            ivp_core_stable_order_key(core);
    }
}

static bool strong_phantom_scene_init(strong_phantom_scene_t *scene)
{
    memset(scene, 0, sizeof(*scene));
    scene->environment = strong_environment_create(1);
    if (!scene->environment) return false;

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.25);
    object_template.mass = IVP_REAL(1.0);
    object_template.start_in_simulation = false;
    object_template.enable_collision_detection = false;
    scene->owner = ivp_environment_create_object(
        scene->environment, &object_template, NULL);
    if (!scene->owner) return false;
    for (int i = 0; i < STRONG_PHANTOM_CORE_COUNT; i++) {
        ivp_vec3r_set(&object_template.position,
                      (ivp_real_t)(4.0 * (double)(i + 1)),
                      IVP_REAL(0.0), IVP_REAL(0.0));
        scene->objects[i] = ivp_environment_create_object(
            scene->environment, &object_template, NULL);
        if (!scene->objects[i]) return false;
    }

    ivp_template_phantom_t phantom_template;
    ivp_template_phantom_init(&phantom_template);
    phantom_template.manage_intruding_cores = true;
    scene->phantom = ivp_object_convert_to_phantom(
        scene->owner, &phantom_template);
    if (!scene->phantom || !scene->phantom->set_of_cores) return false;
    memset(&scene->capture, 0, sizeof(scene->capture));
    scene->capture.listener.core_left_volume =
        strong_capture_phantom_left;
    ivp_controller_phantom_add_listener(
        scene->phantom, &scene->capture.listener);
    for (int i = 0; i < STRONG_PHANTOM_CORE_COUNT; i++) {
        ivp_set_active_add(scene->phantom->set_of_cores,
                           scene->objects[i]->physical_core);
    }
    return true;
}

IVP_TEST(phantom_hash_order_does_not_change_callback_order)
{
    strong_phantom_scene_t canonical;
    strong_phantom_scene_t perturbed;
    bool ok = strong_phantom_scene_init(&canonical) &&
              strong_phantom_scene_init(&perturbed);
    if (ok) {
        set_phantom_hash_order(&canonical.phantom->set_of_cores->set,
                               canonical.objects, false);
        set_phantom_hash_order(&perturbed.phantom->set_of_cores->set,
                               perturbed.objects, true);
        ivp_controller_phantom_destroy(canonical.phantom);
        canonical.phantom = NULL;
        ivp_controller_phantom_destroy(perturbed.phantom);
        perturbed.phantom = NULL;
        ok = canonical.capture.count == STRONG_PHANTOM_CORE_COUNT &&
             perturbed.capture.count == STRONG_PHANTOM_CORE_COUNT &&
             memcmp(canonical.capture.keys, perturbed.capture.keys,
                    sizeof(canonical.capture.keys)) == 0;
    }
    ivp_environment_destroy(perturbed.environment);
    ivp_environment_destroy(canonical.environment);
    IVP_ASSERT_TRUE(ok);
}

int main(void)
{
    IVP_REGISTER(arena_history_does_not_change_visible_state);
    IVP_REGISTER(sim_unit_list_order_does_not_change_visible_state);
    IVP_REGISTER(ov_tree_order_does_not_change_visible_state);
    IVP_REGISTER(client_pointer_order_does_not_change_visible_state);
    IVP_REGISTER(contact_list_order_does_not_change_visible_state);
    IVP_REGISTER(mindist_manager_order_does_not_change_visible_state);
    IVP_REGISTER(object_mindist_order_does_not_change_visible_state);
    IVP_REGISTER(combined_internal_perturbations_do_not_change_visible_state);
    IVP_REGISTER(pool_history_does_not_change_visible_state);
    IVP_REGISTER(worker_count_does_not_change_canonical_state);
    IVP_REGISTER(relocated_and_shared_geometry_do_not_change_visible_state);
    IVP_REGISTER(contact_stack_order_does_not_change_visible_state);
    IVP_REGISTER(mindist_stack_order_does_not_change_visible_state);
    IVP_REGISTER(compound_object_order_does_not_change_visible_state);
    IVP_REGISTER(revive_queue_order_does_not_change_observable_order);
    IVP_REGISTER(cluster_link_order_does_not_change_observable_order);
    IVP_REGISTER(cluster_identity_blocks_sequence_reuse);
    IVP_REGISTER(phantom_hash_order_does_not_change_callback_order);
    IVP_RUN_ALL();
}
