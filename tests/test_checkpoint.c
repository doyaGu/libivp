/* test_checkpoint.c -- a world built in a memory region and put back to a
 * checkpoint must continue bit for bit like it did the first time.
 *
 * Every scene keeps its environment, controller states and its own
 * bookkeeping in the region; geometry stays outside.  A reference world
 * records a fingerprint of every object after each PSI.  A second world
 * runs to a capture point, takes a checkpoint, runs to the end (the
 * checkpoint must not disturb it), is restored, must match the capture
 * point, and runs to the end again; it is restored once more and runs part
 * of the way.  The scenes cover what world snapshots could not: vehicles,
 * objects of several convex ledges, merged cores, objects created and
 * destroyed after the checkpoint, motion controllers, a buoyancy pool and
 * a forcefield following a caller's set, and a world whose islands run on
 * a task pool (which must stay outside the region). */
#include "ivp_test.h"
#include "ivp_test_scene.h"

#include <ivp/base/ivp_memory_region.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/containers/ivp_set.h>
#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/controllers/ivp_controller_buoyancy.h>
#include <ivp/controllers/ivp_controller_golem.h>
#include <ivp/controllers/ivp_controller_motion.h>
#include <ivp/controllers/ivp_controller_raycast_car.h>
#include <ivp/controllers/ivp_forcefield.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_merge_core.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/geometry/ivp_surman.h>

#include <stdint.h>
#include <string.h>

#define CK_PSIS 360

/* geometry shared by every world, outside the regions */
typedef struct ck_assets {
    ivp_test_scene_assets_t scene;
    ivp_compact_surface_t *compound_surface;
    ivp_surman_polygon_t compound_surman;
} ck_assets_t;

typedef struct ck_world {
    ivp_memory_region_t *region;
    const ivp_allocator_t *alloc;
    ivp_environment_t *env;
    const ck_assets_t *assets;
    void *scene;   /* the scene's own state, in the region */
} ck_world_t;

typedef struct ck_scene {
    const char *name;
    int workers;   /* parallel_worker_count */
    size_t state_size;
    bool (*build)(ck_world_t *world);
    void (*drive)(ck_world_t *world, int psi);   /* before PSI psi; may be NULL */
    void (*teardown)(ck_world_t *world);         /* before destroying; may be NULL */
} ck_scene_t;

/* ── Helpers ─────────────────────────────────────────────────────────── */

static ivp_object_t *ck_body_ex(ck_world_t *world, const ivp_surman_t *surman,
                                bool ball, double x, double y, double z,
                                bool simulated)
{
    return ivp_test_scene_body(world->env, &world->assets->scene,
                               ball ? NULL : surman, x, y, z, simulated);
}

static ivp_object_t *ck_body(ck_world_t *world, const ivp_surman_t *surman,
                             bool ball, double x, double y, double z)
{
    return ck_body_ex(world, surman, ball, x, y, z, true);
}

static ivp_object_t *ck_ground(ck_world_t *world)
{
    return ivp_test_scene_ground(world->env, &world->assets->scene);
}

static const ivp_surman_t *ck_cube(const ck_world_t *world)
{
    return &world->assets->scene.cube_surman.base;
}

static void ck_push(ivp_object_t *object, double vx, double vy, double vz)
{
    ivp_vec3r_t speed = ivp_vec3r((ivp_real_t)vx, (ivp_real_t)vy, (ivp_real_t)vz);
    ivp_object_async_add_speed_ws(object, &speed);
}

/* four cubes, one on top, a ball rolling into them */
static bool ck_pile(ck_world_t *world, double x, ivp_object_t *out[6])
{
    for (int i = 0; i < 4; i++) {
        out[i] = ck_body(world, ck_cube(world), false,
                         x + ((i & 1) ? 0.55 : -0.55), 0.9,
                         (i & 2) ? 0.55 : -0.55);
        if (!out[i]) return false;
    }
    out[4] = ck_body(world, ck_cube(world), false, x + 0.1, -0.4, -0.1);
    out[5] = ck_body(world, NULL, true, x + 3.0, 1.05, 0.0);
    if (!out[4] || !out[5]) return false;
    ck_push(out[5], -1.5, 0.0, 0.25);
    return true;
}

/* ── Scenes ──────────────────────────────────────────────────────────── */

static bool ck_build_piles(ck_world_t *world)
{
    ivp_object_t *a[6], *b[6];
    return ck_ground(world) && ck_pile(world, -4.0, a) && ck_pile(world, 4.0, b);
}

typedef struct ck_linked {
    ivp_spring_state_t springs[2];
    ivp_constraint_state_t constraint;
} ck_linked_t;

static bool ck_build_linked(ck_world_t *world)
{
    ck_linked_t *s = (ck_linked_t *)world->scene;
    ivp_object_t *a[6];
    if (!ck_ground(world) || !ck_pile(world, 0.0, a)) return false;
    ivp_vec3r_t center = ivp_vec3r_zero();
    for (int i = 0; i < 2; i++) {
        ivp_spring_state_init(&s->springs[i]);
        ivp_spring_state_setup(&s->springs[i], a[2 * i], &center, a[4 + i],
                               &center, IVP_REAL(20.0), IVP_REAL(2.0),
                               IVP_REAL(1.5));
        if (!ivp_controller_handle_is_valid(
                ivp_environment_announce_dependent_controller(
                    world->env, 100, ivp_controller_spring, &s->springs[i],
                    ivp_spring_discovery))) {
            return false;
        }
    }
    ivp_vec3r_t anchor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.9), IVP_REAL(0.0));
    ivp_constraint_init_ball_socket_ws(&s->constraint, a[1], &anchor, a[3]);
    return ivp_controller_handle_is_valid(
        ivp_environment_announce_dependent_controller(
            world->env, 50, ivp_controller_constraint, &s->constraint,
            ivp_constraint_discovery));
}

typedef struct ck_vehicle {
    ivp_car_state_t car;
    ivp_object_t *ground;
} ck_vehicle_t;

/* the ground's top face, straight down along +y */
static void ck_cast_rays(void *ctx, const ivp_event_sim_t *es, int n_rays,
                         const ivp_ray_solver_template_t *rays,
                         ivp_ray_hit_t *hits, float *friction)
{
    (void)es;
    const ck_vehicle_t *s = (const ck_vehicle_t *)ctx;
    for (int i = 0; i < n_rays; i++) {
        memset(&hits[i], 0, sizeof(hits[i]));
        friction[i] = 0.8f;
        double dy = (double)rays[i].ray_normized_direction.k[1];
        if (!(dy > 1e-6)) continue;
        double t = (IVP_TEST_GROUND_TOP - (double)rays[i].ray_start_point.k[1]) / dy;
        if (t < 0.0 || t > (double)rays[i].ray_length) continue;
        hits[i].hit_object = s->ground;
        ivp_vec3r_set(&hits[i].hit_normal, IVP_REAL(0.0), IVP_REAL(-1.0),
                      IVP_REAL(0.0));
        hits[i].hit_distance = (ivp_real_t)(float)t;
    }
}

static bool ck_build_vehicle(ck_world_t *world)
{
    ck_vehicle_t *s = (ck_vehicle_t *)world->scene;
    s->ground = ck_ground(world);
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_POLYGON;
    t.mass = IVP_REAL(400.0);
    ivp_vec3r_set(&t.position, IVP_REAL(0.0), IVP_REAL(0.4), IVP_REAL(-6.0));
    ivp_object_t *body = ivp_environment_create_object(world->env, &t, ck_cube(world));
    ivp_object_t *a[6];
    if (!s->ground || !body || !ck_pile(world, 0.0, a) ||
        !ivp_car_init_4wheel(&s->car, body, IVP_REAL(1.6), IVP_REAL(1.2),
                             IVP_REAL(0.3), IVP_REAL(0.5))) {
        return false;
    }
    ivp_car_set_raycast(&s->car, ck_cast_rays, s);
    ck_push(body, 0.0, 0.0, 2.0);
    return true;
}

/* throttle and steering change over the run */
static void ck_drive_vehicle(ck_world_t *world, int psi)
{
    ck_vehicle_t *s = (ck_vehicle_t *)world->scene;
    if (psi % 40 == 0) {
        float torque = (psi / 40) % 2 ? 60.0f : 140.0f;
        ivp_car_change_wheel_torque(&s->car, 2, torque);
        ivp_car_change_wheel_torque(&s->car, 3, torque);
        ivp_car_do_steering(&s->car, (float)((psi / 40) % 3 - 1) * 0.2f);
    }
}

static void ck_teardown_vehicle(ck_world_t *world)
{
    ivp_car_destroy(&((ck_vehicle_t *)world->scene)->car);
}

/* bodies of two convex ledges (recursive mindists) tumbling together */
static bool ck_build_compound(ck_world_t *world)
{
    if (!ck_ground(world)) return false;
    const ivp_surman_t *compound = &world->assets->compound_surman.base;
    for (int i = 0; i < 4; i++) {
        ivp_object_t *body = ck_body(world, compound, false, -1.5 + i, -0.5 - 1.2 * i,
                                     0.3 * (i & 1));
        if (!body) return false;
        ck_push(body, 0.4 * (1 - i), 0.0, 0.2);
    }
    return true;
}

typedef struct ck_merged {
    ivp_object_t *later[2];
} ck_merged_t;

/* two cubes merged at the start, two more hanging until they are merged
 * mid-run; merging takes objects that are not simulated yet */
static bool ck_build_merged(ck_world_t *world)
{
    ck_merged_t *s = (ck_merged_t *)world->scene;
    ivp_object_t *a[6];
    if (!ck_ground(world) || !ck_pile(world, 0.0, a)) return false;
    ivp_object_t *first[2] = {
        ck_body_ex(world, ck_cube(world), false, -3.0, -1.0, 0.0, false),
        ck_body_ex(world, ck_cube(world), false, -2.0, -1.0, 0.0, false)
    };
    s->later[0] = ck_body_ex(world, ck_cube(world), false, 3.0, -2.0, 0.0, false);
    s->later[1] = ck_body_ex(world, ck_cube(world), false, 3.0, -3.0, 0.0, false);
    if (!first[0] || !first[1] || !s->later[0] || !s->later[1] ||
        !ivp_environment_merge_objects(world->env, first, 2)) {
        return false;
    }
    ck_push(first[0], 0.5, 0.0, 0.3);
    return true;
}

static void ck_drive_merged(ck_world_t *world, int psi)
{
    ck_merged_t *s = (ck_merged_t *)world->scene;
    if (psi == 150 && ivp_environment_merge_objects(world->env, s->later, 2)) {
        ck_push(s->later[0], -0.4, 0.0, 0.2);
    }
}

#define CK_SPAWNED 8

typedef struct ck_spawn {
    ivp_object_t *spawned[CK_SPAWNED];
    int next;
} ck_spawn_t;

/* balls created every 30 PSIs, the oldest destroyed every 45 */
static bool ck_build_spawn(ck_world_t *world)
{
    ivp_object_t *a[6];
    return ck_ground(world) && ck_pile(world, 0.0, a);
}

static void ck_drive_spawn(ck_world_t *world, int psi)
{
    ck_spawn_t *s = (ck_spawn_t *)world->scene;
    if (psi % 30 == 10) {
        int slot = s->next++ % CK_SPAWNED;
        if (s->spawned[slot]) ivp_environment_destroy_object(world->env, s->spawned[slot]);
        s->spawned[slot] = ck_body(world, NULL, true, 0.2 * (psi % 7) - 0.6,
                                   -2.0, 0.1 * (psi % 5));
    }
    if (psi % 45 == 20) {
        for (int i = 0; i < CK_SPAWNED; i++) {
            if (s->spawned[i]) {
                ivp_environment_destroy_object(world->env, s->spawned[i]);
                s->spawned[i] = NULL;
                break;
            }
        }
    }
}

typedef struct ck_driven {
    ivp_motion_state_t motion;
    ivp_golem_state_t golem;
} ck_driven_t;

/* a motion controller on every core and a golem on one */
static bool ck_build_driven(ck_world_t *world)
{
    ck_driven_t *s = (ck_driven_t *)world->scene;
    ivp_object_t *a[6], *b[6];
    if (!ck_ground(world) || !ck_pile(world, -4.0, a) || !ck_pile(world, 4.0, b)) {
        return false;
    }
    ivp_motion_state_init(&s->motion);
    ivp_motion_state_setup(&s->motion, a[4]);
    ivp_vec3r_t target = ivp_vec3r(IVP_REAL(-3.0), IVP_REAL(-0.5), IVP_REAL(1.0));
    ivp_motion_state_set_target_position(&s->motion, &target);
    ivp_golem_state_init(&s->golem);
    ivp_golem_state_setup(&s->golem, b[4]);
    target = ivp_vec3r(IVP_REAL(5.0), IVP_REAL(-0.8), IVP_REAL(-0.5));
    ivp_golem_set_target_position(&s->golem, &target);
    ivp_controller_handle_t golem = ivp_environment_create_core_controller(
        world->env, IVP_CP_MOTION, ivp_controller_golem, &s->golem);
    return ivp_controller_handle_is_valid(ivp_environment_add_controller(
               world->env, IVP_CP_MOTION, ivp_controller_motion, &s->motion)) &&
           ivp_controller_handle_is_valid(golem) &&
           ivp_environment_add_controller_to_core(world->env, golem, b[4]);
}

#define CK_POOL_OBJECTS 6

typedef struct ck_pool {
    ivp_object_t *members[CK_POOL_OBJECTS];
    ivp_buoyancy_state_t pool;
    ivp_set_active_t set;   /* the caller's set, in the region */
    ivp_forcefield_state_t field;
    bool pool_ready;
    bool field_ready;
} ck_pool_t;

/* a buoyancy pool over some objects, and a forcefield following a set of
 * cores the scene changes during the run */
static bool ck_build_pool(ck_world_t *world)
{
    ck_pool_t *s = (ck_pool_t *)world->scene;
    if (!ck_ground(world)) return false;
    for (int i = 0; i < CK_POOL_OBJECTS; i++) {
        s->members[i] = ck_body(world, i & 1 ? NULL : ck_cube(world), i & 1,
                                -2.5 + i, -1.0 - 0.3 * i, 0.4 * (i % 3));
        if (!s->members[i]) return false;
    }
    ivp_buoyancy_init(&s->pool, IVP_REAL(0.8), IVP_REAL(1.2));
    s->pool_ready = true;
    ivp_buoyancy_set_objects(&s->pool, s->members, 4);
    if (!ivp_controller_handle_is_valid(ivp_environment_add_controller(
            world->env, 200, ivp_controller_buoyancy, &s->pool))) {
        return false;
    }
    ivp_set_active_init(&s->set, world->alloc, 8);
    ivp_set_active_add(&s->set, s->members[4]->physical_core);
    ivp_vec3r_t push = ivp_vec3r(IVP_REAL(1.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_forcefield_state_init(&s->field, IVP_FORCEFIELD_CONSTANT, &push);
    s->field_ready = ivp_forcefield_attach_set(&s->field, world->env, &s->set, false);
    return s->field_ready;
}

static void ck_drive_pool(ck_world_t *world, int psi)
{
    ck_pool_t *s = (ck_pool_t *)world->scene;
    if (psi == 100) ivp_set_active_add(&s->set, s->members[5]->physical_core);
    if (psi == 220) ivp_set_active_remove(&s->set, s->members[4]->physical_core);
}

static void ck_teardown_pool(ck_world_t *world)
{
    ck_pool_t *s = (ck_pool_t *)world->scene;
    if (s->field_ready) ivp_forcefield_destroy(&s->field);
    ivp_set_active_destroy(&s->set);
    if (s->pool_ready) ivp_buoyancy_state_destroy(&s->pool);
}

#define CK_ISLANDS 200

/* many separate falling cubes: until they land each is an island of its
 * own that the parallel kernels take */
static bool ck_build_islands(ck_world_t *world)
{
    if (!ck_ground(world)) return false;
    for (int i = 0; i < CK_ISLANDS; i++) {
        double x = -19.0 + 2.0 * (i % 20);
        double z = -9.0 + 2.0 * (i / 20);
        ivp_object_t *cube = ck_body(world, ck_cube(world), false, x,
                                     -3.0 - 0.05 * (i % 7), z);
        if (!cube) return false;
        ck_push(cube, 0.1 * (i % 3 - 1), 0.0, 0.05 * (i % 2));
    }
    return true;
}

static const ck_scene_t ck_scenes[] = {
    { "piles", 1, 1, ck_build_piles, NULL, NULL },
    { "linked", 1, sizeof(ck_linked_t), ck_build_linked, NULL, NULL },
    { "vehicle", 1, sizeof(ck_vehicle_t), ck_build_vehicle, ck_drive_vehicle,
      ck_teardown_vehicle },
    { "compound", 1, 1, ck_build_compound, NULL, NULL },
    { "merged", 1, sizeof(ck_merged_t), ck_build_merged, ck_drive_merged, NULL },
    { "spawn", 1, sizeof(ck_spawn_t), ck_build_spawn, ck_drive_spawn, NULL },
    { "driven", 1, sizeof(ck_driven_t), ck_build_driven, NULL, NULL },
    { "pool", 1, sizeof(ck_pool_t), ck_build_pool, ck_drive_pool, ck_teardown_pool },
    { "islands", 4, 1, ck_build_islands, NULL, NULL },
};

static const ck_scene_t *ck_find_scene(const char *name)
{
    for (size_t i = 0; i < sizeof(ck_scenes) / sizeof(ck_scenes[0]); i++) {
        if (!strcmp(ck_scenes[i].name, name)) return &ck_scenes[i];
    }
    return NULL;
}

/* ── Worlds ──────────────────────────────────────────────────────────── */

static bool ck_assets_init(ck_assets_t *assets)
{
    memset(assets, 0, sizeof(*assets));
    if (!ivp_test_scene_assets_init(&assets->scene)) return false;
    ivp_vec3f_t points[2][8];
    ivp_compact_ledge_t *ledges[2];
    for (int l = 0; l < 2; l++) {
        for (int i = 0; i < 8; i++) {
            points[l][i] = ivp_vec3f((i & 1) ? 0.3f : -0.3f,
                                     ((i & 2) ? 0.25f : -0.25f) + (l ? 0.45f : -0.45f),
                                     (i & 4) ? 0.35f : -0.35f);
        }
        ledges[l] = ivp_build_convex_ledge(NULL, points[l], 8);
    }
    if (ledges[0] && ledges[1]) {
        ivp_ledge_soup_t soup;
        ivp_ledge_soup_init(&soup, NULL);
        ivp_ledge_soup_add(&soup, ledges[0]);
        ivp_ledge_soup_add(&soup, ledges[1]);
        assets->compound_surface = ivp_ledge_soup_compile(&soup);
        ivp_ledge_soup_destroy(&soup);
    }
    for (int l = 0; l < 2; l++) {
        if (ledges[l]) ivp_compact_ledge_free(NULL, ledges[l]);
    }
    if (!assets->compound_surface) return false;
    ivp_surman_polygon_init(&assets->compound_surman, assets->compound_surface);
    return true;
}

static void ck_assets_destroy(ck_assets_t *assets)
{
    if (assets->compound_surface) ivp_compact_surface_free(NULL, assets->compound_surface);
    ivp_test_scene_assets_destroy(&assets->scene);
}

static void ck_world_destroy(const ck_scene_t *scene, ck_world_t *world)
{
    if (world->env && scene->teardown) scene->teardown(world);
    if (world->env) ivp_environment_destroy(world->env);
    ivp_memory_region_destroy(world->region);
    memset(world, 0, sizeof(*world));
}

static bool ck_world_build(const ck_scene_t *scene, const ck_assets_t *assets,
                           ck_world_t *world)
{
    memset(world, 0, sizeof(*world));
    world->assets = assets;
    world->region = ivp_memory_region_create(0);
    if (!world->region) return false;
    world->alloc = ivp_memory_region_allocator(world->region);
    ivp_environment_config_t config;
    ivp_environment_config_init(&config);
    config.disable_performance_counter = true;
    config.allocator = (ivp_allocator_t *)world->alloc;
    config.parallel_worker_count = scene->workers;
    world->env = ivp_environment_create(&config);
    world->scene = ivp_alloc(world->alloc, scene->state_size);
    if (!world->env || !world->scene || !scene->build(world)) {
        ck_world_destroy(scene, world);
        return false;
    }
    return true;
}

/* fingerprint of every object's visible state and the world time */
static uint64_t ck_fingerprint(const ivp_environment_t *env)
{
    uint64_t hash = 0xcbf29ce484222325ull;
#define CK_MIX(value)                                                     \
    do {                                                                  \
        uint64_t bits_ = 0;                                               \
        double value_ = (double)(value);                                  \
        memcpy(&bits_, &value_, sizeof(bits_));                           \
        hash = (hash ^ bits_) * 0x100000001b3ull;                         \
    } while (0)
    CK_MIX(ivp_environment_get_time(env));
    int count = ivp_environment_get_object_count(env);
    CK_MIX(count);
    for (int i = 0; i < count; i++) {
        ivp_object_t *object = ivp_environment_get_object(env, i);
        ivp_vec3r_t position, speed;
        ivp_vec3f_t rot_speed;
        ivp_quatr_t orientation;
        ivp_object_get_position(object, &position);
        ivp_object_get_orientation(object, &orientation);
        ivp_object_get_speed(object, &speed);
        ivp_object_get_rot_speed(object, &rot_speed);
        for (int k = 0; k < 3; k++) {
            CK_MIX(position.k[k]);
            CK_MIX(speed.k[k]);
            CK_MIX(rot_speed.k[k]);
        }
        CK_MIX(orientation.x);
        CK_MIX(orientation.y);
        CK_MIX(orientation.z);
        CK_MIX(orientation.w);
        CK_MIX(object->physical_core->movement_state);
    }
#undef CK_MIX
    return hash;
}

static void ck_step(const ck_scene_t *scene, ck_world_t *world, int psi)
{
    if (scene->drive) scene->drive(world, psi);
    ivp_environment_simulate_one_psi(world->env);
}

/* run from psi to end, comparing with expected[k] (the state after k + 1
 * PSIs); the first differing PSI, or end */
static int ck_run(const ck_scene_t *scene, ck_world_t *world, int psi, int end,
                  const uint64_t *expected)
{
    for (; psi < end; psi++) {
        ck_step(scene, world, psi);
        if (ck_fingerprint(world->env) != expected[psi]) return psi;
    }
    return end;
}

static bool ck_check_scene(const ck_scene_t *scene, const ck_assets_t *assets,
                           int capture)
{
    static uint64_t expected[CK_PSIS];
    ck_world_t world;
    if (!ck_world_build(scene, assets, &world)) {
        fprintf(stderr, "checkpoint %s: build failed\n", scene->name);
        return false;
    }
    for (int psi = 0; psi < CK_PSIS; psi++) {
        ck_step(scene, &world, psi);
        expected[psi] = ck_fingerprint(world.env);
    }
    ck_world_destroy(scene, &world);

    if (!ck_world_build(scene, assets, &world)) return false;
    bool ok = ck_run(scene, &world, 0, capture, expected) == capture;
    ivp_memory_checkpoint_t *checkpoint =
        ivp_memory_region_checkpoint(world.region, NULL);
    ok = ok && checkpoint;
    int reached = ok ? ck_run(scene, &world, capture, CK_PSIS, expected) : -1;
    if (reached != CK_PSIS) {
        fprintf(stderr, "checkpoint %s@%d: taking it changed PSI %d\n",
                scene->name, capture, reached);
        ok = false;
    }
    for (int again = 0; ok && again < 2; again++) {
        int end = again ? capture + 40 : CK_PSIS;
        ok = ivp_memory_region_restore(world.region, checkpoint) == IVP_OK &&
             ck_fingerprint(world.env) == expected[capture - 1];
        reached = ok ? ck_run(scene, &world, capture, end, expected) : -1;
        if (reached != end) {
            fprintf(stderr, "checkpoint %s@%d: restore %d diverges at PSI %d\n",
                    scene->name, capture, again + 1, reached);
            ok = false;
        }
    }
    fprintf(stderr, "checkpoint %s@%d: %s, %zu bytes\n", scene->name, capture,
            ok ? "exact" : "FAILED", ivp_memory_checkpoint_size(checkpoint));
    ivp_memory_checkpoint_destroy(checkpoint);
    ck_world_destroy(scene, &world);
    return ok;
}

IVP_TEST(restored_worlds_continue_exactly)
{
    ck_assets_t assets;
    IVP_ASSERT_TRUE(ck_assets_init(&assets));
    static const int captures[] = { 1, 37, 149, 151, 250 };
    bool ok = true;
    for (size_t s = 0; s < sizeof(ck_scenes) / sizeof(ck_scenes[0]); s++) {
        for (size_t c = 0; c < sizeof(captures) / sizeof(captures[0]); c++) {
            ok = ck_check_scene(&ck_scenes[s], &assets, captures[c]) && ok;
        }
    }
    ck_assets_destroy(&assets);
    IVP_ASSERT_TRUE(ok);
}

/* the scenes do what they are there for */
IVP_TEST(scenes_exercise_their_features)
{
    ck_assets_t assets;
    IVP_ASSERT_TRUE(ck_assets_init(&assets));
    ck_world_t world;

    const ck_scene_t *scene = ck_find_scene("vehicle");
    IVP_ASSERT_TRUE(ck_world_build(scene, &assets, &world));
    ivp_object_t *body = ((ck_vehicle_t *)world.scene)->car.car_body;
    ivp_vec3r_t start, end;
    ivp_object_get_position(body, &start);
    for (int psi = 0; psi < CK_PSIS; psi++) ck_step(scene, &world, psi);
    ivp_object_get_position(body, &end);
    IVP_ASSERT_TRUE(end.k[2] - start.k[2] > 1.0);   /* it drove */
    IVP_ASSERT_TRUE(end.k[1] < IVP_TEST_GROUND_TOP);      /* on its wheels, above ground */
    ck_world_destroy(scene, &world);

    scene = ck_find_scene("spawn");
    IVP_ASSERT_TRUE(ck_world_build(scene, &assets, &world));
    int count = ivp_environment_get_object_count(world.env);
    int most = count;
    for (int psi = 0; psi < CK_PSIS; psi++) {
        ck_step(scene, &world, psi);
        int now = ivp_environment_get_object_count(world.env);
        if (now > most) most = now;
    }
    IVP_ASSERT_TRUE(most > count + 1);
    ck_world_destroy(scene, &world);

    scene = ck_find_scene("merged");
    IVP_ASSERT_TRUE(ck_world_build(scene, &assets, &world));
    ck_merged_t *merged = (ck_merged_t *)world.scene;
    IVP_ASSERT_TRUE(merged->later[0]->physical_core != merged->later[1]->physical_core);
    for (int psi = 0; psi < 200; psi++) ck_step(scene, &world, psi);
    IVP_ASSERT_TRUE(merged->later[0]->physical_core == merged->later[1]->physical_core);
    ck_world_destroy(scene, &world);

    /* the pool floats its members; the field follows the set */
    scene = ck_find_scene("pool");
    IVP_ASSERT_TRUE(ck_world_build(scene, &assets, &world));
    ck_pool_t *pool = (ck_pool_t *)world.scene;
    ivp_vec3r_t floating, pushed_late;
    for (int psi = 0; psi < CK_PSIS; psi++) ck_step(scene, &world, psi);
    ivp_object_get_position(pool->members[0], &floating);
    ivp_object_get_position(pool->members[5], &pushed_late);
    IVP_ASSERT_TRUE(floating.k[1] < IVP_REAL(1.0));   /* held above the ground */
    IVP_ASSERT_TRUE(pushed_late.k[0] > IVP_REAL(2.6));  /* moved +x after joining */
    ck_world_destroy(scene, &world);

    /* the islands run on the task pool (debug builds run them serially) */
    scene = ck_find_scene("islands");
    IVP_ASSERT_TRUE(ck_world_build(scene, &assets, &world));
    IVP_ASSERT_EQ(ivp_environment_get_parallel_worker_count(world.env), 4);
    for (int psi = 0; psi < 30; psi++) ck_step(scene, &world, psi);
#ifndef IVP_DEBUG_BUILD
    ivp_parallel_stats_t stats;
    memset(&stats, 0, sizeof(stats));
    ivp_environment_get_parallel_stats(world.env, &stats);
    IVP_ASSERT_TRUE(stats.island_dispatches > 0 && stats.max_workers_used > 1);
#endif
    ck_world_destroy(scene, &world);
    ck_assets_destroy(&assets);
}

int main(void)
{
    IVP_REGISTER(restored_worlds_continue_exactly);
    IVP_REGISTER(scenes_exercise_their_features);
    IVP_RUN_ALL();
}
