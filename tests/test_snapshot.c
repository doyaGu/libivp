/* test_snapshot.c -- a world restored from a PSI-boundary snapshot must
 * continue bit for bit like the uninterrupted world.
 *
 * For every scene the uninterrupted run records the visible state after
 * each PSI.  For every capture point k a fresh world runs k PSIs, is
 * captured and destroyed, and the restored world runs the remaining PSIs;
 * each PSI must match the uninterrupted state of every object and the
 * world time.  A capture point may also be rejected when the world holds
 * state the snapshot does not support yet (friction contacts, for one).
 * Every scene records how many capture points continue exactly, so the
 * count must change, and be updated here, as soon as coverage grows.
 *
 * A dense pass captures the uninterrupted world after every PSI without
 * destroying it: the world must not notice (capture is read-only), and
 * every accepted capture must continue exactly for SNAP_DENSE_CONTINUATION
 * PSIs.  Coverage checks require captures that restore frozen next to
 * simulated objects and captures that restore exact mindists. */
#include "ivp_test.h"
#include "ivp_test_fault_allocator.h"

#include <ivp/builder/ivp_build_surface.h>
#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_snapshot.h>
#include <ivp/geometry/ivp_surman.h>

#include "dynamics/ivp_env_pipeline.h"

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define SNAP_MAX_OBJECTS 32
#define SNAP_PSI_COUNT 800
/* a capture point with frozen and simulated objects must leave at least
 * this many PSIs to continue */
#define SNAP_MIN_CONTINUATION 50
#define SNAP_DENSE_CONTINUATION 30

static const int snap_capture_points[] = { 1, 2, 17, 60, 150, 330, 640, 700 };
#define SNAP_CAPTURE_COUNT \
    ((int)(sizeof(snap_capture_points) / sizeof(snap_capture_points[0])))

typedef struct snap_assets {
    ivp_compact_surface_t *ground_surface;
    ivp_compact_surface_t *cube_surface;
    ivp_surman_polygon_t ground_surman;
    ivp_surman_polygon_t cube_surman;
} snap_assets_t;

#define SNAP_MAX_LINKS 4

/* controller states of an original world; they must outlive it */
typedef struct snap_controllers {
    ivp_spring_state_t springs[SNAP_MAX_LINKS];
    ivp_constraint_state_t constraints[SNAP_MAX_LINKS];
} snap_controllers_t;

typedef struct snap_scene {
    const char *name;
    bool gravity;
    bool (*build)(ivp_environment_t *env, snap_assets_t *assets,
                  snap_controllers_t *controllers);
    int exact_points; /* capture points that continue exactly */
    bool freezes;     /* objects freeze during the run */
} snap_scene_t;

typedef struct snap_object_state {
    ivp_time_t time_of_last_psi;
    int movement_state;
    int object_movement_state;
    ivp_real_t position[3];
    ivp_real_t speed[3];
    float rot_speed[3];
    ivp_real_t orientation[4];
    ivp_real_t next_orientation[4];
} snap_object_state_t;

typedef struct snap_state {
    ivp_time_t time;
    int object_count;
    snap_object_state_t objects[SNAP_MAX_OBJECTS];
} snap_state_t;

typedef struct snap_field {
    const char *name;
    size_t offset;
    size_t size;
} snap_field_t;

#define SNAP_FIELD(member) \
    { #member, offsetof(snap_object_state_t, member), \
      sizeof(((snap_object_state_t *)0)->member) }

static const snap_field_t snap_fields[] = {
    SNAP_FIELD(time_of_last_psi),
    SNAP_FIELD(movement_state),
    SNAP_FIELD(object_movement_state),
    SNAP_FIELD(position),
    SNAP_FIELD(speed),
    SNAP_FIELD(rot_speed),
    SNAP_FIELD(orientation),
    SNAP_FIELD(next_orientation)
};

#undef SNAP_FIELD

/* ── Scenes ──────────────────────────────────────────────────────────── */

static ivp_object_t *snap_create(ivp_environment_t *env, snap_assets_t *assets,
                                 bool ball, double x, double y, double z,
                                 double damp)
{
    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.material.friction = IVP_REAL(0.6);
    object_template.material.elasticity = IVP_REAL(0.3);
    object_template.speed_damp_factor = (ivp_real_t)damp;
    ivp_vec3r_set(&object_template.rot_speed_damp_factor, (ivp_real_t)damp,
                  (ivp_real_t)damp, (ivp_real_t)damp);
    ivp_vec3r_set(&object_template.position, (ivp_real_t)x, (ivp_real_t)y,
                  (ivp_real_t)z);
    if (ball) {
        object_template.type = IVP_OBJECT_BALL;
        object_template.ball_radius = IVP_REAL(0.4);
        object_template.mass = IVP_REAL(0.8);
        return ivp_environment_create_object(env, &object_template, NULL);
    }
    object_template.type = IVP_OBJECT_POLYGON;
    object_template.mass = IVP_REAL(0.6);
    ivp_vec3r_set(&object_template.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1),
                  IVP_REAL(0.1));
    object_template.rot_inertia_is_factor = false;
    return ivp_environment_create_object(env, &object_template,
                                         &assets->cube_surman.base);
}

static bool snap_push(ivp_object_t *object, double vx, double vy, double vz,
                      double wx, double wy, double wz)
{
    if (!object) return false;
    ivp_vec3r_t speed;
    ivp_vec3r_set(&speed, (ivp_real_t)vx, (ivp_real_t)vy, (ivp_real_t)vz);
    ivp_object_async_add_speed_ws(object, &speed);
    ivp_vec3r_t rot;
    ivp_vec3r_set(&rot, (ivp_real_t)wx, (ivp_real_t)wy, (ivp_real_t)wz);
    ivp_object_async_add_rot_speed_cs(object, &rot);
    return true;
}

static bool snap_create_ground(ivp_environment_t *env, snap_assets_t *assets)
{
    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_POLYGON;
    object_template.physical_unmoveable = true;
    object_template.mass = IVP_REAL(1.0e6);
    ivp_vec3r_set(&object_template.rot_inertia, IVP_REAL(1.0e6),
                  IVP_REAL(1.0e6), IVP_REAL(1.0e6));
    object_template.rot_inertia_is_factor = false;
    object_template.material.friction = IVP_REAL(0.6);
    object_template.material.elasticity = IVP_REAL(0.3);
    ivp_vec3r_set(&object_template.position, IVP_REAL(0.0), IVP_REAL(2.0),
                  IVP_REAL(0.0));
    return ivp_environment_create_object(env, &object_template,
                                         &assets->ground_surman.base) != NULL;
}

/* zero gravity, far apart: drifting and spinning, never near each other */
static bool snap_build_drift(ivp_environment_t *env, snap_assets_t *assets,
                             snap_controllers_t *controllers)
{
    (void)controllers;
    for (int i = 0; i < 6; i++) {
        bool ball = (i & 1) != 0;
        ivp_object_t *object = snap_create(env, assets, ball,
                                           (double)i * 20.0, 0.0, 0.0, 0.0);
        if (!snap_push(object, 0.25 * (double)(i % 3) - 0.25, 0.125,
                       0.0625 * (double)i, 0.5, 0.25 * (double)i, -0.75)) {
            return false;
        }
    }
    return true;
}

/* gravity, no ground: falling and tumbling */
static bool snap_build_fall(ivp_environment_t *env, snap_assets_t *assets,
                            snap_controllers_t *controllers)
{
    (void)controllers;
    for (int i = 0; i < 6; i++) {
        bool ball = (i % 3) == 0;
        ivp_object_t *object = snap_create(env, assets, ball,
                                           (double)i * 20.0,
                                           -(double)i, 0.0, 0.0);
        if (!snap_push(object, 0.5, -1.0 + 0.25 * (double)i, 0.0,
                       0.25 * (double)i, 1.0, 0.0)) {
            return false;
        }
    }
    return true;
}

/* zero gravity with damping: everything slows down and freezes, so the
 * movement checks and freezing cross the capture points */
static bool snap_build_settle(ivp_environment_t *env, snap_assets_t *assets,
                              snap_controllers_t *controllers)
{
    (void)controllers;
    for (int i = 0; i < 6; i++) {
        bool ball = (i & 1) == 0;
        ivp_object_t *object = snap_create(env, assets, ball,
                                           (double)i * 20.0, 0.0, 0.0,
                                           0.5 + 0.25 * (double)i);
        if (!snap_push(object, 0.5, 0.0, -0.25 * (double)i,
                       0.125 * (double)i, 0.0, 0.5)) {
            return false;
        }
    }
    return true;
}

/* cubes and balls dropped on the ground: impacts and resting contacts */
static bool snap_build_pile(ivp_environment_t *env, snap_assets_t *assets,
                            snap_controllers_t *controllers)
{
    (void)controllers;
    if (!snap_create_ground(env, assets)) return false;
    for (int i = 0; i < 6; i++) {
        bool ball = i >= 4;
        ivp_object_t *object = snap_create(
            env, assets, ball, (double)(i % 2) * 1.1 - 0.55,
            0.8 - 1.2 * (double)(i / 2), (double)(i % 3) * 0.3, 0.02);
        if (!snap_push(object, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)) return false;
    }
    return true;
}

/* zero gravity, elastic balls and cubes colliding in pairs: impacts and
 * mindists, but nothing stays in contact */
static bool snap_build_collide(ivp_environment_t *env, snap_assets_t *assets,
                               snap_controllers_t *controllers)
{
    (void)controllers;
    for (int pair = 0; pair < 3; pair++) {
        double x = (double)pair * 20.0;
        for (int side = 0; side < 2; side++) {
            bool ball = ((pair + side) & 1) == 0;
            double sign = side ? -1.0 : 1.0;
            ivp_object_t *object = snap_create(
                env, assets, ball, x - sign * 2.0, 0.125 * (double)side,
                0.0625 * (double)pair, 0.0);
            if (!object) return false;
            object->material.elasticity = IVP_REAL(0.9);
            if (!snap_push(object, sign * (0.5 + 0.25 * (double)pair), 0.0,
                           0.0, 0.0, 0.25 * (double)pair, 0.5 * sign)) {
                return false;
            }
        }
    }
    return true;
}

/* zero gravity, pairs drifting side by side with a small gap and a slow
 * sideways closing speed: mindists stay near for the whole run, across
 * the 10 s hull manager reset */
static bool snap_build_graze(ivp_environment_t *env, snap_assets_t *assets,
                             snap_controllers_t *controllers)
{
    (void)controllers;
    for (int pair = 0; pair < 3; pair++) {
        double x = (double)pair * 20.0;
        double gap = 0.4 + 0.2 * (double)pair;
        for (int side = 0; side < 2; side++) {
            bool ball = ((pair + side) & 1) != 0;
            double sign = side ? -1.0 : 1.0;
            ivp_object_t *object = snap_create(
                env, assets, ball, x, 0.0, sign * (0.5 + gap / 2.0), 0.0);
            if (!object) return false;
            object->material.elasticity = IVP_REAL(0.9);
            if (!snap_push(object, 0.75, 0.0625 * sign,
                           -sign * (0.004 + 0.002 * (double)pair),
                           0.0, 0.0, 0.25 * sign)) {
                return false;
            }
        }
    }
    return true;
}

/* balls thrown along the ground, bouncing apart */
static bool snap_build_bounce(ivp_environment_t *env, snap_assets_t *assets,
                              snap_controllers_t *controllers)
{
    (void)controllers;
    if (!snap_create_ground(env, assets)) return false;
    for (int i = 0; i < 4; i++) {
        ivp_object_t *object = snap_create(env, assets, true,
                                           (double)i * 10.0, -1.0 - (double)i,
                                           0.0, 0.0);
        if (!snap_push(object, 1.0, 0.0, 0.5, 0.0, 0.0, 0.0)) return false;
    }
    return true;
}

/* gravity, two piles on the ground like the locality test's linked
 * islands: one joined by springs, one by ball-socket constraints */
static bool snap_add_pile(ivp_environment_t *env, snap_assets_t *assets,
                          double x, ivp_object_t *out[6])
{
    for (int local = 0; local < 4; local++) {
        out[local] = snap_create(env, assets, false,
                                 x + ((local & 1) ? 0.55 : -0.55), 0.9,
                                 (local & 2) ? 0.55 : -0.55, 0.02);
        if (!out[local]) return false;
    }
    out[4] = snap_create(env, assets, false, x + 0.1, -0.4, -0.1, 0.02);
    out[5] = snap_create(env, assets, true, x + 3.0, 1.05, 0.0, 0.02);
    return out[4] && out[5] &&
           snap_push(out[4], 0.25, 0.0, 0.0, 0.0, 0.0, 0.0) &&
           snap_push(out[5], 0.75, 0.0, 0.25, 0.0, 0.0, 0.0);
}

static bool snap_link_spring(ivp_environment_t *env, ivp_spring_state_t *spring,
                             ivp_object_t *a, ivp_object_t *b,
                             double rest_length, double stiffness,
                             double damping)
{
    ivp_vec3r_t center = ivp_vec3r_zero();
    ivp_spring_state_init(spring);
    ivp_spring_state_setup(spring, a, &center, b, &center,
                           (ivp_real_t)stiffness, (ivp_real_t)damping,
                           (ivp_real_t)rest_length);
    return ivp_controller_handle_is_valid(
        ivp_environment_announce_dependent_controller(
            env, 100, ivp_controller_spring, spring, ivp_spring_discovery));
}

static bool snap_link_ball_socket(ivp_environment_t *env,
                                  ivp_constraint_state_t *constraint,
                                  ivp_object_t *a, ivp_object_t *b)
{
    ivp_vec3r_t pa, pb, anchor;
    ivp_object_get_position(a, &pa);
    ivp_object_get_position(b, &pb);
    ivp_vec3r_set(&anchor, (pa.k[0] + pb.k[0]) * IVP_REAL(0.5),
                  (pa.k[1] + pb.k[1]) * IVP_REAL(0.5),
                  (pa.k[2] + pb.k[2]) * IVP_REAL(0.5));
    ivp_constraint_init_ball_socket_ws(constraint, a, &anchor, b);
    return ivp_controller_handle_is_valid(
        ivp_environment_announce_dependent_controller(
            env, 50, ivp_controller_constraint, constraint,
            ivp_constraint_discovery));
}

static bool snap_build_linked(ivp_environment_t *env, snap_assets_t *assets,
                              snap_controllers_t *controllers)
{
    ivp_object_t *springs[6];
    ivp_object_t *joints[6];
    return snap_create_ground(env, assets) &&
           snap_add_pile(env, assets, -6.0, springs) &&
           snap_add_pile(env, assets, 6.0, joints) &&
           snap_link_spring(env, &controllers->springs[0], springs[0],
                            springs[1], 0.8, 30.0, 3.0) &&
           snap_link_spring(env, &controllers->springs[1], springs[4],
                            springs[5], 2.0, 10.0, 1.0) &&
           snap_link_ball_socket(env, &controllers->constraints[0], joints[1],
                                 joints[3]) &&
           snap_link_ball_socket(env, &controllers->constraints[1], joints[0],
                                 joints[4]);
}

static const snap_scene_t snap_scenes[] = {
    { "drift", false, snap_build_drift, 8, false },
    { "fall", true, snap_build_fall, 8, false },
    { "settle", false, snap_build_settle, 8, true },
    { "collide", false, snap_build_collide, 8, false },
    { "graze", false, snap_build_graze, 8, false },
    { "pile", true, snap_build_pile, 8, true },
    { "bounce", true, snap_build_bounce, 8, false },
    { "linked", true, snap_build_linked, 8, false },
};

/* ── Worlds ──────────────────────────────────────────────────────────── */

static bool snap_assets_init(snap_assets_t *assets)
{
    memset(assets, 0, sizeof(*assets));
    assets->ground_surface = ivp_build_box_surface(NULL, 40.0f, 0.5f, 40.0f);
    assets->cube_surface = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    if (!assets->ground_surface || !assets->cube_surface) return false;
    ivp_surman_polygon_init(&assets->ground_surman, assets->ground_surface);
    ivp_surman_polygon_init(&assets->cube_surman, assets->cube_surface);
    return true;
}

static void snap_assets_destroy(snap_assets_t *assets)
{
    if (assets->cube_surface) {
        ivp_compact_surface_free(NULL, assets->cube_surface);
    }
    if (assets->ground_surface) {
        ivp_compact_surface_free(NULL, assets->ground_surface);
    }
    memset(assets, 0, sizeof(*assets));
}

static void snap_config(const snap_scene_t *scene,
                        ivp_environment_config_t *config)
{
    ivp_environment_config_init(config);
    config->disable_performance_counter = true;
    if (!scene->gravity) ivp_vec3r_set_zero(&config->gravity);
}

/* a world and the controller state storage it points to */
typedef struct snap_world {
    ivp_environment_t *env;
    snap_controllers_t *controllers;   /* original worlds */
    void **states;                     /* restored worlds: one per controller */
    int state_count;
} snap_world_t;

static void snap_world_destroy(snap_world_t *world)
{
    if (world->env) ivp_environment_destroy(world->env);
    free(world->controllers);
    for (int i = 0; i < world->state_count; i++) free(world->states[i]);
    free(world->states);
    memset(world, 0, sizeof(*world));
}

static bool snap_build_world(const snap_scene_t *scene, snap_assets_t *assets,
                             snap_world_t *world)
{
    memset(world, 0, sizeof(*world));
    ivp_environment_config_t config;
    snap_config(scene, &config);
    world->controllers =
        (snap_controllers_t *)calloc(1, sizeof(*world->controllers));
    world->env = world->controllers ? ivp_environment_create(&config) : NULL;
    if (!world->env || !scene->build(world->env, assets, world->controllers)) {
        snap_world_destroy(world);
        return false;
    }
    return true;
}

/* restore with fresh storage for every captured controller state */
static bool snap_restore_world(const snap_scene_t *scene,
                               const ivp_world_snapshot_t *snapshot,
                               snap_world_t *world, ivp_result_t *result)
{
    memset(world, 0, sizeof(*world));
    int count = ivp_world_snapshot_controller_count(snapshot);
    world->states = (void **)calloc((size_t)(count > 0 ? count : 1),
                                    sizeof(*world->states));
    if (!world->states) return false;
    world->state_count = count;
    for (int i = 0; i < count; i++) {
        world->states[i] =
            calloc(1, ivp_world_snapshot_controller_state_size(snapshot, i));
        if (!world->states[i]) {
            snap_world_destroy(world);
            return false;
        }
    }
    ivp_world_snapshot_bindings_t bindings;
    bindings.controller_states = world->states;
    bindings.controller_handles = NULL;
    bindings.controller_count = count;
    ivp_environment_config_t config;
    snap_config(scene, &config);
    world->env = ivp_world_snapshot_restore(snapshot, &config, &bindings, result);
    if (!world->env) {
        snap_world_destroy(world);
        return false;
    }
    return true;
}

static int snap_compare_order(const void *left, const void *right)
{
    uint64_t a = ivp_object_order_key(*(const ivp_object_t *const *)left);
    uint64_t b = ivp_object_order_key(*(const ivp_object_t *const *)right);
    return a < b ? -1 : a > b ? 1 : 0;
}

static void snap_capture_state(const ivp_environment_t *env,
                               snap_state_t *state)
{
    /* define every byte, including padding */
    memset(state, 0, sizeof(*state));
    state->time = ivp_environment_get_time(env);
    ivp_object_t *ordered[SNAP_MAX_OBJECTS];
    int count = ivp_environment_get_object_count(env);
    if (count > SNAP_MAX_OBJECTS) count = SNAP_MAX_OBJECTS;
    for (int i = 0; i < count; i++) {
        ordered[i] = ivp_environment_get_object(env, i);
    }
    qsort(ordered, (size_t)count, sizeof(ordered[0]), snap_compare_order);
    state->object_count = count;
    for (int i = 0; i < count; i++) {
        const ivp_core_t *core = ordered[i]->physical_core;
        snap_object_state_t *out = &state->objects[i];
        out->time_of_last_psi = core->time_of_last_psi;
        out->movement_state = (int)core->movement_state;
        out->object_movement_state = (int)ordered[i]->object_movement_state;
        for (int axis = 0; axis < 3; axis++) {
            out->position[axis] = core->pos_world_f_core_last_psi.k[axis];
            out->speed[axis] = core->speed.k[axis];
            out->rot_speed[axis] = core->rot_speed.k[axis];
        }
        out->orientation[0] = core->q_world_f_core_last_psi.x;
        out->orientation[1] = core->q_world_f_core_last_psi.y;
        out->orientation[2] = core->q_world_f_core_last_psi.z;
        out->orientation[3] = core->q_world_f_core_last_psi.w;
        out->next_orientation[0] = core->q_world_f_core_next_psi.x;
        out->next_orientation[1] = core->q_world_f_core_next_psi.y;
        out->next_orientation[2] = core->q_world_f_core_next_psi.z;
        out->next_orientation[3] = core->q_world_f_core_next_psi.w;
    }
}

/* first difference as (object, field); object -1 for world fields */
static bool snap_states_equal(const snap_state_t *a, const snap_state_t *b,
                              int *object, const char **field)
{
    *object = -1;
    *field = NULL;
    if (memcmp(&a->time, &b->time, sizeof(a->time)) != 0) {
        *field = "time";
        return false;
    }
    if (a->object_count != b->object_count) {
        *field = "object_count";
        return false;
    }
    for (int i = 0; i < a->object_count; i++) {
        for (size_t f = 0; f < sizeof(snap_fields) / sizeof(snap_fields[0]);
             f++) {
            const snap_field_t *spec = &snap_fields[f];
            if (memcmp((const unsigned char *)&a->objects[i] + spec->offset,
                       (const unsigned char *)&b->objects[i] + spec->offset,
                       spec->size) != 0) {
                *object = i;
                *field = spec->name;
                return false;
            }
        }
    }
    return true;
}

/* ── Driver ──────────────────────────────────────────────────────────── */

static int snap_frozen_count(const snap_state_t *state)
{
    int frozen = 0;
    for (int i = 0; i < state->object_count; i++) {
        if (state->objects[i].object_movement_state == IVP_MT_NOT_SIM) {
            frozen++;
        }
    }
    return frozen;
}

/* some capture point restores frozen next to simulated objects and
 * continues long enough for the rest to freeze or move on */
static bool snap_covers_partial_freeze(const snap_state_t *expected)
{
    for (int i = 0; i < SNAP_CAPTURE_COUNT; i++) {
        int capture = snap_capture_points[i];
        const snap_state_t *state = &expected[capture - 1];
        int frozen = snap_frozen_count(state);
        if (frozen > 0 && frozen < state->object_count &&
            SNAP_PSI_COUNT - capture >= SNAP_MIN_CONTINUATION) {
            return true;
        }
    }
    return false;
}

/* Returns the PSI of the first difference after restoring at capture,
 * SNAP_PSI_COUNT if the continuation matched, 0 if the capture was
 * rejected as unsupported, or -1 on failure. */
static int snap_continue_from(const snap_scene_t *scene, snap_assets_t *assets,
                              const snap_state_t *expected, int capture)
{
    snap_world_t original;
    if (!snap_build_world(scene, assets, &original)) return -1;
    for (int psi = 0; psi < capture; psi++) {
        ivp_environment_simulate_one_psi(original.env);
    }
    ivp_result_t result = IVP_OK;
    ivp_world_snapshot_t *snapshot =
        ivp_world_snapshot_capture(original.env, &result);
    snap_world_destroy(&original);
    if (!snapshot) {
        fprintf(stderr, "snapshot %s@%d: capture %s: %s\n", scene->name,
                capture,
                result == IVP_ERR_INVALID_STATE ? "rejected" : "failed",
                ivp_result_str(result));
        return result == IVP_ERR_INVALID_STATE ? 0 : -1;
    }

    snap_world_t world;
    bool restored_ok = snap_restore_world(scene, snapshot, &world, &result);
    ivp_world_snapshot_destroy(snapshot);
    if (!restored_ok) {
        fprintf(stderr, "snapshot %s@%d: restore failed: %s\n", scene->name,
                capture, ivp_result_str(result));
        return -1;
    }
    ivp_environment_t *restored = world.env;

    int first_difference = SNAP_PSI_COUNT;
    snap_state_t actual;
    snap_capture_state(restored, &actual);
    int object = -1;
    const char *field = NULL;
    /* expected[k - 1] is the state after k PSIs */
    if (!snap_states_equal(&expected[capture - 1], &actual, &object,
                           &field)) {
        first_difference = capture;
    }
    for (int psi = capture; psi < SNAP_PSI_COUNT &&
                            first_difference == SNAP_PSI_COUNT; psi++) {
        ivp_environment_simulate_one_psi(restored);
        snap_capture_state(restored, &actual);
        if (!snap_states_equal(&expected[psi], &actual, &object, &field)) {
            first_difference = psi + 1;
        }
    }
    if (first_difference < SNAP_PSI_COUNT) {
        fprintf(stderr,
                "snapshot %s@%d: diverges after %d PSIs: object %d field %s\n",
                scene->name, capture, first_difference, object,
                field ? field : "none");
    }
    snap_world_destroy(&world);
    return first_difference;
}

typedef struct snap_coverage {
    int accepted;
    int rejected;
    int with_exact_mindists;
    int with_controllers;
} snap_coverage_t;

/* restore a capture of the running world and compare `steps` PSIs from
 * expected[psi]; *accepted reports whether the capture was supported */
static bool snap_check_dense(const snap_scene_t *scene, ivp_environment_t *env,
                             const snap_state_t *expected, int psi, int steps,
                             snap_coverage_t *coverage)
{
    ivp_result_t result = IVP_OK;
    ivp_world_snapshot_t *snapshot = ivp_world_snapshot_capture(env, &result);
    if (!snapshot) {
        if (result != IVP_ERR_INVALID_STATE) {
            fprintf(stderr, "snapshot %s: dense capture after %d PSIs "
                            "failed: %s\n",
                    scene->name, psi, ivp_result_str(result));
            return false;
        }
        coverage->rejected++;
        return true;
    }
    coverage->accepted++;
    if (env->pl->mindist_mgr.exact_mindists) coverage->with_exact_mindists++;
    if (ivp_world_snapshot_controller_count(snapshot) > 0) {
        coverage->with_controllers++;
    }

    snap_world_t world;
    bool restored_ok = snap_restore_world(scene, snapshot, &world, &result);
    ivp_world_snapshot_destroy(snapshot);
    ivp_environment_t *restored = world.env;
    if (!restored_ok) {
        fprintf(stderr, "snapshot %s: dense restore after %d PSIs failed: "
                        "%s\n",
                scene->name, psi, ivp_result_str(result));
        return false;
    }
    bool ok = true;
    snap_state_t actual;
    for (int step = 0; step < steps && ok; step++) {
        ivp_environment_simulate_one_psi(restored);
        snap_capture_state(restored, &actual);
        int object = -1;
        const char *field = NULL;
        if (!snap_states_equal(&expected[psi + step], &actual, &object,
                               &field)) {
            fprintf(stderr, "snapshot %s: dense capture after %d PSIs "
                            "diverges %d PSIs later: object %d field %s\n",
                    scene->name, psi, step + 1, object,
                    field ? field : "none");
            ok = false;
        }
    }
    snap_world_destroy(&world);
    return ok;
}

/* the uninterrupted world, captured after every PSI */
static bool snap_run_dense(const snap_scene_t *scene, snap_assets_t *assets,
                           const snap_state_t *expected,
                           snap_coverage_t *coverage)
{
    snap_world_t world;
    if (!snap_build_world(scene, assets, &world)) return false;
    ivp_environment_t *env = world.env;
    bool ok = true;
    snap_state_t actual;
    for (int psi = 0; psi < SNAP_PSI_COUNT && ok; psi++) {
        ivp_environment_simulate_one_psi(env);
        snap_capture_state(env, &actual);
        int object = -1;
        const char *field = NULL;
        if (!snap_states_equal(&expected[psi], &actual, &object, &field)) {
            fprintf(stderr, "snapshot %s: capturing changed the world after "
                            "%d PSIs: object %d field %s\n",
                    scene->name, psi + 1, object, field ? field : "none");
            ok = false;
            break;
        }
        int remaining = SNAP_PSI_COUNT - (psi + 1);
        if (remaining == 0) break;
        ok = snap_check_dense(scene, env, expected, psi + 1,
                              remaining < SNAP_DENSE_CONTINUATION
                                  ? remaining : SNAP_DENSE_CONTINUATION,
                              coverage);
    }
    snap_world_destroy(&world);
    fprintf(stderr, "snapshot %s: dense %d accepted (%d with exact "
                    "mindists, %d with controllers), %d rejected\n",
            scene->name, coverage->accepted, coverage->with_exact_mindists,
            coverage->with_controllers, coverage->rejected);
    return ok;
}

static bool snap_run_scene(const snap_scene_t *scene, snap_coverage_t *coverage)
{
    snap_assets_t assets;
    snap_state_t *expected =
        (snap_state_t *)calloc(SNAP_PSI_COUNT, sizeof(*expected));
    if (!expected || !snap_assets_init(&assets)) {
        free(expected);
        snap_assets_destroy(&assets);
        return false;
    }

    bool ok = true;
    snap_world_t world;
    if (!snap_build_world(scene, &assets, &world)) {
        ok = false;
        goto cleanup;
    }
    for (int psi = 0; psi < SNAP_PSI_COUNT; psi++) {
        ivp_environment_simulate_one_psi(world.env);
        snap_capture_state(world.env, &expected[psi]);
    }
    snap_world_destroy(&world);
    if (scene->freezes && !snap_covers_partial_freeze(expected)) {
        fprintf(stderr, "snapshot %s: no capture point restores frozen and "
                        "simulated objects together\n", scene->name);
        ok = false;
        goto cleanup;
    }

    int exact = 0;
    for (int i = 0; i < SNAP_CAPTURE_COUNT; i++) {
        int reached = snap_continue_from(scene, &assets, expected,
                                         snap_capture_points[i]);
        if (reached < 0) {
            ok = false;
            goto cleanup;
        }
        if (reached == SNAP_PSI_COUNT) exact++;
    }
    fprintf(stderr, "snapshot %s: %d/%d capture points continue exactly\n",
            scene->name, exact, SNAP_CAPTURE_COUNT);
    if (!snap_run_dense(scene, &assets, expected, coverage)) ok = false;
    if (exact != scene->exact_points) {
        fprintf(stderr, "snapshot %s: expected %d exact capture points%s\n",
                scene->name, scene->exact_points,
                exact > scene->exact_points ? "; raise the count" : "");
        ok = false;
    }

cleanup:
    snap_assets_destroy(&assets);
    free(expected);
    return ok;
}

IVP_TEST(restored_worlds_continue_exactly)
{
    bool ok = true;
    int with_exact_mindists = 0;
    int with_controllers = 0;
    for (size_t i = 0; i < sizeof(snap_scenes) / sizeof(snap_scenes[0]);
         i++) {
        snap_coverage_t coverage;
        memset(&coverage, 0, sizeof(coverage));
        ok = snap_run_scene(&snap_scenes[i], &coverage) && ok;
        with_exact_mindists += coverage.with_exact_mindists;
        with_controllers += coverage.with_controllers;
    }
    IVP_ASSERT_TRUE(ok);
    /* the exact-mindist restore path must be exercised */
    IVP_ASSERT_TRUE(with_exact_mindists > 0);
    /* and so must the controller rebinding path */
    IVP_ASSERT_TRUE(with_controllers > 0);
}

/* ── Allocation failures during restore ──────────────────────────────── */

#define SNAP_FAULT_CONTINUATION 20

/* Restore `snapshot` with a world allocator failing at its fail_at-th
 * allocation.  A failed restore must report out of memory and free
 * everything; a restore that succeeds must continue exactly.  *done turns
 * true once the restore needed fewer allocations than fail_at. */
static bool snap_restore_failing_at(const snap_scene_t *scene,
                                    const ivp_world_snapshot_t *snapshot,
                                    const snap_state_t *expected, int capture,
                                    int fail_at, bool *done)
{
    ivp_test_fault_allocator_t fault;
    ivp_test_fault_allocator_init_tracked(&fault, NULL);
    ivp_test_fault_allocator_reset(&fault, fail_at);

    int count = ivp_world_snapshot_controller_count(snapshot);
    void *states[SNAP_MAX_LINKS * 2];
    if (count > SNAP_MAX_LINKS * 2) return false;
    for (int i = 0; i < count; i++) {
        states[i] = calloc(1, ivp_world_snapshot_controller_state_size(snapshot, i));
    }
    ivp_world_snapshot_bindings_t bindings;
    bindings.controller_states = states;
    bindings.controller_handles = NULL;
    bindings.controller_count = count;
    ivp_environment_config_t config;
    snap_config(scene, &config);
    config.allocator = &fault.allocator;

    ivp_result_t result = IVP_OK;
    ivp_environment_t *env =
        ivp_world_snapshot_restore(snapshot, &config, &bindings, &result);
    *done = fault.attempts < fail_at;
    /* only the restore fails; the continuation checks what it built */
    fault.fail_at = INT_MAX;
    bool ok = true;
    if (!env) {
        if (result != IVP_ERR_OUT_OF_MEMORY) {
            fprintf(stderr, "snapshot %s@%d: restore failing at allocation %d "
                            "reported %s\n",
                    scene->name, capture, fail_at, ivp_result_str(result));
            ok = false;
        }
    } else {
        snap_state_t actual;
        for (int step = 0; step < SNAP_FAULT_CONTINUATION && ok; step++) {
            ivp_environment_simulate_one_psi(env);
            snap_capture_state(env, &actual);
            int object = -1;
            const char *field = NULL;
            if (!snap_states_equal(&expected[capture + step], &actual, &object,
                                   &field)) {
                fprintf(stderr, "snapshot %s@%d: restore failing at allocation "
                                "%d succeeded but diverges %d PSIs later: "
                                "object %d field %s\n",
                        scene->name, capture, fail_at, step + 1, object,
                        field ? field : "none");
                ok = false;
            }
        }
        ivp_environment_destroy(env);
    }
    if (fault.live_bytes != 0) {
        fprintf(stderr, "snapshot %s@%d: restore failing at allocation %d "
                        "leaks %zu bytes\n",
                scene->name, capture, fail_at, fault.live_bytes);
        ok = false;
    }
    for (int i = 0; i < count; i++) free(states[i]);
    return ok;
}

/* every allocation of the restore, failed once each */
static bool snap_check_restore_faults(const snap_scene_t *scene, int capture,
                                      int *allocation_count)
{
    snap_assets_t assets;
    snap_state_t *expected = (snap_state_t *)calloc(
        (size_t)(capture + SNAP_FAULT_CONTINUATION), sizeof(*expected));
    snap_world_t world;
    if (!expected || !snap_assets_init(&assets) ||
        !snap_build_world(scene, &assets, &world)) {
        free(expected);
        snap_assets_destroy(&assets);
        return false;
    }
    ivp_world_snapshot_t *snapshot = NULL;
    for (int psi = 0; psi < capture + SNAP_FAULT_CONTINUATION; psi++) {
        if (psi == capture) snapshot = ivp_world_snapshot_capture(world.env, NULL);
        ivp_environment_simulate_one_psi(world.env);
        snap_capture_state(world.env, &expected[psi]);
    }
    bool ok = snapshot != NULL;
    int fail_at = 1;
    for (bool done = false; ok && !done; fail_at++) {
        ok = snap_restore_failing_at(scene, snapshot, expected, capture,
                                     fail_at, &done);
    }
    *allocation_count = fail_at - 2;
    ivp_world_snapshot_destroy(snapshot);
    snap_world_destroy(&world);
    snap_assets_destroy(&assets);
    free(expected);
    return ok;
}

static const snap_scene_t *snap_find_scene(const char *name)
{
    for (size_t i = 0; i < sizeof(snap_scenes) / sizeof(snap_scenes[0]); i++) {
        if (strcmp(snap_scenes[i].name, name) == 0) return &snap_scenes[i];
    }
    return NULL;
}

/* Captures holding contacts, frozen units, exact mindists, springs and
 * constraints: every allocation of their restore may fail. */
IVP_TEST(restore_survives_every_allocation_failure)
{
    static const struct { const char *scene; int capture; } cases[] = {
        { "pile", 230 }, { "bounce", 60 }, { "linked", 150 }, { "collide", 90 }
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        const snap_scene_t *scene = snap_find_scene(cases[i].scene);
        IVP_ASSERT_TRUE(scene != NULL);
        int allocations = 0;
        bool ok = snap_check_restore_faults(scene, cases[i].capture, &allocations);
        fprintf(stderr, "snapshot %s@%d: %d restore allocations failed in turn\n",
                cases[i].scene, cases[i].capture, allocations);
        IVP_ASSERT_TRUE(ok);
        IVP_ASSERT_TRUE(allocations > 0);
    }
}

static void snap_noop_controller(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *object, void *context)
{
    (void)event_sim;
    (void)object;
    (void)context;
}

/* State the snapshot cannot restore yet is rejected, not dropped. */
IVP_TEST(capture_rejects_unsupported_state)
{
    snap_assets_t assets;
    IVP_ASSERT_TRUE(snap_assets_init(&assets));
    snap_world_t world;
    IVP_ASSERT_TRUE(snap_build_world(&snap_scenes[0], &assets, &world));
    ivp_environment_t *env = world.env;
    ivp_controller_handle_t controller = ivp_environment_add_controller(
        env, IVP_CP_MOTION, snap_noop_controller, NULL);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(controller));
    ivp_environment_simulate_one_psi(env);
    ivp_result_t result = IVP_OK;
    ivp_world_snapshot_t *snapshot = ivp_world_snapshot_capture(env, &result);
    IVP_ASSERT_TRUE(snapshot == NULL);
    IVP_ASSERT_EQ(result, IVP_ERR_INVALID_STATE);
    snap_world_destroy(&world);
    snap_assets_destroy(&assets);
}

int main(void)
{
    IVP_REGISTER(restored_worlds_continue_exactly);
    IVP_REGISTER(capture_rejects_unsupported_state);
    IVP_REGISTER(restore_survives_every_allocation_failure);
    IVP_RUN_ALL();
}
