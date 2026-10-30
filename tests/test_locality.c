/* test_locality.c -- scene edits must not change objects outside their
 * causal interaction closure (built only with IVP_EDIT_LOCALITY).
 *
 * Each case runs a base world and an edited world that differ by one object
 * (added or removed, at creation or mid-run).  A global collision listener
 * records every impact and every friction contact interval in both worlds.
 * The closure starts at the edited object at the edit time and spreads along
 * those interactions in time order; unmoveable objects other than the edited
 * one neither receive nor carry influence.  Springs and constraints link
 * their two objects for the rest of the run.  A buoyancy pool registered as
 * one independent controller links nothing: its members only share the
 * controller.  Every object outside the closure must keep a bit-identical
 * visible state at every PSI. */
#include "ivp_test.h"

#include <ivp/builder/ivp_build_surface.h>
#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/controllers/ivp_controller_buoyancy.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/events/ivp_listener_collision.h>
#include <ivp/geometry/ivp_surman.h>

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define LOC_ISLAND_COUNT 9
#define LOC_SPRING_ISLAND 6
#define LOC_CONSTRAINT_ISLAND 7
#define LOC_POOL_ISLAND 8
#define LOC_OBJECTS_PER_ISLAND 6
#define LOC_GROUND_ID 0
#define LOC_BASE_COUNT (1 + LOC_ISLAND_COUNT * LOC_OBJECTS_PER_ISLAND)
#define LOC_EDIT_ID LOC_BASE_COUNT
#define LOC_OBJECT_COUNT (LOC_BASE_COUNT + 1)
#define LOC_PSI_COUNT 600
#define LOC_REPORT_LIMIT 8

typedef enum loc_shape {
    LOC_SHAPE_GROUND = 0,
    LOC_SHAPE_CUBE,
    LOC_SHAPE_BALL,
    LOC_SHAPE_PILLAR
} loc_shape_t;

typedef struct loc_spec {
    loc_shape_t shape;
    double position[3];
    double speed[3];
} loc_spec_t;

typedef enum loc_edit_kind {
    LOC_EDIT_ADD = 0,
    LOC_EDIT_REMOVE
} loc_edit_kind_t;

typedef struct loc_edit {
    const char *name;
    loc_edit_kind_t kind;
    int psi;             /* 0: applied while the scene is created */
    loc_spec_t spec;     /* LOC_EDIT_ADD */
    int create_after;    /* LOC_EDIT_ADD at psi 0: base id created before it */
    int target;          /* LOC_EDIT_REMOVE: base id */
    bool expect_spread;  /* the edited object must reach another object */
    bool in_pool;        /* LOC_EDIT_ADD at psi 0: joins the buoyancy pool */
} loc_edit_t;

typedef struct loc_interaction {
    int a;
    int b;
    ivp_time_t start;
    ivp_time_t end;
    const struct ivp_contact_pt *contact;  /* open friction contact, or NULL */
} loc_interaction_t;

typedef struct loc_recorder {
    loc_interaction_t *items;
    size_t count;
    size_t capacity;
    bool failed;
} loc_recorder_t;

typedef struct loc_assets {
    ivp_compact_surface_t *ground_surface;
    ivp_compact_surface_t *cube_surface;
    ivp_surman_polygon_t ground_surman;
    ivp_surman_polygon_t cube_surman;
} loc_assets_t;

typedef struct loc_world {
    ivp_environment_t *environment;
    ivp_object_t *objects[LOC_OBJECT_COUNT];
    bool unmoveable[LOC_OBJECT_COUNT];
    ivp_collision_listener_t listener;
    loc_recorder_t recorder;
    ivp_spring_state_t springs[2];
    ivp_constraint_state_t constraints[2];
    ivp_buoyancy_state_t pool;
    ivp_object_t *pool_objects[LOC_OBJECT_COUNT];
    bool pool_ready;
} loc_world_t;

typedef struct loc_object_state {
    bool present;
    int movement_state;
    int object_movement_state;
    ivp_time_t time_of_last_psi;
    ivp_real_t position[3];
    ivp_real_t speed[3];
    float rot_speed[3];
    ivp_real_t orientation[4];
    ivp_real_t next_orientation[4];
} loc_object_state_t;

typedef struct loc_snapshot {
    ivp_time_t time;
    loc_object_state_t objects[LOC_OBJECT_COUNT];
} loc_snapshot_t;

typedef struct loc_field {
    const char *name;
    size_t offset;
    size_t size;
} loc_field_t;

#define LOC_FIELD(member) \
    { #member, offsetof(loc_object_state_t, member), \
      sizeof(((loc_object_state_t *)0)->member) }

static const loc_field_t loc_object_fields[] = {
    LOC_FIELD(present),
    LOC_FIELD(movement_state),
    LOC_FIELD(object_movement_state),
    LOC_FIELD(time_of_last_psi),
    LOC_FIELD(position),
    LOC_FIELD(speed),
    LOC_FIELD(rot_speed),
    LOC_FIELD(orientation),
    LOC_FIELD(next_orientation)
};

#undef LOC_FIELD

/* ── Scene ───────────────────────────────────────────────────────────── */

static double loc_island_x(int island) { return (double)(island % 3) * 24.0 - 24.0; }
static double loc_island_z(int island) { return (double)(island / 3) * 24.0 - 24.0; }

static int loc_island_id(int island, int local)
{
    return 1 + island * LOC_OBJECTS_PER_ISLAND + local;
}

/* Y is down: the ground top is at y = 1.5.  Each island has four cubes on
 * the ground with small gaps, one cube dropped on top of them, and one ball
 * rolling away from the pile.  loc_world_link adds springs, constraints and
 * the buoyancy pool to the last three islands. */
static void loc_base_spec(int id, loc_spec_t *spec)
{
    memset(spec, 0, sizeof(*spec));
    if (id == LOC_GROUND_ID) {
        spec->shape = LOC_SHAPE_GROUND;
        spec->position[1] = 2.0;
        return;
    }
    int island = (id - 1) / LOC_OBJECTS_PER_ISLAND;
    int local = (id - 1) % LOC_OBJECTS_PER_ISLAND;
    double x = loc_island_x(island);
    double z = loc_island_z(island);
    /* vary the islands so they do not settle in lockstep */
    double skew = 0.03125 * (double)island;
    if (local < 4) {
        spec->shape = LOC_SHAPE_CUBE;
        spec->position[0] = x + ((local & 1) ? 0.55 : -0.55);
        spec->position[1] = 0.9 - skew;
        spec->position[2] = z + ((local & 2) ? 0.55 : -0.55);
    } else if (local == 4) {
        spec->shape = LOC_SHAPE_CUBE;
        spec->position[0] = x + skew;
        spec->position[1] = -0.4 - 2.0 * skew;
        spec->position[2] = z - skew;
        spec->speed[0] = 0.25 * (double)((island % 3) - 1);
    } else {
        spec->shape = LOC_SHAPE_BALL;
        spec->position[0] = x + 3.0;
        spec->position[1] = 1.05;
        spec->position[2] = z;
        spec->speed[0] = 0.5 + 0.125 * (double)island;
        spec->speed[2] = 0.25 * (double)(island & 1);
    }
}

static bool loc_assets_init(loc_assets_t *assets)
{
    memset(assets, 0, sizeof(*assets));
    assets->ground_surface = ivp_build_box_surface(NULL, 80.0f, 0.5f, 80.0f);
    assets->cube_surface = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    if (!assets->ground_surface || !assets->cube_surface) return false;
    ivp_surman_polygon_init(&assets->ground_surman, assets->ground_surface);
    ivp_surman_polygon_init(&assets->cube_surman, assets->cube_surface);
    return true;
}

static void loc_assets_destroy(loc_assets_t *assets)
{
    if (assets->cube_surface) {
        ivp_compact_surface_free(NULL, assets->cube_surface);
    }
    if (assets->ground_surface) {
        ivp_compact_surface_free(NULL, assets->ground_surface);
    }
    memset(assets, 0, sizeof(*assets));
}

/* ── Interaction recorder ────────────────────────────────────────────── */

static int loc_object_id(const loc_world_t *world, const ivp_object_t *object)
{
    for (int id = 0; id < LOC_OBJECT_COUNT; id++) {
        if (world->objects[id] == object) return id;
    }
    return -1;
}

static loc_interaction_t *loc_record_ids(loc_world_t *world, int ia, int ib,
                                         ivp_time_t start, ivp_time_t end)
{
    loc_recorder_t *recorder = &world->recorder;
    if (recorder->count == recorder->capacity) {
        size_t capacity = recorder->capacity ? recorder->capacity * 2 : 256;
        loc_interaction_t *items = (loc_interaction_t *)realloc(
            recorder->items, capacity * sizeof(*items));
        if (!items) {
            recorder->failed = true;
            return NULL;
        }
        recorder->items = items;
        recorder->capacity = capacity;
    }
    loc_interaction_t *item = &recorder->items[recorder->count++];
    item->a = ia;
    item->b = ib;
    item->start = start;
    item->end = end;
    item->contact = NULL;
    return item;
}

static loc_interaction_t *loc_record(loc_world_t *world,
                                     const ivp_object_t *a,
                                     const ivp_object_t *b,
                                     ivp_time_t start, ivp_time_t end)
{
    int ia = loc_object_id(world, a);
    int ib = loc_object_id(world, b);
    if (ia < 0 || ib < 0) {
        world->recorder.failed = true;
        return NULL;
    }
    return loc_record_ids(world, ia, ib, start, end);
}

static void loc_on_impact(void *context, const ivp_collision_event_t *event)
{
    loc_world_t *world = (loc_world_t *)context;
    ivp_time_t now = ivp_environment_get_time(world->environment);
    (void)loc_record(world, event->obj_a, event->obj_b, now, now);
}

static void loc_on_contact_created(void *context,
                                   const ivp_friction_event_t *event)
{
    loc_world_t *world = (loc_world_t *)context;
    loc_interaction_t *item = loc_record(
        world, event->obj_a, event->obj_b,
        ivp_environment_get_time(world->environment), INFINITY);
    if (item) item->contact = event->friction_handle;
}

static void loc_on_contact_deleted(void *context,
                                   const ivp_friction_event_t *event)
{
    loc_world_t *world = (loc_world_t *)context;
    loc_recorder_t *recorder = &world->recorder;
    for (size_t i = recorder->count; i > 0; i--) {
        loc_interaction_t *item = &recorder->items[i - 1];
        if (item->contact == event->friction_handle) {
            item->end = ivp_environment_get_time(world->environment);
            item->contact = NULL;
            return;
        }
    }
    /* a deletion without a recorded creation would hide an interaction */
    recorder->failed = true;
}

/* ── World ───────────────────────────────────────────────────────────── */

static bool loc_create(loc_world_t *world, loc_assets_t *assets, int id,
                       const loc_spec_t *spec)
{
    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.material.friction = IVP_REAL(0.6);
    object_template.material.elasticity = IVP_REAL(0.1);
    object_template.speed_damp_factor = IVP_REAL(0.02);
    ivp_vec3r_set(&object_template.rot_speed_damp_factor,
                  IVP_REAL(0.02), IVP_REAL(0.02), IVP_REAL(0.02));
    ivp_vec3r_set(&object_template.position,
                  (ivp_real_t)spec->position[0],
                  (ivp_real_t)spec->position[1],
                  (ivp_real_t)spec->position[2]);

    ivp_surman_t *surman = NULL;
    switch (spec->shape) {
        case LOC_SHAPE_GROUND:
        case LOC_SHAPE_PILLAR:
            object_template.type = IVP_OBJECT_POLYGON;
            object_template.physical_unmoveable = true;
            object_template.mass = IVP_REAL(1.0e6);
            ivp_vec3r_set(&object_template.rot_inertia, IVP_REAL(1.0e6),
                          IVP_REAL(1.0e6), IVP_REAL(1.0e6));
            object_template.rot_inertia_is_factor = false;
            surman = spec->shape == LOC_SHAPE_GROUND
                         ? &assets->ground_surman.base
                         : &assets->cube_surman.base;
            break;
        case LOC_SHAPE_CUBE:
            object_template.type = IVP_OBJECT_POLYGON;
            object_template.mass = IVP_REAL(0.6);
            ivp_vec3r_set(&object_template.rot_inertia, IVP_REAL(0.1),
                          IVP_REAL(0.1), IVP_REAL(0.1));
            object_template.rot_inertia_is_factor = false;
            surman = &assets->cube_surman.base;
            break;
        case LOC_SHAPE_BALL:
            object_template.type = IVP_OBJECT_BALL;
            object_template.ball_radius = IVP_REAL(0.4);
            object_template.mass = IVP_REAL(0.8);
            break;
    }

    ivp_object_t *object = ivp_environment_create_object(
        world->environment, &object_template, surman);
    if (!object) return false;
    world->objects[id] = object;
    world->unmoveable[id] = object_template.physical_unmoveable;

    if (!object_template.physical_unmoveable &&
        (spec->speed[0] != 0.0 || spec->speed[1] != 0.0 ||
         spec->speed[2] != 0.0)) {
        ivp_vec3r_t speed;
        ivp_vec3r_set(&speed, (ivp_real_t)spec->speed[0],
                      (ivp_real_t)spec->speed[1], (ivp_real_t)spec->speed[2]);
        ivp_object_async_add_speed_ws(object, &speed);
    }
    return true;
}

static void loc_destroy_object(loc_world_t *world, int id)
{
    /* keep the id mapping alive for callbacks fired during teardown */
    ivp_environment_destroy_object(world->environment, world->objects[id]);
    world->objects[id] = NULL;
}

/* ── Springs, constraints and the buoyancy pool ──────────────────────── */

static bool loc_link_spring(loc_world_t *world, int slot, int a, int b,
                            ivp_real_t rest_length, ivp_real_t stiffness,
                            ivp_real_t damping)
{
    if (!world->objects[a] || !world->objects[b]) return true;
    ivp_spring_state_t *spring = &world->springs[slot];
    ivp_vec3r_t center = ivp_vec3r_zero();
    ivp_spring_state_init(spring);
    ivp_spring_state_setup(spring, world->objects[a], &center,
                           world->objects[b], &center, stiffness, damping,
                           rest_length);
    if (!ivp_controller_handle_is_valid(
            ivp_environment_announce_dependent_controller(
                world->environment, 100, ivp_controller_spring, spring,
                ivp_spring_discovery))) {
        return false;
    }
    return loc_record_ids(world, a, b,
                          ivp_environment_get_time(world->environment),
                          INFINITY) != NULL;
}

/* ball socket at the midpoint of the two objects' initial positions */
static bool loc_link_ball_socket(loc_world_t *world, int slot, int a, int b)
{
    if (!world->objects[a] || !world->objects[b]) return true;
    loc_spec_t spec_a;
    loc_spec_t spec_b;
    loc_base_spec(a, &spec_a);
    loc_base_spec(b, &spec_b);
    ivp_vec3r_t anchor;
    ivp_vec3r_set(&anchor,
                  (ivp_real_t)(0.5 * (spec_a.position[0] + spec_b.position[0])),
                  (ivp_real_t)(0.5 * (spec_a.position[1] + spec_b.position[1])),
                  (ivp_real_t)(0.5 * (spec_a.position[2] + spec_b.position[2])));
    ivp_constraint_state_t *constraint = &world->constraints[slot];
    ivp_constraint_init_ball_socket_ws(constraint, world->objects[a], &anchor,
                                       world->objects[b]);
    if (!ivp_controller_handle_is_valid(
            ivp_environment_announce_dependent_controller(
                world->environment, 50, ivp_controller_constraint,
                constraint, ivp_constraint_discovery))) {
        return false;
    }
    return loc_record_ids(world, a, b,
                          ivp_environment_get_time(world->environment),
                          INFINITY) != NULL;
}

/* One independent buoyancy controller for the pool island (and the edited
 * object when it joins); it records no interaction. */
static bool loc_link_pool(loc_world_t *world, bool include_edit)
{
    int count = 0;
    for (int local = 0; local < LOC_OBJECTS_PER_ISLAND; local++) {
        ivp_object_t *object =
            world->objects[loc_island_id(LOC_POOL_ISLAND, local)];
        if (object) world->pool_objects[count++] = object;
    }
    if (include_edit && world->objects[LOC_EDIT_ID]) {
        world->pool_objects[count++] = world->objects[LOC_EDIT_ID];
    }
    ivp_buoyancy_init(&world->pool, IVP_REAL(0.6), IVP_REAL(1.0));
    world->pool_ready = true;
    ivp_buoyancy_set_objects(&world->pool, world->pool_objects, count);
    return ivp_controller_handle_is_valid(ivp_environment_add_controller(
        world->environment, 200, ivp_controller_buoyancy, &world->pool));
}

static bool loc_world_link(loc_world_t *world, bool pool_includes_edit)
{
    int s0 = loc_island_id(LOC_SPRING_ISLAND, 0);
    int c0 = loc_island_id(LOC_CONSTRAINT_ISLAND, 0);
    return loc_link_spring(world, 0, s0, s0 + 1, IVP_REAL(0.8),
                           IVP_REAL(30.0), IVP_REAL(3.0)) &&
           loc_link_spring(world, 1, s0 + 4, s0 + 5, IVP_REAL(2.0),
                           IVP_REAL(10.0), IVP_REAL(1.0)) &&
           loc_link_ball_socket(world, 0, c0 + 1, c0 + 3) &&
           loc_link_ball_socket(world, 1, c0 + 0, c0 + 4) &&
           loc_link_pool(world, pool_includes_edit);
}

/* edit: NULL builds the base world */
static bool loc_world_init(loc_world_t *world, loc_assets_t *assets,
                           const loc_edit_t *edit)
{
    memset(world, 0, sizeof(*world));
    ivp_environment_config_t config;
    ivp_environment_config_init(&config);
    config.disable_performance_counter = true;
    world->environment = ivp_environment_create(&config);
    if (!world->environment) return false;

    ivp_collision_listener_init(
        &world->listener,
        IVP_COLLISION_CB_PRE_COLLISION | IVP_COLLISION_CB_FRICTION_CREATED |
            IVP_COLLISION_CB_FRICTION_DELETED,
        world);
    world->listener.pre_collision_fn = loc_on_impact;
    world->listener.friction_created_fn = loc_on_contact_created;
    world->listener.friction_deleted_fn = loc_on_contact_deleted;
    if (ivp_environment_add_collision_listener_global(
            world->environment, &world->listener) != 0) {
        return false;
    }

    bool add_now = edit && edit->kind == LOC_EDIT_ADD && edit->psi == 0;
    bool remove_now = edit && edit->kind == LOC_EDIT_REMOVE && edit->psi == 0;
    for (int id = 0; id < LOC_BASE_COUNT; id++) {
        if (!(remove_now && id == edit->target)) {
            loc_spec_t spec;
            loc_base_spec(id, &spec);
            if (!loc_create(world, assets, id, &spec)) return false;
        }
        if (add_now && id == edit->create_after &&
            !loc_create(world, assets, LOC_EDIT_ID, &edit->spec)) {
            return false;
        }
    }
    return loc_world_link(world, add_now && edit->in_pool);
}

static void loc_world_destroy(loc_world_t *world)
{
    if (world->environment) ivp_environment_destroy(world->environment);
    if (world->pool_ready) ivp_buoyancy_state_destroy(&world->pool);
    free(world->recorder.items);
    memset(world, 0, sizeof(*world));
}

static bool loc_apply_late_edit(loc_world_t *world, loc_assets_t *assets,
                                const loc_edit_t *edit)
{
    if (edit->kind == LOC_EDIT_ADD) {
        return loc_create(world, assets, LOC_EDIT_ID, &edit->spec);
    }
    if (!world->objects[edit->target]) return false;
    loc_destroy_object(world, edit->target);
    return true;
}

static void loc_capture(const loc_world_t *world, loc_snapshot_t *snapshot)
{
    /* define every byte, including padding and absent objects */
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->time = ivp_environment_get_time(world->environment);
    for (int id = 0; id < LOC_OBJECT_COUNT; id++) {
        const ivp_object_t *object = world->objects[id];
        if (!object) continue;
        const ivp_core_t *core = object->physical_core;
        loc_object_state_t *state = &snapshot->objects[id];
        state->present = true;
        state->movement_state = (int)core->movement_state;
        state->object_movement_state = (int)object->object_movement_state;
        state->time_of_last_psi = core->time_of_last_psi;
        for (int axis = 0; axis < 3; axis++) {
            state->position[axis] = core->pos_world_f_core_last_psi.k[axis];
            state->speed[axis] = core->speed.k[axis];
            state->rot_speed[axis] = core->rot_speed.k[axis];
        }
        state->orientation[0] = core->q_world_f_core_last_psi.x;
        state->orientation[1] = core->q_world_f_core_last_psi.y;
        state->orientation[2] = core->q_world_f_core_last_psi.z;
        state->orientation[3] = core->q_world_f_core_last_psi.w;
        state->next_orientation[0] = core->q_world_f_core_next_psi.x;
        state->next_orientation[1] = core->q_world_f_core_next_psi.y;
        state->next_orientation[2] = core->q_world_f_core_next_psi.z;
        state->next_orientation[3] = core->q_world_f_core_next_psi.w;
    }
}

/* ── Causal closure ──────────────────────────────────────────────────── */

static void loc_spread(const loc_recorder_t *recorder, const bool *unmoveable,
                       ivp_time_t affected[LOC_OBJECT_COUNT], bool *changed)
{
    for (size_t i = 0; i < recorder->count; i++) {
        const loc_interaction_t *item = &recorder->items[i];
        for (int direction = 0; direction < 2; direction++) {
            int from = direction ? item->b : item->a;
            int to = direction ? item->a : item->b;
            /* unmoveable objects neither receive nor carry influence;
             * the edited object always carries it */
            if (unmoveable[to]) continue;
            if (from != LOC_EDIT_ID && unmoveable[from]) continue;
            if (affected[from] > item->end) continue;
            ivp_time_t reached =
                affected[from] > item->start ? affected[from] : item->start;
            if (reached < affected[to]) {
                affected[to] = reached;
                *changed = true;
            }
        }
    }
}

static void loc_closure(const loc_world_t *base, const loc_world_t *edited,
                        ivp_time_t edit_time,
                        ivp_time_t affected[LOC_OBJECT_COUNT])
{
    bool unmoveable[LOC_OBJECT_COUNT];
    for (int id = 0; id < LOC_OBJECT_COUNT; id++) {
        affected[id] = INFINITY;
        unmoveable[id] = base->unmoveable[id] || edited->unmoveable[id];
    }
    affected[LOC_EDIT_ID] = edit_time;
    bool changed = true;
    while (changed) {
        changed = false;
        loc_spread(&base->recorder, unmoveable, affected, &changed);
        loc_spread(&edited->recorder, unmoveable, affected, &changed);
    }
}

/* ── Comparison ──────────────────────────────────────────────────────── */

static const char *loc_first_different_field(const loc_object_state_t *a,
                                             const loc_object_state_t *b)
{
    for (size_t i = 0; i < sizeof(loc_object_fields) /
                               sizeof(loc_object_fields[0]); i++) {
        const loc_field_t *field = &loc_object_fields[i];
        if (memcmp((const unsigned char *)a + field->offset,
                   (const unsigned char *)b + field->offset,
                   field->size) != 0) {
            return field->name;
        }
    }
    return NULL;
}

static bool loc_compare(const loc_edit_t *edit,
                        const loc_snapshot_t *base,
                        const loc_snapshot_t *edited,
                        const ivp_time_t affected[LOC_OBJECT_COUNT],
                        ivp_real_t delta_psi_time)
{
    int first_psi[LOC_OBJECT_COUNT];
    const char *first_field[LOC_OBJECT_COUNT];
    int diverged = 0;
    bool ok = true;
    for (int id = 0; id < LOC_OBJECT_COUNT; id++) {
        first_psi[id] = -1;
        first_field[id] = NULL;
    }

    for (int psi = 0; psi < LOC_PSI_COUNT; psi++) {
        if (memcmp(&base[psi].time, &edited[psi].time,
                   sizeof(base[psi].time)) != 0) {
            fprintf(stderr, "locality %s: world time differs at psi %d\n",
                    edit->name, psi);
            return false;
        }
        /* one PSI of slack: an interaction inside a step reaches the
         * state captured at its end */
        ivp_time_t horizon = base[psi].time + (ivp_time_t)delta_psi_time;
        for (int id = 0; id < LOC_EDIT_ID; id++) {
            if (first_psi[id] >= 0) continue;
            if (id == LOC_GROUND_ID) continue;
            if (edit->kind == LOC_EDIT_REMOVE && id == edit->target) continue;
            if (!(horizon < affected[id])) continue;
            const char *field = loc_first_different_field(
                &base[psi].objects[id], &edited[psi].objects[id]);
            if (!field) continue;
            first_psi[id] = psi;
            first_field[id] = field;
            diverged++;
        }
    }

    int closure = 0;
    for (int id = 0; id < LOC_EDIT_ID; id++) {
        if (affected[id] != INFINITY) closure++;
    }
    fprintf(stderr, "locality %s: closure=%d diverged-outside=%d\n",
            edit->name, closure, diverged);
    if (edit->expect_spread && closure == 0) {
        fprintf(stderr, "locality %s: the edit reached no object; the "
                        "interaction recorder missed it\n", edit->name);
        ok = false;
    }
    if (diverged == 0) return ok;

    int reported = 0;
    for (int psi = 0; psi < LOC_PSI_COUNT && reported < LOC_REPORT_LIMIT;
         psi++) {
        for (int id = 0; id < LOC_EDIT_ID && reported < LOC_REPORT_LIMIT;
             id++) {
            if (first_psi[id] != psi) continue;
            const loc_object_state_t *a = &base[psi].objects[id];
            const loc_object_state_t *b = &edited[psi].objects[id];
            fprintf(stderr,
                    "  object %d (island %d local %d) psi %d t=%a field=%s "
                    "affected_at=%a movement=%d/%d frozen=%d/%d\n",
                    id, (id - 1) / LOC_OBJECTS_PER_ISLAND,
                    (id - 1) % LOC_OBJECTS_PER_ISLAND, psi,
                    base[psi].time, first_field[id], affected[id],
                    a->movement_state, b->movement_state,
                    a->object_movement_state, b->object_movement_state);
            reported++;
        }
    }
    return false;
}

/* ── Driver ──────────────────────────────────────────────────────────── */

/* Track a removal target under LOC_EDIT_ID, where the closure starts. */
static void loc_alias_to_edit_id(loc_world_t *world, int target)
{
    world->objects[LOC_EDIT_ID] = world->objects[target];
    world->unmoveable[LOC_EDIT_ID] = world->unmoveable[target];
    world->objects[target] = NULL;
    for (size_t i = 0; i < world->recorder.count; i++) {
        loc_interaction_t *item = &world->recorder.items[i];
        if (item->a == target) item->a = LOC_EDIT_ID;
        if (item->b == target) item->b = LOC_EDIT_ID;
    }
}

static bool loc_run_edit(const loc_edit_t *edit)
{
    loc_assets_t assets;
    loc_world_t base;
    loc_world_t edited;
    memset(&base, 0, sizeof(base));
    memset(&edited, 0, sizeof(edited));
    loc_snapshot_t *base_snapshots = (loc_snapshot_t *)calloc(
        LOC_PSI_COUNT, sizeof(*base_snapshots));
    loc_snapshot_t *edited_snapshots = (loc_snapshot_t *)calloc(
        LOC_PSI_COUNT, sizeof(*edited_snapshots));
    bool ok = base_snapshots && edited_snapshots && loc_assets_init(&assets);
    if (!ok) {
        fprintf(stderr, "locality %s: setup allocation failed\n", edit->name);
        goto cleanup;
    }

    if (!loc_world_init(&base, &assets, NULL) ||
        !loc_world_init(&edited, &assets, edit)) {
        fprintf(stderr, "locality %s: world setup failed\n", edit->name);
        ok = false;
        goto cleanup;
    }
    if (edit->kind == LOC_EDIT_REMOVE) {
        loc_alias_to_edit_id(&base, edit->target);
        loc_alias_to_edit_id(&edited, edit->target);
    }

    ivp_time_t edit_time = 0.0;
    for (int psi = 0; psi < LOC_PSI_COUNT; psi++) {
        if (edit->psi > 0 && psi == edit->psi) {
            edit_time = ivp_environment_get_time(edited.environment);
            loc_edit_t late = *edit;
            late.target = LOC_EDIT_ID;
            if (!loc_apply_late_edit(&edited, &assets, &late)) {
                fprintf(stderr, "locality %s: edit failed\n", edit->name);
                ok = false;
                goto cleanup;
            }
        }
        ivp_environment_simulate_one_psi(base.environment);
        ivp_environment_simulate_one_psi(edited.environment);
        loc_capture(&base, &base_snapshots[psi]);
        loc_capture(&edited, &edited_snapshots[psi]);
    }
    if (base.recorder.failed || edited.recorder.failed) {
        fprintf(stderr, "locality %s: interaction recording failed\n",
                edit->name);
        ok = false;
        goto cleanup;
    }

    ivp_time_t affected[LOC_OBJECT_COUNT];
    loc_closure(&base, &edited, edit_time, affected);
    ok =loc_compare(edit, base_snapshots, edited_snapshots, affected,
                     ivp_environment_get_delta_psi_time(base.environment));

cleanup:
    loc_world_destroy(&edited);
    loc_world_destroy(&base);
    loc_assets_destroy(&assets);
    free(edited_snapshots);
    free(base_snapshots);
    return ok;
}

static loc_spec_t loc_far_cube(void)
{
    loc_spec_t spec;
    memset(&spec, 0, sizeof(spec));
    spec.shape = LOC_SHAPE_CUBE;
    spec.position[0] = 0.0;
    spec.position[1] = -0.5;
    spec.position[2] = 52.0;
    spec.speed[2] = 0.5;
    return spec;
}

static loc_spec_t loc_hitting_ball(void)
{
    loc_spec_t spec;
    memset(&spec, 0, sizeof(spec));
    spec.shape = LOC_SHAPE_BALL;
    spec.position[0] = loc_island_x(1) - 6.0;
    spec.position[1] = -1.0;
    spec.position[2] = loc_island_z(1);
    spec.speed[0] = 4.0;
    return spec;
}

static loc_spec_t loc_near_pillar(void)
{
    /* the island's ground cubes reach x + 1.05; leave a 0.45 gap */
    loc_spec_t spec;
    memset(&spec, 0, sizeof(spec));
    spec.shape = LOC_SHAPE_PILLAR;
    spec.position[0] = loc_island_x(2) - 2.0;
    spec.position[1] = 1.0;
    spec.position[2] = loc_island_z(2);
    return spec;
}

static loc_spec_t loc_pool_cube(void)
{
    /* in the pool island's liquid, clear of its pile and rolling ball */
    loc_spec_t spec;
    memset(&spec, 0, sizeof(spec));
    spec.shape = LOC_SHAPE_CUBE;
    spec.position[0] = loc_island_x(LOC_POOL_ISLAND) - 6.0;
    spec.position[1] = 0.9;
    spec.position[2] = loc_island_z(LOC_POOL_ISLAND);
    return spec;
}

/* Edits far from every island: the closure must stay empty. */
IVP_TEST(far_object_edits_leave_other_objects_unchanged)
{
    const loc_edit_t edits[] = {
        { "add-far-appended", LOC_EDIT_ADD, 0, loc_far_cube(),
          LOC_BASE_COUNT - 1, -1, false },
        { "add-far-inserted", LOC_EDIT_ADD, 0, loc_far_cube(),
          loc_island_id(2, LOC_OBJECTS_PER_ISLAND - 1), -1, false },
        { "add-far-late", LOC_EDIT_ADD, 120, loc_far_cube(),
          -1, -1, false },
    };
    bool ok = true;
    for (size_t i = 0; i < sizeof(edits) / sizeof(edits[0]); i++) {
        ok = loc_run_edit(&edits[i]) && ok;
    }
    IVP_ASSERT_TRUE(ok);
}

/* Edits that interact with one island must leave the other islands alone;
 * these also check that the recorder sees the interactions. */
IVP_TEST(interacting_edits_stay_inside_their_closure)
{
    loc_spec_t none;
    memset(&none, 0, sizeof(none));
    const loc_edit_t edits[] = {
        { "add-hitting-ball", LOC_EDIT_ADD, 0, loc_hitting_ball(),
          LOC_BASE_COUNT - 1, -1, true },
        { "remove-top-cube", LOC_EDIT_REMOVE, 0, none,
          -1, loc_island_id(3, 4), true },
        { "remove-ball-late", LOC_EDIT_REMOVE, 200, none,
          -1, loc_island_id(4, 5), false },
        { "remove-resting-cube-late", LOC_EDIT_REMOVE, 300, none,
          -1, loc_island_id(5, 0), true },
        { "remove-spring-end-late", LOC_EDIT_REMOVE, 250, none,
          -1, loc_island_id(LOC_SPRING_ISLAND, 1), true },
        { "remove-constraint-end-late", LOC_EDIT_REMOVE, 250, none,
          -1, loc_island_id(LOC_CONSTRAINT_ISLAND, 3), true },
    };
    bool ok = true;
    for (size_t i = 0; i < sizeof(edits) / sizeof(edits[0]); i++) {
        ok = loc_run_edit(&edits[i]) && ok;
    }
    IVP_ASSERT_TRUE(ok);
}

/* Members of one independent buoyancy controller do not interact. */
IVP_TEST(shared_pool_members_stay_independent)
{
    loc_spec_t none;
    memset(&none, 0, sizeof(none));
    const loc_edit_t edits[] = {
        { "add-into-pool", LOC_EDIT_ADD, 0, loc_pool_cube(),
          LOC_BASE_COUNT - 1, -1, false, true },
        /* created before most pool members, so its per-core data may take
         * an earlier slot */
        { "add-into-pool-inserted", LOC_EDIT_ADD, 0, loc_pool_cube(),
          loc_island_id(2, LOC_OBJECTS_PER_ISLAND - 1), -1, false, true },
        { "remove-pool-member", LOC_EDIT_REMOVE, 0, none,
          -1, loc_island_id(LOC_POOL_ISLAND, 5), false },
    };
    bool ok = true;
    for (size_t i = 0; i < sizeof(edits) / sizeof(edits[0]); i++) {
        ok = loc_run_edit(&edits[i]) && ok;
    }
    IVP_ASSERT_TRUE(ok);
}

/* An unmoveable object close to a pile but never touching it. */
IVP_TEST(nearby_static_object_does_not_change_neighbours)
{
    const loc_edit_t edit = {
        "add-near-pillar", LOC_EDIT_ADD, 0, loc_near_pillar(),
        LOC_BASE_COUNT - 1, -1, false
    };
    IVP_ASSERT_TRUE(loc_run_edit(&edit));
}

int main(void)
{
    IVP_REGISTER(far_object_edits_leave_other_objects_unchanged);
    IVP_REGISTER(interacting_edits_stay_inside_their_closure);
    IVP_REGISTER(shared_pool_members_stay_independent);
    IVP_REGISTER(nearby_static_object_does_not_change_neighbours);
    IVP_RUN_ALL();
}
