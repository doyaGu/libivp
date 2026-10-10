/* test_replay_equivalence.c -- replay semantic operation logs through every
 * supported execution mode and require bit-exact observable state. */
#include "ivp_test.h"

#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_world_scheduler.h>

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define REPLAY_OBJECT_COUNT 224
#define REPLAY_OPERATION_COUNT 56
#define REPLAY_EVENT_CAPACITY 2048
#define REPLAY_CONTROLLER_SLOT 0

static const uint32_t replay_seeds[] = {
    UINT32_C(0x7a5b39d1),
    UINT32_C(0x243f6a88),
    UINT32_C(0x85a308d3),
    UINT32_C(0x13198a2e),
    UINT32_C(0x03707344),
    UINT32_C(0xa4093822),
    UINT32_C(0x299f31d0),
    UINT32_C(0x082efa98)
};

typedef enum replay_mode {
    REPLAY_MODE_DIRECT = 0,
    REPLAY_MODE_SERIAL_SCHEDULER,
    REPLAY_MODE_PARALLEL_SCHEDULER_A,
    REPLAY_MODE_PARALLEL_SCHEDULER_B,
    REPLAY_MODE_INTERNAL_WORKERS,
    REPLAY_MODE_COUNT
} replay_mode_t;

typedef struct replay_mode_config {
    const char *name;
    int worker_count;
    int history_count;
} replay_mode_config_t;

static const replay_mode_config_t replay_modes[] = {
    { "direct", 1, 0 },
    { "serial-scheduler", 1, 5 },
    { "parallel-scheduler-a", 1, 11 },
    { "parallel-scheduler-b", 1, 19 },
    { "internal-workers", 4, 27 }
};

_Static_assert(sizeof(replay_modes) / sizeof(replay_modes[0]) ==
                   REPLAY_MODE_COUNT,
               "each replay mode needs one configuration");

typedef enum replay_operation_kind {
    REPLAY_ADD_SPEED = 0,
    REPLAY_ADD_ROT_SPEED,
    REPLAY_SET_LAYER,
    REPLAY_SET_GROUP,
    REPLAY_SET_SIMULATION,
    REPLAY_BEAM,
    REPLAY_RECREATE,
    REPLAY_SET_PINNED,
    REPLAY_SET_MASS,
    REPLAY_TOGGLE_CONTROLLER,
    REPLAY_OPERATION_KIND_COUNT
} replay_operation_kind_t;

typedef struct replay_operation {
    replay_operation_kind_t kind;
    int slot;
    uint32_t a;
    uint32_t b;
    uint32_t c;
} replay_operation_t;

typedef struct replay_event_entry {
    int type;
    ivp_time_t time;
    ivp_real_t dt;
    int sim_unit_index;
    int object_index;
} replay_event_entry_t;

typedef struct replay_event_trace {
    replay_event_entry_t entries[REPLAY_EVENT_CAPACITY];
    size_t count;
    bool overflow;
} replay_event_trace_t;

typedef struct replay_controller_context {
    uint64_t calls;
} replay_controller_context_t;

typedef struct replay_world {
    ivp_environment_t *environment;
    ivp_object_t *objects[REPLAY_OBJECT_COUNT];
    uint32_t generations[REPLAY_OBJECT_COUNT];
    replay_event_trace_t events;
    replay_controller_context_t controller_context;
    ivp_controller_handle_t controller;
    ivp_event_listener_handle_t event_listener;
} replay_world_t;

typedef struct replay_object_state {
    bool present;
    bool pinned;
    bool collision_enabled;
    int movement_state;
    int collision_group;
    uint32_t layer_mask;
    uint32_t generation;
    ivp_real_t mass;
    ivp_time_t time_of_last_psi;
    ivp_real_t position[3];
    ivp_real_t speed[3];
    ivp_real_t speed_change[3];
    ivp_real_t rot_speed_change[3];
    float rot_speed[3];
    ivp_real_t orientation[4];
    ivp_real_t next_orientation[4];
    ivp_real_t energy;
} replay_object_state_t;

typedef struct replay_snapshot {
    ivp_time_t time;
    uint64_t controller_calls;
    bool controller_active;
    replay_object_state_t objects[REPLAY_OBJECT_COUNT];
} replay_snapshot_t;

typedef struct replay_field {
    const char *name;
    size_t offset;
    size_t size;
} replay_field_t;

#define REPLAY_FIELD(type, member)                                         \
    { #member, offsetof(type, member), sizeof(((type *)0)->member) }

static const replay_field_t replay_snapshot_fields[] = {
    REPLAY_FIELD(replay_snapshot_t, time),
    REPLAY_FIELD(replay_snapshot_t, controller_calls),
    REPLAY_FIELD(replay_snapshot_t, controller_active)
};

static const replay_field_t replay_object_fields[] = {
    REPLAY_FIELD(replay_object_state_t, present),
    REPLAY_FIELD(replay_object_state_t, pinned),
    REPLAY_FIELD(replay_object_state_t, collision_enabled),
    REPLAY_FIELD(replay_object_state_t, movement_state),
    REPLAY_FIELD(replay_object_state_t, collision_group),
    REPLAY_FIELD(replay_object_state_t, layer_mask),
    REPLAY_FIELD(replay_object_state_t, generation),
    REPLAY_FIELD(replay_object_state_t, mass),
    REPLAY_FIELD(replay_object_state_t, time_of_last_psi),
    REPLAY_FIELD(replay_object_state_t, position),
    REPLAY_FIELD(replay_object_state_t, speed),
    REPLAY_FIELD(replay_object_state_t, speed_change),
    REPLAY_FIELD(replay_object_state_t, rot_speed_change),
    REPLAY_FIELD(replay_object_state_t, rot_speed),
    REPLAY_FIELD(replay_object_state_t, orientation),
    REPLAY_FIELD(replay_object_state_t, next_orientation),
    REPLAY_FIELD(replay_object_state_t, energy)
};

static const replay_field_t replay_event_fields[] = {
    REPLAY_FIELD(replay_event_entry_t, type),
    REPLAY_FIELD(replay_event_entry_t, time),
    REPLAY_FIELD(replay_event_entry_t, dt),
    REPLAY_FIELD(replay_event_entry_t, sim_unit_index),
    REPLAY_FIELD(replay_event_entry_t, object_index)
};

#undef REPLAY_FIELD

static uint32_t replay_next(uint32_t *state)
{
    uint32_t value = *state;
    value ^= value << 13;
    value ^= value >> 17;
    value ^= value << 5;
    *state = value;
    return value;
}

static ivp_real_t replay_signed_value(uint32_t value, ivp_real_t scale)
{
    int centered = (int)(value % UINT32_C(17)) - 8;
    return (ivp_real_t)centered * scale;
}

static replay_operation_t replay_make_operation(int operation_index,
                                                uint32_t *random_state)
{
    replay_operation_t operation;
    operation.kind = (replay_operation_kind_t)(
        operation_index % (int)REPLAY_OPERATION_KIND_COUNT);
    operation.slot = REPLAY_CONTROLLER_SLOT + 1 +
        (int)(replay_next(random_state) %
              (uint32_t)(REPLAY_OBJECT_COUNT - REPLAY_CONTROLLER_SLOT - 1));
    operation.a = replay_next(random_state);
    operation.b = replay_next(random_state);
    operation.c = replay_next(random_state);
    return operation;
}

static void replay_reset_events(replay_world_t *world)
{
    /* Start each operation with an independent public-event trace. */
    memset(&world->events, 0, sizeof(world->events));
}

static void replay_record_event(void *context, ivp_environment_t *environment,
                                const ivp_event_t *event)
{
    (void)environment;
    replay_event_trace_t *trace = (replay_event_trace_t *)context;
    if (trace->count >= REPLAY_EVENT_CAPACITY) {
        trace->overflow = true;
        return;
    }
    replay_event_entry_t *entry = &trace->entries[trace->count++];
    entry->type = (int)event->type;
    entry->time = event->time;
    entry->dt = event->dt;
    entry->sim_unit_index = event->sim_unit_index;
    entry->object_index = event->object_index;
}

static void replay_discard_event(void *context,
                                 ivp_environment_t *environment,
                                 const ivp_event_t *event)
{
    (void)context;
    (void)environment;
    (void)event;
}

static void replay_count_controller(const ivp_event_sim_t *event_sim,
                                    ivp_object_t *object, void *context)
{
    (void)event_sim;
    (void)object;
    replay_controller_context_t *controller =
        (replay_controller_context_t *)context;
    controller->calls++;
}

static void replay_noop_controller(const ivp_event_sim_t *event_sim,
                                   ivp_object_t *object, void *context)
{
    (void)event_sim;
    (void)object;
    (void)context;
}

static void replay_fill_template(ivp_object_template_t *object_template,
                                 int slot, uint32_t generation)
{
    ivp_object_template_init(object_template);
    object_template->type = IVP_OBJECT_BALL;
    object_template->ball_radius = IVP_REAL(0.45);
    object_template->mass = IVP_REAL(1.0) +
        (ivp_real_t)((slot + (int)(generation % UINT32_C(3))) % 4) *
            IVP_REAL(0.25);
    object_template->speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&object_template->rot_speed_damp_factor,
                  IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    object_template->material.friction = IVP_REAL(0.6);
    object_template->material.elasticity = IVP_REAL(0.5);

    if (slot < 32) {
        int pair = slot / 2;
        ivp_vec3r_set(&object_template->position,
                      (slot & 1) ? IVP_REAL(0.65) : IVP_REAL(-0.65),
                      IVP_REAL(0.0),
                      (ivp_real_t)pair * IVP_REAL(3.0));
    } else {
        int local_slot = slot - 32;
        ivp_vec3r_set(
            &object_template->position,
            (ivp_real_t)(local_slot % 16) * IVP_REAL(4.0) +
                (ivp_real_t)generation * IVP_REAL(0.03125),
            (ivp_real_t)(local_slot / 16) * IVP_REAL(4.0),
            IVP_REAL(40.0) + (ivp_real_t)(slot % 7) * IVP_REAL(4.0));
    }
}

static ivp_object_t *replay_create_object(replay_world_t *world, int slot)
{
    ivp_object_template_t object_template;
    replay_fill_template(&object_template, slot, world->generations[slot]);
    ivp_object_t *object = ivp_environment_create_object(
        world->environment, &object_template, NULL);
    if (!object) return NULL;

    ivp_vec3r_t speed = ivp_vec3r_zero();
    if (slot < 32) {
        speed.k[0] = (slot & 1) ? IVP_REAL(-1.0) : IVP_REAL(1.0);
    } else {
        speed.k[0] = (ivp_real_t)((slot % 5) - 2) * IVP_REAL(0.03125);
        speed.k[2] = (ivp_real_t)((slot % 3) - 1) * IVP_REAL(0.015625);
    }
    ivp_object_async_add_speed_ws(object, &speed);
    return object;
}

static bool replay_prewarm(replay_world_t *world, int history_count)
{
    enum { REPLAY_PREWARM_MAX = 31, REPLAY_PREWARM_HANDLES = 7 };
    ivp_object_t *objects[REPLAY_PREWARM_MAX] = {0};
    ivp_event_listener_handle_t listeners[REPLAY_PREWARM_HANDLES];
    ivp_controller_handle_t controllers[REPLAY_PREWARM_HANDLES];

    int listener_count = history_count % REPLAY_PREWARM_HANDLES;
    for (int i = 0; i < listener_count; i++) {
        listeners[i] = ivp_environment_add_event_listener(
            world->environment, replay_discard_event, NULL);
        if (!ivp_event_listener_handle_is_valid(listeners[i])) return false;
    }
    for (int i = 0; i < listener_count; i++) {
        if (!ivp_environment_remove_event_listener(
                world->environment, listeners[i])) {
            return false;
        }
    }

    int controller_count = (history_count * 3) % REPLAY_PREWARM_HANDLES;
    for (int i = 0; i < controller_count; i++) {
        controllers[i] = ivp_environment_create_core_controller(
            world->environment, IVP_CP_MOTION,
            replay_noop_controller, NULL);
        if (!ivp_controller_handle_is_valid(controllers[i])) return false;
    }
    for (int i = controller_count; i > 0; i--) {
        if (!ivp_environment_remove_controller(
                world->environment, controllers[i - 1])) {
            return false;
        }
    }

    int object_count = history_count % REPLAY_PREWARM_MAX;
    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.2);
    object_template.start_in_simulation = false;
    for (int i = 0; i < object_count; i++) {
        ivp_vec3r_set(&object_template.position,
                      (ivp_real_t)i * IVP_REAL(2.0),
                      IVP_REAL(-100.0), IVP_REAL(0.0));
        objects[i] = ivp_environment_create_object(
            world->environment, &object_template, NULL);
        if (!objects[i]) return false;
    }
    for (int parity = 0; parity < 2; parity++) {
        for (int i = object_count - 1; i >= 0; i--) {
            if ((i & 1) == parity) {
                ivp_environment_destroy_object(world->environment, objects[i]);
            }
        }
    }
    return true;
}

static bool replay_world_init(replay_world_t *world, int worker_count,
                              int history_count)
{
    memset(world, 0, sizeof(*world));
    world->controller = ivp_controller_handle_invalid();
    world->event_listener = ivp_event_listener_handle_invalid();

    ivp_environment_config_t config;
    ivp_environment_config_init(&config);
    ivp_vec3r_set(&config.gravity,
                  IVP_REAL(0.0), IVP_REAL(0.35), IVP_REAL(0.0));
    config.disable_performance_counter = true;
    config.parallel_worker_count = worker_count;
    config.initial_object_capacity = history_count == 0 ? 256 : 16;
    config.initial_controller_capacity = history_count == 0 ? 16 : 2;
    config.initial_event_listener_capacity = history_count == 0 ? 8 : 2;
    world->environment = ivp_environment_create(&config);
    if (!world->environment || !replay_prewarm(world, history_count)) {
        return false;
    }

    world->event_listener = ivp_environment_add_event_listener(
        world->environment, replay_record_event, &world->events);
    if (!ivp_event_listener_handle_is_valid(world->event_listener)) {
        return false;
    }
    for (int slot = 0; slot < REPLAY_OBJECT_COUNT; slot++) {
        world->objects[slot] = replay_create_object(world, slot);
        if (!world->objects[slot]) return false;
    }
    return true;
}

static void replay_world_destroy(replay_world_t *world)
{
    if (world->environment) {
        ivp_environment_destroy(world->environment);
    }
    memset(world, 0, sizeof(*world));
}

static bool replay_toggle_controller(replay_world_t *world)
{
    if (ivp_controller_handle_is_valid(world->controller)) {
        if (!ivp_environment_remove_controller(
                world->environment, world->controller)) {
            return false;
        }
        world->controller = ivp_controller_handle_invalid();
        return true;
    }

    world->controller = ivp_environment_create_core_controller(
        world->environment, IVP_CP_MOTION,
        replay_count_controller, &world->controller_context);
    if (!ivp_controller_handle_is_valid(world->controller)) return false;
    if (!ivp_environment_add_controller_to_core(
            world->environment, world->controller,
            world->objects[REPLAY_CONTROLLER_SLOT])) {
        (void)ivp_environment_remove_controller(
            world->environment, world->controller);
        world->controller = ivp_controller_handle_invalid();
        return false;
    }
    return true;
}

static bool replay_apply_operation(replay_world_t *world,
                                   const replay_operation_t *operation)
{
    ivp_object_t *object = world->objects[operation->slot];
    if (!object) return false;

    switch (operation->kind) {
        case REPLAY_ADD_SPEED: {
            ivp_vec3r_t speed;
            ivp_vec3r_set(&speed,
                          replay_signed_value(operation->a, IVP_REAL(0.03125)),
                          replay_signed_value(operation->b, IVP_REAL(0.015625)),
                          replay_signed_value(operation->c, IVP_REAL(0.03125)));
            ivp_object_async_add_speed_ws(object, &speed);
            return true;
        }
        case REPLAY_ADD_ROT_SPEED: {
            ivp_vec3r_t speed;
            ivp_vec3r_set(&speed,
                          replay_signed_value(operation->a, IVP_REAL(0.0078125)),
                          replay_signed_value(operation->b, IVP_REAL(0.0078125)),
                          replay_signed_value(operation->c, IVP_REAL(0.0078125)));
            ivp_object_async_add_rot_speed_cs(object, &speed);
            return true;
        }
        case REPLAY_SET_LAYER: {
            static const uint32_t masks[] = {
                UINT32_C(0), UINT32_C(1), UINT32_C(3), UINT32_C(0xffffffff)
            };
            ivp_object_set_layer_mask(
                object, masks[operation->a %
                              (uint32_t)(sizeof(masks) / sizeof(masks[0]))]);
            return true;
        }
        case REPLAY_SET_GROUP:
            ivp_object_set_collision_group(
                object, (int)(operation->a % UINT32_C(5)));
            return true;
        case REPLAY_SET_SIMULATION:
            if ((operation->a & UINT32_C(1)) != 0u) {
                (void)ivp_object_disable_simulation(object);
            } else {
                ivp_object_ensure_in_simulation_now(object);
            }
            return true;
        case REPLAY_BEAM: {
            ivp_quatr_t orientation = ivp_quatr_identity();
            ivp_vec3r_t position;
            ivp_vec3r_set(
                &position,
                (ivp_real_t)(operation->slot % 16) * IVP_REAL(2.5) +
                    replay_signed_value(operation->a, IVP_REAL(0.0625)),
                (ivp_real_t)(operation->slot / 16) * IVP_REAL(1.5) +
                    replay_signed_value(operation->b, IVP_REAL(0.0625)),
                IVP_REAL(20.0) +
                    replay_signed_value(operation->c, IVP_REAL(0.125)));
            ivp_object_beam_to_new_position(
                object, &orientation, &position,
                (operation->a & UINT32_C(2)) != 0u);
            return true;
        }
        case REPLAY_RECREATE:
            ivp_environment_destroy_object(world->environment, object);
            world->objects[operation->slot] = NULL;
            world->generations[operation->slot]++;
            world->objects[operation->slot] = replay_create_object(
                world, operation->slot);
            return world->objects[operation->slot] != NULL;
        case REPLAY_SET_PINNED:
            ivp_object_set_pinned(
                object, (operation->a & UINT32_C(1)) != 0u);
            return true;
        case REPLAY_SET_MASS:
            ivp_object_change_mass(
                object, IVP_REAL(0.5) +
                    (ivp_real_t)(operation->a % UINT32_C(8)) *
                        IVP_REAL(0.25));
            return true;
        case REPLAY_TOGGLE_CONTROLLER:
            return replay_toggle_controller(world);
        case REPLAY_OPERATION_KIND_COUNT:
            break;
    }
    return false;
}

static void replay_capture_snapshot(const replay_world_t *world,
                                    replay_snapshot_t *snapshot)
{
    /* Define every field, including fields belonging to absent objects. */
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->time = ivp_environment_get_time(world->environment);
    snapshot->controller_calls = world->controller_context.calls;
    snapshot->controller_active =
        ivp_controller_handle_is_valid(world->controller);

    for (int slot = 0; slot < REPLAY_OBJECT_COUNT; slot++) {
        const ivp_object_t *object = world->objects[slot];
        replay_object_state_t *state = &snapshot->objects[slot];
        state->generation = world->generations[slot];
        if (!object) continue;

        const ivp_core_t *core = ivp_object_get_core_const(object);
        ivp_vec3r_t position;
        ivp_vec3r_t speed;
        ivp_vec3f_t rot_speed;
        ivp_quatr_t orientation;
        ivp_object_get_position(object, &position);
        ivp_object_get_speed(object, &speed);
        ivp_object_get_rot_speed(object, &rot_speed);
        ivp_object_get_orientation(object, &orientation);

        state->present = true;
        state->pinned = core->pinned;
        state->collision_enabled = object->collision_detection_enabled;
        state->movement_state = (int)core->movement_state;
        state->collision_group = object->collision_group;
        state->layer_mask = object->layer_mask;
        state->mass = core->mass;
        state->time_of_last_psi = core->time_of_last_psi;
        for (int axis = 0; axis < 3; axis++) {
            state->position[axis] = position.k[axis];
            state->speed[axis] = speed.k[axis];
            state->speed_change[axis] = core->speed_change.k[axis];
            state->rot_speed_change[axis] = core->rot_speed_change.k[axis];
            state->rot_speed[axis] = rot_speed.k[axis];
        }
        state->orientation[0] = orientation.x;
        state->orientation[1] = orientation.y;
        state->orientation[2] = orientation.z;
        state->orientation[3] = orientation.w;
        state->next_orientation[0] = core->q_world_f_core_next_psi.x;
        state->next_orientation[1] = core->q_world_f_core_next_psi.y;
        state->next_orientation[2] = core->q_world_f_core_next_psi.z;
        state->next_orientation[3] = core->q_world_f_core_next_psi.w;
        state->energy = ivp_object_get_energy(object);
    }
}

static const char *replay_first_different_field(
    const void *expected, const void *actual,
    const replay_field_t *fields, size_t field_count)
{
    const unsigned char *expected_bytes = (const unsigned char *)expected;
    const unsigned char *actual_bytes = (const unsigned char *)actual;
    for (size_t index = 0; index < field_count; index++) {
        const replay_field_t *field = &fields[index];
        if (memcmp(expected_bytes + field->offset,
                   actual_bytes + field->offset, field->size) != 0) {
            return field->name;
        }
    }
    return NULL;
}

static bool replay_snapshots_equal(const replay_snapshot_t *expected,
                                   const replay_snapshot_t *actual,
                                   int *different_slot,
                                   const char **different_field)
{
    *different_slot = -1;
    *different_field = replay_first_different_field(
        expected, actual, replay_snapshot_fields,
        sizeof(replay_snapshot_fields) / sizeof(replay_snapshot_fields[0]));
    if (*different_field) return false;

    for (int slot = 0; slot < REPLAY_OBJECT_COUNT; slot++) {
        const replay_object_state_t *expected_state =
            &expected->objects[slot];
        const replay_object_state_t *actual_state = &actual->objects[slot];
        *different_field = replay_first_different_field(
            expected_state, actual_state, replay_object_fields,
            sizeof(replay_object_fields) / sizeof(replay_object_fields[0]));
        if (*different_field) {
            *different_slot = slot;
            return false;
        }
    }
    return true;
}

static bool replay_events_equal(const replay_event_trace_t *expected,
                                const replay_event_trace_t *actual,
                                int *different_event,
                                const char **different_field)
{
    *different_event = -1;
    *different_field = NULL;
    if (expected->overflow != actual->overflow) {
        *different_field = "overflow";
        return false;
    }
    if (expected->count != actual->count) {
        *different_field = "count";
        return false;
    }
    for (size_t index = 0; index < expected->count; index++) {
        *different_field = replay_first_different_field(
            &expected->entries[index], &actual->entries[index],
            replay_event_fields,
            sizeof(replay_event_fields) / sizeof(replay_event_fields[0]));
        if (*different_field) {
            *different_event = (int)index;
            return false;
        }
    }
    return true;
}

static bool replay_compare_worlds(replay_world_t worlds[REPLAY_MODE_COUNT],
                                  uint32_t seed,
                                  int operation_index,
                                  const replay_operation_t *operation)
{
    replay_snapshot_t expected;
    replay_snapshot_t actual;
    replay_capture_snapshot(&worlds[REPLAY_MODE_DIRECT], &expected);

    if (worlds[REPLAY_MODE_DIRECT].events.overflow) {
        fprintf(stderr,
                "replay event overflow: seed=0x%08" PRIx32 " op=%d\n",
                seed, operation_index);
        return false;
    }
    for (replay_mode_t mode = REPLAY_MODE_SERIAL_SCHEDULER;
         mode < REPLAY_MODE_COUNT; mode++) {
        int different_slot = -1;
        int different_event = -1;
        const char *state_field = NULL;
        const char *event_field = NULL;
        replay_capture_snapshot(&worlds[mode], &actual);
        bool states_equal = replay_snapshots_equal(
            &expected, &actual, &different_slot, &state_field);
        bool events_equal = replay_events_equal(
            &worlds[REPLAY_MODE_DIRECT].events, &worlds[mode].events,
            &different_event, &event_field);
        if (!states_equal || !events_equal) {
            fprintf(stderr,
                    "replay mismatch: seed=0x%08" PRIx32
                    " op=%d kind=%d slot=%d mode=%s state_slot=%d"
                    " state_field=%s event_index=%d event_field=%s"
                    " events=%zu/%zu overflow=%d\n",
                    seed, operation_index,
                    operation ? (int)operation->kind : -1,
                    operation ? operation->slot : -1,
                    replay_modes[mode].name, different_slot,
                    state_field ? state_field : "none", different_event,
                    event_field ? event_field : "none",
                    worlds[REPLAY_MODE_DIRECT].events.count,
                    worlds[mode].events.count,
                    worlds[mode].events.overflow ? 1 : 0);
            return false;
        }
    }
    return true;
}

static bool replay_step_worlds(replay_world_t worlds[REPLAY_MODE_COUNT],
                               ivp_world_scheduler_t *serial_scheduler,
                               ivp_world_scheduler_t *parallel_scheduler,
                               bool use_elapsed_time)
{
    ivp_real_t elapsed = ivp_environment_get_delta_psi_time(
        worlds[REPLAY_MODE_DIRECT].environment) * IVP_REAL(2.0);
    if (use_elapsed_time) {
        ivp_environment_simulate_dtime(
            worlds[REPLAY_MODE_DIRECT].environment, elapsed);
    } else {
        ivp_environment_simulate_one_psi(
            worlds[REPLAY_MODE_DIRECT].environment);
    }

    ivp_environment_t *serial_worlds[] = {
        worlds[REPLAY_MODE_SERIAL_SCHEDULER].environment
    };
    ivp_result_t result = use_elapsed_time
        ? ivp_world_scheduler_simulate_dtime(
              serial_scheduler, serial_worlds, 1, elapsed)
        : ivp_world_scheduler_simulate_one_psi(
              serial_scheduler, serial_worlds, 1);
    if (result != IVP_OK) return false;

    ivp_environment_t *parallel_worlds[] = {
        worlds[REPLAY_MODE_PARALLEL_SCHEDULER_A].environment,
        worlds[REPLAY_MODE_PARALLEL_SCHEDULER_B].environment
    };
#if defined(IVP_HOME_BUILD) || defined(IVP_BENCHMARK_DIAGNOSTICS)
    for (size_t i = 0; i < 2; i++) {
        result = use_elapsed_time
            ? ivp_world_scheduler_simulate_dtime(
                  parallel_scheduler, &parallel_worlds[i], 1, elapsed)
            : ivp_world_scheduler_simulate_one_psi(
                  parallel_scheduler, &parallel_worlds[i], 1);
        if (result != IVP_OK) return false;
    }
#else
    result = use_elapsed_time
        ? ivp_world_scheduler_simulate_dtime(
              parallel_scheduler, parallel_worlds, 2, elapsed)
        : ivp_world_scheduler_simulate_one_psi(
              parallel_scheduler, parallel_worlds, 2);
    if (result != IVP_OK) return false;
#endif

    if (use_elapsed_time) {
        ivp_environment_simulate_dtime(
            worlds[REPLAY_MODE_INTERNAL_WORKERS].environment, elapsed);
    } else {
        ivp_environment_simulate_one_psi(
            worlds[REPLAY_MODE_INTERNAL_WORKERS].environment);
    }
    return true;
}

static bool replay_run_seed(uint32_t seed)
{
    replay_world_t worlds[REPLAY_MODE_COUNT];
    memset(worlds, 0, sizeof(worlds));
    ivp_world_scheduler_t *serial_scheduler = NULL;
    ivp_world_scheduler_t *parallel_scheduler = NULL;
    bool ok = true;

    for (replay_mode_t mode = REPLAY_MODE_DIRECT;
         mode < REPLAY_MODE_COUNT; mode++) {
        if (!replay_world_init(&worlds[mode], replay_modes[mode].worker_count,
                               replay_modes[mode].history_count)) {
            fprintf(stderr,
                    "replay setup failed: seed=0x%08" PRIx32 " mode=%s\n",
                    seed, replay_modes[mode].name);
            ok = false;
            goto cleanup;
        }
    }
    serial_scheduler = ivp_world_scheduler_create(1, NULL);
    parallel_scheduler = ivp_world_scheduler_create(4, NULL);
    if (!serial_scheduler || !parallel_scheduler) {
        fprintf(stderr,
                "replay scheduler setup failed: seed=0x%08" PRIx32 "\n",
                seed);
        ok = false;
        goto cleanup;
    }
    if (!replay_compare_worlds(worlds, seed, -1, NULL)) {
        ok = false;
        goto cleanup;
    }

    uint32_t random_state = seed;
    for (int operation_index = 0;
         operation_index < REPLAY_OPERATION_COUNT; operation_index++) {
        replay_operation_t operation = replay_make_operation(
            operation_index, &random_state);
        for (replay_mode_t mode = REPLAY_MODE_DIRECT;
             mode < REPLAY_MODE_COUNT; mode++) {
            replay_reset_events(&worlds[mode]);
            if (!replay_apply_operation(&worlds[mode], &operation)) {
                fprintf(stderr,
                        "replay operation failed: seed=0x%08" PRIx32
                        " op=%d kind=%d slot=%d mode=%s\n",
                        seed, operation_index, (int)operation.kind,
                        operation.slot, replay_modes[mode].name);
                ok = false;
                goto cleanup;
            }
        }
        if (!replay_step_worlds(
                worlds, serial_scheduler, parallel_scheduler,
                operation_index % 13 == 0)) {
            fprintf(stderr,
                    "replay dispatch failed: seed=0x%08" PRIx32
                    " op=%d kind=%d slot=%d\n",
                    seed, operation_index, (int)operation.kind,
                    operation.slot);
            ok = false;
            goto cleanup;
        }
        if (!replay_compare_worlds(
                worlds, seed, operation_index, &operation)) {
            ok = false;
            goto cleanup;
        }
    }

#if !defined(IVP_ENABLE_ASSERTS) && \
    !defined(IVP_HOME_BUILD) && !defined(IVP_BENCHMARK_DIAGNOSTICS)
    ivp_parallel_stats_t stats;
    ivp_environment_get_parallel_stats(
        worlds[REPLAY_MODE_INTERNAL_WORKERS].environment, &stats);
    if (stats.island_dispatches == 0u || stats.island_tasks == 0u) {
        fprintf(stderr,
                "replay did not exercise internal island dispatch: "
                "seed=0x%08" PRIx32 "\n",
                seed);
        ok = false;
    }
#endif

cleanup:
    ivp_world_scheduler_destroy(parallel_scheduler);
    ivp_world_scheduler_destroy(serial_scheduler);
    for (replay_mode_t mode = REPLAY_MODE_DIRECT;
         mode < REPLAY_MODE_COUNT; mode++) {
        replay_world_destroy(&worlds[mode]);
    }
    return ok;
}

IVP_TEST(operation_logs_match_execution_modes)
{
    for (size_t index = 0;
         index < sizeof(replay_seeds) / sizeof(replay_seeds[0]); index++) {
        IVP_ASSERT_TRUE(replay_run_seed(replay_seeds[index]));
    }
}

int main(void)
{
    IVP_REGISTER(operation_logs_match_execution_modes);
    IVP_RUN_ALL();
}
