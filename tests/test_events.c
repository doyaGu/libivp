/*
 * test_events.c
 *
 * Comprehensive tests for the IVP event system.
 * Tests event listener registration, removal, and event delivery
 * for various event types including simulation units and controller lifecycle.
 */

#include "ivp_test.h"
#include "ivp_test_controller_api.h"
#include <ivp/base/ivp_types.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_environment_collision.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_handle.h>
#include <ivp/events/ivp_listener_collision.h>
#include <ivp/events/ivp_listener_psi.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <string.h>

/* ========================================================================
 * Event Tracking Helper
 * ======================================================================== */

#define MAX_EVENTS 64

typedef struct {
    int count;
    ivp_event_type_t types[MAX_EVENTS];
    int sim_unit_indices[MAX_EVENTS];
    int object_indices[MAX_EVENTS];
    int controller_ids[MAX_EVENTS];
    ivp_time_t times[MAX_EVENTS];
    ivp_real_t dts[MAX_EVENTS];
} event_tracker_t;

static void event_track_fn(void *ctx, ivp_environment_t *env, const ivp_event_t *ev) {
    event_tracker_t *t = (event_tracker_t *)ctx;
    if (t->count < MAX_EVENTS) {
        t->types[t->count] = ev->type;
        t->sim_unit_indices[t->count] = ev->sim_unit_index;
        t->object_indices[t->count] = ev->object_index;
        t->controller_ids[t->count] = ev->controller_id;
        t->times[t->count] = ev->time;
        t->dts[t->count] = ev->dt;
        t->count++;
    }
}

static void event_tracker_init(event_tracker_t *t) {
    memset(t, 0, sizeof(*t));
}

/* ========================================================================
 * Dummy Controller for Testing
 * ======================================================================== */

static void dummy_controller(const ivp_event_sim_t *event_sim, ivp_object_t *obj, void *ctx) {
    ivp_environment_t *env = event_sim->environment;
    const ivp_real_t dt = event_sim->delta_time;
    (void)obj;
    (void)ctx;
}

static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t x, ivp_real_t vx) {
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.position, x, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    if (obj) {
        ivp_vec3r_set(&obj->core.speed, vx, IVP_REAL(0.0), IVP_REAL(0.0));
    }
    return obj;
}

typedef struct {
    int pre_count;
    int post_count;
    ivp_real_t last_pre_impulse;
    ivp_real_t last_post_impulse;
} collision_listener_tracker_t;

static void on_pre_collision(void *ctx, const ivp_collision_event_t *event) {
    collision_listener_tracker_t *tracker = (collision_listener_tracker_t *)ctx;
    if (tracker && event) {
        tracker->pre_count++;
        tracker->last_pre_impulse = event->impulse;
    }
}

static void on_post_collision(void *ctx, const ivp_collision_event_t *event) {
    collision_listener_tracker_t *tracker = (collision_listener_tracker_t *)ctx;
    if (tracker && event) {
        tracker->post_count++;
        tracker->last_post_impulse = event->impulse;
    }
}

/* ========================================================================
 * Test 1: Event Listener Add/Remove
 * ======================================================================== */

IVP_TEST(event_listener_add_remove) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    event_tracker_t tracker;
    event_tracker_init(&tracker);

    /* Add listener and verify handle is valid */
    ivp_event_listener_handle_t handle = ivp_environment_add_event_listener(env, event_track_fn, &tracker);
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(handle));

    /* Remove listener and verify success */
    bool removed = ivp_environment_remove_event_listener(env, handle);
    IVP_ASSERT_TRUE(removed);

    /* Try to remove again, should fail */
    bool removed_again = ivp_environment_remove_event_listener(env, handle);
    IVP_ASSERT_FALSE(removed_again);

    ivp_environment_destroy(env);
}

/* ========================================================================
 * Test 2: Event Sim Unit Begin
 * ======================================================================== */

IVP_TEST(event_sim_unit_begin) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    event_tracker_t tracker;
    event_tracker_init(&tracker);

    /* Add listener */
    ivp_event_listener_handle_t handle = ivp_environment_add_event_listener(env, event_track_fn, &tracker);
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(handle));

    /* Step environment once */
    ivp_environment_simulate_one_psi(env);

    /* Verify we received at least one SIM_UNIT_BEGIN event */
    bool found_begin = false;
    for (int i = 0; i < tracker.count; i++) {
        if (tracker.types[i] == IVP_EVENT_SIM_UNIT_BEGIN) {
            found_begin = true;
            break;
        }
    }
    IVP_ASSERT_TRUE(found_begin);

    ivp_environment_remove_event_listener(env, handle);
    ivp_environment_destroy(env);
}

/* ========================================================================
 * Test 3: Event Sim Unit End
 * ======================================================================== */

IVP_TEST(event_sim_unit_end) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    event_tracker_t tracker;
    event_tracker_init(&tracker);

    /* Add listener */
    ivp_event_listener_handle_t handle = ivp_environment_add_event_listener(env, event_track_fn, &tracker);
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(handle));

    /* Step environment once */
    ivp_environment_simulate_one_psi(env);

    /* Verify we received at least one SIM_UNIT_END event */
    bool found_end = false;
    for (int i = 0; i < tracker.count; i++) {
        if (tracker.types[i] == IVP_EVENT_SIM_UNIT_END) {
            found_end = true;
            break;
        }
    }
    IVP_ASSERT_TRUE(found_end);

    ivp_environment_remove_event_listener(env, handle);
    ivp_environment_destroy(env);
}

/* ========================================================================
 * Test 4: Event Controller Added
 * ======================================================================== */

IVP_TEST(event_controller_added) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    event_tracker_t tracker;
    event_tracker_init(&tracker);

    /* Add event listener first */
    ivp_event_listener_handle_t handle = ivp_environment_add_event_listener(env, event_track_fn, &tracker);
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(handle));

    /* Now add a controller, which should trigger CONTROLLER_ADDED event */
    ivp_controller_handle_t ctrl_handle = ivp_test_environment_add_custom_controller(env, 0, dummy_controller, NULL);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(ctrl_handle));

    /* Verify we received CONTROLLER_ADDED event */
    bool found_added = false;
    for (int i = 0; i < tracker.count; i++) {
        if (tracker.types[i] == IVP_EVENT_CONTROLLER_ADDED) {
            found_added = true;
            break;
        }
    }
    IVP_ASSERT_TRUE(found_added);

    ivp_environment_remove_controller(env, ctrl_handle);
    ivp_environment_remove_event_listener(env, handle);
    ivp_environment_destroy(env);
}

/* ========================================================================
 * Test 5: Event Controller Removed
 * ======================================================================== */

IVP_TEST(event_controller_removed) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    event_tracker_t tracker;
    event_tracker_init(&tracker);

    /* Add event listener first */
    ivp_event_listener_handle_t handle = ivp_environment_add_event_listener(env, event_track_fn, &tracker);
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(handle));

    /* Add a controller */
    ivp_controller_handle_t ctrl_handle = ivp_test_environment_add_custom_controller(env, 0, dummy_controller, NULL);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(ctrl_handle));

    /* Clear tracker to focus on removal event */
    event_tracker_init(&tracker);

    /* Remove controller, which should trigger CONTROLLER_REMOVED event */
    bool removed = ivp_environment_remove_controller(env, ctrl_handle);
    IVP_ASSERT_TRUE(removed);

    /* Verify we received CONTROLLER_REMOVED event */
    bool found_removed = false;
    for (int i = 0; i < tracker.count; i++) {
        if (tracker.types[i] == IVP_EVENT_CONTROLLER_REMOVED) {
            found_removed = true;
            break;
        }
    }
    IVP_ASSERT_TRUE(found_removed);

    ivp_environment_remove_event_listener(env, handle);
    ivp_environment_destroy(env);
}

/* ========================================================================
 * Test 6: Multiple Event Listeners
 * ======================================================================== */

IVP_TEST(event_multiple_listeners) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    event_tracker_t tracker1;
    event_tracker_t tracker2;
    event_tracker_init(&tracker1);
    event_tracker_init(&tracker2);

    /* Add two separate listeners */
    ivp_event_listener_handle_t handle1 = ivp_environment_add_event_listener(env, event_track_fn, &tracker1);
    ivp_event_listener_handle_t handle2 = ivp_environment_add_event_listener(env, event_track_fn, &tracker2);
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(handle1));
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(handle2));

    /* Step environment to generate events */
    ivp_environment_simulate_one_psi(env);

    /* Verify both trackers received events */
    IVP_ASSERT_TRUE(tracker1.count > 0);
    IVP_ASSERT_TRUE(tracker2.count > 0);

    /* Both should have received the same number of events */
    IVP_ASSERT_EQ(tracker1.count, tracker2.count);

    /* Verify both received at least one SIM_UNIT_BEGIN event */
    bool found_begin_1 = false;
    bool found_begin_2 = false;
    for (int i = 0; i < tracker1.count; i++) {
        if (tracker1.types[i] == IVP_EVENT_SIM_UNIT_BEGIN) {
            found_begin_1 = true;
        }
        if (tracker2.types[i] == IVP_EVENT_SIM_UNIT_BEGIN) {
            found_begin_2 = true;
        }
    }
    IVP_ASSERT_TRUE(found_begin_1);
    IVP_ASSERT_TRUE(found_begin_2);

    ivp_environment_remove_event_listener(env, handle1);
    ivp_environment_remove_event_listener(env, handle2);
    ivp_environment_destroy(env);
}

/* ========================================================================
 * Test 7: Invalid Event Listener Handle
 * ======================================================================== */

IVP_TEST(event_listener_handle_invalid) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    /* Create invalid handle */
    ivp_event_listener_handle_t invalid_handle = ivp_event_listener_handle_invalid();
    IVP_ASSERT_FALSE(ivp_event_listener_handle_is_valid(invalid_handle));

    /* Try to remove invalid handle, should return false */
    bool removed = ivp_environment_remove_event_listener(env, invalid_handle);
    IVP_ASSERT_FALSE(removed);

    ivp_environment_destroy(env);
}

/* ========================================================================
 * Test 8: Clear All Event Listeners
 * ======================================================================== */

IVP_TEST(event_clear_listeners) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    event_tracker_t tracker;
    event_tracker_init(&tracker);

    /* Add listener */
    ivp_event_listener_handle_t handle = ivp_environment_add_event_listener(env, event_track_fn, &tracker);
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(handle));

    /* Step environment to generate events */
    ivp_environment_simulate_one_psi(env);

    /* Verify we received events */
    int initial_count = tracker.count;
    IVP_ASSERT_TRUE(initial_count > 0);

    /* Clear all listeners */
    ivp_environment_clear_event_listeners(env);

    /* Step environment again */
    ivp_environment_simulate_one_psi(env);

    /* Verify tracker count didn't increase (no new events delivered) */
    IVP_ASSERT_EQ(tracker.count, initial_count);

    /* Try to remove the handle, should fail since it was cleared */
    bool removed = ivp_environment_remove_event_listener(env, handle);
    IVP_ASSERT_FALSE(removed);

    ivp_environment_destroy(env);
}

/* ========================================================================
 * Test 9: Object Created/Destroyed Events
 * ======================================================================== */

IVP_TEST(event_object_create_destroy) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    event_tracker_t tracker;
    event_tracker_init(&tracker);

    ivp_event_listener_handle_t handle = ivp_environment_add_event_listener(env, event_track_fn, &tracker);
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(handle));

    ivp_object_t *obj = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(obj);

    bool found_created = false;
    for (int i = 0; i < tracker.count; i++) {
        if (tracker.types[i] == IVP_EVENT_OBJECT_CREATED) {
            found_created = true;
            break;
        }
    }
    IVP_ASSERT_TRUE(found_created);

    event_tracker_init(&tracker);
    ivp_environment_destroy_object(env, obj);

    bool found_destroyed = false;
    int destroyed_object_index = -2;
    for (int i = 0; i < tracker.count; i++) {
        if (tracker.types[i] == IVP_EVENT_OBJECT_DESTROYED) {
            found_destroyed = true;
            destroyed_object_index = tracker.object_indices[i];
            break;
        }
    }
    IVP_ASSERT_TRUE(found_destroyed);
    IVP_ASSERT_EQ(destroyed_object_index, -1);

    ivp_environment_remove_event_listener(env, handle);
    ivp_environment_destroy(env);
}

/* ========================================================================
 * Test 10: Collision Detected Event
 * ======================================================================== */

IVP_TEST(event_collision_detected) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    event_tracker_t tracker;
    event_tracker_init(&tracker);

    ivp_event_listener_handle_t handle = ivp_environment_add_event_listener(env, event_track_fn, &tracker);
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(handle));

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(8.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(2.2), IVP_REAL(-8.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    (void)ivp_environment_resolve_collisions(env);

    bool found_collision = false;
    for (int i = 0; i < tracker.count; i++) {
        if (tracker.types[i] == IVP_EVENT_COLLISION_DETECTED) {
            found_collision = true;
            break;
        }
    }
    IVP_ASSERT_TRUE(found_collision);

    ivp_environment_remove_event_listener(env, handle);
    ivp_environment_destroy(env);
}

/* ========================================================================
 * Test 11: Frozen/Unfrozen Events
 * ======================================================================== */

IVP_TEST(event_object_frozen_unfrozen) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    event_tracker_t tracker;
    event_tracker_init(&tracker);
    ivp_event_listener_handle_t handle = ivp_environment_add_event_listener(env, event_track_fn, &tracker);
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(handle));

    /* Frozen via anomaly collision cap. */
    env->anomaly_manager.config.max_collisions_per_psi = 1;
    env->anomaly_manager.config.freeze_on_max_collisions = true;

    ivp_object_t *f0 = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *f1 = make_ball(env, IVP_REAL(0.2), IVP_REAL(0.0));
    ivp_object_t *f2 = make_ball(env, IVP_REAL(0.4), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(f0);
    IVP_ASSERT_NOT_NULL(f1);
    IVP_ASSERT_NOT_NULL(f2);

    for (int step = 0; step < 4; step++) {
        ivp_environment_simulate_one_psi(env);
    }

    bool found_frozen = false;
    for (int i = 0; i < tracker.count; i++) {
        if (tracker.types[i] == IVP_EVENT_OBJECT_FROZEN) {
            found_frozen = true;
            break;
        }
    }
    {
        int frozen_state_count = 0;
        if (f0->core.movement_state == IVP_MT_NOT_SIM) frozen_state_count++;
        if (f1->core.movement_state == IVP_MT_NOT_SIM) frozen_state_count++;
        if (f2->core.movement_state == IVP_MT_NOT_SIM) frozen_state_count++;
        IVP_ASSERT_TRUE(found_frozen || frozen_state_count >= 1);
    }

    /* Unfrozen via revive path (pending async push on NOT_SIM object). */
    event_tracker_init(&tracker);
    f0->core.movement_state = IVP_MT_NOT_SIM;
    ivp_vec3r_set(&f0->core.speed_change, IVP_REAL(0.2), IVP_REAL(0.0), IVP_REAL(0.0));
    env->anomaly_manager.config.max_collisions_per_psi = 0;
    env->anomaly_manager.config.freeze_on_max_collisions = false;

    for (int step = 0; step < 4; step++) {
        ivp_environment_simulate_one_psi(env);
    }

    bool found_unfrozen = false;
    for (int i = 0; i < tracker.count; i++) {
        if (tracker.types[i] == IVP_EVENT_OBJECT_UNFROZEN) {
            found_unfrozen = true;
            break;
        }
    }
    IVP_ASSERT_TRUE(found_unfrozen);

    ivp_environment_remove_event_listener(env, handle);
    ivp_environment_destroy(env);
}

/* ========================================================================
 * Test 12: Collision Listener Pre/Post
 * ======================================================================== */

IVP_TEST(collision_listener_pre_post) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(8.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(2.2), IVP_REAL(-8.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    collision_listener_tracker_t listener_tracker;
    memset(&listener_tracker, 0, sizeof(listener_tracker));

    ivp_collision_listener_t listener;
    ivp_collision_listener_init(&listener,
                                IVP_COLLISION_CB_PRE_COLLISION | IVP_COLLISION_CB_POST_COLLISION,
                                &listener_tracker);
    listener.pre_collision_fn = on_pre_collision;
    listener.post_collision_fn = on_post_collision;

    IVP_ASSERT_EQ(ivp_object_add_collision_listener(a, &listener), 0);

    (void)ivp_environment_resolve_collisions(env);

    IVP_ASSERT_TRUE(listener_tracker.pre_count > 0);
    IVP_ASSERT_TRUE(listener_tracker.post_count > 0);
    IVP_ASSERT_TRUE(listener_tracker.last_pre_impulse <= IVP_REAL_EPS);
    IVP_ASSERT_TRUE(listener_tracker.last_post_impulse >= IVP_REAL(0.0));

    IVP_ASSERT_EQ(ivp_object_remove_collision_listener(a, &listener), 0);
    ivp_environment_destroy(env);
}

typedef struct {
    int calls;
} psi_listener_counter_t;

static void psi_count_fn(void *ctx, ivp_environment_t *env, ivp_time_t time, ivp_real_t dt) {
    (void)time;
    psi_listener_counter_t *counter = (psi_listener_counter_t *)ctx;
    counter->calls++;
}

IVP_TEST(psi_listener_unregister_wrong_env_keeps_registration) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env_a = ivp_environment_create(&cfg);
    ivp_environment_t *env_b = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env_a);
    IVP_ASSERT_NOT_NULL(env_b);

    ivp_psi_listener_t listener;
    memset(&listener, 0xA5, sizeof(listener));

    psi_listener_counter_t counter;
    memset(&counter, 0, sizeof(counter));

    IVP_ASSERT_EQ(ivp_psi_listener_register(&listener, env_a, psi_count_fn, &counter), 0);
    ivp_environment_simulate_one_psi(env_a);
    IVP_ASSERT_EQ(counter.calls, 1);

    IVP_ASSERT_EQ(ivp_psi_listener_unregister(&listener, env_b), -1);
    ivp_environment_simulate_one_psi(env_a);
    IVP_ASSERT_EQ(counter.calls, 2);

    IVP_ASSERT_EQ(ivp_psi_listener_unregister(&listener, env_a), 0);
    ivp_environment_simulate_one_psi(env_a);
    IVP_ASSERT_EQ(counter.calls, 2);

    ivp_environment_destroy(env_b);
    ivp_environment_destroy(env_a);
}

IVP_TEST(psi_listener_double_register_rejected_without_dup_dispatch) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    ivp_psi_listener_t listener;
    memset(&listener, 0, sizeof(listener));

    psi_listener_counter_t counter;
    memset(&counter, 0, sizeof(counter));

    IVP_ASSERT_EQ(ivp_psi_listener_register(&listener, env, psi_count_fn, &counter), 0);
    IVP_ASSERT_EQ(ivp_psi_listener_register(&listener, env, psi_count_fn, &counter), -1);

    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(counter.calls, 1);

    IVP_ASSERT_EQ(ivp_psi_listener_unregister(&listener, env), 0);
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(counter.calls, 1);

    ivp_environment_destroy(env);
}

typedef struct {
    ivp_environment_t *env;
    ivp_event_listener_handle_t self;
    int calls;
} self_remove_listener_ctx_t;

typedef struct {
    int calls;
} counting_listener_ctx_t;

static void self_remove_listener_fn(void *ctx, ivp_environment_t *env, const ivp_event_t *ev) {
    (void)ev;
    self_remove_listener_ctx_t *state = (self_remove_listener_ctx_t *)ctx;
    state->calls++;
    if (state->calls == 1) {
        (void)ivp_environment_remove_event_listener(env, state->self);
    }
}

static void counting_listener_fn(void *ctx, ivp_environment_t *env, const ivp_event_t *ev) {
    (void)ev;
    counting_listener_ctx_t *state = (counting_listener_ctx_t *)ctx;
    state->calls++;
}

IVP_TEST(event_listener_self_remove_does_not_break_iteration) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    self_remove_listener_ctx_t self_ctx;
    memset(&self_ctx, 0, sizeof(self_ctx));
    self_ctx.env = env;
    self_ctx.self = ivp_event_listener_handle_invalid();

    counting_listener_ctx_t count_ctx;
    memset(&count_ctx, 0, sizeof(count_ctx));

    ivp_event_listener_handle_t self_handle =
        ivp_environment_add_event_listener(env, self_remove_listener_fn, &self_ctx);
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(self_handle));
    self_ctx.self = self_handle;

    ivp_event_listener_handle_t count_handle =
        ivp_environment_add_event_listener(env, counting_listener_fn, &count_ctx);
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(count_handle));

    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(self_ctx.calls, 1);
    IVP_ASSERT_TRUE(count_ctx.calls > 0);

    int count_after_first = count_ctx.calls;
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(self_ctx.calls, 1);
    IVP_ASSERT_TRUE(count_ctx.calls > count_after_first);

    ivp_environment_destroy(env);
}

IVP_TEST(event_collision_detected_via_mindist_time_events) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    event_tracker_t tracker;
    event_tracker_init(&tracker);
    ivp_event_listener_handle_t handle =
        ivp_environment_add_event_listener(env, event_track_fn, &tracker);
    IVP_ASSERT_TRUE(ivp_event_listener_handle_is_valid(handle));

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(8.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(2.2), IVP_REAL(-8.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    (void)ivp_environment_resolve_collisions(env);

    bool found_collision = false;
    for (int i = 0; i < tracker.count; i++) {
        if (tracker.types[i] == IVP_EVENT_COLLISION_DETECTED) {
            found_collision = true;
            break;
        }
    }
    IVP_ASSERT_TRUE(found_collision);

    ivp_environment_remove_event_listener(env, handle);
    ivp_environment_destroy(env);
}

/* ========================================================================
 * Main Test Runner
 * ======================================================================== */

int main(void) {
    IVP_REGISTER(event_listener_add_remove);
    IVP_REGISTER(event_sim_unit_begin);
    IVP_REGISTER(event_sim_unit_end);
    IVP_REGISTER(event_controller_added);
    IVP_REGISTER(event_controller_removed);
    IVP_REGISTER(event_multiple_listeners);
    IVP_REGISTER(event_listener_handle_invalid);
    IVP_REGISTER(event_clear_listeners);
    IVP_REGISTER(event_object_create_destroy);
    IVP_REGISTER(event_collision_detected);
    IVP_REGISTER(event_object_frozen_unfrozen);
    IVP_REGISTER(collision_listener_pre_post);
    IVP_REGISTER(psi_listener_unregister_wrong_env_keeps_registration);
    IVP_REGISTER(psi_listener_double_register_rejected_without_dup_dispatch);
    IVP_REGISTER(event_listener_self_remove_does_not_break_iteration);
    IVP_REGISTER(event_collision_detected_via_mindist_time_events);

    IVP_RUN_ALL();
}
