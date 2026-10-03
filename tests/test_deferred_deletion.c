/* test_deferred_deletion.c — deferred object deletion reentrancy
 *
 * Reference c90899a (tests/core/deferred_deletion_reentrancy.cxx): the
 * deletion queue stays deferred while end_deferred_deletion drains it.  A
 * deletion callback that deletes another object appends it to the queue
 * (it is deleted once, after the current object); one that deletes the
 * object being deleted is a no-op.
 */
#include "ivp_test.h"

#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/events/ivp_listener_object.h>
#include <ivp/math/ivp_quat.h>
#include "dynamics/ivp_sim_internal.h"

#include <string.h>

typedef struct cascade {
    ivp_listener_object_t listener;
    ivp_environment_t *env;
    ivp_object_t *trigger;   /* deleting it deletes 'victim' */
    ivp_object_t *victim;
    int deleted;
} cascade_t;

static void cascade_deleted(ivp_listener_object_t *self, ivp_event_object_t *ev)
{
    cascade_t *c = (cascade_t *)self->user_data;
    c->deleted++;
    if (ev->real_object == c->trigger && c->victim) {
        ivp_object_t *o = c->victim;
        c->victim = NULL;
        ivp_environment_destroy_object(c->env, o);
    }
}

static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t x)
{
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.ball_radius = IVP_REAL(0.5);
    t.mass = IVP_REAL(1.0);
    t.rot_inertia_is_factor = false;
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    ivp_vec3r_set(&t.position, x, IVP_REAL(0.0), IVP_REAL(0.0));
    t.orientation = ivp_quatr_identity();
    return ivp_environment_create_object(env, &t, NULL);
}

static ivp_environment_t *make_env(cascade_t *c)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    memset(c, 0, sizeof(*c));
    c->env = env;
    c->listener.event_object_deleted = cascade_deleted;
    c->listener.user_data = c;
    return env;
}

/* the reference scenario: both objects queued, deleting the first deletes
 * the second (already queued) */
IVP_TEST(deferred_deletion_callback_deletes_queued_object)
{
    cascade_t c;
    ivp_environment_t *env = make_env(&c);
    c.trigger = make_ball(env, IVP_REAL(0.0));
    c.victim = make_ball(env, IVP_REAL(2.0));
    ivp_object_t *second = c.victim;
    int n0 = ivp_environment_get_object_count(env);
    ivp_environment_add_listener_object_global(env, &c.listener);

    ivp_env_begin_deferred_deletion(env);
    ivp_environment_destroy_object(env, c.trigger);
    ivp_environment_destroy_object(env, second);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(env), n0);
    ivp_env_end_deferred_deletion(env);

    IVP_ASSERT_EQ(c.deleted, 2);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(env), n0 - 2);
    ivp_environment_remove_listener_object_global(env, &c.listener);
    ivp_environment_destroy(env);
}

/* a callback during the drain deletes an object that was not queued: it is
 * appended and deleted after the current one */
IVP_TEST(deferred_deletion_callback_appends_to_queue)
{
    cascade_t c;
    ivp_environment_t *env = make_env(&c);
    c.trigger = make_ball(env, IVP_REAL(0.0));
    c.victim = make_ball(env, IVP_REAL(2.0));
    int n0 = ivp_environment_get_object_count(env);
    ivp_environment_add_listener_object_global(env, &c.listener);

    ivp_env_begin_deferred_deletion(env);
    ivp_environment_destroy_object(env, c.trigger);
    ivp_env_end_deferred_deletion(env);

    IVP_ASSERT_EQ(c.deleted, 2);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(env), n0 - 2);
    ivp_environment_remove_listener_object_global(env, &c.listener);
    ivp_environment_destroy(env);
}

/* a callback deletes the object whose deletion is running */
IVP_TEST(deferred_deletion_callback_deletes_current_object)
{
    cascade_t c;
    ivp_environment_t *env = make_env(&c);
    c.trigger = make_ball(env, IVP_REAL(0.0));
    c.victim = c.trigger;
    int n0 = ivp_environment_get_object_count(env);
    ivp_environment_add_listener_object_global(env, &c.listener);

    ivp_env_begin_deferred_deletion(env);
    ivp_environment_destroy_object(env, c.trigger);
    ivp_env_end_deferred_deletion(env);

    IVP_ASSERT_EQ(c.deleted, 1);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(env), n0 - 1);
    ivp_environment_remove_listener_object_global(env, &c.listener);
    ivp_environment_destroy(env);
}

/* the same outside a deferred scope */
IVP_TEST(immediate_deletion_callback_deletes_current_object)
{
    cascade_t c;
    ivp_environment_t *env = make_env(&c);
    c.trigger = make_ball(env, IVP_REAL(0.0));
    c.victim = c.trigger;
    int n0 = ivp_environment_get_object_count(env);
    ivp_environment_add_listener_object_global(env, &c.listener);

    ivp_environment_destroy_object(env, c.trigger);

    IVP_ASSERT_EQ(c.deleted, 1);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(env), n0 - 1);
    ivp_environment_remove_listener_object_global(env, &c.listener);
    ivp_environment_destroy(env);
}

int main(void)
{
    IVP_REGISTER(deferred_deletion_callback_deletes_queued_object);
    IVP_REGISTER(deferred_deletion_callback_appends_to_queue);
    IVP_REGISTER(deferred_deletion_callback_deletes_current_object);
    IVP_REGISTER(immediate_deletion_callback_deletes_current_object);
    IVP_RUN_ALL();
}
