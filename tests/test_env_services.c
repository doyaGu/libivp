/*
 * test_env_services.c
 *
 * Environment services: IVP_Controller_Phantom (ivp_object_convert_to_phantom),
 * object / collision / PSI listeners, universe manager, active value manager
 * and user time events.
 */
#include "ivp_test.h"
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_phantom.h>
#include <ivp/events/ivp_listener_object.h>
#include <ivp/events/ivp_listener_collision.h>
#include <ivp/dynamics/ivp_universe_manager.h>
#include <ivp/utility/ivp_active_value.h>
#include <ivp/dynamics/ivp_time_manager.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/collision/ivp_collision_core.h>
#include <math.h>
#include <string.h>

typedef struct {
    ivp_listener_phantom_t base;
    int mindist_entered, mindist_left, core_entered, core_left, deleted;
    ivp_object_t *last_other;
} phantom_counter_t;

static void pc_md_entered(ivp_listener_phantom_t *self, ivp_controller_phantom_t *c, struct ivp_mindist *md)
{
    phantom_counter_t *pc = (phantom_counter_t *)self;
    pc->mindist_entered++;
    pc->last_other = ivp_controller_phantom_get_other_object(c, md);
}
static void pc_md_left(ivp_listener_phantom_t *self, ivp_controller_phantom_t *c, struct ivp_mindist *md)
{
    (void)c; (void)md;
    ((phantom_counter_t *)self)->mindist_left++;
}
static void pc_core_entered(ivp_listener_phantom_t *self, ivp_controller_phantom_t *c, struct ivp_core *core)
{
    (void)c; (void)core;
    ((phantom_counter_t *)self)->core_entered++;
}
static void pc_core_left(ivp_listener_phantom_t *self, ivp_controller_phantom_t *c, struct ivp_core *core)
{
    (void)c; (void)core;
    ((phantom_counter_t *)self)->core_left++;
}
static void pc_deleted(ivp_listener_phantom_t *self, ivp_controller_phantom_t *c)
{
    (void)c;
    ((phantom_counter_t *)self)->deleted++;
}

static void phantom_counter_init(phantom_counter_t *pc)
{
    memset(pc, 0, sizeof(*pc));
    pc->base.mindist_entered_volume = pc_md_entered;
    pc->base.mindist_left_volume = pc_md_left;
    pc->base.core_entered_volume = pc_core_entered;
    pc->base.core_left_volume = pc_core_left;
    pc->base.phantom_is_going_to_be_deleted_event = pc_deleted;
}

static ivp_environment_t *make_env(void)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

static ivp_object_t *make_static_box(ivp_environment_t *env, ivp_surman_polygon_t *sm, ivp_compact_surface_t **surf)
{
    *surf = ivp_build_box_surface(NULL, 2.0f, 2.0f, 2.0f);
    if (!*surf) return NULL;
    ivp_surman_polygon_init(sm, *surf);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.physical_unmoveable = true;
    tmpl.mass = IVP_REAL(1.0);
    return ivp_environment_create_object(env, &tmpl, &sm->base);
}

static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t x, ivp_real_t vx)
{
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.ball_radius = IVP_REAL(0.5);
    tmpl.mass = IVP_REAL(1.0);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&tmpl.position, x, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    if (obj) ivp_vec3r_set(&obj->physical_core->speed, vx, IVP_REAL(0.0), IVP_REAL(0.0));
    return obj;
}

IVP_TEST(phantom_reports_ball_passing_through)
{
    ivp_environment_t *env = make_env();
    IVP_ASSERT_TRUE(env != NULL);
    ivp_surman_polygon_t sm;
    ivp_compact_surface_t *surf = NULL;
    ivp_object_t *box = make_static_box(env, &sm, &surf);
    IVP_ASSERT_TRUE(box != NULL);

    ivp_template_phantom_t pt;
    ivp_template_phantom_init(&pt);
    pt.manage_intruding_objects = true;
    pt.manage_intruding_cores = true;
    ivp_controller_phantom_t *ph = ivp_object_convert_to_phantom(box, &pt);
    IVP_ASSERT_TRUE(ph != NULL);
    IVP_ASSERT_TRUE(ivp_object_get_controller_phantom(box) == ph);
    IVP_ASSERT_EQ((int)box->object_movement_state, (int)IVP_MT_STATIC_PHANTOM);

    phantom_counter_t pc;
    phantom_counter_init(&pc);
    ivp_controller_phantom_add_listener(ph, &pc.base);

    ivp_object_t *ball = make_ball(env, IVP_REAL(-6.0), IVP_REAL(5.0));
    IVP_ASSERT_TRUE(ball != NULL);

    int max_inside = 0;
    for (int i = 0; i < 200; i++) { /* 200 / 66 s: ball travels ~15 m */
        ivp_environment_simulate_one_psi(env);
        int n = ivp_set_count(&ivp_controller_phantom_get_intruding_objects(ph)->set);
        if (n > max_inside) max_inside = n;
    }
    IVP_ASSERT_EQ(max_inside, 1);
    IVP_ASSERT_TRUE(pc.mindist_entered >= 1);
    IVP_ASSERT_EQ(pc.mindist_entered, pc.mindist_left);
    IVP_ASSERT_EQ(pc.core_entered, 1);
    IVP_ASSERT_EQ(pc.core_left, 1);
    IVP_ASSERT_TRUE(pc.last_other == ball);
    IVP_ASSERT_EQ(ivp_set_count(&ivp_controller_phantom_get_intruding_objects(ph)->set), 0);
    /* a phantom never pushes */
    IVP_ASSERT_NEAR((double)ball->physical_core->speed.k[0], 5.0, 1e-4);
    IVP_ASSERT_NEAR((double)ball->physical_core->speed.k[1], 0.0, 1e-6);
    IVP_ASSERT_TRUE(ball->physical_core->m_world_f_core_last_psi.pos.k[0] > IVP_REAL(6.0));

    ivp_environment_destroy_object(env, box);
    IVP_ASSERT_EQ(pc.deleted, 1);
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, surf);
}

typedef struct {
    ivp_listener_object_t base;
    int created, deleted, revived, frozen, env_deleted;
    ivp_object_t *last;
} obj_counter_t;

static void oc_created(ivp_listener_object_t *self, ivp_event_object_t *ev)
{
    obj_counter_t *oc = (obj_counter_t *)self;
    oc->created++;
    oc->last = ev->real_object;
}
static void oc_deleted(ivp_listener_object_t *self, ivp_event_object_t *ev)
{
    obj_counter_t *oc = (obj_counter_t *)self;
    oc->deleted++;
    oc->last = ev->real_object;
}
static void oc_revived(ivp_listener_object_t *self, ivp_event_object_t *ev)
{
    (void)ev;
    ((obj_counter_t *)self)->revived++;
}
static void oc_frozen(ivp_listener_object_t *self, ivp_event_object_t *ev)
{
    (void)ev;
    ((obj_counter_t *)self)->frozen++;
}
static void oc_env_deleted(ivp_listener_object_t *self, struct ivp_environment *env)
{
    (void)env;
    ((obj_counter_t *)self)->env_deleted++;
}

static void obj_counter_init(obj_counter_t *oc)
{
    memset(oc, 0, sizeof(*oc));
    oc->base.event_object_created = oc_created;
    oc->base.event_object_deleted = oc_deleted;
    oc->base.event_object_revived = oc_revived;
    oc->base.event_object_frozen = oc_frozen;
    oc->base.event_environment_deleted = oc_env_deleted;
}

IVP_TEST(object_listeners_global_and_private)
{
    ivp_environment_t *env = make_env();
    IVP_ASSERT_TRUE(env != NULL);
    obj_counter_t global, priv;
    obj_counter_init(&global);
    obj_counter_init(&priv);
    ivp_environment_add_listener_object_global(env, &global.base);

    ivp_object_t *ball = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    IVP_ASSERT_TRUE(ball != NULL);
    IVP_ASSERT_EQ(global.created, 1);
    IVP_ASSERT_TRUE(global.last == ball);
    /* created in simulation: revived once */
    IVP_ASSERT_EQ(global.revived, 1);

    ivp_object_add_listener_object(ball, &priv.base);
    IVP_ASSERT_TRUE(ball->object_listener_exists);

    /* a ball at rest freezes after the calm period */
    for (int i = 0; i < 400 && global.frozen == 0; i++) ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(global.frozen, 1);
    IVP_ASSERT_EQ(priv.frozen, 1);

    ivp_environment_ensure_core_in_simulation(env, ball);
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(global.revived, 2);
    IVP_ASSERT_EQ(priv.revived, 1);

    ivp_object_remove_listener_object(ball, &priv.base);
    IVP_ASSERT_TRUE(!ball->object_listener_exists);
    ivp_object_add_listener_object(ball, &priv.base);

    ivp_environment_destroy_object(env, ball);
    IVP_ASSERT_EQ(global.deleted, 1);
    IVP_ASSERT_EQ(priv.deleted, 1);
    IVP_ASSERT_TRUE(priv.last == ball);

    ivp_environment_destroy(env);
    IVP_ASSERT_EQ(global.env_deleted, 1);
}


typedef struct {
    int pre, post, env_deleted;
    double last_dt;
    ivp_environment_t *env_seen;
    double speed_len;
} coll_counter_t;

static void cc_pre(void *ctx, const ivp_collision_event_t *ev)
{
    coll_counter_t *c = (coll_counter_t *)ctx;
    c->pre++;
    c->last_dt = (double)ev->d_time_since_last_collision;
    c->env_seen = ev->environment;
}
/* contact_situation->speed is the return value of the impact solver: only
 * meaningful in the post collision event (as in the reference) */
static void cc_post(void *ctx, const ivp_collision_event_t *ev)
{
    coll_counter_t *c = (coll_counter_t *)ctx;
    c->post++;
    if (c->post == 1) {
        c->speed_len = sqrt((double)(ev->speed.k[0] * ev->speed.k[0] + ev->speed.k[1] * ev->speed.k[1] +
                                     ev->speed.k[2] * ev->speed.k[2]));
    }
}
static void cc_env_deleted(void *ctx, struct ivp_environment *env)
{
    (void)env;
    ((coll_counter_t *)ctx)->env_deleted++;
}

IVP_TEST(collision_listeners_unlimited_and_event_fields)
{
    ivp_environment_t *env = make_env();
    IVP_ASSERT_TRUE(env != NULL);
    ivp_object_t *a = make_ball(env, IVP_REAL(-2.0), IVP_REAL(3.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(2.0), IVP_REAL(-3.0));
    IVP_ASSERT_TRUE(a && b);

    coll_counter_t global;
    memset(&global, 0, sizeof(global));
    ivp_collision_listener_t gl;
    ivp_collision_listener_init(&gl, IVP_COLLISION_CB_ALL, &global);
    gl.pre_collision_fn = cc_pre;
    gl.post_collision_fn = cc_post;
    gl.environment_deleted_fn = cc_env_deleted;
    IVP_ASSERT_EQ(ivp_environment_add_collision_listener_global(env, &gl), 0);

    enum { N = 6 };
    coll_counter_t per[N];
    ivp_collision_listener_t pl[N];
    for (int i = 0; i < N; i++) {
        memset(&per[i], 0, sizeof(per[i]));
        ivp_collision_listener_init(&pl[i], IVP_COLLISION_CB_PRE_COLLISION | IVP_COLLISION_CB_POST_COLLISION, &per[i]);
        pl[i].pre_collision_fn = cc_pre;
        pl[i].post_collision_fn = cc_post;
        IVP_ASSERT_EQ(ivp_object_add_collision_listener(a, &pl[i]), 0);
    }
    IVP_ASSERT_TRUE(a->collision_listener_exists);

    for (int i = 0; i < 66; i++) ivp_environment_simulate_one_psi(env);

    IVP_ASSERT_TRUE(global.pre >= 1);
    IVP_ASSERT_EQ(global.pre, global.post);
    IVP_ASSERT_TRUE(global.env_seen == env);
    IVP_ASSERT_TRUE(global.speed_len > 1.0);
    IVP_ASSERT_TRUE(global.last_dt > 0.0);
    for (int i = 0; i < N; i++) {
        IVP_ASSERT_EQ(per[i].pre, global.pre);
        IVP_ASSERT_EQ(per[i].post, global.post);
    }
    for (int i = 0; i < N; i++) IVP_ASSERT_EQ(ivp_object_remove_collision_listener(a, &pl[i]), 0);
    IVP_ASSERT_TRUE(!a->collision_listener_exists);

    ivp_environment_destroy(env);
    IVP_ASSERT_EQ(global.env_deleted, 1);
}

typedef struct {
    ivp_environment_t *env;
    int evicted;
    int deleted;
} universe_ctx_t;

static void um_evict(void *ctx, ivp_object_t *obj)
{
    universe_ctx_t *u = (universe_ctx_t *)ctx;
    u->evicted++;
    ivp_environment_destroy_object(u->env, obj); /* object_no_longer_needed */
}
static void um_deleted(void *ctx, ivp_object_t *obj)
{
    (void)obj;
    ((universe_ctx_t *)ctx)->deleted++;
}

IVP_TEST(universe_manager_evicts_unreferenced_objects)
{
    ivp_environment_t *env = make_env();
    IVP_ASSERT_TRUE(env != NULL);
    universe_ctx_t u = { env, 0, 0 };
    ivp_universe_manager_t mgr;
    ivp_universe_manager_init(&mgr);
    mgr.evict_fn = um_evict;
    mgr.deleted_fn = um_deleted;
    mgr.ctx = &u;
    ivp_environment_set_universe_manager(env, &mgr);

    /* two balls touching each other reference each other; a far one does not */
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.2), IVP_REAL(0.0));
    ivp_object_t *far = make_ball(env, IVP_REAL(500.0), IVP_REAL(0.0));
    IVP_ASSERT_TRUE(a && b && far);
    IVP_ASSERT_TRUE(ivp_object_get_collision_check_reference_count(a) >= 1);
    IVP_ASSERT_EQ(ivp_object_get_collision_check_reference_count(far), 0);

    for (int i = 0; i < 10; i++) ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(u.evicted, 1);
    IVP_ASSERT_EQ(u.deleted, 1);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(env), 2);
    ivp_environment_destroy(env);
}

IVP_TEST(active_value_manager_refreshed_each_psi)
{
    ivp_environment_t *env = make_env();
    IVP_ASSERT_TRUE(env != NULL);
    ivp_active_value_manager_t *m = ivp_environment_get_active_value_manager(env);
    IVP_ASSERT_TRUE(m != NULL);
    ivp_active_float_t *t = ivp_active_value_manager_current_time(m);
    IVP_ASSERT_TRUE(t != NULL);
    IVP_ASSERT_TRUE(ivp_environment_get_static_object(env) != NULL);
    IVP_ASSERT_EQ(ivp_environment_get_object_count(env), 0);
    for (int i = 0; i < 3; i++) ivp_environment_simulate_one_psi(env);
    /* refresh_psi_active_values(current_time) at the start of the 3rd PSI */
    double dt = (double)(float)(1.0f / 66.0f);
    IVP_ASSERT_NEAR(ivp_active_float_get(t), 2.0 * dt, 1e-9);
    ivp_environment_destroy(env);
}

typedef struct {
    ivp_time_event_t ev;
    int fired;
    double fired_at;
} user_event_t;

static void user_event_fn(ivp_time_event_t *ev, struct ivp_environment *env)
{
    user_event_t *u = (user_event_t *)ev;
    u->fired++;
    u->fired_at = (double)ivp_environment_get_time(env);
}

IVP_TEST(user_time_event_fires_between_psis)
{
    ivp_environment_t *env = make_env();
    IVP_ASSERT_TRUE(env != NULL);
    user_event_t u;
    memset(&u, 0, sizeof(u));
    ivp_time_event_init(&u.ev, user_event_fn);
    for (int i = 0; i < 3; i++) ivp_environment_simulate_one_psi(env);
    /* due several PSIs later */
    double t = (double)ivp_environment_get_time(env) + 0.1;
    ivp_time_manager_insert_event(ivp_environment_get_time_manager(env), &u.ev, t);
    ivp_environment_simulate_until(env, t + 0.05);
    IVP_ASSERT_EQ(u.fired, 1);
    IVP_ASSERT_NEAR(u.fired_at, t, 1e-6);
    ivp_environment_destroy(env);
}

int main(void)
{
    IVP_REGISTER(phantom_reports_ball_passing_through);
    IVP_REGISTER(object_listeners_global_and_private);
    IVP_REGISTER(collision_listeners_unlimited_and_event_fields);
    IVP_REGISTER(universe_manager_evicts_unreferenced_objects);
    IVP_REGISTER(active_value_manager_refreshed_each_psi);
    IVP_REGISTER(user_time_event_fires_between_psis);
    IVP_RUN_ALL();
}
