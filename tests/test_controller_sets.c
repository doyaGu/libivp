/*
 * IVP Physics Engine - C17 Port
 * Set-driven controllers
 *
 *   - IVP_Attacher_To_Cores (ivp_physics/ivp_attacher_to_cores.hxx)
 *   - IVP_Forcefield driven by an IVP_U_Set_Active<IVP_Core>
 *     (ivp_controller/ivp_forcefield.cxx)
 *   - IVP_Attacher_To_Cores_Buoyancy and the unbounded per-core data of
 *     the libivp all-cores buoyancy registration
 */

#include "ivp_test.h"

#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/controllers/ivp_attacher_to_cores.h>
#include <ivp/controllers/ivp_forcefield.h>
#include <ivp/controllers/ivp_controller_buoyancy.h>
#include <ivp/containers/ivp_set.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifdef IVP_REAL_DOUBLE
  #define LTOL 1e-6
#else
  #define LTOL 1e-3f
#endif

#define DT60 (IVP_REAL(1.0) / IVP_REAL(60.0))

static ivp_environment_t *make_env(bool gravity)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = DT60;
    if (gravity) ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.81), IVP_REAL(0.0));
    else ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t x, ivp_real_t y, ivp_real_t z)
{
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.ball_radius = IVP_REAL(0.5);
    t.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    t.rot_inertia_is_factor = false;
    t.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&t.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&t.position, x, y, z);
    t.orientation = ivp_quatr_identity();
    return ivp_environment_create_object(env, &t, NULL);
}

static void step(ivp_environment_t *env, int n)
{
    for (int i = 0; i < n; i++) ivp_environment_simulate_one_psi(env);
}

/* ── IVP_Attacher_To_Cores ────────────────────────────────────────────── */

static int g_created, g_deleted, g_attacher_deleted;

typedef struct test_attachment {
    ivp_core_t *core;
} test_attachment_t;

static void *cnt_create(ivp_attacher_to_cores_t *a, ivp_core_t *core)
{
    (void)a;
    test_attachment_t *t = (test_attachment_t *)malloc(sizeof(*t));
    t->core = core;
    g_created++;
    return t;
}

static void cnt_delete(ivp_attacher_to_cores_t *a, ivp_core_t *core, void *attachment)
{
    (void)a;
    test_attachment_t *t = (test_attachment_t *)attachment;
    if (t->core == core) g_deleted++;
    free(t);
}

static void cnt_attacher_deleted(ivp_attacher_to_cores_t *a)
{
    (void)a;
    g_attacher_deleted++;
}

static const ivp_attacher_to_cores_class_t cnt_class = { cnt_create, cnt_delete, cnt_attacher_deleted };

IVP_TEST(attacher_follows_set) {
    g_created = g_deleted = g_attacher_deleted = 0;
    ivp_environment_t *env = make_env(false);
    ivp_object_t *o1 = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *o2 = make_ball(env, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *o3 = make_ball(env, IVP_REAL(6.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_set_active_t set;
    ivp_set_active_init(&set, NULL, 16);
    ivp_set_active_add(&set, o1->physical_core);
    ivp_set_active_add(&set, o2->physical_core);

    ivp_attacher_to_cores_t at;
    IVP_ASSERT_TRUE(ivp_attacher_to_cores_init(&at, env, &set, &cnt_class, NULL));
    IVP_ASSERT_TRUE(at.core_to_attachment_hash.alloc == &at.allocator);
    IVP_ASSERT_TRUE(at.allocator.alloc == env->allocator->alloc);
    IVP_ASSERT_TRUE(at.allocator.realloc == env->allocator->realloc);
    IVP_ASSERT_TRUE(at.allocator.free == env->allocator->free);
    IVP_ASSERT_TRUE(at.allocator.ctx == env->allocator->ctx);
    IVP_ASSERT_EQ(g_created, 2);
    IVP_ASSERT_EQ(ivp_attacher_to_cores_count(&at), 2);
    IVP_ASSERT_NOT_NULL(ivp_attacher_to_cores_get_attachment(&at, o1->physical_core));
    IVP_ASSERT_NULL(ivp_attacher_to_cores_get_attachment(&at, o3->physical_core));
    IVP_ASSERT_TRUE(ivp_attacher_core_object(o2->physical_core) == o2);

    ivp_set_active_add(&set, o3->physical_core);
    IVP_ASSERT_EQ(g_created, 3);
    ivp_set_active_install(&set, o3->physical_core); /* already in the set */
    IVP_ASSERT_EQ(g_created, 3);

    ivp_set_active_remove(&set, o2->physical_core);
    IVP_ASSERT_EQ(g_deleted, 1);
    IVP_ASSERT_EQ(ivp_attacher_to_cores_count(&at), 2);

    /* Core deletion deletes the attachment and removes the invalid set
     * entry. */
    ivp_core_t *dead = o3->physical_core;
    ivp_environment_destroy_object(env, o3);
    IVP_ASSERT_EQ(g_deleted, 2);
    IVP_ASSERT_EQ(ivp_attacher_to_cores_count(&at), 1);
    IVP_ASSERT_NULL(ivp_set_find(&set.set, dead));

    /* deleting the set deletes the attachments and the attacher */
    ivp_set_active_destroy(&set);
    IVP_ASSERT_EQ(g_deleted, 3);
    IVP_ASSERT_EQ(g_attacher_deleted, 1);
    IVP_ASSERT_EQ(g_created, g_deleted);
    ivp_environment_destroy(env);
}

IVP_TEST(attacher_destroy_and_environment_deletion) {
    g_created = g_deleted = g_attacher_deleted = 0;
    ivp_environment_t *env = make_env(false);
    ivp_object_t *o1 = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_set_active_t set;
    ivp_set_active_init(&set, NULL, 16);
    ivp_set_active_add(&set, o1->physical_core);

    ivp_attacher_to_cores_t at;
    IVP_ASSERT_TRUE(ivp_attacher_to_cores_init(&at, env, &set, &cnt_class, NULL));
    ivp_attacher_to_cores_destroy(&at);
    IVP_ASSERT_EQ(g_deleted, 1);
    IVP_ASSERT_EQ(set.n_listeners, 0);
    IVP_ASSERT_EQ(g_attacher_deleted, 0);

    /* environment deleted first: attachments go with it */
    IVP_ASSERT_TRUE(ivp_attacher_to_cores_init(&at, env, &set, &cnt_class, NULL));
    IVP_ASSERT_EQ(g_created, 2);
    ivp_environment_destroy(env);
    IVP_ASSERT_EQ(g_deleted, 2);
    IVP_ASSERT_TRUE(at.environment_deleted);
    IVP_ASSERT_EQ(ivp_set_count(&set.set), 0);
    ivp_set_active_destroy(&set);
    IVP_ASSERT_EQ(g_attacher_deleted, 1);
    IVP_ASSERT_EQ(g_deleted, 2);
}

static void *no_attachment_create(ivp_attacher_to_cores_t *a,
                                  ivp_core_t *core)
{
    (void)a;
    (void)core;
    g_created++;
    return NULL;
}

static const ivp_attacher_to_cores_class_t no_attachment_class = {
    no_attachment_create,
    cnt_delete,
    cnt_attacher_deleted
};

IVP_TEST(attacher_allows_core_without_attachment) {
    g_created = g_deleted = g_attacher_deleted = 0;
    ivp_environment_t *env = make_env(false);
    ivp_object_t *object = make_ball(
        env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_set_active_t set;
    ivp_set_active_init(&set, NULL, 16);
    ivp_set_active_add(&set, object->physical_core);

    ivp_attacher_to_cores_t attacher;
    IVP_ASSERT_TRUE(ivp_attacher_to_cores_init(
        &attacher, env, &set, &no_attachment_class, NULL));
    IVP_ASSERT_EQ(g_created, 1);
    IVP_ASSERT_EQ(ivp_attacher_to_cores_count(&attacher), 0);
    ivp_set_active_destroy(&set);
    IVP_ASSERT_EQ(g_deleted, 0);
    IVP_ASSERT_EQ(g_attacher_deleted, 1);
    ivp_environment_destroy(env);
}

static bool far_x_filter(void *ctx, ivp_object_t *obj)
{
    (void)ctx;
    return obj->physical_core->m_world_f_core_last_psi.pos.k[0] > IVP_REAL(10.0);
}

IVP_TEST(attacher_object_filter_installs_created_objects) {
    g_created = g_deleted = g_attacher_deleted = 0;
    ivp_environment_t *env = make_env(false);
    ivp_set_active_t set;
    ivp_set_active_init(&set, NULL, 16);
    ivp_attacher_to_cores_t at;
    IVP_ASSERT_TRUE(ivp_attacher_to_cores_init(&at, env, &set, &cnt_class, NULL));
    ivp_attacher_to_cores_set_object_filter(&at, far_x_filter, NULL);
    ivp_object_t *near = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *far = make_ball(env, IVP_REAL(20.0), IVP_REAL(0.0), IVP_REAL(0.0));
    IVP_ASSERT_EQ(ivp_set_count(&set.set), 1);
    IVP_ASSERT_NULL(ivp_attacher_to_cores_get_attachment(&at, near->physical_core));
    IVP_ASSERT_NOT_NULL(ivp_attacher_to_cores_get_attachment(&at, far->physical_core));
    ivp_attacher_to_cores_destroy(&at);
    IVP_ASSERT_EQ(g_created, g_deleted);
    ivp_set_active_destroy(&set);
    ivp_environment_destroy(env);
}

/* ── Set-driven IVP_Forcefield ────────────────────────────────────────── */

static int g_ff_deleted;

static void ff_deleted_cb(void *ctx, ivp_forcefield_state_t *f)
{
    (void)ctx;
    (void)f;
    g_ff_deleted++;
}

IVP_TEST(forcefield_set_driven_acts_on_set_cores_only) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_set_active_t set;
    ivp_set_active_init(&set, NULL, 16);
    ivp_set_active_add(&set, a->physical_core);

    ivp_forcefield_state_t ff;
    ivp_vec3r_t acc = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(10.0));
    ivp_forcefield_state_init(&ff, IVP_FORCEFIELD_CONSTANT, &acc);
    IVP_ASSERT_TRUE(ivp_forcefield_attach_set(&ff, env, &set, false));
    IVP_ASSERT_EQ(ivp_pvec_len(&ff.attached_cores), 1);

    step(env, 10);
    const ivp_real_t dv10 = IVP_REAL(10.0) * DT60 * IVP_REAL(10.0);
    IVP_ASSERT_NEAR(a->physical_core->speed.k[2], dv10, LTOL);
    IVP_ASSERT_NEAR(b->physical_core->speed.k[2], IVP_REAL(0.0), 0.0);

    ivp_set_active_add(&set, b->physical_core);
    step(env, 10);
    IVP_ASSERT_NEAR(a->physical_core->speed.k[2], dv10 * IVP_REAL(2.0), LTOL);
    IVP_ASSERT_NEAR(b->physical_core->speed.k[2], dv10, LTOL);

    ivp_set_active_remove(&set, a->physical_core);
    IVP_ASSERT_EQ(ivp_pvec_len(&ff.attached_cores), 1);
    step(env, 10);
    IVP_ASSERT_NEAR(a->physical_core->speed.k[2], dv10 * IVP_REAL(2.0), LTOL);
    IVP_ASSERT_NEAR(b->physical_core->speed.k[2], dv10 * IVP_REAL(2.0), LTOL);

    ivp_forcefield_destroy(&ff);
    IVP_ASSERT_EQ(set.n_listeners, 0);
    step(env, 10);
    IVP_ASSERT_NEAR(b->physical_core->speed.k[2], dv10 * IVP_REAL(2.0), LTOL);
    ivp_set_active_destroy(&set);
    ivp_environment_destroy(env);
}

IVP_TEST(forcefield_empty_set_attaches_nothing) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_set_active_t set;
    ivp_set_active_init(&set, NULL, 16);
    ivp_forcefield_state_t ff;
    ivp_vec3r_t acc = ivp_vec3r(IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_forcefield_state_init(&ff, IVP_FORCEFIELD_CONSTANT, &acc);
    IVP_ASSERT_TRUE(ivp_forcefield_attach_set(&ff, env, &set, true));
    step(env, 5);
    IVP_ASSERT_NEAR(a->physical_core->speed.k[0], IVP_REAL(0.0), 0.0);
    /* objects created later are not attached either */
    ivp_object_t *c = make_ball(env, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    step(env, 5);
    IVP_ASSERT_NEAR(c->physical_core->speed.k[0], IVP_REAL(0.0), 0.0);
    ivp_set_active_add(&set, c->physical_core);
    step(env, 6);
    IVP_ASSERT_NEAR(c->physical_core->speed.k[0], IVP_REAL(5.0) * DT60 * IVP_REAL(6.0), LTOL);
    IVP_ASSERT_NEAR(a->physical_core->speed.k[0], IVP_REAL(0.0), 0.0);
    ivp_forcefield_destroy(&ff);
    ivp_set_active_destroy(&set);
    ivp_environment_destroy(env);
}

IVP_TEST(forcefield_core_deletion_removes_core_from_set) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_set_active_t set;
    ivp_set_active_init(&set, NULL, 16);
    ivp_set_active_add(&set, a->physical_core);
    ivp_set_active_add(&set, b->physical_core);
    ivp_forcefield_state_t ff;
    ivp_vec3r_t acc = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_forcefield_state_init(&ff, IVP_FORCEFIELD_CONSTANT, &acc);
    IVP_ASSERT_TRUE(ivp_forcefield_attach_set(&ff, env, &set, false));

    ivp_environment_destroy_object(env, a);
    IVP_ASSERT_EQ(ivp_set_count(&set.set), 1);
    IVP_ASSERT_EQ(ivp_pvec_len(&ff.attached_cores), 1);
    step(env, 3);
    IVP_ASSERT_NEAR(b->physical_core->speed.k[0], DT60 * IVP_REAL(3.0), LTOL);

    /* deleting the set deletes the field */
    g_ff_deleted = 0;
    ivp_forcefield_set_deleted_listener(&ff, ff_deleted_cb, NULL);
    ivp_set_active_destroy(&set);
    IVP_ASSERT_EQ(g_ff_deleted, 1);
    IVP_ASSERT_FALSE(ff.set_driven);
    step(env, 3);
    IVP_ASSERT_NEAR(b->physical_core->speed.k[0], DT60 * IVP_REAL(3.0), LTOL);
    ivp_forcefield_destroy(&ff); /* no-op */
    ivp_environment_destroy(env);
}

IVP_TEST(forcefield_environment_deleted_first) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_set_active_t set;
    ivp_set_active_init(&set, NULL, 16);
    ivp_set_active_add(&set, a->physical_core);
    ivp_forcefield_state_t ff;
    ivp_vec3r_t acc = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_forcefield_state_init(&ff, IVP_FORCEFIELD_CONSTANT, &acc);
    IVP_ASSERT_TRUE(ivp_forcefield_attach_set(&ff, env, &set, false));
    ivp_environment_destroy(env);
    /* the deleted cores left the set (reference core_is_going_to_be_deleted_event) */
    IVP_ASSERT_EQ(ivp_set_count(&set.set), 0);
    IVP_ASSERT_NULL(ff.environment);
    ivp_forcefield_destroy(&ff);
    IVP_ASSERT_EQ(set.n_listeners, 0);
    ivp_set_active_destroy(&set);
}

/* ── Buoyancy ─────────────────────────────────────────────────────────── */

static void buoy_template(ivp_buoyancy_state_t *s)
{
    /* liquid for y > 0 (Y-down), density 1 */
    ivp_buoyancy_init(s, IVP_REAL(0.0), IVP_REAL(1.0));
    s->linear_drag = IVP_REAL(1.0);
    ivp_vec3r_set(&s->flow_velocity, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
}

IVP_TEST(buoyancy_attacher_matches_all_cores_registration) {
    ivp_environment_t *env1 = make_env(true);
    ivp_environment_t *env2 = make_env(true);
    const ivp_real_t ys[3] = { IVP_REAL(0.2), IVP_REAL(0.0), IVP_REAL(-0.3) };
    ivp_object_t *o1[3], *o2[3];
    for (int i = 0; i < 3; i++) {
        o1[i] = make_ball(env1, IVP_REAL(3.0) * (ivp_real_t)i, ys[i], IVP_REAL(0.0));
        o2[i] = make_ball(env2, IVP_REAL(3.0) * (ivp_real_t)i, ys[i], IVP_REAL(0.0));
    }
    ivp_object_t *outsider1 = make_ball(env1, IVP_REAL(20.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *outsider2 = make_ball(env2, IVP_REAL(20.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* libivp all-cores registration with a static object list */
    ivp_buoyancy_state_t s1;
    buoy_template(&s1);
    ivp_buoyancy_set_objects(&s1, o1, 3);
    ivp_environment_add_controller(env1, IVP_CP_FORCEFIELDS, ivp_controller_buoyancy, &s1);

    /* reference IVP_Attacher_To_Cores_Buoyancy */
    ivp_buoyancy_state_t templ;
    buoy_template(&templ);
    ivp_set_active_t set;
    ivp_set_active_init(&set, NULL, 16);
    for (int i = 0; i < 3; i++) ivp_set_active_add(&set, o2[i]->physical_core);
    ivp_buoyancy_attacher_t *ba = ivp_buoyancy_attacher_create(env2, &templ, &set);
    IVP_ASSERT_NOT_NULL(ba);
    IVP_ASSERT_EQ(ivp_attacher_to_cores_count(&ba->attacher), 3);
    IVP_ASSERT_NOT_NULL(ivp_buoyancy_attacher_get_core_data(ba, o2[1]->physical_core));
    IVP_ASSERT_NULL(ivp_buoyancy_attacher_get_core_data(ba, outsider2->physical_core));

    step(env1, 90);
    step(env2, 90);
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) {
            IVP_ASSERT_TRUE(o1[i]->physical_core->m_world_f_core_last_psi.pos.k[k] ==
                            o2[i]->physical_core->m_world_f_core_last_psi.pos.k[k]);
            IVP_ASSERT_TRUE(o1[i]->physical_core->speed.k[k] == o2[i]->physical_core->speed.k[k]);
        }
    }
    /* the outsider falls freely in both */
    IVP_ASSERT_TRUE(outsider1->physical_core->speed.k[1] == outsider2->physical_core->speed.k[1]);
    IVP_ASSERT_TRUE(outsider1->physical_core->speed.k[1] > o1[0]->physical_core->speed.k[1]);

    /* removing a core from the set stops its buoyancy */
    ivp_set_active_remove(&set, o2[0]->physical_core);
    IVP_ASSERT_EQ(ivp_attacher_to_cores_count(&ba->attacher), 2);
    ivp_buoyancy_attacher_destroy(ba);
    IVP_ASSERT_EQ(set.n_listeners, 0);
    ivp_set_active_destroy(&set);
    ivp_buoyancy_state_destroy(&s1);
    ivp_environment_destroy(env1);
    ivp_environment_destroy(env2);
}

static int g_buoy_deleted;
static void buoy_deleted_cb(void *ctx, ivp_buoyancy_attacher_t *a)
{
    (void)ctx;
    (void)a;
    g_buoy_deleted++;
}

IVP_TEST(buoyancy_attacher_deleted_with_set) {
    ivp_environment_t *env = make_env(true);
    ivp_object_t *o = make_ball(env, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_buoyancy_state_t templ;
    buoy_template(&templ);
    ivp_set_active_t set;
    ivp_set_active_init(&set, NULL, 16);
    ivp_set_active_add(&set, o->physical_core);
    ivp_buoyancy_attacher_t *ba = ivp_buoyancy_attacher_create(env, &templ, &set);
    IVP_ASSERT_NOT_NULL(ba);
    g_buoy_deleted = 0;
    ivp_buoyancy_attacher_set_deleted_listener(ba, buoy_deleted_cb, NULL);
    step(env, 2);
    ivp_set_active_destroy(&set);
    IVP_ASSERT_EQ(g_buoy_deleted, 1);
    /* free fall afterwards */
    ivp_real_t v0 = o->physical_core->speed.k[1];
    step(env, 1);
    IVP_ASSERT_NEAR(o->physical_core->speed.k[1] - v0, IVP_REAL(9.81) * DT60, LTOL);

    /* environment deleted before the attacher */
    ivp_set_active_init(&set, NULL, 16);
    ivp_set_active_add(&set, o->physical_core);
    ba = ivp_buoyancy_attacher_create(env, &templ, &set);
    IVP_ASSERT_NOT_NULL(ba);
    ivp_environment_destroy(env);
    IVP_ASSERT_EQ(ivp_attacher_to_cores_count(&ba->attacher), 0);
    ivp_buoyancy_attacher_destroy(ba);
    ivp_set_active_destroy(&set);
}

#define MANY_CORES 80

IVP_TEST(buoyancy_more_than_64_cores) {
    ivp_environment_t *env = make_env(true);
    ivp_object_t *objs[MANY_CORES];
    for (int i = 0; i < MANY_CORES; i++) {
        objs[i] = make_ball(env, IVP_REAL(3.0) * (ivp_real_t)(i % 10), IVP_REAL(2.0),
                            IVP_REAL(3.0) * (ivp_real_t)(i / 10));
    }
    ivp_buoyancy_state_t s;
    buoy_template(&s);
    ivp_vec3r_set_zero(&s.flow_velocity);
    ivp_environment_add_controller(env, IVP_CP_FORCEFIELDS, ivp_controller_buoyancy, &s);
    step(env, 1);
    IVP_ASSERT_EQ(s.core_count, MANY_CORES);
    const ivp_real_t free_fall = IVP_REAL(9.81) * DT60;
    for (int i = 0; i < MANY_CORES; i++) {
        /* buoyancy of 0.52 m^3 at density 1 nearly halves the fall */
        IVP_ASSERT_TRUE(objs[i]->physical_core->speed.k[1] < IVP_REAL(0.6) * free_fall);
        IVP_ASSERT_NEAR(objs[i]->physical_core->speed.k[1], objs[0]->physical_core->speed.k[1], LTOL);
    }

    /* a deleted core's entry is reused by a new core */
    ivp_environment_destroy_object(env, objs[70]);
    ivp_object_t *n = make_ball(env, IVP_REAL(50.0), IVP_REAL(2.0), IVP_REAL(50.0));
    step(env, 1);
    IVP_ASSERT_EQ(s.core_count, MANY_CORES);
    IVP_ASSERT_TRUE(n->physical_core->speed.k[1] < IVP_REAL(0.6) * free_fall);
    ivp_buoyancy_state_destroy(&s);
    IVP_ASSERT_EQ(s.core_count, 0);
    IVP_ASSERT_NULL(s.cores_overflow);
    ivp_environment_destroy(env);
}

IVP_TEST(buoyancy_attacher_more_than_64_cores) {
    ivp_environment_t *env = make_env(true);
    ivp_set_active_t set;
    ivp_set_active_init(&set, NULL, 16);
    ivp_object_t *objs[MANY_CORES];
    for (int i = 0; i < MANY_CORES; i++) {
        objs[i] = make_ball(env, IVP_REAL(3.0) * (ivp_real_t)(i % 10), IVP_REAL(2.0),
                            IVP_REAL(3.0) * (ivp_real_t)(i / 10));
        ivp_set_active_add(&set, objs[i]->physical_core);
    }
    ivp_buoyancy_state_t templ;
    buoy_template(&templ);
    ivp_buoyancy_attacher_t *ba = ivp_buoyancy_attacher_create(env, &templ, &set);
    IVP_ASSERT_NOT_NULL(ba);
    IVP_ASSERT_EQ(ivp_attacher_to_cores_count(&ba->attacher), MANY_CORES);
    step(env, 1);
    const ivp_real_t free_fall = IVP_REAL(9.81) * DT60;
    for (int i = 0; i < MANY_CORES; i++) {
        IVP_ASSERT_TRUE(objs[i]->physical_core->speed.k[1] < IVP_REAL(0.6) * free_fall);
    }
    ivp_set_active_destroy(&set); /* deletes the attacher */
    ivp_environment_destroy(env);
}

int main(void) {
    IVP_REGISTER(attacher_follows_set);
    IVP_REGISTER(attacher_destroy_and_environment_deletion);
    IVP_REGISTER(attacher_allows_core_without_attachment);
    IVP_REGISTER(attacher_object_filter_installs_created_objects);
    IVP_REGISTER(forcefield_set_driven_acts_on_set_cores_only);
    IVP_REGISTER(forcefield_empty_set_attaches_nothing);
    IVP_REGISTER(forcefield_core_deletion_removes_core_from_set);
    IVP_REGISTER(forcefield_environment_deleted_first);
    IVP_REGISTER(buoyancy_attacher_matches_all_cores_registration);
    IVP_REGISTER(buoyancy_attacher_deleted_with_set);
    IVP_REGISTER(buoyancy_more_than_64_cores);
    IVP_REGISTER(buoyancy_attacher_more_than_64_cores);
    IVP_RUN_ALL();
}
