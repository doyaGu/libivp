#include "ivp_test.h"
#include <ivp/dynamics/ivp_object_attach.h>
#include <ivp/dynamics/ivp_merge_core.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_handle.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_result.h>
#include <ivp/dynamics/ivp_cluster.h>
#include <ivp/events/ivp_listener_object.h>
#include <ivp/math/ivp_transform.h>
#include "dynamics/ivp_sim_internal.h"
#include <math.h>
#include <string.h>

#ifdef IVP_REAL_DOUBLE
  #define TOL 1e-6
#else
  #define TOL 1e-3f
#endif

/* Helper: create basic environment with zero gravity */
static ivp_environment_t *create_test_env(void) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

/* Helper: create basic object at position */
static ivp_object_t *create_test_object_ex(ivp_environment_t *env, ivp_real_t x, ivp_real_t y, ivp_real_t z,
                                           ivp_real_t mass, bool start_in_simulation) {
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.ball_radius = IVP_REAL(0.5); /* IVP_Template_Ball::radius */
    t.mass = mass;
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    t.rot_inertia_is_factor = false; /* absolute inertia */
    t.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&t.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&t.position, x, y, z);
    t.orientation = ivp_quatr_identity();
    t.start_in_simulation = start_in_simulation;
    return ivp_environment_create_object(env, &t, NULL);
}

static ivp_object_t *create_test_object(ivp_environment_t *env, ivp_real_t x, ivp_real_t y, ivp_real_t z) {
    return create_test_object_ex(env, x, y, z, IVP_REAL(1.0), true);
}

/* world position of the object origin from the core's PSI matrix */
static void object_origin_psi(const ivp_object_t *o, ivp_vec3r_t *out) {
    const ivp_transform_r_t *m = &o->physical_core->m_world_f_core_last_psi;
    ivp_transform_point(out, m, &o->shift_core_f_object);
}

static void detach_template(ivp_object_template_t *t, ivp_real_t mass) {
    ivp_object_template_init(t);
    t->type = IVP_OBJECT_BALL;
    t->mass = mass;
    ivp_vec3r_set(&t->rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    t->rot_inertia_is_factor = false;
    t->speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&t->rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    t->extra_radius = IVP_REAL(0.5);   /* detach_object: a ball's radius is the template's extra radius */
}

/* ========== IVP_Object_Attach TESTS ========== */

IVP_TEST(attach_object_shares_parent_core) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *parent = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *child = create_test_object(env, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    IVP_ASSERT_TRUE(parent && child);
    ivp_core_t *pcore = parent->physical_core;
    IVP_ASSERT_TRUE(child->physical_core != pcore);

    ivp_object_attach_object(parent, child, IVP_OBJECT_ATTACH_CURRENT_DISTANCE);
    IVP_ASSERT_TRUE(child->physical_core == pcore);
    IVP_ASSERT_TRUE(child->friction_core == pcore);
    IVP_ASSERT_TRUE(child->original_core == pcore);
    IVP_ASSERT_TRUE(ivp_object_get_original_core(child) == pcore);
    IVP_ASSERT_EQ(ivp_pvec_len(&pcore->objects), 2);
    IVP_ASSERT_NEAR(pcore->mass, IVP_REAL(1.0), TOL);   /* the parent's physical values */
    IVP_ASSERT_TRUE(pcore->upper_limit_radius >= IVP_REAL(2.0));

    /* the child keeps its place, now 2 m from the core */
    ivp_vec3r_t p;
    object_origin_psi(child, &p);
    IVP_ASSERT_NEAR(p.k[0], IVP_REAL(2.0), TOL);
    IVP_ASSERT_NEAR(p.k[1], IVP_REAL(0.0), TOL);

    /* both move with the shared core */
    ivp_vec3r_t v = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_async_add_speed_ws(parent, &v);
    for (int i = 0; i < 30; i++) ivp_environment_simulate_one_psi(env);
    ivp_vec3r_t pp, cp;
    object_origin_psi(parent, &pp);
    object_origin_psi(child, &cp);
    IVP_ASSERT_TRUE(pp.k[0] > IVP_REAL(0.1));
    IVP_ASSERT_NEAR(cp.k[0] - pp.k[0], IVP_REAL(2.0), TOL);
    ivp_environment_destroy(env);
}

IVP_TEST(attach_object_reposition) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *parent = create_test_object(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *child = create_test_object(env, IVP_REAL(4.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_attach_object(parent, child, 4.0);
    ivp_quatr_t q = ivp_quatr_identity();
    ivp_vec3r_t shift = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0));
    IVP_ASSERT_EQ(ivp_object_reposition_object_Ros(parent, child, &q, &shift, false), IVP_OK);
    ivp_vec3r_t p;
    object_origin_psi(child, &p);
    IVP_ASSERT_NEAR(p.k[0], IVP_REAL(1.0), TOL);
    IVP_ASSERT_NEAR(p.k[1], IVP_REAL(3.0), TOL);
    IVP_ASSERT_NEAR(p.k[2], IVP_REAL(0.0), TOL);
    ivp_environment_destroy(env);
}

IVP_TEST(detach_object_gets_own_core) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *parent = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *child = create_test_object(env, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_attach_object(parent, child, IVP_OBJECT_ATTACH_CURRENT_DISTANCE);
    ivp_vec3r_t v = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.5));
    ivp_object_async_add_speed_ws(parent, &v);
    for (int i = 0; i < 10; i++) ivp_environment_simulate_one_psi(env);

    ivp_object_template_t t;
    detach_template(&t, IVP_REAL(2.0));
    ivp_object_detach_object(child, &t);
    IVP_ASSERT_TRUE(child->physical_core != parent->physical_core);
    IVP_ASSERT_TRUE(child->original_core == child->physical_core && child->friction_core == child->physical_core);
    IVP_ASSERT_EQ(ivp_pvec_len(&parent->physical_core->objects), 1);
    IVP_ASSERT_EQ(ivp_pvec_len(&child->physical_core->objects), 1);
    IVP_ASSERT_NEAR(child->physical_core->mass, IVP_REAL(2.0), TOL);
    /* the old core was simulated: the detached object keeps moving with the
     * surface speed of the old core */
    IVP_ASSERT_FALSE(ivp_object_is_frozen(child));
    IVP_ASSERT_NEAR(child->physical_core->speed.k[2], IVP_REAL(1.5), TOL);
    ivp_environment_destroy(env);
}

IVP_TEST(detach_object_new_parameters) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *obj = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_core_t *old_core = obj->physical_core;
    (void)old_core;
    ivp_object_template_t t;
    detach_template(&t, IVP_REAL(3.0));
    ivp_object_detach_object(obj, &t);   /* not attached: only new physical parameters */
    IVP_ASSERT_NEAR(obj->physical_core->mass, IVP_REAL(3.0), TOL);
    /* the old core was deleted with its only object: no wake up */
    IVP_ASSERT_TRUE(ivp_object_is_frozen(obj));
    ivp_environment_destroy(env);
}

/* ========== IVP_Environment::merge_objects TESTS ========== */

IVP_TEST(merge_objects_shared_core) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *a = create_test_object_ex(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0), false);
    ivp_object_t *b = create_test_object_ex(env, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(3.0), false);
    ivp_object_t *c = create_test_object_ex(env, IVP_REAL(0.0), IVP_REAL(4.0), IVP_REAL(0.0), IVP_REAL(2.0), false);
    IVP_ASSERT_TRUE(a && b && c);
    ivp_object_t *objs[3] = { a, b, c };
    IVP_ASSERT_TRUE(ivp_environment_merge_objects(env, objs, 3));
    ivp_core_t *core = a->physical_core;
    IVP_ASSERT_TRUE(b->physical_core == core && c->physical_core == core);
    IVP_ASSERT_TRUE(a->original_core == core && a->friction_core == core);
    IVP_ASSERT_EQ(ivp_pvec_len(&core->objects), 3);
    IVP_ASSERT_NEAR(core->mass, IVP_REAL(6.0), TOL);
    /* matrix position: mass center; pos_world_f_core_last_psi: the heaviest
     * core (IVP_Merge_Core::find_main_axis / find_mass_center) */
    IVP_ASSERT_NEAR(core->m_world_f_core_last_psi.pos.k[0], IVP_REAL(1.0), TOL);
    IVP_ASSERT_NEAR(core->m_world_f_core_last_psi.pos.k[1], IVP_REAL(4.0) / IVP_REAL(3.0), TOL);
    IVP_ASSERT_NEAR(core->pos_world_f_core_last_psi.k[0], IVP_REAL(2.0), TOL);
    IVP_ASSERT_NEAR(core->pos_world_f_core_last_psi.k[1], IVP_REAL(0.0), TOL);
    /* every object keeps its place in the merged core's PSI frame */
    ivp_vec3r_t p;
    object_origin_psi(b, &p);
    IVP_ASSERT_NEAR(p.k[0], IVP_REAL(2.0), TOL);
    IVP_ASSERT_NEAR(p.k[1], IVP_REAL(0.0), TOL);
    object_origin_psi(c, &p);
    IVP_ASSERT_NEAR(p.k[0], IVP_REAL(0.0), TOL);
    IVP_ASSERT_NEAR(p.k[1], IVP_REAL(4.0), TOL);
    IVP_ASSERT_TRUE(ivp_object_is_frozen(a) && core->movement_state == IVP_MT_NOT_SIM);

    /* wake the merged body: all objects move together */
    ivp_vec3r_t v = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(2.0));
    ivp_object_async_add_speed_ws(c, &v);
    for (int i = 0; i < 10; i++) ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_FALSE(ivp_object_is_frozen(a));
    IVP_ASSERT_NEAR(core->speed.k[2], IVP_REAL(2.0), TOL);
    ivp_environment_destroy(env);
}

IVP_TEST(merge_objects_rejects_unsafe_input) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *a = create_test_object_ex(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0), false);
    ivp_object_t *b = create_test_object_ex(env, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0), false);
    ivp_object_t *moving = create_test_object(env, IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0));
    ivp_core_t *ca = a->physical_core;
    /* simulated core (the reference loops forever) */
    ivp_object_t *with_moving[2] = { a, moving };
    IVP_ASSERT_FALSE(ivp_environment_merge_objects(env, with_moving, 2));
    IVP_ASSERT_TRUE(a->physical_core == ca && moving->physical_core != ca);
    /* the same core twice (deleted twice by the reference) */
    ivp_object_t *twice[2] = { a, a };
    IVP_ASSERT_FALSE(ivp_environment_merge_objects(env, twice, 2));
    IVP_ASSERT_FALSE(ivp_environment_merge_objects(env, NULL, 0));
    /* an object whose core is shared with an object that is not merged */
    ivp_object_t *ab[2] = { a, b };
    IVP_ASSERT_TRUE(ivp_environment_merge_objects(env, ab, 2));
    ivp_object_t *c = create_test_object_ex(env, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0), false);
    ivp_object_t *ac[2] = { a, c };
    IVP_ASSERT_FALSE(ivp_environment_merge_objects(env, ac, 2));
    IVP_ASSERT_TRUE(a->physical_core == b->physical_core && c->physical_core != a->physical_core);
    ivp_environment_destroy(env);
}

/* Reference 8ad306e: invalid requests are rejected atomically (nothing
 * changes); more than IVP_MAX_CORE_STACK (6) objects merge fine. */
IVP_TEST(merge_objects_rejects_null_and_foreign_objects) {
    ivp_environment_t *env = create_test_env();
    ivp_environment_t *other = create_test_env();
    ivp_object_t *a = create_test_object_ex(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0), false);
    ivp_object_t *b = create_test_object_ex(env, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0), false);
    ivp_object_t *foreign = create_test_object_ex(other, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0), false);
    ivp_core_t *ca = a->physical_core, *cb = b->physical_core;
    ivp_object_t *with_null[3] = { a, NULL, b };
    IVP_ASSERT_FALSE(ivp_environment_merge_objects(env, with_null, 3));
    ivp_object_t *with_foreign[3] = { a, b, foreign };
    IVP_ASSERT_FALSE(ivp_environment_merge_objects(env, with_foreign, 3));
    ivp_object_t *ab[2] = { a, b };
    IVP_ASSERT_FALSE(ivp_environment_merge_objects(env, ab, 0));
    IVP_ASSERT_TRUE(a->physical_core == ca && b->physical_core == cb);
    IVP_ASSERT_EQ(ivp_pvec_len(&ca->objects), 1);
    IVP_ASSERT_EQ(ivp_pvec_len(&cb->objects), 1);
    ivp_environment_destroy(other);
    ivp_environment_destroy(env);
}

IVP_TEST(merge_objects_more_than_six) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *objs[8];
    for (int i = 0; i < 8; i++) {
        objs[i] = create_test_object_ex(env, IVP_REAL(2.0) * (ivp_real_t)i, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0), false);
        IVP_ASSERT_TRUE(objs[i] != NULL);
    }
    IVP_ASSERT_TRUE(ivp_environment_merge_objects(env, objs, 8));
    ivp_core_t *core = objs[0]->physical_core;
    for (int i = 1; i < 8; i++) IVP_ASSERT_TRUE(objs[i]->physical_core == core);
    IVP_ASSERT_EQ(ivp_pvec_len(&core->objects), 8);
    IVP_ASSERT_NEAR(core->mass, IVP_REAL(8.0), TOL);
    IVP_ASSERT_NEAR(core->m_world_f_core_last_psi.pos.k[0], IVP_REAL(7.0), TOL);
    ivp_environment_destroy(env);
}

/* Reference f105953: attach_object(NULL, x), attach_object(x, NULL) and
 * attaching an object to itself return without changes. */
IVP_TEST(attach_object_ignores_null_and_self) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *a = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_core_t *ca = a->physical_core;
    ivp_object_attach_object(NULL, a, IVP_OBJECT_ATTACH_CURRENT_DISTANCE);
    ivp_object_attach_object(a, NULL, IVP_OBJECT_ATTACH_CURRENT_DISTANCE);
    ivp_object_attach_object(a, a, IVP_OBJECT_ATTACH_CURRENT_DISTANCE);
    IVP_ASSERT_TRUE(a->physical_core == ca && a->friction_core == ca && a->original_core == ca);
    IVP_ASSERT_EQ(ivp_pvec_len(&ca->objects), 1);
    for (int i = 0; i < 3; i++) ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_TRUE(a->physical_core == ca);
    ivp_environment_destroy(env);
}

/* Reference 8ca7bb2: IVP_Core_Merged(core0, core1) starts the next-PSI
 * quaternion from q_world_f_core_last_psi before its first matrix is
 * calculated (the core has no objects yet). */
IVP_TEST(merged_core_from_two_cores_has_valid_next_psi_quat) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *a = create_test_object_ex(env, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0), false);
    ivp_object_t *b = create_test_object_ex(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0), false);
    ivp_core_merged_t *mc = ivp_core_merged_create_from_cores(a->physical_core, b->physical_core);
    IVP_ASSERT_TRUE(mc != NULL);
    IVP_ASSERT_TRUE(fabs((double)mc->core.q_world_f_core_next_psi.w) > 0.5);
    IVP_ASSERT_EQ(ivp_pvec_len(&mc->core.objects), 0);
    ivp_core_delete(&mc->core);
    ivp_environment_destroy(env);
}

/* ========== environment teardown order (~IVP_Environment) ========== */

typedef struct deleted_log {
    ivp_object_t *objs[8];
    int n;
} deleted_log_t;

static void log_object_deleted(ivp_listener_object_t *self, ivp_event_object_t *ev) {
    deleted_log_t *log = (deleted_log_t *)self->user_data;
    if (log->n < 8) log->objs[log->n++] = ev->real_object;
}

IVP_TEST(environment_teardown_cluster_order) {
    ivp_environment_t *env = create_test_env();
    ivp_object_t *a = create_test_object(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = create_test_object(env, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *c = create_test_object(env, IVP_REAL(6.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *stat = ivp_environment_get_static_object(env);
    /* root: k{b}, c, a, static */
    ivp_cluster_t *k = ivp_environment_create_cluster(env, NULL, NULL);
    ivp_cluster_add_child(k, b);
    deleted_log_t log;
    memset(&log, 0, sizeof(log));
    ivp_listener_object_t l;
    memset(&l, 0, sizeof(l));
    l.event_object_deleted = log_object_deleted;
    l.user_data = &log;
    ivp_environment_add_listener_object_global(env, &l);
    ivp_environment_destroy(env);
    /* the static object first, then the root cluster's first object until it
     * is empty (the sub-cluster's objects first) */
    IVP_ASSERT_EQ(log.n, 4);
    IVP_ASSERT_TRUE(log.objs[0] == stat);
    IVP_ASSERT_TRUE(log.objs[1] == b);
    IVP_ASSERT_TRUE(log.objs[2] == c);
    IVP_ASSERT_TRUE(log.objs[3] == a);
}



/* ========== HANDLE SYSTEM TESTS (4) ========== */

IVP_TEST(handle_invalid_default) {
    ivp_object_handle_t h = ivp_object_handle_invalid();

    IVP_ASSERT_TRUE(ivp_object_handle_is_valid(h) == false);
    IVP_ASSERT_TRUE(h.slot == UINT32_MAX);
    IVP_ASSERT_TRUE(h.generation == 0);
}

IVP_TEST(handle_is_valid) {
    ivp_environment_t *env = create_test_env();

    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    t.rot_inertia_is_factor = false; /* absolute inertia */
    ivp_vec3r_set(&t.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    t.orientation = ivp_quatr_identity();

    ivp_object_handle_t handle;
    ivp_object_t *obj = ivp_environment_create_object_with_handle(env, &t, NULL, &handle);

    IVP_ASSERT_TRUE(obj != NULL);
    IVP_ASSERT_TRUE(ivp_object_handle_is_valid(handle) == true);

    /* Destroy object */
    ivp_environment_destroy_object(env, obj);

    /* Handle should now be stale/invalid (generation mismatch) */
    ivp_object_t *retrieved = ivp_environment_get_object_by_handle(env, handle);
    IVP_ASSERT_TRUE(retrieved == NULL);

    ivp_environment_destroy(env);
}

IVP_TEST(handle_get_object) {
    ivp_environment_t *env = create_test_env();

    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    t.rot_inertia_is_factor = false; /* absolute inertia */
    ivp_vec3r_set(&t.position, IVP_REAL(5.0), IVP_REAL(3.0), IVP_REAL(2.0));
    t.orientation = ivp_quatr_identity();

    ivp_object_handle_t handle;
    ivp_object_t *obj = ivp_environment_create_object_with_handle(env, &t, NULL, &handle);

    /* Retrieve via handle */
    ivp_object_t *retrieved = ivp_environment_get_object_by_handle(env, handle);

    IVP_ASSERT_TRUE(retrieved == obj);
    IVP_ASSERT_TRUE(ivp_fabs(retrieved->physical_core->pos_world_f_core_last_psi.k[0] - IVP_REAL(5.0)) < TOL);
    IVP_ASSERT_TRUE(ivp_fabs(retrieved->physical_core->pos_world_f_core_last_psi.k[1] - IVP_REAL(3.0)) < TOL);
    IVP_ASSERT_TRUE(ivp_fabs(retrieved->physical_core->pos_world_f_core_last_psi.k[2] - IVP_REAL(2.0)) < TOL);

    ivp_environment_destroy(env);
}

IVP_TEST(handle_destroy_by_handle) {
    ivp_environment_t *env = create_test_env();

    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    t.rot_inertia_is_factor = false; /* absolute inertia */
    ivp_vec3r_set(&t.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    t.orientation = ivp_quatr_identity();

    ivp_object_handle_t handle;
    ivp_environment_create_object_with_handle(env, &t, NULL, &handle);

    IVP_ASSERT_TRUE(ivp_object_handle_is_valid(handle) == true);

    /* Destroy by handle */
    bool success = ivp_environment_destroy_object_by_handle(env, handle);

    IVP_ASSERT_TRUE(success == true);

    /* Object should be gone */
    ivp_object_t *retrieved = ivp_environment_get_object_by_handle(env, handle);
    IVP_ASSERT_TRUE(retrieved == NULL);

    /* Try to destroy again - should fail */
    bool success2 = ivp_environment_destroy_object_by_handle(env, handle);
    IVP_ASSERT_TRUE(success2 == false);

    ivp_environment_destroy(env);
}

/* ========== MAIN ========== */

int main(void) {
    /* IVP_Object_Attach */
    IVP_REGISTER(attach_object_shares_parent_core);
    IVP_REGISTER(attach_object_reposition);
    IVP_REGISTER(detach_object_gets_own_core);
    IVP_REGISTER(detach_object_new_parameters);

    /* IVP_Environment::merge_objects */
    IVP_REGISTER(merge_objects_shared_core);
    IVP_REGISTER(merge_objects_rejects_unsafe_input);
    IVP_REGISTER(merge_objects_rejects_null_and_foreign_objects);
    IVP_REGISTER(merge_objects_more_than_six);
    IVP_REGISTER(attach_object_ignores_null_and_self);
    IVP_REGISTER(merged_core_from_two_cores_has_valid_next_psi_quat);

    /* ~IVP_Environment */
    IVP_REGISTER(environment_teardown_cluster_order);

    /* Handle system tests */
    IVP_REGISTER(handle_invalid_default);
    IVP_REGISTER(handle_is_valid);
    IVP_REGISTER(handle_get_object);
    IVP_REGISTER(handle_destroy_by_handle);

    IVP_RUN_ALL();
}
