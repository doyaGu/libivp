#include "ivp_test.h"
#include <string.h>
#include <ivp/collision/ivp_range_manager.h>
#include <ivp/collision/ivp_collision_filter.h>
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/collision/ivp_compact_ledge_solver.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_surman.h>
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include "ivp_test_fault_allocator.h"
#include "dynamics/ivp_env_internal.h"
#include "solver/ivp_friction_internal.h"

#ifdef IVP_REAL_DOUBLE
  #define TOL 1e-6
#else
  #define TOL 1e-3f
#endif

/* ========================================================================
 * Range Manager Tests
 * ======================================================================== */

IVP_TEST(range_manager_init_defaults) {
    ivp_range_manager_t rm;
    ivp_real_t dt = IVP_REAL(1.0) / IVP_REAL(60.0);

    ivp_range_manager_init(&rm, dt);

    /* Verify parameters are set to reasonable defaults */
    IVP_ASSERT_TRUE(rm.look_ahead_min_distance_intra > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(rm.look_ahead_max_distance_intra > rm.look_ahead_min_distance_intra);
    IVP_ASSERT_TRUE(rm.look_ahead_time_intra > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(rm.look_ahead_time_world > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(rm.delta_psi_time == dt);
}

IVP_TEST(range_manager_intra_stationary) {
    ivp_range_manager_t rm;
    ivp_real_t dt = IVP_REAL(1.0) / IVP_REAL(60.0);
    ivp_range_manager_init(&rm, dt);

    /* Create two stationary cores */
    ivp_core_t core0, core1;
    ivp_vec3r_t inertia = ivp_vec3r(IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    ivp_core_init(&core0, IVP_REAL(1.0), &inertia);
    ivp_core_init(&core1, IVP_REAL(1.0), &inertia);
    core0.upper_limit_radius = IVP_REAL(0.5);
    core1.upper_limit_radius = IVP_REAL(0.5);

    /* Both cores are stationary (speed = 0 from init) */
    ivp_real_t range0, range1;
    ivp_range_manager_get_coll_range_intra(&rm, &core0, &core1, &range0, &range1);

    /* For stationary objects, total range should be >= minimum distance */
    IVP_ASSERT_TRUE(range0 > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(range1 > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(range0 + range1 >= rm.look_ahead_min_distance_intra - TOL);
}

IVP_TEST(range_manager_intra_fast) {
    ivp_range_manager_t rm;
    ivp_real_t dt = IVP_REAL(1.0) / IVP_REAL(60.0);
    ivp_range_manager_init(&rm, dt);

    /* Create two cores, one moving fast */
    ivp_core_t core0, core1;
    ivp_vec3r_t inertia = ivp_vec3r(IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    ivp_core_init(&core0, IVP_REAL(1.0), &inertia);
    ivp_core_init(&core1, IVP_REAL(1.0), &inertia);
    core0.upper_limit_radius = IVP_REAL(0.5);
    core1.upper_limit_radius = IVP_REAL(0.5);

    /* Get stationary ranges first */
    ivp_real_t range0_still, range1_still;
    ivp_range_manager_get_coll_range_intra(&rm, &core0, &core1, &range0_still, &range1_still);

    /* Set core0 to move fast */
    core0.speed.k[0] = IVP_REAL(100.0);
    core0.current_speed = IVP_REAL(100.0);

    /* Get ranges with fast-moving object */
    ivp_real_t range0_fast, range1_fast;
    ivp_range_manager_get_coll_range_intra(&rm, &core0, &core1, &range0_fast, &range1_fast);

    /* Fast-moving object pair should have larger total range */
    ivp_real_t total_still = range0_still + range1_still;
    ivp_real_t total_fast  = range0_fast  + range1_fast;
    IVP_ASSERT_TRUE(total_fast > total_still + TOL);
    IVP_ASSERT_TRUE(range0_fast > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(range1_fast > IVP_REAL(0.0));
}

IVP_TEST(range_manager_world) {
    ivp_range_manager_t rm;
    ivp_real_t dt = IVP_REAL(1.0) / IVP_REAL(60.0);
    ivp_range_manager_init(&rm, dt);

    /* Create single stationary core */
    ivp_core_t core0;
    ivp_vec3r_t inertia = ivp_vec3r(IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    ivp_core_init(&core0, IVP_REAL(1.0), &inertia);
    core0.upper_limit_radius = IVP_REAL(0.5);

    ivp_real_t range = ivp_range_manager_get_coll_range_world(&rm, &core0);

    /* World range should be positive */
    IVP_ASSERT_TRUE(range > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(range >= rm.look_ahead_min_distance_world - TOL);
}

/* ========================================================================
 * Collision Filter Tests
 * ======================================================================== */

IVP_TEST(collision_filter_allow_all) {
    ivp_collision_filter_t filter = ivp_collision_filter_allow_all();

    /* Should allow collision between any objects, even NULL */
    bool result = ivp_collision_filter_check(&filter, NULL, NULL);
    IVP_ASSERT_TRUE(result == true);
}

static bool always_true(const ivp_object_t *a, const ivp_object_t *b, void *ud) {
    (void)a; (void)b; (void)ud;
    return true;
}

IVP_TEST(collision_filter_callback_true) {
    ivp_collision_filter_t filter = ivp_collision_filter_callback(always_true, NULL);

    bool result = ivp_collision_filter_check(&filter, NULL, NULL);
    IVP_ASSERT_TRUE(result == true);
}

static bool always_false(const ivp_object_t *a, const ivp_object_t *b, void *ud) {
    (void)a; (void)b; (void)ud;
    return false;
}

IVP_TEST(collision_filter_callback_false) {
    ivp_collision_filter_t filter = ivp_collision_filter_callback(always_false, NULL);

    bool result = ivp_collision_filter_check(&filter, NULL, NULL);
    IVP_ASSERT_TRUE(result == false);
}

IVP_TEST(collision_filter_no_collision_group) {
    /* Create environment */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    /* Create two objects with the same no_collision_group */
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    t.rot_inertia_is_factor = false; /* absolute inertia */
    t.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&t.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&t.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    t.orientation = ivp_quatr_identity();
    t.no_collision_group = "groupA";

    ivp_object_t *obj1 = ivp_environment_create_object(env, &t, NULL);
    IVP_ASSERT_TRUE(obj1 != NULL);

    /* Create second object in same group, different position */
    ivp_vec3r_set(&t.position, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj2 = ivp_environment_create_object(env, &t, NULL);
    IVP_ASSERT_TRUE(obj2 != NULL);

    /* Verify objects have the same no_collision_group */
    IVP_ASSERT_TRUE(obj1->no_collision_group != NULL);
    IVP_ASSERT_TRUE(obj2->no_collision_group != NULL);
    IVP_ASSERT_TRUE(strcmp(obj1->no_collision_group, obj2->no_collision_group) == 0);

    /* Cleanup */
    ivp_environment_destroy(env);
}

/* ========================================================================
 * Surface manager ledge references (IVP_SurfaceManager::add_reference_to_ledge
 * / remove_reference_to_ledge)
 * ======================================================================== */

typedef struct counting_surman {
    ivp_surman_polygon_t poly;           /* must be first: polygon vtable casts */
    ivp_surman_vtable_t  vt;
    const ivp_compact_ledge_t *ledge;    /* the single convex ledge */
    int adds, removes, foreign;
} counting_surman_t;

static void cnt_add(const ivp_surman_t *sm, const ivp_compact_ledge_t *ledge)
{
    counting_surman_t *c = (counting_surman_t *)(uintptr_t)sm;
    if (ledge == c->ledge) c->adds++;
    else c->foreign++;
}

static void cnt_remove(const ivp_surman_t *sm, const ivp_compact_ledge_t *ledge)
{
    counting_surman_t *c = (counting_surman_t *)(uintptr_t)sm;
    if (ledge == c->ledge) c->removes++;
    else c->foreign++;
}

static void counting_surman_init(counting_surman_t *c, const ivp_compact_surface_t *surf)
{
    memset(c, 0, sizeof(*c));
    ivp_surman_polygon_init(&c->poly, surf);
    c->vt = *c->poly.base.vtable;
    c->vt.add_reference_to_ledge = cnt_add;
    c->vt.remove_reference_to_ledge = cnt_remove;
    c->poly.base.vtable = &c->vt;
    c->ledge = ivp_surface_get_single_convex(surf);
}

IVP_TEST(surman_ledge_references_balanced) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    ivp_compact_surface_t *ground_surf = ivp_build_box_surface(NULL, IVP_REAL(5.0), IVP_REAL(0.5), IVP_REAL(5.0));
    ivp_compact_surface_t *box_surf = ivp_build_box_surface(NULL, IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(0.5));
    counting_surman_t ground_sm, box_sm;
    counting_surman_init(&ground_sm, ground_surf);
    counting_surman_init(&box_sm, box_surf);

    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_POLYGON;
    t.physical_unmoveable = true;
    t.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&t.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *ground = ivp_environment_create_object(env, &t, &ground_sm.poly.base);
    IVP_ASSERT_NOT_NULL(ground);
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_POLYGON;
    t.mass = IVP_REAL(1.0);
    t.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&t.position, IVP_REAL(0.0), IVP_REAL(-1.5), IVP_REAL(0.0)); /* y down is +gravity */
    ivp_object_t *box = ivp_environment_create_object(env, &t, &box_sm.poly.base);
    IVP_ASSERT_NOT_NULL(box);

    int max_live = 0;
    for (int i = 0; i < 120; i++) {
        ivp_environment_simulate_one_psi(env);
        IVP_ASSERT_TRUE(box_sm.adds >= box_sm.removes);
        /* every synapse pair uses one ledge of each object */
        IVP_ASSERT_EQ(box_sm.adds - box_sm.removes, ground_sm.adds - ground_sm.removes);
        if (box_sm.adds - box_sm.removes > max_live) max_live = box_sm.adds - box_sm.removes;
    }
    /* the falling / resting box had mindists (and contact points) to the ground */
    IVP_ASSERT_TRUE(max_live > 0);

    /* deleting the box drops every synapse that used its ledge */
    ivp_environment_destroy_object(env, box);
    IVP_ASSERT_EQ(box_sm.adds, box_sm.removes);
    IVP_ASSERT_EQ(ground_sm.adds, ground_sm.removes);
    IVP_ASSERT_EQ(box_sm.foreign, 0);
    IVP_ASSERT_EQ(ground_sm.foreign, 0);

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, box_surf);
    ivp_compact_surface_free(NULL, ground_surf);
}

/* ========================================================================
 * Compact Ledge Solver Tests
 * ======================================================================== */

IVP_TEST(compact_ledge_solver_point_to_ledge) {
    /* Build a simple box surface */
    ivp_compact_surface_t *surf = ivp_build_box_surface(NULL,
                                                         IVP_REAL(1.0),
                                                         IVP_REAL(1.0),
                                                         IVP_REAL(1.0));
    IVP_ASSERT_TRUE(surf != NULL);

    /* Get the single convex ledge from the surface */
    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surf);
    IVP_ASSERT_TRUE(ledge != NULL);

    /* Point outside the box */
    ivp_vec3r_t point = ivp_vec3r(IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Compute squared distance - note: arguments are swapped (ledge first, then point) */
    ivp_real_t qdist = ivp_cls_qdist_point_to_ledge(ledge, &point);

    /* Should be positive and reasonable (point is outside) */
    IVP_ASSERT_TRUE(qdist > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(qdist > IVP_REAL(1.0));

    /* Cleanup */
    ivp_compact_surface_free(NULL, surf);
}

IVP_TEST(compact_ledge_solver_box_box) {
    /* Build first box */
    ivp_compact_surface_t *surf_a = ivp_build_box_surface(NULL,
                                                           IVP_REAL(1.0),
                                                           IVP_REAL(1.0),
                                                           IVP_REAL(1.0));
    IVP_ASSERT_TRUE(surf_a != NULL);

    /* Build second box */
    ivp_compact_surface_t *surf_b = ivp_build_box_surface(NULL,
                                                           IVP_REAL(1.0),
                                                           IVP_REAL(1.0),
                                                           IVP_REAL(1.0));
    IVP_ASSERT_TRUE(surf_b != NULL);

    /* Get ledges */
    const ivp_compact_ledge_t *ledge_a = ivp_surface_get_single_convex(surf_a);
    const ivp_compact_ledge_t *ledge_b = ivp_surface_get_single_convex(surf_b);
    IVP_ASSERT_TRUE(ledge_a != NULL);
    IVP_ASSERT_TRUE(ledge_b != NULL);

    /* Test point-to-ledge distance for each box.
     * Since ivp_qdist_ledge_ledge() does not exist in the API,
     * we test a point from box B against ledge A to verify reasonable distance.
     */

    /* Point at center of second box (offset 5 units in X) */
    ivp_vec3r_t point_b = ivp_vec3r(IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Compute squared distance from point_b to ledge_a */
    ivp_real_t qdist = ivp_cls_qdist_point_to_ledge(ledge_a, &point_b);

    /* Distance from point at (5,0,0) to box centered at origin with half-extent 1
     * should be approximately 4.0 (5.0 - 1.0), so qdist should be around 16.0 */
    IVP_ASSERT_TRUE(qdist > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(qdist > IVP_REAL(9.0));

    /* Cleanup */
    ivp_compact_surface_free(NULL, surf_a);
    ivp_compact_surface_free(NULL, surf_b);
}

IVP_TEST(ov_tree_hash_grows_and_releases_allocator_storage) {
    enum { element_count = 5000 };
    ivp_allocator_t counting = ivp_counting_allocator_create(ivp_default_allocator());
    ivp_ov_tree_t tree;
    IVP_ASSERT_EQ(ivp_ov_tree_init_with_allocator(&tree, &counting), IVP_OK);

    ivp_ov_element_t *elements = (ivp_ov_element_t *)calloc(element_count, sizeof(*elements));
    IVP_ASSERT_NOT_NULL(elements);
    if (!elements) {
        ivp_ov_tree_destroy(&tree);
        ivp_counting_allocator_destroy(&counting);
        return;
    }
    for (int i = 0; i < element_count; i++) {
        elements[i].center.k[0] = (float)(i * 4);
        elements[i].center.k[1] = (float)((i % 17) * 4);
        elements[i].center.k[2] = (float)((i % 31) * 4);
        (void)ivp_ov_tree_insert_ov_element(&tree, &elements[i], 0.25, 0.25, NULL);
        IVP_ASSERT_NOT_NULL(elements[i].node);
    }
    IVP_ASSERT_TRUE(tree.node_count > 4096u);
    IVP_ASSERT_TRUE(tree.hash_size > IVP_OV_HASH_INITIAL_SIZE);

    for (int i = element_count - 1; i >= 0; i--) {
        ivp_ov_tree_remove_ov_element(&tree, &elements[i]);
        IVP_ASSERT_TRUE(elements[i].node == NULL);
    }
    IVP_ASSERT_EQ(tree.node_count, 0u);
    free(elements);
    ivp_ov_tree_destroy(&tree);

    ivp_alloc_stats_t stats;
    ivp_counting_allocator_get_stats(&counting, &stats);
    IVP_ASSERT_EQ(stats.current_bytes, 0);
    ivp_counting_allocator_destroy(&counting);
}

static void ov_tree_far_insert_attempts(int fail_at, int *attempts_out,
                                        int *_fail_count)
{
    ivp_test_fault_allocator_t state;
    ivp_test_fault_allocator_init_tracked(&state, NULL);
    ivp_ov_tree_t tree;
    IVP_ASSERT_EQ(ivp_ov_tree_init_with_allocator(&tree, &state.allocator), IVP_OK);

    ivp_ov_element_t base;
    ivp_ov_element_t target;
    memset(&base, 0, sizeof(base));
    memset(&target, 0, sizeof(target));
    target.center.k[0] = 4096.0f;
    IVP_ASSERT_EQ(ivp_ov_tree_insert_ov_element_ex(
                      &tree, &base, 0.25, 0.25, NULL, NULL), IVP_OK);

    ivp_test_fault_allocator_reset(&state, fail_at);
    ivp_result_t result = ivp_ov_tree_insert_ov_element_ex(
        &tree, &target, 0.25, 0.25, NULL, NULL);
    *attempts_out = state.attempts;
    if (fail_at == INT_MAX) {
        IVP_ASSERT_EQ(result, IVP_OK);
    } else {
        IVP_ASSERT_EQ(result, IVP_ERR_OUT_OF_MEMORY);
        IVP_ASSERT_NULL(target.node);
        IVP_ASSERT_NOT_NULL(base.node);
        ivp_test_fault_allocator_reset(&state, INT_MAX);
        IVP_ASSERT_EQ(ivp_ov_tree_insert_ov_element_ex(
                          &tree, &target, 0.25, 0.25, NULL, NULL), IVP_OK);
    }

    ivp_ov_tree_remove_ov_element(&tree, &target);
    ivp_ov_tree_remove_ov_element(&tree, &base);
    ivp_ov_tree_destroy(&tree);
    IVP_ASSERT_EQ(state.live_bytes, 0u);
}

typedef struct test_collision_delegator {
    ivp_collision_delegator_t base;
    ivp_coll_fvector_t *collisions;
} test_collision_delegator_t;

static void test_collision_deleted(ivp_collision_delegator_t *base,
                                   ivp_collision_t *collision)
{
    test_collision_delegator_t *delegator =
        (test_collision_delegator_t *)base;
    if (ivp_coll_fvector_index_of(delegator->collisions, collision) >= 0) {
        ivp_coll_fvector_remove_allow_resort(delegator->collisions, collision);
    }
}

static const ivp_collision_delegator_vt_t test_collision_delegator_vt = {
    test_collision_deleted, NULL, NULL
};

static void exact_mindist_allocation_attempt(int fail_at, bool expect_collision,
                                             int *attempts_out,
                                             int *_fail_count)
{
    ivp_test_fault_allocator_t state;
    ivp_test_fault_allocator_init_tracked(&state, NULL);
    ivp_environment_config_t config;
    ivp_environment_config_init(&config);
    config.allocator = &state.allocator;
    ivp_environment_t *environment = ivp_environment_create(&config);
    IVP_ASSERT_NOT_NULL(environment);

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(1.0);
    object_template.enable_collision_detection = false;
    object_template.start_in_simulation = false;
    ivp_object_t *object0 = ivp_environment_create_object(
        environment, &object_template, NULL);
    object_template.client_data = (void *)(uintptr_t)1;
    object_template.position.k[0] = IVP_REAL(1.5);
    ivp_object_t *object1 = ivp_environment_create_object(
        environment, &object_template, NULL);
    IVP_ASSERT_TRUE(object0 && object1);

    ivp_coll_fvector_t collisions;
    ivp_coll_fvector_init_with_allocator(&collisions, &state.allocator);
    test_collision_delegator_t delegator = {
        .base = { .dvt = &test_collision_delegator_vt },
        .collisions = &collisions
    };
    ivp_test_fault_allocator_reset(&state, fail_at);
    ivp_mindist_mgr_create_exact_mindists(
        object0, object1, 4.0, &collisions,
        NULL, NULL, NULL, NULL, &delegator.base);
    *attempts_out = state.attempts;
    if (collisions.n != (expect_collision ? 1 : 0)) {
        fprintf(stderr, "unexpected collision count %d at failure point %d (%d attempts)\n",
                collisions.n, fail_at, state.attempts);
        (*_fail_count)++;
    }
    if ((object0->exact_synapses != NULL) != expect_collision ||
        (object1->exact_synapses != NULL) != expect_collision) {
        fprintf(stderr, "published mindist at failure point %d (%d attempts)\n",
                fail_at, state.attempts);
        (*_fail_count)++;
    }

    ivp_test_fault_allocator_reset(&state, INT_MAX);
    ivp_environment_destroy(environment);
    IVP_ASSERT_EQ(collisions.n, 0);
    ivp_coll_fvector_clear(&collisions);
    IVP_ASSERT_EQ(state.live_bytes, 0u);
}

IVP_TEST(exact_mindist_allocation_failures_leave_collision_set_unchanged) {
    /* Ledge vectors, mindist object, pending vector and destination vector. */
    int attempts = 0;
    for (int fail_at = 1; fail_at <= 5; fail_at++) {
        exact_mindist_allocation_attempt(fail_at, false, &attempts, _fail_count);
    }
    exact_mindist_allocation_attempt(INT_MAX, true, &attempts, _fail_count);
    IVP_ASSERT_TRUE(attempts >= 5);
}

static void friction_creation_allocation_attempt(int fail_at, int *attempts_out,
                                                 bool expect_contact,
                                                 int *_fail_count)
{
    ivp_test_fault_allocator_t state;
    ivp_test_fault_allocator_init_tracked(&state, NULL);
    ivp_environment_config_t config;
    ivp_environment_config_init(&config);
    config.allocator = &state.allocator;
    ivp_environment_t *environment = ivp_environment_create(&config);
    IVP_ASSERT_NOT_NULL(environment);

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(1.0);
    object_template.enable_collision_detection = false;
    ivp_object_t *object0 = ivp_environment_create_object(
        environment, &object_template, NULL);
    object_template.client_data = (void *)(uintptr_t)1;
    object_template.position.k[0] = IVP_REAL(1.5);
    object_template.physical_unmoveable = true;
    object_template.start_in_simulation = false;
    ivp_object_t *object1 = ivp_environment_create_object(
        environment, &object_template, NULL);
    IVP_ASSERT_TRUE(object0 && object1);

    ivp_coll_fvector_t collisions;
    ivp_coll_fvector_init_with_allocator(&collisions, &state.allocator);
    test_collision_delegator_t delegator = {
        .base = { .dvt = &test_collision_delegator_vt },
        .collisions = &collisions
    };
    ivp_mindist_mgr_create_exact_mindists(
        object0, object1, 4.0, &collisions,
        NULL, NULL, NULL, NULL, &delegator.base);
    IVP_ASSERT_EQ(collisions.n, 1);

    struct ivp_fr_system *friction_system = NULL;
    bool having_new = false;
    ivp_test_fault_allocator_reset(&state, fail_at);
    ivp_env_sim_mem_begin(environment);
    struct ivp_contact_pt *contact =
        ivp_mindist_try_to_generate_managed_friction(
            (ivp_mindist_t *)collisions.elems[0], &friction_system,
            &having_new, NULL, true);
    ivp_env_sim_mem_end(environment);
    *attempts_out = state.attempts;
    IVP_ASSERT_EQ(contact != NULL, expect_contact);
    IVP_ASSERT_EQ(having_new, expect_contact);
    IVP_ASSERT_EQ(friction_system != NULL, expect_contact);
    if (!expect_contact) {
        IVP_ASSERT_NULL(object0->fr_synapses);
        IVP_ASSERT_NULL(object1->fr_synapses);
        IVP_ASSERT_NULL(object0->physical_core->core_friction_info.for_moveables.moveable_core_friction_info);
    }

    ivp_test_fault_allocator_reset(&state, INT_MAX);
    ivp_environment_destroy(environment);
    IVP_ASSERT_EQ(collisions.n, 0);
    ivp_coll_fvector_clear(&collisions);
    IVP_ASSERT_EQ(state.live_bytes, 0u);
}

IVP_TEST(friction_creation_allocation_failures_are_atomic) {
    int allocation_points = 0;
    friction_creation_allocation_attempt(INT_MAX, &allocation_points, true,
                                         _fail_count);
    IVP_ASSERT_TRUE(allocation_points > 5);
    for (int fail_at = 1; fail_at <= allocation_points; fail_at++) {
        int attempts = 0;
        friction_creation_allocation_attempt(fail_at, &attempts, false,
                                             _fail_count);
        IVP_ASSERT_EQ(attempts, fail_at);
    }
}

static ivp_contact_pt_t *create_test_contact(
    ivp_environment_t *environment, ivp_object_t *object0,
    ivp_object_t *object1, ivp_coll_fvector_t *collisions,
    test_collision_delegator_t *delegator, ivp_fr_system_t **system_out)
{
    ivp_mindist_mgr_create_exact_mindists(
        object0, object1, 4.0, collisions,
        NULL, NULL, NULL, NULL, &delegator->base);
    if (collisions->n != 1) return NULL;
    bool having_new = false;
    ivp_env_sim_mem_begin(environment);
    ivp_contact_pt_t *contact = ivp_mindist_try_to_generate_managed_friction(
        (ivp_mindist_t *)collisions->elems[0], system_out, &having_new,
        NULL, true);
    ivp_env_sim_mem_end(environment);
    return having_new ? contact : NULL;
}

static void friction_fusion_allocation_attempt(int fail_at, bool expect_fusion,
                                               int *attempts_out,
                                               int *_fail_count)
{
    ivp_test_fault_allocator_t state;
    ivp_test_fault_allocator_init_tracked(&state, NULL);
    ivp_environment_config_t config;
    ivp_environment_config_init(&config);
    config.allocator = &state.allocator;
    ivp_environment_t *environment = ivp_environment_create(&config);
    IVP_ASSERT_NOT_NULL(environment);

    ivp_object_t *objects[4] = {0};
    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(1.0);
    object_template.enable_collision_detection = false;
    for (int i = 0; i < 4; i++) {
        object_template.client_data = (void *)(uintptr_t)(i + 1);
        object_template.position.k[0] = (ivp_real_t)((i / 2) * 4 +
            (i % 2) * 1.5);
        object_template.physical_unmoveable = (i % 2) != 0;
        object_template.start_in_simulation = !object_template.physical_unmoveable;
        objects[i] = ivp_environment_create_object(
            environment, &object_template, NULL);
        IVP_ASSERT_NOT_NULL(objects[i]);
    }

    ivp_coll_fvector_t collisions[2];
    test_collision_delegator_t delegators[2];
    for (int i = 0; i < 2; i++) {
        ivp_coll_fvector_init_with_allocator(&collisions[i], &state.allocator);
        delegators[i].base.dvt = &test_collision_delegator_vt;
        delegators[i].collisions = &collisions[i];
    }
    ivp_fr_system_t *systems[2] = {0};
    ivp_contact_pt_t *contacts[2];
    contacts[0] = create_test_contact(environment, objects[0], objects[1],
                                      &collisions[0], &delegators[0],
                                      &systems[0]);
    contacts[1] = create_test_contact(environment, objects[2], objects[3],
                                      &collisions[1], &delegators[1],
                                      &systems[1]);
    IVP_ASSERT_TRUE(contacts[0] && contacts[1]);
    IVP_ASSERT_TRUE(systems[0] && systems[1] && systems[0] != systems[1]);

    int first_core_count = ivp_pvec_len(&systems[0]->cores_of_friction_system);
    int second_core_count = ivp_pvec_len(&systems[1]->cores_of_friction_system);
    ivp_test_fault_allocator_reset(&state, fail_at);
    bool fused = ivp_fr_system_fusion_friction_systems(systems[0], systems[1]);
    *attempts_out = state.attempts;
    IVP_ASSERT_EQ(fused, expect_fusion);
    if (expect_fusion) {
        IVP_ASSERT_TRUE(contacts[0]->l_friction_system == systems[0]);
        IVP_ASSERT_TRUE(contacts[1]->l_friction_system == systems[0]);
        IVP_ASSERT_NOT_NULL(ivp_core_get_friction_info(
            objects[2]->physical_core, systems[0]));
    } else {
        IVP_ASSERT_TRUE(contacts[0]->l_friction_system == systems[0]);
        IVP_ASSERT_TRUE(contacts[1]->l_friction_system == systems[1]);
        IVP_ASSERT_EQ(ivp_pvec_len(&systems[0]->cores_of_friction_system),
                      first_core_count);
        IVP_ASSERT_EQ(ivp_pvec_len(&systems[1]->cores_of_friction_system),
                      second_core_count);
        IVP_ASSERT_NOT_NULL(ivp_core_get_friction_info(
            objects[2]->physical_core, systems[1]));
        IVP_ASSERT_NULL(ivp_core_get_friction_info(
            objects[2]->physical_core, systems[0]));
    }

    ivp_test_fault_allocator_reset(&state, INT_MAX);
    ivp_environment_destroy(environment);
    for (int i = 0; i < 2; i++) {
        IVP_ASSERT_EQ(collisions[i].n, 0);
        ivp_coll_fvector_clear(&collisions[i]);
    }
    IVP_ASSERT_EQ(state.live_bytes, 0u);
}

IVP_TEST(friction_fusion_allocation_failures_are_atomic) {
    int allocation_points = 0;
    friction_fusion_allocation_attempt(INT_MAX, true, &allocation_points,
                                       _fail_count);
    IVP_ASSERT_TRUE(allocation_points > 5);
    for (int fail_at = 1; fail_at <= allocation_points; fail_at++) {
        int attempts = 0;
        friction_fusion_allocation_attempt(fail_at, false, &attempts,
                                           _fail_count);
        IVP_ASSERT_EQ(attempts, fail_at);
    }
}

IVP_TEST(ov_tree_insertion_failures_are_recoverable) {
    int allocation_points = 0;
    ov_tree_far_insert_attempts(INT_MAX, &allocation_points, _fail_count);
    IVP_ASSERT_TRUE(allocation_points > 2);
    for (int fail_at = 1; fail_at <= allocation_points; fail_at++) {
        int attempts = 0;
        ov_tree_far_insert_attempts(fail_at, &attempts, _fail_count);
    }

    ivp_test_fault_allocator_t state;
    ivp_test_fault_allocator_init_tracked(&state, NULL);
    ivp_ov_tree_t tree;
    IVP_ASSERT_EQ(ivp_ov_tree_init_with_allocator(&tree, &state.allocator), IVP_OK);
    ivp_ov_element_t base;
    ivp_ov_element_t target;
    memset(&base, 0, sizeof(base));
    memset(&target, 0, sizeof(target));
    IVP_ASSERT_EQ(ivp_ov_tree_insert_ov_element_ex(
                      &tree, &base, 1.0, 1.0, NULL, NULL), IVP_OK);
    ivp_pvec_t partners;
    ivp_pvec_init_with_allocator(&partners, &state.allocator);
    ivp_test_fault_allocator_reset(&state, 1);
    IVP_ASSERT_EQ(ivp_ov_tree_insert_ov_element_ex(
                      &tree, &target, 1.0, 1.0, &partners, NULL),
                  IVP_ERR_OUT_OF_MEMORY);
    IVP_ASSERT_NULL(target.node);
    IVP_ASSERT_EQ(ivp_pvec_len(&partners), 0);
    ivp_test_fault_allocator_reset(&state, INT_MAX);
    ivp_pvec_clear(&partners);
    ivp_ov_tree_remove_ov_element(&tree, &base);
    ivp_ov_tree_destroy(&tree);
    IVP_ASSERT_EQ(state.live_bytes, 0u);
}

/* ========================================================================
 * Test Registration and Main
 * ======================================================================== */

int main(void) {
    /* Range Manager Tests */
    IVP_REGISTER(range_manager_init_defaults);
    IVP_REGISTER(range_manager_intra_stationary);
    IVP_REGISTER(range_manager_intra_fast);
    IVP_REGISTER(range_manager_world);

    /* Collision Filter Tests */
    IVP_REGISTER(collision_filter_allow_all);
    IVP_REGISTER(collision_filter_callback_true);
    IVP_REGISTER(collision_filter_callback_false);
    IVP_REGISTER(collision_filter_no_collision_group);

    /* Compact Ledge Solver Tests */
    IVP_REGISTER(compact_ledge_solver_point_to_ledge);
    IVP_REGISTER(compact_ledge_solver_box_box);
    IVP_REGISTER(surman_ledge_references_balanced);
    IVP_REGISTER(ov_tree_hash_grows_and_releases_allocator_storage);
    IVP_REGISTER(ov_tree_insertion_failures_are_recoverable);
    IVP_REGISTER(exact_mindist_allocation_failures_leave_collision_set_unchanged);
    IVP_REGISTER(friction_creation_allocation_failures_are_atomic);
    IVP_REGISTER(friction_fusion_allocation_failures_are_atomic);

    IVP_RUN_ALL();
}
