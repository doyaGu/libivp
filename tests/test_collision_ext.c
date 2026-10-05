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

typedef struct ov_fail_allocator {
    ivp_allocator_t allocator;
    const ivp_allocator_t *inner;
    int attempts;
    int fail_at;
    size_t live_bytes;
} ov_fail_allocator_t;

static void *ov_fail_alloc(size_t size, void *ctx)
{
    ov_fail_allocator_t *state = (ov_fail_allocator_t *)ctx;
    state->attempts++;
    if (state->attempts == state->fail_at) return NULL;
    void *ptr = ivp_alloc(state->inner, size);
    if (ptr) state->live_bytes += size;
    return ptr;
}

static void *ov_fail_realloc(void *ptr, size_t old_size, size_t new_size, void *ctx)
{
    ov_fail_allocator_t *state = (ov_fail_allocator_t *)ctx;
    state->attempts++;
    if (state->attempts == state->fail_at) return NULL;
    void *new_ptr = ivp_realloc(state->inner, ptr, old_size, new_size);
    if (new_ptr) state->live_bytes = state->live_bytes - old_size + new_size;
    return new_ptr;
}

static void ov_fail_free(void *ptr, size_t size, void *ctx)
{
    ov_fail_allocator_t *state = (ov_fail_allocator_t *)ctx;
    if (!ptr) return;
    ivp_free(state->inner, ptr, size);
    state->live_bytes -= size;
}

static void ov_fail_allocator_init(ov_fail_allocator_t *state)
{
    memset(state, 0, sizeof(*state));
    state->inner = ivp_default_allocator();
    state->fail_at = INT_MAX;
    state->allocator = (ivp_allocator_t){
        .alloc = ov_fail_alloc,
        .realloc = ov_fail_realloc,
        .free = ov_fail_free,
        .ctx = state
    };
}

static void ov_tree_far_insert_attempts(int fail_at, int *attempts_out,
                                        int *_fail_count)
{
    ov_fail_allocator_t state;
    ov_fail_allocator_init(&state);
    ivp_ov_tree_t tree;
    IVP_ASSERT_EQ(ivp_ov_tree_init_with_allocator(&tree, &state.allocator), IVP_OK);

    ivp_ov_element_t base;
    ivp_ov_element_t target;
    memset(&base, 0, sizeof(base));
    memset(&target, 0, sizeof(target));
    target.center.k[0] = 4096.0f;
    IVP_ASSERT_EQ(ivp_ov_tree_insert_ov_element_ex(
                      &tree, &base, 0.25, 0.25, NULL, NULL), IVP_OK);

    state.attempts = 0;
    state.fail_at = fail_at;
    ivp_result_t result = ivp_ov_tree_insert_ov_element_ex(
        &tree, &target, 0.25, 0.25, NULL, NULL);
    *attempts_out = state.attempts;
    if (fail_at == INT_MAX) {
        IVP_ASSERT_EQ(result, IVP_OK);
    } else {
        IVP_ASSERT_EQ(result, IVP_ERR_OUT_OF_MEMORY);
        IVP_ASSERT_NULL(target.node);
        IVP_ASSERT_NOT_NULL(base.node);
        state.fail_at = INT_MAX;
        IVP_ASSERT_EQ(ivp_ov_tree_insert_ov_element_ex(
                          &tree, &target, 0.25, 0.25, NULL, NULL), IVP_OK);
    }

    ivp_ov_tree_remove_ov_element(&tree, &target);
    ivp_ov_tree_remove_ov_element(&tree, &base);
    ivp_ov_tree_destroy(&tree);
    IVP_ASSERT_EQ(state.live_bytes, 0u);
}

IVP_TEST(ov_tree_insertion_failures_are_recoverable) {
    int allocation_points = 0;
    ov_tree_far_insert_attempts(INT_MAX, &allocation_points, _fail_count);
    IVP_ASSERT_TRUE(allocation_points > 2);
    for (int fail_at = 1; fail_at <= allocation_points; fail_at++) {
        int attempts = 0;
        ov_tree_far_insert_attempts(fail_at, &attempts, _fail_count);
    }

    ov_fail_allocator_t state;
    ov_fail_allocator_init(&state);
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
    state.attempts = 0;
    state.fail_at = 1;
    IVP_ASSERT_EQ(ivp_ov_tree_insert_ov_element_ex(
                      &tree, &target, 1.0, 1.0, &partners, NULL),
                  IVP_ERR_OUT_OF_MEMORY);
    IVP_ASSERT_NULL(target.node);
    IVP_ASSERT_EQ(ivp_pvec_len(&partners), 0);
    state.fail_at = INT_MAX;
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

    IVP_RUN_ALL();
}
