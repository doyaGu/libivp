/* test_3d_solver.c — Tests for CCD 3D solver and matrix cache */
#include "ivp_test.h"
#include <ivp/collision/ivp_3d_solver.h>
#include <ivp/collision/ivp_matrix_cache.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>

static ivp_environment_t *make_env(void) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

static ivp_object_t *make_ball_at(ivp_environment_t *env, ivp_real_t x) {
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.position, x, IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create_object(env, &tmpl, NULL);
}

/* ── 3D Solver tests ──────────────────────────────────────────────────── */

IVP_TEST(solver_init) {
    ivp_3d_solver_t solver;
    ivp_3d_solver_init(&solver, ivp_3d_dist_point_point, NULL, IVP_REAL(0.01));
    IVP_ASSERT_NOT_NULL(solver.distance_fn);
    IVP_ASSERT_NEAR(solver.max_deviation, 0.01, 1e-6);
    IVP_ASSERT_TRUE(solver.inv_max_deviation > IVP_REAL(0.0));
}

IVP_TEST(solver_null_safety) {
    /* Null pointers should return false, not crash */
    ivp_time_t t_out = 0;
    bool found = ivp_3d_solver_find_first_t(NULL, IVP_REAL(1.0),
        0.0, 1.0, NULL, NULL, &t_out);
    IVP_ASSERT_FALSE(found);
}

IVP_TEST(solver_no_crossing) {
    /* Two stationary objects far apart: no threshold crossing */
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball_at(env, IVP_REAL(0.0));
    ivp_object_t *b = make_ball_at(env, IVP_REAL(100.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_matrix_cache_t cache_a, cache_b;
    ivp_matrix_cache_init(&cache_a, a, 0.0, 1.0 / 60.0);
    ivp_matrix_cache_init(&cache_b, b, 0.0, 1.0 / 60.0);

    ivp_3d_solver_t solver;
    ivp_3d_solver_init(&solver, ivp_3d_dist_point_point, NULL, IVP_REAL(0.01));

    ivp_time_t t_out = 0;
    bool found = ivp_3d_solver_find_first_t(&solver, IVP_REAL(1.0),
        0.0, 1.0 / 60.0, &cache_a, &cache_b, &t_out);
    IVP_ASSERT_FALSE(found);

    ivp_environment_destroy(env);
}

IVP_TEST(solver_point_point_dist) {
    /* Test built-in point-point distance function */
    ivp_transform_r_t ta, tb;
    memset(&ta, 0, sizeof(ta));
    memset(&tb, 0, sizeof(tb));
    ivp_vec3r_set(&ta.pos, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&tb.pos, IVP_REAL(3.0), IVP_REAL(4.0), IVP_REAL(0.0));

    ivp_real_t d = ivp_3d_dist_point_point(&ta, &tb, NULL);
    IVP_ASSERT_NEAR(d, 5.0, 1e-4);
}

IVP_TEST(solver_ball_poly_dist) {
    /* Test ball-poly distance: center distance minus combined radii */
    ivp_transform_r_t ta, tb;
    memset(&ta, 0, sizeof(ta));
    memset(&tb, 0, sizeof(tb));
    ivp_vec3r_set(&ta.pos, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&tb.pos, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_ball_poly_data_t data;
    data.radius_a = IVP_REAL(2.0);
    data.radius_b = IVP_REAL(3.0);

    ivp_real_t d = ivp_3d_dist_ball_poly(&ta, &tb, &data);
    /* distance = 10 - 2 - 3 = 5 */
    IVP_ASSERT_NEAR(d, 5.0, 1e-4);
}

/* ── Matrix cache tests ───────────────────────────────────────────────── */

IVP_TEST(matrix_cache_init_step0) {
    /* Step 0 should be computed immediately (current transform) */
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *obj = make_ball_at(env, IVP_REAL(5.0));
    IVP_ASSERT_NOT_NULL(obj);

    ivp_matrix_cache_t cache;
    ivp_matrix_cache_init(&cache, obj, 0.0, 1.0 / 60.0);

    IVP_ASSERT_TRUE(cache.computed[0]);
    IVP_ASSERT_FALSE(cache.computed[1]);

    const ivp_transform_r_t *t0 = ivp_matrix_cache_get(&cache, 0);
    IVP_ASSERT_NOT_NULL(t0);
    /* Position at step 0 should match last PSI position */
    IVP_ASSERT_NEAR(t0->pos.k[0], obj->core.m_world_f_core_last_psi.pos.k[0], 1e-4);

    ivp_environment_destroy(env);
}

IVP_TEST(matrix_cache_lazy_compute) {
    /* Accessing step N should lazily compute and cache */
    ivp_environment_t *env = make_env();
    ivp_object_t *obj = make_ball_at(env, IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(obj);

    ivp_matrix_cache_t cache;
    ivp_matrix_cache_init(&cache, obj, 0.0, 1.0 / 60.0);

    IVP_ASSERT_FALSE(cache.computed[10]);
    const ivp_transform_r_t *t10 = ivp_matrix_cache_get(&cache, 10);
    IVP_ASSERT_NOT_NULL(t10);
    IVP_ASSERT_TRUE(cache.computed[10]);

    /* Second call should return same pointer (cached) */
    const ivp_transform_r_t *t10b = ivp_matrix_cache_get(&cache, 10);
    IVP_ASSERT_TRUE(t10 == t10b);

    ivp_environment_destroy(env);
}

IVP_TEST(matrix_cache_clamp_index) {
    /* Out-of-range indices should be clamped */
    ivp_environment_t *env = make_env();
    ivp_object_t *obj = make_ball_at(env, IVP_REAL(0.0));

    ivp_matrix_cache_t cache;
    ivp_matrix_cache_init(&cache, obj, 0.0, 1.0 / 60.0);

    /* Negative index clamped to 0 */
    const ivp_transform_r_t *tn = ivp_matrix_cache_get(&cache, -5);
    IVP_ASSERT_NOT_NULL(tn);

    /* Over max clamped to MAX_STEPS */
    const ivp_transform_r_t *tm = ivp_matrix_cache_get(&cache, 999);
    IVP_ASSERT_NOT_NULL(tm);

    ivp_environment_destroy(env);
}

IVP_TEST(solver_max_steps_constant) {
    IVP_ASSERT_EQ(IVP_3D_SOLVER_MAX_STEPS, 20);
}

/* ── Registration ──────────────────────────────────────────────────────── */

int main(void) {
    IVP_REGISTER(solver_init);
    IVP_REGISTER(solver_null_safety);
    IVP_REGISTER(solver_no_crossing);
    IVP_REGISTER(solver_point_point_dist);
    IVP_REGISTER(solver_ball_poly_dist);
    IVP_REGISTER(matrix_cache_init_step0);
    IVP_REGISTER(matrix_cache_lazy_compute);
    IVP_REGISTER(matrix_cache_clamp_index);
    IVP_REGISTER(solver_max_steps_constant);
    IVP_RUN_ALL();
}
