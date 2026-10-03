/* test_solver.c - Tests for the solver module public API and solver-level
 * environment behaviour.
 *
 * Test suites:
 *   1. Great matrix solver (Ax=b, determinant, singular)
 *   2. Two-ball scenarios (environment-level impact resolution)
 *   3. Kinetic energy queries
 *   4. Static objects
 *   5. Solver constants
 *
 * The unit tests of the old per-step contact/impact/friction internals
 * (ivp_contact.h, ivp_impact.h, ivp_environment_collision.h) were removed
 * when that pipeline was replaced by the reference event-driven pipeline.
 */
#include "ivp_test.h"

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/solver/ivp_solver_constants.h>
#include <ivp/solver/ivp_great_matrix.h>
#include <ivp/math/ivp_fast_math.h>
#include <string.h>


/* ════════════════════════════════════════════════════════════════════════════
 * Great Matrix Tests
 * ════════════════════════════════════════════════════════════════════════════ */

/* 1x1: trivial case */
IVP_TEST(matrix_1x1) {
    ivp_great_matrix_t m;
    IVP_ASSERT_EQ(ivp_great_matrix_init(&m, 1, NULL), IVP_OK);
    ivp_great_matrix_set(&m, 0, 0, IVP_REAL(2.0));
    ivp_great_matrix_set_rhs(&m, 0, IVP_REAL(6.0));
    ivp_real_t x;
    IVP_ASSERT_EQ(ivp_great_matrix_solve(&m, &x), IVP_OK);
    IVP_ASSERT_NEAR(x, 3.0, 1e-6);
    ivp_great_matrix_destroy(&m, NULL);
}

/* 2x2 system */
IVP_TEST(matrix_2x2) {
    ivp_great_matrix_t m;
    IVP_ASSERT_EQ(ivp_great_matrix_init(&m, 2, NULL), IVP_OK);
    /* [2 1 | 5]
     * [1 3 | 7]  �?x=8/5=1.6, y=9/5=1.8 */
    ivp_great_matrix_set(&m, 0, 0, IVP_REAL(2.0));
    ivp_great_matrix_set(&m, 0, 1, IVP_REAL(1.0));
    ivp_great_matrix_set_rhs(&m, 0, IVP_REAL(5.0));
    ivp_great_matrix_set(&m, 1, 0, IVP_REAL(1.0));
    ivp_great_matrix_set(&m, 1, 1, IVP_REAL(3.0));
    ivp_great_matrix_set_rhs(&m, 1, IVP_REAL(7.0));
    ivp_real_t x[2];
    IVP_ASSERT_EQ(ivp_great_matrix_solve(&m, x), IVP_OK);
    IVP_ASSERT_NEAR(x[0], 1.6, 1e-6);
    IVP_ASSERT_NEAR(x[1], 1.8, 1e-6);
    ivp_great_matrix_destroy(&m, NULL);
}

/* 3x3 system with pivoting */
IVP_TEST(matrix_3x3_pivot) {
    ivp_great_matrix_t m;
    IVP_ASSERT_EQ(ivp_great_matrix_init(&m, 3, NULL), IVP_OK);
    /* [0  1  1 | 6]
     * [3  2  1 | 11]
     * [1  1  1 | 7]   �?x=1, y=2, z=4
     * Requires pivoting: column 0 row 0 is 0, pivot to row 1. */
    ivp_great_matrix_set(&m, 0, 0, IVP_REAL(0.0));
    ivp_great_matrix_set(&m, 0, 1, IVP_REAL(1.0));
    ivp_great_matrix_set(&m, 0, 2, IVP_REAL(1.0));
    ivp_great_matrix_set_rhs(&m, 0, IVP_REAL(6.0));
    ivp_great_matrix_set(&m, 1, 0, IVP_REAL(3.0));
    ivp_great_matrix_set(&m, 1, 1, IVP_REAL(2.0));
    ivp_great_matrix_set(&m, 1, 2, IVP_REAL(1.0));
    ivp_great_matrix_set_rhs(&m, 1, IVP_REAL(11.0));
    ivp_great_matrix_set(&m, 2, 0, IVP_REAL(1.0));
    ivp_great_matrix_set(&m, 2, 1, IVP_REAL(1.0));
    ivp_great_matrix_set(&m, 2, 2, IVP_REAL(1.0));
    ivp_great_matrix_set_rhs(&m, 2, IVP_REAL(7.0));
    ivp_real_t x[3];
    IVP_ASSERT_EQ(ivp_great_matrix_solve(&m, x), IVP_OK);
    IVP_ASSERT_NEAR(x[0], 1.0, 1e-5);
    IVP_ASSERT_NEAR(x[1], 2.0, 1e-5);
    IVP_ASSERT_NEAR(x[2], 4.0, 1e-5);
    ivp_great_matrix_destroy(&m, NULL);
}

/* Singular matrix �?error */
IVP_TEST(matrix_singular) {
    ivp_great_matrix_t m;
    IVP_ASSERT_EQ(ivp_great_matrix_init(&m, 2, NULL), IVP_OK);
    /* Rank-deficient system: row 2 = 2 × row 1. Consistent RHS (6 = 2×3).
     * Reference alignment: solver sets degenerate row's x=0 instead of failing.
     * This matches ivp_great_matrix.cxx solve_lower_null_matrix behavior. */
    ivp_great_matrix_set(&m, 0, 0, IVP_REAL(1.0));
    ivp_great_matrix_set(&m, 0, 1, IVP_REAL(2.0));
    ivp_great_matrix_set_rhs(&m, 0, IVP_REAL(3.0));
    ivp_great_matrix_set(&m, 1, 0, IVP_REAL(2.0));
    ivp_great_matrix_set(&m, 1, 1, IVP_REAL(4.0));
    ivp_great_matrix_set_rhs(&m, 1, IVP_REAL(6.0));
    ivp_real_t x[2];
    ivp_result_t res = ivp_great_matrix_solve(&m, x);
    IVP_ASSERT_EQ(res, IVP_OK);
    /* Solution should satisfy both equations */
    ivp_real_t r0 = IVP_REAL(1.0)*x[0] + IVP_REAL(2.0)*x[1] - IVP_REAL(3.0);
    ivp_real_t r1 = IVP_REAL(2.0)*x[0] + IVP_REAL(4.0)*x[1] - IVP_REAL(6.0);
    IVP_ASSERT_NEAR(r0, IVP_REAL(0.0), IVP_REAL(1e-3));
    IVP_ASSERT_NEAR(r1, IVP_REAL(0.0), IVP_REAL(1e-3));

    /* Test truly inconsistent: row 2 = 2 × row 1, but RHS inconsistent */
    ivp_great_matrix_t m2;
    IVP_ASSERT_EQ(ivp_great_matrix_init(&m2, 2, NULL), IVP_OK);
    ivp_great_matrix_set(&m2, 0, 0, IVP_REAL(1.0));
    ivp_great_matrix_set(&m2, 0, 1, IVP_REAL(2.0));
    ivp_great_matrix_set_rhs(&m2, 0, IVP_REAL(3.0));
    ivp_great_matrix_set(&m2, 1, 0, IVP_REAL(2.0));
    ivp_great_matrix_set(&m2, 1, 1, IVP_REAL(4.0));
    ivp_great_matrix_set_rhs(&m2, 1, IVP_REAL(99.0));  /* inconsistent */
    IVP_ASSERT_EQ(ivp_great_matrix_solve(&m2, x), IVP_ERR_SINGULAR_MATRIX);
    ivp_great_matrix_destroy(&m2, NULL);
    ivp_great_matrix_destroy(&m, NULL);
}

/* Determinant computation */
IVP_TEST(matrix_determinant) {
    ivp_great_matrix_t m;
    IVP_ASSERT_EQ(ivp_great_matrix_init(&m, 3, NULL), IVP_OK);
    /* det of [[1,2,3],[4,5,6],[7,8,0]] = 1*(0-48) - 2*(0-42) + 3*(32-35)
     *       = -48 + 84 - 9 = 27 */
    ivp_great_matrix_set(&m, 0, 0, IVP_REAL(1.0));
    ivp_great_matrix_set(&m, 0, 1, IVP_REAL(2.0));
    ivp_great_matrix_set(&m, 0, 2, IVP_REAL(3.0));
    ivp_great_matrix_set(&m, 1, 0, IVP_REAL(4.0));
    ivp_great_matrix_set(&m, 1, 1, IVP_REAL(5.0));
    ivp_great_matrix_set(&m, 1, 2, IVP_REAL(6.0));
    ivp_great_matrix_set(&m, 2, 0, IVP_REAL(7.0));
    ivp_great_matrix_set(&m, 2, 1, IVP_REAL(8.0));
    ivp_great_matrix_set(&m, 2, 2, IVP_REAL(0.0));
    ivp_real_t det;
    IVP_ASSERT_EQ(ivp_great_matrix_determinant(&m, &det), IVP_OK);
    IVP_ASSERT_NEAR(det, 27.0, 1e-4);
    ivp_great_matrix_destroy(&m, NULL);
}

/* Identity matrix �?x = b */
IVP_TEST(matrix_identity) {
    ivp_great_matrix_t m;
    IVP_ASSERT_EQ(ivp_great_matrix_init(&m, 4, NULL), IVP_OK);
    for (int i = 0; i < 4; i++) {
        ivp_great_matrix_set(&m, i, i, IVP_REAL(1.0));
        ivp_great_matrix_set_rhs(&m, i, (ivp_real_t)(i + 1));
    }
    ivp_real_t x[4];
    IVP_ASSERT_EQ(ivp_great_matrix_solve(&m, x), IVP_OK);
    for (int i = 0; i < 4; i++) {
        IVP_ASSERT_NEAR(x[i], (double)(i + 1), 1e-6);
    }
    ivp_great_matrix_destroy(&m, NULL);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Two-Ball Scenario (Environment-Level)
 * ════════════════════════════════════════════════════════════════════════════ */

/* Ball template: IVP_Template_Ball::radius defines the collision sphere. */
static void ball_template(ivp_object_template_t *tmpl, ivp_real_t mass, ivp_real_t radius)
{
    ivp_object_template_init(tmpl);
    tmpl->type = IVP_OBJECT_BALL;
    tmpl->ball_radius = radius;
    tmpl->mass = mass;
    ivp_real_t I = IVP_REAL(0.4) * mass * radius * radius;
    ivp_vec3r_set(&tmpl->rot_inertia, I, I, I);
    tmpl->rot_inertia_is_factor = false; /* absolute inertia */
    tmpl->material.elasticity = IVP_REAL(1.0);
    tmpl->material.friction = IVP_REAL(0.0);
    tmpl->speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl->rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
}

static ivp_environment_t *make_zero_g_env(void)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

/* Two equal-mass balls head-on collision via environment simulation */
IVP_TEST(two_balls_head_on) {
    ivp_environment_t *env = make_zero_g_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_template_t tmpl;
    ball_template(&tmpl, IVP_REAL(1.0), IVP_REAL(0.5));

    /* Ball A at (-2, 0, 0) moving +x at 1 m/s */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *ball_a = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(ball_a);
    ivp_vec3r_set(&ball_a->physical_core->speed, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Ball B at (2, 0, 0) moving -x at 1 m/s */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *ball_b = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(ball_b);
    ivp_vec3r_set(&ball_b->physical_core->speed, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Approach speed = 2 m/s, gap = 3 m -> contact after ~1.5 s.
     * Run 200 PSIs (~3 s). */
    ivp_real_t initial_momentum =
        ball_a->physical_core->speed.k[0] * ball_a->physical_core->mass +
        ball_b->physical_core->speed.k[0] * ball_b->physical_core->mass;

    for (int i = 0; i < 200; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    ivp_real_t final_momentum =
        ball_a->physical_core->speed.k[0] * ball_a->physical_core->mass +
        ball_b->physical_core->speed.k[0] * ball_b->physical_core->mass;
    IVP_ASSERT_NEAR(initial_momentum, final_momentum, 0.05);

    /* After the elastic head-on collision the velocities have reversed. */
    IVP_ASSERT_TRUE(ball_a->physical_core->speed.k[0] < IVP_REAL(0.0));
    IVP_ASSERT_TRUE(ball_b->physical_core->speed.k[0] > IVP_REAL(0.0));

    ivp_environment_destroy(env);
}

/* Two balls with different masses */
IVP_TEST(two_balls_different_mass) {
    ivp_environment_t *env = make_zero_g_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_template_t tmpl;

    /* Heavy ball (mass=3) at -2, moving +x at 1 */
    ball_template(&tmpl, IVP_REAL(3.0), IVP_REAL(0.5));
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *heavy = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(heavy);
    ivp_vec3r_set(&heavy->physical_core->speed, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Light ball (mass=1) at +2, stationary */
    ball_template(&tmpl, IVP_REAL(1.0), IVP_REAL(0.5));
    ivp_vec3r_set(&tmpl.position, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *light = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(light);

    ivp_real_t p0 = heavy->physical_core->speed.k[0] * heavy->physical_core->mass +
                    light->physical_core->speed.k[0] * light->physical_core->mass;

    for (int i = 0; i < 300; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    ivp_real_t p1 = heavy->physical_core->speed.k[0] * heavy->physical_core->mass +
                    light->physical_core->speed.k[0] * light->physical_core->mass;

    /* Momentum conservation */
    IVP_ASSERT_NEAR(p0, p1, 0.1);

    /* Heavy ball still moves forward (slower), light ball is pushed faster */
    IVP_ASSERT_TRUE(heavy->physical_core->speed.k[0] > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(light->physical_core->speed.k[0] > heavy->physical_core->speed.k[0]);

    ivp_environment_destroy(env);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Energy Tests
 * ════════════════════════════════════════════════════════════════════════════ */

IVP_TEST(kinetic_energy_basic) {
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.ball_radius = IVP_REAL(1.0);
    tmpl.mass = IVP_REAL(2.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.8), IVP_REAL(0.8), IVP_REAL(0.8));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */

    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj);
    ivp_vec3r_set(&obj->physical_core->speed, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* KE = 0.5 * 2 * 9 = 9 */
    IVP_ASSERT_NEAR(ivp_object_get_energy(obj), 9.0, 1e-5);

    /* Rotational part: 0.5 * 0.8 * 2^2 = 1.6 */
    obj->physical_core->rot_speed = ivp_vec3f(0.0f, 2.0f, 0.0f);
    IVP_ASSERT_NEAR(ivp_object_get_energy(obj), 10.6, 1e-5);

    ivp_environment_destroy(env);
}

IVP_TEST(total_energy_conservation) {
    /* Two-ball elastic collision should conserve kinetic energy */
    ivp_environment_t *env = make_zero_g_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_template_t tmpl;
    ball_template(&tmpl, IVP_REAL(1.0), IVP_REAL(0.5));

    ivp_vec3r_set(&tmpl.position, IVP_REAL(-1.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *a = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(a);
    ivp_vec3r_set(&a->physical_core->speed, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_vec3r_set(&tmpl.position, IVP_REAL(1.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(b);

    ivp_real_t ke_before = ivp_object_get_energy(a) + ivp_object_get_energy(b);

    /* Run until collision and separation */
    for (int i = 0; i < 200; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    ivp_real_t ke_after = ivp_object_get_energy(a) + ivp_object_get_energy(b);

    /* The collision happened (B was pushed) and energy is approximately
     * conserved (elastic collision, no damping). */
    IVP_ASSERT_TRUE(b->physical_core->speed.k[0] > IVP_REAL(0.5));
    IVP_ASSERT_NEAR(ke_before, ke_after, 0.5);

    ivp_environment_destroy(env);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Static Object Tests
 * ════════════════════════════════════════════════════════════════════════════ */

IVP_TEST(static_object_not_moved) {
    ivp_environment_t *env = make_zero_g_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_template_t tmpl;
    ball_template(&tmpl, IVP_REAL(1e6), IVP_REAL(0.5));
    tmpl.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *wall = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(wall);

    /* Ball hitting the static ball */
    ball_template(&tmpl, IVP_REAL(1.0), IVP_REAL(0.5));
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *ball = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(ball);
    ivp_vec3r_set(&ball->physical_core->speed, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    for (int i = 0; i < 100; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    /* Wall has not moved, ball bounced back */
    IVP_ASSERT_NEAR(wall->physical_core->pos_world_f_core_last_psi.k[0], 1.0, 1e-6);
    IVP_ASSERT_NEAR(wall->physical_core->speed.k[0], 0.0, 1e-6);
    IVP_ASSERT_TRUE(ball->physical_core->speed.k[0] < IVP_REAL(0.0));

    ivp_environment_destroy(env);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Solver Constants Sanity
 * ════════════════════════════════════════════════════════════════════════════ */

IVP_TEST(solver_constants_values) {
    IVP_ASSERT_NEAR((double)IVP_SOLVER_MATRIX_EPS, 1e-9, 1e-15);
    IVP_ASSERT_NEAR((double)IVP_IMPACT_EPS, 1e-4, 1e-10);
    IVP_ASSERT_EQ(IVP_MAX_GREAT_MATRIX_SIZE, 2048);
    IVP_ASSERT_EQ(IVP_MAXIMUM_SYSTEM_PUSH_NUMBER, 5000);
    IVP_ASSERT_EQ(IVP_MAX_FRICTION_MATRIX, 150);
}

/* ════════════════════════════════════════════════════════════════════════════
 * Test Registration and Main
 * ════════════════════════════════════════════════════════════════════════════ */

int main(void) {
    /* Great matrix */
    IVP_REGISTER(matrix_1x1);
    IVP_REGISTER(matrix_2x2);
    IVP_REGISTER(matrix_3x3_pivot);
    IVP_REGISTER(matrix_singular);
    IVP_REGISTER(matrix_determinant);
    IVP_REGISTER(matrix_identity);

    /* Two-ball scenarios */
    IVP_REGISTER(two_balls_head_on);
    IVP_REGISTER(two_balls_different_mass);

    /* Energy */
    IVP_REGISTER(kinetic_energy_basic);
    IVP_REGISTER(total_energy_conservation);

    /* Static */
    IVP_REGISTER(static_object_not_moved);

    /* Constants */
    IVP_REGISTER(solver_constants_values);

    IVP_RUN_ALL();
}
