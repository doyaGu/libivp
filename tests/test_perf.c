/* test_perf.c — Performance regression smoke tests for IVP C17
 *
 * Runs the three parity scenarios (freefall, two_balls, slope_friction)
 * with a larger step count and measures wall-clock time per step.
 * Fails if the per-step time exceeds the budget threshold.
 *
 * These are intentionally generous budgets to catch major regressions,
 * not micro-benchmarks.  Run with:
 *   test_perf [--budget-us <microseconds_per_step>]
 */
#include "ivp_test.h"

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_time.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/builder/ivp_build_surface.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
static double perf_now_us(void) {
    LARGE_INTEGER freq, cnt;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&cnt);
    return (double)cnt.QuadPart / (double)freq.QuadPart * 1e6;
}
#else
#include <time.h>
static double perf_now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e6 + (double)ts.tv_nsec / 1000.0;
}
#endif

/* Default per-step budget: 500 µs.  Even a debug build on low-end
 * hardware should easily beat this for simple 1–2 object scenarios. */
#define DEFAULT_BUDGET_US 500.0

/* Step counts for each scenario — enough to amortize startup. */
#define PERF_STEPS 1000

/* ── Axis-angle → quaternion ────────────────────────────────────────────── */

static ivp_quatr_t quat_from_axis_angle_perf(ivp_real_t ax, ivp_real_t ay,
                                              ivp_real_t az, ivp_real_t radians)
{
    ivp_real_t half = radians * IVP_REAL(0.5);
    ivp_real_t s = ivp_sin(half);
    ivp_real_t c = ivp_cos(half);
    return ivp_quatr_make(ax * s, ay * s, az * s, c);
}

/* ── Helper: create ball ────────────────────────────────────────────────── */

static ivp_object_t *make_ball(ivp_environment_t *env,
                               ivp_real_t mass, ivp_real_t radius,
                               ivp_real_t x, ivp_real_t y, ivp_real_t z,
                               ivp_real_t friction, ivp_real_t elasticity)
{
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = mass;
    ivp_real_t inertia = IVP_REAL(0.4) * mass * radius * radius;
    ivp_vec3r_set(&tmpl.rot_inertia, inertia, inertia, inertia);
    tmpl.material.friction   = friction;
    tmpl.material.elasticity = elasticity;
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl.position, x, y, z);
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    if (obj) {
        obj->core.upper_limit_radius = radius;
        ivp_vec3r_set(&obj->core.speed, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        obj->core.rot_speed.k[0] = 0.0f;
        obj->core.rot_speed.k[1] = 0.0f;
        obj->core.rot_speed.k[2] = 0.0f;
        obj->core.delta_world_f_core_psis = obj->core.speed;
    }
    return obj;
}

/* ── Perf: freefall ─────────────────────────────────────────────────────── */

IVP_TEST(perf_freefall) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)(1.0 / 60.0);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *ball = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.5),
                                   IVP_REAL(0.0), IVP_REAL(-5.0), IVP_REAL(0.0),
                                   IVP_REAL(0.8), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(ball);

    double t0 = perf_now_us();
    for (int i = 0; i < PERF_STEPS; i++) {
        ivp_environment_simulate_one_psi(env);
    }
    double elapsed_us = perf_now_us() - t0;
    double per_step = elapsed_us / PERF_STEPS;

    printf("(%.1f us/step, total %.1f ms) ", per_step, elapsed_us / 1000.0);

    IVP_ASSERT_TRUE(per_step < DEFAULT_BUDGET_US);
    ivp_environment_destroy(env);
}

/* ── Perf: two balls ────────────────────────────────────────────────────── */

IVP_TEST(perf_two_balls) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)(1.0 / 60.0);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.5),
                                IVP_REAL(-2.0), IVP_REAL(-2.0), IVP_REAL(0.0),
                                IVP_REAL(0.8), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.5),
                                IVP_REAL(2.0), IVP_REAL(-2.0), IVP_REAL(0.0),
                                IVP_REAL(0.8), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    /* Initial velocities: approaching */
    ivp_vec3r_set(&a->core.speed, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    a->core.delta_world_f_core_psis = a->core.speed;
    ivp_vec3r_set(&b->core.speed, IVP_REAL(-2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    b->core.delta_world_f_core_psis = b->core.speed;

    double t0 = perf_now_us();
    for (int i = 0; i < PERF_STEPS; i++) {
        ivp_environment_simulate_one_psi(env);
    }
    double elapsed_us = perf_now_us() - t0;
    double per_step = elapsed_us / PERF_STEPS;

    printf("(%.1f us/step, total %.1f ms) ", per_step, elapsed_us / 1000.0);

    IVP_ASSERT_TRUE(per_step < DEFAULT_BUDGET_US);
    ivp_environment_destroy(env);
}

/* ── Perf: slope friction ───────────────────────────────────────────────── */

IVP_TEST(perf_slope_friction) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)(1.0 / 60.0);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    /* Ramp */
    ivp_compact_surface_t *ramp_surface =
        ivp_build_box_surface(NULL, 5.0f, 0.25f, 2.5f);
    IVP_ASSERT_NOT_NULL(ramp_surface);

    ivp_surman_polygon_t ramp_surman;
    ivp_surman_polygon_init(&ramp_surman, ramp_surface);

    ivp_object_template_t tmpl_ramp;
    ivp_object_template_init(&tmpl_ramp);
    tmpl_ramp.type = IVP_OBJECT_POLYGON;
    tmpl_ramp.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl_ramp.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl_ramp.material.friction   = IVP_REAL(0.8);
    tmpl_ramp.material.elasticity = IVP_REAL(0.0);
    tmpl_ramp.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl_ramp.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl_ramp.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl_ramp.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl_ramp.orientation = quat_from_axis_angle_perf(
        IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0),
        (ivp_real_t)(30.0 * 3.14159265358979323846 / 180.0));

    ivp_object_t *ramp = ivp_environment_create_object(
        env, &tmpl_ramp, &ramp_surman.base);
    IVP_ASSERT_NOT_NULL(ramp);
    ramp->core.upper_limit_radius = IVP_REAL(5.0);

    /* Ball above ramp */
    ivp_object_t *ball = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.5),
                                   IVP_REAL(-4.0), IVP_REAL(-4.0), IVP_REAL(0.0),
                                   IVP_REAL(0.4), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(ball);

    double t0 = perf_now_us();
    for (int i = 0; i < PERF_STEPS; i++) {
        ivp_environment_simulate_one_psi(env);
    }
    double elapsed_us = perf_now_us() - t0;
    double per_step = elapsed_us / PERF_STEPS;

    printf("(%.1f us/step, total %.1f ms) ", per_step, elapsed_us / 1000.0);

    IVP_ASSERT_TRUE(per_step < DEFAULT_BUDGET_US);

    ivp_free_aligned(NULL, ramp_surface);
    ivp_environment_destroy(env);
}

/* ── Perf: multi-object scaling ─────────────────────────────────────────── */

IVP_TEST(perf_multi_object) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)(1.0 / 60.0);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    /* Create 20 balls in a grid — tests N² broadphase scaling */
    const int N = 20;
    for (int i = 0; i < N; i++) {
        ivp_real_t x = (ivp_real_t)(i % 5) * IVP_REAL(3.0);
        ivp_real_t y = (ivp_real_t)(i / 5) * IVP_REAL(3.0);
        ivp_object_t *obj = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.5),
                                       x, y, IVP_REAL(0.0),
                                       IVP_REAL(0.5), IVP_REAL(0.0));
        IVP_ASSERT_NOT_NULL(obj);
    }

    /* Budget: 2000 µs per step for 20 objects (generous for N² broadphase) */
    double budget = 2000.0;
    int perf_steps = 500;

    double t0 = perf_now_us();
    for (int i = 0; i < perf_steps; i++) {
        ivp_environment_simulate_one_psi(env);
    }
    double elapsed_us = perf_now_us() - t0;
    double per_step = elapsed_us / perf_steps;

    printf("(%.1f us/step for %d objects, total %.1f ms) ",
           per_step, N, elapsed_us / 1000.0);

    IVP_ASSERT_TRUE(per_step < budget);
    ivp_environment_destroy(env);
}

/* ── Registration ──────────────────────────────────────────────────────── */

int main(void) {
    IVP_REGISTER(perf_freefall);
    IVP_REGISTER(perf_two_balls);
    IVP_REGISTER(perf_slope_friction);
    IVP_REGISTER(perf_multi_object);
    IVP_RUN_ALL();
}
