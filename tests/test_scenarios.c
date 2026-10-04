/* test_scenarios.c — Scenario parity tests for IVP C17
 *
 * Runs freefall, two_balls, and slope_friction scenarios identical to
 * tools/ivp_reference_runner/main.cxx and outputs JSONL snapshots.
 *
 * Usage:
 *   test_scenarios --scenario freefall|two_balls|slope_friction [--steps N] [--dt seconds] [--output file.jsonl]
 *
 * If --output is omitted, writes to stdout.
 * CTest invokes this, pipes output to a file, then compares via compare_jsonl.py.
 */

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_time.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/controllers/ivp_controller_stiff_spring.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/controllers/ivp_actuator_rot_mot.h>
#include <ivp/controllers/ivp_actuator_force.h>
#include <ivp/controllers/ivp_actuator_check_dist.h>
#include <ivp/controllers/ivp_forcefield.h>
#include <ivp/controllers/ivp_controller_motion.h>
#include <ivp/controllers/ivp_controller_phantom.h>
#include <ivp/controllers/ivp_controller_raycast_car.h>
#include <ivp/controllers/ivp_controller_buoyancy.h>
#include <ivp/controllers/ivp_car_system.h>
#include <ivp/controllers/ivp_actuator_torque.h>
#include <ivp/controllers/ivp_actuator_stabilizer.h>
#include <ivp/controllers/ivp_actuator_extra.h>
#include <ivp/controllers/ivp_controller_floating.h>
#include <ivp/controllers/ivp_controller_world_friction.h>
#include <ivp/controllers/ivp_controller_golem.h>
#include <ivp/controllers/ivp_constraint_fixed_keyed.h>
#include <ivp/controllers/ivp_attacher_to_cores.h>
#include <ivp/controllers/ivp_controller_airboat.h>
#include <ivp/controllers/ivp_controller_fake_jetski.h>
#include <ivp/containers/ivp_set.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/builder/ivp_build_grid.h>
#include <ivp/geometry/ivp_surman_grid.h>
#include <ivp/dynamics/ivp_universe_manager.h>
#include <ivp/dynamics/ivp_merge_core.h>
#include <ivp/dynamics/ivp_object_attach.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <time.h>
#endif

static int64_t benchmark_now_ns(void)
{
#ifdef _WIN32
    LARGE_INTEGER frequency, counter;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&counter);
    return (int64_t)((double)counter.QuadPart * 1000000000.0 /
                     (double)frequency.QuadPart);
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * INT64_C(1000000000) + (int64_t)ts.tv_nsec;
#endif
}

static int64_t g_benchmark_simulate_ns;
static int g_benchmark_enabled;

/* ── Scenario IDs (match reference runner) ──────────────────────────────── */
enum {
    SCENARIO_FREEFALL       = 0,
    SCENARIO_TWO_BALLS      = 1,
    SCENARIO_SLOPE_FRICTION = 2,
    SCENARIO_SPRINGS        = 3,
    SCENARIO_ROPE           = 4,
    SCENARIO_MOTOR          = 5,
    SCENARIO_BUOYANCY       = 6,

    SCENARIO_CUBES           = 100,
    SCENARIO_FORCE_ACTUATOR  = 101,
    SCENARIO_FORCEFIELD      = 102,
    SCENARIO_STIFF_SPRING    = 103,
    SCENARIO_CHECK_DISTANCE  = 104,
    SCENARIO_COLLISION_FILTER= 105,
    SCENARIO_MOTION_CONTROLLER = 106,
    SCENARIO_PHANTOM         = 107,
    SCENARIO_VEHICLE         = 108,
    SCENARIO_CAR_REAL_WHEELS = 109,

    /* controllers2 (run_* block "Controller coverage scenarios") */
    SCENARIO_TORQUE           = 300,
    SCENARIO_STABILIZER       = 301,
    SCENARIO_FLOATING         = 302,
    SCENARIO_WORLD_FRICTION   = 303,
    SCENARIO_GOLEM            = 304,
    SCENARIO_FIXED_KEYED      = 305,
    SCENARIO_HINGE_LIMITS     = 306,
    SCENARIO_CARDAN_TENSE     = 307,
    SCENARIO_SLIDER_LIMITS    = 308,
    SCENARIO_CONSTRAINT_BREAK = 309,
    SCENARIO_ATTACHER         = 310,
    SCENARIO_ACTUATOR_EXTRA   = 311,
    SCENARIO_AIRBOAT          = 312,
    SCENARIO_FAKE_JETSKI      = 313,

    /* controllers3 (run_* block "Controller coverage scenarios (controllers3)") */
    SCENARIO_RAYCAST_CAR_DRIVE = 400,
    SCENARIO_CHECK_DIST_EVENTS = 401,
    SCENARIO_GOLEM_BEAM        = 402,

    /* misc (run_* block "Miscellaneous parity scenarios") */
    SCENARIO_UNIVERSE_EVICT    = 500,

    /* shared cores (run_* block "Shared core parity scenarios") */
    SCENARIO_MERGE_OBJECTS     = 600,
    SCENARIO_OBJECT_ATTACH     = 601,
    SCENARIO_MERGE_BUOYANCY    = 602,
    SCENARIO_ANCHOR_FOLLOW     = 603,

    SCENARIO_CONCAVE_STATIC   = 200,
    SCENARIO_COMPOUND_DYNAMIC = 201,
    SCENARIO_CONVEX_HULLS     = 202,
    SCENARIO_GRID_TERRAIN     = 203,
    SCENARIO_HULL_PILE        = 204,
    SCENARIO_GALTON           = 205,
    SCENARIO_FAST_IMPACTS     = 206,
    SCENARIO_SPAWN_REMOVE     = 207,
    SCENARIO_LONG_RANGE       = 208
};

static const char *scenario_name_from_id(int scenario_id)
{
    switch (scenario_id) {
        case SCENARIO_FREEFALL: return "freefall";
        case SCENARIO_TWO_BALLS: return "two_balls";
        case SCENARIO_SLOPE_FRICTION: return "slope_friction";
        case SCENARIO_SPRINGS: return "springs";
        case SCENARIO_ROPE: return "rope";
        case SCENARIO_MOTOR: return "motor";
        case SCENARIO_BUOYANCY: return "buoyancy";
        case SCENARIO_CUBES: return "cubes";
        case SCENARIO_FORCE_ACTUATOR: return "force_actuator";
        case SCENARIO_FORCEFIELD: return "forcefield";
        case SCENARIO_STIFF_SPRING: return "stiff_spring";
        case SCENARIO_CHECK_DISTANCE: return "check_distance";
        case SCENARIO_COLLISION_FILTER: return "collision_filter";
        case SCENARIO_MOTION_CONTROLLER: return "motion_controller";
        case SCENARIO_PHANTOM: return "phantom";
        case SCENARIO_VEHICLE: return "vehicle";
        case SCENARIO_CAR_REAL_WHEELS: return "car_real_wheels";
        case SCENARIO_TORQUE: return "torque";
        case SCENARIO_STABILIZER: return "stabilizer";
        case SCENARIO_FLOATING: return "floating";
        case SCENARIO_WORLD_FRICTION: return "world_friction";
        case SCENARIO_GOLEM: return "golem";
        case SCENARIO_FIXED_KEYED: return "fixed_keyed";
        case SCENARIO_HINGE_LIMITS: return "hinge_limits";
        case SCENARIO_CARDAN_TENSE: return "cardan_tense";
        case SCENARIO_SLIDER_LIMITS: return "slider_limits";
        case SCENARIO_CONSTRAINT_BREAK: return "constraint_break";
        case SCENARIO_ATTACHER: return "attacher";
        case SCENARIO_ACTUATOR_EXTRA: return "actuator_extra";
        case SCENARIO_AIRBOAT: return "airboat";
        case SCENARIO_FAKE_JETSKI: return "fake_jetski";
        case SCENARIO_RAYCAST_CAR_DRIVE: return "raycast_car_drive";
        case SCENARIO_CHECK_DIST_EVENTS: return "check_dist_events";
        case SCENARIO_GOLEM_BEAM: return "golem_beam";
        case SCENARIO_UNIVERSE_EVICT: return "universe_evict";
        case SCENARIO_MERGE_OBJECTS: return "merge_objects";
        case SCENARIO_OBJECT_ATTACH: return "object_attach";
        case SCENARIO_MERGE_BUOYANCY: return "merge_buoyancy";
        case SCENARIO_ANCHOR_FOLLOW: return "anchor_follow";
        case SCENARIO_CONCAVE_STATIC: return "concave_static";
        case SCENARIO_COMPOUND_DYNAMIC: return "compound_dynamic";
        case SCENARIO_CONVEX_HULLS: return "convex_hulls";
        case SCENARIO_GRID_TERRAIN: return "grid_terrain";
        case SCENARIO_HULL_PILE: return "hull_pile";
        case SCENARIO_GALTON: return "galton";
        case SCENARIO_FAST_IMPACTS: return "fast_impacts";
        case SCENARIO_SPAWN_REMOVE: return "spawn_remove";
        case SCENARIO_LONG_RANGE: return "long_range";
        default: return "unknown";
    }
}

/* ── Invariant check infrastructure ────────────────────────────────────── */

#define INV_MAX_MESSAGES 32

typedef struct {
    int checks_run;
    int checks_passed;
    int checks_failed;
    char messages[INV_MAX_MESSAGES][256];
    int msg_count;
} invariant_result_t;

static void inv_init(invariant_result_t *inv)
{
    inv->checks_run = 0;
    inv->checks_passed = 0;
    inv->checks_failed = 0;
    inv->msg_count = 0;
}

static void inv_check(invariant_result_t *inv, int condition, const char *fmt, ...)
{
    inv->checks_run++;
    if (condition) {
        inv->checks_passed++;
    } else {
        inv->checks_failed++;
        if (inv->msg_count < INV_MAX_MESSAGES) {
            va_list ap;
            va_start(ap, fmt);
            vsnprintf(inv->messages[inv->msg_count], 256, fmt, ap);
            va_end(ap);
            inv->msg_count++;
        }
    }
}

static void inv_report(const invariant_result_t *inv, const char *scenario_name)
{
    fprintf(stderr, "invariants[%s]: %d/%d passed",
            scenario_name, inv->checks_passed, inv->checks_run);
    if (inv->checks_failed > 0) {
        fprintf(stderr, " (%d FAILED)\n", inv->checks_failed);
        for (int i = 0; i < inv->msg_count; i++) {
            fprintf(stderr, "  FAIL: %s\n", inv->messages[i]);
        }
    } else {
        fprintf(stderr, " (all ok)\n");
    }
}

static int g_phantom_event_count = 0;
static int g_check_dist_event_count = 0;

static void phantom_event_counter_cb(void *ctx,
                                     ivp_object_t *phantom_owner,
                                     ivp_object_t *other,
                                     bool entered)
{
    (void)ctx;
    (void)phantom_owner;
    (void)other;
    (void)entered;
    g_phantom_event_count++;
}

static void check_dist_counter_cb(void *ctx,
                                  ivp_object_t *obj_a,
                                  ivp_object_t *obj_b,
                                  ivp_real_t distance,
                                  bool is_inside_range)
{
    (void)ctx;
    (void)obj_a;
    (void)obj_b;
    (void)distance;
    (void)is_inside_range;
    g_check_dist_event_count++;
}

/* ── JSONL output helpers ───────────────────────────────────────────────── */

static void json_vec3(FILE *f, const ivp_vec3r_t *v)
{
    fprintf(f, "{\"x\":%.17g,\"y\":%.17g,\"z\":%.17g}",
            (double)v->k[0], (double)v->k[1], (double)v->k[2]);
}

static void json_vec3f(FILE *f, const ivp_vec3f_t *v)
{
    fprintf(f, "{\"x\":%.9g,\"y\":%.9g,\"z\":%.9g}",
            (double)v->k[0], (double)v->k[1], (double)v->k[2]);
}

/* Cast ivp_vec3r_t (double) to float before output, matching reference
 * runner's json_vec3_float which reads IVP_U_Float_Point (float). */
static void json_vec3_as_float(FILE *f, const ivp_vec3r_t *v)
{
    fprintf(f, "{\"x\":%.9g,\"y\":%.9g,\"z\":%.9g}",
            (double)(float)v->k[0], (double)(float)v->k[1], (double)(float)v->k[2]);
}

static void json_quat(FILE *f, const ivp_quatr_t *q)
{
    fprintf(f, "{\"x\":%.17g,\"y\":%.17g,\"z\":%.17g,\"w\":%.17g}",
            (double)q->x, (double)q->y, (double)q->z, (double)q->w);
}

static void snapshot_object(FILE *f, int id, const char *type,
                            const ivp_object_t *obj)
{
    const ivp_core_t *core = ivp_object_get_core_const(obj);

    fprintf(f, "{\"id\":%d,\"type\":\"%s\",\"pos\":", id, type);
    json_vec3(f, &core->m_world_f_core_last_psi.pos);   /* get_position_PSI() */

    /* Normalize quaternion before output — integration drifts slightly
     * from unit length, which would cause self-compare angular error. */
    ivp_quatr_t q_norm = core->q_world_f_core_last_psi;
    ivp_quat_normalize(&q_norm);
    fprintf(f, ",\"quat\":");
    json_quat(f, &q_norm);
    /* speed is ivp_vec3r_t in C17 but IVP_U_Float_Point (float) in original;
     * cast to float to match reference runner output precision. */
    fprintf(f, ",\"lin_vel\":");
    json_vec3_as_float(f, &core->speed);
    fprintf(f, ",\"ang_vel\":");
    json_vec3f(f, &core->rot_speed);

    ivp_real_t energy = ivp_core_get_energy(core);   /* get_energy_on_test(speed, rot_speed) */
    fprintf(f, ",\"energy\":%.17g}", (double)energy);
}

static void snapshot_frame(FILE *f, int scenario_id, int step, double dt,
                           ivp_environment_t *env,
                           ivp_object_t **objects, const char **types,
                           int object_count)
{
    if (!f) return;
    double t = (double)ivp_environment_get_time(env);
    fprintf(f, "{\"scenario\":%d,\"step\":%d,\"dt\":%.17g,\"time\":%.17g,\"objects\":[",
            scenario_id, step, dt, t);
    for (int i = 0; i < object_count; i++) {
        if (i) fprintf(f, ",");
        snapshot_object(f, i, types[i], objects[i]);
    }
    fprintf(f, "]}\n");
}


/* Reference runner build_box_compact_surface: pointsoup of the 8 corners
 * (sx, sy, sz in -1..1 order) through the qhull convex hull builder. */
static ivp_compact_surface_t *ref_box_surface(double hx, double hy, double hz)
{
    double coords[24];
    int n = 0;
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                coords[n++] = (double)sx * hx;
                coords[n++] = (double)sy * hy;
                coords[n++] = (double)sz * hz;
            }
        }
    }
    return ivp_build_convex_surface_d(NULL, coords, 8);
}

/* ── Axis-angle → quaternion ────────────────────────────────────────────── */

static ivp_quatr_t quat_from_axis_angle(ivp_real_t ax, ivp_real_t ay,
                                         ivp_real_t az, ivp_real_t radians)
{
    ivp_real_t half = radians * IVP_REAL(0.5);
    ivp_real_t s = ivp_sin(half);
    ivp_real_t c = ivp_cos(half);
    return ivp_quatr_make(ax * s, ay * s, az * s, c);
}

static bool scenario_init_box_surman(float hx, float hy, float hz,
                                     ivp_compact_surface_t **out_surface,
                                     ivp_surman_polygon_t *out_surman)
{
    ivp_compact_surface_t *surface = ref_box_surface(hx, hy, hz);
    if (!surface) return false;
    ivp_surman_polygon_init(out_surman, surface);
    *out_surface = surface;
    return true;
}

static void scenario_free_surfaces(ivp_compact_surface_t **surfaces, int n)
{
    for (int i = 0; i < n; i++) ivp_compact_surface_free(NULL, surfaces[i]);
}

/* Reference runner create_static_box / create_dynamic_box: every object gets
 * its own compact surface and surface manager, built right before the object
 * is created.  Sharing one ledge between objects is not equivalent: the
 * minimizer's loop detection hash is keyed on edge addresses, so a shared
 * ledge can report a false loop when the same edge pair shows up with the
 * objects swapped.  The surface is returned through *out_surface (NULL when
 * building it failed) and must be freed after ivp_environment_destroy. */
static ivp_object_t *scenario_create_box(ivp_environment_t *env, const ivp_object_template_t *tmpl,
                                         double hx, double hy, double hz,
                                         ivp_compact_surface_t **out_surface,
                                         ivp_surman_polygon_t *out_surman)
{
    ivp_compact_surface_t *surface = ref_box_surface(hx, hy, hz);
    *out_surface = surface;
    if (!surface) return NULL;
    ivp_surman_polygon_init(out_surman, surface);
    return ivp_environment_create_object(env, tmpl, &out_surman->base);
}

/* The reference runner keeps the IVP_Environment default gravity
 * IVP_U_Point(0.0f, 9.83f, 0.0f) (env->gravity, gravity_scalar). */
#define SCENARIO_REF_GRAVITY ((ivp_real_t)9.83f)

/* One reference runner step: env->simulate_until(env->get_current_time() + cfg.dt)
 * (differs from simulate_one_psi when set_delta_PSI_time rejected dt). */
static void scenario_step(ivp_environment_t *env, double dt)
{
    int64_t start = g_benchmark_enabled ? benchmark_now_ns() : 0;
    ivp_environment_simulate_until(env, env->current_time + dt);
    if (g_benchmark_enabled) g_benchmark_simulate_ns += benchmark_now_ns() - start;
}

/* ── Scenario implementations ───────────────────────────────────────────── */

static int run_freefall(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));

    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    /* I = (2/5) * m * r^2 = 0.4 * 1.0 * 0.25 = 0.1 for r=0.5, m=1.0 */
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;

    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-5.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    tmpl.ball_radius = IVP_REAL(0.5);
    ivp_object_t *ball = ivp_environment_create_object(env, &tmpl, NULL);
    if (!ball) { ivp_environment_destroy(env); return 1; }

    ivp_vec3r_set(&ball->physical_core->speed, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ball->physical_core->rot_speed.k[0] = 0.0f;
    ball->physical_core->rot_speed.k[1] = 0.0f;
    ball->physical_core->rot_speed.k[2] = 0.0f;

    ivp_object_t *objects[1] = { ball };
    const char   *types[1]   = { "ball" };

    snapshot_frame(out, SCENARIO_FREEFALL, 0, dt, env, objects, types, 1);
    for (int step = 1; step <= steps; step++) {
        scenario_step(env, dt);
        snapshot_frame(out, SCENARIO_FREEFALL, step, dt, env, objects, types, 1);
    }

    if (inv) {
        const ivp_core_t *core = ivp_object_get_core_const(ball);
        double final_y = (double)core->pos_world_f_core_last_psi.k[1];
        double final_x = (double)core->pos_world_f_core_last_psi.k[0];
        double final_z = (double)core->pos_world_f_core_last_psi.k[2];
        double final_vy = (double)core->speed.k[1];
        double final_vx = (double)core->speed.k[0];
        double final_vz = (double)core->speed.k[2];

        /* Analytical: y = y0 + 0.5 * g * t^2 (Y-down, g=+9.83) */
        double t = (double)steps * dt;
        double y0 = -5.0;
        double g = 9.83;
        double expected_y = y0 + 0.5 * g * t * t;
        double pos_err = fabs(final_y - expected_y);
        /* Allow integrator error: Euler has O(dt) global error, be generous */
        double pos_tol = g * dt * t + 0.5;
        inv_check(inv, pos_err < pos_tol,
                  "freefall analytical y: got %.6f expected %.6f (err=%.6g tol=%.6g)",
                  final_y, expected_y, pos_err, pos_tol);

        /* Energy conservation: KE + PE should be constant.
         * Initial: KE=0, PE = m*g*(-y0) (using y0=-5 in Y-down).
         * We check |KE+PE - E0|/|E0| < 0.02 */
        double mass = 1.0;
        double KE = 0.5 * mass * (final_vx*final_vx + final_vy*final_vy + final_vz*final_vz);
        double PE = -mass * g * final_y; /* PE = -m*g*y in Y-down coords */
        double E0 = -mass * g * y0;
        double E_now = KE + PE;
        double energy_drift = (E0 != 0.0) ? fabs(E_now - E0) / fabs(E0) : fabs(E_now - E0);
        inv_check(inv, energy_drift < 0.10,
                  "freefall energy conservation: drift_ratio=%.6g (limit 0.10)", energy_drift);

        /* No lateral drift: x,z should stay at 0 */
        inv_check(inv, fabs(final_x) < 1e-6,
                  "freefall no lateral drift x: got %.12g", final_x);
        inv_check(inv, fabs(final_z) < 1e-6,
                  "freefall no lateral drift z: got %.12g", final_z);

        /* Velocity purely in Y: vx,vz should be ~0 */
        inv_check(inv, fabs(final_vx) < 1e-6,
                  "freefall velocity x should be zero: got %.12g", final_vx);
        inv_check(inv, fabs(final_vz) < 1e-6,
                  "freefall velocity z should be zero: got %.12g", final_vz);
    }

    ivp_environment_destroy(env);
    return 0;
}

static int run_two_balls(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));

    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    /* I = (2/5) * m * r^2 = 0.4 * 1.0 * 0.25 = 0.1 for r=0.5, m=1.0 */
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();

    /* Ball A at (-2, -2, 0) */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-2.0), IVP_REAL(-2.0), IVP_REAL(0.0));
    tmpl.ball_radius = IVP_REAL(0.5);
    ivp_object_t *a = ivp_environment_create_object(env, &tmpl, NULL);
    if (!a) { ivp_environment_destroy(env); return 1; }

    /* Ball B at (+2, -2, 0) */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(2.0), IVP_REAL(-2.0), IVP_REAL(0.0));
    tmpl.ball_radius = IVP_REAL(0.5);
    ivp_object_t *b = ivp_environment_create_object(env, &tmpl, NULL);
    if (!b) { ivp_environment_destroy(env); return 1; }

    /* Set initial velocities: A→+x, B→-x */
    ivp_vec3r_set(&a->physical_core->speed, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    a->physical_core->rot_speed.k[0] = 0.0f;
    a->physical_core->rot_speed.k[1] = 0.0f;
    a->physical_core->rot_speed.k[2] = 0.0f;

    ivp_vec3r_set(&b->physical_core->speed, IVP_REAL(-2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    b->physical_core->rot_speed.k[0] = 0.0f;
    b->physical_core->rot_speed.k[1] = 0.0f;
    b->physical_core->rot_speed.k[2] = 0.0f;

    ivp_object_t *objects[2] = { a, b };
    const char   *types[2]   = { "ball", "ball" };

    /* Track initial velocities for collision detection */
    double a_vx_initial = (double)a->physical_core->speed.k[0];
    double b_vx_initial = (double)b->physical_core->speed.k[0];
    int collision_detected = 0;

    snapshot_frame(out, SCENARIO_TWO_BALLS, 0, dt, env, objects, types, 2);
    for (int step = 1; step <= steps; step++) {
        scenario_step(env, dt);
        snapshot_frame(out, SCENARIO_TWO_BALLS, step, dt, env, objects, types, 2);

        /* Detect collision: velocity sign change for either ball */
        if (!collision_detected) {
            double a_vx = (double)a->physical_core->speed.k[0];
            double b_vx = (double)b->physical_core->speed.k[0];
            if ((a_vx_initial > 0 && a_vx <= 0) || (b_vx_initial < 0 && b_vx >= 0))
                collision_detected = 1;
        }
    }

    if (inv) {
        const ivp_core_t *core_a = ivp_object_get_core_const(a);
        const ivp_core_t *core_b = ivp_object_get_core_const(b);

        /* X-momentum conservation: equal mass, symmetric → total x-momentum ≈ 0 */
        double mass_a = 1.0, mass_b = 1.0;
        double px = mass_a * (double)core_a->speed.k[0] + mass_b * (double)core_b->speed.k[0];
        inv_check(inv, fabs(px) < 0.1,
                  "two_balls x-momentum conservation: |p_total_x|=%.6g (limit 0.1)", fabs(px));

        /* X-position symmetry: |a.x + b.x| should stay small */
        double ax = (double)core_a->pos_world_f_core_last_psi.k[0];
        double bx = (double)core_b->pos_world_f_core_last_psi.k[0];
        inv_check(inv, fabs(ax + bx) < 0.5,
                  "two_balls x-position symmetry: |a.x+b.x|=%.6g (limit 0.5)", fabs(ax + bx));

        /* Collision must have been detected */
        inv_check(inv, collision_detected,
                  "two_balls collision detected: velocity sign change did not occur");

    }

    ivp_environment_destroy(env);
    return 0;
}

static int run_slope_friction(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));

    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    /* --- Build ramp surface: box 5×0.25×2.5, rotated 30° around Z --- */
    ivp_compact_surface_t *ramp_surface =
        ref_box_surface(5.0f, 0.25f, 2.5f);
    if (!ramp_surface) { ivp_environment_destroy(env); return 1; }

    ivp_surman_polygon_t ramp_surman;
    ivp_surman_polygon_init(&ramp_surman, ramp_surface);

    ivp_object_template_t tmpl_ramp;
    ivp_object_template_init(&tmpl_ramp);
    tmpl_ramp.type = IVP_OBJECT_POLYGON;
    tmpl_ramp.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl_ramp.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl_ramp.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl_ramp.material.friction   = IVP_REAL(0.8);
    tmpl_ramp.material.elasticity = IVP_REAL(0.0);
    tmpl_ramp.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl_ramp.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl_ramp.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl_ramp.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Rotation: 30° around Z axis */
    ivp_real_t angle_rad = (ivp_real_t)(30.0 * 3.14159265358979323846 / 180.0);
    tmpl_ramp.orientation = quat_from_axis_angle(
        IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0), angle_rad);

    ivp_object_t *ramp = ivp_environment_create_object(
        env, &tmpl_ramp, &ramp_surman.base);
    if (!ramp) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, ramp_surface);
        return 1;
    }
    ivp_vec3r_set(&ramp->physical_core->speed, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ramp->physical_core->rot_speed.k[0] = 0.0f;
    ramp->physical_core->rot_speed.k[1] = 0.0f;
    ramp->physical_core->rot_speed.k[2] = 0.0f;

    /* --- Ball above ramp --- */
    ivp_object_template_t tmpl_ball;
    ivp_object_template_init(&tmpl_ball);
    tmpl_ball.type = IVP_OBJECT_BALL;
    tmpl_ball.mass = IVP_REAL(1.0);
    /* I = (2/5) * m * r^2 = 0.4 * 1.0 * 0.25 = 0.1 for r=0.5, m=1.0 */
    ivp_vec3r_set(&tmpl_ball.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    tmpl_ball.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl_ball.material.friction   = IVP_REAL(0.4);
    tmpl_ball.material.elasticity = IVP_REAL(0.0);
    tmpl_ball.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl_ball.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl_ball.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl_ball.position, IVP_REAL(-4.0), IVP_REAL(-4.0), IVP_REAL(0.0));
    tmpl_ball.orientation = ivp_quatr_identity();

    tmpl_ball.ball_radius = IVP_REAL(0.5);
    ivp_object_t *ball = ivp_environment_create_object(env, &tmpl_ball, NULL);
    if (!ball) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, ramp_surface);
        return 1;
    }
    ivp_vec3r_set(&ball->physical_core->speed, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ball->physical_core->rot_speed.k[0] = 0.0f;
    ball->physical_core->rot_speed.k[1] = 0.0f;
    ball->physical_core->rot_speed.k[2] = 0.0f;

    ivp_object_t *objects[2] = { ramp, ball };
    const char   *types[2]   = { "poly", "ball" };

    double ramp_y0 = (double)ramp->physical_core->pos_world_f_core_last_psi.k[1];
    double ball_y0 = (double)ball->physical_core->pos_world_f_core_last_psi.k[1];
    double ball_speed0 = 0.0;

    snapshot_frame(out, SCENARIO_SLOPE_FRICTION, 0, dt, env, objects, types, 2);
    for (int step = 1; step <= steps; step++) {
        scenario_step(env, dt);
        snapshot_frame(out, SCENARIO_SLOPE_FRICTION, step, dt, env, objects, types, 2);
    }

    if (inv) {
        const ivp_core_t *ramp_core = ivp_object_get_core_const(ramp);
        const ivp_core_t *ball_core = ivp_object_get_core_const(ball);

        /* Ramp stays static (position unchanged) */
        double ramp_y_final = (double)ramp_core->pos_world_f_core_last_psi.k[1];
        inv_check(inv, fabs(ramp_y_final - ramp_y0) < 1e-6,
                  "slope_friction ramp static: y moved by %.12g", fabs(ramp_y_final - ramp_y0));

        /* Ball moves downslope (y increases in Y-down coords) */
        double ball_y_final = (double)ball_core->pos_world_f_core_last_psi.k[1];
        inv_check(inv, ball_y_final > ball_y0,
                  "slope_friction ball moves downslope: y0=%.6f y_final=%.6f", ball_y0, ball_y_final);

        /* Ball gains speed */
        double vx = (double)ball_core->speed.k[0];
        double vy = (double)ball_core->speed.k[1];
        double vz = (double)ball_core->speed.k[2];
        double speed_final = sqrt(vx*vx + vy*vy + vz*vz);
        inv_check(inv, speed_final > ball_speed0,
                  "slope_friction ball gains speed: initial=%.6f final=%.6f", ball_speed0, speed_final);
    }

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, ramp_surface);
    return 0;
}

static int run_springs(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    /* Reduced gravity for springs scenario */
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0));

    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    /* --- Ground: static box 15×0.5×15 --- */
    ivp_compact_surface_t *ground_surf = NULL;
    ivp_surman_polygon_t ground_sm;
    if (!scenario_init_box_surman(15.0f, 0.5f, 15.0f, &ground_surf, &ground_sm)) {
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    if (!ground) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Pillar L: static box 0.5×2×0.5 at (-6,-7,0) --- */
    ivp_compact_surface_t *pillar_surf[2] = { NULL, NULL };
    ivp_surman_polygon_t pillar_sm[2];

    ivp_vec3r_set(&tmpl.position, IVP_REAL(-6.0), IVP_REAL(-7.0), IVP_REAL(0.0));
    ivp_object_t *pillar_l = scenario_create_box(env, &tmpl, 0.5f, 2.0f, 0.5f, &pillar_surf[0], &pillar_sm[0]);
    if (!pillar_l) {
        ivp_environment_destroy(env);
        scenario_free_surfaces(pillar_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Pillar R: static box 0.5×2×0.5 at (6,-7,0) --- */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(6.0), IVP_REAL(-7.0), IVP_REAL(0.0));
    ivp_object_t *pillar_r = scenario_create_box(env, &tmpl, 0.5f, 2.0f, 0.5f, &pillar_surf[1], &pillar_sm[1]);
    if (!pillar_r) {
        ivp_environment_destroy(env);
        scenario_free_surfaces(pillar_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Bridge nodes: 4 dynamic boxes 0.5×0.5×0.5 --- */
    #define N_BRIDGE 4
    ivp_real_t node_spacing = IVP_REAL(12.0) / ((ivp_real_t)N_BRIDGE + IVP_REAL(1.0));
    /* One compact surface per node, as the reference runner (create_dynamic_box
     * builds one per call): with a shared ledge the minimizer's loop hash, keyed
     * on edge addresses, mistakes situations of two nodes for a loop. */
    ivp_compact_surface_t *node_surf[N_BRIDGE] = { NULL };
    ivp_surman_polygon_t node_sm[N_BRIDGE];

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(0.6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.07), IVP_REAL(0.07), IVP_REAL(0.07));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.7);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor   = IVP_REAL(0.15);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *nodes[N_BRIDGE];
    for (int i = 0; i < N_BRIDGE; i++) {
        ivp_real_t x = IVP_REAL(-6.0) + node_spacing * ((ivp_real_t)i + IVP_REAL(1.0));
        ivp_vec3r_set(&tmpl.position, x, IVP_REAL(-7.0), IVP_REAL(0.0));
        nodes[i] = NULL;
        if (scenario_init_box_surman(0.5f, 0.5f, 0.5f, &node_surf[i], &node_sm[i])) {
            nodes[i] = ivp_environment_create_object(env, &tmpl, &node_sm[i].base);
        }
        if (!nodes[i]) {
            ivp_environment_destroy(env);
            scenario_free_surfaces(node_surf, N_BRIDGE);
            scenario_free_surfaces(pillar_surf, 2);
            ivp_compact_surface_free(NULL, ground_surf);
            return 1;
        }
    }

    /* --- Ball: dynamic sphere r=0.8 at (0,-10,0) --- */
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(0.5);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.48), IVP_REAL(0.48), IVP_REAL(0.48));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-10.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_surman_ball_t ball_sm;
    ivp_surman_ball_init(&ball_sm, 0.8f);

    tmpl.ball_radius = IVP_REAL(0.8);
    ivp_object_t *ball = ivp_environment_create_object(env, &tmpl, &ball_sm.base);
    if (!ball) {
        ivp_environment_destroy(env);
        scenario_free_surfaces(node_surf, N_BRIDGE);
        scenario_free_surfaces(pillar_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Bungee: dynamic box 0.6×0.6×0.6 at (10,-6,0) --- */
    ivp_compact_surface_t *bungee_surf = NULL;
    ivp_surman_polygon_t bungee_sm;
    if (!scenario_init_box_surman(0.6f, 0.6f, 0.6f, &bungee_surf, &bungee_sm)) {
        ivp_environment_destroy(env);
        scenario_free_surfaces(node_surf, N_BRIDGE);
        scenario_free_surfaces(pillar_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(3.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.36), IVP_REAL(0.36), IVP_REAL(0.36));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.6);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(10.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *bungee = ivp_environment_create_object(env, &tmpl, &bungee_sm.base);
    if (!bungee) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, bungee_surf);
        scenario_free_surfaces(node_surf, N_BRIDGE);
        scenario_free_surfaces(pillar_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Anchor: static box 0.3×0.3×0.3 at (10,-14,0) --- */
    ivp_compact_surface_t *anchor_surf = NULL;
    ivp_surman_polygon_t anchor_sm;
    if (!scenario_init_box_surman(0.3f, 0.3f, 0.3f, &anchor_surf, &anchor_sm)) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, bungee_surf);
        scenario_free_surfaces(node_surf, N_BRIDGE);
        scenario_free_surfaces(pillar_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(10.0), IVP_REAL(-14.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *anchor = ivp_environment_create_object(env, &tmpl, &anchor_sm.base);
    if (!anchor) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, anchor_surf);
        ivp_compact_surface_free(NULL, bungee_surf);
        scenario_free_surfaces(node_surf, N_BRIDGE);
        scenario_free_surfaces(pillar_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Springs (matching example_springs.c) --- */
    #define MAX_SPRINGS 16
    ivp_spring_state_t springs[MAX_SPRINGS];
    int n_springs = 0;

    ivp_real_t bridge_rest_len = node_spacing - IVP_REAL(1.0);
    if (bridge_rest_len < IVP_REAL(0.3)) bridge_rest_len = IVP_REAL(0.3);

    /* pillar_l → node[0] */
    {
        ivp_vec3r_t a_pt = ivp_vec3r(IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t b_pt = ivp_vec3r(IVP_REAL(-0.5), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_spring_state_init(&springs[n_springs]);
        ivp_spring_state_setup(&springs[n_springs],
            pillar_l, &a_pt, nodes[0], &b_pt,
            IVP_REAL(120.0), IVP_REAL(30.0), bridge_rest_len);
        /* reference IVP_Actuator_Spring is a dependent controller */
        ivp_environment_announce_dependent_controller(env, 100, ivp_controller_spring,
                                                      &springs[n_springs], ivp_spring_discovery);
        n_springs++;
    }

    /* node[i-1] → node[i] */
    for (int i = 1; i < N_BRIDGE; i++) {
        ivp_vec3r_t a_pt = ivp_vec3r(IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t b_pt = ivp_vec3r(IVP_REAL(-0.5), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_spring_state_init(&springs[n_springs]);
        ivp_spring_state_setup(&springs[n_springs],
            nodes[i-1], &a_pt, nodes[i], &b_pt,
            IVP_REAL(120.0), IVP_REAL(30.0), bridge_rest_len);
        /* reference IVP_Actuator_Spring is a dependent controller */
        ivp_environment_announce_dependent_controller(env, 100, ivp_controller_spring,
                                                      &springs[n_springs], ivp_spring_discovery);
        n_springs++;
    }

    /* node[3] → pillar_r */
    {
        ivp_vec3r_t a_pt = ivp_vec3r(IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t b_pt = ivp_vec3r(IVP_REAL(-0.5), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_spring_state_init(&springs[n_springs]);
        ivp_spring_state_setup(&springs[n_springs],
            nodes[N_BRIDGE-1], &a_pt, pillar_r, &b_pt,
            IVP_REAL(120.0), IVP_REAL(30.0), bridge_rest_len);
        /* reference IVP_Actuator_Spring is a dependent controller */
        ivp_environment_announce_dependent_controller(env, 100, ivp_controller_spring,
                                                      &springs[n_springs], ivp_spring_discovery);
        n_springs++;
    }

    /* Bungee spring: anchor → bungee */
    {
        ivp_vec3r_t a_pt = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.3), IVP_REAL(0.0));
        ivp_vec3r_t b_pt = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.6), IVP_REAL(0.0));
        ivp_spring_state_init(&springs[n_springs]);
        ivp_spring_state_setup(&springs[n_springs],
            anchor, &a_pt, bungee, &b_pt,
            IVP_REAL(8.0), IVP_REAL(6.0), IVP_REAL(8.0));
        /* reference IVP_Actuator_Spring is a dependent controller */
        ivp_environment_announce_dependent_controller(env, 100, ivp_controller_spring,
                                                      &springs[n_springs], ivp_spring_discovery);
        n_springs++;
    }
    #undef MAX_SPRINGS

    /* --- Track objects --- */
    ivp_object_t *objects[10] = { ground, pillar_l, pillar_r, nodes[0], nodes[1], nodes[2], nodes[3], ball, bungee, anchor };
    const char   *types[10]   = { "ground", "pillar", "pillar", "node", "node", "node", "node", "ball", "bungee", "anchor" };

    snapshot_frame(out, SCENARIO_SPRINGS, 0, dt, env, objects, types, 10);
    for (int step = 1; step <= steps; step++) {
        scenario_step(env, dt);
        snapshot_frame(out, SCENARIO_SPRINGS, step, dt, env, objects, types, 10);
    }

    if (inv) {
        /* Total energy of dynamic objects bounded (< 1000) */
        double total_energy = 0.0;
        for (int i = 0; i < 10; i++) {
            total_energy += (double)ivp_object_get_energy(objects[i]);
        }
        inv_check(inv, total_energy < 1000.0,
                  "springs total energy bounded: got %.6f (limit 1000)", total_energy);

        /* Bridge nodes stay between pillars (x in [-8, 8]) */
        for (int i = 0; i < N_BRIDGE; i++) {
            double nx = (double)nodes[i]->physical_core->pos_world_f_core_last_psi.k[0];
            inv_check(inv, nx > -8.0 && nx < 8.0,
                      "springs node[%d] x in bounds: got %.6f (range [-8,8])", i, nx);
        }
    }

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, anchor_surf);
    ivp_compact_surface_free(NULL, bungee_surf);
    scenario_free_surfaces(node_surf, N_BRIDGE);
    scenario_free_surfaces(pillar_surf, 2);
    ivp_compact_surface_free(NULL, ground_surf);
    return 0;
    #undef N_BRIDGE
}

static int run_rope(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));

    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    /* --- Ground: static box 12×0.5×12 at (0,2,0) --- */
    ivp_compact_surface_t *ground_surf = NULL;
    ivp_surman_polygon_t ground_sm;
    if (!scenario_init_box_surman(12.0f, 0.5f, 12.0f, &ground_surf, &ground_sm)) {
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    if (!ground) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Ceiling: static box 1.5×0.3×0.3 at (0,-15,0) --- */
    ivp_compact_surface_t *ceiling_surf = NULL;
    ivp_surman_polygon_t ceiling_sm;
    if (!scenario_init_box_surman(1.5f, 0.3f, 0.3f, &ceiling_surf, &ceiling_sm)) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-15.0), IVP_REAL(0.0));
    ivp_object_t *ceiling = ivp_environment_create_object(env, &tmpl, &ceiling_sm.base);
    if (!ceiling) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, ceiling_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Chain links: 8 dynamic boxes 0.2×0.55×0.2 --- */
    #define CHAIN_LEN 8
    ivp_real_t step_y = IVP_REAL(2.0) * IVP_REAL(0.55) + IVP_REAL(0.05); /* 1.15 */
    ivp_compact_surface_t *link_surf[CHAIN_LEN] = { NULL };
    ivp_surman_polygon_t link_sm[CHAIN_LEN];

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(0.6);
    /* I = m/3 * (hy^2 + hz^2), I = m/3 * (hx^2 + hz^2), I = m/3 * (hx^2 + hy^2) */
    ivp_real_t m = IVP_REAL(0.6);
    ivp_real_t hx = IVP_REAL(0.2), hy = IVP_REAL(0.55), hz = IVP_REAL(0.2);
    ivp_real_t ix = m / IVP_REAL(3.0) * (hy*hy + hz*hz);
    ivp_real_t iy = m / IVP_REAL(3.0) * (hx*hx + hz*hz);
    ivp_real_t iz = m / IVP_REAL(3.0) * (hx*hx + hy*hy);
    ivp_vec3r_set(&tmpl.rot_inertia, ix, iy, iz);
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.6);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor   = IVP_REAL(0.03);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *links[CHAIN_LEN];
    for (int i = 0; i < CHAIN_LEN; i++) {
        ivp_real_t y = IVP_REAL(-15.0) + step_y * ((ivp_real_t)i + IVP_REAL(1.0));
        ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), y, IVP_REAL(0.0));
        links[i] = scenario_create_box(env, &tmpl, 0.2f, 0.55f, 0.2f, &link_surf[i], &link_sm[i]);
        if (!links[i]) {
            ivp_environment_destroy(env);
            scenario_free_surfaces(link_surf, CHAIN_LEN);
            ivp_compact_surface_free(NULL, ceiling_surf);
            ivp_compact_surface_free(NULL, ground_surf);
            return 1;
        }
    }

    /* --- Weight: dynamic box 0.6×0.6×0.6 at chain end --- */
    ivp_compact_surface_t *weight_surf = NULL;
    ivp_surman_polygon_t weight_sm;
    if (!scenario_init_box_surman(0.6f, 0.6f, 0.6f, &weight_surf, &weight_sm)) {
        ivp_environment_destroy(env);
        scenario_free_surfaces(link_surf, CHAIN_LEN);
        ivp_compact_surface_free(NULL, ceiling_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(2.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.48), IVP_REAL(0.48), IVP_REAL(0.48));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.1);
    tmpl.speed_damp_factor   = IVP_REAL(0.02);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    ivp_real_t weight_y = IVP_REAL(-15.0) + step_y * ((ivp_real_t)CHAIN_LEN + IVP_REAL(1.0));
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), weight_y, IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *weight = ivp_environment_create_object(env, &tmpl, &weight_sm.base);
    if (!weight) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, weight_surf);
        scenario_free_surfaces(link_surf, CHAIN_LEN);
        ivp_compact_surface_free(NULL, ceiling_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Ball-socket constraints (reference runner
     * add_ballsocket_constraint_two_anchors: set_ballsocket_ws at the
     * reference object's anchor, attached object relaxed; announced as a
     * dependent controller like IVP_Constraint) --- */
    ivp_constraint_state_t constraints[CHAIN_LEN + 1];
    memset(constraints, 0, sizeof(constraints));
    {
        ivp_object_t *refs[CHAIN_LEN + 1];
        ivp_object_t *atts[CHAIN_LEN + 1];
        ivp_vec3r_t anchors_r[CHAIN_LEN + 1];
        refs[0] = ceiling; atts[0] = links[0];
        anchors_r[0] = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.3), IVP_REAL(0.0));
        for (int i = 1; i < CHAIN_LEN; i++) {
            refs[i] = links[i - 1]; atts[i] = links[i];
            anchors_r[i] = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.55), IVP_REAL(0.0));
        }
        refs[CHAIN_LEN] = links[CHAIN_LEN - 1]; atts[CHAIN_LEN] = weight;
        anchors_r[CHAIN_LEN] = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.55), IVP_REAL(0.0));
        for (int i = 0; i <= CHAIN_LEN; i++) {
            ivp_vec3r_t anchor_ws;
            ivp_transform_point(&anchor_ws, &refs[i]->physical_core->m_world_f_core_last_psi, &anchors_r[i]);
            ivp_constraint_init_ball_socket_ws(&constraints[i], refs[i], &anchor_ws, atts[i]);
            ivp_environment_announce_dependent_controller(env, 50, ivp_controller_constraint,
                                                          &constraints[i], ivp_constraint_discovery);
        }
    }

    /* --- Track objects --- */
    ivp_object_t *objects[11] = { ground, ceiling, links[0], links[1], links[2], links[3], links[4], links[5], links[6], links[7], weight };
    const char   *types[11]   = { "ground", "ceiling", "link", "link", "link", "link", "link", "link", "link", "link", "weight" };

    snapshot_frame(out, SCENARIO_ROPE, 0, dt, env, objects, types, 11);
    for (int step = 1; step <= steps; step++) {
        scenario_step(env, dt);
        snapshot_frame(out, SCENARIO_ROPE, step, dt, env, objects, types, 11);
    }

    if (inv) {
        /* Weight hangs below top link (weight.y > link[0].y in Y-down) */
        double weight_y = (double)weight->physical_core->pos_world_f_core_last_psi.k[1];
        double link0_y  = (double)links[0]->physical_core->pos_world_f_core_last_psi.k[1];
        inv_check(inv, weight_y > link0_y,
                  "rope weight below top link: weight_y=%.6f link0_y=%.6f", weight_y, link0_y);

        /* Total chain energy bounded.
         * Under descending reference-priority dispatch, rope transients are
         * higher; keep a non-divergence envelope instead of legacy 5000 cap. */
        double total_energy = 0.0;
        for (int i = 0; i < 11; i++) {
            total_energy += (double)ivp_object_get_energy(objects[i]);
        }
        inv_check(inv, total_energy < 10000.0,
                  "rope total energy bounded: got %.6f (limit 10000)", total_energy);
    }

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, weight_surf);
    scenario_free_surfaces(link_surf, CHAIN_LEN);
    ivp_compact_surface_free(NULL, ceiling_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    return 0;
    #undef CHAIN_LEN
}

static int run_motor_common(FILE *out, int steps, double dt, invariant_result_t *inv, int scenario_id)
{
    static const char *motor_internal = "motint";

    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));

    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    /* --- Ground: static box 15×0.5×15 --- */
    ivp_compact_surface_t *ground_surf = NULL;
    ivp_surman_polygon_t ground_sm;
    if (!scenario_init_box_surman(15.0f, 0.5f, 15.0f, &ground_surf, &ground_sm)) {
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    if (!ground) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Pedestal: static box 0.15×0.8×0.15 at (0,-5.6,0) --- */
    ivp_compact_surface_t *pedestal_surf = NULL;
    ivp_surman_polygon_t pedestal_sm;
    if (!scenario_init_box_surman(0.15f, 0.8f, 0.15f, &pedestal_surf, &pedestal_sm)) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-5.6), IVP_REAL(0.0));
    tmpl.no_collision_group = motor_internal;
    ivp_object_t *pedestal = ivp_environment_create_object(env, &tmpl, &pedestal_sm.base);
    if (!pedestal) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, pedestal_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }
    tmpl.no_collision_group = NULL;

    /* --- Blade: dynamic box 4.0×0.25×0.4 at (0,-6.8,0) --- */
    ivp_compact_surface_t *blade_surf = NULL;
    ivp_surman_polygon_t blade_sm;
    if (!scenario_init_box_surman(4.0f, 0.25f, 0.4f, &blade_surf, &blade_sm)) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, pedestal_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1.2);
    /* I = m/3 * (hy^2 + hz^2), etc. for hx=4.0, hy=0.25, hz=0.4 */
    ivp_real_t m_blade = IVP_REAL(1.2);
    ivp_real_t hx_blade = IVP_REAL(4.0), hy_blade = IVP_REAL(0.25), hz_blade = IVP_REAL(0.4);
    ivp_real_t ix_blade = m_blade / IVP_REAL(3.0) * (hy_blade*hy_blade + hz_blade*hz_blade);
    ivp_real_t iy_blade = m_blade / IVP_REAL(3.0) * (hx_blade*hx_blade + hz_blade*hz_blade);
    ivp_real_t iz_blade = m_blade / IVP_REAL(3.0) * (hx_blade*hx_blade + hy_blade*hy_blade);
    ivp_vec3r_set(&tmpl.rot_inertia, ix_blade, iy_blade, iz_blade);
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.35);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-6.8), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();
    tmpl.no_collision_group = motor_internal;

    ivp_object_t *blade = ivp_environment_create_object(env, &tmpl, &blade_sm.base);
    if (!blade) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, blade_surf);
        ivp_compact_surface_free(NULL, pedestal_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }
    tmpl.no_collision_group = NULL;

    /* --- Targets: 8 dynamic boxes 0.4×0.4×0.4 positioned in circle --- */
    #define N_TARGETS 8
    ivp_real_t radius = IVP_REAL(3.5);
    /* Start the targets just above the blade (blade spans y -7.05..-6.55,
     * targets are 0.8 m tall) so nothing is spawned intersecting the blade.
     * The two on-axis targets drop onto the blade and are carried away by
     * it; the others fall clear.  Spawning them inside the blade (the old
     * y = -6.4) made the outcome depend on how an invalid initial overlap
     * happens to be resolved. */
    ivp_real_t target_y = IVP_REAL(-7.9);
    ivp_compact_surface_t *target_surf[N_TARGETS] = { NULL };
    ivp_surman_polygon_t target_sm[N_TARGETS];

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(0.06);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.05), IVP_REAL(0.05), IVP_REAL(0.05));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.2);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *targets[N_TARGETS];
    for (int i = 0; i < N_TARGETS; i++) {
        ivp_real_t angle = IVP_REAL(2.0) * IVP_REAL(3.14159265358979323846) * (ivp_real_t)i / (ivp_real_t)N_TARGETS;
        ivp_real_t x = radius * ivp_cos(angle);
        ivp_real_t z = radius * ivp_sin(angle);
        ivp_vec3r_set(&tmpl.position, x, target_y, z);
        targets[i] = scenario_create_box(env, &tmpl, 0.4f, 0.4f, 0.4f, &target_surf[i], &target_sm[i]);
        if (!targets[i]) {
            ivp_environment_destroy(env);
            scenario_free_surfaces(target_surf, N_TARGETS);
            ivp_compact_surface_free(NULL, blade_surf);
            ivp_compact_surface_free(NULL, pedestal_surf);
            ivp_compact_surface_free(NULL, ground_surf);
            return 1;
        }
    }

    /* --- Hinge constraint: pedestal→blade (matching example_motor.c) --- */
    ivp_constraint_state_t pivot;
    {
        ivp_vec3r_t anchor_ws = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-6.8), IVP_REAL(0.0));
        ivp_vec3r_t axis_ws   = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
        ivp_constraint_init_hinge_ws(&pivot, pedestal, &anchor_ws, &axis_ws, blade);
        pivot.tau     = IVP_REAL(0.25);
        pivot.damping = IVP_REAL(0.95);
    }
    ivp_environment_announce_dependent_controller(env, 50, ivp_controller_constraint, &pivot,
                                                  ivp_constraint_discovery);

    /* --- Rotation motor: pedestal→blade (matching example_motor.c) --- */
    ivp_rot_mot_state_t motor;
    ivp_rot_mot_state_init(&motor);
    {
        ivp_vec3r_t a0 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t a1 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
        ivp_rot_mot_state_setup(&motor,
            blade,    &a0,
            blade,    &a1,
            IVP_REAL(3.0),    /* power */
            IVP_REAL(4.0),    /* max torque */
            IVP_REAL(1.6));   /* max speed (rad/s) */
    }
    ivp_environment_announce_dependent_controller(env, 100, ivp_controller_rot_mot, &motor,
                                                  ivp_rot_mot_discovery);

    /* check_distance variant: blade tip vs. a pedestal point at the tip's
     * initial position; range 1 m (inside at start, outside once the blade
     * turned ~0.25 rad). */
    ivp_check_dist_state_t check;
    ivp_check_dist_state_init(&check);
    g_check_dist_event_count = 0;
    if (scenario_id == SCENARIO_CHECK_DISTANCE) {
        ivp_vec3r_t tip_ws = ivp_vec3r(IVP_REAL(4.0), IVP_REAL(-6.8), IVP_REAL(0.0));
        ivp_check_dist_state_setup(&check, blade, &tip_ws, pedestal, &tip_ws, IVP_REAL(1.0));
        ivp_check_dist_set_callback(&check, check_dist_counter_cb, NULL);
        ivp_environment_add_controller(env, 100, ivp_controller_check_dist, &check);
    }

    /* --- Track objects --- */
    ivp_object_t *objects[11] = { ground, pedestal, blade, targets[0], targets[1], targets[2], targets[3], targets[4], targets[5], targets[6], targets[7] };
    const char   *types[11]   = { "ground", "pedestal", "blade", "target", "target", "target", "target", "target", "target", "target", "target" };

    /* Record initial target positions for invariant checks */
    double target_init_x[N_TARGETS], target_init_z[N_TARGETS];
    for (int i = 0; i < N_TARGETS; i++) {
        target_init_x[i] = (double)targets[i]->physical_core->pos_world_f_core_last_psi.k[0];
        target_init_z[i] = (double)targets[i]->physical_core->pos_world_f_core_last_psi.k[2];
    }

    snapshot_frame(out, scenario_id, 0, dt, env, objects, types, 11);
    for (int step = 1; step <= steps; step++) {
        scenario_step(env, dt);
        snapshot_frame(out, scenario_id, step, dt, env, objects, types, 11);
    }

    if (inv) {
        /* Blade rotating at final step (|ang_vel_y| > 0.1) */
        double blade_ang_vel_y = (double)blade->physical_core->rot_speed.k[1];
        inv_check(inv, fabs(blade_ang_vel_y) > 0.1,
                  "motor blade rotating: |ang_vel_y|=%.6g (need > 0.1)", fabs(blade_ang_vel_y));

        /* At least 2 targets displaced from initial position */
        int displaced = 0;
        for (int i = 0; i < N_TARGETS; i++) {
            double tx = (double)targets[i]->physical_core->pos_world_f_core_last_psi.k[0];
            double tz = (double)targets[i]->physical_core->pos_world_f_core_last_psi.k[2];
            double dx = tx - target_init_x[i];
            double dz = tz - target_init_z[i];
            if (sqrt(dx*dx + dz*dz) > 0.1)
                displaced++;
        }
        inv_check(inv, displaced >= 2,
                  "motor targets displaced: %d targets moved (need >= 2)", displaced);
        if (scenario_id == SCENARIO_CHECK_DISTANCE) {
            inv_check(inv, g_check_dist_event_count > 0 && check.is_outside == 1,
                      "check_distance fired (count=%d) and blade tip ended outside (is_outside=%d)",
                      g_check_dist_event_count, check.is_outside);
        }
    }

    ivp_environment_destroy(env);
    scenario_free_surfaces(target_surf, N_TARGETS);
    ivp_compact_surface_free(NULL, blade_surf);
    ivp_compact_surface_free(NULL, pedestal_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    return 0;
    #undef N_TARGETS
}

static int run_motor(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    return run_motor_common(out, steps, dt, inv, SCENARIO_MOTOR);
}

static int run_buoyancy(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));

    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    /* --- Ground: static box 15×0.5×15 at (0,5,0) --- */
    ivp_compact_surface_t *ground_surf = ref_box_surface(15.0f, 0.5f, 15.0f);
    if (!ground_surf) { ivp_environment_destroy(env); return 1; }
    ivp_surman_polygon_t ground_sm;
    ivp_surman_polygon_init(&ground_sm, ground_surf);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(5.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    if (!ground) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Wall L: static box 0.3×6×15 at (-12,0,0) --- */
    ivp_compact_surface_t *wall_surf[2] = { NULL, NULL };
    ivp_surman_polygon_t wall_sm[2];

    ivp_vec3r_set(&tmpl.position, IVP_REAL(-12.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *wall_l = scenario_create_box(env, &tmpl, 0.3f, 6.0f, 15.0f, &wall_surf[0], &wall_sm[0]);
    if (!wall_l) {
        ivp_environment_destroy(env);
        scenario_free_surfaces(wall_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Wall R: static box 0.3×6×15 at (12,0,0) --- */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(12.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *wall_r = scenario_create_box(env, &tmpl, 0.3f, 6.0f, 15.0f, &wall_surf[1], &wall_sm[1]);
    if (!wall_r) {
        ivp_environment_destroy(env);
        scenario_free_surfaces(wall_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Cork: box 0.5×0.5×0.5, mass=0.3 at (-6,-6,0) --- */
    ivp_compact_surface_t *cork_surf = ref_box_surface(0.5f, 0.5f, 0.5f);
    if (!cork_surf) {
        ivp_environment_destroy(env);
        scenario_free_surfaces(wall_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }
    ivp_surman_polygon_t cork_sm;
    ivp_surman_polygon_init(&cork_sm, cork_surf);

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(0.3);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.072), IVP_REAL(0.072), IVP_REAL(0.072));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.2);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-6.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *cork = ivp_environment_create_object(env, &tmpl, &cork_sm.base);
    if (!cork) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, cork_surf);
        scenario_free_surfaces(wall_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Wood: box 0.6×0.4×0.6, mass=1.0 at (-3,-5,0) --- */
    ivp_compact_surface_t *wood_surf = ref_box_surface(0.6f, 0.4f, 0.6f);
    if (!wood_surf) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, cork_surf);
        scenario_free_surfaces(wall_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }
    ivp_surman_polygon_t wood_sm;
    ivp_surman_polygon_init(&wood_sm, wood_surf);

    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.24), IVP_REAL(0.24), IVP_REAL(0.24));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-3.0), IVP_REAL(-5.0), IVP_REAL(0.0));

    ivp_object_t *wood = ivp_environment_create_object(env, &tmpl, &wood_sm.base);
    if (!wood) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        scenario_free_surfaces(wall_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Ball: sphere r=0.55, mass=1.5 at (0,-7,0) --- */
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.5);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.36), IVP_REAL(0.36), IVP_REAL(0.36));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.2);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-7.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_surman_ball_t ball_sm;
    ivp_surman_ball_init(&ball_sm, 0.55f);

    tmpl.ball_radius = IVP_REAL(0.55);
    ivp_object_t *ball = ivp_environment_create_object(env, &tmpl, &ball_sm.base);
    if (!ball) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        scenario_free_surfaces(wall_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Steel: box 0.5×0.5×0.5, mass=8.0 at (3,-4,0) --- */
    ivp_compact_surface_t *steel_surf = ref_box_surface(0.5f, 0.5f, 0.5f);
    if (!steel_surf) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        scenario_free_surfaces(wall_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }
    ivp_surman_polygon_t steel_sm;
    ivp_surman_polygon_init(&steel_sm, steel_surf);

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(8.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1.92), IVP_REAL(1.92), IVP_REAL(1.92));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction   = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.2);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(3.0), IVP_REAL(-4.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *steel = ivp_environment_create_object(env, &tmpl, &steel_sm.base);
    if (!steel) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, steel_surf);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        scenario_free_surfaces(wall_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Brick: box 0.7×0.35×0.5, mass=3.0 at (6,-6,0) --- */
    ivp_compact_surface_t *brick_surf = ref_box_surface(0.7f, 0.35f, 0.5f);
    if (!brick_surf) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, steel_surf);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        scenario_free_surfaces(wall_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }
    ivp_surman_polygon_t brick_sm;
    ivp_surman_polygon_init(&brick_sm, brick_surf);

    tmpl.mass = IVP_REAL(3.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.72), IVP_REAL(0.72), IVP_REAL(0.72));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(6.0), IVP_REAL(-6.0), IVP_REAL(0.0));

    ivp_object_t *brick = ivp_environment_create_object(env, &tmpl, &brick_sm.base);
    if (!brick) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, brick_surf);
        ivp_compact_surface_free(NULL, steel_surf);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        scenario_free_surfaces(wall_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Foam: box 0.8×0.8×0.8, mass=0.2 at (-4.5,-9,3) --- */
    ivp_compact_surface_t *foam_surf = ref_box_surface(0.8f, 0.8f, 0.8f);
    if (!foam_surf) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, brick_surf);
        ivp_compact_surface_free(NULL, steel_surf);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        scenario_free_surfaces(wall_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }
    ivp_surman_polygon_t foam_sm;
    ivp_surman_polygon_init(&foam_sm, foam_surf);

    tmpl.mass = IVP_REAL(0.2);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.048), IVP_REAL(0.048), IVP_REAL(0.048));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-4.5), IVP_REAL(-9.0), IVP_REAL(3.0));

    ivp_object_t *foam = ivp_environment_create_object(env, &tmpl, &foam_sm.base);
    if (!foam) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, foam_surf);
        ivp_compact_surface_free(NULL, brick_surf);
        ivp_compact_surface_free(NULL, steel_surf);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        scenario_free_surfaces(wall_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Lead: box 0.4×0.4×0.4, mass=12.0 at (4.5,-8,-3) --- */
    ivp_compact_surface_t *lead_surf = ref_box_surface(0.4f, 0.4f, 0.4f);
    if (!lead_surf) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, foam_surf);
        ivp_compact_surface_free(NULL, brick_surf);
        ivp_compact_surface_free(NULL, steel_surf);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        scenario_free_surfaces(wall_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }
    ivp_surman_polygon_t lead_sm;
    ivp_surman_polygon_init(&lead_sm, lead_surf);

    tmpl.mass = IVP_REAL(12.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(2.88), IVP_REAL(2.88), IVP_REAL(2.88));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(4.5), IVP_REAL(-8.0), IVP_REAL(-3.0));

    ivp_object_t *lead = ivp_environment_create_object(env, &tmpl, &lead_sm.base);
    if (!lead) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, lead_surf);
        ivp_compact_surface_free(NULL, foam_surf);
        ivp_compact_surface_free(NULL, brick_surf);
        ivp_compact_surface_free(NULL, steel_surf);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        scenario_free_surfaces(wall_surf, 2);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* --- Buoyancy (reference runner setup_buoyancy): IVP_Template_Buoyancy
     * with pressure_damp 3.0, friction_damp 0.0, ball_rot_dampening 2.0,
     * viscosity 0.0, no interpolation; liquid surface hesse (0,1,0,1), i.e.
     * liquid for y > -1, current (1.5,0,0); attached to the 7 dynamic cores.
     * medium_density is 1.0: the reference runner originally used 1000 with
     * object densities of 0.05..23 kg/m^3, which launched every object out
     * of the liquid (the reference diverged by itself); with 1.0 cork, wood
     * and foam float while ball, steel, brick and lead sink as the scene
     * names intend.  The reference runner was changed identically. --- */
    static ivp_buoyancy_state_t buoy;
    ivp_buoyancy_init(&buoy, IVP_REAL(-1.0), IVP_REAL(1.0));
    buoy.linear_drag            = IVP_REAL(3.0);  /* pressure_damp_factor */
    buoy.friction_damp_factor   = IVP_REAL(0.0);
    buoy.angular_drag           = IVP_REAL(2.0);  /* ball_rot_dampening_factor */
    buoy.viscosity_factor       = IVP_REAL(0.0);
    buoy.use_interpolation      = false;
    ivp_vec3r_set(&buoy.flow_velocity, IVP_REAL(1.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *buoy_objects[7] = { cork, wood, ball, steel, brick, foam, lead };
    ivp_buoyancy_set_objects(&buoy, buoy_objects, 7);
    ivp_environment_add_controller(env, 200, ivp_controller_buoyancy, &buoy);

    /* --- Track objects --- */
    ivp_object_t *objects[10] = { ground, wall_l, wall_r, cork, wood, ball, steel, brick, foam, lead };
    const char   *types[10]   = { "ground", "wall", "wall", "cork", "wood", "ball", "steel", "brick", "foam", "lead" };

    snapshot_frame(out, SCENARIO_BUOYANCY, 0, dt, env, objects, types, 10);
    for (int step = 1; step <= steps; step++) {
        scenario_step(env, dt);
        snapshot_frame(out, SCENARIO_BUOYANCY, step, dt, env, objects, types, 10);
    }

    if (inv) {
        /* Heavy objects (steel id=6, lead id=9) sink: y > -3.0 in Y-down */
        double steel_y = (double)steel->physical_core->pos_world_f_core_last_psi.k[1];
        double lead_y  = (double)lead->physical_core->pos_world_f_core_last_psi.k[1];
        inv_check(inv, steel_y > -3.0,
                  "buoyancy steel sinks: y=%.6f (need > -3.0 in Y-down)", steel_y);
        inv_check(inv, lead_y > -3.0,
                  "buoyancy lead sinks: y=%.6f (need > -3.0 in Y-down)", lead_y);

        /* Energy doesn't grow unboundedly: total KE < 10000 */
        double total_energy = 0.0;
        for (int i = 0; i < 10; i++) {
            total_energy += (double)ivp_object_get_energy(objects[i]);
        }
        inv_check(inv, total_energy < 10000.0,
                  "buoyancy energy bounded: got %.6f (limit 10000)", total_energy);
    }

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, lead_surf);
    ivp_compact_surface_free(NULL, foam_surf);
    ivp_compact_surface_free(NULL, brick_surf);
    ivp_compact_surface_free(NULL, steel_surf);
    ivp_compact_surface_free(NULL, wood_surf);
    ivp_compact_surface_free(NULL, cork_surf);
    scenario_free_surfaces(wall_surf, 2);
    ivp_compact_surface_free(NULL, ground_surf);
    return 0;
}

static int run_cubes(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    enum { N_CUBES = 16, N_OBJECTS = 17 };

    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *ground_surf = NULL;
    ivp_compact_surface_t *cube_surf[N_CUBES] = { NULL };
    ivp_surman_polygon_t ground_sm, cube_sm[N_CUBES];

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));

    ivp_object_t *objects[N_OBJECTS] = {0};
    const char *types[N_OBJECTS] = {0};
    objects[0] = scenario_create_box(env, &tmpl, 20.0f, 0.5f, 20.0f, &ground_surf, &ground_sm);
    types[0] = "ground";
    if (!objects[0]) {
        ivp_environment_destroy(env);
        scenario_free_surfaces(cube_surf, N_CUBES);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(0.6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.6);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor = IVP_REAL(0.02);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();

    int idx = 1;
    for (int y = 0; y < 4; y++) {
        for (int x = 0; x < 4; x++) {
            ivp_vec3r_set(&tmpl.position,
                          IVP_REAL(-2.25 + x * 1.5),
                          IVP_REAL(-2.0 - y * 1.1),
                          IVP_REAL(0.0));
            objects[idx] = scenario_create_box(env, &tmpl, 0.5f, 0.5f, 0.5f, &cube_surf[idx - 1], &cube_sm[idx - 1]);
            types[idx] = "cube";
            if (!objects[idx]) {
                ivp_environment_destroy(env);
                scenario_free_surfaces(cube_surf, N_CUBES);
                ivp_compact_surface_free(NULL, ground_surf);
                return 1;
            }
            idx++;
        }
    }

    const double first_cube_y0 = (double)objects[1]->physical_core->pos_world_f_core_last_psi.k[1];

    snapshot_frame(out, SCENARIO_CUBES, 0, dt, env, objects, types, N_OBJECTS);
    for (int step = 1; step <= steps; step++) {
        scenario_step(env, dt);
        snapshot_frame(out, SCENARIO_CUBES, step, dt, env, objects, types, N_OBJECTS);
    }

    if (inv) {
        const double first_cube_y = (double)objects[1]->physical_core->pos_world_f_core_last_psi.k[1];
        inv_check(inv, first_cube_y > first_cube_y0,
                  "cubes first cube falls: y0=%.6f y=%.6f", first_cube_y0, first_cube_y);
    }

    ivp_environment_destroy(env);
    scenario_free_surfaces(cube_surf, N_CUBES);
    ivp_compact_surface_free(NULL, ground_surf);
    return 0;
}

static int run_force_actuator(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *ground_surf = NULL;
    ivp_compact_surface_t *box_surf = NULL;
    ivp_surman_polygon_t ground_sm, box_sm;

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    ivp_object_t *ground = scenario_create_box(env, &tmpl, 20.0f, 0.5f, 20.0f, &ground_surf, &ground_sm);
    if (!ground) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, box_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.24), IVP_REAL(0.24), IVP_REAL(0.24));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor = IVP_REAL(0.12);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-2.0), IVP_REAL(0.0));
    ivp_object_t *box = scenario_create_box(env, &tmpl, 0.6f, 0.6f, 0.6f, &box_surf, &box_sm);
    if (!box) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, box_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    ivp_force_actuator_state_t force;
    ivp_force_actuator_state_init(&force);
    ivp_vec3r_t a = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t b = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_force_actuator_state_setup(&force, box, &a, box, &b, IVP_REAL(1.2), true, false);
    ivp_environment_announce_dependent_controller(env, 100, ivp_controller_force_actuator, &force,
                                                  ivp_force_actuator_discovery);

    const double x0 = (double)box->physical_core->pos_world_f_core_last_psi.k[0];
    ivp_object_t *objects[2] = { ground, box };
    const char *types[2] = { "ground", "force_box" };
    snapshot_frame(out, SCENARIO_FORCE_ACTUATOR, 0, dt, env, objects, types, 2);
    for (int step = 1; step <= steps; step++) {
        scenario_step(env, dt);
        snapshot_frame(out, SCENARIO_FORCE_ACTUATOR, step, dt, env, objects, types, 2);
    }

    if (inv) {
        const double x1 = (double)box->physical_core->pos_world_f_core_last_psi.k[0];
        /* Anchors a=(0,0,0), b=(1,0,0): reference pushes A along (a - b) = -X. */
        inv_check(inv, x1 < x0 - 0.05, "force_actuator box moves -X: x0=%.6f x=%.6f", x0, x1);
    }

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, box_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    return 0;
}

static int run_forcefield(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *ground_surf = NULL;
    ivp_compact_surface_t *box_surf = NULL;
    ivp_surman_polygon_t ground_sm, box_sm;

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    ivp_object_t *ground = scenario_create_box(env, &tmpl, 20.0f, 0.5f, 20.0f, &ground_surf, &ground_sm);
    if (!ground) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, box_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.24), IVP_REAL(0.24), IVP_REAL(0.24));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor = IVP_REAL(0.12);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-2.0), IVP_REAL(0.0));
    ivp_object_t *box = scenario_create_box(env, &tmpl, 0.6f, 0.6f, 0.6f, &box_surf, &box_sm);
    if (!box) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, box_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* Reference runner: SCENARIO_FORCEFIELD is setup_force_actuator_like(
     * stronger = TRUE), i.e. an IVP_Actuator_Force with anchors (0,0,0) and
     * (1,0,0) on the box, 2.5 N, push_first_object only (pushes along -X). */
    ivp_force_actuator_state_t field;
    ivp_force_actuator_state_init(&field);
    ivp_vec3r_t fa = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t fb = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_force_actuator_state_setup(&field, box, &fa, box, &fb, IVP_REAL(2.5), true, false);
    ivp_environment_announce_dependent_controller(env, 100, ivp_controller_force_actuator, &field,
                                                  ivp_force_actuator_discovery);

    const double x0 = (double)box->physical_core->pos_world_f_core_last_psi.k[0];
    ivp_object_t *objects[2] = { ground, box };
    const char *types[2] = { "ground", "forcefield_box" };
    snapshot_frame(out, SCENARIO_FORCEFIELD, 0, dt, env, objects, types, 2);
    for (int step = 1; step <= steps; step++) {
        scenario_step(env, dt);
        snapshot_frame(out, SCENARIO_FORCEFIELD, step, dt, env, objects, types, 2);
    }

    if (inv) {
        const double x1 = (double)box->physical_core->pos_world_f_core_last_psi.k[0];
        inv_check(inv, x1 < x0 - 0.1, "forcefield box moves -X: x0=%.6f x=%.6f", x0, x1);
    }

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, box_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    return 0;
}

static int run_stiff_spring(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *anchor_surf[2] = { NULL, NULL };
    ivp_compact_surface_t *dyn_surf[2] = { NULL, NULL };
    ivp_surman_polygon_t anchor_sm[2], dyn_sm[2];

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-4.0), IVP_REAL(-8.0), IVP_REAL(0.0));
    ivp_object_t *anchor_l = scenario_create_box(env, &tmpl, 0.4f, 0.4f, 0.4f, &anchor_surf[0], &anchor_sm[0]);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(4.0), IVP_REAL(-8.0), IVP_REAL(0.0));
    ivp_object_t *anchor_r = scenario_create_box(env, &tmpl, 0.4f, 0.4f, 0.4f, &anchor_surf[1], &anchor_sm[1]);
    if (!anchor_l || !anchor_r) {
        ivp_environment_destroy(env);
        scenario_free_surfaces(dyn_surf, 2);
        scenario_free_surfaces(anchor_surf, 2);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(0.8);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.12), IVP_REAL(0.12), IVP_REAL(0.12));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.6);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();

    ivp_vec3r_set(&tmpl.position, IVP_REAL(-2.0), IVP_REAL(-10.5), IVP_REAL(0.0));
    ivp_object_t *soft = scenario_create_box(env, &tmpl, 0.5f, 0.5f, 0.5f, &dyn_surf[0], &dyn_sm[0]);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(2.0), IVP_REAL(-10.5), IVP_REAL(0.0));
    ivp_object_t *stiff = scenario_create_box(env, &tmpl, 0.5f, 0.5f, 0.5f, &dyn_surf[1], &dyn_sm[1]);
    if (!soft || !stiff) {
        ivp_environment_destroy(env);
        scenario_free_surfaces(dyn_surf, 2);
        scenario_free_surfaces(anchor_surf, 2);
        return 1;
    }

    /* Reference runner setup_stiff_spring uses add_spring(), i.e. the plain
     * IVP_Actuator_Spring (k in N/m), not IVP_Controller_Stiff_Spring
     * (whose constants are dimensionless). */
    ivp_vec3r_t a0 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_spring_state_t spring_soft, spring_stiff;
    ivp_spring_state_setup(&spring_soft,
                           anchor_l, &a0, soft, &a0,
                           IVP_REAL(25.0), IVP_REAL(6.0), IVP_REAL(2.5));
    ivp_spring_state_setup(&spring_stiff,
                           anchor_r, &a0, stiff, &a0,
                           IVP_REAL(320.0), IVP_REAL(45.0), IVP_REAL(2.5));
    ivp_environment_announce_dependent_controller(env, 100, ivp_controller_spring, &spring_soft,
                                                  ivp_spring_discovery);
    ivp_environment_announce_dependent_controller(env, 100, ivp_controller_spring, &spring_stiff,
                                                  ivp_spring_discovery);

    const double soft_y0 = (double)soft->physical_core->pos_world_f_core_last_psi.k[1];
    const double stiff_y0 = (double)stiff->physical_core->pos_world_f_core_last_psi.k[1];

    ivp_object_t *objects[4] = { anchor_l, anchor_r, soft, stiff };
    const char *types[4] = { "anchor", "anchor", "soft", "stiff" };
    snapshot_frame(out, SCENARIO_STIFF_SPRING, 0, dt, env, objects, types, 4);
    for (int step = 1; step <= steps; step++) {
        scenario_step(env, dt);
        snapshot_frame(out, SCENARIO_STIFF_SPRING, step, dt, env, objects, types, 4);
    }

    if (inv) {
        const double soft_delta = fabs((double)soft->physical_core->pos_world_f_core_last_psi.k[1] - soft_y0);
        const double stiff_delta = fabs((double)stiff->physical_core->pos_world_f_core_last_psi.k[1] - stiff_y0);
        inv_check(inv, !isnan(soft_delta) && !isnan(stiff_delta),
                  "stiff_spring finite displacement: soft=%.6f stiff=%.6f",
                  soft_delta, stiff_delta);
        inv_check(inv, soft_delta < 100.0 && stiff_delta < 100.0,
                  "stiff_spring bounded displacement: soft=%.6f stiff=%.6f",
                  soft_delta, stiff_delta);
    }

    ivp_environment_destroy(env);
    scenario_free_surfaces(dyn_surf, 2);
    scenario_free_surfaces(anchor_surf, 2);
    return 0;
}

static int run_check_distance(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    /* Reference runner: SCENARIO_CHECK_DISTANCE runs setup_motor (identical
     * scene).  libivp additionally attaches a check-dist actuator (no physical
     * effect; reference IVP_Actuator_Check_Dist is not a controller) between
     * the blade tip and the pedestal, see run_motor_common. */
    return run_motor_common(out, steps, dt, inv, SCENARIO_CHECK_DISTANCE);
}

static int run_collision_filter(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *ground_surf = ref_box_surface(20.0f, 0.5f, 20.0f);
    if (!ground_surf) {
        ivp_environment_destroy(env);
        return 1;
    }
    ivp_surman_polygon_t ground_sm;
    ivp_surman_polygon_init(&ground_sm, ground_surf);
    ivp_surman_ball_t ball_sm;
    ivp_surman_ball_init(&ball_sm, 0.5f);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    if (!ground) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    static const char *grp = "grpA";
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.6);
    tmpl.material.elasticity = IVP_REAL(0.1);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.no_collision_group = grp;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-3.0), IVP_REAL(-3.0), IVP_REAL(0.0));
    tmpl.ball_radius = IVP_REAL(0.5);
    ivp_object_t *a = ivp_environment_create_object(env, &tmpl, &ball_sm.base);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(3.0), IVP_REAL(-3.0), IVP_REAL(0.0));
    tmpl.ball_radius = IVP_REAL(0.5);
    ivp_object_t *b = ivp_environment_create_object(env, &tmpl, &ball_sm.base);
    if (!a || !b) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }
    ivp_vec3r_set(&a->physical_core->speed, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&b->physical_core->speed, IVP_REAL(-3.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_object_t *objects[3] = { ground, a, b };
    const char *types[3] = { "ground", "filter_ball", "filter_ball" };
    snapshot_frame(out, SCENARIO_COLLISION_FILTER, 0, dt, env, objects, types, 3);
    for (int step = 1; step <= steps; step++) {
        scenario_step(env, dt);
        snapshot_frame(out, SCENARIO_COLLISION_FILTER, step, dt, env, objects, types, 3);
    }

    if (inv) {
        inv_check(inv, a->physical_core->speed.k[0] > IVP_REAL(0.0) && b->physical_core->speed.k[0] < IVP_REAL(0.0),
                  "collision_filter disabled collision keeps directions: vxA=%.6f vxB=%.6f",
                  (double)a->physical_core->speed.k[0], (double)b->physical_core->speed.k[0]);
    }

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, ground_surf);
    return 0;
}

static int run_motion_controller(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *ground_surf = NULL;
    ivp_compact_surface_t *box_surf = NULL;
    ivp_surman_polygon_t ground_sm, box_sm;

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0));
    ivp_object_t *ground = scenario_create_box(env, &tmpl, 20.0f, 0.5f, 20.0f, &ground_surf, &ground_sm);
    if (!ground) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, box_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1.2);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.32), IVP_REAL(0.32), IVP_REAL(0.32));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-6.0), IVP_REAL(-4.0), IVP_REAL(0.0));
    ivp_object_t *box = scenario_create_box(env, &tmpl, 0.7f, 0.7f, 0.7f, &box_surf, &box_sm);
    if (!box) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, box_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    /* Reference runner: IVP_Controller_Motion target-position mode with
     * force_factor 0.8, damp 1.2, torque 0.8, angular damp 1.2,
     * max_translation_force 200, max_torque 80, target (6, -4, 0). */
    ivp_motion_state_t motion;
    ivp_motion_state_init(&motion);
    motion.force_factor = IVP_REAL(0.8);
    motion.damp_factor = IVP_REAL(1.2);
    motion.torque_factor = IVP_REAL(0.8);
    motion.angular_damp_factor = IVP_REAL(1.2);
    ivp_vec3r_set(&motion.max_translation_force, IVP_REAL(200.0), IVP_REAL(200.0), IVP_REAL(200.0));
    ivp_vec3r_set(&motion.max_torque, IVP_REAL(80.0), IVP_REAL(80.0), IVP_REAL(80.0));
    ivp_vec3r_t target_pos = ivp_vec3r(IVP_REAL(6.0), IVP_REAL(-4.0), IVP_REAL(0.0));
    ivp_motion_state_set_target_position(&motion, &target_pos);
    ivp_quatr_t target_q = ivp_quatr_identity();
    ivp_motion_state_set_target_orientation(&motion, &target_q);
    ivp_environment_add_controller(env, 100, ivp_controller_motion, &motion);

    const double x0 = (double)box->physical_core->pos_world_f_core_last_psi.k[0];
    ivp_object_t *objects[2] = { ground, box };
    const char *types[2] = { "ground", "motion_box" };
    snapshot_frame(out, SCENARIO_MOTION_CONTROLLER, 0, dt, env, objects, types, 2);
    for (int step = 1; step <= steps; step++) {
        scenario_step(env, dt);
        snapshot_frame(out, SCENARIO_MOTION_CONTROLLER, step, dt, env, objects, types, 2);
    }

    if (inv) {
        const double x1 = (double)box->physical_core->pos_world_f_core_last_psi.k[0];
        inv_check(inv, x1 > x0 + 0.5,
                  "motion_controller advances toward +X: x0=%.6f x=%.6f",
                  x0, x1);
    }

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, box_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    return 0;
}

static int run_phantom(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *ground_surf = NULL;
    ivp_compact_surface_t *trigger_surf = NULL;
    ivp_surman_polygon_t ground_sm, trigger_sm;
    ivp_surman_ball_t ball_sm;
    ivp_surman_ball_init(&ball_sm, 0.5f);

    static const char *phntm = "phntm";
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    ivp_object_t *ground = scenario_create_box(env, &tmpl, 20.0f, 0.5f, 20.0f, &ground_surf, &ground_sm);
    if (!ground) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, trigger_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    tmpl.no_collision_group = phntm;
    ivp_object_t *trigger = scenario_create_box(env, &tmpl, 2.5f, 2.5f, 2.5f, &trigger_surf, &trigger_sm);
    if (!trigger) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, trigger_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.no_collision_group = phntm;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-9.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    tmpl.ball_radius = IVP_REAL(0.5);
    ivp_object_t *ball = ivp_environment_create_object(env, &tmpl, &ball_sm.base);
    if (!ball) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, trigger_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }
    ivp_vec3r_set(&ball->physical_core->speed, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Reference runner setup_phantom: static trigger box and ball share the
     * no-collision group "phntm", so the ball flies through the trigger.  The
     * reference scene has no phantom controller (its phantom would see no
     * mindist because of the shared group); libivp attaches an object phantom
     * to the trigger with the collision filter ignored, purely for the
     * invariant check (a phantom never applies forces). */
    ivp_phantom_state_t phantom;
    ivp_phantom_template_t ptmpl;
    ivp_phantom_template_init(&ptmpl);
    ptmpl.respect_collision_filter = false;
    ivp_phantom_init_object(&phantom, trigger, &ptmpl, phantom_event_counter_cb, NULL);
    g_phantom_event_count = 0;
    ivp_environment_add_controller(env, 100, ivp_controller_phantom, &phantom);

    ivp_object_t *objects[3] = { ground, trigger, ball };
    const char *types[3] = { "ground", "phantom", "ball" };
    snapshot_frame(out, SCENARIO_PHANTOM, 0, dt, env, objects, types, 3);
    for (int step = 1; step <= steps; step++) {
        scenario_step(env, dt);
        snapshot_frame(out, SCENARIO_PHANTOM, step, dt, env, objects, types, 3);
    }

    if (inv) {
        /* the ball enters the trigger volume and leaves it again
         * (exit after the 0.5 m exit_policy_extra_radius hysteresis) */
        inv_check(inv, g_phantom_event_count >= 2,
                  "phantom enter+exit events fired: count=%d", g_phantom_event_count);
        inv_check(inv, phantom.n_tracked == 0,
                  "phantom empty after the ball left: tracked=%d", phantom.n_tracked);
    }

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, trigger_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    return 0;
}

static int run_vehicle(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *ground_surf = NULL;
    ivp_compact_surface_t *body_surf = NULL;
    ivp_surman_polygon_t ground_sm, body_sm;
    ivp_surman_ball_t wheel_sm;
    ivp_surman_ball_init(&wheel_sm, 0.35f);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.9);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0));
    ivp_object_t *ground = scenario_create_box(env, &tmpl, 30.0f, 0.5f, 30.0f, &ground_surf, &ground_sm);
    if (!ground) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, body_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(2.2);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.9), IVP_REAL(0.9), IVP_REAL(0.9));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.6);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-4.0), IVP_REAL(0.0));
    ivp_object_t *body = scenario_create_box(env, &tmpl, 1.6f, 0.35f, 0.8f, &body_surf, &body_sm);
    if (!body) {
        ivp_environment_destroy(env);
        ivp_compact_surface_free(NULL, body_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        return 1;
    }
    ivp_vec3r_set(&body->physical_core->speed, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(0.3);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.02), IVP_REAL(0.02), IVP_REAL(0.02));
    tmpl.rot_inertia_is_factor = false; /* absolute inertia */
    tmpl.material.friction = IVP_REAL(0.6);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();

    tmpl.ball_radius = IVP_REAL(0.35);
    ivp_object_t *wheels[4] = {0};
    for (int i = 0; i < 4; i++) {
        const ivp_real_t wx = (i < 2) ? IVP_REAL(-1.1) : IVP_REAL(1.1);
        const ivp_real_t wz = (i % 2 == 0) ? IVP_REAL(-0.95) : IVP_REAL(0.95);
        ivp_vec3r_set(&tmpl.position, wx, IVP_REAL(-3.2), wz);
        wheels[i] = ivp_environment_create_object(env, &tmpl, &wheel_sm.base);
        if (!wheels[i]) {
            ivp_environment_destroy(env);
            ivp_compact_surface_free(NULL, body_surf);
            ivp_compact_surface_free(NULL, ground_surf);
            return 1;
        }
    }

    const double body_x0 = (double)body->physical_core->pos_world_f_core_last_psi.k[0];

    ivp_object_t *objects[6] = { ground, wheels[0], wheels[1], wheels[2], wheels[3], body };
    const char *types[6] = { "ground", "wheel", "wheel", "wheel", "wheel", "vehicle_body" };
    snapshot_frame(out, SCENARIO_VEHICLE, 0, dt, env, objects, types, 6);
    for (int step = 1; step <= steps; step++) {
        scenario_step(env, dt);
        snapshot_frame(out, SCENARIO_VEHICLE, step, dt, env, objects, types, 6);
    }

    if (inv) {
        inv_check(inv, body->physical_core->pos_world_f_core_last_psi.k[0] > body_x0,
                  "vehicle body advances: x0=%.6f x=%.6f",
                  body_x0, (double)body->physical_core->pos_world_f_core_last_psi.k[0]);
    }

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, body_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    return 0;
}

/* car_real_wheels: IVP_Car_System_Real_Wheels (constraint car solver,
 * suspensions, wheel torques, extra gravity / down force anchored on the
 * static object) on a static ground box.  Mirrors setup_car_real_wheels in
 * tools/ivp_reference_runner/ref_runner_scenarios_car.cxx (keep in sync):
 * objects are created in the order ground, body, wheels; the controls are
 * applied before simulating steps 181 / 361 / 541. */
static void car_real_wheels_controls(ivp_car_system_t *car, int step)
{
    if (step == 181) { /* t = 1.0 s: rear wheel drive, steer left, down force */
        ivp_car_system_set_wheel_torque(car, IVP_WHEEL_REAR_LEFT, IVP_REAL(700.0));
        ivp_car_system_set_wheel_torque(car, IVP_WHEEL_REAR_RIGHT, IVP_REAL(700.0));
        ivp_car_system_update_body_countertorque(car);
        ivp_car_system_set_steering(car, (ivp_real_t)0.25f);
        ivp_car_system_set_body_downforce(car, IVP_REAL(1500.0));
    }
    if (step == 361) { /* t = 2.0 s: steer the other way, more throttle */
        ivp_car_system_set_wheel_torque(car, IVP_WHEEL_REAR_LEFT, IVP_REAL(900.0));
        ivp_car_system_set_wheel_torque(car, IVP_WHEEL_REAR_RIGHT, IVP_REAL(900.0));
        ivp_car_system_update_body_countertorque(car);
        ivp_car_system_set_steering(car, (ivp_real_t)-0.15f);
    }
    if (step == 541) { /* t = 3.0 s: release throttle, handbrake on the rear wheels */
        ivp_car_system_set_wheel_torque(car, IVP_WHEEL_REAR_LEFT, IVP_REAL(0.0));
        ivp_car_system_set_wheel_torque(car, IVP_WHEEL_REAR_RIGHT, IVP_REAL(0.0));
        ivp_car_system_update_body_countertorque(car);
        ivp_car_system_set_steering(car, IVP_REAL(0.0));
        ivp_car_system_fix_wheel(car, IVP_WHEEL_REAR_LEFT, true);
        ivp_car_system_fix_wheel(car, IVP_WHEEL_REAR_RIGHT, true);
    }
}

static int run_car_real_wheels(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    enum { N_OBJECTS = 6 };
    const double body_hx = 0.9, body_hy = 0.3, body_hz = 2.0;
    const double wheel_x = 0.85, wheel_y = 0.4, wheel_z = 1.4;
    const double wheel_radius = 0.35;
    const double body_y = 1.7; /* wheels start 5 cm above the ground top (y = 2.5) */

    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *ground_surf = NULL;
    ivp_compact_surface_t *body_surf = NULL;
    ivp_surman_polygon_t ground_sm, body_sm;

    ivp_object_t *objects[N_OBJECTS] = {0};
    const char *types[N_OBJECTS] = { "ground", "wheel", "wheel", "wheel", "wheel", "car_body" };
    int rc = 1;
    ivp_car_system_t car;
    double body_z0 = 0.0;
    double max_wheel_offset = 0.0; /* drift of a wheel from its hard point (body x/z plane) */

    /* static ground */
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false;
    tmpl.material.friction = IVP_REAL(0.9);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0));
    objects[0] = scenario_create_box(env, &tmpl, 30.0, 0.5, 30.0, &ground_surf, &ground_sm);

    /* car body */
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(800.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1100.0), IVP_REAL(1300.0), IVP_REAL(250.0));
    tmpl.rot_inertia_is_factor = false;
    tmpl.material.friction = IVP_REAL(0.6);
    tmpl.material.elasticity = IVP_REAL(0.1);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();
    tmpl.nocoll_group_ident = "car"; /* the wheels overlap the body box */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), (ivp_real_t)body_y, IVP_REAL(0.0));
    objects[5] = scenario_create_box(env, &tmpl, body_hx, body_hy, body_hz, &body_surf, &body_sm);

    /* wheels: 0 FL, 1 FR, 2 RL, 3 RR (x right, y down, z forward) */
    ivp_car_system_template_t tcs;
    ivp_car_system_template_init(&tcs, 4, 2);
    tcs.car_body = objects[5];
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(25.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1.225), IVP_REAL(1.225), IVP_REAL(1.225));
    tmpl.rot_inertia_is_factor = false;
    tmpl.material.friction = IVP_REAL(0.9);
    tmpl.material.elasticity = IVP_REAL(0.1);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();
    tmpl.ball_radius = (ivp_real_t)wheel_radius;
    tmpl.nocoll_group_ident = "car";
    for (int i = 0; i < 4; i++) {
        const double wx = (i & 1) ? wheel_x : -wheel_x;
        const double wz = (i & 2) ? -wheel_z : wheel_z;
        ivp_vec3r_set(&tmpl.position, (ivp_real_t)wx, (ivp_real_t)(body_y + wheel_y), (ivp_real_t)wz);
        objects[1 + i] = ivp_environment_create_object(env, &tmpl, NULL);

        tcs.wheel_objects[i] = objects[1 + i];
        tcs.wheel_radius[i] = (ivp_real_t)(float)wheel_radius;
        tcs.wheel_reversed_sign[i] = IVP_REAL(1.0);
        tcs.wheel_friction[i] = (ivp_real_t)0.9f;
        tcs.wheel_pos_Bos[i] = ivp_vec3f((float)wx, (float)wheel_y, (float)wz);
        tcs.trace_pos_Bos[i] = tcs.wheel_pos_Bos[i];
        tcs.spring_constant[i] = IVP_REAL(40000.0);
        tcs.spring_damping[i] = IVP_REAL(3000.0);
        tcs.spring_damping_compression[i] = IVP_REAL(3000.0);
        tcs.max_body_force[i] = IVP_REAL(60000.0);
        tcs.spring_pre_tension[i] = (ivp_real_t)0.05f;
    }
    tcs.stabilizer_constant[0] = IVP_REAL(1500.0);
    tcs.stabilizer_constant[1] = IVP_REAL(1500.0);
    tcs.wheel_max_rotation_speed[0] = IVP_REAL(60.0);
    tcs.wheel_max_rotation_speed[1] = IVP_REAL(60.0);
    tcs.body_counter_torque_factor = (ivp_real_t)0.2f;
    tcs.extra_gravity_force = IVP_REAL(2000.0);
    tcs.extra_gravity_height_offset = IVP_REAL(0.0);
    tcs.body_down_force_vertical_offset = (ivp_real_t)0.2f;
    tcs.fast_turn_factor = IVP_REAL(1.0);

    for (int i = 0; i < N_OBJECTS; i++) {
        if (!objects[i]) goto cleanup;
    }

    if (!ivp_car_system_init(&car, &tcs)) goto cleanup;

    body_z0 = (double)objects[5]->physical_core->pos_world_f_core_last_psi.k[2];

    snapshot_frame(out, SCENARIO_CAR_REAL_WHEELS, 0, dt, env, objects, types, N_OBJECTS);
    for (int step = 1; step <= steps; step++) {
        car_real_wheels_controls(&car, step);
        scenario_step(env, dt);
        snapshot_frame(out, SCENARIO_CAR_REAL_WHEELS, step, dt, env, objects, types, N_OBJECTS);

        if (inv) {
            /* constraint: wheel center in the body's x/z plane stays on its hard point */
            const ivp_core_t *bc = objects[5]->physical_core;
            for (int i = 0; i < 4; i++) {
                ivp_vec3r_t d, d_bs;
                ivp_vec3r_sub(&d, &objects[1 + i]->physical_core->m_world_f_core_last_psi.pos, &bc->m_world_f_core_last_psi.pos);
                ivp_mat3r_vimult(&d_bs, &bc->m_world_f_core_last_psi.rot, &d);
                const double ex = (double)d_bs.k[0] - (double)tcs.wheel_pos_Bos[i].k[0];
                const double ez = (double)d_bs.k[2] - (double)tcs.wheel_pos_Bos[i].k[2];
                const double e = sqrt(ex * ex + ez * ez);
                if (e > max_wheel_offset) max_wheel_offset = e;
            }
        }
    }

    if (inv) {
        const double body_z = (double)objects[5]->physical_core->pos_world_f_core_last_psi.k[2];
        const double body_y = (double)objects[5]->physical_core->pos_world_f_core_last_psi.k[1];
        inv_check(inv, body_y < 2.5, "car body stays above the ground top: y=%.6f", body_y);
        inv_check(inv, steps < 360 || body_z > body_z0 + 1.0, "car drives forward: z0=%.6f z=%.6f", body_z0, body_z);
        inv_check(inv, max_wheel_offset < 0.1, "wheels stay on their hard points: max offset %.6f", max_wheel_offset);
        for (int i = 0; i < N_OBJECTS; i++) {
            const ivp_vec3r_t *p = &objects[i]->physical_core->pos_world_f_core_last_psi;
            inv_check(inv, isfinite((double)p->k[0]) && isfinite((double)p->k[1]) && isfinite((double)p->k[2]),
                      "object %d position finite", i);
        }
    }

    ivp_car_system_destroy(&car);
    rc = 0;

cleanup:
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, body_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    return rc;
}

/* ── Geometry / collision coverage scenarios ─────────────────────────────
 *
 * Mirror tools/ivp_reference_runner/ref_runner_scenarios_geometry.cxx:
 * concave ledge soups (static trough, dynamic compounds with and without
 * root convex hull), irregular pointsoup hulls and a compact grid
 * heightfield (IVP_GridBuilder_Array + IVP_SurfaceManager_Grid).  Point
 * lists, quaternions and materials are the reference literals; objects are
 * created in the reference order. */

enum { GEO_MAX_OBJECTS = 16, GEO_MAX_SURFACES = 16 };

typedef struct geo_scene {
    ivp_environment_t *env;
    ivp_compact_surface_t *surfaces[GEO_MAX_SURFACES];
    ivp_surman_polygon_t surmans[GEO_MAX_SURFACES];
    int n_surfaces;
    ivp_compact_grid_t *grid;
    ivp_surman_compact_grid_t grid_sm;
    ivp_object_t *objects[GEO_MAX_OBJECTS];
    const char *types[GEO_MAX_OBJECTS];
    int count;
    bool failed;
} geo_scene_t;

static bool geo_scene_init(geo_scene_t *sc, double dt)
{
    memset(sc, 0, sizeof(*sc));
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));
    sc->env = ivp_environment_create(&cfg);
    return sc->env != NULL;
}

static void geo_scene_destroy(geo_scene_t *sc)
{
    if (sc->env) ivp_environment_destroy(sc->env);
    for (int i = 0; i < sc->n_surfaces; i++) ivp_compact_surface_free(NULL, sc->surfaces[i]);
    if (sc->grid) ivp_compact_grid_free(NULL, sc->grid);
}

/* takes ownership of the surface; returns its polygon surface manager */
static const ivp_surman_t *geo_scene_surman(geo_scene_t *sc, ivp_compact_surface_t *s)
{
    if (!s || sc->n_surfaces >= GEO_MAX_SURFACES) {
        if (s) ivp_compact_surface_free(NULL, s);
        sc->failed = true;
        return NULL;
    }
    int i = sc->n_surfaces++;
    sc->surfaces[i] = s;
    ivp_surman_polygon_init(&sc->surmans[i], s);
    return &sc->surmans[i].base;
}

static void geo_scene_add(geo_scene_t *sc, ivp_object_t *obj, const char *type)
{
    if (!obj || sc->count >= GEO_MAX_OBJECTS) {
        sc->failed = true;
        return;
    }
    sc->objects[sc->count] = obj;
    sc->types[sc->count] = type;
    sc->count++;
}

static void geo_box_points(double *c, double hx, double hy, double hz, double ox, double oy, double oz)
{
    int k = 0;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2) {
                c[k++] = (double)sx * hx + ox;
                c[k++] = (double)sy * hy + oy;
                c[k++] = (double)sz * hz + oz;
            }
}

static ivp_compact_ledge_t *geo_box_ledge(double hx, double hy, double hz, double ox, double oy, double oz)
{
    double c[24];
    geo_box_points(c, hx, hy, hz, ox, oy, oz);
    return ivp_build_convex_ledge_d(NULL, c, 8);
}

/* IVP_SurfaceBuilder_Ledge_Soup::compile; frees the input ledges like the
 * reference template default (free_input_compact_ledges) */
static ivp_compact_surface_t *geo_compile_soup(ivp_compact_ledge_t **ledges, int n, bool root_hull)
{
    ivp_compact_surface_t *s = NULL;
    bool ok = true;
    for (int i = 0; i < n; i++) ok = ok && ledges[i];
    if (ok) {
        ivp_ledge_soup_t soup;
        ivp_ledge_soup_init(&soup, NULL);
        for (int i = 0; i < n; i++) ivp_ledge_soup_add(&soup, ledges[i]);
        ivp_ledge_soup_template_t t;
        ivp_ledge_soup_template_init(&t);
        t.build_root_convex_hull = root_hull;
        s = ivp_ledge_soup_compile_ex(&soup, &t);
        ivp_ledge_soup_destroy(&soup);
    }
    for (int i = 0; i < n; i++)
        if (ledges[i]) ivp_compact_ledge_free(NULL, ledges[i]);
    return s;
}

static ivp_object_t *geo_static(geo_scene_t *sc, const ivp_surman_t *sm, double friction, double elasticity,
                                ivp_quatr_t q, double x, double y, double z)
{
    if (!sm) return NULL;
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false;
    tmpl.material.friction = (ivp_real_t)friction;
    tmpl.material.elasticity = (ivp_real_t)elasticity;
    tmpl.physical_unmoveable = true;
    tmpl.orientation = q;
    ivp_vec3r_set(&tmpl.position, (ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
    return ivp_environment_create_object(sc->env, &tmpl, sm);
}

/* inertia == NULL: reference default rot_inertia_is_factor (1,1,1) */
static ivp_object_t *geo_dynamic(geo_scene_t *sc, const ivp_surman_t *sm, double friction, double elasticity,
                                 double mass, const double *inertia, ivp_quatr_t q, double x, double y, double z)
{
    if (!sm) return NULL;
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = (ivp_real_t)mass;
    if (inertia) {
        ivp_vec3r_set(&tmpl.rot_inertia, (ivp_real_t)inertia[0], (ivp_real_t)inertia[1], (ivp_real_t)inertia[2]);
        tmpl.rot_inertia_is_factor = false;
    }
    tmpl.material.friction = (ivp_real_t)friction;
    tmpl.material.elasticity = (ivp_real_t)elasticity;
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.orientation = q;
    ivp_vec3r_set(&tmpl.position, (ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
    return ivp_environment_create_object(sc->env, &tmpl, sm);
}

static ivp_object_t *geo_box(geo_scene_t *sc, double friction, double elasticity, double hx, double hy, double hz,
                             double mass, double ix, double iy, double iz, ivp_quatr_t q,
                             double x, double y, double z)
{
    const double inertia[3] = { ix, iy, iz };
    return geo_dynamic(sc, geo_scene_surman(sc, ref_box_surface(hx, hy, hz)), friction, elasticity, mass,
                       inertia, q, x, y, z);
}

static ivp_object_t *geo_ball(geo_scene_t *sc, double friction, double elasticity, double radius, double mass,
                              double inertia, double x, double y, double z)
{
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = (ivp_real_t)mass;
    ivp_vec3r_set(&tmpl.rot_inertia, (ivp_real_t)inertia, (ivp_real_t)inertia, (ivp_real_t)inertia);
    tmpl.rot_inertia_is_factor = false;
    tmpl.material.friction = (ivp_real_t)friction;
    tmpl.material.elasticity = (ivp_real_t)elasticity;
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, (ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
    tmpl.ball_radius = (ivp_real_t)(float)radius;   /* IVP_Template_Ball::radius is IVP_FLOAT */
    return ivp_environment_create_object(sc->env, &tmpl, NULL);
}

static int geo_run(geo_scene_t *sc, FILE *out, int scenario_id, int steps, double dt)
{
    if (sc->failed) {
        geo_scene_destroy(sc);
        return 1;
    }
    snapshot_frame(out, scenario_id, 0, dt, sc->env, sc->objects, sc->types, sc->count);
    for (int step = 1; step <= steps; step++) {
        scenario_step(sc->env, dt);
        snapshot_frame(out, scenario_id, step, dt, sc->env, sc->objects, sc->types, sc->count);
    }
    return 0;
}

/* objects resting at the end must still be above the floor level y_max */
static void geo_check_contained(const geo_scene_t *sc, invariant_result_t *inv, const char *name, double y_max)
{
    if (!inv) return;
    for (int i = 0; i < sc->count; i++) {
        const ivp_core_t *core = ivp_object_get_core_const(sc->objects[i]);
        if (core->physical_unmoveable) continue;
        double y = (double)core->pos_world_f_core_last_psi.k[1];
        inv_check(inv, y == y && y < y_max, "%s object %d (%s) stays on the geometry: y=%.6f", name, i,
                  sc->types[i], y);
    }
}

static const double GEO_TETRA[4 * 3] = {
    0.0, -0.35, 0.0,
    0.3, 0.2, 0.3,
    -0.35, 0.2, 0.15,
    0.05, 0.2, -0.35,
};

static const double GEO_PRISM[6 * 3] = {
    -0.4, 0.25, -0.5,  0.4, 0.25, -0.5,  0.0, -0.45, -0.5,
    -0.4, 0.25, 0.5,   0.4, 0.25, 0.5,   0.0, -0.45, 0.5,
};

static const double GEO_PYRAMID[5 * 3] = {
    -0.35, 0.2, -0.35,  0.35, 0.2, -0.35,  0.35, 0.2, 0.35,  -0.35, 0.2, 0.35,
    0.05, -0.45, 0.0,
};

static const double GEO_ROCK[10 * 3] = {
    0.42, 0.05, 0.11,   -0.38, 0.12, 0.2,   0.1, -0.41, 0.05,  0.05, 0.36, -0.12,
    -0.15, -0.2, 0.37,  0.22, 0.18, -0.33,  -0.27, -0.25, -0.24, 0.31, -0.22, 0.29,
    -0.05, 0.3, 0.33,   0.12, 0.02, -0.44,
};

/* octagonal cylinder, axis along z: radius 0.35, half length 0.4 */
static void geo_cylinder_points(double *c)
{
    static const double ring[8][2] = {
        {0.35, 0.0}, {0.24748737341529164, 0.24748737341529164}, {0.0, 0.35},
        {-0.24748737341529164, 0.24748737341529164}, {-0.35, 0.0},
        {-0.24748737341529164, -0.24748737341529164}, {0.0, -0.35},
        {0.24748737341529164, -0.24748737341529164}};
    int k = 0;
    for (int s = -1; s <= 1; s += 2)
        for (int i = 0; i < 8; ++i) {
            c[k++] = ring[i][0];
            c[k++] = ring[i][1];
            c[k++] = (double)s * 0.4;
        }
}

/* concave_static: concave trough (ledge soup without root hull: ledgetree
 * traversal, recursive mindists) with balls, boxes and a hull on it */
static int run_concave_static(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    geo_scene_t sc;
    if (!geo_scene_init(&sc, dt)) return 1;
    const ivp_quatr_t q_ident = ivp_quatr_identity();

    static const double ramp[8 * 3] = {
        -3.0, 0.0, -1.5,  -3.0, 0.0, 1.5,  -6.0, -2.5, -1.5,  -6.0, -2.5, 1.5,
        -6.0, 0.5, -1.5,  -6.0, 0.5, 1.5,  -3.0, 0.5, -1.5,   -3.0, 0.5, 1.5,
    };
    ivp_compact_ledge_t *ledges[5];
    ledges[0] = geo_box_ledge(3.0, 0.25, 1.5, 0.0, 0.25, 0.0);        /* floor */
    ledges[1] = ivp_build_convex_ledge_d(NULL, ramp, 8);               /* ramp */
    ledges[2] = geo_box_ledge(0.25, 1.25, 1.5, 3.25, -0.75, 0.0);     /* right wall */
    ledges[3] = geo_box_ledge(0.4, 0.15, 1.5, 1.0, -0.15, 0.0);       /* bump */
    ledges[4] = geo_box_ledge(4.75, 0.75, 0.25, -1.25, -0.25, -1.75); /* back wall */
    const ivp_surman_t *trough = geo_scene_surman(&sc, geo_compile_soup(ledges, 5, false));
    geo_scene_add(&sc, geo_static(&sc, trough, 0.7, 0.1, q_ident, 0.0, 0.0, 0.0), "trough");

    geo_scene_add(&sc, geo_ball(&sc, 0.5, 0.2, 0.4, 1.0, 0.064, -5.2, -4.0, 0.3), "ball");
    geo_scene_add(&sc, geo_box(&sc, 0.5, 0.2, 0.3, 0.3, 0.3, 0.8, 0.048, 0.048, 0.048,
                               ivp_quatr_make(0.0, 0.0, 0.17364817766693033, 0.984807753012208),
                               -4.0, -3.0, -0.6), "box");
    ivp_object_t *b3 = geo_ball(&sc, 0.5, 0.2, 0.35, 0.7, 0.0343, 1.5, -2.0, 0.4);
    if (b3) ivp_vec3r_set(&b3->physical_core->speed, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    geo_scene_add(&sc, b3, "ball");
    geo_scene_add(&sc, geo_box(&sc, 0.5, 0.2, 0.4, 0.2, 0.3, 1.2, 0.052, 0.1, 0.08,
                               ivp_quatr_make(0.3007057995042731, 0.0, 0.0, 0.9537169507482269),
                               0.8, -1.2, -0.3), "box");
    geo_scene_add(&sc, geo_dynamic(&sc, geo_scene_surman(&sc, ivp_build_convex_surface_d(NULL, GEO_TETRA, 4)),
                                   0.5, 0.2, 0.5, NULL, q_ident, -2.0, -2.5, 0.5), "hull");

    int rc = geo_run(&sc, out, SCENARIO_CONCAVE_STATIC, steps, dt);
    if (rc) return rc;
    geo_check_contained(&sc, inv, "concave_static", 0.5);
    geo_scene_destroy(&sc);
    return 0;
}

/* compound_dynamic: moving compounds (L shape without root hull, dumbbell
 * with root convex hull) tumbling on a box floor, hit by a box and a ball */
static int run_compound_dynamic(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    geo_scene_t sc;
    if (!geo_scene_init(&sc, dt)) return 1;
    const ivp_quatr_t q_ident = ivp_quatr_identity();

    geo_scene_add(&sc, geo_static(&sc, geo_scene_surman(&sc, ref_box_surface(20.0, 0.5, 20.0)), 0.8, 0.0,
                                  q_ident, 0.0, 2.0, 0.0), "ground");

    ivp_compact_ledge_t *l_ledges[2];
    l_ledges[0] = geo_box_ledge(0.6, 0.15, 0.3, 0.0, 0.0, 0.0);
    l_ledges[1] = geo_box_ledge(0.15, 0.45, 0.3, -0.45, -0.6, 0.0);
    const ivp_surman_t *l_shape = geo_scene_surman(&sc, geo_compile_soup(l_ledges, 2, false));
    ivp_object_t *lobj = geo_dynamic(&sc, l_shape, 0.6, 0.15, 2.0, NULL,
                                     ivp_quatr_make(0.15529142706151244, 0.0, 0.2070552360820166,
                                                    0.9659258262890683),
                                     -1.2, -1.0, 0.0);
    if (lobj) ivp_vec3r_set(&lobj->physical_core->speed, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    geo_scene_add(&sc, lobj, "compound");

    ivp_compact_ledge_t *d_ledges[3];
    d_ledges[0] = geo_box_ledge(0.3, 0.3, 0.3, -0.8, 0.0, 0.0);
    d_ledges[1] = geo_box_ledge(0.3, 0.3, 0.3, 0.8, 0.0, 0.0);
    d_ledges[2] = geo_box_ledge(0.5, 0.08, 0.08, 0.0, 0.0, 0.0);
    const ivp_surman_t *dumbbell = geo_scene_surman(&sc, geo_compile_soup(d_ledges, 3, true));
    ivp_object_t *dobj = geo_dynamic(&sc, dumbbell, 0.6, 0.15, 3.0, NULL,
                                     ivp_quatr_make(0.0, 0.0, 0.21643961393810288, 0.9762960071199334),
                                     1.4, -2.2, 0.25);
    if (dobj) {
        dobj->physical_core->rot_speed.k[0] = 0.5f;
        dobj->physical_core->rot_speed.k[1] = 0.0f;
        dobj->physical_core->rot_speed.k[2] = 1.5f;
    }
    geo_scene_add(&sc, dobj, "compound");

    geo_scene_add(&sc, geo_box(&sc, 0.6, 0.15, 0.25, 0.25, 0.25, 0.5, 0.02, 0.02, 0.02, q_ident,
                               1.3, -4.0, 0.0), "box");
    geo_scene_add(&sc, geo_ball(&sc, 0.6, 0.15, 0.3, 0.4, 0.0144, -1.5, -3.5, 0.0), "ball");

    int rc = geo_run(&sc, out, SCENARIO_COMPOUND_DYNAMIC, steps, dt);
    if (rc) return rc;
    geo_check_contained(&sc, inv, "compound_dynamic", 1.5);
    geo_scene_destroy(&sc);
    return 0;
}

/* convex_hulls: irregular pointsoup hulls (tetrahedron, prism, octagonal
 * cylinder, rock, pyramid) falling on a floor and onto each other */
static int run_convex_hulls(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    geo_scene_t sc;
    if (!geo_scene_init(&sc, dt)) return 1;
    const ivp_quatr_t q_ident = ivp_quatr_identity();

    geo_scene_add(&sc, geo_static(&sc, geo_scene_surman(&sc, ref_box_surface(10.0, 0.5, 10.0)), 0.8, 0.0,
                                  q_ident, 0.0, 2.0, 0.0), "ground");

    geo_scene_add(&sc, geo_dynamic(&sc, geo_scene_surman(&sc, ivp_build_convex_surface_d(NULL, GEO_TETRA, 4)),
                                   0.55, 0.1, 0.6, NULL,
                                   ivp_quatr_make(0.20521208599540122, 0.273616114660535, 0.0,
                                                  0.9396926207859084),
                                   -1.5, -1.0, 0.0), "tetra");
    geo_scene_add(&sc, geo_dynamic(&sc, geo_scene_surman(&sc, ivp_build_convex_surface_d(NULL, GEO_PRISM, 6)),
                                   0.55, 0.1, 0.9, NULL,
                                   ivp_quatr_make(0.0, 0.0, 0.573576436351046, 0.8191520442889918),
                                   0.0, -1.0, 0.0), "prism");
    double cyl[16 * 3];
    geo_cylinder_points(cyl);
    ivp_object_t *cobj = geo_dynamic(&sc, geo_scene_surman(&sc, ivp_build_convex_surface_d(NULL, cyl, 16)),
                                     0.55, 0.1, 1.0, NULL, q_ident, 1.5, -0.5, 0.0);
    if (cobj) ivp_vec3r_set(&cobj->physical_core->speed, IVP_REAL(-1.5), IVP_REAL(0.0), IVP_REAL(0.0));
    geo_scene_add(&sc, cobj, "cylinder");
    static const double rock_inertia[3] = { 0.09, 0.1, 0.08 };
    geo_scene_add(&sc, geo_dynamic(&sc, geo_scene_surman(&sc, ivp_build_convex_surface_d(NULL, GEO_ROCK, 10)),
                                   0.55, 0.1, 1.3, rock_inertia,
                                   ivp_quatr_make(0.24399876718044458, 0.24399876718044458,
                                                  0.24399876718044458, 0.9063077870366499),
                                   -0.3, -2.6, 0.1), "rock");
    geo_scene_add(&sc, geo_dynamic(&sc, geo_scene_surman(&sc, ivp_build_convex_surface_d(NULL, GEO_PYRAMID, 5)),
                                   0.55, 0.1, 0.7, NULL, q_ident, -1.4, -2.8, 0.05), "pyramid");

    int rc = geo_run(&sc, out, SCENARIO_CONVEX_HULLS, steps, dt);
    if (rc) return rc;
    geo_check_contained(&sc, inv, "convex_hulls", 1.5);
    geo_scene_destroy(&sc);
    return 0;
}

/* grid_terrain: compact grid bowl (IVP_GridBuilder_Array, rows -> x,
 * columns -> z, heights -> -y) with balls, boxes and a pyramid rolling in */
enum { GEO_GRID_N = 10 };

static void geo_grid_heights(float *h)
{
    for (int r = 0; r < GEO_GRID_N; ++r)
        for (int c = 0; c < GEO_GRID_N; ++c) {
            int d = (2 * r - 9) * (2 * r - 9) + (2 * c - 9) * (2 * c - 9);
            h[r * GEO_GRID_N + c] = 0.02f * (float)d + 0.1f * (float)((r * 3 + c * 5) % 4);
        }
}

static int run_grid_terrain(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    geo_scene_t sc;
    if (!geo_scene_init(&sc, dt)) return 1;
    const ivp_quatr_t q_ident = ivp_quatr_identity();

    float heights[GEO_GRID_N * GEO_GRID_N];
    geo_grid_heights(heights);
    ivp_template_compact_grid_t gt;
    ivp_template_compact_grid_init(&gt);
    gt.row_info.n_points = GEO_GRID_N;
    gt.row_info.maps_to = 0;     /* IVP_INDEX_X */
    gt.row_info.invert_axis = false;
    gt.column_info.n_points = GEO_GRID_N;
    gt.column_info.maps_to = 2;  /* IVP_INDEX_Z */
    gt.column_info.invert_axis = false;
    gt.height_maps_to = 1;       /* IVP_INDEX_Y */
    gt.height_invert_axis = true;
    gt.grid_field_size = 1.0f;
    sc.grid = ivp_gridbuild_array_to_compact_grid(NULL, &gt, heights);
    if (!sc.grid) {
        geo_scene_destroy(&sc);
        return 1;
    }
    ivp_surman_compact_grid_init(&sc.grid_sm, sc.grid);
    geo_scene_add(&sc, geo_static(&sc, &sc.grid_sm.base, 0.7, 0.0, q_ident, -4.5, 0.0, -4.5), "grid");

    geo_scene_add(&sc, geo_ball(&sc, 0.5, 0.1, 0.4, 1.0, 0.064, -3.0, -2.5, -0.4), "ball");
    geo_scene_add(&sc, geo_ball(&sc, 0.5, 0.1, 0.3, 0.6, 0.0216, 2.6, -2.2, 1.5), "ball");
    geo_scene_add(&sc, geo_box(&sc, 0.5, 0.1, 0.3, 0.3, 0.3, 0.8, 0.048, 0.048, 0.048,
                               ivp_quatr_make(0.0, 0.13052619222005157, 0.0, 0.9914448613738104),
                               0.4, -2.6, -3.1), "box");
    geo_scene_add(&sc, geo_box(&sc, 0.5, 0.1, 0.5, 0.2, 0.35, 1.5, 0.08125, 0.18625, 0.145,
                               ivp_quatr_make(0.25881904510252074, 0.0, 0.0, 0.9659258262890683),
                               -1.8, -2.4, 2.6), "box");
    geo_scene_add(&sc, geo_dynamic(&sc, geo_scene_surman(&sc, ivp_build_convex_surface_d(NULL, GEO_PYRAMID, 5)),
                                   0.5, 0.1, 0.7, NULL,
                                   ivp_quatr_make(0.0, 0.49999999999999994, 0.0, 0.8660254037844387),
                                   3.0, -2.4, -1.2), "pyramid");

    int rc = geo_run(&sc, out, SCENARIO_GRID_TERRAIN, steps, dt);
    if (rc) return rc;
    geo_check_contained(&sc, inv, "grid_terrain", 0.5);
    geo_scene_destroy(&sc);
    return 0;
}

static const double GEO_GEODE[32 * 3] = {
    0.092, 0.358, 0.0,     -0.112, 0.325, 0.102,   0.018, 0.328, -0.208,   0.134, 0.277, 0.175,
    -0.262, 0.275, -0.046, 0.237, 0.244, -0.151,   -0.074, 0.21, 0.274,    -0.149, 0.202, -0.286,
    0.292, 0.165, 0.107,   -0.318, 0.153, 0.131,   0.141, 0.122, -0.301,   0.102, 0.1, 0.325,
    -0.317, 0.082, -0.184, 0.386, 0.062, -0.085,   -0.204, 0.033, 0.291,   -0.047, 0.011, -0.36,
    0.296, -0.012, 0.25,   -0.405, -0.038, 0.017,  0.269, -0.06, -0.268,   -0.017, -0.082, 0.364,
    -0.251, -0.115, -0.301, 0.328, -0.121, 0.044,  -0.301, -0.163, 0.21,   0.071, -0.172, -0.316,
    0.151, -0.19, 0.263,   -0.273, -0.212, -0.087, 0.252, -0.242, -0.117,  -0.107, -0.287, 0.256,
    -0.076, -0.282, -0.212, 0.183, -0.325, 0.096,  -0.159, -0.352, 0.042,  0.05, -0.361, -0.078,
};

static const double GEO_SMALL_TETRA[4 * 3] = {
    0.0, -0.175, 0.0,  0.15, 0.1, 0.15,  -0.175, 0.1, 0.075,  0.025, 0.1, -0.175,
};

/* 16-gon disc, axis y: radius 0.45, half thickness 0.05 */
static void geo_disc_points(double *c)
{
    static const double ring[16][2] = {
        {0.45, 0.0}, {0.4157, 0.1722}, {0.3182, 0.3182}, {0.1722, 0.4157},
        {0.0, 0.45}, {-0.1722, 0.4157}, {-0.3182, 0.3182}, {-0.4157, 0.1722},
        {-0.45, 0.0}, {-0.4157, -0.1722}, {-0.3182, -0.3182}, {-0.1722, -0.4157},
        {0.0, -0.45}, {0.1722, -0.4157}, {0.3182, -0.3182}, {0.4157, -0.1722}};
    int k = 0;
    for (int s = -1; s <= 1; s += 2)
        for (int i = 0; i < 16; ++i) {
            c[k++] = ring[i][0];
            c[k++] = (double)s * 0.05;
            c[k++] = ring[i][1];
        }
}

/* hexagonal needle along x: half length 0.6, radius 0.06 */
static void geo_needle_points(double *c)
{
    static const double ring[6][2] = {
        {0.06, 0.0}, {0.03, 0.052}, {-0.03, 0.052}, {-0.06, 0.0}, {-0.03, -0.052}, {0.03, -0.052}};
    int k = 0;
    for (int s = -1; s <= 1; s += 2)
        for (int i = 0; i < 6; ++i) {
            c[k++] = (double)s * 0.6;
            c[k++] = ring[i][0];
            c[k++] = ring[i][1];
        }
}

static ivp_compact_surface_t *geo_dumbbell_surface(void)
{
    ivp_compact_ledge_t *d_ledges[3];
    d_ledges[0] = geo_box_ledge(0.3, 0.3, 0.3, -0.8, 0.0, 0.0);
    d_ledges[1] = geo_box_ledge(0.3, 0.3, 0.3, 0.8, 0.0, 0.0);
    d_ledges[2] = geo_box_ledge(0.5, 0.08, 0.08, 0.0, 0.0, 0.0);
    return geo_compile_soup(d_ledges, 3, true);
}

static ivp_compact_surface_t *geo_l_surface(void)
{
    ivp_compact_ledge_t *l_ledges[2];
    l_ledges[0] = geo_box_ledge(0.6, 0.15, 0.3, 0.0, 0.0, 0.0);
    l_ledges[1] = geo_box_ledge(0.15, 0.45, 0.3, -0.45, -0.6, 0.0);
    return geo_compile_soup(l_ledges, 2, false);
}

static const ivp_surman_t *geo_hull(geo_scene_t *sc, const double *c, int n)
{
    return geo_scene_surman(sc, ivp_build_convex_surface_d(NULL, c, n));
}

/* hull_pile: mixed hulls (32 point geode, thin disc, needle, rock, prism,
 * pyramid) and compounds piling up in a rotated, shifted concave hopper */
static int run_hull_pile(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    geo_scene_t sc;
    if (!geo_scene_init(&sc, dt)) return 1;
    const double fs = 0.6, es = 0.1, fd = 0.45, ed = 0.2;

    static const double wall_xm[8 * 3] = {
        -1.0, 0.2, -2.8,  -1.0, 0.2, 2.8,  -2.8, -3.0, -2.8,  -2.8, -3.0, 2.8,
        -3.1, -3.0, -2.8, -3.1, -3.0, 2.8, -1.3, 0.2, -2.8,   -1.3, 0.2, 2.8,
    };
    static const double wall_xp[8 * 3] = {
        1.0, 0.2, -2.8,  1.0, 0.2, 2.8,  2.8, -3.0, -2.8,  2.8, -3.0, 2.8,
        3.1, -3.0, -2.8, 3.1, -3.0, 2.8, 1.3, 0.2, -2.8,   1.3, 0.2, 2.8,
    };
    static const double wall_zm[8 * 3] = {
        -2.8, 0.2, -1.0,  2.8, 0.2, -1.0,  -2.8, -3.0, -2.8,  2.8, -3.0, -2.8,
        -2.8, -3.0, -3.1, 2.8, -3.0, -3.1, -2.8, 0.2, -1.3,   2.8, 0.2, -1.3,
    };
    static const double wall_zp[8 * 3] = {
        -2.8, 0.2, 1.0,  2.8, 0.2, 1.0,  -2.8, -3.0, 2.8,  2.8, -3.0, 2.8,
        -2.8, -3.0, 3.1, 2.8, -3.0, 3.1, -2.8, 0.2, 1.3,   2.8, 0.2, 1.3,
    };
    ivp_compact_ledge_t *ledges[5];
    ledges[0] = geo_box_ledge(1.0, 0.1, 1.0, 0.0, 0.1, 0.0);
    ledges[1] = ivp_build_convex_ledge_d(NULL, wall_xm, 8);
    ledges[2] = ivp_build_convex_ledge_d(NULL, wall_xp, 8);
    ledges[3] = ivp_build_convex_ledge_d(NULL, wall_zm, 8);
    ledges[4] = ivp_build_convex_ledge_d(NULL, wall_zp, 8);
    const ivp_surman_t *hopper = geo_scene_surman(&sc, geo_compile_soup(ledges, 5, false));
    geo_scene_add(&sc, geo_static(&sc, hopper, fs, es,
                                  ivp_quatr_make(0.0, 0.25881904510252074, 0.0, 0.9659258262890683),
                                  0.5, 1.0, -0.3), "hopper");

    const ivp_quatr_t q_ident = ivp_quatr_identity();
    const ivp_quatr_t q_a = ivp_quatr_make(0.20521208599540122, 0.273616114660535, 0.0, 0.9396926207859084);
    const ivp_quatr_t q_b = ivp_quatr_make(0.24399876718044458, 0.24399876718044458, 0.24399876718044458,
                                           0.9063077870366499);
    const ivp_quatr_t q_c = ivp_quatr_make(0.15529142706151244, 0.0, 0.2070552360820166, 0.9659258262890683);

    geo_scene_add(&sc, geo_dynamic(&sc, geo_hull(&sc, GEO_GEODE, 32), fd, ed, 1.2, NULL, q_ident, 0.6, -2.0, -0.2),
                  "geode");
    double disc[32 * 3];
    geo_disc_points(disc);
    geo_scene_add(&sc, geo_dynamic(&sc, geo_hull(&sc, disc, 32), fd, ed, 0.8, NULL, q_a, 0.3, -2.9, -0.5), "disc");
    double needle[12 * 3];
    geo_needle_points(needle);
    geo_scene_add(&sc, geo_dynamic(&sc, geo_hull(&sc, needle, 12), fd, ed, 0.3, NULL, q_b, 0.9, -3.6, 0.1),
                  "needle");
    geo_scene_add(&sc, geo_dynamic(&sc, geo_scene_surman(&sc, geo_l_surface()), fd, ed, 2.0, NULL, q_c,
                                   0.2, -4.4, -0.1), "compound");
    geo_scene_add(&sc, geo_dynamic(&sc, geo_scene_surman(&sc, geo_dumbbell_surface()), fd, ed, 3.0, NULL, q_a,
                                   0.7, -5.6, -0.4), "compound");
    geo_scene_add(&sc, geo_dynamic(&sc, geo_hull(&sc, GEO_ROCK, 10), fd, ed, 1.3, NULL, q_b, 0.1, -6.6, -0.3),
                  "rock");
    geo_scene_add(&sc, geo_dynamic(&sc, geo_hull(&sc, GEO_PRISM, 6), fd, ed, 0.9, NULL, q_c, 0.8, -7.4, 0.0),
                  "prism");
    geo_scene_add(&sc, geo_ball(&sc, fd, ed, 0.3, 0.6, 0.0216, 0.4, -8.2, -0.6), "ball");
    geo_scene_add(&sc, geo_dynamic(&sc, geo_hull(&sc, GEO_PYRAMID, 5), fd, ed, 0.7, NULL, q_a, 0.5, -9.0, -0.1),
                  "pyramid");
    geo_scene_add(&sc, geo_box(&sc, fd, ed, 0.25, 0.2, 0.3, 0.9, 0.039, 0.0459, 0.0309, q_b, 0.6, -9.8, -0.4),
                  "box");
    geo_scene_add(&sc, geo_ball(&sc, fd, ed, 0.22, 0.4, 0.007744, 0.3, -10.6, -0.2), "ball");

    int rc = geo_run(&sc, out, SCENARIO_HULL_PILE, steps, dt);
    if (rc) return rc;
    if (inv) {
        int n_in = 0;
        for (int i = 1; i < sc.count; i++) {
            double y = (double)ivp_object_get_core_const(sc.objects[i])->pos_world_f_core_last_psi.k[1];
            if (y == y && y < 1.0 && y > -3.5) n_in++;
        }
        inv_check(inv, n_in >= 6, "hull_pile: %d objects rest in the hopper", n_in);
    }
    geo_scene_destroy(&sc);
    return 0;
}

/* galton: deep static ledge soup (49 ledges: plates, floor, walls, bin
 * dividers, 39 diamond pegs) with balls, small boxes and small hulls */
static int run_galton(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    geo_scene_t sc;
    if (!geo_scene_init(&sc, dt)) return 1;
    const double fd = 0.3, ed = 0.4;

    ivp_compact_ledge_t *ledges[64];
    int n = 0;
    ledges[n++] = geo_box_ledge(3.4, 3.0, 0.1, 0.0, -2.0, -0.6);  /* back plate */
    ledges[n++] = geo_box_ledge(3.4, 3.0, 0.1, 0.0, -2.0, 0.6);   /* front plate */
    ledges[n++] = geo_box_ledge(3.4, 0.1, 0.7, 0.0, 1.0, 0.0);    /* floor */
    ledges[n++] = geo_box_ledge(0.1, 3.0, 0.7, -3.3, -2.0, 0.0);  /* side walls */
    ledges[n++] = geo_box_ledge(0.1, 3.0, 0.7, 3.3, -2.0, 0.0);
    for (int i = 0; i < 5; ++i)                                     /* bin dividers */
        ledges[n++] = geo_box_ledge(0.03, 0.4, 0.5, -2.0 + 1.0 * (double)i, 0.5, 0.0);
    for (int row = 0; row < 6; ++row) {                            /* diamond pegs */
        int n_pegs = (row % 2 == 0) ? 7 : 6;
        double x0 = (row % 2 == 0) ? -2.4 : -2.0;
        double py = -3.5 + 0.7 * (double)row;
        for (int i = 0; i < n_pegs; ++i) {
            double px = x0 + 0.8 * (double)i;
            double c[8 * 3];
            int k = 0;
            for (int s = -1; s <= 1; s += 2) {
                double z = (double)s * 0.5;
                c[k++] = px + 0.15; c[k++] = py;        c[k++] = z;
                c[k++] = px;        c[k++] = py + 0.15; c[k++] = z;
                c[k++] = px - 0.15; c[k++] = py;        c[k++] = z;
                c[k++] = px;        c[k++] = py - 0.15; c[k++] = z;
            }
            ledges[n++] = ivp_build_convex_ledge_d(NULL, c, 8);
        }
    }
    const ivp_surman_t *board = geo_scene_surman(&sc, geo_compile_soup(ledges, n, false));
    geo_scene_add(&sc, geo_static(&sc, board, 0.4, 0.3, ivp_quatr_identity(), 0.0, 0.0, 0.0), "board");

    const ivp_quatr_t q_a = ivp_quatr_make(0.20521208599540122, 0.273616114660535, 0.0, 0.9396926207859084);
    for (int i = 0; i < 12; ++i) {
        double x = -0.3 + 0.17 * (double)(i % 5);
        double y = -4.6 - 0.45 * (double)i;
        double z = -0.1 + 0.05 * (double)(i % 3);
        if (i % 6 == 2)
            geo_scene_add(&sc, geo_box(&sc, fd, ed, 0.12, 0.12, 0.12, 0.2, 0.00192, 0.00192, 0.00192, q_a, x, y, z),
                          "box");
        else if (i % 6 == 5)
            geo_scene_add(&sc, geo_dynamic(&sc, geo_hull(&sc, GEO_SMALL_TETRA, 4), fd, ed, 0.15, NULL, q_a, x, y, z),
                          "hull");
        else
            geo_scene_add(&sc, geo_ball(&sc, fd, ed, 0.18, 0.3, 0.003888, x, y, z), "ball");
    }

    int rc = geo_run(&sc, out, SCENARIO_GALTON, steps, dt);
    if (rc) return rc;
    geo_check_contained(&sc, inv, "galton", 1.0);
    geo_scene_destroy(&sc);
    return 0;
}

/* fast_impacts: fast and fast spinning projectiles (ball, box, hull,
 * dumbbell compound, needle) inside a closed room of thin plates */
static int run_fast_impacts(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    geo_scene_t sc;
    if (!geo_scene_init(&sc, dt)) return 1;
    const double fd = 0.4, ed = 0.6;

    static const double tilted[8 * 3] = {
        -1.5, 0.5, -2.4,   -1.5, 0.5, 2.4,   -3.0, -1.5, -2.4,   -3.0, -1.5, 2.4,
        -1.54, 0.53, -2.4, -1.54, 0.53, 2.4, -3.04, -1.47, -2.4, -3.04, -1.47, 2.4,
    };
    ivp_compact_ledge_t *ledges[8];
    ledges[0] = geo_box_ledge(4.0, 0.025, 2.5, 0.0, 0.5, 0.0);     /* floor */
    ledges[1] = geo_box_ledge(4.0, 0.025, 2.5, 0.0, -3.0, 0.0);    /* ceiling */
    ledges[2] = geo_box_ledge(0.025, 1.75, 2.5, 4.0, -1.25, 0.0);  /* walls */
    ledges[3] = geo_box_ledge(0.025, 1.75, 2.5, -4.0, -1.25, 0.0);
    ledges[4] = geo_box_ledge(4.0, 1.75, 0.025, 0.0, -1.25, 2.5);
    ledges[5] = geo_box_ledge(4.0, 1.75, 0.025, 0.0, -1.25, -2.5);
    ledges[6] = geo_box_ledge(0.025, 0.8, 1.2, 1.5, -0.3, 0.0);    /* inner wall */
    ledges[7] = ivp_build_convex_ledge_d(NULL, tilted, 8);
    const ivp_surman_t *plates = geo_scene_surman(&sc, geo_compile_soup(ledges, 8, false));
    const ivp_quatr_t q_ident = ivp_quatr_identity();
    geo_scene_add(&sc, geo_static(&sc, plates, 0.5, 0.5, q_ident, 0.0, 0.0, 0.0), "plates");

    ivp_object_t *o = geo_ball(&sc, fd, ed, 0.1, 0.2, 0.0008, -1.0, -0.5, 0.0);
    if (o) ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(40.0), IVP_REAL(-2.0), IVP_REAL(0.0));
    geo_scene_add(&sc, o, "ball");

    const ivp_quatr_t q_a = ivp_quatr_make(0.20521208599540122, 0.273616114660535, 0.0, 0.9396926207859084);
    o = geo_box(&sc, fd, ed, 0.1, 0.1, 0.1, 0.3, 0.002, 0.002, 0.002, q_a, 0.0, -1.0, 0.5);
    if (o) {
        ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(-35.0), IVP_REAL(-5.0), IVP_REAL(1.0));
        o->physical_core->rot_speed.k[0] = 10.0f;
        o->physical_core->rot_speed.k[1] = 0.0f;
        o->physical_core->rot_speed.k[2] = 5.0f;
    }
    geo_scene_add(&sc, o, "box");

    o = geo_dynamic(&sc, geo_hull(&sc, GEO_TETRA, 4), fd, ed, 0.5, NULL, q_ident, 1.0, -2.0, -1.0);
    if (o) ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(20.0), IVP_REAL(25.0), IVP_REAL(0.0));
    geo_scene_add(&sc, o, "hull");

    o = geo_dynamic(&sc, geo_scene_surman(&sc, geo_dumbbell_surface()), fd, ed, 3.0, NULL,
                    ivp_quatr_make(0.0, 0.0, 0.21643961393810288, 0.9762960071199334), -1.0, -1.5, -1.5);
    if (o) {
        ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(0.0), IVP_REAL(15.0), IVP_REAL(0.0));
        o->physical_core->rot_speed.k[0] = 0.0f;
        o->physical_core->rot_speed.k[1] = 0.0f;
        o->physical_core->rot_speed.k[2] = 40.0f;
    }
    geo_scene_add(&sc, o, "compound");

    double needle[12 * 3];
    geo_needle_points(needle);
    o = geo_dynamic(&sc, geo_hull(&sc, needle, 12), fd, ed, 0.2, NULL,
                    ivp_quatr_make(0.24399876718044458, 0.24399876718044458, 0.24399876718044458,
                                   0.9063077870366499), 1.5, -2.5, 1.5);
    if (o) {
        ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(-10.0), IVP_REAL(-20.0), IVP_REAL(0.0));
        o->physical_core->rot_speed.k[0] = 0.0f;
        o->physical_core->rot_speed.k[1] = 20.0f;
        o->physical_core->rot_speed.k[2] = 30.0f;
    }
    geo_scene_add(&sc, o, "needle");

    o = geo_ball(&sc, fd, ed, 0.25, 1.0, 0.025, -2.0, -2.0, 1.0);
    if (o) ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(-30.0), IVP_REAL(10.0), IVP_REAL(-2.0));
    geo_scene_add(&sc, o, "ball");

    int rc = geo_run(&sc, out, SCENARIO_FAST_IMPACTS, steps, dt);
    if (rc) return rc;
    geo_check_contained(&sc, inv, "fast_impacts", 0.5);
    geo_scene_destroy(&sc);
    return 0;
}

/* spawn_remove: objects created and deleted during the simulation.  Spawned
 * objects are appended to the snapshot list; the deleted objects (middle
 * box of a stack at step 300, a hull resting on the dumbbell at step 450)
 * are never listed. */
static int run_spawn_remove(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    geo_scene_t sc;
    if (!geo_scene_init(&sc, dt)) return 1;
    const double fd = 0.5, ed = 0.2;
    const ivp_quatr_t q_ident = ivp_quatr_identity();

    geo_scene_add(&sc, geo_static(&sc, geo_scene_surman(&sc, ref_box_surface(20.0, 0.5, 20.0)), 0.8, 0.0,
                                  q_ident, 0.0, 2.0, 0.0), "ground");
    ivp_compact_ledge_t *ledges[3];
    ledges[0] = geo_box_ledge(1.5, 0.25, 1.0, 0.0, 1.25, 0.0);
    ledges[1] = geo_box_ledge(1.0, 0.25, 1.0, 0.5, 0.75, 0.0);
    ledges[2] = geo_box_ledge(0.5, 0.25, 1.0, 1.0, 0.25, 0.0);
    geo_scene_add(&sc, geo_static(&sc, geo_scene_surman(&sc, geo_compile_soup(ledges, 3, false)), 0.8, 0.0,
                                  q_ident, 4.0, 0.0, 0.0), "stairs");

    geo_scene_add(&sc, geo_box(&sc, fd, ed, 0.4, 0.4, 0.4, 1.0, 0.1067, 0.1067, 0.1067, q_ident, -2.0, 1.1, 0.0),
                  "box");
    ivp_object_t *middle = geo_box(&sc, fd, ed, 0.4, 0.4, 0.4, 1.0, 0.1067, 0.1067, 0.1067, q_ident, -2.0, 0.3, 0.0);
    geo_scene_add(&sc, geo_box(&sc, fd, ed, 0.4, 0.4, 0.4, 1.0, 0.1067, 0.1067, 0.1067, q_ident, -2.0, -0.5, 0.0),
                  "box");
    geo_scene_add(&sc, geo_dynamic(&sc, geo_scene_surman(&sc, geo_dumbbell_surface()), fd, ed, 3.0, NULL, q_ident,
                                   0.5, 1.15, 2.5), "compound");
    ivp_object_t *hull = geo_dynamic(&sc, geo_hull(&sc, GEO_TETRA, 4), fd, ed, 0.5, NULL, q_ident, 0.5, -0.2, 2.5);
    geo_scene_add(&sc, geo_ball(&sc, fd, ed, 0.3, 0.6, 0.0216, 4.0, -1.0, 0.2), "ball");
    if (!middle || !hull) sc.failed = true;
    if (sc.failed) {
        geo_scene_destroy(&sc);
        return 1;
    }

    snapshot_frame(out, SCENARIO_SPAWN_REMOVE, 0, dt, sc.env, sc.objects, sc.types, sc.count);
    for (int step = 1; step <= steps; step++) {
        /* ScenarioResources::step_hook of the reference runner */
        if (step == 200) {
            geo_scene_add(&sc, geo_dynamic(&sc, geo_hull(&sc, GEO_ROCK, 10), fd, ed, 1.3, NULL,
                                           ivp_quatr_make(0.24399876718044458, 0.24399876718044458,
                                                          0.24399876718044458, 0.9063077870366499),
                                           -2.0, -2.5, 0.1), "rock");
        }
        if (step == 300 && middle) {
            ivp_environment_destroy_object(sc.env, middle);   /* delete_and_check_vicinity */
            middle = NULL;
        }
        if (step == 400) {
            ivp_object_t *l = geo_dynamic(&sc, geo_scene_surman(&sc, geo_l_surface()), fd, ed, 2.0, NULL,
                                          ivp_quatr_make(0.15529142706151244, 0.0, 0.2070552360820166,
                                                         0.9659258262890683),
                                          4.3, -1.6, 0.3);
            if (l) ivp_vec3r_set(&l->physical_core->speed, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));
            geo_scene_add(&sc, l, "compound");
        }
        if (step == 450 && hull) {
            ivp_environment_destroy_object(sc.env, hull);
            hull = NULL;
        }
        if (step == 550) {
            ivp_object_t *b = geo_ball(&sc, fd, ed, 0.2, 0.3, 0.0048, -4.0, 0.9, 0.0);
            if (b) ivp_vec3r_set(&b->physical_core->speed, IVP_REAL(15.0), IVP_REAL(0.0), IVP_REAL(0.0));
            geo_scene_add(&sc, b, "ball");
        }
        if (sc.failed) {
            geo_scene_destroy(&sc);
            return 1;
        }
        scenario_step(sc.env, dt);
        snapshot_frame(out, SCENARIO_SPAWN_REMOVE, step, dt, sc.env, sc.objects, sc.types, sc.count);
    }

    geo_check_contained(&sc, inv, "spawn_remove", 1.5);
    geo_scene_destroy(&sc);
    return 0;
}

/* long_range: long throws over a huge floor into a far concave catcher and
 * onto a far, rotated compact grid (rows -> -z, columns -> x, field 2) */
static void geo_bump_heights(float *h)
{
    for (int r = 0; r < GEO_GRID_N; ++r)
        for (int c = 0; c < GEO_GRID_N; ++c)
            h[r * GEO_GRID_N + c] = 0.15f * (float)((r * 7 + c * 3) % 5);
}

static int run_long_range(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    geo_scene_t sc;
    if (!geo_scene_init(&sc, dt)) return 1;
    const double fs = 0.6, es = 0.1, fd = 0.4, ed = 0.3;
    const ivp_quatr_t q_ident = ivp_quatr_identity();

    geo_scene_add(&sc, geo_static(&sc, geo_scene_surman(&sc, ref_box_surface(300.0, 0.5, 300.0)), fs, es,
                                  q_ident, 0.0, 2.0, 0.0), "ground");
    ivp_compact_ledge_t *ledges[3];
    ledges[0] = geo_box_ledge(0.5, 3.0, 6.0, 6.5, -1.5, 0.0);
    ledges[1] = geo_box_ledge(6.5, 3.0, 0.5, 0.0, -1.5, 6.5);
    ledges[2] = geo_box_ledge(6.5, 3.0, 0.5, 0.0, -1.5, -6.5);
    geo_scene_add(&sc, geo_static(&sc, geo_scene_surman(&sc, geo_compile_soup(ledges, 3, false)), fs, es,
                                  q_ident, 118.5, 0.0, 0.0), "catcher");

    float heights[GEO_GRID_N * GEO_GRID_N];
    geo_bump_heights(heights);
    ivp_template_compact_grid_t gt;
    ivp_template_compact_grid_init(&gt);
    gt.row_info.n_points = GEO_GRID_N;
    gt.row_info.maps_to = 2;     /* IVP_INDEX_Z */
    gt.row_info.invert_axis = true;
    gt.column_info.n_points = GEO_GRID_N;
    gt.column_info.maps_to = 0;  /* IVP_INDEX_X */
    gt.column_info.invert_axis = false;
    gt.height_maps_to = 1;       /* IVP_INDEX_Y */
    gt.height_invert_axis = true;
    gt.grid_field_size = 2.0f;
    gt.position_origin_os[0] = 1.0f;
    gt.position_origin_os[1] = 0.5f;
    gt.position_origin_os[2] = 2.0f;
    sc.grid = ivp_gridbuild_array_to_compact_grid(NULL, &gt, heights);
    if (!sc.grid) {
        geo_scene_destroy(&sc);
        return 1;
    }
    ivp_surman_compact_grid_init(&sc.grid_sm, sc.grid);
    geo_scene_add(&sc, geo_static(&sc, &sc.grid_sm.base, fs, es,
                                  ivp_quatr_make(0.0, 0.7071067811865475, 0.0, 0.7071067811865476),
                                  -88.0, 0.9, 0.0), "grid");

    ivp_object_t *o = geo_ball(&sc, fd, ed, 0.5, 1.0, 0.1, 0.0, -1.0, 0.0);
    if (o) ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(35.0), IVP_REAL(-12.0), IVP_REAL(1.0));
    geo_scene_add(&sc, o, "ball");

    o = geo_box(&sc, fd, ed, 0.4, 0.4, 0.4, 1.0, 0.1067, 0.1067, 0.1067,
                ivp_quatr_make(0.20521208599540122, 0.273616114660535, 0.0, 0.9396926207859084), 0.0, -1.0, 2.0);
    if (o) ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(40.0), IVP_REAL(-10.0), IVP_REAL(-2.0));
    geo_scene_add(&sc, o, "box");

    o = geo_dynamic(&sc, geo_hull(&sc, GEO_ROCK, 10), fd, ed, 1.3, NULL,
                    ivp_quatr_make(0.24399876718044458, 0.24399876718044458, 0.24399876718044458,
                                   0.9063077870366499), 0.0, -1.0, -1.0);
    if (o) ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(-38.0), IVP_REAL(-11.0), IVP_REAL(-4.0));
    geo_scene_add(&sc, o, "rock");

    o = geo_dynamic(&sc, geo_scene_surman(&sc, geo_dumbbell_surface()), fd, ed, 3.0, NULL, q_ident, 0.0, -1.0, -3.0);
    if (o) {
        ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(-34.0), IVP_REAL(-14.0), IVP_REAL(-3.0));
        o->physical_core->rot_speed.k[0] = 0.0f;
        o->physical_core->rot_speed.k[1] = 3.0f;
        o->physical_core->rot_speed.k[2] = 6.0f;
    }
    geo_scene_add(&sc, o, "compound");

    o = geo_ball(&sc, fd, ed, 0.3, 0.6, 0.0216, 2.0, -1.0, 0.0);
    if (o) ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(2.0), IVP_REAL(-9.0), IVP_REAL(30.0));
    geo_scene_add(&sc, o, "ball");

    o = geo_dynamic(&sc, geo_scene_surman(&sc, geo_l_surface()), fd, ed, 2.0, NULL,
                    ivp_quatr_make(0.15529142706151244, 0.0, 0.2070552360820166, 0.9659258262890683),
                    0.0, -1.0, 4.0);
    if (o) ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(0.0), IVP_REAL(-20.0), IVP_REAL(0.0));
    geo_scene_add(&sc, o, "compound");

    o = geo_dynamic(&sc, geo_hull(&sc, GEO_TETRA, 4), fd, ed, 0.5, NULL, q_ident, -2.0, -1.0, 0.0);
    if (o) ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(30.0), IVP_REAL(-15.0), IVP_REAL(0.0));
    geo_scene_add(&sc, o, "hull");

    int rc = geo_run(&sc, out, SCENARIO_LONG_RANGE, steps, dt);
    if (rc) return rc;
    geo_check_contained(&sc, inv, "long_range", 1.5);
    geo_scene_destroy(&sc);
    return 0;
}

/* ── Miscellaneous parity scenarios ──────────────────────────────────────
 *
 * Mirror tools/ivp_reference_runner/ref_runner_scenarios_misc.cxx (keep in
 * sync).
 *
 * universe_evict: a custom universe manager streams static tiles in
 * (ensure_fn = IVP_Universe_Manager::ensure_objects_in_environment) and out
 * (evict_fn = object_no_longer_needed) while balls and boxes cross a 5 x 5
 * tile field.  A tile is deleted when nobody checks it for collision; its
 * next height depends on the index of that eviction and on how often the
 * second level check (one collision partner) visited it, i.e. on the order
 * in which IVP_Cluster_Manager::check_for_unused_objects walks the cluster
 * tree. */

enum { UE_N = 5, UE_SLOTS = UE_N * UE_N, UE_HEIGHTS = 5 };

typedef struct ue_slot {
    ivp_object_t *obj;
    int generation;   /* index of the eviction that removed the tile last */
    int touches;      /* second level checks (one collision partner) */
    double x, z;
} ue_slot_t;

typedef struct ue_scene {
    geo_scene_t *sc;
    const ivp_surman_t *surmans[UE_HEIGHTS];
    ue_slot_t slots[UE_SLOTS];
    int evict_counter;
} ue_scene_t;

static double ue_half_height(int h) { return 0.02 + 0.02 * (double)h; }

static void ue_create_tile(ue_scene_t *ue, int i)
{
    ue_slot_t *s = &ue->slots[i];
    int h = (s->generation + s->touches) % UE_HEIGHTS;
    s->obj = geo_static(ue->sc, ue->surmans[h], 0.3, 0.5, ivp_quatr_identity(), s->x, -ue_half_height(h), s->z);
    if (!s->obj) ue->sc->failed = true;
}

static void ue_ensure(void *ctx, ivp_object_t *object, const float *center, double radius)
{
    (void)object;
    ue_scene_t *ue = (ue_scene_t *)ctx;
    const double reach = radius + 1.0; /* tile half size 0.9 */
    for (int i = 0; i < UE_SLOTS; i++) {
        ue_slot_t *s = &ue->slots[i];
        if (s->obj) continue;
        const double dx = s->x - (double)center[0];
        const double dz = s->z - (double)center[2];
        if (dx * dx + dz * dz > reach * reach) continue;
        ue_create_tile(ue, i);
    }
}

static void ue_evict(void *ctx, ivp_object_t *object)
{
    ue_scene_t *ue = (ue_scene_t *)ctx;
    for (int i = 0; i < UE_SLOTS; i++) {
        ue_slot_t *s = &ue->slots[i];
        if (s->obj != object) continue;
        if (ivp_object_get_collision_check_reference_count(object) == 0) {
            s->obj = NULL;
            s->generation = ue->evict_counter++;
            ivp_environment_destroy_object(ue->sc->env, object); /* delete_and_check_vicinity */
        } else {
            s->touches++;
        }
        return;
    }
}

static void ue_deleted(void *ctx, ivp_object_t *object)
{
    ue_scene_t *ue = (ue_scene_t *)ctx;
    for (int i = 0; i < UE_SLOTS; i++) {
        if (ue->slots[i].obj == object) ue->slots[i].obj = NULL;
    }
}

static int run_universe_evict(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    geo_scene_t sc;
    if (!geo_scene_init(&sc, dt)) return 1;
    ue_scene_t ue;
    memset(&ue, 0, sizeof(ue));
    ue.sc = &sc;
    for (int h = 0; h < UE_HEIGHTS; h++) {
        ue.surmans[h] = geo_scene_surman(&sc, ref_box_surface(0.9, ue_half_height(h), 0.9));
    }
    const ivp_quatr_t q_ident = ivp_quatr_identity();
    geo_scene_add(&sc, geo_static(&sc, geo_scene_surman(&sc, ref_box_surface(12.0, 0.5, 12.0)), 0.3, 0.3,
                                  q_ident, 0.0, 0.5, 0.0), "ground");

    /* the full tile field; initial heights by slot */
    for (int i = 0; i < UE_SLOTS; i++) {
        ue_slot_t *s = &ue.slots[i];
        s->generation = i;
        s->x = -6.0 + 3.0 * (double)(i % UE_N);
        s->z = -6.0 + 3.0 * (double)(i / UE_N);
        ue_create_tile(&ue, i);
    }

    const double fd = 0.2, ed = 0.5;
    ivp_object_t *o = geo_ball(&sc, fd, ed, 0.3, 1.0, 0.036, -7.5, -0.6, -6.0);
    if (o) ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(8.0), IVP_REAL(0.0), IVP_REAL(1.5));
    geo_scene_add(&sc, o, "ball");
    o = geo_ball(&sc, fd, ed, 0.25, 0.6, 0.015, 7.0, -0.8, -3.0);
    if (o) ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(-8.5), IVP_REAL(-2.0), IVP_REAL(0.75));
    geo_scene_add(&sc, o, "ball");
    o = geo_ball(&sc, fd, ed, 0.35, 1.5, 0.0735, 0.0, -1.5, 7.5);
    if (o) ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(-9.0));
    geo_scene_add(&sc, o, "ball");
    o = geo_box(&sc, fd, ed, 0.3, 0.3, 0.3, 1.0, 0.06, 0.06, 0.06, q_ident, -6.0, -0.5, 6.0);
    if (o) ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(7.0), IVP_REAL(-1.0), IVP_REAL(-6.0));
    geo_scene_add(&sc, o, "box");
    o = geo_box(&sc, fd, ed, 0.4, 0.2, 0.3, 1.2, 0.052, 0.1, 0.08,
                ivp_quatr_make(0.0, 0.25881904510252074, 0.0, 0.9659258262890683), 6.0, -0.6, 6.0);
    if (o) ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(-6.5), IVP_REAL(-1.5), IVP_REAL(-6.5));
    geo_scene_add(&sc, o, "box");

    ivp_universe_manager_t um;
    ivp_universe_manager_init(&um);
    um.ensure_fn = ue_ensure;
    um.evict_fn = ue_evict;
    um.deleted_fn = ue_deleted;
    um.ctx = &ue;
    um.num_objects_in_environment_threshold_0 = 1;
    um.check_objects_per_second_threshold_0 = 450; /* int(450 / 180) + 1 = 3 checks per PSI */
    um.num_objects_in_environment_threshold_1 = 14;
    um.check_objects_per_second_threshold_1 = 630; /* 4 checks per PSI */
    if (!sc.failed) ivp_environment_set_universe_manager(sc.env, &um);

    int rc = geo_run(&sc, out, SCENARIO_UNIVERSE_EVICT, steps, dt);
    if (rc == 0) {
        if (inv) {
            int present = 0;
            for (int i = 0; i < UE_SLOTS; i++) present += ue.slots[i].obj != NULL;
            inv_check(inv, ue.evict_counter > 0 && present < UE_SLOTS,
                      "universe_evict evicted tiles: evictions=%d tiles=%d", ue.evict_counter, present);
            for (int i = 0; i < sc.count; i++) {
                const ivp_vec3r_t *p = &sc.objects[i]->physical_core->pos_world_f_core_last_psi;
                inv_check(inv, isfinite((double)p->k[0]) && isfinite((double)p->k[1]) && isfinite((double)p->k[2]),
                          "universe_evict object %d position finite", i);
            }
        }
        /* the environment outlives the manager (reference cleanup hook) */
        ivp_environment_set_universe_manager(sc.env, NULL);
        geo_scene_destroy(&sc);
    }
    return rc;
}

/* ── Controller coverage scenarios (controllers2) ────────────────────────
 *
 * Mirror tools/ivp_reference_runner/ref_runner_scenarios_controllers2.cxx
 * (keep in sync): torque, stabilizer, floating, world_friction, golem,
 * fixed_keyed, hinge_limits, cardan_tense, slider_limits, constraint_break,
 * attacher and actuator_extra.  Objects and controllers are created in the
 * reference order with the reference literals; per-step inputs are applied
 * before simulating the given step (reference ScenarioResources::step_hook). */

enum { C2_MAX_OBJECTS = 16, C2_MAX_SURFACES = 16 };

typedef struct c2_scene {
    ivp_environment_t *env;
    ivp_compact_surface_t *surfaces[C2_MAX_SURFACES];
    ivp_surman_polygon_t surmans[C2_MAX_SURFACES];
    int n_surfaces;
    ivp_object_t *objects[C2_MAX_OBJECTS];
    const char *types[C2_MAX_OBJECTS];
    int count;
    bool failed;
} c2_scene_t;

static const double C2_GROUND_TOP = 2.5; /* ground box (hy 0.5) centred at y = 3 */

static bool c2_scene_init(c2_scene_t *sc, double dt)
{
    memset(sc, 0, sizeof(*sc));
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), SCENARIO_REF_GRAVITY, IVP_REAL(0.0));
    sc->env = ivp_environment_create(&cfg);
    return sc->env != NULL;
}

static void c2_scene_destroy(c2_scene_t *sc)
{
    if (sc->env) ivp_environment_destroy(sc->env);
    for (int i = 0; i < sc->n_surfaces; i++) ivp_compact_surface_free(NULL, sc->surfaces[i]);
}

/* reference build_box_compact_surface + IVP_SurfaceManager_Polygon */
static const ivp_surman_t *c2_box_surman(c2_scene_t *sc, double hx, double hy, double hz)
{
    ivp_compact_surface_t *s = ref_box_surface(hx, hy, hz);
    if (!s || sc->n_surfaces >= C2_MAX_SURFACES) {
        if (s) ivp_compact_surface_free(NULL, s);
        sc->failed = true;
        return NULL;
    }
    int i = sc->n_surfaces++;
    sc->surfaces[i] = s;
    ivp_surman_polygon_init(&sc->surmans[i], s);
    return &sc->surmans[i].base;
}

static void c2_add(c2_scene_t *sc, ivp_object_t *obj, const char *type)
{
    if (!obj || sc->count >= C2_MAX_OBJECTS) {
        sc->failed = true;
        return;
    }
    sc->objects[sc->count] = obj;
    sc->types[sc->count] = type;
    sc->count++;
}

/* reference set_quat_axis_angle */
static ivp_quatr_t c2_quat(double ax, double ay, double az, double radians)
{
    const double half = 0.5 * radians;
    const double s = sin(half);
    const double c = cos(half);
    return ivp_quatr_make((ivp_real_t)(ax * s), (ivp_real_t)(ay * s), (ivp_real_t)(az * s), (ivp_real_t)c);
}

/* reference create_static_box */
static ivp_object_t *c2_static(c2_scene_t *sc, double friction, double elasticity,
                               double hx, double hy, double hz, double x, double y, double z)
{
    const ivp_surman_t *sm = c2_box_surman(sc, hx, hy, hz);
    if (!sm) return NULL;
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false;
    tmpl.material.friction = (ivp_real_t)friction;
    tmpl.material.elasticity = (ivp_real_t)elasticity;
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, (ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
    return ivp_environment_create_object(sc->env, &tmpl, sm);
}

static ivp_object_t *c2_ground(c2_scene_t *sc, double friction)
{
    return c2_static(sc, friction, 0.0, 20.0, 0.5, 20.0, 0.0, 3.0, 0.0);
}

/* reference create_dynamic_box (explicit inertia, no damping) */
static ivp_object_t *c2_box(c2_scene_t *sc, double friction, double elasticity,
                            double hx, double hy, double hz, double mass,
                            double ix, double iy, double iz,
                            double x, double y, double z, const ivp_quatr_t *q)
{
    const ivp_surman_t *sm = c2_box_surman(sc, hx, hy, hz);
    if (!sm) return NULL;
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = (ivp_real_t)mass;
    ivp_vec3r_set(&tmpl.rot_inertia, (ivp_real_t)ix, (ivp_real_t)iy, (ivp_real_t)iz);
    tmpl.rot_inertia_is_factor = false;
    tmpl.material.friction = (ivp_real_t)friction;
    tmpl.material.elasticity = (ivp_real_t)elasticity;
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = q ? *q : ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, (ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
    return ivp_environment_create_object(sc->env, &tmpl, sm);
}

/* reference create_dynamic_ball */
static ivp_object_t *c2_ball(c2_scene_t *sc, double friction, double elasticity,
                             double radius, double mass, double inertia, double x, double y, double z)
{
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = (ivp_real_t)mass;
    ivp_vec3r_set(&tmpl.rot_inertia, (ivp_real_t)inertia, (ivp_real_t)inertia, (ivp_real_t)inertia);
    tmpl.rot_inertia_is_factor = false;
    tmpl.material.friction = (ivp_real_t)friction;
    tmpl.material.elasticity = (ivp_real_t)elasticity;
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, (ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
    tmpl.ball_radius = (ivp_real_t)(float)radius; /* IVP_Template_Ball::radius is IVP_FLOAT */
    ivp_object_t *o = ivp_environment_create_object(sc->env, &tmpl, NULL);
    if (!o) sc->failed = true;
    return o;
}

/* core->speed.set(x, y, z) (IVP_U_Float_Point) */
static void c2_set_speed(ivp_object_t *o, float x, float y, float z)
{
    ivp_vec3r_set(&o->physical_core->speed, (ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
}

static void c2_set_rot_speed(ivp_object_t *o, float x, float y, float z)
{
    o->physical_core->rot_speed.k[0] = x;
    o->physical_core->rot_speed.k[1] = y;
    o->physical_core->rot_speed.k[2] = z;
}

/* reference add_spring (plain IVP_Actuator_Spring) */
static void c2_spring(ivp_environment_t *env, ivp_spring_state_t *s,
                      ivp_object_t *a, double ax, double ay, double az,
                      ivp_object_t *b, double bx, double by, double bz,
                      double constant, double damp, double len)
{
    ivp_vec3r_t pa = ivp_vec3r((ivp_real_t)ax, (ivp_real_t)ay, (ivp_real_t)az);
    ivp_vec3r_t pb = ivp_vec3r((ivp_real_t)bx, (ivp_real_t)by, (ivp_real_t)bz);
    ivp_spring_state_setup(s, a, &pa, b, &pb, (ivp_real_t)constant, (ivp_real_t)damp, (ivp_real_t)len);
    ivp_environment_announce_dependent_controller(env, 100, ivp_controller_spring, s, ivp_spring_discovery);
}

/* IVP_Controller_Independent::add_controller_to_core */
static ivp_controller_handle_t c2_core_controller(ivp_environment_t *env, int priority,
                                                  ivp_controller_fn_t fn, void *ctx, ivp_object_t *obj)
{
    ivp_controller_handle_t h = ivp_environment_create_core_controller(env, priority, fn, ctx);
    ivp_environment_add_controller_to_core(env, h, obj);
    return h;
}

/* reference IVP_Controller_Factory::create_constraint */
static void c2_constraint(ivp_environment_t *env, ivp_constraint_state_t *c, const ivp_constraint_template_t *t)
{
    ivp_constraint_init_from_template(c, t);
    ivp_environment_announce_dependent_controller(env, 0, ivp_controller_constraint, c, ivp_constraint_discovery);
}

typedef void (*c2_hook_fn_t)(int step, ivp_environment_t *env, void *ctx);

static int c2_run(c2_scene_t *sc, FILE *out, int scenario_id, int steps, double dt,
                  c2_hook_fn_t hook, void *ctx)
{
    if (sc->failed) return 1;
    snapshot_frame(out, scenario_id, 0, dt, sc->env, sc->objects, sc->types, sc->count);
    for (int step = 1; step <= steps; step++) {
        if (hook) hook(step, sc->env, ctx);
        scenario_step(sc->env, dt);
        snapshot_frame(out, scenario_id, step, dt, sc->env, sc->objects, sc->types, sc->count);
    }
    return 0;
}

static void c2_check_finite(const c2_scene_t *sc, invariant_result_t *inv, const char *name)
{
    if (!inv) return;
    for (int i = 0; i < sc->count; i++) {
        const ivp_vec3r_t *p = &sc->objects[i]->physical_core->pos_world_f_core_last_psi;
        inv_check(inv, isfinite((double)p->k[0]) && isfinite((double)p->k[1]) && isfinite((double)p->k[2]) &&
                       fabs((double)p->k[1]) < 100.0,
                  "%s object %d (%s) position finite and bounded", name, i, sc->types[i]);
    }
}

/* ── torque: IVP_Actuator_Torque ─────────────────────────────────────── */

typedef struct c2_torque {
    ivp_torque_actuator_state_t spinner, roller, tumbler;
} c2_torque_t;

static void c2_add_torque(ivp_environment_t *env, ivp_torque_actuator_state_t *st, ivp_object_t *obj,
                          double ax, double ay, double az, double torque, double max_speed)
{
    ivp_vec3r_t a0 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t a1 = ivp_vec3r((ivp_real_t)ax, (ivp_real_t)ay, (ivp_real_t)az);
    ivp_torque_actuator_state_setup_anchors(st, obj, &a0, &a1, (ivp_real_t)torque, (ivp_real_t)max_speed);
    ivp_environment_add_controller(env, IVP_CP_ACTUATOR, ivp_controller_torque_actuator, st);
}

static void c2_torque_hook(int step, ivp_environment_t *env, void *ctx)
{
    (void)env;
    c2_torque_t *s = (c2_torque_t *)ctx;
    if (step == 241) {
        ivp_torque_actuator_set_torque(&s->roller, IVP_REAL(-2.5));
        ivp_torque_actuator_set_max_rotation_speed(&s->spinner, IVP_REAL(8.0));
    }
    if (step == 361) ivp_torque_actuator_set_torque(&s->tumbler, IVP_REAL(0.0));
}

static int run_torque(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    c2_torque_t s;
    c2_add(&sc, c2_ground(&sc, 0.8), "ground");
    ivp_object_t *spinner = c2_box(&sc, 0.5, 0.1, 0.8, 0.3, 0.8, 2.0, 0.5, 0.85, 0.5, -4.0, 2.15, 0.0, NULL);
    ivp_object_t *roller = c2_ball(&sc, 0.5, 0.1, 0.5, 2.0, 0.2, 0.0, 1.95, -3.0);
    ivp_quatr_t qt = c2_quat(1.0, 0.0, 0.0, 0.3);
    ivp_object_t *tumbler = c2_box(&sc, 0.5, 0.1, 0.4, 0.4, 0.4, 1.0, 0.11, 0.11, 0.11, 4.0, -2.0, 0.0, &qt);
    int rc = 1;
    if (!sc.failed) {
        c2_add_torque(sc.env, &s.spinner, spinner, 0.0, -1.0, 0.0, 12.0, 4.0);
        c2_add_torque(sc.env, &s.roller, roller, 0.0, 0.0, 1.0, 2.5, 6.0);
        c2_add_torque(sc.env, &s.tumbler, tumbler, 0.6, 0.6, 0.2, 1.5, 2.0);
        c2_add(&sc, spinner, "spinner");
        c2_add(&sc, roller, "roller");
        c2_add(&sc, tumbler, "tumbler");
        rc = c2_run(&sc, out, SCENARIO_TORQUE, steps, dt, c2_torque_hook, &s);
        if (inv && rc == 0) {
            c2_check_finite(&sc, inv, "torque");
            inv_check(inv, steps < 240 || fabs((double)spinner->physical_core->rot_speed.k[1]) > 1.0,
                      "torque spinner rotates: w_y=%.6f", (double)spinner->physical_core->rot_speed.k[1]);
        }
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── stabilizer: IVP_Actuator_Stabilizer (announced explicitly) ──────── */

static int run_stabilizer(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    ivp_spring_state_t springs[5];
    ivp_stabilizer_state_t stab[2];
    ivp_object_t *frame = c2_static(&sc, 0.8, 0.0, 2.0, 0.2, 0.3, 0.0, -8.0, 0.0);
    ivp_object_t *wl = c2_box(&sc, 0.5, 0.1, 0.3, 0.3, 0.3, 1.0, 0.06, 0.06, 0.06, -1.5, -5.5, 0.0, NULL);
    ivp_object_t *wr = c2_box(&sc, 0.5, 0.1, 0.3, 0.3, 0.3, 1.0, 0.06, 0.06, 0.06, 1.5, -4.5, 0.0, NULL);
    if (!sc.failed) {
        c2_spring(sc.env, &springs[0], frame, -1.5, 0.2, 0.0, wl, 0.0, -0.3, 0.0, 60.0, 2.0, 2.0);
        c2_spring(sc.env, &springs[1], frame, 1.5, 0.2, 0.0, wr, 0.0, -0.3, 0.0, 60.0, 2.0, 2.0);
    }
    ivp_object_t *hook = c2_static(&sc, 0.8, 0.0, 0.2, 0.2, 0.2, 0.0, -10.0, 6.0);
    ivp_object_t *bar = c2_box(&sc, 0.5, 0.1, 2.0, 0.15, 0.3, 2.0, 0.05, 0.7, 0.68, 0.0, -8.0, 6.0, NULL);
    ivp_object_t *wl2 = c2_box(&sc, 0.5, 0.1, 0.3, 0.3, 0.3, 0.8, 0.05, 0.05, 0.05, -1.8, -5.0, 6.0, NULL);
    ivp_object_t *wr2 = c2_box(&sc, 0.5, 0.1, 0.3, 0.3, 0.3, 1.2, 0.07, 0.07, 0.07, 1.8, -6.0, 6.0, NULL);
    int rc = 1;
    if (!sc.failed) {
        c2_spring(sc.env, &springs[2], hook, 0.0, 0.2, 0.0, bar, 0.0, -0.15, 0.0, 200.0, 10.0, 1.6);
        c2_spring(sc.env, &springs[3], bar, -1.8, 0.15, 0.0, wl2, 0.0, -0.3, 0.0, 50.0, 1.0, 2.2);
        c2_spring(sc.env, &springs[4], bar, 1.8, 0.15, 0.0, wr2, 0.0, -0.3, 0.0, 50.0, 1.0, 1.4);
        /* the reference IVP_Actuator_Four_Point never announces itself (a
         * factory stabilizer is inert); the reference runner announces it
         * explicitly, ivp_environment_add_controller does the same */
        const ivp_vec3r_t f_l = ivp_vec3r(IVP_REAL(-1.5), IVP_REAL(0.2), IVP_REAL(0.0));
        const ivp_vec3r_t f_r = ivp_vec3r(IVP_REAL(1.5), IVP_REAL(0.2), IVP_REAL(0.0));
        const ivp_vec3r_t b_l = ivp_vec3r(IVP_REAL(-1.8), IVP_REAL(0.15), IVP_REAL(0.0));
        const ivp_vec3r_t b_r = ivp_vec3r(IVP_REAL(1.8), IVP_REAL(0.15), IVP_REAL(0.0));
        const ivp_vec3r_t w_top = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.3), IVP_REAL(0.0));
        ivp_stabilizer_state_init(&stab[0]);
        ivp_stabilizer_state_setup(&stab[0], frame, wl, frame, wr, &f_l, &w_top, &f_r, &w_top, IVP_REAL(20.0));
        ivp_environment_add_controller(sc.env, IVP_CP_ACTUATOR, ivp_controller_stabilizer, &stab[0]);
        ivp_stabilizer_state_init(&stab[1]);
        ivp_stabilizer_state_setup(&stab[1], bar, wl2, bar, wr2, &b_l, &w_top, &b_r, &w_top, IVP_REAL(15.0));
        ivp_environment_add_controller(sc.env, IVP_CP_ACTUATOR, ivp_controller_stabilizer, &stab[1]);
        c2_add(&sc, frame, "frame");
        c2_add(&sc, wl, "weight");
        c2_add(&sc, wr, "weight");
        c2_add(&sc, hook, "hook");
        c2_add(&sc, bar, "bar");
        c2_add(&sc, wl2, "weight");
        c2_add(&sc, wr2, "weight");
        rc = c2_run(&sc, out, SCENARIO_STABILIZER, steps, dt, NULL, NULL);
        if (inv && rc == 0) c2_check_finite(&sc, inv, "stabilizer");
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── floating: IVP_Controller_Floating ──────────────────────────────── */

/* reference RunnerFloating::do_ray_casting: distance along the ray from the
 * anchor to the ground plane y = C2_GROUND_TOP */
static bool c2_ground_ray(void *ctx, const ivp_object_t *obj, const ivp_vec3r_t *ray_start_ws,
                          const ivp_vec3r_t *ray_dir_ws, ivp_real_t *out_distance)
{
    (void)ctx;
    (void)obj;
    *out_distance = (ivp_real_t)((C2_GROUND_TOP - (double)ray_start_ws->k[1]) / (double)ray_dir_ws->k[1]);
    return true;
}

/* IVP_Template_Controller_Floating + IVP_Controller_Floating constructor */
static ivp_controller_handle_t c2_add_floating(ivp_environment_t *env, ivp_floating_state_t *st, ivp_object_t *obj,
                                               float max_repulsive, float max_adhesive,
                                               double px, double py, double pz,
                                               const ivp_vec3r_t *ray_dir_ws, float target)
{
    ivp_floating_init_defaults(st);
    st->max_repulsive_force = (ivp_real_t)max_repulsive;
    st->max_adhesive_force = (ivp_real_t)max_adhesive;
    ivp_vec3r_t p = ivp_vec3r((ivp_real_t)px, (ivp_real_t)py, (ivp_real_t)pz);
    ivp_floating_set_position_os(st, &p);
    if (ray_dir_ws) ivp_floating_set_ray_direction_ws(st, ray_dir_ws);
    st->target_distance = (ivp_real_t)target;
    st->current_distance = IVP_REAL(0.0);
    ivp_floating_set_object(st, obj);
    ivp_floating_set_raycast(st, c2_ground_ray, NULL);
    return c2_core_controller(env, IVP_CP_FLOATING, ivp_controller_floating, st, obj);
}

typedef struct c2_floating {
    ivp_floating_state_t a, b, c;
} c2_floating_t;

static void c2_floating_hook(int step, ivp_environment_t *env, void *ctx)
{
    (void)env;
    c2_floating_t *s = (c2_floating_t *)ctx;
    if (step == 241) {
        ivp_floating_set_target_distance(&s->a, IVP_REAL(0.4));
        ivp_vec3r_t d = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), (ivp_real_t)0.2f); /* IVP_U_Float_Point */
        ivp_floating_set_ray_direction_ws(&s->b, &d);
    }
    if (step == 331) ivp_floating_set_target_distance(&s->c, IVP_REAL(0.8));
}

static int run_floating(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    c2_floating_t s;
    c2_add(&sc, c2_ground(&sc, 0.8), "ground");
    ivp_object_t *a = c2_box(&sc, 0.5, 0.1, 0.5, 0.5, 0.5, 1.0, 0.17, 0.17, 0.17, -4.0, 0.0, 0.0, NULL);
    ivp_quatr_t qb = c2_quat(0.0, 0.0, 1.0, 0.2);
    ivp_object_t *b = c2_box(&sc, 0.5, 0.1, 0.6, 0.25, 0.4, 2.0, 0.15, 0.35, 0.28, 0.0, 0.5, 0.0, &qb);
    ivp_object_t *c = c2_ball(&sc, 0.5, 0.1, 0.4, 1.5, 0.096, 4.0, -1.0, 0.0);
    int rc = 1;
    if (!sc.failed) {
        const ivp_vec3r_t dir_c = ivp_vec3r(IVP_REAL(0.3), IVP_REAL(1.0), IVP_REAL(0.0));
        c2_add_floating(sc.env, &s.a, a, 30.0f, 5.0f, 0.0, 0.5, 0.0, NULL, 1.0f);
        c2_add_floating(sc.env, &s.b, b, 40.0f, 40.0f, 0.4, 0.25, 0.2, NULL, 0.6f);
        c2_add_floating(sc.env, &s.c, c, 15.0f, 2.0f, 0.0, 0.0, 0.0, &dir_c, 1.5f);
        c2_add(&sc, a, "hover");
        c2_add(&sc, b, "hover");
        c2_add(&sc, c, "hover");
        rc = c2_run(&sc, out, SCENARIO_FLOATING, steps, dt, c2_floating_hook, &s);
        if (inv && rc == 0) {
            c2_check_finite(&sc, inv, "floating");
            inv_check(inv, (double)a->physical_core->pos_world_f_core_last_psi.k[1] < C2_GROUND_TOP - 0.5,
                      "floating box hovers above the ground: y=%.6f", (double)a->physical_core->pos_world_f_core_last_psi.k[1]);
        }
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── world_friction: IVP_Controller_World_Friction ──────────────────── */

typedef struct c2_world_friction {
    ivp_world_friction_state_t slider, floater, tilt;
} c2_world_friction_t;

static void c2_world_friction_hook(int step, ivp_environment_t *env, void *ctx)
{
    (void)env;
    c2_world_friction_t *s = (c2_world_friction_t *)ctx;
    if (step == 241) {
        const ivp_vec3r_t r = ivp_vec3r(IVP_REAL(2.0), IVP_REAL(0.1), IVP_REAL(5.0));
        ivp_world_friction_set_friction_value_rotation(&s->tilt, &r);
        const ivp_vec3r_t v = ivp_vec3r(IVP_REAL(-1.0), IVP_REAL(-0.5), IVP_REAL(0.5));
        ivp_world_friction_set_desired_speed_ws(&s->floater, &v);
    }
    if (step == 301) {
        const ivp_vec3r_t t = ivp_vec3r(IVP_REAL(-1.0), IVP_REAL(3.0), IVP_REAL(3.0));
        ivp_world_friction_set_friction_value_translation(&s->slider, &t);
    }
}

static int run_world_friction(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    c2_world_friction_t s;
    c2_add(&sc, c2_ground(&sc, 0.6), "ground");
    ivp_object_t *slider = c2_box(&sc, 0.3, 0.0, 0.5, 0.5, 0.5, 2.0, 0.33, 0.33, 0.33, -6.0, 1.95, 0.0, NULL);
    ivp_object_t *floater = c2_ball(&sc, 0.5, 0.1, 0.5, 1.0, 0.1, 0.0, -2.0, 0.0);
    ivp_quatr_t qt = c2_quat(0.0, 0.0, 1.0, 0.5);
    ivp_object_t *tilt = c2_box(&sc, 0.5, 0.1, 0.6, 0.3, 0.3, 1.5, 0.09, 0.225, 0.225, 5.0, -3.0, 0.0, &qt);
    int rc = 1;
    if (!sc.failed) {
        c2_set_speed(slider, 6.0f, 0.0f, 2.0f);
        c2_set_rot_speed(slider, 0.0f, 3.0f, 0.0f);
        c2_set_speed(tilt, 0.0f, 0.0f, -3.0f);

        ivp_world_friction_init(&s.slider, slider);
        c2_core_controller(sc.env, IVP_CP_CONSTRAINTS_MAX, ivp_controller_world_friction, &s.slider, slider);

        ivp_world_friction_init(&s.floater, floater);
        ivp_vec3r_set(&s.floater.desired_speed_ws, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_set(&s.floater.desired_rot_speed_cs, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(2.0));
        ivp_vec3r_set(&s.floater.friction_value_translation, IVP_REAL(2.0), IVP_REAL(12.0), IVP_REAL(2.0));
        ivp_vec3r_set(&s.floater.friction_value_rotation, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
        c2_core_controller(sc.env, IVP_CP_CONSTRAINTS_MAX, ivp_controller_world_friction, &s.floater, floater);

        ivp_world_friction_init(&s.tilt, tilt);
        ivp_vec3r_set(&s.tilt.friction_value_translation, IVP_REAL(3.0), IVP_REAL(0.5), IVP_REAL(6.0));
        ivp_vec3r_set(&s.tilt.friction_value_rotation, IVP_REAL(0.4), IVP_REAL(0.4), IVP_REAL(0.4));
        c2_core_controller(sc.env, IVP_CP_CONSTRAINTS_MAX, ivp_controller_world_friction, &s.tilt, tilt);

        c2_add(&sc, slider, "slider");
        c2_add(&sc, floater, "floater");
        c2_add(&sc, tilt, "tilt");
        rc = c2_run(&sc, out, SCENARIO_WORLD_FRICTION, steps, dt, c2_world_friction_hook, &s);
        if (inv && rc == 0) c2_check_finite(&sc, inv, "world_friction");
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── golem: IVP_Controller_Golem ─────────────────────────────────────── */

typedef struct c2_golem {
    ivp_golem_state_t a, b, c;
    int problems;
} c2_golem_t;

/* reference RunnerGolem::resolve_for_problem: count, no beaming */
static void c2_golem_problem(void *ctx, ivp_object_t *object, ivp_real_t pos_error, ivp_real_t rot_error)
{
    (void)object;
    (void)pos_error;
    (void)rot_error;
    (*(int *)ctx)++;
}

/* IVP_Template_Controller_Golem defaults + IVP_Controller_Golem constructor */
static void c2_add_golem(c2_golem_t *s, ivp_environment_t *env, ivp_golem_state_t *g, ivp_object_t *obj,
                         float max_force, float max_torque, float force_factor, float damp_factor,
                         float torque_factor)
{
    ivp_golem_state_init(g);
    ivp_vec3r_set(&g->max_translation_force, (ivp_real_t)max_force, (ivp_real_t)max_force, (ivp_real_t)max_force);
    ivp_vec3r_set(&g->max_torque, (ivp_real_t)max_torque, (ivp_real_t)max_torque, (ivp_real_t)max_torque);
    g->force_factor = (ivp_real_t)force_factor;
    g->damp_factor = (ivp_real_t)damp_factor;
    g->torque_factor = (ivp_real_t)torque_factor;
    g->problem_fn = c2_golem_problem;
    g->problem_ctx = &s->problems;
    c2_core_controller(env, IVP_CP_MOTION, ivp_controller_golem, g, obj);
    ivp_golem_state_setup(g, obj);
}

static void c2_golem_hook(int step, ivp_environment_t *env, void *ctx)
{
    c2_golem_t *s = (c2_golem_t *)ctx;
    const ivp_time_t now = ivp_environment_get_time(env);
    if (step == 121) {
        const ivp_quatr_t q = c2_quat(1.0, 0.0, 0.0, 2.0);
        ivp_golem_set_prime_orientation(&s->c, &q, now, NULL, IVP_REAL(0.0));
    }
    if (step == 201) {
        const ivp_vec3r_t p = ivp_vec3r(IVP_REAL(2.0), IVP_REAL(-1.0), IVP_REAL(15.0));
        const ivp_vec3r_t v = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_golem_set_prime_position(&s->b, &p, &v, now);
    }
    if (step == 261) {
        const ivp_quatr_t q0 = c2_quat(1.0, 0.0, 0.0, 0.6);
        const ivp_quatr_t q1 = c2_quat(0.0, 1.0, 0.0, 0.8);
        ivp_golem_set_prime_orientation(&s->c, &q0, now, &q1, IVP_REAL(1.0));
    }
    if (step == 331) {
        const ivp_vec3r_t p = ivp_vec3r(IVP_REAL(2.0), IVP_REAL(0.5), IVP_REAL(3.0));
        const ivp_vec3r_t v = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(-1.0));
        ivp_golem_set_prime_position(&s->b, &p, &v, now);
    }
}

static int run_golem(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    c2_golem_t s;
    memset(&s, 0, sizeof(s));
    c2_add(&sc, c2_ground(&sc, 0.8), "ground");
    ivp_object_t *a = c2_box(&sc, 0.5, 0.1, 0.4, 0.4, 0.4, 1.5, 0.16, 0.16, 0.16, -4.0, 0.0, 0.0, NULL);
    ivp_object_t *b = c2_ball(&sc, 0.5, 0.1, 0.5, 1.0, 0.1, 2.0, -1.0, 0.0);
    ivp_object_t *c = c2_box(&sc, 0.5, 0.1, 0.5, 0.3, 0.5, 2.0, 0.2, 0.33, 0.2, 5.0, 2.15, 0.0, NULL);
    int rc = 1;
    if (!sc.failed) {
        const ivp_time_t now = ivp_environment_get_time(sc.env);
        c2_add_golem(&s, sc.env, &s.a, a, 60.0f, 20.0f, 0.8f, 1.0f, 0.8f);
        {
            const ivp_vec3r_t p = ivp_vec3r(IVP_REAL(-4.0), IVP_REAL(0.0), IVP_REAL(0.0));
            const ivp_vec3r_t v = ivp_vec3r(IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.5));
            ivp_golem_set_prime_position(&s.a, &p, &v, now);
            const ivp_quatr_t q0 = ivp_quatr_identity();
            const ivp_quatr_t q1 = c2_quat(0.0, 1.0, 0.0, 1.0);
            ivp_golem_set_prime_orientation(&s.a, &q0, now, &q1, IVP_REAL(2.0));
        }
        c2_add_golem(&s, sc.env, &s.b, b, 100.0f, 50.0f, 0.8f, 1.0f, 0.8f);
        c2_add_golem(&s, sc.env, &s.c, c, 40.0f, 15.0f, 0.6f, 0.9f, 0.7f);
        c2_add(&sc, a, "golem");
        c2_add(&sc, b, "golem");
        c2_add(&sc, c, "golem");
        rc = c2_run(&sc, out, SCENARIO_GOLEM, steps, dt, c2_golem_hook, &s);
        if (inv && rc == 0) {
            c2_check_finite(&sc, inv, "golem");
            inv_check(inv, steps < 240 || s.problems > 0, "golem problems reported: %d", s.problems);
        }
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── fixed_keyed: IVP_Constraint_Fixed_Keyframed ────────────────────── */

typedef struct c2_fixed_keyed {
    ivp_fixed_keyed_state_t fk1, fk2;
} c2_fixed_keyed_t;

static void c2_add_fixed_keyed(ivp_environment_t *env, ivp_fixed_keyed_state_t *k,
                               ivp_object_t *ref, ivp_object_t *att, float max_force, float max_torque,
                               float force_factor, float damp_factor, float torque_factor, float angular_damp_factor)
{
    ivp_fixed_keyed_init(k, ref, att, NULL);
    ivp_vec3r_set(&k->max_translation_force, (ivp_real_t)max_force, (ivp_real_t)max_force, (ivp_real_t)max_force);
    ivp_vec3r_set(&k->max_torque, (ivp_real_t)max_torque, (ivp_real_t)max_torque, (ivp_real_t)max_torque);
    k->force_factor = (ivp_real_t)force_factor;
    k->damp_factor = (ivp_real_t)damp_factor;
    k->torque_factor = (ivp_real_t)torque_factor;
    k->angular_damp_factor = (ivp_real_t)angular_damp_factor;
    ivp_environment_announce_dependent_controller(env, 0, ivp_controller_fixed_keyed, k, ivp_fixed_keyed_discovery);
}

static void c2_fixed_keyed_hook(int step, ivp_environment_t *env, void *ctx)
{
    c2_fixed_keyed_t *s = (c2_fixed_keyed_t *)ctx;
    const ivp_time_t now = ivp_environment_get_time(env);
    if (step == 121) {
        const ivp_vec3r_t p = ivp_vec3r(IVP_REAL(2.0), IVP_REAL(-1.0), IVP_REAL(1.0));
        const ivp_vec3r_t v = ivp_vec3r(IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_fixed_keyed_set_prime_position_Ros(&s->fk1, &p, &v, now);
        const ivp_quatr_t q0 = ivp_quatr_identity();
        const ivp_quatr_t q1 = c2_quat(0.0, 0.0, 1.0, 1.2);
        ivp_fixed_keyed_set_prime_orientation_Ros(&s->fk1, &q0, now, &q1, IVP_REAL(1.5));
    }
    if (step == 201) {
        const ivp_vec3r_t p = ivp_vec3r(IVP_REAL(0.5), IVP_REAL(-1.5), IVP_REAL(0.0));
        const ivp_vec3r_t v = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_fixed_keyed_set_prime_position_Ros(&s->fk2, &p, &v, now);
    }
}

static int run_fixed_keyed(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    c2_fixed_keyed_t s;
    c2_add(&sc, c2_ground(&sc, 0.8), "ground");
    ivp_object_t *post = c2_static(&sc, 0.8, 0.0, 0.2, 0.2, 0.2, 0.0, -2.0, 0.0);
    ivp_object_t *fa = c2_box(&sc, 0.5, 0.1, 0.3, 0.3, 0.3, 1.0, 0.06, 0.06, 0.06, 2.0, -2.0, 0.0, NULL);
    ivp_object_t *base = c2_box(&sc, 0.5, 0.1, 0.8, 0.3, 0.8, 3.0, 0.73, 1.28, 0.73, 4.0, 2.15, 4.0, NULL);
    ivp_object_t *fb = c2_box(&sc, 0.5, 0.1, 0.25, 0.25, 0.25, 0.5, 0.02, 0.02, 0.02, 4.0, 1.0, 4.0, NULL);
    int rc = 1;
    if (!sc.failed) {
        /* IVP_Template_Controller_Motion defaults: force 0.8, damp 1, torque 0.8, angular damp 1 */
        c2_add_fixed_keyed(sc.env, &s.fk1, post, fa, 200.0f, 50.0f, 0.8f, 1.0f, 0.8f, 1.0f);
        c2_add_fixed_keyed(sc.env, &s.fk2, base, fb, 40.0f, 10.0f, 0.5f, 0.6f, 0.6f, 0.8f);
        c2_add(&sc, post, "post");
        c2_add(&sc, fa, "keyed");
        c2_add(&sc, base, "base");
        c2_add(&sc, fb, "keyed");
        rc = c2_run(&sc, out, SCENARIO_FIXED_KEYED, steps, dt, c2_fixed_keyed_hook, &s);
        if (inv && rc == 0) c2_check_finite(&sc, inv, "fixed_keyed");
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── hinge_limits: hinges with limited rotation axis ────────────────── */

static int run_hinge_limits(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    ivp_constraint_state_t cons[3];
    c2_add(&sc, c2_ground(&sc, 0.8), "ground");
    ivp_object_t *post = c2_static(&sc, 0.8, 0.0, 0.2, 0.2, 0.2, 0.0, -8.0, 0.0);
    ivp_object_t *arm1 = c2_box(&sc, 0.5, 0.1, 1.0, 0.1, 0.1, 1.0, 0.05, 0.337, 0.337, 1.2, -6.0, 0.0, NULL);
    ivp_object_t *arm2 = c2_box(&sc, 0.5, 0.1, 0.6, 0.1, 0.1, 0.5, 0.03, 0.062, 0.062, 2.9, -6.0, 0.0, NULL);
    ivp_object_t *post2 = c2_static(&sc, 0.8, 0.0, 0.2, 0.2, 0.2, 0.0, -8.0, 5.0);
    ivp_object_t *door = c2_box(&sc, 0.5, 0.1, 0.1, 0.8, 0.6, 2.0, 0.67, 0.25, 0.43, 0.0, -6.0, 5.8, NULL);
    int rc = 1;
    if (!sc.failed) {
        c2_set_rot_speed(door, 0.0f, 4.0f, 0.0f);
        const ivp_vec3r_t axis_z = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0));
        const ivp_vec3r_t axis_y = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
        ivp_constraint_template_t t;
        {
            const ivp_vec3r_t anchor = ivp_vec3r(IVP_REAL(0.2), IVP_REAL(-6.0), IVP_REAL(0.0));
            ivp_constraint_template_init(&t);
            ivp_constraint_template_set_hinge_ws(&t, post, &anchor, &axis_z, arm1);
            ivp_constraint_template_limit_axis(&t, 5, (ivp_real_t)-0.7f, (ivp_real_t)0.7f);
            c2_constraint(sc.env, &cons[0], &t);
        }
        {
            const ivp_vec3r_t anchor = ivp_vec3r(IVP_REAL(2.25), IVP_REAL(-6.0), IVP_REAL(0.0));
            ivp_constraint_template_init(&t);
            ivp_constraint_template_set_hinge_ws(&t, arm1, &anchor, &axis_z, arm2);
            ivp_constraint_template_limit_axis(&t, 5, (ivp_real_t)-0.2f, (ivp_real_t)0.4f);
            t.limited_axis_stiffness = (ivp_real_t)0.8f;
            t.force_factor = (ivp_real_t)0.7f;
            t.damp_factor = (ivp_real_t)0.9f;
            c2_constraint(sc.env, &cons[1], &t);
        }
        {
            const ivp_vec3r_t anchor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-6.0), IVP_REAL(5.2));
            ivp_constraint_template_init(&t);
            ivp_constraint_template_set_hinge_ws(&t, post2, &anchor, &axis_y, door);
            ivp_constraint_template_limit_axis(&t, 5, (ivp_real_t)-1.0f, (ivp_real_t)0.5f);
            c2_constraint(sc.env, &cons[2], &t);
        }
        c2_add(&sc, post, "post");
        c2_add(&sc, arm1, "arm");
        c2_add(&sc, arm2, "arm");
        c2_add(&sc, post2, "post");
        c2_add(&sc, door, "door");
        rc = c2_run(&sc, out, SCENARIO_HINGE_LIMITS, steps, dt, NULL, NULL);
        if (inv && rc == 0) c2_check_finite(&sc, inv, "hinge_limits");
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── cardan_tense: cardan joint, tense ball socket, fixed ───────────── */

static int run_cardan_tense(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    ivp_constraint_state_t cons[3];
    c2_add(&sc, c2_ground(&sc, 0.8), "ground");
    ivp_object_t *ceiling = c2_static(&sc, 0.8, 0.0, 3.0, 0.2, 0.3, 0.0, -9.3, 0.0);
    ivp_object_t *pend = c2_box(&sc, 0.5, 0.1, 0.2, 0.8, 0.2, 1.0, 0.227, 0.027, 0.227, 0.0, -8.0, 0.0, NULL);
    if (pend) {
        c2_set_rot_speed(pend, 1.5f, 3.0f, 0.5f);
        c2_set_speed(pend, 2.0f, 0.0f, 0.0f);
    }
    ivp_object_t *tense = c2_ball(&sc, 0.5, 0.1, 0.3, 0.7, 0.025, 2.5, -7.5, 0.0);
    ivp_object_t *weld_a = c2_box(&sc, 0.5, 0.1, 0.3, 0.3, 0.3, 1.0, 0.06, 0.06, 0.06, -3.0, -1.0, 0.0, NULL);
    ivp_quatr_t qw = c2_quat(0.0, 1.0, 0.0, 0.3);
    ivp_object_t *weld_b = c2_box(&sc, 0.5, 0.1, 0.3, 0.3, 0.3, 0.6, 0.036, 0.036, 0.036, -2.2, -1.0, 0.0, &qw);
    int rc = 1;
    if (!sc.failed) {
        c2_set_rot_speed(weld_a, 0.0f, 0.0f, 2.0f);
        ivp_constraint_template_t t;
        {
            const ivp_vec3r_t anchor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-8.8), IVP_REAL(0.0));
            const ivp_vec3r_t axis = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
            ivp_constraint_template_init(&t);
            ivp_constraint_template_set_cardanjoint_ws(&t, ceiling, &anchor, &axis, pend);
            c2_constraint(sc.env, &cons[0], &t);
        }
        {
            const ivp_vec3r_t anchor_Ros = ivp_vec3r(IVP_REAL(2.0), IVP_REAL(0.2), IVP_REAL(0.0));
            const ivp_vec3r_t dist_Ros = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
            ivp_constraint_template_init(&t);
            ivp_constraint_template_set_ballsocket_tense_Ros(&t, ceiling, &anchor_Ros, tense, &dist_Ros);
            t.force_factor = (ivp_real_t)0.6f;
            c2_constraint(sc.env, &cons[1], &t);
        }
        {
            ivp_constraint_template_init(&t);
            ivp_constraint_template_set_fixed(&t, weld_a, weld_b);
            c2_constraint(sc.env, &cons[2], &t);
        }
        c2_add(&sc, ceiling, "ceiling");
        c2_add(&sc, pend, "cardan");
        c2_add(&sc, tense, "tense");
        c2_add(&sc, weld_a, "weld");
        c2_add(&sc, weld_b, "weld");
        rc = c2_run(&sc, out, SCENARIO_CARDAN_TENSE, steps, dt, NULL, NULL);
        if (inv && rc == 0) c2_check_finite(&sc, inv, "cardan_tense");
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── slider_limits: limited translation axes ─────────────────────────── */

static int run_slider_limits(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    ivp_constraint_state_t cons[2];
    c2_add(&sc, c2_ground(&sc, 0.8), "ground");
    ivp_object_t *rail = c2_static(&sc, 0.8, 0.0, 0.2, 0.2, 0.2, 0.0, -6.0, 0.0);
    ivp_object_t *slider = c2_box(&sc, 0.5, 0.1, 0.3, 0.3, 0.3, 1.0, 0.06, 0.06, 0.06, -2.0, -4.0, 0.0, NULL);
    ivp_object_t *rail2 = c2_static(&sc, 0.8, 0.0, 0.2, 0.2, 0.2, 0.0, -6.0, 4.0);
    ivp_object_t *puck = c2_ball(&sc, 0.5, 0.1, 0.3, 0.8, 0.029, 2.0, -3.0, 4.0);
    int rc = 1;
    if (!sc.failed) {
        c2_set_speed(puck, 3.0f, 0.0f, 1.0f);
        c2_set_rot_speed(puck, 0.0f, 2.0f, 0.0f);
        ivp_constraint_template_t t;
        {
            const ivp_vec3r_t anchor = ivp_vec3r(IVP_REAL(-2.0), IVP_REAL(-4.0), IVP_REAL(0.0));
            const ivp_vec3r_t axis = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(0.0));
            ivp_constraint_template_init(&t);
            ivp_constraint_template_set_constraint_ws(&t, rail, &anchor, &axis, 2, 3, slider, NULL);
            ivp_constraint_template_limit_axis(&t, 2, (ivp_real_t)-1.0f, (ivp_real_t)1.5f);
            c2_constraint(sc.env, &cons[0], &t);
        }
        {
            const ivp_vec3r_t anchor = ivp_vec3r(IVP_REAL(2.0), IVP_REAL(-3.0), IVP_REAL(4.0));
            const ivp_vec3r_t axis = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
            ivp_constraint_template_init(&t);
            ivp_constraint_template_set_constraint_ws(&t, rail2, &anchor, &axis, 1, 0, puck, NULL);
            ivp_constraint_template_limit_axis(&t, 1, (ivp_real_t)-0.5f, (ivp_real_t)0.5f);
            ivp_constraint_template_limit_axis(&t, 2, (ivp_real_t)-1.0f, (ivp_real_t)1.0f);
            t.limited_axis_stiffness = (ivp_real_t)0.5f;
            c2_constraint(sc.env, &cons[1], &t);
        }
        c2_add(&sc, rail, "rail");
        c2_add(&sc, slider, "slider");
        c2_add(&sc, rail2, "rail");
        c2_add(&sc, puck, "puck");
        rc = c2_run(&sc, out, SCENARIO_SLIDER_LIMITS, steps, dt, NULL, NULL);
        if (inv && rc == 0) c2_check_finite(&sc, inv, "slider_limits");
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── constraint_break: IVP_CFE_BREAK / IVP_CFE_CLIP ──────────────────── */

static void c2_max_impulse(ivp_constraint_template_t *t, int first, ivp_constraint_force_exceed_t type, float impulse)
{
    /* set_max_translation_impulse (first 0) / set_max_rotation_impulse (first 3) */
    for (int i = first; i < first + 3; i++) {
        t->maximpulse[i] = (ivp_real_t)impulse;
        t->maximpulse_type[i] = type;
    }
}

static int run_constraint_break(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    ivp_constraint_state_t cons[3];
    c2_add(&sc, c2_ground(&sc, 0.8), "ground");
    ivp_object_t *anchor1 = c2_static(&sc, 0.8, 0.0, 0.2, 0.2, 0.2, -3.0, -10.5, 0.0);
    ivp_quatr_t qh = c2_quat(0.0, 0.0, 1.0, 1.5707963267948966);
    ivp_object_t *a1 = c2_box(&sc, 0.5, 0.1, 0.15, 0.5, 0.15, 1.0, 0.091, 0.015, 0.091, -2.5, -9.8, 0.0, &qh);
    ivp_object_t *anchor2 = c2_static(&sc, 0.8, 0.0, 0.2, 0.2, 0.2, 3.0, -10.5, 0.0);
    ivp_object_t *b1 = c2_box(&sc, 0.5, 0.1, 0.15, 0.5, 0.15, 2.0, 0.18, 0.03, 0.18, 3.0, -9.3, 0.0, NULL);
    if (b1) c2_set_speed(b1, 0.0f, 0.0f, 4.0f);
    ivp_object_t *anchor3 = c2_static(&sc, 0.8, 0.0, 0.2, 0.2, 0.2, 0.0, -11.2, 4.0);
    ivp_object_t *door = c2_box(&sc, 0.5, 0.1, 0.6, 0.6, 0.05, 1.0, 0.12, 0.12, 0.24, 0.8, -10.0, 4.0, NULL);
    ivp_object_t *hammer = c2_ball(&sc, 0.5, 0.1, 0.3, 3.0, 0.108, 1.1, -13.0, 4.0);
    int rc = 1;
    if (!sc.failed) {
        ivp_constraint_template_t t;
        {
            const ivp_vec3r_t anchor = ivp_vec3r(IVP_REAL(-3.0), IVP_REAL(-9.8), IVP_REAL(0.0));
            ivp_constraint_template_init(&t);
            ivp_constraint_template_set_ballsocket_ws(&t, anchor1, &anchor, a1);
            c2_max_impulse(&t, 0, IVP_CFE_BREAK, 0.1f);
            c2_constraint(sc.env, &cons[0], &t);
        }
        {
            const ivp_vec3r_t anchor = ivp_vec3r(IVP_REAL(3.0), IVP_REAL(-9.8), IVP_REAL(0.0));
            ivp_constraint_template_init(&t);
            ivp_constraint_template_set_ballsocket_ws(&t, anchor2, &anchor, b1);
            c2_max_impulse(&t, 0, IVP_CFE_CLIP, 0.2f);
            c2_constraint(sc.env, &cons[1], &t);
        }
        {
            const ivp_vec3r_t anchor = ivp_vec3r(IVP_REAL(0.2), IVP_REAL(-10.0), IVP_REAL(4.0));
            const ivp_vec3r_t axis = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
            ivp_constraint_template_init(&t);
            ivp_constraint_template_set_hinge_ws(&t, anchor3, &anchor, &axis, door);
            c2_max_impulse(&t, 0, IVP_CFE_BREAK, 0.4f);
            c2_max_impulse(&t, 3, IVP_CFE_BREAK, 0.4f);
            c2_constraint(sc.env, &cons[2], &t);
        }
        c2_add(&sc, anchor1, "anchor");
        c2_add(&sc, a1, "breaker");
        c2_add(&sc, anchor2, "anchor");
        c2_add(&sc, b1, "clipper");
        c2_add(&sc, anchor3, "anchor");
        c2_add(&sc, door, "door");
        c2_add(&sc, hammer, "hammer");
        rc = c2_run(&sc, out, SCENARIO_CONSTRAINT_BREAK, steps, dt, NULL, NULL);
        if (inv && rc == 0) {
            c2_check_finite(&sc, inv, "constraint_break");
            inv_check(inv, steps < 240 || (cons[0].is_broken && cons[2].is_broken),
                      "constraint_break: breaker and door constraints broke (%d, %d)",
                      (int)cons[0].is_broken, (int)cons[2].is_broken);
        }
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── attacher: IVP_Attacher_To_Cores<floating attachment> ───────────── */

typedef struct c2_hover {
    ivp_floating_state_t floating;
    ivp_controller_handle_t handle;
    ivp_object_t *object;
} c2_hover_t;

/* HoverAttachment(attacher, core): IVP_Controller_Floating on the core's object */
static void *c2_hover_create(ivp_attacher_to_cores_t *a, ivp_core_t *core)
{
    c2_hover_t *h = (c2_hover_t *)calloc(1, sizeof(*h));
    if (!h) return NULL;
    h->object = ivp_attacher_core_object(core);
    h->handle = c2_add_floating(a->environment, &h->floating, h->object, 25.0f, 10.0f, 0.0, 0.4, 0.0, NULL, 1.2f);
    return h;
}

/* ~HoverAttachment / ~IVP_Controller_Floating: remove_controller_from_core */
static void c2_hover_delete(ivp_attacher_to_cores_t *a, ivp_core_t *core, void *attachment)
{
    (void)core;
    c2_hover_t *h = (c2_hover_t *)attachment;
    if (a->environment) {
        ivp_environment_remove_controller_from_core(a->environment, h->handle, h->object);
        ivp_environment_remove_controller(a->environment, h->handle);
    }
    free(h);
}

static const ivp_attacher_to_cores_class_t c2_hover_class = { c2_hover_create, c2_hover_delete, NULL };

typedef struct c2_attacher {
    ivp_set_active_t set;
    ivp_attacher_to_cores_t attacher;
    ivp_object_t *box[4];
} c2_attacher_t;

static void c2_attacher_hook(int step, ivp_environment_t *env, void *ctx)
{
    (void)env;
    c2_attacher_t *s = (c2_attacher_t *)ctx;
    if (step == 121) ivp_set_active_install(&s->set, s->box[2]->physical_core);
    if (step == 241) ivp_set_active_remove(&s->set, s->box[0]->physical_core);
    if (step == 361) ivp_set_active_install(&s->set, s->box[3]->physical_core);
}

static int run_attacher(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    c2_attacher_t s;
    memset(&s, 0, sizeof(s));
    c2_add(&sc, c2_ground(&sc, 0.8), "ground");
    for (int i = 0; i < 4; i++) {
        s.box[i] = c2_box(&sc, 0.5, 0.1, 0.4, 0.4, 0.4, 1.0, 0.107, 0.107, 0.107,
                          -6.0 + 4.0 * (double)i, 0.0, 0.0, NULL);
        c2_add(&sc, s.box[i], "box");
    }
    int rc = 1;
    if (!sc.failed) {
        ivp_set_active_init(&s.set, NULL, 16);
        ivp_set_active_install(&s.set, s.box[0]->physical_core);
        ivp_set_active_install(&s.set, s.box[1]->physical_core);
        if (ivp_attacher_to_cores_init(&s.attacher, sc.env, &s.set, &c2_hover_class, NULL)) {
            rc = c2_run(&sc, out, SCENARIO_ATTACHER, steps, dt, c2_attacher_hook, &s);
            if (inv && rc == 0) {
                c2_check_finite(&sc, inv, "attacher");
                /* {0, 1} + 2 at step 121 - 0 at step 241 + 3 at step 361 */
                inv_check(inv, steps < 361 || ivp_attacher_to_cores_count(&s.attacher) == 3,
                          "attacher: %d attachments (expected 3)", ivp_attacher_to_cores_count(&s.attacher));
            }
        }
        ivp_set_active_destroy(&s.set); /* deletes the attachments and the attacher */
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── actuator_extra: IVP_Actuator_Extra (float cam + puck force) ────── */

static int run_actuator_extra(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    ivp_actuator_extra_state_t cam, puck;
    ivp_object_t *ground = c2_ground(&sc, 0.8);
    c2_add(&sc, ground, "ground");
    ivp_object_t *cam_box = c2_box(&sc, 0.5, 0.1, 0.4, 0.4, 0.4, 1.0, 0.107, 0.107, 0.107, -3.0, 1.9, 0.0, NULL);
    ivp_object_t *rest_box = c2_box(&sc, 0.5, 0.1, 0.4, 0.4, 0.4, 1.0, 0.107, 0.107, 0.107, 3.0, 1.9, 0.0, NULL);
    ivp_object_t *p0 = c2_ball(&sc, 0.5, 0.1, 0.3, 0.5, 0.018, -1.0, 2.2, 3.0);
    ivp_object_t *p1 = c2_ball(&sc, 0.5, 0.1, 0.3, 0.5, 0.018, 1.0, 2.2, 3.0);
    int rc = 1;
    if (!sc.failed) {
        c2_set_speed(p0, 1.0f, 0.0f, 0.0f);
        c2_set_speed(p1, -0.5f, 0.0f, 0.5f);
        ivp_template_extra_t te;
        /* anchors (core space of objects whose core and object frames coincide) */
        ivp_template_extra_init(&te);
        te.objects[0] = cam_box;
        te.objects[1] = ground;
        ivp_vec3r_set(&te.anchors_cs[1], IVP_REAL(0.0), IVP_REAL(-0.5), IVP_REAL(-4.0));
        te.info.is_float_cam = true;
        ivp_actuator_extra_init(&cam, &te);
        ivp_environment_add_controller(sc.env, IVP_CP_ACTUATOR, ivp_controller_actuator_extra, &cam);
        ivp_template_extra_init(&te);
        te.objects[0] = p0;
        te.objects[1] = p1;
        te.info.is_puck_force = 1;
        ivp_actuator_extra_init(&puck, &te);
        ivp_environment_add_controller(sc.env, IVP_CP_ACTUATOR, ivp_controller_actuator_extra, &puck);
        c2_add(&sc, cam_box, "cam_box");
        c2_add(&sc, rest_box, "rest_box");
        c2_add(&sc, p0, "puck");
        c2_add(&sc, p1, "puck");
        rc = c2_run(&sc, out, SCENARIO_ACTUATOR_EXTRA, steps, dt, NULL, NULL);
        if (inv && rc == 0) c2_check_finite(&sc, inv, "actuator_extra");
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── airboat: IVP_Controller_Raycast_Airboat ────────────────────────── */

static const double C2_BEACH_EDGE_X = -2.0; /* ground for x < edge, water beyond */
static const double C2_WATER_Y = 2.5;       /* water surface, level with the beach */

/* reference RunnerAirboat::do_raycasts_gameside */
static void c2_airboat_rays(void *ctx, int n_rays, const ivp_ray_solver_template_t *rays,
                            ivp_airboat_impact_t *impacts)
{
    (void)ctx;
    for (int i = 0; i < n_rays; i++) {
        const ivp_vec3r_t *start = &rays[i].ray_start_point;
        const ivp_vec3r_t *dir = &rays[i].ray_normized_direction; /* float-exact */
        const double len = (double)rays[i].ray_length;
        ivp_airboat_impact_t *imp = &impacts[i];
        memset(imp, 0, sizeof(*imp));
        if ((double)start->k[0] < C2_BEACH_EDGE_X) {
            if ((float)dir->k[1] > 1e-6f) {
                const double t = (C2_GROUND_TOP - (double)start->k[1]) / (double)dir->k[1];
                if (t >= 0.0 && t <= len) {
                    imp->impact = true;
                    ivp_vec3r_set(&imp->impact_point_ws,
                                  (ivp_real_t)((double)start->k[0] + (double)dir->k[0] * t),
                                  (ivp_real_t)((double)start->k[1] + (double)dir->k[1] * t),
                                  (ivp_real_t)((double)start->k[2] + (double)dir->k[2] * t));
                    imp->impact_normal_ws = ivp_vec3f(0.0f, -1.0f, 0.0f);
                    imp->friction = 0.8f;
                }
            }
        } else {
            imp->in_water = (double)start->k[1] > C2_WATER_Y;
            if ((float)dir->k[1] > 1e-6f) {
                const double t = (C2_WATER_Y - (double)start->k[1]) / (double)dir->k[1];
                if (t >= 0.0 && t <= len) imp->impact_water = true;
            }
        }
    }
}

/* reference c2_airboat(): IVP_Template_Car_System(4, 2) */
static bool c2_add_airboat(ivp_airboat_state_t *a, ivp_object_t *body)
{
    ivp_car_system_template_t tcs;
    ivp_car_system_template_init(&tcs, 4, 2);
    tcs.car_body = body;
    for (int i = 0; i < 4; i++) {
        const double wx = (i & 1) ? 0.7 : -0.7; /* odd = right */
        const double wz = (i & 2) ? -1.3 : 1.3; /* 2,3 = rear */
        tcs.wheel_pos_Bos[i] = ivp_vec3f((float)wx, 0.25f, (float)wz);
        tcs.trace_pos_Bos[i] = ivp_vec3f((float)wx, 0.25f, (float)wz);
        tcs.wheel_radius[i] = (ivp_real_t)0.3f;
        tcs.spring_constant[i] = IVP_REAL(10000.0);
        tcs.spring_damping[i] = IVP_REAL(800.0);
        tcs.spring_damping_compression[i] = IVP_REAL(800.0);
        tcs.spring_pre_tension[i] = IVP_REAL(0.0);
    }
    tcs.stabilizer_constant[0] = IVP_REAL(1000.0);
    tcs.stabilizer_constant[1] = IVP_REAL(1000.0);
    tcs.wheel_max_rotation_speed[0] = IVP_REAL(50.0);
    tcs.wheel_max_rotation_speed[1] = IVP_REAL(50.0);
    tcs.extra_gravity_force = IVP_REAL(0.0);
    tcs.body_down_force_vertical_offset = (ivp_real_t)0.2f;
    if (!ivp_airboat_init(a, &tcs)) return false;
    ivp_airboat_set_raycast(a, c2_airboat_rays, NULL);
    return true;
}

typedef struct c2_airboat {
    ivp_airboat_state_t land, water;
} c2_airboat_t;

static void c2_airboat_hook(int step, ivp_environment_t *env, void *ctx)
{
    (void)env;
    c2_airboat_t *s = (c2_airboat_t *)ctx;
    if (step == 91) {
        ivp_airboat_update_throttle(&s->land, 0.6f);  /* does not wake the body */
        ivp_airboat_do_steering(&s->land, 0.005f);    /* wakes it (below the 0.01 steering threshold) */
        ivp_airboat_update_throttle(&s->water, 0.15f);
    }
    if (step == 301) {
        ivp_airboat_do_steering(&s->land, 0.3f);
        ivp_airboat_do_steering(&s->water, -0.3f);
    }
    if (step == 341) {
        ivp_airboat_do_steering(&s->land, 0.0f);
        ivp_airboat_do_steering(&s->water, 0.0f);
    }
    if (step == 501) {
        ivp_airboat_do_steering(&s->land, -0.2f);
        ivp_airboat_update_throttle(&s->land, -0.3f);
        ivp_airboat_update_throttle(&s->water, 0.0f);
    }
    if (step == 531) ivp_airboat_do_steering(&s->land, 0.0f);
}

static int run_airboat(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    c2_airboat_t s;
    memset(&s, 0, sizeof(s));
    c2_add(&sc, c2_static(&sc, 0.8, 0.0, 14.0, 0.5, 20.0, -16.0, 3.0, 0.0), "beach");
    /* land boat on the beach (forward +z), water boat afloat (forward +x) */
    ivp_object_t *land = c2_box(&sc, 0.5, 0.1, 0.8, 0.25, 1.5, 300.0, 231.0, 289.0, 70.0, -16.0, 1.95, -6.0, NULL);
    ivp_quatr_t q = c2_quat(0.0, 1.0, 0.0, 1.5707963267948966);
    ivp_object_t *water = c2_box(&sc, 0.5, 0.1, 0.8, 0.25, 1.5, 300.0, 231.0, 289.0, 70.0, 6.0, 2.25, 0.0, &q);
    int rc = 1;
    if (!sc.failed && c2_add_airboat(&s.land, land) && c2_add_airboat(&s.water, water)) {
        c2_add(&sc, land, "airboat");
        c2_add(&sc, water, "airboat");
        rc = c2_run(&sc, out, SCENARIO_AIRBOAT, steps, dt, c2_airboat_hook, &s);
        if (inv && rc == 0) {
            c2_check_finite(&sc, inv, "airboat");
            inv_check(inv, steps < 360 || (double)land->physical_core->pos_world_f_core_last_psi.k[2] > -4.0,
                      "airboat: the land boat drove forward: z=%.6f", (double)land->physical_core->pos_world_f_core_last_psi.k[2]);
        }
    }
    ivp_airboat_destroy(&s.land);
    ivp_airboat_destroy(&s.water);
    c2_scene_destroy(&sc);
    return rc;
}

/* ── fake_jetski: IVP_Controller_Raycast_Fake_Jetski ────────────────── */

/* reference RunnerJetski::do_raycasts: the ground plane y = C2_GROUND_TOP */
static void c2_jetski_rays(void *ctx, const ivp_event_sim_t *es, int n_wheels,
                           const ivp_ray_solver_template_t *rays, ivp_ray_hit_t *hits_out, float *friction_out)
{
    (void)es;
    ivp_object_t *ground = (ivp_object_t *)ctx;
    for (int i = 0; i < n_wheels; i++) {
        const ivp_vec3r_t *start = &rays[i].ray_start_point;
        const ivp_vec3r_t *dir = &rays[i].ray_normized_direction; /* float-exact */
        ivp_ray_hit_t *hit = &hits_out[i];
        memset(hit, 0, sizeof(*hit));
        friction_out[i] = 1.0f;
        if ((float)dir->k[1] > 1e-6f) {
            const double t = (C2_GROUND_TOP - (double)start->k[1]) / (double)dir->k[1];
            if (t >= 0.0 && t <= (double)rays[i].ray_length) {
                hit->hit_object = ground;
                ivp_vec3r_set(&hit->hit_normal, IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
                hit->hit_distance = (ivp_real_t)(float)t; /* IVP_FLOAT hit_distance */
                friction_out[i] = 0.8f;
            }
        }
    }
}

static void c2_jetski_hook(int step, ivp_environment_t *env, void *ctx)
{
    (void)env;
    ivp_jetski_state_t *k = (ivp_jetski_state_t *)ctx;
    if (step == 61) {
        for (int i = 0; i < 4; i++) ivp_jetski_fix_wheel(k, (ivp_jetski_wheel_pos_t)i, false);
        ivp_jetski_change_wheel_torque(k, IVP_JETSKI_REAR_LEFT, 600.0f);
        ivp_jetski_change_wheel_torque(k, IVP_JETSKI_REAR_RIGHT, 600.0f);
    }
    if (step == 241) ivp_jetski_do_steering(k, 0.3f);
    if (step == 361) ivp_jetski_activate_booster(k, 8.0f, 0.5f, 1.0f);
    if (step == 481) {
        ivp_jetski_change_wheel_torque(k, IVP_JETSKI_REAR_LEFT, 0.0f);
        ivp_jetski_change_wheel_torque(k, IVP_JETSKI_REAR_RIGHT, 0.0f);
        ivp_jetski_do_steering(k, -0.2f);
    }
    if (step == 601) {
        ivp_jetski_fix_wheel(k, IVP_JETSKI_REAR_LEFT, true);
        ivp_jetski_fix_wheel(k, IVP_JETSKI_REAR_RIGHT, true);
    }
}

static int run_fake_jetski(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    ivp_jetski_state_t ski;
    memset(&ski, 0, sizeof(ski));
    ivp_object_t *ground = c2_ground(&sc, 0.8);
    c2_add(&sc, ground, "ground");
    ivp_object_t *body = c2_box(&sc, 0.5, 0.1, 0.5, 0.3, 1.2, 250.0, 132.0, 141.0, 28.0, 0.0, 1.6, -6.0, NULL);
    int rc = 1;
    if (!sc.failed) {
        ivp_car_system_template_t tcs;
        ivp_car_system_template_init(&tcs, 4, 2);
        tcs.car_body = body;
        for (int i = 0; i < 4; i++) {
            const double wx = (i & 1) ? 0.45 : -0.45;
            const double wz = (i & 2) ? -0.9 : 0.9;
            tcs.wheel_pos_Bos[i] = ivp_vec3f((float)wx, 0.3f, (float)wz);
            tcs.trace_pos_Bos[i] = ivp_vec3f((float)wx, 0.3f, (float)wz);
            tcs.wheel_radius[i] = (ivp_real_t)0.3f;
            tcs.spring_constant[i] = IVP_REAL(15000.0);
            tcs.spring_damping[i] = IVP_REAL(1500.0);
            tcs.spring_damping_compression[i] = IVP_REAL(1500.0);
            tcs.spring_pre_tension[i] = (ivp_real_t)-0.35f;
        }
        tcs.stabilizer_constant[0] = IVP_REAL(2000.0);
        tcs.stabilizer_constant[1] = IVP_REAL(2000.0);
        tcs.wheel_max_rotation_speed[0] = IVP_REAL(50.0);
        tcs.wheel_max_rotation_speed[1] = IVP_REAL(50.0);
        tcs.extra_gravity_force = IVP_REAL(500.0);
        tcs.body_down_force_vertical_offset = (ivp_real_t)0.2f;
        if (ivp_jetski_init(&ski, &tcs)) {
            ivp_jetski_set_raycast(&ski, c2_jetski_rays, ground);
            c2_add(&sc, body, "jetski");
            rc = c2_run(&sc, out, SCENARIO_FAKE_JETSKI, steps, dt, c2_jetski_hook, &ski);
            if (inv && rc == 0) {
                c2_check_finite(&sc, inv, "fake_jetski");
                inv_check(inv, steps < 360 || (double)body->physical_core->pos_world_f_core_last_psi.k[2] > -2.0,
                          "fake_jetski drove forward: z=%.6f", (double)body->physical_core->pos_world_f_core_last_psi.k[2]);
            }
        }
        ivp_jetski_destroy(&ski);
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── Controller coverage scenarios (controllers3) ────────────────────────
 *
 * Mirror tools/ivp_reference_runner/ref_runner_scenarios_controllers3.cxx
 * (keep in sync): raycast_car_drive, check_dist_events and golem_beam.  The
 * per-step inputs are scheduled in ticks of 1/60 s (tick k = step
 * k * (steps per second / 60) + 1), so every dt runs the same program. */

/* steps per tick of 1/60 s (reference get_delta_PSI_time is an IVP_FLOAT) */
static int c3_steps_per_tick(const ivp_environment_t *env)
{
    const int sps = (int)(1.0 / (double)(float)ivp_environment_get_delta_psi_time(env) + 0.5);
    const int spt = sps / 60;
    return spt < 1 ? 1 : spt;
}

/* reference create_static_box with an orientation */
static ivp_object_t *c3_static(c2_scene_t *sc, double friction, double elasticity,
                               double hx, double hy, double hz, double x, double y, double z,
                               const ivp_quatr_t *q)
{
    const ivp_surman_t *sm = c2_box_surman(sc, hx, hy, hz);
    if (!sm) return NULL;
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false;
    tmpl.material.friction = (ivp_real_t)friction;
    tmpl.material.elasticity = (ivp_real_t)elasticity;
    tmpl.physical_unmoveable = true;
    tmpl.orientation = q ? *q : ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, (ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
    return ivp_environment_create_object(sc->env, &tmpl, sm);
}

/* ── raycast_car_drive: IVP_Controller_Raycast_Car ──────────────────── */

/* reference C3RaySurface / c3_cast_rays: the top faces (object space
 * y = -hy, normal (0,-1,0)) of static boxes, closest hit within the ray
 * length (first box on ties), ray taken to object space with the core
 * matrix */
typedef struct c3_ray_surface {
    ivp_object_t *obj;
    double hx, hy, hz;
    float friction;
} c3_ray_surface_t;

typedef struct c3_ray_world {
    c3_ray_surface_t s[8];
    int n;
} c3_ray_world_t;

static void c3_cast_rays(void *ctx, const ivp_event_sim_t *es, int n_rays,
                         const ivp_ray_solver_template_t *rays, ivp_ray_hit_t *hits_out, float *friction_out)
{
    (void)es;
    const c3_ray_world_t *w = (const c3_ray_world_t *)ctx;
    for (int i = 0; i < n_rays; i++) {
        const ivp_vec3r_t *st = &rays[i].ray_start_point;
        const ivp_vec3r_t *dir = &rays[i].ray_normized_direction; /* float-exact */
        const double len = (double)rays[i].ray_length;           /* IVP_FLOAT */
        ivp_ray_hit_t *hit = &hits_out[i];
        memset(hit, 0, sizeof(*hit));
        friction_out[i] = 1.0f;
        int best = -1;
        double best_t = 0.0;
        for (int j = 0; j < w->n; j++) {
            const c3_ray_surface_t *s = &w->s[j];
            const ivp_transform_r_t *m = &s->obj->physical_core->m_world_f_core_last_psi;
#define C3E(r, c) ((double)m->rot.rows[r].k[c])
            const double px = (double)st->k[0] - (double)m->pos.k[0];
            const double py = (double)st->k[1] - (double)m->pos.k[1];
            const double pz = (double)st->k[2] - (double)m->pos.k[2];
            const double dx = (double)dir->k[0], dy = (double)dir->k[1], dz = (double)dir->k[2];
            const double ly = C3E(0, 1) * px + C3E(1, 1) * py + C3E(2, 1) * pz;
            const double vy = C3E(0, 1) * dx + C3E(1, 1) * dy + C3E(2, 1) * dz;
            if (!(vy > 1e-6)) continue;
            const double t = (-s->hy - ly) / vy;
            if (t < 0.0 || t > len) continue;
            if (best >= 0 && !(t < best_t)) continue;
            const double lx = C3E(0, 0) * px + C3E(1, 0) * py + C3E(2, 0) * pz;
            const double vx = C3E(0, 0) * dx + C3E(1, 0) * dy + C3E(2, 0) * dz;
            const double hx = lx + vx * t;
            if (hx < -s->hx || hx > s->hx) continue;
            const double lz = C3E(0, 2) * px + C3E(1, 2) * py + C3E(2, 2) * pz;
            const double vz = C3E(0, 2) * dx + C3E(1, 2) * dy + C3E(2, 2) * dz;
            const double hz = lz + vz * t;
            if (hz < -s->hz || hz > s->hz) continue;
#undef C3E
            best = j;
            best_t = t;
        }
        if (best >= 0) {
            hit->hit_object = w->s[best].obj;
            ivp_vec3r_set(&hit->hit_normal, IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
            hit->hit_distance = (ivp_real_t)(float)best_t; /* IVP_FLOAT hit_distance */
            friction_out[i] = w->s[best].friction;
        }
    }
}

typedef struct c3_raycast_car {
    c3_ray_world_t world;
    ivp_car_state_t car, truck, trike;
    int spt;
} c3_raycast_car_t;

static void c3_raycast_car_hook(int step, ivp_environment_t *env, void *ctx)
{
    (void)env;
    c3_raycast_car_t *s = (c3_raycast_car_t *)ctx;
    ivp_car_state_t *a = &s->car, *b = &s->truck, *c = &s->trike;
    if ((step - 1) % s->spt) return;
    const int tick = (step - 1) / s->spt;
    switch (tick) {
    case 15:
        ivp_car_change_wheel_torque(a, IVP_WHEEL_REAR_LEFT, 700.0f);
        ivp_car_change_wheel_torque(a, IVP_WHEEL_REAR_RIGHT, 700.0f);
        for (int i = 2; i < 6; i++) ivp_car_change_wheel_torque(b, i, 900.0f);
        ivp_car_change_wheel_torque(c, 2, 120.0f);
        break;
    case 40:
        ivp_car_do_steering(a, 0.12f);
        break;
    case 70:
        ivp_car_do_steering(a, -0.12f);
        ivp_car_do_steering(c, 0.12f);
        break;
    case 95:
        ivp_car_do_steering(a, 0.0f);
        ivp_car_change_spring_constant(b, 0, 26000.0f);
        break;
    case 110:
        ivp_car_activate_booster(a, 6.0f, 0.4f, 0.8f);
        break;
    case 120:
        ivp_car_do_steering(b, 0.1f);
        break;
    case 130:
        ivp_car_do_steering(c, -0.08f);
        break;
    case 140:
        ivp_car_activate_booster(a, 9.0f, 0.4f, 0.8f); /* refused: not ready */
        break;
    case 150:
        ivp_car_do_steering(a, 0.15f);
        ivp_car_change_stabilizer_constant(b, 1, 0.0f);
        ivp_car_change_wheel_torque(c, 2, 0.0f);
        ivp_car_fix_wheel(c, 2, true);
        break;
    case 160:
        ivp_car_do_steering(b, -0.1f);
        break;
    case 170:
        ivp_car_change_wheel_torque(a, IVP_WHEEL_REAR_LEFT, -500.0f);
        ivp_car_change_wheel_torque(a, IVP_WHEEL_REAR_RIGHT, -500.0f);
        ivp_car_do_steering(a, -0.12f);
        break;
    case 190:
        ivp_car_change_wheel_torque(a, IVP_WHEEL_REAR_LEFT, 0.0f);
        ivp_car_change_wheel_torque(a, IVP_WHEEL_REAR_RIGHT, 0.0f);
        ivp_car_fix_wheel(a, IVP_WHEEL_REAR_LEFT, true);
        ivp_car_fix_wheel(a, IVP_WHEEL_REAR_RIGHT, true);
        ivp_car_do_steering(a, 0.2f);
        ivp_car_fix_wheel(b, 4, true);
        ivp_car_fix_wheel(b, 5, true);
        ivp_car_do_steering(b, 0.0f);
        break;
    case 215:
        ivp_car_fix_wheel(a, IVP_WHEEL_FRONT_LEFT, true);
        ivp_car_fix_wheel(a, IVP_WHEEL_FRONT_RIGHT, true);
        break;
    case 230:
        for (int i = 0; i < 4; i++) ivp_car_fix_wheel(a, i, false);
        ivp_car_do_steering(a, 0.0f);
        ivp_car_fix_wheel(b, 4, false);
        ivp_car_fix_wheel(b, 5, false);
        for (int i = 2; i < 6; i++) ivp_car_change_wheel_torque(b, i, 0.0f);
        break;
    case 260:
        ivp_car_activate_booster(a, 5.0f, 0.5f, 0.5f);
        ivp_car_fix_wheel(c, 2, false);
        break;
    case 290:
        ivp_car_change_wheel_torque(c, 2, 180.0f);
        break;
    case 300:
        ivp_car_set_booster_acceleration(a, 2.0f);
        ivp_car_change_wheel_torque(b, 2, -600.0f);
        ivp_car_change_wheel_torque(b, 3, -600.0f);
        ivp_car_do_steering(b, 0.15f);
        break;
    case 330:
        ivp_car_set_booster_acceleration(a, 0.0f);
        ivp_car_change_wheel_torque(a, IVP_WHEEL_REAR_LEFT, 300.0f);
        ivp_car_change_wheel_torque(a, IVP_WHEEL_REAR_RIGHT, 300.0f);
        ivp_car_do_steering(a, -0.1f);
        break;
    case 400:
        ivp_car_do_steering(a, 0.08f);
        ivp_car_do_steering(c, 0.0f);
        break;
    default:
        break;
    }
}

static int run_raycast_car_drive(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    c3_raycast_car_t *s = (c3_raycast_car_t *)calloc(1, sizeof(*s));
    if (!s) {
        c2_scene_destroy(&sc);
        return 1;
    }
    /* ground (top y = 2.5), ramp in the car's lane, bumps in the truck's and
     * the trike's lanes (gravity +y: "up" is -y) */
    ivp_object_t *ground = c3_static(&sc, 0.8, 0.0, 40.0, 0.5, 80.0, 0.0, 3.0, 30.0, NULL);
    const ivp_quatr_t qr = c2_quat(1.0, 0.0, 0.0, 0.11);
    ivp_object_t *ramp = c3_static(&sc, 0.8, 0.0, 1.8, 0.5, 2.0, -6.0, 2.79, -5.0, &qr);
    ivp_object_t *bump0 = c3_static(&sc, 0.8, 0.0, 0.4, 0.1, 0.3, 6.9, 2.4, -12.5, NULL);
    ivp_object_t *bump1 = c3_static(&sc, 0.8, 0.0, 0.4, 0.12, 0.5, 5.1, 2.38, -9.5, NULL);
    ivp_object_t *bump2 = c3_static(&sc, 0.8, 0.0, 0.3, 0.08, 0.6, 0.0, 2.42, -11.0, NULL);
    /* car: 4 wheels / 2 axes, rear drive */
    ivp_object_t *car_body = c2_box(&sc, 0.5, 0.1, 0.9, 0.3, 1.9, 400.0, 493.0, 589.0, 120.0, -6.0, 1.3, -16.0, NULL);
    if (car_body) c2_set_speed(car_body, 0.0f, 0.0f, 5.0f);
    /* truck: 6 wheels / 3 axes */
    ivp_object_t *truck_body = c2_box(&sc, 0.5, 0.1, 1.0, 0.35, 2.6, 700.0, 1606.0, 1810.0, 262.0, 6.0, 1.2, -18.0, NULL);
    if (truck_body) c2_set_speed(truck_body, 0.0f, 0.0f, 4.0f);
    /* trike: 3 wheels / 2 axes (front pair, single rear wheel) */
    ivp_object_t *trike_body = c2_box(&sc, 0.5, 0.1, 0.6, 0.25, 1.2, 150.0, 75.0, 90.0, 40.0, 0.0, 1.65, -15.0, NULL);
    if (trike_body) c2_set_speed(trike_body, 0.0f, 0.0f, 3.0f);
    int rc = 1;
    bool ok = !sc.failed && ground && ramp && bump0 && bump1 && bump2 && car_body && truck_body && trike_body;
    if (ok) {
        s->spt = c3_steps_per_tick(sc.env);
        const c3_ray_surface_t surf[5] = {
            { ground, 40.0, 0.5, 80.0, 0.8f },
            { ramp, 1.8, 0.5, 2.0, 0.9f },
            { bump0, 0.4, 0.1, 0.3, 0.7f },
            { bump1, 0.4, 0.12, 0.5, 0.7f },
            { bump2, 0.3, 0.08, 0.6, 0.7f },
        };
        for (int i = 0; i < 5; i++) s->world.s[s->world.n++] = surf[i];

        ivp_car_system_template_t tcs;
        ivp_car_system_template_init(&tcs, 4, 2);
        tcs.car_body = car_body;
        for (int i = 0; i < 4; i++) {
            const double wx = (i & 1) ? 0.8 : -0.8;   /* odd = right */
            const double wz = (i & 2) ? -1.35 : 1.35; /* 2,3 = rear */
            tcs.wheel_pos_Bos[i] = ivp_vec3f((float)wx, 0.25f, (float)wz);
            tcs.trace_pos_Bos[i] = tcs.wheel_pos_Bos[i];
            tcs.wheel_radius[i] = (ivp_real_t)0.35f;
            tcs.spring_constant[i] = IVP_REAL(25000.0);
            tcs.spring_damping[i] = IVP_REAL(2400.0);
            tcs.spring_damping_compression[i] = IVP_REAL(1800.0);
            tcs.spring_pre_tension[i] = (ivp_real_t)-0.45f;
        }
        tcs.stabilizer_constant[0] = IVP_REAL(3000.0);
        tcs.stabilizer_constant[1] = IVP_REAL(2000.0);
        tcs.wheel_max_rotation_speed[0] = IVP_REAL(50.0);
        tcs.wheel_max_rotation_speed[1] = IVP_REAL(50.0);
        tcs.extra_gravity_force = IVP_REAL(0.0);
        tcs.body_down_force_vertical_offset = (ivp_real_t)0.2f;
        ok = ivp_car_init(&s->car, &tcs);
        if (ok) ivp_car_set_raycast(&s->car, c3_cast_rays, &s->world);

        ivp_car_system_template_init(&tcs, 6, 3);
        tcs.car_body = truck_body;
        for (int i = 0; i < 6; i++) {
            const double wx = (i & 1) ? 0.9 : -0.9;
            const double wz = 1.9 - 1.9 * (double)(i >> 1); /* axes at z = 1.9, 0, -1.9 */
            tcs.wheel_pos_Bos[i] = ivp_vec3f((float)wx, 0.3f, (float)wz);
            tcs.trace_pos_Bos[i] = tcs.wheel_pos_Bos[i];
            tcs.wheel_radius[i] = (ivp_real_t)0.4f;
            tcs.spring_constant[i] = IVP_REAL(30000.0);
            tcs.spring_damping[i] = IVP_REAL(3000.0);
            tcs.spring_damping_compression[i] = IVP_REAL(2500.0);
            tcs.spring_pre_tension[i] = (ivp_real_t)-0.4f;
        }
        tcs.stabilizer_constant[0] = IVP_REAL(4000.0);
        tcs.stabilizer_constant[1] = IVP_REAL(2500.0);
        tcs.stabilizer_constant[2] = IVP_REAL(2500.0);
        tcs.wheel_max_rotation_speed[0] = IVP_REAL(40.0);
        tcs.wheel_max_rotation_speed[1] = IVP_REAL(40.0);
        tcs.wheel_max_rotation_speed[2] = IVP_REAL(40.0);
        tcs.extra_gravity_force = IVP_REAL(0.0);
        tcs.body_down_force_vertical_offset = (ivp_real_t)0.3f;
        ok = ok && ivp_car_init(&s->truck, &tcs);
        if (ok) ivp_car_set_raycast(&s->truck, c3_cast_rays, &s->world);

        ivp_car_system_template_init(&tcs, 3, 2);
        tcs.car_body = trike_body;
        tcs.wheel_pos_Bos[0] = ivp_vec3f(-0.8f, 0.25f, 0.8f);
        tcs.wheel_pos_Bos[1] = ivp_vec3f(0.8f, 0.25f, 0.8f);
        tcs.wheel_pos_Bos[2] = ivp_vec3f(0.0f, 0.25f, -0.9f);
        for (int i = 0; i < 3; i++) {
            tcs.trace_pos_Bos[i] = tcs.wheel_pos_Bos[i];
            tcs.wheel_radius[i] = (ivp_real_t)0.3f;
            tcs.spring_constant[i] = IVP_REAL(12000.0);
            tcs.spring_damping[i] = IVP_REAL(1100.0);
            tcs.spring_damping_compression[i] = IVP_REAL(900.0);
            tcs.spring_pre_tension[i] = (ivp_real_t)-0.35f;
        }
        tcs.stabilizer_constant[0] = IVP_REAL(1500.0);
        tcs.stabilizer_constant[1] = IVP_REAL(1500.0);
        tcs.wheel_max_rotation_speed[0] = IVP_REAL(60.0);
        tcs.wheel_max_rotation_speed[1] = IVP_REAL(60.0);
        tcs.extra_gravity_force = IVP_REAL(300.0);
        tcs.body_down_force_vertical_offset = (ivp_real_t)0.1f;
        ok = ok && ivp_car_init(&s->trike, &tcs);
        if (ok) ivp_car_set_raycast(&s->trike, c3_cast_rays, &s->world);
    }
    if (ok) {
        c2_add(&sc, ground, "ground");
        c2_add(&sc, ramp, "ramp");
        c2_add(&sc, bump0, "bump");
        c2_add(&sc, bump1, "bump");
        c2_add(&sc, bump2, "bump");
        c2_add(&sc, car_body, "car");
        c2_add(&sc, truck_body, "truck");
        c2_add(&sc, trike_body, "trike");
        rc = c2_run(&sc, out, SCENARIO_RAYCAST_CAR_DRIVE, steps, dt, c3_raycast_car_hook, s);
        if (inv && rc == 0) {
            c2_check_finite(&sc, inv, "raycast_car_drive");
            inv_check(inv, steps < 240 || (double)car_body->physical_core->pos_world_f_core_last_psi.k[2] > -10.0,
                      "raycast_car_drive: the car drove forward: z=%.6f",
                      (double)car_body->physical_core->pos_world_f_core_last_psi.k[2]);
        }
    }
    ivp_car_destroy(&s->car);
    ivp_car_destroy(&s->truck);
    ivp_car_destroy(&s->trike);
    c2_scene_destroy(&sc);
    free(s);
    return rc;
}

/* ── check_dist_events: IVP_Actuator_Check_Dist events with effects ──── */

/* Reference RunnerCheckDistListener / CheckDistScenario: yo-yo (outside ->
 * upward force on, inside -> off), wiper (inside -> reverse the rot motor's
 * max_rotation_speed, outside -> alternate its power), ping-pong (inside ->
 * fixed impulse towards the other post).  The reference listener receives
 * is_outside, libivp's callback is_inside_range. */
struct c3_check_dist;

typedef struct c3_cd_listener {
    struct c3_check_dist *s;
    int which;
} c3_cd_listener_t;

typedef struct c3_check_dist {
    ivp_spring_state_t spring;
    ivp_force_actuator_state_t yoyo_force;
    ivp_constraint_state_t hinge;
    ivp_rot_mot_state_t wiper_motor;
    ivp_object_t *pong;
    ivp_check_dist_state_t cd[5];
    c3_cd_listener_t listener[5];
    float wiper_speed;
    int wiper_power_toggle;
    int events;
} c3_check_dist_t;

static void c3_cd_event(void *ctx, ivp_object_t *obj_a, ivp_object_t *obj_b, ivp_real_t distance,
                        bool is_inside_range)
{
    (void)obj_a;
    (void)obj_b;
    (void)distance;
    const c3_cd_listener_t *l = (const c3_cd_listener_t *)ctx;
    c3_check_dist_t *s = l->s;
    const bool is_outside = !is_inside_range;
    s->events++;
    switch (l->which) {
    case 0:
        ivp_force_actuator_set_force(&s->yoyo_force, is_outside ? IVP_REAL(14.0) : IVP_REAL(0.0));
        break;
    case 1:
    case 2:
        if (!is_outside) {
            s->wiper_speed = -s->wiper_speed;
            ivp_rot_mot_set_max_rotation_speed(&s->wiper_motor, (ivp_real_t)s->wiper_speed);
        } else {
            s->wiper_power_toggle ^= 1;
            ivp_rot_mot_set_power(&s->wiper_motor, s->wiper_power_toggle ? IVP_REAL(4.5) : IVP_REAL(6.0));
        }
        break;
    case 3:
    case 4:
        if (!is_outside) {
            const ivp_vec3r_t pos = s->pong->physical_core->m_world_f_core_last_psi.pos; /* get_position_PSI */
            const ivp_vec3r_t impulse = ivp_vec3r(l->which == 3 ? IVP_REAL(2.0) : IVP_REAL(-2.0), IVP_REAL(0.0),
                                                  IVP_REAL(0.0));
            ivp_object_async_push_ws(s->pong, &pos, &impulse);
        }
        break;
    default:
        break;
    }
}

/* reference IVP_Controller_Factory::create_check_dist + add_listener_check_dist_event */
static bool c3_check_dist(ivp_environment_t *env, ivp_check_dist_state_t *cd,
                          ivp_object_t *o0, double x0, double y0, double z0,
                          ivp_object_t *o1, double x1, double y1, double z1, double range,
                          c3_cd_listener_t *listener)
{
    const ivp_vec3r_t p0 = ivp_vec3r((ivp_real_t)x0, (ivp_real_t)y0, (ivp_real_t)z0);
    const ivp_vec3r_t p1 = ivp_vec3r((ivp_real_t)x1, (ivp_real_t)y1, (ivp_real_t)z1);
    ivp_check_dist_state_setup(cd, o0, &p0, o1, &p1, (ivp_real_t)range);
    ivp_check_dist_set_callback(cd, c3_cd_event, listener);
    return ivp_check_dist_attach(cd, env);
}

static int run_check_dist_events(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    c3_check_dist_t *s = (c3_check_dist_t *)calloc(1, sizeof(*s));
    if (!s) {
        c2_scene_destroy(&sc);
        return 1;
    }
    ivp_object_t *ground = c3_static(&sc, 0.8, 0.0, 20.0, 0.5, 20.0, 0.0, 3.0, 0.0, NULL);
    ivp_object_t *hook = c3_static(&sc, 0.8, 0.0, 0.2, 0.2, 0.2, -6.0, -3.0, 0.0, NULL);
    ivp_object_t *yoyo = c2_ball(&sc, 0.5, 0.1, 0.4, 1.0, 0.064, -6.0, -1.0, 0.0);
    ivp_object_t *pedestal = c3_static(&sc, 0.8, 0.0, 0.15, 0.6, 0.15, 4.0, -4.0, 0.0, NULL);
    ivp_object_t *blade = c2_box(&sc, 0.5, 0.1, 2.0, 0.1, 0.3, 1.0, 0.04, 1.36, 1.34, 4.0, -5.2, 0.0, NULL);
    ivp_object_t *post_l = c3_static(&sc, 0.8, 0.0, 0.2, 0.5, 0.2, -2.5, 2.0, 6.0, NULL);
    ivp_object_t *post_r = c3_static(&sc, 0.8, 0.0, 0.2, 0.5, 0.2, 2.5, 2.0, 6.0, NULL);
    ivp_object_t *pong = c2_ball(&sc, 0.0, 0.0, 0.3, 0.5, 0.018, 0.0, 2.18, 6.0);
    int rc = 1;
    bool ok = !sc.failed && ground && hook && yoyo && pedestal && blade && post_l && post_r && pong;
    if (ok) {
        c2_set_speed(pong, 2.0f, 0.0f, 0.0f);
        s->wiper_speed = 3.0f;
        s->wiper_power_toggle = 0;
        s->events = 0;
        s->pong = pong;

        c2_spring(sc.env, &s->spring, hook, 0.0, 0.2, 0.0, yoyo, 0.0, 0.0, 0.0, 40.0, 0.5, 1.8);
        {
            const ivp_vec3r_t a0 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
            const ivp_vec3r_t a1 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
            ivp_force_actuator_state_init(&s->yoyo_force);
            ivp_force_actuator_state_setup(&s->yoyo_force, yoyo, &a0, yoyo, &a1, IVP_REAL(0.0), true, false);
            ivp_environment_announce_dependent_controller(sc.env, 100, ivp_controller_force_actuator, &s->yoyo_force,
                                                          ivp_force_actuator_discovery);
        }
        {
            const ivp_vec3r_t anchor = ivp_vec3r(IVP_REAL(4.0), IVP_REAL(-5.2), IVP_REAL(0.0));
            const ivp_vec3r_t axis = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
            ivp_constraint_template_t t;
            ivp_constraint_template_init(&t);
            ivp_constraint_template_set_hinge_ws(&t, pedestal, &anchor, &axis, blade);
            c2_constraint(sc.env, &s->hinge, &t);
        }
        {
            const ivp_vec3r_t a0 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
            const ivp_vec3r_t a1 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
            ivp_rot_mot_state_init(&s->wiper_motor);
            ivp_rot_mot_state_setup(&s->wiper_motor, blade, &a0, blade, &a1, IVP_REAL(6.0), IVP_REAL(8.0), IVP_REAL(3.0));
            ivp_environment_announce_dependent_controller(sc.env, 100, ivp_controller_rot_mot, &s->wiper_motor,
                                                          ivp_rot_mot_discovery);
        }
        for (int i = 0; i < 5; i++) {
            s->listener[i].s = s;
            s->listener[i].which = i;
        }
        ok = c3_check_dist(sc.env, &s->cd[0], hook, -6.0, -3.0, 0.0, yoyo, -6.0, -1.0, 0.0, 2.15, &s->listener[0]) &&
             c3_check_dist(sc.env, &s->cd[1], blade, 6.0, -5.2, 0.0, pedestal, 4.0, -5.2, 2.0, 0.6, &s->listener[1]) &&
             c3_check_dist(sc.env, &s->cd[2], blade, 6.0, -5.2, 0.0, pedestal, 4.0, -5.2, -2.0, 0.6, &s->listener[2]) &&
             c3_check_dist(sc.env, &s->cd[3], pong, 0.0, 2.2, 6.0, post_l, -2.5, 2.2, 6.0, 1.0, &s->listener[3]) &&
             c3_check_dist(sc.env, &s->cd[4], pong, 0.0, 2.2, 6.0, post_r, 2.5, 2.2, 6.0, 1.0, &s->listener[4]);
    }
    if (ok) {
        c2_add(&sc, ground, "ground");
        c2_add(&sc, hook, "hook");
        c2_add(&sc, yoyo, "yoyo");
        c2_add(&sc, pedestal, "pedestal");
        c2_add(&sc, blade, "wiper");
        c2_add(&sc, post_l, "post");
        c2_add(&sc, post_r, "post");
        c2_add(&sc, pong, "pong");
        rc = c2_run(&sc, out, SCENARIO_CHECK_DIST_EVENTS, steps, dt, NULL, NULL);
        if (inv && rc == 0) {
            c2_check_finite(&sc, inv, "check_dist_events");
            inv_check(inv, steps < 240 || s->events >= 6, "check_dist_events: events fired: %d", s->events);
        }
    }
    for (int i = 0; i < 5; i++) ivp_check_dist_detach(&s->cd[i]);
    c2_scene_destroy(&sc);
    free(s);
    return rc;
}

/* ── golem_beam: IVP_Controller_Golem::beam_object_to_target_position ── */

/* reference c3_offset_box_surface: build_box_compact_surface with offset
 * points (the core is shifted to the mass center) */
static const ivp_surman_t *c3_offset_box_surman(c2_scene_t *sc, double hx, double hy, double hz,
                                                double ox, double oy, double oz)
{
    double coords[24];
    int n = 0;
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                coords[n++] = (double)sx * hx + ox;
                coords[n++] = (double)sy * hy + oy;
                coords[n++] = (double)sz * hz + oz;
            }
        }
    }
    ivp_compact_surface_t *s = ivp_build_convex_surface_d(NULL, coords, 8);
    if (!s || sc->n_surfaces >= C2_MAX_SURFACES) {
        if (s) ivp_compact_surface_free(NULL, s);
        sc->failed = true;
        return NULL;
    }
    int i = sc->n_surfaces++;
    sc->surfaces[i] = s;
    ivp_surman_polygon_init(&sc->surmans[i], s);
    return &sc->surmans[i].base;
}

/* reference RunnerBeamGolem::resolve_for_problem: count, then
 * beam_object_to_target_position */
typedef struct c3_beam_golem {
    ivp_golem_state_t g;
    int far_count, angle_count;
} c3_beam_golem_t;

static void c3_golem_beam_problem(void *ctx, ivp_object_t *object, ivp_real_t pos_error, ivp_real_t rot_error)
{
    (void)pos_error;
    (void)rot_error;
    c3_beam_golem_t *b = (c3_beam_golem_t *)ctx;
    if (b->g.last_problem == IVP_GP_FAR_DISTANCE) b->far_count++;
    else b->angle_count++;
    ivp_golem_beam_object_to_target_position(&b->g, object);
}

/* IVP_Template_Controller_Golem (+ the given members) + IVP_Controller_Golem constructor */
static void c3_add_golem(ivp_environment_t *env, c3_beam_golem_t *b, ivp_object_t *obj,
                         float max_force, float max_torque, float force_factor, float damp_factor,
                         float torque_factor, float max_delta_position, float max_delta_orientation)
{
    ivp_golem_state_t *g = &b->g;
    ivp_golem_state_init(g);
    ivp_vec3r_set(&g->max_translation_force, (ivp_real_t)max_force, (ivp_real_t)max_force, (ivp_real_t)max_force);
    ivp_vec3r_set(&g->max_torque, (ivp_real_t)max_torque, (ivp_real_t)max_torque, (ivp_real_t)max_torque);
    g->force_factor = (ivp_real_t)force_factor;
    g->damp_factor = (ivp_real_t)damp_factor;
    g->torque_factor = (ivp_real_t)torque_factor;
    g->max_delta_position = (ivp_real_t)max_delta_position;
    g->max_delta_orientation = (ivp_real_t)max_delta_orientation;
    g->problem_fn = c3_golem_beam_problem;
    g->problem_ctx = b;
    c2_core_controller(env, IVP_CP_MOTION, ivp_controller_golem, g, obj);
    ivp_golem_state_setup(g, obj);
}

typedef struct c3_golem_beam {
    c3_beam_golem_t a, b, c;
    int spt;
} c3_golem_beam_t;

static void c3_golem_beam_hook(int step, ivp_environment_t *env, void *ctx)
{
    c3_golem_beam_t *s = (c3_golem_beam_t *)ctx;
    if ((step - 1) % s->spt) return;
    const int tick = (step - 1) / s->spt;
    const ivp_time_t now = ivp_environment_get_time(env);
    switch (tick) {
    case 30: {
        const ivp_vec3r_t p = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.5), IVP_REAL(1.0));
        const ivp_vec3r_t v = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_golem_set_prime_position(&s->b.g, &p, &v, now);
        break;
    }
    case 60: {
        const ivp_quatr_t q = c2_quat(1.0, 0.0, 0.0, 1.2);
        ivp_golem_set_prime_orientation(&s->b.g, &q, now, NULL, IVP_REAL(0.0));
        break;
    }
    case 90: {
        const ivp_vec3r_t p = ivp_vec3r(IVP_REAL(4.0), IVP_REAL(-1.0), IVP_REAL(8.0));
        const ivp_vec3r_t v = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(-3.0));
        ivp_golem_set_prime_position(&s->c.g, &p, &v, now);
        break;
    }
    case 120: {
        const ivp_quatr_t q0 = c2_quat(0.0, 0.0, 1.0, 0.3);
        const ivp_quatr_t q1 = c2_quat(0.0, 0.0, 1.0, 1.0);
        ivp_golem_set_prime_orientation(&s->b.g, &q0, now, &q1, (ivp_real_t)1.5f);
        break;
    }
    case 130: {
        const ivp_quatr_t q = c2_quat(0.0, 1.0, 0.0, -1.5);
        ivp_golem_set_prime_orientation(&s->c.g, &q, now, NULL, IVP_REAL(0.0));
        break;
    }
    case 150: {
        /* the same rotation as +0.5 rad about y, negated (w < 0) */
        ivp_quatr_t q = c2_quat(0.0, 1.0, 0.0, 0.5);
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
        q.w = -q.w;
        ivp_golem_set_prime_orientation(&s->a.g, &q, now, NULL, IVP_REAL(0.0));
        break;
    }
    case 180: {
        const ivp_vec3r_t p = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.5), IVP_REAL(6.0));
        const ivp_vec3r_t v = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(-1.0));
        ivp_golem_set_prime_position(&s->b.g, &p, &v, now);
        break;
    }
    case 200:
        ivp_environment_reset_time(env);
        break;
    case 210: {
        /* 0.3 rad from the ball's orientation, negated: acos_quat < 0 without a problem */
        ivp_quatr_t q = c2_quat(0.0, 1.0, 0.0, -1.2);
        q.x = -q.x;
        q.y = -q.y;
        q.z = -q.z;
        q.w = -q.w;
        ivp_golem_set_prime_orientation(&s->c.g, &q, now, NULL, IVP_REAL(0.0));
        break;
    }
    case 260: {
        const ivp_vec3r_t p = ivp_vec3r(IVP_REAL(-2.0), IVP_REAL(0.0), IVP_REAL(-2.0));
        const ivp_vec3r_t v = ivp_vec3r(IVP_REAL(1.5), IVP_REAL(0.0), IVP_REAL(0.5));
        ivp_golem_set_prime_position(&s->a.g, &p, &v, now);
        break;
    }
    case 330: {
        const ivp_quatr_t q = c2_quat(1.0, 0.0, 0.0, 0.5);
        ivp_golem_set_prime_orientation(&s->c.g, &q, now, NULL, IVP_REAL(0.0));
        break;
    }
    default:
        break;
    }
}

static int run_golem_beam(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    c3_golem_beam_t *s = (c3_golem_beam_t *)calloc(1, sizeof(*s));
    if (!s) {
        c2_scene_destroy(&sc);
        return 1;
    }
    ivp_object_t *ground = c3_static(&sc, 0.8, 0.0, 20.0, 0.5, 20.0, 0.0, 3.0, 0.0, NULL);
    ivp_object_t *a = c2_box(&sc, 0.5, 0.1, 0.4, 0.4, 0.4, 1.5, 0.16, 0.16, 0.16, -4.0, 1.0, -3.0, NULL);
    ivp_object_t *b = NULL;
    {
        const ivp_surman_t *sm = c3_offset_box_surman(&sc, 0.3, 0.25, 0.5, 0.0, 0.0, 0.6);
        if (sm) {
            ivp_object_template_t tmpl;
            ivp_object_template_init(&tmpl);
            tmpl.type = IVP_OBJECT_POLYGON;
            tmpl.mass = IVP_REAL(2.0);
            ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.21), IVP_REAL(0.23), IVP_REAL(0.1));
            tmpl.rot_inertia_is_factor = false;
            tmpl.material.friction = IVP_REAL(0.5);
            tmpl.material.elasticity = IVP_REAL(0.1);
            tmpl.speed_damp_factor = IVP_REAL(0.0);
            ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
            tmpl.physical_unmoveable = false;
            tmpl.orientation = ivp_quatr_identity();
            ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
            b = ivp_environment_create_object(sc.env, &tmpl, sm);
        }
    }
    ivp_object_t *c = c2_ball(&sc, 0.5, 0.1, 0.5, 1.0, 0.1, 4.0, 1.0, 2.0);
    int rc = 1;
    if (!sc.failed && ground && a && b && c) {
        s->spt = c3_steps_per_tick(sc.env);
        const ivp_time_t now = ivp_environment_get_time(sc.env);
        c3_add_golem(sc.env, &s->a, a, 30.0f, 10.0f, 0.8f, 1.0f, 0.8f, 1.5f, 1.0f);
        {
            const ivp_vec3r_t p = ivp_vec3r(IVP_REAL(-4.0), IVP_REAL(1.0), IVP_REAL(-3.0));
            const ivp_vec3r_t v = ivp_vec3r(IVP_REAL(2.5), IVP_REAL(0.0), IVP_REAL(0.0));
            ivp_golem_set_prime_position(&s->a.g, &p, &v, now);
            const ivp_quatr_t q0 = ivp_quatr_identity();
            const ivp_quatr_t q1 = c2_quat(0.0, 1.0, 0.0, 1.5);
            ivp_golem_set_prime_orientation(&s->a.g, &q0, now, &q1, IVP_REAL(2.0));
        }
        c3_add_golem(sc.env, &s->b, b, 60.0f, 20.0f, 0.7f, 0.9f, 0.6f, 3.0f, 0.8f);
        c3_add_golem(sc.env, &s->c, c, 40.0f, 30.0f, 0.8f, 1.0f, 0.8f, 2.5f, (float)(0.5f * IVP_PI));
        c2_add(&sc, ground, "ground");
        c2_add(&sc, a, "golem");
        c2_add(&sc, b, "golem_shifted");
        c2_add(&sc, c, "golem");
        rc = c2_run(&sc, out, SCENARIO_GOLEM_BEAM, steps, dt, c3_golem_beam_hook, s);
        if (inv && rc == 0) {
            c2_check_finite(&sc, inv, "golem_beam");
            inv_check(inv, !b->shift_core_f_object_is_zero, "golem_beam: shifted core");
            inv_check(inv, steps < 720 || (s->a.far_count + s->b.far_count + s->c.far_count > 0 &&
                                           s->b.angle_count > 0),
                      "golem_beam: beams far=%d/%d/%d angle=%d", s->a.far_count, s->b.far_count,
                      s->c.far_count, s->b.angle_count);
        }
    }
    c2_scene_destroy(&sc);
    free(s);
    return rc;
}

/* ── Shared core parity scenarios (merge) ────────────────────────────────
 *
 * Mirror tools/ivp_reference_runner/ref_runner_scenarios_merge.cxx (keep in
 * sync).
 *
 * merge_objects: ivp_environment_merge_objects (IVP_Core_Merged /
 * IVP_Merge_Core) of a box + box + ball compound at setup, of two hanging
 * boxes mid-run and of a sliding box frozen with ivp_object_disable_simulation
 * plus a hanging ball; the merged bodies are pushed / spun with async pushes
 * and hit the ground and other bodies.
 * object_attach: ivp_object_attach_object (current distance and an explicit
 * one) of a flying box (its spring actuator is deleted with its core) and a
 * hanging ball to a spinning parent and of a flying ball to a frozen box,
 * reposition_object_Ros, detach_object of the attached box and ball and of
 * the frozen box (whose core keeps the ball attached to it).
 * merge_buoyancy: the buoyancy attacher on a merged raft (box + box + ball)
 * and on a box that gets a ball attached (buoyancy and dampening for every
 * object of the core).
 * anchor_follow: springs on an attached box and its parent; reposition_object_Ros
 * and detach_object move the box into a new core space, and the springs'
 * anchors follow it (IVP_Real_Object::set_new_m_object_f_core). */

/* reference mg_box / mg_ball: create_dynamic_box / ball, woken only when wake */
static ivp_object_t *mg_box(c2_scene_t *sc, double friction, double elasticity,
                            double hx, double hy, double hz, double mass,
                            double ix, double iy, double iz,
                            double x, double y, double z, const ivp_quatr_t *q, bool wake)
{
    const ivp_surman_t *sm = c2_box_surman(sc, hx, hy, hz);
    if (!sm) return NULL;
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = (ivp_real_t)mass;
    ivp_vec3r_set(&tmpl.rot_inertia, (ivp_real_t)ix, (ivp_real_t)iy, (ivp_real_t)iz);
    tmpl.rot_inertia_is_factor = false;
    tmpl.material.friction = (ivp_real_t)friction;
    tmpl.material.elasticity = (ivp_real_t)elasticity;
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = q ? *q : ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, (ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
    tmpl.start_in_simulation = wake;
    return ivp_environment_create_object(sc->env, &tmpl, sm);
}

static ivp_object_t *mg_ball(c2_scene_t *sc, double friction, double elasticity,
                             double radius, double mass, double inertia, double x, double y, double z, bool wake)
{
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = (ivp_real_t)mass;
    ivp_vec3r_set(&tmpl.rot_inertia, (ivp_real_t)inertia, (ivp_real_t)inertia, (ivp_real_t)inertia);
    tmpl.rot_inertia_is_factor = false;
    tmpl.material.friction = (ivp_real_t)friction;
    tmpl.material.elasticity = (ivp_real_t)elasticity;
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, (ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
    tmpl.ball_radius = (ivp_real_t)(float)radius;
    tmpl.start_in_simulation = wake;
    ivp_object_t *o = ivp_environment_create_object(sc->env, &tmpl, NULL);
    if (!o) sc->failed = true;
    return o;
}

/* reference create_static_box with an orientation */
static ivp_object_t *mg_static(c2_scene_t *sc, double friction, double elasticity,
                               double hx, double hy, double hz, double x, double y, double z, const ivp_quatr_t *q)
{
    const ivp_surman_t *sm = c2_box_surman(sc, hx, hy, hz);
    if (!sm) return NULL;
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.rot_inertia_is_factor = false;
    tmpl.material.friction = (ivp_real_t)friction;
    tmpl.material.elasticity = (ivp_real_t)elasticity;
    tmpl.physical_unmoveable = true;
    tmpl.orientation = q ? *q : ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, (ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
    return ivp_environment_create_object(sc->env, &tmpl, sm);
}

/* reference mg_template (configure_dynamic_template + explicit inertia) */
static void mg_template(ivp_object_template_t *t, double friction, double elasticity,
                        double mass, double ix, double iy, double iz)
{
    ivp_object_template_init(t);
    t->type = IVP_OBJECT_POLYGON;
    t->mass = (ivp_real_t)mass;
    ivp_vec3r_set(&t->rot_inertia, (ivp_real_t)ix, (ivp_real_t)iy, (ivp_real_t)iz);
    t->rot_inertia_is_factor = false;
    t->material.friction = (ivp_real_t)friction;
    t->material.elasticity = (ivp_real_t)elasticity;
    t->speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&t->rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    t->physical_unmoveable = false;
}

/* async_add_speed_object_ws + async_add_rot_speed_object_cs (IVP_U_Float_Point) */
static void mg_push(ivp_object_t *o, float vx, float vy, float vz, float wx, float wy, float wz)
{
    ivp_vec3r_t v = ivp_vec3r((ivp_real_t)vx, (ivp_real_t)vy, (ivp_real_t)vz);
    ivp_object_async_add_speed_ws(o, &v);
    ivp_vec3r_t w = ivp_vec3r((ivp_real_t)wx, (ivp_real_t)wy, (ivp_real_t)wz);
    ivp_object_async_add_rot_speed_cs(o, &w);
}

/* ── merge_objects ────────────────────────────────────────────────────── */

typedef struct mg_merge {
    ivp_object_t *b1, *b2;   /* hanging boxes, merged mid-run */
    ivp_object_t *c1, *c2;   /* sliding box (frozen mid-run) + hanging ball */
    ivp_object_t *heavy;
    bool b_done, c_frozen, c_done;
    int merges;
} mg_merge_t;

static void mg_merge_hook(int step, ivp_environment_t *env, void *ctx)
{
    mg_merge_t *s = (mg_merge_t *)ctx;
    const double t = (double)ivp_environment_get_time(env);
    if (step == 1) {
        /* the setup compound: speed and spin (applied at the first PSI) */
        mg_push(s->heavy, 3.0f, -0.5f, 0.6f, 0.3f, 2.5f, -0.8f);
    }
    if (!s->b_done && t >= 0.75) {
        s->b_done = true;
        ivp_object_t *objs[2] = { s->b2, s->b1 };
        if (ivp_environment_merge_objects(env, objs, 2)) s->merges++;
        mg_push(s->b1, -0.5f, 0.0f, 0.3f, 1.2f, -0.6f, 3.0f);
    }
    if (!s->c_frozen && t >= 0.5) {
        s->c_frozen = true;
        (void)ivp_object_disable_simulation(s->c1);
    }
    if (s->c_frozen && !s->c_done && t >= 0.6) {
        s->c_done = true;
        if (s->c1->object_movement_state == IVP_MT_NOT_SIM && !s->c1->fr_synapses) {
            ivp_object_t *objs[2] = { s->c1, s->c2 };
            if (ivp_environment_merge_objects(env, objs, 2)) s->merges++;
            mg_push(s->c2, -1.0f, -1.5f, 0.0f, 0.0f, 0.0f, 4.0f);
        }
    }
}

static int run_merge_objects(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    mg_merge_t s;
    memset(&s, 0, sizeof(s));

    c2_add(&sc, mg_static(&sc, 0.6, 0.2, 20.0, 0.5, 20.0, 0.0, 3.0, 0.0, NULL), "ground");
    ivp_quatr_t qw = c2_quat(0.0, 1.0, 0.0, 0.4);
    c2_add(&sc, mg_static(&sc, 0.6, 0.2, 0.3, 0.6, 2.0, 4.5, 1.9, 0.0, &qw), "wall");

    /* compound merged at setup: heavy box (main axis), rotated light box, ball */
    ivp_quatr_t qh = c2_quat(0.0, 0.0, 1.0, 0.15);
    ivp_object_t *heavy = mg_box(&sc, 0.4, 0.3, 0.5, 0.25, 0.4, 4.0, 0.29, 0.54, 0.41, 0.0, 1.6, 0.0, &qh, false);
    ivp_quatr_t ql = c2_quat(0.0, 0.0, 1.0, 0.5);
    ivp_object_t *light = mg_box(&sc, 0.4, 0.3, 0.2, 0.2, 0.2, 1.0, 0.027, 0.027, 0.027, 0.85, 1.5, 0.1, &ql, false);
    ivp_object_t *ball = mg_ball(&sc, 0.4, 0.3, 0.25, 0.8, 0.02, -0.8, 1.55, -0.2, false);
    if (heavy && light && ball) {
        ivp_object_t *objs[3] = { heavy, light, ball };
        if (ivp_environment_merge_objects(sc.env, objs, 3)) s.merges++;
    }
    s.heavy = heavy;
    /* a spring from a static hook to the light box (anchor in a merged object) */
    ivp_spring_state_t spring;
    ivp_object_t *hook = mg_static(&sc, 0.6, 0.2, 0.1, 0.1, 0.1, 1.5, -0.5, 0.0, NULL);
    if (hook && light) c2_spring(sc.env, &spring, hook, 0.0, 0.1, 0.0, light, 0.1, -0.1, 0.0, 15.0, 1.5, 1.6);
    c2_add(&sc, hook, "hook");
    c2_add(&sc, heavy, "heavy");
    c2_add(&sc, light, "light");
    c2_add(&sc, ball, "ball");

    /* free bodies on the ground in the compound's way */
    c2_add(&sc, mg_box(&sc, 0.4, 0.3, 0.3, 0.3, 0.3, 1.2, 0.072, 0.072, 0.072, 2.2, 2.15, 0.5, NULL, true), "free_box");
    c2_add(&sc, mg_ball(&sc, 0.4, 0.3, 0.3, 1.0, 0.036, 1.6, 2.1, -0.6, true), "free_ball");

    /* two hanging boxes, merged mid-run */
    ivp_quatr_t qb = c2_quat(1.0, 0.0, 0.0, 0.6);
    s.b1 = mg_box(&sc, 0.4, 0.3, 0.35, 0.2, 0.3, 1.5, 0.065, 0.11, 0.075, -3.0, 0.5, 0.0, NULL, false);
    s.b2 = mg_box(&sc, 0.4, 0.3, 0.15, 0.15, 0.35, 0.7, 0.034, 0.034, 0.011, -2.45, 0.35, 0.25, &qb, false);
    c2_add(&sc, s.b1, "b1");
    c2_add(&sc, s.b2, "b2");

    /* a box sliding on the ground (frozen mid-run with disable_simulation,
     * which removes its contact points) and a hanging ball */
    s.c1 = mg_box(&sc, 0.4, 0.3, 0.3, 0.2, 0.3, 1.0, 0.043, 0.06, 0.043, -2.0, 2.25, 3.0, NULL, true);
    if (s.c1) c2_set_speed(s.c1, 2.5f, 0.0f, 0.2f);
    s.c2 = mg_ball(&sc, 0.4, 0.3, 0.2, 0.5, 0.008, -0.4, 1.6, 3.4, false);
    c2_add(&sc, s.c1, "c1");
    c2_add(&sc, s.c2, "c2");

    int rc = c2_run(&sc, out, SCENARIO_MERGE_OBJECTS, steps, dt, mg_merge_hook, &s);
    if (rc == 0 && inv) {
        c2_check_finite(&sc, inv, "merge_objects");
        inv_check(inv, s.merges == 1 + (s.b_done ? 1 : 0) + (s.c_done ? 1 : 0),
                  "merge_objects: every merge accepted (%d)", s.merges);
        inv_check(inv, heavy->physical_core == light->physical_core && light->physical_core == ball->physical_core &&
                       ivp_pvec_len(&heavy->physical_core->objects) == 3,
                  "merge_objects: setup compound shares one core");
        inv_check(inv, !s.b_done || (s.b1->physical_core == s.b2->physical_core && s.b1->original_core == s.b1->physical_core),
                  "merge_objects: b1/b2 share one core");
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── object_attach ────────────────────────────────────────────────────── */

typedef struct mg_attach {
    ivp_object_t *parent, *box, *ball, *free_box, *proj;
    int done;
} mg_attach_t;

static void mg_attach_hook(int step, ivp_environment_t *env, void *ctx)
{
    (void)step;
    mg_attach_t *s = (mg_attach_t *)ctx;
    const double t = (double)ivp_environment_get_time(env);
    if (s->done == 0 && t >= 0.2) {
        s->done = 1;
        ivp_object_attach_object(s->parent, s->box, (double)-1.0f);
    } else if (s->done == 1 && t >= 0.3) {
        s->done = 2;
        ivp_object_attach_object(s->parent, s->ball, 1.5);   /* the hanging ball is revived first */
    } else if (s->done == 2 && t >= 0.5) {
        s->done = 3;
        ivp_quatr_t q = c2_quat(0.0, 1.0, 0.0, 0.35);
        ivp_vec3r_t shift = ivp_vec3r(IVP_REAL(0.75), IVP_REAL(-0.1), IVP_REAL(0.2));
        (void)ivp_object_reposition_object_Ros(s->parent, s->box, &q, &shift, false);
    } else if (s->done == 3 && t >= 0.55) {
        s->done = 31;
        ivp_object_attach_object(s->free_box, s->proj, (double)-1.0f);   /* the frozen parent is revived */
    } else if (s->done == 31 && t >= 1.8) {
        s->done = 4;
        ivp_object_template_t templ;
        mg_template(&templ, 0.5, 0.2, 1.1, 0.03, 0.03, 0.03);
        ivp_object_detach_object(s->box, &templ);
    } else if (s->done == 4 && t >= 2.4) {
        s->done = 5;
        ivp_object_template_t templ;
        mg_template(&templ, 0.5, 0.2, 2.5, 0.11, 0.11, 0.11);
        ivp_object_detach_object(s->free_box, &templ);
    } else if (s->done == 5 && t >= 2.7) {
        s->done = 6;
        ivp_object_template_t templ;
        mg_template(&templ, 0.5, 0.2, 0.6, 0.015, 0.015, 0.015);
        templ.extra_radius = (ivp_real_t)0.25f;   /* detach_object: the ball radius comes from extra_radius only */
        ivp_object_detach_object(s->ball, &templ);
    }
}

static int run_object_attach(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    mg_attach_t s;
    memset(&s, 0, sizeof(s));

    c2_add(&sc, mg_static(&sc, 0.7, 0.1, 20.0, 0.5, 20.0, 0.0, 3.0, 0.0, NULL), "ground");

    ivp_quatr_t qp = c2_quat(1.0, 0.0, 0.0, 0.2);
    ivp_object_t *parent = mg_box(&sc, 0.5, 0.2, 0.6, 0.2, 0.4, 3.0, 0.2, 0.52, 0.4, 0.0, 0.8, 0.0, &qp, true);
    if (parent) {
        c2_set_speed(parent, 1.0f, -1.0f, 0.0f);
        c2_set_rot_speed(parent, 0.0f, 2.0f, 0.5f);
    }
    ivp_quatr_t qb = c2_quat(0.0, 0.0, 1.0, 0.3);
    ivp_object_t *box = mg_box(&sc, 0.5, 0.2, 0.2, 0.2, 0.2, 1.0, 0.027, 0.027, 0.027, 1.6, 0.4, 0.3, &qb, true);
    if (box) c2_set_speed(box, 0.5f, -2.0f, 0.0f);
    ivp_object_t *ball = mg_ball(&sc, 0.5, 0.2, 0.25, 0.6, 0.015, -1.0, 0.75, -0.5, false);   /* hanging */
    ivp_object_t *free_box = mg_box(&sc, 0.5, 0.2, 0.3, 0.3, 0.3, 1.2, 0.072, 0.072, 0.072, 3.0, 2.15, 1.0, NULL, true);
    ivp_object_t *proj = mg_ball(&sc, 0.5, 0.2, 0.15, 0.4, 0.004, 4.6, 0.2, 1.2, true);
    if (proj) c2_set_speed(proj, -1.2f, -1.5f, 0.0f);
    /* a spring from a static hook to the box: deleted with the box's core at attach_object */
    ivp_spring_state_t spring;
    ivp_object_t *hook = mg_static(&sc, 0.7, 0.1, 0.1, 0.1, 0.1, 2.5, -0.6, 0.3, NULL);
    if (hook && box) c2_spring(sc.env, &spring, hook, 0.0, 0.1, 0.0, box, 0.0, -0.2, 0.0, 20.0, 1.0, 0.8);

    s.parent = parent;
    s.box = box;
    s.ball = ball;
    s.free_box = free_box;
    s.proj = proj;
    c2_add(&sc, parent, "parent");
    c2_add(&sc, box, "box");
    c2_add(&sc, ball, "ball");
    c2_add(&sc, free_box, "free_box");
    c2_add(&sc, proj, "proj");
    c2_add(&sc, hook, "hook");

    int rc = c2_run(&sc, out, SCENARIO_OBJECT_ATTACH, steps, dt, mg_attach_hook, &s);
    if (rc == 0 && inv) {
        c2_check_finite(&sc, inv, "object_attach");
        if (s.done >= 6) {
            inv_check(inv, box->physical_core != parent->physical_core && ball->physical_core != parent->physical_core &&
                           ivp_pvec_len(&parent->physical_core->objects) == 1,
                      "object_attach: detached objects have their own cores");
            inv_check(inv, (float)box->physical_core->mass == 1.1f && (float)free_box->physical_core->mass == 2.5f,
                      "object_attach: detach_object applies the template mass");
        }
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── merge_buoyancy ───────────────────────────────────────────────────── */

typedef struct mg_buoyancy {
    ivp_object_t *raft_a, *parent, *child;
    int done;
} mg_buoyancy_t;

static void mg_buoyancy_hook(int step, ivp_environment_t *env, void *ctx)
{
    mg_buoyancy_t *s = (mg_buoyancy_t *)ctx;
    const double t = (double)ivp_environment_get_time(env);
    if (step == 1) {
        mg_push(s->raft_a, 0.5f, 0.0f, 0.2f, 0.5f, 1.0f, 0.0f);   /* wakes the merged raft */
    }
    if (s->done == 0 && t >= 0.15) {
        s->done = 1;
        ivp_object_attach_object(s->parent, s->child, (double)-1.0f);
    }
}

static int run_merge_buoyancy(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    mg_buoyancy_t s;
    memset(&s, 0, sizeof(s));

    c2_add(&sc, mg_static(&sc, 0.8, 0.0, 15.0, 0.5, 15.0, 0.0, 5.0, 0.0, NULL), "ground");

    /* a raft of two boxes and a ball merged at setup */
    ivp_object_t *raft_a = mg_box(&sc, 0.5, 0.2, 0.6, 0.15, 0.4, 0.4, 0.025, 0.06, 0.05, -2.0, -2.0, 0.0, NULL, false);
    ivp_quatr_t qb = c2_quat(0.0, 0.0, 1.0, 0.4);
    ivp_object_t *raft_b = mg_box(&sc, 0.5, 0.2, 0.25, 0.25, 0.25, 0.3, 0.0125, 0.0125, 0.0125, -1.3, -2.15, 0.25, &qb, false);
    ivp_object_t *raft_ball = mg_ball(&sc, 0.5, 0.2, 0.3, 0.2, 0.0072, -2.8, -2.05, -0.25, false);
    bool merged = false;
    if (raft_a && raft_b && raft_ball) {
        ivp_object_t *objs[3] = { raft_a, raft_b, raft_ball };
        merged = ivp_environment_merge_objects(sc.env, objs, 3);
    }
    s.raft_a = raft_a;
    c2_add(&sc, raft_a, "raft_a");
    c2_add(&sc, raft_b, "raft_b");
    c2_add(&sc, raft_ball, "raft_ball");

    ivp_object_t *single = mg_box(&sc, 0.5, 0.2, 0.4, 0.4, 0.4, 0.5, 0.053, 0.053, 0.053, 0.5, -3.0, 1.5, NULL, true);
    c2_add(&sc, single, "single");

    /* a box and a ball falling side by side; the ball is attached to the box */
    ivp_object_t *parent = mg_box(&sc, 0.5, 0.2, 0.5, 0.25, 0.5, 0.6, 0.0375, 0.1, 0.0375, 2.5, -3.5, 0.0, NULL, true);
    if (parent) c2_set_speed(parent, 0.0f, 1.0f, 0.0f);
    ivp_object_t *child = mg_ball(&sc, 0.5, 0.2, 0.25, 0.3, 0.0075, 3.6, -3.4, 0.5, true);
    if (child) c2_set_speed(child, 0.0f, 1.0f, 0.0f);
    s.parent = parent;
    s.child = child;
    c2_add(&sc, parent, "parent");
    c2_add(&sc, child, "child");

    int rc = 1;
    if (!sc.failed && merged) {
        /* IVP_Attacher_To_Cores_Buoyancy: liquid for y > -1 (hesse (0,1,0,1)),
         * current (1.0, 0, 0.3) */
        ivp_buoyancy_state_t templ;
        ivp_buoyancy_init(&templ, IVP_REAL(-1.0), (ivp_real_t)2.5f);   /* medium_density */
        templ.linear_drag = (ivp_real_t)2.0f;                          /* pressure_damp_factor */
        templ.friction_damp_factor = (ivp_real_t)0.1f;
        templ.angular_drag = (ivp_real_t)1.0f;                         /* ball_rot_dampening_factor */
        templ.viscosity_factor = (ivp_real_t)0.05f;
        templ.use_interpolation = false;
        ivp_vec3r_set(&templ.flow_velocity, (ivp_real_t)1.0f, IVP_REAL(0.0), (ivp_real_t)0.3f);
        ivp_set_active_t set;
        ivp_set_active_init(&set, NULL, 16);
        ivp_set_active_install(&set, raft_a->physical_core);   /* the merged core: 3 objects */
        ivp_set_active_install(&set, single->physical_core);
        ivp_set_active_install(&set, parent->physical_core);   /* gets the attached ball */
        ivp_buoyancy_attacher_t *ba = ivp_buoyancy_attacher_create(sc.env, &templ, &set);
        if (ba) {
            rc = c2_run(&sc, out, SCENARIO_MERGE_BUOYANCY, steps, dt, mg_buoyancy_hook, &s);
            if (rc == 0 && inv) {
                c2_check_finite(&sc, inv, "merge_buoyancy");
                inv_check(inv, raft_a->physical_core == raft_ball->physical_core &&
                               (s.done == 0 || child->physical_core == parent->physical_core),
                          "merge_buoyancy: shared cores");
                ivp_buoyancy_core_data_t *cd = ivp_buoyancy_attacher_get_core_data(ba, raft_a->physical_core);
                inv_check(inv, cd && cd->interpolator_count == 3, "merge_buoyancy: raft buoyancy for its 3 objects");
                inv_check(inv, raft_a->physical_core->pos_world_f_core_last_psi.k[1] < IVP_REAL(0.0),
                          "merge_buoyancy: the raft floats");
            }
        }
        ivp_set_active_destroy(&set);   /* deletes the attachments and the attacher */
        ivp_buoyancy_state_destroy(&templ);
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── anchor_follow ────────────────────────────────────────────────────── */

typedef struct mg_anchor_follow {
    ivp_object_t *parent, *box, *hook;
    ivp_spring_state_t spring_box, spring_parent;
    int done;
} mg_anchor_follow_t;

static void mg_anchor_follow_hook(int step, ivp_environment_t *env, void *ctx)
{
    (void)step;
    mg_anchor_follow_t *s = (mg_anchor_follow_t *)ctx;
    const double t = (double)ivp_environment_get_time(env);
    if (s->done == 0 && t >= 0.1) {
        s->done = 1;
        ivp_object_attach_object(s->parent, s->box, (double)-1.0f);
        /* after the attach: the springs act on the parent's core through the box's anchors */
        c2_spring(env, &s->spring_box, s->hook, 0.0, 0.1, 0.0, s->box, 0.1, -0.2, 0.05, 30.0, 1.5, 0.6);
        c2_spring(env, &s->spring_parent, s->hook, 0.3, 0.1, 0.0, s->parent, -0.4, 0.0, 0.1, 15.0, 1.0, 0.9);
    } else if (s->done == 1 && t >= 0.5) {
        s->done = 2;
        ivp_quatr_t q = c2_quat(0.0, 1.0, 0.0, 0.6);
        ivp_vec3r_t shift = ivp_vec3r(IVP_REAL(0.6), IVP_REAL(-0.15), IVP_REAL(0.25));
        (void)ivp_object_reposition_object_Ros(s->parent, s->box, &q, &shift, false);
    } else if (s->done == 2 && t >= 1.0) {
        s->done = 3;
        ivp_object_template_t templ;
        mg_template(&templ, 0.5, 0.2, 1.1, 0.03, 0.03, 0.03);
        ivp_object_detach_object(s->box, &templ);
    }
}

static int run_anchor_follow(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    c2_scene_t sc;
    if (!c2_scene_init(&sc, dt)) return 1;
    mg_anchor_follow_t s;
    memset(&s, 0, sizeof(s));

    c2_add(&sc, mg_static(&sc, 0.7, 0.1, 20.0, 0.5, 20.0, 0.0, 3.0, 0.0, NULL), "ground");

    ivp_quatr_t qp = c2_quat(1.0, 0.0, 0.0, 0.2);
    ivp_object_t *parent = mg_box(&sc, 0.5, 0.2, 0.6, 0.2, 0.4, 3.0, 0.2, 0.52, 0.4, 0.0, 0.8, 0.0, &qp, true);
    if (parent) {
        c2_set_speed(parent, 0.5f, -1.0f, 0.0f);
        c2_set_rot_speed(parent, 0.0f, 1.0f, 0.3f);
    }
    ivp_quatr_t qb = c2_quat(0.0, 0.0, 1.0, 0.3);
    ivp_object_t *box = mg_box(&sc, 0.5, 0.2, 0.2, 0.2, 0.2, 1.0, 0.027, 0.027, 0.027, 1.2, 0.5, 0.3, &qb, true);
    ivp_object_t *hook = mg_static(&sc, 0.7, 0.1, 0.1, 0.1, 0.1, 1.0, -1.5, 0.2, NULL);

    s.parent = parent;
    s.box = box;
    s.hook = hook;
    c2_add(&sc, parent, "parent");
    c2_add(&sc, box, "box");
    c2_add(&sc, hook, "hook");

    int rc = (parent && box && hook) ? c2_run(&sc, out, SCENARIO_ANCHOR_FOLLOW, steps, dt, mg_anchor_follow_hook, &s) : 1;
    if (rc == 0 && inv) {
        c2_check_finite(&sc, inv, "anchor_follow");
        if (s.done >= 3) {
            inv_check(inv, box->physical_core != parent->physical_core, "anchor_follow: the box is detached");
            inv_check(inv, s.spring_box.anchors[1].linked && s.spring_box.anchors[1].l_anchor_object == box,
                      "anchor_follow: the box spring stays anchored to the box");
        }
    }
    c2_scene_destroy(&sc);
    return rc;
}

/* ── CLI ────────────────────────────────────────────────────────────────── */

static int parse_scenario_name(const char *name, int *scenario_out)
{
    if (!strcmp(name, "freefall"))            *scenario_out = SCENARIO_FREEFALL;
    else if (!strcmp(name, "two_balls"))      *scenario_out = SCENARIO_TWO_BALLS;
    else if (!strcmp(name, "slope_friction")) *scenario_out = SCENARIO_SLOPE_FRICTION;
    else if (!strcmp(name, "springs"))        *scenario_out = SCENARIO_SPRINGS;
    else if (!strcmp(name, "rope"))           *scenario_out = SCENARIO_ROPE;
    else if (!strcmp(name, "motor"))          *scenario_out = SCENARIO_MOTOR;
    else if (!strcmp(name, "buoyancy"))       *scenario_out = SCENARIO_BUOYANCY;
    else if (!strcmp(name, "cubes"))          *scenario_out = SCENARIO_CUBES;
    else if (!strcmp(name, "force_actuator")) *scenario_out = SCENARIO_FORCE_ACTUATOR;
    else if (!strcmp(name, "forcefield"))     *scenario_out = SCENARIO_FORCEFIELD;
    else if (!strcmp(name, "stiff_spring"))   *scenario_out = SCENARIO_STIFF_SPRING;
    else if (!strcmp(name, "check_distance")) *scenario_out = SCENARIO_CHECK_DISTANCE;
    else if (!strcmp(name, "collision_filter")) *scenario_out = SCENARIO_COLLISION_FILTER;
    else if (!strcmp(name, "motion_controller")) *scenario_out = SCENARIO_MOTION_CONTROLLER;
    else if (!strcmp(name, "phantom"))        *scenario_out = SCENARIO_PHANTOM;
    else if (!strcmp(name, "vehicle"))        *scenario_out = SCENARIO_VEHICLE;
    else if (!strcmp(name, "car_real_wheels")) *scenario_out = SCENARIO_CAR_REAL_WHEELS;
    else if (!strcmp(name, "torque")) *scenario_out = SCENARIO_TORQUE;
    else if (!strcmp(name, "stabilizer")) *scenario_out = SCENARIO_STABILIZER;
    else if (!strcmp(name, "floating")) *scenario_out = SCENARIO_FLOATING;
    else if (!strcmp(name, "world_friction")) *scenario_out = SCENARIO_WORLD_FRICTION;
    else if (!strcmp(name, "golem")) *scenario_out = SCENARIO_GOLEM;
    else if (!strcmp(name, "fixed_keyed")) *scenario_out = SCENARIO_FIXED_KEYED;
    else if (!strcmp(name, "hinge_limits")) *scenario_out = SCENARIO_HINGE_LIMITS;
    else if (!strcmp(name, "cardan_tense")) *scenario_out = SCENARIO_CARDAN_TENSE;
    else if (!strcmp(name, "slider_limits")) *scenario_out = SCENARIO_SLIDER_LIMITS;
    else if (!strcmp(name, "constraint_break")) *scenario_out = SCENARIO_CONSTRAINT_BREAK;
    else if (!strcmp(name, "attacher")) *scenario_out = SCENARIO_ATTACHER;
    else if (!strcmp(name, "actuator_extra")) *scenario_out = SCENARIO_ACTUATOR_EXTRA;
    else if (!strcmp(name, "airboat")) *scenario_out = SCENARIO_AIRBOAT;
    else if (!strcmp(name, "fake_jetski")) *scenario_out = SCENARIO_FAKE_JETSKI;
    else if (!strcmp(name, "raycast_car_drive")) *scenario_out = SCENARIO_RAYCAST_CAR_DRIVE;
    else if (!strcmp(name, "check_dist_events")) *scenario_out = SCENARIO_CHECK_DIST_EVENTS;
    else if (!strcmp(name, "golem_beam")) *scenario_out = SCENARIO_GOLEM_BEAM;
    else if (!strcmp(name, "concave_static")) *scenario_out = SCENARIO_CONCAVE_STATIC;
    else if (!strcmp(name, "compound_dynamic")) *scenario_out = SCENARIO_COMPOUND_DYNAMIC;
    else if (!strcmp(name, "convex_hulls"))   *scenario_out = SCENARIO_CONVEX_HULLS;
    else if (!strcmp(name, "grid_terrain"))   *scenario_out = SCENARIO_GRID_TERRAIN;
    else if (!strcmp(name, "hull_pile"))      *scenario_out = SCENARIO_HULL_PILE;
    else if (!strcmp(name, "galton"))         *scenario_out = SCENARIO_GALTON;
    else if (!strcmp(name, "fast_impacts"))   *scenario_out = SCENARIO_FAST_IMPACTS;
    else if (!strcmp(name, "spawn_remove"))   *scenario_out = SCENARIO_SPAWN_REMOVE;
    else if (!strcmp(name, "long_range"))     *scenario_out = SCENARIO_LONG_RANGE;
    else if (!strcmp(name, "universe_evict")) *scenario_out = SCENARIO_UNIVERSE_EVICT;
    else if (!strcmp(name, "merge_objects")) *scenario_out = SCENARIO_MERGE_OBJECTS;
    else if (!strcmp(name, "object_attach")) *scenario_out = SCENARIO_OBJECT_ATTACH;
    else if (!strcmp(name, "merge_buoyancy")) *scenario_out = SCENARIO_MERGE_BUOYANCY;
    else if (!strcmp(name, "anchor_follow")) *scenario_out = SCENARIO_ANCHOR_FOLLOW;
    else return 0;
    return 1;
}

static void print_usage(const char *exe)
{
    fprintf(stderr,
            "Usage: %s --scenario freefall|two_balls|slope_friction|springs|rope|motor|buoyancy|"
            "cubes|force_actuator|forcefield|stiff_spring|check_distance|collision_filter|"
            "motion_controller|phantom|vehicle|car_real_wheels|"
            "torque|stabilizer|floating|world_friction|golem|fixed_keyed|hinge_limits|cardan_tense|"
            "slider_limits|constraint_break|attacher|actuator_extra|"
            "airboat|fake_jetski|"
            "raycast_car_drive|check_dist_events|golem_beam|"
            "concave_static|compound_dynamic|convex_hulls|grid_terrain|hull_pile|galton|fast_impacts|"
            "spawn_remove|long_range|universe_evict|merge_objects|object_attach|merge_buoyancy|anchor_follow "
            "[--steps N] [--dt seconds] [--output file.jsonl] [--check-invariants] [--assert-continue] "
            "[--benchmark [--samples N] [--warmup N] [--min-sample-ms MS]]\n"
            "Outputs JSONL snapshots, or benchmark records in benchmark mode.\n"
            "With --check-invariants, runs physics correctness checks after simulation.\n", exe);
}

/* --assert-continue: report failed IVP_ASSERT / CORE like the reference
 * debug build (ivu_types.hxx) and go on, the equivalent of running the
 * reference debug runner with SIGINT ignored. */
static void assert_continue_handler(const char *expr, const char *file, int line, const char *func,
                                    const char *msg, void *userdata)
{
    (void)func;
    (void)msg;
    (void)userdata;
    fprintf(stderr, "\nASSERTION FAILURE: %s\nFILE: %s\nLINE: %d\n\n", expr, file, line);
}

int main(int argc, char **argv)
{
    int    scenario = -1;
    int    steps    = 240;
    double dt       = 1.0 / 60.0;
    const char *output_path = NULL;
    int    check_invariants = 0;
    int    benchmark = 0;
    int    samples = 5;
    int    warmup = 1;
    double min_sample_ms = 200.0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            print_usage(argv[0]);
            return 0;
        }
        if (!strcmp(argv[i], "--scenario")) {
            if (i + 1 >= argc) { print_usage(argv[0]); return 2; }
            i++;
            if (!parse_scenario_name(argv[i], &scenario)) {
                fprintf(stderr, "Unknown scenario: %s\n", argv[i]);
                return 2;
            }
            continue;
        }
        if (!strcmp(argv[i], "--steps")) {
            if (i + 1 >= argc) { print_usage(argv[0]); return 2; }
            steps = atoi(argv[++i]);
            if (steps < 0) { fprintf(stderr, "Invalid --steps\n"); return 2; }
            continue;
        }
        if (!strcmp(argv[i], "--dt")) {
            if (i + 1 >= argc) { print_usage(argv[0]); return 2; }
            dt = atof(argv[++i]);
            if (dt <= 0.0) { fprintf(stderr, "Invalid --dt\n"); return 2; }
            continue;
        }
        if (!strcmp(argv[i], "--output")) {
            if (i + 1 >= argc) { print_usage(argv[0]); return 2; }
            output_path = argv[++i];
            continue;
        }
        if (!strcmp(argv[i], "--check-invariants")) {
            check_invariants = 1;
            continue;
        }
        if (!strcmp(argv[i], "--benchmark")) {
            benchmark = 1;
            continue;
        }
        if (!strcmp(argv[i], "--samples")) {
            if (i + 1 >= argc || (samples = atoi(argv[++i])) <= 0) {
                fprintf(stderr, "Invalid --samples\n"); return 2;
            }
            continue;
        }
        if (!strcmp(argv[i], "--warmup")) {
            if (i + 1 >= argc || (warmup = atoi(argv[++i])) < 0) {
                fprintf(stderr, "Invalid --warmup\n"); return 2;
            }
            continue;
        }
        if (!strcmp(argv[i], "--min-sample-ms")) {
            if (i + 1 >= argc || (min_sample_ms = atof(argv[++i])) <= 0.0) {
                fprintf(stderr, "Invalid --min-sample-ms\n"); return 2;
            }
            continue;
        }
        if (!strcmp(argv[i], "--assert-continue")) {
            ivp_set_assert_handler(assert_continue_handler, NULL);
            continue;
        }
        fprintf(stderr, "Unknown arg: %s\n", argv[i]);
        print_usage(argv[0]);
        return 2;
    }

    if (scenario < 0) {
        fprintf(stderr, "No --scenario specified.\n");
        print_usage(argv[0]);
        return 2;
    }
    if (benchmark && (steps <= 0 || output_path || check_invariants)) {
        fprintf(stderr, "Benchmark mode requires steps > 0 and cannot use --output or --check-invariants.\n");
        return 2;
    }

    FILE *out = benchmark ? NULL : stdout;
    if (output_path) {
        out = fopen(output_path, "w");
        if (!out) {
            fprintf(stderr, "Cannot open output file: %s\n", output_path);
            return 1;
        }
    }

    invariant_result_t inv_result;
    invariant_result_t *inv = NULL;
    if (check_invariants) {
        inv_init(&inv_result);
        inv = &inv_result;
    }

    int rc = 0;
    g_benchmark_enabled = benchmark;
    const int first_sample = benchmark ? -warmup : 0;
    const int sample_limit = benchmark ? samples : 1;
    const int64_t minimum_ns = (int64_t)(min_sample_ms * 1000000.0);
    for (int sample = first_sample; sample < sample_limit && rc == 0; sample++) {
        int worlds = 0;
        int64_t aggregate_simulate_ns = 0;
        int64_t aggregate_total_ns = 0;
        do {
            g_benchmark_simulate_ns = 0;
            int64_t total_start = benchmark ? benchmark_now_ns() : 0;
            switch (scenario) {
        case SCENARIO_FREEFALL:       rc = run_freefall(out, steps, dt, inv);       break;
        case SCENARIO_TWO_BALLS:      rc = run_two_balls(out, steps, dt, inv);      break;
        case SCENARIO_SLOPE_FRICTION: rc = run_slope_friction(out, steps, dt, inv); break;
        case SCENARIO_SPRINGS:        rc = run_springs(out, steps, dt, inv);        break;
        case SCENARIO_ROPE:           rc = run_rope(out, steps, dt, inv);           break;
        case SCENARIO_MOTOR:          rc = run_motor(out, steps, dt, inv);          break;
        case SCENARIO_BUOYANCY:       rc = run_buoyancy(out, steps, dt, inv);       break;
        case SCENARIO_CUBES:          rc = run_cubes(out, steps, dt, inv);          break;
        case SCENARIO_FORCE_ACTUATOR: rc = run_force_actuator(out, steps, dt, inv); break;
        case SCENARIO_FORCEFIELD:     rc = run_forcefield(out, steps, dt, inv);     break;
        case SCENARIO_STIFF_SPRING:   rc = run_stiff_spring(out, steps, dt, inv);   break;
        case SCENARIO_CHECK_DISTANCE: rc = run_check_distance(out, steps, dt, inv); break;
        case SCENARIO_COLLISION_FILTER: rc = run_collision_filter(out, steps, dt, inv); break;
        case SCENARIO_MOTION_CONTROLLER: rc = run_motion_controller(out, steps, dt, inv); break;
        case SCENARIO_PHANTOM:        rc = run_phantom(out, steps, dt, inv);        break;
        case SCENARIO_VEHICLE:        rc = run_vehicle(out, steps, dt, inv);        break;
        case SCENARIO_CAR_REAL_WHEELS: rc = run_car_real_wheels(out, steps, dt, inv); break;
        case SCENARIO_TORQUE: rc = run_torque(out, steps, dt, inv); break;
        case SCENARIO_STABILIZER: rc = run_stabilizer(out, steps, dt, inv); break;
        case SCENARIO_FLOATING: rc = run_floating(out, steps, dt, inv); break;
        case SCENARIO_WORLD_FRICTION: rc = run_world_friction(out, steps, dt, inv); break;
        case SCENARIO_GOLEM: rc = run_golem(out, steps, dt, inv); break;
        case SCENARIO_FIXED_KEYED: rc = run_fixed_keyed(out, steps, dt, inv); break;
        case SCENARIO_HINGE_LIMITS: rc = run_hinge_limits(out, steps, dt, inv); break;
        case SCENARIO_CARDAN_TENSE: rc = run_cardan_tense(out, steps, dt, inv); break;
        case SCENARIO_SLIDER_LIMITS: rc = run_slider_limits(out, steps, dt, inv); break;
        case SCENARIO_CONSTRAINT_BREAK: rc = run_constraint_break(out, steps, dt, inv); break;
        case SCENARIO_ATTACHER: rc = run_attacher(out, steps, dt, inv); break;
        case SCENARIO_ACTUATOR_EXTRA: rc = run_actuator_extra(out, steps, dt, inv); break;
        case SCENARIO_AIRBOAT: rc = run_airboat(out, steps, dt, inv); break;
        case SCENARIO_FAKE_JETSKI: rc = run_fake_jetski(out, steps, dt, inv); break;
        case SCENARIO_RAYCAST_CAR_DRIVE: rc = run_raycast_car_drive(out, steps, dt, inv); break;
        case SCENARIO_CHECK_DIST_EVENTS: rc = run_check_dist_events(out, steps, dt, inv); break;
        case SCENARIO_GOLEM_BEAM: rc = run_golem_beam(out, steps, dt, inv); break;
        case SCENARIO_CONCAVE_STATIC: rc = run_concave_static(out, steps, dt, inv); break;
        case SCENARIO_COMPOUND_DYNAMIC: rc = run_compound_dynamic(out, steps, dt, inv); break;
        case SCENARIO_CONVEX_HULLS:   rc = run_convex_hulls(out, steps, dt, inv);   break;
        case SCENARIO_GRID_TERRAIN:   rc = run_grid_terrain(out, steps, dt, inv);   break;
        case SCENARIO_HULL_PILE:      rc = run_hull_pile(out, steps, dt, inv);      break;
        case SCENARIO_GALTON:         rc = run_galton(out, steps, dt, inv);         break;
        case SCENARIO_FAST_IMPACTS:   rc = run_fast_impacts(out, steps, dt, inv);   break;
        case SCENARIO_SPAWN_REMOVE:   rc = run_spawn_remove(out, steps, dt, inv);   break;
        case SCENARIO_LONG_RANGE:     rc = run_long_range(out, steps, dt, inv);     break;
        case SCENARIO_UNIVERSE_EVICT: rc = run_universe_evict(out, steps, dt, inv); break;
        case SCENARIO_MERGE_OBJECTS:  rc = run_merge_objects(out, steps, dt, inv);  break;
        case SCENARIO_OBJECT_ATTACH:  rc = run_object_attach(out, steps, dt, inv);  break;
        case SCENARIO_MERGE_BUOYANCY: rc = run_merge_buoyancy(out, steps, dt, inv); break;
        case SCENARIO_ANCHOR_FOLLOW:  rc = run_anchor_follow(out, steps, dt, inv);  break;
            default:                      rc = 1;                                       break;
            }
            if (benchmark) {
                aggregate_total_ns += benchmark_now_ns() - total_start;
                aggregate_simulate_ns += g_benchmark_simulate_ns;
                worlds++;
            }
        } while (benchmark && sample >= 0 && rc == 0 &&
                 aggregate_simulate_ns < minimum_ns);

        if (benchmark && sample >= 0 && rc == 0) {
            fprintf(stdout,
                    "{\"schema\":\"libivp-benchmark-v1\",\"kind\":\"scenario\","
                    "\"implementation\":\"libivp\",\"scenario\":\"%s\","
                    "\"sample\":%d,\"worlds\":%d,\"steps_per_world\":%d,"
                    "\"dt\":%.17g,\"simulate_ns\":%lld,\"total_ns\":%lld}\n",
                    scenario_name_from_id(scenario), sample, worlds, steps, dt,
                    (long long)aggregate_simulate_ns, (long long)aggregate_total_ns);
        }
    }

    if (output_path && out) fclose(out);

    if (check_invariants && rc == 0) {
        inv_report(&inv_result, scenario_name_from_id(scenario));
        if (inv_result.checks_failed > 0)
            rc = 1;
    }

    return rc;
}
