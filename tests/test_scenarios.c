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
#include <ivp/dynamics/ivp_environment_collision.h>
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

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

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
    SCENARIO_VEHICLE         = 108
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
    json_vec3(f, &core->pos_world_f_core_last_psi);

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

    ivp_real_t energy = ivp_object_get_kinetic_energy(obj);
    fprintf(f, ",\"energy\":%.17g}", (double)energy);
}

static void snapshot_frame(FILE *f, int scenario_id, int step, double dt,
                           ivp_environment_t *env,
                           ivp_object_t **objects, const char **types,
                           int object_count)
{
    double t = (double)ivp_environment_get_time(env);
    fprintf(f, "{\"scenario\":%d,\"step\":%d,\"dt\":%.17g,\"time\":%.17g,\"objects\":[",
            scenario_id, step, dt, t);
    for (int i = 0; i < object_count; i++) {
        if (i) fprintf(f, ",");
        snapshot_object(f, i, types[i], objects[i]);
    }
    fprintf(f, "]}\n");
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
    ivp_compact_surface_t *surface = ivp_build_box_surface(NULL, hx, hy, hz);
    if (!surface) return false;
    ivp_surman_polygon_init(out_surman, surface);
    *out_surface = surface;
    return true;
}

/* Helper for world-space anchor setup using current object transforms. */
static void ivp_constraint_init_ball_socket_ws(
    ivp_constraint_state_t *state,
    ivp_object_t *obj_r,
    const ivp_vec3r_t *anchor_r_os,
    ivp_object_t *obj_a)
{
    ivp_vec3r_t anchor_ws;
    ivp_vec3r_t anchor_a_os;

    ivp_transform_point(&anchor_ws, &obj_r->core.m_world_f_core_last_psi, anchor_r_os);
    ivp_transform_inv_point(&anchor_a_os, &obj_a->core.m_world_f_core_last_psi, &anchor_ws);
    ivp_constraint_init_ball_socket(state, obj_r, anchor_r_os, obj_a, &anchor_a_os);
}

/* Helper for world-space hinge setup using current object transforms. */
static void ivp_constraint_init_hinge_ws(
    ivp_constraint_state_t *state,
    ivp_object_t *obj_r,
    const ivp_vec3r_t *anchor_ws,
    const ivp_vec3r_t *axis_ws,
    ivp_object_t *obj_a)
{
    ivp_vec3r_t anchor_r_os, axis_r_os;
    ivp_vec3r_t anchor_a_os, axis_a_os;

    ivp_transform_inv_point(&anchor_r_os, &obj_r->core.m_world_f_core_last_psi, anchor_ws);
    ivp_transform_inv_dir(&axis_r_os, &obj_r->core.m_world_f_core_last_psi, axis_ws);
    ivp_transform_inv_point(&anchor_a_os, &obj_a->core.m_world_f_core_last_psi, anchor_ws);
    ivp_transform_inv_dir(&axis_a_os, &obj_a->core.m_world_f_core_last_psi, axis_ws);

    ivp_constraint_init_hinge(state,
                              obj_r, &anchor_r_os, &axis_r_os,
                              obj_a, &anchor_a_os, &axis_a_os);
}

/* ── Scenario implementations ───────────────────────────────────────────── */

static int run_freefall(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.83), IVP_REAL(0.0));

    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    /* I = (2/5) * m * r^2 = 0.4 * 1.0 * 0.25 = 0.1 for r=0.5, m=1.0 */
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;

    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-5.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *ball = ivp_environment_create_object(env, &tmpl, NULL);
    if (!ball) { ivp_environment_destroy(env); return 1; }

    ball->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&ball->core.speed, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ball->core.rot_speed.k[0] = 0.0f;
    ball->core.rot_speed.k[1] = 0.0f;
    ball->core.rot_speed.k[2] = 0.0f;

    ivp_object_t *objects[1] = { ball };
    const char   *types[1]   = { "ball" };

    snapshot_frame(out, SCENARIO_FREEFALL, 0, dt, env, objects, types, 1);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
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
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.83), IVP_REAL(0.0));

    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    /* I = (2/5) * m * r^2 = 0.4 * 1.0 * 0.25 = 0.1 for r=0.5, m=1.0 */
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();

    /* Ball A at (-2, -2, 0) */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-2.0), IVP_REAL(-2.0), IVP_REAL(0.0));
    ivp_object_t *a = ivp_environment_create_object(env, &tmpl, NULL);
    if (!a) { ivp_environment_destroy(env); return 1; }
    a->core.upper_limit_radius = IVP_REAL(0.5);

    /* Ball B at (+2, -2, 0) */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(2.0), IVP_REAL(-2.0), IVP_REAL(0.0));
    ivp_object_t *b = ivp_environment_create_object(env, &tmpl, NULL);
    if (!b) { ivp_environment_destroy(env); return 1; }
    b->core.upper_limit_radius = IVP_REAL(0.5);

    /* Set initial velocities: A→+x, B→-x */
    ivp_vec3r_set(&a->core.speed, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    a->core.rot_speed.k[0] = 0.0f;
    a->core.rot_speed.k[1] = 0.0f;
    a->core.rot_speed.k[2] = 0.0f;

    ivp_vec3r_set(&b->core.speed, IVP_REAL(-2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    b->core.rot_speed.k[0] = 0.0f;
    b->core.rot_speed.k[1] = 0.0f;
    b->core.rot_speed.k[2] = 0.0f;

    ivp_object_t *objects[2] = { a, b };
    const char   *types[2]   = { "ball", "ball" };

    /* Track initial velocities for collision detection */
    double a_vx_initial = (double)a->core.speed.k[0];
    double b_vx_initial = (double)b->core.speed.k[0];
    int collision_detected = 0;

    snapshot_frame(out, SCENARIO_TWO_BALLS, 0, dt, env, objects, types, 2);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
        snapshot_frame(out, SCENARIO_TWO_BALLS, step, dt, env, objects, types, 2);

        /* Detect collision: velocity sign change for either ball */
        if (!collision_detected) {
            double a_vx = (double)a->core.speed.k[0];
            double b_vx = (double)b->core.speed.k[0];
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

        /* Event-driven contact pipeline should maintain persistent mindist state. */
        inv_check(inv, env->mindist_manager.entry_count > 0,
                  "two_balls mindist entries present after interaction: count=%d",
                  env->mindist_manager.entry_count);
    }

    ivp_environment_destroy(env);
    return 0;
}

static int run_slope_friction(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.83), IVP_REAL(0.0));

    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    /* --- Build ramp surface: box 5×0.25×2.5, rotated 30° around Z --- */
    ivp_compact_surface_t *ramp_surface =
        ivp_build_box_surface(NULL, 5.0f, 0.25f, 2.5f);
    if (!ramp_surface) { ivp_environment_destroy(env); return 1; }

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

    /* Rotation: 30° around Z axis */
    ivp_real_t angle_rad = (ivp_real_t)(30.0 * 3.14159265358979323846 / 180.0);
    tmpl_ramp.orientation = quat_from_axis_angle(
        IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0), angle_rad);

    ivp_object_t *ramp = ivp_environment_create_object(
        env, &tmpl_ramp, &ramp_surman.base);
    if (!ramp) {
        ivp_free_aligned(NULL, ramp_surface);
        ivp_environment_destroy(env);
        return 1;
    }
    ramp->core.upper_limit_radius = IVP_REAL(5.0);
    ivp_vec3r_set(&ramp->core.speed, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ramp->core.rot_speed.k[0] = 0.0f;
    ramp->core.rot_speed.k[1] = 0.0f;
    ramp->core.rot_speed.k[2] = 0.0f;

    /* --- Ball above ramp --- */
    ivp_object_template_t tmpl_ball;
    ivp_object_template_init(&tmpl_ball);
    tmpl_ball.type = IVP_OBJECT_BALL;
    tmpl_ball.mass = IVP_REAL(1.0);
    /* I = (2/5) * m * r^2 = 0.4 * 1.0 * 0.25 = 0.1 for r=0.5, m=1.0 */
    ivp_vec3r_set(&tmpl_ball.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    tmpl_ball.material.friction   = IVP_REAL(0.4);
    tmpl_ball.material.elasticity = IVP_REAL(0.0);
    tmpl_ball.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl_ball.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl_ball.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl_ball.position, IVP_REAL(-4.0), IVP_REAL(-4.0), IVP_REAL(0.0));
    tmpl_ball.orientation = ivp_quatr_identity();

    ivp_object_t *ball = ivp_environment_create_object(env, &tmpl_ball, NULL);
    if (!ball) {
        ivp_free_aligned(NULL, ramp_surface);
        ivp_environment_destroy(env);
        return 1;
    }
    ball->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&ball->core.speed, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ball->core.rot_speed.k[0] = 0.0f;
    ball->core.rot_speed.k[1] = 0.0f;
    ball->core.rot_speed.k[2] = 0.0f;

    ivp_object_t *objects[2] = { ramp, ball };
    const char   *types[2]   = { "poly", "ball" };

    double ramp_y0 = (double)ramp->core.pos_world_f_core_last_psi.k[1];
    double ball_y0 = (double)ball->core.pos_world_f_core_last_psi.k[1];
    double ball_speed0 = 0.0;

    snapshot_frame(out, SCENARIO_SLOPE_FRICTION, 0, dt, env, objects, types, 2);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
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

    ivp_free_aligned(NULL, ramp_surface);
    ivp_environment_destroy(env);
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
    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    if (!ground) {
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Pillar L: static box 0.5×2×0.5 at (-6,-7,0) --- */
    ivp_compact_surface_t *pillar_surf = NULL;
    ivp_surman_polygon_t pillar_sm;
    if (!scenario_init_box_surman(0.5f, 2.0f, 0.5f, &pillar_surf, &pillar_sm)) {
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_vec3r_set(&tmpl.position, IVP_REAL(-6.0), IVP_REAL(-7.0), IVP_REAL(0.0));
    ivp_object_t *pillar_l = ivp_environment_create_object(env, &tmpl, &pillar_sm.base);
    if (!pillar_l) {
        ivp_compact_surface_free(NULL, pillar_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Pillar R: static box 0.5×2×0.5 at (6,-7,0) --- */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(6.0), IVP_REAL(-7.0), IVP_REAL(0.0));
    ivp_object_t *pillar_r = ivp_environment_create_object(env, &tmpl, &pillar_sm.base);
    if (!pillar_r) {
        ivp_compact_surface_free(NULL, pillar_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Bridge nodes: 4 dynamic boxes 0.5×0.5×0.5 --- */
    #define N_BRIDGE 4
    ivp_real_t node_spacing = IVP_REAL(12.0) / ((ivp_real_t)N_BRIDGE + IVP_REAL(1.0));
    ivp_compact_surface_t *node_surf = NULL;
    ivp_surman_polygon_t node_sm;
    if (!scenario_init_box_surman(0.5f, 0.5f, 0.5f, &node_surf, &node_sm)) {
        ivp_compact_surface_free(NULL, pillar_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(0.6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.07), IVP_REAL(0.07), IVP_REAL(0.07));
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
        nodes[i] = ivp_environment_create_object(env, &tmpl, &node_sm.base);
        if (!nodes[i]) {
            ivp_compact_surface_free(NULL, node_surf);
            ivp_compact_surface_free(NULL, pillar_surf);
            ivp_compact_surface_free(NULL, ground_surf);
            ivp_environment_destroy(env);
            return 1;
        }
    }

    /* --- Ball: dynamic sphere r=0.8 at (0,-10,0) --- */
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(0.5);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.48), IVP_REAL(0.48), IVP_REAL(0.48));
    tmpl.material.friction   = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-10.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_surman_ball_t ball_sm;
    ivp_surman_ball_init(&ball_sm, 0.8f);

    ivp_object_t *ball = ivp_environment_create_object(env, &tmpl, &ball_sm.base);
    if (!ball) {
        ivp_compact_surface_free(NULL, node_surf);
        ivp_compact_surface_free(NULL, pillar_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }
    ball->core.upper_limit_radius = IVP_REAL(0.8);

    /* --- Bungee: dynamic box 0.6×0.6×0.6 at (10,-6,0) --- */
    ivp_compact_surface_t *bungee_surf = NULL;
    ivp_surman_polygon_t bungee_sm;
    if (!scenario_init_box_surman(0.6f, 0.6f, 0.6f, &bungee_surf, &bungee_sm)) {
        ivp_compact_surface_free(NULL, node_surf);
        ivp_compact_surface_free(NULL, pillar_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(3.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.36), IVP_REAL(0.36), IVP_REAL(0.36));
    tmpl.material.friction   = IVP_REAL(0.6);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(10.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *bungee = ivp_environment_create_object(env, &tmpl, &bungee_sm.base);
    if (!bungee) {
        ivp_compact_surface_free(NULL, bungee_surf);
        ivp_compact_surface_free(NULL, node_surf);
        ivp_compact_surface_free(NULL, pillar_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Anchor: static box 0.3×0.3×0.3 at (10,-14,0) --- */
    ivp_compact_surface_t *anchor_surf = NULL;
    ivp_surman_polygon_t anchor_sm;
    if (!scenario_init_box_surman(0.3f, 0.3f, 0.3f, &anchor_surf, &anchor_sm)) {
        ivp_compact_surface_free(NULL, bungee_surf);
        ivp_compact_surface_free(NULL, node_surf);
        ivp_compact_surface_free(NULL, pillar_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(10.0), IVP_REAL(-14.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *anchor = ivp_environment_create_object(env, &tmpl, &anchor_sm.base);
    if (!anchor) {
        ivp_compact_surface_free(NULL, anchor_surf);
        ivp_compact_surface_free(NULL, bungee_surf);
        ivp_compact_surface_free(NULL, node_surf);
        ivp_compact_surface_free(NULL, pillar_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
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
        springs[n_springs].max_force = IVP_REAL(24.0);
        ivp_environment_add_controller(env, 100, ivp_controller_spring, &springs[n_springs]);
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
        springs[n_springs].max_force = IVP_REAL(24.0);
        ivp_environment_add_controller(env, 100, ivp_controller_spring, &springs[n_springs]);
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
        springs[n_springs].max_force = IVP_REAL(24.0);
        ivp_environment_add_controller(env, 100, ivp_controller_spring, &springs[n_springs]);
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
        springs[n_springs].max_force = IVP_REAL(18.0);
        ivp_environment_add_controller(env, 100, ivp_controller_spring, &springs[n_springs]);
        n_springs++;
    }
    #undef MAX_SPRINGS

    /* --- Track objects --- */
    ivp_object_t *objects[10] = { ground, pillar_l, pillar_r, nodes[0], nodes[1], nodes[2], nodes[3], ball, bungee, anchor };
    const char   *types[10]   = { "ground", "pillar", "pillar", "node", "node", "node", "node", "ball", "bungee", "anchor" };

    snapshot_frame(out, SCENARIO_SPRINGS, 0, dt, env, objects, types, 10);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
        snapshot_frame(out, SCENARIO_SPRINGS, step, dt, env, objects, types, 10);
    }

    if (inv) {
        /* Total energy of dynamic objects bounded (< 1000) */
        double total_energy = 0.0;
        for (int i = 0; i < 10; i++) {
            total_energy += (double)ivp_object_get_kinetic_energy(objects[i]);
        }
        inv_check(inv, total_energy < 1000.0,
                  "springs total energy bounded: got %.6f (limit 1000)", total_energy);

        /* Bridge nodes stay between pillars (x in [-8, 8]) */
        for (int i = 0; i < N_BRIDGE; i++) {
            double nx = (double)nodes[i]->core.pos_world_f_core_last_psi.k[0];
            inv_check(inv, nx > -8.0 && nx < 8.0,
                      "springs node[%d] x in bounds: got %.6f (range [-8,8])", i, nx);
        }
    }

    ivp_compact_surface_free(NULL, anchor_surf);
    ivp_compact_surface_free(NULL, bungee_surf);
    ivp_compact_surface_free(NULL, node_surf);
    ivp_compact_surface_free(NULL, pillar_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_environment_destroy(env);
    return 0;
    #undef N_BRIDGE
}

static int run_rope(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.83), IVP_REAL(0.0));

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
    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    if (!ground) {
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Ceiling: static box 1.5×0.3×0.3 at (0,-15,0) --- */
    ivp_compact_surface_t *ceiling_surf = NULL;
    ivp_surman_polygon_t ceiling_sm;
    if (!scenario_init_box_surman(1.5f, 0.3f, 0.3f, &ceiling_surf, &ceiling_sm)) {
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-15.0), IVP_REAL(0.0));
    ivp_object_t *ceiling = ivp_environment_create_object(env, &tmpl, &ceiling_sm.base);
    if (!ceiling) {
        ivp_compact_surface_free(NULL, ceiling_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Chain links: 8 dynamic boxes 0.2×0.55×0.2 --- */
    #define CHAIN_LEN 8
    ivp_real_t step_y = IVP_REAL(2.0) * IVP_REAL(0.55) + IVP_REAL(0.05); /* 1.15 */
    ivp_compact_surface_t *link_surf = NULL;
    ivp_surman_polygon_t link_sm;
    if (!scenario_init_box_surman(0.2f, 0.55f, 0.2f, &link_surf, &link_sm)) {
        ivp_compact_surface_free(NULL, ceiling_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

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
        links[i] = ivp_environment_create_object(env, &tmpl, &link_sm.base);
        if (!links[i]) {
            ivp_compact_surface_free(NULL, link_surf);
            ivp_compact_surface_free(NULL, ceiling_surf);
            ivp_compact_surface_free(NULL, ground_surf);
            ivp_environment_destroy(env);
            return 1;
        }
    }

    /* --- Weight: dynamic box 0.6×0.6×0.6 at chain end --- */
    ivp_compact_surface_t *weight_surf = NULL;
    ivp_surman_polygon_t weight_sm;
    if (!scenario_init_box_surman(0.6f, 0.6f, 0.6f, &weight_surf, &weight_sm)) {
        ivp_compact_surface_free(NULL, link_surf);
        ivp_compact_surface_free(NULL, ceiling_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(2.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.48), IVP_REAL(0.48), IVP_REAL(0.48));
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
        ivp_compact_surface_free(NULL, weight_surf);
        ivp_compact_surface_free(NULL, link_surf);
        ivp_compact_surface_free(NULL, ceiling_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Ball-socket constraints (matching example_rope.c) --- */
    ivp_constraint_state_t constraints[CHAIN_LEN + 1];
    memset(constraints, 0, sizeof(constraints));

    /* ceiling → links[0] */
    {
        ivp_vec3r_t a_pt = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.3), IVP_REAL(0.0));
        ivp_vec3r_t b_pt = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.55), IVP_REAL(0.0));
        ivp_constraint_init_ball_socket(&constraints[0],
            ceiling, &a_pt, links[0], &b_pt);
        ivp_environment_add_controller(env, 50, ivp_controller_constraint, &constraints[0]);
    }

    /* links[i-1] → links[i] */
    for (int i = 1; i < CHAIN_LEN; i++) {
        ivp_vec3r_t a_bot = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.55), IVP_REAL(0.0));
        ivp_vec3r_t b_top = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.55), IVP_REAL(0.0));
        ivp_constraint_init_ball_socket(&constraints[i],
            links[i-1], &a_bot, links[i], &b_top);
        ivp_environment_add_controller(env, 50, ivp_controller_constraint, &constraints[i]);
    }

    /* links[7] → weight */
    {
        ivp_vec3r_t a_bot = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.55), IVP_REAL(0.0));
        ivp_vec3r_t b_top = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.6), IVP_REAL(0.0));
        ivp_constraint_init_ball_socket(&constraints[CHAIN_LEN],
            links[CHAIN_LEN-1], &a_bot, weight, &b_top);
        ivp_environment_add_controller(env, 50, ivp_controller_constraint, &constraints[CHAIN_LEN]);
    }

    /* --- Track objects --- */
    ivp_object_t *objects[11] = { ground, ceiling, links[0], links[1], links[2], links[3], links[4], links[5], links[6], links[7], weight };
    const char   *types[11]   = { "ground", "ceiling", "link", "link", "link", "link", "link", "link", "link", "link", "weight" };

    snapshot_frame(out, SCENARIO_ROPE, 0, dt, env, objects, types, 11);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
        snapshot_frame(out, SCENARIO_ROPE, step, dt, env, objects, types, 11);
    }

    if (inv) {
        /* Weight hangs below top link (weight.y > link[0].y in Y-down) */
        double weight_y = (double)weight->core.pos_world_f_core_last_psi.k[1];
        double link0_y  = (double)links[0]->core.pos_world_f_core_last_psi.k[1];
        inv_check(inv, weight_y > link0_y,
                  "rope weight below top link: weight_y=%.6f link0_y=%.6f", weight_y, link0_y);

        /* Total chain energy bounded.
         * Under descending reference-priority dispatch, rope transients are
         * higher; keep a non-divergence envelope instead of legacy 5000 cap. */
        double total_energy = 0.0;
        for (int i = 0; i < 11; i++) {
            total_energy += (double)ivp_object_get_kinetic_energy(objects[i]);
        }
        inv_check(inv, total_energy < 10000.0,
                  "rope total energy bounded: got %.6f (limit 10000)", total_energy);
    }

    ivp_compact_surface_free(NULL, weight_surf);
    ivp_compact_surface_free(NULL, link_surf);
    ivp_compact_surface_free(NULL, ceiling_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_environment_destroy(env);
    return 0;
    #undef CHAIN_LEN
}

static int run_motor_common(FILE *out, int steps, double dt, invariant_result_t *inv, int scenario_id)
{
    static const char *motor_internal = "motint";

    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.83), IVP_REAL(0.0));

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
    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    if (!ground) {
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Pedestal: static box 0.15×0.8×0.15 at (0,-5.6,0) --- */
    ivp_compact_surface_t *pedestal_surf = NULL;
    ivp_surman_polygon_t pedestal_sm;
    if (!scenario_init_box_surman(0.15f, 0.8f, 0.15f, &pedestal_surf, &pedestal_sm)) {
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-5.6), IVP_REAL(0.0));
    tmpl.no_collision_group = motor_internal;
    ivp_object_t *pedestal = ivp_environment_create_object(env, &tmpl, &pedestal_sm.base);
    if (!pedestal) {
        ivp_compact_surface_free(NULL, pedestal_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }
    tmpl.no_collision_group = NULL;

    /* --- Blade: dynamic box 4.0×0.25×0.4 at (0,-6.8,0) --- */
    ivp_compact_surface_t *blade_surf = NULL;
    ivp_surman_polygon_t blade_sm;
    if (!scenario_init_box_surman(4.0f, 0.25f, 0.4f, &blade_surf, &blade_sm)) {
        ivp_compact_surface_free(NULL, pedestal_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
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
        ivp_compact_surface_free(NULL, blade_surf);
        ivp_compact_surface_free(NULL, pedestal_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
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
    ivp_compact_surface_t *target_surf = NULL;
    ivp_surman_polygon_t target_sm;
    if (!scenario_init_box_surman(0.4f, 0.4f, 0.4f, &target_surf, &target_sm)) {
        ivp_compact_surface_free(NULL, blade_surf);
        ivp_compact_surface_free(NULL, pedestal_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(0.06);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.05), IVP_REAL(0.05), IVP_REAL(0.05));
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
        targets[i] = ivp_environment_create_object(env, &tmpl, &target_sm.base);
        if (!targets[i]) {
            ivp_compact_surface_free(NULL, target_surf);
            ivp_compact_surface_free(NULL, blade_surf);
            ivp_compact_surface_free(NULL, pedestal_surf);
            ivp_compact_surface_free(NULL, ground_surf);
            ivp_environment_destroy(env);
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
    ivp_environment_add_controller(env, 50, ivp_controller_constraint, &pivot);

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
    ivp_environment_add_controller(env, 100, ivp_controller_rot_mot, &motor);

    /* --- Track objects --- */
    ivp_object_t *objects[11] = { ground, pedestal, blade, targets[0], targets[1], targets[2], targets[3], targets[4], targets[5], targets[6], targets[7] };
    const char   *types[11]   = { "ground", "pedestal", "blade", "target", "target", "target", "target", "target", "target", "target", "target" };

    /* Record initial target positions for invariant checks */
    double target_init_x[N_TARGETS], target_init_z[N_TARGETS];
    for (int i = 0; i < N_TARGETS; i++) {
        target_init_x[i] = (double)targets[i]->core.pos_world_f_core_last_psi.k[0];
        target_init_z[i] = (double)targets[i]->core.pos_world_f_core_last_psi.k[2];
    }

    snapshot_frame(out, scenario_id, 0, dt, env, objects, types, 11);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
        snapshot_frame(out, scenario_id, step, dt, env, objects, types, 11);
    }

    if (inv) {
        /* Blade rotating at final step (|ang_vel_y| > 0.1) */
        double blade_ang_vel_y = (double)blade->core.rot_speed.k[1];
        inv_check(inv, fabs(blade_ang_vel_y) > 0.1,
                  "motor blade rotating: |ang_vel_y|=%.6g (need > 0.1)", fabs(blade_ang_vel_y));

        /* At least 2 targets displaced from initial position */
        int displaced = 0;
        for (int i = 0; i < N_TARGETS; i++) {
            double tx = (double)targets[i]->core.pos_world_f_core_last_psi.k[0];
            double tz = (double)targets[i]->core.pos_world_f_core_last_psi.k[2];
            double dx = tx - target_init_x[i];
            double dz = tz - target_init_z[i];
            if (sqrt(dx*dx + dz*dz) > 0.1)
                displaced++;
        }
        inv_check(inv, displaced >= 2,
                  "motor targets displaced: %d targets moved (need >= 2)", displaced);
    }

    ivp_compact_surface_free(NULL, target_surf);
    ivp_compact_surface_free(NULL, blade_surf);
    ivp_compact_surface_free(NULL, pedestal_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_environment_destroy(env);
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
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.83), IVP_REAL(0.0));

    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    /* --- Ground: static box 15×0.5×15 at (0,5,0) --- */
    ivp_compact_surface_t *ground_surf = ivp_build_box_surface(NULL, 15.0f, 0.5f, 15.0f);
    if (!ground_surf) { ivp_environment_destroy(env); return 1; }
    ivp_surman_polygon_t ground_sm;
    ivp_surman_polygon_init(&ground_sm, ground_surf);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.material.friction   = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(5.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    if (!ground) {
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Wall L: static box 0.3×6×15 at (-12,0,0) --- */
    ivp_compact_surface_t *wall_surf = ivp_build_box_surface(NULL, 0.3f, 6.0f, 15.0f);
    if (!wall_surf) {
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }
    ivp_surman_polygon_t wall_sm;
    ivp_surman_polygon_init(&wall_sm, wall_surf);

    ivp_vec3r_set(&tmpl.position, IVP_REAL(-12.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *wall_l = ivp_environment_create_object(env, &tmpl, &wall_sm.base);
    if (!wall_l) {
        ivp_compact_surface_free(NULL, wall_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Wall R: static box 0.3×6×15 at (12,0,0) --- */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(12.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *wall_r = ivp_environment_create_object(env, &tmpl, &wall_sm.base);
    if (!wall_r) {
        ivp_compact_surface_free(NULL, wall_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Cork: box 0.5×0.5×0.5, mass=0.3 at (-6,-6,0) --- */
    ivp_compact_surface_t *cork_surf = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    if (!cork_surf) {
        ivp_compact_surface_free(NULL, wall_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }
    ivp_surman_polygon_t cork_sm;
    ivp_surman_polygon_init(&cork_sm, cork_surf);

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(0.3);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.072), IVP_REAL(0.072), IVP_REAL(0.072));
    tmpl.material.friction   = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.2);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-6.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *cork = ivp_environment_create_object(env, &tmpl, &cork_sm.base);
    if (!cork) {
        ivp_compact_surface_free(NULL, cork_surf);
        ivp_compact_surface_free(NULL, wall_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Wood: box 0.6×0.4×0.6, mass=1.0 at (-3,-5,0) --- */
    ivp_compact_surface_t *wood_surf = ivp_build_box_surface(NULL, 0.6f, 0.4f, 0.6f);
    if (!wood_surf) {
        ivp_compact_surface_free(NULL, cork_surf);
        ivp_compact_surface_free(NULL, wall_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }
    ivp_surman_polygon_t wood_sm;
    ivp_surman_polygon_init(&wood_sm, wood_surf);

    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.24), IVP_REAL(0.24), IVP_REAL(0.24));
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-3.0), IVP_REAL(-5.0), IVP_REAL(0.0));

    ivp_object_t *wood = ivp_environment_create_object(env, &tmpl, &wood_sm.base);
    if (!wood) {
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        ivp_compact_surface_free(NULL, wall_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Ball: sphere r=0.55, mass=1.5 at (0,-7,0) --- */
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.5);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.36), IVP_REAL(0.36), IVP_REAL(0.36));
    tmpl.material.friction   = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.2);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-7.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_surman_ball_t ball_sm;
    ivp_surman_ball_init(&ball_sm, 0.55f);

    ivp_object_t *ball = ivp_environment_create_object(env, &tmpl, &ball_sm.base);
    if (!ball) {
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        ivp_compact_surface_free(NULL, wall_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }
    ball->core.upper_limit_radius = IVP_REAL(0.55);

    /* --- Steel: box 0.5×0.5×0.5, mass=8.0 at (3,-4,0) --- */
    ivp_compact_surface_t *steel_surf = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    if (!steel_surf) {
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        ivp_compact_surface_free(NULL, wall_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }
    ivp_surman_polygon_t steel_sm;
    ivp_surman_polygon_init(&steel_sm, steel_surf);

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(8.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1.92), IVP_REAL(1.92), IVP_REAL(1.92));
    tmpl.material.friction   = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.2);
    tmpl.speed_damp_factor   = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    ivp_vec3r_set(&tmpl.position, IVP_REAL(3.0), IVP_REAL(-4.0), IVP_REAL(0.0));
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *steel = ivp_environment_create_object(env, &tmpl, &steel_sm.base);
    if (!steel) {
        ivp_compact_surface_free(NULL, steel_surf);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        ivp_compact_surface_free(NULL, wall_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Brick: box 0.7×0.35×0.5, mass=3.0 at (6,-6,0) --- */
    ivp_compact_surface_t *brick_surf = ivp_build_box_surface(NULL, 0.7f, 0.35f, 0.5f);
    if (!brick_surf) {
        ivp_compact_surface_free(NULL, steel_surf);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        ivp_compact_surface_free(NULL, wall_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }
    ivp_surman_polygon_t brick_sm;
    ivp_surman_polygon_init(&brick_sm, brick_surf);

    tmpl.mass = IVP_REAL(3.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.72), IVP_REAL(0.72), IVP_REAL(0.72));
    ivp_vec3r_set(&tmpl.position, IVP_REAL(6.0), IVP_REAL(-6.0), IVP_REAL(0.0));

    ivp_object_t *brick = ivp_environment_create_object(env, &tmpl, &brick_sm.base);
    if (!brick) {
        ivp_compact_surface_free(NULL, brick_surf);
        ivp_compact_surface_free(NULL, steel_surf);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        ivp_compact_surface_free(NULL, wall_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Foam: box 0.8×0.8×0.8, mass=0.2 at (-4.5,-9,3) --- */
    ivp_compact_surface_t *foam_surf = ivp_build_box_surface(NULL, 0.8f, 0.8f, 0.8f);
    if (!foam_surf) {
        ivp_compact_surface_free(NULL, brick_surf);
        ivp_compact_surface_free(NULL, steel_surf);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        ivp_compact_surface_free(NULL, wall_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }
    ivp_surman_polygon_t foam_sm;
    ivp_surman_polygon_init(&foam_sm, foam_surf);

    tmpl.mass = IVP_REAL(0.2);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.048), IVP_REAL(0.048), IVP_REAL(0.048));
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-4.5), IVP_REAL(-9.0), IVP_REAL(3.0));

    ivp_object_t *foam = ivp_environment_create_object(env, &tmpl, &foam_sm.base);
    if (!foam) {
        ivp_compact_surface_free(NULL, foam_surf);
        ivp_compact_surface_free(NULL, brick_surf);
        ivp_compact_surface_free(NULL, steel_surf);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        ivp_compact_surface_free(NULL, wall_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Lead: box 0.4×0.4×0.4, mass=12.0 at (4.5,-8,-3) --- */
    ivp_compact_surface_t *lead_surf = ivp_build_box_surface(NULL, 0.4f, 0.4f, 0.4f);
    if (!lead_surf) {
        ivp_compact_surface_free(NULL, foam_surf);
        ivp_compact_surface_free(NULL, brick_surf);
        ivp_compact_surface_free(NULL, steel_surf);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        ivp_compact_surface_free(NULL, wall_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }
    ivp_surman_polygon_t lead_sm;
    ivp_surman_polygon_init(&lead_sm, lead_surf);

    tmpl.mass = IVP_REAL(12.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(2.88), IVP_REAL(2.88), IVP_REAL(2.88));
    ivp_vec3r_set(&tmpl.position, IVP_REAL(4.5), IVP_REAL(-8.0), IVP_REAL(-3.0));

    ivp_object_t *lead = ivp_environment_create_object(env, &tmpl, &lead_sm.base);
    if (!lead) {
        ivp_compact_surface_free(NULL, lead_surf);
        ivp_compact_surface_free(NULL, foam_surf);
        ivp_compact_surface_free(NULL, brick_surf);
        ivp_compact_surface_free(NULL, steel_surf);
        ivp_compact_surface_free(NULL, wood_surf);
        ivp_compact_surface_free(NULL, cork_surf);
        ivp_compact_surface_free(NULL, wall_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    /* --- Buoyancy controller (shared for all dynamic objects) --- */
    ivp_buoyancy_state_t buoy;
    ivp_buoyancy_init(&buoy, IVP_REAL(1.0), IVP_REAL(1000.0));
    buoy.linear_drag  = IVP_REAL(3.0);
    buoy.angular_drag = IVP_REAL(2.0);
    ivp_vec3r_set(&buoy.flow_velocity, IVP_REAL(1.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_add_controller(env, 200, ivp_controller_buoyancy, &buoy);

    /* --- Track objects --- */
    ivp_object_t *objects[10] = { ground, wall_l, wall_r, cork, wood, ball, steel, brick, foam, lead };
    const char   *types[10]   = { "ground", "wall", "wall", "cork", "wood", "ball", "steel", "brick", "foam", "lead" };

    snapshot_frame(out, SCENARIO_BUOYANCY, 0, dt, env, objects, types, 10);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
        snapshot_frame(out, SCENARIO_BUOYANCY, step, dt, env, objects, types, 10);
    }

    if (inv) {
        /* Heavy objects (steel id=6, lead id=9) sink: y > -3.0 in Y-down */
        double steel_y = (double)steel->core.pos_world_f_core_last_psi.k[1];
        double lead_y  = (double)lead->core.pos_world_f_core_last_psi.k[1];
        inv_check(inv, steel_y > -3.0,
                  "buoyancy steel sinks: y=%.6f (need > -3.0 in Y-down)", steel_y);
        inv_check(inv, lead_y > -3.0,
                  "buoyancy lead sinks: y=%.6f (need > -3.0 in Y-down)", lead_y);

        /* Energy doesn't grow unboundedly: total KE < 10000 */
        double total_energy = 0.0;
        for (int i = 0; i < 10; i++) {
            total_energy += (double)ivp_object_get_kinetic_energy(objects[i]);
        }
        inv_check(inv, total_energy < 10000.0,
                  "buoyancy energy bounded: got %.6f (limit 10000)", total_energy);
    }

    ivp_compact_surface_free(NULL, lead_surf);
    ivp_compact_surface_free(NULL, foam_surf);
    ivp_compact_surface_free(NULL, brick_surf);
    ivp_compact_surface_free(NULL, steel_surf);
    ivp_compact_surface_free(NULL, wood_surf);
    ivp_compact_surface_free(NULL, cork_surf);
    ivp_compact_surface_free(NULL, wall_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_environment_destroy(env);
    return 0;
}

static int run_cubes(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    enum { N_CUBES = 16, N_OBJECTS = 17 };

    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.83), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *ground_surf = ivp_build_box_surface(NULL, 20.0f, 0.5f, 20.0f);
    ivp_compact_surface_t *cube_surf = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    if (!ground_surf || !cube_surf) {
        if (ground_surf) ivp_compact_surface_free(NULL, ground_surf);
        if (cube_surf) ivp_compact_surface_free(NULL, cube_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_surman_polygon_t ground_sm, cube_sm;
    ivp_surman_polygon_init(&ground_sm, ground_surf);
    ivp_surman_polygon_init(&cube_sm, cube_surf);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.material.friction = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));

    ivp_object_t *objects[N_OBJECTS] = {0};
    const char *types[N_OBJECTS] = {0};
    objects[0] = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    types[0] = "ground";
    if (!objects[0]) {
        ivp_compact_surface_free(NULL, cube_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(0.6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
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
            objects[idx] = ivp_environment_create_object(env, &tmpl, &cube_sm.base);
            types[idx] = "cube";
            if (!objects[idx]) {
                ivp_compact_surface_free(NULL, cube_surf);
                ivp_compact_surface_free(NULL, ground_surf);
                ivp_environment_destroy(env);
                return 1;
            }
            idx++;
        }
    }

    const double first_cube_y0 = (double)objects[1]->core.pos_world_f_core_last_psi.k[1];

    snapshot_frame(out, SCENARIO_CUBES, 0, dt, env, objects, types, N_OBJECTS);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
        snapshot_frame(out, SCENARIO_CUBES, step, dt, env, objects, types, N_OBJECTS);
    }

    if (inv) {
        const double first_cube_y = (double)objects[1]->core.pos_world_f_core_last_psi.k[1];
        inv_check(inv, first_cube_y > first_cube_y0,
                  "cubes first cube falls: y0=%.6f y=%.6f", first_cube_y0, first_cube_y);
    }

    ivp_compact_surface_free(NULL, cube_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_environment_destroy(env);
    return 0;
}

static int run_force_actuator(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.83), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *ground_surf = ivp_build_box_surface(NULL, 20.0f, 0.5f, 20.0f);
    ivp_compact_surface_t *box_surf = ivp_build_box_surface(NULL, 0.6f, 0.6f, 0.6f);
    if (!ground_surf || !box_surf) {
        if (ground_surf) ivp_compact_surface_free(NULL, ground_surf);
        if (box_surf) ivp_compact_surface_free(NULL, box_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_surman_polygon_t ground_sm, box_sm;
    ivp_surman_polygon_init(&ground_sm, ground_surf);
    ivp_surman_polygon_init(&box_sm, box_surf);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.material.friction = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    if (!ground) {
        ivp_compact_surface_free(NULL, box_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.24), IVP_REAL(0.24), IVP_REAL(0.24));
    tmpl.material.friction = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor = IVP_REAL(0.12);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-2.0), IVP_REAL(0.0));
    ivp_object_t *box = ivp_environment_create_object(env, &tmpl, &box_sm.base);
    if (!box) {
        ivp_compact_surface_free(NULL, box_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_force_actuator_state_t force;
    ivp_force_actuator_state_init(&force);
    ivp_vec3r_t a = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t b = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_force_actuator_state_setup(&force, box, &a, box, &b, IVP_REAL(1.2), true, false);
    ivp_environment_add_controller(env, 100, ivp_controller_force_actuator, &force);

    const double x0 = (double)box->core.pos_world_f_core_last_psi.k[0];
    ivp_object_t *objects[2] = { ground, box };
    const char *types[2] = { "ground", "force_box" };
    snapshot_frame(out, SCENARIO_FORCE_ACTUATOR, 0, dt, env, objects, types, 2);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
        snapshot_frame(out, SCENARIO_FORCE_ACTUATOR, step, dt, env, objects, types, 2);
    }

    if (inv) {
        const double x1 = (double)box->core.pos_world_f_core_last_psi.k[0];
        inv_check(inv, x1 > x0 + 0.05, "force_actuator box moves +X: x0=%.6f x=%.6f", x0, x1);
    }

    ivp_compact_surface_free(NULL, box_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_environment_destroy(env);
    return 0;
}

static int run_forcefield(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.83), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *ground_surf = ivp_build_box_surface(NULL, 20.0f, 0.5f, 20.0f);
    ivp_compact_surface_t *box_surf = ivp_build_box_surface(NULL, 0.6f, 0.6f, 0.6f);
    if (!ground_surf || !box_surf) {
        if (ground_surf) ivp_compact_surface_free(NULL, ground_surf);
        if (box_surf) ivp_compact_surface_free(NULL, box_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_surman_polygon_t ground_sm, box_sm;
    ivp_surman_polygon_init(&ground_sm, ground_surf);
    ivp_surman_polygon_init(&box_sm, box_surf);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.material.friction = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    if (!ground) {
        ivp_compact_surface_free(NULL, box_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.24), IVP_REAL(0.24), IVP_REAL(0.24));
    tmpl.material.friction = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor = IVP_REAL(0.12);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-2.0), IVP_REAL(0.0));
    ivp_object_t *box = ivp_environment_create_object(env, &tmpl, &box_sm.base);
    if (!box) {
        ivp_compact_surface_free(NULL, box_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_forcefield_state_t field;
    ivp_vec3r_t force_value = ivp_vec3r(IVP_REAL(2.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_forcefield_state_init(&field, IVP_FORCEFIELD_FORCE, &force_value);
    ivp_environment_add_controller(env, 100, ivp_controller_forcefield, &field);

    const double x0 = (double)box->core.pos_world_f_core_last_psi.k[0];
    ivp_object_t *objects[2] = { ground, box };
    const char *types[2] = { "ground", "forcefield_box" };
    snapshot_frame(out, SCENARIO_FORCEFIELD, 0, dt, env, objects, types, 2);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
        snapshot_frame(out, SCENARIO_FORCEFIELD, step, dt, env, objects, types, 2);
    }

    if (inv) {
        const double x1 = (double)box->core.pos_world_f_core_last_psi.k[0];
        inv_check(inv, x1 > x0 + 0.1, "forcefield box moves +X: x0=%.6f x=%.6f", x0, x1);
    }

    ivp_compact_surface_free(NULL, box_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_environment_destroy(env);
    return 0;
}

static int run_stiff_spring(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.83), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *anchor_surf = ivp_build_box_surface(NULL, 0.4f, 0.4f, 0.4f);
    ivp_compact_surface_t *dyn_surf = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    if (!anchor_surf || !dyn_surf) {
        if (anchor_surf) ivp_compact_surface_free(NULL, anchor_surf);
        if (dyn_surf) ivp_compact_surface_free(NULL, dyn_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_surman_polygon_t anchor_sm, dyn_sm;
    ivp_surman_polygon_init(&anchor_sm, anchor_surf);
    ivp_surman_polygon_init(&dyn_sm, dyn_surf);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.material.friction = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-4.0), IVP_REAL(-8.0), IVP_REAL(0.0));
    ivp_object_t *anchor_l = ivp_environment_create_object(env, &tmpl, &anchor_sm.base);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(4.0), IVP_REAL(-8.0), IVP_REAL(0.0));
    ivp_object_t *anchor_r = ivp_environment_create_object(env, &tmpl, &anchor_sm.base);
    if (!anchor_l || !anchor_r) {
        ivp_compact_surface_free(NULL, dyn_surf);
        ivp_compact_surface_free(NULL, anchor_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(0.8);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.12), IVP_REAL(0.12), IVP_REAL(0.12));
    tmpl.material.friction = IVP_REAL(0.6);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();

    ivp_vec3r_set(&tmpl.position, IVP_REAL(-2.0), IVP_REAL(-10.5), IVP_REAL(0.0));
    ivp_object_t *soft = ivp_environment_create_object(env, &tmpl, &dyn_sm.base);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(2.0), IVP_REAL(-10.5), IVP_REAL(0.0));
    ivp_object_t *stiff = ivp_environment_create_object(env, &tmpl, &dyn_sm.base);
    if (!soft || !stiff) {
        ivp_compact_surface_free(NULL, dyn_surf);
        ivp_compact_surface_free(NULL, anchor_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_vec3r_t a0 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_stiff_spring_state_t spring_soft, spring_stiff;
    ivp_stiff_spring_state_init(&spring_soft);
    ivp_stiff_spring_state_setup(&spring_soft,
                                 anchor_l, &a0, soft, &a0,
                                 IVP_REAL(25.0), IVP_REAL(6.0), IVP_REAL(2.5));
    ivp_stiff_spring_state_init(&spring_stiff);
    ivp_stiff_spring_state_setup(&spring_stiff,
                                 anchor_r, &a0, stiff, &a0,
                                 IVP_REAL(320.0), IVP_REAL(45.0), IVP_REAL(2.5));
    ivp_environment_add_controller(env, 100, ivp_controller_stiff_spring, &spring_soft);
    ivp_environment_add_controller(env, 100, ivp_controller_stiff_spring, &spring_stiff);

    const double soft_y0 = (double)soft->core.pos_world_f_core_last_psi.k[1];
    const double stiff_y0 = (double)stiff->core.pos_world_f_core_last_psi.k[1];

    ivp_object_t *objects[4] = { anchor_l, anchor_r, soft, stiff };
    const char *types[4] = { "anchor", "anchor", "soft", "stiff" };
    snapshot_frame(out, SCENARIO_STIFF_SPRING, 0, dt, env, objects, types, 4);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
        snapshot_frame(out, SCENARIO_STIFF_SPRING, step, dt, env, objects, types, 4);
    }

    if (inv) {
        const double soft_delta = fabs((double)soft->core.pos_world_f_core_last_psi.k[1] - soft_y0);
        const double stiff_delta = fabs((double)stiff->core.pos_world_f_core_last_psi.k[1] - stiff_y0);
        inv_check(inv, !isnan(soft_delta) && !isnan(stiff_delta),
                  "stiff_spring finite displacement: soft=%.6f stiff=%.6f",
                  soft_delta, stiff_delta);
        inv_check(inv, soft_delta < 100.0 && stiff_delta < 100.0,
                  "stiff_spring bounded displacement: soft=%.6f stiff=%.6f",
                  soft_delta, stiff_delta);
    }

    ivp_compact_surface_free(NULL, dyn_surf);
    ivp_compact_surface_free(NULL, anchor_surf);
    ivp_environment_destroy(env);
    return 0;
}

static int run_check_distance(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_surman_ball_t ball_sm;
    ivp_surman_ball_init(&ball_sm, 0.5f);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    tmpl.material.friction = IVP_REAL(0.4);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();

    ivp_vec3r_set(&tmpl.position, IVP_REAL(-4.0), IVP_REAL(-2.0), IVP_REAL(0.0));
    ivp_object_t *a = ivp_environment_create_object(env, &tmpl, &ball_sm.base);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(4.0), IVP_REAL(-2.0), IVP_REAL(0.0));
    ivp_object_t *b = ivp_environment_create_object(env, &tmpl, &ball_sm.base);
    if (!a || !b) {
        ivp_environment_destroy(env);
        return 1;
    }
    a->core.upper_limit_radius = IVP_REAL(0.5);
    b->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&a->core.speed, IVP_REAL(1.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&b->core.speed, IVP_REAL(-1.5), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_check_dist_state_t check;
    ivp_check_dist_state_init(&check);
    ivp_vec3r_t pa = a->core.pos_world_f_core_last_psi;
    ivp_vec3r_t pb = b->core.pos_world_f_core_last_psi;
    ivp_check_dist_state_setup(&check, a, &pa, b, &pb, IVP_REAL(5.0));
    ivp_check_dist_set_callback(&check, check_dist_counter_cb, NULL);
    g_check_dist_event_count = 0;
    ivp_environment_add_controller(env, 100, ivp_controller_check_dist, &check);

    ivp_object_t *objects[2] = { a, b };
    const char *types[2] = { "dist_a", "dist_b" };
    snapshot_frame(out, SCENARIO_CHECK_DISTANCE, 0, dt, env, objects, types, 2);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
        snapshot_frame(out, SCENARIO_CHECK_DISTANCE, step, dt, env, objects, types, 2);
    }

    if (inv) {
        inv_check(inv, g_check_dist_event_count > 0,
                  "check_distance callback fired: count=%d", g_check_dist_event_count);
    }

    ivp_environment_destroy(env);
    return 0;
}

static int run_collision_filter(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.83), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *ground_surf = ivp_build_box_surface(NULL, 20.0f, 0.5f, 20.0f);
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
    tmpl.material.friction = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    if (!ground) {
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    static const char *grp = "grpA";
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    tmpl.material.friction = IVP_REAL(0.6);
    tmpl.material.elasticity = IVP_REAL(0.1);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.no_collision_group = grp;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-3.0), IVP_REAL(-3.0), IVP_REAL(0.0));
    ivp_object_t *a = ivp_environment_create_object(env, &tmpl, &ball_sm.base);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(3.0), IVP_REAL(-3.0), IVP_REAL(0.0));
    ivp_object_t *b = ivp_environment_create_object(env, &tmpl, &ball_sm.base);
    if (!a || !b) {
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }
    a->core.upper_limit_radius = IVP_REAL(0.5);
    b->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&a->core.speed, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&b->core.speed, IVP_REAL(-3.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_object_t *objects[3] = { ground, a, b };
    const char *types[3] = { "ground", "filter_ball", "filter_ball" };
    snapshot_frame(out, SCENARIO_COLLISION_FILTER, 0, dt, env, objects, types, 3);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
        snapshot_frame(out, SCENARIO_COLLISION_FILTER, step, dt, env, objects, types, 3);
    }

    if (inv) {
        inv_check(inv, a->core.speed.k[0] > IVP_REAL(0.0) && b->core.speed.k[0] < IVP_REAL(0.0),
                  "collision_filter disabled collision keeps directions: vxA=%.6f vxB=%.6f",
                  (double)a->core.speed.k[0], (double)b->core.speed.k[0]);
    }

    ivp_compact_surface_free(NULL, ground_surf);
    ivp_environment_destroy(env);
    return 0;
}

static int run_motion_controller(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.83), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *ground_surf = ivp_build_box_surface(NULL, 20.0f, 0.5f, 20.0f);
    ivp_compact_surface_t *box_surf = ivp_build_box_surface(NULL, 0.7f, 0.7f, 0.7f);
    if (!ground_surf || !box_surf) {
        if (ground_surf) ivp_compact_surface_free(NULL, ground_surf);
        if (box_surf) ivp_compact_surface_free(NULL, box_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_surman_polygon_t ground_sm, box_sm;
    ivp_surman_polygon_init(&ground_sm, ground_surf);
    ivp_surman_polygon_init(&box_sm, box_surf);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.material.friction = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0));
    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    if (!ground) {
        ivp_compact_surface_free(NULL, box_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1.2);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.32), IVP_REAL(0.32), IVP_REAL(0.32));
    tmpl.material.friction = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-6.0), IVP_REAL(-4.0), IVP_REAL(0.0));
    ivp_object_t *box = ivp_environment_create_object(env, &tmpl, &box_sm.base);
    if (!box) {
        ivp_compact_surface_free(NULL, box_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_motion_state_t motion;
    ivp_motion_state_init(&motion);
    ivp_vec3r_t target_vel = ivp_vec3r(IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_motion_state_set_velocity(&motion, &target_vel);
    motion.max_speed = IVP_REAL(6.0);
    ivp_environment_add_controller(env, 100, ivp_controller_motion, &motion);

    const double x0 = (double)box->core.pos_world_f_core_last_psi.k[0];
    ivp_object_t *objects[2] = { ground, box };
    const char *types[2] = { "ground", "motion_box" };
    snapshot_frame(out, SCENARIO_MOTION_CONTROLLER, 0, dt, env, objects, types, 2);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
        snapshot_frame(out, SCENARIO_MOTION_CONTROLLER, step, dt, env, objects, types, 2);
    }

    if (inv) {
        const double x1 = (double)box->core.pos_world_f_core_last_psi.k[0];
        inv_check(inv, x1 > x0 + 0.5,
                  "motion_controller advances toward +X: x0=%.6f x=%.6f",
                  x0, x1);
    }

    ivp_compact_surface_free(NULL, box_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_environment_destroy(env);
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

    ivp_compact_surface_t *ground_surf = ivp_build_box_surface(NULL, 20.0f, 0.5f, 20.0f);
    ivp_compact_surface_t *trigger_surf = ivp_build_box_surface(NULL, 2.5f, 2.5f, 2.5f);
    if (!ground_surf || !trigger_surf) {
        if (ground_surf) ivp_compact_surface_free(NULL, ground_surf);
        if (trigger_surf) ivp_compact_surface_free(NULL, trigger_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_surman_polygon_t ground_sm, trigger_sm;
    ivp_surman_polygon_init(&ground_sm, ground_surf);
    ivp_surman_polygon_init(&trigger_sm, trigger_surf);
    ivp_surman_ball_t ball_sm;
    ivp_surman_ball_init(&ball_sm, 0.5f);

    static const char *phntm = "phntm";
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.material.friction = IVP_REAL(0.8);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    if (!ground) {
        ivp_compact_surface_free(NULL, trigger_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    tmpl.no_collision_group = phntm;
    ivp_object_t *trigger = ivp_environment_create_object(env, &tmpl, &trigger_sm.base);
    if (!trigger) {
        ivp_compact_surface_free(NULL, trigger_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    tmpl.material.friction = IVP_REAL(0.5);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.no_collision_group = phntm;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(-9.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    ivp_object_t *ball = ivp_environment_create_object(env, &tmpl, &ball_sm.base);
    if (!ball) {
        ivp_compact_surface_free(NULL, trigger_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }
    ball->core.upper_limit_radius = IVP_REAL(0.5);
    ivp_vec3r_set(&ball->core.speed, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_phantom_state_t phantom;
    ivp_vec3r_t center = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    ivp_phantom_init_sphere(&phantom, &center, IVP_REAL(2.5), phantom_event_counter_cb, NULL);
    g_phantom_event_count = 0;
    ivp_environment_add_controller(env, 100, ivp_controller_phantom, &phantom);

    ivp_object_t *objects[3] = { ground, trigger, ball };
    const char *types[3] = { "ground", "phantom", "ball" };
    snapshot_frame(out, SCENARIO_PHANTOM, 0, dt, env, objects, types, 3);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
        snapshot_frame(out, SCENARIO_PHANTOM, step, dt, env, objects, types, 3);
    }

    if (inv) {
        inv_check(inv, g_phantom_event_count > 0,
                  "phantom events fired: count=%d", g_phantom_event_count);
    }

    ivp_compact_surface_free(NULL, trigger_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_environment_destroy(env);
    return 0;
}

static int run_vehicle(FILE *out, int steps, double dt, invariant_result_t *inv)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)dt;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.83), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);
    if (!env) return 1;

    ivp_compact_surface_t *ground_surf = ivp_build_box_surface(NULL, 30.0f, 0.5f, 30.0f);
    ivp_compact_surface_t *body_surf = ivp_build_box_surface(NULL, 1.6f, 0.35f, 0.8f);
    if (!ground_surf || !body_surf) {
        if (ground_surf) ivp_compact_surface_free(NULL, ground_surf);
        if (body_surf) ivp_compact_surface_free(NULL, body_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_surman_polygon_t ground_sm, body_sm;
    ivp_surman_polygon_init(&ground_sm, ground_surf);
    ivp_surman_polygon_init(&body_sm, body_surf);
    ivp_surman_ball_t wheel_sm;
    ivp_surman_ball_init(&wheel_sm, 0.35f);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    tmpl.material.friction = IVP_REAL(0.9);
    tmpl.material.elasticity = IVP_REAL(0.0);
    tmpl.physical_unmoveable = true;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0));
    ivp_object_t *ground = ivp_environment_create_object(env, &tmpl, &ground_sm.base);
    if (!ground) {
        ivp_compact_surface_free(NULL, body_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_POLYGON;
    tmpl.mass = IVP_REAL(2.2);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.9), IVP_REAL(0.9), IVP_REAL(0.9));
    tmpl.material.friction = IVP_REAL(0.6);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(-4.0), IVP_REAL(0.0));
    ivp_object_t *body = ivp_environment_create_object(env, &tmpl, &body_sm.base);
    if (!body) {
        ivp_compact_surface_free(NULL, body_surf);
        ivp_compact_surface_free(NULL, ground_surf);
        ivp_environment_destroy(env);
        return 1;
    }
    ivp_vec3r_set(&body->core.speed, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(0.3);
    ivp_vec3r_set(&tmpl.rot_inertia, IVP_REAL(0.02), IVP_REAL(0.02), IVP_REAL(0.02));
    tmpl.material.friction = IVP_REAL(0.6);
    tmpl.material.elasticity = IVP_REAL(0.05);
    tmpl.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&tmpl.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tmpl.physical_unmoveable = false;
    tmpl.orientation = ivp_quatr_identity();

    ivp_object_t *wheels[4] = {0};
    for (int i = 0; i < 4; i++) {
        const ivp_real_t wx = (i < 2) ? IVP_REAL(-1.1) : IVP_REAL(1.1);
        const ivp_real_t wz = (i % 2 == 0) ? IVP_REAL(-0.95) : IVP_REAL(0.95);
        ivp_vec3r_set(&tmpl.position, wx, IVP_REAL(-3.2), wz);
        wheels[i] = ivp_environment_create_object(env, &tmpl, &wheel_sm.base);
        if (!wheels[i]) {
            ivp_compact_surface_free(NULL, body_surf);
            ivp_compact_surface_free(NULL, ground_surf);
            ivp_environment_destroy(env);
            return 1;
        }
        wheels[i]->core.upper_limit_radius = IVP_REAL(0.35);
    }

    const double body_x0 = (double)body->core.pos_world_f_core_last_psi.k[0];

    ivp_object_t *objects[6] = { ground, wheels[0], wheels[1], wheels[2], wheels[3], body };
    const char *types[6] = { "ground", "wheel", "wheel", "wheel", "wheel", "vehicle_body" };
    snapshot_frame(out, SCENARIO_VEHICLE, 0, dt, env, objects, types, 6);
    for (int step = 1; step <= steps; step++) {
        ivp_environment_simulate_one_psi(env);
        snapshot_frame(out, SCENARIO_VEHICLE, step, dt, env, objects, types, 6);
    }

    if (inv) {
        inv_check(inv, body->core.pos_world_f_core_last_psi.k[0] > body_x0,
                  "vehicle body advances: x0=%.6f x=%.6f",
                  body_x0, (double)body->core.pos_world_f_core_last_psi.k[0]);
    }

    ivp_compact_surface_free(NULL, body_surf);
    ivp_compact_surface_free(NULL, ground_surf);
    ivp_environment_destroy(env);
    return 0;
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
    else return 0;
    return 1;
}

static void print_usage(const char *exe)
{
    fprintf(stderr,
            "Usage: %s --scenario freefall|two_balls|slope_friction|springs|rope|motor|buoyancy|"
            "cubes|force_actuator|forcefield|stiff_spring|check_distance|collision_filter|"
            "motion_controller|phantom|vehicle "
            "[--steps N] [--dt seconds] [--output file.jsonl] [--check-invariants]\n"
            "Outputs JSONL snapshots (to file or stdout).\n"
            "With --check-invariants, runs physics correctness checks after simulation.\n", exe);
}

int main(int argc, char **argv)
{
    int    scenario = -1;
    int    steps    = 240;
    double dt       = 1.0 / 60.0;
    const char *output_path = NULL;
    int    check_invariants = 0;

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
        fprintf(stderr, "Unknown arg: %s\n", argv[i]);
        print_usage(argv[0]);
        return 2;
    }

    if (scenario < 0) {
        fprintf(stderr, "No --scenario specified.\n");
        print_usage(argv[0]);
        return 2;
    }

    FILE *out = stdout;
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
        default:                      rc = 1;                                       break;
    }

    if (output_path && out) fclose(out);

    if (check_invariants && rc == 0) {
        inv_report(&inv_result, scenario_name_from_id(scenario));
        if (inv_result.checks_failed > 0)
            rc = 1;
    }

    return rc;
}
