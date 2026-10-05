/*
 * IVP Physics Engine - C17 Port
 * Controller reference-parity tests
 *
 * Focused checks of the formulas ported from the reference controllers
 * (ivp_controller/ sources).  Most tests call a controller callback
 * directly with a hand-made ivp_event_sim_t and compare the resulting
 * (sync + async) velocity change with the reference law.
 */

#include "ivp_test.h"

#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/controllers/ivp_actuator_suspension.h>
#include <ivp/controllers/ivp_actuator_force.h>
#include <ivp/controllers/ivp_actuator_torque.h>
#include <ivp/controllers/ivp_actuator_rot_mot.h>
#include <ivp/controllers/ivp_actuator_check_dist.h>
#include <ivp/controllers/ivp_controller_stiff_spring.h>
#include <ivp/controllers/ivp_controller_golem.h>
#include <ivp/controllers/ivp_controller_floating.h>
#include <ivp/controllers/ivp_controller_world_friction.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/controllers/ivp_constraint_fixed_keyed.h>
#include <ivp/controllers/ivp_controller_buoyancy.h>
#include <ivp/utility/ivp_multidimensional_interp.h>
#include <ivp/controllers/ivp_controller_phantom.h>
#include <ivp/controllers/ivp_controller_raycast_car.h>
#include <ivp/controllers/ivp_controller_airboat.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/math/ivp_fast_math.h>

#include <math.h>
#include <string.h>

#ifdef IVP_REAL_DOUBLE
  #define TOL  1e-6
#else
  #define TOL  1e-4f
#endif

#define DT60 (IVP_REAL(1.0) / IVP_REAL(60.0))

/* ── helpers ──────────────────────────────────────────────────────────── */

static ivp_environment_t *make_env(bool gravity)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = DT60;
    if (gravity) ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(9.81), IVP_REAL(0.0));
    else ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

/* ball, radius 0.5, I = 0.4 m r^2 */
static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t mass,
                               ivp_real_t x, ivp_real_t y, ivp_real_t z)
{
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.ball_radius = IVP_REAL(0.5); /* IVP_Template_Ball::radius */
    t.mass = mass;
    ivp_real_t I = IVP_REAL(0.4) * mass * IVP_REAL(0.25);
    ivp_vec3r_set(&t.rot_inertia, I, I, I);
    t.rot_inertia_is_factor = false; /* absolute inertia */
    t.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&t.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&t.position, x, y, z);
    t.orientation = ivp_quatr_identity();
    ivp_object_t *obj = ivp_environment_create_object(env, &t, NULL);
    if (obj) {
        (void)ivp_environment_ensure_core_in_simulation(env, obj);
    }
    return obj;
}

typedef struct box_obj {
    ivp_compact_surface_t *surface;
    ivp_surman_polygon_t   surman;
    ivp_object_t          *obj;
} box_obj_t;

static void make_box(ivp_environment_t *env, box_obj_t *b, float hx, float hy, float hz,
                     ivp_real_t mass, bool fixed, ivp_real_t x, ivp_real_t y, ivp_real_t z)
{
    memset(b, 0, sizeof(*b));
    b->surface = ivp_build_box_surface(NULL, hx, hy, hz);
    if (!b->surface) return;
    ivp_surman_polygon_init(&b->surman, b->surface);
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_POLYGON;
    t.mass = mass;
    ivp_real_t ix = mass * (ivp_real_t)(hy * hy + hz * hz) / IVP_REAL(3.0);
    ivp_real_t iy = mass * (ivp_real_t)(hx * hx + hz * hz) / IVP_REAL(3.0);
    ivp_real_t iz = mass * (ivp_real_t)(hx * hx + hy * hy) / IVP_REAL(3.0);
    ivp_vec3r_set(&t.rot_inertia, ix, iy, iz);
    t.rot_inertia_is_factor = false; /* absolute inertia */
    t.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&t.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    t.physical_unmoveable = fixed;
    t.material.friction = IVP_REAL(0.8);
    t.material.elasticity = IVP_REAL(0.0);
    ivp_vec3r_set(&t.position, x, y, z);
    t.orientation = ivp_quatr_identity();
    b->obj = ivp_environment_create_object(env, &t, &b->surman.base);
    if (b->obj && !fixed) (void)ivp_environment_ensure_core_in_simulation(env, b->obj);
}

static void free_box(box_obj_t *b)
{
    if (b->surface) ivp_compact_surface_free(NULL, b->surface);
    b->surface = NULL;
}

static ivp_event_sim_t make_es(ivp_environment_t *env)
{
    ivp_event_sim_t es;
    memset(&es, 0, sizeof(es));
    es.delta_time = DT60;
    es.inverse_delta_time = IVP_REAL(60.0);
    es.environment = env;
    return es;
}

/* velocity after the PSI: sync speed + pending async change */
static ivp_vec3r_t total_speed(const ivp_object_t *o)
{
    ivp_vec3r_t v;
    ivp_vec3r_add(&v, &o->physical_core->speed, &o->physical_core->speed_change);
    return v;
}

static ivp_vec3r_t total_rot_speed(const ivp_object_t *o)
{
    ivp_vec3r_t w;
    w.k[0] = (ivp_real_t)o->physical_core->rot_speed.k[0] + o->physical_core->rot_speed_change.k[0];
    w.k[1] = (ivp_real_t)o->physical_core->rot_speed.k[1] + o->physical_core->rot_speed_change.k[1];
    w.k[2] = (ivp_real_t)o->physical_core->rot_speed.k[2] + o->physical_core->rot_speed_change.k[2];
    return w;
}

static void place(ivp_object_t *o, ivp_real_t x, ivp_real_t y, ivp_real_t z)
{
    ivp_vec3r_set(&o->physical_core->pos_world_f_core_last_psi, x, y, z);
    o->physical_core->m_world_f_core_last_psi.pos = o->physical_core->pos_world_f_core_last_psi;
}

static void step(ivp_environment_t *env, int n)
{
    for (int i = 0; i < n; i++) ivp_environment_simulate_one_psi(env);
}

static int g_events;
static bool g_last_flag;

/* ── Rot_Mot (ivp_actuator.cxx IVP_Actuator_Rot_Mot::do_simulation_controller) */

IVP_TEST(rot_mot_reference_law_max_torque_zero_means_no_clip) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_rot_mot_state_t m;
    ivp_vec3r_t a0 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t a1 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_rot_mot_state_setup(&m, b, &a0, NULL, &a1, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(100.0));
    ivp_event_sim_t es = make_es(env);
    ivp_controller_rot_mot(&es, b, &m);
    /* dw = 0 -> 0.1; force = P / (dw * |I.axis|) = 1/(0.1*0.1) = 100 */
    IVP_ASSERT_NEAR(m.current_torque, IVP_REAL(100.0), IVP_REAL(1e-3));
    ivp_vec3r_t w = total_rot_speed(b);
    IVP_ASSERT_NEAR(w.k[1], IVP_REAL(100.0) * DT60 * IVP_REAL(10.0), IVP_REAL(1e-3));

    /* max_torque != 0 clips the drive term */
    memset(&b->physical_core->rot_speed_change, 0, sizeof(b->physical_core->rot_speed_change));
    ivp_rot_mot_set_max_torque(&m, IVP_REAL(5.0));
    ivp_controller_rot_mot(&es, b, &m);
    w = total_rot_speed(b);
    IVP_ASSERT_NEAR(w.k[1], IVP_REAL(5.0) * DT60 * IVP_REAL(10.0), IVP_REAL(1e-4));
    ivp_environment_destroy(env);
}

/* ── Torque (ivp_actuator.cxx IVP_Actuator_Torque): |w| > max skips both ways */

IVP_TEST(torque_speed_limit_applies_in_both_directions) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_torque_actuator_state_t t;
    ivp_vec3r_t axis = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_torque_actuator_state_setup(&t, b, &axis, IVP_REAL(1.0), IVP_REAL(2.0));
    ivp_event_sim_t es = make_es(env);

    b->physical_core->rot_speed.k[1] = -3.0f; /* opposite to the torque, but too fast */
    ivp_controller_torque_actuator(&es, b, &t);
    IVP_ASSERT_NEAR(b->physical_core->rot_speed_change.k[1], IVP_REAL(0.0), TOL);

    b->physical_core->rot_speed.k[1] = 1.0f;
    ivp_controller_torque_actuator(&es, b, &t);
    IVP_ASSERT_NEAR(b->physical_core->rot_speed_change.k[1], DT60 * IVP_REAL(10.0), TOL);

    ivp_torque_actuator_state_t d;
    ivp_torque_actuator_state_init(&d);
    IVP_ASSERT_NEAR(d.max_rotation_speed, IVP_REAL(0.0), TOL); /* template default */
    ivp_environment_destroy(env);
}

/* ── Spring (ivp_actuator_spring.cxx:96-164) ──────────────────────────── */

static void spring_broken_cb(void *ctx, ivp_spring_state_t *s)
{
    (void)ctx;
    (void)s;
    g_events++;
}

IVP_TEST(spring_rel_pos_damp_damps_full_relative_velocity) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t z = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_spring_state_t s;
    ivp_spring_state_setup(&s, a, &z, b, &z, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(2.0));
    s.rel_pos_damp = IVP_REAL(3.0);
    ivp_vec3r_set(&b->physical_core->speed, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0)); /* orthogonal */
    ivp_event_sim_t es = make_es(env);
    ivp_controller_spring(&es, a, &s);
    /* impulse on b = -dt * rel_pos_damp * (v_b - v_a) */
    IVP_ASSERT_NEAR(b->physical_core->speed_change.k[1], IVP_REAL(-3.0) * DT60, TOL);
    IVP_ASSERT_NEAR(a->physical_core->speed_change.k[1], IVP_REAL(3.0) * DT60, TOL);
    ivp_environment_destroy(env);
}

IVP_TEST(spring_only_on_stretch_and_break) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t z = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_spring_state_t s;
    ivp_spring_state_setup(&s, a, &z, b, &z, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(2.0));
    s.spring_force_only_on_stretch = true;
    ivp_event_sim_t es = make_es(env);
    ivp_controller_spring(&es, a, &s); /* compressed: no force */
    IVP_ASSERT_NEAR(b->physical_core->speed_change.k[0], IVP_REAL(0.0), TOL);

    /* stretched beyond break_max_len with BREAK: broken event, no force */
    place(b, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    s.max_len_exceed_type = IVP_SFE_BREAK;
    s.break_max_len = IVP_REAL(2.5);
    g_events = 0;
    ivp_spring_set_broken_listener(&s, spring_broken_cb, NULL);
    ivp_controller_spring(&es, a, &s);
    IVP_ASSERT_TRUE(s.broken);
    IVP_ASSERT_EQ(g_events, 1);
    IVP_ASSERT_NEAR(b->physical_core->speed_change.k[0], IVP_REAL(0.0), TOL);
    ivp_environment_destroy(env);
}

IVP_TEST(spring_relative_values_use_virtual_mass) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(6.0), IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t z = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_spring_state_t s;
    ivp_spring_state_setup(&s, a, &z, b, &z, IVP_REAL(10.0), IVP_REAL(1.0), IVP_REAL(2.0));
    ivp_spring_state_make_relative(&s);
    /* virt mass at the centers: 2*6/(2+6) = 1.5 (ivp_actuator_spring.cxx:66-77) */
    IVP_ASSERT_NEAR(s.stiffness, IVP_REAL(15.0), IVP_REAL(1e-3));
    IVP_ASSERT_NEAR(s.damping, IVP_REAL(1.5), IVP_REAL(1e-4));
    ivp_environment_destroy(env);
}

/* ── Suspension (ivp_actuator.cxx IVP_Actuator_Suspension) ─────────────── */

IVP_TEST(suspension_max_body_force_is_hard_clip) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *body = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *wheel = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_vec3r_t z = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_suspension_state_t s;
    ivp_suspension_state_setup(&s, body, wheel, &z, &z, IVP_REAL(100.0), IVP_REAL(0.0),
                               IVP_REAL(0.0), IVP_REAL(2.0));
    ivp_suspension_set_max_body_force(&s, IVP_REAL(30.0));
    ivp_event_sim_t es = make_es(env);
    ivp_controller_suspension(&es, body, &s);
    /* wheel gets the full 100 N, the body only the clipped 30 N */
    IVP_ASSERT_NEAR(wheel->physical_core->speed_change.k[1], IVP_REAL(100.0) * DT60, IVP_REAL(1e-3));
    IVP_ASSERT_NEAR(body->physical_core->speed_change.k[1], IVP_REAL(-30.0) * DT60, IVP_REAL(1e-3));

    /* max_body_force 0: no body force at all */
    memset(&body->physical_core->speed_change, 0, sizeof(body->physical_core->speed_change));
    ivp_suspension_set_max_body_force(&s, IVP_REAL(0.0));
    ivp_controller_suspension(&es, body, &s);
    IVP_ASSERT_NEAR(body->physical_core->speed_change.k[1], IVP_REAL(0.0), TOL);
    ivp_environment_destroy(env);
}

/* ── Force actuator template defaults (ivp_actuator.hxx) ──────────────── */

IVP_TEST(force_actuator_push_second_default_false) {
    ivp_force_actuator_state_t f;
    ivp_force_actuator_state_init(&f);
    IVP_ASSERT_TRUE(f.push_first_object);
    IVP_ASSERT_FALSE(f.push_second_object);
}

/* ── Check_Dist (ivp_actuator.cxx:283-371) ────────────────────────────── */

static void check_cb(void *ctx, ivp_object_t *a, ivp_object_t *b, ivp_real_t d, bool inside)
{
    (void)ctx; (void)a; (void)b; (void)d;
    g_events++;
    g_last_flag = inside;
}

IVP_TEST(check_dist_boundary_is_outside_and_init_is_silent) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t pa = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t pb = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    g_events = 0;
    ivp_check_dist_state_t c;
    ivp_check_dist_state_setup(&c, a, &pa, b, &pb, IVP_REAL(1.0));
    ivp_check_dist_set_callback(&c, check_cb, NULL);
    IVP_ASSERT_EQ(c.is_outside, 1);   /* len == range is outside (len < range = inside) */
    IVP_ASSERT_EQ(g_events, 0);       /* constructor evaluation has no listener */

    place(b, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_event_sim_t es = make_es(env);
    ivp_controller_check_dist(&es, a, &c);
    IVP_ASSERT_EQ(g_events, 1);
    IVP_ASSERT_TRUE(g_last_flag);
    ivp_environment_destroy(env);
}

/* libivp extra use_object_centers also holds in the attached (reference
 * hull-event) mode: the anchors are placed at the mass centres on attach. */
IVP_TEST(check_dist_attached_uses_object_centers) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0));
    /* anchors 10 m off both bodies: anchor distance 20 m, centre distance 3 m */
    ivp_vec3r_t pa = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(10.0), IVP_REAL(0.0));
    ivp_vec3r_t pb = ivp_vec3r(IVP_REAL(3.0), IVP_REAL(-10.0), IVP_REAL(0.0));
    ivp_check_dist_state_t c;
    ivp_check_dist_state_setup(&c, a, &pa, b, &pb, IVP_REAL(5.0));
    c.use_object_centers = true;
    IVP_ASSERT_TRUE(ivp_check_dist_attach(&c, env));
    IVP_ASSERT_NEAR(c.current_distance, IVP_REAL(3.0), IVP_REAL(1e-5));
    IVP_ASSERT_EQ(c.is_outside, 0);
    ivp_check_dist_detach(&c);
    ivp_environment_destroy(env);
}

/* ── Stiff spring (ivp_controller_stiff_spring.cxx): k = 1 closes the gap in one PSI */

IVP_TEST(stiff_spring_full_stiffness_reaches_rest_length_in_one_psi) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    /* The spring closes the 1 m surface gap of the r = 0.5 balls within the
     * PSI; keep the pair from colliding so only the spring impulse is seen. */
    ivp_object_set_collision_group(a, 1);
    ivp_object_set_collision_group(b, 1);
    ivp_vec3r_t z = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_stiff_spring_state_t s;
    ivp_stiff_spring_state_setup(&s, a, &z, b, &z, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_environment_announce_dependent_controller(env, 0, ivp_controller_stiff_spring, &s,
                                                  ivp_stiff_spring_discovery);
    step(env, 1);
    /* a = (len_0 - len) / dt * k; impulse = a / tpm00 (tpm00 = 1/m0 + 1/m1) */
    IVP_ASSERT_NEAR(s.last_impulse, IVP_REAL(-30.0), IVP_REAL(1e-2));
    /* closing speed 60 m/s: the gap of 1 m is closed during the next PSI */
    ivp_vec3r_t va = total_speed(a), vb = total_speed(b);
    IVP_ASSERT_NEAR(va.k[0] - vb.k[0], IVP_REAL(60.0), IVP_REAL(1e-2));
    IVP_ASSERT_NEAR(va.k[0] + vb.k[0], IVP_REAL(0.0), IVP_REAL(1e-3));
    ivp_environment_destroy(env);
}

/* ── Golem (ivp_controller_golem.cxx): far distance -> problem, no force */

static void golem_problem_cb(void *ctx, ivp_object_t *o, ivp_real_t pe, ivp_real_t re)
{
    (void)ctx; (void)o; (void)pe; (void)re;
    g_events++;
}

IVP_TEST(golem_far_distance_calls_problem_without_force) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_golem_state_t g;
    ivp_golem_state_init(&g);
    ivp_golem_state_setup(&g, b);
    g.problem_fn = golem_problem_cb;
    ivp_vec3r_t far = ivp_vec3r(IVP_REAL(20.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_golem_set_target_position(&g, &far);
    g_events = 0;
    ivp_event_sim_t es = make_es(env);
    ivp_controller_golem(&es, b, &g);
    IVP_ASSERT_EQ(g_events, 1);
    IVP_ASSERT_EQ((int)g.last_problem, (int)IVP_GP_FAR_DISTANCE);
    ivp_vec3r_t v = total_speed(b);
    IVP_ASSERT_NEAR(v.k[0], IVP_REAL(0.0), TOL);
    ivp_environment_destroy(env);
}

/* ── Floating (ivp_controller_floating.cxx:30-70) ─────────────────────── */

IVP_TEST(floating_point_impulse_and_force_clip) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_floating_state_t f;
    ivp_floating_init_defaults(&f);
    IVP_ASSERT_NEAR(f.max_adhesive_force, (ivp_real_t)IVP_FLOAT_MAX, IVP_REAL(1e10)); /* P_FLOAT_MAX */
    IVP_ASSERT_NEAR(f.max_repulsive_force, (ivp_real_t)IVP_FLOAT_MAX, IVP_REAL(1e10));
    ivp_floating_set_object(&f, b);
    f.target_distance = IVP_REAL(0.5);
    f.current_distance = IVP_REAL(1.0);
    ivp_event_sim_t es = make_es(env);
    ivp_controller_floating(&es, b, &f);
    /* a = (cur - target) / dt; impulse = a / (1/m): v.y = 30 */
    ivp_vec3r_t v = total_speed(b);
    IVP_ASSERT_NEAR(v.k[1], IVP_REAL(30.0), IVP_REAL(1e-3));

    ivp_vec3r_set_zero(&b->physical_core->speed);
    ivp_vec3r_set_zero(&b->physical_core->speed_change);
    f.max_adhesive_force = IVP_REAL(600.0);
    ivp_controller_floating(&es, b, &f);
    v = total_speed(b);
    IVP_ASSERT_NEAR(v.k[1], IVP_REAL(10.0), IVP_REAL(1e-3));
    ivp_environment_destroy(env);
}

/* ── World friction (ivp_controller_world_friction.cxx) ───────────────── */

IVP_TEST(world_friction_reference_desired_speed_clamp) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_world_friction_state_t w;
    ivp_world_friction_init(&w, b);
    IVP_ASSERT_NEAR(w.friction_value_translation.k[0], (ivp_real_t)(0.4f * 9.81f), TOL);
    IVP_ASSERT_NEAR(w.friction_value_rotation.k[0], (ivp_real_t)(0.4f * 9.81f) * IVP_REAL(0.3), TOL);
    ivp_vec3r_set(&b->physical_core->speed, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_event_sim_t es = make_es(env);
    ivp_controller_world_friction(&es, b, &w);
    ivp_vec3r_t v = total_speed(b);
    IVP_ASSERT_NEAR(v.k[0], IVP_REAL(1.0) - w.friction_value_translation.k[0] * DT60, IVP_REAL(1e-4));

    /* a desired speed is approached, not the rest state */
    ivp_vec3r_t want = ivp_vec3r(IVP_REAL(0.95), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_world_friction_set_desired_speed_ws(&w, &want);
    ivp_vec3r_set(&b->physical_core->speed, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set_zero(&b->physical_core->speed_change);
    ivp_controller_world_friction(&es, b, &w);
    v = total_speed(b);
    IVP_ASSERT_NEAR(v.k[0], IVP_REAL(0.95), IVP_REAL(1e-4));
    ivp_environment_destroy(env);
}

/* ── Constraint_Local (ivp_constraint_local.cxx) ───────────────────────── */

static ivp_real_t anchor_drift(const ivp_object_t *b, const ivp_vec3r_t *anchor_os)
{
    ivp_vec3r_t p;
    ivp_transform_point(&p, &b->physical_core->m_world_f_core_last_psi, anchor_os);
    return ivp_vec3r_length(&p);
}

IVP_TEST(constraint_ball_socket_ws_holds_pendulum) {
    ivp_environment_t *env = make_env(true);
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t anchor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_constraint_state_t c;
    /* World/static endpoints stay on the generic path. */
    ivp_constraint_init_ball_socket_ws(&c, NULL, &anchor, b);
    ivp_environment_announce_dependent_controller(env, 0, ivp_controller_constraint, &c,
                                                  ivp_constraint_discovery);
    step(env, 120);
    ivp_vec3r_t a_os = ivp_vec3r(IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    IVP_ASSERT_TRUE(anchor_drift(b, &a_os) < IVP_REAL(0.05));
    ivp_vec3r_t p;
    ivp_object_get_position(b, &p);
    IVP_ASSERT_TRUE(p.k[1] > IVP_REAL(0.2)); /* it swings down */
    IVP_ASSERT_FALSE(c.is_broken);
    ivp_environment_destroy(env);
}

static void run_limited_ball_socket(ivp_constraint_force_exceed_t action, ivp_constraint_state_t *c,
                                    ivp_vec3r_t *out_pos)
{
    ivp_environment_t *env = make_env(true);
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_vec3r_t anchor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_constraint_init_ball_socket_ws(c, NULL, &anchor, b);
    c->break_force_threshold = IVP_REAL(1e-3);
    c->force_exceed_action = action;
    ivp_environment_announce_dependent_controller(env, 0, ivp_controller_constraint, c,
                                                  ivp_constraint_discovery);
    step(env, 30);
    ivp_object_get_position(b, out_pos);
    ivp_environment_destroy(env);
}

IVP_TEST(constraint_max_impulse_break_and_clip) {
    ivp_constraint_state_t c;
    ivp_vec3r_t p;
    run_limited_ball_socket(IVP_CFE_BREAK, &c, &p);
    IVP_ASSERT_TRUE(c.is_broken);
    IVP_ASSERT_TRUE(p.k[1] > IVP_REAL(1.5)); /* free fall after breaking */

    run_limited_ball_socket(IVP_CFE_CLIP, &c, &p);
    IVP_ASSERT_FALSE(c.is_broken);
    IVP_ASSERT_TRUE(p.k[1] > IVP_REAL(1.5)); /* clipped impulses cannot hold it */
}

IVP_TEST(constraint_hinge_ws_keeps_motion_in_plane) {
    ivp_environment_t *env = make_env(true);
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    b->physical_core->speed.k[2] = IVP_REAL(1.0); /* out of plane kick */
    ivp_vec3r_t anchor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t axis = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0));
    ivp_constraint_state_t c;
    ivp_constraint_init_hinge_ws(&c, NULL, &anchor, &axis, b);
    ivp_environment_announce_dependent_controller(env, 0, ivp_controller_constraint, &c,
                                                  ivp_constraint_discovery);
    step(env, 60);
    ivp_vec3r_t p;
    ivp_object_get_position(b, &p);
    IVP_ASSERT_TRUE(ivp_fabs(p.k[2]) < IVP_REAL(0.05));
    ivp_vec3r_t w = total_rot_speed(b);
    IVP_ASSERT_TRUE(ivp_fabs(w.k[0]) < IVP_REAL(0.05));
    IVP_ASSERT_TRUE(ivp_fabs(w.k[1]) < IVP_REAL(0.05));
    ivp_environment_destroy(env);
}

/* ── Fixed keyed (ivp_constraint_fixed_keyed.cxx) ─────────────────────── */

IVP_TEST(fixed_keyed_drives_object_to_target) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_fixed_keyed_state_t k;
    ivp_fixed_keyed_init(&k, NULL, b, NULL);
    IVP_ASSERT_NEAR(k.force_factor, IVP_REAL(0.8), TOL);
    IVP_ASSERT_NEAR(k.damp_factor, IVP_REAL(1.0), TOL);
    ivp_vec3r_t target = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_fixed_keyed_set_target_pos(&k, &target);
    ivp_environment_announce_dependent_controller(env, 0, ivp_controller_fixed_keyed, &k,
                                                  ivp_fixed_keyed_discovery);
    step(env, 60);
    ivp_vec3r_t p;
    ivp_object_get_position(b, &p);
    IVP_ASSERT_TRUE(ivp_vec3r_length(&p) < IVP_REAL(0.01));
    ivp_environment_destroy(env);
}

/* ── Buoyancy (ivp_buoyancy_solver.cxx): exact submerged pyramid volume ── */

IVP_TEST(buoyancy_half_submerged_box_impulse) {
    ivp_environment_t *env = make_env(true);
    box_obj_t box;
    make_box(env, &box, 0.5f, 0.5f, 0.5f, IVP_REAL(2.0), false, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    IVP_ASSERT_TRUE(box.obj != NULL);
    ivp_buoyancy_state_t s;
    ivp_buoyancy_init(&s, IVP_REAL(0.0), IVP_REAL(1.0));
    s.linear_drag = IVP_REAL(0.0);
    s.friction_damp_factor = IVP_REAL(0.0);
    s.angular_drag = IVP_REAL(0.0);
    s.viscosity_factor = IVP_REAL(0.0);
    ivp_event_sim_t es = make_es(env);
    ivp_controller_buoyancy(&es, box.obj, &s);
    /* V_under = 0.5 m^3, impulse = -g * V * rho * dt, dv = impulse / m */
    ivp_vec3r_t v = total_speed(box.obj);
    IVP_ASSERT_NEAR(v.k[1], IVP_REAL(-9.81) * IVP_REAL(0.5) * DT60 / IVP_REAL(2.0), IVP_REAL(1e-3));
    IVP_ASSERT_NEAR(v.k[0], IVP_REAL(0.0), IVP_REAL(1e-4));
    ivp_vec3r_t w = total_rot_speed(box.obj);
    IVP_ASSERT_NEAR(w.k[0], IVP_REAL(0.0), IVP_REAL(1e-3));
    IVP_ASSERT_NEAR(w.k[2], IVP_REAL(0.0), IVP_REAL(1e-3));
    ivp_buoyancy_state_destroy(&s);
    ivp_environment_destroy(env);
    free_box(&box);
}

/* ── IVP_Multidimensional_Interpolator (ivp_multidimensional_interp.cxx) ──
 * Buoyancy interpolation (use_interpolation) is compiled out of the
 * reference engine (IVP_NO_MD_INTERPOLATION), so no scenario reaches it.
 * The expected states below come from the reference
 * IVP_Multidimensional_Interpolator compiled without that define and
 * driven by the same sequence (strict FP reference build). */

typedef struct mi_ref_state {
    int   ok;          /* check_interpolation result, -1 after an insertion */
    int   nr_of_vectors_involved;
    int   nr_occupied;
    float out[2];      /* output vector */
    float slot[4][3];  /* previous_inputs[i]->weight_statistic, previous_inputs[i]->element[0],
                        * previous_solutions[i]->element[1] */
    int   counter_vector_replacement;
    int   counter_tries_nr_of_vectors_involved;
} mi_ref_state_t;

static const mi_ref_state_t mi_ref_states[] = {
    /* 0 */ { -1, 2, 1, { 0x0p+0f, 0x0p+0f }, { { 0x1.400002p+2f, 0x0p+0f, -0x1p+1f }, { 0x0p+0f, 0x0p+0f, 0x0p+0f }, { 0x0p+0f, 0x0p+0f, 0x0p+0f }, { 0x0p+0f, 0x0p+0f, 0x0p+0f } }, 50, 0 },
    /* 1 */ { -1, 2, 2, { 0x0p+0f, 0x0p+0f }, { { 0x1.400002p+2f, 0x1p+0f, -0x1p+0f }, { 0x1.400002p+2f, 0x0p+0f, -0x1p+1f }, { 0x0p+0f, 0x0p+0f, 0x0p+0f }, { 0x0p+0f, 0x0p+0f, 0x0p+0f } }, 50, 0 },
    /* 2 */ { -1, 2, 3, { 0x0p+0f, 0x0p+0f }, { { 0x1.400002p+2f, 0x1p-2f, 0x1p-1f }, { 0x1.400002p+2f, 0x1p+0f, -0x1p+0f }, { 0x1.400002p+2f, 0x0p+0f, -0x1p+1f }, { 0x0p+0f, 0x0p+0f, 0x0p+0f } }, 50, 0 },
    /* 3 */ { 1, 3, 3, { 0x1.d33334p+0f, -0x1.ccccd2p-2f }, { { 0x1.200002p+2f, 0x1p-2f, 0x1p-1f }, { 0x1.133336p+2f, 0x1p+0f, -0x1p+0f }, { 0x1.0ccccep+2f, 0x0p+0f, -0x1p+1f }, { 0x0p+0f, 0x0p+0f, 0x0p+0f } }, 50, 1 },
    /* 4 */ { 0, 4, 3, { 0x1.d33334p+0f, -0x1.ccccd2p-2f }, { { 0x1.200002p+2f, 0x1p-2f, 0x1p-1f }, { 0x1.133336p+2f, 0x1p+0f, -0x1p+0f }, { 0x1.0ccccep+2f, 0x0p+0f, -0x1p+1f }, { 0x0p+0f, 0x0p+0f, 0x0p+0f } }, 50, 2 },
    /* 5 */ { 1, 1, 3, { 0x0p+0f, 0x0p+0f }, { { 0x1.133336p+2f, 0x1p+0f, -0x1p+0f }, { 0x1.0ccccep+2f, 0x0p+0f, -0x1p+1f }, { 0x1.ccccdp+1f, 0x1p-2f, 0x1p-1f }, { 0x0p+0f, 0x0p+0f, 0x0p+0f } }, 50, 3 },
    /* 6 */ { -1, 1, 4, { 0x0p+0f, 0x0p+0f }, { { 0x1.400002p+2f, 0x1p-1f, 0x1.ccccccp-1f }, { 0x1.133336p+2f, 0x1p+0f, -0x1p+0f }, { 0x1.0ccccep+2f, 0x0p+0f, -0x1p+1f }, { 0x1.ccccdp+1f, 0x1p-2f, 0x1p-1f } }, 50, 3 },
    /* 7 */ { -1, 1, 4, { 0x0p+0f, 0x0p+0f }, { { 0x1.400002p+2f, 0x1p-1f, 0x1.ccccccp-1f }, { 0x1.400002p+2f, 0x1.8p-1f, 0x1.99999ap-3f }, { 0x1.133336p+2f, 0x1p+0f, -0x1p+0f }, { 0x1.ccccdp+1f, 0x1p-2f, 0x1p-1f } }, 49, 3 },
    /* 8 */ { -1, 1, 4, { 0x0p+0f, 0x0p+0f }, { { 0x1.400002p+2f, 0x1p-1f, 0x1.ccccccp-1f }, { 0x1.400002p+2f, 0x1.99999ap-4f, 0x1.8p+0f }, { 0x1.133336p+2f, 0x1p+0f, -0x1p+0f }, { 0x1.ccccdp+1f, 0x1p-2f, 0x1p-1f } }, 48, 3 },
    /* 9 */ { 0, 5, 4, { 0x0p+0f, 0x0p+0f }, { { 0x1.400002p+2f, 0x1p-1f, 0x1.ccccccp-1f }, { 0x1.400002p+2f, 0x1.99999ap-4f, 0x1.8p+0f }, { 0x1.133336p+2f, 0x1p+0f, -0x1p+0f }, { 0x1.ccccdp+1f, 0x1p-2f, 0x1p-1f } }, 48, 0 },
    /* 10 */ { 0, 5, 4, { 0x0p+0f, 0x0p+0f }, { { 0x1.400002p+2f, 0x1p-1f, 0x1.ccccccp-1f }, { 0x1.400002p+2f, 0x1.99999ap-4f, 0x1.8p+0f }, { 0x1.133336p+2f, 0x1p+0f, -0x1p+0f }, { 0x1.ccccdp+1f, 0x1p-2f, 0x1p-1f } }, 48, 1 },
    /* 11 */ { 0, 5, 4, { 0x0p+0f, 0x0p+0f }, { { 0x1.400002p+2f, 0x1p-1f, 0x1.ccccccp-1f }, { 0x1.400002p+2f, 0x1.99999ap-4f, 0x1.8p+0f }, { 0x1.133336p+2f, 0x1p+0f, -0x1p+0f }, { 0x1.ccccdp+1f, 0x1p-2f, 0x1p-1f } }, 48, 2 },
    /* 12 */ { -1, 5, 4, { 0x0p+0f, 0x0p+0f }, { { 0x1.400002p+2f, 0x0p+0f, 0x1.ccccccp-1f }, { 0x1.400002p+2f, 0x1.99999ap-4f, 0x1.8p+0f }, { 0x1.133336p+2f, 0x1p+0f, -0x1p+0f }, { 0x1.ccccdp+1f, 0x1p-2f, 0x1p-1f } }, 47, 2 },
    /* 13 */ { 0, 5, 4, { 0x0p+0f, 0x0p+0f }, { { 0x1.400002p+2f, 0x0p+0f, 0x1.ccccccp-1f }, { 0x1.400002p+2f, 0x1.99999ap-4f, 0x1.8p+0f }, { 0x1.133336p+2f, 0x1p+0f, -0x1p+0f }, { 0x1.ccccdp+1f, 0x1p-2f, 0x1p-1f } }, 47, 0 },
    /* 14 */ { 0, 5, 4, { 0x0p+0f, 0x0p+0f }, { { 0x1.400002p+2f, 0x0p+0f, 0x1.ccccccp-1f }, { 0x1.400002p+2f, 0x1.99999ap-4f, 0x1.8p+0f }, { 0x1.133336p+2f, 0x1p+0f, -0x1p+0f }, { 0x1.ccccdp+1f, 0x1p-2f, 0x1p-1f } }, 47, 1 },
    /* 15 */ { 1, 4, 4, { 0x1.bf076p-1f, 0x1.6dd0bap-2f }, { { 0x1.19611cp+2f, 0x1.99999ap-4f, 0x1.8p+0f }, { 0x1.0d054ep+2f, 0x0p+0f, 0x1.ccccccp-1f }, { 0x1.ed491cp+1f, 0x1p+0f, -0x1p+0f }, { 0x1.6edfep+1f, 0x1p-2f, 0x1p-1f } }, 47, 2 },
    /* 16 */ { -1, 4, 4, { 0x0p+0f, 0x0p+0f }, { { 0x1.400002p+2f, 0x1p+0f, 0x1.99999ap-3f }, { 0x1.19611cp+2f, 0x1.99999ap-4f, 0x1.8p+0f }, { 0x1.0d054ep+2f, 0x0p+0f, 0x1.ccccccp-1f }, { 0x1.ed491cp+1f, 0x1p+0f, -0x1p+0f } }, 46, 2 },
    /* 17 */ { 0, 5, 4, { 0x1.bf076p-1f, 0x1.6dd0bap-2f }, { { 0x1.400002p+2f, 0x1p+0f, 0x1.99999ap-3f }, { 0x1.19611cp+2f, 0x1.99999ap-4f, 0x1.8p+0f }, { 0x1.0d054ep+2f, 0x0p+0f, 0x1.ccccccp-1f }, { 0x1.ed491cp+1f, 0x1p+0f, -0x1p+0f } }, 46, 0 },
    /* 18 */ { 0, 5, 4, { 0x1.bf076p-1f, 0x1.6dd0bap-2f }, { { 0x1.400002p+2f, 0x1p+0f, 0x1.99999ap-3f }, { 0x1.19611cp+2f, 0x1.99999ap-4f, 0x1.8p+0f }, { 0x1.0d054ep+2f, 0x0p+0f, 0x1.ccccccp-1f }, { 0x1.ed491cp+1f, 0x1p+0f, -0x1p+0f } }, 46, 1 },
    /* 19 */ { 0, 5, 4, { 0x1.bf076p-1f, 0x1.6dd0bap-2f }, { { 0x1.400002p+2f, 0x1p+0f, 0x1.99999ap-3f }, { 0x1.19611cp+2f, 0x1.99999ap-4f, 0x1.8p+0f }, { 0x1.0d054ep+2f, 0x0p+0f, 0x1.ccccccp-1f }, { 0x1.ed491cp+1f, 0x1p+0f, -0x1p+0f } }, 46, 2 },
    /* 20 */ { -1, 5, 4, { 0x0p+0f, 0x0p+0f }, { { 0x1.400002p+2f, 0x1p+0f, 0x1.99999ap-3f }, { 0x1.400002p+2f, 0x1p-2f, 0x1.8p+0f }, { 0x1.19611cp+2f, 0x1.99999ap-4f, 0x1.8p+0f }, { 0x1.ed491cp+1f, 0x1p+0f, -0x1p+0f } }, 45, 2 },
};

/* 0 when the interpolator matches ref, else the 1-based field index */
static int mi_state_mismatch(const ivp_mi_interpolator_t *mi, int ok, const ivp_mi_vector_t *out,
                             const mi_ref_state_t *ref)
{
    if (ok != ref->ok) return 1;
    if (mi->nr_of_vectors_involved != ref->nr_of_vectors_involved) return 2;
    if (mi->nr_occupied != ref->nr_occupied) return 3;
    float o0 = out ? out->element[0] : 0.0f, o1 = out ? out->element[1] : 0.0f;
    if (memcmp(&o0, &ref->out[0], sizeof(float)) || memcmp(&o1, &ref->out[1], sizeof(float))) return 4;
    for (int i = 0; i < 4; i++) {
        if (memcmp(&mi->previous_inputs[i]->weight_statistic, &ref->slot[i][0], sizeof(float))) return 5;
        if (memcmp(&mi->previous_inputs[i]->element[0], &ref->slot[i][1], sizeof(float))) return 6;
        if (memcmp(&mi->previous_solutions[i]->element[1], &ref->slot[i][2], sizeof(float))) return 7;
    }
    if (mi->counter_vector_replacement != ref->counter_vector_replacement) return 8;
    if (mi->counter_tries_nr_of_vectors_involved != ref->counter_tries_nr_of_vectors_involved) return 9;
    return 0;
}

static ivp_mi_vector_t mi_tmp(float *buf, const float *values, int n)
{
    ivp_mi_vector_t v;
    ivp_mi_vector_init(&v, buf, n);
    for (int i = 0; i < n; i++) v.element[i] = values[i];
    return v;
}

IVP_TEST(multidimensional_interpolator_matches_reference) {
    static const float I[6][3] = { {0.0f, 0.0f, 0.0f}, {1.0f, 0.5f, 0.0f}, {0.25f, 1.0f, 0.5f},
                                   {0.5f, 0.25f, 1.0f}, {0.75f, 0.75f, 0.25f}, {0.1f, 0.9f, 0.9f} };
    static const float Sv[6][2] = { {1.0f, -2.0f}, {2.5f, -1.0f}, {1.75f, 0.5f},
                                    {0.3f, 0.9f}, {1.1f, 0.2f}, {-0.5f, 1.5f} };
    static const float Q[6][3] = { {0.425f, 0.65f, 0.25f}, {5.0f, 5.0f, 5.0f}, {0.25f, 1.0f, 0.5f},
                                   {0.3f, 0.4f, 0.5f}, {0.6f, 0.2f, 0.7f}, {0.45f, 0.55f, 0.35f} };
    ivp_mi_interpolator_t mi;
    IVP_ASSERT_EQ(ivp_mi_init(&mi, 4, 3, 2, NULL), 0);
    ivp_mi_vector_t *out = ivp_mi_vector_malloc(2, NULL);
    IVP_ASSERT_NOT_NULL(out);
    float bi[3], bs[2];
    ivp_mi_vector_t vi, vs;
    int step = 0, ok, mismatch;
    const int n_states = (int)(sizeof(mi_ref_states) / sizeof(mi_ref_states[0]));

#define MI_EXPECT(ok_, out_)                                                          \
    do {                                                                              \
        IVP_ASSERT_TRUE(step < n_states);                                             \
        mismatch = mi_state_mismatch(&mi, (ok_), (out_), &mi_ref_states[step]);       \
        if (mismatch) fprintf(stderr, "  MI step %d: field %d differs\n", step, mismatch); \
        IVP_ASSERT_EQ(mismatch, 0);                                                   \
        step++;                                                                       \
    } while (0)
#define MI_CONV(k, sk)                                                                \
    do {                                                                              \
        vi = mi_tmp(bi, I[k], 3); vs = mi_tmp(bs, Sv[sk], 2);                         \
        ivp_mi_add_new_input_solution_combination_conventional(&mi, &vi, &vs);        \
        MI_EXPECT(-1, NULL);                                                          \
    } while (0)
#define MI_STOCH(k, sk)                                                               \
    do {                                                                              \
        vi = mi_tmp(bi, I[k], 3); vs = mi_tmp(bs, Sv[sk], 2);                         \
        ivp_mi_add_new_input_solution_combination_stochastic(&mi, &vi, &vs);          \
        MI_EXPECT(-1, NULL);                                                          \
    } while (0)
#define MI_CHECK(q, tries, max_res)                                                   \
    do {                                                                              \
        vi = mi_tmp(bi, Q[q], 3);                                                     \
        ok = ivp_mi_check_interpolation(&mi, &vi, (tries), (max_res), out) ? 1 : 0;   \
        MI_EXPECT(ok, out);                                                           \
    } while (0)

    for (int i = 0; i < 3; i++) MI_CONV(i, i);
    MI_CHECK(0, 15, 1e-2f); /* affine combination of the three inputs */
    MI_CHECK(1, 15, 1e-2f); /* far outside: weights out of range */
    MI_CHECK(2, 15, 1e-2f); /* equals the first input: one vector, zero output */
    MI_CONV(3, 3);
    MI_STOCH(4, 4);
    MI_STOCH(5, 5);
    for (int r = 0; r < 3; r++) {
        MI_CHECK(3, 2, 1e-2f);
        MI_CHECK(4, 2, 1e-2f);
        MI_CHECK(5, 2, 1e-3f);
        MI_STOCH(r, r + 3);
    }
    IVP_ASSERT_EQ(step, n_states);
#undef MI_EXPECT
#undef MI_CONV
#undef MI_STOCH
#undef MI_CHECK

    ivp_mi_vector_free(out, NULL);
    ivp_mi_destroy(&mi);
}

/* ── Phantom (ivp_phantom.cxx): exact overlap + 0.5 m exit hysteresis ─── */

static void phantom_cb(void *ctx, ivp_object_t *owner, ivp_object_t *other, bool entered)
{
    (void)ctx; (void)owner; (void)other;
    g_events++;
    g_last_flag = entered;
}

IVP_TEST(phantom_enter_exit_hysteresis) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(1.4), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_phantom_state_t p;
    ivp_vec3r_t c = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_phantom_init_sphere(&p, &c, IVP_REAL(1.0), phantom_cb, NULL);
    IVP_ASSERT_NEAR(p.exit_policy_extra_radius, IVP_REAL(0.5), TOL);
    g_events = 0;
    ivp_event_sim_t es = make_es(env);
    ivp_controller_phantom(&es, b, &p); /* overlap 0.1: enter */
    IVP_ASSERT_EQ(g_events, 1);
    IVP_ASSERT_TRUE(g_last_flag);

    place(b, IVP_REAL(1.7), IVP_REAL(0.0), IVP_REAL(0.0)); /* separated 0.2, moved 0.3 < 0.6 */
    ivp_controller_phantom(&es, b, &p);
    IVP_ASSERT_EQ(g_events, 1);
    IVP_ASSERT_EQ(p.n_tracked, 1);

    place(b, IVP_REAL(2.2), IVP_REAL(0.0), IVP_REAL(0.0)); /* moved 0.8 > 0.6: re-check, exit */
    ivp_controller_phantom(&es, b, &p);
    IVP_ASSERT_EQ(g_events, 2);
    IVP_ASSERT_FALSE(g_last_flag);
    IVP_ASSERT_EQ(p.n_tracked, 0);
    ivp_environment_destroy(env);
}

IVP_TEST(phantom_object_distance_is_exact_convex) {
    ivp_environment_t *env = make_env(false);
    box_obj_t box;
    make_box(env, &box, 1.0f, 1.0f, 1.0f, IVP_REAL(1.0), true, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(2.0), IVP_REAL(0.0));
    ivp_phantom_state_t p;
    ivp_phantom_init_object(&p, box.obj, NULL, phantom_cb, NULL);
    /* nearest box point (1,1,0): distance sqrt(2) - 0.5 (a bounding sphere would overlap) */
    ivp_real_t d = ivp_phantom_distance_to_object(&p, b);
    IVP_ASSERT_NEAR(d, ivp_sqrt(IVP_REAL(2.0)) - IVP_REAL(0.5), IVP_REAL(1e-3));
    ivp_environment_destroy(env);
    free_box(&box);
}

/* ── Raycast car (ivp_controller_raycast_car.cxx) ─────────────────────── */

IVP_TEST(raycast_car_rests_on_real_ground) {
    ivp_environment_t *env = make_env(true);
    box_obj_t ground, chassis;
    make_box(env, &ground, 10.0f, 0.5f, 10.0f, IVP_REAL(1.0), true, IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    make_box(env, &chassis, 1.0f, 0.25f, 2.0f, IVP_REAL(100.0), false, IVP_REAL(0.0), IVP_REAL(-0.1), IVP_REAL(0.0));
    IVP_ASSERT_TRUE(ground.obj && chassis.obj);
    ivp_car_state_t car;
    IVP_ASSERT_TRUE(ivp_car_init_4wheel(&car, chassis.obj, IVP_REAL(3.0), IVP_REAL(1.8), IVP_REAL(0.3), IVP_REAL(0.5)));
    for (int i = 0; i < car.n_wheels; i++) {
        car.wheels[i].spring_constant = (float)(IVP_REAL(100.0) * IVP_REAL(9.81) / IVP_REAL(4.0) / IVP_REAL(0.15));
        car.wheels[i].spring_damp_relax = 400.0f;
        car.wheels[i].spring_damp_compress = 400.0f;
    }
    step(env, 240);
    ivp_real_t sum = IVP_REAL(0.0);
    for (int i = 0; i < car.n_wheels; i++) {
        IVP_ASSERT_TRUE(car.wheels[i].grounded);
        sum += car.wheels[i].pressure;
    }
    /* the springs carry the weight (pressure = spring force of the PSI) */
    IVP_ASSERT_NEAR(sum, IVP_REAL(100.0) * IVP_REAL(9.81), IVP_REAL(60.0));
    ivp_vec3r_t p;
    ivp_object_get_position(chassis.obj, &p);
    /* equilibrium: about 0.8 - 0.15 below the hard points (the compression
     * damping of the per-PSI gravity speed carries part of the weight) */
    IVP_ASSERT_NEAR(p.k[1], IVP_REAL(0.5) - IVP_REAL(0.65), IVP_REAL(0.06));
    ivp_car_destroy(&car);
    ivp_environment_destroy(env);
    free_box(&chassis);
    free_box(&ground);
}

/* ── Airboat (ivp_controller_airboat.cpp:272-306) ─────────────────────── */

static void *controller_parity_fail_alloc(size_t size, void *ctx)
{
    (void)size;
    (void)ctx;
    return NULL;
}

IVP_TEST(airboat_registration_failure_is_atomic) {
    ivp_environment_t *env = make_env(true);
    ivp_object_t *b = make_ball(env, IVP_REAL(100.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(b);
    ivp_vec3r_set(&b->physical_core->speed, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(3.0));
    b->physical_core->rot_speed = ivp_vec3f(4.0f, 5.0f, 6.0f);
    const ivp_vec3r_t speed_before = b->physical_core->speed;
    const ivp_vec3f_t rot_speed_before = b->physical_core->rot_speed;

    ivp_car_system_template_t t;
    ivp_car_system_template_init(&t, 4, 2);
    t.car_body = b;
    ivp_allocator_t saved_allocator = *env->allocator;
    env->allocator->alloc = controller_parity_fail_alloc;
    ivp_airboat_state_t a;
    IVP_ASSERT_FALSE(ivp_airboat_init(&a, &t));
    *env->allocator = saved_allocator;

    IVP_ASSERT_FALSE(a.active);
    IVP_ASSERT_FALSE(ivp_controller_handle_is_valid(a.handle));
    IVP_ASSERT_NULL(a.gravity_controller);
    for (int i = 0; i < 3; i++) {
        IVP_ASSERT_TRUE(b->physical_core->speed.k[i] == speed_before.k[i]);
        IVP_ASSERT_TRUE(b->physical_core->rot_speed.k[i] == rot_speed_before.k[i]);
    }

    IVP_ASSERT_TRUE(ivp_airboat_init(&a, &t));
    ivp_airboat_destroy(&a);
    ivp_vec3r_set(&b->physical_core->speed, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    step(env, 1);
    IVP_ASSERT_NEAR(b->physical_core->speed.k[1], IVP_REAL(9.81) * DT60, IVP_REAL(1e-6));
    ivp_environment_destroy(env);
}

IVP_TEST(airboat_drag_uses_half_rho) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *b = make_ball(env, IVP_REAL(100.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_car_system_template_t t;
    ivp_car_system_template_init(&t, 4, 2); /* pontoons at the center, no ray caster */
    t.car_body = b;
    ivp_airboat_state_t a;
    IVP_ASSERT_TRUE(ivp_airboat_init(&a, &t)); /* clears the body speed (reference) */
    ivp_vec3r_set(&b->physical_core->speed, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_event_sim_t es = make_es(env);
    ivp_controller_airboat(&es, b, &a);
    /* F = 0.5 * 0.003 * 1000 * 3.53 * v^2; the steering terms cancel linearly */
    ivp_real_t drag = IVP_REAL(0.5) * IVP_REAL(0.003) * IVP_REAL(1000.0) * IVP_REAL(3.53) * IVP_REAL(100.0);
    ivp_vec3r_t v = total_speed(b);
    IVP_ASSERT_NEAR(v.k[0], IVP_REAL(10.0) - drag * DT60 / IVP_REAL(100.0), IVP_REAL(1e-3));
    ivp_airboat_destroy(&a);
    ivp_environment_destroy(env);
}

/* the airboat replaces the body's standard gravity by its own 7 m/s^2 */
IVP_TEST(airboat_own_gravity) {
    ivp_environment_t *env = make_env(true);
    ivp_object_t *b = make_ball(env, IVP_REAL(100.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_car_system_template_t t;
    ivp_car_system_template_init(&t, 4, 2);
    t.car_body = b;
    ivp_airboat_state_t a;
    IVP_ASSERT_TRUE(ivp_airboat_init(&a, &t));
    step(env, 6);
    /* free fall at 7 m/s^2 (no thrust, no pontoon contact; drag is tiny) */
    IVP_ASSERT_NEAR(b->physical_core->speed.k[1], IVP_REAL(7.0) * IVP_REAL(6.0) * DT60, IVP_REAL(2e-3));
    ivp_airboat_destroy(&a);
    ivp_environment_destroy(env);
}

/* ── Anchors and reference precision (IVP_Anchor, IVP_U_Float_Point) ──── */

static bool is_float_exact(ivp_real_t v)
{
    return (double)(float)v == (double)v;
}

static bool vec_is_float_exact(const ivp_vec3r_t *v)
{
    return is_float_exact(v->k[0]) && is_float_exact(v->k[1]) && is_float_exact(v->k[2]);
}

/* IVP_Anchor::init_anchor: object space -> world (cache object, includes
 * the object<->core shift) -> core space.  An object whose mass center is
 * not the object origin gets core_pos = anchor_os - mass_center. */
IVP_TEST(anchor_object_space_includes_mass_center_shift) {
    ivp_environment_t *env = make_env(false);
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.ball_radius = IVP_REAL(0.5);
    t.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
    t.rot_inertia_is_factor = false;
    t.use_mass_center_override = true;
    ivp_vec3r_set(&t.mass_center_override, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_set(&t.position, IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(3.0));
    t.orientation = ivp_quatr_identity();
    ivp_object_t *a = ivp_environment_create_object(env, &t, NULL);
    IVP_ASSERT_TRUE(a != NULL);
    IVP_ASSERT_FALSE(a->shift_core_f_object_is_zero);
    /* the core sits at the mass center */
    IVP_ASSERT_NEAR(a->physical_core->m_world_f_core_last_psi.pos.k[0], IVP_REAL(1.5), TOL);

    ivp_spring_state_t s;
    ivp_vec3r_t origin_os = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t world = ivp_vec3r(IVP_REAL(4.0), IVP_REAL(2.0), IVP_REAL(3.0));
    ivp_spring_state_setup(&s, a, &origin_os, NULL, &world, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(1.0));
    /* object origin in core space */
    IVP_ASSERT_NEAR(s.anchor_a_os.k[0], IVP_REAL(-0.5), 0.0);
    IVP_ASSERT_NEAR(s.anchor_a_os.k[1], IVP_REAL(0.0), 0.0);
    IVP_ASSERT_NEAR(s.anchor_a_os.k[2], IVP_REAL(0.0), 0.0);
    /* world anchor on the static object: float-rounded world coordinates */
    IVP_ASSERT_NEAR(s.anchor_b_os.k[0], IVP_REAL(4.0), 0.0);

    /* the spring pulls at the object origin (x = 1): length 3 */
    ivp_event_sim_t es = make_es(env);
    ivp_controller_spring(&es, a, &s);
    IVP_ASSERT_NEAR(s.current_length, IVP_REAL(3.0), 1e-6);
    ivp_environment_destroy(env);
}

/* A world anchor (NULL object) uses the static object frame: core_pos is the
 * IVP_U_Float_Point of the world coordinates. */
IVP_TEST(anchor_world_is_float_rounded) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t w = ivp_vec3r(IVP_REAL(0.1), IVP_REAL(0.2), IVP_REAL(0.3));
    ivp_vec3r_t z = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_force_actuator_state_t f;
    ivp_force_actuator_state_setup(&f, b, &z, NULL, &w, IVP_REAL(1.0), true, false);
    IVP_ASSERT_TRUE(f.anchor_b_os.k[0] == (ivp_real_t)0.1f);
    IVP_ASSERT_TRUE(f.anchor_b_os.k[1] == (ivp_real_t)0.2f);
    IVP_ASSERT_TRUE(f.anchor_b_os.k[2] == (ivp_real_t)0.3f);
    IVP_ASSERT_TRUE(f.force == (ivp_real_t)1.0f);
    ivp_environment_destroy(env);
}

/* Actuator pushes accumulate in IVP_U_Float_Point (async_push_core_ws):
 * speed_change / rot_speed_change stay float-exact. */
IVP_TEST(actuator_pushes_are_float_exact) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, IVP_REAL(1.3), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(0.7), IVP_REAL(1.7), IVP_REAL(0.3), IVP_REAL(0.1));
    ivp_vec3r_set(&a->physical_core->speed, (ivp_real_t)0.3f, (ivp_real_t)-0.2f, (ivp_real_t)0.1f);
    a->physical_core->rot_speed.k[2] = 0.7f;
    ivp_vec3r_t pa = ivp_vec3r(IVP_REAL(0.1), IVP_REAL(0.2), IVP_REAL(0.0));
    ivp_vec3r_t pb = ivp_vec3r(IVP_REAL(-0.1), IVP_REAL(0.0), IVP_REAL(0.3));
    ivp_spring_state_t s;
    ivp_spring_state_setup(&s, a, &pa, b, &pb, IVP_REAL(13.0), IVP_REAL(1.1), IVP_REAL(0.9));
    ivp_spring_set_rel_pos_damp(&s, IVP_REAL(0.3));
    IVP_ASSERT_TRUE(is_float_exact(s.stiffness) && is_float_exact(s.damping) &&
                    is_float_exact(s.rest_length) && is_float_exact(s.rel_pos_damp));
    IVP_ASSERT_TRUE(vec_is_float_exact(&s.anchor_a_os) && vec_is_float_exact(&s.anchor_b_os));
    ivp_event_sim_t es = make_es(env);
    ivp_controller_spring(&es, a, &s);
    IVP_ASSERT_TRUE(ivp_vec3r_length(&a->physical_core->speed_change) > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(vec_is_float_exact(&a->physical_core->speed_change));
    IVP_ASSERT_TRUE(vec_is_float_exact(&b->physical_core->speed_change));
    IVP_ASSERT_TRUE(vec_is_float_exact(&a->physical_core->rot_speed_change));
    IVP_ASSERT_TRUE(vec_is_float_exact(&b->physical_core->rot_speed_change));

    /* the float IVP_Solver_Core_Reaction keeps the core speed float */
    ivp_stiff_spring_state_t ss;
    ivp_stiff_spring_state_setup(&ss, a, &pa, b, &pb, IVP_REAL(0.5), IVP_REAL(0.3), IVP_REAL(0.9));
    ivp_controller_stiff_spring(&es, a, &ss);
    IVP_ASSERT_TRUE(ss.last_impulse != IVP_REAL(0.0));
    IVP_ASSERT_TRUE(vec_is_float_exact(&a->physical_core->speed));
    IVP_ASSERT_TRUE(vec_is_float_exact(&b->physical_core->speed));
    ivp_environment_destroy(env);
}

/* Mirrored springs give mirrored float pushes (the reference keeps the
 * springs scenario exactly symmetric until float accumulation order breaks
 * it). */
IVP_TEST(spring_mirrored_pushes_are_symmetric) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *l = make_ball(env, IVP_REAL(0.6), IVP_REAL(-1.2), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *r = make_ball(env, IVP_REAL(0.6), IVP_REAL(1.2), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t wl = ivp_vec3r(IVP_REAL(-3.1), IVP_REAL(0.1), IVP_REAL(0.0));
    ivp_vec3r_t wr = ivp_vec3r(IVP_REAL(3.1), IVP_REAL(0.1), IVP_REAL(0.0));
    ivp_vec3r_t ol = ivp_vec3r(IVP_REAL(-0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t orr = ivp_vec3r(IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_spring_state_t sl, sr;
    ivp_spring_state_setup(&sl, NULL, &wl, l, &ol, IVP_REAL(120.0), IVP_REAL(30.0), IVP_REAL(1.4));
    ivp_spring_state_setup(&sr, r, &orr, NULL, &wr, IVP_REAL(120.0), IVP_REAL(30.0), IVP_REAL(1.4));
    ivp_event_sim_t es = make_es(env);
    ivp_controller_spring(&es, l, &sl);
    ivp_controller_spring(&es, r, &sr);
    IVP_ASSERT_TRUE(l->physical_core->speed_change.k[0] == -r->physical_core->speed_change.k[0]);
    IVP_ASSERT_TRUE(l->physical_core->speed_change.k[1] == r->physical_core->speed_change.k[1]);
    IVP_ASSERT_TRUE(l->physical_core->rot_speed_change.k[2] == -r->physical_core->rot_speed_change.k[2]);
    ivp_environment_destroy(env);
}

/* IVP_Actuator_Torque: axis_in_core_coord_system is an IVP_U_Float_Point
 * normized once; rot_speed_out is IVP_FLOAT. */
IVP_TEST(torque_axis_is_float_normized_core_axis) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *b = make_ball(env, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t axis = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(2.0));
    ivp_torque_actuator_state_t t;
    ivp_torque_actuator_state_setup(&t, b, &axis, IVP_REAL(1.0), IVP_REAL(100.0));
    IVP_ASSERT_TRUE(vec_is_float_exact(&t.axis_os));
    IVP_ASSERT_NEAR(t.axis_os.k[0], IVP_REAL(1.0) / IVP_REAL(3.0), 1e-6);
    IVP_ASSERT_NEAR(t.axis_os.k[1], IVP_REAL(2.0) / IVP_REAL(3.0), 1e-6);
    b->physical_core->rot_speed.k[0] = 0.3f;
    ivp_event_sim_t es = make_es(env);
    ivp_controller_torque_actuator(&es, b, &t);
    IVP_ASSERT_TRUE(is_float_exact(t.current_speed));
    IVP_ASSERT_TRUE(vec_is_float_exact(&b->physical_core->rot_speed_change));
    ivp_environment_destroy(env);
}

int main(void) {
    IVP_REGISTER(rot_mot_reference_law_max_torque_zero_means_no_clip);
    IVP_REGISTER(torque_speed_limit_applies_in_both_directions);
    IVP_REGISTER(spring_rel_pos_damp_damps_full_relative_velocity);
    IVP_REGISTER(spring_only_on_stretch_and_break);
    IVP_REGISTER(spring_relative_values_use_virtual_mass);
    IVP_REGISTER(suspension_max_body_force_is_hard_clip);
    IVP_REGISTER(force_actuator_push_second_default_false);
    IVP_REGISTER(check_dist_boundary_is_outside_and_init_is_silent);
    IVP_REGISTER(check_dist_attached_uses_object_centers);
    IVP_REGISTER(stiff_spring_full_stiffness_reaches_rest_length_in_one_psi);
    IVP_REGISTER(golem_far_distance_calls_problem_without_force);
    IVP_REGISTER(floating_point_impulse_and_force_clip);
    IVP_REGISTER(world_friction_reference_desired_speed_clamp);
    IVP_REGISTER(constraint_ball_socket_ws_holds_pendulum);
    IVP_REGISTER(constraint_max_impulse_break_and_clip);
    IVP_REGISTER(constraint_hinge_ws_keeps_motion_in_plane);
    IVP_REGISTER(fixed_keyed_drives_object_to_target);
    IVP_REGISTER(buoyancy_half_submerged_box_impulse);
    IVP_REGISTER(multidimensional_interpolator_matches_reference);
    IVP_REGISTER(phantom_enter_exit_hysteresis);
    IVP_REGISTER(phantom_object_distance_is_exact_convex);
    IVP_REGISTER(raycast_car_rests_on_real_ground);
    IVP_REGISTER(airboat_registration_failure_is_atomic);
    IVP_REGISTER(airboat_drag_uses_half_rho);
    IVP_REGISTER(airboat_own_gravity);
    IVP_REGISTER(anchor_object_space_includes_mass_center_shift);
    IVP_REGISTER(anchor_world_is_float_rounded);
    IVP_REGISTER(actuator_pushes_are_float_exact);
    IVP_REGISTER(spring_mirrored_pushes_are_symmetric);
    IVP_REGISTER(torque_axis_is_float_normized_core_axis);
    IVP_RUN_ALL();
}
