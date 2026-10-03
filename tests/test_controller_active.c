/*
 * IVP Physics Engine - C17 Port
 * Active-value actuators and controller reference fixes
 *
 *   - IVP_Actuator_{Force,Torque,Rot_Mot,Spring}_Active and
 *     IVP_Controller_Stiff_Spring_Active bindings (ivp_actuator.cxx,
 *     ivp_actuator_spring.cxx, ivp_controller_stiff_spring.cxx)
 *   - rotation speed output terminals of torque / rot mot
 *   - IVP_Actuator_Extra (float cam / puck force)
 *   - stiff spring: non-simulated cores receive no impulse
 *   - golem: object/core shift, reset_time
 *   - motion: bound object wake-up, orientation normalization
 *   - suspension: inherited spring setters
 */

#include "ivp_test.h"

#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/controllers/ivp_actuator_force.h>
#include <ivp/controllers/ivp_actuator_torque.h>
#include <ivp/controllers/ivp_actuator_rot_mot.h>
#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/controllers/ivp_actuator_suspension.h>
#include <ivp/controllers/ivp_actuator_extra.h>
#include <ivp/controllers/ivp_controller_stiff_spring.h>
#include <ivp/controllers/ivp_controller_golem.h>
#include <ivp/controllers/ivp_controller_motion.h>
#include <ivp/utility/ivp_active_value.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>

#include <ivp/base/ivp_assert.h>

#include <math.h>
#include <string.h>

/* Reference IVP_ASSERTs that the tests trigger on purpose (the reference
 * debug build asserts on the same input): count the reports instead of
 * trapping.  Release builds compile the asserts out. */
static int g_assert_reports;
static void count_assert_report(const char *expr, const char *file, int line, const char *func,
                                const char *msg, void *userdata)
{
    (void)expr; (void)file; (void)line; (void)func; (void)msg; (void)userdata;
    g_assert_reports++;
}
#ifdef IVP_DEBUG_BUILD
#define EXPECTED_ASSERT_REPORTS(n) (n)
#else
#define EXPECTED_ASSERT_REPORTS(n) 0
#endif

#ifdef IVP_REAL_DOUBLE
  #define TOL 1e-9
#else
  #define TOL 1e-5f
#endif

#define DT60 (IVP_REAL(1.0) / IVP_REAL(60.0))

/* ── helpers ──────────────────────────────────────────────────────────── */

static ivp_environment_t *make_env(void)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = DT60;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

/* ball, radius 0.5, I = 0.4 m r^2; frozen (not simulated) unless awake */
static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t x, ivp_real_t y, ivp_real_t z, bool awake)
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
    t.start_in_simulation = awake;
    return ivp_environment_create_object(env, &t, NULL);
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

static bool simulated(const ivp_object_t *o) { return IVP_MTIS_SIMULATED(o->physical_core->movement_state); }

static ivp_real_t fval(double v) { return (ivp_real_t)(float)v; } /* get_float_value */

/* ── Force_Active ─────────────────────────────────────────────────────── */

IVP_TEST(force_active_follows_value_and_wakes) {
    ivp_environment_t *env = make_env();
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0), false);
    ivp_object_t *b = make_ball(env, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0), false);
    IVP_ASSERT_FALSE(simulated(a));
    ivp_vec3r_t z = ivp_vec3r_zero();
    ivp_force_actuator_state_t s;
    ivp_force_actuator_state_setup(&s, a, &z, b, &z, IVP_REAL(1.0), true, false);

    ivp_active_float_t *af = ivp_active_float_terminal_create(NULL, "force", 1.0);
    ivp_force_actuator_bind_active(&s, af);
    IVP_ASSERT_TRUE(s.active_float_force == af);
    IVP_ASSERT_EQ(af->listener_count, 1);
    IVP_ASSERT_EQ(af->ref_count, 2);
    /* unchanged value: set_force returns early, the actuator stays asleep */
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_FALSE(simulated(a));

    /* new value: active_float_changed -> set_force -> ensure_actuator_in_simulation */
    ivp_active_float_set(af, 0.1);
    IVP_ASSERT_NEAR(s.force, fval(0.1), 0.0);
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_TRUE(simulated(a));
    IVP_ASSERT_TRUE(simulated(b));

    ivp_force_actuator_unbind_active(&s);
    IVP_ASSERT_NULL(s.active_float_force);
    IVP_ASSERT_EQ(af->listener_count, 0);
    IVP_ASSERT_EQ(af->ref_count, 1);
    ivp_active_float_set(af, 9.0);
    IVP_ASSERT_NEAR(s.force, fval(0.1), 0.0);

    /* binding applies the current value at once */
    ivp_force_actuator_bind_active(&s, af);
    IVP_ASSERT_NEAR(s.force, IVP_REAL(9.0), 0.0);
    ivp_force_actuator_bind_active(&s, NULL); /* releases */
    IVP_ASSERT_EQ(af->ref_count, 1);
    ivp_active_float_remove_ref(af);
    ivp_environment_destroy(env);
}

IVP_TEST(force_active_derived_value) {
    ivp_environment_t *env = make_env();
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0), true);
    ivp_vec3r_t z = ivp_vec3r_zero();
    ivp_force_actuator_state_t s;
    ivp_force_actuator_state_setup(&s, a, &z, NULL, &z, IVP_REAL(0.0), true, false);
    ivp_active_float_t *t = ivp_active_float_terminal_create(NULL, "t", 2.0);
    ivp_active_float_t *k = ivp_active_float_terminal_create(NULL, "k", 3.0);
    ivp_active_float_t *m = ivp_active_float_mult_create(NULL, "m", t, k);
    ivp_force_actuator_bind_active(&s, m);
    IVP_ASSERT_NEAR(s.force, IVP_REAL(6.0), 0.0);
    ivp_active_float_set(t, 4.0);
    IVP_ASSERT_NEAR(s.force, IVP_REAL(12.0), 0.0);
    ivp_force_actuator_unbind_active(&s);
    ivp_active_float_remove_ref(m);
    ivp_active_float_remove_ref(k);
    ivp_active_float_remove_ref(t);
    ivp_environment_destroy(env);
}

/* ── Torque_Active + rotation speed output ────────────────────────────── */

IVP_TEST(torque_active_and_rotation_speed_out) {
    ivp_environment_t *env = make_env();
    ivp_object_t *b = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0), true);
    ivp_torque_actuator_state_t t;
    ivp_vec3r_t axis = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_torque_actuator_state_setup(&t, b, &axis, IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_active_float_t *af_t = ivp_active_float_terminal_create(NULL, "torque", 2.0);
    ivp_active_float_t *af_m = ivp_active_float_terminal_create(NULL, "max", 5.0);
    ivp_torque_actuator_bind_active(&t, af_t, af_m);
    IVP_ASSERT_NEAR(t.torque, IVP_REAL(2.0), 0.0);
    IVP_ASSERT_NEAR(t.max_rotation_speed, IVP_REAL(5.0), 0.0);
    ivp_active_float_set(af_m, 0.3);
    IVP_ASSERT_NEAR(t.max_rotation_speed, fval(0.3), 0.0);
    ivp_active_float_set(af_m, 5.0);

    ivp_active_float_t *out = ivp_active_float_terminal_create(NULL, "speed", 0.0);
    ivp_torque_actuator_set_rotation_speed_out(&t, out);
    IVP_ASSERT_EQ(out->ref_count, 2);
    b->physical_core->rot_speed.k[1] = 1.25f;
    ivp_event_sim_t es = make_es(env);
    ivp_controller_torque_actuator(&es, b, &t);
    IVP_ASSERT_NEAR(ivp_active_float_get(out), 1.25, 0.0);
    /* torque 0: no output update (reference returns before) */
    ivp_active_float_set(af_t, 0.0);
    b->physical_core->rot_speed.k[1] = 2.0f;
    ivp_controller_torque_actuator(&es, b, &t);
    IVP_ASSERT_NEAR(ivp_active_float_get(out), 1.25, 0.0);

    ivp_torque_actuator_unbind_active(&t);
    IVP_ASSERT_EQ(af_t->listener_count, 0);
    IVP_ASSERT_EQ(af_m->listener_count, 0);
    IVP_ASSERT_EQ(out->ref_count, 1);
    IVP_ASSERT_NULL(t.active_float_rotation_speed_out);
    ivp_active_float_remove_ref(out);
    ivp_active_float_remove_ref(af_m);
    ivp_active_float_remove_ref(af_t);
    ivp_environment_destroy(env);
}

/* ── Rot_Mot_Active ───────────────────────────────────────────────────── */

IVP_TEST(rot_mot_active_binding) {
    ivp_environment_t *env = make_env();
    ivp_object_t *b = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0), true);
    ivp_rot_mot_state_t m;
    ivp_vec3r_t a0 = ivp_vec3r_zero();
    ivp_vec3r_t a1 = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_rot_mot_state_setup(&m, b, &a0, b, &a1, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    ivp_active_float_t *speed = ivp_active_float_terminal_create(NULL, "speed", 10.0);
    ivp_active_float_t *power = ivp_active_float_terminal_create(NULL, "power", 1.0);
    ivp_active_float_t *torque = ivp_active_float_terminal_create(NULL, "torque", 0.0);
    ivp_rot_mot_bind_active(&m, speed, power, torque);
    IVP_ASSERT_NEAR(m.max_rotation_speed, IVP_REAL(10.0), 0.0);
    IVP_ASSERT_NEAR(m.power, IVP_REAL(1.0), 0.0);
    IVP_ASSERT_NEAR(m.max_torque, IVP_REAL(0.0), 0.0);
    ivp_active_float_set(torque, 2.0);
    IVP_ASSERT_NEAR(m.max_torque, IVP_REAL(2.0), 0.0);
    ivp_active_float_set(power, 0.7);
    IVP_ASSERT_NEAR(m.power, fval(0.7), 0.0);

    ivp_active_float_t *out = ivp_active_float_terminal_create(NULL, "out", -1.0);
    ivp_rot_mot_set_rotation_speed_out(&m, out);
    b->physical_core->rot_speed.k[1] = 0.5f;
    ivp_event_sim_t es = make_es(env);
    ivp_controller_rot_mot(&es, b, &m);
    IVP_ASSERT_NEAR(ivp_active_float_get(out), 0.5, 0.0);
    /* opposite direction: domega cleared before the output */
    b->physical_core->rot_speed.k[1] = -0.5f;
    ivp_controller_rot_mot(&es, b, &m);
    IVP_ASSERT_NEAR(ivp_active_float_get(out), 0.0, 0.0);

    ivp_rot_mot_unbind_active(&m);
    IVP_ASSERT_EQ(speed->listener_count + power->listener_count + torque->listener_count, 0);
    IVP_ASSERT_EQ(out->ref_count, 1);
    ivp_active_float_remove_ref(out);
    ivp_active_float_remove_ref(torque);
    ivp_active_float_remove_ref(power);
    ivp_active_float_remove_ref(speed);
    ivp_environment_destroy(env);
}

/* ── Spring_Active ────────────────────────────────────────────────────── */

IVP_TEST(spring_active_binding_uses_values_factor) {
    ivp_environment_t *env = make_env();
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0), true);
    ivp_object_t *b = make_ball(env, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0), true);
    ivp_vec3r_t z = ivp_vec3r_zero();
    ivp_spring_state_t s;
    ivp_spring_state_setup(&s, a, &z, b, &z, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_spring_state_make_relative(&s); /* two 1 kg cores at their centers: factor 0.5 */
    IVP_ASSERT_NEAR(s.spring_values_factor, IVP_REAL(0.5), TOL);

    ivp_active_float_t *len = ivp_active_float_terminal_create(NULL, "len", 1.5);
    ivp_active_float_t *k = ivp_active_float_terminal_create(NULL, "k", 10.0);
    ivp_active_float_t *d = ivp_active_float_terminal_create(NULL, "d", 4.0);
    ivp_active_float_t *r = ivp_active_float_terminal_create(NULL, "r", 2.0);
    ivp_spring_bind_active(&s, len, k, d, r);
    IVP_ASSERT_NEAR(s.rest_length, IVP_REAL(1.5), 0.0);
    IVP_ASSERT_NEAR(s.stiffness, IVP_REAL(10.0) * s.spring_values_factor, TOL);
    IVP_ASSERT_NEAR(s.damping, IVP_REAL(4.0) * s.spring_values_factor, TOL);
    IVP_ASSERT_NEAR(s.rel_pos_damp, IVP_REAL(2.0) * s.spring_values_factor, TOL);
    ivp_active_float_set(k, 20.0);
    IVP_ASSERT_NEAR(s.stiffness, IVP_REAL(20.0) * s.spring_values_factor, TOL);
    ivp_active_float_set(len, 0.25);
    IVP_ASSERT_NEAR(s.rest_length, IVP_REAL(0.25), 0.0);
    ivp_active_float_set(r, 0.0);
    IVP_ASSERT_NEAR(s.rel_pos_damp, IVP_REAL(0.0), 0.0);

    /* rebinding releases the old dependencies */
    ivp_spring_bind_active(&s, NULL, k, NULL, NULL);
    IVP_ASSERT_EQ(len->listener_count, 0);
    IVP_ASSERT_EQ(k->listener_count, 1);
    ivp_spring_unbind_active(&s);
    IVP_ASSERT_EQ(k->listener_count, 0);
    ivp_active_float_remove_ref(len);
    ivp_active_float_remove_ref(k);
    ivp_active_float_remove_ref(d);
    ivp_active_float_remove_ref(r);
    ivp_environment_destroy(env);
}

/* ── Stiff_Spring_Active ──────────────────────────────────────────────── */

IVP_TEST(stiff_spring_active_constructor_does_not_wake) {
    ivp_environment_t *env = make_env();
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0), false);
    ivp_object_t *b = make_ball(env, IVP_REAL(3.0), IVP_REAL(0.0), IVP_REAL(0.0), false);
    ivp_vec3r_t z = ivp_vec3r_zero();
    ivp_stiff_spring_state_t s;
    ivp_stiff_spring_state_setup(&s, a, &z, b, &z, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(3.0));
    ivp_active_float_t *len = ivp_active_float_terminal_create(NULL, "len", 2.0);
    ivp_active_float_t *k = ivp_active_float_terminal_create(NULL, "k", 0.5);
    ivp_stiff_spring_bind_active(&s, len, k, NULL);
    IVP_ASSERT_NEAR(s.rest_length, IVP_REAL(2.0), 0.0);
    IVP_ASSERT_NEAR(s.stiffness, IVP_REAL(0.5), 0.0);
    IVP_ASSERT_NEAR(s.damping, (ivp_real_t)0.1f, 0.0); /* IVP_FLOAT member */
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_FALSE(simulated(a)); /* values copied, no ensure_actuator_in_simulation */

    ivp_active_float_set(len, 2.5); /* set_len wakes */
    IVP_ASSERT_NEAR(s.rest_length, IVP_REAL(2.5), 0.0);
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_TRUE(simulated(a));
    ivp_stiff_spring_unbind_active(&s);
    IVP_ASSERT_EQ(len->listener_count + k->listener_count, 0);
    ivp_active_float_remove_ref(len);
    ivp_active_float_remove_ref(k);
    ivp_environment_destroy(env);
}

/* ── Stiff spring: reference nulls non-simulated cores ───────────────── */

IVP_TEST(stiff_spring_frozen_partner_receives_no_impulse) {
    ivp_environment_t *env = make_env();
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0), true);
    ivp_object_t *b = make_ball(env, IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0), false);
    IVP_ASSERT_TRUE(simulated(a));
    IVP_ASSERT_FALSE(simulated(b));
    ivp_vec3r_t z = ivp_vec3r_zero();
    ivp_stiff_spring_state_t s;
    ivp_stiff_spring_state_setup(&s, a, &z, b, &z, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_event_sim_t es = make_es(env);
    ivp_controller_stiff_spring(&es, a, &s);
    /* only pc0: a = (1 - 2) * 60 * 1 = -60; tpm00 = 1/m0 = 1; the impulse
     * acts along dir = pos0 - pos1 = -x, so a moves towards b */
    IVP_ASSERT_NEAR(s.last_impulse, IVP_REAL(-60.0), IVP_REAL(1e-4));
    IVP_ASSERT_NEAR(a->physical_core->speed.k[0] + a->physical_core->speed_change.k[0], IVP_REAL(60.0), IVP_REAL(1e-4));
    IVP_ASSERT_NEAR(b->physical_core->speed.k[0] + b->physical_core->speed_change.k[0], IVP_REAL(0.0), 0.0);

    /* pinned cores are nulled as well */
    ivp_vec3r_set_zero(&a->physical_core->speed);
    ivp_vec3r_set_zero(&a->physical_core->speed_change);
    b->physical_core->movement_state = a->physical_core->movement_state;
    b->physical_core->pinned = true;
    ivp_controller_stiff_spring(&es, a, &s);
    IVP_ASSERT_NEAR(b->physical_core->speed.k[0] + b->physical_core->speed_change.k[0], IVP_REAL(0.0), 0.0);
    b->physical_core->pinned = false;
    b->physical_core->movement_state = IVP_MT_NOT_SIM;
    ivp_environment_destroy(env);
}

/* ── Golem: object origin target (shift_core_f_object), reset_time ───── */

IVP_TEST(golem_target_is_object_origin) {
    ivp_environment_t *env = make_env();
    ivp_object_t *b = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0), true);
    /* object origin 1 m along core x */
    ivp_vec3r_set(&b->shift_core_f_object, IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    b->shift_core_f_object_is_zero = false;

    ivp_golem_state_t g;
    ivp_golem_state_init(&g);
    ivp_golem_state_setup(&g, b);
    ivp_vec3r_t origin = b->physical_core->m_world_f_core_last_psi.pos;
    origin.k[0] += IVP_REAL(1.0);
    ivp_vec3r_t zero = ivp_vec3r_zero();
    ivp_golem_set_prime_position(&g, &origin, &zero, ivp_environment_get_time(env));
    ivp_event_sim_t es = make_es(env);
    ivp_controller_golem(&es, b, &g);
    /* the object origin is at the target: no translation force */
    IVP_ASSERT_NEAR(b->physical_core->speed.k[0], IVP_REAL(0.0), TOL);
    IVP_ASSERT_NEAR(b->physical_core->speed.k[1], IVP_REAL(0.0), TOL);

    /* prime velocity extrapolation uses (float) now - t_prime; reset_time
     * shifts the prime position time */
    g.time_of_prime_position = IVP_REAL(5.0);
    ivp_golem_reset_time(&g, 2.0);
    IVP_ASSERT_NEAR(g.time_of_prime_position, 3.0, 0.0);
    ivp_environment_destroy(env);
}

/* ── Motion: bound object, wake-up and orientation normalization ──────── */

IVP_TEST(motion_bound_object_wakes_and_normalizes) {
    ivp_environment_t *env = make_env();
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0), false);
    ivp_object_t *other = make_ball(env, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0), true);
    ivp_motion_state_t m;
    ivp_motion_state_init(&m);
    ivp_motion_state_setup(&m, a);

    ivp_vec3r_t target = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_motion_state_set_target_position(&m, &target);
    IVP_ASSERT_EQ((int)m.linear_mode, (int)IVP_MOTION_TARGET);
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_TRUE(simulated(a)); /* ensure_core_in_simulation */

    /* orientation: normalized, invalid input ignored */
    ivp_quatr_t q;
    q.x = IVP_REAL(0.0); q.y = IVP_REAL(0.0); q.z = IVP_REAL(0.0); q.w = IVP_REAL(2.0);
    ivp_motion_state_set_target_orientation(&m, &q);
    IVP_ASSERT_NEAR(m.target_q_world_f_core.w, IVP_REAL(1.0), 0.0);
    ivp_quatr_t bad;
    bad.x = bad.y = bad.z = bad.w = IVP_REAL(0.0);
    q.x = IVP_REAL(0.6); q.y = IVP_REAL(0.0); q.z = IVP_REAL(0.0); q.w = IVP_REAL(0.8);
    ivp_motion_state_set_target_orientation(&m, &q);
    /* set_target_q_world_f_core: IVP_ASSERT(0) and ignore (ivp_controller_motion.cxx:198) */
    g_assert_reports = 0;
    ivp_set_assert_handler(count_assert_report, NULL);
    ivp_motion_state_set_target_orientation(&m, &bad);
    ivp_set_assert_handler(NULL, NULL);
    IVP_ASSERT_EQ(g_assert_reports, EXPECTED_ASSERT_REPORTS(1));
    IVP_ASSERT_NEAR(m.target_q_world_f_core.x, IVP_REAL(0.6), TOL);
    IVP_ASSERT_NEAR(m.target_q_world_f_core.w, IVP_REAL(0.8), TOL);

    /* the bound controller acts on its object only */
    ivp_event_sim_t es = make_es(env);
    ivp_controller_motion(&es, other, &m);
    IVP_ASSERT_NEAR(other->physical_core->speed.k[0], IVP_REAL(0.0), 0.0);
    ivp_controller_motion(&es, a, &m);
    IVP_ASSERT_TRUE(a->physical_core->speed.k[0] > IVP_REAL(0.0));

    /* set_target_object_position_ws removes the core/object shift */
    ivp_vec3r_set(&a->shift_core_f_object, IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    a->shift_core_f_object_is_zero = false;
    ivp_quatr_t id = ivp_quatr_identity();
    ivp_vec3r_t center = ivp_vec3r(IVP_REAL(3.0), IVP_REAL(3.0), IVP_REAL(3.0));
    ivp_motion_state_set_target_object_position(&m, a, &id, &center);
    IVP_ASSERT_NEAR(m.target_pos_ws.k[1], IVP_REAL(1.0), TOL);
    IVP_ASSERT_NEAR(m.target_pos_ws.k[0], IVP_REAL(3.0), TOL);
    ivp_environment_destroy(env);
}

/* ── Motion: reference default target (origin / identity) ─────────────── */

IVP_TEST(motion_drives_toward_default_target) {
    ivp_environment_t *env = make_env();
    ivp_object_t *a = make_ball(env, IVP_REAL(2.0), IVP_REAL(-1.0), IVP_REAL(0.5), false);
    ivp_motion_state_t m;
    ivp_motion_state_init(&m);
    ivp_motion_state_setup(&m, a);
    /* IVP_Controller_Motion::set_target_position_ws: an unchanged target
     * (the constructor's origin) returns early and does not wake */
    ivp_vec3r_t origin = ivp_vec3r_zero();
    ivp_motion_state_set_target_position(&m, &origin);
    ivp_quatr_t id = ivp_quatr_identity();
    ivp_motion_state_set_target_orientation(&m, &id);
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_FALSE(simulated(a));

    /* awake, the controller pulls the object toward the origin without any
     * set_target call */
    ivp_environment_add_controller(env, IVP_CP_MOTION, ivp_controller_motion, &m);
    ivp_environment_ensure_core_in_simulation(env, a);
    for (int i = 0; i < 30; i++) ivp_environment_simulate_one_psi(env);
    const ivp_vec3r_t *p = &a->physical_core->pos_world_f_core_last_psi;
    IVP_ASSERT_TRUE(p->k[0] < IVP_REAL(1.5));
    IVP_ASSERT_TRUE(p->k[1] > IVP_REAL(-0.75));
    IVP_ASSERT_TRUE(p->k[2] < IVP_REAL(0.4));
    IVP_ASSERT_TRUE(p->k[0] > IVP_REAL(-0.5)); /* damped, no overshoot past the origin */
    ivp_environment_destroy(env);
}

/* ── Suspension: inherited spring setters ─────────────────────────────── */

IVP_TEST(suspension_inherited_setters) {
    ivp_suspension_state_t s;
    ivp_suspension_state_init(&s);
    s.spring_values_factor = IVP_REAL(2.0);
    ivp_suspension_set_constant(&s, IVP_REAL(3.0));
    IVP_ASSERT_NEAR(s.stiffness, IVP_REAL(6.0), 0.0);
    ivp_suspension_set_damp(&s, IVP_REAL(1.5));
    IVP_ASSERT_NEAR(s.damping, IVP_REAL(3.0), 0.0);
    ivp_suspension_set_len(&s, IVP_REAL(0.4));
    IVP_ASSERT_NEAR(s.rest_length, (ivp_real_t)0.4f, 0.0); /* IVP_FLOAT member */
    /* set_spring_damp_compression compares the stored (scaled) value */
    ivp_suspension_set_damping_compression(&s, IVP_REAL(1.0));
    IVP_ASSERT_NEAR(s.damping_compression, IVP_REAL(2.0), 0.0);
    ivp_suspension_set_damping_compression(&s, IVP_REAL(2.0)); /* == stored: unchanged */
    IVP_ASSERT_NEAR(s.damping_compression, IVP_REAL(2.0), 0.0);
}

/* ── Actuator_Extra ───────────────────────────────────────────────────── */

IVP_TEST(actuator_extra_dependencies_and_props) {
    ivp_environment_t *env = make_env();
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0), false);
    ivp_object_t *b = make_ball(env, IVP_REAL(-1.0), IVP_REAL(0.0), IVP_REAL(0.0), false);
    ivp_active_float_t *h = ivp_active_float_terminal_create(NULL, "h", 2.0);
    ivp_active_float_t *th = ivp_active_float_terminal_create(NULL, "th", 1.0);
    ivp_active_float_t *dist = ivp_active_float_terminal_create(NULL, "dist", 5.0);
    ivp_active_float_t *speed = ivp_active_float_terminal_create(NULL, "speed", 0.5);
    ivp_active_float_t *force = ivp_active_float_terminal_create(NULL, "force", 7.0);
    ivp_active_float_t *pf = ivp_active_float_terminal_create(NULL, "pf", 1.0);

    ivp_template_extra_t t;
    ivp_template_extra_init(&t);
    IVP_ASSERT_FALSE(t.info.is_float_cam);
    IVP_ASSERT_NULL(t.info.active_float_force);
    t.objects[0] = a;
    t.objects[1] = b;
    t.info.is_float_cam = true;
    t.info.mod_fc_height = h;
    t.info.mod_fc_target_height = th;
    t.info.mod_fc_dist = dist;
    t.info.mod_fc_speed = speed;
    t.info.active_float_force = force;
    t.info.mod_pf_forward = pf; /* not a dependency in the reference */

    ivp_actuator_extra_state_t x;
    ivp_actuator_extra_init(&x, &t);
    IVP_ASSERT_EQ(h->listener_count + th->listener_count + dist->listener_count + speed->listener_count, 4);
    IVP_ASSERT_EQ(force->listener_count, 1);
    IVP_ASSERT_EQ(pf->listener_count, 0);
    IVP_ASSERT_NEAR(ivp_actuator_extra_get_force(&x), 7.0, 0.0);
    ivp_active_float_set(force, 3.0); /* active_float_changed does nothing */
    IVP_ASSERT_NEAR(ivp_actuator_extra_get_force(&x), 3.0, 0.0);

    double fh, fth, fd, fs;
    const ivp_vec3d_t *cam = ivp_actuator_extra_get_float_cam_props(&x, &fh, &fth, &fd, &fs);
    IVP_ASSERT_NEAR(fh, 2.0, 0.0);
    IVP_ASSERT_NEAR(fth, 1.0, 0.0);
    IVP_ASSERT_NEAR(fd, 5.0, 0.0);
    IVP_ASSERT_NEAR(fs, 0.5, 0.0);
    IVP_ASSERT_NEAR(cam->k[1], -100.0, 0.0);

    /* the constructor wakes the actuator */
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_TRUE(simulated(a));

    /* float cam: dir = (1,0,0); wanted (5,-2,0); s = speed * dt */
    ivp_event_sim_t es = make_es(env);
    ivp_vec3r_t p0 = a->physical_core->m_world_f_core_last_psi.pos;
    ivp_controller_actuator_extra(&es, a, &x);
    double s = 0.5 * (double)DT60;
    IVP_ASSERT_NEAR(x.current_float_cam_pos.k[0], (1.0 - s) * 0.0 + s * ((double)p0.k[0] + 5.0), 1e-9);
    IVP_ASSERT_NEAR(x.current_float_cam_pos.k[1], (1.0 - s) * -100.0 + s * ((double)p0.k[1] - 2.0), 1e-9);
    IVP_ASSERT_TRUE(x.current_vals_are_set);
    IVP_ASSERT_NEAR(x.current_look_point.k[1], (double)p0.k[1] - 1.0, 1e-9);

    ivp_transform_r_t m;
    ivp_actuator_extra_calc_float_cam_matrix(&x, &m);
    for (int c = 0; c < 3; c++) {
        double len = 0.0;
        for (int r = 0; r < 3; r++) len += (double)m.rot.rows[r].k[c] * (double)m.rot.rows[r].k[c];
        IVP_ASSERT_NEAR(len, 1.0, 1e-5);
    }
    IVP_ASSERT_NEAR(m.pos.k[1], x.current_float_cam_pos.k[1], 1e-5);

    ivp_actuator_extra_destroy(&x);
    IVP_ASSERT_EQ(h->listener_count + th->listener_count + dist->listener_count + speed->listener_count +
                  force->listener_count, 0);
    ivp_active_float_remove_ref(h);
    ivp_active_float_remove_ref(th);
    ivp_active_float_remove_ref(dist);
    ivp_active_float_remove_ref(speed);
    ivp_active_float_remove_ref(force);
    ivp_active_float_remove_ref(pf);
    ivp_environment_destroy(env);
}

IVP_TEST(actuator_extra_puck_force_is_zero_force) {
    ivp_environment_t *env = make_env();
    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0), true);
    ivp_object_t *b = make_ball(env, IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0), true);
    ivp_template_extra_t t;
    ivp_template_extra_init(&t);
    t.objects[0] = a;
    t.objects[1] = b; /* vertical pair: horizontal direction degenerates */
    t.info.is_puck_force = 1;
    ivp_actuator_extra_state_t x;
    ivp_actuator_extra_init(&x, &t);
    ivp_environment_announce_dependent_controller(env, IVP_CP_ACTUATOR, ivp_controller_actuator_extra, &x,
                                                  ivp_actuator_extra_discovery);
    for (int i = 0; i < 5; i++) ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_NEAR(ivp_vec3r_length(&a->physical_core->speed), IVP_REAL(0.0), 0.0);
    IVP_ASSERT_NEAR(ivp_vec3r_length(&b->physical_core->speed), IVP_REAL(0.0), 0.0);
    IVP_ASSERT_TRUE(a->physical_core->speed.k[0] == a->physical_core->speed.k[0]); /* no NaN */
    ivp_actuator_extra_destroy(&x);
    ivp_environment_destroy(env);
}

int main(void) {
    IVP_REGISTER(force_active_follows_value_and_wakes);
    IVP_REGISTER(force_active_derived_value);
    IVP_REGISTER(torque_active_and_rotation_speed_out);
    IVP_REGISTER(rot_mot_active_binding);
    IVP_REGISTER(spring_active_binding_uses_values_factor);
    IVP_REGISTER(stiff_spring_active_constructor_does_not_wake);
    IVP_REGISTER(stiff_spring_frozen_partner_receives_no_impulse);
    IVP_REGISTER(golem_target_is_object_origin);
    IVP_REGISTER(motion_bound_object_wakes_and_normalizes);
    IVP_REGISTER(motion_drives_toward_default_target);
    IVP_REGISTER(suspension_inherited_setters);
    IVP_REGISTER(actuator_extra_dependencies_and_props);
    IVP_REGISTER(actuator_extra_puck_force_is_zero_force);
    IVP_RUN_ALL();
}
