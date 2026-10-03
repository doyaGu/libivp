/* test_controller_factory.c — IVP_Controller_Factory (ivp_controller_factory.h)
 *
 * Template defaults, anchor conversion (IVP_Template_Anchor ->
 * IVP_Anchor::init_anchor), registration as the reference constructors do
 * (announced actuators / constraint, motion controller on one core, inert
 * stabilizer, check dist without controller), active value bindings and the
 * ownership rules (delete, deletion with an anchor object / the
 * environment).
 */
#include "ivp_test.h"

#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/controllers/ivp_controller_factory.h>
#include <ivp/utility/ivp_active_value.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>

#include <math.h>
#include <string.h>

#ifdef IVP_REAL_DOUBLE
  #define TOL 1e-9
#else
  #define TOL 1e-5f
#endif

static ivp_environment_t *make_env(bool gravity)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = IVP_REAL(1.0) / IVP_REAL(60.0);
    if (!gravity) ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

/* ball radius 0.5, rotated so that object and world space differ */
static ivp_object_t *make_ball(ivp_environment_t *env, double x, double y, double z, bool unmoveable)
{
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.ball_radius = IVP_REAL(0.5);
    t.mass = IVP_REAL(2.0);
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(0.2), IVP_REAL(0.3), IVP_REAL(0.25));
    t.rot_inertia_is_factor = false;
    t.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&t.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    t.physical_unmoveable = unmoveable;
    t.orientation = ivp_quatr_make(0.0, 0.25881904510252074, 0.0, 0.9659258262890683);
    ivp_vec3r_set(&t.position, (ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
    return ivp_environment_create_object(env, &t, NULL);
}

/* library internal (src/controllers/ivp_controller_system.h) */
int ivp_controller_system_count(const struct ivp_controller_system *sys);

static int controller_count(const ivp_environment_t *env)
{
    return ivp_controller_system_count(env->controller_system);
}

static void anchor_os(ivp_template_anchor_t *a, ivp_object_t *obj, double x, double y, double z)
{
    ivp_vec3r_t p = ivp_vec3r((ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
    ivp_template_anchor_set_position_os(a, obj, &p);
}

static bool vec_eq(const ivp_vec3r_t *a, const ivp_vec3r_t *b)
{
    return a->k[0] == b->k[0] && a->k[1] == b->k[1] && a->k[2] == b->k[2];
}

IVP_TEST(factory_template_defaults) {
    ivp_template_force_t f;
    ivp_template_force_init(&f);
    IVP_ASSERT_TRUE(f.push_first_object);
    IVP_ASSERT_FALSE(f.push_second_object);
    IVP_ASSERT_TRUE(f.force == 0.0f && f.anchors[0] == NULL && f.active_float_force == NULL);
    ivp_template_spring_t s;
    ivp_template_spring_init(&s);
    IVP_ASSERT_TRUE(s.break_max_len == IVP_FLOAT_MAX);
    IVP_ASSERT_EQ((int)s.max_len_exceed_type, (int)IVP_SFE_NONE);
    ivp_template_suspension_t su;
    ivp_template_suspension_init(&su);
    IVP_ASSERT_TRUE(su.spring.break_max_len == 10e8f);
    IVP_ASSERT_TRUE(su.max_body_force == 0.0f && su.spring_dampening_compression == 0.0f);
    ivp_template_controller_motion_t m;
    ivp_template_controller_motion_init(&m);
    IVP_ASSERT_TRUE(m.force_factor == 0.8f && m.damp_factor == 1.0f);
    IVP_ASSERT_TRUE(m.torque_factor == 0.8f && m.angular_damp_factor == 1.0f);
    IVP_ASSERT_TRUE(m.max_torque == 1e6f && m.max_translation_force.k[1] == 1e6f);
}

/* the template anchors give the same core positions as the os setup path */
IVP_TEST(factory_force_matches_setup_and_announces) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, 0.0, 0.0, 0.0, false);
    ivp_object_t *b = make_ball(env, 4.0, 1.0, -1.0, false);
    ivp_template_anchor_t an[2];
    anchor_os(&an[0], a, 0.1, 0.2, 0.3);
    anchor_os(&an[1], b, -0.3, 0.1, 0.05);
    ivp_template_force_t tf;
    ivp_template_force_init(&tf);
    tf.anchors[0] = &an[0];
    tf.anchors[1] = &an[1];
    tf.force = 3.25f;
    tf.client_data = &tf;
    const int n0 = controller_count(env);
    ivp_force_actuator_state_t *f = ivp_controller_factory_create_force(env, &tf);
    IVP_ASSERT_NOT_NULL(f);
    IVP_ASSERT_EQ(controller_count(env), n0 + 1);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(ivp_controller_factory_get_handle(env, f)));
    IVP_ASSERT_TRUE(ivp_controller_factory_get_client_data(env, f) == &tf);

    ivp_force_actuator_state_t ref;
    ivp_vec3r_t pa = ivp_vec3r(IVP_REAL(0.1), IVP_REAL(0.2), IVP_REAL(0.3));
    ivp_vec3r_t pb = ivp_vec3r(IVP_REAL(-0.3), IVP_REAL(0.1), IVP_REAL(0.05));
    ivp_force_actuator_state_setup(&ref, a, &pa, b, &pb, IVP_REAL(3.25), true, false);
    IVP_ASSERT_TRUE(vec_eq(&f->anchor_a_os, &ref.anchor_a_os));
    IVP_ASSERT_TRUE(vec_eq(&f->anchor_b_os, &ref.anchor_b_os));
    IVP_ASSERT_TRUE(f->obj_a == a && f->obj_b == b);
    IVP_ASSERT_TRUE(f->force == ref.force && f->push_first_object && !f->push_second_object);

    for (int i = 0; i < 10; i++) ivp_environment_simulate_one_psi(env);
    /* force from b to a pushes a away from b (-x) */
    IVP_ASSERT_TRUE(a->physical_core->speed.k[0] < IVP_REAL(0.0));

    IVP_ASSERT_TRUE(ivp_controller_factory_delete(env, f));
    IVP_ASSERT_EQ(controller_count(env), n0);
    IVP_ASSERT_FALSE(ivp_controller_factory_delete(env, f)); /* already deleted */
    ivp_environment_destroy(env);
}

/* world anchors: NULL or the static object */
IVP_TEST(factory_force_world_anchor) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, 0.0, 0.0, 0.0, false);
    ivp_template_anchor_t an[2];
    anchor_os(&an[0], a, 0.0, 0.0, 0.0);
    ivp_vec3r_t w = ivp_vec3r(IVP_REAL(-5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_template_anchor_set_position_ws(&an[1], ivp_environment_get_static_object(env), &w);
    ivp_template_force_t tf;
    ivp_template_force_init(&tf);
    tf.anchors[0] = &an[0];
    tf.anchors[1] = &an[1];
    tf.force = 1.0f;
    ivp_force_actuator_state_t *f = ivp_controller_factory_create_force(env, &tf);
    IVP_ASSERT_NOT_NULL(f);
    IVP_ASSERT_TRUE(f->obj_b == NULL); /* static object -> world anchor */
    IVP_ASSERT_NEAR(f->anchor_b_os.k[0], IVP_REAL(-5.0), 0.0);
    for (int i = 0; i < 10; i++) ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_TRUE(a->physical_core->speed.k[0] > IVP_REAL(0.0)); /* pushed away from (-5, 0, 0) */
    ivp_environment_destroy(env); /* frees the factory controller */
}

IVP_TEST(factory_torque_axis_active_values) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, 0.0, 0.0, 0.0, false);
    ivp_template_anchor_t an[2];
    anchor_os(&an[0], a, 0.0, 0.0, 0.0);
    anchor_os(&an[1], a, 0.0, 2.0, 0.0);
    ivp_active_float_t *af_torque = ivp_active_float_terminal_create(NULL, "torque", 0.0);
    ivp_active_float_t *af_speed = ivp_active_float_terminal_create(NULL, "speed", 50.0);
    ivp_active_float_t *speed_out = ivp_active_float_terminal_create(NULL, "speed_out", 0.0);
    ivp_template_torque_t tt;
    ivp_template_torque_init(&tt);
    tt.anchors[0] = &an[0];
    tt.anchors[1] = &an[1];
    tt.torque = 7.0f; /* overridden by the active float */
    tt.active_float_torque = af_torque;
    tt.active_float_max_rotation_speed = af_speed;
    tt.active_float_rotation_speed_out = speed_out;
    ivp_torque_actuator_state_t *t = ivp_controller_factory_create_torque(env, &tt);
    IVP_ASSERT_NOT_NULL(t);
    IVP_ASSERT_TRUE(t->obj == a);
    IVP_ASSERT_NEAR(t->torque, IVP_REAL(0.0), 0.0);              /* Torque_Active: af value */
    IVP_ASSERT_NEAR(t->max_rotation_speed, IVP_REAL(50.0), 0.0);
    /* the object space y axis in core space (rotation about y keeps it) */
    IVP_ASSERT_NEAR(t->axis_os.k[1], IVP_REAL(1.0), TOL);
    IVP_ASSERT_TRUE(t->active_float_rotation_speed_out == speed_out);
    ivp_active_float_set(af_torque, 2.0);
    IVP_ASSERT_NEAR(t->torque, IVP_REAL(2.0), 0.0);
    for (int i = 0; i < 10; i++) ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_TRUE(fabs((double)a->physical_core->rot_speed.k[1]) > 0.1);
    /* deleting the object deletes the actuator (anchor_will_be_deleted_event)
     * and releases its active values */
    ivp_environment_destroy_object(env, a);
    IVP_ASSERT_FALSE(ivp_controller_factory_delete(env, t));
    ivp_active_float_set(af_torque, 3.0); /* no listener left */
    ivp_environment_destroy(env);
    ivp_active_float_remove_ref(af_torque);
    ivp_active_float_remove_ref(af_speed);
    ivp_active_float_remove_ref(speed_out);
}

IVP_TEST(factory_rotmot) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, 0.0, 0.0, 0.0, false);
    ivp_template_anchor_t an[2];
    anchor_os(&an[0], a, 0.0, 0.0, 0.0);
    anchor_os(&an[1], a, 1.0, 0.0, 0.0);
    ivp_template_rot_mot_t tr;
    ivp_template_rot_mot_init(&tr);
    tr.anchors[0] = &an[0];
    tr.anchors[1] = &an[1];
    tr.power = 20.0f;
    tr.max_torque = 5.0f;
    tr.max_rotation_speed = 10.0f;
    ivp_rot_mot_state_t *rm = ivp_controller_factory_create_rotmot(env, &tr);
    IVP_ASSERT_NOT_NULL(rm);
    ivp_rot_mot_state_t ref;
    ivp_vec3r_t p0 = ivp_vec3r_zero(), p1 = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_rot_mot_state_setup(&ref, a, &p0, a, &p1, IVP_REAL(20.0), IVP_REAL(5.0), IVP_REAL(10.0));
    IVP_ASSERT_TRUE(vec_eq(&rm->anchor_a_os, &ref.anchor_a_os) && vec_eq(&rm->anchor_b_os, &ref.anchor_b_os));
    IVP_ASSERT_TRUE(rm->power == ref.power && rm->max_torque == ref.max_torque &&
                    rm->max_rotation_speed == ref.max_rotation_speed);
    for (int i = 0; i < 20; i++) ivp_environment_simulate_one_psi(env);
    double w = sqrt((double)a->physical_core->rot_speed.k[0] * a->physical_core->rot_speed.k[0] +
                    (double)a->physical_core->rot_speed.k[1] * a->physical_core->rot_speed.k[1] +
                    (double)a->physical_core->rot_speed.k[2] * a->physical_core->rot_speed.k[2]);
    IVP_ASSERT_TRUE(w > 0.1);
    ivp_environment_destroy(env);
}

IVP_TEST(factory_spring_relative_and_active) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, 0.0, 0.0, 0.0, false);
    ivp_object_t *b = make_ball(env, 3.0, 0.0, 0.0, false);
    ivp_template_anchor_t an[2];
    anchor_os(&an[0], a, 0.2, 0.0, 0.0);
    anchor_os(&an[1], b, -0.2, 0.0, 0.0);
    ivp_active_float_t *len = ivp_active_float_terminal_create(NULL, "len", 1.5);
    ivp_template_spring_t ts;
    ivp_template_spring_init(&ts);
    ts.anchors[0] = &an[0];
    ts.anchors[1] = &an[1];
    ts.spring_len = 2.0f;
    ts.spring_constant = 10.0f;
    ts.spring_damp = 0.5f;
    ts.rel_pos_damp = 0.25f;
    ts.spring_values_are_relative = true;
    ts.active_float_spring_len = len;
    ivp_spring_state_t *s = ivp_controller_factory_create_spring(env, &ts);
    IVP_ASSERT_NOT_NULL(s);
    ivp_spring_state_t ref;
    ivp_vec3r_t pa = ivp_vec3r(IVP_REAL(0.2), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t pb = ivp_vec3r(IVP_REAL(-0.2), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_spring_state_setup(&ref, a, &pa, b, &pb, IVP_REAL(10.0), IVP_REAL(0.5), IVP_REAL(2.0));
    ref.rel_pos_damp = IVP_REAL(0.25);
    ivp_spring_state_make_relative(&ref);
    IVP_ASSERT_TRUE(s->spring_values_factor == ref.spring_values_factor);
    /* m0 * m1 / (m0 + m1) of the virtual masses at the anchors (< 2 * 2 / 4) */
    IVP_ASSERT_TRUE(s->spring_values_factor > IVP_REAL(0.0) && s->spring_values_factor < IVP_REAL(1.0));
    IVP_ASSERT_TRUE(s->stiffness == ref.stiffness && s->damping == ref.damping && s->rel_pos_damp == ref.rel_pos_damp);
    IVP_ASSERT_NEAR(s->rest_length, IVP_REAL(1.5), 0.0); /* Spring_Active: set_len(af) */
    IVP_ASSERT_TRUE(s->break_max_len == (ivp_real_t)IVP_FLOAT_MAX);
    ivp_active_float_set(len, 1.0);
    IVP_ASSERT_NEAR(s->rest_length, IVP_REAL(1.0), 0.0);
    for (int i = 0; i < 30; i++) ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_TRUE(b->physical_core->pos_world_f_core_last_psi.k[0] < IVP_REAL(3.0)); /* pulled together */
    ivp_environment_destroy(env);
    ivp_active_float_remove_ref(len);
}

IVP_TEST(factory_suspension) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *body = make_ball(env, 0.0, 0.0, 0.0, false);
    ivp_object_t *wheel = make_ball(env, 0.0, 1.5, 0.0, false);
    ivp_template_anchor_t an[2];
    anchor_os(&an[0], body, 0.0, 0.3, 0.0);
    anchor_os(&an[1], wheel, 0.0, 0.0, 0.0);
    ivp_template_suspension_t tsu;
    ivp_template_suspension_init(&tsu);
    tsu.spring.anchors[0] = &an[0];
    tsu.spring.anchors[1] = &an[1];
    tsu.spring.spring_len = 1.0f;
    tsu.spring.spring_constant = 40.0f;
    tsu.spring.spring_damp = 2.0f;
    tsu.spring_dampening_compression = 1.0f;
    tsu.max_body_force = 100.0f;
    ivp_suspension_state_t *su = ivp_controller_factory_create_suspension(env, &tsu);
    IVP_ASSERT_NOT_NULL(su);
    IVP_ASSERT_TRUE(su->obj_a == body && su->obj_b == wheel);
    IVP_ASSERT_NEAR(su->stiffness, IVP_REAL(40.0), 0.0);
    IVP_ASSERT_NEAR(su->damping_compression, IVP_REAL(1.0), 0.0);
    IVP_ASSERT_NEAR(su->max_body_force, IVP_REAL(100.0), 0.0);
    IVP_ASSERT_NEAR(su->spring_values_factor, IVP_REAL(1.0), 0.0);
    for (int i = 0; i < 10; i++) ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_TRUE(wheel->physical_core->speed.k[1] < IVP_REAL(0.0)); /* stretched 1.2 -> pulled back */
    ivp_environment_destroy(env);
}

/* IVP_Actuator_Four_Point never announces the stabilizer */
IVP_TEST(factory_stabilizer_inert_until_announced) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *o[4];
    for (int i = 0; i < 4; i++) o[i] = make_ball(env, 3.0 * i, 0.0, 0.0, false);
    ivp_template_anchor_t an[4];
    for (int i = 0; i < 4; i++) anchor_os(&an[i], o[i], 0.0, 0.1, 0.0);
    ivp_template_stabilizer_t tst;
    ivp_template_stabilizer_init(&tst);
    for (int i = 0; i < 4; i++) tst.anchors[i] = &an[i];
    tst.stabi_constant = 12.0f;
    const int n0 = controller_count(env);
    ivp_stabilizer_state_t *st = ivp_controller_factory_create_stabilizer(env, &tst);
    IVP_ASSERT_NOT_NULL(st);
    IVP_ASSERT_EQ(controller_count(env), n0);
    IVP_ASSERT_FALSE(ivp_controller_handle_is_valid(ivp_controller_factory_get_handle(env, st)));
    IVP_ASSERT_NEAR(st->stabi_constant, IVP_REAL(12.0), 0.0);
    ivp_controller_handle_t h = ivp_controller_factory_announce_stabilizer(env, st);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(h));
    IVP_ASSERT_EQ(controller_count(env), n0 + 1);
    IVP_ASSERT_TRUE(ivp_controller_factory_announce_stabilizer(env, st).slot == h.slot);
    /* deleting an anchor object deletes the stabilizer */
    ivp_environment_destroy_object(env, o[2]);
    IVP_ASSERT_EQ(controller_count(env), n0);
    IVP_ASSERT_FALSE(ivp_controller_factory_delete(env, st));
    ivp_environment_destroy(env);
}

static int g_cd_events, g_cd_deleted;
static void cd_event(void *ctx, ivp_object_t *a, ivp_object_t *b, ivp_real_t d, bool inside)
{
    (void)ctx; (void)a; (void)b; (void)d; (void)inside;
    g_cd_events++;
}
static void cd_deleted(void *ctx, ivp_check_dist_state_t *s)
{
    (void)ctx; (void)s;
    g_cd_deleted++;
}

IVP_TEST(factory_check_dist) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, 0.0, 0.0, 0.0, false);
    ivp_object_t *b = make_ball(env, 2.0, 0.0, 0.0, false);
    ivp_active_int_t *outside = ivp_active_int_terminal_create(NULL, "outside", -1);
    ivp_template_check_dist_t tc;
    ivp_template_check_dist_init(&tc);
    tc.objects[0] = a;
    tc.objects[1] = b;
    tc.position_world_space[0] = ivp_vec3r_zero();
    tc.position_world_space[1] = ivp_vec3r(IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    tc.range = 3.0f;
    tc.mod_is_outside = outside;
    const int n0 = controller_count(env);
    ivp_check_dist_state_t *cd = ivp_controller_factory_create_check_dist(env, &tc);
    IVP_ASSERT_NOT_NULL(cd);
    IVP_ASSERT_EQ(controller_count(env), n0); /* not a controller */
    IVP_ASSERT_TRUE(cd->attached);
    IVP_ASSERT_EQ(cd->is_outside, 0);
    g_cd_events = g_cd_deleted = 0;
    ivp_check_dist_set_callback(cd, cd_event, NULL);
    cd->deleted_fn = cd_deleted;
    ivp_vec3r_set(&b->physical_core->speed, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_ensure_core_in_simulation(env, b);
    for (int i = 0; i < 30; i++) ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(cd->is_outside, 1);
    IVP_ASSERT_TRUE(g_cd_events > 0);
    ivp_environment_destroy_object(env, b); /* the check dist goes with its anchor */
    IVP_ASSERT_EQ(g_cd_deleted, 1);
    IVP_ASSERT_FALSE(ivp_controller_factory_delete(env, cd));
    ivp_environment_destroy(env);
    ivp_active_int_remove_ref(outside);
}

IVP_TEST(factory_controller_motion) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, 2.0, -1.0, 0.5, false);
    ivp_object_t *other = make_ball(env, -3.0, 0.0, 0.0, false);
    ivp_template_controller_motion_t tm;
    ivp_template_controller_motion_init(&tm);
    tm.damp_factor = 1.2f;
    tm.max_torque = 80.0f;
    const int n0 = controller_count(env);
    ivp_motion_state_t *m = ivp_controller_factory_create_controller_motion(env, a, &tm);
    IVP_ASSERT_NOT_NULL(m);
    IVP_ASSERT_EQ(controller_count(env), n0 + 1);
    IVP_ASSERT_TRUE(m->object == a);
    IVP_ASSERT_NEAR(m->damp_factor, (ivp_real_t)1.2f, 0.0);
    IVP_ASSERT_NEAR(m->max_torque.k[2], IVP_REAL(80.0), 0.0);
    IVP_ASSERT_EQ((int)m->linear_mode, (int)IVP_MOTION_TARGET);
    ivp_environment_ensure_core_in_simulation(env, a);
    ivp_environment_ensure_core_in_simulation(env, other);
    for (int i = 0; i < 30; i++) ivp_environment_simulate_one_psi(env);
    /* drives toward the origin; the other object is not controlled */
    IVP_ASSERT_TRUE(a->physical_core->pos_world_f_core_last_psi.k[0] < IVP_REAL(1.5));
    IVP_ASSERT_NEAR(other->physical_core->speed.k[0], IVP_REAL(0.0), 0.0);
    ivp_environment_destroy_object(env, a); /* core_is_going_to_be_deleted_event */
    IVP_ASSERT_EQ(controller_count(env), n0);
    IVP_ASSERT_FALSE(ivp_controller_factory_delete(env, m));
    ivp_environment_destroy(env);
}

IVP_TEST(factory_constraint) {
    ivp_environment_t *env = make_env(false);
    ivp_object_t *a = make_ball(env, 0.0, 0.0, 0.0, false);
    ivp_object_t *b = make_ball(env, 1.5, 0.0, 0.0, false);
    ivp_constraint_template_t tc;
    ivp_constraint_template_init(&tc);
    IVP_ASSERT_TRUE(ivp_controller_factory_create_constraint(env, &tc) == NULL); /* no objects */
    tc.obj_r = a;
    tc.obj_a = b;
    tc.force_factor = IVP_REAL(0.0);
    const int n0 = controller_count(env);
    ivp_constraint_state_t *c = ivp_controller_factory_create_constraint(env, &tc);
    IVP_ASSERT_NOT_NULL(c);
    IVP_ASSERT_EQ(controller_count(env), n0 + 1);
    IVP_ASSERT_TRUE(c->tau == (ivp_real_t)P_FLOAT_EPS); /* force_factor raised to P_FLOAT_EPS */
    ivp_environment_destroy_object(env, b);
    IVP_ASSERT_EQ(controller_count(env), n0);
    IVP_ASSERT_FALSE(ivp_controller_factory_delete(env, c));
    ivp_environment_destroy(env);
}

int main(void)
{
    IVP_REGISTER(factory_template_defaults);
    IVP_REGISTER(factory_force_matches_setup_and_announces);
    IVP_REGISTER(factory_force_world_anchor);
    IVP_REGISTER(factory_torque_axis_active_values);
    IVP_REGISTER(factory_rotmot);
    IVP_REGISTER(factory_spring_relative_and_active);
    IVP_REGISTER(factory_suspension);
    IVP_REGISTER(factory_stabilizer_inert_until_announced);
    IVP_REGISTER(factory_check_dist);
    IVP_REGISTER(factory_controller_motion);
    IVP_REGISTER(factory_constraint);
    IVP_RUN_ALL();
}
