/* test_car_system.c -- unit tests for the real-wheel car system
 * (ivp_car_system.h, port of IVP_Car_System_Real_Wheels) and the car
 * constraint solver (ivp_constraint_car.h, IVP_Constraint_Solver_Car).
 *
 * Scene: static ground box (top at y = 2.5, gravity +y = down), a box body
 * and four ball wheels in one collision group; x = right, y = down,
 * z = forward.
 */
#include "ivp_test.h"

#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/controllers/ivp_car_system.h>
#include <ivp/controllers/ivp_constraint_car.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_quat.h>
#include "controllers/ivp_constraint_car_internal.h"

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

#define CAR_DT (1.0 / 180.0)

static const double WHEEL_X = 0.85, WHEEL_Y = 0.4, WHEEL_Z = 1.4, WHEEL_R = 0.35;

typedef struct car_scene {
    ivp_environment_t        *env;
    ivp_compact_surface_t    *ground_surf;
    ivp_compact_surface_t    *body_surf;
    ivp_surman_polygon_t      ground_sm;
    ivp_surman_polygon_t      body_sm;
    ivp_object_t             *ground;
    ivp_object_t             *body;
    ivp_object_t             *wheels[4];
    ivp_car_system_template_t tmpl;
    ivp_car_system_t          car;
} car_scene_t;

static ivp_compact_surface_t *box_surface(double hx, double hy, double hz)
{
    double c[24];
    int n = 0;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2) {
                c[n++] = sx * hx;
                c[n++] = sy * hy;
                c[n++] = sz * hz;
            }
    return ivp_build_convex_surface_d(NULL, c, 8);
}

/* Creates the world and fills sc->tmpl (the car itself is not created). */
static bool scene_create(car_scene_t *sc, double body_y)
{
    memset(sc, 0, sizeof(*sc));
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = (ivp_real_t)CAR_DT;
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), (ivp_real_t)9.83, IVP_REAL(0.0));
    sc->env = ivp_environment_create(&cfg);
    if (!sc->env) return false;

    sc->ground_surf = box_surface(30.0, 0.5, 30.0);
    sc->body_surf = box_surface(0.9, 0.3, 2.0);
    if (!sc->ground_surf || !sc->body_surf) return false;
    ivp_surman_polygon_init(&sc->ground_sm, sc->ground_surf);
    ivp_surman_polygon_init(&sc->body_sm, sc->body_surf);

    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_POLYGON;
    t.mass = IVP_REAL(1e6);
    t.physical_unmoveable = true;
    t.material.friction = (ivp_real_t)0.9;
    t.material.elasticity = IVP_REAL(0.0);
    ivp_vec3r_set(&t.position, IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0));
    sc->ground = ivp_environment_create_object(sc->env, &t, &sc->ground_sm.base);

    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_POLYGON;
    t.mass = IVP_REAL(800.0);
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(1100.0), IVP_REAL(1300.0), IVP_REAL(250.0));
    t.rot_inertia_is_factor = false;
    t.material.friction = (ivp_real_t)0.6;
    t.material.elasticity = (ivp_real_t)0.1;
    t.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&t.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    t.nocoll_group_ident = "car";
    ivp_vec3r_set(&t.position, IVP_REAL(0.0), (ivp_real_t)body_y, IVP_REAL(0.0));
    sc->body = ivp_environment_create_object(sc->env, &t, &sc->body_sm.base);

    ivp_car_system_template_init(&sc->tmpl, 4, 2);
    sc->tmpl.car_body = sc->body;

    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.mass = IVP_REAL(25.0);
    ivp_vec3r_set(&t.rot_inertia, (ivp_real_t)1.225, (ivp_real_t)1.225, (ivp_real_t)1.225);
    t.rot_inertia_is_factor = false;
    t.material.friction = (ivp_real_t)0.9;
    t.material.elasticity = (ivp_real_t)0.1;
    t.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set(&t.rot_speed_damp_factor, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    t.ball_radius = (ivp_real_t)WHEEL_R;
    t.nocoll_group_ident = "car";
    for (int i = 0; i < 4; i++) {
        const double wx = (i & 1) ? WHEEL_X : -WHEEL_X;
        const double wz = (i & 2) ? -WHEEL_Z : WHEEL_Z;
        ivp_vec3r_set(&t.position, (ivp_real_t)wx, (ivp_real_t)(body_y + WHEEL_Y), (ivp_real_t)wz);
        sc->wheels[i] = ivp_environment_create_object(sc->env, &t, NULL);
        if (!sc->wheels[i]) return false;
        sc->tmpl.wheel_objects[i] = sc->wheels[i];
        sc->tmpl.wheel_radius[i] = (ivp_real_t)WHEEL_R;
        sc->tmpl.wheel_pos_Bos[i] = ivp_vec3f((float)wx, (float)WHEEL_Y, (float)wz);
        sc->tmpl.spring_constant[i] = IVP_REAL(40000.0);
        sc->tmpl.spring_damping[i] = IVP_REAL(3000.0);
        sc->tmpl.spring_damping_compression[i] = IVP_REAL(3000.0);
        sc->tmpl.max_body_force[i] = IVP_REAL(60000.0);
        sc->tmpl.spring_pre_tension[i] = (ivp_real_t)0.05;
    }
    sc->tmpl.stabilizer_constant[0] = IVP_REAL(1500.0);
    sc->tmpl.stabilizer_constant[1] = IVP_REAL(1500.0);
    sc->tmpl.wheel_max_rotation_speed[0] = IVP_REAL(60.0);
    sc->tmpl.wheel_max_rotation_speed[1] = IVP_REAL(60.0);
    sc->tmpl.body_counter_torque_factor = (ivp_real_t)0.2;
    return sc->ground && sc->body;
}

static void scene_destroy(car_scene_t *sc)
{
    ivp_car_system_destroy(&sc->car);
    if (sc->env) ivp_environment_destroy(sc->env);
    if (sc->body_surf) ivp_compact_surface_free(NULL, sc->body_surf);
    if (sc->ground_surf) ivp_compact_surface_free(NULL, sc->ground_surf);
}

static void scene_step(car_scene_t *sc, int n)
{
    for (int i = 0; i < n; i++) ivp_environment_simulate_one_psi(sc->env);
}

/* wheel center relative to the body center in the body core system */
static ivp_vec3r_t wheel_in_body(const car_scene_t *sc, int i)
{
    const ivp_transform_r_t *mb = &sc->body->physical_core->m_world_f_core_last_psi;
    ivp_vec3r_t d, r;
    ivp_vec3r_sub(&d, &sc->wheels[i]->physical_core->m_world_f_core_last_psi.pos, &mb->pos);
    ivp_mat3r_vimult(&r, &mb->rot, &d);
    return r;
}

static double max_hard_point_error(const car_scene_t *sc)
{
    double m = 0.0;
    for (int i = 0; i < 4; i++) {
        ivp_vec3r_t r = wheel_in_body(sc, i);
        double ex = (double)r.k[0] - (double)sc->tmpl.wheel_pos_Bos[i].k[0];
        double ez = (double)r.k[2] - (double)sc->tmpl.wheel_pos_Bos[i].k[2];
        double e = sqrt(ex * ex + ez * ez);
        if (e > m) m = e;
    }
    return m;
}

/* ── template / tools ───────────────────────────────────────────────────── */

IVP_TEST(template_init_reference_defaults)
{
    ivp_car_system_template_t t;
    ivp_car_system_template_init(&t, 4, 2);
    IVP_ASSERT_EQ(t.n_wheels, 4);
    IVP_ASSERT_EQ(t.n_axes, 2);
    IVP_ASSERT_EQ(t.index_x, 0);
    IVP_ASSERT_EQ(t.index_y, 1);
    IVP_ASSERT_EQ(t.index_z, 2);
    IVP_ASSERT_FALSE(t.is_left_handed);
    IVP_ASSERT_NEAR(t.fast_turn_factor, 1.0, 0.0);
    for (int i = 0; i < 4; i++) IVP_ASSERT_NEAR(t.wheel_reversed_sign[i], 1.0, 0.0);
    IVP_ASSERT_NEAR(t.wheel_reversed_sign[4], 0.0, 0.0);
    IVP_ASSERT_NEAR(t.max_body_force[0], 0.0, 0.0); /* reference: no body force by default */
    IVP_ASSERT_FALSE(t.wheel_pos_from_objects);

    /* clamping of the reference constructor */
    ivp_car_system_template_init(&t, 20, 0);
    IVP_ASSERT_EQ(t.n_wheels, IVP_CAR_SYSTEM_MAX_WHEELS);
    IVP_ASSERT_EQ(t.n_axes, 1);
    ivp_car_system_template_init(&t, 2, 5);
    IVP_ASSERT_EQ(t.n_axes, 2);

    ivp_car_system_template_default(&t);
    IVP_ASSERT_EQ(t.n_wheels, 4);
    IVP_ASSERT_TRUE(t.wheel_pos_from_objects);
    IVP_ASSERT_TRUE(t.max_body_force[0] > 0.0);
}

IVP_TEST(ackerman_angle)
{
    /* below 0.001 rad the angle is returned unchanged */
    IVP_ASSERT_NEAR(ivp_car_system_calc_ackerman_angle((ivp_real_t)0.0005, IVP_REAL(1.7), IVP_REAL(2.8)), 0.0005, 1e-7);
    const double a = 0.3, dx = 1.7, dz = 2.8;
    const double ta = tan(a);
    const double expected = atan(dz * ta / (dx * ta + dz));
    ivp_real_t r = ivp_car_system_calc_ackerman_angle((ivp_real_t)a, (ivp_real_t)dx, (ivp_real_t)dz);
    IVP_ASSERT_NEAR(r, expected, 1e-6);
    IVP_ASSERT_TRUE(r < a); /* the outer wheel turns less */
    IVP_ASSERT_NEAR(ivp_car_system_calc_ackerman_angle((ivp_real_t)-a, (ivp_real_t)dx, (ivp_real_t)dz), -expected, 1e-6);
}

/* ── construction ───────────────────────────────────────────────────────── */

IVP_TEST(init_sets_car_wheel_and_geometry)
{
    car_scene_t sc;
    IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
    IVP_ASSERT_TRUE(ivp_car_system_init(&sc.car, &sc.tmpl));
    IVP_ASSERT_TRUE(sc.car.active);
    IVP_ASSERT_EQ(sc.car.n_wheels, 4);
    IVP_ASSERT_NOT_NULL(sc.car.car_constraint_solver);
    IVP_ASSERT_TRUE(ivp_constraint_solver_car_is_active(sc.car.car_constraint_solver));
    IVP_ASSERT_EQ(ivp_constraint_solver_car_get_num_wheels(sc.car.car_constraint_solver), 4);

    /* the reference sets car_wheel on the body core as well */
    IVP_ASSERT_TRUE(sc.body->physical_core->car_wheel == ivp_constraint_solver_car_get_body(sc.car.car_constraint_solver));
    for (int i = 0; i < 4; i++) {
        ivp_constraint_car_object_t *w = ivp_constraint_solver_car_get_wheel(sc.car.car_constraint_solver, i);
        IVP_ASSERT_NOT_NULL(w);
        IVP_ASSERT_TRUE(sc.wheels[i]->physical_core->car_wheel == w);
        IVP_ASSERT_TRUE(w->real_object == sc.wheels[i]);
        IVP_ASSERT_NEAR(w->target_position_bs.pos.k[0], sc.tmpl.wheel_pos_Bos[i].k[0], 1e-6);
        IVP_ASSERT_NEAR(w->target_position_bs.pos.k[1], sc.tmpl.wheel_pos_Bos[i].k[1], 1e-6);
        IVP_ASSERT_NEAR(w->target_position_bs.pos.k[2], sc.tmpl.wheel_pos_Bos[i].k[2], 1e-6);
    }
    IVP_ASSERT_TRUE(sc.ground->physical_core->car_wheel == NULL);

    IVP_ASSERT_NEAR(ivp_car_system_get_orig_front_wheel_distance(&sc.car), 2.0 * WHEEL_X, 1e-5);
    IVP_ASSERT_NEAR(ivp_car_system_get_orig_axles_distance(&sc.car), 2.0 * WHEEL_Z, 1e-5);

    /* suspension: anchor 500 m "up" along index_y, rest length 500 - pre tension */
    IVP_ASSERT_NEAR(sc.car.car_spring[0].rest_length, 500.0 - 0.05, 1e-4);
    IVP_ASSERT_NEAR(sc.car.car_spring[0].anchor_a_os.k[1], WHEEL_Y - 500.0, 1e-3);
    IVP_ASSERT_NEAR(sc.car.car_spring[0].max_body_force, 60000.0, 0.0);
    /* drive axis = -x for wheel_reversed_sign 1 */
    IVP_ASSERT_NEAR(sc.car.car_act_torque[0].axis_os.k[0], -1.0, 1e-6);
    IVP_ASSERT_NEAR(sc.car.car_act_torque_body.max_rotation_speed, (float)IVP_PI * 100.0f, 1e-3);
    IVP_ASSERT_TRUE(sc.car.car_stabilizer_exists[0]);
    IVP_ASSERT_TRUE(sc.car.car_stabilizer_exists[1]);
    IVP_ASSERT_NEAR(sc.car.car_act_extra_gravity.force, 0.0, 0.0);
    /* the static object anchor lies 1e8 m along gravity (y down -> -1e8 is up) */
    IVP_ASSERT_NEAR(sc.car.car_act_down_force.anchor_b_os.k[1], -1e8, 1.0);
    IVP_ASSERT_TRUE(sc.car.car_act_down_force.obj_b == ivp_environment_get_static_object(sc.env));

    ivp_car_system_destroy(&sc.car);
    IVP_ASSERT_FALSE(sc.car.active);
    for (int i = 0; i < 4; i++) IVP_ASSERT_TRUE(sc.wheels[i]->physical_core->car_wheel == NULL);
    IVP_ASSERT_TRUE(sc.body->physical_core->car_wheel == NULL);
    ivp_car_system_destroy(&sc.car); /* twice is fine */

    /* still simulates without the car */
    scene_step(&sc, 30);
    scene_destroy(&sc);
}

IVP_TEST(init_rejects_invalid_templates)
{
    car_scene_t sc;
    IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
    ivp_car_system_template_t t = sc.tmpl;
    t.n_wheels = 0;
    IVP_ASSERT_FALSE(ivp_car_system_init(&sc.car, &t));
    IVP_ASSERT_FALSE(sc.car.active);

    t = sc.tmpl;
    t.index_y = t.index_x;
    IVP_ASSERT_FALSE(ivp_car_system_init(&sc.car, &t));

    t = sc.tmpl;
    t.wheel_objects[2] = sc.ground; /* unmoveable wheel */
    IVP_ASSERT_FALSE(ivp_car_system_init(&sc.car, &t));
    for (int i = 0; i < 4; i++) IVP_ASSERT_TRUE(sc.wheels[i]->physical_core->car_wheel == NULL);
    IVP_ASSERT_TRUE(sc.body->physical_core->car_wheel == NULL);

    /* calls on an inactive car are no-ops */
    ivp_car_system_set_steering(&sc.car, IVP_REAL(0.2));
    ivp_car_system_set_wheel_torque(&sc.car, 2, IVP_REAL(100.0));
    ivp_car_system_fix_wheel(&sc.car, 2, true);
    IVP_ASSERT_NEAR(ivp_car_system_get_body_speed(&sc.car), 0.0, 0.0);
    scene_destroy(&sc);
}

IVP_TEST(wheel_pos_from_objects)
{
    car_scene_t sc;
    IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
    sc.tmpl.wheel_pos_from_objects = true;
    for (int i = 0; i < 4; i++) sc.tmpl.wheel_pos_Bos[i] = ivp_vec3f(0.0f, 0.0f, 0.0f);
    IVP_ASSERT_TRUE(ivp_car_system_init(&sc.car, &sc.tmpl));
    ivp_constraint_car_object_t *w3 = ivp_constraint_solver_car_get_wheel(sc.car.car_constraint_solver, 3);
    IVP_ASSERT_NEAR(w3->target_position_bs.pos.k[0], WHEEL_X, 1e-5);
    IVP_ASSERT_NEAR(w3->target_position_bs.pos.k[1], WHEEL_Y, 1e-5);
    IVP_ASSERT_NEAR(w3->target_position_bs.pos.k[2], -WHEEL_Z, 1e-5);
    scene_destroy(&sc);
}

/* ── simulation ─────────────────────────────────────────────────────────── */

IVP_TEST(settles_on_suspension)
{
    car_scene_t sc;
    IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
    IVP_ASSERT_TRUE(ivp_car_system_init(&sc.car, &sc.tmpl));
    scene_step(&sc, 180);

    /* wheels on the ground, body hanging on the suspension springs */
    for (int i = 0; i < 4; i++) {
        double y = (double)sc.wheels[i]->physical_core->pos_world_f_core_last_psi.k[1];
        IVP_ASSERT_TRUE(y > 2.5 - WHEEL_R - 0.05 && y < 2.5 - WHEEL_R + 0.01);
        /* compressed spring (shorter than the 500 m - pre tension rest length) */
        IVP_ASSERT_TRUE(sc.car.car_spring[i].current_length < sc.car.car_spring[i].rest_length);
    }
    IVP_ASSERT_TRUE(max_hard_point_error(&sc) < 1e-3);
    IVP_ASSERT_NEAR(ivp_car_system_get_body_speed(&sc.car), 0.0, 0.05);
    /* the wheels are kept parallel to the body (rotation constraint) */
    for (int i = 0; i < 4; i++) {
        const ivp_mat3r_t *rw = &sc.wheels[i]->physical_core->m_world_f_core_last_psi.rot;
        const ivp_mat3r_t *rb = &sc.body->physical_core->m_world_f_core_last_psi.rot;
        double dot = 0.0;
        for (int k = 0; k < 3; k++) dot += (double)rw->rows[k].k[0] * (double)rb->rows[k].k[0];
        IVP_ASSERT_TRUE(dot > 0.999);
    }
    scene_destroy(&sc);
}

IVP_TEST(drive_steer_and_handbrake)
{
    car_scene_t sc;
    IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
    IVP_ASSERT_TRUE(ivp_car_system_init(&sc.car, &sc.tmpl));
    scene_step(&sc, 120);
    const double z0 = (double)sc.body->physical_core->pos_world_f_core_last_psi.k[2];

    /* rear wheel drive (positive torque about -x rolls towards +z) */
    ivp_car_system_set_wheel_torque(&sc.car, IVP_WHEEL_REAR_LEFT, IVP_REAL(700.0));
    ivp_car_system_set_wheel_torque(&sc.car, IVP_WHEEL_REAR_RIGHT, IVP_REAL(700.0));
    ivp_car_system_update_body_countertorque(&sc.car);
    IVP_ASSERT_NEAR(sc.car.car_act_torque_body.torque, -1400.0 * 0.2, 1e-2);
    scene_step(&sc, 180);
    const double v = (double)ivp_car_system_get_body_speed(&sc.car);
    IVP_ASSERT_TRUE(v > 2.0);
    IVP_ASSERT_TRUE((double)sc.body->physical_core->pos_world_f_core_last_psi.k[2] > z0 + 1.0);
    IVP_ASSERT_TRUE(ivp_car_system_get_wheel_angular_velocity(&sc.car, IVP_WHEEL_REAR_LEFT) > 2.0);
    IVP_ASSERT_TRUE(max_hard_point_error(&sc) < 0.02);

    ivp_car_system_wheel_skid_info_t skid[4];
    ivp_car_system_get_skid_info(&sc.car, skid);
    IVP_ASSERT_TRUE(skid[2].last_skid_time > 0.0); /* 2D wheel friction ran */
    IVP_ASSERT_TRUE(skid[2].last_contact_position_ws.k[1] > 2.3f);

    /* steering: the outer (right) front wheel gets the Ackermann angle,
     * the inner one the full angle; the wheel targets rotate about y */
    ivp_car_system_set_steering(&sc.car, (ivp_real_t)0.3);
    IVP_ASSERT_NEAR(sc.car.steering_angle, 0.3, 1e-6);
    ivp_constraint_car_object_t *fl = ivp_constraint_solver_car_get_wheel(sc.car.car_constraint_solver, 0);
    ivp_constraint_car_object_t *fr = ivp_constraint_solver_car_get_wheel(sc.car.car_constraint_solver, 1);
    IVP_ASSERT_NEAR(fr->target_position_bs.rot.rows[0].k[0], cos(0.3), 1e-6);
    IVP_ASSERT_NEAR(fr->target_position_bs.rot.rows[2].k[0], -sin(0.3), 1e-6);
    double ack = (double)ivp_car_system_calc_ackerman_angle((ivp_real_t)0.3, (ivp_real_t)(2.0 * WHEEL_X),
                                                            (ivp_real_t)(2.0 * WHEEL_Z));
    IVP_ASSERT_NEAR(fl->target_position_bs.rot.rows[0].k[0], cos(ack), 1e-6);
    IVP_ASSERT_NEAR(fl->target_position_bs.pos.k[2], WHEEL_Z, 1e-6); /* translation kept */
    scene_step(&sc, 90);
    /* turning towards +x: yaw about +y */
    IVP_ASSERT_TRUE(sc.body->physical_core->rot_speed.k[1] > 0.05f);

    /* handbrake on the rear wheels */
    ivp_car_system_set_wheel_torque(&sc.car, IVP_WHEEL_REAR_LEFT, IVP_REAL(0.0));
    ivp_car_system_set_wheel_torque(&sc.car, IVP_WHEEL_REAR_RIGHT, IVP_REAL(0.0));
    ivp_car_system_update_body_countertorque(&sc.car);
    ivp_car_system_set_steering(&sc.car, IVP_REAL(0.0));
    ivp_car_system_fix_wheel(&sc.car, IVP_WHEEL_REAR_LEFT, true);
    ivp_car_system_fix_wheel(&sc.car, IVP_WHEEL_REAR_RIGHT, true);
    ivp_constraint_car_object_t *rl = ivp_constraint_solver_car_get_wheel(sc.car.car_constraint_solver, 2);
    IVP_ASSERT_NOT_NULL(rl->fix_wheel_constraint);
    const double v_before = (double)ivp_car_system_get_body_speed(&sc.car);
    scene_step(&sc, 90);
    /* blocked: the rear wheels turn with the body only */
    {
        float spin = sc.wheels[2]->physical_core->rot_speed.k[0];
        IVP_ASSERT_TRUE(fabsf(spin) < 0.5f);
    }
    IVP_ASSERT_TRUE((double)ivp_car_system_get_body_speed(&sc.car) < v_before);
    ivp_car_system_fix_wheel(&sc.car, IVP_WHEEL_REAR_LEFT, false);
    IVP_ASSERT_NULL(rl->fix_wheel_constraint);
    IVP_ASSERT_NULL(sc.car.fix_wheel_constraint[IVP_WHEEL_REAR_LEFT]);
    scene_step(&sc, 10);
    scene_destroy(&sc);
}

IVP_TEST(adjustment_setters)
{
    car_scene_t sc;
    IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
    IVP_ASSERT_TRUE(ivp_car_system_init(&sc.car, &sc.tmpl));
    ivp_car_system_set_spring_constant(&sc.car, 0, IVP_REAL(30000.0));
    IVP_ASSERT_NEAR(sc.car.car_spring[0].stiffness, 30000.0, 0.0);
    ivp_car_system_set_spring_damping(&sc.car, 0, IVP_REAL(1000.0));
    IVP_ASSERT_NEAR(sc.car.car_spring[0].damping, 1000.0, 0.0);
    ivp_car_system_set_spring_damping_compression(&sc.car, 0, IVP_REAL(2000.0));
    IVP_ASSERT_NEAR(sc.car.car_spring[0].damping_compression, 2000.0, 0.0);
    ivp_car_system_set_max_body_force(&sc.car, 0, IVP_REAL(5000.0));
    IVP_ASSERT_NEAR(sc.car.car_spring[0].max_body_force, 5000.0, 0.0);
    ivp_car_system_set_spring_pre_tension(&sc.car, 0, IVP_REAL(0.1));
    IVP_ASSERT_NEAR(sc.car.car_spring[0].rest_length, 500.0f - 0.1f, 1e-4);
    ivp_car_system_set_spring_length(&sc.car, 1, IVP_REAL(499.5));
    IVP_ASSERT_NEAR(sc.car.car_spring[1].rest_length, 499.5, 0.0);
    ivp_car_system_set_stabilizer(&sc.car, IVP_AXIS_REAR, IVP_REAL(700.0));
    IVP_ASSERT_NEAR(sc.car.car_stabilizer[1].stabi_constant, 700.0, 0.0);
    ivp_car_system_set_body_downforce(&sc.car, IVP_REAL(1234.0));
    IVP_ASSERT_NEAR(sc.car.car_act_down_force.force, 1234.0, 0.0);
    ivp_car_system_set_powerslide(&sc.car, IVP_REAL(1.0), IVP_REAL(2.0));
    IVP_ASSERT_NEAR(sc.car.car_act_powerslide_back.force, 800.0, 1e-3);
    IVP_ASSERT_NEAR(sc.car.car_act_powerslide_front.force, 1600.0, 1e-3);
    ivp_car_system_set_fast_turn_factor(&sc.car, IVP_REAL(0.5));
    IVP_ASSERT_NEAR(sc.car.fast_turn_factor, 0.5, 0.0);
    ivp_car_system_set_wheel_speed_dampening(&sc.car, 3, IVP_REAL(0.25));
    IVP_ASSERT_NEAR(sc.wheels[3]->physical_core->speed_damp_factor, 0.25, 0.0);
    /* out of range indices are ignored */
    ivp_car_system_set_wheel_torque(&sc.car, 7, IVP_REAL(1.0));
    ivp_car_system_set_stabilizer(&sc.car, 4, IVP_REAL(1.0));
    scene_step(&sc, 20);
    scene_destroy(&sc);
}

IVP_TEST(booster)
{
    car_scene_t sc;
    IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
    IVP_ASSERT_TRUE(ivp_car_system_init(&sc.car, &sc.tmpl));
    scene_step(&sc, 60);
    ivp_car_system_activate_booster(&sc.car, IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(1.0));
    IVP_ASSERT_TRUE(sc.car.booster_active);
    IVP_ASSERT_NEAR(ivp_car_system_get_booster_time_to_go(&sc.car), 0.5, 1e-6);
    IVP_ASSERT_NEAR(ivp_car_system_get_booster_delay(&sc.car), 1.5, 1e-6);
    /* thrust 0.5 g on 800 kg */
    IVP_ASSERT_NEAR(sc.car.booster_actuator[0].force, 0.5f * 9.83f * 800.0f, 0.5);
    /* gravity compensation pushes the body away from the anchor below it */
    IVP_ASSERT_NEAR(sc.car.booster_actuator[1].force, -800.0 * 9.83, 0.5);
    for (int i = 0; i < 100; i++) {
        ivp_car_system_update_booster(&sc.car, (ivp_real_t)CAR_DT);
        ivp_environment_simulate_one_psi(sc.env);
    }
    IVP_ASSERT_FALSE(sc.car.booster_active);
    IVP_ASSERT_TRUE((double)ivp_car_system_get_body_speed(&sc.car) > 0.5);
    /* still recharging */
    ivp_car_system_activate_booster(&sc.car, IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(1.0));
    IVP_ASSERT_FALSE(sc.car.booster_active);
    scene_destroy(&sc);
}

/* ── life cycle ─────────────────────────────────────────────────────────── */

IVP_TEST(destroy_after_environment)
{
    car_scene_t sc;
    IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
    IVP_ASSERT_TRUE(ivp_car_system_init(&sc.car, &sc.tmpl));
    ivp_car_system_fix_wheel(&sc.car, 2, true);
    scene_step(&sc, 20);
    ivp_environment_destroy(sc.env);
    sc.env = NULL;
    ivp_car_system_destroy(&sc.car); /* must not touch the dead environment */
    IVP_ASSERT_FALSE(sc.car.active);
    scene_destroy(&sc);
}

IVP_TEST(deleting_a_wheel_detaches_the_car)
{
    car_scene_t sc;
    IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
    IVP_ASSERT_TRUE(ivp_car_system_init(&sc.car, &sc.tmpl));
    scene_step(&sc, 20);
    ivp_environment_destroy_object(sc.env, sc.wheels[1]);
    sc.wheels[1] = NULL;
    IVP_ASSERT_FALSE(ivp_constraint_solver_car_is_active(sc.car.car_constraint_solver));
    IVP_ASSERT_TRUE(sc.body->physical_core->car_wheel == NULL);
    IVP_ASSERT_TRUE(sc.wheels[0]->physical_core->car_wheel == NULL);
    /* the car is unusable but safe */
    ivp_car_system_set_wheel_torque(&sc.car, 0, IVP_REAL(100.0));
    ivp_car_system_set_steering(&sc.car, IVP_REAL(0.2));
    ivp_car_system_fix_wheel(&sc.car, 0, true);
    scene_step(&sc, 20);
    ivp_car_system_destroy(&sc.car);
    scene_step(&sc, 5);
    scene_destroy(&sc);
}

IVP_TEST(constraint_solver_standalone)
{
    car_scene_t sc;
    IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
    ivp_constraint_solver_car_t *s = ivp_constraint_solver_car_create(0, 1, 2, false);
    IVP_ASSERT_NOT_NULL(s);
    IVP_ASSERT_FALSE(ivp_constraint_solver_car_is_active(s));
    int x = -1, y = -1, z = -1;
    ivp_constraint_solver_car_get_axes(s, &x, &y, &z);
    IVP_ASSERT_TRUE(x == 0 && y == 1 && z == 2);
    /* NULL hard points: keep the current relative poses */
    IVP_ASSERT_EQ(ivp_constraint_solver_car_init_constraint_system(s, sc.env, sc.body, sc.wheels, NULL, 4), IVP_OK);
    IVP_ASSERT_TRUE(ivp_constraint_solver_car_is_active(s));
    IVP_ASSERT_EQ(ivp_constraint_solver_car_init_constraint_system(s, sc.env, sc.body, sc.wheels, NULL, 4),
                  IVP_ERR_INVALID_STATE);
    ivp_constraint_car_object_t *w0 = ivp_constraint_solver_car_get_wheel(s, 0);
    IVP_ASSERT_NEAR(w0->target_position_bs.pos.k[0], -WHEEL_X, 1e-6);
    IVP_ASSERT_NEAR(w0->target_position_bs.pos.k[1], WHEEL_Y, 1e-6);

    /* without springs the wheels fall away from the body (y is free) but stay
     * on their hard points in x/z */
    scene_step(&sc, 30);
    IVP_ASSERT_TRUE(max_hard_point_error(&sc) < 1e-3);
    ivp_constraint_solver_car_destroy(s);
    IVP_ASSERT_TRUE(sc.wheels[0]->physical_core->car_wheel == NULL);
    scene_step(&sc, 5);
    scene_destroy(&sc);
}

IVP_TEST(constraint_solver_core_list_failure_is_atomic)
{
    car_scene_t sc;
    IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
    ivp_constraint_solver_car_t *s = ivp_constraint_solver_car_create(0, 1, 2, false);
    IVP_ASSERT_NOT_NULL(s);

    ivp_pvec_t saved = s->cores_of_constraint_system;
    s->cores_of_constraint_system = (ivp_pvec_t){ .elems = NULL, .n = 0, .cap = -1 };
    IVP_ASSERT_EQ(ivp_constraint_solver_car_init_constraint_system(
                      s, sc.env, sc.body, sc.wheels, NULL, 4),
                  IVP_ERR_OUT_OF_MEMORY);
    IVP_ASSERT_FALSE(ivp_constraint_solver_car_is_active(s));
    IVP_ASSERT_TRUE(sc.body->physical_core->car_wheel == NULL);
    for (int i = 0; i < 4; i++) {
        IVP_ASSERT_TRUE(sc.wheels[i]->physical_core->car_wheel == NULL);
    }
    s->cores_of_constraint_system = saved;

    IVP_ASSERT_EQ(ivp_constraint_solver_car_init_constraint_system(
                      s, sc.env, sc.body, sc.wheels, NULL, 4),
                  IVP_OK);
    ivp_constraint_solver_car_destroy(s);
    scene_destroy(&sc);
}

static void *car_fail_alloc(size_t size, void *ctx)
{
    (void)size;
    (void)ctx;
    return NULL;
}

typedef struct car_fail_nth_alloc_state {
    ivp_allocator_t base;
    int attempts;
    int fail_at;
} car_fail_nth_alloc_state_t;

static void *car_fail_nth_alloc(size_t size, void *ctx)
{
    car_fail_nth_alloc_state_t *state = (car_fail_nth_alloc_state_t *)ctx;
    state->attempts++;
    if (state->attempts == state->fail_at) return NULL;
    return state->base.alloc(size, state->base.ctx);
}

static void *car_fail_nth_realloc(void *ptr, size_t old_size, size_t new_size, void *ctx)
{
    car_fail_nth_alloc_state_t *state = (car_fail_nth_alloc_state_t *)ctx;
    state->attempts++;
    if (state->attempts == state->fail_at) return NULL;
    return state->base.realloc(ptr, old_size, new_size, state->base.ctx);
}

static void car_fail_nth_free(void *ptr, size_t size, void *ctx)
{
    car_fail_nth_alloc_state_t *state = (car_fail_nth_alloc_state_t *)ctx;
    state->base.free(ptr, size, state->base.ctx);
}

IVP_TEST(constraint_solver_listener_failure_is_atomic)
{
    car_scene_t sc;
    IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
    ivp_constraint_solver_car_t *s = ivp_constraint_solver_car_create(0, 1, 2, false);
    IVP_ASSERT_NOT_NULL(s);

    ivp_allocator_t saved_allocator = *sc.env->allocator;
    sc.env->allocator->alloc = car_fail_alloc;
    IVP_ASSERT_EQ(ivp_constraint_solver_car_init_constraint_system(
                      s, sc.env, sc.body, sc.wheels, NULL, 4),
                  IVP_ERR_OUT_OF_MEMORY);
    *sc.env->allocator = saved_allocator;

    IVP_ASSERT_FALSE(ivp_constraint_solver_car_is_active(s));
    IVP_ASSERT_TRUE(sc.body->physical_core->car_wheel == NULL);
    for (int i = 0; i < 4; i++) {
        IVP_ASSERT_TRUE(sc.wheels[i]->physical_core->car_wheel == NULL);
    }
    IVP_ASSERT_EQ(ivp_constraint_solver_car_init_constraint_system(
                      s, sc.env, sc.body, sc.wheels, NULL, 4),
                  IVP_OK);
    ivp_constraint_solver_car_destroy(s);
    scene_destroy(&sc);
}

IVP_TEST(car_early_allocation_failure_is_atomic)
{
    car_scene_t sc;
    IVP_ASSERT_TRUE(scene_create(&sc, 1.7));

    car_fail_nth_alloc_state_t state = {
        .base = *sc.env->allocator,
        .attempts = 0,
        .fail_at = 3
    };
    *sc.env->allocator = (ivp_allocator_t){
        .alloc = car_fail_nth_alloc,
        .realloc = car_fail_nth_realloc,
        .free = car_fail_nth_free,
        .ctx = &state
    };
    IVP_ASSERT_FALSE(ivp_car_system_init(&sc.car, &sc.tmpl));
    *sc.env->allocator = state.base;

    IVP_ASSERT_TRUE(state.attempts >= state.fail_at);
    IVP_ASSERT_FALSE(sc.car.active);
    IVP_ASSERT_NULL(sc.car.car_constraint_solver);
    IVP_ASSERT_EQ(sc.car.n_wheels, 0);
    IVP_ASSERT_TRUE(sc.body->physical_core->car_wheel == NULL);
    for (int i = 0; i < 4; i++) {
        IVP_ASSERT_TRUE(sc.wheels[i]->physical_core->car_wheel == NULL);
    }

    IVP_ASSERT_TRUE(ivp_car_system_init(&sc.car, &sc.tmpl));
    ivp_car_system_destroy(&sc.car);
    scene_destroy(&sc);
}

static bool car_init_failure_is_clean(const car_scene_t *sc)
{
    if (sc->car.active || sc->car.car_constraint_solver || sc->car.n_wheels != 0 ||
        sc->body->physical_core->car_wheel) {
        return false;
    }
    for (int i = 0; i < 4; i++) {
        if (sc->wheels[i]->physical_core->car_wheel ||
            ivp_controller_handle_is_valid(sc->car.h_spring[i]) ||
            ivp_controller_handle_is_valid(sc->car.h_torque[i])) {
            return false;
        }
    }
    return !ivp_controller_handle_is_valid(sc->car.h_torque_body) &&
           !ivp_controller_handle_is_valid(sc->car.h_down_force) &&
           !ivp_controller_handle_is_valid(sc->car.h_extra_gravity) &&
           !ivp_controller_handle_is_valid(sc->car.h_powerslide_back) &&
           !ivp_controller_handle_is_valid(sc->car.h_powerslide_front);
}

IVP_TEST(car_controller_registration_failures_are_atomic)
{
    int allocation_attempts;
    {
        car_scene_t sc;
        IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
        car_fail_nth_alloc_state_t state = {
            .base = *sc.env->allocator,
            .attempts = 0,
            .fail_at = INT_MAX
        };
        *sc.env->allocator = (ivp_allocator_t){
            .alloc = car_fail_nth_alloc,
            .realloc = car_fail_nth_realloc,
            .free = car_fail_nth_free,
            .ctx = &state
        };
        IVP_ASSERT_TRUE(ivp_car_system_init(&sc.car, &sc.tmpl));
        *sc.env->allocator = state.base;
        allocation_attempts = state.attempts;
        scene_destroy(&sc);
    }

    IVP_ASSERT_TRUE(allocation_attempts > 3);
    /* Attempts 1 and 2 belong to constraint-solver setup, whose reference
     * failure path asserts in debug builds; attempt 3 is covered above. */
    for (int fail_at = 4; fail_at <= allocation_attempts; fail_at++) {
        car_scene_t sc;
        IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
        car_fail_nth_alloc_state_t state = {
            .base = *sc.env->allocator,
            .attempts = 0,
            .fail_at = fail_at
        };
        *sc.env->allocator = (ivp_allocator_t){
            .alloc = car_fail_nth_alloc,
            .realloc = car_fail_nth_realloc,
            .free = car_fail_nth_free,
            .ctx = &state
        };
        IVP_ASSERT_FALSE(ivp_car_system_init(&sc.car, &sc.tmpl));
        *sc.env->allocator = state.base;
        IVP_ASSERT_TRUE(car_init_failure_is_clean(&sc));

        IVP_ASSERT_TRUE(ivp_car_system_init(&sc.car, &sc.tmpl));
        scene_destroy(&sc);
    }
}

IVP_TEST(booster_registration_failures_are_atomic)
{
    int allocation_attempts;
    {
        car_scene_t sc;
        IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
        IVP_ASSERT_TRUE(ivp_car_system_init(&sc.car, &sc.tmpl));
        car_fail_nth_alloc_state_t state = {
            .base = *sc.env->allocator,
            .attempts = 0,
            .fail_at = INT_MAX
        };
        *sc.env->allocator = (ivp_allocator_t){
            .alloc = car_fail_nth_alloc,
            .realloc = car_fail_nth_realloc,
            .free = car_fail_nth_free,
            .ctx = &state
        };
        ivp_car_system_activate_booster(&sc.car, IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(1.0));
        *sc.env->allocator = state.base;
        IVP_ASSERT_TRUE(sc.car.booster_active);
        allocation_attempts = state.attempts;
        scene_destroy(&sc);
    }

    IVP_ASSERT_TRUE(allocation_attempts > 1);
    for (int fail_at = 1; fail_at <= allocation_attempts; fail_at++) {
        car_scene_t sc;
        IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
        IVP_ASSERT_TRUE(ivp_car_system_init(&sc.car, &sc.tmpl));
        car_fail_nth_alloc_state_t state = {
            .base = *sc.env->allocator,
            .attempts = 0,
            .fail_at = fail_at
        };
        *sc.env->allocator = (ivp_allocator_t){
            .alloc = car_fail_nth_alloc,
            .realloc = car_fail_nth_realloc,
            .free = car_fail_nth_free,
            .ctx = &state
        };
        ivp_car_system_activate_booster(&sc.car, IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(1.0));
        *sc.env->allocator = state.base;

        IVP_ASSERT_FALSE(sc.car.booster_active);
        IVP_ASSERT_FALSE(ivp_controller_handle_is_valid(sc.car.h_booster[0]));
        IVP_ASSERT_FALSE(ivp_controller_handle_is_valid(sc.car.h_booster[1]));
        IVP_ASSERT_NEAR(ivp_car_system_get_booster_time_to_go(&sc.car), 0.0, 0.0);
        IVP_ASSERT_NEAR(ivp_car_system_get_booster_delay(&sc.car), 0.0, 0.0);

        ivp_car_system_activate_booster(&sc.car, IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(1.0));
        IVP_ASSERT_TRUE(sc.car.booster_active);
        scene_destroy(&sc);
    }
}

IVP_TEST(fix_wheel_registration_failure_is_atomic)
{
    car_scene_t sc;
    IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
    IVP_ASSERT_TRUE(ivp_car_system_init(&sc.car, &sc.tmpl));

    ivp_vec3f_t speed_before = sc.wheels[0]->physical_core->rot_speed;
    ivp_allocator_t saved_allocator = *sc.env->allocator;
    sc.env->allocator->alloc = car_fail_alloc;
    ivp_car_system_fix_wheel(&sc.car, 0, true);
    *sc.env->allocator = saved_allocator;

    IVP_ASSERT_NULL(sc.car.fix_wheel_constraint[0]);
    IVP_ASSERT_FALSE(ivp_controller_handle_is_valid(sc.car.h_fix_wheel[0]));
    IVP_ASSERT_NULL(ivp_constraint_solver_car_get_wheel(
                        sc.car.car_constraint_solver, 0)->fix_wheel_constraint);
    for (int i = 0; i < 3; i++) {
        IVP_ASSERT_TRUE(sc.wheels[0]->physical_core->rot_speed.k[i] == speed_before.k[i]);
    }

    ivp_car_system_fix_wheel(&sc.car, 0, true);
    IVP_ASSERT_NOT_NULL(sc.car.fix_wheel_constraint[0]);
    IVP_ASSERT_TRUE(ivp_controller_handle_is_valid(sc.car.h_fix_wheel[0]));
    ivp_car_system_destroy(&sc.car);
    scene_destroy(&sc);
}

IVP_TEST(compat_controller_entry_is_noop)
{
    car_scene_t sc;
    IVP_ASSERT_TRUE(scene_create(&sc, 1.7));
    IVP_ASSERT_TRUE(ivp_car_system_init(&sc.car, &sc.tmpl));
    ivp_controller_handle_t h =
        ivp_environment_add_controller(sc.env, IVP_CP_CONSTRAINTS, ivp_controller_car_system, &sc.car);
    scene_step(&sc, 30);
    if (ivp_controller_handle_is_valid(h)) (void)ivp_environment_remove_controller(sc.env, h);
    IVP_ASSERT_TRUE(max_hard_point_error(&sc) < 1e-3);
    scene_destroy(&sc);
}

int main(void)
{
    IVP_REGISTER(template_init_reference_defaults);
    IVP_REGISTER(ackerman_angle);
    IVP_REGISTER(init_sets_car_wheel_and_geometry);
    IVP_REGISTER(init_rejects_invalid_templates);
    IVP_REGISTER(wheel_pos_from_objects);
    IVP_REGISTER(settles_on_suspension);
    IVP_REGISTER(drive_steer_and_handbrake);
    IVP_REGISTER(adjustment_setters);
    IVP_REGISTER(booster);
    IVP_REGISTER(destroy_after_environment);
    IVP_REGISTER(deleting_a_wheel_detaches_the_car);
    IVP_REGISTER(constraint_solver_standalone);
    IVP_REGISTER(constraint_solver_core_list_failure_is_atomic);
    IVP_REGISTER(constraint_solver_listener_failure_is_atomic);
    IVP_REGISTER(car_early_allocation_failure_is_atomic);
    IVP_REGISTER(car_controller_registration_failures_are_atomic);
    IVP_REGISTER(booster_registration_failures_are_atomic);
    IVP_REGISTER(fix_wheel_registration_failure_is_atomic);
    IVP_REGISTER(compat_controller_entry_is_noop);
    IVP_RUN_ALL();
}
