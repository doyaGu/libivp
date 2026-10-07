/* example_wheelcar.c -- A car on real wheels: the car system and the car constraint solver
 *
 * Unlike the raycast cars, this one has four ball objects as wheels. The car
 * system (IVP_Car_System_Real_Wheels) keeps each wheel on its hard point in the
 * body with the car constraint solver, hangs it on a suspension actuator, drives
 * the rear wheels with torque actuators, steers the front wheels with an Ackermann
 * angle, blocks them for the handbrake and adds a body counter torque. The wheels
 * grip the ground through ordinary contact friction.
 *
 *   Up / Down      drive forward / reverse
 *   Left / Right   steer
 *   B              handbrake (hold)
 *   F5 panel       drive torque, spring stiffness, stabilizer, steering lock
 *
 * Demonstrates: ivp_car_system_init (real wheels), set_wheel_torque,
 *               set_steering, fix_wheel, set_spring_constant, set_stabilizer,
 *               get_body_speed, get_wheel_angular_velocity, get_skid_info,
 *               the car constraint solver behind it.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_car_system.h>
#include <ivp/controllers/ivp_constraint_car.h>

#define WHEEL_X  0.95
#define WHEEL_Y  0.4
#define WHEEL_Z  1.4
#define WHEEL_R  0.35

typedef struct wheelcar_demo {
    ivp_environment_t        *env;
    ivp_object_t             *ground, *ramp, *cone[3];
    ivp_object_t             *body, *wheel[4];
    ivp_car_system_template_t tmpl;
    ivp_car_system_t          car;
    bool                      car_ok;
    /* panel */
    float                     torque, spring_k, stabilizer, lock;
    /* controls */
    float                     throttle, steer;
    bool                      brake;
    /* measurements */
    float                     max_hard_error, max_tilt, speed_peak, heading0;
    double                    z0;
    float                     speed_before_brake;
    bool                      braked;
} wheelcar_demo_t;

static float hard_point_error(const wheelcar_demo_t *s)
{
    const ivp_transform_r_t *mb = &s->body->physical_core->m_world_f_core_last_psi;
    float worst = 0.0f;
    for (int i = 0; i < 4; i++) {
        ivp_vec3r_t d, r;
        ivp_vec3r_sub(&d, &s->wheel[i]->physical_core->m_world_f_core_last_psi.pos, &mb->pos);
        ivp_mat3r_vimult(&r, &mb->rot, &d);
        const float ex = (float)r.k[0] - s->tmpl.wheel_pos_Bos[i].k[0], ez = (float)r.k[2] - s->tmpl.wheel_pos_Bos[i].k[2];
        const float e = sqrtf(ex * ex + ez * ez);
        if (e > worst) worst = e;
    }
    return worst;
}

static float heading_deg(const wheelcar_demo_t *s)
{
    float pos[3], rot[9];
    demo_object_pose(s->body, pos, rot);
    return atan2f(rot[6], rot[8]) * (180.0f / 3.14159265f);
}

static void drive(wheelcar_demo_t *s)
{
    const ivp_real_t t = (ivp_real_t)(s->throttle * s->torque);
    ivp_car_system_set_wheel_torque(&s->car, IVP_WHEEL_REAR_LEFT, t);
    ivp_car_system_set_wheel_torque(&s->car, IVP_WHEEL_REAR_RIGHT, t);
    ivp_car_system_update_body_countertorque(&s->car);
    ivp_car_system_set_steering(&s->car, (ivp_real_t)(s->steer * s->lock));
    ivp_car_system_fix_wheel(&s->car, IVP_WHEEL_REAR_LEFT, s->brake);
    ivp_car_system_fix_wheel(&s->car, IVP_WHEEL_REAR_RIGHT, s->brake);
}

static void apply_suspension(void *ctx)
{
    wheelcar_demo_t *s = (wheelcar_demo_t *)ctx;
    for (int i = 0; i < 4; i++) ivp_car_system_set_spring_constant(&s->car, i, (ivp_real_t)s->spring_k);
    ivp_car_system_set_stabilizer(&s->car, 0, (ivp_real_t)s->stabilizer);
    ivp_car_system_set_stabilizer(&s->car, 1, (ivp_real_t)s->stabilizer);
    ivp_environment_ensure_core_in_simulation(s->env, s->body);
}

static void apply_drive(void *ctx)
{
    wheelcar_demo_t *s = (wheelcar_demo_t *)ctx;
    drive(s);
    ivp_environment_ensure_core_in_simulation(s->env, s->body);
}

static void wheelcar_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    wheelcar_demo_t *s = (wheelcar_demo_t *)app->user;
    demo_panel_t *p = &v->panel;
    demo_panel_add_float(p, "drive torque", &s->torque, 100.0f, 1500.0f, apply_drive, s);
    demo_panel_add_float(p, "steering lock", &s->lock, 0.1f, 0.6f, apply_drive, s);
    demo_panel_add_float(p, "spring k", &s->spring_k, 10000.0f, 80000.0f, apply_suspension, s);
    demo_panel_add_float(p, "stabilizer", &s->stabilizer, 0.0f, 4000.0f, apply_suspension, s);
}

static void wheelcar_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    wheelcar_demo_t *s = (wheelcar_demo_t *)app->user;
    if (!s->car_ok) return;
    if (v) {
        const bool *keys = SDL_GetKeyboardState(NULL);
        s->throttle = (keys[SDL_SCANCODE_UP] ? 1.0f : 0.0f) - (keys[SDL_SCANCODE_DOWN] ? 0.5f : 0.0f);
        s->steer = (keys[SDL_SCANCODE_RIGHT] ? 1.0f : 0.0f) - (keys[SDL_SCANCODE_LEFT] ? 1.0f : 0.0f);
        s->brake = keys[SDL_SCANCODE_B];
        drive(s);
        float c[3];
        demo_object_center(s->body, c);
        for (int k = 0; k < 3; k++) v->cam.target[k] += (c[k] - v->cam.target[k]) * 0.05f;
    } else {
        /* settle, drive, steer, release the throttle and brake */
        if (frame == 90) {
            s->throttle = 1.0f;
            drive(s);
        }
        if (frame == 240) s->heading0 = heading_deg(s);
        if (frame == 241) {
            s->steer = 1.0f;
            drive(s);
        }
        if (frame == 330) {
            s->steer = 0.0f;
            s->throttle = 0.0f;
            s->speed_before_brake = (float)ivp_car_system_get_body_speed(&s->car);
            s->brake = true;
            s->braked = true;
            drive(s);
        }
    }
    if (frame > 60) {
        const float e = hard_point_error(s), tilt = demo_object_tilt_deg(s->body), sp = (float)ivp_car_system_get_body_speed(&s->car);
        if (e > s->max_hard_error) s->max_hard_error = e;
        if (tilt > s->max_tilt) s->max_tilt = tilt;
        if (sp > s->speed_peak) s->speed_peak = sp;
    }
}

static void wheelcar_reset(demo_app_t *app)
{
    wheelcar_demo_t *s = (wheelcar_demo_t *)app->user;
    s->max_hard_error = s->max_tilt = s->speed_peak = 0.0f;
}

static void wheelcar_draw(demo_app_t *app, demo_viewer_t *v)
{
    wheelcar_demo_t *s = (wheelcar_demo_t *)app->user;
    const float c_ground[3] = { 0.45f, 0.50f, 0.44f }, c_ramp[3] = { 0.60f, 0.55f, 0.45f }, c_cone[3] = { 0.95f, 0.50f, 0.20f },
                c_body[3] = { 0.30f, 0.60f, 0.95f }, c_wheel[3] = { 0.20f, 0.20f, 0.22f };
    demo_draw_object(v, s->ground, c_ground);
    demo_draw_object(v, s->ramp, c_ramp);
    for (int i = 0; i < 3; i++) demo_draw_object(v, s->cone[i], c_cone);
    demo_draw_object(v, s->body, c_body);
    ivp_car_system_wheel_skid_info_t skid_now[4] = { 0 };
    if (s->car_ok) ivp_car_system_get_skid_info(&s->car, skid_now);
    for (int i = 0; i < 4; i++) {
        /* a wheel that skids glows red */
        const float k = fminf(1.0f, skid_now[i].last_skid_value);
        const float wheel_col[3] = { c_wheel[0] + (0.95f - c_wheel[0]) * k, c_wheel[1] + (0.25f - c_wheel[1]) * k,
                                     c_wheel[2] + (0.15f - c_wheel[2]) * k };
        demo_draw_object(v, s->wheel[i], wheel_col);
        float a[3], b[3];
        demo_object_center(s->body, a);
        demo_object_center(s->wheel[i], b);
        demo_line(v, a, b, DEMO_COL_SPRING);               /* the strut */
    }
    if (s->car_ok)
        demo_hud(v, DEMO_COL_INFO, "speed %.1f m/s  wheel spin %.1f rad/s  hard points %.3f m off  tilt %.0f deg%s",
                 (double)ivp_car_system_get_body_speed(&s->car),
                 (double)ivp_car_system_get_wheel_angular_velocity(&s->car, IVP_WHEEL_REAR_LEFT), (double)hard_point_error(s),
                 (double)demo_object_tilt_deg(s->body), s->brake ? "  HANDBRAKE" : "");
    ivp_car_system_wheel_skid_info_t skid[4];
    if (s->car_ok) {
        ivp_car_system_get_skid_info(&s->car, skid);
        demo_hud(v, DEMO_COL_HELP, "skid values %.2f %.2f %.2f %.2f", (double)skid[0].last_skid_value,
                 (double)skid[1].last_skid_value, (double)skid[2].last_skid_value, (double)skid[3].last_skid_value);
    }
}

static void wheelcar_stats(demo_app_t *app, char *buf, size_t size)
{
    wheelcar_demo_t *s = (wheelcar_demo_t *)app->user;
    float c[3];
    demo_object_center(s->body, c);
    snprintf(buf, size, "pos (%.1f %.1f %.1f) speed %.2f hard error %.3f tilt %.0f", (double)c[0], (double)c[1], (double)c[2],
             s->car_ok ? (double)ivp_car_system_get_body_speed(&s->car) : 0.0, (double)hard_point_error(s), (double)s->max_tilt);
}

static bool wheelcar_check(demo_app_t *app)
{
    wheelcar_demo_t *s = (wheelcar_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    float c[3];
    demo_object_center(s->body, c);
    ok &= demo_check(tag, s->car_ok, "the car system started");
    ok &= demo_check(tag, s->speed_peak > 3.0f, "the rear wheels drive it (up to %.1f m/s)", (double)s->speed_peak);
    ok &= demo_check(tag, c[2] - (float)s->z0 > 6.0f, "it drove %.1f m", (double)(c[2] - (float)s->z0));
    ok &= demo_check(tag, s->max_hard_error < 0.15f, "the wheels stay on their hard points (worst %.3f m)", (double)s->max_hard_error);
    ok &= demo_check(tag, s->max_tilt < 30.0f, "it stays upright (tilt up to %.0f deg)", (double)s->max_tilt);
    ok &= demo_check(tag, fabsf(heading_deg(s) - s->heading0) > 8.0f, "steering turned it (%.0f deg to %.0f deg)", (double)s->heading0,
                     (double)heading_deg(s));
    ok &= demo_check(tag, s->braked && (float)ivp_car_system_get_body_speed(&s->car) < s->speed_before_brake,
                     "the handbrake slows it (%.1f to %.1f m/s)", (double)s->speed_before_brake,
                     (double)ivp_car_system_get_body_speed(&s->car));
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Wheel Car -- Car System with Real Wheels";
    app.cfg.tag         = "wheelcar";
    app.cfg.friction    = 0.9;
    app.cfg.elasticity  = 0.0;
    app.cfg.orbit_dist  = 14.0f;
    app.cfg.orbit_yaw   = 160.0f;
    app.cfg.orbit_pitch = -20.0f;
    app.cfg.target[2]   = -18.0f;
    app.cfg.grid_extent = 36.0f;
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ecfg.gravity = ivp_vec3r(IVP_REAL(0.0), (ivp_real_t)9.83, IVP_REAL(0.0));
    ecfg.delta_psi_time = (ivp_real_t)(1.0 / 180.0);
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    wheelcar_demo_t *s = (wheelcar_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->env = w->env;
    s->torque = 700.0f;
    s->spring_k = 40000.0f;
    s->stabilizer = 1500.0f;
    s->lock = 0.3f;
    s->z0 = -20.0;

    s->ground = demo_create_box(w, NULL, 40.0, 0.5, 40.0, 0.0, NULL, 0.0, 0.5, 0.0);
    s->ramp = NULL;
    s->ramp = demo_create_box(w, NULL, 3.0, 0.2, 4.0, 0.0, &(const ivp_quatr_t){ IVP_REAL(0.10), IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.995) }, 8.0,
                              -0.1, 6.0);
    for (int i = 0; i < 3; i++) s->cone[i] = demo_create_box(w, NULL, 0.3, 0.4, 0.3, 5.0, NULL, -5.0 + 5.0 * i, -0.4, 4.0 + 6.0 * i);

    /* the body and the wheels share a collision group, so they do not collide with each other */
    ivp_object_template_t bt;
    const ivp_material_t bm = demo_material(0.6, 0.1);
    demo_template_dynamic(&bt, &bm, 800.0);
    bt.rot_inertia_is_factor = false;
    bt.rot_inertia = ivp_vec3r(IVP_REAL(1100.0), IVP_REAL(1300.0), IVP_REAL(250.0));
    bt.speed_damp_factor = IVP_REAL(0.0);
    bt.rot_speed_damp_factor = ivp_vec3r_zero();
    bt.nocoll_group_ident = "car";
    demo_template_pose(&bt, NULL, 0.0, -0.8, s->z0);
    s->body = demo_create_box_templ(w, &bt, 0.7, 0.3, 2.0);
    if (!s->body) return 1;

    ivp_car_system_template_init(&s->tmpl, 4, 2);
    s->tmpl.car_body = s->body;
    for (int i = 0; i < 4; i++) {
        const double wx = (i & 1) ? WHEEL_X : -WHEEL_X, wz = (i & 2) ? -WHEEL_Z : WHEEL_Z;
        ivp_object_template_t wt;
        const ivp_material_t wm = demo_material(0.9, 0.1);
        demo_template_dynamic(&wt, &wm, 25.0);
        wt.rot_inertia_is_factor = false;
        wt.rot_inertia = ivp_vec3r((ivp_real_t)1.225, (ivp_real_t)1.225, (ivp_real_t)1.225);
        wt.speed_damp_factor = IVP_REAL(0.0);
        wt.rot_speed_damp_factor = ivp_vec3r_zero();
        wt.nocoll_group_ident = "car";
        demo_template_pose(&wt, NULL, wx, -0.8 + WHEEL_Y, s->z0 + wz);
        s->wheel[i] = demo_create_ball_templ(w, &wt, WHEEL_R);
        if (!s->wheel[i]) return 1;
        s->tmpl.wheel_objects[i] = s->wheel[i];
        s->tmpl.wheel_radius[i] = (ivp_real_t)WHEEL_R;
        s->tmpl.wheel_pos_Bos[i] = ivp_vec3f((float)wx, (float)WHEEL_Y, (float)wz);
        s->tmpl.spring_constant[i] = (ivp_real_t)s->spring_k;
        s->tmpl.spring_damping[i] = IVP_REAL(3000.0);
        s->tmpl.spring_damping_compression[i] = IVP_REAL(3000.0);
        s->tmpl.max_body_force[i] = IVP_REAL(60000.0);
        s->tmpl.spring_pre_tension[i] = (ivp_real_t)0.05;
    }
    s->tmpl.stabilizer_constant[0] = (ivp_real_t)s->stabilizer;
    s->tmpl.stabilizer_constant[1] = (ivp_real_t)s->stabilizer;
    s->tmpl.wheel_max_rotation_speed[0] = IVP_REAL(60.0);
    s->tmpl.wheel_max_rotation_speed[1] = IVP_REAL(60.0);
    s->tmpl.body_counter_torque_factor = (ivp_real_t)0.2;
    s->car_ok = ivp_car_system_init(&s->car, &s->tmpl);
    if (!s->car_ok) {
        fprintf(stderr, "[wheelcar] ivp_car_system_init failed\n");
        return 1;
    }

    app.controls    = "Up/Down: drive   Left/Right: steer   B: handbrake   F5: torque, lock, springs";
    app.viewer_init = wheelcar_viewer_init;
    app.update      = wheelcar_update;
    app.on_reset    = wheelcar_reset;
    app.draw        = wheelcar_draw;
    app.stats       = wheelcar_stats;
    app.check       = wheelcar_check;

    int rc = demo_run(&app, argc, argv);
    ivp_car_system_destroy(&s->car);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
