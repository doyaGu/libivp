/* example_vehicle.c -- Raycast vehicle demo
 *
 * A four-wheel raycast car drives on a flat ground with ramps.
 * Arrow keys steer and accelerate, B brakes (Space resets the scene in
 * all examples).  Wheel contact points are drawn as small spheres.
 * Suspension compression is visualized as coloured lines.
 *
 * Demonstrates: ivp_controller_raycast_car, 4-wheel init, steering,
 *               engine, brake, wheel contact visualization, ramps.
 */
#include "demo_app.h"
#include "demo_drive.h"

#include <ivp/controllers/ivp_controller_raycast_car.h>
#include "demo_car.h"
#include <ivp/math/ivp_fast_math.h>

#define STEER_ANGLE  0.4      /* rad */

typedef struct vehicle_demo {
    ivp_object_t   *ground, *ramp1, *ramp2, *chassis;
    ivp_object_t   *obs[4];
    ivp_car_state_t car;
    /* headless measurements */
    int             grounded_settled;    /* wheels on the ground after settling */
    float           start_pos[3];
    float           drive_speed;         /* speed after 2 s of throttle */
    float           drive_dist;
    float           heading0, heading1;  /* yaw before / after steering */
    float           brake_v0, brake_v1;  /* speed when braking starts / 1.5 s later */
    float           max_tilt;
    /* tunables, also on the F5 panel */
    float           engine;              /* N, balanced for the 20 kg chassis; reverse is half */
    float           brake;               /* N, 200 is about 1 g deceleration */
    float           spring_k, spring_damp, wheel_friction;
    demo_drive_t drive;
} vehicle_demo_t;

/* The 4-wheel defaults are 50000 N/m and 3000 N*s/m; tuned for the 20 kg
 * chassis. The compression damping must be tuned as well: with 3000 N*s/m a
 * 6 m/s landing made each shock push with 18000 N for one 1/66 s PSI and the
 * car was launched 100 m high. */
static void vehicle_apply(void *ctx)
{
    vehicle_demo_t *s = (vehicle_demo_t *)ctx;
    for (int i = 0; i < s->car.n_wheels; i++) {
        s->car.wheels[i].spring_constant      = s->spring_k;
        s->car.wheels[i].spring_damp_relax    = s->spring_damp;
        s->car.wheels[i].spring_damp_compress = s->spring_damp;
        s->car.wheels[i].friction_of_wheel    = s->wheel_friction;
    }
}

static void vehicle_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    vehicle_demo_t *s = (vehicle_demo_t *)app->user;
    demo_panel_t *p = &v->panel;
    demo_panel_add_float(p, "engine (N)",     &s->engine,         50.0f, 500.0f,  NULL, NULL);
    demo_panel_add_float(p, "brake (N)",      &s->brake,          20.0f, 600.0f,  NULL, NULL);
    demo_panel_add_float(p, "spring k",       &s->spring_k,       200.0f, 2500.0f, vehicle_apply, s);
    demo_panel_add_float(p, "spring damping", &s->spring_damp,    5.0f,  200.0f,  vehicle_apply, s);
    demo_panel_add_float(p, "tyre friction",  &s->wheel_friction, 0.2f,  2.5f,    vehicle_apply, s);
    demo_drive_panel(p, &s->drive);
}

static int wheels_grounded(const vehicle_demo_t *s)
{
    int n = 0;
    for (int i = 0; i < s->car.n_wheels; i++)
        if (s->car.wheels[i].grounded) n++;
    return n;
}

/* yaw of the car's forward (+z core) axis in the ground plane, degrees */
static float heading_deg(const vehicle_demo_t *s)
{
    float rot[9];
    quat_to_col_rot(&s->chassis->physical_core->q_world_f_core_last_psi, rot);
    return atan2f(rot[6], rot[8]) * (180.0f / 3.14159265f);   /* column 2 = local z */
}

static void apply_input(vehicle_demo_t *s, float steer, float engine, float brake)
{
    ivp_car_set_steering(&s->car, (ivp_real_t)steer);
    ivp_car_set_engine(&s->car, (ivp_real_t)engine);
    ivp_car_set_brake(&s->car, (ivp_real_t)brake);
}

static void vehicle_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    vehicle_demo_t *s = (vehicle_demo_t *)app->user;
    float tilt = demo_object_tilt_deg(s->chassis);
    if (tilt > s->max_tilt) s->max_tilt = tilt;
    if (v) {
        demo_drive_input_t in;
        demo_drive_read(&s->drive, &in);
        demo_drive_step(&s->drive, &in, v->frame_dt);
        float rot[9];
        quat_to_col_rot(&s->chassis->physical_core->q_world_f_core_last_psi, rot);
        const ivp_vec3r_t *sp = &s->chassis->physical_core->speed;
        const float fwd = (float)(sp->k[0] * rot[6] + sp->k[1] * rot[7] + sp->k[2] * rot[8]);
        /* + steer is a turn to the left here; the lock shrinks with the speed */
        const float steer = -s->drive.steer * (float)STEER_ANGLE * demo_drive_lock(fwd, 14.0f);
        float engine = s->drive.throttle > 0.0f ? s->engine * s->drive.throttle : 0.0f, brake = 0.0f;
        if (in.throttle < 0.0f) {
            /* like a pedal: brake while rolling forward, reverse once stopped */
            if (fwd > 0.8f) brake = s->brake * -in.throttle;
            else engine = 0.5f * s->engine * s->drive.throttle;
        }
        if (in.brake) brake = s->brake;
        apply_input(s, steer, engine, brake);
        return;
    }
    /* scripted: settle 1 s, throttle 2 s, throttle + steer left 1.5 s, brake */
    float steer = 0.0f, engine = 0.0f, brake = 0.0f;
    if (frame == 60) {
        s->grounded_settled = wheels_grounded(s);
        demo_object_center(s->chassis, s->start_pos);
    }
    if (frame >= 60 && frame < 240) engine = s->engine;
    if (frame == 180) {
        float p[3];
        demo_object_center(s->chassis, p);
        s->drive_speed = demo_object_speed(s->chassis);
        s->drive_dist = demo_distance(p, s->start_pos);
        s->heading0 = heading_deg(s);
    }
    if (frame >= 180 && frame < 240) steer = (float)STEER_ANGLE;
    if (frame == 240) { s->heading1 = heading_deg(s); s->brake_v0 = demo_object_speed(s->chassis); }
    if (frame >= 240) brake = s->brake;
    if (frame == 330) s->brake_v1 = demo_object_speed(s->chassis);
    apply_input(s, steer, engine, brake);
}

static void vehicle_update_view(demo_app_t *app, demo_viewer_t *v)
{
    vehicle_demo_t *s = (vehicle_demo_t *)app->user;
    demo_drive_follow_object(v, &s->drive, s->chassis);
}

static void vehicle_draw(demo_app_t *app, demo_viewer_t *v)
{
    vehicle_demo_t *s = (vehicle_demo_t *)app->user;
    const float c_ground[]  = {0.22f, 0.24f, 0.30f};
    const float c_chassis[] = {0.2f, 0.45f, 0.9f};
    const float c_ramp[]    = {0.6f, 0.5f, 0.3f};
    const float c_obs[]     = {0.9f, 0.5f, 0.2f};
    const float c_wheel_g[] = {0.2f, 1.0f, 0.3f};  /* grounded */
    const float c_wheel_a[] = {1.0f, 0.3f, 0.2f};  /* airborne */
    const float c_susp[]    = {1.0f, 1.0f, 0.2f};
    const float c_vel[]     = {0.2f, 1.0f, 0.4f};

    demo_draw_object(v, s->ground, c_ground);
    demo_draw_object(v, s->ramp1, c_ramp);
    demo_draw_object(v, s->ramp2, c_ramp);
    for (int i = 0; i < 4; i++) demo_draw_object(v, s->obs[i], c_obs);
    demo_draw_object(v, s->chassis, c_chassis);
    if (demo_object_speed(s->chassis) > 0.5f) demo_draw_velocity(v, s->chassis, c_vel);

    /* solid view: the car model; wire view: wheel contact points and suspension
     * (hard points are in the chassis core system) */
    demo_draw_car(v, s->chassis, &s->car, c_chassis);
    for (int i = 0; i < s->car.n_wheels && !v->solid; i++) {
        const ivp_car_wheel_t *wh = &s->car.wheels[i];
        ivp_vec3r_t hp_cs = ivp_vec3r(wh->hp_cs.k[0], wh->hp_cs.k[1], wh->hp_cs.k[2]);
        float hp[3];
        demo_core_point_ws(s->chassis, &hp_cs, hp);
        if (wh->grounded) {
            float cp[3];
            vec3r_to_float3(&wh->contact_point_ws, cp);
            demo_sphere(v, cp, wh->wheel_radius, c_wheel_g);
            demo_line(v, hp, cp, c_susp);
        } else {
            demo_sphere(v, hp, wh->wheel_radius, c_wheel_a);
        }
    }
    {
        float org[3] = {0, 0, 0};
        demo_axes(v, org, NULL, 2.0f);
    }
    demo_hud(v, DEMO_COL_INFO, "Speed: %.1f m/s  Grounded: %d/%d", (double)demo_object_speed(s->chassis),
             wheels_grounded(s), s->car.n_wheels);
    demo_hud(v, c_susp, "Steer: %.2f  Engine: %.0f N  Brake: %.0f N",
             (double)s->car.steering_angle, (double)s->car.engine_force, (double)s->car.brake_force);
}

static void vehicle_stats(demo_app_t *app, char *buf, size_t size)
{
    vehicle_demo_t *s = (vehicle_demo_t *)app->user;
    float p[3];
    demo_object_center(s->chassis, p);
    snprintf(buf, size, "pos=(%.2f,%.2f,%.2f) speed=%.3f heading=%.1f tilt=%.1f steer=%.2f engine=%.0f brake=%.0f grounded=%d/%d",
             (double)p[0], (double)p[1], (double)p[2], (double)demo_object_speed(s->chassis),
             (double)heading_deg(s), (double)demo_object_tilt_deg(s->chassis), (double)s->car.steering_angle,
             (double)s->car.engine_force, (double)s->car.brake_force, wheels_grounded(s), s->car.n_wheels);
}

static bool vehicle_check(demo_app_t *app)
{
    vehicle_demo_t *s = (vehicle_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, s->grounded_settled == 4, "car settled on its suspension (%d/4 wheels on the ground)",
                     s->grounded_settled);
    ok &= demo_check(tag, s->drive_speed > 3.0f && s->drive_dist > 3.0f,
                     "engine drives the car: %.2f m/s, %.2f m after 2 s", (double)s->drive_speed, (double)s->drive_dist);
    float turn = s->heading1 - s->heading0;
    ok &= demo_check(tag, turn > 15.0f, "steering left turned the car by %.1f degrees in 1 s", (double)turn);
    ok &= demo_check(tag, s->brake_v0 > 3.0f && s->brake_v1 < 0.3f,
                     "brake stopped the car: %.2f -> %.2f m/s in 1.5 s", (double)s->brake_v0, (double)s->brake_v1);
    ok &= demo_check(tag, s->max_tilt < 20.0f && wheels_grounded(s) == 4,
                     "car stayed upright on its wheels (max tilt %.1f degrees)", (double)s->max_tilt);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title        = "IVP Vehicle -- Raycast Car";
    app.cfg.tag          = "vehicle";
    app.cfg.orbit_dist   = 20.0f;
    app.cfg.orbit_yaw    = 30.0f;
    app.cfg.orbit_pitch  = -25.0f;
    app.cfg.target[1]    = -2.0f;
    app.cfg.grid_extent  = 40.0f;
    app.cfg.grid_spacing = 4.0f;

    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    vehicle_demo_t *s = (vehicle_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    demo_drive_defaults(&s->drive);
    app.user = s;
    demo_world_t *w = &app.world;

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    s->ground = demo_create_box_templ(w, &gt, 40.0, 0.5, 40.0);

    /* ── Ramps (15 degrees around Z, axis-angle quaternion) ─────────────── */
    {
        ivp_object_template_t rpt;
        ivp_object_template_init(&rpt);
        rpt.mass = 0.0f;
        rpt.physical_unmoveable = true;
        ivp_real_t ramp_half = IVP_REAL(15.0) * IVP_REAL(3.14159265) / IVP_REAL(360.0);
        rpt.position = ivp_vec3r(IVP_REAL(15.0), IVP_REAL(-1.0), IVP_REAL(0.0));
        rpt.orientation = ivp_quatr_make(IVP_REAL(0.0), IVP_REAL(0.0), ivp_sin(ramp_half), ivp_cos(ramp_half));
        ivp_quat_fast_normalize(&rpt.orientation);
        s->ramp1 = demo_create_box_templ(w, &rpt, 4.0, 0.3, 3.0);
        rpt.position = ivp_vec3r(IVP_REAL(-15.0), IVP_REAL(-1.0), IVP_REAL(0.0));
        rpt.orientation = ivp_quatr_make(IVP_REAL(0.0), IVP_REAL(0.0), -ivp_sin(ramp_half), ivp_cos(ramp_half));
        ivp_quat_fast_normalize(&rpt.orientation);
        s->ramp2 = demo_create_box_templ(w, &rpt, 4.0, 0.3, 3.0);
    }

    /* ── Obstacle boxes ─────────────────────────────────────────────────── */
    for (int i = 0; i < 4; i++) {
        ivp_object_template_t ot;
        ivp_object_template_init(&ot);
        ot.mass = IVP_REAL(1.0);
        set_box_inertia(&ot, IVP_REAL(1.0), IVP_REAL(0.8), IVP_REAL(0.8), IVP_REAL(0.8));
        ot.material.friction   = IVP_REAL(0.6);
        ot.material.elasticity = IVP_REAL(0.3);
        float angle = 6.2832f * (float)i / 4.0f;
        ot.position = ivp_vec3r((ivp_real_t)(8.0f * cosf(angle)), IVP_REAL(-1.3), (ivp_real_t)(8.0f * sinf(angle)));
        s->obs[i] = demo_create_box_templ(w, &ot, 0.8, 0.8, 0.8);
    }

    /* ── Chassis (box) ──────────────────────────────────────────────────── */
    ivp_object_template_t cht;
    ivp_object_template_init(&cht);
    cht.mass = IVP_REAL(20.0);
    set_box_inertia(&cht, IVP_REAL(20.0), IVP_REAL(1.0), IVP_REAL(0.4), IVP_REAL(2.0));
    cht.material.friction   = IVP_REAL(0.3);
    cht.material.elasticity = IVP_REAL(0.1);
    cht.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-3.0), IVP_REAL(0.0));
    s->chassis = demo_create_box_templ(w, &cht, 1.0, 0.4, 2.0);

    /* ── Raycast car ────────────────────────────────────────────────────── *
     * registers the controller (front axle at +z, steered; the engine
     * drives the rear axle); without a ray caster the wheels cast rays
     * against the environment's objects */
    if (!ivp_car_init_4wheel(&s->car, s->chassis,
                             IVP_REAL(3.0),    /* wheelbase */
                             IVP_REAL(1.8),    /* track width */
                             IVP_REAL(0.35),   /* wheel radius */
                             IVP_REAL(0.6))) { /* suspension length */
        fprintf(stderr, "ivp_car_init_4wheel failed\n");
        return 1;
    }
    s->engine = 200.0f;
    s->brake = 200.0f;
    s->spring_k = 800.0f;
    s->spring_damp = 60.0f;
    s->wheel_friction = 1.2f;
    vehicle_apply(s);

    app.controls = "Up/Down: drive, brake, reverse   Left/Right: steer   B: handbrake";
    app.viewer_init = vehicle_viewer_init;
    app.update   = vehicle_update;
    app.update_view = vehicle_update_view;
    app.draw     = vehicle_draw;
    app.stats    = vehicle_stats;
    app.check    = vehicle_check;

    int rc = demo_run(&app, argc, argv);
    ivp_car_destroy(&s->car);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
