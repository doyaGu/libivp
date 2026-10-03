/* example_check_distance.c -- Distance-triggered rotation motor
 *
 * A pendulum swings above a small box.  When the pendulum bob comes
 * within a distance threshold, a callback activates a rotation motor
 * on the box.  When the pendulum swings away, the motor stops.
 * A circle on the ground visualizes the threshold radius.
 *
 * The ball-socket joint sits at the pivot: the bob is attached to it at
 * its start distance (7.1 m rod).  (It used to join the pivot centre to the
 * top of the bob, 6.4 m apart at the start: the joint yanked the bob up to
 * the pivot, where it hung 8 m above the box and never reached the range.)
 *
 * Demonstrates: ivp_actuator_check_dist, distance threshold callbacks,
 *               ivp_constraint (ball_socket), ivp_actuator_rot_mot,
 *               combined controller interaction.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_actuator_check_dist.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/controllers/ivp_actuator_rot_mot.h>

#define RANGE        4.0
#define MOTOR_POWER  8.0
/* max torque: the box's base (friction 0.8 x 0.5 on the ground) needs
 * ~9 N*m to turn; 5 N*m (as before) could not move it at all */
#define MOTOR_TORQUE 15.0

/* ── Distance event callback ────────────────────────────────────────────── */

typedef struct {
    ivp_rot_mot_state_t *motor;
    ivp_real_t           motor_power;
    bool                 motor_active;
    int                  enter_count;
    int                  exit_count;
} dist_event_ctx_t;

static void on_distance_event(void *ctx, ivp_object_t *obj_a, ivp_object_t *obj_b,
                              ivp_real_t distance, bool is_inside_range)
{
    (void)obj_a; (void)obj_b; (void)distance;
    dist_event_ctx_t *dc = ctx;
    if (is_inside_range) {
        ivp_rot_mot_set_power(dc->motor, dc->motor_power);
        dc->motor_active = true;
        dc->enter_count++;
    } else {
        ivp_rot_mot_set_power(dc->motor, IVP_REAL(0.0));
        dc->motor_active = false;
        dc->exit_count++;
    }
}

typedef struct check_demo {
    ivp_object_t          *ground, *pivot, *bob, *spin_box;
    ivp_constraint_state_t pend_joint;
    ivp_rot_mot_state_t    motor;
    ivp_check_dist_state_t dist_check;
    dist_event_ctx_t       dist_ctx;
    float                  rod_length;
    float                  max_rod_error;
    float                  max_spin_on;        /* box |omega_y| while the motor is on */
    float                  max_spin_off;       /* ... 0.5 s or more after it was switched off */
    double                 off_since;
    float                  min_dist, max_dist;
    int                    state_mismatch;     /* motor flag != inside the range */
} check_demo_t;

static float box_spin(const check_demo_t *s)
{
    return fabsf(s->spin_box->physical_core->rot_speed.k[1]);
}

static void check_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    check_demo_t *s = (check_demo_t *)app->user;
    (void)v;
    if (frame == 0) return;
    float rod = demo_object_distance(s->pivot, s->bob);
    float err = fabsf(rod - s->rod_length);
    if (err > s->max_rod_error) s->max_rod_error = err;
    double t = ivp_environment_get_time(app->world.env);
    if (s->dist_ctx.motor_active) {
        if (box_spin(s) > s->max_spin_on) s->max_spin_on = box_spin(s);
        s->off_since = -1.0;
    } else {
        if (s->off_since < 0.0) s->off_since = t;
        if (t - s->off_since > 0.5 && box_spin(s) > s->max_spin_off) s->max_spin_off = box_spin(s);
    }
    float d = (float)s->dist_check.current_distance;
    if (d < s->min_dist) s->min_dist = d;
    if (d > s->max_dist) s->max_dist = d;
    if (s->dist_ctx.motor_active != (s->dist_check.is_outside == 0)) s->state_mismatch++;
}

static void check_reset(demo_app_t *app)
{
    check_demo_t *s = (check_demo_t *)app->user;
    /* the bob was beamed back to its start pose: re-evaluate the range now
     * (fires the callback, i.e. switches the motor, on a change) */
    ivp_check_dist_set_range(&s->dist_check, s->dist_check.range);
}

static void check_draw(demo_app_t *app, demo_viewer_t *v)
{
    check_demo_t *s = (check_demo_t *)app->user;
    const float c_ground[]  = {0.22f, 0.24f, 0.30f};
    const float c_pivot[]   = {0.5f, 0.5f, 0.5f};
    const float c_bob[]     = {0.2f, 0.8f, 1.0f};
    const float c_box_on[]  = {0.2f, 1.0f, 0.3f};
    const float c_box_off[] = {1.0f, 0.5f, 0.1f};
    const float c_range[]   = {1.0f, 1.0f, 0.2f};
    const float c_link[]    = {0.6f, 0.6f, 0.6f};
    const float c_vel[]     = {0.2f, 1.0f, 0.4f};

    demo_draw_object(v, s->ground, c_ground);
    demo_draw_object(v, s->pivot, c_pivot);
    demo_draw_object(v, s->bob, c_bob);
    {
        float pp[3], pb[3];
        demo_object_center(s->pivot, pp);
        demo_object_center(s->bob, pb);
        demo_thick_line(v, pp, pb, c_link, 2);
    }
    demo_draw_object(v, s->spin_box, s->dist_ctx.motor_active ? c_box_on : c_box_off);

    /* Threshold circle (on the ground plane, 16 segments) */
    float box[3];
    demo_object_center(s->spin_box, box);
    for (int i = 0; i < 16; i++) {
        float a0 = 6.2832f * (float)i / 16.0f;
        float a1 = 6.2832f * (float)(i + 1) / 16.0f;
        float r = (float)RANGE;
        float p0[3] = {box[0] + r * cosf(a0), -0.6f, box[2] + r * sinf(a0)};
        float p1[3] = {box[0] + r * cosf(a1), -0.6f, box[2] + r * sinf(a1)};
        demo_line(v, p0, p1, c_range);
    }
    demo_draw_velocity(v, s->bob, c_vel);
    {
        float org[3] = {0, 0, 0};
        demo_axes(v, org, NULL, 1.5f);
    }

    demo_hud(v, c_range, "Distance: %.2f m  Threshold: %.1f m  (enters %d, exits %d)",
             (double)s->dist_check.current_distance, RANGE, s->dist_ctx.enter_count, s->dist_ctx.exit_count);
    demo_hud(v, s->dist_ctx.motor_active ? c_box_on : c_box_off, "Motor: %s  Speed: %.1f rad/s",
             s->dist_ctx.motor_active ? "ON" : "OFF", (double)s->motor.current_speed);
}

static void check_stats(demo_app_t *app, char *buf, size_t size)
{
    check_demo_t *s = (check_demo_t *)app->user;
    float pb[3];
    demo_object_center(s->bob, pb);
    snprintf(buf, size, "dist=%.3f inside=%d motor=%d enters=%d exits=%d box_spin=%.3f bob=(%.2f,%.2f,%.2f) rod=%.3f",
             (double)s->dist_check.current_distance, 1 - s->dist_check.is_outside,
             s->dist_ctx.motor_active ? 1 : 0, s->dist_ctx.enter_count, s->dist_ctx.exit_count,
             (double)box_spin(s), (double)pb[0], (double)pb[1], (double)pb[2],
             (double)demo_object_distance(s->pivot, s->bob));
}

static bool check_check(demo_app_t *app)
{
    check_demo_t *s = (check_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, s->max_rod_error < 0.05f,
                     "ball-socket joint keeps the pendulum length (%.3f m, max error %.4f m)",
                     (double)s->rod_length, (double)s->max_rod_error);
    ok &= demo_check(tag, s->min_dist < 2.5f && s->max_dist > 5.0f,
                     "bob swings past the box (distance %.2f .. %.2f m, range %.1f m)",
                     (double)s->min_dist, (double)s->max_dist, RANGE);
    ok &= demo_check(tag, s->dist_ctx.enter_count >= 3 && s->dist_ctx.exit_count >= 2,
                     "range events: %d enters, %d exits", s->dist_ctx.enter_count, s->dist_ctx.exit_count);
    ok &= demo_check(tag, s->state_mismatch == 0, "motor switched on exactly while inside the range (%d mismatching frames)",
                     s->state_mismatch);
    ok &= demo_check(tag, s->max_spin_on > 0.5f && s->max_spin_off < 0.05f,
                     "box spins while the motor is on (max %.2f rad/s) and stops when it is off (%.3f rad/s)",
                     (double)s->max_spin_on, (double)s->max_spin_off);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title        = "IVP Check Distance -- Motor Trigger";
    app.cfg.tag          = "check_distance";
    app.cfg.orbit_dist   = 25.0f;
    app.cfg.orbit_yaw    = 30.0f;
    app.cfg.orbit_pitch  = -20.0f;
    app.cfg.target[1]    = -5.0f;
    app.cfg.grid_extent  = 15.0f;
    app.cfg.grid_spacing = 2.0f;

    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    check_demo_t *s = (check_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    s->ground = demo_create_box_templ(w, &gt, 15.0, 0.5, 15.0);

    /* ── Pendulum pivot (static) ────────────────────────────────────────── */
    ivp_object_template_t pvt;
    ivp_object_template_init(&pvt);
    pvt.mass = 0.0f;
    pvt.physical_unmoveable = true;
    pvt.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-10.0), IVP_REAL(0.0));
    s->pivot = demo_create_box_templ(w, &pvt, 0.3, 0.3, 0.3);

    /* ── Pendulum bob (heavy sphere), displaced to the side so it swings ── */
    ivp_object_template_t bobt;
    ivp_object_template_init(&bobt);
    bobt.mass = IVP_REAL(5.0);
    set_sphere_inertia(&bobt, IVP_REAL(5.0), IVP_REAL(0.6));
    bobt.material.friction   = IVP_REAL(0.3);
    bobt.material.elasticity = IVP_REAL(0.1);
    bobt.position = ivp_vec3r(IVP_REAL(5.0), IVP_REAL(-5.0), IVP_REAL(0.0));
    s->bob = demo_create_ball_templ(w, &bobt, 0.6);

    /* ── Ball-socket constraint (pendulum joint at the pivot) ───────────── */
    {
        ivp_vec3r_t a_anchor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t b_anchor = ivp_vec3r(IVP_REAL(-5.0), IVP_REAL(-5.0), IVP_REAL(0.0));  /* pivot - bob */
        ivp_constraint_init_ball_socket(&s->pend_joint, s->pivot, &a_anchor, s->bob, &b_anchor);
    }
    ivp_environment_add_controller(w->env, 50, ivp_controller_constraint, &s->pend_joint);
    s->rod_length = demo_object_distance(s->pivot, s->bob);

    /* ── Spinning box (target on ground) ────────────────────────────────── */
    ivp_object_template_t bxt;
    ivp_object_template_init(&bxt);
    bxt.mass = IVP_REAL(2.0);
    set_box_inertia(&bxt, IVP_REAL(2.0), IVP_REAL(0.8), IVP_REAL(0.4), IVP_REAL(0.8));
    bxt.material.friction   = IVP_REAL(0.8);
    bxt.material.elasticity = IVP_REAL(0.05);
    bxt.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.9), IVP_REAL(0.0));
    s->spin_box = demo_create_box_templ(w, &bxt, 0.8, 0.4, 0.8);

    /* ── Rotation motor on the box (vertical axis), off until triggered ─── */
    ivp_rot_mot_state_init(&s->motor);
    {
        ivp_vec3r_t ma = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t mb = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
        ivp_rot_mot_state_setup(&s->motor, NULL, &ma, s->spin_box, &mb,
                                IVP_REAL(0.0), (ivp_real_t)MOTOR_TORQUE, IVP_REAL(6.0));
    }
    ivp_environment_add_controller(w->env, 100, ivp_controller_rot_mot, &s->motor);

    /* ── Distance checker (bob <-> spin_box centres, threshold 4 m) ─────── */
    s->dist_ctx.motor = &s->motor;
    s->dist_ctx.motor_power = (ivp_real_t)MOTOR_POWER;
    ivp_check_dist_state_init(&s->dist_check);
    {
        /* anchors at the object centres (world positions at setup) */
        ivp_vec3r_t pa = s->bob->physical_core->pos_world_f_core_last_psi;
        ivp_vec3r_t pb = s->spin_box->physical_core->pos_world_f_core_last_psi;
        ivp_check_dist_state_setup(&s->dist_check, s->bob, &pa, s->spin_box, &pb, (ivp_real_t)RANGE);
    }
    ivp_check_dist_set_callback(&s->dist_check, on_distance_event, &s->dist_ctx);
    ivp_environment_add_controller(w->env, 100, ivp_controller_check_dist, &s->dist_check);
    s->min_dist = 1e9f;
    s->off_since = -1.0;

    app.update   = check_update;
    app.on_reset = check_reset;
    app.draw     = check_draw;
    app.stats    = check_stats;
    app.check    = check_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
