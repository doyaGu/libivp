/* example_stiff_spring.c -- Regular vs stiff spring comparison
 *
 * Two side-by-side spring pairs, both start displaced (stretched 1.9 m).
 *   Left:  regular spring (ivp_actuator_spring): an explicit force,
 *          k = 5000 N/m, damping 20 N*s/m on the 2 kg cube.
 *   Right: stiff spring (ivp_controller_stiff_spring): an implicit
 *          velocity correction with DIMENSIONLESS constants in [0..1]
 *          (constant 0.5 = half of the length error corrected per PSI,
 *          about 0.5 * m / dt^2 = 4400 N/m for this cube at the 1/66 s PSI;
 *          damping 0.3).
 * The initial snap exceeds the 50 m/s anomaly limit and is clamped (anomaly
 * counter).  The explicit spring rings at 8 Hz (omega * dt = 0.76 at this
 * PSI, stable below 2), the implicit one converges within a few PSIs without
 * overshoot.  A heavy 8 kg ball then drops onto each cube: the implicit
 * spring carries it steadily, the explicit one keeps ringing and the ball
 * chatters on its cube (and is usually thrown off).
 *
 * (The stiff spring used to get k = 5000 / damping 20 as if it were in N/m:
 *  it then over-corrected the length error 5000 times per PSI and the
 *  damping factor k + d * (1 - k) = -94980 amplified the relative velocity,
 *  so the cube reached more than 1e5 m/s in a few PSIs -- the reference debug
 *  build rejects that in IVP_Core::core_plausible_check.  The balls also
 *  started right above the ceiling mounts and just landed on them.)
 *
 * Demonstrates: ivp_actuator_spring, ivp_controller_stiff_spring,
 *               explicit vs implicit spring integration, the anomaly
 *               manager speed limit (ivp_anomaly_config_t max_speed).
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/controllers/ivp_controller_stiff_spring.h>

#define REG_K       5000.0     /* N/m */
#define REG_DAMP    20.0       /* N*s/m */
#define STIFF_K     0.5        /* dimensionless [0..1] */
#define STIFF_DAMP  0.3        /* dimensionless [0..1] */
#define REST_LEN    3.0
#define MAX_SPEED   50.0       /* anomaly limit */

typedef struct stiff_demo {
    ivp_object_t            *ground, *anchor_l, *anchor_r, *cube_l, *cube_r, *ball_l, *ball_r;
    ivp_spring_state_t       reg_spring;
    ivp_stiff_spring_state_t stiff_spring;
    float                    max_reg_ext, max_stiff_ext;    /* whole run */
    float                    ring_reg, ring_stiff;          /* 0.2 .. 0.5 s: after the snap */
    float                    hit_reg, hit_stiff;            /* 0.5 .. 2 s: the balls land */
    float                    late_reg, late_stiff;          /* after 2 s */
    float                    max_speed_l, max_speed_r;
    int                      prev_anomalies;
} stiff_demo_t;

static void track_max(float *m, float v)
{
    if (v > *m) *m = v;
}

static void stiff_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    stiff_demo_t *s = (stiff_demo_t *)app->user;
    (void)v;
    if (frame == 0) return;   /* lengths are computed by the first PSI */
    double t = ivp_environment_get_time(app->world.env);
    float reg_ext = fabsf((float)(s->reg_spring.current_length - s->reg_spring.rest_length));
    float stiff_ext = fabsf((float)(s->stiff_spring.current_length - s->stiff_spring.rest_length));
    track_max(&s->max_reg_ext, reg_ext);
    track_max(&s->max_stiff_ext, stiff_ext);
    if (t >= 0.2 && t < 0.5) {
        track_max(&s->ring_reg, reg_ext);
        track_max(&s->ring_stiff, stiff_ext);
    } else if (t >= 0.5 && t < 2.0) {
        track_max(&s->hit_reg, reg_ext);
        track_max(&s->hit_stiff, stiff_ext);
    } else if (t >= 2.0) {
        track_max(&s->late_reg, reg_ext);
        track_max(&s->late_stiff, stiff_ext);
    }
    track_max(&s->max_speed_l, demo_object_speed(s->cube_l));
    track_max(&s->max_speed_r, demo_object_speed(s->cube_r));
}

static void stiff_reset(demo_app_t *app)
{
    stiff_demo_t *s = (stiff_demo_t *)app->user;
    s->max_reg_ext = s->max_stiff_ext = 0.0f;
}

static void stiff_draw(demo_app_t *app, demo_viewer_t *v)
{
    stiff_demo_t *s = (stiff_demo_t *)app->user;
    const float c_ground[]   = {0.22f, 0.24f, 0.30f};
    const float c_anchor[]   = {0.5f, 0.5f, 0.5f};
    const float c_reg[]      = {1.0f, 0.4f, 0.2f};
    const float c_stiff[]    = {0.3f, 0.8f, 1.0f};
    const float c_spring_r[] = {1.0f, 0.8f, 0.2f};
    const float c_spring_s[] = {0.2f, 1.0f, 0.8f};
    const float c_ball[]     = {0.6f, 0.6f, 0.7f};
    const float c_vel[]      = {0.2f, 1.0f, 0.4f};

    demo_draw_object(v, s->ground, c_ground);
    demo_draw_object(v, s->anchor_l, c_anchor);
    demo_draw_object(v, s->anchor_r, c_anchor);
    demo_draw_object(v, s->cube_l, c_reg);
    demo_draw_object(v, s->cube_r, c_stiff);
    {
        float a[3], b[3];
        demo_core_point_ws(s->reg_spring.obj_a, &s->reg_spring.anchor_a_os, a);
        demo_core_point_ws(s->reg_spring.obj_b, &s->reg_spring.anchor_b_os, b);
        demo_thick_line(v, a, b, c_spring_r, 2);
        demo_core_point_ws(s->stiff_spring.obj_a, &s->stiff_spring.anchor_a_os, a);
        demo_core_point_ws(s->stiff_spring.obj_b, &s->stiff_spring.anchor_b_os, b);
        demo_thick_line(v, a, b, c_spring_s, 2);
    }
    demo_draw_object(v, s->ball_l, c_ball);
    demo_draw_object(v, s->ball_r, c_ball);
    demo_draw_velocity(v, s->ball_l, c_vel);
    demo_draw_velocity(v, s->ball_r, c_vel);
    {
        float org[3] = {0, 0, 0};
        demo_axes(v, org, NULL, 1.5f);
    }

    const ivp_anomaly_manager_t *am = &app->world.env->anomaly_manager;
    demo_hud(v, c_reg, "Left  REGULAR (explicit, k=%.0f N/m, damp %.0f): len %.3f  force %.1f N  max ext %.2f",
             REG_K, REG_DAMP, (double)s->reg_spring.current_length, (double)s->reg_spring.current_force,
             (double)s->max_reg_ext);
    demo_hud(v, c_stiff, "Right STIFF (implicit, constant %.2f, damp %.2f): len %.3f  max ext %.2f",
             STIFF_K, STIFF_DAMP, (double)s->stiff_spring.current_length, (double)s->max_stiff_ext);
    {
        const float c_anom[] = {1.0f, 0.6f, 0.2f};
        int anom = am->anomalies_detected;
        demo_hud(v, anom > s->prev_anomalies ? c_anom : DEMO_COL_INFO,
                 "Anomalies: %d detected, %d corrected (max_speed=%.0f m/s)",
                 anom, am->anomalies_corrected, (double)am->config.max_speed);
        s->prev_anomalies = anom;
    }
}

static void stiff_stats(demo_app_t *app, char *buf, size_t size)
{
    stiff_demo_t *s = (stiff_demo_t *)app->user;
    float bl[3], br[3];
    demo_object_center(s->ball_l, bl);
    demo_object_center(s->ball_r, br);
    snprintf(buf, size, "reg len=%.3f force=%.1f max_ext=%.3f vmax=%.2f | stiff len=%.3f max_ext=%.3f vmax=%.2f | balls (%.2f,%.2f,%.2f)/(%.2f,%.2f,%.2f)",
             (double)s->reg_spring.current_length, (double)s->reg_spring.current_force,
             (double)s->max_reg_ext, (double)s->max_speed_l,
             (double)s->stiff_spring.current_length, (double)s->max_stiff_ext, (double)s->max_speed_r,
             (double)bl[0], (double)bl[1], (double)bl[2], (double)br[0], (double)br[1], (double)br[2]);
}

static bool stiff_check(demo_app_t *app)
{
    stiff_demo_t *s = (stiff_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    const ivp_anomaly_manager_t *am = &app->world.env->anomaly_manager;
    bool ok = true;
    ok &= demo_check(tag, am->anomalies_detected > 0 && s->max_speed_l <= (float)MAX_SPEED &&
                          s->max_speed_r <= (float)MAX_SPEED,
                     "initial snap clamped by the anomaly manager (%d anomalies, cube speeds <= %.2f / %.2f m/s)",
                     am->anomalies_detected, (double)s->max_speed_l, (double)s->max_speed_r);
    ok &= demo_check(tag, s->ring_reg > 0.1f && s->ring_stiff < 0.02f,
                     "after the snap the explicit spring rings (%.3f m), the implicit one has converged (%.4f m)",
                     (double)s->ring_reg, (double)s->ring_stiff);
    ok &= demo_check(tag, s->hit_reg > 0.05f && s->hit_stiff > 0.05f,
                     "the balls hit both cubes (extension %.3f / %.3f m)",
                     (double)s->hit_reg, (double)s->hit_stiff);
    ok &= demo_check(tag, s->late_reg < 0.3f && s->late_stiff < 0.1f,
                     "both springs stay bounded after 2 s (max extension %.3f / %.3f m)",
                     (double)s->late_reg, (double)s->late_stiff);
    float stiff_ext = (float)(s->stiff_spring.current_length - s->stiff_spring.rest_length);
    float ball_r_y = (float)s->ball_r->physical_core->pos_world_f_core_last_psi.k[1];
    float cube_r_y = (float)s->cube_r->physical_core->pos_world_f_core_last_psi.k[1];
    ok &= demo_check(tag, fabsf(stiff_ext) < 0.05f && demo_object_speed(s->cube_r) < 0.05f &&
                          fabsf(cube_r_y - 0.8f - 0.6f - ball_r_y) < 0.05f,
                     "stiff spring carries the ball steadily (extension %.3f m, cube speed %.3f, ball %.2f m above the cube centre)",
                     (double)stiff_ext, (double)demo_object_speed(s->cube_r), (double)(cube_r_y - ball_r_y));
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title        = "IVP Stiff Spring -- Regular vs Implicit";
    app.cfg.tag          = "stiff_spring";
    app.cfg.orbit_dist   = 20.0f;
    app.cfg.orbit_yaw    = 0.0f;
    app.cfg.orbit_pitch  = -15.0f;
    app.cfg.target[1]    = -7.0f;
    app.cfg.grid_extent  = 20.0f;
    app.cfg.grid_spacing = 2.0f;

    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    /* Enable anomaly detection: speeds above 50 m/s are clamped. */
    ecfg.anomaly_config.max_speed = (ivp_real_t)MAX_SPEED;
    ecfg.anomaly_config.auto_correct = true;
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    stiff_demo_t *s = (stiff_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    s->ground = demo_create_box_templ(w, &gt, 20.0, 0.5, 10.0);

    /* ── Static anchors (ceiling) ───────────────────────────────────────── */
    ivp_object_template_t at;
    ivp_object_template_init(&at);
    at.mass = 0.0f;
    at.physical_unmoveable = true;
    at.position = ivp_vec3r(IVP_REAL(-6.0), IVP_REAL(-12.0), IVP_REAL(0.0));
    s->anchor_l = demo_create_box_templ(w, &at, 0.3, 0.3, 0.3);
    at.position = ivp_vec3r(IVP_REAL(6.0), IVP_REAL(-12.0), IVP_REAL(0.0));
    s->anchor_r = demo_create_box_templ(w, &at, 0.3, 0.3, 0.3);

    /* ── Hanging cubes (displaced downward to stretch springs) ──────────── */
    ivp_object_template_t ct;
    ivp_object_template_init(&ct);
    ct.mass = IVP_REAL(2.0);
    set_box_inertia(&ct, IVP_REAL(2.0), IVP_REAL(0.8), IVP_REAL(0.8), IVP_REAL(0.8));
    ct.material.friction   = IVP_REAL(0.6);
    ct.material.elasticity = IVP_REAL(0.1);
    ct.position = ivp_vec3r(IVP_REAL(-6.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    s->cube_l = demo_create_box_templ(w, &ct, 0.8, 0.8, 0.8);
    ct.position = ivp_vec3r(IVP_REAL(6.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    s->cube_r = demo_create_box_templ(w, &ct, 0.8, 0.8, 0.8);

    /* ── Heavy stress-test balls ────────────────────────────────────────── *
     * Released right below the ceiling mounts (above them they would just
     * land on the mounts), 1.2 m above the highest point of the cubes. */
    ivp_object_template_t bt;
    ivp_object_template_init(&bt);
    bt.mass = IVP_REAL(8.0);
    set_sphere_inertia(&bt, IVP_REAL(8.0), IVP_REAL(0.6));
    bt.material.friction   = IVP_REAL(0.4);
    bt.material.elasticity = IVP_REAL(0.3);
    bt.position = ivp_vec3r(IVP_REAL(-6.0), IVP_REAL(-10.8), IVP_REAL(0.0));
    s->ball_l = demo_create_ball_templ(w, &bt, 0.6);
    bt.position = ivp_vec3r(IVP_REAL(6.0), IVP_REAL(-10.8), IVP_REAL(0.0));
    s->ball_r = demo_create_ball_templ(w, &bt, 0.6);

    /* ── Regular spring (LEFT) ──────────────────────────────────────────── */
    ivp_spring_state_init(&s->reg_spring);
    {
        ivp_vec3r_t a = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.3), IVP_REAL(0.0));
        ivp_vec3r_t b = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.8), IVP_REAL(0.0));
        ivp_spring_state_setup(&s->reg_spring, s->anchor_l, &a, s->cube_l, &b,
                               (ivp_real_t)REG_K, (ivp_real_t)REG_DAMP, (ivp_real_t)REST_LEN);
    }
    ivp_environment_add_controller(w->env, 100, ivp_controller_spring, &s->reg_spring);

    /* ── Stiff spring (RIGHT) ───────────────────────────────────────────── */
    ivp_stiff_spring_state_init(&s->stiff_spring);
    {
        ivp_vec3r_t a = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.3), IVP_REAL(0.0));
        ivp_vec3r_t b = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.8), IVP_REAL(0.0));
        ivp_stiff_spring_state_setup(&s->stiff_spring, s->anchor_r, &a, s->cube_r, &b,
                                     (ivp_real_t)STIFF_K, (ivp_real_t)STIFF_DAMP, (ivp_real_t)REST_LEN);
    }
    ivp_environment_add_controller(w->env, 100, ivp_controller_stiff_spring, &s->stiff_spring);

    app.update   = stiff_update;
    app.on_reset = stiff_reset;
    app.draw     = stiff_draw;
    app.stats    = stiff_stats;
    app.check    = stiff_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
