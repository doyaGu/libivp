/* example_force_actuator.c -- Magnetic force actuator demo
 *
 * Two cubes connected by a force actuator that pulls them together.
 * Every 5 seconds (simulated time) the magnet toggles on/off.  Force
 * direction arrows and distance readout shown in HUD.
 *
 * The force acts along the line between the two anchors; like the
 * reference IVP_Actuator_Force a positive force pushes the first object
 * away from the second one, so the attracting magnet uses a negative force.
 *
 * Demonstrates: ivp_actuator_force, force toggling, anchor-based
 *               force application, HUD text, velocity arrows.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_actuator_force.h>

/* 15 N attraction on 3 kg cubes = 5 m/s^2.  Note: real magnetic force has
 * inverse-square falloff with distance; this constant force is a
 * simplification for demonstration. */
#define MAGNET_FORCE   (-15.0)
#define TOGGLE_PERIOD  5.0
#define CUBE_HS        1.0
#define PUSH_SPEED     3.0      /* P: push the cubes apart (m/s each) */

typedef struct force_demo {
    ivp_object_t              *ground, *cube_a, *cube_b;
    ivp_force_actuator_state_t fa;
    bool                       magnet_on;
    int                        toggle_count;
    double                     next_toggle;
    float                      start_dist;
    float                      min_dist_on;      /* while the magnet is on */
    float                      dist_at_toggle;   /* when it was switched off */
    int                        n_pushes;
    float                      dist_after_push_on;   /* max distance after the push, magnet on */
} force_demo_t;

static float anchor_distance(const force_demo_t *s)
{
    float pa[3], pb[3];
    demo_core_point_ws(s->fa.obj_a, &s->fa.anchor_a_os, pa);
    demo_core_point_ws(s->fa.obj_b, &s->fa.anchor_b_os, pb);
    return demo_distance(pa, pb);
}

static void set_magnet(force_demo_t *s, bool on)
{
    s->magnet_on = on;
    ivp_force_actuator_set_force(&s->fa, on ? (ivp_real_t)MAGNET_FORCE : IVP_REAL(0.0));
}

static void push_apart(demo_app_t *app)
{
    force_demo_t *s = (force_demo_t *)app->user;
    demo_async_add_speed_ws(app->world.env, s->cube_a, -PUSH_SPEED, 0.0, 0.0);
    demo_async_add_speed_ws(app->world.env, s->cube_b, PUSH_SPEED, 0.0, 0.0);
    s->n_pushes++;
    printf("[force_actuator] t=%.2f push apart (magnet %s)\n", ivp_environment_get_time(app->world.env),
           s->magnet_on ? "ON" : "OFF");
}

static void force_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    force_demo_t *s = (force_demo_t *)app->user;
    if (v) {
        if (demo_key_just_pressed(v, SDL_SCANCODE_P)) push_apart(app);
    } else {
        /* once while the magnet is on, once while it is off */
        if (frame == 180 || frame == 360) push_apart(app);
    }
    /* Toggle the magnet every 5 seconds of simulated time */
    double t = ivp_environment_get_time(app->world.env);
    if (t >= s->next_toggle) {
        s->next_toggle += TOGGLE_PERIOD;
        set_magnet(s, !s->magnet_on);
        s->toggle_count++;
        if (!s->magnet_on) s->dist_at_toggle = anchor_distance(s);
        printf("[force_actuator] t=%.2f magnet %s\n", t, s->magnet_on ? "ON" : "OFF");
    }
    float d = anchor_distance(s);
    if (s->magnet_on) {
        if (d < s->min_dist_on) s->min_dist_on = d;
        if (s->n_pushes > 0 && d > s->dist_after_push_on) s->dist_after_push_on = d;
    }
}

static void force_reset(demo_app_t *app)
{
    force_demo_t *s = (force_demo_t *)app->user;
    set_magnet(s, true);
    s->next_toggle = ivp_environment_get_time(app->world.env) + TOGGLE_PERIOD;
}

static void force_draw(demo_app_t *app, demo_viewer_t *v)
{
    force_demo_t *s = (force_demo_t *)app->user;
    const float c_ground[] = {0.22f, 0.24f, 0.30f};
    const float c_cube_a[] = {1.0f, 0.4f, 0.2f};
    const float c_cube_b[] = {0.3f, 0.5f, 1.0f};
    const float c_force[]  = {1.0f, 1.0f, 0.2f};
    const float c_vel[]    = {0.2f, 1.0f, 0.4f};
    const float c_on[]     = {0.2f, 1.0f, 0.3f};
    const float c_off[]    = {1.0f, 0.3f, 0.2f};

    demo_draw_object(v, s->ground, c_ground);
    demo_draw_object(v, s->cube_a, c_cube_a);
    demo_draw_object(v, s->cube_b, c_cube_b);
    if (s->magnet_on) {
        /* the pull on each cube, towards the other anchor */
        float pa[3], pb[3];
        demo_core_point_ws(s->fa.obj_a, &s->fa.anchor_a_os, pa);
        demo_core_point_ws(s->fa.obj_b, &s->fa.anchor_b_os, pb);
        float mid[3] = {(pa[0] + pb[0]) * 0.5f, (pa[1] + pb[1]) * 0.5f, (pa[2] + pb[2]) * 0.5f};
        demo_arrow(v, pa, mid, c_force, 0.4f);
        demo_arrow(v, pb, mid, c_force, 0.4f);
    }
    demo_draw_velocity(v, s->cube_a, c_vel);
    demo_draw_velocity(v, s->cube_b, c_vel);
    {
        float org[3] = {0, 0, 0};
        demo_axes(v, org, NULL, 1.5f);
    }

    demo_hud(v, DEMO_COL_INFO, "Time: %.1fs", (double)ivp_environment_get_time(app->world.env));
    demo_hud(v, c_force, "Force: %.1f N (negative = attraction)  Distance: %.2f m",
             (double)s->fa.force, (double)anchor_distance(s));
    demo_hud(v, s->magnet_on ? c_on : c_off, "Magnet: %s  (toggles every %.0f s, %d toggles)",
             s->magnet_on ? "ON" : "OFF", TOGGLE_PERIOD, s->toggle_count);
}

static void force_stats(demo_app_t *app, char *buf, size_t size)
{
    force_demo_t *s = (force_demo_t *)app->user;
    float pa[3], pb[3];
    demo_object_center(s->cube_a, pa);
    demo_object_center(s->cube_b, pb);
    snprintf(buf, size, "dist=%.3f force=%.1f on=%d toggles=%d a=(%.2f,%.2f) b=(%.2f,%.2f) speed_a=%.3f speed_b=%.3f",
             (double)anchor_distance(s), (double)s->fa.force, s->magnet_on ? 1 : 0, s->toggle_count,
             (double)pa[0], (double)pa[1], (double)pb[0], (double)pb[1],
             (double)demo_object_speed(s->cube_a), (double)demo_object_speed(s->cube_b));
}

static bool force_check(demo_app_t *app)
{
    force_demo_t *s = (force_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    float d = anchor_distance(s);
    ok &= demo_check(tag, s->min_dist_on < 0.1f, "magnet pulled the cubes together (anchor distance %.2f -> %.3f m)",
                     (double)s->start_dist, (double)s->min_dist_on);
    ok &= demo_check(tag, s->dist_after_push_on > 0.5f && s->dist_at_toggle < 0.1f,
                     "pushed apart while on (%.2f m), the magnet pulled them back (%.3f m when switched off)",
                     (double)s->dist_after_push_on, (double)s->dist_at_toggle);
    ok &= demo_check(tag, s->toggle_count == 1 && !s->magnet_on && s->fa.force == IVP_REAL(0.0),
                     "magnet toggled off after %.0f s (%d toggle, force %.1f N)", TOGGLE_PERIOD,
                     s->toggle_count, (double)s->fa.force);
    ok &= demo_check(tag, d > 1.5f && demo_object_speed(s->cube_a) < 0.01f && demo_object_speed(s->cube_b) < 0.01f,
                     "pushed apart while off, the cubes stay apart (%.2f m, at rest)", (double)d);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title        = "IVP Force Actuator -- Magnet Toggle";
    app.cfg.tag          = "force_actuator";
    app.cfg.orbit_dist   = 22.0f;
    app.cfg.orbit_yaw    = 20.0f;
    app.cfg.orbit_pitch  = -20.0f;
    app.cfg.target[1]    = -3.0f;
    app.cfg.grid_extent  = 15.0f;
    app.cfg.grid_spacing = 2.0f;

    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    force_demo_t *s = (force_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    s->ground = demo_create_box_templ(w, &gt, 15.0, 0.5, 10.0);

    /* ── Two cubes ~8m apart ────────────────────────────────────────────── */
    ivp_object_template_t ct;
    ivp_object_template_init(&ct);
    ct.mass = IVP_REAL(3.0);
    set_box_inertia(&ct, IVP_REAL(3.0), (ivp_real_t)CUBE_HS, (ivp_real_t)CUBE_HS, (ivp_real_t)CUBE_HS);
    ct.material.friction   = IVP_REAL(0.6);
    ct.material.elasticity = IVP_REAL(0.2);
    ct.position = ivp_vec3r(IVP_REAL(-4.0), IVP_REAL(-2.0), IVP_REAL(0.0));
    s->cube_a = demo_create_box_templ(w, &ct, CUBE_HS, CUBE_HS, CUBE_HS);
    ct.position = ivp_vec3r(IVP_REAL(4.0), IVP_REAL(-2.0), IVP_REAL(0.0));
    s->cube_b = demo_create_box_templ(w, &ct, CUBE_HS, CUBE_HS, CUBE_HS);

    /* ── Force actuator between the facing faces ────────────────────────── */
    ivp_force_actuator_state_init(&s->fa);
    {
        ivp_vec3r_t anchor_a = ivp_vec3r((ivp_real_t)CUBE_HS, IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t anchor_b = ivp_vec3r(-(ivp_real_t)CUBE_HS, IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_force_actuator_state_setup(&s->fa, s->cube_a, &anchor_a, s->cube_b, &anchor_b,
                                       (ivp_real_t)MAGNET_FORCE, true, true);
    }
    ivp_environment_add_controller(w->env, 100, ivp_controller_force_actuator, &s->fa);
    s->magnet_on = true;
    s->next_toggle = TOGGLE_PERIOD;
    s->start_dist = anchor_distance(s);
    s->min_dist_on = 1e9f;

    app.controls = "P: push the cubes apart";
    app.update   = force_update;
    app.on_reset = force_reset;
    app.draw     = force_draw;
    app.stats    = force_stats;
    app.check    = force_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
