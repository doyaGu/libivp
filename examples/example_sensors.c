/* example_sensors.c -- Proximity queries and the collision ranges
 *
 * A sentry stands in an arena full of rolling balls and boxes.
 *
 *   radar   ivp_environment_do_radar from the sentry reports every object within
 *           a range (a callback gets the object, the distance and the direction);
 *           the objects it finds turn yellow and a line joins them to the sentry
 *   probe   a sphere sweeps through the arena on a circle; the sphere solver
 *           (ivp_sphere_solver_check_environment) lists the objects that
 *           intrude it, and they turn cyan
 *   ranges  the range manager decides how far ahead the collision detection
 *           looks: the radius of the OV hull around an object grows with its
 *           speed (the magenta sphere around the fast ball) and with the
 *           look-ahead time, which the panel changes
 *   tolerance  the global collision tolerance sets the gap a resting body keeps
 *           above the ground (the HUD shows the gap of the crate and the
 *           thresholds ivp_collision_settings_init derives from the tolerance)
 *
 *   F5 panel   radar range, probe radius, look-ahead time, collision tolerance
 *
 * Demonstrates: ivp_environment_do_radar, ivp_sphere_solver_check_environment,
 *               ivp_range_manager (get_coll_range_in_world, parameters),
 *               ivp_environment_set_global_collision_tolerance,
 *               ivp_collision_settings_init.
 */
#include "demo_app.h"

#include <ivp/collision/ivp_collision_tolerance.h>
#include <ivp/collision/ivp_radar.h>
#include <ivp/collision/ivp_range_manager.h>
#include <ivp/collision/ivp_sphere_solver.h>

#define N_BODIES  12
#define ARENA     11.0
#define MAX_HITS  32

typedef struct sensors_demo {
    ivp_environment_t *env;
    ivp_object_t      *ground, *wall[4], *sentry;
    ivp_object_t      *body[N_BODIES];
    ivp_object_t      *crate, *bullet;
    /* radar */
    ivp_object_t      *radar_hit[MAX_HITS];
    float              radar_dist[MAX_HITS];
    int                n_radar;
    /* probe */
    ivp_object_t      *probe_hit[MAX_HITS];
    int                n_probe;
    float              probe_pos[3];
    /* panel */
    float              radar_range, probe_radius, look_ahead, tolerance;
    /* measurements */
    int                radar_max, probe_max;
    float              gap_before, gap_after;
    int                radar_wide, radar_narrow;            /* hits at the full range and at 2.5 m */
    float              range_min, range_max;                /* hull range of the bullet over the run */
} sensors_demo_t;

/* ── radar and probe ─────────────────────────────────────────────────────── */

static void radar_hit(void *ctx, ivp_object_t *target, ivp_real_t distance, const ivp_vec3r_t *direction)
{
    sensors_demo_t *s = (sensors_demo_t *)ctx;
    (void)direction;
    if (target == s->ground || target == s->sentry) return;
    for (int i = 0; i < 4; i++)
        if (target == s->wall[i]) return;
    if (s->n_radar < MAX_HITS) {
        s->radar_hit[s->n_radar] = target;
        s->radar_dist[s->n_radar++] = (float)distance;
    }
}

static void scan(sensors_demo_t *s, double t)
{
    s->n_radar = 0;
    ivp_radar_config_t cfg;
    ivp_radar_config_init(&cfg);
    cfg.max_range = (ivp_real_t)s->radar_range;
    cfg.skip_unmoveable = true;
    cfg.hit_fn = radar_hit;
    cfg.ctx = s;
    ivp_environment_do_radar(s->env, s->sentry, &cfg);
    if (s->n_radar > s->radar_max) s->radar_max = s->n_radar;

    /* the probe sweeps a circle */
    const double a = 0.45 * t;
    s->probe_pos[0] = (float)(7.0 * cos(a));
    s->probe_pos[1] = -1.2f;
    s->probe_pos[2] = (float)(7.0 * sin(a));
    ivp_sphere_solver_template_t tmpl;
    tmpl.center = ivp_vec3r((ivp_real_t)s->probe_pos[0], (ivp_real_t)s->probe_pos[1], (ivp_real_t)s->probe_pos[2]);
    tmpl.radius = (ivp_real_t)s->probe_radius;
    tmpl.max_traversal_depth = 8;
    s->n_probe = ivp_sphere_solver_check_environment(&tmpl, s->env, s->probe_hit, MAX_HITS);
    if (s->n_probe < 0) s->n_probe = 0;
    if (s->n_probe > s->probe_max) s->probe_max = s->n_probe;
}

static bool in_list(ivp_object_t *const *list, int n, const ivp_object_t *o)
{
    for (int i = 0; i < n; i++)
        if (list[i] == o) return true;
    return false;
}

/* ── ranges and tolerance ────────────────────────────────────────────────── */

static float hull_range(const sensors_demo_t *s, const ivp_object_t *o)
{
    return (float)ivp_range_manager_coll_range_in_world(ivp_environment_get_range_manager(s->env),
                                                        (double)ivp_environment_get_delta_psi_time(s->env), o);
}

static float crate_gap(const sensors_demo_t *s)
{
    float c[3];
    demo_object_center(s->crate, c);
    return -c[1] - 0.4f;                       /* ground top is y = 0, half height 0.4 */
}

static void apply_look_ahead(void *ctx)
{
    sensors_demo_t *s = (sensors_demo_t *)ctx;
    ivp_range_manager_t *rm = ivp_environment_get_range_manager(s->env);
    rm->look_ahead_time_world = s->look_ahead;
}

static void apply_tolerance(void *ctx)
{
    sensors_demo_t *s = (sensors_demo_t *)ctx;
    ivp_environment_set_global_collision_tolerance(s->env, (ivp_real_t)s->tolerance);
    for (int i = 0; i < N_BODIES; i++) ivp_environment_ensure_core_in_simulation(s->env, s->body[i]);
    ivp_environment_ensure_core_in_simulation(s->env, s->crate);
}

static void sensors_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    sensors_demo_t *s = (sensors_demo_t *)app->user;
    demo_panel_t *p = &v->panel;
    demo_panel_add_float(p, "radar range", &s->radar_range, 2.0f, 20.0f, NULL, s);
    demo_panel_add_float(p, "probe radius", &s->probe_radius, 0.5f, 5.0f, NULL, s);
    demo_panel_add_float(p, "look-ahead", &s->look_ahead, 0.1f, 3.0f, apply_look_ahead, s);
    demo_panel_add_float(p, "tolerance", &s->tolerance, 0.005f, 0.05f, apply_tolerance, s);
}

/* ── per frame ───────────────────────────────────────────────────────────── */

static void sensors_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    sensors_demo_t *s = (sensors_demo_t *)app->user;
    (void)v;
    const double t = (double)ivp_environment_get_time(s->env);
    scan(s, t);

    if (frame > 2) {
        const float r = hull_range(s, s->bullet);
        if (r < s->range_min) s->range_min = r;
        if (r > s->range_max) s->range_max = r;
    }
    /* the headless run: the gap of the resting crate at two tolerances */
    if (!v) {
        if (frame == 240) {
            s->gap_before = crate_gap(s);
            s->tolerance = 0.04f;
            apply_tolerance(s);
        }
        if (frame == 480) s->gap_after = crate_gap(s);
        if (frame == 300) {
            s->radar_wide = s->n_radar;
            s->radar_range = 2.5f;
        }
        if (frame == 310) s->radar_narrow = s->n_radar;
    }
}

static void sensors_reset(demo_app_t *app)
{
    sensors_demo_t *s = (sensors_demo_t *)app->user;
    s->radar_max = s->probe_max = 0;
    s->range_min = 1e9f;
    s->range_max = 0.0f;
}

/* ── drawing ─────────────────────────────────────────────────────────────── */

static void ring(demo_viewer_t *v, const float c[3], float r, const float col[3])
{
    float prev[3] = { c[0] + r, c[1], c[2] };
    for (int i = 1; i <= 48; i++) {
        const float a = 6.2831853f * (float)i / 48.0f;
        const float p[3] = { c[0] + r * cosf(a), c[1], c[2] + r * sinf(a) };
        demo_line(v, prev, p, col);
        memcpy(prev, p, sizeof(prev));
    }
}

static void sensors_draw(demo_app_t *app, demo_viewer_t *v)
{
    sensors_demo_t *s = (sensors_demo_t *)app->user;
    const float c_ground[3] = { 0.50f, 0.53f, 0.57f }, c_wall[3] = { 0.40f, 0.43f, 0.48f }, c_sentry[3] = { 0.90f, 0.35f, 0.35f },
                c_plain[3] = { 0.85f, 0.85f, 0.88f }, c_radar[3] = { 0.98f, 0.88f, 0.30f }, c_probe[3] = { 0.30f, 0.90f, 0.95f },
                c_range[3] = { 1.0f, 0.4f, 1.0f };
    demo_draw_object(v, s->ground, c_ground);
    for (int i = 0; i < 4; i++) demo_draw_object(v, s->wall[i], c_wall);
    demo_draw_object(v, s->sentry, c_sentry);
    demo_draw_object(v, s->crate, c_plain);
    demo_draw_object(v, s->bullet, c_range);
    for (int i = 0; i < N_BODIES; i++) {
        const ivp_object_t *o = s->body[i];
        const bool by_radar = in_list(s->radar_hit, s->n_radar, o), by_probe = in_list(s->probe_hit, s->n_probe, o);
        demo_draw_object(v, o, by_radar ? c_radar : (by_probe ? c_probe : c_plain));
    }
    float sp[3];
    demo_object_center(s->sentry, sp);
    sp[1] = -0.05f;
    ring(v, sp, s->radar_range, c_radar);
    for (int i = 0; i < s->n_radar; i++) {
        float p[3];
        demo_object_center(s->radar_hit[i], p);
        float top[3] = { sp[0], -2.4f, sp[2] };
        demo_line(v, top, p, c_radar);
    }
    demo_sphere(v, s->probe_pos, s->probe_radius, c_probe);
    /* the hull the collision detection looks ahead with around the bullet */
    float bp[3];
    demo_object_center(s->bullet, bp);
    demo_sphere(v, bp, hull_range(s, s->bullet), c_range);

    ivp_collision_settings_t cs;
    ivp_collision_settings_init(&cs, (ivp_real_t)s->tolerance);
    demo_hud(v, c_radar, "radar: %d objects within %.1f m", s->n_radar, (double)s->radar_range);
    demo_hud(v, c_probe, "probe: %d objects in the sphere of %.1f m", s->n_probe, (double)s->probe_radius);
    demo_hud(v, c_range, "bullet: %.1f m/s, hull range %.2f m (look-ahead %.1f s)", (double)demo_object_speed(s->bullet),
             (double)hull_range(s, s->bullet), (double)s->look_ahead);
    demo_hud(v, DEMO_COL_INFO, "tolerance %.3f: crate gap %.4f m, friction dist %.3f, keeper dist %.3f", (double)s->tolerance,
             (double)crate_gap(s), (double)cs.friction_dist, (double)cs.keeper_dist);
}

static void sensors_stats(demo_app_t *app, char *buf, size_t size)
{
    sensors_demo_t *s = (sensors_demo_t *)app->user;
    snprintf(buf, size, "radar %d, probe %d, bullet range %.2f, gap %.4f", s->n_radar, s->n_probe,
             (double)hull_range(s, s->bullet), (double)crate_gap(s));
}

static bool sensors_check(demo_app_t *app)
{
    sensors_demo_t *s = (sensors_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, s->radar_max >= 2, "the radar found up to %d objects", s->radar_max);
    ok &= demo_check(tag, s->radar_narrow < s->radar_wide, "a shorter range finds fewer (%d at 2.5 m, %d at the full range)",
                     s->radar_narrow, s->radar_wide);
    ok &= demo_check(tag, s->probe_max >= 1, "the probe found up to %d objects", s->probe_max);
    /* every radar hit is really within range of the sentry */
    bool near = true;
    for (int i = 0; i < s->n_radar; i++) near &= s->radar_dist[i] <= s->radar_range + 1.0f;
    ok &= demo_check(tag, near, "all radar hits are within the range (%.1f m)", (double)s->radar_range);
    ok &= demo_check(tag, s->range_max > s->range_min + 0.5f,
                     "the hull range of the bullet follows its speed (%.2f m .. %.2f m)", (double)s->range_min,
                     (double)s->range_max);
    ok &= demo_check(tag, s->gap_before > 0.0f && s->gap_before < 0.1f, "the resting crate keeps a gap of %.4f m", (double)s->gap_before);
    ok &= demo_check(tag, fabsf(s->gap_after - s->gap_before) > 0.002f,
                     "the gap follows the tolerance (%.4f m at 0.01, %.4f m at 0.04)", (double)s->gap_before,
                     (double)s->gap_after);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Sensors -- Radar, Sphere Probe, Ranges";
    app.cfg.tag         = "sensors";
    app.cfg.friction    = 0.2;
    app.cfg.elasticity  = 0.8;
    app.cfg.orbit_dist  = 26.0f;
    app.cfg.orbit_yaw   = 30.0f;
    app.cfg.orbit_pitch = -45.0f;
    app.cfg.target[1]   = 0.0f;
    app.cfg.grid_extent = 12.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    sensors_demo_t *s = (sensors_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->env = w->env;
    s->radar_range = 7.0f;
    s->probe_radius = 2.0f;
    s->look_ahead = ivp_environment_get_range_manager(w->env)->look_ahead_time_world;
    s->tolerance = 0.01f;
    s->range_min = 1e9f;

    s->ground = demo_create_box(w, NULL, ARENA + 1.0, 0.5, ARENA + 1.0, 0.0, NULL, 0.0, 0.5, 0.0);
    s->wall[0] = demo_create_box(w, NULL, 0.5, 1.0, ARENA + 1.0, 0.0, NULL, ARENA + 0.5, -1.0, 0.0);
    s->wall[1] = demo_create_box(w, NULL, 0.5, 1.0, ARENA + 1.0, 0.0, NULL, -ARENA - 0.5, -1.0, 0.0);
    s->wall[2] = demo_create_box(w, NULL, ARENA + 1.0, 1.0, 0.5, 0.0, NULL, 0.0, -1.0, ARENA + 0.5);
    s->wall[3] = demo_create_box(w, NULL, ARENA + 1.0, 1.0, 0.5, 0.0, NULL, 0.0, -1.0, -ARENA - 0.5);
    s->sentry = demo_create_box(w, NULL, 0.3, 1.2, 0.3, 0.0, NULL, 0.0, -1.2, 0.0);

    /* rolling bodies with a fixed seed, so the headless run is repeatable */
    unsigned seed = 7;
    for (int i = 0; i < N_BODIES; i++) {
        seed = seed * 1103515245u + 12345u;
        const double a = 0.52 * (double)i + (double)(seed % 100) * 0.003, r = 3.0 + 0.55 * (double)i;
        const double x = r * cos(a), z = r * sin(a);
        s->body[i] = (i & 1) ? demo_create_ball(w, NULL, 0.45, 1.0, NULL, x, -0.46, z)
                             : demo_create_box(w, NULL, 0.4, 0.4, 0.4, 1.0, NULL, x, -0.41, z);
        if (!s->body[i]) return 1;
        seed = seed * 1103515245u + 12345u;
        const double v = 2.0 + (double)(seed % 300) * 0.01, d = a + 1.7 + (double)(seed % 7) * 0.2;
        s->body[i]->physical_core->speed = ivp_vec3r((ivp_real_t)(v * cos(d)), IVP_REAL(0.0), (ivp_real_t)(v * sin(d)));
    }
    s->crate = demo_create_box(w, NULL, 0.4, 0.4, 0.4, 2.0, NULL, -9.5, -0.41, 9.5);
    s->bullet = demo_create_ball(w, NULL, 0.25, 0.5, NULL, -ARENA + 1.0, -1.0, ARENA - 4.0);
    if (!s->crate || !s->bullet) return 1;
    s->bullet->physical_core->speed = ivp_vec3r(IVP_REAL(11.0), IVP_REAL(0.0), IVP_REAL(-2.0));

    app.controls    = "F5 panel: radar range, probe radius, look-ahead time, collision tolerance";
    app.viewer_init = sensors_viewer_init;
    app.update      = sensors_update;
    app.on_reset    = sensors_reset;
    app.draw        = sensors_draw;
    app.stats       = sensors_stats;
    app.check       = sensors_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
