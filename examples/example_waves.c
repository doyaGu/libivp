/* example_waves.c -- A wave pool: a liquid surface descriptor and a buoyancy attacher
 *
 * The water is not a plane. A liquid surface descriptor (ivp_liquid_surface_t)
 * answers, for the centre of a body, with the local plane of a travelling
 * wave: its height, its slope (the normal tilts with the wave) and the speed
 * of the water there. The buoyancy controller then lifts and tilts the body
 * with the surface.
 *
 * The controllers are not added one by one: a buoyancy attacher
 * (IVP_Attacher_To_Cores_Buoyancy) keeps one buoyancy controller on every core
 * in an active set. Taking a core out of the set removes its buoyancy, putting
 * it back restores it, and a core that is taken out sinks to the floor.
 *
 *   F5 panel   wave height, wave speed, water density, drag, and one
 *              switch per body (in the set / not in the set)
 *
 * Demonstrates: ivp_liquid_surface_init (a custom eval callback),
 *               ivp_buoyancy_attacher_create, ivp_set_active_add / remove.
 */
#include "demo_app.h"

#include <ivp/containers/ivp_set.h>
#include <ivp/controllers/ivp_controller_buoyancy.h>
#include <ivp/controllers/ivp_liquid_surface.h>

#define N_BODIES    5
#define WATER_Y     (-2.0)             /* mean surface (Y down: up is -Y) */
#define FLOOR_Y     4.5                /* top of the pool floor */
#define POOL_X      13.0
#define POOL_Z      6.0
#define WAVE_LEN    8.0
#define WATER_DENSITY 1.0f             /* masses are 1/1000 of real ones, see example_buoyancy.c */

typedef struct body_desc {
    const char *name;
    bool        ball;
    double      hx, hy, hz, mass, x, y, z;
    float       col[3];
} body_desc_t;

static const body_desc_t descs[N_BODIES] = {
    { "cork",  false, 0.5, 0.5, 0.5, 0.30, -7.0, -4.0, -2.0, { 0.95f, 0.85f, 0.35f } },
    { "plank", false, 1.8, 0.15, 0.6, 0.70, -2.0, -4.0, 1.5,  { 0.75f, 0.55f, 0.25f } },
    { "ball",  true,  0.6, 0.6, 0.6, 0.40, 3.0, -4.0, -1.5, { 0.30f, 0.80f, 1.00f } },
    { "log",   false, 2.2, 0.35, 0.35, 0.60, 7.0, -4.0, 2.0, { 0.60f, 0.40f, 0.25f } },
    { "steel", false, 0.5, 0.5, 0.5, 8.00, 0.0, -5.0, 3.5,  { 0.65f, 0.65f, 0.70f } },
};

typedef struct wave_ctx {
    ivp_environment_t *env;
    double             amp, speed;     /* m, m/s */
} wave_ctx_t;

typedef struct waves_demo {
    ivp_environment_t       *env;
    ivp_object_t            *floor, *wall[2];
    ivp_object_t            *body[N_BODIES];
    wave_ctx_t               wave;
    ivp_liquid_surface_t     surface;
    ivp_buoyancy_state_t     templ;
    ivp_buoyancy_attacher_t *attacher;
    ivp_set_active_t         set;
    bool                     in_set[N_BODIES];
    /* panel */
    float                    amp, speed, density, drag;
    /* measurements */
    float                    y_min[N_BODIES], y_max[N_BODIES];
    float                    max_tilt;
    long                     samples;
} waves_demo_t;

/* surface height (Y) of the wave at x, and its slope along x */
static double wave_y(const wave_ctx_t *w, double x, double t)
{
    const double k = 2.0 * M_PI / WAVE_LEN;
    return WATER_Y - w->amp * sin(k * x - k * w->speed * t);
}

static double wave_slope(const wave_ctx_t *w, double x, double t)
{
    const double k = 2.0 * M_PI / WAVE_LEN;
    return -w->amp * k * cos(k * x - k * w->speed * t);
}

/* IVP_Liquid_Surface_Descriptor::calc_liquid_surface: the local plane at the
 * query point. The normal points out of the liquid (up, tilted by the slope);
 * normal . p = height on the surface. */
static bool wave_eval(void *ctx, const ivp_vec3r_t *pos, ivp_vec3r_t *normal, ivp_real_t *height, ivp_vec3r_t *velocity)
{
    const wave_ctx_t *w = (const wave_ctx_t *)ctx;
    const double t = (double)ivp_environment_get_time(w->env), x = (double)pos->k[0];
    const double ys = wave_y(w, x, t), sl = wave_slope(w, x, t);
    const double len = sqrt(sl * sl + 1.0);
    const ivp_vec3r_t n = ivp_vec3r((ivp_real_t)(sl / len), (ivp_real_t)(-1.0 / len), IVP_REAL(0.0));
    *normal = n;
    *height = (ivp_real_t)(n.k[0] * x + n.k[1] * ys + n.k[2] * (double)pos->k[2]);
    /* the water at the surface moves with the wave: a horizontal swing in phase with the height */
    const double k = 2.0 * M_PI / WAVE_LEN;
    *velocity = ivp_vec3r((ivp_real_t)(w->amp * k * w->speed * sin(k * x - k * w->speed * t)), IVP_REAL(0.0), IVP_REAL(0.0));
    return true;
}

/* ── panel ───────────────────────────────────────────────────────────────── */

static void wake_all(waves_demo_t *s)
{
    for (int i = 0; i < N_BODIES; i++) ivp_environment_ensure_core_in_simulation(s->env, s->body[i]);
}

static void apply_wave(void *ctx)
{
    waves_demo_t *s = (waves_demo_t *)ctx;
    s->wave.amp = (double)s->amp;
    s->wave.speed = (double)s->speed;
    wake_all(s);
}

static void apply_water(void *ctx)
{
    waves_demo_t *s = (waves_demo_t *)ctx;
    ivp_buoyancy_state_t *t = ivp_buoyancy_attacher_get_template(s->attacher);
    t->fluid_density = (ivp_real_t)s->density;
    t->linear_drag = (ivp_real_t)s->drag;
    t->angular_drag = (ivp_real_t)s->drag;
    wake_all(s);
}

static void apply_set(waves_demo_t *s, int i)
{
    ivp_core_t *core = s->body[i]->physical_core;
    if (s->in_set[i]) ivp_set_active_add(&s->set, core);
    else ivp_set_active_remove(&s->set, core);
    wake_all(s);
}

static void apply_cork(void *ctx) { apply_set((waves_demo_t *)ctx, 0); }
static void apply_plank(void *ctx) { apply_set((waves_demo_t *)ctx, 1); }
static void apply_ball(void *ctx) { apply_set((waves_demo_t *)ctx, 2); }

static void waves_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    waves_demo_t *s = (waves_demo_t *)app->user;
    demo_panel_t *p = &v->panel;
    demo_panel_add_float(p, "wave height", &s->amp, 0.0f, 1.2f, apply_wave, s);
    demo_panel_add_float(p, "wave speed", &s->speed, 0.0f, 5.0f, apply_wave, s);
    demo_panel_add_float(p, "density", &s->density, 0.3f, 2.5f, apply_water, s);
    demo_panel_add_float(p, "drag", &s->drag, 0.0f, 6.0f, apply_water, s);
    demo_panel_add_bool(p, "cork in set", &s->in_set[0], apply_cork, s);
    demo_panel_add_bool(p, "plank in set", &s->in_set[1], apply_plank, s);
    demo_panel_add_bool(p, "ball in set", &s->in_set[2], apply_ball, s);
}

/* ── per frame ───────────────────────────────────────────────────────────── */

/* pitch of the body's x axis in the xy plane (the wave runs along x), degrees */
static float tilt_of(const ivp_object_t *obj)
{
    ivp_mat3r_t m;
    ivp_quat_to_mat3(&obj->physical_core->q_world_f_core_last_psi, &m);
    return (float)fabs(atan2(m.rows[1].k[0], m.rows[0].k[0]) * 57.29578);
}

static void waves_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    waves_demo_t *s = (waves_demo_t *)app->user;
    const double t = (double)ivp_environment_get_time(s->env);

    /* the headless run takes the cork out of the set after 6 s: it must sink */
    if (!v && frame == 360) {
        s->in_set[0] = false;
        apply_set(s, 0);
    }
    if (t > 3.0 && t < 6.0) {
        for (int i = 0; i < N_BODIES; i++) {
            float c[3];
            demo_object_center(s->body[i], c);
            if (c[1] < s->y_min[i]) s->y_min[i] = c[1];
            if (c[1] > s->y_max[i]) s->y_max[i] = c[1];
        }
        const float tl = tilt_of(s->body[3]);
        if (tl > s->max_tilt) s->max_tilt = tl;
        s->samples++;
    }
}

static void waves_reset(demo_app_t *app)
{
    waves_demo_t *s = (waves_demo_t *)app->user;
    for (int i = 0; i < N_BODIES; i++) {
        s->y_min[i] = 1e9f;
        s->y_max[i] = -1e9f;
        if (!s->in_set[i]) { s->in_set[i] = true; apply_set(s, i); }
    }
    s->max_tilt = 0.0f;
    s->samples = 0;
}

/* ── drawing ─────────────────────────────────────────────────────────────── */

static void draw_water(demo_viewer_t *v, const waves_demo_t *s)
{
    const double t = (double)ivp_environment_get_time(s->env);
    const float c1[3] = { 0.30f, 0.55f, 0.90f }, c2[3] = { 0.25f, 0.45f, 0.80f };
    for (int zi = -2; zi <= 2; zi++) {
        const float z = (float)(zi * POOL_Z * 0.45);
        float prev[3] = { 0 };
        for (int xi = 0; xi <= 52; xi++) {
            const double x = -POOL_X + 0.5 * xi;
            const float p[3] = { (float)x, (float)wave_y(&s->wave, x, t), z };
            if (xi) demo_line(v, prev, p, c1);
            memcpy(prev, p, sizeof(prev));
        }
    }
    for (int xi = -6; xi <= 6; xi++) {
        const double x = xi * 2.0;
        float prev[3] = { 0 };
        for (int zi = 0; zi <= 12; zi++) {
            const float z = (float)(-POOL_Z + zi);
            const float p[3] = { (float)x, (float)wave_y(&s->wave, x, t), z };
            if (zi) demo_line(v, prev, p, c2);
            memcpy(prev, p, sizeof(prev));
        }
    }
}

static void waves_draw(demo_app_t *app, demo_viewer_t *v)
{
    waves_demo_t *s = (waves_demo_t *)app->user;
    const float c_floor[3] = { 0.45f, 0.48f, 0.52f }, c_wall[3] = { 0.38f, 0.41f, 0.46f };
    demo_draw_object(v, s->floor, c_floor);
    demo_draw_object(v, s->wall[0], c_wall);
    demo_draw_object(v, s->wall[1], c_wall);
    for (int i = 0; i < N_BODIES; i++) demo_draw_object(v, s->body[i], descs[i].col);
    draw_water(v, s);

    const double t = (double)ivp_environment_get_time(s->env);
    for (int i = 0; i < N_BODIES; i++) {
        float c[3];
        demo_object_center(s->body[i], c);
        const double surf = wave_y(&s->wave, (double)c[0], t);
        demo_hud(v, s->in_set[i] ? DEMO_COL_INFO : DEMO_COL_WARNING, "%-6s %s  %+.2f m %s the surface", descs[i].name,
                 s->in_set[i] ? "in set    " : "not in set", fabs((double)c[1] - surf), (double)c[1] < surf ? "above" : "below");
    }
}

static void waves_stats(demo_app_t *app, char *buf, size_t size)
{
    waves_demo_t *s = (waves_demo_t *)app->user;
    snprintf(buf, size, "set size %d, log pitch max %.1f deg", ivp_attacher_to_cores_count(&s->attacher->attacher),
             (double)s->max_tilt);
}

static bool waves_check(demo_app_t *app)
{
    waves_demo_t *s = (waves_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    const double t = (double)ivp_environment_get_time(s->env);
    ok &= demo_check(tag, s->samples > 100, "%ld samples between 3 s and 6 s", s->samples);
    /* the plank and the ball ride the wave: their height swings with it */
    for (int i = 1; i <= 2; i++)
        ok &= demo_check(tag, s->y_max[i] - s->y_min[i] > 0.5f * (float)s->wave.amp,
                         "the %s rises and falls with the wave (swing %.2f m, wave height %.2f m)", descs[i].name,
                         (double)(s->y_max[i] - s->y_min[i]), 2.0 * s->wave.amp);
    ok &= demo_check(tag, s->max_tilt > 3.0f, "the surface slope pitches the log (up to %.1f deg)", (double)s->max_tilt);
    float steel[3];
    demo_object_center(s->body[4], steel);
    ok &= demo_check(tag, steel[1] > 3.0f, "the steel block lies on the floor (y %.2f)", (double)steel[1]);
    float cork[3];
    demo_object_center(s->body[0], cork);
    ok &= demo_check(tag, !s->in_set[0] && cork[1] > WATER_Y + 1.5f && t > 6.0,
                     "the cork sank after it left the set (y %.2f, surface %.1f)", (double)cork[1], WATER_Y);
    float plank[3];
    demo_object_center(s->body[1], plank);
    ok &= demo_check(tag, plank[1] < WATER_Y + 0.8 && plank[1] > WATER_Y - 1.5, "the plank floats at the surface (y %.2f)",
                     (double)plank[1]);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Waves -- Liquid Surface and Buoyancy Attacher";
    app.cfg.tag         = "waves";
    app.cfg.friction    = 0.5;
    app.cfg.elasticity  = 0.2;
    app.cfg.orbit_dist  = 28.0f;
    app.cfg.orbit_yaw   = 160.0f;
    app.cfg.orbit_pitch = -18.0f;
    app.cfg.target[1]   = 0.0f;
    app.cfg.grid_extent = -1.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    waves_demo_t *s = (waves_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->env = w->env;
    s->amp = 0.5f;
    s->speed = 2.0f;
    s->density = WATER_DENSITY;
    s->drag = 3.0f;
    for (int i = 0; i < N_BODIES; i++) { s->y_min[i] = 1e9f; s->y_max[i] = -1e9f; }

    s->floor = demo_create_box(w, NULL, POOL_X + 0.5, 0.5, POOL_Z + 0.5, 0.0, NULL, 0.0, FLOOR_Y + 0.5, 0.0);
    s->wall[0] = demo_create_box(w, NULL, 0.3, 4.0, POOL_Z + 0.5, 0.0, NULL, -POOL_X - 0.3, 0.5, 0.0);
    s->wall[1] = demo_create_box(w, NULL, 0.3, 4.0, POOL_Z + 0.5, 0.0, NULL, POOL_X + 0.3, 0.5, 0.0);

    ivp_set_active_init(&s->set, NULL, 16);
    for (int i = 0; i < N_BODIES; i++) {
        const body_desc_t *d = &descs[i];
        const ivp_material_t m = demo_material(0.5, 0.2);
        if (d->ball) s->body[i] = demo_create_ball(w, &m, d->hx, d->mass, NULL, d->x, d->y, d->z);
        else s->body[i] = demo_create_box(w, &m, d->hx, d->hy, d->hz, d->mass, NULL, d->x, d->y, d->z);
        if (!s->body[i]) return 1;
        s->in_set[i] = true;
        ivp_set_active_add(&s->set, s->body[i]->physical_core);
    }

    s->wave.env = w->env;
    s->wave.amp = (double)s->amp;
    s->wave.speed = (double)s->speed;
    ivp_liquid_surface_init(&s->surface, wave_eval, &s->wave);
    ivp_buoyancy_init(&s->templ, (ivp_real_t)WATER_Y, (ivp_real_t)WATER_DENSITY);
    s->templ.linear_drag = (ivp_real_t)s->drag;
    s->templ.angular_drag = (ivp_real_t)s->drag;
    ivp_buoyancy_set_liquid_surface(&s->templ, &s->surface);
    s->attacher = ivp_buoyancy_attacher_create(w->env, &s->templ, &s->set);
    if (!s->attacher) return 1;

    app.controls    = "F5 panel: wave, water, and which bodies are in the buoyancy set";
    app.viewer_init = waves_viewer_init;
    app.update      = waves_update;
    app.on_reset    = waves_reset;
    app.draw        = waves_draw;
    app.stats       = waves_stats;
    app.check       = waves_check;

    int rc = demo_run(&app, argc, argv);
    ivp_buoyancy_attacher_destroy(s->attacher);
    ivp_set_active_destroy(&s->set);
    ivp_buoyancy_state_destroy(&s->templ);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
