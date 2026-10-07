/* example_hover.c -- Hover pads on the floating controller
 *
 * Four pads cross a rolling heightfield at the same speed. Every pad has four
 * floating controllers, one at each bottom corner, that keep the corner at a
 * target distance from the ground: a ray cast straight down (the game-side
 * callback here reads the height function) measures the distance, and the
 * controller pushes or pulls with the impulse that would close the gap in one
 * PSI, clipped to a repulsive and an adhesive limit.
 *
 *   1 hover    target 1.2 m, no limits: it follows the hills
 *   2 weak     the repulsive limit is lower than its weight: it cannot lift
 *              itself and scrapes the ground
 *   3 no pull  adhesive limit 0: it is pushed up but never pulled down, so it
 *              overshoots the crests and only gravity brings it back
 *   4 high     target 2.5 m
 *
 *   F5 panel   hover height of pad 1, speed, repulsive limit of pad 2
 *
 * Demonstrates: ivp_floating_* (target distance, force limits, ray-cast
 *               callback), ivp_environment_create_core_controller,
 *               demo_terrain.
 */
#include "demo_app.h"
#include "demo_terrain.h"

#include <ivp/controllers/ivp_controller_floating.h>

#define N_PADS    4
#define N_GRID    41
#define FIELD     1.5f
#define PAD_HX    0.9
#define PAD_HY    0.12
#define PAD_MASS  5.0
#define START_X   (-24.0)
#define BIG       1e9f

typedef struct pad_desc {
    const char *name;
    float       target, repulsive, adhesive;
    float       col[3];
} pad_desc_t;

static const pad_desc_t descs[N_PADS] = {
    { "hover",   1.2f, BIG, BIG, { 0.40f, 0.78f, 0.95f } },
    { "weak",    1.2f, 8.0f, BIG, { 0.95f, 0.45f, 0.35f } },
    { "no pull", 1.2f, BIG, 0.0f, { 0.95f, 0.80f, 0.35f } },
    { "high",    2.5f, BIG, BIG, { 0.55f, 0.90f, 0.50f } },
};

typedef struct hover_demo {
    ivp_environment_t   *env;
    demo_terrain_t       terrain;
    ivp_object_t        *pad[N_PADS];
    ivp_floating_state_t fl[N_PADS][4];
    float                heights[N_GRID * N_GRID];
    /* panel */
    float                height1, speed, weak_limit;
    /* measurements (t > 3 s) */
    double               err_sum[N_PADS];
    float                dist_max[N_PADS], dist_mean_sum[N_PADS];
    long                 samples;
} hover_demo_t;

static float terrain_height(float x, float z)
{
    return 3.0f + 1.6f * sinf(x * 0.16f) * cosf(z * 0.11f) + 0.7f * sinf(x * 0.45f + z * 0.2f);
}

/* the game-side ray: straight down from the anchor to the height function */
static bool down_ray(void *ctx, const ivp_object_t *obj, const ivp_vec3r_t *start, const ivp_vec3r_t *dir,
                     ivp_real_t *out)
{
    (void)ctx;
    (void)obj;
    (void)dir;
    const float half = demo_terrain_half_extent(&((hover_demo_t *)ctx)->terrain);
    const float x = (float)start->k[0], z = (float)start->k[2];
    if (fabsf(x) > half || fabsf(z) > half) return false;
    *out = (ivp_real_t)(-terrain_height(x, z) - (float)start->k[1]);
    return true;
}

static void apply_limits(hover_demo_t *s)
{
    for (int c = 0; c < 4; c++) {
        ivp_floating_set_target_distance(&s->fl[0][c], (ivp_real_t)s->height1);
        s->fl[1][c].max_repulsive_force = (ivp_real_t)s->weak_limit;
    }
    for (int i = 0; i < N_PADS; i++) ivp_environment_ensure_core_in_simulation(s->env, s->pad[i]);
}

static void apply_panel(void *ctx) { apply_limits((hover_demo_t *)ctx); }

static void hover_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    hover_demo_t *s = (hover_demo_t *)app->user;
    demo_panel_t *p = &v->panel;
    demo_terrain_make_mesh(&s->terrain, v->renderer);
    demo_panel_add_float(p, "hover height", &s->height1, 0.4f, 3.0f, apply_panel, s);
    demo_panel_add_float(p, "speed", &s->speed, 0.0f, 10.0f, apply_panel, s);
    demo_panel_add_float(p, "weak limit", &s->weak_limit, 0.0f, 30.0f, apply_panel, s);
}

static float pad_distance(const hover_demo_t *s, int i)
{
    float c[3];
    demo_object_center(s->pad[i], c);
    return -terrain_height(c[0], c[2]) - c[1];          /* centre to ground, positive above */
}

static void hover_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    hover_demo_t *s = (hover_demo_t *)app->user;
    (void)v;
    (void)frame;
    const double t = (double)ivp_environment_get_time(s->env);

    for (int i = 0; i < N_PADS; i++) {
        /* a driver: hold the forward speed */
        const double vx = s->pad[i]->physical_core->speed.k[0];
        double dv = (double)s->speed - vx;
        if (dv > 0.4) dv = 0.4;
        if (dv < -0.4) dv = -0.4;
        if (t > 0.5) demo_async_add_speed_ws(s->env, s->pad[i], dv, 0.0, 0.0);
    }
    if (t > 3.0) {
        for (int i = 0; i < N_PADS; i++) {
            const float d = pad_distance(s, i);
            const float want = i == 0 ? s->height1 + (float)PAD_HY : descs[i].target + (float)PAD_HY;
            s->err_sum[i] += fabs((double)(d - want));
            s->dist_mean_sum[i] += d;
            if (d > s->dist_max[i]) s->dist_max[i] = d;
        }
        s->samples++;
    }
}

static void hover_reset(demo_app_t *app)
{
    hover_demo_t *s = (hover_demo_t *)app->user;
    memset(s->err_sum, 0, sizeof(s->err_sum));
    memset(s->dist_max, 0, sizeof(s->dist_max));
    memset(s->dist_mean_sum, 0, sizeof(s->dist_mean_sum));
    s->samples = 0;
    apply_limits(s);
}

static void hover_draw(demo_app_t *app, demo_viewer_t *v)
{
    hover_demo_t *s = (hover_demo_t *)app->user;
    const float c_land[3] = { 0.40f, 0.52f, 0.36f }, c_wire[3] = { 0.30f, 0.40f, 0.28f };
    demo_terrain_draw(v, &s->terrain, v->solid ? c_land : c_wire);
    for (int i = 0; i < N_PADS; i++) {
        demo_draw_object(v, s->pad[i], descs[i].col);
        /* the four rays */
        for (int c = 0; c < 4; c++) {
            float a[3];
            demo_core_point_ws(s->pad[i], &s->fl[i][c].position_os, a);          /* the box frame is the core frame */
            const float b[3] = { a[0], -terrain_height(a[0], a[2]), a[2] };
            /* the ray is green on target, red when the corner is too close to the ground, yellow when too far */
            const float want = i == 0 ? s->height1 : descs[i].target, err = (b[1] - a[1]) - want;
            const float k = fminf(1.0f, fabsf(err) / 0.5f);
            const float ray_col[3] = { 0.3f + 0.7f * k, err < 0.0f ? 0.9f - 0.7f * k : 0.9f, 0.3f - 0.1f * k };
            demo_thick_line(v, a, b, ray_col, 0.03f);
            demo_sphere(v, b, 0.08f + 0.12f * k, ray_col);
        }
    }
    for (int i = 0; i < N_PADS; i++) {
        const float d = pad_distance(s, i) - (float)PAD_HY;
        const float want = i == 0 ? s->height1 : descs[i].target;
        demo_hud(v, descs[i].col, "%d %-8s %.2f m above the ground (target %.1f)", i + 1, descs[i].name, (double)d, (double)want);
    }
}

static void hover_stats(demo_app_t *app, char *buf, size_t size)
{
    hover_demo_t *s = (hover_demo_t *)app->user;
    snprintf(buf, size, "distance %.2f %.2f %.2f %.2f", (double)pad_distance(s, 0), (double)pad_distance(s, 1),
             (double)pad_distance(s, 2), (double)pad_distance(s, 3));
}

static bool hover_check(demo_app_t *app)
{
    hover_demo_t *s = (hover_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    const double n = (double)(s->samples > 0 ? s->samples : 1);
    ok &= demo_check(tag, s->samples > 100, "%ld samples", s->samples);
    ok &= demo_check(tag, s->err_sum[0] / n < 0.45, "the hover pad keeps its height (mean error %.2f m)", s->err_sum[0] / n);
    const double mean1 = s->dist_mean_sum[1] / n, mean0 = s->dist_mean_sum[0] / n;
    ok &= demo_check(tag, mean1 < mean0 - 0.4, "the weak pad sags (mean %.2f m against %.2f m)", mean1, mean0);
    ok &= demo_check(tag, s->err_sum[2] / n > 2.0 * s->err_sum[0] / n + 0.02,
                     "without the pull the third pad overshoots the crests (mean error %.2f m against %.2f m)",
                     s->err_sum[2] / n, s->err_sum[0] / n);
    const double mean3 = s->dist_mean_sum[3] / n;
    ok &= demo_check(tag, mean3 > mean0 + 0.8, "the high pad stays higher (mean %.2f m against %.2f m)", mean3, mean0);
    float c[3];
    demo_object_center(s->pad[0], c);
    ok &= demo_check(tag, c[0] > START_X + 10.0, "and it has travelled (x %.1f)", (double)c[0]);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Hover -- Floating Controller";
    app.cfg.tag         = "hover";
    app.cfg.friction    = 0.4;
    app.cfg.elasticity  = 0.1;
    app.cfg.orbit_dist  = 22.0f;
    app.cfg.orbit_yaw   = 150.0f;
    app.cfg.orbit_pitch = -28.0f;
    app.cfg.target[0]   = -4.0f;
    app.cfg.target[1]   = -3.0f;
    app.cfg.grid_extent = -1.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    hover_demo_t *s = (hover_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->env = w->env;
    s->height1 = descs[0].target;
    s->speed = 5.0f;
    s->weak_limit = descs[1].repulsive;

    for (int r = 0; r < N_GRID; r++)
        for (int c = 0; c < N_GRID; c++)
            s->heights[r * N_GRID + c] = terrain_height(((float)r - 0.5f * (N_GRID - 1)) * FIELD,
                                                        ((float)c - 0.5f * (N_GRID - 1)) * FIELD);
    if (!demo_terrain_create(w, &s->terrain, N_GRID, FIELD, s->heights, app.cfg.friction, app.cfg.elasticity)) {
        fprintf(stderr, "[hover] terrain build failed\n");
        return 1;
    }

    for (int i = 0; i < N_PADS; i++) {
        const double z = (i - 1.5) * 4.0, x = START_X;
        const double y = -terrain_height((float)x, (float)z) - (double)descs[i].target - PAD_HY;
        s->pad[i] = demo_create_box(w, NULL, PAD_HX, PAD_HY, PAD_HX, PAD_MASS, NULL, x, y, z);
        if (!s->pad[i]) return 1;
        for (int c = 0; c < 4; c++) {
            ivp_floating_state_t *f = &s->fl[i][c];
            ivp_floating_init_defaults(f);
            f->max_repulsive_force = (ivp_real_t)descs[i].repulsive;
            f->max_adhesive_force = (ivp_real_t)descs[i].adhesive;
            f->target_distance = (ivp_real_t)descs[i].target;
            const ivp_vec3r_t p = ivp_vec3r((ivp_real_t)((c & 1) ? 0.8 : -0.8), (ivp_real_t)PAD_HY,
                                            (ivp_real_t)((c & 2) ? 0.8 : -0.8));
            ivp_floating_set_position_os(f, &p);
            const ivp_vec3r_t down = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
            ivp_floating_set_ray_direction_ws(f, &down);
            ivp_floating_set_object(f, s->pad[i]);
            ivp_floating_set_raycast(f, down_ray, s);
            ivp_controller_handle_t h = ivp_environment_create_core_controller(w->env, IVP_CP_FLOATING,
                                                                               ivp_controller_floating, f);
            ivp_environment_add_controller_to_core(w->env, h, s->pad[i]);
        }
    }

    app.controls    = "F5 panel: hover height of pad 1, speed, repulsive limit of pad 2";
    app.viewer_init = hover_viewer_init;
    app.update      = hover_update;
    app.on_reset    = hover_reset;
    app.draw        = hover_draw;
    app.stats       = hover_stats;
    app.check       = hover_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    demo_terrain_destroy(&s->terrain);
    free(s);
    return rc;
}
