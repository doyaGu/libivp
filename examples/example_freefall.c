/* example_freefall.c -- Balls bouncing under gravity
 *
 * Multiple balls of varying sizes, masses, and elasticities are dropped
 * from different heights onto a ground plane.  The HUD shows per-ball
 * velocity, bounce count (sign changes of vertical speed), and height.
 *
 * Demonstrates: ball surface manager, gravity, elasticity differences,
 *               position/speed readback, HUD text, velocity arrows.
 */
#include "demo_app.h"

#define N_BALLS 5
#define GROUND_TOP (-0.5f)

typedef struct {
    float radius;
    float mass;
    float elasticity;
    float start_y;     /* negative = above ground in Y-down */
    float start_x;
    float col[3];
    const char *label;
} ball_desc_t;

static const ball_desc_t descs[N_BALLS] = {
    {0.6f,  1.0f, 0.90f, -12.0f, -5.0f, {0.2f, 0.8f, 1.0f},  "Rubber"},
    {0.8f,  2.0f, 0.60f, -10.0f, -2.0f, {1.0f, 0.5f, 0.1f},  "Plastic"},
    {1.0f,  5.0f, 0.30f, -15.0f,  1.0f, {0.6f, 0.6f, 0.7f},  "Steel"},
    {0.5f,  0.5f, 0.80f,  -8.0f,  4.0f, {0.3f, 1.0f, 0.3f},  "Tennis"},
    {1.2f, 10.0f, 0.10f, -18.0f,  7.0f, {0.5f, 0.3f, 0.2f},  "Clay"},
};

typedef struct freefall_demo {
    ivp_object_t *ground;
    ivp_object_t *balls[N_BALLS];
    int           bounces[N_BALLS];
    float         prev_vy[N_BALLS];
    float         peak_down_vy[N_BALLS];   /* impact speed of the first fall */
    float         rebound_h[N_BALLS];      /* highest point after the first bounce */
} freefall_demo_t;

static float ball_height(const ivp_object_t *b, float radius)
{
    /* height of the lowest point above the ground (Y down) */
    return GROUND_TOP - (float)b->physical_core->pos_world_f_core_last_psi.k[1] - radius;
}

static void start_speed(freefall_demo_t *s)
{
    /* Small initial downward velocity for a natural drop */
    for (int i = 0; i < N_BALLS; i++)
        s->balls[i]->physical_core->speed = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
}

static void freefall_reset(demo_app_t *app)
{
    freefall_demo_t *s = (freefall_demo_t *)app->user;
    for (int i = 0; i < N_BALLS; i++) {
        s->bounces[i] = 0;
        s->prev_vy[i] = 0.0f;
        s->peak_down_vy[i] = 0.0f;
        s->rebound_h[i] = 0.0f;
        demo_async_add_speed_ws(app->world.env, s->balls[i], 0.0, 1.0, 0.0);
    }
}

static void freefall_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    freefall_demo_t *s = (freefall_demo_t *)app->user;
    (void)v;
    (void)frame;
    for (int i = 0; i < N_BALLS; i++) {
        float vy = (float)s->balls[i]->physical_core->speed.k[1];
        if (s->bounces[i] == 0 && vy > s->peak_down_vy[i]) s->peak_down_vy[i] = vy;
        /* bounce: was moving down, now moving up (low thresholds catch the
         * low-energy bounces near settling) */
        if (s->prev_vy[i] > 0.05f && vy < -0.05f) s->bounces[i]++;
        s->prev_vy[i] = vy;
        if (s->bounces[i] == 1) {
            float h = ball_height(s->balls[i], descs[i].radius);
            if (h > s->rebound_h[i]) s->rebound_h[i] = h;
        }
    }
}

static void freefall_draw(demo_app_t *app, demo_viewer_t *v)
{
    freefall_demo_t *s = (freefall_demo_t *)app->user;
    const float c_ground[] = {0.35f, 0.35f, 0.40f};
    const float c_vel[]    = {0.20f, 1.0f,  0.40f};
    demo_draw_object(v, s->ground, c_ground);
    {
        float org[3] = {0, 0, 0};
        demo_axes(v, org, NULL, 2.0f);
    }
    for (int i = 0; i < N_BALLS; i++) {
        demo_draw_object(v, s->balls[i], descs[i].col);
        if (demo_object_speed(s->balls[i]) > 0.5f) demo_draw_velocity(v, s->balls[i], c_vel);
    }
    demo_hud(v, DEMO_COL_INFO, "Balls: %d  Time: %.1fs", N_BALLS,
             (double)ivp_environment_get_time(app->world.env));
    for (int i = 0; i < N_BALLS; i++) {
        demo_hud(v, descs[i].col, "%s (e=%.2f m=%.0f): h=%.1f v=%.1f bounces=%d",
                 descs[i].label, (double)descs[i].elasticity, (double)descs[i].mass,
                 (double)ball_height(s->balls[i], descs[i].radius),
                 (double)demo_object_speed(s->balls[i]), s->bounces[i]);
    }
}

static void freefall_stats(demo_app_t *app, char *buf, size_t size)
{
    freefall_demo_t *s = (freefall_demo_t *)app->user;
    int n = 0;
    for (int i = 0; i < N_BALLS && n >= 0 && (size_t)n < size; i++) {
        n += snprintf(buf + n, size - (size_t)n, "%s h=%.2f v=%.2f b=%d  ", descs[i].label,
                      (double)ball_height(s->balls[i], descs[i].radius),
                      (double)demo_object_speed(s->balls[i]), s->bounces[i]);
    }
}

static float drop_height(const ball_desc_t *d)
{
    return GROUND_TOP - d->start_y - d->radius;
}

static bool freefall_check(demo_app_t *app)
{
    freefall_demo_t *s = (freefall_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    for (int i = 0; i < N_BALLS; i++) {
        const ball_desc_t *d = &descs[i];
        /* free fall with the initial 1 m/s: v^2 = v0^2 + 2 g h */
        float v_expect = sqrtf(1.0f + 2.0f * 9.83f * drop_height(d));
        ok &= demo_check(tag, fabsf(s->peak_down_vy[i] - v_expect) < 0.05f * v_expect,
                         "%s hit the ground at %.2f m/s (free fall %.2f m/s)", d->label,
                         (double)s->peak_down_vy[i], (double)v_expect);
        float h = ball_height(s->balls[i], d->radius);
        float speed = demo_object_speed(s->balls[i]);
        ok &= demo_check(tag, s->bounces[i] >= 1 && h > -0.05f && h < 0.1f && speed < 0.1f,
                         "%s (e=%.2f) bounced %d times (first rebound %.2f of %.2f m), rests on the ground (h %.3f, v %.3f)",
                         d->label, (double)d->elasticity, s->bounces[i], (double)s->rebound_h[i],
                         (double)drop_height(d), (double)h, (double)speed);
    }
    /* the rebound fraction grows with the elasticity */
    int order[N_BALLS];
    for (int i = 0; i < N_BALLS; i++) order[i] = i;
    for (int i = 1; i < N_BALLS; i++) {
        for (int j = i; j > 0 && descs[order[j]].elasticity < descs[order[j - 1]].elasticity; j--) {
            int t = order[j]; order[j] = order[j - 1]; order[j - 1] = t;
        }
    }
    bool increasing = true;
    char buf[256];
    int n = 0;
    for (int k = 0; k < N_BALLS; k++) {
        int i = order[k];
        float frac = s->rebound_h[i] / drop_height(&descs[i]);
        if (k > 0) {
            int p = order[k - 1];
            if (!(frac > s->rebound_h[p] / drop_height(&descs[p]))) increasing = false;
        }
        if (n >= 0 && (size_t)n < sizeof(buf))
            n += snprintf(buf + n, sizeof(buf) - (size_t)n, " %s %.3f", descs[i].label, (double)frac);
    }
    ok &= demo_check(tag, increasing, "rebound height / drop height grows with the elasticity:%s", buf);
    ok &= demo_check(tag, s->bounces[0] > s->bounces[4],
                     "rubber (e=0.9) bounces more often than clay (e=0.1): %d vs %d",
                     s->bounces[0], s->bounces[4]);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title        = "IVP Freefall -- Bouncing Balls";
    app.cfg.tag          = "freefall";
    app.cfg.orbit_dist   = 28.0f;
    app.cfg.orbit_yaw    = 15.0f;
    app.cfg.orbit_pitch  = -15.0f;
    app.cfg.target[1]    = -5.0f;
    app.cfg.grid_extent  = 20.0f;
    app.cfg.grid_spacing = 2.0f;

    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ecfg.ccd_enabled = true;
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    freefall_demo_t *s = (freefall_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    s->ground = demo_create_box_templ(&app.world, &gt, 20.0, 0.5, 12.0);

    /* ── Balls ──────────────────────────────────────────────────────────── */
    for (int i = 0; i < N_BALLS; i++) {
        const ball_desc_t *d = &descs[i];
        ivp_object_template_t bt;
        ivp_object_template_init(&bt);
        bt.mass = (ivp_real_t)d->mass;
        set_sphere_inertia(&bt, bt.mass, (ivp_real_t)d->radius);
        bt.material.friction   = IVP_REAL(0.5);
        bt.material.elasticity = (ivp_real_t)d->elasticity;
        bt.position = ivp_vec3r((ivp_real_t)d->start_x, (ivp_real_t)d->start_y, IVP_REAL(0.0));
        s->balls[i] = demo_create_ball_templ(&app.world, &bt, d->radius);
        if (!s->balls[i]) return 1;
    }
    start_speed(s);

    app.update   = freefall_update;
    app.on_reset = freefall_reset;
    app.draw     = freefall_draw;
    app.stats    = freefall_stats;
    app.check    = freefall_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
