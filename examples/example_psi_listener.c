/* example_psi_listener.c -- A flight recorder on the PSI listener
 *
 * Four bodies fly across a ground slab: a rubber ball, a steel ball and a
 * tumbling box, all dropped from a height, and a ball that is thrown along
 * the ground. A PSI listener (IVP_Listener_PSI) is called once at the start
 * of every PSI; it records the pose of each body into a trail and its
 * mechanical energy into a strip chart:
 *
 *   E = 1/2 m v^2 + 1/2 w.I.w + m g h   (h above the resting height)
 *
 * The chart shows E / E0, so the bodies are comparable: the rubber ball
 * loses a little with every bounce, the steel ball and the box lose much
 * more, and the ball on the ground turns its sliding into rolling, then
 * rolls out. The listener never touches the simulation; it only reads.
 *
 *   F5 panel   trails, energy chart
 *
 * Demonstrates: ivp_psi_listener_register / unregister, core readback
 *               (speed, rot_speed, rot_inertia, mass), strip chart HUD.
 */
#include "demo_app.h"

#include <ivp/events/ivp_listener_psi.h>

#define N_BODIES   4
#define HISTORY    600                /* PSIs kept: 10 s */
#define CHART_W    330.0f
#define CHART_H    130.0f

typedef struct tracked {
    const char   *name;
    float         col[3];
    ivp_object_t *obj;
    double        rest_y;             /* centre height when it lies on the ground */
    double        e0;                 /* energy at the first sample */
    float         trail[HISTORY][3];
    float         energy[HISTORY];    /* E / E0 */
} tracked_t;

typedef struct recorder_demo {
    ivp_object_t       *ground;
    tracked_t           body[N_BODIES];
    ivp_psi_listener_t  listener;
    ivp_environment_t  *env;
    int                 head;         /* next slot */
    int                 count;        /* samples kept, <= HISTORY */
    long                calls;        /* listener calls */
    double              last_time;
    bool                time_ok;      /* the listener times only ever grow */
    bool                show_trails, show_chart;
} recorder_demo_t;

static double body_energy(const tracked_t *t)
{
    const ivp_core_t *c = t->obj->physical_core;
    double v2 = 0.0, rot = 0.0;
    for (int k = 0; k < 3; k++) {
        v2  += (double)c->speed.k[k] * (double)c->speed.k[k];
        rot += (double)c->rot_inertia.k[k] * (double)c->rot_speed.k[k] * (double)c->rot_speed.k[k];
    }
    const double h = t->rest_y - (double)c->pos_world_f_core_last_psi.k[1];      /* Y is down */
    return 0.5 * (double)c->mass * v2 + 0.5 * rot + (double)c->mass * DEMO_GRAVITY * h;
}

/* IVP_Listener_PSI::event_PSI: the start of a PSI. */
static void record_psi(void *ctx, ivp_environment_t *env, ivp_time_t time, ivp_real_t dt)
{
    recorder_demo_t *s = (recorder_demo_t *)ctx;
    (void)env;
    (void)dt;
    s->calls++;
    if ((double)time < s->last_time) s->time_ok = false;
    s->last_time = (double)time;

    const int slot = s->head;
    for (int i = 0; i < N_BODIES; i++) {
        tracked_t *t = &s->body[i];
        const ivp_core_t *c = t->obj->physical_core;
        for (int k = 0; k < 3; k++) t->trail[slot][k] = (float)c->pos_world_f_core_last_psi.k[k];
        const double e = body_energy(t);
        if (s->count == 0) t->e0 = e;
        t->energy[slot] = t->e0 > 1e-9 ? (float)(e / t->e0) : 0.0f;
    }
    s->head = (s->head + 1) % HISTORY;
    if (s->count < HISTORY) s->count++;
}

/* The i-th oldest sample. */
static int sample_slot(const recorder_demo_t *s, int i)
{
    return (s->head - s->count + i + 2 * HISTORY) % HISTORY;
}

static void recorder_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    recorder_demo_t *s = (recorder_demo_t *)app->user;
    demo_param_bool(v, "trails", &s->show_trails);
    demo_param_bool(v, "energy chart", &s->show_chart);
}

static void recorder_reset(demo_app_t *app)
{
    recorder_demo_t *s = (recorder_demo_t *)app->user;
    s->head = 0;
    s->count = 0;
    s->last_time = 0.0;
}

static void draw_trails(demo_viewer_t *v, const recorder_demo_t *s)
{
    for (int i = 0; i < N_BODIES; i++) {
        const tracked_t *t = &s->body[i];
        for (int n = 1; n < s->count; n++) {
            const float fade = 0.45f + 0.55f * (float)n / (float)s->count;
            const float col[3] = { t->col[0] * fade, t->col[1] * fade, t->col[2] * fade };
            demo_line(v, t->trail[sample_slot(s, n - 1)], t->trail[sample_slot(s, n)], col);
        }
    }
}

static void draw_chart(demo_viewer_t *v, const recorder_demo_t *s)
{
    ivp_renderer_t *r = v->renderer;
    if (!r) return;
    const float x0 = (float)v->win_w - CHART_W - 24.0f, y1 = (float)v->win_h - 190.0f, y0 = y1 - CHART_H;
    const float plate[3] = { 0.04f, 0.05f, 0.07f }, grid[3] = { 0.35f, 0.38f, 0.42f };

    ivp_render_draw_panel(r, x0 - 10.0f, y0 - 30.0f, CHART_W + 20.0f, CHART_H + 66.0f, plate, 0.7f);
    ivp_render_draw_text(r, x0, y0 - 22.0f, "E / E0 over the last 10 s", DEMO_COL_HELP);
    for (int g = 0; g <= 4; g++) {
        ivp_render_draw_rect(r, x0, y0 + CHART_H * (float)g / 4.0f, CHART_W, 1.0f, grid, 0.5f);
    }
    if (s->count < 2) return;
    for (int i = 0; i < N_BODIES; i++) {
        const tracked_t *t = &s->body[i];
        float prev = -1.0f;
        for (int px = 0; px < (int)CHART_W; px++) {
            /* the chart is anchored to the right: the newest sample at the right edge */
            const float age = (CHART_W - 1.0f - (float)px) * (float)HISTORY / CHART_W;
            const int n = s->count - 1 - (int)age;
            if (n < 0) continue;
            float e = t->energy[sample_slot(s, n)];
            if (e > 1.1f) e = 1.1f;
            if (e < 0.0f) e = 0.0f;
            /* join the steps: a column from the previous value to this one */
            const float lo = prev < 0.0f ? e : (prev < e ? prev : e), hi = prev < 0.0f ? e : (prev < e ? e : prev);
            const float top = y1 - CHART_H * hi / 1.1f;
            ivp_render_draw_rect(r, x0 + (float)px, top - 1.0f, 1.0f, CHART_H * (hi - lo) / 1.1f + 2.0f, t->col, 1.0f);
            prev = e;
        }
    }
    /* legend, newest values */
    for (int i = 0; i < N_BODIES; i++) {
        char buf[48];
        const tracked_t *t = &s->body[i];
        snprintf(buf, sizeof(buf), "%s %3.0f%%", t->name, (double)(t->energy[sample_slot(s, s->count - 1)] * 100.0f));
        ivp_render_draw_text(r, x0 + 4.0f + (float)(i % 2) * 150.0f, y1 + 6.0f + (float)(i / 2) * 16.0f, buf, t->col);
    }
}

static void recorder_draw(demo_app_t *app, demo_viewer_t *v)
{
    recorder_demo_t *s = (recorder_demo_t *)app->user;
    const float c_ground[3] = { 0.50f, 0.54f, 0.58f };
    demo_draw_object(v, s->ground, c_ground);
    for (int i = 0; i < N_BODIES; i++) demo_draw_object(v, s->body[i].obj, s->body[i].col);
    if (s->show_trails) draw_trails(v, s);
    if (s->show_chart) draw_chart(v, s);
    demo_hud(v, DEMO_COL_INFO, "PSI listener: %ld calls, %d samples kept", s->calls, s->count);
}

static void recorder_stats(demo_app_t *app, char *buf, size_t size)
{
    recorder_demo_t *s = (recorder_demo_t *)app->user;
    const int last = s->count ? sample_slot(s, s->count - 1) : 0;
    snprintf(buf, size, "calls %ld, E/E0 %.2f %.2f %.2f %.2f", s->calls, (double)s->body[0].energy[last],
             (double)s->body[1].energy[last], (double)s->body[2].energy[last], (double)s->body[3].energy[last]);
}

static bool recorder_check(demo_app_t *app)
{
    recorder_demo_t *s = (recorder_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    const long psis = (long)(ivp_environment_get_time(app->world.env) / DEMO_PSI_TIME + 0.5);
    ok &= demo_check(tag, s->calls > 0 && labs(s->calls - psis) <= 1,
                     "the listener ran once per PSI (%ld calls, %ld PSIs)", s->calls, psis);
    ok &= demo_check(tag, s->time_ok, "the listener times only grow");
    ok &= demo_check(tag, s->count > 60, "%d samples recorded", s->count);
    float peak = 0.0f;
    for (int i = 0; i < N_BODIES; i++)
        for (int n = 0; n < s->count; n++)
            if (s->body[i].energy[sample_slot(s, n)] > peak) peak = s->body[i].energy[sample_slot(s, n)];
    ok &= demo_check(tag, peak < 1.15f, "no body gains energy (peak E/E0 %.2f)", (double)peak);
    float mean[N_BODIES] = { 0 };
    for (int i = 0; i < N_BODIES; i++) {
        for (int n = 0; n < s->count; n++) mean[i] += s->body[i].energy[sample_slot(s, n)];
        mean[i] /= (float)s->count;
    }
    ok &= demo_check(tag, mean[0] > mean[1] + 0.05f,
                     "the rubber ball keeps more energy than the steel ball (mean E/E0 %.2f / %.2f)",
                     (double)mean[0], (double)mean[1]);
    ok &= demo_check(tag, mean[3] > 0.3f && mean[3] < 0.9f,
                     "the rolling ball loses its sliding energy but keeps rolling (mean E/E0 %.2f)", (double)mean[3]);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP PSI Listener -- Flight Recorder";
    app.cfg.tag         = "psi_listener";
    app.cfg.friction    = 0.6;
    app.cfg.elasticity  = 0.5;
    app.cfg.orbit_dist  = 36.0f;
    app.cfg.orbit_yaw   = 15.0f;
    app.cfg.orbit_pitch = -24.0f;
    app.cfg.target[0]   = 9.0f;
    app.cfg.target[1]   = -3.0f;
    app.cfg.grid_extent = 26.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    recorder_demo_t *s = (recorder_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->env = w->env;
    s->time_ok = true;
    s->show_trails = true;
    s->show_chart = true;

    s->ground = demo_create_box(w, NULL, 26.0, 0.5, 6.0, 0.0, NULL, 10.0, 0.5, 0.0);

    const ivp_material_t m_rubber = demo_material(0.6, 0.95), m_steel = demo_material(0.4, 0.35),
                         m_wood = demo_material(0.5, 0.3), m_roll = demo_material(0.7, 0.2);
    ivp_quatr_t tilt = ivp_quatr_make(IVP_REAL(0.25), IVP_REAL(0.1), IVP_REAL(0.15), IVP_REAL(0.95));
    ivp_quat_normalize(&tilt);
    tracked_t *b = s->body;
    b[0] = (tracked_t){ "rubber", { 0.95f, 0.45f, 0.35f }, NULL, -0.5 };
    b[1] = (tracked_t){ "steel",      { 0.60f, 0.80f, 0.95f }, NULL, -0.5 };
    b[2] = (tracked_t){ "box",         { 0.95f, 0.80f, 0.35f }, NULL, -0.5 };
    b[3] = (tracked_t){ "roller",     { 0.55f, 0.90f, 0.50f }, NULL, -0.5 };
    b[0].obj = demo_create_ball(w, &m_rubber, 0.5, 1.0, NULL, -14.0, -7.0, -3.6);
    b[1].obj = demo_create_ball(w, &m_steel,  0.5, 4.0, NULL, -14.0, -7.0, -1.2);
    b[2].obj = demo_create_box(w, &m_wood, 0.5, 0.5, 0.5, 2.0, &tilt, -14.0, -7.0, 1.2);
    b[3].obj = demo_create_ball(w, &m_roll, 0.5, 1.0, NULL, -14.0, -0.52, 3.6);
    for (int i = 0; i < N_BODIES; i++) {
        if (!b[i].obj) return 1;
        const double vx = i == 3 ? 4.5 : 2.5;
        b[i].obj->physical_core->speed = ivp_vec3r((ivp_real_t)vx, IVP_REAL(0.0), IVP_REAL(0.0));
    }
    /* the box rests on its side: half the diagonal of a tilted cube is not its resting height */
    b[2].rest_y = -0.5;

    if (ivp_psi_listener_register(&s->listener, w->env, record_psi, s) != 0) return 1;

    app.controls    = "F5 panel: trails and the energy chart";
    app.viewer_init = recorder_viewer_init;
    app.on_reset    = recorder_reset;
    app.draw        = recorder_draw;
    app.stats       = recorder_stats;
    app.check       = recorder_check;

    int rc = demo_run(&app, argc, argv);
    ivp_psi_listener_unregister(&s->listener, w->env);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
