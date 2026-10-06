/* example_phantom.c -- Trigger volume (phantom) demo
 *
 * A stationary sphere phantom at the center (drawn as wireframe) and
 * cubes/spheres dropped from above.  Enter/exit events are tracked.
 * Objects inside the phantom are drawn brighter.  HUD shows the
 * tracked count and last event.  Press K to kick the objects inside the
 * phantom straight up: they leave the trigger volume and fall back in.
 *
 * Demonstrates: ivp_controller_phantom, sphere phantom, enter/exit
 *               callbacks, tracked object list, HUD event log.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_controller_phantom.h>

#define EVENT_LOG_SIZE 5
#define N_DROP 10
#define KICK_SPEED 12.0     /* upward, m/s: ~7 m high, well out of the sphere */

/* ── Phantom event tracking ─────────────────────────────────────────────── */

typedef struct {
    int  total_enters;
    int  total_exits;
    int  enters[N_DROP];
    int  exits[N_DROP];
    char last_events[EVENT_LOG_SIZE][64];
    int  log_head;
    ivp_object_t **objs;
} phantom_tracker_t;

static void on_phantom_event(void *ctx, ivp_object_t *phantom_owner, ivp_object_t *other, bool entered)
{
    (void)phantom_owner;
    phantom_tracker_t *tr = ctx;
    if (entered) tr->total_enters++;
    else         tr->total_exits++;
    for (int i = 0; i < N_DROP; i++) {
        if (tr->objs[i] == other) {
            if (entered) tr->enters[i]++;
            else tr->exits[i]++;
        }
    }
    const char *name = other->name ? other->name : "?";
    snprintf(tr->last_events[tr->log_head], 64, "%s %s", entered ? "ENTER" : "EXIT", name);
    tr->log_head = (tr->log_head + 1) % EVENT_LOG_SIZE;
}

typedef struct phantom_demo {
    ivp_object_t        *ground;
    ivp_object_t        *drop_objs[N_DROP];
    char                 drop_names[N_DROP][16];
    ivp_phantom_state_t  phantom;
    phantom_tracker_t    tracker;
    int                  n_kicks;
    int                  kicked_inside;    /* objects inside at the (last) kick */
    bool                 kicked[N_DROP];
    int                  inconsistent;     /* frames where the tracked list != enters - exits */
} phantom_demo_t;

static const float drop_cols[][3] = {
    {1.0f, 0.4f, 0.2f}, {0.3f, 1.0f, 0.3f}, {0.3f, 0.5f, 1.0f},
    {1.0f, 1.0f, 0.2f}, {1.0f, 0.2f, 0.8f}, {0.2f, 1.0f, 0.9f},
    {0.9f, 0.6f, 0.1f}, {0.6f, 0.3f, 1.0f},
    {0.8f, 0.8f, 0.4f}, {0.4f, 0.8f, 0.8f}
};

static bool is_tracked(const phantom_demo_t *s, const ivp_object_t *obj)
{
    for (int t = 0; t < s->phantom.n_tracked; t++)
        if (s->phantom.tracked[t] == obj) return true;
    return false;
}

/* K: kick every object inside the phantom straight up */
static void kick_out(demo_app_t *app)
{
    phantom_demo_t *s = (phantom_demo_t *)app->user;
    s->n_kicks++;
    s->kicked_inside = 0;
    for (int i = 0; i < N_DROP; i++) {
        ivp_object_t *o = s->drop_objs[i];
        s->kicked[i] = false;
        if (!is_tracked(s, o)) continue;
        s->kicked_inside++;
        s->kicked[i] = true;
        demo_async_add_speed_ws(app->world.env, o, 0.0, -KICK_SPEED, 0.0);
    }
    printf("[phantom] t=%.2f kicked %d objects out of the phantom\n",
           ivp_environment_get_time(app->world.env), s->kicked_inside);
}

static void start_speed(phantom_demo_t *s, ivp_environment_t *env, bool async)
{
    /* Initial downward velocity for natural drop appearance */
    for (int i = 0; i < N_DROP; i++) {
        if (async) demo_async_add_speed_ws(env, s->drop_objs[i], 0.0, 0.5, 0.0);
        else s->drop_objs[i]->physical_core->speed = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.5), IVP_REAL(0.0));
    }
}

static void phantom_reset(demo_app_t *app)
{
    phantom_demo_t *s = (phantom_demo_t *)app->user;
    start_speed(s, app->world.env, true);
}

static void phantom_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    phantom_demo_t *s = (phantom_demo_t *)app->user;
    if (s->phantom.n_tracked != s->tracker.total_enters - s->tracker.total_exits) s->inconsistent++;
    if (v) {
        if (demo_key_just_pressed(v, SDL_SCANCODE_K)) kick_out(app);
        return;
    }
    if (frame == 300) kick_out(app);
}

static void phantom_draw(demo_app_t *app, demo_viewer_t *v)
{
    phantom_demo_t *s = (phantom_demo_t *)app->user;
    const float c_ground[]  = {0.22f, 0.24f, 0.30f};
    const float c_phantom[] = {1.0f, 1.0f, 0.2f};
    const float c_bright[]  = {1.0f, 1.0f, 1.0f};
    const float c_vel[]     = {0.2f, 1.0f, 0.4f};
    demo_draw_object(v, s->ground, c_ground);
    {
        float pc[3];
        vec3r_to_float3(&s->phantom.center_ws, pc);
        if (v->solid) {
            demo_set_alpha(v, 0.22f);
            demo_solid_sphere(v, pc, (float)s->phantom.radius, c_phantom);
            demo_set_alpha(v, 1.0f);
        }
        demo_sphere(v, pc, (float)s->phantom.radius, c_phantom);
    }
    for (int i = 0; i < N_DROP; i++) {
        ivp_object_t *o = s->drop_objs[i];
        demo_draw_object(v, o, is_tracked(s, o) ? c_bright : drop_cols[i]);
        if (demo_object_speed(o) > 0.5f) demo_draw_velocity(v, o, c_vel);
    }
    {
        float org[3] = {0, 0, 0};
        demo_axes(v, org, NULL, 1.5f);
    }
    demo_hud(v, c_phantom, "Phantom: tracked=%d  enters=%d  exits=%d",
             s->phantom.n_tracked, s->tracker.total_enters, s->tracker.total_exits);
    for (int i = 0; i < EVENT_LOG_SIZE; i++) {
        int idx = (s->tracker.log_head - 1 - i + EVENT_LOG_SIZE) % EVENT_LOG_SIZE;
        if (s->tracker.last_events[idx][0] != '\0')
            demo_hud(v, DEMO_COL_INFO, "%s", s->tracker.last_events[idx]);
    }
}

static void phantom_stats(demo_app_t *app, char *buf, size_t size)
{
    phantom_demo_t *s = (phantom_demo_t *)app->user;
    int n = snprintf(buf, size, "tracked=%d enters=%d exits=%d inside:", s->phantom.n_tracked,
                     s->tracker.total_enters, s->tracker.total_exits);
    for (int i = 0; i < N_DROP && n > 0 && (size_t)n < size; i++)
        if (is_tracked(s, s->drop_objs[i])) n += snprintf(buf + n, size - (size_t)n, " %d", i);
}

static bool phantom_check(demo_app_t *app)
{
    phantom_demo_t *s = (phantom_demo_t *)app->user;
    const phantom_tracker_t *tr = &s->tracker;
    const char *tag = app->cfg.tag;
    bool ok = true;
    /* objects dropped at 2 m from the axis fall into the sphere, the others
     * (5 m, 8 m) never reach it */
    int near_in = 0, n_near = 0, far_in = 0;
    for (int i = 0; i < N_DROP; i++) {
        if (i % 3 == 0) { n_near++; if (tr->enters[i] > 0) near_in++; }
        else if (tr->enters[i] > 0) far_in++;
    }
    ok &= demo_check(tag, near_in == n_near && far_in == 0,
                     "the %d objects dropped into the phantom entered it, none of the others (%d)", near_in, far_in);
    int back = 0;
    for (int i = 0; i < N_DROP; i++)
        if (s->kicked[i] && tr->exits[i] >= 1 && tr->enters[i] >= 2 && is_tracked(s, s->drop_objs[i])) back++;
    ok &= demo_check(tag, s->n_kicks == 1 && s->kicked_inside >= 3 && back == s->kicked_inside,
                     "kicked %d objects up: %d left the phantom and fell back in", s->kicked_inside, back);
    ok &= demo_check(tag, s->inconsistent == 0 && s->phantom.n_tracked == tr->total_enters - tr->total_exits,
                     "tracked list consistent with the events (%d tracked, %d enters, %d exits)",
                     s->phantom.n_tracked, tr->total_enters, tr->total_exits);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title        = "IVP Phantom -- Trigger Volumes";
    app.cfg.tag          = "phantom";
    app.cfg.orbit_dist   = 25.0f;
    app.cfg.orbit_yaw    = 20.0f;
    app.cfg.orbit_pitch  = -25.0f;
    app.cfg.target[1]    = -4.0f;
    app.cfg.grid_extent  = 15.0f;
    app.cfg.grid_spacing = 2.0f;

    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    phantom_demo_t *s = (phantom_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    s->ground = demo_create_box_templ(w, &gt, 15.0, 0.5, 15.0);

    /* ── Phantom: sphere at center, radius 3 ────────────────────────────── */
    s->tracker.objs = s->drop_objs;
    {
        ivp_vec3r_t center = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-3.0), IVP_REAL(0.0));
        ivp_phantom_init_sphere(&s->phantom, &center, IVP_REAL(3.0), on_phantom_event, &s->tracker);
    }
    ivp_environment_add_controller(w->env, 100, ivp_controller_phantom, &s->phantom);

    /* ── Drop objects: cubes and spheres from varying heights ────────────── */
    for (int i = 0; i < N_DROP; i++) {
        snprintf(s->drop_names[i], sizeof(s->drop_names[i]), "obj_%d", i);
        ivp_object_template_t t;
        ivp_object_template_init(&t);
        t.mass = IVP_REAL(1.0) + IVP_REAL(0.5) * (ivp_real_t)i;
        t.material.friction   = IVP_REAL(0.5);
        t.material.elasticity = IVP_REAL(0.4);
        t.name = s->drop_names[i];
        float angle = 6.2832f * (float)i / (float)N_DROP;
        float r = 2.0f + 3.0f * (float)(i % 3);
        t.position = ivp_vec3r((ivp_real_t)(r * cosf(angle)),
                               IVP_REAL(-8.0) - IVP_REAL(2.0) * (ivp_real_t)(i % 4),
                               (ivp_real_t)(r * sinf(angle)));
        if (i % 2 == 0) {
            set_box_inertia(&t, t.mass, IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(0.5));
            s->drop_objs[i] = demo_create_box_templ(w, &t, 0.5, 0.5, 0.5);
        } else {
            set_sphere_inertia(&t, t.mass, IVP_REAL(0.5));
            s->drop_objs[i] = demo_create_ball_templ(w, &t, 0.5);
        }
    }
    start_speed(s, w->env, false);

    app.controls = "K: kick the objects inside the phantom upward";
    app.update   = phantom_update;
    app.on_reset = phantom_reset;
    app.draw     = phantom_draw;
    app.stats    = phantom_stats;
    app.check    = phantom_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
