/* example_platform.c -- Platforms driven by golem controllers
 *
 * Three heavy boxes follow a scripted path through IVP_Controller_Golem: an
 * elevator going up and down, a slider moving sideways and a turntable. Each
 * frame the example hands the controller the next prime position (with its
 * velocity) or orientation; the golem drives the core towards it with a
 * velocity feed-forward. Crates stand on the platforms and are carried by
 * friction. Space resets the scene.
 *
 * Demonstrates: ivp_golem_state_init / _setup, ivp_golem_set_prime_position
 *               and _set_prime_orientation, a core controller at
 *               IVP_CP_MOTION, kinematic-style platforms that still collide.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_controller_golem.h>

#define N_PLATFORMS  3
#define CRATES_PER   4
#define N_CRATES     (N_PLATFORMS * CRATES_PER)

enum { P_ELEVATOR, P_SLIDER, P_TURNTABLE };

typedef struct platform {
    ivp_object_t     *obj;
    ivp_golem_state_t golem;
    double            half[3];
    double            rest[3];       /* path centre */
    int               problems;
} platform_t;

typedef struct platform_demo {
    ivp_object_t *ground;
    platform_t    plat[N_PLATFORMS];
    ivp_object_t *crates[N_CRATES];
    float         worst_error;       /* largest distance to the golem target */
} platform_demo_t;

static const char *plat_name[N_PLATFORMS] = { "elevator", "slider", "turntable" };

/* Path of platform p at time t: position (and velocity) or rotation angle. */
static void plat_target(int p, double t, double pos[3], double vel[3], double *angle)
{
    const double two_pi = 6.28318530717958647692;
    pos[0] = pos[1] = pos[2] = 0.0;
    vel[0] = vel[1] = vel[2] = 0.0;
    *angle = 0.0;
    switch (p) {
    case P_ELEVATOR: {                           /* up is -Y */
        const double A = 3.0, w = two_pi / 7.0;
        pos[1] = -A * sin(w * t);
        vel[1] = -A * w * cos(w * t);
        break;
    }
    case P_SLIDER: {
        const double A = 3.5, w = two_pi / 8.0;
        pos[0] = A * sin(w * t);
        vel[0] = A * w * cos(w * t);
        break;
    }
    default:
        *angle = 0.55 * t;
        break;
    }
}

static void problem_cb(void *ctx, ivp_object_t *object, ivp_real_t pos_err, ivp_real_t rot_err)
{
    (void)object; (void)pos_err; (void)rot_err;
    (*(int *)ctx)++;
}

static void drive(platform_demo_t *s, ivp_environment_t *env)
{
    const ivp_time_t now = ivp_environment_get_time(env);
    const double t = (double)now;
    for (int p = 0; p < N_PLATFORMS; p++) {
        platform_t *pl = &s->plat[p];
        double pos[3], vel[3], angle, pos2[3], vel2[3], angle2;
        plat_target(p, t, pos, vel, &angle);
        if (p == P_TURNTABLE) {
            /* orientation now and 0.1 s later: the golem derives the spin */
            plat_target(p, t + 0.1, pos2, vel2, &angle2);
            const ivp_quatr_t q0 = ivp_quatr_make(0.0, sin(0.5 * angle), 0.0, cos(0.5 * angle));
            const ivp_quatr_t q1 = ivp_quatr_make(0.0, sin(0.5 * angle2), 0.0, cos(0.5 * angle2));
            ivp_golem_set_prime_orientation(&pl->golem, &q0, now, &q1, IVP_REAL(0.1));
        } else {
            const ivp_vec3r_t pp = ivp_vec3r((ivp_real_t)(pl->rest[0] + pos[0]),
                                             (ivp_real_t)(pl->rest[1] + pos[1]),
                                             (ivp_real_t)(pl->rest[2] + pos[2]));
            const ivp_vec3r_t vv = ivp_vec3r((ivp_real_t)vel[0], (ivp_real_t)vel[1], (ivp_real_t)vel[2]);
            ivp_golem_set_prime_position(&pl->golem, &pp, &vv, now);
        }
    }
}

static void platform_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    platform_demo_t *s = (platform_demo_t *)app->user;
    (void)v; (void)frame;
    drive(s, app->world.env);
    for (int p = 0; p < N_PLATFORMS; p++) {
        platform_t *pl = &s->plat[p];
        float c[3];
        demo_object_center(pl->obj, c);
        float dx = c[0] - (float)pl->golem.target_pos_ws.k[0];
        float dy = c[1] - (float)pl->golem.target_pos_ws.k[1];
        float dz = c[2] - (float)pl->golem.target_pos_ws.k[2];
        float e = sqrtf(dx * dx + dy * dy + dz * dz);
        if (ivp_environment_get_time(app->world.env) > 1.0 && e > s->worst_error) s->worst_error = e;
    }
}

static void platform_reset(demo_app_t *app)
{
    platform_demo_t *s = (platform_demo_t *)app->user;
    s->worst_error = 0.0f;
    for (int p = 0; p < N_PLATFORMS; p++) ivp_golem_state_setup(&s->plat[p].golem, s->plat[p].obj);
}

static const float plat_col[N_PLATFORMS][3] = {
    { 0.35f, 0.65f, 0.95f }, { 0.95f, 0.70f, 0.25f }, { 0.55f, 0.85f, 0.50f } };
static const float crate_col[CRATES_PER][3] = {
    { 0.95f, 0.40f, 0.30f }, { 0.90f, 0.85f, 0.30f }, { 0.75f, 0.45f, 0.90f }, { 0.95f, 0.95f, 0.95f } };

/* does crate c lie on platform p (above its top face, inside its footprint)? */
static bool on_platform(const platform_t *pl, const ivp_object_t *crate)
{
    float cp[3], pp[3];
    demo_object_center(crate, cp);
    demo_object_center(pl->obj, pp);
    float above = pp[1] - cp[1];               /* up is -Y */
    float lim = (float)(pl->half[0] + 0.5);
    return above > (float)pl->half[1] && above < (float)pl->half[1] + 1.4f &&
           fabsf(cp[0] - pp[0]) < lim && fabsf(cp[2] - pp[2]) < (float)(pl->half[2] + 0.5);
}

static void platform_draw(demo_app_t *app, demo_viewer_t *v)
{
    platform_demo_t *s = (platform_demo_t *)app->user;
    const float c_ground[3] = { 0.55f, 0.58f, 0.62f };
    demo_draw_object(v, s->ground, c_ground);
    for (int p = 0; p < N_PLATFORMS; p++) {
        demo_draw_object(v, s->plat[p].obj, plat_col[p]);
        /* where the golem wants the platform */
        float t[3] = { (float)s->plat[p].golem.target_pos_ws.k[0],
                       (float)s->plat[p].golem.target_pos_ws.k[1],
                       (float)s->plat[p].golem.target_pos_ws.k[2] };
        demo_cross(v, t, 0.3f, DEMO_COL_HIGHLIGHT);
    }
    for (int i = 0; i < N_CRATES; i++) demo_draw_object(v, s->crates[i], crate_col[i % CRATES_PER]);

    for (int p = 0; p < N_PLATFORMS; p++) {
        int carried = 0;
        for (int i = 0; i < N_CRATES; i++) carried += on_platform(&s->plat[p], s->crates[i]);
        float c[3];
        demo_object_center(s->plat[p].obj, c);
        demo_hud(v, DEMO_COL_INFO, "%-9s pos (%.2f, %.2f, %.2f)  carries %d crates", plat_name[p],
                 (double)c[0], (double)c[1], (double)c[2], carried);
    }
    demo_hud(v, DEMO_COL_INFO, "worst distance to the golem target: %.3f m", (double)s->worst_error);
}

static void platform_stats(demo_app_t *app, char *buf, size_t size)
{
    platform_demo_t *s = (platform_demo_t *)app->user;
    snprintf(buf, size, "t=%.1f worst golem error %.3f m", ivp_environment_get_time(app->world.env),
             (double)s->worst_error);
}

static bool platform_check(demo_app_t *app)
{
    platform_demo_t *s = (platform_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, s->worst_error < 0.4f, "platforms follow their golem targets (worst %.3f m)",
                     (double)s->worst_error);
    int problems = 0;
    for (int p = 0; p < N_PLATFORMS; p++) problems += s->plat[p].problems;
    ok &= demo_check(tag, problems == 0, "no golem problem callbacks (%d)", problems);
    for (int p = 0; p < N_PLATFORMS; p++) {
        int carried = 0;
        for (int i = 0; i < CRATES_PER; i++) carried += on_platform(&s->plat[p], s->crates[p * CRATES_PER + i]);
        ok &= demo_check(tag, carried == CRATES_PER, "%s still carries %d/%d crates", plat_name[p], carried,
                         CRATES_PER);
    }
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Platform -- Golem Controllers";
    app.cfg.tag         = "platform";
    app.cfg.friction    = 0.9;
    app.cfg.elasticity  = 0.0;
    app.cfg.orbit_dist  = 19.0f;
    app.cfg.target[1]   = -3.0f;
    app.cfg.grid_extent = -1.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    platform_demo_t *s = (platform_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;

    s->ground = demo_create_box(&app.world, NULL, 12.0, 0.5, 12.0, 0.0, NULL, 0.0, 0.0, 0.0);

    /* half extents and resting centres: up is -Y, the ground top is at -0.5 */
    static const double half[N_PLATFORMS][3] = { { 2.0, 0.25, 2.0 }, { 2.0, 0.25, 1.5 }, { 2.2, 0.2, 2.2 } };
    static const double rest[N_PLATFORMS][3] = { { -5.0, -4.2, -6.5 }, { 0.0, -2.4, 0.0 }, { 5.0, -1.6, 6.5 } };
    for (int p = 0; p < N_PLATFORMS; p++) {
        platform_t *pl = &s->plat[p];
        memcpy(pl->half, half[p], sizeof(pl->half));
        memcpy(pl->rest, rest[p], sizeof(pl->rest));
        const double mass = 800.0;
        ivp_object_template_t t;
        demo_template_dynamic(&t, &app.world.mat, mass);
        demo_set_box_inertia(&t, mass, half[p][0], half[p][1], half[p][2]);
        t.speed_damp_factor = IVP_REAL(0.0);
        t.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        demo_template_pose(&t, NULL, rest[p][0], rest[p][1], rest[p][2]);
        pl->obj = demo_create_box_templ(&app.world, &t, half[p][0], half[p][1], half[p][2]);
        if (!pl->obj) return 1;

        ivp_golem_state_init(&pl->golem);
        pl->golem.max_translation_force = ivp_vec3r(IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
        pl->golem.max_torque = ivp_vec3r(IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
        pl->golem.problem_fn = problem_cb;
        pl->golem.problem_ctx = &pl->problems;
        ivp_controller_handle_t h = ivp_environment_create_core_controller(
            app.world.env, IVP_CP_MOTION, ivp_controller_golem, &pl->golem);
        ivp_environment_add_controller_to_core(app.world.env, h, pl->obj);
        ivp_golem_state_setup(&pl->golem, pl->obj);

        /* crates on the top face, in a 2 x 2 pattern */
        for (int i = 0; i < CRATES_PER; i++) {
            double cx = rest[p][0] + ((i & 1) ? 0.9 : -0.9);
            double cz = rest[p][2] + ((i & 2) ? 0.8 : -0.8);
            double cy = rest[p][1] - half[p][1] - 0.5 - 0.02;
            s->crates[p * CRATES_PER + i] =
                demo_create_box(&app.world, NULL, 0.5, 0.5, 0.5, 6.0, NULL, cx, cy, cz);
            if (!s->crates[p * CRATES_PER + i]) return 1;
        }
    }

    app.update  = platform_update;
    app.on_reset = platform_reset;
    app.draw    = platform_draw;
    app.stats   = platform_stats;
    app.check   = platform_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
