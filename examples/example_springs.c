/* example_springs.c -- Spring lattice and trampoline
 *
 * Multiple objects connected by springs in a network.  A heavy sphere
 * is dropped onto a spring-bridge structure.  Force arrows show spring
 * tension.  An anchored spring acts as a bungee cord.
 *
 * Demonstrates: ivp_actuator_spring, multiple springs, stiffness/damping,
 *               rest length, sphere, force visualization, HUD.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_actuator_spring.h>

#define N_BRIDGE 4        /* bridge nodes */
#define MAX_SPRINGS 16
#define GRAVITY 9.81
#define BRIDGE_Y (-7.0f)
#define BRIDGE_K 400.0
#define BRIDGE_DAMP 10.0
#define BUNGEE_K 26.0
#define BUNGEE_REST 8.0
#define BUNGEE_MASS 3.0

typedef struct springs_demo {
    ivp_object_t      *ground, *pillar_l, *pillar_r, *anchor, *ball, *bungee;
    ivp_object_t      *nodes[N_BRIDGE];
    ivp_spring_state_t springs[MAX_SPRINGS];
    int                n_springs;
    /* tracking */
    float              max_bridge_sag, max_bridge_force;
    float              late_bridge_force;                /* max |force| after 1.5 s */
    float              early_min, early_max;             /* bridge centre height 1..4 s */
    float              late_min, late_max;               /* 7..10 s */
} springs_demo_t;

static float bridge_mean_y(const springs_demo_t *s)
{
    float y = 0.0f;
    for (int i = 0; i < N_BRIDGE; i++) y += (float)s->nodes[i]->physical_core->pos_world_f_core_last_psi.k[1];
    return y / (float)N_BRIDGE;
}

static void reset_tracking(springs_demo_t *s)
{
    s->max_bridge_sag = 0.0f;
    s->max_bridge_force = 0.0f;
    s->late_bridge_force = 0.0f;
    s->early_min = s->late_min = 1e9f;
    s->early_max = s->late_max = -1e9f;
}

static void springs_reset(demo_app_t *app)
{
    springs_demo_t *s = (springs_demo_t *)app->user;
    reset_tracking(s);
    /* initial downward velocity of the ball (~0.5 m free fall) */
    demo_async_add_speed_ws(app->world.env, s->ball, 0.0, 1.0, 0.0);
}

static void springs_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    springs_demo_t *s = (springs_demo_t *)app->user;
    (void)v;
    if (frame == 0) return;   /* spring lengths are computed by the first PSI */
    double t = ivp_environment_get_time(app->world.env);
    float y = bridge_mean_y(s);
    if (y - BRIDGE_Y > s->max_bridge_sag) s->max_bridge_sag = y - BRIDGE_Y;
    for (int i = 0; i < N_BRIDGE + 1; i++) {
        float f = fabsf((float)s->springs[i].current_force);
        if (f > s->max_bridge_force) s->max_bridge_force = f;
        if (t > 1.5 && f > s->late_bridge_force) s->late_bridge_force = f;
    }
    if (t >= 1.0 && t < 4.0) {
        if (y < s->early_min) s->early_min = y;
        if (y > s->early_max) s->early_max = y;
    } else if (t >= 7.0 && t < 10.0) {
        if (y < s->late_min) s->late_min = y;
        if (y > s->late_max) s->late_max = y;
    }
}

static void springs_draw(demo_app_t *app, demo_viewer_t *v)
{
    springs_demo_t *s = (springs_demo_t *)app->user;
    const float c_node[]   = {1.0f, 0.5f, 0.1f};
    const float c_ball[]   = {0.2f, 0.8f, 1.0f};
    const float c_bungee[] = {0.9f, 0.3f, 0.9f};
    const float c_anchor[] = {0.5f, 0.5f, 0.5f};
    const float c_pillar[] = {0.45f, 0.40f, 0.35f};
    const float c_spring[] = {1.0f, 1.0f, 0.2f};
    const float c_ground[] = {0.30f, 0.30f, 0.35f};
    const float c_vel[]    = {0.20f, 1.0f,  0.40f};

    demo_draw_object(v, s->ground, c_ground);
    demo_draw_object(v, s->pillar_l, c_pillar);
    demo_draw_object(v, s->pillar_r, c_pillar);
    for (int i = 0; i < N_BRIDGE; i++) demo_draw_object(v, s->nodes[i], c_node);

    /* spring lines between the anchor points */
    for (int i = 0; i < s->n_springs; i++) {
        float a[3], b[3];
        demo_core_point_ws(s->springs[i].obj_a, &s->springs[i].anchor_a_os, a);
        demo_core_point_ws(s->springs[i].obj_b, &s->springs[i].anchor_b_os, b);
        demo_thick_line(v, a, b, c_spring, 2);
    }

    demo_draw_object(v, s->ball, c_ball);
    demo_draw_velocity(v, s->ball, c_vel);
    demo_draw_object(v, s->anchor, c_anchor);
    demo_draw_object(v, s->bungee, c_bungee);
    demo_draw_velocity(v, s->bungee, c_vel);
    {
        float org[3] = {0, 0, 0};
        demo_axes(v, org, NULL, 1.5f);
    }

    demo_hud(v, DEMO_COL_INFO, "Springs: %d  Time: %.1fs", s->n_springs,
             (double)ivp_environment_get_time(app->world.env));
    demo_hud_object(v, "Ball", s->ball);
    demo_hud_object(v, "Bungee", s->bungee);
    demo_hud(v, DEMO_COL_INFO, "Bridge sag %.2f m (max %.2f)  max spring force %.1f N (clamp 80)",
             (double)(bridge_mean_y(s) - BRIDGE_Y), (double)s->max_bridge_sag, (double)s->max_bridge_force);
    demo_hud(v, c_bungee, "Bungee length %.2f m (rest %.1f, k=%.0f N/m)",
             (double)s->springs[N_BRIDGE + 1].current_length, BUNGEE_REST, BUNGEE_K);
}

static void springs_stats(demo_app_t *app, char *buf, size_t size)
{
    springs_demo_t *s = (springs_demo_t *)app->user;
    float bp[3];
    demo_object_center(s->ball, bp);
    snprintf(buf, size, "bridge sag=%.3f (max %.3f) F=%.0f/%.0f/%.0f/%.0f/%.0f N | ball (%.2f,%.2f,%.2f) v=%.3f | bungee len=%.3f v=%.3f",
             (double)(bridge_mean_y(s) - BRIDGE_Y), (double)s->max_bridge_sag,
             (double)s->springs[0].current_force, (double)s->springs[1].current_force,
             (double)s->springs[2].current_force, (double)s->springs[3].current_force,
             (double)s->springs[4].current_force,
             (double)bp[0], (double)bp[1], (double)bp[2], (double)demo_object_speed(s->ball),
             (double)s->springs[N_BRIDGE + 1].current_length, (double)demo_object_speed(s->bungee));
}

static bool springs_check(demo_app_t *app)
{
    springs_demo_t *s = (springs_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    float early = s->early_max - s->early_min, late = s->late_max - s->late_min;
    ok &= demo_check(tag, s->max_bridge_sag > 0.5f && s->max_bridge_sag < 3.0f,
                     "bridge sags under its load (max %.2f m)", (double)s->max_bridge_sag);
    ok &= demo_check(tag, early > 0.3f && late < 0.5f * early,
                     "bridge oscillates and settles (centre height range %.3f m at 1..4 s, %.3f m at 7..10 s)",
                     (double)early, (double)late);
    ok &= demo_check(tag, s->late_bridge_force < 80.0f,
                     "bridge spring forces stay below the 80 N clamp after the impact (max %.1f N)",
                     (double)s->late_bridge_force);
    float bp[3];
    demo_object_center(s->ball, bp);
    ok &= demo_check(tag, fabsf(bp[0]) < 1.2f && bp[1] < BRIDGE_Y + 2.0f && bp[1] > BRIDGE_Y - 1.0f,
                     "ball rests on the bridge between the middle nodes (%.2f, %.2f, %.2f)",
                     (double)bp[0], (double)bp[1], (double)bp[2]);
    /* static stretch of the bungee: m g / k */
    float expect = (float)(BUNGEE_REST + BUNGEE_MASS * GRAVITY / BUNGEE_K);
    float len = (float)s->springs[N_BRIDGE + 1].current_length;
    ok &= demo_check(tag, fabsf(len - expect) < 0.05f && demo_object_speed(s->bungee) < 0.01f,
                     "bungee settled at rest length + m g / k: %.3f m (expected %.3f m)",
                     (double)len, (double)expect);
    return ok;
}

static ivp_spring_state_t *add_spring(demo_app_t *app, ivp_object_t *a, ivp_vec3r_t pa,
                                      ivp_object_t *b, ivp_vec3r_t pb,
                                      double k, double damp, double rest, double max_force)
{
    springs_demo_t *s = (springs_demo_t *)app->user;
    ivp_spring_state_t *sp = &s->springs[s->n_springs++];
    ivp_spring_state_init(sp);
    ivp_spring_state_setup(sp, a, &pa, b, &pb, (ivp_real_t)k, (ivp_real_t)damp, (ivp_real_t)rest);
    sp->max_force = (ivp_real_t)max_force;
    ivp_environment_add_controller(app->world.env, 100, ivp_controller_spring, sp);
    return sp;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title        = "IVP Springs -- Bridge & Bungee";
    app.cfg.tag          = "springs";
    app.cfg.orbit_dist   = 28.0f;
    app.cfg.orbit_yaw    = 25.0f;
    app.cfg.orbit_pitch  = -20.0f;
    app.cfg.target[1]    = -4.0f;
    app.cfg.grid_extent  = 15.0f;
    app.cfg.grid_spacing = 2.0f;

    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    /* Standard Earth gravity (Y-down). */
    ivp_vec3r_set(&ecfg.gravity, IVP_REAL(0.0), (ivp_real_t)GRAVITY, IVP_REAL(0.0));
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    springs_demo_t *s = (springs_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    /* ── Ground ─────────────────────────────────────────────────────────── */
    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    s->ground = demo_create_box_templ(w, &gt, 15.0, 0.5, 15.0);

    /* ── Bridge pillars (static) ────────────────────────────────────────── */
    ivp_object_template_t pilt;
    ivp_object_template_init(&pilt);
    pilt.mass = 0.0f;
    pilt.physical_unmoveable = true;
    pilt.position = ivp_vec3r(IVP_REAL(-6.0), IVP_REAL(-7.0), IVP_REAL(0.0));
    s->pillar_l = demo_create_box_templ(w, &pilt, 0.5, 2.0, 0.5);
    pilt.position = ivp_vec3r(IVP_REAL(6.0), IVP_REAL(-7.0), IVP_REAL(0.0));
    s->pillar_r = demo_create_box_templ(w, &pilt, 0.5, 2.0, 0.5);

    /* ── Bridge nodes (dynamic cubes hanging on springs) ────────────────── */
    float node_spacing = 12.0f / (float)(N_BRIDGE + 1);
    for (int i = 0; i < N_BRIDGE; i++) {
        ivp_object_template_t nt;
        ivp_object_template_init(&nt);
        nt.mass = IVP_REAL(0.6);
        set_box_inertia(&nt, IVP_REAL(0.6), IVP_REAL(0.5), IVP_REAL(0.5), IVP_REAL(0.5));
        nt.material.friction   = IVP_REAL(0.7);
        nt.material.elasticity = IVP_REAL(0.05);
        nt.speed_damp_factor     = IVP_REAL(0.15);
        nt.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        float nx = -6.0f + node_spacing * (float)(i + 1);
        nt.position = ivp_vec3r((ivp_real_t)nx, (ivp_real_t)BRIDGE_Y, IVP_REAL(0.0));
        s->nodes[i] = demo_create_box_templ(w, &nt, 0.5, 0.5, 0.5);
    }

    /* ── Heavy sphere (dropped onto bridge) ─────────────────────────────── */
    ivp_object_template_t bt;
    ivp_object_template_init(&bt);
    bt.mass = IVP_REAL(0.5);
    set_sphere_inertia(&bt, IVP_REAL(0.5), IVP_REAL(0.8));
    bt.material.friction   = IVP_REAL(0.5);
    bt.material.elasticity = IVP_REAL(0.0);
    bt.speed_damp_factor = IVP_REAL(0.0);
    bt.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    bt.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-10.0), IVP_REAL(0.0));
    s->ball = demo_create_ball_templ(w, &bt, 0.8);
    /* Initial downward velocity (~0.5m free-fall equivalent) for natural drop */
    s->ball->physical_core->speed = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));

    /* ── Bungee cube (attached to ceiling spring) ───────────────────────── */
    ivp_object_template_t bgt;
    ivp_object_template_init(&bgt);
    bgt.mass = (ivp_real_t)BUNGEE_MASS;
    set_box_inertia(&bgt, bgt.mass, IVP_REAL(0.6), IVP_REAL(0.6), IVP_REAL(0.6));
    bgt.material.friction   = IVP_REAL(0.6);
    bgt.material.elasticity = IVP_REAL(0.0);
    bgt.speed_damp_factor = IVP_REAL(0.0);
    bgt.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    bgt.position = ivp_vec3r(IVP_REAL(10.0), IVP_REAL(-6.0), IVP_REAL(0.0));
    s->bungee = demo_create_box_templ(w, &bgt, 0.6, 0.6, 0.6);

    /* ── Static ceiling anchor for bungee ───────────────────────────────── */
    ivp_object_template_t at;
    ivp_object_template_init(&at);
    at.mass = 0.0f;
    at.physical_unmoveable = true;
    at.position = ivp_vec3r(IVP_REAL(10.0), IVP_REAL(-14.0), IVP_REAL(0.0));
    s->anchor = demo_create_box_templ(w, &at, 0.3, 0.3, 0.3);

    /* ── Springs ────────────────────────────────────────────────────────── *
     * Bridge springs: pillar_l -> node[0] -> ... -> node[N-1] -> pillar_r,
     * k=400 N/m, damping 10 N*s/m: damping ratio ~0.3 for a 0.6 kg node
     * (underdamped, visible oscillation).  (The damping used to be 50: the
     * springs act explicitly once per PSI, and 50 N*s/m removed 2.5 times
     * the relative velocity of two 0.6 kg nodes per 1/66 s PSI -- the
     * damping overshot and diverged, the forces flipped between +-80 N (the
     * max_force clamp) every PSI and the ball finally slipped through.) */
    {
        ivp_real_t rest = (ivp_real_t)node_spacing - IVP_REAL(1.0);
        if (rest < IVP_REAL(0.3)) rest = IVP_REAL(0.3);
        ivp_vec3r_t right = ivp_vec3r(IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_t left = ivp_vec3r(IVP_REAL(-0.5), IVP_REAL(0.0), IVP_REAL(0.0));
        add_spring(&app, s->pillar_l, right, s->nodes[0], left, BRIDGE_K, BRIDGE_DAMP, rest, 80.0);
        for (int i = 1; i < N_BRIDGE; i++)
            add_spring(&app, s->nodes[i - 1], right, s->nodes[i], left, BRIDGE_K, BRIDGE_DAMP, rest, 80.0);
        add_spring(&app, s->nodes[N_BRIDGE - 1], right, s->pillar_r, left, BRIDGE_K, BRIDGE_DAMP, rest, 80.0);
    }
    /* Bungee: k=26 N/m supports 3kg at ~1.1m extension; damping 12. */
    add_spring(&app, s->anchor, ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.3), IVP_REAL(0.0)),
               s->bungee, ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.6), IVP_REAL(0.0)),
               BUNGEE_K, 12.0, BUNGEE_REST, 60.0);
    reset_tracking(s);

    app.update   = springs_update;
    app.on_reset = springs_reset;
    app.draw     = springs_draw;
    app.stats    = springs_stats;
    app.check    = springs_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
