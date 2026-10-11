/* example_rope.c -- Rope / pendulum chain with weight
 *
 * A longer chain of links connected by ball-socket constraints,
 * anchored to a static ceiling bar.  A heavy weight hangs at the end.
 * A ground plane catches anything that falls.  Joints are highlighted.
 * Press K to kick the weight sideways (the chain swings like a pendulum),
 * F1 toggles the solid / wireframe view.
 *
 * Demonstrates: ivp_constraint (ball_socket), chain building,
 *               stacking objects, weight at chain end, HUD text.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_constraint.h>
#include <ivp/controllers/ivp_controller.h>

#define CHAIN_LEN   8
#define LINK_HX     0.20f
#define LINK_HY     0.55f
#define LINK_HZ     0.20f
#define LINK_GAP    0.00f
#define WEIGHT_HS   0.6f
#define INITIAL_SWING_X 0.02f
#define ROPE_SIM_DT  (1.0f / 600.0f)
#define CEILING_Y   (-15.0f)
#define KICK_SPEED  6.0
/* Ceiling bar, links and weight share a no-collision group (reference
 * nocoll_group_ident, like ive_quickstart_rope): the joints hold the links
 * end to end, and contacts between them only fought the constraints. */
#define ROPE_GROUP  "rope"

typedef struct rope_demo {
    ivp_object_t          *ceiling, *ground, *weight;
    ivp_object_t          *links[CHAIN_LEN];
    ivp_constraint_state_t constraints[CHAIN_LEN + 1];
    int                    n_kicks;
    float                  max_joint_gap;       /* whole run */
    int                    worst_joint;
    float                  peak_swing_x;        /* since the last kick */
    float                  peak_speed;
    float                  swing_early;         /* |x| peak 1..3 s after the kick */
    float                  swing_late;          /* |x| peak 7..9 s after the kick */
    double                 kick_time;
} rope_demo_t;

/* Largest separation between the two world-space anchors of any joint. */
static float joint_gap(const rope_demo_t *s, int *worst)
{
    float max_gap = 0.0f;
    int worst_joint = -1;
    for (int i = 0; i <= CHAIN_LEN; i++) {
        const ivp_constraint_state_t *constraint = &s->constraints[i];
        float a[3], b[3];
        demo_core_point_ws(constraint->obj_a, &constraint->anchor_a_os, a);
        demo_core_point_ws(constraint->obj_b, &constraint->anchor_b_os, b);
        float gap = demo_distance(a, b);
        if (gap > max_gap) {
            max_gap = gap;
            worst_joint = i;
        }
    }
    if (worst) *worst = worst_joint;
    return max_gap;
}

static void kick(demo_app_t *app)
{
    rope_demo_t *s = (rope_demo_t *)app->user;
    demo_async_add_speed_ws(app->world.env, s->weight, KICK_SPEED, 0.0, 0.0);
    s->n_kicks++;
    s->peak_swing_x = 0.0f;
    s->peak_speed = 0.0f;
    s->swing_early = s->swing_late = 0.0f;
    s->kick_time = ivp_environment_get_time(app->world.env);
}

static void rope_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    rope_demo_t *s = (rope_demo_t *)app->user;
    if (v) {
        if (demo_key_just_pressed(v, SDL_SCANCODE_K)) kick(app);
    } else {
        if (frame == 60) kick(app);
    }
    int worst = -1;
    float gap = joint_gap(s, &worst);
    if (gap > s->max_joint_gap) {
        s->max_joint_gap = gap;
        s->worst_joint = worst;
    }
    float wx = fabsf((float)s->weight->physical_core->pos_world_f_core_last_psi.k[0]);
    if (wx > s->peak_swing_x) s->peak_swing_x = wx;
    float sp = demo_object_speed(s->weight);
    if (sp > s->peak_speed) s->peak_speed = sp;
    if (s->n_kicks > 0) {
        double dt = ivp_environment_get_time(app->world.env) - s->kick_time;
        if (dt >= 1.0 && dt < 3.0 && wx > s->swing_early) s->swing_early = wx;
        if (dt >= 7.0 && dt < 9.0 && wx > s->swing_late) s->swing_late = wx;
    }
}

static void rope_reset(demo_app_t *app)
{
    rope_demo_t *s = (rope_demo_t *)app->user;
    s->max_joint_gap = 0.0f;
    s->peak_swing_x = 0.0f;
    s->peak_speed = 0.0f;
}

static void rope_draw(demo_app_t *app, demo_viewer_t *v)
{
    rope_demo_t *s = (rope_demo_t *)app->user;
    const bool wire = !v->solid;
    const float c_link[]   = {0.85f, 0.65f, 0.20f};
    const float c_weight[] = {0.90f, 0.25f, 0.20f};
    const float c_ceil[]   = {0.50f, 0.50f, 0.55f};
    const float c_joint[]  = {1.0f,  0.30f, 0.30f};
    const float c_ground[] = {0.58f, 0.58f, 0.60f};
    const float c_grid[]   = {0.12f, 0.12f, 0.18f};
    const float c_vel[]    = {0.20f, 1.0f,  0.40f};

    if (wire) demo_draw_grid_ex(v, 12.0f, 2.0f, c_grid);
    demo_draw_object(v, s->ground, c_ground);
    demo_draw_object(v, s->ceiling, c_ceil);

    float prev[3];
    demo_object_center(s->ceiling, prev);
    for (int i = 0; i < CHAIN_LEN; i++) {
        float lc[3] = {c_link[0], c_link[1], c_link[2]};
        if (i % 2) { lc[0] *= 0.8f; lc[1] *= 0.8f; lc[2] *= 0.8f; }
        demo_draw_object(v, s->links[i], lc);
        float p[3];
        demo_object_center(s->links[i], p);
        float jp[3] = {(prev[0] + p[0]) * 0.5f, (prev[1] + p[1]) * 0.5f, (prev[2] + p[2]) * 0.5f};
        if (wire) demo_sphere(v, jp, 0.08f, c_joint);
        else demo_solid_sphere(v, jp, 0.08f, c_joint);
        prev[0] = p[0]; prev[1] = p[1]; prev[2] = p[2];
    }
    demo_draw_object(v, s->weight, c_weight);
    if (demo_object_speed(s->weight) > 0.3f) {
        float p[3];
        demo_object_center(s->weight, p);
        const ivp_vec3r_t *w = &s->weight->physical_core->speed;
        float to[3] = {p[0] + (float)w->k[0] * 0.4f, p[1] + (float)w->k[1] * 0.4f, p[2] + (float)w->k[2] * 0.4f};
        demo_arrow(v, p, to, c_vel, 0.25f);
    }

    int worst = -1;
    float gap = joint_gap(s, &worst);
    demo_hud(v, DEMO_COL_INFO, "Chain: %d links  View: %s (F1)  psi: %.4f s  kicks: %d",
             CHAIN_LEN, wire ? "wire" : "solid",
             (double)ivp_environment_get_delta_psi_time(app->world.env), s->n_kicks);
    demo_hud_object(v, "Weight", s->weight);
    demo_hud(v, DEMO_COL_INFO, "joint gap: %.3f m (joint %d)  peak swing x: %.2f m",
             (double)gap, worst, (double)s->peak_swing_x);
}

static void rope_stats(demo_app_t *app, char *buf, size_t size)
{
    rope_demo_t *s = (rope_demo_t *)app->user;
    int worst = -1;
    float gap = joint_gap(s, &worst);
    float wp[3], mean_x = 0.0f;
    demo_object_center(s->weight, wp);
    for (int i = 0; i < CHAIN_LEN; i++)
        mean_x += (float)s->links[i]->physical_core->pos_world_f_core_last_psi.k[0];
    snprintf(buf, size, "weight (%.2f,%.2f,%.2f) speed=%.3f joint_gap=%.4f (joint %d, max %.4f) chain_mean_x=%.3f",
             (double)wp[0], (double)wp[1], (double)wp[2], (double)demo_object_speed(s->weight),
             (double)gap, worst, (double)s->max_joint_gap, (double)(mean_x / (float)CHAIN_LEN));
}

static bool rope_check(demo_app_t *app)
{
    rope_demo_t *s = (rope_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    int worst = -1;
    float gap = joint_gap(s, &worst);
    ok &= demo_check(tag, s->max_joint_gap < 0.06f && gap < 0.04f,
                     "chain stays connected (joint gap %.4f m now, max %.4f m at joint %d)",
                     (double)gap, (double)s->max_joint_gap, s->worst_joint);
    ok &= demo_check(tag, s->n_kicks == 1 && s->swing_early > 1.0f,
                     "kicked weight swings out (peak |x| %.2f m, peak speed %.2f m/s)",
                     (double)s->swing_early, (double)s->peak_speed);
    ok &= demo_check(tag, s->swing_late > 0.2f && s->swing_late < s->swing_early,
                     "the pendulum keeps swinging with decaying amplitude (%.2f m -> %.2f m)",
                     (double)s->swing_early, (double)s->swing_late);
    /* hanging length: ceiling bottom (-14.7) + 8 links + the weight */
    float wp[3];
    demo_object_center(s->weight, wp);
    float chain = (float)CHAIN_LEN * 2.0f * LINK_HY + 2.0f * WEIGHT_HS;
    float hang = wp[1] - (CEILING_Y + 0.3f);
    ok &= demo_check(tag, hang > 0.85f * chain && hang < 1.05f * chain && wp[1] < 1.5f - WEIGHT_HS,
                     "weight hangs %.2f m below the ceiling (chain + weight %.2f m), above the ground",
                     (double)hang, (double)chain);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title        = "IVP Rope -- Chain Pendulum";
    app.cfg.tag          = "rope";
    app.cfg.orbit_dist   = 22.0f;
    app.cfg.orbit_yaw    = 25.0f;
    app.cfg.orbit_pitch  = -15.0f;
    app.cfg.target[1]    = -8.0f;
    app.cfg.grid_extent  = -1.0f;          /* drawn by the example (wire view) */
    /* the window loop steps the PSI length: avoids jitter from a dt mismatch */
    app.cfg.sim_dt       = ROPE_SIM_DT;
    app.cfg.max_substeps = 64;

    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ecfg.delta_psi_time = (ivp_real_t)ROPE_SIM_DT;
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    rope_demo_t *s = (rope_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    /* ── Ground plane ───────────────────────────────────────────────────── */
    ivp_object_template_t grt;
    ivp_object_template_init(&grt);
    grt.mass = 0.0f;
    grt.physical_unmoveable = true;
    grt.position = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    s->ground = demo_create_box_templ(w, &grt, 12.0, 0.5, 12.0);

    /* ── Ceiling bar (static, wider) ────────────────────────────────────── */
    ivp_object_template_t ct;
    ivp_object_template_init(&ct);
    ct.mass = 0.0f;
    ct.physical_unmoveable = true;
    ct.position = ivp_vec3r(IVP_REAL(0.0), (ivp_real_t)CEILING_Y, IVP_REAL(0.0));
    ct.nocoll_group_ident = ROPE_GROUP;
    s->ceiling = demo_create_box_templ(w, &ct, 1.5, 0.3, 0.3);

    /* ── Chain links ────────────────────────────────────────────────────── *
     * The full chain starts at one lateral offset so that the constraints
     * start near rest. */
    ivp_real_t step = 2.0f * LINK_HY + LINK_GAP;
    ivp_real_t first_link_y = (ivp_real_t)CEILING_Y + IVP_REAL(0.3) +
                              (ivp_real_t)LINK_HY + (ivp_real_t)LINK_GAP;
    for (int i = 0; i < CHAIN_LEN; i++) {
        ivp_object_template_t lt;
        ivp_object_template_init(&lt);
        lt.mass = IVP_REAL(0.8);
        set_box_inertia(&lt, IVP_REAL(0.8), (ivp_real_t)LINK_HX, (ivp_real_t)LINK_HY, (ivp_real_t)LINK_HZ);
        lt.material.friction   = IVP_REAL(0.6);
        lt.material.elasticity = IVP_REAL(0.0);
        lt.speed_damp_factor     = IVP_REAL(0.14);
        lt.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.18), IVP_REAL(0.18), IVP_REAL(0.18));
        lt.nocoll_group_ident = ROPE_GROUP;
        lt.position = ivp_vec3r((ivp_real_t)INITIAL_SWING_X,
                                first_link_y + step * (ivp_real_t)i,
                                IVP_REAL(0.0));
        s->links[i] = demo_create_box_templ(w, &lt, LINK_HX, LINK_HY, LINK_HZ);
    }

    /* ── Weight at chain end ────────────────────────────────────────────── */
    ivp_object_template_t wt;
    ivp_object_template_init(&wt);
    wt.mass = IVP_REAL(1.4);
    set_box_inertia(&wt, IVP_REAL(1.4), WEIGHT_HS, WEIGHT_HS, WEIGHT_HS);
    wt.material.friction   = IVP_REAL(0.8);
    wt.material.elasticity = IVP_REAL(0.0);
    wt.speed_damp_factor     = IVP_REAL(0.12);
    wt.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.15), IVP_REAL(0.15), IVP_REAL(0.15));
    wt.nocoll_group_ident = ROPE_GROUP;
    wt.position = ivp_vec3r((ivp_real_t)INITIAL_SWING_X,
                            first_link_y + step * (ivp_real_t)(CHAIN_LEN - 1) +
                                (ivp_real_t)LINK_HY + (ivp_real_t)WEIGHT_HS +
                                (ivp_real_t)LINK_GAP,
                            IVP_REAL(0.0));
    s->weight = demo_create_box_templ(w, &wt, WEIGHT_HS, WEIGHT_HS, WEIGHT_HS);

    /* ── Constraints ────────────────────────────────────────────────────── */
    memset(s->constraints, 0, sizeof(s->constraints));
    {
        /* Ceiling -> first link */
        ivp_vec3r_t a_pt = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.3), IVP_REAL(0.0));
        ivp_vec3r_t b_pt = ivp_vec3r(IVP_REAL(0.0), -LINK_HY, IVP_REAL(0.0));
        ivp_constraint_init_ball_socket(&s->constraints[0], s->ceiling, &a_pt, s->links[0], &b_pt);
    }
    for (int i = 1; i < CHAIN_LEN; i++) {
        /* Link-to-link */
        ivp_vec3r_t a_bot = ivp_vec3r(IVP_REAL(0.0), LINK_HY, IVP_REAL(0.0));
        ivp_vec3r_t b_top = ivp_vec3r(IVP_REAL(0.0), -LINK_HY, IVP_REAL(0.0));
        ivp_constraint_init_ball_socket(&s->constraints[i], s->links[i - 1], &a_bot, s->links[i], &b_top);
    }
    {
        /* Last link -> weight */
        ivp_vec3r_t a_bot = ivp_vec3r(IVP_REAL(0.0), LINK_HY, IVP_REAL(0.0));
        ivp_vec3r_t b_top = ivp_vec3r(IVP_REAL(0.0), -WEIGHT_HS, IVP_REAL(0.0));
        ivp_constraint_init_ball_socket(&s->constraints[CHAIN_LEN], s->links[CHAIN_LEN - 1], &a_bot, s->weight, &b_top);
    }
    for (int i = 0; i <= CHAIN_LEN; i++)
        ivp_environment_add_controller(w->env, IVP_CP_CONSTRAINTS, ivp_controller_constraint, &s->constraints[i]);

    app.controls    = "K: kick the weight sideways";
    app.update      = rope_update;
    app.on_reset    = rope_reset;
    app.draw        = rope_draw;
    app.stats       = rope_stats;
    app.check       = rope_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
