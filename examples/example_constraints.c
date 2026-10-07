/* example_constraints.c -- Local 6-DOF constraints and a keyframed arm
 *
 * Four stations, left to right:
 *
 *   slider   a crate on a rail: the local constraint fixes y, z and every
 *            rotation and limits x to +-2.5 m (a prismatic joint). A push
 *            every 1.6 s sends it into the stops.
 *   door     a door on a pillar: translation fixed at the hinge, rotation
 *            about y limited to +-1 rad, x and z fixed (a hinge with stops).
 *            It is kicked every 2.2 s.
 *   weld     a 20 kg weight welded under a beam (all six axes fixed) with an
 *            impulse limit in BREAK mode. At 3 s it gets a jerk; the weld
 *            breaks and the weight falls. Space repairs it.
 *   arm      a keyframed fixed constraint: the head follows a circle that is
 *            given in the space of a light cart on a slippery floor, so the cart is pushed around
 *            by the reaction.
 *
 * Demonstrates: ivp_constraint_local_* (fix / limit / free per axis,
 *               max impulse in BREAK mode, broken / reset), ivp_fixed_keyed_*
 *               (prime position and velocity in the reference space),
 *               ivp_environment_announce_dependent_controller.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_constraint_fixed_keyed.h>
#include <ivp/controllers/ivp_constraint_local.h>

#define SLIDE_LIMIT   2.5
#define DOOR_LIMIT    1.0
#define WELD_BREAK    5.0           /* impulse per PSI (the weight alone needs 20 * 9.81 / 60 = 3.3) */
#define X_SLIDER     (-12.0)
#define X_DOOR       (-4.0)
#define X_WELD       4.0
#define X_ARM        12.0

typedef struct constraints_demo {
    ivp_environment_t           *env;
    ivp_object_t                *ground;
    /* slider */
    ivp_object_t                *crate;
    ivp_constraint_local_state_t slider;
    float                        slide_min, slide_max;
    /* door */
    ivp_object_t                *pillar, *door;
    ivp_constraint_local_state_t hinge;
    float                        yaw_min, yaw_max;
    /* weld */
    ivp_object_t                *beam, *weight;
    ivp_constraint_local_state_t weld;
    bool                         weld_broke;
    double                       weld_broke_at;
    /* arm */
    ivp_object_t                *cart, *head;
    ivp_fixed_keyed_state_t      keyed;
    float                        arm_worst, cart_moved;
    /* panel */
    float                        arm_radius, arm_speed;
    bool                         show_limits;
} constraints_demo_t;

static double now(const constraints_demo_t *s) { return (double)ivp_environment_get_time(s->env); }

static void to_f3(const ivp_vec3r_t *v, float out[3])
{
    for (int k = 0; k < 3; k++) out[k] = (float)v->k[k];
}

/* yaw of the object's x axis in the xz plane */
static float yaw_of(const ivp_object_t *obj)
{
    ivp_mat3r_t m;
    ivp_quat_to_mat3(&obj->physical_core->q_world_f_core_last_psi, &m);
    return (float)atan2(m.rows[2].k[0], m.rows[0].k[0]);
}

/* ── setup ───────────────────────────────────────────────────────────────── */

static void add_constraint(constraints_demo_t *s, ivp_constraint_local_state_t *c)
{
    ivp_environment_announce_dependent_controller(s->env, IVP_CP_CONSTRAINTS, ivp_controller_constraint_local, c,
                                                  ivp_constraint_local_discovery);
}

static void setup_slider(constraints_demo_t *s, demo_world_t *w)
{
    s->crate = demo_create_box(w, NULL, 0.6, 0.6, 0.6, 4.0, NULL, X_SLIDER, -3.0, 0.0);
    /* the reference is the world: anchor a is a world point */
    const ivp_vec3r_t a = ivp_vec3r(IVP_REAL(X_SLIDER), IVP_REAL(-3.0), IVP_REAL(0.0)), b = ivp_vec3r_zero();
    ivp_constraint_local_init(&s->slider, NULL, &a, s->crate, &b);
    ivp_constraint_local_limit_translation(&s->slider, 0, IVP_REAL(-SLIDE_LIMIT), IVP_REAL(SLIDE_LIMIT));
    ivp_constraint_local_fix_translation(&s->slider, 1);
    ivp_constraint_local_fix_translation(&s->slider, 2);
    for (int k = 0; k < 3; k++) ivp_constraint_local_fix_rotation(&s->slider, k);
    add_constraint(s, &s->slider);
}

static void setup_door(constraints_demo_t *s, demo_world_t *w)
{
    s->pillar = demo_create_box(w, NULL, 0.15, 2.0, 0.15, 0.0, NULL, X_DOOR, -2.0, 0.0);
    s->door = demo_create_box(w, NULL, 1.0, 1.4, 0.08, 3.0, NULL, X_DOOR + 1.4, -2.0, 0.0);
    const ivp_vec3r_t a = ivp_vec3r_zero(), b = ivp_vec3r(IVP_REAL(-1.4), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_constraint_local_init(&s->hinge, s->pillar, &a, s->door, &b);
    ivp_constraint_local_fix_rotation(&s->hinge, 0);
    ivp_constraint_local_fix_rotation(&s->hinge, 2);
    ivp_constraint_local_limit_rotation(&s->hinge, 1, IVP_REAL(-DOOR_LIMIT), IVP_REAL(DOOR_LIMIT));
    add_constraint(s, &s->hinge);
}

static void setup_weld(constraints_demo_t *s, demo_world_t *w)
{
    s->beam = demo_create_box(w, NULL, 1.4, 0.2, 0.4, 0.0, NULL, X_WELD, -6.0, 0.0);
    s->weight = demo_create_box(w, NULL, 0.5, 0.5, 0.5, 20.0, NULL, X_WELD, -5.0, 0.0);
    /* both anchors at the top face of the weight: (4, -5.5, 0) */
    const ivp_vec3r_t a = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.5), IVP_REAL(0.0)),
                      b = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-0.5), IVP_REAL(0.0));
    ivp_constraint_local_init(&s->weld, s->beam, &a, s->weight, &b);
    ivp_constraint_local_set_max_impulse(&s->weld, 1, IVP_REAL(WELD_BREAK), IVP_CFORCE_BREAK);
    add_constraint(s, &s->weld);
}

static void setup_arm(constraints_demo_t *s, demo_world_t *w)
{
    const ivp_material_t slick = demo_material(0.15, 0.2);
    s->cart = demo_create_box(w, &slick, 1.4, 0.3, 1.0, 8.0, NULL, X_ARM, -0.3, 0.0);
    s->head = demo_create_ball(w, NULL, 0.35, 2.0, NULL, X_ARM, -3.0, 0.0);
    ivp_fixed_keyed_init(&s->keyed, s->cart, s->head, NULL);
    ivp_environment_announce_dependent_controller(s->env, IVP_CP_CONSTRAINTS, ivp_controller_fixed_keyed, &s->keyed,
                                                  ivp_fixed_keyed_discovery);
}

/* ── per frame ───────────────────────────────────────────────────────────── */

static void arm_target(const constraints_demo_t *s, double t, ivp_vec3r_t *p, ivp_vec3r_t *v)
{
    /* a circle in the xz plane of the cart, 2.7 m above its centre, starting at the head */
    const double r = (double)s->arm_radius, w = (double)s->arm_speed;
    *p = ivp_vec3r((ivp_real_t)(r * sin(w * t)), IVP_REAL(-2.7), (ivp_real_t)(r * (cos(w * t) - 1.0)));
    *v = ivp_vec3r((ivp_real_t)(r * w * cos(w * t)), IVP_REAL(0.0), (ivp_real_t)(-r * w * sin(w * t)));
}

static void constraints_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    constraints_demo_t *s = (constraints_demo_t *)app->user;
    (void)v;
    (void)frame;
    const double t = now(s);

    /* the slider and the door get a kick at fixed times */
    if (t > 0.5 && fmod(t - 0.5, 1.6) < DEMO_PSI_TIME) {
        const int side = ((int)((t - 0.5) / 1.6) & 1) ? -1 : 1;
        demo_async_add_speed_ws(s->env, s->crate, 7.0 * side, 0.0, 0.0);
    }
    if (t > 1.0 && fmod(t - 1.0, 2.2) < DEMO_PSI_TIME) {
        const int side = ((int)((t - 1.0) / 2.2) & 1) ? -1 : 1;
        demo_async_add_rot_speed_cs(s->env, s->door, 0.0, 4.0 * side, 0.0);
    }
    /* the jerk that breaks the weld */
    if (!s->weld_broke && t > 3.0 && t < 3.0 + DEMO_PSI_TIME * 1.5)
        demo_async_add_speed_ws(s->env, s->weight, 0.0, 6.0, 0.0);
    if (!s->weld_broke && ivp_constraint_local_is_broken(&s->weld)) {
        s->weld_broke = true;
        s->weld_broke_at = t;
    }

    /* the arm target, given in the cart's space */
    ivp_vec3r_t p, vel;
    arm_target(s, t, &p, &vel);
    ivp_fixed_keyed_set_prime_position_Ros(&s->keyed, &p, &vel, ivp_environment_get_time(s->env));

    /* measurements */
    float c[3];
    demo_object_center(s->crate, c);
    const float sx = c[0] - (float)X_SLIDER;
    if (sx < s->slide_min) s->slide_min = sx;
    if (sx > s->slide_max) s->slide_max = sx;
    const float yaw = yaw_of(s->door);
    if (yaw < s->yaw_min) s->yaw_min = yaw;
    if (yaw > s->yaw_max) s->yaw_max = yaw;

    float hc[3], cc[3];
    demo_object_center(s->head, hc);
    demo_object_center(s->cart, cc);
    /* the head's place the target asks for, in world space (the cart barely rotates) */
    const float ex = hc[0] - cc[0] - (float)p.k[0], ey = hc[1] - cc[1] - (float)p.k[1],
                ez = hc[2] - cc[2] - (float)p.k[2];
    const float err = sqrtf(ex * ex + ey * ey + ez * ez);
    if (t > 1.0 && err > s->arm_worst) s->arm_worst = err;
    s->cart_moved = fabsf(cc[0] - (float)X_ARM) + fabsf(cc[2]);
}

static void constraints_reset(demo_app_t *app)
{
    constraints_demo_t *s = (constraints_demo_t *)app->user;
    s->slide_min = s->slide_max = 0.0f;
    s->yaw_min = s->yaw_max = 0.0f;
    s->weld_broke = false;
    s->arm_worst = 0.0f;
    ivp_constraint_local_reset_broken(&s->weld, true);
    ivp_constraint_local_rebind_reference(&s->weld);
    ivp_environment_ensure_core_in_simulation(s->env, s->weight);
}

static void constraints_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    constraints_demo_t *s = (constraints_demo_t *)app->user;
    demo_param_float(v, "arm radius", &s->arm_radius, 0.0f, 1.5f);
    demo_param_float(v, "arm speed", &s->arm_speed, 0.0f, 4.0f);
    demo_param_bool(v, "show limits", &s->show_limits);
}

/* ── drawing ─────────────────────────────────────────────────────────────── */

static void constraints_draw(demo_app_t *app, demo_viewer_t *v)
{
    constraints_demo_t *s = (constraints_demo_t *)app->user;
    const float c_ground[3] = { 0.50f, 0.53f, 0.57f }, c_fixed[3] = { 0.45f, 0.48f, 0.55f },
                c_a[3] = { 0.95f, 0.65f, 0.30f }, c_b[3] = { 0.40f, 0.78f, 0.95f }, c_c[3] = { 0.55f, 0.90f, 0.50f },
                c_broken[3] = { 0.95f, 0.30f, 0.30f }, c_lim[3] = { 1.0f, 0.9f, 0.3f };
    demo_draw_object(v, s->ground, c_ground);
    demo_draw_object(v, s->crate, c_a);
    demo_draw_object(v, s->pillar, c_fixed);
    demo_draw_object(v, s->door, c_b);
    demo_draw_object(v, s->beam, c_fixed);
    demo_draw_object(v, s->weight, ivp_constraint_local_is_broken(&s->weld) ? c_broken : c_c);
    demo_draw_object(v, s->cart, c_fixed);
    demo_draw_object(v, s->head, c_a);

    if (s->show_limits) {
        /* the rail and its stops */
        const float r0[3] = { (float)(X_SLIDER - SLIDE_LIMIT - 0.6), -3.0f, 0.0f },
                    r1[3] = { (float)(X_SLIDER + SLIDE_LIMIT + 0.6), -3.0f, 0.0f };
        demo_line(v, r0, r1, c_lim);
        for (int e = -1; e <= 1; e += 2) {
            const float a[3] = { (float)(X_SLIDER + e * (SLIDE_LIMIT + 0.6)), -3.6f, 0.0f },
                        b[3] = { a[0], -2.4f, 0.0f };
            demo_line(v, a, b, c_lim);
        }
        /* the door's swing range: two lines from the hinge */
        for (int e = -1; e <= 1; e += 2) {
            const float h[3] = { (float)X_DOOR, -2.0f, 0.0f },
                        tip[3] = { (float)(X_DOOR + 1.4 * cos(DOOR_LIMIT)), -2.0f,
                                   (float)(e * 1.4 * sin(DOOR_LIMIT)) };
            demo_line(v, h, tip, c_lim);
        }
        /* the arm's target */
        ivp_vec3r_t p, vel;
        arm_target(s, now(s), &p, &vel);
        float cc[3];
        demo_object_center(s->cart, cc);
        const float tp[3] = { cc[0] + (float)p.k[0], cc[1] + (float)p.k[1], cc[2] + (float)p.k[2] };
        demo_cross(v, tp, 0.3f, c_lim);
    }

    /* the load on the weld against its limit: a bar beside the weight that grows and goes red towards the break */
    {
        float wc2[3];
        demo_object_center(s->weight, wc2);
        const float load = fminf(1.2f, (float)s->weld.accumulated_impulse[1] / (float)WELD_BREAK);
        const float a[3] = { wc2[0] + 1.0f, wc2[1] + 0.5f, wc2[2] }, b[3] = { wc2[0] + 1.0f, wc2[1] + 0.5f - 1.6f * load, wc2[2] };
        const float col[3] = { 0.3f + 0.7f * load, 0.9f - 0.7f * load, 0.3f };
        if (!ivp_constraint_local_is_broken(&s->weld)) demo_thick_line(v, a, b, col, 0.12f);
    }
    float wc[3];
    demo_object_center(s->weight, wc);
    demo_hud(v, DEMO_COL_INFO, "slider  x range %+.2f .. %+.2f m (stops at +-%.1f)", (double)s->slide_min,
             (double)s->slide_max, SLIDE_LIMIT);
    demo_hud(v, DEMO_COL_INFO, "door    yaw range %+.2f .. %+.2f rad (stops at +-%.1f)", (double)s->yaw_min,
             (double)s->yaw_max, DOOR_LIMIT);
    demo_hud(v, ivp_constraint_local_is_broken(&s->weld) ? DEMO_COL_WARNING : DEMO_COL_INFO,
             "weld    %s  (limit %.0f, weight alone %.1f)",
             ivp_constraint_local_is_broken(&s->weld) ? "BROKEN" : "holding", WELD_BREAK, 20.0 * DEMO_GRAVITY / 60.0);
    demo_hud(v, DEMO_COL_INFO, "arm     worst error %.2f m, cart pushed %.2f m", (double)s->arm_worst,
             (double)s->cart_moved);
}

static void constraints_stats(demo_app_t *app, char *buf, size_t size)
{
    constraints_demo_t *s = (constraints_demo_t *)app->user;
    snprintf(buf, size, "slide [%.2f %.2f] yaw [%.2f %.2f] weld %s arm err %.2f", (double)s->slide_min,
             (double)s->slide_max, (double)s->yaw_min, (double)s->yaw_max,
             s->weld_broke ? "broken" : "holds", (double)s->arm_worst);
}

static bool constraints_check(demo_app_t *app)
{
    constraints_demo_t *s = (constraints_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, s->slide_max > 1.5f && s->slide_min < -1.5f,
                     "the crate slides along the rail (%.2f .. %.2f m)", (double)s->slide_min, (double)s->slide_max);
    ok &= demo_check(tag, s->slide_max < SLIDE_LIMIT + 0.4 && s->slide_min > -SLIDE_LIMIT - 0.4,
                     "and the stops hold (%.2f .. %.2f m)", (double)s->slide_min, (double)s->slide_max);
    ok &= demo_check(tag, s->yaw_max > 0.5f && s->yaw_min < -0.5f, "the door swings (%.2f .. %.2f rad)",
                     (double)s->yaw_min, (double)s->yaw_max);
    ok &= demo_check(tag, s->yaw_max < DOOR_LIMIT + 0.3 && s->yaw_min > -DOOR_LIMIT - 0.3,
                     "within its stops (%.2f .. %.2f rad)", (double)s->yaw_min, (double)s->yaw_max);
    float wc[3];
    demo_object_center(s->weight, wc);
    ok &= demo_check(tag, s->weld_broke && wc[1] > -3.0f,
                     "the weld broke under the jerk at %.2f s and the weight fell to y %.2f", s->weld_broke_at,
                     (double)wc[1]);
    ok &= demo_check(tag, s->arm_worst < 0.6f, "the keyed head follows its target (worst %.2f m)", (double)s->arm_worst);
    ok &= demo_check(tag, s->cart_moved > 0.15f, "and the reaction moves the cart (%.2f m)", (double)s->cart_moved);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Constraints -- Local and Keyframed";
    app.cfg.tag         = "constraints";
    app.cfg.friction    = 0.6;
    app.cfg.elasticity  = 0.2;
    app.cfg.orbit_dist  = 21.0f;
    app.cfg.orbit_yaw   = 180.0f;
    app.cfg.orbit_pitch = -14.0f;
    app.cfg.target[1]   = -2.5f;
    app.cfg.grid_extent = -1.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    constraints_demo_t *s = (constraints_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->env = w->env;
    s->arm_radius = 1.0f;
    s->arm_speed = 2.0f;
    s->show_limits = true;

    s->ground = demo_create_box(w, NULL, 17.0, 0.5, 4.0, 0.0, NULL, 0.0, 0.5, 0.0);
    setup_slider(s, w);
    setup_door(s, w);
    setup_weld(s, w);
    setup_arm(s, w);

    app.controls    = "Space: repair the weld   F5: arm circle and the limit markers";
    app.viewer_init = constraints_viewer_init;
    app.update      = constraints_update;
    app.on_reset    = constraints_reset;
    app.draw        = constraints_draw;
    app.stats       = constraints_stats;
    app.check       = constraints_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
