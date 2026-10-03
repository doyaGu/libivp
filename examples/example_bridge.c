/* example_bridge.c -- Bridge of constraint-connected planks
 *
 * Port of ivp_examples/examples/ive_example_bridge.cxx.
 *
 * Two static pillars carry a chain of ten thin planks.  The first plank is
 * tied to the left pillar, neighbouring planks are tied to each other and
 * the last plank to the right pillar, all with ball-socket constraints
 * anchored at world points between the parts.  A heavy box (5 kg) is
 * dropped onto the middle of the bridge.
 *
 * As in the reference, the chain is taut (the joint-to-joint distances add
 * up to the span) and the ball sockets lie on the bridge axis: the soft
 * constraints let the bridge sag under its own weight, the box makes it
 * bounce, and the planks are free to roll about the chain axis, so the box
 * finally tips the planks over and slides off.  The headless run matches
 * the legacy C++ engine running the same scene (identical for the first
 * three seconds, the final resting place of the box differs by ~0.15 m).
 *
 * Demonstrates: ball-socket constraints given in world space
 *               (ivp_constraint_init_ball_socket_ws, reference
 *               IVP_Template_Constraint::set_ballsocket_ws), constraints
 *               between static and dynamic objects, constraint chains.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_constraint.h>

#define NUM_PLANKS 10
#define NUM_JOINTS (NUM_PLANKS + 1)

static const double PILLAR_HX = 0.5, PILLAR_HY = 1.5, PILLAR_HZ = 0.5;
static const double PLANK_HX = 0.4, PLANK_HY = 0.06, PLANK_HZ = 0.4;
static const double BOX_HS = 0.4;

typedef struct bridge {
    ivp_object_t          *ground;
    ivp_object_t          *left_pillar;
    ivp_object_t          *right_pillar;
    ivp_object_t          *planks[NUM_PLANKS];
    ivp_object_t          *heavy_box;
    ivp_constraint_state_t joints[NUM_JOINTS];
    double                 plank_y;
    float                  peak_sag;   /* since start / reset */
    float                  peak_gap;
} bridge_t;

/* Largest distance between the two anchor points of any joint. */
static float max_joint_gap(const bridge_t *s)
{
    float worst = 0.0f;
    for (int i = 0; i < NUM_JOINTS; i++) {
        const ivp_constraint_state_t *c = &s->joints[i];
        float pa[3], pb[3];
        demo_core_point_ws(c->obj_a, &c->anchor_a_os, pa);
        demo_core_point_ws(c->obj_b, &c->anchor_b_os, pb);
        float d = demo_distance(pa, pb);
        if (d > worst) worst = d;
    }
    return worst;
}

static float max_sag(const bridge_t *s)
{
    float worst = 0.0f;
    for (int i = 0; i < NUM_PLANKS; i++) {
        float d = (float)(s->planks[i]->physical_core->pos_world_f_core_last_psi.k[1] - s->plank_y);
        if (d > worst) worst = d;
    }
    return worst;
}

static void bridge_draw(demo_app_t *app, demo_viewer_t *v)
{
    bridge_t *s = (bridge_t *)app->user;
    demo_draw_object(v, s->ground, DEMO_COL_STATIC);
    demo_draw_object(v, s->left_pillar, DEMO_COL_STATIC);
    demo_draw_object(v, s->right_pillar, DEMO_COL_STATIC);
    for (int i = 0; i < NUM_PLANKS; i++)
        demo_draw_object(v, s->planks[i], DEMO_COL_OBJECT_B);

    /* connection lines between adjacent planks */
    for (int i = 0; i < NUM_PLANKS - 1; i++)
        demo_draw_link(v, s->planks[i], s->planks[i + 1], DEMO_COL_OBJECT_B);

    /* joint anchors */
    for (int i = 0; i < NUM_JOINTS; i++) {
        float p[3];
        demo_core_point_ws(s->joints[i].obj_b, &s->joints[i].anchor_b_os, p);
        demo_cross(v, p, 0.08f, DEMO_COL_HIGHLIGHT);
    }

    demo_draw_object(v, s->heavy_box, DEMO_COL_WARNING);

    demo_hud(v, DEMO_COL_INFO, "Planks: %d   ball-socket joints: %d", NUM_PLANKS, NUM_JOINTS);
    demo_hud(v, DEMO_COL_INFO, "Max sag: %.3f m   max joint gap: %.4f m",
             (double)max_sag(s), (double)max_joint_gap(s));
    demo_hud_object(v, "Heavy box", s->heavy_box);
}

static float max_plank_tilt(const bridge_t *s)
{
    float worst = 0.0f;
    for (int i = 0; i < NUM_PLANKS; i++) {
        float t = demo_object_tilt_deg(s->planks[i]);
        if (t > worst) worst = t;
    }
    return worst;
}

static void bridge_stats(demo_app_t *app, char *buf, size_t size)
{
    bridge_t *s = (bridge_t *)app->user;
    const ivp_vec3r_t *b = &s->heavy_box->physical_core->pos_world_f_core_last_psi;
    snprintf(buf, size, "box=(%.2f,%.3f,%.2f) box_speed=%.3f max_sag=%.3f max_plank_tilt=%.1f max_joint_gap=%.4f",
             (double)b->k[0], (double)b->k[1], (double)b->k[2],
             (double)demo_object_speed(s->heavy_box),
             (double)max_sag(s), (double)max_plank_tilt(s), (double)max_joint_gap(s));
}

static void bridge_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    bridge_t *s = (bridge_t *)app->user;
    (void)v;
    (void)frame;
    float sag = max_sag(s), gap = max_joint_gap(s);
    if (sag > s->peak_sag) s->peak_sag = sag;
    if (gap > s->peak_gap) s->peak_gap = gap;
}

static void bridge_reset(demo_app_t *app)
{
    bridge_t *s = (bridge_t *)app->user;
    s->peak_sag = s->peak_gap = 0.0f;
}

static bool bridge_check(demo_app_t *app)
{
    bridge_t *s = (bridge_t *)app->user;
    const char *tag = app->cfg.tag;
    const ivp_vec3r_t *b = &s->heavy_box->physical_core->pos_world_f_core_last_psi;
    bool ok = true;
    /* The joints are soft and the chain is taut: the reference engine shows
     * the same sag, bounce and roll (compared against the legacy C++ build). */
    ok &= demo_check(tag, s->peak_sag > 0.5f && s->peak_sag < 3.0f,
                     "bridge sagged under the load (peak sag %.3f m)", (double)s->peak_sag);
    ok &= demo_check(tag, s->peak_gap < 0.6f && max_joint_gap(s) < 0.1f,
                     "joints held together (peak gap %.3f m, final %.4f m)",
                     (double)s->peak_gap, (double)max_joint_gap(s));
    ok &= demo_check(tag, b->k[1] < 0.0 && b->k[1] > -6.5 && fabs((double)b->k[0]) < 6.0 &&
                          fabs((double)b->k[2]) < 5.0,
                     "heavy box stayed in the scene (%.2f, %.2f, %.2f)",
                     (double)b->k[0], (double)b->k[1], (double)b->k[2]);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Example: Bridge";
    app.cfg.tag         = "bridge";
    app.cfg.orbit_dist  = 20.0f;
    app.cfg.orbit_pitch = -20.0f;
    app.cfg.target[1]   = -1.0f;
    app.cfg.friction    = 0.6;
    app.cfg.elasticity  = 0.5;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    bridge_t *s = (bridge_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    /* Ground */
    s->ground = demo_create_box(w, NULL, 20.0, 0.5, 5.0, 0.0, NULL, 0.0, 0.5, 0.0);

    /* Pillars */
    s->left_pillar  = demo_create_box(w, NULL, PILLAR_HX, PILLAR_HY, PILLAR_HZ, 0.0, NULL,
                                      -5.0, -PILLAR_HY, 0.0);
    s->right_pillar = demo_create_box(w, NULL, PILLAR_HX, PILLAR_HY, PILLAR_HZ, 0.0, NULL,
                                      5.0, -PILLAR_HY, 0.0);

    /* Planks spanning between the pillars */
    const double start_x = -4.0, end_x = 4.0;
    const double spacing = (end_x - start_x) / (NUM_PLANKS - 1);
    s->plank_y = -(PILLAR_HY * 2.0) - PLANK_HY;
    for (int i = 0; i < NUM_PLANKS; i++) {
        s->planks[i] = demo_create_box(w, NULL, PLANK_HX, PLANK_HY, PLANK_HZ, 0.3, NULL,
                                       start_x + i * spacing, s->plank_y, 0.0);
    }

    /* First plank -> left pillar */
    {
        ivp_vec3r_t anchor = ivp_vec3r((ivp_real_t)(start_x - PLANK_HX), (ivp_real_t)s->plank_y, 0);
        ivp_constraint_init_ball_socket_ws(&s->joints[0], s->left_pillar, &anchor, s->planks[0]);
    }
    /* Adjacent planks */
    for (int i = 0; i < NUM_PLANKS - 1; i++) {
        ivp_vec3r_t anchor = ivp_vec3r((ivp_real_t)(start_x + (i + 0.5) * spacing),
                                       (ivp_real_t)s->plank_y, 0);
        ivp_constraint_init_ball_socket_ws(&s->joints[i + 1], s->planks[i], &anchor, s->planks[i + 1]);
    }
    /* Last plank -> right pillar */
    {
        ivp_vec3r_t anchor = ivp_vec3r((ivp_real_t)(end_x + PLANK_HX), (ivp_real_t)s->plank_y, 0);
        ivp_constraint_init_ball_socket_ws(&s->joints[NUM_JOINTS - 1], s->right_pillar, &anchor,
                                           s->planks[NUM_PLANKS - 1]);
    }
    /* IVP_Controller_Factory::create_constraint */
    for (int i = 0; i < NUM_JOINTS; i++)
        ivp_environment_add_controller(w->env, IVP_CP_CONSTRAINTS, ivp_controller_constraint, &s->joints[i]);

    /* Heavy box dropped onto the bridge */
    s->heavy_box = demo_create_box(w, NULL, BOX_HS, BOX_HS, BOX_HS, 5.0, NULL, 0.0, -6.0, 0.0);

    app.update   = bridge_update;
    app.on_reset = bridge_reset;
    app.draw     = bridge_draw;
    app.stats    = bridge_stats;
    app.check    = bridge_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
