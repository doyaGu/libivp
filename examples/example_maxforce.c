/* example_maxforce.c -- Constraints with a maximum impulse
 *
 * Port of ivp_examples/quickstart/ive_quickstart_maxforce.cxx
 * (IVP manual section 5.17, Constraints with a maximum force).
 *
 * Two pairs of cubes; in each pair a dynamic cube is attached 2 m above a
 * static one with a ball-socket constraint anchored at the static cube's
 * centre (it balances upright on the joint until it is kicked).
 * Both constraints limit all six axes to a maximum impulse of 4 N*s:
 *   left  (orange): IVP_CFE_BREAK -- the constraint breaks (is deactivated
 *                   for good) when the impulse exceeds the threshold,
 *   right (blue):   IVP_CFE_CLIP  -- the impulse is clipped: the joint gives
 *                   way temporarily but keeps pulling the cube back.
 * Press K to kick both hanging cubes upward (12 m/s) so that the
 * constraints exceed the limit.  (The reference uses SPACE, which resets
 * the scene in the libivp examples; the scene reset re-arms a broken
 * constraint.)
 *
 * Demonstrates: ivp_constraint_template_t maximpulse / maximpulse_type
 *               (IVP_Template_Constraint::set_max_translation_impulse /
 *               set_max_rotation_impulse), IVP_CFE_BREAK / IVP_CFE_CLIP,
 *               the IVP_EVENT_CONSTRAINT_BROKEN event.
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_constraint.h>

static const double CUBE_HS = 0.35;
static const float  MAX_IMPULSE = 4.0f;   /* N*s, all six axes */

typedef struct maxforce_demo {
    ivp_object_t             *ground;
    ivp_object_t             *break_ref, *break_dyn;
    ivp_object_t             *clip_ref, *clip_dyn;
    ivp_constraint_template_t break_tmpl, clip_tmpl;
    ivp_constraint_state_t    break_joint, clip_joint;
    int                       n_broken_events;
    int                       n_kicks;
    float                     clip_max_stretch;   /* max anchor distance since the last kick */
    float                     break_max_gap;      /* same for the BREAK constraint */
    bool                      clip_flew_out;      /* the CLIP cube left the joint after the kick */
    float                     clip_return_min;    /* closest anchor distance since then */
} maxforce_demo_t;

/* the cubes start 2 m from their anchors */
static const float JOINT_LENGTH = 2.0f;

static void make_joint(ivp_constraint_template_t *t, ivp_object_t *ref, ivp_object_t *dyn,
                       ivp_constraint_force_exceed_t type)
{
    ivp_vec3r_t anchor = ref->physical_core->pos_world_f_core_last_psi;
    ivp_constraint_template_init(t);
    ivp_constraint_template_set_ballsocket_ws(t, ref, &anchor, dyn);
    /* set_max_translation_impulse + set_max_rotation_impulse: all six axes
     * must use the same value and type */
    for (int i = 0; i < 6; i++) {
        t->maximpulse[i] = (ivp_real_t)MAX_IMPULSE;
        t->maximpulse_type[i] = type;
    }
}

static void arm_joints(maxforce_demo_t *s)
{
    ivp_constraint_init_from_template(&s->break_joint, &s->break_tmpl);
    ivp_constraint_init_from_template(&s->clip_joint, &s->clip_tmpl);
}

static void on_event(void *ctx, ivp_environment_t *env, const ivp_event_t *ev)
{
    maxforce_demo_t *s = (maxforce_demo_t *)ctx;
    (void)env;
    if (ev->type == IVP_EVENT_CONSTRAINT_BROKEN) {
        s->n_broken_events++;
        printf("[maxforce] constraint broken event at t=%.3f\n", (double)ev->time);
    }
}

static float anchor_distance(const ivp_constraint_state_t *c)
{
    float pa[3], pb[3];
    demo_core_point_ws(c->obj_a, &c->anchor_a_os, pa);
    demo_core_point_ws(c->obj_b, &c->anchor_b_os, pb);
    return demo_distance(pa, pb);
}

static void kick(demo_app_t *app)
{
    maxforce_demo_t *s = (maxforce_demo_t *)app->user;
    demo_async_add_speed_ws(app->world.env, s->break_dyn, 0.0, -12.0, 0.0);
    demo_async_add_speed_ws(app->world.env, s->clip_dyn, 0.0, -12.0, 0.0);
    s->n_kicks++;
    s->clip_max_stretch = 0.0f;
    s->break_max_gap = 0.0f;
    s->clip_flew_out = false;
    s->clip_return_min = 1e9f;
}

static void maxforce_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    maxforce_demo_t *s = (maxforce_demo_t *)app->user;
    float d = anchor_distance(&s->clip_joint);
    if (d > s->clip_max_stretch) s->clip_max_stretch = d;
    if (s->n_kicks > 0 && d > JOINT_LENGTH + 1.0f) s->clip_flew_out = true;
    if (s->clip_flew_out && d < s->clip_return_min) s->clip_return_min = d;
    d = anchor_distance(&s->break_joint);
    if (d > s->break_max_gap) s->break_max_gap = d;
    if (v) {
        if (demo_key_just_pressed(v, SDL_SCANCODE_K)) kick(app);
        return;
    }
    if (frame == 120) kick(app);
}

static void maxforce_reset(demo_app_t *app)
{
    maxforce_demo_t *s = (maxforce_demo_t *)app->user;
    /* poses are restored: re-arm both constraints (relaxed at the start pose) */
    arm_joints(s);
    s->clip_max_stretch = 0.0f;
    s->break_max_gap = 0.0f;
    s->clip_flew_out = false;
    s->clip_return_min = 1e9f;
}

static void maxforce_draw(demo_app_t *app, demo_viewer_t *v)
{
    maxforce_demo_t *s = (maxforce_demo_t *)app->user;
    demo_draw_object(v, s->ground, DEMO_COL_STATIC);
    demo_draw_object(v, s->break_ref, DEMO_COL_STATIC);
    demo_draw_object(v, s->break_dyn, DEMO_COL_OBJECT_B);
    demo_draw_object(v, s->clip_ref, DEMO_COL_STATIC);
    demo_draw_object(v, s->clip_dyn, DEMO_COL_OBJECT_A);

    if (s->break_joint.active)
        demo_draw_link(v, s->break_ref, s->break_dyn, DEMO_COL_OBJECT_B);
    demo_draw_link(v, s->clip_ref, s->clip_dyn, DEMO_COL_OBJECT_A);

    demo_hud(v, DEMO_COL_INFO, "Left  (orange): IVP_CFE_BREAK -- %s",
             s->break_joint.is_broken ? "BROKEN" : "intact");
    demo_hud(v, DEMO_COL_INFO, "  constraint deactivates itself at the limit");
    demo_hud(v, DEMO_COL_INFO, "Right (blue):   IVP_CFE_CLIP -- anchor gap %.3f m (max %.3f)",
             (double)anchor_distance(&s->clip_joint), (double)s->clip_max_stretch);
    demo_hud(v, DEMO_COL_INFO, "  constraint gives way, recovers");
    demo_hud(v, DEMO_COL_INFO, "maximpulse = %.1f N*s (both pairs)", (double)MAX_IMPULSE);
}

static void maxforce_stats(demo_app_t *app, char *buf, size_t size)
{
    maxforce_demo_t *s = (maxforce_demo_t *)app->user;
    snprintf(buf, size, "kicks=%d break:%s gap=%.3f dist=%.3f events=%d | clip:%s gap=%.3f dist=%.3f max_gap=%.3f",
             s->n_kicks, s->break_joint.is_broken ? "broken" : "intact",
             (double)anchor_distance(&s->break_joint), (double)demo_object_distance(s->break_ref, s->break_dyn),
             s->n_broken_events, s->clip_joint.is_broken ? "broken" : "intact",
             (double)anchor_distance(&s->clip_joint), (double)demo_object_distance(s->clip_ref, s->clip_dyn),
             (double)s->clip_max_stretch);
}

static bool maxforce_check(demo_app_t *app)
{
    maxforce_demo_t *s = (maxforce_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, s->break_joint.is_broken && !s->break_joint.active,
                     "BREAK constraint broke after the kick");
    ok &= demo_check(tag, s->n_broken_events == 1, "one constraint-broken event (%d)", s->n_broken_events);
    ok &= demo_check(tag, s->break_max_gap > 1.0f,
                     "released cube flew off the joint (max anchor gap %.3f m)", (double)s->break_max_gap);
    ok &= demo_check(tag, !s->clip_joint.is_broken && s->clip_joint.active,
                     "CLIP constraint did not break");
    /* the reference CLIP is only partially implemented (manual 5.17): while
     * the limit is exceeded the impulse is scaled by
     * d_time * halfimpulse / |impulse|, which practically switches the joint
     * off, so the clipped cube can fly as far as the released one (same with
     * the legacy engine).  It stays attached, though: once the cube has
     * flown out, the joint pulls it back to its anchor.  The cube keeps
     * swinging afterwards, so its distance at the end of the run says
     * nothing. */
    ok &= demo_check(tag, s->clip_flew_out && s->clip_return_min < JOINT_LENGTH + 0.5f,
                     "CLIP pulled its cube back after it flew out (closest %.3f m, joint %.1f m, max %.3f m)",
                     (double)s->clip_return_min, (double)JOINT_LENGTH, (double)s->clip_max_stretch);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Quickstart: Maxforce Constraints (Sec 5.17)";
    app.cfg.tag         = "maxforce";
    app.cfg.orbit_dist  = 20.0f;
    app.cfg.orbit_pitch = -20.0f;
    app.cfg.target[1]   = -3.0f;
    app.cfg.friction    = 0.5;
    app.cfg.elasticity  = 0.3;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    maxforce_demo_t *s = (maxforce_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    s->ground = demo_create_box(w, NULL, 12.0, 0.5, 12.0, 0.0, NULL, 0.0, 0.5, 0.0);

    /* Left pair: IVP_CFE_BREAK */
    s->break_ref = demo_create_box(w, NULL, CUBE_HS, CUBE_HS, CUBE_HS, 0.0, NULL, -2.5, -2.0, 0.0);
    s->break_dyn = demo_create_box(w, NULL, CUBE_HS, CUBE_HS, CUBE_HS, 1.0, NULL, -2.5, -4.0, 0.0);
    make_joint(&s->break_tmpl, s->break_ref, s->break_dyn, IVP_CFE_BREAK);

    /* Right pair: IVP_CFE_CLIP */
    s->clip_ref = demo_create_box(w, NULL, CUBE_HS, CUBE_HS, CUBE_HS, 0.0, NULL, 2.5, -2.0, 0.0);
    s->clip_dyn = demo_create_box(w, NULL, CUBE_HS, CUBE_HS, CUBE_HS, 1.0, NULL, 2.5, -4.0, 0.0);
    make_joint(&s->clip_tmpl, s->clip_ref, s->clip_dyn, IVP_CFE_CLIP);

    arm_joints(s);
    ivp_environment_add_controller(w->env, IVP_CP_CONSTRAINTS, ivp_controller_constraint, &s->break_joint);
    ivp_environment_add_controller(w->env, IVP_CP_CONSTRAINTS, ivp_controller_constraint, &s->clip_joint);
    ivp_environment_add_event_listener(w->env, on_event, s);

    app.controls = "K: kick the attached cubes upward";
    app.update   = maxforce_update;
    app.on_reset = maxforce_reset;
    app.draw     = maxforce_draw;
    app.stats    = maxforce_stats;
    app.check    = maxforce_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
