/* example_cut.c -- Chopping a surface into pieces
 *
 * Port of ivp_examples/quickstart/ive_quickstart_cut.cxx
 * (IVP manual section 5.15, Cutting an object).
 *
 * A 1.2 m cube surface is chopped twice with IVP_Compact_Modify::chop: once
 * along +X and once along -X, each time removing a 0.2 m slice.  The two
 * resulting 1.0 x 1.2 x 1.2 pieces are created as separate objects and
 * dropped onto the ground next to each other.
 *
 * ivp_compact_modify_chop(alloc, surface, dir, depth) has the reference
 * IVP_Compact_Modify::chop semantics: dir points to the remaining body and a
 * slice of thickness depth is removed from the opposite extreme.
 *
 * Demonstrates: ivp_compact_modify_chop, objects from arbitrary compact
 *               surfaces (mass centre != object origin), wireframes drawn
 *               from the compact ledges.
 */
#include "demo_app.h"

#include <ivp/builder/ivp_compact_modify.h>

typedef struct cut_demo {
    ivp_object_t *ground;
    ivp_object_t *piece1;
    ivp_object_t *piece2;
    float         extent1[2];   /* piece x range (object space) */
    float         extent2[2];
} cut_demo_t;

/* x range of a single convex surface (object space). */
static void surface_x_range(const ivp_compact_surface_t *surf, float range[2])
{
    range[0] = 1e30f;
    range[1] = -1e30f;
    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surf);
    if (!ledge) return;
    const ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array(ledge);
    int n = ivp_ledge_get_n_points(ledge);
    for (int i = 0; i < n; i++) {
        if (pts[i].k[0] < range[0]) range[0] = pts[i].k[0];
        if (pts[i].k[0] > range[1]) range[1] = pts[i].k[0];
    }
}

static ivp_object_t *create_piece(demo_world_t *w, ivp_compact_surface_t *piece, double x)
{
    ivp_object_template_t t;
    demo_template_dynamic(&t, &w->mat, 1.0);
    /* the reference uses the box inertia of a 0.4 x 0.6 x 0.6 half box */
    demo_set_box_inertia(&t, 1.0, 0.4, 0.6, 0.6);
    demo_template_pose(&t, NULL, x, -5.0, 0.0);
    return demo_create_polygon(w, piece, &t);
}

static void cut_draw(demo_app_t *app, demo_viewer_t *v)
{
    cut_demo_t *s = (cut_demo_t *)app->user;
    demo_draw_object(v, s->ground, DEMO_COL_STATIC);
    demo_draw_object(v, s->piece1, DEMO_COL_OBJECT_B);
    demo_draw_object(v, s->piece2, DEMO_COL_OBJECT_A);
    /* object origin (the original cube centre) vs mass centre */
    float o[3], rot[9], c[3];
    demo_object_pose(s->piece1, o, rot);
    demo_object_center(s->piece1, c);
    demo_cross(v, o, 0.12f, DEMO_COL_HIGHLIGHT);
    demo_point(v, c, 6.0f, DEMO_COL_WARNING);
    demo_object_pose(s->piece2, o, rot);
    demo_object_center(s->piece2, c);
    demo_cross(v, o, 0.12f, DEMO_COL_HIGHLIGHT);
    demo_point(v, c, 6.0f, DEMO_COL_WARNING);

    demo_hud_object(v, "Piece 1 (orange)", s->piece1);
    demo_hud_object(v, "Piece 2 (blue)", s->piece2);
    demo_hud(v, DEMO_COL_INFO, "Original 1.2 m box chopped into two pieces");
    demo_hud(v, DEMO_COL_INFO, "piece 1 x: [%.2f, %.2f]   piece 2 x: [%.2f, %.2f]",
             (double)s->extent1[0], (double)s->extent1[1],
             (double)s->extent2[0], (double)s->extent2[1]);
    demo_hud(v, DEMO_COL_HELP, "cross = object origin, dot = mass centre");
}

static void cut_stats(demo_app_t *app, char *buf, size_t size)
{
    cut_demo_t *s = (cut_demo_t *)app->user;
    float p1[3], p2[3];
    demo_object_center(s->piece1, p1);
    demo_object_center(s->piece2, p2);
    snprintf(buf, size, "p1=(%.3f,%.3f,%.3f) p2=(%.3f,%.3f,%.3f) tilt1=%.1f tilt2=%.1f v1=%.3f v2=%.3f",
             (double)p1[0], (double)p1[1], (double)p1[2],
             (double)p2[0], (double)p2[1], (double)p2[2],
             (double)demo_object_tilt_deg(s->piece1), (double)demo_object_tilt_deg(s->piece2),
             (double)demo_object_speed(s->piece1), (double)demo_object_speed(s->piece2));
}

static bool cut_check(demo_app_t *app)
{
    cut_demo_t *s = (cut_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, fabsf(s->extent1[0] + 0.4f) < 1e-3f && fabsf(s->extent1[1] - 0.6f) < 1e-3f,
                     "piece 1 spans x [-0.4, 0.6] (got [%.3f, %.3f])",
                     (double)s->extent1[0], (double)s->extent1[1]);
    ok &= demo_check(tag, fabsf(s->extent2[0] + 0.6f) < 1e-3f && fabsf(s->extent2[1] - 0.4f) < 1e-3f,
                     "piece 2 spans x [-0.6, 0.4] (got [%.3f, %.3f])",
                     (double)s->extent2[0], (double)s->extent2[1]);
    /* resting on the ground: a 1.2 m tall piece (half 0.6) lies at y = -0.6,
     * one lying on its side (half 0.5) at y = -0.5 */
    float y1 = (float)s->piece1->physical_core->pos_world_f_core_last_psi.k[1];
    float y2 = (float)s->piece2->physical_core->pos_world_f_core_last_psi.k[1];
    ok &= demo_check(tag, y1 > -0.7f && y1 < -0.45f && y2 > -0.7f && y2 < -0.45f,
                     "both pieces rest on the ground (y = %.3f, %.3f)", (double)y1, (double)y2);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title      = "IVP Quickstart: Cut";
    app.cfg.tag        = "cut";
    app.cfg.orbit_dist = 15.0f;
    app.cfg.target[1]  = -3.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    cut_demo_t *s = (cut_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;

    s->ground = demo_create_box(&app.world, NULL, 15.0, 0.5, 15.0, 0.0, NULL, 0.0, 0.5, 0.0);

    /* Original box surface, chopped along +X and -X (remove 0.2 m) */
    ivp_compact_surface_t *original = ivp_build_box_surface(NULL, 0.6f, 0.6f, 0.6f);
    if (!original) return 1;
    const float dir1[3] = { 1.0f, 0.0f, 0.0f };
    const float dir2[3] = { -1.0f, 0.0f, 0.0f };
    ivp_compact_surface_t *piece1 = ivp_compact_modify_chop(NULL, original, dir1, 0.2f);
    ivp_compact_surface_t *piece2 = ivp_compact_modify_chop(NULL, original, dir2, 0.2f);
    if (!piece1 || !piece2) {
        fprintf(stderr, "[cut] chop failed (%p, %p)\n", (void *)piece1, (void *)piece2);
        return 1;
    }
    ivp_compact_surface_free(NULL, original);
    surface_x_range(piece1, s->extent1);
    surface_x_range(piece2, s->extent2);
    printf("[cut] piece 1: x [%.3f, %.3f], mass centre x %.3f\n", (double)s->extent1[0],
           (double)s->extent1[1], (double)piece1->mass_center[0]);
    printf("[cut] piece 2: x [%.3f, %.3f], mass centre x %.3f\n", (double)s->extent2[0],
           (double)s->extent2[1], (double)piece2->mass_center[0]);

    s->piece1 = create_piece(&app.world, piece1, -0.8);
    s->piece2 = create_piece(&app.world, piece2, 0.8);
    if (!s->piece1 || !s->piece2) return 1;

    app.draw  = cut_draw;
    app.stats = cut_stats;
    app.check = cut_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
