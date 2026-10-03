/* example_ring.c -- A concave ring falling down
 *
 * Port of ivp_examples/quickstart/ive_quickstart_ring.cxx
 * (IVP manual section 5.3, Creating a concave ring).
 *
 * A square ring (6 x 1 x 6 m outside, 4 x 4 m hole) is built from four
 * convex box bars with the ledge soup builder (one compound, concave
 * surface with a convex hull around the whole object at the root node).
 * It is dropped tilted (rotation vector (-0.7, 0.1, 0.1)) onto the ground.
 * In addition to the reference a small ball is dropped through the hole
 * to show that the hole is really empty.
 *
 * Demonstrates: ivp_ledge_soup_t (IVP_SurfaceBuilder_Ledge_Soup) with
 *               build_root_convex_hull / IVP_SLMP_MERGE_AND_REALLOCATE,
 *               ivp_build_convex_ledge_d (convert_pointsoup_to_compact_ledge),
 *               concave objects, ivp_quat_set_fast_multiple
 *               (IVP_U_Quat(IVP_U_Point) rotation vector constructor).
 */
#include "demo_app.h"

#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/builder/ivp_build_ledge_soup.h>

typedef struct ring_demo {
    ivp_object_t *ground;
    ivp_object_t *ring;
    ivp_object_t *ball;
    int           n_ledges;
    float         ball_min_y;   /* highest point of the ball after passing the ring */
} ring_demo_t;

/* Four convex bars forming a square frame. */
static ivp_compact_surface_t *build_ring_surface(int *out_n_ledges)
{
    struct bar_def { double x0, x1, y0, y1, z0, z1; };
    static const struct bar_def bars[4] = {
        { -3.0, -2.0, -0.5, 0.5, -3.0,  3.0 },
        {  2.0,  3.0, -0.5, 0.5, -3.0,  3.0 },
        { -2.0,  2.0, -0.5, 0.5, -3.0, -2.0 },
        { -2.0,  2.0, -0.5, 0.5,  2.0,  3.0 }
    };

    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);
    *out_n_ledges = 0;
    ivp_compact_ledge_t *ledges[4] = { NULL, NULL, NULL, NULL };
    for (int b = 0; b < 4; b++) {
        double pts[8 * 3];
        int idx = 0;
        for (int sy = 0; sy <= 1; sy++)
            for (int sz = 0; sz <= 1; sz++)
                for (int sx = 0; sx <= 1; sx++) {
                    pts[idx * 3 + 0] = sx ? bars[b].x1 : bars[b].x0;
                    pts[idx * 3 + 1] = sy ? bars[b].y1 : bars[b].y0;
                    pts[idx * 3 + 2] = sz ? bars[b].z1 : bars[b].z0;
                    idx++;
                }
        ledges[b] = ivp_build_convex_ledge_d(NULL, pts, 8);
        if (ledges[b] && ivp_ledge_soup_add(&soup, ledges[b]) >= 0) (*out_n_ledges)++;
    }

    ivp_ledge_soup_template_t tls;
    ivp_ledge_soup_template_init(&tls);
    tls.build_root_convex_hull = true;
    tls.merge_points = IVP_SLMP_MERGE_AND_REALLOCATE;
    ivp_compact_surface_t *surface = ivp_ledge_soup_compile_ex(&soup, &tls);

    ivp_ledge_soup_destroy(&soup);
    for (int b = 0; b < 4; b++)
        if (ledges[b]) ivp_compact_ledge_free(NULL, ledges[b]);
    return surface;
}

static void ring_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    ring_demo_t *s = (ring_demo_t *)app->user;
    (void)v;
    (void)frame;
    float y = (float)s->ball->physical_core->pos_world_f_core_last_psi.k[1];
    if (y < s->ball_min_y) s->ball_min_y = y;
}

static void ring_draw(demo_app_t *app, demo_viewer_t *v)
{
    ring_demo_t *s = (ring_demo_t *)app->user;
    demo_draw_object(v, s->ground, DEMO_COL_STATIC);
    demo_draw_object(v, s->ring, DEMO_COL_OBJECT_A);
    demo_draw_velocity(v, s->ring, DEMO_COL_VELOCITY);
    demo_draw_object(v, s->ball, DEMO_COL_OBJECT_B);
    demo_hud_object(v, "Ring", s->ring);
    demo_hud_object(v, "Ball", s->ball);
    demo_hud(v, DEMO_COL_INFO, "Ring: %d convex ledges + root convex hull, tilt %.1f deg",
             s->n_ledges, (double)demo_object_tilt_deg(s->ring));
}

static void ring_stats(demo_app_t *app, char *buf, size_t size)
{
    ring_demo_t *s = (ring_demo_t *)app->user;
    float p[3], b[3];
    demo_object_center(s->ring, p);
    demo_object_center(s->ball, b);
    snprintf(buf, size, "ring=(%.3f,%.3f,%.3f) tilt=%.2f speed=%.3f%s ball=(%.2f,%.2f,%.2f)",
             (double)p[0], (double)p[1], (double)p[2], (double)demo_object_tilt_deg(s->ring),
             (double)demo_object_speed(s->ring), ivp_object_is_frozen(s->ring) ? " frozen" : "",
             (double)b[0], (double)b[1], (double)b[2]);
}

static bool ring_check(demo_app_t *app)
{
    ring_demo_t *s = (ring_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    float p[3], b[3];
    demo_object_center(s->ring, p);
    demo_object_center(s->ball, b);
    ok &= demo_check(tag, s->n_ledges == 4, "ledge soup holds 4 convex ledges");
    ok &= demo_check(tag, p[1] > -0.6f && p[1] < -0.4f && demo_object_tilt_deg(s->ring) < 5.0f,
                     "ring lies flat on the ground (y = %.3f, tilt %.2f deg)",
                     (double)p[1], (double)demo_object_tilt_deg(s->ring));
    /* the ball rests on the ground (y = -0.3), not on the ring (y ~ -1.3) */
    ok &= demo_check(tag, b[1] > -0.4f, "ball fell through the hole onto the ground (y = %.3f)",
                     (double)b[1]);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title      = "IVP Quickstart: Ring";
    app.cfg.tag        = "ring";
    app.cfg.orbit_dist = 25.0f;
    app.cfg.target[1]  = -3.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    ring_demo_t *s = (ring_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;

    s->ground = demo_create_box(&app.world, NULL, 15.0, 0.5, 15.0, 0.0, NULL, 0.0, 0.5, 0.0);

    ivp_compact_surface_t *compact = build_ring_surface(&s->n_ledges);
    if (!compact) {
        fprintf(stderr, "[ring] ledge soup compile failed\n");
        return 1;
    }
    {
        ivp_object_template_t t;
        demo_template_dynamic(&t, &app.world.mat, 2.0);
        demo_set_box_inertia(&t, 2.0, 3.0, 0.5, 3.0);
        ivp_quatr_t q;
        ivp_vec3r_t rot = ivp_vec3r(IVP_REAL(-0.7), IVP_REAL(0.1), IVP_REAL(0.1));
        ivp_quat_set_fast_multiple(&q, &rot, 1.0);   /* IVP_U_Quat(IVP_U_Point(-0.7, 0.1, 0.1)) */
        demo_template_pose(&t, &q, 0.0, -8.0, 0.0);
        s->ring = demo_create_polygon(&app.world, compact, &t);
        if (!s->ring) return 1;
    }

    /* extra: a ball dropped through the hole once the ring has landed */
    s->ball = demo_create_ball(&app.world, NULL, 0.3, 0.5, NULL, 0.0, -14.0, 0.0);
    s->ball_min_y = 0.0f;

    app.update = ring_update;
    app.draw   = ring_draw;
    app.stats  = ring_stats;
    app.check  = ring_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
