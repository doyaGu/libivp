/* example_shapes.c -- Convex hull bodies, drawn as lit solid meshes
 *
 * Cylinders, cones, prisms and pyramids are built as point clouds,
 * turned into convex compact surfaces by the qhull based builder
 * (ivp_build_convex_surface_d) and dropped onto a static ground.  The
 * viewer draws them as flat shaded meshes generated from the ledges; F1
 * switches to wireframe.
 *
 * Demonstrates: ivp_build_convex_surface_d, demo_create_polygon, convex
 *               hulls rolling, sliding and stacking by shape.
 */
#include "demo_app.h"

#define N_KINDS   6
#define PER_KIND  3
#define N_BODIES  (N_KINDS * PER_KIND)
#define GROUND_TOP (-0.5f)   /* IVP worlds are Y down: up is -Y */

typedef struct shapes_demo {
    ivp_object_t *ground;
    ivp_object_t *walls[4];
    ivp_object_t *bodies[N_BODIES];
    int           kind[N_BODIES];
} shapes_demo_t;

static const char *kind_name[N_KINDS] = {
    "cylinder", "cone", "hex prism", "pyramid", "octahedron", "icosahedron"
};

/* Point cloud of one kind; returns the count, half_y is the bounding half
 * height (for the inertia estimate). */
static int kind_points(int kind, double *pts, double *half_r, double *half_y)
{
    const int N = 20;
    int n = 0;
    const double two_pi = 6.28318530717958647692;
#define P(x, y, z) do { pts[3*n] = (x); pts[3*n+1] = (y); pts[3*n+2] = (z); n++; } while (0)
    switch (kind) {
    case 0:   /* cylinder */
        *half_r = 0.5;  *half_y = 0.6;
        for (int i = 0; i < N; i++) {
            double a = two_pi * i / N;
            P(0.5 * cos(a),  0.6, 0.5 * sin(a));
            P(0.5 * cos(a), -0.6, 0.5 * sin(a));
        }
        break;
    case 1:   /* cone, centred on its centre of mass; apex up = -Y */
        *half_r = 0.6;  *half_y = 0.6;
        for (int i = 0; i < N; i++) {
            double a = two_pi * i / N;
            P(0.6 * cos(a), 0.3, 0.6 * sin(a));
        }
        P(0.0, -0.9, 0.0);
        break;
    case 2:   /* hexagonal prism */
        *half_r = 0.6;  *half_y = 0.4;
        for (int i = 0; i < 6; i++) {
            double a = two_pi * i / 6;
            P(0.6 * cos(a),  0.4, 0.6 * sin(a));
            P(0.6 * cos(a), -0.4, 0.6 * sin(a));
        }
        break;
    case 3:   /* square pyramid */
        *half_r = 0.7;  *half_y = 0.6;
        P( 0.7, 0.4,  0.7); P(-0.7, 0.4,  0.7);
        P(-0.7, 0.4, -0.7); P( 0.7, 0.4, -0.7);
        P( 0.0, -0.8, 0.0);
        break;
    case 4:   /* octahedron */
        *half_r = 0.7;  *half_y = 0.7;
        P( 0.7, 0, 0); P(-0.7, 0, 0); P(0,  0.7, 0);
        P(0, -0.7, 0); P(0, 0,  0.7); P(0, 0, -0.7);
        break;
    default: {   /* icosahedron */
        const double t = (1.0 + sqrt(5.0)) * 0.5, s = 0.45;
        *half_r = 0.8;  *half_y = 0.8;
        for (int i = 0; i < 2; i++)
            for (int j = 0; j < 2; j++) {
                double a = (i ? -1.0 : 1.0) * s, b = (j ? -1.0 : 1.0) * s * t;
                P(0, a, b); P(a, b, 0); P(b, 0, a);
            }
        break;
    }
    }
#undef P
    return n;
}

static ivp_object_t *create_body(demo_world_t *w, int kind, double x, double y, double z)
{
    double pts[3 * 64], hr, hy;
    int n = kind_points(kind, pts, &hr, &hy);
    ivp_compact_surface_t *surf = ivp_build_convex_surface_d(NULL, pts, n);
    if (!surf) return NULL;
    ivp_object_template_t t;
    demo_template_dynamic(&t, &w->mat, 1.0);
    demo_set_box_inertia(&t, 1.0, hr, hy, hr);
    demo_template_pose(&t, NULL, x, y, z);
    return demo_create_polygon(w, surf, &t);   /* the world owns the surface */
}

static const float palette[N_KINDS][3] = {
    { 0.95f, 0.45f, 0.30f }, { 0.35f, 0.75f, 0.95f }, { 0.55f, 0.85f, 0.45f },
    { 0.95f, 0.80f, 0.30f }, { 0.80f, 0.50f, 0.90f }, { 0.95f, 0.55f, 0.70f },
};

static void shapes_draw(demo_app_t *app, demo_viewer_t *v)
{
    shapes_demo_t *s = (shapes_demo_t *)app->user;
    const float ground[3] = { 0.55f, 0.58f, 0.62f };
    demo_draw_object(v, s->ground, ground);
    for (int i = 0; i < 4; i++) demo_draw_object(v, s->walls[i], ground);
    for (int i = 0; i < N_BODIES; i++)
        demo_draw_object(v, s->bodies[i], palette[s->kind[i]]);

    int resting = 0;
    for (int i = 0; i < N_BODIES; i++)
        if (demo_object_speed(s->bodies[i]) < 0.05f) resting++;
    demo_hud(v, DEMO_COL_INFO, "%d convex hulls, %d at rest   (F1: wireframe)", N_BODIES, resting);
}

static void shapes_stats(demo_app_t *app, char *buf, size_t size)
{
    shapes_demo_t *s = (shapes_demo_t *)app->user;
    float vmax = 0.0f;
    for (int i = 0; i < N_BODIES; i++) {
        float sp = demo_object_speed(s->bodies[i]);
        if (sp > vmax) vmax = sp;
    }
    snprintf(buf, size, "max speed %.3f m/s", (double)vmax);
}

static bool shapes_check(demo_app_t *app)
{
    shapes_demo_t *s = (shapes_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    int above = 0, inside = 0;
    for (int i = 0; i < N_BODIES; i++) {
        float p[3];
        demo_object_center(s->bodies[i], p);
        if (p[1] < GROUND_TOP) above++;
        if (fabsf(p[0]) < 10.0f && fabsf(p[2]) < 10.0f) inside++;
    }
    ok &= demo_check(tag, above == N_BODIES, "%d/%d hulls stay above the ground", above, N_BODIES);
    ok &= demo_check(tag, inside == N_BODIES, "%d/%d hulls stay on the ground plate", inside, N_BODIES);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title      = "IVP Shapes -- Convex Hulls";
    app.cfg.tag        = "shapes";
    app.cfg.orbit_dist = 14.0f;
    app.cfg.target[1]  = -1.0f;
    app.cfg.grid_extent = -1.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    shapes_demo_t *s = (shapes_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    app.controls = "F1:wireframe";

    s->ground = demo_create_box(&app.world, NULL, 10.0, 0.5, 10.0, 0.0, NULL, 0.0, 0.0, 0.0);

    /* low walls keep the rolling hulls on the plate (up is -Y) */
    s->walls[0] = demo_create_box(&app.world, NULL, 10.0, 0.75, 0.4, 0.0, NULL, 0.0, -1.25, -10.4);
    s->walls[1] = demo_create_box(&app.world, NULL, 10.0, 0.75, 0.4, 0.0, NULL, 0.0, -1.25, 10.4);
    s->walls[2] = demo_create_box(&app.world, NULL, 0.4, 0.75, 10.8, 0.0, NULL, -10.4, -1.25, 0.0);
    s->walls[3] = demo_create_box(&app.world, NULL, 0.4, 0.75, 10.8, 0.0, NULL, 10.4, -1.25, 0.0);

    /* deterministic scatter: kinds in a loose grid, dropped from rising heights */
    unsigned seed = 12345u;
    for (int i = 0; i < N_BODIES; i++) {
        int kind = i % N_KINDS;
        seed = seed * 1664525u + 1013904223u;
        double jx = ((seed >> 8) & 255) / 255.0 - 0.5;
        seed = seed * 1664525u + 1013904223u;
        double jz = ((seed >> 8) & 255) / 255.0 - 0.5;
        double x = (kind - 2.5) * 2.0 + jx * 0.6;
        double z = (i / N_KINDS - 1) * 2.4 + jz * 0.6;
        double y = -(1.5 + 1.4 * (i / N_KINDS) + 0.35 * kind);
        s->kind[i] = kind;
        s->bodies[i] = create_body(&app.world, kind, x, y, z);
        if (!s->bodies[i]) return 1;
    }
    (void)kind_name;

    app.draw  = shapes_draw;
    app.stats = shapes_stats;
    app.check = shapes_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
