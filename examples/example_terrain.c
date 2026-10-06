/* example_terrain.c -- A heightfield terrain with rolling bodies
 *
 * A 26 x 26 heightfield (bowl with hills and a ridge) is turned into a
 * compact grid with ivp_gridbuild_array_to_compact_grid and used as a static
 * polygon object through ivp_surman_compact_grid_init, the O(1) terrain
 * collision of the engine. Balls, boxes and convex hulls are dropped on it
 * and roll into the valleys. Space resets the scene.
 *
 * The viewer draws the same heightfield as a flat shaded mesh through
 * ivp_render_mesh_create; F1 switches to the wire grid.
 *
 * Demonstrates: ivp_gridbuild_array_to_compact_grid, the compact grid
 *               surface manager, a static object made by hand, and a custom
 *               render mesh.
 */
#include "demo_app.h"

#include "demo_terrain.h"

#define N          26          /* grid points per side */
#define FIELD      1.0f        /* metres between points */
#define N_BALLS    14
#define N_BOXES    10
#define N_HULLS    8
#define N_BODIES   (N_BALLS + N_BOXES + N_HULLS)

typedef struct terrain_demo {
    demo_terrain_t       terrain;
    ivp_object_t        *bodies[N_BODIES];
} terrain_demo_t;

static float terrain_height(int r, int c)
{
    float x = ((float)r - 0.5f * (N - 1)) * FIELD;
    float z = ((float)c - 0.5f * (N - 1)) * FIELD;
    float bowl  = 0.022f * (x * x + z * z);
    float hills = 1.1f * sinf(x * 0.55f) * cosf(z * 0.45f);
    float ridge = 1.4f * expf(-0.5f * ((x + 2.0f) * (x + 2.0f)) / 1.5f) * (z > -4.0f ? 1.0f : 0.0f);
    return bowl + hills + ridge + 0.6f;
}

static void terrain_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    terrain_demo_t *s = (terrain_demo_t *)app->user;
    demo_terrain_make_mesh(&s->terrain, v->renderer);
}

static const float palette[][3] = {
    { 0.95f, 0.45f, 0.30f }, { 0.35f, 0.75f, 0.95f }, { 0.95f, 0.80f, 0.30f },
    { 0.55f, 0.85f, 0.45f }, { 0.80f, 0.50f, 0.90f }, { 0.95f, 0.55f, 0.70f },
};

static void terrain_draw(demo_app_t *app, demo_viewer_t *v)
{
    terrain_demo_t *s = (terrain_demo_t *)app->user;
    const float c_land[3] = { 0.36f, 0.58f, 0.34f }, c_wire[3] = { 0.30f, 0.62f, 0.30f };

    demo_terrain_draw(v, &s->terrain, v->solid ? c_land : c_wire);

    int resting = 0;
    for (int i = 0; i < N_BODIES; i++) {
        demo_draw_object(v, s->bodies[i], palette[i % 6]);
        if (demo_object_speed(s->bodies[i]) < 0.1f) resting++;
    }
    demo_hud(v, DEMO_COL_INFO, "%d bodies on a %dx%d heightfield (%.1f .. %.1f m), %d at rest",
             N_BODIES, N, N, (double)s->terrain.h_min, (double)s->terrain.h_max, resting);
}

static void terrain_stats(demo_app_t *app, char *buf, size_t size)
{
    terrain_demo_t *s = (terrain_demo_t *)app->user;
    float vmax = 0.0f;
    for (int i = 0; i < N_BODIES; i++) {
        float sp = demo_object_speed(s->bodies[i]);
        if (sp > vmax) vmax = sp;
    }
    snprintf(buf, size, "max speed %.2f m/s", (double)vmax);
}

static bool terrain_check(demo_app_t *app)
{
    terrain_demo_t *s = (terrain_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    int on_top = 0, inside = 0;
    const float half = 0.5f * (N - 1) * FIELD;
    for (int i = 0; i < N_BODIES; i++) {
        float p[3];
        demo_object_center(s->bodies[i], p);
        /* up is -Y: a body that fell through the surface lies below its lowest point */
        if (p[1] < -s->terrain.h_min + 0.2f) on_top++;
        if (fabsf(p[0]) < half && fabsf(p[2]) < half) inside++;
    }
    ok &= demo_check(tag, on_top == N_BODIES, "%d/%d bodies stay on the terrain", on_top, N_BODIES);
    ok &= demo_check(tag, inside == N_BODIES, "%d/%d bodies stay inside the field", inside, N_BODIES);
    return ok;
}

static ivp_object_t *make_hull(demo_world_t *w, int kind, double x, double y, double z)
{
    static const double tetra[] = { 0.6, 0.6, 0.6, -0.6, -0.6, 0.6, -0.6, 0.6, -0.6, 0.6, -0.6, -0.6 };
    static const double wedge[] = { 0.7, 0.4, 0.5, -0.7, 0.4, 0.5, 0.7, 0.4, -0.5, -0.7, 0.4, -0.5,
                                    0.7, -0.4, 0.0, -0.7, -0.4, 0.0 };
    const double *pts = kind ? wedge : tetra;
    int n = kind ? 6 : 4;
    ivp_compact_surface_t *surf = ivp_build_convex_surface_d(NULL, pts, n);
    if (!surf) return NULL;
    ivp_object_template_t t;
    demo_template_dynamic(&t, &w->mat, 1.0);
    demo_set_box_inertia(&t, 1.0, 0.7, 0.5, 0.6);
    demo_template_pose(&t, NULL, x, y, z);
    return demo_create_polygon(w, surf, &t);
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Terrain -- Compact Grid";
    app.cfg.tag         = "terrain";
    app.cfg.friction    = 0.7;
    app.cfg.elasticity  = 0.2;
    app.cfg.orbit_dist  = 30.0f;
    app.cfg.target[1]   = -2.0f;
    app.cfg.grid_extent = -1.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    terrain_demo_t *s = (terrain_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;

    float heights[N * N];
    for (int r = 0; r < N; r++)
        for (int c = 0; c < N; c++) heights[r * N + c] = terrain_height(r, c);
    if (!demo_terrain_create(&app.world, &s->terrain, N, FIELD, heights, app.cfg.friction,
                             app.cfg.elasticity)) {
        fprintf(stderr, "[terrain] grid build failed\n");
        return 1;
    }

    /* deterministic drop pattern over the bowl */
    unsigned seed = 2024u;
    for (int i = 0; i < N_BODIES; i++) {
        seed = seed * 1664525u + 1013904223u;
        double ang = ((seed >> 8) & 1023) / 1023.0 * 6.2831853;
        seed = seed * 1664525u + 1013904223u;
        double rad = 1.5 + ((seed >> 8) & 1023) / 1023.0 * 6.0;
        double x = rad * cos(ang), z = rad * sin(ang);
        double y = -((double)s->terrain.h_max + 2.5 + 0.45 * i);
        ivp_object_t *o;
        if (i < N_BALLS) {
            double r = 0.35 + 0.1 * (i % 4);
            o = demo_create_ball(&app.world, NULL, r, 1.0 + r, NULL, x, y, z);
        } else if (i < N_BALLS + N_BOXES) {
            double h = 0.3 + 0.1 * (i % 3);
            o = demo_create_box(&app.world, NULL, h, h, h, 1.0 + 4.0 * h, NULL, x, y, z);
        } else {
            o = make_hull(&app.world, (i - N_BALLS - N_BOXES) & 1, x, y, z);
        }
        if (!o) return 1;
        s->bodies[i] = o;
    }

    app.viewer_init = terrain_viewer_init;
    app.draw        = terrain_draw;
    app.stats       = terrain_stats;
    app.check       = terrain_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    demo_terrain_destroy(&s->terrain);
    free(s);
    return rc;
}
