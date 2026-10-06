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

#include <ivp/builder/ivp_build_grid.h>
#include <ivp/geometry/ivp_compact_grid.h>
#include <ivp/geometry/ivp_surman_grid.h>

#define N          26          /* grid points per side */
#define FIELD      1.0f        /* metres between points */
#define N_BALLS    14
#define N_BOXES    10
#define N_HULLS    8
#define N_BODIES   (N_BALLS + N_BOXES + N_HULLS)

typedef struct terrain_demo {
    float                heights[N * N];     /* row * N + column, metres up */
    float                h_min, h_max;
    ivp_compact_grid_t  *grid;
    ivp_surman_compact_grid_t surman;
    ivp_object_t        *ground;
    ivp_object_t        *bodies[N_BODIES];
    ivp_render_mesh_t   *mesh;               /* window only */
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

/* World position of grid point (r, c): rows run along +X, columns along +Z,
 * the height along -Y (IVP worlds are Y down). */
static void grid_point(const terrain_demo_t *s, int r, int c, float p[3])
{
    p[0] = ((float)r - 0.5f * (N - 1)) * FIELD;
    p[1] = -s->heights[r * N + c];
    p[2] = ((float)c - 0.5f * (N - 1)) * FIELD;
}

/* Two triangles per cell, wound counter-clockwise seen from above (-Y). */
static ivp_render_mesh_t *build_mesh(ivp_renderer_t *rend, const terrain_demo_t *s)
{
    static float pos[N * N * 3];
    static int idx[(N - 1) * (N - 1) * 6];
    for (int r = 0; r < N; r++)
        for (int c = 0; c < N; c++) grid_point(s, r, c, &pos[(r * N + c) * 3]);

    int n = 0;
    for (int r = 0; r < N - 1; r++)
        for (int c = 0; c < N - 1; c++) {
            int a = r * N + c, b = a + 1, d = a + N, e = d + 1;
            const int tri[2][3] = { { a, b, d }, { b, e, d } };
            for (int t = 0; t < 2; t++) {
                const float *p0 = &pos[tri[t][0] * 3], *p1 = &pos[tri[t][1] * 3], *p2 = &pos[tri[t][2] * 3];
                float ny = (p1[2] - p0[2]) * (p2[0] - p0[0]) - (p1[0] - p0[0]) * (p2[2] - p0[2]);
                bool up = ny < 0.0f;                      /* normal towards -Y */
                idx[n++] = tri[t][0];
                idx[n++] = up ? tri[t][1] : tri[t][2];
                idx[n++] = up ? tri[t][2] : tri[t][1];
            }
        }
    return ivp_render_mesh_create(rend, pos, N * N, idx, n / 3);
}

static void terrain_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    terrain_demo_t *s = (terrain_demo_t *)app->user;
    s->mesh = build_mesh(v->renderer, s);
}

static const float palette[][3] = {
    { 0.95f, 0.45f, 0.30f }, { 0.35f, 0.75f, 0.95f }, { 0.95f, 0.80f, 0.30f },
    { 0.55f, 0.85f, 0.45f }, { 0.80f, 0.50f, 0.90f }, { 0.95f, 0.55f, 0.70f },
};

static void terrain_draw(demo_app_t *app, demo_viewer_t *v)
{
    terrain_demo_t *s = (terrain_demo_t *)app->user;
    const float c_land[3] = { 0.36f, 0.58f, 0.34f };

    if (v->solid && s->mesh) {
        const float origin[3] = { 0.0f, 0.0f, 0.0f }, id[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        demo_solid_mesh(v, s->mesh, origin, id, c_land);
    } else {
        const float c_wire[3] = { 0.30f, 0.62f, 0.30f };
        for (int r = 0; r < N; r++)
            for (int c = 0; c < N; c++) {
                float p[3], q[3];
                grid_point(s, r, c, p);
                if (r + 1 < N) { grid_point(s, r + 1, c, q); demo_line(v, p, q, c_wire); }
                if (c + 1 < N) { grid_point(s, r, c + 1, q); demo_line(v, p, q, c_wire); }
            }
    }

    int resting = 0;
    for (int i = 0; i < N_BODIES; i++) {
        demo_draw_object(v, s->bodies[i], palette[i % 6]);
        if (demo_object_speed(s->bodies[i]) < 0.1f) resting++;
    }
    demo_hud(v, DEMO_COL_INFO, "%d bodies on a %dx%d heightfield (%.1f .. %.1f m), %d at rest",
             N_BODIES, N, N, (double)s->h_min, (double)s->h_max, resting);
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
        if (p[1] < -s->h_min + 0.2f) on_top++;
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

    s->h_min = 1e9f;
    s->h_max = -1e9f;
    for (int r = 0; r < N; r++)
        for (int c = 0; c < N; c++) {
            float h = terrain_height(r, c);
            s->heights[r * N + c] = h;
            if (h < s->h_min) s->h_min = h;
            if (h > s->h_max) s->h_max = h;
        }

    /* compact grid: rows -> x, columns -> z, heights -> -y */
    ivp_template_compact_grid_t gt;
    ivp_template_compact_grid_init(&gt);
    gt.row_info.n_points = N;
    gt.row_info.maps_to = 0;
    gt.row_info.invert_axis = false;
    gt.column_info.n_points = N;
    gt.column_info.maps_to = 2;
    gt.column_info.invert_axis = false;
    gt.height_maps_to = 1;
    gt.height_invert_axis = true;
    gt.grid_field_size = FIELD;
    s->grid = ivp_gridbuild_array_to_compact_grid(NULL, &gt, s->heights);
    if (!s->grid) {
        fprintf(stderr, "[terrain] grid build failed\n");
        return 1;
    }
    ivp_surman_compact_grid_init(&s->surman, s->grid);

    {
        ivp_object_template_t t;
        demo_template_static(&t, &app.world.mat);
        t.type = IVP_OBJECT_POLYGON;
        demo_template_pose(&t, NULL, -0.5 * (N - 1) * FIELD, 0.0, -0.5 * (N - 1) * FIELD);
        s->ground = ivp_environment_create_object(app.world.env, &t, &s->surman.base);
        if (!s->ground) return 1;
    }

    /* deterministic drop pattern over the bowl */
    unsigned seed = 2024u;
    for (int i = 0; i < N_BODIES; i++) {
        seed = seed * 1664525u + 1013904223u;
        double ang = ((seed >> 8) & 1023) / 1023.0 * 6.2831853;
        seed = seed * 1664525u + 1013904223u;
        double rad = 1.5 + ((seed >> 8) & 1023) / 1023.0 * 6.0;
        double x = rad * cos(ang), z = rad * sin(ang);
        double y = -(s->h_max + 2.5 + 0.45 * i);
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
    ivp_compact_grid_free(NULL, s->grid);
    free(s);
    return rc;
}
