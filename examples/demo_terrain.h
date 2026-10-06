/* demo_terrain.h -- A heightfield terrain for the examples
 *
 * The caller fills an n x n array of heights (metres up, row-major, rows
 * along +X, columns along +Z). demo_terrain_create turns it into a compact
 * grid (ivp_gridbuild_array_to_compact_grid) and a static polygon object
 * through the compact grid surface manager; demo_terrain_draw shows it as
 * a flat shaded render mesh or as a wire grid. The terrain is centred on
 * the origin; IVP worlds are Y down, so a height h is the world y = -h.
 *
 * Included after demo_app.h.
 */
#ifndef IVP_EXAMPLE_DEMO_TERRAIN_H
#define IVP_EXAMPLE_DEMO_TERRAIN_H

#include <ivp/builder/ivp_build_grid.h>
#include <ivp/collision/ivp_ray_solver.h>
#include <ivp/geometry/ivp_compact_grid.h>
#include <ivp/geometry/ivp_surman_grid.h>

typedef struct demo_terrain {
    int                       n;           /* points per side */
    float                     field;       /* metres between points */
    float                    *heights;     /* n * n, metres up */
    float                     h_min, h_max;
    ivp_compact_grid_t       *grid;
    ivp_surman_compact_grid_t surman;
    ivp_object_t             *obj;
    ivp_render_mesh_t        *mesh;        /* window only, see demo_terrain_make_mesh */
} demo_terrain_t;

static inline float demo_terrain_half_extent(const demo_terrain_t *t)
{
    return 0.5f * (float)(t->n - 1) * t->field;
}

/* World position of grid point (r, c). */
static inline void demo_terrain_point(const demo_terrain_t *t, int r, int c, float p[3])
{
    const float half = demo_terrain_half_extent(t);
    p[0] = (float)r * t->field - half;
    p[1] = -t->heights[r * t->n + c];
    p[2] = (float)c * t->field - half;
}

/* An upper bound of the surface height (metres up) at world x, z: the highest
 * of the four grid points around it. Good for spawning bodies above the
 * terrain. Outside the field the edge is used. */
static inline float demo_terrain_height_at(const demo_terrain_t *t, float x, float z)
{
    const float half = demo_terrain_half_extent(t);
    int r = (int)floorf((x + half) / t->field), c = (int)floorf((z + half) / t->field);
    if (r < 0) r = 0;
    if (c < 0) c = 0;
    if (r > t->n - 2) r = t->n - 2;
    if (c > t->n - 2) c = t->n - 2;
    const float *h = t->heights;
    float m = h[r * t->n + c];
    if (h[(r + 1) * t->n + c] > m) m = h[(r + 1) * t->n + c];
    if (h[r * t->n + c + 1] > m) m = h[r * t->n + c + 1];
    if (h[(r + 1) * t->n + c + 1] > m) m = h[(r + 1) * t->n + c + 1];
    return m;
}

/* Exact surface height: the engine's ray solver, a ray straight down onto the
 * terrain object. Returns false when the ray misses (outside the field). */
static inline bool demo_terrain_surface_y(const demo_terrain_t *t, float x, float z, float *y_out)
{
    ivp_ray_solver_template_t tmpl;
    tmpl.ray_start_point = ivp_vec3r((ivp_real_t)x, (ivp_real_t)(-(t->h_max + 10.0f)), (ivp_real_t)z);
    tmpl.ray_normized_direction = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));   /* down is +Y */
    tmpl.ray_length = (ivp_real_t)(t->h_max - t->h_min + 20.0f);
    tmpl.ray_flags = IVP_RAY_SOLVER_ALL;
    ivp_ray_solver_min_t rs;
    ivp_ray_solver_min_init(&rs, &tmpl);
    ivp_ray_solver_check_object(&rs.base, t->obj);
    const ivp_ray_hit_t *h = ivp_ray_solver_min_get_ray_hit(&rs);
    if (!h) return false;
    *y_out = -(t->h_max + 10.0f) + (float)h->hit_distance;
    return true;
}

/* Build the collision object. `heights` (n * n floats) is copied. */
static inline bool demo_terrain_create(demo_world_t *w, demo_terrain_t *t, int n, float field,
                                       const float *heights, double friction, double elasticity)
{
    memset(t, 0, sizeof(*t));
    t->n = n;
    t->field = field;
    t->heights = (float *)malloc((size_t)n * (size_t)n * sizeof(float));
    if (!t->heights) return false;
    memcpy(t->heights, heights, (size_t)n * (size_t)n * sizeof(float));
    t->h_min = 1e30f;
    t->h_max = -1e30f;
    for (int i = 0; i < n * n; i++) {
        if (t->heights[i] < t->h_min) t->h_min = t->heights[i];
        if (t->heights[i] > t->h_max) t->h_max = t->heights[i];
    }

    /* compact grid: rows -> x, columns -> z, heights -> -y */
    ivp_template_compact_grid_t gt;
    ivp_template_compact_grid_init(&gt);
    gt.row_info.n_points = n;
    gt.row_info.maps_to = 0;
    gt.row_info.invert_axis = false;
    gt.column_info.n_points = n;
    gt.column_info.maps_to = 2;
    gt.column_info.invert_axis = false;
    gt.height_maps_to = 1;
    gt.height_invert_axis = true;
    gt.grid_field_size = field;
    t->grid = ivp_gridbuild_array_to_compact_grid(NULL, &gt, t->heights);
    if (!t->grid) {
        free(t->heights);
        t->heights = NULL;
        return false;
    }
    ivp_surman_compact_grid_init(&t->surman, t->grid);

    ivp_material_t mat = demo_material(friction, elasticity);
    ivp_object_template_t ot;
    demo_template_static(&ot, &mat);
    ot.type = IVP_OBJECT_POLYGON;
    const double corner = -(double)demo_terrain_half_extent(t);
    demo_template_pose(&ot, NULL, corner, 0.0, corner);
    t->obj = ivp_environment_create_object(w->env, &ot, &t->surman.base);
    return t->obj != NULL;
}

/* Two triangles per cell, wound counter-clockwise seen from above (-Y). */
static inline void demo_terrain_make_mesh(demo_terrain_t *t, ivp_renderer_t *rend)
{
    const int n = t->n;
    float *pos = (float *)malloc((size_t)n * (size_t)n * 3 * sizeof(float));
    int *idx = (int *)malloc((size_t)(n - 1) * (size_t)(n - 1) * 6 * sizeof(int));
    if (!pos || !idx) { free(pos); free(idx); return; }
    for (int r = 0; r < n; r++)
        for (int c = 0; c < n; c++) demo_terrain_point(t, r, c, &pos[(r * n + c) * 3]);

    int count = 0;
    for (int r = 0; r < n - 1; r++)
        for (int c = 0; c < n - 1; c++) {
            int a = r * n + c, b = a + 1, d = a + n, e = d + 1;
            const int tri[2][3] = { { a, b, d }, { b, e, d } };
            for (int k = 0; k < 2; k++) {
                const float *p0 = &pos[tri[k][0] * 3], *p1 = &pos[tri[k][1] * 3], *p2 = &pos[tri[k][2] * 3];
                float ny = (p1[2] - p0[2]) * (p2[0] - p0[0]) - (p1[0] - p0[0]) * (p2[2] - p0[2]);
                bool up = ny < 0.0f;                          /* normal towards -Y */
                idx[count++] = tri[k][0];
                idx[count++] = up ? tri[k][1] : tri[k][2];
                idx[count++] = up ? tri[k][2] : tri[k][1];
            }
        }
    t->mesh = ivp_render_mesh_create(rend, pos, n * n, idx, count / 3);
    free(pos);
    free(idx);
}

static inline void demo_terrain_draw(demo_viewer_t *v, const demo_terrain_t *t, const float color[3])
{
    if (v->solid && t->mesh) {
        const float origin[3] = { 0.0f, 0.0f, 0.0f }, id[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
        demo_solid_mesh(v, t->mesh, origin, id, color);
        return;
    }
    for (int r = 0; r < t->n; r++)
        for (int c = 0; c < t->n; c++) {
            float p[3], q[3];
            demo_terrain_point(t, r, c, p);
            if (r + 1 < t->n) { demo_terrain_point(t, r + 1, c, q); demo_line(v, p, q, color); }
            if (c + 1 < t->n) { demo_terrain_point(t, r, c + 1, q); demo_line(v, p, q, color); }
        }
}

/* The grid is freed here; the object goes away with the environment, so
 * destroy the world first. */
static inline void demo_terrain_destroy(demo_terrain_t *t)
{
    if (t->grid) ivp_compact_grid_free(NULL, t->grid);
    free(t->heights);
    memset(t, 0, sizeof(*t));
}

#endif /* IVP_EXAMPLE_DEMO_TERRAIN_H */
