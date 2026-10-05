/* ivp_surman_grid.c -- Grid-based surface manager implementation */

#include <ivp/geometry/ivp_surman_grid.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/collision/ivp_ray_solver.h>
#include <ivp/base/ivp_assert.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <limits.h>

/* ── Surman vtable ─────────────────────────────────────────────────────── */

static const ivp_compact_ledge_t *grid_get_single_convex(const ivp_surman_t *sm)
{
    (void)sm;
    return NULL; /* grid surfaces are always multi-ledge */
}

static void grid_get_mass_center(const ivp_surman_t *sm, float out[3])
{
    const ivp_surman_grid_t *g = (const ivp_surman_grid_t *)sm;
    out[0] = g->compact_surface->mass_center[0];
    out[1] = g->compact_surface->mass_center[1];
    out[2] = g->compact_surface->mass_center[2];
}

static void grid_get_rotation_inertia(const ivp_surman_t *sm, float out[3])
{
    const ivp_surman_grid_t *g = (const ivp_surman_grid_t *)sm;
    out[0] = g->compact_surface->rotation_inertia[0];
    out[1] = g->compact_surface->rotation_inertia[1];
    out[2] = g->compact_surface->rotation_inertia[2];
}

static void grid_get_radius_and_deviation(const ivp_surman_t *sm,
                                            const float center[3],
                                            float *out_radius,
                                            float *out_deviation)
{
    /* the libivp grid wraps a compact surface: same header based answer as
     * IVP_SurfaceManager_Polygon (ivp_surman_polygon.cxx:214) */
    const ivp_surman_grid_t *g = (const ivp_surman_grid_t *)sm;
    ivp_surman_polygon_surface_radius_and_deviation(g->compact_surface, center,
                                                    out_radius, out_deviation);
}

static void grid_ledges_within_radius(const ivp_surman_t *sm, const double pos[3],
                                      double radius, const ivp_compact_ledge_t *root_ledge,
                                      const ivp_compact_ledge_t *other_reference_ledge,
                                      ivp_surman_ledge_fn add_fn, void *ctx)
{
    (void)other_reference_ledge;
    const ivp_surman_grid_t *g = (const ivp_surman_grid_t *)sm;
    ivp_surman_polygon_surface_ledges_within_radius(g->compact_surface, pos, radius,
                                                    root_ledge, add_fn, ctx);
}

static void grid_terminal_ledges(const ivp_surman_t *sm, ivp_surman_ledge_fn fn, void *ctx)
{
    const ivp_surman_grid_t *g = (const ivp_surman_grid_t *)sm;
    if (!fn) return;
    for (int i = 0; i < g->n_ledges; i++) fn(ctx, g->ledges[i]);
}

static void grid_insert_ledges_hitting_ray(const ivp_surman_t *sm,
                                           struct ivp_ray_solver *ray_solver,
                                           struct ivp_object *object)
{
    const ivp_surman_grid_t *g = (const ivp_surman_grid_t *)sm;
    ivp_ray_solver_insert_surface_hits(ray_solver, object, g->compact_surface);
}

static ivp_surman_type_t grid_get_type(const ivp_surman_t *sm)
{
    (void)sm;
    return IVP_SURMAN_POLYGON;
}

static void grid_destroy(ivp_surman_t *sm)
{
    ivp_surman_grid_destroy((ivp_surman_grid_t *)sm);
}

static const ivp_surman_vtable_t s_grid_vtable = {
    .get_single_convex      = grid_get_single_convex,
    .get_mass_center        = grid_get_mass_center,
    .get_rotation_inertia   = grid_get_rotation_inertia,
    .get_radius_and_deviation = grid_get_radius_and_deviation,
    .get_type               = grid_get_type,
    .get_ledgetree_root     = NULL,
    .destroy                = grid_destroy,
    .get_all_ledges_within_radius = grid_ledges_within_radius,
    .get_all_terminal_ledges = grid_terminal_ledges,
    .insert_all_ledges_hitting_ray = grid_insert_ledges_hitting_ray
};

/* ── Ledge collection callback ─────────────────────────────────────────── */

typedef struct ledge_collect_ctx {
    const ivp_compact_ledge_t **ledges;
    int count;
    int capacity;
} ledge_collect_ctx_t;

static bool collect_ledge(const ivp_compact_ledge_t *ledge, void *user_data)
{
    ledge_collect_ctx_t *ctx = (ledge_collect_ctx_t *)user_data;
    if (ctx->count < ctx->capacity) {
        ctx->ledges[ctx->count++] = ledge;
    }
    return true;
}

/* ── Cell helpers ──────────────────────────────────────────────────────── */

static bool size_mul_overflow(size_t a, size_t b, size_t *out)
{
    if (a != 0 && b > SIZE_MAX / a) return true;
    *out = a * b;
    return false;
}

static void cell_init(ivp_surman_grid_cell_t *cell)
{
    cell->ledge_indices = NULL;
    cell->count = 0;
    cell->capacity = 0;
}

static bool cell_add(ivp_surman_grid_cell_t *cell, int index,
                     const ivp_allocator_t *alloc)
{
    if (cell->count >= cell->capacity) {
        if (cell->capacity > INT_MAX / 2) return false;
        int new_cap = cell->capacity == 0 ? 4 : cell->capacity * 2;
        size_t bytes = 0;
        if (size_mul_overflow((size_t)new_cap, sizeof(int), &bytes)) {
            return false;
        }
        int *new_arr = (int *)ivp_alloc(alloc,
                                         bytes);
        if (!new_arr) return false;
        if (cell->ledge_indices) {
            memcpy(new_arr, cell->ledge_indices,
                   (size_t)cell->count * sizeof(int));
            ivp_free(alloc, cell->ledge_indices,
                     (size_t)cell->capacity * sizeof(int));
        }
        cell->ledge_indices = new_arr;
        cell->capacity = new_cap;
    }
    cell->ledge_indices[cell->count++] = index;
    return true;
}

static void cell_destroy(ivp_surman_grid_cell_t *cell,
                         const ivp_allocator_t *alloc)
{
    if (cell->ledge_indices) {
        ivp_free(alloc, cell->ledge_indices,
                 (size_t)cell->capacity * sizeof(int));
    }
    cell->ledge_indices = NULL;
    cell->count = 0;
    cell->capacity = 0;
}

/* ── Coordinate helpers ────────────────────────────────────────────────── */

static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void world_to_cell(const ivp_surman_grid_t *sm,
                           float wx, float wy, float wz,
                           int *cx, int *cy, int *cz)
{
    *cx = (int)floorf((wx - sm->aabb_min[0]) * sm->inv_cell_size[0]);
    *cy = (int)floorf((wy - sm->aabb_min[1]) * sm->inv_cell_size[1]);
    *cz = (int)floorf((wz - sm->aabb_min[2]) * sm->inv_cell_size[2]);
    *cx = clamp_int(*cx, 0, sm->cells_x - 1);
    *cy = clamp_int(*cy, 0, sm->cells_y - 1);
    *cz = clamp_int(*cz, 0, sm->cells_z - 1);
}

static int cell_index(const ivp_surman_grid_t *sm, int cx, int cy, int cz)
{
    return cz * (sm->cells_x * sm->cells_y) + cy * sm->cells_x + cx;
}

/* ── Init / Destroy ────────────────────────────────────────────────────── */

ivp_result_t ivp_surman_grid_init(ivp_surman_grid_t *sm,
                                    const ivp_compact_surface_t *surface,
                                    const ivp_surman_grid_config_t *config,
                                    const ivp_allocator_t *alloc)
{
    ivp_result_t err = IVP_OK;
    if (!sm || !surface || !alloc) return IVP_ERR_INVALID_ARG;

    memset(sm, 0, sizeof(*sm));
    sm->base.vtable = &s_grid_vtable;
    sm->compact_surface = surface;
    sm->allocator = alloc;

    ivp_surman_grid_config_t cfg;
    if (config) cfg = *config;
    else ivp_surman_grid_config_init(&cfg);

    sm->cells_x = cfg.cells_x > 0 ? cfg.cells_x : 1;
    sm->cells_y = cfg.cells_y > 0 ? cfg.cells_y : 1;
    sm->cells_z = cfg.cells_z > 0 ? cfg.cells_z : 1;
    {
        size_t total_cells = 0;
        if (size_mul_overflow((size_t)sm->cells_x, (size_t)sm->cells_y, &total_cells) ||
            size_mul_overflow(total_cells, (size_t)sm->cells_z, &total_cells) ||
            total_cells > (size_t)INT_MAX) {
            return IVP_ERR_CAPACITY_EXCEEDED;
        }
        sm->total_cells = (int)total_cells;
    }

    /* Count ledges */
    const ivp_compact_ledgetree_node_t *root =
        ivp_surface_get_ledgetree_root(surface);
    int n_ledges = ivp_ledgetree_count_terminal_ledges(root);
    if (n_ledges <= 0) return IVP_ERR_INVALID_ARG;

    /* Collect all ledge pointers */
    size_t ledges_bytes = 0;
    if (size_mul_overflow((size_t)n_ledges, sizeof(*sm->ledges), &ledges_bytes)) {
        return IVP_ERR_CAPACITY_EXCEEDED;
    }
    sm->ledges = (const ivp_compact_ledge_t **)ivp_alloc(
        alloc, ledges_bytes);
    if (!sm->ledges) {
        err = IVP_ERR_OUT_OF_MEMORY;
        goto fail;
    }
    sm->n_ledges = n_ledges; /* allocation size used by destroy() on failure paths */

    ledge_collect_ctx_t cctx;
    cctx.ledges = sm->ledges;
    cctx.count = 0;
    cctx.capacity = n_ledges;
    ivp_ledgetree_visit_all(root, collect_ledge, &cctx);
    if (cctx.count != n_ledges) {
        err = IVP_ERR_INVALID_STATE;
        goto fail;
    }
    sm->n_ledges = cctx.count;

    /* Compute AABB from all ledge bounding spheres */
    float large = 1e30f;
    sm->aabb_min[0] = sm->aabb_min[1] = sm->aabb_min[2] = large;
    sm->aabb_max[0] = sm->aabb_max[1] = sm->aabb_max[2] = -large;

    /* Walk ledgetree nodes to get bounding info */
    /* Use the root center/radius as overall bound */
    for (int a = 0; a < 3; a++) {
        sm->aabb_min[a] = root->center[a] - root->radius;
        sm->aabb_max[a] = root->center[a] + root->radius;
    }

    /* Compute cell sizes */
    for (int a = 0; a < 3; a++) {
        float extent = sm->aabb_max[a] - sm->aabb_min[a];
        if (extent < 1e-6f) extent = 1.0f;
        int n_cells = (a == 0) ? sm->cells_x :
                      (a == 1) ? sm->cells_y : sm->cells_z;
        sm->cell_size[a] = extent / (float)n_cells;
        sm->inv_cell_size[a] = 1.0f / sm->cell_size[a];
    }

    /* Allocate cells */
    size_t cells_bytes = 0;
    if (size_mul_overflow((size_t)sm->total_cells, sizeof(ivp_surman_grid_cell_t), &cells_bytes)) {
        err = IVP_ERR_CAPACITY_EXCEEDED;
        goto fail;
    }
    sm->cells = (ivp_surman_grid_cell_t *)ivp_alloc(
        alloc, cells_bytes);
    if (!sm->cells) {
        err = IVP_ERR_OUT_OF_MEMORY;
        goto fail;
    }

    for (int i = 0; i < sm->total_cells; i++) {
        cell_init(&sm->cells[i]);
    }

    /* Insert each ledge into overlapping grid cells.
     * Walk the ledgetree to get bounding info for each terminal. */
    /* For simplicity, use each ledge's bounding sphere from the
     * compact surface. We need to traverse ledgetree nodes. */
    /* Since we only have terminal ledge pointers, we approximate
     * the bounding sphere from the ledge's own points. */
    for (int li = 0; li < sm->n_ledges; li++) {
        const ivp_compact_ledge_t *ledge = sm->ledges[li];
        int n_pts = ivp_ledge_get_n_points(ledge);
        const ivp_compact_poly_point_t *pts =
            ivp_ledge_get_point_array(ledge);

        /* Compute ledge AABB from points */
        float lmin[3] = { large, large, large };
        float lmax[3] = { -large, -large, -large };
        for (int p = 0; p < n_pts; p++) {
            for (int a = 0; a < 3; a++) {
                float v = pts[p].k[a];
                if (v < lmin[a]) lmin[a] = v;
                if (v > lmax[a]) lmax[a] = v;
            }
        }

        /* Map AABB to cell range */
        int c_min[3], c_max[3];
        world_to_cell(sm, lmin[0], lmin[1], lmin[2],
                      &c_min[0], &c_min[1], &c_min[2]);
        world_to_cell(sm, lmax[0], lmax[1], lmax[2],
                      &c_max[0], &c_max[1], &c_max[2]);

        /* Insert into all overlapping cells */
        for (int cz = c_min[2]; cz <= c_max[2]; cz++) {
            for (int cy = c_min[1]; cy <= c_max[1]; cy++) {
                for (int cx = c_min[0]; cx <= c_max[0]; cx++) {
                    int ci = cell_index(sm, cx, cy, cz);
                    if (!cell_add(&sm->cells[ci], li, alloc)) {
                        err = IVP_ERR_OUT_OF_MEMORY;
                        goto fail;
                    }
                }
            }
        }
    }

    return IVP_OK;

fail:
    ivp_surman_grid_destroy(sm);
    return err;
}

void ivp_surman_grid_destroy(ivp_surman_grid_t *sm)
{
    if (!sm || !sm->allocator) return;

    const ivp_allocator_t *alloc = sm->allocator;

    if (sm->cells) {
        for (int i = 0; i < sm->total_cells; i++) {
            cell_destroy(&sm->cells[i], alloc);
        }
        ivp_free(alloc, sm->cells,
                 (size_t)sm->total_cells * sizeof(ivp_surman_grid_cell_t));
    }

    if (sm->ledges) {
        ivp_free(alloc, (void *)sm->ledges,
                 (size_t)sm->n_ledges * sizeof(void *));
    }

    memset(sm, 0, sizeof(*sm));
}

/* ── Queries ───────────────────────────────────────────────────────────── */

int ivp_surman_grid_query_point(const ivp_surman_grid_t *sm,
                                  float x, float y, float z,
                                  const int **out_indices)
{
    if (!sm || !sm->cells || !out_indices) return 0;

    /* Check bounds */
    if (x < sm->aabb_min[0] || x > sm->aabb_max[0] ||
        y < sm->aabb_min[1] || y > sm->aabb_max[1] ||
        z < sm->aabb_min[2] || z > sm->aabb_max[2]) {
        *out_indices = NULL;
        return 0;
    }

    int cx, cy, cz;
    world_to_cell(sm, x, y, z, &cx, &cy, &cz);
    int ci = cell_index(sm, cx, cy, cz);

    *out_indices = sm->cells[ci].ledge_indices;
    return sm->cells[ci].count;
}

int ivp_surman_grid_query_aabb(const ivp_surman_grid_t *sm,
                                 const float aabb_min[3],
                                 const float aabb_max[3],
                                 int *out_indices,
                                 int max_out)
{
    if (!sm || !sm->cells || !out_indices || max_out <= 0) return 0;
    if (aabb_min[0] > aabb_max[0] ||
        aabb_min[1] > aabb_max[1] ||
        aabb_min[2] > aabb_max[2]) {
        return 0;
    }
    if (aabb_max[0] < sm->aabb_min[0] || aabb_min[0] > sm->aabb_max[0] ||
        aabb_max[1] < sm->aabb_min[1] || aabb_min[1] > sm->aabb_max[1] ||
        aabb_max[2] < sm->aabb_min[2] || aabb_min[2] > sm->aabb_max[2]) {
        return 0;
    }

    float qmin[3] = {
        (aabb_min[0] > sm->aabb_min[0]) ? aabb_min[0] : sm->aabb_min[0],
        (aabb_min[1] > sm->aabb_min[1]) ? aabb_min[1] : sm->aabb_min[1],
        (aabb_min[2] > sm->aabb_min[2]) ? aabb_min[2] : sm->aabb_min[2]
    };
    float qmax[3] = {
        (aabb_max[0] < sm->aabb_max[0]) ? aabb_max[0] : sm->aabb_max[0],
        (aabb_max[1] < sm->aabb_max[1]) ? aabb_max[1] : sm->aabb_max[1],
        (aabb_max[2] < sm->aabb_max[2]) ? aabb_max[2] : sm->aabb_max[2]
    };

    int c_min[3], c_max[3];
    world_to_cell(sm, qmin[0], qmin[1], qmin[2],
                  &c_min[0], &c_min[1], &c_min[2]);
    world_to_cell(sm, qmax[0], qmax[1], qmax[2],
                  &c_max[0], &c_max[1], &c_max[2]);

    /* Use a simple bitset for deduplication (up to n_ledges) */
    int count = 0;

    /* Simple O(n) dedup: mark seen indices */
    /* For large ledge counts, a proper bitset would be better.
     * Here we use a linear scan since max_out is typically small. */
    for (int cz = c_min[2]; cz <= c_max[2]; cz++) {
        for (int cy = c_min[1]; cy <= c_max[1]; cy++) {
            for (int cx = c_min[0]; cx <= c_max[0]; cx++) {
                int ci = cell_index(sm, cx, cy, cz);
                const ivp_surman_grid_cell_t *cell = &sm->cells[ci];
                for (int i = 0; i < cell->count; i++) {
                    int idx = cell->ledge_indices[i];
                    /* Check if already in output */
                    bool dup = false;
                    for (int j = 0; j < count; j++) {
                        if (out_indices[j] == idx) { dup = true; break; }
                    }
                    if (!dup && count < max_out) {
                        out_indices[count++] = idx;
                    }
                }
            }
        }
    }

    return count;
}

/* ── Reference compact grid surface manager ───────────────────────────── */

#include <ivp/geometry/ivp_compact_ledge_geom.h>
#include "ivp_check_ledge_debug.h"
#include <float.h>

/* IVP_SurfaceManager_Grid::traverse_grid (ivp_surman_grid.cxx:16) */
static void cg_traverse(const ivp_surman_compact_grid_t *sm, const double pos_os[3], double radius,
                        ivp_surman_ledge_fn add_fn, void *ctx)
{
    const ivp_compact_grid_t *g = sm->compact_grid;
    double gs[3];
    ivp_compact_grid_to_grid_space(g, pos_os, gs);
    double radius_gs = radius * (double)g->inv_grid_size;
    double fmin_row = gs[0] - radius_gs;
    double fmax_row = gs[0] + radius_gs;
    double fmin_col = gs[1] - radius_gs;
    double fmax_col = gs[1] + radius_gs;
    if (fmin_row < 0) fmin_row = 0;
    if (fmin_col < 0) fmin_col = 0;
    if (fmax_row >= (double)g->n_rows) fmax_row = (double)((float)g->n_rows - 0.5f);
    if (fmax_col >= (double)g->n_columns) fmax_col = (double)((float)g->n_columns - 0.5f);
    /* no square touched (also keeps the int conversions defined) */
    if (fmax_row < -1.0 || fmax_col < -1.0) return;
    if (fmin_row > (double)g->n_rows || fmin_col > (double)g->n_columns) return;
    int min_row = (int)fmin_row, min_col = (int)fmin_col;
    int max_row = (int)fmax_row, max_col = (int)fmax_col;

    unsigned char flag_buffer[8192];
    unsigned char *flags = flag_buffer;
    const ivp_allocator_t *alloc = &sm->allocator;
    size_t flags_sz = (size_t)g->n_compact_ledges;
    if (g->n_compact_ledges >= 8192) {
        flags = (unsigned char *)ivp_alloc(alloc, flags_sz);
        if (!flags) return;
    }
    memset(flags, 0, flags_sz);

    const ivp_compact_grid_element_t *ge = ivp_compact_grid_get_elements(g);
    for (int row = min_row; row <= max_row; row++) {
        for (int col = min_col; col <= max_col; col++) {
            int idx = row * g->n_columns + col;
            for (int i = 0; i < 2; i++) {
                int ledge_id = ge[idx].compact_ledge_index[i];
                if (ledge_id < 0) continue; /* flagged as not existent */
                if (ledge_id >= g->n_compact_ledges) {
                    IVP_ASSERT(ledge_id < g->n_compact_ledges);
                    continue;
                }
                if (flags[ledge_id]) continue;
                flags[ledge_id] = 1;
                const ivp_compact_ledge_t *ledge = ivp_compact_grid_get_ledge(g, ledge_id);
#ifdef IVP_DEBUG_BUILD
                ivp_debug_check_ledge(ledge);
#endif
                if (ivp_ledge_get_n_triangles(ledge) == 2) {
                    double qdist = ivp_ledge_calc_qlen_pf(ledge, ivp_ledge_get_first_triangle(ledge), pos_os);
                    if (qdist > radius * radius) continue;
                }
                add_fn(ctx, ledge);
            }
        }
    }
    if (flags != flag_buffer) ivp_free(alloc, flags, flags_sz);
}

static const ivp_compact_ledge_t *cg_single_convex(const ivp_surman_t *sm)
{
    (void)sm;
    return NULL;
}

static void cg_mass_center(const ivp_surman_t *sm, float out[3])
{
    const ivp_compact_grid_t *g = ((const ivp_surman_compact_grid_t *)sm)->compact_grid;
    out[0] = g->center[0];
    out[1] = g->center[1];
    out[2] = g->center[2];
}

static void cg_rotation_inertia(const ivp_surman_t *sm, float out[3])
{
    const ivp_compact_grid_t *g = ((const ivp_surman_compact_grid_t *)sm)->compact_grid;
    double r = (double)g->radius * (double)g->radius;
    out[0] = out[1] = out[2] = (float)r;
}

static void cg_radius_and_deviation(const ivp_surman_t *sm, const float center[3],
                                    float *out_radius, float *out_deviation)
{
    const ivp_compact_grid_t *g = ((const ivp_surman_compact_grid_t *)sm)->compact_grid;
    double dx = (double)g->center[0] - (double)center[0];
    double dy = (double)g->center[1] - (double)center[1];
    double dz = (double)g->center[2] - (double)center[2];
    double shift = sqrt(dx * dx + dy * dy + dz * dz);
    *out_radius = (float)((double)g->radius + shift);
    *out_deviation = *out_radius; /* grid has no separate deviation metric */
}

static ivp_surman_type_t cg_type(const ivp_surman_t *sm)
{
    (void)sm;
    return IVP_SURMAN_POLYGON;
}

static void cg_destroy(ivp_surman_t *sm)
{
    (void)sm;
}

static void cg_ledges_within_radius(const ivp_surman_t *sm, const double pos[3], double radius,
                                    const ivp_compact_ledge_t *root_ledge,
                                    const ivp_compact_ledge_t *other_reference_ledge,
                                    ivp_surman_ledge_fn add_fn, void *ctx)
{
    (void)root_ledge;
    (void)other_reference_ledge;
    if (!add_fn) return;
    cg_traverse((const ivp_surman_compact_grid_t *)sm, pos, radius, add_fn, ctx);
}

static void cg_terminal_ledges(const ivp_surman_t *sm, ivp_surman_ledge_fn fn, void *ctx)
{
    if (!fn) return;
    const double zero[3] = { 0.0, 0.0, 0.0 };
    cg_traverse((const ivp_surman_compact_grid_t *)sm, zero, DBL_MAX, fn, ctx);
}

typedef struct cg_ray_ctx {
    const ivp_allocator_t *allocator;
    const ivp_compact_ledge_t *l[256];
    int n;
    const ivp_compact_ledge_t **heap;
    int heap_n;
    int heap_cap;
    bool fail;
} cg_ray_ctx_t;

static void cg_collect(void *c, const ivp_compact_ledge_t *ledge)
{
    cg_ray_ctx_t *rc = (cg_ray_ctx_t *)c;
    if (rc->n < 256) {
        rc->l[rc->n++] = ledge;
        return;
    }
    if (rc->heap_n >= rc->heap_cap) {
        int nc = rc->heap_cap ? rc->heap_cap * 2 : 256;
        const ivp_compact_ledge_t **nh = (const ivp_compact_ledge_t **)ivp_alloc(
            rc->allocator, (size_t)nc * sizeof(*nh));
        if (!nh) {
            rc->fail = true;
            return;
        }
        if (rc->heap) {
            memcpy(nh, rc->heap, (size_t)rc->heap_n * sizeof(*nh));
            ivp_free(rc->allocator, (void *)rc->heap,
                     (size_t)rc->heap_cap * sizeof(*nh));
        }
        rc->heap = nh;
        rc->heap_cap = nc;
    }
    rc->heap[rc->heap_n++] = ledge;
}

static void cg_insert_ray(const ivp_surman_t *sm, struct ivp_ray_solver *ray_solver,
                          struct ivp_object *object)
{
    /* ivp_surman_grid.cxx:108 */
    ivp_ray_solver_os_t os;
    ivp_ray_solver_os_init(&os, ray_solver, object);
    double center[3] = { (double)os.ray_center_point[0], (double)os.ray_center_point[1],
                         (double)os.ray_center_point[2] };
    cg_ray_ctx_t rc;
    memset(&rc, 0, sizeof(rc));
    rc.allocator = &((const ivp_surman_compact_grid_t *)sm)->allocator;
    cg_traverse((const ivp_surman_compact_grid_t *)sm, center,
                (double)(os.ray_length * 0.5f), cg_collect, &rc);
    int total = rc.n + rc.heap_n;
    for (int i = total - 1; i >= 0; i--) {
        const ivp_compact_ledge_t *l = i < rc.n ? rc.l[i] : rc.heap[i - rc.n];
        ivp_ray_solver_os_check_compact_ledge(&os, l);
    }
    if (rc.heap) {
        ivp_free(rc.allocator, (void *)rc.heap,
                 (size_t)rc.heap_cap * sizeof(*rc.heap));
    }
}

static const ivp_surman_vtable_t s_compact_grid_vtable = {
    .get_single_convex = cg_single_convex,
    .get_mass_center = cg_mass_center,
    .get_rotation_inertia = cg_rotation_inertia,
    .get_radius_and_deviation = cg_radius_and_deviation,
    .get_type = cg_type,
    .get_ledgetree_root = NULL,
    .destroy = cg_destroy,
    .get_all_ledges_within_radius = cg_ledges_within_radius,
    .get_all_terminal_ledges = cg_terminal_ledges,
    .insert_all_ledges_hitting_ray = cg_insert_ray
};

void ivp_surman_compact_grid_init(ivp_surman_compact_grid_t *sm, const ivp_compact_grid_t *grid)
{
    ivp_surman_compact_grid_init_with_allocator(sm, grid, NULL);
}

void ivp_surman_compact_grid_init_with_allocator(
    ivp_surman_compact_grid_t *sm, const ivp_compact_grid_t *grid,
    const ivp_allocator_t *allocator)
{
    sm->base.vtable = &s_compact_grid_vtable;
    sm->compact_grid = grid;
    sm->allocator = *(allocator ? allocator : ivp_default_allocator());
}
