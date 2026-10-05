/* ivp_build_grid.c — Grid/heightfield surface builder
 *
 * Maps a 2D heightfield array to a 3D vertex grid, triangulates each
 * cell into 2 triangles, and builds a compound compact surface.
 *
 * Ported from ivp_surface_manager/ivp_gridbuild_array.hxx.
 */

#include <ivp/builder/ivp_build_grid.h>
#include <ivp/builder/ivp_build_concave.h>
#include "ivp_build_checked.h"
#include <string.h>

void ivp_grid_config_init(ivp_grid_config_t *cfg)
{
    cfg->n_rows      = 9;
    cfg->n_columns   = 9;
    cfg->field_size  = 1.0f;
    cfg->height_axis = 1;   /* Y */
    cfg->origin[0]   = 0.0f;
    cfg->origin[1]   = 0.0f;
    cfg->origin[2]   = 0.0f;
}

ivp_compact_surface_t *ivp_build_grid_surface(const ivp_allocator_t *alloc,
                                              const ivp_grid_config_t *cfg,
                                              const float *height_field)
{
    ivp_grid_config_t defaults;
    if (!cfg) {
        ivp_grid_config_init(&defaults);
        cfg = &defaults;
    }

    if (!height_field) return NULL;
    if (!alloc) alloc = ivp_default_allocator();
    if (cfg->n_rows < 2 || cfg->n_columns < 2) return NULL;

    int rows = cfg->n_rows;
    int cols = cfg->n_columns;
    int n_verts;
    int n_quads;
    if (!ivp_build_int_mul(rows, cols, &n_verts) ||
        !ivp_build_int_mul(rows - 1, cols - 1, &n_quads)) return NULL;

    /* Determine axis mapping:
     * height_axis = Y(1): row→X, col→Z, height→Y
     * height_axis = X(0): row→Y, col→Z, height→X
     * height_axis = Z(2): row→X, col→Y, height→Z */
    int row_axis, col_axis, h_axis;
    h_axis = cfg->height_axis;
    if (h_axis < 0 || h_axis > 2) h_axis = 1;
    switch (h_axis) {
        case 0: row_axis = 1; col_axis = 2; break;
        case 2: row_axis = 0; col_axis = 1; break;
        default: row_axis = 0; col_axis = 2; break;
    }

    /* Create a thin slab mesh: duplicate grid vertices at a small offset
     * below the surface to give the mesh thickness (QHull needs 3D volume). */
    float thickness = cfg->field_size * 0.01f;
    if (thickness < 0.001f) thickness = 0.001f;

    int n_verts_total;
    int n_tris_top;
    int n_tris_sides_rows;
    int n_tris_sides_cols;
    int n_tris_sides;
    int n_tris_faces;
    int n_tris_total;
    int n_vert_scalars;
    int n_index_scalars;
    size_t vert_sz;
    size_t idx_sz;
    if (!ivp_build_int_mul(n_verts, 2, &n_verts_total) ||
        !ivp_build_int_mul(n_quads, 2, &n_tris_top) ||
        !ivp_build_int_mul(rows - 1, 4, &n_tris_sides_rows) ||
        !ivp_build_int_mul(cols - 1, 4, &n_tris_sides_cols) ||
        !ivp_build_int_add(n_tris_sides_rows, n_tris_sides_cols, &n_tris_sides) ||
        !ivp_build_int_add(n_tris_top, n_tris_top, &n_tris_faces) ||
        !ivp_build_int_add(n_tris_faces, n_tris_sides, &n_tris_total) ||
        !ivp_build_int_mul(n_verts_total, 3, &n_vert_scalars) ||
        !ivp_build_int_mul(n_tris_total, 3, &n_index_scalars) ||
        !ivp_build_count_bytes(n_vert_scalars, sizeof(float), &vert_sz) ||
        !ivp_build_count_bytes(n_index_scalars, sizeof(int), &idx_sz)) return NULL;

    float *verts = (float *)ivp_alloc(alloc, vert_sz);
    int   *inds  = (int *)ivp_alloc(alloc, idx_sz);
    if (!verts || !inds) {
        if (verts) ivp_free(alloc, verts, vert_sz);
        if (inds)  ivp_free(alloc, inds, idx_sz);
        return NULL;
    }

    /* Generate top-layer vertices (indices 0..n_verts-1) */
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            int vi = (r * cols + c) * 3;
            float pos[3];
            pos[row_axis] = cfg->origin[row_axis] + (float)r * cfg->field_size;
            pos[col_axis] = cfg->origin[col_axis] + (float)c * cfg->field_size;
            pos[h_axis]   = cfg->origin[h_axis] + height_field[r * cols + c];
            verts[vi + 0] = pos[0];
            verts[vi + 1] = pos[1];
            verts[vi + 2] = pos[2];
        }
    }

    /* Generate bottom-layer vertices (indices n_verts..2*n_verts-1) */
    for (int i = 0; i < n_verts; i++) {
        int si = i * 3;
        int di = (n_verts + i) * 3;
        verts[di + 0] = verts[si + 0];
        verts[di + 1] = verts[si + 1];
        verts[di + 2] = verts[si + 2];
        verts[di + h_axis] -= thickness;
    }

    /* Triangulate top face */
    int ti = 0;
    for (int r = 0; r < rows - 1; r++) {
        for (int c = 0; c < cols - 1; c++) {
            int v00 = r * cols + c;
            int v10 = (r + 1) * cols + c;
            int v01 = r * cols + (c + 1);
            int v11 = (r + 1) * cols + (c + 1);

            if ((r + c) % 2 == 0) {
                inds[ti++] = v00; inds[ti++] = v10; inds[ti++] = v11;
                inds[ti++] = v00; inds[ti++] = v11; inds[ti++] = v01;
            } else {
                inds[ti++] = v00; inds[ti++] = v10; inds[ti++] = v01;
                inds[ti++] = v10; inds[ti++] = v11; inds[ti++] = v01;
            }
        }
    }

    /* Triangulate bottom face (reversed winding) */
    for (int r = 0; r < rows - 1; r++) {
        for (int c = 0; c < cols - 1; c++) {
            int v00 = n_verts + r * cols + c;
            int v10 = n_verts + (r + 1) * cols + c;
            int v01 = n_verts + r * cols + (c + 1);
            int v11 = n_verts + (r + 1) * cols + (c + 1);

            if ((r + c) % 2 == 0) {
                inds[ti++] = v00; inds[ti++] = v11; inds[ti++] = v10;
                inds[ti++] = v00; inds[ti++] = v01; inds[ti++] = v11;
            } else {
                inds[ti++] = v00; inds[ti++] = v01; inds[ti++] = v10;
                inds[ti++] = v10; inds[ti++] = v01; inds[ti++] = v11;
            }
        }
    }

    /* Triangulate perimeter sides: row edges (r=0 and r=rows-1) */
    for (int c = 0; c < cols - 1; c++) {
        /* r=0 edge: top row → bottom row */
        int t0 = c, t1 = c + 1;
        int b0 = n_verts + c, b1 = n_verts + c + 1;
        inds[ti++] = t0; inds[ti++] = b1; inds[ti++] = b0;
        inds[ti++] = t0; inds[ti++] = t1; inds[ti++] = b1;

        /* r=rows-1 edge */
        t0 = (rows - 1) * cols + c; t1 = (rows - 1) * cols + c + 1;
        b0 = n_verts + (rows - 1) * cols + c; b1 = n_verts + (rows - 1) * cols + c + 1;
        inds[ti++] = t0; inds[ti++] = b0; inds[ti++] = b1;
        inds[ti++] = t0; inds[ti++] = b1; inds[ti++] = t1;
    }

    /* Column edges (c=0 and c=cols-1) */
    for (int r = 0; r < rows - 1; r++) {
        /* c=0 edge */
        int t0 = r * cols, t1 = (r + 1) * cols;
        int b0 = n_verts + r * cols, b1 = n_verts + (r + 1) * cols;
        inds[ti++] = t0; inds[ti++] = b0; inds[ti++] = b1;
        inds[ti++] = t0; inds[ti++] = b1; inds[ti++] = t1;

        /* c=cols-1 edge */
        t0 = r * cols + cols - 1; t1 = (r + 1) * cols + cols - 1;
        b0 = n_verts + r * cols + cols - 1; b1 = n_verts + (r + 1) * cols + cols - 1;
        inds[ti++] = t0; inds[ti++] = b1; inds[ti++] = b0;
        inds[ti++] = t0; inds[ti++] = t1; inds[ti++] = b1;
    }

    /* Build compound surface via concave builder.
     * Use force_single_hull for the thin slab to avoid decomposition issues. */
    ivp_concave_options_t opts = ivp_concave_options_default();
    opts.force_single_hull = true;
    ivp_compact_surface_t *surface = ivp_build_concave_surface(
        alloc, verts, n_verts_total, inds, ti / 3, &opts);

    ivp_free(alloc, verts, vert_sz);
    ivp_free(alloc, inds, idx_sz);

    return surface;
}

ivp_result_t ivp_build_grid_with_surman(const ivp_allocator_t *alloc,
                                        const ivp_grid_config_t *cfg,
                                        const float *height_field,
                                        ivp_surman_grid_t *surman_out,
                                        const ivp_surman_grid_config_t *grid_cfg)
{
    if (!surman_out) return IVP_ERR_INVALID_ARG;
    if (!alloc) alloc = ivp_default_allocator();

    ivp_compact_surface_t *surface = ivp_build_grid_surface(alloc, cfg, height_field);
    if (!surface) return IVP_ERR_DEGENERATE_GEOMETRY;

    ivp_result_t result = ivp_surman_grid_init(surman_out, surface, grid_cfg, alloc);
    if (!ivp_ok(result)) {
        ivp_compact_surface_free(alloc, surface);
        return result;
    }

    return IVP_OK;
}
