/* ivp_surman_grid.h -- Grid-based surface manager for terrain
 *
 * Subdivides a compact surface (ledge soup) into a uniform grid of cells
 * for O(1) spatial lookup. Each cell stores indices of ledges whose
 * bounding spheres overlap that cell.
 *
 * Use for large static terrain where broadphase against hundreds of
 * ledges would be too slow.
 *
 * Ported from ivp_surman_grid.hxx.
 */
#ifndef IVP_SURMAN_GRID_H
#define IVP_SURMAN_GRID_H

#include <ivp/geometry/ivp_surman.h>
#include <ivp/base/ivp_allocator.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Grid configuration ────────────────────────────────────────────────── */

typedef struct ivp_surman_grid_config {
    int   cells_x;    /* grid resolution along X (default: 16) */
    int   cells_y;    /* grid resolution along Y (default: 4)  */
    int   cells_z;    /* grid resolution along Z (default: 16) */
} ivp_surman_grid_config_t;

static inline void ivp_surman_grid_config_init(ivp_surman_grid_config_t *cfg) {
    cfg->cells_x = 16;
    cfg->cells_y = 4;
    cfg->cells_z = 16;
}

/* ── Grid cell ─────────────────────────────────────────────────────────── */

typedef struct ivp_surman_grid_cell {
    int  *ledge_indices;   /* array of ledge indices in this cell */
    int   count;           /* number of ledges overlapping this cell */
    int   capacity;        /* allocated capacity */
} ivp_surman_grid_cell_t;

/* ── Grid surface manager ──────────────────────────────────────────────── */

typedef struct ivp_surman_grid {
    ivp_surman_t            base;
    const ivp_compact_surface_t *compact_surface;

    /* Grid dimensions */
    int     cells_x, cells_y, cells_z;
    int     total_cells;

    /* World-space AABB of the surface */
    float   aabb_min[3];
    float   aabb_max[3];
    float   cell_size[3];      /* size of each cell */
    float   inv_cell_size[3];  /* 1.0 / cell_size */

    /* Flat array of cells: index = z * (cells_x * cells_y) + y * cells_x + x */
    ivp_surman_grid_cell_t *cells;

    /* Flat array of all ledge pointers (for index-based access) */
    const ivp_compact_ledge_t **ledges;
    int   n_ledges;

    /* Allocator used for construction */
    const ivp_allocator_t  *allocator;
} ivp_surman_grid_t;

/* ── Construction / Destruction ────────────────────────────────────────── */

/* Build a grid surface manager from a compact surface (ledge soup).
 * The compact surface must outlive the grid surman.
 * config may be NULL to use defaults.
 * Returns IVP_OK on success. */
ivp_result_t ivp_surman_grid_init(ivp_surman_grid_t *sm,
                                    const ivp_compact_surface_t *surface,
                                    const ivp_surman_grid_config_t *config,
                                    const ivp_allocator_t *alloc);

/* Free all internal allocations. */
void ivp_surman_grid_destroy(ivp_surman_grid_t *sm);

/* ── Spatial query ─────────────────────────────────────────────────────── */

/* Get all ledge indices overlapping the cell at world position (x,y,z).
 * Returns count; out_indices points into internal storage (valid until
 * grid is destroyed). Returns 0 if position is outside the grid. */
int ivp_surman_grid_query_point(const ivp_surman_grid_t *sm,
                                  float x, float y, float z,
                                  const int **out_indices);

/* Get all ledge indices overlapping cells that intersect the given AABB.
 * Fills out_indices (caller-owned buffer of max_out capacity).
 * Returns number of unique ledge indices written. */
int ivp_surman_grid_query_aabb(const ivp_surman_grid_t *sm,
                                 const float aabb_min[3],
                                 const float aabb_max[3],
                                 int *out_indices,
                                 int max_out);

/* Get a ledge by index. */
static inline const ivp_compact_ledge_t *ivp_surman_grid_get_ledge(
    const ivp_surman_grid_t *sm, int index)
{
    if (index < 0 || index >= sm->n_ledges) return NULL;
    return sm->ledges[index];
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_SURMAN_GRID_H */
