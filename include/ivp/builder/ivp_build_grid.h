/* ivp_build_grid.h — Grid/heightfield surface builder
 *
 * Converts a 2D heightfield array into a compact surface composed of
 * triangle wedge ledges. Optionally wraps the result in a grid surface
 * manager for O(1) spatial queries.
 *
 * Ported from ivp_surface_manager/ivp_gridbuild_array.hxx.
 */
#ifndef IVP_BUILD_GRID_H
#define IVP_BUILD_GRID_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/geometry/ivp_surman_grid.h>
#include <ivp/geometry/ivp_compact_grid.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Grid configuration ────────────────────────────────────────────────── */

typedef struct ivp_grid_config {
    int   n_rows;         /* default: 9 */
    int   n_columns;      /* default: 9 */
    float field_size;     /* meters between grid points, default: 1.0 */
    int   height_axis;    /* 0=X, 1=Y (default), 2=Z */
    float origin[3];      /* world-space origin, default: {0,0,0} */
} ivp_grid_config_t;

/* Fill with defaults. */
void ivp_grid_config_init(ivp_grid_config_t *cfg);

/* ── Building ──────────────────────────────────────────────────────────── */

/* Build a compact surface from a heightfield array.
 *
 * height_field: flat array of [n_rows * n_columns] float heights.
 *               height_field[row * n_columns + col] = height at (row, col).
 * cfg may be NULL for defaults.
 * Returns a newly allocated compact surface, or NULL on failure. */
ivp_compact_surface_t *ivp_build_grid_surface(const ivp_allocator_t *alloc,
                                              const ivp_grid_config_t *cfg,
                                              const float *height_field);

/* Build a grid surface and wrap it in a grid surface manager.
 *
 * Convenience function that combines ivp_build_grid_surface() with
 * ivp_surman_grid_init(). grid_cfg may be NULL for defaults.
 * Returns IVP_OK on success. surman_out must be freed by caller. */
ivp_result_t ivp_build_grid_with_surman(const ivp_allocator_t *alloc,
                                        const ivp_grid_config_t *cfg,
                                        const float *height_field,
                                        ivp_surman_grid_t *surman_out,
                                        const ivp_surman_grid_config_t *grid_cfg);

/* ── Reference compact grid (IVP_GridBuilder_Array) ───────────────────── */

/* IVP_Template_Grid_Axle_Descript */
typedef struct ivp_template_grid_axle_descript {
    int  n_points;     /* number of points (not fields) along this direction */
    int  maps_to;      /* object space axis (0..2) of this array direction */
    bool invert_axis;  /* true: maps to the negative axis direction */
} ivp_template_grid_axle_descript_t;

/* IVP_Template_Compact_Grid */
typedef struct ivp_template_compact_grid {
    ivp_template_grid_axle_descript_t row_info;
    ivp_template_grid_axle_descript_t column_info;
    int   height_maps_to;          /* object space axis of the height values */
    bool  height_invert_axis;
    float grid_field_size;         /* distance between two grid points */
    float position_origin_os[3];   /* position of height_field[0] */
} ivp_template_compact_grid_t;

void ivp_template_compact_grid_init(ivp_template_compact_grid_t *templ);

/* IVP_GridBuilder_Array::convert_array_to_compact_grid
 * (ivp_gridbuild_array.cxx:983).  height_field has row_info.n_points rows of
 * column_info.n_points values (at most IVP_GRID_MAX_ROWS x
 * IVP_GRID_MAX_COLUMNS).  Free the result with ivp_compact_grid_free().
 * Use with ivp_surman_compact_grid_t (ivp_surman_grid.h). */
ivp_compact_grid_t *ivp_gridbuild_array_to_compact_grid(const ivp_allocator_t *alloc,
                                                        const ivp_template_compact_grid_t *templ,
                                                        const float *height_field);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BUILD_GRID_H */
