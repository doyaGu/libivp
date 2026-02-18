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

#ifdef __cplusplus
}
#endif

#endif /* IVP_BUILD_GRID_H */
