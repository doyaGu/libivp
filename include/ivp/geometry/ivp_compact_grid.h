/* ivp_compact_grid.h — Compact grid (heightfield terrain)
 *
 * C counterpart of the reference IVP_Compact_Grid
 * (ivp_surface_manager/ivp_compact_grid.hxx), byte-identical to the
 * reference blob on LP64 (IVP_GridBuilder_Array::
 * compile_ledges_into_compact_grid):
 *   header (176 bytes, offsets as in the C++ class)
 *   int32 offset_compact_ledge_array[n_compact_ledges]
 *   (pad to 16)                                  offset_grid_elements
 *   ivp_compact_grid_element_t [n_rows * n_columns]   two ledge indices per
 *                                                      grid square
 *   ivp_compact_poly_point_t   shared point array (directly after the
 *                              elements: only 4-byte aligned, like the
 *                              reference)
 *   (pad to 16)
 *   non-compact compact ledges (header + triangles; their c_point_offset
 *   points into the shared point array)
 *   (byte_size keeps the reference's worst-case padding estimate, so the
 *   blob may end with up to 15 unused bytes)
 * Bytes the reference leaves uninitialised (header padding, alignment gaps,
 * the hesse_val of the grid points, the unused tail) are zero here.
 * Build with ivp_gridbuild_array_to_compact_grid() (ivp_build_grid.h), free
 * with ivp_compact_grid_free().
 */
#ifndef IVP_COMPACT_GRID_H
#define IVP_COMPACT_GRID_H

#include <stddef.h>
#include <stdint.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/base/ivp_allocator.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reference IVP_GRID_MAX_ROWS / IVP_GRID_MAX_COLUMNS */
#define IVP_GRID_MAX_ROWS    257
#define IVP_GRID_MAX_COLUMNS IVP_GRID_MAX_ROWS

typedef struct ivp_compact_grid_element {
    int16_t compact_ledge_index[2];   /* max two ledges per square, < 0: none */
} ivp_compact_grid_element_t;

typedef struct IVP_ALIGN16 ivp_compact_grid {
    /* IVP_U_Float_Hesse center: bounding sphere center (object space),
     * center[3] (hesse_val) unused */
    float    center[4];
    /* IVP_U_Matrix m_grid_f_object (object -> grid, field size 1): three
     * IVP_U_Point rows and the IVP_U_Point vv, each IVP_DOUBLE k[3] plus the
     * unused IVP_DOUBLE hesse_val */
    double   m_grid_f_object_rot[3][4];
    double   m_grid_f_object_pos[4];
    int16_t  n_rows;                   /* number of grid squares per column */
    int16_t  n_columns;                /* number of grid squares per row */
    int32_t  n_compact_ledges;
    float    radius;                   /* bounding sphere radius */
    intptr_t byte_size;                /* reference intp */
    float    inv_grid_size;
    int32_t  offset_grid_elements;
    int32_t  offset_compact_ledge_array[]; /* [n_compact_ledges] */
} ivp_compact_grid_t;

/* reference IVP_Compact_Grid offsets (LP64) */
_Static_assert(offsetof(ivp_compact_grid_t, m_grid_f_object_rot) == 16, "IVP_Compact_Grid::m_grid_f_object");
_Static_assert(sizeof(void *) != 8 || offsetof(ivp_compact_grid_t, n_rows) == 144, "IVP_Compact_Grid::n_rows");
_Static_assert(sizeof(void *) != 8 || offsetof(ivp_compact_grid_t, byte_size) == 160, "IVP_Compact_Grid::byte_size");
_Static_assert(sizeof(void *) != 8 || offsetof(ivp_compact_grid_t, offset_compact_ledge_array) == 176,
               "IVP_Compact_Grid::offset_compact_ledge_array");

static inline const ivp_compact_grid_element_t *ivp_compact_grid_get_elements(
    const ivp_compact_grid_t *g)
{
    return (const ivp_compact_grid_element_t *)((const char *)g + g->offset_grid_elements);
}

static inline const ivp_compact_ledge_t *ivp_compact_grid_get_ledge(const ivp_compact_grid_t *g,
                                                                    int i)
{
    return (const ivp_compact_ledge_t *)((const char *)g + g->offset_compact_ledge_array[i]);
}

/* object space point -> grid space (m_grid_f_object.vmult4) */
static inline void ivp_compact_grid_to_grid_space(const ivp_compact_grid_t *g,
                                                  const double p_os[3], double out[3])
{
    for (int r = 0; r < 3; r++) {
        out[r] = g->m_grid_f_object_rot[r][0] * p_os[0] + g->m_grid_f_object_rot[r][1] * p_os[1] +
                 g->m_grid_f_object_rot[r][2] * p_os[2] + g->m_grid_f_object_pos[r];
    }
}

void ivp_compact_grid_free(const ivp_allocator_t *alloc, ivp_compact_grid_t *grid);

#ifdef __cplusplus
}
#endif

#endif /* IVP_COMPACT_GRID_H */
