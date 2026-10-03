/* ivp_build_compact_recursive.h — Recursive (hull) compact ledges
 *
 * Computes a single convex hull that envelops all vertices of a set of
 * compact ledges and marks every hull triangle / edge that does not exist
 * in the input ledges as virtual.  In the reference engine this hull is
 * stored as the compact hull of the root ledgetree node
 * (IVP_SurfaceBuilder_Ledge_Soup::build_root_convex_hull), which makes
 * IVP_SurfaceManager_Polygon::get_single_convex() return it and lets
 * IVP_Mindist_Recursive test the hull before descending.  Use
 * ivp_ledge_soup_compile_ex() with build_root_convex_hull = true, or
 * ivp_build_compound_with_root_hull() below, to get that layout.
 *
 * Ported from ivp_compact_builder/ivp_compact_recursive.cxx.
 */
#ifndef IVP_BUILD_COMPACT_RECURSIVE_H
#define IVP_BUILD_COMPACT_RECURSIVE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/geometry/ivp_compact_ledge.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IVP_Compact_Recursive::compile(): convex hull (pointsoup) of all distinct
 * vertices of `ledges` (visited in reverse list order, like the reference)
 * followed by set_rekursive_convex_hull().  Returns a new ledge (free with
 * ivp_compact_ledge_free()) or NULL. */
ivp_compact_ledge_t *ivp_compact_recursive_build_hull(const ivp_allocator_t *alloc,
                                                      const ivp_compact_ledge_t *const *ledges,
                                                      int n_ledges);

/* IVP_Compact_Recursive::set_rekursive_convex_hull(): mark hull triangles
 * that are not triangles of any input ledge (matched by point identity)
 * and hull edges that are not input edges as virtual. */
void ivp_compact_recursive_set_virtual_flags(ivp_compact_ledge_t *hull,
                                             const ivp_compact_ledge_t *const *ledges,
                                             int n_ledges);

/* Build a compound surface from `ledges` whose root ledgetree node carries
 * the recursive convex hull (ledge soup with build_root_convex_hull).
 * Input ledges are copied, not freed.  For a single ledge no hull is
 * generated (reference: only when c_ledge_vec.len() > 1). */
ivp_compact_surface_t *ivp_build_compound_with_root_hull(const ivp_allocator_t *alloc,
                                                         const ivp_compact_ledge_t **ledges,
                                                         int n_ledges);

/* Convenience: the convex hull of all ledges as a separate single-ledge
 * compact surface (not part of the reference API).  Caller frees with
 * ivp_compact_surface_free(). */
ivp_compact_surface_t *ivp_build_root_convex_hull(
    const ivp_allocator_t *alloc,
    const ivp_compact_ledge_t **ledges,
    int n_ledges);

/* Extract all unique vertices (bitwise float identity) from a set of
 * compact ledges into a flat float array.  Caller frees *out_coords with
 * ivp_free(alloc, ptr, count * 3 * sizeof(float)).
 * Returns 0 on success, -1 on failure. */
int ivp_extract_ledge_vertices(
    const ivp_compact_ledge_t **ledges,
    int n_ledges,
    float **out_coords,
    int *out_count,
    const ivp_allocator_t *alloc);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BUILD_COMPACT_RECURSIVE_H */
