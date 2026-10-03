/* ivp_build_halfspacesoup_internal.h -- half-space soup -> compact ledge
 *
 * IVP_SurfaceBuilder_Halfspacesoup::convert_halfspacesoup_to_compact_ledge
 * (ivp_surbuild_halfspacesoup.cxx:195): the intersection points of the
 * half-spaces (convert_halfspacesoup_to_points) through the pointsoup
 * convex builder (convert_pointsoup_to_compact_ledge).  Unlike
 * convert_halfspacesoup_to_compact_surface (ivp_build_halfspacesoup_surface)
 * it prints nothing when no ledge can be built; the reference's
 * IVP_Compact_Modify::shrink and IVP_SurfaceBuilder_Q12 use this one.
 */
#ifndef IVP_BUILD_HALFSPACESOUP_INTERNAL_H
#define IVP_BUILD_HALFSPACESOUP_INTERNAL_H

#include <ivp/builder/ivp_build_halfspacesoup.h>
#include <ivp/geometry/ivp_compact_ledge.h>

/* Returns a new compact ledge (free with ivp_compact_ledge_free()) or NULL.
 * merge_threshold <= 0 selects the default 0.01 of
 * ivp_build_halfspacesoup_surface(). */
ivp_compact_ledge_t *ivp_hs_halfspacesoup_to_ledge(const ivp_allocator_t *alloc,
                                                   const ivp_halfspace_t *halfspaces,
                                                   int n_halfspaces,
                                                   float merge_threshold);

#endif /* IVP_BUILD_HALFSPACESOUP_INTERNAL_H */
