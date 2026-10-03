/* ivp_build_ledge_soup.h -- Ledge soup builder for compound surfaces
 *
 * Combines one or more convex compact ledges into a single (possibly
 * concave) compact surface with a bounding-volume hierarchy (ledgetree).
 * Also used for single convex ledges: every reference builder
 * (pointsoup, halfspacesoup, polygon) wraps its ledge through the ledge soup
 * so that the surface header (mass center, unit-mass rotation inertia,
 * upper_limit_radius, max_factor_surface_deviation) is computed by
 * insert_radius_in_compact_surface().
 *
 * Usage:
 *   ivp_ledge_soup_t soup;
 *   ivp_ledge_soup_init(&soup, alloc);
 *   ivp_ledge_soup_add(&soup, ledge1);
 *   ivp_ledge_soup_add(&soup, ledge2);
 *   ...
 *   ivp_compact_surface_t *surface = ivp_ledge_soup_compile(&soup);
 *   ivp_ledge_soup_destroy(&soup);
 *
 * The compile step copies all ledge data into a contiguous memory block.
 *
 * Ported from ivp_surbuild_ledge_soup.hxx / ivp_surbuild_ledge_soup.cxx and
 * ivp_template_surbuild.hxx.
 */
#ifndef IVP_BUILD_LEDGE_SOUP_H
#define IVP_BUILD_LEDGE_SOUP_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_surface.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum ledges supported in a single soup (reference IVV_Sphere_Cluster
 * links are ushort). */
#define IVP_LEDGE_SOUP_MAX_LEDGES 65535

/* ── Template (IVP_Template_Surbuild_LedgeSoup) ────────────────────────── */

typedef enum ivp_ledge_soup_merge_points {
    IVP_SLMP_NO_MERGE = 0,          /* every ledge keeps its own points       */
    IVP_SLMP_MERGE_AND_REALLOCATE,  /* share equal points, shrink the block   */
    IVP_SLMP_MERGE_NO_REALLOCATE    /* share equal points, keep block size    */
} ivp_ledge_soup_merge_points_t;

typedef struct ivp_ledge_soup_template {
    /* Hull to insert at the root node instead of computing one (the copy
     * gets its virtual flags set like IVP_Compact_Recursive::
     * set_rekursive_convex_hull).  Only used with build_root_convex_hull.
     * The input is not modified or freed. */
    const ivp_compact_ledge_t *force_convex_hull;

    /* Build an extra convex hull around the whole object and store it as
     * the compact hull of the root ledgetree node (default false). */
    bool build_root_convex_hull;

    /* Free the input ledges with ivp_free_aligned(soup->alloc, ...) after
     * they have been copied.  Reference default is TRUE; the libivp default
     * is false because callers own their ledges. */
    bool free_input_compact_ledges;

    /* Reference option to reference the input ledges instead of copying
     * them.  Not supported (ignored: ledges are always copied). */
    bool link_to_input_compact_ledges;

    /* Point merging; default IVP_SLMP_NO_MERGE (reference default on every
     * non-WIN32 platform, ivp_template_surbuild.hxx:73). */
    ivp_ledge_soup_merge_points_t merge_points;
} ivp_ledge_soup_template_t;

void ivp_ledge_soup_template_init(ivp_ledge_soup_template_t *templ);

/* ── Builder state ─────────────────────────────────────────────────────── */

typedef struct ivp_ledge_soup {
    const ivp_allocator_t *alloc;
    const ivp_compact_ledge_t **ledges;  /* array of input ledge pointers */
    int n_ledges;
    int capacity;
} ivp_ledge_soup_t;

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

/* Initialize a ledge soup builder. */
void ivp_ledge_soup_init(ivp_ledge_soup_t *soup, const ivp_allocator_t *alloc);

/* Destroy builder and free internal storage. Does NOT free input ledges. */
void ivp_ledge_soup_destroy(ivp_ledge_soup_t *soup);

/* ── Building ──────────────────────────────────────────────────────────── */

/* Add a convex compact ledge to the soup. The ledge must remain valid
 * until compile() is called. Returns 0 on success, -1 on failure. */
int ivp_ledge_soup_add(ivp_ledge_soup_t *soup, const ivp_compact_ledge_t *ledge);

/* Compile all added ledges into a single compact surface with the default
 * template (ivp_ledge_soup_template_init).  Returns a newly allocated
 * compact surface, or NULL on failure.  Free with ivp_compact_surface_free().
 * The soup's ledge list is cleared afterwards (like the reference). */
ivp_compact_surface_t *ivp_ledge_soup_compile(ivp_ledge_soup_t *soup);

/* Compile with an explicit template (NULL = defaults). */
ivp_compact_surface_t *ivp_ledge_soup_compile_ex(ivp_ledge_soup_t *soup,
                                                 const ivp_ledge_soup_template_t *templ);

/* Recompute the surface header (mass center, rotation inertia,
 * upper_limit_radius, max_factor_surface_deviation) of an existing compact
 * surface exactly like IVP_SurfaceBuilder_Ledge_Soup::
 * insert_radius_in_compact_surface (ivp_surbuild_ledge_soup.cxx:1700). */
void ivp_ledge_soup_insert_radius(ivp_compact_surface_t *surface);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BUILD_LEDGE_SOUP_H */
