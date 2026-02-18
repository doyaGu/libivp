/* ivp_build_ledge_soup.h -- Ledge soup builder for compound surfaces
 *
 * Combines multiple convex compact ledges into a single (possibly concave)
 * compact surface with a bounding-volume hierarchy (ledgetree).
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
 * Input ledges are NOT freed by the builder.
 *
 * Ported from ivp_surbuild_ledge_soup.hxx / ivp_surbuild_ledge_soup.cxx.
 */
#ifndef IVP_BUILD_LEDGE_SOUP_H
#define IVP_BUILD_LEDGE_SOUP_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_surface.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum ledges supported in a single soup. */
#define IVP_LEDGE_SOUP_MAX_LEDGES 65536

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

/* Compile all added ledges into a single compact surface.
 * Returns a newly allocated compact surface, or NULL on failure.
 * The caller must free with ivp_compact_surface_free() or ivp_free_aligned().
 *
 * After compile, the builder can be destroyed. Input ledges are not freed. */
ivp_compact_surface_t *ivp_ledge_soup_compile(ivp_ledge_soup_t *soup);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BUILD_LEDGE_SOUP_H */
