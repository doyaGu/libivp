/* ivp_compact_surface.h — Compact surface header (48 bytes, 16-byte aligned)
 *
 * The compact surface is the top-level data structure describing an object's
 * geometric shape. It is a header for a contiguous memory block containing:
 *   [IVP_Compact_Surface header]        48 bytes
 *   [IVP_Compact_Ledge[0] + data]       variable
 *   [IVP_Compact_Ledge[1] + data]       variable (if multi-ledge)
 *   ...
 *   [IVP_Compact_Ledgetree_Node[0]]     28 bytes each
 *   [IVP_Compact_Ledgetree_Node[1]]     ...
 *   ...
 *
 * Original bitfield layout for size_and_deviation (MSVC order, LSB first):
 *   [7:0]   max_factor_surface_deviation  (8 bits, unsigned)
 *   [31:8]  byte_size                     (24 bits, signed)
 */
#ifndef IVP_COMPACT_SURFACE_H
#define IVP_COMPACT_SURFACE_H

#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/base/ivp_allocator.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Surface structure ──────────────────────────────────────────────────── */

typedef struct IVP_ALIGN16 ivp_compact_surface {
    float    mass_center[3];            /* 12 bytes — center of mass */
    float    rotation_inertia[3];       /* 12 bytes — diagonal of inertia tensor */
    float    upper_limit_radius;        /* 4 bytes — max distance from mass center */
    uint32_t size_and_deviation;        /* packed: max_factor(8) + byte_size(24) */
    int32_t  offset_ledgetree_root;     /* byte offset to root ledgetree node */
    uint8_t  _pad[12];                  /* explicit padding for 16-byte alignment */
} ivp_compact_surface_t;

/* Size assertion */
_Static_assert(sizeof(ivp_compact_surface_t) == 48,
               "ivp_compact_surface_t must be 48 bytes");

/* ── Bit field accessors ────────────────────────────────────────────────── */

static inline int ivp_surface_get_max_deviation(const ivp_compact_surface_t *s) {
    return (int)(s->size_and_deviation & 0xFFu);
}

static inline int ivp_surface_get_byte_size(const ivp_compact_surface_t *s) {
    /* Extract bits [31:8] as signed 24-bit value. */
    return ((int32_t)s->size_and_deviation) >> 8;
}

static inline void ivp_surface_set_max_deviation(ivp_compact_surface_t *s,
                                                   int val) {
    s->size_and_deviation = (s->size_and_deviation & 0xFFFFFF00u)
                          | ((uint32_t)val & 0xFFu);
}

static inline void ivp_surface_set_byte_size(ivp_compact_surface_t *s,
                                               int size) {
    s->size_and_deviation = (s->size_and_deviation & 0xFFu)
                          | ((uint32_t)(size & 0xFFFFFF) << 8);
}

/* ── Ledgetree root access ──────────────────────────────────────────────── */

static inline const ivp_compact_ledgetree_node_t *ivp_surface_get_ledgetree_root(
    const ivp_compact_surface_t *s)
{
    return (const ivp_compact_ledgetree_node_t *)((const char *)s +
           s->offset_ledgetree_root);
}

/* ── High-level convenience ─────────────────────────────────────────────── */

/* Get the single convex ledge if this surface has exactly one terminal ledge.
 * Returns NULL if the surface is compound (multiple ledges). */
const ivp_compact_ledge_t *ivp_surface_get_single_convex(
    const ivp_compact_surface_t *s);

/* Free a compact surface allocated with ivp_alloc_aligned. */
void ivp_compact_surface_free(const ivp_allocator_t *alloc,
                               ivp_compact_surface_t *s);

#ifdef __cplusplus
}
#endif

#endif /* IVP_COMPACT_SURFACE_H */
