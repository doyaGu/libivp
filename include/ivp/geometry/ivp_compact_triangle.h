/* ivp_compact_triangle.h — Compact triangle (16 bytes, explicit bit operations)
 *
 * Original IVP_Compact_Triangle bitfield layout (MSVC order, LSB first):
 *   [11:0]  tri_index      (12 bits) — triangle index within ledge (upward nav)
 *   [23:12] pierce_index   (12 bits) — index of pierced triangle
 *   [30:24] material_index (7 bits)  — material ID
 *   [31]    is_virtual     (1 bit)
 *
 * Followed by 3 compact edges (12 bytes).
 * Total: 4 + 12 = 16 bytes. Must equal sizeof(ivp_compact_ledge_t).
 */
#ifndef IVP_COMPACT_TRIANGLE_H
#define IVP_COMPACT_TRIANGLE_H

#include <ivp/base/ivp_assert.h>
#include <ivp/geometry/ivp_compact_edge.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Triangle structure ─────────────────────────────────────────────────── */

struct ivp_compact_triangle {
    uint32_t           packed;        /* bitfield header (4 bytes) */
    ivp_compact_edge_t edges[3];     /* three half-edges (12 bytes) */
};

/* Size assertion: must be exactly 16 bytes */
_Static_assert(sizeof(ivp_compact_triangle_t) == 16,
               "ivp_compact_triangle_t must be 16 bytes");

/* ── Constants ──────────────────────────────────────────────────────────── */

/* tri_index is a 12 bit field (reference ivp_compact_ledge.hxx:40) */
#define IVP_MAX_TRIANGLES_PER_LEDGE (1 << 12)

/* ── Bit field accessors ────────────────────────────────────────────────── */

static inline int ivp_tri_get_tri_index(const ivp_compact_triangle_t *t) {
    return (int)(t->packed & 0xFFFu);
}

static inline int ivp_tri_get_pierce_index(const ivp_compact_triangle_t *t) {
    return (int)((t->packed >> 12) & 0xFFFu);
}

static inline int ivp_tri_get_material_index(const ivp_compact_triangle_t *t) {
    return (int)((t->packed >> 24) & 0x7Fu);
}

static inline int ivp_tri_get_is_virtual(const ivp_compact_triangle_t *t) {
    return (int)(t->packed >> 31);
}

/* ── Bit field setters ──────────────────────────────────────────────────── */

static inline void ivp_tri_set_tri_index(ivp_compact_triangle_t *t, int val) {
    IVP_ASSERT(val >= 0 && val < (1 << 12));
    t->packed = (t->packed & 0xFFFFF000u) | ((uint32_t)val & 0xFFFu);
}

static inline void ivp_tri_set_pierce_index(ivp_compact_triangle_t *t, int val) {
    IVP_ASSERT(val >= 0 && val < (1 << 12));
    t->packed = (t->packed & 0xFF000FFFu) | (((uint32_t)val & 0xFFFu) << 12);
}

static inline void ivp_tri_set_material_index(ivp_compact_triangle_t *t, int val) {
    IVP_ASSERT(val >= 0 && val < (1 << 7));
    t->packed = (t->packed & 0x80FFFFFFu) | (((uint32_t)val & 0x7Fu) << 24);
}

static inline void ivp_tri_set_is_virtual(ivp_compact_triangle_t *t, int val) {
    if (val)
        t->packed |= 0x80000000u;
    else
        t->packed &= 0x7FFFFFFFu;
}

static inline void ivp_tri_init(ivp_compact_triangle_t *t) {
    t->packed = 0;
    ivp_edge_init(&t->edges[0]);
    ivp_edge_init(&t->edges[1]);
    ivp_edge_init(&t->edges[2]);
}

/* ── Triangle navigation ────────────────────────────────────────────────── */

static inline const ivp_compact_edge_t *ivp_tri_get_first_edge(
    const ivp_compact_triangle_t *t)
{
    return &t->edges[0];
}

static inline const ivp_compact_edge_t *ivp_tri_get_edge(
    const ivp_compact_triangle_t *t, int index)
{
    IVP_ASSERT(index >= 0 && index < 3);
    return &t->edges[index];
}

static inline const ivp_compact_triangle_t *ivp_tri_get_next(
    const ivp_compact_triangle_t *t)
{
    return t + 1;
}

/* Get the compact ledge that owns this triangle (walk back via tri_index).
 * NOTE: Defined in ivp_compact_ledge.h after ivp_compact_ledge_t is complete. */
const ivp_compact_ledge_t *ivp_tri_get_compact_ledge(
    const ivp_compact_triangle_t *t);

#ifdef __cplusplus
}
#endif

#endif /* IVP_COMPACT_TRIANGLE_H */
