/* ivp_compact_edge.h — Compact half-edge (4 bytes, explicit bit operations)
 *
 * Original IVP_Compact_Edge bitfield layout (MSVC order, LSB first):
 *   [15:0]  start_point_index  (16 bits, unsigned)
 *   [30:16] opposite_index     (15 bits, signed)
 *   [31]    is_virtual         (1 bit)
 *
 * Navigation:
 *   get_triangle()   — mask address to 16-byte boundary
 *   get_edge_index() — extract from address bits [3:2]
 *   get_next/prev()  — use navigation tables
 *   get_opposite()   — this + opposite_index (pointer arithmetic)
 */
#ifndef IVP_COMPACT_EDGE_H
#define IVP_COMPACT_EDGE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/geometry/ivp_compact_poly_point.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
typedef struct ivp_compact_triangle ivp_compact_triangle_t;
typedef struct ivp_compact_ledge    ivp_compact_ledge_t;

/* ── Edge structure ─────────────────────────────────────────────────────── */

typedef struct ivp_compact_edge {
    uint32_t packed;
} ivp_compact_edge_t;

/* Size assertion */
_Static_assert(sizeof(ivp_compact_edge_t) == 4,
               "ivp_compact_edge_t must be 4 bytes");

/* ── Bit field accessors ────────────────────────────────────────────────── */

static inline int ivp_edge_get_start_point_index(const ivp_compact_edge_t *e) {
    return (int)(e->packed & 0xFFFFu);
}

static inline int ivp_edge_get_opposite_index(const ivp_compact_edge_t *e) {
    /* Extract bits [30:16] as signed 15-bit value.
     * Shift left 1 to put sign bit at bit 31, then arithmetic right shift 17. */
    return ((int32_t)(e->packed << 1)) >> 17;
}

static inline int ivp_edge_get_is_virtual(const ivp_compact_edge_t *e) {
    return (int)(e->packed >> 31);
}

/* ── Bit field setters ──────────────────────────────────────────────────── */

static inline void ivp_edge_set_start_point_index(ivp_compact_edge_t *e, int val) {
    IVP_ASSERT(val >= 0 && val < (1 << 16));
    e->packed = (e->packed & 0xFFFF0000u) | ((uint32_t)val & 0xFFFFu);
}

static inline void ivp_edge_set_opposite_index(ivp_compact_edge_t *e, int val) {
    IVP_ASSERT(val >= -(1 << 14) && val < (1 << 14));
    uint32_t masked = (uint32_t)(val & 0x7FFF) << 16;
    e->packed = (e->packed & 0x8000FFFFu) | masked;
}

static inline void ivp_edge_set_is_virtual(ivp_compact_edge_t *e, int val) {
    if (val)
        e->packed |= 0x80000000u;
    else
        e->packed &= 0x7FFFFFFFu;
}

static inline void ivp_edge_init(ivp_compact_edge_t *e) {
    e->packed = 0;
}

static inline void ivp_edge_set(ivp_compact_edge_t *e,
                                 int start_point_index,
                                 int opposite_index,
                                 int is_virtual) {
    IVP_ASSERT(start_point_index >= 0 && start_point_index < (1 << 16));
    IVP_ASSERT(opposite_index >= -(1 << 14) && opposite_index < (1 << 14));
    e->packed = ((uint32_t)start_point_index & 0xFFFFu)
              | (((uint32_t)(opposite_index & 0x7FFF)) << 16)
              | ((uint32_t)(is_virtual ? 1 : 0) << 31);
}

/* ── Navigation (address-based, requires 16-byte aligned triangles) ──── */

/* Navigation tables (defined in ivp_compact_edge.c) */
extern const int ivp_edge_next_table[4];
extern const int ivp_edge_prev_table[4];

/* Get the triangle that owns this edge (mask to 16-byte boundary).
 * This works because sizeof(ivp_compact_triangle_t) == 16 and triangles
 * are 16-byte aligned. The edge address within a triangle is at offset
 * 4, 8, or 12, so masking off the low 4 bits gives the triangle base. */
static inline const ivp_compact_triangle_t *ivp_edge_get_triangle(
    const ivp_compact_edge_t *e)
{
    return (const ivp_compact_triangle_t *)((uintptr_t)e & ~(uintptr_t)0xF);
}

/* Edge index within its triangle (0, 1, or 2).
 * Address bits [3:2] encode: 01=edge0, 10=edge1, 11=edge2.
 * Subtracting 1 gives 0-based index. */
static inline int ivp_edge_get_edge_index(const ivp_compact_edge_t *e) {
    return (int)(((uintptr_t)e & 0x0Cu) >> 2) - 1;
}

/* Navigate to the next edge within the same triangle (0→1→2→0). */
static inline const ivp_compact_edge_t *ivp_edge_get_next(
    const ivp_compact_edge_t *e)
{
    int idx = (int)((uintptr_t)e & 0x0Cu);
    return (const ivp_compact_edge_t *)((const char *)e +
           *(const int *)((const char *)ivp_edge_next_table + idx));
}

/* Navigate to the previous edge within the same triangle (0→2→1→0). */
static inline const ivp_compact_edge_t *ivp_edge_get_prev(
    const ivp_compact_edge_t *e)
{
    int idx = (int)((uintptr_t)e & 0x0Cu);
    return (const ivp_compact_edge_t *)((const char *)e +
           *(const int *)((const char *)ivp_edge_prev_table + idx));
}

/* Navigate to the opposite half-edge (the one sharing the same geometric
 * edge but belonging to the adjacent triangle). */
static inline const ivp_compact_edge_t *ivp_edge_get_opposite(
    const ivp_compact_edge_t *e)
{
    int idx = ivp_edge_get_opposite_index(e);
    return e + idx;
}

/* Get the start point of this edge. Requires the owning ledge. */
static inline const ivp_compact_poly_point_t *ivp_edge_get_start_point(
    const ivp_compact_edge_t *e,
    const ivp_compact_ledge_t *ledge);  /* defined after ledge is complete */

/* Get the compact ledge that contains this edge. */
const ivp_compact_ledge_t *ivp_edge_get_compact_ledge(const ivp_compact_edge_t *e);

#ifdef __cplusplus
}
#endif

#endif /* IVP_COMPACT_EDGE_H */
