/* ivp_compact_ledge.h — Compact ledge (16 bytes, explicit bit operations)
 *
 * Memory layout of a complete compact ledge:
 *   [IVP_Compact_Ledge header]       16 bytes
 *   [IVP_Compact_Triangle[0]]        16 bytes
 *   [IVP_Compact_Triangle[1]]        16 bytes
 *   ...
 *   [IVP_Compact_Triangle[N-1]]      16 bytes
 *   [IVP_Compact_Poly_Point[0]]      16 bytes
 *   [IVP_Compact_Poly_Point[1]]      16 bytes
 *   ...
 *
 * Original bitfield layout for flags_and_size (MSVC order, LSB first):
 *   [1:0]  has_children_flag  (2 bits)
 *   [3:2]  is_compact_flag    (2 bits)
 *   [7:4]  dummy              (4 bits)
 *   [31:8] size_div_16        (24 bits)
 */
#ifndef IVP_COMPACT_LEDGE_H
#define IVP_COMPACT_LEDGE_H

#include <ivp/geometry/ivp_compact_triangle.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Ledge structure ────────────────────────────────────────────────────── */

struct ivp_compact_ledge {
    int32_t  c_point_offset;    /* byte offset from 'this' to point array */
    int32_t  node_or_client;    /* ledgetree_node_offset or client_data */
    uint32_t flags_and_size;    /* packed bitfield (see header comment) */
    int16_t  n_triangles;       /* number of triangles */
    int16_t  reserved;          /* for future use */
};

/* Size assertion: must equal sizeof(ivp_compact_triangle_t) */
_Static_assert(sizeof(ivp_compact_ledge_t) == 16,
               "ivp_compact_ledge_t must be 16 bytes");
_Static_assert(sizeof(ivp_compact_ledge_t) == sizeof(ivp_compact_triangle_t),
               "ivp_compact_ledge_t must equal ivp_compact_triangle_t in size");

/* ── Bit field accessors for flags_and_size ─────────────────────────────── */

static inline int ivp_ledge_get_has_children(const ivp_compact_ledge_t *l) {
    return (int)(l->flags_and_size & 0x3u);
}

static inline int ivp_ledge_get_is_compact(const ivp_compact_ledge_t *l) {
    return (int)((l->flags_and_size >> 2) & 0x3u);
}

static inline int ivp_ledge_get_size_div_16(const ivp_compact_ledge_t *l) {
    return (int)((l->flags_and_size >> 8) & 0xFFFFFFu);
}

static inline int ivp_ledge_get_size(const ivp_compact_ledge_t *l) {
    return ivp_ledge_get_size_div_16(l) * 16;
}

/* ── Bit field setters ──────────────────────────────────────────────────── */

static inline void ivp_ledge_set_has_children(ivp_compact_ledge_t *l, int val) {
    l->flags_and_size = (l->flags_and_size & 0xFFFFFFFCu)
                      | ((uint32_t)val & 0x3u);
}

static inline void ivp_ledge_set_is_compact(ivp_compact_ledge_t *l, int val) {
    l->flags_and_size = (l->flags_and_size & 0xFFFFFFF3u)
                      | (((uint32_t)val & 0x3u) << 2);
}

static inline void ivp_ledge_set_size(ivp_compact_ledge_t *l, int size) {
    IVP_ASSERT((size > 0) && (size & 0xF) == 0);
    uint32_t div16 = (uint32_t)(size >> 4);
    l->flags_and_size = (l->flags_and_size & 0xFFu)
                      | (div16 << 8);
}

/* ── Ledge initialization ───────────────────────────────────────────────── */

static inline void ivp_ledge_init(ivp_compact_ledge_t *l) {
    l->c_point_offset = 0;
    l->node_or_client = 0;
    l->flags_and_size = 0;
    l->n_triangles = 0;
    l->reserved = 0;
}

/* ── Point array access ─────────────────────────────────────────────────── */

static inline const ivp_compact_poly_point_t *ivp_ledge_get_point_array(
    const ivp_compact_ledge_t *l)
{
    return (const ivp_compact_poly_point_t *)((const char *)l + l->c_point_offset);
}

static inline ivp_compact_poly_point_t *ivp_ledge_get_point_array_mut(
    ivp_compact_ledge_t *l)
{
    return (ivp_compact_poly_point_t *)((char *)l + l->c_point_offset);
}

/* ── Triangle access ────────────────────────────────────────────────────── */

/* Triangles are always placed immediately after the ledge header. */
static inline const ivp_compact_triangle_t *ivp_ledge_get_first_triangle(
    const ivp_compact_ledge_t *l)
{
    return (const ivp_compact_triangle_t *)(l + 1);
}

static inline ivp_compact_triangle_t *ivp_ledge_get_first_triangle_mut(
    ivp_compact_ledge_t *l)
{
    return (ivp_compact_triangle_t *)(l + 1);
}

static inline int ivp_ledge_get_n_triangles(const ivp_compact_ledge_t *l) {
    return (int)l->n_triangles;
}

static inline int ivp_ledge_get_n_points(const ivp_compact_ledge_t *l) {
    return ivp_ledge_get_size_div_16(l) - (int)l->n_triangles - 1;
}

/* ── Terminal / non-terminal ────────────────────────────────────────────── */

static inline bool ivp_ledge_is_terminal(const ivp_compact_ledge_t *l) {
    return ivp_ledge_get_has_children(l) == 0;
}

static inline int ivp_ledge_get_client_data(const ivp_compact_ledge_t *l) {
    IVP_ASSERT(ivp_ledge_is_terminal(l));
    return l->node_or_client;
}

static inline void ivp_ledge_set_client_data(ivp_compact_ledge_t *l,
                                              int value) {
    IVP_ASSERT(ivp_ledge_is_terminal(l));
    if (ivp_ledge_is_terminal(l))   /* ivp_compact_ledge.hxx:245 */
        l->node_or_client = value;
}

/* ── Deferred inline: ivp_edge_get_start_point ──────────────────────────── */

static inline const ivp_compact_poly_point_t *ivp_edge_get_start_point(
    const ivp_compact_edge_t *e,
    const ivp_compact_ledge_t *ledge)
{
    IVP_ASSERT(ledge == ivp_edge_get_compact_ledge(e));   /* ivp_compact_ledge.hxx:341 */
    return &ivp_ledge_get_point_array(ledge)[ivp_edge_get_start_point_index(e)];
}

/* ── Deferred: ivp_tri_get_compact_ledge (needs sizeof(ivp_compact_ledge_t)) ── */

static inline const ivp_compact_ledge_t *ivp_tri_get_compact_ledge_impl(
    const ivp_compact_triangle_t *t)
{
    const ivp_compact_triangle_t *first_tri = t - ivp_tri_get_tri_index(t);
    return (const ivp_compact_ledge_t *)((const char *)first_tri -
           sizeof(ivp_compact_ledge_t));
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_COMPACT_LEDGE_H */
