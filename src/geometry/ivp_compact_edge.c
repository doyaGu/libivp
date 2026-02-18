/* ivp_compact_edge.c — Edge navigation tables and non-inline functions */

#include <ivp/geometry/ivp_compact_ledge.h>
#include <string.h>

/* ── ivp_tri_get_compact_ledge (deferred from header) ───────────────────── */

const ivp_compact_ledge_t *ivp_tri_get_compact_ledge(
    const ivp_compact_triangle_t *t)
{
    return ivp_tri_get_compact_ledge_impl(t);
}

/* ── Navigation tables ──────────────────────────────────────────────────── */

/* These tables are indexed by the 2 address bits [3:2] of the edge pointer.
 * Edge addresses within a 16-byte triangle are at offsets 4, 8, 12:
 *   bits[3:2] = 01 → edge 0 (byte offset 4)
 *   bits[3:2] = 10 → edge 1 (byte offset 8)
 *   bits[3:2] = 11 → edge 2 (byte offset 12)
 *   bits[3:2] = 00 → invalid (this would be the triangle header)
 *
 * The table values are byte deltas to add to the edge pointer.
 *
 * next_table: edge0→edge1(+4), edge1→edge2(+4), edge2→edge0(-8)
 * prev_table: edge0→edge2(+8), edge1→edge0(-4), edge2→edge1(-4)
 */
const int ivp_edge_next_table[4] = {
    0,                                /* [00] invalid */
    1 * (int)sizeof(ivp_compact_edge_t),   /* [01] edge0 → edge1: +4 */
    1 * (int)sizeof(ivp_compact_edge_t),   /* [10] edge1 → edge2: +4 */
    -2 * (int)sizeof(ivp_compact_edge_t)   /* [11] edge2 → edge0: -8 */
};

const int ivp_edge_prev_table[4] = {
    0,                                 /* [00] invalid */
    2 * (int)sizeof(ivp_compact_edge_t),   /* [01] edge0 → edge2: +8 */
    -1 * (int)sizeof(ivp_compact_edge_t),  /* [10] edge1 → edge0: -4 */
    -1 * (int)sizeof(ivp_compact_edge_t)   /* [11] edge2 → edge1: -4 */
};

/* ── Non-inline functions ───────────────────────────────────────────────── */

const ivp_compact_ledge_t *ivp_edge_get_compact_ledge(
    const ivp_compact_edge_t *e)
{
    const ivp_compact_triangle_t *tri = ivp_edge_get_triangle(e);
    /* Walk back to the first triangle in the ledge */
    tri -= ivp_tri_get_tri_index(tri);
    /* The ledge header is immediately before the first triangle */
    return (const ivp_compact_ledge_t *)((const char *)tri -
           sizeof(ivp_compact_ledge_t));
}
