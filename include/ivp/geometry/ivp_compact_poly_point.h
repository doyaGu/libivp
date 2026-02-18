/* ivp_compact_poly_point.h — Compact poly point (16-byte aligned vertex)
 *
 * Matches original IVP_Compact_Poly_Point layout:
 *   float k[3]       — vertex position (12 bytes)
 *   float hesse_val  — plane distance / client data (4 bytes)
 * Total: 16 bytes, must be 16-byte aligned.
 */
#ifndef IVP_COMPACT_POLY_POINT_H
#define IVP_COMPACT_POLY_POINT_H

#include <ivp/base/ivp_types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct IVP_ALIGN16 ivp_compact_poly_point {
    float k[3];         /* vertex position */
    float hesse_val;    /* plane distance or client data */
} ivp_compact_poly_point_t;

/* Size assertion: must be exactly 16 bytes */
_Static_assert(sizeof(ivp_compact_poly_point_t) == 16,
               "ivp_compact_poly_point_t must be 16 bytes");

/* ── Construction ───────────────────────────────────────────────────────── */

static inline ivp_compact_poly_point_t ivp_compact_poly_point(float x, float y, float z) {
    ivp_compact_poly_point_t p;
    p.k[0] = x; p.k[1] = y; p.k[2] = z;
    p.hesse_val = 0.0f;
    return p;
}

/* ── Client data access (original IVP stores pointer in hesse_val) ────── */

static inline void ivp_poly_point_set_client_data(ivp_compact_poly_point_t *p,
                                                    uint32_t value) {
    union { float f; uint32_t u; } conv;
    conv.u = value;
    p->hesse_val = conv.f;
}

static inline uint32_t ivp_poly_point_get_client_data(const ivp_compact_poly_point_t *p) {
    union { float f; uint32_t u; } conv;
    conv.f = p->hesse_val;
    return conv.u;
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_COMPACT_POLY_POINT_H */
