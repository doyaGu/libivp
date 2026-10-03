/* ivp_compact_ledge_geom.h — Reference-precision compact ledge geometry helpers
 *
 * Ports of the IVP_Compact_Ledge_Solver helpers that the geometry builders,
 * surface managers and the ray solver rely on.  They reproduce the reference
 * arithmetic exactly: float vertices are promoted to double before any
 * subtraction (IVP_U_Point = double), cross products are evaluated in
 * double, and the unscaled QR checks are stored as float in the reference
 * edge-relative order.
 *
 * Reference: ivp_collision/ivp_compact_ledge_solver.cxx
 */
#ifndef IVP_COMPACT_LEDGE_GEOM_H
#define IVP_COMPACT_LEDGE_GEOM_H

#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reference P_DOUBLE_EPS / P_DOUBLE_RES (ivu_types.hxx:420-422, IVP_DOUBLE build). */
#define IVP_GEOM_DOUBLE_EPS 1e-10
#define IVP_GEOM_DOUBLE_RES 1e-12
#define IVP_GEOM_FLOAT_RES  1e-6f
/* Reference P_MIN_EDGE_LEN (ivu_types.hxx:427). */
#define IVP_GEOM_MIN_EDGE_LEN 0.01f

/* ── Unscaled QR / S results (reference IVP_Unscaled_QR_Result) ─────────── */

typedef struct ivp_ledge_unscaled_qr {
    float checks[3];  /* checks[j] > 0 -> inside edge (edge_index + j) % 3 */
    float scale;      /* determinant */
} ivp_ledge_unscaled_qr_t;

static inline bool ivp_ledge_unscaled_qr_is_outside(const ivp_ledge_unscaled_qr_t *qr)
{
    union { float f; uint32_t u; } a, b, c;
    a.f = qr->checks[0];
    b.f = qr->checks[1];
    c.f = qr->checks[2];
    return ((a.u | b.u | c.u) >> 31) != 0;
}

/* Hesse vector (not normalized) of the triangle owning `edge`:
 * (p_next - p) x (p_prev - p), evaluated in double.  Points outward for
 * the reference CCW winding.  Ref: calc_hesse_vec_object_not_normized. */
void ivp_ledge_calc_hesse_vec(const ivp_compact_ledge_t *ledge,
                              const ivp_compact_edge_t *edge,
                              double out[3]);

/* Unscaled barycentric checks of p projected onto the triangle owning
 * `edge`.  checks are stored relative to the edge index exactly like
 * calc_unscaled_qr_vals_F_space (ivp_uqr_mod_table). */
void ivp_ledge_calc_unscaled_qr_vals(const ivp_compact_ledge_t *ledge,
                                     const ivp_compact_edge_t *edge,
                                     const double p[3],
                                     ivp_ledge_unscaled_qr_t *out);

/* Unscaled edge parameter checks (calc_unscaled_s_val_K_space). */
void ivp_ledge_calc_unscaled_s_vals(const ivp_compact_ledge_t *ledge,
                                    const ivp_compact_edge_t *edge,
                                    const double p[3],
                                    float checks_out[2]);

/* Squared distance from p to triangle `tri` (calc_qlen_PF_F_space). */
double ivp_ledge_calc_qlen_pf(const ivp_compact_ledge_t *ledge,
                              const ivp_compact_triangle_t *tri,
                              const double p[3]);

/* Axis aligned bounding box of a ledge (calc_bounding_box).  The extents
 * are accumulated in float and returned as double like the reference. */
void ivp_ledge_calc_bounding_box_d(const ivp_compact_ledge_t *ledge,
                                   double min_out[3],
                                   double max_out[3]);

/* Max distance and max surface deviation of a ledge relative to `center`
 * (calc_radius_to_given_center).  Does NOT initialise the outputs: the
 * results are max-merged into *radius_inout / *dev_inout.  dev_inout may
 * be NULL. */
void ivp_ledge_calc_radius_to_given_center(const ivp_compact_ledge_t *ledge,
                                           const double center[3],
                                           double *radius_inout,
                                           double *dev_inout);

/* Same for every terminal ledge of a compact surface.  Initialises both
 * outputs to 0 first. */
void ivp_surface_calc_radius_to_given_center(const ivp_compact_surface_t *surface,
                                             const double center[3],
                                             double *radius_out,
                                             double *dev_out);

/* Collect all terminal ledges of a ledgetree in reference order
 * (IVP_Compact_Ledge_Solver::get_all_ledges: depth first, left first).
 * Writes at most `capacity` pointers, returns the total number found. */
int ivp_ledgetree_collect_terminal_ledges(const ivp_compact_ledgetree_node_t *root,
                                          const ivp_compact_ledge_t **out,
                                          int capacity);

/* ── Validation (IVP_Compact_Ledge_Solver::check_ledge) ─────────────────── */

/* Validate topology/convexity of one compact ledge.  Returns true if the
 * ledge passes every check of the reference check_ledge(): 16-byte
 * alignment, tri_index sequence, opposite symmetry, opposite start points,
 * convexity (neighbour apex not more than 10000*P_FLOAT_RES below the
 * plane) and finite edge round trips.  Never asserts. */
bool ivp_compact_ledge_check(const ivp_compact_ledge_t *ledge);

/* Load-time validator for a compact surface blob of `buffer_size` bytes
 * (pass 0 to use the header byte_size).  Checks header sanity, that every
 * ledgetree node / ledge / point lies inside the buffer, tree structure and
 * every ledge with ivp_compact_ledge_check().  Returns true when valid. */
bool ivp_compact_surface_check(const ivp_compact_surface_t *surface,
                               size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* IVP_COMPACT_LEDGE_GEOM_H */
