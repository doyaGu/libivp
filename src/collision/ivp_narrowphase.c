/* ivp_narrowphase.c — Narrowphase distance primitives
 *
 * Implements point-triangle, point-edge, edge-edge distance computations
 * matching original IVP's Compact Ledge Solver (IVP_CLS).
 *
 * Algorithms:
 *   QR-vals: Barycentric coordinates via Gram matrix (Q·R decomposition-style)
 *   S-vals:  Parametric edge projection
 *   KK-vals: Edge-edge closest points via cross product planes
 */
#include <ivp/collision/ivp_narrowphase.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/math/ivp_fast_math.h>

/* ── Helper: get vertex position as ivp_vec3r_t ────────────────────────── */

static inline ivp_vec3r_t point_to_vec3r(const ivp_compact_poly_point_t *p)
{
    return ivp_vec3r((ivp_real_t)p->k[0],
                     (ivp_real_t)p->k[1],
                     (ivp_real_t)p->k[2]);
}

/* ── Barycentric QR computation ─────────────────────────────────────────── */

void ivp_calc_qr_vals(const ivp_compact_ledge_t    *ledge,
                      const ivp_compact_edge_t     *edge,
                      const ivp_vec3r_t            *point_os,
                      ivp_qr_result_t              *result)
{
    IVP_ASSERT(ledge != NULL);
    IVP_ASSERT(edge != NULL);
    IVP_ASSERT(point_os != NULL);
    IVP_ASSERT(result != NULL);

    /* Get the first edge of the triangle */
    const ivp_compact_triangle_t *tri = ivp_edge_get_triangle(edge);
    const ivp_compact_edge_t *e0 = &tri->edges[0];
    const ivp_compact_edge_t *e1 = &tri->edges[1];
    const ivp_compact_edge_t *e2 = &tri->edges[2];

    /* Get three vertices: p0 = start of e0, p1 = start of e1, p2 = start of e2 */
    ivp_vec3r_t p0 = point_to_vec3r(ivp_edge_get_start_point(e0, ledge));
    ivp_vec3r_t p1 = point_to_vec3r(ivp_edge_get_start_point(e1, ledge));
    ivp_vec3r_t p2 = point_to_vec3r(ivp_edge_get_start_point(e2, ledge));

    /* Q = p0 - p1, R = p2 - p1, Pvec = point - p1
     * (Matching original IVP convention where p1 is the "origin" vertex) */
    ivp_vec3r_t Q, R, Pvec;
    ivp_vec3r_sub(&Q, &p0, &p1);
    ivp_vec3r_sub(&R, &p2, &p1);
    ivp_vec3r_sub(&Pvec, point_os, &p1);

    /* Gram matrix entries */
    ivp_real_t QQ = ivp_vec3r_dot(&Q, &Q);
    ivp_real_t RR = ivp_vec3r_dot(&R, &R);
    ivp_real_t QR = ivp_vec3r_dot(&Q, &R);
    ivp_real_t PQ = ivp_vec3r_dot(&Pvec, &Q);
    ivp_real_t PR = ivp_vec3r_dot(&Pvec, &R);

    /* Determinant = QQ*RR - QR*QR (always >= 0 for non-degenerate triangle) */
    ivp_real_t det = QQ * RR - QR * QR;

    /* Barycentric coordinates (unscaled by det):
     *   q = RR*PQ - QR*PR  (weight for vertex p0)
     *   r = QQ*PR - QR*PQ  (weight for vertex p2)
     *   1-q-r             (weight for vertex p1)
     */
    ivp_real_t q = RR * PQ - QR * PR;
    ivp_real_t r = QQ * PR - QR * PQ;

    /* Map to per-edge checks.
     * Original IVP maps: checks[0]=r, checks[1]=det-q-r, checks[2]=q
     * Each check corresponds to one edge of the triangle.
     * Point is inside when all checks >= 0. */
    result->checks[0] = r;
    result->checks[1] = det - q - r;
    result->checks[2] = q;
    result->scale = det;
}

/* ── Parametric S computation ───────────────────────────────────────────── */

void ivp_calc_s_val(const ivp_compact_ledge_t    *ledge,
                    const ivp_compact_edge_t     *edge,
                    const ivp_vec3r_t            *point_os,
                    ivp_s_result_t               *result)
{
    IVP_ASSERT(ledge != NULL);
    IVP_ASSERT(edge != NULL);
    IVP_ASSERT(point_os != NULL);
    IVP_ASSERT(result != NULL);

    /* Get edge endpoints */
    const ivp_compact_edge_t *next_edge = ivp_edge_get_next(edge);
    ivp_vec3r_t p0 = point_to_vec3r(ivp_edge_get_start_point(edge, ledge));
    ivp_vec3r_t p1 = point_to_vec3r(ivp_edge_get_start_point(next_edge, ledge));

    /* d = p1 - p0 (edge direction) */
    ivp_vec3r_t d;
    ivp_vec3r_sub(&d, &p1, &p0);

    /* d0 = p0 - point, d1 = p1 - point */
    ivp_vec3r_t d0, d1;
    ivp_vec3r_sub(&d0, &p0, point_os);
    ivp_vec3r_sub(&d1, &p1, point_os);

    /* checks[0] = -d·d0  (positive if point is past start)
     * checks[1] =  d·d1  (positive if point is before end) */
    result->checks[0] = -ivp_vec3r_dot(&d, &d0);
    result->checks[1] =  ivp_vec3r_dot(&d, &d1);
}

/* ── Edge-edge KK computation ───────────────────────────────────────────── */

void ivp_calc_kk_vals(const ivp_compact_ledge_t    *ledge_k,
                      const ivp_compact_edge_t     *edge_k,
                      const ivp_compact_ledge_t    *ledge_l,
                      const ivp_compact_edge_t     *edge_l,
                      ivp_kk_result_t              *result)
{
    IVP_ASSERT(ledge_k != NULL && edge_k != NULL);
    IVP_ASSERT(ledge_l != NULL && edge_l != NULL);
    IVP_ASSERT(result != NULL);

    /* Get edge K endpoints */
    const ivp_compact_edge_t *next_k = ivp_edge_get_next(edge_k);
    ivp_vec3r_t k0 = point_to_vec3r(ivp_edge_get_start_point(edge_k, ledge_k));
    ivp_vec3r_t k1 = point_to_vec3r(ivp_edge_get_start_point(next_k, ledge_k));

    /* Get edge L endpoints */
    const ivp_compact_edge_t *next_l = ivp_edge_get_next(edge_l);
    ivp_vec3r_t l0 = point_to_vec3r(ivp_edge_get_start_point(edge_l, ledge_l));
    ivp_vec3r_t l1 = point_to_vec3r(ivp_edge_get_start_point(next_l, ledge_l));

    /* Edge direction vectors */
    ivp_vec3r_t k_vec, l_vec;
    ivp_vec3r_sub(&k_vec, &k1, &k0);
    ivp_vec3r_sub(&l_vec, &l1, &l0);

    /* Cross product: norm = k_vec × l_vec */
    ivp_vec3r_t cross;
    ivp_vec3r_cross(&cross, &k_vec, &l_vec);
    ivp_real_t cross_sq = ivp_vec3r_quad_length(&cross);

    /* Check if edges are nearly parallel */
    ivp_real_t k_sq = ivp_vec3r_quad_length(&k_vec);
    ivp_real_t l_sq = ivp_vec3r_quad_length(&l_vec);
    ivp_real_t threshold = k_sq * l_sq * IVP_REAL_RES * IVP_REAL_RES;

    if (cross_sq > threshold) {
        /* Non-parallel case: use perpendicular planes.
         * K_area = k_vec × norm (perpendicular to K in plane of K,L)
         * L_area = l_vec × norm (perpendicular to L in plane of K,L) */
        ivp_vec3r_t k_area, l_area;
        ivp_vec3r_cross(&k_area, &k_vec, &cross);
        ivp_vec3r_cross(&l_area, &l_vec, &cross);

        /* Project L endpoints onto K_area plane:
         * Signed distance from K line to L endpoint */
        ivp_vec3r_t k0_l0, k0_l1;
        ivp_vec3r_sub(&k0_l0, &l0, &k0);
        ivp_vec3r_sub(&k0_l1, &l1, &k0);

        /* checks_l: parametric position on L where closest to K */
        ivp_real_t dl0 = ivp_vec3r_dot(&k0_l0, &k_area);
        ivp_real_t dl1 = ivp_vec3r_dot(&k0_l1, &k_area);
        result->checks_l[0] = -dl0;  /* positive = past start of L */
        result->checks_l[1] =  dl1;  /* positive = before end of L */

        /* Project K endpoints onto L_area plane */
        ivp_vec3r_t l0_k0, l0_k1;
        ivp_vec3r_sub(&l0_k0, &k0, &l0);
        ivp_vec3r_sub(&l0_k1, &k1, &l0);

        ivp_real_t dk0 = ivp_vec3r_dot(&l0_k0, &l_area);
        ivp_real_t dk1 = ivp_vec3r_dot(&l0_k1, &l_area);
        result->checks_k[0] = -dk0;
        result->checks_k[1] =  dk1;

    } else {
        /* Nearly parallel edges: compute proper overlap-based checks.
         * Project each edge's endpoints onto the other edge's direction
         * to determine the parametric position of the closest point. */

        ivp_vec3r_t d;

        /* Project L endpoints onto edge K direction */
        ivp_vec3r_sub(&d, &l0, &k0);
        ivp_real_t l0_on_k = ivp_vec3r_dot(&d, &k_vec);

        ivp_vec3r_sub(&d, &l1, &k0);
        ivp_real_t l1_on_k = ivp_vec3r_dot(&d, &k_vec);

        /* Project K endpoints onto edge L direction */
        ivp_vec3r_sub(&d, &k0, &l0);
        ivp_real_t k0_on_l = ivp_vec3r_dot(&d, &l_vec);

        ivp_vec3r_sub(&d, &k1, &l0);
        ivp_real_t k1_on_l = ivp_vec3r_dot(&d, &l_vec);

        /* Find overlap interval on K: L's projection [proj_min, proj_max]
         * clamped to K's span [0, k_sq]. */
        ivp_real_t proj_min_k = (l0_on_k < l1_on_k) ? l0_on_k : l1_on_k;
        ivp_real_t proj_max_k = (l0_on_k > l1_on_k) ? l0_on_k : l1_on_k;
        ivp_real_t overlap_start_k = (proj_min_k > IVP_REAL(0.0)) ? proj_min_k : IVP_REAL(0.0);
        ivp_real_t overlap_end_k   = (proj_max_k < k_sq) ? proj_max_k : k_sq;

        if (overlap_start_k <= overlap_end_k) {
            /* Segments overlap on K axis: closest point in overlap center */
            ivp_real_t s_on_k = (overlap_start_k + overlap_end_k) * IVP_REAL(0.5);
            result->checks_k[0] = s_on_k;
            result->checks_k[1] = k_sq - s_on_k;
        } else if (proj_max_k < IVP_REAL(0.0)) {
            /* L entirely before K start */
            result->checks_k[0] = proj_max_k;  /* negative → before start */
            result->checks_k[1] = k_sq;
        } else {
            /* L entirely past K end */
            result->checks_k[0] = k_sq;
            result->checks_k[1] = k_sq - proj_min_k;  /* negative → past end */
        }

        /* Find overlap interval on L: K's projection [proj_min, proj_max]
         * clamped to L's span [0, l_sq]. */
        ivp_real_t proj_min_l = (k0_on_l < k1_on_l) ? k0_on_l : k1_on_l;
        ivp_real_t proj_max_l = (k0_on_l > k1_on_l) ? k0_on_l : k1_on_l;
        ivp_real_t overlap_start_l = (proj_min_l > IVP_REAL(0.0)) ? proj_min_l : IVP_REAL(0.0);
        ivp_real_t overlap_end_l   = (proj_max_l < l_sq) ? proj_max_l : l_sq;

        if (overlap_start_l <= overlap_end_l) {
            ivp_real_t s_on_l = (overlap_start_l + overlap_end_l) * IVP_REAL(0.5);
            result->checks_l[0] = s_on_l;
            result->checks_l[1] = l_sq - s_on_l;
        } else if (proj_max_l < IVP_REAL(0.0)) {
            result->checks_l[0] = proj_max_l;
            result->checks_l[1] = l_sq;
        } else {
            result->checks_l[0] = l_sq;
            result->checks_l[1] = l_sq - proj_min_l;
        }
    }
}

/* ── Squared distance: point-point ──────────────────────────────────────── */

ivp_real_t ivp_qdist_pp(const ivp_vec3r_t *a, const ivp_vec3r_t *b)
{
    return ivp_vec3r_quad_distance(a, b);
}

/* ── Squared distance: point-edge ───────────────────────────────────────── */

ivp_real_t ivp_qdist_pk(const ivp_compact_ledge_t *ledge,
                        const ivp_compact_edge_t  *edge,
                        const ivp_vec3r_t         *point_os)
{
    IVP_ASSERT(ledge != NULL && edge != NULL && point_os != NULL);

    /* Get edge endpoints */
    const ivp_compact_edge_t *next_edge = ivp_edge_get_next(edge);
    ivp_vec3r_t p0 = point_to_vec3r(ivp_edge_get_start_point(edge, ledge));
    ivp_vec3r_t p1 = point_to_vec3r(ivp_edge_get_start_point(next_edge, ledge));

    /* Compute s-val to determine if point projects onto edge interior */
    ivp_s_result_t sv;
    ivp_calc_s_val(ledge, edge, point_os, &sv);

    if (sv.checks[0] < IVP_REAL(0.0)) {
        /* Point is behind start vertex → closest to p0 */
        return ivp_vec3r_quad_distance(point_os, &p0);
    }
    if (sv.checks[1] < IVP_REAL(0.0)) {
        /* Point is past end vertex → closest to p1 */
        return ivp_vec3r_quad_distance(point_os, &p1);
    }

    /* Point projects onto edge interior → perpendicular distance */
    return ivp_qdist_point_to_line(ledge, edge, point_os);
}

/* ── Perpendicular squared distance: point to infinite line ─────────────── */

ivp_real_t ivp_qdist_point_to_line(const ivp_compact_ledge_t *ledge,
                                    const ivp_compact_edge_t  *edge,
                                    const ivp_vec3r_t         *point_os)
{
    IVP_ASSERT(ledge != NULL && edge != NULL && point_os != NULL);

    const ivp_compact_edge_t *next_edge = ivp_edge_get_next(edge);
    ivp_vec3r_t p0 = point_to_vec3r(ivp_edge_get_start_point(edge, ledge));
    ivp_vec3r_t p1 = point_to_vec3r(ivp_edge_get_start_point(next_edge, ledge));

    /* d = edge direction, delta = point - p0 */
    ivp_vec3r_t d, delta;
    ivp_vec3r_sub(&d, &p1, &p0);
    ivp_vec3r_sub(&delta, point_os, &p0);

    /* cross = delta × d */
    ivp_vec3r_t cross;
    ivp_vec3r_cross(&cross, &delta, &d);

    /* Squared perpendicular distance = |cross|² / |d|² */
    ivp_real_t d_sq = ivp_vec3r_quad_length(&d);
    if (d_sq < IVP_REAL_RES * IVP_REAL_RES) {
        /* Degenerate edge — return point-point distance */
        return ivp_vec3r_quad_distance(point_os, &p0);
    }

    return ivp_vec3r_quad_length(&cross) / d_sq;
}

/* ── Squared distance: point-triangle ───────────────────────────────────── */

ivp_real_t ivp_qdist_pf(const ivp_compact_ledge_t *ledge,
                        const ivp_compact_edge_t  *tri_edge,
                        const ivp_vec3r_t         *point_os)
{
    IVP_ASSERT(ledge != NULL && tri_edge != NULL && point_os != NULL);

    /* Compute barycentric coordinates */
    ivp_qr_result_t qr;
    const ivp_compact_triangle_t *tri = ivp_edge_get_triangle(tri_edge);
    ivp_calc_qr_vals(ledge, &tri->edges[0], point_os, &qr);

    if (!ivp_qr_is_outside(&qr)) {
        /* Point projects inside triangle → perpendicular distance to plane.
         * dist² = (normal · (point - v))² / |normal|² */
        ivp_vec3r_t hesse;
        ivp_calc_hesse_vec(ledge, &tri->edges[0], &hesse);

        ivp_vec3r_t p0 = point_to_vec3r(
            ivp_edge_get_start_point(&tri->edges[0], ledge));
        ivp_vec3r_t delta;
        ivp_vec3r_sub(&delta, point_os, &p0);

        ivp_real_t dot = ivp_vec3r_dot(&hesse, &delta);
        ivp_real_t hesse_sq = ivp_vec3r_quad_length(&hesse);

        if (hesse_sq < IVP_REAL_RES * IVP_REAL_RES) {
            return IVP_REAL(0.0); /* degenerate triangle */
        }

        return (dot * dot) / hesse_sq;
    }

    /* Point is outside triangle → find closest edge/vertex.
     * Test all 3 edges, find minimum distance. */
    ivp_real_t min_dist = IVP_REAL_MAX;

    for (int i = 0; i < 3; i++) {
        const ivp_compact_edge_t *e = &tri->edges[i];

        /* If this edge's QR check is negative, the point is on
         * the outside of this edge. */
        if (qr.checks[i] < IVP_REAL(0.0)) {
            /* Check if point projects onto this edge */
            ivp_s_result_t sv;
            ivp_calc_s_val(ledge, e, point_os, &sv);

            ivp_real_t d;
            if (sv.checks[0] < IVP_REAL(0.0)) {
                /* Closest to start vertex of this edge */
                ivp_vec3r_t v = point_to_vec3r(
                    ivp_edge_get_start_point(e, ledge));
                d = ivp_vec3r_quad_distance(point_os, &v);
            } else if (sv.checks[1] < IVP_REAL(0.0)) {
                /* Closest to end vertex of this edge (start of next) */
                const ivp_compact_edge_t *ne = ivp_edge_get_next(e);
                ivp_vec3r_t v = point_to_vec3r(
                    ivp_edge_get_start_point(ne, ledge));
                d = ivp_vec3r_quad_distance(point_os, &v);
            } else {
                /* Point projects onto edge interior */
                d = ivp_qdist_point_to_line(ledge, e, point_os);
            }

            if (d < min_dist) {
                min_dist = d;
            }
        }
    }

    return min_dist;
}

/* ── Squared distance: edge-edge ────────────────────────────────────────── */

/* Helper: compute squared distance between closest points on two finite
 * edge segments using parametric clamping. */
static ivp_real_t calc_quad_distance_edge_edge(
    const ivp_vec3r_t *k0, const ivp_vec3r_t *k1,
    const ivp_vec3r_t *l0, const ivp_vec3r_t *l1)
{
    ivp_vec3r_t k_vec, l_vec, d;
    ivp_vec3r_sub(&k_vec, k1, k0);
    ivp_vec3r_sub(&l_vec, l1, l0);
    ivp_vec3r_sub(&d, k0, l0);

    ivp_real_t a = ivp_vec3r_dot(&k_vec, &k_vec);
    ivp_real_t e = ivp_vec3r_dot(&l_vec, &l_vec);
    ivp_real_t f = ivp_vec3r_dot(&l_vec, &d);
    ivp_real_t b = ivp_vec3r_dot(&k_vec, &l_vec);
    ivp_real_t c = ivp_vec3r_dot(&k_vec, &d);

    ivp_real_t denom = a * e - b * b;

    ivp_real_t s, t;

    if (denom > IVP_REAL_RES * IVP_REAL_RES) {
        s = ivp_clamp((b * f - c * e) / denom, IVP_REAL(0.0), IVP_REAL(1.0));
    } else {
        s = IVP_REAL(0.0);
    }

    t = (b * s + f) / (e > IVP_REAL_RES * IVP_REAL_RES ? e : IVP_REAL(1.0));

    if (t < IVP_REAL(0.0)) {
        t = IVP_REAL(0.0);
        s = ivp_clamp(-c / (a > IVP_REAL_RES * IVP_REAL_RES ? a : IVP_REAL(1.0)),
                      IVP_REAL(0.0), IVP_REAL(1.0));
    } else if (t > IVP_REAL(1.0)) {
        t = IVP_REAL(1.0);
        s = ivp_clamp((b - c) / (a > IVP_REAL_RES * IVP_REAL_RES ? a : IVP_REAL(1.0)),
                      IVP_REAL(0.0), IVP_REAL(1.0));
    }

    /* Compute closest points and distance */
    ivp_vec3r_t cp_k, cp_l, diff;
    ivp_vec3r_lerp(&cp_k, k0, k1, s);
    ivp_vec3r_lerp(&cp_l, l0, l1, t);
    ivp_vec3r_sub(&diff, &cp_k, &cp_l);

    return ivp_vec3r_quad_length(&diff);
}

ivp_real_t ivp_qdist_kk(const ivp_compact_ledge_t *ledge_k,
                        const ivp_compact_edge_t  *edge_k,
                        const ivp_compact_ledge_t *ledge_l,
                        const ivp_compact_edge_t  *edge_l)
{
    IVP_ASSERT(ledge_k != NULL && edge_k != NULL);
    IVP_ASSERT(ledge_l != NULL && edge_l != NULL);

    /* Get all 4 endpoints */
    const ivp_compact_edge_t *next_k = ivp_edge_get_next(edge_k);
    const ivp_compact_edge_t *next_l = ivp_edge_get_next(edge_l);

    ivp_vec3r_t k0 = point_to_vec3r(ivp_edge_get_start_point(edge_k, ledge_k));
    ivp_vec3r_t k1 = point_to_vec3r(ivp_edge_get_start_point(next_k, ledge_k));
    ivp_vec3r_t l0 = point_to_vec3r(ivp_edge_get_start_point(edge_l, ledge_l));
    ivp_vec3r_t l1 = point_to_vec3r(ivp_edge_get_start_point(next_l, ledge_l));

    return calc_quad_distance_edge_edge(&k0, &k1, &l0, &l1);
}

/* ── Sphere-sphere distance ─────────────────────────────────────────────── */

ivp_real_t ivp_dist_sphere_sphere(const ivp_vec3r_t *center_a, ivp_real_t radius_a,
                                   const ivp_vec3r_t *center_b, ivp_real_t radius_b)
{
    ivp_real_t dist = ivp_sqrt(ivp_vec3r_quad_distance(center_a, center_b));
    return dist - radius_a - radius_b;
}

/* ── Hesse normal vector (unnormalized) ─────────────────────────────────── */

void ivp_calc_hesse_vec(const ivp_compact_ledge_t *ledge,
                        const ivp_compact_edge_t  *tri_edge,
                        ivp_vec3r_t               *hesse_out)
{
    IVP_ASSERT(ledge != NULL && tri_edge != NULL && hesse_out != NULL);

    const ivp_compact_triangle_t *tri = ivp_edge_get_triangle(tri_edge);
    const ivp_compact_edge_t *e0 = &tri->edges[0];
    const ivp_compact_edge_t *e1 = &tri->edges[1];
    const ivp_compact_edge_t *e2 = &tri->edges[2];

    ivp_vec3r_t p0 = point_to_vec3r(ivp_edge_get_start_point(e0, ledge));
    ivp_vec3r_t p1 = point_to_vec3r(ivp_edge_get_start_point(e1, ledge));
    ivp_vec3r_t p2 = point_to_vec3r(ivp_edge_get_start_point(e2, ledge));

    /* edge vectors: e01 = p1 - p0, e02 = p2 - p0 */
    ivp_vec3r_t e01, e02;
    ivp_vec3r_sub(&e01, &p1, &p0);
    ivp_vec3r_sub(&e02, &p2, &p0);

    /* hesse = e01 × e02 (outward normal × 2·area) */
    ivp_vec3r_cross(hesse_out, &e01, &e02);
}

/* ── Signed distance to triangle plane ──────────────────────────────────── */

ivp_real_t ivp_signed_dist_to_plane(const ivp_compact_ledge_t *ledge,
                                     const ivp_compact_edge_t  *tri_edge,
                                     const ivp_vec3r_t         *point_os)
{
    IVP_ASSERT(ledge != NULL && tri_edge != NULL && point_os != NULL);

    ivp_vec3r_t hesse;
    ivp_calc_hesse_vec(ledge, tri_edge, &hesse);

    ivp_real_t hesse_len = ivp_sqrt(ivp_vec3r_quad_length(&hesse));
    if (hesse_len < IVP_REAL_RES) {
        return IVP_REAL(0.0); /* degenerate triangle */
    }

    const ivp_compact_triangle_t *tri = ivp_edge_get_triangle(tri_edge);
    ivp_vec3r_t p0 = point_to_vec3r(
        ivp_edge_get_start_point(&tri->edges[0], ledge));
    ivp_vec3r_t delta;
    ivp_vec3r_sub(&delta, point_os, &p0);

    return ivp_vec3r_dot(&hesse, &delta) / hesse_len;
}
