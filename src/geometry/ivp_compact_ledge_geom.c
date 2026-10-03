/* ivp_compact_ledge_geom.c — Reference-precision compact ledge geometry helpers
 *
 * Ported from ivp_collision/ivp_compact_ledge_solver.cxx (Ipion Software GmbH).
 * All intermediate values follow the reference IVP_DOUBLE (= double) build:
 * IVP_U_Point arithmetic is double, IVP_U_Float_Point storage is float.
 */
#include <ivp/geometry/ivp_compact_ledge_geom.h>
#include <float.h>
#include <math.h>
#include <string.h>

/* ── Small double helpers ──────────────────────────────────────────────── */

static inline void pt_d(const ivp_compact_poly_point_t *p, double out[3])
{
    out[0] = (double)p->k[0];
    out[1] = (double)p->k[1];
    out[2] = (double)p->k[2];
}

static inline double dot_d(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/* IVP_U_Point::subtract(const IVP_U_Float_Point *, const IVP_U_Float_Point *):
 * the difference is a float expression, rounded to float before it is
 * stored in the double result (ivu_linear_double.hxx:315). */
static inline void sub_ff(const ivp_compact_poly_point_t *a, const ivp_compact_poly_point_t *b, double out[3])
{
    out[0] = (double)(a->k[0] - b->k[0]);
    out[1] = (double)(a->k[1] - b->k[1]);
    out[2] = (double)(a->k[2] - b->k[2]);
}

static inline const ivp_compact_poly_point_t *edge_pt(const ivp_compact_edge_t *e,
                                                       const ivp_compact_ledge_t *l)
{
    return ivp_edge_get_start_point(e, l);
}

/* IVP_U_Point::inline_set_vert_to_area_defined_by_three_points(tp0, tp1, tp2)
 * with double inputs (ivu_linear_double.hxx:525). */
static void area_vec(const double tp0[3], const double tp1[3], const double tp2[3],
                     double out[3])
{
    double a0 = tp1[0] - tp0[0];
    double a1 = tp1[1] - tp0[1];
    double a2 = tp1[2] - tp0[2];
    double b0 = tp2[0] - tp0[0];
    double b1 = tp2[1] - tp0[1];
    double b2 = tp2[2] - tp0[2];
    out[0] = b1 * a2 - a1 * b2;
    out[1] = b2 * a0 - a2 * b0;
    out[2] = b0 * a1 - a0 * b1;
}

/* ── Hesse vector ──────────────────────────────────────────────────────── */

void ivp_ledge_calc_hesse_vec(const ivp_compact_ledge_t *ledge,
                              const ivp_compact_edge_t *edge,
                              double out[3])
{
    /* ivp_compact_ledge_solver.cxx:606: area(p0, p_prev, p_next) */
    double p0[3], p1[3], p2[3];
    pt_d(edge_pt(edge, ledge), p0);
    pt_d(edge_pt(ivp_edge_get_next(edge), ledge), p1);
    pt_d(edge_pt(ivp_edge_get_prev(edge), ledge), p2);
    area_vec(p0, p2, p1, out);
}

/* ── Unscaled QR values ────────────────────────────────────────────────── */

/* ivp_compact_ledge_solver.cxx:400 (byte offsets / sizeof(float)) */
static const int s_uqr_mod_table[5] = { 0, 2, 1, 0, 2 };

void ivp_ledge_calc_unscaled_qr_vals(const ivp_compact_ledge_t *ledge,
                                     const ivp_compact_edge_t *edge,
                                     const double p[3],
                                     ivp_ledge_unscaled_qr_t *out)
{
    const int idx = ivp_edge_get_edge_index(edge);
    const ivp_compact_triangle_t *tri = ivp_edge_get_triangle(edge);

    const ivp_compact_poly_point_t *tp = edge_pt(&tri->edges[0], ledge);
    const ivp_compact_poly_point_t *tp_next = edge_pt(&tri->edges[1], ledge);
    const ivp_compact_poly_point_t *tp_prev = edge_pt(&tri->edges[2], ledge);

    /* Q, R: IVP_U_Point::subtract(Float, Float) (float differences) */
    double Q[3], R[3], P[3];
    sub_ff(tp, tp_next, Q);
    sub_ff(tp_prev, tp_next, R);
    for (int i = 0; i < 3; i++) P[i] = p[i] - (double)tp_next->k[i];
    double QQ = dot_d(Q, Q);
    double RR = dot_d(R, R);
    double QR = dot_d(R, Q);
    double det = QQ * RR - QR * QR;
    double sq = dot_d(P, Q);
    double sr = dot_d(P, R);
    double q = RR * sq - sr * QR;
    double r = QQ * sr - sq * QR;

    out->scale = (float)det;
    out->checks[s_uqr_mod_table[idx]]     = (float)r;
    out->checks[s_uqr_mod_table[idx + 1]] = (float)(det - q - r);
    out->checks[s_uqr_mod_table[idx + 2]] = (float)q;
}

void ivp_ledge_calc_unscaled_s_vals(const ivp_compact_ledge_t *ledge,
                                    const ivp_compact_edge_t *edge,
                                    const double p[3],
                                    float checks_out[2])
{
    const ivp_compact_poly_point_t *fp0 = edge_pt(edge, ledge);
    const ivp_compact_poly_point_t *fp1 = edge_pt(ivp_edge_get_next(edge), ledge);
    /* ivp_compact_ledge_solver.cxx:310: d = p1 - p0 is a float difference */
    double d[3], d0[3], d1[3];
    sub_ff(fp1, fp0, d);
    for (int i = 0; i < 3; i++) {
        d0[i] = (double)fp0->k[i] - p[i];
        d1[i] = (double)fp1->k[i] - p[i];
    }
    checks_out[0] = (float)(-dot_d(d, d0));
    checks_out[1] = (float)dot_d(d, d1);
}

/* quad_dist_edge_to_point_K_space (ivp_compact_ledge_solver.cxx:622) */
static double qdist_edge_to_point(const ivp_compact_ledge_t *ledge,
                                  const ivp_compact_edge_t *edge,
                                  const double p[3])
{
    const ivp_compact_poly_point_t *fp0 = edge_pt(edge, ledge);
    const ivp_compact_poly_point_t *fp1 = edge_pt(ivp_edge_get_next(edge), ledge);
    double p0[3], p1[3], v[3];
    pt_d(fp0, p0);
    pt_d(fp1, p1);
    area_vec(p0, p1, p, v);
    /* IVP_U_Float_Point::quad_distance_to(IVP_U_Float_Point): float
     * differences, squared and summed in double */
    double dxyz[3];
    sub_ff(fp0, fp1, dxyz);
    double edge_ql = dxyz[0] * dxyz[0] + dxyz[1] * dxyz[1] + dxyz[2] * dxyz[2];
    if (edge_ql < IVP_GEOM_DOUBLE_EPS) return 0.0;
    return dot_d(v, v) / edge_ql;
}

double ivp_ledge_calc_qlen_pf(const ivp_compact_ledge_t *ledge,
                              const ivp_compact_triangle_t *tri,
                              const double p[3])
{
    /* ivp_compact_ledge_solver.cxx:634 */
    const ivp_compact_edge_t *e = &tri->edges[0];
    ivp_ledge_unscaled_qr_t qr;
    ivp_ledge_calc_unscaled_qr_vals(ledge, e, p, &qr);

    if (!ivp_ledge_unscaled_qr_is_outside(&qr)) {
        double normal[3], start[3];
        pt_d(edge_pt(e, ledge), start);
        ivp_ledge_calc_hesse_vec(ledge, e, normal);
        double n_ql = dot_d(normal, normal);
        if (n_ql < IVP_GEOM_DOUBLE_EPS) return DBL_MAX;
        double res = dot_d(normal, p) - dot_d(normal, start);
        return res * res / n_ql;
    }

    double qres = DBL_MAX;
    for (int i = 2; i >= 0; i--) {
        e = &tri->edges[i];
        float sr[2];
        ivp_ledge_calc_unscaled_s_vals(ledge, e, p, sr);
        if (sr[0] < 0.0f) {
            double ep[3];
            pt_d(edge_pt(e, ledge), ep);
            double dx = p[0] - ep[0], dy = p[1] - ep[1], dz = p[2] - ep[2];
            double qdist = dx * dx + dy * dy + dz * dz;
            if (qdist < qres) qres = qdist;
            continue;
        }
        if (sr[1] < 0.0f) continue;
        double qdist = qdist_edge_to_point(ledge, e, p);
        if (qdist < qres) qres = qdist;
    }
    return qres;
}

/* ── Bounding box / radius ─────────────────────────────────────────────── */

void ivp_ledge_calc_bounding_box_d(const ivp_compact_ledge_t *ledge,
                                   double min_out[3],
                                   double max_out[3])
{
    /* ivp_compact_ledge_solver.cxx:41 — extents kept in IVP_U_Float_Point */
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
    const ivp_compact_poly_point_t *any = edge_pt(&tri->edges[0], ledge);
    float mn[3] = { any->k[0], any->k[1], any->k[2] };
    float mx[3] = { any->k[0], any->k[1], any->k[2] };

    int n = ivp_ledge_get_n_triangles(ledge);
    for (int t = 0; t < n; t++, tri++) {
        for (int e = 0; e < 3; e++) {
            const ivp_compact_poly_point_t *p = edge_pt(&tri->edges[e], ledge);
            for (int i = 0; i < 3; i++) {
                if (p->k[i] < mn[i]) mn[i] = p->k[i];
                else if (p->k[i] > mx[i]) mx[i] = p->k[i];
            }
        }
    }
    for (int i = 0; i < 3; i++) {
        min_out[i] = (double)mn[i];
        max_out[i] = (double)mx[i];
    }
}

void ivp_ledge_calc_radius_to_given_center(const ivp_compact_ledge_t *ledge,
                                           const double center[3],
                                           double *radius_inout,
                                           double *dev_inout)
{
    /* ivp_compact_ledge_solver.cxx:157 */
    double q_max = 0.0;
    double dev_max = 0.0;

    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
    int n = ivp_ledge_get_n_triangles(ledge);
    for (int t = 0; t < n; t++, tri++) {
        const ivp_compact_edge_t *e = &tri->edges[0];
        double hesse[3];
        ivp_ledge_calc_hesse_vec(ledge, e, hesse);
        double hv_ql = dot_d(hesse, hesse);
        if (hv_ql < IVP_GEOM_DOUBLE_EPS) continue;
        double i_hv_len_squared = 1.0 / hv_ql;

        for (int j = 0; j < 3; j++) {
            double pp[3], p[3];
            pt_d(edge_pt(&tri->edges[j], ledge), pp);
            p[0] = pp[0] - center[0];
            p[1] = pp[1] - center[1];
            p[2] = pp[2] - center[2];
            double q = dot_d(p, p);
            if (q > q_max) q_max = q;

            double cp[3];
            cp[0] = p[1] * hesse[2] - p[2] * hesse[1];
            cp[1] = p[2] * hesse[0] - p[0] * hesse[2];
            cp[2] = p[0] * hesse[1] - p[1] * hesse[0];
            double dev = dot_d(cp, cp) * i_hv_len_squared;
            if (dev > dev_max) dev_max = dev;
        }
    }
    q_max = sqrt(q_max);
    if (q_max > *radius_inout) *radius_inout = q_max;
    if (dev_inout) {
        dev_max = sqrt(dev_max);
        if (dev_max > *dev_inout) *dev_inout = dev_max;
    }
}

int ivp_ledgetree_collect_terminal_ledges(const ivp_compact_ledgetree_node_t *root,
                                          const ivp_compact_ledge_t **out,
                                          int capacity)
{
    if (!root) return 0;
    if (ivp_ledgetree_is_terminal(root)) {
        if (out && capacity > 0) out[0] = ivp_ledgetree_get_compact_ledge(root);
        return 1;
    }
    int n_left = ivp_ledgetree_collect_terminal_ledges(ivp_ledgetree_left_son(root),
                                                       out, capacity);
    int rem = capacity - n_left;
    int n_right = ivp_ledgetree_collect_terminal_ledges(
        ivp_ledgetree_right_son(root),
        (out && rem > 0) ? out + n_left : NULL,
        rem > 0 ? rem : 0);
    return n_left + n_right;
}

static void radius_rec(const ivp_compact_ledgetree_node_t *node,
                       const double center[3], double *r, double *dev)
{
    if (ivp_ledgetree_is_terminal(node)) {
        ivp_ledge_calc_radius_to_given_center(ivp_ledgetree_get_compact_ledge(node),
                                              center, r, dev);
        return;
    }
    radius_rec(ivp_ledgetree_left_son(node), center, r, dev);
    radius_rec(ivp_ledgetree_right_son(node), center, r, dev);
}

void ivp_surface_calc_radius_to_given_center(const ivp_compact_surface_t *surface,
                                             const double center[3],
                                             double *radius_out,
                                             double *dev_out)
{
    /* ivp_compact_ledge_solver.cxx:232 (max is order independent) */
    double r = 0.0, d = 0.0;
    radius_rec(ivp_surface_get_ledgetree_root(surface), center, &r, &d);
    *radius_out = r;
    if (dev_out) *dev_out = d;
}

/* ── check_ledge ───────────────────────────────────────────────────────── */

bool ivp_compact_ledge_check(const ivp_compact_ledge_t *cl)
{
    /* ivp_compact_ledge_solver.cxx:788 */
    if (!cl) return false;
    if (((uintptr_t)cl & 15u) != 0) return false;
    const ivp_compact_poly_point_t *pa = ivp_ledge_get_point_array(cl);
    if (((uintptr_t)pa & 15u) != 0) return false;

    int n_tri = ivp_ledge_get_n_triangles(cl);
    if (n_tri <= 0) return false;
    const ivp_compact_triangle_t *tris = ivp_ledge_get_first_triangle(cl);
    const ivp_compact_edge_t *first_edge = &tris[0].edges[0];
    const ivp_compact_edge_t *last_edge = &tris[n_tri - 1].edges[2];

    for (int i = 0; i < n_tri; i++) {
        const ivp_compact_triangle_t *tri = &tris[i];
        if (ivp_tri_get_tri_index(tri) != i) return false;

        for (int j = 0; j < 3; j++) {
            const ivp_compact_edge_t *edge = &tri->edges[j];
            const ivp_compact_edge_t *oppo = ivp_edge_get_opposite(edge);
            if (oppo < first_edge || oppo > last_edge) return false;
            if (ivp_edge_get_edge_index(oppo) < 0) return false;
            if (ivp_edge_get_opposite(oppo) != edge) return false;
            if (ivp_edge_get_start_point_index(edge) !=
                ivp_edge_get_start_point_index(ivp_edge_get_next(oppo))) {
                return false;
            }

            /* convexity: IVP_U_Hesse::calc_hesse(p0, p1, p2) is the
             * normalised area normal (p1-p0)x(p2-p0) */
            double p0[3], p1[3], p2[3], pp[3], h[3];
            pt_d(edge_pt(edge, cl), p0);
            pt_d(edge_pt(ivp_edge_get_next(edge), cl), p1);
            pt_d(edge_pt(ivp_edge_get_prev(edge), cl), p2);
            area_vec(p0, p2, p1, h);
            double hl = sqrt(dot_d(h, h));
            if (hl > IVP_GEOM_DOUBLE_EPS) {
                h[0] /= hl; h[1] /= hl; h[2] /= hl;
                pt_d(edge_pt(ivp_edge_get_prev(oppo), cl), pp);
                double test_dist = dot_d(h, pp) - dot_d(h, p0);
                /* get_dist() is positive outside; convexity requires the
                 * neighbour apex not to be in front of the plane. */
                if (-test_dist < -(double)IVP_GEOM_FLOAT_RES * 10000.0) return false;
            }

            /* round trip around the start vertex */
            int max_c = 0;
            for (const ivp_compact_edge_t *e = ivp_edge_get_opposite(edge);;
                 e = ivp_edge_get_opposite(ivp_edge_get_next(e))) {
                if (max_c++ > 1000) return false;
                if (ivp_edge_get_opposite(e) == edge) break;
            }
        }
    }
    return true;
}

/* ── Surface validator ─────────────────────────────────────────────────── */

typedef struct surf_check_ctx {
    const char *base;
    size_t size;
    int nodes_left;
} surf_check_ctx_t;

static bool range_ok(const surf_check_ctx_t *c, const void *p, size_t len)
{
    const char *cp = (const char *)p;
    if (cp < c->base) return false;
    size_t off = (size_t)(cp - c->base);
    return off <= c->size && len <= c->size - off;
}

static bool check_ledge_in_buffer(const surf_check_ctx_t *c, const ivp_compact_ledge_t *l)
{
    if (!range_ok(c, l, sizeof(ivp_compact_ledge_t))) return false;
    if (((uintptr_t)l & 15u) != 0) return false;
    int n_tri = ivp_ledge_get_n_triangles(l);
    if (n_tri <= 0 || n_tri > (1 << 12)) return false;
    if (!range_ok(c, l, sizeof(ivp_compact_ledge_t) +
                           (size_t)n_tri * sizeof(ivp_compact_triangle_t))) {
        return false;
    }
    const ivp_compact_poly_point_t *pa = ivp_ledge_get_point_array(l);
    int max_index = -1;
    const ivp_compact_triangle_t *tris = ivp_ledge_get_first_triangle(l);
    for (int t = 0; t < n_tri; t++) {
        for (int e = 0; e < 3; e++) {
            int pi = ivp_edge_get_start_point_index(&tris[t].edges[e]);
            if (pi > max_index) max_index = pi;
        }
    }
    if (!range_ok(c, pa, (size_t)(max_index + 1) * sizeof(ivp_compact_poly_point_t))) {
        return false;
    }
    for (int i = 0; i <= max_index; i++) {
        for (int k = 0; k < 3; k++) {
            if (!isfinite(pa[i].k[k])) return false;
        }
    }
    return ivp_compact_ledge_check(l);
}

static bool check_node(surf_check_ctx_t *c, const ivp_compact_ledgetree_node_t *n, int depth)
{
    if (depth > 256 || --c->nodes_left < 0) return false;
    if (!range_ok(c, n, sizeof(*n))) return false;
    if (!(n->radius >= 0.0f) || !isfinite(n->radius)) return false;
    if (ivp_ledgetree_is_terminal(n)) {
        if (n->offset_compact_ledge == 0) return false;
        const ivp_compact_ledge_t *l = ivp_ledgetree_get_compact_ledge(n);
        if (!check_ledge_in_buffer(c, l)) return false;
        return ivp_ledge_is_terminal(l);
    }
    if (n->offset_right_node <= (int32_t)sizeof(*n)) return false;
    const ivp_compact_ledge_t *hull = ivp_ledgetree_get_compact_hull(n);
    if (hull) {
        if (!check_ledge_in_buffer(c, hull)) return false;
        if (ivp_ledge_is_terminal(hull)) return false;
    }
    if (!check_node(c, ivp_ledgetree_left_son(n), depth + 1)) return false;
    return check_node(c, ivp_ledgetree_right_son(n), depth + 1);
}

bool ivp_compact_surface_check(const ivp_compact_surface_t *surface, size_t buffer_size)
{
    if (!surface) return false;
    if (((uintptr_t)surface & 15u) != 0) return false;
    int byte_size = ivp_surface_get_byte_size(surface);
    if (byte_size < (int)(sizeof(ivp_compact_surface_t) + sizeof(ivp_compact_ledgetree_node_t))) {
        return false;
    }
    if (buffer_size == 0) buffer_size = (size_t)byte_size;
    if ((size_t)byte_size > buffer_size) return false;
    for (int i = 0; i < 3; i++) {
        if (!isfinite(surface->mass_center[i]) || !isfinite(surface->rotation_inertia[i])) {
            return false;
        }
    }
    if (!isfinite(surface->upper_limit_radius) || surface->upper_limit_radius < 0.0f) return false;
    if (surface->offset_ledgetree_root < (int32_t)sizeof(ivp_compact_surface_t)) return false;

    surf_check_ctx_t c;
    c.base = (const char *)surface;
    c.size = (size_t)byte_size;
    c.nodes_left = byte_size / (int)sizeof(ivp_compact_ledgetree_node_t);
    return check_node(&c, ivp_surface_get_ledgetree_root(surface), 0);
}
