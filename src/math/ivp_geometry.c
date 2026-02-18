/* ivp_geometry.c -- Geometric utility implementations
 *
 * Ported from ivu_geometry.cxx (Ipion Software GmbH).
 */
#include <ivp/math/ivp_geometry.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_assert.h>

/* ── ivp_vec3r_calc_orthogonal ─────────────────────────────────────────── */

void ivp_vec3r_calc_orthogonal(ivp_vec3r_t *out, const ivp_vec3r_t *v) {
    /* Find the component with smallest absolute value and cross with
     * the corresponding axis vector to get an orthogonal result. */
    ivp_real_t ax = v->k[0] < IVP_REAL(0.0) ? -v->k[0] : v->k[0];
    ivp_real_t ay = v->k[1] < IVP_REAL(0.0) ? -v->k[1] : v->k[1];
    ivp_real_t az = v->k[2] < IVP_REAL(0.0) ? -v->k[2] : v->k[2];

    if (ax <= ay && ax <= az) {
        /* cross with X axis: (1,0,0) x v = (0, v.z, -v.y) */
        out->k[0] = IVP_REAL(0.0);
        out->k[1] = v->k[2];
        out->k[2] = -v->k[1];
    } else if (ay <= az) {
        /* cross with Y axis: (0,1,0) x v = (-v.z, 0, v.x) */
        out->k[0] = -v->k[2];
        out->k[1] = IVP_REAL(0.0);
        out->k[2] = v->k[0];
    } else {
        /* cross with Z axis: (0,0,1) x v = (v.y, -v.x, 0) */
        out->k[0] = v->k[1];
        out->k[1] = -v->k[0];
        out->k[2] = IVP_REAL(0.0);
    }
}

/* ── Hesse normal form ─────────────────────────────────────────────────── */

ivp_result_t ivp_hesse_calc(ivp_hesse_t *out,
                             const ivp_vec3r_t *p0,
                             const ivp_vec3r_t *p1,
                             const ivp_vec3r_t *p2) {
    ivp_vec3r_t e1, e2;
    ivp_vec3r_sub(&e1, p1, p0);
    ivp_vec3r_sub(&e2, p2, p0);
    ivp_vec3r_cross(&out->normal, &e1, &e2);

    ivp_result_t r = ivp_vec3r_normalize(&out->normal);
    if (r != IVP_OK) return IVP_ERR_DEGENERATE_GEOMETRY;

    out->hesse_val = ivp_vec3r_dot(&out->normal, p0);
    return IVP_OK;
}

/* ── Straight (parametric line) ────────────────────────────────────────── */

ivp_result_t ivp_straight_init(ivp_straight_t *s,
                                const ivp_vec3r_t *start_point,
                                const ivp_vec3r_t *direction) {
    s->start_point = *start_point;
    s->dir = *direction;
    return ivp_vec3r_normalize(&s->dir);
}

void ivp_straight_ortho_vec(const ivp_straight_t *s,
                             const ivp_vec3r_t *point,
                             ivp_vec3r_t *out) {
    /* vec = start_point - point */
    ivp_vec3r_sub(out, &s->start_point, point);

    /* Remove the component along the line direction */
    ivp_real_t sp = ivp_vec3r_dot(&s->dir, out);
    ivp_vec3r_add_scaled(out, &s->dir, -sp);
}

ivp_real_t ivp_straight_qdist_to_point(const ivp_straight_t *s,
                                        const ivp_vec3r_t *point) {
    ivp_vec3r_t w;
    ivp_vec3r_sub(&w, point, &s->start_point);

    /* |dir x w|^2 gives squared distance (since dir is unit length) */
    ivp_vec3r_t cross;
    ivp_vec3r_cross(&cross, &s->dir, &w);
    return ivp_vec3r_quad_length(&cross);
}

/* ── Line-plane intersection ───────────────────────────────────────────── */

ivp_result_t ivp_hesse_intersect_straight(const ivp_hesse_t *h,
                                           const ivp_straight_t *s,
                                           ivp_vec3r_t *point_out) {
    ivp_real_t dist1 = ivp_hesse_dist(h, &s->start_point);

    /* Second point on the line: start + dir */
    ivp_vec3r_t p2;
    ivp_vec3r_add(&p2, &s->start_point, &s->dir);
    ivp_real_t dist2 = ivp_hesse_dist(h, &p2);

    ivp_real_t delta = dist2 - dist1;
    ivp_real_t abs_delta = delta < IVP_REAL(0.0) ? -delta : delta;

    if (abs_delta < IVP_DOUBLE_EPS) {
        /* Line is parallel to plane */
        ivp_real_t abs_dist1 = dist1 < IVP_REAL(0.0) ? -dist1 : dist1;
        if (abs_dist1 >= IVP_DOUBLE_EPS) {
            return IVP_ERR_DEGENERATE_GEOMETRY;  /* truly parallel */
        }
        /* Line lies on the plane */
        *point_out = s->start_point;
        return IVP_OK;
    }

    ivp_real_t factor = dist1 / delta;
    point_out->k[0] = s->start_point.k[0] - s->dir.k[0] * factor;
    point_out->k[1] = s->start_point.k[1] - s->dir.k[1] * factor;
    point_out->k[2] = s->start_point.k[2] - s->dir.k[2] * factor;
    return IVP_OK;
}

/* ── Line-line intersection ────────────────────────────────────────────── */

ivp_intersect_type_t ivp_straight_intersect(const ivp_straight_t *s1,
                                             const ivp_straight_t *s2,
                                             ivp_vec3r_t *point_out,
                                             ivp_real_t *qdist_out) {
    ivp_vec3r_t norm;
    ivp_vec3r_cross(&norm, &s1->dir, &s2->dir);

    ivp_real_t ql = ivp_vec3r_quad_length(&norm);
    if (ql < IVP_DOUBLE_EPS * IVP_DOUBLE_EPS) {
        /* Lines are parallel -- check if identical */
        ivp_vec3r_t hp;
        ivp_straight_ortho_vec(s1, &s2->start_point, &hp);
        *qdist_out = ivp_vec3r_quad_length(&hp);
        if (*qdist_out < IVP_DOUBLE_EPS * IVP_DOUBLE_EPS) {
            return IVP_INTERSECT_IDENTICAL;
        }
        return IVP_INTERSECT_PARALLEL;
    }

    /* Build a plane through s2 with normal = s2.dir x norm */
    ivp_vec3r_t e_p1, e_p2;
    ivp_vec3r_add(&e_p1, &s2->start_point, &s2->dir);
    ivp_vec3r_add(&e_p2, &s2->start_point, &norm);

    ivp_hesse_t hesse;
    ivp_result_t r = ivp_hesse_calc(&hesse,
                                     &s2->start_point,
                                     &e_p1,
                                     &e_p2);
    if (r != IVP_OK) {
        *qdist_out = IVP_REAL(0.0);
        return IVP_INTERSECT_PARALLEL;
    }

    r = ivp_hesse_intersect_straight(&hesse, s1, point_out);
    IVP_ASSERT(r == IVP_OK);
    (void)r;

    /* Measure distance from the found point to s2 */
    ivp_vec3r_t hp;
    ivp_straight_ortho_vec(s2, point_out, &hp);
    *qdist_out = ivp_vec3r_quad_length(&hp);

    if (*qdist_out < IVP_DOUBLE_EPS * IVP_DOUBLE_EPS) {
        return IVP_INTERSECT_OK;
    }
    return IVP_INTERSECT_NO_INTERSECTION;
}

/* ── Plain (plane with spanning vectors) ───────────────────────────────── */

ivp_result_t ivp_plain_init_from_hesse(ivp_plain_t *out, const ivp_hesse_t *hesse) {
    out->hesse = *hesse;

    /* Find a point on the plane by projecting the origin */
    ivp_vec3r_t origin = ivp_vec3r_zero();
    ivp_hesse_project(hesse, &origin, &out->start_point);

    /* Compute spanning vectors */
    ivp_vec3r_calc_orthogonal(&out->vec1, &hesse->normal);
    ivp_vec3r_cross(&out->vec2, &out->vec1, &hesse->normal);

    return IVP_OK;
}

ivp_result_t ivp_plain_init_from_points(ivp_plain_t *out,
                                         const ivp_vec3r_t *p0,
                                         const ivp_vec3r_t *p1,
                                         const ivp_vec3r_t *p2) {
    ivp_result_t r = ivp_hesse_calc(&out->hesse, p0, p1, p2);
    if (r != IVP_OK) return r;

    out->start_point = *p0;
    ivp_vec3r_sub(&out->vec1, p1, p0);
    ivp_vec3r_sub(&out->vec2, p2, p0);
    return IVP_OK;
}

ivp_result_t ivp_plain_intersect(const ivp_plain_t *plane1,
                                  const ivp_hesse_t *plane2,
                                  ivp_straight_t *straight_out) {
    if (ivp_hesse_is_parallel(&plane1->hesse, plane2, IVP_GEOM_EPS_PARALLEL)) {
        return IVP_ERR_DEGENERATE_GEOMETRY;
    }

    /* Direction of intersection line = n1 x n2 */
    ivp_vec3r_cross(&straight_out->dir, &plane1->hesse.normal, &plane2->normal);
    ivp_result_t r = ivp_vec3r_normalize(&straight_out->dir);
    if (r != IVP_OK) return IVP_ERR_DEGENERATE_GEOMETRY;

    /* Find a point on the intersection line:
     * 1. Project plane1's start_point onto plane2
     * 2. Project that onto plane1
     * 3. Use a line through plane1's start toward that projection
     * 4. Intersect that line with plane2 */
    ivp_vec3r_t hp, hp2;
    ivp_hesse_project(plane2, &plane1->start_point, &hp);
    ivp_hesse_project(&plane1->hesse, &hp, &hp2);

    ivp_straight_t temp;
    temp.start_point = plane1->start_point;
    ivp_vec3r_sub(&temp.dir, &hp2, &plane1->start_point);
    r = ivp_vec3r_normalize(&temp.dir);
    if (r != IVP_OK) {
        /* start_point is already on plane2 */
        straight_out->start_point = plane1->start_point;
        return IVP_OK;
    }

    return ivp_hesse_intersect_straight(plane2, &temp, &straight_out->start_point);
}
