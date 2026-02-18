/* ivp_geometry.h -- Geometric utility types and operations for IVP C17
 *
 * Provides:
 *   ivp_straight_t  -- parametric line (normalized direction + point)
 *   ivp_hesse_t     -- Hesse normal form plane (unit normal + distance)
 *   ivp_plain_t     -- plane with spanning vectors + intersection ops
 *
 * Ported from ivu_geometry.hxx / ivu_geometry.cxx.
 */
#ifndef IVP_GEOMETRY_H
#define IVP_GEOMETRY_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/math/ivp_vec3.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

#define IVP_GEOM_EPS_PARALLEL  IVP_REAL(0.000000001)

/* ── Intersection result ───────────────────────────────────────────────── */

typedef enum ivp_intersect_type {
    IVP_INTERSECT_OK             = 0,
    IVP_INTERSECT_NO_INTERSECTION = 1,
    IVP_INTERSECT_IDENTICAL      = 2,
    IVP_INTERSECT_PARALLEL       = 3
} ivp_intersect_type_t;

/* ── Hesse normal form plane ───────────────────────────────────────────── */

/* Unit normal (n) + signed distance (d) such that n.x = d for points on the plane. */
typedef struct ivp_hesse {
    ivp_vec3r_t normal;     /* unit normal */
    ivp_real_t  hesse_val;  /* signed distance from origin */
} ivp_hesse_t;

/* Compute Hesse normal form from three points (p0, p1, p2).
 * Normal = normalize((p1-p0) x (p2-p0)), hesse_val = dot(normal, p0). */
ivp_result_t ivp_hesse_calc(ivp_hesse_t *out,
                             const ivp_vec3r_t *p0,
                             const ivp_vec3r_t *p1,
                             const ivp_vec3r_t *p2);

/* Signed distance from point to plane. Positive = same side as normal. */
static inline ivp_real_t ivp_hesse_dist(const ivp_hesse_t *h, const ivp_vec3r_t *p) {
    return ivp_vec3r_dot(&h->normal, p) - h->hesse_val;
}

/* Project point onto plane. */
static inline void ivp_hesse_project(const ivp_hesse_t *h,
                                      const ivp_vec3r_t *p,
                                      ivp_vec3r_t *out) {
    ivp_real_t d = ivp_hesse_dist(h, p);
    out->k[0] = p->k[0] - h->normal.k[0] * d;
    out->k[1] = p->k[1] - h->normal.k[1] * d;
    out->k[2] = p->k[2] - h->normal.k[2] * d;
}

/* Check if two planes are parallel (|dot(n1, n2)| > 1 - eps). */
static inline bool ivp_hesse_is_parallel(const ivp_hesse_t *a,
                                          const ivp_hesse_t *b,
                                          ivp_real_t eps) {
    ivp_real_t d = ivp_vec3r_dot(&a->normal, &b->normal);
    ivp_real_t ad = d < IVP_REAL(0.0) ? -d : d;
    return ad > IVP_REAL(1.0) - eps;
}

/* ── Parametric line (Straight) ────────────────────────────────────────── */

/* Normalized direction vector + a point on the line. */
typedef struct ivp_straight {
    ivp_vec3r_t dir;          /* unit direction */
    ivp_vec3r_t start_point;  /* a point on the line */
} ivp_straight_t;

/* Initialize a straight from start point and direction (will be normalized). */
ivp_result_t ivp_straight_init(ivp_straight_t *s,
                                const ivp_vec3r_t *start_point,
                                const ivp_vec3r_t *direction);

/* Compute the orthogonal vector from a point to its projection on the line.
 * Result points from the projection to the point. */
void ivp_straight_ortho_vec(const ivp_straight_t *s,
                             const ivp_vec3r_t *point,
                             ivp_vec3r_t *out);

/* Squared distance from point to line. */
ivp_real_t ivp_straight_qdist_to_point(const ivp_straight_t *s,
                                        const ivp_vec3r_t *point);

/* Intersection of line with Hesse plane.
 * Returns IVP_OK on success (writes point_out).
 * Returns IVP_ERR_DEGENERATE_GEOMETRY if parallel. */
ivp_result_t ivp_hesse_intersect_straight(const ivp_hesse_t *h,
                                           const ivp_straight_t *s,
                                           ivp_vec3r_t *point_out);

/* Intersection of two lines in 3D.
 * Returns type and nearest point (on line 1) and squared distance.
 * If lines are skew, returns IVP_INTERSECT_NO_INTERSECTION with qdist. */
ivp_intersect_type_t ivp_straight_intersect(const ivp_straight_t *s1,
                                             const ivp_straight_t *s2,
                                             ivp_vec3r_t *point_out,
                                             ivp_real_t *qdist_out);

/* ── Plane with spanning vectors ───────────────────────────────────────── */

/* Full plane representation: Hesse normal + start point + two spanning vectors. */
typedef struct ivp_plain {
    ivp_hesse_t   hesse;
    ivp_vec3r_t   start_point;
    ivp_vec3r_t   vec1;
    ivp_vec3r_t   vec2;
} ivp_plain_t;

/* Initialize from Hesse normal form. Computes spanning vectors automatically. */
ivp_result_t ivp_plain_init_from_hesse(ivp_plain_t *out, const ivp_hesse_t *hesse);

/* Initialize from three points. */
ivp_result_t ivp_plain_init_from_points(ivp_plain_t *out,
                                         const ivp_vec3r_t *p0,
                                         const ivp_vec3r_t *p1,
                                         const ivp_vec3r_t *p2);

/* Intersection of two planes. Returns a line (straight).
 * Returns IVP_ERR_DEGENERATE_GEOMETRY if parallel. */
ivp_result_t ivp_plain_intersect(const ivp_plain_t *plane1,
                                  const ivp_hesse_t *plane2,
                                  ivp_straight_t *straight_out);

/* ── Utility functions ─────────────────────────────────────────────────── */

/* Compute an orthogonal vector to a given vector.
 * Finds the axis with smallest component and crosses with it. */
void ivp_vec3r_calc_orthogonal(ivp_vec3r_t *out, const ivp_vec3r_t *v);

/* Project point onto line, returning the parameter t such that
 * projection = start + t * dir. */
static inline ivp_real_t ivp_straight_project(const ivp_straight_t *s,
                                               const ivp_vec3r_t *point) {
    ivp_vec3r_t diff;
    ivp_vec3r_sub(&diff, point, &s->start_point);
    return ivp_vec3r_dot(&s->dir, &diff);
}

/* Closest point on line to a given point. */
static inline void ivp_straight_closest_point(const ivp_straight_t *s,
                                               const ivp_vec3r_t *point,
                                               ivp_vec3r_t *out) {
    ivp_real_t t = ivp_straight_project(s, point);
    out->k[0] = s->start_point.k[0] + s->dir.k[0] * t;
    out->k[1] = s->start_point.k[1] + s->dir.k[1] * t;
    out->k[2] = s->start_point.k[2] + s->dir.k[2] * t;
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_GEOMETRY_H */
