/* ivp_mindist.h -- Minimum distance between convex ledges
 *
 * Finds the closest pair of geometric features (vertex, edge, or triangle)
 * between two convex compact ledges and returns distance + contact info.
 *
 * This is the core collision narrowphase:
 *   Given two convex shapes (in the same coordinate frame),
 *   find their minimum distance and closest features.
 *
 * Feature types follow original IVP conventions:
 *   P (Point/Vertex), K (Kante/Edge), F (Face/Triangle), B (Ball)
 *
 * Ported from ivp_mindist.hxx / ivp_mindist_mcases.cxx /
 *            ivp_mindist_minimize.cxx.
 */
#ifndef IVP_MINDIST_H
#define IVP_MINDIST_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/collision/ivp_narrowphase.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Feature type ──────────────────────────────────────────────────────── */

typedef enum ivp_feature_type {
    IVP_FEATURE_POINT    = 0,   /* vertex */
    IVP_FEATURE_EDGE     = 1,   /* edge (between two vertices) */
    IVP_FEATURE_TRIANGLE = 2    /* face/triangle */
} ivp_feature_type_t;

typedef enum ivp_mindist_status {
    IVP_MINDIST_STATUS_OK = 0,
    IVP_MINDIST_STATUS_BACKSIDE,
    IVP_MINDIST_STATUS_LOOP,
    IVP_MINDIST_STATUS_MAX_ITER
} ivp_mindist_status_t;

/* ── Closest feature pair result ───────────────────────────────────────── */

typedef struct ivp_closest_pair {
    ivp_mindist_status_t status;

    /* Feature types for each ledge. */
    ivp_feature_type_t type_a;
    ivp_feature_type_t type_b;

    /* Reference edges for the closest features.
     * For POINT: the edge whose start vertex is closest.
     * For EDGE: the closest edge itself.
     * For TRIANGLE: any edge of the closest triangle. */
    const ivp_compact_edge_t *edge_a;
    const ivp_compact_edge_t *edge_b;

    /* Closest points on each ledge (in the coordinate frame used). */
    ivp_vec3r_t point_a;
    ivp_vec3r_t point_b;

    /* Contact normal (unit vector from A to B).
     * If distance is zero/negative, this may be approximate. */
    ivp_vec3r_t normal;

    /* Squared distance between closest points. */
    ivp_real_t qdistance;

    /* Signed distance (positive = separated, negative = penetrating).
     * Equal to sqrt(qdistance) when separated. */
    ivp_real_t distance;
} ivp_closest_pair_t;

/* ── Find closest features between two convex ledges ───────────────────── */

/* Find the minimum distance between two convex compact ledges.
 *
 * Both ledges must be expressed in the same coordinate frame.
 * For world-space queries, transform one ledge's vertices into the
 * other's object space first.
 *
 * Parameters:
 *   ledge_a  -- first convex ledge
 *   ledge_b  -- second convex ledge
 *   result   -- output: closest pair information
 *
 * Returns:
 *   IVP_OK on success.
 *   IVP_ERR_INVALID_ARG if ledge_a or ledge_b is NULL.
 */
ivp_result_t ivp_find_closest_pair(
    const ivp_compact_ledge_t *ledge_a,
    const ivp_compact_ledge_t *ledge_b,
    ivp_closest_pair_t *result);

/* Convenience: squared distance between two convex ledges.
 * Both ledges must be in the same coordinate frame.
 * Returns IVP_REAL_MAX on error. */
ivp_real_t ivp_qdist_ledge_ledge(
    const ivp_compact_ledge_t *ledge_a,
    const ivp_compact_ledge_t *ledge_b);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MINDIST_H */
