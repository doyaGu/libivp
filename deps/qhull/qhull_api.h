/* qhull_api.h -- minimal reentrant interface to Qhull 2020.2
 *
 * The convex pointsoup builder only needs to construct a 3-D hull, walk
 * its facets and vertices, and release it.  Qhull's public headers remain
 * private to this adapter so the rest of libivp does not depend on its
 * types or warning policy.
 *
 * Each qh_api_context_t owns one independent qhT.  Different contexts may
 * be used concurrently.  A single context must only be used by one thread
 * at a time, as required by libqhull_r.
 */
#ifndef QHULL_API_H
#define QHULL_API_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct qh_api_context qh_api_context_t;

/* Allocate and initialize an independent Qhull context. */
qh_api_context_t *qh_api_context_create(void);

/* qh_new_qhull(qh, 3, n_points, points, False, flags, NULL, stderr).
 * `points` is referenced, not copied, and must stay valid until
 * qh_api_free_hull().  Returns Qhull's exit code (0 = success). */
int qh_api_new_hull(qh_api_context_t *context, double *points,
                    int n_points, const char *flags);

typedef void (*qh_api_facet_fn)(void *ctx, const double normal[3],
                                const int *point_ids, int n_ids);

/* Walk facets in Qhull's FORALLfacets order. */
int qh_api_for_each_facet(qh_api_context_t *context,
                          qh_api_facet_fn fn, void *ctx);

typedef void (*qh_api_vertex_fn)(void *ctx, int point_id);

/* Walk vertices in Qhull's FORALLvertices order. */
int qh_api_for_each_vertex(qh_api_context_t *context,
                           qh_api_vertex_fn fn, void *ctx);

/* Release the long-lived allocations for the current hull. */
void qh_api_free_hull(qh_api_context_t *context);

/* Release the current hull, Qhull's short-memory pool, and the context. */
void qh_api_context_destroy(qh_api_context_t *context);

#ifdef __cplusplus
}
#endif

#endif /* QHULL_API_H */
