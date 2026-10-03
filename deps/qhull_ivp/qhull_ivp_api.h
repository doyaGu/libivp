/* qhull_ivp_api.h -- minimal interface of the vendored IVP qhull 2.6
 *
 * libivp's convex pointsoup builder (src/builder/ivp_build_convex_qhull.c)
 * only needs what IVP_SurfaceBuilder_Pointsoup uses from qhull: build a
 * hull with a flag string, walk the facets (FORALLfacets order) with their
 * normals and their qh_facet3vertex() vertex order, and free the result.
 * Keeping the qhull headers behind this interface lets the builder compile
 * with the project's strict warning set.
 *
 * NOT REENTRANT: qhull 2.6 keeps all of its state in the global `qh_qh`
 * structure (qh_QHpointer == 0) and in qhull's memory manager `qhmem`.
 * Only one hull may exist at a time and calls must not run concurrently.
 */
#ifndef QHULL_IVP_API_H
#define QHULL_IVP_API_H

#ifdef __cplusplus
extern "C" {
#endif

/* qh_new_qhull(3, n_points, points, False, flags, stderr, stderr), as
 * IVP_SurfaceBuilder_Pointsoup calls it (outfile and errfile = stderr on
 * every platform but PSXII/WIN32/LINUX/GEKKO, where the reference passes
 * NULL and therefore prints nothing).  `points` (n_points * 3 doubles) is
 * referenced, not copied, and must stay valid until qh_ivp_free_hull().
 * Returns qhull's exit code (0 = ok).  On errors (and for options that
 * request output) qhull writes like the reference: its fprintf(qh ferr)
 * text to stderr and its ivp_message() text to the message sink. */
int qh_ivp_new_hull(double *points, int n_points, const char *flags);

/* Sink for the text qhull passes to IVP's ivp_message(): the formatted text
 * (truncated to 9999 characters like ivu_string.cxx) without the "[havok] "
 * prefix.  NULL (the default) prints "[havok] " + text to stdout, which is
 * what the reference ivp_message() does. */
typedef void (*qh_ivp_message_fn)(const char *text);
void qh_ivp_set_message_sink(qh_ivp_message_fn fn);

/* Callback for qh_ivp_for_each_facet(): facet hyperplane normal and the
 * input point indices (qh_pointid) of the facet's vertices in
 * qh_facet3vertex() order. */
typedef void (*qh_ivp_facet_fn)(void *ctx, const double normal[3],
                                const int *point_ids, int n_ids);

/* Walk the facets of the current hull (FORALLfacets).  Returns the number
 * of facets visited, or -1 on allocation failure. */
int qh_ivp_for_each_facet(qh_ivp_facet_fn fn, void *ctx);

/* Callback for qh_ivp_for_each_vertex(): input point index (qh_pointid). */
typedef void (*qh_ivp_vertex_fn)(void *ctx, int point_id);

/* Walk the vertices of the current hull (FORALLvertices; the reference
 * pointsoup builder prints them in its debug build).  Returns the number of
 * vertices visited. */
int qh_ivp_for_each_vertex(qh_ivp_vertex_fn fn, void *ctx);

/* qh_freeqhull(!qh_ALL): free the long memory of the current hull. */
void qh_ivp_free_hull(void);

/* qh_memfreeshort(): release qhull's short memory pool. */
void qh_ivp_free_short_memory(void);

#ifdef __cplusplus
}
#endif

#endif /* QHULL_IVP_API_H */
