/* ivp_qhull_api.h -- narrow Qhull adapter used by the convex builder.
 *
 * Qhull headers stay private to one of the two adapter implementations.  The
 * reentrant backend gives every context independent Qhull state.  The classic
 * backend guards its process-global state for the complete context lifetime.
 */
#ifndef IVP_QHULL_API_H
#define IVP_QHULL_API_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ivp_qhull_context ivp_qhull_context_t;

ivp_qhull_context_t *ivp_qhull_context_create(void);

/* `points` is referenced, not copied, and must remain valid until
 * ivp_qhull_free_hull().  Returns Qhull's exit code (zero on success). */
int ivp_qhull_new_hull(ivp_qhull_context_t *context, double *points,
                       int n_points, const char *flags);

typedef void (*ivp_qhull_facet_fn)(void *context, const double normal[3],
                                   const int *point_ids, int point_count);

int ivp_qhull_for_each_facet(ivp_qhull_context_t *context,
                             ivp_qhull_facet_fn fn, void *callback_context);

typedef void (*ivp_qhull_vertex_fn)(void *context, int point_id);

int ivp_qhull_for_each_vertex(ivp_qhull_context_t *context,
                              ivp_qhull_vertex_fn fn,
                              void *callback_context);

void ivp_qhull_free_hull(ivp_qhull_context_t *context);
void ivp_qhull_context_destroy(ivp_qhull_context_t *context);

#if defined(IVP_QHULL_TEST_HOOKS)
/* Test seam at the backend-dispatch boundary.  Install or remove the hook
 * only while no hull build is active. */
typedef void (*ivp_qhull_test_hook_fn)(void *context);
void ivp_qhull_set_test_before_build_hook(ivp_qhull_test_hook_fn hook,
                                          void *context);
#endif

#ifdef __cplusplus
}
#endif

#endif /* IVP_QHULL_API_H */
