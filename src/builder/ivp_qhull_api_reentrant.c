/* ivp_qhull_api_reentrant.c -- Qhull libqhull_r adapter. */
#include "ivp_qhull_api.h"

#include <libqhull_r/qhull_ra.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

struct ivp_qhull_context {
    qhT qh;
    bool memory_initialized;
    bool hull_active;
};

ivp_qhull_context_t *ivp_qhull_context_create(void)
{
    ivp_qhull_context_t *context =
        (ivp_qhull_context_t *)calloc(1, sizeof(*context));
    if (!context) return NULL;

    QHULL_LIB_CHECK
    qh_zero(&context->qh, stderr);
    return context;
}

int ivp_qhull_new_hull(ivp_qhull_context_t *context, double *points,
                       int n_points, const char *flags)
{
    char command[256];
    size_t length;

    if (!context || !points || !flags || context->hull_active)
        return qh_ERRinput;
    length = strlen(flags);
    if (length >= sizeof(command)) return qh_ERRinput;
    memcpy(command, flags, length + 1u);

    context->memory_initialized = true;
    context->hull_active = true;
    return qh_new_qhull(&context->qh, 3, n_points, points, False,
                        command, NULL, stderr);
}

int ivp_qhull_for_each_facet(ivp_qhull_context_t *context,
                             ivp_qhull_facet_fn fn, void *callback_context)
{
    qhT *qh;
    facetT *facet;
    int facet_count = 0;
    int capacity = 0;
    int *point_ids = NULL;

    if (!context || !context->hull_active || !fn) return 0;
    qh = &context->qh;

    FORALLfacets {
        vertexT *vertex, **vertexp;
        setT *vertices = qh_facet3vertex(qh, facet);
        int count = qh_setsize(qh, vertices);
        double normal[3];
        int index = 0;

        if (count > capacity) {
            int *new_ids =
                (int *)realloc(point_ids, (size_t)count * sizeof(*point_ids));
            if (!new_ids) {
                qh_settempfree(qh, &vertices);
                free(point_ids);
                return -1;
            }
            point_ids = new_ids;
            capacity = count;
        }
        FOREACHvertex_(vertices) {
            point_ids[index++] = qh_pointid(qh, vertex->point);
        }
        qh_settempfree(qh, &vertices);
        normal[0] = facet->normal[0];
        normal[1] = facet->normal[1];
        normal[2] = facet->normal[2];
        fn(callback_context, normal, point_ids, index);
        facet_count++;
    }
    free(point_ids);
    return facet_count;
}

int ivp_qhull_for_each_vertex(ivp_qhull_context_t *context,
                              ivp_qhull_vertex_fn fn,
                              void *callback_context)
{
    qhT *qh;
    vertexT *vertex;
    int vertex_count = 0;

    if (!context || !context->hull_active || !fn) return 0;
    qh = &context->qh;
    FORALLvertices {
        fn(callback_context, qh_pointid(qh, vertex->point));
        vertex_count++;
    }
    return vertex_count;
}

void ivp_qhull_free_hull(ivp_qhull_context_t *context)
{
    if (!context || !context->hull_active) return;
    qh_freeqhull(&context->qh, !qh_ALL);
    context->hull_active = false;
}

void ivp_qhull_context_destroy(ivp_qhull_context_t *context)
{
    int current_long;
    int total_long;

    if (!context) return;
    ivp_qhull_free_hull(context);
    if (context->memory_initialized)
        qh_memfreeshort(&context->qh, &current_long, &total_long);
    free(context);
}
