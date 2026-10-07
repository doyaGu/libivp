/* qhull_api.c -- see qhull_api.h */
#include "libqhull_r/qhull_ra.h"
#include "qhull_api.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

struct qh_api_context {
    qhT qh;
    bool memory_initialized;
    bool hull_active;
};

qh_api_context_t *qh_api_context_create(void)
{
    qh_api_context_t *context = (qh_api_context_t *)calloc(1, sizeof(*context));
    if (!context) return NULL;

    QHULL_LIB_CHECK
    qh_zero(&context->qh, stderr);
    return context;
}

int qh_api_new_hull(qh_api_context_t *context, double *points,
                    int n_points, const char *flags)
{
    char command[256];
    size_t length;

    if (!context || !points || !flags || context->hull_active)
        return qh_ERRinput;
    length = strlen(flags);
    if (length >= sizeof(command))
        return qh_ERRinput;
    memcpy(command, flags, length + 1);

    context->memory_initialized = true;
    context->hull_active = true;
    return qh_new_qhull(&context->qh, 3, n_points, points, False,
                        command, NULL, stderr);
}

int qh_api_for_each_facet(qh_api_context_t *context,
                          qh_api_facet_fn fn, void *ctx)
{
    qhT *qh;
    facetT *facet;
    int n_facets = 0;
    int capacity = 0;
    int *ids = NULL;

    if (!context || !context->hull_active || !fn) return 0;
    qh = &context->qh;

    FORALLfacets {
        vertexT *vertex, **vertexp;
        setT *vertices = qh_facet3vertex(qh, facet);
        int count = qh_setsize(qh, vertices);
        double normal[3];
        int index = 0;

        if (count > capacity) {
            int *new_ids = (int *)realloc(ids, (size_t)count * sizeof(*ids));
            if (!new_ids) {
                qh_settempfree(qh, &vertices);
                free(ids);
                return -1;
            }
            ids = new_ids;
            capacity = count;
        }
        FOREACHvertex_(vertices) {
            ids[index++] = qh_pointid(qh, vertex->point);
        }
        qh_settempfree(qh, &vertices);
        normal[0] = facet->normal[0];
        normal[1] = facet->normal[1];
        normal[2] = facet->normal[2];
        fn(ctx, normal, ids, index);
        n_facets++;
    }
    free(ids);
    return n_facets;
}

int qh_api_for_each_vertex(qh_api_context_t *context,
                           qh_api_vertex_fn fn, void *ctx)
{
    qhT *qh;
    vertexT *vertex;
    int n_vertices = 0;

    if (!context || !context->hull_active || !fn) return 0;
    qh = &context->qh;
    FORALLvertices {
        fn(ctx, qh_pointid(qh, vertex->point));
        n_vertices++;
    }
    return n_vertices;
}

void qh_api_free_hull(qh_api_context_t *context)
{
    if (!context || !context->hull_active) return;
    qh_freeqhull(&context->qh, !qh_ALL);
    context->hull_active = false;
}

void qh_api_context_destroy(qh_api_context_t *context)
{
    int curlong;
    int totlong;

    if (!context) return;
    qh_api_free_hull(context);
    if (context->memory_initialized)
        qh_memfreeshort(&context->qh, &curlong, &totlong);
    free(context);
}
