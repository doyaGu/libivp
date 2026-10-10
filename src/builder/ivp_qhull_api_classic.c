/* ivp_qhull_api_classic.c -- adapter for classic process-global Qhull. */
#include "ivp_qhull_api.h"

#include <qhull_a.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#include <windows.h>
static SRWLOCK s_qhull_lock = SRWLOCK_INIT;
static void qhull_lock(void) { AcquireSRWLockExclusive(&s_qhull_lock); }
static void qhull_unlock(void) { ReleaseSRWLockExclusive(&s_qhull_lock); }
#else
#include <pthread.h>
static pthread_mutex_t s_qhull_lock = PTHREAD_MUTEX_INITIALIZER;
static void qhull_lock(void) { (void)pthread_mutex_lock(&s_qhull_lock); }
static void qhull_unlock(void) { (void)pthread_mutex_unlock(&s_qhull_lock); }
#endif

struct ivp_qhull_context {
    bool memory_initialized;
    bool hull_active;
};

/* Classic Qhull expects its embedding application to provide this label. */
char qh_version[] = "version 3.0 2001/02/11";

ivp_qhull_context_t *ivp_qhull_context_create(void)
{
    ivp_qhull_context_t *context;

    qhull_lock();
    context = (ivp_qhull_context_t *)calloc(1, sizeof(*context));
    if (!context) qhull_unlock();
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
    return qh_new_qhull(3, n_points, points, False, command, NULL, stderr);
}

int ivp_qhull_for_each_facet(ivp_qhull_context_t *context,
                             ivp_qhull_facet_fn fn, void *callback_context)
{
    facetT *facet;
    int facet_count = 0;
    int capacity = 0;
    int *point_ids = NULL;

    if (!context || !context->hull_active || !fn) return 0;

    FORALLfacets {
        vertexT *vertex, **vertexp;
        setT *vertices = qh_facet3vertex(facet);
        int count = qh_setsize(vertices);
        double normal[3];
        int index = 0;

        if (count > capacity) {
            int *new_ids =
                (int *)realloc(point_ids, (size_t)count * sizeof(*point_ids));
            if (!new_ids) {
                qh_settempfree(&vertices);
                free(point_ids);
                return -1;
            }
            point_ids = new_ids;
            capacity = count;
        }
        FOREACHvertex_(vertices) {
            point_ids[index++] = qh_pointid(vertex->point);
        }
        qh_settempfree(&vertices);
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
    vertexT *vertex;
    int vertex_count = 0;

    if (!context || !context->hull_active || !fn) return 0;
    FORALLvertices {
        fn(callback_context, qh_pointid(vertex->point));
        vertex_count++;
    }
    return vertex_count;
}

void ivp_qhull_free_hull(ivp_qhull_context_t *context)
{
    if (!context || !context->hull_active) return;
    qh_freeqhull(!qh_ALL);
    context->hull_active = false;
}

void ivp_qhull_context_destroy(ivp_qhull_context_t *context)
{
    int current_long;
    int total_long;

    if (!context) return;
    ivp_qhull_free_hull(context);
    if (context->memory_initialized)
        qh_memfreeshort(&current_long, &total_long);
    free(context);
    qhull_unlock();
}
