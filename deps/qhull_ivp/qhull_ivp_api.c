/* qhull_ivp_api.c -- see qhull_ivp_api.h */
#include "qhull_a.h"
#include "qhull_ivp_api.h"

int qh_ivp_new_hull(double *points, int n_points, const char *flags)
{
  char cmd[256];
  size_t len = strlen(flags);
  if (len >= sizeof(cmd))
    return qh_ERRinput;
  memcpy(cmd, flags, len + 1);
  /* ivp_surbuild_pointsoup.cxx: outfile = errfile = stderr (qh_meminit()
   * latches the errfile of the first call) */
  return qh_new_qhull(3, n_points, points, False, cmd, stderr, stderr);
}

int qh_ivp_for_each_facet(qh_ivp_facet_fn fn, void *ctx)
{
  facetT *facet;
  int n_facets = 0;
  int cap = 0;
  int *ids = NULL;

  FORALLfacets {
    vertexT *vertex, **vertexp;
    setT *vertices = qh_facet3vertex(facet);
    int n = qh_setsize(vertices);
    double normal[3];
    int k = 0;
    if (n > cap) {
      int *nids = (int *)realloc(ids, (size_t)n * sizeof(int));
      if (!nids) {
        qh_settempfree(&vertices);
        free(ids);
        return -1;
      }
      ids = nids;
      cap = n;
    }
    FOREACHvertex_(vertices) {
      ids[k++] = qh_pointid(vertex->point);
    }
    qh_settempfree(&vertices);
    normal[0] = facet->normal[0];
    normal[1] = facet->normal[1];
    normal[2] = facet->normal[2];
    fn(ctx, normal, ids, k);
    n_facets++;
  }
  free(ids);
  return n_facets;
}

int qh_ivp_for_each_vertex(qh_ivp_vertex_fn fn, void *ctx)
{
  vertexT *vertex;
  int n_vertices = 0;

  FORALLvertices {
    fn(ctx, qh_pointid(vertex->point));
    n_vertices++;
  }
  return n_vertices;
}

void qh_ivp_free_hull(void)
{
  qh_freeqhull(!qh_ALL);
}

void qh_ivp_free_short_memory(void)
{
  int curlong, totlong;
  qh_memfreeshort(&curlong, &totlong);
}
