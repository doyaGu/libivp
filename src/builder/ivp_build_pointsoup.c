/* ivp_build_pointsoup.c -- Point soup builder implementation
 *
 * IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_surface
 * (ivp_surbuild_pointsoup.cxx:829).  Exact duplicates are removed by the
 * convex builder itself; weld_epsilon > 0 additionally merges near points.
 */

#include <ivp/builder/ivp_build_pointsoup.h>
#include <ivp/builder/ivp_build_surface.h>
#include <string.h>

ivp_compact_surface_t *ivp_build_pointsoup_surface(
    const ivp_allocator_t *alloc,
    const float *coords,
    int n_points,
    const ivp_pointsoup_options_t *options)
{
    if (!coords || n_points < 3) return NULL;
    if (!alloc) alloc = ivp_default_allocator();

    ivp_pointsoup_options_t opts = options ? *options : ivp_pointsoup_options_default();

    size_t sz = (size_t)n_points * 3u * sizeof(double);
    double *pts = (double *)ivp_alloc(alloc, sz);
    if (!pts) return NULL;
    for (int i = 0; i < 3 * n_points; i++) pts[i] = (double)coords[i];

    int count = n_points;
    if (opts.weld_epsilon > 0.0f) {
        /* keep the first of every cluster, preserve input order */
        double eps2 = (double)opts.weld_epsilon * (double)opts.weld_epsilon;
        int out = 0;
        for (int i = 0; i < count; i++) {
            bool dup = false;
            for (int j = 0; j < out; j++) {
                double dx = pts[3 * i] - pts[3 * j];
                double dy = pts[3 * i + 1] - pts[3 * j + 1];
                double dz = pts[3 * i + 2] - pts[3 * j + 2];
                if (dx * dx + dy * dy + dz * dz < eps2) {
                    dup = true;
                    break;
                }
            }
            if (!dup) {
                memmove(&pts[3 * out], &pts[3 * i], 3 * sizeof(double));
                out++;
            }
        }
        count = out;
    }

    if (opts.center_points && count > 0) {
        double c[3] = { 0.0, 0.0, 0.0 };
        for (int i = 0; i < count; i++) {
            c[0] += pts[3 * i];
            c[1] += pts[3 * i + 1];
            c[2] += pts[3 * i + 2];
        }
        for (int k = 0; k < 3; k++) c[k] /= (double)count;
        for (int i = 0; i < count; i++) {
            pts[3 * i] -= c[0];
            pts[3 * i + 1] -= c[1];
            pts[3 * i + 2] -= c[2];
        }
    }

    ivp_compact_surface_t *surface = ivp_build_convex_surface_d(alloc, pts, count);
    ivp_free(alloc, pts, sz);
    return surface;
}
