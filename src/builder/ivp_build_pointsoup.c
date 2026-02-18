/* ivp_build_pointsoup.c -- Point soup builder implementation */

#include <ivp/builder/ivp_build_pointsoup.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/math/ivp_vec3.h>
#include <string.h>
#include <math.h>

ivp_compact_surface_t *ivp_build_pointsoup_surface(
    const ivp_allocator_t *alloc,
    const float *coords,
    int n_points,
    const ivp_pointsoup_options_t *options)
{
    if (!coords || n_points < 4) return NULL;
    if (!alloc) alloc = ivp_default_allocator();

    ivp_pointsoup_options_t opts;
    if (options) opts = *options;
    else opts = ivp_pointsoup_options_default();

    /* Copy points into vec3f array for processing */
    ivp_vec3f_t *pts = (ivp_vec3f_t *)ivp_alloc(
        alloc, (size_t)n_points * sizeof(ivp_vec3f_t));
    if (!pts) return NULL;

    for (int i = 0; i < n_points; i++) {
        pts[i].k[0] = coords[i * 3 + 0];
        pts[i].k[1] = coords[i * 3 + 1];
        pts[i].k[2] = coords[i * 3 + 2];
    }

    /* Weld duplicate points */
    int welded_count = n_points;
    if (opts.weld_epsilon > 0.0f) {
        float eps2 = opts.weld_epsilon * opts.weld_epsilon;
        for (int i = 0; i < welded_count; i++) {
            for (int j = i + 1; j < welded_count; ) {
                float dx = pts[j].k[0] - pts[i].k[0];
                float dy = pts[j].k[1] - pts[i].k[1];
                float dz = pts[j].k[2] - pts[i].k[2];
                if (dx * dx + dy * dy + dz * dz < eps2) {
                    /* Remove duplicate by swapping with last */
                    welded_count--;
                    if (j < welded_count) {
                        pts[j] = pts[welded_count];
                    }
                } else {
                    j++;
                }
            }
        }
    }

    if (welded_count < 4) {
        ivp_free(alloc, pts, (size_t)n_points * sizeof(ivp_vec3f_t));
        return NULL;
    }

    /* Center points at centroid */
    float cx = 0.0f, cy = 0.0f, cz = 0.0f;
    if (opts.center_points) {
        for (int i = 0; i < welded_count; i++) {
            cx += pts[i].k[0];
            cy += pts[i].k[1];
            cz += pts[i].k[2];
        }
        float inv_n = 1.0f / (float)welded_count;
        cx *= inv_n;
        cy *= inv_n;
        cz *= inv_n;
        for (int i = 0; i < welded_count; i++) {
            pts[i].k[0] -= cx;
            pts[i].k[1] -= cy;
            pts[i].k[2] -= cz;
        }
    }

    /* Build convex hull */
    ivp_compact_surface_t *surface =
        ivp_build_convex_surface(alloc, pts, welded_count);

    /* Shift mass center back if we centered */
    if (surface && opts.center_points) {
        surface->mass_center[0] += cx;
        surface->mass_center[1] += cy;
        surface->mass_center[2] += cz;
    }

    ivp_free(alloc, pts, (size_t)n_points * sizeof(ivp_vec3f_t));
    return surface;
}
