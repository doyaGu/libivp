/* ivp_convex_decompose.c -- Convex decomposition of concave polyhedra
 *
 * Port of IVP_Convex_Decompositor::perform_convex_decomposition_on_concave_
 * polyhedron (ivp_compact_builder/ivp_convex_decompositor.cxx) and of
 * IVP_SurfaceBuilder_Polyhedron_Concave::convert_concave_polyhedron_to_
 * compact_ledges (ivp_surbuild_polyhdrn_cncv.cxx).  The decomposition itself
 * is GEOMPACK (ivp_geompack.c).
 */
#include <ivp/builder/ivp_convex_decompose.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/builder/ivp_build_ledge.h>
#include "ivp_geompack_internal.h"

#include <ivp/base/ivp_assert.h>
#include <ivp/diagnostics/ivp_debug_manager.h>

#include <limits.h>
#include <string.h>

int ivp_convex_decompose_points(const ivp_allocator_t *alloc,
                                const double *points, int n_points,
                                const int *face_indices, const int *face_sizes, int n_faces,
                                const ivp_decompose_options_t *options,
                                double **out_part_points, int **out_part_offsets)
{
    if (!points || !face_indices || !face_sizes || n_points < 4 || n_faces < 4 ||
        !out_part_points || !out_part_offsets) {
        return -1;
    }
    if (!alloc) alloc = ivp_default_allocator();
    ivp_decompose_options_t opts = options ? *options : ivp_decompose_options_default();
    *out_part_points = NULL;
    *out_part_offsets = NULL;

    long fvl_length = 0;
    for (int i = 0; i < n_faces; i++) {
        if (face_sizes[i] < 3) return -1;
        fvl_length += face_sizes[i];
        if (fvl_length > INT_MAX / 8) return -1;
    }

    /* vertex coordinate list (capacity nvc + 2 like size_vcl) */
    ivp_geompack_parameters_t gp;
    memset(&gp, 0, sizeof(gp));
    gp.vcl_cap_bytes = (size_t)(n_points + 2) * 3u * sizeof(double);
    gp.vcl = (double *)ivp_alloc(alloc, gp.vcl_cap_bytes);
    gp.facep_cap_bytes = (size_t)(n_faces + 2) * 3u * sizeof(int);
    gp.facep = (int *)ivp_alloc(alloc, gp.facep_cap_bytes);
    size_t fvl_sz = (size_t)fvl_length * sizeof(int);
    int *fvl = (int *)ivp_alloc(alloc, fvl_sz);
    if (!gp.vcl || !gp.facep || !fvl) {
        if (!gp.vcl) {
            IVP_IFDEBUG(IVP_DEBUG_IPION_ERROR_MSG) {
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DEBUG_IPION_ERROR_MSG,
                                                "IVP_Convex_Decompositor::perform_convex_decomposition_on_concave_polyhedron()\n"
                                                "Not enough memory to allocate 'points_list' (needed: %d bytes)\n",
                                                (int)((size_t)n_points * 3u * sizeof(float)));
            }
        }
        if (gp.vcl) ivp_free(alloc, gp.vcl, gp.vcl_cap_bytes);
        if (gp.facep) ivp_free(alloc, gp.facep, gp.facep_cap_bytes);
        if (fvl) ivp_free(alloc, fvl, fvl_sz);
        return -1;
    }
    memset(gp.vcl, 0, gp.vcl_cap_bytes);
    memset(gp.facep, 0, gp.facep_cap_bytes);
    memcpy(gp.vcl, points, (size_t)n_points * 3u * sizeof(double));

    int entry_ctr = 0;
    int off = 0;
    int i;
    for (i = 0; i < n_faces; i++) {
        gp.facep[i * 3] = entry_ctr + 1;
        for (int j = 0; j < face_sizes[i]; j++) {
            int v = face_indices[off++];
            if (v < 0 || v >= n_points) {
                ivp_free(alloc, gp.vcl, gp.vcl_cap_bytes);
                ivp_free(alloc, gp.facep, gp.facep_cap_bytes);
                ivp_free(alloc, fvl, fvl_sz);
                return -1;
            }
            fvl[entry_ctr++] = v + 1;
        }
    }
    gp.facep[i * 3] = entry_ctr + 1;

    gp.tolin = (double)opts.tolin;
    gp.angacc = (double)opts.angacc;
    gp.rdacc = (double)opts.rdacc;
    gp.nvc = n_points;
    gp.nface = n_faces;
    gp.fvl = fvl;

    ivp_geompack_decompose(alloc, &gp);
    ivp_free(alloc, fvl, fvl_sz);

    /* process GEOMPACK results: unique vertex offsets per subpart, in
     * order of appearance in the face vertex list */
    int npolh = gp.npolh_out;
    int nvert = gp.nvert_out;
    int result = -1;
    int *offsets = NULL;      /* per part start (npolh + 1) */
    int *members = NULL;      /* vertex index lists */
    unsigned char *seen = NULL;
    double *pts = NULL;
    size_t offsets_sz = 0, members_sz = 0, seen_sz = 0, pts_sz = 0;

    if (npolh <= 0 || !gp.fvl_out || !gp.facep_out || !gp.vcl_out) {
        result = 0;
        goto done;
    }

    offsets_sz = (size_t)(npolh + 1) * sizeof(int);
    members_sz = (size_t)(nvert > 0 ? nvert : 1) * sizeof(int);
    seen_sz = (size_t)npolh * (size_t)(gp.nvc_out + 1);
    offsets = (int *)ivp_alloc(alloc, offsets_sz);
    members = (int *)ivp_alloc(alloc, members_sz);
    seen = (unsigned char *)ivp_alloc(alloc, seen_sz);
    if (!offsets || !members || !seen) goto done;
    memset(seen, 0, seen_sz);

    /* two passes: count unique members per part, then fill */
    for (int p = 0; p <= npolh; p++) offsets[p] = 0;
    for (int k = 0; k < nvert; k++) {
        int face_number = gp.fvl_out[k * 6 + 1] - 1;
        if (face_number < 0 || face_number >= gp.nface_out) continue;
        int subpart = gp.facep_out[face_number * 3 + 1];
        if (subpart < 0) subpart = -subpart;
        subpart--;
        int v = gp.fvl_out[k * 6 + 0];
        if (subpart < 0 || subpart >= npolh || v < 1 || v > gp.nvc_out) continue;
        unsigned char *flag = &seen[(size_t)subpart * (size_t)(gp.nvc_out + 1) + (size_t)v];
        if (*flag) continue;
        *flag = 1;
        offsets[subpart + 1]++;
    }
    for (int p = 0; p < npolh; p++) offsets[p + 1] += offsets[p];
    int total = offsets[npolh];
    memset(seen, 0, seen_sz);
    {
        int *fill = (int *)ivp_alloc(alloc, offsets_sz);
        if (!fill) goto done;
        memcpy(fill, offsets, offsets_sz);
        for (int k = 0; k < nvert; k++) {
            int face_number = gp.fvl_out[k * 6 + 1] - 1;
            if (face_number < 0 || face_number >= gp.nface_out) continue;
            int subpart = gp.facep_out[face_number * 3 + 1];
            if (subpart < 0) subpart = -subpart;
            subpart--;
            int v = gp.fvl_out[k * 6 + 0];
            if (subpart < 0 || subpart >= npolh || v < 1 || v > gp.nvc_out) continue;
            unsigned char *flag = &seen[(size_t)subpart * (size_t)(gp.nvc_out + 1) + (size_t)v];
            if (*flag) continue;
            *flag = 1;
            members[fill[subpart]++] = v;
        }
        ivp_free(alloc, fill, offsets_sz);
    }

    pts_sz = (size_t)(total > 0 ? total : 1) * 3u * sizeof(double);
    pts = (double *)ivp_alloc(alloc, pts_sz);
    if (!pts) goto done;
    for (int k = 0; k < total; k++) {
        int v = members[k] - 1;
        pts[3 * k + 0] = gp.vcl_out[v * 3 + 0];
        pts[3 * k + 1] = gp.vcl_out[v * 3 + 1];
        pts[3 * k + 2] = gp.vcl_out[v * 3 + 2];
    }
    /* ivp_convex_decompositor.cxx:305/317: per subpart the header, then its
     * points in the order they are added to the subpart's pointsoup */
    for (int p = 0; p < npolh; p++) {
        IVP_IF(1) {
            IVP_IFDEBUG(IVP_DM_CONVEX_DECOMPOSITOR) {
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_CONVEX_DECOMPOSITOR, "Points of convex subpart:\n");
            }
        }
        for (int k = offsets[p]; k < offsets[p + 1]; k++) {
            IVP_IF(1) {
                IVP_IFDEBUG(IVP_DM_CONVEX_DECOMPOSITOR) {
                    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_CONVEX_DECOMPOSITOR, "   %.6f %.6f %.6f\n",
                                                    pts[3 * k + 0], pts[3 * k + 1], pts[3 * k + 2]);
                }
            }
        }
    }

    *out_part_points = pts;
    *out_part_offsets = offsets;
    pts = NULL;
    offsets = NULL;
    result = npolh;

done:
    if (gp.facep_out) ivp_free(alloc, gp.facep_out, gp.facep_out_cap);
    if (gp.fvl_out) ivp_free(alloc, gp.fvl_out, gp.fvl_out_cap);
    if (gp.vcl_out) ivp_free(alloc, gp.vcl_out, gp.vcl_out_cap);
    if (offsets) ivp_free(alloc, offsets, offsets_sz);
    if (members) ivp_free(alloc, members, members_sz);
    if (seen) ivp_free(alloc, seen, seen_sz);
    if (pts) ivp_free(alloc, pts, pts_sz);
    return result;
}

void ivp_decompose_points_free(const ivp_allocator_t *alloc, int n_parts,
                               double *part_points, int *part_offsets)
{
    if (!alloc) alloc = ivp_default_allocator();
    if (part_offsets && n_parts >= 0) {
        int total = part_offsets[n_parts];
        if (part_points) {
            ivp_free(alloc, part_points, (size_t)(total > 0 ? total : 1) * 3u * sizeof(double));
        }
        ivp_free(alloc, part_offsets, (size_t)(n_parts + 1) * sizeof(int));
    }
}

int ivp_convex_decompose_to_ledges(const ivp_allocator_t *alloc,
                                   const double *points, int n_points,
                                   const int *face_indices, const int *face_sizes, int n_faces,
                                   const ivp_decompose_options_t *options,
                                   ivp_compact_ledge_t ***out_ledges)
{
    if (!out_ledges) return -1;
    if (!alloc) alloc = ivp_default_allocator();
    *out_ledges = NULL;
    double *pp = NULL;
    int *po = NULL;
    int n = ivp_convex_decompose_points(alloc, points, n_points, face_indices, face_sizes,
                                        n_faces, options, &pp, &po);
    if (n <= 0) return n;
    ivp_compact_ledge_t **ledges =
        (ivp_compact_ledge_t **)ivp_alloc(alloc, (size_t)n * sizeof(*ledges));
    if (!ledges) {
        ivp_decompose_points_free(alloc, n, pp, po);
        return -1;
    }
    int n_out = 0;
    for (int p = 0; p < n; p++) {
        ivp_compact_ledge_t *l = ivp_build_convex_ledge_d(alloc, &pp[3 * po[p]], po[p + 1] - po[p]);
        if (l) ledges[n_out++] = l;
    }
    ivp_decompose_points_free(alloc, n, pp, po);
    if (n_out == 0) {
        ivp_free(alloc, ledges, (size_t)n * sizeof(*ledges));
        return 0;
    }
    if (n_out < n) {
        ivp_compact_ledge_t **shrunk =
            (ivp_compact_ledge_t **)ivp_alloc(alloc, (size_t)n_out * sizeof(*ledges));
        if (shrunk) {
            memcpy(shrunk, ledges, (size_t)n_out * sizeof(*ledges));
            ivp_free(alloc, ledges, (size_t)n * sizeof(*ledges));
            ledges = shrunk;
        } else {
            for (int i = 0; i < n_out; i++) ivp_compact_ledge_free(alloc, ledges[i]);
            ivp_free(alloc, ledges, (size_t)n * sizeof(*ledges));
            return -1;
        }
    }
    *out_ledges = ledges;
    return n_out;
}

void ivp_decompose_result_free(ivp_decompose_result_t *result)
{
    if (!result) return;
    if (result->parts) {
        const ivp_allocator_t *a = result->allocator ? result->allocator : ivp_default_allocator();
        for (int i = 0; i < result->n_parts; i++) {
            if (result->parts[i]) ivp_compact_surface_free(a, result->parts[i]);
        }
        ivp_free(a, result->parts, (size_t)result->n_parts * sizeof(ivp_compact_surface_t *));
    }
    result->parts = NULL;
    result->n_parts = 0;
}

ivp_result_t ivp_convex_decompose(const ivp_allocator_t *alloc,
                                  const float *vertices,
                                  int n_vertices,
                                  const int *indices,
                                  int n_triangles,
                                  const ivp_decompose_options_t *options,
                                  ivp_decompose_result_t *result)
{
    if (!result) return IVP_ERR_INVALID_ARG;
    memset(result, 0, sizeof(*result));
    if (!vertices || !indices || n_vertices < 4 || n_triangles < 4) return IVP_ERR_INVALID_ARG;
    if (!alloc) alloc = ivp_default_allocator();
    result->allocator = alloc;

    size_t pts_sz = (size_t)n_vertices * 3u * sizeof(double);
    size_t sizes_sz = (size_t)n_triangles * sizeof(int);
    double *pts = (double *)ivp_alloc(alloc, pts_sz);
    int *sizes = (int *)ivp_alloc(alloc, sizes_sz);
    if (!pts || !sizes) {
        if (pts) ivp_free(alloc, pts, pts_sz);
        if (sizes) ivp_free(alloc, sizes, sizes_sz);
        return IVP_ERR_OUT_OF_MEMORY;
    }
    for (int i = 0; i < 3 * n_vertices; i++) pts[i] = (double)vertices[i];
    for (int i = 0; i < n_triangles; i++) sizes[i] = 3;

    ivp_compact_ledge_t **ledges = NULL;
    int n = ivp_convex_decompose_to_ledges(alloc, pts, n_vertices, indices, sizes, n_triangles,
                                           options, &ledges);
    ivp_free(alloc, pts, pts_sz);
    ivp_free(alloc, sizes, sizes_sz);
    if (n < 0) return IVP_ERR_OUT_OF_MEMORY;
    if (n == 0) return IVP_ERR_DEGENERATE_GEOMETRY;

    result->parts = (ivp_compact_surface_t **)ivp_alloc(alloc, (size_t)n * sizeof(ivp_compact_surface_t *));
    if (!result->parts) {
        for (int i = 0; i < n; i++) ivp_compact_ledge_free(alloc, ledges[i]);
        ivp_free(alloc, ledges, (size_t)n * sizeof(*ledges));
        return IVP_ERR_OUT_OF_MEMORY;
    }
    int n_parts = 0;
    for (int i = 0; i < n; i++) {
        ivp_compact_surface_t *s = ivp_build_surface_from_ledge(alloc, ledges[i]);
        if (s) result->parts[n_parts++] = s;
        ivp_compact_ledge_free(alloc, ledges[i]);
    }
    ivp_free(alloc, ledges, (size_t)n * sizeof(*ledges));
    if (n_parts == 0) {
        ivp_free(alloc, result->parts, (size_t)n * sizeof(ivp_compact_surface_t *));
        result->parts = NULL;
        return IVP_ERR_DEGENERATE_GEOMETRY;
    }
    if (n_parts < n) {
        /* keep the allocation size consistent with n_parts for freeing */
        ivp_compact_surface_t **shrunk =
            (ivp_compact_surface_t **)ivp_alloc(alloc, (size_t)n_parts * sizeof(*shrunk));
        if (!shrunk) {
            for (int i = 0; i < n_parts; i++) ivp_compact_surface_free(alloc, result->parts[i]);
            ivp_free(alloc, result->parts, (size_t)n * sizeof(ivp_compact_surface_t *));
            result->parts = NULL;
            return IVP_ERR_OUT_OF_MEMORY;
        }
        memcpy(shrunk, result->parts, (size_t)n_parts * sizeof(*shrunk));
        ivp_free(alloc, result->parts, (size_t)n * sizeof(ivp_compact_surface_t *));
        result->parts = shrunk;
    }
    result->n_parts = n_parts;
    return IVP_OK;
}
