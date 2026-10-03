/* ivp_check_ledge_debug.h — IVP_Compact_Ledge_Solver::check_ledge
 *
 * The reference compiles check_ledge() only in its debug build
 * (ivp_collision/ivp_compact_ledge_solver.cxx:787, #ifdef DEBUG) and calls
 * it from #ifdef DEBUG blocks of the pointsoup builder, the grid builder
 * and the grid surface manager.  It only asserts (alignment of the ledge,
 * its point array and its triangles, opposite edge links, convexity and the
 * edge round trip), nothing is changed.
 *
 * Internal header of the builder / geometry modules; only defined in
 * debug builds (IVP_DEBUG_BUILD), like the reference "#ifdef DEBUG".
 */
#ifndef IVP_CHECK_LEDGE_DEBUG_H
#define IVP_CHECK_LEDGE_DEBUG_H

#include <ivp/base/ivp_assert.h>

#ifdef IVP_DEBUG_BUILD

#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_compact_poly_point.h>
#include <ivp/geometry/ivp_compact_triangle.h>

#include <stdbool.h>
#include <stdint.h>

/* check_ledge with an explicit point array: `ppppp` is cl->get_point_array()
 * in the reference.  libivp's grid builder keeps the work ledges and the
 * shared point buffer in separate allocations and sets the point offset only
 * when the grid is compiled, so it passes its point buffer here. */
static inline bool ivp_debug_check_ledge_points(const ivp_compact_ledge_t *cl,
                                                const ivp_compact_poly_point_t *ppppp)
{
    typedef intptr_t intp;
    /* For debugging only.  Checks for consistency */
    IVP_ASSERT(((intp)cl & 15) == 0);
    IVP_ASSERT(((intp)ppppp & 3) == 0); /* grid ledge points are only 4-byte aligned */

    /* all triangles */
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(cl);
    for (int i = 0; i < ivp_ledge_get_n_triangles(cl); i++) {
        IVP_ASSERT(i == ivp_tri_get_tri_index(tri));

        IVP_ASSERT(((intp)tri & 15) == 0);

        /* all edges */
        for (int j = 0; j < 3; j++) {
            const ivp_compact_edge_t *edge = &tri->edges[j];
            const ivp_compact_edge_t *oppo = ivp_edge_get_opposite(edge);
            IVP_ASSERT(edge == ivp_edge_get_opposite(oppo));
            IVP_ASSERT(ivp_edge_get_start_point_index(edge) ==
                       ivp_edge_get_start_point_index(ivp_edge_get_next(oppo)));

            /* convexity: IVP_U_Hesse::calc_hesse(IVP_U_Point p0, p1, p2) and
             * get_dist(IVP_U_Point pp), all in double */
            const float *tp0 = ppppp[ivp_edge_get_start_point_index(edge)].k;
            const float *tp1 = ppppp[ivp_edge_get_start_point_index(ivp_edge_get_next(edge))].k;
            const float *tp2 = ppppp[ivp_edge_get_start_point_index(ivp_edge_get_prev(edge))].k;
            const float *tpp = ppppp[ivp_edge_get_start_point_index(ivp_edge_get_prev(oppo))].k;
            double fp0 = (double)tp0[0];
            double fp1 = (double)tp0[1];
            double fp2 = (double)tp0[2];
            double a0 = (double)tp1[0] - fp0;
            double a1 = (double)tp1[1] - fp1;
            double a2 = (double)tp1[2] - fp2;
            double b0 = (double)tp2[0] - fp0;
            double b1 = (double)tp2[1] - fp1;
            double b2 = (double)tp2[2] - fp2;
            double c0 = b1 * a2 - a1 * b2;
            double c1 = b2 * a0 - a2 * b0;
            double c2 = b0 * a1 - a0 * b1;
            double hesse_val = -(c0 * fp0 + c1 * fp1 + c2 * fp2);
            double test_dist = c0 * (double)tpp[0] + c1 * (double)tpp[1] + c2 * (double)tpp[2] + hesse_val;
            IVP_ASSERT(test_dist >= (double)(-1e-6f * 10000)); /* -P_FLOAT_RES * 10000 */
            IVP_USE(test_dist);

            /* check for a round trip */
            int max_c = 0;
            for (const ivp_compact_edge_t *e = ivp_edge_get_opposite(edge);;
                 e = ivp_edge_get_opposite(ivp_edge_get_next(e))) {
                if (max_c++ > 1000) IVP_CORE();
                if (ivp_edge_get_opposite(e) == edge) break; /* loop finished */
            }
        }

        tri = ivp_tri_get_next(tri);
    }
    return true;
}

/* IVP_Compact_Ledge_Solver::check_ledge(cl) */
static inline bool ivp_debug_check_ledge(const ivp_compact_ledge_t *cl)
{
    return ivp_debug_check_ledge_points(cl, ivp_ledge_get_point_array(cl));
}

#endif /* IVP_DEBUG_BUILD */

#endif /* IVP_CHECK_LEDGE_DEBUG_H */
