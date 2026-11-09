/* ivp_rot_inertia.c -- Mass center and rotational inertia solver
 *
 * Faithful port of ivp_compact_builder/ivp_rot_inertia_solver.cxx
 * (Ipion Software GmbH), including its mixed float/double arithmetic:
 *   - IVP_Compact_Ledge_Find_Mass_Center keeps sum_mass and the triangle
 *     normal in IVP_U_Float_Point (float) and det/qsum in double;
 *   - IVP_Compact_Ledge_Mass_Center_Solver uses a double hesse vector, but
 *     the vertices relative to the mass center are rounded to float
 *     (transform->vimult4 into an IVP_U_Float_Point) and the edge vector is
 *     their float difference (IVP_U_Point::subtract(float, float));
 *   - ledges are integrated in reverse get_all_ledges() order.
 *
 * Triangle winding: the reference area normal area(p0, p_prev, p_next) =
 * (p1-p0) x (p2-p0) points outwards for CCW triangles, so sum_det > 0 for
 * closed outward oriented meshes.
 */
#include <ivp/builder/ivp_rot_inertia.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_compact_ledge_geom.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_assert.h>

#include <math.h>

#define RI_DOUBLE_EPS 1e-10  /* P_DOUBLE_EPS */
#define RI_DOUBLE_RES 1e-12  /* P_DOUBLE_RES */

/* ── IVP_Compact_Ledge_Find_Mass_Center ────────────────────────────────── */

typedef struct ri_find_mass_center {
    float  sum_mass[3];
    double qsum_surface;
    double sum_det;
} ri_find_mass_center_t;

/* IVP_U_Float_Point::add_multiple(v, f): this = float(v*f + this) */
static inline void fp_add_multiple(float acc[3], const float v[3], double f)
{
    for (int i = 0; i < 3; i++) {
        double a = (double)v[i] * f;
        a += (double)acc[i];
        acc[i] = (float)a;
    }
}

static void fmc_integrate_triangle(ri_find_mass_center_t *fmc,
                                   const ivp_compact_ledge_t *ledge,
                                   const ivp_compact_triangle_t *tri)
{
    /* ivp_rot_inertia_solver.cxx:134 */
    const ivp_compact_edge_t *edge = &tri->edges[0];
    const float *p0 = ivp_edge_get_start_point(edge, ledge)->k;
    const float *p1 = ivp_edge_get_start_point(ivp_edge_get_next(edge), ledge)->k;
    const float *p2 = ivp_edge_get_start_point(ivp_edge_get_prev(edge), ledge)->k;

    /* IVP_U_Float_Point::inline_set_vert_to_area_defined_by_three_points(p0, p2, p1) */
    double a0 = (double)p2[0] - (double)p0[0];
    double a1 = (double)p2[1] - (double)p0[1];
    double a2 = (double)p2[2] - (double)p0[2];
    double b0 = (double)p1[0] - (double)p0[0];
    double b1 = (double)p1[1] - (double)p0[1];
    double b2 = (double)p1[2] - (double)p0[2];
    float n[3];
    n[0] = (float)(b1 * a2 - a1 * b2);
    n[1] = (float)(b2 * a0 - a2 * b0);
    n[2] = (float)(b0 * a1 - a0 * b1);

    /* IVP_U_Float_Point::dot_product: double products summed z, y, x;
     * quad_length: float arithmetic */
    double det = (double)n[2] * (double)p0[2] + (double)n[1] * (double)p0[1] + (double)n[0] * (double)p0[0];
    float ql_f = n[0] * n[0] + n[1] * n[1] + n[2] * n[2];

    fmc->qsum_surface += (double)ql_f;
    fmc->sum_det += det;
    double f = det * (double)0.25f;
    fp_add_multiple(fmc->sum_mass, p0, f);
    fp_add_multiple(fmc->sum_mass, p1, f);
    fp_add_multiple(fmc->sum_mass, p2, f);
}

/* ── IVP_Compact_Ledge_Mass_Center_Solver ──────────────────────────────── */

typedef struct ri_mass_center_solver {
    const double *center;   /* transform.vv (rotation is identity) */
    double msum;
    double mzsum;
    double mzzsum;
} ri_mass_center_solver_t;

static void mcs_integrate_triangle(ri_mass_center_solver_t *s,
                                   const ivp_compact_ledge_t *ledge,
                                   const ivp_compact_triangle_t *tri,
                                   int x, int y, int z)
{
    /* ivp_rot_inertia_solver.cxx:44 */
    double hesse[3];
    ivp_ledge_calc_hesse_vec(ledge, &tri->edges[0], hesse);
    {   /* IVP_U_Point::normize(): mult by isqrt(len, 3) unless len < eps */
        double len = hesse[0] * hesse[0] + hesse[1] * hesse[1] + hesse[2] * hesse[2];
        if (len >= RI_DOUBLE_EPS) {
            double f = ivp_isqrt_double(len, 3);
            hesse[0] *= f;
            hesse[1] *= f;
            hesse[2] *= f;
        }
    }

    double fzdy, fydz;
    int three_edge_area;
    if (fabs(hesse[z]) < fabs(hesse[y])) {
        three_edge_area = 0;
        fydz = (double)0.5f * hesse[z] / hesse[y];
        fzdy = 0.0;
    } else {
        three_edge_area = 1;
        if (fabs(hesse[z]) > RI_DOUBLE_RES) {
            fzdy = (double)-0.5f * hesse[y] / hesse[z];
        } else {
            fzdy = 0.0;
        }
        fydz = 0.0;
    }

    double area_msum = 0.0;
    double area_mzsum = 0.0;
    double area_mzzsum = 0.0;
    for (int l = 0; l < 3; l++) {
        const ivp_compact_edge_t *e = &tri->edges[l];
        const ivp_compact_edge_t *en = ivp_edge_get_next(e);
        const float *p0o = ivp_edge_get_start_point(e, ledge)->k;
        const float *p1o = ivp_edge_get_start_point(en, ledge)->k;

        /* transform->vimult4(float in, float out): (p - vv) rounded to float */
        float p0f[3], p1f[3];
        for (int i = 0; i < 3; i++) {
            p0f[i] = (float)((double)p0o[i] - s->center[i]);
            p1f[i] = (float)((double)p1o[i] - s->center[i]);
        }
        double p0[3] = { (double)p0f[0], (double)p0f[1], (double)p0f[2] };
        double p1[3] = { (double)p1f[0], (double)p1f[1], (double)p1f[2] };

        /* IVP_U_Point::subtract(const IVP_U_Float_Point *, const IVP_U_Float_Point *):
         * the difference is taken in float, then widened */
        float vf[3];
        for (int i = 0; i < 3; i++) vf[i] = p1f[i] - p0f[i];
        double vec[3] = { (double)vf[0], (double)vf[1], (double)vf[2] };
        double vlen = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
        if (fabs(vec[x]) < RI_DOUBLE_RES * vlen) continue;

        double lydx = vec[y] / vec[x];
        double ly0 = p0[y] - lydx * p0[x];

        double ka, kb, kc;
        if (three_edge_area) {
            ka = fzdy * ly0 * ly0;
            kb = fzdy * (double)2.0f * ly0 * lydx;
            kc = fzdy * lydx * lydx;
        } else {
            double lzdx = vec[z] / vec[x];
            double lz0 = p0[z] - lzdx * p0[x];
            ka = (ly0 + fydz * lz0) * lz0;
            kb = ly0 * lzdx + lydx * lz0 + (double)2.0f * fydz * lz0 * lzdx;
            kc = (lydx + fydz * lzdx) * lzdx;
        }
        double x0 = p0[x];
        double x1 = p1[x];
        double dx = x1 - x0;
        double dx2 = (x1 * x1 - x0 * x0) * (double)0.5f;
        double dx3 = (x1 * x1 * x1 - x0 * x0 * x0) * (double)(1.0f / 3.0f);
        double dx4 = (x1 * x1 * x1 * x1 - x0 * x0 * x0 * x0) * (double)0.25f;
        double dx5 = (x1 * x1 * x1 * x1 * x1 - x0 * x0 * x0 * x0 * x0) * (double)0.2f;
        double fa = dx * ka + dx2 * kb + dx3 * kc;
        double k = dx2 * ka + dx3 * kb + dx4 * kc;
        double m = dx3 * ka + dx4 * kb + dx5 * kc;
        area_msum += fa;
        area_mzsum += k;
        area_mzzsum += m;
    }
    s->msum += area_msum;
    s->mzsum += area_mzsum;
    s->mzzsum += area_mzzsum;
}

static void find_center_given_xyz(const ivp_compact_ledge_t *const *ledges, int n_ledges,
                                  int x, int y, int z, const double center[3],
                                  double *center_out, double *mass_out,
                                  double *inertia_out)
{
    /* ivp_rot_inertia_solver.cxx:184 */
    ri_mass_center_solver_t solver;
    solver.center = center;
    solver.msum = 0.0;
    solver.mzsum = 0.0;
    solver.mzzsum = 0.0;
    for (int l = n_ledges - 1; l >= 0; l--) {
        const ivp_compact_ledge_t *ledge = ledges[l];
        const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
        int n = ivp_ledge_get_n_triangles(ledge);
        for (int t = 0; t < n; t++, tri++) {
            mcs_integrate_triangle(&solver, ledge, tri, x, y, z);
        }
    }
    if (solver.msum < RI_DOUBLE_EPS) {
        *mass_out = 0.0;
        *center_out = 0.0;
        *inertia_out = 1.0;
    } else {
        *center_out = solver.mzsum / solver.msum;
        *mass_out = solver.msum;
        *inertia_out = solver.mzzsum / solver.msum;
    }
}

/* ── Public API ────────────────────────────────────────────────────────── */

ivp_result_t ivp_calc_mass_center_and_rot_inertia_ledges(
    const ivp_compact_ledge_t *const *all_ledges,
    int n_ledges,
    double mass_center_out[3],
    double rot_inertia_out[3])
{
    /* ivp_rot_inertia_solver.cxx:211 localCalcMassCenterAndRotInertia */
    if (!all_ledges || n_ledges <= 0) return IVP_ERR_INVALID_ARG;

    ri_find_mass_center_t fmc;
    fmc.sum_mass[0] = fmc.sum_mass[1] = fmc.sum_mass[2] = 0.0f;
    fmc.qsum_surface = 0.0;
    fmc.sum_det = 0.0;
    for (int l = n_ledges - 1; l >= 0; l--) {
        const ivp_compact_ledge_t *ledge = all_ledges[l];
        const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
        int n = ivp_ledge_get_n_triangles(ledge);
        for (int t = 0; t < n; t++, tri++) {
            fmc_integrate_triangle(&fmc, ledge, tri);
        }
    }

    double surface = sqrt(fmc.qsum_surface);
    if (fmc.sum_det > surface * surface * surface * (double)1e-9f) {
        /* get_mass_center(): set_multiple(&sum_mass, 1.0f / sum_det) */
        double inv = 1.0 / fmc.sum_det;
        double center[3];
        for (int i = 0; i < 3; i++) center[i] = (double)fmc.sum_mass[i] * inv;
        mass_center_out[0] = center[0];
        mass_center_out[1] = center[1];
        mass_center_out[2] = center[2];

        double volume, check_center;
        double ri2d[3];
        find_center_given_xyz(all_ledges, n_ledges, 0, 1, 2, center, &check_center, &volume, &ri2d[0]);
        find_center_given_xyz(all_ledges, n_ledges, 1, 2, 0, center, &check_center, &volume, &ri2d[1]);
        find_center_given_xyz(all_ledges, n_ledges, 2, 0, 1, center, &check_center, &volume, &ri2d[2]);

        /* calc 3d rotation inertia based on 2d values */
        double a = ri2d[0] * ri2d[0];
        double b = ri2d[1] * ri2d[1];
        double c = ri2d[2] * ri2d[2];
        rot_inertia_out[0] = sqrt(b + c);
        rot_inertia_out[1] = sqrt(a + c);
        rot_inertia_out[2] = sqrt(a + b);
        return IVP_OK;
    }

    /* no valid masses found, take geom center */
    double mn[3], mx[3];
    ivp_ledge_calc_bounding_box_d(all_ledges[0], mn, mx);
    for (int l = n_ledges - 1; l > 0; l--) {
        double lmn[3], lmx[3];
        ivp_ledge_calc_bounding_box_d(all_ledges[l], lmn, lmx);
        for (int i = 0; i < 3; i++) {
            if (lmn[i] < mn[i]) mn[i] = lmn[i];
            if (lmx[i] > mx[i]) mx[i] = lmx[i];
        }
    }
    for (int i = 0; i < 3; i++) {
        /* set_interpolate(min, max, 0.5) */
        mass_center_out[i] = mn[i] * (1.0 - (double)0.5f) + mx[i] * (double)0.5f;
    }
    double dx = mx[0] - mn[0], dy = mx[1] - mn[1], dz = mx[2] - mn[2];
    double radius = (double)0.5f * sqrt(dx * dx + dy * dy + dz * dz);
    double est = radius * radius * (double)0.5f;
    rot_inertia_out[0] = est;
    rot_inertia_out[1] = est;
    rot_inertia_out[2] = est;
    return IVP_ERR_DEGENERATE_GEOMETRY;
}

ivp_result_t ivp_calc_mass_center_and_rot_inertia_d(
    const ivp_compact_surface_t *surface,
    double mass_center_out[3],
    double rot_inertia_out[3])
{
    if (!surface || !mass_center_out || !rot_inertia_out) return IVP_ERR_INVALID_ARG;

    const ivp_compact_ledgetree_node_t *root = ivp_surface_get_ledgetree_root(surface);
    int n_ledges = ivp_ledgetree_collect_terminal_ledges(root, NULL, 0);
    if (n_ledges <= 0) return IVP_ERR_INVALID_ARG;

    const ivp_allocator_t *alloc = ivp_default_allocator();
    const ivp_compact_ledge_t *stack_buf[128];
    const ivp_compact_ledge_t **ledges = stack_buf;
    size_t sz = (size_t)n_ledges * sizeof(ivp_compact_ledge_t *);
    if (n_ledges > 128) {
        ledges = (const ivp_compact_ledge_t **)ivp_alloc(alloc, sz);
        if (!ledges) return IVP_ERR_OUT_OF_MEMORY;
    }
    ivp_ledgetree_collect_terminal_ledges(root, ledges, n_ledges);

    ivp_result_t r = ivp_calc_mass_center_and_rot_inertia_ledges(
        ledges, n_ledges, mass_center_out, rot_inertia_out);

    if (ledges != stack_buf) ivp_free(alloc, (void *)ledges, sz);
    return r;
}

ivp_result_t ivp_calc_mass_center_and_rot_inertia(
    const ivp_compact_surface_t *surface,
    float mass_center_out[3],
    float rot_inertia_out[3])
{
    if (!mass_center_out || !rot_inertia_out) return IVP_ERR_INVALID_ARG;
    double mc[3], ri[3];
    ivp_result_t r = ivp_calc_mass_center_and_rot_inertia_d(surface, mc, ri);
    if (r == IVP_OK || r == IVP_ERR_DEGENERATE_GEOMETRY) {
        for (int i = 0; i < 3; i++) {
            mass_center_out[i] = (float)mc[i];
            rot_inertia_out[i] = (float)ri[i];
        }
    }
    return r;
}

/* ── Bounding box helpers (float API kept for compatibility) ───────────── */

void ivp_ledge_calc_bounding_box(const ivp_compact_ledge_t *ledge,
                                 float min_out[3],
                                 float max_out[3])
{
    double mn[3], mx[3];
    ivp_ledge_calc_bounding_box_d(ledge, mn, mx);
    for (int i = 0; i < 3; i++) {
        min_out[i] = (float)mn[i];
        max_out[i] = (float)mx[i];
    }
}

void ivp_ledge_calc_geom_center_and_radius(const ivp_compact_ledge_t *ledge,
                                           float center_out[3],
                                           float *radius_out)
{
    /* ivp_compact_ledge_solver.cxx:118 calc_geom_center_and_radius */
    double mn[3], mx[3];
    ivp_ledge_calc_bounding_box_d(ledge, mn, mx);
    for (int i = 0; i < 3; i++) {
        center_out[i] = (float)(mn[i] * (1.0 - (double)0.5f) + mx[i] * (double)0.5f);
    }
    double dx = mx[0] - mn[0], dy = mx[1] - mn[1], dz = mx[2] - mn[2];
    *radius_out = (float)((double)0.5f * sqrt(dx * dx + dy * dy + dz * dz));
}
