/* ivp_rot_inertia.c -- Mass center and rotational inertia solver
 *
 * Ported from ivp_rot_inertia_solver.cxx (Ipion Software GmbH).
 *
 * Algorithm:
 *   1. Integrate signed volume contributions from all triangles to find
 *      the center of mass (divergence theorem on triangle mesh).
 *   2. For each axis permutation (x,y,z), (y,z,x), (z,x,y), integrate
 *      the second moment of area to get 2D inertia values.
 *   3. Convert 2D inertia values to 3D diagonal inertia tensor.
 *
 * Falls back to bounding-box-based estimates for degenerate geometry.
 */
#include <ivp/builder/ivp_rot_inertia.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_assert.h>

#include <math.h>

/* ── Mass center finder ────────────────────────────────────────────────── */

typedef struct ivp_find_mass_center {
    float sum_mass[3];   /* weighted sum of vertex positions */
    double qsum_surface; /* sum of squared surface normals */
    double sum_det;      /* sum of signed volume determinants */
} ivp_find_mass_center_t;

static void fmc_init(ivp_find_mass_center_t *fmc) {
    fmc->sum_mass[0] = 0.0f;
    fmc->sum_mass[1] = 0.0f;
    fmc->sum_mass[2] = 0.0f;
    fmc->qsum_surface = 0.0;
    fmc->sum_det = 0.0;
}

/* Cross product: out = (b-a) x (c-a) */
static void cross_abc(const float *a, const float *b, const float *c,
                       float out[3]) {
    float bax = b[0] - a[0], bay = b[1] - a[1], baz = b[2] - a[2];
    float cax = c[0] - a[0], cay = c[1] - a[1], caz = c[2] - a[2];
    out[0] = bay * caz - baz * cay;
    out[1] = baz * cax - bax * caz;
    out[2] = bax * cay - bay * cax;
}

static void fmc_integrate_triangle(ivp_find_mass_center_t *fmc,
                                    const ivp_compact_ledge_t *ledge,
                                    const ivp_compact_triangle_t *tri) {
    const ivp_compact_edge_t *e0 = ivp_tri_get_edge(tri, 0);
    const ivp_compact_edge_t *e1 = ivp_tri_get_edge(tri, 1);
    const ivp_compact_edge_t *e2 = ivp_tri_get_edge(tri, 2);

    const ivp_compact_poly_point_t *p0 = ivp_edge_get_start_point(e0, ledge);
    const ivp_compact_poly_point_t *p1 = ivp_edge_get_start_point(e1, ledge);
    const ivp_compact_poly_point_t *p2 = ivp_edge_get_start_point(e2, ledge);

    /* Surface normal = (p2 - p0) x (p1 - p0) -- matches original winding */
    float normal[3];
    cross_abc(p0->k, p2->k, p1->k, normal);

    double det = (double)normal[0] * (double)p0->k[0]
               + (double)normal[1] * (double)p0->k[1]
               + (double)normal[2] * (double)p0->k[2];

    double ql = (double)normal[0] * (double)normal[0]
              + (double)normal[1] * (double)normal[1]
              + (double)normal[2] * (double)normal[2];

    fmc->qsum_surface += ql;
    fmc->sum_det += det;

    float d25 = (float)(det * 0.25);
    for (int i = 0; i < 3; i++) {
        fmc->sum_mass[i] += (p0->k[i] + p1->k[i] + p2->k[i]) * d25;
    }
}

static void fmc_integrate_ledge(ivp_find_mass_center_t *fmc,
                                 const ivp_compact_ledge_t *ledge) {
    int n = ivp_ledge_get_n_triangles(ledge);
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
    for (int t = 0; t < n; t++, tri++) {
        fmc_integrate_triangle(fmc, ledge, tri);
    }
}

/* ── Mass center solver for one axis ───────────────────────────────────── */

typedef struct ivp_mass_center_solver {
    const float *transform_pos;  /* center offset (3 floats) */
    double msum;
    double mzsum;
    double mzzsum;
} ivp_mass_center_solver_t;

static void mcs_init(ivp_mass_center_solver_t *s, const float *center) {
    s->transform_pos = center;
    s->msum = 0.0;
    s->mzsum = 0.0;
    s->mzzsum = 0.0;
}

static void mcs_integrate_triangle(ivp_mass_center_solver_t *s,
                                    const ivp_compact_ledge_t *ledge,
                                    const ivp_compact_triangle_t *tri,
                                    int x, int y, int z) {
    const ivp_compact_edge_t *e0 = ivp_tri_get_edge(tri, 0);

    /* Hesse normal vector (unnormalized) */
    const ivp_compact_poly_point_t *pp0 = ivp_edge_get_start_point(e0, ledge);
    const ivp_compact_poly_point_t *pp1 = ivp_edge_get_start_point(
        ivp_tri_get_edge(tri, 1), ledge);
    const ivp_compact_poly_point_t *pp2 = ivp_edge_get_start_point(
        ivp_tri_get_edge(tri, 2), ledge);

    float hesse[3];
    cross_abc(pp0->k, pp2->k, pp1->k, hesse);

    /* Normalize */
    float hl = sqrtf(hesse[0]*hesse[0] + hesse[1]*hesse[1] + hesse[2]*hesse[2]);
    if (hl > 1e-20f) {
        float inv = 1.0f / hl;
        hesse[0] *= inv;
        hesse[1] *= inv;
        hesse[2] *= inv;
    }

    double fzdy, fydz;
    int three_edge_area;

    double abs_hz = fabs((double)hesse[z]);
    double abs_hy = fabs((double)hesse[y]);

    if (abs_hz < abs_hy) {
        three_edge_area = 0;
        fydz = 0.5 * (double)hesse[z] / (double)hesse[y];
        fzdy = 0.0;
    } else {
        three_edge_area = 1;
        if (abs_hz > 1e-12) {
            fzdy = -0.5 * (double)hesse[y] / (double)hesse[z];
        } else {
            fzdy = 0.0;
        }
        fydz = 0.0;
    }

    double area_msum = 0.0;
    double area_mzsum = 0.0;
    double area_mzzsum = 0.0;

    for (int l = 0; l < 3; l++) {
        const ivp_compact_edge_t *e = ivp_tri_get_edge(tri, l);
        const ivp_compact_edge_t *en = ivp_tri_get_edge(tri, (l + 1) % 3);

        const ivp_compact_poly_point_t *ep0 = ivp_edge_get_start_point(e, ledge);
        const ivp_compact_poly_point_t *ep1 = ivp_edge_get_start_point(en, ledge);

        /* Transform: subtract center */
        double p0[3], p1[3];
        for (int i = 0; i < 3; i++) {
            p0[i] = (double)ep0->k[i] - (double)s->transform_pos[i];
            p1[i] = (double)ep1->k[i] - (double)s->transform_pos[i];
        }

        double vx = p1[x] - p0[x];
        double vy = p1[y] - p0[y];
        double vz = p1[z] - p0[z];
        double vlen = sqrt(vx*vx + vy*vy + vz*vz);

        if (fabs(vx) < 1e-12 * vlen) {
            continue;
        }

        double lydx = vy / vx;
        double ly0 = p0[y] - lydx * p0[x];

        double ka, kb, kc;
        if (three_edge_area) {
            ka = fzdy * ly0 * ly0;
            kb = fzdy * 2.0 * ly0 * lydx;
            kc = fzdy * lydx * lydx;
        } else {
            double lzdx = vz / vx;
            double lz0 = p0[z] - lzdx * p0[x];
            ka = (ly0 + fydz * lz0) * lz0;
            kb = ly0 * lzdx + lydx * lz0 + 2.0 * fydz * lz0 * lzdx;
            kc = (lydx + fydz * lzdx) * lzdx;
        }

        double x0 = p0[x];
        double x1 = p1[x];
        double dx  = x1 - x0;
        double dx2 = (x1*x1 - x0*x0) * 0.5;
        double dx3 = (x1*x1*x1 - x0*x0*x0) * (1.0 / 3.0);
        double dx4 = (x1*x1*x1*x1 - x0*x0*x0*x0) * 0.25;
        double dx5 = (x1*x1*x1*x1*x1 - x0*x0*x0*x0*x0) * 0.2;

        double f = dx * ka + dx2 * kb + dx3 * kc;
        double k = dx2 * ka + dx3 * kb + dx4 * kc;
        double m = dx3 * ka + dx4 * kb + dx5 * kc;

        area_msum += f;
        area_mzsum += k;
        area_mzzsum += m;
    }

    s->msum += area_msum;
    s->mzsum += area_mzsum;
    s->mzzsum += area_mzzsum;
}

/* ── Bounding box ──────────────────────────────────────────────────────── */

void ivp_ledge_calc_bounding_box(const ivp_compact_ledge_t *ledge,
                                  float min_out[3],
                                  float max_out[3]) {
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
    const ivp_compact_edge_t *first_edge = ivp_tri_get_edge(tri, 0);
    const ivp_compact_poly_point_t *p = ivp_edge_get_start_point(first_edge, ledge);

    for (int i = 0; i < 3; i++) {
        min_out[i] = p->k[i];
        max_out[i] = p->k[i];
    }

    int n = ivp_ledge_get_n_triangles(ledge);
    for (int t = 0; t < n; t++) {
        const ivp_compact_triangle_t *tr = &ivp_ledge_get_first_triangle(ledge)[t];
        for (int e = 0; e < 3; e++) {
            const ivp_compact_edge_t *edge = ivp_tri_get_edge(tr, e);
            const ivp_compact_poly_point_t *pt = ivp_edge_get_start_point(edge, ledge);
            for (int i = 0; i < 3; i++) {
                if (pt->k[i] < min_out[i]) min_out[i] = pt->k[i];
                if (pt->k[i] > max_out[i]) max_out[i] = pt->k[i];
            }
        }
    }
}

void ivp_ledge_calc_geom_center_and_radius(const ivp_compact_ledge_t *ledge,
                                            float center_out[3],
                                            float *radius_out) {
    float mn[3], mx[3];
    ivp_ledge_calc_bounding_box(ledge, mn, mx);

    for (int i = 0; i < 3; i++) {
        center_out[i] = 0.5f * (mn[i] + mx[i]);
    }

    float dx = mx[0] - mn[0];
    float dy = mx[1] - mn[1];
    float dz = mx[2] - mn[2];
    *radius_out = 0.5f * sqrtf(dx*dx + dy*dy + dz*dz);
}

/* ── Collect all ledges ────────────────────────────────────────────────── */

typedef struct ledge_collector {
    const ivp_compact_ledge_t **ledges;
    int count;
    int capacity;
} ledge_collector_t;

static bool collect_ledge(const ivp_compact_ledge_t *ledge, void *user_data) {
    ledge_collector_t *col = (ledge_collector_t *)user_data;
    if (col->count < col->capacity) {
        col->ledges[col->count++] = ledge;
    }
    return true;
}

/* ── Main computation ──────────────────────────────────────────────────── */

static void find_center_given_xyz(const ivp_compact_ledge_t **ledges, int n_ledges,
                                   int x, int y, int z,
                                   const float *center,
                                   double *center_out, double *mass_out,
                                   double *inertia_out) {
    ivp_mass_center_solver_t solver;
    mcs_init(&solver, center);

    for (int l = 0; l < n_ledges; l++) {
        const ivp_compact_ledge_t *ledge = ledges[l];
        int n = ivp_ledge_get_n_triangles(ledge);
        const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
        for (int t = 0; t < n; t++, tri++) {
            mcs_integrate_triangle(&solver, ledge, tri, x, y, z);
        }
    }

    if (solver.msum < 1e-10) {
        *mass_out = 0.0;
        *center_out = 0.0;
        *inertia_out = 1.0;
    } else {
        *center_out = solver.mzsum / solver.msum;
        *mass_out = solver.msum;
        *inertia_out = solver.mzzsum / solver.msum;
    }
}

ivp_result_t ivp_calc_mass_center_and_rot_inertia(
    const ivp_compact_surface_t *surface,
    float mass_center_out[3],
    float rot_inertia_out[3])
{
    if (!surface) return IVP_ERR_INVALID_ARG;

    /* Collect all terminal ledges */
    const ivp_compact_ledgetree_node_t *root = ivp_surface_get_ledgetree_root(surface);
    int n_ledges = ivp_ledgetree_count_terminal_ledges(root);
    if (n_ledges <= 0) return IVP_ERR_INVALID_ARG;

    /* Stack-allocate for small counts, heap for large */
    const ivp_allocator_t *alloc = ivp_default_allocator();
    const size_t ledges_sz = (size_t)n_ledges * sizeof(ivp_compact_ledge_t *);
    const ivp_compact_ledge_t *stack_buf[128];
    const ivp_compact_ledge_t **all_ledges = stack_buf;
    if (n_ledges > 128) {
        all_ledges = (const ivp_compact_ledge_t **)ivp_alloc(alloc, ledges_sz);
        if (!all_ledges) return IVP_ERR_OUT_OF_MEMORY;
    }

    ledge_collector_t col = { all_ledges, 0, n_ledges };
    ivp_ledgetree_visit_all(root, collect_ledge, &col);

    /* Phase 1: Find mass center */
    ivp_find_mass_center_t fmc;
    fmc_init(&fmc);
    for (int l = 0; l < col.count; l++) {
        fmc_integrate_ledge(&fmc, col.ledges[l]);
    }

    double surface_val = sqrt(fmc.qsum_surface);
    ivp_result_t result = IVP_OK;

    if (fmc.sum_det > surface_val * surface_val * surface_val * 1e-9) {
        /* Valid volume: compute proper mass center */
        float center[3];
        double inv_det = 1.0 / fmc.sum_det;
        center[0] = (float)((double)fmc.sum_mass[0] * inv_det);
        center[1] = (float)((double)fmc.sum_mass[1] * inv_det);
        center[2] = (float)((double)fmc.sum_mass[2] * inv_det);

        mass_center_out[0] = center[0];
        mass_center_out[1] = center[1];
        mass_center_out[2] = center[2];

        /* Phase 2: Compute 2D inertia values for each axis permutation */
        double volume;
        double rot_2d[3], check_center;

        find_center_given_xyz(col.ledges, col.count, 0, 1, 2,
                              center, &check_center, &volume, &rot_2d[0]);
        find_center_given_xyz(col.ledges, col.count, 1, 2, 0,
                              center, &check_center, &volume, &rot_2d[1]);
        find_center_given_xyz(col.ledges, col.count, 2, 0, 1,
                              center, &check_center, &volume, &rot_2d[2]);

        /* Convert 2D inertia to 3D diagonal inertia tensor */
        double a2 = rot_2d[0] * rot_2d[0];
        double b2 = rot_2d[1] * rot_2d[1];
        double c2 = rot_2d[2] * rot_2d[2];

        rot_inertia_out[0] = (float)sqrt(b2 + c2);
        rot_inertia_out[1] = (float)sqrt(a2 + c2);
        rot_inertia_out[2] = (float)sqrt(a2 + b2);
    } else {
        /* Degenerate: fallback to bounding-box estimate */
        float total_min[3], total_max[3];
        ivp_ledge_calc_bounding_box(col.ledges[0], total_min, total_max);

        for (int l = 1; l < col.count; l++) {
            float mn[3], mx[3];
            ivp_ledge_calc_bounding_box(col.ledges[l], mn, mx);
            for (int i = 0; i < 3; i++) {
                if (mn[i] < total_min[i]) total_min[i] = mn[i];
                if (mx[i] > total_max[i]) total_max[i] = mx[i];
            }
        }

        for (int i = 0; i < 3; i++) {
            mass_center_out[i] = 0.5f * (total_min[i] + total_max[i]);
        }

        float dx = total_max[0] - total_min[0];
        float dy = total_max[1] - total_min[1];
        float dz = total_max[2] - total_min[2];
        float radius = 0.5f * sqrtf(dx*dx + dy*dy + dz*dz);
        float est = radius * radius * 0.5f;

        rot_inertia_out[0] = est;
        rot_inertia_out[1] = est;
        rot_inertia_out[2] = est;

        result = IVP_ERR_DEGENERATE_GEOMETRY;
    }

    if (all_ledges != stack_buf) {
        ivp_free(alloc, (void *)all_ledges, ledges_sz);
    }

    return result;
}
