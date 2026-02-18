/* ivp_surman.c — Surface manager implementations */

#include <ivp/geometry/ivp_surman.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <math.h>

/* ── Polygon surface manager implementation ─────────────────────────────── */

static const ivp_compact_ledge_t *polygon_get_single_convex(
    const ivp_surman_t *sm)
{
    const ivp_surman_polygon_t *poly = (const ivp_surman_polygon_t *)sm;
    return ivp_surface_get_single_convex(poly->compact_surface);
}

static void polygon_get_mass_center(const ivp_surman_t *sm,
                                     float out[3])
{
    const ivp_surman_polygon_t *poly = (const ivp_surman_polygon_t *)sm;
    out[0] = poly->compact_surface->mass_center[0];
    out[1] = poly->compact_surface->mass_center[1];
    out[2] = poly->compact_surface->mass_center[2];
}

static void polygon_get_rotation_inertia(const ivp_surman_t *sm,
                                          float out[3])
{
    const ivp_surman_polygon_t *poly = (const ivp_surman_polygon_t *)sm;
    out[0] = poly->compact_surface->rotation_inertia[0];
    out[1] = poly->compact_surface->rotation_inertia[1];
    out[2] = poly->compact_surface->rotation_inertia[2];
}

static void polygon_get_radius_and_deviation(const ivp_surman_t *sm,
                                               const float center[3],
                                               float *out_radius,
                                               float *out_deviation)
{
    const ivp_surman_polygon_t *poly = (const ivp_surman_polygon_t *)sm;
    const ivp_compact_surface_t *surface = poly->compact_surface;

    /* Walk all ledge vertices and compute max distance from given center.
     * Also compute max perpendicular deviation from each face plane
     * (cross-product method matching legacy IVP radius/deviation calc). */
    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surface);
    if (!ledge) {
        /* Fallback: use pre-computed values from surface header */
        *out_radius = surface->upper_limit_radius;
        int dev_factor = ivp_surface_get_max_deviation(surface);
        *out_deviation = (float)dev_factor * surface->upper_limit_radius *
                         IVP_COMPACT_SURFACE_DEVIATION_STEP_SIZE;
        return;
    }

    float max_radius = 0.0f;
    float max_deviation = 0.0f;

    int n_tri = ivp_ledge_get_n_triangles(ledge);
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);

    for (int t = 0; t < n_tri; t++, tri++) {
        const ivp_compact_edge_t *e0 = ivp_tri_get_edge(tri, 0);
        const ivp_compact_edge_t *e1 = ivp_tri_get_edge(tri, 1);
        const ivp_compact_edge_t *e2 = ivp_tri_get_edge(tri, 2);

        const ivp_compact_poly_point_t *p0 = ivp_edge_get_start_point(e0, ledge);
        const ivp_compact_poly_point_t *p1 = ivp_edge_get_start_point(e1, ledge);
        const ivp_compact_poly_point_t *p2 = ivp_edge_get_start_point(e2, ledge);

        /* Hesse vector (unnormalized face normal) */
        float ax = p2->k[0] - p0->k[0], ay = p2->k[1] - p0->k[1], az = p2->k[2] - p0->k[2];
        float bx = p1->k[0] - p0->k[0], by = p1->k[1] - p0->k[1], bz = p1->k[2] - p0->k[2];
        float hx = ay * bz - az * by;
        float hy = az * bx - ax * bz;
        float hz = ax * by - ay * bx;
        float h_qlen = hx*hx + hy*hy + hz*hz;
        float inv_h_qlen = (h_qlen > 1e-20f) ? (1.0f / h_qlen) : 0.0f;

        const ivp_compact_poly_point_t *pts[3] = { p0, p1, p2 };
        for (int j = 0; j < 3; j++) {
            float dx = pts[j]->k[0] - center[0];
            float dy = pts[j]->k[1] - center[1];
            float dz = pts[j]->k[2] - center[2];

            float r = sqrtf(dx*dx + dy*dy + dz*dz);
            if (r > max_radius) max_radius = r;

            /* Perpendicular deviation: |d × hesse| / |hesse| */
            float cx = dy * hz - dz * hy;
            float cy = dz * hx - dx * hz;
            float cz = dx * hy - dy * hx;
            float dev = sqrtf((cx*cx + cy*cy + cz*cz) * inv_h_qlen);
            if (dev > max_deviation) max_deviation = dev;
        }
    }

    *out_radius = max_radius;
    *out_deviation = max_deviation;
}

static ivp_surman_type_t polygon_get_type(const ivp_surman_t *sm) {
    (void)sm;
    return IVP_SURMAN_POLYGON;
}

static const ivp_compact_ledgetree_node_t *polygon_get_ledgetree_root(
    const ivp_surman_t *sm)
{
    const ivp_surman_polygon_t *poly = (const ivp_surman_polygon_t *)sm;
    const ivp_compact_ledgetree_node_t *root =
        ivp_surface_get_ledgetree_root(poly->compact_surface);
    /* Only return root if it is non-terminal (compound shape).
     * Single-convex shapes use the existing single-convex path. */
    if (root && !ivp_ledgetree_is_terminal(root)) {
        return root;
    }
    return NULL;
}

static void polygon_destroy(ivp_surman_t *sm) {
    (void)sm;
    /* The polygon surman does not own the compact surface. Nothing to free. */
}

static const ivp_surman_vtable_t s_polygon_vtable = {
    .get_single_convex      = polygon_get_single_convex,
    .get_mass_center        = polygon_get_mass_center,
    .get_rotation_inertia   = polygon_get_rotation_inertia,
    .get_radius_and_deviation = polygon_get_radius_and_deviation,
    .get_type               = polygon_get_type,
    .get_ledgetree_root     = polygon_get_ledgetree_root,
    .destroy                = polygon_destroy
};

void ivp_surman_polygon_init(ivp_surman_polygon_t *sm,
                              const ivp_compact_surface_t *surface)
{
    sm->base.vtable = &s_polygon_vtable;
    sm->compact_surface = surface;
}

/* ── Ball surface manager implementation ────────────────────────────────── */

static const ivp_compact_ledge_t *ball_get_single_convex(const ivp_surman_t *sm)
{
    (void)sm;
    return NULL; /* balls have no ledge representation */
}

static void ball_get_mass_center(const ivp_surman_t *sm, float out[3])
{
    (void)sm;
    out[0] = 0.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
}

static void ball_get_rotation_inertia(const ivp_surman_t *sm, float out[3])
{
    const ivp_surman_ball_t *b = (const ivp_surman_ball_t *)sm;
    /* I = (2/5) * m * r^2 for unit mass: (2/5) * r^2 */
    float rr = b->radius * b->radius;
    float inertia = 0.4f * rr;
    out[0] = inertia;
    out[1] = inertia;
    out[2] = inertia;
}

static void ball_get_radius_and_deviation(const ivp_surman_t *sm,
                                           const float center[3],
                                           float *out_radius,
                                           float *out_deviation)
{
    (void)center;
    const ivp_surman_ball_t *b = (const ivp_surman_ball_t *)sm;
    *out_radius = b->radius;
    *out_deviation = 0.0f;
}

static ivp_surman_type_t ball_get_type(const ivp_surman_t *sm)
{
    (void)sm;
    return IVP_SURMAN_BALL;
}

static void ball_destroy(ivp_surman_t *sm)
{
    (void)sm;
    /* stack-allocated; nothing to free */
}

static const ivp_surman_vtable_t s_ball_vtable = {
    .get_single_convex      = ball_get_single_convex,
    .get_mass_center        = ball_get_mass_center,
    .get_rotation_inertia   = ball_get_rotation_inertia,
    .get_radius_and_deviation = ball_get_radius_and_deviation,
    .get_type               = ball_get_type,
    .get_ledgetree_root     = NULL,
    .destroy                = ball_destroy
};

void ivp_surman_ball_init(ivp_surman_ball_t *sm, float radius)
{
    sm->base.vtable = &s_ball_vtable;
    sm->radius = radius;
}
