/* ivp_surman.c — Surface manager implementations
 *
 * Polygon: ivp_surface_manager/ivp_surman_polygon.cxx
 * Ball:    ivp_physics/ivp_ball.cxx (IVP_SurfaceManager_Ball)
 */

#include <ivp/geometry/ivp_surman.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_compact_ledge_geom.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/collision/ivp_ray_solver.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <math.h>

/* ── Polygon: shared helpers ───────────────────────────────────────────── */

void ivp_surman_polygon_surface_radius_and_deviation(const ivp_compact_surface_t *surface,
                                                     const float center[3],
                                                     float *out_radius,
                                                     float *out_deviation)
{
    /* ivp_surman_polygon.cxx:214 get_radius_and_radius_dev_to_given_center */
    double dx = (double)surface->mass_center[0] - (double)center[0];
    double dy = (double)surface->mass_center[1] - (double)center[1];
    double dz = (double)surface->mass_center[2] - (double)center[2];
    double center_shift = sqrt(dx * dx + dy * dy + dz * dz);
    *out_radius = (float)((double)surface->upper_limit_radius + center_shift);
    float dev_f = (float)ivp_surface_get_max_deviation(surface) *
                  IVP_COMPACT_SURFACE_DEVIATION_STEP_SIZE *
                  surface->upper_limit_radius;
    *out_deviation = (float)((double)dev_f + center_shift);
}

/* IVP_SurfaceManager_Polygon_Solver::traverse_cluster (ivp_surman_polygon.cxx:38) */
static void traverse_cluster(const ivp_compact_ledgetree_node_t *node,
                             const double pos[3], double radius,
                             ivp_surman_ledge_fn add_fn, void *ctx)
{
    const double work = (double)IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;

    /* sphere against sphere */
    double dist_vec[3];
    for (int i = 0; i < 3; i++) dist_vec[i] = (double)node->center[i] - pos[i];
    double check_dist = radius + (double)node->radius;
    double ql = dist_vec[0] * dist_vec[0] + dist_vec[1] * dist_vec[1] + dist_vec[2] * dist_vec[2];
    if (ql > check_dist * check_dist) {
        IVP_IF(0) {
            ivp_debug_printf("Sphere\n");
        }
        return;
    }

    /* box against box */
    float work2 = (float)(work * (double)node->radius);
    for (int i = 0; i < 3; i++) {
        if (fabs(dist_vec[i]) >= (double)((float)node->box_sizes[i] * work2) + radius) {
            IVP_IF(0) {
                ivp_debug_printf("Box\n");
            }
            return;
        }
    }

    /* Note: like the reference, get_compact_hull() is checked first; for a
     * terminal node it returns the terminal ledge itself, so the
     * two-triangle distance test below is only reached for (impossible)
     * terminal nodes without ledge offset. */
    const ivp_compact_ledge_t *hull = ivp_ledgetree_get_compact_hull(node);
    if (hull) {
        add_fn(ctx, hull);
        return;
    }
    if (ivp_ledgetree_is_terminal(node)) {
        const ivp_compact_ledge_t *ledge = ivp_ledgetree_get_compact_ledge(node);
        if (ivp_ledge_get_n_triangles(ledge) == 2) {
            double qdist = ivp_ledge_calc_qlen_pf(ledge, ivp_ledge_get_first_triangle(ledge), pos);
            if (qdist > radius * radius) return;
        }
        add_fn(ctx, ledge);
        return;
    }
    traverse_cluster(ivp_ledgetree_left_son(node), pos, radius, add_fn, ctx);
    traverse_cluster(ivp_ledgetree_right_son(node), pos, radius, add_fn, ctx);
}

void ivp_surman_polygon_surface_ledges_within_radius(const ivp_compact_surface_t *surface,
                                                     const double pos[3],
                                                     double radius,
                                                     const ivp_compact_ledge_t *root_ledge,
                                                     ivp_surman_ledge_fn add_fn,
                                                     void *ctx)
{
    /* ivp_surman_polygon.cxx:141 get_all_ledges_within_radius */
    if (!add_fn) return;
    if (!root_ledge) {
        const ivp_compact_ledgetree_node_t *root_node = ivp_surface_get_ledgetree_root(surface);
        if (!root_node) return;
        traverse_cluster(root_node, pos, radius, add_fn, ctx);
        return;
    }
    if (ivp_ledge_is_terminal(root_ledge)) {
        if (ivp_ledge_get_n_triangles(root_ledge) == 2) {
            double qdist = ivp_ledge_calc_qlen_pf(root_ledge,
                                                  ivp_ledge_get_first_triangle(root_ledge), pos);
            if (qdist > radius * radius) return;
        }
        add_fn(ctx, root_ledge);
        return;
    }
    IVP_ASSERT(!ivp_ledge_is_terminal(root_ledge));   /* get_ledgetree_node (ivp_compact_ledge.hxx:213) */
    const ivp_compact_ledgetree_node_t *root_node =
        (const ivp_compact_ledgetree_node_t *)((const char *)root_ledge + root_ledge->node_or_client);
    if (ivp_ledgetree_is_terminal(root_node)) {
        traverse_cluster(root_node, pos, radius, add_fn, ctx);
        return;
    }
    traverse_cluster(ivp_ledgetree_left_son(root_node), pos, radius, add_fn, ctx);
    traverse_cluster(ivp_ledgetree_right_son(root_node), pos, radius, add_fn, ctx);
}

/* ── Polygon surface manager implementation ─────────────────────────────── */

static const ivp_compact_ledge_t *polygon_get_single_convex(const ivp_surman_t *sm)
{
    /* ivp_surman_polygon.cxx:231 */
    const ivp_surman_polygon_t *poly = (const ivp_surman_polygon_t *)sm;
    const ivp_compact_ledgetree_node_t *root = ivp_surface_get_ledgetree_root(poly->compact_surface);
    if (!root) return NULL;
    if (ivp_ledgetree_is_terminal(root)) return ivp_ledgetree_get_compact_ledge(root);
    return ivp_ledgetree_get_compact_hull(root);
}

static void polygon_get_mass_center(const ivp_surman_t *sm, float out[3])
{
    const ivp_surman_polygon_t *poly = (const ivp_surman_polygon_t *)sm;
    out[0] = poly->compact_surface->mass_center[0];
    out[1] = poly->compact_surface->mass_center[1];
    out[2] = poly->compact_surface->mass_center[2];
}

static void polygon_get_rotation_inertia(const ivp_surman_t *sm, float out[3])
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
    ivp_surman_polygon_surface_radius_and_deviation(poly->compact_surface, center,
                                                    out_radius, out_deviation);
}

static ivp_surman_type_t polygon_get_type(const ivp_surman_t *sm)
{
    (void)sm;
    return IVP_SURMAN_POLYGON;
}

static const ivp_compact_ledgetree_node_t *polygon_get_ledgetree_root(const ivp_surman_t *sm)
{
    const ivp_surman_polygon_t *poly = (const ivp_surman_polygon_t *)sm;
    const ivp_compact_ledgetree_node_t *root =
        ivp_surface_get_ledgetree_root(poly->compact_surface);
    /* Only return root if it is non-terminal (compound shape).
     * Single-convex shapes use the existing single-convex path. */
    if (root && !ivp_ledgetree_is_terminal(root)) return root;
    return NULL;
}

static void polygon_destroy(ivp_surman_t *sm)
{
    (void)sm;
    /* The polygon surman does not own the compact surface. Nothing to free. */
}

static void polygon_ledges_within_radius(const ivp_surman_t *sm, const double pos[3],
                                         double radius, const ivp_compact_ledge_t *root_ledge,
                                         const ivp_compact_ledge_t *other_reference_ledge,
                                         ivp_surman_ledge_fn add_fn, void *ctx)
{
    (void)other_reference_ledge;
    const ivp_surman_polygon_t *poly = (const ivp_surman_polygon_t *)sm;
    ivp_surman_polygon_surface_ledges_within_radius(poly->compact_surface, pos, radius,
                                                    root_ledge, add_fn, ctx);
}

static void terminal_rec(const ivp_compact_ledgetree_node_t *n, ivp_surman_ledge_fn fn, void *ctx)
{
    if (ivp_ledgetree_is_terminal(n)) {
        fn(ctx, ivp_ledgetree_get_compact_ledge(n));
        return;
    }
    terminal_rec(ivp_ledgetree_left_son(n), fn, ctx);
    terminal_rec(ivp_ledgetree_right_son(n), fn, ctx);
}

static void polygon_terminal_ledges(const ivp_surman_t *sm, ivp_surman_ledge_fn fn, void *ctx)
{
    const ivp_surman_polygon_t *poly = (const ivp_surman_polygon_t *)sm;
    if (!fn) return;
    terminal_rec(ivp_surface_get_ledgetree_root(poly->compact_surface), fn, ctx);
}

static void polygon_insert_ledges_hitting_ray(const ivp_surman_t *sm,
                                              struct ivp_ray_solver *ray_solver,
                                              struct ivp_object *object)
{
    /* ivp_surman_polygon.cxx:134 */
    const ivp_surman_polygon_t *poly = (const ivp_surman_polygon_t *)sm;
    ivp_ray_solver_insert_surface_hits(ray_solver, object, poly->compact_surface);
}

static const ivp_surman_vtable_t s_polygon_vtable = {
    .get_single_convex      = polygon_get_single_convex,
    .get_mass_center        = polygon_get_mass_center,
    .get_rotation_inertia   = polygon_get_rotation_inertia,
    .get_radius_and_deviation = polygon_get_radius_and_deviation,
    .get_type               = polygon_get_type,
    .get_ledgetree_root     = polygon_get_ledgetree_root,
    .destroy                = polygon_destroy,
    .get_all_ledges_within_radius = polygon_ledges_within_radius,
    .get_all_terminal_ledges = polygon_terminal_ledges,
    .insert_all_ledges_hitting_ray = polygon_insert_ledges_hitting_ray
};

void ivp_surman_polygon_init(ivp_surman_polygon_t *sm,
                              const ivp_compact_surface_t *surface)
{
    sm->base.vtable = &s_polygon_vtable;
    sm->compact_surface = surface;
}

const ivp_compact_surface_t *ivp_surman_get_polygon_surface(const ivp_surman_t *sm)
{
    if (!sm || sm->vtable != &s_polygon_vtable) return NULL;
    return ((const ivp_surman_polygon_t *)sm)->compact_surface;
}

/* ── Ball surface manager implementation ────────────────────────────────── */

/* IVP_SurfaceManager_Ball::compact_ledge: zeroed ledge + one triangle */
static _Alignas(16) const struct {
    ivp_compact_ledge_t ledge;
    ivp_compact_triangle_t tri;
} s_ball_dummy_ledge;

const ivp_compact_ledge_t *ivp_surman_ball_dummy_ledge(void)
{
    return &s_ball_dummy_ledge.ledge;
}

static const ivp_compact_ledge_t *ball_get_single_convex(const ivp_surman_t *sm)
{
    (void)sm;
    return &s_ball_dummy_ledge.ledge;
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
    /* reference: set_to_zero(); the object computes (2/5) r^2 itself */
    (void)sm;
    out[0] = 0.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
}

static void ball_get_radius_and_deviation(const ivp_surman_t *sm,
                                          const float center[3],
                                          float *out_radius,
                                          float *out_deviation)
{
    /* reference: the shared ball surman has no per object radius */
    (void)sm;
    (void)center;
    if (out_radius) *out_radius = 0.0f;
    if (out_deviation) *out_deviation = 0.0f;
}

static ivp_surman_type_t ball_get_type(const ivp_surman_t *sm)
{
    (void)sm;
    return IVP_SURMAN_BALL;
}

static void ball_destroy(ivp_surman_t *sm)
{
    (void)sm;
}

static void ball_ledges_within_radius(const ivp_surman_t *sm, const double pos[3],
                                      double radius, const ivp_compact_ledge_t *root_ledge,
                                      const ivp_compact_ledge_t *other_reference_ledge,
                                      ivp_surman_ledge_fn add_fn, void *ctx)
{
    (void)sm;
    (void)root_ledge;
    (void)other_reference_ledge;
    if (!add_fn) return;
    double ql = pos[0] * pos[0] + pos[1] * pos[1] + pos[2] * pos[2];
    if (ql > radius * radius) return;
    add_fn(ctx, &s_ball_dummy_ledge.ledge);
}

static void ball_terminal_ledges(const ivp_surman_t *sm, ivp_surman_ledge_fn fn, void *ctx)
{
    (void)sm;
    if (fn) fn(ctx, &s_ball_dummy_ledge.ledge);
}

static const ivp_surman_vtable_t s_ball_vtable = {
    .get_single_convex      = ball_get_single_convex,
    .get_mass_center        = ball_get_mass_center,
    .get_rotation_inertia   = ball_get_rotation_inertia,
    .get_radius_and_deviation = ball_get_radius_and_deviation,
    .get_type               = ball_get_type,
    .get_ledgetree_root     = NULL,
    .destroy                = ball_destroy,
    .get_all_ledges_within_radius = ball_ledges_within_radius,
    .get_all_terminal_ledges = ball_terminal_ledges,
    .insert_all_ledges_hitting_ray = NULL /* balls: IVP_Ray_Solver::check_ray_against_ball */
};

void ivp_surman_ball_init(ivp_surman_ball_t *sm, float radius)
{
    sm->base.vtable = &s_ball_vtable;
    sm->radius = radius;
}

float ivp_surman_get_ball_radius(const ivp_surman_t *sm)
{
    if (!sm || sm->vtable != &s_ball_vtable) return 0.0f;
    return ((const ivp_surman_ball_t *)sm)->radius;
}
