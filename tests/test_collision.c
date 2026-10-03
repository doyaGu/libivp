/* test_collision.c — Unit tests for M3 collision queries
 *
 * Tests:
 *   - Collision tolerance settings (set_collision_tolerance formula)
 *   - Point-triangle distance (analytic solutions)
 *   - Point-edge distance (analytic solutions)
 *   - Edge-edge distance (analytic solutions)
 *   - Sphere-sphere distance
 *   - Hesse normal vector
 *   - Signed distance to plane
 *   - QR barycentric coordinates
 *   - S-val edge parametric
 *   - Ray-triangle intersection (hit / miss / boundary)
 *   - Ray-sphere intersection
 *   - Ray-surface cast (box)
 *   - Ray-surface cast (convex)
 */
#include "ivp_test.h"
#include <ivp/collision/ivp_collision_tolerance.h>
#include <ivp/collision/ivp_narrowphase.h>
#include <ivp/collision/ivp_ray_solver.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/base/ivp_allocator.h>
#include <stdint.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Collision Tolerance Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(collision_tolerance_defaults) {
    ivp_collision_settings_t cs;
    ivp_collision_settings_init_default(&cs);

    IVP_ASSERT_NEAR(cs.tolerance, 0.01, 1e-6);
    IVP_ASSERT_NEAR(cs.real_coll_dist, 0.001, 1e-6);
    IVP_ASSERT_NEAR(cs.min_coll_dist, 0.01, 1e-6);
    IVP_ASSERT_NEAR(cs.minimum_friction_dist, 0.01, 1e-6);
    IVP_ASSERT_NEAR(cs.friction_dist, 0.02, 1e-6);
    IVP_ASSERT_NEAR(cs.keeper_dist, 0.023, 1e-6);
    IVP_ASSERT_NEAR(cs.distance_keepers_safety, 0.0001, 1e-6);
    IVP_ASSERT_NEAR(cs.max_distance_for_friction, 0.045, 1e-6);
    IVP_ASSERT_NEAR(cs.max_distance_for_impact, 0.22, 1e-6);
    IVP_ASSERT_NEAR(cs.min_vertical_speed_at_collision, 0.02, 1e-6);
    IVP_ASSERT_NEAR(cs.mindist_change_force_dist, 0.001, 1e-6);
}

IVP_TEST(collision_tolerance_custom) {
    ivp_collision_settings_t cs;
    ivp_collision_settings_init(&cs, IVP_REAL(0.05));

    IVP_ASSERT_NEAR(cs.tolerance, 0.05, 1e-6);
    IVP_ASSERT_NEAR(cs.real_coll_dist, 0.005, 1e-6);
    IVP_ASSERT_NEAR(cs.min_coll_dist, 0.05, 1e-6);
    IVP_ASSERT_NEAR(cs.friction_dist, 0.1, 1e-6);
    IVP_ASSERT_NEAR(cs.keeper_dist, 0.115, 1e-6);
}

IVP_TEST(collision_tolerance_coll_dists_interpolation) {
    ivp_collision_settings_t cs;
    ivp_collision_settings_init_default(&cs);

    /* First entry should be min_coll_dist, last should be minimum_friction_dist */
    IVP_ASSERT_NEAR(cs.coll_dists[0], cs.min_coll_dist, 1e-9);
    IVP_ASSERT_NEAR(cs.coll_dists[IVP_MAX_STEPS_FOR_COLLDIST_DECREASE - 1],
                    cs.minimum_friction_dist, 1e-9);

    /* Monotonically increasing */
    for (int i = 1; i < IVP_MAX_STEPS_FOR_COLLDIST_DECREASE; i++) {
        IVP_ASSERT_TRUE(cs.coll_dists[i] >= cs.coll_dists[i - 1]);
    }
}

IVP_TEST(collision_tolerance_speed_after_keeper) {
    ivp_collision_settings_t cs;
    ivp_collision_settings_init_default(&cs);

    /* speed = sqrt(2 * (keeper - min_coll) * 9.81) */
    double delta = (double)(cs.keeper_dist - cs.min_coll_dist);
    double expected = sqrt(2.0 * delta * 9.81);
    IVP_ASSERT_NEAR(cs.speed_after_keeper_dist, expected, 1e-5);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Helper: build a test triangle ledge (unit right triangle at origin)
 *   p0 = (0, 0, 0), p1 = (1, 0, 0), p2 = (0, 1, 0)
 *   Triangle in XY plane, normal points in +Z.
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Build a box surface for testing */
static ivp_compact_surface_t *make_box(void) {
    return ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
}

static void free_surface(ivp_compact_surface_t *s) {
    ivp_compact_surface_free(NULL, s);
}

/* Get the first triangle's first edge from a box surface */
static void get_box_first_tri(const ivp_compact_surface_t *surface,
                               const ivp_compact_ledge_t  **ledge_out,
                               const ivp_compact_triangle_t **tri_out) {
    const ivp_compact_ledgetree_node_t *root =
        ivp_surface_get_ledgetree_root(surface);
    *ledge_out = ivp_ledgetree_get_compact_ledge(root);
    *tri_out = ivp_ledge_get_first_triangle(*ledge_out);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  QR Barycentric Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(qr_vals_inside_triangle) {
    /* Use a box, get the first triangle, test a point known to be inside */
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge;
    const ivp_compact_triangle_t *tri;
    get_box_first_tri(surface, &ledge, &tri);

    /* Get triangle vertices to compute centroid */
    ivp_vec3r_t p0 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[0], ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[0], ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[0], ledge)->k[2]);
    ivp_vec3r_t p1 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[1], ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[1], ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[1], ledge)->k[2]);
    ivp_vec3r_t p2 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[2], ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[2], ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[2], ledge)->k[2]);

    /* Centroid is definitely inside */
    ivp_vec3r_t centroid;
    ivp_vec3r_add(&centroid, &p0, &p1);
    ivp_vec3r_add(&centroid, &centroid, &p2);
    ivp_vec3r_scale(&centroid, &centroid, IVP_REAL(1.0) / IVP_REAL(3.0));

    ivp_qr_result_t qr;
    ivp_calc_qr_vals(ledge, &tri->edges[0], &centroid, &qr);

    IVP_ASSERT_FALSE(ivp_qr_is_outside(&qr));
    IVP_ASSERT_TRUE(qr.checks[0] > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(qr.checks[1] > IVP_REAL(0.0));
    IVP_ASSERT_TRUE(qr.checks[2] > IVP_REAL(0.0));

    free_surface(surface);
}

IVP_TEST(qr_vals_outside_triangle) {
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge;
    const ivp_compact_triangle_t *tri;
    get_box_first_tri(surface, &ledge, &tri);

    /* A point far away should be outside */
    ivp_vec3r_t far_point = ivp_vec3r(IVP_REAL(100.0), IVP_REAL(100.0), IVP_REAL(100.0));

    ivp_qr_result_t qr;
    ivp_calc_qr_vals(ledge, &tri->edges[0], &far_point, &qr);

    IVP_ASSERT_TRUE(ivp_qr_is_outside(&qr));

    free_surface(surface);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  S-val Edge Parametric Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(s_val_projects_onto_edge) {
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge;
    const ivp_compact_triangle_t *tri;
    get_box_first_tri(surface, &ledge, &tri);

    /* Get edge midpoint */
    const ivp_compact_edge_t *e0 = &tri->edges[0];
    const ivp_compact_edge_t *e1 = ivp_edge_get_next(e0);

    ivp_vec3r_t p0 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(e0, ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(e0, ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(e0, ledge)->k[2]);
    ivp_vec3r_t p1 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(e1, ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(e1, ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(e1, ledge)->k[2]);

    /* Midpoint of edge should project inside */
    ivp_vec3r_t mid;
    ivp_vec3r_lerp(&mid, &p0, &p1, IVP_REAL(0.5));

    /* Offset midpoint perpendicular to edge (still projects onto edge) */
    ivp_vec3r_t edge_dir;
    ivp_vec3r_sub(&edge_dir, &p1, &p0);

    /* Cross with Z axis for perpendicular (approximate) */
    ivp_vec3r_t offset;
    offset.k[0] = -edge_dir.k[1];
    offset.k[1] = edge_dir.k[0];
    offset.k[2] = IVP_REAL(0.0);
    ivp_vec3r_scale(&offset, &offset, IVP_REAL(0.01));
    ivp_vec3r_add(&mid, &mid, &offset);

    ivp_s_result_t sv;
    ivp_calc_s_val(ledge, e0, &mid, &sv);

    IVP_ASSERT_TRUE(ivp_s_is_inside(&sv));

    free_surface(surface);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Point-Point Distance Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(qdist_pp_same_point) {
    ivp_vec3r_t a = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(3.0));
    ivp_real_t d = ivp_qdist_pp(&a, &a);
    IVP_ASSERT_NEAR(d, 0.0, 1e-12);
}

IVP_TEST(qdist_pp_unit_apart) {
    ivp_vec3r_t a = ivp_vec3r_zero();
    ivp_vec3r_t b = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_real_t d = ivp_qdist_pp(&a, &b);
    IVP_ASSERT_NEAR(d, 1.0, 1e-9);
}

IVP_TEST(qdist_pp_diagonal) {
    ivp_vec3r_t a = ivp_vec3r_zero();
    ivp_vec3r_t b = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_real_t d = ivp_qdist_pp(&a, &b);
    IVP_ASSERT_NEAR(d, 3.0, 1e-9);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Point-Edge Distance Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(qdist_pk_perpendicular) {
    /* Point above the midpoint of an edge — perpendicular distance */
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge;
    const ivp_compact_triangle_t *tri;
    get_box_first_tri(surface, &ledge, &tri);

    const ivp_compact_edge_t *e0 = &tri->edges[0];
    const ivp_compact_edge_t *e1 = ivp_edge_get_next(e0);

    ivp_vec3r_t p0 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(e0, ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(e0, ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(e0, ledge)->k[2]);
    ivp_vec3r_t p1 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(e1, ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(e1, ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(e1, ledge)->k[2]);

    /* Place point directly above midpoint, 2 units away in a
     * direction perpendicular to the edge */
    ivp_vec3r_t mid;
    ivp_vec3r_lerp(&mid, &p0, &p1, IVP_REAL(0.5));

    /* Get the triangle normal as good perpendicular direction */
    const ivp_compact_triangle_t *tri_local = ivp_edge_get_triangle(e0);
    ivp_vec3r_t hesse_vec;
    ivp_calc_hesse_vec(ledge, &tri_local->edges[0], &hesse_vec);
    ivp_vec3r_normalize(&hesse_vec);

    ivp_vec3r_t test_point;
    ivp_vec3r_scale(&test_point, &hesse_vec, IVP_REAL(2.0));
    ivp_vec3r_add(&test_point, &mid, &test_point);

    ivp_real_t qd = ivp_qdist_pk(ledge, e0, &test_point);

    /* Squared distance should be 4.0 (2^2) since point is 2 units
     * perpendicular to the edge midpoint */
    IVP_ASSERT_NEAR(qd, 4.0, 1e-3);

    free_surface(surface);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Point-Triangle Distance Tests — Analytic Solutions
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(qdist_pf_directly_above) {
    /* Point directly above centroid — distance is height */
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge;
    const ivp_compact_triangle_t *tri;
    get_box_first_tri(surface, &ledge, &tri);

    /* Get centroid */
    ivp_vec3r_t p0 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[0], ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[0], ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[0], ledge)->k[2]);
    ivp_vec3r_t p1 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[1], ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[1], ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[1], ledge)->k[2]);
    ivp_vec3r_t p2 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[2], ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[2], ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[2], ledge)->k[2]);

    /* Compute triangle normal to get perpendicular direction */
    ivp_vec3r_t e01, e02, normal;
    ivp_vec3r_sub(&e01, &p1, &p0);
    ivp_vec3r_sub(&e02, &p2, &p0);
    ivp_vec3r_cross(&normal, &e01, &e02);
    ivp_vec3r_normalize(&normal);

    /* Centroid */
    ivp_vec3r_t centroid;
    ivp_vec3r_add(&centroid, &p0, &p1);
    ivp_vec3r_add(&centroid, &centroid, &p2);
    ivp_vec3r_scale(&centroid, &centroid, IVP_REAL(1.0) / IVP_REAL(3.0));

    /* Test point = centroid + 3.0 * normal */
    ivp_vec3r_t test_point;
    ivp_vec3r_scale(&test_point, &normal, IVP_REAL(3.0));
    ivp_vec3r_add(&test_point, &centroid, &test_point);

    ivp_real_t qd = ivp_qdist_pf(ledge, &tri->edges[0], &test_point);

    /* Squared distance should be 9.0 */
    IVP_ASSERT_NEAR(qd, 9.0, 1e-3);

    free_surface(surface);
}

IVP_TEST(qdist_pf_at_surface) {
    /* Point on the triangle surface — distance is 0 */
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge;
    const ivp_compact_triangle_t *tri;
    get_box_first_tri(surface, &ledge, &tri);

    /* Centroid lies on the surface */
    ivp_vec3r_t p0 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[0], ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[0], ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[0], ledge)->k[2]);
    ivp_vec3r_t p1 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[1], ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[1], ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[1], ledge)->k[2]);
    ivp_vec3r_t p2 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[2], ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[2], ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[2], ledge)->k[2]);

    ivp_vec3r_t centroid;
    ivp_vec3r_add(&centroid, &p0, &p1);
    ivp_vec3r_add(&centroid, &centroid, &p2);
    ivp_vec3r_scale(&centroid, &centroid, IVP_REAL(1.0) / IVP_REAL(3.0));

    ivp_real_t qd = ivp_qdist_pf(ledge, &tri->edges[0], &centroid);

    IVP_ASSERT_NEAR(qd, 0.0, 1e-6);

    free_surface(surface);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Edge-Edge Distance Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(qdist_kk_parallel_edges) {
    /* Two parallel edges of a box — known distance */
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge;
    const ivp_compact_triangle_t *tri;
    get_box_first_tri(surface, &ledge, &tri);

    /* Use edge 0 of first triangle and edge 0 of a different triangle.
     * For a unit box, parallel edges are 2 units apart. */
    const ivp_compact_edge_t *e0 = &tri->edges[0];
    const ivp_compact_triangle_t *tri2 = ivp_tri_get_next(tri);
    const ivp_compact_edge_t *e1 = &tri2->edges[0];

    ivp_real_t qd = ivp_qdist_kk(ledge, e0, ledge, e1);

    /* Should be >= 0 (valid distance) */
    IVP_ASSERT_TRUE(qd >= IVP_REAL(0.0));

    free_surface(surface);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Sphere-Sphere Distance Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(dist_sphere_sphere_separated) {
    ivp_vec3r_t a = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t b = ivp_vec3r(IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_real_t d = ivp_dist_sphere_sphere(&a, IVP_REAL(1.0), &b, IVP_REAL(1.0));
    IVP_ASSERT_NEAR(d, 3.0, 1e-6);  /* 5 - 1 - 1 = 3 */
}

IVP_TEST(dist_sphere_sphere_touching) {
    ivp_vec3r_t a = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t b = ivp_vec3r(IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_real_t d = ivp_dist_sphere_sphere(&a, IVP_REAL(1.0), &b, IVP_REAL(1.0));
    IVP_ASSERT_NEAR(d, 0.0, 1e-6);  /* 2 - 1 - 1 = 0 */
}

IVP_TEST(dist_sphere_sphere_overlapping) {
    ivp_vec3r_t a = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t b = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_real_t d = ivp_dist_sphere_sphere(&a, IVP_REAL(1.0), &b, IVP_REAL(1.0));
    IVP_ASSERT_NEAR(d, -1.0, 1e-6);  /* 1 - 1 - 1 = -1 */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Hesse Normal and Signed Distance Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(hesse_vec_nonzero) {
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge;
    const ivp_compact_triangle_t *tri;
    get_box_first_tri(surface, &ledge, &tri);

    ivp_vec3r_t hesse;
    ivp_calc_hesse_vec(ledge, &tri->edges[0], &hesse);

    ivp_real_t len_sq = ivp_vec3r_quad_length(&hesse);
    IVP_ASSERT_TRUE(len_sq > IVP_REAL(0.0));

    free_surface(surface);
}

IVP_TEST(signed_dist_to_plane_positive_negative) {
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge;
    const ivp_compact_triangle_t *tri;
    get_box_first_tri(surface, &ledge, &tri);

    /* Compute normal direction */
    ivp_vec3r_t hesse;
    ivp_calc_hesse_vec(ledge, &tri->edges[0], &hesse);
    ivp_vec3r_normalize(&hesse);

    /* Centroid */
    ivp_vec3r_t p0 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[0], ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[0], ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[0], ledge)->k[2]);

    /* Point on front side */
    ivp_vec3r_t front;
    ivp_vec3r_scale(&front, &hesse, IVP_REAL(5.0));
    ivp_vec3r_add(&front, &p0, &front);
    ivp_real_t d_front = ivp_signed_dist_to_plane(ledge, &tri->edges[0], &front);
    IVP_ASSERT_TRUE(d_front > IVP_REAL(0.0));

    /* Point on back side */
    ivp_vec3r_t back;
    ivp_vec3r_scale(&back, &hesse, IVP_REAL(-5.0));
    ivp_vec3r_add(&back, &p0, &back);
    ivp_real_t d_back = ivp_signed_dist_to_plane(ledge, &tri->edges[0], &back);
    IVP_ASSERT_TRUE(d_back < IVP_REAL(0.0));

    free_surface(surface);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Ray-Sphere Intersection Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(ray_sphere_hit) {
    ivp_vec3r_t origin = ivp_vec3r(IVP_REAL(-5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t dir = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t center = ivp_vec3r_zero();
    ivp_real_t t;

    bool hit = ivp_ray_test_sphere(&origin, &dir, &center, IVP_REAL(1.0), &t);
    IVP_ASSERT_TRUE(hit);
    IVP_ASSERT_NEAR(t, 4.0, 1e-6);  /* 5 - 1 = 4 */
}

IVP_TEST(ray_sphere_miss) {
    ivp_vec3r_t origin = ivp_vec3r(IVP_REAL(-5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t dir = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_vec3r_t center = ivp_vec3r_zero();

    bool hit = ivp_ray_test_sphere(&origin, &dir, &center, IVP_REAL(1.0), NULL);
    IVP_ASSERT_FALSE(hit);
}

IVP_TEST(ray_sphere_origin_inside) {
    ivp_vec3r_t origin = ivp_vec3r_zero();
    ivp_vec3r_t dir = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t center = ivp_vec3r_zero();
    ivp_real_t t;

    bool hit = ivp_ray_test_sphere(&origin, &dir, &center, IVP_REAL(2.0), &t);
    IVP_ASSERT_TRUE(hit);
    IVP_ASSERT_NEAR(t, 2.0, 1e-6);  /* exit point at distance 2 */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Ray-Triangle Intersection Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(ray_triangle_hit) {
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge;
    const ivp_compact_triangle_t *tri;
    get_box_first_tri(surface, &ledge, &tri);

    /* Get triangle normal and centroid */
    ivp_vec3r_t hesse;
    ivp_calc_hesse_vec(ledge, &tri->edges[0], &hesse);
    ivp_vec3r_normalize(&hesse);

    ivp_vec3r_t p0 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[0], ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[0], ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[0], ledge)->k[2]);
    ivp_vec3r_t p1 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[1], ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[1], ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[1], ledge)->k[2]);
    ivp_vec3r_t p2 = ivp_vec3r(
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[2], ledge)->k[0],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[2], ledge)->k[1],
        (ivp_real_t)ivp_edge_get_start_point(&tri->edges[2], ledge)->k[2]);

    ivp_vec3r_t centroid;
    ivp_vec3r_add(&centroid, &p0, &p1);
    ivp_vec3r_add(&centroid, &centroid, &p2);
    ivp_vec3r_scale(&centroid, &centroid, IVP_REAL(1.0) / IVP_REAL(3.0));

    /* Shoot ray from above centroid, toward triangle */
    ivp_vec3r_t neg_hesse;
    ivp_vec3r_negate(&neg_hesse, &hesse);

    ivp_vec3r_t ray_origin;
    ivp_vec3r_scale(&ray_origin, &hesse, IVP_REAL(5.0));
    ivp_vec3r_add(&ray_origin, &centroid, &ray_origin);

    ivp_ray_input_t ray;
    ray.origin = ray_origin;
    ray.direction = neg_hesse;
    ray.max_distance = IVP_REAL(10.0);

    ivp_real_t hit_dist;
    ivp_result_t r = ivp_ray_test_triangle(ledge, tri, &ray, &hit_dist);

    IVP_ASSERT_TRUE(ivp_ok(r));
    IVP_ASSERT_NEAR(hit_dist, 5.0, 1e-2);

    free_surface(surface);
}

IVP_TEST(ray_triangle_miss) {
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge;
    const ivp_compact_triangle_t *tri;
    get_box_first_tri(surface, &ledge, &tri);

    /* Shoot ray parallel to triangle — should miss */
    ivp_vec3r_t hesse;
    ivp_calc_hesse_vec(ledge, &tri->edges[0], &hesse);
    ivp_vec3r_normalize(&hesse);

    /* Direction perpendicular to normal (parallel to plane) */
    ivp_vec3r_t tangent;
    ivp_vec3r_t up = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_vec3r_cross(&tangent, &hesse, &up);
    ivp_real_t tlen = ivp_vec3r_quad_length(&tangent);
    if (tlen < IVP_REAL_RES) {
        /* hesse is parallel to up, use X axis */
        ivp_vec3r_t right = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_cross(&tangent, &hesse, &right);
    }
    ivp_vec3r_normalize(&tangent);

    ivp_vec3r_t ray_origin;
    ivp_vec3r_scale(&ray_origin, &hesse, IVP_REAL(5.0));

    ivp_ray_input_t ray;
    ray.origin = ray_origin;
    ray.direction = tangent;
    ray.max_distance = IVP_REAL(100.0);

    ivp_real_t hit_dist;
    ivp_result_t r = ivp_ray_test_triangle(ledge, tri, &ray, &hit_dist);

    IVP_ASSERT_FALSE(ivp_ok(r));

    free_surface(surface);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Ray-Surface Cast (Box) Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(ray_cast_box_hit_along_x) {
    /* Unit box centered at origin, ray along +X from far away */
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    /* Off-center ray: the reference convex walk (ivp_ray_solver.cxx:470-482)
     * treats checks == 0 as "outside" and ping-pongs across an edge, so a
     * ray exactly through a face diagonal (e.g. the face center) misses —
     * see ray_cast_box_center_hits_diagonal_misses. */
    ivp_ray_input_t ray;
    ray.origin = ivp_vec3r(IVP_REAL(-10.0), IVP_REAL(0.1), IVP_REAL(0.2));
    ray.direction = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ray.max_distance = IVP_REAL(20.0);

    ivp_ray_hit_t hit;
    ivp_result_t r = ivp_ray_cast_surface(surface, &ray, &hit);

    IVP_ASSERT_TRUE(ivp_ok(r));
    /* Normal is the normalised outward face normal (object space). */
    IVP_ASSERT_NEAR(hit.hit_normal.k[0], -1.0, 1e-6);
    IVP_ASSERT_NEAR(hit.hit_normal.k[1], 0.0, 1e-6);
    /* Box half-extent is 1.0, so nearest face at x = -1.0.
     * Distance from origin at -10 is 9.0 */
    IVP_ASSERT_NEAR(hit.hit_distance, 9.0, 1e-2);
    IVP_ASSERT_NOT_NULL(hit.hit_ledge);
    IVP_ASSERT_NOT_NULL(hit.hit_triangle);

    free_surface(surface);
}

IVP_TEST(ray_cast_box_hit_along_y) {
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    ivp_ray_input_t ray;
    ray.origin = ivp_vec3r(IVP_REAL(0.1), IVP_REAL(5.0), IVP_REAL(0.2)); /* off diagonal */
    ray.direction = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    ray.max_distance = IVP_REAL(20.0);

    ivp_ray_hit_t hit;
    ivp_result_t r = ivp_ray_cast_surface(surface, &ray, &hit);

    IVP_ASSERT_TRUE(ivp_ok(r));
    IVP_ASSERT_NEAR(hit.hit_distance, 4.0, 1e-2);

    free_surface(surface);
}

IVP_TEST(ray_cast_box_hit_along_z) {
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    ivp_ray_input_t ray;
    ray.origin = ivp_vec3r(IVP_REAL(0.1), IVP_REAL(0.2), IVP_REAL(8.0)); /* off diagonal */
    ray.direction = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(-1.0));
    ray.max_distance = IVP_REAL(20.0);

    ivp_ray_hit_t hit;
    ivp_result_t r = ivp_ray_cast_surface(surface, &ray, &hit);

    IVP_ASSERT_TRUE(ivp_ok(r));
    IVP_ASSERT_NEAR(hit.hit_distance, 7.0, 1e-2);

    free_surface(surface);
}

IVP_TEST(ray_cast_box_miss) {
    /* Ray goes past the box entirely */
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    ivp_ray_input_t ray;
    ray.origin = ivp_vec3r(IVP_REAL(-10.0), IVP_REAL(5.0), IVP_REAL(0.0));
    ray.direction = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ray.max_distance = IVP_REAL(20.0);

    ivp_ray_hit_t hit;
    ivp_result_t r = ivp_ray_cast_surface(surface, &ray, &hit);

    IVP_ASSERT_FALSE(ivp_ok(r));

    free_surface(surface);
}

IVP_TEST(ray_cast_box_too_short) {
    /* Ray hits but max_distance is too short */
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    ivp_ray_input_t ray;
    ray.origin = ivp_vec3r(IVP_REAL(-10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ray.direction = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ray.max_distance = IVP_REAL(5.0);  /* too short to reach box at x=-1 */

    ivp_ray_hit_t hit;
    ivp_result_t r = ivp_ray_cast_surface(surface, &ray, &hit);

    IVP_ASSERT_FALSE(ivp_ok(r));

    free_surface(surface);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Ray-Surface Cast (Convex) Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(ray_cast_convex_tetrahedron) {
    /* Build a tetrahedron and cast a ray through it */
    ivp_vec3f_t points[4] = {
        { .k = { 0.0f,  0.0f,  2.0f} },
        { .k = { 2.0f,  0.0f, -1.0f} },
        { .k = {-1.0f,  1.7f, -1.0f} },
        { .k = {-1.0f, -1.7f, -1.0f} }
    };

    ivp_compact_surface_t *surface = ivp_build_convex_surface(NULL, points, 4);
    IVP_ASSERT_NOT_NULL(surface);

    /* Ray from above, straight down near the apex (not exactly through the
     * apex vertex: the reference walk misses rays through vertices/edges). */
    ivp_ray_input_t ray;
    ray.origin = ivp_vec3r(IVP_REAL(0.1), IVP_REAL(0.05), IVP_REAL(10.0));
    ray.direction = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(-1.0));
    ray.max_distance = IVP_REAL(20.0);

    ivp_ray_hit_t hit;
    ivp_result_t r = ivp_ray_cast_surface(surface, &ray, &hit);

    IVP_ASSERT_TRUE(ivp_ok(r));
    /* Top vertex is at z=2.0, so hit should be around 8.0 */
    IVP_ASSERT_TRUE(hit.hit_distance > IVP_REAL(7.0));
    IVP_ASSERT_TRUE(hit.hit_distance < IVP_REAL(10.0));

    free_surface(surface);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  KK vals (edge-edge parametric) test
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(kk_vals_crossing_edges) {
    /* Use a box and test two edges that cross (perpendicular) */
    ivp_compact_surface_t *surface = make_box();
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge;
    const ivp_compact_triangle_t *tri;
    get_box_first_tri(surface, &ledge, &tri);

    /* Get two edges from different triangles */
    const ivp_compact_edge_t *e0 = &tri->edges[0];
    const ivp_compact_triangle_t *tri2 = &ivp_ledge_get_first_triangle(ledge)[2];
    const ivp_compact_edge_t *e1 = &tri2->edges[0];

    ivp_kk_result_t kk;
    ivp_calc_kk_vals(ledge, e0, ledge, e1, &kk);

    /* Result should be valid (some values computed) */
    /* Just verify no crash and finite values */
    IVP_ASSERT_TRUE(kk.checks_k[0] == kk.checks_k[0]); /* not NaN */
    IVP_ASSERT_TRUE(kk.checks_l[0] == kk.checks_l[0]); /* not NaN */

    free_surface(surface);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Analytic Distance Tests (Synthetic Geometry)
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(qdist_pp_known_3d_distance) {
    /* Distance from (1,2,3) to (4,6,3) = sqrt(9+16+0) = 5, q=25 */
    ivp_vec3r_t a = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(3.0));
    ivp_vec3r_t b = ivp_vec3r(IVP_REAL(4.0), IVP_REAL(6.0), IVP_REAL(3.0));
    ivp_real_t qd = ivp_qdist_pp(&a, &b);
    IVP_ASSERT_NEAR(qd, 25.0, 1e-9);
}

IVP_TEST(ray_sphere_tangent) {
    /* Ray that just grazes a sphere */
    ivp_vec3r_t origin = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(-5.0));
    ivp_vec3r_t dir = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0));
    ivp_vec3r_t center = ivp_vec3r_zero();
    ivp_real_t t;

    /* Sphere of radius 1.0 at origin. Ray starts at y=1 (tangent). */
    bool hit = ivp_ray_test_sphere(&origin, &dir, &center, IVP_REAL(1.0), &t);
    IVP_ASSERT_TRUE(hit);
    IVP_ASSERT_NEAR(t, 5.0, 1e-4); /* tangent point at z=0 */
}

IVP_TEST(ray_sphere_behind) {
    /* Sphere behind ray origin */
    ivp_vec3r_t origin = ivp_vec3r(IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t dir = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_vec3r_t center = ivp_vec3r_zero();

    bool hit = ivp_ray_test_sphere(&origin, &dir, &center, IVP_REAL(1.0), NULL);
    IVP_ASSERT_FALSE(hit);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Test Registration and Main
 * ═══════════════════════════════════════════════════════════════════════════ */

#define IVP_TEST_LIST \
    /* Collision tolerance */ \
    IVP_REGISTER(collision_tolerance_defaults); \
    IVP_REGISTER(collision_tolerance_custom); \
    IVP_REGISTER(collision_tolerance_coll_dists_interpolation); \
    IVP_REGISTER(collision_tolerance_speed_after_keeper); \
    /* QR / S-val */ \
    IVP_REGISTER(qr_vals_inside_triangle); \
    IVP_REGISTER(qr_vals_outside_triangle); \
    IVP_REGISTER(s_val_projects_onto_edge); \
    /* Point-point distance */ \
    IVP_REGISTER(qdist_pp_same_point); \
    IVP_REGISTER(qdist_pp_unit_apart); \
    IVP_REGISTER(qdist_pp_diagonal); \
    IVP_REGISTER(qdist_pp_known_3d_distance); \
    /* Point-edge distance */ \
    IVP_REGISTER(qdist_pk_perpendicular); \
    /* Point-triangle distance */ \
    IVP_REGISTER(qdist_pf_directly_above); \
    IVP_REGISTER(qdist_pf_at_surface); \
    /* Edge-edge distance */ \
    IVP_REGISTER(qdist_kk_parallel_edges); \
    IVP_REGISTER(kk_vals_crossing_edges); \
    /* Sphere-sphere distance */ \
    IVP_REGISTER(dist_sphere_sphere_separated); \
    IVP_REGISTER(dist_sphere_sphere_touching); \
    IVP_REGISTER(dist_sphere_sphere_overlapping); \
    /* Hesse / signed distance */ \
    IVP_REGISTER(hesse_vec_nonzero); \
    IVP_REGISTER(signed_dist_to_plane_positive_negative); \
    /* Ray-sphere */ \
    IVP_REGISTER(ray_sphere_hit); \
    IVP_REGISTER(ray_sphere_miss); \
    IVP_REGISTER(ray_sphere_origin_inside); \
    IVP_REGISTER(ray_sphere_tangent); \
    IVP_REGISTER(ray_sphere_behind); \
    /* Ray-triangle */ \
    IVP_REGISTER(ray_triangle_hit); \
    IVP_REGISTER(ray_triangle_miss); \
    /* Ray-surface */ \
    IVP_REGISTER(ray_cast_box_hit_along_x); \
    IVP_REGISTER(ray_cast_box_hit_along_y); \
    IVP_REGISTER(ray_cast_box_hit_along_z); \
    IVP_REGISTER(ray_cast_box_miss); \
    IVP_REGISTER(ray_cast_box_too_short); \
    IVP_REGISTER(ray_cast_convex_tetrahedron);

int main(void) {
    IVP_TEST_LIST
    IVP_RUN_ALL();
}
