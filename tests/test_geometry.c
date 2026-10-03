/* test_geometry.c — Unit tests for M2: geometry (compact surface / ledge / edge)
 *
 * Tests:
 *   1. Struct size assertions
 *   2. Edge bit field packing/unpacking (including signed opposite_index)
 *   3. Triangle bit field packing/unpacking
 *   4. Box builder: construct and verify topology
 *   5. Box builder: edge navigation (next/prev/opposite consistency)
 *   6. Box builder: triangle → ledge upward navigation
 *   7. Box builder: ledgetree traversal
 *   8. Surface manager (polygon) basic operations
 */

#include "ivp_test.h"
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/geometry/ivp_surman_grid.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/base/ivp_allocator.h>
#include <string.h>

/* ── T1: Struct sizes ───────────────────────────────────────────────────── */

IVP_TEST(struct_sizes) {
    IVP_ASSERT_EQ((int)sizeof(ivp_compact_edge_t), 4);
    IVP_ASSERT_EQ((int)sizeof(ivp_compact_triangle_t), 16);
    IVP_ASSERT_EQ((int)sizeof(ivp_compact_ledge_t), 16);
    IVP_ASSERT_EQ((int)sizeof(ivp_compact_poly_point_t), 16);
    IVP_ASSERT_EQ((int)sizeof(ivp_compact_ledgetree_node_t), 28);
    IVP_ASSERT_EQ((int)sizeof(ivp_compact_surface_t), 48);

    /* Critical: ledge and triangle must be the same size */
    IVP_ASSERT_EQ((int)sizeof(ivp_compact_ledge_t),
                  (int)sizeof(ivp_compact_triangle_t));
}

/* ── T2: Edge bit field round-trip ──────────────────────────────────────── */

IVP_TEST(edge_bitfields) {
    ivp_compact_edge_t e;

    /* Basic packing */
    ivp_edge_set(&e, 42, 7, 0);
    IVP_ASSERT_EQ(ivp_edge_get_start_point_index(&e), 42);
    IVP_ASSERT_EQ(ivp_edge_get_opposite_index(&e), 7);
    IVP_ASSERT_EQ(ivp_edge_get_is_virtual(&e), 0);

    /* Negative opposite index */
    ivp_edge_set(&e, 100, -22, 1);
    IVP_ASSERT_EQ(ivp_edge_get_start_point_index(&e), 100);
    IVP_ASSERT_EQ(ivp_edge_get_opposite_index(&e), -22);
    IVP_ASSERT_EQ(ivp_edge_get_is_virtual(&e), 1);

    /* Max values */
    ivp_edge_set(&e, 65535, 16383, 1);
    IVP_ASSERT_EQ(ivp_edge_get_start_point_index(&e), 65535);
    IVP_ASSERT_EQ(ivp_edge_get_opposite_index(&e), 16383);
    IVP_ASSERT_EQ(ivp_edge_get_is_virtual(&e), 1);

    /* Min negative opposite */
    ivp_edge_set(&e, 0, -16383, 0);
    IVP_ASSERT_EQ(ivp_edge_get_start_point_index(&e), 0);
    IVP_ASSERT_EQ(ivp_edge_get_opposite_index(&e), -16383);
    IVP_ASSERT_EQ(ivp_edge_get_is_virtual(&e), 0);

    /* Full signed 15-bit lower bound */
    ivp_edge_set(&e, 0, -16384, 0);
    IVP_ASSERT_EQ(ivp_edge_get_opposite_index(&e), -16384);

    /* Zero */
    ivp_edge_set(&e, 0, 0, 0);
    IVP_ASSERT_EQ(ivp_edge_get_start_point_index(&e), 0);
    IVP_ASSERT_EQ(ivp_edge_get_opposite_index(&e), 0);
    IVP_ASSERT_EQ(ivp_edge_get_is_virtual(&e), 0);
}

typedef struct {
    int count;
    int stop_after;
} stop_visitor_ctx_t;

static bool stop_after_n_visitor(const ivp_compact_ledge_t *ledge, void *user_data) {
    (void)ledge;
    stop_visitor_ctx_t *ctx = (stop_visitor_ctx_t *)user_data;
    ctx->count++;
    return ctx->count < ctx->stop_after;
}

/* ── T3: Triangle bit field round-trip ──────────────────────────────────── */

IVP_TEST(triangle_bitfields) {
    ivp_compact_triangle_t t;
    ivp_tri_init(&t);

    ivp_tri_set_tri_index(&t, 123);
    ivp_tri_set_pierce_index(&t, 456);
    ivp_tri_set_material_index(&t, 7);
    ivp_tri_set_is_virtual(&t, 1);

    IVP_ASSERT_EQ(ivp_tri_get_tri_index(&t), 123);
    IVP_ASSERT_EQ(ivp_tri_get_pierce_index(&t), 456);
    IVP_ASSERT_EQ(ivp_tri_get_material_index(&t), 7);
    IVP_ASSERT_EQ(ivp_tri_get_is_virtual(&t), 1);

    /* Max values */
    ivp_tri_set_tri_index(&t, 4095);
    ivp_tri_set_pierce_index(&t, 4095);
    ivp_tri_set_material_index(&t, 127);
    IVP_ASSERT_EQ(ivp_tri_get_tri_index(&t), 4095);
    IVP_ASSERT_EQ(ivp_tri_get_pierce_index(&t), 4095);
    IVP_ASSERT_EQ(ivp_tri_get_material_index(&t), 127);
}

/* ── T4: Ledge bit field round-trip ─────────────────────────────────────── */

IVP_TEST(ledge_bitfields) {
    ivp_compact_ledge_t l;
    ivp_ledge_init(&l);

    ivp_ledge_set_has_children(&l, 2);
    ivp_ledge_set_is_compact(&l, 1);
    ivp_ledge_set_size(&l, 336);  /* 21 * 16 */
    l.n_triangles = 12;

    IVP_ASSERT_EQ(ivp_ledge_get_has_children(&l), 2);
    IVP_ASSERT_EQ(ivp_ledge_get_is_compact(&l), 1);
    IVP_ASSERT_EQ(ivp_ledge_get_size(&l), 336);
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(&l), 12);
    IVP_ASSERT_EQ(ivp_ledge_get_size_div_16(&l), 21);
}

/* ── T5: Box builder — basic structure ──────────────────────────────────── */

IVP_TEST(box_builder_basic) {
    const ivp_allocator_t *alloc = ivp_default_allocator();
    ivp_compact_surface_t *surface = ivp_build_box_surface(alloc, 1.0f, 2.0f, 3.0f);
    IVP_ASSERT_NOT_NULL(surface);

    /* Surface header */
    IVP_ASSERT_NEAR(surface->mass_center[0], 0.0f, 1e-6f);
    IVP_ASSERT_NEAR(surface->mass_center[1], 0.0f, 1e-6f);
    IVP_ASSERT_NEAR(surface->mass_center[2], 0.0f, 1e-6f);
    IVP_ASSERT_NEAR(surface->upper_limit_radius, sqrtf(1.0f + 4.0f + 9.0f), 1e-5f);

    /* Rotation inertia for unit mass, reference IVP_Rot_Inertia_Solver
     * (ivp_rot_inertia_solver.cxx:236-248): 2D moments a=hx²/3, b=hy²/3,
     * c=hz²/3 and I_x = sqrt(b²+c²) (cyclic) — not the physical (b+c). */
    {
        float a = 1.0f / 3.0f, b = 4.0f / 3.0f, c = 9.0f / 3.0f;
        IVP_ASSERT_NEAR(surface->rotation_inertia[0], sqrtf(b * b + c * c), 1e-5f);
        IVP_ASSERT_NEAR(surface->rotation_inertia[1], sqrtf(a * a + c * c), 1e-5f);
        IVP_ASSERT_NEAR(surface->rotation_inertia[2], sqrtf(a * a + b * b), 1e-5f);
    }
    /* deviation factor (ivp_surbuild_ledge_soup.cxx:1719): every vertex is
     * sqrt(14) from the center, in-plane distance of the face corners
     * |p x n| max = sqrt(13) -> int(1 + sqrt(13)/(sqrt(14)/250)) = 241 */
    IVP_ASSERT_EQ(ivp_surface_get_max_deviation(surface), 241);

    /* Ledgetree root */
    const ivp_compact_ledgetree_node_t *root =
        ivp_surface_get_ledgetree_root(surface);
    IVP_ASSERT_NOT_NULL(root);
    IVP_ASSERT_TRUE(ivp_ledgetree_is_terminal(root));

    /* Single convex */
    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surface);
    IVP_ASSERT_NOT_NULL(ledge);
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(ledge), 12);
    IVP_ASSERT_EQ(ivp_ledge_get_n_points(ledge), 8);
    IVP_ASSERT_TRUE(ivp_ledge_is_terminal(ledge));

    /* Clean up */
    ivp_compact_surface_free(alloc, surface);
}

/* ── T6: Box builder — vertex positions ─────────────────────────────────── */

IVP_TEST(box_builder_vertices) {
    ivp_compact_surface_t *surface = ivp_build_box_surface(NULL, 1.0f, 2.0f, 3.0f);
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surface);
    IVP_ASSERT_NOT_NULL(ledge);

    const ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array(ledge);
    int np = ivp_ledge_get_n_points(ledge);
    IVP_ASSERT_EQ(np, 8);

    /* Verify all 8 vertices exist with correct coordinates, in the
     * reference compact point order (pointsoup of the corners
     * for sx, sy, sz in {-1,+1}: qhull 2.6 + IVP_SurfaceBuilder_Polygon_Convex,
     * points ordered by first use in the generated triangles). */
    static const int sign[8][3] = {
        { -1, +1, +1 }, { +1, -1, +1 }, { +1, +1, +1 }, { -1, -1, +1 },
        { -1, +1, -1 }, { +1, +1, -1 }, { -1, -1, -1 }, { +1, -1, -1 },
    };
    for (int idx = 0; idx < 8; idx++) {
        IVP_ASSERT_NEAR(pts[idx].k[0], (float)sign[idx][0] * 1.0f, 1e-6f);
        IVP_ASSERT_NEAR(pts[idx].k[1], (float)sign[idx][1] * 2.0f, 1e-6f);
        IVP_ASSERT_NEAR(pts[idx].k[2], (float)sign[idx][2] * 3.0f, 1e-6f);
    }

    ivp_compact_surface_free(NULL, surface);
}

/* ── T7: Box builder — edge navigation consistency ──────────────────────── */

IVP_TEST(box_edge_navigation) {
    ivp_compact_surface_t *surface = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surface);
    IVP_ASSERT_NOT_NULL(ledge);

    const ivp_compact_triangle_t *first_tri = ivp_ledge_get_first_triangle(ledge);
    int n_tri = ivp_ledge_get_n_triangles(ledge);

    for (int t = 0; t < n_tri; t++) {
        const ivp_compact_triangle_t *tri = &first_tri[t];

        /* Verify tri_index */
        IVP_ASSERT_EQ(ivp_tri_get_tri_index(tri), t);

        for (int e = 0; e < 3; e++) {
            const ivp_compact_edge_t *edge = ivp_tri_get_edge(tri, e);

            /* get_triangle() should recover the owning triangle */
            const ivp_compact_triangle_t *owner = ivp_edge_get_triangle(edge);
            IVP_ASSERT_TRUE(owner == tri);

            /* get_edge_index() should match */
            IVP_ASSERT_EQ(ivp_edge_get_edge_index(edge), e);

            /* next(next(next(edge))) == edge (cycle of 3) */
            const ivp_compact_edge_t *e1 = ivp_edge_get_next(edge);
            const ivp_compact_edge_t *e2 = ivp_edge_get_next(e1);
            const ivp_compact_edge_t *e3 = ivp_edge_get_next(e2);
            IVP_ASSERT_TRUE(e3 == edge);

            /* prev(prev(prev(edge))) == edge */
            const ivp_compact_edge_t *p1 = ivp_edge_get_prev(edge);
            const ivp_compact_edge_t *p2 = ivp_edge_get_prev(p1);
            const ivp_compact_edge_t *p3 = ivp_edge_get_prev(p2);
            IVP_ASSERT_TRUE(p3 == edge);

            /* next(prev(edge)) == edge */
            IVP_ASSERT_TRUE(ivp_edge_get_next(ivp_edge_get_prev(edge)) == edge);

            /* opposite(opposite(edge)) == edge (involution) */
            const ivp_compact_edge_t *opp = ivp_edge_get_opposite(edge);
            const ivp_compact_edge_t *opp2 = ivp_edge_get_opposite(opp);
            IVP_ASSERT_TRUE(opp2 == edge);

            /* The opposite edge should be in a different triangle */
            const ivp_compact_triangle_t *opp_tri = ivp_edge_get_triangle(opp);
            IVP_ASSERT_TRUE(opp_tri != tri);
        }
    }

    ivp_compact_surface_free(NULL, surface);
}

/* ── T8: Box builder — manifold check (half-edge mesh consistency) ──── */

IVP_TEST(box_manifold_check) {
    ivp_compact_surface_t *surface = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surface);
    const ivp_compact_triangle_t *first_tri = ivp_ledge_get_first_triangle(ledge);
    const ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array(ledge);
    int n_tri = ivp_ledge_get_n_triangles(ledge);

    /* For each half-edge, verify that opposite half-edge connects the same
     * pair of vertices but in reverse direction. */
    for (int t = 0; t < n_tri; t++) {
        const ivp_compact_triangle_t *tri = &first_tri[t];
        for (int e = 0; e < 3; e++) {
            const ivp_compact_edge_t *edge = ivp_tri_get_edge(tri, e);
            const ivp_compact_edge_t *next = ivp_edge_get_next(edge);
            const ivp_compact_edge_t *opp  = ivp_edge_get_opposite(edge);

            /* This edge: start → next.start (i.e., this.start → end) */
            int this_start = ivp_edge_get_start_point_index(edge);
            int this_end   = ivp_edge_get_start_point_index(next);

            /* Opposite edge: opp.start → opp.next.start */
            const ivp_compact_edge_t *opp_next = ivp_edge_get_next(opp);
            int opp_start = ivp_edge_get_start_point_index(opp);
            int opp_end   = ivp_edge_get_start_point_index(opp_next);

            /* Opposite should reverse the direction */
            IVP_ASSERT_EQ(this_start, opp_end);
            IVP_ASSERT_EQ(this_end, opp_start);
        }
    }

    /* Verify all vertex positions are valid box corners */
    int n_pts = ivp_ledge_get_n_points(ledge);
    for (int i = 0; i < n_pts; i++) {
        float x = pts[i].k[0], y = pts[i].k[1], z = pts[i].k[2];
        IVP_ASSERT_TRUE(x == 1.0f || x == -1.0f);
        IVP_ASSERT_TRUE(y == 1.0f || y == -1.0f);
        IVP_ASSERT_TRUE(z == 1.0f || z == -1.0f);
    }

    ivp_compact_surface_free(NULL, surface);
}

/* ── T9: Box builder — upward navigation (edge→ledge, tri→ledge) ────── */

IVP_TEST(box_upward_navigation) {
    ivp_compact_surface_t *surface = ivp_build_box_surface(NULL, 2.0f, 3.0f, 4.0f);
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surface);
    const ivp_compact_triangle_t *first_tri = ivp_ledge_get_first_triangle(ledge);

    for (int t = 0; t < ivp_ledge_get_n_triangles(ledge); t++) {
        const ivp_compact_triangle_t *tri = &first_tri[t];

        /* Triangle → ledge */
        const ivp_compact_ledge_t *recovered_ledge = ivp_tri_get_compact_ledge(tri);
        IVP_ASSERT_TRUE(recovered_ledge == ledge);

        /* Edge → ledge */
        for (int e = 0; e < 3; e++) {
            const ivp_compact_edge_t *edge = ivp_tri_get_edge(tri, e);
            const ivp_compact_ledge_t *edge_ledge = ivp_edge_get_compact_ledge(edge);
            IVP_ASSERT_TRUE(edge_ledge == ledge);
        }
    }

    ivp_compact_surface_free(NULL, surface);
}

/* ── T10: Ledgetree traversal ───────────────────────────────────────────── */

IVP_TEST(ledgetree_traversal) {
    ivp_compact_surface_t *surface = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledgetree_node_t *root =
        ivp_surface_get_ledgetree_root(surface);

    /* Single convex → exactly 1 terminal ledge */
    int count = ivp_ledgetree_count_terminal_ledges(root);
    IVP_ASSERT_EQ(count, 1);

    /* Bounding sphere */
    IVP_ASSERT_NEAR(root->center[0], 0.0f, 1e-6f);
    IVP_ASSERT_NEAR(root->center[1], 0.0f, 1e-6f);
    IVP_ASSERT_NEAR(root->center[2], 0.0f, 1e-6f);
    IVP_ASSERT_NEAR(root->radius, sqrtf(3.0f), 1e-5f);

    ivp_compact_surface_free(NULL, surface);
}

/* ── T11b: Ledgetree traversal early stop contract ─────────────────────── */

IVP_TEST(ledgetree_traversal_stop) {
    ivp_compact_ledge_t ledges[2];
    ivp_ledge_init(&ledges[0]);
    ivp_ledge_init(&ledges[1]);

    ivp_compact_ledgetree_node_t nodes[3];
    memset(nodes, 0, sizeof(nodes));

    /* Root non-terminal with two terminal children. */
    nodes[0].offset_right_node = (int32_t)((const char *)&nodes[2] - (const char *)&nodes[0]);

    nodes[1].offset_right_node = 0;
    nodes[1].offset_compact_ledge = (int32_t)((const char *)&ledges[0] - (const char *)&nodes[1]);

    nodes[2].offset_right_node = 0;
    nodes[2].offset_compact_ledge = (int32_t)((const char *)&ledges[1] - (const char *)&nodes[2]);

    stop_visitor_ctx_t ctx = { 0, 1 };
    ivp_ledgetree_visit_all(&nodes[0], stop_after_n_visitor, &ctx);
    IVP_ASSERT_EQ(ctx.count, 1);
}

/* ── T11: Surface manager (polygon) ─────────────────────────────────────── */

IVP_TEST(surman_polygon) {
    ivp_compact_surface_t *surface = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(surface);

    ivp_surman_polygon_t sm;
    ivp_surman_polygon_init(&sm, surface);

    IVP_ASSERT_EQ((int)ivp_surman_get_type(&sm.base), (int)IVP_SURMAN_POLYGON);

    /* Single convex */
    const ivp_compact_ledge_t *ledge = ivp_surman_get_single_convex(&sm.base);
    IVP_ASSERT_NOT_NULL(ledge);

    /* Mass center */
    IVP_ASSERT_NEAR(surface->mass_center[0], 0.0f, 1e-6f);
    IVP_ASSERT_NEAR(surface->mass_center[1], 0.0f, 1e-6f);
    IVP_ASSERT_NEAR(surface->mass_center[2], 0.0f, 1e-6f);

    /* Rotation inertia, reference value for a cube of half size h:
     * sqrt(2)/3 h² = 0.4714 h² (ivp_rot_inertia_solver.cxx:236-248;
     * legacy engine prints 0.471404523 for h = 1). */
    IVP_ASSERT_NEAR(surface->rotation_inertia[0], 0.471404523f, 1e-6f);
    IVP_ASSERT_NEAR(surface->rotation_inertia[1], 0.471404523f, 1e-6f);
    IVP_ASSERT_NEAR(surface->rotation_inertia[2], 0.471404523f, 1e-6f);

    /* vtable mass properties (ivp_surman_polygon.cxx:204-226) */
    float mc[3], ri[3];
    ivp_surman_get_mass_center(&sm.base, mc);
    ivp_surman_get_rotation_inertia(&sm.base, ri);
    IVP_ASSERT_NEAR(mc[0], 0.0f, 1e-6f);
    IVP_ASSERT_NEAR(ri[1], 0.471404523f, 1e-6f);
    float center[3] = { 0.0f, 0.0f, 0.0f };
    float rad = 0.0f, dev = 0.0f;
    ivp_surman_get_radius_and_deviation(&sm.base, center, &rad, &dev);
    IVP_ASSERT_NEAR(rad, 1.73205078f, 1e-6f);
    /* 205 * (1/250) * radius */
    IVP_ASSERT_NEAR(dev, 205.0f * (1.0f / 250.0f) * 1.73205078f, 1e-5f);

    ivp_surman_destroy(&sm.base);
    ivp_compact_surface_free(NULL, surface);
}

/* ── T11c: Surman grid query outside bounds ────────────────────────────── */

IVP_TEST(surman_grid_query_outside_bounds) {
    ivp_compact_surface_t *surface = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(surface);

    ivp_surman_grid_t grid;
    ivp_result_t r = ivp_surman_grid_init(&grid, surface, NULL, ivp_default_allocator());
    IVP_ASSERT_EQ(r, IVP_OK);

    float inside_min[3] = { -1.1f, -1.1f, -1.1f };
    float inside_max[3] = {  1.1f,  1.1f,  1.1f };
    int inside_idx[16];
    int inside_count = ivp_surman_grid_query_aabb(&grid, inside_min, inside_max,
                                                  inside_idx, 16);
    IVP_ASSERT_NE(inside_count, 0);

    float outside_min[3] = { 100.0f, 100.0f, 100.0f };
    float outside_max[3] = { 110.0f, 110.0f, 110.0f };
    int outside_idx[16];
    int outside_count = ivp_surman_grid_query_aabb(&grid, outside_min, outside_max,
                                                   outside_idx, 16);
    IVP_ASSERT_EQ(outside_count, 0);

    ivp_surman_grid_destroy(&grid);
    ivp_compact_surface_free(NULL, surface);
}

/* ── T12: Edge — get_start_point with ledge ─────────────────────────────── */

IVP_TEST(edge_start_point) {
    ivp_compact_surface_t *surface = ivp_build_box_surface(NULL, 1.0f, 2.0f, 3.0f);
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surface);
    const ivp_compact_triangle_t *first_tri = ivp_ledge_get_first_triangle(ledge);
    const ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array(ledge);

    /* Check first triangle's first edge start point */
    const ivp_compact_edge_t *e0 = ivp_tri_get_edge(&first_tri[0], 0);
    const ivp_compact_poly_point_t *p0 = ivp_edge_get_start_point(e0, ledge);
    int idx0 = ivp_edge_get_start_point_index(e0);
    IVP_ASSERT_TRUE(p0 == &pts[idx0]);

    ivp_compact_surface_free(NULL, surface);
}

/* ── T13: Convex builder (qhull) — tetrahedron ─────────────────────────── */

IVP_TEST(convex_builder_tetra) {
    ivp_vec3f_t pts[4] = {
        ivp_vec3f(0.0f, 0.0f, 0.0f),
        ivp_vec3f(1.0f, 0.0f, 0.0f),
        ivp_vec3f(0.0f, 1.0f, 0.0f),
        ivp_vec3f(0.0f, 0.0f, 1.0f),
    };

    ivp_compact_surface_t *surface = ivp_build_convex_surface(NULL, pts, 4);
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surface);
    IVP_ASSERT_NOT_NULL(ledge);

    /* tetrahedron triangulation: 4 triangles */
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(ledge), 4);
    IVP_ASSERT_EQ(ivp_ledge_get_n_points(ledge), 4);

    /* every half-edge must have a valid opposite */
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
    int ntri = ivp_ledge_get_n_triangles(ledge);
    for (int t = 0; t < ntri; t++) {
        for (int e = 0; e < 3; e++) {
            const ivp_compact_edge_t *edge = ivp_tri_get_edge(&tri[t], e);
            const ivp_compact_edge_t *opp = ivp_edge_get_opposite(edge);
            IVP_ASSERT_NOT_NULL(opp);
            IVP_ASSERT_TRUE(ivp_edge_get_opposite(opp) == edge);
        }
    }

    ivp_compact_surface_free(NULL, surface);
}

/* ── T14: Convex builder (qhull) — cube cloud with interior point ───────── */

IVP_TEST(convex_builder_cube_cloud) {
    ivp_vec3f_t pts[9] = {
        ivp_vec3f(-1.0f, -1.0f, -1.0f),
        ivp_vec3f(-1.0f, -1.0f,  1.0f),
        ivp_vec3f(-1.0f,  1.0f, -1.0f),
        ivp_vec3f(-1.0f,  1.0f,  1.0f),
        ivp_vec3f( 1.0f, -1.0f, -1.0f),
        ivp_vec3f( 1.0f, -1.0f,  1.0f),
        ivp_vec3f( 1.0f,  1.0f, -1.0f),
        ivp_vec3f( 1.0f,  1.0f,  1.0f),
        ivp_vec3f( 0.0f,  0.0f,  0.0f), /* interior point */
    };

    ivp_compact_surface_t *surface = ivp_build_convex_surface(NULL, pts, 9);
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surface);
    IVP_ASSERT_NOT_NULL(ledge);

    /* cube hull should triangulate to 12 triangles */
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(ledge), 12);
    IVP_ASSERT_EQ(ivp_ledge_get_n_points(ledge), 8);

    ivp_compact_surface_free(NULL, surface);
}

/* ── Test registration ──────────────────────────────────────────────────── */

int main(void) {
    IVP_REGISTER(struct_sizes);
    IVP_REGISTER(edge_bitfields);
    IVP_REGISTER(triangle_bitfields);
    IVP_REGISTER(ledge_bitfields);
    IVP_REGISTER(box_builder_basic);
    IVP_REGISTER(box_builder_vertices);
    IVP_REGISTER(box_edge_navigation);
    IVP_REGISTER(box_manifold_check);
    IVP_REGISTER(box_upward_navigation);
    IVP_REGISTER(ledgetree_traversal);
    IVP_REGISTER(ledgetree_traversal_stop);
    IVP_REGISTER(surman_polygon);
    IVP_REGISTER(surman_grid_query_outside_bounds);
    IVP_REGISTER(edge_start_point);
    IVP_REGISTER(convex_builder_tetra);
    IVP_REGISTER(convex_builder_cube_cloud);
    IVP_RUN_ALL();
}
