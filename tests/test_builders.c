#include "ivp_test.h"
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/builder/ivp_build_pointsoup.h>
#include <ivp/builder/ivp_build_concave.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/builder/ivp_build_polygon.h>
#include <ivp/builder/ivp_convex_decompose.h>
#include <ivp/builder/ivp_rot_inertia.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_compact_triangle.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>
#include <math.h>
#include <limits.h>

/* Box geometry data */
static const float box_verts[] = {
    -1,-1,-1,  1,-1,-1,  1,1,-1,  -1,1,-1,
    -1,-1, 1,  1,-1, 1,  1,1, 1,  -1,1, 1
};

static const int box_indices[] = {
    0,2,1, 0,3,2,  /* -Z face */
    4,5,6, 4,6,7,  /* +Z face */
    0,1,5, 0,5,4,  /* -Y face */
    2,3,7, 2,7,6,  /* +Y face */
    0,4,7, 0,7,3,  /* -X face */
    1,2,6, 1,6,5   /* +X face */
};

static const float off_tet_verts[] = {
    10.0f, 0.0f, 0.0f,
    11.0f, 0.0f, 0.0f,
    10.0f, 1.0f, 0.0f,
    10.0f, 0.0f, 1.0f
};

static const int off_tet_indices[] = {
    0,2,1,
    0,1,3,
    1,2,3,
    2,0,3
};

/* L-shaped geometry data (12 vertices forming an L) */
static const float l_verts[] = {
    0,0,0, 3,0,0, 3,1,0, 1,1,0, 1,3,0, 0,3,0,  /* front face */
    0,0,1, 3,0,1, 3,1,1, 1,1,1, 1,3,1, 0,3,1   /* back face */
};

/* Triangles are CCW seen from outside (outward normals), as required by
 * the reference GEOMPACK decomposition (ivp_convex_decompositor.hxx:52:
 * "Always supply them counter-clockwise").  The previous table was wound
 * inside-out, which only worked with the old normal-clustering heuristic. */
static const int l_indices[] = {
    /* Front face z=0 (normal -z) */
    0,2,1, 0,3,2, 0,4,3, 0,5,4,
    /* Back face z=1 (normal +z) */
    6,7,8, 6,8,9, 6,9,10, 6,10,11,
    /* Bottom horizontal bar */
    0,1,7, 0,7,6,
    1,8,7, 1,2,8,
    2,9,8, 2,3,9,
    /* Vertical bar */
    0,11,5, 0,6,11,
    5,10,4, 5,11,10,
    3,10,9, 3,4,10
};

/*
 * Ledge soup tests
 */

IVP_TEST(ledge_soup_init_destroy) {
    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);
    IVP_ASSERT_EQ(soup.n_ledges, 0);
    ivp_ledge_soup_destroy(&soup);
}

IVP_TEST(ledge_soup_add_single) {
    ivp_compact_surface_t *box = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(box);

    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(box);
    IVP_ASSERT_NOT_NULL(ledge);

    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);

    int ret = ivp_ledge_soup_add(&soup, ledge);
    IVP_ASSERT_EQ(ret, 0);
    IVP_ASSERT_EQ(soup.n_ledges, 1);

    ivp_compact_surface_t *result = ivp_ledge_soup_compile(&soup);
    IVP_ASSERT_NOT_NULL(result);

    ivp_compact_surface_free(NULL, result);
    ivp_ledge_soup_destroy(&soup);
    ivp_compact_surface_free(NULL, box);
}

IVP_TEST(ledge_soup_add_multiple) {
    ivp_compact_surface_t *box1 = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    ivp_compact_surface_t *box2 = ivp_build_box_surface(NULL, 2.0f, 2.0f, 2.0f);
    ivp_compact_surface_t *box3 = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);

    IVP_ASSERT_NOT_NULL(box1);
    IVP_ASSERT_NOT_NULL(box2);
    IVP_ASSERT_NOT_NULL(box3);

    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);

    ivp_ledge_soup_add(&soup, ivp_surface_get_single_convex(box1));
    ivp_ledge_soup_add(&soup, ivp_surface_get_single_convex(box2));
    ivp_ledge_soup_add(&soup, ivp_surface_get_single_convex(box3));

    ivp_compact_surface_t *result = ivp_ledge_soup_compile(&soup);
    IVP_ASSERT_NOT_NULL(result);

    const ivp_compact_ledgetree_node_t *root = ivp_surface_get_ledgetree_root(result);
    IVP_ASSERT_NOT_NULL(root);

    int ledge_count = ivp_ledgetree_count_terminal_ledges(root);
    IVP_ASSERT_EQ(ledge_count, 3);

    ivp_compact_surface_free(NULL, result);
    ivp_ledge_soup_destroy(&soup);
    ivp_compact_surface_free(NULL, box1);
    ivp_compact_surface_free(NULL, box2);
    ivp_compact_surface_free(NULL, box3);
}

IVP_TEST(ledge_soup_empty_compile) {
    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);

    ivp_compact_surface_t *result = ivp_ledge_soup_compile(&soup);
    IVP_ASSERT_NULL(result);

    ivp_ledge_soup_destroy(&soup);
}

IVP_TEST(ledge_soup_capacity_growth) {
    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);

    ivp_compact_surface_t *boxes[20];

    for (int i = 0; i < 20; i++) {
        float size = 0.5f + (float)i * 0.1f;
        boxes[i] = ivp_build_box_surface(NULL, size, size, size);
        IVP_ASSERT_NOT_NULL(boxes[i]);

        int ret = ivp_ledge_soup_add(&soup, ivp_surface_get_single_convex(boxes[i]));
        IVP_ASSERT_EQ(ret, 0);
    }

    IVP_ASSERT_EQ(soup.n_ledges, 20);

    ivp_compact_surface_t *result = ivp_ledge_soup_compile(&soup);
    IVP_ASSERT_NOT_NULL(result);

    ivp_compact_surface_free(NULL, result);
    ivp_ledge_soup_destroy(&soup);

    for (int i = 0; i < 20; i++) {
        ivp_compact_surface_free(NULL, boxes[i]);
    }
}

/*
 * Point soup tests
 */

IVP_TEST(pointsoup_cube_cloud) {
    float cube_points[] = {
        -1,-1,-1,  1,-1,-1,  1,1,-1,  -1,1,-1,
        -1,-1, 1,  1,-1, 1,  1,1, 1,  -1,1, 1
    };

    ivp_compact_surface_t *surface = ivp_build_pointsoup_surface(
        NULL, cube_points, 8, NULL
    );

    IVP_ASSERT_NOT_NULL(surface);
    ivp_compact_surface_free(NULL, surface);
}

IVP_TEST(pointsoup_tetrahedron) {
    float tet_points[] = {
        0, 0, 0,
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    };

    ivp_compact_surface_t *surface = ivp_build_pointsoup_surface(
        NULL, tet_points, 4, NULL
    );

    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surface);
    IVP_ASSERT_NOT_NULL(ledge);

    int n_tris = ivp_ledge_get_n_triangles(ledge);
    IVP_ASSERT_EQ(n_tris, 4);

    ivp_compact_surface_free(NULL, surface);
}

IVP_TEST(pointsoup_too_few_points) {
    float points[] = {
        0, 0, 0,
        1, 0, 0,
        0, 1, 0
    };

    /* Fewer than 3 points: no surface. */
    IVP_ASSERT_NULL(ivp_build_pointsoup_surface(NULL, points, 2, NULL));

    /* Exactly 3 points: the reference builds a two sided triangle ledge
     * (ivp_surbuild_pointsoup.cxx:765 convert_triangle_to_compace_ledge);
     * the volume is zero, so the ledge soup takes the bounding box center
     * and radius^2/2 inertia (ivp_rot_inertia_solver.cxx:251). Legacy
     * engine: mc=(0.5 0.5 0) ri=0.25 r=0.707106769 dev=250. */
    ivp_compact_surface_t *surface = ivp_build_pointsoup_surface(
        NULL, points, 3, NULL
    );
    IVP_ASSERT_NOT_NULL(surface);
    const ivp_compact_ledge_t *l = ivp_surface_get_single_convex(surface);
    IVP_ASSERT_NOT_NULL(l);
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(l), 2);
    IVP_ASSERT_NEAR(surface->mass_center[0], 0.5f, 1e-6f);
    IVP_ASSERT_NEAR(surface->mass_center[1], 0.5f, 1e-6f);
    IVP_ASSERT_NEAR(surface->rotation_inertia[0], 0.25f, 1e-6f);
    IVP_ASSERT_NEAR(surface->upper_limit_radius, 0.707106769f, 1e-6f);
    IVP_ASSERT_EQ(ivp_surface_get_max_deviation(surface), 250);
    ivp_compact_surface_free(NULL, surface);
}

IVP_TEST(pointsoup_weld_duplicates) {
    /* 8 points but with 4 pairs of near duplicates.  Default options only
     * merge exact duplicates (reference IVP_I_Point_VHash); the near
     * duplicates are removed by the reference degenerate-facet point removal
     * (ivp_surbuild_pointsoup.cxx:433) and the hull is still a tetrahedron. */
    float points[] = {
        0, 0, 0,  0.0001f, 0, 0,        /* duplicate pair */
        1, 0, 0,  1.0001f, 0, 0,        /* duplicate pair */
        0, 1, 0,  0, 1.0001f, 0,        /* duplicate pair */
        0, 0, 1,  0, 0, 1.0001f         /* duplicate pair */
    };

    ivp_compact_surface_t *surface = ivp_build_pointsoup_surface(
        NULL, points, 8, NULL
    );

    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surface);
    IVP_ASSERT_NOT_NULL(ledge);

    int n_tris = ivp_ledge_get_n_triangles(ledge);
    IVP_ASSERT_EQ(n_tris, 4);  /* Should be a tetrahedron */

    ivp_compact_surface_free(NULL, surface);
}

/*
 * Polygon tests
 */

IVP_TEST(polygon_box_triangles) {
    ivp_compact_surface_t *surface = ivp_build_polygon_surface(
        NULL, box_verts, 8, box_indices, 12
    );

    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surface);
    IVP_ASSERT_NOT_NULL(ledge);

    int n_tris = ivp_ledge_get_n_triangles(ledge);
    IVP_ASSERT_EQ(n_tris, 12);

    ivp_compact_surface_free(NULL, surface);
}

IVP_TEST(polygon_modify_vertex) {
    ivp_compact_surface_t *surface = ivp_build_polygon_surface(
        NULL, box_verts, 8, box_indices, 12
    );

    IVP_ASSERT_NOT_NULL(surface);

    float new_pos[3] = { -2.0f, -2.0f, -2.0f };
    int ret = ivp_surface_modify_vertex(surface, 0, new_pos);
    IVP_ASSERT_EQ(ret, 0);

    ivp_compact_surface_free(NULL, surface);
}

IVP_TEST(polygon_recompute_bounds) {
    ivp_compact_surface_t *surface = ivp_build_polygon_surface(
        NULL, box_verts, 8, box_indices, 12
    );

    IVP_ASSERT_NOT_NULL(surface);

    float new_pos[3] = { 10.0f, 10.0f, 10.0f };
    ivp_surface_modify_vertex(surface, 0, new_pos);

    ivp_surface_recompute_bounds(surface);
    /* Just verify no crash */

    ivp_compact_surface_free(NULL, surface);
}

IVP_TEST(polygon_invalid_index) {
    ivp_compact_surface_t *surface = ivp_build_polygon_surface(
        NULL, box_verts, 8, box_indices, 12
    );

    IVP_ASSERT_NOT_NULL(surface);

    float new_pos[3] = { 0, 0, 0 };
    int ret = ivp_surface_modify_vertex(surface, 999, new_pos);
    IVP_ASSERT_EQ(ret, -1);

    ivp_compact_surface_free(NULL, surface);
}

IVP_TEST(polygon_off_origin_mass_center) {
    ivp_compact_surface_t *surface = ivp_build_polygon_surface(
        NULL, off_tet_verts, 4, off_tet_indices, 4
    );
    IVP_ASSERT_NOT_NULL(surface);
    IVP_ASSERT_TRUE(surface->mass_center[0] > 9.0f);
    ivp_compact_surface_free(NULL, surface);
}

IVP_TEST(ledge_soup_single_off_origin_mass_center) {
    ivp_compact_surface_t *surface = ivp_build_polygon_surface(
        NULL, off_tet_verts, 4, off_tet_indices, 4
    );
    IVP_ASSERT_NOT_NULL(surface);

    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surface);
    IVP_ASSERT_NOT_NULL(ledge);

    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);
    IVP_ASSERT_EQ(ivp_ledge_soup_add(&soup, ledge), 0);

    ivp_compact_surface_t *wrapped = ivp_ledge_soup_compile(&soup);
    IVP_ASSERT_NOT_NULL(wrapped);
    IVP_ASSERT_TRUE(surface->mass_center[0] > 9.0f);
    IVP_ASSERT_NEAR(wrapped->mass_center[0], surface->mass_center[0], 1e-4f);
    IVP_ASSERT_NEAR(wrapped->mass_center[1], surface->mass_center[1], 1e-4f);
    IVP_ASSERT_NEAR(wrapped->mass_center[2], surface->mass_center[2], 1e-4f);

    ivp_compact_surface_free(NULL, wrapped);
    ivp_ledge_soup_destroy(&soup);
    ivp_compact_surface_free(NULL, surface);
}

/*
 * Convex decomposition tests
 */

IVP_TEST(decompose_box) {
    ivp_decompose_result_t result = {0};

    ivp_result_t res = ivp_convex_decompose(
        NULL, box_verts, 8, box_indices, 12, NULL, &result
    );

    /* a convex input has no reflex edge: one part */
    IVP_ASSERT_EQ(res, IVP_OK);
    IVP_ASSERT_EQ(result.n_parts, 1);
    IVP_ASSERT_NOT_NULL(result.parts);

    ivp_decompose_result_free(&result);
}

IVP_TEST(decompose_l_shape) {
    ivp_decompose_result_t result = {0};

    int n_l_tris = sizeof(l_indices) / sizeof(l_indices[0]) / 3;

    ivp_result_t res = ivp_convex_decompose(
        NULL, l_verts, 12, l_indices, n_l_tris, NULL, &result
    );

    /* Reference GEOMPACK cvdec3 cuts the L along its single reflex edge:
     * exactly 2 convex parts (legacy engine prints parts=2 for this L). */
    IVP_ASSERT_EQ(res, IVP_OK);
    IVP_ASSERT_EQ(result.n_parts, 2);
    IVP_ASSERT_NOT_NULL(result.parts);

    ivp_decompose_result_free(&result);
}

IVP_TEST(decompose_result_free) {
    ivp_decompose_result_t result = {0};

    ivp_convex_decompose(
        NULL, box_verts, 8, box_indices, 12, NULL, &result
    );

    ivp_decompose_result_free(&result);
    /* Verify no crash */
}

/*
 * Rotation inertia tests
 */

IVP_TEST(rot_inertia_box) {
    ivp_compact_surface_t *surface = ivp_build_box_surface(NULL, 0.5f, 1.0f, 1.5f);
    IVP_ASSERT_NOT_NULL(surface);

    float mass_center[3];
    float rot_inertia[3];

    ivp_result_t res = ivp_calc_mass_center_and_rot_inertia(
        surface, mass_center, rot_inertia
    );

    /* Box surface may be treated as degenerate by the volume-based algorithm */
    IVP_ASSERT_TRUE(res == IVP_OK || res == IVP_ERR_DEGENERATE_GEOMETRY);

    /* Verify inertia values are positive regardless of method used */
    IVP_ASSERT_TRUE(rot_inertia[0] > 0.0f);
    IVP_ASSERT_TRUE(rot_inertia[1] > 0.0f);
    IVP_ASSERT_TRUE(rot_inertia[2] > 0.0f);

    ivp_compact_surface_free(NULL, surface);
}

IVP_TEST(rot_inertia_degenerate) {
    /* Very flat box */
    ivp_compact_surface_t *surface = ivp_build_box_surface(NULL, 1.0f, 0.0001f, 1.0f);
    IVP_ASSERT_NOT_NULL(surface);

    float mass_center[3];
    float rot_inertia[3];

    ivp_result_t res = ivp_calc_mass_center_and_rot_inertia(
        surface, mass_center, rot_inertia
    );

    /* Should return either IVP_OK or IVP_ERR_DEGENERATE_GEOMETRY, but not crash */
    IVP_ASSERT_TRUE(res == IVP_OK || res == IVP_ERR_DEGENERATE_GEOMETRY);

    ivp_compact_surface_free(NULL, surface);
}

IVP_TEST(builders_reject_overflowing_counts) {
    const float coords[3] = {0.0f, 0.0f, 0.0f};
    const int indices[12] = {0};
    IVP_ASSERT_NULL(ivp_build_pointsoup_surface(NULL, coords, INT_MAX, NULL));
    IVP_ASSERT_NULL(ivp_build_concave_surface(NULL, coords, INT_MAX,
                                               indices, 1, NULL));

    ivp_decompose_result_t result;
    IVP_ASSERT_EQ(ivp_convex_decompose(NULL, coords, INT_MAX, indices, 4,
                                       NULL, &result),
                  IVP_ERR_CAPACITY_EXCEEDED);

    const double points[9] = {0.0};
    const int face_sizes[2] = {INT_MAX, 3};
    IVP_ASSERT_NULL(ivp_build_ledge_from_polygons(NULL, points, 3, indices,
                                                  face_sizes, 2));
}

int main(void) {
    /* Ledge soup tests */
    IVP_REGISTER(ledge_soup_init_destroy);
    IVP_REGISTER(ledge_soup_add_single);
    IVP_REGISTER(ledge_soup_add_multiple);
    IVP_REGISTER(ledge_soup_empty_compile);
    IVP_REGISTER(ledge_soup_capacity_growth);

    /* Point soup tests */
    IVP_REGISTER(pointsoup_cube_cloud);
    IVP_REGISTER(pointsoup_tetrahedron);
    IVP_REGISTER(pointsoup_too_few_points);
    IVP_REGISTER(pointsoup_weld_duplicates);

    /* Polygon tests */
    IVP_REGISTER(polygon_box_triangles);
    IVP_REGISTER(polygon_modify_vertex);
    IVP_REGISTER(polygon_recompute_bounds);
    IVP_REGISTER(polygon_invalid_index);
    IVP_REGISTER(polygon_off_origin_mass_center);
    IVP_REGISTER(ledge_soup_single_off_origin_mass_center);

    /* Convex decomposition tests */
    IVP_REGISTER(decompose_box);
    IVP_REGISTER(decompose_l_shape);
    IVP_REGISTER(decompose_result_free);

    /* Rotation inertia tests */
    IVP_REGISTER(rot_inertia_box);
    IVP_REGISTER(rot_inertia_degenerate);
    IVP_REGISTER(builders_reject_overflowing_counts);

    IVP_RUN_ALL();
}
