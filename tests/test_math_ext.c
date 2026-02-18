// Copyright (c) 2025 IVP C17 Port
// Test suite for geometry utilities and mapping

#include "ivp_test.h"

#include <ivp/math/ivp_geometry.h>
#include <ivp/math/ivp_mapping.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_fast_math.h>
#include <math.h>

#ifdef IVP_REAL_DOUBLE
#define TOL 1e-10
#else
#define TOL 1e-5f
#endif

// ============================================================================
// GEOMETRY TESTS
// ============================================================================

IVP_TEST(hesse_calc_from_points) {
    // Three points on XZ plane: (0,0,0), (1,0,0), (0,0,1)
    ivp_vec3r_t p0 = {{0.0, 0.0, 0.0}};
    ivp_vec3r_t p1 = {{1.0, 0.0, 0.0}};
    ivp_vec3r_t p2 = {{0.0, 0.0, 1.0}};

    ivp_hesse_t plane;
    ivp_result_t result = ivp_hesse_calc(&plane, &p0, &p1, &p2);
    IVP_ASSERT_TRUE(result == IVP_OK);

    // Normal should be (0, 1, 0) or (0, -1, 0)
    ivp_real_t abs_ny = ivp_fabs(plane.normal.k[IVP_INDEX_Y]);
    IVP_ASSERT_NEAR(abs_ny, 1.0, TOL);
    IVP_ASSERT_NEAR(plane.normal.k[IVP_INDEX_X], 0.0, TOL);
    IVP_ASSERT_NEAR(plane.normal.k[IVP_INDEX_Z], 0.0, TOL);

    // hesse_val should be 0 (plane passes through origin)
    IVP_ASSERT_NEAR(ivp_fabs(plane.hesse_val), 0.0, TOL);
}

IVP_TEST(hesse_dist_positive_side) {
    // Create XZ plane at Y=0
    ivp_hesse_t plane;
    plane.normal.k[IVP_INDEX_X] = 0.0;
    plane.normal.k[IVP_INDEX_Y] = 1.0;
    plane.normal.k[IVP_INDEX_Z] = 0.0;
    plane.hesse_val = 0.0;

    // Point above plane
    ivp_vec3r_t point = {{0.0, 5.0, 0.0}};
    ivp_real_t dist = ivp_hesse_dist(&plane, &point);

    IVP_ASSERT_NEAR(dist, 5.0, TOL);
}

IVP_TEST(hesse_dist_negative_side) {
    // Create XZ plane at Y=0
    ivp_hesse_t plane;
    plane.normal.k[IVP_INDEX_X] = 0.0;
    plane.normal.k[IVP_INDEX_Y] = 1.0;
    plane.normal.k[IVP_INDEX_Z] = 0.0;
    plane.hesse_val = 0.0;

    // Point below plane
    ivp_vec3r_t point = {{0.0, -5.0, 0.0}};
    ivp_real_t dist = ivp_hesse_dist(&plane, &point);

    IVP_ASSERT_NEAR(dist, -5.0, TOL);
}

IVP_TEST(hesse_project_onto_plane) {
    // Create XZ plane at Y=0
    ivp_hesse_t plane;
    plane.normal.k[IVP_INDEX_X] = 0.0;
    plane.normal.k[IVP_INDEX_Y] = 1.0;
    plane.normal.k[IVP_INDEX_Z] = 0.0;
    plane.hesse_val = 0.0;

    // Project point (3, 7, 2) onto XZ plane
    ivp_vec3r_t point = {{3.0, 7.0, 2.0}};
    ivp_vec3r_t projected;
    ivp_hesse_project(&plane, &point, &projected);

    // Y coordinate should be ~0, X and Z preserved
    IVP_ASSERT_NEAR(projected.k[IVP_INDEX_X], 3.0, TOL);
    IVP_ASSERT_NEAR(projected.k[IVP_INDEX_Y], 0.0, TOL);
    IVP_ASSERT_NEAR(projected.k[IVP_INDEX_Z], 2.0, TOL);
}

IVP_TEST(hesse_is_parallel) {
    // Two parallel XZ planes at different heights
    ivp_hesse_t plane1;
    plane1.normal.k[IVP_INDEX_X] = 0.0;
    plane1.normal.k[IVP_INDEX_Y] = 1.0;
    plane1.normal.k[IVP_INDEX_Z] = 0.0;
    plane1.hesse_val = 0.0;

    ivp_hesse_t plane2;
    plane2.normal.k[IVP_INDEX_X] = 0.0;
    plane2.normal.k[IVP_INDEX_Y] = 1.0;
    plane2.normal.k[IVP_INDEX_Z] = 0.0;
    plane2.hesse_val = 5.0;

    IVP_ASSERT_TRUE(ivp_hesse_is_parallel(&plane1, &plane2, TOL));

    // XZ plane vs XY plane (not parallel)
    ivp_hesse_t plane3;
    plane3.normal.k[IVP_INDEX_X] = 0.0;
    plane3.normal.k[IVP_INDEX_Y] = 0.0;
    plane3.normal.k[IVP_INDEX_Z] = 1.0;
    plane3.hesse_val = 0.0;

    IVP_ASSERT_FALSE(ivp_hesse_is_parallel(&plane1, &plane3, TOL));
}

IVP_TEST(straight_init) {
    ivp_vec3r_t start = {{0.0, 0.0, 0.0}};
    ivp_vec3r_t direction = {{3.0, 0.0, 0.0}};

    ivp_straight_t line;
    ivp_result_t result = ivp_straight_init(&line, &start, &direction);

    IVP_ASSERT_TRUE(result == IVP_OK);

    // Direction should be normalized to (1, 0, 0)
    IVP_ASSERT_NEAR(line.dir.k[IVP_INDEX_X], 1.0, TOL);
    IVP_ASSERT_NEAR(line.dir.k[IVP_INDEX_Y], 0.0, TOL);
    IVP_ASSERT_NEAR(line.dir.k[IVP_INDEX_Z], 0.0, TOL);

    // Start point should be preserved
    IVP_ASSERT_NEAR(line.start_point.k[IVP_INDEX_X], 0.0, TOL);
    IVP_ASSERT_NEAR(line.start_point.k[IVP_INDEX_Y], 0.0, TOL);
    IVP_ASSERT_NEAR(line.start_point.k[IVP_INDEX_Z], 0.0, TOL);
}

IVP_TEST(straight_qdist_to_point) {
    // Line along X axis
    ivp_straight_t line;
    ivp_vec3r_t start = {{0.0, 0.0, 0.0}};
    ivp_vec3r_t direction = {{1.0, 0.0, 0.0}};
    ivp_straight_init(&line, &start, &direction);

    // Point at (0, 5, 0) - distance should be 5, squared distance = 25
    ivp_vec3r_t point = {{0.0, 5.0, 0.0}};
    ivp_real_t qdist = ivp_straight_qdist_to_point(&line, &point);

    IVP_ASSERT_NEAR(qdist, 25.0, TOL);
}

IVP_TEST(hesse_intersect_straight) {
    // XZ plane at Y=0
    ivp_hesse_t plane;
    plane.normal.k[IVP_INDEX_X] = 0.0;
    plane.normal.k[IVP_INDEX_Y] = 1.0;
    plane.normal.k[IVP_INDEX_Z] = 0.0;
    plane.hesse_val = 0.0;

    // Line from (0, 5, 0) pointing down along Y axis
    ivp_straight_t line;
    ivp_vec3r_t start = {{0.0, 5.0, 0.0}};
    ivp_vec3r_t direction = {{0.0, -1.0, 0.0}};
    ivp_straight_init(&line, &start, &direction);

    ivp_vec3r_t intersection;
    ivp_result_t result = ivp_hesse_intersect_straight(&plane, &line, &intersection);

    IVP_ASSERT_TRUE(result == IVP_OK);

    // Intersection should be at (0, 0, 0)
    IVP_ASSERT_NEAR(intersection.k[IVP_INDEX_X], 0.0, TOL);
    IVP_ASSERT_NEAR(intersection.k[IVP_INDEX_Y], 0.0, TOL);
    IVP_ASSERT_NEAR(intersection.k[IVP_INDEX_Z], 0.0, TOL);
}

IVP_TEST(hesse_intersect_parallel) {
    // XZ plane at Y=0
    ivp_hesse_t plane;
    plane.normal.k[IVP_INDEX_X] = 0.0;
    plane.normal.k[IVP_INDEX_Y] = 1.0;
    plane.normal.k[IVP_INDEX_Z] = 0.0;
    plane.hesse_val = 0.0;

    // Line parallel to plane (along X axis, not touching plane)
    ivp_straight_t line;
    ivp_vec3r_t start = {{0.0, 5.0, 0.0}};
    ivp_vec3r_t direction = {{1.0, 0.0, 0.0}};
    ivp_straight_init(&line, &start, &direction);

    ivp_vec3r_t intersection;
    ivp_result_t result = ivp_hesse_intersect_straight(&plane, &line, &intersection);

    // Should return error for parallel/no intersection
    IVP_ASSERT_TRUE(result == IVP_ERR_DEGENERATE_GEOMETRY);
}

IVP_TEST(straight_intersect) {
    // Line along X axis
    ivp_straight_t line1;
    ivp_vec3r_t start1 = {{0.0, 0.0, 0.0}};
    ivp_vec3r_t dir1 = {{1.0, 0.0, 0.0}};
    ivp_straight_init(&line1, &start1, &dir1);

    // Line along Y axis
    ivp_straight_t line2;
    ivp_vec3r_t start2 = {{0.0, 0.0, 0.0}};
    ivp_vec3r_t dir2 = {{0.0, 1.0, 0.0}};
    ivp_straight_init(&line2, &start2, &dir2);

    ivp_vec3r_t intersection;
    ivp_real_t qdist;
    ivp_intersect_type_t result = ivp_straight_intersect(&line1, &line2, &intersection, &qdist);

    IVP_ASSERT_TRUE(result == IVP_INTERSECT_OK);

    // Intersection should be near origin
    IVP_ASSERT_NEAR(intersection.k[IVP_INDEX_X], 0.0, TOL);
    IVP_ASSERT_NEAR(intersection.k[IVP_INDEX_Y], 0.0, TOL);
    IVP_ASSERT_NEAR(intersection.k[IVP_INDEX_Z], 0.0, TOL);

    // Squared distance should be ~0 (perfect intersection)
    IVP_ASSERT_NEAR(qdist, 0.0, TOL);
}

// ============================================================================
// MAPPING TESTS
// ============================================================================

IVP_TEST(mapping_identity) {
    ivp_mapping_t map = ivp_mapping_identity();

    ivp_vec3r_t input = {{1.0, 2.0, 3.0}};
    ivp_vec3r_t output;

    ivp_mapping_vapply(&map, &input, &output);

    // Identity mapping should preserve the vector
    IVP_ASSERT_NEAR(output.k[IVP_INDEX_X], 1.0, TOL);
    IVP_ASSERT_NEAR(output.k[IVP_INDEX_Y], 2.0, TOL);
    IVP_ASSERT_NEAR(output.k[IVP_INDEX_Z], 3.0, TOL);
}

IVP_TEST(mapping_vapply_permute) {
    // Mapping k={1, 2, 0} means: out[0]=in[1], out[1]=in[2], out[2]=in[0]
    // XYZ -> YZX
    ivp_mapping_t map;
    map.k[0] = 1;
    map.k[1] = 2;
    map.k[2] = 0;

    ivp_vec3r_t input = {{1.0, 2.0, 3.0}};
    ivp_vec3r_t output;

    ivp_mapping_vapply(&map, &input, &output);

    // Should get (2, 3, 1)
    IVP_ASSERT_NEAR(output.k[0], 2.0, TOL);
    IVP_ASSERT_NEAR(output.k[1], 3.0, TOL);
    IVP_ASSERT_NEAR(output.k[2], 1.0, TOL);
}

IVP_TEST(mapping_viapply_inverse) {
    // vapply followed by viapply should give identity
    ivp_mapping_t map;
    map.k[0] = 1;
    map.k[1] = 2;
    map.k[2] = 0;

    ivp_vec3r_t input = {{1.0, 2.0, 3.0}};
    ivp_vec3r_t temp;
    ivp_vec3r_t output;

    ivp_mapping_vapply(&map, &input, &temp);
    ivp_mapping_viapply(&map, &temp, &output);

    // Should recover original vector
    IVP_ASSERT_NEAR(output.k[IVP_INDEX_X], 1.0, TOL);
    IVP_ASSERT_NEAR(output.k[IVP_INDEX_Y], 2.0, TOL);
    IVP_ASSERT_NEAR(output.k[IVP_INDEX_Z], 3.0, TOL);
}

IVP_TEST(mapping_type_handedness) {
    // Identity mapping should be right-handed
    ivp_mapping_t identity = ivp_mapping_identity();
    IVP_ASSERT_TRUE(ivp_mapping_type(&identity) == IVP_MC_RIGHT_HANDED);

    // Swap X and Y: k={1, 0, 2} should be left-handed
    ivp_mapping_t swap_xy;
    swap_xy.k[0] = 1;
    swap_xy.k[1] = 0;
    swap_xy.k[2] = 2;
    IVP_ASSERT_TRUE(ivp_mapping_type(&swap_xy) == IVP_MC_LEFT_HANDED);
}

IVP_TEST(mapping_mapply_inplace_matches_outofplace) {
    ivp_mapping_t map;
    map.k[0] = 1;
    map.k[1] = 2;
    map.k[2] = 0;

    ivp_mat3r_t input = {{{{1.0, 2.0, 3.0}},
                          {{4.0, 5.0, 6.0}},
                          {{7.0, 8.0, 9.0}}}};

    ivp_mat3r_t expected;
    ivp_mapping_mapply(&map, &input, &expected);

    ivp_mat3r_t inplace = input;
    ivp_mapping_mapply(&map, &inplace, &inplace);

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            IVP_ASSERT_NEAR(inplace.rows[r].k[c], expected.rows[r].k[c], TOL);
        }
    }
}

IVP_TEST(mapping_miapply_inplace_matches_outofplace) {
    ivp_mapping_t map;
    map.k[0] = 1;
    map.k[1] = 2;
    map.k[2] = 0;

    ivp_mat3r_t input = {{{{1.0, 2.0, 3.0}},
                          {{4.0, 5.0, 6.0}},
                          {{7.0, 8.0, 9.0}}}};

    ivp_mat3r_t expected;
    ivp_mapping_miapply(&map, &input, &expected);

    ivp_mat3r_t inplace = input;
    ivp_mapping_miapply(&map, &inplace, &inplace);

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            IVP_ASSERT_NEAR(inplace.rows[r].k[c], expected.rows[r].k[c], TOL);
        }
    }
}

IVP_TEST(mapping_mi2apply_inplace_matches_outofplace) {
    ivp_mapping_t map;
    map.k[0] = 1;
    map.k[1] = 2;
    map.k[2] = 0;

    ivp_mat3r_t input = {{{{1.0, 2.0, 3.0}},
                          {{4.0, 5.0, 6.0}},
                          {{7.0, 8.0, 9.0}}}};

    ivp_mat3r_t expected;
    ivp_mapping_mi2apply(&map, &input, &expected);

    ivp_mat3r_t inplace = input;
    ivp_mapping_mi2apply(&map, &inplace, &inplace);

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            IVP_ASSERT_NEAR(inplace.rows[r].k[c], expected.rows[r].k[c], TOL);
        }
    }
}

// ============================================================================
// MAIN
// ============================================================================

int main(void) {
    // Register geometry tests
    IVP_REGISTER(hesse_calc_from_points);
    IVP_REGISTER(hesse_dist_positive_side);
    IVP_REGISTER(hesse_dist_negative_side);
    IVP_REGISTER(hesse_project_onto_plane);
    IVP_REGISTER(hesse_is_parallel);
    IVP_REGISTER(straight_init);
    IVP_REGISTER(straight_qdist_to_point);
    IVP_REGISTER(hesse_intersect_straight);
    IVP_REGISTER(hesse_intersect_parallel);
    IVP_REGISTER(straight_intersect);

    // Register mapping tests
    IVP_REGISTER(mapping_identity);
    IVP_REGISTER(mapping_vapply_permute);
    IVP_REGISTER(mapping_viapply_inverse);
    IVP_REGISTER(mapping_type_handedness);
    IVP_REGISTER(mapping_mapply_inplace_matches_outofplace);
    IVP_REGISTER(mapping_miapply_inplace_matches_outofplace);
    IVP_REGISTER(mapping_mi2apply_inplace_matches_outofplace);

    IVP_RUN_ALL();
}
