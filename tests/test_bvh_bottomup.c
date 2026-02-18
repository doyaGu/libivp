/* test_bvh_bottomup.c — Tests for bottom-up BVH builder */
#include "ivp_test.h"
#include <ivp/builder/ivp_build_bvh_bottomup.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/builder/ivp_build_polygon.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/base/ivp_allocator.h>
#include <string.h>

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

IVP_TEST(bottomup_config_defaults) {
    ivp_bvh_bottomup_config_t cfg;
    ivp_bvh_bottomup_config_init(&cfg);
    IVP_ASSERT_NEAR(cfg.radius_tolerance, 1.1, 1e-6);
    IVP_ASSERT_NULL(cfg.alloc);
}

IVP_TEST(bottomup_single_ledge) {
    /* Degenerate case: single ledge should produce a valid surface */
    ivp_compact_surface_t *box = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(box);

    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(box);
    IVP_ASSERT_NOT_NULL(ledge);

    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);
    IVP_ASSERT_EQ(ivp_ledge_soup_add(&soup, ledge), 0);

    ivp_bvh_bottomup_config_t cfg;
    ivp_bvh_bottomup_config_init(&cfg);

    ivp_compact_surface_t *result = ivp_ledge_soup_compile_bottomup(&soup, &cfg);
    IVP_ASSERT_NOT_NULL(result);

    /* Result should have a valid ledgetree root */
    IVP_ASSERT_TRUE(result->offset_ledgetree_root != 0);

    ivp_compact_surface_free(NULL, result);
    ivp_ledge_soup_destroy(&soup);
    ivp_compact_surface_free(NULL, box);
}

IVP_TEST(bottomup_two_ledges) {
    /* Two ledges should merge into a single tree */
    ivp_compact_surface_t *box1 = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    ivp_compact_surface_t *box2 = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    IVP_ASSERT_NOT_NULL(box1);
    IVP_ASSERT_NOT_NULL(box2);

    const ivp_compact_ledge_t *ledge1 = ivp_surface_get_single_convex(box1);
    const ivp_compact_ledge_t *ledge2 = ivp_surface_get_single_convex(box2);
    IVP_ASSERT_NOT_NULL(ledge1);
    IVP_ASSERT_NOT_NULL(ledge2);

    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);
    IVP_ASSERT_EQ(ivp_ledge_soup_add(&soup, ledge1), 0);
    IVP_ASSERT_EQ(ivp_ledge_soup_add(&soup, ledge2), 0);

    ivp_bvh_bottomup_config_t cfg;
    ivp_bvh_bottomup_config_init(&cfg);

    ivp_compact_surface_t *result = ivp_ledge_soup_compile_bottomup(&soup, &cfg);
    IVP_ASSERT_NOT_NULL(result);

    /* Verify the surface has valid size */
    int byte_size = ivp_surface_get_byte_size(result);
    IVP_ASSERT_TRUE(byte_size > 0);

    ivp_compact_surface_free(NULL, result);
    ivp_ledge_soup_destroy(&soup);
    ivp_compact_surface_free(NULL, box2);
    ivp_compact_surface_free(NULL, box1);
}

IVP_TEST(bottomup_null_soup) {
    /* NULL soup should return NULL */
    ivp_bvh_bottomup_config_t cfg;
    ivp_bvh_bottomup_config_init(&cfg);

    ivp_compact_surface_t *result = ivp_ledge_soup_compile_bottomup(NULL, &cfg);
    IVP_ASSERT_NULL(result);
}

IVP_TEST(bottomup_empty_soup) {
    /* Empty soup should return NULL */
    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);

    ivp_bvh_bottomup_config_t cfg;
    ivp_bvh_bottomup_config_init(&cfg);

    ivp_compact_surface_t *result = ivp_ledge_soup_compile_bottomup(&soup, &cfg);
    IVP_ASSERT_NULL(result);

    ivp_ledge_soup_destroy(&soup);
}

IVP_TEST(bottomup_vs_topdown_valid) {
    /* Both builders should produce valid surfaces from the same input */
    ivp_compact_surface_t *box1 = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    ivp_compact_surface_t *box2 = ivp_build_box_surface(NULL, 0.5f, 2.0f, 0.5f);
    IVP_ASSERT_NOT_NULL(box1);
    IVP_ASSERT_NOT_NULL(box2);

    const ivp_compact_ledge_t *ledge1 = ivp_surface_get_single_convex(box1);
    const ivp_compact_ledge_t *ledge2 = ivp_surface_get_single_convex(box2);

    /* Top-down */
    ivp_ledge_soup_t soup_td;
    ivp_ledge_soup_init(&soup_td, NULL);
    ivp_ledge_soup_add(&soup_td, ledge1);
    ivp_ledge_soup_add(&soup_td, ledge2);
    ivp_compact_surface_t *td_result = ivp_ledge_soup_compile(&soup_td);
    IVP_ASSERT_NOT_NULL(td_result);

    /* Bottom-up */
    ivp_ledge_soup_t soup_bu;
    ivp_ledge_soup_init(&soup_bu, NULL);
    ivp_ledge_soup_add(&soup_bu, ledge1);
    ivp_ledge_soup_add(&soup_bu, ledge2);
    ivp_bvh_bottomup_config_t cfg;
    ivp_bvh_bottomup_config_init(&cfg);
    ivp_compact_surface_t *bu_result = ivp_ledge_soup_compile_bottomup(&soup_bu, &cfg);
    IVP_ASSERT_NOT_NULL(bu_result);

    /* Both should have valid positive byte size */
    IVP_ASSERT_TRUE(ivp_surface_get_byte_size(td_result) > 0);
    IVP_ASSERT_TRUE(ivp_surface_get_byte_size(bu_result) > 0);

    ivp_compact_surface_free(NULL, bu_result);
    ivp_compact_surface_free(NULL, td_result);
    ivp_ledge_soup_destroy(&soup_bu);
    ivp_ledge_soup_destroy(&soup_td);
    ivp_compact_surface_free(NULL, box2);
    ivp_compact_surface_free(NULL, box1);
}

/* ── Registration ──────────────────────────────────────────────────────── */

IVP_TEST(bottomup_zero_tolerance_returns_null) {
    ivp_compact_surface_t *box1 = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    ivp_compact_surface_t *box2 = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    IVP_ASSERT_NOT_NULL(box1);
    IVP_ASSERT_NOT_NULL(box2);

    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);
    IVP_ASSERT_EQ(ivp_ledge_soup_add(&soup, ivp_surface_get_single_convex(box1)), 0);
    IVP_ASSERT_EQ(ivp_ledge_soup_add(&soup, ivp_surface_get_single_convex(box2)), 0);

    ivp_bvh_bottomup_config_t cfg;
    ivp_bvh_bottomup_config_init(&cfg);
    cfg.radius_tolerance = 0.0f;

    ivp_compact_surface_t *result = ivp_ledge_soup_compile_bottomup(&soup, &cfg);
    IVP_ASSERT_NULL(result);

    ivp_ledge_soup_destroy(&soup);
    ivp_compact_surface_free(NULL, box2);
    ivp_compact_surface_free(NULL, box1);
}

IVP_TEST(bottomup_single_ledge_off_origin_mass_center) {
    ivp_compact_surface_t *poly = ivp_build_polygon_surface(
        NULL, off_tet_verts, 4, off_tet_indices, 4
    );
    IVP_ASSERT_NOT_NULL(poly);
    IVP_ASSERT_TRUE(poly->mass_center[0] > 9.0f);

    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);
    IVP_ASSERT_EQ(ivp_ledge_soup_add(&soup, ivp_surface_get_single_convex(poly)), 0);

    ivp_bvh_bottomup_config_t cfg;
    ivp_bvh_bottomup_config_init(&cfg);
    ivp_compact_surface_t *result = ivp_ledge_soup_compile_bottomup(&soup, &cfg);
    IVP_ASSERT_NOT_NULL(result);
    IVP_ASSERT_NEAR(result->mass_center[0], poly->mass_center[0], 1e-4f);
    IVP_ASSERT_NEAR(result->mass_center[1], poly->mass_center[1], 1e-4f);
    IVP_ASSERT_NEAR(result->mass_center[2], poly->mass_center[2], 1e-4f);

    ivp_compact_surface_free(NULL, result);
    ivp_ledge_soup_destroy(&soup);
    ivp_compact_surface_free(NULL, poly);
}

int main(void) {
    IVP_REGISTER(bottomup_config_defaults);
    IVP_REGISTER(bottomup_single_ledge);
    IVP_REGISTER(bottomup_two_ledges);
    IVP_REGISTER(bottomup_null_soup);
    IVP_REGISTER(bottomup_empty_soup);
    IVP_REGISTER(bottomup_vs_topdown_valid);
    IVP_REGISTER(bottomup_zero_tolerance_returns_null);
    IVP_REGISTER(bottomup_single_ledge_off_origin_mass_center);
    IVP_RUN_ALL();
}
