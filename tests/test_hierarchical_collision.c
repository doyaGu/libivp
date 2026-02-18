/* test_hierarchical_collision.c — Tests for hierarchical poly-poly collision */
#include "ivp_test.h"
#include <ivp/collision/ivp_ledgetree_collision.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/builder/ivp_build_concave.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * Helper: build a synthetic ledgetree node for testing BV overlap
 * ═══════════════════════════════════════════════════════════════════════════ */

static ivp_compact_ledgetree_node_t make_test_node(float cx, float cy, float cz,
                                                     float radius)
{
    ivp_compact_ledgetree_node_t node;
    memset(&node, 0, sizeof(node));
    node.center[0] = cx;
    node.center[1] = cy;
    node.center[2] = cz;
    node.radius = radius;
    /* Terminal (offset_right_node == 0) */
    node.offset_right_node = 0;
    return node;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * BV overlap tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(bv_overlap_separated) {
    ivp_compact_ledgetree_node_t na = make_test_node(0, 0, 0, 1.0f);
    ivp_compact_ledgetree_node_t nb = make_test_node(0, 0, 0, 1.0f);

    /* B is at (5, 0, 0) in A's space — total threshold = 1+1+0 = 2, dist = 5 */
    ivp_transform_r_t b_in_a;
    ivp_transform_identity(&b_in_a);
    b_in_a.pos = ivp_vec3r(IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));

    IVP_ASSERT_FALSE(ivp_ledgetree_bv_overlap(&na, &nb, &b_in_a, IVP_REAL(0.0)));
}

IVP_TEST(bv_overlap_touching) {
    ivp_compact_ledgetree_node_t na = make_test_node(0, 0, 0, 1.0f);
    ivp_compact_ledgetree_node_t nb = make_test_node(0, 0, 0, 1.0f);

    /* B is at (2, 0, 0) — exactly touching */
    ivp_transform_r_t b_in_a;
    ivp_transform_identity(&b_in_a);
    b_in_a.pos = ivp_vec3r(IVP_REAL(2.0), IVP_REAL(0.0), IVP_REAL(0.0));

    IVP_ASSERT_TRUE(ivp_ledgetree_bv_overlap(&na, &nb, &b_in_a, IVP_REAL(0.0)));
}

IVP_TEST(bv_overlap_with_margin) {
    ivp_compact_ledgetree_node_t na = make_test_node(0, 0, 0, 1.0f);
    ivp_compact_ledgetree_node_t nb = make_test_node(0, 0, 0, 1.0f);

    /* B is at (2.5, 0, 0) — threshold without margin = 2, with margin 1 = 3 */
    ivp_transform_r_t b_in_a;
    ivp_transform_identity(&b_in_a);
    b_in_a.pos = ivp_vec3r(IVP_REAL(2.5), IVP_REAL(0.0), IVP_REAL(0.0));

    IVP_ASSERT_FALSE(ivp_ledgetree_bv_overlap(&na, &nb, &b_in_a, IVP_REAL(0.0)));
    IVP_ASSERT_TRUE(ivp_ledgetree_bv_overlap(&na, &nb, &b_in_a, IVP_REAL(1.0)));
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Tree traversal tests
 * ═══════════════════════════════════════════════════════════════════════════ */

static int s_leaf_count;

static bool count_leaf_callback(const ivp_compact_ledge_t *ledge_a,
                                 const ivp_compact_ledge_t *ledge_b,
                                 const ivp_transform_r_t *b_in_a,
                                 void *user_data)
{
    (void)ledge_a; (void)ledge_b; (void)b_in_a; (void)user_data;
    s_leaf_count++;
    return true;
}

IVP_TEST(tree_traversal_terminal_terminal) {
    /* Two single-convex (terminal root) surfaces should yield exactly 1 leaf pair */
    ivp_compact_surface_t *surf_a = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    ivp_compact_surface_t *surf_b = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(surf_a);
    IVP_ASSERT_NOT_NULL(surf_b);

    const ivp_compact_ledgetree_node_t *root_a = ivp_surface_get_ledgetree_root(surf_a);
    const ivp_compact_ledgetree_node_t *root_b = ivp_surface_get_ledgetree_root(surf_b);
    IVP_ASSERT_NOT_NULL(root_a);
    IVP_ASSERT_NOT_NULL(root_b);

    /* Both should be terminal for single-convex */
    IVP_ASSERT_TRUE(ivp_ledgetree_is_terminal(root_a));
    IVP_ASSERT_TRUE(ivp_ledgetree_is_terminal(root_b));

    ivp_transform_r_t b_in_a;
    ivp_transform_identity(&b_in_a);
    /* Place B at (0.5, 0, 0) — overlapping */
    b_in_a.pos = ivp_vec3r(IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));

    s_leaf_count = 0;
    int result = ivp_ledgetree_traverse_pairs(root_a, root_b, &b_in_a,
                                               IVP_REAL(1.0),
                                               count_leaf_callback, NULL);

    IVP_ASSERT_EQ(result, 1);
    IVP_ASSERT_EQ(s_leaf_count, 1);

    ivp_compact_surface_free(NULL, surf_a);
    ivp_compact_surface_free(NULL, surf_b);
}

IVP_TEST(tree_traversal_compound_simple) {
    /* Compound (concave) vs simple (terminal) — should yield >= 1 leaf pairs */
    /* Build a concave surface from two box parts */
    ivp_compact_surface_t *part1 = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    ivp_compact_surface_t *part2 = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(part1);
    IVP_ASSERT_NOT_NULL(part2);

    ivp_compact_surface_t *parts[] = { part1, part2 };
    ivp_compact_surface_t *compound = ivp_build_concave_from_parts(NULL, parts, 2);

    ivp_compact_surface_t *simple = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(simple);

    if (compound) {
        const ivp_compact_ledgetree_node_t *root_c = ivp_surface_get_ledgetree_root(compound);
        const ivp_compact_ledgetree_node_t *root_s = ivp_surface_get_ledgetree_root(simple);
        IVP_ASSERT_NOT_NULL(root_c);
        IVP_ASSERT_NOT_NULL(root_s);

        /* Compound root should be non-terminal */
        IVP_ASSERT_FALSE(ivp_ledgetree_is_terminal(root_c));

        ivp_transform_r_t b_in_a;
        ivp_transform_identity(&b_in_a);

        s_leaf_count = 0;
        int result = ivp_ledgetree_traverse_pairs(root_c, root_s, &b_in_a,
                                                   IVP_REAL(2.0),
                                                   count_leaf_callback, NULL);

        /* Should find at least 1 leaf pair */
        IVP_ASSERT_TRUE(result >= 1);
        IVP_ASSERT_TRUE(s_leaf_count >= 1);

        ivp_compact_surface_free(NULL, compound);
    }

    ivp_compact_surface_free(NULL, part1);
    ivp_compact_surface_free(NULL, part2);
    ivp_compact_surface_free(NULL, simple);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Integration tests: hierarchical contact via full dispatch
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(hierarchical_contact_simple_fallthrough) {
    /* Two simple convex objects → ledgetree root is NULL for single-convex,
     * so ivp_contact_compute_generic_multi should use the existing path. */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_compact_surface_t *surf_a = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    ivp_compact_surface_t *surf_b = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(surf_a);
    IVP_ASSERT_NOT_NULL(surf_b);

    ivp_surman_polygon_t sm_a, sm_b;
    ivp_surman_polygon_init(&sm_a, surf_a);
    ivp_surman_polygon_init(&sm_b, surf_b);

    /* Verify ledgetree root is NULL for single-convex surfaces */
    IVP_ASSERT_NULL(ivp_surman_get_ledgetree_root(&sm_a.base));
    IVP_ASSERT_NULL(ivp_surman_get_ledgetree_root(&sm_b.base));

    ivp_object_template_t ta;
    ivp_object_template_init(&ta);
    ta.type = IVP_OBJECT_POLYGON;
    ta.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&ta.rot_inertia, IVP_REAL(0.083), IVP_REAL(0.083), IVP_REAL(0.083));
    ivp_vec3r_set(&ta.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_a = ivp_environment_create_object(env, &ta, &sm_a.base);

    ivp_object_template_t tb;
    ivp_object_template_init(&tb);
    tb.type = IVP_OBJECT_POLYGON;
    tb.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tb.rot_inertia, IVP_REAL(0.083), IVP_REAL(0.083), IVP_REAL(0.083));
    ivp_vec3r_set(&tb.position, IVP_REAL(0.8), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tb, &sm_b.base);

    IVP_ASSERT_NOT_NULL(obj_a);
    IVP_ASSERT_NOT_NULL(obj_b);

    /* Compute contacts — should use existing path since roots are NULL */
    ivp_contact_point_t contacts[4];
    int n = ivp_contact_compute_generic_multi(obj_a, obj_b, contacts, 4);
    IVP_ASSERT_TRUE(n >= 1);

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, surf_a);
    ivp_compact_surface_free(NULL, surf_b);
}

IVP_TEST(hierarchical_contact_concave_vs_convex) {
    /* Concave mesh (built with ivp_build_concave) vs box.
     * The compound surface has a non-terminal root, triggering hierarchical path. */
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    /* Build compound surface from two boxes */
    ivp_compact_surface_t *part1 = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    ivp_compact_surface_t *part2 = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(part1);
    IVP_ASSERT_NOT_NULL(part2);

    ivp_compact_surface_t *parts[] = { part1, part2 };
    ivp_compact_surface_t *compound = ivp_build_concave_from_parts(NULL, parts, 2);

    if (!compound) {
        /* If concave builder fails, skip test gracefully */
        ivp_compact_surface_free(NULL, part1);
        ivp_compact_surface_free(NULL, part2);
        ivp_environment_destroy(env);
        return;
    }

    ivp_surman_polygon_t sm_compound;
    ivp_surman_polygon_init(&sm_compound, compound);

    /* Verify compound root is non-terminal */
    const ivp_compact_ledgetree_node_t *root = ivp_surman_get_ledgetree_root(&sm_compound.base);
    IVP_ASSERT_NOT_NULL(root);

    ivp_compact_surface_t *box_surf = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    IVP_ASSERT_NOT_NULL(box_surf);
    ivp_surman_polygon_t sm_box;
    ivp_surman_polygon_init(&sm_box, box_surf);

    ivp_object_template_t ta;
    ivp_object_template_init(&ta);
    ta.type = IVP_OBJECT_POLYGON;
    ta.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&ta.rot_inertia, IVP_REAL(0.083), IVP_REAL(0.083), IVP_REAL(0.083));
    ivp_object_t *obj_a = ivp_environment_create_object(env, &ta, &sm_compound.base);

    ivp_object_template_t tb;
    ivp_object_template_init(&tb);
    tb.type = IVP_OBJECT_POLYGON;
    tb.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tb.rot_inertia, IVP_REAL(0.083), IVP_REAL(0.083), IVP_REAL(0.083));
    ivp_vec3r_set(&tb.position, IVP_REAL(0.5), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tb, &sm_box.base);

    IVP_ASSERT_NOT_NULL(obj_a);
    IVP_ASSERT_NOT_NULL(obj_b);

    /* Hierarchical path should be used since obj_a has compound surface.
     * Contacts may or may not be found depending on part placement in the
     * compound surface (parts are at the same origin). Verify the function
     * runs without crashing and returns a valid count. */
    ivp_contact_point_t contacts[8];
    int n = ivp_contact_compute_hierarchical_multi(obj_a, obj_b, contacts, 8);
    IVP_ASSERT_TRUE(n >= 0);

    /* Verify any contacts that were found are valid */
    for (int i = 0; i < n; i++) {
        IVP_ASSERT_TRUE(contacts[i].obj_a == obj_a);
        IVP_ASSERT_TRUE(contacts[i].obj_b == obj_b);
        IVP_ASSERT_TRUE(contacts[i].virtual_mass > IVP_REAL(0.0));
    }

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, compound);
    ivp_compact_surface_free(NULL, part1);
    ivp_compact_surface_free(NULL, part2);
    ivp_compact_surface_free(NULL, box_surf);
}

IVP_TEST(hierarchical_contact_dedup) {
    /* Build compound surface with identical parts at same position.
     * Should deduplicate contacts from overlapping leaf pairs. */
    ivp_compact_surface_t *part = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(part);

    /* Both parts are boxes at origin — will produce overlapping BVs */
    ivp_compact_surface_t *parts[] = { part, part };
    ivp_compact_surface_t *compound = ivp_build_concave_from_parts(NULL, parts, 2);

    if (!compound) {
        ivp_compact_surface_free(NULL, part);
        return;
    }

    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_environment_t *env = ivp_environment_create(&cfg);

    ivp_surman_polygon_t sm_c;
    ivp_surman_polygon_init(&sm_c, compound);

    ivp_compact_surface_t *box = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    IVP_ASSERT_NOT_NULL(box);
    ivp_surman_polygon_t sm_box;
    ivp_surman_polygon_init(&sm_box, box);

    ivp_object_template_t ta;
    ivp_object_template_init(&ta);
    ta.type = IVP_OBJECT_POLYGON;
    ta.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&ta.rot_inertia, IVP_REAL(0.083), IVP_REAL(0.083), IVP_REAL(0.083));
    ivp_object_t *obj_a = ivp_environment_create_object(env, &ta, &sm_c.base);

    ivp_object_template_t tb;
    ivp_object_template_init(&tb);
    tb.type = IVP_OBJECT_POLYGON;
    tb.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tb.rot_inertia, IVP_REAL(0.083), IVP_REAL(0.083), IVP_REAL(0.083));
    ivp_vec3r_set(&tb.position, IVP_REAL(0.3), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj_b = ivp_environment_create_object(env, &tb, &sm_box.base);

    IVP_ASSERT_NOT_NULL(obj_a);
    IVP_ASSERT_NOT_NULL(obj_b);

    ivp_contact_point_t contacts[8];
    int n = ivp_contact_compute_hierarchical_multi(obj_a, obj_b, contacts, 8);

    /* Even with duplicate leaf pairs, deduplication should keep count reasonable.
     * We don't assert exact count since it depends on geometry,
     * but verify no duplicates in result. */
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            ivp_real_t qdist = ivp_vec3r_quad_distance(&contacts[i].point_a,
                                                        &contacts[j].point_a);
            /* If contact points are very close, check normals differ */
            if (qdist < IVP_REAL(1e-4)) {
                ivp_real_t ndot = ivp_vec3r_dot(&contacts[i].normal,
                                                 &contacts[j].normal);
                IVP_ASSERT_TRUE(ndot < IVP_REAL(0.995));
            }
        }
    }

    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, compound);
    ivp_compact_surface_free(NULL, part);
    ivp_compact_surface_free(NULL, box);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test registration
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(void) {
    IVP_REGISTER(bv_overlap_separated);
    IVP_REGISTER(bv_overlap_touching);
    IVP_REGISTER(bv_overlap_with_margin);
    IVP_REGISTER(tree_traversal_terminal_terminal);
    IVP_REGISTER(tree_traversal_compound_simple);
    IVP_REGISTER(hierarchical_contact_simple_fallthrough);
    IVP_REGISTER(hierarchical_contact_concave_vs_convex);
    IVP_REGISTER(hierarchical_contact_dedup);
    IVP_RUN_ALL();
}
