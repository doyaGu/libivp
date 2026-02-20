#include "ivp_test.h"
#include <ivp/utility/ivp_active_value.h>
#include <ivp/utility/ivp_union_find.h>

IVP_TEST(active_switch_remove_ref_unregisters_all_parents) {
    ivp_active_int_t *cond = ivp_active_int_terminal_create(NULL, "cond", 0);
    ivp_active_float_t *f_false = ivp_active_float_terminal_create(NULL, "f", 1.0);
    ivp_active_float_t *f_true = ivp_active_float_terminal_create(NULL, "t", 2.0);
    ivp_active_float_t *sw = ivp_active_float_switch_create(NULL, "sw", cond, f_false, f_true);

    IVP_ASSERT_NOT_NULL(cond);
    IVP_ASSERT_NOT_NULL(f_false);
    IVP_ASSERT_NOT_NULL(f_true);
    IVP_ASSERT_NOT_NULL(sw);

    IVP_ASSERT_EQ(cond->dependent_count, 1);
    IVP_ASSERT_EQ(f_false->dependent_count, 1);
    IVP_ASSERT_EQ(f_true->dependent_count, 1);

    ivp_active_float_remove_ref(sw);

    IVP_ASSERT_EQ(cond->dependent_count, 0);
    IVP_ASSERT_EQ(f_false->dependent_count, 0);
    IVP_ASSERT_EQ(f_true->dependent_count, 0);
    IVP_ASSERT_EQ(cond->ref_count, 1);
    IVP_ASSERT_EQ(f_false->ref_count, 1);
    IVP_ASSERT_EQ(f_true->ref_count, 1);

    ivp_active_int_remove_ref(cond);
    ivp_active_float_remove_ref(f_false);
    ivp_active_float_remove_ref(f_true);
}

IVP_TEST(active_switch_same_branch_node_unregisters_twice) {
    ivp_active_int_t *cond = ivp_active_int_terminal_create(NULL, "cond", 0);
    ivp_active_float_t *f = ivp_active_float_terminal_create(NULL, "v", 3.0);
    ivp_active_float_t *sw = ivp_active_float_switch_create(NULL, "sw", cond, f, f);

    IVP_ASSERT_NOT_NULL(cond);
    IVP_ASSERT_NOT_NULL(f);
    IVP_ASSERT_NOT_NULL(sw);
    IVP_ASSERT_EQ(cond->dependent_count, 1);
    IVP_ASSERT_EQ(f->dependent_count, 2);

    ivp_active_float_remove_ref(sw);

    IVP_ASSERT_EQ(cond->dependent_count, 0);
    IVP_ASSERT_EQ(f->dependent_count, 0);
    IVP_ASSERT_EQ(cond->ref_count, 1);
    IVP_ASSERT_EQ(f->ref_count, 1);

    ivp_active_int_remove_ref(cond);
    ivp_active_float_remove_ref(f);
}

IVP_TEST(union_find_union_and_find_connect_components) {
    int parent[6];
    ivp_union_find_init(parent, 6);

    ivp_union_find_union(parent, 0, 1);
    ivp_union_find_union(parent, 1, 2);
    ivp_union_find_union(parent, 3, 4);

    IVP_ASSERT_EQ(ivp_union_find_find(parent, 0), ivp_union_find_find(parent, 2));
    IVP_ASSERT_EQ(ivp_union_find_find(parent, 3), ivp_union_find_find(parent, 4));
    IVP_ASSERT_TRUE(ivp_union_find_find(parent, 0) != ivp_union_find_find(parent, 5));
}

IVP_TEST(union_find_union_by_rank_connects_sets) {
    int parent[8];
    int rank[8];
    ivp_union_find_init_with_rank(parent, rank, 8);

    ivp_union_find_union_by_rank(parent, rank, 0, 1);
    ivp_union_find_union_by_rank(parent, rank, 2, 3);
    ivp_union_find_union_by_rank(parent, rank, 1, 3);

    IVP_ASSERT_EQ(ivp_union_find_find(parent, 0), ivp_union_find_find(parent, 2));
    IVP_ASSERT_EQ(ivp_union_find_find(parent, 1), ivp_union_find_find(parent, 3));
    IVP_ASSERT_TRUE(ivp_union_find_find(parent, 0) != ivp_union_find_find(parent, 7));
}

IVP_TEST(union_find_union_min_root_keeps_lowest_root) {
    int parent[5];
    ivp_union_find_init(parent, 5);

    ivp_union_find_union_min_root(parent, 4, 2);
    ivp_union_find_union_min_root(parent, 2, 3);

    int root2 = ivp_union_find_find(parent, 2);
    int root3 = ivp_union_find_find(parent, 3);
    int root4 = ivp_union_find_find(parent, 4);

    IVP_ASSERT_EQ(root2, root3);
    IVP_ASSERT_EQ(root2, root4);
    IVP_ASSERT_EQ(root2, 2);
}

int main(void) {
    IVP_REGISTER(active_switch_remove_ref_unregisters_all_parents);
    IVP_REGISTER(active_switch_same_branch_node_unregisters_twice);
    IVP_REGISTER(union_find_union_and_find_connect_components);
    IVP_REGISTER(union_find_union_by_rank_connects_sets);
    IVP_REGISTER(union_find_union_min_root_keeps_lowest_root);
    IVP_RUN_ALL();
}
