#include "ivp_test.h"
#include <ivp/utility/ivp_active_value.h>

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

int main(void) {
    IVP_REGISTER(active_switch_remove_ref_unregisters_all_parents);
    IVP_REGISTER(active_switch_same_branch_node_unregisters_twice);
    IVP_RUN_ALL();
}
