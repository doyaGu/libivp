/* test_matrix_cache.c — IVP_U_Matrix_Cache index behavior
 *
 * Reference 8a9c33e (ivp_3d_solver.hxx): calc_matrix_at clamps the index to
 * [0, IVP_3D_SOLVER_MAX_STEPS_PER_PSI] and checks time and index only for a
 * request that was in range.  Out-of-range requests use the boundary slot
 * and report no assertion failure.
 */
#include "ivp_test.h"

#include "collision/ivp_matrix_cache.h"

#include <string.h>

static int g_mc_assert_count;

static void mc_count_assert(const char *expr, const char *file, int line,
                            const char *func, const char *msg, void *userdata)
{
    (void)expr; (void)file; (void)line; (void)func; (void)msg; (void)userdata;
    g_mc_assert_count++;
}

/* every slot preset, so no matrix is calculated */
static void mc_preset(matrix_cache_t *mc, ivp_transform_r_t *slots)
{
    memset(mc, 0, sizeof(*mc));
    mc->base_time = 1.0;
    for (int i = 0; i <= IVP_3D_SOLVER_MAX_STEPS_PER_PSI; i++) mc->m_world_f_object[i] = &slots[i];
}

IVP_TEST(matrix_cache_in_range_index)
{
    matrix_cache_t mc;
    ivp_transform_r_t slots[IVP_3D_SOLVER_MAX_STEPS_PER_PSI + 1];
    mc_preset(&mc, slots);
    g_mc_assert_count = 0;
    ivp_set_assert_handler(mc_count_assert, NULL);
    ivp_transform_r_t *m = mc_calc_matrix_at(&mc, 1.0 + 3.0 / IVP_3D_SOLVER_STEPS_PER_SECOND, 3);
    ivp_set_assert_handler(NULL, NULL);
    IVP_ASSERT_TRUE(m == &slots[3]);
    IVP_ASSERT_EQ(g_mc_assert_count, 0);
}

IVP_TEST(matrix_cache_negative_index_uses_first_slot)
{
    matrix_cache_t mc;
    ivp_transform_r_t slots[IVP_3D_SOLVER_MAX_STEPS_PER_PSI + 1];
    mc_preset(&mc, slots);
    g_mc_assert_count = 0;
    ivp_set_assert_handler(mc_count_assert, NULL);
    ivp_transform_r_t *m = mc_calc_matrix_at(&mc, 1.0 - 2.0 / IVP_3D_SOLVER_STEPS_PER_SECOND, -2);
    ivp_set_assert_handler(NULL, NULL);
    IVP_ASSERT_TRUE(m == &slots[0]);
    IVP_ASSERT_EQ(g_mc_assert_count, 0);
}

IVP_TEST(matrix_cache_large_index_uses_last_slot)
{
    matrix_cache_t mc;
    ivp_transform_r_t slots[IVP_3D_SOLVER_MAX_STEPS_PER_PSI + 1];
    mc_preset(&mc, slots);
    const int index = IVP_3D_SOLVER_MAX_STEPS_PER_PSI + 5;
    g_mc_assert_count = 0;
    ivp_set_assert_handler(mc_count_assert, NULL);
    ivp_transform_r_t *m = mc_calc_matrix_at(&mc, 1.0 + (double)index / IVP_3D_SOLVER_STEPS_PER_SECOND, index);
    ivp_set_assert_handler(NULL, NULL);
    IVP_ASSERT_TRUE(m == &slots[IVP_3D_SOLVER_MAX_STEPS_PER_PSI]);
    IVP_ASSERT_EQ(g_mc_assert_count, 0);
}

int main(void)
{
    IVP_REGISTER(matrix_cache_in_range_index);
    IVP_REGISTER(matrix_cache_negative_index_uses_first_slot);
    IVP_REGISTER(matrix_cache_large_index_uses_last_slot);
    IVP_RUN_ALL();
}
