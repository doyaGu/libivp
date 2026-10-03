/* ivp_statistic_manager.c — IVP_Statistic_Manager (ivp_physics/ivp_physic.cxx) */
#include <ivp/diagnostics/ivp_statistic_manager.h>
#include <ivp/base/ivp_message.h>

#include <stddef.h>

void ivp_statistic_manager_output(ivp_statistic_manager_t *sm)
{
    if (!sm) return;
    ivp_debug_printf("nr_impacts %d  hard_resc %d  resc_after %d  delayed %d  sys_strt %d sys_imps %d unmov %d  "
                     "coll_check %d  mindists %d  gen_mindists %d ov %d\n",
                     sm->impact_counter, sm->impact_hard_rescue_counter, sm->impact_rescue_after_counter,
                     sm->impact_delayed_counter, sm->impact_sys_num, sm->impact_sum_sys, sm->impact_unmov,
                     sm->impact_coll_checks, sm->sum_of_mindists, sm->mindists_generated, sm->range_world_exceeded);
    ivp_statistic_manager_clear(sm);
}

void ivp_statistic_manager_clear(ivp_statistic_manager_t *sm)
{
    if (!sm) return;
    sm->max_rescue_speed = 0.0f;
    sm->max_speed_gain = 0.0f;
    sm->impact_sys_num = 0;
    sm->impact_counter = 0;
    sm->impact_sum_sys = 0;
    sm->impact_hard_rescue_counter = 0;
    sm->impact_rescue_after_counter = 0;
    sm->impact_delayed_counter = 0;
    sm->impact_coll_checks = 0;
    sm->mindists_deleted = 0;
    sm->mindists_generated = 0;
    sm->processed_fmindists = 0;
    sm->range_intra_exceeded = 0;
    sm->range_world_exceeded = 0;
    sm->impact_unmov = 0;
}

void ivp_statistic_manager_init(ivp_statistic_manager_t *sm)
{
    if (!sm) return;
    sm->l_environment = NULL;
    sm->last_statistic_output = 0.0; /* IVP_Time() */
    sm->sum_energy_destr = 0;

    sm->max_rescue_speed = sm->max_speed_gain = 0;

    sm->impact_sys_num = sm->impact_counter = sm->impact_sum_sys = 0;
    sm->impact_hard_rescue_counter = sm->impact_rescue_after_counter = sm->impact_delayed_counter = 0;

    sm->impact_coll_checks = sm->impact_unmov = 0;

    sm->sum_of_mindists = sm->mindists_generated = sm->mindists_deleted = 0;
    sm->range_intra_exceeded = sm->range_world_exceeded = 0;

    sm->processed_fmindists = 0;
    sm->global_fmd_counter = 0;
}
