/* ivp_statistic_manager.h — IVP_Statistic_Manager
 *
 * Port of the reference IVP_Statistic_Manager (ivp_physics/ivp_environment.hxx,
 * methods in ivp_physics/ivp_physic.cxx): simple per environment counters.
 * The engine updates them in every build, exactly where the reference does
 * (mindists generated / deleted, OV tree range events, impacts, rescues,
 * friction mindists, destroyed energy, ...).  Debug builds (IVP_IF) print
 * them with output_statistic once per simulated second at the start of a PSI
 * (ivp_environment.cxx simulate_psi), which also clears them.
 *
 * Get it with ivp_environment_get_statistic_manager.
 */
#ifndef IVP_STATISTIC_MANAGER_H
#define IVP_STATISTIC_MANAGER_H

#include <ivp/base/ivp_time.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_environment;

typedef struct ivp_statistic_manager {
    struct ivp_environment *l_environment;
    /* statistic section */
    ivp_time_t last_statistic_output;

    /* impact section */
    float max_rescue_speed;
    float max_speed_gain;
    int impact_sys_num;
    int impact_counter;              /* number of total impacts */
    int impact_sum_sys;              /* number of system impacts (which are much faster) */
    int impact_hard_rescue_counter;  /* number of bad impacts (very slow movement) */
    int impact_rescue_after_counter;
    int impact_delayed_counter;      /* impacts where one object gets a delayed speed update */
    int impact_coll_checks;          /* number of AT collision checks during impacts */
    int impact_unmov;

    double sum_energy_destr;

    /* collision checks section */
    int sum_of_mindists;      /* number of mindists (not cleared by clear_statistic) */
    int mindists_generated;   /* number of mindists just created */
    int mindists_deleted;     /* number of just deleted mindists */

    int range_intra_exceeded; /* events: sphere for checking two objects recalculated */
    int range_world_exceeded; /* events: sphere for checking object - world recalculated */

    /* friction */
    int processed_fmindists;  /* number of calls to recalc_friction_s_vals */
    int global_fmd_counter;
} ivp_statistic_manager_t;

/* IVP_Statistic_Manager::IVP_Statistic_Manager (l_environment = NULL,
 * last_statistic_output = 0, every counter 0) */
void ivp_statistic_manager_init(ivp_statistic_manager_t *sm);
/* clear_statistic: every counter except sum_of_mindists, sum_energy_destr
 * and global_fmd_counter */
void ivp_statistic_manager_clear(ivp_statistic_manager_t *sm);
/* output_statistic: one "nr_impacts ..." line (ivp_debug_printf), then clear */
void ivp_statistic_manager_output(ivp_statistic_manager_t *sm);

#ifdef __cplusplus
}
#endif

#endif /* IVP_STATISTIC_MANAGER_H */
