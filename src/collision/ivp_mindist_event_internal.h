/* ivp_mindist_event_internal.h — IVP_Mindist_Event_Solver state (internal) */
#ifndef IVP_MINDIST_EVENT_INTERNAL_H
#define IVP_MINDIST_EVENT_INTERNAL_H

#include <ivp/collision/ivp_collision_core.h>

typedef struct ivp_mindist_event_solver {
    ivp_real_t sum_max_surface_rot_speed;
    ivp_real_t projected_center_speed;
    ivp_real_t max_coll_speed;
    ivp_real_t worst_case_speed;
    ivp_mindist_t *mindist;
    struct ivp_environment *environment;
    ivp_time_t t_now;
    ivp_time_t t_max;
    ivp_coll_type_t event_type_out;
    ivp_time_t event_time_out;
} ivp_mindist_event_solver_t;

void ivp_mindist_event_solver_calc_time_of_next_event(ivp_mindist_event_solver_t *mim);

#endif
