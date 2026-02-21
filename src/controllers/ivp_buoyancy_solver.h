#ifndef IVP_BUOYANCY_SOLVER_INTERNAL_H
#define IVP_BUOYANCY_SOLVER_INTERNAL_H

#include <ivp/controllers/ivp_controller_buoyancy.h>

#ifdef __cplusplus
extern "C" {
#endif

void ivp_buoyancy_solver_step(const ivp_event_sim_t *event_sim,
                              ivp_object_t *obj,
                              const ivp_buoyancy_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
