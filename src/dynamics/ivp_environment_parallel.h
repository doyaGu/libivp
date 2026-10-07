/* ivp_environment_parallel.h -- private ownership seam for world schedulers. */
#ifndef IVP_ENVIRONMENT_PARALLEL_H
#define IVP_ENVIRONMENT_PARALLEL_H

#include <ivp/dynamics/ivp_environment.h>

/* A simulation call owns an environment from try_begin through end.  These
 * functions do not make one environment internally parallel; they prevent
 * accidental concurrent entry while separate environments run in parallel. */
bool ivp_environment_try_begin_simulation(ivp_environment_t *env);
void ivp_environment_end_simulation(ivp_environment_t *env);
void ivp_environment_simulate_dtime_owned(ivp_environment_t *env,
                                          ivp_real_t elapsed_time);
void ivp_environment_simulate_one_psi_owned(ivp_environment_t *env);

#endif /* IVP_ENVIRONMENT_PARALLEL_H */
