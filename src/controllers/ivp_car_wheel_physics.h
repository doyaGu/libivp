/* ivp_car_wheel_physics.h -- car-wheel hooks of the core integrator and the
 * friction system (reference ivp_controller/ivp_car_wheel_physics.cxx).
 *
 * Both functions are only called for cores whose car_wheel is set (by the
 * car constraint solver, ivp_constraint_car.h).  The reference defines them
 * in the controller library "to break the ivp_physics -> ivp_controller
 * cycle"; libivp keeps the split for the same reason.
 */
#ifndef IVP_CAR_WHEEL_PHYSICS_H
#define IVP_CAR_WHEEL_PHYSICS_H

#include <stdbool.h>
#include <ivp/math/ivp_quat.h>

struct ivp_core;
struct ivp_contact_pt;
struct ivp_impact_solver_lt;
struct ivp_sim_es;

/* IVP_Calc_Next_PSI_Solver::calc_fast_wheel_rotation (IVP_FAST_WHEELS_ENABLED):
 * the spin about the wheel axis (solver x_idx) is integrated exactly, the
 * remaining (gyro) rotation with set_fast_multiple_with_clip. */
void ivp_car_wheel_calc_fast_wheel_rotation(struct ivp_core *core, float delta_sim_time,
                                            ivp_quatr_t *q_core_f_core_out);

/* IVP_Contact_Point::friction_force_local_constraint_2d_wheel.  Returns
 * false when the generic 2D friction solver has to be used instead;
 * *flEnergy receives the friction energy change (always 0). */
bool ivp_car_wheel_friction_force_local_constraint_2d_wheel(struct ivp_contact_pt *cp, struct ivp_core *core_a,
                                                            struct ivp_impact_solver_lt *info,
                                                            const struct ivp_sim_es *es, float *flEnergy);

#endif /* IVP_CAR_WHEEL_PHYSICS_H */
