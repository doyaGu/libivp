/* ivp_buoyancy_solver.h -- internal port of IVP_Buoyancy_Solver
 * (ivp_controller/ivp_buoyancy_solver.hxx), with the reference member and
 * value types: IVP_FLOAT members and IVP_U_Float_Point vectors are float,
 * IVP_DOUBLE locals are ivp_real_t. */
#ifndef IVP_BUOYANCY_SOLVER_INTERNAL_H
#define IVP_BUOYANCY_SOLVER_INTERNAL_H

#include <ivp/controllers/ivp_controller_buoyancy.h>
#include <ivp/math/ivp_vec3.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reference IVP_U_Float_Hesse: get_dist(p) = n . p + hesse_val
 * (positive = in the liquid). */
typedef struct ivp_bs_hesse {
    ivp_vec3f_t n;
    float hesse_val;
} ivp_bs_hesse_t;

typedef struct ivp_buoyancy_solver {
    float buoyancy_eps;
    ivp_core_t *core;
    ivp_real_t delta_time;

    bool  simulate_wing_behavior;
    float medium_density;
    float pressure_damp_factor;    /* template value * 0.5f * medium_density */
    float friction_damp_factor;    /* template value * 0.5f * medium_density */
    float viscosity_factor;
    ivp_vec3f_t resulting_speed_of_current_ws;
    ivp_vec3f_t resulting_speed_of_current_os;
    float torque_factor;
    float viscosity_input_factor;
    float ball_rot_dampening_factor;

    /* results */
    float       object_visible_surface_content_under;
    ivp_vec3f_t sum_impulse;
    ivp_vec3f_t sum_impulse_x_point;
    ivp_vec3f_t sum_impulse_x_movevector;
    float       volume_under;
    ivp_vec3f_t volume_center_under;
} ivp_buoyancy_solver_t;

/* IVP_Buoyancy_Solver(core, cntrl, input, resulting_speed_of_current_ws) */
void ivp_buoyancy_solver_init(ivp_buoyancy_solver_t *bs,
                              ivp_core_t *core,
                              const ivp_buoyancy_state_t *tmpl,
                              const ivp_vec3f_t *resulting_speed_of_current_ws);

/* IVP_Buoyancy_Solver::compute_forces: true when the object is in the
 * liquid. */
bool ivp_buoyancy_solver_compute_forces(ivp_buoyancy_solver_t *bs,
                                        const ivp_vec3f_t *rel_speed_of_current_os,
                                        const ivp_bs_hesse_t *surface_os,
                                        ivp_object_t *object);

#ifdef __cplusplus
}
#endif

#endif
