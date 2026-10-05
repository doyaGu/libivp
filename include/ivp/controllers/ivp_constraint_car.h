/* ivp_constraint_car.h -- Car constraint solver (body + wheels)
 *
 * Port of IVP_Constraint_Solver_Car / IVP_Constraint_Car_Object
 * (ivp_controller/ivp_constraint_car.{hxx,cxx}).
 *
 * A special, very fast vehicle constraint system: one body and up to
 * IVP_CONSTRAINT_CAR_MAX_WHEELS appended wheels.  Every PSI (priority
 * IVP_CP_CONSTRAINTS, an IVP_Controller_Dependent over the body and wheel
 * cores) it
 *   - turns each wheel so that its spin axis (x_idx column of the wheel
 *     target matrix) matches the body (rotation about the wheel's y_idx and
 *     z_idx axes, one 2x2 solve per wheel), and
 *   - keeps every wheel center on its hard point in the body's x_idx/z_idx
 *     directions through a pre-inverted push matrix (co_matrix).
 * NOT constrained are the vertical (y_idx) wheel translation (that is the
 * job of the suspension springs) and the wheel spin.  When a wheel drifts
 * away too fast (> max_delta_speed) the solver switches to "plan B": one
 * ball-and-socket constraint per wheel for at least 10 PSIs.
 *
 * Creating the solver sets core->car_wheel of the body and every wheel
 * core, which enables the reference "fast wheel" code paths of the core
 * integrator, the friction system (2D wheel friction), the impact solver
 * (no impact friction) and the mindist manager (wheel look-ahead mindists).
 *
 * Usually created by ivp_car_system_init (ivp_car_system.h).
 *
 * The builder's temporary matrix uses row stride == columns (reference
 * 81357f3; before that fix the reference left the stride at 0 and could
 * not build a car).
 */
#ifndef IVP_CONSTRAINT_CAR_H
#define IVP_CONSTRAINT_CAR_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/base/ivp_result.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IVP_CONSTRAINT_CAR_MAX_WHEELS 12

typedef struct ivp_constraint_solver_car ivp_constraint_solver_car_t;

/* IVP_Constraint_Car_Object: one terminal of the car constraint system
 * (the body or a wheel).  core->car_wheel of its object points here. */
typedef struct ivp_constraint_car_object {
    ivp_object_t                *real_object;
    ivp_constraint_solver_car_t *solver_car;
    ivp_transform_r_t            target_position_bs;      /* wheel pose in the body core system
                                                             (identity for the body) */
    ivp_vec3f_t                  last_contact_position_ws; /* written by the 2D wheel friction */
    float                        last_skid_value;          /* 0 = no skidding */
    ivp_time_t                   last_skid_time;
    void                        *fix_wheel_constraint;     /* non-NULL while the wheel is blocked
                                                              (handbrake, ivp_car_system_fix_wheel) */
} ivp_constraint_car_object_t;

/* Reference constructor IVP_Constraint_Solver_Car(right, up, forward,
 * is_left_hand).  Indices are 0..2 (x, y, z); "up" is the suspension axis
 * (pointing down for the default y-down gravity).  Returns NULL on OOM. */
ivp_constraint_solver_car_t *ivp_constraint_solver_car_create(int right_idx, int up_idx, int forward_idx,
                                                              bool is_left_hand);

/* Allocator-aware form used when the solver belongs to an environment.
 * The vtable is copied; its context must outlive the solver. */
ivp_constraint_solver_car_t *ivp_constraint_solver_car_create_with_allocator(
    int right_idx, int up_idx, int forward_idx, bool is_left_hand,
    const ivp_allocator_t *allocator);

/* Reference init_constraint_system: creates the car objects, builds and
 * inverts the push matrix and announces the controller to the environment.
 * wheel_pos_Bos[i] is the wheel hard point in the BODY's object system
 * (NULL: keep the wheel's current pose relative to the body).  On failure
 * everything is rolled back and IVP_ERR_* is returned. */
ivp_result_t ivp_constraint_solver_car_init_constraint_system(ivp_constraint_solver_car_t *solver,
                                                              ivp_environment_t *env,
                                                              ivp_object_t *body,
                                                              ivp_object_t *const *wheels,
                                                              const ivp_vec3f_t *wheel_pos_Bos,
                                                              int n_wheels);

/* ~IVP_Constraint_Solver_Car: removes the controller (silently), clears
 * core->car_wheel and frees plan-B constraints and the solver.  Safe after
 * the environment has been destroyed (then only memory is released). */
void ivp_constraint_solver_car_destroy(ivp_constraint_solver_car_t *solver);

/* True while the solver is announced to an environment (false before
 * init_constraint_system, after a failed init, and after one of its
 * objects was deleted -- the reference deletes the solver in that case,
 * libivp detaches it). */
bool ivp_constraint_solver_car_is_active(const ivp_constraint_solver_car_t *solver);

int ivp_constraint_solver_car_get_num_wheels(const ivp_constraint_solver_car_t *solver);
ivp_constraint_car_object_t *ivp_constraint_solver_car_get_wheel(ivp_constraint_solver_car_t *solver, int i);
ivp_constraint_car_object_t *ivp_constraint_solver_car_get_body(ivp_constraint_solver_car_t *solver);
void ivp_constraint_solver_car_get_axes(const ivp_constraint_solver_car_t *solver,
                                        int *x_idx, int *y_idx, int *z_idx);

/* IVP_Controller_Manager::ensure_controller_in_simulation(solver). */
void ivp_constraint_solver_car_ensure_in_simulation(ivp_constraint_solver_car_t *solver);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONSTRAINT_CAR_H */
