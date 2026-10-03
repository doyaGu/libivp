/* ivp_controller_golem.h — Animation-driven physics controller
 *
 * Port of IVP_Controller_Golem (ivp_controller/ivp_controller_motion.cxx:
 * 206-399), which derives from IVP_Controller_Motion.  The animation system
 * provides a "prime" position + velocity (and one or two prime orientations);
 * each PSI the controller extrapolates the target from them and drives the
 * core like the motion controller, with a velocity feed-forward term:
 *
 *   target = prime_pos + velocity * (now - t_prime)   (object origin)
 *   delta  = (target - pos - R * shift_core_f_object) * force_factor
 *          + velocity * dt * damp_factor - speed * dt * damp_factor
 *   (per-axis clip to max_translation_force)   speed += delta / dt
 *
 * The rotational part mirrors the motion controller (damped with
 * damp_factor, as in the reference).  When the position error exceeds
 * max_delta_position (IVP_GP_FAR_DISTANCE) or the angle error exceeds
 * max_delta_orientation (IVP_GP_BIG_ANGLE) the problem callback is called
 * (reference resolve_for_problem) and NO further force is applied this PSI.
 *
 * Bind the state to an object with ivp_golem_state_setup(); an unbound
 * state acts on every core it is dispatched for (legacy libivp behaviour).
 */
#ifndef IVP_CONTROLLER_GOLEM_H
#define IVP_CONTROLLER_GOLEM_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reference IVP_GOLEM_PROBLEM. */
typedef enum ivp_golem_problem {
    IVP_GP_NONE = -1,
    IVP_GP_FAR_DISTANCE = 0,
    IVP_GP_BIG_ANGLE = 1,
    IVP_GP_TIME_BY_DISTANCE_TOO_LONG = 2
} ivp_golem_problem_t;

/* ── Problem callback (reference resolve_for_problem) ──────────────────── */

/* pos_error: distance between current and target position.
 * rot_error: |angles| of the orientation error (radians).
 * state->last_problem tells which problem triggered the call. */
typedef void (*ivp_golem_problem_fn_t)(void *ctx,
                                        ivp_object_t *object,
                                        ivp_real_t pos_error,
                                        ivp_real_t rot_error);

/* ── Golem state ───────────────────────────────────────────────────────── */

typedef struct ivp_golem_state {
    ivp_object_t *object;           /* bound object (NULL = any dispatched core) */

    /* IVP_Template_Controller_Motion part (reference defaults) */
    ivp_real_t  force_factor;           /* 0.8 */
    ivp_real_t  damp_factor;            /* 1.0 */
    ivp_real_t  torque_factor;          /* 0.8 */
    ivp_real_t  angular_damp_factor;    /* 1.0 (unused by the golem, as in the reference) */
    ivp_vec3r_t max_translation_force;  /* 1e6 per axis */
    ivp_vec3r_t max_torque;             /* 1e6 per axis */

    /* IVP_Template_Controller_Golem part */
    ivp_real_t  max_delta_position;            /* 10 m */
    ivp_real_t  max_delta_orientation;         /* 0.5 * PI */
    ivp_real_t  max_integrated_delta_position; /* 10 (not used, as in the reference) */
    ivp_real_t  max_golem_force;               /* IVP_FLOAT_MAX (not used) */
    ivp_real_t  filter_dtime;                  /* 5 (not used) */

    /* Prime (animation) data */
    ivp_vec3r_t target_position;        /* prime_position_ws */
    ivp_vec3r_t velocity_ws;            /* prime velocity */
    ivp_time_t  time_of_prime_position;
    ivp_quatr_t target_orientation;     /* prime_orientation_0 */
    ivp_quatr_t prime_orientation_1;
    ivp_time_t  time_of_prime_orientation_0;
    ivp_real_t  i_delta_prime_orientation_time;
    bool        angular_velocity_set;

    /* Dynamic values */
    ivp_vec3r_t target_pos_ws;          /* extrapolated target of the last PSI */
    ivp_quatr_t target_q_world_f_core;  /* interpolated target of the last PSI */
    ivp_real_t  integrated_delta_position;
    ivp_golem_problem_t last_problem;

    /* Problem callback */
    ivp_golem_problem_fn_t problem_fn;
    void                  *problem_ctx;

    /* Whether the prime data was set (else primed from the object on the
     * first PSI, like the reference constructor). */
    bool target_position_set;
    bool target_orientation_set;
} ivp_golem_state_t;

/* ── Initialization ────────────────────────────────────────────────────── */

/* Initialize golem state with the reference template defaults. */
void ivp_golem_state_init(ivp_golem_state_t *state);

/* Bind to an object and prime position/orientation from its current pose
 * at the environment's current time (reference constructor). */
void ivp_golem_state_setup(ivp_golem_state_t *state, ivp_object_t *object);

/* ── Prime setters (reference set_prime_position / set_prime_orientation) ─ */

void ivp_golem_set_prime_position(ivp_golem_state_t *state,
                                  const ivp_vec3r_t *position,
                                  const ivp_vec3r_t *velocity,
                                  ivp_time_t time);

/* orientation1 may be NULL (no angular velocity); dt is the time between the
 * two orientations. */
void ivp_golem_set_prime_orientation(ivp_golem_state_t *state,
                                     const ivp_quatr_t *orientation0,
                                     ivp_time_t time0,
                                     const ivp_quatr_t *orientation1,
                                     ivp_real_t dt);

/* Reference IVP_Controller_Golem::reset_time (IVP_Environment::reset_time
 * shifts all times by offset): time_of_prime_position -= offset. */
void ivp_golem_reset_time(ivp_golem_state_t *state, ivp_time_t offset);

/* Convenience: static prime position (zero velocity) at the current time. */
void ivp_golem_set_target_position(ivp_golem_state_t *state,
                                   const ivp_vec3r_t *pos);

/* Convenience: static prime orientation at the current time. */
void ivp_golem_set_target_orientation(ivp_golem_state_t *state,
                                      const ivp_quatr_t *orientation);

/* Reference beam_object_to_target_position (for use in the problem callback). */
void ivp_golem_beam_object_to_target_position(ivp_golem_state_t *state,
                                              ivp_object_t *object);

/* ── Controller callback ───────────────────────────────────────────────── */

/* The controller callback. Pass an ivp_golem_state_t* as ctx.
 * Registered at IVP_CP_MOTION (synchronous). */
void ivp_controller_golem(const ivp_event_sim_t *event_sim,
                          ivp_object_t *obj,
                          void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_GOLEM_H */
