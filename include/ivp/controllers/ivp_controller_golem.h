/* ivp_controller_golem.h — Animation-driven physics controller
 *
 * Drives objects toward animation keyframe targets. User sets target
 * position/orientation each frame; the controller computes velocity
 * corrections during each PSI to track the targets.
 *
 * If deviation exceeds thresholds, calls a user-provided problem_fn.
 *
 * Registered as a standard controller via ivp_environment_add_controller().
 *
 * Ported from ivp_controller/ivp_controller_golem.hxx.
 */
#ifndef IVP_CONTROLLER_GOLEM_H
#define IVP_CONTROLLER_GOLEM_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Problem callback ──────────────────────────────────────────────────── */

/* Called when the object deviates too far from its target.
 * pos_error: distance between current and target position.
 * rot_error: angular distance in radians. */
typedef void (*ivp_golem_problem_fn_t)(void *ctx,
                                        ivp_object_t *object,
                                        ivp_real_t pos_error,
                                        ivp_real_t rot_error);

/* ── Golem state ───────────────────────────────────────────────────────── */

typedef struct ivp_golem_state {
    /* Target position and orientation (world space) */
    ivp_vec3r_t target_position;
    ivp_quatr_t target_orientation;

    /* Tracking gains (stiffness). Higher = more aggressive correction.
     * Range: 0 (no correction) to 1 (full correction per step). */
    ivp_real_t  position_gain;   /* default: 0.5 */
    ivp_real_t  rotation_gain;   /* default: 0.5 */

    /* Speed limits to avoid instability */
    ivp_real_t  max_linear_speed;   /* default: 100 */
    ivp_real_t  max_angular_speed;  /* default: 10 (rad/s) */

    /* Deviation thresholds for problem callback */
    ivp_real_t  max_position_error; /* default: 1.0 (meters) */
    ivp_real_t  max_rotation_error; /* default: 0.5 (radians) */

    /* Problem callback */
    ivp_golem_problem_fn_t problem_fn;
    void                  *problem_ctx;

    /* Internal: whether targets have been set */
    bool target_position_set;
    bool target_orientation_set;
} ivp_golem_state_t;

/* ── Initialization ────────────────────────────────────────────────────── */

/* Initialize golem state with defaults. */
void ivp_golem_state_init(ivp_golem_state_t *state);

/* ── Target setters ────────────────────────────────────────────────────── */

/* Set the target position the object should track. */
void ivp_golem_set_target_position(ivp_golem_state_t *state,
                                   const ivp_vec3r_t *pos);

/* Set the target orientation the object should track. */
void ivp_golem_set_target_orientation(ivp_golem_state_t *state,
                                      const ivp_quatr_t *orientation);

/* ── Controller callback ───────────────────────────────────────────────── */

/* The controller callback. Pass an ivp_golem_state_t* as ctx.
 * Register with ivp_environment_add_controller(env, priority, fn, state). */
void ivp_controller_golem(const ivp_event_sim_t *event_sim,
                          ivp_object_t *obj,
                          void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_GOLEM_H */
