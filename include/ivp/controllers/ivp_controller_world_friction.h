/* ivp_controller_world_friction.h -- World friction controller
 *
 * Applies a ground/world friction force to objects, simulating contact
 * with an invisible ground plane or general environmental resistance.
 *
 * Modes:
 *   REFERENCE - port of IVP_Controller_World_Friction
 *               (ivp_controller_world_frict.cxx:96-178): per core axis the
 *               velocity is moved towards desired_speed_ws / desired_rot_speed_cs
 *               by at most friction_value_* * dt (values in m/s per second,
 *               reference defaults 0.4*9.81 and 0.4*9.81*0.3).
 *   GROUND    - libivp extra: Coulomb friction near a ground plane
 *   GLOBAL    - libivp extra: Coulomb friction for all dynamic objects
 *
 * Ported from ivp_controller_world_frict.hxx.
 */
#ifndef IVP_CONTROLLER_WORLD_FRICTION_H
#define IVP_CONTROLLER_WORLD_FRICTION_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── World friction mode ────────────────────────────────────────────────── */

typedef enum ivp_world_friction_mode {
    IVP_WORLD_FRICTION_GROUND = 0,  /* ground plane friction */
    IVP_WORLD_FRICTION_GLOBAL = 1,  /* global velocity damping */
    IVP_WORLD_FRICTION_REFERENCE = 2 /* reference desired-speed friction */
} ivp_world_friction_mode_t;

/* ── World friction state ───────────────────────────────────────────────── */

typedef struct ivp_world_friction_state {
    ivp_world_friction_mode_t mode;

    /* Ground plane: defined by normal and distance from origin */
    ivp_vec3r_t ground_normal;      /* unit normal (default: 0,1,0) */
    ivp_real_t  ground_dist;        /* signed distance from origin */
    ivp_real_t  ground_tolerance;   /* distance from plane to apply friction */

    /* Friction parameters */
    ivp_real_t  friction_coeff;     /* Coulomb friction coefficient */
    ivp_real_t  angular_friction;   /* angular velocity damping factor */

    /* REFERENCE mode (IVP_Template_Controller_World_Friction) */
    ivp_object_t *object;                  /* controlled object (NULL = any) */
    ivp_vec3r_t desired_speed_ws;
    ivp_vec3r_t desired_rot_speed_cs;
    ivp_vec3r_t friction_value_translation; /* per core axis [m/s^2], >= 0 */
    ivp_vec3r_t friction_value_rotation;    /* per core axis [rad/s^2], >= 0 */

    bool        active;
} ivp_world_friction_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

/* Initialize ground-plane world friction.
 * normal is the ground plane unit normal, height is the Y-position of
 * the ground plane. */
void ivp_world_friction_init_ground(ivp_world_friction_state_t *state,
                                      const ivp_vec3r_t *normal,
                                      ivp_real_t height,
                                      ivp_real_t friction_coeff);

/* Initialize the reference world friction controller for one object
 * (object may be NULL = every dispatched core) with the reference template
 * defaults: desired speeds 0, translation 0.4*9.81, rotation 0.4*9.81*0.3. */
void ivp_world_friction_init(ivp_world_friction_state_t *state, ivp_object_t *object);

/* Reference setters (clamp friction values to >= 0, wake the object). */
void ivp_world_friction_set_desired_speed_ws(ivp_world_friction_state_t *state,
                                             const ivp_vec3r_t *speed_ws);
void ivp_world_friction_set_desired_rot_speed_cs(ivp_world_friction_state_t *state,
                                                 const ivp_vec3r_t *rot_speed_cs);
void ivp_world_friction_set_friction_value_translation(ivp_world_friction_state_t *state,
                                                       const ivp_vec3r_t *value);
void ivp_world_friction_set_friction_value_rotation(ivp_world_friction_state_t *state,
                                                    const ivp_vec3r_t *value);

/* Initialize global world friction (applied to all objects). */
void ivp_world_friction_init_global(ivp_world_friction_state_t *state,
                                      ivp_real_t friction_coeff);

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_world_friction(const ivp_event_sim_t *event_sim,
                                     ivp_object_t *obj,
                                     void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_WORLD_FRICTION_H */
