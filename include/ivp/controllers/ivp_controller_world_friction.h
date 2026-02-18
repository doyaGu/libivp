/* ivp_controller_world_friction.h -- World friction controller
 *
 * Applies a ground/world friction force to objects, simulating contact
 * with an invisible ground plane or general environmental resistance.
 *
 * Two modes:
 *   GROUND - friction relative to a ground plane (objects must be near
 *            the plane to receive friction)
 *   GLOBAL - friction applied to all dynamic objects regardless of position
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
    IVP_WORLD_FRICTION_GLOBAL = 1   /* global velocity damping */
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
