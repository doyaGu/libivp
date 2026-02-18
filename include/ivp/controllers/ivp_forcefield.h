/* ivp_forcefield.h -- Force field controller
 *
 * Applies a constant or position-dependent force to objects.
 * Useful for custom gravity regions, wind, buoyancy approximation, etc.
 *
 * Ported from ivp_forcefield.hxx.
 */
#ifndef IVP_FORCEFIELD_H
#define IVP_FORCEFIELD_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Force field mode ───────────────────────────────────────────────────── */

typedef enum ivp_forcefield_mode {
    IVP_FORCEFIELD_CONSTANT     = 0,  /* constant acceleration (like gravity) */
    IVP_FORCEFIELD_FORCE        = 1,  /* constant force (mass-dependent) */
    IVP_FORCEFIELD_DRAG         = 2   /* velocity-proportional drag */
} ivp_forcefield_mode_t;

/* ── Force field state ──────────────────────────────────────────────────── */

typedef struct ivp_forcefield_state {
    ivp_forcefield_mode_t mode;
    ivp_vec3r_t           value;       /* acceleration (m/s²), force (N),
                                          or drag coefficient */
    bool                  active;      /* on/off toggle */
} ivp_forcefield_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

/* Initialize with a constant acceleration (like gravity). */
void ivp_forcefield_state_init(ivp_forcefield_state_t *state,
                                ivp_forcefield_mode_t mode,
                                const ivp_vec3r_t *value);

/* ── Controller callback ────────────────────────────────────────────────── */

/* Force field controller. Pass ivp_forcefield_state_t* as ctx. */
void ivp_controller_forcefield(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_FORCEFIELD_H */
