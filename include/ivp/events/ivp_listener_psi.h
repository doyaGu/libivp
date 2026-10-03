/* ivp_listener_psi.h — Per-PSI step listener (convenience wrapper)
 *
 * A simpler callback interface for code that needs to run once per PSI
 * step. Wraps the generic event system and filters for SIM_UNIT_BEGIN.
 *
 * Ported from ivp_listener_psi.hxx.
 */
#ifndef IVP_LISTENER_PSI_H
#define IVP_LISTENER_PSI_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/base/ivp_forward_decls.h>
#include <ivp/dynamics/ivp_handle.h>
#include <ivp/events/ivp_event.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── PSI callback ──────────────────────────────────────────────────────── */

/* Called once per simulation unit within each PSI step.
 *
 * Parameters:
 *   ctx     -- user context pointer
 *   env     -- the environment
 *   time    -- current simulation time
 *   dt      -- duration of this simulation unit step */
typedef void (*ivp_psi_listener_fn_t)(void *ctx,
                                      ivp_environment_t *env,
                                      ivp_time_t time,
                                      ivp_real_t dt);

/* ── PSI listener state ────────────────────────────────────────────────── */

/* IVP_Listener_PSI::environment_will_be_deleted */
typedef void (*ivp_psi_listener_env_deleted_fn_t)(void *ctx, ivp_environment_t *env);

typedef struct ivp_psi_listener {
    ivp_psi_listener_fn_t     fn;
    ivp_psi_listener_env_deleted_fn_t env_deleted_fn; /* optional */
    void                     *ctx;
    ivp_event_listener_handle_t event_handle; /* internal: handle in event system */
    ivp_environment_t        *registered_env; /* internal: env owning event_handle */
    bool                      active;
    uint32_t                  _magic;         /* internal: initialization guard */
} ivp_psi_listener_t;

/* ── API ───────────────────────────────────────────────────────────────── */

/* Register a PSI listener on an environment.
 * Returns 0 on success, -1 on failure. */
int ivp_psi_listener_register(ivp_psi_listener_t *listener,
                              ivp_environment_t *env,
                              ivp_psi_listener_fn_t fn,
                              void *ctx);

/* Unregister a PSI listener.
 * Returns 0 on success, -1 if not found. */
int ivp_psi_listener_unregister(ivp_psi_listener_t *listener,
                                ivp_environment_t *env);

/* Optional IVP_Listener_PSI::environment_will_be_deleted callback (called with
 * the listener ctx when the environment is destroyed while registered). */
void ivp_psi_listener_set_environment_deleted_fn(ivp_psi_listener_t *listener,
                                                 ivp_psi_listener_env_deleted_fn_t fn);

#ifdef __cplusplus
}
#endif

#endif /* IVP_LISTENER_PSI_H */
