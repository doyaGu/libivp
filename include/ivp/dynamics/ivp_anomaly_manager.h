/* ivp_anomaly_manager.h -- Physics anomaly detection and recovery
 *
 * Detects and corrects simulation anomalies:
 *   - NaN / Inf velocities
 *   - Excessive speed (above a configurable threshold)
 *   - Excessive energy gain (energy explosion)
 *   - Deep interpenetration beyond collision tolerance
 *
 * On detection, fires a callback and optionally auto-corrects by
 * clamping velocities, zeroing NaN values, or freezing objects.
 *
 * Ported from ivp_anomaly_manager.hxx.
 */
#ifndef IVP_ANOMALY_MANAGER_H
#define IVP_ANOMALY_MANAGER_H

#include <ivp/base/ivp_types.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/base/ivp_forward_decls.h>
#include <ivp/events/ivp_event.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Anomaly types ─────────────────────────────────────────────────────── */

typedef enum ivp_anomaly_type {
    IVP_ANOMALY_NAN_VELOCITY     = 0x01,
    IVP_ANOMALY_EXCESSIVE_SPEED  = 0x02,
    IVP_ANOMALY_ENERGY_EXPLOSION = 0x04,
    IVP_ANOMALY_DEEP_PENETRATION = 0x08,
    IVP_ANOMALY_NAN_POSITION     = 0x10,
    IVP_ANOMALY_MAX_COLLISIONS   = 0x20
} ivp_anomaly_type_t;

/* ── Anomaly callback ──────────────────────────────────────────────────── */

typedef void (*ivp_anomaly_callback_fn_t)(void *ctx,
                                            ivp_object_t *obj,
                                            ivp_anomaly_type_t type,
                                            const char *message);

/* ── Anomaly manager configuration ─────────────────────────────────────── */

typedef struct ivp_anomaly_config {
    ivp_real_t max_speed;          /* max allowed speed [m/s] (default: 1000) */
    ivp_real_t max_angular_speed;  /* max angular speed [rad/s] (default: 100) */
    ivp_real_t energy_gain_ratio;  /* max allowed energy gain per PSI (default: 10.0) */
    bool       auto_correct;       /* automatically fix anomalies (default: true) */
    int        max_collisions_per_psi;  /* max collisions before freeze (default: 200) */
    bool       freeze_on_max_collisions; /* freeze object on collision limit (default: true) */
    ivp_anomaly_callback_fn_t callback;
    void      *callback_ctx;
} ivp_anomaly_config_t;

static inline void ivp_anomaly_config_init(ivp_anomaly_config_t *cfg) {
    cfg->max_speed = IVP_REAL(2000.0);         /* reference: IVP_Anomaly_Limits::max_velocity = 2000 */
    cfg->max_angular_speed = IVP_REAL(100.0);   /* reference: PI/2 per PSI ≈ 103 rad/s at 66Hz */
    cfg->energy_gain_ratio = IVP_REAL(10.0);
    cfg->auto_correct = true;
    cfg->max_collisions_per_psi = 200;
    cfg->freeze_on_max_collisions = true;
    cfg->callback = NULL;
    cfg->callback_ctx = NULL;
}

/* ── Anomaly manager state ─────────────────────────────────────────────── */

typedef struct ivp_anomaly_manager {
    ivp_anomaly_config_t config;
    int  anomalies_detected;  /* cumulative count */
    int  anomalies_corrected; /* cumulative count */
    int  collisions_this_psi; /* total collisions counted this PSI */
    bool active;
} ivp_anomaly_manager_t;

/* ── API ───────────────────────────────────────────────────────────────── */

/* Initialize the anomaly manager. config may be NULL for defaults. */
void ivp_anomaly_manager_init(ivp_anomaly_manager_t *mgr,
                                const ivp_anomaly_config_t *config);

/* Check a single object for anomalies. Returns bitmask of detected types.
 * If auto_correct is enabled, anomalies are fixed in-place. */
int ivp_anomaly_check_object(ivp_anomaly_manager_t *mgr,
                               ivp_object_t *obj);

/* Check all objects in an environment. Returns total anomaly count. */
struct ivp_environment;
int ivp_anomaly_check_all(ivp_anomaly_manager_t *mgr,
                            struct ivp_environment *env);

/* Reset per-PSI collision counter. Call at start of each PSI. */
void ivp_anomaly_manager_begin_psi(ivp_anomaly_manager_t *mgr);

/* Count a collision for an object. Returns true if collision is allowed,
 * false if max_collisions_per_psi exceeded (object should be frozen). */
bool ivp_anomaly_manager_count_collision(ivp_anomaly_manager_t *mgr,
                                           ivp_object_t *obj);

/* Controller callback version: checks the object passed in.
 * Pass ivp_anomaly_manager_t* as ctx. */
void ivp_controller_anomaly_check(const ivp_event_sim_t *event_sim,
                                    ivp_object_t *obj,
                                    void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ANOMALY_MANAGER_H */
