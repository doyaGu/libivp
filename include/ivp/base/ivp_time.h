/* ivp_time.h — Time types for IVP C17
 *
 * PSI (Physics Simulation Interval) uses fixed time steps.
 * ivp_time_t is always double (accumulated time needs precision).
 */
#ifndef IVP_TIME_H
#define IVP_TIME_H

#include <ivp/base/ivp_types.h>
#include <limits.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Time type (always double for accumulation precision) ───────────────── */

typedef double ivp_time_t;

/* Default PSI rate: 66 Hz (original IVP default) */
#define IVP_DEFAULT_PSI_RATE  66.0

/* Default delta time per PSI step */
#define IVP_DEFAULT_DELTA_TIME  (1.0 / IVP_DEFAULT_PSI_RATE)

/* ── Helpers ────────────────────────────────────────────────────────────── */

/* Calculate the number of PSI steps to simulate for a given elapsed time.
 * Returns the integer step count; remainder is stored in *residual if non-NULL. */
static inline int ivp_time_calc_steps(ivp_time_t elapsed, ivp_time_t dt,
                                       ivp_time_t *residual) {
    if (dt <= 0.0) {
        if (residual) *residual = elapsed;
        return 0;
    }
    ivp_time_t raw_steps = elapsed / dt;
    if (raw_steps <= 0.0) {
        if (residual) *residual = elapsed;
        return 0;
    }
    if (raw_steps >= (ivp_time_t)INT_MAX) {
        if (residual) {
            ivp_time_t rem = elapsed - ((ivp_time_t)INT_MAX * dt);
            *residual = rem > 0.0 ? rem : 0.0;
        }
        return INT_MAX;
    }
    int steps = (int)raw_steps;
    if (steps < 0) steps = 0;
    if (residual) {
        ivp_time_t rem = elapsed - (ivp_time_t)steps * dt;
        *residual = rem > 0.0 ? rem : 0.0;
    }
    return steps;
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_TIME_H */
