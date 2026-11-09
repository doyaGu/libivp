/* ivp_legacy.h — Reference-engine numeric conventions used by the core port
 *
 * The reference IVP default build uses IVP_DOUBLE = double for world-space
 * quantities and IVP_FLOAT = float for speeds, directions and per-object
 * caches.  The core pipeline (time manager, hull managers, mindists,
 * impacts, friction) is ported with those exact storage types so rounding
 * matches the reference.
 */
#ifndef IVP_LEGACY_H
#define IVP_LEGACY_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ivu_types.hxx (double build) */
#define P_FLOAT_EPS        1e-10f
#define P_FLOAT_RES        1e-6f
#define P_FLOAT_MAX        1e16f
#ifdef IVP_REAL_DOUBLE
#define P_DOUBLE_MAX       1e20
#define P_DOUBLE_RES       1E-12
#define P_DOUBLE_EPS       1e-19 /* retail division/normalization cutoff */
#define P_MAX_WORLD_DOUBLE 10000.0
#else
#define P_DOUBLE_MAX       P_FLOAT_MAX
#define P_DOUBLE_RES       P_FLOAT_RES
#define P_DOUBLE_EPS       P_FLOAT_EPS
#define P_MAX_WORLD_DOUBLE 3000.0f
#endif
#define P_MAX_OBJECT_SIZE  1000.0f
#define P_MIN_EDGE_LEN     0.01f
#define P_RES_EPS          (P_MAX_WORLD_DOUBLE * P_DOUBLE_RES)

/* IVP_Time::operator- (ivu_types.hxx).  Every reference "time difference"
 * goes through it. */
static inline double ivp_time_diff(ivp_time_t a, ivp_time_t b) {
    return a - b; /* retail: no float cast (the later SDK rounded to float) */
}

/* IVP_Time_CODE: incremented whenever the environment time changes */
typedef unsigned int ivp_time_code_t;

#ifdef __cplusplus
}
#endif

#endif /* IVP_LEGACY_H */
