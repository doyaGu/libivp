/* ivp_controller_stiff_spring.h -- Stiff spring controller
 *
 * Port of IVP_Controller_Stiff_Spring
 * (ivp_controller/ivp_controller_stiff_spring.cxx:182-226).
 *
 * Unlike ivp_actuator_spring the constants are dimensionless:
 *   spring_constant (stiffness) in [0.001 .. 1], spring_damp (damping) in [0 .. 1].
 * Each PSI the relative anchor velocity along the spring axis is driven
 * towards the rest length with an impulse solved through the two-core
 * reaction matrix (linear AND angular response at the anchor):
 *   a   = (rest_length - dlen) / dt * k - dv * (k + d * (1 - k))
 *   imp = a / tpm00,  applied at anchor 0 (+imp on obj_a, -imp on obj_b).
 * Priority IVP_CP_STIFF_SPRINGS (synchronous).
 */
#ifndef IVP_CONTROLLER_STIFF_SPRING_H
#define IVP_CONTROLLER_STIFF_SPRING_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/utility/ivp_active_value.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_stiff_spring_state;

/* Reference IVP_Listener_Stiff_Spring::event_stiff_spring_broken. */
typedef void (*ivp_stiff_spring_broken_fn_t)(void *ctx, struct ivp_stiff_spring_state *spring);

/* ── Stiff spring state ────────────────────────────────────────────────── */

typedef struct ivp_stiff_spring_state {
    /* Attachment */
    ivp_object_t *obj_a;            /* first object / anchor 0 (NULL = world) */
    ivp_object_t *obj_b;            /* second object / anchor 1 (NULL = world) */
    /* Reference IVP_Anchor::core_pos (float-exact core-space anchors); the
     * setup takes object-space coordinates, world coordinates for NULL. */
    ivp_vec3r_t   anchor_a_os;
    ivp_vec3r_t   anchor_b_os;

    /* Spring parameters (reference template defaults: all 0) */
    ivp_real_t    stiffness;        /* spring_constant [0..1] (dimensionless) */
    ivp_real_t    damping;          /* spring_damp [0..1] (dimensionless) */
    ivp_real_t    rest_length;      /* rest length (m) */
    ivp_real_t    max_dv;           /* libivp extra: |impulse/m_eff| clamp (0 = off) */

    /* Break handling (reference IVP_SFE_BREAK) */
    ivp_real_t                    break_max_len;       /* default IVP_FLOAT_MAX */
    ivp_spring_force_exceed_t     max_len_exceed_type; /* default IVP_SFE_NONE */
    bool                          broken;
    ivp_stiff_spring_broken_fn_t  broken_fn;
    void                         *broken_ctx;

    bool          active;

    /* Runtime output */
    ivp_real_t    current_length;
    ivp_real_t    last_impulse;     /* impulse applied in the last PSI */

    /* Reference IVP_Controller_Stiff_Spring_Active
     * (IVP_Template_Stiff_Spring_Active), see ivp_stiff_spring_bind_active. */
    ivp_active_float_t *active_float_spring_len;
    ivp_active_float_t *active_float_spring_constant;
    ivp_active_float_t *active_float_spring_damp;
} ivp_stiff_spring_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_stiff_spring_state_init(ivp_stiff_spring_state_t *state);

void ivp_stiff_spring_state_setup(ivp_stiff_spring_state_t *state,
                                    ivp_object_t *obj_a,
                                    const ivp_vec3r_t *anchor_a_os,
                                    ivp_object_t *obj_b,
                                    const ivp_vec3r_t *anchor_b_os,
                                    ivp_real_t stiffness,
                                    ivp_real_t damping,
                                    ivp_real_t rest_length);

/* Reference setters (wake the controlled objects). */
void ivp_stiff_spring_set_constant(ivp_stiff_spring_state_t *state, ivp_real_t value);
void ivp_stiff_spring_set_damp(ivp_stiff_spring_state_t *state, ivp_real_t value);
void ivp_stiff_spring_set_len(ivp_stiff_spring_state_t *state, ivp_real_t value);
void ivp_stiff_spring_set_break_max_len(ivp_stiff_spring_state_t *state, ivp_real_t value);
void ivp_stiff_spring_set_broken_listener(ivp_stiff_spring_state_t *state,
                                          ivp_stiff_spring_broken_fn_t fn, void *ctx);

/* Reference IVP_Controller_Stiff_Spring_Active: subscribes to the given
 * active floats (each may be NULL).  As the reference constructor, the
 * current float values are copied into rest_length / stiffness / damping
 * directly (no wake-up); later changes go through ivp_stiff_spring_set_len /
 * set_constant / set_damp (which wake the spring).  Call after setup;
 * previous bindings are released first. */
void ivp_stiff_spring_bind_active(ivp_stiff_spring_state_t *state,
                                  ivp_active_float_t *active_float_spring_len,
                                  ivp_active_float_t *active_float_spring_constant,
                                  ivp_active_float_t *active_float_spring_damp);

/* Reference ~IVP_Controller_Stiff_Spring_Active: removes the dependencies. */
void ivp_stiff_spring_unbind_active(ivp_stiff_spring_state_t *state);

/* Discovery function for ivp_environment_announce_dependent_controller():
 * reports the movable objects of the actuator so that their simulation
 * units are merged (reference IVP_Controller_Dependent semantics). */
int ivp_stiff_spring_discovery(void *ctx, ivp_environment_t *env,
               ivp_object_handle_t *out_handles, int max_handles);

/* ── Controller callback ────────────────────────────────────────────────── */

void ivp_controller_stiff_spring(const ivp_event_sim_t *event_sim,
                                   ivp_object_t *obj,
                                   void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_STIFF_SPRING_H */
