/* ivp_actuator_spring.h -- Spring force actuator
 *
 * Connects two objects (or one object to a world anchor) with a spring.
 * Port of IVP_Actuator_Spring (ivp_controller/ivp_actuator_spring.cxx):
 *
 *   dir   = pos_a - pos_b (normalized), dlen = |pos_a - pos_b|
 *   force = (dlen - rest_length) * stiffness - damping * dir.(v_b - v_a)
 *   F     = dir * force * dt - rel_pos_damp * dt * (v_b - v_a)
 *   obj_b is pushed by +F at its anchor, obj_a by -F (point impulses).
 *
 * Reference defaults (IVP_Template_Spring): all constants 0,
 * break_max_len = P_FLOAT_MAX, no exceed handling.
 */
#ifndef IVP_ACTUATOR_SPRING_H
#define IVP_ACTUATOR_SPRING_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/utility/ivp_active_value.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reference IVP_SPRING_FORCE_EXCEED. */
typedef enum ivp_spring_force_exceed {
    IVP_SFE_NONE  = 0,
    IVP_SFE_BREAK = 1   /* destroy spring when length exceeds break_max_len */
} ivp_spring_force_exceed_t;

struct ivp_spring_state;

/* Reference IVP_Listener_Spring::event_spring_broken. */
typedef void (*ivp_spring_broken_fn_t)(void *ctx, struct ivp_spring_state *spring);

/* ── Spring state ───────────────────────────────────────────────────────── */

typedef struct ivp_spring_state {
    /* Attachment: object pointers and local anchor points */
    ivp_object_t *obj_a;            /* first object / anchor 0 (NULL = world) */
    ivp_object_t *obj_b;            /* second object / anchor 1 (NULL = world) */
    /* Reference IVP_Anchor::core_pos (IVP_U_Float_Point, float-exact): the
     * anchor in the core space of obj_a / obj_b (of the environment's static
     * object, i.e. world space, for a NULL object). */
    ivp_vec3r_t   anchor_a_os;
    ivp_vec3r_t   anchor_b_os;

    /* Spring parameters (already multiplied with spring_values_factor) */
    ivp_real_t    stiffness;        /* spring constant k (N/m) */
    ivp_real_t    damping;          /* damping constant c (N·s/m) */
    ivp_real_t    rest_length;      /* rest length (m) */
    ivp_real_t    rel_pos_damp;     /* "manhattan" damping on full relative velocity */
    ivp_real_t    max_force;        /* libivp extra: |force| clamp (0 = unlimited) */

    /* Reference relative values: constants are scaled by the reduced
     * minimal virtual mass of both anchors (see ivp_spring_state_make_relative). */
    ivp_real_t    spring_values_factor;   /* default 1 */
    bool          spring_force_only_on_stretch;

    /* Break handling */
    ivp_real_t                 break_max_len;        /* default IVP_FLOAT_MAX */
    ivp_spring_force_exceed_t  max_len_exceed_type;  /* default IVP_SFE_NONE */
    bool                       broken;               /* set once broken; spring is inert */
    ivp_spring_broken_fn_t     broken_fn;
    void                      *broken_ctx;

    /* State (computed per step) */
    ivp_real_t    current_length;   /* current spring length */
    ivp_real_t    current_force;    /* current force magnitude */

    /* Reference IVP_Actuator_Spring_Active: optional overrides of
     * IVP_Template_Spring (see ivp_spring_bind_active). */
    ivp_active_float_t *active_float_spring_len;
    ivp_active_float_t *active_float_spring_constant;
    ivp_active_float_t *active_float_spring_damp;
    ivp_active_float_t *active_float_spring_rel_pos_damp;
} ivp_spring_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

/* Initialize a spring with the reference template defaults (all zero). */
void ivp_spring_state_init(ivp_spring_state_t *state);

/* Configure a spring between two objects at given anchor points.
 * anchor_a_os / anchor_b_os are given in each object's object space
 * (reference IVP_Template_Anchor::set_anchor_position_os) and stored as the
 * float core_pos of IVP_Anchor::init_anchor (object -> world via the
 * object's cache matrix, i.e. including the object/core shift -> core).
 * If obj_a or obj_b is NULL, the corresponding anchor is in world space.
 * Break settings, listener and relative factor are reset to defaults. */
void ivp_spring_state_setup(ivp_spring_state_t *state,
                              ivp_object_t *obj_a,
                              const ivp_vec3r_t *anchor_a_os,
                              ivp_object_t *obj_b,
                              const ivp_vec3r_t *anchor_b_os,
                              ivp_real_t stiffness,
                              ivp_real_t damping,
                              ivp_real_t rest_length);

/* Reference spring_values_are_relative: multiply stiffness, damping and
 * rel_pos_damp by (m0*m1)/(m0+m1) where m_i = calc_virt_mass(anchor_i, NULL).
 * A static/world endpoint counts as infinitely heavy (factor = other mass).
 * Call once after setup. */
void ivp_spring_state_make_relative(ivp_spring_state_t *state);

/* Setters (reference: multiplied by spring_values_factor, wake the actuator). */
void ivp_spring_set_constant(ivp_spring_state_t *state, ivp_real_t value);
void ivp_spring_set_damp(ivp_spring_state_t *state, ivp_real_t value);
void ivp_spring_set_rel_pos_damp(ivp_spring_state_t *state, ivp_real_t value);
void ivp_spring_set_len(ivp_spring_state_t *state, ivp_real_t value);
void ivp_spring_set_break_max_len(ivp_spring_state_t *state, ivp_real_t value);
void ivp_spring_set_broken_listener(ivp_spring_state_t *state,
                                    ivp_spring_broken_fn_t fn, void *ctx);

/* Reference ensure_actuator_in_simulation(). */
void ivp_spring_ensure_in_simulation(ivp_spring_state_t *state);

/* Reference IVP_Actuator_Spring_Active (IVP_Controller_Factory::create_spring
 * with template active floats): subscribes to the given active floats (each
 * may be NULL) and applies their current float values in the reference
 * order len, constant, damp, rel_pos_damp through ivp_spring_set_len /
 * set_constant / set_damp / set_rel_pos_damp (constants are multiplied with
 * spring_values_factor, so call after ivp_spring_state_make_relative); later
 * changes call the setter again (which wakes the spring).  Call after setup;
 * previous bindings are released first. */
void ivp_spring_bind_active(ivp_spring_state_t *state,
                            ivp_active_float_t *active_float_spring_len,
                            ivp_active_float_t *active_float_spring_constant,
                            ivp_active_float_t *active_float_spring_damp,
                            ivp_active_float_t *active_float_spring_rel_pos_damp);

/* Reference ~IVP_Actuator_Spring_Active: removes the dependencies. */
void ivp_spring_unbind_active(ivp_spring_state_t *state);

/* Discovery function for ivp_environment_announce_dependent_controller():
 * reports the movable objects of the actuator so that their simulation
 * units are merged (reference IVP_Controller_Dependent semantics). */
int ivp_spring_discovery(void *ctx, ivp_environment_t *env,
               ivp_object_handle_t *out_handles, int max_handles);

/* ── Controller callback ────────────────────────────────────────────────── */

/* The spring controller callback. Pass an ivp_spring_state_t* as ctx.
 * Executes once per PSI (on the first simulated movable endpoint). */
void ivp_controller_spring(const ivp_event_sim_t *event_sim,
                             ivp_object_t *obj,
                             void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTUATOR_SPRING_H */
