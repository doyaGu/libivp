/* ivp_actuator_check_dist.h -- Distance checking actuator
 *
 * Monitors the distance between two anchor points (fixed in the objects'
 * spaces) and fires an event when it crosses the range.
 * Reference semantics (IVP_Actuator_Check_Dist, ivp_actuator.cxx:283-371):
 *   len < range  -> inside  (is_outside = 0)
 *   len >= range -> outside (is_outside = 1)
 * The initial state is evaluated at setup without notifying listeners (the
 * reference evaluates it in the constructor, before any listener exists).
 *
 * Ported from IVP_Actuator_Check_Dist (ivp_actuator.hxx).
 */
#ifndef IVP_ACTUATOR_CHECK_DIST_H
#define IVP_ACTUATOR_CHECK_DIST_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/collision/ivp_hull_mgr.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_active_int;
struct ivp_check_dist_state;

/* IVP_Anchor_Check_Dist: a lazy hull synapse on an anchor object */
typedef struct ivp_check_dist_anchor {
    ivp_listener_hull_t          lh;          /* must be first */
    struct ivp_check_dist_state *state;
    ivp_object_t                *real_object;
    ivp_vec3f_t                  object_pos;  /* IVP_U_Float_Point, object space */
} ivp_check_dist_anchor_t;

/* IVP_Listener_Check_Dist_Event::check_dist_is_going_to_be_deleted_event */
typedef void (*ivp_check_dist_deleted_fn_t)(void *ctx, struct ivp_check_dist_state *state);

/* Callback when distance status changes */
typedef void (*ivp_check_dist_event_fn_t)(void *ctx,
                                           ivp_object_t *obj_a,
                                           ivp_object_t *obj_b,
                                           ivp_real_t distance,
                                           bool is_inside_range);

/* ── Check distance state ───────────────────────────────────────────────── */

typedef struct ivp_check_dist_state {
    /* Objects to monitor (NULL = world) */
    ivp_object_t *obj_a;
    ivp_object_t *obj_b;

    /* World positions given at setup (used directly for NULL objects) */
    ivp_vec3r_t position_a_ws;
    ivp_vec3r_t position_b_ws;

    /* Anchors in object (core) space, computed at setup */
    ivp_vec3r_t anchor_a_os;
    ivp_vec3r_t anchor_b_os;

    /* Configuration */
    ivp_real_t range;                   /* distance threshold */
    bool       use_object_centers;      /* libivp extra: measure CoM distance (attached
                                         * mode: the anchors are placed at the mass
                                         * centres when ivp_check_dist_attach runs) */

    /* Result state */
    ivp_real_t current_distance;
    int        is_outside;              /* 1 = outside range, 0 = inside */
    int        prev_is_outside;         /* for change detection */

    /* Optional event callback */
    ivp_check_dist_event_fn_t event_fn;
    void                     *event_ctx;
    ivp_check_dist_deleted_fn_t deleted_fn;   /* actuator deleted (detach / object deleted) */

    /* IVP_Template_Check_Dist::mod_is_outside: set (delayed) on every evaluation */
    struct ivp_active_int    *mod_is_outside;

    bool       active;

    /* reference event driven mode (ivp_check_dist_attach) */
    bool                     attached;
    struct ivp_environment  *env;
    ivp_check_dist_anchor_t  anchors[2];
} ivp_check_dist_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

void ivp_check_dist_state_init(ivp_check_dist_state_t *state);

void ivp_check_dist_state_setup(ivp_check_dist_state_t *state,
                                  ivp_object_t *obj_a,
                                  const ivp_vec3r_t *pos_a_ws,
                                  ivp_object_t *obj_b,
                                  const ivp_vec3r_t *pos_b_ws,
                                  ivp_real_t range);

/* Reference IVP_Actuator_Check_Dist::set_range: re-evaluates immediately
 * and notifies the callback on a state change. */
void ivp_check_dist_set_range(ivp_check_dist_state_t *state, ivp_real_t range);

/* IVP_Actuator_Check_Dist constructor: insert lazy hull synapses on both
 * anchor objects (NULL = the environment static object) and evaluate; the
 * distance is then re-evaluated whenever an anchor object's hull manager
 * reports that it may have moved by |len - range| / 2, exactly like the
 * reference.  ivp_environment_add_controller(ivp_controller_check_dist)
 * attaches automatically and ivp_environment_remove_controller detaches.
 * Deleting an anchor object detaches (reference: the actuator is deleted)
 * and calls deleted_fn. */
bool ivp_check_dist_attach(ivp_check_dist_state_t *state, struct ivp_environment *env);
/* ~IVP_Actuator_Check_Dist: calls deleted_fn and removes the synapses */
void ivp_check_dist_detach(ivp_check_dist_state_t *state);

/* Set event callback for distance threshold crossing */
void ivp_check_dist_set_callback(ivp_check_dist_state_t *state,
                                   ivp_check_dist_event_fn_t fn,
                                   void *ctx);

/* ── Controller callback ────────────────────────────────────────────────── */

/* When the state is attached this does nothing (hull events drive it);
 * otherwise it polls once per PSI (libivp fallback). */
void ivp_controller_check_dist(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTUATOR_CHECK_DIST_H */
