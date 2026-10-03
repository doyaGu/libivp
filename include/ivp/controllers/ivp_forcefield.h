/* ivp_forcefield.h -- Force field controller
 *
 * Applies a constant or position-dependent force to objects.
 * Useful for custom gravity regions, wind, buoyancy approximation, etc.
 *
 * Reference IVP_Forcefield (ivp_forcefield.cxx) is an abstract
 * IVP_Controller_Independent driven by an IVP_U_Set_Active<IVP_Core>,
 * priority IVP_CP_ACTUATOR (1500, asynchronous):
 *   - every core of the set gets the field (add_controller_to_core), cores
 *     added to / removed from the set later are attached / detached;
 *   - a deleted core is removed from the set (which detaches it);
 *   - when the set is deleted the field deletes itself.
 * This is ivp_forcefield_attach_set (set_of_cores holds ivp_core_t*).
 * The force law is the reference do_simulation_controller override: the
 * libivp CONSTANT/FORCE/DRAG laws or a CUSTOM law.
 *
 * libivp extras (no reference counterpart): registering the state with
 * ivp_environment_add_controller attaches it to every core, optionally
 * filtered by a static object list (ivp_forcefield_set_objects).
 *
 * Ported from ivp_forcefield.hxx.
 */
#ifndef IVP_FORCEFIELD_H
#define IVP_FORCEFIELD_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/containers/ivp_set.h>
#include <ivp/containers/ivp_pvec.h>
#include <ivp/events/ivp_listener_object.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Force field mode ───────────────────────────────────────────────────── */

typedef enum ivp_forcefield_mode {
    IVP_FORCEFIELD_CONSTANT     = 0,  /* constant acceleration (like gravity) */
    IVP_FORCEFIELD_FORCE        = 1,  /* constant force (mass-dependent) */
    IVP_FORCEFIELD_DRAG         = 2,  /* velocity-proportional drag */
    IVP_FORCEFIELD_CUSTOM       = 3   /* user law (reference subclass) */
} ivp_forcefield_mode_t;

/* ── Force field state ──────────────────────────────────────────────────── */

/* Custom law: called once per PSI for every core of the field. */
typedef void (*ivp_forcefield_custom_fn_t)(void *ctx,
                                           const ivp_event_sim_t *event_sim,
                                           ivp_object_t *obj);

struct ivp_forcefield_state;

/* Called after a set-driven field removed itself because its set was
 * deleted (reference P_DELETE_THIS in pset_is_going_to_be_deleted). */
typedef void (*ivp_forcefield_deleted_fn_t)(void *ctx, struct ivp_forcefield_state *field);

typedef struct ivp_forcefield_state {
    ivp_forcefield_mode_t mode;
    ivp_vec3r_t           value;       /* acceleration (m/s²), force (N),
                                          or drag coefficient */
    bool                  active;      /* on/off toggle */

    /* libivp extra (ivp_environment_add_controller registration): optional
     * static object list; NULL = every dispatched core. The array is owned
     * by the caller. */
    ivp_object_t *const  *objects;
    int                   object_count;

    /* IVP_FORCEFIELD_CUSTOM */
    ivp_forcefield_custom_fn_t custom_fn;
    void                      *custom_ctx;

    /* Reference set-driven IVP_Forcefield (ivp_forcefield_attach_set) */
    ivp_environment_t        *environment;      /* NULL when not set-driven / env deleted */
    ivp_set_active_t         *set_of_cores;
    bool                      i_am_owner_of_set_of_cores; /* stored only (as the reference) */
    bool                      set_is_going_to_be_deleted;
    bool                      set_driven;
    ivp_pvec_t                attached_cores;   /* ivp_core_t* */
    ivp_controller_handle_t   controller;
    ivp_set_active_listener_t set_listener;
    ivp_listener_object_t     object_listener;
    ivp_forcefield_deleted_fn_t deleted_fn;
    void                       *deleted_ctx;
} ivp_forcefield_state_t;

/* ── Initialization ─────────────────────────────────────────────────────── */

/* Initialize with a constant acceleration (like gravity). */
void ivp_forcefield_state_init(ivp_forcefield_state_t *state,
                                ivp_forcefield_mode_t mode,
                                const ivp_vec3r_t *value);

/* Restrict the field to a set of objects (NULL / 0 = all cores). */
void ivp_forcefield_set_objects(ivp_forcefield_state_t *state,
                                ivp_object_t *const *objects, int count);

/* Use a custom law (switches mode to IVP_FORCEFIELD_CUSTOM). */
void ivp_forcefield_set_custom(ivp_forcefield_state_t *state,
                               ivp_forcefield_custom_fn_t fn, void *ctx);

/* ── Reference set-driven field ─────────────────────────────────────────── */

/* Reference constructor IVP_Forcefield(env, set_of_cores, owner_of_set):
 * registers the field as an independent controller (IVP_CP_ACTUATOR) of
 * every core in set_of_cores (ivp_core_t* elements; frozen cores are not
 * woken) and follows the set from then on.  Do NOT also register the state
 * with ivp_environment_add_controller.  Call after ivp_forcefield_state_init;
 * returns false on bad arguments / no memory. */
bool ivp_forcefield_attach_set(ivp_forcefield_state_t *state,
                               ivp_environment_t *env,
                               ivp_set_active_t *set_of_cores,
                               bool owner_of_set);

/* Hook called after the field removed itself because its set was
 * deleted (the state may be freed in the hook). */
void ivp_forcefield_set_deleted_listener(ivp_forcefield_state_t *state,
                                         ivp_forcefield_deleted_fn_t fn, void *ctx);

/* Reference ~IVP_Forcefield: detaches the field from all cores, stops
 * listening to the set and unregisters the controller.  Safe after the
 * environment or the set were deleted, and for fields that are not
 * set-driven (no-op). */
void ivp_forcefield_destroy(ivp_forcefield_state_t *state);

/* ── Controller callback ────────────────────────────────────────────────── */

/* Force field controller. Pass ivp_forcefield_state_t* as ctx.
 * Registered at IVP_CP_ACTUATOR (1500), as the reference IVP_Forcefield. */
void ivp_controller_forcefield(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_FORCEFIELD_H */
