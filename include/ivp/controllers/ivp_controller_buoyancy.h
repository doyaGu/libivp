/* ivp_controller_buoyancy.h -- Buoyancy / fluid dynamics controller
 *
 * Port of IVP_Controller_Buoyancy + IVP_Buoyancy_Solver
 * (ivp_controller/ivp_controller_buoyancy.cxx, ivp_buoyancy_solver.cxx):
 *   - polygons: the submerged volume and its center are integrated exactly
 *     from one pyramid per compact-ledge triangle (apex = object center
 *     projected onto the liquid surface), clipping triangles at the surface;
 *   - balls: analytic spherical cap volume/center plus the reference
 *     projected-ellipse dampening model (cases 0..4);
 *   - buoyancy impulse  = -gravity * volume_under * medium_density * dt at
 *     the submerged volume center;
 *   - dampening per submerged triangle: pressure (0.5*rho*pressure_damp)
 *     and friction (0.5*rho*friction_damp) terms along the relative current
 *     (or against the triangle normal with simulate_wing_behavior), applied
 *     at the object center with "impulse x point" and "impulse x move vector"
 *     torques (torque_factor), ball rotation dampening, and the reference
 *     particle-acceleration viscosity model of the water around each core.
 *
 * The liquid surface comes from a liquid surface descriptor
 * (ivp_liquid_surface_t, reference IVP_Liquid_Surface_Descriptor) or, when
 * none is set, from the plane surface_normal/surface_height plus
 * flow_velocity.  Convention of the plane: surface_normal points OUT of the
 * liquid and normal . p = surface_height on the surface.
 *
 * Reference IVP_Template_Buoyancy defaults: medium_density 0.3,
 * pressure_damp_factor 0.1, friction_damp_factor 0.05, torque_factor 1,
 * buoyancy_eps 1e-10, ball_rot_dampening_factor 0.01, viscosity_factor 0.01,
 * viscosity_input_factor 2, use_interpolation false (with interpolation:
 * max_interpolation_tries 10, max_tries_nr_of_vectors_involved 15,
 * use_stochastic_insertion true, insert_extrapol_only true, MI weights 1,
 * max_res 1e-2, nr_future_psi_for_extrapolation 1).  The template values
 * are IVP_FLOAT in the reference and are used as float.  Like the reference
 * build (ivp_physics.hxx defines IVP_NO_MD_INTERPOLATION) the controller
 * always runs the solver; the multidimensional interpolation path
 * (ivp/utility/ivp_multidimensional_interp.h) is only compiled in with
 * -DIVP_MD_INTERPOLATION (CMake option IVP_MD_INTERPOLATION).
 *
 * Attaching (reference IVP_Attacher_To_Cores_Buoyancy): ivp_buoyancy_attacher_create
 * copies the template (this state) and attaches one buoyancy controller
 * (IVP_CP_FORCEFIELDS, per-core data) to every core of an active set of
 * cores through the generic ivp_attacher_to_cores.  libivp extra: the state
 * itself can be registered with ivp_environment_add_controller, acting on
 * every core (optionally filtered by ivp_buoyancy_set_objects); its per-core
 * data is unbounded (the first IVP_BUOYANCY_INLINE_CORES cores are stored
 * inline, more in a heap block released by ivp_buoyancy_state_destroy).
 */
#ifndef IVP_CONTROLLER_BUOYANCY_H
#define IVP_CONTROLLER_BUOYANCY_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/controllers/ivp_liquid_surface.h>
#include <ivp/controllers/ivp_attacher_to_cores.h>
#include <ivp/containers/ivp_set.h>
#include <ivp/dynamics/ivp_handle.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Per-core data stored inline in ivp_buoyancy_state_t (all-cores mode);
 * further cores use a heap block (no limit, reference is unbounded). */
#define IVP_BUOYANCY_INLINE_CORES 64

struct ivp_mi_interpolator;
struct ivp_mi_vector;
struct ivp_buoyancy_attacher;

/* Attacher_Interpolator: multidimensional interpolation data of one object
 * of the core (IVP_Multidimensional_Interpolator(5, ...) and the Last_IO
 * vectors, created on first use). */
typedef struct ivp_buoyancy_object_interpolator {
    const ivp_object_t         *object;
    struct ivp_mi_interpolator *mi;
    struct ivp_mi_vector       *last_input;
    struct ivp_mi_vector       *last_solution;
    double                      last_psi_time;
} ivp_buoyancy_object_interpolator_t;

/* Per-core dynamic data (reference: one IVP_Controller_Buoyancy per core;
 * buoyancy and dampening are computed for every object of the core). */
typedef struct ivp_buoyancy_core_data {
    const ivp_object_t *object;
    ivp_object_handle_t object_handle;  /* entries of deleted objects are recycled */
    ivp_vec3r_t relative_speed_of_current_in_objects_vicinity_old;
    ivp_real_t  core_visible_surface_content_under;
    ivp_real_t  core_visible_surface_content_under_old;
    /* multidimensional interpolation (use_interpolation): attacher_interpolator
     * (one per object of the core, rebuilt when the core's objects change,
     * sync_attacher_interpolator_with_core_objects) and interpolation_counter */
    ivp_buoyancy_object_interpolator_t *interpolators;
    int                                 interpolator_count;
    int                                 interpolation_counter;
} ivp_buoyancy_core_data_t;

/* ── Buoyancy state ────────────────────────────────────────────────────── */

typedef struct ivp_buoyancy_state {
    /* Liquid surface plane (used when liquid_surface is NULL) */
    ivp_vec3r_t surface_normal;    /* unit normal pointing OUT of the liquid */
    ivp_real_t  surface_height;    /* normal . p = surface_height on the surface */
    ivp_vec3r_t flow_velocity;     /* abs_speed_of_current (world space) */

    /* Optional liquid surface descriptor (reference calc_liquid_surface);
     * evaluated at the core position. */
    const ivp_liquid_surface_t *liquid_surface;

    /* IVP_Template_Buoyancy */
    ivp_real_t  fluid_density;     /* medium_density [kg/m^3] */
    ivp_real_t  linear_drag;       /* pressure_damp_factor */
    ivp_real_t  friction_damp_factor;
    ivp_real_t  torque_factor;
    ivp_real_t  buoyancy_eps;
    ivp_real_t  angular_drag;      /* ball_rot_dampening_factor */
    ivp_real_t  viscosity_factor;
    ivp_real_t  viscosity_input_factor;
    bool        simulate_wing_behavior;
    bool        use_interpolation;                 /* ignored unless libivp is built with
                                                    * -DIVP_MD_INTERPOLATION: the reference
                                                    * engine defines IVP_NO_MD_INTERPOLATION */
    int         max_interpolation_tries;           /* 10 */
    int         max_tries_nr_of_vectors_involved;  /* 15 */
    bool        use_stochastic_insertion;          /* true */
    bool        insert_extrapol_only;              /* true */
    ivp_real_t  mi_weight_current_speed;           /* 1 */
    ivp_real_t  mi_weight_surface;                 /* 1 */
    ivp_real_t  mi_weight_rot_speed;               /* 1 */
    ivp_real_t  max_res;                           /* 1e-2 */
    int         nr_future_psi_for_extrapolation;   /* 1 */

    /* libivp extra (all-cores registration): optional static object list;
     * NULL = every core the controller is dispatched for.  The reference
     * set of cores is ivp_buoyancy_attacher_create. */
    ivp_object_t *const *objects;
    int                  object_count;

    /* Per-core dynamic data of the all-cores registration (unbounded):
     * entries [0, IVP_BUOYANCY_INLINE_CORES) inline, the rest in
     * cores_overflow. */
    ivp_buoyancy_core_data_t  cores[IVP_BUOYANCY_INLINE_CORES];
    ivp_buoyancy_core_data_t *cores_overflow;
    int                       cores_overflow_capacity;
    int                       core_count;

    /* Set when this state is the template copy of an attacher (the
     * per-core data then belongs to the attachments). */
    struct ivp_buoyancy_attacher *attacher;

    bool active;
} ivp_buoyancy_state_t;

/* Reference IVP_Attacher_To_Cores_Buoyancy. */
typedef struct ivp_buoyancy_attacher ivp_buoyancy_attacher_t;

/* Called after an attacher deleted itself because its set was deleted
 * (the attacher memory is freed right after the call). */
typedef void (*ivp_buoyancy_attacher_deleted_fn_t)(void *ctx, ivp_buoyancy_attacher_t *attacher);

struct ivp_buoyancy_attacher {
    ivp_buoyancy_state_t      template_buoyancy;   /* copy of the template */
    ivp_attacher_to_cores_t   attacher;            /* IVP_Attacher_To_Cores<IVP_Controller_Buoyancy> */
    ivp_environment_t        *environment;
    ivp_controller_handle_t   controller;          /* the buoyancy controllers of all cores */
    ivp_buoyancy_attacher_deleted_fn_t deleted_fn;
    void                              *deleted_ctx;
};

/* ── Initialization ────────────────────────────────────────────────────── */

/* Reference template defaults, liquid surface at Y = surface_y in a Y-down
 * world (liquid for y > surface_y), medium density fluid_density. */
void ivp_buoyancy_init(ivp_buoyancy_state_t *state,
                         ivp_real_t surface_y,
                         ivp_real_t fluid_density);

/* Initialize buoyancy with custom plane (normal out of the liquid). */
void ivp_buoyancy_init_plane(ivp_buoyancy_state_t *state,
                               const ivp_vec3r_t *normal,
                               ivp_real_t height,
                               ivp_real_t fluid_density);

/* Reference IVP_Liquid_Surface_Descriptor wiring. */
void ivp_buoyancy_set_liquid_surface(ivp_buoyancy_state_t *state,
                                     const ivp_liquid_surface_t *surface);

/* libivp extra: restrict an all-cores registration to a static object list. */
void ivp_buoyancy_set_objects(ivp_buoyancy_state_t *state,
                              ivp_object_t *const *objects, int count);

/* Free the per-core data of an all-cores registration (interpolation data,
 * cores beyond IVP_BUOYANCY_INLINE_CORES). */
void ivp_buoyancy_state_destroy(ivp_buoyancy_state_t *state);

/* ── Reference IVP_Attacher_To_Cores_Buoyancy ──────────────────────────── */

/* IVP_Attacher_To_Cores_Buoyancy(templ, set_of_cores, liquid_surface):
 * copies templ (parameters and liquid surface descriptor, which must stay
 * valid) and attaches a buoyancy controller to every core of set_of_cores
 * (ivp_core_t* elements), following the set from then on.  When the set
 * is deleted the attacher deletes itself.  NULL on failure. */
ivp_buoyancy_attacher_t *ivp_buoyancy_attacher_create(ivp_environment_t *env,
                                                      const ivp_buoyancy_state_t *templ,
                                                      ivp_set_active_t *set_of_cores);

/* Detach from all cores and free the attacher (libivp: the reference
 * attacher is only deleted together with its set). */
void ivp_buoyancy_attacher_destroy(ivp_buoyancy_attacher_t *attacher);

/* Reference get_parameters_per_core: the template copy used for every core
 * (may be modified). */
static inline ivp_buoyancy_state_t *ivp_buoyancy_attacher_get_template(ivp_buoyancy_attacher_t *attacher) {
    return attacher ? &attacher->template_buoyancy : NULL;
}

/* Hook called when the attacher deletes itself with its set. */
void ivp_buoyancy_attacher_set_deleted_listener(ivp_buoyancy_attacher_t *attacher,
                                                ivp_buoyancy_attacher_deleted_fn_t fn, void *ctx);

/* Per-core data of a core (NULL if the core is not attached). */
ivp_buoyancy_core_data_t *ivp_buoyancy_attacher_get_core_data(ivp_buoyancy_attacher_t *attacher,
                                                              const ivp_core_t *core);

/* ── Controller callback ───────────────────────────────────────────────── */

/* Pass ivp_buoyancy_state_t* as ctx. Priority IVP_CP_FORCEFIELDS (async). */
void ivp_controller_buoyancy(const ivp_event_sim_t *event_sim,
                               ivp_object_t *obj,
                               void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTROLLER_BUOYANCY_H */
