/* ivp_constraint_types.h -- Shared constraint types and the reference
 *                           IVP_Constraint_Local solver core
 *
 * Port of ivp_controller/ivp_constraint_types.hxx,
 * ivp_template_constraint.{hxx,cxx} and the solver of
 * ivp_constraint_local.cxx (IVP_Constraint_Local::do_simulation_controller).
 *
 * Every PSI the solver builds a dense matrix of the relative velocity
 * response of all fixed axes (translation in the reference frame "fs",
 * rotation in "rs") to unit test impulses, solves it with Gauss
 * elimination (IVP_Great_Matrix_Many_Zero, MATRIX_EPS 1e-9) for the
 * impulses that remove force_factor of the positional / angular error
 * (plus damp_factor of the relative velocity) within one PSI, applies the
 * MaxImpulse CLIP / BREAK / BEND rules and pushes both cores synchronously.
 *
 * Both ivp_constraint.h (ball socket / hinge / fixed / cardan / ...) and
 * ivp_constraint_local.h (per-axis configurable) are thin wrappers around
 * this core, so both headers can be included together.
 */
#ifndef IVP_CONSTRAINT_TYPES_H
#define IVP_CONSTRAINT_TYPES_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/controllers/ivp_controller.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reference IVP_CONSTRAINT_AXIS_TYPE (bit values matter: a LIMITED axis
 * that hits a border gets the FIXED bit set for that PSI). */
typedef enum ivp_constraint_axis_type {
    IVP_CONSTRAINT_AXIS_FREE    = 0x0,
    IVP_CONSTRAINT_AXIS_FIXED   = 0x1,
    IVP_CONSTRAINT_AXIS_LIMITED = 0x2
} ivp_constraint_axis_type_t;

/* Reference IVP_CONSTRAINT_FORCE_EXCEED (reference ordering). */
typedef enum ivp_constraint_force_exceed {
    IVP_CFE_NONE  = 0,  /* no limit */
    IVP_CFE_CLIP  = 1,  /* translation: scale impulses (reference: by
                         * dt * maximpulse / |impulse|); rotation: no-op */
    IVP_CFE_BREAK = 2,  /* deactivate the constraint + broken event */
    IVP_CFE_BEND  = 3   /* reference: not implemented (no effect) */
} ivp_constraint_force_exceed_t;

/* Reference IVP_NORM (IVP_Constraint_Local::init always selects MAXIMUM). */
typedef enum ivp_constraint_norm {
    IVP_NORM_MINIMUM  = 0,
    IVP_NORM_EUCLIDIC = 1,
    IVP_NORM_MAXIMUM  = 2
} ivp_constraint_norm_t;

/* Reference IVP_U_Matrix (double): p_out = rows * p + vv. */
typedef struct ivp_cmatrix {
    double rows[3][3];
    double vv[3];
} ivp_cmatrix_t;

void ivp_cmatrix_identity(ivp_cmatrix_t *m);

/* ── Template (reference IVP_Template_Constraint) ──────────────────────── */

typedef struct ivp_constraint_template {
    ivp_object_t *obj_r;              /* reference object (NULL = world) */
    ivp_object_t *obj_a;              /* attached object  (NULL = world) */

    bool          has_m_Ros_f_Rfs;    /* translation frame in R object space */
    ivp_cmatrix_t m_Ros_f_Rfs;
    bool          has_m_Ros_f_Rrs;    /* separate rotation frame (rows used) */
    ivp_cmatrix_t m_Ros_f_Rrs;
    bool          has_m_Aos_f_Afs;    /* attached frame; false = relaxed */
    ivp_cmatrix_t m_Aos_f_Afs;

    ivp_constraint_axis_type_t    axis_type[6];  /* 0..2 translation, 3..5 rotation */
    ivp_real_t                    borderleft_Rfs[6];
    ivp_real_t                    borderright_Rfs[6];
    ivp_real_t                    maximpulse[6];
    ivp_constraint_force_exceed_t maximpulse_type[6];

    ivp_real_t force_factor;            /* default 1.0 */
    ivp_real_t damp_factor;             /* default 1.0 */
    ivp_real_t limited_axis_stiffness;  /* default 0.3 */
} ivp_constraint_template_t;

/* Reference defaults: translation FIXED, rotation FREE, factors 1/1, 0.3. */
void ivp_constraint_template_init(ivp_constraint_template_t *t);

/* Reference set_constraint_Ros / set_constraint_ws.  known_axis is the hinge
 * free axis (rotdim/transdim == 2, mapped to Z) or the fixed axis
 * (rotdim/transdim == 1, mapped to X).  m_Rfs_f_Afs (may be NULL = relaxed)
 * places the attached frame relative to the reference frame. */
void ivp_constraint_template_set_constraint_Ros(ivp_constraint_template_t *t,
                                                ivp_object_t *obj_r,
                                                const ivp_vec3r_t *anchor_Ros,
                                                const ivp_vec3r_t *known_axis_Ros,
                                                unsigned transdim, unsigned rotdim,
                                                ivp_object_t *obj_a,
                                                const ivp_cmatrix_t *m_Rfs_f_Afs);
void ivp_constraint_template_set_constraint_ws(ivp_constraint_template_t *t,
                                               ivp_object_t *obj_r,
                                               const ivp_vec3r_t *anchor_ws,
                                               const ivp_vec3r_t *known_axis_ws,
                                               unsigned transdim, unsigned rotdim,
                                               ivp_object_t *obj_a,
                                               const ivp_cmatrix_t *m_Rfs_f_Afs);

/* Reference convenience setters. */
void ivp_constraint_template_set_ballsocket_ws(ivp_constraint_template_t *t, ivp_object_t *obj_r,
                                               const ivp_vec3r_t *anchor_ws, ivp_object_t *obj_a);
void ivp_constraint_template_set_ballsocket_tense_Ros(ivp_constraint_template_t *t, ivp_object_t *obj_r,
                                                      const ivp_vec3r_t *anchor_Ros, ivp_object_t *obj_a,
                                                      const ivp_vec3r_t *distanceAR_Ros);
void ivp_constraint_template_set_cardanjoint_ws(ivp_constraint_template_t *t, ivp_object_t *obj_r,
                                                const ivp_vec3r_t *anchor_ws,
                                                const ivp_vec3r_t *fixed_axis_ws, ivp_object_t *obj_a);
void ivp_constraint_template_set_hinge_ws(ivp_constraint_template_t *t, ivp_object_t *obj_r,
                                          const ivp_vec3r_t *anchor_ws,
                                          const ivp_vec3r_t *free_axis_ws, ivp_object_t *obj_a);
void ivp_constraint_template_set_fixed(ivp_constraint_template_t *t, ivp_object_t *obj_r,
                                       ivp_object_t *obj_a);
void ivp_constraint_template_set_orientation(ivp_constraint_template_t *t, ivp_object_t *obj_r,
                                             ivp_object_t *obj_a);

/* Reference set_attached_fixing_point_Aos / set_attached_translation_axes_Aos
 * combined: place the attached frame at anchor_Aos.  The frame axes follow
 * the current relative pose (no orientation error); when axis_Aos is given
 * and disagrees with that pose, column axis_index of the frame is set to it
 * (init_normized3_col).  Call after set_constraint_*. */
void ivp_constraint_template_set_attached_Aos(ivp_constraint_template_t *t,
                                              const ivp_vec3r_t *anchor_Aos,
                                              const ivp_vec3r_t *axis_Aos,
                                              int axis_index);

/* Reference limit_translation_axis / limit_rotation_axis (template). */
void ivp_constraint_template_limit_axis(ivp_constraint_template_t *t, int axis6,
                                        ivp_real_t border_left, ivp_real_t border_right);

/* ── Solver core (reference IVP_Constraint_Local) ──────────────────────── */

typedef struct ivp_constraint_local_core {
    ivp_object_t *obj_r;
    ivp_object_t *obj_a;

    ivp_cmatrix_t m_Rfs_f_Rcs;
    ivp_cmatrix_t m_Afs_f_Acs;
    bool          has_rot;              /* separate rotation frames */
    double        m_Rrs_f_Rcs[3][3];
    double        m_Ars_f_Acs[3][3];

    int           fixed[6];             /* ivp_constraint_axis_type_t bits */
    ivp_real_t    borderleft_Rfs[6];
    ivp_real_t    borderright_Rfs[6];
    ivp_real_t    limited_axis_stiffness;
    bool          use_maxforce;
    ivp_real_t    halfimpulse[6];
    ivp_constraint_force_exceed_t maxforce_type[6];

    int           mapping_uRfs_f_Rfs[3];
    int           mapping_uRrs_f_Rrs[3];
    int           fixedtrans_dim, fixedrot_dim;
    int           limitedtrans_dim, limitedrot_dim;
    int           matrix_size;
    ivp_constraint_norm_t norm;

    ivp_real_t    force_factor;
    ivp_real_t    damp_factor_div_force;

    bool          enabled;
    bool          broken;               /* BREAK fired (constraint deactivated) */
    ivp_real_t    last_impulse[6];      /* impulses of the last PSI (R side) */
} ivp_constraint_local_core_t;

/* Reference IVP_Constraint_Local::init (uses the objects' current poses). */
void ivp_constraint_local_core_init(ivp_constraint_local_core_t *core,
                                    const ivp_constraint_template_t *tmpl);

/* Re-derive the axis mappings after changing core->fixed[] directly. */
void ivp_constraint_local_core_sort_mappings(ivp_constraint_local_core_t *core);

/* Set force/damp factors (reference: damp_factor_div_force = damp / force). */
void ivp_constraint_local_core_set_factors(ivp_constraint_local_core_t *core,
                                           ivp_real_t force_factor,
                                           ivp_real_t damp_factor);

/* One PSI of the reference solver.  Returns true when the constraint broke
 * during this call (BREAK); the caller fires the broken event. */
bool ivp_constraint_local_core_simulate(ivp_constraint_local_core_t *core,
                                        const ivp_event_sim_t *event_sim);

/* Reference change_Aos_to_relaxe_constraint / change_Ros_to_relaxe_constraint. */
void ivp_constraint_local_core_relax_attached(ivp_constraint_local_core_t *core);
void ivp_constraint_local_core_relax_reference(ivp_constraint_local_core_t *core);

/* Reference ensure_in_simulation of the constraint objects. */
void ivp_constraint_local_core_wake(ivp_constraint_local_core_t *core);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONSTRAINT_TYPES_H */
