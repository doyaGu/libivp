/* ivp_solver_core_reaction.h — Two-core reaction matrix solver
 *
 * Port of IVP_Solver_Core_Reaction (ivp_physics/ivp_solver_core_reaction.cxx).
 *
 * Builds the symmetric matrix that maps impulses applied along up to three
 * orthonormal world directions (at a world point for the translational
 * variant, or as pure angular impulses for the rotational variant) to the
 * resulting relative velocity change between two cores, plus the current
 * relative velocity along those directions.  Either core may be NULL, which
 * stands for a static (infinitely heavy) partner.
 *
 * Sign convention (reference): impulses are applied +impulse on core_0 and
 * -impulse on core_1; delta_velocity_ds = v0·d - v1·d.
 *
 * Results are bit-identical to the reference: directions and impulses are
 * used as IVP_U_Float_Point (rounded to float), the cross_direction /
 * cr_mult_inv points and delta_velocity_ds hold the reference's float values,
 * the matrix m is the reference's double IVP_U_Matrix3 (accumulated from
 * float dot products) and the core speeds are pushed with float stores.
 */
#ifndef IVP_SOLVER_CORE_REACTION_H
#define IVP_SOLVER_CORE_REACTION_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/math/ivp_vec3.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_core;

typedef struct ivp_scr_point4 {
    double k[3];
    double w;      /* reference hesse_val: 1 for translation, 0 for rotation */
} ivp_scr_point4_t;

typedef struct ivp_solver_core_reaction {
    const ivp_vec3r_t *direction_ws[3];
    ivp_scr_point4_t   cross_direction_position_cs0[3];
    ivp_scr_point4_t   cross_direction_position_cs1[3];
    ivp_scr_point4_t   cr_mult_inv0[3];
    ivp_scr_point4_t   cr_mult_inv1[3];

    /* m_velocity_ds_f_impulse_ds (upper triangle filled by init_*;
     * full after invert_3x3) */
    double m[3][3];
    double delta_velocity_ds[3];
} ivp_solver_core_reaction_t;

/* Directions must be normalized and mutually orthogonal.  direction_1 /
 * direction_2 may be NULL (fewer dimensions).  The direction pointers are
 * stored and must stay valid until the exert_* call. */
void ivp_scr_init_translation_ws(ivp_solver_core_reaction_t *s,
                                 struct ivp_core *core_0, struct ivp_core *core_1,
                                 const ivp_vec3r_t *pos_ws,
                                 const ivp_vec3r_t *direction_0_ws,
                                 const ivp_vec3r_t *direction_1_ws,
                                 const ivp_vec3r_t *direction_2_ws);

void ivp_scr_init_rotation_ws(ivp_solver_core_reaction_t *s,
                              struct ivp_core *core_0, struct ivp_core *core_1,
                              const ivp_vec3r_t *direction_0_ws,
                              const ivp_vec3r_t *direction_1_ws,
                              const ivp_vec3r_t *direction_2_ws);

/* Mirror the upper triangle and invert in place (full 3x3).
 * Returns IVP_ERR_DEGENERATE_GEOMETRY if singular. */
ivp_result_t ivp_scr_invert_3x3(ivp_solver_core_reaction_t *s);

/* Invert the leading 2x2 block in place (reference uses
 * IVP_U_Matrix3::real_invert on a matrix whose third row/col is unit).
 * Returns IVP_ERR_DEGENERATE_GEOMETRY if singular. */
ivp_result_t ivp_scr_invert_2x2(ivp_solver_core_reaction_t *s);

void ivp_scr_exert_impulse_dim1(const ivp_solver_core_reaction_t *s,
                                struct ivp_core *core_0, struct ivp_core *core_1,
                                const double impulse_ds[3]);
void ivp_scr_exert_impulse_dim2(const ivp_solver_core_reaction_t *s,
                                struct ivp_core *core_0, struct ivp_core *core_1,
                                const double impulse_ds[3]);
void ivp_scr_exert_impulse_dim3(const ivp_solver_core_reaction_t *s,
                                struct ivp_core *core_0, struct ivp_core *core_1,
                                const double impulse_ds[3]);

void ivp_scr_exert_angular_impulse_dim1(const ivp_solver_core_reaction_t *s,
                                        struct ivp_core *core_0, struct ivp_core *core_1,
                                        const double impulse_rs[3]);
void ivp_scr_exert_angular_impulse_dim2(const ivp_solver_core_reaction_t *s,
                                        struct ivp_core *core_0, struct ivp_core *core_1,
                                        const double impulse_rs[3]);
void ivp_scr_exert_angular_impulse_dim3(const ivp_solver_core_reaction_t *s,
                                        struct ivp_core *core_0, struct ivp_core *core_1,
                                        const double impulse_rs[3]);

/* Out = M * v (full matrix) */
static inline void ivp_scr_mult(const ivp_solver_core_reaction_t *s,
                                const double v[3], double out[3]) {
    for (int r = 0; r < 3; r++)
        out[r] = s->m[r][0] * v[0] + s->m[r][1] * v[1] + s->m[r][2] * v[2];
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_SOLVER_CORE_REACTION_H */
