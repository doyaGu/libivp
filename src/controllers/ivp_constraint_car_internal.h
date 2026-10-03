/* ivp_constraint_car_internal.h -- private layout of the car constraint
 * solver (IVP_Constraint_Solver_Car) and the reference-precision
 * IVP_Solver_Core_Reaction used by the car code.
 *
 * Shared by ivp_constraint_car.c (solver), ivp_car_wheel_physics.c (fast
 * wheel rotation, 2D wheel friction) and ivp_car_system.c.
 *
 * Precision: IVP_U_Float_Point -> ivp_vec3f_t (lvf_*), IVP_U_Point /
 * IVP_U_Matrix -> ivp_vec3r_t / ivp_transform_r_t (lvp_* / lvm_*),
 * IVP_DOUBLE -> D (ivp_real_t), IVP_FLOAT -> float.
 */
#ifndef IVP_CONSTRAINT_CAR_INTERNAL_H
#define IVP_CONSTRAINT_CAR_INTERNAL_H

#include <ivp/controllers/ivp_constraint_car.h>
#include <ivp/solver/ivp_great_matrix.h>
#include <ivp/dynamics/ivp_object.h>
#include "collision/ivp_lvec.h"
#include "dynamics/ivp_sim_internal.h"

struct ivp_constraint_state;

/* IVP_Constraint_Solver_Car (an IVP_Controller_Dependent) */
struct ivp_constraint_solver_car {
    ivp_sim_ctrl_t               base;                 /* must stay first */
    ivp_constraint_car_object_t *body_object;
    ivp_constraint_car_object_t *wheel_objects[IVP_CONSTRAINT_CAR_MAX_WHEELS];
    int                          n_wheel_objects;
    ivp_gmmz_t                   co_matrix;            /* inverted push matrix, all in B */
    bool                         co_matrix_allocated;
    int                          x_idx;                /* right   */
    int                          y_idx;                /* "up" (suspension axis) */
    int                          z_idx;                /* forward */
    float                        angle_sign;           /* 1 or -1 (right / left handed) */

    ivp_pvec_t                   cores_of_constraint_system; /* ivp_core_t*: body, wheels */
    bool                         constraint_is_disabled[6];
    bool                         local_translation_in_use;   /* plan B */
    struct ivp_constraint_state *c_local_ballsocket[IVP_CONSTRAINT_CAR_MAX_WHEELS];
    ivp_controller_handle_t      c_local_ballsocket_handle[IVP_CONSTRAINT_CAR_MAX_WHEELS];
    ivp_environment_t           *environment;
    int                          psis_left_for_plan_B;
    float                        max_delta_speed;

    /* libivp lifetime bookkeeping */
    bool                         announced;            /* controller is in the environment */
    bool                         environment_dying;    /* IVP_EVENT_ENVIRONMENT_DELETED seen */
    ivp_event_listener_handle_t  env_listener;
};

/* ── reference IVP_Solver_Core_Reaction (float vectors, IVP_U_Point_4) ──── */

typedef struct ivp_car_point4 {   /* IVP_U_Point_4 : IVP_U_Float_Hesse */
    ivp_vec3f_t v;
    float       hesse_val;
} ivp_car_point4_t;

typedef struct ivp_car_scr {
    const ivp_vec3f_t *direction_ws[3];
    ivp_car_point4_t   cross_direction_position_cs0[3];
    ivp_car_point4_t   cross_direction_position_cs1[3];
    ivp_car_point4_t   cr_mult_inv0[3];
    ivp_car_point4_t   cr_mult_inv1[3];
    D                  m[3][3];          /* m_velocity_ds_f_impulse_ds (upper triangle) */
    ivp_vec3f_t        delta_velocity_ds;
} ivp_car_scr_t;

void ivp_car_scr_init_translation_ws(ivp_car_scr_t *s, ivp_core_t *core_0, ivp_core_t *core_1,
                                     const ivp_vec3r_t *pos_ws, const ivp_vec3f_t *direction_0_ws,
                                     const ivp_vec3f_t *direction_1_ws, const ivp_vec3f_t *direction_2_ws);
void ivp_car_scr_init_rotation_ws(ivp_car_scr_t *s, ivp_core_t *core_0, ivp_core_t *core_1,
                                  const ivp_vec3f_t *direction_0_ws, const ivp_vec3f_t *direction_1_ws,
                                  const ivp_vec3f_t *direction_2_ws);
void ivp_car_scr_exert_impulse_dim2(const ivp_car_scr_t *s, ivp_core_t *core_0, ivp_core_t *core_1,
                                    const ivp_vec3f_t *impulse_ds);
void ivp_car_scr_exert_angular_impulse_dim2(const ivp_car_scr_t *s, ivp_core_t *core_0, ivp_core_t *core_1,
                                            const ivp_vec3f_t *impulse_ds);

/* IVP_Inline_Math::invert_2x2_matrix */
static inline bool ivp_car_invert_2x2_matrix(D a1, D b1, D a2, D b2, D *i_a1, D *i_b1, D *i_a2, D *i_b2)
{
    D det = (a1 * b2) - (b1 * a2);
    if (det * det < (D)P_DOUBLE_EPS * (D)P_DOUBLE_EPS) return false;
    D di = 1.0f / det;
    *i_a1 = b2 * di;
    *i_b1 = -a2 * di;
    *i_a2 = -b1 * di;
    *i_b2 = a1 * di;
    return true;
}

/* IVP_Real_Object::calc_m_core_f_object */
void ivp_car_calc_m_core_f_object(const ivp_object_t *obj, ivp_transform_r_t *m_core_f_object);

/* car object of a core (core->car_wheel) */
static inline ivp_constraint_car_object_t *ivp_car_object_of_core(const ivp_core_t *core)
{
    return (ivp_constraint_car_object_t *)core->car_wheel;
}

#endif /* IVP_CONSTRAINT_CAR_INTERNAL_H */
