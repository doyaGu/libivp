/* ivp_3d_solver.h — 3D Solver for continuous collision detection
 *
 * Time-of-impact solver using interval search and secant/bisection
 * root finding. Uses matrix cache for efficient sub-PSI transform
 * evaluation.
 *
 * Ported from IVP_3D_Solver.
 */
#ifndef IVP_3D_SOLVER_H
#define IVP_3D_SOLVER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/collision/ivp_matrix_cache.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Distance function callback ────────────────────────────────────────── */

/* Returns the distance metric between two objects given their transforms.
 * Positive = separated, negative = penetrating. */
typedef ivp_real_t (*ivp_3d_distance_fn)(const ivp_transform_r_t *m_a,
                                           const ivp_transform_r_t *m_b,
                                           void *user_data);

/* ── 3D Solver ─────────────────────────────────────────────────────────── */

typedef struct ivp_3d_solver {
    ivp_3d_distance_fn distance_fn;
    void              *user_data;
    ivp_real_t         max_deviation;
    ivp_real_t         inv_max_deviation;
} ivp_3d_solver_t;

/* ── API ───────────────────────────────────────────────────────────────── */

void ivp_3d_solver_init(ivp_3d_solver_t *solver,
                           ivp_3d_distance_fn fn,
                           void *user_data,
                           ivp_real_t max_deviation);

/* Find first t in [t_now, t_max] where distance crosses threshold.
 * Returns true + t_out if found, false if no crossing. */
bool ivp_3d_solver_find_first_t(ivp_3d_solver_t *solver,
                                  ivp_real_t threshold,
                                  ivp_time_t t_now,
                                  ivp_time_t t_max,
                                  ivp_matrix_cache_t *cache_a,
                                  ivp_matrix_cache_t *cache_b,
                                  ivp_time_t *t_out);

/* ── Built-in distance functions ───────────────────────────────────────── */

/* Point-point distance: ||pos_a - pos_b|| */
ivp_real_t ivp_3d_dist_point_point(const ivp_transform_r_t *a,
                                     const ivp_transform_r_t *b,
                                     void *ud);

/* Ball-poly distance (simplified): center distance minus combined radii.
 * user_data points to ivp_ball_poly_data_t. */
typedef struct ivp_ball_poly_data {
    ivp_real_t radius_a;
    ivp_real_t radius_b;
} ivp_ball_poly_data_t;

ivp_real_t ivp_3d_dist_ball_poly(const ivp_transform_r_t *a,
                                   const ivp_transform_r_t *b,
                                   void *ud);

#ifdef __cplusplus
}
#endif

#endif /* IVP_3D_SOLVER_H */
