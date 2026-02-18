/* ivp_3d_solver.c — 3D Solver for continuous collision detection
 *
 * Root-finding algorithm (from legacy):
 *   1. Step through sub-PSI intervals (dt/20 each)
 *   2. Evaluate distance at each step via matrix cache (lazy compute)
 *   3. When value crosses threshold between steps i and i+1:
 *      - Use secant method to narrow
 *      - Alternate with midpoint every 4th iteration (convergence guarantee)
 *      - Terminate when |value - threshold| < eps
 */

#include <ivp/collision/ivp_3d_solver.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_vec3.h>
#include <string.h>

/* ── Initialization ────────────────────────────────────────────────────── */

void ivp_3d_solver_init(ivp_3d_solver_t *solver,
                           ivp_3d_distance_fn fn,
                           void *user_data,
                           ivp_real_t max_deviation)
{
    solver->distance_fn = fn;
    solver->user_data = user_data;
    solver->max_deviation = max_deviation;
    solver->inv_max_deviation = (max_deviation > IVP_REAL_EPS)
        ? IVP_REAL(1.0) / max_deviation
        : IVP_REAL(0.0);
}

/* ── Evaluate distance at a sub-step ───────────────────────────────────── */

static ivp_real_t evaluate_distance(ivp_3d_solver_t *solver,
                                      ivp_matrix_cache_t *cache_a,
                                      ivp_matrix_cache_t *cache_b,
                                      int index)
{
    const ivp_transform_r_t *ma = ivp_matrix_cache_get(cache_a, index);
    const ivp_transform_r_t *mb = ivp_matrix_cache_get(cache_b, index);
    return solver->distance_fn(ma, mb, solver->user_data);
}

/* ── Root refinement (secant + bisection) ──────────────────────────────── */

static ivp_time_t refine_root(ivp_3d_solver_t *solver,
                                ivp_real_t threshold,
                                ivp_matrix_cache_t *cache_a,
                                ivp_matrix_cache_t *cache_b,
                                ivp_time_t t0, ivp_real_t v0,
                                ivp_time_t t1, ivp_real_t v1)
{
    const int max_refine = 16;
    const ivp_real_t eps = IVP_REAL(1e-6);

    for (int iter = 0; iter < max_refine; iter++) {
        ivp_time_t t_new;

        if (iter % 4 == 3) {
            /* Bisection step every 4th iteration for convergence guarantee */
            t_new = (t0 + t1) * 0.5;
        } else {
            /* Secant method */
            ivp_real_t dv = v1 - v0;
            if (ivp_fabs(dv) < IVP_REAL_EPS) {
                t_new = (t0 + t1) * 0.5;
            } else {
                ivp_real_t frac = (threshold - v0) / dv;
                if (frac < IVP_REAL(0.0)) frac = IVP_REAL(0.0);
                if (frac > IVP_REAL(1.0)) frac = IVP_REAL(1.0);
                t_new = t0 + (ivp_time_t)frac * (t1 - t0);
            }
        }

        /* Evaluate at t_new using closest cache index */
        ivp_real_t dt_total = (ivp_real_t)(cache_a->dt);
        ivp_real_t frac_new = (dt_total > IVP_REAL_EPS)
            ? (ivp_real_t)(t_new - cache_a->base_time) / dt_total
            : IVP_REAL(0.0);
        int idx = (int)(frac_new * (ivp_real_t)IVP_3D_SOLVER_MAX_STEPS);
        if (idx < 0) idx = 0;
        if (idx > IVP_3D_SOLVER_MAX_STEPS) idx = IVP_3D_SOLVER_MAX_STEPS;

        const ivp_transform_r_t *ma = ivp_matrix_cache_get(cache_a, idx);
        const ivp_transform_r_t *mb = ivp_matrix_cache_get(cache_b, idx);
        ivp_real_t v_new = solver->distance_fn(ma, mb, solver->user_data);

        if (ivp_fabs(v_new - threshold) < eps) {
            return t_new;
        }

        /* Update bracket: keep the sign change */
        if ((v0 - threshold) * (v_new - threshold) < IVP_REAL(0.0)) {
            t1 = t_new;
            v1 = v_new;
        } else {
            t0 = t_new;
            v0 = v_new;
        }
    }

    return t0;
}

/* ── Main root finder ──────────────────────────────────────────────────── */

bool ivp_3d_solver_find_first_t(ivp_3d_solver_t *solver,
                                  ivp_real_t threshold,
                                  ivp_time_t t_now,
                                  ivp_time_t t_max,
                                  ivp_matrix_cache_t *cache_a,
                                  ivp_matrix_cache_t *cache_b,
                                  ivp_time_t *t_out)
{
    if (!solver || !solver->distance_fn || !cache_a || !cache_b || !t_out) {
        return false;
    }

    /* Step through sub-PSI intervals */
    ivp_real_t prev_val = evaluate_distance(solver, cache_a, cache_b, 0);

    for (int step = 1; step <= IVP_3D_SOLVER_MAX_STEPS; step++) {
        ivp_real_t val = evaluate_distance(solver, cache_a, cache_b, step);

        /* Check for threshold crossing: distance goes below threshold */
        if (prev_val > threshold && val <= threshold) {
            /* Found crossing between step-1 and step */
            ivp_real_t dt_total = (ivp_real_t)(t_max - t_now);
            ivp_time_t step_t0 = t_now + (ivp_time_t)(dt_total * (ivp_real_t)(step - 1) / (ivp_real_t)IVP_3D_SOLVER_MAX_STEPS);
            ivp_time_t step_t1 = t_now + (ivp_time_t)(dt_total * (ivp_real_t)step / (ivp_real_t)IVP_3D_SOLVER_MAX_STEPS);

            *t_out = refine_root(solver, threshold, cache_a, cache_b,
                                  step_t0, prev_val, step_t1, val);
            return true;
        }

        prev_val = val;
    }

    return false;
}

/* ── Built-in distance functions ───────────────────────────────────────── */

ivp_real_t ivp_3d_dist_point_point(const ivp_transform_r_t *a,
                                     const ivp_transform_r_t *b,
                                     void *ud)
{
    (void)ud;
    ivp_vec3r_t diff;
    ivp_vec3r_sub(&diff, &b->pos, &a->pos);
    return ivp_vec3r_length(&diff);
}

ivp_real_t ivp_3d_dist_ball_poly(const ivp_transform_r_t *a,
                                   const ivp_transform_r_t *b,
                                   void *ud)
{
    ivp_ball_poly_data_t *data = (ivp_ball_poly_data_t *)ud;
    ivp_vec3r_t diff;
    ivp_vec3r_sub(&diff, &b->pos, &a->pos);
    ivp_real_t dist = ivp_vec3r_length(&diff);
    return dist - data->radius_a - data->radius_b;
}
