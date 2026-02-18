/* ivp_multidimensional_interp.h — Multidimensional linear interpolation
 *
 * Caches input/solution vector pairs and uses linear fitting (LINFIT)
 * to predict new solutions from new inputs.  When the prediction is
 * within tolerance, the cached result is reused instead of computing
 * a fresh solution.
 *
 * Used by advanced buoyancy and fluid dynamics for performance.
 *
 * Ported from ivp_multidimensional_interp.hxx.
 */
#ifndef IVP_MULTIDIMENSIONAL_INTERP_H
#define IVP_MULTIDIMENSIONAL_INTERP_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Configuration ────────────────────────────────────────────────────── */

#define IVP_MI_MAX_HISTORY  32   /* max cached input/solution pairs */

/* ── Vector type ──────────────────────────────────────────────────────── */

/* Variable-length float vector used for interpolation inputs/outputs.
 * Size is fixed at creation time. */
typedef struct ivp_mi_vector {
    int    n;            /* number of elements */
    float  weight;       /* importance weight (for sorting) */
    float *data;         /* [n] elements (owned, or inline for stack alloc) */
} ivp_mi_vector_t;

/* ── Interpolator state ───────────────────────────────────────────────── */

typedef struct ivp_mi_interpolator {
    const ivp_allocator_t *alloc;

    int input_dim;       /* dimensionality of input vectors */
    int output_dim;      /* dimensionality of solution vectors */

    /* History ring buffer */
    ivp_mi_vector_t *inputs[IVP_MI_MAX_HISTORY];
    ivp_mi_vector_t *outputs[IVP_MI_MAX_HISTORY];
    int              n_entries;
    int              next_slot;

    /* Scratch space for linfit */
    float *scratch;      /* workspace */
    int    scratch_size;
} ivp_mi_interpolator_t;

/* ── Lifecycle ────────────────────────────────────────────────────────── */

/* Initialize an interpolator.
 *   input_dim  -- number of input floats per query
 *   output_dim -- number of output floats per solution */
int ivp_mi_init(ivp_mi_interpolator_t *interp,
                 int input_dim, int output_dim,
                 const ivp_allocator_t *alloc);

/* Destroy and free all internal storage. */
void ivp_mi_destroy(ivp_mi_interpolator_t *interp);

/* ── Vector helpers ───────────────────────────────────────────────────── */

/* Allocate a vector of the given dimension. */
ivp_mi_vector_t *ivp_mi_vector_alloc(int n, const ivp_allocator_t *alloc);

/* Free a vector. */
void ivp_mi_vector_free(ivp_mi_vector_t *v, const ivp_allocator_t *alloc);

/* ── Core operations ──────────────────────────────────────────────────── */

/* Try to interpolate a solution for new_input from cached history.
 *
 * Parameters:
 *   interp           -- the interpolator
 *   new_input        -- input vector (input_dim elements)
 *   max_vectors      -- max history entries to use (1..n_entries)
 *   max_residual     -- acceptable interpolation error
 *   output           -- pre-allocated output vector (output_dim elements)
 *
 * Returns 0 if interpolation succeeded (output is valid).
 * Returns -1 if interpolation failed (caller must compute fresh). */
int ivp_mi_check_interpolation(ivp_mi_interpolator_t *interp,
                                 const ivp_mi_vector_t *new_input,
                                 int max_vectors,
                                 float max_residual,
                                 ivp_mi_vector_t *output);

/* Add a computed input/solution pair to the history cache.
 * The vectors are copied internally. */
void ivp_mi_add_entry(ivp_mi_interpolator_t *interp,
                        const ivp_mi_vector_t *input,
                        const ivp_mi_vector_t *solution);

/* Clear all cached history entries. */
void ivp_mi_clear(ivp_mi_interpolator_t *interp);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MULTIDIMENSIONAL_INTERP_H */
