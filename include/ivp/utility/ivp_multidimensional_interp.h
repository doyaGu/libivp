/* ivp_multidimensional_interp.h — Multidimensional interpolator
 *
 * Port of IVP_MI_Vector and IVP_Multidimensional_Interpolator
 * (ivp_controller/ivp_multidimensional_interp.hxx/.cxx): stores previous
 * input/solution vector pairs and tries to express a new input vector as
 * an affine combination of stored inputs (Givens-rotation least squares,
 * "linfit"); on success the same combination of the stored solutions is
 * returned instead of running the expensive solver.  All arithmetic is
 * IVP_FLOAT (float) like the reference, IVP_Inline_Math::ivp_sqrtf / fabsd
 * act on ivp_real_t.
 *
 * Used by the buoyancy controller (IVP_Template_Buoyancy::use_interpolation).
 */
#ifndef IVP_MULTIDIMENSIONAL_INTERP_H
#define IVP_MULTIDIMENSIONAL_INTERP_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_assert.h> /* IVP_DEBUG_BUILD, IVP_ASSERT */

#ifdef __cplusplus
extern "C" {
#endif

/* ── IVP_MI_Vector ────────────────────────────────────────────────────── */

/* IVP_MI_Vector_Base + element array.  Heap vectors (ivp_mi_vector_malloc)
 * store their elements right after the struct; stack vectors
 * (IVP_MI_VECTOR_ALLOCA) use caller storage via ivp_mi_vector_init. */
typedef struct ivp_mi_vector {
    int    nr_of_elements;     /* number of elements in 'element' */
    float  weight_statistic;   /* weight used by sort_vectors */
    double time_stamp;         /* IVP_Time time_stamp (debugging only) */
    float *element;            /* [nr_of_elements] */
    int    capacity;           /* libivp: allocated elements */
} ivp_mi_vector_t;

/* IVP_MI_Vector::malloc_mi_vector: zeroed elements, weight 0. */
ivp_mi_vector_t *ivp_mi_vector_malloc(int size, const ivp_allocator_t *alloc);

/* IVP_MI_Vector::malloc_and_set_mi_vector. */
ivp_mi_vector_t *ivp_mi_vector_malloc_and_set(int size, const float *values, const ivp_allocator_t *alloc);

/* Free a vector from ivp_mi_vector_malloc*. */
void ivp_mi_vector_free(ivp_mi_vector_t *v, const ivp_allocator_t *alloc);

/* IVP_MI_VECTOR_ALLOCA / IVP_MI_VECTOR_CLEAR on caller storage of at
 * least 'size' floats: zeroed elements, weight 0. */
void ivp_mi_vector_init(ivp_mi_vector_t *v, float *storage, int size);

/* IVP_MI_Vector::set(v): copies nr_of_elements, weight_statistic and the
 * elements (ignored when 'v' has more elements than this vector). */
void ivp_mi_vector_set(ivp_mi_vector_t *v, const ivp_mi_vector_t *src);

/* IVP_MI_Vector::set(pos, value) */
static inline void ivp_mi_vector_set_element(ivp_mi_vector_t *v, int pos, float value)
{
    IVP_ASSERT((pos < v->nr_of_elements) && (pos >= 0));
    v->element[pos] = value;
}

void  ivp_mi_vector_subtract(ivp_mi_vector_t *v, const ivp_mi_vector_t *o);          /* element -= o */
void  ivp_mi_vector_add(ivp_mi_vector_t *v, const ivp_mi_vector_t *o);               /* element += o */
void  ivp_mi_vector_add_multiple(ivp_mi_vector_t *v, const ivp_mi_vector_t *o, float d); /* element += o * d */
void  ivp_mi_vector_mult(ivp_mi_vector_t *v, float d);                               /* element *= d */
float ivp_mi_vector_length(const ivp_mi_vector_t *v);                                /* euclidean length */

static inline void ivp_mi_vector_set_time_stamp(ivp_mi_vector_t *v, double current_time)
{
    v->time_stamp = current_time;
}

/* ── IVP_Multidimensional_Interpolator ────────────────────────────────── */

typedef struct ivp_mi_interpolator {
    const ivp_allocator_t *alloc;

    /* stored input / solution vectors; previous_solutions has one extra
     * vector (scratch_area_index) holding the last interpolated solution */
    ivp_mi_vector_t **previous_inputs;     /* [nr_of_vectors] */
    ivp_mi_vector_t **previous_solutions;  /* [nr_of_vectors + 1] */

    int   nr_of_vectors;
    int   nr_of_elements_input;
    int   nr_of_elements_solution;

    float MI_eps;                   /* 1e-14f */
    float influence_of_old_weight;  /* 0.8f */

    int   nr_occupied;
    int   scratch_area_index;       /* nr_of_vectors */
    int   nr_of_vectors_involved;   /* of the last interpolation (starts at 2) */
    int   counter_tries_nr_of_vectors_involved;
    int   initial_value_vector_replacement;   /* 50 */
    int   counter_vector_replacement;

    /* check_interpolation scratch (alloca in the reference) */
    ivp_mi_vector_t  *scratchboard;    /* [nr_of_vectors + 1] */
    ivp_mi_vector_t **scratchboard_A;  /* pointers to scratchboard */
    float            *scratch_floats;
    int              scratch_floats_count;

#ifdef IVP_DEBUG_BUILD
    /* debug statistics (reference "#ifdef DEBUG" members, [nr_of_vectors]) */
    int *nr_res_over_limit;
    int *nr_int_weight_over_limit;
    int *nr_of_linfit_failure;
    int *nr_of_success;
    int  nr_one_vector_sufficient;
#endif
} ivp_mi_interpolator_t;

/* IVP_Multidimensional_Interpolator(nr_of_vectors, nr_of_elements_input,
 * nr_of_elements_solution).  Returns 0 on success, -1 when out of memory. */
int ivp_mi_init(ivp_mi_interpolator_t *mi,
                int nr_of_vectors, int nr_of_elements_input, int nr_of_elements_solution,
                const ivp_allocator_t *alloc);

/* ~IVP_Multidimensional_Interpolator */
void ivp_mi_destroy(ivp_mi_interpolator_t *mi);

static inline int ivp_mi_get_nr_of_vectors(const ivp_mi_interpolator_t *mi) { return mi->nr_of_vectors; }
static inline int ivp_mi_get_nr_occupied(const ivp_mi_interpolator_t *mi) { return mi->nr_occupied; }

/* IVP_Multidimensional_Interpolator::check_interpolation: true (IVP_OK)
 * when new_input could be interpolated; 'output' then holds the
 * interpolated solution (it needs nr_of_elements_solution elements). */
bool ivp_mi_check_interpolation(ivp_mi_interpolator_t *mi,
                                const ivp_mi_vector_t *new_input,
                                int max_tries_nr_of_vectors_involved,
                                float max_res,
                                ivp_mi_vector_t *output);

/* add_new_input_solution_combination_conventional: FIFO insertion at the
 * front. */
void ivp_mi_add_new_input_solution_combination_conventional(ivp_mi_interpolator_t *mi,
                                                            const ivp_mi_vector_t *new_input,
                                                            const ivp_mi_vector_t *new_solution);

/* add_new_input_solution_combination_stochastic: replaces a pseudo-random
 * slot, then re-sorts by weight. */
void ivp_mi_add_new_input_solution_combination_stochastic(ivp_mi_interpolator_t *mi,
                                                          const ivp_mi_vector_t *new_input,
                                                          const ivp_mi_vector_t *new_solution);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MULTIDIMENSIONAL_INTERP_H */
