/* ivp_multidimensional_interp.c — port of IVP_MI_Vector and
 * IVP_Multidimensional_Interpolator (ivp_controller/ivp_multidimensional_interp.cxx).
 *
 * IVP_FLOAT arithmetic stays float; IVP_Inline_Math::ivp_sqrtf / fabsd take
 * an ivp_real_t (IVP_DOUBLE) argument like the reference.
 */
#include <ivp/utility/ivp_multidimensional_interp.h>

#include <math.h>
#include <string.h>

#ifdef IVP_REAL_DOUBLE
#define MI_SQRTF(x) sqrt((double)(x))   /* IVP_Inline_Math::ivp_sqrtf */
#define MI_FABSD(x) fabs((double)(x))   /* IVP_Inline_Math::fabsd */
#else
#define MI_SQRTF(x) sqrtf((float)(x))
#define MI_FABSD(x) fabsf((float)(x))
#endif

/* ── IVP_MI_Vector ────────────────────────────────────────────────────── */

static size_t mi_vector_bytes(int size)
{
    return sizeof(ivp_mi_vector_t) + sizeof(float) * (size_t)size;
}

ivp_mi_vector_t *ivp_mi_vector_malloc(int size, const ivp_allocator_t *alloc)
{
    if (size < 1) {
        IVP_ASSERT(size > 0);
        size = 1;
    }
    if (!alloc) alloc = ivp_default_allocator();
    ivp_mi_vector_t *result = (ivp_mi_vector_t *)ivp_alloc(alloc, mi_vector_bytes(size));
    if (!result) return NULL;
    result->element = (float *)(result + 1);
    result->capacity = size;
    result->nr_of_elements = size;
    for (int i = 0; i < size; i++) result->element[i] = 0.0f;
    result->weight_statistic = 0.0f;
    result->time_stamp = 0.0;
    return result;
}

ivp_mi_vector_t *ivp_mi_vector_malloc_and_set(int size, const float *values, const ivp_allocator_t *alloc)
{
    if (size < 1) {
        IVP_ASSERT(size > 0);
        size = 1;
    }
    ivp_mi_vector_t *result = ivp_mi_vector_malloc(size, alloc);
    if (!result) return NULL;
    for (int i = 0; i < size; i++) result->element[i] = values[i];
    return result;
}

void ivp_mi_vector_free(ivp_mi_vector_t *v, const ivp_allocator_t *alloc)
{
    if (!v) return;
    if (!alloc) alloc = ivp_default_allocator();
    ivp_free(alloc, v, mi_vector_bytes(v->capacity));
}

void ivp_mi_vector_init(ivp_mi_vector_t *v, float *storage, int size)
{
    v->element = storage;
    v->capacity = size;
    v->nr_of_elements = size;
    for (int i = 0; i < size; i++) v->element[i] = 0.0f;
    v->weight_statistic = 0.0f;
    v->time_stamp = 0.0;
}

void ivp_mi_vector_set(ivp_mi_vector_t *v, const ivp_mi_vector_t *src)
{
    if (v->nr_of_elements < src->nr_of_elements) {
        IVP_ASSERT(v->nr_of_elements >= src->nr_of_elements);
        return;
    }
    v->nr_of_elements = src->nr_of_elements;
    v->weight_statistic = src->weight_statistic;
    for (int i = 0; i < v->nr_of_elements; i++) v->element[i] = src->element[i];
}

void ivp_mi_vector_subtract(ivp_mi_vector_t *v, const ivp_mi_vector_t *o)
{
    IVP_ASSERT(v->nr_of_elements == o->nr_of_elements);
    for (int i = 0; i < v->nr_of_elements; i++) v->element[i] -= o->element[i];
}

void ivp_mi_vector_add(ivp_mi_vector_t *v, const ivp_mi_vector_t *o)
{
    IVP_ASSERT(v->nr_of_elements == o->nr_of_elements);
    for (int i = 0; i < v->nr_of_elements; i++) v->element[i] += o->element[i];
}

void ivp_mi_vector_add_multiple(ivp_mi_vector_t *v, const ivp_mi_vector_t *o, float d)
{
    IVP_ASSERT(v->nr_of_elements == o->nr_of_elements);
    for (int i = 0; i < v->nr_of_elements; i++) v->element[i] += (o->element[i] * d);
}

void ivp_mi_vector_mult(ivp_mi_vector_t *v, float d)
{
    for (int i = 0; i < v->nr_of_elements; i++) v->element[i] *= d;
}

float ivp_mi_vector_length(const ivp_mi_vector_t *v)
{
    float result = 0.0f;
    for (int i = 0; i < v->nr_of_elements; i++) result += v->element[i] * v->element[i];
    return (float)(MI_SQRTF(result));
}

/* ── linfit ───────────────────────────────────────────────────────────── */

/* IVP_Multidimensional_Interpolator::linfit: solves the linear
 * compensation problem [A_0..A_{n-1}] X = A_n (m lines) with Givens
 * rotations; res is the residuum. */
static bool mi_linfit(const ivp_mi_interpolator_t *mi, int m, int n, ivp_mi_vector_t **A, float *X, float *res)
{
    const float MI_eps = mi->MI_eps;

    /* cancel condition value f */
    float s;
    float f = 0.0f;
    for (int k = 0; k < n; k++) {
        s = 0.0f;
        for (int i = 0; i < m; i++) s += A[k]->element[i] * A[k]->element[i];
        if (s > f) f = s;
    }
    f = (float)MI_SQRTF(f) * MI_eps;

    /* orthogonal elimination */
    for (int j = 0; j < n; j++) {
        for (int i = j + 1; i < m; i++) {
            if (MI_FABSD(A[j]->element[i]) > (ivp_real_t)MI_eps) {
                float q = (float)MI_SQRTF(A[j]->element[i] * A[j]->element[i] + A[j]->element[j] * A[j]->element[j]);
                IVP_ASSERT(q != 0.0f);
                float c = A[j]->element[j] / q;
                s = -A[j]->element[i] / q;
                A[j]->element[j] = q;
                for (int k = j + 1; k <= n; k++) {
                    float b = s * A[k]->element[j];
                    ivp_mi_vector_set_element(A[k], j, c * A[k]->element[j] - s * A[k]->element[i]);
                    ivp_mi_vector_set_element(A[k], i, c * A[k]->element[i] + b);
                }
            }
        }
        if (MI_FABSD(A[j]->element[j]) <= (ivp_real_t)f) return false; /* singular */
    }

    /* substitute back */
    X[n - 1] = A[n]->element[n - 1] / A[n - 1]->element[n - 1];
    for (int i = n - 2; i >= 0; i--) {
        X[i] = A[n]->element[i] - (X[n - 1] * A[n - 1]->element[i]);
        for (int j = i + 1; j <= n - 2; j++) X[i] -= A[j]->element[i] * X[j];
        IVP_ASSERT(A[i]->element[i] != 0.0f);
        X[i] /= A[i]->element[i];
    }

    s = 0;
    for (int l = n; l < m; l++) s += A[n]->element[l] * A[n]->element[l];
    *res = (float)MI_SQRTF(s);
    return true;
}

/* ── sort_vectors ─────────────────────────────────────────────────────── */

static void mi_sort_vectors(ivp_mi_interpolator_t *mi, int nr_to_be_sorted)
{
    ivp_mi_vector_t **pi = mi->previous_inputs;
    ivp_mi_vector_t **ps = mi->previous_solutions;
    int first = 0;
    for (;;) {
        int second = first + 1;
        if (second >= nr_to_be_sorted) break;

        float second_weight = pi[second]->weight_statistic;
        if (pi[first]->weight_statistic < second_weight) {
            ivp_mi_vector_t *tmp_vec = pi[second];
            pi[second] = pi[first];
            pi[first] = tmp_vec;
            tmp_vec = ps[second];
            ps[second] = ps[first];
            ps[first] = tmp_vec;

            int test_pos = first;
            while ((test_pos > 0) && (pi[test_pos - 1]->weight_statistic < second_weight)) {
                tmp_vec = pi[test_pos];
                pi[test_pos] = pi[test_pos - 1];
                pi[test_pos - 1] = tmp_vec;
                tmp_vec = ps[test_pos];
                ps[test_pos] = ps[test_pos - 1];
                ps[test_pos - 1] = tmp_vec;
                test_pos--;
            }
        }
        first = second;
    }
}

/* ── check_interpolation ──────────────────────────────────────────────── */

bool ivp_mi_check_interpolation(ivp_mi_interpolator_t *mi,
                                const ivp_mi_vector_t *new_input,
                                int max_tries_nr_of_vectors_involved,
                                float max_res,
                                ivp_mi_vector_t *output)
{
    if ((mi->nr_occupied < 2) || (mi->nr_of_elements_input <= 0)) return false;

    int max_vectors_involved = mi->nr_occupied;
    const int max_vectors_by_dimension = mi->nr_of_elements_input + 1;
    if (max_vectors_involved > max_vectors_by_dimension) max_vectors_involved = max_vectors_by_dimension;
    if (max_vectors_involved < 2) return false;

    /* scratchboard_A[0..nr_occupied] (IVP_MI_VECTOR_ALLOCA: zeroed) */
    const int nin = mi->nr_of_elements_input;
    ivp_mi_vector_t **A = mi->scratchboard_A;
    float *floats = mi->scratch_floats;
    for (int i = 0; i <= mi->nr_occupied; i++) {
        ivp_mi_vector_init(&mi->scratchboard[i], floats + (size_t)i * (size_t)nin, nin);
        A[i] = &mi->scratchboard[i];
    }

    float *interpolation_weights = floats + (size_t)(mi->nr_of_vectors + 1) * (size_t)nin;
    float interpolation_weigh_first = 0.0f;
    float res = 0.0f;
    bool success = false;

    ivp_mi_vector_t new_input_difference;
    ivp_mi_vector_init(&new_input_difference, interpolation_weights + mi->nr_of_vectors, nin);
    ivp_mi_vector_set(&new_input_difference, new_input);
    ivp_mi_vector_subtract(&new_input_difference, mi->previous_inputs[0]);

    /* after some time try again with a smaller number of vectors */
    if (mi->counter_tries_nr_of_vectors_involved++ >= max_tries_nr_of_vectors_involved) {
        mi->nr_of_vectors_involved = 2;
        mi->counter_tries_nr_of_vectors_involved = 0;
    }

    if (ivp_mi_vector_length(&new_input_difference) < mi->MI_eps) {
        /* the first stored input vector is already sufficient */
        mi->nr_of_vectors_involved = 1;
#ifdef IVP_DEBUG_BUILD
        mi->nr_one_vector_sufficient++;
#endif
        success = true;
    } else {
        if ((mi->nr_of_vectors_involved < 2) || (mi->nr_of_vectors_involved > max_vectors_involved)) {
            mi->nr_of_vectors_involved = 2;
        }

        while (mi->nr_of_vectors_involved <= max_vectors_involved) {
            const int nvi = mi->nr_of_vectors_involved;
            for (int i = 0; i < (nvi - 1); i++) {
                ivp_mi_vector_set(A[i], mi->previous_inputs[i + 1]);
                ivp_mi_vector_subtract(A[i], mi->previous_inputs[0]);
            }
            ivp_mi_vector_set(A[nvi - 1], &new_input_difference); /* [A|b] ready for linfit */

            success = mi_linfit(mi, mi->nr_of_elements_input, nvi - 1, A, interpolation_weights, &res);

            if (success) {
                interpolation_weigh_first = 1.0f;
                if (res > max_res) {
                    success = false;
                } else {
                    /* interpolation weights must lie in [-0.2, 1.2] */
                    for (int ix = 0; ix < (nvi - 1); ix++) {
                        if (MI_FABSD(interpolation_weights[ix] - 0.5f) > (ivp_real_t)0.7f) {
                            success = false;
                            break;
                        }
                        interpolation_weigh_first -= interpolation_weights[ix];
                    }
                    if ((MI_FABSD(interpolation_weigh_first - 0.5f) > (ivp_real_t)0.7f) && success) {
                        success = false;
                    }
                }
                if (success) {
#ifdef IVP_DEBUG_BUILD
                    mi->nr_of_success[nvi - 1]++;
#endif
                    break;
                }
            } else {
                /* linfit could not solve the interpolation */
#ifdef IVP_DEBUG_BUILD
                mi->nr_of_linfit_failure[nvi - 1]++;
#endif
            }
            mi->nr_of_vectors_involved++;
        }
    }

    if (success) {
        ivp_mi_vector_t *scratch = mi->previous_solutions[mi->scratch_area_index];
        for (int j = 0; j < mi->nr_of_elements_solution; j++) ivp_mi_vector_set_element(scratch, j, 0.0f);

        for (int i = 1; i < mi->nr_of_vectors_involved; i++) {
            mi->previous_inputs[i]->weight_statistic =
                mi->previous_inputs[i]->weight_statistic * mi->influence_of_old_weight + interpolation_weights[i - 1];
            for (int k = 0; k < mi->nr_of_elements_solution; k++) {
                ivp_mi_vector_set_element(scratch, k,
                                          scratch->element[k] + interpolation_weights[i - 1] * mi->previous_solutions[i]->element[k]);
            }
        }

        mi->previous_inputs[0]->weight_statistic =
            (mi->previous_inputs[0]->weight_statistic * mi->influence_of_old_weight) + interpolation_weigh_first;
        for (int k = 0; k < mi->nr_of_elements_solution; k++) {
            ivp_mi_vector_set_element(scratch, k,
                                      scratch->element[k] + interpolation_weigh_first * mi->previous_solutions[0]->element[k]);
        }

        ivp_mi_vector_set(output, scratch);

        /* sort in declining order of the weight statistics */
        mi_sort_vectors(mi, mi->nr_occupied);
    }

    return success;
}

/* ── insertion ────────────────────────────────────────────────────────── */

void ivp_mi_add_new_input_solution_combination_conventional(ivp_mi_interpolator_t *mi,
                                                            const ivp_mi_vector_t *new_input,
                                                            const ivp_mi_vector_t *new_solution)
{
    if (mi->nr_of_vectors <= 0) {
        IVP_ASSERT(mi->nr_of_vectors > 0);
        return;
    }

    /* move all vectors one position further, the new ones go to the front */
    ivp_mi_vector_t *tmp_vec_input = mi->previous_inputs[mi->nr_of_vectors - 1];
    ivp_mi_vector_t *tmp_vec_solution = mi->previous_solutions[mi->nr_of_vectors - 1];
    for (int i = mi->nr_of_vectors - 2; i >= 0; i--) {
        mi->previous_inputs[i + 1] = mi->previous_inputs[i];
        mi->previous_solutions[i + 1] = mi->previous_solutions[i];
    }
    mi->previous_inputs[0] = tmp_vec_input;
    mi->previous_solutions[0] = tmp_vec_solution;

    ivp_mi_vector_set(mi->previous_inputs[0], new_input);
    ivp_mi_vector_set(mi->previous_solutions[0], new_solution);
    ivp_mi_vector_set_time_stamp(mi->previous_inputs[0], new_input->time_stamp);
    ivp_mi_vector_set_time_stamp(mi->previous_solutions[0], new_solution->time_stamp);

    /* a weight that sorts the new input vector very high */
    mi->previous_inputs[0]->weight_statistic = 1.0f / (1.0f - mi->influence_of_old_weight);

    if (mi->nr_occupied < mi->nr_of_vectors) mi->nr_occupied++;
}

void ivp_mi_add_new_input_solution_combination_stochastic(ivp_mi_interpolator_t *mi,
                                                          const ivp_mi_vector_t *new_input,
                                                          const ivp_mi_vector_t *new_solution)
{
    if (mi->nr_of_vectors <= 0) {
        IVP_ASSERT(mi->nr_of_vectors > 0);
        return;
    }

    int c = mi->counter_vector_replacement;
    int solution_slot = ((c < 0 ? -c : c) * 101) % mi->nr_of_vectors;
    mi->counter_vector_replacement--;

    ivp_mi_vector_set(mi->previous_inputs[solution_slot], new_input);
    ivp_mi_vector_set(mi->previous_solutions[solution_slot], new_solution);
    ivp_mi_vector_set_time_stamp(mi->previous_inputs[solution_slot], new_input->time_stamp);
    ivp_mi_vector_set_time_stamp(mi->previous_solutions[solution_slot], new_solution->time_stamp);

    mi->previous_inputs[solution_slot]->weight_statistic = 1.0f / (1.0f - mi->influence_of_old_weight);

    mi_sort_vectors(mi, mi->nr_occupied);

    if (mi->counter_vector_replacement < 0) mi->counter_vector_replacement = mi->initial_value_vector_replacement;
}

/* ── lifecycle ────────────────────────────────────────────────────────── */

int ivp_mi_init(ivp_mi_interpolator_t *mi,
                int nr_of_vectors, int nr_of_elements_input, int nr_of_elements_solution,
                const ivp_allocator_t *alloc)
{
    memset(mi, 0, sizeof(*mi));
    if (!alloc) alloc = ivp_default_allocator();
    mi->alloc = alloc;
    if (nr_of_vectors < 1) {
        IVP_ASSERT(nr_of_vectors > 0);
        nr_of_vectors = 1;
    }
    if (nr_of_elements_input < 1) {
        IVP_ASSERT(nr_of_elements_input > 0);
        nr_of_elements_input = 1;
    }
    if (nr_of_elements_solution < 1) {
        IVP_ASSERT(nr_of_elements_solution > 0);
        nr_of_elements_solution = 1;
    }

    mi->nr_of_vectors = nr_of_vectors;
    mi->nr_of_elements_input = nr_of_elements_input;
    mi->nr_of_elements_solution = nr_of_elements_solution;

    mi->counter_tries_nr_of_vectors_involved = 0;
    mi->initial_value_vector_replacement = 50;
    mi->counter_vector_replacement = mi->initial_value_vector_replacement;

    mi->previous_inputs = (ivp_mi_vector_t **)ivp_alloc(alloc, sizeof(void *) * (size_t)nr_of_vectors);
    mi->previous_solutions = (ivp_mi_vector_t **)ivp_alloc(alloc, sizeof(void *) * (size_t)(nr_of_vectors + 1));
    mi->scratchboard = (ivp_mi_vector_t *)ivp_alloc(alloc, sizeof(ivp_mi_vector_t) * (size_t)(nr_of_vectors + 1));
    mi->scratchboard_A = (ivp_mi_vector_t **)ivp_alloc(alloc, sizeof(void *) * (size_t)(nr_of_vectors + 1));
    /* scratchboard elements, interpolation weights, new_input_difference */
    mi->scratch_floats_count = (nr_of_vectors + 1) * nr_of_elements_input + nr_of_vectors + nr_of_elements_input;
    mi->scratch_floats = (float *)ivp_alloc(alloc, sizeof(float) * (size_t)mi->scratch_floats_count);
    if (!mi->previous_inputs || !mi->previous_solutions || !mi->scratchboard || !mi->scratchboard_A ||
        !mi->scratch_floats) {
        ivp_mi_destroy(mi);
        return -1;
    }
    memset(mi->previous_inputs, 0, sizeof(void *) * (size_t)nr_of_vectors);
    memset(mi->previous_solutions, 0, sizeof(void *) * (size_t)(nr_of_vectors + 1));
    for (int i = 0; i < nr_of_vectors; i++) {
        mi->previous_inputs[i] = ivp_mi_vector_malloc(nr_of_elements_input, alloc);
        if (!mi->previous_inputs[i]) { ivp_mi_destroy(mi); return -1; }
    }
    for (int i = 0; i <= nr_of_vectors; i++) {
        mi->previous_solutions[i] = ivp_mi_vector_malloc(nr_of_elements_solution, alloc);
        if (!mi->previous_solutions[i]) { ivp_mi_destroy(mi); return -1; }
    }

    mi->influence_of_old_weight = 0.8f;
    mi->nr_occupied = 0;
    mi->nr_of_vectors_involved = 2;
    mi->scratch_area_index = nr_of_vectors;
    mi->MI_eps = 1.0e-14f;

    /* debug variables (reference #ifdef DEBUG: new int[nr_of_vectors]) */
#ifdef IVP_DEBUG_BUILD
    mi->nr_one_vector_sufficient = 0;
    mi->nr_res_over_limit = (int *)ivp_alloc(alloc, sizeof(int) * (size_t)nr_of_vectors);
    mi->nr_int_weight_over_limit = (int *)ivp_alloc(alloc, sizeof(int) * (size_t)nr_of_vectors);
    mi->nr_of_linfit_failure = (int *)ivp_alloc(alloc, sizeof(int) * (size_t)nr_of_vectors);
    mi->nr_of_success = (int *)ivp_alloc(alloc, sizeof(int) * (size_t)nr_of_vectors);
    if (!mi->nr_res_over_limit || !mi->nr_int_weight_over_limit || !mi->nr_of_linfit_failure ||
        !mi->nr_of_success) {
        ivp_mi_destroy(mi);
        return -1;
    }
    for (int i = 0; i < nr_of_vectors; i++) {
        mi->nr_res_over_limit[i] = 0;
        mi->nr_int_weight_over_limit[i] = 0;
        mi->nr_of_linfit_failure[i] = 0;
        mi->nr_of_success[i] = 0;
    }
#endif
    return 0;
}

void ivp_mi_destroy(ivp_mi_interpolator_t *mi)
{
    if (!mi) return;
    const ivp_allocator_t *alloc = mi->alloc ? mi->alloc : ivp_default_allocator();
#ifdef IVP_DEBUG_BUILD
    {
        int **debug_arrays[4] = { &mi->nr_res_over_limit, &mi->nr_int_weight_over_limit,
                                  &mi->nr_of_linfit_failure, &mi->nr_of_success };
        for (int d = 0; d < 4; d++) {
            if (*debug_arrays[d]) ivp_free(alloc, *debug_arrays[d], sizeof(int) * (size_t)mi->nr_of_vectors);
            *debug_arrays[d] = NULL;
        }
    }
#endif
    if (mi->previous_inputs) {
        for (int i = 0; i < mi->nr_of_vectors; i++) ivp_mi_vector_free(mi->previous_inputs[i], alloc);
        ivp_free(alloc, mi->previous_inputs, sizeof(void *) * (size_t)mi->nr_of_vectors);
    }
    if (mi->previous_solutions) {
        for (int i = 0; i <= mi->nr_of_vectors; i++) ivp_mi_vector_free(mi->previous_solutions[i], alloc);
        ivp_free(alloc, mi->previous_solutions, sizeof(void *) * (size_t)(mi->nr_of_vectors + 1));
    }
    if (mi->scratchboard) {
        ivp_free(alloc, mi->scratchboard, sizeof(ivp_mi_vector_t) * (size_t)(mi->nr_of_vectors + 1));
    }
    if (mi->scratchboard_A) {
        ivp_free(alloc, mi->scratchboard_A, sizeof(void *) * (size_t)(mi->nr_of_vectors + 1));
    }
    if (mi->scratch_floats) {
        ivp_free(alloc, mi->scratch_floats, sizeof(float) * (size_t)mi->scratch_floats_count);
    }
    memset(mi, 0, sizeof(*mi));
}
