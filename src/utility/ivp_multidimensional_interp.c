/* ivp_multidimensional_interp.c — Multidimensional linear interpolation
 *
 * Uses LINFIT (least-squares linear fitting) to predict solutions from
 * cached input/output pairs.
 */
#include <ivp/utility/ivp_multidimensional_interp.h>
#include <math.h>
#include <string.h>

/* ── Vector helpers ───────────────────────────────────────────────────── */

ivp_mi_vector_t *ivp_mi_vector_alloc(int n, const ivp_allocator_t *alloc)
{
    if (!alloc) alloc = ivp_default_allocator();
    ivp_mi_vector_t *v = (ivp_mi_vector_t *)ivp_alloc(alloc, sizeof(*v));
    if (!v) return NULL;
    v->n = n;
    v->weight = 1.0f;
    v->data = (float *)ivp_alloc(alloc, (size_t)n * sizeof(float));
    if (!v->data) {
        ivp_free(alloc, v, sizeof(*v));
        return NULL;
    }
    memset(v->data, 0, (size_t)n * sizeof(float));
    return v;
}

void ivp_mi_vector_free(ivp_mi_vector_t *v, const ivp_allocator_t *alloc)
{
    if (!v) return;
    if (!alloc) alloc = ivp_default_allocator();
    if (v->data) {
        ivp_free(alloc, v->data, (size_t)v->n * sizeof(float));
    }
    ivp_free(alloc, v, sizeof(*v));
}

static ivp_mi_vector_t *vector_clone(const ivp_mi_vector_t *src,
                                       const ivp_allocator_t *alloc)
{
    ivp_mi_vector_t *v = ivp_mi_vector_alloc(src->n, alloc);
    if (!v) return NULL;
    memcpy(v->data, src->data, (size_t)src->n * sizeof(float));
    v->weight = src->weight;
    return v;
}

/* ── Lifecycle ────────────────────────────────────────────────────────── */

int ivp_mi_init(ivp_mi_interpolator_t *interp,
                 int input_dim, int output_dim,
                 const ivp_allocator_t *alloc)
{
    if (!alloc) alloc = ivp_default_allocator();
    memset(interp, 0, sizeof(*interp));
    interp->alloc = alloc;
    interp->input_dim = input_dim;
    interp->output_dim = output_dim;

    /* Allocate scratch space for linfit:
     * Need at least (n+1) * (m+1) floats for the augmented matrix. */
    int max_n = IVP_MI_MAX_HISTORY + 1;
    int max_m = input_dim + 1;
    interp->scratch_size = max_n * max_m + max_n + max_m;
    interp->scratch = (float *)ivp_alloc(alloc,
        (size_t)interp->scratch_size * sizeof(float));
    if (!interp->scratch) return -1;

    return 0;
}

void ivp_mi_destroy(ivp_mi_interpolator_t *interp)
{
    if (!interp) return;
    const ivp_allocator_t *alloc = interp->alloc;

    for (int i = 0; i < IVP_MI_MAX_HISTORY; i++) {
        if (interp->inputs[i])
            ivp_mi_vector_free(interp->inputs[i], alloc);
        if (interp->outputs[i])
            ivp_mi_vector_free(interp->outputs[i], alloc);
    }

    if (interp->scratch) {
        ivp_free(alloc, interp->scratch,
                 (size_t)interp->scratch_size * sizeof(float));
    }

    memset(interp, 0, sizeof(*interp));
}

void ivp_mi_clear(ivp_mi_interpolator_t *interp)
{
    if (!interp) return;
    const ivp_allocator_t *alloc = interp->alloc;

    for (int i = 0; i < IVP_MI_MAX_HISTORY; i++) {
        if (interp->inputs[i]) {
            ivp_mi_vector_free(interp->inputs[i], alloc);
            interp->inputs[i] = NULL;
        }
        if (interp->outputs[i]) {
            ivp_mi_vector_free(interp->outputs[i], alloc);
            interp->outputs[i] = NULL;
        }
    }
    interp->n_entries = 0;
    interp->next_slot = 0;
}

/* ── Add entry ────────────────────────────────────────────────────────── */

void ivp_mi_add_entry(ivp_mi_interpolator_t *interp,
                        const ivp_mi_vector_t *input,
                        const ivp_mi_vector_t *solution)
{
    if (!interp || !input || !solution) return;
    if (input->n != interp->input_dim || solution->n != interp->output_dim)
        return;

    int slot = interp->next_slot;
    const ivp_allocator_t *alloc = interp->alloc;

    /* Free old entry in this slot */
    if (interp->inputs[slot])
        ivp_mi_vector_free(interp->inputs[slot], alloc);
    if (interp->outputs[slot])
        ivp_mi_vector_free(interp->outputs[slot], alloc);

    interp->inputs[slot]  = vector_clone(input, alloc);
    interp->outputs[slot] = vector_clone(solution, alloc);

    interp->next_slot = (slot + 1) % IVP_MI_MAX_HISTORY;
    if (interp->n_entries < IVP_MI_MAX_HISTORY)
        interp->n_entries++;
}

/* ── LINFIT: least-squares linear fitting ─────────────────────────────── */

/* Solve:  output = sum_i( coeff_i * history_output_i )
 * where coefficients are found by minimizing:
 *   || new_input - sum_i( coeff_i * history_input_i ) ||^2
 *
 * This is a standard linear least squares problem:
 *   A * x = b
 * where A[i][j] = history_input_j[i], b[i] = new_input[i].
 *
 * We solve via normal equations: (A^T A) x = A^T b. */
static int linfit(const ivp_mi_interpolator_t *interp,
                   const ivp_mi_vector_t *new_input,
                   int n_vecs,
                   float *coeffs,      /* [n_vecs] output */
                   float *residual)    /* output: fitting residual */
{
    int m = interp->input_dim;  /* rows of A */
    int n = n_vecs;             /* columns of A */
    if (n > m) return -1;       /* underdetermined */

    /* Build A^T A (n×n symmetric) and A^T b (n×1) in scratch */
    float *ata = interp->scratch;           /* [n * n] */
    float *atb = interp->scratch + n * n;   /* [n] */

    memset(ata, 0, (size_t)(n * n) * sizeof(float));
    memset(atb, 0, (size_t)n * sizeof(float));

    for (int i = 0; i < n; i++) {
        const float *ai = interp->inputs[i]->data;
        for (int j = i; j < n; j++) {
            const float *aj = interp->inputs[j]->data;
            float dot = 0.0f;
            for (int k = 0; k < m; k++) {
                dot += ai[k] * aj[k];
            }
            ata[i * n + j] = dot;
            ata[j * n + i] = dot;
        }
        float dot_b = 0.0f;
        for (int k = 0; k < m; k++) {
            dot_b += ai[k] * new_input->data[k];
        }
        atb[i] = dot_b;
    }

    /* Solve via Gauss elimination with partial pivoting */
    /* Augmented matrix: [ata | atb] stored in-place */
    float *aug = interp->scratch + n * n + n;  /* [n * (n+1)] */
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            aug[i * (n + 1) + j] = ata[i * n + j];
        }
        aug[i * (n + 1) + n] = atb[i];
    }

    for (int col = 0; col < n; col++) {
        /* Partial pivot */
        int pivot = col;
        float max_val = fabsf(aug[col * (n + 1) + col]);
        for (int row = col + 1; row < n; row++) {
            float v = fabsf(aug[row * (n + 1) + col]);
            if (v > max_val) {
                max_val = v;
                pivot = row;
            }
        }
        if (max_val < 1e-12f) return -1;  /* singular */

        /* Swap rows */
        if (pivot != col) {
            for (int j = col; j <= n; j++) {
                float tmp = aug[col * (n + 1) + j];
                aug[col * (n + 1) + j] = aug[pivot * (n + 1) + j];
                aug[pivot * (n + 1) + j] = tmp;
            }
        }

        /* Eliminate */
        float diag = aug[col * (n + 1) + col];
        for (int row = col + 1; row < n; row++) {
            float factor = aug[row * (n + 1) + col] / diag;
            for (int j = col; j <= n; j++) {
                aug[row * (n + 1) + j] -= factor * aug[col * (n + 1) + j];
            }
        }
    }

    /* Back-substitute */
    for (int i = n - 1; i >= 0; i--) {
        float sum = aug[i * (n + 1) + n];
        for (int j = i + 1; j < n; j++) {
            sum -= aug[i * (n + 1) + j] * coeffs[j];
        }
        coeffs[i] = sum / aug[i * (n + 1) + i];
    }

    /* Compute residual: || new_input - sum(coeffs[i] * inputs[i]) ||^2 */
    float res = 0.0f;
    for (int k = 0; k < m; k++) {
        float predicted = 0.0f;
        for (int i = 0; i < n; i++) {
            predicted += coeffs[i] * interp->inputs[i]->data[k];
        }
        float diff = new_input->data[k] - predicted;
        res += diff * diff;
    }
    *residual = sqrtf(res);

    return 0;
}

/* ── Check interpolation ──────────────────────────────────────────────── */

int ivp_mi_check_interpolation(ivp_mi_interpolator_t *interp,
                                 const ivp_mi_vector_t *new_input,
                                 int max_vectors,
                                 float max_residual,
                                 ivp_mi_vector_t *output)
{
    if (!interp || !new_input || !output) return -1;
    if (interp->n_entries < 1) return -1;
    if (new_input->n != interp->input_dim) return -1;
    if (output->n != interp->output_dim) return -1;

    int n_vecs = interp->n_entries;
    if (n_vecs > max_vectors) n_vecs = max_vectors;
    if (n_vecs > interp->input_dim) n_vecs = interp->input_dim;

    /* Try fitting with n_vecs, then fewer if that fails */
    float coeffs[IVP_MI_MAX_HISTORY];
    float residual;

    for (int try_n = n_vecs; try_n >= 1; try_n--) {
        if (linfit(interp, new_input, try_n, coeffs, &residual) == 0) {
            if (residual <= max_residual) {
                /* Interpolation succeeded: compute output */
                memset(output->data, 0,
                       (size_t)interp->output_dim * sizeof(float));
                for (int i = 0; i < try_n; i++) {
                    for (int k = 0; k < interp->output_dim; k++) {
                        output->data[k] += coeffs[i] *
                                            interp->outputs[i]->data[k];
                    }
                }
                return 0;
            }
        }
    }

    return -1;  /* interpolation failed */
}
