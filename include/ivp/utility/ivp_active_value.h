/* ivp_active_value.h -- Active Values: reactive value compute graph
 *
 * Active Values are simple doubles or integers that automatically
 * propagate changes through a dependency tree.  When a terminal
 * value changes, all dependent values are recalculated immediately
 * (push policy).
 *
 * Types:
 *   Terminal  -- leaf value (user-settable)
 *   Mult      -- product of two active values
 *   Add       -- sum of two active values
 *   Clamp     -- clamped value
 *   Switch    -- if-then-else driven by an active int
 *   Negate    -- negation of an active value
 *
 * Listeners can be attached to any active value to receive
 * notifications on change.
 *
 * Ported from ivu_active_value.hxx / IVP_U_Active_Float et al.
 */
#ifndef IVP_ACTIVE_VALUE_H
#define IVP_ACTIVE_VALUE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Forward declarations ───────────────────────────────────────────────── */

typedef struct ivp_active_float   ivp_active_float_t;
typedef struct ivp_active_int     ivp_active_int_t;

/* ── Listener callback ──────────────────────────────────────────────────── */

typedef void (*ivp_active_float_listener_fn_t)(void *ctx,
                                                ivp_active_float_t *source,
                                                double old_value,
                                                double new_value);

typedef void (*ivp_active_int_listener_fn_t)(void *ctx,
                                              ivp_active_int_t *source,
                                              int old_value,
                                              int new_value);

/* ── Active float types ─────────────────────────────────────────────────── */

typedef enum ivp_active_float_type {
    IVP_ACTIVE_FLOAT_TERMINAL      = 0,
    IVP_ACTIVE_FLOAT_MULT          = 1,
    IVP_ACTIVE_FLOAT_ADD           = 2,
    IVP_ACTIVE_FLOAT_CLAMP         = 3,
    IVP_ACTIVE_FLOAT_NEGATE        = 4,
    IVP_ACTIVE_FLOAT_SWITCH        = 5,
    IVP_ACTIVE_FLOAT_SINE          = 6,   /* amplitude * sin(2*PI*freq*(t + shift)) + offset */
    IVP_ACTIVE_FLOAT_SQUARE        = 7,   /* fmod(t*freq,1) < 0.5 ? high : low */
    IVP_ACTIVE_FLOAT_PULSE         = 8,   /* fmod(t*freq,1) < m1/m2 ? high : low */
    IVP_ACTIVE_FLOAT_SUB           = 9,   /* a - b */
    IVP_ACTIVE_FLOAT_ADD_MULTIPLE  = 10   /* a + factor * b */
} ivp_active_float_type_t;

/* ── max listeners per node ─────────────────────────────────────────────── */

#define IVP_ACTIVE_MAX_LISTENERS 8
#define IVP_ACTIVE_MAX_DEPENDENTS 8

/* ── Active float node ──────────────────────────────────────────────────── */

struct ivp_active_float {
    ivp_active_float_type_t type;
    const char             *name;           /* optional, for debugging */
    double                  value;          /* cached current value */
    int                     ref_count;      /* reference count */
    const ivp_allocator_t  *alloc;          /* allocator used to create this node */

    /* Listeners (downstream notifications) */
    ivp_active_float_listener_fn_t listener_fns[IVP_ACTIVE_MAX_LISTENERS];
    void                          *listener_ctxs[IVP_ACTIVE_MAX_LISTENERS];
    int                            listener_count;

    /* Dependent active values (for propagation) */
    ivp_active_float_t *dependents[IVP_ACTIVE_MAX_DEPENDENTS];
    int                  dependent_count;

    /* Source pointers (for non-terminal types) */
    ivp_active_float_t *src_a;
    ivp_active_float_t *src_b;
    ivp_active_int_t   *src_condition;      /* for switch type */
    ivp_active_float_t *src_true_val;       /* for switch type */
    ivp_active_float_t *src_false_val;      /* for switch type */

    /* Clamp bounds */
    double              clamp_min;
    double              clamp_max;

    /* Oscillator parameters (for SINE, SQUARE, PULSE, ADD_MULTIPLE) */
    double              osc_frequency;
    double              osc_amplitude;
    double              osc_null_level;     /* offset for SINE; low_val for SQUARE/PULSE */
    double              osc_time_shift;     /* phase shift for SINE; high_val for SQUARE/PULSE */
    int                 osc_pulse_m1;       /* duty numerator for PULSE */
    int                 osc_pulse_m2;       /* duty denominator for PULSE */
    double              osc_factor;         /* factor for ADD_MULTIPLE */
};

/* ── Active int node ────────────────────────────────────────────────────── */

typedef enum ivp_active_int_type {
    IVP_ACTIVE_INT_TERMINAL = 0
} ivp_active_int_type_t;

struct ivp_active_int {
    ivp_active_int_type_t type;
    const char           *name;
    int                   value;
    int                   ref_count;
    const ivp_allocator_t *alloc;          /* allocator used to create this node */

    ivp_active_int_listener_fn_t listener_fns[IVP_ACTIVE_MAX_LISTENERS];
    void                        *listener_ctxs[IVP_ACTIVE_MAX_LISTENERS];
    int                          listener_count;

    /* Dependent float switches (for propagation) */
    ivp_active_float_t *dependents[IVP_ACTIVE_MAX_DEPENDENTS];
    int                  dependent_count;
};

/* ── Terminal constructors ──────────────────────────────────────────────── */

/* Create a terminal float active value. name is borrowed (not copied).
 * Pass NULL for alloc to use the default allocator. */
ivp_active_float_t *ivp_active_float_terminal_create(const ivp_allocator_t *alloc,
                                                       const char *name,
                                                       double initial_value);

/* Create a terminal int active value. */
ivp_active_int_t *ivp_active_int_terminal_create(const ivp_allocator_t *alloc,
                                                   const char *name,
                                                   int initial_value);

/* ── Compound constructors ──────────────────────────────────────────────── */

/* result = a * b */
ivp_active_float_t *ivp_active_float_mult_create(const ivp_allocator_t *alloc,
                                                    const char *name,
                                                    ivp_active_float_t *a,
                                                    ivp_active_float_t *b);

/* result = a + b */
ivp_active_float_t *ivp_active_float_add_create(const ivp_allocator_t *alloc,
                                                   const char *name,
                                                   ivp_active_float_t *a,
                                                   ivp_active_float_t *b);

/* result = clamp(source, min, max) */
ivp_active_float_t *ivp_active_float_clamp_create(const ivp_allocator_t *alloc,
                                                     const char *name,
                                                     ivp_active_float_t *source,
                                                     double min_val,
                                                     double max_val);

/* result = -source */
ivp_active_float_t *ivp_active_float_negate_create(const ivp_allocator_t *alloc,
                                                      const char *name,
                                                      ivp_active_float_t *source);

/* result = (condition != 0) ? true_val : false_val */
ivp_active_float_t *ivp_active_float_switch_create(const ivp_allocator_t *alloc,
                                                      const char *name,
                                                      ivp_active_int_t *condition,
                                                      ivp_active_float_t *false_val,
                                                      ivp_active_float_t *true_val);

/* result = amplitude * sin(2*PI*freq*(t + shift)) + null_level
 * time_source is used as src_a */
ivp_active_float_t *ivp_active_float_sine_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *time_source,
    double frequency, double amplitude, double null_level, double time_shift);

/* result = fmod(t*freq,1) < 0.5 ? high_val : low_val */
ivp_active_float_t *ivp_active_float_square_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *time_source,
    double frequency, double low_val, double high_val);

/* result = fmod(t*freq,1) < (m1/m2) ? high_val : low_val */
ivp_active_float_t *ivp_active_float_pulse_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *time_source,
    double frequency, int m1, int m2, double low_val, double high_val);

/* result = a - b */
ivp_active_float_t *ivp_active_float_sub_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *a, ivp_active_float_t *b);

/* result = a + factor * b */
ivp_active_float_t *ivp_active_float_add_multiple_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *a, ivp_active_float_t *b, double factor);

/* ── Getters / Setters ──────────────────────────────────────────────────── */

/* Get current cached value (O(1), no recomputation). */
static inline double ivp_active_float_get(const ivp_active_float_t *af) {
    return af->value;
}

static inline int ivp_active_int_get(const ivp_active_int_t *ai) {
    return ai->value;
}

/* Set terminal value and trigger cascade to all dependents. */
void ivp_active_float_set(ivp_active_float_t *af, double new_value);
void ivp_active_int_set(ivp_active_int_t *ai, int new_value);

/* ── Reference counting ────────────────────────────────────────────────── */

/* Add a reference. Call when storing a pointer to an active value. */
void ivp_active_float_add_ref(ivp_active_float_t *af);
void ivp_active_int_add_ref(ivp_active_int_t *ai);

/* Remove a reference. When ref_count reaches 0, the node is freed. */
void ivp_active_float_remove_ref(ivp_active_float_t *af);
void ivp_active_int_remove_ref(ivp_active_int_t *ai);

/* ── Listener API ───────────────────────────────────────────────────────── */

/* Add a listener to an active float. Returns true on success. */
bool ivp_active_float_add_listener(ivp_active_float_t *af,
                                     ivp_active_float_listener_fn_t fn,
                                     void *ctx);

/* Remove a listener. Returns true if found and removed. */
bool ivp_active_float_remove_listener(ivp_active_float_t *af,
                                        ivp_active_float_listener_fn_t fn,
                                        void *ctx);

bool ivp_active_int_add_listener(ivp_active_int_t *ai,
                                   ivp_active_int_listener_fn_t fn,
                                   void *ctx);

bool ivp_active_int_remove_listener(ivp_active_int_t *ai,
                                      ivp_active_int_listener_fn_t fn,
                                      void *ctx);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTIVE_VALUE_H */
