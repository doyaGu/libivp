/* ivp_active_value.h -- Active Values: reactive value compute graph
 *
 * Active Values are doubles or integers that propagate changes through a
 * dependency graph (push policy).  Port of ivu_active_value.hxx /
 * ivu_active_value.cxx (IVP_U_Active_Float, IVP_U_Active_Int and
 * IVP_U_Active_Value_Manager).
 *
 * Float node types (reference class):
 *   Terminal      IVP_U_Active_Terminal_Double  (user settable)
 *   Mult / Add / Sub / Add_Multiple             IVP_U_Active_Mult/Add/Sub/Add_Multiple
 *   Limit (Clamp) IVP_U_Active_Limit
 *   Switch        IVP_U_Active_Switch (driven by an active int)
 *   Sine / Square / Pulse                       IVP_U_Active_Sine/Square/Pulse
 *   Negate        libivp extension (no reference class)
 * Int node types:
 *   Terminal      IVP_U_Active_Terminal_Int
 *   Test_Range    IVP_U_Active_Test_Range (1 if low <= test <= high)
 *
 * Propagation semantics:
 *   - Every terminal update carries one propagation stamp. With stable
 *     internals, stamps are local to the terminal or its manager; a manager
 *     refresh uses one stamp for current_time and all delayed terminals.
 *     The default reference policy uses a process-wide change meter.
 *   - Every derived node remembers its last propagation stamp and ignores
 *     further notifications carrying the same stamp.
 *     Within one terminal change each node is therefore recomputed at most
 *     once; in a diamond (A->B, A->C, B,C->D) D is
 *     computed when the FIRST of its parents notifies it and later parent
 *     notifications are ignored, so D may keep a value computed from a
 *     not-yet-updated parent (reference behaviour, reproduced).
 *   - A node notifies its derived entries (dependent nodes and listeners,
 *     one ordered list) only if its value changed, iterating from the
 *     most recently added entry to the first one.
 *   - Construction evaluates each node once through the same guard.  Square
 *     nodes are not evaluated at construction, matching the reference.
 *
 * With IVP_STABLE_INTERNAL_STATE off, the reference process-wide change
 * meter includes its construction-at-meter-zero quirk.
 *
 * Ownership (libivp): nodes are reference counted; create returns a node
 * with ref_count 1, every dependent node, listener and manager
 * registration adds one reference.  remove_ref at 0 unlinks the node from
 * its sources (dropping their references) and frees it.  Names are copied.
 */
#ifndef IVP_ACTIVE_VALUE_H
#define IVP_ACTIVE_VALUE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef IVP_STABLE_INTERNAL_STATE
#define IVP_STABLE_INTERNAL_STATE 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ── Forward declarations ───────────────────────────────────────────────── */

typedef struct ivp_active_float          ivp_active_float_t;
typedef struct ivp_active_int            ivp_active_int_t;
typedef struct ivp_active_value_manager  ivp_active_value_manager_t;

/* Reference IVP_ACTIVE_FLOAT_CURRENT_TIME_NAME */
#define IVP_ACTIVE_FLOAT_CURRENT_TIME_NAME "current_time"

/* ── Listener callback ──────────────────────────────────────────────────── *
 * Called (like IVP_U_Active_Float_Listener::active_float_changed) when the
 * value of `source` changed; old_value is the value before the change. */

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
    IVP_ACTIVE_FLOAT_MULT          = 1,   /* a * b */
    IVP_ACTIVE_FLOAT_ADD           = 2,   /* a + b */
    IVP_ACTIVE_FLOAT_LIMIT         = 3,   /* IVP_U_Active_Limit: clamp(a, low, high) */
    IVP_ACTIVE_FLOAT_NEGATE        = 4,   /* -a (libivp extension) */
    IVP_ACTIVE_FLOAT_SWITCH        = 5,   /* cond ? true_val : false_val */
    IVP_ACTIVE_FLOAT_SINE          = 6,   /* sin(t*freq + time_shift)*amp + null_level */
    IVP_ACTIVE_FLOAT_SQUARE        = 7,   /* ((int)(t*freq) & 1) ? high : low */
    IVP_ACTIVE_FLOAT_PULSE         = 8,   /* ((int)(t*freq*m2) % m2 < m1) ? high : low */
    IVP_ACTIVE_FLOAT_SUB           = 9,   /* a - b */
    IVP_ACTIVE_FLOAT_ADD_MULTIPLE  = 10   /* a + factor * b */
} ivp_active_float_type_t;

/* Old name of IVP_ACTIVE_FLOAT_LIMIT, kept for API compatibility. */
#define IVP_ACTIVE_FLOAT_CLAMP IVP_ACTIVE_FLOAT_LIMIT

typedef enum ivp_active_int_type {
    IVP_ACTIVE_INT_TERMINAL   = 0,
    IVP_ACTIVE_INT_TEST_RANGE = 1    /* (low <= test <= high) ? 1 : 0 */
} ivp_active_int_type_t;

/* ── Derived entry (dependent node or listener), one ordered list ─────── */

typedef enum ivp_active_derived_kind {
    IVP_ACTIVE_DERIVED_FLOAT_NODE     = 0,
    IVP_ACTIVE_DERIVED_INT_NODE       = 1,
    IVP_ACTIVE_DERIVED_FLOAT_LISTENER = 2,
    IVP_ACTIVE_DERIVED_INT_LISTENER   = 3
} ivp_active_derived_kind_t;

typedef struct ivp_active_derived {
    ivp_active_derived_kind_t kind;
    void *target;                       /* node pointer, or listener ctx */
    union {
        ivp_active_float_listener_fn_t f;
        ivp_active_int_listener_fn_t   i;
    } fn;
} ivp_active_derived_t;

/* ── Active float node ──────────────────────────────────────────────────── */

struct ivp_active_float {
    ivp_active_float_type_t type;
    char                   *name;           /* owned copy (may be NULL) */
    double                  value;          /* IVP_U_Active_Float::double_value */
    double                  old_value;      /* terminal: value at last update */
#if !IVP_STABLE_INTERNAL_STATE
    int                     last_update;    /* reference change meter */
#else
    uint64_t                last_update_source;
    uint64_t                last_update_sequence;
    uint64_t                update_source;   /* terminal source identity */
    uint64_t                update_sequence;
#endif
    int                     ref_count;
    const ivp_allocator_t  *alloc;
    ivp_active_value_manager_t *manager;    /* backlink (l_mod_manager) */

    /* Derived entries in insertion order (notified in reverse order) */
    ivp_active_derived_t   *derived;
    int                     derived_count;
    int                     derived_cap;
    int                     dependent_count;   /* node entries in derived[] */
    int                     listener_count;    /* listener entries in derived[] */

    /* Source pointers (for non-terminal types) */
    ivp_active_float_t *src_a;
    ivp_active_float_t *src_b;
    ivp_active_int_t   *src_condition;      /* for switch type */
    ivp_active_float_t *src_true_val;       /* for switch type */
    ivp_active_float_t *src_false_val;      /* for switch type */

    /* Limit bounds */
    double              clamp_min;
    double              clamp_max;

    /* Oscillator parameters */
    double              osc_frequency;
    double              osc_amplitude;
    double              osc_null_level;     /* SINE null level; SQUARE/PULSE low value */
    double              osc_time_shift;     /* SINE time shift; SQUARE/PULSE high value */
    int                 osc_pulse_m1;       /* PULSE: m1 out of m2 */
    int                 osc_pulse_m2;
    double              osc_factor;         /* ADD_MULTIPLE factor */
};

/* ── Active int node ────────────────────────────────────────────────────── */

struct ivp_active_int {
    ivp_active_int_type_t type;
    char                 *name;
    int                   value;
    int                   old_value;
#if !IVP_STABLE_INTERNAL_STATE
    int                   last_update;
#else
    uint64_t              last_update_source;
    uint64_t              last_update_sequence;
    uint64_t              update_source;
    uint64_t              update_sequence;
#endif
    int                   ref_count;
    const ivp_allocator_t *alloc;
    ivp_active_value_manager_t *manager;

    ivp_active_derived_t *derived;
    int                   derived_count;
    int                   derived_cap;
    int                   dependent_count;
    int                   listener_count;

    /* TEST_RANGE sources */
    ivp_active_float_t   *src_test;
    ivp_active_float_t   *src_low;
    ivp_active_float_t   *src_high;
};

/* ── Reference change meter ─────────────────────────────────────────────── */

/* IVP_U_Active_Float::change_meter in a reference-internals build.  The
 * stable-internals build has no process-wide propagation state and returns -1. */
int  ivp_active_value_change_meter(void);

/* ── Terminal constructors ──────────────────────────────────────────────── */

/* Pass NULL for alloc to use the default allocator.  name is copied. */
ivp_active_float_t *ivp_active_float_terminal_create(const ivp_allocator_t *alloc,
                                                       const char *name,
                                                       double initial_value);

ivp_active_int_t *ivp_active_int_terminal_create(const ivp_allocator_t *alloc,
                                                   const char *name,
                                                   int initial_value);

/* ── Compound constructors ──────────────────────────────────────────────── */

ivp_active_float_t *ivp_active_float_mult_create(const ivp_allocator_t *alloc,
                                                    const char *name,
                                                    ivp_active_float_t *a,
                                                    ivp_active_float_t *b);

ivp_active_float_t *ivp_active_float_add_create(const ivp_allocator_t *alloc,
                                                   const char *name,
                                                   ivp_active_float_t *a,
                                                   ivp_active_float_t *b);

/* IVP_U_Active_Limit: result = source clamped to [low_val, high_val]
 * (low checked first, then high). */
ivp_active_float_t *ivp_active_float_limit_create(const ivp_allocator_t *alloc,
                                                     const char *name,
                                                     ivp_active_float_t *source,
                                                     double low_val,
                                                     double high_val);

/* Old name of ivp_active_float_limit_create. */
ivp_active_float_t *ivp_active_float_clamp_create(const ivp_allocator_t *alloc,
                                                     const char *name,
                                                     ivp_active_float_t *source,
                                                     double min_val,
                                                     double max_val);

/* result = -source (libivp extension) */
ivp_active_float_t *ivp_active_float_negate_create(const ivp_allocator_t *alloc,
                                                      const char *name,
                                                      ivp_active_float_t *source);

/* IVP_U_Active_Switch: result = (condition != 0) ? true_val : false_val.
 * NOTE: libivp argument order is (condition, false_val, true_val); the
 * reference constructor takes (cond, true, false).  Dependencies are
 * registered in the reference order (cond, true, false). */
ivp_active_float_t *ivp_active_float_switch_create(const ivp_allocator_t *alloc,
                                                      const char *name,
                                                      ivp_active_int_t *condition,
                                                      ivp_active_float_t *false_val,
                                                      ivp_active_float_t *true_val);

/* IVP_U_Active_Sine: result = sin(t * frequency + time_shift) * amplitude
 *                             + null_level   (no 2*PI factor; frequency is
 * an angular frequency in rad/s despite the reference "Hz" comment). */
ivp_active_float_t *ivp_active_float_sine_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *time_source,
    double frequency, double amplitude, double null_level, double time_shift);

/* IVP_U_Active_Square: result = ((int)(t * frequency) & 1) ? high_val : low_val
 * (period 2/frequency, starts low).  Not evaluated at construction: the
 * value is 0 until the time source changes (reference behaviour). */
ivp_active_float_t *ivp_active_float_square_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *time_source,
    double frequency, double low_val, double high_val);

/* IVP_U_Active_Pulse: result = ((int)(t*frequency*m2) % m2 < m1) ? high : low */
ivp_active_float_t *ivp_active_float_pulse_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *time_source,
    double frequency, int m1, int m2, double low_val, double high_val);

/* result = a - b */
ivp_active_float_t *ivp_active_float_sub_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *a, ivp_active_float_t *b);

/* result = a + factor * b */
ivp_active_float_t *ivp_active_float_add_multiple_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *a, ivp_active_float_t *b, double factor);

/* IVP_U_Active_Test_Range (int node): (low <= test <= high) ? 1 : 0 */
ivp_active_int_t *ivp_active_int_test_range_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *test,
    ivp_active_float_t *low_val, ivp_active_float_t *high_val);

/* ── Getters / Setters ──────────────────────────────────────────────────── */

static inline double ivp_active_float_get(const ivp_active_float_t *af) {
    return af->value;
}

/* IVP_U_Active_Float::get_float_value */
static inline float ivp_active_float_get_float(const ivp_active_float_t *af) {
    return (float)af->value;
}

static inline int ivp_active_int_get(const ivp_active_int_t *ai) {
    return ai->value;
}

static inline const char *ivp_active_float_name(const ivp_active_float_t *af) {
    return af->name;
}

static inline const char *ivp_active_int_name(const ivp_active_int_t *ai) {
    return ai->name;
}

/* IVP_U_Active_Terminal_Double::set_double(value): sets the value and
 * propagates with a new source-local stamp if the value differs from the
 * last propagated value.  A reference-internals build increments the
 * process-wide change meter instead.  No-op for non-terminal nodes. */
void ivp_active_float_set(ivp_active_float_t *af, double new_value);
void ivp_active_int_set(ivp_active_int_t *ai, int new_value);

/* set_double / set_int with the reference `delayed` flag: when delayed and
 * the terminal is registered with a manager, the value is stored at once
 * but propagation is deferred to ivp_active_value_manager_update_delayed
 * (or refresh_psi). */
void ivp_active_float_set_delayed(ivp_active_float_t *af, double new_value, bool delayed);
void ivp_active_int_set_delayed(ivp_active_int_t *ai, int new_value, bool delayed);

/* ── Reference counting ────────────────────────────────────────────────── */

void ivp_active_float_add_ref(ivp_active_float_t *af);
void ivp_active_int_add_ref(ivp_active_int_t *ai);
void ivp_active_float_remove_ref(ivp_active_float_t *af);
void ivp_active_int_remove_ref(ivp_active_int_t *ai);

/* ── Listener API ───────────────────────────────────────────────────────── */

/* Append a listener (IVP_U_Active_Float::add_dependency); adds a reference. */
bool ivp_active_float_add_listener(ivp_active_float_t *af,
                                     ivp_active_float_listener_fn_t fn,
                                     void *ctx);

/* Remove the most recently added matching listener (order preserved). */
bool ivp_active_float_remove_listener(ivp_active_float_t *af,
                                        ivp_active_float_listener_fn_t fn,
                                        void *ctx);

bool ivp_active_int_add_listener(ivp_active_int_t *ai,
                                   ivp_active_int_listener_fn_t fn,
                                   void *ctx);

bool ivp_active_int_remove_listener(ivp_active_int_t *ai,
                                      ivp_active_int_listener_fn_t fn,
                                      void *ctx);

/* ── Active value manager (IVP_U_Active_Value_Manager) ─────────────────── *
 * A name → active value hash plus the delayed-update queues.  On creation
 * the generic terminals "double_null" (0.0) and "current_time" (0.0) are
 * installed.  Nodes registered in the manager hold one manager reference;
 * terminals created by the manager (install/create/get-by-number) are owned
 * by the manager (the returned pointer is borrowed). */

ivp_active_value_manager_t *ivp_active_value_manager_create(const ivp_allocator_t *alloc);
void ivp_active_value_manager_destroy(ivp_active_value_manager_t *m);

/* insert_active_float: false (nothing done) if the node has no name or the
 * name is already used. */
bool ivp_active_value_manager_insert_float(ivp_active_value_manager_t *m,
                                           ivp_active_float_t *af);
void ivp_active_value_manager_remove_float(ivp_active_value_manager_t *m,
                                           ivp_active_float_t *af);
bool ivp_active_value_manager_insert_int(ivp_active_value_manager_t *m,
                                         ivp_active_int_t *ai);
void ivp_active_value_manager_remove_int(ivp_active_value_manager_t *m,
                                         ivp_active_int_t *ai);

/* install_active_float: existing node with that name, else a new terminal */
ivp_active_float_t *ivp_active_value_manager_install_float(ivp_active_value_manager_t *m,
                                                           const char *name, double value);
/* create_active_float: NULL if the name exists, else a new terminal */
ivp_active_float_t *ivp_active_value_manager_create_float(ivp_active_value_manager_t *m,
                                                          const char *name, double value);
/* get_active_float_by_name: NULL if not found, except for names that look
 * like a number ("-", "." prefixes allowed, then a digit): a terminal with
 * atof(name) is created and registered on the fly. */
ivp_active_float_t *ivp_active_value_manager_get_float(ivp_active_value_manager_t *m,
                                                       const char *name);

ivp_active_int_t *ivp_active_value_manager_install_int(ivp_active_value_manager_t *m,
                                                       const char *name, int value);
ivp_active_int_t *ivp_active_value_manager_create_int(ivp_active_value_manager_t *m,
                                                      const char *name, int value);
ivp_active_int_t *ivp_active_value_manager_get_int(ivp_active_value_manager_t *m,
                                                   const char *name);

/* The "current_time" terminal. */
ivp_active_float_t *ivp_active_value_manager_current_time(ivp_active_value_manager_t *m);

/* delay_active_float / delay_active_int: queue a terminal (once) */
void ivp_active_value_manager_delay_float(ivp_active_value_manager_t *m,
                                          ivp_active_float_t *af);
void ivp_active_value_manager_delay_int(ivp_active_value_manager_t *m,
                                        ivp_active_int_t *ai);

/* update_delayed_active_values: propagate all queued floats (in queue
 * order), then all queued ints, and clear the queues. */
void ivp_active_value_manager_update_delayed(ivp_active_value_manager_t *m);

/* refresh_psi_active_values(time): propagate current_time and all queued
 * terminals as one manager-local update.  A reference-internals build uses
 * current_time.set(time), then update_delayed_active_values(). */
void ivp_active_value_manager_refresh_psi(ivp_active_value_manager_t *m, double time);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ACTIVE_VALUE_H */
