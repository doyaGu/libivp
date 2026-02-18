/* ivp_active_value.c -- Active Values implementation
 *
 * Reactive compute graph: push-based propagation of value changes
 * through a dependency tree.
 *
 * Ported from ivu_active_value.cxx (Ipion SDK).
 */
#include <ivp/utility/ivp_active_value.h>
#include <string.h>
#include <math.h>


/* ── Internal: allocate a zeroed active float ──────────────────────────── */

static ivp_active_float_t *alloc_float(const ivp_allocator_t *alloc)
{
    ivp_active_float_t *af = (ivp_active_float_t *)ivp_alloc(alloc, sizeof(*af));
    if (!af) return NULL;
    memset(af, 0, sizeof(*af));
    af->alloc = alloc;
    return af;
}

/* ── Internal: recompute a non-terminal float ───────────────────────────── */

static double recompute_value(ivp_active_float_t *af)
{
    switch (af->type) {
    case IVP_ACTIVE_FLOAT_TERMINAL:
        return af->value;  /* terminals are set directly */
    case IVP_ACTIVE_FLOAT_MULT:
        return af->src_a->value * af->src_b->value;
    case IVP_ACTIVE_FLOAT_ADD:
        return af->src_a->value + af->src_b->value;
    case IVP_ACTIVE_FLOAT_CLAMP: {
        double v = af->src_a->value;
        if (v < af->clamp_min) v = af->clamp_min;
        if (v > af->clamp_max) v = af->clamp_max;
        return v;
    }
    case IVP_ACTIVE_FLOAT_NEGATE:
        return -af->src_a->value;
    case IVP_ACTIVE_FLOAT_SWITCH:
        return (af->src_condition->value != 0)
            ? af->src_true_val->value
            : af->src_false_val->value;
    case IVP_ACTIVE_FLOAT_SINE: {
        double t = af->src_a->value;
        return af->osc_amplitude * sin(2.0 * IVP_PI_D * af->osc_frequency * (t + af->osc_time_shift))
               + af->osc_null_level;
    }
    case IVP_ACTIVE_FLOAT_SQUARE: {
        double t = af->src_a->value;
        double phase = fmod(t * af->osc_frequency, 1.0);
        if (phase < 0.0) phase += 1.0;
        return phase < 0.5 ? af->osc_time_shift : af->osc_null_level;
    }
    case IVP_ACTIVE_FLOAT_PULSE: {
        double t = af->src_a->value;
        double phase = fmod(t * af->osc_frequency, 1.0);
        if (phase < 0.0) phase += 1.0;
        double duty = (af->osc_pulse_m2 != 0)
            ? (double)af->osc_pulse_m1 / (double)af->osc_pulse_m2
            : 0.5;
        return phase < duty ? af->osc_time_shift : af->osc_null_level;
    }
    case IVP_ACTIVE_FLOAT_SUB:
        return af->src_a->value - af->src_b->value;
    case IVP_ACTIVE_FLOAT_ADD_MULTIPLE:
        return af->src_a->value + af->osc_factor * af->src_b->value;
    }
    return 0.0;
}

/* ── Internal: propagate change through dependents ──────────────────────── */

static void propagate_float(ivp_active_float_t *af)
{
    double old = af->value;
    double nv  = recompute_value(af);
    if (nv == old) return;

    af->value = nv;

    /* Notify listeners */
    for (int i = 0; i < af->listener_count; i++) {
        af->listener_fns[i](af->listener_ctxs[i], af, old, nv);
    }

    /* Propagate to downstream dependents */
    for (int i = 0; i < af->dependent_count; i++) {
        propagate_float(af->dependents[i]);
    }
}

/* ── Internal: register a dependent link ────────────────────────────────── */

static bool add_float_dependent(ivp_active_float_t *parent,
                                  ivp_active_float_t *child)
{
    if (parent->dependent_count >= IVP_ACTIVE_MAX_DEPENDENTS) return false;
    parent->dependents[parent->dependent_count++] = child;
    parent->ref_count++;
    return true;
}

static bool add_int_dependent(ivp_active_int_t *parent,
                                ivp_active_float_t *child)
{
    if (parent->dependent_count >= IVP_ACTIVE_MAX_DEPENDENTS) return false;
    parent->dependents[parent->dependent_count++] = child;
    parent->ref_count++;
    return true;
}

/* ── Terminal constructors ──────────────────────────────────────────────── */

ivp_active_float_t *ivp_active_float_terminal_create(const ivp_allocator_t *alloc,
                                                       const char *name,
                                                       double initial_value)
{
    if (!alloc) alloc = ivp_default_allocator();
    ivp_active_float_t *af = alloc_float(alloc);
    if (!af) return NULL;
    af->type = IVP_ACTIVE_FLOAT_TERMINAL;
    af->name = name;
    af->value = initial_value;
    af->ref_count = 1;
    return af;
}

ivp_active_int_t *ivp_active_int_terminal_create(const ivp_allocator_t *alloc,
                                                   const char *name,
                                                   int initial_value)
{
    if (!alloc) alloc = ivp_default_allocator();
    ivp_active_int_t *ai = (ivp_active_int_t *)ivp_alloc(alloc, sizeof(*ai));
    if (!ai) return NULL;
    memset(ai, 0, sizeof(*ai));
    ai->alloc = alloc;
    ai->type = IVP_ACTIVE_INT_TERMINAL;
    ai->name = name;
    ai->value = initial_value;
    ai->ref_count = 1;
    return ai;
}

/* ── Compound constructors ──────────────────────────────────────────────── */

ivp_active_float_t *ivp_active_float_mult_create(const ivp_allocator_t *alloc,
                                                    const char *name,
                                                    ivp_active_float_t *a,
                                                    ivp_active_float_t *b)
{
    if (!alloc) alloc = ivp_default_allocator();
    ivp_active_float_t *af = alloc_float(alloc);
    if (!af) return NULL;
    af->type = IVP_ACTIVE_FLOAT_MULT;
    af->name = name;
    af->src_a = a;
    af->src_b = b;
    af->value = a->value * b->value;
    af->ref_count = 1;
    add_float_dependent(a, af);
    add_float_dependent(b, af);
    return af;
}

ivp_active_float_t *ivp_active_float_add_create(const ivp_allocator_t *alloc,
                                                   const char *name,
                                                   ivp_active_float_t *a,
                                                   ivp_active_float_t *b)
{
    if (!alloc) alloc = ivp_default_allocator();
    ivp_active_float_t *af = alloc_float(alloc);
    if (!af) return NULL;
    af->type = IVP_ACTIVE_FLOAT_ADD;
    af->name = name;
    af->src_a = a;
    af->src_b = b;
    af->value = a->value + b->value;
    af->ref_count = 1;
    add_float_dependent(a, af);
    add_float_dependent(b, af);
    return af;
}

ivp_active_float_t *ivp_active_float_clamp_create(const ivp_allocator_t *alloc,
                                                     const char *name,
                                                     ivp_active_float_t *source,
                                                     double min_val,
                                                     double max_val)
{
    if (!alloc) alloc = ivp_default_allocator();
    ivp_active_float_t *af = alloc_float(alloc);
    if (!af) return NULL;
    af->type = IVP_ACTIVE_FLOAT_CLAMP;
    af->name = name;
    af->src_a = source;
    af->clamp_min = min_val;
    af->clamp_max = max_val;
    double v = source->value;
    if (v < min_val) v = min_val;
    if (v > max_val) v = max_val;
    af->value = v;
    af->ref_count = 1;
    add_float_dependent(source, af);
    return af;
}

ivp_active_float_t *ivp_active_float_negate_create(const ivp_allocator_t *alloc,
                                                      const char *name,
                                                      ivp_active_float_t *source)
{
    if (!alloc) alloc = ivp_default_allocator();
    ivp_active_float_t *af = alloc_float(alloc);
    if (!af) return NULL;
    af->type = IVP_ACTIVE_FLOAT_NEGATE;
    af->name = name;
    af->src_a = source;
    af->value = -source->value;
    af->ref_count = 1;
    add_float_dependent(source, af);
    return af;
}

ivp_active_float_t *ivp_active_float_switch_create(const ivp_allocator_t *alloc,
                                                      const char *name,
                                                      ivp_active_int_t *condition,
                                                      ivp_active_float_t *false_val,
                                                      ivp_active_float_t *true_val)
{
    if (!alloc) alloc = ivp_default_allocator();
    ivp_active_float_t *af = alloc_float(alloc);
    if (!af) return NULL;
    af->type = IVP_ACTIVE_FLOAT_SWITCH;
    af->name = name;
    af->src_condition = condition;
    af->src_false_val = false_val;
    af->src_true_val = true_val;
    af->value = (condition->value != 0)
        ? true_val->value
        : false_val->value;
    af->ref_count = 1;
    add_int_dependent(condition, af);
    add_float_dependent(false_val, af);
    add_float_dependent(true_val, af);
    return af;
}

/* ── Oscillator / extended compound constructors ─────────────────────────── */

ivp_active_float_t *ivp_active_float_sine_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *time_source,
    double frequency, double amplitude, double null_level, double time_shift)
{
    if (!alloc) alloc = ivp_default_allocator();
    ivp_active_float_t *af = alloc_float(alloc);
    if (!af) return NULL;
    af->type = IVP_ACTIVE_FLOAT_SINE;
    af->name = name;
    af->src_a = time_source;
    af->osc_frequency = frequency;
    af->osc_amplitude = amplitude;
    af->osc_null_level = null_level;
    af->osc_time_shift = time_shift;
    af->value = amplitude * sin(2.0 * IVP_PI_D * frequency * (time_source->value + time_shift))
                + null_level;
    af->ref_count = 1;
    add_float_dependent(time_source, af);
    return af;
}

ivp_active_float_t *ivp_active_float_square_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *time_source,
    double frequency, double low_val, double high_val)
{
    if (!alloc) alloc = ivp_default_allocator();
    ivp_active_float_t *af = alloc_float(alloc);
    if (!af) return NULL;
    af->type = IVP_ACTIVE_FLOAT_SQUARE;
    af->name = name;
    af->src_a = time_source;
    af->osc_frequency = frequency;
    af->osc_null_level = low_val;     /* low_val stored in null_level */
    af->osc_time_shift = high_val;    /* high_val stored in time_shift */
    double phase = fmod(time_source->value * frequency, 1.0);
    if (phase < 0.0) phase += 1.0;
    af->value = phase < 0.5 ? high_val : low_val;
    af->ref_count = 1;
    add_float_dependent(time_source, af);
    return af;
}

ivp_active_float_t *ivp_active_float_pulse_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *time_source,
    double frequency, int m1, int m2, double low_val, double high_val)
{
    if (!alloc) alloc = ivp_default_allocator();
    ivp_active_float_t *af = alloc_float(alloc);
    if (!af) return NULL;
    af->type = IVP_ACTIVE_FLOAT_PULSE;
    af->name = name;
    af->src_a = time_source;
    af->osc_frequency = frequency;
    af->osc_null_level = low_val;
    af->osc_time_shift = high_val;
    af->osc_pulse_m1 = m1;
    af->osc_pulse_m2 = m2;
    double duty = (m2 != 0) ? (double)m1 / (double)m2 : 0.5;
    double phase = fmod(time_source->value * frequency, 1.0);
    if (phase < 0.0) phase += 1.0;
    af->value = phase < duty ? high_val : low_val;
    af->ref_count = 1;
    add_float_dependent(time_source, af);
    return af;
}

ivp_active_float_t *ivp_active_float_sub_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *a, ivp_active_float_t *b)
{
    if (!alloc) alloc = ivp_default_allocator();
    ivp_active_float_t *af = alloc_float(alloc);
    if (!af) return NULL;
    af->type = IVP_ACTIVE_FLOAT_SUB;
    af->name = name;
    af->src_a = a;
    af->src_b = b;
    af->value = a->value - b->value;
    af->ref_count = 1;
    add_float_dependent(a, af);
    add_float_dependent(b, af);
    return af;
}

ivp_active_float_t *ivp_active_float_add_multiple_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *a, ivp_active_float_t *b, double factor)
{
    if (!alloc) alloc = ivp_default_allocator();
    ivp_active_float_t *af = alloc_float(alloc);
    if (!af) return NULL;
    af->type = IVP_ACTIVE_FLOAT_ADD_MULTIPLE;
    af->name = name;
    af->src_a = a;
    af->src_b = b;
    af->osc_factor = factor;
    af->value = a->value + factor * b->value;
    af->ref_count = 1;
    add_float_dependent(a, af);
    add_float_dependent(b, af);
    return af;
}

/* ── Setters with propagation ───────────────────────────────────────────── */

void ivp_active_float_set(ivp_active_float_t *af, double new_value)
{
    if (af->type != IVP_ACTIVE_FLOAT_TERMINAL) return;
    if (af->value == new_value) return;

    double old = af->value;
    af->value = new_value;

    /* Notify own listeners */
    for (int i = 0; i < af->listener_count; i++) {
        af->listener_fns[i](af->listener_ctxs[i], af, old, new_value);
    }

    /* Propagate to dependents */
    for (int i = 0; i < af->dependent_count; i++) {
        propagate_float(af->dependents[i]);
    }
}

void ivp_active_int_set(ivp_active_int_t *ai, int new_value)
{
    if (ai->type != IVP_ACTIVE_INT_TERMINAL) return;
    if (ai->value == new_value) return;

    int old = ai->value;
    ai->value = new_value;

    /* Notify own listeners */
    for (int i = 0; i < ai->listener_count; i++) {
        ai->listener_fns[i](ai->listener_ctxs[i], ai, old, new_value);
    }

    /* Propagate to dependent float switches */
    for (int i = 0; i < ai->dependent_count; i++) {
        propagate_float(ai->dependents[i]);
    }
}

/* ── Reference counting ────────────────────────────────────────────────── */

void ivp_active_float_add_ref(ivp_active_float_t *af)
{
    if (af) af->ref_count++;
}

void ivp_active_int_add_ref(ivp_active_int_t *ai)
{
    if (ai) ai->ref_count++;
}

void ivp_active_float_remove_ref(ivp_active_float_t *af)
{
    if (!af) return;
    af->ref_count--;
    if (af->ref_count <= 0) {
        /* Remove self from parent dependents */
        if (af->src_a) {
            for (int i = 0; i < af->src_a->dependent_count; i++) {
                if (af->src_a->dependents[i] == af) {
                    af->src_a->dependents[i] =
                        af->src_a->dependents[--af->src_a->dependent_count];
                    af->src_a->ref_count--;
                    break;
                }
            }
        }
        if (af->src_b) {
            for (int i = 0; i < af->src_b->dependent_count; i++) {
                if (af->src_b->dependents[i] == af) {
                    af->src_b->dependents[i] =
                        af->src_b->dependents[--af->src_b->dependent_count];
                    af->src_b->ref_count--;
                    break;
                }
            }
        }
        if (af->src_condition) {
            ivp_active_int_t *ci = af->src_condition;
            for (int i = 0; i < ci->dependent_count; i++) {
                if (ci->dependents[i] == af) {
                    ci->dependents[i] = ci->dependents[--ci->dependent_count];
                    ci->ref_count--;
                    break;
                }
            }
        }
        if (af->src_false_val) {
            for (int i = 0; i < af->src_false_val->dependent_count; i++) {
                if (af->src_false_val->dependents[i] == af) {
                    af->src_false_val->dependents[i] =
                        af->src_false_val->dependents[--af->src_false_val->dependent_count];
                    af->src_false_val->ref_count--;
                    break;
                }
            }
        }
        if (af->src_true_val) {
            for (int i = 0; i < af->src_true_val->dependent_count; i++) {
                if (af->src_true_val->dependents[i] == af) {
                    af->src_true_val->dependents[i] =
                        af->src_true_val->dependents[--af->src_true_val->dependent_count];
                    af->src_true_val->ref_count--;
                    break;
                }
            }
        }
        ivp_free(af->alloc, af, sizeof(*af));
    }
}

void ivp_active_int_remove_ref(ivp_active_int_t *ai)
{
    if (!ai) return;
    ai->ref_count--;
    if (ai->ref_count <= 0) {
        ivp_free(ai->alloc, ai, sizeof(*ai));
    }
}

/* ── Listener API ───────────────────────────────────────────────────────── */

bool ivp_active_float_add_listener(ivp_active_float_t *af,
                                     ivp_active_float_listener_fn_t fn,
                                     void *ctx)
{
    if (af->listener_count >= IVP_ACTIVE_MAX_LISTENERS) return false;
    af->listener_fns[af->listener_count] = fn;
    af->listener_ctxs[af->listener_count] = ctx;
    af->listener_count++;
    af->ref_count++;
    return true;
}

bool ivp_active_float_remove_listener(ivp_active_float_t *af,
                                        ivp_active_float_listener_fn_t fn,
                                        void *ctx)
{
    for (int i = 0; i < af->listener_count; i++) {
        if (af->listener_fns[i] == fn && af->listener_ctxs[i] == ctx) {
            af->listener_count--;
            af->listener_fns[i] = af->listener_fns[af->listener_count];
            af->listener_ctxs[i] = af->listener_ctxs[af->listener_count];
            af->ref_count--;
            return true;
        }
    }
    return false;
}

bool ivp_active_int_add_listener(ivp_active_int_t *ai,
                                   ivp_active_int_listener_fn_t fn,
                                   void *ctx)
{
    if (ai->listener_count >= IVP_ACTIVE_MAX_LISTENERS) return false;
    ai->listener_fns[ai->listener_count] = fn;
    ai->listener_ctxs[ai->listener_count] = ctx;
    ai->listener_count++;
    ai->ref_count++;
    return true;
}

bool ivp_active_int_remove_listener(ivp_active_int_t *ai,
                                      ivp_active_int_listener_fn_t fn,
                                      void *ctx)
{
    for (int i = 0; i < ai->listener_count; i++) {
        if (ai->listener_fns[i] == fn && ai->listener_ctxs[i] == ctx) {
            ai->listener_count--;
            ai->listener_fns[i] = ai->listener_fns[ai->listener_count];
            ai->listener_ctxs[i] = ai->listener_ctxs[ai->listener_count];
            ai->ref_count--;
            return true;
        }
    }
    return false;
}
