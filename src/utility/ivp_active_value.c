/* ivp_active_value.c -- Active Values implementation
 *
 * Port of ivu_active_value.cxx (IVP_U_Active_Float / IVP_U_Active_Int /
 * IVP_U_Active_Value_Manager) with the reference change-meter based
 * propagation.  See ivp_active_value.h for the semantics.
 */
#include <ivp/utility/ivp_active_value.h>
#include <ivp/containers/ivp_vhash.h>
#include <ivp/utility/ivp_hash.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdatomic.h>

/* ── Global change meter (IVP_U_Active_Float::change_meter = -1) ─────── */

static atomic_int g_change_meter = -1;

static inline int change_meter_increment(void) {
    int old = atomic_fetch_add_explicit(&g_change_meter, 1,
                                        memory_order_relaxed);
    return (int)((unsigned int)old + 1u);
}

int ivp_active_value_change_meter(void) {
    return atomic_load_explicit(&g_change_meter, memory_order_relaxed);
}

/* ── Helpers ────────────────────────────────────────────────────────────── */

static char *dup_name(const ivp_allocator_t *alloc, const char *name) {
    if (!name) return NULL;
    size_t n = strlen(name) + 1;
    char *s = (char *)ivp_alloc(alloc, n);
    if (s) memcpy(s, name, n);
    return s;
}

static void free_name(const ivp_allocator_t *alloc, char *name) {
    if (name) ivp_free(alloc, name, strlen(name) + 1);
}

static bool derived_reserve(const ivp_allocator_t *alloc, ivp_active_derived_t **arr,
                            int *count, int *cap) {
    if (*count < *cap) return true;
    int new_cap = *cap ? *cap * 2 : 4;
    ivp_active_derived_t *n = (ivp_active_derived_t *)ivp_realloc(
        alloc, *arr, (size_t)*cap * sizeof(**arr), (size_t)new_cap * sizeof(**arr));
    if (!n) return false;
    *arr = n;
    *cap = new_cap;
    return true;
}

/* IVP_U_Vector::remove: removes the LAST matching entry, order preserved. */
static int derived_find_last_node(const ivp_active_derived_t *arr, int count,
                                  ivp_active_derived_kind_t kind, const void *target) {
    for (int i = count - 1; i >= 0; i--) {
        if (arr[i].kind == kind && arr[i].target == target) return i;
    }
    return -1;
}

static int derived_find_last_float_listener(const ivp_active_derived_t *arr, int count,
                                            ivp_active_float_listener_fn_t fn,
                                            const void *ctx) {
    for (int i = count - 1; i >= 0; i--) {
        if (arr[i].kind == IVP_ACTIVE_DERIVED_FLOAT_LISTENER && arr[i].target == ctx &&
            arr[i].fn.f == fn)
            return i;
    }
    return -1;
}

static int derived_find_last_int_listener(const ivp_active_derived_t *arr, int count,
                                          ivp_active_int_listener_fn_t fn,
                                          const void *ctx) {
    for (int i = count - 1; i >= 0; i--) {
        if (arr[i].kind == IVP_ACTIVE_DERIVED_INT_LISTENER && arr[i].target == ctx &&
            arr[i].fn.i == fn)
            return i;
    }
    return -1;
}

static void derived_remove_at(ivp_active_derived_t *arr, int *count, int index) {
    (*count)--;
    for (int j = index; j < *count; j++) arr[j] = arr[j + 1];
}

/* ── Allocation ─────────────────────────────────────────────────────────── */

static ivp_active_float_t *alloc_float(const ivp_allocator_t *alloc, const char *name,
                                       ivp_active_float_type_t type) {
    if (!alloc) alloc = ivp_default_allocator();
    ivp_active_float_t *af = (ivp_active_float_t *)ivp_alloc(alloc, sizeof(*af));
    if (!af) return NULL;
    memset(af, 0, sizeof(*af));
    af->alloc = alloc;
    af->type = type;
    af->name = dup_name(alloc, name);
    if (name && !af->name) {
        ivp_free(alloc, af, sizeof(*af));
        return NULL;
    }
    af->ref_count = 1;
    af->last_update = 0; /* "should be updated" */
    return af;
}

static ivp_active_int_t *alloc_int(const ivp_allocator_t *alloc, const char *name,
                                   ivp_active_int_type_t type) {
    if (!alloc) alloc = ivp_default_allocator();
    ivp_active_int_t *ai = (ivp_active_int_t *)ivp_alloc(alloc, sizeof(*ai));
    if (!ai) return NULL;
    memset(ai, 0, sizeof(*ai));
    ai->alloc = alloc;
    ai->type = type;
    ai->name = dup_name(alloc, name);
    if (name && !ai->name) {
        ivp_free(alloc, ai, sizeof(*ai));
        return NULL;
    }
    ai->ref_count = 1;
    ai->last_update = 0;
    return ai;
}

/* ── Dependency registration (add_dependency) ───────────────────────────── */

static bool float_add_derived(ivp_active_float_t *parent, ivp_active_derived_t e) {
    if (!derived_reserve(parent->alloc, &parent->derived, &parent->derived_count,
                         &parent->derived_cap))
        return false;
    parent->derived[parent->derived_count++] = e;
    if (e.kind == IVP_ACTIVE_DERIVED_FLOAT_NODE || e.kind == IVP_ACTIVE_DERIVED_INT_NODE)
        parent->dependent_count++;
    else
        parent->listener_count++;
    parent->ref_count++;
    return true;
}

static bool int_add_derived(ivp_active_int_t *parent, ivp_active_derived_t e) {
    if (!derived_reserve(parent->alloc, &parent->derived, &parent->derived_count,
                         &parent->derived_cap))
        return false;
    parent->derived[parent->derived_count++] = e;
    if (e.kind == IVP_ACTIVE_DERIVED_FLOAT_NODE || e.kind == IVP_ACTIVE_DERIVED_INT_NODE)
        parent->dependent_count++;
    else
        parent->listener_count++;
    parent->ref_count++;
    return true;
}

static ivp_active_derived_t node_entry(ivp_active_derived_kind_t kind, void *target) {
    ivp_active_derived_t e;
    memset(&e, 0, sizeof(e));
    e.kind = kind;
    e.target = target;
    return e;
}

static void float_depend_on_float(ivp_active_float_t *node, ivp_active_float_t *parent) {
    float_add_derived(parent, node_entry(IVP_ACTIVE_DERIVED_FLOAT_NODE, node));
}

/* remove_dependency: drop the entry and the parent's reference */
static void float_remove_derived_node(ivp_active_float_t *parent,
                                      ivp_active_derived_kind_t kind, void *node) {
    int i = derived_find_last_node(parent->derived, parent->derived_count, kind, node);
    IVP_ASSERT(i >= 0); /* derived_mods.remove (ivu_vector.hxx:224) */
    if (i < 0) return;
    derived_remove_at(parent->derived, &parent->derived_count, i);
    parent->dependent_count--;
    ivp_active_float_remove_ref(parent);
}

static void int_remove_derived_node(ivp_active_int_t *parent, void *node) {
    int i = derived_find_last_node(parent->derived, parent->derived_count,
                                   IVP_ACTIVE_DERIVED_FLOAT_NODE, node);
    IVP_ASSERT(i >= 0); /* derived_mods.remove (ivu_vector.hxx:224) */
    if (i < 0) return;
    derived_remove_at(parent->derived, &parent->derived_count, i);
    parent->dependent_count--;
    ivp_active_int_remove_ref(parent);
}

/* ── Propagation ────────────────────────────────────────────────────────── */

static void float_node_changed(ivp_active_float_t *af, int change_meter);
static void int_node_changed(ivp_active_int_t *ai, int change_meter);

/* IVP_U_Active_Float::update_derived: notify derived entries from the last
 * one to the first one. */
static void float_update_derived(ivp_active_float_t *af, double old_value,
                                 int change_meter) {
    for (int i = af->derived_count - 1; i >= 0; i--) {
        if (i >= af->derived_count) continue; /* list shrank during a callback */
        ivp_active_derived_t e = af->derived[i];
        switch (e.kind) {
        case IVP_ACTIVE_DERIVED_FLOAT_NODE:
            float_node_changed((ivp_active_float_t *)e.target, change_meter);
            break;
        case IVP_ACTIVE_DERIVED_INT_NODE:
            int_node_changed((ivp_active_int_t *)e.target, change_meter);
            break;
        case IVP_ACTIVE_DERIVED_FLOAT_LISTENER:
            e.fn.f(e.target, af, old_value, af->value);
            break;
        case IVP_ACTIVE_DERIVED_INT_LISTENER:
            break;
        }
    }
}

/* IVP_U_Active_Int::update_derived */
static void int_update_derived(ivp_active_int_t *ai, int old_value,
                               int change_meter) {
    for (int i = ai->derived_count - 1; i >= 0; i--) {
        if (i >= ai->derived_count) continue;
        ivp_active_derived_t e = ai->derived[i];
        switch (e.kind) {
        case IVP_ACTIVE_DERIVED_FLOAT_NODE:
            /* IVP_U_Active_Switch::active_int_changed → active_float_changed */
            float_node_changed((ivp_active_float_t *)e.target, change_meter);
            break;
        case IVP_ACTIVE_DERIVED_INT_NODE:
            int_node_changed((ivp_active_int_t *)e.target, change_meter);
            break;
        case IVP_ACTIVE_DERIVED_INT_LISTENER:
            e.fn.i(e.target, ai, old_value, ai->value);
            break;
        case IVP_ACTIVE_DERIVED_FLOAT_LISTENER:
            break;
        }
    }
}

static double float_compute(const ivp_active_float_t *af) {
    switch (af->type) {
    case IVP_ACTIVE_FLOAT_TERMINAL:
        return af->value;
    case IVP_ACTIVE_FLOAT_MULT:
        return af->src_a->value * af->src_b->value;
    case IVP_ACTIVE_FLOAT_ADD:
        return af->src_a->value + af->src_b->value;
    case IVP_ACTIVE_FLOAT_SUB:
        return af->src_a->value - af->src_b->value;
    case IVP_ACTIVE_FLOAT_ADD_MULTIPLE:
        return af->src_a->value + af->osc_factor * af->src_b->value;
    case IVP_ACTIVE_FLOAT_LIMIT: {
        double new_val = af->src_a->value;
        if (new_val < af->clamp_min) new_val = af->clamp_min;
        if (new_val > af->clamp_max) new_val = af->clamp_max;
        return new_val;
    }
    case IVP_ACTIVE_FLOAT_NEGATE:
        return -af->src_a->value;
    case IVP_ACTIVE_FLOAT_SWITCH: {
        double cond_val = (double)af->src_condition->value;
        return (cond_val != 0.0) ? af->src_true_val->value : af->src_false_val->value;
    }
    case IVP_ACTIVE_FLOAT_SINE: {
        double time = af->src_a->value;
        return sin(time * af->osc_frequency + af->osc_time_shift) * af->osc_amplitude +
               af->osc_null_level;
    }
    case IVP_ACTIVE_FLOAT_SQUARE: {
        double time = af->src_a->value;
        int h = (int)(time * af->osc_frequency) & 1;
        return h ? af->osc_time_shift /* high */ : af->osc_null_level /* low */;
    }
    case IVP_ACTIVE_FLOAT_PULSE: {
        double time = af->src_a->value;
        int m2 = af->osc_pulse_m2;
        int val = (m2 != 0) ? (int)(time * af->osc_frequency * m2) % m2 : 0;
        return (val < af->osc_pulse_m1) ? af->osc_time_shift : af->osc_null_level;
    }
    }
    return 0.0;
}

/* active_float_changed() of a derived float node */
static void float_node_changed(ivp_active_float_t *af, int change_meter) {
    if (af->type == IVP_ACTIVE_FLOAT_TERMINAL) return;
    if (af->last_update == change_meter) return; /* already valid */
    af->last_update = change_meter;

    double new_val = float_compute(af);
    if (new_val != af->value) {
        double old = af->value;
        af->value = new_val;
        float_update_derived(af, old, change_meter);
    }
}

/* IVP_U_Active_Test_Range::active_float_changed */
static void int_node_changed(ivp_active_int_t *ai, int change_meter) {
    if (ai->type != IVP_ACTIVE_INT_TEST_RANGE) return;
    if (ai->last_update == change_meter) return;
    ai->last_update = change_meter;

    double cmp_val = ai->src_test->value;
    double low_val = ai->src_low->value;
    double high_val = ai->src_high->value;
    int new_val = ((cmp_val >= low_val) && (cmp_val <= high_val)) ? 1 : 0;
    if (new_val != ai->value) {
        int old = ai->value;
        ai->value = new_val;
        int_update_derived(ai, old, change_meter);
    }
}

/* IVP_U_Active_Terminal_Double::update_float */
static void terminal_float_update(ivp_active_float_t *af, int change_meter) {
    if (af->value != af->old_value) {
        double old = af->old_value;
        af->old_value = af->value;
        float_update_derived(af, old, change_meter);
    }
}

/* IVP_U_Active_Terminal_Int::update_int */
static void terminal_int_update(ivp_active_int_t *ai, int change_meter) {
    if (ai->value != ai->old_value) {
        int old = ai->old_value;
        ai->old_value = ai->value;
        int_update_derived(ai, old, change_meter);
    }
}

/* ── Terminal constructors ──────────────────────────────────────────────── */

ivp_active_float_t *ivp_active_float_terminal_create(const ivp_allocator_t *alloc,
                                                       const char *name,
                                                       double initial_value) {
    ivp_active_float_t *af = alloc_float(alloc, name, IVP_ACTIVE_FLOAT_TERMINAL);
    if (!af) return NULL;
    af->value = initial_value;
    af->old_value = initial_value;
    return af;
}

ivp_active_int_t *ivp_active_int_terminal_create(const ivp_allocator_t *alloc,
                                                   const char *name,
                                                   int initial_value) {
    ivp_active_int_t *ai = alloc_int(alloc, name, IVP_ACTIVE_INT_TERMINAL);
    if (!ai) return NULL;
    ai->value = initial_value;
    ai->old_value = initial_value;
    return ai;
}

/* ── Compound constructors ──────────────────────────────────────────────── */

static ivp_active_float_t *binary_create(const ivp_allocator_t *alloc, const char *name,
                                         ivp_active_float_type_t type,
                                         ivp_active_float_t *a, ivp_active_float_t *b,
                                         double factor) {
    ivp_active_float_t *af = alloc_float(alloc, name, type);
    if (!af) return NULL;
    af->src_a = a;
    af->src_b = b;
    af->osc_factor = factor;
    float_depend_on_float(af, a); /* mod0->add_dependency(this) */
    float_depend_on_float(af, b); /* mod1->add_dependency(this) */
    float_node_changed(af, ivp_active_value_change_meter());
    /* this->active_float_changed(this) */
    return af;
}

ivp_active_float_t *ivp_active_float_mult_create(const ivp_allocator_t *alloc,
                                                    const char *name,
                                                    ivp_active_float_t *a,
                                                    ivp_active_float_t *b) {
    return binary_create(alloc, name, IVP_ACTIVE_FLOAT_MULT, a, b, 0.0);
}

ivp_active_float_t *ivp_active_float_add_create(const ivp_allocator_t *alloc,
                                                   const char *name,
                                                   ivp_active_float_t *a,
                                                   ivp_active_float_t *b) {
    return binary_create(alloc, name, IVP_ACTIVE_FLOAT_ADD, a, b, 0.0);
}

ivp_active_float_t *ivp_active_float_sub_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *a, ivp_active_float_t *b) {
    return binary_create(alloc, name, IVP_ACTIVE_FLOAT_SUB, a, b, 0.0);
}

ivp_active_float_t *ivp_active_float_add_multiple_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *a, ivp_active_float_t *b, double factor) {
    return binary_create(alloc, name, IVP_ACTIVE_FLOAT_ADD_MULTIPLE, a, b, factor);
}

ivp_active_float_t *ivp_active_float_limit_create(const ivp_allocator_t *alloc,
                                                     const char *name,
                                                     ivp_active_float_t *source,
                                                     double low_val,
                                                     double high_val) {
    ivp_active_float_t *af = alloc_float(alloc, name, IVP_ACTIVE_FLOAT_LIMIT);
    if (!af) return NULL;
    af->src_a = source;
    af->clamp_min = low_val;
    af->clamp_max = high_val;
    float_depend_on_float(af, source);
    float_node_changed(af, ivp_active_value_change_meter());
    return af;
}

ivp_active_float_t *ivp_active_float_clamp_create(const ivp_allocator_t *alloc,
                                                     const char *name,
                                                     ivp_active_float_t *source,
                                                     double min_val,
                                                     double max_val) {
    return ivp_active_float_limit_create(alloc, name, source, min_val, max_val);
}

ivp_active_float_t *ivp_active_float_negate_create(const ivp_allocator_t *alloc,
                                                      const char *name,
                                                      ivp_active_float_t *source) {
    ivp_active_float_t *af = alloc_float(alloc, name, IVP_ACTIVE_FLOAT_NEGATE);
    if (!af) return NULL;
    af->src_a = source;
    float_depend_on_float(af, source);
    float_node_changed(af, ivp_active_value_change_meter());
    return af;
}

ivp_active_float_t *ivp_active_float_switch_create(const ivp_allocator_t *alloc,
                                                      const char *name,
                                                      ivp_active_int_t *condition,
                                                      ivp_active_float_t *false_val,
                                                      ivp_active_float_t *true_val) {
    ivp_active_float_t *af = alloc_float(alloc, name, IVP_ACTIVE_FLOAT_SWITCH);
    if (!af) return NULL;
    af->src_condition = condition;
    af->src_false_val = false_val;
    af->src_true_val = true_val;
    /* reference order: mod_cond, mod_true, mod_false */
    int_add_derived(condition, node_entry(IVP_ACTIVE_DERIVED_FLOAT_NODE, af));
    float_depend_on_float(af, true_val);
    float_depend_on_float(af, false_val);
    float_node_changed(af, ivp_active_value_change_meter());
    return af;
}

static ivp_active_float_t *osc_create(const ivp_allocator_t *alloc, const char *name,
                                      ivp_active_float_type_t type,
                                      ivp_active_float_t *time_source) {
    ivp_active_float_t *af = alloc_float(alloc, name, type);
    if (!af) return NULL;
    af->src_a = time_source;
    float_depend_on_float(af, time_source);
    return af;
}

ivp_active_float_t *ivp_active_float_sine_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *time_source,
    double frequency, double amplitude, double null_level, double time_shift) {
    ivp_active_float_t *af = osc_create(alloc, name, IVP_ACTIVE_FLOAT_SINE, time_source);
    if (!af) return NULL;
    af->osc_frequency = frequency;
    af->osc_amplitude = amplitude;
    af->osc_null_level = null_level;
    af->osc_time_shift = time_shift;
    float_node_changed(af, ivp_active_value_change_meter());
    /* no derived existing yet */
    return af;
}

ivp_active_float_t *ivp_active_float_square_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *time_source,
    double frequency, double low_val, double high_val) {
    ivp_active_float_t *af = osc_create(alloc, name, IVP_ACTIVE_FLOAT_SQUARE, time_source);
    if (!af) return NULL;
    af->osc_frequency = frequency;
    af->osc_null_level = low_val;  /* low_val stored in null_level */
    af->osc_time_shift = high_val; /* high_val stored in time_shift */
    /* reference constructor does not evaluate the node */
    return af;
}

ivp_active_float_t *ivp_active_float_pulse_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *time_source,
    double frequency, int m1, int m2, double low_val, double high_val) {
    ivp_active_float_t *af = osc_create(alloc, name, IVP_ACTIVE_FLOAT_PULSE, time_source);
    if (!af) return NULL;
    af->osc_frequency = frequency;
    af->osc_null_level = low_val;
    af->osc_time_shift = high_val;
    af->osc_pulse_m1 = m1;
    af->osc_pulse_m2 = m2;
    float_node_changed(af, ivp_active_value_change_meter());
    return af;
}

ivp_active_int_t *ivp_active_int_test_range_create(const ivp_allocator_t *alloc,
    const char *name, ivp_active_float_t *test,
    ivp_active_float_t *low_val, ivp_active_float_t *high_val) {
    ivp_active_int_t *ai = alloc_int(alloc, name, IVP_ACTIVE_INT_TEST_RANGE);
    if (!ai) return NULL;
    ai->src_test = test;
    ai->src_low = low_val;
    ai->src_high = high_val;
    float_add_derived(test, node_entry(IVP_ACTIVE_DERIVED_INT_NODE, ai));
    float_add_derived(low_val, node_entry(IVP_ACTIVE_DERIVED_INT_NODE, ai));
    float_add_derived(high_val, node_entry(IVP_ACTIVE_DERIVED_INT_NODE, ai));
    int_node_changed(ai, ivp_active_value_change_meter());
    return ai;
}

/* ── Setters with propagation ───────────────────────────────────────────── */

void ivp_active_float_set_delayed(ivp_active_float_t *af, double new_value, bool delayed) {
    if (af->type != IVP_ACTIVE_FLOAT_TERMINAL) return;
    af->value = new_value;
    int change_meter = change_meter_increment();
    if (delayed && af->manager) {
        ivp_active_value_manager_delay_float(af->manager, af);
    } else {
        terminal_float_update(af, change_meter);
    }
}

void ivp_active_int_set_delayed(ivp_active_int_t *ai, int new_value, bool delayed) {
    if (ai->type != IVP_ACTIVE_INT_TERMINAL) return;
    ai->value = new_value;
    int change_meter = change_meter_increment();
    /* IVP_U_Active_Float::change_meter++ */
    if (delayed && ai->manager) {
        ivp_active_value_manager_delay_int(ai->manager, ai);
    } else {
        terminal_int_update(ai, change_meter);
    }
}

void ivp_active_float_set(ivp_active_float_t *af, double new_value) {
    ivp_active_float_set_delayed(af, new_value, false);
}

void ivp_active_int_set(ivp_active_int_t *ai, int new_value) {
    ivp_active_int_set_delayed(ai, new_value, false);
}

/* ── Reference counting ────────────────────────────────────────────────── */

void ivp_active_float_add_ref(ivp_active_float_t *af) {
    if (af) af->ref_count++;
}

void ivp_active_int_add_ref(ivp_active_int_t *ai) {
    if (ai) ai->ref_count++;
}

void ivp_active_float_remove_ref(ivp_active_float_t *af) {
    if (!af) return;
    af->ref_count--;
    if (af->ref_count > 0) return;

    /* destructor: remove_dependency on all sources */
    if (af->src_condition) int_remove_derived_node(af->src_condition, af);
    if (af->src_true_val) float_remove_derived_node(af->src_true_val, IVP_ACTIVE_DERIVED_FLOAT_NODE, af);
    if (af->src_false_val) float_remove_derived_node(af->src_false_val, IVP_ACTIVE_DERIVED_FLOAT_NODE, af);
    if (af->src_a) float_remove_derived_node(af->src_a, IVP_ACTIVE_DERIVED_FLOAT_NODE, af);
    if (af->src_b) float_remove_derived_node(af->src_b, IVP_ACTIVE_DERIVED_FLOAT_NODE, af);

    if (af->derived) {
        ivp_free(af->alloc, af->derived, (size_t)af->derived_cap * sizeof(*af->derived));
    }
    free_name(af->alloc, af->name);
    IVP_ASSERT(af->ref_count == 0); /* ~IVP_U_Active_Value */
    ivp_free(af->alloc, af, sizeof(*af));
}

void ivp_active_int_remove_ref(ivp_active_int_t *ai) {
    if (!ai) return;
    ai->ref_count--;
    if (ai->ref_count > 0) return;

    if (ai->src_test) float_remove_derived_node(ai->src_test, IVP_ACTIVE_DERIVED_INT_NODE, ai);
    if (ai->src_low) float_remove_derived_node(ai->src_low, IVP_ACTIVE_DERIVED_INT_NODE, ai);
    if (ai->src_high) float_remove_derived_node(ai->src_high, IVP_ACTIVE_DERIVED_INT_NODE, ai);

    if (ai->derived) {
        ivp_free(ai->alloc, ai->derived, (size_t)ai->derived_cap * sizeof(*ai->derived));
    }
    free_name(ai->alloc, ai->name);
    IVP_ASSERT(ai->ref_count == 0); /* ~IVP_U_Active_Value */
    ivp_free(ai->alloc, ai, sizeof(*ai));
}

/* ── Listener API ───────────────────────────────────────────────────────── */

bool ivp_active_float_add_listener(ivp_active_float_t *af,
                                     ivp_active_float_listener_fn_t fn,
                                     void *ctx) {
    ivp_active_derived_t e = node_entry(IVP_ACTIVE_DERIVED_FLOAT_LISTENER, ctx);
    e.fn.f = fn;
    return float_add_derived(af, e);
}

bool ivp_active_float_remove_listener(ivp_active_float_t *af,
                                        ivp_active_float_listener_fn_t fn,
                                        void *ctx) {
    int i = derived_find_last_float_listener(af->derived, af->derived_count, fn, ctx);
    IVP_ASSERT(i >= 0); /* remove_dependency: derived_mods.remove (ivu_vector.hxx:224) */
    if (i < 0) return false;
    derived_remove_at(af->derived, &af->derived_count, i);
    af->listener_count--;
    af->ref_count--;
    return true;
}

bool ivp_active_int_add_listener(ivp_active_int_t *ai,
                                   ivp_active_int_listener_fn_t fn,
                                   void *ctx) {
    ivp_active_derived_t e = node_entry(IVP_ACTIVE_DERIVED_INT_LISTENER, ctx);
    e.fn.i = fn;
    return int_add_derived(ai, e);
}

bool ivp_active_int_remove_listener(ivp_active_int_t *ai,
                                      ivp_active_int_listener_fn_t fn,
                                      void *ctx) {
    int i = derived_find_last_int_listener(ai->derived, ai->derived_count, fn, ctx);
    IVP_ASSERT(i >= 0); /* remove_dependency: derived_mods.remove (ivu_vector.hxx:224) */
    if (i < 0) return false;
    derived_remove_at(ai->derived, &ai->derived_count, i);
    ai->listener_count--;
    ai->ref_count--;
    return true;
}

/* ── Active value manager ───────────────────────────────────────────────── */

#define IVP_U_MOD_NAME_HASH_SIZE 16

struct ivp_active_value_manager {
    const ivp_allocator_t *alloc;
    ivp_vhash_t            floats_name_hash;
    ivp_vhash_t            ints_name_hash;
    ivp_active_float_t   **delayed_floats;
    int                    n_delayed_floats, cap_delayed_floats;
    ivp_active_int_t     **delayed_ints;
    int                    n_delayed_ints, cap_delayed_ints;
    ivp_active_float_t    *mod_current_time;
};

/* p_strcmp semantics on the names */
static int name_cmp(const char *s1, const char *s2) {
    if (s1 == NULL) return s2 == NULL ? 0 : 1;
    if (s2 == NULL) return -1;
    return strcmp(s1, s2);
}

static bool float_name_compare(const void *e0, const void *e1) {
    return name_cmp(((const ivp_active_float_t *)e0)->name,
                    ((const ivp_active_float_t *)e1)->name) == 0;
}

static bool int_name_compare(const void *e0, const void *e1) {
    return name_cmp(((const ivp_active_int_t *)e0)->name,
                    ((const ivp_active_int_t *)e1)->name) == 0;
}

/* IVP_Active_Value_Hash::object_to_index: hash_index(name, strlen(name)) */
static uint32_t name_index(const char *name) {
    return ivp_vhash_index(name, (int)strlen(name));
}

static ivp_active_float_t *find_float(ivp_active_value_manager_t *m, const char *name) {
    if (!name) return NULL;
    ivp_active_float_t probe;
    memset(&probe, 0, sizeof(probe));
    union { const char *c; char *p; } u;
    u.c = name;
    probe.name = u.p;
    return (ivp_active_float_t *)ivp_vhash_find(&m->floats_name_hash, &probe, name_index(name));
}

static ivp_active_int_t *find_int(ivp_active_value_manager_t *m, const char *name) {
    if (!name) return NULL;
    ivp_active_int_t probe;
    memset(&probe, 0, sizeof(probe));
    union { const char *c; char *p; } u;
    u.c = name;
    probe.name = u.p;
    return (ivp_active_int_t *)ivp_vhash_find(&m->ints_name_hash, &probe, name_index(name));
}

ivp_active_value_manager_t *ivp_active_value_manager_create(const ivp_allocator_t *alloc) {
    if (!alloc) alloc = ivp_default_allocator();
    ivp_active_value_manager_t *m =
        (ivp_active_value_manager_t *)ivp_alloc(alloc, sizeof(*m));
    if (!m) return NULL;
    memset(m, 0, sizeof(*m));
    m->alloc = alloc;
    ivp_vhash_init(&m->floats_name_hash, alloc, IVP_U_MOD_NAME_HASH_SIZE, float_name_compare);
    ivp_vhash_init(&m->ints_name_hash, alloc, IVP_U_MOD_NAME_HASH_SIZE, int_name_compare);
    if (!m->floats_name_hash.elems || !m->ints_name_hash.elems) {
        ivp_active_value_manager_destroy(m);
        return NULL;
    }

    /* init_active_values_generic: "double_null" and "current_time" */
    ivp_active_float_t *mod_double_null = ivp_active_float_terminal_create(alloc, "double_null", 0.0);
    if (!mod_double_null ||
        !ivp_active_value_manager_insert_float(m, mod_double_null)) {
        if (mod_double_null) ivp_active_float_remove_ref(mod_double_null);
        ivp_active_value_manager_destroy(m);
        return NULL;
    }
    ivp_active_float_remove_ref(mod_double_null); /* manager owns it */

    ivp_active_float_t *mod_current_time =
        ivp_active_float_terminal_create(alloc, IVP_ACTIVE_FLOAT_CURRENT_TIME_NAME, 0.0);
    if (!mod_current_time ||
        !ivp_active_value_manager_insert_float(m, mod_current_time)) {
        if (mod_current_time) ivp_active_float_remove_ref(mod_current_time);
        ivp_active_value_manager_destroy(m);
        return NULL;
    }
    m->mod_current_time = mod_current_time;
    ivp_active_float_remove_ref(mod_current_time); /* manager owns it */
    return m;
}

void ivp_active_value_manager_destroy(ivp_active_value_manager_t *m) {
    if (!m) return;
    /* ~IVP_Active_Value_Hash: remove_reference from the last slot down */
    for (int i = ivp_vhash_capacity(&m->floats_name_hash) - 1; i >= 0; i--) {
        ivp_active_float_t *af = (ivp_active_float_t *)ivp_vhash_element_at(&m->floats_name_hash, i);
        if (af) {
            af->manager = NULL;
            ivp_active_float_remove_ref(af);
        }
    }
    for (int i = ivp_vhash_capacity(&m->ints_name_hash) - 1; i >= 0; i--) {
        ivp_active_int_t *ai = (ivp_active_int_t *)ivp_vhash_element_at(&m->ints_name_hash, i);
        if (ai) {
            ai->manager = NULL;
            ivp_active_int_remove_ref(ai);
        }
    }
    ivp_vhash_destroy(&m->floats_name_hash);
    ivp_vhash_destroy(&m->ints_name_hash);
    if (m->delayed_floats)
        ivp_free(m->alloc, m->delayed_floats, (size_t)m->cap_delayed_floats * sizeof(*m->delayed_floats));
    if (m->delayed_ints)
        ivp_free(m->alloc, m->delayed_ints, (size_t)m->cap_delayed_ints * sizeof(*m->delayed_ints));
    ivp_free(m->alloc, m, sizeof(*m));
}

bool ivp_active_value_manager_insert_float(ivp_active_value_manager_t *m,
                                           ivp_active_float_t *af) {
    const char *name = af->name;
    if (!name || name[0] == 0) { /* p_strlen(name) == 0 */
        ivp_debug_printf("insert_active_float: tried to insert active_IVP_FLOAT without name!\n");
        return false;
    }
    if (find_float(m, name)) {
        ivp_debug_printf("insert_active_float: name '%s' already exists in name_hash!\n", name);
        return false;
    }
    ivp_vhash_add(&m->floats_name_hash, af, name_index(af->name));
    af->ref_count++;
    af->manager = m; /* provide backlink */
    return true;
}

void ivp_active_value_manager_remove_float(ivp_active_value_manager_t *m,
                                           ivp_active_float_t *af) {
    if (!af->name) return;
    ivp_active_float_t *found = (ivp_active_float_t *)ivp_vhash_remove(
        &m->floats_name_hash, af, name_index(af->name));
    if (found) {
        /* drop it from the delayed queue too (it may be freed now) */
        for (int i = 0; i < m->n_delayed_floats; i++) {
            if (m->delayed_floats[i] == found) {
                for (int j = i; j + 1 < m->n_delayed_floats; j++)
                    m->delayed_floats[j] = m->delayed_floats[j + 1];
                m->n_delayed_floats--;
                break;
            }
        }
        if (found == m->mod_current_time) m->mod_current_time = NULL;
        found->manager = NULL;
        ivp_active_float_remove_ref(found);
    }
}

bool ivp_active_value_manager_insert_int(ivp_active_value_manager_t *m,
                                         ivp_active_int_t *ai) {
    const char *name = ai->name;
    if (!name || name[0] == 0) { /* p_strlen(name) == 0 */
        ivp_debug_printf("insert_active_int: tried to insert active_int without name!\n");
        return false;
    }
    if (find_int(m, name)) {
        ivp_debug_printf("insert_active_int: name '%s' already exists in name_hash!\n", name);
        return false;
    }
    ivp_vhash_add(&m->ints_name_hash, ai, name_index(ai->name));
    ai->ref_count++;
    ai->manager = m;
    return true;
}

void ivp_active_value_manager_remove_int(ivp_active_value_manager_t *m,
                                         ivp_active_int_t *ai) {
    if (!ai->name) return;
    ivp_active_int_t *found = (ivp_active_int_t *)ivp_vhash_remove(
        &m->ints_name_hash, ai, name_index(ai->name));
    if (found) {
        for (int i = 0; i < m->n_delayed_ints; i++) {
            if (m->delayed_ints[i] == found) {
                for (int j = i; j + 1 < m->n_delayed_ints; j++)
                    m->delayed_ints[j] = m->delayed_ints[j + 1];
                m->n_delayed_ints--;
                break;
            }
        }
        found->manager = NULL;
        ivp_active_int_remove_ref(found);
    }
}

static ivp_active_float_t *manager_new_float(ivp_active_value_manager_t *m,
                                             const char *name, double value) {
    ivp_active_float_t *atd = ivp_active_float_terminal_create(m->alloc, name, value);
    if (!atd) return NULL;
    if (!ivp_active_value_manager_insert_float(m, atd)) {
        ivp_active_float_remove_ref(atd);
        return NULL;
    }
    ivp_active_float_remove_ref(atd); /* only the manager reference remains */
    return atd;
}

static ivp_active_int_t *manager_new_int(ivp_active_value_manager_t *m,
                                         const char *name, int value) {
    ivp_active_int_t *ati = ivp_active_int_terminal_create(m->alloc, name, value);
    if (!ati) return NULL;
    if (!ivp_active_value_manager_insert_int(m, ati)) {
        ivp_active_int_remove_ref(ati);
        return NULL;
    }
    ivp_active_int_remove_ref(ati);
    return ati;
}

ivp_active_float_t *ivp_active_value_manager_install_float(ivp_active_value_manager_t *m,
                                                           const char *name, double value) {
    ivp_active_float_t *mod = find_float(m, name);
    if (mod) return mod;
    return manager_new_float(m, name, value);
}

ivp_active_float_t *ivp_active_value_manager_create_float(ivp_active_value_manager_t *m,
                                                          const char *name, double value) {
    if (find_float(m, name)) return NULL;
    return manager_new_float(m, name, value);
}

/* "if module name is a number": optional '-', optional '.', then a digit */
static bool name_is_number(const char *name) {
    int pos = 0;
    if (name[pos] == '-') pos++; /* for negative sign */
    if (name[pos] == '.') pos++; /* for omitted leading zero */
    return name[pos] >= '0' && name[pos] <= '9';
}

ivp_active_float_t *ivp_active_value_manager_get_float(ivp_active_value_manager_t *m,
                                                       const char *name) {
    if (!name) return NULL;
    ivp_active_float_t *mod = find_float(m, name);
    if (mod) return mod;
    if (name_is_number(name)) {
        return manager_new_float(m, name, atof(name));
    }
    return NULL;
}

ivp_active_int_t *ivp_active_value_manager_install_int(ivp_active_value_manager_t *m,
                                                       const char *name, int value) {
    ivp_active_int_t *mod = find_int(m, name);
    if (mod) return mod;
    return manager_new_int(m, name, value);
}

ivp_active_int_t *ivp_active_value_manager_create_int(ivp_active_value_manager_t *m,
                                                      const char *name, int value) {
    if (find_int(m, name)) return NULL;
    return manager_new_int(m, name, value);
}

ivp_active_int_t *ivp_active_value_manager_get_int(ivp_active_value_manager_t *m,
                                                   const char *name) {
    if (!name) return NULL;
    ivp_active_int_t *mod = find_int(m, name);
    if (mod) return mod;
    if (name_is_number(name)) {
        return manager_new_int(m, name, atoi(name));
    }
    return NULL;
}

ivp_active_float_t *ivp_active_value_manager_current_time(ivp_active_value_manager_t *m) {
    return m->mod_current_time;
}

void ivp_active_value_manager_delay_float(ivp_active_value_manager_t *m,
                                          ivp_active_float_t *af) {
    for (int i = m->n_delayed_floats - 1; i >= 0; i--) {
        if (m->delayed_floats[i] == af) return; /* index_of(mod) != -1 */
    }
    if (m->n_delayed_floats >= m->cap_delayed_floats) {
        int nc = m->cap_delayed_floats ? m->cap_delayed_floats * 2 : 8;
        ivp_active_float_t **n = (ivp_active_float_t **)ivp_realloc(
            m->alloc, m->delayed_floats, (size_t)m->cap_delayed_floats * sizeof(*n),
            (size_t)nc * sizeof(*n));
        if (!n) return;
        m->delayed_floats = n;
        m->cap_delayed_floats = nc;
    }
    m->delayed_floats[m->n_delayed_floats++] = af;
}

void ivp_active_value_manager_delay_int(ivp_active_value_manager_t *m,
                                        ivp_active_int_t *ai) {
    for (int i = m->n_delayed_ints - 1; i >= 0; i--) {
        if (m->delayed_ints[i] == ai) return;
    }
    if (m->n_delayed_ints >= m->cap_delayed_ints) {
        int nc = m->cap_delayed_ints ? m->cap_delayed_ints * 2 : 8;
        ivp_active_int_t **n = (ivp_active_int_t **)ivp_realloc(
            m->alloc, m->delayed_ints, (size_t)m->cap_delayed_ints * sizeof(*n),
            (size_t)nc * sizeof(*n));
        if (!n) return;
        m->delayed_ints = n;
        m->cap_delayed_ints = nc;
    }
    m->delayed_ints[m->n_delayed_ints++] = ai;
}

void ivp_active_value_manager_update_delayed(ivp_active_value_manager_t *m) {
    const int change_meter = ivp_active_value_change_meter();
    for (int i = 0; i < m->n_delayed_floats; i++) {
        terminal_float_update(m->delayed_floats[i], change_meter);
        /* mod->update_float() */
    }
    m->n_delayed_floats = 0;
    for (int i = 0; i < m->n_delayed_ints; i++) {
        terminal_int_update(m->delayed_ints[i], change_meter);
    }
    m->n_delayed_ints = 0;
}

void ivp_active_value_manager_refresh_psi(ivp_active_value_manager_t *m, double time) {
    if (m->mod_current_time) {
        ivp_active_float_set(m->mod_current_time, time);
    }
    ivp_active_value_manager_update_delayed(m);
}
