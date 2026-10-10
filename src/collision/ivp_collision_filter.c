/* ivp_collision_filter.c — Built-in collision filter callbacks
 *
 * Implements group, layer mask, exclusive pair, and meta filters.
 */
#include <ivp/collision/ivp_collision_filter.h>
#include <ivp/dynamics/ivp_object.h>
#include <string.h>

/* ── Group filter ──────────────────────────────────────────────────────── */

static bool group_check(const struct ivp_object *a, const struct ivp_object *b, void *ud)
{
    (void)ud;
    const ivp_object_t *oa = (const ivp_object_t *)a;
    const ivp_object_t *ob = (const ivp_object_t *)b;
    if (oa->collision_group != 0 && oa->collision_group == ob->collision_group) {
        return false; /* same non-zero group: skip */
    }
    return true;
}

ivp_collision_filter_t ivp_collision_filter_group(void)
{
    ivp_collision_filter_t f;
    f.type = IVP_CFILTER_GROUP;
    f.check_fn = group_check;
    f.user_data = NULL;
    return f;
}

/* ── Layer mask filter ─────────────────────────────────────────────────── */

static bool layer_check(const struct ivp_object *a, const struct ivp_object *b, void *ud)
{
    (void)ud;
    const ivp_object_t *oa = (const ivp_object_t *)a;
    const ivp_object_t *ob = (const ivp_object_t *)b;
    return (oa->layer_mask & ob->layer_mask) != 0;
}

ivp_collision_filter_t ivp_collision_filter_layer_mask(void)
{
    ivp_collision_filter_t f;
    f.type = IVP_CFILTER_LAYER_MASK;
    f.check_fn = layer_check;
    f.user_data = NULL;
    return f;
}

/* ── Exclusive pair filter ─────────────────────────────────────────────── */

static ivp_exclusive_pair_t make_pair(const struct ivp_object *a, const struct ivp_object *b)
{
    uintptr_t pa = (uintptr_t)a;
    uintptr_t pb = (uintptr_t)b;
    ivp_exclusive_pair_t pair;
    /* Normalize unordered pairs by storing the smaller pointer first. */
    if (pa > pb) {
        uintptr_t tmp = pa;
        pa = pb;
        pb = tmp;
    }
    pair.obj_a = pa;
    pair.obj_b = pb;
    return pair;
}

static bool pair_equals(const ivp_exclusive_pair_t *lhs, const ivp_exclusive_pair_t *rhs)
{
    return lhs->obj_a == rhs->obj_a && lhs->obj_b == rhs->obj_b;
}

static bool exclusive_pair_check(const struct ivp_object *a, const struct ivp_object *b, void *ud)
{
    ivp_exclusive_pair_filter_t *f = (ivp_exclusive_pair_filter_t *)ud;
    ivp_exclusive_pair_t pair = make_pair(a, b);
    for (int i = 0; i < f->count; i++) {
        if (pair_equals(&f->pairs[i], &pair)) return false; /* pair disabled */
    }
    return true;
}

ivp_result_t ivp_exclusive_pair_filter_init(ivp_exclusive_pair_filter_t *f,
                                              const ivp_allocator_t *alloc)
{
    if (!alloc) alloc = ivp_default_allocator();
    memset(f, 0, sizeof(*f));
    f->alloc = alloc;
    f->capacity = 16;
    f->pairs = (ivp_exclusive_pair_t *)ivp_alloc(
        alloc, (size_t)f->capacity * sizeof(ivp_exclusive_pair_t));
    if (!f->pairs) return IVP_ERR_OUT_OF_MEMORY;

    f->base.type = IVP_CFILTER_CALLBACK;
    f->base.check_fn = exclusive_pair_check;
    f->base.user_data = f;
    return IVP_OK;
}

void ivp_exclusive_pair_filter_destroy(ivp_exclusive_pair_filter_t *f)
{
    if (f->pairs) {
        ivp_free(f->alloc, f->pairs,
                 (size_t)f->capacity * sizeof(ivp_exclusive_pair_t));
        f->pairs = NULL;
    }
    f->count = 0;
    f->capacity = 0;
}

void ivp_exclusive_pair_filter_disable(ivp_exclusive_pair_filter_t *f,
                                         const struct ivp_object *a,
                                         const struct ivp_object *b)
{
    ivp_exclusive_pair_t pair = make_pair(a, b);

    /* Check if already present */
    for (int i = 0; i < f->count; i++) {
        if (pair_equals(&f->pairs[i], &pair)) return;
    }

    /* Grow if needed */
    if (f->count >= f->capacity) {
        int new_cap = f->capacity * 2;
        ivp_exclusive_pair_t *new_pairs = ivp_realloc(
            f->alloc, f->pairs,
            (size_t)f->capacity * sizeof(ivp_exclusive_pair_t),
            (size_t)new_cap * sizeof(ivp_exclusive_pair_t));
        if (!new_pairs) return;
        f->pairs = new_pairs;
        f->capacity = new_cap;
    }

    f->pairs[f->count++] = pair;
}

void ivp_exclusive_pair_filter_enable(ivp_exclusive_pair_filter_t *f,
                                        const struct ivp_object *a,
                                        const struct ivp_object *b)
{
    ivp_exclusive_pair_t pair = make_pair(a, b);
    for (int i = 0; i < f->count; i++) {
        if (pair_equals(&f->pairs[i], &pair)) {
            /* Remove by swapping with last */
            f->pairs[i] = f->pairs[f->count - 1];
            f->count--;
            return;
        }
    }
}

/* ── Meta filter ───────────────────────────────────────────────────────── */

static bool meta_check(const struct ivp_object *a, const struct ivp_object *b, void *ud)
{
    ivp_meta_collision_filter_t *meta = (ivp_meta_collision_filter_t *)ud;
    for (int i = 0; i < meta->count; i++) {
        if (!ivp_collision_filter_check(&meta->filters[i], a, b)) {
            return false;
        }
    }
    return true;
}

ivp_collision_filter_t ivp_collision_filter_meta(ivp_meta_collision_filter_t *meta,
                                                  ivp_collision_filter_t *filters,
                                                  int count)
{
    meta->filters = filters;
    meta->count = count;
    meta->base.type = IVP_CFILTER_META;
    meta->base.check_fn = meta_check;
    meta->base.user_data = meta;
    return meta->base;
}
