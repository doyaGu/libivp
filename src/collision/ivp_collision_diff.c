/* Indexed lookup for IVP_Diff_Hash's object-collision partition. The vector
 * and ivp_coll_fvector_swap still determine all observable ordering. */
#include "ivp_collision_diff.h"
#include "ivp_recheck_diagnostics.h"
#include <stdint.h>
#include <string.h>
#include <limits.h>

typedef struct diff_entry { struct ivp_object *key; int index; } diff_entry_t;
struct ivp_collision_diff_storage {
    size_t bytes;
    size_t slots;
    int capacity;
    diff_entry_t entries[];
};

static uint32_t *positions(ivp_collision_diff_storage_t *s)
{
    return (uint32_t *)(s->entries + s->slots);
}
static size_t bucket(const struct ivp_object *obj, size_t mask)
{
    uintptr_t value = (uintptr_t)obj;
    value >>= 4;
    value ^= value >> 16;
    value *= UINT32_C(0x9e3779b1);
    return (size_t)value & mask;
}
void ivp_collision_diff_storage_free(ivp_collision_diff_storage_t *s, const ivp_allocator_t *a)
{
    if (s) ivp_free(a, s, s->bytes);
}
static bool reserve(ivp_collision_diff_storage_t **storage, int n, const ivp_allocator_t *a)
{
    if (*storage && (*storage)->capacity >= n) return true;
    size_t slots = 1;
    if ((size_t)n > SIZE_MAX / 2) return false;
    while (slots < (size_t)n * 2) {
        if (slots > SIZE_MAX / 2) return false;
        slots *= 2;
    }
    /* Both vector indices and slot backlinks fit 32 bits, including on
     * 64-bit hosts. Guard byte arithmetic before allocating. */
    if (slots - 1 > UINT32_MAX || slots > (SIZE_MAX - sizeof(**storage)) / sizeof(diff_entry_t)) return false;
    size_t bytes = sizeof(**storage) + slots * sizeof(diff_entry_t);
    int capacity = slots / 2 > INT_MAX ? INT_MAX : (int)(slots / 2);
    if ((size_t)capacity > (SIZE_MAX - bytes) / sizeof(uint32_t)) return false;
    bytes += (size_t)capacity * sizeof(uint32_t);
    ivp_collision_diff_storage_t *s = ivp_alloc(a, bytes);
    if (!s) return false;
    s->bytes = bytes; s->slots = slots; s->capacity = capacity;
    ivp_collision_diff_storage_free(*storage, a);
    *storage = s;
    return true;
}
void ivp_collision_diff_begin(ivp_collision_diff_t *d, ivp_coll_fvector_t *v,
                              const struct ivp_object *owner, int queries,
                              ivp_collision_diff_storage_t **storage,
                              const ivp_allocator_t *a, ivp_collision_diff_key_fn key_fn)
{
    *d = (ivp_collision_diff_t){v, 0, v->n, owner, NULL};
    if (v->n < 64 || queries < 64) return;
#ifdef IVP_BENCHMARK_DIAGNOSTICS
    ivp_recheck_diagnostics_t *counts = ivp_recheck_diagnostics_mutable();
#endif
    if (!key_fn) {
#ifdef IVP_BENCHMARK_DIAGNOSTICS
        counts->unsafe_fallbacks++;
#endif
        return;
    }
    if (!reserve(storage, v->n, a)) {
#ifdef IVP_BENCHMARK_DIAGNOSTICS
        counts->oom_fallbacks++;
#endif
        return;
    }
    ivp_collision_diff_storage_t *s = *storage;
    memset(s->entries, 0, s->slots * sizeof(*s->entries));
    uint32_t *pos = positions(s);
    size_t mask = s->slots - 1;
    for (int i = 0; i < v->n; ++i) {
        struct ivp_object *obj = key_fn(v->elems[i], owner);
        if (!obj || obj == owner) {
#ifdef IVP_BENCHMARK_DIAGNOSTICS
            counts->unsafe_fallbacks++;
#endif
            return;
        }
        size_t h = bucket(obj, mask);
        while (s->entries[h].key) {
            /* First-remaining-entry matching matters for duplicates. Do not
             * adopt the reference hash's reverse insertion tie choice. */
            if (s->entries[h].key == obj) {
#ifdef IVP_BENCHMARK_DIAGNOSTICS
                counts->duplicate_fallbacks++;
#endif
                return;
            }
            h = (h + 1) & mask;
        }
        s->entries[h] = (diff_entry_t){obj, i};
        pos[i] = (uint32_t)h;
#ifdef IVP_BENCHMARK_DIAGNOSTICS
        counts->index_entries++;
#endif
    }
    d->index = s;
#ifdef IVP_BENCHMARK_DIAGNOSTICS
    counts->indexed_rechecks++;
#endif
}
bool ivp_collision_diff_check(ivp_collision_diff_t *d, struct ivp_object *obj)
{
    ivp_coll_fvector_t *v = d->vector;
#ifdef IVP_BENCHMARK_DIAGNOSTICS
    ivp_recheck_diagnostics_t *counts = ivp_recheck_diagnostics_mutable();
    counts->searches++;
#endif
    if (obj == d->owner || v->n != d->indexed_length) d->index = NULL;
    int index = -1;
    size_t h = 0;
    if (d->index) {
        ivp_collision_diff_storage_t *s = d->index;
        size_t mask = s->slots - 1;
        h = bucket(obj, mask);
#ifdef IVP_BENCHMARK_DIAGNOSTICS
        counts->indexed_searches++;
        uint64_t probes = 0;
#endif
        for (;;) {
#ifdef IVP_BENCHMARK_DIAGNOSTICS
            counts->index_probes++;
            probes++;
#endif
            if (!s->entries[h].key) break;
            if (s->entries[h].key == obj) {
                int i = s->entries[h].index;
                if (i >= d->found) index = i;
                break;
            }
            h = (h + 1) & mask;
        }
#ifdef IVP_BENCHMARK_DIAGNOSTICS
        if (probes > counts->max_index_probes) counts->max_index_probes = probes;
        if (index >= 0) counts->indexed_found++;
#endif
    } else {
#ifdef IVP_BENCHMARK_DIAGNOSTICS
        uint64_t scanned = 0;
#endif
        for (int i = d->found; i < v->n; ++i) {
#ifdef IVP_BENCHMARK_DIAGNOSTICS
            counts->entries_compared++;
            scanned++;
#endif
            struct ivp_object *objects[2];
            v->elems[i]->cvt->get_objects(v->elems[i], objects);
            if (objects[0] == obj || objects[1] == obj) {
                index = i;
                break;
            }
        }
#ifdef IVP_BENCHMARK_DIAGNOSTICS
        if (scanned > counts->max_scan) counts->max_scan = scanned;
#endif
    }
    if (index < 0) {
#ifdef IVP_BENCHMARK_DIAGNOSTICS
        counts->misses++;
#endif
        return false;
    }
#ifdef IVP_BENCHMARK_DIAGNOSTICS
    counts->found++;
    if (index > d->found) counts->swaps++;
#endif
    if (index > d->found) {
        ivp_coll_fvector_swap(v, d->found, index);
        if (d->index) {
            uint32_t *pos = positions(d->index);
            uint32_t other = pos[d->found];
            d->index->entries[other].index = index;
            d->index->entries[h].index = d->found;
            pos[index] = other; pos[d->found] = (uint32_t)h;
        }
    }
    ++d->found;
    return true;
}
