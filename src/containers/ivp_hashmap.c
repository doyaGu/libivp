/* ivp_hashmap.c — Open-addressing hash map implementation */
#include <ivp/containers/ivp_hashmap.h>
#include <ivp/base/ivp_assert.h>
#include <limits.h>
#include <string.h>

/* ── Internal helpers ───────────────────────────────────────────────────── */

#define EMPTY_HASH 0u
/* We reserve hash=0 as "empty slot". Real hashes set bit 0 to ensure non-zero. */

static inline uint32_t fix_hash(uint32_t h) {
    return h == 0 ? 1 : h;
}

static ivp_result_t hashmap_rehash(ivp_hashmap_t *map, int new_cap);
static ivp_result_t hashmap_put_internal(ivp_hashmap_t *map,
                                         const void *key, void *value);

/* ── API implementation ─────────────────────────────────────────────────── */

ivp_result_t ivp_hashmap_init(ivp_hashmap_t *map, const ivp_allocator_t *alloc,
                               int initial_capacity,
                               ivp_hash_fn_t hash_fn, ivp_eq_fn_t eq_fn,
                               size_t key_size) {
    if (!map) return IVP_ERR_INVALID_ARG;
    if (!hash_fn || !eq_fn) return IVP_ERR_INVALID_ARG;
    if (!alloc) alloc = ivp_default_allocator();

    /* Round up to power of 2 */
    int cap = 16;
    while (cap < initial_capacity) {
        if (cap > INT_MAX / 2) return IVP_ERR_CAPACITY_EXCEEDED;
        cap *= 2;
    }
    if ((size_t)cap > ((size_t)-1) / sizeof(ivp_hashmap_entry_t)) {
        return IVP_ERR_CAPACITY_EXCEEDED;
    }

    size_t entry_bytes = (size_t)cap * sizeof(ivp_hashmap_entry_t);
    ivp_hashmap_entry_t *entries = (ivp_hashmap_entry_t *)ivp_alloc(alloc, entry_bytes);
    if (!entries) return IVP_ERR_OUT_OF_MEMORY;
    memset(entries, 0, entry_bytes);

    map->entries  = entries;
    map->capacity = cap;
    map->count    = 0;
    map->mask     = cap - 1;
    map->key_size = key_size;
    map->hash_fn  = hash_fn;
    map->eq_fn    = eq_fn;
    map->alloc    = alloc;

    return IVP_OK;
}

ivp_result_t ivp_hashmap_reserve(ivp_hashmap_t *map, int min_count) {
    if (!map) return IVP_ERR_INVALID_ARG;
    if (min_count <= 0) return IVP_OK;
    int needed_cap = 16;
    while ((needed_cap / 2) < min_count) {
        if (needed_cap > INT_MAX / 2) return IVP_ERR_CAPACITY_EXCEEDED;
        needed_cap *= 2;
    }
    if (needed_cap <= map->capacity) return IVP_OK;
    return hashmap_rehash(map, needed_cap);
}

void ivp_hashmap_destroy(ivp_hashmap_t *map) {
    if (!map) return;
    if (map->entries) {
        ivp_free(map->alloc, map->entries,
                 (size_t)map->capacity * sizeof(ivp_hashmap_entry_t));
    }
    map->entries  = NULL;
    map->capacity = 0;
    map->count    = 0;
}

static ivp_result_t hashmap_rehash(ivp_hashmap_t *map, int new_cap) {
    if (new_cap <= 0 || (new_cap & (new_cap - 1)) != 0) {
        return IVP_ERR_INVALID_ARG;
    }
    if ((size_t)new_cap > ((size_t)-1) / sizeof(ivp_hashmap_entry_t)) {
        return IVP_ERR_CAPACITY_EXCEEDED;
    }
    size_t new_bytes = (size_t)new_cap * sizeof(ivp_hashmap_entry_t);
    ivp_hashmap_entry_t *new_entries = (ivp_hashmap_entry_t *)ivp_alloc(map->alloc, new_bytes);
    if (!new_entries) return IVP_ERR_OUT_OF_MEMORY;
    memset(new_entries, 0, new_bytes);

    int new_mask = new_cap - 1;
    ivp_hashmap_entry_t *old = map->entries;
    int old_cap = map->capacity;

    /* Re-insert all existing entries */
    for (int i = 0; i < old_cap; i++) {
        if (old[i].hash == EMPTY_HASH) continue;

        uint32_t h = old[i].hash;
        int pos = (int)(h & (uint32_t)new_mask);
        for (int s = 0; s < new_cap && new_entries[pos].hash != EMPTY_HASH; s++) {
            pos = (pos + 1) & new_mask;
        }
        new_entries[pos] = old[i];
    }

    ivp_free(map->alloc, old, (size_t)old_cap * sizeof(ivp_hashmap_entry_t));
    map->entries  = new_entries;
    map->capacity = new_cap;
    map->mask     = new_mask;

    return IVP_OK;
}

static ivp_result_t hashmap_put_internal(ivp_hashmap_t *map,
                                         const void *key, void *value) {
    uint32_t h = fix_hash(map->hash_fn(key, map->key_size));
    int pos = (int)(h & (uint32_t)map->mask);

    for (int searched = 0; searched < map->capacity; searched++) {
        if (map->entries[pos].hash == EMPTY_HASH) {
            map->entries[pos].hash  = h;
            map->entries[pos].key   = key;
            map->entries[pos].value = value;
            map->count++;
            return IVP_OK;
        }
        if (map->entries[pos].hash == h &&
            map->eq_fn(map->entries[pos].key, key, map->key_size)) {
            map->entries[pos].value = value;
            return IVP_OK;
        }
        pos = (pos + 1) & map->mask;
    }
    return IVP_ERR_CAPACITY_EXCEEDED;
}

ivp_result_t ivp_hashmap_put(ivp_hashmap_t *map, const void *key, void *value) {
    /* Rehash at 50% load */
    if (map->count >= map->capacity / 2) {
        if (map->capacity > INT_MAX / 2) return IVP_ERR_CAPACITY_EXCEEDED;
        ivp_result_t r = hashmap_rehash(map, map->capacity * 2);
        if (r != IVP_OK) return r;
    }

    return hashmap_put_internal(map, key, value);
}

void *ivp_hashmap_get(const ivp_hashmap_t *map, const void *key) {
    uint32_t h = fix_hash(map->hash_fn(key, map->key_size));
    int pos = (int)(h & (uint32_t)map->mask);

    for (int searched = 0; searched < map->capacity; searched++) {
        if (map->entries[pos].hash == EMPTY_HASH) return NULL;
        if (map->entries[pos].hash == h &&
            map->eq_fn(map->entries[pos].key, key, map->key_size)) {
            return map->entries[pos].value;
        }
        pos = (pos + 1) & map->mask;
    }
    return NULL;
}

void *ivp_hashmap_remove(ivp_hashmap_t *map, const void *key) {
    uint32_t h = fix_hash(map->hash_fn(key, map->key_size));
    int pos = (int)(h & (uint32_t)map->mask);

    for (int searched = 0; searched < map->capacity; searched++) {
        if (map->entries[pos].hash == EMPTY_HASH) return NULL;
        if (map->entries[pos].hash == h &&
            map->eq_fn(map->entries[pos].key, key, map->key_size)) {
            void *val = map->entries[pos].value;
            /* Mark as empty and rehash subsequent entries */
            map->entries[pos].hash  = EMPTY_HASH;
            map->entries[pos].key   = NULL;
            map->entries[pos].value = NULL;
            map->count--;

            /* Rehash chain after the removed entry */
            int next = (pos + 1) & map->mask;
            for (int s = 0; s < map->capacity && map->entries[next].hash != EMPTY_HASH; s++) {
                ivp_hashmap_entry_t tmp = map->entries[next];
                map->entries[next].hash = EMPTY_HASH;
                map->entries[next].key  = NULL;
                map->entries[next].value = NULL;
                map->count--;
                (void)hashmap_put_internal(map, tmp.key, tmp.value);
                next = (next + 1) & map->mask;
            }

            return val;
        }
        pos = (pos + 1) & map->mask;
    }
    return NULL;
}

bool ivp_hashmap_contains(const ivp_hashmap_t *map, const void *key) {
    return ivp_hashmap_get(map, key) != NULL;
}

void ivp_hashmap_clear(ivp_hashmap_t *map) {
    memset(map->entries, 0, (size_t)map->capacity * sizeof(ivp_hashmap_entry_t));
    map->count = 0;
}
