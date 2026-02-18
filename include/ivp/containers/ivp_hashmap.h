/* ivp_hashmap.h — Open-addressing hash map for IVP C17
 *
 * Power-of-2 sized. Robin Hood linear probing. Auto-expands at 50% load.
 * NOT to be used for iteration order in solver logic.
 */
#ifndef IVP_HASHMAP_H
#define IVP_HASHMAP_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>
#include <ivp/containers/ivp_elem_typeinfo.h>
#include <ivp/utility/ivp_hash.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Hash / equality callbacks ──────────────────────────────────────────── */

typedef uint32_t (*ivp_hash_fn_t)(const void *key, size_t key_size);
typedef bool     (*ivp_eq_fn_t)(const void *a, const void *b, size_t key_size);

/* ── Type ───────────────────────────────────────────────────────────────── */

typedef struct ivp_hashmap_entry {
    uint32_t    hash;
    const void *key;
    void       *value;
} ivp_hashmap_entry_t;

typedef struct ivp_hashmap {
    ivp_hashmap_entry_t *entries;
    int                  capacity;    /* always power-of-2 */
    int                  count;
    int                  mask;        /* capacity - 1 */
    size_t               key_size;
    ivp_hash_fn_t        hash_fn;
    ivp_eq_fn_t          eq_fn;
    const ivp_allocator_t *alloc;
    const ivp_elem_typeinfo_t *elem_type;
    void                *elem_ctx;
} ivp_hashmap_t;

/* ── Create / Destroy ───────────────────────────────────────────────────── */

/* hash_fn and eq_fn must be non-NULL. */
ivp_result_t ivp_hashmap_init(ivp_hashmap_t *map, const ivp_allocator_t *alloc,
                               int initial_capacity,
                               ivp_hash_fn_t hash_fn, ivp_eq_fn_t eq_fn,
                               size_t key_size);
ivp_result_t ivp_hashmap_init_ex(ivp_hashmap_t *map, const ivp_allocator_t *alloc,
                                  int initial_capacity,
                                  ivp_hash_fn_t hash_fn, ivp_eq_fn_t eq_fn,
                                  size_t key_size,
                                  const ivp_elem_typeinfo_t *elem_type,
                                  void *elem_ctx);
void         ivp_hashmap_destroy(ivp_hashmap_t *map);

/* ── Operations ─────────────────────────────────────────────────────────── */

/* Insert or update. Returns IVP_ERR_OUT_OF_MEMORY if rehash fails. */
ivp_result_t ivp_hashmap_put(ivp_hashmap_t *map, const void *key, void *value);

/* Lookup. Returns NULL if not found. */
void *ivp_hashmap_get(const ivp_hashmap_t *map, const void *key);

/* Remove. Returns the removed value, or NULL if not found. */
void *ivp_hashmap_remove(ivp_hashmap_t *map, const void *key);

/* Check if key exists. */
bool ivp_hashmap_contains(const ivp_hashmap_t *map, const void *key);

/* Number of entries. */
static inline int ivp_hashmap_count(const ivp_hashmap_t *map) { return map->count; }
static inline int ivp_hashmap_cap(const ivp_hashmap_t *map) { return map->capacity; }

/* Ensure map can hold at least min_count entries without further rehash. */
ivp_result_t ivp_hashmap_reserve(ivp_hashmap_t *map, int min_count);

/* Clear all entries (doesn't free table). */
void ivp_hashmap_clear(ivp_hashmap_t *map);

#ifdef __cplusplus
}
#endif

#endif /* IVP_HASHMAP_H */
