#ifndef IVP_BUILD_EDGE_MAP_INTERNAL_H
#define IVP_BUILD_EDGE_MAP_INTERNAL_H

#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>
#include <ivp/containers/ivp_hashmap.h>

#include <stdint.h>

typedef struct ivp_build_edge_map {
    ivp_hashmap_t map;
    uint64_t *key_storage;
    int key_count;
    int key_capacity;
    const ivp_allocator_t *alloc;
} ivp_build_edge_map_t;

static inline uint64_t ivp_build_make_edge_key(int a, int b) {
    return ((uint64_t)(uint32_t)a << 32) | (uint32_t)b;
}

static inline int ivp_build_edge_map_init(ivp_build_edge_map_t *edge_map,
                                          const ivp_allocator_t *alloc,
                                          int n_edges)
{
    if (!edge_map || !alloc || n_edges <= 0) return 0;

    int initial_capacity = n_edges * 2;
    if (initial_capacity < 16) initial_capacity = 16;

    if (!ivp_ok(ivp_hashmap_init(&edge_map->map, alloc,
                                 initial_capacity,
                                 ivp_hash_u64, ivp_eq_memcmp,
                                 sizeof(uint64_t)))) {
        return 0;
    }

    if ((size_t)n_edges > ((size_t)-1) / sizeof(uint64_t)) {
        ivp_hashmap_destroy(&edge_map->map);
        return 0;
    }

    edge_map->key_storage = (uint64_t *)ivp_alloc(alloc, (size_t)n_edges * sizeof(uint64_t));
    if (!edge_map->key_storage) {
        ivp_hashmap_destroy(&edge_map->map);
        return 0;
    }

    edge_map->key_count = 0;
    edge_map->key_capacity = n_edges;
    edge_map->alloc = alloc;
    return 1;
}

static inline void ivp_build_edge_map_destroy(ivp_build_edge_map_t *edge_map) {
    if (!edge_map) return;

    if (edge_map->key_storage) {
        ivp_free(edge_map->alloc, edge_map->key_storage,
                 (size_t)edge_map->key_capacity * sizeof(uint64_t));
    }
    ivp_hashmap_destroy(&edge_map->map);

    edge_map->key_storage = NULL;
    edge_map->key_count = 0;
    edge_map->key_capacity = 0;
    edge_map->alloc = NULL;
}

static inline int ivp_build_edge_map_get_linear(const ivp_build_edge_map_t *edge_map,
                                                uint64_t key,
                                                int *out_linear)
{
    if (!edge_map || !out_linear) return 0;

    void *value = ivp_hashmap_get(&edge_map->map, &key);
    if (!value) return 0;

    *out_linear = (int)((uintptr_t)value - (uintptr_t)1);
    return 1;
}

static inline int ivp_build_edge_map_insert_linear(ivp_build_edge_map_t *edge_map,
                                                   uint64_t key,
                                                   int linear)
{
    if (!edge_map) return 0;

    if (ivp_hashmap_contains(&edge_map->map, &key)) {
        return 0;
    }

    if (edge_map->key_count >= edge_map->key_capacity) {
        return 0;
    }

    edge_map->key_storage[edge_map->key_count] = key;
    const uint64_t *stored_key = &edge_map->key_storage[edge_map->key_count++];

    void *packed_linear = (void *)((uintptr_t)linear + (uintptr_t)1);
    return ivp_ok(ivp_hashmap_put(&edge_map->map, stored_key, packed_linear)) ? 1 : 0;
}

#endif /* IVP_BUILD_EDGE_MAP_INTERNAL_H */
