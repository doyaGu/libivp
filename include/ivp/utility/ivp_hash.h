/* ivp_hash.h -- Shared hash/CRC utilities for IVP C17
 *
 * Centralized home for CRC table based hashing and common key helpers.
 */
#ifndef IVP_UTILITY_HASH_H
#define IVP_UTILITY_HASH_H

#include <ivp/base/ivp_types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IVP_VHASH_TOUCH_BIT 0x80000000U

/* Ordered pair key, useful for collision pair maps. */
typedef struct ivp_pair_key {
    uint32_t a, b;  /* always stored as (min, max) */
} ivp_pair_key_t;

static inline ivp_pair_key_t ivp_pair_key(uint32_t x, uint32_t y) {
    ivp_pair_key_t pk;
    pk.a = x < y ? x : y;
    pk.b = x < y ? y : x;
    return pk;
}

/* CRC-based hash (matching original IVP_Hash_crctab / IVP_VHash behavior). */
uint32_t ivp_hash_crc(const void *key, size_t key_size);

/* Fast hash for integer keys. */
uint32_t ivp_hash_u32(const void *key, size_t key_size);
uint32_t ivp_hash_u64(const void *key, size_t key_size);

/* Pair-key hash/equality helpers. */
uint32_t ivp_hash_pair(const void *key, size_t key_size);
bool     ivp_eq_pair(const void *a, const void *b, size_t key_size);

/* Generic byte-wise key equality helper. */
bool ivp_eq_memcmp(const void *a, const void *b, size_t key_size);

/* VHash index helpers. Return hash index with touch-bit set. */
uint32_t ivp_vhash_index(const void *key, int key_size);
uint32_t ivp_vhash_fast_index(uintptr_t key);

/* Hash for a void pointer (convenience wrapper). */
static inline uint32_t ivp_vhash_ptr_index(const void *ptr) {
    return ivp_vhash_fast_index((uintptr_t)ptr);
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_UTILITY_HASH_H */
