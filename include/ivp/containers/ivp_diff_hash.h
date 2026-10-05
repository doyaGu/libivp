/* ivp_diff_hash.h -- IVP_Diff_Hash port (ivu_diff_hash.hxx)
 *
 * Re-sorts a pointer vector while elements are checked against it: every
 * element found by ivp_diff_hash_check is swapped to the front (position
 * n_found_objects), so after a pass the found elements occupy
 * [0, n_found_objects) and the not found ones the rest.
 *
 * The template parameters of the reference become callbacks:
 *   base_index(base_elem, reference)  CMP::calc_hash_index(BASE*, SEARCH* reference)
 *   search_index(search_elem)         CMP::calc_hash_index(SEARCH*)
 *   are_equal(base_elem, search_elem) CMP::are_equal(BASE*, SEARCH*)
 * Both index functions must agree for equal elements.
 */
#ifndef IVP_DIFF_HASH_H
#define IVP_DIFF_HASH_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef unsigned int (*ivp_diff_hash_base_index_fn)(const void *base_elem, const void *reference);
typedef unsigned int (*ivp_diff_hash_search_index_fn)(const void *search_elem);
typedef bool (*ivp_diff_hash_equal_fn)(const void *base_elem, const void *search_elem);

typedef struct ivp_diff_hash {
    short                         *hash_to_vector_index;
    int                            buffersize_minus_one;
    bool                           private_buffer;
    void                         **base_vector;   /* resorted in place */
    int                            base_len;
    int                            n_found_objects;
    ivp_diff_hash_base_index_fn    base_index;
    ivp_diff_hash_search_index_fn  search_index;
    ivp_diff_hash_equal_fn         are_equal;
    const ivp_allocator_t         *alloc;
} ivp_diff_hash_t;

/* buffer/buffer_size: caller scratch buffer (buffer_size zero or a power of
 * two); a private buffer is allocated when buffer_size <= base_len (the
 * reference condition), growing until it exceeds 2 * base_len.  The compact
 * hash slots store signed-short vector indices, so base_len may not exceed
 * SHRT_MAX + 1. */
ivp_result_t ivp_diff_hash_init(ivp_diff_hash_t *dh, const ivp_allocator_t *alloc,
                                void **base, int base_len,
                                short *buffer, int buffer_size,
                                const void *reference,
                                ivp_diff_hash_base_index_fn base_index,
                                ivp_diff_hash_search_index_fn search_index,
                                ivp_diff_hash_equal_fn are_equal);
void ivp_diff_hash_destroy(ivp_diff_hash_t *dh);

/* check_element: returns the base element equal to elem (and moves it to
 * position n_found_objects), or NULL. */
void *ivp_diff_hash_check(ivp_diff_hash_t *dh, const void *elem, const void *reference);

#ifdef __cplusplus
}
#endif

#endif /* IVP_DIFF_HASH_H */
