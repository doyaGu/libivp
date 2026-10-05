/* ivp_diff_hash.c -- IVP_Diff_Hash port (ivu_diff_hash.hxx) */
#include <ivp/containers/ivp_diff_hash.h>
#include <ivp/base/ivp_assert.h>
#include <limits.h>
#include <string.h>

static bool diff_hash_is_power_of_two(int value) {
    return value > 0 && (value & (value - 1)) == 0;
}

ivp_result_t ivp_diff_hash_init(ivp_diff_hash_t *dh, const ivp_allocator_t *alloc,
                                void **base, int base_len,
                                short *buffer, int buffer_size,
                                const void *reference,
                                ivp_diff_hash_base_index_fn base_index,
                                ivp_diff_hash_search_index_fn search_index,
                                ivp_diff_hash_equal_fn are_equal) {
    if (!dh) return IVP_ERR_INVALID_ARG;
    memset(dh, 0, sizeof(*dh));
    if (base_len < 0 || base_len > SHRT_MAX + 1 ||
        (base_len > 0 && !base) || buffer_size < 0 ||
        (buffer_size > 0 && !diff_hash_is_power_of_two(buffer_size)) ||
        !base_index || !search_index || !are_equal) {
        return base_len > SHRT_MAX + 1 ? IVP_ERR_CAPACITY_EXCEEDED
                                      : IVP_ERR_INVALID_ARG;
    }
    if (!alloc) alloc = ivp_default_allocator();
    if (!alloc->alloc || !alloc->realloc || !alloc->free) {
        return IVP_ERR_INVALID_ARG;
    }
    dh->alloc = alloc;
    dh->base_index = base_index;
    dh->search_index = search_index;
    dh->are_equal = are_equal;
    if (buffer_size <= base_len || !buffer) {
        dh->private_buffer = true;
        int x = buffer_size > 0 ? buffer_size : 1;
        const int minimum = base_len * 2;
        while (x <= minimum) {
            if (x > INT_MAX / 2) return IVP_ERR_CAPACITY_EXCEEDED;
            x *= 2;
        }
        dh->buffersize_minus_one = x - 1;
        dh->hash_to_vector_index = (short *)ivp_alloc(alloc, (size_t)x * sizeof(short));
        if (!dh->hash_to_vector_index) return IVP_ERR_OUT_OF_MEMORY;
    } else {
        dh->private_buffer = false;
        dh->buffersize_minus_one = buffer_size - 1;
        dh->hash_to_vector_index = buffer;
    }
    dh->n_found_objects = 0;
    dh->base_vector = base;
    dh->base_len = base_len;
    /* all slots = -1 (reference writes 0xffffffff ints over the shorts) */
    memset(dh->hash_to_vector_index, 0xff,
           (size_t)(dh->buffersize_minus_one + 1) * sizeof(short));

    /* insert all existing elements, last to first */
    for (int i = base_len - 1; i >= 0; i--) {
        const void *b = base[i];
        for (unsigned int index = base_index(b, reference);; index++) {
            index &= (unsigned int)dh->buffersize_minus_one;
            if (dh->hash_to_vector_index[index] == -1) {
                dh->hash_to_vector_index[index] = (short)i; /* free place found */
                break;
            }
        }
    }
    return IVP_OK;
}

void ivp_diff_hash_destroy(ivp_diff_hash_t *dh) {
    if (!dh) return;
    if (dh->private_buffer && dh->hash_to_vector_index) {
        ivp_free(dh->alloc, dh->hash_to_vector_index,
                 (size_t)(dh->buffersize_minus_one + 1) * sizeof(short));
    }
    dh->hash_to_vector_index = NULL;
    dh->base_vector = NULL;
    dh->base_len = 0;
    dh->n_found_objects = 0;
}

void *ivp_diff_hash_check(ivp_diff_hash_t *dh, const void *elem, const void *reference) {
    if (!dh || !dh->hash_to_vector_index || !dh->base_vector ||
        !dh->search_index || !dh->base_index || !dh->are_equal) {
        return NULL;
    }
    unsigned int mask = (unsigned int)dh->buffersize_minus_one;
    for (unsigned int index = dh->search_index(elem);; index++) {
        index &= mask;
        int vector_index = dh->hash_to_vector_index[index];
        if (vector_index == -1) {
            return NULL;
        }
        void *base_elem = dh->base_vector[vector_index];
        if (!dh->are_equal(base_elem, elem)) {
            continue;
        }
        /* now we found the element */
        if (vector_index < dh->n_found_objects) return NULL;
        IVP_ASSERT(vector_index >= dh->n_found_objects);
        if (vector_index > dh->n_found_objects) {
            /* swap elems in base vector: n_found_objects <-> vector_index */
            void *nfb = dh->base_vector[dh->n_found_objects];
            for (unsigned int i = dh->base_index(nfb, reference);; i++) {
                i &= mask;
                if (dh->hash_to_vector_index[i] != dh->n_found_objects) {
                    continue;
                }
                dh->hash_to_vector_index[i] = (short)vector_index;
                dh->hash_to_vector_index[index] = (short)dh->n_found_objects;
                dh->base_vector[vector_index] = nfb;
                dh->base_vector[dh->n_found_objects] = base_elem;
                break;
            }
        }
        dh->n_found_objects++;
        return base_elem;
    }
}
