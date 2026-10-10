#include "ivp_registry.h"

static inline ivp_registry_slot_meta_t *ivp_registry_get_slot_meta(const ivp_registry_t *reg,
                                                                   uint32_t slot)
{
    return (ivp_registry_slot_meta_t *)ivp_array_get(&reg->slots, (int)slot);
}

ivp_result_t ivp_registry_init(ivp_registry_t *reg,
                               const ivp_allocator_t *alloc,
                               int initial_capacity,
                               ivp_array_index_cb_t index_cb,
                               void *index_userdata,
                               ivp_registry_get_index_fn_t get_index)
{
    if (!reg) return IVP_ERR_INVALID_ARG;
    if (!alloc) alloc = ivp_default_allocator();

    reg->alloc = alloc;
    reg->get_index = get_index;

    if (ivp_array_init_indexed(&reg->dense, alloc, initial_capacity, index_cb, index_userdata) != IVP_OK) {
        return IVP_ERR_OUT_OF_MEMORY;
    }

    if (ivp_array_init(&reg->slots, alloc, initial_capacity) != IVP_OK) {
        ivp_array_destroy(&reg->dense);
        return IVP_ERR_OUT_OF_MEMORY;
    }

    if (ivp_array_init(&reg->free_slots, alloc, initial_capacity) != IVP_OK) {
        ivp_array_destroy(&reg->slots);
        ivp_array_destroy(&reg->dense);
        return IVP_ERR_OUT_OF_MEMORY;
    }

    return IVP_OK;
}

void ivp_registry_destroy(ivp_registry_t *reg,
                          bool free_elements,
                          size_t element_size)
{
    if (!reg) return;

    if (free_elements) {
        for (int i = ivp_array_len(&reg->dense) - 1; i >= 0; i--) {
            void *elem = ivp_registry_get_at(reg, i);
            ivp_free(reg->alloc, elem, element_size);
        }
    }

    for (int i = 0; i < ivp_array_len(&reg->slots); i++) {
        ivp_registry_slot_meta_t *meta =
            (ivp_registry_slot_meta_t *)ivp_array_get(&reg->slots, i);
        if (meta) {
            ivp_free(reg->alloc, meta, sizeof(ivp_registry_slot_meta_t));
        }
    }

    ivp_array_destroy(&reg->free_slots);
    ivp_array_destroy(&reg->slots);
    ivp_array_destroy(&reg->dense);

    reg->alloc = NULL;
    reg->get_index = NULL;
}

int ivp_registry_count(const ivp_registry_t *reg)
{
    return ivp_array_len(&reg->dense);
}

void *ivp_registry_get_at(const ivp_registry_t *reg, int dense_index)
{
    return ivp_array_get(&reg->dense, dense_index);
}

static bool ivp_registry_alloc_slot(ivp_registry_t *reg,
                                    uint32_t *out_slot,
                                    uint32_t *out_generation)
{
    if (ivp_array_len(&reg->free_slots) > 0) {
        const uintptr_t packed = (uintptr_t)ivp_array_pop(&reg->free_slots);
        const uint32_t slot = (uint32_t)(packed - 1u);
        ivp_registry_slot_meta_t *meta = ivp_registry_get_slot_meta(reg, slot);
        if (meta->generation == 0) {
            meta->generation = 1;
        }
        *out_slot = slot;
        *out_generation = meta->generation;
        return true;
    }

    ivp_registry_slot_meta_t *meta =
        (ivp_registry_slot_meta_t *)ivp_alloc(reg->alloc, sizeof(ivp_registry_slot_meta_t));
    if (!meta) {
        return false;
    }

    meta->generation = 1;
    meta->element = NULL;

    if (ivp_array_push(&reg->slots, meta) != IVP_OK) {
        ivp_free(reg->alloc, meta, sizeof(ivp_registry_slot_meta_t));
        return false;
    }

    const uint32_t slot = (uint32_t)(ivp_array_len(&reg->slots) - 1);

    *out_slot = slot;
    *out_generation = meta->generation;
    return true;
}

ivp_result_t ivp_registry_insert(ivp_registry_t *reg,
                                 void *elem,
                                 ivp_handle_t *out_handle)
{
    if (!reg || !elem || !out_handle) return IVP_ERR_INVALID_ARG;

    if (ivp_array_push(&reg->dense, elem) != IVP_OK) {
        return IVP_ERR_OUT_OF_MEMORY;
    }

    uint32_t slot = 0;
    uint32_t generation = 0;
    if (!ivp_registry_alloc_slot(reg, &slot, &generation)) {
        const int idx = reg->get_index(elem);
        (void)ivp_array_remove_swap(&reg->dense, idx);
        return IVP_ERR_OUT_OF_MEMORY;
    }

    ivp_registry_slot_meta_t *meta = ivp_registry_get_slot_meta(reg, slot);
    IVP_ASSERT(meta->element == NULL);
    meta->element = elem;

    out_handle->slot = slot;
    out_handle->generation = generation;
    return IVP_OK;
}

void *ivp_registry_get_by_handle(const ivp_registry_t *reg,
                                 ivp_handle_t handle)
{
    if (!reg) return NULL;
    if (handle.slot == UINT32_MAX || handle.generation == 0) return NULL;
    if (handle.slot >= (uint32_t)ivp_array_len(&reg->slots)) return NULL;

    ivp_registry_slot_meta_t *meta =
        ivp_registry_get_slot_meta(reg, handle.slot);
    if (!meta || meta->generation != handle.generation) return NULL;
    return meta->element;
}

bool ivp_registry_remove_by_handle(ivp_registry_t *reg,
                                   ivp_handle_t handle,
                                   void **out_removed_elem)
{
    if (!reg || !out_removed_elem) return false;
    *out_removed_elem = NULL;

    if (handle.slot == UINT32_MAX || handle.generation == 0) return false;

    if (handle.slot >= (uint32_t)ivp_array_len(&reg->slots)) return false;
    ivp_registry_slot_meta_t *meta =
        ivp_registry_get_slot_meta(reg, handle.slot);
    if (!meta || meta->generation != handle.generation || !meta->element) {
        return false;
    }
    void *elem = meta->element;

    const int idx = reg->get_index(elem);
    (void)ivp_array_remove_swap(&reg->dense, idx);

    meta->element = NULL;
    meta->generation++;
    if (meta->generation == 0) meta->generation = 1;
    (void)ivp_array_push(&reg->free_slots, (void *)(uintptr_t)(handle.slot + 1u));

    *out_removed_elem = elem;
    return true;
}
