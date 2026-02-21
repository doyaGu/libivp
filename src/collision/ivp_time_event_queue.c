/* ivp_time_event_queue.c — Future time-event priority queue */

#include <ivp/collision/ivp_time_event_queue.h>
#include <ivp/utility/ivp_hash.h>

#include <math.h>
#include <string.h>

static bool handle_eq(ivp_object_handle_t a, ivp_object_handle_t b)
{
    return a.slot == b.slot && a.generation == b.generation;
}

static bool handle_less(ivp_object_handle_t a, ivp_object_handle_t b)
{
    if (a.slot != b.slot) return a.slot < b.slot;
    return a.generation < b.generation;
}

static ivp_time_event_pair_key_t make_pair_key(ivp_object_handle_t a,
                                                ivp_object_handle_t b,
                                                ivp_time_event_type_t type,
                                                uint32_t mindist_kind,
                                                ivp_synapse_status_t status_a,
                                                ivp_synapse_status_t status_b)
{
    ivp_time_event_pair_key_t key;
    ivp_object_handle_t first = a;
    ivp_object_handle_t second = b;
    ivp_synapse_status_t first_status = status_a;
    ivp_synapse_status_t second_status = status_b;
    if (handle_less(second, first)) {
        first = b;
        second = a;
        first_status = status_b;
        second_status = status_a;
    }
    key.a_slot = first.slot;
    key.a_gen = first.generation;
    key.b_slot = second.slot;
    key.b_gen = second.generation;
    key.type = (uint32_t)type;
    key.mindist_kind = mindist_kind;
    key.status_a = (uint32_t)first_status;
    key.status_b = (uint32_t)second_status;
    return key;
}

static uint32_t pair_key_hash(const void *key, size_t key_size)
{
    return ivp_hash_crc(key, key_size);
}

static bool pair_index_init(ivp_time_event_queue_t *queue)
{
    return ivp_hashmap_init_ex(&queue->pair_index,
                               queue->alloc,
                               256,
                               pair_key_hash,
                               ivp_eq_memcmp,
                               sizeof(ivp_time_event_pair_key_t),
                               &IVP_ELEM_TYPEINFO_PTR,
                               NULL) == IVP_OK;
}

static void pair_index_clear(ivp_time_event_queue_t *queue)
{
    ivp_hashmap_clear(&queue->pair_index);
}

static ivp_time_event_t *pair_index_get(const ivp_time_event_queue_t *queue,
                                        const ivp_time_event_pair_key_t *key)
{
    return (ivp_time_event_t *)ivp_hashmap_get(&queue->pair_index, key);
}

static bool pair_index_put(ivp_time_event_queue_t *queue,
                           const ivp_time_event_pair_key_t *key,
                           ivp_time_event_t *event)
{
    return ivp_hashmap_put(&queue->pair_index, key, event) == IVP_OK;
}

static void pair_index_remove(ivp_time_event_queue_t *queue,
                              const ivp_time_event_pair_key_t *key)
{
    (void)ivp_hashmap_remove(&queue->pair_index, key);
}

static void time_event_heap_index_cb(void *value, int new_index)
{
    ivp_time_event_t *evt = (ivp_time_event_t *)value;
    if (!evt) return;
    evt->heap_index = new_index;
}

static ivp_time_event_t *alloc_event(ivp_time_event_queue_t *queue, const ivp_time_event_t *src)
{
    ivp_time_event_t *evt = (ivp_time_event_t *)ivp_alloc(queue->alloc, sizeof(*evt));
    if (!evt) return NULL;
    *evt = *src;
    return evt;
}

static void free_event(ivp_time_event_queue_t *queue, ivp_time_event_t *evt)
{
    if (!evt) return;
    ivp_free(queue->alloc, evt, sizeof(*evt));
}

void ivp_time_event_queue_init(ivp_time_event_queue_t *queue,
                               const ivp_allocator_t *alloc)
{
    if (!queue) return;
    memset(queue, 0, sizeof(*queue));
    queue->alloc = alloc ? alloc : ivp_default_allocator();

    if (ivp_minheap_init(&queue->heap, queue->alloc, 128, time_event_heap_index_cb) != IVP_OK) {
        memset(queue, 0, sizeof(*queue));
        return;
    }
    if (!pair_index_init(queue)) {
        ivp_minheap_destroy(&queue->heap);
        memset(queue, 0, sizeof(*queue));
        return;
    }

    queue->initialized = true;
}

void ivp_time_event_queue_destroy(ivp_time_event_queue_t *queue)
{
    if (!queue) return;
    if (!queue->initialized) {
        memset(queue, 0, sizeof(*queue));
        return;
    }
    for (int i = 0; i < queue->heap.len; i++) {
        ivp_time_event_t *evt = (ivp_time_event_t *)queue->heap.data[i].value;
        free_event(queue, evt);
    }
    ivp_minheap_destroy(&queue->heap);
    ivp_hashmap_destroy(&queue->pair_index);
    memset(queue, 0, sizeof(*queue));
}

void ivp_time_event_queue_clear(ivp_time_event_queue_t *queue)
{
    if (!queue || !queue->initialized) return;
    for (int i = 0; i < queue->heap.len; i++) {
        ivp_time_event_t *evt = (ivp_time_event_t *)queue->heap.data[i].value;
        free_event(queue, evt);
    }
    ivp_minheap_clear(&queue->heap);
    pair_index_clear(queue);
}

bool ivp_time_event_queue_push(ivp_time_event_queue_t *queue,
                               const ivp_time_event_t *event)
{
    if (!queue || !queue->initialized || !event) return false;
    if (!isfinite((double)event->trigger_time)) return false;

    ivp_time_event_pair_key_t key = make_pair_key(event->obj_a,
                                                  event->obj_b,
                                                  event->type,
                                                  event->mindist_kind,
                                                  event->status_a,
                                                  event->status_b);
    ivp_time_event_t *cur = pair_index_get(queue, &key);
    if (cur) {
        if (event->trigger_time < cur->trigger_time) {
            cur->trigger_time = event->trigger_time;
            cur->expected_gap = event->expected_gap;
            cur->mindist_kind = event->mindist_kind;
            cur->status_a = event->status_a;
            cur->status_b = event->status_b;
            ivp_minheap_update_key(&queue->heap, cur->heap_index, cur->trigger_time);
        }
        return true;
    }

    ivp_time_event_t e = *event;
    e.serial = ++queue->serial_counter;
    e.heap_index = -1;
    e.pair_key = key;
    ivp_time_event_t *evt = alloc_event(queue, &e);
    if (!evt) return false;

    if (ivp_minheap_push(&queue->heap, evt->trigger_time, evt) != IVP_OK) {
        free_event(queue, evt);
        return false;
    }

    if (!pair_index_put(queue, &evt->pair_key, evt)) {
        ivp_minheap_remove(&queue->heap, evt->heap_index);
        free_event(queue, evt);
        return false;
    }

    return true;
}

bool ivp_time_event_queue_pop_due(ivp_time_event_queue_t *queue,
                                  ivp_time_t up_to_time,
                                  ivp_time_event_t *out_event)
{
    if (!queue || !queue->initialized || ivp_minheap_empty(&queue->heap)) return false;

    ivp_heap_entry_t *top = ivp_minheap_peek(&queue->heap);
    if (!top || top->key > up_to_time) return false;

    ivp_heap_entry_t popped;
    if (ivp_minheap_pop(&queue->heap, &popped) != IVP_OK) return false;

    ivp_time_event_t *evt = (ivp_time_event_t *)popped.value;
    if (evt) {
        pair_index_remove(queue, &evt->pair_key);
    }
    if (out_event && evt) {
        *out_event = *evt;
    }
    free_event(queue, evt);
    return true;
}

void ivp_time_event_queue_remove_object(ivp_time_event_queue_t *queue,
                                        ivp_object_handle_t handle)
{
    if (!queue || !queue->initialized) return;
    for (int i = 0; i < queue->heap.len;) {
        ivp_time_event_t *e = (ivp_time_event_t *)queue->heap.data[i].value;
        if (handle_eq(e->obj_a, handle) || handle_eq(e->obj_b, handle)) {
            pair_index_remove(queue, &e->pair_key);
            free_event(queue, e);
            ivp_minheap_remove(&queue->heap, i);
            continue;
        }
        i++;
    }
}
