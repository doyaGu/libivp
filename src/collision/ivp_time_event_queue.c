/* ivp_time_event_queue.c — Future time-event priority queue */

#include <ivp/collision/ivp_time_event_queue.h>

#include <string.h>

static bool handle_eq(ivp_object_handle_t a, ivp_object_handle_t b)
{
    return a.slot == b.slot && a.generation == b.generation;
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
    (void)ivp_minheap_init(&queue->heap, queue->alloc, 128, NULL);
}

void ivp_time_event_queue_destroy(ivp_time_event_queue_t *queue)
{
    if (!queue) return;
    for (int i = 0; i < queue->heap.len; i++) {
        ivp_time_event_t *evt = (ivp_time_event_t *)queue->heap.data[i].value;
        free_event(queue, evt);
    }
    ivp_minheap_destroy(&queue->heap);
    memset(queue, 0, sizeof(*queue));
}

void ivp_time_event_queue_clear(ivp_time_event_queue_t *queue)
{
    if (!queue) return;
    for (int i = 0; i < queue->heap.len; i++) {
        ivp_time_event_t *evt = (ivp_time_event_t *)queue->heap.data[i].value;
        free_event(queue, evt);
    }
    ivp_minheap_clear(&queue->heap);
}

bool ivp_time_event_queue_push(ivp_time_event_queue_t *queue,
                               const ivp_time_event_t *event)
{
    if (!queue || !event) return false;

    /* Keep earliest event for the same pair+type, avoid queue growth by duplicates. */
    for (int i = 0; i < queue->heap.len; i++) {
        ivp_time_event_t *cur = (ivp_time_event_t *)queue->heap.data[i].value;
        if (cur->type != event->type) continue;

        bool same_order = handle_eq(cur->obj_a, event->obj_a) &&
                          handle_eq(cur->obj_b, event->obj_b);
        bool reverse_order = handle_eq(cur->obj_a, event->obj_b) &&
                             handle_eq(cur->obj_b, event->obj_a);
        if (!(same_order || reverse_order)) continue;

        if (event->trigger_time < cur->trigger_time) {
            cur->trigger_time = event->trigger_time;
            cur->expected_gap = event->expected_gap;
            cur->status_a = event->status_a;
            cur->status_b = event->status_b;
            ivp_minheap_update_key(&queue->heap, i, (ivp_real_t)cur->trigger_time);
        }
        return true;
    }

    ivp_time_event_t e = *event;
    e.serial = ++queue->serial_counter;
    ivp_time_event_t *evt = alloc_event(queue, &e);
    if (!evt) return false;

    if (ivp_minheap_push(&queue->heap, (ivp_real_t)evt->trigger_time, evt) != IVP_OK) {
        free_event(queue, evt);
        return false;
    }
    return true;
}

bool ivp_time_event_queue_pop_due(ivp_time_event_queue_t *queue,
                                  ivp_time_t up_to_time,
                                  ivp_time_event_t *out_event)
{
    if (!queue || ivp_minheap_empty(&queue->heap)) return false;

    ivp_heap_entry_t *top = ivp_minheap_peek(&queue->heap);
    if (!top || (ivp_time_t)top->key > up_to_time) return false;

    ivp_heap_entry_t popped;
    if (ivp_minheap_pop(&queue->heap, &popped) != IVP_OK) return false;

    ivp_time_event_t *evt = (ivp_time_event_t *)popped.value;
    if (out_event && evt) {
        *out_event = *evt;
    }
    free_event(queue, evt);
    return true;
}

void ivp_time_event_queue_remove_object(ivp_time_event_queue_t *queue,
                                        ivp_object_handle_t handle)
{
    if (!queue) return;
    for (int i = 0; i < queue->heap.len;) {
        ivp_time_event_t *e = (ivp_time_event_t *)queue->heap.data[i].value;
        if (handle_eq(e->obj_a, handle) || handle_eq(e->obj_b, handle)) {
            free_event(queue, e);
            ivp_minheap_remove(&queue->heap, i);
            continue;
        }
        i++;
    }
}
