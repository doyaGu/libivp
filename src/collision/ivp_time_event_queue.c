/* ivp_time_event_queue.c — Future time-event priority queue */

#include <ivp/collision/ivp_time_event_queue.h>

#include <string.h>

static bool handle_eq(ivp_object_handle_t a, ivp_object_handle_t b)
{
    return a.slot == b.slot && a.generation == b.generation;
}

static bool event_less(const ivp_time_event_t *a, const ivp_time_event_t *b)
{
    if (a->trigger_time < b->trigger_time) return true;
    if (a->trigger_time > b->trigger_time) return false;
    return a->serial < b->serial;
}

static void heap_swap(ivp_time_event_t *a, ivp_time_event_t *b)
{
    ivp_time_event_t tmp = *a;
    *a = *b;
    *b = tmp;
}

static void sift_up(ivp_time_event_queue_t *queue, int index)
{
    while (index > 0) {
        int parent = (index - 1) / 2;
        if (!event_less(&queue->events[index], &queue->events[parent])) {
            break;
        }
        heap_swap(&queue->events[index], &queue->events[parent]);
        index = parent;
    }
}

static void sift_down(ivp_time_event_queue_t *queue, int index)
{
    for (;;) {
        int left = index * 2 + 1;
        int right = left + 1;
        int best = index;

        if (left < queue->count &&
            event_less(&queue->events[left], &queue->events[best])) {
            best = left;
        }
        if (right < queue->count &&
            event_less(&queue->events[right], &queue->events[best])) {
            best = right;
        }

        if (best == index) break;
        heap_swap(&queue->events[index], &queue->events[best]);
        index = best;
    }
}

static bool ensure_capacity(ivp_time_event_queue_t *queue, int needed)
{
    if (needed <= queue->capacity) return true;

    int new_capacity = queue->capacity > 0 ? queue->capacity * 2 : 128;
    if (new_capacity < needed) new_capacity = needed;

    const size_t new_size = (size_t)new_capacity * sizeof(ivp_time_event_t);
    ivp_time_event_t *new_events = (ivp_time_event_t *)ivp_alloc(queue->alloc, new_size);
    if (!new_events) return false;

    memset(new_events, 0, new_size);
    if (queue->events && queue->count > 0) {
        const size_t old_size = (size_t)queue->count * sizeof(ivp_time_event_t);
        memcpy(new_events, queue->events, old_size);
        ivp_free(queue->alloc,
                 queue->events,
                 (size_t)queue->capacity * sizeof(ivp_time_event_t));
    }

    queue->events = new_events;
    queue->capacity = new_capacity;
    return true;
}

void ivp_time_event_queue_init(ivp_time_event_queue_t *queue,
                               const ivp_allocator_t *alloc)
{
    if (!queue) return;
    memset(queue, 0, sizeof(*queue));
    queue->alloc = alloc ? alloc : ivp_default_allocator();
}

void ivp_time_event_queue_destroy(ivp_time_event_queue_t *queue)
{
    if (!queue) return;
    if (queue->events) {
        ivp_free(queue->alloc,
                 queue->events,
                 (size_t)queue->capacity * sizeof(ivp_time_event_t));
    }
    memset(queue, 0, sizeof(*queue));
}

void ivp_time_event_queue_clear(ivp_time_event_queue_t *queue)
{
    if (!queue) return;
    queue->count = 0;
}

bool ivp_time_event_queue_push(ivp_time_event_queue_t *queue,
                               const ivp_time_event_t *event)
{
    if (!queue || !event) return false;

    /* Keep earliest event for the same pair+type, avoid queue growth by duplicates. */
    for (int i = 0; i < queue->count; i++) {
        ivp_time_event_t *cur = &queue->events[i];
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
            sift_up(queue, i);
        }
        return true;
    }

    if (!ensure_capacity(queue, queue->count + 1)) return false;

    ivp_time_event_t e = *event;
    e.serial = ++queue->serial_counter;
    queue->events[queue->count] = e;
    sift_up(queue, queue->count);
    queue->count++;
    return true;
}

bool ivp_time_event_queue_pop_due(ivp_time_event_queue_t *queue,
                                  ivp_time_t up_to_time,
                                  ivp_time_event_t *out_event)
{
    if (!queue || queue->count <= 0) return false;

    ivp_time_event_t *top = &queue->events[0];
    if (top->trigger_time > up_to_time) return false;

    if (out_event) {
        *out_event = *top;
    }

    queue->count--;
    if (queue->count > 0) {
        queue->events[0] = queue->events[queue->count];
        sift_down(queue, 0);
    }
    return true;
}

void ivp_time_event_queue_remove_object(ivp_time_event_queue_t *queue,
                                        ivp_object_handle_t handle)
{
    if (!queue) return;
    for (int i = 0; i < queue->count;) {
        ivp_time_event_t *e = &queue->events[i];
        if (handle_eq(e->obj_a, handle) || handle_eq(e->obj_b, handle)) {
            queue->count--;
            if (i != queue->count) {
                queue->events[i] = queue->events[queue->count];
                sift_down(queue, i);
                sift_up(queue, i);
            }
            continue;
        }
        i++;
    }
}
