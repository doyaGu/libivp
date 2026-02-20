/* ivp_time_event_queue.h — Future time-event priority queue */
#ifndef IVP_TIME_EVENT_QUEUE_H
#define IVP_TIME_EVENT_QUEUE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/dynamics/ivp_handle.h>
#include <ivp/collision/ivp_synapse.h>
#include <ivp/containers/ivp_minheap.h>
#include <ivp/containers/ivp_hashmap.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum ivp_time_event_type {
    IVP_TIME_EVENT_NONE = 0,
    IVP_TIME_EVENT_MINDIST_IMPACT,
    IVP_TIME_EVENT_MINDIST_RECALC
} ivp_time_event_type_t;

typedef struct ivp_time_event_pair_key {
    uint32_t a_slot;
    uint32_t a_gen;
    uint32_t b_slot;
    uint32_t b_gen;
    uint32_t type;
} ivp_time_event_pair_key_t;

typedef struct ivp_time_event {
    ivp_time_t trigger_time;
    ivp_object_handle_t obj_a;
    ivp_object_handle_t obj_b;

    ivp_time_event_type_t type;
    ivp_synapse_status_t status_a;
    ivp_synapse_status_t status_b;

    ivp_real_t expected_gap;
    uint32_t serial;
    int heap_index;
    ivp_time_event_pair_key_t pair_key;
} ivp_time_event_t;

typedef struct ivp_time_event_queue {
    ivp_minheap_t heap;
    ivp_hashmap_t pair_index;
    uint32_t serial_counter;
    const ivp_allocator_t *alloc;
    bool initialized;
} ivp_time_event_queue_t;

void ivp_time_event_queue_init(ivp_time_event_queue_t *queue,
                               const ivp_allocator_t *alloc);
void ivp_time_event_queue_destroy(ivp_time_event_queue_t *queue);
void ivp_time_event_queue_clear(ivp_time_event_queue_t *queue);

bool ivp_time_event_queue_push(ivp_time_event_queue_t *queue,
                               const ivp_time_event_t *event);
bool ivp_time_event_queue_pop_due(ivp_time_event_queue_t *queue,
                                  ivp_time_t up_to_time,
                                  ivp_time_event_t *out_event);

void ivp_time_event_queue_remove_object(ivp_time_event_queue_t *queue,
                                        ivp_object_handle_t handle);

static inline int ivp_time_event_queue_count(const ivp_time_event_queue_t *queue)
{
    return queue ? ivp_minheap_count(&queue->heap) : 0;
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_TIME_EVENT_QUEUE_H */
