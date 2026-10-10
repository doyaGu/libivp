/* ivp_ov_query.c -- internal ordered OV-tree query traversal */
#include "ivp_ov_query.h"

#include <ivp/base/ivp_allocator.h>
#include <ivp/dynamics/ivp_object.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define IVP_OV_QUERY_INLINE_CAPACITY 64u

typedef struct ivp_ov_query_objects {
    const ivp_allocator_t *allocator;
    ivp_object_t **items;
    size_t count;
    size_t capacity;
    uint64_t previous_key;
    bool has_previous;
    bool ordered;
    ivp_object_t *inline_items[IVP_OV_QUERY_INLINE_CAPACITY];
} ivp_ov_query_objects_t;

typedef struct ivp_ov_query_context {
    ivp_ov_query_node_filter_fn node_filter;
    ivp_ov_query_object_filter_fn object_filter;
    ivp_ov_query_object_visit_fn visit;
    void *user;
} ivp_ov_query_context_t;

static void query_objects_init(
    ivp_ov_query_objects_t *objects, const ivp_allocator_t *allocator)
{
    objects->allocator = allocator;
    objects->items = objects->inline_items;
    objects->count = 0u;
    objects->capacity = IVP_OV_QUERY_INLINE_CAPACITY;
    objects->previous_key = 0u;
    objects->has_previous = false;
    objects->ordered = true;
}

static void query_objects_destroy(ivp_ov_query_objects_t *objects)
{
    if (objects->items != objects->inline_items) {
        ivp_free(objects->allocator, objects->items,
                 objects->capacity * sizeof(*objects->items));
    }
}

static bool key_follows(
    ivp_ov_query_order_t order, uint64_t previous, uint64_t key)
{
    return order == IVP_OV_QUERY_FORWARD
               ? previous < key
               : previous > key;
}

static bool query_objects_grow(ivp_ov_query_objects_t *objects)
{
    if (objects->capacity > SIZE_MAX / 2u ||
        objects->capacity * 2u > SIZE_MAX / sizeof(*objects->items)) {
        return false;
    }
    const size_t new_capacity = objects->capacity * 2u;
    ivp_object_t **new_items;
    if (objects->items == objects->inline_items) {
        new_items = (ivp_object_t **)ivp_alloc(
            objects->allocator, new_capacity * sizeof(*new_items));
        if (new_items) {
            memcpy(new_items, objects->inline_items,
                   objects->count * sizeof(*new_items));
        }
    } else {
        new_items = (ivp_object_t **)ivp_realloc(
            objects->allocator, objects->items,
            objects->capacity * sizeof(*new_items),
            new_capacity * sizeof(*new_items));
    }
    if (!new_items) return false;
    objects->items = new_items;
    objects->capacity = new_capacity;
    return true;
}

static bool query_objects_add(
    ivp_ov_query_objects_t *objects, ivp_ov_query_order_t order,
    ivp_object_t *object)
{
    if (objects->count == objects->capacity &&
        !query_objects_grow(objects)) {
        return false;
    }
    const uint64_t key = ivp_object_order_key(object);
    if (objects->has_previous &&
        !key_follows(order, objects->previous_key, key)) {
        objects->ordered = false;
    }
    objects->has_previous = true;
    objects->previous_key = key;
    objects->items[objects->count++] = object;
    return true;
}

static int compare_objects_forward(const void *left_in, const void *right_in)
{
    const ivp_object_t *left = *(ivp_object_t *const *)left_in;
    const ivp_object_t *right = *(ivp_object_t *const *)right_in;
    const uint64_t left_key = ivp_object_order_key(left);
    const uint64_t right_key = ivp_object_order_key(right);
    return left_key < right_key ? -1 : left_key > right_key ? 1 : 0;
}

static int compare_objects_reverse(const void *left_in, const void *right_in)
{
    return compare_objects_forward(right_in, left_in);
}

static bool collect_query_objects(
    const ivp_ov_node_t *node, const ivp_ov_query_context_t *query,
    ivp_ov_query_order_t order, ivp_ov_query_objects_t *objects)
{
    if (!node || !query->node_filter(query->user, node)) return true;
    const int element_count = ivp_pvec_len(&node->elements);
    const int step = order == IVP_OV_QUERY_FORWARD ? 1 : -1;
    for (int i = order == IVP_OV_QUERY_FORWARD ? 0 : element_count - 1;
         i >= 0 && i < element_count; i += step) {
        const ivp_ov_element_t *element =
            (const ivp_ov_element_t *)ivp_pvec_at(&node->elements, i);
        ivp_object_t *object = element->real_object;
        if (query->object_filter(query->user, object) &&
            !query_objects_add(objects, order, object)) {
            return false;
        }
    }
    const int child_count = ivp_pvec_len(&node->children);
    for (int i = order == IVP_OV_QUERY_FORWARD ? 0 : child_count - 1;
         i >= 0 && i < child_count; i += step) {
        if (!collect_query_objects(
                (const ivp_ov_node_t *)ivp_pvec_at(&node->children, i),
                query, order, objects)) {
            return false;
        }
    }
    return true;
}

static void find_next_query_object(
    const ivp_ov_node_t *node, const ivp_ov_query_context_t *query,
    ivp_ov_query_order_t order, bool has_bound, uint64_t bound,
    ivp_object_t **found, uint64_t *found_key)
{
    if (!node || !query->node_filter(query->user, node)) return;
    for (int i = 0; i < ivp_pvec_len(&node->elements); i++) {
        const ivp_ov_element_t *element =
            (const ivp_ov_element_t *)ivp_pvec_at(&node->elements, i);
        ivp_object_t *object = element->real_object;
        const uint64_t key = ivp_object_order_key(object);
        const bool beyond_bound = !has_bound ||
            (order == IVP_OV_QUERY_FORWARD ? key > bound : key < bound);
        const bool better = !*found ||
            (order == IVP_OV_QUERY_FORWARD
                 ? key < *found_key
                 : key > *found_key);
        if (beyond_bound && better &&
            query->object_filter(query->user, object)) {
            *found = object;
            *found_key = key;
        }
    }
    for (int i = 0; i < ivp_pvec_len(&node->children); i++) {
        find_next_query_object(
            (const ivp_ov_node_t *)ivp_pvec_at(&node->children, i),
            query, order, has_bound, bound, found, found_key);
    }
}

static void visit_without_storage(
    const ivp_ov_node_t *root, const ivp_ov_query_context_t *query,
    ivp_ov_query_order_t order)
{
    bool has_bound = false;
    uint64_t bound = 0u;
    for (;;) {
        ivp_object_t *object = NULL;
        uint64_t key = 0u;
        find_next_query_object(
            root, query, order, has_bound, bound, &object, &key);
        if (!object) return;
        query->visit(query->user, object);
        has_bound = true;
        bound = key;
    }
}

void ivp_ov_query_visit_objects(
    const ivp_ov_tree_t *tree, const ivp_ov_node_t *root,
    ivp_ov_query_order_t order,
    ivp_ov_query_node_filter_fn node_filter,
    ivp_ov_query_object_filter_fn object_filter,
    ivp_ov_query_object_visit_fn visit,
    void *context)
{
    if (!tree || !root || !node_filter || !object_filter || !visit ||
        (order != IVP_OV_QUERY_FORWARD &&
         order != IVP_OV_QUERY_REVERSE)) {
        return;
    }
    const ivp_ov_query_context_t query = {
        node_filter, object_filter, visit, context
    };
    ivp_ov_query_objects_t objects;
    query_objects_init(&objects, tree->allocator);
    if (collect_query_objects(root, &query, order, &objects)) {
        if (!objects.ordered) {
            qsort(objects.items, objects.count, sizeof(*objects.items),
                  order == IVP_OV_QUERY_FORWARD
                      ? compare_objects_forward
                      : compare_objects_reverse);
        }
        for (size_t i = 0; i < objects.count; i++) {
            visit(context, objects.items[i]);
        }
    } else {
        visit_without_storage(root, &query, order);
    }
    query_objects_destroy(&objects);
}
