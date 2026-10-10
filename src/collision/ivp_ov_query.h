/* ivp_ov_query.h -- internal ordered OV-tree query traversal */
#ifndef IVP_OV_QUERY_H
#define IVP_OV_QUERY_H

#include <ivp/collision/ivp_collision_core.h>

#include <stdbool.h>

typedef bool (*ivp_ov_query_node_filter_fn)(
    void *context, const ivp_ov_node_t *node);
typedef bool (*ivp_ov_query_object_filter_fn)(
    void *context, ivp_object_t *object);
typedef void (*ivp_ov_query_object_visit_fn)(
    void *context, ivp_object_t *object);

typedef enum ivp_ov_query_order {
    IVP_OV_QUERY_FORWARD,
    IVP_OV_QUERY_REVERSE
} ivp_ov_query_order_t;

/* Visit every accepted object in world-local object order.  Small queries
 * stay on the stack.  If scratch growth fails, the same order is produced by
 * repeated allocation-free scans. */
void ivp_ov_query_visit_objects(
    const ivp_ov_tree_t *tree, const ivp_ov_node_t *root,
    ivp_ov_query_order_t order,
    ivp_ov_query_node_filter_fn node_filter,
    ivp_ov_query_object_filter_fn object_filter,
    ivp_ov_query_object_visit_fn visit,
    void *context);

#endif /* IVP_OV_QUERY_H */
