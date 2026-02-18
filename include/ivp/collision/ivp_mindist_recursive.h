/* ivp_mindist_recursive.h — Recursive/compound mindist delegator */
#ifndef IVP_MINDIST_RECURSIVE_H
#define IVP_MINDIST_RECURSIVE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/collision/ivp_mindist_persistent.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ivp_mindist_recursive {
    ivp_mindist_entry_t **children;
    int child_count;
    int child_capacity;
    const ivp_allocator_t *alloc;
} ivp_mindist_recursive_t;

void ivp_mindist_recursive_init(ivp_mindist_recursive_t *recursive,
                                const ivp_allocator_t *alloc);
void ivp_mindist_recursive_destroy(ivp_mindist_recursive_t *recursive);

bool ivp_mindist_recursive_add_child(ivp_mindist_recursive_t *recursive,
                                     ivp_mindist_entry_t *entry);
void ivp_mindist_recursive_remove_child(ivp_mindist_recursive_t *recursive,
                                        ivp_mindist_entry_t *entry);

void ivp_mindist_recursive_invalidate(ivp_mindist_recursive_t *recursive);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MINDIST_RECURSIVE_H */
