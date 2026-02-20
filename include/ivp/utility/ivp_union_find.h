/* ivp_union_find.h -- Shared disjoint-set (union-find) utilities */
#ifndef IVP_UTILITY_UNION_FIND_H
#define IVP_UTILITY_UNION_FIND_H

#include <ivp/base/ivp_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialize parent[i] = i for i in [0, count). */
void ivp_union_find_init(int *parent, int count);

/* Initialize parent[i] = i and rank[i] = 0 for i in [0, count). */
void ivp_union_find_init_with_rank(int *parent, int *rank, int count);

/* Find root with path compression (path halving). */
int ivp_union_find_find(int *parent, int index);

/* Union two sets by attaching root(b) under root(a). */
void ivp_union_find_union(int *parent, int a, int b);

/* Union two sets using rank heuristic. */
void ivp_union_find_union_by_rank(int *parent, int *rank, int a, int b);

/* Union two sets by always keeping the smaller root index as parent. */
void ivp_union_find_union_min_root(int *parent, int a, int b);

#ifdef __cplusplus
}
#endif

#endif /* IVP_UTILITY_UNION_FIND_H */
