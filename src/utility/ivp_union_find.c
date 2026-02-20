/* ivp_union_find.c -- Shared disjoint-set (union-find) utilities */
#include <ivp/utility/ivp_union_find.h>

void ivp_union_find_init(int *parent, int count) {
    for (int i = 0; i < count; i++) {
        parent[i] = i;
    }
}

void ivp_union_find_init_with_rank(int *parent, int *rank, int count) {
    for (int i = 0; i < count; i++) {
        parent[i] = i;
        rank[i] = 0;
    }
}

int ivp_union_find_find(int *parent, int index) {
    while (parent[index] != index) {
        parent[index] = parent[parent[index]];
        index = parent[index];
    }
    return index;
}

void ivp_union_find_union(int *parent, int a, int b) {
    int root_a = ivp_union_find_find(parent, a);
    int root_b = ivp_union_find_find(parent, b);
    if (root_a != root_b) {
        parent[root_b] = root_a;
    }
}

void ivp_union_find_union_by_rank(int *parent, int *rank, int a, int b) {
    int root_a = ivp_union_find_find(parent, a);
    int root_b = ivp_union_find_find(parent, b);
    if (root_a == root_b) return;

    if (rank[root_a] < rank[root_b]) {
        parent[root_a] = root_b;
    } else if (rank[root_a] > rank[root_b]) {
        parent[root_b] = root_a;
    } else {
        parent[root_b] = root_a;
        rank[root_a]++;
    }
}

void ivp_union_find_union_min_root(int *parent, int a, int b) {
    int root_a = ivp_union_find_find(parent, a);
    int root_b = ivp_union_find_find(parent, b);
    if (root_a == root_b) return;

    if (root_a > root_b) {
        int t = root_a;
        root_a = root_b;
        root_b = t;
    }
    parent[root_b] = root_a;
}
