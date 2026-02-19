/* ivp_build_bvh_bottomup.c -- Bottom-up BVH builder implementation
 *
 * Implements greedy agglomeration BVH construction. Iteratively merges
 * closest sphere pairs within a radius tolerance, producing tighter
 * bounding volumes for certain geometry distributions compared to
 * top-down median split.
 *
 * Algorithm:
 *   1. Create one bounding sphere per input ledge.
 *   2. Loop until one root remains:
 *      a. For each active sphere, find nearest neighbor (N^2 scan).
 *      b. Compute merged bounding sphere for each candidate pair.
 *      c. Accept pairs where merged_radius <= max(r1,r2) * tolerance.
 *      d. Remove conflicting pairs (same sphere in multiple merges).
 *      e. Execute merges: create parent spheres, remove children.
 *      f. If no merges possible, increase tolerance by 10%.
 *   3. Compile into compact surface with ledgetree.
 *
 * Ported from legacy bottom-up BVH strategy.
 */
#include <ivp/builder/ivp_build_bvh_bottomup.h>
#include <ivp/builder/ivp_rot_inertia.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/base/ivp_assert.h>

#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ── Bottom-up BVH sphere node ─────────────────────────────────────────── */

typedef struct bu_sphere {
    float center[3];
    float radius;
    uint8_t box_sizes[3];
    const ivp_compact_ledge_t *ledge;  /* non-NULL for terminal nodes */
    struct bu_sphere *child_1;
    struct bu_sphere *child_2;
} bu_sphere_t;

/* ── Internal arena for BVH nodes ──────────────────────────────────────── */

typedef struct bu_arena {
    bu_sphere_t *nodes;
    int count;
    int capacity;
} bu_arena_t;

static bu_sphere_t *bu_alloc(bu_arena_t *arena) {
    IVP_ASSERT(arena->count < arena->capacity);
    bu_sphere_t *n = &arena->nodes[arena->count++];
    memset(n, 0, sizeof(*n));
    return n;
}

/* ── Config initialization ─────────────────────────────────────────────── */

void ivp_bvh_bottomup_config_init(ivp_bvh_bottomup_config_t *cfg) {
    cfg->radius_tolerance = IVP_REAL(1.1);
    cfg->alloc = NULL;
}

/* ── Merge candidate pair ──────────────────────────────────────────────── */

typedef struct merge_pair {
    bu_sphere_t *s1;
    bu_sphere_t *s2;
    float merged_radius;
    float merged_center[3];
} merge_pair_t;

/* ── Compute merged bounding sphere ────────────────────────────────────── */

static void compute_merged_sphere(const bu_sphere_t *s1, const bu_sphere_t *s2,
                                    float center_out[3], float *radius_out) {
    /* Vector from s1 center to s2 center */
    float dx = s2->center[0] - s1->center[0];
    float dy = s2->center[1] - s1->center[1];
    float dz = s2->center[2] - s1->center[2];
    float dist = sqrtf(dx*dx + dy*dy + dz*dz);

    /* If one sphere is contained in the other, use the larger sphere */
    if (dist + s2->radius <= s1->radius) {
        center_out[0] = s1->center[0];
        center_out[1] = s1->center[1];
        center_out[2] = s1->center[2];
        *radius_out = s1->radius;
        return;
    }
    if (dist + s1->radius <= s2->radius) {
        center_out[0] = s2->center[0];
        center_out[1] = s2->center[1];
        center_out[2] = s2->center[2];
        *radius_out = s2->radius;
        return;
    }

    /* Compute minimal enclosing sphere */
    float new_radius = (dist + s1->radius + s2->radius) * 0.5f;

    if (dist > 1e-10f) {
        /* Move along line from s1 to s2 */
        float t = (new_radius - s1->radius) / dist;
        center_out[0] = s1->center[0] + dx * t;
        center_out[1] = s1->center[1] + dy * t;
        center_out[2] = s1->center[2] + dz * t;
    } else {
        /* Centers are coincident */
        center_out[0] = s1->center[0];
        center_out[1] = s1->center[1];
        center_out[2] = s1->center[2];
    }

    *radius_out = new_radius;
}

/* ── Find nearest neighbor for each sphere ─────────────────────────────── */

static void find_merge_candidates(bu_sphere_t **spheres, int n,
                                    float tolerance,
                                    merge_pair_t *pairs_out,
                                    int *n_pairs_out) {
    int n_pairs = 0;

    for (int i = 0; i < n; i++) {
        bu_sphere_t *s1 = spheres[i];
        float best_dist = 1e20f;
        bu_sphere_t *best_neighbor = NULL;
        float best_merged_radius = 0.0f;
        float best_merged_center[3] = {0.0f, 0.0f, 0.0f};

        for (int j = 0; j < n; j++) {
            if (i == j) continue;

            bu_sphere_t *s2 = spheres[j];

            /* Compute distance between centers */
            float dx = s2->center[0] - s1->center[0];
            float dy = s2->center[1] - s1->center[1];
            float dz = s2->center[2] - s1->center[2];
            float dist = sqrtf(dx*dx + dy*dy + dz*dz);

            if (dist < best_dist) {
                /* Compute merged sphere */
                float merged_center[3];
                float merged_radius;
                compute_merged_sphere(s1, s2, merged_center, &merged_radius);

                /* Check if merge is acceptable */
                float max_radius = (s1->radius > s2->radius) ? s1->radius : s2->radius;
                if (merged_radius <= max_radius * tolerance) {
                    best_dist = dist;
                    best_neighbor = s2;
                    best_merged_radius = merged_radius;
                    best_merged_center[0] = merged_center[0];
                    best_merged_center[1] = merged_center[1];
                    best_merged_center[2] = merged_center[2];
                }
            }
        }

        /* Record pair if valid neighbor found */
        if (best_neighbor != NULL) {
            merge_pair_t *p = &pairs_out[n_pairs++];
            p->s1 = s1;
            p->s2 = best_neighbor;
            p->merged_radius = best_merged_radius;
            p->merged_center[0] = best_merged_center[0];
            p->merged_center[1] = best_merged_center[1];
            p->merged_center[2] = best_merged_center[2];
        }
    }

    *n_pairs_out = n_pairs;
}

/* ── Remove conflicting pairs ──────────────────────────────────────────── */

static void remove_conflicts(merge_pair_t *pairs, int *n_pairs,
                               const ivp_allocator_t *alloc) {
    int n = *n_pairs;

    /* Mark pairs that have conflicts */
    int *keep = (int *)ivp_alloc(alloc, (size_t)n * sizeof(int));
    if (!keep) {
        *n_pairs = 0;
        return;
    }

    for (int i = 0; i < n; i++) {
        keep[i] = 1;
    }

    for (int i = 0; i < n; i++) {
        if (!keep[i]) continue;

        /* Check if any later pair conflicts with this one */
        for (int j = i + 1; j < n; j++) {
            if (!keep[j]) continue;

            /* Conflict if they share a sphere */
            if (pairs[i].s1 == pairs[j].s1 || pairs[i].s1 == pairs[j].s2 ||
                pairs[i].s2 == pairs[j].s1 || pairs[i].s2 == pairs[j].s2) {
                /* Keep the pair with smaller merged radius */
                if (pairs[i].merged_radius <= pairs[j].merged_radius) {
                    keep[j] = 0;
                } else {
                    keep[i] = 0;
                    break;
                }
            }
        }
    }

    /* Compact array */
    int write = 0;
    for (int read = 0; read < n; read++) {
        if (keep[read]) {
            if (write != read) {
                pairs[write] = pairs[read];
            }
            write++;
        }
    }

    *n_pairs = write;
    ivp_free(alloc, keep, (size_t)n * sizeof(int));
}

/* ── Execute merges ────────────────────────────────────────────────────── */

static void execute_merges(merge_pair_t *pairs, int n_pairs,
                             bu_sphere_t **spheres, int *n_spheres,
                             bu_arena_t *arena,
                             const ivp_allocator_t *alloc) {
    if (n_pairs == 0) return;

    int orig_n = *n_spheres;

    /* Mark spheres that are being merged */
    int *merged = (int *)ivp_alloc(alloc, (size_t)orig_n * sizeof(int));
    if (!merged) return;

    for (int i = 0; i < orig_n; i++) {
        merged[i] = 0;
    }

    for (int i = 0; i < n_pairs; i++) {
        for (int j = 0; j < orig_n; j++) {
            if (spheres[j] == pairs[i].s1 || spheres[j] == pairs[i].s2) {
                merged[j] = 1;
            }
        }
    }

    /* Create parent spheres and add them to active set */
    int alloc_size = orig_n + n_pairs;
    bu_sphere_t **new_spheres = (bu_sphere_t **)ivp_alloc(alloc,
        (size_t)alloc_size * sizeof(bu_sphere_t *));
    if (!new_spheres) {
        ivp_free(alloc, merged, (size_t)orig_n * sizeof(int));
        return;
    }

    int new_count = 0;

    /* Keep unmerged spheres */
    for (int i = 0; i < orig_n; i++) {
        if (!merged[i]) {
            new_spheres[new_count++] = spheres[i];
        }
    }

    /* Add new parent spheres */
    for (int i = 0; i < n_pairs; i++) {
        bu_sphere_t *parent = bu_alloc(arena);
        parent->center[0] = pairs[i].merged_center[0];
        parent->center[1] = pairs[i].merged_center[1];
        parent->center[2] = pairs[i].merged_center[2];
        parent->radius = pairs[i].merged_radius;
        parent->ledge = NULL;
        parent->child_1 = pairs[i].s1;
        parent->child_2 = pairs[i].s2;

        /* Compute box_sizes from children */
        if (parent->radius > 0.0f) {
            float step = IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;
            float work = parent->radius * step;

            /* Compute extents from children */
            float ext_max[3];
            for (int a = 0; a < 3; a++) {
                float c1_min = pairs[i].s1->center[a] -
                               (float)pairs[i].s1->box_sizes[a] *
                               pairs[i].s1->radius * step;
                float c1_max = pairs[i].s1->center[a] +
                               (float)pairs[i].s1->box_sizes[a] *
                               pairs[i].s1->radius * step;
                float c2_min = pairs[i].s2->center[a] -
                               (float)pairs[i].s2->box_sizes[a] *
                               pairs[i].s2->radius * step;
                float c2_max = pairs[i].s2->center[a] +
                               (float)pairs[i].s2->box_sizes[a] *
                               pairs[i].s2->radius * step;

                (void)c1_min;  /* Only max is used for box_sizes */
                (void)c2_min;
                ext_max[a] = (c1_max > c2_max) ? c1_max : c2_max;
            }

            for (int a = 0; a < 3; a++) {
                int bs = (int)((ext_max[a] - parent->center[a]) / work) + 1;
                parent->box_sizes[a] = (uint8_t)(bs > 255 ? 255 : bs);
            }
        } else {
            parent->box_sizes[0] = parent->box_sizes[1] = parent->box_sizes[2] = 1;
        }

        new_spheres[new_count++] = parent;
    }

    /* Update active set */
    for (int i = 0; i < new_count; i++) {
        spheres[i] = new_spheres[i];
    }
    *n_spheres = new_count;

    /* Cleanup */
    ivp_free(alloc, new_spheres, (size_t)alloc_size * sizeof(bu_sphere_t *));
    ivp_free(alloc, merged, (size_t)orig_n * sizeof(int));
}

/* ── Build bottom-up BVH ───────────────────────────────────────────────── */

/* build_bvh_bottomup returns the root sphere and writes the arena pointer
 * to *arena_nodes_out (with size *arena_sz_out) so the caller can free it. */
static bu_sphere_t *build_bvh_bottomup(ivp_ledge_soup_t *soup,
                                        float radius_tolerance,
                                        const ivp_allocator_t *alloc,
                                        void **arena_nodes_out,
                                        size_t *arena_sz_out) {
    if (!soup || !arena_nodes_out || !arena_sz_out) return NULL;
    *arena_nodes_out = NULL;
    *arena_sz_out = 0;

    int n_ledges = soup->n_ledges;

    /* Allocate arena: at most 2*n_ledges - 1 nodes */
    int max_nodes = 2 * n_ledges;
    size_t arena_sz = (size_t)max_nodes * sizeof(bu_sphere_t);

    bu_arena_t arena;
    arena.nodes = (bu_sphere_t *)ivp_alloc(alloc, arena_sz);
    if (!arena.nodes) return NULL;
    memset(arena.nodes, 0, arena_sz);
    arena.count = 0;
    arena.capacity = max_nodes;

    /* Create terminal spheres */
    bu_sphere_t **active = (bu_sphere_t **)ivp_alloc(alloc,
        (size_t)max_nodes * sizeof(bu_sphere_t *));
    if (!active) {
        ivp_free(alloc, arena.nodes, arena_sz);
        return NULL;
    }

    int n_active = 0;
    for (int i = 0; i < n_ledges; i++) {
        bu_sphere_t *s = bu_alloc(&arena);
        const ivp_compact_ledge_t *ledge = soup->ledges[i];

        float mn[3], mx[3];
        ivp_ledge_calc_bounding_box(ledge, mn, mx);

        for (int a = 0; a < 3; a++) {
            s->center[a] = 0.5f * (mn[a] + mx[a]);
        }
        float dx = mx[0] - s->center[0];
        float dy = mx[1] - s->center[1];
        float dz = mx[2] - s->center[2];
        s->radius = sqrtf(dx*dx + dy*dy + dz*dz);

        if (s->radius > 0.0f) {
            float work = s->radius * IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;
            for (int a = 0; a < 3; a++) {
                int bs = (int)((mx[a] - s->center[a]) / work) + 1;
                s->box_sizes[a] = (uint8_t)(bs > 255 ? 255 : bs);
            }
        } else {
            s->box_sizes[0] = s->box_sizes[1] = s->box_sizes[2] = 1;
        }

        s->ledge = ledge;
        s->child_1 = NULL;
        s->child_2 = NULL;

        active[n_active++] = s;
    }

    /* Allocate merge pair buffer */
    merge_pair_t *pairs = (merge_pair_t *)ivp_alloc(alloc,
        (size_t)max_nodes * sizeof(merge_pair_t));
    if (!pairs) {
        ivp_free(alloc, active, (size_t)max_nodes * sizeof(bu_sphere_t *));
        ivp_free(alloc, arena.nodes, arena_sz);
        return NULL;
    }

    /* Iterative merging */
    float current_tolerance = (float)radius_tolerance;
    const int MAX_ITERATIONS = 10000;
    int iteration = 0;

    while (n_active > 1 && iteration++ < MAX_ITERATIONS) {
        /* Find merge candidates */
        int n_pairs = 0;
        find_merge_candidates(active, n_active, current_tolerance, pairs, &n_pairs);

        if (n_pairs == 0) {
            /* No valid merges at current tolerance; increase by 10% */
            current_tolerance *= 1.1f;
            continue;
        }

        /* Remove conflicts */
        remove_conflicts(pairs, &n_pairs, alloc);

        if (n_pairs == 0) {
            /* All pairs conflicted; increase tolerance */
            current_tolerance *= 1.1f;
            continue;
        }

        /* Execute merges */
        execute_merges(pairs, n_pairs, active, &n_active, &arena, alloc);
    }

    if (n_active != 1) {
        ivp_free(alloc, pairs, (size_t)max_nodes * sizeof(merge_pair_t));
        ivp_free(alloc, active, (size_t)max_nodes * sizeof(bu_sphere_t *));
        ivp_free(alloc, arena.nodes, arena_sz);
        return NULL;
    }

    /* Cleanup and return root */
    bu_sphere_t *root = active[0];
    ivp_free(alloc, pairs, (size_t)max_nodes * sizeof(merge_pair_t));
    ivp_free(alloc, active, (size_t)max_nodes * sizeof(bu_sphere_t *));

    /* Keep arena.nodes alive — it contains the tree */
    *arena_nodes_out = arena.nodes;
    *arena_sz_out = arena_sz;
    return root;
}

/* ── Count BVH nodes ───────────────────────────────────────────────────── */

static int count_bvh_nodes(const bu_sphere_t *node) {
    if (!node->child_1) return 1;  /* terminal */
    return 1 + count_bvh_nodes(node->child_1) + count_bvh_nodes(node->child_2);
}

/* ── Write ledgetree from BVH ──────────────────────────────────────────── */

typedef struct ledgetree_writer {
    ivp_compact_ledgetree_node_t *nodes;
    int write_pos;
    int n_nodes;
} ledgetree_writer_t;

typedef struct ledge_remap {
    const ivp_compact_ledge_t *src;
    ivp_compact_ledge_t *dst;
} ledge_remap_t;

static ivp_compact_ledge_t *find_dst_ledge(const ledge_remap_t *remap, int n,
                                             const ivp_compact_ledge_t *src) {
    for (int i = 0; i < n; i++) {
        if (remap[i].src == src) return remap[i].dst;
    }
    return NULL;
}

static ivp_compact_ledgetree_node_t *write_ledgetree(
    ledgetree_writer_t *w,
    const bu_sphere_t *node,
    const ledge_remap_t *remap, int n_remap)
{
    IVP_ASSERT(w->write_pos < w->n_nodes);
    ivp_compact_ledgetree_node_t *out = &w->nodes[w->write_pos++];

    out->center[0] = node->center[0];
    out->center[1] = node->center[1];
    out->center[2] = node->center[2];
    out->radius = node->radius;
    out->box_sizes[0] = node->box_sizes[0];
    out->box_sizes[1] = node->box_sizes[1];
    out->box_sizes[2] = node->box_sizes[2];
    out->padding = 0;

    if (!node->child_1) {
        /* Terminal node */
        ivp_compact_ledge_t *dst = find_dst_ledge(remap, n_remap, node->ledge);
        IVP_ASSERT(dst != NULL);
        out->offset_right_node = 0;
        out->offset_compact_ledge = (int32_t)((char *)dst - (char *)out);

        /* Update ledge's node_or_client to point back to this tree node */
        dst->node_or_client = (int32_t)((char *)out - (char *)dst);
        ivp_ledge_set_has_children(dst, 0);
    } else {
        /* Internal node: no associated ledge */
        out->offset_compact_ledge = 0;

        /* Left child is always the next node */
        write_ledgetree(w, node->child_1, remap, n_remap);

        /* Right child */
        ivp_compact_ledgetree_node_t *right =
            write_ledgetree(w, node->child_2, remap, n_remap);

        out->offset_right_node = (int32_t)((char *)right - (char *)out);
    }

    return out;
}

/* ── Compute radius and deviation ──────────────────────────────────────── */

static void calc_radius_to_center(const ivp_compact_ledge_t *ledge,
                                   const float center[3],
                                   float *radius_inout,
                                   float *deviation_inout) {
    int n = ivp_ledge_get_n_triangles(ledge);
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);

    for (int t = 0; t < n; t++, tri++) {
        /* Compute unnormalized normal for deviation */
        const ivp_compact_edge_t *e0 = ivp_tri_get_edge(tri, 0);
        const ivp_compact_edge_t *e1 = ivp_tri_get_edge(tri, 1);
        const ivp_compact_edge_t *e2 = ivp_tri_get_edge(tri, 2);

        const ivp_compact_poly_point_t *p0 = ivp_edge_get_start_point(e0, ledge);
        const ivp_compact_poly_point_t *p1 = ivp_edge_get_start_point(e1, ledge);
        const ivp_compact_poly_point_t *p2 = ivp_edge_get_start_point(e2, ledge);

        /* Hesse vector (unnormalized) */
        float ax = p2->k[0] - p0->k[0], ay = p2->k[1] - p0->k[1], az = p2->k[2] - p0->k[2];
        float bx = p1->k[0] - p0->k[0], by = p1->k[1] - p0->k[1], bz = p1->k[2] - p0->k[2];
        float hx = ay * bz - az * by;
        float hy = az * bx - ax * bz;
        float hz = ax * by - ay * bx;
        float h_qlen = hx*hx + hy*hy + hz*hz;
        float inv_h_qlen = (h_qlen > 1e-20f) ? (1.0f / h_qlen) : 0.0f;

        const ivp_compact_poly_point_t *pts[3] = { p0, p1, p2 };
        for (int j = 0; j < 3; j++) {
            float dx = pts[j]->k[0] - center[0];
            float dy = pts[j]->k[1] - center[1];
            float dz = pts[j]->k[2] - center[2];

            float q = dx*dx + dy*dy + dz*dz;
            float r = sqrtf(q);
            if (r > *radius_inout) *radius_inout = r;

            /* Cross product p × hesse for deviation */
            float cx = dy * hz - dz * hy;
            float cy = dz * hx - dx * hz;
            float cz = dx * hy - dy * hx;
            float dev = sqrtf((cx*cx + cy*cy + cz*cz) * inv_h_qlen);
            if (dev > *deviation_inout) *deviation_inout = dev;
        }
    }
}

/* ── Compile ───────────────────────────────────────────────────────────── */

ivp_compact_surface_t *ivp_ledge_soup_compile_bottomup(
    ivp_ledge_soup_t *soup,
    const ivp_bvh_bottomup_config_t *cfg)
{
    if (!soup || soup->n_ledges <= 0) return NULL;

    const ivp_allocator_t *alloc = cfg ? cfg->alloc : NULL;
    if (!alloc) alloc = ivp_default_allocator();

    float radius_tolerance = cfg ? (float)cfg->radius_tolerance : 1.1f;
    if (radius_tolerance <= 0.0f) return NULL;

    int n_ledges = soup->n_ledges;

    /* Single ledge case: delegate to simple wrapper */
    if (n_ledges == 1) {
        const ivp_compact_ledge_t *src = soup->ledges[0];
        int ledge_size = ivp_ledge_get_size(src);
        int header_size = (int)sizeof(ivp_compact_surface_t);
        int node_size = (int)sizeof(ivp_compact_ledgetree_node_t);
        int total = (header_size + ledge_size + node_size + 15) & ~15;

        void *mem = ivp_alloc_aligned(alloc, (size_t)total, 16);
        if (!mem) return NULL;
        memset(mem, 0, (size_t)total);

        ivp_compact_surface_t *surface = (ivp_compact_surface_t *)mem;
        ivp_compact_ledge_t *dst_ledge = (ivp_compact_ledge_t *)((char *)mem + header_size);
        ivp_compact_ledgetree_node_t *node =
            (ivp_compact_ledgetree_node_t *)((char *)dst_ledge + ledge_size);

        memcpy(dst_ledge, src, (size_t)ledge_size);

        /* Set ledgetree node */
        node->offset_right_node = 0;
        node->offset_compact_ledge = (int32_t)((char *)dst_ledge - (char *)node);
        dst_ledge->node_or_client = (int32_t)((char *)node - (char *)dst_ledge);
        ivp_ledge_set_has_children(dst_ledge, 0);

        /* Bounding sphere */
        float center[3];
        float radius = 0.0f;
        ivp_ledge_calc_geom_center_and_radius(dst_ledge, center, &radius);

        float dev = 0.0f;
        calc_radius_to_center(dst_ledge, center, &radius, &dev);

        node->center[0] = center[0];
        node->center[1] = center[1];
        node->center[2] = center[2];
        node->radius = radius;

        if (radius > 0.0f) {
            float inv_step = 1.0f / (radius * IVP_COMPACT_BOUNDINGBOX_STEP_SIZE);
            float mn[3], mx[3];
            ivp_ledge_calc_bounding_box(dst_ledge, mn, mx);
            for (int a = 0; a < 3; a++) {
                int bs = (int)(fabsf(mx[a] - center[a]) * inv_step) + 1;
                node->box_sizes[a] = (uint8_t)(bs > 255 ? 255 : bs);
            }
        } else {
            node->box_sizes[0] = node->box_sizes[1] = node->box_sizes[2] = 1;
        }
        node->padding = 0;
        surface->offset_ledgetree_root = (int32_t)((char *)node - (char *)surface);

        /* Surface header */
        float mass_center[3], rot_inertia[3];
        ivp_calc_mass_center_and_rot_inertia(surface, mass_center, rot_inertia);

        surface->mass_center[0] = mass_center[0];
        surface->mass_center[1] = mass_center[1];
        surface->mass_center[2] = mass_center[2];
        surface->rotation_inertia[0] = rot_inertia[0];
        surface->rotation_inertia[1] = rot_inertia[1];
        surface->rotation_inertia[2] = rot_inertia[2];
        surface->upper_limit_radius = radius;
        ivp_surface_set_byte_size(surface, total);

        int dev_factor = (int)(1.0f + dev / (radius * IVP_COMPACT_SURFACE_DEVIATION_STEP_SIZE + 1e-20f));
        if (dev_factor > 255) dev_factor = 255;
        ivp_surface_set_max_deviation(surface, dev_factor);

        memset(surface->_pad, 0, sizeof(surface->_pad));

        return surface;
    }

    /* Multi-ledge case: build bottom-up BVH */

    /* Phase 1: Build BVH */
    void *arena_nodes = NULL;
    size_t arena_sz = 0;
    bu_sphere_t *root = build_bvh_bottomup(soup, radius_tolerance, alloc,
                                             &arena_nodes, &arena_sz);
    if (!root) return NULL;

    int total_bvh_nodes = count_bvh_nodes(root);

    /* Phase 2: Compute memory layout */
    int header_size = (int)sizeof(ivp_compact_surface_t);
    int total_ledge_data = 0;
    for (int i = 0; i < n_ledges; i++) {
        total_ledge_data += ivp_ledge_get_size(soup->ledges[i]);
    }
    int tree_size = total_bvh_nodes * (int)sizeof(ivp_compact_ledgetree_node_t);
    int total = (header_size + total_ledge_data + tree_size + 15) & ~15;

    void *mem = ivp_alloc_aligned(alloc, (size_t)total, 16);
    if (!mem) {
        ivp_free(alloc, arena_nodes, arena_sz);
        return NULL;
    }
    memset(mem, 0, (size_t)total);

    ivp_compact_surface_t *surface = (ivp_compact_surface_t *)mem;

    /* Phase 3: Copy ledges */
    size_t remap_sz = (size_t)n_ledges * sizeof(ledge_remap_t);
    ledge_remap_t *remap = (ledge_remap_t *)ivp_alloc(alloc, remap_sz);
    if (!remap) {
        ivp_free_aligned(alloc, mem);
        ivp_free(alloc, arena_nodes, arena_sz);
        return NULL;
    }

    char *dst = (char *)mem + header_size;
    for (int i = 0; i < n_ledges; i++) {
        const ivp_compact_ledge_t *src = soup->ledges[i];
        int sz = ivp_ledge_get_size(src);
        memcpy(dst, src, (size_t)sz);

        remap[i].src = src;
        remap[i].dst = (ivp_compact_ledge_t *)dst;
        dst += sz;
    }

    /* Phase 4: Write ledgetree */
    ivp_compact_ledgetree_node_t *tree_start =
        (ivp_compact_ledgetree_node_t *)((char *)mem + header_size + total_ledge_data);

    ledgetree_writer_t writer;
    writer.nodes = tree_start;
    writer.write_pos = 0;
    writer.n_nodes = total_bvh_nodes;

    write_ledgetree(&writer, root, remap, n_ledges);

    /* Phase 5: Surface header */
    surface->offset_ledgetree_root = (int32_t)((char *)tree_start - (char *)surface);
    memset(surface->_pad, 0, sizeof(surface->_pad));

    /* Compute mass center and inertia */
    ivp_surface_set_byte_size(surface, total);
    ivp_surface_set_max_deviation(surface, 1);  /* default; overwritten below */

    float mass_center[3], rot_inertia[3];
    ivp_calc_mass_center_and_rot_inertia(surface, mass_center, rot_inertia);

    surface->mass_center[0] = mass_center[0];
    surface->mass_center[1] = mass_center[1];
    surface->mass_center[2] = mass_center[2];
    surface->rotation_inertia[0] = rot_inertia[0];
    surface->rotation_inertia[1] = rot_inertia[1];
    surface->rotation_inertia[2] = rot_inertia[2];

    /* Compute bounding radius and deviation from mass center */
    float radius = 0.0f;
    float deviation = 0.0f;
    for (int i = 0; i < n_ledges; i++) {
        calc_radius_to_center(remap[i].dst, mass_center, &radius, &deviation);
    }
    surface->upper_limit_radius = radius;

    int dev_factor = (int)(1.0f + deviation /
        (radius * IVP_COMPACT_SURFACE_DEVIATION_STEP_SIZE + 1e-20f));
    if (dev_factor > 255) dev_factor = 255;
    if (dev_factor < 1) dev_factor = 1;
    ivp_surface_set_max_deviation(surface, dev_factor);

    /* Cleanup */
    ivp_free(alloc, remap, remap_sz);
    ivp_free(alloc, arena_nodes, arena_sz);

    return surface;
}
