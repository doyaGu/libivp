/* ivp_build_ledge_soup.c -- Ledge soup builder implementation
 *
 * Ported from ivp_surbuild_ledge_soup.cxx (Ipion Software GmbH).
 *
 * Algorithm:
 *   1. Wrap each input ledge in a bounding sphere + AABB.
 *   2. Build a BVH via top-down balanced median-cut:
 *      - For each set of spheres, try all 3 axes.
 *      - Split at the mean center position on each axis.
 *      - Choose the axis that minimizes the sum of child bounding volumes.
 *      - Recurse until each leaf is a single sphere.
 *   3. Allocate contiguous memory: [surface header][ledge copies][ledgetree nodes].
 *   4. Copy ledge data into the contiguous block.
 *   5. Write ledgetree nodes from the BVH.
 *   6. Compute mass center, inertia, bounding radius.
 */
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/builder/ivp_rot_inertia.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/base/ivp_assert.h>

#include <math.h>
#include <string.h>

/* ── BVH sphere node ───────────────────────────────────────────────────── */

typedef struct bvh_sphere {
    float center[3];
    float radius;
    uint8_t box_sizes[3];
    const ivp_compact_ledge_t *ledge;  /* non-NULL for terminal nodes */
    struct bvh_sphere *child_1;
    struct bvh_sphere *child_2;
} bvh_sphere_t;

/* ── Internal arena for BVH nodes ──────────────────────────────────────── */

typedef struct bvh_arena {
    bvh_sphere_t *nodes;
    int count;
    int capacity;
} bvh_arena_t;

static bvh_sphere_t *bvh_alloc(bvh_arena_t *arena) {
    IVP_ASSERT(arena->count < arena->capacity);
    bvh_sphere_t *n = &arena->nodes[arena->count++];
    memset(n, 0, sizeof(*n));
    return n;
}

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

void ivp_ledge_soup_init(ivp_ledge_soup_t *soup, const ivp_allocator_t *alloc) {
    if (!soup) return;
    if (!alloc) alloc = ivp_default_allocator();
    soup->alloc = alloc;
    soup->n_ledges = 0;
    soup->capacity = 16;
    soup->ledges = (const ivp_compact_ledge_t **)ivp_alloc(alloc,
        (size_t)soup->capacity * sizeof(ivp_compact_ledge_t *));
}

void ivp_ledge_soup_destroy(ivp_ledge_soup_t *soup) {
    if (!soup) return;
    if (soup->ledges) {
        ivp_free(soup->alloc, soup->ledges,
                  (size_t)soup->capacity * sizeof(ivp_compact_ledge_t *));
        soup->ledges = NULL;
    }
    soup->n_ledges = 0;
    soup->capacity = 0;
}

int ivp_ledge_soup_add(ivp_ledge_soup_t *soup, const ivp_compact_ledge_t *ledge) {
    if (!soup) return -1;
    if (!ledge) return -1;
    if (!soup->ledges || soup->capacity <= 0) return -1;
    if (soup->n_ledges >= IVP_LEDGE_SOUP_MAX_LEDGES) return -1;

    if (soup->n_ledges >= soup->capacity) {
        int new_cap = soup->capacity * 2;
        const ivp_compact_ledge_t **new_arr = (const ivp_compact_ledge_t **)ivp_alloc(
            soup->alloc, (size_t)new_cap * sizeof(ivp_compact_ledge_t *));
        if (!new_arr) return -1;
        memcpy(new_arr, soup->ledges,
               (size_t)soup->n_ledges * sizeof(ivp_compact_ledge_t *));
        ivp_free(soup->alloc, soup->ledges,
                  (size_t)soup->capacity * sizeof(ivp_compact_ledge_t *));
        soup->ledges = new_arr;
        soup->capacity = new_cap;
    }

    soup->ledges[soup->n_ledges++] = ledge;
    return 0;
}

/* ── BVH construction ──────────────────────────────────────────────────── */

static void calc_bounding_from_spheres(bvh_sphere_t **spheres, int n,
                                        float ext_min[3], float ext_max[3]) {
    float step = IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;

    ext_min[0] = ext_min[1] = ext_min[2] = 1e10f;
    ext_max[0] = ext_max[1] = ext_max[2] = -1e10f;

    for (int i = 0; i < n; i++) {
        bvh_sphere_t *s = spheres[i];
        float work = step * s->radius;
        for (int a = 0; a < 3; a++) {
            float lo = s->center[a] - s->box_sizes[a] * work;
            float hi = s->center[a] + s->box_sizes[a] * work;
            if (lo < ext_min[a]) ext_min[a] = lo;
            if (hi > ext_max[a]) ext_max[a] = hi;
        }
    }
}

/* Insertion sort by sphere center on a given axis.
 * Replaces qsort + static global to avoid thread-safety issues.
 * Arrays are small (number of physics ledges), so O(n^2) is fine. */
static void sort_spheres_by_axis(bvh_sphere_t **spheres, int n, int axis) {
    for (int i = 1; i < n; i++) {
        bvh_sphere_t *key = spheres[i];
        int j = i - 1;
        while (j >= 0 && spheres[j]->center[axis] > key->center[axis]) {
            spheres[j + 1] = spheres[j];
            j--;
        }
        spheres[j + 1] = key;
    }
}

static bvh_sphere_t *build_bvh_recursive(bvh_sphere_t **spheres, int n,
                                          bvh_arena_t *arena,
                                          bvh_sphere_t **temp_buf) {
    IVP_ASSERT(n > 0);

    if (n == 1) return spheres[0];

    /* Compute bounding sphere */
    float ext_min[3], ext_max[3];
    calc_bounding_from_spheres(spheres, n, ext_min, ext_max);

    bvh_sphere_t *node = bvh_alloc(arena);
    float step = IVP_COMPACT_BOUNDINGBOX_STEP_SIZE;

    for (int a = 0; a < 3; a++) {
        node->center[a] = 0.5f * (ext_min[a] + ext_max[a]);
    }

    float dx = ext_max[0] - node->center[0];
    float dy = ext_max[1] - node->center[1];
    float dz = ext_max[2] - node->center[2];
    node->radius = sqrtf(dx * dx + dy * dy + dz * dz);
    node->ledge = NULL;

    if (node->radius > 0.0f) {
        float work = node->radius * step;
        for (int a = 0; a < 3; a++) {
            int bs = (int)((ext_max[a] - node->center[a]) / work) + 1;
            node->box_sizes[a] = (uint8_t)(bs > 255 ? 255 : bs);
        }
    } else {
        node->box_sizes[0] = node->box_sizes[1] = node->box_sizes[2] = 1;
    }

    /* Base case: two spheres */
    if (n == 2) {
        node->child_1 = spheres[0];
        node->child_2 = spheres[1];
        return node;
    }

    /* Try all 3 axes, pick the best split */
    int best_axis = 0;
    float best_cost = 1e20f;
    int best_split = n / 2;

    for (int axis = 0; axis < 3; axis++) {
        /* Compute mean center on this axis */
        float mean = 0.0f;
        for (int i = 0; i < n; i++) {
            mean += spheres[i]->center[axis];
        }
        mean /= (float)n;

        /* Partition around mean */
        int n_left = 0;
        for (int i = 0; i < n; i++) {
            if (spheres[i]->center[axis] < mean) {
                n_left++;
            }
        }

        /* Ensure both sides have at least one element */
        if (n_left == 0) n_left = 1;
        if (n_left >= n) n_left = n - 1;

        /* Sort on this axis to get proper partition */
        memcpy(temp_buf, spheres, (size_t)n * sizeof(bvh_sphere_t *));
        sort_spheres_by_axis(temp_buf, n, axis);

        /* Compute bounding volumes of left and right subsets */
        float left_min[3], left_max[3], right_min[3], right_max[3];
        calc_bounding_from_spheres(temp_buf, n_left, left_min, left_max);
        calc_bounding_from_spheres(temp_buf + n_left, n - n_left, right_min, right_max);

        float lv = fabsf(left_max[0] - left_min[0])
                  * fabsf(left_max[1] - left_min[1])
                  * fabsf(left_max[2] - left_min[2]);
        float rv = fabsf(right_max[0] - right_min[0])
                  * fabsf(right_max[1] - right_min[1])
                  * fabsf(right_max[2] - right_min[2]);
        float cost = lv + rv;

        if (cost < best_cost) {
            best_cost = cost;
            best_axis = axis;
            best_split = n_left;
        }
    }

    /* Final sort on best axis */
    sort_spheres_by_axis(spheres, n, best_axis);

    /* Ensure split is valid */
    if (best_split <= 0) best_split = 1;
    if (best_split >= n) best_split = n - 1;

    node->child_1 = build_bvh_recursive(spheres, best_split, arena, temp_buf);
    node->child_2 = build_bvh_recursive(spheres + best_split, n - best_split, arena, temp_buf);

    return node;
}

/* ── Count BVH nodes ───────────────────────────────────────────────────── */

static int count_bvh_nodes(const bvh_sphere_t *node) {
    if (!node->child_1) return 1;  /* terminal */
    return 1 + count_bvh_nodes(node->child_1) + count_bvh_nodes(node->child_2);
}

/* ── Write ledgetree from BVH ──────────────────────────────────────────── */

typedef struct ledgetree_writer {
    ivp_compact_ledgetree_node_t *nodes;  /* output array */
    int write_pos;
    int n_nodes;
} ledgetree_writer_t;

/* Maps input ledge pointer → destination ledge pointer in contiguous block */
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
    const bvh_sphere_t *node,
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

ivp_compact_surface_t *ivp_ledge_soup_compile(ivp_ledge_soup_t *soup) {
    if (!soup || soup->n_ledges <= 0) return NULL;

    const ivp_allocator_t *alloc = soup->alloc;
    int n_ledges = soup->n_ledges;

    /* Special case: single ledge → wrap in a simple surface */
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

        /* Fixup: update c_point_offset stays the same (relative to ledge) */
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

        /* If rot_inertia calc fails (surface not yet complete), use bbox estimate */
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

        memset(surface->dummy, 0, sizeof(surface->dummy));

        return surface;
    }

    /* Multi-ledge case: build BVH and compile into contiguous surface */

    /* Phase 1: Create terminal bounding spheres */
    /* Arena: at most 2*n_ledges - 1 nodes (n terminals + n-1 internal) */
    int max_nodes = 2 * n_ledges;
    size_t arena_sz = (size_t)max_nodes * sizeof(bvh_sphere_t);
    size_t ptrs_sz  = (size_t)n_ledges * sizeof(bvh_sphere_t *);

    bvh_arena_t arena;
    arena.nodes = (bvh_sphere_t *)ivp_alloc(alloc, arena_sz);
    if (!arena.nodes) return NULL;
    memset(arena.nodes, 0, arena_sz);
    arena.count = 0;
    arena.capacity = max_nodes;

    bvh_sphere_t **terminal_ptrs = (bvh_sphere_t **)ivp_alloc(alloc, ptrs_sz);
    bvh_sphere_t **temp_buf = (bvh_sphere_t **)ivp_alloc(alloc, ptrs_sz);
    if (!terminal_ptrs || !temp_buf) {
        ivp_free(alloc, arena.nodes, arena_sz);
        if (terminal_ptrs) ivp_free(alloc, terminal_ptrs, ptrs_sz);
        if (temp_buf) ivp_free(alloc, temp_buf, ptrs_sz);
        return NULL;
    }

    for (int i = 0; i < n_ledges; i++) {
        bvh_sphere_t *s = bvh_alloc(&arena);
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

        terminal_ptrs[i] = s;
    }

    /* Phase 2: Build BVH */
    bvh_sphere_t *root = build_bvh_recursive(terminal_ptrs, n_ledges, &arena, temp_buf);
    ivp_free(alloc, temp_buf, ptrs_sz);

    int total_bvh_nodes = count_bvh_nodes(root);

    /* Phase 3: Compute memory layout */
    int header_size = (int)sizeof(ivp_compact_surface_t);
    int total_ledge_data = 0;
    for (int i = 0; i < n_ledges; i++) {
        total_ledge_data += ivp_ledge_get_size(soup->ledges[i]);
    }
    int tree_size = total_bvh_nodes * (int)sizeof(ivp_compact_ledgetree_node_t);
    int total = (header_size + total_ledge_data + tree_size + 15) & ~15;

    void *mem = ivp_alloc_aligned(alloc, (size_t)total, 16);
    if (!mem) {
        ivp_free(alloc, terminal_ptrs, ptrs_sz);
        ivp_free(alloc, arena.nodes, arena_sz);
        return NULL;
    }
    memset(mem, 0, (size_t)total);

    ivp_compact_surface_t *surface = (ivp_compact_surface_t *)mem;

    /* Phase 4: Copy ledges */
    size_t remap_sz = (size_t)n_ledges * sizeof(ledge_remap_t);
    ledge_remap_t *remap = (ledge_remap_t *)ivp_alloc(alloc, remap_sz);
    if (!remap) {
        ivp_free_aligned(alloc, mem);
        ivp_free(alloc, terminal_ptrs, ptrs_sz);
        ivp_free(alloc, arena.nodes, arena_sz);
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

    /* Phase 5: Write ledgetree */
    ivp_compact_ledgetree_node_t *tree_start =
        (ivp_compact_ledgetree_node_t *)((char *)mem + header_size + total_ledge_data);

    ledgetree_writer_t writer;
    writer.nodes = tree_start;
    writer.write_pos = 0;
    writer.n_nodes = total_bvh_nodes;

    write_ledgetree(&writer, root, remap, n_ledges);

    /* Phase 6: Surface header */
    surface->offset_ledgetree_root = (int32_t)((char *)tree_start - (char *)surface);
    memset(surface->dummy, 0, sizeof(surface->dummy));

    /* Compute mass center and inertia */
    ivp_surface_set_byte_size(surface, total);
    ivp_surface_set_max_deviation(surface, 1);  /* default; overwritten below after radius computation */

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
    ivp_free(alloc, terminal_ptrs, ptrs_sz);
    ivp_free(alloc, arena.nodes, arena_sz);

    return surface;
}
