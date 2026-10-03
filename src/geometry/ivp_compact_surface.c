/* ivp_compact_surface.c — Compact surface functions */

#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <stdlib.h>
#include <string.h>

const ivp_compact_ledge_t *ivp_surface_get_single_convex(
    const ivp_compact_surface_t *s)
{
    const ivp_compact_ledgetree_node_t *root = ivp_surface_get_ledgetree_root(s);
    if (ivp_ledgetree_is_terminal(root)) {
        return ivp_ledgetree_get_compact_ledge(root);
    }
    return NULL;
}

void ivp_compact_surface_free(const ivp_allocator_t *alloc,
                               ivp_compact_surface_t *s)
{
    if (s) {
        if (!alloc) alloc = ivp_default_allocator();
        ivp_free_aligned(alloc, s);
    }
}

/* ── Endian conversion ─────────────────────────────────────────────────── */

static void bs32(void *p)
{
    uint32_t v;
    memcpy(&v, p, 4);
    v = ((v & 0x000000FFu) << 24) | ((v & 0x0000FF00u) << 8) |
        ((v & 0x00FF0000u) >> 8) | ((v & 0xFF000000u) >> 24);
    memcpy(p, &v, 4);
}

static void bs16(void *p)
{
    uint16_t v;
    memcpy(&v, p, 2);
    v = (uint16_t)((v << 8) | (v >> 8));
    memcpy(p, &v, 2);
}

typedef struct bs_points {
    const ivp_allocator_t *alloc;
    ivp_compact_poly_point_t **pts;
    int n;
    int cap;
    bool fail;
} bs_points_t;

static void bs_add_point(bs_points_t *l, ivp_compact_poly_point_t *p)
{
    if (l->fail) return;
    if (l->n >= l->cap) {
        int nc = l->cap ? l->cap * 2 : 64;
        ivp_compact_poly_point_t **np = (ivp_compact_poly_point_t **)ivp_alloc(
            l->alloc, (size_t)nc * sizeof(*np));
        if (!np) {
            l->fail = true;
            return;
        }
        if (l->pts) {
            memcpy(np, l->pts, (size_t)l->n * sizeof(*np));
            ivp_free(l->alloc, l->pts, (size_t)l->cap * sizeof(*np));
        }
        l->pts = np;
        l->cap = nc;
    }
    l->pts[l->n++] = p;
}

static void bs_ledge(ivp_compact_ledge_t *l, bool foreign, bs_points_t *pts)
{
    if (foreign) {
        bs32(&l->c_point_offset);
        bs32(&l->node_or_client);
        bs32(&l->flags_and_size);
        bs16(&l->n_triangles);
        bs16(&l->reserved);
    }
    int n_tri = l->n_triangles;
    ivp_compact_poly_point_t *pa = (ivp_compact_poly_point_t *)((char *)l + l->c_point_offset);
    ivp_compact_triangle_t *tri = (ivp_compact_triangle_t *)(l + 1);
    for (int t = 0; t < n_tri; t++, tri++) {
        if (foreign) {
            bs32(&tri->packed);
            for (int e = 0; e < 3; e++) bs32(&tri->edges[e].packed);
        }
        for (int e = 0; e < 3; e++) {
            bs_add_point(pts, &pa[ivp_edge_get_start_point_index(&tri->edges[e])]);
        }
        if (!foreign) {
            bs32(&tri->packed);
            for (int e = 0; e < 3; e++) bs32(&tri->edges[e].packed);
        }
    }
    if (!foreign) {
        bs32(&l->c_point_offset);
        bs32(&l->node_or_client);
        bs32(&l->flags_and_size);
        bs16(&l->n_triangles);
        bs16(&l->reserved);
    }
}

static void bs_node(ivp_compact_ledgetree_node_t *n, bool foreign, bs_points_t *pts, int depth)
{
    if (depth > 1024) return;
    if (foreign) {
        bs32(&n->offset_right_node);
        bs32(&n->offset_compact_ledge);
        for (int i = 0; i < 3; i++) bs32(&n->center[i]);
        bs32(&n->radius);
    }
    int32_t right = n->offset_right_node;
    int32_t offl = n->offset_compact_ledge;
    if (!foreign) {
        bs32(&n->offset_right_node);
        bs32(&n->offset_compact_ledge);
        for (int i = 0; i < 3; i++) bs32(&n->center[i]);
        bs32(&n->radius);
    }
    if (offl != 0) bs_ledge((ivp_compact_ledge_t *)((char *)n + offl), foreign, pts);
    if (right != 0) {
        /* IVP_Compact_Ledgetree_Node::byte_swap_all (ivp_compact_ledge.cxx:295-300) */
        ivp_compact_ledgetree_node_t *l = n + 1;
        IVP_ASSERT(l);
        bs_node(l, foreign, pts, depth + 1);
        ivp_compact_ledgetree_node_t *r = (ivp_compact_ledgetree_node_t *)((char *)n + right);
        IVP_ASSERT(r);
        bs_node(r, foreign, pts, depth + 1);
    }
}

static int bs_cmp_ptr(const void *a, const void *b)
{
    uintptr_t pa = (uintptr_t)*(ivp_compact_poly_point_t *const *)a;
    uintptr_t pb = (uintptr_t)*(ivp_compact_poly_point_t *const *)b;
    return (pa > pb) - (pa < pb);
}

static void bs_header(ivp_compact_surface_t *s)
{
    for (int i = 0; i < 3; i++) {
        bs32(&s->mass_center[i]);
        bs32(&s->rotation_inertia[i]);
    }
    bs32(&s->upper_limit_radius);
    bs32(&s->size_and_deviation);
    bs32(&s->offset_ledgetree_root);
    for (int i = 0; i < 3; i++) bs32(&s->_pad[4 * i]);
}

bool ivp_compact_surface_byte_swap_all(ivp_compact_surface_t *s, bool data_is_foreign)
{
    if (!s) return false;
    if (data_is_foreign) bs_header(s);
    int32_t root_off = s->offset_ledgetree_root;
    if (!data_is_foreign) bs_header(s);

    bs_points_t pts;
    memset(&pts, 0, sizeof(pts));
    pts.alloc = ivp_default_allocator();
    bs_node((ivp_compact_ledgetree_node_t *)((char *)s + root_off), data_is_foreign, &pts, 0);

    if (pts.n > 1) qsort(pts.pts, (size_t)pts.n, sizeof(*pts.pts), bs_cmp_ptr);
    int n_unique = 0; /* swapped_points.len() of the reference */
    for (int i = 0; i < pts.n; i++) {
        if (i > 0 && pts.pts[i] == pts.pts[i - 1]) continue;
        for (int k = 0; k < 3; k++) bs32(&pts.pts[i]->k[k]);
        bs32(&pts.pts[i]->hesse_val);
        n_unique++;
    }
    IVP_IF(1) { ivp_message("Found %d unique points\n", n_unique); }
    if (pts.pts) ivp_free(pts.alloc, pts.pts, (size_t)pts.cap * sizeof(*pts.pts));
    return !pts.fail;
}
