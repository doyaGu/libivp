/* ivp_build_compact_recursive.c — Recursive (hull) compact ledges
 *
 * Port of ivp_compact_builder/ivp_compact_recursive.cxx (IVP_Compact_Recursive).
 */
#include <ivp/builder/ivp_build_compact_recursive.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/containers/ivp_hashmap.h>
#include <ivp/base/ivp_assert.h>
#include "ivp_build_checked.h"
#include <string.h>

static uint32_t cr_hash(const void *key, size_t len)
{
    const unsigned char *b = (const unsigned char *)key;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        h ^= b[i];
        h *= 16777619u;
    }
    return h;
}

/* Collect distinct points (bitwise float identity) of all ledges in the
 * reference visiting order: ledges from last to first, triangles forward,
 * edges 0..2 (build_convex_hull, ivp_compact_recursive.cxx:39). */
static int cr_collect_points(const ivp_allocator_t *alloc,
                             const ivp_compact_ledge_t *const *ledges, int n_ledges,
                             double **out, size_t *out_sz)
{
    int total = 0;
    for (int k = 0; k < n_ledges; k++) {
        if (ledges[k]) {
            int ledge_points;
            if (!ivp_build_int_mul(ivp_ledge_get_n_triangles(ledges[k]), 3,
                                   &ledge_points) ||
                !ivp_build_int_add(total, ledge_points, &total)) return -1;
        }
    }
    if (total <= 0) return -1;
    int n_coords;
    size_t sz;
    if (!ivp_build_int_mul(total, 3, &n_coords) ||
        !ivp_build_count_bytes(n_coords, sizeof(double), &sz)) return -1;
    double *pts = (double *)ivp_alloc(alloc, sz);
    if (!pts) return -1;

    ivp_hashmap_t map;
    int minimum_capacity;
    int cap;
    if (!ivp_build_int_mul(total, 2, &minimum_capacity) ||
        !ivp_build_int_grow_pow2(minimum_capacity, &cap)) {
        ivp_free(alloc, pts, sz);
        return -1;
    }
    if (!ivp_ok(ivp_hashmap_init(&map, alloc, cap, cr_hash, ivp_eq_memcmp, 3 * sizeof(float)))) {
        ivp_free(alloc, pts, sz);
        return -1;
    }
    int n = 0;
    for (int k = n_ledges - 1; k >= 0; k--) {
        const ivp_compact_ledge_t *ledge = ledges[k];
        if (!ledge) continue;
        const ivp_compact_poly_point_t *pa = ivp_ledge_get_point_array(ledge);
        const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
        int nt = ivp_ledge_get_n_triangles(ledge);
        for (int t = 0; t < nt; t++, tri++) {
            for (int e = 0; e < 3; e++) {
                const ivp_compact_poly_point_t *p = &pa[ivp_edge_get_start_point_index(&tri->edges[e])];
                if (ivp_hashmap_contains(&map, p->k)) continue;
                (void)ivp_hashmap_put(&map, p->k, (void *)1);
                pts[3 * n + 0] = (double)p->k[0];
                pts[3 * n + 1] = (double)p->k[1];
                pts[3 * n + 2] = (double)p->k[2];
                n++;
            }
        }
    }
    ivp_hashmap_destroy(&map);
    *out = pts;
    *out_sz = sz;
    return n;
}

/* ── set_rekursive_convex_hull ─────────────────────────────────────────── */

typedef struct cr_tri_key { int p[3]; } cr_tri_key_t;
typedef struct cr_edge_key { int p[2]; } cr_edge_key_t;

static void cr_sort3(int *a, int *b, int *c)
{
    int x;
    if (*b < *a) { x = *a; *a = *b; *b = x; }
    if (*c < *a) { x = *a; *a = *c; *c = x; }
    if (*c < *b) { x = *b; *b = *c; *c = x; }
}

void ivp_compact_recursive_set_virtual_flags(ivp_compact_ledge_t *hull,
                                             const ivp_compact_ledge_t *const *ledges,
                                             int n_ledges)
{
    /* ivp_compact_recursive.cxx:68 */
    if (!hull || !ledges || n_ledges <= 0) return;
    const ivp_allocator_t *alloc = ivp_default_allocator();

    int total_tri = 0;
    for (int i = 0; i < n_ledges; i++) {
        if (ledges[i] &&
            !ivp_build_int_add(total_tri, ivp_ledge_get_n_triangles(ledges[i]),
                               &total_tri)) return;
    }
    if (total_tri <= 0) return;
    int edge_count;
    int hash_entries;
    int minimum_capacity;
    int cap;
    if (!ivp_build_int_mul(total_tri, 3, &edge_count) ||
        !ivp_build_int_mul(total_tri, 6, &hash_entries) ||
        !ivp_build_int_add(hash_entries, 16, &minimum_capacity) ||
        !ivp_build_int_grow_pow2(minimum_capacity, &cap)) return;

    /* point identity -> index; triangle / edge key storage */
    size_t tkeys_sz;
    size_t ekeys_sz;
    if (!ivp_build_count_bytes(total_tri, sizeof(cr_tri_key_t), &tkeys_sz) ||
        !ivp_build_count_bytes(edge_count, sizeof(cr_edge_key_t), &ekeys_sz)) return;
    cr_tri_key_t *tkeys = (cr_tri_key_t *)ivp_alloc(alloc, tkeys_sz);
    cr_edge_key_t *ekeys = (cr_edge_key_t *)ivp_alloc(alloc, ekeys_sz);
    ivp_hashmap_t pmap, tmap, emap;
    bool p_ok = false, t_ok = false, e_ok = false;
    if (!tkeys || !ekeys) goto done;
    p_ok = ivp_ok(ivp_hashmap_init(&pmap, alloc, cap, cr_hash, ivp_eq_memcmp, 3 * sizeof(float)));
    t_ok = ivp_ok(ivp_hashmap_init(&tmap, alloc, cap, cr_hash, ivp_eq_memcmp, sizeof(cr_tri_key_t)));
    e_ok = ivp_ok(ivp_hashmap_init(&emap, alloc, cap, cr_hash, ivp_eq_memcmp, sizeof(cr_edge_key_t)));
    if (!p_ok || !t_ok || !e_ok) goto done;

    int n_points = 0, n_tkeys = 0, n_ekeys = 0;
    for (int li = n_ledges - 1; li >= 0; li--) {
        const ivp_compact_ledge_t *ledge = ledges[li];
        if (!ledge) continue;
        const ivp_compact_poly_point_t *pa = ivp_ledge_get_point_array(ledge);
        const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
        int nt = ivp_ledge_get_n_triangles(ledge);
        for (int t = 0; t < nt; t++, tri++) {
            int pi[3];
            for (int i = 0; i < 3; i++) {
                const float *k = pa[ivp_edge_get_start_point_index(&tri->edges[i])].k;
                void *v = ivp_hashmap_get(&pmap, k);
                if (!v) {
                    (void)ivp_hashmap_put(&pmap, k, (void *)((uintptr_t)n_points + 1u));
                    pi[i] = n_points++;
                } else {
                    pi[i] = (int)((uintptr_t)v - 1u);
                }
            }
            cr_tri_key_t *tk = &tkeys[n_tkeys];
            tk->p[0] = pi[0]; tk->p[1] = pi[1]; tk->p[2] = pi[2];
            cr_sort3(&tk->p[0], &tk->p[1], &tk->p[2]);
            if (!ivp_hashmap_contains(&tmap, tk)) {
                (void)ivp_hashmap_put(&tmap, tk, (void *)1);
                n_tkeys++;
            }
            for (int j = 0; j < 3; j++) {
                cr_edge_key_t *ek = &ekeys[n_ekeys];
                int a = pi[j], b = pi[(j + 1) % 3];
                ek->p[0] = a < b ? a : b;
                ek->p[1] = a < b ? b : a;
                if (!ivp_hashmap_contains(&emap, ek)) {
                    (void)ivp_hashmap_put(&emap, ek, (void *)1);
                    n_ekeys++;
                }
            }
        }
    }

    {
        const ivp_compact_poly_point_t *pa = ivp_ledge_get_point_array(hull);
        ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle_mut(hull);
        int nt = ivp_ledge_get_n_triangles(hull);
        for (int t = 0; t < nt; t++, tri++) {
            int pi[3];
            for (int i = 0; i < 3; i++) {
                void *v = ivp_hashmap_get(&pmap, pa[ivp_edge_get_start_point_index(&tri->edges[i])].k);
                pi[i] = v ? (int)((uintptr_t)v - 1u) : -1 - i; /* distinct "not found" ids */
            }
            cr_tri_key_t tk = { { pi[0], pi[1], pi[2] } };
            cr_sort3(&tk.p[0], &tk.p[1], &tk.p[2]);
            if (!ivp_hashmap_contains(&tmap, &tk)) ivp_tri_set_is_virtual(tri, 1);
            for (int j = 0; j < 3; j++) {
                int a = pi[j], b = pi[(j + 1) % 3];
                cr_edge_key_t ek = { { a < b ? a : b, a < b ? b : a } };
                if (!ivp_hashmap_contains(&emap, &ek)) ivp_edge_set_is_virtual(&tri->edges[j], 1);
            }
        }
        IVP_IF(1) {
            /* printf("extra convex hull rekursive: edges not found %i found %i\n", edges_not_found, edges_found); */
        }
    }

done:
    if (p_ok) ivp_hashmap_destroy(&pmap);
    if (t_ok) ivp_hashmap_destroy(&tmap);
    if (e_ok) ivp_hashmap_destroy(&emap);
    if (tkeys) ivp_free(alloc, tkeys, tkeys_sz);
    if (ekeys) ivp_free(alloc, ekeys, ekeys_sz);
}

ivp_compact_ledge_t *ivp_compact_recursive_build_hull(const ivp_allocator_t *alloc,
                                                      const ivp_compact_ledge_t *const *ledges,
                                                      int n_ledges)
{
    /* IVP_Compact_Recursive::compile() */
    if (!ledges || n_ledges <= 0) return NULL;
    if (!alloc) alloc = ivp_default_allocator();
    double *pts = NULL;
    size_t sz = 0;
    int n = cr_collect_points(alloc, ledges, n_ledges, &pts, &sz);
    if (n < 0) return NULL;
    ivp_compact_ledge_t *hull = ivp_build_convex_ledge_d(alloc, pts, n);
    ivp_free(alloc, pts, sz);
    if (hull) ivp_compact_recursive_set_virtual_flags(hull, ledges, n_ledges);
    return hull;
}

ivp_compact_surface_t *ivp_build_compound_with_root_hull(const ivp_allocator_t *alloc,
                                                         const ivp_compact_ledge_t **ledges,
                                                         int n_ledges)
{
    if (!ledges || n_ledges <= 0) return NULL;
    if (!alloc) alloc = ivp_default_allocator();
    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, alloc);
    for (int i = 0; i < n_ledges; i++) {
        if (ivp_ledge_soup_add(&soup, ledges[i]) != 0) {
            ivp_ledge_soup_destroy(&soup);
            return NULL;
        }
    }
    ivp_ledge_soup_template_t t;
    ivp_ledge_soup_template_init(&t);
    t.build_root_convex_hull = true;
    ivp_compact_surface_t *s = ivp_ledge_soup_compile_ex(&soup, &t);
    ivp_ledge_soup_destroy(&soup);
    return s;
}

ivp_compact_surface_t *ivp_build_root_convex_hull(const ivp_allocator_t *alloc,
                                                  const ivp_compact_ledge_t **ledges,
                                                  int n_ledges)
{
    if (!alloc) alloc = ivp_default_allocator();
    ivp_compact_ledge_t *hull = ivp_compact_recursive_build_hull(alloc, ledges, n_ledges);
    if (!hull) return NULL;
    /* standalone surface: the hull is a terminal convex ledge here */
    for (int t = 0; t < ivp_ledge_get_n_triangles(hull); t++) {
        ivp_compact_triangle_t *tri = &ivp_ledge_get_first_triangle_mut(hull)[t];
        ivp_tri_set_is_virtual(tri, 0);
        for (int e = 0; e < 3; e++) ivp_edge_set_is_virtual(&tri->edges[e], 0);
    }
    ivp_compact_surface_t *s = ivp_build_surface_from_ledge(alloc, hull);
    ivp_compact_ledge_free(alloc, hull);
    return s;
}

int ivp_extract_ledge_vertices(const ivp_compact_ledge_t **ledges,
                               int n_ledges,
                               float **out_coords,
                               int *out_count,
                               const ivp_allocator_t *alloc)
{
    if (!ledges || n_ledges < 1 || !out_coords || !out_count) return -1;
    if (!alloc) alloc = ivp_default_allocator();
    double *pts = NULL;
    size_t sz = 0;
    int n = cr_collect_points(alloc, (const ivp_compact_ledge_t *const *)ledges, n_ledges, &pts, &sz);
    if (n <= 0) return -1;
    int n_coords;
    size_t coords_sz;
    if (!ivp_build_int_mul(n, 3, &n_coords) ||
        !ivp_build_count_bytes(n_coords, sizeof(float), &coords_sz)) {
        ivp_free(alloc, pts, sz);
        return -1;
    }
    float *coords = (float *)ivp_alloc(alloc, coords_sz);
    if (!coords) {
        ivp_free(alloc, pts, sz);
        return -1;
    }
    for (int i = 0; i < n_coords; i++) coords[i] = (float)pts[i];
    ivp_free(alloc, pts, sz);
    *out_coords = coords;
    *out_count = n;
    return 0;
}
