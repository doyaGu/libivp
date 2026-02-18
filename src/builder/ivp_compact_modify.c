/* ivp_compact_modify.c — Surface modification utilities
 *
 * Chop and shrink operations on compact surfaces.
 */
#include <ivp/builder/ivp_compact_modify.h>
#include <ivp/builder/ivp_build_halfspacesoup.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <math.h>
#include <string.h>

/* ── Helpers ──────────────────────────────────────────────────────────── */

/* Extract halfspace planes from a convex ledge.
 * Each triangle defines a face plane; duplicate normals are merged.
 * Returns count of planes written, or -1 on failure. */
static int extract_halfspaces_from_ledge(const ivp_compact_ledge_t *ledge,
                                          ivp_halfspace_t *out,
                                          int max_planes)
{
    int n_tri = ivp_ledge_get_n_triangles(ledge);
    const ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array(ledge);
    const ivp_compact_triangle_t *tris = ivp_ledge_get_first_triangle(ledge);
    int count = 0;

    for (int i = 0; i < n_tri && count < max_planes; i++) {
        /* Get triangle vertex positions */
        int i0 = ivp_edge_get_start_point_index(&tris[i].edges[0]);
        int i1 = ivp_edge_get_start_point_index(&tris[i].edges[1]);
        int i2 = ivp_edge_get_start_point_index(&tris[i].edges[2]);

        float ax = pts[i1].k[0] - pts[i0].k[0];
        float ay = pts[i1].k[1] - pts[i0].k[1];
        float az = pts[i1].k[2] - pts[i0].k[2];
        float bx = pts[i2].k[0] - pts[i0].k[0];
        float by = pts[i2].k[1] - pts[i0].k[1];
        float bz = pts[i2].k[2] - pts[i0].k[2];

        /* Normal = a × b */
        float nx = ay * bz - az * by;
        float ny = az * bx - ax * bz;
        float nz = ax * by - ay * bx;

        float len = sqrtf(nx * nx + ny * ny + nz * nz);
        if (len < 1e-8f) continue;
        float inv = 1.0f / len;
        nx *= inv; ny *= inv; nz *= inv;

        /* Check for duplicate normal (within tolerance) */
        bool duplicate = false;
        for (int j = 0; j < count; j++) {
            float dot = out[j].normal[0] * nx +
                        out[j].normal[1] * ny +
                        out[j].normal[2] * nz;
            if (dot > 0.9999f) {
                duplicate = true;
                /* Keep the larger distance (outermost plane) */
                float dist = nx * pts[i0].k[0] +
                             ny * pts[i0].k[1] +
                             nz * pts[i0].k[2];
                if (dist > out[j].dist) out[j].dist = dist;
                break;
            }
        }

        if (!duplicate) {
            out[count].normal[0] = nx;
            out[count].normal[1] = ny;
            out[count].normal[2] = nz;
            out[count].dist = nx * pts[i0].k[0] +
                              ny * pts[i0].k[1] +
                              nz * pts[i0].k[2];
            count++;
        }
    }

    return count;
}

/* ── Chop ─────────────────────────────────────────────────────────────── */

ivp_compact_surface_t *ivp_compact_modify_chop(
    const ivp_allocator_t *alloc,
    const ivp_compact_surface_t *surface_in,
    const float chop_normal[3],
    float chop_depth)
{
    if (!alloc) alloc = ivp_default_allocator();
    if (!surface_in) return NULL;

    /* Only supports single convex ledge */
    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(surface_in);
    if (!ledge) return NULL;

    /* Extract halfspace planes from the ledge */
    ivp_halfspace_t planes[256];
    int n_planes = extract_halfspaces_from_ledge(ledge, planes, 255);
    if (n_planes < 4) return NULL;

    /* Add the chop plane */
    float len = sqrtf(chop_normal[0] * chop_normal[0] +
                      chop_normal[1] * chop_normal[1] +
                      chop_normal[2] * chop_normal[2]);
    if (len < 1e-8f) return NULL;
    float inv = 1.0f / len;

    planes[n_planes].normal[0] = -chop_normal[0] * inv;
    planes[n_planes].normal[1] = -chop_normal[1] * inv;
    planes[n_planes].normal[2] = -chop_normal[2] * inv;
    planes[n_planes].dist = -chop_depth * inv;
    n_planes++;

    /* Build new surface from modified halfspaces */
    return ivp_build_halfspacesoup_surface(alloc, planes, n_planes, 1e-4f);
}

/* ── Shrink ledge ─────────────────────────────────────────────────────── */

ivp_compact_surface_t *ivp_compact_modify_shrink_ledge(
    const ivp_allocator_t *alloc,
    const ivp_compact_ledge_t *ledge_in,
    float shrink_value,
    float merge_threshold)
{
    if (!alloc) alloc = ivp_default_allocator();
    if (!ledge_in || shrink_value <= 0.0f) return NULL;
    if (merge_threshold <= 0.0f) merge_threshold = 1e-4f;

    /* Extract halfspace planes */
    ivp_halfspace_t planes[256];
    int n_planes = extract_halfspaces_from_ledge(ledge_in, planes, 256);
    if (n_planes < 4) return NULL;

    /* Shrink: shift each plane inward by shrink_value */
    for (int i = 0; i < n_planes; i++) {
        planes[i].dist -= shrink_value;
    }

    /* Build new surface from shrunken halfspaces */
    return ivp_build_halfspacesoup_surface(alloc, planes, n_planes,
                                            merge_threshold);
}

/* ── Shrink full surface ──────────────────────────────────────────────── */

/* Visitor context for collecting terminal ledges */
typedef struct {
    const ivp_compact_ledge_t **ledges;
    int count;
    int capacity;
} collect_ctx_t;

static bool collect_visitor(const ivp_compact_ledge_t *ledge, void *user_data) {
    collect_ctx_t *ctx = (collect_ctx_t *)user_data;
    if (ctx->count < ctx->capacity) {
        ctx->ledges[ctx->count++] = ledge;
    }
    return true;
}

ivp_compact_surface_t *ivp_compact_modify_shrink(
    const ivp_allocator_t *alloc,
    const ivp_compact_surface_t *surface_in,
    float shrink_value,
    float merge_threshold)
{
    if (!alloc) alloc = ivp_default_allocator();
    if (!surface_in || shrink_value <= 0.0f) return NULL;
    if (merge_threshold <= 0.0f) merge_threshold = 1e-4f;

    /* Single convex: shortcut */
    const ivp_compact_ledge_t *single = ivp_surface_get_single_convex(surface_in);
    if (single) {
        return ivp_compact_modify_shrink_ledge(alloc, single, shrink_value,
                                                merge_threshold);
    }

    /* Compound: collect all terminal ledges */
    const ivp_compact_ledgetree_node_t *root =
        ivp_surface_get_ledgetree_root(surface_in);
    int n_terminals = ivp_ledgetree_count_terminal_ledges(root);
    if (n_terminals < 1) return NULL;

    const ivp_compact_ledge_t **terminal_ledges =
        (const ivp_compact_ledge_t **)ivp_alloc(alloc,
            (size_t)n_terminals * sizeof(ivp_compact_ledge_t *));
    if (!terminal_ledges) return NULL;

    collect_ctx_t ctx;
    ctx.ledges = terminal_ledges;
    ctx.count = 0;
    ctx.capacity = n_terminals;
    ivp_ledgetree_visit_all(root, collect_visitor, &ctx);

    /* Shrink each terminal ledge individually and collect results */
    ivp_compact_surface_t **shrunk_parts =
        (ivp_compact_surface_t **)ivp_alloc(alloc,
            (size_t)ctx.count * sizeof(ivp_compact_surface_t *));
    if (!shrunk_parts) {
        ivp_free(alloc, (void *)terminal_ledges,
                 (size_t)n_terminals * sizeof(ivp_compact_ledge_t *));
        return NULL;
    }

    int n_good = 0;
    for (int i = 0; i < ctx.count; i++) {
        shrunk_parts[i] = ivp_compact_modify_shrink_ledge(
            alloc, terminal_ledges[i], shrink_value, merge_threshold);
        if (shrunk_parts[i]) n_good++;
    }

    ivp_free(alloc, (void *)terminal_ledges,
             (size_t)n_terminals * sizeof(ivp_compact_ledge_t *));

    if (n_good < 1) {
        for (int i = 0; i < ctx.count; i++) {
            if (shrunk_parts[i])
                ivp_compact_surface_free(alloc, shrunk_parts[i]);
        }
        ivp_free(alloc, shrunk_parts,
                 (size_t)ctx.count * sizeof(ivp_compact_surface_t *));
        return NULL;
    }

    /* Compile shrunk parts into a new compound surface via ledge soup */
    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, alloc);
    int add_failed = 0;

    for (int i = 0; i < ctx.count; i++) {
        if (!shrunk_parts[i]) continue;
        const ivp_compact_ledge_t *ledge =
            ivp_surface_get_single_convex(shrunk_parts[i]);
        if (ledge) {
            if (ivp_ledge_soup_add(&soup, ledge) != 0) {
                add_failed = 1;
                break;
            }
        }
    }

    ivp_compact_surface_t *result = add_failed ? NULL : ivp_ledge_soup_compile(&soup);
    ivp_ledge_soup_destroy(&soup);

    /* Free intermediate shrunk surfaces */
    for (int i = 0; i < ctx.count; i++) {
        if (shrunk_parts[i])
            ivp_compact_surface_free(alloc, shrunk_parts[i]);
    }
    ivp_free(alloc, shrunk_parts,
             (size_t)ctx.count * sizeof(ivp_compact_surface_t *));

    return result;
}
