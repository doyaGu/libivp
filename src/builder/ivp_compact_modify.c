/* ivp_compact_modify.c — IVP_Compact_Modify (ivp_compact_modify.cxx)
 *
 * Chop and shrink operations on compact surfaces, through the reference
 * IVP_Halfspacesoup (planes of a convex ledge, inward normals, near-identical
 * planes merged) and the half-space soup builder.
 */
#include <ivp/builder/ivp_compact_modify.h>
#include <ivp/builder/ivp_build_halfspacesoup.h>
#include "ivp_build_halfspacesoup_internal.h"
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/base/ivp_assert.h>
#include <math.h>
#include <string.h>

/* ── IVP_Halfspacesoup (ivp_halfspacesoup.cxx) ──────────────────────────── */

/* HALFSPACESOUP_TOLERANCE (0.01f * P_MIN_EDGE_LEN) */
#define CM_HALFSPACESOUP_TOLERANCE ((double)(0.01f * 0.01f))
#define CM_MAX_PLANES 512

/* libivp plane {n, dist} == reference IVP_U_Hesse(n, hesse_val = -dist) */
typedef struct cm_hesse {
    double n[3];
    double hesse_val;
} cm_hesse_t;

typedef struct cm_soup {
    cm_hesse_t planes[CM_MAX_PLANES];
    int        n;
} cm_soup_t;

/* IVP_Halfspacesoup::add_halfspace */
static void cm_add_halfspace(cm_soup_t *soup, const cm_hesse_t *plane)
{
    bool to_delete[CM_MAX_PLANES];
    for (int i = 0; i < soup->n; i++) {
        const cm_hesse_t *old = &soup->planes[i];
        double dp = plane->n[0] * old->n[0] + plane->n[1] * old->n[1] + plane->n[2] * old->n[2];
        to_delete[i] = false;
        if (dp > 1.0 - CM_HALFSPACESOUP_TOLERANCE) {
            if (old->hesse_val < plane->hesse_val) return; /* drop the new, further plane */
            to_delete[i] = true;
        }
    }
    if (soup->n >= CM_MAX_PLANES) return;
    to_delete[soup->n] = false;
    soup->planes[soup->n++] = *plane;
    int w = 0;
    for (int i = 0; i < soup->n; i++) {
        if (!to_delete[i]) soup->planes[w++] = soup->planes[i];
    }
    soup->n = w;
}

/* IVP_Halfspacesoup::IVP_Halfspacesoup(const IVP_Compact_Ledge *) */
static void cm_soup_from_ledge(cm_soup_t *soup, const ivp_compact_ledge_t *ledge)
{
    soup->n = 0;
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
    int n_tri = ivp_ledge_get_n_triangles(ledge);
    for (int i = 0; i < n_tri; i++, tri++) {
        const ivp_compact_edge_t *edge = &tri->edges[0];
        const float *p0 = ivp_edge_get_start_point(edge, ledge)->k;
        const float *p1 = ivp_edge_get_start_point(ivp_edge_get_next(edge), ledge)->k;
        const float *p2 = ivp_edge_get_start_point(ivp_edge_get_prev(edge), ledge)->k;
        /* calc_hesse_vec_object_not_normized:
         * inline_set_vert_to_area_defined_by_three_points(p0, p2, p1) */
        double a0 = (double)p2[0] - (double)p0[0], a1 = (double)p2[1] - (double)p0[1], a2 = (double)p2[2] - (double)p0[2];
        double b0 = (double)p1[0] - (double)p0[0], b1 = (double)p1[1] - (double)p0[1], b2 = (double)p1[2] - (double)p0[2];
        cm_hesse_t h;
        h.n[0] = b1 * a2 - a1 * b2;
        h.n[1] = b2 * a0 - a2 * b0;
        h.n[2] = b0 * a1 - a0 * b1;
        h.hesse_val = 0.0;
        /* normize(): 4 component hesse normalised by its vector length */
        double len = sqrt(h.n[0] * h.n[0] + h.n[1] * h.n[1] + h.n[2] * h.n[2]);
        if (len < 1e-30) continue;
        double inv = 1.0 / len;
        for (int k = 0; k < 3; k++) h.n[k] = -(h.n[k] * inv); /* mult(-1): inward */
        /* calc_hesse_val(start point) */
        h.hesse_val = -(h.n[0] * (double)p0[0] + h.n[1] * (double)p0[1] + h.n[2] * (double)p0[2]);
        cm_add_halfspace(soup, &h);
    }
}

static void cm_soup_to_planes(const cm_soup_t *soup, ivp_halfspace_t *planes)
{
    for (int i = 0; i < soup->n; i++) {
        for (int k = 0; k < 3; k++) planes[i].normal[k] = (float)soup->planes[i].n[k];
        planes[i].dist = (float)(-soup->planes[i].hesse_val);
    }
}

/* IVP_SurfaceBuilder_Halfspacesoup::convert_halfspacesoup_to_compact_surface */
static ivp_compact_surface_t *cm_soup_to_surface(const ivp_allocator_t *alloc, const cm_soup_t *soup, float merge)
{
    ivp_halfspace_t planes[CM_MAX_PLANES];
    cm_soup_to_planes(soup, planes);
    return ivp_build_halfspacesoup_surface(alloc, planes, soup->n, merge);
}

/* IVP_SurfaceBuilder_Halfspacesoup::convert_halfspacesoup_to_compact_ledge,
 * wrapped into a surface by the ledge soup */
static ivp_compact_surface_t *cm_soup_to_ledge_surface(const ivp_allocator_t *alloc, const cm_soup_t *soup,
                                                       float merge)
{
    ivp_halfspace_t planes[CM_MAX_PLANES];
    cm_soup_to_planes(soup, planes);
    ivp_compact_ledge_t *ledge = ivp_hs_halfspacesoup_to_ledge(alloc, planes, soup->n, merge);
    if (!ledge) return NULL;
    ivp_compact_surface_t *surface = ivp_build_surface_from_ledge(alloc, ledge);
    ivp_compact_ledge_free(alloc, ledge);
    return surface;
}

/* ── Chop (IVP_Compact_Modify::chop) ──────────────────────────────────────── */

ivp_compact_surface_t *ivp_compact_modify_chop(
    const ivp_allocator_t *alloc,
    const ivp_compact_surface_t *surface_in,
    const float chop_normal[3],
    float chop_depth)
{
    if (!alloc) alloc = ivp_default_allocator();
    if (!surface_in || !chop_normal) return NULL;

    /* currently, we just want to chop objects that consist of just ONE convex ledge */
    const ivp_compact_ledge_t *the_ledge = ivp_surface_get_single_convex(surface_in);
    IVP_ASSERT(ivp_ledgetree_count_terminal_ledges(ivp_surface_get_ledgetree_root(surface_in)) == 1); /* ledges.len() == 1 */
    if (!the_ledge) return NULL;

    cm_soup_t *soup = (cm_soup_t *)ivp_alloc(alloc, sizeof(cm_soup_t));
    if (!soup) return NULL;
    cm_soup_from_ledge(soup, the_ledge);

    const double merge_eps = (double)0.01f;

    /* chop_vector_normized (IVP_U_Point::normize) */
    double v[3] = { (double)chop_normal[0], (double)chop_normal[1], (double)chop_normal[2] };
    double vl = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (vl < 1e-30) {
        ivp_free(alloc, soup, sizeof(cm_soup_t));
        return NULL;
    }
    for (int k = 0; k < 3; k++) v[k] /= vl;

    /* find out nearest point to a distant plane */
    double far = (double)-1e6f;
    double test_hesse = -(v[0] * (v[0] * far) + v[1] * (v[1] * far) + v[2] * (v[2] * far));
    double min_d = -1.0;
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(the_ledge);
    for (int i = 0; i < ivp_ledge_get_n_triangles(the_ledge); i++, tri++) {
        for (int e = 0; e < 3; e++) {
            const float *sp = ivp_edge_get_start_point(&tri->edges[e], the_ledge)->k;
            double dist = v[0] * (double)sp[0] + v[1] * (double)sp[1] + v[2] * (double)sp[2] + test_hesse;
            IVP_ASSERT(dist > 0.0); /* otherwise we are not far enough away */
            if (dist < min_d || min_d < 0.0) min_d = dist;
        }
    }
    double off = far + min_d + (double)chop_depth;

    cm_hesse_t chop_halfspace;
    for (int k = 0; k < 3; k++) chop_halfspace.n[k] = v[k];
    chop_halfspace.hesse_val = -(v[0] * (v[0] * off) + v[1] * (v[1] * off) + v[2] * (v[2] * off));
    cm_add_halfspace(soup, &chop_halfspace);

    ivp_compact_surface_t *result = cm_soup_to_surface(alloc, soup, (float)merge_eps);
    ivp_free(alloc, soup, sizeof(cm_soup_t));
    return result;
}

/* ── Shrink ledge (IVP_Compact_Modify::shrink(ledge)) ─────────────────────── */

ivp_compact_surface_t *ivp_compact_modify_shrink_ledge(
    const ivp_allocator_t *alloc,
    const ivp_compact_ledge_t *ledge_in,
    float shrink_value,
    float merge_threshold)
{
    if (!alloc) alloc = ivp_default_allocator();
    if (!ledge_in) return NULL;
    if (merge_threshold <= 0.0f) merge_threshold = 1e-4f;

    cm_soup_t *soup = (cm_soup_t *)ivp_alloc(alloc, sizeof(cm_soup_t));
    if (!soup) return NULL;
    cm_soup_from_ledge(soup, ledge_in);
    for (int i = soup->n - 1; i >= 0; i--) soup->planes[i].hesse_val -= (double)shrink_value;
    ivp_compact_surface_t *result = cm_soup_to_ledge_surface(alloc, soup, merge_threshold);
    ivp_free(alloc, soup, sizeof(cm_soup_t));
    return result;
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
    if (!surface_in) return NULL;
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
    /* reference: for (l = ledges.len()-1; l >= 0; l--) */
    for (int i = ctx.count - 1; i >= 0; i--) {
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

    for (int i = ctx.count - 1; i >= 0; i--) {
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
