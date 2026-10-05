/* test_geometry_parity.c — Geometry builders / surface managers vs. the
 * reference IVP engine.
 *
 * Every expected number below was printed by the legacy C++ engine
 * (../ivp, IVP_SurfaceBuilder_Pointsoup /
 * IVP_SurfaceBuilder_Ledge_Soup / IVP_SurfaceBuilder_Halfspacesoup, see the
 * report for the dump program) and is compared with a relative tolerance of
 * 1e-5 (absolute 1e-6 for values near zero).
 */
#include "ivp_test.h"

#include <ivp/builder/ivp_build_surface.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/builder/ivp_build_pointsoup.h>
#include <ivp/builder/ivp_build_polygon.h>
#include <ivp/builder/ivp_build_halfspacesoup.h>
#include <ivp/builder/ivp_build_compact_recursive.h>
#include <ivp/builder/ivp_rot_inertia.h>
#include <ivp/geometry/ivp_compact_ledge_geom.h>
#include <ivp/geometry/ivp_surman.h>

#include <limits.h>
#include <stdint.h>
#include <string.h>

/* ── Helpers ───────────────────────────────────────────────────────────── */

static int near_rel(double a, double b)
{
    double d = fabs(a - b);
    double m = fabs(b) > 1.0 ? fabs(b) : 1.0;
    return d <= 1e-5 * m || d <= 1e-6;
}

typedef struct ref_header {
    double mc[3];
    double ri[3];
    double r;
    int dev;
    int size;
} ref_header_t;

static int header_matches(const ivp_compact_surface_t *s, const ref_header_t *h)
{
    if (!s) return 0;
    for (int i = 0; i < 3; i++) {
        if (!near_rel((double)s->mass_center[i], h->mc[i])) return 0;
        if (!near_rel((double)s->rotation_inertia[i], h->ri[i])) return 0;
    }
    if (!near_rel((double)s->upper_limit_radius, h->r)) return 0;
    if (ivp_surface_get_max_deviation(s) != h->dev) return 0;
    if (h->size && ivp_surface_get_byte_size(s) != h->size) return 0;
    return 1;
}

static void box_coords(double *c, double hx, double hy, double hz, double ox, double oy, double oz)
{
    int k = 0;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2) {
                c[k++] = sx * hx + ox;
                c[k++] = sy * hy + oy;
                c[k++] = sz * hz + oz;
            }
}

static ivp_compact_ledge_t *box_ledge(double hx, double hy, double hz, double ox, double oy, double oz)
{
    double c[24];
    box_coords(c, hx, hy, hz, ox, oy, oz);
    return ivp_build_convex_ledge_d(NULL, c, 8);
}

/* soup3 of the reference dump */
static ivp_compact_surface_t *make_soup3(bool root_hull, ivp_ledge_soup_merge_points_t merge)
{
    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);
    ivp_compact_ledge_t *l[3] = { box_ledge(0.5, 0.5, 0.5, 0, 0, 0),
                                  box_ledge(0.5, 0.25, 0.5, 2, 0, 0),
                                  box_ledge(0.25, 1.0, 0.5, 0, 3, 1) };
    for (int i = 0; i < 3; i++) ivp_ledge_soup_add(&soup, l[i]);
    ivp_ledge_soup_template_t t;
    ivp_ledge_soup_template_init(&t);
    t.build_root_convex_hull = root_hull;
    t.merge_points = merge;
    ivp_compact_surface_t *s = ivp_ledge_soup_compile_ex(&soup, &t);
    ivp_ledge_soup_destroy(&soup);
    for (int i = 0; i < 3; i++) ivp_compact_ledge_free(NULL, l[i]);
    return s;
}

/* ── Box (pointsoup path) ──────────────────────────────────────────────── */

IVP_TEST(box_header_matches_reference)
{
    /* legacy: build_box_compact_surface(hx,hy,hz) */
    static const struct { float h[3]; ref_header_t ref; } cases[] = {
        { {0.5f, 0.5f, 0.5f}, {{0, 0, 0}, {0.117851131, 0.117851131, 0.117851131}, 0.866025388, 205, 412} },
        { {1.6f, 0.35f, 0.8f}, {{0, 0, 0}, {0.217206076, 0.879595935, 0.854309797}, 1.82277262, 246, 412} },
        { {5.0f, 0.25f, 2.5f}, {{0, 0, 0}, {2.08343744, 8.5898037, 8.33335972}, 5.59575748, 250, 412} },
        { {20.0f, 0.5f, 20.0f}, {{0, 0, 0}, {133.333359, 188.561813, 133.333359}, 28.2886906, 250, 412} },
        { {0.3f, 6.0f, 15.0f}, {{0, 0, 0}, {75.9539337, 75.0000076, 12.0000381}, 16.1582794, 250, 412} },
        { {1.0f, 1.0f, 1.0f}, {{0, 0, 0}, {0.471404523, 0.471404523, 0.471404523}, 1.73205078, 205, 412} },
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        ivp_compact_surface_t *s = ivp_build_box_surface(NULL, cases[i].h[0], cases[i].h[1], cases[i].h[2]);
        IVP_ASSERT_TRUE(header_matches(s, &cases[i].ref));
        IVP_ASSERT_TRUE(ivp_compact_surface_check(s, 0));
        /* reference ledgetree node: bbox center, half diagonal, box sizes */
        const ivp_compact_ledgetree_node_t *n = ivp_surface_get_ledgetree_root(s);
        IVP_ASSERT_TRUE(ivp_ledgetree_is_terminal(n));
        IVP_ASSERT_TRUE(near_rel((double)n->radius, cases[i].ref.r));
        ivp_compact_surface_free(NULL, s);

        /* the generic convex builder must agree */
        double c[24];
        box_coords(c, cases[i].h[0], cases[i].h[1], cases[i].h[2], 0, 0, 0);
        s = ivp_build_convex_surface_d(NULL, c, 8);
        IVP_ASSERT_TRUE(header_matches(s, &cases[i].ref));
        ivp_compact_surface_free(NULL, s);
    }
}

IVP_TEST(box_node_box_sizes_match_reference)
{
    ivp_compact_surface_t *s = ivp_build_box_surface(NULL, 1.6f, 0.35f, 0.8f);
    const ivp_compact_ledgetree_node_t *n = ivp_surface_get_ledgetree_root(s);
    IVP_ASSERT_EQ(n->box_sizes[0], 220);
    IVP_ASSERT_EQ(n->box_sizes[1], 49);
    IVP_ASSERT_EQ(n->box_sizes[2], 110);
    ivp_compact_surface_free(NULL, s);
}

IVP_TEST(box_pierce_info)
{
    /* insert_pierce_info: every triangle is paired with an opposite facing one */
    ivp_compact_surface_t *s = ivp_build_box_surface(NULL, 1.0f, 2.0f, 3.0f);
    const ivp_compact_ledge_t *l = ivp_surface_get_single_convex(s);
    const ivp_compact_triangle_t *t = ivp_ledge_get_first_triangle(l);
    for (int i = 0; i < 12; i++) {
        int p = ivp_tri_get_pierce_index(&t[i]);
        IVP_ASSERT_TRUE(p >= 0 && p < 12);
        double h0[3], h1[3];
        ivp_ledge_calc_hesse_vec(l, &t[i].edges[0], h0);
        ivp_ledge_calc_hesse_vec(l, &t[p].edges[0], h1);
        double d = h0[0] * h1[0] + h0[1] * h1[1] + h0[2] * h1[2];
        IVP_ASSERT_TRUE(d < 0.0);
    }
    /* terminal ledge client data is untouched (reference union) */
    IVP_ASSERT_EQ(ivp_ledge_get_client_data(l), 0);
    ivp_compact_surface_free(NULL, s);
}

/* ── Convex pointsoup ──────────────────────────────────────────────────── */

IVP_TEST(pointsoup_headers_match_reference)
{
    {   /* off center box: never re-centered */
        double c[24];
        box_coords(c, 0.5, 1.0, 1.5, 3.0, -2.0, 1.0);
        ref_header_t r = {{3, -2, 1}, {0.820738196, 0.754615426, 0.343592137}, 1.87082875, 241, 412};
        ivp_compact_surface_t *s = ivp_build_convex_surface_d(NULL, c, 8);
        IVP_ASSERT_TRUE(header_matches(s, &r));
        const ivp_compact_ledgetree_node_t *n = ivp_surface_get_ledgetree_root(s);
        IVP_ASSERT_NEAR(n->center[0], 3.0, 1e-6);
        ivp_compact_surface_free(NULL, s);
    }
    {   /* cloud with interior points */
        const double c[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0.2, 0.2, 0.2,
                            0.9, 0.1, 0.4, -0.3, 0.5, 0.6, 0.4, -0.2, 0.8, 0.6, 0.7, -0.1};
        ref_header_t r = {{0.404405743, 0.412500024, 0.436782807},
                          {0.099027209, 0.103858024, 0.106560037}, 1.00851488, 225, 460};
        ivp_compact_surface_t *s = ivp_build_convex_surface_d(NULL, c, 10);
        IVP_ASSERT_TRUE(header_matches(s, &r));
        IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(ivp_surface_get_single_convex(s)), 14);
        ivp_compact_surface_free(NULL, s);
    }
    {
        const double c[] = {0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 4};
        ref_header_t r = {{0.5, 0.75, 1}, {0.688408673, 0.61846602, 0.369332254}, 3.13249111, 249, 220};
        ivp_compact_surface_t *s = ivp_build_convex_surface_d(NULL, c, 4);
        IVP_ASSERT_TRUE(header_matches(s, &r));
        /* node sphere is the bounding box sphere, not the mass sphere */
        const ivp_compact_ledgetree_node_t *n = ivp_surface_get_ledgetree_root(s);
        IVP_ASSERT_NEAR(n->radius, 2.69258237, 1e-5);
        ivp_compact_surface_free(NULL, s);
    }
    {
        const double c[] = {1, 0, 0, -1, 0, 0, 0, 2, 0, 0, -2, 0, 0, 0, 3, 0, 0, -3};
        ref_header_t r = {{0, 0, 0}, {0.98488611, 0.905538857, 0.412310719}, 3, 240, 316};
        ivp_compact_surface_t *s = ivp_build_convex_surface_d(NULL, c, 6);
        IVP_ASSERT_TRUE(header_matches(s, &r));
        ivp_compact_surface_free(NULL, s);
    }
    {   /* 12-gon prism: coplanar caps merged into polygons, 44 triangles */
        double c[24 * 3];
        int k = 0;
        for (int i = 0; i < 12; i++) {
            double a = i * 3.14159265358979323846 / 6.0;
            c[k++] = 0.7 * cos(a); c[k++] = -0.4; c[k++] = 0.7 * sin(a);
            c[k++] = 0.7 * cos(a); c[k++] = 0.4;  c[k++] = 0.7 * sin(a);
        }
        ref_header_t r = {{0, 0, 0}, {0.12860918, 0.165504545, 0.12860918}, 0.806225777, 218, 1180};
        ivp_compact_surface_t *s = ivp_build_convex_surface_d(NULL, c, 24);
        IVP_ASSERT_TRUE(header_matches(s, &r));
        IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(ivp_surface_get_single_convex(s)), 44);
        ivp_compact_surface_free(NULL, s);
    }
}

IVP_TEST(pointsoup_options_default_no_recentering)
{
    float pts[24];
    double c[24];
    box_coords(c, 0.5, 1.0, 1.5, 3.0, -2.0, 1.0);
    for (int i = 0; i < 24; i++) pts[i] = (float)c[i];
    ivp_pointsoup_options_t o = ivp_pointsoup_options_default();
    IVP_ASSERT_TRUE(!o.center_points);
    ivp_compact_surface_t *s = ivp_build_pointsoup_surface(NULL, pts, 8, NULL);
    IVP_ASSERT_NOT_NULL(s);
    IVP_ASSERT_NEAR(s->mass_center[0], 3.0, 1e-6);
    /* geometry and mass center share the input frame */
    const ivp_compact_ledge_t *l = ivp_surface_get_single_convex(s);
    double mn[3], mx[3];
    ivp_ledge_calc_bounding_box_d(l, mn, mx);
    IVP_ASSERT_NEAR(mn[0], 2.5, 1e-6);
    IVP_ASSERT_NEAR(mx[0], 3.5, 1e-6);
    ivp_compact_surface_free(NULL, s);

    /* explicit centering: geometry AND mass center in the shifted frame */
    o.center_points = true;
    s = ivp_build_pointsoup_surface(NULL, pts, 8, &o);
    IVP_ASSERT_NEAR(s->mass_center[0], 0.0, 1e-6);
    ivp_ledge_calc_bounding_box_d(ivp_surface_get_single_convex(s), mn, mx);
    IVP_ASSERT_NEAR(mn[0], -0.5, 1e-6);
    ivp_compact_surface_free(NULL, s);
}

IVP_TEST(pointsoup_coplanar_input_gives_two_sided_triangle_or_null)
{
    /* 4 coplanar points: qhull fails, joggle retries, the degenerate facets
     * are reduced; the reference ends with a 3 point triangle ledge or
     * NULL -- never an invalid ledge */
    const double c[] = {0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0};
    ivp_compact_ledge_t *l = ivp_build_convex_ledge_d(NULL, c, 4);
    if (l) {
        IVP_ASSERT_TRUE(ivp_compact_ledge_check(l));
        ivp_compact_ledge_free(NULL, l);
    }
}

/* ── Halfspace soup ────────────────────────────────────────────────────── */

IVP_TEST(halfspacesoup_cube_matches_reference)
{
    ivp_halfspace_t hs[6] = {
        {{-1, 0, 0}, -0.5f}, {{1, 0, 0}, -0.5f}, {{0, -1, 0}, -0.5f},
        {{0, 1, 0}, -0.5f}, {{0, 0, -1}, -0.5f}, {{0, 0, 1}, -0.5f}};
    ref_header_t r = {{0, 0, 0}, {0.117851131, 0.117851131, 0.117851131}, 0.866025388, 205, 412};
    ivp_compact_surface_t *s = ivp_build_halfspacesoup_surface(NULL, hs, 6, 0.001f);
    IVP_ASSERT_TRUE(header_matches(s, &r));
    ivp_compact_surface_free(NULL, s);
}

IVP_TEST(halfspacesoup_tolerance_is_1e4)
{
    /* HALFSPACESOUP_TOLERANCE = 0.01 * P_MIN_EDGE_LEN = 1e-4: a cube corner
     * 5e-4 outside an extra cutting plane is dropped, the cut survives */
    ivp_halfspace_t hs[7] = {
        {{-1, 0, 0}, -0.5f}, {{1, 0, 0}, -0.5f}, {{0, -1, 0}, -0.5f},
        {{0, 1, 0}, -0.5f}, {{0, 0, -1}, -0.5f}, {{0, 0, 1}, -0.5f},
        /* plane x + y + z <= 1.5 - 5e-4*sqrt(3), inward normal */
        {{-0.57735027f, -0.57735027f, -0.57735027f}, -(0.8660254f - 0.0005f)}};
    float *pts = NULL;
    int n = 0;
    IVP_ASSERT_EQ(ivp_halfspacesoup_to_points(hs, 7, 1e-6f, &pts, &n, NULL), IVP_OK);
    /* 7 untouched corners + 3 cut points (the old 1e-3 tolerance also kept
     * the cut-off corner: 11 points) */
    IVP_ASSERT_EQ(n, 10);
    ivp_free(ivp_default_allocator(), pts, (size_t)n * 3u * sizeof(float));
}

IVP_TEST(halfspacesoup_rejects_overflowing_count)
{
    ivp_halfspace_t halfspace = {{1.0f, 0.0f, 0.0f}, 1.0f};
    float *points = (float *)(uintptr_t)1;
    int n_points = -1;
    IVP_ASSERT_EQ(ivp_halfspacesoup_to_points(&halfspace, INT_MAX, 0.01f,
                                              &points, &n_points, NULL),
                  IVP_ERR_OUT_OF_MEMORY);
    IVP_ASSERT_NULL(points);
    IVP_ASSERT_EQ(n_points, 0);
}

/* ── Ledge soup ────────────────────────────────────────────────────────── */

typedef struct node_ref {
    int depth;
    double c[3];
    double r;
    int bs[3];
    int term;
} node_ref_t;

static int collect_nodes(const ivp_compact_ledgetree_node_t *n, int depth, node_ref_t *out, int pos)
{
    out[pos].depth = depth;
    for (int i = 0; i < 3; i++) {
        out[pos].c[i] = (double)n->center[i];
        out[pos].bs[i] = n->box_sizes[i];
    }
    out[pos].r = (double)n->radius;
    out[pos].term = ivp_ledgetree_is_terminal(n);
    pos++;
    if (!ivp_ledgetree_is_terminal(n)) {
        pos = collect_nodes(ivp_ledgetree_left_son(n), depth + 1, out, pos);
        pos = collect_nodes(ivp_ledgetree_right_son(n), depth + 1, out, pos);
    }
    return pos;
}

static int nodes_match(const ivp_compact_surface_t *s, const node_ref_t *ref, int n_ref)
{
    node_ref_t got[64];
    int n = collect_nodes(ivp_surface_get_ledgetree_root(s), 0, got, 0);
    if (n != n_ref) return 0;
    for (int i = 0; i < n; i++) {
        if (got[i].depth != ref[i].depth || got[i].term != ref[i].term) return 0;
        for (int k = 0; k < 3; k++) {
            if (!near_rel(got[i].c[k], ref[i].c[k])) return 0;
            if (got[i].bs[k] != ref[i].bs[k]) return 0;
        }
        if (!near_rel(got[i].r, ref[i].r)) return 0;
    }
    return 1;
}

IVP_TEST(ledge_soup_soup3_matches_reference)
{
    static const ref_header_t r = {{0.400000006, 1.20000005, 0.400000006},
                                   {2.35315299, 0.769554317, 2.43319821}, 3.07774258, 245, 1196};
    static const node_ref_t nodes[] = {
        {0, {0.999352574, 1.75064456, 0.500894308}, 2.88739824, {131, 196, 87}, 0},
        {1, {0.999352574, 0, 0}, 1.66118777, {226, 76, 76}, 0},
        {2, {0, 0, 0}, 0.866025388, {145, 145, 145}, 1},
        {2, {2, 0, 0}, 0.75, {167, 84, 167}, 1},
        {1, {0, 3, 1}, 1.14564395, {55, 219, 110}, 1},
    };
    ivp_compact_surface_t *s = make_soup3(false, IVP_SLMP_NO_MERGE);
    IVP_ASSERT_TRUE(header_matches(s, &r));
    IVP_ASSERT_TRUE(nodes_match(s, nodes, 5));
    IVP_ASSERT_TRUE(ivp_compact_surface_check(s, 0));
    ivp_compact_surface_free(NULL, s);
}

IVP_TEST(ledge_soup_median_ties_match_reference)
{
    /* 2x2x2 grid of boxes: every split has spheres on the median */
    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);
    ivp_compact_ledge_t *l[8];
    for (int i = 0; i < 8; i++) {
        l[i] = box_ledge(0.4, 0.4, 0.4, (i & 1) * 1.0, ((i >> 1) & 1) * 1.0, ((i >> 2) & 1) * 1.0);
        ivp_ledge_soup_add(&soup, l[i]);
    }
    ivp_compact_surface_t *s = ivp_ledge_soup_compile(&soup);
    ivp_ledge_soup_destroy(&soup);
    for (int i = 0; i < 8; i++) ivp_compact_ledge_free(NULL, l[i]);

    static const ref_header_t r = {{0.499999464, 0.499999315, 0.499999404},
                                   {0.428978115, 0.428978115, 0.428978115}, 1.55884671, 205, 3156};
    static const node_ref_t nodes[] = {
        {0, {0.5, 0.5, 0.5}, 1.56202543, {145, 145, 145}, 0},
        {1, {0, 0.5, 0.5}, 1.33719397, {76, 169, 169}, 0},
        {2, {0, 0, 0.5}, 1.06595111, {95, 95, 212}, 0},
        {3, {0, 0, 0}, 0.692820311, {145, 145, 145}, 1},
        {3, {0, 0, 1}, 0.692820311, {145, 145, 145}, 1},
        {2, {0, 1, 0.5}, 1.06595111, {95, 95, 212}, 0},
        {3, {0, 1, 0}, 0.692820311, {145, 145, 145}, 1},
        {3, {0, 1, 1}, 0.692820311, {145, 145, 145}, 1},
        {1, {1, 0.5, 0.5}, 1.33719397, {76, 169, 169}, 0},
        {2, {1, 0, 0.5}, 1.06595111, {95, 95, 212}, 0},
        {3, {1, 0, 0}, 0.692820311, {145, 145, 145}, 1},
        {3, {1, 0, 1}, 0.692820311, {145, 145, 145}, 1},
        {2, {1, 1, 0.5}, 1.06595111, {95, 95, 212}, 0},
        {3, {1, 1, 0}, 0.692820311, {145, 145, 145}, 1},
        {3, {1, 1, 1}, 0.692820311, {145, 145, 145}, 1},
    };
    IVP_ASSERT_TRUE(header_matches(s, &r));
    IVP_ASSERT_TRUE(nodes_match(s, nodes, 15));
    ivp_compact_surface_free(NULL, s);
}

IVP_TEST(ledge_soup_root_hull_matches_reference)
{
    ivp_compact_surface_t *s = make_soup3(true, IVP_SLMP_NO_MERGE);
    static const ref_header_t r = {{0.400000006, 1.20000005, 0.400000006},
                                   {2.35315299, 0.769554317, 2.43319821}, 3.07774258, 245, 1964};
    IVP_ASSERT_TRUE(header_matches(s, &r));
    const ivp_compact_ledgetree_node_t *root = ivp_surface_get_ledgetree_root(s);
    const ivp_compact_ledge_t *hull = ivp_ledgetree_get_compact_hull(root);
    IVP_ASSERT_NOT_NULL(hull);
    IVP_ASSERT_FALSE(ivp_ledge_is_terminal(hull));
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(hull), 30);
    IVP_ASSERT_EQ(ivp_ledge_get_n_points(hull), 17);
    /* the hull points back to its node */
    IVP_ASSERT_TRUE((const char *)hull + hull->node_or_client == (const char *)root);
    /* some hull triangles bridge the parts and are virtual */
    int n_virtual = 0;
    for (int i = 0; i < 30; i++) {
        n_virtual += ivp_tri_get_is_virtual(&ivp_ledge_get_first_triangle(hull)[i]);
    }
    IVP_ASSERT_TRUE(n_virtual > 0 && n_virtual < 30);
    IVP_ASSERT_TRUE(ivp_compact_surface_check(s, 0));

    /* surface manager: get_single_convex returns the root hull */
    ivp_surman_polygon_t sm;
    ivp_surman_polygon_init(&sm, s);
    IVP_ASSERT_TRUE(ivp_surman_get_single_convex(&sm.base) == hull);
    ivp_compact_surface_free(NULL, s);

    /* compound without hull: NULL */
    s = make_soup3(false, IVP_SLMP_NO_MERGE);
    ivp_surman_polygon_init(&sm, s);
    IVP_ASSERT_NULL(ivp_surman_get_single_convex(&sm.base));
    ivp_compact_surface_free(NULL, s);
}

IVP_TEST(ledge_soup_merge_points)
{
    ivp_compact_surface_t *plain = make_soup3(false, IVP_SLMP_NO_MERGE);
    ivp_compact_surface_t *merged = make_soup3(false, IVP_SLMP_MERGE_AND_REALLOCATE);
    ivp_compact_surface_t *merged_nr = make_soup3(true, IVP_SLMP_MERGE_NO_REALLOCATE);
    IVP_ASSERT_NOT_NULL(merged);
    IVP_ASSERT_NOT_NULL(merged_nr);
    /* same physics, shared point array: 24 points instead of 3*8 copies
     * -> identical, the boxes share no corners */
    for (int i = 0; i < 3; i++) {
        IVP_ASSERT_NEAR(merged->mass_center[i], plain->mass_center[i], 1e-6);
        IVP_ASSERT_NEAR(merged->rotation_inertia[i], plain->rotation_inertia[i], 1e-5);
    }
    IVP_ASSERT_EQ(ivp_surface_get_byte_size(merged), ivp_surface_get_byte_size(plain));
    IVP_ASSERT_TRUE(ivp_compact_surface_check(merged, 0));
    IVP_ASSERT_TRUE(ivp_compact_surface_check(merged_nr, 0));
    ivp_compact_surface_free(NULL, plain);
    ivp_compact_surface_free(NULL, merged);
    ivp_compact_surface_free(NULL, merged_nr);

    /* two boxes sharing a face: 4 shared corners are stored once */
    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);
    ivp_compact_ledge_t *a = box_ledge(0.5, 0.5, 0.5, 0, 0, 0);
    ivp_compact_ledge_t *b = box_ledge(0.5, 0.5, 0.5, 1, 0, 0);
    ivp_ledge_soup_add(&soup, a);
    ivp_ledge_soup_add(&soup, b);
    ivp_ledge_soup_template_t t;
    ivp_ledge_soup_template_init(&t);
    t.merge_points = IVP_SLMP_MERGE_AND_REALLOCATE;
    t.free_input_compact_ledges = true;
    ivp_compact_surface_t *s = ivp_ledge_soup_compile_ex(&soup, &t);
    ivp_ledge_soup_destroy(&soup);
    IVP_ASSERT_NOT_NULL(s);
    /* 48 + 2*(16+12*16) + 12*16 points + 3*28 nodes */
    IVP_ASSERT_EQ(ivp_surface_get_byte_size(s), 48 + 2 * (16 + 192) + 12 * 16 + 3 * 28);
    IVP_ASSERT_TRUE(ivp_compact_surface_check(s, 0));
    ivp_compact_surface_free(NULL, s);
}

IVP_TEST(ledge_soup_single_and_triangle)
{
    /* 3 point two sided triangle (legacy: mc=(0.5 0.5 0) ri=0.25 r=0.7071 dev=250 size=172) */
    const double c[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    ivp_compact_ledge_t *l = ivp_build_convex_ledge_d(NULL, c, 3);
    IVP_ASSERT_NOT_NULL(l);
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(l), 2);
    const ivp_compact_triangle_t *t = ivp_ledge_get_first_triangle(l);
    IVP_ASSERT_EQ(ivp_tri_get_pierce_index(&t[0]), 1);
    IVP_ASSERT_EQ(ivp_tri_get_pierce_index(&t[1]), 0);
    IVP_ASSERT_EQ(ivp_edge_get_opposite_index(&t[0].edges[0]), 4);
    IVP_ASSERT_EQ(ivp_edge_get_opposite_index(&t[0].edges[1]), 5);
    IVP_ASSERT_EQ(ivp_edge_get_opposite_index(&t[0].edges[2]), 3);
    ivp_compact_surface_t *s = ivp_build_surface_from_ledge(NULL, l);
    ref_header_t r = {{0.5, 0.5, 0}, {0.25, 0.25, 0.25}, 0.707106769, 250, 172};
    IVP_ASSERT_TRUE(header_matches(s, &r));
    ivp_compact_surface_free(NULL, s);
    ivp_compact_ledge_free(NULL, l);
}

/* ── Polygon builders ──────────────────────────────────────────────────── */

IVP_TEST(polygon_convex_ngon_faces)
{
    /* hexagonal prism given as two 6-gons + 6 quads */
    float v[12 * 3];
    for (int i = 0; i < 6; i++) {
        double a = i * 3.14159265358979323846 / 3.0;
        v[3 * i + 0] = (float)cos(a); v[3 * i + 1] = -0.5f; v[3 * i + 2] = (float)sin(a);
        v[3 * (i + 6) + 0] = (float)cos(a); v[3 * (i + 6) + 1] = 0.5f; v[3 * (i + 6) + 2] = (float)sin(a);
    }
    int idx[12 + 24];
    int sizes[8];
    int k = 0;
    /* bottom (normal -y): CCW seen from below */
    for (int i = 0; i < 6; i++) idx[k++] = i;
    sizes[0] = 6;
    /* top (normal +y) */
    for (int i = 5; i >= 0; i--) idx[k++] = i + 6;
    sizes[1] = 6;
    for (int i = 0; i < 6; i++) {
        int j = (i + 1) % 6;
        idx[k++] = i; idx[k++] = i + 6; idx[k++] = j + 6; idx[k++] = j;
        sizes[2 + i] = 4;
    }
    ivp_compact_surface_t *s = ivp_build_polygon_convex_surface(NULL, v, 12, idx, sizes, 8);
    IVP_ASSERT_NOT_NULL(s);
    const ivp_compact_ledge_t *l = ivp_surface_get_single_convex(s);
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(l), 20);
    IVP_ASSERT_TRUE(ivp_compact_ledge_check(l));

    /* identical mass properties as the qhull path on the same points */
    ivp_vec3f_t pv[12];
    for (int i = 0; i < 12; i++) pv[i] = ivp_vec3f(v[3 * i], v[3 * i + 1], v[3 * i + 2]);
    ivp_compact_surface_t *q = ivp_build_convex_surface(NULL, pv, 12);
    for (int i = 0; i < 3; i++) {
        IVP_ASSERT_NEAR(s->mass_center[i], q->mass_center[i], 1e-6);
        IVP_ASSERT_NEAR(s->rotation_inertia[i], q->rotation_inertia[i], 1e-5);
    }
    IVP_ASSERT_NEAR(s->upper_limit_radius, q->upper_limit_radius, 1e-6);
    IVP_ASSERT_EQ(ivp_surface_get_max_deviation(s), ivp_surface_get_max_deviation(q));
    ivp_compact_surface_free(NULL, s);
    ivp_compact_surface_free(NULL, q);
}

/* ── Rot inertia solver ────────────────────────────────────────────────── */

IVP_TEST(rot_inertia_box_is_reference_value)
{
    ivp_compact_surface_t *s = ivp_build_box_surface(NULL, 0.5f, 1.0f, 1.5f);
    float mc[3], ri[3];
    IVP_ASSERT_EQ(ivp_calc_mass_center_and_rot_inertia(s, mc, ri), IVP_OK);
    /* a=hx^2/3, b=hy^2/3, c=hz^2/3 -> I_x = sqrt(b^2+c^2) */
    double a = 0.25 / 3.0, b = 1.0 / 3.0, c = 2.25 / 3.0;
    IVP_ASSERT_NEAR(ri[0], sqrt(b * b + c * c), 1e-5);
    IVP_ASSERT_NEAR(ri[1], sqrt(a * a + c * c), 1e-5);
    IVP_ASSERT_NEAR(ri[2], sqrt(a * a + b * b), 1e-5);
    ivp_compact_surface_free(NULL, s);
}

/* ── Validation / endian conversion ────────────────────────────────────── */

IVP_TEST(check_ledge_detects_corruption)
{
    ivp_compact_surface_t *s = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_TRUE(ivp_compact_surface_check(s, 0));
    ivp_compact_ledge_t *l = (ivp_compact_ledge_t *)ivp_surface_get_single_convex(s);
    ivp_compact_triangle_t *t = ivp_ledge_get_first_triangle_mut(l);

    /* broken opposite link */
    uint32_t saved = t[0].edges[0].packed;
    ivp_edge_set_opposite_index(&t[0].edges[0], 1);
    IVP_ASSERT_FALSE(ivp_compact_ledge_check(l));
    IVP_ASSERT_FALSE(ivp_compact_surface_check(s, 0));
    t[0].edges[0].packed = saved;

    /* non convex: move a vertex inside */
    ivp_compact_poly_point_t *pa = ivp_ledge_get_point_array_mut(l);
    float sx = pa[0].k[0];
    pa[0].k[0] = 0.5f;
    IVP_ASSERT_FALSE(ivp_compact_ledge_check(l));
    pa[0].k[0] = sx;
    IVP_ASSERT_TRUE(ivp_compact_ledge_check(l));

    /* root offset out of range */
    int32_t root = s->offset_ledgetree_root;
    s->offset_ledgetree_root = 100000;
    IVP_ASSERT_FALSE(ivp_compact_surface_check(s, 0));
    s->offset_ledgetree_root = root;
    IVP_ASSERT_TRUE(ivp_compact_surface_check(s, 0));
    ivp_compact_surface_free(NULL, s);
}

static int swap_roundtrip(ivp_compact_surface_t *s)
{
    int size = ivp_surface_get_byte_size(s);
    unsigned char *copy = (unsigned char *)malloc((size_t)size);
    memcpy(copy, s, (size_t)size);
    if (!ivp_compact_surface_byte_swap_all(s, false)) return 0;
    int differs = memcmp(copy, s, (size_t)size) != 0;
    /* the swapped header is not readable natively */
    if (!ivp_compact_surface_byte_swap_all(s, true)) return 0;
    int same = memcmp(copy, s, (size_t)size) == 0;
    free(copy);
    return differs && same && ivp_compact_surface_check(s, 0);
}

IVP_TEST(byte_swap_roundtrip)
{
    ivp_compact_surface_t *s = make_soup3(true, IVP_SLMP_NO_MERGE);
    IVP_ASSERT_TRUE(swap_roundtrip(s));
    ivp_compact_surface_free(NULL, s);
    s = make_soup3(true, IVP_SLMP_MERGE_AND_REALLOCATE);
    IVP_ASSERT_TRUE(swap_roundtrip(s));

    /* a swapped float reads as the byte reversed value */
    float mc0 = s->mass_center[1];
    ivp_compact_surface_byte_swap_all(s, false);
    uint32_t a, b;
    memcpy(&a, &mc0, 4);
    memcpy(&b, &s->mass_center[1], 4);
    IVP_ASSERT_EQ(b, ((a & 0xFFu) << 24) | ((a & 0xFF00u) << 8) | ((a >> 8) & 0xFF00u) | (a >> 24));
    ivp_compact_surface_byte_swap_all(s, true);
    ivp_compact_surface_free(NULL, s);
}

/* ── Surface managers ──────────────────────────────────────────────────── */

typedef struct ledge_list {
    const ivp_compact_ledge_t *l[64];
    int n;
} ledge_list_t;

static void add_ledge(void *ctx, const ivp_compact_ledge_t *ledge)
{
    ledge_list_t *ll = (ledge_list_t *)ctx;
    if (ll->n < 64) ll->l[ll->n++] = ledge;
}

IVP_TEST(surman_polygon_radius_with_center_shift)
{
    double c[24];
    box_coords(c, 0.5, 1.0, 1.5, 3.0, -2.0, 1.0);
    ivp_compact_surface_t *s = ivp_build_convex_surface_d(NULL, c, 8);
    ivp_surman_polygon_t sm;
    ivp_surman_polygon_init(&sm, s);
    /* ivp_surman_polygon.cxx:214: r = upper_limit_radius + |mc - center|,
     * dev = factor * r_header / 250 + |shift| */
    float center[3] = { 0.0f, 0.0f, 0.0f };
    float r = 0, d = 0;
    ivp_surman_get_radius_and_deviation(&sm.base, center, &r, &d);
    double shift = sqrt(9.0 + 4.0 + 1.0);
    IVP_ASSERT_NEAR(r, 1.87082875 + shift, 1e-5);
    IVP_ASSERT_NEAR(d, 241.0 / 250.0 * 1.87082875 + shift, 1e-5);
    float mcf[3] = { 3.0f, -2.0f, 1.0f };
    ivp_surman_get_radius_and_deviation(&sm.base, mcf, &r, &d);
    IVP_ASSERT_NEAR(r, 1.87082875, 1e-6);
    ivp_compact_surface_free(NULL, s);
}

IVP_TEST(surman_ball_reference_values)
{
    ivp_surman_ball_t b;
    ivp_surman_ball_init(&b, 0.7f);
    float v[3];
    ivp_surman_get_mass_center(&b.base, v);
    IVP_ASSERT_NEAR(v[0], 0.0, 0.0);
    ivp_surman_get_rotation_inertia(&b.base, v);
    IVP_ASSERT_NEAR(v[0], 0.0, 0.0);
    float c[3] = { 1, 2, 3 }, r = -1, d = -1;
    ivp_surman_get_radius_and_deviation(&b.base, c, &r, &d);
    IVP_ASSERT_NEAR(r, 0.0, 0.0);
    IVP_ASSERT_NEAR(d, 0.0, 0.0);
    IVP_ASSERT_NEAR(ivp_surman_get_ball_radius(&b.base), 0.7, 1e-7);
    const ivp_compact_ledge_t *dl = ivp_surman_get_single_convex(&b.base);
    IVP_ASSERT_NOT_NULL(dl);
    IVP_ASSERT_TRUE(dl == ivp_surman_ball_dummy_ledge());
    IVP_ASSERT_EQ(((uintptr_t)dl) & 15u, 0u);
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(dl), 0);

    ledge_list_t ll = { {0}, 0 };
    double p_in[3] = { 0.1, 0.2, 0.0 }, p_out[3] = { 2.0, 0.0, 0.0 };
    ivp_surman_get_all_ledges_within_radius(&b.base, p_in, 0.5, NULL, NULL, add_ledge, &ll);
    IVP_ASSERT_EQ(ll.n, 1);
    IVP_ASSERT_TRUE(ll.l[0] == dl);
    ivp_surman_get_all_ledges_within_radius(&b.base, p_out, 0.5, NULL, NULL, add_ledge, &ll);
    IVP_ASSERT_EQ(ll.n, 1);
}

IVP_TEST(surman_ledges_within_radius_reference_order)
{
    /* soup3 tree: root(left: [box0, box1], right: box2) */
    ivp_compact_surface_t *s = make_soup3(false, IVP_SLMP_NO_MERGE);
    ivp_surman_polygon_t sm;
    ivp_surman_polygon_init(&sm, s);
    const ivp_compact_ledgetree_node_t *root = ivp_surface_get_ledgetree_root(s);
    const ivp_compact_ledge_t *box0 =
        ivp_ledgetree_get_compact_ledge(ivp_ledgetree_left_son(ivp_ledgetree_left_son(root)));
    const ivp_compact_ledge_t *box1 =
        ivp_ledgetree_get_compact_ledge(ivp_ledgetree_right_son(ivp_ledgetree_left_son(root)));
    const ivp_compact_ledge_t *box2 = ivp_ledgetree_get_compact_ledge(ivp_ledgetree_right_son(root));

    ledge_list_t ll = { {0}, 0 };
    double big[3] = { 1.0, 1.5, 0.5 };
    ivp_surman_get_all_ledges_within_radius(&sm.base, big, 10.0, NULL, NULL, add_ledge, &ll);
    IVP_ASSERT_EQ(ll.n, 3);
    IVP_ASSERT_TRUE(ll.l[0] == box0 && ll.l[1] == box1 && ll.l[2] == box2);

    ll.n = 0;
    double near1[3] = { 2.0, 0.0, 0.0 };
    ivp_surman_get_all_ledges_within_radius(&sm.base, near1, 0.3, NULL, NULL, add_ledge, &ll);
    IVP_ASSERT_EQ(ll.n, 1);
    IVP_ASSERT_TRUE(ll.l[0] == box1);

    ll.n = 0;
    double far_away[3] = { 50.0, 0.0, 0.0 };
    ivp_surman_get_all_ledges_within_radius(&sm.base, far_away, 1.0, NULL, NULL, add_ledge, &ll);
    IVP_ASSERT_EQ(ll.n, 0);

    /* terminal root_ledge: added directly */
    ll.n = 0;
    ivp_surman_get_all_ledges_within_radius(&sm.base, far_away, 1.0, box2, NULL, add_ledge, &ll);
    IVP_ASSERT_EQ(ll.n, 1);

    ledge_list_t all = { {0}, 0 };
    ivp_surman_get_all_terminal_ledges(&sm.base, add_ledge, &all);
    IVP_ASSERT_EQ(all.n, 3);
    ivp_compact_surface_free(NULL, s);

    /* with root hull: the hull node returns its hull and stops; a
     * non-terminal root_ledge (the hull) descends into the children */
    s = make_soup3(true, IVP_SLMP_NO_MERGE);
    ivp_surman_polygon_init(&sm, s);
    const ivp_compact_ledge_t *hull = ivp_surman_get_single_convex(&sm.base);
    ll.n = 0;
    ivp_surman_get_all_ledges_within_radius(&sm.base, big, 10.0, NULL, NULL, add_ledge, &ll);
    IVP_ASSERT_EQ(ll.n, 1);
    IVP_ASSERT_TRUE(ll.l[0] == hull);
    ll.n = 0;
    ivp_surman_get_all_ledges_within_radius(&sm.base, big, 10.0, hull, NULL, add_ledge, &ll);
    IVP_ASSERT_EQ(ll.n, 3);
    ivp_compact_surface_free(NULL, s);
}

IVP_TEST(surman_two_triangle_root_ledge_distance)
{
    const double c[] = {0, 0, 0, 1, 0, 0, 0, 1, 0};
    ivp_compact_ledge_t *l = ivp_build_convex_ledge_d(NULL, c, 3);
    ivp_compact_surface_t *s = ivp_build_surface_from_ledge(NULL, l);
    ivp_surman_polygon_t sm;
    ivp_surman_polygon_init(&sm, s);
    const ivp_compact_ledge_t *tl = ivp_surman_get_single_convex(&sm.base);
    ledge_list_t ll = { {0}, 0 };
    /* 0.5 above the triangle plane: qlen_PF = 0.25 */
    double p[3] = { 0.2, 0.2, 0.5 };
    ivp_surman_get_all_ledges_within_radius(&sm.base, p, 0.49, tl, NULL, add_ledge, &ll);
    IVP_ASSERT_EQ(ll.n, 0);
    ivp_surman_get_all_ledges_within_radius(&sm.base, p, 0.51, tl, NULL, add_ledge, &ll);
    IVP_ASSERT_EQ(ll.n, 1);
    IVP_ASSERT_NEAR(ivp_ledge_calc_qlen_pf(tl, ivp_ledge_get_first_triangle(tl), p), 0.25, 1e-7);
    /* outside the triangle: distance to the nearest vertex / edge */
    double q[3] = { 2.0, 0.0, 0.0 };
    IVP_ASSERT_NEAR(ivp_ledge_calc_qlen_pf(tl, ivp_ledge_get_first_triangle(tl), q), 1.0, 1e-7);
    ivp_compact_surface_free(NULL, s);
    ivp_compact_ledge_free(NULL, l);
}

/* ── IVP_Compact_Ledge_Solver point/triangle helpers, bit exact ────────── */

/* calc_qlen_PF_F_space / calc_unscaled_qr_vals_F_space /
 * calc_unscaled_s_val_K_space on an irregular 32 point hull: the
 * IVP_U_Point::subtract(Float, Float) and IVP_U_Float_Point::
 * quad_distance_to differences are float expressions.  The hash covers the
 * bit patterns of all 8 x 6 (triangle, point) results printed by the
 * reference (qlen as double, qr checks/scale and s checks as float). */
static uint64_t fnv_bits(uint64_t h, uint64_t v, int nbytes)
{
    for (int i = 0; i < nbytes; i++) {
        h ^= (v >> (8 * i)) & 0xffu;
        h *= 0x100000001b3ull;
    }
    return h;
}

IVP_TEST(ledge_point_helpers_bit_exact)
{
    static const double geode[32 * 3] = {
        0.092, 0.358, 0.0,     -0.112, 0.325, 0.102,   0.018, 0.328, -0.208,   0.134, 0.277, 0.175,
        -0.262, 0.275, -0.046, 0.237, 0.244, -0.151,   -0.074, 0.21, 0.274,    -0.149, 0.202, -0.286,
        0.292, 0.165, 0.107,   -0.318, 0.153, 0.131,   0.141, 0.122, -0.301,   0.102, 0.1, 0.325,
        -0.317, 0.082, -0.184, 0.386, 0.062, -0.085,   -0.204, 0.033, 0.291,   -0.047, 0.011, -0.36,
        0.296, -0.012, 0.25,   -0.405, -0.038, 0.017,  0.269, -0.06, -0.268,   -0.017, -0.082, 0.364,
        -0.251, -0.115, -0.301, 0.328, -0.121, 0.044,  -0.301, -0.163, 0.21,   0.071, -0.172, -0.316,
        0.151, -0.19, 0.263,   -0.273, -0.212, -0.087, 0.252, -0.242, -0.117,  -0.107, -0.287, 0.256,
        -0.076, -0.282, -0.212, 0.183, -0.325, 0.096,  -0.159, -0.352, 0.042,  0.05, -0.361, -0.078,
    };
    static const double pts[6][3] = {
        {0.31, -0.07, 0.52}, {-0.2, 0.45, 0.1}, {0.05, 0.4, -0.33},
        {0.6, 0.2, -0.1}, {-0.41, -0.3, 0.27}, {0.12, 0.33, 0.29}};
    ivp_compact_ledge_t *l = ivp_build_convex_ledge_d(NULL, geode, 32);
    IVP_ASSERT_TRUE(l != NULL);
    IVP_ASSERT_TRUE(ivp_ledge_get_n_triangles(l) >= 8);

    /* triangle 0, point 0 (reference values) */
    const ivp_compact_triangle_t *t0 = ivp_ledge_get_first_triangle(l);
    IVP_ASSERT_TRUE(ivp_ledge_calc_qlen_pf(l, t0, pts[0]) == 0x1.787ab90a3bc52p-1);
    ivp_ledge_unscaled_qr_t qr;
    ivp_ledge_calc_unscaled_qr_vals(l, &t0->edges[0], pts[0], &qr);
    IVP_ASSERT_TRUE(qr.checks[0] == 0x1.9842dp-8f && qr.checks[1] == 0x1.a51f42p-8f &&
                    qr.checks[2] == -0x1.251f8p-7f && qr.scale == 0x1.e64622p-9f);
    float sr[2];
    ivp_ledge_calc_unscaled_s_vals(l, &t0->edges[0], pts[0], sr);
    IVP_ASSERT_TRUE(sr[0] == -0x1.601de2p-5f && sr[1] == 0x1.a3066p-4f);

    uint64_t h = 0xcbf29ce484222325ull;
    const ivp_compact_triangle_t *t = t0;
    for (int k = 0; k < 8; k++, t++) {
        for (int n = 0; n < 6; n++) {
            double q = ivp_ledge_calc_qlen_pf(l, t, pts[n]);
            uint64_t qb;
            memcpy(&qb, &q, sizeof(qb));
            h = fnv_bits(h, qb, 8);
            for (int e = 0; e < 3; e++) {
                ivp_ledge_calc_unscaled_qr_vals(l, &t->edges[e], pts[n], &qr);
                ivp_ledge_calc_unscaled_s_vals(l, &t->edges[e], pts[n], sr);
                const float f[6] = { qr.checks[0], qr.checks[1], qr.checks[2], qr.scale, sr[0], sr[1] };
                for (int i = 0; i < 6; i++) {
                    uint32_t fb;
                    memcpy(&fb, &f[i], sizeof(fb));
                    h = fnv_bits(h, fb, 4);
                }
            }
        }
    }
    IVP_ASSERT_TRUE(h == 0xa571750317213ea3ull);
    ivp_compact_ledge_free(NULL, l);
}

int main(void)
{
    IVP_REGISTER(box_header_matches_reference);
    IVP_REGISTER(box_node_box_sizes_match_reference);
    IVP_REGISTER(box_pierce_info);
    IVP_REGISTER(pointsoup_headers_match_reference);
    IVP_REGISTER(pointsoup_options_default_no_recentering);
    IVP_REGISTER(pointsoup_coplanar_input_gives_two_sided_triangle_or_null);
    IVP_REGISTER(halfspacesoup_cube_matches_reference);
    IVP_REGISTER(halfspacesoup_tolerance_is_1e4);
    IVP_REGISTER(halfspacesoup_rejects_overflowing_count);
    IVP_REGISTER(ledge_soup_soup3_matches_reference);
    IVP_REGISTER(ledge_soup_median_ties_match_reference);
    IVP_REGISTER(ledge_soup_root_hull_matches_reference);
    IVP_REGISTER(ledge_soup_merge_points);
    IVP_REGISTER(ledge_soup_single_and_triangle);
    IVP_REGISTER(polygon_convex_ngon_faces);
    IVP_REGISTER(rot_inertia_box_is_reference_value);
    IVP_REGISTER(check_ledge_detects_corruption);
    IVP_REGISTER(byte_swap_roundtrip);
    IVP_REGISTER(surman_polygon_radius_with_center_shift);
    IVP_REGISTER(surman_ball_reference_values);
    IVP_REGISTER(surman_ledges_within_radius_reference_order);
    IVP_REGISTER(surman_two_triangle_root_ledge_distance);
    IVP_REGISTER(ledge_point_helpers_bit_exact);
    IVP_RUN_ALL();
}
