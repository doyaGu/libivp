/* test_convex_decompose.c — GEOMPACK convex decomposition parity.
 *
 * Expected values printed by the legacy engine
 * (IVP_Convex_Decompositor::perform_convex_decomposition_on_concave_polyhedron
 * and IVP_SurfaceBuilder_Polyhedron_Concave::convert_concave_polyhedron_to_
 * single_compact_surface, tolin = 1e-9) for the identical prisms.
 */
#include "ivp_test.h"

#include <ivp/builder/ivp_convex_decompose.h>
#include <ivp/builder/ivp_build_concave.h>
#include <ivp/geometry/ivp_compact_ledge_geom.h>
#include <ivp/geometry/ivp_surman.h>

#include <string.h>

/* Extrude a CCW 2D profile (head vertex convex) into a prism with polygon
 * faces CCW seen from outside, exactly like the reference test generator. */
typedef struct prism {
    double pts[2 * 16 * 3];
    int faces[2 * 16 + 4 * 16];
    int sizes[2 + 16];
    int n_pts;
    int n_faces;
} prism_t;

static void make_prism(prism_t *p, const double *prof, int n, double h)
{
    p->n_pts = 2 * n;
    for (int i = 0; i < n; i++) {
        p->pts[3 * i + 0] = prof[2 * i];
        p->pts[3 * i + 1] = prof[2 * i + 1];
        p->pts[3 * i + 2] = 0.0;
        p->pts[3 * (n + i) + 0] = prof[2 * i];
        p->pts[3 * (n + i) + 1] = prof[2 * i + 1];
        p->pts[3 * (n + i) + 2] = h;
    }
    int k = 0, f = 0;
    p->faces[k++] = 0;
    for (int i = n - 1; i >= 1; i--) p->faces[k++] = i;
    p->sizes[f++] = n;
    for (int i = 0; i < n; i++) p->faces[k++] = n + i;
    p->sizes[f++] = n;
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        p->faces[k++] = i;
        p->faces[k++] = j;
        p->faces[k++] = n + j;
        p->faces[k++] = n + i;
        p->sizes[f++] = 4;
    }
    p->n_faces = f;
}

static const double L_prof[] = { 0, 0, 3, 0, 3, 1, 1, 1, 1, 3, 0, 3 };

/* the L of the reference dump: triangulated caps, quad sides */
static void make_l(prism_t *p)
{
    static const int faces[] = {
        0, 2, 1, 0, 3, 2, 0, 4, 3, 0, 5, 4,
        6, 7, 8, 6, 8, 9, 6, 9, 10, 6, 10, 11,
        0, 1, 7, 6, 1, 2, 8, 7, 2, 3, 9, 8, 3, 4, 10, 9, 4, 5, 11, 10, 5, 0, 6, 11 };
    static const int sizes[] = { 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4 };
    make_prism(p, L_prof, 6, 1.0);
    memcpy(p->faces, faces, sizeof(faces));
    memcpy(p->sizes, sizes, sizeof(sizes));
    p->n_faces = 14;
}
static const double U_prof[] = { 0, 0, 3, 0, 3, 3, 2, 3, 2, 1, 1, 1, 1, 3, 0, 3 };
static const double PLUS_prof[] = { 1, 0, 2, 0, 2, 1, 3, 1, 3, 2, 2, 2, 2, 3, 1, 3, 1, 2, 0, 2, 0, 1, 1, 1 };
static const double STAR_prof[] = {
    0.0, 2.0, -0.470228, 0.647214, -1.902113, 0.618034, -0.760845, -0.247214,
    -1.175571, -1.618034, -0.0, -0.8, 1.175571, -1.618034, 0.760845, -0.247214,
    1.902113, 0.618034, 0.470228, 0.647214 };

typedef struct expect {
    const double *prof;
    int n;
    double h;
    float angacc, rdacc;
    int parts;
    double mc[3], ri[3], r;
    int dev, size;
} expect_t;

static int near_rel(double a, double b)
{
    double d = fabs(a - b);
    double m = fabs(b) > 1.0 ? fabs(b) : 1.0;
    return d <= 1e-5 * m || d <= 1e-6;
}

IVP_TEST(geompack_prisms_match_reference)
{
    static const expect_t cases[] = {
        { L_prof, 6, 1.0, 30.0f, 0.3f, 2, {1.1, 1.1, 0.5}, {0.7281178, 0.7281178, 1.022948}, 2.251666, 244, 1044 },
        { L_prof, 6, 1.0, 10.0f, 0.1f, 2, {1.1, 1.1, 0.5}, {0.7281178, 0.7281178, 1.022948}, 2.251666, 244, 1044 },
        { U_prof, 8, 2.0, 30.0f, 0.3f, 3, {1.5, 1.357143, 1}, {0.845676, 0.9978009, 1.220062}, 2.439053, 229, 1340 },
        { PLUS_prof, 12, 1.0, 30.0f, 0.3f, 3, {1.5, 1.5, 0.5}, {0.4904647, 0.4904647, 0.6835366}, 1.658312, 239, 1660 },
        { STAR_prof, 10, 0.7, 30.0f, 0.3f, 5, {0.01529428, -0.06233155, 0.35}, {0.5386139, 0.4791927, 0.7186068}, 2.091876, 247, 2028 },
        { STAR_prof, 10, 0.7, 50.0f, 0.5f, 6, {0.0, 0.0, 0.35}, {0.4962186, 0.4962187, 0.6993791}, 2.030394, 250, 2132 },
    };
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        const expect_t *e = &cases[c];
        prism_t p;
        if (e->prof == L_prof) make_l(&p);
        else make_prism(&p, e->prof, e->n, e->h);
        ivp_decompose_options_t o = ivp_decompose_options_default();
        o.tolin = 1e-9f;
        o.angacc = e->angacc;
        o.rdacc = e->rdacc;

        double *pp = NULL;
        int *po = NULL;
        int n = ivp_convex_decompose_points(NULL, p.pts, p.n_pts, p.faces, p.sizes, p.n_faces, &o, &pp, &po);
        IVP_ASSERT_EQ(n, e->parts);
        ivp_decompose_points_free(NULL, n, pp, po);

        ivp_concave_options_t co = ivp_concave_options_default();
        co.decompose = o;
        ivp_compact_surface_t *s = ivp_build_concave_polyhedron_surface(NULL, p.pts, p.n_pts, p.faces,
                                                                        p.sizes, p.n_faces, &co);
        IVP_ASSERT_NOT_NULL(s);
        for (int i = 0; i < 3; i++) {
            IVP_ASSERT_TRUE(near_rel((double)s->mass_center[i], e->mc[i]));
            IVP_ASSERT_TRUE(near_rel((double)s->rotation_inertia[i], e->ri[i]));
        }
        IVP_ASSERT_TRUE(near_rel((double)s->upper_limit_radius, e->r));
        IVP_ASSERT_EQ(ivp_surface_get_max_deviation(s), e->dev);
        IVP_ASSERT_EQ(ivp_surface_get_byte_size(s), e->size);
        IVP_ASSERT_TRUE(ivp_compact_surface_check(s, 0));
        /* root hull like the reference concave builder */
        ivp_surman_polygon_t sm;
        ivp_surman_polygon_init(&sm, s);
        const ivp_compact_ledge_t *hull = ivp_surman_get_single_convex(&sm.base);
        IVP_ASSERT_NOT_NULL(hull);
        IVP_ASSERT_FALSE(ivp_ledge_is_terminal(hull));
        ivp_compact_surface_free(NULL, s);
    }
}

IVP_TEST(geompack_l_cut_vertices)
{
    /* legacy part0 of the L: 12 points incl. the inserted cut vertices
     * (0.333333 1 0) and (5.55112e-17 1 0) */
    prism_t p;
    make_l(&p);
    ivp_decompose_options_t o = ivp_decompose_options_default();
    double *pp = NULL;
    int *po = NULL;
    int n = ivp_convex_decompose_points(NULL, p.pts, p.n_pts, p.faces, p.sizes, p.n_faces, &o, &pp, &po);
    IVP_ASSERT_EQ(n, 2);
    IVP_ASSERT_EQ(po[1] - po[0], 12);
    IVP_ASSERT_EQ(po[2] - po[1], 10);
    IVP_ASSERT_NEAR(pp[3 * 8 + 0], 1.0 / 3.0, 1e-6);
    IVP_ASSERT_NEAR(pp[3 * 8 + 1], 1.0, 1e-12);
    IVP_ASSERT_NEAR(pp[3 * 9 + 0], 0.0, 1e-12);
    ivp_decompose_points_free(NULL, n, pp, po);
}

IVP_TEST(geompack_convex_input_single_part)
{
    static const double sq[] = { 0, 0, 1, 0, 1, 1, 0, 1 };
    prism_t p;
    make_prism(&p, sq, 4, 1.0);
    double *pp = NULL;
    int *po = NULL;
    int n = ivp_convex_decompose_points(NULL, p.pts, p.n_pts, p.faces, p.sizes, p.n_faces, NULL, &pp, &po);
    IVP_ASSERT_EQ(n, 1);
    IVP_ASSERT_EQ(po[1], 8);
    ivp_decompose_points_free(NULL, n, pp, po);
}

IVP_TEST(geompack_triangle_mesh_api)
{
    /* the same L as a CCW triangle mesh through ivp_build_concave_surface */
    prism_t p;
    make_l(&p);
    float v[36];
    for (int i = 0; i < 36; i++) v[i] = (float)p.pts[i];
    int tris[3 * 64];
    int nt = 0, k = 0;
    for (int f = 0; f < p.n_faces; f++) {
        for (int i = 1; i + 1 < p.sizes[f]; i++) {
            tris[3 * nt + 0] = p.faces[k];
            tris[3 * nt + 1] = p.faces[k + i];
            tris[3 * nt + 2] = p.faces[k + i + 1];
            nt++;
        }
        k += p.sizes[f];
    }
    ivp_compact_surface_t *s = ivp_build_concave_surface(NULL, v, 12, tris, nt, NULL);
    IVP_ASSERT_NOT_NULL(s);
    IVP_ASSERT_TRUE(near_rel((double)s->mass_center[0], 1.1));
    IVP_ASSERT_TRUE(near_rel((double)s->rotation_inertia[2], 1.022948));
    IVP_ASSERT_TRUE(ivp_compact_surface_check(s, 0));
    ivp_compact_surface_free(NULL, s);
}

int main(void)
{
    IVP_REGISTER(geompack_prisms_match_reference);
    IVP_REGISTER(geompack_l_cut_vertices);
    IVP_REGISTER(geompack_convex_input_single_part);
    IVP_REGISTER(geompack_triangle_mesh_api);
    IVP_RUN_ALL();
}
