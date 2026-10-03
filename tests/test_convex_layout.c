/* test_convex_layout.c -- reference layout of the convex pointsoup builder
 *
 * The expected values were dumped from the reference IVP
 * (IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_surface: qhull
 * 2.6 -> planes_to_template -> IVP_SurfaceBuilder_Polygon_Convex ->
 * IVP_Compact_Ledge_Generator -> IVP_SurfaceBuilder_Ledge_Soup).  Point
 * order, triangle order, edge start points / opposite indices and therefore
 * the float accumulated mass center are layout dependent, so they are
 * checked bit for bit.
 */
#include "ivp_test.h"
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_compact_triangle.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge_geom.h>
#include <ivp/geometry/ivp_ledgetree.h>

#include <math.h>
#include <stdint.h>
#include <string.h>

static uint32_t f_bits(float f)
{
    uint32_t u;
    memcpy(&u, &f, sizeof(u));
    return u;
}

/* corners for sx, sy, sz in -1, +1 (reference runner build_box_compact_surface) */
static ivp_compact_surface_t *box_pointsoup(double hx, double hy, double hz)
{
    double c[24];
    int n = 0;
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                c[n++] = (double)sx * hx;
                c[n++] = (double)sy * hy;
                c[n++] = (double)sz * hz;
            }
        }
    }
    return ivp_build_convex_surface_d(NULL, c, 8);
}

/* reference start_point_index(opposite_index) of edges 0..2, triangles 0..11 */
static const int box_edges[12][3][2] = {
    { { 0, 6 }, { 1, 25 }, { 2, 6 } },     { { 0, 12 }, { 3, 29 }, { 1, -6 } },
    { { 0, -6 }, { 2, 3 }, { 4, 11 } },    { { 4, -3 }, { 2, 12 }, { 5, 28 } },
    { { 3, -12 }, { 0, 3 }, { 6, 18 } },   { { 6, -3 }, { 0, -11 }, { 4, 24 } },
    { { 1, 6 }, { 5, -12 }, { 2, -25 } },  { { 1, 5 }, { 7, 11 }, { 5, -6 } },
    { { 3, 6 }, { 7, -5 }, { 1, -29 } },   { { 3, -18 }, { 6, 8 }, { 7, -6 } },
    { { 5, -11 }, { 7, 3 }, { 4, -28 } },  { { 4, -3 }, { 7, -8 }, { 6, -24 } },
};

/* reference point order (sign of x, y, z) */
static const int box_point_sign[8][3] = {
    { -1, +1, +1 }, { +1, -1, +1 }, { +1, +1, +1 }, { -1, -1, +1 },
    { -1, +1, -1 }, { +1, +1, -1 }, { -1, -1, -1 }, { +1, -1, -1 },
};

/* assertion helper: _fail_count is the IVP_TEST failure counter */
static void check_box_layout(int *_fail_count, const ivp_compact_ledge_t *l, float h)
{
    IVP_ASSERT_EQ(ivp_ledge_get_n_points(l), 8);
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(l), 12);
    IVP_ASSERT_EQ(ivp_ledge_get_size(l), 336);
    const ivp_compact_poly_point_t *pa = ivp_ledge_get_point_array(l);
    for (int i = 0; i < 8; i++) {
        for (int k = 0; k < 3; k++) {
            IVP_ASSERT_EQ(f_bits(pa[i].k[k]), f_bits((float)box_point_sign[i][k] * h));
        }
        IVP_ASSERT_EQ(f_bits(pa[i].hesse_val), 0u);
    }
    const ivp_compact_triangle_t *t = ivp_ledge_get_first_triangle(l);
    for (int i = 0; i < 12; i++) {
        IVP_ASSERT_EQ(ivp_tri_get_tri_index(&t[i]), i);
        IVP_ASSERT_EQ(ivp_tri_get_pierce_index(&t[i]), 11 - i);
        IVP_ASSERT_EQ(ivp_tri_get_material_index(&t[i]), 0);
        IVP_ASSERT_EQ(ivp_tri_get_is_virtual(&t[i]), 0);
        for (int e = 0; e < 3; e++) {
            IVP_ASSERT_EQ(ivp_edge_get_start_point_index(&t[i].edges[e]), box_edges[i][e][0]);
            IVP_ASSERT_EQ(ivp_edge_get_opposite_index(&t[i].edges[e]), box_edges[i][e][1]);
            IVP_ASSERT_EQ(ivp_edge_get_is_virtual(&t[i].edges[e]), 0);
        }
    }
    IVP_ASSERT_TRUE(ivp_compact_ledge_check(l));
}

IVP_TEST(box_0_6_layout_and_mass_center)
{
    ivp_compact_surface_t *s = box_pointsoup(0.6, 0.6, 0.6);
    IVP_ASSERT_NOT_NULL(s);
    check_box_layout(_fail_count, ivp_surface_get_single_convex(s), 0.6f);

    /* reference mass center 0x1.57dc8ap-34 (x3): sign and bits depend on
     * the triangle order */
    for (int k = 0; k < 3; k++) {
        IVP_ASSERT_EQ(f_bits(s->mass_center[k]), 0x2eabee45u);
        IVP_ASSERT_EQ(f_bits(s->rotation_inertia[k]), 0x3e2dc751u); /* 0x1.5b8ea2p-3 */
    }
    IVP_ASSERT_EQ(f_bits(s->upper_limit_radius), 0x3f850582u); /* 0x1.0a0b04p+0 */
    IVP_ASSERT_EQ(ivp_surface_get_max_deviation(s), 205);
    IVP_ASSERT_EQ(ivp_surface_get_byte_size(s), 412);
    ivp_compact_surface_free(NULL, s);
}

IVP_TEST(box_0_6_blob_matches_reference)
{
    /* complete reference compact surface (header, ledge, ledgetree node),
     * little endian */
    static const char ref_hex[] =
        "45eeab2e45eeab2e45eeab2e51c72d3e51c72d3e51c72d3e8205853fcd9c0100"
        "80010000000000000000000000000000d000000000000000041500000c000000"
        "00b0000000000600010019000200060001a0000000000c0003001d000100fa7f"
        "029000000000fa7f0200030004000b00038000000400fd7f02000c0005001c00"
        "047000000300f47f0000030006001200056000000600fd7f0000f57f04001800"
        "06500000010006000500f47f0200e77f074000000100050007000b000500fa7f"
        "08300000030006000700fb7f0100e37f092000000300ee7f060008000700fa7f"
        "0a1000000500f57f070003000400e47f0b0000000400fd7f0700f87f0600e87f"
        "9a9919bf9a99193f9a99193f000000009a99193f9a9919bf9a99193f00000000"
        "9a99193f9a99193f9a99193f000000009a9919bf9a9919bf9a99193f00000000"
        "9a9919bf9a99193f9a9919bf000000009a99193f9a99193f9a9919bf00000000"
        "9a9919bf9a9919bf9a9919bf000000009a99193f9a9919bf9a9919bf00000000"
        "00000000b0feffff0000000000000000000000008205853f91919100";
    const uint16_t probe = 1;
    unsigned char little_endian;
    memcpy(&little_endian, &probe, 1);
    if (!little_endian) return; /* the blob is stored in native byte order */

    ivp_compact_surface_t *s = box_pointsoup(0.6, 0.6, 0.6);
    IVP_ASSERT_NOT_NULL(s);
    int size = ivp_surface_get_byte_size(s);
    IVP_ASSERT_EQ(size * 2, (int)strlen(ref_hex));
    const unsigned char *b = (const unsigned char *)s;
    int first_diff = -1;
    for (int i = 0; i < size; i++) {
        unsigned int v = 0;
        for (int d = 0; d < 2; d++) {
            char ch = ref_hex[2 * i + d];
            v = v * 16u + (unsigned int)(ch <= '9' ? ch - '0' : ch - 'a' + 10);
        }
        if (b[i] != (unsigned char)v) {
            first_diff = i;
            break;
        }
    }
    IVP_ASSERT_EQ(first_diff, -1);
    ivp_compact_surface_free(NULL, s);
}

IVP_TEST(box_0_4_mass_center)
{
    ivp_compact_surface_t *s = box_pointsoup(0.4, 0.4, 0.4);
    IVP_ASSERT_NOT_NULL(s);
    check_box_layout(_fail_count, ivp_surface_get_single_convex(s), 0.4f);
    for (int k = 0; k < 3; k++) {
        IVP_ASSERT_EQ(f_bits(s->mass_center[k]), 0xaf088889u);      /* -0x1.111112p-33 */
        IVP_ASSERT_EQ(f_bits(s->rotation_inertia[k]), 0x3d9a7848u); /* 0x1.34f09p-4 */
    }
    ivp_compact_surface_free(NULL, s);
}

IVP_TEST(box_builder_uses_pointsoup_layout)
{
    /* ivp_build_box_surface == pointsoup of the 8 corners */
    ivp_compact_surface_t *a = ivp_build_box_surface(NULL, 0.6f, 0.6f, 0.6f);
    ivp_compact_surface_t *b = box_pointsoup((double)0.6f, (double)0.6f, (double)0.6f);
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);
    IVP_ASSERT_EQ(ivp_surface_get_byte_size(a), ivp_surface_get_byte_size(b));
    IVP_ASSERT_EQ(memcmp(a, b, (size_t)ivp_surface_get_byte_size(a)), 0);
    check_box_layout(_fail_count, ivp_surface_get_single_convex(a), 0.6f);
    ivp_compact_surface_free(NULL, a);
    ivp_compact_surface_free(NULL, b);
}

IVP_TEST(triangle_ledge_layout)
{
    /* convert_triangle_to_compace_ledge: two sided template ledge */
    const double c[9] = { 0, 0, 0, 1, 0, 0, 0, 1, 0 };
    ivp_compact_ledge_t *l = ivp_build_convex_ledge_d(NULL, c, 3);
    IVP_ASSERT_NOT_NULL(l);
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(l), 2);
    IVP_ASSERT_EQ(ivp_ledge_get_n_points(l), 3);
    static const int edges[2][3][2] = {
        { { 0, 4 }, { 1, 5 }, { 2, 3 } },
        { { 1, -4 }, { 0, -3 }, { 2, -5 } },
    };
    const ivp_compact_triangle_t *t = ivp_ledge_get_first_triangle(l);
    for (int i = 0; i < 2; i++) {
        IVP_ASSERT_EQ(ivp_tri_get_tri_index(&t[i]), i);
        IVP_ASSERT_EQ(ivp_tri_get_pierce_index(&t[i]), 1 - i);
        for (int e = 0; e < 3; e++) {
            IVP_ASSERT_EQ(ivp_edge_get_start_point_index(&t[i].edges[e]), edges[i][e][0]);
            IVP_ASSERT_EQ(ivp_edge_get_opposite_index(&t[i].edges[e]), edges[i][e][1]);
        }
    }
    ivp_compact_ledge_free(NULL, l);

    /* degenerate triangle */
    const double d[9] = { 0, 0, 0, 1, 0, 0, 2, 0, 0 };
    IVP_ASSERT_NULL(ivp_build_convex_ledge_d(NULL, d, 3));
}

IVP_TEST(prism16_valid_ledge)
{
    /* 16-gon caps are triangulated by P_Sur_2D (ear cutting in the face
     * plane); every edge must be linked and the hull convex */
    double c[32 * 3];
    for (int i = 0; i < 16; i++) {
        double a = 6.283185307179586 * i / 16;
        c[3 * i + 0] = cos(a);
        c[3 * i + 1] = sin(a);
        c[3 * i + 2] = -0.5;
        c[3 * (16 + i) + 0] = cos(a);
        c[3 * (16 + i) + 1] = sin(a);
        c[3 * (16 + i) + 2] = 0.5;
    }
    ivp_compact_ledge_t *l = ivp_build_convex_ledge_d(NULL, c, 32);
    IVP_ASSERT_NOT_NULL(l);
    IVP_ASSERT_EQ(ivp_ledge_get_n_points(l), 32);
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(l), 60);
    IVP_ASSERT_TRUE(ivp_compact_ledge_check(l));
    ivp_compact_ledge_free(NULL, l);
}

int main(void)
{
    IVP_REGISTER(box_0_6_layout_and_mass_center);
    IVP_REGISTER(box_0_6_blob_matches_reference);
    IVP_REGISTER(box_0_4_mass_center);
    IVP_REGISTER(box_builder_uses_pointsoup_layout);
    IVP_REGISTER(triangle_ledge_layout);
    IVP_REGISTER(prism16_valid_ledge);

    IVP_RUN_ALL();
}
