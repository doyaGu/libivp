/* test_convex_robustness.c -- the mainline convex pointsoup builder on
 * inputs the reference construction gets wrong: hulls Qhull 2020.2 rejects
 * under the reference's fixed roundoff, hulls too large for one ledge, and
 * flat or collinear point sets.  Each built ledge must pass the reference
 * check_ledge and hold every input point within IVP_MIN_EDGE_LEN (the
 * degenerate-facet point removal works at that resolution). */
#include "ivp_test.h"

#include <ivp/base/ivp_types.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_compact_ledge_geom.h>

#include <math.h>
#include <stdbool.h>
#include <string.h>

#define MAX_POINTS 2600

typedef struct {
    double c[3 * MAX_POINTS];
    int n;
} soup_t;

static void add(soup_t *s, double x, double y, double z)
{
    s->c[3 * s->n + 0] = x;
    s->c[3 * s->n + 1] = y;
    s->c[3 * s->n + 2] = z;
    s->n++;
}

/* A cube of half size 1 with a k x k grid of points on every face (each face
 * point is coplanar with the face's corners), rotated, scaled and moved, then
 * rounded to float like mesh vertices */
static void grid_box(soup_t *s, int k, double scale, const double offset[3])
{
    const double ax = 1.234, ay = 0.567, az = 2.89;
    s->n = 0;
    for (int i = 0; i <= k; i++) {
        for (int j = 0; j <= k; j++) {
            double u = -1.0 + 2.0 * i / k, v = -1.0 + 2.0 * j / k;
            const double faces[6][3] = {{1, u, v}, {-1, u, v}, {u, 1, v},
                                        {u, -1, v}, {u, v, 1}, {u, v, -1}};
            for (int f = 0; f < 6; f++) {
                double x = faces[f][0] * scale, y = faces[f][1] * scale, z = faces[f][2] * scale;
                double y1 = y * cos(ax) - z * sin(ax), z1 = y * sin(ax) + z * cos(ax);
                double x2 = x * cos(ay) + z1 * sin(ay), z2 = -x * sin(ay) + z1 * cos(ay);
                double x3 = x2 * cos(az) - y1 * sin(az), y3 = x2 * sin(az) + y1 * cos(az);
                add(s, (float)(x3 + offset[0]), (float)(y3 + offset[1]), (float)(z2 + offset[2]));
            }
        }
    }
}

/* A dome of radius 5: the top, `lat` rings of `ring` points down to the rim,
 * and an inner ring and centre on the flat base */
static void dome(soup_t *s, int ring, int lat)
{
    s->n = 0;
    add(s, 0, 5, 0);
    for (int l = 1; l <= lat; l++) {
        double phi = 1.5707963267948966 * l / lat;
        for (int k = 0; k < ring; k++) {
            double t = 6.283185307179586 * k / ring;
            add(s, (float)(5 * sin(phi) * cos(t)), (float)(5 * cos(phi)), (float)(5 * sin(phi) * sin(t)));
        }
    }
    for (int k = 0; k < ring; k++) {
        double t = 6.283185307179586 * k / ring;
        add(s, (float)(2.5 * cos(t)), 0, (float)(2.5 * sin(t)));
    }
    add(s, 0, 0, 0);
}

/* How far the farthest input point lies outside the ledge (0 when inside) */
static double outside_distance(const ivp_compact_ledge_t *l, const soup_t *s)
{
    int n_points = ivp_ledge_get_n_points(l);
    const ivp_compact_poly_point_t *points = ivp_ledge_get_point_array(l);
    double centroid[3] = {0, 0, 0};
    for (int i = 0; i < n_points; i++) {
        for (int c = 0; c < 3; c++) centroid[c] += (double)points[i].k[c] / n_points;
    }
    double worst = 0.0;
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(l);
    for (int t = 0; t < ivp_ledge_get_n_triangles(l); t++, tri++) {
        double p[3][3];
        for (int j = 0; j < 3; j++) {
            const ivp_compact_poly_point_t *q = ivp_edge_get_start_point(&tri->edges[j], l);
            for (int c = 0; c < 3; c++) p[j][c] = q->k[c];
        }
        double e1[3], e2[3];
        for (int c = 0; c < 3; c++) {
            e1[c] = p[1][c] - p[0][c];
            e2[c] = p[2][c] - p[0][c];
        }
        double nrm[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
                         e1[0] * e2[1] - e1[1] * e2[0]};
        double len = sqrt(nrm[0] * nrm[0] + nrm[1] * nrm[1] + nrm[2] * nrm[2]);
        if (len == 0.0) continue;
        double inward = 0.0;
        for (int c = 0; c < 3; c++) inward += nrm[c] / len * (centroid[c] - p[0][c]);
        if (fabs(inward) < 1e-9) continue; /* a flat ledge */
        double sign = inward > 0 ? -1.0 : 1.0;
        for (int i = 0; i < s->n; i++) {
            double d = 0.0;
            for (int c = 0; c < 3; c++) d += nrm[c] / len * (s->c[3 * i + c] - p[0][c]);
            if (sign * d > worst) worst = sign * d;
        }
    }
    return worst;
}

/* A ledge passing check_ledge that holds every input point */
static bool valid_hull(const ivp_compact_ledge_t *l, const soup_t *s)
{
    return l && ivp_compact_ledge_check(l) && outside_distance(l, s) <= IVP_MIN_EDGE_LEN;
}

/* The reference's E1.0e-18 roundoff is below double precision at 10^4:
 * Qhull rejected every attempt and the build failed */
IVP_TEST(rotated_box_far_from_origin)
{
    static soup_t s;
    const double offset[3] = {1e4, -3e3, 7e3};
    grid_box(&s, 8, 1.0, offset);
    ivp_compact_ledge_t *l = ivp_build_convex_ledge_d(NULL, s.c, s.n);
    IVP_ASSERT_TRUE(valid_hull(l, &s));
    if (l) ivp_compact_ledge_free(NULL, l);
}

/* Scaled by 100 the reference's first pass failed as a wide merge */
IVP_TEST(scaled_rotated_box)
{
    static soup_t s;
    const double offset[3] = {0, 0, 0};
    grid_box(&s, 8, 100.0, offset);
    ivp_compact_ledge_t *l = ivp_build_convex_ledge_d(NULL, s.c, s.n);
    IVP_ASSERT_TRUE(valid_hull(l, &s));
    if (l) ivp_compact_ledge_free(NULL, l);
}

/* The rim and the flat base meet at a near-coplanar ring: every ring point is a
 * vertex, and no base point is */
IVP_TEST(dome_keeps_its_vertices)
{
    static soup_t s;
    dome(&s, 32, 8);
    ivp_compact_ledge_t *l = ivp_build_convex_ledge_d(NULL, s.c, s.n);
    IVP_ASSERT_TRUE(valid_hull(l, &s));
    if (l) {
        IVP_ASSERT_EQ(ivp_ledge_get_n_points(l), 1 + 32 * 8);
        IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(l), 2 * (1 + 32 * 8) - 4);
        ivp_compact_ledge_free(NULL, l);
    }
}

/* 2305 hull vertices need 4606 triangles, more than one ledge holds: no
 * ledge, where the reference joggled until fewer vertices remained and
 * linked the joggled facets over the real points (not convex) */
IVP_TEST(hull_beyond_the_triangle_limit_is_refused)
{
    static soup_t s;
    dome(&s, 96, 24);
    IVP_ASSERT_TRUE(2 * (1 + 96 * 24) - 4 > IVP_BUILD_MAX_TRIANGLES_PER_LEDGE);
    IVP_ASSERT_NULL(ivp_build_convex_ledge_d(NULL, s.c, s.n));
    dome(&s, 64, 16); /* 1025 vertices fit */
    ivp_compact_ledge_t *l = ivp_build_convex_ledge_d(NULL, s.c, s.n);
    IVP_ASSERT_TRUE(valid_hull(l, &s));
    if (l) {
        IVP_ASSERT_EQ(ivp_ledge_get_n_points(l), 1 + 64 * 16);
        ivp_compact_ledge_free(NULL, l);
    }
}

/* Coplanar points: a two sided ledge of their 2D hull (interior points dropped) */
IVP_TEST(flat_points_make_a_two_sided_polygon)
{
    static soup_t s;
    s.n = 0;
    for (int k = 0; k < 40; k++) {
        double t = 6.283185307179586 * k / 40;
        add(&s, cos(t), 0.25, sin(t));
        add(&s, 0.5 * cos(t), 0.25, 0.5 * sin(t));
    }
    ivp_compact_ledge_t *l = ivp_build_convex_ledge_d(NULL, s.c, s.n);
    IVP_ASSERT_NOT_NULL(l);
    if (l) {
        IVP_ASSERT_EQ(ivp_ledge_get_n_points(l), 40);
        IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(l), 2 * 40 - 4);
        IVP_ASSERT_TRUE(ivp_compact_ledge_check(l));
        const ivp_compact_poly_point_t *points = ivp_ledge_get_point_array(l);
        for (int i = 0; i < 40; i++) IVP_ASSERT_NEAR(points[i].k[1], 0.25, 1e-7);
        ivp_compact_ledge_free(NULL, l);
    }
    /* a square: four triangles, two a side */
    const double square[12] = {0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1};
    l = ivp_build_convex_ledge_d(NULL, square, 4);
    IVP_ASSERT_NOT_NULL(l);
    if (l) {
        IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(l), 4);
        IVP_ASSERT_TRUE(ivp_compact_ledge_check(l));
        ivp_compact_ledge_free(NULL, l);
    }
}

IVP_TEST(collinear_points_make_no_ledge)
{
    double c[3 * 10];
    for (int k = 0; k < 10; k++) {
        c[3 * k + 0] = k;
        c[3 * k + 1] = 2.0 * k;
        c[3 * k + 2] = -k;
    }
    IVP_ASSERT_NULL(ivp_build_convex_ledge_d(NULL, c, 10));
}

int main(void)
{
    IVP_REGISTER(rotated_box_far_from_origin);
    IVP_REGISTER(scaled_rotated_box);
    IVP_REGISTER(dome_keeps_its_vertices);
    IVP_REGISTER(hull_beyond_the_triangle_limit_is_refused);
    IVP_REGISTER(flat_points_make_a_two_sided_polygon);
    IVP_REGISTER(collinear_points_make_no_ledge);
    IVP_RUN_ALL();
}
