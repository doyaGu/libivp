/* test_ray_solver.c — IVP_Ray_Solver / IVP_Sphere_Solver parity tests.
 *
 * Expected results were produced by the legacy C++ engine
 * (IVP_Ray_Solver_Min_Hash::check_ray_against_object /
 * check_ray_against_all_objects_in_sim and IVP_Sphere_Solver::
 * check_sphere_against_environment) on the identical scene.
 */
#include "ivp_test.h"

#include <ivp/builder/ivp_build_surface.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/collision/ivp_ray_solver.h>
#include <ivp/collision/ivp_sphere_solver.h>
#include <ivp/containers/ivp_min_hash.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_phantom.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/math/ivp_fast_math.h>

#include <stdio.h>
#include <string.h>

/* ── Scene ─────────────────────────────────────────────────────────────── */

typedef struct scene {
    ivp_environment_t *env;
    ivp_compact_surface_t *surf[4];
    ivp_surman_polygon_t sm[4];
    ivp_surman_ball_t sm_ball;
    ivp_object_t *box, *tilted, *tri, *soup3, *ball;
} scene_t;

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

static ivp_object_t *add_static(scene_t *sc, int i, ivp_compact_surface_t *s, double px, double py,
                                double pz, ivp_quatr_t q, const char *name)
{
    sc->surf[i] = s;
    ivp_surman_polygon_init(&sc->sm[i], s);
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_POLYGON;
    t.mass = IVP_REAL(0.0);
    t.physical_unmoveable = true;
    t.name = name;
    t.position = ivp_vec3r((ivp_real_t)px, (ivp_real_t)py, (ivp_real_t)pz);
    t.orientation = q;
    return ivp_environment_create_object(sc->env, &t, &sc->sm[i].base);
}

static void scene_init(scene_t *sc)
{
    memset(sc, 0, sizeof(*sc));
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    sc->env = ivp_environment_create(&cfg);
    ivp_quatr_t qi = ivp_quatr_make(0, 0, 0, 1);
    double c[24];

    box_coords(c, 1, 1, 1, 0, 0, 0);
    sc->box = add_static(sc, 0, ivp_build_convex_surface_d(NULL, c, 8), 0, 0, 0, qi, "box");

    box_coords(c, 0.5, 0.5, 0.5, 0, 0, 0);
    ivp_quatr_t qr = ivp_quatr_make(0, 0, (ivp_real_t)sin(0.2), (ivp_real_t)cos(0.2));
    sc->tilted = add_static(sc, 1, ivp_build_convex_surface_d(NULL, c, 8), 10, 0.3, 0, qr, "tilted");

    const double tc[] = { -2, -2, 0, 2, -2, 0, 0, 2, 0 };
    ivp_compact_ledge_t *tl = ivp_build_convex_ledge_d(NULL, tc, 3);
    sc->tri = add_static(sc, 2, ivp_build_surface_from_ledge(NULL, tl), 0, 0, -5, qi, "tri");
    ivp_compact_ledge_free(NULL, tl);

    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);
    ivp_compact_ledge_t *l[3];
    for (int i = 0; i < 3; i++) {
        box_coords(c, 0.5, 0.5, 0.5, 2.0 * i, 0, 0);
        l[i] = ivp_build_convex_ledge_d(NULL, c, 8);
        ivp_ledge_soup_add(&soup, l[i]);
    }
    sc->soup3 = add_static(sc, 3, ivp_ledge_soup_compile(&soup), 0, 20, 0, qi, "soup3");
    ivp_ledge_soup_destroy(&soup);
    for (int i = 0; i < 3; i++) ivp_compact_ledge_free(NULL, l[i]);

    ivp_surman_ball_init(&sc->sm_ball, 1.5f);
    ivp_object_template_t bt;
    ivp_object_template_init(&bt);
    bt.type = IVP_OBJECT_BALL;
    bt.mass = IVP_REAL(0.0);
    bt.physical_unmoveable = true;
    bt.name = "ball";
    bt.position = ivp_vec3r(0, -20, 0);
    /* The reference ball radius is the object's extra_radius (IVP_Ball),
     * taken from the ball surface manager at creation. */
    sc->ball = ivp_environment_create_object(sc->env, &bt, &sc->sm_ball.base);
}

static void scene_free(scene_t *sc)
{
    ivp_environment_destroy(sc->env);
    for (int i = 0; i < 4; i++) ivp_compact_surface_free(NULL, sc->surf[i]);
}

typedef struct cast_result {
    int n;
    const ivp_ray_hit_t *hits[IVP_MAX_NUM_RAY_HITS];
} cast_result_t;

static ivp_ray_solver_min_hash_t s_rs;

static void cast(scene_t *sc, double sx, double sy, double sz, double dx, double dy, double dz,
                 double len, ivp_object_t *only, unsigned flags, cast_result_t *out)
{
    double l = sqrt(dx * dx + dy * dy + dz * dz);
    ivp_ray_solver_template_t t;
    t.ray_start_point = ivp_vec3r((ivp_real_t)sx, (ivp_real_t)sy, (ivp_real_t)sz);
    t.ray_normized_direction = ivp_vec3r((ivp_real_t)(dx / l), (ivp_real_t)(dy / l), (ivp_real_t)(dz / l));
    t.ray_length = (ivp_real_t)len;
    t.ray_flags = flags;
    ivp_ray_solver_min_hash_init(&s_rs, &t);
    if (only) ivp_ray_solver_check_object(&s_rs.base, only);
    else ivp_ray_solver_check_all_objects(&s_rs.base, sc->env);
    out->n = ivp_ray_solver_min_hash_get_sorted(&s_rs, out->hits, IVP_MAX_NUM_RAY_HITS);
}

/* ── Tests ─────────────────────────────────────────────────────────────── */

IVP_TEST(ray_reference_object_hits)
{
    scene_t sc;
    scene_init(&sc);
    cast_result_t r;

    /* legacy: center_x hits=0 — a ray through the exact face center runs
     * along the face diagonal: checks == 0, endless walk, no hit */
    cast(&sc, -10, 0, 0, 1, 0, 0, 20, sc.box, 0, &r);
    IVP_ASSERT_EQ(r.n, 0);

    /* legacy: offset_x dist=9 n=(-1 0 0) */
    cast(&sc, -10, 0.1, 0.2, 1, 0, 0, 20, sc.box, 0, &r);
    IVP_ASSERT_EQ(r.n, 1);
    IVP_ASSERT_NEAR(r.hits[0]->hit_distance, 9.0, 1e-5);
    IVP_ASSERT_NEAR(r.hits[0]->hit_normal.k[0], -1.0, 1e-6);
    IVP_ASSERT_TRUE(r.hits[0]->hit_object == sc.box);
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(r.hits[0]->hit_ledge), 12);
    IVP_ASSERT_NEAR(r.hits[0]->hit_point.k[0], -1.0, 1e-5);

    /* legacy: a ray starting inside a convex ledge does not hit it */
    cast(&sc, 0, 0.1, 0.2, 1, 0, 0, 20, sc.box, 0, &r);
    IVP_ASSERT_EQ(r.n, 0);

    /* legacy: too short */
    cast(&sc, -10, 0.1, 0.2, 1, 0, 0, 5, sc.box, 0, &r);
    IVP_ASSERT_EQ(r.n, 0);

    /* legacy: oblique dist=4.821826 n=(-1 0 0) */
    cast(&sc, -5, -3.3, 1.7, 0.8, 0.5, -0.2, 20, sc.box, 0, &r);
    IVP_ASSERT_EQ(r.n, 1);
    IVP_ASSERT_NEAR(r.hits[0]->hit_distance, 4.821826, 1e-5);
    IVP_ASSERT_NEAR(r.hits[0]->hit_normal.k[0], -1.0, 1e-6);

    /* legacy: rotated object, normal stays in object space:
     * tilted dist=4.478288 n=(-1 0 -0) */
    cast(&sc, 5, 0.25, 0.1, 1, 0, 0, 20, sc.tilted, 0, &r);
    IVP_ASSERT_EQ(r.n, 1);
    IVP_ASSERT_NEAR(r.hits[0]->hit_distance, 4.478288, 2e-6 * 5);
    IVP_ASSERT_NEAR(r.hits[0]->hit_normal.k[0], -1.0, 1e-6);
    IVP_ASSERT_NEAR(r.hits[0]->hit_normal.k[1], 0.0, 1e-6);

    scene_free(&sc);
}

IVP_TEST(ray_reference_world_queries)
{
    scene_t sc;
    scene_init(&sc);
    cast_result_t r;

    /* legacy tri_above: box dist=4 n=(0 0 1); tri dist=10 n=(0 0 1) */
    cast(&sc, 0.1, 0.2, 5, 0, 0, -1, 20, NULL, 0, &r);
    IVP_ASSERT_EQ(r.n, 2);
    IVP_ASSERT_TRUE(r.hits[0]->hit_object == sc.box);
    IVP_ASSERT_NEAR(r.hits[0]->hit_distance, 4.0, 1e-5);
    IVP_ASSERT_TRUE(r.hits[1]->hit_object == sc.tri);
    IVP_ASSERT_NEAR(r.hits[1]->hit_distance, 10.0, 1e-5);
    IVP_ASSERT_NEAR(r.hits[1]->hit_normal.k[2], 1.0, 1e-6);
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(r.hits[1]->hit_ledge), 2);

    /* legacy tri_below_long: the two sided triangle is hit from below, the
     * normal is flipped towards the ray start: tri dist=5 n=(0 0 -1),
     * box dist=9 n=(0 0 -1) */
    cast(&sc, 0.1, 0.2, -10, 0, 0, 1, 20, NULL, 0, &r);
    IVP_ASSERT_EQ(r.n, 2);
    IVP_ASSERT_TRUE(r.hits[0]->hit_object == sc.tri);
    IVP_ASSERT_NEAR(r.hits[0]->hit_distance, 5.0, 1e-5);
    IVP_ASSERT_NEAR(r.hits[0]->hit_normal.k[2], -1.0, 1e-6);
    IVP_ASSERT_NEAR(r.hits[1]->hit_distance, 9.0, 1e-5);

    /* legacy tri_below (too short) hits=0 */
    cast(&sc, 0.1, 0.2, -10, 0, 0, 1, 4.5, NULL, 0, &r);
    IVP_ASSERT_EQ(r.n, 0);

    /* legacy soup3_line: one hit per terminal ledge: 4.5 6.5 8.5 */
    cast(&sc, -5, 20.1, 0.2, 1, 0, 0, 20, NULL, 0, &r);
    IVP_ASSERT_EQ(r.n, 3);
    IVP_ASSERT_NEAR(r.hits[0]->hit_distance, 4.5, 1e-5);
    IVP_ASSERT_NEAR(r.hits[1]->hit_distance, 6.5, 1e-5);
    IVP_ASSERT_NEAR(r.hits[2]->hit_distance, 8.5, 1e-5);
    IVP_ASSERT_TRUE(r.hits[0]->hit_ledge != r.hits[1]->hit_ledge);

    /* legacy ball dist=8.543978 n=(0.2 0.970681 0.133333) (world space) */
    cast(&sc, 0.3, -10, 0.2, 0, -1, 0, 20, NULL, 0, &r);
    IVP_ASSERT_EQ(r.n, 1);
    IVP_ASSERT_TRUE(r.hits[0]->hit_object == sc.ball);
    IVP_ASSERT_NEAR(r.hits[0]->hit_distance, 8.543978, 1e-5);
    IVP_ASSERT_NEAR(r.hits[0]->hit_normal.k[0], 0.2, 1e-5);
    IVP_ASSERT_NEAR(r.hits[0]->hit_normal.k[1], 0.970681, 1e-5);
    IVP_ASSERT_NULL(r.hits[0]->hit_ledge);

    /* legacy ball_inside hits=0 */
    cast(&sc, 0.3, -20, 0.2, 0, -1, 0, 20, NULL, 0, &r);
    IVP_ASSERT_EQ(r.n, 0);

    /* legacy all_long: box 29, tilted 39.54171 */
    cast(&sc, -30, 0.1, 0.2, 1, 0, 0, 60, NULL, 0, &r);
    IVP_ASSERT_EQ(r.n, 2);
    IVP_ASSERT_NEAR(r.hits[0]->hit_distance, 29.0, 1e-4);
    IVP_ASSERT_NEAR(r.hits[1]->hit_distance, 39.54171, 1e-4);

    /* legacy tilted_side dist=4.136008 n=(0 1 0) */
    cast(&sc, 10.05, 5, 0.1, 0, -1, 0, 10, NULL, 0, &r);
    IVP_ASSERT_EQ(r.n, 1);
    IVP_ASSERT_NEAR(r.hits[0]->hit_distance, 4.136008, 1e-5);
    IVP_ASSERT_NEAR(r.hits[0]->hit_normal.k[1], 1.0, 1e-6);

    /* flags: every object is static */
    cast(&sc, 0.1, 0.2, 5, 0, 0, -1, 20, NULL, IVP_RAY_SOLVER_IGNORE_STATICS, &r);
    IVP_ASSERT_EQ(r.n, 0);
    cast(&sc, 0.1, 0.2, 5, 0, 0, -1, 20, NULL, IVP_RAY_SOLVER_IGNORE_MOVINGS, &r);
    IVP_ASSERT_EQ(r.n, 2);

    scene_free(&sc);
}

IVP_TEST(ray_min_and_group)
{
    scene_t sc;
    scene_init(&sc);

    ivp_ray_solver_template_t t;
    t.ray_start_point = ivp_vec3r(IVP_REAL(0.1), IVP_REAL(0.2), IVP_REAL(5.0));
    t.ray_normized_direction = ivp_vec3r(0, 0, -1);
    t.ray_length = IVP_REAL(20.0);
    t.ray_flags = IVP_RAY_SOLVER_ALL;
    ivp_ray_solver_min_t mn;
    ivp_ray_solver_min_init(&mn, &t);
    IVP_ASSERT_NULL(ivp_ray_solver_min_get_ray_hit(&mn));
    ivp_ray_solver_check_all_objects(&mn.base, sc.env);
    const ivp_ray_hit_t *h = ivp_ray_solver_min_get_ray_hit(&mn);
    IVP_ASSERT_NOT_NULL(h);
    IVP_ASSERT_TRUE(h->hit_object == sc.box);
    IVP_ASSERT_NEAR(h->hit_distance, 4.0, 1e-5);

    /* group of two short rays (car wheel style) */
    ivp_ray_solver_min_t a, b;
    t.ray_start_point = ivp_vec3r(IVP_REAL(-0.5), IVP_REAL(0.3), IVP_REAL(2.0));
    t.ray_length = IVP_REAL(1.5);
    ivp_ray_solver_min_init(&a, &t);
    t.ray_start_point = ivp_vec3r(IVP_REAL(0.5), IVP_REAL(0.3), IVP_REAL(2.0));
    t.ray_length = IVP_REAL(0.5);
    ivp_ray_solver_min_init(&b, &t);
    ivp_ray_solver_t *rs[2] = { &a.base, &b.base };
    ivp_ray_solver_group_t g;
    ivp_ray_solver_group_init(&g, 2, rs);
    ivp_ray_solver_group_check_all_objects(&g, sc.env);
    IVP_ASSERT_NOT_NULL(ivp_ray_solver_min_get_ray_hit(&a));
    IVP_ASSERT_NEAR(ivp_ray_solver_min_get_ray_hit(&a)->hit_distance, 1.0, 1e-5);
    IVP_ASSERT_NULL(ivp_ray_solver_min_get_ray_hit(&b)); /* too short */

    scene_free(&sc);
}

IVP_TEST(ray_object_space_helpers)
{
    ivp_compact_surface_t *s = ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    ivp_ray_input_t ray;
    ivp_ray_hit_t hit;
    ray.origin = ivp_vec3r(IVP_REAL(-10.0), IVP_REAL(0.3), IVP_REAL(-0.2));
    ray.direction = ivp_vec3r(1, 0, 0);
    ray.max_distance = IVP_REAL(20.0);
    IVP_ASSERT_EQ(ivp_ray_cast_surface(s, &ray, &hit), IVP_OK);
    IVP_ASSERT_NEAR(hit.hit_distance, 9.0, 1e-5);
    /* normalised, object space, outward */
    double nl = sqrt((double)(hit.hit_normal.k[0] * hit.hit_normal.k[0] +
                              hit.hit_normal.k[1] * hit.hit_normal.k[1] +
                              hit.hit_normal.k[2] * hit.hit_normal.k[2]));
    IVP_ASSERT_NEAR(nl, 1.0, 1e-6);
    IVP_ASSERT_NEAR(hit.hit_normal.k[0], -1.0, 1e-6);

    /* the exit face is never reported when starting inside */
    ray.origin = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.3), IVP_REAL(-0.2));
    IVP_ASSERT_NE(ivp_ray_cast_surface(s, &ray, &hit), IVP_OK);
    ivp_compact_surface_free(NULL, s);
}

IVP_TEST(ray_object_space_helpers_reject_invalid_arguments)
{
    ivp_compact_surface_t *s =
        ivp_build_box_surface(NULL, 1.0f, 1.0f, 1.0f);
    IVP_ASSERT_NOT_NULL(s);
    const ivp_compact_ledge_t *ledge = ivp_surface_get_single_convex(s);
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);
    ivp_ray_input_t ray;
    ray.origin = ivp_vec3r(IVP_REAL(-10.0), IVP_REAL(0.3), IVP_REAL(-0.2));
    ray.direction = ivp_vec3r(1, 0, 0);
    ray.max_distance = IVP_REAL(20.0);
    ivp_ray_hit_t hit;
    ivp_real_t distance = IVP_REAL(-1.0);

    IVP_ASSERT_EQ(ivp_ray_cast_surface(NULL, &ray, &hit),
                  IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_ray_cast_surface(s, NULL, &hit),
                  IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_ray_cast_surface(s, &ray, NULL),
                  IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_ray_cast_ledge(NULL, &ray, &hit),
                  IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_ray_cast_ledge(ledge, NULL, &hit),
                  IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_ray_cast_ledge(ledge, &ray, NULL),
                  IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_ray_test_triangle(NULL, tri, &ray, &distance),
                  IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_ray_test_triangle(ledge, NULL, &ray, &distance),
                  IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_ray_test_triangle(ledge, tri, NULL, &distance),
                  IVP_ERR_INVALID_ARG);
    IVP_ASSERT_EQ(ivp_ray_test_triangle(ledge, tri, &ray, NULL),
                  IVP_ERR_INVALID_ARG);

    ivp_vec3r_t zero = ivp_vec3r(0, 0, 0);
    IVP_ASSERT_FALSE(ivp_ray_test_sphere(NULL, &zero, &zero,
                                         IVP_REAL(1.0), &distance));
    IVP_ASSERT_FALSE(ivp_ray_test_sphere(&zero, NULL, &zero,
                                         IVP_REAL(1.0), &distance));
    IVP_ASSERT_FALSE(ivp_ray_test_sphere(&zero, &zero, NULL,
                                         IVP_REAL(1.0), &distance));
    IVP_ASSERT_FALSE(ivp_ray_test_sphere(&zero, &zero, &zero,
                                         IVP_REAL(-1.0), &distance));
    ivp_compact_surface_free(NULL, s);
}

IVP_TEST(sphere_solver_environment_matches_reference)
{
    scene_t sc;
    scene_init(&sc);
    /* legacy output per (query, depth 0..2) */
    static const struct {
        double q[4];
        const char *expect[3];
    } cases[] = {
        { {0, 0, 0, 0.5}, {"box", "box", "box"} },
        { {3, 0, 0, 1.5}, {"box", "box", "box"} },
        { {3, 0, 0, 1.0}, {"", "", ""} },
        { {2, 20, 0, 0.1}, {"soup3", "soup3", "soup3"} },
        { {1, 20.9, 0, 0.3}, {"soup3", "soup3", ""} },
        { {0, -18, 0, 0.6}, {"", "", ""} }, /* reference ball quirk */
        { {9.0, 0.3, 0, 0.3}, {"tilted", "tilted", "tilted"} },
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        for (int depth = 0; depth < 3; depth++) {
            ivp_sphere_solver_template_t st;
            st.center = ivp_vec3r((ivp_real_t)cases[i].q[0], (ivp_real_t)cases[i].q[1],
                                  (ivp_real_t)cases[i].q[2]);
            st.radius = (ivp_real_t)cases[i].q[3];
            st.max_traversal_depth = depth;
            ivp_object_t *out[8];
            int n = ivp_sphere_solver_check_environment(&st, sc.env, out, 8);
            const char *e = cases[i].expect[depth];
            if (e[0] == '\0') {
                IVP_ASSERT_EQ(n, 0);
            } else {
                IVP_ASSERT_EQ(n, 1);
                IVP_ASSERT_TRUE(strcmp(out[0]->name, e) == 0);
                IVP_ASSERT_TRUE(ivp_sphere_solver_check_object(&st, out[0]));
            }
        }
    }
    scene_free(&sc);
}


/* ── OV tree scene ──────────────────────────────────────────────────────────
 *
 * Environment queries walk the OV tree.  Expected values come from the
 * legacy engine on the same scene (IVP_Ray_Solver_Min_Hash / _Min /
 * IVP_Ray_Solver_Group::check_ray_group_against_all_objects_in_sim and
 * IVP_Sphere_Solver::check_sphere_against_environment, draining the
 * IVP_U_Min_Hash with find_min_elem / remove_min).  Many boxes share the
 * front face x = -1, so a ray along +x hits them at exactly the same
 * distance.  The normal build reports equal-distance hits in world object
 * order; the reference-internals build preserves the original OV traversal
 * order.  C, N (boxes) and L (ball) have collision detection disabled: they
 * are not in the OV tree and must never be reported.  P and R are phantoms
 * (made from static objects: IVP_MT_STATIC_PHANTOM, which
 * IVP_MTIS_IS_STATIC does not count as static), M and s3 are moving.
 */

enum { OV_STATIC = 0, OV_DYNAMIC = 1, OV_NOCOLL = 2, OV_PHANTOM = 3 };

#define OV_MAX 32

typedef struct ov_scene {
    ivp_environment_t     *env;
    ivp_compact_surface_t *surf[OV_MAX];
    ivp_surman_polygon_t   sm[OV_MAX];
    ivp_surman_ball_t      smb[OV_MAX];
    int                    n_surf, n_ball;
} ov_scene_t;

static ivp_object_t *ov_make(ov_scene_t *sc, const char *name, int kind, ivp_object_type_t type,
                             const ivp_surman_t *sm, double px, double py, double pz)
{
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = type;
    t.name = name;
    t.physical_unmoveable = kind != OV_DYNAMIC;
    t.mass = kind == OV_DYNAMIC ? IVP_REAL(1.0) : IVP_REAL(0.0);
    t.position = ivp_vec3r((ivp_real_t)px, (ivp_real_t)py, (ivp_real_t)pz);
    t.orientation = ivp_quatr_make(0, 0, 0, 1);
    t.enable_collision_detection = kind != OV_NOCOLL;
    ivp_object_t *o = ivp_environment_create_object(sc->env, &t, sm);
    if (o && kind == OV_PHANTOM) ivp_object_convert_to_phantom(o, NULL);
    return o;
}

static ivp_object_t *ov_box(ov_scene_t *sc, const char *name, double h, double px, double py,
                            double pz, int kind)
{
    double c[24];
    box_coords(c, h, h, h, 0, 0, 0);
    int i = sc->n_surf++;
    sc->surf[i] = ivp_build_convex_surface_d(NULL, c, 8);
    ivp_surman_polygon_init(&sc->sm[i], sc->surf[i]);
    return ov_make(sc, name, kind, IVP_OBJECT_POLYGON, &sc->sm[i].base, px, py, pz);
}

static ivp_object_t *ov_ball(ov_scene_t *sc, const char *name, double r, double px, double py,
                             double pz, int kind)
{
    int i = sc->n_ball++;
    ivp_surman_ball_init(&sc->smb[i], (float)r);
    return ov_make(sc, name, kind, IVP_OBJECT_BALL, &sc->smb[i].base, px, py, pz);
}

static void ov_scene_init(ov_scene_t *sc)
{
    static const char *small_names[6] = { "s0", "s1", "s2", "s3", "s4", "s5" };
    memset(sc, 0, sizeof(*sc));
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    sc->env = ivp_environment_create(&cfg);
    ov_box(sc, "A", 1.0, 0, 0, 0, OV_STATIC);
    ov_box(sc, "B", 1.0, 0, 0, 0, OV_STATIC);
    ov_box(sc, "C", 1.0, 0, 0, 0, OV_NOCOLL);
    ov_box(sc, "D", 0.5, -0.5, 0, 0, OV_STATIC);
    ov_box(sc, "E", 2.0, 1, 0, 0, OV_STATIC);
    ov_box(sc, "F", 0.25, -0.75, 0, 0.25, OV_STATIC);
    ov_box(sc, "P", 1.0, 0, 0, 0, OV_PHANTOM);
    ov_box(sc, "M", 1.0, 0, 0, 0, OV_DYNAMIC);
    ov_box(sc, "N", 0.5, -0.5, 0, 0, OV_NOCOLL);
    ov_box(sc, "G", 1.0, 30, 0, 0, OV_STATIC);
    ov_box(sc, "H", 1.0, 30, 0, 0, OV_STATIC);
    ov_ball(sc, "K", 1.0, 0, 0, 0, OV_STATIC);
    ov_ball(sc, "L", 1.0, 0, 0, 0, OV_NOCOLL);
    for (int i = 0; i < 6; i++) {
        ov_box(sc, small_names[i], 0.125, 4.0 + 2.0 * i, 0, 0, (i == 3) ? OV_DYNAMIC : OV_STATIC);
    }
    ov_box(sc, "Q", 4.0, 0, 0, -40, OV_STATIC);
    ov_box(sc, "R", 4.0, 0, 0, -40, OV_PHANTOM);
    ov_box(sc, "T", 0.25, -12.75, 0.75, 0.75, OV_STATIC);
}

static void ov_scene_free(ov_scene_t *sc)
{
    ivp_environment_destroy(sc->env);
    for (int i = 0; i < sc->n_surf; i++) ivp_compact_surface_free(NULL, sc->surf[i]);
}

/* Ray template like the reference test driver: IVP_U_Point::normize()
 * (isqrt with 3 resolution steps), then stored as IVP_U_Float_Point. */
static void ov_template(ivp_ray_solver_template_t *t, double sx, double sy, double sz, double dx,
                        double dy, double dz, double len, unsigned flags)
{
    double f = ivp_isqrt_double(dx * dx + dy * dy + dz * dz, 3);
    t->ray_start_point = ivp_vec3r((ivp_real_t)sx, (ivp_real_t)sy, (ivp_real_t)sz);
    t->ray_normized_direction = ivp_vec3r((ivp_real_t)(dx * f), (ivp_real_t)(dy * f), (ivp_real_t)(dz * f));
    t->ray_length = (ivp_real_t)len;
    t->ray_flags = flags;
}

/* "name name ..." of the sorted hits */
static const char *sorted_names(const ivp_ray_solver_min_hash_t *mh, char *buf, size_t cap)
{
    const ivp_ray_hit_t *hits[IVP_MAX_NUM_RAY_HITS];
    int n = ivp_ray_solver_min_hash_get_sorted(mh, hits, IVP_MAX_NUM_RAY_HITS);
    size_t len = 0;
    buf[0] = '\0';
    for (int i = 0; i < n && len + 1 < cap; i++) {
        int w = snprintf(buf + len, cap - len, "%s%s", i ? " " : "", hits[i]->hit_object->name);
        if (w < 0) break;
        len += (size_t)w;
    }
    return buf;
}

static bool names_equal(const char *got, const char *want, const char *what)
{
    if (strcmp(got, want) == 0) return true;
    fprintf(stderr, "  %s: got \"%s\", expected \"%s\"\n", what, got, want);
    return false;
}

static ivp_ray_solver_min_hash_t s_ov_mh[4];

typedef struct ov_cast_case {
    const char *name;
    double      ray[7]; /* start, direction (not normalised), length */
    unsigned    flags;
    const char *hits;
    const char *min_hit; /* "" = none */
    double      min_dist;
} ov_cast_case_t;

#if IVP_REFERENCE_INTERNAL_STATE
#define OV_PX_HITS "E M P D B A F K"
#define OV_PX_NOPH_HITS "E M D B A F K"
#define OV_PX_NOMOV_HITS "E D B A F K"
#define OV_PX_LONG_HITS "E M P D B A F K s0 s1 s2 s3 s4 s5 H G"
#define OV_DIAG_HITS "E M P B A K D"
#define OV_PX_MIN "E"
#define OV_PX_NOPH_MIN "E"
#define OV_PX_NOMOV_MIN "E"
#define OV_PX_LONG_MIN "E"
#define OV_CB_FILTERED_HITS "E M D B F K"
#define OV_GROUP_HITS_0 "E M P D B A F K"
#define OV_GROUP_HITS_1 "E M P D B A K"
#define OV_GROUP_NOPH_HITS_0 "E M D B A F K"
#define OV_GROUP_NOPH_HITS_1 "E M D B A K"
#define OV_SPHERE_NEAR_HITS "E A B D P M K F"
#define OV_SPHERE_WIDE_HITS \
    "E A B D P M K F s0 s1 s2 s3 s4 s5 T Q R"
#else
#define OV_PX_HITS "M P F E D B A K"
#define OV_PX_NOPH_HITS "M F E D B A K"
#define OV_PX_NOMOV_HITS "F E D B A K"
#define OV_PX_LONG_HITS "M P F E D B A K s0 s1 s2 s3 s4 s5 H G"
#define OV_DIAG_HITS "M P E B A K D"
#define OV_PX_MIN "M"
#define OV_PX_NOPH_MIN "M"
#define OV_PX_NOMOV_MIN "F"
#define OV_PX_LONG_MIN "M"
#define OV_CB_FILTERED_HITS "M F E D B K"
#define OV_GROUP_HITS_0 "M P F E D B A K"
#define OV_GROUP_HITS_1 "M P E D B A K"
#define OV_GROUP_NOPH_HITS_0 "M F E D B A K"
#define OV_GROUP_NOPH_HITS_1 "M E D B A K"
#define OV_SPHERE_NEAR_HITS "A B D E F P M K"
#define OV_SPHERE_WIDE_HITS \
    "A B D E F P M K s0 s1 s2 s3 s4 s5 Q R T"
#endif

IVP_TEST(ray_env_queries_walk_ov_tree)
{
    ov_scene_t sc;
    ov_scene_init(&sc);
    static const ov_cast_case_t cases[] = {
        { "px", { -10, 0.1, 0.2, 1, 0, 0, 20 }, IVP_RAY_SOLVER_ALL, OV_PX_HITS, OV_PX_MIN, 9.0 },
        { "px_noph", { -10, 0.1, 0.2, 1, 0, 0, 20 }, IVP_RAY_SOLVER_IGNORE_PHANTOMS, OV_PX_NOPH_HITS, OV_PX_NOPH_MIN, 9.0 },
        { "px_nomov", { -10, 0.1, 0.2, 1, 0, 0, 20 }, IVP_RAY_SOLVER_IGNORE_MOVINGS, OV_PX_NOMOV_HITS, OV_PX_NOMOV_MIN, 9.0 },
        { "px_nostat", { -10, 0.1, 0.2, 1, 0, 0, 20 }, IVP_RAY_SOLVER_IGNORE_STATICS, "M P", "M", 9.0 },
        { "px_long", { -20, 0.1, 0.05, 1, 0, 0, 70 }, IVP_RAY_SOLVER_ALL,
          OV_PX_LONG_HITS, OV_PX_LONG_MIN, 19.0 },
        { "my", { 0.1, 10, 0.2, 0, -1, 0, 20 }, IVP_RAY_SOLVER_ALL, "E M P B A K", "E", 8.0 },
        { "mz", { 0.1, 0.2, 10, 0, 0, -1, 80 }, IVP_RAY_SOLVER_ALL, "E M P B A K R Q", "E", 8.0 },
        { "mz_noph", { 0.1, 0.2, 10, 0, 0, -1, 80 }, IVP_RAY_SOLVER_IGNORE_PHANTOMS, "E M B A K Q", "E", 8.0 },
        /* not exactly representable distances: the min hash is keyed by the
         * unrounded double distance while IVP_Ray_Solver_Min compares with
         * its float min_dist, so the closest hit is A, not the first in the
         * min hash (reference behaviour) */
        { "diag", { -10, -7, 3, 1, 0.7, -0.3, 30 }, IVP_RAY_SOLVER_ALL, OV_DIAG_HITS, "A", 11.3128252 },
        { "miss", { -10, 50, 0, 1, 0, 0, 20 }, IVP_RAY_SOLVER_ALL, "", "", 0.0 },
    };
    char buf[256];
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        const ov_cast_case_t *c = &cases[i];
        ivp_ray_solver_template_t t;
        ov_template(&t, c->ray[0], c->ray[1], c->ray[2], c->ray[3], c->ray[4], c->ray[5], c->ray[6], c->flags);
        ivp_ray_solver_min_hash_init(&s_ov_mh[0], &t);
        ivp_ray_solver_check_all_objects(&s_ov_mh[0].base, sc.env);
        IVP_ASSERT_TRUE(names_equal(sorted_names(&s_ov_mh[0], buf, sizeof(buf)), c->hits, c->name));

        ivp_ray_solver_min_t mn;
        ivp_ray_solver_min_init(&mn, &t);
        ivp_ray_solver_check_all_objects(&mn.base, sc.env);
        const ivp_ray_hit_t *h = ivp_ray_solver_min_get_ray_hit(&mn);
        if (c->min_hit[0] == '\0') {
            IVP_ASSERT_NULL(h);
        } else {
            IVP_ASSERT_NOT_NULL(h);
            IVP_ASSERT_TRUE(names_equal(h->hit_object->name, c->min_hit, c->name));
            IVP_ASSERT_NEAR(h->hit_distance, c->min_dist, 1e-5);
        }
    }

    /* distances of the px ray (legacy: E..F 9, K 9.02532101) */
    {
        ivp_ray_solver_template_t t;
        ov_template(&t, -10, 0.1, 0.2, 1, 0, 0, 20, IVP_RAY_SOLVER_ALL);
        ivp_ray_solver_min_hash_init(&s_ov_mh[0], &t);
        ivp_ray_solver_check_all_objects(&s_ov_mh[0].base, sc.env);
        const ivp_ray_hit_t *hits[IVP_MAX_NUM_RAY_HITS];
        int n = ivp_ray_solver_min_hash_get_sorted(&s_ov_mh[0], hits, IVP_MAX_NUM_RAY_HITS);
        IVP_ASSERT_EQ(n, 8);
        for (int i = 0; i < 7; i++) IVP_ASSERT_NEAR(hits[i]->hit_distance, 9.0, 1e-6);
        IVP_ASSERT_NEAR(hits[7]->hit_distance, 9.02532101, 1e-5);
        IVP_ASSERT_NULL(hits[7]->hit_ledge); /* ball */
    }
    ov_scene_free(&sc);
}

static bool phantom_named_a(void *user, const ivp_object_t *object)
{
    (void)user;
    return strcmp(object->name, "A") == 0;
}

IVP_TEST(ray_env_skips_collision_disabled_and_phantoms)
{
    ov_scene_t sc;
    ov_scene_init(&sc);
    char buf[256];
    ivp_ray_solver_template_t t;
    ov_template(&t, -10, 0.1, 0.2, 1, 0, 0, 20, IVP_RAY_SOLVER_ALL);

    /* the collision disabled boxes C / N and ball L are hit when tested
     * directly, but are never visited by the environment query */
    for (int i = 0; i < ivp_environment_get_object_count(sc.env); i++) {
        ivp_object_t *o = ivp_environment_get_object(sc.env, i);
        if (strcmp(o->name, "C") && strcmp(o->name, "N") && strcmp(o->name, "L")) continue;
        IVP_ASSERT_NULL(o->ov_element);
        ivp_ray_solver_min_hash_init(&s_ov_mh[0], &t);
        ivp_ray_solver_check_object(&s_ov_mh[0].base, o);
        IVP_ASSERT_EQ(s_ov_mh[0].n_hits, 1);
    }

    /* phantoms come from ivp_object_convert_to_phantom (controller_phantom) */
    ivp_ray_solver_min_hash_init(&s_ov_mh[0], &t);
    ivp_ray_solver_check_all_objects(&s_ov_mh[0].base, sc.env);
    IVP_ASSERT_TRUE(names_equal(sorted_names(&s_ov_mh[0], buf, sizeof(buf)), OV_PX_HITS, "all"));
    t.ray_flags = IVP_RAY_SOLVER_IGNORE_PHANTOMS;
    ivp_ray_solver_min_hash_init(&s_ov_mh[0], &t);
    ivp_ray_solver_check_all_objects(&s_ov_mh[0].base, sc.env);
    IVP_ASSERT_TRUE(names_equal(sorted_names(&s_ov_mh[0], buf, sizeof(buf)), OV_PX_NOPH_HITS, "noph"));

    /* the optional user classifier adds to the phantom controller test */
    ivp_ray_solver_min_hash_init(&s_ov_mh[0], &t);
    s_ov_mh[0].base.is_phantom = phantom_named_a;
    ivp_ray_solver_check_all_objects(&s_ov_mh[0].base, sc.env);
    IVP_ASSERT_TRUE(names_equal(sorted_names(&s_ov_mh[0], buf, sizeof(buf)), OV_CB_FILTERED_HITS, "noph+cb"));
    /* ... and is ignored without the flag */
    t.ray_flags = IVP_RAY_SOLVER_ALL;
    ivp_ray_solver_min_hash_init(&s_ov_mh[0], &t);
    s_ov_mh[0].base.is_phantom = phantom_named_a;
    ivp_ray_solver_check_all_objects(&s_ov_mh[0].base, sc.env);
    IVP_ASSERT_TRUE(names_equal(sorted_names(&s_ov_mh[0], buf, sizeof(buf)), OV_PX_HITS, "cb only"));
    ov_scene_free(&sc);
}

IVP_TEST(ray_min_hash_order_matches_min_hash_container)
{
    ov_scene_t sc;
    ov_scene_init(&sc);
    static const double rays[][7] = {
        { -10, 0.1, 0.2, 1, 0, 0, 20 },
        { -20, 0.1, 0.05, 1, 0, 0, 70 },
        { -10, -7, 3, 1, 0.7, -0.3, 30 },
        { 0.1, 0.2, 10, 0, 0, -1, 80 },
    };
    for (size_t r = 0; r < sizeof(rays) / sizeof(rays[0]); r++) {
        ivp_ray_solver_template_t t;
        ov_template(&t, rays[r][0], rays[r][1], rays[r][2], rays[r][3], rays[r][4], rays[r][5], rays[r][6],
                    IVP_RAY_SOLVER_ALL);
        ivp_ray_solver_min_hash_init(&s_ov_mh[0], &t);
        ivp_ray_solver_check_all_objects(&s_ov_mh[0].base, sc.env);
        const ivp_ray_hit_t *hits[IVP_MAX_NUM_RAY_HITS];
        int n = ivp_ray_solver_min_hash_get_sorted(&s_ov_mh[0], hits, IVP_MAX_NUM_RAY_HITS);
        IVP_ASSERT_TRUE(n > 0);

        /* draining the IVP_U_Min_Hash port (size 8 like the reference)
         * yields the same sequence */
        ivp_min_hash_t h;
        IVP_ASSERT_EQ(ivp_min_hash_init(&h, NULL, 8), IVP_OK);
        IVP_ASSERT_EQ(ivp_ray_solver_min_hash_fill_min_hash(&s_ov_mh[0], &h), IVP_OK);
        IVP_ASSERT_EQ(ivp_min_hash_count(&h), n);
        bool same = true;
        for (int i = 0; i < n; i++) {
            if (ivp_min_hash_find_min_elem(&h) != (const void *)hits[i]) same = false;
            ivp_min_hash_remove_min(&h);
        }
        ivp_min_hash_destroy(&h);
        IVP_ASSERT_TRUE(same);

        /* capacity smaller than the hit count */
        const ivp_ray_hit_t *first[2];
        IVP_ASSERT_EQ(ivp_ray_solver_min_hash_get_sorted(&s_ov_mh[0], first, 2), n);
        IVP_ASSERT_TRUE(first[0] == hits[0]);
    }
    ov_scene_free(&sc);
}

IVP_TEST(ray_group_env_query_walks_ov_tree)
{
    ov_scene_t sc;
    ov_scene_init(&sc);
    static const struct {
        const char *name;
        int         n;
        double      rays[3][7];
        unsigned    flags;
        const char *hits[3];
    } cases[] = {
        { "g", 3, { { -1.5, 0.1, 0.2, 1, 0, 0, 1.0 }, { -1.5, -0.3, 0.1, 1, 0, 0, 1.0 }, { -1.25, 0.2, -0.2, 1, 0, 0, 0.5 } },
          IVP_RAY_SOLVER_ALL, { OV_GROUP_HITS_0, OV_GROUP_HITS_1, OV_GROUP_HITS_1 } },
        { "g_noph", 3, { { -1.5, 0.1, 0.2, 1, 0, 0, 1.0 }, { -1.5, -0.3, 0.1, 1, 0, 0, 1.0 }, { -1.25, 0.2, -0.2, 1, 0, 0, 0.5 } },
          IVP_RAY_SOLVER_IGNORE_PHANTOMS, { OV_GROUP_NOPH_HITS_0, OV_GROUP_NOPH_HITS_1, OV_GROUP_NOPH_HITS_1 } },
        { "gs", 2, { { 3.5, 0.05, 0.02, 1, 0, 0, 3.0 }, { 3.5, -0.05, -0.02, 1, 0, 0, 5.0 } },
          IVP_RAY_SOLVER_ALL, { "s0 s1", "s0 s1 s2", "" } },
    };
    static const double k_dist[3] = { 0.52532059, 0.551316679, 0.290833682 };
    char buf[256];
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        ivp_ray_solver_t *rs[3];
        for (int i = 0; i < cases[c].n; i++) {
            ivp_ray_solver_template_t t;
            const double *r = cases[c].rays[i];
            ov_template(&t, r[0], r[1], r[2], r[3], r[4], r[5], r[6], cases[c].flags);
            ivp_ray_solver_min_hash_init(&s_ov_mh[i], &t);
            rs[i] = &s_ov_mh[i].base;
        }
        ivp_ray_solver_group_t g;
        ivp_ray_solver_group_init(&g, cases[c].n, rs);
        ivp_ray_solver_group_check_all_objects(&g, sc.env);
        for (int i = 0; i < cases[c].n; i++) {
            IVP_ASSERT_TRUE(names_equal(sorted_names(&s_ov_mh[i], buf, sizeof(buf)), cases[c].hits[i],
                                        cases[c].name));
        }
        if (c == 0) {
            for (int i = 0; i < 3; i++) {
                const ivp_ray_hit_t *hits[IVP_MAX_NUM_RAY_HITS];
                int n = ivp_ray_solver_min_hash_get_sorted(&s_ov_mh[i], hits, IVP_MAX_NUM_RAY_HITS);
                IVP_ASSERT_NEAR(hits[0]->hit_distance, i == 2 ? 0.25 : 0.5, 1e-6);
                IVP_ASSERT_NEAR(hits[n - 1]->hit_distance, k_dist[i], 1e-5);
            }
        }
    }
    ov_scene_free(&sc);
}

IVP_TEST(sphere_env_query_walks_ov_tree)
{
    ov_scene_t sc;
    ov_scene_init(&sc);
    /* Collision-disabled objects (C, N, L) are never reported; phantoms are
     * reported like other objects.  Results are identical for
     * max_traversal_depth 0..2. */
    static const struct {
        double q[4];
        const char *expect;
    } cases[] = {
        { { 0, 0, 0, 0.5 }, OV_SPHERE_NEAR_HITS },
        { { -1.2, 0.1, 0.2, 0.3 }, OV_SPHERE_NEAR_HITS },
        { { 30.5, 0, 0, 2.0 }, "G H" },
        { { 10.0, 0, 0, 0.2 }, "s3" },
        { { 0, 0, -20, 30 }, OV_SPHERE_WIDE_HITS },
        { { 0, 100, 0, 1 }, "" },
    };
    char buf[256];
    for (int depth = 0; depth < 3; depth++) {
        for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
            ivp_sphere_solver_template_t st;
            st.center = ivp_vec3r((ivp_real_t)cases[i].q[0], (ivp_real_t)cases[i].q[1], (ivp_real_t)cases[i].q[2]);
            st.radius = (ivp_real_t)cases[i].q[3];
            st.max_traversal_depth = depth;
            ivp_object_t *out[OV_MAX];
            int n = ivp_sphere_solver_check_environment(&st, sc.env, out, OV_MAX);
            size_t len = 0;
            buf[0] = '\0';
            for (int k = 0; k < n && k < OV_MAX; k++) {
                len += (size_t)snprintf(buf + len, sizeof(buf) - len, "%s%s", k ? " " : "", out[k]->name);
            }
            IVP_ASSERT_TRUE(names_equal(buf, cases[i].expect, "sphere"));
            /* capacity limit: the total is still returned */
            if (n > 2) IVP_ASSERT_EQ(ivp_sphere_solver_check_environment(&st, sc.env, out, 2), n);
        }
    }
    ov_scene_free(&sc);
}

int main(void)
{
    IVP_REGISTER(ray_reference_object_hits);
    IVP_REGISTER(ray_reference_world_queries);
    IVP_REGISTER(ray_min_and_group);
    IVP_REGISTER(ray_object_space_helpers);
    IVP_REGISTER(ray_object_space_helpers_reject_invalid_arguments);
    IVP_REGISTER(sphere_solver_environment_matches_reference);
    IVP_REGISTER(ray_env_queries_walk_ov_tree);
    IVP_REGISTER(ray_env_skips_collision_disabled_and_phantoms);
    IVP_REGISTER(ray_min_hash_order_matches_min_hash_container);
    IVP_REGISTER(ray_group_env_query_walks_ov_tree);
    IVP_REGISTER(sphere_env_query_walks_ov_tree);
    IVP_RUN_ALL();
}
