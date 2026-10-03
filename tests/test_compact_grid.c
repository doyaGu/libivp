/* test_compact_grid.c — IVP_GridBuilder_Array / IVP_SurfaceManager_Grid parity.
 *
 * Expected values printed by the legacy engine for the same 9 x 7
 * heightfield (convert_array_to_compact_grid + IVP_SurfaceManager_Grid).
 */
#include "ivp_test.h"

#include <stddef.h>
#include <stdint.h>

#include <ivp/base/ivp_assert.h>
#include <ivp/builder/ivp_build_grid.h>
#include <ivp/geometry/ivp_compact_grid.h>
#include <ivp/geometry/ivp_surman_grid.h>
#include <ivp/geometry/ivp_compact_ledge_geom.h>
#include <ivp/collision/ivp_ray_solver.h>

#define GF_ROWS 9
#define GF_COLS 7

static void make_field(float *h)
{
    for (int r = 0; r < GF_ROWS; r++) {
        for (int c = 0; c < GF_COLS; c++) {
            float v = 0.0f;
            if (r >= 3 && r <= 5 && c >= 2) v = 0.6f * (float)sin(0.9 * c + 0.4 * r);
            if (r == 7) v = 0.25f * (float)c;
            if (r == 1 && c == 3) v = 1.0f;
            h[r * GF_COLS + c] = v;
        }
    }
}

static void templ_a(ivp_template_compact_grid_t *t)
{
    ivp_template_compact_grid_init(t);
    t->row_info.n_points = GF_ROWS;
    t->row_info.maps_to = 0;
    t->column_info.n_points = GF_COLS;
    t->column_info.maps_to = 2;
    t->height_maps_to = 1;
    t->grid_field_size = 1.0f;
}

typedef struct lst {
    const ivp_compact_ledge_t *l[256];
    int n;
} lst_t;

static void add(void *c, const ivp_compact_ledge_t *l)
{
    lst_t *x = (lst_t *)c;
    if (x->n < 256) x->l[x->n++] = l;
}

static int ledge_index(const ivp_compact_grid_t *g, const ivp_compact_ledge_t *l)
{
    for (int i = 0; i < g->n_compact_ledges; i++)
        if (ivp_compact_grid_get_ledge(g, i) == l) return i;
    return -1;
}

IVP_TEST(compact_grid_matches_reference)
{
    float h[GF_ROWS * GF_COLS];
    make_field(h);
    ivp_template_compact_grid_t t;
    templ_a(&t);
    ivp_compact_grid_t *g = ivp_gridbuild_array_to_compact_grid(NULL, &t, h);
    IVP_ASSERT_NOT_NULL(g);
    /* legacy: nrows=8 ncols=6 nledges=45 center=(4 0.450023 3) radius=5.020211 */
    IVP_ASSERT_EQ(g->n_rows, 8);
    IVP_ASSERT_EQ(g->n_columns, 6);
    IVP_ASSERT_EQ(g->n_compact_ledges, 45);
    IVP_ASSERT_NEAR(g->center[0], 4.0, 1e-6);
    IVP_ASSERT_NEAR(g->center[1], 0.450023, 1e-6);
    IVP_ASSERT_NEAR(g->center[2], 3.0, 1e-6);
    IVP_ASSERT_NEAR(g->radius, 5.020211, 1e-5);

    /* first grid elements (legacy: 0/0 0/0 0/1 1/1 2/2 2/2 3/3 3/3 4/4 4/5) */
    static const int16_t e0[10][2] = { {0, 0}, {0, 0}, {0, 1}, {1, 1}, {2, 2}, {2, 2}, {3, 3}, {3, 3}, {4, 4}, {4, 5} };
    const ivp_compact_grid_element_t *ge = ivp_compact_grid_get_elements(g);
    for (int i = 0; i < 10; i++) {
        IVP_ASSERT_EQ(ge[i].compact_ledge_index[0], e0[i][0]);
        IVP_ASSERT_EQ(ge[i].compact_ledge_index[1], e0[i][1]);
    }
    /* ledge0 is a 10 triangle stripe, ledge2 a 4 triangle square */
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(ivp_compact_grid_get_ledge(g, 0)), 10);
    IVP_ASSERT_EQ(ivp_ledge_get_n_triangles(ivp_compact_grid_get_ledge(g, 2)), 4);
    for (int i = 0; i < g->n_compact_ledges; i++) {
        const ivp_compact_ledge_t *l = ivp_compact_grid_get_ledge(g, i);
        IVP_ASSERT_TRUE(ivp_compact_ledge_check(l));
        IVP_ASSERT_EQ(ivp_ledge_get_is_compact(l), 0);
    }

    ivp_surman_compact_grid_t sm;
    ivp_surman_compact_grid_init(&sm, g);
    float mc[3], ri[3], c0[3] = { 0, 0, 0 }, r, d;
    ivp_surman_get_mass_center(&sm.base, mc);
    ivp_surman_get_rotation_inertia(&sm.base, ri);
    ivp_surman_get_radius_and_deviation(&sm.base, c0, &r, &d);
    /* legacy: mc=(4 0.450023 3) ri=25.20252 r0=10.04042 d0=10.04042 */
    IVP_ASSERT_NEAR(ri[0], 25.20252, 1e-4);
    IVP_ASSERT_NEAR(r, 10.04042, 1e-5);
    IVP_ASSERT_NEAR(d, 10.04042, 1e-5);
    IVP_ASSERT_NULL(ivp_surman_get_single_convex(&sm.base));

    /* legacy query results */
    static const double q[4][4] = { {1.0, 0.2, 1.0, 0.3}, {3.5, 0.0, 2.5, 0.8}, {-3.0, 0.0, -3.0, 0.5},
                                    {4.2, 0.1, 5.9, 0.05} };
    static const int expect_n[4] = { 2, 7, 0, 1 };
    static const int expect[4][7] = { {0, 3}, {6, 7, 8, 13, 14, 15, 20}, {0}, {25} };
    for (int k = 0; k < 4; k++) {
        lst_t res;
        res.n = 0;
        ivp_surman_get_all_ledges_within_radius(&sm.base, q[k], q[k][3], NULL, NULL, add, &res);
        IVP_ASSERT_EQ(res.n, expect_n[k]);
        for (int i = 0; i < res.n; i++) IVP_ASSERT_EQ(ledge_index(g, res.l[i]), expect[k][i]);
    }
    lst_t all;
    all.n = 0;
    ivp_surman_get_all_terminal_ledges(&sm.base, add, &all);
    IVP_ASSERT_EQ(all.n, 45);
    ivp_compact_grid_free(NULL, g);
}

IVP_TEST(compact_grid_inverted_axes_match_reference)
{
    float h[GF_ROWS * GF_COLS];
    make_field(h);
    ivp_template_compact_grid_t t;
    templ_a(&t);
    t.row_info.maps_to = 2;
    t.row_info.invert_axis = true;
    t.column_info.maps_to = 0;
    t.height_invert_axis = true;
    t.grid_field_size = 0.5f;
    t.position_origin_os[0] = 1.0f;
    t.position_origin_os[1] = 2.0f;
    t.position_origin_os[2] = 3.0f;
    ivp_compact_grid_t *g = ivp_gridbuild_array_to_compact_grid(NULL, &t, h);
    IVP_ASSERT_NOT_NULL(g);
    /* legacy: nledges=45 center=(2.5 1.549977 1) radius=3.715153 inv=2 */
    IVP_ASSERT_EQ(g->n_compact_ledges, 45);
    IVP_ASSERT_NEAR(g->center[0], 2.5, 1e-6);
    IVP_ASSERT_NEAR(g->center[1], 1.549977, 1e-6);
    IVP_ASSERT_NEAR(g->center[2], 1.0, 1e-6);
    IVP_ASSERT_NEAR(g->radius, 3.715153, 1e-5);
    IVP_ASSERT_NEAR(g->inv_grid_size, 2.0, 0.0);

    ivp_surman_compact_grid_t sm;
    ivp_surman_compact_grid_init(&sm, g);
    static const double q0[3] = { 1.0, 0.2, 1.0 };
    lst_t res;
    res.n = 0;
    ivp_surman_get_all_ledges_within_radius(&sm.base, q0, 0.3, NULL, NULL, add, &res);
    /* legacy query0: 12 20 */
    IVP_ASSERT_EQ(res.n, 2);
    IVP_ASSERT_EQ(ledge_index(g, res.l[0]), 12);
    IVP_ASSERT_EQ(ledge_index(g, res.l[1]), 20);
    ivp_compact_grid_free(NULL, g);
}

typedef struct ray_ctx {
    double best;
    int n;
} ray_ctx_t;

static void on_hit(void *listener, struct ivp_object *object, const ivp_compact_ledge_t *ledge,
                   const ivp_compact_triangle_t *tri, double dist, const double n[3])
{
    (void)object;
    (void)ledge;
    (void)tri;
    (void)n;
    ray_ctx_t *c = (ray_ctx_t *)listener;
    c->n++;
    if (dist < c->best) c->best = dist;
}

/* Binary layout of IVP_Compact_Grid (LP64): legacy convert_array_to_compact_grid
 * for this 6 x 4 field gives byte_size 2272, 13 ledges, the grid elements at
 * 240, the shared point array at 300 (directly after the 15 elements, only
 * 4-byte aligned) and the first ledge at 848; the FNV-1a hash is over the
 * legacy blob with the bytes it leaves uninitialised set to zero. */
static uint64_t fnv1a64(const unsigned char *p, size_t n)
{
    uint64_t h = 0xcbf29ce484222325ull;
    for (size_t i = 0; i < n; i++) {
        h ^= p[i];
        h *= 0x100000001b3ull;
    }
    return h;
}

IVP_TEST(compact_grid_binary_layout_matches_reference)
{
    float h[6 * 4];
    for (int r = 0; r < 6; r++)
        for (int c = 0; c < 4; c++) h[r * 4 + c] = 0.3f * (float)((r * 5 + c * 3) % 4) - 0.1f * (float)r;
    ivp_template_compact_grid_t t;
    ivp_template_compact_grid_init(&t);
    t.row_info.n_points = 6;
    t.row_info.maps_to = 2;
    t.row_info.invert_axis = true;
    t.column_info.n_points = 4;
    t.column_info.maps_to = 0;
    t.height_maps_to = 1;
    t.height_invert_axis = true;
    t.grid_field_size = 1.5f;
    t.position_origin_os[0] = 1.0f;
    t.position_origin_os[1] = 2.0f;
    t.position_origin_os[2] = -1.0f;
    ivp_compact_grid_t *g = ivp_gridbuild_array_to_compact_grid(NULL, &t, h);
    IVP_ASSERT_NOT_NULL(g);
    IVP_ASSERT_EQ(g->n_compact_ledges, 13);
    IVP_ASSERT_EQ(g->offset_grid_elements, 240);
    IVP_ASSERT_EQ((int)((const char *)ivp_ledge_get_point_array(ivp_compact_grid_get_ledge(g, 0)) - (const char *)g), 300);
    IVP_ASSERT_EQ(g->offset_compact_ledge_array[0], 848);
    IVP_ASSERT_EQ((int)g->byte_size, 2272);
    if (sizeof(void *) == 8) {
        IVP_ASSERT_EQ((int)offsetof(ivp_compact_grid_t, offset_compact_ledge_array), 176);
        IVP_ASSERT_TRUE(fnv1a64((const unsigned char *)g, (size_t)g->byte_size) == 0x0aeb1059d8309edbull);
    }
    ivp_compact_grid_free(NULL, g);
}

/* The grid's shared point array is only 4-byte aligned and byte_size is a
 * padded estimate.  The reference (063fceb, e0dd03a) asserts
 * (offset & 3) == 0 and used <= byte_size, so building a grid reports no
 * assertion failure. */
static int g_grid_assert_count;

static void grid_count_assert(const char *expr, const char *file, int line,
                              const char *func, const char *msg, void *userdata)
{
    (void)expr; (void)file; (void)line; (void)func; (void)msg; (void)userdata;
    g_grid_assert_count++;
}

IVP_TEST(compact_grid_build_reports_no_alignment_asserts)
{
    float h[6 * 4];
    for (int r = 0; r < 6; r++)
        for (int c = 0; c < 4; c++) h[r * 4 + c] = 0.3f * (float)((r * 5 + c * 3) % 4) - 0.1f * (float)r;
    ivp_template_compact_grid_t t;
    ivp_template_compact_grid_init(&t);
    t.row_info.n_points = 6;
    t.row_info.maps_to = 2;
    t.column_info.n_points = 4;
    t.column_info.maps_to = 0;
    t.height_maps_to = 1;
    t.grid_field_size = 1.5f;
    g_grid_assert_count = 0;
    ivp_set_assert_handler(grid_count_assert, NULL);
    ivp_compact_grid_t *g = ivp_gridbuild_array_to_compact_grid(NULL, &t, h);
    ivp_set_assert_handler(NULL, NULL);
    IVP_ASSERT_NOT_NULL(g);
    IVP_ASSERT_EQ(g_grid_assert_count, 0);
    ivp_compact_grid_free(NULL, g);
}

IVP_TEST(compact_grid_ray_hits_flat_area)
{
    float h[GF_ROWS * GF_COLS];
    make_field(h);
    ivp_template_compact_grid_t t;
    templ_a(&t);
    ivp_compact_grid_t *g = ivp_gridbuild_array_to_compact_grid(NULL, &t, h);
    IVP_ASSERT_NOT_NULL(g);
    ivp_surman_compact_grid_t sm;
    ivp_surman_compact_grid_init(&sm, g);
    /* vertical ray from y = 5 down onto the flat ground (height 0) at x=0.3, z=0.6 */
    ivp_ray_solver_template_t rt;
    rt.ray_start_point = ivp_vec3r(IVP_REAL(0.3), IVP_REAL(5.0), IVP_REAL(0.6));
    rt.ray_normized_direction = ivp_vec3r(0, -1, 0);
    rt.ray_length = IVP_REAL(10.0);
    rt.ray_flags = 0;
    ray_ctx_t ctx = { 1e30, 0 };
    ivp_ray_solver_t rs;
    ivp_ray_solver_init(&rs, &rt, on_hit, &ctx);
    ivp_surman_insert_all_ledges_hitting_ray(&sm.base, &rs, NULL);
    IVP_ASSERT_TRUE(ctx.n >= 1);
    IVP_ASSERT_NEAR(ctx.best, 5.0, 1e-5);
    ivp_compact_grid_free(NULL, g);
}

int main(void)
{
    IVP_REGISTER(compact_grid_matches_reference);
    IVP_REGISTER(compact_grid_inverted_axes_match_reference);
    IVP_REGISTER(compact_grid_ray_hits_flat_area);
    IVP_REGISTER(compact_grid_binary_layout_matches_reference);
    IVP_REGISTER(compact_grid_build_reports_no_alignment_asserts);
    IVP_RUN_ALL();
}
