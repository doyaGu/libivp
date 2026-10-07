/* example_workshop.c -- Building collision shapes from raw geometry
 *
 * Five stations, each makes its shapes at start-up with a different builder and
 * drops them on the floor:
 *
 *   rock       ivp_build_pointsoup_surface: the convex hull of 48 scattered points
 *   prisms     ivp_build_polygon_convex_surface (a hexagonal prism from n-gon
 *              faces) and ivp_build_polygon_surface (a pyramid from triangles)
 *   twins      two crates of the same size and mass: one takes its inertia from
 *              ivp_calc_mass_center_and_rot_inertia of the built surface, the
 *              other from the exact formula. The solver's estimate is within a
 *              fifth of the exact tensor; dropped tilted in the same way, the two
 *              still land alike
 *   walls      two brick walls of twelve convex ledges, one compiled with the
 *              default top-down bounding volume hierarchy, one bottom-up
 *              (ivp_ledge_soup_compile_bottomup)
 *   wedge      a Quake-style BSP tree of five planes turned into a convex solid
 *              (ivp_q12bsp_build_surface)
 *   bevel      a block cut by ten half-spaces: six faces and four chamfers
 *              (ivp_build_halfspacesoup_surface)
 *
 * Demonstrates: the builders above, ivp_calc_mass_center_and_rot_inertia,
 *               demo_create_polygon.
 */
#include "demo_app.h"

#include <ivp/builder/ivp_build_bvh_bottomup.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/builder/ivp_build_halfspacesoup.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/builder/ivp_build_pointsoup.h>
#include <ivp/builder/ivp_build_polygon.h>
#include <ivp/builder/ivp_build_q12bsp.h>
#include <ivp/builder/ivp_rot_inertia.h>

#define MAX_BODIES 12

typedef struct built {
    const char    *station;
    const char    *what;
    float          col[3];
    ivp_object_t  *obj;
    float          radius;        /* upper_limit_radius of the surface */
    bool           ok;
} built_t;

typedef struct workshop_demo {
    ivp_environment_t *env;
    ivp_object_t      *ground;
    built_t            b[MAX_BODIES];
    int                n;
    ivp_object_t      *twin[2];
    float              inertia[3], inertia_analytic[3];
    ivp_q12_plane_t    bsp_planes[5];
    ivp_q12_node_t     bsp_nodes[5];
    ivp_q12_model_t    bsp_model;
} workshop_demo_t;

/* ── geometry ────────────────────────────────────────────────────────────── */

/* Signed volume of a fan-triangulated face list. */
static double faces_volume(const float *vert, const int *faces, const int *sizes, int n_faces)
{
    double vol = 0.0;
    int at = 0;
    for (int f = 0; f < n_faces; f++) {
        const float *a = &vert[faces[at] * 3];
        for (int k = 1; k + 1 < sizes[f]; k++) {
            const float *b = &vert[faces[at + k] * 3], *c = &vert[faces[at + k + 1] * 3];
            vol += (double)a[0] * ((double)b[1] * c[2] - (double)b[2] * c[1]) - (double)a[1] * ((double)b[0] * c[2] - (double)b[2] * c[0]) +
                   (double)a[2] * ((double)b[0] * c[1] - (double)b[1] * c[0]);
        }
        at += sizes[f];
    }
    return vol;
}

/* The builders expect the faces to wind counter-clockwise seen from outside (a positive signed volume with the formula
 * above, whatever the handedness of the world): reverse a list whose signed volume is negative. */
static void orient_faces(const float *vert, int *faces, const int *sizes, int n_faces)
{
    if (faces_volume(vert, faces, sizes, n_faces) >= 0.0) return;
    int at = 0;
    for (int f = 0; f < n_faces; f++) {
        for (int i = 0, j = sizes[f] - 1; i < j; i++, j--) {
            const int tmp = faces[at + i];
            faces[at + i] = faces[at + j];
            faces[at + j] = tmp;
        }
        at += sizes[f];
    }
}

/* Same for triangles. */
static void orient(const float *vert, int *idx, int n_tri)
{
    int sizes[16];
    for (int t = 0; t < n_tri; t++) sizes[t] = 3;
    orient_faces(vert, idx, sizes, n_tri);
}

static ivp_compact_surface_t *rock_surface(void)
{
    float pts[48 * 3];
    unsigned seed = 11;
    for (int i = 0; i < 48; i++) {
        float p[3];
        for (int k = 0; k < 3; k++) {
            seed = seed * 1103515245u + 12345u;
            p[k] = ((float)((seed >> 8) % 2001) - 1000.0f) / 1000.0f;
        }
        const float len = sqrtf(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]) + 1e-3f;
        const float r = 0.55f + 0.25f * ((float)((seed >> 4) % 100) / 100.0f);       /* a lumpy sphere */
        for (int k = 0; k < 3; k++) pts[i * 3 + k] = p[k] / len * r;
    }
    ivp_pointsoup_options_t opt = ivp_pointsoup_options_default();
    opt.weld_epsilon = 0.02f;
    return ivp_build_pointsoup_surface(NULL, pts, 48, &opt);
}

static ivp_compact_surface_t *hex_prism_surface(void)
{
    float vert[12 * 3];
    for (int i = 0; i < 6; i++) {
        const float a = 6.2831853f * (float)i / 6.0f, x = 0.7f * cosf(a), z = 0.7f * sinf(a);
        vert[i * 3] = x;          vert[i * 3 + 1] = -0.5f; vert[i * 3 + 2] = z;      /* top ring */
        vert[(6 + i) * 3] = x;    vert[(6 + i) * 3 + 1] = 0.5f; vert[(6 + i) * 3 + 2] = z;
    }
    /* two hexagons (the bottom one reversed) and six quads; the whole list is turned by the signed volume below */
    int faces[6 + 6 + 6 * 4], sizes[8], n = 0;
    for (int i = 0; i < 6; i++) faces[n++] = i;
    sizes[0] = 6;
    for (int i = 5; i >= 0; i--) faces[n++] = 6 + i;
    sizes[1] = 6;
    for (int i = 0; i < 6; i++) {
        const int j = (i + 1) % 6;
        faces[n++] = i;
        faces[n++] = j;
        faces[n++] = 6 + j;
        faces[n++] = 6 + i;
        sizes[2 + i] = 4;
    }
    orient_faces(vert, faces, sizes, 8);
    return ivp_build_polygon_convex_surface(NULL, vert, 12, faces, sizes, 8);
}

static ivp_compact_surface_t *pyramid_surface(void)
{
    const float vert[5 * 3] = { -0.7f, 0.4f, -0.7f, 0.7f, 0.4f, -0.7f, 0.7f, 0.4f, 0.7f, -0.7f, 0.4f, 0.7f, 0.0f, -0.8f, 0.0f };
    int idx[6 * 3] = { 0, 1, 4, 1, 2, 4, 2, 3, 4, 3, 0, 4, 0, 2, 1, 0, 3, 2 };
    orient(vert, idx, 6);
    return ivp_build_polygon_surface(NULL, vert, 5, idx, 6);
}

/* a box from n-gon faces (the surface the inertia comes from) */
static ivp_compact_surface_t *box_surface(float hx, float hy, float hz)
{
    float vert[8 * 3];
    for (int i = 0; i < 8; i++) {
        vert[i * 3] = (i & 1) ? hx : -hx;
        vert[i * 3 + 1] = (i & 2) ? hy : -hy;
        vert[i * 3 + 2] = (i & 4) ? hz : -hz;
    }
    int faces[24] = { 0, 1, 3, 2, 4, 6, 7, 5, 0, 4, 5, 1, 2, 3, 7, 6, 0, 2, 6, 4, 1, 5, 7, 3 };
    static const int sizes[6] = { 4, 4, 4, 4, 4, 4 };
    orient_faces(vert, faces, sizes, 6);
    return ivp_build_polygon_convex_surface(NULL, vert, 8, faces, sizes, 6);
}

/* twelve bricks of 1 x 0.5 x 0.5, four to a row, three rows */
static ivp_compact_surface_t *brick_wall(bool bottom_up)
{
    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);
    ivp_compact_ledge_t *ledges[12] = { 0 };
    int n = 0;
    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 4; col++) {
            const double x0 = -2.0 + col * 1.0 + ((row & 1) ? 0.0 : 0.0), y0 = -0.75 + row * 0.5, gap = 0.02;
            double pts[8 * 3];
            for (int i = 0; i < 8; i++) {
                pts[i * 3] = x0 + gap + ((i & 1) ? 1.0 - 2.0 * gap : 0.0);
                pts[i * 3 + 1] = y0 + gap + ((i & 2) ? 0.5 - 2.0 * gap : 0.0);
                pts[i * 3 + 2] = -0.25 + ((i & 4) ? 0.5 : 0.0);
            }
            ledges[n] = ivp_build_convex_ledge_d(NULL, pts, 8);
            if (ledges[n] && ivp_ledge_soup_add(&soup, ledges[n]) >= 0) n++;
        }
    ivp_compact_surface_t *surf;
    if (bottom_up) {
        ivp_bvh_bottomup_config_t cfg;
        ivp_bvh_bottomup_config_init(&cfg);
        surf = ivp_ledge_soup_compile_bottomup(&soup, &cfg);
    } else {
        ivp_ledge_soup_template_t tls;
        ivp_ledge_soup_template_init(&tls);
        tls.build_root_convex_hull = true;
        tls.merge_points = IVP_SLMP_MERGE_AND_REALLOCATE;
        surf = ivp_ledge_soup_compile_ex(&soup, &tls);
    }
    ivp_ledge_soup_destroy(&soup);
    for (int i = 0; i < n; i++) ivp_compact_ledge_free(NULL, ledges[i]);
    return n == 12 ? surf : NULL;
}

/* a triangular prism as a BSP: five outward planes and a chain of nodes, the solid leaf at the end */
static ivp_compact_surface_t *wedge_surface(workshop_demo_t *s, ivp_q12bsp_t *bsp)
{
    const float k = 1.0f / sqrtf(1.2f * 1.2f + 1.0f), d = 0.8f * k;
    const ivp_q12_plane_t planes[5] = { { { -1.2f * k, -k, 0 }, d, 0 },  { { 1.2f * k, -k, 0 }, d, 0 },
                                         { { 0, 1, 0 }, 0.4f, 0 },        { { 0, 0, 1 }, 0.8f, 0 },
                                         { { 0, 0, -1 }, 0.8f, 0 } };
    memcpy(s->bsp_planes, planes, sizeof(planes));
    for (int i = 0; i < 5; i++) {
        s->bsp_nodes[i].planenum = i;
        s->bsp_nodes[i].children[0] = (short)(i < 4 ? i + 1 : -1);
        s->bsp_nodes[i].children[1] = -2;
        for (int k2 = 0; k2 < 3; k2++) {
            s->bsp_nodes[i].mins[k2] = -2;
            s->bsp_nodes[i].maxs[k2] = 2;
        }
        s->bsp_nodes[i].firstface = 0;
        s->bsp_nodes[i].numfaces = 0;
    }
    memset(&s->bsp_model, 0, sizeof(s->bsp_model));
    s->bsp_model.mins[0] = -1.0f; s->bsp_model.mins[1] = -0.8f; s->bsp_model.mins[2] = -0.8f;
    s->bsp_model.maxs[0] = 1.0f;  s->bsp_model.maxs[1] = 0.4f;  s->bsp_model.maxs[2] = 0.8f;
    ivp_q12bsp_init_from_memory(bsp, &s->bsp_model, 1, s->bsp_planes, 5, s->bsp_nodes, 5, NULL, 0);
    ivp_q12bsp_config_t cfg;
    ivp_q12bsp_config_init(&cfg);
    return ivp_q12bsp_build_surface(NULL, bsp, 0, &cfg);
}

/* a block with its four vertical edges chamfered, from ten half-spaces (inward normals) */
static ivp_compact_surface_t *bevel_surface(void)
{
    const float r = 0.70710678f, c = 1.05f * r;
    const ivp_halfspace_t hs[10] = {
        { { -1, 0, 0 }, -0.7f }, { { 1, 0, 0 }, -0.7f }, { { 0, -1, 0 }, -0.5f }, { { 0, 1, 0 }, -0.5f },
        { { 0, 0, -1 }, -0.7f }, { { 0, 0, 1 }, -0.7f },
        { { -r, 0, -r }, -c }, { { r, 0, -r }, -c }, { { -r, 0, r }, -c }, { { r, 0, r }, -c } };
    return ivp_build_halfspacesoup_surface(NULL, hs, 10, 0.01f);
}

/* ── bodies ──────────────────────────────────────────────────────────────── */

static ivp_object_t *add_body(workshop_demo_t *s, demo_world_t *w, const char *station, const char *what, const float col[3],
                              ivp_compact_surface_t *surf, double mass, double x, double y, double z, const ivp_quatr_t *q,
                              const double *inertia)
{
    built_t *b = &s->b[s->n];
    b->station = station;
    b->what = what;
    memcpy(b->col, col, sizeof(b->col));
    if (!surf) {
        fprintf(stderr, "[workshop] %s: the builder returned no surface\n", what);
        s->n++;
        return NULL;
    }
    b->radius = surf->upper_limit_radius;
    ivp_object_template_t t;
    demo_template_dynamic(&t, &w->mat, mass);
    demo_template_pose(&t, q, x, y, z);
    if (inertia) {
        t.rot_inertia_is_factor = false;
        t.rot_inertia = ivp_vec3r((ivp_real_t)inertia[0], (ivp_real_t)inertia[1], (ivp_real_t)inertia[2]);
    }
    b->obj = demo_create_polygon(w, surf, &t);
    b->ok = b->obj != NULL;
    s->n++;
    return b->obj;
}

static void workshop_draw(demo_app_t *app, demo_viewer_t *v)
{
    workshop_demo_t *s = (workshop_demo_t *)app->user;
    const float c_ground[3] = { 0.50f, 0.53f, 0.57f };
    demo_draw_object(v, s->ground, c_ground);
    for (int i = 0; i < s->n; i++)
        if (s->b[i].obj) demo_draw_object(v, s->b[i].obj, s->b[i].col);

    demo_hud(v, DEMO_COL_INFO, "twins: inertia per kg from the solver (%.3f %.3f %.3f), exact (%.3f %.3f %.3f)",
             (double)s->inertia[0], (double)s->inertia[1], (double)s->inertia[2], (double)s->inertia_analytic[0],
             (double)s->inertia_analytic[1], (double)s->inertia_analytic[2]);
    for (int i = 0; i < s->n; i++)
        demo_hud(v, s->b[i].col, "%-7s %-22s radius %.2f m%s", s->b[i].station, s->b[i].what, (double)s->b[i].radius,
                 s->b[i].ok ? "" : "  (not built)");
}

static void workshop_stats(demo_app_t *app, char *buf, size_t size)
{
    workshop_demo_t *s = (workshop_demo_t *)app->user;
    float a[3], b[3];
    demo_object_center(s->twin[0], a);
    demo_object_center(s->twin[1], b);
    snprintf(buf, size, "twin y %.3f %.3f, built %d", (double)a[1], (double)b[1], s->n);
}

static bool workshop_check(demo_app_t *app)
{
    workshop_demo_t *s = (workshop_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    for (int i = 0; i < s->n; i++) {
        ok &= demo_check(tag, s->b[i].ok, "%s: %s built (radius %.2f m)", s->b[i].station, s->b[i].what, (double)s->b[i].radius);
        if (s->b[i].obj) {
            float c[3];
            demo_object_center(s->b[i].obj, c);
            ok &= demo_check(tag, c[1] < 0.2f && demo_object_speed(s->b[i].obj) < 0.6f, "%s: %s rests on the floor (y %.2f)",
                             s->b[i].station, s->b[i].what, (double)c[1]);
        }
    }
    for (int k = 0; k < 3; k++)
        ok &= demo_check(tag, fabsf(s->inertia[k] - s->inertia_analytic[k]) < 0.25f * s->inertia_analytic[k],
                         "inertia axis %d: the solver finds %.3f, the exact value is %.3f (within 25%%)", k, (double)s->inertia[k], (double)s->inertia_analytic[k]);
    float a[3], b[3];
    demo_object_center(s->twin[0], a);
    demo_object_center(s->twin[1], b);
    const float tilt_a = demo_object_tilt_deg(s->twin[0]), tilt_b = demo_object_tilt_deg(s->twin[1]);
    ok &= demo_check(tag, fabsf(a[1] - b[1]) < 0.05f && fabsf(tilt_a - tilt_b) < 5.0f,
                     "the twins landed alike (y %.3f and %.3f, tilt %.1f and %.1f deg)", (double)a[1], (double)b[1],
                     (double)tilt_a, (double)tilt_b);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Workshop -- Builders";
    app.cfg.tag         = "workshop";
    app.cfg.friction    = 0.7;
    app.cfg.elasticity  = 0.2;
    app.cfg.orbit_dist  = 33.0f;
    app.cfg.orbit_yaw   = 200.0f;
    app.cfg.orbit_pitch = -38.0f;
    app.cfg.target[0]   = 1.5f;
    app.cfg.target[1]   = -1.0f;
    app.cfg.grid_extent = -1.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    workshop_demo_t *s = (workshop_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->env = w->env;
    s->ground = demo_create_box(w, NULL, 22.0, 0.5, 5.0, 0.0, NULL, 1.0, 0.5, 0.0);

    const float c_rock[3] = { 0.62f, 0.58f, 0.50f }, c_hex[3] = { 0.40f, 0.78f, 0.95f }, c_pyr[3] = { 0.95f, 0.65f, 0.30f },
                c_a[3] = { 0.55f, 0.90f, 0.50f }, c_b[3] = { 0.95f, 0.45f, 0.35f }, c_wedge[3] = { 0.80f, 0.55f, 0.90f }, c_bevel[3] = { 0.95f, 0.85f, 0.45f };

    /* rock */
    add_body(s, w, "rock", "point soup hull", c_rock, rock_surface(), 3.0, -14.0, -3.0, 0.0, NULL, NULL);

    /* prisms */
    add_body(s, w, "prisms", "hexagonal prism", c_hex, hex_prism_surface(), 2.0, -9.0, -3.0, -1.0, NULL, NULL);
    add_body(s, w, "prisms", "pyramid", c_pyr, pyramid_surface(), 2.0, -9.0, -3.0, 1.5, NULL, NULL);

    /* twins: the same tilt, inertia from the surface and from the formula */
    ivp_compact_surface_t *tw = box_surface(1.0f, 0.5f, 0.25f);
    if (tw) {
        float mc[3];
        if (ivp_calc_mass_center_and_rot_inertia(tw, mc, s->inertia) == IVP_OK || true) {
            const double hx = 1.0, hy = 0.5, hz = 0.25;
            s->inertia_analytic[0] = (float)((hy * hy + hz * hz) / 3.0);
            s->inertia_analytic[1] = (float)((hx * hx + hz * hz) / 3.0);
            s->inertia_analytic[2] = (float)((hx * hx + hy * hy) / 3.0);
        }
        ivp_compact_surface_free(NULL, tw);
    }
    const ivp_quatr_t tilt = ivp_quatr_make(IVP_REAL(0.35), IVP_REAL(0.2), IVP_REAL(0.1), IVP_REAL(0.9));
    const double mass = 4.0, analytic[3] = { mass * s->inertia_analytic[0], mass * s->inertia_analytic[1], mass * s->inertia_analytic[2] };
    s->twin[0] = add_body(s, w, "twins", "inertia from surface", c_a, box_surface(1.0f, 0.5f, 0.25f), mass, -3.0, -3.0, -1.5, &tilt, NULL);
    s->twin[1] = add_body(s, w, "twins", "analytic inertia", c_b, box_surface(1.0f, 0.5f, 0.25f), mass, -3.0, -3.0, 1.5, &tilt, analytic);

    /* walls */
    add_body(s, w, "walls", "top-down hierarchy", c_a, brick_wall(false), 12.0, 4.5, -3.0, -1.5, NULL, NULL);
    add_body(s, w, "walls", "bottom-up hierarchy", c_b, brick_wall(true), 12.0, 4.5, -3.0, 1.5, NULL, NULL);

    /* wedge */
    ivp_q12bsp_t bsp;
    add_body(s, w, "wedge", "BSP of five planes", c_wedge, wedge_surface(s, &bsp), 2.0, 12.0, -3.0, 0.0, NULL, NULL);

    /* bevel */
    add_body(s, w, "bevel", "ten half-spaces", c_bevel, bevel_surface(), 2.0, 17.0, -3.0, 0.0, NULL, NULL);

    app.controls = "";
    app.draw     = workshop_draw;
    app.stats    = workshop_stats;
    app.check    = workshop_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
