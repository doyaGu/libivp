/* example_concave.c -- Concave bodies from a triangle mesh
 *
 * A letter L, a plus, a cup, a star and a crescent are extruded from 2D
 * outlines into closed triangle meshes. ivp_build_concave_surface cuts every
 * mesh into convex parts with the GEOMPACK decomposition and builds one
 * compound surface from them, so the bodies keep their dents: six balls
 * dropped into the cup stay in it, which they would not in a convex hull.
 *
 *   F5 panel   "convex parts" draws the parts of every body in their own colour
 *
 * Demonstrates: ivp_build_concave_surface, ivp_convex_decompose parts,
 *               terminal ledges of a compound surface, demo_create_polygon.
 */
#include "demo_app.h"

#include <ivp/builder/ivp_build_concave.h>

#define N_SHAPES   5
#define N_BALLS    6
#define MAX_OUTLINE 32

typedef struct shape {
    const char *name;
    float       col[3];
    double      x;                 /* spawn position */
    double      half_z;            /* half thickness of the extrusion */
    double      mass;
    float       rdacc;             /* decomposition: min relative distance of a cut plane (0 = default 0.3) */
    int         n;                 /* outline points, counter-clockwise (x, up); 0 = the tray */
    float       pt[MAX_OUTLINE][2];
    /* filled by build */
    ivp_object_t *obj;
    int           n_parts;
    float         bound[3];        /* half extents of the bounding box */
} shape_t;

typedef struct concave_demo {
    ivp_object_t *ground;
    shape_t       shape[N_SHAPES];
    ivp_object_t *balls[N_BALLS];
    bool          show_parts;
    int           balls_in_cup;
} concave_demo_t;

enum { SH_L, SH_PLUS, SH_CUP, SH_STAR, SH_CRESCENT };

static void outline(shape_t *s, const float (*pts)[2], int n)
{
    s->n = n;
    for (int i = 0; i < n; i++) { s->pt[i][0] = pts[i][0]; s->pt[i][1] = pts[i][1]; }
}

static void define_shapes(shape_t *sh)
{
    static const float L[][2] = { {0,0}, {3,0}, {3,1}, {1,1}, {1,3}, {0,3} };
    static const float PLUS[][2] = { {1,0}, {2,0}, {2,1}, {3,1}, {3,2}, {2,2}, {2,3}, {1,3}, {1,2}, {0,2}, {0,1}, {1,1} };

    sh[SH_L]    = (shape_t){ "L", { 0.95f, 0.50f, 0.30f }, -10.0, 0.5, 3.0 };
    outline(&sh[SH_L], L, 6);
    sh[SH_PLUS] = (shape_t){ "plus", { 0.35f, 0.75f, 0.95f }, -5.0, 0.5, 3.0 };
    outline(&sh[SH_PLUS], PLUS, 12);
    sh[SH_CUP]  = (shape_t){ "cup", { 0.90f, 0.80f, 0.30f }, 0.0, 1.5, 8.0, 0.1f };    /* a tray: n == 0 */

    sh[SH_STAR] = (shape_t){ "star", { 0.60f, 0.85f, 0.45f }, 5.5, 0.5, 3.0 };
    for (int i = 0; i < 10; i++) {
        float a = 6.2831853f * (float)i / 10.0f + 1.5707963f, r = (i & 1) ? 0.9f : 2.0f;
        sh[SH_STAR].pt[i][0] = r * cosf(a);
        sh[SH_STAR].pt[i][1] = r * sinf(a);
    }
    sh[SH_STAR].n = 10;

    sh[SH_CRESCENT] = (shape_t){ "crescent", { 0.80f, 0.55f, 0.90f }, 11.0, 0.5, 3.0 };
    {
        shape_t *c = &sh[SH_CRESCENT];
        const int arc = 8;                         /* segments per arc */
        const float a0 = 0.8f, a1 = 6.2831853f - 0.8f;
        int n = 0;
        for (int i = 0; i <= arc; i++) {           /* outer arc counter-clockwise */
            float a = a0 + (a1 - a0) * (float)i / (float)arc;
            c->pt[n][0] = 2.0f * cosf(a); c->pt[n][1] = 2.0f * sinf(a); n++;
        }
        for (int i = arc; i >= 0; i--) {           /* inner arc back */
            float a = a0 + (a1 - a0) * (float)i / (float)arc;
            c->pt[n][0] = 1.1f * cosf(a); c->pt[n][1] = 1.1f * sinf(a); n++;
        }
        c->n = n;
    }
}

/* ── mesh: extrude an outline ────────────────────────────────────────────── */

static float cross2(const float a[2], const float b[2], const float c[2])
{
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

static bool in_triangle(const float p[2], const float a[2], const float b[2], const float c[2])
{
    return cross2(a, b, p) >= 0.0f && cross2(b, c, p) >= 0.0f && cross2(c, a, p) >= 0.0f;
}

/* Ear clipping of a counter-clockwise simple polygon: n - 2 triangles of
 * outline indices into tri. */
static int triangulate(const float (*p)[2], int n, int tri[][3])
{
    int idx[MAX_OUTLINE], count = n, out = 0;
    for (int i = 0; i < n; i++) idx[i] = i;
    int guard = 0;
    while (count > 3 && guard++ < 1000) {
        bool clipped = false;
        for (int i = 0; i < count && !clipped; i++) {
            int a = idx[(i + count - 1) % count], b = idx[i], c = idx[(i + 1) % count];
            if (cross2(p[a], p[b], p[c]) <= 1e-6f) continue;               /* reflex corner */
            bool ear = true;
            for (int j = 0; j < count && ear; j++) {
                int q = idx[j];
                if (q == a || q == b || q == c) continue;
                if (in_triangle(p[q], p[a], p[b], p[c])) ear = false;
            }
            if (!ear) continue;
            tri[out][0] = a; tri[out][1] = b; tri[out][2] = c; out++;
            for (int j = i; j < count - 1; j++) idx[j] = idx[j + 1];
            count--;
            clipped = true;
        }
        if (!clipped) break;
    }
    if (count == 3) { tri[out][0] = idx[0]; tri[out][1] = idx[1]; tri[out][2] = idx[2]; out++; }
    return out;
}

/* Closed triangle mesh of the outline extruded along z (y is -up, so the outline
 * is mirrored; the winding is flipped to keep the normals pointing out), centred
 * on the centroid of the outline. Returns the triangle count. */
static int extrude(const shape_t *s, float *vert, int *idx, float centre[2])
{
    const int n = s->n;
    const float h = (float)s->half_z;
    int cap[MAX_OUTLINE][3];
    int n_cap = triangulate(s->pt, n, cap);

    /* area centroid of the outline */
    float area = 0.0f, cx = 0.0f, cy = 0.0f;
    for (int i = 0; i < n; i++) {
        const float *a = s->pt[i], *b = s->pt[(i + 1) % n];
        float w = a[0] * b[1] - b[0] * a[1];
        area += w;
        cx += (a[0] + b[0]) * w;
        cy += (a[1] + b[1]) * w;
    }
    centre[0] = cx / (3.0f * area);
    centre[1] = cy / (3.0f * area);

    for (int i = 0; i < n; i++) {
        for (int k = 0; k < 2; k++) {
            float *v = &vert[(k * n + i) * 3];
            v[0] = s->pt[i][0] - centre[0];
            v[1] = -(s->pt[i][1] - centre[1]);             /* up is -Y */
            v[2] = k ? h : -h;
        }
    }
    int t = 0;
#define TRI(a, b, c) do { idx[t * 3] = (a); idx[t * 3 + 1] = (c); idx[t * 3 + 2] = (b); t++; } while (0)
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        TRI(i, j, n + j);                                  /* side, outward in (x, up, z) */
        TRI(i, n + j, n + i);
    }
    for (int i = 0; i < n_cap; i++) {
        TRI(n + cap[i][0], n + cap[i][1], n + cap[i][2]);  /* front, +z */
        TRI(cap[i][0], cap[i][2], cap[i][1]);              /* back, -z */
    }
#undef TRI
    return t;
}

/* A tray: a 4 x 2.5 x 3 box with a pit, a closed mesh with a cavity (a profile
 * extruded along z would be open at both ends). 12 shared vertices, centred on
 * the bounding box. Returns the triangle count. */
#define TRAY_W   2.0f      /* half width (x) */
#define TRAY_D   1.5f      /* half depth (z) */
#define TRAY_H   2.5f      /* height */
#define TRAY_WALL 0.5f
#define TRAY_FLOOR 0.6f

static int quad(int *idx, int t, const float *vert, int a, int b, int c, int d, const float out[3])
{
    /* vert holds (x, up, z); orient the quad a b c d to face `out` */
    const float *pa = &vert[a * 3], *pb = &vert[b * 3], *pc = &vert[c * 3];
    float u[3] = { pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2] }, w[3] = { pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2] };
    float n[3] = { u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0] };
    if (n[0] * out[0] + n[1] * out[1] + n[2] * out[2] < 0.0f) { int tmp = b; b = d; d = tmp; }
    /* y = -up mirrors the mesh: swap two indices of each triangle */
    idx[t * 3] = a; idx[t * 3 + 1] = c; idx[t * 3 + 2] = b; t++;
    idx[t * 3] = a; idx[t * 3 + 1] = d; idx[t * 3 + 2] = c; t++;
    return t;
}

static int tray(float *vert, int *idx, float *centre_u)
{
    const float W = TRAY_W, D = TRAY_D, H = TRAY_H, wi = TRAY_W - TRAY_WALL, di = TRAY_D - TRAY_WALL;
    const float fl = TRAY_FLOOR, wf = 0.6f, df = 0.4f;     /* the pit floor: slanted walls */
    /* 0..3 bottom ring, 4..7 top outer ring, 8..11 top pit ring, 12..15 pit floor (u up) */
    const float P[16][3] = {
        { -W, 0, -D }, { W, 0, -D }, { W, 0, D }, { -W, 0, D },
        { -W, H, -D }, { W, H, -D }, { W, H, D }, { -W, H, D },
        { -wi, H, -di }, { wi, H, -di }, { wi, H, di }, { -wi, H, di },
        { -wf, fl, -df }, { wf, fl, -df }, { wf, fl, df }, { -wf, fl, df } };
    *centre_u = 0.5f * H;
    float up_space[16 * 3];                                   /* (x, up, z), for orienting the faces */
    for (int i = 0; i < 16; i++) {
        up_space[i * 3] = vert[i * 3] = P[i][0];
        up_space[i * 3 + 1] = P[i][1] - *centre_u;
        vert[i * 3 + 1] = -(P[i][1] - *centre_u);              /* up is -Y */
        up_space[i * 3 + 2] = vert[i * 3 + 2] = P[i][2];
    }
    int t = 0;
    const float down[3] = { 0, -1, 0 }, up[3] = { 0, 1, 0 };
    const float px[3] = { 1, 0, 0 }, nx[3] = { -1, 0, 0 }, pz[3] = { 0, 0, 1 }, nz[3] = { 0, 0, -1 };
    t = quad(idx, t, up_space, 0, 1, 2, 3, down);                 /* bottom */
    t = quad(idx, t, up_space, 0, 1, 5, 4, nz);                   /* outer walls */
    t = quad(idx, t, up_space, 1, 2, 6, 5, px);
    t = quad(idx, t, up_space, 2, 3, 7, 6, pz);
    t = quad(idx, t, up_space, 3, 0, 4, 7, nx);
    t = quad(idx, t, up_space, 4, 5, 9, 8, up);                   /* rim, four trapezoids */
    t = quad(idx, t, up_space, 5, 6, 10, 9, up);
    t = quad(idx, t, up_space, 6, 7, 11, 10, up);
    t = quad(idx, t, up_space, 7, 4, 8, 11, up);
    t = quad(idx, t, up_space, 8, 9, 13, 12, pz);                 /* pit walls face into the pit */
    t = quad(idx, t, up_space, 9, 10, 14, 13, nx);
    t = quad(idx, t, up_space, 10, 11, 15, 14, nz);
    t = quad(idx, t, up_space, 11, 8, 12, 15, px);
    t = quad(idx, t, up_space, 12, 13, 14, 15, up);               /* pit floor */
    return t;
}

typedef struct ledge_count { int n; } ledge_count_t;
static void count_ledge(void *ctx, const ivp_compact_ledge_t *ledge)
{
    (void)ledge;
    ((ledge_count_t *)ctx)->n++;
}

static ivp_object_t *build_shape(demo_world_t *w, shape_t *s)
{
    float vert[2 * MAX_OUTLINE * 3];
    int idx[(2 * MAX_OUTLINE + 2 * MAX_OUTLINE) * 3];
    float centre[2];
    int n_vert = 2 * s->n;
    int n_tri;
    ivp_concave_options_t opt = ivp_concave_options_default();
    if (s->rdacc > 0.0f) opt.decompose.rdacc = s->rdacc;
    if (s->n > 0) {
        n_tri = extrude(s, vert, idx, centre);
    } else {
        n_vert = 16;
        n_tri = tray(vert, idx, centre);
    }
    ivp_compact_surface_t *surf = ivp_build_concave_surface(NULL, vert, n_vert, idx, n_tri, &opt);
    if (!surf) return NULL;

    float lo[3] = { 1e9f, 1e9f, 1e9f }, hi[3] = { -1e9f, -1e9f, -1e9f };
    for (int i = 0; i < n_vert; i++)
        for (int k = 0; k < 3; k++) {
            if (vert[i * 3 + k] < lo[k]) lo[k] = vert[i * 3 + k];
            if (vert[i * 3 + k] > hi[k]) hi[k] = vert[i * 3 + k];
        }
    for (int k = 0; k < 3; k++) s->bound[k] = 0.5f * (hi[k] - lo[k]);

    ivp_object_template_t t;
    demo_template_dynamic(&t, &w->mat, s->mass);
    demo_set_box_inertia(&t, s->mass, s->bound[0], s->bound[1], s->bound[2]);
    /* it lands on its lowest point, bound[1] below the centroid at the most */
    demo_template_pose(&t, NULL, s->x, -(s->bound[1] + 0.6), 0.0);
    ivp_object_t *obj = demo_create_polygon(w, surf, &t);       /* the world owns the surface */
    if (!obj) return NULL;
    ledge_count_t cnt = { 0 };
    ivp_surman_get_all_terminal_ledges(obj->surman, count_ledge, &cnt);
    s->n_parts = cnt.n;
    return obj;
}

/* ── drawing ─────────────────────────────────────────────────────────────── */

typedef struct part_ctx {
    demo_wire_ctx_t wire;
    int             index;
} part_ctx_t;

static void part_ledge(void *ctx, const ivp_compact_ledge_t *ledge)
{
    static const float pal[6][3] = { { 1.0f, 0.3f, 0.3f }, { 0.3f, 1.0f, 0.4f }, { 0.4f, 0.6f, 1.0f },
                                     { 1.0f, 0.9f, 0.2f }, { 1.0f, 0.4f, 1.0f }, { 0.2f, 1.0f, 1.0f } };
    part_ctx_t *c = (part_ctx_t *)ctx;
    c->wire.color = pal[c->index++ % 6];
    demo_wire_ledge(&c->wire, ledge);
}

static void draw_parts(demo_viewer_t *v, const ivp_object_t *obj)
{
    part_ctx_t c;
    memset(&c, 0, sizeof(c));
    c.wire.v = v;
    demo_object_pose(obj, c.wire.pos, c.wire.rot);
    ivp_surman_get_all_terminal_ledges(obj->surman, part_ledge, &c);
}

static void concave_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    concave_demo_t *s = (concave_demo_t *)app->user;
    demo_param_bool(v, "convex parts", &s->show_parts);
}

static bool ball_in_cup(const concave_demo_t *s, int i)
{
    float c[3], p[3];
    demo_object_center(s->shape[SH_CUP].obj, c);
    demo_object_center(s->balls[i], p);
    return fabsf(p[0] - c[0]) < 1.5f && fabsf(p[2] - c[2]) < 1.1f && p[1] < c[1] + 0.8f && p[1] > c[1] - 1.6f;
}

static void concave_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    concave_demo_t *s = (concave_demo_t *)app->user;
    (void)v;
    if (frame % 10 == 0) {
        int n = 0;
        for (int i = 0; i < N_BALLS; i++) n += ball_in_cup(s, i);
        s->balls_in_cup = n;
    }
}

static void concave_draw(demo_app_t *app, demo_viewer_t *v)
{
    concave_demo_t *s = (concave_demo_t *)app->user;
    const float c_ground[3] = { 0.55f, 0.58f, 0.62f }, c_ball[3] = { 0.95f, 0.95f, 0.95f };
    demo_draw_object(v, s->ground, c_ground);
    for (int i = 0; i < N_SHAPES; i++) {
        demo_draw_object(v, s->shape[i].obj, s->shape[i].col);
        if (s->show_parts) draw_parts(v, s->shape[i].obj);
    }
    for (int i = 0; i < N_BALLS; i++) demo_draw_object(v, s->balls[i], c_ball);

    char line[160];
    int off = 0;
    for (int i = 0; i < N_SHAPES; i++)
        off += snprintf(line + off, sizeof(line) - (size_t)off, "%s %d  ", s->shape[i].name, s->shape[i].n_parts);
    demo_hud(v, DEMO_COL_INFO, "convex parts per body: %s", line);
    demo_hud(v, DEMO_COL_INFO, "%d of %d balls are in the cup", s->balls_in_cup, N_BALLS);
}

static void concave_stats(demo_app_t *app, char *buf, size_t size)
{
    concave_demo_t *s = (concave_demo_t *)app->user;
    float vmax = 0.0f;
    for (int i = 0; i < N_SHAPES; i++) {
        float sp = demo_object_speed(s->shape[i].obj);
        if (sp > vmax) vmax = sp;
    }
    snprintf(buf, size, "max speed %.2f m/s, %d balls in the cup", (double)vmax, s->balls_in_cup);
}

static bool concave_check(demo_app_t *app)
{
    concave_demo_t *s = (concave_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    for (int i = 0; i < N_SHAPES; i++) {
        float p[3];
        demo_object_center(s->shape[i].obj, p);
        ok &= demo_check(tag, s->shape[i].n_parts >= 2, "%s: %d convex parts", s->shape[i].name, s->shape[i].n_parts);
        ok &= demo_check(tag, p[1] < 0.0f && demo_object_speed(s->shape[i].obj) < 0.5f,
                         "%s rests on the ground (y %.2f, speed %.2f)", s->shape[i].name, (double)p[1],
                         (double)demo_object_speed(s->shape[i].obj));
    }
    int in_cup = 0;
    for (int i = 0; i < N_BALLS; i++) in_cup += ball_in_cup(s, i);
    ok &= demo_check(tag, in_cup >= N_BALLS - 1, "%d of %d balls stay in the cup", in_cup, N_BALLS);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Concave -- Convex Decomposition";
    app.cfg.tag         = "concave";
    app.cfg.friction    = 0.7;
    app.cfg.elasticity  = 0.2;
    app.cfg.orbit_dist  = 19.0f;
    app.cfg.orbit_pitch = -30.0f;
    app.cfg.target[1]   = -1.0f;
    app.cfg.grid_extent = 16.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    concave_demo_t *s = (concave_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->show_parts = true;

    s->ground = demo_create_box(w, NULL, 16.0, 0.5, 6.0, 0.0, NULL, 0.0, 0.5, 0.0);
    define_shapes(s->shape);
    for (int i = 0; i < N_SHAPES; i++) {
        s->shape[i].obj = build_shape(w, &s->shape[i]);
        if (!s->shape[i].obj) {
            fprintf(stderr, "[concave] building %s failed\n", s->shape[i].name);
            return 1;
        }
    }
    /* balls dropped into the cup one after the other */
    for (int i = 0; i < N_BALLS; i++)
        s->balls[i] = demo_create_ball(w, NULL, 0.35, 0.4, NULL, 0.35 * ((i % 3) - 1), -7.0 - 1.2 * i,
                                       0.3 * ((i / 3) - 0.5));

    app.viewer_init = concave_viewer_init;
    app.update      = concave_update;
    app.draw        = concave_draw;
    app.stats       = concave_stats;
    app.check       = concave_check;
    app.controls    = "F5 panel: draw the convex parts of every body";

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
