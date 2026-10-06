/* example_3ds_import.c -- Load .3ds file -> physics simulation
 *
 * Demonstrates the content pipeline: load .3ds -> build surface -> create
 * environment + ground + dynamic object built from the imported mesh ->
 * simulate.  The mesh drops onto the ground and comes to rest; it is drawn
 * as the wireframe of its compact surface.  Like the reference importer
 * every face becomes a two sided triangle ledge.
 *
 * Usage: example_3ds_import [filename.3ds] [--headless [frames]]
 *        (if no file is given, a built-in tetrahedron is used)
 *
 * (The example used to build the surface but create both objects without
 *  a surface manager: nothing collided and only placeholder boxes were
 *  drawn.)
 */
#include "demo_app.h"

#include <ivp/builder/ivp_import_3ds.h>

#define DROP_Y     (-5.0)
#define GROUND_TOP (-0.5f)

/* Inline synthetic tetrahedron .3ds for when no file is provided */
static void w16(unsigned char *buf, int *pos, uint16_t val) {
    buf[*pos] = (unsigned char)(val & 0xFF);
    buf[*pos + 1] = (unsigned char)((val >> 8) & 0xFF);
    *pos += 2;
}
static void w32(unsigned char *buf, int *pos, uint32_t val) {
    buf[*pos] = (unsigned char)(val & 0xFF);
    buf[*pos + 1] = (unsigned char)((val >> 8) & 0xFF);
    buf[*pos + 2] = (unsigned char)((val >> 16) & 0xFF);
    buf[*pos + 3] = (unsigned char)((val >> 24) & 0xFF);
    *pos += 4;
}
static void wf32(unsigned char *buf, int *pos, float val) {
    uint32_t bits;
    memcpy(&bits, &val, sizeof(bits));
    w32(buf, pos, bits);
}

static int build_demo_3ds(unsigned char *buf) {
    int pos = 0;
    float verts[4][3] = {
        {-0.5f, 0.0f, -0.3f}, {0.5f, 0.0f, -0.3f},
        {0.0f, 0.0f, 0.6f}, {0.0f, -0.8f, 0.0f}
    };
    uint16_t faces[4][3] = {{0,2,1},{0,1,3},{1,2,3},{0,3,2}};

    int vertlist_size = 6 + 2 + 4 * 3 * 4;
    int facelist_size = 6 + 2 + 4 * 4 * 2;
    int trimesh_size = 6 + vertlist_size + facelist_size;
    const char *name = "demo";
    int objblock_size = 6 + (int)strlen(name) + 1 + trimesh_size;
    int objmesh_size = 6 + objblock_size;
    int main_size = 6 + objmesh_size;

    w16(buf, &pos, 0x4D4D); w32(buf, &pos, (uint32_t)main_size);
    w16(buf, &pos, 0x3D3D); w32(buf, &pos, (uint32_t)objmesh_size);
    w16(buf, &pos, 0x4000); w32(buf, &pos, (uint32_t)objblock_size);
    for (int i = 0; name[i]; i++) buf[pos++] = (unsigned char)name[i];
    buf[pos++] = '\0';
    w16(buf, &pos, 0x4100); w32(buf, &pos, (uint32_t)trimesh_size);
    w16(buf, &pos, 0x4110); w32(buf, &pos, (uint32_t)vertlist_size);
    w16(buf, &pos, 4);
    for (int i = 0; i < 4; i++) {
        wf32(buf, &pos, verts[i][0]);
        wf32(buf, &pos, verts[i][1]);
        wf32(buf, &pos, verts[i][2]);
    }
    w16(buf, &pos, 0x4120); w32(buf, &pos, (uint32_t)facelist_size);
    w16(buf, &pos, 4);
    for (int i = 0; i < 4; i++) {
        w16(buf, &pos, faces[i][0]);
        w16(buf, &pos, faces[i][1]);
        w16(buf, &pos, faces[i][2]);
        w16(buf, &pos, 0);
    }
    return pos;
}

typedef struct import_demo {
    const char   *source;           /* file name or "built-in tetrahedron" */
    int           n_vertices, n_triangles, n_ledges;
    ivp_object_t *ground, *mesh;
    float         max_speed;
} import_demo_t;

static void count_ledge(void *ctx, const ivp_compact_ledge_t *ledge)
{
    (void)ledge;
    (*(int *)ctx)++;
}

static void import_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    import_demo_t *s = (import_demo_t *)app->user;
    (void)v;
    (void)frame;
    float sp = demo_object_speed(s->mesh);
    if (sp > s->max_speed) s->max_speed = sp;
}

static void import_draw(demo_app_t *app, demo_viewer_t *v)
{
    import_demo_t *s = (import_demo_t *)app->user;
    const float ground_color[3] = {0.4f, 0.5f, 0.4f};
    const float mesh_color[3]   = {0.9f, 0.3f, 0.2f};
    demo_draw_object(v, s->ground, ground_color);
    demo_draw_object(v, s->mesh, mesh_color);
    demo_hud(v, DEMO_COL_INFO, "Source: %s", s->source);
    demo_hud(v, DEMO_COL_INFO, "Mesh: %d vertices, %d triangles -> %d ledges",
             s->n_vertices, s->n_triangles, s->n_ledges);
    demo_hud_object(v, "Imported object", s->mesh);
}

static void import_stats(demo_app_t *app, char *buf, size_t size)
{
    import_demo_t *s = (import_demo_t *)app->user;
    float p[3];
    demo_object_center(s->mesh, p);
    snprintf(buf, size, "mesh pos=(%.3f,%.3f,%.3f) speed=%.3f tilt=%.1f%s", (double)p[0], (double)p[1], (double)p[2],
             (double)demo_object_speed(s->mesh), (double)demo_object_tilt_deg(s->mesh),
             ivp_object_is_frozen(s->mesh) ? " frozen" : "");
}

static bool import_check(demo_app_t *app)
{
    import_demo_t *s = (import_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, s->n_ledges == s->n_triangles && s->n_ledges > 0,
                     "imported %d triangles as %d two sided triangle ledges", s->n_triangles, s->n_ledges);
    float p[3];
    demo_object_center(s->mesh, p);
    float radius = s->mesh->physical_core->upper_limit_radius;
    ok &= demo_check(tag, s->max_speed > 3.0f, "mesh fell (max speed %.2f m/s)", (double)s->max_speed);
    ok &= demo_check(tag, p[1] < GROUND_TOP && p[1] > GROUND_TOP - radius - 0.05f && demo_object_speed(s->mesh) < 0.05f,
                     "mesh rests on the ground: centre %.3f m above it (bounding radius %.3f), speed %.3f",
                     (double)(GROUND_TOP - p[1]), (double)radius, (double)demo_object_speed(s->mesh));
    return ok;
}

int main(int argc, char **argv)
{
    /* ── Load 3DS ──────────────────────────────────────────────────────── */
    const char *file = demo_positional_arg(argc, argv);   /* NULL = built-in tetrahedron */
    ivp_3ds_scene_t scene;
    memset(&scene, 0, sizeof(scene));
    ivp_result_t r;
    if (file) {
        r = ivp_3ds_load_file(NULL, file, &scene);
    } else {
        unsigned char buf[1024];
        int sz = build_demo_3ds(buf);
        r = ivp_3ds_load_memory(NULL, buf, sz, &scene);
    }
    if (!ivp_ok(r) || scene.n_meshes < 1) {
        fprintf(stderr, "Failed to load %s: %s\n", file ? file : "the built-in tetrahedron", ivp_result_str(r));
        ivp_3ds_scene_free(&scene);
        return 1;
    }
    ivp_compact_surface_t *mesh_surface = ivp_3ds_build_surface(NULL, &scene, 0, NULL);
    if (!mesh_surface) {
        fprintf(stderr, "Failed to build mesh surface\n");
        ivp_3ds_scene_free(&scene);
        return 1;
    }

    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title        = "IVP 3DS Import Example";
    app.cfg.tag          = "3ds_import";
    app.cfg.orbit_dist   = 8.0f;
    app.cfg.orbit_yaw    = 25.0f;
    app.cfg.orbit_pitch  = -20.0f;
    app.cfg.target[1]    = -0.8f;
    app.cfg.grid_extent  = 8.0f;

    import_demo_t *s = (import_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    s->source = file ? file : "built-in tetrahedron";
    s->n_vertices = scene.meshes[0].n_vertices;
    s->n_triangles = scene.meshes[0].n_triangles;
    ivp_3ds_scene_free(&scene);

    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;
    demo_world_t *w = &app.world;

    /* Ground */
    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.mass = IVP_REAL(0.0);
    gt.physical_unmoveable = true;
    s->ground = demo_create_box_templ(w, &gt, 8.0, 0.5, 8.0);

    /* Dynamic imported object (inertia from the compact surface) */
    ivp_object_template_t ot;
    ivp_object_template_init(&ot);
    ot.mass = IVP_REAL(1.0);
    set_inertia_from_surface(&ot, ot.mass, mesh_surface);
    ivp_vec3r_set(&ot.position, IVP_REAL(0.0), (ivp_real_t)DROP_Y, IVP_REAL(0.0));
    s->mesh = demo_create_polygon(w, mesh_surface, &ot);   /* the world owns the surface */
    if (!s->mesh) {
        fprintf(stderr, "Failed to create the imported object\n");
        return 1;
    }
    ivp_surman_get_all_terminal_ledges(s->mesh->surman, count_ledge, &s->n_ledges);
    printf("[3ds_import] %s: %d vertices, %d triangles, %d ledges\n", s->source, s->n_vertices,
           s->n_triangles, s->n_ledges);

    app.update = import_update;
    app.draw   = import_draw;
    app.stats  = import_stats;
    app.check  = import_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
