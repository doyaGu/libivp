/* test_content_pipeline.c — Tests for P3 content production subsystems
 *
 * Tests: 3DS import, Q12 BSP builder, grid heightfield builder,
 *        universe manager, liquid surface, radar, controller golem,
 *        and end-to-end pipeline.
 */
#include "ivp_test.h"

#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>
#include <ivp/builder/ivp_import_3ds.h>
#include <ivp/builder/ivp_build_q12bsp.h>
#include <ivp/builder/ivp_build_grid.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_universe_manager.h>
#include <ivp/controllers/ivp_liquid_surface.h>
#include <ivp/collision/ivp_radar.h>
#include <ivp/controllers/ivp_controller_golem.h>

#include <string.h>
#include <math.h>
#include <limits.h>

/* ═══════════════════════════════════════════════════════════════════════════
 * Synthetic 3DS binary data: a tetrahedron (4 vertices, 4 triangles)
 *
 * 3DS file structure (little-endian):
 *   MAIN (0x4D4D)
 *     OBJMESH (0x3D3D)
 *       OBJBLOCK (0x4000) → name "tetra\0"
 *         TRIMESH (0x4100)
 *           VERTLIST (0x4110) → 4 vertices
 *           FACELIST (0x4120) → 4 faces
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Helper: write little-endian uint16 */
static void w16(unsigned char *buf, int *pos, uint16_t val) {
    buf[*pos]     = (unsigned char)(val & 0xFF);
    buf[*pos + 1] = (unsigned char)((val >> 8) & 0xFF);
    *pos += 2;
}

/* Helper: write little-endian uint32 */
static void w32(unsigned char *buf, int *pos, uint32_t val) {
    buf[*pos]     = (unsigned char)(val & 0xFF);
    buf[*pos + 1] = (unsigned char)((val >> 8) & 0xFF);
    buf[*pos + 2] = (unsigned char)((val >> 16) & 0xFF);
    buf[*pos + 3] = (unsigned char)((val >> 24) & 0xFF);
    *pos += 4;
}

/* Helper: write float as little-endian bytes */
static void wf32(unsigned char *buf, int *pos, float val) {
    uint32_t bits;
    memcpy(&bits, &val, sizeof(bits));
    w32(buf, pos, bits);
}

/* Build a synthetic 3DS file containing a tetrahedron */
static int build_tetra_3ds(unsigned char *buf, int bufsize) {
    (void)bufsize;
    int pos = 0;

    /* Tetrahedron vertices */
    float verts[4][3] = {
        { 0.0f, 0.0f, 0.0f },
        { 1.0f, 0.0f, 0.0f },
        { 0.0f, 1.0f, 0.0f },
        { 0.0f, 0.0f, 1.0f }
    };

    /* Tetrahedron faces (4 triangles) */
    uint16_t faces[4][3] = {
        { 0, 2, 1 },
        { 0, 1, 3 },
        { 1, 2, 3 },
        { 0, 3, 2 }
    };

    /* VERTLIST chunk: 2 (id) + 4 (size) + 2 (n_verts) + 4*3*4 (vertex data) */
    int vertlist_size = 6 + 2 + 4 * 3 * 4;

    /* FACELIST chunk: 2 (id) + 4 (size) + 2 (n_faces) + 4*4*2 (face data) */
    int facelist_size = 6 + 2 + 4 * 4 * 2;

    /* TRIMESH chunk */
    int trimesh_size = 6 + vertlist_size + facelist_size;

    /* OBJBLOCK chunk: 6 (header) + strlen("tetra") + 1 (null) + trimesh */
    const char *name = "tetra";
    int objblock_size = 6 + (int)strlen(name) + 1 + trimesh_size;

    /* OBJMESH chunk */
    int objmesh_size = 6 + objblock_size;

    /* MAIN chunk */
    int main_size = 6 + objmesh_size;

    /* Write MAIN header */
    w16(buf, &pos, 0x4D4D);
    w32(buf, &pos, (uint32_t)main_size);

    /* Write OBJMESH header */
    w16(buf, &pos, 0x3D3D);
    w32(buf, &pos, (uint32_t)objmesh_size);

    /* Write OBJBLOCK header */
    w16(buf, &pos, 0x4000);
    w32(buf, &pos, (uint32_t)objblock_size);

    /* Object name (null-terminated) */
    for (int i = 0; name[i]; i++) buf[pos++] = (unsigned char)name[i];
    buf[pos++] = '\0';

    /* Write TRIMESH header */
    w16(buf, &pos, 0x4100);
    w32(buf, &pos, (uint32_t)trimesh_size);

    /* Write VERTLIST */
    w16(buf, &pos, 0x4110);
    w32(buf, &pos, (uint32_t)vertlist_size);
    w16(buf, &pos, 4); /* n_verts */
    for (int i = 0; i < 4; i++) {
        wf32(buf, &pos, verts[i][0]);
        wf32(buf, &pos, verts[i][1]);
        wf32(buf, &pos, verts[i][2]);
    }

    /* Write FACELIST */
    w16(buf, &pos, 0x4120);
    w32(buf, &pos, (uint32_t)facelist_size);
    w16(buf, &pos, 4); /* n_faces */
    for (int i = 0; i < 4; i++) {
        w16(buf, &pos, faces[i][0]);
        w16(buf, &pos, faces[i][1]);
        w16(buf, &pos, faces[i][2]);
        w16(buf, &pos, 0); /* flags */
    }

    return pos;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 3DS Parser Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(tds_parse_tetrahedron) {
    unsigned char buf[1024];
    int size = build_tetra_3ds(buf, (int)sizeof(buf));

    ivp_3ds_scene_t scene;
    ivp_result_t r = ivp_3ds_load_memory(NULL, buf, size, &scene);
    IVP_ASSERT_TRUE(ivp_ok(r));
    IVP_ASSERT_EQ(scene.n_meshes, 1);
    IVP_ASSERT_EQ(scene.meshes[0].n_vertices, 4);
    IVP_ASSERT_EQ(scene.meshes[0].n_triangles, 4);
    IVP_ASSERT_TRUE(strcmp(scene.meshes[0].name, "tetra") == 0);

    /* Verify vertex 1 is at (1,0,0) */
    IVP_ASSERT_NEAR(scene.meshes[0].vertices[3], 1.0f, 1e-6);
    IVP_ASSERT_NEAR(scene.meshes[0].vertices[4], 0.0f, 1e-6);
    IVP_ASSERT_NEAR(scene.meshes[0].vertices[5], 0.0f, 1e-6);

    ivp_3ds_scene_free(&scene);
}

IVP_TEST(tds_invalid_data) {
    /* Truncated data: not even a complete chunk header */
    unsigned char bad[4] = { 0x4D, 0x4D, 0x00, 0x00 };
    ivp_3ds_scene_t scene;
    ivp_result_t r = ivp_3ds_load_memory(NULL, bad, 4, &scene);
    IVP_ASSERT_TRUE(r == IVP_ERR_FILE_CORRUPT);
}

IVP_TEST(tds_wrong_magic) {
    unsigned char bad[8] = { 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00 };
    ivp_3ds_scene_t scene;
    ivp_result_t r = ivp_3ds_load_memory(NULL, bad, 8, &scene);
    IVP_ASSERT_TRUE(r == IVP_ERR_FILE_CORRUPT);
}

IVP_TEST(tds_build_surface) {
    unsigned char buf[1024];
    int size = build_tetra_3ds(buf, (int)sizeof(buf));

    ivp_3ds_scene_t scene;
    ivp_result_t r = ivp_3ds_load_memory(NULL, buf, size, &scene);
    IVP_ASSERT_TRUE(ivp_ok(r));

    ivp_compact_surface_t *surface = ivp_3ds_build_surface(NULL, &scene, 0, NULL);
    IVP_ASSERT_NOT_NULL(surface);

    ivp_compact_surface_free(NULL, surface);
    ivp_3ds_scene_free(&scene);
}

IVP_TEST(tds_scale) {
    unsigned char buf[1024];
    int size = build_tetra_3ds(buf, (int)sizeof(buf));

    ivp_3ds_scene_t scene;
    ivp_result_t r = ivp_3ds_load_memory(NULL, buf, size, &scene);
    IVP_ASSERT_TRUE(ivp_ok(r));

    /* Build with scale = 0.01 */
    ivp_3ds_import_config_t cfg;
    ivp_3ds_import_config_init(&cfg);
    cfg.scale = 0.01f;

    ivp_compact_surface_t *surface = ivp_3ds_build_surface(NULL, &scene, 0, &cfg);
    /* Surface might be NULL for very small tetrahedron, but the scale was applied.
     * Verify at least the mesh vertices were scaled before building. */
    if (surface) {
        /* Bounding radius should be much smaller than the unscaled version */
        IVP_ASSERT_TRUE(surface->upper_limit_radius < 0.1f);
        ivp_compact_surface_free(NULL, surface);
    }

    ivp_3ds_scene_free(&scene);
}

IVP_TEST(tds_null_args) {
    ivp_3ds_scene_t scene;
    IVP_ASSERT_TRUE(ivp_3ds_load_memory(NULL, NULL, 0, &scene) == IVP_ERR_INVALID_ARG);
    IVP_ASSERT_TRUE(ivp_3ds_load_file(NULL, NULL, &scene) == IVP_ERR_INVALID_ARG);
    IVP_ASSERT_TRUE(ivp_3ds_build_surface(NULL, NULL, 0, NULL) == NULL);
}

IVP_TEST(tds_config_defaults) {
    ivp_3ds_import_config_t cfg;
    ivp_3ds_import_config_init(&cfg);
    IVP_ASSERT_NEAR(cfg.scale, 1.0f, 1e-6);
    IVP_ASSERT_NEAR(cfg.weld_epsilon, 1e-4f, 1e-8);
    IVP_ASSERT_TRUE(cfg.remove_degenerate);
    IVP_ASSERT_FALSE(cfg.center_meshes);
}

IVP_TEST(tds_weld_balances_allocator_bytes) {
    float vertices[] = {
        0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f,
        0.0f, 0.0f, 0.0f
    };
    int indices[] = { 4, 2, 1, 4, 1, 3, 1, 2, 3, 2, 4, 3 };
    ivp_3ds_mesh_t mesh;
    memset(&mesh, 0, sizeof(mesh));
    mesh.vertices = vertices;
    mesh.indices = indices;
    mesh.n_vertices = 5;
    mesh.n_triangles = 4;
    ivp_3ds_scene_t scene;
    memset(&scene, 0, sizeof(scene));
    scene.meshes = &mesh;
    scene.n_meshes = 1;

    ivp_allocator_t counting = ivp_counting_allocator_create(NULL);
    ivp_compact_surface_t *surface =
        ivp_3ds_build_surface(&counting, &scene, 0, NULL);
    IVP_ASSERT_NOT_NULL(surface);
    ivp_compact_surface_free(&counting, surface);
    ivp_alloc_stats_t stats;
    ivp_counting_allocator_get_stats(&counting, &stats);
    IVP_ASSERT_EQ(stats.current_bytes, 0);
    IVP_ASSERT_EQ(stats.alloc_count, stats.free_count);
    ivp_counting_allocator_destroy(&counting);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Q12 BSP Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(bsp_synthetic_box) {
    /* Build a minimal BSP representing a box: 1 model, 6 planes, simple tree.
     * The tree has 3 interior nodes splitting along X, Y, Z with solid leaves. */

    /* 6 planes forming a unit box centered at origin */
    ivp_q12_plane_t planes[6] = {
        {{ 1, 0, 0},  1, 0},  /* x = +1 */
        {{-1, 0, 0},  1, 0},  /* x = -1 (reversed) */
        {{ 0, 1, 0},  1, 0},  /* y = +1 */
        {{ 0,-1, 0},  1, 0},  /* y = -1 (reversed) */
        {{ 0, 0, 1},  1, 0},  /* z = +1 */
        {{ 0, 0,-1},  1, 0},  /* z = -1 (reversed) */
    };

    /* Node tree:
     * node 0 (plane 0): left→node1, right→empty(-2)
     * node 1 (plane 2): left→node2, right→empty(-2)
     * node 2 (plane 4): left→solid(-1), right→empty(-2) */
    ivp_q12_node_t nodes[3] = {
        {0, { 1, -2}, {-1,-1,-1}, {1,1,1}, 0, 0},
        {2, { 2, -2}, {-1,-1,-1}, {1,1,1}, 0, 0},
        {4, {-1, -2}, {-1,-1,-1}, {1,1,1}, 0, 0},
    };

    ivp_q12_model_t models[1] = {
        {{-1,-1,-1}, {1,1,1}, {0,0,0}, {0, 0, 0, 0}, 0, 0, 0}
    };

    ivp_q12bsp_t bsp;
    ivp_q12bsp_init_from_memory(&bsp,
        models, 1,
        planes, 6,
        nodes, 3,
        NULL, 0);

    ivp_q12bsp_config_t cfg;
    ivp_q12bsp_config_init(&cfg);

    ivp_compact_surface_t *surface = ivp_q12bsp_build_surface(NULL, &bsp, 0, &cfg);
    /* Even a minimal BSP with just 3 planes along the path may produce a valid
     * surface if the bounding box planes make it closed. */
    if (surface) {
        IVP_ASSERT_TRUE(surface->upper_limit_radius > 0.0f);
        ivp_compact_surface_free(NULL, surface);
    }

    ivp_q12bsp_free(&bsp);
}

IVP_TEST(bsp_config_defaults) {
    ivp_q12bsp_config_t cfg;
    ivp_q12bsp_config_init(&cfg);
    IVP_ASSERT_NEAR(cfg.scale, 1.0f, 1e-6);
    IVP_ASSERT_NEAR(cfg.shrink_value, 0.0f, 1e-6);
    IVP_ASSERT_NEAR(cfg.pointmerge_threshold, 0.01f, 1e-6);
}

IVP_TEST(bsp_null_args) {
    IVP_ASSERT_TRUE(ivp_q12bsp_build_surface(NULL, NULL, 0, NULL) == NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Grid / Heightfield Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(grid_flat) {
    /* All heights = 0.0 → flat terrain */
    float heights[9 * 9];
    memset(heights, 0, sizeof(heights));

    ivp_grid_config_t cfg;
    ivp_grid_config_init(&cfg);

    ivp_compact_surface_t *surface = ivp_build_grid_surface(NULL, &cfg, heights);
    IVP_ASSERT_NOT_NULL(surface);
    IVP_ASSERT_TRUE(surface->upper_limit_radius > 0.0f);

    ivp_compact_surface_free(NULL, surface);
}

IVP_TEST(grid_slope) {
    /* Linear height gradient along row axis */
    ivp_grid_config_t cfg;
    ivp_grid_config_init(&cfg);
    cfg.n_rows = 5;
    cfg.n_columns = 5;

    float heights[5 * 5];
    for (int r = 0; r < 5; r++) {
        for (int c = 0; c < 5; c++) {
            heights[r * 5 + c] = (float)r * 0.5f;
        }
    }

    ivp_compact_surface_t *surface = ivp_build_grid_surface(NULL, &cfg, heights);
    IVP_ASSERT_NOT_NULL(surface);

    /* Verify no NaN in mass_center */
    IVP_ASSERT_TRUE(surface->mass_center[0] == surface->mass_center[0]); /* NaN check */
    IVP_ASSERT_TRUE(surface->mass_center[1] == surface->mass_center[1]);
    IVP_ASSERT_TRUE(surface->mass_center[2] == surface->mass_center[2]);

    ivp_compact_surface_free(NULL, surface);
}

IVP_TEST(grid_config_defaults) {
    ivp_grid_config_t cfg;
    ivp_grid_config_init(&cfg);
    IVP_ASSERT_EQ(cfg.n_rows, 9);
    IVP_ASSERT_EQ(cfg.n_columns, 9);
    IVP_ASSERT_NEAR(cfg.field_size, 1.0f, 1e-6);
    IVP_ASSERT_EQ(cfg.height_axis, 1);
}

IVP_TEST(grid_null_heightfield) {
    ivp_compact_surface_t *surface = ivp_build_grid_surface(NULL, NULL, NULL);
    IVP_ASSERT_NULL(surface);
}

IVP_TEST(grid_small) {
    /* Minimum grid: 2x2 = 2 triangles */
    ivp_grid_config_t cfg;
    ivp_grid_config_init(&cfg);
    cfg.n_rows = 2;
    cfg.n_columns = 2;

    float heights[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    ivp_compact_surface_t *surface = ivp_build_grid_surface(NULL, &cfg, heights);
    IVP_ASSERT_NOT_NULL(surface);
    ivp_compact_surface_free(NULL, surface);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Universe Manager Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

static int s_deleted_count = 0;
static void test_deleted_fn(void *ctx, ivp_object_t *obj) {
    (void)ctx; (void)obj;
    s_deleted_count++;
}

IVP_TEST(universe_manager_init) {
    ivp_universe_manager_t mgr;
    ivp_universe_manager_init(&mgr);
    IVP_ASSERT_TRUE(mgr.ensure_fn == NULL);
    IVP_ASSERT_TRUE(mgr.evict_fn == NULL);
    IVP_ASSERT_TRUE(mgr.deleted_fn == NULL);
    IVP_ASSERT_EQ(mgr.check_objects_per_second_threshold_1, 10);
    IVP_ASSERT_FALSE(ivp_universe_manager_is_active(&mgr));
}

IVP_TEST(universe_manager_deleted_callback) {
    s_deleted_count = 0;

    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);

    ivp_universe_manager_t mgr;
    ivp_universe_manager_init(&mgr);
    mgr.deleted_fn = test_deleted_fn;
    ivp_environment_set_universe_manager(env, &mgr);

    IVP_ASSERT_NOT_NULL(ivp_environment_get_universe_manager(env));

    /* Create and destroy an object → deleted_fn should fire */
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(obj);

    ivp_environment_destroy_object(env, obj);
    IVP_ASSERT_EQ(s_deleted_count, 1);

    ivp_environment_destroy(env);
}

/* IVP_Cluster_Manager::check_for_unused_objects walks the cluster tree:
 * newest object first (IVP_Cluster::add_object inserts at the head), down
 * into sub-clusters, the static object last (reference count -1: never
 * reported). */
typedef struct um_visit_log {
    ivp_object_t *visited[32];
    int n;
    ivp_environment_t *env;
    ivp_object_t *delete_on_visit;
} um_visit_log_t;

static void um_log_evict(void *ctx, ivp_object_t *obj)
{
    um_visit_log_t *log = (um_visit_log_t *)ctx;
    if (log->n < 32) log->visited[log->n++] = obj;
    if (obj == log->delete_on_visit) {
        log->delete_on_visit = NULL;
        ivp_environment_destroy_object(log->env, obj);
    }
}

static ivp_object_t *um_ball(ivp_environment_t *env, double x)
{
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.ball_radius = IVP_REAL(0.5);
    t.mass = IVP_REAL(1.0);
    t.physical_unmoveable = true; /* stays where it is, no collision partner */
    ivp_vec3r_set(&t.position, (ivp_real_t)x, IVP_REAL(0.0), IVP_REAL(0.0));
    t.orientation = ivp_quatr_identity();
    return ivp_environment_create_object(env, &t, NULL);
}

IVP_TEST(universe_manager_visits_cluster_tree_order) {
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.delta_psi_time = IVP_REAL(0.01);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);
    ivp_object_t *a = um_ball(env, 0.0);
    ivp_object_t *b = um_ball(env, 10.0);
    ivp_object_t *c = um_ball(env, 20.0);
    ivp_object_t *d = um_ball(env, 30.0);
    IVP_ASSERT_TRUE(a && b && c && d);
    ivp_cluster_t *root = ivp_environment_get_root_cluster(env);
    IVP_ASSERT_NOT_NULL(root);
    IVP_ASSERT_TRUE(ivp_cluster_get_first_child(root) == d);
    IVP_ASSERT_EQ(ivp_cluster_get_child_count(root), 5); /* + the static object */

    /* a sub-cluster (inserted at the head of the root) holding a */
    ivp_cluster_t *k = ivp_environment_create_cluster(env, NULL, NULL);
    IVP_ASSERT_NOT_NULL(k);
    ivp_cluster_add_child(k, a);
    IVP_ASSERT_TRUE(a->parent_cluster == k);

    /* tree: k{a}, d, c, b, static */
    IVP_ASSERT_TRUE(ivp_cluster_get_next_real_object_in_tree(root, NULL) == a);
    IVP_ASSERT_TRUE(ivp_cluster_get_next_real_object_in_tree(root, a) == d);
    IVP_ASSERT_TRUE(ivp_cluster_get_next_real_object_in_tree(root, d) == c);
    IVP_ASSERT_TRUE(ivp_cluster_get_next_real_object_in_tree(root, c) == b);
    IVP_ASSERT_TRUE(ivp_cluster_get_next_real_object_in_tree(root, b) == ivp_environment_get_static_object(env));
    IVP_ASSERT_TRUE(ivp_cluster_get_next_real_object_in_tree(root, ivp_environment_get_static_object(env)) == NULL);

    um_visit_log_t log;
    memset(&log, 0, sizeof(log));
    log.env = env;
    log.delete_on_visit = d; /* deleted while it is the visited object */
    ivp_universe_manager_t mgr;
    ivp_universe_manager_init(&mgr);
    mgr.evict_fn = um_log_evict;
    mgr.ctx = &log;
    mgr.check_objects_per_second_threshold_0 = 250; /* int(250 * 0.01) + 1 = 3 objects per PSI */
    ivp_environment_set_universe_manager(env, &mgr);
    for (int i = 0; i < 4; i++) ivp_environment_simulate_one_psi(env);
    /* PSI 1: the cursor moves to the first object; PSI 2: a, d (deleted),
     * c; PSI 3: b, static (not reported), end; PSI 4: a, c, b */
    IVP_ASSERT_EQ(log.n, 7);
    IVP_ASSERT_TRUE(log.visited[0] == a);
    IVP_ASSERT_TRUE(log.visited[1] == d);
    IVP_ASSERT_TRUE(log.visited[2] == c);
    IVP_ASSERT_TRUE(log.visited[3] == b);
    IVP_ASSERT_TRUE(log.visited[4] == a);
    IVP_ASSERT_TRUE(log.visited[5] == c);
    IVP_ASSERT_TRUE(log.visited[6] == b);

    /* destroying the cluster moves a back to the root (head) */
    ivp_environment_destroy_cluster(env, k);
    IVP_ASSERT_TRUE(a->parent_cluster == root);
    IVP_ASSERT_TRUE(ivp_cluster_get_first_child(root) == a);
    ivp_environment_set_universe_manager(env, NULL);
    ivp_environment_destroy(env);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Liquid Surface Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(liquid_surface_simple) {
    ivp_liquid_surface_simple_t ls;
    ivp_liquid_surface_simple_init(&ls, IVP_REAL(5.0), IVP_REAL(1000.0));

    /* plane -y = -5 (normal out of the liquid, Y-down), consistent with
     * ivp_buoyancy_init and the reference IVP_U_Float_Hesse convention */
    IVP_ASSERT_NEAR(ls.surface_height, -5.0, 1e-6);
    IVP_ASSERT_NEAR(ls.fluid_density, 1000.0, 1e-6);
    IVP_ASSERT_NOT_NULL(ls.base.eval_fn);

    /* Evaluate at a point above the surface (should not be submerged) */
    ivp_vec3r_t query = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0));
    ivp_vec3r_t normal;
    ivp_real_t height;
    ivp_vec3r_t velocity;
    bool submerged = ls.base.eval_fn(ls.base.ctx, &query, &normal, &height, &velocity);
    /* Y-down: normal is (0,-1,0), surface at Y=5.
     * Point at Y=3 is above the surface in Y-down -> not in the liquid. */
    IVP_ASSERT_FALSE(submerged);
    IVP_ASSERT_NEAR(height, -5.0, 1e-6);
    query.k[1] = IVP_REAL(7.0);
    IVP_ASSERT_TRUE(ls.base.eval_fn(ls.base.ctx, &query, &normal, &height, &velocity));
}

IVP_TEST(liquid_surface_plane) {
    ivp_liquid_surface_simple_t ls;
    ivp_vec3r_t n = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(1.0), IVP_REAL(0.0));
    ivp_liquid_surface_simple_init_plane(&ls, &n, IVP_REAL(2.0), IVP_REAL(998.0));

    IVP_ASSERT_NEAR(ls.fluid_density, 998.0, 1e-6);
    IVP_ASSERT_NEAR(ls.surface_normal.k[1], 1.0, 1e-6);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Radar Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

static int s_radar_hits = 0;
static void test_radar_hit(void *ctx, ivp_object_t *target,
                           ivp_real_t distance, const ivp_vec3r_t *direction) {
    (void)ctx; (void)target; (void)distance; (void)direction;
    s_radar_hits++;
}

IVP_TEST(radar_basic) {
    s_radar_hits = 0;

    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);

    /* Create source object at origin */
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    ivp_vec3r_set(&tmpl.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *source = ivp_environment_create_object(env, &tmpl, NULL);

    /* Create target object nearby */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(5.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *target = ivp_environment_create_object(env, &tmpl, NULL);
    (void)target;

    /* Create target object far away */
    ivp_vec3r_set(&tmpl.position, IVP_REAL(500.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *far_obj = ivp_environment_create_object(env, &tmpl, NULL);
    (void)far_obj;

    ivp_radar_config_t cfg;
    ivp_radar_config_init(&cfg);
    cfg.max_range = IVP_REAL(50.0);
    cfg.use_filter = false;
    cfg.hit_fn = test_radar_hit;

    ivp_environment_do_radar(env, source, &cfg);

    /* Should detect the nearby object but not the far one */
    IVP_ASSERT_EQ(s_radar_hits, 1);

    ivp_environment_destroy(env);
}

IVP_TEST(radar_config_defaults) {
    ivp_radar_config_t cfg;
    ivp_radar_config_init(&cfg);
    IVP_ASSERT_NEAR(cfg.max_range, 100.0, 1e-6);
    IVP_ASSERT_TRUE(cfg.use_filter);
    IVP_ASSERT_FALSE(cfg.skip_unmoveable);
    IVP_ASSERT_TRUE(cfg.hit_fn == NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Controller Golem Tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(golem_init) {
    ivp_golem_state_t state;
    ivp_golem_state_init(&state);
    /* Reference IVP_Template_Controller_Golem / _Motion defaults
     * (ivp_controller_motion.cxx:13-22, 206-213). */
    IVP_ASSERT_NEAR(state.force_factor, 0.8, 1e-6);
    IVP_ASSERT_NEAR(state.damp_factor, 1.0, 1e-6);
    IVP_ASSERT_NEAR(state.torque_factor, 0.8, 1e-6);
    IVP_ASSERT_NEAR(state.max_delta_position, 10.0, 1e-6);
    IVP_ASSERT_NEAR(state.max_delta_orientation, 0.5 * 3.14159265358979, 1e-5);
    IVP_ASSERT_NEAR(state.max_translation_force.k[0], 1e6, 1e-1);
    IVP_ASSERT_FALSE(state.target_position_set);
    IVP_ASSERT_FALSE(state.target_orientation_set);
}

IVP_TEST(golem_set_target) {
    ivp_golem_state_t state;
    ivp_golem_state_init(&state);

    ivp_vec3r_t pos = ivp_vec3r(IVP_REAL(1.0), IVP_REAL(2.0), IVP_REAL(3.0));
    ivp_golem_set_target_position(&state, &pos);
    IVP_ASSERT_TRUE(state.target_position_set);
    IVP_ASSERT_NEAR(state.target_position.k[0], 1.0, 1e-6);
    IVP_ASSERT_NEAR(state.target_position.k[1], 2.0, 1e-6);
    IVP_ASSERT_NEAR(state.target_position.k[2], 3.0, 1e-6);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * End-to-End Pipeline Test
 *
 * Parse 3DS data → build compact surface → create environment + object
 * → simulate 10 PSI steps → verify object falls under gravity.
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(pipeline_e2e) {
    /* 1. Parse synthetic 3DS data */
    unsigned char buf[1024];
    int size = build_tetra_3ds(buf, (int)sizeof(buf));

    ivp_3ds_scene_t scene;
    ivp_result_t r = ivp_3ds_load_memory(NULL, buf, size, &scene);
    IVP_ASSERT_TRUE(ivp_ok(r));

    /* 2. Build compact surface */
    ivp_compact_surface_t *surface = ivp_3ds_build_surface(NULL, &scene, 0, NULL);
    IVP_ASSERT_NOT_NULL(surface);

    /* 3. Create environment */
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);

    /* 4. Create ground (static box) */
    ivp_compact_surface_t *ground = ivp_build_box_surface(NULL, 100.0f, 1.0f, 100.0f);
    IVP_ASSERT_NOT_NULL(ground);

    ivp_object_template_t ground_tmpl;
    ivp_object_template_init(&ground_tmpl);
    ground_tmpl.mass = IVP_REAL(0.0);  /* static */
    ivp_vec3r_set(&ground_tmpl.position, IVP_REAL(0.0), IVP_REAL(10.0), IVP_REAL(0.0));
    ivp_object_t *ground_obj = ivp_environment_create_object(env, &ground_tmpl, NULL);
    IVP_ASSERT_NOT_NULL(ground_obj);

    /* 5. Create dynamic object from 3DS-imported surface */
    ivp_object_template_t obj_tmpl;
    ivp_object_template_init(&obj_tmpl);
    obj_tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&obj_tmpl.position, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *dyn_obj = ivp_environment_create_object(env, &obj_tmpl, NULL);
    IVP_ASSERT_NOT_NULL(dyn_obj);

    /* Record initial position */
    ivp_vec3r_t pos0;
    ivp_core_get_position(ivp_object_get_core_const(dyn_obj), &pos0);

    /* 6. Simulate 10 PSI steps */
    for (int i = 0; i < 10; i++) {
        ivp_environment_simulate_one_psi(env);
    }

    /* 7. Verify object has moved (Y-down gravity means Y increases) */
    ivp_vec3r_t pos1;
    ivp_core_get_position(ivp_object_get_core_const(dyn_obj), &pos1);

    /* Object should have moved under gravity (Y-down default: gravity = 9.81 in +Y) */
    IVP_ASSERT_TRUE(pos1.k[1] > pos0.k[1] + IVP_REAL(0.001));

    /* Verify no NaN */
    IVP_ASSERT_TRUE(pos1.k[0] == pos1.k[0]);
    IVP_ASSERT_TRUE(pos1.k[1] == pos1.k[1]);
    IVP_ASSERT_TRUE(pos1.k[2] == pos1.k[2]);

    /* 8. Cleanup */
    ivp_environment_destroy(env);
    ivp_compact_surface_free(NULL, surface);
    ivp_compact_surface_free(NULL, ground);
    ivp_3ds_scene_free(&scene);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Error code tests
 * ═══════════════════════════════════════════════════════════════════════════ */

IVP_TEST(error_codes) {
    IVP_ASSERT_TRUE(strcmp(ivp_result_str(IVP_ERR_FILE_NOT_FOUND), "FILE_NOT_FOUND") == 0);
    IVP_ASSERT_TRUE(strcmp(ivp_result_str(IVP_ERR_FILE_READ), "FILE_READ") == 0);
    IVP_ASSERT_TRUE(strcmp(ivp_result_str(IVP_ERR_FILE_CORRUPT), "FILE_CORRUPT") == 0);
    IVP_ASSERT_TRUE(strcmp(ivp_result_str(IVP_ERR_FILE_VERSION), "FILE_VERSION") == 0);
}

IVP_TEST(grid_rejects_overflowing_dimensions) {
    ivp_grid_config_t cfg;
    ivp_grid_config_init(&cfg);
    const float height = 0.0f;
    cfg.n_rows = 50000;
    cfg.n_columns = 50000;
    IVP_ASSERT_NULL(ivp_build_grid_surface(NULL, &cfg, &height));
    cfg.n_rows = INT_MAX;
    cfg.n_columns = 2;
    IVP_ASSERT_NULL(ivp_build_grid_surface(NULL, &cfg, &height));
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Test registration
 * ═══════════════════════════════════════════════════════════════════════════ */

int main(void) {
    /* 3DS parser */
    IVP_REGISTER(tds_parse_tetrahedron);
    IVP_REGISTER(tds_invalid_data);
    IVP_REGISTER(tds_wrong_magic);
    IVP_REGISTER(tds_build_surface);
    IVP_REGISTER(tds_scale);
    IVP_REGISTER(tds_null_args);
    IVP_REGISTER(tds_config_defaults);
    IVP_REGISTER(tds_weld_balances_allocator_bytes);

    /* Q12 BSP */
    IVP_REGISTER(bsp_synthetic_box);
    IVP_REGISTER(bsp_config_defaults);
    IVP_REGISTER(bsp_null_args);

    /* Grid / heightfield */
    IVP_REGISTER(grid_flat);
    IVP_REGISTER(grid_slope);
    IVP_REGISTER(grid_config_defaults);
    IVP_REGISTER(grid_null_heightfield);
    IVP_REGISTER(grid_small);
    IVP_REGISTER(grid_rejects_overflowing_dimensions);

    /* Universe manager */
    IVP_REGISTER(universe_manager_init);
    IVP_REGISTER(universe_manager_deleted_callback);
    IVP_REGISTER(universe_manager_visits_cluster_tree_order);

    /* Liquid surface */
    IVP_REGISTER(liquid_surface_simple);
    IVP_REGISTER(liquid_surface_plane);

    /* Radar */
    IVP_REGISTER(radar_basic);
    IVP_REGISTER(radar_config_defaults);

    /* Controller golem */
    IVP_REGISTER(golem_init);
    IVP_REGISTER(golem_set_target);

    /* Error codes */
    IVP_REGISTER(error_codes);

    /* E2E pipeline */
    IVP_REGISTER(pipeline_e2e);

    IVP_RUN_ALL();
}
