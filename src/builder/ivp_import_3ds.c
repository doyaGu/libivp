/* ivp_import_3ds.c — 3DS file import and surface builder
 *
 * Parses Autodesk 3DS binary chunk format. Supports file and memory I/O.
 * Ported from ivp_compact_builder/3dsimport_load.cxx and 3dsimport_co.cxx.
 */

#include <ivp/builder/ivp_import_3ds.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/base/ivp_message.h>
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ── 3DS chunk IDs ─────────────────────────────────────────────────────── */

#define CHUNK_MAIN      0x4D4D
#define CHUNK_OBJMESH   0x3D3D
#define CHUNK_OBJBLOCK  0x4000
#define CHUNK_TRIMESH   0x4100
#define CHUNK_VERTLIST  0x4110
#define CHUNK_FACELIST  0x4120
#define CHUNK_MAPLIST   0x4140
#define CHUNK_TRMATRIX  0x4160

/* ── Reader context (replaces setjmp/longjmp with error flag) ──────────── */

typedef struct reader_ctx {
    const unsigned char *data;
    int                  size;
    int                  pos;
    bool                 error;
} reader_ctx_t;

static void reader_init(reader_ctx_t *r, const void *data, int size)
{
    r->data  = (const unsigned char *)data;
    r->size  = size;
    r->pos   = 0;
    r->error = false;
}

static bool reader_can_read(const reader_ctx_t *r, int bytes)
{
    return !r->error && bytes >= 0 && r->pos >= 0 && r->pos <= r->size &&
           bytes <= r->size - r->pos;
}

static uint16_t reader_u16(reader_ctx_t *r)
{
    if (!reader_can_read(r, 2)) { r->error = true; return 0; }
    uint16_t v = (uint16_t)(r->data[r->pos] | (r->data[r->pos + 1] << 8));
    r->pos += 2;
    return v;
}

static uint32_t reader_u32(reader_ctx_t *r)
{
    if (!reader_can_read(r, 4)) { r->error = true; return 0; }
    uint32_t v = (uint32_t)r->data[r->pos]
               | ((uint32_t)r->data[r->pos + 1] << 8)
               | ((uint32_t)r->data[r->pos + 2] << 16)
               | ((uint32_t)r->data[r->pos + 3] << 24);
    r->pos += 4;
    return v;
}

static float reader_f32(reader_ctx_t *r)
{
    uint32_t bits = reader_u32(r);
    float f;
    memcpy(&f, &bits, sizeof(f));
    return f;
}

static void reader_read_name(reader_ctx_t *r, char *buf, int max_len)
{
    int n = 0;
    while (reader_can_read(r, 1)) {
        char c = (char)r->data[r->pos++];
        if (n < max_len - 1) buf[n++] = c;
        if (c == '\0') return;
    }
    buf[n < max_len ? n : max_len - 1] = '\0';
}

static void reader_skip(reader_ctx_t *r, int bytes)
{
    if (!reader_can_read(r, bytes)) { r->error = true; return; }
    r->pos += bytes;
}

/* ── Temporary mesh accumulator ────────────────────────────────────────── */

#define MAX_3DS_MESHES 256

typedef struct mesh_accum {
    float *verts;       /* [n_verts * 3] */
    int   *faces;       /* [n_faces * 3] */
    int    n_verts;
    int    n_faces;
    int    verts_cap;
    int    faces_cap;
    char   name[64];
} mesh_accum_t;

typedef struct parse_state {
    mesh_accum_t        meshes[MAX_3DS_MESHES];
    int                 n_meshes;
    const ivp_allocator_t *alloc;
} parse_state_t;

static mesh_accum_t *state_new_mesh(parse_state_t *st, const char *name)
{
    if (st->n_meshes >= MAX_3DS_MESHES) return NULL;
    mesh_accum_t *m = &st->meshes[st->n_meshes++];
    memset(m, 0, sizeof(*m));
    if (name) {
        size_t len = strlen(name);
        if (len >= sizeof(m->name)) len = sizeof(m->name) - 1;
        memcpy(m->name, name, len);
        m->name[len] = '\0';
    }
    return m;
}

/* ── Chunk parser ──────────────────────────────────────────────────────── */

static void parse_vertlist(reader_ctx_t *r, mesh_accum_t *mesh, const ivp_allocator_t *alloc)
{
    uint16_t n_verts = reader_u16(r);
    if (r->error || n_verts == 0) return;

    mesh->n_verts = n_verts;
    mesh->verts_cap = n_verts;
    size_t sz = (size_t)n_verts * 3 * sizeof(float);
    mesh->verts = (float *)ivp_alloc(alloc, sz);
    if (!mesh->verts) { r->error = true; return; }

    for (int i = 0; i < n_verts; i++) {
        mesh->verts[i * 3 + 0] = reader_f32(r);
        mesh->verts[i * 3 + 1] = reader_f32(r);
        mesh->verts[i * 3 + 2] = reader_f32(r);
    }
}

static void parse_facelist(reader_ctx_t *r, mesh_accum_t *mesh, const ivp_allocator_t *alloc)
{
    uint16_t n_faces = reader_u16(r);
    if (r->error || n_faces == 0) return;

    mesh->n_faces = n_faces;
    mesh->faces_cap = n_faces;
    size_t sz = (size_t)n_faces * 3 * sizeof(int);
    mesh->faces = (int *)ivp_alloc(alloc, sz);
    if (!mesh->faces) { r->error = true; return; }

    for (int i = 0; i < n_faces; i++) {
        mesh->faces[i * 3 + 0] = (int)reader_u16(r);
        mesh->faces[i * 3 + 1] = (int)reader_u16(r);
        mesh->faces[i * 3 + 2] = (int)reader_u16(r);
        reader_u16(r); /* flags — unused for physics */
    }
}

static void parse_trimesh(reader_ctx_t *r, int chunk_end, mesh_accum_t *mesh,
                          const ivp_allocator_t *alloc)
{
    while (r->pos < chunk_end && !r->error) {
        if (!reader_can_read(r, 6)) break;
        uint16_t id   = reader_u16(r);
        uint32_t size = reader_u32(r);
        int sub_end   = r->pos + (int)size - 6;
        if (sub_end > chunk_end) { r->error = true; return; }

        switch (id) {
            case CHUNK_VERTLIST: parse_vertlist(r, mesh, alloc); break;
            case CHUNK_FACELIST: parse_facelist(r, mesh, alloc); break;
            default: reader_skip(r, (int)size - 6); break;
        }
        r->pos = sub_end;
    }
}

static void parse_objblock(reader_ctx_t *r, int chunk_end, parse_state_t *st)
{
    char name[64];
    reader_read_name(r, name, sizeof(name));

    mesh_accum_t *mesh = NULL;

    while (r->pos < chunk_end && !r->error) {
        if (!reader_can_read(r, 6)) break;
        uint16_t id   = reader_u16(r);
        uint32_t size = reader_u32(r);
        int sub_end   = r->pos + (int)size - 6;
        if (sub_end > chunk_end) { r->error = true; return; }

        if (id == CHUNK_TRIMESH) {
            mesh = state_new_mesh(st, name);
            if (!mesh) { r->error = true; return; }
            parse_trimesh(r, sub_end, mesh, st->alloc);
        } else {
            reader_skip(r, (int)size - 6);
        }
        r->pos = sub_end;
    }
}

static void parse_objmesh(reader_ctx_t *r, int chunk_end, parse_state_t *st)
{
    while (r->pos < chunk_end && !r->error) {
        if (!reader_can_read(r, 6)) break;
        uint16_t id   = reader_u16(r);
        uint32_t size = reader_u32(r);
        int sub_end   = r->pos + (int)size - 6;
        if (sub_end > chunk_end) { r->error = true; return; }

        if (id == CHUNK_OBJBLOCK) {
            parse_objblock(r, sub_end, st);
        } else {
            reader_skip(r, (int)size - 6);
        }
        r->pos = sub_end;
    }
}

static ivp_result_t parse_3ds(reader_ctx_t *r, parse_state_t *st)
{
    if (!reader_can_read(r, 6)) return IVP_ERR_FILE_CORRUPT;

    uint16_t id   = reader_u16(r);
    uint32_t size = reader_u32(r);

    if (id != CHUNK_MAIN) return IVP_ERR_FILE_CORRUPT;

    int main_end = (int)size;
    if (main_end > r->size) main_end = r->size;

    while (r->pos < main_end && !r->error) {
        if (!reader_can_read(r, 6)) break;
        uint16_t sub_id   = reader_u16(r);
        uint32_t sub_size = reader_u32(r);
        int sub_end       = r->pos + (int)sub_size - 6;
        if (sub_end > main_end) { r->error = true; break; }

        if (sub_id == CHUNK_OBJMESH) {
            parse_objmesh(r, sub_end, st);
        } else {
            reader_skip(r, (int)sub_size - 6);
        }
        r->pos = sub_end;
    }

    if (r->error) return IVP_ERR_FILE_CORRUPT;
    if (st->n_meshes == 0) return IVP_ERR_FILE_CORRUPT;
    return IVP_OK;
}

/* ── Degenerate face removal ───────────────────────────────────────────── */

static void remove_degenerate_faces(ivp_3ds_mesh_t *mesh)
{
    int dst = 0;
    for (int i = 0; i < mesh->n_triangles; i++) {
        int i0 = mesh->indices[i * 3 + 0];
        int i1 = mesh->indices[i * 3 + 1];
        int i2 = mesh->indices[i * 3 + 2];
        if (i0 == i1 || i0 == i2 || i1 == i2) continue;

        /* Check for zero-area triangles */
        float *v0 = &mesh->vertices[i0 * 3];
        float *v1 = &mesh->vertices[i1 * 3];
        float *v2 = &mesh->vertices[i2 * 3];
        float ex = v1[0] - v0[0], ey = v1[1] - v0[1], ez = v1[2] - v0[2];
        float fx = v2[0] - v0[0], fy = v2[1] - v0[1], fz = v2[2] - v0[2];
        float cx = ey * fz - ez * fy;
        float cy = ez * fx - ex * fz;
        float cz = ex * fy - ey * fx;
        float area2 = cx * cx + cy * cy + cz * cz;
        if (area2 < 1e-12f) continue;

        mesh->indices[dst * 3 + 0] = i0;
        mesh->indices[dst * 3 + 1] = i1;
        mesh->indices[dst * 3 + 2] = i2;
        dst++;
    }
    mesh->n_triangles = dst;
}

/* ── Vertex welding (dedup within epsilon) ─────────────────────────────── */

static void weld_vertices(ivp_3ds_mesh_t *mesh, float epsilon,
                          const ivp_allocator_t *alloc)
{
    if (mesh->n_vertices <= 1) return;

    float eps2 = epsilon * epsilon;
    const size_t remap_size = (size_t)mesh->n_vertices * sizeof(int);
    int *remap = (int *)ivp_alloc(alloc, remap_size);
    if (!remap) return;

    int new_count = 0;
    for (int i = 0; i < mesh->n_vertices; i++) {
        remap[i] = -1;
        float *vi = &mesh->vertices[i * 3];
        for (int j = 0; j < new_count; j++) {
            float *vj = &mesh->vertices[j * 3];
            float dx = vi[0] - vj[0];
            float dy = vi[1] - vj[1];
            float dz = vi[2] - vj[2];
            if (dx * dx + dy * dy + dz * dz < eps2) {
                remap[i] = j;
                break;
            }
        }
        if (remap[i] < 0) {
            remap[i] = new_count;
            mesh->vertices[new_count * 3 + 0] = vi[0];
            mesh->vertices[new_count * 3 + 1] = vi[1];
            mesh->vertices[new_count * 3 + 2] = vi[2];
            new_count++;
        }
    }

    /* Remap face indices */
    for (int i = 0; i < mesh->n_triangles * 3; i++) {
        mesh->indices[i] = remap[mesh->indices[i]];
    }

    mesh->n_vertices = new_count;
    ivp_free(alloc, remap, remap_size);
}

/* ── Apply scale ───────────────────────────────────────────────────────── */

static void apply_scale(ivp_3ds_mesh_t *mesh, float scale)
{
    if (scale == 1.0f) return;
    for (int i = 0; i < mesh->n_vertices * 3; i++) {
        mesh->vertices[i] *= scale;
    }
}

/* ── Center mesh at origin ─────────────────────────────────────────────── */

static void center_mesh(ivp_3ds_mesh_t *mesh)
{
    if (mesh->n_vertices == 0) return;
    float cx = 0, cy = 0, cz = 0;
    for (int i = 0; i < mesh->n_vertices; i++) {
        cx += mesh->vertices[i * 3 + 0];
        cy += mesh->vertices[i * 3 + 1];
        cz += mesh->vertices[i * 3 + 2];
    }
    float inv = 1.0f / (float)mesh->n_vertices;
    cx *= inv; cy *= inv; cz *= inv;
    for (int i = 0; i < mesh->n_vertices; i++) {
        mesh->vertices[i * 3 + 0] -= cx;
        mesh->vertices[i * 3 + 1] -= cy;
        mesh->vertices[i * 3 + 2] -= cz;
    }
}

/* ── Convert parse state → scene ───────────────────────────────────────── */

static ivp_result_t build_scene_from_state(const ivp_allocator_t *alloc,
                                           parse_state_t *st,
                                           ivp_3ds_scene_t *out)
{
    out->allocator = alloc;
    out->n_meshes  = st->n_meshes;
    out->meshes    = (ivp_3ds_mesh_t *)ivp_alloc(alloc,
                        (size_t)st->n_meshes * sizeof(ivp_3ds_mesh_t));
    if (!out->meshes) return IVP_ERR_OUT_OF_MEMORY;

    for (int i = 0; i < st->n_meshes; i++) {
        mesh_accum_t *src = &st->meshes[i];
        ivp_3ds_mesh_t *dst = &out->meshes[i];

        dst->n_vertices  = src->n_verts;
        dst->n_triangles = src->n_faces;
        memcpy(dst->name, src->name, sizeof(dst->name));

        /* Transfer ownership of vertex/index arrays */
        dst->vertices = src->verts;
        dst->indices  = src->faces;
        src->verts = NULL;
        src->faces = NULL;
    }

    return IVP_OK;
}

static void free_parse_state(parse_state_t *st)
{
    for (int i = 0; i < st->n_meshes; i++) {
        if (st->meshes[i].verts)
            ivp_free(st->alloc, st->meshes[i].verts,
                     (size_t)st->meshes[i].n_verts * 3 * sizeof(float));
        if (st->meshes[i].faces)
            ivp_free(st->alloc, st->meshes[i].faces,
                     (size_t)st->meshes[i].n_faces * 3 * sizeof(int));
    }
}

/* ── Public API ────────────────────────────────────────────────────────── */

void ivp_3ds_import_config_init(ivp_3ds_import_config_t *cfg)
{
    cfg->scale            = 1.0f;
    cfg->weld_epsilon     = 1e-4f;
    cfg->remove_degenerate = true;
    cfg->center_meshes    = false;
}

ivp_result_t ivp_3ds_load_memory(const ivp_allocator_t *alloc,
                                 const void *data, int size,
                                 ivp_3ds_scene_t *out)
{
    if (!data || !out) return IVP_ERR_INVALID_ARG;
    if (size < 6) return IVP_ERR_FILE_CORRUPT;
    if (!alloc) alloc = ivp_default_allocator();

    memset(out, 0, sizeof(*out));

    reader_ctx_t reader;
    reader_init(&reader, data, size);

    parse_state_t state;
    memset(&state, 0, sizeof(state));
    state.alloc = alloc;

    ivp_result_t result = parse_3ds(&reader, &state);
    if (!ivp_ok(result)) {
        free_parse_state(&state);
        return result;
    }

    result = build_scene_from_state(alloc, &state, out);
    if (!ivp_ok(result)) {
        free_parse_state(&state);
        return result;
    }

    return IVP_OK;
}

ivp_result_t ivp_3ds_load_file(const ivp_allocator_t *alloc,
                               const char *filename,
                               ivp_3ds_scene_t *out)
{
    /* the file part of IVP_SurfaceBuilder_3ds::convert_3ds_to_concave
     * (3dsimport_co.cxx:617-708) with its messages */
    if (!out) return IVP_ERR_INVALID_ARG;
    if (!filename) {
        ivp_message("No inputfile specified!\n");
        return IVP_ERR_INVALID_ARG;
    }
    if (!alloc) alloc = ivp_default_allocator();

    memset(out, 0, sizeof(*out));

    FILE *f = fopen(filename, "rb");
    if (!f) {
        ivp_message("Failed to open %s\n", filename);
        return IVP_ERR_FILE_NOT_FOUND;
    }

    if (fseek(f, 0, SEEK_END)) {
        ivp_message("Error seeking %s\n", filename);
        fclose(f);
        return IVP_ERR_FILE_READ;
    }
    long len = ftell(f);
    if (len == -1L) {
        ivp_message("Error seeking %s\n", filename);
        fclose(f);
        return IVP_ERR_FILE_READ;
    }
    fseek(f, 0, SEEK_SET);

    /* HRead3dsScene(inf, 0, size) == 0: "Failed to load" */
    if (len <= 0 || len > 256 * 1024 * 1024) {
        ivp_message("Failed to load %s\n", filename);
        fclose(f);
        return IVP_ERR_FILE_READ;
    }

    void *buf = ivp_alloc(alloc, (size_t)len);
    if (!buf) {
        ivp_message("Failed to load %s\n", filename);
        fclose(f);
        return IVP_ERR_OUT_OF_MEMORY;
    }

    size_t read = fread(buf, 1, (size_t)len, f);
    fclose(f);

    if ((long)read != len) {
        ivp_message("Failed to load %s\n", filename);
        ivp_free(alloc, buf, (size_t)len);
        return IVP_ERR_FILE_READ;
    }

    ivp_result_t result = ivp_3ds_load_memory(alloc, buf, (int)len, out);
    ivp_free(alloc, buf, (size_t)len);
    if (!ivp_ok(result)) ivp_message("Failed to load %s\n", filename);
    return result;
}

void ivp_3ds_scene_free(ivp_3ds_scene_t *scene)
{
    if (!scene || !scene->meshes) return;
    const ivp_allocator_t *alloc = scene->allocator;
    if (!alloc) alloc = ivp_default_allocator();

    for (int i = 0; i < scene->n_meshes; i++) {
        ivp_3ds_mesh_t *m = &scene->meshes[i];
        if (m->vertices)
            ivp_free(alloc, m->vertices,
                     (size_t)m->n_vertices * 3 * sizeof(float));
        if (m->indices)
            ivp_free(alloc, m->indices,
                     (size_t)m->n_triangles * 3 * sizeof(int));
    }
    ivp_free(alloc, scene->meshes,
             (size_t)scene->n_meshes * sizeof(ivp_3ds_mesh_t));

    memset(scene, 0, sizeof(*scene));
}

ivp_compact_surface_t *ivp_3ds_build_surface(const ivp_allocator_t *alloc,
                                             const ivp_3ds_scene_t *scene,
                                             int mesh_index,
                                             const ivp_3ds_import_config_t *cfg)
{
    if (!scene || mesh_index < 0 || mesh_index >= scene->n_meshes) return NULL;
    if (!alloc) alloc = ivp_default_allocator();

    ivp_3ds_import_config_t defaults;
    if (!cfg) {
        ivp_3ds_import_config_init(&defaults);
        cfg = &defaults;
    }

    const ivp_3ds_mesh_t *src = &scene->meshes[mesh_index];
    if (src->n_vertices < 3 || src->n_triangles < 1) return NULL;

    /* Make a mutable copy of vertices/indices for processing */
    size_t vert_sz = (size_t)src->n_vertices * 3 * sizeof(float);
    size_t idx_sz  = (size_t)src->n_triangles * 3 * sizeof(int);

    float *verts = (float *)ivp_alloc(alloc, vert_sz);
    int   *inds  = (int *)ivp_alloc(alloc, idx_sz);
    if (!verts || !inds) {
        if (verts) ivp_free(alloc, verts, vert_sz);
        if (inds)  ivp_free(alloc, inds, idx_sz);
        return NULL;
    }

    memcpy(verts, src->vertices, vert_sz);
    memcpy(inds, src->indices, idx_sz);

    /* Build temporary mesh for processing */
    ivp_3ds_mesh_t tmp;
    tmp.vertices    = verts;
    tmp.indices     = inds;
    tmp.n_vertices  = src->n_vertices;
    tmp.n_triangles = src->n_triangles;

    /* Apply processing pipeline */
    apply_scale(&tmp, cfg->scale);
    if (cfg->center_meshes) center_mesh(&tmp);
    if (cfg->weld_epsilon > 0.0f) weld_vertices(&tmp, cfg->weld_epsilon, alloc);
    if (cfg->remove_degenerate) remove_degenerate_faces(&tmp);

    if (tmp.n_triangles < 1 || tmp.n_vertices < 3) {
        ivp_free(alloc, verts, vert_sz);
        ivp_free(alloc, inds, idx_sz);
        return NULL;
    }

    /* Reference p_graphlib_robust_convert_3dmax_object_to_compact_ledges
     * (3dsimport_co.cxx:905): every face becomes its own pointsoup ledge
     * (a two sided triangle, convert_pointsoup_to_compact_ledge with 3
     * points) and the ledges are combined by the ledge soup builder. */
    ivp_compact_surface_t *surface = NULL;
    {
        ivp_ledge_soup_t soup;
        ivp_ledge_soup_init(&soup, alloc);
        ivp_ledge_soup_template_t lt;
        ivp_ledge_soup_template_init(&lt);
        lt.free_input_compact_ledges = true; /* the ledges are ours */
        bool ok = true;
        int n_added = 0;
        for (int t = 0; t < tmp.n_triangles && ok; t++) {
            double tri[9];
            for (int k = 0; k < 3; k++) {
                const float *v = &tmp.vertices[3 * tmp.indices[3 * t + k]];
                tri[3 * k + 0] = (double)v[0];
                tri[3 * k + 1] = (double)v[1];
                tri[3 * k + 2] = (double)v[2];
            }
            ivp_compact_ledge_t *l = ivp_build_convex_ledge_d(alloc, tri, 3);
            if (!l) continue; /* degenerate face: skipped like the reference */
            if (ivp_ledge_soup_add(&soup, l) != 0) {
                ivp_compact_ledge_free(alloc, l);
                ok = false;
                break;
            }
            n_added++;
        }
        if (ok && n_added > 0) {
            surface = ivp_ledge_soup_compile_ex(&soup, &lt);
            if (!surface) {
                for (int i = 0; i < soup.n_ledges; i++) {
                    ivp_compact_ledge_free(alloc, (ivp_compact_ledge_t *)soup.ledges[i]);
                }
            }
        } else {
            for (int i = 0; i < soup.n_ledges; i++) {
                ivp_compact_ledge_free(alloc, (ivp_compact_ledge_t *)soup.ledges[i]);
            }
        }
        ivp_ledge_soup_destroy(&soup);
    }

    ivp_free(alloc, verts, vert_sz);
    ivp_free(alloc, inds, idx_sz);

    return surface;
}

ivp_compact_surface_t *ivp_3ds_load_and_build(const ivp_allocator_t *alloc,
                                              const char *filename,
                                              const ivp_3ds_import_config_t *cfg)
{
    if (!alloc) alloc = ivp_default_allocator();

    ivp_3ds_scene_t scene;
    ivp_result_t result = ivp_3ds_load_file(alloc, filename, &scene);
    if (!ivp_ok(result)) return NULL;

    ivp_compact_surface_t *surface = ivp_3ds_build_surface(alloc, &scene, 0, cfg);
    ivp_3ds_scene_free(&scene);

    return surface;
}
