/* ivp_build_q12bsp.c — Quake 1/2 BSP to compact surface builder
 *
 * Recursive BSP traversal: each solid leaf collects planes along the
 * path from root → leaf, builds a convex piece via half-space intersection,
 * then all pieces are assembled via ledge soup compiler.
 *
 * Coordinate transform: Quake Y↔Z swap, negate Z.
 * Ported from ivp_compact_builder/ivp_surbuild_q12.cxx.
 */

#include <ivp/builder/ivp_build_q12bsp.h>
#include <ivp/builder/ivp_build_halfspacesoup.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/builder/ivp_build_concave.h>
#include <string.h>
#include <stdio.h>

/* ── BSP file format constants ─────────────────────────────────────────── */

#define BSP_VERSION_Q1    29
#define BSP_LUMP_COUNT    15

#define LUMP_PLANES       1
#define LUMP_NODES        5
#define LUMP_CLIPNODES    9
#define LUMP_MODELS      14

/* Leaf type for solid */
#define CONTENTS_SOLID   -1
#define CONTENTS_EMPTY   -2

/* ── Traversal context ─────────────────────────────────────────────────── */

#define MAX_BSP_DEPTH 256
#define MAX_PLANES_PER_LEAF 128

typedef struct bsp_traverse_ctx {
    const ivp_q12bsp_t       *bsp;
    const ivp_q12bsp_config_t *cfg;
    const ivp_allocator_t     *alloc;

    /* Path stack: node indices from root to current position */
    int   path[MAX_BSP_DEPTH];
    int   path_side[MAX_BSP_DEPTH]; /* 0 = went left, 1 = went right */
    int   depth;

    /* Output: assembled ledge soup */
    ivp_ledge_soup_t  soup;

    /* Intermediate: convex surfaces built for each solid leaf */
    ivp_compact_surface_t **parts;
    int n_parts;
    int parts_cap;
} bsp_traverse_ctx_t;

/* ── Plane insertion with Quake→IVP coordinate swap ────────────────────── */

static void ctx_add_plane(ivp_halfspace_t *planes, int *n_planes,
                          float nx, float ny, float nz, float dist,
                          float scale, float shrink)
{
    if (*n_planes >= MAX_PLANES_PER_LEAF) return;
    ivp_halfspace_t *p = &planes[*n_planes];
    /* Quake→IVP: swap Y↔Z, negate Z */
    p->normal[0] = nx;
    p->normal[1] = -nz;
    p->normal[2] = ny;
    p->dist = -(dist + shrink) * scale;
    (*n_planes)++;
}

/* ── Convert solid leaf ────────────────────────────────────────────────── */

static void convert_solid_leaf(bsp_traverse_ctx_t *ctx)
{
    ivp_halfspace_t planes[MAX_PLANES_PER_LEAF];
    int n_planes = 0;

    float scale  = ctx->cfg->scale;
    float shrink = ctx->cfg->shrink_value;

    /* Extract planes from the path: each node contributes its splitting plane.
     * The side we descended into determines whether the plane is flipped. */
    for (int i = 0; i < ctx->depth; i++) {
        int node_idx = ctx->path[i];
        int side     = ctx->path_side[i];
        const ivp_q12_plane_t *bp = &ctx->bsp->planes[ctx->bsp->nodes[node_idx].planenum];

        float nx = bp->normal[0];
        float ny = bp->normal[1];
        float nz = bp->normal[2];
        float d  = bp->dist;

        if (side == 0) {
            /* Went to positive child → this side of the plane */
            ctx_add_plane(planes, &n_planes, nx, ny, nz, d, scale, shrink);
        } else {
            /* Went to negative child → flip plane */
            ctx_add_plane(planes, &n_planes, -nx, -ny, -nz, -d, scale, shrink);
        }
    }

    if (n_planes < 4) {
        /* Add bounding box planes from the model to close the volume */
        const ivp_q12_model_t *model = &ctx->bsp->models[0];
        float margin = 1.0f;
        ctx_add_plane(planes, &n_planes,  1, 0, 0, model->maxs[0] + margin, scale, shrink);
        ctx_add_plane(planes, &n_planes, -1, 0, 0, -(model->mins[0] - margin), scale, shrink);
        ctx_add_plane(planes, &n_planes, 0,  1, 0, model->maxs[1] + margin, scale, shrink);
        ctx_add_plane(planes, &n_planes, 0, -1, 0, -(model->mins[1] - margin), scale, shrink);
        ctx_add_plane(planes, &n_planes, 0, 0,  1, model->maxs[2] + margin, scale, shrink);
        ctx_add_plane(planes, &n_planes, 0, 0, -1, -(model->mins[2] - margin), scale, shrink);
    }

    /* Build convex surface from halfspace intersection */
    ivp_compact_surface_t *part = ivp_build_halfspacesoup_surface(
        ctx->alloc, planes, n_planes, ctx->cfg->pointmerge_threshold);

    if (part) {
        /* Store part for later assembly */
        if (ctx->n_parts >= ctx->parts_cap) {
            int new_cap = ctx->parts_cap * 2;
            if (new_cap < 16) new_cap = 16;
            size_t old_sz = (size_t)ctx->parts_cap * sizeof(ivp_compact_surface_t *);
            size_t new_sz = (size_t)new_cap * sizeof(ivp_compact_surface_t *);
            ivp_compact_surface_t **new_arr = (ivp_compact_surface_t **)
                ivp_realloc(ctx->alloc, ctx->parts, old_sz, new_sz);
            if (!new_arr) {
                ivp_compact_surface_free(ctx->alloc, part);
                return;
            }
            ctx->parts = new_arr;
            ctx->parts_cap = new_cap;
        }
        ctx->parts[ctx->n_parts++] = part;
    }
}

/* ── Recursive BSP traversal ───────────────────────────────────────────── */

static void traverse_node(bsp_traverse_ctx_t *ctx, int node_idx)
{
    if (ctx->depth >= MAX_BSP_DEPTH) return;
    if (node_idx < 0 || node_idx >= ctx->bsp->n_nodes) return;

    const ivp_q12_node_t *node = &ctx->bsp->nodes[node_idx];

    ctx->path[ctx->depth] = node_idx;

    /* Traverse left child (positive side) */
    ctx->path_side[ctx->depth] = 0;
    ctx->depth++;
    if (node->children[0] > 0) {
        traverse_node(ctx, node->children[0]);
    } else if (node->children[0] == CONTENTS_SOLID) {
        convert_solid_leaf(ctx);
    }
    ctx->depth--;

    /* Traverse right child (negative side) */
    ctx->path_side[ctx->depth] = 1;
    ctx->depth++;
    if (node->children[1] > 0) {
        traverse_node(ctx, node->children[1]);
    } else if (node->children[1] == CONTENTS_SOLID) {
        convert_solid_leaf(ctx);
    }
    ctx->depth--;
}

/* ── Public API ────────────────────────────────────────────────────────── */

void ivp_q12bsp_config_init(ivp_q12bsp_config_t *cfg)
{
    cfg->scale                = 1.0f;
    cfg->shrink_value         = 0.0f;
    cfg->pointmerge_threshold = 0.01f;
}

void ivp_q12bsp_init_from_memory(ivp_q12bsp_t *bsp,
                                 ivp_q12_model_t *models, int n_models,
                                 ivp_q12_plane_t *planes, int n_planes,
                                 ivp_q12_node_t *nodes, int n_nodes,
                                 ivp_q12_clipnode_t *clipnodes, int n_clipnodes)
{
    memset(bsp, 0, sizeof(*bsp));
    bsp->models     = models;
    bsp->planes     = planes;
    bsp->nodes      = nodes;
    bsp->clipnodes  = clipnodes;
    bsp->n_models   = n_models;
    bsp->n_planes   = n_planes;
    bsp->n_nodes    = n_nodes;
    bsp->n_clipnodes = n_clipnodes;
    bsp->owns_data  = false;
    bsp->allocator  = NULL;
}

void ivp_q12bsp_free(ivp_q12bsp_t *bsp)
{
    if (!bsp || !bsp->owns_data) return;
    const ivp_allocator_t *alloc = bsp->allocator;
    if (!alloc) alloc = ivp_default_allocator();

    if (bsp->models)
        ivp_free(alloc, bsp->models, (size_t)bsp->n_models * sizeof(ivp_q12_model_t));
    if (bsp->planes)
        ivp_free(alloc, bsp->planes, (size_t)bsp->n_planes * sizeof(ivp_q12_plane_t));
    if (bsp->nodes)
        ivp_free(alloc, bsp->nodes, (size_t)bsp->n_nodes * sizeof(ivp_q12_node_t));
    if (bsp->clipnodes)
        ivp_free(alloc, bsp->clipnodes, (size_t)bsp->n_clipnodes * sizeof(ivp_q12_clipnode_t));

    memset(bsp, 0, sizeof(*bsp));
}

ivp_result_t ivp_q12bsp_load_file(const ivp_allocator_t *alloc,
                                  const char *filename,
                                  ivp_q12bsp_t *out)
{
    if (!filename || !out) return IVP_ERR_INVALID_ARG;
    if (!alloc) alloc = ivp_default_allocator();

    memset(out, 0, sizeof(*out));

    FILE *f = fopen(filename, "rb");
    if (!f) return IVP_ERR_FILE_NOT_FOUND;

    /* Read BSP header: version + 15 lumps */
    int32_t version;
    if (fread(&version, 4, 1, f) != 1) { fclose(f); return IVP_ERR_FILE_READ; }
    if (version != BSP_VERSION_Q1) { fclose(f); return IVP_ERR_FILE_VERSION; }

    struct { int32_t offset, length; } lumps[BSP_LUMP_COUNT];
    if (fread(lumps, sizeof(lumps), 1, f) != 1) { fclose(f); return IVP_ERR_FILE_READ; }

    /* Helper macro to load a lump into an array */
    #define LOAD_LUMP(lump_idx, arr, count, type) do {                         \
        int32_t off = lumps[lump_idx].offset;                                  \
        int32_t len = lumps[lump_idx].length;                                  \
        if (len <= 0) { (arr) = NULL; (count) = 0; break; }                   \
        (count) = len / (int32_t)sizeof(type);                                 \
        (arr) = (type *)ivp_alloc(alloc, (size_t)len);                         \
        if (!(arr)) { fclose(f); ivp_q12bsp_free(out); return IVP_ERR_OUT_OF_MEMORY; } \
        fseek(f, off, SEEK_SET);                                               \
        if (fread((arr), 1, (size_t)len, f) != (size_t)len) {                  \
            fclose(f); ivp_q12bsp_free(out); return IVP_ERR_FILE_READ;         \
        }                                                                      \
    } while (0)

    out->allocator = alloc;
    out->owns_data = true;

    LOAD_LUMP(LUMP_MODELS,    out->models,    out->n_models,    ivp_q12_model_t);
    LOAD_LUMP(LUMP_PLANES,    out->planes,    out->n_planes,    ivp_q12_plane_t);
    LOAD_LUMP(LUMP_NODES,     out->nodes,     out->n_nodes,     ivp_q12_node_t);
    LOAD_LUMP(LUMP_CLIPNODES, out->clipnodes, out->n_clipnodes, ivp_q12_clipnode_t);

    #undef LOAD_LUMP

    fclose(f);

    if (out->n_models == 0 || out->n_planes == 0 || out->n_nodes == 0)
        return IVP_ERR_FILE_CORRUPT;

    return IVP_OK;
}

ivp_compact_surface_t *ivp_q12bsp_build_surface(const ivp_allocator_t *alloc,
                                                const ivp_q12bsp_t *bsp,
                                                int model_index,
                                                const ivp_q12bsp_config_t *cfg)
{
    if (!bsp || model_index < 0 || model_index >= bsp->n_models) return NULL;
    if (!alloc) alloc = ivp_default_allocator();

    ivp_q12bsp_config_t defaults;
    if (!cfg) {
        ivp_q12bsp_config_init(&defaults);
        cfg = &defaults;
    }

    const ivp_q12_model_t *model = &bsp->models[model_index];

    /* Initialize traversal context */
    bsp_traverse_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.bsp   = bsp;
    ctx.cfg   = cfg;
    ctx.alloc = alloc;

    /* Start traversal from model's head node */
    int head = model->headnode[0];
    if (head < 0 || head >= bsp->n_nodes) return NULL;

    traverse_node(&ctx, head);

    if (ctx.n_parts == 0) {
        if (ctx.parts) ivp_free(alloc, ctx.parts,
                                (size_t)ctx.parts_cap * sizeof(ivp_compact_surface_t *));
        return NULL;
    }

    /* If single part, return it directly */
    ivp_compact_surface_t *result = NULL;
    if (ctx.n_parts == 1) {
        result = ctx.parts[0];
    } else {
        /* Assemble compound surface from all convex parts */
        result = ivp_build_concave_from_parts(alloc, ctx.parts, ctx.n_parts);
        /* Free individual parts (concave_from_parts copies the data) */
        for (int i = 0; i < ctx.n_parts; i++) {
            ivp_compact_surface_free(alloc, ctx.parts[i]);
        }
    }

    if (ctx.parts)
        ivp_free(alloc, ctx.parts,
                 (size_t)ctx.parts_cap * sizeof(ivp_compact_surface_t *));

    return result;
}
