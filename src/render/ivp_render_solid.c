/* ivp_render_solid.c -- Solid geometry: prototype mesh generation & draw
 *
 * Generates icosphere, unit box, and capsule prototype meshes.
 * Provides public API for queuing instanced solid draws that are
 * flushed during the render pass.
 */

#include "ivp_render_internal.h"
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_assert.h>
#include <math.h>
#include <string.h>

/* ── Prototype vertex: position + normal ───────────────────────────────── */

typedef struct proto_vertex {
    float pos[3];
    float nrm[3];
} proto_vertex_t;

/* ═══════════════════════════════════════════════════════════════════════════
 * Icosphere generation (subdivision level 2 → ~320 triangles)
 * ═══════════════════════════════════════════════════════════════════════════ */

#define ICO_MAX_VERTS 384                     /* uncached subdiv=2 uses up to 312 */
#define ICO_MAX_TRIS  (20 * 4 * 4)           /* subdiv 2 = 320 tris */

typedef struct ico_builder {
    proto_vertex_t verts[ICO_MAX_VERTS];
    Uint16         indices[ICO_MAX_TRIS * 3];
    bool           overflow;
    int            nv, ni;
} ico_builder_t;

static int ico_add_vert(ico_builder_t *b, float x, float y, float z)
{
    if (b->nv >= ICO_MAX_VERTS) {
        b->overflow = true;
        IVP_ASSERT_MSG(false, "icosphere vertex capacity exceeded");
        return 0;
    }
    float len = sqrtf(x*x + y*y + z*z);
    if (len > 1e-8f) { x /= len; y /= len; z /= len; }
    b->verts[b->nv].pos[0] = x;
    b->verts[b->nv].pos[1] = y;
    b->verts[b->nv].pos[2] = z;
    b->verts[b->nv].nrm[0] = x;
    b->verts[b->nv].nrm[1] = y;
    b->verts[b->nv].nrm[2] = z;
    return b->nv++;
}

static int ico_midpoint(ico_builder_t *b, int i0, int i1)
{
    if (i0 < 0 || i0 >= b->nv || i1 < 0 || i1 >= b->nv) {
        b->overflow = true;
        IVP_ASSERT_MSG(false, "icosphere midpoint index out of range");
        return 0;
    }
    float mx = (b->verts[i0].pos[0] + b->verts[i1].pos[0]) * 0.5f;
    float my = (b->verts[i0].pos[1] + b->verts[i1].pos[1]) * 0.5f;
    float mz = (b->verts[i0].pos[2] + b->verts[i1].pos[2]) * 0.5f;
    return ico_add_vert(b, mx, my, mz);
}

static void ico_build(ico_builder_t *b, int subdivisions)
{
    memset(b, 0, sizeof(*b));
    IVP_ASSERT_MSG(subdivisions >= 0, "subdivisions must be non-negative");
    float t = (1.0f + sqrtf(5.0f)) * 0.5f;

    ico_add_vert(b, -1,  t,  0);
    ico_add_vert(b,  1,  t,  0);
    ico_add_vert(b, -1, -t,  0);
    ico_add_vert(b,  1, -t,  0);
    ico_add_vert(b,  0, -1,  t);
    ico_add_vert(b,  0,  1,  t);
    ico_add_vert(b,  0, -1, -t);
    ico_add_vert(b,  0,  1, -t);
    ico_add_vert(b,  t,  0, -1);
    ico_add_vert(b,  t,  0,  1);
    ico_add_vert(b, -t,  0, -1);
    ico_add_vert(b, -t,  0,  1);

    static const Uint16 base_tris[20][3] = {
        {0,11,5},{0,5,1},{0,1,7},{0,7,10},{0,10,11},
        {1,5,9},{5,11,4},{11,10,2},{10,7,6},{7,1,8},
        {3,9,4},{3,4,2},{3,2,6},{3,6,8},{3,8,9},
        {4,9,5},{2,4,11},{6,2,10},{8,6,7},{9,8,1}
    };
    b->ni = 20 * 3;
    for (int i = 0; i < 20; i++) {
        b->indices[i*3+0] = base_tris[i][0];
        b->indices[i*3+1] = base_tris[i][1];
        b->indices[i*3+2] = base_tris[i][2];
    }

    /* Subdivide */
    for (int s = 0; s < subdivisions; s++) {
        int old_ni = b->ni;
        Uint16 new_idx[ICO_MAX_TRIS * 3];
        int new_ni = 0;
        for (int i = 0; i < old_ni; i += 3) {
            int v0 = b->indices[i+0];
            int v1 = b->indices[i+1];
            int v2 = b->indices[i+2];
            int a = ico_midpoint(b, v0, v1);
            int bb = ico_midpoint(b, v1, v2);
            int c = ico_midpoint(b, v2, v0);
            if (b->overflow) {
                b->ni = 0;
                return;
            }
            new_idx[new_ni++] = (Uint16)v0; new_idx[new_ni++] = (Uint16)a;  new_idx[new_ni++] = (Uint16)c;
            new_idx[new_ni++] = (Uint16)v1; new_idx[new_ni++] = (Uint16)bb; new_idx[new_ni++] = (Uint16)a;
            new_idx[new_ni++] = (Uint16)v2; new_idx[new_ni++] = (Uint16)c;  new_idx[new_ni++] = (Uint16)bb;
            new_idx[new_ni++] = (Uint16)a;  new_idx[new_ni++] = (Uint16)bb; new_idx[new_ni++] = (Uint16)c;
        }
        memcpy(b->indices, new_idx, (size_t)new_ni * sizeof(Uint16));
        b->ni = new_ni;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Unit box generation (12 triangles, flat normals)
 * ═══════════════════════════════════════════════════════════════════════════ */

#define BOX_VERTS 24
#define BOX_TRIS  12

static void box_build(proto_vertex_t verts[BOX_VERTS], Uint16 indices[BOX_TRIS * 3])
{
    /* 6 faces × 4 unique vertices (flat normal per face) */
    static const float faces[6][4][3] = {
        /* +X */ {{ 1,-1,-1},{ 1, 1,-1},{ 1, 1, 1},{ 1,-1, 1}},
        /* -X */ {{-1,-1, 1},{-1, 1, 1},{-1, 1,-1},{-1,-1,-1}},
        /* +Y */ {{-1, 1,-1},{-1, 1, 1},{ 1, 1, 1},{ 1, 1,-1}},
        /* -Y */ {{-1,-1, 1},{-1,-1,-1},{ 1,-1,-1},{ 1,-1, 1}},
        /* +Z */ {{-1,-1, 1},{ 1,-1, 1},{ 1, 1, 1},{-1, 1, 1}},
        /* -Z */ {{ 1,-1,-1},{-1,-1,-1},{-1, 1,-1},{ 1, 1,-1}},
    };
    static const float normals[6][3] = {
        { 1, 0, 0},{-1, 0, 0},{ 0, 1, 0},{ 0,-1, 0},{ 0, 0, 1},{ 0, 0,-1}
    };

    for (int f = 0; f < 6; f++) {
        for (int v = 0; v < 4; v++) {
            int idx = f * 4 + v;
            memcpy(verts[idx].pos, faces[f][v], sizeof(float) * 3);
            memcpy(verts[idx].nrm, normals[f], sizeof(float) * 3);
        }
        int base = f * 4;
        int ti = f * 6;
        indices[ti+0] = (Uint16)(base+0); indices[ti+1] = (Uint16)(base+1); indices[ti+2] = (Uint16)(base+2);
        indices[ti+3] = (Uint16)(base+0); indices[ti+4] = (Uint16)(base+2); indices[ti+5] = (Uint16)(base+3);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Proto shape initialization / teardown
 * ═══════════════════════════════════════════════════════════════════════════ */

ivp_result_t ivp_proto_shapes_init(ivp_proto_shapes_t *ps, ivp_gpu_device_t *dev)
{
    memset(ps, 0, sizeof(*ps));
    ivp_result_t res;

    /* Icosphere (subdiv=2) */
    {
        ico_builder_t ico;
        ico_build(&ico, 2);
        if (ico.overflow || ico.nv <= 0 || ico.ni <= 0) {
            return IVP_ERR_CAPACITY_EXCEEDED;
        }
        res = ivp_gpu_proto_mesh_upload(&ps->sphere, dev,
            ico.verts, ico.nv, (int)sizeof(proto_vertex_t),
            ico.indices, ico.ni);
        if (res != IVP_OK) return res;
    }

    /* Unit box */
    {
        proto_vertex_t bv[BOX_VERTS];
        Uint16 bi[BOX_TRIS * 3];
        box_build(bv, bi);
        res = ivp_gpu_proto_mesh_upload(&ps->box, dev,
            bv, BOX_VERTS, (int)sizeof(proto_vertex_t),
            bi, BOX_TRIS * 3);
        if (res != IVP_OK) {
            ivp_gpu_proto_mesh_destroy(&ps->sphere, dev);
            return res;
        }
    }

    /* Instance buffers (capacity: 4096 instances each) */
    res = ivp_gpu_instance_buf_create(&ps->sphere_instances, dev,
        4096, (int)sizeof(ivp_solid_instance_t));
    if (res != IVP_OK) goto fail;

    res = ivp_gpu_instance_buf_create(&ps->box_instances, dev,
        4096, (int)sizeof(ivp_solid_instance_t));
    if (res != IVP_OK) goto fail;

    ps->ready = true;
    return IVP_OK;

fail:
    ivp_proto_shapes_destroy(ps, dev);
    return res;
}

void ivp_proto_shapes_destroy(ivp_proto_shapes_t *ps, ivp_gpu_device_t *dev)
{
    ivp_gpu_instance_buf_destroy(&ps->box_instances, dev);
    ivp_gpu_instance_buf_destroy(&ps->sphere_instances, dev);
    ivp_gpu_proto_mesh_destroy(&ps->box, dev);
    ivp_gpu_proto_mesh_destroy(&ps->sphere, dev);
    ps->ready = false;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Public solid draw API -- queue instances for this frame
 * ═══════════════════════════════════════════════════════════════════════════ */

static void make_scale_translate_mat4(float out[16],
                                      const float pos[3],
                                      float sx, float sy, float sz)
{
    memset(out, 0, 16 * sizeof(float));
    out[0]  = sx;
    out[5]  = sy;
    out[10] = sz;
    out[12] = pos[0];
    out[13] = pos[1];
    out[14] = pos[2];
    out[15] = 1.0f;
}

void ivp_render_draw_solid_sphere(ivp_renderer_t *r,
                                  const float pos[3],
                                  float radius,
                                  const float color[3])
{
    if (!r->protos.ready) return;

    /* Frustum cull */
    if (!ivp_frustum_test_sphere(&r->frustum, pos, radius)) {
        r->culled_count++;
        return;
    }

    ivp_solid_instance_t *inst = (ivp_solid_instance_t *)
        ivp_gpu_instance_buf_push(&r->protos.sphere_instances, 1);
    if (!inst) return;

    make_scale_translate_mat4(inst->transform, pos, radius, radius, radius);
    memcpy(inst->color, color, 3 * sizeof(float));
    inst->_pad = 0.0f;
}

void ivp_render_draw_solid_box(ivp_renderer_t *r,
                               const float pos[3],
                               const float half_size[3],
                               const float *rot,
                               const float color[3])
{
    if (!r->protos.ready) return;

    /* Frustum cull (compute bounding sphere) */
    float bsph_r = sqrtf(half_size[0]*half_size[0] +
                         half_size[1]*half_size[1] +
                         half_size[2]*half_size[2]);
    if (!ivp_frustum_test_sphere(&r->frustum, pos, bsph_r)) {
        r->culled_count++;
        return;
    }

    ivp_solid_instance_t *inst = (ivp_solid_instance_t *)
        ivp_gpu_instance_buf_push(&r->protos.box_instances, 1);
    if (!inst) return;

    if (rot) {
        /* Build column-major 4×4 from 3×3 rotation + half_size scale + translation */
        float *m = inst->transform;
        memset(m, 0, 16 * sizeof(float));
        m[0]  = rot[0] * half_size[0];
        m[1]  = rot[1] * half_size[0];
        m[2]  = rot[2] * half_size[0];
        m[4]  = rot[3] * half_size[1];
        m[5]  = rot[4] * half_size[1];
        m[6]  = rot[5] * half_size[1];
        m[8]  = rot[6] * half_size[2];
        m[9]  = rot[7] * half_size[2];
        m[10] = rot[8] * half_size[2];
        m[12] = pos[0];
        m[13] = pos[1];
        m[14] = pos[2];
        m[15] = 1.0f;
    } else {
        make_scale_translate_mat4(inst->transform, pos,
                                  half_size[0], half_size[1], half_size[2]);
    }
    memcpy(inst->color, color, 3 * sizeof(float));
    inst->_pad = 0.0f;
}

void ivp_render_draw_solid_aabb(ivp_renderer_t *r,
                                const float mn[3], const float mx[3],
                                const float color[3])
{
    float pos[3], hs[3];
    for (int k = 0; k < 3; k++) {
        pos[k] = (mn[k] + mx[k]) * 0.5f;
        hs[k]  = (mx[k] - mn[k]) * 0.5f;
    }
    ivp_render_draw_solid_box(r, pos, hs, NULL, color);
}

void ivp_render_set_light_dir(ivp_renderer_t *r, const float dir[3])
{
    memcpy(r->lighting.light_dir, dir, 3 * sizeof(float));
    ivp_render_vec3_normalize(r->lighting.light_dir);
}

void ivp_render_set_ambient(ivp_renderer_t *r, float ambient)
{
    if (ambient < 0.0f) ambient = 0.0f;
    if (ambient > 1.0f) ambient = 1.0f;
    r->lighting.ambient = ambient;
}
