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
#include <stdlib.h>
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
 * Cylinder and cone generation (radius 1, y in [-1,1], axis = +Y)
 * ═══════════════════════════════════════════════════════════════════════════ */

#define SHAPE_SEGMENTS   24
#define CYL_VERTS        (4 * SHAPE_SEGMENTS + 2)
#define CYL_TRIS         (4 * SHAPE_SEGMENTS)
#define CONE_VERTS       (3 * SHAPE_SEGMENTS + 1)
#define CONE_TRIS        (2 * SHAPE_SEGMENTS)

typedef struct shape_builder {
    proto_vertex_t *verts;
    Uint16         *indices;
    int             nv, ni;
} shape_builder_t;

static int shape_vert(shape_builder_t *b, float x, float y, float z,
                      float nx, float ny, float nz)
{
    proto_vertex_t *v = &b->verts[b->nv];
    float len = sqrtf(nx*nx + ny*ny + nz*nz);
    if (len > 1e-8f) { nx /= len; ny /= len; nz /= len; }
    v->pos[0] = x;  v->pos[1] = y;  v->pos[2] = z;
    v->nrm[0] = nx; v->nrm[1] = ny; v->nrm[2] = nz;
    return b->nv++;
}

/* Emit a triangle, flipping it when its winding disagrees with the vertex
 * normals so every generated face is counter-clockwise from outside. */
static void shape_tri(shape_builder_t *b, int i0, int i1, int i2)
{
    const proto_vertex_t *a = &b->verts[i0], *c = &b->verts[i1], *d = &b->verts[i2];
    float e1[3] = { c->pos[0]-a->pos[0], c->pos[1]-a->pos[1], c->pos[2]-a->pos[2] };
    float e2[3] = { d->pos[0]-a->pos[0], d->pos[1]-a->pos[1], d->pos[2]-a->pos[2] };
    float n[3]  = { e1[1]*e2[2]-e1[2]*e2[1],
                    e1[2]*e2[0]-e1[0]*e2[2],
                    e1[0]*e2[1]-e1[1]*e2[0] };
    float dot = n[0]*(a->nrm[0]+c->nrm[0]+d->nrm[0])
              + n[1]*(a->nrm[1]+c->nrm[1]+d->nrm[1])
              + n[2]*(a->nrm[2]+c->nrm[2]+d->nrm[2]);
    if (dot < 0.0f) { int t = i1; i1 = i2; i2 = t; }
    b->indices[b->ni++] = (Uint16)i0;
    b->indices[b->ni++] = (Uint16)i1;
    b->indices[b->ni++] = (Uint16)i2;
}

static void shape_angle(int i, float *c, float *s)
{
    float a = 6.28318530717958647692f * (float)i / (float)SHAPE_SEGMENTS;
    *c = cosf(a);
    *s = sinf(a);
}

static void cylinder_build(proto_vertex_t verts[CYL_VERTS], Uint16 indices[CYL_TRIS * 3],
                           int *nv, int *ni)
{
    shape_builder_t b = { verts, indices, 0, 0 };
    const int N = SHAPE_SEGMENTS;
    int side0 = b.nv;
    for (int i = 0; i < N; i++) {              /* side: top/bottom pair per column */
        float c, s;  shape_angle(i, &c, &s);
        shape_vert(&b, c,  1.0f, s, c, 0.0f, s);
        shape_vert(&b, c, -1.0f, s, c, 0.0f, s);
    }
    for (int i = 0; i < N; i++) {
        int j = (i + 1) % N;
        int t0 = side0 + 2*i, b0 = t0 + 1, t1 = side0 + 2*j, b1 = t1 + 1;
        shape_tri(&b, t0, b0, b1);
        shape_tri(&b, t0, b1, t1);
    }
    for (int cap = 0; cap < 2; cap++) {        /* caps: +Y then -Y */
        float y = cap ? -1.0f : 1.0f;
        int centre = shape_vert(&b, 0.0f, y, 0.0f, 0.0f, y, 0.0f);
        int ring0 = b.nv;
        for (int i = 0; i < N; i++) {
            float c, s;  shape_angle(i, &c, &s);
            shape_vert(&b, c, y, s, 0.0f, y, 0.0f);
        }
        for (int i = 0; i < N; i++)
            shape_tri(&b, centre, ring0 + i, ring0 + (i + 1) % N);
    }
    *nv = b.nv;  *ni = b.ni;
}

static void cone_build(proto_vertex_t verts[CONE_VERTS], Uint16 indices[CONE_TRIS * 3],
                       int *nv, int *ni)
{
    shape_builder_t b = { verts, indices, 0, 0 };
    const int N = SHAPE_SEGMENTS;
    /* Side normal of a cone with base radius 1 and height 2: (2c, 1, 2s). */
    int ring0 = b.nv;
    for (int i = 0; i < N; i++) {
        float c, s;  shape_angle(i, &c, &s);
        shape_vert(&b, c, -1.0f, s, 2.0f*c, 1.0f, 2.0f*s);
    }
    int tip0 = b.nv;
    for (int i = 0; i < N; i++) {              /* one apex copy per segment */
        float c, s;  shape_angle(i, &c, &s);
        float c1, s1;  shape_angle(i + 1, &c1, &s1);
        float mx = c + c1, mz = s + s1;        /* mid-angle direction */
        shape_vert(&b, 0.0f, 1.0f, 0.0f, mx, 1.0f, mz);
    }
    for (int i = 0; i < N; i++)
        shape_tri(&b, ring0 + i, ring0 + (i + 1) % N, tip0 + i);
    int centre = shape_vert(&b, 0.0f, -1.0f, 0.0f, 0.0f, -1.0f, 0.0f);
    int cap0 = b.nv;
    for (int i = 0; i < N; i++) {
        float c, s;  shape_angle(i, &c, &s);
        shape_vert(&b, c, -1.0f, s, 0.0f, -1.0f, 0.0f);
    }
    for (int i = 0; i < N; i++)
        shape_tri(&b, centre, cap0 + i, cap0 + (i + 1) % N);
    *nv = b.nv;  *ni = b.ni;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Proto shape initialization / teardown
 * ═══════════════════════════════════════════════════════════════════════════ */

#define BUILTIN_COUNT 4    /* sphere, box, cylinder, cone */

static void builtin(ivp_proto_shapes_t *ps, int i,
                    ivp_gpu_mesh_t **mesh, ivp_gpu_instance_buf_t **inst)
{
    switch (i) {
    case 0:  *mesh = ps->sphere;   *inst = &ps->sphere_instances;   break;
    case 1:  *mesh = ps->box;      *inst = &ps->box_instances;      break;
    case 2:  *mesh = ps->cylinder; *inst = &ps->cylinder_instances; break;
    default: *mesh = ps->cone;     *inst = &ps->cone_instances;     break;
    }
}

/* Instance buffer a draw of built-in shape i goes to (opaque or blend). */
static ivp_gpu_instance_buf_t *builtin_buf(ivp_renderer_t *r, int i)
{
    ivp_gpu_mesh_t *m;
    ivp_gpu_instance_buf_t *b;
    builtin(&r->protos, i, &m, &b);
    return r->solid_alpha < 0.999f ? &r->protos.blend[i] : b;
}

static ivp_result_t shapes_upload_generated(ivp_gpu_mesh_t **out,
                                            ivp_gpu_device_t *dev, bool cone)
{
    proto_vertex_t verts[CYL_VERTS > CONE_VERTS ? CYL_VERTS : CONE_VERTS];
    Uint16 idx[(CYL_TRIS > CONE_TRIS ? CYL_TRIS : CONE_TRIS) * 3];
    int nv, ni;
    if (cone) cone_build(verts, idx, &nv, &ni);
    else      cylinder_build(verts, idx, &nv, &ni);
    *out = ivp_gpu_mesh_create(dev, verts, nv, (int)sizeof(proto_vertex_t), idx, ni);
    return *out ? IVP_OK : IVP_ERR_OUT_OF_MEMORY;
}

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
        ps->sphere = ivp_gpu_mesh_create(dev,
            ico.verts, ico.nv, (int)sizeof(proto_vertex_t),
            ico.indices, ico.ni);
        res = IVP_ERR_OUT_OF_MEMORY;
        if (!ps->sphere) goto fail;
    }

    /* Unit box */
    {
        proto_vertex_t bv[BOX_VERTS];
        Uint16 bi[BOX_TRIS * 3];
        box_build(bv, bi);
        ps->box = ivp_gpu_mesh_create(dev,
            bv, BOX_VERTS, (int)sizeof(proto_vertex_t),
            bi, BOX_TRIS * 3);
        res = IVP_ERR_OUT_OF_MEMORY;
        if (!ps->box) goto fail;
    }

    res = shapes_upload_generated(&ps->cylinder, dev, false);
    if (res != IVP_OK) goto fail;
    res = shapes_upload_generated(&ps->cone, dev, true);
    if (res != IVP_OK) goto fail;

    /* Instance buffers (capacity: 4096 instances each) */
    res = ivp_gpu_instance_buf_create(&ps->sphere_instances, dev,
        4096, (int)sizeof(ivp_solid_instance_t));
    if (res != IVP_OK) goto fail;

    res = ivp_gpu_instance_buf_create(&ps->box_instances, dev,
        4096, (int)sizeof(ivp_solid_instance_t));
    if (res != IVP_OK) goto fail;

    res = ivp_gpu_instance_buf_create(&ps->cylinder_instances, dev,
        4096, (int)sizeof(ivp_solid_instance_t));
    if (res != IVP_OK) goto fail;

    res = ivp_gpu_instance_buf_create(&ps->cone_instances, dev,
        4096, (int)sizeof(ivp_solid_instance_t));
    if (res != IVP_OK) goto fail;

    for (int i = 0; i < BUILTIN_COUNT; i++) {
        res = ivp_gpu_instance_buf_create(&ps->blend[i], dev,
            512, (int)sizeof(ivp_solid_instance_t));
        if (res != IVP_OK) goto fail;
    }

    ps->ready = true;
    return IVP_OK;

fail:
    ivp_proto_shapes_destroy(ps, dev);
    return res;
}

void ivp_proto_shapes_destroy(ivp_proto_shapes_t *ps, ivp_gpu_device_t *dev)
{
    for (int i = 0; i < BUILTIN_COUNT; i++)
        ivp_gpu_instance_buf_destroy(&ps->blend[i], dev);
    ivp_gpu_instance_buf_destroy(&ps->cone_instances, dev);
    ivp_gpu_instance_buf_destroy(&ps->cylinder_instances, dev);
    ivp_gpu_instance_buf_destroy(&ps->box_instances, dev);
    ivp_gpu_instance_buf_destroy(&ps->sphere_instances, dev);
    ivp_gpu_mesh_destroy(dev, ps->cone);
    ivp_gpu_mesh_destroy(dev, ps->cylinder);
    ivp_gpu_mesh_destroy(dev, ps->box);
    ivp_gpu_mesh_destroy(dev, ps->sphere);
    ps->cone = ps->cylinder = ps->box = ps->sphere = NULL;
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
        ivp_gpu_instance_buf_push(builtin_buf(r, 0), 1);
    if (!inst) return;

    make_scale_translate_mat4(inst->transform, pos, radius, radius, radius);
    memcpy(inst->color, color, 3 * sizeof(float));
    inst->alpha = r->solid_alpha;
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
        ivp_gpu_instance_buf_push(builtin_buf(r, 1), 1);
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
    inst->alpha = r->solid_alpha;
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

/* ═══════════════════════════════════════════════════════════════════════════
 * Cylinder / cone / capsule
 * ═══════════════════════════════════════════════════════════════════════════ */

/* Model matrix for a unit shape (radius 1, y in [-1,1]) mapped onto the
 * segment p0 -> p1: +Y along the segment, centred on its midpoint, scaled by
 * `radius` across and half the length along.  Returns the bounding radius,
 * or a negative value when the segment is degenerate. */
static float segment_model(float m[16], const float p0[3], const float p1[3],
                           float radius)
{
    float y[3] = { p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2] };
    float len = sqrtf(y[0]*y[0] + y[1]*y[1] + y[2]*y[2]);
    if (!(len > 1e-9f)) return -1.0f;
    y[0] /= len;  y[1] /= len;  y[2] /= len;

    /* x: any unit vector perpendicular to y */
    float x[3];
    if (fabsf(y[0]) < 0.9f) { x[0] = 0.0f; x[1] = -y[2]; x[2] = y[1]; }
    else                    { x[0] = y[2]; x[1] = 0.0f;  x[2] = -y[0]; }
    float xl = sqrtf(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]);
    x[0] /= xl;  x[1] /= xl;  x[2] /= xl;
    float z[3] = { y[1]*x[2]-y[2]*x[1], y[2]*x[0]-y[0]*x[2], y[0]*x[1]-y[1]*x[0] };

    float half = len * 0.5f;
    memset(m, 0, 16 * sizeof(float));
    for (int k = 0; k < 3; k++) {
        m[0 + k]  = x[k] * radius;
        m[4 + k]  = y[k] * half;
        m[8 + k]  = z[k] * radius;
        m[12 + k] = (p0[k] + p1[k]) * 0.5f;
    }
    m[15] = 1.0f;
    return sqrtf(half*half + radius*radius);
}

static void queue_shape(ivp_renderer_t *r, ivp_gpu_instance_buf_t *buf,
                        const float m[16], const float centre[3], float bound_r,
                        const float color[3])
{
    if (!ivp_frustum_test_sphere(&r->frustum, centre, bound_r)) {
        r->culled_count++;
        return;
    }
    ivp_solid_instance_t *inst = (ivp_solid_instance_t *)
        ivp_gpu_instance_buf_push(buf, 1);
    if (!inst) return;
    memcpy(inst->transform, m, 16 * sizeof(float));
    memcpy(inst->color, color, 3 * sizeof(float));
    inst->alpha = r->solid_alpha;
}

void ivp_render_draw_solid_cylinder(ivp_renderer_t *r,
                                    const float a[3], const float b[3],
                                    float radius, const float color[3])
{
    if (!r->protos.ready) return;
    float m[16];
    float br = segment_model(m, a, b, radius);
    if (br < 0.0f) return;
    queue_shape(r, builtin_buf(r, 2), m, &m[12], br, color);
}

void ivp_render_draw_solid_cone(ivp_renderer_t *r,
                                const float apex[3],
                                const float base_center[3],
                                float base_radius, const float color[3])
{
    if (!r->protos.ready) return;
    float m[16];
    /* The prototype has its apex at +Y, so run the segment base -> apex. */
    float br = segment_model(m, base_center, apex, base_radius);
    if (br < 0.0f) return;
    queue_shape(r, builtin_buf(r, 3), m, &m[12], br, color);
}

void ivp_render_draw_solid_capsule(ivp_renderer_t *r,
                                   const float a[3], const float b[3],
                                   float radius, const float color[3])
{
    ivp_render_draw_solid_cylinder(r, a, b, radius, color);
    ivp_render_draw_solid_sphere(r, a, radius, color);
    ivp_render_draw_solid_sphere(r, b, radius, color);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Static meshes
 * ═══════════════════════════════════════════════════════════════════════════ */

#define MESH_MAX_VERTS      65535
#define MESH_INSTANCE_CAP   1024

ivp_render_mesh_t *ivp_render_mesh_create(ivp_renderer_t *r,
                                          const float *positions,
                                          int vertex_count,
                                          const int *indices,
                                          int triangle_count)
{
    if (!r || !r->protos.ready || !positions || !indices ||
        vertex_count <= 0 || triangle_count <= 0 ||
        triangle_count > MESH_MAX_VERTS / 3)
        return NULL;
    for (int i = 0; i < triangle_count * 3; i++)
        if (indices[i] < 0 || indices[i] >= vertex_count) return NULL;

    int nv = triangle_count * 3;
    proto_vertex_t *verts = (proto_vertex_t *)malloc((size_t)nv * sizeof(*verts));
    Uint16 *idx = (Uint16 *)malloc((size_t)nv * sizeof(*idx));
    ivp_render_mesh_t *m = (ivp_render_mesh_t *)calloc(1, sizeof(*m));
    if (!verts || !idx || !m) goto fail;

    /* Unroll to flat-shaded vertices; collect the bounds on the way. */
    float mn[3] = {  1e30f,  1e30f,  1e30f };
    float mx[3] = { -1e30f, -1e30f, -1e30f };
    for (int t = 0; t < triangle_count; t++) {
        const float *p[3];
        for (int k = 0; k < 3; k++) {
            p[k] = positions + 3 * indices[3*t + k];
            for (int c = 0; c < 3; c++) {
                if (p[k][c] < mn[c]) mn[c] = p[k][c];
                if (p[k][c] > mx[c]) mx[c] = p[k][c];
            }
        }
        float e1[3] = { p[1][0]-p[0][0], p[1][1]-p[0][1], p[1][2]-p[0][2] };
        float e2[3] = { p[2][0]-p[0][0], p[2][1]-p[0][1], p[2][2]-p[0][2] };
        float n[3]  = { e1[1]*e2[2]-e1[2]*e2[1],
                        e1[2]*e2[0]-e1[0]*e2[2],
                        e1[0]*e2[1]-e1[1]*e2[0] };
        float nl = sqrtf(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
        if (nl > 1e-20f) { n[0] /= nl; n[1] /= nl; n[2] /= nl; }
        for (int k = 0; k < 3; k++) {
            proto_vertex_t *v = &verts[3*t + k];
            memcpy(v->pos, p[k], 3 * sizeof(float));
            memcpy(v->nrm, n, 3 * sizeof(float));
            idx[3*t + k] = (Uint16)(3*t + k);
        }
    }
    float rad2 = 0.0f;
    for (int c = 0; c < 3; c++) {
        m->center[c] = (mn[c] + mx[c]) * 0.5f;
        float h = (mx[c] - mn[c]) * 0.5f;
        rad2 += h * h;
    }
    m->radius = sqrtf(rad2);

    m->mesh = ivp_gpu_mesh_create(r->gpu, verts, nv,
                                  (int)sizeof(proto_vertex_t), idx, nv);
    if (!m->mesh) goto fail;
    if (ivp_gpu_instance_buf_create(&m->instances, r->gpu, MESH_INSTANCE_CAP,
                                    (int)sizeof(ivp_solid_instance_t)) != IVP_OK) {
        ivp_gpu_mesh_destroy(r->gpu, m->mesh);
        goto fail;
    }
    if (ivp_gpu_instance_buf_create(&m->blend_instances, r->gpu, MESH_INSTANCE_CAP / 4,
                                    (int)sizeof(ivp_solid_instance_t)) != IVP_OK) {
        ivp_gpu_instance_buf_destroy(&m->instances, r->gpu);
        ivp_gpu_mesh_destroy(r->gpu, m->mesh);
        goto fail;
    }
    free(verts);
    free(idx);
    m->renderer = r;
    m->next = r->meshes;
    r->meshes = m;
    return m;

fail:
    free(verts);
    free(idx);
    free(m);
    return NULL;
}

static void mesh_release(ivp_render_mesh_t *m)
{
    ivp_renderer_t *r = m->renderer;
    /* The GPU may still read the buffers of frames in flight. */
    ivp_gpu_wait_idle(r->gpu);
    ivp_gpu_instance_buf_destroy(&m->blend_instances, r->gpu);
    ivp_gpu_instance_buf_destroy(&m->instances, r->gpu);
    ivp_gpu_mesh_destroy(r->gpu, m->mesh);
    free(m);
}

void ivp_render_mesh_destroy(ivp_render_mesh_t *mesh)
{
    if (!mesh) return;
    ivp_renderer_t *r = mesh->renderer;
    for (ivp_render_mesh_t **pp = &r->meshes; *pp; pp = &(*pp)->next) {
        if (*pp == mesh) { *pp = mesh->next; break; }
    }
    mesh_release(mesh);
}

void ivp_render_draw_solid_mesh(ivp_renderer_t *r, ivp_render_mesh_t *mesh,
                                const float model[16], const float color[3])
{
    if (!r->protos.ready || !mesh) return;

    /* Bounding sphere through the model matrix (largest axis scale). */
    float wc[3], s2 = 0.0f;
    for (int k = 0; k < 3; k++)
        wc[k] = model[k]*mesh->center[0] + model[4+k]*mesh->center[1] +
                model[8+k]*mesh->center[2] + model[12+k];
    for (int c = 0; c < 3; c++) {
        float l2 = model[4*c]*model[4*c] + model[4*c+1]*model[4*c+1] +
                   model[4*c+2]*model[4*c+2];
        if (l2 > s2) s2 = l2;
    }
    queue_shape(r, r->solid_alpha < 0.999f ? &mesh->blend_instances : &mesh->instances,
                model, wc, mesh->radius * sqrtf(s2), color);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Per-frame hooks
 * ═══════════════════════════════════════════════════════════════════════════ */

bool ivp_solid_has_instances(const ivp_renderer_t *r)
{
    if (!r->protos.ready) return false;
    const ivp_gpu_instance_buf_t *b[BUILTIN_COUNT] = {
        &r->protos.sphere_instances, &r->protos.box_instances,
        &r->protos.cylinder_instances, &r->protos.cone_instances };
    for (int i = 0; i < BUILTIN_COUNT; i++)
        if (b[i]->count > 0) return true;
    for (const ivp_render_mesh_t *u = r->meshes; u; u = u->next)
        if (u->instances.count > 0) return true;
    return false;
}

void ivp_solid_begin_frame(ivp_renderer_t *r)
{
    if (!r->protos.ready) return;
    ivp_gpu_mesh_t *m;  ivp_gpu_instance_buf_t *b;
    for (int i = 0; i < BUILTIN_COUNT; i++) {
        builtin(&r->protos, i, &m, &b);
        ivp_gpu_instance_buf_reset(b);
        ivp_gpu_instance_buf_reset(&r->protos.blend[i]);
    }
    for (ivp_render_mesh_t *u = r->meshes; u; u = u->next) {
        ivp_gpu_instance_buf_reset(&u->instances);
        ivp_gpu_instance_buf_reset(&u->blend_instances);
    }
    r->solid_alpha = 1.0f;
}

void ivp_solid_upload(ivp_renderer_t *r)
{
    if (!r->protos.ready) return;
    ivp_gpu_mesh_t *m;  ivp_gpu_instance_buf_t *b;
    for (int i = 0; i < BUILTIN_COUNT; i++) {
        builtin(&r->protos, i, &m, &b);
        ivp_gpu_instance_buf_upload(b, r->gpu);
        ivp_gpu_instance_buf_upload(&r->protos.blend[i], r->gpu);
    }
    for (ivp_render_mesh_t *u = r->meshes; u; u = u->next) {
        ivp_gpu_instance_buf_upload(&u->instances, r->gpu);
        ivp_gpu_instance_buf_upload(&u->blend_instances, r->gpu);
    }
}

void ivp_solid_draw(ivp_renderer_t *r, ivp_gpu_pass_t *pass)
{
    if (!r->protos.ready) return;
    ivp_gpu_mesh_t *m;  ivp_gpu_instance_buf_t *b;
    for (int i = 0; i < BUILTIN_COUNT; i++) {
        builtin(&r->protos, i, &m, &b);
        ivp_gpu_instance_buf_draw(b, m, pass);
    }
    for (ivp_render_mesh_t *u = r->meshes; u; u = u->next)
        ivp_gpu_instance_buf_draw(&u->instances, u->mesh, pass);
}

/* Translucent instances; the caller binds the blend pipeline. */
void ivp_solid_draw_blend(ivp_renderer_t *r, ivp_gpu_pass_t *pass)
{
    if (!r->protos.ready) return;
    ivp_gpu_mesh_t *m;  ivp_gpu_instance_buf_t *b;
    for (int i = 0; i < BUILTIN_COUNT; i++) {
        builtin(&r->protos, i, &m, &b);
        ivp_gpu_instance_buf_draw(&r->protos.blend[i], m, pass);
    }
    for (ivp_render_mesh_t *u = r->meshes; u; u = u->next)
        ivp_gpu_instance_buf_draw(&u->blend_instances, u->mesh, pass);
}

void ivp_render_set_solid_alpha(ivp_renderer_t *r, float alpha)
{
    if (!(alpha >= 0.0f)) alpha = 0.0f;       /* also NaN */
    if (alpha > 1.0f) alpha = 1.0f;
    r->solid_alpha = alpha;
}

void ivp_solid_destroy_meshes(ivp_renderer_t *r)
{
    while (r->meshes) {
        ivp_render_mesh_t *m = r->meshes;
        r->meshes = m->next;
        mesh_release(m);
    }
}
