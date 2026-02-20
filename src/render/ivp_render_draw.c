/* ivp_render_draw.c -- 3D wireframe drawing primitives
 *
 * All functions take ivp_renderer_t* directly and push vertices into
 * the renderer's unified stream_3d via ivp_gpu_stream_push.
 */

#include "ivp_render_internal.h"
#include <string.h>
#include <math.h>

/* ── Internal vertex type (must match stream_3d layout) ────────────────── */

typedef struct draw_vertex {
    float pos[3];
    float col[3];
} draw_vertex_t;

/* ── Push helper ───────────────────────────────────────────────────────── */

static void push_line(ivp_renderer_t *r,
                      const float a[3], const float b[3],
                      const float col[3])
{
    draw_vertex_t *v = (draw_vertex_t *)ivp_gpu_stream_push(&r->stream_3d, 2);
    if (!v) return;
    memcpy(v[0].pos, a,   3 * sizeof(float));
    memcpy(v[0].col, col, 3 * sizeof(float));
    memcpy(v[1].pos, b,   3 * sizeof(float));
    memcpy(v[1].col, col, 3 * sizeof(float));
}

/* ── Tangent frame helper ──────────────────────────────────────────────── */

static void build_tangent_frame(const float n[3], float u[3], float v[3])
{
    float nn[3] = { n[0], n[1], n[2] };
    ivp_render_vec3_normalize(nn);
    if (fabsf(nn[1]) < 0.9f) {
        float tmp[3] = {0, 1, 0};
        ivp_render_vec3_cross(u, nn, tmp);
    } else {
        float tmp[3] = {1, 0, 0};
        ivp_render_vec3_cross(u, nn, tmp);
    }
    ivp_render_vec3_normalize(u);
    ivp_render_vec3_cross(v, nn, u);
}

/* ── Drawing primitives ────────────────────────────────────────────────── */

void ivp_render_draw_line(ivp_renderer_t *r,
                          const float a[3], const float b[3],
                          const float color[3])
{
    push_line(r, a, b, color);
}

void ivp_render_draw_thick_line(ivp_renderer_t *r,
                                const float a[3], const float b[3],
                                const float color[3], float thickness)
{
    push_line(r, a, b, color);
    float offset = thickness * 0.002f;
    for (int axis = 0; axis < 3; axis++) {
        float a2[3], b2[3];
        memcpy(a2, a, sizeof(float) * 3);
        memcpy(b2, b, sizeof(float) * 3);
        a2[axis] += offset; b2[axis] += offset;
        push_line(r, a2, b2, color);
        a2[axis] -= 2 * offset; b2[axis] -= 2 * offset;
        push_line(r, a2, b2, color);
    }
}

void ivp_render_draw_wire_box(ivp_renderer_t *r,
                              const float pos[3],
                              const float hs[3],
                              const float *rot,
                              const float col[3])
{
    float corners[8][3];
    int idx = 0;
    for (int z = -1; z <= 1; z += 2)
    for (int y = -1; y <= 1; y += 2)
    for (int x = -1; x <= 1; x += 2) {
        float lx = (float)x * hs[0];
        float ly = (float)y * hs[1];
        float lz = (float)z * hs[2];
        if (rot) {
            corners[idx][0] = pos[0] + rot[0]*lx + rot[3]*ly + rot[6]*lz;
            corners[idx][1] = pos[1] + rot[1]*lx + rot[4]*ly + rot[7]*lz;
            corners[idx][2] = pos[2] + rot[2]*lx + rot[5]*ly + rot[8]*lz;
        } else {
            corners[idx][0] = pos[0] + lx;
            corners[idx][1] = pos[1] + ly;
            corners[idx][2] = pos[2] + lz;
        }
        idx++;
    }
    static const int edges[12][2] = {
        {0,1},{2,3},{4,5},{6,7},
        {0,2},{1,3},{4,6},{5,7},
        {0,4},{1,5},{2,6},{3,7}
    };
    for (int i = 0; i < 12; i++)
        push_line(r, corners[edges[i][0]], corners[edges[i][1]], col);
}

void ivp_render_draw_wire_sphere_ex(ivp_renderer_t *r,
                                    const float pos[3],
                                    float radius,
                                    const float col[3],
                                    int segments)
{
    if (segments < 4) segments = 4;
    for (int ring = 0; ring < 3; ring++) {
        for (int i = 0; i < segments; i++) {
            float a0 = (float)i       / (float)segments * 2.0f * IVP_RENDER_PI_F;
            float a1 = (float)(i + 1) / (float)segments * 2.0f * IVP_RENDER_PI_F;
            float p0[3], p1[3];
            if (ring == 0) {
                p0[0] = pos[0]+cosf(a0)*radius; p0[1] = pos[1]+sinf(a0)*radius; p0[2] = pos[2];
                p1[0] = pos[0]+cosf(a1)*radius; p1[1] = pos[1]+sinf(a1)*radius; p1[2] = pos[2];
            } else if (ring == 1) {
                p0[0] = pos[0]+cosf(a0)*radius; p0[1] = pos[1]; p0[2] = pos[2]+sinf(a0)*radius;
                p1[0] = pos[0]+cosf(a1)*radius; p1[1] = pos[1]; p1[2] = pos[2]+sinf(a1)*radius;
            } else {
                p0[0] = pos[0]; p0[1] = pos[1]+cosf(a0)*radius; p0[2] = pos[2]+sinf(a0)*radius;
                p1[0] = pos[0]; p1[1] = pos[1]+cosf(a1)*radius; p1[2] = pos[2]+sinf(a1)*radius;
            }
            push_line(r, p0, p1, col);
        }
    }
}

void ivp_render_draw_wire_sphere(ivp_renderer_t *r,
                                 const float pos[3],
                                 float radius,
                                 const float col[3])
{
    ivp_render_draw_wire_sphere_ex(r, pos, radius, col, r->sphere_segments);
}

void ivp_render_draw_wire_cylinder(ivp_renderer_t *r,
                                   const float base[3],
                                   const float axis[3],
                                   float height, float radius,
                                   int segments,
                                   const float color[3])
{
    if (segments < 4) segments = 4;
    float u[3], v[3];
    build_tangent_frame(axis, u, v);

    float top[3] = {
        base[0] + axis[0]*height,
        base[1] + axis[1]*height,
        base[2] + axis[2]*height
    };

    for (int i = 0; i < segments; i++) {
        float a0 = (float)i       / (float)segments * 2.0f * IVP_RENDER_PI_F;
        float a1 = (float)(i + 1) / (float)segments * 2.0f * IVP_RENDER_PI_F;
        float c0 = cosf(a0), s0 = sinf(a0);
        float c1 = cosf(a1), s1 = sinf(a1);
        float b0[3], b1[3], t0[3], t1[3];
        for (int k = 0; k < 3; k++) {
            b0[k] = base[k] + (u[k]*c0 + v[k]*s0)*radius;
            b1[k] = base[k] + (u[k]*c1 + v[k]*s1)*radius;
            t0[k] = top[k]  + (u[k]*c0 + v[k]*s0)*radius;
            t1[k] = top[k]  + (u[k]*c1 + v[k]*s1)*radius;
        }
        push_line(r, b0, b1, color);
        push_line(r, t0, t1, color);
        push_line(r, b0, t0, color);
    }
}

void ivp_render_draw_wire_quad(ivp_renderer_t *r,
                               const float corners[4][3],
                               const float color[3])
{
    push_line(r, corners[0], corners[1], color);
    push_line(r, corners[1], corners[2], color);
    push_line(r, corners[2], corners[3], color);
    push_line(r, corners[3], corners[0], color);
    push_line(r, corners[0], corners[2], color);
}

void ivp_render_draw_arrow(ivp_renderer_t *r,
                           const float from[3], const float to[3],
                           const float color[3], float head_size)
{
    push_line(r, from, to, color);
    float dir[3];
    ivp_render_vec3_sub(dir, from, to);
    float len = sqrtf(ivp_render_vec3_dot(dir, dir));
    if (len < 1e-6f) return;
    dir[0] /= len; dir[1] /= len; dir[2] /= len;

    float perp[3];
    if (fabsf(dir[1]) < 0.9f) {
        float tmp[3] = {0, 1, 0};
        ivp_render_vec3_cross(perp, dir, tmp);
    } else {
        float tmp[3] = {1, 0, 0};
        ivp_render_vec3_cross(perp, dir, tmp);
    }
    ivp_render_vec3_normalize(perp);
    float perp2[3];
    ivp_render_vec3_cross(perp2, dir, perp);

    for (int i = 0; i < 4; i++) {
        float angle = (float)i * IVP_RENDER_PI_F * 0.5f;
        float c = cosf(angle), s = sinf(angle);
        float tip[3];
        tip[0] = to[0] + (dir[0] + perp[0]*c + perp2[0]*s) * head_size * 0.3f;
        tip[1] = to[1] + (dir[1] + perp[1]*c + perp2[1]*s) * head_size * 0.3f;
        tip[2] = to[2] + (dir[2] + perp[2]*c + perp2[2]*s) * head_size * 0.3f;
        push_line(r, to, tip, color);
    }
}

void ivp_render_draw_ground_grid(ivp_renderer_t *r,
                                 float extent, float spacing,
                                 const float col[3])
{
    for (float x = -extent; x <= extent + 0.01f; x += spacing) {
        float a[3] = {x, 0.0f, -extent};
        float b[3] = {x, 0.0f,  extent};
        push_line(r, a, b, col);
    }
    for (float z = -extent; z <= extent + 0.01f; z += spacing) {
        float a[3] = {-extent, 0.0f, z};
        float b[3] = { extent, 0.0f, z};
        push_line(r, a, b, col);
    }
}

void ivp_render_draw_axes(ivp_renderer_t *r,
                          const float pos[3],
                          const float *rot,
                          float scale)
{
    const float red[3]   = {1, 0, 0};
    const float green[3] = {0, 1, 0};
    const float blue[3]  = {0, 0, 1};
    float x[3], y[3], z[3];
    if (rot) {
        x[0] = pos[0]+rot[0]*scale; x[1] = pos[1]+rot[1]*scale; x[2] = pos[2]+rot[2]*scale;
        y[0] = pos[0]+rot[3]*scale; y[1] = pos[1]+rot[4]*scale; y[2] = pos[2]+rot[5]*scale;
        z[0] = pos[0]+rot[6]*scale; z[1] = pos[1]+rot[7]*scale; z[2] = pos[2]+rot[8]*scale;
    } else {
        x[0] = pos[0]+scale; x[1] = pos[1]; x[2] = pos[2];
        y[0] = pos[0]; y[1] = pos[1]+scale; y[2] = pos[2];
        z[0] = pos[0]; z[1] = pos[1]; z[2] = pos[2]+scale;
    }
    push_line(r, pos, x, red);
    push_line(r, pos, y, green);
    push_line(r, pos, z, blue);
}

void ivp_render_draw_circle(ivp_renderer_t *r,
                            const float center[3],
                            const float normal[3],
                            float radius, int segments,
                            const float color[3])
{
    if (segments < 4) segments = 4;
    float u[3], v[3];
    build_tangent_frame(normal, u, v);

    for (int i = 0; i < segments; i++) {
        float a0 = (float)i       / (float)segments * 2.0f * IVP_RENDER_PI_F;
        float a1 = (float)(i + 1) / (float)segments * 2.0f * IVP_RENDER_PI_F;
        float c0 = cosf(a0), s0 = sinf(a0);
        float c1 = cosf(a1), s1 = sinf(a1);
        float p0[3], p1[3];
        for (int k = 0; k < 3; k++) {
            p0[k] = center[k] + (u[k]*c0 + v[k]*s0)*radius;
            p1[k] = center[k] + (u[k]*c1 + v[k]*s1)*radius;
        }
        push_line(r, p0, p1, color);
    }
}

void ivp_render_draw_wire_cone(ivp_renderer_t *r,
                               const float apex[3],
                               const float base_center[3],
                               float base_radius, int segments,
                               const float color[3])
{
    if (segments < 4) segments = 4;
    float axis[3];
    ivp_render_vec3_sub(axis, apex, base_center);
    float nn[3] = { axis[0], axis[1], axis[2] };
    ivp_render_vec3_normalize(nn);

    float u[3], v[3];
    build_tangent_frame(nn, u, v);

    for (int i = 0; i < segments; i++) {
        float a0 = (float)i       / (float)segments * 2.0f * IVP_RENDER_PI_F;
        float a1 = (float)(i + 1) / (float)segments * 2.0f * IVP_RENDER_PI_F;
        float c0 = cosf(a0), s0 = sinf(a0);
        float c1 = cosf(a1), s1 = sinf(a1);
        float b0[3], b1[3];
        for (int k = 0; k < 3; k++) {
            b0[k] = base_center[k] + (u[k]*c0 + v[k]*s0)*base_radius;
            b1[k] = base_center[k] + (u[k]*c1 + v[k]*s1)*base_radius;
        }
        push_line(r, b0, b1, color);
        push_line(r, b0, apex, color);
    }
}

void ivp_render_draw_wire_capsule(ivp_renderer_t *r,
                                  const float a[3], const float b[3],
                                  float radius, int segments,
                                  const float color[3])
{
    if (segments < 4) segments = 4;

    float axis[3];
    ivp_render_vec3_sub(axis, b, a);
    float len = sqrtf(ivp_render_vec3_dot(axis, axis));
    if (len < 1e-6f) {
        ivp_render_draw_wire_sphere_ex(r, a, radius, color, segments);
        return;
    }
    float nn[3] = { axis[0]/len, axis[1]/len, axis[2]/len };

    float u[3], v[3];
    build_tangent_frame(nn, u, v);

    /* Cylinder body */
    for (int i = 0; i < segments; i++) {
        float ang0 = (float)i       / (float)segments * 2.0f * IVP_RENDER_PI_F;
        float ang1 = (float)(i + 1) / (float)segments * 2.0f * IVP_RENDER_PI_F;
        float c0 = cosf(ang0), s0 = sinf(ang0);
        float c1 = cosf(ang1), s1 = sinf(ang1);
        float ba0[3], ba1[3], bb0[3], bb1[3];
        for (int k = 0; k < 3; k++) {
            float off0 = (u[k]*c0 + v[k]*s0)*radius;
            float off1 = (u[k]*c1 + v[k]*s1)*radius;
            ba0[k] = a[k] + off0;
            ba1[k] = a[k] + off1;
            bb0[k] = b[k] + off0;
            bb1[k] = b[k] + off1;
        }
        push_line(r, ba0, ba1, color);
        push_line(r, bb0, bb1, color);
        push_line(r, ba0, bb0, color);
    }

    /* Hemisphere caps */
    int half_segs = segments / 2;
    if (half_segs < 2) half_segs = 2;

    /* Cap at 'a' (along -nn) */
    for (int ring = 0; ring < 2; ring++) {
        float *tang = (ring == 0) ? u : v;
        for (int i = 0; i < half_segs; i++) {
            float ang0 = IVP_RENDER_PI_F * 0.5f + (float)i       / (float)half_segs * IVP_RENDER_PI_F;
            float ang1 = IVP_RENDER_PI_F * 0.5f + (float)(i + 1) / (float)half_segs * IVP_RENDER_PI_F;
            float p0[3], p1[3];
            for (int k = 0; k < 3; k++) {
                p0[k] = a[k] + tang[k]*cosf(ang0)*radius + nn[k]*sinf(ang0)*radius;
                p1[k] = a[k] + tang[k]*cosf(ang1)*radius + nn[k]*sinf(ang1)*radius;
            }
            push_line(r, p0, p1, color);
        }
    }

    /* Cap at 'b' (along +nn) */
    for (int ring = 0; ring < 2; ring++) {
        float *tang = (ring == 0) ? u : v;
        for (int i = 0; i < half_segs; i++) {
            float ang0 = -(IVP_RENDER_PI_F * 0.5f) + (float)i       / (float)half_segs * IVP_RENDER_PI_F;
            float ang1 = -(IVP_RENDER_PI_F * 0.5f) + (float)(i + 1) / (float)half_segs * IVP_RENDER_PI_F;
            float p0[3], p1[3];
            for (int k = 0; k < 3; k++) {
                p0[k] = b[k] + tang[k]*cosf(ang0)*radius - nn[k]*sinf(ang0)*radius;
                p1[k] = b[k] + tang[k]*cosf(ang1)*radius - nn[k]*sinf(ang1)*radius;
            }
            push_line(r, p0, p1, color);
        }
    }
}

void ivp_render_draw_plane(ivp_renderer_t *r,
                           const float center[3],
                           const float normal[3],
                           float extent,
                           const float color[3])
{
    float u[3], v[3];
    build_tangent_frame(normal, u, v);

    int lines = 5;
    float step = extent * 2.0f / (float)lines;
    for (int i = 0; i <= lines; i++) {
        float t = -extent + step * (float)i;
        float a0[3], a1[3], b0[3], b1[3];
        for (int k = 0; k < 3; k++) {
            a0[k] = center[k] + u[k]*t + v[k]*(-extent);
            a1[k] = center[k] + u[k]*t + v[k]*extent;
            b0[k] = center[k] + u[k]*(-extent) + v[k]*t;
            b1[k] = center[k] + u[k]*extent    + v[k]*t;
        }
        push_line(r, a0, a1, color);
        push_line(r, b0, b1, color);
    }

    float nn[3] = { normal[0], normal[1], normal[2] };
    ivp_render_vec3_normalize(nn);
    float tip[3];
    for (int k = 0; k < 3; k++)
        tip[k] = center[k] + nn[k] * extent * 0.3f;
    ivp_render_draw_arrow(r, center, tip, color, extent * 0.08f);
}

void ivp_render_draw_aabb(ivp_renderer_t *r,
                          const float mn[3], const float mx[3],
                          const float color[3])
{
    float pos[3], hs[3];
    for (int k = 0; k < 3; k++) {
        pos[k] = (mn[k] + mx[k]) * 0.5f;
        hs[k]  = (mx[k] - mn[k]) * 0.5f;
    }
    ivp_render_draw_wire_box(r, pos, hs, NULL, color);
}

void ivp_render_draw_cross(ivp_renderer_t *r,
                           const float pos[3],
                           float size,
                           const float color[3])
{
    float half = size * 0.5f;
    float a[3], b[3];
    /* X axis */
    a[0] = pos[0]-half; a[1] = pos[1]; a[2] = pos[2];
    b[0] = pos[0]+half; b[1] = pos[1]; b[2] = pos[2];
    push_line(r, a, b, color);
    /* Y axis */
    a[0] = pos[0]; a[1] = pos[1]-half; a[2] = pos[2];
    b[0] = pos[0]; b[1] = pos[1]+half; b[2] = pos[2];
    push_line(r, a, b, color);
    /* Z axis */
    a[0] = pos[0]; a[1] = pos[1]; a[2] = pos[2]-half;
    b[0] = pos[0]; b[1] = pos[1]; b[2] = pos[2]+half;
    push_line(r, a, b, color);
}

void ivp_render_draw_point(ivp_renderer_t *r,
                           const float pos[3],
                           float size,
                           const float color[3])
{
    float h = size * 0.5f;
    float px[3] = { pos[0]+h, pos[1],   pos[2]   };
    float nx[3] = { pos[0]-h, pos[1],   pos[2]   };
    float py[3] = { pos[0],   pos[1]+h, pos[2]   };
    float ny[3] = { pos[0],   pos[1]-h, pos[2]   };
    float pz[3] = { pos[0],   pos[1],   pos[2]+h };
    float nz[3] = { pos[0],   pos[1],   pos[2]-h };

    push_line(r, px, py, color);
    push_line(r, py, nx, color);
    push_line(r, nx, ny, color);
    push_line(r, ny, px, color);
    push_line(r, pz, py, color);
    push_line(r, py, nz, color);
    push_line(r, nz, ny, color);
    push_line(r, ny, pz, color);
    push_line(r, px, pz, color);
    push_line(r, pz, nx, color);
    push_line(r, nx, nz, color);
    push_line(r, nz, px, color);
}

void ivp_render_draw_contact(ivp_renderer_t *r,
                             const float pos[3],
                             const float normal[3],
                             float normal_length,
                             const float color[3])
{
    ivp_render_draw_cross(r, pos, normal_length * 0.3f, color);
    float tip[3];
    tip[0] = pos[0] + normal[0] * normal_length;
    tip[1] = pos[1] + normal[1] * normal_length;
    tip[2] = pos[2] + normal[2] * normal_length;
    ivp_render_draw_arrow(r, pos, tip, color, normal_length * 0.15f);
}