/* ivp_render_pick.c -- Screen-to-world unprojection and ray intersection */

#include "ivp_render_internal.h"
#include <math.h>

/* ── Unproject ─────────────────────────────────────────────────────────── */

void ivp_render_unproject_ray_ex(const float vp_mat[16],
                                 int viewport_w, int viewport_h,
                                 float screen_x, float screen_y,
                                 float ray_origin[3],
                                 float ray_dir[3])
{
    float inv_vp[16];
    if (!ivp_render_mat4_invert(inv_vp, vp_mat)) {
        ray_origin[0] = ray_origin[1] = ray_origin[2] = 0.0f;
        ray_dir[0] = ray_dir[1] = 0.0f;
        ray_dir[2] = -1.0f;
        return;
    }

    float ndc_x = (screen_x / (float)viewport_w) * 2.0f - 1.0f;
    float ndc_y = 1.0f - (screen_y / (float)viewport_h) * 2.0f;

    float near_clip[4] = { ndc_x, ndc_y, -1.0f, 1.0f };
    float far_clip[4]  = { ndc_x, ndc_y,  1.0f, 1.0f };

    float near_world[4], far_world[4];
    for (int i = 0; i < 4; i++) {
        near_world[i] = inv_vp[i]*near_clip[0] + inv_vp[4+i]*near_clip[1]
                       + inv_vp[8+i]*near_clip[2] + inv_vp[12+i]*near_clip[3];
        far_world[i]  = inv_vp[i]*far_clip[0] + inv_vp[4+i]*far_clip[1]
                       + inv_vp[8+i]*far_clip[2] + inv_vp[12+i]*far_clip[3];
    }

    if (fabsf(near_world[3]) > 1e-8f) {
        float inv_w = 1.0f / near_world[3];
        near_world[0] *= inv_w;
        near_world[1] *= inv_w;
        near_world[2] *= inv_w;
    }
    if (fabsf(far_world[3]) > 1e-8f) {
        float inv_w = 1.0f / far_world[3];
        far_world[0] *= inv_w;
        far_world[1] *= inv_w;
        far_world[2] *= inv_w;
    }

    ray_origin[0] = near_world[0];
    ray_origin[1] = near_world[1];
    ray_origin[2] = near_world[2];

    ray_dir[0] = far_world[0] - near_world[0];
    ray_dir[1] = far_world[1] - near_world[1];
    ray_dir[2] = far_world[2] - near_world[2];
    ivp_render_vec3_normalize(ray_dir);
}

void ivp_render_unproject_ray(const ivp_renderer_t *r,
                              float screen_x, float screen_y,
                              float ray_origin[3],
                              float ray_dir[3])
{
    ivp_render_unproject_ray_ex(r->vp_mat, r->width, r->height,
                                screen_x, screen_y,
                                ray_origin, ray_dir);
}

/* ── Ray-sphere intersection ───────────────────────────────────────────── */

bool ivp_render_ray_sphere(const float ray_origin[3],
                           const float ray_dir[3],
                           const float sphere_center[3],
                           float sphere_radius,
                           float *t_out)
{
    float oc[3];
    oc[0] = ray_origin[0] - sphere_center[0];
    oc[1] = ray_origin[1] - sphere_center[1];
    oc[2] = ray_origin[2] - sphere_center[2];

    float a = ivp_render_vec3_dot(ray_dir, ray_dir);
    float b = 2.0f * ivp_render_vec3_dot(oc, ray_dir);
    float c = ivp_render_vec3_dot(oc, oc) - sphere_radius * sphere_radius;
    float disc = b * b - 4.0f * a * c;

    if (disc < 0.0f) return false;

    float sqrt_d = sqrtf(disc);
    float inv_2a = 1.0f / (2.0f * a);

    float t0 = (-b - sqrt_d) * inv_2a;
    float t1 = (-b + sqrt_d) * inv_2a;

    if (t0 > 0.0f) { if (t_out) *t_out = t0; return true; }
    if (t1 > 0.0f) { if (t_out) *t_out = t1; return true; }
    return false;
}

/* ── Ray-AABB intersection (slab method) ───────────────────────────────── */

bool ivp_render_ray_aabb(const float ray_origin[3],
                         const float ray_dir[3],
                         const float aabb_min[3],
                         const float aabb_max[3],
                         float *t_out)
{
    float tmin = -1e30f;
    float tmax =  1e30f;

    for (int i = 0; i < 3; i++) {
        if (fabsf(ray_dir[i]) < 1e-8f) {
            if (ray_origin[i] < aabb_min[i] || ray_origin[i] > aabb_max[i])
                return false;
        } else {
            float inv_d = 1.0f / ray_dir[i];
            float t0 = (aabb_min[i] - ray_origin[i]) * inv_d;
            float t1 = (aabb_max[i] - ray_origin[i]) * inv_d;
            if (t0 > t1) { float tmp = t0; t0 = t1; t1 = tmp; }
            if (t0 > tmin) tmin = t0;
            if (t1 < tmax) tmax = t1;
            if (tmin > tmax) return false;
        }
    }

    if (tmax < 0.0f) return false;
    if (t_out) *t_out = (tmin >= 0.0f) ? tmin : tmax;
    return true;
}