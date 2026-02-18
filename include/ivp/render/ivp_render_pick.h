/* ivp_render_pick.h -- Screen-to-world unprojection and ray intersection */
#ifndef IVP_RENDER_PICK_H
#define IVP_RENDER_PICK_H

#include <ivp/render/ivp_render_types.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Unproject a screen point to a world-space ray.
 * screen_x, screen_y are in pixel coordinates (top-left origin).
 * Outputs ray_origin[3] and ray_dir[3] (normalized). */
void ivp_render_unproject_ray(const ivp_renderer_t *r,
                              float screen_x, float screen_y,
                              float ray_origin[3],
                              float ray_dir[3]);

/* Unproject using explicit matrices (for custom camera setups). */
void ivp_render_unproject_ray_ex(const float vp_mat[16],
                                 int viewport_w, int viewport_h,
                                 float screen_x, float screen_y,
                                 float ray_origin[3],
                                 float ray_dir[3]);

/* Ray-sphere intersection test.
 * Returns true if ray hits sphere, with parametric t in *t_out
 * (hit point = origin + dir * t). */
bool ivp_render_ray_sphere(const float ray_origin[3],
                           const float ray_dir[3],
                           const float sphere_center[3],
                           float sphere_radius,
                           float *t_out);

/* Ray-AABB intersection test (slab method).
 * Returns true if ray hits box, with entry t in *t_out. */
bool ivp_render_ray_aabb(const float ray_origin[3],
                         const float ray_dir[3],
                         const float aabb_min[3],
                         const float aabb_max[3],
                         float *t_out);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_PICK_H */
