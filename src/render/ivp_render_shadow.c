/* ivp_render_shadow.c -- Directional light shadow map
 *
 * One orthographic depth pass from the light over a box around the camera
 * target; the instanced solid shader compares against it with a 3x3 tap
 * PCF.  Only solids cast and receive shadows.
 */

#include "ivp_render_internal.h"
#include <math.h>
#include <string.h>

#define SHADOW_MAP_SIZE 2048

/* World -> light clip space with depth in [0,1] (SDL GPU / Metal), covering
 * [-extent, extent]^2 around `center`.  light_dir points towards the light.
 * The window is snapped to whole shadow texels so edges do not shimmer when
 * the camera target moves. */
void ivp_shadow_light_matrix(float out[16], const float light_dir[3],
                             const float center[3], float extent, int map_size)
{
    float l[3] = { light_dir[0], light_dir[1], light_dir[2] };
    ivp_render_vec3_normalize(l);

    const float dist = extent * 3.0f;
    const float eye[3] = { center[0] + l[0] * dist, center[1] + l[1] * dist,
                           center[2] + l[2] * dist };
    float up[3] = { 0.0f, 1.0f, 0.0f };
    if (fabsf(l[1]) > 0.99f) { up[0] = 1.0f; up[1] = 0.0f; }   /* light straight up/down */

    float view[16], proj[16];
    ivp_render_mat4_lookat(view, eye, center, up);

    const float zn = dist - extent * 2.0f, zf = dist + extent * 2.0f;
    memset(proj, 0, sizeof(proj));
    proj[0]  = 1.0f / extent;
    proj[5]  = 1.0f / extent;
    proj[10] = -1.0f / (zf - zn);
    proj[14] = -zn / (zf - zn);
    proj[15] = 1.0f;

    ivp_render_mat4_mul(out, proj, view);

    /* snap: where does the target land in NDC xy, in whole texels? */
    const float texel = 2.0f / (float)map_size;
    float cx = out[0] * center[0] + out[4] * center[1] + out[8]  * center[2] + out[12];
    float cy = out[1] * center[0] + out[5] * center[1] + out[9]  * center[2] + out[13];
    out[12] += roundf(cx / texel) * texel - cx;
    out[13] += roundf(cy / texel) * texel - cy;
}

bool ivp_shadow_init(ivp_renderer_t *r)
{
    r->shadow_map = ivp_gpu_shadow_map_create(r->gpu, SHADOW_MAP_SIZE);
    if (!r->shadow_map) return false;
    r->shadow_map_size = SHADOW_MAP_SIZE;

    ivp_gpu_pipeline_desc_t desc;
    ivp_render_solid_pipeline_desc(&desc, IVP_GPU_PROGRAM_SHADOW);
    desc.depth_only = true;

    r->shadow_pipeline = ivp_gpu_pipeline_create(r->gpu, &desc);
    if (!r->shadow_pipeline) {
        ivp_gpu_shadow_map_destroy(r->gpu, r->shadow_map);
        r->shadow_map = NULL;
        return false;
    }
    r->shadows_enabled = true;
    return true;
}

void ivp_shadow_destroy(ivp_renderer_t *r)
{
    ivp_gpu_pipeline_destroy(r->gpu, r->shadow_pipeline);
    r->shadow_pipeline = NULL;
    ivp_gpu_shadow_map_destroy(r->gpu, r->shadow_map);
    r->shadow_map = NULL;
}

/* Render the depth pass for this frame's solids.  Sets shadow_valid and
 * light_vp; call between the uploads and the main pass. */
void ivp_shadow_render(ivp_renderer_t *r)
{
    r->shadow_valid = false;
    if (!r->shadows_enabled || !r->shadow_pipeline || !r->shadow_map) return;
    if (!ivp_solid_has_instances(r)) return;

    /* The box follows the camera: roughly what the camera can look at. */
    float extent = r->cam_dist * 0.8f;
    if (extent < 8.0f) extent = 8.0f;
    if (extent > 60.0f) extent = 60.0f;
    ivp_shadow_light_matrix(r->light_vp, r->lighting.light_dir, r->cam_target,
                            extent, r->shadow_map_size);

    ivp_gpu_pass_t *pass = ivp_gpu_shadow_pass_begin(r->gpu, r->shadow_map);
    if (!pass) return;
    ivp_gpu_bind_pipeline(pass, r->shadow_pipeline);
    ivp_gpu_push_uniforms(pass, r->light_vp, 64);
    ivp_solid_draw(r, pass);
    ivp_gpu_pass_end(pass);
    r->shadow_valid = true;
}

void ivp_render_set_shadows(ivp_renderer_t *r, bool on)
{
    r->shadows_enabled = on && r->shadow_pipeline != NULL;
}

bool ivp_render_get_shadows(const ivp_renderer_t *r)
{
    return r->shadows_enabled;
}
