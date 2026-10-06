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
    if (!ivp_gpu_shadow_map_create(&r->shadow_map, &r->gpu, SHADOW_MAP_SIZE))
        return false;

    ivp_gpu_stream_desc_t desc = {0};
    ivp_gpu_attrib_desc_t vtx_attrs[2] = {
        { 0, 3, 0 },
        { 1, 3, 3 * (int)sizeof(float) },
    };
    ivp_gpu_attrib_desc_t inst_attrs[5] = {
        { 2, 4, 0 }, { 3, 4, 4 * (int)sizeof(float) },
        { 4, 4, 8 * (int)sizeof(float) }, { 5, 4, 12 * (int)sizeof(float) },
        { 6, 4, 16 * (int)sizeof(float) },
    };
    desc.capacity          = 1;
    desc.stride            = 6 * (int)sizeof(float);
    desc.attribs           = vtx_attrs;
    desc.attrib_count      = 2;
    desc.inst_attribs      = inst_attrs;
    desc.inst_attrib_count = 5;
    desc.inst_stride       = (int)sizeof(ivp_solid_instance_t);
    desc.depth_test        = true;
    desc.depth_write       = true;
    desc.depth_only        = true;
    desc.primitive_type    = SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    desc.cull_mode         = SDL_GPU_CULLMODE_NONE;

    r->shadow_pipeline = ivp_gpu_create_pipeline(&r->gpu, &desc,
        r->shaders.vs_shadow, r->shaders.vs_shadow_len,
        r->shaders.fs_shadow, r->shaders.fs_shadow_len);
    if (!r->shadow_pipeline) {
        ivp_gpu_shadow_map_destroy(&r->shadow_map, &r->gpu);
        return false;
    }
    r->shadows_enabled = true;
    return true;
}

void ivp_shadow_destroy(ivp_renderer_t *r)
{
    if (r->shadow_pipeline) {
        SDL_ReleaseGPUGraphicsPipeline(r->gpu.device, r->shadow_pipeline);
        r->shadow_pipeline = NULL;
    }
    ivp_gpu_shadow_map_destroy(&r->shadow_map, &r->gpu);
}

/* Render the depth pass for this frame's solids.  Sets shadow_valid and
 * light_vp; call between the uploads and the main pass. */
void ivp_shadow_render(ivp_renderer_t *r)
{
    r->shadow_valid = false;
    if (!r->shadows_enabled || !r->shadow_pipeline || !r->shadow_map.tex) return;
    if (!ivp_solid_has_instances(r)) return;

    /* The box follows the camera: roughly what the camera can look at. */
    float extent = r->cam_dist * 0.8f;
    if (extent < 8.0f) extent = 8.0f;
    if (extent > 60.0f) extent = 60.0f;
    ivp_shadow_light_matrix(r->light_vp, r->lighting.light_dir, r->cam_target,
                            extent, r->shadow_map.size);

    SDL_GPUDepthStencilTargetInfo dt = {0};
    dt.texture          = r->shadow_map.tex;
    dt.clear_depth      = 1.0f;
    dt.load_op          = SDL_GPU_LOADOP_CLEAR;
    dt.store_op         = SDL_GPU_STOREOP_STORE;
    dt.stencil_load_op  = SDL_GPU_LOADOP_DONT_CARE;
    dt.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;

    SDL_GPURenderPass *pass = SDL_BeginGPURenderPass(r->cmd, NULL, 0, &dt);
    if (!pass) return;
    SDL_BindGPUGraphicsPipeline(pass, r->shadow_pipeline);
    SDL_PushGPUVertexUniformData(r->cmd, 0, r->light_vp, 64);
    ivp_solid_draw(r, pass);
    SDL_EndGPURenderPass(pass);
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
