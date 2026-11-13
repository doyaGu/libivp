/* ivp_render_sdlgpu.c -- GPU backend on the SDL3 GPU API (Metal, Vulkan, D3D12)
 *
 * Implements ivp_render_gpu.h.  Per-frame buffers are triple-buffered: each
 * keeps a GPU buffer and an upload transfer buffer for each of the RING_FRAMES
 * frames in flight, used in turn, and ivp_gpu_frame_begin waits for the fence
 * of the frame that last used the current slot.  Shaders are SPIR-V or MSL
 * files loaded at device creation.
 */

#include "ivp_render_gpu.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef IVP_RENDER_SHADER_DIR_FALLBACK
#define IVP_RENDER_SHADER_DIR_FALLBACK "shaders"
#endif

#define RING_FRAMES 3

struct ivp_gpu_pass {
    ivp_gpu_device_t  *dev;
    SDL_GPURenderPass *rp;
};

struct ivp_gpu_pipeline {
    SDL_GPUGraphicsPipeline *pipe;
};

struct ivp_gpu_buffer {
    SDL_GPUBuffer         *gpu[RING_FRAMES];
    SDL_GPUTransferBuffer *transfer[RING_FRAMES];
};

struct ivp_gpu_mesh {
    SDL_GPUBuffer *vtx;
    SDL_GPUBuffer *idx;
    int            index_count;
};

struct ivp_gpu_shadow_map {
    SDL_GPUTexture *tex;
    SDL_GPUSampler *smp;
};

struct ivp_gpu_device {
    SDL_GPUDevice        *device;
    SDL_Window           *window;
    SDL_GPUShaderFormat   shader_format;
    SDL_GPUTextureFormat  swapchain_format;
    SDL_GPUSampleCount    sample_count;

    /* Shader code per program: [0] vertex, [1] fragment */
    unsigned char        *code[IVP_GPU_PROGRAM_COUNT][2];
    size_t                code_len[IVP_GPU_PROGRAM_COUNT][2];

    /* Depth target of the single-sample path */
    SDL_GPUTexture       *depth_texture;
    int                   depth_w, depth_h;

    /* MSAA colour and depth targets, resolved into the frame's target */
    SDL_GPUTexture       *msaa_color;
    SDL_GPUTexture       *msaa_depth;
    int                   msaa_w, msaa_h;
    bool                  msaa_enabled;

    /* Frame state, valid between frame_begin and frame_end */
    SDL_GPUCommandBuffer *cmd;
    SDL_GPUTexture       *swapchain_tex;
    SDL_GPUFence         *frame_fences[RING_FRAMES];
    int                   frame_ring;   /* buffer slot of this frame */
    int                   frame_w, frame_h;   /* targets, device pixels */
    ivp_gpu_pass_t        pass;

    /* Screenshot: the frame renders into capture_tex, which is blitted to
     * the swapchain and downloaded (SDL GPU cannot read a swapchain) */
    bool                   capturing;
    SDL_GPUTexture        *capture_tex;
    int                    capture_w, capture_h;
    SDL_GPUTransferBuffer *capture_tb;
    unsigned char         *capture_rgb;
};

/* ═══════════════════════════════════════════════════════════════════════════
 * Shader loading
 * ═══════════════════════════════════════════════════════════════════════════ */

static bool load_shader_file(unsigned char **out, size_t *out_len,
                             const char *file_name)
{
    char path[1024];

    /* 1. Environment variable override */
    const char *env_dir = SDL_getenv("IVP_RENDER_SHADER_DIR");
    if (env_dir && env_dir[0]) {
        SDL_snprintf(path, sizeof(path), "%s/%s", env_dir, file_name);
        void *data = SDL_LoadFile(path, out_len);
        if (data && *out_len > 0) { *out = (unsigned char *)data; return true; }
        if (data) SDL_free(data);
    }

    /* 2. Executable base path */
    const char *base = SDL_GetBasePath();
    if (base) {
        SDL_snprintf(path, sizeof(path), "%sshaders/%s", base, file_name);
        void *data = SDL_LoadFile(path, out_len);
        if (data && *out_len > 0) { *out = (unsigned char *)data; return true; }
        if (data) SDL_free(data);
    }

    /* 3. Build-time fallback directory */
    SDL_snprintf(path, sizeof(path), "%s/%s",
                 IVP_RENDER_SHADER_DIR_FALLBACK, file_name);
    {
        void *data = SDL_LoadFile(path, out_len);
        if (data && *out_len > 0) { *out = (unsigned char *)data; return true; }
        if (data) SDL_free(data);
    }

    /* 4. Relative to the working directory */
    SDL_snprintf(path, sizeof(path), "shaders/%s", file_name);
    {
        void *data = SDL_LoadFile(path, out_len);
        if (data && *out_len > 0) { *out = (unsigned char *)data; return true; }
        if (data) SDL_free(data);
    }

    fprintf(stderr, "Failed to load shader '%s'\n", file_name);
    *out = NULL;
    *out_len = 0;
    return false;
}

/* Shader files of each program: <name>.<stage>.<spv|msl> */
static const char *const program_files[IVP_GPU_PROGRAM_COUNT][2] = {
    [IVP_GPU_PROGRAM_LINE3D] = { "line3d.vert",     "line.frag"       },
    [IVP_GPU_PROGRAM_LINE2D] = { "line2d.vert",     "line.frag"       },
    [IVP_GPU_PROGRAM_SOLID]  = { "tri3d_inst.vert", "tri3d_inst.frag" },
    [IVP_GPU_PROGRAM_SHADOW] = { "shadow.vert",     "shadow.frag"     },
    [IVP_GPU_PROGRAM_RECT]   = { "hud_rect.vert",   "hud_rect.frag"   },
};

static void free_shaders(ivp_gpu_device_t *dev)
{
    for (int p = 0; p < IVP_GPU_PROGRAM_COUNT; p++) {
        for (int s = 0; s < 2; s++) {
            if (dev->code[p][s]) SDL_free(dev->code[p][s]);
            dev->code[p][s] = NULL;
            dev->code_len[p][s] = 0;
        }
    }
}

static bool load_shaders(ivp_gpu_device_t *dev)
{
    const char *ext = (dev->shader_format == SDL_GPU_SHADERFORMAT_MSL) ? "msl" : "spv";
    for (int p = 0; p < IVP_GPU_PROGRAM_COUNT; p++) {
        for (int s = 0; s < 2; s++) {
            char name[64];
            SDL_snprintf(name, sizeof(name), "%s.%s", program_files[p][s], ext);
            if (!load_shader_file(&dev->code[p][s], &dev->code_len[p][s], name)) {
                free_shaders(dev);
                return false;
            }
        }
    }
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Device
 * ═══════════════════════════════════════════════════════════════════════════ */

static SDL_GPUTexture *create_depth_texture(SDL_GPUDevice *device, int w, int h,
                                            SDL_GPUSampleCount samples)
{
    SDL_GPUTextureCreateInfo ci = {0};
    ci.type                 = SDL_GPU_TEXTURETYPE_2D;
    ci.format               = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    ci.width                = (Uint32)w;
    ci.height               = (Uint32)h;
    ci.layer_count_or_depth = 1;
    ci.num_levels           = 1;
    ci.sample_count         = samples;
    ci.usage                = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET;
    return SDL_CreateGPUTexture(device, &ci);
}

static void ensure_depth(ivp_gpu_device_t *dev, int w, int h)
{
    if (w == dev->depth_w && h == dev->depth_h) return;

    SDL_GPUTexture *tex = create_depth_texture(dev->device, w, h, dev->sample_count);
    if (!tex) {
        fprintf(stderr, "Depth resize failed (%dx%d): %s\n", w, h, SDL_GetError());
        return;
    }
    if (dev->depth_texture)
        SDL_ReleaseGPUTexture(dev->device, dev->depth_texture);
    dev->depth_texture = tex;
    dev->depth_w = w;
    dev->depth_h = h;
}

static void msaa_destroy(ivp_gpu_device_t *dev)
{
    if (dev->msaa_color) SDL_ReleaseGPUTexture(dev->device, dev->msaa_color);
    if (dev->msaa_depth) SDL_ReleaseGPUTexture(dev->device, dev->msaa_depth);
    dev->msaa_color = NULL;
    dev->msaa_depth = NULL;
    dev->msaa_enabled = false;
}

static void ensure_msaa(ivp_gpu_device_t *dev, int w, int h)
{
    SDL_GPUSampleCount sc = dev->sample_count;
    if (sc <= SDL_GPU_SAMPLECOUNT_1) {
        dev->msaa_enabled = false;
        return;
    }
    if (dev->msaa_color && dev->msaa_w == w && dev->msaa_h == h)
        return;  /* already the right size */

    msaa_destroy(dev);

    SDL_GPUTextureCreateInfo ci = {0};
    ci.type                 = SDL_GPU_TEXTURETYPE_2D;
    ci.format               = dev->swapchain_format;
    ci.width                = (Uint32)w;
    ci.height               = (Uint32)h;
    ci.layer_count_or_depth = 1;
    ci.num_levels           = 1;
    ci.sample_count         = sc;
    ci.usage                = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET;
    dev->msaa_color = SDL_CreateGPUTexture(dev->device, &ci);
    if (!dev->msaa_color) {
        fprintf(stderr, "MSAA color texture failed: %s\n", SDL_GetError());
        return;
    }
    dev->msaa_depth = create_depth_texture(dev->device, w, h, sc);
    if (!dev->msaa_depth) {
        fprintf(stderr, "MSAA depth texture failed: %s\n", SDL_GetError());
        SDL_ReleaseGPUTexture(dev->device, dev->msaa_color);
        dev->msaa_color = NULL;
        return;
    }
    dev->msaa_w = w;
    dev->msaa_h = h;
    dev->msaa_enabled = true;
}

static bool select_shader_format(ivp_gpu_device_t *dev,
                                 SDL_GPUShaderFormat supported)
{
    const char *platform = SDL_GetPlatform();
    bool is_apple = (platform && (!SDL_strcasecmp(platform, "macOS") ||
                                  !SDL_strcasecmp(platform, "iOS")));

    if (is_apple && (supported & SDL_GPU_SHADERFORMAT_MSL)) {
        dev->shader_format = SDL_GPU_SHADERFORMAT_MSL;
    } else if (supported & SDL_GPU_SHADERFORMAT_SPIRV) {
        dev->shader_format = SDL_GPU_SHADERFORMAT_SPIRV;
    } else if (supported & SDL_GPU_SHADERFORMAT_MSL) {
        dev->shader_format = SDL_GPU_SHADERFORMAT_MSL;
    } else {
        return false;
    }

    const char *driver = SDL_GetGPUDeviceDriver(dev->device);
    if (driver && !SDL_strcasecmp(driver, "metal") &&
        dev->shader_format != SDL_GPU_SHADERFORMAT_MSL) {
        return false;
    }
    return true;
}

SDL_WindowFlags ivp_gpu_window_flags(int msaa_samples)
{
    (void)msaa_samples;
    return 0;
}

ivp_result_t ivp_gpu_device_create(ivp_gpu_device_t **out, SDL_Window *window,
                                   int msaa_samples, bool vsync)
{
    *out = NULL;
    ivp_gpu_device_t *dev = (ivp_gpu_device_t *)calloc(1, sizeof(*dev));
    if (!dev) return IVP_ERR_OUT_OF_MEMORY;
    dev->window = window;

    dev->device = SDL_CreateGPUDevice(
        SDL_GPU_SHADERFORMAT_SPIRV | SDL_GPU_SHADERFORMAT_MSL,
        false, NULL);
    if (!dev->device) {
        fprintf(stderr, "SDL_CreateGPUDevice failed: %s\n", SDL_GetError());
        free(dev);
        return IVP_ERR_RENDER_GPU_DEVICE;
    }

    if (!SDL_ClaimWindowForGPUDevice(dev->device, window)) {
        fprintf(stderr, "SDL_ClaimWindowForGPUDevice failed: %s\n", SDL_GetError());
        SDL_DestroyGPUDevice(dev->device);
        free(dev);
        return IVP_ERR_RENDER_GPU_DEVICE;
    }

    ivp_result_t res = IVP_ERR_RENDER_GPU_DEVICE;
    SDL_GPUShaderFormat supported = SDL_GetGPUShaderFormats(dev->device);
    if (!select_shader_format(dev, supported)) {
        fprintf(stderr, "Unsupported shader format (mask=0x%x)\n",
                (unsigned)supported);
        goto fail;
    }

    dev->swapchain_format = SDL_GetGPUSwapchainTextureFormat(dev->device, window);
    dev->sample_count = (msaa_samples >= 4) ? SDL_GPU_SAMPLECOUNT_4
                      : (msaa_samples >= 2) ? SDL_GPU_SAMPLECOUNT_2
                      :                       SDL_GPU_SAMPLECOUNT_1;

    int w, h;
    SDL_GetWindowSizeInPixels(window, &w, &h);
    dev->depth_texture = create_depth_texture(dev->device, w, h, dev->sample_count);
    if (!dev->depth_texture) {
        fprintf(stderr, "Depth texture creation failed: %s\n", SDL_GetError());
        goto fail;
    }
    dev->depth_w = w;
    dev->depth_h = h;

    SDL_SetGPUSwapchainParameters(dev->device, window,
        SDL_GPU_SWAPCHAINCOMPOSITION_SDR,
        vsync ? SDL_GPU_PRESENTMODE_VSYNC : SDL_GPU_PRESENTMODE_IMMEDIATE);

    if (!load_shaders(dev)) {
        res = IVP_ERR_RENDER_SHADER;
        goto fail;
    }

    ensure_msaa(dev, w, h);
    *out = dev;
    return IVP_OK;

fail:
    if (dev->depth_texture) SDL_ReleaseGPUTexture(dev->device, dev->depth_texture);
    SDL_ReleaseWindowFromGPUDevice(dev->device, window);
    SDL_DestroyGPUDevice(dev->device);
    free(dev);
    return res;
}

void ivp_gpu_device_destroy(ivp_gpu_device_t *dev)
{
    if (!dev) return;
    SDL_WaitForGPUIdle(dev->device);
    for (int i = 0; i < RING_FRAMES; i++) {
        if (dev->frame_fences[i]) SDL_ReleaseGPUFence(dev->device, dev->frame_fences[i]);
        dev->frame_fences[i] = NULL;
    }
    msaa_destroy(dev);
    if (dev->capture_tb) SDL_ReleaseGPUTransferBuffer(dev->device, dev->capture_tb);
    if (dev->capture_tex) SDL_ReleaseGPUTexture(dev->device, dev->capture_tex);
    free(dev->capture_rgb);
    if (dev->depth_texture) SDL_ReleaseGPUTexture(dev->device, dev->depth_texture);
    free_shaders(dev);
    SDL_ReleaseWindowFromGPUDevice(dev->device, dev->window);
    SDL_DestroyGPUDevice(dev->device);
    free(dev);
}

void ivp_gpu_wait_idle(ivp_gpu_device_t *dev)
{
    if (dev) SDL_WaitForGPUIdle(dev->device);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Frame
 * ═══════════════════════════════════════════════════════════════════════════ */

ivp_gpu_frame_status_t ivp_gpu_frame_begin(ivp_gpu_device_t *dev)
{
    /* Depth + MSAA resize */
    int pw, ph;
    SDL_GetWindowSizeInPixels(dev->window, &pw, &ph);
    ensure_depth(dev, pw, ph);
    ensure_msaa(dev, pw, ph);
    if (!dev->depth_texture) return IVP_GPU_FRAME_SKIP;
    dev->frame_w = pw;
    dev->frame_h = ph;

    /* Wait on the fence of the frame that last used this buffer slot, so
     * its transfer buffers are safe to reuse before we upload into them. */
    SDL_GPUFence **fence = &dev->frame_fences[dev->frame_ring];
    if (*fence) {
        SDL_WaitForGPUFences(dev->device, true, fence, 1);
        SDL_ReleaseGPUFence(dev->device, *fence);
        *fence = NULL;
    }

    dev->cmd = SDL_AcquireGPUCommandBuffer(dev->device);
    if (!dev->cmd) return IVP_GPU_FRAME_ERROR;

    dev->swapchain_tex = NULL;
    Uint32 sw, sh;
    if (!SDL_WaitAndAcquireGPUSwapchainTexture(dev->cmd, dev->window,
                                                &dev->swapchain_tex, &sw, &sh)) {
        SDL_CancelGPUCommandBuffer(dev->cmd);
        dev->cmd = NULL;
        return IVP_GPU_FRAME_SKIP;
    }
    (void)sw; (void)sh;
    if (!dev->swapchain_tex) {
        SDL_SubmitGPUCommandBuffer(dev->cmd);
        dev->cmd = NULL;
        return IVP_GPU_FRAME_SKIP;
    }
    return IVP_GPU_FRAME_READY;
}

static void capture_record(ivp_gpu_device_t *dev);
static void capture_finish(ivp_gpu_device_t *dev, SDL_GPUFence *fence);

void ivp_gpu_frame_end(ivp_gpu_device_t *dev)
{
    if (dev->capturing) capture_record(dev);

    /* Submit and keep the fence for the triple-buffered slots */
    SDL_GPUFence *frame_fence = SDL_SubmitGPUCommandBufferAndAcquireFence(dev->cmd);
    if (dev->capturing) capture_finish(dev, frame_fence);

    /* Released in frame_begin when this slot comes round again */
    dev->frame_fences[dev->frame_ring] = frame_fence;
    dev->frame_ring = (dev->frame_ring + 1) % RING_FRAMES;

    dev->cmd = NULL;
    dev->swapchain_tex = NULL;
}

void ivp_gpu_frame_cancel(ivp_gpu_device_t *dev)
{
    if (dev->cmd) SDL_CancelGPUCommandBuffer(dev->cmd);
    dev->cmd = NULL;
    dev->swapchain_tex = NULL;
    dev->capturing = false;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Pipelines
 * ═══════════════════════════════════════════════════════════════════════════ */

static const char *shader_format_name(SDL_GPUShaderFormat fmt)
{
    if (fmt == SDL_GPU_SHADERFORMAT_SPIRV) return "SPIR-V";
    if (fmt == SDL_GPU_SHADERFORMAT_MSL)   return "MSL";
    return "UNKNOWN";
}

static SDL_GPUVertexElementFormat float_format(int components)
{
    return (components == 2) ? SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2 :
           (components == 4) ? SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4 :
                               SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
}

ivp_gpu_pipeline_t *ivp_gpu_pipeline_create(ivp_gpu_device_t *dev,
                                            const ivp_gpu_pipeline_desc_t *desc)
{
    const int prog = (int)desc->program;
    bool is_msl = (dev->shader_format == SDL_GPU_SHADERFORMAT_MSL);

    /* Every vertex shader reads one uniform block: [[buffer(0)]] in MSL,
     * set = 1, binding = 0 in SPIR-V (SDL GPU has no push constants). */
    SDL_GPUShaderCreateInfo vs_ci = {0};
    vs_ci.code                = dev->code[prog][0];
    vs_ci.code_size           = dev->code_len[prog][0];
    vs_ci.entrypoint          = is_msl ? "vs_main" : "main";
    vs_ci.format              = dev->shader_format;
    vs_ci.stage               = SDL_GPU_SHADERSTAGE_VERTEX;
    vs_ci.num_uniform_buffers = 1;

    SDL_GPUShader *vs = SDL_CreateGPUShader(dev->device, &vs_ci);
    if (!vs) {
        fprintf(stderr, "VS creation failed (%s): %s\n",
                shader_format_name(dev->shader_format), SDL_GetError());
        return NULL;
    }

    SDL_GPUShaderCreateInfo fs_ci = {0};
    fs_ci.code         = dev->code[prog][1];
    fs_ci.code_size    = dev->code_len[prog][1];
    fs_ci.entrypoint   = is_msl ? "fs_main" : "main";
    fs_ci.format       = dev->shader_format;
    fs_ci.stage        = SDL_GPU_SHADERSTAGE_FRAGMENT;
    fs_ci.num_samplers = desc->shadow_sampler ? 1u : 0u;

    SDL_GPUShader *fs = SDL_CreateGPUShader(dev->device, &fs_ci);
    if (!fs) {
        fprintf(stderr, "FS creation failed (%s): %s\n",
                shader_format_name(dev->shader_format), SDL_GetError());
        SDL_ReleaseGPUShader(dev->device, vs);
        return NULL;
    }

    /* Vertex buffer descriptions -- slot 0 always present */
    SDL_GPUVertexBufferDescription vbufs[2] = {{0}};
    int num_vbufs = 1;
    vbufs[0].slot       = 0;
    vbufs[0].pitch      = (Uint32)desc->stride;
    vbufs[0].input_rate = SDL_GPU_VERTEXINPUTRATE_VERTEX;

    if (desc->inst_attribs && desc->inst_attrib_count > 0) {
        vbufs[1].slot       = 1;
        vbufs[1].pitch      = (Uint32)desc->inst_stride;
        vbufs[1].input_rate = SDL_GPU_VERTEXINPUTRATE_INSTANCE;
        num_vbufs = 2;
    }

    /* Build attribute list */
    SDL_GPUVertexAttribute attrs[16] = {0};
    int total_attrs = 0;

    int n = (desc->attrib_count < 8) ? desc->attrib_count : 8;
    for (int i = 0; i < n; i++) {
        attrs[total_attrs].location    = (Uint32)desc->attribs[i].location;
        attrs[total_attrs].buffer_slot = 0;
        attrs[total_attrs].format      = float_format(desc->attribs[i].components);
        attrs[total_attrs].offset      = (Uint32)desc->attribs[i].offset;
        total_attrs++;
    }

    if (desc->inst_attribs) {
        int ni = (desc->inst_attrib_count < 8) ? desc->inst_attrib_count : 8;
        for (int i = 0; i < ni; i++) {
            attrs[total_attrs].location    = (Uint32)desc->inst_attribs[i].location;
            attrs[total_attrs].buffer_slot = 1;
            attrs[total_attrs].format      = float_format(desc->inst_attribs[i].components);
            attrs[total_attrs].offset      = (Uint32)desc->inst_attribs[i].offset;
            total_attrs++;
        }
    }

    SDL_GPUVertexInputState vis = {0};
    vis.vertex_buffer_descriptions = vbufs;
    vis.num_vertex_buffers         = (Uint32)num_vbufs;
    vis.vertex_attributes          = attrs;
    vis.num_vertex_attributes      = (Uint32)total_attrs;

    SDL_GPUDepthStencilState dss = {0};
    dss.compare_op         = desc->depth_always ? SDL_GPU_COMPAREOP_ALWAYS
                                                : SDL_GPU_COMPAREOP_LESS;
    dss.enable_depth_test  = desc->depth_test || desc->depth_always;
    dss.enable_depth_write = desc->depth_write;

    SDL_GPUColorTargetDescription ct = {0};
    ct.format = dev->swapchain_format;
    if (desc->blend) {
        ct.blend_state.enable_blend          = true;
        ct.blend_state.src_color_blendfactor = SDL_GPU_BLENDFACTOR_SRC_ALPHA;
        ct.blend_state.dst_color_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
        ct.blend_state.color_blend_op        = SDL_GPU_BLENDOP_ADD;
        ct.blend_state.src_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE;
        ct.blend_state.dst_alpha_blendfactor = SDL_GPU_BLENDFACTOR_ONE_MINUS_SRC_ALPHA;
        ct.blend_state.alpha_blend_op        = SDL_GPU_BLENDOP_ADD;
    }

    SDL_GPUGraphicsPipelineCreateInfo gp = {0};
    gp.vertex_shader       = vs;
    gp.fragment_shader     = fs;
    gp.vertex_input_state  = vis;
    gp.primitive_type      = desc->primitive == IVP_GPU_PRIMITIVE_LINES
                                 ? SDL_GPU_PRIMITIVETYPE_LINELIST
                                 : SDL_GPU_PRIMITIVETYPE_TRIANGLELIST;
    gp.depth_stencil_state = dss;
    gp.target_info.color_target_descriptions = &ct;
    gp.target_info.num_color_targets         = 1;
    if (desc->depth_only) {
        gp.target_info.color_target_descriptions = NULL;
        gp.target_info.num_color_targets         = 0;
        gp.rasterizer_state.enable_depth_bias         = true;
        gp.rasterizer_state.depth_bias_constant_factor = 2.0f;
        gp.rasterizer_state.depth_bias_slope_factor    = 2.5f;
    }
    gp.target_info.has_depth_stencil_target  = desc->depth_test || desc->depth_write ||
                                               desc->depth_always;
    gp.target_info.depth_stencil_format      = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    gp.rasterizer_state.fill_mode            = SDL_GPU_FILLMODE_FILL;
    gp.rasterizer_state.cull_mode            = SDL_GPU_CULLMODE_NONE;
    gp.multisample_state.sample_count        = desc->depth_only ? SDL_GPU_SAMPLECOUNT_1
                                                                : dev->sample_count;

    SDL_GPUGraphicsPipeline *pipe = SDL_CreateGPUGraphicsPipeline(dev->device, &gp);

    SDL_ReleaseGPUShader(dev->device, vs);
    SDL_ReleaseGPUShader(dev->device, fs);

    if (!pipe) {
        fprintf(stderr, "Pipeline creation failed: %s\n", SDL_GetError());
        return NULL;
    }
    ivp_gpu_pipeline_t *p = (ivp_gpu_pipeline_t *)malloc(sizeof(*p));
    if (!p) {
        SDL_ReleaseGPUGraphicsPipeline(dev->device, pipe);
        return NULL;
    }
    p->pipe = pipe;
    return p;
}

void ivp_gpu_pipeline_destroy(ivp_gpu_device_t *dev, ivp_gpu_pipeline_t *pipe)
{
    if (!pipe) return;
    SDL_ReleaseGPUGraphicsPipeline(dev->device, pipe->pipe);
    free(pipe);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Buffers and meshes
 * ═══════════════════════════════════════════════════════════════════════════ */

ivp_gpu_buffer_t *ivp_gpu_buffer_create(ivp_gpu_device_t *dev, size_t size)
{
    ivp_gpu_buffer_t *b = (ivp_gpu_buffer_t *)calloc(1, sizeof(*b));
    if (!b) return NULL;
    for (int i = 0; i < RING_FRAMES; i++) {
        SDL_GPUBufferCreateInfo buf_ci = {0};
        buf_ci.usage = SDL_GPU_BUFFERUSAGE_VERTEX;
        buf_ci.size  = (Uint32)size;
        b->gpu[i] = SDL_CreateGPUBuffer(dev->device, &buf_ci);
        if (!b->gpu[i]) goto fail;

        SDL_GPUTransferBufferCreateInfo tb_ci = {0};
        tb_ci.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
        tb_ci.size  = (Uint32)size;
        b->transfer[i] = SDL_CreateGPUTransferBuffer(dev->device, &tb_ci);
        if (!b->transfer[i]) goto fail;
    }
    return b;

fail:
    ivp_gpu_buffer_destroy(dev, b);
    return NULL;
}

void ivp_gpu_buffer_destroy(ivp_gpu_device_t *dev, ivp_gpu_buffer_t *buf)
{
    if (!buf) return;
    for (int i = 0; i < RING_FRAMES; i++) {
        if (buf->transfer[i]) SDL_ReleaseGPUTransferBuffer(dev->device, buf->transfer[i]);
        if (buf->gpu[i]) SDL_ReleaseGPUBuffer(dev->device, buf->gpu[i]);
    }
    free(buf);
}

void ivp_gpu_buffer_upload(ivp_gpu_device_t *dev, ivp_gpu_buffer_t *buf,
                           const void *data, size_t size)
{
    /* The fence wait in frame_begin makes this frame's slot safe to reuse. */
    const int slot = dev->frame_ring;

    void *mapped = SDL_MapGPUTransferBuffer(dev->device, buf->transfer[slot], true);
    if (!mapped) return;
    memcpy(mapped, data, size);
    SDL_UnmapGPUTransferBuffer(dev->device, buf->transfer[slot]);

    SDL_GPUCopyPass *copy = SDL_BeginGPUCopyPass(dev->cmd);
    SDL_GPUTransferBufferLocation src = {0};
    src.transfer_buffer = buf->transfer[slot];
    SDL_GPUBufferRegion dst = {0};
    dst.buffer = buf->gpu[slot];
    dst.size   = (Uint32)size;
    SDL_UploadToGPUBuffer(copy, &src, &dst, false);
    SDL_EndGPUCopyPass(copy);
}

ivp_gpu_mesh_t *ivp_gpu_mesh_create(ivp_gpu_device_t *dev,
                                    const void *vtx_data, int vtx_count,
                                    int vtx_stride,
                                    const Uint16 *idx_data, int idx_count)
{
    ivp_gpu_mesh_t *m = (ivp_gpu_mesh_t *)calloc(1, sizeof(*m));
    if (!m) return NULL;
    m->index_count = idx_count;

    Uint32 vtx_size = (Uint32)vtx_count * (Uint32)vtx_stride;
    Uint32 idx_size = (Uint32)idx_count * (Uint32)sizeof(Uint16);

    SDL_GPUBufferCreateInfo bc = {0};
    bc.usage = SDL_GPU_BUFFERUSAGE_VERTEX;
    bc.size  = vtx_size;
    m->vtx = SDL_CreateGPUBuffer(dev->device, &bc);
    bc.usage = SDL_GPU_BUFFERUSAGE_INDEX;
    bc.size  = idx_size;
    m->idx = SDL_CreateGPUBuffer(dev->device, &bc);
    if (!m->vtx || !m->idx) goto fail;

    /* Upload via a one-shot transfer */
    SDL_GPUTransferBufferCreateInfo tc = {0};
    tc.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    tc.size  = vtx_size + idx_size;
    SDL_GPUTransferBuffer *xfer = SDL_CreateGPUTransferBuffer(dev->device, &tc);
    if (!xfer) goto fail;

    void *mapped = SDL_MapGPUTransferBuffer(dev->device, xfer, false);
    if (!mapped) {
        SDL_ReleaseGPUTransferBuffer(dev->device, xfer);
        goto fail;
    }
    memcpy(mapped, vtx_data, vtx_size);
    memcpy((char *)mapped + vtx_size, idx_data, idx_size);
    SDL_UnmapGPUTransferBuffer(dev->device, xfer);

    SDL_GPUCommandBuffer *cmd = SDL_AcquireGPUCommandBuffer(dev->device);
    SDL_GPUCopyPass *copy = SDL_BeginGPUCopyPass(cmd);
    {
        SDL_GPUTransferBufferLocation src = {0};
        src.transfer_buffer = xfer;
        src.offset          = 0;
        SDL_GPUBufferRegion dst = {0};
        dst.buffer = m->vtx;
        dst.size   = vtx_size;
        SDL_UploadToGPUBuffer(copy, &src, &dst, false);
    }
    {
        SDL_GPUTransferBufferLocation src = {0};
        src.transfer_buffer = xfer;
        src.offset          = vtx_size;
        SDL_GPUBufferRegion dst = {0};
        dst.buffer = m->idx;
        dst.size   = idx_size;
        SDL_UploadToGPUBuffer(copy, &src, &dst, false);
    }
    SDL_EndGPUCopyPass(copy);
    SDL_GPUFence *fence = SDL_SubmitGPUCommandBufferAndAcquireFence(cmd);
    SDL_WaitForGPUFences(dev->device, true, &fence, 1);
    SDL_ReleaseGPUFence(dev->device, fence);
    SDL_ReleaseGPUTransferBuffer(dev->device, xfer);
    return m;

fail:
    ivp_gpu_mesh_destroy(dev, m);
    return NULL;
}

void ivp_gpu_mesh_destroy(ivp_gpu_device_t *dev, ivp_gpu_mesh_t *mesh)
{
    if (!mesh) return;
    if (mesh->vtx) SDL_ReleaseGPUBuffer(dev->device, mesh->vtx);
    if (mesh->idx) SDL_ReleaseGPUBuffer(dev->device, mesh->idx);
    free(mesh);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Shadow map
 * ═══════════════════════════════════════════════════════════════════════════ */

ivp_gpu_shadow_map_t *ivp_gpu_shadow_map_create(ivp_gpu_device_t *dev, int size)
{
    ivp_gpu_shadow_map_t *map = (ivp_gpu_shadow_map_t *)calloc(1, sizeof(*map));
    if (!map) return NULL;

    SDL_GPUTextureCreateInfo ci = {0};
    ci.type                 = SDL_GPU_TEXTURETYPE_2D;
    ci.format               = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    ci.width                = (Uint32)size;
    ci.height               = (Uint32)size;
    ci.layer_count_or_depth = 1;
    ci.num_levels           = 1;
    ci.sample_count         = SDL_GPU_SAMPLECOUNT_1;
    ci.usage                = SDL_GPU_TEXTUREUSAGE_DEPTH_STENCIL_TARGET |
                              SDL_GPU_TEXTUREUSAGE_SAMPLER;
    map->tex = SDL_CreateGPUTexture(dev->device, &ci);
    if (!map->tex) {
        fprintf(stderr, "shadow map texture failed: %s\n", SDL_GetError());
        free(map);
        return NULL;
    }

    SDL_GPUSamplerCreateInfo si = {0};
    si.min_filter     = SDL_GPU_FILTER_LINEAR;     /* hardware 2x2 PCF */
    si.mag_filter     = SDL_GPU_FILTER_LINEAR;
    si.mipmap_mode    = SDL_GPU_SAMPLERMIPMAPMODE_NEAREST;
    si.address_mode_u = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    si.address_mode_v = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    si.address_mode_w = SDL_GPU_SAMPLERADDRESSMODE_CLAMP_TO_EDGE;
    si.enable_compare = true;
    si.compare_op     = SDL_GPU_COMPAREOP_LESS_OR_EQUAL;
    map->smp = SDL_CreateGPUSampler(dev->device, &si);
    if (!map->smp) {
        fprintf(stderr, "shadow sampler failed: %s\n", SDL_GetError());
        SDL_ReleaseGPUTexture(dev->device, map->tex);
        free(map);
        return NULL;
    }
    return map;
}

void ivp_gpu_shadow_map_destroy(ivp_gpu_device_t *dev, ivp_gpu_shadow_map_t *map)
{
    if (!map) return;
    SDL_ReleaseGPUSampler(dev->device, map->smp);
    SDL_ReleaseGPUTexture(dev->device, map->tex);
    free(map);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Passes and draws
 * ═══════════════════════════════════════════════════════════════════════════ */

ivp_gpu_pass_t *ivp_gpu_shadow_pass_begin(ivp_gpu_device_t *dev,
                                          ivp_gpu_shadow_map_t *map)
{
    SDL_GPUDepthStencilTargetInfo dt = {0};
    dt.texture          = map->tex;
    dt.clear_depth      = 1.0f;
    dt.load_op          = SDL_GPU_LOADOP_CLEAR;
    dt.store_op         = SDL_GPU_STOREOP_STORE;
    dt.stencil_load_op  = SDL_GPU_LOADOP_DONT_CARE;
    dt.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;

    SDL_GPURenderPass *rp = SDL_BeginGPURenderPass(dev->cmd, NULL, 0, &dt);
    if (!rp) return NULL;
    dev->pass.dev = dev;
    dev->pass.rp  = rp;
    return &dev->pass;
}

ivp_gpu_pass_t *ivp_gpu_main_pass_begin(ivp_gpu_device_t *dev,
                                        const float clear[4])
{
    SDL_GPUColorTargetInfo ct = {0};
    SDL_GPUDepthStencilTargetInfo dt = {0};

    /* A capture frame renders into the device's own texture instead. */
    SDL_GPUTexture *out_tex = dev->capturing ? dev->capture_tex : dev->swapchain_tex;
    SDL_FColor clear_color = { clear[0], clear[1], clear[2], clear[3] };

    if (dev->msaa_enabled) {
        /* Render to the MSAA colour target, resolve into the output */
        ct.texture         = dev->msaa_color;
        ct.resolve_texture = out_tex;
        ct.load_op         = SDL_GPU_LOADOP_CLEAR;
        ct.store_op        = SDL_GPU_STOREOP_RESOLVE;
        ct.clear_color     = clear_color;
        dt.texture         = dev->msaa_depth;
    } else {
        ct.texture     = out_tex;
        ct.load_op     = SDL_GPU_LOADOP_CLEAR;
        ct.store_op    = SDL_GPU_STOREOP_STORE;
        ct.clear_color = clear_color;
        dt.texture     = dev->depth_texture;
    }

    dt.clear_depth      = 1.0f;
    dt.load_op          = SDL_GPU_LOADOP_CLEAR;
    dt.store_op         = SDL_GPU_STOREOP_DONT_CARE;
    dt.stencil_load_op  = SDL_GPU_LOADOP_DONT_CARE;
    dt.stencil_store_op = SDL_GPU_STOREOP_DONT_CARE;

    SDL_GPURenderPass *rp = SDL_BeginGPURenderPass(dev->cmd, &ct, 1, &dt);
    if (!rp) return NULL;

    SDL_GPUViewport vp = { 0, 0, (float)dev->frame_w, (float)dev->frame_h, 0.0f, 1.0f };
    SDL_SetGPUViewport(rp, &vp);

    dev->pass.dev = dev;
    dev->pass.rp  = rp;
    return &dev->pass;
}

void ivp_gpu_pass_end(ivp_gpu_pass_t *pass)
{
    SDL_EndGPURenderPass(pass->rp);
    pass->rp = NULL;
}

void ivp_gpu_bind_pipeline(ivp_gpu_pass_t *pass, ivp_gpu_pipeline_t *pipe)
{
    SDL_BindGPUGraphicsPipeline(pass->rp, pipe->pipe);
}

void ivp_gpu_push_uniforms(ivp_gpu_pass_t *pass, const void *data, size_t size)
{
    SDL_PushGPUVertexUniformData(pass->dev->cmd, 0, data, (Uint32)size);
}

void ivp_gpu_bind_shadow_map(ivp_gpu_pass_t *pass, ivp_gpu_shadow_map_t *map)
{
    SDL_GPUTextureSamplerBinding sb = { map->tex, map->smp };
    SDL_BindGPUFragmentSamplers(pass->rp, 0, &sb, 1);
}

void ivp_gpu_draw(ivp_gpu_pass_t *pass, const ivp_gpu_buffer_t *buf,
                  int vertex_count)
{
    SDL_GPUBufferBinding binding = {0};
    binding.buffer = buf->gpu[pass->dev->frame_ring];
    SDL_BindGPUVertexBuffers(pass->rp, 0, &binding, 1);
    SDL_DrawGPUPrimitives(pass->rp, (Uint32)vertex_count, 1, 0, 0);
}

void ivp_gpu_draw_mesh_instanced(ivp_gpu_pass_t *pass, const ivp_gpu_mesh_t *mesh,
                                 const ivp_gpu_buffer_t *instances,
                                 int instance_count)
{
    if (!mesh->vtx || !mesh->idx) return;

    SDL_GPUBufferBinding bindings[2] = {{0}};
    bindings[0].buffer = mesh->vtx;
    bindings[1].buffer = instances->gpu[pass->dev->frame_ring];
    SDL_BindGPUVertexBuffers(pass->rp, 0, bindings, 2);

    SDL_GPUBufferBinding ib = {0};
    ib.buffer = mesh->idx;
    SDL_BindGPUIndexBuffer(pass->rp, &ib, SDL_GPU_INDEXELEMENTSIZE_16BIT);

    SDL_DrawGPUIndexedPrimitives(pass->rp,
        (Uint32)mesh->index_count, (Uint32)instance_count, 0, 0, 0);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Screenshot
 * ═══════════════════════════════════════════════════════════════════════════ */

bool ivp_gpu_capture_begin(ivp_gpu_device_t *dev)
{
    int pw, ph;
    SDL_GetWindowSizeInPixels(dev->window, &pw, &ph);
    if (dev->capture_tex && (dev->capture_w != pw || dev->capture_h != ph)) {
        SDL_ReleaseGPUTexture(dev->device, dev->capture_tex);
        dev->capture_tex = NULL;
    }
    if (!dev->capture_tex) {
        SDL_GPUTextureCreateInfo ci = {0};
        ci.type                 = SDL_GPU_TEXTURETYPE_2D;
        ci.format               = dev->swapchain_format;
        ci.width                = (Uint32)pw;
        ci.height               = (Uint32)ph;
        ci.layer_count_or_depth = 1;
        ci.num_levels           = 1;
        ci.sample_count         = SDL_GPU_SAMPLECOUNT_1;
        ci.usage                = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET |
                                  SDL_GPU_TEXTUREUSAGE_SAMPLER;
        dev->capture_tex = SDL_CreateGPUTexture(dev->device, &ci);
        if (!dev->capture_tex) {
            fprintf(stderr, "capture texture failed: %s\n", SDL_GetError());
            return false;
        }
        dev->capture_w = pw;
        dev->capture_h = ph;
    }
    free(dev->capture_rgb);
    dev->capture_rgb = NULL;
    dev->capturing = true;
    return true;
}

/* Blit the capture texture to the swapchain and download it. */
static void capture_record(ivp_gpu_device_t *dev)
{
    SDL_GPUBlitInfo blit = {0};
    blit.source.texture       = dev->capture_tex;
    blit.source.w             = (Uint32)dev->capture_w;
    blit.source.h             = (Uint32)dev->capture_h;
    blit.destination.texture  = dev->swapchain_tex;
    blit.destination.w        = (Uint32)dev->capture_w;
    blit.destination.h        = (Uint32)dev->capture_h;
    blit.load_op              = SDL_GPU_LOADOP_DONT_CARE;
    blit.filter               = SDL_GPU_FILTER_NEAREST;
    SDL_BlitGPUTexture(dev->cmd, &blit);

    SDL_GPUTransferBufferCreateInfo tci = {0};
    tci.usage = SDL_GPU_TRANSFERBUFFERUSAGE_DOWNLOAD;
    tci.size  = (Uint32)(dev->capture_w * dev->capture_h * 4);
    dev->capture_tb = SDL_CreateGPUTransferBuffer(dev->device, &tci);
    if (!dev->capture_tb) {
        fprintf(stderr, "capture buffer failed: %s\n", SDL_GetError());
        return;
    }
    SDL_GPUCopyPass *cp = SDL_BeginGPUCopyPass(dev->cmd);
    SDL_GPUTextureRegion src = {0};
    src.texture = dev->capture_tex;
    src.w = (Uint32)dev->capture_w;
    src.h = (Uint32)dev->capture_h;
    src.d = 1;
    SDL_GPUTextureTransferInfo dst = {0};
    dst.transfer_buffer = dev->capture_tb;
    SDL_DownloadFromGPUTexture(cp, &src, &dst);
    SDL_EndGPUCopyPass(cp);
}

/* Wait for the frame, then convert the download to RGB. */
static void capture_finish(ivp_gpu_device_t *dev, SDL_GPUFence *fence)
{
    if (!dev->capture_tb) return;
    if (fence && SDL_WaitForGPUFences(dev->device, true, &fence, 1)) {
        const unsigned char *px = (const unsigned char *)
            SDL_MapGPUTransferBuffer(dev->device, dev->capture_tb, false);
        size_t n = (size_t)dev->capture_w * (size_t)dev->capture_h;
        unsigned char *rgb = px ? (unsigned char *)malloc(n * 3) : NULL;
        if (rgb) {
            bool bgr = dev->swapchain_format == SDL_GPU_TEXTUREFORMAT_B8G8R8A8_UNORM ||
                       dev->swapchain_format == SDL_GPU_TEXTUREFORMAT_B8G8R8A8_UNORM_SRGB;
            for (size_t i = 0; i < n; i++) {
                rgb[3*i + 0] = px[4*i + (bgr ? 2 : 0)];
                rgb[3*i + 1] = px[4*i + 1];
                rgb[3*i + 2] = px[4*i + (bgr ? 0 : 2)];
            }
            dev->capture_rgb = rgb;
        }
        if (px) SDL_UnmapGPUTransferBuffer(dev->device, dev->capture_tb);
    }
    SDL_ReleaseGPUTransferBuffer(dev->device, dev->capture_tb);
    dev->capture_tb = NULL;
}

unsigned char *ivp_gpu_capture_take(ivp_gpu_device_t *dev, int *width, int *height)
{
    unsigned char *rgb = dev->capture_rgb;
    dev->capture_rgb = NULL;
    dev->capturing = false;
    *width  = dev->capture_w;
    *height = dev->capture_h;
    return rgb;
}
