/* ivp_render_gpu.c -- SDL3 GPU device, triple-buffered streams, MSAA,
 *                     instance buffers, proto meshes, and shader management
 *
 * All GPU resource management lives here.  Key abstractions:
 *   ivp_gpu_stream_t       - triple-buffered vertex stream (pipeline+bufs)
 *   ivp_gpu_instance_buf_t - triple-buffered per-instance data
 *   ivp_gpu_proto_mesh_t   - static vertex+index GPU meshes
 *   ivp_gpu_msaa_t         - MSAA intermediate colour/depth targets
 */

#include "ivp_render_gpu.h"
#include <ivp/base/ivp_allocator.h>

#include <stdio.h>
#include <string.h>

#ifndef IVP_RENDER_SHADER_DIR_FALLBACK
#define IVP_RENDER_SHADER_DIR_FALLBACK "shaders"
#endif

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

    /* 4. CWD/shaders */
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

ivp_result_t ivp_shader_cache_load(ivp_shader_cache_t *cache,
                                   SDL_GPUShaderFormat format)
{
    memset(cache, 0, sizeof(*cache));

    const char *ext = (format == SDL_GPU_SHADERFORMAT_MSL) ? "msl" : "spv";
    char name[64];
    bool ok = true;

    SDL_snprintf(name, sizeof(name), "line3d.vert.%s", ext);
    ok = ok && load_shader_file(&cache->vs_3d, &cache->vs_3d_len, name);

    SDL_snprintf(name, sizeof(name), "line.frag.%s", ext);
    ok = ok && load_shader_file(&cache->fs, &cache->fs_len, name);

    SDL_snprintf(name, sizeof(name), "line2d.vert.%s", ext);
    ok = ok && load_shader_file(&cache->vs_2d, &cache->vs_2d_len, name);

    SDL_snprintf(name, sizeof(name), "tri3d.vert.%s", ext);
    ok = ok && load_shader_file(&cache->vs_tri3d, &cache->vs_tri3d_len, name);

    SDL_snprintf(name, sizeof(name), "tri3d.frag.%s", ext);
    ok = ok && load_shader_file(&cache->fs_tri3d, &cache->fs_tri3d_len, name);

    SDL_snprintf(name, sizeof(name), "tri3d_inst.vert.%s", ext);
    ok = ok && load_shader_file(&cache->vs_inst, &cache->vs_inst_len, name);

    SDL_snprintf(name, sizeof(name), "tri3d_inst.frag.%s", ext);
    ok = ok && load_shader_file(&cache->fs_inst, &cache->fs_inst_len, name);

    SDL_snprintf(name, sizeof(name), "shadow.vert.%s", ext);
    ok = ok && load_shader_file(&cache->vs_shadow, &cache->vs_shadow_len, name);

    SDL_snprintf(name, sizeof(name), "shadow.frag.%s", ext);
    ok = ok && load_shader_file(&cache->fs_shadow, &cache->fs_shadow_len, name);

    if (!ok) {
        ivp_shader_cache_free(cache);
        return IVP_ERR_RENDER_SHADER;
    }
    return IVP_OK;
}

void ivp_shader_cache_free(ivp_shader_cache_t *cache)
{
    if (cache->vs_3d)    { SDL_free(cache->vs_3d);    cache->vs_3d    = NULL; }
    if (cache->fs)       { SDL_free(cache->fs);       cache->fs       = NULL; }
    if (cache->vs_2d)    { SDL_free(cache->vs_2d);    cache->vs_2d    = NULL; }
    if (cache->vs_tri3d) { SDL_free(cache->vs_tri3d); cache->vs_tri3d = NULL; }
    if (cache->fs_tri3d) { SDL_free(cache->fs_tri3d); cache->fs_tri3d = NULL; }
    if (cache->vs_inst)  { SDL_free(cache->vs_inst);  cache->vs_inst  = NULL; }
    if (cache->fs_inst)  { SDL_free(cache->fs_inst);  cache->fs_inst  = NULL; }
    if (cache->vs_shadow) { SDL_free(cache->vs_shadow); cache->vs_shadow = NULL; }
    if (cache->fs_shadow) { SDL_free(cache->fs_shadow); cache->fs_shadow = NULL; }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Depth texture helper
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

/* ═══════════════════════════════════════════════════════════════════════════
 * GPU device lifecycle
 * ═══════════════════════════════════════════════════════════════════════════ */

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

ivp_result_t ivp_gpu_device_init(ivp_gpu_device_t *dev, SDL_Window *window,
                                 int msaa_samples)
{
    memset(dev, 0, sizeof(*dev));
    dev->window = window;

    dev->device = SDL_CreateGPUDevice(
        SDL_GPU_SHADERFORMAT_SPIRV | SDL_GPU_SHADERFORMAT_MSL,
        false, NULL);
    if (!dev->device) {
        fprintf(stderr, "SDL_CreateGPUDevice failed: %s\n", SDL_GetError());
        return IVP_ERR_RENDER_GPU_DEVICE;
    }

    if (!SDL_ClaimWindowForGPUDevice(dev->device, window)) {
        fprintf(stderr, "SDL_ClaimWindowForGPUDevice failed: %s\n", SDL_GetError());
        SDL_DestroyGPUDevice(dev->device);
        dev->device = NULL;
        return IVP_ERR_RENDER_GPU_DEVICE;
    }

    SDL_GPUShaderFormat supported = SDL_GetGPUShaderFormats(dev->device);
    if (!select_shader_format(dev, supported)) {
        fprintf(stderr, "Unsupported shader format (mask=0x%x)\n",
                (unsigned)supported);
        SDL_ReleaseWindowFromGPUDevice(dev->device, window);
        SDL_DestroyGPUDevice(dev->device);
        dev->device = NULL;
        return IVP_ERR_RENDER_GPU_DEVICE;
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
        SDL_ReleaseWindowFromGPUDevice(dev->device, window);
        SDL_DestroyGPUDevice(dev->device);
        dev->device = NULL;
        return IVP_ERR_RENDER_GPU_DEVICE;
    }
    dev->depth_w = w;
    dev->depth_h = h;
    return IVP_OK;
}

void ivp_gpu_device_destroy(ivp_gpu_device_t *dev)
{
    if (!dev || !dev->device) return;
    if (dev->depth_texture) {
        SDL_ReleaseGPUTexture(dev->device, dev->depth_texture);
        dev->depth_texture = NULL;
    }
    SDL_ReleaseWindowFromGPUDevice(dev->device, dev->window);
    SDL_DestroyGPUDevice(dev->device);
    dev->device = NULL;
}

bool ivp_gpu_device_ensure_depth(ivp_gpu_device_t *dev, int w, int h)
{
    if (w == dev->depth_w && h == dev->depth_h) return false;

    SDL_GPUTexture *tex = create_depth_texture(dev->device, w, h, dev->sample_count);
    if (!tex) {
        fprintf(stderr, "Depth resize failed (%dx%d): %s\n", w, h, SDL_GetError());
        return false;
    }
    if (dev->depth_texture)
        SDL_ReleaseGPUTexture(dev->device, dev->depth_texture);
    dev->depth_texture = tex;
    dev->depth_w = w;
    dev->depth_h = h;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * MSAA targets
 * ═══════════════════════════════════════════════════════════════════════════ */

bool ivp_gpu_msaa_ensure(ivp_gpu_msaa_t *msaa, ivp_gpu_device_t *dev,
                         int w, int h)
{
    SDL_GPUSampleCount sc = dev->sample_count;
    if (sc <= SDL_GPU_SAMPLECOUNT_1) {
        msaa->enabled = false;
        return false;
    }

    if (msaa->color_tex && msaa->width == w && msaa->height == h)
        return false;  /* already correct size */

    /* Destroy old */
    ivp_gpu_msaa_destroy(msaa, dev);

    /* Create MSAA color target */
    SDL_GPUTextureCreateInfo ci = {0};
    ci.type                 = SDL_GPU_TEXTURETYPE_2D;
    ci.format               = dev->swapchain_format;
    ci.width                = (Uint32)w;
    ci.height               = (Uint32)h;
    ci.layer_count_or_depth = 1;
    ci.num_levels           = 1;
    ci.sample_count         = sc;
    ci.usage                = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET;
    msaa->color_tex = SDL_CreateGPUTexture(dev->device, &ci);
    if (!msaa->color_tex) {
        fprintf(stderr, "MSAA color texture failed: %s\n", SDL_GetError());
        return false;
    }

    /* Create MSAA depth target */
    msaa->depth_tex = create_depth_texture(dev->device, w, h, sc);
    if (!msaa->depth_tex) {
        fprintf(stderr, "MSAA depth texture failed: %s\n", SDL_GetError());
        SDL_ReleaseGPUTexture(dev->device, msaa->color_tex);
        msaa->color_tex = NULL;
        return false;
    }

    msaa->width = w;
    msaa->height = h;
    msaa->sample_count = sc;
    msaa->enabled = true;
    return true;
}

void ivp_gpu_msaa_destroy(ivp_gpu_msaa_t *msaa, ivp_gpu_device_t *dev)
{
    if (!msaa || !dev || !dev->device) return;
    if (msaa->color_tex) {
        SDL_ReleaseGPUTexture(dev->device, msaa->color_tex);
        msaa->color_tex = NULL;
    }
    if (msaa->depth_tex) {
        SDL_ReleaseGPUTexture(dev->device, msaa->depth_tex);
        msaa->depth_tex = NULL;
    }
    msaa->enabled = false;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Shadow map
 * ═══════════════════════════════════════════════════════════════════════════ */

bool ivp_gpu_shadow_map_create(ivp_gpu_shadow_map_t *map, ivp_gpu_device_t *dev, int size)
{
    memset(map, 0, sizeof(*map));

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
        return false;
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
        map->tex = NULL;
        return false;
    }
    map->size = size;
    return true;
}

void ivp_gpu_shadow_map_destroy(ivp_gpu_shadow_map_t *map, ivp_gpu_device_t *dev)
{
    if (map->smp) SDL_ReleaseGPUSampler(dev->device, map->smp);
    if (map->tex) SDL_ReleaseGPUTexture(dev->device, map->tex);
    memset(map, 0, sizeof(*map));
}

/* ═══════════════════════════════════════════════════════════════════════════
 * GPU stream (triple-buffered)
 * ═══════════════════════════════════════════════════════════════════════════ */

static const char *shader_format_name(SDL_GPUShaderFormat fmt)
{
    if (fmt == SDL_GPU_SHADERFORMAT_SPIRV) return "SPIR-V";
    if (fmt == SDL_GPU_SHADERFORMAT_MSL)   return "MSL";
    return "UNKNOWN";
}

SDL_GPUGraphicsPipeline *ivp_gpu_create_pipeline(
    ivp_gpu_device_t *dev,
    const ivp_gpu_stream_desc_t *desc,
    const unsigned char *vs_code, size_t vs_len,
    const unsigned char *fs_code, size_t fs_len)
{
    bool is_msl = (dev->shader_format == SDL_GPU_SHADERFORMAT_MSL);
    int num_ub = desc->num_uniform_bufs;
    if (num_ub <= 0) num_ub = is_msl ? 1 : 0;

    SDL_GPUShaderCreateInfo vs_ci = {0};
    vs_ci.code                = vs_code;
    vs_ci.code_size           = vs_len;
    vs_ci.entrypoint          = is_msl ? "vs_main" : "main";
    vs_ci.format              = dev->shader_format;
    vs_ci.stage               = SDL_GPU_SHADERSTAGE_VERTEX;
    vs_ci.num_uniform_buffers = (Uint32)num_ub;

    SDL_GPUShader *vs = SDL_CreateGPUShader(dev->device, &vs_ci);
    if (!vs) {
        fprintf(stderr, "VS creation failed (%s): %s\n",
                shader_format_name(dev->shader_format), SDL_GetError());
        return NULL;
    }

    SDL_GPUShaderCreateInfo fs_ci = {0};
    fs_ci.code        = fs_code;
    fs_ci.code_size   = fs_len;
    fs_ci.entrypoint  = is_msl ? "fs_main" : "main";
    fs_ci.format      = dev->shader_format;
    fs_ci.stage       = SDL_GPU_SHADERSTAGE_FRAGMENT;
    fs_ci.num_samplers = (Uint32)desc->fs_num_samplers;

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
        attrs[total_attrs].format =
            (desc->attribs[i].components == 2) ? SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2 :
            (desc->attribs[i].components == 4) ? SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4 :
                                                  SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
        attrs[total_attrs].offset = (Uint32)desc->attribs[i].offset;
        total_attrs++;
    }

    if (desc->inst_attribs) {
        int ni = (desc->inst_attrib_count < 8) ? desc->inst_attrib_count : 8;
        for (int i = 0; i < ni; i++) {
            attrs[total_attrs].location    = (Uint32)desc->inst_attribs[i].location;
            attrs[total_attrs].buffer_slot = 1;
            attrs[total_attrs].format =
                (desc->inst_attribs[i].components == 2) ? SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2 :
                (desc->inst_attribs[i].components == 4) ? SDL_GPU_VERTEXELEMENTFORMAT_FLOAT4 :
                                                           SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
            attrs[total_attrs].offset = (Uint32)desc->inst_attribs[i].offset;
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

    SDL_GPUPrimitiveType prim = desc->primitive_type;

    SDL_GPUCullMode cull = desc->cull_mode;

    SDL_GPUGraphicsPipelineCreateInfo gp = {0};
    gp.vertex_shader       = vs;
    gp.fragment_shader     = fs;
    gp.vertex_input_state  = vis;
    gp.primitive_type      = prim;
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
    gp.rasterizer_state.cull_mode            = cull;
    gp.multisample_state.sample_count        = desc->depth_only ? SDL_GPU_SAMPLECOUNT_1
                                                                : dev->sample_count;

    SDL_GPUGraphicsPipeline *pipe = SDL_CreateGPUGraphicsPipeline(dev->device, &gp);

    SDL_ReleaseGPUShader(dev->device, vs);
    SDL_ReleaseGPUShader(dev->device, fs);

    if (!pipe)
        fprintf(stderr, "Pipeline creation failed: %s\n", SDL_GetError());
    return pipe;
}

ivp_result_t ivp_gpu_stream_create(ivp_gpu_stream_t *out,
                                   ivp_gpu_device_t *dev,
                                   const ivp_gpu_stream_desc_t *desc,
                                   const unsigned char *vs_code, size_t vs_len,
                                   const unsigned char *fs_code, size_t fs_len)
{
    memset(out, 0, sizeof(*out));

    if (!vs_code || vs_len == 0 || !fs_code || fs_len == 0) {
        fprintf(stderr, "Shader code not available for %s\n",
                shader_format_name(dev->shader_format));
        return IVP_ERR_RENDER_SHADER;
    }

    out->pipeline = ivp_gpu_create_pipeline(dev, desc, vs_code, vs_len, fs_code, fs_len);
    if (!out->pipeline) return IVP_ERR_RENDER_SHADER;

    out->capacity = desc->capacity;
    out->stride   = desc->stride;

    Uint32 buf_size = (Uint32)desc->capacity * (Uint32)desc->stride;

    /* Triple-buffered GPU + transfer buffers */
    for (int i = 0; i < IVP_GPU_RING_FRAMES; i++) {
        SDL_GPUBufferCreateInfo buf_ci = {0};
        buf_ci.usage = SDL_GPU_BUFFERUSAGE_VERTEX;
        buf_ci.size  = buf_size;
        out->gpu_buf[i] = SDL_CreateGPUBuffer(dev->device, &buf_ci);
        if (!out->gpu_buf[i]) goto fail;

        SDL_GPUTransferBufferCreateInfo tb_ci = {0};
        tb_ci.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
        tb_ci.size  = buf_size;
        out->transfer_buf[i] = SDL_CreateGPUTransferBuffer(dev->device, &tb_ci);
        if (!out->transfer_buf[i]) goto fail;

        out->fence[i] = NULL;  /* no fence until first submit */
    }
    out->ring_slot = 0;

    {
        const ivp_allocator_t *alloc = ivp_default_allocator();
        out->vertices = ivp_alloc(alloc, (size_t)buf_size);
        if (!out->vertices) goto fail;
    }

    out->vertex_count = 0;
    return IVP_OK;

fail:
    ivp_gpu_stream_destroy(out, dev);
    return IVP_ERR_OUT_OF_MEMORY;
}

void ivp_gpu_stream_destroy(ivp_gpu_stream_t *s, ivp_gpu_device_t *dev)
{
    if (!s) return;
    if (s->vertices) {
        const ivp_allocator_t *alloc = ivp_default_allocator();
        ivp_free(alloc, s->vertices, (size_t)s->capacity * (size_t)s->stride);
        s->vertices = NULL;
    }
    if (dev && dev->device) {
        /* Wait for any in-flight fences before destroying buffers */
        for (int i = 0; i < IVP_GPU_RING_FRAMES; i++) {
            if (s->fence[i]) {
                SDL_WaitForGPUFences(dev->device, true, &s->fence[i], 1);
                SDL_ReleaseGPUFence(dev->device, s->fence[i]);
                s->fence[i] = NULL;
            }
        }
        for (int i = 0; i < IVP_GPU_RING_FRAMES; i++) {
            if (s->transfer_buf[i]) {
                SDL_ReleaseGPUTransferBuffer(dev->device, s->transfer_buf[i]);
                s->transfer_buf[i] = NULL;
            }
            if (s->gpu_buf[i]) {
                SDL_ReleaseGPUBuffer(dev->device, s->gpu_buf[i]);
                s->gpu_buf[i] = NULL;
            }
        }
        if (s->pipeline) {
            SDL_ReleaseGPUGraphicsPipeline(dev->device, s->pipeline);
            s->pipeline = NULL;
        }
    }
}

void *ivp_gpu_stream_push(ivp_gpu_stream_t *s, int count)
{
    if (s->vertex_count + count > s->capacity) return NULL;
    char *ptr = (char *)s->vertices + (size_t)s->vertex_count * (size_t)s->stride;
    s->vertex_count += count;
    return ptr;
}

void ivp_gpu_stream_upload(ivp_gpu_stream_t *s,
                           ivp_gpu_device_t *dev,
                           SDL_GPUCommandBuffer *cmd)
{
    if (s->vertex_count <= 0) return;

    int slot = s->ring_slot;
    Uint32 data_size = (Uint32)s->vertex_count * (Uint32)s->stride;

    /* Frame-level fence wait (in begin_frame) ensures this slot is safe. */

    void *mapped = SDL_MapGPUTransferBuffer(dev->device, s->transfer_buf[slot], true);
    if (!mapped) return;
    memcpy(mapped, s->vertices, data_size);
    SDL_UnmapGPUTransferBuffer(dev->device, s->transfer_buf[slot]);

    SDL_GPUCopyPass *copy = SDL_BeginGPUCopyPass(cmd);
    SDL_GPUTransferBufferLocation src = {0};
    src.transfer_buffer = s->transfer_buf[slot];
    SDL_GPUBufferRegion dst = {0};
    dst.buffer = s->gpu_buf[slot];
    dst.size   = data_size;
    SDL_UploadToGPUBuffer(copy, &src, &dst, false);
    SDL_EndGPUCopyPass(copy);
}

void ivp_gpu_stream_draw(ivp_gpu_stream_t *s,
                         SDL_GPURenderPass *pass,
                         SDL_GPUCommandBuffer *cmd,
                         const void *uniform_data,
                         Uint32 uniform_size)
{
    if (s->vertex_count <= 0) return;

    int slot = s->ring_slot;

    SDL_BindGPUGraphicsPipeline(pass, s->pipeline);
    SDL_GPUBufferBinding binding = {0};
    binding.buffer = s->gpu_buf[slot];
    SDL_BindGPUVertexBuffers(pass, 0, &binding, 1);
    SDL_PushGPUVertexUniformData(cmd, 0, uniform_data, uniform_size);
    SDL_DrawGPUPrimitives(pass, (Uint32)s->vertex_count, 1, 0, 0);
}

void ivp_gpu_stream_advance(ivp_gpu_stream_t *s, SDL_GPUFence *frame_fence)
{
    int slot = s->ring_slot;
    (void)frame_fence;  /* fence managed at renderer level */
    s->ring_slot = (slot + 1) % IVP_GPU_RING_FRAMES;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Instance buffer (triple-buffered)
 * ═══════════════════════════════════════════════════════════════════════════ */

ivp_result_t ivp_gpu_instance_buf_create(ivp_gpu_instance_buf_t *out,
                                         ivp_gpu_device_t *dev,
                                         int capacity, int stride)
{
    memset(out, 0, sizeof(*out));
    out->capacity = capacity;
    out->stride   = stride;
    Uint32 buf_size = (Uint32)capacity * (Uint32)stride;

    for (int i = 0; i < IVP_GPU_RING_FRAMES; i++) {
        SDL_GPUBufferCreateInfo buf_ci = {0};
        buf_ci.usage = SDL_GPU_BUFFERUSAGE_VERTEX;
        buf_ci.size  = buf_size;
        out->gpu_buf[i] = SDL_CreateGPUBuffer(dev->device, &buf_ci);
        if (!out->gpu_buf[i]) goto fail;

        SDL_GPUTransferBufferCreateInfo tb_ci = {0};
        tb_ci.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
        tb_ci.size  = buf_size;
        out->transfer_buf[i] = SDL_CreateGPUTransferBuffer(dev->device, &tb_ci);
        if (!out->transfer_buf[i]) goto fail;

        out->fence[i] = NULL;
    }
    out->ring_slot = 0;

    {
        const ivp_allocator_t *alloc = ivp_default_allocator();
        out->data = ivp_alloc(alloc, (size_t)buf_size);
        if (!out->data) goto fail;
    }
    out->count = 0;
    return IVP_OK;

fail:
    ivp_gpu_instance_buf_destroy(out, dev);
    return IVP_ERR_OUT_OF_MEMORY;
}

void ivp_gpu_instance_buf_destroy(ivp_gpu_instance_buf_t *b,
                                  ivp_gpu_device_t *dev)
{
    if (!b) return;
    if (b->data) {
        const ivp_allocator_t *alloc = ivp_default_allocator();
        ivp_free(alloc, b->data, (size_t)b->capacity * (size_t)b->stride);
        b->data = NULL;
    }
    if (dev && dev->device) {
        for (int i = 0; i < IVP_GPU_RING_FRAMES; i++) {
            if (b->fence[i]) {
                SDL_WaitForGPUFences(dev->device, true, &b->fence[i], 1);
                SDL_ReleaseGPUFence(dev->device, b->fence[i]);
                b->fence[i] = NULL;
            }
        }
        for (int i = 0; i < IVP_GPU_RING_FRAMES; i++) {
            if (b->transfer_buf[i]) {
                SDL_ReleaseGPUTransferBuffer(dev->device, b->transfer_buf[i]);
                b->transfer_buf[i] = NULL;
            }
            if (b->gpu_buf[i]) {
                SDL_ReleaseGPUBuffer(dev->device, b->gpu_buf[i]);
                b->gpu_buf[i] = NULL;
            }
        }
    }
}

void *ivp_gpu_instance_buf_push(ivp_gpu_instance_buf_t *b, int count)
{
    if (b->count + count > b->capacity) return NULL;
    char *ptr = (char *)b->data + (size_t)b->count * (size_t)b->stride;
    b->count += count;
    return ptr;
}

void ivp_gpu_instance_buf_upload(ivp_gpu_instance_buf_t *b,
                                 ivp_gpu_device_t *dev,
                                 SDL_GPUCommandBuffer *cmd)
{
    if (b->count <= 0) return;

    int slot = b->ring_slot;
    Uint32 data_size = (Uint32)b->count * (Uint32)b->stride;

    /* Frame-level fence wait (in begin_frame) ensures this slot is safe. */

    void *mapped = SDL_MapGPUTransferBuffer(dev->device, b->transfer_buf[slot], true);
    if (!mapped) return;
    memcpy(mapped, b->data, data_size);
    SDL_UnmapGPUTransferBuffer(dev->device, b->transfer_buf[slot]);

    SDL_GPUCopyPass *copy = SDL_BeginGPUCopyPass(cmd);
    SDL_GPUTransferBufferLocation src = {0};
    src.transfer_buffer = b->transfer_buf[slot];
    SDL_GPUBufferRegion dst = {0};
    dst.buffer = b->gpu_buf[slot];
    dst.size   = data_size;
    SDL_UploadToGPUBuffer(copy, &src, &dst, false);
    SDL_EndGPUCopyPass(copy);
}

void ivp_gpu_instance_buf_advance(ivp_gpu_instance_buf_t *b,
                                  SDL_GPUFence *frame_fence)
{
    (void)frame_fence;  /* fence managed at renderer level */
    b->ring_slot = (b->ring_slot + 1) % IVP_GPU_RING_FRAMES;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * Prototype mesh (static GPU vertex + index buffers)
 * ═══════════════════════════════════════════════════════════════════════════ */

ivp_result_t ivp_gpu_proto_mesh_upload(ivp_gpu_proto_mesh_t *out,
                                       ivp_gpu_device_t *dev,
                                       const void *vtx_data, int vtx_count,
                                       int vtx_stride,
                                       const Uint16 *idx_data, int idx_count)
{
    memset(out, 0, sizeof(*out));
    out->vertex_count = vtx_count;
    out->index_count  = idx_count;
    out->vtx_stride   = vtx_stride;

    Uint32 vtx_size = (Uint32)vtx_count * (Uint32)vtx_stride;
    Uint32 idx_size = (Uint32)idx_count * (Uint32)sizeof(Uint16);

    /* Vertex buffer */
    {
        SDL_GPUBufferCreateInfo bc = {0};
        bc.usage = SDL_GPU_BUFFERUSAGE_VERTEX;
        bc.size  = vtx_size;
        out->vtx_buf = SDL_CreateGPUBuffer(dev->device, &bc);
        if (!out->vtx_buf) return IVP_ERR_OUT_OF_MEMORY;
    }

    /* Index buffer */
    {
        SDL_GPUBufferCreateInfo bc = {0};
        bc.usage = SDL_GPU_BUFFERUSAGE_INDEX;
        bc.size  = idx_size;
        out->idx_buf = SDL_CreateGPUBuffer(dev->device, &bc);
        if (!out->idx_buf) {
            SDL_ReleaseGPUBuffer(dev->device, out->vtx_buf);
            out->vtx_buf = NULL;
            return IVP_ERR_OUT_OF_MEMORY;
        }
    }

    /* Upload via one-shot transfer */
    SDL_GPUTransferBufferCreateInfo tc = {0};
    tc.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    tc.size  = vtx_size + idx_size;
    SDL_GPUTransferBuffer *xfer = SDL_CreateGPUTransferBuffer(dev->device, &tc);
    if (!xfer) {
        ivp_gpu_proto_mesh_destroy(out, dev);
        return IVP_ERR_OUT_OF_MEMORY;
    }

    void *mapped = SDL_MapGPUTransferBuffer(dev->device, xfer, false);
    if (!mapped) {
        SDL_ReleaseGPUTransferBuffer(dev->device, xfer);
        ivp_gpu_proto_mesh_destroy(out, dev);
        return IVP_ERR_OUT_OF_MEMORY;
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
        dst.buffer = out->vtx_buf;
        dst.size   = vtx_size;
        SDL_UploadToGPUBuffer(copy, &src, &dst, false);
    }
    {
        SDL_GPUTransferBufferLocation src = {0};
        src.transfer_buffer = xfer;
        src.offset          = vtx_size;
        SDL_GPUBufferRegion dst = {0};
        dst.buffer = out->idx_buf;
        dst.size   = idx_size;
        SDL_UploadToGPUBuffer(copy, &src, &dst, false);
    }

    SDL_EndGPUCopyPass(copy);
    SDL_GPUFence *fence = SDL_SubmitGPUCommandBufferAndAcquireFence(cmd);
    SDL_WaitForGPUFences(dev->device, true, &fence, 1);
    SDL_ReleaseGPUFence(dev->device, fence);
    SDL_ReleaseGPUTransferBuffer(dev->device, xfer);
    return IVP_OK;
}

void ivp_gpu_proto_mesh_destroy(ivp_gpu_proto_mesh_t *m, ivp_gpu_device_t *dev)
{
    if (!m || !dev || !dev->device) return;
    if (m->vtx_buf) { SDL_ReleaseGPUBuffer(dev->device, m->vtx_buf); m->vtx_buf = NULL; }
    if (m->idx_buf) { SDL_ReleaseGPUBuffer(dev->device, m->idx_buf); m->idx_buf = NULL; }
}

void ivp_gpu_proto_mesh_draw_instanced(const ivp_gpu_proto_mesh_t *mesh,
                                       const ivp_gpu_instance_buf_t *inst,
                                       SDL_GPURenderPass *pass)
{
    if (!mesh->vtx_buf || !mesh->idx_buf || inst->count <= 0) return;

    int slot = inst->ring_slot;

    SDL_GPUBufferBinding bindings[2] = {{0}};
    bindings[0].buffer = mesh->vtx_buf;
    bindings[1].buffer = inst->gpu_buf[slot];
    SDL_BindGPUVertexBuffers(pass, 0, bindings, 2);

    SDL_GPUBufferBinding ib = {0};
    ib.buffer = mesh->idx_buf;
    SDL_BindGPUIndexBuffer(pass, &ib, SDL_GPU_INDEXELEMENTSIZE_16BIT);

    SDL_DrawGPUIndexedPrimitives(pass,
        (Uint32)mesh->index_count, (Uint32)inst->count, 0, 0, 0);
}