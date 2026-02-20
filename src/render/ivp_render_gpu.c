/* ivp_render_gpu.c -- SDL3 GPU device, pipeline and buffer management */

#include "ivp_render_gpu.h"

#include <stdio.h>
#include <string.h>

#ifndef IVP_RENDER_SHADER_DIR_FALLBACK
#define IVP_RENDER_SHADER_DIR_FALLBACK "shaders"
#endif

typedef struct shader_blob {
    unsigned char *bytes;
    size_t         size;
} shader_blob_t;

static shader_blob_t g_spv_vs_3d = {0};
static shader_blob_t g_spv_fs    = {0};
static shader_blob_t g_spv_vs_2d = {0};
static shader_blob_t g_msl_vs_3d = {0};
static shader_blob_t g_msl_fs    = {0};
static shader_blob_t g_msl_vs_2d = {0};

static const char *shader_format_name(SDL_GPUShaderFormat format)
{
    if (format == SDL_GPU_SHADERFORMAT_SPIRV) return "SPIR-V";
    if (format == SDL_GPU_SHADERFORMAT_MSL) return "MSL";
    if (format == SDL_GPU_SHADERFORMAT_DXIL) return "DXIL";
    if (format == SDL_GPU_SHADERFORMAT_METALLIB) return "METALLIB";
    return "UNKNOWN";
}

static bool load_shader_blob(shader_blob_t *blob, const char *file_name)
{
    if (blob->bytes) return true;

    const char *env_dir = SDL_getenv("IVP_RENDER_SHADER_DIR");
    char path[1024];

    if (env_dir && env_dir[0]) {
        SDL_snprintf(path, sizeof(path), "%s/%s", env_dir, file_name);
        {
            size_t size = 0;
            void *data = SDL_LoadFile(path, &size);
            if (data && size > 0) {
                blob->bytes = (unsigned char *)data;
                blob->size  = size;
                return true;
            }
            if (data) SDL_free(data);
        }
    }

    {
        const char *base = SDL_GetBasePath();
        if (base) {
            SDL_snprintf(path, sizeof(path), "%sshaders/%s", base, file_name);
            {
                size_t size = 0;
                void *data = SDL_LoadFile(path, &size);
                if (data && size > 0) {
                    blob->bytes = (unsigned char *)data;
                    blob->size  = size;
                    return true;
                }
                if (data) SDL_free(data);
            }
        }
    }

    SDL_snprintf(path, sizeof(path), "%s/%s", IVP_RENDER_SHADER_DIR_FALLBACK, file_name);
    {
        size_t size = 0;
        void *data = SDL_LoadFile(path, &size);
        if (data && size > 0) {
            blob->bytes = (unsigned char *)data;
            blob->size  = size;
            return true;
        }
        if (data) SDL_free(data);
    }

    SDL_snprintf(path, sizeof(path), "shaders/%s", file_name);
    {
        size_t size = 0;
        void *data = SDL_LoadFile(path, &size);
        if (data && size > 0) {
            blob->bytes = (unsigned char *)data;
            blob->size  = size;
            return true;
        }
        if (data) SDL_free(data);
    }

    fprintf(stderr, "Failed to load shader '%s' from env/base/fallback/cwd paths\n", file_name);
    return false;
}

static const shader_blob_t *get_vs_3d_blob(SDL_GPUShaderFormat format)
{
    if (format == SDL_GPU_SHADERFORMAT_MSL) {
        if (!load_shader_blob(&g_msl_vs_3d, "line3d.vert.msl")) return NULL;
        return &g_msl_vs_3d;
    }

    if (!load_shader_blob(&g_spv_vs_3d, "line3d.vert.spv")) return NULL;
    return &g_spv_vs_3d;
}

static const shader_blob_t *get_fs_blob(SDL_GPUShaderFormat format)
{
    if (format == SDL_GPU_SHADERFORMAT_MSL) {
        if (!load_shader_blob(&g_msl_fs, "line.frag.msl")) return NULL;
        return &g_msl_fs;
    }

    if (!load_shader_blob(&g_spv_fs, "line.frag.spv")) return NULL;
    return &g_spv_fs;
}

static const shader_blob_t *get_vs_2d_blob(SDL_GPUShaderFormat format)
{
    if (format == SDL_GPU_SHADERFORMAT_MSL) {
        if (!load_shader_blob(&g_msl_vs_2d, "line2d.vert.msl")) return NULL;
        return &g_msl_vs_2d;
    }

    if (!load_shader_blob(&g_spv_vs_2d, "line2d.vert.spv")) return NULL;
    return &g_spv_vs_2d;
}

/* ── Depth texture helper ──────────────────────────────────────────────── */

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

static void destroy_gpu_device_claim(ivp_gpu_device_t *dev)
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

    {
        const char *driver = SDL_GetGPUDeviceDriver(dev->device);
        if (driver && !SDL_strcasecmp(driver, "metal") &&
            dev->shader_format != SDL_GPU_SHADERFORMAT_MSL) {
            return false;
        }
    }

    return true;
}

/* ── Device lifecycle ──────────────────────────────────────────────────── */

ivp_result_t ivp_gpu_device_init(ivp_gpu_device_t *dev, SDL_Window *window,
                                 int msaa_samples)
{
    memset(dev, 0, sizeof(*dev));
    dev->window = window;

    dev->device = SDL_CreateGPUDevice(
        SDL_GPU_SHADERFORMAT_SPIRV | SDL_GPU_SHADERFORMAT_MSL,
        false,
        NULL
    );
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

    {
        SDL_GPUShaderFormat supported = SDL_GetGPUShaderFormats(dev->device);
        if (!select_shader_format(dev, supported)) {
            fprintf(stderr, "Unsupported shader format for backend '%s' (supported mask=0x%x)\n",
                    SDL_GetGPUDeviceDriver(dev->device), (unsigned int)supported);
            destroy_gpu_device_claim(dev);
            return IVP_ERR_RENDER_GPU_DEVICE;
        }
    }

    dev->swapchain_format = SDL_GetGPUSwapchainTextureFormat(dev->device, window);
    dev->sample_count = (msaa_samples >= 4) ? SDL_GPU_SAMPLECOUNT_4
                      : (msaa_samples >= 2) ? SDL_GPU_SAMPLECOUNT_2
                      :                       SDL_GPU_SAMPLECOUNT_1;

    int w, h;
    SDL_GetWindowSizeInPixels(window, &w, &h);
    dev->depth_texture = create_depth_texture(dev->device, w, h, dev->sample_count);
    if (!dev->depth_texture) {
        fprintf(stderr, "Failed to create depth texture: %s\n", SDL_GetError());
        destroy_gpu_device_claim(dev);
        return IVP_ERR_RENDER_GPU_DEVICE;
    }
    dev->depth_w = w;
    dev->depth_h = h;

    return IVP_OK;
}

void ivp_gpu_device_destroy(ivp_gpu_device_t *dev)
{
    destroy_gpu_device_claim(dev);
}

bool ivp_gpu_device_ensure_depth(ivp_gpu_device_t *dev, int w, int h)
{
    if (w == dev->depth_w && h == dev->depth_h) return false;

    SDL_GPUTexture *new_depth = create_depth_texture(dev->device, w, h, dev->sample_count);
    if (!new_depth) {
        fprintf(stderr, "Depth texture resize failed (%dx%d): %s\n", w, h, SDL_GetError());
        return false;
    }

    if (dev->depth_texture) {
        SDL_ReleaseGPUTexture(dev->device, dev->depth_texture);
    }

    dev->depth_texture = new_depth;
    dev->depth_w = w;
    dev->depth_h = h;
    return true;
}

/* ── Pipeline creation ─────────────────────────────────────────────────── */

ivp_result_t ivp_gpu_pipeline_create(ivp_gpu_pipeline_t *out,
                                     ivp_gpu_device_t *dev,
                                     const unsigned char *vs_spirv,
                                     size_t vs_spirv_len,
                                     const unsigned char *fs_spirv,
                                     size_t fs_spirv_len,
                                     int stride,
                                     const ivp_gpu_attrib_desc_t *attribs,
                                     int attrib_count,
                                     bool depth_test,
                                     bool depth_write)
{
    memset(out, 0, sizeof(*out));

    if (!vs_spirv || vs_spirv_len == 0 || !fs_spirv || fs_spirv_len == 0) {
        fprintf(stderr, "Shader code not available for format %s\n",
                shader_format_name(dev->shader_format));
        return IVP_ERR_RENDER_SHADER;
    }

    SDL_GPUShaderCreateInfo vs_ci = {0};
    vs_ci.code                = vs_spirv;
    vs_ci.code_size           = vs_spirv_len;
    vs_ci.entrypoint          = (dev->shader_format == SDL_GPU_SHADERFORMAT_MSL) ? "vs_main" : "main";
    vs_ci.format              = dev->shader_format;
    vs_ci.stage               = SDL_GPU_SHADERSTAGE_VERTEX;
    vs_ci.num_uniform_buffers = (dev->shader_format == SDL_GPU_SHADERFORMAT_MSL) ? 1 : 0;
    vs_ci.num_storage_buffers = 0;
    vs_ci.num_storage_textures = 0;
    vs_ci.num_samplers        = 0;

    SDL_GPUShader *vs = SDL_CreateGPUShader(dev->device, &vs_ci);
    if (!vs) {
        fprintf(stderr, "VS shader creation failed (%s): %s\n",
                shader_format_name(dev->shader_format), SDL_GetError());
        return IVP_ERR_RENDER_SHADER;
    }

    SDL_GPUShaderCreateInfo fs_ci = {0};
    fs_ci.code                = fs_spirv;
    fs_ci.code_size           = fs_spirv_len;
    fs_ci.entrypoint          = (dev->shader_format == SDL_GPU_SHADERFORMAT_MSL) ? "fs_main" : "main";
    fs_ci.format              = dev->shader_format;
    fs_ci.stage               = SDL_GPU_SHADERSTAGE_FRAGMENT;
    fs_ci.num_uniform_buffers = 0;
    fs_ci.num_storage_buffers = 0;
    fs_ci.num_storage_textures = 0;
    fs_ci.num_samplers        = 0;

    SDL_GPUShader *fs = SDL_CreateGPUShader(dev->device, &fs_ci);
    if (!fs) {
        fprintf(stderr, "FS shader creation failed (%s): %s\n",
                shader_format_name(dev->shader_format), SDL_GetError());
        SDL_ReleaseGPUShader(dev->device, vs);
        return IVP_ERR_RENDER_SHADER;
    }

    SDL_GPUVertexBufferDescription vbuf_desc = {0};
    vbuf_desc.slot               = 0;
    vbuf_desc.pitch              = (Uint32)stride;
    vbuf_desc.input_rate         = SDL_GPU_VERTEXINPUTRATE_VERTEX;
    vbuf_desc.instance_step_rate = 0;

    SDL_GPUVertexAttribute vert_attrs[4] = {0};
    for (int i = 0; i < attrib_count && i < 4; i++) {
        vert_attrs[i].location    = (Uint32)attribs[i].location;
        vert_attrs[i].buffer_slot = 0;
        vert_attrs[i].format      = (attribs[i].components == 2)
                                  ? SDL_GPU_VERTEXELEMENTFORMAT_FLOAT2
                                  : SDL_GPU_VERTEXELEMENTFORMAT_FLOAT3;
        vert_attrs[i].offset      = (Uint32)attribs[i].offset;
    }

    SDL_GPUVertexInputState vis = {0};
    vis.vertex_buffer_descriptions = &vbuf_desc;
    vis.num_vertex_buffers         = 1;
    vis.vertex_attributes          = vert_attrs;
    vis.num_vertex_attributes      = (Uint32)attrib_count;

    SDL_GPUDepthStencilState dss = {0};
    dss.compare_op          = SDL_GPU_COMPAREOP_LESS;
    dss.enable_depth_test   = depth_test;
    dss.enable_depth_write  = depth_write;
    dss.enable_stencil_test = false;

    SDL_GPUColorTargetDescription color_target = {0};
    color_target.format = dev->swapchain_format;

    SDL_GPUGraphicsPipelineCreateInfo gp_ci = {0};
    gp_ci.vertex_shader       = vs;
    gp_ci.fragment_shader     = fs;
    gp_ci.vertex_input_state  = vis;
    gp_ci.primitive_type      = SDL_GPU_PRIMITIVETYPE_LINELIST;
    gp_ci.depth_stencil_state = dss;
    gp_ci.target_info.color_target_descriptions = &color_target;
    gp_ci.target_info.num_color_targets         = 1;
    gp_ci.target_info.has_depth_stencil_target  = depth_test || depth_write;
    gp_ci.target_info.depth_stencil_format      = SDL_GPU_TEXTUREFORMAT_D32_FLOAT;
    gp_ci.rasterizer_state.fill_mode            = SDL_GPU_FILLMODE_FILL;
    gp_ci.rasterizer_state.cull_mode            = SDL_GPU_CULLMODE_NONE;
    gp_ci.multisample_state.sample_count        = dev->sample_count;

    out->pipeline = SDL_CreateGPUGraphicsPipeline(dev->device, &gp_ci);

    SDL_ReleaseGPUShader(dev->device, vs);
    SDL_ReleaseGPUShader(dev->device, fs);

    if (!out->pipeline) {
        fprintf(stderr, "Pipeline creation failed: %s\n", SDL_GetError());
        return IVP_ERR_RENDER_SHADER;
    }

    return IVP_OK;
}

void ivp_gpu_pipeline_destroy(ivp_gpu_pipeline_t *pipe, ivp_gpu_device_t *dev)
{
    if (pipe->pipeline) {
        SDL_ReleaseGPUGraphicsPipeline(dev->device, pipe->pipeline);
        pipe->pipeline = NULL;
    }
}

/* ── Buffer management ─────────────────────────────────────────────────── */

ivp_result_t ivp_gpu_buffer_create(ivp_gpu_buffer_t *out,
                                   ivp_gpu_device_t *dev,
                                   int capacity,
                                   int stride)
{
    memset(out, 0, sizeof(*out));

    Uint32 size = (Uint32)capacity * (Uint32)stride;

    SDL_GPUBufferCreateInfo buf_ci = {0};
    buf_ci.usage = SDL_GPU_BUFFERUSAGE_VERTEX;
    buf_ci.size  = size;
    out->gpu_buf = SDL_CreateGPUBuffer(dev->device, &buf_ci);
    if (!out->gpu_buf) return IVP_ERR_OUT_OF_MEMORY;

    SDL_GPUTransferBufferCreateInfo tb_ci = {0};
    tb_ci.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    tb_ci.size  = size;
    out->transfer_buf = SDL_CreateGPUTransferBuffer(dev->device, &tb_ci);
    if (!out->transfer_buf) {
        SDL_ReleaseGPUBuffer(dev->device, out->gpu_buf);
        out->gpu_buf = NULL;
        return IVP_ERR_OUT_OF_MEMORY;
    }

    out->capacity = capacity;
    out->stride   = stride;
    return IVP_OK;
}

void ivp_gpu_buffer_destroy(ivp_gpu_buffer_t *buf, ivp_gpu_device_t *dev)
{
    if (!dev || !dev->device) return;
    if (buf->transfer_buf) {
        SDL_ReleaseGPUTransferBuffer(dev->device, buf->transfer_buf);
        buf->transfer_buf = NULL;
    }
    if (buf->gpu_buf) {
        SDL_ReleaseGPUBuffer(dev->device, buf->gpu_buf);
        buf->gpu_buf = NULL;
    }
}

void ivp_gpu_buffer_upload(ivp_gpu_buffer_t *buf,
                           ivp_gpu_device_t *dev,
                           SDL_GPUCommandBuffer *cmd,
                           const void *data,
                           int vertex_count)
{
    if (vertex_count <= 0) return;

    Uint32 data_size = (Uint32)vertex_count * (Uint32)buf->stride;

    void *mapped = SDL_MapGPUTransferBuffer(dev->device, buf->transfer_buf, true);
    if (!mapped) return;

    memcpy(mapped, data, data_size);
    SDL_UnmapGPUTransferBuffer(dev->device, buf->transfer_buf);

    SDL_GPUCopyPass *copy = SDL_BeginGPUCopyPass(cmd);

    SDL_GPUTransferBufferLocation src = {0};
    src.transfer_buffer = buf->transfer_buf;
    src.offset          = 0;

    SDL_GPUBufferRegion dst = {0};
    dst.buffer = buf->gpu_buf;
    dst.offset = 0;
    dst.size   = data_size;

    SDL_UploadToGPUBuffer(copy, &src, &dst, false);
    SDL_EndGPUCopyPass(copy);
}

void ivp_gpu_buffer_draw(ivp_gpu_buffer_t *buf,
                         SDL_GPURenderPass *pass,
                         SDL_GPUCommandBuffer *cmd,
                         const ivp_gpu_pipeline_t *pipe,
                         const float *matrix,
                         int vertex_count)
{
    if (vertex_count <= 0) return;

    SDL_BindGPUGraphicsPipeline(pass, pipe->pipeline);

    SDL_GPUBufferBinding binding = {0};
    binding.buffer = buf->gpu_buf;
    binding.offset = 0;
    SDL_BindGPUVertexBuffers(pass, 0, &binding, 1);

    SDL_PushGPUVertexUniformData(cmd, 0, matrix, 64);
    SDL_DrawGPUPrimitives(pass, (Uint32)vertex_count, 1, 0, 0);
}

/* ── Shader code accessors ─────────────────────────────────────────────── */

const unsigned char *ivp_gpu_vs_3d_code(const ivp_gpu_device_t *dev, size_t *len)
{
    const shader_blob_t *blob = get_vs_3d_blob(dev->shader_format);
    if (!blob) {
        if (len) *len = 0;
        return NULL;
    }
    if (len) *len = blob->size;
    return blob->bytes;
}

const unsigned char *ivp_gpu_fs_3d_code(const ivp_gpu_device_t *dev, size_t *len)
{
    const shader_blob_t *blob = get_fs_blob(dev->shader_format);
    if (!blob) {
        if (len) *len = 0;
        return NULL;
    }
    if (len) *len = blob->size;
    return blob->bytes;
}

const unsigned char *ivp_gpu_vs_2d_code(const ivp_gpu_device_t *dev, size_t *len)
{
    const shader_blob_t *blob = get_vs_2d_blob(dev->shader_format);
    if (!blob) {
        if (len) *len = 0;
        return NULL;
    }
    if (len) *len = blob->size;
    return blob->bytes;
}

const unsigned char *ivp_gpu_fs_2d_code(const ivp_gpu_device_t *dev, size_t *len)
{
    const shader_blob_t *blob = get_fs_blob(dev->shader_format);
    if (!blob) {
        if (len) *len = 0;
        return NULL;
    }
    if (len) *len = blob->size;
    return blob->bytes;
}
