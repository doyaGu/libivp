/* ivp_render_gpu.h -- Internal: SDL3 GPU device, pipeline, stream management
 *
 * This header is INTERNAL to the render module. It is not part of the public
 * API and should not be included by code outside src/render/.
 *
 * Key abstractions:
 *   ivp_gpu_stream_t       -- triple-buffered vertex stream (pipeline+bufs)
 *   ivp_gpu_instance_buf_t -- triple-buffered instance data for batched draw
 *   ivp_gpu_msaa_t         -- MSAA intermediate targets (color + depth)
 *   ivp_gpu_proto_mesh_t   -- static prototype mesh (vertex+index GPU bufs)
 */
#ifndef IVP_RENDER_GPU_H
#define IVP_RENDER_GPU_H

#include <ivp/base/ivp_result.h>
#include <SDL3/SDL.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ─────────────────────────────────────────────────────────── */

#define IVP_GPU_RING_FRAMES 3   /* triple-buffered cycling */

/* ── GPU device ────────────────────────────────────────────────────────── */

typedef struct ivp_gpu_device {
    SDL_GPUDevice          *device;
    SDL_Window             *window;
    SDL_GPUShaderFormat     shader_format;
    SDL_GPUTextureFormat    swapchain_format;
    SDL_GPUTexture         *depth_texture;
    int                     depth_w, depth_h;
    SDL_GPUSampleCount      sample_count;
} ivp_gpu_device_t;

ivp_result_t ivp_gpu_device_init(ivp_gpu_device_t *dev, SDL_Window *window,
                                 int msaa_samples);
void         ivp_gpu_device_destroy(ivp_gpu_device_t *dev);
bool         ivp_gpu_device_ensure_depth(ivp_gpu_device_t *dev, int w, int h);

/* ── MSAA targets ──────────────────────────────────────────────────────── */

typedef struct ivp_gpu_msaa {
    SDL_GPUTexture *color_tex;   /* multisampled color render target */
    SDL_GPUTexture *depth_tex;   /* multisampled depth render target */
    int             width, height;
    SDL_GPUSampleCount sample_count;
    bool            enabled;
} ivp_gpu_msaa_t;

bool ivp_gpu_msaa_ensure(ivp_gpu_msaa_t *msaa, ivp_gpu_device_t *dev,
                         int w, int h);
void ivp_gpu_msaa_destroy(ivp_gpu_msaa_t *msaa, ivp_gpu_device_t *dev);

/* ── Vertex attribute descriptor ───────────────────────────────────────── */

typedef struct ivp_gpu_attrib_desc {
    int location;
    int components;  /* 2, 3, or 4 */
    int offset;      /* byte offset within vertex struct */
} ivp_gpu_attrib_desc_t;

/* ── GPU vertex stream (triple-buffered) ───────────────────────────────── */
/*
 * A stream bundles: pipeline + triple-buffered GPU/transfer buffers + CPU staging.
 * Triple-buffering eliminates per-frame upload stalls by cycling through
 * buffer slots and using fences to ensure the GPU is done before reuse.
 */

typedef struct ivp_gpu_stream {
    /* Pipeline */
    SDL_GPUGraphicsPipeline *pipeline;
    /* Triple-buffered GPU + transfer */
    SDL_GPUBuffer           *gpu_buf[IVP_GPU_RING_FRAMES];
    SDL_GPUTransferBuffer   *transfer_buf[IVP_GPU_RING_FRAMES];
    SDL_GPUFence            *fence[IVP_GPU_RING_FRAMES];
    int                      ring_slot;     /* current slot [0,2] */
    /* CPU staging */
    void                    *vertices;
    int                      vertex_count;
    int                      capacity;      /* in vertices */
    int                      stride;        /* bytes per vertex */
} ivp_gpu_stream_t;

/* Pipeline creation descriptor (consumed once during stream_create). */
typedef struct ivp_gpu_stream_desc {
    int                         capacity;
    int                         stride;
    const ivp_gpu_attrib_desc_t *attribs;
    int                         attrib_count;
    bool                        depth_test;
    bool                        depth_write;
    bool                        depth_always;     /* keep the depth target, never reject (overlays) */
    SDL_GPUPrimitiveType        primitive_type;   /* explicitly set by caller */
    SDL_GPUCullMode             cull_mode;        /* 0 = NONE (default) */
    int                         num_uniform_bufs; /* 0 = auto (1 for MSL) */
    int                         fs_num_samplers;  /* fragment shader texture samplers */
    bool                        blend;            /* straight alpha blending */
    bool                        depth_only;       /* no colour target, 1 sample, depth bias (shadow pass) */
    /* Optional: second vertex buffer for instancing (slot 1) */
    const ivp_gpu_attrib_desc_t *inst_attribs;
    int                         inst_attrib_count;
    int                         inst_stride;
} ivp_gpu_stream_desc_t;

/* Create a fully-initialized stream: pipeline + triple-buffered bufs. */
ivp_result_t ivp_gpu_stream_create(ivp_gpu_stream_t *out,
                                   ivp_gpu_device_t *dev,
                                   const ivp_gpu_stream_desc_t *desc,
                                   const unsigned char *vs_code, size_t vs_len,
                                   const unsigned char *fs_code, size_t fs_len);

void ivp_gpu_stream_destroy(ivp_gpu_stream_t *s, ivp_gpu_device_t *dev);

/* Reset vertex count to zero (call at start of frame). */
static inline void ivp_gpu_stream_reset(ivp_gpu_stream_t *s)
{
    s->vertex_count = 0;
}

/* Push raw vertex data into the CPU staging area.
 * Returns pointer to the pushed region, or NULL if out of capacity. */
void *ivp_gpu_stream_push(ivp_gpu_stream_t *s, int count);

/* Upload CPU staging → GPU via copy pass (triple-buffered: waits fence). */
void ivp_gpu_stream_upload(ivp_gpu_stream_t *s,
                           ivp_gpu_device_t *dev,
                           SDL_GPUCommandBuffer *cmd);

/* Bind pipeline, bind vertex buffer, push uniform, draw.
 * Call inside an active render pass. `uniform_data` and `uniform_size`
 * specify the push-constant payload (typically 64 or 160 bytes). */
void ivp_gpu_stream_draw(ivp_gpu_stream_t *s,
                         SDL_GPURenderPass *pass,
                         SDL_GPUCommandBuffer *cmd,
                         const void *uniform_data,
                         Uint32 uniform_size);

/* Advance ring slot after frame submit (call after SubmitGPUCommandBuffer). */
void ivp_gpu_stream_advance(ivp_gpu_stream_t *s, SDL_GPUFence *frame_fence);

/* ── Instance buffer (triple-buffered) ─────────────────────────────────── */

typedef struct ivp_gpu_instance_buf {
    SDL_GPUBuffer          *gpu_buf[IVP_GPU_RING_FRAMES];
    SDL_GPUTransferBuffer  *transfer_buf[IVP_GPU_RING_FRAMES];
    SDL_GPUFence           *fence[IVP_GPU_RING_FRAMES];
    int                     ring_slot;
    void                   *data;       /* CPU staging */
    int                     count;      /* current instance count */
    int                     capacity;   /* max instances */
    int                     stride;     /* bytes per instance */
} ivp_gpu_instance_buf_t;

ivp_result_t ivp_gpu_instance_buf_create(ivp_gpu_instance_buf_t *out,
                                         ivp_gpu_device_t *dev,
                                         int capacity, int stride);
void ivp_gpu_instance_buf_destroy(ivp_gpu_instance_buf_t *b,
                                  ivp_gpu_device_t *dev);

static inline void ivp_gpu_instance_buf_reset(ivp_gpu_instance_buf_t *b)
{
    b->count = 0;
}

void *ivp_gpu_instance_buf_push(ivp_gpu_instance_buf_t *b, int count);
void  ivp_gpu_instance_buf_upload(ivp_gpu_instance_buf_t *b,
                                  ivp_gpu_device_t *dev,
                                  SDL_GPUCommandBuffer *cmd);
void  ivp_gpu_instance_buf_advance(ivp_gpu_instance_buf_t *b,
                                   SDL_GPUFence *frame_fence);

/* ── Prototype mesh (static GPU vertex+index buffers) ──────────────────── */

typedef struct ivp_gpu_proto_mesh {
    SDL_GPUBuffer *vtx_buf;
    SDL_GPUBuffer *idx_buf;
    int            vertex_count;
    int            index_count;
    int            vtx_stride;
} ivp_gpu_proto_mesh_t;

ivp_result_t ivp_gpu_proto_mesh_upload(ivp_gpu_proto_mesh_t *out,
                                       ivp_gpu_device_t *dev,
                                       const void *vtx_data, int vtx_count,
                                       int vtx_stride,
                                       const Uint16 *idx_data, int idx_count);
void ivp_gpu_proto_mesh_destroy(ivp_gpu_proto_mesh_t *m, ivp_gpu_device_t *dev);

/* Instanced draw: prototype mesh + instance buffer.
 * Pipeline must already be bound. */
void ivp_gpu_proto_mesh_draw_instanced(const ivp_gpu_proto_mesh_t *mesh,
                                       const ivp_gpu_instance_buf_t *inst,
                                       SDL_GPURenderPass *pass);

/* ── Pipeline creation (internal) ──────────────────────────────────────── */

/* Create a standalone graphics pipeline from a stream descriptor.
 * Used for instanced pipelines that don't have their own stream. */
SDL_GPUGraphicsPipeline *ivp_gpu_create_pipeline(
    ivp_gpu_device_t *dev,
    const ivp_gpu_stream_desc_t *desc,
    const unsigned char *vs_code, size_t vs_len,
    const unsigned char *fs_code, size_t fs_len);

/* ── Shadow map (depth texture + comparison sampler) ──────────────────── */

typedef struct ivp_gpu_shadow_map {
    SDL_GPUTexture *tex;
    SDL_GPUSampler *smp;
    int             size;     /* square, texels */
} ivp_gpu_shadow_map_t;

bool ivp_gpu_shadow_map_create(ivp_gpu_shadow_map_t *map, ivp_gpu_device_t *dev, int size);
void ivp_gpu_shadow_map_destroy(ivp_gpu_shadow_map_t *map, ivp_gpu_device_t *dev);

/* ── Shader blob loading ───────────────────────────────────────────────── */

/* Shader blob cache (one per renderer, not global). */
typedef struct ivp_shader_cache {
    unsigned char *vs_3d;       size_t vs_3d_len;
    unsigned char *fs;          size_t fs_len;
    unsigned char *vs_2d;       size_t vs_2d_len;
    unsigned char *vs_tri3d;    size_t vs_tri3d_len;
    unsigned char *fs_tri3d;    size_t fs_tri3d_len;
    unsigned char *vs_inst;     size_t vs_inst_len;
    unsigned char *fs_inst;     size_t fs_inst_len;
    unsigned char *vs_shadow;   size_t vs_shadow_len;
    unsigned char *fs_shadow;   size_t fs_shadow_len;
    unsigned char *vs_rect;     size_t vs_rect_len;
    unsigned char *fs_rect;     size_t fs_rect_len;
} ivp_shader_cache_t;

ivp_result_t ivp_shader_cache_load(ivp_shader_cache_t *cache,
                                   SDL_GPUShaderFormat format);
void         ivp_shader_cache_free(ivp_shader_cache_t *cache);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_GPU_H */
