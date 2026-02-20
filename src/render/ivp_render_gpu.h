/* ivp_render_gpu.h -- Internal: SDL3 GPU device, pipeline, stream management
 *
 * This header is INTERNAL to the render module. It is not part of the public
 * API and should not be included by code outside src/render/.
 *
 * Key abstraction: ivp_gpu_stream_t -- a unified vertex stream that owns a
 * GPU pipeline, vertex buffer pair, and CPU staging area.  Replaces the
 * previously-duplicated lifecycle code in draw and hud subsystems.
 */
#ifndef IVP_RENDER_GPU_H
#define IVP_RENDER_GPU_H

#include <ivp/base/ivp_result.h>
#include <SDL3/SDL.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

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

/* ── Vertex attribute descriptor ───────────────────────────────────────── */

typedef struct ivp_gpu_attrib_desc {
    int location;
    int components;  /* 2 or 3 */
    int offset;      /* byte offset within vertex struct */
} ivp_gpu_attrib_desc_t;

/* ── GPU vertex stream ─────────────────────────────────────────────────── */
/*
 * A stream bundles: pipeline + GPU buffer + transfer buffer + CPU staging.
 * Used for both 3D wireframe and 2D HUD -- the only difference is the
 * vertex layout (stride) and pipeline configuration.
 */

typedef struct ivp_gpu_stream {
    /* Pipeline */
    SDL_GPUGraphicsPipeline *pipeline;
    /* Buffers */
    SDL_GPUBuffer           *gpu_buf;
    SDL_GPUTransferBuffer   *transfer_buf;
    /* CPU staging */
    void                    *vertices;
    int                      vertex_count;
    int                      capacity;     /* in vertices */
    int                      stride;       /* bytes per vertex */
} ivp_gpu_stream_t;

/* Pipeline creation descriptor (consumed once during stream_create). */
typedef struct ivp_gpu_stream_desc {
    int                         capacity;
    int                         stride;
    const ivp_gpu_attrib_desc_t *attribs;
    int                         attrib_count;
    bool                        depth_test;
    bool                        depth_write;
} ivp_gpu_stream_desc_t;

/* Create a fully-initialized stream: pipeline + buffers + CPU staging. */
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

/* Upload CPU staging → GPU via copy pass. Call before render pass. */
void ivp_gpu_stream_upload(ivp_gpu_stream_t *s,
                           ivp_gpu_device_t *dev,
                           SDL_GPUCommandBuffer *cmd);

/* Bind pipeline, bind vertex buffer, push uniform matrix, draw.
 * Call inside an active render pass. */
void ivp_gpu_stream_draw(ivp_gpu_stream_t *s,
                         SDL_GPURenderPass *pass,
                         SDL_GPUCommandBuffer *cmd,
                         const float *matrix);

/* ── Shader blob loading ───────────────────────────────────────────────── */

/* Shader blob cache (one per renderer, not global). */
typedef struct ivp_shader_cache {
    unsigned char *vs_3d;  size_t vs_3d_len;
    unsigned char *fs;     size_t fs_len;
    unsigned char *vs_2d;  size_t vs_2d_len;
} ivp_shader_cache_t;

ivp_result_t ivp_shader_cache_load(ivp_shader_cache_t *cache,
                                   SDL_GPUShaderFormat format);
void         ivp_shader_cache_free(ivp_shader_cache_t *cache);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_GPU_H */
