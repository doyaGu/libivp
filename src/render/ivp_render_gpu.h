/* ivp_render_gpu.h -- Internal: SDL3 GPU pipeline and buffer management
 *
 * This header is INTERNAL to the render module. It is not part of the public
 * API and should not be included by code outside src/render/.
 */
#ifndef IVP_RENDER_GPU_H
#define IVP_RENDER_GPU_H

#include <ivp/render/ivp_render_types.h>
#include <SDL3/SDL.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── GPU device handle ─────────────────────────────────────────────────── */

typedef struct ivp_gpu_device {
    SDL_GPUDevice          *device;
    SDL_Window             *window;
    SDL_GPUShaderFormat     shader_format;
    SDL_GPUTextureFormat    swapchain_format;
    SDL_GPUTexture         *depth_texture;
    int                     depth_w, depth_h;
    SDL_GPUSampleCount      sample_count;
} ivp_gpu_device_t;

/* ── Pipeline handle ───────────────────────────────────────────────────── */

typedef struct ivp_gpu_pipeline {
    SDL_GPUGraphicsPipeline *pipeline;
} ivp_gpu_pipeline_t;

/* ── Transfer / vertex buffer handle ───────────────────────────────────── */

typedef struct ivp_gpu_buffer {
    SDL_GPUBuffer          *gpu_buf;
    SDL_GPUTransferBuffer  *transfer_buf;
    int                     capacity;  /* in vertices */
    int                     stride;    /* bytes per vertex */
} ivp_gpu_buffer_t;

/* ── Embedded shader code (SPIR-V / MSL) ───────────────────────────────── */

/* These are set during pipeline creation from embedded SPIRV or runtime
 * compilation via SDL_ShaderCross.  No raw GLSL source strings are used. */

/* ── Device lifecycle ──────────────────────────────────────────────────── */

ivp_result_t ivp_gpu_device_init(ivp_gpu_device_t *dev, SDL_Window *window,
                                 int msaa_samples);
void         ivp_gpu_device_destroy(ivp_gpu_device_t *dev);

/* Ensure depth texture matches current window size. Returns true if recreated. */
bool ivp_gpu_device_ensure_depth(ivp_gpu_device_t *dev, int w, int h);

/* ── Pipeline creation ─────────────────────────────────────────────────── */

/* Vertex attribute descriptor for pipeline creation */
typedef struct ivp_gpu_attrib_desc {
    int         location;
    int         components;  /* 2 or 3 */
    int         offset;      /* byte offset within vertex struct */
} ivp_gpu_attrib_desc_t;

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
                                     bool depth_write);

void ivp_gpu_pipeline_destroy(ivp_gpu_pipeline_t *pipe, ivp_gpu_device_t *dev);

/* ── Buffer management ─────────────────────────────────────────────────── */

ivp_result_t ivp_gpu_buffer_create(ivp_gpu_buffer_t *out,
                                   ivp_gpu_device_t *dev,
                                   int capacity,
                                   int stride);
void         ivp_gpu_buffer_destroy(ivp_gpu_buffer_t *buf, ivp_gpu_device_t *dev);

/* Upload CPU vertex data to the GPU via a copy pass.
 * Must be called BEFORE the render pass begins on the same command buffer. */
void ivp_gpu_buffer_upload(ivp_gpu_buffer_t *buf,
                           ivp_gpu_device_t *dev,
                           SDL_GPUCommandBuffer *cmd,
                           const void *data,
                           int vertex_count);

/* Bind pipeline, bind vertex buffer, push matrix, and draw lines.
 * Must be called INSIDE an active render pass. */
void ivp_gpu_buffer_draw(ivp_gpu_buffer_t *buf,
                         SDL_GPURenderPass *pass,
                         SDL_GPUCommandBuffer *cmd,
                         const ivp_gpu_pipeline_t *pipe,
                         const float *matrix,
                         int vertex_count);

/* ── Shader code accessors ─────────────────────────────────────────────── */

const unsigned char *ivp_gpu_vs_3d_code(const ivp_gpu_device_t *dev, size_t *len);
const unsigned char *ivp_gpu_fs_3d_code(const ivp_gpu_device_t *dev, size_t *len);
const unsigned char *ivp_gpu_vs_2d_code(const ivp_gpu_device_t *dev, size_t *len);
const unsigned char *ivp_gpu_fs_2d_code(const ivp_gpu_device_t *dev, size_t *len);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_GPU_H */
