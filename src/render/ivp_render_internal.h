/* ivp_render_internal.h -- Internal render module interfaces */
#ifndef IVP_RENDER_INTERNAL_H
#define IVP_RENDER_INTERNAL_H

#include <ivp/render/ivp_render_draw.h>
#include <ivp/render/ivp_render_hud.h>
#include "ivp_render_gpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── Internal draw-context lifecycle (called only from core.c) ─────────── */

ivp_result_t ivp_render_draw_ctx_create_ex(ivp_render_draw_ctx_t **out,
                                           const ivp_render_draw_config_t *config,
                                           ivp_gpu_device_t *gpu);
void ivp_render_draw_ctx_destroy(ivp_render_draw_ctx_t *ctx,
                                 ivp_gpu_device_t *gpu);
void ivp_render_draw_ctx_upload(ivp_render_draw_ctx_t *ctx,
                                ivp_gpu_device_t *gpu,
                                SDL_GPUCommandBuffer *cmd);
void ivp_render_draw_ctx_draw(ivp_render_draw_ctx_t *ctx,
                              ivp_gpu_device_t *gpu,
                              SDL_GPUCommandBuffer *cmd,
                              SDL_GPURenderPass *pass,
                              const float vp_mat[16]);

/* ── Internal HUD-context lifecycle (called only from core.c) ──────────── */

ivp_result_t ivp_render_hud_ctx_create_ex(ivp_render_hud_ctx_t **out,
                                          const ivp_render_hud_config_t *config,
                                          ivp_gpu_device_t *gpu);
void ivp_render_hud_ctx_destroy(ivp_render_hud_ctx_t *ctx,
                                ivp_gpu_device_t *gpu);
void ivp_render_hud_ctx_upload(ivp_render_hud_ctx_t *ctx,
                               ivp_gpu_device_t *gpu,
                               SDL_GPUCommandBuffer *cmd);
void ivp_render_hud_ctx_draw(ivp_render_hud_ctx_t *ctx,
                             ivp_gpu_device_t *gpu,
                             SDL_GPUCommandBuffer *cmd,
                             SDL_GPURenderPass *pass,
                             int width, int height);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_INTERNAL_H */
