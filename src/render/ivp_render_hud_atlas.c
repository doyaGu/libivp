/* ivp_render_hud_atlas.c -- GPU font atlas for efficient HUD text rendering
 *
 * Generates a 16×6-cell bitmap font atlas texture from the 5×7 font data.
 * Renders text as textured quads via stream_2d instead of one line per pixel.
 *
 * Each glyph cell is (FONT_CELL_W × FONT_CELL_H) pixels including 1px padding.
 * The atlas is laid out as 16 columns × 6 rows = 96 cells (ASCII 32-127).
 */

#include "ivp_render_internal.h"
#include <string.h>
#include <stdio.h>

/* ── Font atlas dimensions ─────────────────────────────────────────────── */

#define FONT_CELL_W  (IVP_RENDER_HUD_FONT_W + 1)  /* 6 */
#define FONT_CELL_H  (IVP_RENDER_HUD_FONT_H + 1)  /* 8 */
#define ATLAS_COLS   16
#define ATLAS_ROWS   6
#define ATLAS_W      (ATLAS_COLS * FONT_CELL_W)    /* 96 */
#define ATLAS_H      (ATLAS_ROWS * FONT_CELL_H)    /* 48 */
#define FONT_FIRST   32
#define FONT_LAST    126

/* ── External font data (defined in ivp_render_hud.c) ──────────────────── */

extern const unsigned char ivp_hud_font_data[][IVP_RENDER_HUD_FONT_H];

/* ── Atlas texture lifecycle ───────────────────────────────────────────── */

SDL_GPUTexture *ivp_hud_atlas_create(ivp_gpu_device_t *dev)
{
    /* Build RGBA8 pixel data */
    Uint8 pixels[ATLAS_W * ATLAS_H * 4];
    memset(pixels, 0, sizeof(pixels));

    for (int ch = FONT_FIRST; ch <= FONT_LAST; ch++) {
        int ci = ch - FONT_FIRST;
        int col = ci % ATLAS_COLS;
        int row = ci / ATLAS_COLS;
        int ox = col * FONT_CELL_W;
        int oy = row * FONT_CELL_H;

        const unsigned char *g = ivp_hud_font_data[ci];
        for (int y = 0; y < IVP_RENDER_HUD_FONT_H; y++) {
            unsigned char bits = g[y];
            for (int x = 0; x < IVP_RENDER_HUD_FONT_W; x++) {
                if (bits & (1 << (IVP_RENDER_HUD_FONT_W - 1 - x))) {
                    int px = (oy + y) * ATLAS_W + (ox + x);
                    pixels[px * 4 + 0] = 255;
                    pixels[px * 4 + 1] = 255;
                    pixels[px * 4 + 2] = 255;
                    pixels[px * 4 + 3] = 255;
                }
            }
        }
    }

    /* Create GPU texture */
    SDL_GPUTextureCreateInfo ci = {0};
    ci.type                 = SDL_GPU_TEXTURETYPE_2D;
    ci.format               = SDL_GPU_TEXTUREFORMAT_R8G8B8A8_UNORM;
    ci.width                = ATLAS_W;
    ci.height               = ATLAS_H;
    ci.layer_count_or_depth = 1;
    ci.num_levels           = 1;
    ci.sample_count         = SDL_GPU_SAMPLECOUNT_1;
    ci.usage                = SDL_GPU_TEXTUREUSAGE_SAMPLER;

    SDL_GPUTexture *tex = SDL_CreateGPUTexture(dev->device, &ci);
    if (!tex) {
        fprintf(stderr, "Font atlas texture creation failed: %s\n", SDL_GetError());
        return NULL;
    }

    /* Upload via transfer buffer */
    Uint32 data_size = (Uint32)(ATLAS_W * ATLAS_H * 4);
    SDL_GPUTransferBufferCreateInfo tc = {0};
    tc.usage = SDL_GPU_TRANSFERBUFFERUSAGE_UPLOAD;
    tc.size  = data_size;
    SDL_GPUTransferBuffer *xfer = SDL_CreateGPUTransferBuffer(dev->device, &tc);
    if (!xfer) {
        SDL_ReleaseGPUTexture(dev->device, tex);
        return NULL;
    }

    void *mapped = SDL_MapGPUTransferBuffer(dev->device, xfer, false);
    if (mapped) {
        memcpy(mapped, pixels, data_size);
        SDL_UnmapGPUTransferBuffer(dev->device, xfer);

        SDL_GPUCommandBuffer *cmd = SDL_AcquireGPUCommandBuffer(dev->device);
        SDL_GPUCopyPass *copy = SDL_BeginGPUCopyPass(cmd);

        SDL_GPUTextureTransferInfo src = {0};
        src.transfer_buffer = xfer;
        src.offset          = 0;
        SDL_GPUTextureRegion dst_region = {0};
        dst_region.texture = tex;
        dst_region.w       = ATLAS_W;
        dst_region.h       = ATLAS_H;
        dst_region.d       = 1;
        SDL_UploadToGPUTexture(copy, &src, &dst_region, false);

        SDL_EndGPUCopyPass(copy);
        SDL_GPUFence *fence = SDL_SubmitGPUCommandBufferAndAcquireFence(cmd);
        SDL_WaitForGPUFences(dev->device, true, &fence, 1);
        SDL_ReleaseGPUFence(dev->device, fence);
    }

    SDL_ReleaseGPUTransferBuffer(dev->device, xfer);
    return tex;
}
