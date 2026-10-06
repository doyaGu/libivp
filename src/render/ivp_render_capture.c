/* ivp_render_capture.c -- Screenshot capture (PNG)
 *
 * SDL GPU cannot read back a swapchain texture, so the frame that is being
 * captured renders into a texture owned by the renderer; that texture is
 * blitted to the swapchain and downloaded to a transfer buffer.
 */

#include "ivp_render_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── Minimal PNG writer (stored deflate blocks, RGB8) ──────────────────── */

static uint32_t crc_table[256];

static void crc_init(void)
{
    if (crc_table[1]) return;
    for (uint32_t n = 0; n < 256; n++) {
        uint32_t c = n;
        for (int k = 0; k < 8; k++)
            c = (c & 1u) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
        crc_table[n] = c;
    }
}

static uint32_t crc_update(uint32_t crc, const unsigned char *p, size_t n)
{
    for (size_t i = 0; i < n; i++)
        crc = crc_table[(crc ^ p[i]) & 0xFF] ^ (crc >> 8);
    return crc;
}

static void put_be32(unsigned char *p, uint32_t v)
{
    p[0] = (unsigned char)(v >> 24);
    p[1] = (unsigned char)(v >> 16);
    p[2] = (unsigned char)(v >> 8);
    p[3] = (unsigned char)v;
}

static bool write_chunk(FILE *f, const char type[4],
                        const unsigned char *data, size_t len)
{
    unsigned char hdr[8];
    put_be32(hdr, (uint32_t)len);
    memcpy(hdr + 4, type, 4);
    uint32_t crc = crc_update(0xFFFFFFFFu, (const unsigned char *)type, 4);
    crc = crc_update(crc, data, len) ^ 0xFFFFFFFFu;
    unsigned char tail[4];
    put_be32(tail, crc);
    return fwrite(hdr, 1, 8, f) == 8 &&
           (len == 0 || fwrite(data, 1, len, f) == len) &&
           fwrite(tail, 1, 4, f) == 4;
}

bool ivp_png_write_rgb(const char *path, const unsigned char *rgb, int w, int h)
{
    if (!path || !rgb || w <= 0 || h <= 0) return false;
    crc_init();

    size_t row = (size_t)w * 3 + 1;                 /* filter byte + pixels */
    size_t raw = row * (size_t)h;
    size_t nblocks = (raw + 65534) / 65535;
    size_t zlen = 2 + raw + nblocks * 5 + 4;
    unsigned char *z = (unsigned char *)malloc(zlen);
    if (!z) return false;

    size_t zi = 0;
    z[zi++] = 0x78;  z[zi++] = 0x01;
    uint32_t a = 1, b = 0;
    size_t pos = 0;                                  /* offset in the raw stream */
    while (pos < raw) {
        size_t n = raw - pos > 65535 ? 65535 : raw - pos;
        z[zi++] = (pos + n == raw) ? 1 : 0;
        z[zi++] = (unsigned char)(n & 0xFF);
        z[zi++] = (unsigned char)(n >> 8);
        z[zi++] = (unsigned char)(~n & 0xFF);
        z[zi++] = (unsigned char)((~n >> 8) & 0xFF);
        for (size_t i = 0; i < n; i++, pos++) {
            size_t col = pos % row;
            unsigned char v = col == 0 ? 0 : rgb[(pos / row) * (size_t)w * 3 + (col - 1)];
            z[zi++] = v;
            a = (a + v) % 65521u;
            b = (b + a) % 65521u;
        }
    }
    put_be32(z + zi, (b << 16) | a);
    zi += 4;

    FILE *f = fopen(path, "wb");
    if (!f) { free(z); return false; }
    static const unsigned char sig[8] = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };
    unsigned char ihdr[13];
    put_be32(ihdr, (uint32_t)w);
    put_be32(ihdr + 4, (uint32_t)h);
    ihdr[8] = 8;  ihdr[9] = 2;  ihdr[10] = 0;  ihdr[11] = 0;  ihdr[12] = 0;
    bool ok = fwrite(sig, 1, 8, f) == 8 &&
              write_chunk(f, "IHDR", ihdr, 13) &&
              write_chunk(f, "IDAT", z, zi) &&
              write_chunk(f, "IEND", NULL, 0);
    ok = (fclose(f) == 0) && ok;
    free(z);
    return ok;
}

/* ── Frame capture ─────────────────────────────────────────────────────── */

void ivp_render_request_capture(ivp_renderer_t *r, const char *path)
{
    if (!r) return;
    if (!path) { r->capture_path[0] = '\0'; return; }
    snprintf(r->capture_path, sizeof(r->capture_path), "%s", path);
}

bool ivp_render_capture_pending(const ivp_renderer_t *r)
{
    return r && r->capture_path[0] != '\0';
}

SDL_GPUTexture *ivp_capture_target(ivp_renderer_t *r)
{
    if (!r->capture_path[0]) return NULL;
    int pw, ph;
    SDL_GetWindowSizeInPixels(r->window, &pw, &ph);
    if (r->capture_tex && (r->capture_w != pw || r->capture_h != ph)) {
        SDL_ReleaseGPUTexture(r->gpu.device, r->capture_tex);
        r->capture_tex = NULL;
    }
    if (!r->capture_tex) {
        SDL_GPUTextureCreateInfo ci = {0};
        ci.type                 = SDL_GPU_TEXTURETYPE_2D;
        ci.format               = r->gpu.swapchain_format;
        ci.width                = (Uint32)pw;
        ci.height               = (Uint32)ph;
        ci.layer_count_or_depth = 1;
        ci.num_levels           = 1;
        ci.sample_count         = SDL_GPU_SAMPLECOUNT_1;
        ci.usage                = SDL_GPU_TEXTUREUSAGE_COLOR_TARGET |
                                  SDL_GPU_TEXTUREUSAGE_SAMPLER;
        r->capture_tex = SDL_CreateGPUTexture(r->gpu.device, &ci);
        if (!r->capture_tex) {
            fprintf(stderr, "capture texture failed: %s\n", SDL_GetError());
            r->capture_path[0] = '\0';
            return NULL;
        }
        r->capture_w = pw;
        r->capture_h = ph;
    }
    return r->capture_tex;
}

void ivp_capture_record(ivp_renderer_t *r)
{
    if (!r->capture_tex || !r->capture_path[0]) return;

    SDL_GPUBlitInfo blit = {0};
    blit.source.texture       = r->capture_tex;
    blit.source.w             = (Uint32)r->capture_w;
    blit.source.h             = (Uint32)r->capture_h;
    blit.destination.texture  = r->swapchain_tex;
    blit.destination.w        = (Uint32)r->capture_w;
    blit.destination.h        = (Uint32)r->capture_h;
    blit.load_op              = SDL_GPU_LOADOP_DONT_CARE;
    blit.filter               = SDL_GPU_FILTER_NEAREST;
    SDL_BlitGPUTexture(r->cmd, &blit);

    SDL_GPUTransferBufferCreateInfo tci = {0};
    tci.usage = SDL_GPU_TRANSFERBUFFERUSAGE_DOWNLOAD;
    tci.size  = (Uint32)(r->capture_w * r->capture_h * 4);
    r->capture_tb = SDL_CreateGPUTransferBuffer(r->gpu.device, &tci);
    if (!r->capture_tb) {
        fprintf(stderr, "capture buffer failed: %s\n", SDL_GetError());
        r->capture_path[0] = '\0';
        return;
    }
    SDL_GPUCopyPass *cp = SDL_BeginGPUCopyPass(r->cmd);
    SDL_GPUTextureRegion src = {0};
    src.texture = r->capture_tex;
    src.w = (Uint32)r->capture_w;
    src.h = (Uint32)r->capture_h;
    src.d = 1;
    SDL_GPUTextureTransferInfo dst = {0};
    dst.transfer_buffer = r->capture_tb;
    SDL_DownloadFromGPUTexture(cp, &src, &dst);
    SDL_EndGPUCopyPass(cp);
}

void ivp_capture_finish(ivp_renderer_t *r, SDL_GPUFence *fence)
{
    if (!r->capture_tb) return;
    bool ok = false;
    if (fence && SDL_WaitForGPUFences(r->gpu.device, true, &fence, 1)) {
        const unsigned char *px = (const unsigned char *)
            SDL_MapGPUTransferBuffer(r->gpu.device, r->capture_tb, false);
        size_t n = (size_t)r->capture_w * (size_t)r->capture_h;
        unsigned char *rgb = px ? (unsigned char *)malloc(n * 3) : NULL;
        if (rgb) {
            bool bgr = r->gpu.swapchain_format == SDL_GPU_TEXTUREFORMAT_B8G8R8A8_UNORM ||
                       r->gpu.swapchain_format == SDL_GPU_TEXTUREFORMAT_B8G8R8A8_UNORM_SRGB;
            for (size_t i = 0; i < n; i++) {
                rgb[3*i + 0] = px[4*i + (bgr ? 2 : 0)];
                rgb[3*i + 1] = px[4*i + 1];
                rgb[3*i + 2] = px[4*i + (bgr ? 0 : 2)];
            }
            ok = ivp_png_write_rgb(r->capture_path, rgb, r->capture_w, r->capture_h);
            free(rgb);
        }
        if (px) SDL_UnmapGPUTransferBuffer(r->gpu.device, r->capture_tb);
    }
    if (!ok) fprintf(stderr, "screenshot to %s failed\n", r->capture_path);
    SDL_ReleaseGPUTransferBuffer(r->gpu.device, r->capture_tb);
    r->capture_tb = NULL;
    r->capture_path[0] = '\0';
}

void ivp_capture_destroy(ivp_renderer_t *r)
{
    if (r->capture_tb) SDL_ReleaseGPUTransferBuffer(r->gpu.device, r->capture_tb);
    if (r->capture_tex) SDL_ReleaseGPUTexture(r->gpu.device, r->capture_tex);
    r->capture_tb = NULL;
    r->capture_tex = NULL;
}
