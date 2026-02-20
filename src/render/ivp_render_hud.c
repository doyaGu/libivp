/* ivp_render_hud.c -- 2D text overlay with bitmap font (SDL3 GPU backend) */

#include <ivp/render/ivp_render_hud.h>
#include <ivp/render/ivp_render_math.h>
#include <ivp/base/ivp_allocator.h>
#include "ivp_render_gpu.h"

#include <string.h>
#include <stddef.h>
#include <stdarg.h>
#include <stdio.h>

/* ── Internal vertex type ──────────────────────────────────────────────── */

typedef struct hud_vertex {
    float pos[2];
    float col[3];
} hud_vertex_t;

/* ── HUD context ───────────────────────────────────────────────────────── */

struct ivp_render_hud_ctx {
    ivp_gpu_pipeline_t  pipeline;
    ivp_gpu_buffer_t    buffer;
    hud_vertex_t       *vertices;
    int                 vertex_count;
    ivp_render_hud_config_t config;
};

/* ── Bitmap font data (5x7, ASCII 32-126) ──────────────────────────────── */

#define FONT_FIRST 32
#define FONT_LAST  126

static const unsigned char s_font[][IVP_RENDER_HUD_FONT_H] = {
    /* 32 ' ' */ {0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    /* 33 '!' */ {0x04,0x04,0x04,0x04,0x00,0x04,0x00},
    /* 34 '"' */ {0x0A,0x0A,0x00,0x00,0x00,0x00,0x00},
    /* 35 '#' */ {0x0A,0x1F,0x0A,0x0A,0x1F,0x0A,0x00},
    /* 36 '$' */ {0x04,0x0F,0x14,0x0E,0x05,0x1E,0x04},
    /* 37 '%' */ {0x19,0x19,0x02,0x04,0x08,0x13,0x13},
    /* 38 '&' */ {0x08,0x14,0x14,0x08,0x15,0x12,0x0D},
    /* 39 ''' */ {0x04,0x04,0x00,0x00,0x00,0x00,0x00},
    /* 40 '(' */ {0x02,0x04,0x08,0x08,0x08,0x04,0x02},
    /* 41 ')' */ {0x08,0x04,0x02,0x02,0x02,0x04,0x08},
    /* 42 '*' */ {0x04,0x15,0x0E,0x1F,0x0E,0x15,0x04},
    /* 43 '+' */ {0x00,0x04,0x04,0x1F,0x04,0x04,0x00},
    /* 44 ',' */ {0x00,0x00,0x00,0x00,0x04,0x04,0x08},
    /* 45 '-' */ {0x00,0x00,0x00,0x1F,0x00,0x00,0x00},
    /* 46 '.' */ {0x00,0x00,0x00,0x00,0x00,0x04,0x00},
    /* 47 '/' */ {0x01,0x01,0x02,0x04,0x08,0x10,0x10},
    /* 48 '0' */ {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E},
    /* 49 '1' */ {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
    /* 50 '2' */ {0x0E,0x11,0x01,0x06,0x08,0x10,0x1F},
    /* 51 '3' */ {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E},
    /* 52 '4' */ {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02},
    /* 53 '5' */ {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E},
    /* 54 '6' */ {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E},
    /* 55 '7' */ {0x1F,0x01,0x02,0x04,0x08,0x08,0x08},
    /* 56 '8' */ {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E},
    /* 57 '9' */ {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C},
    /* 58 ':' */ {0x00,0x04,0x00,0x00,0x04,0x00,0x00},
    /* 59 ';' */ {0x00,0x04,0x00,0x00,0x04,0x04,0x08},
    /* 60 '<' */ {0x02,0x04,0x08,0x10,0x08,0x04,0x02},
    /* 61 '=' */ {0x00,0x00,0x1F,0x00,0x1F,0x00,0x00},
    /* 62 '>' */ {0x08,0x04,0x02,0x01,0x02,0x04,0x08},
    /* 63 '?' */ {0x0E,0x11,0x01,0x02,0x04,0x00,0x04},
    /* 64 '@' */ {0x0E,0x11,0x17,0x15,0x17,0x10,0x0E},
    /* 65 'A' */ {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11},
    /* 66 'B' */ {0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E},
    /* 67 'C' */ {0x0E,0x11,0x10,0x10,0x10,0x11,0x0E},
    /* 68 'D' */ {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E},
    /* 69 'E' */ {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F},
    /* 70 'F' */ {0x1F,0x10,0x10,0x1E,0x10,0x10,0x10},
    /* 71 'G' */ {0x0E,0x11,0x10,0x17,0x11,0x11,0x0F},
    /* 72 'H' */ {0x11,0x11,0x11,0x1F,0x11,0x11,0x11},
    /* 73 'I' */ {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E},
    /* 74 'J' */ {0x07,0x02,0x02,0x02,0x02,0x12,0x0C},
    /* 75 'K' */ {0x11,0x12,0x14,0x18,0x14,0x12,0x11},
    /* 76 'L' */ {0x10,0x10,0x10,0x10,0x10,0x10,0x1F},
    /* 77 'M' */ {0x11,0x1B,0x15,0x15,0x11,0x11,0x11},
    /* 78 'N' */ {0x11,0x19,0x15,0x13,0x11,0x11,0x11},
    /* 79 'O' */ {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E},
    /* 80 'P' */ {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10},
    /* 81 'Q' */ {0x0E,0x11,0x11,0x11,0x15,0x12,0x0D},
    /* 82 'R' */ {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11},
    /* 83 'S' */ {0x0E,0x11,0x10,0x0E,0x01,0x11,0x0E},
    /* 84 'T' */ {0x1F,0x04,0x04,0x04,0x04,0x04,0x04},
    /* 85 'U' */ {0x11,0x11,0x11,0x11,0x11,0x11,0x0E},
    /* 86 'V' */ {0x11,0x11,0x11,0x0A,0x0A,0x04,0x04},
    /* 87 'W' */ {0x11,0x11,0x11,0x15,0x15,0x1B,0x11},
    /* 88 'X' */ {0x11,0x11,0x0A,0x04,0x0A,0x11,0x11},
    /* 89 'Y' */ {0x11,0x11,0x0A,0x04,0x04,0x04,0x04},
    /* 90 'Z' */ {0x1F,0x01,0x02,0x04,0x08,0x10,0x1F},
    /* 91 '[' */ {0x0E,0x08,0x08,0x08,0x08,0x08,0x0E},
    /* 92 '\' */ {0x10,0x10,0x08,0x04,0x02,0x01,0x01},
    /* 93 ']' */ {0x0E,0x02,0x02,0x02,0x02,0x02,0x0E},
    /* 94 '^' */ {0x04,0x0A,0x11,0x00,0x00,0x00,0x00},
    /* 95 '_' */ {0x00,0x00,0x00,0x00,0x00,0x00,0x1F},
    /* 96 '`' */ {0x08,0x04,0x00,0x00,0x00,0x00,0x00},
    /* 97 'a' */ {0x00,0x00,0x0E,0x01,0x0F,0x11,0x0F},
    /* 98 'b' */ {0x10,0x10,0x1E,0x11,0x11,0x11,0x1E},
    /* 99 'c' */ {0x00,0x00,0x0E,0x11,0x10,0x11,0x0E},
    /*100 'd' */ {0x01,0x01,0x0F,0x11,0x11,0x11,0x0F},
    /*101 'e' */ {0x00,0x00,0x0E,0x11,0x1F,0x10,0x0E},
    /*102 'f' */ {0x06,0x08,0x1C,0x08,0x08,0x08,0x08},
    /*103 'g' */ {0x00,0x00,0x0F,0x11,0x0F,0x01,0x0E},
    /*104 'h' */ {0x10,0x10,0x1E,0x11,0x11,0x11,0x11},
    /*105 'i' */ {0x04,0x00,0x0C,0x04,0x04,0x04,0x0E},
    /*106 'j' */ {0x02,0x00,0x06,0x02,0x02,0x12,0x0C},
    /*107 'k' */ {0x10,0x10,0x12,0x14,0x18,0x14,0x12},
    /*108 'l' */ {0x0C,0x04,0x04,0x04,0x04,0x04,0x0E},
    /*109 'm' */ {0x00,0x00,0x1A,0x15,0x15,0x11,0x11},
    /*110 'n' */ {0x00,0x00,0x1E,0x11,0x11,0x11,0x11},
    /*111 'o' */ {0x00,0x00,0x0E,0x11,0x11,0x11,0x0E},
    /*112 'p' */ {0x00,0x00,0x1E,0x11,0x1E,0x10,0x10},
    /*113 'q' */ {0x00,0x00,0x0F,0x11,0x0F,0x01,0x01},
    /*114 'r' */ {0x00,0x00,0x16,0x19,0x10,0x10,0x10},
    /*115 's' */ {0x00,0x00,0x0F,0x10,0x0E,0x01,0x1E},
    /*116 't' */ {0x08,0x08,0x1C,0x08,0x08,0x09,0x06},
    /*117 'u' */ {0x00,0x00,0x11,0x11,0x11,0x13,0x0D},
    /*118 'v' */ {0x00,0x00,0x11,0x11,0x0A,0x0A,0x04},
    /*119 'w' */ {0x00,0x00,0x11,0x11,0x15,0x15,0x0A},
    /*120 'x' */ {0x00,0x00,0x11,0x0A,0x04,0x0A,0x11},
    /*121 'y' */ {0x00,0x00,0x11,0x11,0x0F,0x01,0x0E},
    /*122 'z' */ {0x00,0x00,0x1F,0x02,0x04,0x08,0x1F},
    /*123 '{' */ {0x02,0x04,0x04,0x08,0x04,0x04,0x02},
    /*124 '|' */ {0x04,0x04,0x04,0x04,0x04,0x04,0x04},
    /*125 '}' */ {0x08,0x04,0x04,0x02,0x04,0x04,0x08},
    /*126 '~' */ {0x00,0x00,0x08,0x15,0x02,0x00,0x00},
};

/* ── Internal: push a HUD line segment ─────────────────────────────────── */

static void push_hud_line(ivp_render_hud_ctx_t *ctx,
                          float x0, float y0, float x1, float y1,
                          const float col[3])
{
    if (ctx->vertex_count + 2 > ctx->config.max_vertices) return;
    hud_vertex_t *v = &ctx->vertices[ctx->vertex_count];
    v[0].pos[0] = x0; v[0].pos[1] = y0;
    memcpy(v[0].col, col, 3 * sizeof(float));
    v[1].pos[0] = x1; v[1].pos[1] = y1;
    memcpy(v[1].col, col, 3 * sizeof(float));
    ctx->vertex_count += 2;
}

/* ── Config ────────────────────────────────────────────────────────────── */

void ivp_render_hud_config_init(ivp_render_hud_config_t *cfg)
{
    cfg->max_vertices = IVP_RENDER_HUD_MAX_VERTICES;
    cfg->font_scale   = IVP_RENDER_HUD_FONT_SCALE;
}

/* ── Lifecycle ─────────────────────────────────────────────────────────── */

ivp_result_t ivp_render_hud_ctx_create_ex(ivp_render_hud_ctx_t **out,
                                          const ivp_render_hud_config_t *config,
                                          ivp_gpu_device_t *gpu)
{
    const ivp_allocator_t *alloc = ivp_default_allocator();

    ivp_render_hud_ctx_t *ctx = (ivp_render_hud_ctx_t *)ivp_alloc(alloc, sizeof(*ctx));
    if (!ctx) return IVP_ERR_OUT_OF_MEMORY;
    memset(ctx, 0, sizeof(*ctx));

    if (config) {
        ctx->config = *config;
    } else {
        ivp_render_hud_config_init(&ctx->config);
    }
    if (ctx->config.max_vertices <= 0)
        ctx->config.max_vertices = IVP_RENDER_HUD_MAX_VERTICES;
    if (ctx->config.font_scale <= 0)
        ctx->config.font_scale = IVP_RENDER_HUD_FONT_SCALE;

    /* Create 2D pipeline (no depth test, no depth write) */
    size_t vs_len, fs_len;
    const unsigned char *vs = ivp_gpu_vs_2d_code(gpu, &vs_len);
    const unsigned char *fs = ivp_gpu_fs_2d_code(gpu, &fs_len);

    ivp_gpu_attrib_desc_t attribs[2] = {
        { 0, 2, (int)offsetof(hud_vertex_t, pos) },
        { 1, 3, (int)offsetof(hud_vertex_t, col) },
    };
    ivp_result_t res = ivp_gpu_pipeline_create(
        &ctx->pipeline, gpu, vs, vs_len, fs, fs_len,
        (int)sizeof(hud_vertex_t), attribs, 2, false, false);
    if (res != IVP_OK) {
        ivp_free(alloc, ctx, sizeof(*ctx));
        return res;
    }

    res = ivp_gpu_buffer_create(&ctx->buffer, gpu,
                                ctx->config.max_vertices,
                                (int)sizeof(hud_vertex_t));
    if (res != IVP_OK) {
        ivp_gpu_pipeline_destroy(&ctx->pipeline, gpu);
        ivp_free(alloc, ctx, sizeof(*ctx));
        return res;
    }

    size_t vertices_bytes = (size_t)ctx->config.max_vertices * sizeof(hud_vertex_t);
    ctx->vertices = (hud_vertex_t *)ivp_alloc(alloc, vertices_bytes);
    if (!ctx->vertices) {
        ivp_gpu_buffer_destroy(&ctx->buffer, gpu);
        ivp_gpu_pipeline_destroy(&ctx->pipeline, gpu);
        ivp_free(alloc, ctx, sizeof(*ctx));
        return IVP_ERR_OUT_OF_MEMORY;
    }
    ctx->vertex_count = 0;

    *out = ctx;
    return IVP_OK;
}

void ivp_render_hud_ctx_destroy(ivp_render_hud_ctx_t *ctx,
                                ivp_gpu_device_t *gpu)
{
    if (!ctx) return;
    const ivp_allocator_t *alloc = ivp_default_allocator();
    size_t vertices_bytes = (size_t)ctx->config.max_vertices * sizeof(hud_vertex_t);
    if (ctx->vertices) {
        ivp_free(alloc, ctx->vertices, vertices_bytes);
    }
    ivp_gpu_buffer_destroy(&ctx->buffer, gpu);
    ivp_gpu_pipeline_destroy(&ctx->pipeline, gpu);
    ivp_free(alloc, ctx, sizeof(*ctx));
}

void ivp_render_hud_ctx_reset(ivp_render_hud_ctx_t *ctx)
{
    ctx->vertex_count = 0;
}

void ivp_render_hud_ctx_upload(ivp_render_hud_ctx_t *ctx,
                               ivp_gpu_device_t *gpu,
                               SDL_GPUCommandBuffer *cmd)
{
    ivp_gpu_buffer_upload(&ctx->buffer, gpu, cmd,
                          ctx->vertices, ctx->vertex_count);
}

void ivp_render_hud_ctx_draw(ivp_render_hud_ctx_t *ctx,
                             ivp_gpu_device_t *gpu,
                             SDL_GPUCommandBuffer *cmd,
                             SDL_GPURenderPass *pass,
                             int width, int height)
{
    if (ctx->vertex_count <= 0) return;
    (void)gpu;
    float ortho[16];
    ivp_render_mat4_ortho(ortho, 0.0f, (float)width, (float)height, 0.0f,
                          -1.0f, 1.0f);
    ivp_gpu_buffer_draw(&ctx->buffer, pass, cmd,
                        &ctx->pipeline, ortho, ctx->vertex_count);
}

/* ── Text drawing ──────────────────────────────────────────────────────── */

void ivp_render_draw_text_2d_scaled(ivp_render_hud_ctx_t *ctx,
                                    float x, float y,
                                    const char *text,
                                    const float color[3],
                                    int scale)
{
    if (scale <= 0) scale = ctx->config.font_scale;
    float cx = x, cy = y;
    float sc = (float)scale;

    for (const char *p = text; *p; p++) {
        if (*p == '\n') { cx = x; cy += (IVP_RENDER_HUD_FONT_H + 1) * sc; continue; }
        int ch = (int)(unsigned char)*p;
        if (ch < FONT_FIRST || ch > FONT_LAST) { cx += (IVP_RENDER_HUD_FONT_W + 1)*sc; continue; }

        const unsigned char *g = s_font[ch - FONT_FIRST];
        for (int row = 0; row < IVP_RENDER_HUD_FONT_H; row++) {
            unsigned char bits = g[row];
            for (int col = 0; col < IVP_RENDER_HUD_FONT_W; col++) {
                if (bits & (1 << (IVP_RENDER_HUD_FONT_W - 1 - col))) {
                    float px = cx + (float)col * sc;
                    float py = cy + (float)row * sc;
                    push_hud_line(ctx, px, py, px + sc, py, color);
                }
            }
        }
        cx += (IVP_RENDER_HUD_FONT_W + 1) * sc;
    }
}

void ivp_render_draw_text_2d(ivp_render_hud_ctx_t *ctx,
                             float x, float y,
                             const char *text,
                             const float color[3])
{
    ivp_render_draw_text_2d_scaled(ctx, x, y, text, color,
                                   ctx->config.font_scale);
}

void ivp_render_draw_text_2d_aligned(ivp_render_hud_ctx_t *ctx,
                                     float x, float y,
                                     const char *text,
                                     const float color[3],
                                     ivp_text_align_t align)
{
    if (align == IVP_TEXT_ALIGN_CENTER) {
        float w = ivp_render_hud_text_width_scaled(text, ctx->config.font_scale);
        x -= w * 0.5f;
    } else if (align == IVP_TEXT_ALIGN_RIGHT) {
        float w = ivp_render_hud_text_width_scaled(text, ctx->config.font_scale);
        x -= w;
    }
    ivp_render_draw_text_2d(ctx, x, y, text, color);
}

void ivp_render_draw_text_2d_fmt(ivp_render_hud_ctx_t *ctx,
                                 float x, float y,
                                 const float color[3],
                                 const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    ivp_render_draw_text_2d(ctx, x, y, buf, color);
}

/* ── Utilities ─────────────────────────────────────────────────────────── */

float ivp_render_hud_line_height_scaled(int scale)
{
    return (IVP_RENDER_HUD_FONT_H + 1) * (float)scale;
}

float ivp_render_hud_line_height(void)
{
    return ivp_render_hud_line_height_scaled(IVP_RENDER_HUD_FONT_SCALE);
}

float ivp_render_hud_text_width_scaled(const char *text, int scale)
{
    float sc = (float)scale;
    int count = 0;
    for (const char *p = text; *p && *p != '\n'; p++) {
        count++;
    }
    return (float)count * (IVP_RENDER_HUD_FONT_W + 1) * sc;
}

float ivp_render_hud_text_width(const char *text)
{
    return ivp_render_hud_text_width_scaled(text, IVP_RENDER_HUD_FONT_SCALE);
}

int ivp_render_hud_ctx_count(const ivp_render_hud_ctx_t *ctx)
{
    return ctx->vertex_count;
}

int ivp_render_hud_ctx_capacity(const ivp_render_hud_ctx_t *ctx)
{
    return ctx->config.max_vertices;
}
