/* ivp_render_capture.c -- Screenshot capture (PNG)
 *
 * The backend reads the captured frame back (ivp_gpu_capture_begin /
 * ivp_gpu_capture_take); this file writes it as a PNG.
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
    r->capture_written = false;
    if (!path) { r->capture_path[0] = '\0'; return; }
    snprintf(r->capture_path, sizeof(r->capture_path), "%s", path);
}

bool ivp_render_capture_pending(const ivp_renderer_t *r)
{
    return r && r->capture_path[0] != '\0';
}

bool ivp_render_capture_written(const ivp_renderer_t *r)
{
    return r && r->capture_written;
}

bool ivp_capture_begin(ivp_renderer_t *r)
{
    if (!r->capture_path[0]) return false;
    if (!ivp_gpu_capture_begin(r->gpu)) {
        r->capture_path[0] = '\0';
        return false;
    }
    return true;
}

void ivp_capture_finish(ivp_renderer_t *r)
{
    int w = 0, h = 0;
    unsigned char *rgb = ivp_gpu_capture_take(r->gpu, &w, &h);
    bool ok = rgb && ivp_png_write_rgb(r->capture_path, rgb, w, h);
    if (!ok) fprintf(stderr, "screenshot to %s failed\n", r->capture_path);
    free(rgb);
    r->capture_path[0] = '\0';
    r->capture_written = ok;
}
