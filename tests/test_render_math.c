/* test_render_math.c -- Render module logic that needs no GPU
 *
 * Matrix helpers, screen-to-world rays, ray/sphere and ray/AABB tests,
 * frustum culling and the PNG writer used for screenshots.
 */
#include "ivp_test.h"

#include <ivp/render/ivp_render.h>
#include <ivp/render/ivp_render_math.h>
#include "ivp_render_internal.h"

#include <stdint.h>

/* Camera at (0,0,10) looking at the origin, 60 degrees, 16:9. */
static void make_vp(float vp[16])
{
    float view[16], proj[16];
    const float eye[3] = { 0, 0, 10 }, target[3] = { 0, 0, 0 }, up[3] = { 0, 1, 0 };
    ivp_render_mat4_lookat(view, eye, target, up);
    ivp_render_mat4_perspective(proj, 1.0471975512f, 16.0f / 9.0f, 0.1f, 100.0f);
    ivp_render_mat4_mul(vp, proj, view);
}

IVP_TEST(mat4_invert_roundtrip) {
    float vp[16], inv[16], id[16];
    make_vp(vp);
    IVP_ASSERT_TRUE(ivp_render_mat4_invert(inv, vp));
    ivp_render_mat4_mul(id, vp, inv);
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            IVP_ASSERT_NEAR(id[4 * c + r], c == r ? 1.0f : 0.0f, 1e-3);
}

IVP_TEST(mat4_invert_general) {
    /* neither symmetric nor affine: every cofactor term is exercised */
    const float m[16] = {  2, -1,  0.5f, 3,
                           1,  4, -2,    0.25f,
                          -3,  1,  5,    2,
                           0.5f, 2, 1,   6 };
    float inv[16], id[16];
    IVP_ASSERT_TRUE(ivp_render_mat4_invert(inv, m));
    ivp_render_mat4_mul(id, m, inv);
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            IVP_ASSERT_NEAR(id[4 * c + r], c == r ? 1.0f : 0.0f, 1e-4);
    ivp_render_mat4_mul(id, inv, m);
    for (int c = 0; c < 4; c++)
        for (int r = 0; r < 4; r++)
            IVP_ASSERT_NEAR(id[4 * c + r], c == r ? 1.0f : 0.0f, 1e-4);
}

IVP_TEST(mat4_invert_affine) {
    /* scale + translation: the inverse undoes both */
    const float m[16] = { 2, 0, 0, 0,  0, 3, 0, 0,  0, 0, 4, 0,  1, 2, 3, 1 };
    float inv[16];
    IVP_ASSERT_TRUE(ivp_render_mat4_invert(inv, m));
    IVP_ASSERT_NEAR(inv[0], 0.5f, 1e-6);
    IVP_ASSERT_NEAR(inv[5], 1.0f / 3.0f, 1e-6);
    IVP_ASSERT_NEAR(inv[10], 0.25f, 1e-6);
    IVP_ASSERT_NEAR(inv[12], -0.5f, 1e-6);
    IVP_ASSERT_NEAR(inv[13], -2.0f / 3.0f, 1e-6);
    IVP_ASSERT_NEAR(inv[14], -0.75f, 1e-6);
}

IVP_TEST(mat4_invert_singular) {
    float zero[16] = { 0 }, out[16];
    IVP_ASSERT_FALSE(ivp_render_mat4_invert(out, zero));
}

IVP_TEST(unproject_center_ray_hits_target) {
    float vp[16], o[3], d[3];
    make_vp(vp);
    ivp_render_unproject_ray_ex(vp, 1280, 720, 640.0f, 360.0f, o, d);
    /* the ray runs along -Z through the origin */
    IVP_ASSERT_NEAR(d[0], 0.0f, 1e-4);
    IVP_ASSERT_NEAR(d[1], 0.0f, 1e-4);
    IVP_ASSERT_NEAR(d[2], -1.0f, 1e-4);
    float t = (0.0f - o[2]) / d[2];
    IVP_ASSERT_NEAR(o[0] + d[0] * t, 0.0f, 1e-3);
    IVP_ASSERT_NEAR(o[1] + d[1] * t, 0.0f, 1e-3);
}

IVP_TEST(unproject_corner_matches_projection) {
    float vp[16], o[3], d[3];
    make_vp(vp);
    /* top-left pixel: the ray goes left and up (screen y grows downwards) */
    ivp_render_unproject_ray_ex(vp, 1280, 720, 0.0f, 0.0f, o, d);
    IVP_ASSERT_TRUE(d[0] < 0.0f);
    IVP_ASSERT_TRUE(d[1] > 0.0f);
    IVP_ASSERT_NEAR(sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]), 1.0f, 1e-4);
    /* half the vertical field of view */
    float half_h = tanf(1.0471975512f * 0.5f);
    IVP_ASSERT_NEAR(d[1] / -d[2], half_h, 1e-3);
}

IVP_TEST(ray_sphere_hit_and_miss) {
    const float o[3] = { 0, 0, 10 }, d[3] = { 0, 0, -1 };
    const float c[3] = { 0, 0, 0 };
    float t = 0.0f;
    IVP_ASSERT_TRUE(ivp_render_ray_sphere(o, d, c, 2.0f, &t));
    IVP_ASSERT_NEAR(t, 8.0f, 1e-4);
    const float off[3] = { 3, 0, 0 };
    IVP_ASSERT_FALSE(ivp_render_ray_sphere(o, d, off, 2.0f, &t));
    const float behind[3] = { 0, 0, 20 };
    IVP_ASSERT_FALSE(ivp_render_ray_sphere(o, d, behind, 2.0f, &t));
}

IVP_TEST(ray_aabb_hit_and_miss) {
    const float o[3] = { 0, 0, 10 }, d[3] = { 0, 0, -1 };
    const float mn[3] = { -1, -1, -1 }, mx[3] = { 1, 1, 1 };
    float t = 0.0f;
    IVP_ASSERT_TRUE(ivp_render_ray_aabb(o, d, mn, mx, &t));
    IVP_ASSERT_NEAR(t, 9.0f, 1e-4);
    const float mn2[3] = { 2, -1, -1 }, mx2[3] = { 4, 1, 1 };
    IVP_ASSERT_FALSE(ivp_render_ray_aabb(o, d, mn2, mx2, &t));
}

IVP_TEST(frustum_sphere_culling) {
    float vp[16];
    ivp_frustum_t f;
    make_vp(vp);
    ivp_frustum_extract(&f, vp);
    const float visible[3] = { 0, 0, 0 };
    const float behind[3] = { 0, 0, 30 };
    const float beyond_far[3] = { 0, 0, -200 };
    const float far_left[3] = { -80, 0, 0 };
    IVP_ASSERT_TRUE(ivp_frustum_test_sphere(&f, visible, 1.0f));
    IVP_ASSERT_FALSE(ivp_frustum_test_sphere(&f, behind, 1.0f));
    IVP_ASSERT_FALSE(ivp_frustum_test_sphere(&f, beyond_far, 1.0f));
    IVP_ASSERT_FALSE(ivp_frustum_test_sphere(&f, far_left, 1.0f));
    /* a large sphere reaching into the view is kept */
    IVP_ASSERT_TRUE(ivp_frustum_test_sphere(&f, far_left, 90.0f));
}

IVP_TEST(frustum_aabb_culling) {
    float vp[16];
    ivp_frustum_t f;
    make_vp(vp);
    ivp_frustum_extract(&f, vp);
    const float a_mn[3] = { -1, -1, -1 }, a_mx[3] = { 1, 1, 1 };
    const float b_mn[3] = { 100, 0, 0 }, b_mx[3] = { 101, 1, 1 };
    IVP_ASSERT_TRUE(ivp_frustum_test_aabb(&f, a_mn, a_mx));
    IVP_ASSERT_FALSE(ivp_frustum_test_aabb(&f, b_mn, b_mx));
}

/* ── PNG writer ───────────────────────────────────────────────────────── */

static uint32_t be32(const unsigned char *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static uint32_t crc32_of(const unsigned char *p, size_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) c = (c & 1u) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
    }
    return c ^ 0xFFFFFFFFu;
}

IVP_TEST(png_writer_roundtrip) {
    /* 300 x 100 RGB: a gradient, 90 KB raw, so the stored deflate stream
     * needs two blocks */
    enum { W = 300, H = 100 };
    static unsigned char rgb[W * H * 3];
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            rgb[(y * W + x) * 3 + 0] = (unsigned char)x;
            rgb[(y * W + x) * 3 + 1] = (unsigned char)y;
            rgb[(y * W + x) * 3 + 2] = (unsigned char)(x ^ y);
        }
    const char *path = "test_render_math.png";
    IVP_ASSERT_TRUE(ivp_png_write_rgb(path, rgb, W, H));

    FILE *f = fopen(path, "rb");
    IVP_ASSERT_NOT_NULL(f);
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char *buf = (unsigned char *)malloc((size_t)size);
    IVP_ASSERT_NOT_NULL(buf);
    IVP_ASSERT_EQ((long)fread(buf, 1, (size_t)size, f), size);
    fclose(f);
    remove(path);

    static const unsigned char sig[8] = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };
    IVP_ASSERT_EQ(memcmp(buf, sig, 8), 0);

    /* walk the chunks, checking each CRC; gather the IDAT payload */
    size_t pos = 8;
    unsigned char *z = (unsigned char *)malloc((size_t)size);
    size_t zlen = 0;
    int saw_ihdr = 0, saw_iend = 0;
    while (pos + 12 <= (size_t)size) {
        uint32_t len = be32(buf + pos);
        const unsigned char *type = buf + pos + 4;
        IVP_ASSERT_TRUE(pos + 12 + len <= (size_t)size);
        IVP_ASSERT_EQ(crc32_of(type, 4 + len), be32(buf + pos + 8 + len));
        if (memcmp(type, "IHDR", 4) == 0) {
            IVP_ASSERT_EQ(be32(type + 4), (uint32_t)W);
            IVP_ASSERT_EQ(be32(type + 8), (uint32_t)H);
            IVP_ASSERT_EQ(type[12], 8);    /* bit depth */
            IVP_ASSERT_EQ(type[13], 2);    /* RGB */
            saw_ihdr = 1;
        } else if (memcmp(type, "IDAT", 4) == 0) {
            memcpy(z + zlen, type + 4, len);
            zlen += len;
        } else if (memcmp(type, "IEND", 4) == 0) {
            saw_iend = 1;
        }
        pos += 12 + len;
    }
    IVP_ASSERT_TRUE(saw_ihdr && saw_iend);
    IVP_ASSERT_EQ(pos, (size_t)size);

    /* inflate the stored blocks and compare with the pixels */
    IVP_ASSERT_TRUE(zlen > 6 && z[0] == 0x78);
    size_t raw_len = (size_t)(W * 3 + 1) * H;
    unsigned char *raw = (unsigned char *)malloc(raw_len);
    size_t zi = 2, ri = 0;
    int final = 0;
    while (!final) {
        IVP_ASSERT_TRUE(zi + 5 <= zlen);
        final = z[zi] & 1;
        IVP_ASSERT_EQ(z[zi] & 6, 0);       /* stored block */
        uint32_t n = z[zi + 1] | ((uint32_t)z[zi + 2] << 8);
        uint32_t nn = z[zi + 3] | ((uint32_t)z[zi + 4] << 8);
        IVP_ASSERT_EQ(n, (~nn) & 0xFFFFu);
        zi += 5;
        IVP_ASSERT_TRUE(zi + n <= zlen && ri + n <= raw_len);
        memcpy(raw + ri, z + zi, n);
        zi += n;
        ri += n;
    }
    IVP_ASSERT_EQ(ri, raw_len);
    uint32_t a = 1, b = 0;
    for (size_t i = 0; i < raw_len; i++) { a = (a + raw[i]) % 65521u; b = (b + a) % 65521u; }
    IVP_ASSERT_EQ(be32(z + zi), (b << 16) | a);
    for (int y = 0; y < H; y++) {
        IVP_ASSERT_EQ(raw[y * (W * 3 + 1)], 0);   /* filter none */
        IVP_ASSERT_EQ(memcmp(raw + y * (W * 3 + 1) + 1, rgb + y * W * 3, W * 3), 0);
    }
    free(raw);
    free(z);
    free(buf);
}

IVP_TEST(png_writer_rejects_bad_input) {
    unsigned char px[3] = { 0 };
    IVP_ASSERT_FALSE(ivp_png_write_rgb(NULL, px, 1, 1));
    IVP_ASSERT_FALSE(ivp_png_write_rgb("x.png", NULL, 1, 1));
    IVP_ASSERT_FALSE(ivp_png_write_rgb("x.png", px, 0, 1));
}

int main(void) {
    IVP_REGISTER(mat4_invert_roundtrip);
    IVP_REGISTER(mat4_invert_general);
    IVP_REGISTER(mat4_invert_affine);
    IVP_REGISTER(mat4_invert_singular);
    IVP_REGISTER(unproject_center_ray_hits_target);
    IVP_REGISTER(unproject_corner_matches_projection);
    IVP_REGISTER(ray_sphere_hit_and_miss);
    IVP_REGISTER(ray_aabb_hit_and_miss);
    IVP_REGISTER(frustum_sphere_culling);
    IVP_REGISTER(frustum_aabb_culling);
    IVP_REGISTER(png_writer_roundtrip);
    IVP_REGISTER(png_writer_rejects_bad_input);
    IVP_RUN_ALL();
}
