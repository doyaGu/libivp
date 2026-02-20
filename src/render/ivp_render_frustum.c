/* ivp_render_frustum.c -- View frustum extraction and culling tests
 *
 * Implements Gribb-Hartmann method: extract 6 frustum planes from the
 * combined view-projection matrix. Provides sphere and AABB culling tests.
 */

#include "ivp_render_internal.h"
#include <math.h>

/* ── Normalize a plane (nx,ny,nz,d) ───────────────────────────────────── */

static void normalize_plane(float p[4])
{
    float len = sqrtf(p[0]*p[0] + p[1]*p[1] + p[2]*p[2]);
    if (len > 1e-8f) {
        float inv = 1.0f / len;
        p[0] *= inv;
        p[1] *= inv;
        p[2] *= inv;
        p[3] *= inv;
    }
}

/* ── Gribb-Hartmann frustum extraction from column-major VP matrix ────── */

void ivp_frustum_extract(ivp_frustum_t *f, const float m[16])
{
    /* Left:   row3 + row0 */
    f->planes[0][0] = m[3]  + m[0];
    f->planes[0][1] = m[7]  + m[4];
    f->planes[0][2] = m[11] + m[8];
    f->planes[0][3] = m[15] + m[12];
    normalize_plane(f->planes[0]);

    /* Right:  row3 - row0 */
    f->planes[1][0] = m[3]  - m[0];
    f->planes[1][1] = m[7]  - m[4];
    f->planes[1][2] = m[11] - m[8];
    f->planes[1][3] = m[15] - m[12];
    normalize_plane(f->planes[1]);

    /* Bottom: row3 + row1 */
    f->planes[2][0] = m[3]  + m[1];
    f->planes[2][1] = m[7]  + m[5];
    f->planes[2][2] = m[11] + m[9];
    f->planes[2][3] = m[15] + m[13];
    normalize_plane(f->planes[2]);

    /* Top:    row3 - row1 */
    f->planes[3][0] = m[3]  - m[1];
    f->planes[3][1] = m[7]  - m[5];
    f->planes[3][2] = m[11] - m[9];
    f->planes[3][3] = m[15] - m[13];
    normalize_plane(f->planes[3]);

    /* Near:   row3 + row2 */
    f->planes[4][0] = m[3]  + m[2];
    f->planes[4][1] = m[7]  + m[6];
    f->planes[4][2] = m[11] + m[10];
    f->planes[4][3] = m[15] + m[14];
    normalize_plane(f->planes[4]);

    /* Far:    row3 - row2 */
    f->planes[5][0] = m[3]  - m[2];
    f->planes[5][1] = m[7]  - m[6];
    f->planes[5][2] = m[11] - m[10];
    f->planes[5][3] = m[15] - m[14];
    normalize_plane(f->planes[5]);
}

/* ── Sphere vs frustum ─────────────────────────────────────────────────── */

bool ivp_frustum_test_sphere(const ivp_frustum_t *f,
                             const float center[3], float radius)
{
    for (int i = 0; i < 6; i++) {
        float dist = f->planes[i][0] * center[0]
                   + f->planes[i][1] * center[1]
                   + f->planes[i][2] * center[2]
                   + f->planes[i][3];
        if (dist < -radius) return false;  /* completely outside */
    }
    return true;  /* inside or intersecting */
}

/* ── AABB vs frustum ───────────────────────────────────────────────────── */

bool ivp_frustum_test_aabb(const ivp_frustum_t *f,
                           const float mn[3], const float mx[3])
{
    for (int i = 0; i < 6; i++) {
        /* Find the p-vertex (the corner most aligned with the plane normal) */
        float px = (f->planes[i][0] >= 0.0f) ? mx[0] : mn[0];
        float py = (f->planes[i][1] >= 0.0f) ? mx[1] : mn[1];
        float pz = (f->planes[i][2] >= 0.0f) ? mx[2] : mn[2];

        float dist = f->planes[i][0] * px
                   + f->planes[i][1] * py
                   + f->planes[i][2] * pz
                   + f->planes[i][3];
        if (dist < 0.0f) return false;  /* entirely outside this plane */
    }
    return true;
}
