/* demo_water.h -- a translucent water surface for the examples
 *
 * The renderer's meshes are uploaded once, and a wave moves every frame, so the
 * surface is drawn as a grid of thin translucent plates (instanced solid
 * boxes), each tilted to the slope of the height field at its centre. Y is
 * down: the height function returns the Y of the surface.
 */
#ifndef IVP_EXAMPLE_DEMO_WATER_H
#define IVP_EXAMPLE_DEMO_WATER_H

#include "demo_app.h"

typedef float (*demo_height_fn)(void *ctx, float x, float z);

static inline void demo_vec3_norm(float v[3])
{
    const float l = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (l > 1e-8f) {
        v[0] /= l;
        v[1] /= l;
        v[2] /= l;
    }
}

/* Plates of `step` metres over [x0, x1] x [z0, z1] on the grid of multiples of step, coloured
 * between `deep` and `crest` by the slope, with the given opacity. */
static inline void demo_draw_water_plates(demo_viewer_t *v, float x0, float x1, float z0, float z1, float step,
                                          demo_height_fn height, void *ctx, const float deep[3], const float crest[3],
                                          float alpha)
{
    if (!v->renderer) return;
    const int ix0 = (int)floorf(x0 / step), ix1 = (int)ceilf(x1 / step), iz0 = (int)floorf(z0 / step), iz1 = (int)ceilf(z1 / step);
    const float e = 0.5f * step;
    demo_set_alpha(v, alpha);
    for (int iz = iz0; iz < iz1; iz++)
        for (int ix = ix0; ix < ix1; ix++) {
            const float x = ((float)ix + 0.5f) * step, z = ((float)iz + 0.5f) * step;
            const float y = height(ctx, x, z);
            const float sx = (height(ctx, x + e, z) - height(ctx, x - e, z)) / step;
            const float sz = (height(ctx, x, z + e) - height(ctx, x, z - e)) / step;
            float ax[3] = { 1.0f, sx, 0.0f }, tz[3] = { 0.0f, sz, 1.0f };
            demo_vec3_norm(ax);
            demo_vec3_norm(tz);
            /* normal = tz x ax points up (-Y) */
            float ay[3] = { tz[1] * ax[2] - tz[2] * ax[1], tz[2] * ax[0] - tz[0] * ax[2], tz[0] * ax[1] - tz[1] * ax[0] };
            if (ay[1] > 0.0f) {
                ay[0] = -ay[0];
                ay[1] = -ay[1];
                ay[2] = -ay[2];
            }
            demo_vec3_norm(ay);
            const float az[3] = { ax[1] * ay[2] - ax[2] * ay[1], ax[2] * ay[0] - ax[0] * ay[2], ax[0] * ay[1] - ax[1] * ay[0] };
            const float rot[9] = { ax[0], ax[1], ax[2], ay[0], ay[1], ay[2], az[0], az[1], az[2] };
            const float slope = fminf(1.0f, 2.5f * sqrtf(sx * sx + sz * sz));
            const float col[3] = { deep[0] + (crest[0] - deep[0]) * slope, deep[1] + (crest[1] - deep[1]) * slope,
                                   deep[2] + (crest[2] - deep[2]) * slope };
            const float pos[3] = { x, y, z }, half[3] = { e, 0.02f, e };
            demo_solid_box(v, pos, half, rot, col);
        }
    demo_set_alpha(v, 1.0f);
}

#endif /* IVP_EXAMPLE_DEMO_WATER_H */
