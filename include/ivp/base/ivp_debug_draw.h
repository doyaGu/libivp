/* ivp_debug_draw.h -- Debug visualization callback interface
 *
 * Provides a vtable-based debug drawing interface. Users implement
 * the callbacks to render debug overlays (lines, spheres, AABBs,
 * contact points, etc.) in their rendering engine.
 *
 * The physics engine calls these during debug phases if a debug
 * drawer is registered on the environment.
 *
 * This is a header-only interface (no .c file needed).
 */
#ifndef IVP_DEBUG_DRAW_H
#define IVP_DEBUG_DRAW_H

#include <ivp/base/ivp_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Debug draw colors ─────────────────────────────────────────────────── */

typedef struct ivp_debug_color {
    float r, g, b, a;
} ivp_debug_color_t;

static inline ivp_debug_color_t ivp_debug_color(float r, float g, float b) {
    ivp_debug_color_t c;
    c.r = r; c.g = g; c.b = b; c.a = 1.0f;
    return c;
}

#define IVP_DEBUG_RED     ivp_debug_color(1.0f, 0.0f, 0.0f)
#define IVP_DEBUG_GREEN   ivp_debug_color(0.0f, 1.0f, 0.0f)
#define IVP_DEBUG_BLUE    ivp_debug_color(0.0f, 0.0f, 1.0f)
#define IVP_DEBUG_YELLOW  ivp_debug_color(1.0f, 1.0f, 0.0f)
#define IVP_DEBUG_WHITE   ivp_debug_color(1.0f, 1.0f, 1.0f)
#define IVP_DEBUG_CYAN    ivp_debug_color(0.0f, 1.0f, 1.0f)
#define IVP_DEBUG_MAGENTA ivp_debug_color(1.0f, 0.0f, 1.0f)

/* ── Debug drawer vtable ───────────────────────────────────────────────── */

typedef struct ivp_debug_drawer {
    /* Draw a line segment from p0 to p1. */
    void (*draw_line)(void *ctx,
                      const float p0[3], const float p1[3],
                      ivp_debug_color_t color);

    /* Draw a point / small sphere at position p with given radius. */
    void (*draw_point)(void *ctx,
                       const float p[3], float radius,
                       ivp_debug_color_t color);

    /* Draw a wireframe sphere. */
    void (*draw_sphere)(void *ctx,
                        const float center[3], float radius,
                        ivp_debug_color_t color);

    /* Draw an axis-aligned bounding box. */
    void (*draw_aabb)(void *ctx,
                      const float aabb_min[3], const float aabb_max[3],
                      ivp_debug_color_t color);

    /* Draw a contact point (normal + contact). */
    void (*draw_contact)(void *ctx,
                         const float point[3],
                         const float normal[3],
                         float length,
                         ivp_debug_color_t color);

    /* Draw text at a 3D position. */
    void (*draw_text)(void *ctx,
                      const float pos[3],
                      const char *text,
                      ivp_debug_color_t color);

    /* User context pointer. */
    void *ctx;
} ivp_debug_drawer_t;

/* ── Convenience: null-check wrappers ──────────────────────────────────── */

static inline void ivp_debug_line(const ivp_debug_drawer_t *dd,
                                    const float p0[3], const float p1[3],
                                    ivp_debug_color_t color) {
    if (dd && dd->draw_line) dd->draw_line(dd->ctx, p0, p1, color);
}

static inline void ivp_debug_point(const ivp_debug_drawer_t *dd,
                                     const float p[3], float radius,
                                     ivp_debug_color_t color) {
    if (dd && dd->draw_point) dd->draw_point(dd->ctx, p, radius, color);
}

static inline void ivp_debug_sphere(const ivp_debug_drawer_t *dd,
                                      const float center[3], float radius,
                                      ivp_debug_color_t color) {
    if (dd && dd->draw_sphere) dd->draw_sphere(dd->ctx, center, radius, color);
}

static inline void ivp_debug_aabb(const ivp_debug_drawer_t *dd,
                                    const float aabb_min[3],
                                    const float aabb_max[3],
                                    ivp_debug_color_t color) {
    if (dd && dd->draw_aabb) dd->draw_aabb(dd->ctx, aabb_min, aabb_max, color);
}

static inline void ivp_debug_contact(const ivp_debug_drawer_t *dd,
                                       const float point[3],
                                       const float normal[3],
                                       float length,
                                       ivp_debug_color_t color) {
    if (dd && dd->draw_contact) dd->draw_contact(dd->ctx, point, normal, length, color);
}

static inline void ivp_debug_text(const ivp_debug_drawer_t *dd,
                                    const float pos[3],
                                    const char *text,
                                    ivp_debug_color_t color) {
    if (dd && dd->draw_text) dd->draw_text(dd->ctx, pos, text, color);
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_DEBUG_DRAW_H */
