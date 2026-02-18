/* ivp_surman.h — Surface manager vtable interface
 *
 * The surface manager provides a uniform interface to query geometric surfaces
 * regardless of their internal representation (polygon, ball, grid, etc.).
 *
 * In this C17 rewrite, we use a function-pointer vtable instead of C++ virtual.
 */
#ifndef IVP_SURMAN_H
#define IVP_SURMAN_H

#include <ivp/geometry/ivp_compact_surface.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Surface manager type enumeration ───────────────────────────────────── */

typedef enum ivp_surman_type {
    IVP_SURMAN_POLYGON = 0,
    IVP_SURMAN_BALL    = 1
} ivp_surman_type_t;

/* ── Surface manager vtable ─────────────────────────────────────────────── */

typedef struct ivp_surman ivp_surman_t;

typedef struct ivp_surman_vtable {
    /* Get the single convex ledge if this is a simple convex shape.
     * Returns NULL for compound objects. */
    const ivp_compact_ledge_t *(*get_single_convex)(const ivp_surman_t *sm);

    /* Get the center of mass. */
    void (*get_mass_center)(const ivp_surman_t *sm, float out[3]);

    /* Get the rotation inertia (diagonal of inertia tensor for unit mass). */
    void (*get_rotation_inertia)(const ivp_surman_t *sm, float out[3]);

    /* Get bounding radius and deviation from a given center. */
    void (*get_radius_and_deviation)(const ivp_surman_t *sm,
                                      const float center[3],
                                      float *out_radius,
                                      float *out_deviation);

    /* Get the type of this surface manager. */
    ivp_surman_type_t (*get_type)(const ivp_surman_t *sm);

    /* Get the ledgetree root node for hierarchical collision.
     * Returns NULL for simple (single-convex) objects and balls. */
    const ivp_compact_ledgetree_node_t *(*get_ledgetree_root)(const ivp_surman_t *sm);

    /* Destructor. */
    void (*destroy)(ivp_surman_t *sm);
} ivp_surman_vtable_t;

/* ── Surface manager instance ───────────────────────────────────────────── */

struct ivp_surman {
    const ivp_surman_vtable_t *vtable;
    /* Implementation-specific data follows */
};

/* ── Inline dispatchers ─────────────────────────────────────────────────── */

static inline const ivp_compact_ledge_t *ivp_surman_get_single_convex(
    const ivp_surman_t *sm)
{
    return sm->vtable->get_single_convex(sm);
}

static inline void ivp_surman_get_mass_center(const ivp_surman_t *sm,
                                               float out[3]) {
    sm->vtable->get_mass_center(sm, out);
}

static inline void ivp_surman_get_rotation_inertia(const ivp_surman_t *sm,
                                                     float out[3]) {
    sm->vtable->get_rotation_inertia(sm, out);
}

static inline ivp_surman_type_t ivp_surman_get_type(const ivp_surman_t *sm) {
    return sm->vtable->get_type(sm);
}

static inline const ivp_compact_ledgetree_node_t *ivp_surman_get_ledgetree_root(
    const ivp_surman_t *sm)
{
    if (sm->vtable->get_ledgetree_root) {
        return sm->vtable->get_ledgetree_root(sm);
    }
    return NULL;
}

static inline void ivp_surman_destroy(ivp_surman_t *sm) {
    if (sm->vtable->destroy) {
        sm->vtable->destroy(sm);
    }
}

/* ── Polygon surface manager ────────────────────────────────────────────── */

/* Surface manager for polygon objects (wraps an ivp_compact_surface_t). */
typedef struct ivp_surman_polygon {
    ivp_surman_t             base;
    const ivp_compact_surface_t *compact_surface;
} ivp_surman_polygon_t;

/* Initialize a polygon surface manager. The compact surface must outlive
 * the surface manager. */
void ivp_surman_polygon_init(ivp_surman_polygon_t *sm,
                              const ivp_compact_surface_t *surface);

/* ── Ball surface manager ───────────────────────────────────────────────── */

/* Surface manager for ball (sphere) objects with a given radius. */
typedef struct ivp_surman_ball {
    ivp_surman_t base;
    float        radius;
} ivp_surman_ball_t;

/* Initialize a ball surface manager with the given radius. */
void ivp_surman_ball_init(ivp_surman_ball_t *sm, float radius);

#ifdef __cplusplus
}
#endif

#endif /* IVP_SURMAN_H */
