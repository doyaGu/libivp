/* ivp_surman.h — Surface manager vtable interface
 *
 * The surface manager provides a uniform interface to query geometric surfaces
 * regardless of their internal representation (polygon, ball, grid, etc.).
 *
 * In this C17 rewrite, we use a function-pointer vtable instead of C++ virtual.
 * Semantics follow the reference IVP_SurfaceManager family
 * (ivp_surface_manager/ivp_surface_manager.hxx, ivp_surman_polygon.cxx,
 * ivp_physics/ivp_ball.cxx).
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
struct ivp_ray_solver;
struct ivp_object;

/* Callback receiving ledges from the ledge queries. */
typedef void (*ivp_surman_ledge_fn)(void *ctx, const ivp_compact_ledge_t *ledge);

typedef struct ivp_surman_vtable {
    /* Reference get_single_convex(): the root ledge for a single convex
     * surface, the root compact hull for a compound surface built with a
     * root hull (non-terminal ledge, use the recursive path), NULL for a
     * compound without hull; balls return a dummy (zeroed) ledge. */
    const ivp_compact_ledge_t *(*get_single_convex)(const ivp_surman_t *sm);

    /* Get the center of mass. */
    void (*get_mass_center)(const ivp_surman_t *sm, float out[3]);

    /* Get the rotation inertia (diagonal of inertia tensor for unit mass). */
    void (*get_rotation_inertia)(const ivp_surman_t *sm, float out[3]);

    /* Radius and radius deviation relative to `center`
     * (get_radius_and_radius_dev_to_given_center). */
    void (*get_radius_and_deviation)(const ivp_surman_t *sm,
                                      const float center[3],
                                      float *out_radius,
                                      float *out_deviation);

    /* Get the type of this surface manager. */
    ivp_surman_type_t (*get_type)(const ivp_surman_t *sm);

    /* libivp: ledgetree root node for hierarchical collision.
     * Returns NULL for simple (single-convex) objects and balls. */
    const ivp_compact_ledgetree_node_t *(*get_ledgetree_root)(const ivp_surman_t *sm);

    /* Destructor. */
    void (*destroy)(ivp_surman_t *sm);

    /* All ledges intruding the sphere (visitor_position_os, radius), in the
     * reference traversal order (IVP_SurfaceManager_Polygon::
     * get_all_ledges_within_radius / IVP_SurfaceManager_Polygon_Solver::
     * traverse_cluster).  root_ledge: start below this (non-terminal) ledge
     * instead of the surface root; other_reference_ledge is unused by the
     * reference implementations. */
    void (*get_all_ledges_within_radius)(const ivp_surman_t *sm,
                                          const double visitor_position_os[3],
                                          double radius,
                                          const ivp_compact_ledge_t *root_ledge,
                                          const ivp_compact_ledge_t *other_reference_ledge,
                                          ivp_surman_ledge_fn add_fn,
                                          void *ctx);

    /* All terminal ledges (get_all_terminal_ledges). */
    void (*get_all_terminal_ledges)(const ivp_surman_t *sm,
                                     ivp_surman_ledge_fn add_fn,
                                     void *ctx);

    /* Feed all ledges hit by the ray into the ray solver
     * (insert_all_ledges_hitting_ray).  `object` is passed to the hit
     * listener.  NULL for surface managers without polygon data. */
    void (*insert_all_ledges_hitting_ray)(const ivp_surman_t *sm,
                                           struct ivp_ray_solver *ray_solver,
                                           struct ivp_object *object);

    /* IVP_SurfaceManager::add_reference_to_ledge / remove_reference_to_ledge:
     * a synapse starts / stops using a ledge of this surface manager, so a
     * manager that loads ledges on demand knows which ones are in use.
     * Called (like the reference) for polygon objects when a mindist is
     * initialised (IVP_Mindist::init_mindist) and destroyed (~IVP_Mindist),
     * and for both synapses of a friction contact point (IVP_Contact_Point
     * ctor / dtor).  The calls are balanced per synapse.  NULL: no-op (the
     * reference default; the built-in polygon, ball and grid managers ignore
     * them too).  The surface manager is passed const like everywhere in
     * this vtable (objects borrow it const); a reference counting manager
     * owns its counters and may cast. */
    void (*add_reference_to_ledge)(const ivp_surman_t *sm, const ivp_compact_ledge_t *ledge);
    void (*remove_reference_to_ledge)(const ivp_surman_t *sm, const ivp_compact_ledge_t *ledge);
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

static inline void ivp_surman_get_radius_and_deviation(const ivp_surman_t *sm,
                                                        const float center[3],
                                                        float *out_radius,
                                                        float *out_deviation) {
    sm->vtable->get_radius_and_deviation(sm, center, out_radius, out_deviation);
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

static inline void ivp_surman_get_all_ledges_within_radius(
    const ivp_surman_t *sm,
    const double visitor_position_os[3],
    double radius,
    const ivp_compact_ledge_t *root_ledge,
    const ivp_compact_ledge_t *other_reference_ledge,
    ivp_surman_ledge_fn add_fn,
    void *ctx)
{
    if (sm->vtable->get_all_ledges_within_radius) {
        sm->vtable->get_all_ledges_within_radius(sm, visitor_position_os, radius,
                                                 root_ledge, other_reference_ledge,
                                                 add_fn, ctx);
    }
}

static inline void ivp_surman_get_all_terminal_ledges(const ivp_surman_t *sm,
                                                      ivp_surman_ledge_fn add_fn,
                                                      void *ctx)
{
    if (sm->vtable->get_all_terminal_ledges) {
        sm->vtable->get_all_terminal_ledges(sm, add_fn, ctx);
    }
}

static inline void ivp_surman_insert_all_ledges_hitting_ray(const ivp_surman_t *sm,
                                                            struct ivp_ray_solver *ray_solver,
                                                            struct ivp_object *object)
{
    if (sm->vtable->insert_all_ledges_hitting_ray) {
        sm->vtable->insert_all_ledges_hitting_ray(sm, ray_solver, object);
    }
}

static inline void ivp_surman_add_reference_to_ledge(const ivp_surman_t *sm,
                                                     const ivp_compact_ledge_t *ledge)
{
    if (sm && sm->vtable->add_reference_to_ledge) {
        sm->vtable->add_reference_to_ledge(sm, ledge);
    }
}

static inline void ivp_surman_remove_reference_to_ledge(const ivp_surman_t *sm,
                                                        const ivp_compact_ledge_t *ledge)
{
    if (sm && sm->vtable->remove_reference_to_ledge) {
        sm->vtable->remove_reference_to_ledge(sm, ledge);
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

/* The compact surface of a plain polygon surface manager; NULL for every
 * other kind (balls, compact grids ...).  Surface managers of other kinds
 * also report IVP_SURMAN_POLYGON from get_type, so do not cast on the type
 * alone. */
const ivp_compact_surface_t *ivp_surman_get_polygon_surface(const ivp_surman_t *sm);

/* Shared polygon implementations (used by the grid surface manager too). */
void ivp_surman_polygon_surface_ledges_within_radius(
    const ivp_compact_surface_t *surface,
    const double visitor_position_os[3],
    double radius,
    const ivp_compact_ledge_t *root_ledge,
    ivp_surman_ledge_fn add_fn,
    void *ctx);

void ivp_surman_polygon_surface_radius_and_deviation(
    const ivp_compact_surface_t *surface,
    const float center[3],
    float *out_radius,
    float *out_deviation);

/* ── Ball surface manager ───────────────────────────────────────────────── */

/* Surface manager for ball (sphere) objects.  Like the reference
 * IVP_SurfaceManager_Ball it reports mass center 0, radius 0, deviation 0
 * and rotation inertia 0: the ball radius is an object property (reference
 * IVP_Ball::extra_radius = template radius + extra radius), and the object
 * computes the 2/5 r^2 inertia itself (IVP_Real_Object::init_object_core).
 * `radius` is kept here for the object creation code to read. */
typedef struct ivp_surman_ball {
    ivp_surman_t base;
    float        radius;
} ivp_surman_ball_t;

/* Initialize a ball surface manager with the given radius. */
void ivp_surman_ball_init(ivp_surman_ball_t *sm, float radius);

/* The dummy compact ledge shared by every ball surface manager
 * (sizeof(ledge) + sizeof(triangle) zero bytes, 16-byte aligned). */
const ivp_compact_ledge_t *ivp_surman_ball_dummy_ledge(void);

/* Radius stored in a ball surface manager (0 for other types). */
float ivp_surman_get_ball_radius(const ivp_surman_t *sm);

#ifdef __cplusplus
}
#endif

#endif /* IVP_SURMAN_H */
