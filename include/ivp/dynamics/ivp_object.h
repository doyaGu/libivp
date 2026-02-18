/* ivp_object.h — Physics object for IVP C17
 *
 * An object ties together a core (dynamics state), surface manager (geometry),
 * material, and user client data.
 */
#ifndef IVP_OBJECT_H
#define IVP_OBJECT_H

#include <ivp/base/ivp_types.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/geometry/ivp_surman.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Object type ────────────────────────────────────────────────────────── */

typedef enum ivp_object_type {
    IVP_OBJECT_BALL    = 0,   /* sphere: uses upper_limit_radius          */
    IVP_OBJECT_POLYGON = 1    /* convex polyhedron: uses surman for ledge  */
} ivp_object_type_t;

/* ── Material ───────────────────────────────────────────────────────────── */

typedef struct ivp_material {
    ivp_real_t friction;      /* default: 0.5   — combined by multiplication */
    ivp_real_t elasticity;    /* default: 0.3   — combined by multiplication */
    ivp_real_t adhesion;      /* default: 0.0   — combined by addition       */
    bool       second_friction_x_enabled; /* anisotropic friction flag       */
    ivp_real_t second_friction_factor;    /* X-axis friction multiplier      */
} ivp_material_t;

static inline ivp_material_t ivp_material_default(void) {
    ivp_material_t m;
    m.friction   = IVP_REAL(0.5);
    m.elasticity = IVP_REAL(0.3);
    m.adhesion   = IVP_REAL(0.0);
    m.second_friction_x_enabled = false;
    m.second_friction_factor = IVP_REAL(1.0);
    return m;
}

/* ── Object template (creation parameters) ──────────────────────────────── */

typedef struct ivp_object_template {
    ivp_object_type_t type;                        /* BALL or POLYGON           */
    ivp_real_t        mass;                       /* must be > 0 for dynamic   */
    ivp_vec3r_t       rot_inertia;                /* diagonal inertia tensor   */
    ivp_vec3r_t       position;                   /* initial world position    */
    ivp_quatr_t       orientation;                /* initial orientation       */
    ivp_real_t        speed_damp_factor;           /* default: 0.01            */
    ivp_vec3r_t       rot_speed_damp_factor;       /* default: (0.01,0.01,0.01)*/
    ivp_material_t    material;
    const char       *no_collision_group;          /* same non-null value => no collision */
    const char       *name;                        /* optional debug name      */
    void             *client_data;                 /* user pointer             */
    bool              physical_unmoveable;          /* static object            */
    bool              ccd_enabled;                  /* per-object CCD (default: false) */
    ivp_real_t        ccd_speed_threshold;          /* 0 = auto (50% of radius / dt) */
} ivp_object_template_t;

/* Fill with sensible defaults. Must still set mass and surface. */
void ivp_object_template_init(ivp_object_template_t *tmpl);

/* ── Object ─────────────────────────────────────────────────────────────── */

/* Forward declarations */
struct ivp_environment;
struct ivp_collision_listener;
struct ivp_synapse_link;

#define IVP_MAX_COLLISION_LISTENERS_PER_OBJECT 4

typedef struct ivp_object {
    ivp_core_t           core;
    ivp_object_type_t    type;          /* BALL or POLYGON              */
    ivp_material_t       material;
    const char          *no_collision_group; /* borrowed group tag (legacy) */
    int                  collision_group;    /* 0 = no group (default) */
    uint32_t             layer_mask;         /* default: 0xFFFFFFFF (collide with all) */
    const ivp_surman_t  *surman;        /* geometry accessor (borrowed) */
    const char          *name;          /* debug name (borrowed)        */
    void                *client_data;   /* user data pointer            */
    struct ivp_environment *owner;      /* back-link to owning world    */
    int                  dense_index;   /* index in object system array */
    int                  sim_unit_id;   /* simulation unit (0 = default) */
    uint32_t             handle_slot;
    uint32_t             handle_generation;
    uint64_t             handle_key;

    /* Per-object collision listeners (up to IVP_MAX_COLLISION_LISTENERS_PER_OBJECT) */
    struct ivp_collision_listener *collision_listeners[IVP_MAX_COLLISION_LISTENERS_PER_OBJECT];

    /* Legacy-style reverse lookup lists for active mindist/friction synapses. */
    struct ivp_synapse_link *mindist_synapses;
    struct ivp_synapse_link *friction_synapses;

    /* Continuous collision detection */
    bool              ccd_enabled;           /* per-object CCD flag (default: false) */
    ivp_real_t        ccd_speed_threshold;   /* 0 = auto (50% of radius / dt) */
} ivp_object_t;

/* Get the core from an object */
static inline ivp_core_t *ivp_object_get_core(ivp_object_t *obj) {
    return &obj->core;
}

static inline const ivp_core_t *ivp_object_get_core_const(const ivp_object_t *obj) {
    return &obj->core;
}

/* Get/set client data */
static inline void *ivp_object_get_client_data(const ivp_object_t *obj) {
    return obj->client_data;
}

static inline void ivp_object_set_client_data(ivp_object_t *obj, void *data) {
    obj->client_data = data;
}

/* Get world position at current PSI time */
static inline void ivp_object_get_position(const ivp_object_t *obj,
                                            ivp_vec3r_t *out) {
    *out = obj->core.pos_world_f_core_last_psi;
}

/* Get world orientation at current PSI time */
static inline void ivp_object_get_orientation(const ivp_object_t *obj,
                                               ivp_quatr_t *out) {
    *out = obj->core.q_world_f_core_last_psi;
}

/* Get linear velocity */
static inline void ivp_object_get_speed(const ivp_object_t *obj,
                                         ivp_vec3r_t *out) {
    *out = obj->core.speed;
}

/* Get angular velocity (core space) */
static inline void ivp_object_get_rot_speed(const ivp_object_t *obj,
                                              ivp_vec3f_t *out) {
    *out = obj->core.rot_speed;
}

/* Check if object is static */
static inline bool ivp_object_is_unmoveable(const ivp_object_t *obj) {
    return obj->core.physical_unmoveable;
}

/* Check if object is frozen */
static inline bool ivp_object_is_frozen(const ivp_object_t *obj) {
    return obj->core.movement_state == IVP_MT_NOT_SIM;
}

/* Get kinetic energy */
static inline ivp_real_t ivp_object_get_energy(const ivp_object_t *obj) {
    return ivp_core_get_energy(&obj->core);
}

/* ── CCD accessors ─────────────────────────────────────────────────────── */

static inline bool ivp_object_get_ccd_enabled(const ivp_object_t *obj) {
    return obj->ccd_enabled;
}

static inline void ivp_object_set_ccd_enabled(ivp_object_t *obj, bool enabled) {
    obj->ccd_enabled = enabled;
}

static inline ivp_real_t ivp_object_get_ccd_speed_threshold(const ivp_object_t *obj) {
    return obj->ccd_speed_threshold;
}

static inline void ivp_object_set_ccd_speed_threshold(ivp_object_t *obj,
                                                       ivp_real_t threshold) {
    obj->ccd_speed_threshold = threshold;
}

/* ── Object mutation (requires environment) ───────────────────────────── */

/* Change object mass at runtime. */
void ivp_object_change_mass(ivp_object_t *obj, ivp_real_t new_mass);

/* Make object unmoveable/moveable at runtime. */
void ivp_object_change_unmovable(ivp_object_t *obj, bool unmovable);

/* Pin/unpin object (no gravity/linear push). */
void ivp_object_set_pinned(ivp_object_t *obj, bool pinned);

/* Set collision group. Objects in the same non-zero group skip collision. */
static inline void ivp_object_set_collision_group(ivp_object_t *obj, int group) {
    obj->collision_group = group;
}

/* Set collision layer mask. Pairs with no overlapping bits skip collision. */
static inline void ivp_object_set_layer_mask(ivp_object_t *obj, uint32_t mask) {
    obj->layer_mask = mask;
}

/* Enable collisions (restore default layer mask, clear group). */
static inline void ivp_object_enable_collision(ivp_object_t *obj) {
    obj->collision_group = 0;
    obj->layer_mask = 0xFFFFFFFF;
}

/* Disable all collisions for this object (clears layer mask). */
static inline void ivp_object_disable_collision(ivp_object_t *obj) {
    obj->layer_mask = 0;
}

/* Recompile object after geometry change.
 * Updates bounding radius from surface manager and resets movement state. */
void ivp_object_recompile(ivp_object_t *obj);

#ifdef __cplusplus
}
#endif

#endif /* IVP_OBJECT_H */
