/* ivp_object.h — Physics object for IVP C17
 *
 * An object ties together a core (dynamics state), surface manager (geometry),
 * material, and user client data.
 */
#ifndef IVP_OBJECT_H
#define IVP_OBJECT_H

#include <ivp/base/ivp_types.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_handle.h>
#include <ivp/geometry/ivp_surman.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Object type ────────────────────────────────────────────────────────── */

typedef enum ivp_object_type {
    IVP_OBJECT_NONE    = 0,   /* sentinel / uninitialized                 */
    IVP_OBJECT_CLUSTER = 1,   /* container for other objects (hierarchy)   */
    IVP_OBJECT_POLYGON = 2,   /* convex polyhedron: uses surman for ledge  */
    IVP_OBJECT_BALL    = 3,   /* sphere: uses upper_limit_radius           */
    IVP_OBJECT_MAX     = 4    /* enum count sentinel                       */
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
    const char       *no_collision_group;          /* DEPRECATED: use collision_group instead. same non-null value => no collision */
    const char       *name;                        /* optional debug name      */
    void             *client_data;                 /* user pointer             */
    bool              physical_unmoveable;          /* static object            */
    bool              ccd_enabled;                  /* unused (kept for API compatibility) */
    ivp_real_t        ccd_speed_threshold;          /* unused */

    /* IVP_Template_Real_Object / IVP_Template_Ball fields */
    ivp_real_t        ball_radius;                  /* IVP_Template_Ball::radius (BALL objects)       */
    ivp_real_t        extra_radius;                 /* default 0                                      */
    bool              rot_inertia_is_factor;        /* default true (reference): inertia =
                                                     * surface inertia (unit mass) * rot_inertia * mass;
                                                     * false: rot_inertia is the absolute inertia      */
    ivp_real_t        auto_check_rot_inertia;       /* default 0.03                                   */
    bool              pinned;                       /* default false                                  */
    bool              enable_piling_optimization;   /* default false                                  */
    bool              use_mass_center_override;     /* default false                                  */
    ivp_vec3r_t       mass_center_override;         /* object space                                   */
    /* at most IVP_NO_COLL_GROUP_STRING_LEN - 1 = 7 characters: a longer id
     * COREs when it is used (reference IVP_Template_Real_Object::
     * set_nocoll_group_ident) */
    const char       *nocoll_group_ident;           /* reference collision group string (NULL = none) */
    bool              enable_collision_detection;   /* default true                                   */
    bool              start_in_simulation;          /* default true (ensure_in_simulation_now)        */
} ivp_object_template_t;

/* Fill with sensible defaults. Must still set mass and surface. */
void ivp_object_template_init(ivp_object_template_t *tmpl);

/* ── Object ─────────────────────────────────────────────────────────────── */

/* Forward declarations */
struct ivp_environment;
struct ivp_collision_listener;
struct ivp_cluster;

#define IVP_MAX_COLLISION_LISTENERS_PER_OBJECT 4 /* deprecated: no limit any more */

typedef struct ivp_object {
    /* IVP_Real_Object cores (heap allocated, owned by the environment):
     *   physical_core  coordinate system, mass, speed ... (eventually merged)
     *   friction_core  long term (merged) core for frictions
     *   original_core  the core that was valid at creation of the object
     * All three are the same core unless objects are merged or attached
     * (then all three are the shared core).  NULL for clusters. */
    ivp_core_t          *physical_core;
    ivp_core_t          *friction_core;
    ivp_core_t          *original_core;
    ivp_object_type_t    type;          /* BALL or POLYGON              */
    ivp_material_t       material;
    const char          *no_collision_group; /* DEPRECATED: legacy group tag (pointer equality) */
    int                  collision_group;    /* preferred: 0 = no group (default) */
    uint32_t             layer_mask;         /* default: 0xFFFFFFFF (collide with all) */
    const ivp_surman_t  *surman;        /* geometry accessor (borrowed) */
    const char          *name;          /* debug name (borrowed)        */
    void                *client_data;   /* user data pointer            */
    struct ivp_environment *owner;      /* back-link to owning world    */
    int                  dense_index;   /* index in object system array */
    ivp_handle_t         handle;        /* generational handle */
    uint64_t             creation_sequence; /* engine-owned world-local order */
    /* Derived only from the object's own creation data, so scene edits
     * elsewhere cannot change it; paces movement checks with
     * IVP_EDIT_LOCALITY and stays zero otherwise. */
    uint64_t             movement_check_salt;

    /* flags.collision_listener_exists: per-object collision listeners are
     * kept by the environment (IVP_Cluster_Manager::collision_callback_hash) */
    bool                 collision_listener_exists;

    /* Continuous collision detection */
    bool              ccd_enabled;           /* per-object CCD flag (default: false) */
    ivp_real_t        ccd_speed_threshold;   /* 0 = auto (50% of radius / dt) */

    /* Core-to-object offset (ported from IVP_Real_Object) */
    ivp_vec3r_t         shift_core_f_object;         /* translation from core CoM to object origin (core frame) */
    bool                shift_core_f_object_is_zero;  /* optimization flag — true when shift is identity */
    ivp_quatr_t        *q_core_f_object;             /* rotation from core to object frame (NULL = identity) */

    /* Cluster hierarchy (ported from IVP_Object): real objects live in the
     * environment's root cluster unless moved into a sub-cluster */
    struct ivp_object  *next_in_cluster;    /* next sibling in parent cluster  */
    struct ivp_object  *prev_in_cluster;    /* previous sibling                */
    struct ivp_cluster *parent_cluster;     /* parent cluster (NULL if none)   */

    /* ── Collision core (port of IVP_Real_Object collision state) ──────── */
    struct ivp_md_synapse  *exact_synapses;    /* exact mindist synapses           */
    struct ivp_md_synapse  *invalid_synapses;  /* intruded mindist synapses        */
    struct ivp_fr_synapse  *fr_synapses;       /* contact point synapses           */
    struct ivp_ov_element  *ov_element;        /* OV-tree element (NULL = disabled)*/
    struct ivp_cache_obj   *cache_object;      /* AT-transform cache (lazy)        */
    void                   *controller_phantom; /* IVP_Controller_Phantom (opaque)  */
    uint8_t                 object_movement_state; /* flags.object_movement_state   */
    bool                    collision_detection_enabled;
    float                   extra_radius;      /* extra collision radius           */
    char                    nocoll_group_ident[8]; /* IVP_NO_COLL_GROUP_STRING_LEN  */
    struct ivp_hull_mgr    *hull_manager;      /* per-object hull manager          */
    bool                    object_deletion_deferred;
    bool                    check_vicinity_on_deferred_deletion;
    bool                    object_listener_exists; /* flags.object_listener_exists */
    struct ivp_anchor      *anchors;           /* anchors of registered actuators (ivp_anchor.h) */
} ivp_object_t;

/* Internal physics ordering must not depend on allocation addresses or on
 * the registry's reusable handle slots. */
static inline uint64_t ivp_object_order_key(const ivp_object_t *obj) {
    return obj->creation_sequence;
}

/* IVP_Real_Object::get_core: the current (physical) core.  Several objects
 * share one core after ivp_environment_merge_objects / ivp_object_attach_object;
 * the object frame relative to the core is shift_core_f_object /
 * q_core_f_object. */
static inline ivp_core_t *ivp_object_get_core(ivp_object_t *obj) {
    return obj->physical_core;
}

static inline const ivp_core_t *ivp_object_get_core_const(const ivp_object_t *obj) {
    return obj->physical_core;
}

/* IVP_Real_Object::get_original_core: the core that was valid at creation of
 * the object (replaced by merge_objects / attach_object / detach_object). */
static inline ivp_core_t *ivp_object_get_original_core(const ivp_object_t *obj) {
    return obj->original_core;
}

/* Get/set client data */
static inline void *ivp_object_get_client_data(const ivp_object_t *obj) {
    return obj->client_data;
}

static inline void ivp_object_set_client_data(ivp_object_t *obj, void *data) {
    obj->client_data = data;
}

/* Get the core (mass center) world position at the current PSI
 * (IVP_Core::get_position_PSI, m_world_f_core_last_psi). */
static inline void ivp_object_get_position(const ivp_object_t *obj,
                                            ivp_vec3r_t *out) {
    *out = obj->physical_core->m_world_f_core_last_psi.pos;
}

/* Get world orientation at current PSI time */
static inline void ivp_object_get_orientation(const ivp_object_t *obj,
                                               ivp_quatr_t *out) {
    *out = obj->physical_core->q_world_f_core_last_psi;
}

/* Get linear velocity */
static inline void ivp_object_get_speed(const ivp_object_t *obj,
                                         ivp_vec3r_t *out) {
    *out = obj->physical_core->speed;
}

/* Get angular velocity (core space) */
static inline void ivp_object_get_rot_speed(const ivp_object_t *obj,
                                              ivp_vec3f_t *out) {
    *out = obj->physical_core->rot_speed;
}

/* Check if object is static */
static inline bool ivp_object_is_unmoveable(const ivp_object_t *obj) {
    return obj->physical_core->physical_unmoveable;
}

/* Check if object is frozen (IVP_Real_Object::get_movement_state() == IVP_MT_NOT_SIM;
 * static objects report IVP_MT_STATIC, not frozen) */
static inline bool ivp_object_is_frozen(const ivp_object_t *obj) {
    return obj->object_movement_state == IVP_MT_NOT_SIM;
}

/* Check if two objects share the same legacy no_collision_group (pointer equality). */
static inline bool ivp_objects_share_no_collision_group(const ivp_object_t *a,
                                                         const ivp_object_t *b) {
    return a->no_collision_group && b->no_collision_group &&
           a->no_collision_group == b->no_collision_group;
}

/* Get kinetic energy */
static inline ivp_real_t ivp_object_get_energy(const ivp_object_t *obj) {
    return ivp_core_get_energy(obj->physical_core);
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

/* IVP_Real_Object::recheck_collision_filter: re-evaluate the collision filter
 * for all pairs of this object (drops mindists the filter now rejects and
 * creates the ones it now accepts).  The object setters below call this
 * automatically; call it directly after changing the environment's filter,
 * exclusive pairs, or the public fields without using those setters. */
void ivp_object_recheck_collision_filter(ivp_object_t *obj);

/* Set collision group. Objects in the same non-zero group skip collision.
 * Existing pairs are re-evaluated immediately. */
static inline void ivp_object_set_collision_group(ivp_object_t *obj, int group) {
    obj->collision_group = group;
    ivp_object_recheck_collision_filter(obj);
}

/* Set collision layer mask. Pairs with no overlapping bits skip collision.
 * Existing pairs are re-evaluated immediately. */
static inline void ivp_object_set_layer_mask(ivp_object_t *obj, uint32_t mask) {
    obj->layer_mask = mask;
    ivp_object_recheck_collision_filter(obj);
}

/* Enable collisions immediately (restore default layer mask, clear group). */
static inline void ivp_object_enable_collision(ivp_object_t *obj) {
    obj->collision_group = 0;
    obj->layer_mask = 0xFFFFFFFF;
    ivp_object_recheck_collision_filter(obj);
}

/* Disable all collisions for this object immediately (clears layer mask). */
static inline void ivp_object_disable_collision(ivp_object_t *obj) {
    obj->layer_mask = 0;
    ivp_object_recheck_collision_filter(obj);
}

/* ── IVP_Real_Object helpers ───────────────────────────────────────────── */

/* IVP_Real_Object::disable_simulation: freeze the object's core (and its
 * simulation unit when everything in it is calm) immediately, regardless of
 * the speed; its contact points are removed silently.  Returns true when the
 * object is not simulated any more (unmoveable objects: always true). */
bool ivp_object_disable_simulation(ivp_object_t *obj);

/* IVP_Real_Object::change_fast_piling_allowed (on the friction core). */
void ivp_object_change_fast_piling_allowed(ivp_object_t *obj, bool flag);

/* IVP_Real_Object::ensure_in_simulation: revive at the next PSI. */
void ivp_object_ensure_in_simulation(ivp_object_t *obj);
/* IVP_Real_Object::ensure_in_simulation_now: revive immediately. */
void ivp_object_ensure_in_simulation_now(ivp_object_t *obj);

/* IVP_Real_Object::async_add_speed_object_ws / async_add_rot_speed_object_cs /
 * async_push_object_ws: wake the object; the change is committed at the
 * next PSI (impulse in kg*m/s, rotation speed in core space). */
void ivp_object_async_add_speed_ws(ivp_object_t *obj, const ivp_vec3r_t *speed_ws);
void ivp_object_async_add_rot_speed_cs(ivp_object_t *obj, const ivp_vec3r_t *rot_speed_cs);
void ivp_object_async_push_ws(ivp_object_t *obj, const ivp_vec3r_t *position_ws, const ivp_vec3r_t *impulse_ws);

/* IVP_Real_Object::beam_object_to_new_position (object frame pose). */
void ivp_object_beam_to_new_position(ivp_object_t *obj, const ivp_quatr_t *rotation_world_f_object,
                                     const ivp_vec3r_t *position_ws, bool optimize_for_repeated_calls);

/* IVP_Real_Object::get_m_world_f_object_AT: object frame at the current
 * (interpolated) environment time. */
void ivp_object_get_m_world_f_object_AT(const ivp_object_t *obj, ivp_transform_r_t *m_world_f_object_out);

/* Recompile object after geometry change.
 * Updates bounding radius from surface manager and resets movement state. */
void ivp_object_recompile(ivp_object_t *obj);

#ifdef __cplusplus
}
#endif

#endif /* IVP_OBJECT_H */
