/* ivp_snapshot.c -- world snapshots at PSI boundaries (libivp extension).
 *
 * Restore creates every object frozen and without collision detection and
 * overwrites its value state bit for bit, reviving it if it was simulated.
 * The collision state is then rebuilt structure by structure instead of by
 * the creation logic: OV elements go into their captured nodes, mindists get
 * their captured fields, and every list keeps its captured order.  Sorted
 * lists (hull manager synapses, the time manager queue) are rebuilt by
 * inserting in reverse order: a finite key goes in front of equal keys, so
 * the enumeration order comes back exactly.
 *
 * Snapshot storage uses the default allocator, since a snapshot outlives the
 * world it was captured from; restore scratch uses the new world's. */
#include <ivp/dynamics/ivp_snapshot.h>

#include <ivp/base/ivp_allocator.h>
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/collision/ivp_hull_mgr.h>
#include <ivp/controllers/ivp_actuator_spring.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/dynamics/ivp_object.h>

#include "collision/ivp_collision_restore.h"
#include "controllers/ivp_controller_system.h"
#include "dynamics/ivp_env_pipeline.h"
#include "dynamics/ivp_sim_internal.h"
#include "solver/ivp_friction_internal.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

typedef enum snapshot_ref_kind {
    SNAPSHOT_REF_PSI_EVENT = 0,
    SNAPSHOT_REF_OV_ELEMENT,
    SNAPSHOT_REF_SYNAPSE,
    SNAPSHOT_REF_MINDIST,
    SNAPSHOT_REF_OBJECT,
    SNAPSHOT_REF_CONTACT,     /* contact point; side picks a synapse */
    SNAPSHOT_REF_PAIR,
    SNAPSHOT_REF_INFO,
    SNAPSHOT_REF_CONTROLLER,  /* snapshot_controller_id; side counts objects */
    SNAPSHOT_REF_ANCHOR       /* registered controller; side picks its anchor */
} snapshot_ref_kind_t;

/* controller identities: the gravity controller, three per friction system
 * (dynamic, static, energy), then the registered controllers */
#define SNAPSHOT_CONTROLLER_GRAVITY 0
#define SNAPSHOT_CONTROLLER_ID(system, which) (1 + 3 * (system) + (which))
#define SNAPSHOT_SLOT_ID(system_count, slot) (1 + 3 * (system_count) + (slot))

/* ── registered controllers ──────────────────────────────────────────── */

#define SNAPSHOT_CONTROLLER_OBJECTS 4

/* A built-in controller whose state can be copied: where its object
 * pointers and anchors are, and whether a state can be captured at all. */
typedef struct snapshot_controller_kind {
    ivp_controller_fn_t fn;
    size_t state_size;
    int (*objects)(void *state, ivp_object_t **fields[SNAPSHOT_CONTROLLER_OBJECTS]);
    ivp_anchor_t *(*anchors)(void *state, int *count);
    bool (*capturable)(const void *state);
} snapshot_controller_kind_t;

static int snapshot_spring_objects(void *state,
                                   ivp_object_t **fields[SNAPSHOT_CONTROLLER_OBJECTS])
{
    ivp_spring_state_t *spring = (ivp_spring_state_t *)state;
    fields[0] = &spring->obj_a;
    fields[1] = &spring->obj_b;
    fields[2] = &spring->anchors[0].l_anchor_object;
    fields[3] = &spring->anchors[1].l_anchor_object;
    return 4;
}

static ivp_anchor_t *snapshot_spring_anchors(void *state, int *count)
{
    *count = 2;
    return ((ivp_spring_state_t *)state)->anchors;
}

/* active values belong to their manager, which the snapshot does not hold */
static bool snapshot_spring_capturable(const void *state)
{
    const ivp_spring_state_t *spring = (const ivp_spring_state_t *)state;
    return !spring->active_float_spring_len &&
           !spring->active_float_spring_constant &&
           !spring->active_float_spring_damp &&
           !spring->active_float_spring_rel_pos_damp;
}

static int snapshot_constraint_objects(void *state,
                                       ivp_object_t **fields[SNAPSHOT_CONTROLLER_OBJECTS])
{
    ivp_constraint_state_t *constraint = (ivp_constraint_state_t *)state;
    fields[0] = &constraint->obj_a;
    fields[1] = &constraint->obj_b;
    fields[2] = &constraint->core.obj_r;
    fields[3] = &constraint->core.obj_a;
    return 4;
}

static ivp_anchor_t *snapshot_no_anchors(void *state, int *count)
{
    (void)state;
    *count = 0;
    return NULL;
}

static bool snapshot_always_capturable(const void *state)
{
    (void)state;
    return true;
}

static const snapshot_controller_kind_t snapshot_controller_kinds[] = {
    { ivp_controller_spring, sizeof(ivp_spring_state_t),
      snapshot_spring_objects, snapshot_spring_anchors,
      snapshot_spring_capturable },
    { ivp_controller_constraint, sizeof(ivp_constraint_state_t),
      snapshot_constraint_objects, snapshot_no_anchors,
      snapshot_always_capturable },
};

static const snapshot_controller_kind_t *snapshot_find_kind(ivp_controller_fn_t fn)
{
    for (size_t i = 0; i < sizeof(snapshot_controller_kinds) /
                               sizeof(snapshot_controller_kinds[0]);
         i++) {
        if (snapshot_controller_kinds[i].fn == fn) {
            return &snapshot_controller_kinds[i];
        }
    }
    return NULL;
}

/* one entry of a captured list: index into the captured array of its
 * kind; side picks a synapse, or counts the object refs that follow a
 * controller */
typedef struct snapshot_ref {
    snapshot_ref_kind_t kind;
    int index;
    int side;
    float value;   /* key in sorted lists */
} snapshot_ref_t;

typedef struct snapshot_span {
    int first;
    int count;
} snapshot_span_t;

typedef struct snapshot_object {
    uint64_t creation_sequence;
    uint64_t movement_check_salt;
    const ivp_surman_t *surman;
    ivp_object_type_t type;
    ivp_material_t material;
    const char *no_collision_group;
    int collision_group;
    uint32_t layer_mask;
    const char *name;
    void *client_data;
    char nocoll_group_ident[8];
    float extra_radius;
    bool collision_detection_enabled;
    uint8_t object_movement_state;
    bool ccd_enabled;
    ivp_real_t ccd_speed_threshold;
    ivp_vec3r_t shift_core_f_object;
    bool shift_core_f_object_is_zero;

    /* only the value fields are restored (snapshot_restore_core) */
    ivp_core_t core;
    bool has_old_sync_rot;
    ivp_old_sync_rot_t old_sync_rot;

    int sim_unit;
    snapshot_span_t controllers;       /* controller refs (controllers_of_core) */
    snapshot_span_t infos;             /* friction info refs, in list order */
    snapshot_span_t contact_synapses;  /* contact refs (fr_synapses chain) */
    snapshot_span_t anchors;           /* anchor refs (anchors list) */

    /* collision state; the hull manager's list is in hull_listeners */
    ivp_hull_mgr_t hull;
    bool has_ov_element;
    ivp_vec3f_t ov_center;
    float ov_radius;
    ivp_ov_node_data_t ov_node;
    snapshot_span_t hull_listeners;    /* OV element or synapse refs */
    snapshot_span_t collisions;        /* mindist refs (OV element) */
    snapshot_span_t exact_synapses;    /* synapse refs */
    snapshot_span_t invalid_synapses;  /* synapse refs */
} snapshot_object_t;

typedef struct snapshot_mindist {
    ivp_mindist_t md;   /* value fields only (snapshot_restore_mindist) */
    int object[2];
    bool self_linked[2];
} snapshot_mindist_t;

typedef struct snapshot_sim_unit {
    uint8_t movement_type;
    bool union_find_needed;
    bool has_fast_objects;
    bool just_slowed_down;
    snapshot_span_t cores;        /* object refs */
    snapshot_span_t controllers;  /* controller ref, then its object refs */
} snapshot_sim_unit_t;

typedef struct snapshot_controller {
    const snapshot_controller_kind_t *kind;
    void *state;   /* captured copy, default allocator */
    int object[SNAPSHOT_CONTROLLER_OBJECTS];   /* -1: no object */
    int priority;
    ivp_controller_mode_t mode;
    ivp_controller_discovery_fn_t discovery;
    uint64_t registration_sequence;
    bool enabled;
} snapshot_controller_t;

typedef struct snapshot_contact {
    ivp_contact_pt_t cp;   /* value fields; links come from the lists */
    int object[2];
} snapshot_contact_t;

typedef struct snapshot_pair {
    ivp_fr_core_pair_t pair;   /* value fields */
    int system;
    int object[2];
    snapshot_span_t contacts;  /* contact refs (fr_dists) */
} snapshot_pair_t;

typedef struct snapshot_info {
    int system;
    snapshot_span_t springs;   /* contact refs (friction_springs) */
} snapshot_info_t;

typedef struct snapshot_system {
    ivp_fr_system_t fs;              /* values and controller sequences */
    snapshot_span_t contacts;        /* contact refs, friction dist list */
    snapshot_span_t cores;           /* object refs */
    snapshot_span_t moveable_cores;  /* object refs */
    snapshot_span_t pairs;           /* pair refs */
} snapshot_system_t;

struct ivp_world_snapshot {
    ivp_time_t current_time;
    ivp_time_t time_of_last_psi;
    ivp_time_t time_of_next_psi;
    ivp_time_t time_manager_base_time;
    double time_manager_last_time;
    ivp_time_code_t current_time_code;
    ivp_time_code_t mindist_event_timestamp_reference;
    short next_movement_check;
    int rand_seed;
    uint64_t next_object_sequence;
    uint64_t next_controller_sequence;
    uint64_t next_contact_sequence;
#if IVP_EDIT_LOCALITY
    uint64_t psi_index;
#endif
    int object_count;
    snapshot_object_t *objects;
    int mindist_count;
    snapshot_mindist_t *mindists;
    int ref_count;
    int ref_capacity;
    snapshot_ref_t *refs;
    int info_capacity;
    int sim_unit_count;
    snapshot_sim_unit_t *sim_units;
    int system_count;
    snapshot_system_t *systems;
    int contact_count;
    snapshot_contact_t *contacts;
    int pair_count;
    snapshot_pair_t *pairs;
    int info_count;
    snapshot_info_t *infos;
    int controller_count;
    snapshot_controller_t *controllers;
    snapshot_span_t events;           /* PSI event and mindist refs */
    snapshot_span_t exact_mindists;   /* mindist refs */
    snapshot_span_t invalid_mindists; /* mindist refs */
};

static void snapshot_set_result(ivp_result_t *result, ivp_result_t value)
{
    if (result) *result = value;
}

/* arrays of at least one element, so a zero count still allocates */
static size_t snapshot_bytes(size_t element_size, int count)
{
    return element_size * (size_t)(count > 0 ? count : 1);
}

static void *snapshot_alloc_zeroed(const ivp_allocator_t *allocator,
                                   size_t element_size, int count)
{
    size_t bytes = snapshot_bytes(element_size, count);
    void *memory = ivp_alloc(allocator, bytes);
    if (memory) memset(memory, 0, bytes);
    return memory;
}

static void snapshot_free_array(const ivp_allocator_t *allocator, void *memory,
                                size_t element_size, int count)
{
    if (memory) ivp_free(allocator, memory, snapshot_bytes(element_size, count));
}

/* doubles capacity; false leaves the array unchanged */
static bool snapshot_grow(void **memory, int *capacity, size_t element_size)
{
    int grown = *capacity ? *capacity * 2 : 64;
    const ivp_allocator_t *allocator = ivp_default_allocator();
    void *moved = *memory
        ? ivp_realloc(allocator, *memory, element_size * (size_t)*capacity,
                      element_size * (size_t)grown)
        : ivp_alloc(allocator, element_size * (size_t)grown);
    if (!moved) return false;
    *memory = moved;
    *capacity = grown;
    return true;
}

/* ── pointer lookup ──────────────────────────────────────────────────── */

/* Maps pointers to indices assigned in a canonical walk; the address
 * order only serves the binary search. */
typedef struct snapshot_lookup_entry {
    const void *pointer;
    int index;
} snapshot_lookup_entry_t;

typedef struct snapshot_lookup {
    snapshot_lookup_entry_t *entries;
    int count;
    int capacity;
} snapshot_lookup_t;

static int snapshot_compare_lookup(const void *left, const void *right)
{
    uintptr_t a = (uintptr_t)((const snapshot_lookup_entry_t *)left)->pointer;
    uintptr_t b = (uintptr_t)((const snapshot_lookup_entry_t *)right)->pointer;
    return a < b ? -1 : a > b ? 1 : 0;
}

static bool snapshot_lookup_add(snapshot_lookup_t *lookup, const void *pointer)
{
    if (lookup->count == lookup->capacity &&
        !snapshot_grow((void **)&lookup->entries, &lookup->capacity,
                       sizeof(*lookup->entries))) {
        return false;
    }
    lookup->entries[lookup->count].pointer = pointer;
    lookup->entries[lookup->count].index = -1;
    lookup->count++;
    return true;
}

/* sort and drop duplicates; indices stay unassigned */
static void snapshot_lookup_seal(snapshot_lookup_t *lookup)
{
    if (lookup->count == 0) return;
    qsort(lookup->entries, (size_t)lookup->count, sizeof(*lookup->entries),
          snapshot_compare_lookup);
    int unique = 1;
    for (int i = 1; i < lookup->count; i++) {
        if (lookup->entries[i].pointer != lookup->entries[unique - 1].pointer) {
            lookup->entries[unique++] = lookup->entries[i];
        }
    }
    lookup->count = unique;
}

static void snapshot_lookup_destroy(snapshot_lookup_t *lookup)
{
    if (lookup->entries) {
        ivp_free(ivp_default_allocator(), lookup->entries,
                 sizeof(*lookup->entries) * (size_t)lookup->capacity);
    }
}

static snapshot_lookup_entry_t *snapshot_lookup_find(
    const snapshot_lookup_t *lookup, const void *pointer)
{
    if (lookup->count == 0) return NULL;
    snapshot_lookup_entry_t key;
    key.pointer = pointer;
    key.index = -1;
    return (snapshot_lookup_entry_t *)bsearch(
        &key, lookup->entries, (size_t)lookup->count, sizeof(*lookup->entries),
        snapshot_compare_lookup);
}

static int snapshot_lookup_index(const snapshot_lookup_t *lookup,
                                 const void *pointer)
{
    const snapshot_lookup_entry_t *entry = snapshot_lookup_find(lookup, pointer);
    return entry ? entry->index : -1;
}

/* ── capture ─────────────────────────────────────────────────────────── */

typedef struct snapshot_capture {
    const ivp_environment_t *env;
    ivp_world_snapshot_t *snapshot;
    ivp_object_t **ordered;
    snapshot_lookup_t objects;
    snapshot_lookup_t mindists;
    const ivp_mindist_t **mindist_order;
    snapshot_lookup_t sim_units;
    snapshot_lookup_t systems;
    snapshot_lookup_t contacts;
    snapshot_lookup_t pairs;
    snapshot_lookup_t controllers;
    snapshot_lookup_t anchors;   /* index = controller * 2 + anchor */
    const ivp_controller_slot_t **slots;   /* in registration order */
    int slot_count;
    const void **sim_unit_order;
    const void **system_order;
    const void **contact_order;
    const void **pair_order;
    int info_capacity;
    int object_count;
    bool supported;
    bool out_of_memory;
} snapshot_capture_t;

static int snapshot_compare_objects(const void *left, const void *right)
{
    uint64_t a = ivp_object_order_key(*(const ivp_object_t *const *)left);
    uint64_t b = ivp_object_order_key(*(const ivp_object_t *const *)right);
    return a < b ? -1 : a > b ? 1 : 0;
}

static bool snapshot_object_supported(const ivp_environment_t *env,
                                      const ivp_object_t *object)
{
    const ivp_core_t *core = object->physical_core;
    return core && object->friction_core == core &&
           object->original_core == core &&
           ivp_pvec_len(&core->objects) == 1 &&
           !core->merged_core_which_replace_this_core &&
           !core->car_wheel && !object->q_core_f_object &&
           !object->controller_phantom &&
           object->hull_manager && core->sim_unit &&
           object->hull_manager->sorted_synapses.nonfinite_count == 0 &&
           object->parent_cluster == &env->pl->root_cluster;
}

static bool snapshot_mindist_supported(const ivp_environment_t *env,
                                       const ivp_mindist_t *md)
{
    return !md->is_recursive && md->mindist_function == IVP_MF_COLLISION &&
           !md->is_in_phantom_set &&
           md->coll.delegator == &env->collision_delegator_roots[0]->base;
}

static snapshot_ref_t *snapshot_add_ref(snapshot_capture_t *capture,
                                        snapshot_ref_kind_t kind, int index,
                                        int side, float value)
{
    ivp_world_snapshot_t *snapshot = capture->snapshot;
    if (snapshot->ref_count == snapshot->ref_capacity &&
        !snapshot_grow((void **)&snapshot->refs, &snapshot->ref_capacity,
                       sizeof(*snapshot->refs))) {
        capture->out_of_memory = true;
        return NULL;
    }
    snapshot_ref_t *ref = &snapshot->refs[snapshot->ref_count++];
    memset(ref, 0, sizeof(*ref));
    ref->kind = kind;
    ref->index = index;
    ref->side = side;
    ref->value = value;
    return ref;
}

static int snapshot_mindist_index(snapshot_capture_t *capture,
                                  const ivp_mindist_t *md)
{
    int index = snapshot_lookup_index(&capture->mindists, md);
    if (index < 0) capture->supported = false;
    return index;
}

/* a synapse of a captured mindist, as (mindist, side) */
static bool snapshot_add_synapse_ref(snapshot_capture_t *capture,
                                     const ivp_md_synapse_t *syn, float value)
{
    const ivp_mindist_t *md = syn->mindist;
    int index = snapshot_mindist_index(capture, md);
    if (index < 0) return false;
    return snapshot_add_ref(capture, SNAPSHOT_REF_SYNAPSE, index,
                            syn == &md->synapse[0] ? 0 : 1, value) != NULL;
}

/* Index every mindist in the canonical walk: objects in creation order,
 * each OV element's collisions in vector order. */
static bool snapshot_discover_mindists(snapshot_capture_t *capture, int count)
{
    for (int i = 0; i < count; i++) {
        const ivp_ov_element_t *el = capture->ordered[i]->ov_element;
        if (!el) continue;
        for (int c = 0; c < el->collision_fvector.n; c++) {
            const ivp_collision_t *coll = el->collision_fvector.elems[c];
            if (!ivp_collision_is_mindist(coll) ||
                !snapshot_mindist_supported(capture->env,
                                            (const ivp_mindist_t *)coll)) {
                capture->supported = false;
                return true;
            }
            if (!snapshot_lookup_add(&capture->mindists, coll)) return false;
        }
    }
    snapshot_lookup_seal(&capture->mindists);
    capture->mindist_order = (const ivp_mindist_t **)snapshot_alloc_zeroed(
        ivp_default_allocator(), sizeof(*capture->mindist_order),
        capture->mindists.count);
    if (!capture->mindist_order) return false;
    int assigned = 0;
    for (int i = 0; i < count; i++) {
        const ivp_ov_element_t *el = capture->ordered[i]->ov_element;
        if (!el) continue;
        for (int c = 0; c < el->collision_fvector.n; c++) {
            snapshot_lookup_entry_t *entry = snapshot_lookup_find(
                &capture->mindists, el->collision_fvector.elems[c]);
            if (entry->index < 0) {
                entry->index = assigned;
                capture->mindist_order[assigned++] =
                    (const ivp_mindist_t *)entry->pointer;
            }
        }
    }
    return true;
}

static void snapshot_capture_mindist(snapshot_capture_t *capture,
                                     const ivp_mindist_t *md,
                                     snapshot_mindist_t *out)
{
    out->md = *md;
    for (int side = 0; side < 2; side++) {
        const ivp_md_synapse_t *syn = &md->synapse[side];
        out->object[side] = snapshot_lookup_index(&capture->objects, syn->l_obj);
        if (out->object[side] < 0) capture->supported = false;
        out->self_linked[side] = syn->next == syn;
    }
}

static void snapshot_begin_span(const snapshot_capture_t *capture,
                                snapshot_span_t *span)
{
    span->first = capture->snapshot->ref_count;
    span->count = 0;
}

static void snapshot_end_span(const snapshot_capture_t *capture,
                              snapshot_span_t *span)
{
    span->count = capture->snapshot->ref_count - span->first;
}

static void snapshot_capture_collision(snapshot_capture_t *capture,
                                       const ivp_object_t *object,
                                       snapshot_object_t *out)
{
    const ivp_hull_mgr_t *hm = object->hull_manager;
    out->hull = *hm;
    memset(&out->hull.sorted_synapses, 0, sizeof(out->hull.sorted_synapses));

    const ivp_ov_element_t *el = object->ov_element;
    out->has_ov_element = el != NULL;
    if (el) {
        if (!el->node) {
            capture->supported = false;
            return;
        }
        out->ov_center = el->center;
        out->ov_radius = el->radius;
        out->ov_node = el->node->data;
    }

    snapshot_begin_span(capture, &out->hull_listeners);
    const ivp_minlist_t *ml = &hm->sorted_synapses;
    for (uint32_t i = ml->first_element; i != IVP_MINLIST_UNUSED;
         i = ml->elems[i].next) {
        const ivp_listener_hull_t *lh =
            (const ivp_listener_hull_t *)ml->elems[i].element;
        float value = ml->elems[i].value;
        if (ivp_listener_hull_is_ov_element(lh) && el &&
            lh == &el->lh && el->hull_manager == hm) {
            if (!snapshot_add_ref(capture, SNAPSHOT_REF_OV_ELEMENT, -1, 0,
                                  value)) {
                return;
            }
        } else if (ivp_listener_hull_is_md_synapse(lh)) {
            if (!snapshot_add_synapse_ref(capture,
                                          (const ivp_md_synapse_t *)lh,
                                          value)) {
                return;
            }
        } else {
            capture->supported = false;
            return;
        }
    }
    snapshot_end_span(capture, &out->hull_listeners);

    snapshot_begin_span(capture, &out->collisions);
    if (el) {
        for (int c = 0; c < el->collision_fvector.n; c++) {
            int index = snapshot_mindist_index(
                capture, (const ivp_mindist_t *)el->collision_fvector.elems[c]);
            if (index < 0 ||
                !snapshot_add_ref(capture, SNAPSHOT_REF_MINDIST, index, 0,
                                  0.0f)) {
                return;
            }
        }
    }
    snapshot_end_span(capture, &out->collisions);

    snapshot_begin_span(capture, &out->exact_synapses);
    for (const ivp_md_synapse_t *syn = object->exact_synapses; syn;
         syn = syn->next) {
        if (!snapshot_add_synapse_ref(capture, syn, 0.0f)) return;
    }
    snapshot_end_span(capture, &out->exact_synapses);

    snapshot_begin_span(capture, &out->invalid_synapses);
    for (const ivp_md_synapse_t *syn = object->invalid_synapses; syn;
         syn = syn->next) {
        if (!snapshot_add_synapse_ref(capture, syn, 0.0f)) return;
    }
    snapshot_end_span(capture, &out->invalid_synapses);
}

static void snapshot_capture_mindist_list(snapshot_capture_t *capture,
                                          const ivp_mindist_t *head,
                                          snapshot_span_t *span)
{
    snapshot_begin_span(capture, span);
    for (const ivp_mindist_t *md = head; md; md = md->next) {
        int index = snapshot_mindist_index(capture, md);
        if (index < 0 ||
            !snapshot_add_ref(capture, SNAPSHOT_REF_MINDIST, index, 0, 0.0f)) {
            return;
        }
    }
    snapshot_end_span(capture, span);
}

static void snapshot_capture_events(snapshot_capture_t *capture)
{
    const ivp_time_manager_t *tm = &capture->env->pl->time_manager;
    const ivp_minlist_t *ml = &tm->min_hash;
    snapshot_begin_span(capture, &capture->snapshot->events);
    if (ml->nonfinite_count != 0) {
        capture->supported = false;
        return;
    }
    for (uint32_t i = ml->first_element; i != IVP_MINLIST_UNUSED;
         i = ml->elems[i].next) {
        const ivp_time_event_t *ev = (const ivp_time_event_t *)ml->elems[i].element;
        float value = ml->elems[i].value;
        if (ev == &tm->psi_event) {
            if (!snapshot_add_ref(capture, SNAPSHOT_REF_PSI_EVENT, -1, 0,
                                  value)) {
                return;
            }
            continue;
        }
        /* mindist time events: the event is the first member */
        int index = snapshot_mindist_index(capture, (const ivp_mindist_t *)ev);
        if (index < 0 ||
            !snapshot_add_ref(capture, SNAPSHOT_REF_MINDIST, index, 0, value)) {
            return;
        }
    }
    snapshot_end_span(capture, &capture->snapshot->events);
}

static void snapshot_capture_object(const ivp_object_t *object,
                                    snapshot_object_t *out)
{
    const ivp_core_t *core = object->physical_core;
    out->creation_sequence = object->creation_sequence;
    out->movement_check_salt = object->movement_check_salt;
    out->surman = object->surman;
    out->type = object->type;
    out->material = object->material;
    out->no_collision_group = object->no_collision_group;
    out->collision_group = object->collision_group;
    out->layer_mask = object->layer_mask;
    out->name = object->name;
    out->client_data = object->client_data;
    memcpy(out->nocoll_group_ident, object->nocoll_group_ident,
           sizeof(out->nocoll_group_ident));
    out->extra_radius = object->extra_radius;
    out->collision_detection_enabled = object->collision_detection_enabled;
    out->object_movement_state = object->object_movement_state;
    out->ccd_enabled = object->ccd_enabled;
    out->ccd_speed_threshold = object->ccd_speed_threshold;
    out->shift_core_f_object = object->shift_core_f_object;
    out->shift_core_f_object_is_zero = object->shift_core_f_object_is_zero;

    out->core = *core;
    out->has_old_sync_rot = core->old_sync_rot != NULL;
    if (core->old_sync_rot) out->old_sync_rot = *core->old_sync_rot;

}

static bool snapshot_environment_supported(const ivp_environment_t *env)
{
    const ivp_env_pipeline_t *pl = env->pl;
    /* the PSI event must be queued; the controller system holds only
     * user and dependent controllers */
    return !pl->in_psi &&
           pl->time_manager.psi_event.index != IVP_MINLIST_UNUSED &&
           env->collision_delegator_root_count == 1 &&
           ivp_pvec_len(&pl->mindist_mgr.wheel_look_ahead_mindists) == 0;
}

/* ── capture: simulation units and friction ──────────────────────────── */

/* Number the pointers added to lookup in the order of `walk`, which lists
 * them with repeats; returns false when out of memory. */
static bool snapshot_number(snapshot_lookup_t *lookup, const void **walk,
                            int walk_count, const void ***order_out)
{
    snapshot_lookup_seal(lookup);
    const void **order = (const void **)snapshot_alloc_zeroed(
        ivp_default_allocator(), sizeof(*order), lookup->count);
    if (!order) return false;
    int assigned = 0;
    for (int i = 0; i < walk_count; i++) {
        snapshot_lookup_entry_t *entry = snapshot_lookup_find(lookup, walk[i]);
        if (entry->index < 0) {
            entry->index = assigned;
            order[assigned++] = entry->pointer;
        }
    }
    *order_out = order;
    return true;
}

/* growable list of pointers in walk order */
typedef struct snapshot_walk {
    const void **items;
    int count;
    int capacity;
} snapshot_walk_t;

static bool snapshot_walk_add(snapshot_walk_t *walk, snapshot_lookup_t *lookup,
                              const void *pointer)
{
    if (walk->count == walk->capacity &&
        !snapshot_grow((void **)&walk->items, &walk->capacity,
                       sizeof(*walk->items))) {
        return false;
    }
    walk->items[walk->count++] = pointer;
    return snapshot_lookup_add(lookup, pointer);
}

static void snapshot_walk_destroy(snapshot_walk_t *walk)
{
    if (walk->items) {
        ivp_free(ivp_default_allocator(), (void *)walk->items,
                 sizeof(*walk->items) * (size_t)walk->capacity);
    }
}

static int snapshot_core_object(snapshot_capture_t *capture,
                                const ivp_core_t *core)
{
    int index = -1;
    if (core && ivp_pvec_len(&core->objects) == 1) {
        index = snapshot_lookup_index(&capture->objects,
                                      ivp_pvec_at(&core->objects, 0));
    }
    if (index < 0) capture->supported = false;
    return index;
}

static int snapshot_index(snapshot_capture_t *capture,
                          const snapshot_lookup_t *lookup, const void *pointer)
{
    int index = snapshot_lookup_index(lookup, pointer);
    if (index < 0) capture->supported = false;
    return index;
}

/* the friction infos of a core, in list order */
static int snapshot_core_infos(const ivp_core_t *core,
                               const ivp_fr_info_for_core_t **out, int max)
{
    if (!core->physical_unmoveable) {
        const ivp_fr_info_for_core_t *info = (const ivp_fr_info_for_core_t *)
            core->core_friction_info.for_moveables.moveable_core_friction_info;
        if (!info) return 0;
        if (max > 0) out[0] = info;
        return 1;
    }
    /* unmoveable cores keep a list keyed by friction system (its first
     * member is the vector, see ivp_core_get_friction_info) */
    const ivp_pvec_t *list = (const ivp_pvec_t *)
        core->core_friction_info.for_unmoveables.l_friction_info_hash;
    if (!list) return 0;
    int count = ivp_pvec_len(list);
    for (int i = 0; i < count && i < max; i++) {
        out[i] = (const ivp_fr_info_for_core_t *)ivp_pvec_at(list, i);
    }
    return count;
}

/* Walk objects in creation order to number simulation units, friction
 * systems (through infos and contact chains), then contacts and pairs in
 * system order. */
static bool snapshot_discover_friction(snapshot_capture_t *capture, int count)
{
    snapshot_walk_t units = { NULL, 0, 0 };
    snapshot_walk_t systems = { NULL, 0, 0 };
    snapshot_walk_t contacts = { NULL, 0, 0 };
    snapshot_walk_t pairs = { NULL, 0, 0 };
    bool ok = true;
    for (int i = 0; i < count && ok; i++) {
        const ivp_object_t *object = capture->ordered[i];
        const ivp_core_t *core = object->physical_core;
        ok = snapshot_walk_add(&units, &capture->sim_units, core->sim_unit);
        const ivp_fr_info_for_core_t *infos[64];
        int info_count = snapshot_core_infos(core, infos, 64);
        if (info_count > 64) {
            capture->supported = false;
            info_count = 64;
        }
        for (int k = 0; k < info_count && ok; k++) {
            ok = snapshot_walk_add(&systems, &capture->systems,
                                   infos[k]->l_friction_system);
        }
        for (const ivp_fr_synapse_t *syn = object->fr_synapses; syn && ok;
             syn = syn->next) {
            if (!syn->contact_point->l_friction_system) {
                capture->supported = false;
                continue;
            }
            ok = snapshot_walk_add(&systems, &capture->systems,
                                   syn->contact_point->l_friction_system);
        }
    }
    ok = ok && snapshot_number(&capture->sim_units, units.items, units.count,
                               &capture->sim_unit_order) &&
         snapshot_number(&capture->systems, systems.items, systems.count,
                         &capture->system_order);
    for (int s = 0; ok && s < capture->systems.count; s++) {
        const ivp_fr_system_t *fs =
            (const ivp_fr_system_t *)capture->system_order[s];
        for (const ivp_contact_pt_t *cp = fs->first_friction_dist; cp && ok;
             cp = cp->next_dist_in_friction) {
            ok = snapshot_walk_add(&contacts, &capture->contacts, cp);
        }
        for (int p = 0; p < ivp_pvec_len(&fs->fr_pairs_of_objs) && ok; p++) {
            ok = snapshot_walk_add(&pairs, &capture->pairs,
                                   ivp_pvec_at(&fs->fr_pairs_of_objs, p));
        }
    }
    ok = ok && snapshot_number(&capture->contacts, contacts.items,
                               contacts.count, &capture->contact_order) &&
         snapshot_number(&capture->pairs, pairs.items, pairs.count,
                         &capture->pair_order);

    /* controllers a core may carry */
    if (ok) {
        const ivp_environment_t *env = capture->env;
        ok = snapshot_lookup_add(&capture->controllers,
                                 ivp_env_get_standard_gravity_controller(
                                     (struct ivp_environment *)env));
        for (int s = 0; ok && s < capture->systems.count; s++) {
            const ivp_fr_system_t *fs =
                (const ivp_fr_system_t *)capture->system_order[s];
            ok = snapshot_lookup_add(&capture->controllers, &fs->dyn_ctrl) &&
                 snapshot_lookup_add(&capture->controllers, &fs->static_ctrl) &&
                 snapshot_lookup_add(&capture->controllers, &fs->energy_ctrl);
        }
        for (int c = 0; ok && c < capture->slot_count; c++) {
            ok = snapshot_lookup_add(&capture->controllers,
                                     &capture->slots[c]->base);
        }
        if (ok) {
            snapshot_lookup_seal(&capture->controllers);
            snapshot_lookup_find(&capture->controllers,
                                 ivp_env_get_standard_gravity_controller(
                                     (struct ivp_environment *)env))->index =
                SNAPSHOT_CONTROLLER_GRAVITY;
            for (int s = 0; s < capture->systems.count; s++) {
                const ivp_fr_system_t *fs =
                    (const ivp_fr_system_t *)capture->system_order[s];
                snapshot_lookup_find(&capture->controllers, &fs->dyn_ctrl)->index =
                    SNAPSHOT_CONTROLLER_ID(s, 0);
                snapshot_lookup_find(&capture->controllers, &fs->static_ctrl)->index =
                    SNAPSHOT_CONTROLLER_ID(s, 1);
                snapshot_lookup_find(&capture->controllers, &fs->energy_ctrl)->index =
                    SNAPSHOT_CONTROLLER_ID(s, 2);
            }
            for (int c = 0; c < capture->slot_count; c++) {
                snapshot_lookup_find(&capture->controllers,
                                     &capture->slots[c]->base)->index =
                    SNAPSHOT_SLOT_ID(capture->systems.count, c);
            }
        }
    }
    snapshot_walk_destroy(&units);
    snapshot_walk_destroy(&systems);
    snapshot_walk_destroy(&contacts);
    snapshot_walk_destroy(&pairs);
    return ok;
}

static bool snapshot_add_contact_ref(snapshot_capture_t *capture,
                                     const ivp_contact_pt_t *cp, int side)
{
    int index = snapshot_index(capture, &capture->contacts, cp);
    return index >= 0 &&
           snapshot_add_ref(capture, SNAPSHOT_REF_CONTACT, index, side, 0.0f);
}

static bool snapshot_add_core_ref(snapshot_capture_t *capture,
                                  const ivp_core_t *core)
{
    int index = snapshot_core_object(capture, core);
    return index >= 0 &&
           snapshot_add_ref(capture, SNAPSHOT_REF_OBJECT, index, 0, 0.0f);
}

static bool snapshot_add_controller_ref(snapshot_capture_t *capture,
                                        const ivp_sim_ctrl_t *c, int count)
{
    int index = snapshot_index(capture, &capture->controllers, c);
    return index >= 0 &&
           snapshot_add_ref(capture, SNAPSHOT_REF_CONTROLLER, index, count,
                            0.0f);
}

static bool snapshot_capture_contact_list(snapshot_capture_t *capture,
                                          const ivp_pvec_t *list,
                                          snapshot_span_t *span)
{
    snapshot_begin_span(capture, span);
    for (int i = 0; i < ivp_pvec_len(list); i++) {
        if (!snapshot_add_contact_ref(
                capture, (const ivp_contact_pt_t *)ivp_pvec_at(list, i), 0)) {
            return false;
        }
    }
    snapshot_end_span(capture, span);
    return true;
}

static bool snapshot_capture_core_list(snapshot_capture_t *capture,
                                       const ivp_pvec_t *list,
                                       snapshot_span_t *span)
{
    snapshot_begin_span(capture, span);
    for (int i = 0; i < ivp_pvec_len(list); i++) {
        if (!snapshot_add_core_ref(capture,
                                   (const ivp_core_t *)ivp_pvec_at(list, i))) {
            return false;
        }
    }
    snapshot_end_span(capture, span);
    return true;
}

static bool snapshot_capture_sim_unit(snapshot_capture_t *capture,
                                      const ivp_sim_unit_t *su,
                                      snapshot_sim_unit_t *out)
{
    out->movement_type = su->sim_unit_movement_type;
    out->union_find_needed = su->union_find_needed_for_sim_unit;
    out->has_fast_objects = su->sim_unit_has_fast_objects;
    out->just_slowed_down = su->sim_unit_just_slowed_down;
    if (!su->manager) capture->supported = false;
    if (!snapshot_capture_core_list(capture, &su->sim_unit_cores, &out->cores)) {
        return false;
    }
    snapshot_begin_span(capture, &out->controllers);
    for (int i = 0; i < ivp_pvec_len(&su->controller_cores); i++) {
        const ivp_su_ctrl_list_t *cl =
            (const ivp_su_ctrl_list_t *)ivp_pvec_at(&su->controller_cores, i);
        if (!snapshot_add_controller_ref(capture, cl->l_controller,
                                         ivp_pvec_len(&cl->cores_controlled_by))) {
            return false;
        }
        for (int k = 0; k < ivp_pvec_len(&cl->cores_controlled_by); k++) {
            if (!snapshot_add_core_ref(
                    capture,
                    (const ivp_core_t *)ivp_pvec_at(&cl->cores_controlled_by, k))) {
                return false;
            }
        }
    }
    snapshot_end_span(capture, &out->controllers);
    return true;
}

static bool snapshot_capture_system(snapshot_capture_t *capture,
                                    const ivp_fr_system_t *fs,
                                    snapshot_system_t *out)
{
    out->fs = *fs;
    snapshot_begin_span(capture, &out->contacts);
    for (const ivp_contact_pt_t *cp = fs->first_friction_dist; cp;
         cp = cp->next_dist_in_friction) {
        if (!snapshot_add_contact_ref(capture, cp, 0)) return false;
    }
    snapshot_end_span(capture, &out->contacts);
    if (!snapshot_capture_core_list(capture, &fs->cores_of_friction_system,
                                    &out->cores) ||
        !snapshot_capture_core_list(capture,
                                    &fs->moveable_cores_of_friction_system,
                                    &out->moveable_cores)) {
        return false;
    }
    snapshot_begin_span(capture, &out->pairs);
    for (int p = 0; p < ivp_pvec_len(&fs->fr_pairs_of_objs); p++) {
        int index = snapshot_index(capture, &capture->pairs,
                                   ivp_pvec_at(&fs->fr_pairs_of_objs, p));
        if (index < 0 ||
            !snapshot_add_ref(capture, SNAPSHOT_REF_PAIR, index, 0, 0.0f)) {
            return false;
        }
    }
    snapshot_end_span(capture, &out->pairs);
    return true;
}

static bool snapshot_capture_object_friction(snapshot_capture_t *capture,
                                             const ivp_object_t *object,
                                             snapshot_object_t *out)
{
    ivp_world_snapshot_t *snapshot = capture->snapshot;
    const ivp_core_t *core = object->physical_core;
    out->sim_unit = snapshot_index(capture, &capture->sim_units, core->sim_unit);

    snapshot_begin_span(capture, &out->controllers);
    for (int i = 0; i < ivp_pvec_len(&core->controllers_of_core); i++) {
        if (!snapshot_add_controller_ref(
                capture,
                (const ivp_sim_ctrl_t *)ivp_pvec_at(&core->controllers_of_core, i),
                0)) {
            return false;
        }
    }
    snapshot_end_span(capture, &out->controllers);

    /* the infos' springs first, so the object's info refs stay contiguous */
    const ivp_fr_info_for_core_t *infos[64];
    int info_count = snapshot_core_infos(core, infos, 64);
    if (info_count > 64) {
        capture->supported = false;
        return true;
    }
    int first_info = snapshot->info_count;
    for (int k = 0; k < info_count; k++) {
        if (snapshot->info_count == capture->info_capacity &&
            !snapshot_grow((void **)&snapshot->infos, &capture->info_capacity,
                           sizeof(*snapshot->infos))) {
            return false;
        }
        snapshot_info_t *info = &snapshot->infos[snapshot->info_count++];
        memset(info, 0, sizeof(*info));
        info->system = snapshot_index(capture, &capture->systems,
                                      infos[k]->l_friction_system);
        snapshot_span_t springs;
        if (!snapshot_capture_contact_list(capture, &infos[k]->friction_springs,
                                           &springs)) {
            return false;
        }
        snapshot->infos[first_info + k].springs = springs;
    }
    snapshot_begin_span(capture, &out->infos);
    for (int k = 0; k < info_count; k++) {
        if (!snapshot_add_ref(capture, SNAPSHOT_REF_INFO, first_info + k, 0,
                              0.0f)) {
            return false;
        }
    }
    snapshot_end_span(capture, &out->infos);

    snapshot_begin_span(capture, &out->contact_synapses);
    for (const ivp_fr_synapse_t *syn = object->fr_synapses; syn; syn = syn->next) {
        const ivp_contact_pt_t *cp = syn->contact_point;
        if (!snapshot_add_contact_ref(capture, cp, syn == &cp->synapse[0] ? 0 : 1)) {
            return false;
        }
    }
    snapshot_end_span(capture, &out->contact_synapses);
    return true;
}

static int snapshot_compare_slots(const void *left, const void *right)
{
    uint64_t a = (*(const ivp_controller_slot_t *const *)left)->base.registration_sequence;
    uint64_t b = (*(const ivp_controller_slot_t *const *)right)->base.registration_sequence;
    return a < b ? -1 : a > b ? 1 : 0;
}

/* Registered controllers in registration order; every one must be a
 * built-in kind whose state can be copied. */
static bool snapshot_collect_slots(snapshot_capture_t *capture)
{
    const ivp_controller_system_t *cs =
        (const ivp_controller_system_t *)capture->env->controller_system;
    int count = ivp_controller_system_count(cs);
    capture->slots = (const ivp_controller_slot_t **)snapshot_alloc_zeroed(
        ivp_default_allocator(), sizeof(*capture->slots), count);
    if (!capture->slots) return false;
    capture->slot_count = count;
    for (int i = 0; i < count; i++) {
        const ivp_controller_slot_t *slot = ivp_controller_system_get_at(cs, i);
        capture->slots[i] = slot;
        const snapshot_controller_kind_t *kind =
            slot ? snapshot_find_kind(slot->event_fn) : NULL;
        if (!kind || !slot->descriptor || slot->associated_count != 0 ||
            slot->deleted_with_core || slot->run_for_all_cores ||
            !kind->capturable(slot->ctx)) {
            capture->supported = false;
        }
    }
    if (capture->supported) {
        qsort((void *)capture->slots, (size_t)count, sizeof(*capture->slots),
              snapshot_compare_slots);
    }
    return true;
}

static bool snapshot_capture_controllers(snapshot_capture_t *capture)
{
    ivp_world_snapshot_t *snapshot = capture->snapshot;
    const ivp_allocator_t *allocator = ivp_default_allocator();
    snapshot->controllers = (snapshot_controller_t *)snapshot_alloc_zeroed(
        allocator, sizeof(*snapshot->controllers), capture->slot_count);
    if (!snapshot->controllers) return false;
    snapshot->controller_count = capture->slot_count;
    for (int c = 0; c < capture->slot_count; c++) {
        const ivp_controller_slot_t *slot = capture->slots[c];
        snapshot_controller_t *out = &snapshot->controllers[c];
        out->kind = snapshot_find_kind(slot->event_fn);
        out->state = ivp_alloc(allocator, out->kind->state_size);
        if (!out->state) return false;
        memcpy(out->state, slot->ctx, out->kind->state_size);
        out->priority = slot->priority;
        out->mode = slot->mode;
        out->discovery = slot->discovery_fn;
        out->registration_sequence = slot->base.registration_sequence;
        out->enabled = slot->enabled;
        ivp_object_t **fields[SNAPSHOT_CONTROLLER_OBJECTS];
        int field_count = out->kind->objects(out->state, fields);
        for (int f = 0; f < SNAPSHOT_CONTROLLER_OBJECTS; f++) {
            out->object[f] = -1;
            if (f < field_count && *fields[f]) {
                out->object[f] = snapshot_index(capture, &capture->objects,
                                                *fields[f]);
            }
        }
        int anchor_count = 0;
        ivp_anchor_t *anchors = out->kind->anchors(slot->ctx, &anchor_count);
        for (int k = 0; k < anchor_count; k++) {
            if (!snapshot_lookup_add(&capture->anchors, &anchors[k])) return false;
        }
    }
    snapshot_lookup_seal(&capture->anchors);
    for (int c = 0; c < capture->slot_count; c++) {
        int anchor_count = 0;
        ivp_anchor_t *anchors = snapshot->controllers[c].kind->anchors(
            capture->slots[c]->ctx, &anchor_count);
        for (int k = 0; k < anchor_count; k++) {
            snapshot_lookup_find(&capture->anchors, &anchors[k])->index = c * 2 + k;
        }
    }
    return true;
}

static bool snapshot_capture_object_anchors(snapshot_capture_t *capture,
                                            const ivp_object_t *object,
                                            snapshot_object_t *out)
{
    snapshot_begin_span(capture, &out->anchors);
    for (const ivp_anchor_t *anchor = object->anchors; anchor;
         anchor = anchor->anchor_next_in_object) {
        int index = snapshot_index(capture, &capture->anchors, anchor);
        if (index < 0 ||
            !snapshot_add_ref(capture, SNAPSHOT_REF_ANCHOR, index / 2,
                              index % 2, 0.0f)) {
            return index < 0;
        }
    }
    snapshot_end_span(capture, &out->anchors);
    return true;
}

static bool snapshot_capture_friction(snapshot_capture_t *capture, int count)
{
    const ivp_allocator_t *allocator = ivp_default_allocator();
    ivp_world_snapshot_t *snapshot = capture->snapshot;
    if (!snapshot_discover_friction(capture, count)) return false;
    if (!capture->supported) return true;

    snapshot->sim_units = (snapshot_sim_unit_t *)snapshot_alloc_zeroed(
        allocator, sizeof(*snapshot->sim_units), capture->sim_units.count);
    if (!snapshot->sim_units) return false;
    snapshot->sim_unit_count = capture->sim_units.count;
    snapshot->systems = (snapshot_system_t *)snapshot_alloc_zeroed(
        allocator, sizeof(*snapshot->systems), capture->systems.count);
    if (!snapshot->systems) return false;
    snapshot->system_count = capture->systems.count;
    snapshot->contacts = (snapshot_contact_t *)snapshot_alloc_zeroed(
        allocator, sizeof(*snapshot->contacts), capture->contacts.count);
    if (!snapshot->contacts) return false;
    snapshot->contact_count = capture->contacts.count;
    snapshot->pairs = (snapshot_pair_t *)snapshot_alloc_zeroed(
        allocator, sizeof(*snapshot->pairs), capture->pairs.count);
    if (!snapshot->pairs) return false;
    snapshot->pair_count = capture->pairs.count;

    for (int u = 0; u < snapshot->sim_unit_count; u++) {
        if (!snapshot_capture_sim_unit(
                capture, (const ivp_sim_unit_t *)capture->sim_unit_order[u],
                &snapshot->sim_units[u])) {
            return false;
        }
    }
    for (int s = 0; s < snapshot->system_count; s++) {
        if (!snapshot_capture_system(
                capture, (const ivp_fr_system_t *)capture->system_order[s],
                &snapshot->systems[s])) {
            return false;
        }
    }
    for (int c = 0; c < snapshot->contact_count; c++) {
        const ivp_contact_pt_t *cp =
            (const ivp_contact_pt_t *)capture->contact_order[c];
        snapshot_contact_t *out = &snapshot->contacts[c];
        out->cp = *cp;
        for (int side = 0; side < 2; side++) {
            out->object[side] = snapshot_index(capture, &capture->objects,
                                               cp->synapse[side].l_obj);
        }
        if (snapshot_lookup_index(&capture->systems, cp->l_friction_system) < 0) {
            capture->supported = false;
        }
    }
    for (int p = 0; p < snapshot->pair_count; p++) {
        const ivp_fr_core_pair_t *pair =
            (const ivp_fr_core_pair_t *)capture->pair_order[p];
        snapshot_pair_t *out = &snapshot->pairs[p];
        out->pair = *pair;
        memset(&out->pair.fr_dists, 0, sizeof(out->pair.fr_dists));
        out->object[0] = snapshot_core_object(capture, pair->objs[0]);
        out->object[1] = snapshot_core_object(capture, pair->objs[1]);
        if (!snapshot_capture_contact_list(capture, &pair->fr_dists,
                                           &out->contacts)) {
            return false;
        }
    }
    /* systems own their pairs: find the owner of every pair */
    for (int s = 0; s < snapshot->system_count; s++) {
        const snapshot_span_t span = snapshot->systems[s].pairs;
        for (int k = 0; k < span.count; k++) {
            snapshot->pairs[snapshot->refs[span.first + k].index].system = s;
        }
    }
    for (int i = 0; i < count; i++) {
        if (!snapshot_capture_object_friction(capture, capture->ordered[i],
                                              &snapshot->objects[i]) ||
            !snapshot_capture_object_anchors(capture, capture->ordered[i],
                                             &snapshot->objects[i])) {
            return false;
        }
    }
    return true;
}

static void snapshot_capture_clock(const ivp_environment_t *env,
                                   ivp_world_snapshot_t *snapshot)
{
    const ivp_env_pipeline_t *pl = env->pl;
    snapshot->current_time = env->current_time;
    snapshot->time_of_last_psi = pl->time_of_last_psi;
    snapshot->time_of_next_psi = pl->time_of_next_psi;
    snapshot->time_manager_base_time = pl->time_manager.base_time;
    snapshot->time_manager_last_time = pl->time_manager.last_time;
    snapshot->current_time_code = pl->current_time_code;
    snapshot->mindist_event_timestamp_reference =
        pl->mindist_event_timestamp_reference;
    snapshot->next_movement_check = pl->next_movement_check;
    snapshot->rand_seed = pl->rand_seed;
    snapshot->next_object_sequence = pl->next_object_sequence;
    snapshot->next_controller_sequence = pl->next_controller_sequence;
    snapshot->next_contact_sequence = pl->next_contact_sequence;
#if IVP_EDIT_LOCALITY
    snapshot->psi_index = pl->psi_index;
#endif
}

/* fills capture->snapshot; returns false only when out of memory */
static bool snapshot_capture_world(snapshot_capture_t *capture)
{
    const ivp_environment_t *env = capture->env;
    ivp_world_snapshot_t *snapshot = capture->snapshot;
    const ivp_allocator_t *allocator = ivp_default_allocator();
    int count = ivp_environment_get_object_count(env);

    capture->object_count = count;
    capture->ordered = (ivp_object_t **)snapshot_alloc_zeroed(
        allocator, sizeof(*capture->ordered), count);
    if (!capture->ordered) return false;
    snapshot->objects = (snapshot_object_t *)snapshot_alloc_zeroed(
        allocator, sizeof(*snapshot->objects), count);
    if (!snapshot->objects) return false;
    snapshot->object_count = count;

    for (int i = 0; i < count; i++) {
        capture->ordered[i] = ivp_environment_get_object(env, i);
        if (!capture->ordered[i] ||
            !snapshot_object_supported(env, capture->ordered[i])) {
            capture->supported = false;
            return true;
        }
    }
    qsort(capture->ordered, (size_t)count, sizeof(*capture->ordered),
          snapshot_compare_objects);
    for (int i = 0; i < count; i++) {
        if (!snapshot_lookup_add(&capture->objects, capture->ordered[i])) {
            return false;
        }
    }
    snapshot_lookup_seal(&capture->objects);
    for (int i = 0; i < count; i++) {
        snapshot_lookup_find(&capture->objects, capture->ordered[i])->index = i;
    }

    if (!snapshot_discover_mindists(capture, count)) return false;
    if (!capture->supported) return true;

    snapshot->mindists = (snapshot_mindist_t *)snapshot_alloc_zeroed(
        allocator, sizeof(*snapshot->mindists), capture->mindists.count);
    if (!snapshot->mindists) return false;
    snapshot->mindist_count = capture->mindists.count;
    for (int i = 0; i < snapshot->mindist_count; i++) {
        snapshot_capture_mindist(capture, capture->mindist_order[i],
                                 &snapshot->mindists[i]);
    }

    for (int i = 0; i < count && capture->supported; i++) {
        snapshot_capture_object(capture->ordered[i], &snapshot->objects[i]);
        snapshot_capture_collision(capture, capture->ordered[i],
                                   &snapshot->objects[i]);
        if (capture->out_of_memory) return false;
    }
    if (!capture->supported) return true;
    if (!snapshot_collect_slots(capture)) return false;
    if (!capture->supported) return true;
    if (!snapshot_capture_controllers(capture)) return false;
    if (!snapshot_capture_friction(capture, count)) return false;
    if (!capture->supported) return true;
    const ivp_mindist_mgr_t *mm = &env->pl->mindist_mgr;
    snapshot_capture_mindist_list(capture, mm->exact_mindists,
                                  &snapshot->exact_mindists);
    snapshot_capture_mindist_list(capture, mm->invalid_mindists,
                                  &snapshot->invalid_mindists);
    snapshot_capture_events(capture);
    if (capture->out_of_memory) return false;
    snapshot_capture_clock(env, snapshot);
    return true;
}

ivp_world_snapshot_t *ivp_world_snapshot_capture(const ivp_environment_t *env,
                                                 ivp_result_t *result)
{
    if (!env || !env->pl) {
        snapshot_set_result(result, IVP_ERR_INVALID_ARG);
        return NULL;
    }
    if (!snapshot_environment_supported(env)) {
        snapshot_set_result(result, IVP_ERR_INVALID_STATE);
        return NULL;
    }
    const ivp_allocator_t *allocator = ivp_default_allocator();
    ivp_world_snapshot_t *snapshot = (ivp_world_snapshot_t *)
        snapshot_alloc_zeroed(allocator, sizeof(*snapshot), 1);
    if (!snapshot) {
        snapshot_set_result(result, IVP_ERR_OUT_OF_MEMORY);
        return NULL;
    }
    snapshot_capture_t capture;
    memset(&capture, 0, sizeof(capture));
    capture.env = env;
    capture.snapshot = snapshot;
    capture.supported = true;

    bool captured = snapshot_capture_world(&capture);
    snapshot_free_array(allocator, capture.ordered, sizeof(*capture.ordered),
                        capture.object_count);
    snapshot_free_array(allocator, (void *)capture.mindist_order,
                        sizeof(*capture.mindist_order), capture.mindists.count);
    snapshot_lookup_destroy(&capture.objects);
    snapshot_lookup_destroy(&capture.mindists);
    snapshot_free_array(allocator, (void *)capture.sim_unit_order,
                        sizeof(*capture.sim_unit_order), capture.sim_units.count);
    snapshot_free_array(allocator, (void *)capture.system_order,
                        sizeof(*capture.system_order), capture.systems.count);
    snapshot_free_array(allocator, (void *)capture.contact_order,
                        sizeof(*capture.contact_order), capture.contacts.count);
    snapshot_free_array(allocator, (void *)capture.pair_order,
                        sizeof(*capture.pair_order), capture.pairs.count);
    snapshot_lookup_destroy(&capture.sim_units);
    snapshot_lookup_destroy(&capture.systems);
    snapshot_lookup_destroy(&capture.contacts);
    snapshot_lookup_destroy(&capture.pairs);
    snapshot_lookup_destroy(&capture.controllers);
    snapshot_lookup_destroy(&capture.anchors);
    snapshot_free_array(allocator, (void *)capture.slots,
                        sizeof(*capture.slots), capture.slot_count);
    if (!captured || !capture.supported) {
        ivp_world_snapshot_destroy(snapshot);
        snapshot_set_result(result, captured ? IVP_ERR_INVALID_STATE
                                             : IVP_ERR_OUT_OF_MEMORY);
        return NULL;
    }
    snapshot_set_result(result, IVP_OK);
    return snapshot;
}

/* ── restore ─────────────────────────────────────────────────────────── */

/* the value fields of IVP_Core; links and owned containers stay */
static void snapshot_restore_core(ivp_core_t *core, const ivp_core_t *from)
{
    core->fast_piling_allowed = from->fast_piling_allowed;
    core->rot_inertias_are_equal = from->rot_inertias_are_equal;
    core->pinned = from->pinned;
    core->upper_limit_radius = from->upper_limit_radius;
    core->max_surface_deviation = from->max_surface_deviation;
    core->rot_inertia = from->rot_inertia;
    core->mass = from->mass;
    core->rot_speed_damp_factor = from->rot_speed_damp_factor;
    core->inv_rot_inertia = from->inv_rot_inertia;
    core->inv_mass = from->inv_mass;
    core->speed_damp_factor = from->speed_damp_factor;
    core->inv_object_diameter = from->inv_object_diameter;
    core->spin_clipping_enabled = from->spin_clipping_enabled;
    core->spin_clipping = from->spin_clipping;
    core->movement_state = from->movement_state;
    core->temporarily_unmovable = from->temporarily_unmovable;
    core->impacts_since_last_PSI = from->impacts_since_last_PSI;
    core->time_of_last_psi = from->time_of_last_psi;
    core->i_delta_time = from->i_delta_time;
    core->rot_speed_change = from->rot_speed_change;
    core->speed_change = from->speed_change;
    core->rot_speed = from->rot_speed;
    core->speed = from->speed;
    core->pos_world_f_core_last_psi = from->pos_world_f_core_last_psi;
    core->delta_world_f_core_psis = from->delta_world_f_core_psis;
    core->q_world_f_core_last_psi = from->q_world_f_core_last_psi;
    core->q_world_f_core_next_psi = from->q_world_f_core_next_psi;
    core->m_world_f_core_last_psi = from->m_world_f_core_last_psi;
    core->rotation_axis_world_space = from->rotation_axis_world_space;
    core->current_speed = from->current_speed;
    core->abs_omega = from->abs_omega;
    core->max_surface_rot_speed = from->max_surface_rot_speed;
    for (int i = 0; i < 2; i++) {
        core->time_of_calm_reference[i] = from->time_of_calm_reference[i];
        core->q_world_f_core_calm_reference[i] =
            from->q_world_f_core_calm_reference[i];
        core->position_world_f_core_calm_reference[i] =
            from->position_world_f_core_calm_reference[i];
    }
    core->mindist_event_already_done = from->mindist_event_already_done;
}

/* the value fields of IVP_Mindist; links come from the captured lists */
static void snapshot_restore_mindist(ivp_mindist_t *md, const ivp_mindist_t *from)
{
    md->coll_type = from->coll_type;
    md->synapse_sort_flag = from->synapse_sort_flag;
    md->is_in_phantom_set = from->is_in_phantom_set;
    md->mindist_function = from->mindist_function;
    md->recalc_result = from->recalc_result;
    md->disable_halfspace_optimization = from->disable_halfspace_optimization;
    md->mindist_status = from->mindist_status;
    md->coll_dist_selector = from->coll_dist_selector;
    md->coll_dist_decrease_counter = from->coll_dist_decrease_counter;
    md->sum_extra_radius = from->sum_extra_radius;
    md->len_numerator = from->len_numerator;
    md->contact_dot_diff_center = from->contact_dot_diff_center;
    md->sum_angular_hull_time = from->sum_angular_hull_time;
    md->contact_plane = from->contact_plane;
    md->recalc_time_stamp = from->recalc_time_stamp;
    md->last_visited_triangle = from->last_visited_triangle;
    md->coll.fvector_index[0] = from->coll.fvector_index[0];
    md->coll.fvector_index[1] = from->coll.fvector_index[1];
}

static void snapshot_restore_hull(ivp_hull_mgr_t *hm, const ivp_hull_mgr_t *from)
{
    hm->last_vpsi_time = from->last_vpsi_time;
    hm->gradient = from->gradient;
    hm->center_gradient = from->center_gradient;
    hm->hull_value_last_vpsi = from->hull_value_last_vpsi;
    hm->hull_center_value_last_vpsi = from->hull_center_value_last_vpsi;
    hm->hull_value_next_psi = from->hull_value_next_psi;
    hm->time_of_next_reset = from->time_of_next_reset;
}

static ivp_object_t *snapshot_restore_object(ivp_environment_t *env,
                                             const snapshot_object_t *from)
{
    const ivp_core_t *core = &from->core;
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = from->type;
    tmpl.mass = core->mass;
    tmpl.rot_inertia = core->rot_inertia;
    tmpl.rot_inertia_is_factor = false;
    tmpl.position = core->pos_world_f_core_last_psi;
    tmpl.orientation = core->q_world_f_core_last_psi;
    tmpl.material = from->material;
    tmpl.no_collision_group = from->no_collision_group;
    tmpl.name = from->name;
    tmpl.client_data = from->client_data;
    tmpl.physical_unmoveable = core->physical_unmoveable;
    if (from->type == IVP_OBJECT_BALL) {
        tmpl.ball_radius = (ivp_real_t)from->extra_radius;
    }
    tmpl.nocoll_group_ident =
        from->nocoll_group_ident[0] ? from->nocoll_group_ident : NULL;
    tmpl.enable_collision_detection = false;
    tmpl.start_in_simulation = false;

    ivp_object_t *object = ivp_environment_create_object(env, &tmpl,
                                                         from->surman);
    if (!object) return NULL;
    object->creation_sequence = from->creation_sequence;
    object->movement_check_salt = from->movement_check_salt;
    object->collision_group = from->collision_group;
    object->layer_mask = from->layer_mask;
    object->extra_radius = from->extra_radius;
    object->ccd_enabled = from->ccd_enabled;
    object->ccd_speed_threshold = from->ccd_speed_threshold;
    object->shift_core_f_object = from->shift_core_f_object;
    object->shift_core_f_object_is_zero = from->shift_core_f_object_is_zero;

    if (from->object_movement_state < IVP_MT_NOT_SIM) {
        ivp_object_ensure_in_simulation_now(object);
    }
    ivp_core_t *restored = object->physical_core;
    snapshot_restore_core(restored, core);
    if (from->has_old_sync_rot && restored->old_sync_rot) {
        *restored->old_sync_rot = from->old_sync_rot;
    }
    ivp_object_set_movement_state(
        object, (ivp_movement_type_t)from->object_movement_state);
    snapshot_restore_hull(object->hull_manager, &from->hull);
    return object;
}

/* Grow a minlist so that `extra` later adds take free slots and cannot
 * fail: add placeholders, then remove them. */
static bool snapshot_reserve_minlist(const ivp_allocator_t *allocator,
                                     ivp_minlist_t *ml, int extra)
{
    if (extra <= 0) return true;
    ivp_minlist_index_t *added = (ivp_minlist_index_t *)snapshot_alloc_zeroed(
        allocator, sizeof(*added), extra);
    if (!added) return false;
    int count = 0;
    bool ok = true;
    for (; count < extra; count++) {
        added[count] = ivp_minlist_add(ml, ml, 0.0f);
        if (added[count] == IVP_MINLIST_UNUSED) {
            ok = false;
            break;
        }
    }
    while (count > 0) ivp_minlist_remove(ml, added[--count]);
    snapshot_free_array(allocator, added, sizeof(*added), extra);
    return ok;
}

typedef struct snapshot_restore {
    ivp_environment_t *env;
    const ivp_world_snapshot_t *snapshot;
    ivp_object_t **objects;
    ivp_mindist_t **mindists;
    ivp_fr_system_t **systems;
    ivp_contact_pt_t **contacts;
    ivp_fr_info_for_core_t **infos;
    const ivp_world_snapshot_bindings_t *bindings;
    ivp_sim_ctrl_t **slots;   /* registered controllers' dispatch objects */
} snapshot_restore_t;

static ivp_md_synapse_t *snapshot_synapse(const snapshot_restore_t *restore,
                                          const snapshot_ref_t *ref)
{
    return &restore->mindists[ref->index]->synapse[ref->side];
}

/* rebuild a per-object synapse list in captured order */
static ivp_md_synapse_t *snapshot_link_synapses(const snapshot_restore_t *restore,
                                                snapshot_span_t span)
{
    const snapshot_ref_t *refs = restore->snapshot->refs + span.first;
    ivp_md_synapse_t *head = NULL;
    ivp_md_synapse_t *previous = NULL;
    for (int i = 0; i < span.count; i++) {
        ivp_md_synapse_t *syn = snapshot_synapse(restore, &refs[i]);
        syn->prev = previous;
        syn->next = NULL;
        if (previous) previous->next = syn;
        else head = syn;
        previous = syn;
    }
    return head;
}

static ivp_mindist_t *snapshot_link_mindists(const snapshot_restore_t *restore,
                                             snapshot_span_t span)
{
    const snapshot_ref_t *refs = restore->snapshot->refs + span.first;
    ivp_mindist_t *head = NULL;
    ivp_mindist_t *previous = NULL;
    for (int i = 0; i < span.count; i++) {
        ivp_mindist_t *md = restore->mindists[refs[i].index];
        md->prev = previous;
        md->next = NULL;
        if (previous) previous->next = md;
        else head = md;
        previous = md;
    }
    return head;
}

/* the element a captured sorted-list entry refers to */
typedef const void *(*snapshot_element_fn_t)(const snapshot_restore_t *restore,
                                             ivp_object_t *object,
                                             const snapshot_ref_t *ref);

/* refill ml in reverse captured order (a finite key goes in front of equal
 * keys) and check the result; allocates nothing beyond reserved slots */
static bool snapshot_rebuild_minlist(const snapshot_restore_t *restore,
                                     ivp_object_t *object, ivp_minlist_t *ml,
                                     snapshot_span_t span,
                                     snapshot_element_fn_t element,
                                     unsigned int *(*index_of)(void *element))
{
    const snapshot_ref_t *refs = restore->snapshot->refs + span.first;
    for (int k = span.count - 1; k >= 0; k--) {
        void *item = (void *)element(restore, object, &refs[k]);
        ivp_minlist_index_t index = ivp_minlist_add(ml, item, refs[k].value);
        *index_of(item) = index;
        if (index == IVP_MINLIST_UNUSED) return false;
    }
    uint32_t i = ml->first_element;
    for (int k = 0; k < span.count; k++) {
        if (i == IVP_MINLIST_UNUSED ||
            ml->elems[i].element != element(restore, object, &refs[k]) ||
            memcmp(&ml->elems[i].value, &refs[k].value, sizeof(float)) != 0) {
            return false;
        }
        i = ml->elems[i].next;
    }
    return i == IVP_MINLIST_UNUSED;
}

static const void *snapshot_hull_element(const snapshot_restore_t *restore,
                                         ivp_object_t *object,
                                         const snapshot_ref_t *ref)
{
    if (ref->kind == SNAPSHOT_REF_OV_ELEMENT) return &object->ov_element->lh;
    return &snapshot_synapse(restore, ref)->lh;
}

static unsigned int *snapshot_listener_index(void *element)
{
    return &((ivp_listener_hull_t *)element)->minlist_index;
}

static bool snapshot_rebuild_hull_list(const snapshot_restore_t *restore,
                                       ivp_object_t *object,
                                       snapshot_span_t span)
{
    const snapshot_ref_t *refs = restore->snapshot->refs + span.first;
    for (int k = 0; k < span.count; k++) {
        if (refs[k].kind == SNAPSHOT_REF_OV_ELEMENT) {
            object->ov_element->hull_manager = object->hull_manager;
        }
    }
    return snapshot_rebuild_minlist(restore, object,
                                    &object->hull_manager->sorted_synapses,
                                    span, snapshot_hull_element,
                                    snapshot_listener_index);
}

static const void *snapshot_event_element(const snapshot_restore_t *restore,
                                          ivp_object_t *object,
                                          const snapshot_ref_t *ref)
{
    (void)object;
    if (ref->kind == SNAPSHOT_REF_PSI_EVENT) {
        return &restore->env->pl->time_manager.psi_event;
    }
    return &restore->mindists[ref->index]->coll.te;
}

static unsigned int *snapshot_event_index(void *element)
{
    return &((ivp_time_event_t *)element)->index;
}

static bool snapshot_rebuild_events(const snapshot_restore_t *restore)
{
    return snapshot_rebuild_minlist(restore, NULL,
                                    &restore->env->pl->time_manager.min_hash,
                                    restore->snapshot->events,
                                    snapshot_event_element,
                                    snapshot_event_index);
}

/* Phases that can fail leave the world safe to destroy: OV elements first,
 * then unpublished mindists, then reserved capacity; linking allocates
 * nothing. */
static ivp_result_t snapshot_restore_collision(snapshot_restore_t *restore)
{
    ivp_environment_t *env = restore->env;
    const ivp_world_snapshot_t *snapshot = restore->snapshot;

    for (int i = 0; i < snapshot->object_count; i++) {
        const snapshot_object_t *from = &snapshot->objects[i];
        ivp_object_t *object = restore->objects[i];
        object->collision_detection_enabled = from->collision_detection_enabled;
        if (!from->has_ov_element) continue;
        ivp_ov_element_t *el = ivp_ov_element_create(object);
        if (!el) return IVP_ERR_OUT_OF_MEMORY;
        el->center = from->ov_center;
        el->radius = from->ov_radius;
        object->ov_element = el;
        ivp_result_t inserted = ivp_ov_tree_insert_ov_element_at(
            ivp_env_get_ov_tree(env), el, &from->ov_node);
        if (inserted != IVP_OK) return inserted;
    }

    ivp_collision_delegator_t *root = &env->collision_delegator_roots[0]->base;
    for (int m = 0; m < snapshot->mindist_count; m++) {
        restore->mindists[m] = ivp_mindist_create(env, root, false);
        if (!restore->mindists[m]) {
            while (m > 0) ivp_mindist_discard_unpublished(restore->mindists[--m]);
            return IVP_ERR_OUT_OF_MEMORY;
        }
    }
    bool reserved = snapshot_reserve_minlist(
        env->allocator, &env->pl->time_manager.min_hash, snapshot->events.count);
    for (int i = 0; i < snapshot->object_count && reserved; i++) {
        const snapshot_object_t *from = &snapshot->objects[i];
        ivp_object_t *object = restore->objects[i];
        reserved = snapshot_reserve_minlist(
            env->allocator, &object->hull_manager->sorted_synapses,
            from->hull_listeners.count);
        if (reserved && object->ov_element) {
            reserved = ivp_coll_fvector_reserve(
                &object->ov_element->collision_fvector, from->collisions.count);
        }
    }
    if (!reserved) {
        for (int m = 0; m < snapshot->mindist_count; m++) {
            ivp_mindist_discard_unpublished(restore->mindists[m]);
        }
        return IVP_ERR_OUT_OF_MEMORY;
    }

    for (int m = 0; m < snapshot->mindist_count; m++) {
        const snapshot_mindist_t *from = &snapshot->mindists[m];
        ivp_mindist_t *md = restore->mindists[m];
        for (int side = 0; side < 2; side++) {
            ivp_md_synapse_t *syn = &md->synapse[side];
            ivp_md_synapse_restore(md, side, restore->objects[from->object[side]],
                                   from->md.synapse[side].edge,
                                   from->md.synapse[side].status);
            if (from->self_linked[side]) syn->next = syn->prev = syn;
        }
    }
    for (int i = 0; i < snapshot->object_count; i++) {
        const snapshot_object_t *from = &snapshot->objects[i];
        ivp_object_t *object = restore->objects[i];
        object->exact_synapses =
            snapshot_link_synapses(restore, from->exact_synapses);
        object->invalid_synapses =
            snapshot_link_synapses(restore, from->invalid_synapses);
        const snapshot_ref_t *refs = snapshot->refs + from->collisions.first;
        for (int c = 0; c < from->collisions.count; c++) {
            (void)ivp_coll_fvector_add(&object->ov_element->collision_fvector,
                                       &restore->mindists[refs[c].index]->coll);
        }
    }
    /* values last: fvector adds assign backlinks, the captured ones win */
    for (int m = 0; m < snapshot->mindist_count; m++) {
        snapshot_restore_mindist(restore->mindists[m], &snapshot->mindists[m].md);
    }
    ivp_mindist_mgr_t *mm = &env->pl->mindist_mgr;
    mm->exact_mindists = snapshot_link_mindists(restore, snapshot->exact_mindists);
    mm->invalid_mindists =
        snapshot_link_mindists(restore, snapshot->invalid_mindists);

    bool matches = true;
    for (int i = 0; i < snapshot->object_count; i++) {
        matches = snapshot_rebuild_hull_list(restore, restore->objects[i],
                                             snapshot->objects[i].hull_listeners) &&
                  matches;
    }
    ivp_time_manager_remove_event(&env->pl->time_manager,
                                  &env->pl->time_manager.psi_event);
    env->pl->time_manager.psi_event.index = IVP_MINLIST_UNUSED;
    matches = snapshot_rebuild_events(restore) && matches;
    IVP_ASSERT(matches);
    return matches ? IVP_OK : IVP_ERR_INVALID_STATE;
}


/* ── restore: simulation units and friction ──────────────────────────── */

static ivp_core_t *snapshot_core(const snapshot_restore_t *restore, int object)
{
    return restore->objects[object]->physical_core;
}

static ivp_sim_ctrl_t *snapshot_controller(const snapshot_restore_t *restore,
                                           int id)
{
    if (id == SNAPSHOT_CONTROLLER_GRAVITY) {
        return ivp_env_get_standard_gravity_controller(restore->env);
    }
    int system_count = restore->snapshot->system_count;
    if (id >= SNAPSHOT_SLOT_ID(system_count, 0)) {
        return restore->slots[id - SNAPSHOT_SLOT_ID(system_count, 0)];
    }
    ivp_fr_system_t *fs = restore->systems[(id - 1) / 3];
    switch ((id - 1) % 3) {
        case 0: return &fs->dyn_ctrl;
        case 1: return &fs->static_ctrl;
        default: return &fs->energy_ctrl;
    }
}

static const snapshot_ref_t *snapshot_span_refs(const snapshot_restore_t *restore,
                                                snapshot_span_t span)
{
    return restore->snapshot->refs + span.first;
}

/* overwrite a vector with the captured order of the same elements */
static bool snapshot_order_cores(const snapshot_restore_t *restore,
                                 ivp_pvec_t *vector, snapshot_span_t span)
{
    if (ivp_pvec_len(vector) != span.count) return false;
    const snapshot_ref_t *refs = snapshot_span_refs(restore, span);
    for (int i = 0; i < span.count; i++) {
        vector->elems[i] = snapshot_core(restore, refs[i].index);
    }
    return true;
}

static bool snapshot_order_contacts(const snapshot_restore_t *restore,
                                    ivp_pvec_t *vector, snapshot_span_t span)
{
    if (ivp_pvec_len(vector) != span.count) return false;
    const snapshot_ref_t *refs = snapshot_span_refs(restore, span);
    for (int i = 0; i < span.count; i++) {
        vector->elems[i] = restore->contacts[refs[i].index];
    }
    return true;
}

static bool snapshot_fuse_sim_units(const snapshot_restore_t *restore)
{
    const ivp_world_snapshot_t *snapshot = restore->snapshot;
    for (int u = 0; u < snapshot->sim_unit_count; u++) {
        snapshot_span_t span = snapshot->sim_units[u].cores;
        const snapshot_ref_t *refs = snapshot_span_refs(restore, span);
        if (span.count == 0) return false;
        ivp_sim_unit_t *unit = snapshot_core(restore, refs[0].index)->sim_unit;
        bool fused = false;
        for (int k = 1; k < span.count; k++) {
            ivp_sim_unit_t *other = snapshot_core(restore, refs[k].index)->sim_unit;
            if (other == unit) continue;
            if (!ivp_sim_unit_fusion_simulation_unities(unit, other)) return false;
            ivp_sim_unit_delete(other);
            fused = true;
        }
        /* fusion leaves the controller lists to a later union-find when it
         * runs out of memory; a restore needs them now */
        if (fused && !ivp_sim_unit_sim_unit_calc_redundants(unit)) return false;
    }
    return true;
}

/* wiring in the order the engine builds it: systems, infos, cores, then
 * contacts, pairs and springs */
static ivp_result_t snapshot_wire_friction(snapshot_restore_t *restore)
{
    ivp_environment_t *env = restore->env;
    const ivp_world_snapshot_t *snapshot = restore->snapshot;
    for (int s = 0; s < snapshot->system_count; s++) {
        const ivp_fr_system_t *from = &snapshot->systems[s].fs;
        ivp_fr_system_t *fs = ivp_fr_system_create(env);
        if (!fs) return IVP_ERR_OUT_OF_MEMORY;
        fs->dyn_ctrl.registration_sequence = from->dyn_ctrl.registration_sequence;
        fs->static_ctrl.registration_sequence =
            from->static_ctrl.registration_sequence;
        fs->energy_ctrl.registration_sequence =
            from->energy_ctrl.registration_sequence;
        restore->systems[s] = fs;
    }
    for (int i = 0; i < snapshot->object_count; i++) {
        snapshot_span_t span = snapshot->objects[i].infos;
        const snapshot_ref_t *refs = snapshot_span_refs(restore, span);
        for (int k = 0; k < span.count; k++) {
            ivp_fr_info_for_core_t *info = ivp_fr_info_create(env->allocator);
            if (!info) return IVP_ERR_OUT_OF_MEMORY;
            info->l_friction_system =
                restore->systems[snapshot->infos[refs[k].index].system];
            if (!ivp_core_add_friction_info(snapshot_core(restore, i), info)) {
                ivp_fr_info_destroy(info);
                return IVP_ERR_OUT_OF_MEMORY;
            }
            restore->infos[refs[k].index] = info;
        }
    }
    for (int s = 0; s < snapshot->system_count; s++) {
        snapshot_span_t span = snapshot->systems[s].cores;
        const snapshot_ref_t *refs = snapshot_span_refs(restore, span);
        for (int k = 0; k < span.count; k++) {
            if (!ivp_fr_system_add_core_to_system(
                    restore->systems[s], snapshot_core(restore, refs[k].index))) {
                return IVP_ERR_OUT_OF_MEMORY;
            }
        }
    }
    for (int c = 0; c < snapshot->contact_count; c++) {
        const snapshot_contact_t *from = &snapshot->contacts[c];
        restore->contacts[c] = ivp_contact_pt_restore(
            env, &from->cp, restore->objects[from->object[0]],
            restore->objects[from->object[1]]);
        if (!restore->contacts[c]) return IVP_ERR_OUT_OF_MEMORY;
    }
    for (int s = 0; s < snapshot->system_count; s++) {
        snapshot_span_t span = snapshot->systems[s].contacts;
        const snapshot_ref_t *refs = snapshot_span_refs(restore, span);
        /* the system prepends, so add from the end */
        for (int k = span.count - 1; k >= 0; k--) {
            ivp_fr_system_add_dist_to_system(restore->systems[s],
                                             restore->contacts[refs[k].index]);
        }
    }
    for (int p = 0; p < snapshot->pair_count; p++) {
        const snapshot_pair_t *from = &snapshot->pairs[p];
        ivp_fr_core_pair_t *pair = ivp_fr_system_restore_pair(
            restore->systems[from->system], &from->pair,
            snapshot_core(restore, from->object[0]),
            snapshot_core(restore, from->object[1]));
        if (!pair) return IVP_ERR_OUT_OF_MEMORY;
        const snapshot_ref_t *refs = snapshot_span_refs(restore, from->contacts);
        for (int k = 0; k < from->contacts.count; k++) {
            if (ivp_pvec_add(&pair->fr_dists, restore->contacts[refs[k].index]) < 0) {
                return IVP_ERR_OUT_OF_MEMORY;
            }
        }
    }
    for (int n = 0; n < snapshot->info_count; n++) {
        snapshot_span_t span = snapshot->infos[n].springs;
        const snapshot_ref_t *refs = snapshot_span_refs(restore, span);
        for (int k = 0; k < span.count; k++) {
            if (!ivp_fr_info_insert_friction_dist(
                    restore->infos[n], restore->contacts[refs[k].index])) {
                return IVP_ERR_OUT_OF_MEMORY;
            }
        }
    }
    return IVP_OK;
}

/* Undo a failed snapshot_wire_friction: remove what it built so far, in
 * reverse, leaving no friction state for the world's teardown. */
static void snapshot_unwire_friction(snapshot_restore_t *restore)
{
    const ivp_world_snapshot_t *snapshot = restore->snapshot;
    for (int c = 0; c < snapshot->contact_count; c++) {
        ivp_contact_pt_t *cp = restore->contacts[c];
        if (!cp) continue;
        if (cp->l_friction_system) {
            ivp_fr_system_remove_dist_from_system(cp->l_friction_system, cp);
        }
        ivp_contact_pt_destroy(cp);
        restore->contacts[c] = NULL;
    }
    for (int s = 0; s < snapshot->system_count; s++) {
        ivp_fr_system_t *fs = restore->systems[s];
        if (!fs) continue;
        ivp_fr_system_discard_pairs(fs);
        while (ivp_pvec_len(&fs->cores_of_friction_system) > 0) {
            ivp_fr_system_remove_core_from_system(
                fs, (ivp_core_t *)ivp_pvec_at(&fs->cores_of_friction_system, 0));
        }
    }
    for (int i = 0; i < snapshot->object_count; i++) {
        snapshot_span_t span = snapshot->objects[i].infos;
        const snapshot_ref_t *refs = snapshot_span_refs(restore, span);
        for (int k = 0; k < span.count; k++) {
            ivp_fr_info_for_core_t *info = restore->infos[refs[k].index];
            if (!info) continue;
            ivp_core_delete_friction_info(snapshot_core(restore, i), info);
            restore->infos[refs[k].index] = NULL;
        }
    }
    for (int s = 0; s < snapshot->system_count; s++) {
        if (restore->systems[s]) ivp_fr_system_destroy(restore->systems[s]);
        restore->systems[s] = NULL;
    }
}

static bool snapshot_order_sim_unit(const snapshot_restore_t *restore,
                                    const snapshot_sim_unit_t *from)
{
    const snapshot_ref_t *cores = snapshot_span_refs(restore, from->cores);
    ivp_sim_unit_t *su = snapshot_core(restore, cores[0].index)->sim_unit;
    if (!snapshot_order_cores(restore, &su->sim_unit_cores, from->cores)) {
        return false;
    }
    const snapshot_ref_t *refs = snapshot_span_refs(restore, from->controllers);
    int position = 0;
    for (int r = 0; r < from->controllers.count; position++) {
        ivp_sim_ctrl_t *c = snapshot_controller(restore, refs[r].index);
        int count = refs[r].side;
        int found = -1;
        for (int j = position; j < ivp_pvec_len(&su->controller_cores); j++) {
            const ivp_su_ctrl_list_t *cl =
                (const ivp_su_ctrl_list_t *)ivp_pvec_at(&su->controller_cores, j);
            if (cl->l_controller == c) {
                found = j;
                break;
            }
        }
        if (found < 0) return false;
        void *moved = su->controller_cores.elems[found];
        su->controller_cores.elems[found] = su->controller_cores.elems[position];
        su->controller_cores.elems[position] = moved;
        ivp_su_ctrl_list_t *cl = (ivp_su_ctrl_list_t *)moved;
        if (ivp_pvec_len(&cl->cores_controlled_by) != count) return false;
        for (int k = 0; k < count; k++) {
            cl->cores_controlled_by.elems[k] =
                snapshot_core(restore, refs[r + 1 + k].index);
        }
        r += 1 + count;
    }
    if (position != ivp_pvec_len(&su->controller_cores)) return false;

    su->sim_unit_movement_type = from->movement_type;
    su->sim_unit_has_fast_objects = from->has_fast_objects;
    su->sim_unit_just_slowed_down = from->just_slowed_down;
    if (from->union_find_needed) {
        ivp_sim_unit_request_union_find(su);
    } else if (su->union_find_needed_for_sim_unit) {
        if (su->manager) su->manager->pending_union_find_count--;
        su->union_find_needed_for_sim_unit = false;
    }
    return true;
}

/* Copy every captured controller state into the caller's storage, point
 * it at the restored objects and register it again; the registration
 * sequence and enabled flag are the captured ones. */
static ivp_result_t snapshot_restore_controllers(snapshot_restore_t *restore)
{
    const ivp_world_snapshot_t *snapshot = restore->snapshot;
    const ivp_world_snapshot_bindings_t *bindings = restore->bindings;
    if (snapshot->controller_count == 0) return IVP_OK;
    if (!bindings || bindings->controller_count != snapshot->controller_count ||
        !bindings->controller_states) {
        return IVP_ERR_INVALID_ARG;
    }
    ivp_controller_system_t *cs =
        (ivp_controller_system_t *)restore->env->controller_system;
    for (int c = 0; c < snapshot->controller_count; c++) {
        const snapshot_controller_t *from = &snapshot->controllers[c];
        void *state = bindings->controller_states[c];
        if (!state) return IVP_ERR_INVALID_ARG;
        memcpy(state, from->state, from->kind->state_size);
        ivp_object_t **fields[SNAPSHOT_CONTROLLER_OBJECTS];
        int field_count = from->kind->objects(state, fields);
        for (int f = 0; f < field_count; f++) {
            *fields[f] = from->object[f] >= 0 ? restore->objects[from->object[f]]
                                              : NULL;
        }
        int anchor_count = 0;
        ivp_anchor_t *anchors = from->kind->anchors(state, &anchor_count);
        for (int k = 0; k < anchor_count; k++) {
            anchors[k].anchor_next_in_object = NULL;
            anchors[k].anchor_prev_in_object = NULL;
            anchors[k].core_pos = NULL;
            anchors[k].linked = false;
        }
        ivp_controller_handle_t handle =
            from->mode == IVP_CONTROLLER_DEPENDENT
                ? ivp_environment_announce_dependent_controller(
                      restore->env, from->priority, from->kind->fn, state,
                      from->discovery)
                : ivp_environment_add_controller(restore->env, from->priority,
                                                 from->kind->fn, state);
        ivp_controller_slot_t *slot =
            ivp_controller_system_get_by_handle(cs, handle);
        if (!slot) return IVP_ERR_OUT_OF_MEMORY;
        slot->base.registration_sequence = from->registration_sequence;
        slot->enabled = from->enabled;
        restore->slots[c] = &slot->base;
        if (bindings->controller_handles) bindings->controller_handles[c] = handle;
    }
    ivp_controller_system_sort(cs, ivp_build_uses_reference_internal_state());
    return IVP_OK;
}

static ivp_anchor_t *snapshot_anchor(const snapshot_restore_t *restore,
                                     const snapshot_ref_t *ref)
{
    const snapshot_controller_t *from = &restore->snapshot->controllers[ref->index];
    int count = 0;
    ivp_anchor_t *anchors =
        from->kind->anchors(restore->bindings->controller_states[ref->index], &count);
    return ref->side < count ? &anchors[ref->side] : NULL;
}

/* Put every friction list, vector and chain into the captured order and
 * copy the remaining values; allocates nothing. */
static bool snapshot_order_friction(const snapshot_restore_t *restore)
{
    const ivp_world_snapshot_t *snapshot = restore->snapshot;
    bool ok = true;
    for (int i = 0; i < snapshot->object_count; i++) {
        const snapshot_object_t *from = &snapshot->objects[i];
        ivp_object_t *object = restore->objects[i];
        const snapshot_ref_t *refs = snapshot_span_refs(restore, from->contact_synapses);
        ivp_fr_synapse_t *previous = NULL;
        object->fr_synapses = NULL;
        for (int k = 0; k < from->contact_synapses.count; k++) {
            ivp_fr_synapse_t *syn =
                &restore->contacts[refs[k].index]->synapse[refs[k].side];
            syn->prev = previous;
            syn->next = NULL;
            if (previous) previous->next = syn;
            else object->fr_synapses = syn;
            previous = syn;
        }
        ivp_core_t *core = object->physical_core;
        if (ivp_pvec_len(&core->controllers_of_core) != from->controllers.count) {
            ok = false;
            continue;
        }
        refs = snapshot_span_refs(restore, from->controllers);
        for (int k = 0; k < from->controllers.count; k++) {
            core->controllers_of_core.elems[k] =
                snapshot_controller(restore, refs[k].index);
        }
        refs = snapshot_span_refs(restore, from->anchors);
        ivp_anchor_t *previous_anchor = NULL;
        object->anchors = NULL;
        for (int k = 0; k < from->anchors.count; k++) {
            ivp_anchor_t *anchor = snapshot_anchor(restore, &refs[k]);
            if (!anchor || !anchor->linked) {
                ok = false;
                break;
            }
            anchor->anchor_prev_in_object = previous_anchor;
            anchor->anchor_next_in_object = NULL;
            if (previous_anchor) previous_anchor->anchor_next_in_object = anchor;
            else object->anchors = anchor;
            previous_anchor = anchor;
        }
    }
    for (int s = 0; s < snapshot->system_count; s++) {
        const snapshot_system_t *from = &snapshot->systems[s];
        ivp_fr_system_t *fs = restore->systems[s];
        ok = snapshot_order_cores(restore, &fs->cores_of_friction_system,
                                  from->cores) && ok;
        ok = snapshot_order_cores(restore, &fs->moveable_cores_of_friction_system,
                                  from->moveable_cores) && ok;
        fs->friction_obj_number = from->fs.friction_obj_number;
        fs->friction_dist_number = from->fs.friction_dist_number;
        fs->complex_not_necessary_number = from->fs.complex_not_necessary_number;
        fs->union_find_necessary = from->fs.union_find_necessary;
        fs->fr_sys_simulated = from->fs.fr_sys_simulated;
        fs->sum_energy_destroyed = from->fs.sum_energy_destroyed;
    }
    for (int p = 0; p < snapshot->pair_count; p++) {
        const snapshot_pair_t *from = &snapshot->pairs[p];
        ivp_fr_core_pair_t *pair = ivp_fr_system_find_pair_of_cores(
            restore->systems[from->system],
            snapshot_core(restore, from->object[0]),
            snapshot_core(restore, from->object[1]));
        ok = pair && snapshot_order_contacts(restore, &pair->fr_dists,
                                             from->contacts) && ok;
    }
    for (int u = 0; u < snapshot->sim_unit_count; u++) {
        ok = snapshot_order_sim_unit(restore, &snapshot->sim_units[u]) && ok;
    }
    return ok;
}

static ivp_result_t snapshot_restore_friction(snapshot_restore_t *restore)
{
    if (!snapshot_fuse_sim_units(restore)) return IVP_ERR_OUT_OF_MEMORY;
    ivp_result_t wired = snapshot_wire_friction(restore);
    if (wired != IVP_OK) {
        snapshot_unwire_friction(restore);
        return wired;
    }
    wired = snapshot_restore_controllers(restore);
    if (wired != IVP_OK) return wired;
    bool ordered = snapshot_order_friction(restore);
    IVP_ASSERT(ordered);
    return ordered ? IVP_OK : IVP_ERR_INVALID_STATE;
}

static void snapshot_restore_clock(ivp_environment_t *env,
                                   const ivp_world_snapshot_t *snapshot)
{
    ivp_env_pipeline_t *pl = env->pl;
    ivp_time_manager_t *tm = &pl->time_manager;
    tm->base_time = snapshot->time_manager_base_time;
    tm->last_time = snapshot->time_manager_last_time;
    env->current_time = snapshot->current_time;
    pl->time_of_last_psi = snapshot->time_of_last_psi;
    pl->time_of_next_psi = snapshot->time_of_next_psi;
}

static void snapshot_restore_counters(ivp_environment_t *env,
                                      const ivp_world_snapshot_t *snapshot)
{
    ivp_env_pipeline_t *pl = env->pl;
    pl->current_time_code = snapshot->current_time_code;
    pl->mindist_event_timestamp_reference =
        snapshot->mindist_event_timestamp_reference;
    pl->next_movement_check = snapshot->next_movement_check;
    pl->rand_seed = snapshot->rand_seed;
    pl->next_object_sequence = snapshot->next_object_sequence;
    pl->next_controller_sequence = snapshot->next_controller_sequence;
    pl->next_contact_sequence = snapshot->next_contact_sequence;
#if IVP_EDIT_LOCALITY
    pl->psi_index = snapshot->psi_index;
#endif
}

ivp_environment_t *ivp_world_snapshot_restore(
    const ivp_world_snapshot_t *snapshot,
    const ivp_environment_config_t *config,
    const ivp_world_snapshot_bindings_t *bindings,
    ivp_result_t *result)
{
    if (!snapshot) {
        snapshot_set_result(result, IVP_ERR_INVALID_ARG);
        return NULL;
    }
    ivp_environment_t *env = ivp_environment_create(config);
    if (!env) {
        snapshot_set_result(result, IVP_ERR_OUT_OF_MEMORY);
        return NULL;
    }
    /* the clock first: objects are created and revived at the captured
     * time */
    snapshot_restore_clock(env, snapshot);

    snapshot_restore_t restore;
    restore.env = env;
    restore.snapshot = snapshot;
    restore.bindings = bindings;
    restore.slots = (ivp_sim_ctrl_t **)snapshot_alloc_zeroed(
        env->allocator, sizeof(*restore.slots), snapshot->controller_count);
    restore.objects = (ivp_object_t **)snapshot_alloc_zeroed(
        env->allocator, sizeof(*restore.objects), snapshot->object_count);
    restore.mindists = (ivp_mindist_t **)snapshot_alloc_zeroed(
        env->allocator, sizeof(*restore.mindists), snapshot->mindist_count);
    restore.systems = (ivp_fr_system_t **)snapshot_alloc_zeroed(
        env->allocator, sizeof(*restore.systems), snapshot->system_count);
    restore.contacts = (ivp_contact_pt_t **)snapshot_alloc_zeroed(
        env->allocator, sizeof(*restore.contacts), snapshot->contact_count);
    restore.infos = (ivp_fr_info_for_core_t **)snapshot_alloc_zeroed(
        env->allocator, sizeof(*restore.infos), snapshot->info_count);
    ivp_result_t status = restore.objects && restore.mindists &&
                                  restore.systems && restore.contacts &&
                                  restore.infos && restore.slots
                              ? IVP_OK : IVP_ERR_OUT_OF_MEMORY;
    for (int i = 0; i < snapshot->object_count && status == IVP_OK; i++) {
        restore.objects[i] = snapshot_restore_object(env, &snapshot->objects[i]);
        if (!restore.objects[i]) status = IVP_ERR_OUT_OF_MEMORY;
    }
    if (status == IVP_OK) status = snapshot_restore_collision(&restore);
    if (status == IVP_OK) status = snapshot_restore_friction(&restore);
    snapshot_free_array(env->allocator, restore.mindists,
                        sizeof(*restore.mindists), snapshot->mindist_count);
    snapshot_free_array(env->allocator, restore.systems,
                        sizeof(*restore.systems), snapshot->system_count);
    snapshot_free_array(env->allocator, restore.contacts,
                        sizeof(*restore.contacts), snapshot->contact_count);
    snapshot_free_array(env->allocator, restore.infos,
                        sizeof(*restore.infos), snapshot->info_count);
    snapshot_free_array(env->allocator, restore.slots,
                        sizeof(*restore.slots), snapshot->controller_count);
    snapshot_free_array(env->allocator, restore.objects,
                        sizeof(*restore.objects), snapshot->object_count);
    if (status != IVP_OK) {
        ivp_environment_destroy(env);
        snapshot_set_result(result, status);
        return NULL;
    }
    snapshot_restore_counters(env, snapshot);
    snapshot_set_result(result, IVP_OK);
    return env;
}

int ivp_world_snapshot_object_count(const ivp_world_snapshot_t *snapshot)
{
    return snapshot ? snapshot->object_count : 0;
}

int ivp_world_snapshot_controller_count(const ivp_world_snapshot_t *snapshot)
{
    return snapshot ? snapshot->controller_count : 0;
}

ivp_controller_fn_t ivp_world_snapshot_controller_fn(
    const ivp_world_snapshot_t *snapshot, int index)
{
    if (!snapshot || index < 0 || index >= snapshot->controller_count) return NULL;
    return snapshot->controllers[index].kind->fn;
}

size_t ivp_world_snapshot_controller_state_size(
    const ivp_world_snapshot_t *snapshot, int index)
{
    if (!snapshot || index < 0 || index >= snapshot->controller_count) return 0;
    return snapshot->controllers[index].kind->state_size;
}

void ivp_world_snapshot_destroy(ivp_world_snapshot_t *snapshot)
{
    if (!snapshot) return;
    const ivp_allocator_t *allocator = ivp_default_allocator();
    if (snapshot->refs) {
        ivp_free(allocator, snapshot->refs,
                 sizeof(*snapshot->refs) * (size_t)snapshot->ref_capacity);
    }
    snapshot_free_array(allocator, snapshot->mindists,
                        sizeof(*snapshot->mindists), snapshot->mindist_count);
    snapshot_free_array(allocator, snapshot->sim_units,
                        sizeof(*snapshot->sim_units), snapshot->sim_unit_count);
    snapshot_free_array(allocator, snapshot->systems,
                        sizeof(*snapshot->systems), snapshot->system_count);
    snapshot_free_array(allocator, snapshot->contacts,
                        sizeof(*snapshot->contacts), snapshot->contact_count);
    snapshot_free_array(allocator, snapshot->pairs,
                        sizeof(*snapshot->pairs), snapshot->pair_count);
    if (snapshot->infos) {
        ivp_free(allocator, snapshot->infos,
                 sizeof(*snapshot->infos) * (size_t)snapshot->info_capacity);
    }
    for (int c = 0; snapshot->controllers && c < snapshot->controller_count; c++) {
        if (snapshot->controllers[c].state) {
            ivp_free(allocator, snapshot->controllers[c].state,
                     snapshot->controllers[c].kind->state_size);
        }
    }
    snapshot_free_array(allocator, snapshot->controllers,
                        sizeof(*snapshot->controllers), snapshot->controller_count);
    snapshot_free_array(allocator, snapshot->objects,
                        sizeof(*snapshot->objects), snapshot->object_count);
    snapshot_free_array(allocator, snapshot, sizeof(*snapshot), 1);
}
