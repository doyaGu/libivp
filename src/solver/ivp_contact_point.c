/* ivp_contact_point.c — port of IVP_Contact_Point / IVP_Synapse_Friction /
 * IVP_Friction_Manager / IVP_Friction_Info_For_Core
 *
 * Reference: ivp_intern/ivp_friction.cxx (contact point ctor/dtor, synapse friction),
 * ivp_intern/ivp_mindist_friction.cxx (recalc_friction_s_vals and p_calc_friction_*),
 * ivp_intern/ivp_impact.cxx (materials), ivp_physics/ivp_core.cxx (friction infos).
 */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/diagnostics/ivp_debug_manager.h>
#include <ivp/diagnostics/ivp_statistic_manager.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_surman.h>
#include "collision/ivp_cls_internal.h"
#include "collision/ivp_recheck_diagnostics.h"
#include "dynamics/ivp_env_internal.h"
#include "dynamics/ivp_env_hot.h"
#include "solver/ivp_friction_internal.h"

/* env->get_debug_manager() */
static inline ivp_debug_manager_t *cp_dm(struct ivp_environment *env) { return ivp_env_get_debug_manager(env); }

static void core_unmovable_core_debug_friction_hash(ivp_core_t *core);

#ifdef IVP_BENCHMARK_DIAGNOSTICS
/* Intrusive, process-global probes for serial benchmark runners only.  They
 * observe inputs and never skip engine work.  The normal build contains none
 * of this state or the diagnostic fields in ivp_contact_pt_t. */
typedef struct cp_fingerprint {
    uint64_t value[2];
} cp_fingerprint_t;

typedef struct cp_seen_state {
    uint64_t epoch;
    const void *owner;
    cp_fingerprint_t fingerprint;
} cp_seen_state_t;

enum { CP_DIAGNOSTIC_STATE_CAPACITY = 16384 };
static cp_seen_state_t cp_seen_object_poses[CP_DIAGNOSTIC_STATE_CAPACITY];
static cp_seen_state_t cp_seen_core_states[CP_DIAGNOSTIC_STATE_CAPACITY];
static cp_seen_state_t cp_seen_point_world[CP_DIAGNOSTIC_STATE_CAPACITY];
static cp_seen_state_t cp_seen_edge_world[CP_DIAGNOSTIC_STATE_CAPACITY];
static cp_seen_state_t cp_seen_face_normals[CP_DIAGNOSTIC_STATE_CAPACITY];
static cp_seen_state_t cp_seen_face_tangents[CP_DIAGNOSTIC_STATE_CAPACITY];
static ivp_time_t cp_seen_time;
static bool cp_seen_time_valid;
static uint64_t cp_seen_epoch = 1;
static uint64_t cp_diagnostic_generation = 1;

static cp_fingerprint_t cp_fingerprint_init(void)
{
    cp_fingerprint_t fingerprint = {{UINT64_C(1469598103934665603),
                                     UINT64_C(1099511628211)}};
    return fingerprint;
}

static void cp_fingerprint_bytes(cp_fingerprint_t *fingerprint, const void *data, size_t size)
{
    const unsigned char *bytes = (const unsigned char *)data;
    for (size_t i = 0; i < size; ++i) {
        fingerprint->value[0] ^= bytes[i];
        fingerprint->value[0] *= UINT64_C(1099511628211);
        fingerprint->value[1] ^= (uint64_t)bytes[i] + UINT64_C(0x9e3779b97f4a7c15);
        fingerprint->value[1] *= UINT64_C(0x100000001b3);
        fingerprint->value[1] ^= fingerprint->value[1] >> 29;
    }
}

static void cp_fingerprint_vec3r(cp_fingerprint_t *fingerprint, const ivp_vec3r_t *value)
{
    for (int i = 0; i < 3; ++i)
        cp_fingerprint_bytes(fingerprint, &value->k[i], sizeof(value->k[i]));
}

static void cp_fingerprint_vec3f(cp_fingerprint_t *fingerprint, const ivp_vec3f_t *value)
{
    for (int i = 0; i < 3; ++i)
        cp_fingerprint_bytes(fingerprint, &value->k[i], sizeof(value->k[i]));
}

static void cp_fingerprint_transform(cp_fingerprint_t *fingerprint,
                                     const ivp_transform_r_t *transform)
{
    for (int row = 0; row < 3; ++row) cp_fingerprint_vec3r(fingerprint, &transform->rot.rows[row]);
    cp_fingerprint_vec3r(fingerprint, &transform->pos);
}

static bool cp_fingerprint_equal(cp_fingerprint_t a, cp_fingerprint_t b)
{
    return a.value[0] == b.value[0] && a.value[1] == b.value[1];
}

static cp_fingerprint_t cp_core_state_fingerprint(const ivp_core_t *core)
{
    cp_fingerprint_t fingerprint = cp_fingerprint_init();
    cp_fingerprint_bytes(&fingerprint, &core->physical_unmoveable,
                         sizeof(core->physical_unmoveable));
    cp_fingerprint_transform(&fingerprint, &core->m_world_f_core_last_psi);
    if (!core->physical_unmoveable) {
        cp_fingerprint_vec3r(&fingerprint, &core->speed);
        cp_fingerprint_vec3f(&fingerprint, &core->rot_speed);
        cp_fingerprint_vec3r(&fingerprint, &core->inv_rot_inertia);
        cp_fingerprint_bytes(&fingerprint, &core->inv_mass, sizeof(core->inv_mass));
    }
    return fingerprint;
}

static bool cp_seen_state_insert(cp_seen_state_t *states, const void *owner,
                                 cp_fingerprint_t fingerprint)
{
    uintptr_t address = (uintptr_t)owner;
    uint64_t hash = (uint64_t)address ^ fingerprint.value[0] ^
                    (fingerprint.value[1] * UINT64_C(0x9e3779b97f4a7c15));
    size_t slot = (size_t)(hash & (CP_DIAGNOSTIC_STATE_CAPACITY - 1));
    for (size_t probe = 0; probe < CP_DIAGNOSTIC_STATE_CAPACITY; ++probe) {
        cp_seen_state_t *state = &states[slot];
        if (state->epoch != cp_seen_epoch) {
            state->epoch = cp_seen_epoch;
            state->owner = owner;
            state->fingerprint = fingerprint;
            return true;
        }
        if (state->owner == owner && cp_fingerprint_equal(state->fingerprint, fingerprint))
            return false;
        slot = (slot + 1) & (CP_DIAGNOSTIC_STATE_CAPACITY - 1);
    }
    ivp_recheck_diagnostics_mutable()->contact_tracking_overflows++;
    return true;
}

static void cp_seen_state_next_epoch(void)
{
    cp_seen_epoch++;
    if (cp_seen_epoch) return;
    memset(cp_seen_object_poses, 0, sizeof(cp_seen_object_poses));
    memset(cp_seen_core_states, 0, sizeof(cp_seen_core_states));
    memset(cp_seen_point_world, 0, sizeof(cp_seen_point_world));
    memset(cp_seen_edge_world, 0, sizeof(cp_seen_edge_world));
    memset(cp_seen_face_normals, 0, sizeof(cp_seen_face_normals));
    memset(cp_seen_face_tangents, 0, sizeof(cp_seen_face_tangents));
    cp_seen_epoch = 1;
}

void ivp_contact_recalc_diagnostics_reset(void)
{
    cp_seen_state_next_epoch();
    cp_seen_time_valid = false;
    cp_diagnostic_generation++;
    if (!cp_diagnostic_generation) cp_diagnostic_generation = 1;
}

static void cp_record_feature_state(cp_seen_state_t *states, const ivp_object_t *object,
                                    const void *feature, cp_fingerprint_t pose,
                                    uint64_t *uses, uint64_t *first_uses, uint64_t *reuses)
{
    cp_fingerprint_bytes(&pose, &feature, sizeof(feature));
    (*uses)++;
    if (cp_seen_state_insert(states, object, pose)) (*first_uses)++;
    else (*reuses)++;
}

static void cp_contact_recalc_diagnostics_record(ivp_contact_pt_t *cp,
                                                 const ivp_cache_obj_t *cache0,
                                                 const ivp_cache_obj_t *cache1,
                                                 ivp_time_t current_time)
{
    ivp_recheck_diagnostics_t *counts = ivp_recheck_diagnostics_mutable();
    ivp_fr_synapse_t *syn0 = &cp->synapse[0];
    ivp_fr_synapse_t *syn1 = &cp->synapse[1];
    if (cp->diagnostic_generation != cp_diagnostic_generation) {
        cp->diagnostic_generation = cp_diagnostic_generation;
        cp->diagnostic_recalc_calls = 0;
        cp->diagnostic_calls_at_time = 0;
    }
    counts->contact_recalc_calls++;
    if ((syn0->status == IVP_ST_BALL || syn0->status == IVP_ST_POINT) &&
        (syn1->status == IVP_ST_BALL || syn1->status == IVP_ST_POINT))
        counts->contact_branch_pp++;
    else if (syn0->status == IVP_ST_EDGE)
        counts->contact_branch_kk++;
    else if (syn1->status == IVP_ST_EDGE)
        counts->contact_branch_pk++;
    else
        counts->contact_branch_pf++;

    /* These two-word fingerprints are equality detectors for consecutive
     * calls on one contact.  They are not persisted or treated as hashes of
     * the simulation state. */
    cp_fingerprint_t geometry = cp_fingerprint_init();
    cp_fingerprint_bytes(&geometry, &syn0->status, sizeof(syn0->status));
    cp_fingerprint_bytes(&geometry, &syn1->status, sizeof(syn1->status));
    cp_fingerprint_bytes(&geometry, &syn0->edge, sizeof(syn0->edge));
    cp_fingerprint_bytes(&geometry, &syn1->edge, sizeof(syn1->edge));
    cp_fingerprint_transform(&geometry, &cache0->m_world_f_object);
    cp_fingerprint_transform(&geometry, &cache1->m_world_f_object);
    cp_fingerprint_bytes(&geometry, &syn0->l_obj->extra_radius,
                         sizeof(syn0->l_obj->extra_radius));
    cp_fingerprint_bytes(&geometry, &syn1->l_obj->extra_radius,
                         sizeof(syn1->l_obj->extra_radius));
    cp_fingerprint_bytes(&geometry, &cp->cp_status, sizeof(cp->cp_status));
    cp_fingerprint_bytes(&geometry, &cp->inv_triangle_det, sizeof(cp->inv_triangle_det));

    cp_fingerprint_t core_state[2] = {
        cp_core_state_fingerprint(syn0->l_obj->physical_core),
        cp_core_state_fingerprint(syn1->l_obj->physical_core),
    };
    cp_fingerprint_t kinematic = geometry;
    cp_fingerprint_bytes(&kinematic, core_state[0].value, sizeof(core_state[0].value));
    cp_fingerprint_bytes(&kinematic, core_state[1].value, sizeof(core_state[1].value));
    cp_fingerprint_t full_input = kinematic;
    cp_fingerprint_bytes(&full_input, &current_time, sizeof(current_time));
    cp_fingerprint_bytes(&full_input, &cp->last_time_of_recalc_friction_s_vals,
                         sizeof(cp->last_time_of_recalc_friction_s_vals));
    cp_fingerprint_bytes(&full_input, cp->span_friction_s, sizeof(cp->span_friction_s));
    const ivp_mindist_settings_t *mindist_settings =
        ivp_env_get_mindist_settings(syn0->l_obj->owner);
    cp_fingerprint_bytes(&full_input, &mindist_settings->keeper_dist,
                         sizeof(mindist_settings->keeper_dist));

    if (cp->diagnostic_recalc_calls) {
        cp_fingerprint_t previous_geometry = {{cp->diagnostic_geometry_fingerprint[0],
                                               cp->diagnostic_geometry_fingerprint[1]}};
        cp_fingerprint_t previous_kinematic = {{cp->diagnostic_kinematic_fingerprint[0],
                                                cp->diagnostic_kinematic_fingerprint[1]}};
        cp_fingerprint_t previous_full = {{cp->diagnostic_full_input_fingerprint[0],
                                           cp->diagnostic_full_input_fingerprint[1]}};
        if (cp_fingerprint_equal(geometry, previous_geometry))
            counts->contact_same_geometry_fingerprint++;
        if (cp_fingerprint_equal(kinematic, previous_kinematic))
            counts->contact_same_kinematic_fingerprint++;
        if (cp_fingerprint_equal(full_input, previous_full))
            counts->contact_same_full_input_fingerprint++;
        if (current_time == cp->diagnostic_last_recalc_time) {
            counts->contact_same_time_calls++;
            cp->diagnostic_calls_at_time++;
        } else {
            cp->diagnostic_calls_at_time = 1;
        }
    } else {
        cp->diagnostic_calls_at_time = 1;
    }
    if (cp->diagnostic_calls_at_time > counts->contact_max_calls_per_contact_at_time)
        counts->contact_max_calls_per_contact_at_time = cp->diagnostic_calls_at_time;
    cp->diagnostic_recalc_calls++;
    cp->diagnostic_last_recalc_time = current_time;
    memcpy(cp->diagnostic_geometry_fingerprint, geometry.value, sizeof(geometry.value));
    memcpy(cp->diagnostic_kinematic_fingerprint, kinematic.value, sizeof(kinematic.value));
    memcpy(cp->diagnostic_full_input_fingerprint, full_input.value, sizeof(full_input.value));

    if (!cp_seen_time_valid || current_time != cp_seen_time) {
        cp_seen_state_next_epoch();
        cp_seen_time = current_time;
        cp_seen_time_valid = true;
    }
    const ivp_cache_obj_t *caches[2] = {cache0, cache1};
    cp_fingerprint_t poses[2];
    /* Object-pose reuse is shared input, not duplicate cache construction:
     * ivp_object_get_cache_object already owns that cache.  Feature counters
     * below identify transforms still repeated by this contact routine. */
    for (int i = 0; i < 2; ++i) {
        poses[i] = cp_fingerprint_init();
        cp_fingerprint_transform(&poses[i], &caches[i]->m_world_f_object);
        counts->contact_side_visits++;
        if (cp_seen_state_insert(cp_seen_object_poses, cp->synapse[i].l_obj, poses[i]))
            counts->contact_object_pose_first_uses++;
        else
            counts->contact_object_pose_reuses++;
        if (cp_seen_state_insert(cp_seen_core_states,
                                 cp->synapse[i].l_obj->physical_core, core_state[i]))
            counts->contact_core_state_first_uses++;
        else
            counts->contact_core_state_reuses++;
    }
    if (syn0->status == IVP_ST_POINT)
        cp_record_feature_state(cp_seen_point_world, syn0->l_obj, syn0->edge, poses[0],
                                &counts->contact_point_world_uses,
                                &counts->contact_point_world_first_uses,
                                &counts->contact_point_world_reuses);
    if (syn1->status == IVP_ST_POINT)
        cp_record_feature_state(cp_seen_point_world, syn1->l_obj, syn1->edge, poses[1],
                                &counts->contact_point_world_uses,
                                &counts->contact_point_world_first_uses,
                                &counts->contact_point_world_reuses);
    if (syn0->status == IVP_ST_EDGE) {
        cp_record_feature_state(cp_seen_edge_world, syn0->l_obj, syn0->edge, poses[0],
                                &counts->contact_edge_world_uses,
                                &counts->contact_edge_world_first_uses,
                                &counts->contact_edge_world_reuses);
        cp_record_feature_state(cp_seen_edge_world, syn1->l_obj, syn1->edge, poses[1],
                                &counts->contact_edge_world_uses,
                                &counts->contact_edge_world_first_uses,
                                &counts->contact_edge_world_reuses);
    } else if (syn1->status == IVP_ST_EDGE) {
        cp_record_feature_state(cp_seen_edge_world, syn1->l_obj, syn1->edge, poses[1],
                                &counts->contact_edge_world_uses,
                                &counts->contact_edge_world_first_uses,
                                &counts->contact_edge_world_reuses);
    } else if (syn1->status == IVP_ST_TRIANGLE) {
        const void *triangle = ivp_edge_get_triangle(syn1->edge);
        cp_record_feature_state(cp_seen_face_normals, syn1->l_obj, triangle, poses[1],
                                &counts->contact_face_normal_uses,
                                &counts->contact_face_normal_first_uses,
                                &counts->contact_face_normal_reuses);
        cp_record_feature_state(cp_seen_face_tangents, syn1->l_obj, syn1->edge, poses[1],
                                &counts->contact_face_tangent_uses,
                                &counts->contact_face_tangent_first_uses,
                                &counts->contact_face_tangent_reuses);
    }
}
#endif

/* ── IVP_Synapse_Friction ───────────────────────────────────────────────── */

void ivp_friction_mgr_restore_object_contact_order(ivp_object_t *obj)
{
    if (!obj || !obj->fr_synapses ||
        obj->fr_synapses->contact_point->creation_sequence == 0u ||
        !obj->fr_synapses->next) {
        return;
    }

    ivp_fr_synapse_t *previous = NULL;
    uint64_t previous_key = 0u;
    bool already_ordered = true;
    for (ivp_fr_synapse_t *synapse = obj->fr_synapses;
         synapse; synapse = synapse->next) {
        uint64_t key = synapse->contact_point->creation_sequence;
        if (synapse->prev != previous || key == 0u ||
            (previous && previous_key >= key)) {
            already_ordered = false;
            break;
        }
        previous = synapse;
        previous_key = key;
    }
    if (already_ordered) return;

    ivp_fr_synapse_t *unsorted = obj->fr_synapses;
    ivp_fr_synapse_t *sorted = NULL;
    while (unsorted) {
        ivp_fr_synapse_t *synapse = unsorted;
        unsorted = unsorted->next;
        uint64_t key = synapse->contact_point->creation_sequence;
        IVP_ASSERT(key != 0u);

        ivp_fr_synapse_t *before = NULL;
        ivp_fr_synapse_t *after = sorted;
        while (after &&
               after->contact_point->creation_sequence < key) {
            before = after;
            after = after->next;
        }
        IVP_ASSERT(!after ||
                   after->contact_point->creation_sequence != key);
        synapse->prev = before;
        synapse->next = after;
        if (before) {
            before->next = synapse;
        } else {
            sorted = synapse;
        }
        if (after) after->prev = synapse;
    }
    obj->fr_synapses = sorted;
}

static void fr_synapse_init(ivp_fr_synapse_t *syn, ivp_contact_pt_t *cp, ivp_object_t *obj,
                            const ivp_compact_edge_t *edge, short status)
{
    syn->l_obj = obj;
    syn->contact_point = cp;
    syn->status = status;
    syn->edge = edge;
    if (cp->creation_sequence == 0u || !obj->fr_synapses) {
        syn->next = obj->fr_synapses;
        syn->prev = NULL;
        if (syn->next) syn->next->prev = syn;
        obj->fr_synapses = syn;
        return;
    }

    uint64_t key = cp->creation_sequence;
    ivp_fr_synapse_t *before = NULL;
    ivp_fr_synapse_t *after = obj->fr_synapses;
    while (after && after->contact_point->creation_sequence < key) {
        before = after;
        after = after->next;
    }
    IVP_ASSERT(!after ||
               after->contact_point->creation_sequence != key);
    syn->prev = before;
    syn->next = after;
    if (before) {
        before->next = syn;
    } else {
        obj->fr_synapses = syn;
    }
    if (after) after->prev = syn;
}

static void fr_synapse_remove_from_object(ivp_fr_synapse_t *syn)
{
    if (syn->next) syn->next->prev = syn->prev;
    if (syn->prev) {
        syn->prev->next = syn->next;
    } else {
        syn->l_obj->fr_synapses = syn->next;
    }
}

static int fr_synapse_get_material_index(const ivp_fr_synapse_t *syn)
{
    return (int)ivp_tri_get_material_index(ivp_edge_get_triangle(syn->edge));
}

/* IVP_Synapse_Friction::is_same_as */
static bool fr_synapse_is_same_as(const ivp_fr_synapse_t *syn0, const ivp_md_synapse_t *syn1)
{
    IVP_ASSERT(syn0->l_obj == syn1->l_obj);
    short syn_stat = syn0->status;
    if (syn_stat != syn1->status) return false;
    switch (syn_stat) {
    case IVP_ST_POINT: {
        const ivp_compact_ledge_t *l0 = ivp_edge_get_compact_ledge(syn0->edge);
        const ivp_compact_ledge_t *l1 = ivp_edge_get_compact_ledge(syn1->edge);
        if (l0 != l1) return false;
        if (ivp_edge_get_start_point_index(syn0->edge) != ivp_edge_get_start_point_index(syn1->edge)) {
            return false;
        }
        return true;
    }
    case IVP_ST_TRIANGLE:
        return ivp_edge_get_triangle(syn0->edge) == ivp_edge_get_triangle(syn1->edge);
    case IVP_ST_BALL:
        return true;
    case IVP_ST_EDGE:
        return syn0->edge == syn1->edge || ivp_edge_get_opposite(syn0->edge) == syn1->edge;
    default:
        IVP_CORE(); /* ivp_friction.cxx:2285, every build */
    }
    return false;
}

bool ivp_contact_pt_is_same_as(const ivp_contact_pt_t *cp, ivp_mindist_t *md2)
{
    const ivp_fr_synapse_t *sy00 = &cp->synapse[0];
    const ivp_fr_synapse_t *sy01 = &cp->synapse[1];
    const ivp_md_synapse_t *sy10 = &md2->synapse[0];
    const ivp_md_synapse_t *sy11 = &md2->synapse[1];
    if (sy00->l_obj == sy10->l_obj && sy01->l_obj == sy11->l_obj) {
        if (fr_synapse_is_same_as(sy00, sy10) && fr_synapse_is_same_as(sy01, sy11)) return true;
    }
    if (sy01->l_obj == sy10->l_obj && sy00->l_obj == sy11->l_obj) {
        if (fr_synapse_is_same_as(sy01, sy10) && fr_synapse_is_same_as(sy00, sy11)) return true;
    }
    return false;
}

/* ── IVP_Contact_Point ctor / dtor ──────────────────────────────────────── */

/* IVP_SurfaceManager::add_reference_to_ledge / remove_reference_to_ledge of
 * the ledges a contact point uses (IVP_Contact_Point ctor / dtor). */
static void cp_ledge_reference(const ivp_object_t *obj, const ivp_compact_edge_t *edge, bool add)
{
    if (!obj->surman || !edge) return;
    const ivp_compact_ledge_t *ledge = ivp_edge_get_compact_ledge(edge);
    if (add) ivp_surman_add_reference_to_ledge(obj->surman, ledge);
    else ivp_surman_remove_reference_to_ledge(obj->surman, ledge);
}

/* Inverse-rotate the contact normal before forming its moment arm (retail
 * RVA 0x1F860): the normal's x component stays double (summed z, y, x for
 * the first core, x, y, z for the second), y/z are float stores, and the
 * cross product uses double products. */
static void cp_contact_normal_cross(const ivp_transform_r_t *m, const ivp_vec3f_t *normal,
                                    const ivp_vec3f_t *point, ivp_vec3f_t *cross, bool second_core)
{
    ivp_vec3f_t n;
    lvm_vimult3_f(m, normal, &n);
    const double n0 = (double)normal->k[0], n1 = (double)normal->k[1], n2 = (double)normal->k[2];
    const double nx = second_core ? LVM_E(m, 0, 0) * n0 + LVM_E(m, 1, 0) * n1 + LVM_E(m, 2, 0) * n2
                                  : LVM_E(m, 2, 0) * n2 + LVM_E(m, 1, 0) * n1 + LVM_E(m, 0, 0) * n0;
    const double p0 = (double)point->k[0], p1 = (double)point->k[1], p2 = (double)point->k[2];
    cross->k[0] = (float)((double)n.k[2] * p1 - (double)n.k[1] * p2);
    cross->k[1] = (float)(nx * p2 - (double)n.k[2] * p0);
    cross->k[2] = (float)((double)n.k[1] * p0 - nx * p1);
}

/* ivp_contact_inverse_mass (ivp_friction.hxx): inverse mass of one contact
 * contribution plus an existing one, rounded once */
static float cp_contact_inverse_mass(const ivp_vec3f_t *cross, const ivp_core_t *core, float previous)
{
    const ivp_vec3f_t iri = ivp_core_inv_rot_inertia_f(core);
    const double c0 = (double)cross->k[0], c1 = (double)cross->k[1], c2 = (double)cross->k[2];
    return (float)(((double)iri.k[2] * c2 * c2 + c1 * (double)iri.k[1] * c1 + (double)iri.k[0] * c0 * c0) +
                   (double)ivp_core_get_inv_mass_f(core) + (double)previous);
}

ivp_contact_pt_t *ivp_contact_pt_create(ivp_mindist_t *md)
{
    const ivp_allocator_t *allocator = ivp_env_get_allocator(md->env);
    ivp_contact_pt_t *cp = (ivp_contact_pt_t *)ivp_alloc(allocator, sizeof(*cp));
    if (!cp) return NULL;
    memset(cp, 0, sizeof(*cp));
    if (!ivp_build_uses_reference_internal_state()) {
        cp->creation_sequence = ivp_env_next_contact_sequence(md->env);
        if (cp->creation_sequence == 0u) {
            ivp_free(allocator, cp, sizeof(*cp));
            return NULL;
        }
    }
    ivp_md_synapse_t *syn0 = ivp_mindist_get_sorted_synapse(md, 0);
    fr_synapse_init(&cp->synapse[0], cp, syn0->l_obj, syn0->edge, syn0->status);
    /* add reference to the used ledge (released in ivp_contact_pt_destroy) */
    cp_ledge_reference(syn0->l_obj, syn0->edge, true);
    ivp_md_synapse_t *syn1 = ivp_mindist_get_sorted_synapse(md, 1);
    fr_synapse_init(&cp->synapse[1], cp, syn1->l_obj, syn1->edge, syn1->status);
    cp_ledge_reference(syn1->l_obj, syn1->edge, true);

    cp->last_time_of_recalc_friction_s_vals = ivp_env_get_current_time(md->env);

    IVP_IF(1) /* ivp_friction.cxx:86 */
    {
        struct ivp_environment *env = md->env;
        IVP_IF(cp_dm(env)->check_fs)
        {
            fprintf(cp_dm(env)->out_deb_file, "create_mindist %f %p cores %p %p\n", ivp_env_get_current_time(env),
                    (void *)cp, (void *)syn0->l_obj->physical_core, (void *)syn1->l_obj->physical_core);
        }
    }
    if (syn1->status == IVP_ST_TRIANGLE) {
        const ivp_compact_edge_t *e1 = syn1->edge;
        ivp_vec3r_t wHesse_vecF_Fos;
        cls_calc_hesse_vec_object_not_normized(e1, ivp_edge_get_compact_ledge(e1), &wHesse_vecF_Fos);
        double hesse_len = (double)lvp_real_length(&wHesse_vecF_Fos);
        cp->inv_triangle_det = (hesse_len > P_DOUBLE_EPS) ? (float)(1.0f / hesse_len) : 0.0f;
    }
    cp->old_energy_dynamic_fr = 0.0f;
    cp->now_friction_pressure = 0.0f;
    cp->integrated_destroyed_energy = 0.0f;
    cp->cp_status = IVP_CPBS_NEEDS_RECHECK;
    cp->span_friction_s[0] = 0.0f;
    cp->span_friction_s[1] = 0.0f;
    cp->last_gap_len = ivp_env_get_mindist_settings(md->env)->friction_dist;
    cp->has_negative_pull_since = 0;
    cp->slowly_turn_on_keeper = IVP_FR_SLOWLY_TURN_ON_KEEPER;
    cp->l_friction_system = NULL;
    cp->two_friction_values = false;
    return cp;
}

void ivp_contact_pt_destroy(ivp_contact_pt_t *cp)
{
    ivp_object_t *obj0 = cp->synapse[0].l_obj;
    struct ivp_environment *env = obj0->owner;
    cp_ledge_reference(cp->synapse[0].l_obj, cp->synapse[0].edge, false);
    cp_ledge_reference(cp->synapse[1].l_obj, cp->synapse[1].edge, false);
    ivp_env_fire_friction_deleted(env, cp);
    {
        ivp_core_t *core0 = NULL, *core1 = NULL;
        IVP_IF(cp_dm(env)->check_fs) /* ivp_friction.cxx:923 */
        {
            core0 = cp->synapse[0].l_obj->friction_core;
            core1 = cp->synapse[1].l_obj->friction_core;
        }
        fr_synapse_remove_from_object(&cp->synapse[0]);
        fr_synapse_remove_from_object(&cp->synapse[1]);
        IVP_IF(cp_dm(env)->check_fs) /* ivp_friction.cxx:932 */
        {
            if (core0->physical_unmoveable) {
                core_unmovable_core_debug_friction_hash(core0);
            }
            if (core1->physical_unmoveable) {
                core_unmovable_core_debug_friction_hash(core1);
            }
        }
    }
    ivp_free(ivp_env_get_allocator(env), cp, sizeof(*cp));
}

void ivp_contact_pt_reset_time(ivp_contact_pt_t *cp, double offset)
{
    cp->last_time_of_recalc_friction_s_vals -= offset;
}

/* ── materials (IVP_Contact_Point::get_material_info etc.) ──────────────── */

static const ivp_material_t *ivp_simple_index_material(void)
{
    /* IVP_Material_Manager::get_material_by_index: static IVP_Material_Simple(0.5f, 0.5f) */
    static const ivp_material_t simple = {
        .friction = IVP_REAL(0.5),
        .elasticity = IVP_REAL(0.5),
        .adhesion = IVP_REAL(0.0),
        .second_friction_x_enabled = false,
        .second_friction_factor = IVP_REAL(0.0),
    };
    return &simple;
}

void ivp_contact_pt_get_material_info(ivp_contact_pt_t *cp, const ivp_material_t *mtl[2])
{
    for (int k = 0; k < 2; k++) {
        int mat_index = fr_synapse_get_material_index(&cp->synapse[k]);
        if (mat_index == 0) {
            mtl[k] = &cp->synapse[k].l_obj->material;
        } else {
            mtl[k] = ivp_simple_index_material();
        }
        IVP_ASSERT(mtl[k]);
    }
}

void ivp_contact_pt_read_materials_for_contact_situation(ivp_contact_pt_t *cp, ivp_impact_solver_lt_t *info)
{
    ivp_object_t *obj0 = cp->synapse[0].l_obj;
    ivp_object_t *obj1 = cp->synapse[1].l_obj;
    ivp_contact_pt_get_material_info(cp, info->materials);
    struct ivp_environment *env = obj0->owner;
    info->objects[0] = obj0;
    info->objects[1] = obj1;
    info->compact_edges[0] = cp->synapse[0].edge;
    info->compact_edges[1] = cp->synapse[1].edge;
    IVP_ASSERT(obj0 != obj1);
    info->impact.percent_energy_conservation = ivp_env_material_get_elasticity(env, info);
    cp->real_friction_factor = ivp_env_material_get_friction_factor(env, info);
}

/* IVP_Contact_Point::calc_virtual_mass_of_mindist */
void ivp_contact_pt_calc_virtual_mass_of_mindist(ivp_contact_pt_t *cp)
{
    const ivp_material_t *mtl[2];
    ivp_contact_pt_get_material_info(cp, mtl);
    if (mtl[0]->second_friction_x_enabled || mtl[1]->second_friction_x_enabled) {
        cp->two_friction_values = true;
    }
    ivp_core_t *core[2];
    for (int i = 0; i < 2; i++) core[i] = cp->synapse[i].l_obj->physical_core;
    double virt_mass_mindist_no_dir;
    ivp_impact_solver_lt_t *tci = cp->tmp_contact_info;
    if (core[0]->physical_unmoveable) {
        virt_mass_mindist_no_dir = ivp_core_calc_virt_mass_worst_case(core[1], &tci->contact_point_cs[1]);
    } else if (core[1]->physical_unmoveable) {
        virt_mass_mindist_no_dir = ivp_core_calc_virt_mass_worst_case(core[0], &tci->contact_point_cs[0]);
    } else {
        double vmass_no_dir[2];
        for (int j = 0; j < 2; j++) {
            vmass_no_dir[j] = ivp_core_calc_virt_mass_worst_case(core[j], &tci->contact_point_cs[j]);
        }
        double vmass_sum = vmass_no_dir[0] + vmass_no_dir[1];
        virt_mass_mindist_no_dir = (vmass_sum > P_DOUBLE_EPS) ? (vmass_no_dir[0] * vmass_no_dir[1]) / vmass_sum : 0.0;
    }
    cp->inv_virt_mass_mindist_no_dir =
        (virt_mass_mindist_no_dir > P_DOUBLE_EPS) ? (float)(1.0f / virt_mass_mindist_no_dir) : 0.0f;
    IVP_ASSERT(virt_mass_mindist_no_dir < (double)1e30f); /* test for NaN */
}

void ivp_contact_pt_recompute_friction(ivp_contact_pt_t *cp)
{
    struct ivp_environment *env = cp->synapse[0].l_obj->owner;
    ivp_env_sim_mem_begin(env);
    ivp_contact_pt_recalc_friction_s_vals(cp);
    ivp_env_sim_mem_end(env);
}

/* ── IVP_Contact_Point::p_calc_friction_* (ivp_mindist_friction.cxx) ────── */

static void p_calc_friction_qr_PF(ivp_contact_pt_t *cp, const ivp_vec3r_t *pp, const ivp_compact_edge_t *F,
                                  ivp_cache_ledge_point_t *m_cache_F, ivp_impact_solver_lt_t *info)
{
    info->contact_point_ws = *pp;
    ivp_vec3r_t pp_Fos;
    co_pos_to_object(m_cache_F->clp_cache_object, pp, &pp_Fos);
    if (cp->cp_status == IVP_CPBS_NEEDS_RECHECK) {
        cp->cp_status = IVP_CPBS_NORMAL;
        qr_result_t qr;
        cls_calc_unscaled_qr_vals_F_space(m_cache_F->compact_ledge, F, &pp_Fos, &qr);
        if (qr_is_outside(&qr)) {
            info->friction_is_broken = true;
        }
    }
    { /* calc surface normal */
        ivp_vec3r_t wHesse_vecF_Fos;
        cls_calc_hesse_vec_object_not_normized(F, m_cache_F->compact_ledge, &wHesse_vecF_Fos);
        lvp_mult(&wHesse_vecF_Fos, (D)cp->inv_triangle_det);
        ivp_vec3r_t hesse_vec_ws;
        co_vec_to_world(m_cache_F->clp_cache_object, &wHesse_vecF_Fos, &hesse_vec_ws);
        ivp_vec3f_t F_O_Fos = clp_f(F, m_cache_F);
        ivp_vec3f_t F_1_Fos = clp_f(ivp_edge_get_next(F), m_cache_F);
        cp->last_gap_len = (float)(lvp_dot(&wHesse_vecF_Fos, &pp_Fos) - lvp_dot_f(&wHesse_vecF_Fos, &F_O_Fos));
        lvf_set_multiple_p(&info->surf_normal, &hesse_vec_ws, -1.0f);
        ivp_vec3f_t qvec_Fos;
        lvf_subtract2(&qvec_Fos, &F_1_Fos, &F_O_Fos);
        /* transform_vector_to_world_coords(const IVP_U_Float_Point*, IVP_U_Float_Point*) */
        lvm_vmult3_f(&m_cache_F->clp_cache_object->m_world_f_object, &qvec_Fos, &info->span_friction_v[0]);
    }
}

static void p_calc_friction_s_PP(ivp_contact_pt_t *cp, const ivp_vec3r_t *pp0, const ivp_vec3r_t *pp1,
                                 ivp_impact_solver_lt_t *info)
{
    ivp_vec3r_t dist_vec;
    lvp_subtract2(&dist_vec, pp1, pp0);
    cp->last_gap_len = (float)lvp_real_length_plus_normize(&dist_vec);
    lvf_set_p(&info->surf_normal, &dist_vec);
    ivp_vec3f_t magic_world_random_vec;
    if (info->surf_normal.k[0] * info->surf_normal.k[0] < 0.9f) {
        lvf_set3(&magic_world_random_vec, 1.0f, 0.0f, 0.0f);
    } else {
        lvf_set3(&magic_world_random_vec, 0.0f, 0.0f, 1.0f);
    }
    lvf_cross_and_normize(&info->span_friction_v[0], &info->surf_normal, &magic_world_random_vec);
    info->contact_point_ws = *pp0;
}

static void p_calc_friction_s_PK(ivp_contact_pt_t *cp, const ivp_vec3r_t *pp, const ivp_compact_edge_t *K,
                                 ivp_cache_ledge_point_t *m_cache_K, ivp_impact_solver_lt_t *info)
{
    ivp_vec3r_t tp, tp_next;
    cls_give_world_coords_AT(K, m_cache_K, &tp);
    cls_give_world_coords_AT(ivp_edge_get_next(K), m_cache_K, &tp_next);
    ivp_vec3f_t vec1, vec2;
    lvf_subtract2_p(&vec1, &tp_next, &tp);
    lvf_subtract2_p(&vec2, pp, &tp);
    double vec1_ql = (double)lvf_quad_length(&vec1);
    double i_len = (vec1_ql > P_DOUBLE_EPS) ? (double)lv_isqrt_double((D)vec1_ql) : 0.0;
    ivp_vec3f_t vert_12;
    lvf_cross(&vert_12, &vec1, &vec2);
    double distance = (double)lvf_real_length(&vert_12) * i_len;
    cp->last_gap_len = (float)distance;
    if (distance * distance > P_DOUBLE_EPS) {
        lvf_set_multiple(&vert_12, &vert_12, (D)(i_len / distance));
    } else {
        lvf_set3(&vert_12, 1.0f, 0.0f, 0.0f);
    }
    lvf_cross_and_normize(&info->surf_normal, &vec1, &vert_12);
    info->contact_point_ws = *pp;
    info->span_friction_v[0] = vert_12;
    IVP_IF(1) /* ivp_mindist_friction.cxx:206 */
    {
        double len = (double)lvf_real_length(&info->span_friction_v[0]);
        len = 1.0f - len;
        if (fabs(len) > (double)0.001f) {
            ivp_debug_printf("span_friction_v_pk not normized\n");
        }
    }
    if (cp->cp_status == IVP_CPBS_NEEDS_RECHECK) {
        cp->cp_status = IVP_CPBS_NORMAL;
        double iqlen = i_len * i_len;
        double s = (double)lvf_dot(&vec1, &vec2) * iqlen;
        if (s < 0.0 || s > 1.0) {
            info->friction_is_broken = true;
        }
    }
}

/* IVP_U_Point::set_interpolate (ivu_linear.cxx) */
static void lvp_set_interpolate_pp(ivp_vec3r_t *r, const ivp_vec3r_t *p0, const ivp_vec3r_t *p1, D s)
{
    D is = 1.0f - s;
    D a = p0->k[0] * is, a2 = p1->k[0] * s;
    D b = p0->k[1] * is, b2 = p1->k[1] * s;
    D c = p0->k[2] * is, c2 = p1->k[2] * s;
    r->k[0] = a + a2;
    r->k[1] = b + b2;
    r->k[2] = c + c2;
}

static void p_calc_friction_ss_KK(ivp_contact_pt_t *cp, const ivp_compact_edge_t *K, const ivp_compact_edge_t *L,
                                  ivp_cache_ledge_point_t *m_cache_K, ivp_cache_ledge_point_t *m_cache_L,
                                  ivp_impact_solver_lt_t *info)
{
    D sl, sk;
    ivp_vec3r_t norm, Kvec, Lvec;
    ivp_vec3r_t Kp, Kp_next, Lp, Lp_next;
    cls_give_world_coords_AT(K, m_cache_K, &Kp);
    cls_give_world_coords_AT(ivp_edge_get_next(K), m_cache_K, &Kp_next);
    cls_give_world_coords_AT(L, m_cache_L, &Lp);
    cls_give_world_coords_AT(ivp_edge_get_next(L), m_cache_L, &Lp_next);
    lvp_subtract2(&Lvec, &Lp_next, &Lp);
    (void)lvp_fast_normize(&Lvec);
    lvp_subtract2(&Kvec, &Kp_next, &Kp);
    (void)lvp_fast_normize(&Kvec);
    lvp_cross(&norm, &Kvec, &Lvec);
    D quad_dist = lvp_quad_length(&norm);
    if (quad_dist > (P_DOUBLE_RES * P_DOUBLE_RES)) {
        ivp_vec3r_t K_area, L_area;
        lvp_cross(&K_area, &Kvec, &norm);
        lvp_cross(&L_area, &Lvec, &norm);
        /* ivp_kk_contact_fraction: the retail build divides without the
         * later SDK's zero guard */
        D a = lvp_dot(&K_area, &Lp);
        D b = lvp_dot(&K_area, &Lp_next);
        D reciprocal = 1.0 / (a - b);
        sl = (a - lvp_dot(&K_area, &Kp)) * reciprocal;
        a = lvp_dot(&L_area, &Kp);
        b = lvp_dot(&L_area, &Kp_next);
        reciprocal = 1.0 / (a - b);
        sk = (a - lvp_dot(&L_area, &Lp)) * reciprocal;
        ivp_vec3r_t world_edge_space0, world_edge_space1;
        lvp_set_interpolate_pp(&world_edge_space0, &Kp, &Kp_next, sk);
        lvp_set_interpolate_pp(&world_edge_space1, &Lp, &Lp_next, sl);
        ivp_vec3r_t world_diff_vec;
        lvp_subtract2(&world_diff_vec, &world_edge_space1, &world_edge_space0);
        D length = lvp_real_length(&world_diff_vec);
        if (length > P_DOUBLE_EPS) {
            cp->last_gap_len = (float)length;
            lvf_set_multiple_p(&info->surf_normal, &world_diff_vec, (D)(1.0f / length));
        } else {
            cp->last_gap_len = 0.0f;
            D sqrt_qd = (D)sqrt((double)quad_dist);
            if (sqrt_qd > P_DOUBLE_EPS) {
                lvf_set_multiple_p(&info->surf_normal, &norm, (D)(1.0f / sqrt_qd));
            } else {
                lvf_set3(&info->surf_normal, 1.0f, 0.0f, 0.0f);
            }
        }
        lvf_set_p(&info->span_friction_v[0], &Kvec);
        info->contact_point_ws = world_edge_space0;
        if (cp->cp_status == IVP_CPBS_NEEDS_RECHECK) {
            cp->cp_status = IVP_CPBS_NORMAL;
            if (sl < 0.0f || sl > 1.0f || sk < 0.0f || sk > 1.0f) {
                info->friction_is_broken = true;
            }
        }
    } else {
        info->friction_is_broken = true;
        cp->last_gap_len = ivp_env_get_mindist_settings(
            cp->synapse[0].l_obj->owner)->keeper_dist;
        info->contact_point_ws = Kp;
        lvf_set3(&info->surf_normal, 1.0f, 0.0f, 0.0f);
        lvf_set3(&info->span_friction_v[0], 0.0f, 1.0f, 0.0f);
    }
}

/* IVP_Contact_Point::recalc_friction_s_vals */
void ivp_contact_pt_recalc_friction_s_vals(ivp_contact_pt_t *cp)
{
    ivp_fr_synapse_t *syn0 = &cp->synapse[0];
    ivp_fr_synapse_t *syn1 = &cp->synapse[1];
    struct ivp_environment *env = syn0->l_obj->owner;
    ivp_impact_solver_lt_t *info =
        (ivp_impact_solver_lt_t *)ivp_env_sim_mem_alloc(env, sizeof(ivp_impact_solver_lt_t));
    ivp_impact_solver_lt_init_tmp(info);
    cp->tmp_contact_info = info;

    /* statistic counter, every build (ivp_mindist_friction.cxx:361) */
    ivp_env_get_statistic_manager(env)->processed_fmindists++;

    short stat0 = syn0->status;
    short stat1 = syn1->status;
    {
        ivp_cache_ledge_point_t m_cache_0, m_cache_1;
        const ivp_compact_edge_t *e1 = syn1->edge;
        ivp_cache_ledge_point_init(&m_cache_1, syn1->l_obj, ivp_edge_get_compact_ledge(e1));
        m_cache_1.tmp.synapse_friction = syn1;
        const ivp_compact_edge_t *e0 = syn0->edge;
        ivp_cache_ledge_point_init(&m_cache_0, syn0->l_obj, ivp_edge_get_compact_ledge(e0));
        m_cache_0.tmp.synapse_friction = syn0;
#ifdef IVP_BENCHMARK_DIAGNOSTICS
        cp_contact_recalc_diagnostics_record(cp, m_cache_0.clp_cache_object,
                                             m_cache_1.clp_cache_object,
                                             ivp_env_get_current_time(env));
#endif

        ivp_vec3r_t point0_position_ws;
        switch (stat0) {
        case IVP_ST_BALL:
        case IVP_ST_POINT:
            if (stat0 == IVP_ST_BALL) {
                point0_position_ws = m_cache_0.clp_cache_object->m_world_f_object.pos;
                cp->cp_status = IVP_CPBS_NEEDS_RECHECK;
            } else {
                cls_give_world_coords_AT(e0, &m_cache_0, &point0_position_ws);
            }
            switch (stat1) {
            case IVP_ST_BALL: {
                ivp_vec3r_t tp2 = m_cache_1.clp_cache_object->m_world_f_object.pos;
                p_calc_friction_s_PP(cp, &point0_position_ws, &tp2, info);
                break;
            }
            case IVP_ST_POINT: {
                ivp_vec3r_t tp2;
                cls_give_world_coords_AT(e1, &m_cache_1, &tp2);
                p_calc_friction_s_PP(cp, &point0_position_ws, &tp2, info);
                break;
            }
            case IVP_ST_EDGE:
                p_calc_friction_s_PK(cp, &point0_position_ws, e1, &m_cache_1, info);
                break;
            case IVP_ST_TRIANGLE:
                p_calc_friction_qr_PF(cp, &point0_position_ws, e1, &m_cache_1, info);
                break;
            default:
                IVP_CORE(); /* ivp_mindist_friction.cxx:427, every build */
            }
            break;
        case IVP_ST_EDGE:
            IVP_ASSERT(stat1 == IVP_ST_EDGE);
            p_calc_friction_ss_KK(cp, e0, e1, &m_cache_0, &m_cache_1, info);
            break;
        default:
            IVP_CORE(); /* ivp_mindist_friction.cxx:438, every build */
        }
        { /* reduce len of friction contact by extra radius */
            float first_radius = syn0->l_obj->extra_radius;
            lvp_add_multiple_f(&info->contact_point_ws, &info->surf_normal, (D)first_radius);
            cp->last_gap_len -= first_radius + syn1->l_obj->extra_radius;
        }
        if (cp->last_gap_len < 0.0f) cp->last_gap_len = 0.0f;
        ivp_cache_ledge_point_remove_reference(&m_cache_0);
        ivp_cache_ledge_point_remove_reference(&m_cache_1);
    }
    (void)lvf_normize(&info->span_friction_v[0]);
    lvf_cross(&info->span_friction_v[1], &info->surf_normal, &info->span_friction_v[0]);

    ivp_vec3f_t delta_velocity_ws;
    {
        ivp_core_t *core0 = syn0->l_obj->physical_core;
        if (!core0->physical_unmoveable) {
            const ivp_transform_r_t *m = &core0->m_world_f_core_last_psi;
            /* retail RVA 0x1F860: the contact point is inverse rotated with
             * double offsets, summed y, z, x */
            const double px = info->contact_point_ws.k[0] - m->pos.k[0];
            const double py = info->contact_point_ws.k[1] - m->pos.k[1];
            const double pz = info->contact_point_ws.k[2] - m->pos.k[2];
            info->contact_point_cs[0].k[0] = (float)(LVM_E(m, 1, 0) * py + LVM_E(m, 2, 0) * pz + LVM_E(m, 0, 0) * px);
            info->contact_point_cs[0].k[1] = (float)(LVM_E(m, 1, 1) * py + LVM_E(m, 2, 1) * pz + LVM_E(m, 0, 1) * px);
            info->contact_point_cs[0].k[2] = (float)(LVM_E(m, 1, 2) * py + LVM_E(m, 2, 2) * pz + LVM_E(m, 0, 2) * px);
            cp_contact_normal_cross(m, &info->surf_normal, &info->contact_point_cs[0],
                                    &info->contact_cross_nomal_cs[0], false);
            ivp_vec3f_t sp = ivp_core_speed_f(core0);
            ivp_core_get_surface_speed_on_test(core0, &info->contact_point_cs[0], &sp, &core0->rot_speed,
                                               &delta_velocity_ws);
            info->inv_virtual_mass = cp_contact_inverse_mass(&info->contact_cross_nomal_cs[0], core0, 0.0f);
        } else {
            core0 = NULL;
            lvf_zero(&delta_velocity_ws);
            info->inv_virtual_mass = 0.0f;
            lvf_zero(&info->contact_point_cs[0]);
            lvf_zero(&info->contact_cross_nomal_cs[0]);
        }
        info->contact_core[0] = core0;
    }
    {
        ivp_core_t *core1 = syn1->l_obj->physical_core;
        if (!core1->physical_unmoveable) {
            const ivp_transform_r_t *m_world_f_core = &core1->m_world_f_core_last_psi;
            lvm_vimult4_pf(m_world_f_core, &info->contact_point_ws, &info->contact_point_cs[1]);
            cp_contact_normal_cross(m_world_f_core, &info->surf_normal, &info->contact_point_cs[1],
                                    &info->contact_cross_nomal_cs[1], true);
            ivp_vec3f_t dh;
            ivp_vec3f_t sp = ivp_core_speed_f(core1);
            ivp_core_get_surface_speed_on_test(core1, &info->contact_point_cs[1], &sp, &core1->rot_speed, &dh);
            lvf_subtract(&delta_velocity_ws, &dh);
            info->inv_virtual_mass = cp_contact_inverse_mass(&info->contact_cross_nomal_cs[1], core1,
                                                             info->inv_virtual_mass);
        } else {
            lvf_zero(&info->contact_point_cs[1]);
            lvf_zero(&info->contact_cross_nomal_cs[1]);
            core1 = NULL;
        }
        info->contact_core[1] = core1;
    }
    if (info->inv_virtual_mass > P_DOUBLE_EPS) {
        info->virtual_mass = 1.0f / info->inv_virtual_mass;
    } else {
        info->virtual_mass = 0.0f;
    }
    {
        ivp_time_t ctime = ivp_env_get_current_time(env);
        double dt = ivp_time_diff(ctime, cp->last_time_of_recalc_friction_s_vals);
        cp->last_time_of_recalc_friction_s_vals = ctime;
        /* IVP_FLOAT span_friction_s -= IVP_DOUBLE expression (double subtraction, then rounded) */
        cp->span_friction_s[0] = (float)((double)cp->span_friction_s[0] - lvf_dot(&delta_velocity_ws, &info->span_friction_v[0]) * dt);
        cp->span_friction_s[1] = (float)((double)cp->span_friction_s[1] - lvf_dot(&delta_velocity_ws, &info->span_friction_v[1]) * dt);
        lvf_set_p(&cp->last_contact_point_ws, &info->contact_point_ws);
        cp->surf_normal_ws = info->surf_normal;
    }
}

/* ── IVP_Friction_Manager ───────────────────────────────────────────────── */

ivp_contact_pt_t *ivp_friction_mgr_get_associated_contact_point(ivp_mindist_t *md)
{
    ivp_object_t *obj0 = md->synapse[0].l_obj;
    ivp_object_t *obj1 = md->synapse[1].l_obj;
    for (ivp_fr_synapse_t *syn0 = obj0->fr_synapses; syn0; syn0 = syn0->next) {
        ivp_contact_pt_t *cp = syn0->contact_point;
        if (cp->synapse[0].l_obj == obj1 || cp->synapse[1].l_obj == obj1) {
            if (ivp_contact_pt_is_same_as(cp, md)) return cp;
        }
    }
    return NULL;
}

void ivp_friction_mgr_delete_all_contact_points_of_object(ivp_object_t *obj0)
{
    ivp_friction_mgr_restore_object_contact_order(obj0);
    ivp_fr_synapse_t *syn0;
    while ((syn0 = obj0->fr_synapses) != NULL) {
        ivp_contact_pt_destroy(syn0->contact_point);
    }
}

ivp_contact_pt_t *ivp_friction_mgr_generate_contact_point(ivp_mindist_t *md, bool *successful)
{
    if (md->mindist_status != IVP_MD_EXACT) {
        *successful = false;
        return NULL;
    }
    ivp_contact_pt_t *already = ivp_friction_mgr_get_associated_contact_point(md);
    if (already) {
        *successful = false;
        return already;
    }
    ivp_contact_pt_t *cp = ivp_contact_pt_create(md);
    *successful = (cp != NULL);
    return cp;
}

/* ── IVP_Friction_Info_For_Core + IVP_Core friction info (ivp_core.cxx) ── */

ivp_fr_info_for_core_t *ivp_fr_info_create(const ivp_allocator_t *allocator)
{
    if (!allocator) allocator = ivp_default_allocator();
    ivp_fr_info_for_core_t *info =
        (ivp_fr_info_for_core_t *)ivp_alloc(allocator, sizeof(*info));
    if (info) {
        memset(info, 0, sizeof(*info));
        info->allocator = allocator;
        ivp_pvec_init_with_allocator(&info->friction_springs, allocator);
    }
    return info;
}

void ivp_fr_info_destroy(ivp_fr_info_for_core_t *info)
{
    if (!info) return;
    ivp_pvec_clear(&info->friction_springs);
    ivp_free(info->allocator, info, sizeof(*info));
}

bool ivp_fr_info_insert_friction_dist(ivp_fr_info_for_core_t *info, ivp_contact_pt_t *cp)
{
    return info && ivp_pvec_add(&info->friction_springs, cp) >= 0;
}

void ivp_fr_info_delete_friction_dist(ivp_fr_info_for_core_t *info, ivp_contact_pt_t *cp)
{
    ivp_pvec_remove(&info->friction_springs, cp);
}

/* Unmoveable cores keep a small list keyed by friction system
 * (reference: IVP_Friction_Hash; only find/add/remove are used). */
typedef struct ivp_fr_info_list {
    ivp_pvec_t infos;
} ivp_fr_info_list_t;

ivp_fr_info_for_core_t *ivp_core_get_friction_info(ivp_core_t *core, ivp_fr_system_t *fs)
{
    if (core->physical_unmoveable) {
        ivp_fr_info_list_t *l = (ivp_fr_info_list_t *)core->core_friction_info.for_unmoveables.l_friction_info_hash;
        if (!l) return NULL;
        for (int i = 0; i < ivp_pvec_len(&l->infos); i++) {
            ivp_fr_info_for_core_t *info = (ivp_fr_info_for_core_t *)ivp_pvec_at(&l->infos, i);
            if (info->l_friction_system == fs) return info;
        }
        return NULL;
    }
    ivp_fr_info_for_core_t *info = (ivp_fr_info_for_core_t *)core->core_friction_info.for_moveables.moveable_core_friction_info;
    if (info && info->l_friction_system == fs) return info;
    return NULL;
}

ivp_fr_info_for_core_t *ivp_core_moveable_core_has_friction_info(ivp_core_t *core)
{
    IVP_ASSERT(!core->physical_unmoveable);
    return (ivp_fr_info_for_core_t *)core->core_friction_info.for_moveables.moveable_core_friction_info;
}

bool ivp_core_add_friction_info(ivp_core_t *core, ivp_fr_info_for_core_t *info)
{
    if (core->physical_unmoveable) {
        ivp_fr_info_list_t *l = (ivp_fr_info_list_t *)core->core_friction_info.for_unmoveables.l_friction_info_hash;
        bool created = false;
        if (!l) {
            const ivp_allocator_t *allocator = ivp_env_get_allocator(ivp_core_env(core));
            l = (ivp_fr_info_list_t *)ivp_alloc(allocator, sizeof(*l));
            if (!l) return false;
            memset(l, 0, sizeof(*l));
            ivp_pvec_init_with_allocator(&l->infos, allocator);
            core->core_friction_info.for_unmoveables.l_friction_info_hash = l;
            created = true;
        }
        IVP_IF(1) /* ivp_core.cxx:1484 */
        {
            IVP_ASSERT(ivp_core_get_friction_info(core, info->l_friction_system) == NULL);
        }
        if (ivp_pvec_add(&l->infos, info) < 0) {
            if (created) {
                core->core_friction_info.for_unmoveables.l_friction_info_hash = NULL;
                ivp_free(ivp_pvec_allocator(&l->infos), l, sizeof(*l));
            }
            return false;
        }
        IVP_IF(1) /* IVP_Friction_Hash::add_friction_info, ivp_i_friction_hash.hxx:17 */
        {
            struct ivp_environment *env = info->l_friction_system->l_environment;
            if (cp_dm(env)->check_fs) {
                IVP_ASSERT(ivp_core_get_friction_info(core, info->l_friction_system) == info);
            }
        }
    } else {
        IVP_ASSERT(core->core_friction_info.for_moveables.moveable_core_friction_info == NULL);
        core->core_friction_info.for_moveables.moveable_core_friction_info = info;
    }
    return true;
}

void ivp_core_unlink_friction_info(ivp_core_t *core, ivp_fr_info_for_core_t *info)
{
    if (core->physical_unmoveable) {
        ivp_fr_info_list_t *l = (ivp_fr_info_list_t *)core->core_friction_info.for_unmoveables.l_friction_info_hash;
        /* remove_friction_info must find it (ivp_core.cxx:1527) */
        IVP_ASSERT(l && ivp_pvec_index_of(&l->infos, info) >= 0);
        if (l) ivp_pvec_remove(&l->infos, info);
    } else {
        IVP_ASSERT(core->core_friction_info.for_moveables.moveable_core_friction_info == info);
        core->core_friction_info.for_moveables.moveable_core_friction_info = NULL;
    }
}

void ivp_core_delete_friction_info(ivp_core_t *core, ivp_fr_info_for_core_t *info)
{
    ivp_core_unlink_friction_info(core, info);
    ivp_fr_info_destroy(info);
}

void ivp_core_free_friction_info_hash(ivp_core_t *core)
{
    if (!core->physical_unmoveable) return;
    ivp_fr_info_list_t *l = (ivp_fr_info_list_t *)core->core_friction_info.for_unmoveables.l_friction_info_hash;
    if (l) {
        IVP_ASSERT(ivp_pvec_len(&l->infos) == 0); /* ~IVP_Friction_Hash, ivp_i_friction_hash.cxx:11 */
        const ivp_allocator_t *allocator = ivp_pvec_allocator(&l->infos);
        ivp_pvec_clear(&l->infos);
        ivp_free(allocator, l, sizeof(*l));
        core->core_friction_info.for_unmoveables.l_friction_info_hash = NULL;
    }
}

/* IVP_Core::unmovable_core_debug_friction_hash (ivp_core.cxx:1503), called
 * from IVP_IF(check_fs) in ~IVP_Contact_Point */
static void core_unmovable_core_debug_friction_hash(ivp_core_t *core)
{
    for (int c = ivp_core_n_objects(core) - 1; c >= 0; c--) {
        ivp_object_t *obj = ivp_core_object_at(core, c);
        for (ivp_fr_synapse_t *fr_synapse = obj->fr_synapses; fr_synapse; fr_synapse = fr_synapse->next) {
            ivp_contact_pt_t *fr_mindist = fr_synapse->contact_point;
            ivp_fr_system_t *fr_sys = fr_mindist->l_friction_system;
            IVP_ASSERT(ivp_core_get_friction_info(core, fr_sys)->l_friction_system == fr_sys); /* error after deleting hash entry */
            IVP_USE(fr_sys);
        }
    }
}
