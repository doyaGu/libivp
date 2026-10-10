/* ivp_attacher_to_cores.c -- IVP_Attacher_To_Cores port
 * (ivp_physics/ivp_attacher_to_cores.hxx).
 */
#include <ivp/controllers/ivp_attacher_to_cores.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_assert.h>

#include "dynamics/ivp_env_internal.h"
#include "dynamics/ivp_sim_internal.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define ATTACHER_HASH_INIT_SIZE 16 /* core_to_attachment_hash(16) */

static ivp_attacher_to_cores_t *attacher_of_set_ctx(void *ctx) { return (ivp_attacher_to_cores_t *)ctx; }

static ivp_attacher_to_cores_t *attacher_of_listener(ivp_listener_object_t *l)
{
    return (ivp_attacher_to_cores_t *)l->user_data;
}

/* element_added: new ATTACH_T(this, elem); hash.add_elem(elem, con) */
static void attacher_element_added(void *ctx, ivp_set_active_t *set, void *elem)
{
    (void)set;
    ivp_attacher_to_cores_t *a = attacher_of_set_ctx(ctx);
    ivp_core_t *core = (ivp_core_t *)elem;
    if (!core || !a->environment) return;
    /* only one attachment per core */
    if (ivp_vhash_store_find(&a->core_to_attachment_hash, core)) return;
    void *con = a->cls->create_attachment ? a->cls->create_attachment(a, core) : NULL;
    if (con) {
        ivp_vhash_store_add(&a->core_to_attachment_hash, core, con);
        if (ivp_vhash_store_find(&a->core_to_attachment_hash, core) != con) {
            a->allocation_failed = true;
            if (a->cls->delete_attachment) {
                a->cls->delete_attachment(a, core, con);
            }
        }
    }
}

/* P_DELETE(con) of a core's attachment; the ATTACH_T destructor calls
 * attachment_is_going_to_be_deleted (hash removal). */
static void attacher_delete_core(ivp_attacher_to_cores_t *a, ivp_core_t *core)
{
    /* only cores with an attachment (the reference ATTACH_T listens to its
     * own core only; remove_elem of a missing key is an error) */
    void *con = ivp_vhash_store_find(&a->core_to_attachment_hash, core);
    if (!con) return;
    (void)ivp_vhash_store_remove(&a->core_to_attachment_hash, core);
    if (a->cls->delete_attachment) a->cls->delete_attachment(a, core, con);
}

/* element_removed: con = hash.find_elem(elem); IVP_ASSERT(con); P_DELETE(con) */
static void attacher_remove_element(ivp_attacher_to_cores_t *a, ivp_core_t *core)
{
    void *con = ivp_vhash_store_find(&a->core_to_attachment_hash, core);
    IVP_ASSERT(con);
    IVP_USE(con);
    attacher_delete_core(a, core);
}

static void attacher_element_removed(void *ctx, ivp_set_active_t *set, void *elem)
{
    (void)set;
    attacher_remove_element(attacher_of_set_ctx(ctx), (ivp_core_t *)elem);
}

typedef void (*attacher_core_action_fn)(ivp_attacher_to_cores_t *attacher,
                                        ivp_core_t *core);

static int attacher_compare_core_identity(const void *left,
                                          const void *right)
{
    const ivp_core_t *left_core = *(ivp_core_t *const *)left;
    const ivp_core_t *right_core = *(ivp_core_t *const *)right;
    uint64_t left_key = ivp_core_stable_order_key(left_core);
    uint64_t right_key = ivp_core_stable_order_key(right_core);
    IVP_ASSERT(left_key != right_key || left_core == right_core);
    return left_key < right_key ? -1 : left_key > right_key ? 1 : 0;
}

static void attacher_add_core(ivp_attacher_to_cores_t *a,
                              ivp_core_t *core)
{
    attacher_element_added(a, NULL, core);
}

static void attacher_apply_set_order(ivp_attacher_to_cores_t *a,
                                     const ivp_set_t *set,
                                     bool attached_only,
                                     attacher_core_action_fn action)
{
    int count = ivp_set_count(set);
    if (count <= 0) return;

    ivp_core_t **cores = (ivp_core_t **)ivp_alloc(
        &a->allocator, (size_t)count * sizeof(*cores));
    if (cores) {
        int actual_count = 0;
        ivp_set_enumerator_t enumerator;
        ivp_set_enumerator_init(&enumerator, set);
        ivp_core_t *core;
        while ((core = (ivp_core_t *)ivp_set_enumerator_next(
                    &enumerator, set)) != NULL) {
            if (attached_only &&
                !ivp_vhash_store_find(&a->core_to_attachment_hash,
                                      core)) {
                continue;
            }
            cores[actual_count++] = core;
        }
        qsort(cores, (size_t)actual_count, sizeof(*cores),
              attacher_compare_core_identity);
        for (int i = 0; i < actual_count; i++) {
            action(a, cores[i]);
        }
        ivp_free(&a->allocator, cores,
                 (size_t)count * sizeof(*cores));
        return;
    }

    /* Keep callback order independent of hash slots even if the temporary
     * snapshot cannot be allocated. */
    bool have_previous = false;
    uint64_t previous_key = 0u;
    for (int emitted = 0; emitted < count; emitted++) {
        ivp_core_t *selected = NULL;
        uint64_t selected_key = UINT64_MAX;
        ivp_set_enumerator_t enumerator;
        ivp_set_enumerator_init(&enumerator, set);
        ivp_core_t *core;
        while ((core = (ivp_core_t *)ivp_set_enumerator_next(
                    &enumerator, set)) != NULL) {
            if (attached_only &&
                !ivp_vhash_store_find(&a->core_to_attachment_hash,
                                      core)) {
                continue;
            }
            uint64_t key = ivp_core_stable_order_key(core);
            if ((!have_previous || key > previous_key) &&
                (!selected || key < selected_key)) {
                selected = core;
                selected_key = key;
            }
        }
        if (!selected) break;
        action(a, selected);
        previous_key = selected_key;
        have_previous = true;
    }
}

/* delete every attachment (hash order) */
static void attacher_delete_all(ivp_attacher_to_cores_t *a)
{
    ivp_vhash_store_t *h = &a->core_to_attachment_hash;
    if (ivp_build_uses_canonical_internal_state()) {
        int count = ivp_vhash_store_count(h);
        if (count <= 0) return;
        ivp_core_t **cores = (ivp_core_t **)ivp_alloc(
            &a->allocator, (size_t)count * sizeof(*cores));
        if (cores) {
            int actual_count = 0;
            for (int i = 0; i < ivp_vhash_store_capacity(h); i++) {
                if (h->elems[i].key_elem) {
                    cores[actual_count++] =
                        (ivp_core_t *)h->elems[i].key_elem;
                }
            }
            qsort(cores, (size_t)actual_count, sizeof(*cores),
                  attacher_compare_core_identity);
            for (int i = 0; i < actual_count; i++) {
                attacher_delete_core(a, cores[i]);
            }
            ivp_free(&a->allocator, cores,
                     (size_t)count * sizeof(*cores));
            return;
        }

        while (ivp_vhash_store_count(h) > 0) {
            ivp_core_t *selected = NULL;
            uint64_t selected_key = UINT64_MAX;
            for (int i = 0; i < ivp_vhash_store_capacity(h); i++) {
                ivp_core_t *core =
                    (ivp_core_t *)h->elems[i].key_elem;
                if (!core) continue;
                uint64_t key = ivp_core_stable_order_key(core);
                if (!selected || key < selected_key) {
                    selected = core;
                    selected_key = key;
                }
            }
            if (!selected) break;
            attacher_delete_core(a, selected);
        }
        return;
    }

    while (ivp_vhash_store_count(h) > 0) {
        void *key = NULL;
        for (int i = ivp_vhash_store_capacity(h) - 1; i >= 0; i--) {
            if (h->elems[i].key_elem) {
                key = h->elems[i].key_elem;
                break;
            }
        }
        if (!key) break;
        attacher_delete_core(a, (ivp_core_t *)key);
    }
}

static void attacher_stop_listening(ivp_attacher_to_cores_t *a)
{
    if (a->set_of_cores) {
        ivp_set_active_remove_listener(a->set_of_cores, &a->set_listener);
        a->set_of_cores = NULL;
    }
    if (a->environment) {
        ivp_environment_remove_listener_object_global(a->environment, &a->object_listener);
        a->environment = NULL;
    }
}

/* pset_is_going_to_be_deleted: element_removed(core) for every core in the
 * set, then P_DELETE_THIS(this) (~IVP_Attacher_To_Cores removes the set
 * listener). */
static void attacher_pset_is_going_to_be_deleted(void *ctx, ivp_set_active_t *set)
{
    ivp_attacher_to_cores_t *a = attacher_of_set_ctx(ctx);
    if (ivp_build_uses_canonical_internal_state()) {
        /* A set can retain a pointer after its core deleted its own
         * attachment.  Preserve the reference assertion for those missing
         * entries without dereferencing the stale pointer for ordering. */
        ivp_set_enumerator_t enumerator;
        ivp_set_enumerator_init(&enumerator, &set->set);
        ivp_core_t *core;
        while ((core = (ivp_core_t *)ivp_set_enumerator_next(
                    &enumerator, &set->set)) != NULL) {
            if (!ivp_vhash_store_find(&a->core_to_attachment_hash, core)) {
                attacher_remove_element(a, core);
            }
        }
        attacher_apply_set_order(a, &set->set, true,
                                 attacher_remove_element);
    } else {
        ivp_set_enumerator_t enumerator;
        ivp_set_enumerator_init(&enumerator, &set->set);
        ivp_core_t *core;
        while ((core = (ivp_core_t *)ivp_set_enumerator_next(
                    &enumerator, &set->set)) != NULL) {
            attacher_remove_element(a, core);
        }
    }
    IVP_ASSERT(ivp_vhash_store_count(&a->core_to_attachment_hash) == 0);
    attacher_delete_all(a);
    attacher_stop_listening(a);
    ivp_vhash_store_destroy(&a->core_to_attachment_hash);
    a->initialized = false;
    if (a->cls->attacher_deleted) a->cls->attacher_deleted(a);
}

/* ATTACH_T listens to its core: IVP_Core::~IVP_Core deletes the attachment
 * (the core is deleted with its last object; cores deleted by
 * ivp_environment_merge_objects / ivp_object_attach_object must be removed
 * from the set first, like the reference's set keeps no deleted cores) */
static void attacher_object_deleted(ivp_listener_object_t *l, ivp_event_object_t *ev)
{
    ivp_attacher_to_cores_t *a = attacher_of_listener(l);
    if (!a->environment || !ev->real_object) return;
    ivp_core_t *core = ev->real_object->physical_core;
    if (!core || ivp_pvec_len(&core->objects) > 1) return;   /* the core outlives the object */
    attacher_delete_core(a, core);
}

static void attacher_object_created(ivp_listener_object_t *l, ivp_event_object_t *ev)
{
    ivp_attacher_to_cores_t *a = attacher_of_listener(l);
    if (!a->environment || !a->set_of_cores || !a->object_filter || !ev->real_object) return;
    if (!a->object_filter(a->object_filter_ctx, ev->real_object)) return;
    ivp_set_active_install(a->set_of_cores, ev->real_object->physical_core);
}

/* The environment is going away: all cores will be deleted, delete the
 * attachments now (while the environment is still valid). */
static void attacher_environment_deleted(ivp_listener_object_t *l, struct ivp_environment *env)
{
    (void)env;
    ivp_attacher_to_cores_t *a = attacher_of_listener(l);
    if (!a->environment) return;
    attacher_delete_all(a);
    ivp_environment_remove_listener_object_global(a->environment, &a->object_listener);
    a->environment = NULL;
    a->environment_deleted = true;
}

bool ivp_attacher_to_cores_init(ivp_attacher_to_cores_t *a, struct ivp_environment *env,
                                ivp_set_active_t *set_of_cores,
                                const ivp_attacher_to_cores_class_t *cls, void *user_data)
{
    if (!a) return false;
    memset(a, 0, sizeof(*a));
    if (!env || !set_of_cores || !cls) return false;
    a->allocator = *env->allocator;
    ivp_vhash_store_init(&a->core_to_attachment_hash, &a->allocator,
                         ATTACHER_HASH_INIT_SIZE);
    if (!a->core_to_attachment_hash.elems) return false;
    a->set_of_cores = set_of_cores;
    a->environment = env;
    a->cls = cls;
    a->user_data = user_data;
    a->initialized = true;

    a->set_listener.element_added = attacher_element_added;
    a->set_listener.element_removed = attacher_element_removed;
    a->set_listener.pset_is_going_to_be_deleted = attacher_pset_is_going_to_be_deleted;
    a->set_listener.ctx = a;

    a->object_listener.event_object_deleted = attacher_object_deleted;
    a->object_listener.event_object_created = attacher_object_created;
    a->object_listener.event_environment_deleted = attacher_environment_deleted;
    a->object_listener.user_data = a;
    ivp_environment_add_listener_object_global(env, &a->object_listener);

    /* attach to every core already in the set, then listen */
    if (ivp_build_uses_canonical_internal_state()) {
        attacher_apply_set_order(a, &set_of_cores->set, false,
                                 attacher_add_core);
    } else {
        ivp_set_enumerator_t enumerator;
        ivp_set_enumerator_init(&enumerator, &set_of_cores->set);
        ivp_core_t *core;
        while ((core = (ivp_core_t *)ivp_set_enumerator_next(
                    &enumerator, &set_of_cores->set)) != NULL) {
            attacher_element_added(a, NULL, core);
        }
    }
    if (a->allocation_failed ||
        !ivp_set_active_add_listener(set_of_cores, &a->set_listener)) {
        a->set_of_cores = NULL;
        ivp_attacher_to_cores_destroy(a);
        return false;
    }
    return true;
}

void ivp_attacher_to_cores_destroy(ivp_attacher_to_cores_t *a)
{
    if (!a || !a->initialized) return;
    /* after environment deletion the attachments are already gone */
    if (a->environment) attacher_delete_all(a);
    attacher_stop_listening(a);
    ivp_vhash_store_destroy(&a->core_to_attachment_hash);
    a->initialized = false;
}

void *ivp_attacher_to_cores_get_attachment(ivp_attacher_to_cores_t *a, const ivp_core_t *core)
{
    if (!a || !a->initialized || !core) return NULL;
    return ivp_vhash_store_find(&a->core_to_attachment_hash, (void *)(uintptr_t)core);
}

void ivp_attacher_to_cores_attachment_is_going_to_be_deleted(ivp_attacher_to_cores_t *a,
                                                             void *attachment, ivp_core_t *attached_core)
{
    (void)attachment;
    if (!a || !a->initialized || !attached_core) return;
    (void)ivp_vhash_store_remove(&a->core_to_attachment_hash, attached_core);
}

void ivp_attacher_to_cores_set_object_filter(ivp_attacher_to_cores_t *a,
                                             ivp_attacher_object_filter_fn_t filter, void *ctx)
{
    if (!a) return;
    a->object_filter = filter;
    a->object_filter_ctx = ctx;
}
