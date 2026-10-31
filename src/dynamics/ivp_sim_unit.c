/* ivp_sim_unit.c — port of IVP_Simulation_Unit / IVP_Sim_Units_Manager /
 * IVP_Controller_Manager (ivp_intern/ivp_sim_unit.cxx)
 */
#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <ivp/containers/ivp_set.h>
#include <ivp/dynamics/ivp_sim_unit.h>
#include "dynamics/ivp_sim_internal.h"
#include "dynamics/ivp_env_hot.h"
#include "dynamics/ivp_env_parallel_tasks.h"
#include "dynamics/ivp_env_pipeline.h"
#include "solver/ivp_friction_internal.h"

#define IVP_SU_PARALLEL_MIN_ISLANDS 32u
#define IVP_SU_PARALLEL_LIGHT_ISLANDS 192u
#define IVP_SU_PARALLEL_MIN_WORK_UNITS 576u

static inline ivp_core_t *su_core_at(const ivp_sim_unit_t *su, int i) {
    return (ivp_core_t *)ivp_pvec_at(&su->sim_unit_cores, i);
}
static inline ivp_su_ctrl_list_t *su_cl_at(const ivp_sim_unit_t *su, int i) {
    return (ivp_su_ctrl_list_t *)ivp_pvec_at(&su->controller_cores, i);
}

/* ── lifecycle ─────────────────────────────────────────────────────────── */

ivp_sim_unit_t *ivp_sim_unit_new(const ivp_allocator_t *allocator)
{
    if (!allocator) allocator = ivp_default_allocator();
    ivp_sim_unit_t *su = (ivp_sim_unit_t *)ivp_alloc(allocator, sizeof(*su));
    if (!su) return NULL;
    memset(su, 0, sizeof(*su));
    su->allocator = allocator;
    su->union_find_needed_for_sim_unit = false;
    su->sim_unit_movement_type = IVP_MT_NOT_SIM;
    su->sim_unit_just_slowed_down = false;
    su->sim_unit_has_fast_objects = false;
    su->prev_sim_unit = NULL;
    su->next_sim_unit = NULL;
    ivp_pvec_init_with_allocator(&su->sim_unit_cores, allocator);
    ivp_pvec_init_with_allocator(&su->controller_cores, allocator);
    return su;
}

static void su_cl_delete(ivp_su_ctrl_list_t *cl)
{
    const ivp_allocator_t *allocator = ivp_pvec_allocator(&cl->cores_controlled_by);
    ivp_pvec_clear(&cl->cores_controlled_by);
    ivp_free(allocator, cl, sizeof(*cl));
}

void ivp_sim_unit_clean_sim_unit(ivp_sim_unit_t *su)
{
    for (int i = ivp_pvec_len(&su->controller_cores) - 1; i >= 0; i--) {
        su_cl_delete(su_cl_at(su, i));
    }
    ivp_pvec_remove_all(&su->controller_cores);
}

void ivp_sim_unit_delete(ivp_sim_unit_t *su)
{
    if (!su) return;
    if (su->manager) ivp_sim_units_mgr_rem_sim_unit(su->manager, su);
    ivp_sim_unit_clean_sim_unit(su);
    ivp_pvec_clear(&su->controller_cores);
    ivp_pvec_clear(&su->sim_unit_cores);
    ivp_free(su->allocator, su, sizeof(*su));
}

/* ── controller lists ──────────────────────────────────────────────────── */

static ivp_su_ctrl_list_t *su_find_controller(const ivp_pvec_t *controllers, ivp_sim_ctrl_t *c)
{
    for (int i = ivp_pvec_len(controllers) - 1; i >= 0; i--) {
        ivp_su_ctrl_list_t *cl = (ivp_su_ctrl_list_t *)ivp_pvec_at(controllers, i);
        if (cl->l_controller == c) return cl;
    }
    return NULL;
}

static ivp_su_ctrl_list_t *su_add_controller_unit_sim(ivp_pvec_t *controllers, ivp_sim_ctrl_t *c)
{
    const ivp_allocator_t *allocator = ivp_pvec_allocator(controllers);
    ivp_su_ctrl_list_t *cl = (ivp_su_ctrl_list_t *)ivp_alloc(allocator, sizeof(*cl));
    if (!cl) return NULL;
    memset(cl, 0, sizeof(*cl));
    cl->l_controller = c;
    ivp_pvec_init_with_allocator(&cl->cores_controlled_by, allocator);
    if (ivp_pvec_add(controllers, cl) < 0) {
        ivp_free(allocator, cl, sizeof(*cl));
        return NULL;
    }
    return cl;
}

static int su_get_pos_of_controller(ivp_sim_unit_t *su, ivp_sim_ctrl_t *c)
{
    int i;
    for (i = ivp_pvec_len(&su->controller_cores) - 1; i >= 0; i--) {
        if (su_cl_at(su, i)->l_controller == c) break;
    }
    return i;
}

/* IVP_Simulation_Unit::sim_unit_exchange_controllers */
static void su_exchange_controllers(ivp_sim_unit_t *su, int first, int second)
{
    IVP_ASSERT(first >= 0);
    IVP_ASSERT(second >= 0);
    IVP_ASSERT(first < ivp_pvec_len(&su->controller_cores));
    IVP_ASSERT(second < ivp_pvec_len(&su->controller_cores));
    ivp_pvec_swap(&su->controller_cores, first, second);
}

static void su_sort_controllers(ivp_sim_unit_t *su)
{
    int contr_num = ivp_pvec_len(&su->controller_cores);
    int first = 0;
    for (;;) {
        int second = first + 1;
        if (second >= contr_num) break;
        int second_prio = ivp_sim_ctrl_priority(su_cl_at(su, second)->l_controller);
        if (ivp_sim_ctrl_priority(su_cl_at(su, first)->l_controller) > second_prio) {
            su_exchange_controllers(su, first, second);
            int test_pos = first;
            while ((test_pos > 0) &&
                   (ivp_sim_ctrl_priority(su_cl_at(su, test_pos - 1)->l_controller) > second_prio)) {
                su_exchange_controllers(su, test_pos - 1, test_pos);
                test_pos--;
            }
        }
        first = second;
    }
    IVP_IF(1)
    {
        for (int i = 0; i < contr_num - 1; i++) {
            IVP_ASSERT(!(ivp_sim_ctrl_priority(su_cl_at(su, i)->l_controller) >
                         ivp_sim_ctrl_priority(su_cl_at(su, i + 1)->l_controller)));
        }
    }
}

bool ivp_sim_unit_sim_unit_calc_redundants(ivp_sim_unit_t *su)
{
    if (su->controller_cores.n < 0 || su->controller_cores.cap < 0 ||
        su->controller_cores.n > su->controller_cores.cap) {
        return false;
    }

    /* Most callers clear the old index first and can reuse its capacity.  A
     * live index is rebuilt beside the old one so OOM leaves it untouched. */
    bool replace_existing = ivp_pvec_len(&su->controller_cores) > 0;
    ivp_pvec_t rebuilt = {0};
    ivp_pvec_t *target = &su->controller_cores;
    if (replace_existing) {
        ivp_pvec_init_with_allocator(&rebuilt,
                                     ivp_pvec_allocator(&su->controller_cores));
        target = &rebuilt;
    }
    for (int i = ivp_pvec_len(&su->sim_unit_cores) - 1; i >= 0; i--) {
        ivp_core_t *my_core = su_core_at(su, i);
        for (int k = ivp_pvec_len(&my_core->controllers_of_core) - 1; k >= 0; k--) {
            ivp_sim_ctrl_t *c = (ivp_sim_ctrl_t *)ivp_pvec_at(&my_core->controllers_of_core, k);
            ivp_su_ctrl_list_t *cl = su_find_controller(target, c);
            if (!cl) cl = su_add_controller_unit_sim(target, c);
            if (!cl || ivp_pvec_install(&cl->cores_controlled_by, my_core) < 0) {
                for (int j = ivp_pvec_len(target) - 1; j >= 0; j--) {
                    su_cl_delete((ivp_su_ctrl_list_t *)ivp_pvec_at(target, j));
                }
                ivp_pvec_remove_all(target);
                if (replace_existing) ivp_pvec_clear(target);
                return false;
            }
        }
    }

    if (replace_existing) {
        ivp_sim_unit_t rebuilt_unit = *su;
        rebuilt_unit.controller_cores = rebuilt;
        su_sort_controllers(&rebuilt_unit);
        rebuilt = rebuilt_unit.controller_cores;

        ivp_sim_unit_clean_sim_unit(su);
        ivp_pvec_clear(&su->controller_cores);
        su->controller_cores = rebuilt;
    } else {
        su_sort_controllers(su);
    }
    return true;
}

static void su_rem_sim_unit_controller(ivp_sim_unit_t *su, ivp_sim_ctrl_t *c)
{
    int pos = su_get_pos_of_controller(su, c);
    IVP_ASSERT(pos >= 0);
    if (pos < 0) return;
    su_cl_delete(su_cl_at(su, pos));
    ivp_pvec_remove_at(&su->controller_cores, pos);
}

bool ivp_sim_unit_add_controller_of_core(ivp_sim_unit_t *su, ivp_core_t *core, ivp_sim_ctrl_t *c)
{
    ivp_su_ctrl_list_t *cl = su_find_controller(&su->controller_cores, c);
    bool created = false;
    if (!cl) {
        cl = su_add_controller_unit_sim(&su->controller_cores, c);
        if (!cl) return false;
        created = true;
    }
    if (ivp_pvec_install(&cl->cores_controlled_by, core) < 0) {
        if (created) {
            ivp_pvec_remove_at(&su->controller_cores, ivp_pvec_len(&su->controller_cores) - 1);
            su_cl_delete(cl);
        }
        return false;
    }
    su_sort_controllers(su);
    return true;
}

void ivp_sim_unit_remove_controller_of_core(ivp_sim_unit_t *su, ivp_core_t *core, ivp_sim_ctrl_t *c)
{
    int pos = su_get_pos_of_controller(su, c);
    IVP_ASSERT(pos >= 0);
    if (pos < 0) return;
    ivp_su_ctrl_list_t *cl = su_cl_at(su, pos);
    ivp_pvec_remove(&cl->cores_controlled_by, core);
    if (ivp_pvec_len(&cl->cores_controlled_by) <= 0) {
        su_cl_delete(cl);
        ivp_pvec_remove_at(&su->controller_cores, pos);
    }
}

/* ── cores ─────────────────────────────────────────────────────────────── */

bool ivp_sim_unit_add_sim_unit_core(ivp_sim_unit_t *su, ivp_core_t *core)
{
    if (!su || !core || ivp_pvec_add(&su->sim_unit_cores, core) < 0) return false;
    ivp_sim_unit_forget_movement_check_salt(su);
    return true;
}
void ivp_sim_unit_rem_sim_unit_core(ivp_sim_unit_t *su, ivp_core_t *core)
{
    ivp_pvec_remove(&su->sim_unit_cores, core);
    ivp_sim_unit_forget_movement_check_salt(su);
}

static void su_remove_core_at(ivp_sim_unit_t *su, int index)
{
    ivp_pvec_remove_at(&su->sim_unit_cores, index);
    ivp_sim_unit_forget_movement_check_salt(su);
}

void ivp_sim_unit_sim_unit_remove_core(ivp_sim_unit_t *su, ivp_core_t *del_core)
{
    ivp_sim_unit_rem_sim_unit_core(su, del_core);
    for (int i = ivp_pvec_len(&del_core->controllers_of_core) - 1; i >= 0; i--) {
        ivp_sim_ctrl_t *c = (ivp_sim_ctrl_t *)ivp_pvec_at(&del_core->controllers_of_core, i);
        int pos = su_get_pos_of_controller(su, c);
        IVP_ASSERT(pos >= 0);
        if (pos < 0) continue;
        ivp_su_ctrl_list_t *cl = su_cl_at(su, pos);
        ivp_pvec_remove(&cl->cores_controlled_by, del_core);
        if (ivp_pvec_len(&cl->cores_controlled_by) == 0) su_rem_sim_unit_controller(su, c);
    }
    if (ivp_pvec_len(&su->sim_unit_cores) == 0) {
        ivp_sim_units_mgr_rem_sim_unit(ivp_env_get_sim_units_manager(ivp_core_env(del_core)), su);
        ivp_sim_unit_delete(su);
    }
}

/* IVP_Simulation_Unit::sim_unit_core_exists */
bool ivp_sim_unit_sim_unit_core_exists(ivp_sim_unit_t *su, ivp_core_t *core)
{
    for (int i = ivp_pvec_len(&su->sim_unit_cores) - 1; i >= 0; i--) {
        if (su_core_at(su, i) == core) return true;
    }
    return false;
}

#ifdef IVP_DEBUG_BUILD
/* IVP_Simulation_Unit::sim_unit_debug_out (reference: #ifdef DEBUG) */
void ivp_sim_unit_sim_unit_debug_out(ivp_sim_unit_t *su)
{
    ivp_debug_printf("sim_unit_cores:\n");
    int i;
    for (i = ivp_pvec_len(&su->sim_unit_cores) - 1; i >= 0; i--) {
        ivp_core_t *my_core = su_core_at(su, i);
        ivp_debug_printf("%p: ", (void *)my_core);
        ivp_sim_ctrl_t *my_cnt;
        int j;
        for (j = ivp_pvec_len(&my_core->controllers_of_core) - 1; j >= 0; j--) {
            my_cnt = (ivp_sim_ctrl_t *)ivp_pvec_at(&my_core->controllers_of_core, j);
            ivp_debug_printf("%p ", (void *)my_cnt);
        }
        ivp_debug_printf("\n");
    }

    for (i = ivp_pvec_len(&su->controller_cores) - 1; i >= 0; i--) {
        ivp_debug_printf("    controlr %p: ", (void *)su_cl_at(su, i)->l_controller);
        int j;
        ivp_su_ctrl_list_t *c_info = su_cl_at(su, i);
        for (j = ivp_pvec_len(&c_info->cores_controlled_by) - 1; j >= 0; j--) {
            ivp_debug_printf("%p ", ivp_pvec_at(&c_info->cores_controlled_by, j));
        }
        ivp_debug_printf("\n");
    }
}

/* IVP_Simulation_Unit::sim_unit_debug_consistency (reference: #ifdef DEBUG) */
void ivp_sim_unit_sim_unit_debug_consistency(ivp_sim_unit_t *su)
{
    /* every core has to occur only once */
    int i, j;
    for (i = ivp_pvec_len(&su->sim_unit_cores) - 1; i >= 0; i--) {
        ivp_core_t *search_core = su_core_at(su, i);
        for (j = i - 1; j >= 0; j--) {
            IVP_ASSERT(su_core_at(su, j) != search_core);
        }
        IVP_ASSERT(!search_core->physical_unmoveable);
        IVP_ASSERT(search_core->sim_unit == su);
    }
    /* every controller has to occur only once */
    for (i = ivp_pvec_len(&su->controller_cores) - 1; i >= 0; i--) {
        ivp_sim_ctrl_t *search_controller = su_cl_at(su, i)->l_controller;
        for (j = i - 1; j >= 0; j--) {
            IVP_ASSERT(su_cl_at(su, j)->l_controller != search_controller);
        }
        IVP_USE(search_controller);
    }

    /* sim_unit_controllers and controller_cores are associated */
    for (i = ivp_pvec_len(&su->controller_cores) - 1; i >= 0; i--) {
        ivp_su_ctrl_list_t *c_info = su_cl_at(su, i);
        /* every core in controller list must exist */
        for (j = ivp_pvec_len(&c_info->cores_controlled_by) - 1; j >= 0; j--) {
            ivp_core_t *test_core = (ivp_core_t *)ivp_pvec_at(&c_info->cores_controlled_by, j);
            IVP_ASSERT(ivp_sim_unit_sim_unit_core_exists(su, test_core) == true);
            IVP_USE(test_core);
        }
    }

    /* all cores must have all their controllers in controller list */
    for (int i1 = ivp_pvec_len(&su->sim_unit_cores) - 1; i1 >= 0; i1--) {
        ivp_core_t *my_core = su_core_at(su, i1);
        for (int i2 = ivp_pvec_len(&my_core->controllers_of_core) - 1; i2 >= 0; i2--) {
            ivp_sim_ctrl_t *my_controller = (ivp_sim_ctrl_t *)ivp_pvec_at(&my_core->controllers_of_core, i2);

            IVP_ASSERT(su_find_controller(&su->controller_cores, my_controller) != NULL);

            /* for every core its via controllers associated cores must exist */
            ivp_pvec_t *core_list = my_controller->vt->get_associated_controlled_cores(my_controller);
            for (int i4 = (core_list ? ivp_pvec_len(core_list) : 0) - 1; i4 >= 0; i4--) {
                ivp_core_t *test_core = (ivp_core_t *)ivp_pvec_at(core_list, i4);
                IVP_ASSERT(!test_core->physical_unmoveable);
                IVP_ASSERT(ivp_sim_unit_sim_unit_core_exists(su, test_core) == true);
                IVP_USE(test_core);
            }

            /* every core/controller pair must be represented in c_info */
            int pos = su_get_pos_of_controller(su, my_controller);
            if (pos < 0) continue; /* libivp: no element_at(-1) */
            ivp_su_ctrl_list_t *c_info = su_cl_at(su, pos);
            int found = 0;
            for (int i5 = ivp_pvec_len(&c_info->cores_controlled_by) - 1; i5 >= 0; i5--) {
                ivp_core_t *tt_core = (ivp_core_t *)ivp_pvec_at(&c_info->cores_controlled_by, i5);
                if (tt_core == my_core) {
                    found = 1;
                }
                /* this controller must be found in my cores controller list */
                int find_contr = 0;
                for (int k = 0; k < ivp_pvec_len(&tt_core->controllers_of_core); k++) {
                    if (ivp_pvec_at(&tt_core->controllers_of_core, k) == (void *)my_controller) {
                        find_contr = 1;
                    }
                }
                IVP_ASSERT(find_contr == 1);
                IVP_USE(find_contr);
            }
            IVP_ASSERT(found == 1);
            IVP_USE(found);
        }
    }
}
#endif /* IVP_DEBUG_BUILD */

/* ── manager ───────────────────────────────────────────────────────────── */

void ivp_sim_units_mgr_init(ivp_sim_units_manager_t *m, struct ivp_environment *env)
{
    m->l_environment = env;
    m->sim_units_slot0 = NULL;
    m->still_slot = NULL;
    m->pending_union_find_count = 0;
}

static void mgr_add_unit_to_slot(ivp_sim_unit_t *sim_u, ivp_sim_unit_t **slot)
{
    ivp_sim_unit_t *s_u = *slot;
    sim_u->next_sim_unit = s_u;
    if (s_u) s_u->prev_sim_unit = sim_u;
    sim_u->prev_sim_unit = NULL;
    *slot = sim_u;
}

static void mgr_rem_unit_from_slot(ivp_sim_unit_t *sim_u, ivp_sim_unit_t **slot)
{
    ivp_sim_unit_t *prev_u = sim_u->prev_sim_unit;
    ivp_sim_unit_t *next_u = sim_u->next_sim_unit;
    if (prev_u) {
        prev_u->next_sim_unit = next_u;
    } else {
        *slot = next_u;
    }
    if (next_u) next_u->prev_sim_unit = prev_u;
}

void ivp_sim_units_mgr_add_sim_unit(ivp_sim_units_manager_t *m, ivp_sim_unit_t *su)
{
    IVP_ASSERT(!su->manager);
    su->manager = m;
    if (su->union_find_needed_for_sim_unit) m->pending_union_find_count++;
    ivp_sim_unit_t **slot = (su->sim_unit_movement_type < IVP_MT_NOT_SIM) ? &m->sim_units_slot0 : &m->still_slot;
    mgr_add_unit_to_slot(su, slot);
}

uint64_t ivp_core_stable_order_key(const ivp_core_t *core)
{
    uint64_t key = UINT64_MAX;
    for (int object_index = 0;
         object_index < ivp_pvec_len(&core->objects);
         object_index++) {
        const ivp_object_t *object =
            (const ivp_object_t *)ivp_pvec_at(&core->objects,
                                              object_index);
        uint64_t object_key = ivp_object_order_key(object);
        if (object_key < key) key = object_key;
    }
    IVP_ASSERT(key != UINT64_MAX);
    return key;
}

uint64_t ivp_sim_unit_stable_order_key(const ivp_sim_unit_t *su)
{
    uint64_t key = UINT64_MAX;
    for (int core_index = 0;
         core_index < ivp_pvec_len(&su->sim_unit_cores);
         core_index++) {
        uint64_t core_key =
            ivp_core_stable_order_key(su_core_at(su, core_index));
        if (core_key < key) key = core_key;
    }
    IVP_ASSERT(key != UINT64_MAX);
    return key;
}

#if IVP_EDIT_LOCALITY
static uint64_t su_scan_movement_check_salt(const ivp_sim_unit_t *su)
{
    const ivp_object_t *first = NULL;
    for (int core_index = 0;
         core_index < ivp_pvec_len(&su->sim_unit_cores);
         core_index++) {
        const ivp_core_t *core = su_core_at(su, core_index);
        for (int object_index = 0;
             object_index < ivp_pvec_len(&core->objects);
             object_index++) {
            const ivp_object_t *object =
                (const ivp_object_t *)ivp_pvec_at(&core->objects,
                                                  object_index);
            if (!first ||
                ivp_object_order_key(object) < ivp_object_order_key(first)) {
                first = object;
            }
        }
    }
    IVP_ASSERT(first);
    return first ? first->movement_check_salt : 0u;
}

/* The salt changes only with the unit's cores and their objects, so the
 * scan runs once per change instead of once per check; debug builds verify
 * the cached value. */
uint64_t ivp_sim_unit_movement_check_salt(ivp_sim_unit_t *su)
{
    if (!su->movement_check_salt_valid) {
        su->movement_check_salt = su_scan_movement_check_salt(su);
        su->movement_check_salt_valid = true;
    }
    IVP_IF(1) {
        IVP_ASSERT(su->movement_check_salt == su_scan_movement_check_salt(su));
    }
    return su->movement_check_salt;
}
#endif

void ivp_sim_unit_forget_movement_check_salt(ivp_sim_unit_t *su)
{
#if IVP_EDIT_LOCALITY
    if (su) su->movement_check_salt_valid = false;
#else
    (void)su;
#endif
}

static int core_identity_compare(const void *left, const void *right)
{
    const ivp_core_t *left_core = *(ivp_core_t *const *)left;
    const ivp_core_t *right_core = *(ivp_core_t *const *)right;
    uint64_t left_key = ivp_core_stable_order_key(left_core);
    uint64_t right_key = ivp_core_stable_order_key(right_core);
    IVP_ASSERT(left_key != right_key || left_core == right_core);
    return left_key < right_key ? -1 : left_key > right_key ? 1 : 0;
}

void ivp_core_sort_by_identity(ivp_core_t **cores, size_t count)
{
    if (!cores || count < 2u) return;
    qsort(cores, count, sizeof(*cores), core_identity_compare);
}

void ivp_core_vector_sort_by_identity(ivp_pvec_t *cores)
{
    if (!cores || ivp_pvec_len(cores) < 2) return;
    ivp_core_sort_by_identity((ivp_core_t **)cores->elems,
                              (size_t)ivp_pvec_len(cores));
}

void ivp_core_set_visit_by_identity(
    const ivp_set_t *set, const ivp_allocator_t *allocator,
    ivp_core_set_filter_fn_t filter, ivp_core_set_visit_fn_t visit,
    void *context)
{
    if (!set || !visit) return;
    const int count = ivp_set_count(set);
    if (count <= 0) return;
    if (!allocator) allocator = ivp_default_allocator();

    ivp_core_t **cores = NULL;
    if ((size_t)count <= SIZE_MAX / sizeof(*cores)) {
        cores = (ivp_core_t **)ivp_alloc(
            allocator, (size_t)count * sizeof(*cores));
    }
    if (cores) {
        int actual_count = 0;
        ivp_set_enumerator_t enumerator;
        ivp_set_enumerator_init(&enumerator, set);
        ivp_core_t *core;
        while (actual_count < count &&
               (core = (ivp_core_t *)ivp_set_enumerator_next(
                    &enumerator, set)) != NULL) {
            if (!filter || filter(context, core)) {
                cores[actual_count++] = core;
            }
        }
        ivp_core_sort_by_identity(cores, (size_t)actual_count);
        for (int i = 0; i < actual_count; i++) {
            visit(context, cores[i]);
        }
        ivp_free(allocator, cores, (size_t)count * sizeof(*cores));
        return;
    }

    /* Keep visit order independent of hash slots without requiring scratch
     * allocation.  Visits may remove the selected core from the set. */
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
            if (filter && !filter(context, core)) continue;
            uint64_t key = ivp_core_stable_order_key(core);
            if ((!have_previous || key > previous_key) &&
                (!selected || key < selected_key)) {
                selected = core;
                selected_key = key;
            }
        }
        if (!selected) break;
        visit(context, selected);
        previous_key = selected_key;
        have_previous = true;
    }
}

typedef int (*mgr_order_compare_fn)(const void *left,
                                    const void *right);

static void mgr_sort_vector(ivp_pvec_t *vector,
                            mgr_order_compare_fn compare)
{
    for (int i = 1; i < ivp_pvec_len(vector); i++) {
        void *value = vector->elems[i];
        int position = i;
        while (position > 0 &&
               compare(vector->elems[position - 1], value) > 0) {
            vector->elems[position] = vector->elems[position - 1];
            position--;
        }
        vector->elems[position] = value;
    }
}

static int mgr_compare_objects(const void *left, const void *right)
{
    uint64_t left_key =
        ivp_object_order_key((const ivp_object_t *)left);
    uint64_t right_key =
        ivp_object_order_key((const ivp_object_t *)right);
    IVP_ASSERT(left_key != right_key || left == right);
    return left_key < right_key ? -1 : left_key > right_key ? 1 : 0;
}

static int mgr_compare_cores(const void *left, const void *right)
{
    uint64_t left_key =
        ivp_core_stable_order_key((const ivp_core_t *)left);
    uint64_t right_key =
        ivp_core_stable_order_key((const ivp_core_t *)right);
    IVP_ASSERT(left_key != right_key || left == right);
    return left_key < right_key ? -1 : left_key > right_key ? 1 : 0;
}

static int mgr_compare_controllers(const void *left, const void *right)
{
    const ivp_sim_ctrl_t *left_controller =
        (const ivp_sim_ctrl_t *)left;
    const ivp_sim_ctrl_t *right_controller =
        (const ivp_sim_ctrl_t *)right;
    int left_priority =
        ivp_sim_ctrl_priority((ivp_sim_ctrl_t *)left_controller);
    int right_priority =
        ivp_sim_ctrl_priority((ivp_sim_ctrl_t *)right_controller);
    if (left_priority != right_priority) {
        return left_priority < right_priority ? -1 : 1;
    }
    uint64_t left_key = left_controller->registration_sequence;
    uint64_t right_key = right_controller->registration_sequence;
    IVP_ASSERT(left_key != 0u && right_key != 0u);
    IVP_ASSERT(left_key != right_key || left == right);
    return left_key < right_key ? -1 : left_key > right_key ? 1 : 0;
}

static int mgr_compare_controller_lists(const void *left,
                                        const void *right)
{
    const ivp_su_ctrl_list_t *left_list =
        (const ivp_su_ctrl_list_t *)left;
    const ivp_su_ctrl_list_t *right_list =
        (const ivp_su_ctrl_list_t *)right;
    return mgr_compare_controllers(left_list->l_controller,
                                   right_list->l_controller);
}

/* Scratch of mgr_restore_controller_order: the cores of the unit grouped
 * into the reference's simulation units, each a list of cores. */
typedef struct mgr_replay {
    ivp_sim_unit_t *unit;
    int core_count, count;
    int *group, *next, *head, *tail, *walk; /* per core */
    int *order, *walked;                    /* per controller entry */
    uint64_t *key;                          /* per controller entry */
    uint64_t sequence;
    int walk_id;
} mgr_replay_t;

static int mgr_replay_core_index(const mgr_replay_t *r, const ivp_core_t *core)
{
    for (int i = 0; i < r->core_count; i++) {
        if (su_core_at(r->unit, i) == core) return i;
    }
    return -1;
}

static bool mgr_replay_controls(const mgr_replay_t *r, int entry, const ivp_core_t *core)
{
    const ivp_pvec_t *cores = &su_cl_at(r->unit, entry)->cores_controlled_by;
    for (int i = 0; i < ivp_pvec_len(cores); i++) {
        if (ivp_pvec_at(cores, i) == core) return true;
    }
    return false;
}

/* IVP_Simulation_Unit::throw_cores_into_my_sim_unit */
static void mgr_replay_fuse(mgr_replay_t *r, int into, int from)
{
    for (int i = r->head[from]; i >= 0; i = r->next[i]) r->group[i] = into;
    r->next[r->tail[into]] = r->head[from];
    r->tail[into] = r->tail[from];
    r->head[from] = r->tail[from] = -1;
}

/* IVP_Simulation_Unit::sim_unit_calc_redundants after the controllers
 * order[0..last] were announced: the group's controllers in the order a walk
 * of its cores and of each core's controllers, both from the last one,
 * meets them. */
static void mgr_replay_calc_redundants(mgr_replay_t *r, int group, int last)
{
    int length = 0;
    for (int i = r->head[group]; i >= 0; i = r->next[i]) r->walk[length++] = i;
    r->walk_id++;
    for (int pos = length - 1; pos >= 0; pos--) {
        const ivp_core_t *core = su_core_at(r->unit, r->walk[pos]);
        for (int k = last; k >= 0; k--) {
            int entry = r->order[k];
            if (r->walked[entry] == r->walk_id || !mgr_replay_controls(r, entry, core)) continue;
            r->walked[entry] = r->walk_id;
            r->key[entry] = ++r->sequence;
        }
    }
}

/* IVP_Controller_Manager::announce_controller_to_environment (dependent
 * controllers: the units of the cores, from the last one, thrown into the
 * first one met, the controllers walked again after a fusion) or
 * add_controller_to_core (independent ones), replayed for order[f]. */
static void mgr_replay_announce(mgr_replay_t *r, int f)
{
    int entry = r->order[f];
    ivp_sim_ctrl_t *c = su_cl_at(r->unit, entry)->l_controller;
    ivp_pvec_t *cores = c->vt->get_associated_controlled_cores(c);
    bool dependent = cores != NULL && ivp_pvec_len(cores) > 0;
    if (!dependent) cores = &su_cl_at(r->unit, entry)->cores_controlled_by;
    int reference = -1;
    bool fused = false;
    for (int i = ivp_pvec_len(cores) - 1; i >= 0; i--) {
        const ivp_core_t *core = (const ivp_core_t *)ivp_pvec_at(cores, i);
        if (!core || core->physical_unmoveable) continue;
        int index = mgr_replay_core_index(r, core);
        if (index < 0) continue;
        int group = r->group[index];
        if (reference < 0) {
            reference = group;
        } else if (group != reference && dependent) {
            mgr_replay_fuse(r, reference, group);
            fused = true;
        }
    }
    if (reference < 0) return;
    if (fused) {
        mgr_replay_calc_redundants(r, reference, f);
    } else if (r->key[entry] == 0) {
        r->key[entry] = ++r->sequence; /* add_controller_unit_sim */
    }
}

/* The reference keeps a simulation unit's controllers in the order its
 * history of announcements and fusions left them, and the PSI loop runs the
 * list backwards, so equal priorities run in that order.  Replay that
 * history from the restored state: the controllers announced in
 * registration order on cores that start in units of their own; units joined
 * otherwise (contacts) fuse at the end in core order.  Then
 * sim_unit_sort_controllers.  Without scratch memory the registration order
 * stays. */
static void mgr_restore_controller_order(ivp_sim_unit_t *unit)
{
    mgr_replay_t r;
    memset(&r, 0, sizeof(r));
    r.unit = unit;
    r.core_count = ivp_pvec_len(&unit->sim_unit_cores);
    r.count = ivp_pvec_len(&unit->controller_cores);
    /* one core: nothing fuses, every announcement appends */
    if (r.count < 2 || r.core_count < 2) return;
    size_t ints = (size_t)r.core_count * 5u + (size_t)r.count * 2u;
    size_t size = ints * sizeof(int) + (size_t)r.count * sizeof(uint64_t);
    const ivp_allocator_t *allocator = ivp_pvec_allocator(&unit->controller_cores);
    uint64_t *memory = (uint64_t *)ivp_alloc(allocator, size);
    if (!memory) return;
    r.key = memory;
    int *ints_base = (int *)(memory + r.count);
    r.group = ints_base;
    r.next = r.group + r.core_count;
    r.head = r.next + r.core_count;
    r.tail = r.head + r.core_count;
    r.walk = r.tail + r.core_count;
    r.order = r.walk + r.core_count;
    r.walked = r.order + r.count;
    for (int i = 0; i < r.core_count; i++) {
        r.group[i] = r.head[i] = r.tail[i] = i;
        r.next[i] = -1;
    }
    /* registration order: the list is sorted by priority, then registration */
    for (int i = 0; i < r.count; i++) {
        int entry = i, pos = i;
        uint64_t sequence = su_cl_at(unit, entry)->l_controller->registration_sequence;
        while (pos > 0 &&
               su_cl_at(unit, r.order[pos - 1])->l_controller->registration_sequence > sequence) {
            r.order[pos] = r.order[pos - 1];
            pos--;
        }
        r.order[pos] = entry;
        r.key[i] = 0;
        r.walked[i] = 0;
    }
    for (int f = 0; f < r.count; f++) mgr_replay_announce(&r, f);
    int first = r.group[0];
    bool fused = false;
    for (int i = 1; i < r.core_count; i++) {
        if (r.group[i] != first) {
            mgr_replay_fuse(&r, first, r.group[i]);
            fused = true;
        }
    }
    if (fused) mgr_replay_calc_redundants(&r, first, r.count - 1);
    for (int i = 1; i < r.count; i++) {
        for (int j = i; j > 0 && r.key[j - 1] > r.key[j]; j--) {
            uint64_t key = r.key[j];
            r.key[j] = r.key[j - 1];
            r.key[j - 1] = key;
            ivp_pvec_swap(&unit->controller_cores, j - 1, j);
        }
    }
    ivp_free(allocator, memory, size);
    su_sort_controllers(unit);
}

static void mgr_restore_unit_order(ivp_sim_units_manager_t *manager,
                                   ivp_sim_unit_t *unit)
{
    for (int i = 0; i < ivp_pvec_len(&unit->sim_unit_cores); i++) {
        ivp_core_t *core = su_core_at(unit, i);
        mgr_sort_vector(&core->objects, mgr_compare_objects);
        for (int j = 0;
             j < ivp_pvec_len(&core->controllers_of_core);
             j++) {
            ivp_env_register_sim_controller(
                manager->l_environment,
                (ivp_sim_ctrl_t *)ivp_pvec_at(
                    &core->controllers_of_core, j));
        }
        mgr_sort_vector(&core->controllers_of_core,
                        mgr_compare_controllers);
    }
    mgr_sort_vector(&unit->sim_unit_cores, mgr_compare_cores);
    for (int i = 0; i < ivp_pvec_len(&unit->controller_cores); i++) {
        ivp_su_ctrl_list_t *controller = su_cl_at(unit, i);
        ivp_env_register_sim_controller(manager->l_environment,
                                        controller->l_controller);
        mgr_sort_vector(&controller->cores_controlled_by,
                        mgr_compare_cores);
    }
    mgr_sort_vector(&unit->controller_cores,
                    mgr_compare_controller_lists);
    mgr_restore_controller_order(unit);
}

static ivp_sim_unit_t *mgr_merge_ordered_units(ivp_sim_unit_t *left,
                                                ivp_sim_unit_t *right)
{
    ivp_sim_unit_t head;
    ivp_sim_unit_t *tail = &head;
    head.next_sim_unit = NULL;
    while (left && right) {
        ivp_sim_unit_t **selected =
            ivp_sim_unit_stable_order_key(left) >=
                    ivp_sim_unit_stable_order_key(right)
                ? &left
                : &right;
        tail->next_sim_unit = *selected;
        tail = *selected;
        *selected = (*selected)->next_sim_unit;
    }
    tail->next_sim_unit = left ? left : right;
    return head.next_sim_unit;
}

static ivp_sim_unit_t *mgr_sort_units_by_identity(ivp_sim_unit_t *head)
{
    if (!head || !head->next_sim_unit) return head;
    bool already_sorted = true;
    for (ivp_sim_unit_t *unit = head; unit->next_sim_unit;
         unit = unit->next_sim_unit) {
        if (ivp_sim_unit_stable_order_key(unit) <
            ivp_sim_unit_stable_order_key(unit->next_sim_unit)) {
            already_sorted = false;
            break;
        }
    }
    if (already_sorted) return head;

    ivp_sim_unit_t *slow = head;
    ivp_sim_unit_t *fast = head->next_sim_unit;
    while (fast && fast->next_sim_unit) {
        slow = slow->next_sim_unit;
        fast = fast->next_sim_unit->next_sim_unit;
    }
    ivp_sim_unit_t *right = slow->next_sim_unit;
    slow->next_sim_unit = NULL;
    return mgr_merge_ordered_units(
        mgr_sort_units_by_identity(head),
        mgr_sort_units_by_identity(right));
}

static void mgr_restore_slot_order(ivp_sim_units_manager_t *manager,
                                   ivp_sim_unit_t **slot)
{
    *slot = mgr_sort_units_by_identity(*slot);
    ivp_sim_unit_t *previous = NULL;
    for (ivp_sim_unit_t *unit = *slot; unit;
         unit = unit->next_sim_unit) {
        unit->prev_sim_unit = previous;
        mgr_restore_unit_order(manager, unit);
        previous = unit;
    }
}

static void mgr_restore_unit_ordering(ivp_sim_units_manager_t *manager)
{
    if (ivp_build_uses_reference_internal_state()) {
        return;
    }
    mgr_restore_slot_order(manager, &manager->sim_units_slot0);
    mgr_restore_slot_order(manager, &manager->still_slot);
}

void ivp_sim_units_mgr_rem_sim_unit(ivp_sim_units_manager_t *m, ivp_sim_unit_t *su)
{
    IVP_ASSERT(su->manager == m);
    if (su->union_find_needed_for_sim_unit) {
        IVP_ASSERT(m->pending_union_find_count > 0);
        m->pending_union_find_count--;
    }
    ivp_sim_unit_t **slot = (su->sim_unit_movement_type < IVP_MT_NOT_SIM) ? &m->sim_units_slot0 : &m->still_slot;
    mgr_rem_unit_from_slot(su, slot);
    su->manager = NULL;
}

/* ── merge / split / union find ────────────────────────────────────────── */

void ivp_sim_unit_request_union_find(ivp_sim_unit_t *su)
{
    if (su->union_find_needed_for_sim_unit) return;
    su->union_find_needed_for_sim_unit = true;
    if (su->manager) su->manager->pending_union_find_count++;
}

static void su_clear_union_find_request(ivp_sim_unit_t *su)
{
    if (!su->union_find_needed_for_sim_unit) return;
    if (su->manager) {
        IVP_ASSERT(su->manager->pending_union_find_count > 0);
        su->manager->pending_union_find_count--;
    }
    su->union_find_needed_for_sim_unit = false;
}

bool ivp_sim_unit_throw_cores_into_my_sim_unit(ivp_sim_unit_t *su, ivp_sim_unit_t *second_unit)
{
    int first_count = ivp_pvec_len(&su->sim_unit_cores);
    int second_count = ivp_pvec_len(&second_unit->sim_unit_cores);
    if (second_count > INT_MAX - first_count ||
        !ivp_pvec_reserve(&su->sim_unit_cores, first_count + second_count)) return false;
    struct ivp_environment *env = NULL;
    for (int i = 0; i < second_count; i++) {
        ivp_core_t *my_core = su_core_at(second_unit, i);
        bool added = ivp_sim_unit_add_sim_unit_core(su, my_core);
        IVP_ASSERT(added);
        if (!added) return false;   /* destination capacity was reserved above */
        env = ivp_core_env(my_core);
        my_core->sim_unit = su;
    }
    if (env) ivp_sim_units_mgr_rem_sim_unit(ivp_env_get_sim_units_manager(env), second_unit);
    return true;
}

bool ivp_sim_unit_fusion_simulation_unities(ivp_sim_unit_t *su, ivp_sim_unit_t *second_unit)
{
    if (!ivp_sim_unit_throw_cores_into_my_sim_unit(su, second_unit)) return false;
    ivp_sim_unit_clean_sim_unit(su);
    if (!ivp_sim_unit_sim_unit_calc_redundants(su)) {
        ivp_sim_unit_request_union_find(su);
    }
    return true;
}

static ivp_core_t *su_union_find_test(ivp_sim_unit_t *su)
{
    for (int i = ivp_pvec_len(&su->sim_unit_cores) - 1; i >= 0; i--) {
        su_core_at(su, i)->union_find_father = NULL;
    }
    for (int i2 = ivp_pvec_len(&su->controller_cores) - 1; i2 >= 0; i2--) {
        ivp_sim_ctrl_t *c = su_cl_at(su, i2)->l_controller;
        ivp_pvec_t *list_of_cores = c->vt->get_associated_controlled_cores(c);
        if (!list_of_cores) continue;
        int j = ivp_pvec_len(list_of_cores) - 1;
        if (j >= 0) {
            ivp_core_t *father_core = ivp_core_union_find_get_father((ivp_core_t *)ivp_pvec_at(list_of_cores, 0));
            for (; j >= 0; j--) {
                ivp_core_t *test_core = (ivp_core_t *)ivp_pvec_at(list_of_cores, j);
                ivp_core_t *test_father = ivp_core_union_find_get_father(test_core);
                if (father_core != test_father) test_father->union_find_father = father_core;
            }
        }
    }
    ivp_core_t *first_father = ivp_core_union_find_get_father(su_core_at(su, 0));
    for (int j = 0; j < ivp_pvec_len(&su->sim_unit_cores); j++) {
        ivp_core_t *test_father = ivp_core_union_find_get_father(su_core_at(su, j));
        if (test_father != first_father) return test_father;
    }
    return NULL;
}

static void su_split_sim_unit(ivp_sim_unit_t *su, ivp_core_t *split_father)
{
    IVP_ASSERT(split_father);
    ivp_core_t *next_split_father = NULL;
    bool next_split_necessary = false;
    ivp_sim_unit_t *split_new_unit = ivp_sim_unit_new(su->allocator);
    if (!split_new_unit) {
        ivp_sim_unit_request_union_find(su);
        return;
    }
    int split_count = 0;
    for (int i = 0; i < ivp_pvec_len(&su->sim_unit_cores); i++) {
        if (ivp_core_union_find_get_father(su_core_at(su, i)) == split_father) split_count++;
    }
    if (!ivp_pvec_reserve(&split_new_unit->sim_unit_cores, split_count)) {
        ivp_sim_unit_delete(split_new_unit);
        ivp_sim_unit_request_union_find(su);
        return;
    }
    split_new_unit->sim_unit_movement_type = IVP_MT_MOVING;
    ivp_sim_units_mgr_add_sim_unit(ivp_env_get_sim_units_manager(ivp_core_env(split_father)), split_new_unit);
    for (int i = 0; i < ivp_pvec_len(&su->sim_unit_cores); i++) {
        ivp_core_t *my_core = su_core_at(su, i);
        ivp_core_t *test_core = ivp_core_union_find_get_father(my_core);
        if (test_core != split_father) {
            if (next_split_father != NULL) {
                if (test_core != next_split_father) next_split_necessary = true;
            } else {
                next_split_father = test_core;
            }
        } else {
            su_remove_core_at(su, i);
            i--;
            bool added = ivp_sim_unit_add_sim_unit_core(split_new_unit, my_core);
            IVP_ASSERT(added);
            if (!added) IVP_CORE();   /* destination capacity was reserved above */
            my_core->sim_unit = split_new_unit;
        }
    }
    bool rebuilt = ivp_sim_unit_sim_unit_calc_redundants(split_new_unit);
    if (!rebuilt) ivp_sim_unit_request_union_find(split_new_unit);
#ifdef IVP_DEBUG_BUILD
    IVP_IF(1)
    {
        if (rebuilt) ivp_sim_unit_sim_unit_debug_consistency(split_new_unit);
    }
#endif
    if (next_split_necessary) su_split_sim_unit(su, next_split_father);
}

static void su_perform_test_and_split(ivp_sim_unit_t *su)
{
    ivp_core_t *father = su_union_find_test(su);
    if (father) {
        ivp_sim_unit_clean_sim_unit(su);
        su_split_sim_unit(su, father);
        bool rebuilt = ivp_sim_unit_sim_unit_calc_redundants(su);
        if (!rebuilt) ivp_sim_unit_request_union_find(su);
#ifdef IVP_DEBUG_BUILD
        if (rebuilt) ivp_sim_unit_sim_unit_debug_consistency(su);
#endif
    }
}

void ivp_sim_unit_do_sim_unit_union_find(ivp_sim_unit_t *su)
{
    ivp_sim_unit_clean_sim_unit(su);
    if (!ivp_sim_unit_sim_unit_calc_redundants(su)) {
        ivp_sim_unit_request_union_find(su);
        return;
    }
    su_clear_union_find_request(su);
    su_perform_test_and_split(su);
}

/* ── revive / freeze ───────────────────────────────────────────────────── */

void ivp_sim_unit_revive_for_simulation(ivp_sim_unit_t *su, struct ivp_environment *env)
{
revive_all_cores:
    for (int i = ivp_pvec_len(&su->sim_unit_cores) - 1; i >= 0; i--) {
        ivp_core_t *my_core = su_core_at(su, i);
        IVP_ASSERT(!my_core->physical_unmoveable);
        if (my_core->movement_state >= IVP_MT_NOT_SIM) {
            bool fusion_was_done = ivp_core_revive_simulation_core(my_core);
            if (fusion_was_done) {
                /* the cores may have moved into another unit */
                su = my_core->sim_unit;
                goto revive_all_cores;
            }
        }
    }
    if (su->sim_unit_movement_type == IVP_MT_NOT_SIM) {
        ivp_sim_units_manager_t *m = ivp_env_get_sim_units_manager(env);
        ivp_sim_units_mgr_rem_sim_unit(m, su);
        su->sim_unit_movement_type = IVP_MT_MOVING;
        ivp_sim_units_mgr_add_sim_unit(m, su);
    }
}

void ivp_sim_unit_ensure_cores_movement(ivp_sim_unit_t *su)
{
    for (int i = ivp_pvec_len(&su->sim_unit_cores) - 1; i >= 0; i--) {
        ivp_core_reset_freeze_check_values(su_core_at(su, i));
    }
}

void ivp_sim_unit_ensure_in_simulation(ivp_sim_unit_t *su)
{
    if (su->sim_unit_movement_type >= IVP_MT_NOT_SIM) {
        IVP_ASSERT(ivp_pvec_len(&su->sim_unit_cores) > 0);
        ivp_sim_unit_revive_for_simulation(su, ivp_core_env(su_core_at(su, 0)));
    } else {
        ivp_sim_unit_ensure_cores_movement(su);
    }
}

static void su_clear_movement_check_values(ivp_sim_unit_t *su)
{
    for (int c = ivp_pvec_len(&su->sim_unit_cores) - 1; c >= 0; c--) {
        ivp_core_reset_freeze_check_values(su_core_at(su, c));
    }
}

static bool su_calc_movement_state_only(ivp_sim_unit_t *su,
                                        struct ivp_environment *env)
{
    int whole_sys = IVP_MT_CALM;
    ivp_time_t current_time = ivp_env_get_current_time(env);
    for (int c = ivp_pvec_len(&su->sim_unit_cores) - 1; c >= 0; c--) {
        ivp_core_t *core = su_core_at(su, c);
        core->movement_state = ivp_core_calc_movement_state(core, current_time);
        whole_sys = whole_sys & (int)core->movement_state;
    }
    return whole_sys == IVP_MT_CALM;
}

static void su_freeze_sim_unit(ivp_sim_unit_t *su,
                               struct ivp_environment *env)
{
    for (int n = ivp_pvec_len(&su->sim_unit_cores) - 1; n >= 0; n--) {
        ivp_core_freeze_simulation_core(su_core_at(su, n));
    }
    ivp_sim_units_manager_t *m = ivp_env_get_sim_units_manager(env);
    ivp_sim_units_mgr_rem_sim_unit(m, su);
    su->sim_unit_movement_type = IVP_MT_NOT_SIM;
    ivp_sim_units_mgr_add_sim_unit(m, su);
}

static bool su_calc_movement_state(ivp_sim_unit_t *su,
                                   struct ivp_environment *env)
{
    if (!su_calc_movement_state_only(su, env)) return false;
    su_freeze_sim_unit(su, env);
    return true;
}

bool ivp_sim_unit_sim_unit_calc_movement_state(ivp_sim_unit_t *su, struct ivp_environment *env)
{
    return su_calc_movement_state(su, env);
}

/* ── PSI ───────────────────────────────────────────────────────────────── */

/* IVP_Simulation_Unit::init_moving_core_for_psi */
static void su_init_moving_core_for_psi(ivp_core_t *core, ivp_time_t c_time)
{
    IVP_IF(1)
    {
        ivp_fr_info_for_core_t *info = ivp_core_moveable_core_has_friction_info(core);
        struct ivp_fr_system *fs = NULL;
        if (info) {
            fs = info->l_friction_system;
        }
        IVP_USE(fs);
    }

    IVP_ASSERT(!core->physical_unmoveable);
    double d_time = ivp_time_diff(c_time, core->time_of_last_psi);
    ivp_quat_to_mat3(&core->q_world_f_core_next_psi, &core->m_world_f_core_last_psi.rot);
    core->m_world_f_core_last_psi.pos.k[0] =
        (ivp_real_t)((double)core->pos_world_f_core_last_psi.k[0] + (double)core->delta_world_f_core_psis.k[0] * d_time);
    core->m_world_f_core_last_psi.pos.k[1] =
        (ivp_real_t)((double)core->pos_world_f_core_last_psi.k[1] + (double)core->delta_world_f_core_psis.k[1] * d_time);
    core->m_world_f_core_last_psi.pos.k[2] =
        (ivp_real_t)((double)core->pos_world_f_core_last_psi.k[2] + (double)core->delta_world_f_core_psis.k[2] * d_time);
}

static int su_fast_moving_flag(ivp_core_t *my_core)
{
    /* IVP_U_Float_Point::quad_length (float arithmetic) */
    const float sx = (float)my_core->speed.k[0], sy = (float)my_core->speed.k[1], sz = (float)my_core->speed.k[2];
    float ql = sx * sx + sy * sy + sz * sz;
    float fast_moving_core_p = (float)((double)(IVP_OBJECT_MOVING_FAST * IVP_OBJECT_MOVING_FAST) - (double)ql);
    int fast_moving_core;
    memcpy(&fast_moving_core, &fast_moving_core_p, sizeof(int));
    return fast_moving_core;
}

static bool su_prepare_single_sim_unit_psi(ivp_sim_unit_t *su,
                                           ivp_sim_es_t *es,
                                           bool movement_check_preassigned)
{
    struct ivp_environment *env = es->environment;
    es->sim_unit = su;
    int controller_num = ivp_pvec_len(&su->controller_cores);
    ivp_time_t current_time = ivp_env_get_current_time(env);
    int fast_moving_flag = 0;
    for (int d = ivp_pvec_len(&su->sim_unit_cores) - 1; d >= 0; d--) {
        ivp_core_t *my_core = su_core_at(su, d);
        su_init_moving_core_for_psi(my_core, current_time);
        IVP_IF(1)
        {
            for (int k = ivp_core_n_objects(my_core) - 1; k >= 0; k--) {
                IVP_ASSERT(ivp_core_object_at(my_core, k)->object_movement_state < IVP_MT_NOT_SIM);
            }
        }
        ivp_core_commit_all_async_pushes_ref(my_core);
        my_core->temporarily_unmovable = false;
        my_core->impacts_since_last_PSI = 0;
        fast_moving_flag |= su_fast_moving_flag(my_core);
    }
    if (fast_moving_flag < 0) {
        su->sim_unit_has_fast_objects = true;
    } else {
        su->sim_unit_just_slowed_down = su->sim_unit_has_fast_objects;
        if (su->sim_unit_just_slowed_down) su_clear_movement_check_values(su);
        su->sim_unit_has_fast_objects = false;
    }
    bool check_movement_state = movement_check_preassigned
                                    ? su->parallel_movement_check
                                    : ivp_env_must_perform_movement_check(env, su);
    for (int j = controller_num - 1; j >= 0; j--) {
        if (j >= ivp_pvec_len(&su->controller_cores)) continue;
        int debug_contr_prio;
        ivp_su_ctrl_list_t *cl = su_cl_at(su, j);
        ivp_sim_ctrl_t *c = cl->l_controller;
        IVP_IF(1)
        {
            debug_contr_prio = ivp_sim_ctrl_priority(c);
            IVP_USE(debug_contr_prio);
        }
        c->vt->do_simulation(c, es, &cl->cores_controlled_by);
        IVP_IF(1)
        {
            for (int tc = ivp_pvec_len(&su->sim_unit_cores) - 1; tc >= 0; tc--) {
                ivp_core_t *tcore = su_core_at(su, tc);
                ivp_core_core_plausible_check(tcore);
            }
        }
    }
    return check_movement_state;
}

static void su_finish_single_sim_unit_psi(ivp_sim_unit_t *su,
                                          struct ivp_environment *env,
                                          ivp_pvec_t *touched_cores,
                                          bool check_movement_state)
{
    for (int c = ivp_pvec_len(&su->sim_unit_cores) - 1; c >= 0; c--) {
        ivp_core_t *core = su_core_at(su, c);
        ivp_pvec_add(touched_cores, core);   /* calc_next_PSI_matrix */
        IVP_IF(0)
        {
            ivp_core_debug_vec_movement_state(core);
        }
    }
    if (check_movement_state) su_calc_movement_state(su, env);
    for (int k = ivp_pvec_len(&su->sim_unit_cores) - 1; k >= 0; k--) {
        ivp_core_t *my_core = su_core_at(su, k);
        for (int c = ivp_core_n_objects(my_core) - 1; c >= 0; c--) {
            ivp_object_recalc_invalid_mindists_of_object(ivp_core_object_at(my_core, c));
        }
    }
}

static void su_simulate_single_sim_unit_psi(ivp_sim_unit_t *su,
                                            ivp_sim_es_t *es,
                                            ivp_pvec_t *touched_cores)
{
#ifdef IVP_DEBUG_BUILD
    IVP_IF(1)
    {
        ivp_sim_unit_sim_unit_debug_consistency(su);
    }
#endif
    struct ivp_environment *env = es->environment;
    ivp_env_sim_mem_begin(env);
    bool check_movement_state =
        su_prepare_single_sim_unit_psi(su, es, false);
    su_finish_single_sim_unit_psi(su, env, touched_cores,
                                  check_movement_state);
    ivp_env_sim_mem_end(env);
}

typedef struct su_parallel_context {
    ivp_pvec_t *sim_units;
    ivp_pvec_t *touched_cores;
    ivp_sim_es_t event_sim;
} su_parallel_context_t;

static void su_parallel_simulate_task(void *opaque, size_t task_index)
{
    su_parallel_context_t *context = (su_parallel_context_t *)opaque;
    ivp_sim_unit_t *su = (ivp_sim_unit_t *)ivp_pvec_at(
        context->sim_units, (int)task_index);
    ivp_sim_es_t event_sim = context->event_sim;
    bool check_movement_state =
        su_prepare_single_sim_unit_psi(su, &event_sim, true);
    int output = su->parallel_touched_offset;
    for (int i = ivp_pvec_len(&su->sim_unit_cores) - 1; i >= 0; i--) {
        context->touched_cores->elems[output++] = su_core_at(su, i);
    }
    su->parallel_should_freeze =
        check_movement_state &&
        su_calc_movement_state_only(su, event_sim.environment);
}

/* These islands run no application callback, allocate no environment scratch,
 * and have no pending invalid-mindist work in the per-island finish. */
static bool su_is_parallel_island(const ivp_sim_unit_t *su)
{
    for (int i = ivp_pvec_len(&su->controller_cores) - 1; i >= 0; i--) {
        if (!ivp_env_sim_ctrl_is_parallel_safe(
                su_cl_at(su, i)->l_controller)) return false;
    }
    for (int i = ivp_pvec_len(&su->sim_unit_cores) - 1; i >= 0; i--) {
        const ivp_core_t *core = su_core_at(su, i);
        for (int j = ivp_core_n_objects(core) - 1; j >= 0; j--) {
            if (ivp_core_object_at(core, j)->invalid_synapses) return false;
        }
    }
    return true;
}

static bool su_try_parallel_island_run(struct ivp_environment *env,
                                       ivp_pvec_t *units,
                                       ivp_sim_es_t *es,
                                       ivp_pvec_t *touched_cores)
{
    const int unit_count = ivp_pvec_len(units);
    const size_t minimum_tasks = IVP_SU_PARALLEL_MIN_ISLANDS;
    if ((size_t)unit_count < minimum_tasks) return false;

    /* A few connected islands can contain much more work than hundreds of
     * one-core islands.  Below the conservative count-only threshold, use
     * the core references traversed by the preparation/controller loops as
     * a deterministic, topology-only cost estimate. */
    if ((size_t)unit_count < IVP_SU_PARALLEL_LIGHT_ISLANDS) {
        size_t work_units = 0u;
        for (int i = 0;
             i < unit_count && work_units < IVP_SU_PARALLEL_MIN_WORK_UNITS;
             i++) {
            ivp_sim_unit_t *su =
                (ivp_sim_unit_t *)ivp_pvec_at(units, i);
            size_t core_count =
                (size_t)ivp_pvec_len(&su->sim_unit_cores);
            size_t remaining = IVP_SU_PARALLEL_MIN_WORK_UNITS - work_units;
            work_units += core_count < remaining ? core_count : remaining;
            for (int j = 0;
                 j < ivp_pvec_len(&su->controller_cores) &&
                 work_units < IVP_SU_PARALLEL_MIN_WORK_UNITS;
                 j++) {
                ivp_su_ctrl_list_t *cl = su_cl_at(su, j);
                size_t controlled_count =
                    (size_t)ivp_pvec_len(&cl->cores_controlled_by);
                remaining = IVP_SU_PARALLEL_MIN_WORK_UNITS - work_units;
                work_units += controlled_count < remaining
                                  ? controlled_count
                                  : remaining;
            }
        }
        if (work_units < IVP_SU_PARALLEL_MIN_WORK_UNITS) return false;
    }

    int touched_count = ivp_pvec_len(touched_cores);
    for (int i = 0; i < unit_count; i++) {
        ivp_sim_unit_t *su = (ivp_sim_unit_t *)ivp_pvec_at(units, i);
        int core_count = ivp_pvec_len(&su->sim_unit_cores);
        if (core_count > INT_MAX - touched_count) return false;
        su->parallel_touched_offset = touched_count;
        touched_count += core_count;
    }
    if (!ivp_pvec_reserve(touched_cores, touched_count)) return false;

    /* Safe controllers do not consume the environment RNG, so movement
     * checks can be assigned now in the exact reference island order.  The
     * IVP_EDIT_LOCALITY checks are stateless; the restore below only
     * matters for the reference countdown. */
    short saved_next_movement_check = env->pl->next_movement_check;
    int saved_rand_seed = env->pl->rand_seed;
    for (int i = 0; i < unit_count; i++) {
        ivp_sim_unit_t *su = (ivp_sim_unit_t *)ivp_pvec_at(units, i);
        su->parallel_movement_check =
            ivp_env_must_perform_movement_check(env, su);
        su->parallel_should_freeze = false;
    }

    su_parallel_context_t context;
    context.sim_units = units;
    context.touched_cores = touched_cores;
    context.event_sim = *es;
    if (!ivp_env_parallel_for(env, IVP_PARALLEL_WORK_ISLAND,
                              (size_t)unit_count, minimum_tasks,
                              su_parallel_simulate_task, &context)) {
        env->pl->next_movement_check = saved_next_movement_check;
        env->pl->rand_seed = saved_rand_seed;
        return false;
    }
    touched_cores->n = touched_count;

    /* Freezing mutates manager lists, so only this small commit remains on
     * the PSI thread and follows the original island order. */
    for (int i = 0; i < unit_count; i++) {
        ivp_sim_unit_t *su = (ivp_sim_unit_t *)ivp_pvec_at(units, i);
        if (su->parallel_should_freeze) su_freeze_sim_unit(su, env);
    }
    return true;
}

static void su_flush_parallel_island_run(struct ivp_environment *env,
                                         ivp_pvec_t *units,
                                         ivp_sim_es_t *es,
                                         ivp_pvec_t *touched_cores)
{
    if (!su_try_parallel_island_run(env, units, es, touched_cores)) {
        for (int i = 0; i < ivp_pvec_len(units); i++) {
            su_simulate_single_sim_unit_psi(
                (ivp_sim_unit_t *)ivp_pvec_at(units, i), es,
                touched_cores);
        }
    }
    ivp_pvec_remove_all(units);
}

static void su_simulate_ordered_island(struct ivp_environment *env,
                                       ivp_sim_unit_t *su,
                                       ivp_pvec_t *units,
                                       ivp_sim_es_t *es,
                                       ivp_pvec_t *touched_cores)
{
    if (su_is_parallel_island(su)) {
        if (ivp_pvec_add(units, su) >= 0) return;

        /* Reusing the released capacity can still succeed after growing a
         * long run failed.  If the scratch vector itself cannot allocate,
         * preserve progress with the reference serial path. */
        su_flush_parallel_island_run(env, units, es, touched_cores);
        if (ivp_pvec_add(units, su) >= 0) return;
    } else {
        su_flush_parallel_island_run(env, units, es, touched_cores);
    }
    su_simulate_single_sim_unit_psi(su, es, touched_cores);
}

static bool su_simulate_segmented_parallel_islands(
    ivp_sim_units_manager_t *sman, ivp_sim_es_t *es,
    ivp_pvec_t *touched_cores)
{
    struct ivp_environment *env = sman->l_environment;
    if (ivp_env_parallel_worker_count(env) < 2) return false;

    ivp_pvec_t *units = &env->pl->parallel_sim_units;
    ivp_pvec_remove_all(units);
    ivp_sim_unit_t *s_u = sman->sim_units_slot0;
    if (!s_u) return true;
    ivp_sim_unit_t *n0_su = s_u->next_sim_unit;
    if (!n0_su) {
        su_simulate_ordered_island(env, s_u, units, es, touched_cores);
        goto finish;
    }
    ivp_sim_unit_t *n1_su = n0_su->next_sim_unit;
    if (!n1_su) {
        su_simulate_ordered_island(env, n0_su, units, es,
                                   touched_cores);
        su_simulate_ordered_island(env, s_u, units, es, touched_cores);
        goto finish;
    }
    ivp_sim_unit_t *n2_su = n1_su->next_sim_unit;
    while (n2_su) {
        su_simulate_ordered_island(env, s_u, units, es, touched_cores);
        s_u = n0_su;
        n0_su = n1_su;
        n1_su = n2_su;
        n2_su = n2_su->next_sim_unit;
    }
    su_simulate_ordered_island(env, n1_su, units, es, touched_cores);
    su_simulate_ordered_island(env, n0_su, units, es, touched_cores);
    su_simulate_ordered_island(env, s_u, units, es, touched_cores);

finish:
    su_flush_parallel_island_run(env, units, es, touched_cores);
    return true;
}

void ivp_sim_units_mgr_simulate_sim_units_psi(ivp_sim_units_manager_t *sman, ivp_pvec_t *touched_cores)
{
    mgr_restore_unit_ordering(sman);
    ivp_sim_es_t es;
    ivp_sim_es_init(&es, sman->l_environment);
    ivp_sim_unit_t *s_u, *n0_su, *n1_su, *n2_su;
    if (su_simulate_segmented_parallel_islands(sman, &es, touched_cores)) {
        goto sim_units_0;
    }
    s_u = sman->sim_units_slot0;
    if (!s_u) goto sim_units_0;
    n0_su = s_u->next_sim_unit;
    if (!n0_su) goto sim_units_1;
    n1_su = n0_su->next_sim_unit;
    if (!n1_su) goto sim_units_2;
    n2_su = n1_su->next_sim_unit;
    while (n2_su) {
        su_simulate_single_sim_unit_psi(s_u, &es, touched_cores);
        s_u = n0_su;
        n0_su = n1_su;
        n1_su = n2_su;
        n2_su = n2_su->next_sim_unit;
    }
    su_simulate_single_sim_unit_psi(n1_su, &es, touched_cores);
sim_units_2:
    su_simulate_single_sim_unit_psi(n0_su, &es, touched_cores);
sim_units_1:
    su_simulate_single_sim_unit_psi(s_u, &es, touched_cores);
sim_units_0:
    /* IVP_Sim_Units_Manager scans both lists in this order. If no linked
     * unit is dirty, both loops can only read links/flags: skip the empty
     * work. Otherwise retain the complete reference traversal (including
     * its saved next pointers and treatment of newly split units). */
    if (!sman->pending_union_find_count) return;
    for (s_u = sman->sim_units_slot0; s_u;) {
        ivp_sim_unit_t *next_su = s_u->next_sim_unit;
        if (s_u->union_find_needed_for_sim_unit) ivp_sim_unit_do_sim_unit_union_find(s_u);
        s_u = next_su;
    }
    for (s_u = sman->still_slot; s_u;) {
        ivp_sim_unit_t *next_su = s_u->next_sim_unit;
        if (s_u->union_find_needed_for_sim_unit) ivp_sim_unit_do_sim_unit_union_find(s_u);
        s_u = next_su;
    }
}

static void su_reset_time(ivp_sim_unit_t *su, double offset)
{
    for (int j = ivp_pvec_len(&su->controller_cores) - 1; j >= 0; j--) {
        ivp_sim_ctrl_t *c = su_cl_at(su, j)->l_controller;
        if (c->vt->reset_time) c->vt->reset_time(c, offset);
    }
    for (int c = ivp_pvec_len(&su->sim_unit_cores) - 1; c >= 0; c--) {
        ivp_core_reset_time(su_core_at(su, c), offset);
    }
}

void ivp_sim_units_mgr_reset_time(ivp_sim_units_manager_t *m, double offset)
{
    mgr_restore_unit_ordering(m);
    for (ivp_sim_unit_t *s = m->sim_units_slot0; s; s = s->next_sim_unit) su_reset_time(s, offset);
    for (ivp_sim_unit_t *s = m->still_slot; s; s = s->next_sim_unit) su_reset_time(s, offset);
}

/* ── IVP_Controller_Manager ────────────────────────────────────────────── */

bool ivp_ctrl_mgr_core_has_controller(ivp_core_t *core, ivp_sim_ctrl_t *c)
{
    if (!core) return false;
    for (int i = ivp_pvec_len(&core->controllers_of_core) - 1; i >= 0; i--) {
        if (ivp_pvec_at(&core->controllers_of_core, i) == c) return true;
    }
    return false;
}

bool ivp_ctrl_mgr_add_controller_to_core(ivp_sim_ctrl_t *c, ivp_core_t *core)
{
    if (!core || !c) return false;
    if (ivp_ctrl_mgr_core_has_controller(core, c)) return true;
    return ivp_core_add_core_controller(core, c);
}

void ivp_ctrl_mgr_remove_controller_from_core(ivp_sim_ctrl_t *c, ivp_core_t *core)
{
    if (!core || !c) return;
    if (!ivp_ctrl_mgr_core_has_controller(core, c)) return;
    ivp_core_rem_core_controller(core, c);
}

void ivp_ctrl_mgr_remove_controller_from_environment(struct ivp_environment *env, ivp_sim_ctrl_t *c, bool silently)
{
    IVP_UNUSED(env);
    ivp_pvec_t *controlled_cores = c->vt->get_associated_controlled_cores(c);
    if (!controlled_cores) return;
    ivp_sim_unit_t *reference_unit = NULL;
    bool multiple_units = false;
    for (int i = ivp_pvec_len(controlled_cores) - 1; i >= 0; i--) {
        ivp_core_t *my_core = (ivp_core_t *)ivp_pvec_at(controlled_cores, i);
        if (!my_core) continue;
        if (!ivp_ctrl_mgr_core_has_controller(my_core, c)) continue;
        ivp_sim_unit_t *core_unit = my_core->sim_unit;
        if (!core_unit) {
            IVP_ASSERT(0);
            ivp_pvec_remove(&my_core->controllers_of_core, c);
            continue;
        }
        ivp_core_rem_core_controller(my_core, c);
        ivp_sim_unit_request_union_find(core_unit);
        if (!reference_unit) {
            reference_unit = core_unit;
        } else if (reference_unit != core_unit) {
            multiple_units = true;
        }
    }
    if (silently || !reference_unit) return;
    if (!multiple_units) {
        ivp_sim_unit_ensure_in_simulation(reference_unit);
        return;
    }

    /* A committed dependent controller normally owns one fused simulation
     * unit.  Preserve the reference order without scratch storage if a
     * partially constructed internal controller spans more than one. */
    for (int i = 0; i < ivp_pvec_len(controlled_cores); i++) {
        ivp_core_t *core = (ivp_core_t *)ivp_pvec_at(controlled_cores, i);
        ivp_sim_unit_t *unit = core ? core->sim_unit : NULL;
        if (!unit || !unit->union_find_needed_for_sim_unit) continue;
        bool appears_later = false;
        for (int j = i + 1; j < ivp_pvec_len(controlled_cores); j++) {
            ivp_core_t *other =
                (ivp_core_t *)ivp_pvec_at(controlled_cores, j);
            if (other && other->sim_unit == unit) {
                appears_later = true;
                break;
            }
        }
        if (!appears_later) ivp_sim_unit_ensure_in_simulation(unit);
    }
}

void ivp_ctrl_mgr_ensure_controller_in_simulation(struct ivp_environment *env, ivp_sim_ctrl_t *c)
{
    (void)env;
    ivp_pvec_t *controlled_cores = c->vt->get_associated_controlled_cores(c);
    IVP_ASSERT(controlled_cores && ivp_pvec_len(controlled_cores) > 0);
    if (!controlled_cores) return;
    ivp_core_t *fallback_core = NULL;
    for (int i = 0; i < ivp_pvec_len(controlled_cores); i++) {
        ivp_core_t *core = (ivp_core_t *)ivp_pvec_at(controlled_cores, i);
        if (!core || core->physical_unmoveable) continue;
        if (!fallback_core) fallback_core = core;
        if (!core->sim_unit) continue;
        ivp_sim_unit_ensure_in_simulation(core->sim_unit);
        return;
    }
    if (fallback_core) ivp_core_ensure_core_in_simulation_delayed(fallback_core);
}

static bool ctrl_list_contains(const ivp_pvec_t *cores,
                               const ivp_core_t *core)
{
    return cores && ivp_pvec_index_of(cores, core) >= 0;
}

static void ctrl_remove_cores_not_in(ivp_sim_ctrl_t *controller,
                                     const ivp_pvec_t *cores,
                                     const ivp_pvec_t *keep)
{
    if (!cores) return;
    for (int i = ivp_pvec_len(cores) - 1; i >= 0; i--) {
        ivp_core_t *core = (ivp_core_t *)ivp_pvec_at(cores, i);
        if (!core || ctrl_list_contains(keep, core) ||
            !ivp_ctrl_mgr_core_has_controller(core, controller)) {
            continue;
        }
        ivp_sim_unit_t *unit = core->sim_unit;
        ivp_core_rem_core_controller(core, controller);
        if (unit) ivp_sim_unit_request_union_find(unit);
    }
}

static bool ctrl_commit_controller_cores(
    struct ivp_environment *env, ivp_sim_ctrl_t *c,
    const ivp_pvec_t *previous_cores)
{
    ivp_pvec_t *controlled_cores = c->vt->get_associated_controlled_cores(c);
    if (!controlled_cores) return true;
    /* Keep fused source units intact until the rebuilt controller index
     * commits.  That makes every allocation failure below reversible. */
    ivp_pvec_t deferred_units;
    ivp_pvec_init_with_allocator(&deferred_units, ivp_env_get_allocator(env));
    ivp_sim_unit_t *reference_unit = NULL;
    bool did_fusion = false;
    bool success = true;
    int mtype = IVP_MT_NOT_SIM;
    for (int i = ivp_pvec_len(controlled_cores) - 1; i >= 0; i--) {
        ivp_core_t *test_core = (ivp_core_t *)ivp_pvec_at(controlled_cores, i);
        if (!test_core || test_core->physical_unmoveable) continue;
        mtype = mtype & (int)test_core->movement_state;
        ivp_sim_unit_t *test_sim_unit = test_core->sim_unit;
        if (!test_sim_unit) {
            IVP_ASSERT(0);
            ivp_core_ensure_core_in_simulation_delayed(test_core);
            continue;
        }
        if (reference_unit != NULL) {
            if (test_sim_unit != reference_unit) {
                if (ivp_pvec_add(&deferred_units, test_sim_unit) < 0) {
                    success = false;
                    break;
                }
                if (!ivp_sim_unit_throw_cores_into_my_sim_unit(reference_unit, test_sim_unit)) {
                    ivp_pvec_remove_at(&deferred_units,
                                       ivp_pvec_len(&deferred_units) - 1);
                    success = false;
                    break;
                }
                did_fusion = true;
            }
        } else {
            reference_unit = test_sim_unit;
        }
        if (!ivp_ctrl_mgr_core_has_controller(test_core, c) &&
            !ivp_core_add_core_controller(test_core, c)) {
            success = false;
            break;
        }
    }
    if (success && did_fusion && reference_unit) {
        if (!ivp_sim_unit_sim_unit_calc_redundants(reference_unit)) {
            success = false;
        }
    }

    if (!success) {
        if (previous_cores) {
            ctrl_remove_cores_not_in(c, controlled_cores, previous_cores);
        } else {
            ivp_ctrl_mgr_remove_controller_from_environment(env, c, true);
        }
        for (int i = ivp_pvec_len(&deferred_units) - 1; i >= 0; i--) {
            ivp_sim_unit_t *restored =
                (ivp_sim_unit_t *)ivp_pvec_at(&deferred_units, i);
            for (int j = ivp_pvec_len(&restored->sim_unit_cores) - 1;
                 j >= 0; j--) {
                int last = ivp_pvec_len(&reference_unit->sim_unit_cores) - 1;
                ivp_core_t *core = su_core_at(restored, j);
                IVP_ASSERT(last >= 0 && su_core_at(reference_unit, last) == core);
                if (last >= 0) su_remove_core_at(reference_unit, last);
                core->sim_unit = restored;
            }
            ivp_sim_units_mgr_add_sim_unit(ivp_env_get_sim_units_manager(env), restored);
        }
    } else {
        for (int i = ivp_pvec_len(&deferred_units) - 1; i >= 0; i--) {
            ivp_sim_unit_delete((ivp_sim_unit_t *)ivp_pvec_at(&deferred_units, i));
        }
        ctrl_remove_cores_not_in(c, previous_cores, controlled_cores);
    }
    ivp_pvec_clear(&deferred_units);

    if (success && mtype < IVP_MT_NOT_SIM && reference_unit) {
        ivp_sim_unit_revive_for_simulation(reference_unit, env);
    }
    return success;
}

bool ivp_ctrl_mgr_announce_controller_to_environment(
    struct ivp_environment *env, ivp_sim_ctrl_t *c)
{
    return ctrl_commit_controller_cores(env, c, NULL);
}

bool ivp_ctrl_mgr_replace_controller_cores(
    struct ivp_environment *env, ivp_sim_ctrl_t *c,
    const ivp_pvec_t *previous_cores)
{
    if (!previous_cores) return false;
    return ctrl_commit_controller_cores(env, c, previous_cores);
}
