/* ivp_sim_unit.c — port of IVP_Simulation_Unit / IVP_Sim_Units_Manager /
 * IVP_Controller_Manager (ivp_intern/ivp_sim_unit.cxx)
 */
#include <stdlib.h>
#include <string.h>

#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <ivp/dynamics/ivp_sim_unit.h>
#include "dynamics/ivp_sim_internal.h"
#include "solver/ivp_friction_internal.h"

static inline ivp_core_t *su_core_at(const ivp_sim_unit_t *su, int i) {
    return (ivp_core_t *)ivp_pvec_at(&su->sim_unit_cores, i);
}
static inline ivp_su_ctrl_list_t *su_cl_at(const ivp_sim_unit_t *su, int i) {
    return (ivp_su_ctrl_list_t *)ivp_pvec_at(&su->controller_cores, i);
}

/* ── lifecycle ─────────────────────────────────────────────────────────── */

ivp_sim_unit_t *ivp_sim_unit_new(void)
{
    ivp_sim_unit_t *su = (ivp_sim_unit_t *)calloc(1, sizeof(*su));
    if (!su) return NULL;
    su->union_find_needed_for_sim_unit = false;
    su->sim_unit_movement_type = IVP_MT_NOT_SIM;
    su->sim_unit_just_slowed_down = false;
    su->sim_unit_has_fast_objects = false;
    su->prev_sim_unit = NULL;
    su->next_sim_unit = NULL;
    ivp_pvec_init(&su->sim_unit_cores);
    ivp_pvec_init(&su->controller_cores);
    return su;
}

static void su_cl_delete(ivp_su_ctrl_list_t *cl)
{
    ivp_pvec_clear(&cl->cores_controlled_by);
    free(cl);
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
    ivp_sim_unit_clean_sim_unit(su);
    ivp_pvec_clear(&su->controller_cores);
    ivp_pvec_clear(&su->sim_unit_cores);
    free(su);
}

/* ── controller lists ──────────────────────────────────────────────────── */

static bool su_controller_is_known(ivp_sim_unit_t *su, ivp_sim_ctrl_t *c)
{
    for (int i = ivp_pvec_len(&su->controller_cores) - 1; i >= 0; i--) {
        if (su_cl_at(su, i)->l_controller == c) return true;
    }
    return false;
}

static void su_add_controlled_core_for_controller(ivp_sim_unit_t *su, ivp_sim_ctrl_t *c, ivp_core_t *core)
{
    int i;
    for (i = ivp_pvec_len(&su->controller_cores) - 1; i >= 0; i--) {
        if (su_cl_at(su, i)->l_controller == c) break;
    }
    IVP_ASSERT(i >= 0);
    if (i < 0) return;
    ivp_pvec_install(&su_cl_at(su, i)->cores_controlled_by, core);
}

static void su_add_controller_unit_sim(ivp_sim_unit_t *su, ivp_sim_ctrl_t *c)
{
    ivp_su_ctrl_list_t *cl = (ivp_su_ctrl_list_t *)calloc(1, sizeof(*cl));
    if (!cl) return;
    cl->l_controller = c;
    ivp_pvec_init(&cl->cores_controlled_by);
    ivp_pvec_add(&su->controller_cores, cl);
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

void ivp_sim_unit_sim_unit_calc_redundants(ivp_sim_unit_t *su)
{
    for (int i = ivp_pvec_len(&su->sim_unit_cores) - 1; i >= 0; i--) {
        ivp_core_t *my_core = su_core_at(su, i);
        for (int k = ivp_pvec_len(&my_core->controllers_of_core) - 1; k >= 0; k--) {
            ivp_sim_ctrl_t *c = (ivp_sim_ctrl_t *)ivp_pvec_at(&my_core->controllers_of_core, k);
            if (!su_controller_is_known(su, c)) su_add_controller_unit_sim(su, c);
            su_add_controlled_core_for_controller(su, c, my_core);
        }
    }
    su_sort_controllers(su);
}

static void su_rem_sim_unit_controller(ivp_sim_unit_t *su, ivp_sim_ctrl_t *c)
{
    int pos = su_get_pos_of_controller(su, c);
    IVP_ASSERT(pos >= 0);
    if (pos < 0) return;
    su_cl_delete(su_cl_at(su, pos));
    ivp_pvec_remove_at(&su->controller_cores, pos);
}

void ivp_sim_unit_add_controller_of_core(ivp_sim_unit_t *su, ivp_core_t *core, ivp_sim_ctrl_t *c)
{
    if (!su_controller_is_known(su, c)) su_add_controller_unit_sim(su, c);
    su_add_controlled_core_for_controller(su, c, core);
    su_sort_controllers(su);
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

void ivp_sim_unit_add_sim_unit_core(ivp_sim_unit_t *su, ivp_core_t *core) { ivp_pvec_add(&su->sim_unit_cores, core); }
void ivp_sim_unit_rem_sim_unit_core(ivp_sim_unit_t *su, ivp_core_t *core) { ivp_pvec_remove(&su->sim_unit_cores, core); }

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

            IVP_ASSERT(su_controller_is_known(su, my_controller) == true);

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
    ivp_sim_unit_t **slot = (su->sim_unit_movement_type < IVP_MT_NOT_SIM) ? &m->sim_units_slot0 : &m->still_slot;
    mgr_add_unit_to_slot(su, slot);
}

void ivp_sim_units_mgr_rem_sim_unit(ivp_sim_units_manager_t *m, ivp_sim_unit_t *su)
{
    ivp_sim_unit_t **slot = (su->sim_unit_movement_type < IVP_MT_NOT_SIM) ? &m->sim_units_slot0 : &m->still_slot;
    mgr_rem_unit_from_slot(su, slot);
}

/* ── merge / split / union find ────────────────────────────────────────── */

void ivp_sim_unit_throw_cores_into_my_sim_unit(ivp_sim_unit_t *su, ivp_sim_unit_t *second_unit)
{
    struct ivp_environment *env = NULL;
    for (int i = 0; i < ivp_pvec_len(&second_unit->sim_unit_cores); i++) {
        ivp_core_t *my_core = su_core_at(second_unit, i);
        ivp_sim_unit_add_sim_unit_core(su, my_core);
        env = ivp_core_env(my_core);
        my_core->sim_unit = su;
    }
    if (env) ivp_sim_units_mgr_rem_sim_unit(ivp_env_get_sim_units_manager(env), second_unit);
}

void ivp_sim_unit_fusion_simulation_unities(ivp_sim_unit_t *su, ivp_sim_unit_t *second_unit)
{
    ivp_sim_unit_clean_sim_unit(su);
    ivp_sim_unit_throw_cores_into_my_sim_unit(su, second_unit);
    ivp_sim_unit_sim_unit_calc_redundants(su);
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
    ivp_sim_unit_t *split_new_unit = ivp_sim_unit_new();
    if (!split_new_unit) return;
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
            ivp_pvec_remove_at(&su->sim_unit_cores, i);
            i--;
            ivp_pvec_add(&split_new_unit->sim_unit_cores, my_core);
            my_core->sim_unit = split_new_unit;
        }
    }
    ivp_sim_unit_sim_unit_calc_redundants(split_new_unit);
#ifdef IVP_DEBUG_BUILD
    IVP_IF(1)
    {
        ivp_sim_unit_sim_unit_debug_consistency(split_new_unit);
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
        ivp_sim_unit_sim_unit_calc_redundants(su);
#ifdef IVP_DEBUG_BUILD
        ivp_sim_unit_sim_unit_debug_consistency(su);
#endif
    }
}

void ivp_sim_unit_do_sim_unit_union_find(ivp_sim_unit_t *su)
{
    ivp_sim_unit_clean_sim_unit(su);
    ivp_sim_unit_sim_unit_calc_redundants(su);
    su_perform_test_and_split(su);
    su->union_find_needed_for_sim_unit = false;
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

static bool su_calc_movement_state(ivp_sim_unit_t *su, struct ivp_environment *env)
{
    int whole_sys = IVP_MT_CALM;
    ivp_time_t current_time = ivp_env_get_current_time(env);
    for (int c = ivp_pvec_len(&su->sim_unit_cores) - 1; c >= 0; c--) {
        ivp_core_t *core = su_core_at(su, c);
        core->movement_state = ivp_core_calc_movement_state(core, current_time);
        whole_sys = whole_sys & (int)core->movement_state;
    }
    if (whole_sys == IVP_MT_CALM) {
        for (int n = ivp_pvec_len(&su->sim_unit_cores) - 1; n >= 0; n--) {
            ivp_core_freeze_simulation_core(su_core_at(su, n));
        }
        ivp_sim_units_manager_t *m = ivp_env_get_sim_units_manager(env);
        ivp_sim_units_mgr_rem_sim_unit(m, su);
        su->sim_unit_movement_type = IVP_MT_NOT_SIM;
        ivp_sim_units_mgr_add_sim_unit(m, su);
        return true;
    }
    return false;
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

static void su_simulate_single_sim_unit_psi(ivp_sim_unit_t *su, ivp_sim_es_t *es, ivp_pvec_t *touched_cores)
{
#ifdef IVP_DEBUG_BUILD
    IVP_IF(1)
    {
        ivp_sim_unit_sim_unit_debug_consistency(su);
    }
#endif
    struct ivp_environment *env = es->environment;
    es->sim_unit = su;
    ivp_env_sim_mem_begin(env);
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
    bool check_movement_state;
    if (fast_moving_flag < 0) {
        su->sim_unit_has_fast_objects = true;
        check_movement_state = ivp_env_must_perform_movement_check(env);
    } else {
        su->sim_unit_just_slowed_down = su->sim_unit_has_fast_objects;
        if (su->sim_unit_just_slowed_down) su_clear_movement_check_values(su);
        su->sim_unit_has_fast_objects = false;
        check_movement_state = ivp_env_must_perform_movement_check(env);
    }
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
    ivp_env_sim_mem_end(env);
}

void ivp_sim_units_mgr_simulate_sim_units_psi(ivp_sim_units_manager_t *sman, ivp_pvec_t *touched_cores)
{
    ivp_sim_es_t es;
    ivp_sim_es_init(&es, sman->l_environment);
    ivp_sim_unit_t *s_u, *n0_su, *n1_su, *n2_su;
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

void ivp_ctrl_mgr_add_controller_to_core(ivp_sim_ctrl_t *c, ivp_core_t *core)
{
    if (!core || !c) return;
    if (ivp_ctrl_mgr_core_has_controller(core, c)) return;
    ivp_core_add_core_controller(core, c);
}

void ivp_ctrl_mgr_remove_controller_from_core(ivp_sim_ctrl_t *c, ivp_core_t *core)
{
    if (!core || !c) return;
    if (!ivp_ctrl_mgr_core_has_controller(core, c)) return;
    ivp_core_rem_core_controller(core, c);
}

static bool su_was_touched(ivp_pvec_t *units, ivp_sim_unit_t *unit)
{
    for (int i = ivp_pvec_len(units) - 1; i >= 0; i--) {
        if (ivp_pvec_at(units, i) == unit) return true;
    }
    return false;
}

void ivp_ctrl_mgr_remove_controller_from_environment(struct ivp_environment *env, ivp_sim_ctrl_t *c, bool silently)
{
    (void)env;
    ivp_pvec_t *controlled_cores = c->vt->get_associated_controlled_cores(c);
    if (!controlled_cores) return;
    ivp_pvec_t touched_units;
    ivp_pvec_init(&touched_units);
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
        if (!su_was_touched(&touched_units, core_unit)) ivp_pvec_add(&touched_units, core_unit);
    }
    for (int i = ivp_pvec_len(&touched_units) - 1; i >= 0; i--) {
        ivp_sim_unit_t *reference_unit = (ivp_sim_unit_t *)ivp_pvec_at(&touched_units, i);
        reference_unit->union_find_needed_for_sim_unit = true;
        if (!silently) ivp_sim_unit_ensure_in_simulation(reference_unit);
    }
    ivp_pvec_clear(&touched_units);
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

void ivp_ctrl_mgr_announce_controller_to_environment(struct ivp_environment *env, ivp_sim_ctrl_t *c)
{
    ivp_pvec_t *controlled_cores = c->vt->get_associated_controlled_cores(c);
    if (!controlled_cores) return;
    ivp_sim_unit_t *reference_unit = NULL;
    bool did_fusion = false;
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
                ivp_sim_unit_throw_cores_into_my_sim_unit(reference_unit, test_sim_unit);
                ivp_sim_unit_delete(test_sim_unit);
                did_fusion = true;
            }
        } else {
            reference_unit = test_sim_unit;
        }
        if (!ivp_ctrl_mgr_core_has_controller(test_core, c)) ivp_core_add_core_controller(test_core, c);
    }
    if (did_fusion && reference_unit) {
        ivp_sim_unit_clean_sim_unit(reference_unit);
        ivp_sim_unit_sim_unit_calc_redundants(reference_unit);
    }
    if (mtype < IVP_MT_NOT_SIM && reference_unit) {
        ivp_sim_unit_revive_for_simulation(reference_unit, env);
    }
}
