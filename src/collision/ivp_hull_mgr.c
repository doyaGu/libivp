/* ivp_hull_mgr.c — port of IVP_Hull_Manager (ivp_hull_manager*.?xx) */
#include <ivp/collision/ivp_hull_mgr.h>
#include <ivp/base/ivp_assert.h>
#include "dynamics/ivp_env_internal.h"

void ivp_hull_mgr_init(ivp_hull_mgr_t *hm, const ivp_allocator_t *alloc)
{
    hm->last_vpsi_time = 0.0;
    hm->gradient = 0.0f;
    hm->center_gradient = 0.0f;
    hm->hull_value_last_vpsi = 0.0f;
    hm->hull_center_value_last_vpsi = 0.0f;
    hm->hull_value_next_psi = 0.0f;
    hm->time_of_next_reset = 0;
    ivp_minlist_init(&hm->sorted_synapses, alloc, 8);
}

void ivp_hull_mgr_delete_listeners(ivp_hull_mgr_t *hm)
{
    while (ivp_minlist_has_elements(&hm->sorted_synapses)) {
        ivp_listener_hull_t *syn = (ivp_listener_hull_t *)ivp_minlist_find_min_elem(&hm->sorted_synapses);
        syn->vt->hull_manager_is_going_to_be_deleted_event(syn, hm);
    }
}

void ivp_hull_mgr_destroy(ivp_hull_mgr_t *hm)
{
    IVP_ASSERT(!ivp_minlist_has_elements(&hm->sorted_synapses));
    ivp_minlist_destroy(&hm->sorted_synapses);
}

float ivp_hull_mgr_insert_synapse(ivp_hull_mgr_t *hm, ivp_listener_hull_t *syn,
                                  ivp_time_t t_now, double delta_valid_hull_time)
{
    float dt = (float)ivp_time_diff(t_now, hm->last_vpsi_time);
    float val = hm->hull_value_last_vpsi + hm->gradient * dt;
    syn->minlist_index = ivp_minlist_add(&hm->sorted_synapses, syn, (float)((double)val + delta_valid_hull_time));
    return (hm->hull_value_last_vpsi - hm->hull_center_value_last_vpsi) + (hm->gradient - hm->center_gradient) * dt;
}

float ivp_hull_mgr_insert_lazy_synapse(ivp_hull_mgr_t *hm, ivp_listener_hull_t *syn,
                                       ivp_time_t t_now, double delta_valid_hull_time)
{
    (void)t_now;
    float val = hm->hull_value_next_psi;
    syn->minlist_index = ivp_minlist_add(&hm->sorted_synapses, syn, (float)((double)val + delta_valid_hull_time));
    return hm->hull_center_value_last_vpsi;
}

void ivp_hull_mgr_remove_synapse(ivp_hull_mgr_t *hm, ivp_listener_hull_t *syn)
{
    ivp_minlist_remove(&hm->sorted_synapses, (ivp_minlist_index_t)syn->minlist_index);
}

void ivp_hull_mgr_update_synapse(ivp_hull_mgr_t *hm, ivp_listener_hull_t *syn,
                                 ivp_time_t t_now, double delta_valid_hull_time)
{
    ivp_minlist_remove(&hm->sorted_synapses, (ivp_minlist_index_t)syn->minlist_index);
    float dt = (float)ivp_time_diff(t_now, hm->last_vpsi_time);
    float val = hm->hull_value_last_vpsi + hm->gradient * dt;
    syn->minlist_index = ivp_minlist_add(&hm->sorted_synapses, syn, (float)((double)val + delta_valid_hull_time));
}

void ivp_hull_mgr_update_lazy_synapse(ivp_hull_mgr_t *hm, ivp_listener_hull_t *syn,
                                      ivp_time_t t_now, double delta_valid_hull_time)
{
    (void)t_now;
    ivp_minlist_remove(&hm->sorted_synapses, (ivp_minlist_index_t)syn->minlist_index);
    float val = hm->hull_value_next_psi;
    syn->minlist_index = ivp_minlist_add(&hm->sorted_synapses, syn, (float)((double)val + delta_valid_hull_time));
}

void ivp_hull_mgr_increase_hull_by_x(ivp_hull_mgr_t *hm, ivp_time_t t_now, float delta_time,
                                     float gradient_, float center_gradient_)
{
    float dt = (float)ivp_time_diff(t_now, hm->last_vpsi_time);
    hm->hull_value_last_vpsi += hm->gradient * dt;
    hm->hull_center_value_last_vpsi += hm->center_gradient * dt;
    hm->last_vpsi_time = t_now;
    hm->gradient = gradient_ * IVP_HULL_MANAGER_GRADIENT_FACTOR;
    hm->center_gradient = center_gradient_;
    float delta = delta_time * hm->gradient;
    hm->hull_value_next_psi = hm->hull_value_last_vpsi + delta;
    IVP_ASSERT(hm->hull_value_next_psi >= 0.0f);
}

void ivp_hull_mgr_check_hull_synapses(ivp_hull_mgr_t *hm, struct ivp_environment *env)
{
    float syn_val;
    int maxcnt = ivp_env_get_max_collision_checks_per_psi(env);
    while ((syn_val = ivp_minlist_find_min_value(&hm->sorted_synapses) - hm->hull_value_next_psi) < 0.0f) {
        ivp_listener_hull_t *syn = (ivp_listener_hull_t *)ivp_minlist_find_min_elem(&hm->sorted_synapses);
        syn->vt->hull_limit_exceeded_event(syn, hm, syn_val);
        if (maxcnt-- < 0) break;
    }
}

static void ivp_hull_mgr_reset_times(ivp_hull_mgr_t *hm)
{
    float mt = -hm->hull_value_last_vpsi;
    float mc = -hm->hull_center_value_last_vpsi;
    ivp_minlist_t *ml = &hm->sorted_synapses;
    for (unsigned int i = ml->first_element; i != IVP_MINLIST_UNUSED; i = ml->elems[i].next) {
        ivp_minlist_elem_t *el = &ml->elems[i];
        ivp_listener_hull_t *lh = (ivp_listener_hull_t *)el->element;
        el->value += mt;
        if (lh->vt->hull_manager_is_reset) lh->vt->hull_manager_is_reset(lh, mt, mc);
    }
    ml->min_value += mt;
    hm->hull_value_last_vpsi = 0.0f;
    hm->hull_center_value_last_vpsi = 0.0f;
    hm->hull_value_next_psi += mt;
}

void ivp_hull_mgr_check_for_reset(ivp_hull_mgr_t *hm)
{
    if (hm->last_vpsi_time > (double)hm->time_of_next_reset) {
        ivp_hull_mgr_reset_times(hm);
        hm->time_of_next_reset = (int)(hm->last_vpsi_time + (double)IVP_MAX_TIME_WITHOUT_RESET);
    }
}

void ivp_hull_mgr_reset_time(ivp_hull_mgr_t *hm, ivp_time_t offset)
{
    hm->last_vpsi_time -= offset;
}

void ivp_hull_mgr_exit_hull_for_simulation(ivp_hull_mgr_t *hm, ivp_time_t t_now)
{
    double dt = ivp_time_diff(t_now, hm->last_vpsi_time);
    hm->hull_value_last_vpsi = (float)((double)hm->hull_value_last_vpsi + (double)hm->gradient * dt);
    hm->hull_center_value_last_vpsi = (float)((double)hm->hull_center_value_last_vpsi + (double)hm->center_gradient * dt);
    hm->gradient = 0.0f;
    hm->center_gradient = 0.0f;
    ivp_hull_mgr_reset_times(hm);
}

void ivp_hull_mgr_jump_add_hull(ivp_hull_mgr_t *hm, float delta_x, float delta_center_x)
{
    hm->hull_value_last_vpsi = hm->hull_value_next_psi = hm->hull_value_next_psi + delta_x;
    IVP_ASSERT(hm->hull_value_next_psi >= 0.0f);
    hm->hull_center_value_last_vpsi += delta_center_x;
    hm->gradient = 0.0f;
    hm->center_gradient = 0.0f;
}
