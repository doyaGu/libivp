/* ivp_time_manager.c — port of IVP_Time_Manager / IVP_Event_Manager_Standard */
#include <ivp/dynamics/ivp_time_manager.h>
#include <ivp/base/ivp_assert.h>
#include "dynamics/ivp_env_internal.h"

/* environment hooks (ivp_environment.c) */
void ivp_environment_set_current_time_internal(struct ivp_environment *env, ivp_time_t time);
void ivp_environment_set_delta_psi_time_internal(struct ivp_environment *env, double psi_time);

ivp_result_t ivp_time_manager_init(ivp_time_manager_t *tm, const ivp_allocator_t *alloc,
                                   ivp_time_event_fn_t psi_fn)
{
    if (!tm || !psi_fn) return IVP_ERR_INVALID_ARG;
    ivp_minlist_init(&tm->min_hash, alloc, 16);
    if (!tm->min_hash.elems) return IVP_ERR_OUT_OF_MEMORY;
    ivp_time_event_init(&tm->psi_event, psi_fn);
    tm->base_time = 0.0;
    tm->last_time = 0.0;
    ivp_time_manager_insert_event(tm, &tm->psi_event, 0.0);
    if (tm->psi_event.index == IVP_MINLIST_UNUSED) {
        ivp_minlist_destroy(&tm->min_hash);
        return IVP_ERR_OUT_OF_MEMORY;
    }
    return IVP_OK;
}

void ivp_time_manager_destroy(ivp_time_manager_t *tm)
{
    /* Events are owned by their creators (mindists etc.); just unlink them. */
    while (ivp_minlist_has_elements(&tm->min_hash)) {
        ivp_time_event_t *ev = (ivp_time_event_t *)ivp_minlist_find_min_elem(&tm->min_hash);
        ivp_minlist_remove(&tm->min_hash, (ivp_minlist_index_t)ev->index);
        ev->index = IVP_MINLIST_UNUSED;
    }
    ivp_minlist_destroy(&tm->min_hash);
}

void ivp_time_manager_insert_event(ivp_time_manager_t *tm, ivp_time_event_t *ev, ivp_time_t time)
{
    float event_time = (float)ivp_time_diff(time, tm->base_time);
    IVP_ASSERT((double)event_time - tm->last_time >= -(double)P_FLOAT_RES);
    ev->index = ivp_minlist_add(&tm->min_hash, ev, event_time);
}

void ivp_time_manager_remove_event(ivp_time_manager_t *tm, ivp_time_event_t *ev)
{
    ivp_minlist_remove(&tm->min_hash, (ivp_minlist_index_t)ev->index);
}

void ivp_time_manager_update_event(ivp_time_manager_t *tm, ivp_time_event_t *ev, ivp_time_t time)
{
    float event_time = (float)ivp_time_diff(time, tm->base_time);
    IVP_ASSERT((double)event_time - tm->last_time >= -(double)P_FLOAT_RES);
    ivp_minlist_remove(&tm->min_hash, (ivp_minlist_index_t)ev->index);
    ev->index = ivp_minlist_add(&tm->min_hash, ev, event_time);
}

int ivp_time_manager_get_event_count(const ivp_time_manager_t *tm)
{
    return (int)tm->min_hash.counter;
}

void ivp_time_manager_event_loop(ivp_time_manager_t *tm, struct ivp_environment *env, ivp_time_t time)
{
    ivp_env_start_pcount(env);

    /* IVP_Event_Manager_Standard::simulate_time_events */
    float event_time;
    while ((event_time = ivp_minlist_find_min_value(&tm->min_hash)) <
           (float)ivp_time_diff(time, tm->base_time)) {
        ivp_time_event_t *ev = (ivp_time_event_t *)ivp_minlist_find_min_elem(&tm->min_hash);
        ivp_minlist_remove(&tm->min_hash, (ivp_minlist_index_t)ev->index);
        ev->index = IVP_MINLIST_UNUSED;
        IVP_ASSERT((double)event_time - tm->last_time >= -(double)P_FLOAT_RES);
        tm->last_time = (double)event_time;
        ivp_environment_set_current_time_internal(env, tm->base_time + (double)event_time);
        ev->simulate_time_event(ev, env);
    }
    ivp_environment_set_current_time_internal(env, time);

    ivp_env_stop_pcount(env);
}

void ivp_time_manager_simulate_variable_time_step(ivp_time_manager_t *tm, struct ivp_environment *env,
                                                  float delta)
{
    ivp_env_start_pcount(env);

    /* IVP_Event_Manager::simulate_variable_time_step */
    int psi_counter = 0;
    for (;;) {
        float event_time = ivp_minlist_find_min_value(&tm->min_hash);
        ivp_time_event_t *ev = (ivp_time_event_t *)ivp_minlist_find_min_elem(&tm->min_hash);
        ivp_environment_set_current_time_internal(env, tm->base_time + (double)event_time);
        if (ev == &tm->psi_event) {
            psi_counter++;
            if (psi_counter == 2) break;
            ivp_environment_set_delta_psi_time_internal(env, (double)delta);
        }
        tm->last_time = (double)event_time;
        ivp_minlist_remove(&tm->min_hash, (ivp_minlist_index_t)ev->index);
        ev->index = IVP_MINLIST_UNUSED;
        ev->simulate_time_event(ev, env);
    }

    ivp_env_stop_pcount(env);
}

void ivp_time_manager_rebase_for_psi(ivp_time_manager_t *tm, ivp_time_t time_of_last_psi)
{
    /* IVP_Time_Event_PSI::simulate_time_event: the queued values are
     * relative to the previous base time, so the delta to it is subtracted
     * and events queued across PSIs keep their absolute time (reference
     * 4d818ea) */
    float event_time = (float)ivp_time_diff(time_of_last_psi, tm->base_time);
    ivp_minlist_t *ml = &tm->min_hash;
    for (unsigned int i = ml->first_element; i != IVP_MINLIST_UNUSED; i = ml->elems[i].next) {
        ml->elems[i].value -= event_time;
    }
    ml->min_value -= event_time;
    tm->base_time = time_of_last_psi;
    tm->last_time = 0.0;
}

void ivp_time_manager_reset_time(ivp_time_manager_t *tm, ivp_time_t offset)
{
    float event_time = (float)ivp_time_diff(offset, tm->base_time);
    ivp_minlist_t *ml = &tm->min_hash;
    for (unsigned int i = ml->first_element; i != IVP_MINLIST_UNUSED; i = ml->elems[i].next) {
        ml->elems[i].value -= event_time;
    }
    ml->min_value -= event_time;
    tm->base_time = 0.0;
    tm->last_time = 0.0;
}
