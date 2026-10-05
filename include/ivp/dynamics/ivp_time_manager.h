/* ivp_time_manager.h — Event-driven time manager (port of IVP_Time_Manager)
 *
 * Reference: ivp_physics/ivp_time.{hxx,cxx}, ivp_time_event.hxx.
 *
 * All simulation advances through this manager: the PSI event fires every
 * delta_PSI_time and runs the PSI; mindist collision events fire at their
 * exact sub-PSI times in between.  Event times are stored as float offsets
 * relative to base_time (= time of the last PSI).
 */
#ifndef IVP_TIME_MANAGER_H
#define IVP_TIME_MANAGER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/base/ivp_legacy.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_result.h>
#include <ivp/containers/ivp_minlist.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_environment;
typedef struct ivp_time_event ivp_time_event_t;

typedef void (*ivp_time_event_fn_t)(ivp_time_event_t *ev, struct ivp_environment *env);

struct ivp_time_event {
    ivp_time_event_fn_t simulate_time_event;
    unsigned int        index;   /* IVP_MINLIST_UNUSED when not queued */
};

static inline void ivp_time_event_init(ivp_time_event_t *ev, ivp_time_event_fn_t fn) {
    ev->simulate_time_event = fn;
    ev->index = IVP_MINLIST_UNUSED;
}

typedef struct ivp_time_manager {
    ivp_minlist_t     min_hash;
    ivp_time_event_t  psi_event;
    double            last_time;  /* relative to base_time */
    ivp_time_t        base_time;  /* values are relative to this */
} ivp_time_manager_t;

ivp_result_t ivp_time_manager_init(ivp_time_manager_t *tm, const ivp_allocator_t *alloc,
                                   ivp_time_event_fn_t psi_fn);
void ivp_time_manager_destroy(ivp_time_manager_t *tm);

void ivp_time_manager_insert_event(ivp_time_manager_t *tm, ivp_time_event_t *ev, ivp_time_t time);
void ivp_time_manager_remove_event(ivp_time_manager_t *tm, ivp_time_event_t *ev);
void ivp_time_manager_update_event(ivp_time_manager_t *tm, ivp_time_event_t *ev, ivp_time_t time);
int  ivp_time_manager_get_event_count(const ivp_time_manager_t *tm);

/* IVP_Event_Manager_Standard::simulate_time_events */
void ivp_time_manager_event_loop(ivp_time_manager_t *tm, struct ivp_environment *env, ivp_time_t time);
/* IVP_Event_Manager::simulate_variable_time_step */
void ivp_time_manager_simulate_variable_time_step(ivp_time_manager_t *tm, struct ivp_environment *env,
                                                  float delta_time);
void ivp_time_manager_reset_time(ivp_time_manager_t *tm, ivp_time_t offset);

/* Shift all queued values by -event_time and rebase (IVP_Time_Event_PSI). */
void ivp_time_manager_rebase_for_psi(ivp_time_manager_t *tm, ivp_time_t time_of_last_psi);

#ifdef __cplusplus
}
#endif

#endif /* IVP_TIME_MANAGER_H */
