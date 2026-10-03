/* ivp_hull_mgr.h — Per-object hull manager (port of IVP_Hull_Manager)
 *
 * Reference: ivp_intern/ivp_hull_manager.{hxx,cxx}, ivp_hull_manager_macros.hxx,
 * ivp_physics/ivp_listener_hull.hxx, ivp_real_object.hxx (Hull_Manager_Base).
 *
 * Every object owns one hull manager.  The "hull value" grows with the
 * object's worst-case surface speed; listeners (synapses of hull mindists,
 * OV elements, OO watchers) register the hull value at which they want to be
 * re-examined and get hull_limit_exceeded_event() once it is reached.
 */
#ifndef IVP_HULL_MGR_H
#define IVP_HULL_MGR_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/base/ivp_legacy.h>
#include <ivp/containers/ivp_minlist.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_environment;
struct ivp_hull_mgr;

#define IVP_MAX_TIME_WITHOUT_RESET        10.0f     /* seconds */
#define IVP_HULL_MANAGER_GRADIENT_FACTOR  1.00001f

typedef enum ivp_hull_elem_type {
    IVP_HULL_ELEM_POLYGON,
    IVP_HULL_ELEM_ANCHOR,
    IVP_HULL_ELEM_OO_WATCHER,
    IVP_HULL_ELEM_OO_CONNECTOR
} ivp_hull_elem_type_t;

typedef struct ivp_listener_hull ivp_listener_hull_t;

typedef struct ivp_listener_hull_vt {
    ivp_hull_elem_type_t type;
    /* ATT: must update/remove itself (or its peer) from the hull manager */
    void (*hull_limit_exceeded_event)(ivp_listener_hull_t *lh, struct ivp_hull_mgr *hm,
                                      float hull_intrusion_value);
    /* must remove itself from the hull manager */
    void (*hull_manager_is_going_to_be_deleted_event)(ivp_listener_hull_t *lh,
                                                      struct ivp_hull_mgr *hm);
    /* optional (NULL = no-op) */
    void (*hull_manager_is_reset)(ivp_listener_hull_t *lh, float dt, float center_dt);
} ivp_listener_hull_vt_t;

struct ivp_listener_hull {
    const ivp_listener_hull_vt_t *vt;
    unsigned int minlist_index;
};

static inline void ivp_listener_hull_init(ivp_listener_hull_t *lh, const ivp_listener_hull_vt_t *vt) {
    lh->vt = vt;
    lh->minlist_index = 0xFFFFFFFFu;
}

typedef struct ivp_hull_mgr {
    /* IVP_Hull_Manager_Base_Gradient */
    ivp_time_t last_vpsi_time;
    float      gradient;            /* slightly higher than the real gradient */
    float      center_gradient;
    float      hull_value_last_vpsi;
    float      hull_center_value_last_vpsi;
    float      hull_value_next_psi; /* optimistic, may be less */
    int        time_of_next_reset;  /* counting seconds only */
    /* IVP_Hull_Manager_Base */
    ivp_minlist_t sorted_synapses;
} ivp_hull_mgr_t;

void ivp_hull_mgr_init(ivp_hull_mgr_t *hm, const ivp_allocator_t *alloc);
/* Fires hull_manager_is_going_to_be_deleted_event for all listeners. */
void ivp_hull_mgr_delete_listeners(ivp_hull_mgr_t *hm);
void ivp_hull_mgr_destroy(ivp_hull_mgr_t *hm);

static inline float ivp_hull_mgr_get_current_hull_time(const ivp_hull_mgr_t *hm) {
    return hm->hull_value_last_vpsi;
}
static inline float ivp_hull_mgr_get_current_center_hull_time(const ivp_hull_mgr_t *hm) {
    return hm->hull_center_value_last_vpsi;
}

static inline float ivp_hull_mgr_get_angular_hull_time(const ivp_hull_mgr_t *hm, ivp_time_t current_time) {
    return (float)(ivp_time_diff(current_time, hm->last_vpsi_time) * (double)(hm->gradient - hm->center_gradient) +
                   (double)(hm->hull_value_last_vpsi - hm->hull_center_value_last_vpsi));
}

/* returns (hull - center) value at t_now */
float ivp_hull_mgr_insert_synapse(ivp_hull_mgr_t *hm, ivp_listener_hull_t *syn,
                                  ivp_time_t t_now, double delta_valid_hull_time);
/* returns hull_center_value_last_vpsi */
float ivp_hull_mgr_insert_lazy_synapse(ivp_hull_mgr_t *hm, ivp_listener_hull_t *syn,
                                       ivp_time_t t_now, double delta_valid_hull_time);
void ivp_hull_mgr_update_synapse(ivp_hull_mgr_t *hm, ivp_listener_hull_t *syn,
                                 ivp_time_t t_now, double delta_valid_hull_time);
void ivp_hull_mgr_update_lazy_synapse(ivp_hull_mgr_t *hm, ivp_listener_hull_t *syn,
                                      ivp_time_t t_now, double delta_valid_hull_time);
void ivp_hull_mgr_remove_synapse(ivp_hull_mgr_t *hm, ivp_listener_hull_t *syn);

void ivp_hull_mgr_increase_hull_by_x(ivp_hull_mgr_t *hm, ivp_time_t t_now, float delta_time,
                                     float gradient, float center_gradient);
static inline bool ivp_hull_mgr_are_events_in_hull(const ivp_hull_mgr_t *hm) {
    return ivp_minlist_find_min_value(&hm->sorted_synapses) - hm->hull_value_next_psi < 0.0f;
}
void ivp_hull_mgr_check_hull_synapses(ivp_hull_mgr_t *hm, struct ivp_environment *env);
void ivp_hull_mgr_check_for_reset(ivp_hull_mgr_t *hm);
void ivp_hull_mgr_reset_time(ivp_hull_mgr_t *hm, ivp_time_t offset);

void ivp_hull_mgr_exit_hull_for_simulation(ivp_hull_mgr_t *hm, ivp_time_t t_now);
void ivp_hull_mgr_jump_add_hull(ivp_hull_mgr_t *hm, float delta_x, float delta_center_x);

#ifdef __cplusplus
}
#endif

#endif /* IVP_HULL_MGR_H */
