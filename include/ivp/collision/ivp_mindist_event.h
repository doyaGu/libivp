/* ivp_mindist_event.h — Mindist event prediction and feature-pair dispatch */
#ifndef IVP_MINDIST_EVENT_H
#define IVP_MINDIST_EVENT_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/collision/ivp_mindist_persistent.h>
#include <ivp/collision/ivp_collision_tolerance.h>
#include <ivp/collision/ivp_time_event_queue.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum ivp_mindist_event_kind {
    IVP_MINDIST_EVENT_NONE = 0,
    IVP_MINDIST_EVENT_IMPACT,
    IVP_MINDIST_EVENT_TOPOLOGY
} ivp_mindist_event_kind_t;

typedef struct ivp_mindist_event_prediction {
    ivp_mindist_event_kind_t kind;
    ivp_time_t event_time;
    ivp_synapse_status_t status_a;
    ivp_synapse_status_t status_b;
    ivp_real_t expected_gap;
} ivp_mindist_event_prediction_t;

bool ivp_mindist_event_predict(const ivp_mindist_entry_t *entry,
                               const ivp_collision_settings_t *settings,
                               ivp_time_t now,
                               ivp_time_t max_time,
                               ivp_mindist_event_prediction_t *out_prediction);

void ivp_mindist_event_to_time_event(const ivp_mindist_entry_t *entry,
                                     const ivp_mindist_event_prediction_t *prediction,
                                     ivp_time_event_t *out_event);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MINDIST_EVENT_H */
