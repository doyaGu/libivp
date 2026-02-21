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
    /* Reference-aligned event coding (matches legacy IVP_COLL_TYPE shape):
     * low nibble == 0 => collision impact event
     * low nibble != 0 => topology/feature-transition event */
    IVP_MINDIST_EVENT_PP_COLL = 0x10,
    IVP_MINDIST_EVENT_PP_PK = 0x11,

    IVP_MINDIST_EVENT_PF_COLL = 0x20,
    IVP_MINDIST_EVENT_PF_NPF = 0x21,

    IVP_MINDIST_EVENT_PK_COLL = 0x30,
    IVP_MINDIST_EVENT_PK_PF = 0x31,
    IVP_MINDIST_EVENT_PK_KK = 0x32,
    IVP_MINDIST_EVENT_PK_NOT_MORE_PARALLEL = 0x33,

    IVP_MINDIST_EVENT_KK_COLL = 0x40,
    IVP_MINDIST_EVENT_KK_PARALLEL = 0x41,
    IVP_MINDIST_EVENT_KK_PF = 0x42,

    /* Ball fallback used by the C17 implementation's generic solver path. */
    IVP_MINDIST_EVENT_BALL_COLL = 0x50,
    IVP_MINDIST_EVENT_BALL_RECALC = 0x51
} ivp_mindist_event_kind_t;

static inline bool ivp_mindist_event_is_impact(ivp_mindist_event_kind_t kind)
{
    return kind != IVP_MINDIST_EVENT_NONE && ((((uint32_t)kind) & 0x0Fu) == 0u);
}

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
