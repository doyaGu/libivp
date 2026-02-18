#ifndef IVP_ENVIRONMENT_EVENTS_H
#define IVP_ENVIRONMENT_EVENTS_H

#include <ivp/dynamics/ivp_environment.h>

/* Internal helper used by runtime subsystems to dispatch ivp_event_t entries. */
void ivp_environment_emit_event_internal(ivp_environment_t *env,
                                         ivp_event_type_t type,
                                         ivp_real_t dt,
                                         int object_index,
                                         int controller_id);

#endif
