/* ivp_synapse.h — Legacy-style synapse linkage for mindist and friction */
#ifndef IVP_SYNAPSE_H
#define IVP_SYNAPSE_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_forward_decls.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_compact_edge;
typedef struct ivp_compact_edge ivp_compact_edge_t;

typedef struct ivp_mindist_entry ivp_mindist_entry_t;
typedef struct ivp_persistent_contact ivp_persistent_contact_t;

typedef enum ivp_synapse_status {
    IVP_ST_POINT = 0,
    IVP_ST_EDGE,
    IVP_ST_TRIANGLE,
    IVP_ST_BALL,
    IVP_ST_MAX_LEGAL
} ivp_synapse_status_t;

typedef enum ivp_synapse_owner_kind {
    IVP_SYNAPSE_OWNER_MINDIST = 0,
    IVP_SYNAPSE_OWNER_FRICTION = 1
} ivp_synapse_owner_kind_t;

typedef struct ivp_synapse_link {
    struct ivp_synapse_link *next;
    struct ivp_synapse_link *prev;

    ivp_object_t *object;
    const ivp_compact_edge_t *edge;
    const void *owner;

    ivp_synapse_status_t status;
    ivp_synapse_owner_kind_t owner_kind;
    uint8_t owner_slot;
    bool inserted;
} ivp_synapse_link_t;

void ivp_synapse_link_init(ivp_synapse_link_t *link,
                           ivp_synapse_owner_kind_t owner_kind,
                           uint8_t owner_slot);
void ivp_synapse_link_insert(ivp_synapse_link_t *link);
void ivp_synapse_link_remove(ivp_synapse_link_t *link);

void ivp_synapse_attach_mindist_entry(ivp_mindist_entry_t *entry);
void ivp_synapse_detach_mindist_entry(ivp_mindist_entry_t *entry);

void ivp_synapse_attach_friction_contact(ivp_persistent_contact_t *pc);
void ivp_synapse_detach_friction_contact(ivp_persistent_contact_t *pc);

ivp_mindist_entry_t *ivp_synapse_link_get_mindist(ivp_synapse_link_t *link);
ivp_persistent_contact_t *ivp_synapse_link_get_friction_contact(ivp_synapse_link_t *link);

#ifdef __cplusplus
}
#endif

#endif /* IVP_SYNAPSE_H */
