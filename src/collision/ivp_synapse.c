/* ivp_synapse.c — Legacy-style synapse linkage for mindist and friction */

#include <ivp/collision/ivp_synapse.h>
#include <ivp/collision/ivp_mindist_persistent.h>
#include <ivp/solver/ivp_friction_system.h>
#include <ivp/dynamics/ivp_object.h>

#include <string.h>

static ivp_synapse_link_t **get_list_head(ivp_object_t *obj,
                                          ivp_synapse_owner_kind_t owner_kind)
{
    if (!obj) return NULL;
    if (owner_kind == IVP_SYNAPSE_OWNER_MINDIST) {
        return &obj->mindist_synapses;
    }
    return &obj->friction_synapses;
}

void ivp_synapse_link_init(ivp_synapse_link_t *link,
                           ivp_synapse_owner_kind_t owner_kind,
                           uint8_t owner_slot)
{
    if (!link) return;
    memset(link, 0, sizeof(*link));
    link->owner_kind = owner_kind;
    link->owner_slot = owner_slot;
    link->status = IVP_ST_POINT;
}

void ivp_synapse_link_insert(ivp_synapse_link_t *link)
{
    if (!link || !link->object || link->inserted) return;

    ivp_synapse_link_t **head = get_list_head(link->object, link->owner_kind);
    if (!head) return;

    link->prev = NULL;
    link->next = *head;
    if (*head) {
        (*head)->prev = link;
    }
    *head = link;
    link->inserted = true;
}

void ivp_synapse_link_remove(ivp_synapse_link_t *link)
{
    if (!link || !link->inserted || !link->object) return;

    ivp_synapse_link_t **head = get_list_head(link->object, link->owner_kind);
    if (!head) return;

    if (link->prev) {
        link->prev->next = link->next;
    } else {
        *head = link->next;
    }
    if (link->next) {
        link->next->prev = link->prev;
    }

    link->next = NULL;
    link->prev = NULL;
    link->inserted = false;
}

static ivp_synapse_status_t default_synapse_status(const ivp_object_t *obj)
{
    if (!obj) return IVP_ST_POINT;
    if (obj->type == IVP_OBJECT_BALL) return IVP_ST_BALL;
    return IVP_ST_TRIANGLE;
}

void ivp_synapse_attach_mindist_entry(ivp_mindist_entry_t *entry)
{
    if (!entry || !entry->obj_a || !entry->obj_b || !entry->active) return;

    ivp_synapse_link_t *s0 = &entry->synapse[0];
    ivp_synapse_link_t *s1 = &entry->synapse[1];

    ivp_synapse_link_init(s0, IVP_SYNAPSE_OWNER_MINDIST, 0u);
    ivp_synapse_link_init(s1, IVP_SYNAPSE_OWNER_MINDIST, 1u);

    s0->object = entry->obj_a;
    s1->object = entry->obj_b;
    s0->owner = entry;
    s1->owner = entry;
    s0->status = default_synapse_status(entry->obj_a);
    s1->status = default_synapse_status(entry->obj_b);

    ivp_synapse_link_insert(s0);
    ivp_synapse_link_insert(s1);
}

void ivp_synapse_detach_mindist_entry(ivp_mindist_entry_t *entry)
{
    if (!entry) return;
    ivp_synapse_link_remove(&entry->synapse[0]);
    ivp_synapse_link_remove(&entry->synapse[1]);
    entry->synapse[0].object = NULL;
    entry->synapse[1].object = NULL;
    entry->synapse[0].owner = NULL;
    entry->synapse[1].owner = NULL;
}

void ivp_synapse_attach_friction_contact(ivp_persistent_contact_t *pc)
{
    if (!pc || !pc->cp.obj_a || !pc->cp.obj_b) return;

    ivp_synapse_link_t *s0 = &pc->friction_synapse[0];
    ivp_synapse_link_t *s1 = &pc->friction_synapse[1];

    ivp_synapse_link_init(s0, IVP_SYNAPSE_OWNER_FRICTION, 0u);
    ivp_synapse_link_init(s1, IVP_SYNAPSE_OWNER_FRICTION, 1u);

    s0->object = pc->cp.obj_a;
    s1->object = pc->cp.obj_b;
    s0->owner = pc;
    s1->owner = pc;
    s0->status = default_synapse_status(pc->cp.obj_a);
    s1->status = default_synapse_status(pc->cp.obj_b);

    ivp_synapse_link_insert(s0);
    ivp_synapse_link_insert(s1);
}

void ivp_synapse_detach_friction_contact(ivp_persistent_contact_t *pc)
{
    if (!pc) return;
    ivp_synapse_link_remove(&pc->friction_synapse[0]);
    ivp_synapse_link_remove(&pc->friction_synapse[1]);
    pc->friction_synapse[0].object = NULL;
    pc->friction_synapse[1].object = NULL;
    pc->friction_synapse[0].owner = NULL;
    pc->friction_synapse[1].owner = NULL;
}

ivp_mindist_entry_t *ivp_synapse_link_get_mindist(ivp_synapse_link_t *link)
{
    if (!link || link->owner_kind != IVP_SYNAPSE_OWNER_MINDIST) return NULL;
    return (ivp_mindist_entry_t *)link->owner;
}

ivp_persistent_contact_t *ivp_synapse_link_get_friction_contact(ivp_synapse_link_t *link)
{
    if (!link || link->owner_kind != IVP_SYNAPSE_OWNER_FRICTION) return NULL;
    return (ivp_persistent_contact_t *)link->owner;
}
