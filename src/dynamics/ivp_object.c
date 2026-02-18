/* ivp_object.c — Physics object implementation */
#include <ivp/dynamics/ivp_object.h>
#include <ivp/geometry/ivp_surman.h>
#include <string.h>

/* ── Object template initialization ─────────────────────────────────────── */

void ivp_object_template_init(ivp_object_template_t *tmpl) {
    memset(tmpl, 0, sizeof(*tmpl));

    tmpl->type = IVP_OBJECT_BALL;
    tmpl->mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl->rot_inertia, IVP_REAL(1.0), IVP_REAL(1.0), IVP_REAL(1.0));
    ivp_vec3r_set_zero(&tmpl->position);
    tmpl->orientation = ivp_quatr_identity();

    tmpl->speed_damp_factor = IVP_REAL(0.01);
    tmpl->rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.01), IVP_REAL(0.01), IVP_REAL(0.01));
    tmpl->material = ivp_material_default();
    tmpl->no_collision_group = NULL;
    tmpl->name = NULL;
    tmpl->client_data = NULL;
    tmpl->physical_unmoveable = false;
    tmpl->ccd_enabled = false;
    tmpl->ccd_speed_threshold = IVP_REAL(0.0);
}

/* ── Object mutation ───────────────────────────────────────────────────── */

void ivp_object_change_mass(ivp_object_t *obj, ivp_real_t new_mass)
{
    if (!obj || new_mass <= IVP_REAL(0.0)) return;
    ivp_core_set_mass(&obj->core, new_mass);
}

void ivp_object_change_unmovable(ivp_object_t *obj, bool unmovable)
{
    if (!obj) return;
    obj->core.physical_unmoveable = unmovable;
    if (unmovable) {
        obj->core.movement_state = IVP_MT_STATIC;
        ivp_vec3r_set_zero(&obj->core.speed);
        obj->core.rot_speed.k[0] = 0.0f;
        obj->core.rot_speed.k[1] = 0.0f;
        obj->core.rot_speed.k[2] = 0.0f;
        ivp_vec3r_set_zero(&obj->core.speed_change);
        ivp_vec3r_set_zero(&obj->core.rot_speed_change);
    } else {
        obj->core.movement_state = IVP_MT_MOVING;
    }
}

void ivp_object_set_pinned(ivp_object_t *obj, bool pinned)
{
    if (!obj) return;
    ivp_core_set_pinned(&obj->core, pinned);
}

void ivp_object_recompile(ivp_object_t *obj)
{
    if (!obj) return;

    /* Re-derive bounding radius from surface manager */
    if (obj->surman && obj->surman->vtable->get_radius_and_deviation) {
        float center[3] = {0.0f, 0.0f, 0.0f};
        float radius = 1.0f, deviation = 0.0f;
        obj->surman->vtable->get_radius_and_deviation(obj->surman, center,
                                                       &radius, &deviation);
        obj->core.upper_limit_radius    = (ivp_real_t)radius;
        obj->core.max_surface_deviation = (ivp_real_t)deviation;
    }

    /* Ensure object is awake after recompile */
    if (!obj->core.physical_unmoveable &&
        obj->core.movement_state == IVP_MT_NOT_SIM) {
        obj->core.movement_state = IVP_MT_MOVING;
    }
}
