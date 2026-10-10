/* ivp_object.c — object template defaults and object mutation API
 * (IVP_Template_Real_Object ctor, IVP_Real_Object::change_mass / change_unmovable_flag /
 * set_pinned / recompile_values_changed).
 */
#include <ivp/dynamics/ivp_object.h>
#include <ivp/geometry/ivp_surman.h>
#include <string.h>
#include "dynamics/ivp_env_pipeline.h"

void ivp_object_template_init(ivp_object_template_t *tmpl)
{
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
    tmpl->ball_radius = IVP_REAL(0.0);
    tmpl->extra_radius = IVP_REAL(0.0);
    tmpl->rot_inertia_is_factor = true;   /* IVP_Template_Real_Object default */
    tmpl->auto_check_rot_inertia = (ivp_real_t)0.03f;
    tmpl->pinned = false;
    tmpl->enable_piling_optimization = false;
    tmpl->use_mass_center_override = false;
    ivp_vec3r_set_zero(&tmpl->mass_center_override);
    tmpl->nocoll_group_ident = NULL;
    tmpl->enable_collision_detection = true;
    tmpl->start_in_simulation = true;
}

void ivp_object_change_mass(ivp_object_t *obj, ivp_real_t new_mass)
{
    if (!obj || new_mass <= IVP_REAL(0.0)) return;
    ivp_core_set_mass(obj->physical_core, new_mass);
}

void ivp_object_change_unmovable(ivp_object_t *obj, bool unmovable)
{
    if (!obj || !obj->owner) return;
    ivp_object_change_unmovable_flag(obj, unmovable);
}

void ivp_object_set_pinned(ivp_object_t *obj, bool pinned)
{
    if (!obj || !obj->owner) return;
    ivp_object_set_pinned_ref(obj, pinned);
}

/* IVP_Real_Object::change_fast_piling_allowed */
void ivp_object_change_fast_piling_allowed(ivp_object_t *obj, bool flag)
{
    if (!obj || !obj->friction_core) return;
    obj->friction_core->fast_piling_allowed = flag;
}

/* IVP_Real_Object::recompile_values_changed */
void ivp_object_recompile(ivp_object_t *obj)
{
    if (!obj || !obj->owner) return;
    ivp_core_values_changed_recalc_redundants(obj->physical_core);
}

/* ── IVP_Real_Object helpers (public wrappers) ──────────────────────────── */

void ivp_object_async_add_speed_ws(ivp_object_t *obj, const ivp_vec3r_t *speed_ws)
{
    if (!obj || !speed_ws) return;
    ivp_vec3f_t f = ivp_vec3f_from_r(speed_ws);
    ivp_object_async_add_speed_object_ws(obj, &f);
}

void ivp_object_async_add_rot_speed_cs(ivp_object_t *obj, const ivp_vec3r_t *rot_speed_cs)
{
    if (!obj || !rot_speed_cs) return;
    ivp_vec3f_t f = ivp_vec3f_from_r(rot_speed_cs);
    ivp_object_async_add_rot_speed_object_cs(obj, &f);
}

void ivp_object_async_push_ws(ivp_object_t *obj, const ivp_vec3r_t *position_ws, const ivp_vec3r_t *impulse_ws)
{
    if (!obj || !position_ws || !impulse_ws) return;
    ivp_vec3f_t f = ivp_vec3f_from_r(impulse_ws);
    ivp_object_async_push_object_ws(obj, position_ws, &f);
}

void ivp_object_beam_to_new_position(ivp_object_t *obj, const ivp_quatr_t *rotation_world_f_object,
                                     const ivp_vec3r_t *position_ws, bool optimize_for_repeated_calls)
{
    if (!obj || !rotation_world_f_object || !position_ws) return;
    ivp_object_beam_object_to_new_position(obj, rotation_world_f_object, position_ws, optimize_for_repeated_calls);
}

void ivp_object_get_m_world_f_object_AT(const ivp_object_t *obj, ivp_transform_r_t *m_world_f_object_out)
{
    if (!obj || !m_world_f_object_out || !obj->owner) return;
    ivp_object_calc_at_matrix(obj, ivp_environment_get_time(obj->owner), m_world_f_object_out);
}

