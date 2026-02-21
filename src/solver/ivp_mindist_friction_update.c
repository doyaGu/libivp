/* ivp_mindist_friction_update.c — Synapse-driven friction contact refresh */

#include <ivp/solver/ivp_mindist_friction_update.h>

#include <ivp/solver/ivp_contact.h>
#include <ivp/dynamics/ivp_object.h>

#include <math.h>

static ivp_synapse_status_t status_from_object(const ivp_object_t *obj)
{
    if (!obj) return IVP_ST_POINT;
    if (obj->type == IVP_OBJECT_BALL) return IVP_ST_BALL;
    return IVP_ST_TRIANGLE;
}

bool ivp_mindist_friction_refresh_contact(ivp_persistent_contact_t *pc,
                                          const ivp_collision_settings_t *settings,
                                          ivp_real_t dt)
{
    if (!pc || !settings || !pc->cp.obj_a || !pc->cp.obj_b) return false;

    ivp_contact_point_t refreshed = pc->cp;
    ivp_contact_refresh_geometry(&refreshed);

    const int saved_age = pc->cp.age_psi;
    pc->cp = refreshed;
    pc->cp.age_psi = saved_age;
    pc->last_gap_len = refreshed.gap_length;
    pc->inv_virt_mass_mindist_no_dir = refreshed.virtual_mass > IVP_REAL_EPS
        ? (IVP_REAL(1.0) / refreshed.virtual_mass)
        : IVP_REAL(0.0);

    pc->psi_scratch.surf_normal = refreshed.normal;
    pc->psi_scratch.contact_point_ws = refreshed.point_a;
    pc->psi_scratch.span_friction_v[0] = refreshed.span_dirs[0];
    pc->psi_scratch.span_friction_v[1] = refreshed.span_dirs[1];
    pc->psi_scratch.contact_core[0] = refreshed.obj_a->core.physical_unmoveable
        ? NULL : &refreshed.obj_a->core;
    pc->psi_scratch.contact_core[1] = refreshed.obj_b->core.physical_unmoveable
        ? NULL : &refreshed.obj_b->core;
    pc->psi_scratch.contact_point_cs[0] = refreshed.r_a;
    pc->psi_scratch.contact_point_cs[1] = refreshed.r_b;
    pc->psi_scratch.contact_cross_normal_cs[0] = refreshed.cross_normal_cs_a;
    pc->psi_scratch.contact_cross_normal_cs[1] = refreshed.cross_normal_cs_b;
    pc->psi_scratch.virtual_mass = refreshed.virtual_mass;
    pc->psi_scratch.inv_virtual_mass = refreshed.inv_virtual_mass;
    pc->psi_scratch.dist_len = refreshed.gap_length;
    pc->psi_scratch.has_negative_pull_since = pc->has_negative_pull_since;

    /* Reference alignment: recalc_friction_s_vals velocity integration.
     * The reference (ivp_mindist_friction.cxx:522-526) integrates relative
     * surface velocity into span_friction_s each PSI:
     *   span_friction_s[i] -= delta_velocity_ws.dot(span_friction_v[i]) * dt
     * where delta_velocity_ws = v_surface_A - v_surface_B.
     * In C17, v_rel = v_B - v_A, so we ADD v_rel.dot(span_v) * dt. */
    if (dt > IVP_REAL(0.0)) {
        ivp_vec3r_t v_rel;
        ivp_contact_relative_velocity(&pc->cp, &v_rel);
        pc->span_friction_s[0] += ivp_vec3r_dot(&v_rel, &refreshed.span_dirs[0]) * dt;
        pc->span_friction_s[1] += ivp_vec3r_dot(&v_rel, &refreshed.span_dirs[1]) * dt;
    }

    pc->friction_synapse[0].status = status_from_object(refreshed.obj_a);
    pc->friction_synapse[1].status = status_from_object(refreshed.obj_b);

    bool broken = false;
    if (refreshed.gap_length > settings->max_distance_for_friction) {
        broken = true;
    }
    if (!isfinite((double)refreshed.gap_length)) {
        broken = true;
    }

    pc->friction_is_broken = broken;
    pc->psi_scratch.friction_is_broken = broken;
    pc->cp_status = broken ? IVP_CP_STATUS_BROKEN : IVP_CP_STATUS_NORMAL;
    return !broken;
}
