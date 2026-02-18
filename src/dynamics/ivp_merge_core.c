/* ivp_merge_core.c -- Merged core implementation */

#include <ivp/dynamics/ivp_merge_core.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>

void ivp_merge_group_init(ivp_merge_group_t *group,
                            ivp_real_t break_impulse)
{
    memset(group, 0, sizeof(*group));
    group->break_impulse = break_impulse;
    group->merged = false;
}

ivp_result_t ivp_merge_group_add(ivp_merge_group_t *group,
                                   ivp_object_t *obj)
{
    if (!group || !obj) return IVP_ERR_INVALID_ARG;
    if (group->n_members >= IVP_MERGE_MAX_OBJECTS) return IVP_ERR_OUT_OF_MEMORY;

    int idx = group->n_members++;
    group->members[idx].obj = obj;
    group->members[idx].saved_core = obj->core;
    /* Offsets computed in merge() after center of mass is known */

    return IVP_OK;
}

void ivp_merge_group_merge(ivp_merge_group_t *group)
{
    if (!group || group->n_members < 1) return;

    /* Compute aggregate center of mass (mass-weighted average) */
    ivp_real_t total_mass = IVP_REAL(0.0);
    ivp_vec3r_t com;
    ivp_vec3r_set(&com, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    for (int i = 0; i < group->n_members; i++) {
        ivp_core_t *c = &group->members[i].obj->core;
        total_mass += c->mass;
        ivp_vec3r_add_scaled(&com, &c->pos_world_f_core_last_psi, c->mass);
    }

    if (total_mass < IVP_REAL_EPS) return;
    ivp_real_t inv_total = IVP_REAL(1.0) / total_mass;
    com.k[0] *= inv_total;
    com.k[1] *= inv_total;
    com.k[2] *= inv_total;

    /* Compute aggregate inertia (parallel axis theorem) */
    ivp_vec3r_t total_inertia;
    ivp_vec3r_set(&total_inertia, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    /* Mass-weighted average velocity */
    ivp_vec3r_t total_vel;
    ivp_vec3r_set(&total_vel, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    for (int i = 0; i < group->n_members; i++) {
        ivp_core_t *c = &group->members[i].obj->core;

        /* Offset from aggregate CoM */
        ivp_vec3r_t offset;
        ivp_vec3r_sub(&offset, &c->pos_world_f_core_last_psi, &com);
        group->members[i].offset_pos = offset;
        group->members[i].offset_rot = c->q_world_f_core_last_psi;

        /* Parallel axis: I_total += I_local + m * (|d|^2 * E - d*d^T) */
        ivp_real_t d2 = ivp_vec3r_dot(&offset, &offset);
        total_inertia.k[0] += c->rot_inertia.k[0] +
            c->mass * (d2 - offset.k[0] * offset.k[0]);
        total_inertia.k[1] += c->rot_inertia.k[1] +
            c->mass * (d2 - offset.k[1] * offset.k[1]);
        total_inertia.k[2] += c->rot_inertia.k[2] +
            c->mass * (d2 - offset.k[2] * offset.k[2]);

        /* Momentum */
        ivp_vec3r_add_scaled(&total_vel, &c->speed, c->mass);
    }

    total_vel.k[0] *= inv_total;
    total_vel.k[1] *= inv_total;
    total_vel.k[2] *= inv_total;

    /* Compute total angular momentum about CoM (world space).
     * L = Σ [ I_i * ω_i (spin, body→world) + r_i × m_i*(v_i - v_com) (orbital) ] */
    ivp_vec3r_t total_ang_mom;
    ivp_vec3r_set(&total_ang_mom, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));

    for (int i = 0; i < group->n_members; i++) {
        ivp_core_t *c = &group->members[i].obj->core;

        /* Spin angular momentum: L_spin = R * (I_body * ω_body) */
        ivp_vec3r_t L_spin_body;
        L_spin_body.k[0] = c->rot_inertia.k[0] * (ivp_real_t)c->rot_speed.k[0];
        L_spin_body.k[1] = c->rot_inertia.k[1] * (ivp_real_t)c->rot_speed.k[1];
        L_spin_body.k[2] = c->rot_inertia.k[2] * (ivp_real_t)c->rot_speed.k[2];

        ivp_vec3r_t L_spin_ws;
        ivp_transform_dir(&L_spin_ws,
                          &c->m_world_f_core_last_psi, &L_spin_body);

        /* Orbital angular momentum: L_orbit = r × (m * (v - v_com)) */
        ivp_vec3r_t v_rel;
        ivp_vec3r_sub(&v_rel, &c->speed, &total_vel);
        ivp_vec3r_t mv;
        ivp_vec3r_scale(&mv, &v_rel, c->mass);
        ivp_vec3r_t L_orbit;
        ivp_vec3r_cross(&L_orbit, &group->members[i].offset_pos, &mv);

        total_ang_mom.k[0] += L_spin_ws.k[0] + L_orbit.k[0];
        total_ang_mom.k[1] += L_spin_ws.k[1] + L_orbit.k[1];
        total_ang_mom.k[2] += L_spin_ws.k[2] + L_orbit.k[2];
    }

    /* Initialize merged core */
    ivp_core_init(&group->merged_core, total_mass, &total_inertia);
    ivp_quatr_t identity = ivp_quatr_identity();
    ivp_core_set_pose(&group->merged_core, &com, &identity);
    group->merged_core.speed = total_vel;
    group->merged_core.movement_state = IVP_MT_MOVING;

    /* Merged core uses identity orientation → body space = world space.
     * ω_merged = I_merged⁻¹ * L_total */
    group->merged_core.rot_speed.k[0] = (float)(total_ang_mom.k[0] *
        group->merged_core.inv_rot_inertia.k[0]);
    group->merged_core.rot_speed.k[1] = (float)(total_ang_mom.k[1] *
        group->merged_core.inv_rot_inertia.k[1]);
    group->merged_core.rot_speed.k[2] = (float)(total_ang_mom.k[2] *
        group->merged_core.inv_rot_inertia.k[2]);

    /* Copy merged state to all member objects */
    for (int i = 0; i < group->n_members; i++) {
        ivp_object_t *obj = group->members[i].obj;
        obj->core.speed = group->merged_core.speed;
        obj->core.rot_speed = group->merged_core.rot_speed;
        obj->core.mass = group->merged_core.mass;
        obj->core.inv_mass = group->merged_core.inv_mass;
    }

    group->merged = true;
}

void ivp_merge_group_split(ivp_merge_group_t *group)
{
    if (!group || !group->merged) return;

    for (int i = 0; i < group->n_members; i++) {
        ivp_object_t *obj = group->members[i].obj;
        ivp_core_t *saved = &group->members[i].saved_core;

        /* Restore original mass/inertia */
        obj->core.mass = saved->mass;
        obj->core.inv_mass = saved->inv_mass;
        obj->core.rot_inertia = saved->rot_inertia;
        obj->core.inv_rot_inertia = saved->inv_rot_inertia;

        /* Position = merged_com + offset */
        ivp_vec3r_add(&obj->core.pos_world_f_core_last_psi,
                      &group->merged_core.pos_world_f_core_last_psi,
                      &group->members[i].offset_pos);

        /* Inherit merged velocity */
        obj->core.speed = group->merged_core.speed;
        obj->core.rot_speed = group->merged_core.rot_speed;
        obj->core.movement_state = IVP_MT_MOVING;
        obj->core.pinned = false;

        /* Rebuild matrix */
        ivp_quat_to_mat3(&obj->core.q_world_f_core_last_psi,
                         &obj->core.m_world_f_core_last_psi.rot);
        obj->core.m_world_f_core_last_psi.pos =
            obj->core.pos_world_f_core_last_psi;
    }

    group->merged = false;
}

void ivp_merge_group_check_break(ivp_merge_group_t *group,
                                   ivp_real_t impulse_magnitude)
{
    if (!group || !group->merged) return;
    if (group->break_impulse <= IVP_REAL(0.0)) return;

    if (impulse_magnitude > group->break_impulse) {
        if (group->break_fn) {
            group->break_fn(group->break_ctx, group);
        }
        ivp_merge_group_split(group);
    }
}
