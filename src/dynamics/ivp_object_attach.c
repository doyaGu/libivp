/* ivp_object_attach.c -- Object attachment implementation */

#include <ivp/dynamics/ivp_object_attach.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/math/ivp_transform.h>
#include <string.h>

void ivp_attachment_init(ivp_attachment_t *att, ivp_object_t *leader)
{
    memset(att, 0, sizeof(*att));
    att->leader = leader;
    att->active = true;
}

ivp_result_t ivp_attachment_add(ivp_attachment_t *att, ivp_object_t *follower)
{
    if (!att || !follower || !att->leader) return IVP_ERR_INVALID_ARG;
    if (follower == att->leader) return IVP_ERR_INVALID_ARG;
    if (att->n_followers >= IVP_ATTACH_MAX_FOLLOWERS) return IVP_ERR_OUT_OF_MEMORY;

    /* Compute offset in leader's core space */
    ivp_core_t *lc = &att->leader->core;
    ivp_core_t *fc = &follower->core;

    /* offset_pos = inv_rotate(leader_quat, follower_pos - leader_pos) */
    ivp_vec3r_t diff;
    ivp_vec3r_sub(&diff,
                  &fc->pos_world_f_core_last_psi,
                  &lc->pos_world_f_core_last_psi);

    ivp_vec3r_t offset_pos;
    ivp_mat3r_vimult(&offset_pos, &lc->m_world_f_core_last_psi.rot, &diff);

    /* offset_rot = conjugate(leader_quat) * follower_quat */
    ivp_quatr_t leader_conj;
    leader_conj.x = -lc->q_world_f_core_last_psi.x;
    leader_conj.y = -lc->q_world_f_core_last_psi.y;
    leader_conj.z = -lc->q_world_f_core_last_psi.z;
    leader_conj.w =  lc->q_world_f_core_last_psi.w;

    ivp_quatr_t offset_rot;
    ivp_quat_mult(&offset_rot, &leader_conj, &fc->q_world_f_core_last_psi);

    int idx = att->n_followers++;
    att->followers[idx].obj = follower;
    att->followers[idx].offset_pos = offset_pos;
    att->followers[idx].offset_rot = offset_rot;

    /* Pin follower so it doesn't get gravity */
    fc->pinned = true;

    return IVP_OK;
}

ivp_result_t ivp_attachment_remove(ivp_attachment_t *att, ivp_object_t *follower)
{
    if (!att || !follower) return IVP_ERR_INVALID_ARG;

    int found = -1;
    for (int i = 0; i < att->n_followers; i++) {
        if (att->followers[i].obj == follower) {
            found = i;
            break;
        }
    }
    if (found < 0) return IVP_ERR_INVALID_ARG;

    /* Inherit velocity from leader */
    ivp_core_t *lc = &att->leader->core;
    ivp_core_t *fc = &follower->core;
    fc->speed = lc->speed;
    fc->rot_speed = lc->rot_speed;
    fc->pinned = false;
    fc->movement_state = IVP_MT_MOVING;

    /* Remove from array */
    att->n_followers--;
    if (found < att->n_followers) {
        att->followers[found] = att->followers[att->n_followers];
    }

    return IVP_OK;
}

void ivp_attachment_sync(ivp_attachment_t *att)
{
    if (!att || !att->active || !att->leader) return;

    ivp_core_t *lc = &att->leader->core;

    for (int i = 0; i < att->n_followers; i++) {
        ivp_object_t *fobj = att->followers[i].obj;
        ivp_core_t *fc = &fobj->core;

        /* world_pos = leader_pos + rotate(leader_quat, offset_pos) */
        ivp_vec3r_t rotated;
        ivp_mat3r_vmult(&rotated, &lc->m_world_f_core_last_psi.rot,
                        &att->followers[i].offset_pos);
        ivp_vec3r_add(&fc->pos_world_f_core_last_psi,
                      &lc->pos_world_f_core_last_psi,
                      &rotated);

        /* world_quat = leader_quat * offset_rot */
        ivp_quat_mult(&fc->q_world_f_core_last_psi,
                      &lc->q_world_f_core_last_psi,
                      &att->followers[i].offset_rot);

        /* Copy delta (previous-PSI speed used for position advance) */
        fc->delta_world_f_core_psis = lc->delta_world_f_core_psis;

        /* Sync velocities */
        fc->speed = lc->speed;
        fc->rot_speed = lc->rot_speed;

        /* Rebuild matrix from quaternion */
        ivp_quat_to_mat3(&fc->q_world_f_core_last_psi,
                         &fc->m_world_f_core_last_psi.rot);
        fc->m_world_f_core_last_psi.pos = fc->pos_world_f_core_last_psi;
    }
}

void ivp_controller_attachment(const ivp_event_sim_t *event_sim,
                                 ivp_object_t *obj,
                                 void *ctx)
{
    ivp_attachment_t *att = (ivp_attachment_t *)ctx;
    if (!att || obj != att->leader) return;
    ivp_attachment_sync(att);
}
