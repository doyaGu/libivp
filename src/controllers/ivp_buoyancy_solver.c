#include "ivp_buoyancy_solver.h"

#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_compact_triangle.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/math/ivp_fast_math.h>

#define IVP_BUOYANCY_PRESSURE_SCALE IVP_REAL(0.03)
#define IVP_BUOYANCY_FRICTION_RATIO IVP_REAL(0.20)
#define IVP_BUOYANCY_USE_POLYGON_TRI_DRAG 1

typedef struct ivp_polygon_drag_context {
    ivp_object_t *obj;
    ivp_core_t *core;
    const ivp_buoyancy_state_t *state;
    ivp_real_t dt;
    ivp_real_t pressure_coeff;
    ivp_real_t friction_coeff;
    ivp_real_t under_area_sum;
    ivp_real_t total_area_sum;
    ivp_vec3r_t accumulated_impulse_ws;
    ivp_vec3r_t accumulated_torque_ws;
} ivp_polygon_drag_context_t;

static inline ivp_real_t ivp_plane_signed_distance(const ivp_vec3r_t *p,
                                                   const ivp_vec3r_t *normal,
                                                   ivp_real_t height)
{
    return ivp_vec3r_dot(p, normal) - height;
}

static inline void ivp_object_point_to_world(const ivp_object_t *obj,
                                             const ivp_vec3r_t *p_os,
                                             ivp_vec3r_t *p_ws)
{
    ivp_mat3r_vmult(p_ws, &obj->core.m_world_f_core_last_psi.rot, p_os);
    ivp_vec3r_add(p_ws, p_ws, &obj->core.pos_world_f_core_last_psi);
}

static inline ivp_vec3r_t ivp_poly_point_to_vec3r(const ivp_compact_poly_point_t *p)
{
    return ivp_vec3r((ivp_real_t)p->k[0],
                     (ivp_real_t)p->k[1],
                     (ivp_real_t)p->k[2]);
}

static int ivp_clip_triangle_under_plane(const ivp_vec3r_t tri[3],
                                         const ivp_real_t dist[3],
                                         ivp_vec3r_t out[5])
{
    int out_count = 0;

    for (int i = 0; i < 3; ++i) {
        const ivp_vec3r_t *a = &tri[i];
        const ivp_vec3r_t *b = &tri[(i + 1) % 3];
        ivp_real_t da = dist[i];
        ivp_real_t db = dist[(i + 1) % 3];
        bool a_in = (da >= IVP_REAL(0.0));
        bool b_in = (db >= IVP_REAL(0.0));

        if (a_in && b_in) {
            out[out_count++] = *b;
            continue;
        }

        if (a_in != b_in) {
            ivp_real_t denom = da - db;
            if (ivp_fabs(denom) > IVP_REAL_EPS) {
                ivp_real_t t = da / denom;
                if (t < IVP_REAL(0.0)) t = IVP_REAL(0.0);
                if (t > IVP_REAL(1.0)) t = IVP_REAL(1.0);
                ivp_vec3r_lerp(&out[out_count++], a, b, t);
            }
        }

        if (!a_in && b_in) {
            out[out_count++] = *b;
        }
    }

    return out_count;
}

static ivp_real_t ivp_polygon_area_centroid(const ivp_vec3r_t *poly,
                                            int n,
                                            ivp_vec3r_t *centroid_out)
{
    ivp_real_t area_sum = IVP_REAL(0.0);
    ivp_vec3r_t centroid_sum = ivp_vec3r_zero();

    for (int i = 1; i < n - 1; ++i) {
        ivp_vec3r_t ab, ac, cross;
        ivp_vec3r_sub(&ab, &poly[i], &poly[0]);
        ivp_vec3r_sub(&ac, &poly[i + 1], &poly[0]);
        ivp_vec3r_cross(&cross, &ab, &ac);

        ivp_real_t tri_area = IVP_REAL(0.5) * ivp_vec3r_length(&cross);
        if (tri_area <= IVP_REAL_EPS) {
            continue;
        }

        ivp_vec3r_t tri_centroid = ivp_vec3r_zero();
        ivp_vec3r_add(&tri_centroid, &tri_centroid, &poly[0]);
        ivp_vec3r_add(&tri_centroid, &tri_centroid, &poly[i]);
        ivp_vec3r_add(&tri_centroid, &tri_centroid, &poly[i + 1]);
        ivp_vec3r_scale(&tri_centroid, &tri_centroid,
                        IVP_REAL(1.0) / IVP_REAL(3.0));

        ivp_vec3r_add_scaled(&centroid_sum, &tri_centroid, tri_area);
        area_sum += tri_area;
    }

    if (area_sum > IVP_REAL_EPS) {
        ivp_vec3r_scale(centroid_out, &centroid_sum, IVP_REAL(1.0) / area_sum);
    } else {
        *centroid_out = poly[0];
    }

    return area_sum;
}

static void ivp_buoyancy_apply_polygon_triangle_drag(ivp_polygon_drag_context_t *ctx,
                                                     const ivp_compact_ledge_t *ledge,
                                                     const ivp_compact_triangle_t *tri)
{
    const ivp_compact_edge_t *e0 = ivp_tri_get_edge(tri, 0);
    const ivp_compact_edge_t *e1 = ivp_tri_get_edge(tri, 1);
    const ivp_compact_edge_t *e2 = ivp_tri_get_edge(tri, 2);

    ivp_vec3r_t tri_os[3] = {
        ivp_poly_point_to_vec3r(ivp_edge_get_start_point(e0, ledge)),
        ivp_poly_point_to_vec3r(ivp_edge_get_start_point(e1, ledge)),
        ivp_poly_point_to_vec3r(ivp_edge_get_start_point(e2, ledge))
    };

    ivp_vec3r_t tri_ws[3];
    ivp_object_point_to_world(ctx->obj, &tri_os[0], &tri_ws[0]);
    ivp_object_point_to_world(ctx->obj, &tri_os[1], &tri_ws[1]);
    ivp_object_point_to_world(ctx->obj, &tri_os[2], &tri_ws[2]);

    ivp_real_t dist[3];
    dist[0] = ivp_plane_signed_distance(&tri_ws[0], &ctx->state->surface_normal,
                                        ctx->state->surface_height);
    dist[1] = ivp_plane_signed_distance(&tri_ws[1], &ctx->state->surface_normal,
                                        ctx->state->surface_height);
    dist[2] = ivp_plane_signed_distance(&tri_ws[2], &ctx->state->surface_normal,
                                        ctx->state->surface_height);

    ivp_vec3r_t clipped[5];
    int clipped_count = ivp_clip_triangle_under_plane(tri_ws, dist, clipped);
    if (clipped_count < 3) {
        return;
    }

    ivp_vec3r_t edge01, edge02, tri_normal_ws;
    ivp_vec3r_sub(&edge01, &tri_ws[1], &tri_ws[0]);
    ivp_vec3r_sub(&edge02, &tri_ws[2], &tri_ws[0]);
    ivp_vec3r_cross(&tri_normal_ws, &edge01, &edge02);

    ivp_real_t normal_len = ivp_vec3r_length(&tri_normal_ws);
    if (normal_len <= IVP_REAL_EPS) {
        return;
    }

    ivp_real_t tri_area = IVP_REAL(0.5) * normal_len;
    ctx->total_area_sum += tri_area;

    ivp_vec3r_t area_center_ws;
    ivp_real_t under_area = ivp_polygon_area_centroid(clipped, clipped_count,
                                                      &area_center_ws);
    if (under_area <= IVP_REAL_EPS) {
        return;
    }
    ctx->under_area_sum += under_area;

    ivp_vec3r_scale(&tri_normal_ws, &tri_normal_ws, IVP_REAL(1.0) / normal_len);

    ivp_vec3r_t object_surface_speed_ws;
    ivp_core_get_surface_speed_ws(ctx->core, &area_center_ws, &object_surface_speed_ws);

    ivp_vec3r_t rel_speed_ws;
    ivp_vec3r_sub(&rel_speed_ws, &ctx->state->flow_velocity, &object_surface_speed_ws);

    ivp_real_t rel_speed_sq = ivp_vec3r_quad_length(&rel_speed_ws);
    if (rel_speed_sq <= IVP_REAL_EPS) {
        return;
    }

    ivp_real_t rel_speed = ivp_sqrt(rel_speed_sq);
    ivp_vec3r_t rel_dir_ws;
    ivp_vec3r_scale(&rel_dir_ws, &rel_speed_ws, IVP_REAL(1.0) / rel_speed);

    ivp_real_t surface_factor = ivp_vec3r_dot(&tri_normal_ws, &rel_dir_ws);
    if (surface_factor >= IVP_REAL(0.0)) {
        return;
    }

    ivp_real_t visible_surface_under = -surface_factor * under_area;
    ivp_real_t drag_scale = visible_surface_under * ctx->pressure_coeff
                          + under_area * ctx->friction_coeff;
    ivp_real_t impulse_mag = drag_scale * rel_speed_sq * ctx->dt;
    if (impulse_mag <= IVP_REAL(0.0)) {
        return;
    }

    ivp_vec3r_t impulse_ws;
    ivp_vec3r_scale(&impulse_ws, &rel_dir_ws, impulse_mag);
    ivp_vec3r_add(&ctx->accumulated_impulse_ws,
                  &ctx->accumulated_impulse_ws,
                  &impulse_ws);

    ivp_vec3r_t r_ws;
    ivp_vec3r_sub(&r_ws, &area_center_ws, &ctx->core->pos_world_f_core_last_psi);
    ivp_vec3r_t torque_ws;
    ivp_vec3r_cross(&torque_ws, &r_ws, &impulse_ws);
    ivp_vec3r_add(&ctx->accumulated_torque_ws,
                  &ctx->accumulated_torque_ws,
                  &torque_ws);
}

static void ivp_buoyancy_apply_polygon_ledge_drag(ivp_polygon_drag_context_t *ctx,
                                                  const ivp_compact_ledge_t *ledge)
{
    int n_triangles = ivp_ledge_get_n_triangles(ledge);
    const ivp_compact_triangle_t *tri = ivp_ledge_get_first_triangle(ledge);

    for (int i = 0; i < n_triangles; ++i, ++tri) {
        ivp_buoyancy_apply_polygon_triangle_drag(ctx, ledge, tri);
    }
}

static bool ivp_buoyancy_visit_polygon_ledge(const ivp_compact_ledge_t *ledge,
                                             void *user_data)
{
    ivp_polygon_drag_context_t *ctx = (ivp_polygon_drag_context_t *)user_data;
    ivp_buoyancy_apply_polygon_ledge_drag(ctx, ledge);
    return true;
}

static ivp_real_t ivp_buoyancy_apply_polygon_drag(const ivp_event_sim_t *event_sim,
                                                  ivp_object_t *obj,
                                                  const ivp_buoyancy_state_t *state)
{
    ivp_polygon_drag_context_t ctx;
    ctx.obj = obj;
    ctx.core = &obj->core;
    ctx.state = state;
    ctx.dt = event_sim->delta_time;
    ctx.pressure_coeff = IVP_REAL(0.5) * state->fluid_density
                       * state->linear_drag * IVP_BUOYANCY_PRESSURE_SCALE;
    ctx.friction_coeff = ctx.pressure_coeff * IVP_BUOYANCY_FRICTION_RATIO;
    ctx.under_area_sum = IVP_REAL(0.0);
    ctx.total_area_sum = IVP_REAL(0.0);
    ivp_vec3r_set_zero(&ctx.accumulated_impulse_ws);
    ivp_vec3r_set_zero(&ctx.accumulated_torque_ws);

    const ivp_compact_ledge_t *single = ivp_surman_get_single_convex(obj->surman);
    if (single) {
        ivp_buoyancy_apply_polygon_ledge_drag(&ctx, single);
    } else {
        const ivp_compact_ledgetree_node_t *root = ivp_surman_get_ledgetree_root(obj->surman);
        if (root) {
            ivp_ledgetree_visit_all(root, ivp_buoyancy_visit_polygon_ledge, &ctx);
        }
    }

    if (ctx.total_area_sum > IVP_REAL_EPS) {
        ivp_real_t impulse_len = ivp_vec3r_length(&ctx.accumulated_impulse_ws);
        if (impulse_len > IVP_REAL_EPS) {
            ivp_real_t impulse_scale = IVP_REAL(1.0);
            if (ctx.core->inv_mass > IVP_REAL_EPS) {
                ivp_vec3r_t rel_core_ws;
                ivp_vec3r_sub(&rel_core_ws, &state->flow_velocity, &ctx.core->speed);
                ivp_real_t rel_core_speed = ivp_vec3r_length(&rel_core_ws);
                ivp_real_t max_impulse = rel_core_speed / ctx.core->inv_mass;
                if (impulse_len > max_impulse && max_impulse > IVP_REAL(0.0)) {
                    impulse_scale = max_impulse / impulse_len;
                }
            }

            if (impulse_scale < IVP_REAL(1.0)) {
                ivp_vec3r_scale(&ctx.accumulated_impulse_ws,
                                &ctx.accumulated_impulse_ws,
                                impulse_scale);
                ivp_vec3r_scale(&ctx.accumulated_torque_ws,
                                &ctx.accumulated_torque_ws,
                                impulse_scale);
            }

            ivp_core_async_center_push(ctx.core, &ctx.accumulated_impulse_ws);

            ivp_vec3r_t torque_cs;
            ivp_mat3r_vimult(&torque_cs,
                             &ctx.core->m_world_f_core_last_psi.rot,
                             &ctx.accumulated_torque_ws);
            ctx.core->rot_speed_change.k[0] += torque_cs.k[0] * ctx.core->inv_rot_inertia.k[0];
            ctx.core->rot_speed_change.k[1] += torque_cs.k[1] * ctx.core->inv_rot_inertia.k[1];
            ctx.core->rot_speed_change.k[2] += torque_cs.k[2] * ctx.core->inv_rot_inertia.k[2];
        }

        ivp_real_t immersion = ctx.under_area_sum / ctx.total_area_sum;
        if (immersion < IVP_REAL(0.0)) immersion = IVP_REAL(0.0);
        if (immersion > IVP_REAL(1.0)) immersion = IVP_REAL(1.0);
        return immersion;
    }

    return IVP_REAL(0.0);
}

static void ivp_buoyancy_apply_sphere_drag(const ivp_event_sim_t *event_sim,
                                           const ivp_object_t *obj,
                                           ivp_core_t *core,
                                           const ivp_buoyancy_state_t *state,
                                           ivp_real_t radius,
                                           ivp_real_t h)
{
    ivp_real_t pi = IVP_PI;
    ivp_real_t proj_area;
    if (h >= IVP_REAL(2.0) * radius) {
        proj_area = pi * radius * radius;
    } else {
        proj_area = pi * h * (IVP_REAL(2.0) * radius - h);
    }

    ivp_vec3r_t rel_vel;
    rel_vel.k[0] = core->speed.k[0] - state->flow_velocity.k[0];
    rel_vel.k[1] = core->speed.k[1] - state->flow_velocity.k[1];
    rel_vel.k[2] = core->speed.k[2] - state->flow_velocity.k[2];

    ivp_real_t rel_speed_sq = ivp_vec3r_quad_length(&rel_vel);
    if (rel_speed_sq <= IVP_REAL_EPS) {
        return;
    }

    ivp_real_t drag_coeff = IVP_REAL(0.5) * state->fluid_density
                          * state->linear_drag * proj_area;
    ivp_real_t rel_speed = ivp_sqrt(rel_speed_sq);
    ivp_real_t drag_force = drag_coeff * rel_speed_sq;
    ivp_real_t drag_impulse = drag_force * event_sim->delta_time;

    if (core->inv_mass > IVP_REAL_EPS) {
        ivp_real_t max_impulse = rel_speed / core->inv_mass;
        if (obj->type == IVP_OBJECT_POLYGON && core->mass > IVP_REAL(5.0)) {
            ivp_real_t alpha = h / radius;
            if (alpha < IVP_REAL(0.2)) {
                ivp_real_t scale = IVP_REAL(30.0) * alpha * alpha - alpha;
                if (scale < IVP_REAL(0.0)) scale = IVP_REAL(0.0);
                if (scale > IVP_REAL(1.0)) scale = IVP_REAL(1.0);
                max_impulse *= scale;
            }
        }
        if (drag_impulse > max_impulse) {
            drag_impulse = max_impulse;
        }
    }

    ivp_vec3r_t drag_impulse_ws;
    ivp_real_t inv_rel_speed = IVP_REAL(1.0) / rel_speed;
    drag_impulse_ws.k[0] = -rel_vel.k[0] * inv_rel_speed * drag_impulse;
    drag_impulse_ws.k[1] = -rel_vel.k[1] * inv_rel_speed * drag_impulse;
    drag_impulse_ws.k[2] = -rel_vel.k[2] * inv_rel_speed * drag_impulse;
    ivp_core_async_center_push(core, &drag_impulse_ws);
}

void ivp_buoyancy_solver_step(const ivp_event_sim_t *event_sim,
                              ivp_object_t *obj,
                              const ivp_buoyancy_state_t *state)
{
    ivp_environment_t *env = event_sim->environment;
    const ivp_real_t dt = event_sim->delta_time;
    ivp_core_t *core = &obj->core;

    ivp_real_t dist = ivp_plane_signed_distance(&core->pos_world_f_core_last_psi,
                                                &state->surface_normal,
                                                state->surface_height);

    ivp_real_t radius = core->upper_limit_radius;
    if (radius < IVP_REAL_EPS) {
        return;
    }

    ivp_real_t h = radius - dist;
    if (h <= IVP_REAL(0.0)) {
        return;
    }
    if (h > IVP_REAL(2.0) * radius) {
        h = IVP_REAL(2.0) * radius;
    }

    ivp_real_t pi = IVP_PI;
    ivp_real_t volume_submerged = (pi * h * h * (IVP_REAL(3.0) * radius - h))
                                / IVP_REAL(3.0);

    ivp_real_t denom_vc = IVP_REAL(3.0) * radius - h;
    ivp_vec3r_t volume_center = core->pos_world_f_core_last_psi;
    if (denom_vc > IVP_REAL_EPS) {
        ivp_real_t center_offset = IVP_REAL(0.75)
                                 * (IVP_REAL(4.0) * radius * radius
                                  - IVP_REAL(4.0) * radius * h
                                  + h * h)
                                 / denom_vc;
        ivp_vec3r_add_scaled(&volume_center, &state->surface_normal, center_offset);
    }

    ivp_vec3r_t buoyancy_impulse_ws;
    ivp_real_t buoyancy_scale = state->fluid_density * volume_submerged * dt;
    buoyancy_impulse_ws.k[0] = -env->gravity.k[0] * buoyancy_scale;
    buoyancy_impulse_ws.k[1] = -env->gravity.k[1] * buoyancy_scale;
    buoyancy_impulse_ws.k[2] = -env->gravity.k[2] * buoyancy_scale;
    ivp_core_async_push_ws(core, &volume_center, &buoyancy_impulse_ws);

    ivp_real_t immersion = h / (IVP_REAL(2.0) * radius);
    if (immersion > IVP_REAL(1.0)) immersion = IVP_REAL(1.0);

    if (state->linear_drag > IVP_REAL(0.0)) {
        if (IVP_BUOYANCY_USE_POLYGON_TRI_DRAG &&
            obj->type == IVP_OBJECT_POLYGON && obj->surman) {
            ivp_real_t area_immersion = ivp_buoyancy_apply_polygon_drag(event_sim,
                                                                        obj,
                                                                        state);
            if (area_immersion > immersion) {
                immersion = area_immersion;
            }
        } else {
            ivp_buoyancy_apply_sphere_drag(event_sim, obj, core, state, radius, h);
        }
    }

    if (state->angular_drag > IVP_REAL(0.0)) {
        ivp_real_t damp_fraction = immersion * state->angular_drag * dt;
        if (damp_fraction > IVP_REAL(1.0)) damp_fraction = IVP_REAL(1.0);

        core->rot_speed_change.k[0] -= (ivp_real_t)core->rot_speed.k[0] * damp_fraction;
        core->rot_speed_change.k[1] -= (ivp_real_t)core->rot_speed.k[1] * damp_fraction;
        core->rot_speed_change.k[2] -= (ivp_real_t)core->rot_speed.k[2] * damp_fraction;
    }
}
