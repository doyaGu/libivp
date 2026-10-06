/* ivp_controller_raycast.c -- private world ray caster for controllers */
#include "ivp_controller_raycast.h"

#include <ivp/collision/ivp_ray_solver.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/math/ivp_fast_math.h>
#include <ivp/math/ivp_transform.h>
#include <math.h>

static bool ray_ball(const ivp_vec3r_t *start, const ivp_vec3r_t *dir, ivp_real_t length,
                     const ivp_vec3r_t *center, ivp_real_t radius,
                     ivp_real_t *t_out, ivp_vec3r_t *normal_out)
{
    ivp_vec3r_t m;
    ivp_vec3r_sub(&m, start, center);
    double b = (double)ivp_vec3r_dot(&m, dir);
    double c = (double)ivp_vec3r_dot(&m, &m) - (double)radius * (double)radius;
    if (c > 0.0 && b > 0.0) return false;
    double disc = b * b - c;
    if (disc < 0.0) return false;
    double t = -b - sqrt(disc);
    if (t < 0.0) t = 0.0;
    if (t > (double)length) return false;
    *t_out = (ivp_real_t)t;
    ivp_vec3r_t p = *start;
    ivp_vec3r_add_scaled(&p, dir, (ivp_real_t)t);
    ivp_vec3r_sub(normal_out, &p, center);
    if (ivp_vec3r_normalize(normal_out) != IVP_OK) {
        ivp_vec3r_scale(normal_out, dir, IVP_REAL(-1.0));
    }
    return true;
}


/* Ray vs compact surface in object space.  ivp_ray_cast_surface() rejects
 * ledgetree nodes whose bounding sphere contains the ray origin (its sphere
 * entry distance is then the far intersection), so the traversal is done
 * here with the entry distance clamped to 0 and ivp_ray_cast_ledge() for
 * the terminal ledges (reference IVP_Ray_Solver::check_ray_against_node). */
#define RAYCAST_MAX_DEPTH 64

static bool ray_node_sphere(const ivp_vec3r_t *o, const ivp_vec3r_t *d,
                            const ivp_compact_ledgetree_node_t *node, ivp_real_t *t_enter)
{
    ivp_vec3r_t c = ivp_vec3r((ivp_real_t)node->center[0], (ivp_real_t)node->center[1],
                              (ivp_real_t)node->center[2]);
    ivp_vec3r_t m;
    ivp_vec3r_sub(&m, o, &c);
    double r = (double)node->radius;
    double b = (double)ivp_vec3r_dot(&m, d);
    double cc = (double)ivp_vec3r_quad_length(&m) - r * r;
    if (cc <= 0.0) { *t_enter = IVP_REAL(0.0); return true; } /* origin inside */
    if (b > 0.0) return false;
    double disc = b * b - cc;
    if (disc < 0.0) return false;
    *t_enter = (ivp_real_t)(-b - sqrt(disc));
    return true;
}

static bool ray_surface_os(const ivp_compact_surface_t *surface, const ivp_ray_input_t *ray,
                           ivp_ray_hit_t *hit_out)
{
    const ivp_compact_ledgetree_node_t *stack[RAYCAST_MAX_DEPTH];
    int top = 0;
    stack[top++] = ivp_surface_get_ledgetree_root(surface);
    ivp_real_t best = ray->max_distance;
    bool found = false;
    while (top > 0) {
        const ivp_compact_ledgetree_node_t *node = stack[--top];
        ivp_real_t t_enter;
        if (!ray_node_sphere(&ray->origin, &ray->direction, node, &t_enter)) continue;
        if (t_enter > best) continue;
        if (ivp_ledgetree_is_terminal(node)) {
            ivp_ray_input_t r = *ray;
            r.max_distance = best;
            ivp_ray_hit_t h;
            if (ivp_ok(ivp_ray_cast_ledge(ivp_ledgetree_get_compact_ledge(node), &r, &h)) &&
                h.hit_distance <= best) {
                best = h.hit_distance;
                *hit_out = h;
                found = true;
            }
        } else if (top + 2 <= RAYCAST_MAX_DEPTH) {
            stack[top++] = ivp_ledgetree_right_son(node);
            stack[top++] = ivp_ledgetree_left_son(node);
        }
    }
    return found;
}

bool ivp_controller_world_raycast(ivp_environment_t *env,
                                  const ivp_object_t *exclude,
                                  const ivp_vec3r_t *start_ws,
                                  const ivp_vec3r_t *dir_ws,
                                  ivp_real_t length,
                                  ivp_real_t *out_distance,
                                  ivp_vec3r_t *out_normal_ws,
                                  ivp_object_t **out_object)
{
    if (!env) return false;
    bool found = false;
    ivp_real_t best = length;
    int n = ivp_environment_get_object_count(env);
    for (int i = 0; i < n; i++) {
        ivp_object_t *o = ivp_environment_get_object(env, i);
        if (!o || o == exclude) continue;
        const ivp_transform_r_t *m = &o->physical_core->m_world_f_core_last_psi;

        /* bounding sphere reject */
        ivp_real_t r = o->physical_core->upper_limit_radius;
        ivp_vec3r_t to_c;
        ivp_vec3r_sub(&to_c, &m->pos, start_ws);
        ivp_real_t along = ivp_vec3r_dot(&to_c, dir_ws);
        ivp_real_t q = ivp_vec3r_quad_length(&to_c) - along * along;
        if (r > IVP_REAL(0.0) && q > r * r) continue;
        if (r > IVP_REAL(0.0) && along + r < IVP_REAL(0.0)) continue;
        if (r > IVP_REAL(0.0) && along - r > best) continue;

        if (o->type == IVP_OBJECT_BALL) {
            ivp_real_t radius = r;
            if (o->surman && ivp_surman_get_type(o->surman) == IVP_SURMAN_BALL) {
                radius = (ivp_real_t)((const ivp_surman_ball_t *)o->surman)->radius;
            }
            ivp_real_t t;
            ivp_vec3r_t nrm;
            if (ray_ball(start_ws, dir_ws, best, &m->pos, radius, &t, &nrm) && t <= best) {
                best = t;
                *out_normal_ws = nrm;
                if (out_object) *out_object = o;
                found = true;
            }
            continue;
        }
        if (!o->surman) continue;
        const ivp_compact_surface_t *surface = ivp_surman_get_polygon_surface(o->surman);
        if (!surface) {
            /* another kind of surface manager (a compact grid terrain ...):
             * let the engine's ray solver walk its ledges */
            ivp_ray_solver_template_t templ;
            templ.ray_start_point = *start_ws;
            templ.ray_normized_direction = *dir_ws;
            templ.ray_length = best;
            templ.ray_flags = IVP_RAY_SOLVER_ALL;
            ivp_ray_solver_min_t rs;
            ivp_ray_solver_min_init(&rs, &templ);
            ivp_ray_solver_check_object(&rs.base, o);
            const ivp_ray_hit_t *h = ivp_ray_solver_min_get_ray_hit(&rs);
            if (!h || h->hit_distance < IVP_REAL(0.0) || h->hit_distance > best) continue;
            best = h->hit_distance;
            ivp_vec3r_t n_ws;
            ivp_transform_dir(&n_ws, m, &h->hit_normal);
            if (ivp_vec3r_normalize(&n_ws) != IVP_OK) ivp_vec3r_scale(&n_ws, dir_ws, IVP_REAL(-1.0));
            if (ivp_vec3r_dot(&n_ws, dir_ws) > IVP_REAL(0.0)) ivp_vec3r_scale(&n_ws, &n_ws, IVP_REAL(-1.0));
            *out_normal_ws = n_ws;
            if (out_object) *out_object = o;
            found = true;
            continue;
        }

        ivp_ray_input_t ray;
        ivp_transform_inv_point(&ray.origin, m, start_ws);
        ivp_transform_inv_dir(&ray.direction, m, dir_ws);
        ray.max_distance = best;
        ivp_ray_hit_t hit = {0};
        if (!ray_surface_os(surface, &ray, &hit)) continue;
        if (hit.hit_distance < IVP_REAL(0.0) || hit.hit_distance > best) continue;
        best = hit.hit_distance;
        ivp_vec3r_t n_ws;
        ivp_transform_dir(&n_ws, m, &hit.hit_normal);
        if (ivp_vec3r_normalize(&n_ws) != IVP_OK) {
            ivp_vec3r_scale(&n_ws, dir_ws, IVP_REAL(-1.0));
        }
        if (ivp_vec3r_dot(&n_ws, dir_ws) > IVP_REAL(0.0)) {
            ivp_vec3r_scale(&n_ws, &n_ws, IVP_REAL(-1.0));
        }
        *out_normal_ws = n_ws;
        if (out_object) *out_object = o;
        found = true;
    }
    if (found) *out_distance = best;
    return found;
}
