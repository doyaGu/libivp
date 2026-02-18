/* ivp_ray_solver.c — Ray casting against compact surfaces
 *
 * Implements ray-triangle intersection and hierarchical ray-surface traversal.
 *
 * For single-triangle ledges (n_triangles == 2):
 *   Plane intersection + barycentric inside test.
 *
 * For multi-triangle convex ledges:
 *   Find entry face, then convex walk across adjacent triangles.
 *
 * Surface-level:
 *   Traverse ledgetree hierarchy, test bounding spheres, recurse into
 *   terminal ledges.
 */
#include <ivp/collision/ivp_ray_solver.h>
#include <ivp/collision/ivp_narrowphase.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/math/ivp_fast_math.h>

/* ── Helper: vertex position as ivp_vec3r_t ─────────────────────────────── */

static inline ivp_vec3r_t point_to_vec3r(const ivp_compact_poly_point_t *p)
{
    return ivp_vec3r((ivp_real_t)p->k[0],
                     (ivp_real_t)p->k[1],
                     (ivp_real_t)p->k[2]);
}

/* ── Ray-sphere intersection ────────────────────────────────────────────── */

bool ivp_ray_test_sphere(const ivp_vec3r_t *origin,
                         const ivp_vec3r_t *direction,
                         const ivp_vec3r_t *center,
                         ivp_real_t         radius,
                         ivp_real_t        *t_out)
{
    /* delta = center - origin */
    ivp_vec3r_t delta;
    ivp_vec3r_sub(&delta, center, origin);

    /* t_closest = dot(delta, direction)  (parametric closest approach) */
    ivp_real_t t_closest = ivp_vec3r_dot(&delta, direction);

    /* Squared distance from origin to center */
    ivp_real_t delta_sq = ivp_vec3r_quad_length(&delta);

    /* Squared distance from closest approach to center:
     * d² = |delta|² - t_closest² */
    ivp_real_t d_sq = delta_sq - t_closest * t_closest;

    ivp_real_t r_sq = radius * radius;

    if (d_sq > r_sq) {
        return false; /* ray misses sphere */
    }

    /* Distance from closest approach to intersection:
     * half_chord = sqrt(r² - d²) */
    ivp_real_t half_chord = ivp_sqrt(r_sq - d_sq);

    /* Two intersection points: t_closest ± half_chord
     * Take the smaller positive root. */
    ivp_real_t t0 = t_closest - half_chord;
    ivp_real_t t1 = t_closest + half_chord;

    if (t1 < IVP_REAL(0.0)) {
        return false; /* sphere is behind ray */
    }

    if (t_out) {
        *t_out = (t0 >= IVP_REAL(0.0)) ? t0 : t1;
    }

    return true;
}

/* ── Ray-triangle intersection ──────────────────────────────────────────── */

ivp_result_t ivp_ray_test_triangle(const ivp_compact_ledge_t    *ledge,
                                    const ivp_compact_triangle_t *tri,
                                    const ivp_ray_input_t        *ray,
                                    ivp_real_t                   *hit_dist)
{
    IVP_ASSERT(ledge != NULL && tri != NULL && ray != NULL && hit_dist != NULL);

    const ivp_compact_edge_t *e0 = &tri->edges[0];
    const ivp_compact_edge_t *e1 = &tri->edges[1];
    const ivp_compact_edge_t *e2 = &tri->edges[2];

    ivp_vec3r_t p0 = point_to_vec3r(ivp_edge_get_start_point(e0, ledge));
    ivp_vec3r_t p1 = point_to_vec3r(ivp_edge_get_start_point(e1, ledge));
    ivp_vec3r_t p2 = point_to_vec3r(ivp_edge_get_start_point(e2, ledge));

    /* Edge vectors */
    ivp_vec3r_t edge01, edge02;
    ivp_vec3r_sub(&edge01, &p1, &p0);
    ivp_vec3r_sub(&edge02, &p2, &p0);

    /* Normal = edge01 × edge02 (unnormalized) */
    ivp_vec3r_t normal;
    ivp_vec3r_cross(&normal, &edge01, &edge02);

    /* denominator = normal · direction */
    ivp_real_t denom = ivp_vec3r_dot(&normal, &ray->direction);

    /* Ray parallel to triangle plane */
    if (ivp_fabs(denom) < IVP_REAL_RES) {
        return IVP_ERR_DEGENERATE_GEOMETRY;
    }

    /* Signed distance from ray origin to triangle plane.
     * t = (normal · (p0 - origin)) / denom */
    ivp_vec3r_t p0_origin;
    ivp_vec3r_sub(&p0_origin, &p0, &ray->origin);
    ivp_real_t t = ivp_vec3r_dot(&normal, &p0_origin) / denom;

    /* Check range */
    if (t < IVP_REAL(0.0) || t > ray->max_distance) {
        return IVP_ERR_DEGENERATE_GEOMETRY;
    }

    /* Compute hit point and check if inside triangle via barycentric coords */
    ivp_vec3r_t hit_point;
    ivp_vec3r_t scaled_dir;
    ivp_vec3r_scale(&scaled_dir, &ray->direction, t);
    ivp_vec3r_add(&hit_point, &ray->origin, &scaled_dir);

    /* Barycentric test */
    ivp_qr_result_t qr;
    ivp_calc_qr_vals(ledge, &tri->edges[0], &hit_point, &qr);

    if (ivp_qr_is_outside(&qr)) {
        return IVP_ERR_DEGENERATE_GEOMETRY;
    }

    *hit_dist = t;
    return IVP_OK;
}

/* ── Ray vs. convex ledge ───────────────────────────────────────────────── */

ivp_result_t ivp_ray_cast_ledge(const ivp_compact_ledge_t *ledge,
                                 const ivp_ray_input_t     *ray,
                                 ivp_ray_hit_t             *hit_out)
{
    IVP_ASSERT(ledge != NULL && ray != NULL && hit_out != NULL);

    int n_tris = ivp_ledge_get_n_triangles(ledge);
    const ivp_compact_triangle_t *tris = ivp_ledge_get_first_triangle(ledge);

    ivp_real_t best_t = ray->max_distance + IVP_REAL(1.0);
    const ivp_compact_triangle_t *best_tri = NULL;

    /* For small ledges, test all triangles.
     * For convex hulls, each pair of triangles shares a geometric triangle
     * (front face + back face), so we test every triangle. The plane
     * intersection + barycentric test handles culling naturally. */

    for (int i = 0; i < n_tris; i++) {
        const ivp_compact_triangle_t *tri = &tris[i];

        /* Skip virtual triangles */
        if (ivp_tri_get_is_virtual(tri)) {
            continue;
        }

        ivp_real_t t;
        ivp_result_t r = ivp_ray_test_triangle(ledge, tri, ray, &t);
        if (ivp_ok(r) && t < best_t) {
            best_t = t;
            best_tri = tri;
        }
    }

    if (best_tri == NULL) {
        return IVP_ERR_DEGENERATE_GEOMETRY;
    }

    /* Fill hit result */
    hit_out->hit_distance = best_t;

    ivp_vec3r_t scaled_dir;
    ivp_vec3r_scale(&scaled_dir, &ray->direction, best_t);
    ivp_vec3r_add(&hit_out->hit_point, &ray->origin, &scaled_dir);

    /* Compute triangle normal */
    ivp_calc_hesse_vec(ledge, &best_tri->edges[0], &hit_out->hit_normal);

    hit_out->hit_ledge = ledge;
    hit_out->hit_triangle = best_tri;
    hit_out->hit_material_index = ivp_tri_get_material_index(best_tri);

    return IVP_OK;
}

/* ── Ray vs. compact surface (with ledgetree traversal) ─────────────────── */

/* Maximum ledgetree traversal stack depth */
#define MAX_TREE_DEPTH 64

typedef struct ray_tree_entry {
    const ivp_compact_ledgetree_node_t *node;
} ray_tree_entry_t;

ivp_result_t ivp_ray_cast_surface(const ivp_compact_surface_t *surface,
                                   const ivp_ray_input_t      *ray,
                                   ivp_ray_hit_t              *hit_out)
{
    IVP_ASSERT(surface != NULL && ray != NULL && hit_out != NULL);

    const ivp_compact_ledgetree_node_t *root =
        ivp_surface_get_ledgetree_root(surface);

    /* Stack for iterative tree traversal */
    ray_tree_entry_t stack[MAX_TREE_DEPTH];
    int stack_top = 0;
    stack[0].node = root;
    stack_top = 1;

    ivp_real_t best_t = ray->max_distance + IVP_REAL(1.0);
    bool found = false;

    while (stack_top > 0) {
        --stack_top;
        const ivp_compact_ledgetree_node_t *node = stack[stack_top].node;

        /* Bounding sphere test */
        ivp_vec3r_t sphere_center = ivp_vec3r(
            (ivp_real_t)node->center[0],
            (ivp_real_t)node->center[1],
            (ivp_real_t)node->center[2]);

        ivp_real_t sphere_t;
        if (!ivp_ray_test_sphere(&ray->origin, &ray->direction,
                                  &sphere_center, (ivp_real_t)node->radius,
                                  &sphere_t)) {
            continue; /* ray doesn't intersect bounding sphere */
        }

        /* If the bounding sphere is further than best hit, skip */
        if (sphere_t > best_t) {
            continue;
        }

        if (ivp_ledgetree_is_terminal(node)) {
            /* Test against the ledge's triangles */
            const ivp_compact_ledge_t *ledge =
                ivp_ledgetree_get_compact_ledge(node);

            ivp_ray_hit_t ledge_hit;
            ivp_ray_input_t ledge_ray = *ray;
            ledge_ray.max_distance = best_t; /* tighten range */

            if (ivp_ok(ivp_ray_cast_ledge(ledge, &ledge_ray, &ledge_hit))) {
                if (ledge_hit.hit_distance < best_t) {
                    best_t = ledge_hit.hit_distance;
                    *hit_out = ledge_hit;
                    found = true;
                }
            }
        } else {
            /* Push children (right first so left is processed first) */
            if (stack_top + 2 <= MAX_TREE_DEPTH) {
                stack[stack_top].node = ivp_ledgetree_right_son(node);
                stack_top++;
                stack[stack_top].node = ivp_ledgetree_left_son(node);
                stack_top++;
            }
        }
    }

    return found ? IVP_OK : IVP_ERR_DEGENERATE_GEOMETRY;
}
