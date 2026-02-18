/* ivp_liquid_surface.c — Liquid surface descriptor implementation */

#include <ivp/controllers/ivp_liquid_surface.h>

/* ── Simple flat-plane eval callback ───────────────────────────────────── */

static bool simple_eval(void *ctx,
                        const ivp_vec3r_t *query_pos_ws,
                        ivp_vec3r_t *out_surface_normal,
                        ivp_real_t  *out_height,
                        ivp_vec3r_t *out_velocity)
{
    const ivp_liquid_surface_simple_t *ls = (const ivp_liquid_surface_simple_t *)ctx;

    *out_surface_normal = ls->surface_normal;
    *out_height         = ls->surface_height;
    *out_velocity       = ls->flow_velocity;

    /* Check if the query point is below the surface.
     * For a plane defined as normal · p = height, a point is inside
     * (below surface) if normal · p > height (normal points OUT). */
    ivp_real_t dot = ivp_vec3r_dot(&ls->surface_normal, query_pos_ws);
    return dot > ls->surface_height;
}

/* ── Public API ────────────────────────────────────────────────────────── */

void ivp_liquid_surface_simple_init(ivp_liquid_surface_simple_t *ls,
                                    ivp_real_t surface_y,
                                    ivp_real_t density)
{
    ivp_vec3r_t normal;
    ivp_vec3r_set(&normal, IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    ivp_liquid_surface_simple_init_plane(ls, &normal, surface_y, density);
}

void ivp_liquid_surface_simple_init_plane(ivp_liquid_surface_simple_t *ls,
                                          const ivp_vec3r_t *normal,
                                          ivp_real_t height,
                                          ivp_real_t density)
{
    ls->base.eval_fn      = simple_eval;
    ls->base.ctx           = ls;  /* self-referential — simple struct is the context */
    ls->surface_normal    = *normal;
    ls->surface_height    = height;
    ls->fluid_density     = density;
    ivp_vec3r_set(&ls->flow_velocity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
}
