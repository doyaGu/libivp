/* demo_debug.h -- Picking, selection and debug overlay state for demo_app.h
 *
 * Included by demo_app.h after the body helpers and before the viewer.
 * Everything here is independent of the renderer:
 *   - exact ray / object intersection (ball, box, convex ledges), so that
 *     hover and selection follow the visible shape instead of a bounding
 *     sphere;
 *   - a ring of recent impacts fed by a global collision listener;
 *   - the toggles of the debug overlays.
 */
#ifndef IVP_EXAMPLE_DEMO_DEBUG_H
#define IVP_EXAMPLE_DEMO_DEBUG_H

#define DEMO_MAX_IMPACTS 64
#define DEMO_IMPACT_LIFE 1.0f      /* seconds an impact marker stays visible */

typedef struct demo_impact {
    float pos[3];
    float nrm[3];
    float speed;       /* closing speed */
    float age;         /* seconds since the impact, < 0 = unused */
} demo_impact_t;

typedef struct demo_debug {
    ivp_object_t *hover;          /* object under the mouse */
    ivp_object_t *selected;       /* last clicked object */
    bool          show_axes;      /* F2: mass centre and axes */
    bool          show_velocity;  /* F3: velocity arrows */
    bool          show_impacts;   /* F4: impact markers */
    demo_impact_t impacts[DEMO_MAX_IMPACTS];
    int           next_impact;
    ivp_collision_listener_t listener;
    bool          listener_attached;
} demo_debug_t;

/* ── Ray / object intersection ────────────────────────────────────────── */

/* Moeller-Trumbore, two sided.  Returns the hit distance in *t. */
static inline bool demo_ray_triangle(const float o[3], const float d[3],
                                     const float a[3], const float b[3],
                                     const float c[3], float *t)
{
    float e1[3] = { b[0]-a[0], b[1]-a[1], b[2]-a[2] };
    float e2[3] = { c[0]-a[0], c[1]-a[1], c[2]-a[2] };
    float p[3]  = { d[1]*e2[2]-d[2]*e2[1], d[2]*e2[0]-d[0]*e2[2], d[0]*e2[1]-d[1]*e2[0] };
    float det = e1[0]*p[0] + e1[1]*p[1] + e1[2]*p[2];
    if (fabsf(det) < 1e-12f) return false;
    float inv = 1.0f / det;
    float s[3] = { o[0]-a[0], o[1]-a[1], o[2]-a[2] };
    float u = (s[0]*p[0] + s[1]*p[1] + s[2]*p[2]) * inv;
    if (u < 0.0f || u > 1.0f) return false;
    float q[3] = { s[1]*e1[2]-s[2]*e1[1], s[2]*e1[0]-s[0]*e1[2], s[0]*e1[1]-s[1]*e1[0] };
    float v = (d[0]*q[0] + d[1]*q[1] + d[2]*q[2]) * inv;
    if (v < 0.0f || u + v > 1.0f) return false;
    float tt = (e2[0]*q[0] + e2[1]*q[1] + e2[2]*q[2]) * inv;
    if (tt <= 0.0f) return false;
    *t = tt;
    return true;
}

typedef struct demo_ray_ctx {
    const float *o, *d;   /* object space */
    float        t;       /* nearest hit so far */
    bool         hit;
} demo_ray_ctx_t;

static inline void demo_ray_ledge(void *ctx, const ivp_compact_ledge_t *ledge)
{
    demo_ray_ctx_t *c = (demo_ray_ctx_t *)ctx;
    const ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array(ledge);
    const ivp_compact_triangle_t *tris = ivp_ledge_get_first_triangle(ledge);
    int n_tri = ivp_ledge_get_n_triangles(ledge);
    for (int i = 0; i < n_tri; i++) {
        const float *a = pts[ivp_edge_get_start_point_index(&tris[i].edges[0])].k;
        const float *b = pts[ivp_edge_get_start_point_index(&tris[i].edges[1])].k;
        const float *e = pts[ivp_edge_get_start_point_index(&tris[i].edges[2])].k;
        float t;
        if (demo_ray_triangle(c->o, c->d, a, b, e, &t) && (!c->hit || t < c->t)) {
            c->t = t;
            c->hit = true;
        }
    }
}

/* Nearest hit of the ray (world space, d normalised) with the object's
 * visible shape.  Objects that were not made by the demo helpers cannot be
 * picked. */
static inline bool demo_ray_object(const ivp_object_t *obj, const float ro[3],
                                   const float rd[3], float *t_out)
{
    const demo_body_t *b = demo_body_of(obj);
    if (!b) return false;

    float pos[3], rot[9];
    demo_object_pose(obj, pos, rot);
    /* ray into object space: rot is column major, so the transpose is the
     * inverse */
    float rel[3] = { ro[0]-pos[0], ro[1]-pos[1], ro[2]-pos[2] };
    float o[3], d[3];
    for (int i = 0; i < 3; i++) {
        o[i] = rot[3*i]*rel[0] + rot[3*i+1]*rel[1] + rot[3*i+2]*rel[2];
        d[i] = rot[3*i]*rd[0]  + rot[3*i+1]*rd[1]  + rot[3*i+2]*rd[2];
    }

    switch (b->shape) {
    case DEMO_SHAPE_BALL: {
        float bq = o[0]*d[0] + o[1]*d[1] + o[2]*d[2];
        float cq = o[0]*o[0] + o[1]*o[1] + o[2]*o[2] - b->radius * b->radius;
        float disc = bq*bq - cq;
        if (disc < 0.0f) return false;
        float s = sqrtf(disc);
        float t = (-bq - s > 0.0f) ? -bq - s : -bq + s;
        if (t <= 0.0f) return false;
        *t_out = t;
        return true;
    }
    case DEMO_SHAPE_BOX: {
        float t0 = 0.0f, t1 = 1e30f;
        for (int i = 0; i < 3; i++) {
            if (fabsf(d[i]) < 1e-12f) {
                if (fabsf(o[i]) > b->half[i]) return false;
                continue;
            }
            float a = (-b->half[i] - o[i]) / d[i];
            float c = ( b->half[i] - o[i]) / d[i];
            if (a > c) { float tmp = a; a = c; c = tmp; }
            if (a > t0) t0 = a;
            if (c < t1) t1 = c;
            if (t0 > t1) return false;
        }
        *t_out = t0 > 0.0f ? t0 : t1;
        return *t_out > 0.0f;
    }
    default: {
        if (!obj->surman) return false;
        demo_ray_ctx_t c = { o, d, 0.0f, false };
        ivp_surman_get_all_terminal_ledges(obj->surman, demo_ray_ledge, &c);
        if (c.hit) *t_out = c.t;
        return c.hit;
    }
    }
}

/* Closest object along the ray, or NULL. */
static inline ivp_object_t *demo_pick_object(ivp_environment_t *env, const float ro[3],
                                             const float rd[3], float *t_out)
{
    ivp_object_t *best = NULL;
    float best_t = 1e30f;
    int n = ivp_environment_get_object_count(env);
    for (int i = 0; i < n; i++) {
        ivp_object_t *obj = ivp_environment_get_object(env, i);
        float t;
        if (obj && demo_ray_object(obj, ro, rd, &t) && t < best_t) {
            best_t = t;
            best = obj;
        }
    }
    if (best && t_out) *t_out = best_t;
    return best;
}

/* ── Impact ring ──────────────────────────────────────────────────────── */

static inline void demo_debug_on_impact(void *ctx, const ivp_collision_event_t *ev)
{
    demo_debug_t *dbg = (demo_debug_t *)ctx;
    demo_impact_t *im = &dbg->impacts[dbg->next_impact];
    dbg->next_impact = (dbg->next_impact + 1) % DEMO_MAX_IMPACTS;
    for (int i = 0; i < 3; i++) {
        im->pos[i] = (float)ev->contact_point.k[i];
        im->nrm[i] = (float)ev->contact_normal.k[i];
    }
    im->speed = (float)fabs((double)ev->closing_speed);
    im->age = 0.0f;
}

static inline void demo_debug_init(demo_debug_t *dbg, ivp_environment_t *env)
{
    memset(dbg, 0, sizeof(*dbg));
    for (int i = 0; i < DEMO_MAX_IMPACTS; i++) dbg->impacts[i].age = -1.0f;
    ivp_collision_listener_init(&dbg->listener, IVP_COLLISION_CB_POST_COLLISION, dbg);
    dbg->listener.post_collision_fn = demo_debug_on_impact;
    dbg->listener_attached = ivp_environment_add_collision_listener_global(env, &dbg->listener) == 0;
}

static inline void demo_debug_destroy(demo_debug_t *dbg, ivp_environment_t *env)
{
    if (dbg->listener_attached && env)
        ivp_environment_remove_collision_listener_global(env, &dbg->listener);
    dbg->listener_attached = false;
}

/* Age the impact markers by wall clock time. */
static inline void demo_debug_tick(demo_debug_t *dbg, float dt)
{
    for (int i = 0; i < DEMO_MAX_IMPACTS; i++) {
        if (dbg->impacts[i].age < 0.0f) continue;
        dbg->impacts[i].age += dt;
        if (dbg->impacts[i].age > DEMO_IMPACT_LIFE) dbg->impacts[i].age = -1.0f;
    }
}

/* A destroyed object must not stay hovered or selected. */
static inline void demo_debug_forget(demo_debug_t *dbg, const ivp_object_t *obj)
{
    if (dbg->hover == obj) dbg->hover = NULL;
    if (dbg->selected == obj) dbg->selected = NULL;
}

#endif /* IVP_EXAMPLE_DEMO_DEBUG_H */
