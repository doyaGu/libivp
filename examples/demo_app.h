/* demo_app.h -- Scaffolding of the libivp examples (the ports of the
 *               reference ivp_examples demos / quickstarts and the libivp
 *               feature examples)
 *
 * The reference demos (ivp_examples/examples, ivp_examples/quickstart) all
 * run on one small framework (ive_sample_app / ive_sample_common):
 *   - an environment with gravity (0, 9.81, 0) and a 1/60 s PSI,
 *   - one default IVP_Material_Simple(friction, elasticity),
 *   - create_box / create_ball helpers with the sample templates
 *     (damping 0.01, textbook box inertia m(h^2+d^2)/12 and 2/5 m r^2,
 *     static objects unmoveable + pinned),
 *   - a scene reset that beams the objects back to their start pose,
 *   - an orbit camera, picking / dragging, and an info panel.
 *
 * This header is the libivp equivalent, built on the shared example
 * helpers (examples_common.h, interaction.h, inspect.h):
 *
 *   demo_world_t   environment + every surface / surface manager created
 *                  through the helpers (owned, freed by demo_world_destroy)
 *   demo_viewer_t  renderer, camera, drag / focus interaction, scene
 *                  (Space) and view (Home) reset, fixed-step loop, HUD
 *   demo_app_t     the per-example callbacks; demo_run() runs them either
 *                  in a window or headless.
 *
 * The feature examples keep their own setups: demo_world_init_env takes the
 * example's environment configuration (e.g. the ivp_environment_config_init
 * defaults: gravity 9.83, PSI 1/66 s), demo_create_box_templ /
 * demo_create_ball_templ create objects from the example's own templates,
 * demo_config_t sets the window loop's fixed step, sub-step limit and
 * ground grid, and the viewer_init callback adjusts the window viewer
 * (lighting, dragger settings).
 *
 * Headless mode: `example_xxx --headless [frames]` (or the environment
 * variable IVP_EXAMPLE_HEADLESS=frames) runs the scene for `frames` frames
 * of 1/60 s without opening a window, prints a stats line every second
 * and finishes with the example's self-check (exit code 1 on failure).
 * The example's update callback receives viewer == NULL in that mode and
 * plays a scripted version of its key bindings.
 *
 * Everything is static inline: include this header from exactly the
 * example's own translation unit.
 */
#ifndef IVP_EXAMPLE_DEMO_APP_H
#define IVP_EXAMPLE_DEMO_APP_H

#include "examples_common.h"
#include "interaction.h"
#include "inspect.h"

#include <ivp/base/ivp_types.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/geometry/ivp_compact_edge.h>
#include <ivp/geometry/ivp_compact_ledge.h>
#include <ivp/geometry/ivp_surman.h>

#include <SDL3/SDL.h>

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ── Reference sample constants ───────────────────────────────────────── */

#define DEMO_GRAVITY        9.81   /* ive::create_environment          */
#define DEMO_PSI_TIME       (1.0 / 60.0)
#define DEMO_FRAME_TIME     (1.0f / 60.0f)  /* headless frame length   */
#define DEMO_HEADLESS_DEFAULT_FRAMES 600

/* ── Colour palette (reference ive::color) ────────────────────────────── *
 * Compound literals: valid for the enclosing block, use them directly as
 * arguments. */
#define DEMO_COL_GRID       ((const float[3]){0.30f, 0.30f, 0.30f})
#define DEMO_COL_TITLE      ((const float[3]){1.00f, 1.00f, 1.00f})
#define DEMO_COL_HELP       ((const float[3]){0.60f, 0.60f, 0.60f})
#define DEMO_COL_INFO       ((const float[3]){0.70f, 0.90f, 0.70f})
#define DEMO_COL_OBJECT_A   ((const float[3]){0.20f, 0.80f, 1.00f})
#define DEMO_COL_OBJECT_B   ((const float[3]){1.00f, 0.60f, 0.20f})
#define DEMO_COL_STATIC     ((const float[3]){0.50f, 0.50f, 0.50f})
#define DEMO_COL_VELOCITY   ((const float[3]){1.00f, 0.40f, 0.20f})
#define DEMO_COL_SPRING     ((const float[3]){0.40f, 1.00f, 0.40f})
#define DEMO_COL_HIGHLIGHT  ((const float[3]){1.00f, 1.00f, 0.30f})
#define DEMO_COL_WARNING    ((const float[3]){1.00f, 0.20f, 0.20f})
#define DEMO_COL_WATER      ((const float[3]){0.20f, 0.40f, 0.80f})
#define DEMO_COL_DRAG       ((const float[3]){1.00f, 0.85f, 0.25f})
#define DEMO_COL_FOCUS      ((const float[3]){1.00f, 0.65f, 0.25f})

/* ── Configuration (reference ive::SampleAppConfig) ───────────────────── */

typedef struct demo_config {
    const char *title;        /* window title                          */
    const char *tag;          /* short name for logs ("bridge")        */
    float       orbit_dist;
    float       orbit_pitch;
    float       orbit_yaw;
    float       target[3];
    double      friction;     /* default material                      */
    double      elasticity;
    float       sim_dt;       /* windowed fixed step (0: 1/300 s)      */
    int         max_substeps; /* windowed sub-steps per frame (0: 24)  */
    float       grid_extent;  /* ground grid (0: 30 m, < 0: no grid)   */
    float       grid_spacing; /* (0: 1 m)                              */
} demo_config_t;

/* ive::app_config_defaults */
static inline void demo_config_defaults(demo_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->title       = "IVP Sample";
    cfg->tag         = "demo";
    cfg->orbit_dist  = 20.0f;
    cfg->orbit_pitch = -30.0f;
    cfg->orbit_yaw   = 45.0f;
    cfg->target[0]   = 0.0f;
    cfg->target[1]   = -3.0f;
    cfg->target[2]   = 0.0f;
    cfg->friction    = 0.7;
    cfg->elasticity  = 0.6;
}

/* ── Bodies: geometry owned by the world ──────────────────────────────── */

typedef enum demo_shape {
    DEMO_SHAPE_BOX  = 0,   /* axis aligned box centred at the origin   */
    DEMO_SHAPE_BALL = 1,
    DEMO_SHAPE_POLY = 2    /* any compact surface (drawn as wireframe) */
} demo_shape_t;

typedef struct demo_body {
    demo_shape_t                 shape;
    float                        half[3];   /* DEMO_SHAPE_BOX            */
    float                        radius;    /* DEMO_SHAPE_BALL           */
    ivp_compact_surface_t       *surface;   /* owned (may be NULL)       */
    const ivp_compact_surface_t *geometry;  /* surface of the object     */
    ivp_surman_polygon_t         sm_poly;
    ivp_surman_ball_t            sm_ball;
    ivp_object_t                *obj;
    struct demo_body            *next;
} demo_body_t;

typedef struct demo_saved_state {
    ivp_object_t *obj;
    ivp_vec3r_t   pos;
    ivp_quatr_t   q;
} demo_saved_state_t;

typedef struct demo_world {
    ivp_environment_t  *env;
    ivp_material_t      mat;        /* default material of the config     */
    demo_body_t        *bodies;     /* live bodies                        */
    demo_body_t        *graveyard;  /* destroyed, freed by demo_world_collect */
    demo_saved_state_t *saved;      /* scene reset snapshot               */
    int                 n_saved;
} demo_world_t;

/* IVP_Material_Simple(friction, elasticity) */
static inline ivp_material_t demo_material(double friction, double elasticity)
{
    ivp_material_t m = ivp_material_default();
    m.friction   = (ivp_real_t)friction;
    m.elasticity = (ivp_real_t)elasticity;
    return m;
}

/* Environment from the example's own configuration (e.g. the
 * ivp_environment_config_init defaults: gravity 9.83, PSI 1/66 s). */
static inline bool demo_world_init_env(demo_world_t *w, const demo_config_t *cfg,
                                       const ivp_environment_config_t *ecfg)
{
    memset(w, 0, sizeof(*w));
    w->env = ivp_environment_create(ecfg);
    w->mat = demo_material(cfg->friction, cfg->elasticity);
    if (!w->env) {
        fprintf(stderr, "ivp_environment_create failed\n");
        return false;
    }
    return true;
}

/* ive::create_environment(1/60): gravity 9.81 (Y down), PSI 1/60 s. */
static inline bool demo_world_init(demo_world_t *w, const demo_config_t *cfg)
{
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ecfg.gravity = ivp_vec3r(IVP_REAL(0.0), (ivp_real_t)DEMO_GRAVITY, IVP_REAL(0.0));
    ecfg.delta_psi_time = (ivp_real_t)DEMO_PSI_TIME;
    return demo_world_init_env(w, cfg, &ecfg);
}

static inline void demo_body_free(demo_body_t *b)
{
    if (b->surface) ivp_compact_surface_free(NULL, b->surface);
    free(b);
}

/* Free the bodies of objects destroyed since the last call.  Object
 * deletion may be deferred to the end of the running PSI, so this is only
 * called outside ivp_environment_simulate_*. */
static inline void demo_world_collect(demo_world_t *w)
{
    while (w->graveyard) {
        demo_body_t *b = w->graveyard;
        w->graveyard = b->next;
        demo_body_free(b);
    }
}

/* Destroy the environment (and every object), then the geometry. */
static inline void demo_world_destroy(demo_world_t *w)
{
    if (w->env) ivp_environment_destroy(w->env);
    w->env = NULL;
    demo_world_collect(w);
    while (w->bodies) {
        demo_body_t *b = w->bodies;
        w->bodies = b->next;
        demo_body_free(b);
    }
    free(w->saved);
    w->saved = NULL;
    w->n_saved = 0;
}

static inline demo_body_t *demo_body_of(const ivp_object_t *obj)
{
    return obj ? (demo_body_t *)obj->client_data : NULL;
}

/* ive::configure_dynamic_template */
static inline void demo_template_dynamic(ivp_object_template_t *t,
                                         const ivp_material_t *mat,
                                         double mass)
{
    ivp_object_template_init(t);
    t->physical_unmoveable = false;
    t->pinned = false;
    t->enable_piling_optimization = false;
    t->mass = (ivp_real_t)mass;
    t->material = *mat;
    t->speed_damp_factor = IVP_REAL(0.01);
    t->rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.01), IVP_REAL(0.01), IVP_REAL(0.01));
    t->extra_radius = IVP_REAL(0.0);
}

/* ive::configure_static_template */
static inline void demo_template_static(ivp_object_template_t *t,
                                        const ivp_material_t *mat)
{
    ivp_object_template_init(t);
    t->physical_unmoveable = true;
    t->pinned = true;
    t->enable_piling_optimization = false;
    t->mass = IVP_REAL(0.0);
    t->material = *mat;
    t->speed_damp_factor = IVP_REAL(0.0);
    t->rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    t->extra_radius = IVP_REAL(0.0);
}

/* ive::set_box_inertia: textbook box inertia with full extents, absolute. */
static inline void demo_set_box_inertia(ivp_object_template_t *t, double mass,
                                        double hx, double hy, double hz)
{
    double w = 2.0 * hx, h = 2.0 * hy, d = 2.0 * hz;
    t->rot_inertia_is_factor = false;
    t->rot_inertia = ivp_vec3r((ivp_real_t)(mass * (h * h + d * d) / 12.0),
                               (ivp_real_t)(mass * (w * w + d * d) / 12.0),
                               (ivp_real_t)(mass * (w * w + h * h) / 12.0));
}

/* ive::set_sphere_inertia: 2/5 m r^2, absolute. */
static inline void demo_set_sphere_inertia(ivp_object_template_t *t, double mass,
                                           double radius)
{
    ivp_real_t inertia = (ivp_real_t)(0.4 * mass * radius * radius);
    t->rot_inertia_is_factor = false;
    t->rot_inertia = ivp_vec3r(inertia, inertia, inertia);
}

static inline void demo_template_pose(ivp_object_template_t *t, const ivp_quatr_t *q,
                                      double x, double y, double z)
{
    t->position = ivp_vec3r((ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
    t->orientation = q ? *q : ivp_quatr_identity();
}

static inline ivp_object_t *demo_world_add_body(demo_world_t *w, demo_body_t *b,
                                                ivp_object_template_t *t,
                                                const ivp_surman_t *sm)
{
    t->client_data = b;
    b->obj = ivp_environment_create_object(w->env, t, sm);
    if (!b->obj) {
        demo_body_free(b);
        return NULL;
    }
    b->next = w->bodies;
    w->bodies = b;
    return b->obj;
}

/* ive::create_box: mass 0 creates a static box.  q may be NULL. */
static inline ivp_object_t *demo_create_box(demo_world_t *w, const ivp_material_t *mat,
                                            double hx, double hy, double hz, double mass,
                                            const ivp_quatr_t *q,
                                            double x, double y, double z)
{
    demo_body_t *b = (demo_body_t *)calloc(1, sizeof(*b));
    if (!b) return NULL;
    b->shape = DEMO_SHAPE_BOX;
    b->half[0] = (float)hx; b->half[1] = (float)hy; b->half[2] = (float)hz;
    b->surface = ivp_build_box_surface(NULL, (float)hx, (float)hy, (float)hz);
    if (!b->surface) { free(b); return NULL; }
    b->geometry = b->surface;
    ivp_surman_polygon_init(&b->sm_poly, b->surface);

    ivp_object_template_t t;
    if (mass > 0.0) {
        demo_template_dynamic(&t, mat ? mat : &w->mat, mass);
        demo_set_box_inertia(&t, mass, hx, hy, hz);
    } else {
        demo_template_static(&t, mat ? mat : &w->mat);
    }
    t.type = IVP_OBJECT_POLYGON;
    demo_template_pose(&t, q, x, y, z);
    return demo_world_add_body(w, b, &t, &b->sm_poly.base);
}

/* ive::create_ball */
static inline ivp_object_t *demo_create_ball(demo_world_t *w, const ivp_material_t *mat,
                                             double radius, double mass,
                                             const ivp_quatr_t *q,
                                             double x, double y, double z)
{
    demo_body_t *b = (demo_body_t *)calloc(1, sizeof(*b));
    if (!b) return NULL;
    b->shape = DEMO_SHAPE_BALL;
    b->radius = (float)radius;
    ivp_surman_ball_init(&b->sm_ball, (float)radius);

    ivp_object_template_t t;
    if (mass > 0.0) {
        demo_template_dynamic(&t, mat ? mat : &w->mat, mass);
        demo_set_sphere_inertia(&t, mass, radius);
    } else {
        demo_template_static(&t, mat ? mat : &w->mat);
    }
    t.type = IVP_OBJECT_BALL;
    t.ball_radius = (ivp_real_t)radius;
    demo_template_pose(&t, q, x, y, z);
    return demo_world_add_body(w, b, &t, &b->sm_ball.base);
}

/* env->create_polygon(new IVP_SurfaceManager_Polygon(surface), templ, q, pos)
 * for an arbitrary compact surface.  The world takes ownership of
 * `surface`.  The template's client_data is overwritten. */
static inline ivp_object_t *demo_create_polygon(demo_world_t *w,
                                                ivp_compact_surface_t *surface,
                                                ivp_object_template_t *t)
{
    demo_body_t *b = (demo_body_t *)calloc(1, sizeof(*b));
    if (!b) {
        ivp_compact_surface_free(NULL, surface);
        return NULL;
    }
    b->shape = DEMO_SHAPE_POLY;
    b->surface = surface;
    b->geometry = surface;
    ivp_surman_polygon_init(&b->sm_poly, surface);
    t->type = IVP_OBJECT_POLYGON;
    return demo_world_add_body(w, b, t, &b->sm_poly.base);
}

/* A box object built from the example's own template (mass, inertia,
 * material, damping, pose, unmoveable ...); t->type is set here.  The box
 * surface is owned by the world. */
static inline ivp_object_t *demo_create_box_templ(demo_world_t *w, ivp_object_template_t *t,
                                                  double hx, double hy, double hz)
{
    demo_body_t *b = (demo_body_t *)calloc(1, sizeof(*b));
    if (!b) return NULL;
    b->shape = DEMO_SHAPE_BOX;
    b->half[0] = (float)hx; b->half[1] = (float)hy; b->half[2] = (float)hz;
    b->surface = ivp_build_box_surface(NULL, (float)hx, (float)hy, (float)hz);
    if (!b->surface) { free(b); return NULL; }
    b->geometry = b->surface;
    ivp_surman_polygon_init(&b->sm_poly, b->surface);
    t->type = IVP_OBJECT_POLYGON;
    return demo_world_add_body(w, b, t, &b->sm_poly.base);
}

/* A ball object built from the example's own template; t->type and
 * t->ball_radius are set here. */
static inline ivp_object_t *demo_create_ball_templ(demo_world_t *w, ivp_object_template_t *t,
                                                   double radius)
{
    demo_body_t *b = (demo_body_t *)calloc(1, sizeof(*b));
    if (!b) return NULL;
    b->shape = DEMO_SHAPE_BALL;
    b->radius = (float)radius;
    ivp_surman_ball_init(&b->sm_ball, (float)radius);
    t->type = IVP_OBJECT_BALL;
    t->ball_radius = (ivp_real_t)radius;
    return demo_world_add_body(w, b, t, &b->sm_ball.base);
}

/* Forget an object in the reset snapshot. */
static inline void demo_world_forget_saved(demo_world_t *w, const ivp_object_t *obj)
{
    for (int i = 0; i < w->n_saved; i++) {
        if (w->saved[i].obj == obj) {
            w->saved[i] = w->saved[w->n_saved - 1];
            w->n_saved--;
            return;
        }
    }
}

/* IVP_Real_Object::delete_and_check_vicinity for an object created through
 * the helpers.  The geometry is released by the next demo_world_collect. */
static inline void demo_destroy_object(demo_world_t *w, ivp_object_t *obj)
{
    demo_body_t *b = demo_body_of(obj);
    demo_world_forget_saved(w, obj);
    for (demo_body_t **pp = &w->bodies; *pp; pp = &(*pp)->next) {
        if (*pp == b) {
            *pp = b->next;
            b->next = w->graveyard;
            w->graveyard = b;
            break;
        }
    }
    ivp_environment_destroy_object(w->env, obj);
}

/* ive::app_save_initial_state: remember the pose of every dynamic object. */
static inline void demo_world_save_state(demo_world_t *w)
{
    int n = 0;
    for (demo_body_t *b = w->bodies; b; b = b->next) n++;
    free(w->saved);
    w->saved = (demo_saved_state_t *)calloc((size_t)(n > 0 ? n : 1), sizeof(*w->saved));
    w->n_saved = 0;
    if (!w->saved) return;
    for (demo_body_t *b = w->bodies; b; b = b->next) {
        if (!b->obj || b->obj->physical_core->physical_unmoveable) continue;
        demo_saved_state_t *s = &w->saved[w->n_saved++];
        s->obj = b->obj;
        s->pos = b->obj->physical_core->pos_world_f_core_last_psi;
        s->q = b->obj->physical_core->q_world_f_core_last_psi;
    }
}

/* Reference reset_objects (beam_object_to_new_position + zero speed).
 * libivp has no public beam_object_to_new_position, so the core pose is
 * set directly and the object is woken up. */
static inline void demo_world_restore_state(demo_world_t *w)
{
    for (int i = 0; i < w->n_saved; i++) {
        demo_saved_state_t *s = &w->saved[i];
        ivp_environment_ensure_core_in_simulation(w->env, s->obj);
        ivp_core_beam_to(s->obj->physical_core, &s->pos, &s->q);
    }
}

/* ── Object helpers ───────────────────────────────────────────────────── */

/* IVP_Real_Object::async_add_speed_object_ws */
static inline void demo_async_add_speed_ws(ivp_environment_t *env, ivp_object_t *obj,
                                           double vx, double vy, double vz)
{
    if (!obj || obj->physical_core->physical_unmoveable) return;
    ivp_environment_ensure_core_in_simulation(env, obj);
    obj->physical_core->speed_change.k[0] += (ivp_real_t)vx;
    obj->physical_core->speed_change.k[1] += (ivp_real_t)vy;
    obj->physical_core->speed_change.k[2] += (ivp_real_t)vz;
}

/* IVP_Real_Object::async_add_rot_speed_object_cs */
static inline void demo_async_add_rot_speed_cs(ivp_environment_t *env, ivp_object_t *obj,
                                               double wx, double wy, double wz)
{
    if (!obj || obj->physical_core->physical_unmoveable) return;
    ivp_environment_ensure_core_in_simulation(env, obj);
    obj->physical_core->rot_speed_change.k[0] += (ivp_real_t)wx;
    obj->physical_core->rot_speed_change.k[1] += (ivp_real_t)wy;
    obj->physical_core->rot_speed_change.k[2] += (ivp_real_t)wz;
}

/* IVP_Real_Object::async_push_object_ws (impulse at a world point). */
static inline void demo_async_push_ws(ivp_environment_t *env, ivp_object_t *obj,
                                      const ivp_vec3r_t *point_ws,
                                      const ivp_vec3r_t *impulse_ws)
{
    if (!obj || obj->physical_core->physical_unmoveable) return;
    ivp_environment_ensure_core_in_simulation(env, obj);
    ivp_core_async_push_ws(obj->physical_core, point_ws, impulse_ws);
}

/* World pose of the object frame (core pose combined with the object's
 * shift_core_f_object / q_core_f_object, like IVP_Cache_Object). */
static inline void demo_object_pose(const ivp_object_t *obj, float pos[3], float rot[9])
{
    const ivp_core_t *core = obj->physical_core;
    ivp_quatr_t q = core->q_world_f_core_last_psi;
    ivp_vec3r_t p = core->pos_world_f_core_last_psi;
    if (!obj->shift_core_f_object_is_zero) {
        ivp_mat3r_t m;
        ivp_quat_to_mat3(&q, &m);
        for (int r = 0; r < 3; r++) {
            p.k[r] += m.rows[r].k[0] * obj->shift_core_f_object.k[0]
                    + m.rows[r].k[1] * obj->shift_core_f_object.k[1]
                    + m.rows[r].k[2] * obj->shift_core_f_object.k[2];
        }
    }
    if (obj->q_core_f_object) ivp_quat_mult(&q, &q, obj->q_core_f_object);
    vec3r_to_float3(&p, pos);
    if (rot) quat_to_col_rot(&q, rot);
}

/* World position of a point given in the core space of obj (NULL = the
 * point is already in world space).  Anchors of springs / constraints are
 * stored in core space. */
static inline void demo_core_point_ws(const ivp_object_t *obj, const ivp_vec3r_t *p_cs,
                                      float out[3])
{
    if (!obj) {
        vec3r_to_float3(p_cs, out);
        return;
    }
    const ivp_core_t *core = obj->physical_core;
    ivp_mat3r_t m;
    ivp_quat_to_mat3(&core->q_world_f_core_last_psi, &m);
    for (int r = 0; r < 3; r++) {
        out[r] = (float)(core->pos_world_f_core_last_psi.k[r]
                         + m.rows[r].k[0] * p_cs->k[0]
                         + m.rows[r].k[1] * p_cs->k[1]
                         + m.rows[r].k[2] * p_cs->k[2]);
    }
}

/* Mass centre (core position) as floats. */
static inline void demo_object_center(const ivp_object_t *obj, float out[3])
{
    vec3r_to_float3(&obj->physical_core->pos_world_f_core_last_psi, out);
}

static inline float demo_object_speed(const ivp_object_t *obj)
{
    return vec3r_speed(&obj->physical_core->speed);
}

static inline float demo_distance(const float a[3], const float b[3])
{
    float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

/* Distance between the mass centres of two objects. */
static inline float demo_object_distance(const ivp_object_t *a, const ivp_object_t *b)
{
    float pa[3], pb[3];
    demo_object_center(a, pa);
    demo_object_center(b, pb);
    return demo_distance(pa, pb);
}

/* Angle (degrees) between the object's local Y axis and the world Y axis. */
static inline float demo_object_tilt_deg(const ivp_object_t *obj)
{
    float rot[9];
    quat_to_col_rot(&obj->physical_core->q_world_f_core_last_psi, rot);
    float c = rot[4];               /* column 1 (local Y), world Y component */
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    return acosf(fabsf(c)) * (180.0f / 3.14159265f);
}

/* All object positions finite and inside a sane box. */
static inline bool demo_world_check_finite(const demo_world_t *w, float limit)
{
    bool ok = true;
    int n = ivp_environment_get_object_count(w->env);
    for (int i = 0; i < n; i++) {
        const ivp_object_t *obj = ivp_environment_get_object(w->env, i);
        if (!obj) continue;
        for (int k = 0; k < 3; k++) {
            double v = (double)obj->physical_core->pos_world_f_core_last_psi.k[k];
            if (!isfinite(v) || fabs(v) > (double)limit) ok = false;
        }
    }
    return ok;
}

/* Report one self-check condition. */
static inline bool demo_check(const char *tag, bool ok, const char *fmt, ...)
{
    printf("[%s check] %s: ", tag, ok ? "ok  " : "FAIL");
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    putchar('\n');
    return ok;
}

/* ── Viewer ───────────────────────────────────────────────────────────── */

typedef struct demo_viewer {
    ivp_renderer_t                *renderer;
    ivp_camera_t                   cam;
    ivp_camera_defaults_t          cam_defaults;
    ivp_example_dragger_t          dragger;
    ivp_example_focus_controller_t focus;
    ivp_example_input_tracker_t    input;
    ivp_example_inspector_t        inspector;
    sim_loop_t                     loop;
    const char                    *title;
    float                          frame_dt;
    int                            win_w, win_h;
    bool                           reset_requested;  /* Space this frame */
    float                          hud_y;
    bool                           key_prev[SDL_SCANCODE_COUNT];
    /* primitives submitted by the draw callbacks; with renderer == NULL
     * (headless draw smoke test) they are only counted and validated */
    long                           n_prims;
    long                           n_bad_prims;   /* non-finite coordinates */
} demo_viewer_t;

static inline void demo_camera_setup(ivp_camera_t *cam, const demo_config_t *cfg)
{
    ivp_camera_init(cam);
    cam->orbit_distance = cfg->orbit_dist;
    cam->orbit_pitch    = cfg->orbit_pitch;
    cam->orbit_yaw      = cfg->orbit_yaw;
    cam->target[0]      = cfg->target[0];
    cam->target[1]      = cfg->target[1];
    cam->target[2]      = cfg->target[2];
    ivp_camera_update_orbit(cam);
}

static inline bool demo_viewer_init(demo_viewer_t *v, const demo_config_t *cfg,
                                    demo_world_t *w)
{
    memset(v, 0, sizeof(*v));
    ivp_render_config_t rcfg;
    ivp_render_config_init(&rcfg);
    rcfg.width  = 1280;
    rcfg.height = 720;
    rcfg.title  = cfg->title;
    rcfg.vsync  = true;
    ivp_renderer_create(&v->renderer, &rcfg);
    if (!v->renderer) {
        fprintf(stderr, "renderer init failed (try --headless)\n");
        return false;
    }
    ivp_renderer_set_clear_color(v->renderer, 0.26f, 0.30f, 0.36f, 1.0f);
    v->title = cfg->title;

    demo_camera_setup(&v->cam, cfg);
    ivp_camera_store_defaults(&v->cam_defaults, &v->cam);

    ivp_example_dragger_init(&v->dragger);
    ivp_example_dragger_attach(&v->dragger, w->env, 100);
    ivp_example_focus_init(&v->focus);
    ivp_example_input_tracker_init(&v->input);
    ivp_example_inspector_init(&v->inspector, cfg->tag);
    sim_loop_init(&v->loop, v->renderer);
    if (cfg->sim_dt > 0.0f) v->loop.sim_dt = cfg->sim_dt;
    if (cfg->max_substeps > 0) v->loop.max_substeps = cfg->max_substeps;
    return true;
}

static inline void demo_viewer_destroy(demo_viewer_t *v, demo_world_t *w)
{
    ivp_example_dragger_detach(&v->dragger, w->env);
    if (v->renderer) ivp_renderer_destroy(v->renderer);
    v->renderer = NULL;
}

/* Forget an object about to be destroyed (dragger / focus). */
static inline void demo_viewer_forget(demo_viewer_t *v, const ivp_object_t *obj)
{
    if (!v) return;
    if (v->dragger.target == obj) {
        v->dragger.active = false;
        v->dragger.target = NULL;
        v->dragger.has_target_point = false;
        ivp_vec3r_set_zero(&v->dragger.force);
    }
    if (v->focus.focused == obj) v->focus.focused = NULL;
}

/* Edge triggered key query (SDL scancode). */
static inline bool demo_key_just_pressed(demo_viewer_t *v, SDL_Scancode sc)
{
    if ((int)sc < 0 || (int)sc >= SDL_SCANCODE_COUNT) return false;
    const bool *keys = SDL_GetKeyboardState(NULL);
    bool down = keys[sc];
    bool was = v->key_prev[sc];
    v->key_prev[sc] = down;
    return down && !was;
}

static inline bool demo_viewer_begin_frame(demo_viewer_t *v, demo_world_t *w)
{
    if (!ivp_renderer_begin_frame(v->renderer, &v->cam)) return false;
    v->frame_dt = sim_loop_begin(&v->loop, v->renderer);

    ivp_example_input_tracker_update(&v->input);
    v->reset_requested = false;
    if (v->input.scene_reset_requested) {
        demo_world_restore_state(w);
        v->loop.sim_accum = 0.0f;
        v->reset_requested = true;
    }
    if (v->input.view_reset_requested) {
        ivp_camera_restore_defaults(&v->cam, &v->cam_defaults);
    }

    ivp_renderer_get_size(v->renderer, &v->win_w, &v->win_h);
    ivp_example_dragger_update(&v->dragger, &v->cam, w->env, v->win_w, v->win_h, v->frame_dt);
    ivp_example_focus_update(&v->focus, &v->cam, w->env, v->frame_dt);
    v->hud_y = 32.0f;
    return true;
}

static inline void demo_viewer_step(demo_viewer_t *v, demo_world_t *w)
{
    sim_loop_step(&v->loop, w->env, v->frame_dt);
    demo_world_collect(w);
}

/* One info line below the title (reference app_nk_label). */
static inline void demo_hud(demo_viewer_t *v, const float color[3], const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    v->n_prims++;
    if (v->renderer) ivp_render_draw_text(v->renderer, 10.0f, v->hud_y, buf, color);
    v->hud_y += 18.0f;
}

/* Reference app_nk_object_info */
static inline void demo_hud_object(demo_viewer_t *v, const char *label, const ivp_object_t *obj)
{
    float p[3];
    demo_object_center(obj, p);
    demo_hud(v, DEMO_COL_INFO, "%s: pos (%.2f, %.2f, %.2f)  speed %.2f%s",
             label, (double)p[0], (double)p[1], (double)p[2],
             (double)demo_object_speed(obj),
             ivp_object_is_frozen(obj) ? "  [frozen]" : "");
}

/* Title / FPS line, the example's controls (newline separated) and the
 * shared interaction help at the bottom of the window. */
static inline void demo_viewer_end_frame(demo_viewer_t *v, const char *controls)
{
    ivp_example_interaction_draw_overlays(v->renderer, &v->dragger, &v->focus,
                                          DEMO_COL_DRAG, DEMO_COL_FOCUS);
    {
        char buf[192];
        snprintf(buf, sizeof(buf), "%s   FPS: %.0f", v->title ? v->title : "",
                 (double)v->loop.fps_display);
        ivp_render_draw_text(v->renderer, 10.0f, 10.0f, buf, DEMO_COL_TITLE);
    }

    int n_lines = 0;
    const char *lines[16];
    int lens[16];
    if (controls) {
        const char *s = controls;
        while (*s && n_lines < 16) {
            const char *e = strchr(s, '\n');
            int len = e ? (int)(e - s) : (int)strlen(s);
            lines[n_lines] = s;
            lens[n_lines] = len;
            n_lines++;
            if (!e) break;
            s = e + 1;
        }
    }
    float y = (float)v->win_h - 18.0f * (float)(n_lines + 3) - 10.0f;
    for (int i = 0; i < n_lines; i++) {
        char buf[192];
        snprintf(buf, sizeof(buf), "%.*s", lens[i], lines[i]);
        ivp_render_draw_text(v->renderer, 10.0f, y, buf, DEMO_COL_HIGHLIGHT);
        y += 18.0f;
    }
    ivp_example_draw_interaction_hud(v->renderer, 10.0f, y, DEMO_COL_HELP, DEMO_COL_HELP);
    ivp_renderer_end_frame(v->renderer);
}

/* ── Drawing ──────────────────────────────────────────────────────────── *
 * Every primitive goes through these wrappers: they count it, reject
 * non-finite coordinates and skip the GPU call when the viewer has no
 * renderer (headless draw smoke test). */

static inline bool demo_finite3(const float p[3])
{
    return isfinite(p[0]) && isfinite(p[1]) && isfinite(p[2]);
}

static inline bool demo_prim(demo_viewer_t *v, const float *a, const float *b)
{
    v->n_prims++;
    if ((a && !demo_finite3(a)) || (b && !demo_finite3(b))) {
        v->n_bad_prims++;
        return false;
    }
    return v->renderer != NULL;
}

static inline void demo_line(demo_viewer_t *v, const float a[3], const float b[3], const float color[3])
{
    if (demo_prim(v, a, b)) ivp_render_draw_line(v->renderer, a, b, color);
}

static inline void demo_arrow(demo_viewer_t *v, const float from[3], const float to[3],
                              const float color[3], float head)
{
    if (demo_prim(v, from, to)) ivp_render_draw_arrow(v->renderer, from, to, color, head);
}

static inline void demo_cross(demo_viewer_t *v, const float pos[3], float size, const float color[3])
{
    if (demo_prim(v, pos, NULL)) ivp_render_draw_cross(v->renderer, pos, size, color);
}

static inline void demo_point(demo_viewer_t *v, const float pos[3], float size, const float color[3])
{
    if (demo_prim(v, pos, NULL)) ivp_render_draw_point(v->renderer, pos, size, color);
}

static inline void demo_sphere(demo_viewer_t *v, const float pos[3], float radius, const float color[3])
{
    if (demo_prim(v, pos, NULL)) ivp_render_draw_wire_sphere(v->renderer, pos, radius, color);
}

static inline void demo_thick_line(demo_viewer_t *v, const float a[3], const float b[3],
                                   const float color[3], float thickness)
{
    if (demo_prim(v, a, b)) ivp_render_draw_thick_line(v->renderer, a, b, color, thickness);
}

static inline void demo_solid_sphere(demo_viewer_t *v, const float pos[3], float radius,
                                     const float color[3])
{
    if (demo_prim(v, pos, NULL)) ivp_render_draw_solid_sphere(v->renderer, pos, radius, color);
}

/* Coordinate axes at pos (rot: 9 floats column major, or NULL). */
static inline void demo_axes(demo_viewer_t *v, const float pos[3], const float *rot, float scale)
{
    bool draw = demo_prim(v, pos, rot);
    if (rot && !(demo_finite3(rot + 3) && demo_finite3(rot + 6))) {
        v->n_bad_prims++;
        draw = false;
    }
    if (draw) ivp_render_draw_axes(v->renderer, pos, rot, scale);
}

static inline void demo_box(demo_viewer_t *v, const float pos[3], const float half[3],
                            const float rot[9], const float color[3])
{
    bool draw = demo_prim(v, pos, rot);
    if (rot && !(demo_finite3(rot + 3) && demo_finite3(rot + 6))) {
        v->n_bad_prims++;
        draw = false;
    }
    if (draw) ivp_render_draw_wire_box(v->renderer, pos, half, rot, color);
}

static inline void demo_solid_box(demo_viewer_t *v, const float pos[3], const float half[3],
                                  const float rot[9], const float color[3])
{
    bool draw = demo_prim(v, pos, rot);
    if (rot && !(demo_finite3(rot + 3) && demo_finite3(rot + 6))) {
        v->n_bad_prims++;
        draw = false;
    }
    if (draw) ivp_render_draw_solid_box(v->renderer, pos, half, rot, color);
}

typedef struct demo_wire_ctx {
    demo_viewer_t  *v;
    float           pos[3];
    float           rot[9];   /* column major */
    const float    *color;
} demo_wire_ctx_t;

static inline void demo_wire_xform(const demo_wire_ctx_t *c, const float p[3], float out[3])
{
    for (int i = 0; i < 3; i++) {
        out[i] = c->pos[i] + c->rot[0 * 3 + i] * p[0]
                           + c->rot[1 * 3 + i] * p[1]
                           + c->rot[2 * 3 + i] * p[2];
    }
}

static inline void demo_tri_normal(const ivp_compact_ledge_t *ledge,
                                   const ivp_compact_triangle_t *tri, float n[3])
{
    const ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array(ledge);
    const float *a = pts[ivp_edge_get_start_point_index(&tri->edges[0])].k;
    const float *b = pts[ivp_edge_get_start_point_index(&tri->edges[1])].k;
    const float *c = pts[ivp_edge_get_start_point_index(&tri->edges[2])].k;
    float u[3] = { b[0] - a[0], b[1] - a[1], b[2] - a[2] };
    float w[3] = { c[0] - a[0], c[1] - a[1], c[2] - a[2] };
    n[0] = u[1] * w[2] - u[2] * w[1];
    n[1] = u[2] * w[0] - u[0] * w[2];
    n[2] = u[0] * w[1] - u[1] * w[0];
    float len = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (len > 1e-12f) { n[0] /= len; n[1] /= len; n[2] /= len; }
}

/* Draw the feature edges of one convex ledge (edges between coplanar
 * triangles are skipped). */
static inline void demo_wire_ledge(void *ctx, const ivp_compact_ledge_t *ledge)
{
    const demo_wire_ctx_t *c = (const demo_wire_ctx_t *)ctx;
    const ivp_compact_poly_point_t *pts = ivp_ledge_get_point_array(ledge);
    const ivp_compact_triangle_t *tris = ivp_ledge_get_first_triangle(ledge);
    int n_tri = ivp_ledge_get_n_triangles(ledge);
    for (int t = 0; t < n_tri; t++) {
        const ivp_compact_triangle_t *tri = &tris[t];
        float n0[3];
        demo_tri_normal(ledge, tri, n0);
        for (int e = 0; e < 3; e++) {
            const ivp_compact_edge_t *edge = &tri->edges[e];
            const ivp_compact_edge_t *opp = ivp_edge_get_opposite(edge);
            const ivp_compact_triangle_t *opp_tri = ivp_edge_get_triangle(opp);
            if (opp_tri < tri) continue;            /* each edge once */
            if (opp_tri != tri) {
                float n1[3];
                demo_tri_normal(ledge, opp_tri, n1);
                if (n0[0] * n1[0] + n0[1] * n1[1] + n0[2] * n1[2] > 0.999f) continue;
            }
            const float *a = pts[ivp_edge_get_start_point_index(edge)].k;
            const float *b = pts[ivp_edge_get_start_point_index(&tri->edges[(e + 1) % 3])].k;
            float wa[3], wb[3];
            demo_wire_xform(c, a, wa);
            demo_wire_xform(c, b, wb);
            demo_line(c->v, wa, wb, c->color);
        }
    }
}

/* Wireframe of the object's compact surface (all terminal ledges). */
static inline void demo_draw_surface(demo_viewer_t *v, const ivp_object_t *obj,
                                     const float color[3])
{
    demo_wire_ctx_t c;
    c.v = v;
    c.color = color;
    demo_object_pose(obj, c.pos, c.rot);
    ivp_surman_get_all_terminal_ledges(obj->surman, demo_wire_ledge, &c);
}

/* ive::draw_object_box / draw_object_ball for any helper-created object. */
static inline void demo_draw_object(demo_viewer_t *v, const ivp_object_t *obj,
                                    const float color[3])
{
    const demo_body_t *b = demo_body_of(obj);
    if (!b) {
        demo_draw_surface(v, obj, color);
        return;
    }
    float pos[3], rot[9];
    demo_object_pose(obj, pos, rot);
    switch (b->shape) {
    case DEMO_SHAPE_BOX:
        demo_box(v, pos, b->half, rot, color);
        break;
    case DEMO_SHAPE_BALL: {
        demo_sphere(v, pos, b->radius, color);
        /* a spoke so that rolling is visible */
        float spoke[3] = { pos[0] + rot[0] * b->radius,
                           pos[1] + rot[1] * b->radius,
                           pos[2] + rot[2] * b->radius };
        demo_line(v, pos, spoke, color);
        break;
    }
    case DEMO_SHAPE_POLY:
    default:
        demo_draw_surface(v, obj, color);
        break;
    }
}

/* ive::draw_spring_line: line between the mass centres. */
static inline void demo_draw_link(demo_viewer_t *v, const ivp_object_t *a,
                                  const ivp_object_t *b, const float color[3])
{
    float pa[3], pb[3];
    demo_object_center(a, pa);
    demo_object_center(b, pb);
    demo_line(v, pa, pb, color);
}

/* ive::draw_velocity_arrow */
static inline void demo_draw_velocity(demo_viewer_t *v, const ivp_object_t *obj,
                                      const float color[3])
{
    float speed = demo_object_speed(obj);
    if (speed < VEL_ARROW_THRESHOLD) return;
    float p[3];
    demo_object_center(obj, p);
    float to[3] = { p[0] + (float)obj->physical_core->speed.k[0] * VEL_ARROW_SCALE,
                    p[1] + (float)obj->physical_core->speed.k[1] * VEL_ARROW_SCALE,
                    p[2] + (float)obj->physical_core->speed.k[2] * VEL_ARROW_SCALE };
    demo_arrow(v, p, to, color, VEL_ARROW_HEAD);
}

/* ive::app_draw_grid (any extent / spacing) */
static inline void demo_draw_grid_ex(demo_viewer_t *v, float extent, float spacing,
                                     const float color[3])
{
    v->n_prims++;
    if (v->renderer) ivp_render_draw_ground_grid(v->renderer, extent, spacing, color);
}

static inline void demo_draw_grid(demo_viewer_t *v)
{
    demo_draw_grid_ex(v, 30.0f, 1.0f, DEMO_COL_GRID);
}

/* ── App runner ───────────────────────────────────────────────────────── */

typedef struct demo_app demo_app_t;

struct demo_app {
    demo_config_t cfg;
    demo_world_t  world;
    void         *user;
    const char   *controls;   /* example key bindings for the HUD */

    /* Optional: once after the window viewer was created (renderer
     * lighting, dragger settings ...).  Not called in headless mode. */
    void (*viewer_init)(demo_app_t *app, demo_viewer_t *viewer);
    /* Called once per frame before the physics step.  viewer is NULL in
     * headless mode (play the scripted key presses there). */
    void (*update)(demo_app_t *app, demo_viewer_t *viewer, int frame);
    /* Draw the scene (the grid is already drawn). */
    void (*draw)(demo_app_t *app, demo_viewer_t *viewer);
    /* After the Space scene reset restored the saved poses. */
    void (*on_reset)(demo_app_t *app);
    /* One line of stats (headless log, IVP_DEBUG inspector). */
    void (*stats)(demo_app_t *app, char *buf, size_t size);
    /* Headless self-check at the end of the run. */
    bool (*check)(demo_app_t *app);
};

/* `--headless [N]` / `-H [N]` or IVP_EXAMPLE_HEADLESS=N; 0 = windowed. */
static inline int demo_headless_frames(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--headless") == 0 || strcmp(argv[i], "-H") == 0) {
            if (i + 1 < argc && argv[i + 1][0] >= '0' && argv[i + 1][0] <= '9') {
                int n = atoi(argv[i + 1]);
                return n > 0 ? n : DEMO_HEADLESS_DEFAULT_FRAMES;
            }
            return DEMO_HEADLESS_DEFAULT_FRAMES;
        }
    }
    const char *e = getenv("IVP_EXAMPLE_HEADLESS");
    if (e && e[0] && e[0] != '0') {
        int n = atoi(e);
        return n > 0 ? n : DEMO_HEADLESS_DEFAULT_FRAMES;
    }
    return 0;
}

static inline int demo_run_headless(demo_app_t *app, int frames)
{
    const char *tag = app->cfg.tag;
    printf("[%s] headless run: %d frames of %.4f s (%.1f s simulated)\n",
           tag, frames, (double)DEMO_FRAME_TIME, (double)((float)frames * DEMO_FRAME_TIME));
    int every = 60;
    {
        const char *e = getenv("IVP_DEBUG_EVERY");
        if (e && atoi(e) > 0) every = atoi(e);
    }
    /* draw smoke test: run the draw callback every 30 frames against a
     * viewer without renderer (primitives are counted and validated) */
    demo_viewer_t *dv = (demo_viewer_t *)calloc(1, sizeof(*dv));
    if (!dv) return 1;
    demo_camera_setup(&dv->cam, &app->cfg);
    dv->title = app->cfg.title;
    dv->win_w = 1280;
    dv->win_h = 720;
    dv->frame_dt = DEMO_FRAME_TIME;
    ivp_example_dragger_init(&dv->dragger);
    ivp_example_focus_init(&dv->focus);
    int n_draws = 0;
    for (int f = 0; f < frames; f++) {
        if (app->update) app->update(app, NULL, f);
        ivp_environment_simulate_dtime(app->world.env, (ivp_real_t)DEMO_FRAME_TIME);
        demo_world_collect(&app->world);
        if (app->draw && f % 30 == 0) {
            dv->hud_y = 32.0f;
            app->draw(app, dv);
            n_draws++;
        }
        if (app->stats && ((f + 1) % every == 0 || f + 1 == frames)) {
            char buf[512];
            buf[0] = '\0';
            app->stats(app, buf, sizeof(buf));
            printf("[%s] t=%6.2f %s\n", tag, (double)((float)(f + 1) * DEMO_FRAME_TIME), buf);
        }
    }
    bool ok = demo_check(tag, demo_world_check_finite(&app->world, 1000.0f),
                         "all %d object positions finite and within 1000 m",
                         ivp_environment_get_object_count(app->world.env));
    if (app->draw) {
        ok = demo_check(tag, dv->n_bad_prims == 0 && dv->n_prims > 0,
                        "draw callback ran %d times without GPU (%ld primitives, %ld non-finite)",
                        n_draws, dv->n_prims, dv->n_bad_prims) && ok;
    }
    free(dv);
    if (app->check) ok = app->check(app) && ok;
    printf("[%s] self-check %s\n", tag, ok ? "PASSED" : "FAILED");
    return ok ? 0 : 1;
}

static inline int demo_run_viewer(demo_app_t *app)
{
    demo_viewer_t *v = (demo_viewer_t *)calloc(1, sizeof(*v));
    if (!v) return 1;
    if (!demo_viewer_init(v, &app->cfg, &app->world)) {
        free(v);
        return 1;
    }
    if (app->viewer_init) app->viewer_init(app, v);
    int frame = 0;
    while (demo_viewer_begin_frame(v, &app->world)) {
        if (v->reset_requested && app->on_reset) app->on_reset(app);
        if (app->update) app->update(app, v, frame);
        demo_viewer_step(v, &app->world);
        if (app->cfg.grid_extent >= 0.0f) {
            demo_draw_grid_ex(v, app->cfg.grid_extent > 0.0f ? app->cfg.grid_extent : 30.0f,
                              app->cfg.grid_spacing > 0.0f ? app->cfg.grid_spacing : 1.0f,
                              DEMO_COL_GRID);
        }
        if (app->draw) app->draw(app, v);
        if (app->stats && ivp_example_inspector_tick(&v->inspector)) {
            char buf[512];
            buf[0] = '\0';
            app->stats(app, buf, sizeof(buf));
            ivp_example_inspector_log(&v->inspector, "%s", buf);
        }
        demo_viewer_end_frame(v, app->controls);
        frame++;
    }
    demo_viewer_destroy(v, &app->world);
    free(v);
    return 0;
}

/* Save the reset snapshot and run the example (window or headless). */
static inline int demo_run(demo_app_t *app, int argc, char **argv)
{
    demo_world_save_state(&app->world);
    int frames = demo_headless_frames(argc, argv);
    if (frames > 0) return demo_run_headless(app, frames);
    return demo_run_viewer(app);
}

#endif /* IVP_EXAMPLE_DEMO_APP_H */
