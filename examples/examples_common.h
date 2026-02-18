/* examples_common.h — Shared utilities for IVP example programs */
#ifndef IVP_EXAMPLES_COMMON_H
#define IVP_EXAMPLES_COMMON_H

#include "render.h"
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_mat3.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_surman.h>
#include <math.h>

/* ── Common visualization thresholds ──────────────────────────────────── */
#define VEL_ARROW_THRESHOLD  0.3f   /* min speed to show velocity arrow  */
#define VEL_ARROW_SCALE      0.3f   /* arrow length = speed * scale      */
#define VEL_ARROW_HEAD       0.2f   /* arrowhead size                    */

/* Convert IVP quaternion to column-major 3x3 rotation for renderer */
static inline void quat_to_col_rot(const ivp_quatr_t *q, float rot[9]) {
    ivp_mat3r_t m;
    ivp_quat_to_mat3(q, &m);
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            rot[c * 3 + r] = (float)m.rows[r].k[c];
}

/* Compute speed magnitude from IVP velocity vector */
static inline float vec3r_speed(const ivp_vec3r_t *v) {
    return sqrtf((float)(v->k[0]*v->k[0] + v->k[1]*v->k[1] + v->k[2]*v->k[2]));
}

/* Copy IVP position to float[3] for renderer */
static inline void vec3r_to_float3(const ivp_vec3r_t *v, float out[3]) {
    out[0] = (float)v->k[0];
    out[1] = (float)v->k[1];
    out[2] = (float)v->k[2];
}

/* ── Inertia helpers ──────────────────────────────────────────────── */

/* Set box rotational inertia: I_x = (m/3)(hy^2 + hz^2), cyclic. */
static inline void set_box_inertia(ivp_object_template_t *t, ivp_real_t mass,
                                    ivp_real_t hx, ivp_real_t hy, ivp_real_t hz)
{
    ivp_real_t ix = (mass / IVP_REAL(3.0)) * (hy * hy + hz * hz);
    ivp_real_t iy = (mass / IVP_REAL(3.0)) * (hx * hx + hz * hz);
    ivp_real_t iz = (mass / IVP_REAL(3.0)) * (hx * hx + hy * hy);
    ivp_vec3r_set(&t->rot_inertia, ix, iy, iz);
}

/* Set sphere rotational inertia: I = (2/5) m r^2. */
static inline void set_sphere_inertia(ivp_object_template_t *t,
                                       ivp_real_t mass, ivp_real_t radius)
{
    ivp_real_t I = IVP_REAL(0.4) * mass * radius * radius;
    ivp_vec3r_set(&t->rot_inertia, I, I, I);
}

/* ── Ground creation helper ──────────────────────────────────────── */

/* Create a static ground box at the given Y position.
 * Caller must eventually free ground_surf and is responsible for sm_ground lifetime. */
static inline ivp_object_t *create_ground_object(
    ivp_environment_t *env,
    ivp_compact_surface_t *ground_surf,
    ivp_surman_polygon_t *sm_ground,
    ivp_real_t pos_y)
{
    ivp_surman_polygon_init(sm_ground, ground_surf);
    ivp_object_template_t gt;
    ivp_object_template_init(&gt);
    gt.type = IVP_OBJECT_POLYGON;
    gt.mass = 0.0f;
    gt.physical_unmoveable = true;
    gt.position = ivp_vec3r(IVP_REAL(0.0), pos_y, IVP_REAL(0.0));
    return ivp_environment_create_object(env, &gt, &sm_ground->base);
}

/* ── Camera defaults (for view reset) ─────────────────────────────── */

typedef struct ivp_camera_defaults {
    float target[3];
    float up[3];
    float fov_deg;
    float near_plane;
    float far_plane;
    float orbit_distance;
    float orbit_yaw;
    float orbit_pitch;
} ivp_camera_defaults_t;

static inline void ivp_camera_store_defaults(ivp_camera_defaults_t *defaults,
                                             const ivp_camera_t *cam)
{
    for (int i = 0; i < 3; i++) {
        defaults->target[i] = cam->target[i];
        defaults->up[i] = cam->up[i];
    }
    defaults->fov_deg        = cam->fov_deg;
    defaults->near_plane     = cam->near_plane;
    defaults->far_plane      = cam->far_plane;
    defaults->orbit_distance = cam->orbit_distance;
    defaults->orbit_yaw      = cam->orbit_yaw;
    defaults->orbit_pitch    = cam->orbit_pitch;
}

static inline void ivp_camera_restore_defaults(ivp_camera_t *cam,
                                               const ivp_camera_defaults_t *defaults)
{
    for (int i = 0; i < 3; i++) {
        cam->target[i] = defaults->target[i];
        cam->up[i] = defaults->up[i];
    }
    cam->fov_deg        = defaults->fov_deg;
    cam->near_plane     = defaults->near_plane;
    cam->far_plane      = defaults->far_plane;
    cam->orbit_distance = defaults->orbit_distance;
    cam->orbit_yaw      = defaults->orbit_yaw;
    cam->orbit_pitch    = defaults->orbit_pitch;
    ivp_camera_update_orbit(cam);
}

/* ── Scene snapshot (for scene reset) ─────────────────────────────── */

#define IVP_SCENE_SNAPSHOT_MAX 64

typedef struct ivp_scene_object_state {
    ivp_object_t *obj;
    ivp_vec3r_t   position;
    ivp_quatr_t   orientation;
    ivp_vec3r_t   speed;
    ivp_vec3f_t   rot_speed;
} ivp_scene_object_state_t;

typedef struct ivp_scene_snapshot {
    int count;
    ivp_scene_object_state_t objects[IVP_SCENE_SNAPSHOT_MAX];
} ivp_scene_snapshot_t;

/* Take a snapshot of all dynamic objects in the environment. */
static inline void ivp_scene_snapshot_take(ivp_scene_snapshot_t *snap,
                                           ivp_environment_t *env)
{
    snap->count = 0;
    int n = ivp_environment_get_object_count(env);
    for (int i = 0; i < n && snap->count < IVP_SCENE_SNAPSHOT_MAX; i++) {
        ivp_object_t *obj = ivp_environment_get_object(env, i);
        if (!obj || obj->core.physical_unmoveable) continue;
        ivp_scene_object_state_t *s = &snap->objects[snap->count];
        s->obj = obj;
        ivp_object_get_position(obj, &s->position);
        ivp_object_get_orientation(obj, &s->orientation);
        ivp_object_get_speed(obj, &s->speed);
        s->rot_speed = obj->core.rot_speed;
        snap->count++;
    }
}

/* Restore all snapshotted objects to their initial state. */
static inline void ivp_scene_snapshot_restore(const ivp_scene_snapshot_t *snap)
{
    for (int i = 0; i < snap->count; i++) {
        const ivp_scene_object_state_t *s = &snap->objects[i];
        ivp_core_set_pose(&s->obj->core, &s->position, &s->orientation);
        s->obj->core.speed = s->speed;
        s->obj->core.rot_speed = s->rot_speed;
        /* Clear accumulated forces and speed changes */
        ivp_vec3r_set_zero(&s->obj->core.speed_change);
        ivp_vec3r_set_zero(&s->obj->core.rot_speed_change);
        /* Reset movement state to MOVING so the simulation picks them up */
        s->obj->core.movement_state = IVP_MT_MOVING;
    }
}

/* ── Fixed-timestep simulation loop state ─────────────────────────── */

typedef struct {
    double t_prev;
    float  sim_accum;
    float  sim_dt;
    int    max_substeps;
    /* FPS tracking */
    int    frame_count;
    float  fps_timer;
    float  fps_display;
} sim_loop_t;

static inline void sim_loop_init(sim_loop_t *sl, const ivp_renderer_t *r) {
    sl->t_prev       = ivp_renderer_get_time(r);
    sl->sim_accum    = 0.0f;
    sl->sim_dt       = 1.0f / 300.0f;
    sl->max_substeps = 24;
    sl->frame_count  = 0;
    sl->fps_timer    = 0.0f;
    sl->fps_display  = 0.0f;
}

/* Returns frame_dt. Call at top of each frame. */
static inline float sim_loop_begin(sim_loop_t *sl, const ivp_renderer_t *r) {
    double now = ivp_renderer_get_time(r);
    float frame_dt = (float)(now - sl->t_prev);
    sl->t_prev = now;
    if (frame_dt > 0.05f) frame_dt = 0.05f;

    sl->frame_count++;
    sl->fps_timer += frame_dt;
    if (sl->fps_timer >= 0.5f) {
        sl->fps_display = (float)sl->frame_count / sl->fps_timer;
        sl->frame_count = 0;
        sl->fps_timer   = 0.0f;
    }
    return frame_dt;
}

/* Step physics with fixed timestep accumulator. Returns substep count. */
static inline int sim_loop_step(sim_loop_t *sl, ivp_environment_t *env,
                                float frame_dt) {
    sl->sim_accum += frame_dt;
    int substeps = 0;
    while (sl->sim_accum >= sl->sim_dt && substeps < sl->max_substeps) {
        ivp_environment_simulate_dtime(env, (ivp_real_t)sl->sim_dt);
        sl->sim_accum -= sl->sim_dt;
        substeps++;
    }
    if (substeps == sl->max_substeps)
        sl->sim_accum = 0.0f;
    return substeps;
}

#endif /* IVP_EXAMPLES_COMMON_H */
