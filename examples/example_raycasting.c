/* example_raycasting.c -- Ray solver demonstration
 *
 * Port of ivp_examples/examples/ive_example_raycasting.cxx.
 *
 * A ground plate with four static cubes and four dynamic objects (balls
 * and a box) dropped onto it.  A ray is cast from the camera eye through
 * the view target (the centre of the screen) into the scene:
 *   T  IVP_Ray_Solver_Min      -- nearest hit only (reference key R; R is
 *                                 a camera key in the libivp examples)
 *   H  IVP_Ray_Solver_Min_Hash -- every hit along the ray, sorted by
 *                                 distance
 * The ray, the hit points and the (nearest) hit object are highlighted.
 *
 * Demonstrates: ivp_ray_solver_min_t / ivp_ray_solver_min_hash_t,
 *               ivp_ray_solver_check_all_objects (reference
 *               check_ray_against_all_objects_in_sim), ray hit normals.
 */
#include "demo_app.h"

#include <ivp/collision/ivp_ray_solver.h>

#define NUM_SCENE_OBJECTS 8
#define MAX_HASH_HITS 32
#define RAY_LENGTH 100.0f

typedef struct hit_point {
    float         pos[3];
    float         normal_ws[3];
    float         dist;
    ivp_object_t *obj;
} hit_point_t;

typedef struct ray_vis {
    bool        active;
    bool        hash_mode;      /* true = Min_Hash, false = Min */
    bool        did_hit;
    float       start[3];
    float       end[3];
    float       hit_dist;
    hit_point_t hits[MAX_HASH_HITS];
    int         num_hits;
} ray_vis_t;

typedef struct raycast {
    ivp_object_t  *ground;
    ivp_object_t  *scene[NUM_SCENE_OBJECTS];
    ivp_object_t  *last_hit_obj;
    ray_vis_t      ray;
    ivp_camera_t   headless_cam;   /* camera used for the scripted casts */
    int            n_min_casts, n_min_hits;
    int            n_hash_casts, max_hash_hits;
    bool           view_min_hit;   /* headless: Min ray of the default view */
    int            side_hits;      /* headless: hits of the side ray */
} raycast_t;

/* Object space normal of a hit -> world space. */
static void hit_normal_ws(const ivp_ray_hit_t *hit, float out[3])
{
    float pos[3], rot[9];
    if (!hit->hit_object) {
        vec3r_to_float3(&hit->hit_normal, out);
        return;
    }
    demo_object_pose(hit->hit_object, pos, rot);
    for (int i = 0; i < 3; i++) {
        out[i] = rot[0 * 3 + i] * (float)hit->hit_normal.k[0]
               + rot[1 * 3 + i] * (float)hit->hit_normal.k[1]
               + rot[2 * 3 + i] * (float)hit->hit_normal.k[2];
    }
}

/* Cast a ray from `eye` through `target` (the camera eye and the view
 * target in the interactive mode). */
static void cast_ray(demo_app_t *app, const float eye[3], const float target[3], bool hash_mode)
{
    raycast_t *s = (raycast_t *)app->user;
    ray_vis_t *rv = &s->ray;

    float dir[3] = { target[0] - eye[0], target[1] - eye[1], target[2] - eye[2] };
    float len = sqrtf(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    if (len > 0.001f) { dir[0] /= len; dir[1] /= len; dir[2] /= len; }

    ivp_ray_solver_template_t templ;
    templ.ray_start_point = ivp_vec3r(eye[0], eye[1], eye[2]);
    templ.ray_normized_direction = ivp_vec3r(dir[0], dir[1], dir[2]);
    templ.ray_length = RAY_LENGTH;
    templ.ray_flags = IVP_RAY_SOLVER_ALL;

    for (int i = 0; i < 3; i++) {
        rv->start[i] = eye[i];
        rv->end[i] = eye[i] + dir[i] * RAY_LENGTH;
    }
    rv->active = true;
    rv->hash_mode = hash_mode;
    rv->num_hits = 0;
    rv->did_hit = false;
    rv->hit_dist = 0.0f;
    s->last_hit_obj = NULL;

    if (!hash_mode) {
        /* IVP_Ray_Solver_Min: nearest hit only */
        ivp_ray_solver_min_t solver;
        ivp_ray_solver_min_init(&solver, &templ);
        ivp_ray_solver_check_all_objects(&solver.base, app->world.env);
        const ivp_ray_hit_t *hit = ivp_ray_solver_min_get_ray_hit(&solver);
        s->n_min_casts++;
        if (hit) {
            hit_point_t *hp = &rv->hits[rv->num_hits++];
            hp->dist = (float)hit->hit_distance;
            hp->obj = hit->hit_object;
            for (int i = 0; i < 3; i++) hp->pos[i] = rv->start[i] + dir[i] * hp->dist;
            hit_normal_ws(hit, hp->normal_ws);
            rv->did_hit = true;
            rv->hit_dist = hp->dist;
            memcpy(rv->end, hp->pos, sizeof(rv->end));
            s->last_hit_obj = hit->hit_object;
            s->n_min_hits++;
        }
    } else {
        /* IVP_Ray_Solver_Min_Hash: all hits, sorted by distance */
        ivp_ray_solver_min_hash_t *solver =
            (ivp_ray_solver_min_hash_t *)malloc(sizeof(ivp_ray_solver_min_hash_t));
        if (!solver) return;
        ivp_ray_solver_min_hash_init(solver, &templ);
        ivp_ray_solver_check_all_objects(&solver->base, app->world.env);
        const ivp_ray_hit_t *sorted[MAX_HASH_HITS];
        int n = ivp_ray_solver_min_hash_get_sorted(solver, sorted, MAX_HASH_HITS);
        s->n_hash_casts++;
        for (int k = 0; k < n; k++) {
            hit_point_t *hp = &rv->hits[rv->num_hits++];
            hp->dist = (float)sorted[k]->hit_distance;
            hp->obj = sorted[k]->hit_object;
            for (int i = 0; i < 3; i++) hp->pos[i] = rv->start[i] + dir[i] * hp->dist;
            hit_normal_ws(sorted[k], hp->normal_ws);
            if (!s->last_hit_obj) s->last_hit_obj = hp->obj;   /* nearest */
        }
        if (n > 0) {
            rv->did_hit = true;
            rv->hit_dist = rv->hits[n - 1].dist;               /* farthest */
            memcpy(rv->end, rv->hits[n - 1].pos, sizeof(rv->end));
        }
        if (n > s->max_hash_hits) s->max_hash_hits = n;
        free(solver);
    }
}

static void print_ray(const demo_app_t *app, const char *what)
{
    const raycast_t *s = (const raycast_t *)app->user;
    printf("[%s] %s: %d hit(s)\n", app->cfg.tag, what, s->ray.num_hits);
    for (int i = 0; i < s->ray.num_hits; i++) {
        const hit_point_t *hp = &s->ray.hits[i];
        printf("[%s]   [%d] dist=%.3f pos=(%.2f, %.2f, %.2f) normal=(%.2f, %.2f, %.2f)\n",
               app->cfg.tag, i, (double)hp->dist,
               (double)hp->pos[0], (double)hp->pos[1], (double)hp->pos[2],
               (double)hp->normal_ws[0], (double)hp->normal_ws[1], (double)hp->normal_ws[2]);
    }
}

static void raycast_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    raycast_t *s = (raycast_t *)app->user;
    if (v) {
        bool do_min  = demo_key_just_pressed(v, SDL_SCANCODE_T);
        bool do_hash = demo_key_just_pressed(v, SDL_SCANCODE_H);
        if (do_min || do_hash) cast_ray(app, v->cam.eye, v->cam.target, do_hash);
        return;
    }
    /* headless script, once the dynamic objects have settled: both ray
     * types from the default camera, then a Min_Hash ray along the X axis
     * through static cube 0, the 2 kg box and static cube 1 */
    if (frame == 240) {
        cast_ray(app, s->headless_cam.eye, s->headless_cam.target, false);
        s->view_min_hit = s->ray.did_hit;
        print_ray(app, "Min ray (default view)");
    }
    if (frame == 241) {
        cast_ray(app, s->headless_cam.eye, s->headless_cam.target, true);
        print_ray(app, "Min_Hash ray (default view)");
    }
    if (frame == 242) {
        const float eye[3] = { -10.0f, -0.8f, 0.0f }, target[3] = { 0.0f, -0.8f, 0.0f };
        cast_ray(app, eye, target, true);
        s->side_hits = s->ray.num_hits;
        print_ray(app, "Min_Hash ray along +X at y=-0.8");
    }
}

static void raycast_draw(demo_app_t *app, demo_viewer_t *v)
{
    raycast_t *s = (raycast_t *)app->user;
    demo_draw_object(v, s->ground, DEMO_COL_STATIC);
    for (int i = 0; i < NUM_SCENE_OBJECTS; i++) {
        const demo_body_t *b = demo_body_of(s->scene[i]);
        const float *col = (b && b->shape == DEMO_SHAPE_BALL) ? DEMO_COL_OBJECT_B : DEMO_COL_OBJECT_A;
        if (s->last_hit_obj == s->scene[i]) col = DEMO_COL_HIGHLIGHT;
        demo_draw_object(v, s->scene[i], col);
    }

    /* screen centre marker: the ray goes through the view target */
    demo_cross(v, v->cam.target, 0.15f, DEMO_COL_HELP);

    const ray_vis_t *rv = &s->ray;
    if (rv->active) {
        demo_line(v, rv->start, rv->end,
                             rv->did_hit ? DEMO_COL_VELOCITY : DEMO_COL_WARNING);
        for (int i = 0; i < rv->num_hits; i++) {
            /* nearest = velocity colour, farthest = highlight */
            float t = (rv->num_hits > 1) ? (float)i / (float)(rv->num_hits - 1) : 0.0f;
            const float *c0 = DEMO_COL_VELOCITY;
            const float *c1 = DEMO_COL_HIGHLIGHT;
            float col[3] = { c0[0] * (1.0f - t) + c1[0] * t,
                             c0[1] * (1.0f - t) + c1[1] * t,
                             c0[2] * (1.0f - t) + c1[2] * t };
            demo_sphere(v, rv->hits[i].pos, rv->hash_mode ? 0.12f : 0.1f, col);
            float tip[3] = { rv->hits[i].pos[0] + rv->hits[i].normal_ws[0] * 0.5f,
                             rv->hits[i].pos[1] + rv->hits[i].normal_ws[1] * 0.5f,
                             rv->hits[i].pos[2] + rv->hits[i].normal_ws[2] * 0.5f };
            demo_arrow(v, rv->hits[i].pos, tip, col, 0.08f);
        }
        demo_hud(v, DEMO_COL_INFO, "Mode: %s", rv->hash_mode ? "Min_Hash (all hits)" : "Min (nearest)");
        if (!rv->did_hit) {
            demo_hud(v, DEMO_COL_WARNING, "MISS");
        } else if (!rv->hash_mode) {
            demo_hud(v, DEMO_COL_INFO, "HIT  dist=%.2f", (double)rv->hit_dist);
        } else {
            demo_hud(v, DEMO_COL_INFO, "HITS: %d", rv->num_hits);
            for (int i = 0; i < rv->num_hits && i < 12; i++)
                demo_hud(v, DEMO_COL_INFO, "  [%d] dist=%.2f", i, (double)rv->hits[i].dist);
        }
    } else {
        demo_hud(v, DEMO_COL_INFO, "No ray cast yet");
    }
}

static void raycast_stats(demo_app_t *app, char *buf, size_t size)
{
    raycast_t *s = (raycast_t *)app->user;
    float max_speed = 0.0f;
    for (int i = 4; i < NUM_SCENE_OBJECTS; i++) {
        float sp = demo_object_speed(s->scene[i]);
        if (sp > max_speed) max_speed = sp;
    }
    snprintf(buf, size, "dyn_max_speed=%.3f min_casts=%d min_hits=%d hash_casts=%d max_hash_hits=%d",
             (double)max_speed, s->n_min_casts, s->n_min_hits, s->n_hash_casts, s->max_hash_hits);
}

static bool raycast_check(demo_app_t *app)
{
    raycast_t *s = (raycast_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, s->view_min_hit, "Min ray of the default view hit something");
    ok &= demo_check(tag, s->side_hits >= 3,
                     "side Min_Hash ray hit cube 0, the box and cube 1 (%d hits)", s->side_hits);
    bool sorted = true;
    for (int i = 1; i < s->ray.num_hits; i++)
        if (s->ray.hits[i].dist < s->ray.hits[i - 1].dist) sorted = false;
    ok &= demo_check(tag, sorted, "Min_Hash hits are sorted by distance");
    /* dynamic objects came to rest on the ground or the static cubes */
    bool rest = true;
    for (int i = 4; i < NUM_SCENE_OBJECTS; i++)
        if (s->scene[i]->physical_core->pos_world_f_core_last_psi.k[1] > 0.0) rest = false;
    ok &= demo_check(tag, rest, "dynamic objects stay above the ground");
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Example: Raycasting";
    app.cfg.tag         = "raycasting";
    app.cfg.orbit_dist  = 18.0f;
    app.cfg.orbit_pitch = -20.0f;
    app.cfg.target[1]   = -2.0f;
    app.cfg.friction    = 0.5;
    app.cfg.elasticity  = 0.4;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    raycast_t *s = (raycast_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    s->ground = demo_create_box(w, NULL, 15.0, 0.5, 15.0, 0.0, NULL, 0.0, 0.5, 0.0);

    const double hs = 0.4;
    /* static cubes */
    s->scene[0] = demo_create_box(w, NULL, hs, hs, hs, 0.0, NULL, -3.0, -1.0, 0.0);
    s->scene[1] = demo_create_box(w, NULL, hs, hs, hs, 0.0, NULL,  3.0, -1.0, 0.0);
    s->scene[2] = demo_create_box(w, NULL, hs, hs, hs, 0.0, NULL,  0.0, -1.0, 3.0);
    s->scene[3] = demo_create_box(w, NULL, hs, hs, hs, 0.0, NULL,  0.0, -1.0, -3.0);
    /* dynamic objects */
    s->scene[4] = demo_create_ball(w, NULL, 0.35, 1.0, NULL, -1.5, -4.0, 1.5);
    s->scene[5] = demo_create_ball(w, NULL, 0.35, 1.0, NULL, 1.5, -4.0, -1.5);
    s->scene[6] = demo_create_box(w, NULL, 0.5, 0.5, 0.5, 2.0, NULL, 0.0, -5.0, 0.0);
    s->scene[7] = demo_create_ball(w, NULL, 0.3, 0.8, NULL, 2.0, -3.5, 2.0);

    demo_camera_setup(&s->headless_cam, &app.cfg);

    app.controls = "T: cast ray (Min -- nearest hit)   H: cast ray (Min_Hash -- all hits)";
    app.update   = raycast_update;
    app.draw     = raycast_draw;
    app.stats    = raycast_stats;
    app.check    = raycast_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
