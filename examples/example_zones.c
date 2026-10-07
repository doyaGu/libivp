/* example_zones.c -- Phantom volumes that hand their contents to other controllers
 *
 * A phantom is a static box that does not collide but keeps a set of the cores
 * that intrude it. That set can drive anything that follows an active set of
 * cores, so each of the three zones uses it differently:
 *
 *   updraft   a set-driven force field (ivp_forcefield_attach_set): inside the
 *             box everything is accelerated upwards, outside nothing is
 *   limiter   the generic attacher (ivp_attacher_to_cores) with an own
 *             attachment: every core that enters gets a speed-limit controller,
 *             every core that leaves loses it again
 *   pool      a buoyancy attacher: the box is a tank, the bodies in it float
 *
 * The bodies are grouped in clusters, one per zone. A phantom listener counts
 * the cores entering and leaving each volume.
 *
 *   K   drop the bodies again from above
 *
 * Demonstrates: ivp_object_convert_to_phantom (manage_intruding_cores), phantom
 *               listeners, ivp_forcefield_attach_set, ivp_attacher_to_cores_*,
 *               ivp_buoyancy_attacher_create, ivp_cluster_* .
 */
#include "demo_app.h"

#include <ivp/controllers/ivp_attacher_to_cores.h>
#include <ivp/controllers/ivp_controller_buoyancy.h>
#include <ivp/controllers/ivp_forcefield.h>
#include <ivp/dynamics/ivp_cluster.h>
#include <ivp/dynamics/ivp_phantom.h>

#define N_ZONES   3
#define N_PER     2
#define SPEED_CAP 3.0
#define FLOOR_Y   (-0.3)               /* bottom of the volumes that reach the floor */

enum { Z_UPDRAFT, Z_LIMITER, Z_POOL };

typedef struct zone zone_t;

typedef struct attachment {
    zone_t                  *zone;
    ivp_core_t              *core;
    ivp_controller_handle_t  handle;
} attachment_t;

struct zone {
    const char                *name;
    double                     x, top, bottom, half;  /* volume: x +- half, from top down to bottom */
    float                      col[3];
    ivp_object_t              *volume;
    ivp_controller_phantom_t  *phantom;
    ivp_listener_phantom_t     listener;
    ivp_cluster_t             *cluster;
    ivp_object_t              *body[N_PER];
    int                        entered, left;
    float                      peak_up, in_speed_max;  /* measurements */
    struct zones_demo         *demo;
};

typedef struct zones_demo {
    ivp_environment_t       *env;
    ivp_object_t            *ground;
    zone_t                   zone[N_ZONES];
    ivp_forcefield_state_t   updraft;
    ivp_attacher_to_cores_t  limiter;
    ivp_liquid_surface_simple_t water;
    ivp_buoyancy_state_t     templ;
    ivp_buoyancy_attacher_t *pool;
    long                     limiter_created, limiter_deleted;
} zones_demo_t;

/* ── the limiter: ATTACH_T of the generic attacher ───────────────────────── */

static void limit_speed(const ivp_event_sim_t *es, ivp_object_t *obj, void *ctx)
{
    (void)es;
    (void)ctx;
    ivp_core_t *core = obj->physical_core;
    double v2 = 0.0;
    for (int k = 0; k < 3; k++) v2 += (double)core->speed.k[k] * (double)core->speed.k[k];
    const double v = sqrt(v2);
    if (v <= SPEED_CAP) return;
    const double change = SPEED_CAP / v - 1.0;                    /* asynchronous: ends at the cap */
    for (int k = 0; k < 3; k++) core->speed_change.k[k] += (ivp_real_t)(change * (double)core->speed.k[k]);
}

static void *limiter_create(ivp_attacher_to_cores_t *at, ivp_core_t *core)
{
    zones_demo_t *s = (zones_demo_t *)at->user_data;
    attachment_t *a = (attachment_t *)calloc(1, sizeof(*a));
    if (!a) return NULL;
    a->core = core;
    a->handle = ivp_environment_create_core_controller(s->env, IVP_CP_ACTUATOR, limit_speed, a);
    ivp_environment_add_controller_to_core(s->env, a->handle, ivp_attacher_core_object(core));
    s->limiter_created++;
    return a;
}

static void limiter_delete(ivp_attacher_to_cores_t *at, ivp_core_t *core, void *attachment)
{
    zones_demo_t *s = (zones_demo_t *)at->user_data;
    attachment_t *a = (attachment_t *)attachment;
    ivp_object_t *obj = ivp_attacher_core_object(core);
    if (obj) ivp_environment_remove_controller_from_core(s->env, a->handle, obj);
    ivp_environment_remove_controller(s->env, a->handle);
    s->limiter_deleted++;
    free(a);
}

static const ivp_attacher_to_cores_class_t limiter_class = { limiter_create, limiter_delete, NULL };

/* ── phantom listener ────────────────────────────────────────────────────── */

static void core_entered(ivp_listener_phantom_t *self, ivp_controller_phantom_t *c, ivp_core_t *core)
{
    (void)c;
    (void)core;
    ((zone_t *)self->user_data)->entered++;
}

static void core_left(ivp_listener_phantom_t *self, ivp_controller_phantom_t *c, ivp_core_t *core)
{
    (void)c;
    (void)core;
    ((zone_t *)self->user_data)->left++;
}

/* ── scene ───────────────────────────────────────────────────────────────── */

static const struct {
    const char *name;
    double      x, top, bottom, half;
    float       col[3];
} zdesc[N_ZONES] = {
    { "updraft", -10.0, -7.5, FLOOR_Y, 2.0, { 0.45f, 0.75f, 0.95f } },
    { "limiter", 0.0, -7.5, -3.0, 2.0, { 0.95f, 0.65f, 0.30f } },
    { "pool", 10.0, -5.0, FLOOR_Y, 2.0, { 0.30f, 0.70f, 0.75f } },
};

static void drop_bodies(zones_demo_t *s)
{
    static const double drop_y[N_ZONES] = { -4.0, -12.0, -8.5 };
    for (int z = 0; z < N_ZONES; z++)
        for (int i = 0; i < N_PER; i++) {
            ivp_object_t *o = s->zone[z].body[i];
            ivp_environment_ensure_core_in_simulation(s->env, o);
            ivp_quatr_t q = ivp_quatr_identity();
            ivp_vec3r_t p = ivp_vec3r((ivp_real_t)(zdesc[z].x + (i ? 0.9 : -0.9)), (ivp_real_t)drop_y[z],
                                      (ivp_real_t)(i ? 0.5 : -0.5));
            ivp_object_beam_to_new_position(o, &q, &p, false);
            o->physical_core->speed = ivp_vec3r_zero();
            o->physical_core->rot_speed = ivp_vec3f(0.0f, 0.0f, 0.0f);
        }
}

static void zones_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    zones_demo_t *s = (zones_demo_t *)app->user;
    (void)frame;
    if (v && demo_key_just_pressed(v, SDL_SCANCODE_K)) drop_bodies(s);
    for (int z = 0; z < N_ZONES; z++) {
        zone_t *zn = &s->zone[z];
        for (int i = 0; i < N_PER; i++) {
            float c[3];
            demo_object_center(zn->body[i], c);
            const float up = -c[1];
            if (up > zn->peak_up) zn->peak_up = up;
            const bool inside = fabsf(c[0] - (float)zn->x) < (float)zn->half && c[1] > (float)zn->top &&
                                c[1] < (float)zn->bottom;
            if (z == Z_LIMITER && inside && frame > 3) {
                const float sp = demo_object_speed(zn->body[i]);
                if (sp > zn->in_speed_max) zn->in_speed_max = sp;
            }
        }
    }
}

static void zones_reset(demo_app_t *app)
{
    zones_demo_t *s = (zones_demo_t *)app->user;
    for (int z = 0; z < N_ZONES; z++) {
        s->zone[z].peak_up = -1e9f;
        s->zone[z].in_speed_max = 0.0f;
    }
}

static void zones_draw(demo_app_t *app, demo_viewer_t *v)
{
    zones_demo_t *s = (zones_demo_t *)app->user;
    const float c_ground[3] = { 0.50f, 0.53f, 0.57f };
    const float c_body[N_PER][3] = { { 0.95f, 0.95f, 0.95f }, { 0.90f, 0.55f, 0.25f } };
    demo_draw_object(v, s->ground, c_ground);
    for (int z = 0; z < N_ZONES; z++) {
        zone_t *zn = &s->zone[z];
        const float half[3] = { (float)zn->half, (float)(0.5 * (zn->bottom - zn->top)), (float)zn->half },
                    pos[3] = { (float)zn->x, (float)(0.5 * (zn->top + zn->bottom)), 0.0f };
        demo_box(v, pos, half, NULL, zn->col);
        for (int i = 0; i < N_PER; i++) demo_draw_object(v, zn->body[i], c_body[i]);
    }
    /* the water of the pool: a translucent block from the surface (y = -2) to the floor of the tank */
    const zone_t *p = &s->zone[Z_POOL];
    if (v->solid) {
        const float water[3] = { 0.15f, 0.45f, 0.85f }, centre[3] = { (float)p->x, -1.15f, 0.0f },
                    half[3] = { (float)p->half, 0.85f, (float)p->half };
        demo_set_alpha(v, 0.38f);
        demo_solid_box(v, centre, half, NULL, water);
        demo_set_alpha(v, 1.0f);
    } else {
        const float w0[3] = { (float)(p->x - p->half), -2.0f, -2.0f }, w1[3] = { (float)(p->x + p->half), -2.0f, -2.0f },
                    w2[3] = { (float)(p->x + p->half), -2.0f, 2.0f }, w3[3] = { (float)(p->x - p->half), -2.0f, 2.0f };
        demo_line(v, w0, w1, DEMO_COL_WATER);
        demo_line(v, w1, w2, DEMO_COL_WATER);
        demo_line(v, w2, w3, DEMO_COL_WATER);
        demo_line(v, w3, w0, DEMO_COL_WATER);
    }

    for (int z = 0; z < N_ZONES; z++) {
        const zone_t *zn = &s->zone[z];
        demo_hud(v, zn->col, "%-8s in volume %d, enter events %d, leave events %d, cluster of %d", zn->name,
                 ivp_set_count(&ivp_controller_phantom_get_intruding_cores(zn->phantom)->set), zn->entered, zn->left,
                 ivp_cluster_get_child_count(zn->cluster));
    }
    demo_hud(v, DEMO_COL_HELP, "limiter attachments %d (made %ld, deleted %ld)", ivp_attacher_to_cores_count(&s->limiter),
             s->limiter_created, s->limiter_deleted);
}

static void zones_stats(demo_app_t *app, char *buf, size_t size)
{
    zones_demo_t *s = (zones_demo_t *)app->user;
    snprintf(buf, size, "entered %d %d %d, left %d %d %d", s->zone[0].entered, s->zone[1].entered, s->zone[2].entered,
             s->zone[0].left, s->zone[1].left, s->zone[2].left);
}

static int real_objects_in_tree(zones_demo_t *s)
{
    int n = 0;
    ivp_cluster_t *root = ivp_environment_get_root_cluster(s->env);
    for (ivp_object_t *o = ivp_cluster_get_next_real_object_in_tree(root, NULL); o;
         o = ivp_cluster_get_next_real_object_in_tree(root, o))
        n++;
    return n;
}

static bool zones_check(demo_app_t *app)
{
    zones_demo_t *s = (zones_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    for (int z = 0; z < N_ZONES; z++)
        ok &= demo_check(tag, s->zone[z].entered >= N_PER, "%s: %d cores entered the volume, %d left", s->zone[z].name,
                         s->zone[z].entered, s->zone[z].left);
    /* dropped from y = -4, the updraft throws them far above the volume */
    ok &= demo_check(tag, s->zone[Z_UPDRAFT].peak_up > 8.0f, "the updraft lifts the bodies (to %.1f m, dropped at 4 m)",
                     (double)s->zone[Z_UPDRAFT].peak_up);
    ok &= demo_check(tag, s->zone[Z_LIMITER].in_speed_max < SPEED_CAP + 1.0 && s->zone[Z_LIMITER].in_speed_max > 1.0f,
                     "the limiter holds the speed inside (%.2f m/s, cap %.1f)", (double)s->zone[Z_LIMITER].in_speed_max,
                     SPEED_CAP);
    ok &= demo_check(tag, s->limiter_created >= N_PER && s->limiter_deleted >= N_PER &&
                              s->limiter_created - s->limiter_deleted == ivp_attacher_to_cores_count(&s->limiter),
                     "the generic attacher makes an attachment when a core enters and deletes it when it leaves (%ld made, %ld deleted, %d alive)",
                     s->limiter_created, s->limiter_deleted, ivp_attacher_to_cores_count(&s->limiter));
    for (int i = 0; i < N_PER; i++) {
        float c[3];
        demo_object_center(s->zone[Z_POOL].body[i], c);
        ok &= demo_check(tag, c[1] < -1.0f && c[1] > -3.2f, "pool body %d floats at the surface (y %.2f, surface -2)", i,
                         (double)c[1]);
    }
    ok &= demo_check(tag, real_objects_in_tree(s) >= N_ZONES * N_PER + N_ZONES + 1,
                     "the cluster tree holds %d real objects", real_objects_in_tree(s));
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Zones -- Phantoms, Sets and Attachers";
    app.cfg.tag         = "zones";
    app.cfg.friction    = 0.6;
    app.cfg.elasticity  = 0.3;
    app.cfg.orbit_dist  = 30.0f;
    app.cfg.orbit_yaw   = 180.0f;
    app.cfg.orbit_pitch = -16.0f;
    app.cfg.target[1]   = -4.0f;
    app.cfg.grid_extent = 16.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    zones_demo_t *s = (zones_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->env = w->env;

    s->ground = demo_create_box(w, NULL, 16.0, 0.5, 4.0, 0.0, NULL, 0.0, 0.5, 0.0);

    for (int z = 0; z < N_ZONES; z++) {
        zone_t *zn = &s->zone[z];
        zn->name = zdesc[z].name;
        zn->x = zdesc[z].x;
        zn->top = zdesc[z].top;
        zn->bottom = zdesc[z].bottom;
        zn->half = zdesc[z].half;
        memcpy(zn->col, zdesc[z].col, sizeof(zn->col));
        zn->demo = s;
        zn->peak_up = -1e9f;

        /* the volume: a static box turned into a phantom that tracks the cores in it */
        zn->volume = demo_create_box(w, NULL, zn->half, 0.5 * (zn->bottom - zn->top), zn->half, 0.0, NULL, zn->x,
                                     0.5 * (zn->top + zn->bottom), 0.0);
        ivp_template_phantom_t pt;
        ivp_template_phantom_init(&pt);
        pt.manage_intruding_cores = true;
        zn->phantom = ivp_object_convert_to_phantom(zn->volume, &pt);
        if (!zn->phantom) return 1;
        zn->listener.core_entered_volume = core_entered;
        zn->listener.core_left_volume = core_left;
        zn->listener.user_data = zn;
        ivp_controller_phantom_add_listener(zn->phantom, &zn->listener);

        zn->cluster = ivp_environment_create_cluster(w->env, NULL, NULL);
        for (int i = 0; i < N_PER; i++) {
            const double bx = zn->x + (i ? 0.9 : -0.9), bz = i ? 0.5 : -0.5;
            ivp_object_t *o;
            if (z == Z_POOL) {                               /* light: cork box and a wooden ball */
                o = i ? demo_create_ball(w, NULL, 0.5, 0.35, NULL, bx, -8.5, bz)
                      : demo_create_box(w, NULL, 0.5, 0.5, 0.5, 0.3, NULL, bx, -8.5, bz);
            } else {
                o = i ? demo_create_ball(w, NULL, 0.4, 1.0, NULL, bx, -4.0, bz)
                      : demo_create_box(w, NULL, 0.4, 0.4, 0.4, 1.0, NULL, bx, -4.0, bz);
            }
            if (!o) return 1;
            zn->body[i] = o;
            ivp_cluster_add_child(zn->cluster, o);
        }
    }
    drop_bodies(s);

    /* updraft: constant acceleration upwards, set-driven by the phantom's core set */
    const ivp_vec3r_t lift = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(-20.0), IVP_REAL(0.0));
    ivp_forcefield_state_init(&s->updraft, IVP_FORCEFIELD_CONSTANT, &lift);
    if (!ivp_forcefield_attach_set(&s->updraft, w->env, ivp_controller_phantom_get_intruding_cores(s->zone[Z_UPDRAFT].phantom),
                                   false))
        return 1;

    /* limiter: an own attachment for every core in the set */
    if (!ivp_attacher_to_cores_init(&s->limiter, w->env, ivp_controller_phantom_get_intruding_cores(s->zone[Z_LIMITER].phantom),
                                    &limiter_class, s))
        return 1;

    /* pool: buoyancy for the cores in the tank, the surface at y = -2 */
    ivp_buoyancy_init(&s->templ, IVP_REAL(-2.0), IVP_REAL(1.0));
    s->templ.linear_drag = IVP_REAL(3.0);
    s->templ.angular_drag = IVP_REAL(2.0);
    s->pool = ivp_buoyancy_attacher_create(w->env, &s->templ, ivp_controller_phantom_get_intruding_cores(s->zone[Z_POOL].phantom));
    if (!s->pool) return 1;

    app.controls    = "K: drop the bodies again";
    app.update      = zones_update;
    app.on_reset    = zones_reset;
    app.draw        = zones_draw;
    app.stats       = zones_stats;
    app.check       = zones_check;

    int rc = demo_run(&app, argc, argv);
    ivp_buoyancy_attacher_destroy(s->pool);
    ivp_attacher_to_cores_destroy(&s->limiter);
    ivp_forcefield_destroy(&s->updraft);
    ivp_buoyancy_state_destroy(&s->templ);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
