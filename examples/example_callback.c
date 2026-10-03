/* example_callback.c -- Object and collision listeners
 *
 * Port of ivp_examples/quickstart/ive_quickstart_callback.cxx
 * (IVP manual section 5.12, Interfacing with callbacks).
 *
 * Four cubes are dropped onto the ground.  Each cube has
 *   - an object listener (IVP_Listener_Object): frozen / revived events,
 *   - a collision listener (IVP_Listener_Collision, post collision).
 * Colours: green = moving, blue = frozen (asleep), red flash = collision.
 * Cube 0 is the "main" cube of the manual: as soon as it comes to a
 * standstill (frozen event) it is pushed back up into the air, forever
 * (when it lands on a sleeping neighbour it wakes that one up, too).
 *
 * Note: the reference quickstart overrides a non-existent
 * IVP_Listener_Collision::event_collision, so its red collision flash never
 * shows; this port uses the post-collision callback it intended (the
 * freeze / revive / collision counts match the legacy engine with
 * event_post_collision).
 *
 * Demonstrates: ivp_object_add_listener_object (event_object_frozen /
 *               event_object_revived), ivp_object_add_collision_listener
 *               (post_collision_fn), asynchronous pushes from callbacks.
 */
#include "demo_app.h"

#include <ivp/events/ivp_listener_collision.h>
#include <ivp/events/ivp_listener_object.h>

#define NUM_CUBES 4

static const double CUBE_HS = 0.4;

enum { STATE_MOVING = 0, STATE_FROZEN = 1 };

typedef struct callback_demo callback_demo_t;

typedef struct cube_listeners {
    ivp_listener_object_t    object_listener;
    ivp_collision_listener_t collision_listener;
    callback_demo_t         *demo;
    int                      idx;
    bool                     relaunch;   /* push upward when frozen */
} cube_listeners_t;

struct callback_demo {
    ivp_environment_t *env;
    ivp_object_t      *ground;
    ivp_object_t      *cubes[NUM_CUBES];
    cube_listeners_t   listeners[NUM_CUBES];
    int                state[NUM_CUBES];
    float              collision_flash[NUM_CUBES];
    int                n_frozen[NUM_CUBES];
    int                n_revived[NUM_CUBES];
    int                n_collisions[NUM_CUBES];
    int                n_relaunches;
};

static void on_frozen(ivp_listener_object_t *self, ivp_event_object_t *ev)
{
    cube_listeners_t *l = (cube_listeners_t *)self->user_data;
    callback_demo_t *d = l->demo;
    d->state[l->idx] = STATE_FROZEN;
    d->n_frozen[l->idx]++;
    if (l->relaunch) {
        /* push the cube upward (negative Y = up) */
        demo_async_add_speed_ws(d->env, ev->real_object, 0.0, -6.0, 0.0);
        d->n_relaunches++;
    }
}

static void on_revived(ivp_listener_object_t *self, ivp_event_object_t *ev)
{
    cube_listeners_t *l = (cube_listeners_t *)self->user_data;
    (void)ev;
    l->demo->state[l->idx] = STATE_MOVING;
    l->demo->n_revived[l->idx]++;
}

static void on_post_collision(void *ctx, const ivp_collision_event_t *ev)
{
    cube_listeners_t *l = (cube_listeners_t *)ctx;
    (void)ev;
    l->demo->collision_flash[l->idx] = 0.3f;
    l->demo->n_collisions[l->idx]++;
}

static void state_color(callback_demo_t *d, int idx, float dt, float out[3])
{
    const float *c;
    if (d->collision_flash[idx] > 0.0f) {
        d->collision_flash[idx] -= dt;
        c = DEMO_COL_WARNING;
    } else if (d->state[idx] == STATE_FROZEN) {
        c = DEMO_COL_OBJECT_A;
    } else {
        c = DEMO_COL_SPRING;
    }
    out[0] = c[0]; out[1] = c[1]; out[2] = c[2];
}

static void callback_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    callback_demo_t *d = (callback_demo_t *)app->user;
    (void)frame;
    if (!v) {
        /* headless: fade the collision flashes like the drawing does */
        for (int i = 0; i < NUM_CUBES; i++)
            if (d->collision_flash[i] > 0.0f) d->collision_flash[i] -= DEMO_FRAME_TIME;
    }
}

static void callback_draw(demo_app_t *app, demo_viewer_t *v)
{
    callback_demo_t *d = (callback_demo_t *)app->user;
    demo_draw_object(v, d->ground, DEMO_COL_STATIC);
    for (int i = 0; i < NUM_CUBES; i++) {
        float col[3];
        state_color(d, i, v->frame_dt, col);
        demo_draw_object(v, d->cubes[i], col);
    }
    for (int i = 0; i < NUM_CUBES; i++) {
        char label[64];
        snprintf(label, sizeof(label), "Cube %d (frozen %d, revived %d, hits %d)", i,
                 d->n_frozen[i], d->n_revived[i], d->n_collisions[i]);
        demo_hud_object(v, label, d->cubes[i]);
    }
    demo_hud(v, DEMO_COL_INFO, "Green=moving  Blue=frozen  Red=collision");
    demo_hud(v, DEMO_COL_INFO, "Cube 0: relaunched when frozen (%d times)", d->n_relaunches);
}

static void callback_stats(demo_app_t *app, char *buf, size_t size)
{
    callback_demo_t *d = (callback_demo_t *)app->user;
    size_t off = 0;
    for (int i = 0; i < NUM_CUBES && off < size; i++) {
        int n = snprintf(buf + off, size - off, "c%d[%s f%d r%d h%d] ", i,
                         d->state[i] == STATE_FROZEN ? "frozen" : "moving",
                         d->n_frozen[i], d->n_revived[i], d->n_collisions[i]);
        if (n < 0) break;
        off += (size_t)n;
    }
    if (off < size) snprintf(buf + off, size - off, "relaunches=%d", d->n_relaunches);
}

static bool callback_check(demo_app_t *app)
{
    callback_demo_t *d = (callback_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    int froze_all = 1, hit_all = 1;
    for (int i = 0; i < NUM_CUBES; i++) {
        if (d->n_frozen[i] == 0) froze_all = 0;
        if (d->n_collisions[i] == 0) hit_all = 0;
    }
    ok &= demo_check(tag, hit_all, "every cube reported post-collision events");
    ok &= demo_check(tag, froze_all, "every cube came to rest and froze at least once");
    ok &= demo_check(tag, d->n_relaunches >= 2 && d->n_revived[0] == d->n_relaunches,
                     "cube 0 was relaunched each time it froze (%d relaunches, %d revives)",
                     d->n_relaunches, d->n_revived[0]);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title      = "IVP Quickstart: Callbacks";
    app.cfg.tag        = "callback";
    app.cfg.orbit_dist = 20.0f;
    app.cfg.target[1]  = -3.0f;
    app.cfg.friction   = 0.8;
    app.cfg.elasticity = 0.5;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    callback_demo_t *d = (callback_demo_t *)calloc(1, sizeof(*d));
    if (!d) return 1;
    app.user = d;
    d->env = app.world.env;

    d->ground = demo_create_box(&app.world, NULL, 10.0, 0.5, 10.0, 0.0, NULL, 0.0, 0.5, 0.0);

    for (int i = 0; i < NUM_CUBES; i++) {
        d->cubes[i] = demo_create_box(&app.world, NULL, CUBE_HS, CUBE_HS, CUBE_HS, 1.0, NULL,
                                      -3.0 + i * 2.0, -3.0 - i * 1.5, 0.0);
        cube_listeners_t *l = &d->listeners[i];
        l->demo = d;
        l->idx = i;
        l->relaunch = (i == 0);

        l->object_listener.event_object_frozen = on_frozen;
        l->object_listener.event_object_revived = on_revived;
        l->object_listener.user_data = l;
        ivp_object_add_listener_object(d->cubes[i], &l->object_listener);

        ivp_collision_listener_init(&l->collision_listener, IVP_COLLISION_CB_POST_COLLISION, l);
        l->collision_listener.post_collision_fn = on_post_collision;
        ivp_object_add_collision_listener(d->cubes[i], &l->collision_listener);
    }

    app.update = callback_update;
    app.draw   = callback_draw;
    app.stats  = callback_stats;
    app.check  = callback_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(d);
    return rc;
}
