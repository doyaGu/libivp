/* example_contacts.c -- Reading contacts from collision, friction and object listeners
 *
 * Three instruments built from listeners only; none of them touches the
 * simulation.
 *
 *   scale   a plate with a stack of three crates (1, 2 and 3 kg). A friction
 *           listener on the plate tracks its live contact points, and
 *           ivp_contact_point_get_vert_force sums their normal forces: the
 *           scale reads the weight of the stack.
 *   brake   a 2 kg crate slides to a stop on a rough strip. The contact point
 *           of its friction counts the energy it eliminates
 *           (ivp_contact_point_get_eliminated_energy): the heat equals the
 *           kinetic energy the crate started with.
 *   drops   three balls with different elasticity fall 5 m. A pre-collision and
 *           a post-collision listener on each ball read the speed of the ball
 *           before and after its first impact; the ratio is the coefficient of
 *           restitution. (The closing speed in the event is the one before the
 *           impact in both callbacks.)
 *
 * A global object listener counts the objects created and deleted: R drops a
 * further ball into the drop test, and every ball is deleted again 6 s later.
 *
 *   R   drop another ball
 *
 * Demonstrates: ivp_object_add_collision_listener (pre, post, friction created
 *               and deleted), ivp_contact_point_*, ivp_environment_add_listener_
 *               object_global (created / deleted).
 */
#include "demo_app.h"

#include <ivp/events/ivp_listener_collision.h>
#include <ivp/events/ivp_listener_object.h>

#define MAX_CP     16
#define N_DROPS    3
#define MAX_SPARES 8
#define SPARE_LIFE 6.0

typedef struct contact_set {
    struct ivp_contact_pt *cp[MAX_CP];
    int                    n;
} contact_set_t;

typedef struct drop {
    ivp_collision_listener_t listener;
    double                   e_set;          /* the elasticity we gave the ball */
    float                    v_in, v_out;    /* closing speeds of the first impact, 0 = not yet */
    bool                     got_in, got_out;
    ivp_object_t            *ball;
} drop_t;

typedef struct spare {
    ivp_object_t *ball;
    double        born;
} spare_t;

typedef struct contacts_demo {
    ivp_environment_t       *env;
    ivp_object_t            *ground, *rough, *plate;
    ivp_object_t            *stack[3];
    ivp_object_t            *slider;
    drop_t                   drop[N_DROPS];
    spare_t                  spare[MAX_SPARES];
    /* scale */
    ivp_collision_listener_t plate_listener;
    contact_set_t            plate_cps;
    float                    scale_max;
    /* brake */
    ivp_collision_listener_t slider_listener;
    contact_set_t            slider_cps;
    double                   heat;            /* J */
    double                   slider_ke0;
    /* object listener */
    ivp_listener_object_t    obj_listener;
    int                      created, deleted;
    int                      spawned;
} contacts_demo_t;

/* ── listeners ───────────────────────────────────────────────────────────── */

static void cps_add(contact_set_t *set, struct ivp_contact_pt *cp)
{
    for (int i = 0; i < set->n; i++)
        if (set->cp[i] == cp) return;
    if (set->n < MAX_CP) set->cp[set->n++] = cp;
}

static void cps_remove(contact_set_t *set, struct ivp_contact_pt *cp)
{
    for (int i = 0; i < set->n; i++)
        if (set->cp[i] == cp) {
            set->cp[i] = set->cp[--set->n];
            return;
        }
}

static void plate_created(void *ctx, const ivp_friction_event_t *ev) { cps_add(&((contacts_demo_t *)ctx)->plate_cps, ev->friction_handle); }
static void plate_deleted(void *ctx, const ivp_friction_event_t *ev) { cps_remove(&((contacts_demo_t *)ctx)->plate_cps, ev->friction_handle); }

static void slider_created(void *ctx, const ivp_friction_event_t *ev) { cps_add(&((contacts_demo_t *)ctx)->slider_cps, ev->friction_handle); }

static void slider_deleted(void *ctx, const ivp_friction_event_t *ev)
{
    contacts_demo_t *s = (contacts_demo_t *)ctx;
    /* the heat of a contact that goes away must not be lost */
    s->heat += (double)ivp_contact_point_get_eliminated_energy(ev->friction_handle);
    ivp_contact_point_reset_eliminated_energy(ev->friction_handle);
    cps_remove(&s->slider_cps, ev->friction_handle);
}

static void drop_pre(void *ctx, const ivp_collision_event_t *ev)
{
    drop_t *d = (drop_t *)ctx;
    if (d->got_in) return;
    (void)ev;
    d->v_in = fabsf((float)d->ball->physical_core->speed.k[1]);        /* the ball before the impact */
    d->got_in = true;
}

static void drop_post(void *ctx, const ivp_collision_event_t *ev)
{
    drop_t *d = (drop_t *)ctx;
    if (!d->got_in || d->got_out) return;
    (void)ev;
    d->v_out = fabsf((float)d->ball->physical_core->speed.k[1]);       /* and after it */
    d->got_out = true;
}

static void on_created(ivp_listener_object_t *self, ivp_event_object_t *ev)
{
    (void)ev;
    ((contacts_demo_t *)self->user_data)->created++;
}

static void on_deleted(ivp_listener_object_t *self, ivp_event_object_t *ev)
{
    (void)ev;
    ((contacts_demo_t *)self->user_data)->deleted++;
}

/* ── scene ───────────────────────────────────────────────────────────────── */

static float scale_reading(const contacts_demo_t *s)
{
    double sum = 0.0;
    for (int i = 0; i < s->plate_cps.n; i++) sum += (double)ivp_contact_point_get_vert_force(s->plate_cps.cp[i]);
    return (float)sum;
}

static void spawn_ball(contacts_demo_t *s, demo_world_t *w)
{
    for (int i = 0; i < MAX_SPARES; i++) {
        if (s->spare[i].ball) continue;
        const ivp_material_t m = demo_material(0.5, 0.7);
        s->spare[i].ball = demo_create_ball(w, &m, 0.3, 0.5, NULL, 8.0 + 0.4 * (double)(s->spawned % 5), -6.0,
                                            0.8 * (double)((s->spawned % 3) - 1));
        s->spare[i].born = (double)ivp_environment_get_time(s->env);
        s->spawned++;
        return;
    }
}

static void contacts_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    contacts_demo_t *s = (contacts_demo_t *)app->user;
    demo_world_t *w = &app->world;
    const double t = (double)ivp_environment_get_time(s->env);
    if ((v && demo_key_just_pressed(v, SDL_SCANCODE_R)) || (!v && (frame == 60 || frame == 120))) spawn_ball(s, w);
    for (int i = 0; i < MAX_SPARES; i++)
        if (s->spare[i].ball && t - s->spare[i].born > SPARE_LIFE) {
            demo_destroy_object(w, s->spare[i].ball);
            s->spare[i].ball = NULL;
        }

    const float reading = scale_reading(s);
    if (t > 2.0 && reading > s->scale_max) s->scale_max = reading;
    /* the brake: count the heat of the live contacts and clear their counters */
    for (int i = 0; i < s->slider_cps.n; i++) {
        s->heat += (double)ivp_contact_point_get_eliminated_energy(s->slider_cps.cp[i]);
        ivp_contact_point_reset_eliminated_energy(s->slider_cps.cp[i]);
    }
}

static void contacts_reset(demo_app_t *app)
{
    contacts_demo_t *s = (contacts_demo_t *)app->user;
    s->heat = 0.0;
    s->scale_max = 0.0f;
    for (int i = 0; i < N_DROPS; i++) s->drop[i].got_in = s->drop[i].got_out = false;
}

static void contacts_draw(demo_app_t *app, demo_viewer_t *v)
{
    contacts_demo_t *s = (contacts_demo_t *)app->user;
    const float c_ground[3] = { 0.50f, 0.53f, 0.57f }, c_rough[3] = { 0.30f, 0.30f, 0.34f }, c_plate[3] = { 0.80f, 0.80f, 0.85f },
                c_crate[3][3] = { { 0.95f, 0.65f, 0.30f }, { 0.40f, 0.78f, 0.95f }, { 0.55f, 0.90f, 0.50f } },
                c_ball[N_DROPS][3] = { { 0.95f, 0.45f, 0.35f }, { 0.95f, 0.80f, 0.35f }, { 0.60f, 0.70f, 0.95f } },
                c_spare[3] = { 1.0f, 1.0f, 1.0f };
    demo_draw_object(v, s->ground, c_ground);
    demo_draw_object(v, s->rough, c_rough);
    demo_draw_object(v, s->plate, c_plate);
    for (int i = 0; i < 3; i++) demo_draw_object(v, s->stack[i], c_crate[i]);
    demo_draw_object(v, s->slider, c_crate[0]);
    for (int i = 0; i < N_DROPS; i++) demo_draw_object(v, s->drop[i].ball, c_ball[i]);
    for (int i = 0; i < MAX_SPARES; i++)
        if (s->spare[i].ball) demo_draw_object(v, s->spare[i].ball, c_spare);
    /* the live contact points of the plate */
    for (int i = 0; i < s->plate_cps.n; i++) {
        ivp_vec3r_t n;
        ivp_contact_point_get_surface_normal_ws(s->plate_cps.cp[i], &n);
        (void)n;
    }

    demo_hud(v, DEMO_COL_INFO, "scale: %d contact points, reads %.1f (stack weight %.1f N)", s->plate_cps.n,
             (double)scale_reading(s), 6.0 * DEMO_GRAVITY);
    demo_hud(v, DEMO_COL_INFO, "brake: %d contact points, heat %.1f J (kinetic energy at the start %.1f J)", s->slider_cps.n,
             s->heat, s->slider_ke0);
    for (int i = 0; i < N_DROPS; i++) {
        const drop_t *d = &s->drop[i];
        if (d->got_out)
            demo_hud(v, c_ball[i], "drop %d: elasticity %.1f, in %.2f m/s, out %.2f m/s, restitution %.2f", i + 1, d->e_set,
                     (double)d->v_in, (double)d->v_out, d->v_in > 0.0f ? (double)(d->v_out / d->v_in) : 0.0);
        else
            demo_hud(v, c_ball[i], "drop %d: elasticity %.1f, waiting for the impact", i + 1, d->e_set);
    }
    demo_hud(v, DEMO_COL_HELP, "object listener: %d created, %d deleted", s->created, s->deleted);
}

static void contacts_stats(demo_app_t *app, char *buf, size_t size)
{
    contacts_demo_t *s = (contacts_demo_t *)app->user;
    snprintf(buf, size, "scale %.1f, heat %.1f J, objects +%d -%d", (double)scale_reading(s), s->heat, s->created, s->deleted);
}

static bool contacts_check(demo_app_t *app)
{
    contacts_demo_t *s = (contacts_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    const float weight = (float)(6.0 * DEMO_GRAVITY);
    ok &= demo_check(tag, s->plate_cps.n > 0, "the plate has %d live contact points", s->plate_cps.n);
    ok &= demo_check(tag, fabsf(scale_reading(s) - weight) < 0.1f * weight, "the scale reads the weight (%.1f against %.1f N)",
                     (double)scale_reading(s), (double)weight);
    ok &= demo_check(tag, fabs(s->heat - s->slider_ke0) < 0.2 * s->slider_ke0,
                     "the eliminated energy is the kinetic energy of the crate (%.1f J against %.1f J)", s->heat, s->slider_ke0);
    const double v_fall = sqrt(2.0 * DEMO_GRAVITY * 5.0);
    for (int i = 0; i < N_DROPS; i++) {
        const drop_t *d = &s->drop[i];
        ok &= demo_check(tag, d->got_out && fabs((double)d->v_in - v_fall) < 0.1 * v_fall,
                         "drop %d: impact speed %.2f m/s (free fall %.2f)", i + 1, (double)d->v_in, v_fall);
    }
    const float r0 = s->drop[0].v_in > 0.0f ? s->drop[0].v_out / s->drop[0].v_in : 0.0f,
                r2 = s->drop[2].v_in > 0.0f ? s->drop[2].v_out / s->drop[2].v_in : 0.0f;
    ok &= demo_check(tag, r0 > r2 + 0.2f, "the restitution follows the elasticity (%.2f against %.2f)", (double)r0, (double)r2);
    ok &= demo_check(tag, s->created >= 2 && s->deleted >= 1,
                     "the global object listener saw %d objects created and %d deleted", s->created, s->deleted);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Contacts -- Reading Listeners";
    app.cfg.tag         = "contacts";
    app.cfg.friction    = 0.6;
    app.cfg.elasticity  = 0.5;
    app.cfg.orbit_dist  = 21.0f;
    app.cfg.orbit_yaw   = 180.0f;
    app.cfg.orbit_pitch = -20.0f;
    app.cfg.target[1]   = -2.0f;
    app.cfg.grid_extent = -1.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    contacts_demo_t *s = (contacts_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->env = w->env;

    s->ground = demo_create_box(w, NULL, 14.0, 0.5, 5.0, 0.0, NULL, 0.0, 0.5, 0.0);

    /* scale */
    s->plate = demo_create_box(w, NULL, 2.0, 0.2, 1.5, 0.0, NULL, -9.0, -0.2, 0.0);
    static const double crate_mass[3] = { 3.0, 2.0, 1.0 };
    for (int i = 0; i < 3; i++) {
        s->stack[i] = demo_create_box(w, NULL, 0.4, 0.4, 0.4, crate_mass[i], NULL, -9.0, -0.4 - 0.4 - 0.8 * i - 0.01, 0.0);
        if (!s->stack[i]) return 1;
    }
    ivp_collision_listener_init(&s->plate_listener, IVP_COLLISION_CB_FRICTION_CREATED | IVP_COLLISION_CB_FRICTION_DELETED, s);
    s->plate_listener.friction_created_fn = plate_created;
    s->plate_listener.friction_deleted_fn = plate_deleted;
    ivp_object_add_collision_listener(s->plate, &s->plate_listener);

    /* brake: a rough strip, the crate comes in at 6 m/s */
    const ivp_material_t rough = demo_material(0.5, 0.05);
    s->rough = demo_create_box(w, &rough, 6.0, 0.05, 1.0, 0.0, NULL, -1.0, -0.05, 3.0);
    const ivp_material_t slid = demo_material(1.0, 0.05);
    s->slider = demo_create_box(w, &slid, 0.4, 0.4, 0.4, 2.0, NULL, -6.5, -0.4 - 0.12, 3.0);
    if (!s->slider) return 1;
    s->slider->physical_core->speed = ivp_vec3r(IVP_REAL(6.0), IVP_REAL(0.0), IVP_REAL(0.0));
    s->slider_ke0 = 0.5 * 2.0 * 36.0;
    ivp_collision_listener_init(&s->slider_listener, IVP_COLLISION_CB_FRICTION_CREATED | IVP_COLLISION_CB_FRICTION_DELETED, s);
    s->slider_listener.friction_created_fn = slider_created;
    s->slider_listener.friction_deleted_fn = slider_deleted;
    ivp_object_add_collision_listener(s->slider, &s->slider_listener);

    /* drops */
    static const double elast[N_DROPS] = { 0.9, 0.5, 0.1 };
    for (int i = 0; i < N_DROPS; i++) {
        drop_t *d = &s->drop[i];
        d->e_set = elast[i];
        const ivp_material_t m = demo_material(0.5, elast[i]);
        d->ball = demo_create_ball(w, &m, 0.3, 0.5, NULL, 3.0 + 1.5 * i, -5.0 - 0.3, -1.0);
        if (!d->ball) return 1;
        ivp_collision_listener_init(&d->listener, IVP_COLLISION_CB_PRE_COLLISION | IVP_COLLISION_CB_POST_COLLISION, d);
        d->listener.pre_collision_fn = drop_pre;
        d->listener.post_collision_fn = drop_post;
        ivp_object_add_collision_listener(d->ball, &d->listener);
    }

    /* every object created from now on and every deleted one */
    s->obj_listener.event_object_created = on_created;
    s->obj_listener.event_object_deleted = on_deleted;
    s->obj_listener.user_data = s;
    ivp_environment_add_listener_object_global(w->env, &s->obj_listener);

    app.controls    = "R: drop another ball";
    app.update      = contacts_update;
    app.on_reset    = contacts_reset;
    app.draw        = contacts_draw;
    app.stats       = contacts_stats;
    app.check       = contacts_check;

    int rc = demo_run(&app, argc, argv);
    ivp_environment_remove_listener_object_global(w->env, &s->obj_listener);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
