/* example_compound.c -- Several shapes in one rigid body
 *
 * A dumbbell, a hammer, a jack and a table are each built from several
 * objects that ivp_object_attach_object merges into the core of a parent:
 * the whole thing moves as one body with the mass and inertia given to the
 * parent, while every part keeps its own shape for collisions.
 *
 *   B   break every compound apart with ivp_object_detach_object (each part
 *       gets its own core and falls on its own)
 *   Space   put everything back: parts are attached again and moved to their
 *       place in the parent with ivp_object_reposition_object_Ros
 *
 * The parts of one compound overlap, and the engine would resolve impacts
 * between objects of the same core, so every compound is one collision
 * group (environment group filter); breaking it apart clears the group.
 *
 * Demonstrates: ivp_object_attach_object, ivp_object_detach_object,
 *               ivp_object_reposition_object_Ros, compound inertia, the
 *               collision group filter.
 */
#include "demo_app.h"

#include <ivp/collision/ivp_collision_filter.h>
#include <ivp/dynamics/ivp_object_attach.h>

#define N_COMPOUNDS 4
#define MAX_KIDS    4

typedef struct compound {
    const char   *name;
    ivp_object_t *parent;
    ivp_object_t *kids[MAX_KIDS];
    double        kid_mass[MAX_KIDS];
    ivp_vec3r_t   local[MAX_KIDS];    /* shift in the parent's object space */
    int           n_kids;
    float         col_parent[3], col_kid[3];
    double        reach;              /* farthest part from the parent's centre */
    double        mass;
    double        half[3];            /* bounding box, for the inertia */
} compound_t;

typedef struct compound_demo {
    ivp_object_t *ground;
    compound_t    c[N_COMPOUNDS];
    bool          broken;
    float         worst_stretch;      /* largest change of a part's distance in an intact compound */
    int           detached_seen;      /* parts loose when the headless run looked */
    float         start_dist[N_COMPOUNDS][MAX_KIDS];
} compound_demo_t;

static float dist3(const float a[3], const float b[3])
{
    return sqrtf((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
}

/* kid i of compound c: a box (hx != 0) or a ball (hx == 0, radius hy). */
static ivp_object_t *make_kid(demo_world_t *w, double mass, double hx, double hy, double hz,
                              double x, double y, double z)
{
    return hx > 0.0 ? demo_create_box(w, NULL, hx, hy, hz, mass, NULL, x, y, z)
                    : demo_create_ball(w, NULL, hy, mass, NULL, x, y, z);
}

static void build(compound_t *c, const char *name, double mass, const double half[3])
{
    c->name = name;
    c->mass = mass;
    memcpy(c->half, half, sizeof(c->half));
}

static bool add_kid(demo_world_t *w, compound_t *c, double px, double py, double pz,
                    double kmass, double hx, double hy, double hz,
                    double lx, double ly, double lz)
{
    if (c->n_kids >= MAX_KIDS) return false;
    ivp_object_t *k = make_kid(w, kmass, hx, hy, hz, px + lx, py + ly, pz + lz);
    if (!k) return false;
    int i = c->n_kids++;
    c->kids[i] = k;
    c->kid_mass[i] = kmass;
    c->local[i] = ivp_vec3r((ivp_real_t)lx, (ivp_real_t)ly, (ivp_real_t)lz);
    double r = sqrt(lx * lx + ly * ly + lz * lz) + (hx > 0.0 ? sqrt(hx*hx + hy*hy + hz*hz) : hy);
    if (r > c->reach) c->reach = r;
    return true;
}

static void attach_all(compound_t *c)
{
    for (int i = 0; i < c->n_kids; i++) ivp_object_attach_object(c->parent, c->kids[i], c->reach);
}

static void set_group(compound_t *c, int group)
{
    ivp_object_set_collision_group(c->parent, group);
    for (int i = 0; i < c->n_kids; i++) ivp_object_set_collision_group(c->kids[i], group);
}

static void detach_all(demo_world_t *w, compound_t *c)
{
    for (int i = 0; i < c->n_kids; i++) {
        if (c->kids[i]->physical_core == c->parent->physical_core) {
            ivp_object_template_t t;
            demo_template_dynamic(&t, &w->mat, c->kid_mass[i]);
            demo_set_box_inertia(&t, c->kid_mass[i], 0.3, 0.3, 0.3);
            t.client_data = c->kids[i]->client_data;       /* keep the demo body */
            t.position = c->kids[i]->physical_core->pos_world_f_core_last_psi;
            ivp_object_detach_object(c->kids[i], &t);
        }
    }
    set_group(c, 0);                 /* loose parts collide with each other */
}

static bool is_attached(const compound_t *c, int i)
{
    return c->kids[i]->physical_core == c->parent->physical_core;
}

static void compound_reset(demo_app_t *app)
{
    compound_demo_t *s = (compound_demo_t *)app->user;
    /* the restored poses put every detached part at its parent's origin;
     * attach it again and move it to its place in the parent */
    for (int k = 0; k < N_COMPOUNDS; k++) {
        compound_t *c = &s->c[k];
        set_group(c, k + 1);
        for (int i = 0; i < c->n_kids; i++) {
            if (!is_attached(c, i)) ivp_object_attach_object(c->parent, c->kids[i], c->reach);
            const ivp_quatr_t q = ivp_quatr_identity();
            ivp_object_reposition_object_Ros(c->parent, c->kids[i], &q, &c->local[i], false);
        }
    }
    s->broken = false;
    s->worst_stretch = 0.0f;
}

static void compound_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    compound_demo_t *s = (compound_demo_t *)app->user;
    bool brk = v ? demo_key_just_pressed(v, SDL_SCANCODE_B) : (frame == 240);
    if (!v && frame == 400) {
        for (int i = 0; i < s->c[N_COMPOUNDS - 1].n_kids; i++)
            s->detached_seen += !is_attached(&s->c[N_COMPOUNDS - 1], i);
    }
    if (!v && frame == 420) {            /* what Space does in a window */
        demo_world_restore_state(&app->world);
        compound_reset(app);
    }
    if (brk && !s->broken) {
        /* the headless run breaks only the last compound, so the others show they stay rigid */
        for (int k = 0; k < N_COMPOUNDS; k++)
            if (v || k == N_COMPOUNDS - 1) detach_all(&app->world, &s->c[k]);
        s->broken = true;
    }
    for (int k = 0; k < N_COMPOUNDS; k++) {
        compound_t *c = &s->c[k];
        float pc[3];
        demo_object_center(c->parent, pc);
        for (int i = 0; i < c->n_kids; i++) {
            if (!is_attached(c, i)) continue;
            float kc[3];
            demo_object_center(c->kids[i], kc);
            float st = fabsf(dist3(pc, kc) - s->start_dist[k][i]);
            if (st > s->worst_stretch) s->worst_stretch = st;
        }
    }
}

static void compound_draw(demo_app_t *app, demo_viewer_t *v)
{
    compound_demo_t *s = (compound_demo_t *)app->user;
    const float c_ground[3] = { 0.55f, 0.58f, 0.62f };
    demo_draw_object(v, s->ground, c_ground);
    for (int k = 0; k < N_COMPOUNDS; k++) {
        compound_t *c = &s->c[k];
        demo_draw_object(v, c->parent, c->col_parent);
        int attached = 0;
        for (int i = 0; i < c->n_kids; i++) {
            demo_draw_object(v, c->kids[i], is_attached(c, i) ? c->col_kid : c->col_parent);
            attached += is_attached(c, i);
        }
        demo_hud(v, DEMO_COL_INFO, "%-9s %d parts, %d attached to one core  (mass %.1f)", c->name,
                 c->n_kids + 1, attached + 1, c->mass);
    }
    demo_hud(v, DEMO_COL_INFO, "largest change of a part's distance in an intact compound: %.4f m",
             (double)s->worst_stretch);
}

static void compound_stats(demo_app_t *app, char *buf, size_t size)
{
    compound_demo_t *s = (compound_demo_t *)app->user;
    float sp = 0.0f;
    for (int k = 0; k < N_COMPOUNDS; k++) {
        float v = demo_object_speed(s->c[k].parent);
        if (v > sp) sp = v;
    }
    snprintf(buf, size, "max speed %.2f m/s, worst stretch %.4f m, broken %d", (double)sp,
             (double)s->worst_stretch, s->broken ? 1 : 0);
}

static bool compound_check(demo_app_t *app)
{
    compound_demo_t *s = (compound_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, s->detached_seen == s->c[N_COMPOUNDS - 1].n_kids,
                     "breaking apart the table left %d/%d parts on their own cores", s->detached_seen,
                     s->c[N_COMPOUNDS - 1].n_kids);
    ok &= demo_check(tag, s->worst_stretch < 0.02f,
                     "attached parts stay rigid, also after the reset (largest change %.4f m)",
                     (double)s->worst_stretch);
    for (int k = 0; k < N_COMPOUNDS; k++) {
        compound_t *c = &s->c[k];
        int attached = 0;
        for (int i = 0; i < c->n_kids; i++) attached += is_attached(c, i);
        ok &= demo_check(tag, attached == c->n_kids, "%s: %d/%d parts attached", c->name, attached, c->n_kids);
        float p[3];
        demo_object_center(c->parent, p);
        ok &= demo_check(tag, p[1] < 0.0f && fabsf(p[0]) < 20.0f && fabsf(p[2]) < 20.0f,
                         "%s rests above the ground (y %.2f)", c->name, (double)p[1]);
    }
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Compound -- Attach and Detach";
    app.cfg.tag         = "compound";
    app.cfg.friction    = 0.8;
    app.cfg.elasticity  = 0.1;
    app.cfg.orbit_dist  = 17.0f;
    app.cfg.target[1]   = -2.0f;
    app.cfg.grid_extent = -1.0f;
    ivp_environment_config_t ecfg;
    ivp_environment_config_init(&ecfg);
    ecfg.gravity = ivp_vec3r(IVP_REAL(0.0), (ivp_real_t)DEMO_GRAVITY, IVP_REAL(0.0));
    ecfg.delta_psi_time = (ivp_real_t)DEMO_PSI_TIME;
    ecfg.collision_filter = ivp_collision_filter_group();
    if (!demo_world_init_env(&app.world, &app.cfg, &ecfg)) return 1;

    compound_demo_t *s = (compound_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    s->ground = demo_create_box(w, NULL, 12.0, 0.5, 8.0, 0.0, NULL, 0.0, 0.0, 0.0);

    /* ── dumbbell: a rod with a ball at each end ───────────────────────── */
    {
        compound_t *c = &s->c[0];
        double px = -7.5, py = -4.0, pz = 0.0, half[3] = { 2.0, 0.5, 0.5 };
        build(c, "dumbbell", 3.0, half);
        ivp_object_template_t t;
        demo_template_dynamic(&t, &w->mat, 3.0);
        demo_set_box_inertia(&t, 3.0, 2.0, 0.5, 0.5);
        demo_template_pose(&t, NULL, px, py, pz);
        c->parent = demo_create_box_templ(w, &t, 1.3, 0.1, 0.1);
        add_kid(w, c, px, py, pz, 0.5, 0.0, 0.5, 0.0, -1.5, 0.0, 0.0);
        add_kid(w, c, px, py, pz, 0.5, 0.0, 0.5, 0.0, 1.5, 0.0, 0.0);
        const float a[3] = { 0.85f, 0.35f, 0.30f }, b[3] = { 0.95f, 0.80f, 0.30f };
        memcpy(c->col_parent, a, sizeof(a)); memcpy(c->col_kid, b, sizeof(b));
    }
    /* ── hammer: heavy head (the parent) and a long handle ─────────────── */
    {
        compound_t *c = &s->c[1];
        double px = -2.5, py = -4.5, pz = 0.0, half[3] = { 1.5, 0.5, 0.5 };
        build(c, "hammer", 4.0, half);
        ivp_object_template_t t;
        demo_template_dynamic(&t, &w->mat, 4.0);
        demo_set_box_inertia(&t, 4.0, 1.7, 0.5, 0.5);
        demo_template_pose(&t, NULL, px, py, pz);
        c->parent = demo_create_box_templ(w, &t, 0.3, 0.5, 0.5);
        add_kid(w, c, px, py, pz, 0.4, 1.4, 0.12, 0.12, 1.7, 0.0, 0.0);
        const float a[3] = { 0.55f, 0.58f, 0.68f }, b[3] = { 0.78f, 0.55f, 0.32f };
        memcpy(c->col_parent, a, sizeof(a)); memcpy(c->col_kid, b, sizeof(b));
    }
    /* ── jack: a cube with three bars through it ───────────────────────── */
    {
        compound_t *c = &s->c[2];
        double px = 2.5, py = -4.0, pz = 0.0, half[3] = { 1.0, 1.0, 1.0 };
        build(c, "jack", 3.0, half);
        ivp_object_template_t t;
        demo_template_dynamic(&t, &w->mat, 3.0);
        demo_set_box_inertia(&t, 3.0, 1.0, 1.0, 1.0);
        demo_template_pose(&t, NULL, px, py, pz);
        c->parent = demo_create_box_templ(w, &t, 0.35, 0.35, 0.35);
        add_kid(w, c, px, py, pz, 0.3, 1.0, 0.1, 0.1, 0.0, 0.0, 0.0);
        add_kid(w, c, px, py, pz, 0.3, 0.1, 1.0, 0.1, 0.0, 0.0, 0.0);
        add_kid(w, c, px, py, pz, 0.3, 0.1, 0.1, 1.0, 0.0, 0.0, 0.0);
        const float a[3] = { 0.35f, 0.70f, 0.90f }, b[3] = { 0.55f, 0.85f, 0.45f };
        memcpy(c->col_parent, a, sizeof(a)); memcpy(c->col_kid, b, sizeof(b));
    }
    /* ── table: a slab on four legs (up is -Y, the legs hang at +Y) ────── */
    {
        compound_t *c = &s->c[3];
        double px = 7.5, py = -3.5, pz = 0.0, half[3] = { 1.3, 0.8, 0.8 };
        build(c, "table", 4.0, half);
        ivp_object_template_t t;
        demo_template_dynamic(&t, &w->mat, 4.0);
        demo_set_box_inertia(&t, 4.0, 1.3, 0.8, 0.8);
        demo_template_pose(&t, NULL, px, py, pz);
        c->parent = demo_create_box_templ(w, &t, 1.3, 0.1, 0.8);
        add_kid(w, c, px, py, pz, 0.2, 0.1, 0.6, 0.1, -1.1, 0.7, -0.6);
        add_kid(w, c, px, py, pz, 0.2, 0.1, 0.6, 0.1, 1.1, 0.7, -0.6);
        add_kid(w, c, px, py, pz, 0.2, 0.1, 0.6, 0.1, -1.1, 0.7, 0.6);
        add_kid(w, c, px, py, pz, 0.2, 0.1, 0.6, 0.1, 1.1, 0.7, 0.6);
        const float a[3] = { 0.80f, 0.60f, 0.40f }, b[3] = { 0.65f, 0.45f, 0.28f };
        memcpy(c->col_parent, a, sizeof(a)); memcpy(c->col_kid, b, sizeof(b));
    }

    for (int k = 0; k < N_COMPOUNDS; k++) {
        compound_t *c = &s->c[k];
        if (!c->parent) return 1;
        set_group(c, k + 1);
        attach_all(c);
        float pc[3];
        demo_object_center(c->parent, pc);
        for (int i = 0; i < c->n_kids; i++) {
            float kc[3];
            demo_object_center(c->kids[i], kc);
            s->start_dist[k][i] = dist3(pc, kc);
        }
    }
    /* tumble the dumbbell and the hammer a little as they fall */
    demo_async_add_rot_speed_cs(w->env, s->c[0].parent, 0.0, 1.5, 2.0);
    demo_async_add_rot_speed_cs(w->env, s->c[1].parent, 1.0, 0.0, 3.0);

    app.update   = compound_update;
    app.on_reset = compound_reset;
    app.draw     = compound_draw;
    app.stats    = compound_stats;
    app.check    = compound_check;
    app.controls = "B: break the compounds apart";

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
