/* example_ballance.c -- A mini Ballance: roll a ball along a course of planks
 *
 * After the game of that name: you roll a ball over wooden planks, ramps, a
 * moving platform, a narrow beam and a wall of crates, and the ball changes its
 * material on the way. All of it is rigid body physics; there is no jump.
 *
 *   wood    light and lively, the ball you start with
 *   paper   a feather: quick to turn, hardly any weight, bounces off everything
 *   stone   heavy and slow, the only one that can push the crates aside
 *
 * Roll over a coloured pad to change. A green pad is a checkpoint: if you fall
 * off the course you start again from the last one and lose a life; three lives
 * and the course starts over. The goal is the platform with the flag.
 *
 *   Up / Down / Left / Right   roll, relative to where the camera looks
 *   Backspace                  back to the last checkpoint
 *   P                          the autopilot rolls for you (the one of the self-check)
 *   (gamepad)                  left stick rolls
 *   The camera follows the ball and turns into the direction it rolls; drag the
 *   right mouse button to look around.
 *
 * Headless, a little autopilot rolls the whole course (it waits for the moving
 * platform) and the self-check requires it to reach the flag without falling.
 *
 * Demonstrates: static boxes as a level (ramps by quaternion), a golem driven
 *               platform, dynamic crates, one ball per material that is swapped
 *               by beaming (ivp_object_beam_to_new_position), friction listeners
 *               as the ground sensor, a chase camera.
 */
#include "demo_app.h"
#include "demo_drive.h"

#include <ivp/controllers/ivp_controller_golem.h>
#include <ivp/events/ivp_listener_collision.h>

#define N_KINDS    3
#define MAX_PIECES 40
#define N_CRATES   9
#define N_PADS     3
#define N_CHECK    3
#define FALL_Y     7.0f
#define LIVES      3
#define PI_F       3.14159265f

enum { WOOD, STONE, PAPER };

typedef struct kind {
    const char *name;
    double      radius, mass, mu, elasticity;
    float       accel;          /* m/s^2 on the ground */
    float       vmax;           /* m/s */
    float       col[3], spot[3];
} kind_t;

static const kind_t kinds[N_KINDS] = {
    { "wood",  0.45, 1.0, 0.7, 0.35, 11.0f, 9.0f, { 0.80f, 0.58f, 0.34f }, { 0.55f, 0.36f, 0.18f } },
    { "stone", 0.50, 5.0, 0.8, 0.10, 7.0f, 7.5f, { 0.62f, 0.63f, 0.66f }, { 0.38f, 0.39f, 0.42f } },
    { "paper", 0.40, 0.25, 0.5, 0.30, 15.0f, 10.5f, { 0.95f, 0.95f, 0.90f }, { 0.70f, 0.78f, 0.90f } },
};

typedef struct piece {
    ivp_object_t *obj;
    float         col[3];
} piece_t;

typedef struct pad {
    double x, z, y_top;
    int    kind;
} pad_t;

typedef struct spot {
    double x, z, y_top;           /* a checkpoint */
} spot_t;

typedef struct ballance_demo {
    ivp_environment_t       *env;
    piece_t                  piece[MAX_PIECES];
    int                      n_pieces;
    ivp_object_t            *crate[N_CRATES];
    float                    crate_start[N_CRATES][3];
    ivp_object_t            *mover;                 /* the moving platform */
    ivp_golem_state_t        golem;
    double                   mover_z0, mover_y;
    ivp_object_t            *park;
    ivp_object_t            *ball[N_KINDS];
    ivp_collision_listener_t lst[N_KINDS];
    int                      contacts[N_KINDS];
    int                      active;
    pad_t                    pad[N_PADS];
    spot_t                   check[N_CHECK];
    int                      checkpoint;
    int                      lives;
    double                   time;                  /* s of play */
    double                   transform_cooldown;
    bool                     done;
    double                   done_time;
    int                      falls;
    unsigned                 kinds_used;
    demo_drive_t             drive;
    /* the autopilot */
    int                      bot_stage;
    bool                     autopilot;               /* the bot plays in the window, too */
    /* measurements */
    float                    max_speed;
    float                    mover_err;
    int                      mover_problems;
    char                     message[80];
    double                   message_until;
} ballance_demo_t;

/* ── the course ──────────────────────────────────────────────────────────── */

static ivp_object_t *add_piece(ballance_demo_t *s, demo_world_t *w, double hx, double hy, double hz, double cx, double cy,
                               double cz, const ivp_quatr_t *q, const float col[3])
{
    if (s->n_pieces >= MAX_PIECES) return NULL;
    const ivp_material_t m = demo_material(0.9, 0.2);
    ivp_object_t *o = demo_create_box(w, &m, hx, hy, hz, 0.0, q, cx, cy, cz);
    if (!o) return NULL;
    s->piece[s->n_pieces].obj = o;
    memcpy(s->piece[s->n_pieces].col, col, sizeof(float) * 3);
    s->n_pieces++;
    return o;
}

/* A platform whose top is at y_top, between z0 and z1, half width hx. */
static void plat(ballance_demo_t *s, demo_world_t *w, double cx, double z0, double z1, double hx, double y_top, const float col[3])
{
    add_piece(s, w, hx, 0.3, 0.5 * (z1 - z0), cx, y_top + 0.3, 0.5 * (z0 + z1), NULL, col);
}

/* A ramp whose top runs from (z0, y0) to (z1, y1); rising along +z when y1 < y0 (up is -Y). */
static void ramp(ballance_demo_t *s, demo_world_t *w, double cx, double z0, double y0, double z1, double y1, double hx,
                 const float col[3])
{
    const double dz = z1 - z0, dy = y1 - y0, len = sqrt(dz * dz + dy * dy), ang = atan2(-dy, dz);
    const ivp_quatr_t q = ivp_quatr_make(sin(0.5 * ang), 0.0, 0.0, cos(0.5 * ang));
    /* the top surface passes through the middle of the run; the slab is 0.3 below it, along its normal */
    const double mz = 0.5 * (z0 + z1), my = 0.5 * (y0 + y1), cy = my + 0.3 * cos(ang), cz = mz + 0.3 * sin(ang);
    add_piece(s, w, hx, 0.3, 0.5 * len, cx, cy, cz, &q, col);
}

static void build_course(ballance_demo_t *s, demo_world_t *w)
{
    const float wood[3] = { 0.72f, 0.55f, 0.34f }, dark[3] = { 0.55f, 0.40f, 0.25f }, stone[3] = { 0.55f, 0.57f, 0.62f },
                beam[3] = { 0.62f, 0.45f, 0.28f };
    plat(s, w, 0.0, 0.0, 8.0, 4.0, 0.0, wood);                  /* start */
    plat(s, w, 0.0, 8.0, 22.0, 1.2, 0.0, dark);                 /* plank */
    ramp(s, w, 0.0, 22.0, 0.0, 32.0, -3.0, 1.2, wood);          /* up */
    plat(s, w, 0.0, 32.0, 40.0, 3.0, -3.0, wood);               /* platform A: paper pad */
    plat(s, w, 0.0, 48.0, 56.0, 3.0, -3.0, wood);               /* platform B, beyond the gap: checkpoint */
    plat(s, w, 0.0, 56.0, 70.0, 0.5, -3.0, beam);               /* the beam */
    plat(s, w, 0.0, 70.0, 80.0, 3.0, -3.0, stone);              /* platform C: stone pad, the crates */
    ramp(s, w, 0.0, 80.0, -3.0, 90.0, 0.0, 1.2, wood);          /* down */
    plat(s, w, 0.0, 90.0, 98.0, 3.0, 0.0, wood);                /* platform D: wood pad, checkpoint */
    plat(s, w, 10.0, 92.8, 95.2, 7.0, 0.0, dark);               /* the plank to the goal */
    plat(s, w, 22.0, 90.0, 98.0, 4.0, 0.0, stone);              /* the goal */
    /* a post and a flag on the goal */
    const float red[3] = { 0.90f, 0.25f, 0.20f };
    add_piece(s, w, 0.06, 1.6, 0.06, 24.0, -1.6, 94.0, NULL, stone);
    add_piece(s, w, 0.04, 0.4, 0.6, 24.0, -2.8, 94.6, NULL, red);

    s->pad[0] = (pad_t){ 0.0, 36.0, -3.0, PAPER };
    s->pad[1] = (pad_t){ 0.0, 72.5, -3.0, STONE };
    s->pad[2] = (pad_t){ 0.0, 93.5, 0.0, WOOD };
    s->check[0] = (spot_t){ 0.0, 2.5, 0.0 };
    s->check[1] = (spot_t){ 0.0, 52.0, -3.0 };
    s->check[2] = (spot_t){ 1.8, 95.2, 0.0 };

    /* the crate wall: three by three, in front of the ramp down */
    int n = 0;
    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++) {
            const double x = (col - 1) * 0.84, y = -3.0 - 0.4 - row * 0.8 - 0.02, z = 76.0;
            const ivp_material_t m = demo_material(0.6, 0.15);
            s->crate[n] = demo_create_box(w, &m, 0.4, 0.4, 0.4, 1.5, NULL, x, y, z);
            s->crate_start[n][0] = (float)x;
            s->crate_start[n][1] = (float)y;
            s->crate_start[n][2] = (float)z;
            n++;
        }

    /* the moving platform between A and B */
    s->mover_z0 = 44.0;
    s->mover_y = -3.0 + 0.3;
    ivp_object_template_t t;
    demo_template_dynamic(&t, &w->mat, 800.0);
    demo_set_box_inertia(&t, 800.0, 1.6, 0.3, 1.6);
    t.speed_damp_factor = IVP_REAL(0.0);
    t.rot_speed_damp_factor = ivp_vec3r_zero();
    demo_template_pose(&t, NULL, 0.0, s->mover_y, s->mover_z0);
    s->mover = demo_create_box_templ(w, &t, 1.6, 0.3, 1.6);
    ivp_golem_state_init(&s->golem);
    s->golem.max_translation_force = ivp_vec3r(IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    s->golem.max_torque = ivp_vec3r(IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    ivp_controller_handle_t h = ivp_environment_create_core_controller(s->env, IVP_CP_MOTION, ivp_controller_golem, &s->golem);
    ivp_environment_add_controller_to_core(s->env, h, s->mover);
    ivp_golem_state_setup(&s->golem, s->mover);
}

/* The platform swings between 1.6 m from A and 1.6 m from B. */
static double mover_z(double t) { return 44.0 + 2.4 * sin(0.8 * t); }

static void drive_mover(ballance_demo_t *s)
{
    const ivp_time_t now = ivp_environment_get_time(s->env);
    const double t = (double)now;
    const ivp_vec3r_t pp = ivp_vec3r(IVP_REAL(0.0), (ivp_real_t)s->mover_y, (ivp_real_t)mover_z(t)),
                      vv = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), (ivp_real_t)(2.4 * 0.8 * cos(0.8 * t)));
    ivp_golem_set_prime_position(&s->golem, &pp, &vv, now);
}

/* ── the balls ───────────────────────────────────────────────────────────── */

static void ball_friction_created(void *ctx, const ivp_friction_event_t *ev)
{
    (void)ev;
    ++*(int *)ctx;
}

static void ball_friction_deleted(void *ctx, const ivp_friction_event_t *ev)
{
    (void)ev;
    int *n = (int *)ctx;
    if (*n > 0) --*n;
}

static void park_ball(ballance_demo_t *s, int k)
{
    const ivp_quatr_t q = ivp_quatr_identity();
    const ivp_vec3r_t p = ivp_vec3r((ivp_real_t)(-60.0 + 6.0 * k), (ivp_real_t)(-kinds[k].radius - 0.02), IVP_REAL(0.0));
    ivp_environment_ensure_core_in_simulation(s->env, s->ball[k]);
    ivp_object_beam_to_new_position(s->ball[k], &q, &p, false);
    s->ball[k]->physical_core->speed = ivp_vec3r_zero();
    s->ball[k]->physical_core->rot_speed = ivp_vec3f(0.0f, 0.0f, 0.0f);
    s->contacts[k] = 0;
}

static void put_ball(ballance_demo_t *s, int k, const double pos[3], const ivp_vec3r_t *vel)
{
    const ivp_quatr_t q = ivp_quatr_identity();
    const ivp_vec3r_t p = ivp_vec3r((ivp_real_t)pos[0], (ivp_real_t)pos[1], (ivp_real_t)pos[2]);
    ivp_environment_ensure_core_in_simulation(s->env, s->ball[k]);
    ivp_object_beam_to_new_position(s->ball[k], &q, &p, false);
    s->ball[k]->physical_core->speed = vel ? *vel : ivp_vec3r_zero();
    s->ball[k]->physical_core->rot_speed = ivp_vec3f(0.0f, 0.0f, 0.0f);
    s->contacts[k] = 0;
}

static void ball_pos(const ballance_demo_t *s, float c[3])
{
    demo_object_center(s->ball[s->active], c);
}

static void say(ballance_demo_t *s, const char *text)
{
    snprintf(s->message, sizeof(s->message), "%s", text);
    s->message_until = s->time + 2.5;
}

/* Change the material of the ball under the player: the new ball takes over position and velocity. */
static void change_kind(ballance_demo_t *s, int k, double y_top)
{
    if (k == s->active) return;
    float c[3];
    ball_pos(s, c);
    const ivp_vec3r_t vel = s->ball[s->active]->physical_core->speed;
    const double pos[3] = { c[0], y_top - kinds[k].radius - 0.02, c[2] };
    park_ball(s, s->active);
    put_ball(s, k, pos, &vel);
    s->active = k;
    s->kinds_used |= 1u << k;
    s->transform_cooldown = 0.6;
    char text[48];
    snprintf(text, sizeof(text), "a %s ball", kinds[k].name);
    say(s, text);
}

static void respawn(ballance_demo_t *s)
{
    static const int stage_after[N_CHECK] = { 1, 6, 13 };      /* the autopilot goes on from the checkpoint */
    s->bot_stage = stage_after[s->checkpoint];
    const spot_t *c = &s->check[s->checkpoint];
    const double pos[3] = { c->x, c->y_top - kinds[s->active].radius - 0.02, c->z };
    put_ball(s, s->active, pos, NULL);
}

/* ── rolling ─────────────────────────────────────────────────────────────── */

/* Accelerate the ball along the world direction (dx, dz); its length 0..1 is the stick. */
static void roll(ballance_demo_t *s, float dx, float dz, float dt)
{
    const float mag = sqrtf(dx * dx + dz * dz);
    if (mag < 1e-4f) return;
    const kind_t *k = &kinds[s->active];
    const float ux = dx / (mag > 1.0f ? mag : 1.0f), uz = dz / (mag > 1.0f ? mag : 1.0f), m = fminf(mag, 1.0f);
    const ivp_vec3r_t *v = &s->ball[s->active]->physical_core->speed;
    const float along = (float)v->k[0] * ux / m + (float)v->k[2] * uz / m;
    const float cap = k->vmax * m;
    if (along >= cap) return;
    float a = k->accel * m * dt * (s->contacts[s->active] > 0 ? 1.0f : 0.25f);
    if (along + a > cap) a = cap - along;
    demo_async_add_speed_ws(s->env, s->ball[s->active], (double)(a * ux / m), 0.0, (double)(a * uz / m));
}

/* The arrow keys in the world: forward is where the camera looks f, and with up = -Y the right of it is
 * (f_z, -f_x): looking along +z the right is +x. */
static void keys_to_world(const float f[2], float throttle, float steer, float out[2])
{
    out[0] = f[0] * throttle + f[1] * steer;
    out[1] = f[1] * throttle - f[0] * steer;
}

/* the horizontal forward of the camera */
static void camera_forward(const demo_viewer_t *v, float f[2])
{
    float x = v->cam.target[0] - v->cam.eye[0], z = v->cam.target[2] - v->cam.eye[2];
    const float l = sqrtf(x * x + z * z);
    if (l < 1e-4f) { f[0] = 0.0f; f[1] = 1.0f; return; }
    f[0] = x / l;
    f[1] = z / l;
}

/* the camera turns into the direction the ball rolls */
static void chase_camera(ballance_demo_t *s, demo_viewer_t *v, const float c[3])
{
    const float dt = v->frame_dt > 0.1f ? 0.1f : v->frame_dt;
    const ivp_vec3r_t *sp = &s->ball[s->active]->physical_core->speed;
    const float vx = (float)sp->k[0], vz = (float)sp->k[2], vh = sqrtf(vx * vx + vz * vz);
    const float kf = 1.0f - expf(-dt * 4.0f);
    for (int i = 0; i < 3; i++) v->cam.target[i] += (c[i] - v->cam.target[i]) * kf;
    if (vh > 2.5f) {
        const float want = atan2f(-vx, -vz) * 180.0f / PI_F;
        float diff = fmodf(want - v->cam.orbit_yaw + 540.0f, 360.0f) - 180.0f;
        v->cam.orbit_yaw += diff * (1.0f - expf(-dt * 1.2f));
    }
}

/* ── the autopilot ───────────────────────────────────────────────────────── */

typedef struct waypoint {
    float x, z;
    float speed;       /* 0..1 of the stick on the way there */
    int   wait;        /* 0 go on, 1 leave when the platform is near A, 2 leave when it is near B */
} waypoint_t;

static const waypoint_t route[] = {
    { 0.0f, 2.5f, 1.0f, 0 },  { 0.0f, 20.0f, 1.0f, 0 },  { 0.0f, 31.0f, 1.0f, 0 },  { 0.0f, 36.0f, 0.8f, 0 },
    { 0.0f, 38.6f, 0.35f, 1 }, { 0.0f, 44.0f, 0.5f, 2 }, { 0.0f, 51.0f, 0.6f, 0 },  { 0.0f, 56.5f, 0.45f, 0 },
    { 0.0f, 69.0f, 0.5f, 0 },  { 0.0f, 72.5f, 0.8f, 0 }, { 0.0f, 79.0f, 1.0f, 0 },  { 0.0f, 89.0f, 0.7f, 0 },
    { 0.0f, 93.5f, 0.45f, 0 }, { 4.0f, 94.0f, 0.55f, 0 }, { 18.0f, 94.0f, 0.9f, 0 }, { 22.0f, 94.0f, 0.5f, 0 },
};
#define N_ROUTE ((int)(sizeof(route) / sizeof(route[0])))

/* The world direction the autopilot rolls in. */
static void bot_direction(ballance_demo_t *s, const float c[3], float out[2])
{
    out[0] = out[1] = 0.0f;
    if (s->bot_stage >= N_ROUTE) return;
    const waypoint_t *to = &route[s->bot_stage], *from = &route[s->bot_stage > 0 ? s->bot_stage - 1 : 0];
    const float mz = (float)mover_z(s->time);
    /* on the moving platform the aim is its middle */
    const float to_z = to->wait == 2 ? mz : to->z;
    const float dx = to->x - c[0], dz = to_z - c[2], dist = sqrtf(dx * dx + dz * dz);
    if (dist < (to->wait == 1 ? 0.5f : (to->wait == 2 ? 1.0f : 1.3f))) {
        bool go = true;
        if (to->wait == 1) go = mz < 41.9f;
        if (to->wait == 2) go = mz > 46.1f;
        if (go) {
            s->bot_stage++;
            return;
        }
        /* wait: stop the ball */
        const ivp_vec3r_t *v = &s->ball[s->active]->physical_core->speed;
        out[0] = -0.6f * (float)v->k[0];
        out[1] = -0.6f * (float)v->k[2];
        return;
    }
    /* stay on the line from the last point: aim at a point a little ahead of the ball on it */
    float lx = to->x - from->x, lz = to->z - from->z;
    const float ll = sqrtf(lx * lx + lz * lz);
    float tx = to->x, tz = to_z;
    if (ll > 1e-3f && to->wait != 2) {
        lx /= ll;
        lz /= ll;
        float along = (c[0] - from->x) * lx + (c[2] - from->z) * lz;
        if (along < 0.0f) along = 0.0f;
        const float ahead = fminf(ll, along + 2.0f);
        tx = from->x + lx * ahead;
        tz = from->z + lz * ahead;
    }
    float ax = tx - c[0], az = tz - c[2];
    const float al = sqrtf(ax * ax + az * az);
    if (al > 1e-4f) {
        ax /= al;
        az /= al;
    }
    out[0] = ax * to->speed;
    out[1] = az * to->speed;
}

/* ── the game ────────────────────────────────────────────────────────────── */

static void restart_game(ballance_demo_t *s)
{
    s->active = WOOD;
    s->checkpoint = 0;
    s->lives = LIVES;
    s->time = 0.0;
    s->done = false;
    s->done_time = 0.0;
    s->falls = 0;
    s->kinds_used = 1u << WOOD;
    s->bot_stage = 0;
    s->max_speed = 0.0f;
    s->mover_err = 0.0f;
    s->message[0] = 0;
    s->transform_cooldown = 0.0;
    for (int i = 0; i < N_KINDS; i++) s->contacts[i] = 0;
    ivp_golem_state_setup(&s->golem, s->mover);
}

static void ballance_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    ballance_demo_t *s = (ballance_demo_t *)app->user;
    (void)frame;
    const float dt = v ? (v->frame_dt > 0.1f ? 0.1f : v->frame_dt) : (float)DEMO_PSI_TIME;
    drive_mover(s);
    {
        float mc[3];
        demo_object_center(s->mover, mc);
        const float e = fabsf(mc[2] - (float)s->golem.target_pos_ws.k[2]);
        if (s->time > 1.0 && e > s->mover_err) s->mover_err = e;
    }

    float c[3];
    ball_pos(s, c);
    if (!s->done) s->time += (double)dt;
    s->transform_cooldown -= (double)dt;
    const float sp = demo_object_speed(s->ball[s->active]);
    if (!s->done && sp > s->max_speed && c[1] < FALL_Y) s->max_speed = sp;
    if (v && demo_key_just_pressed(v, SDL_SCANCODE_P)) {
        s->autopilot = !s->autopilot;
        s->bot_stage = s->bot_stage > 0 ? s->bot_stage : 1;
        say(s, s->autopilot ? "autopilot" : "you drive");
    }

    /* input */
    float dir[2] = { 0.0f, 0.0f };
    if (!s->done) {
        if (v && !s->autopilot) {
            demo_drive_input_t in;
            demo_drive_read(&s->drive, &in);
            demo_drive_step(&s->drive, &in, dt);
            float f[2];
            camera_forward(v, f);
            keys_to_world(f, s->drive.throttle, s->drive.steer, dir);
        } else {
            bot_direction(s, c, dir);
        }
        if (v && demo_key_just_pressed(v, SDL_SCANCODE_BACKSPACE)) respawn(s);
        roll(s, dir[0], dir[1], dt);
    }

    /* pads */
    if (!s->done && s->transform_cooldown <= 0.0)
        for (int i = 0; i < N_PADS; i++) {
            const pad_t *p = &s->pad[i];
            if (fabsf(c[0] - (float)p->x) < 1.1f && fabsf(c[2] - (float)p->z) < 1.1f && c[1] > (float)p->y_top - 1.6f &&
                c[1] < (float)p->y_top + 0.2f) {
                change_kind(s, p->kind, p->y_top);
                break;
            }
        }
    /* checkpoints */
    for (int i = s->checkpoint + 1; i < N_CHECK; i++)
        if (fabsf(c[0] - (float)s->check[i].x) < 2.6f && fabsf(c[2] - (float)s->check[i].z) < 2.6f && c[1] < (float)s->check[i].y_top) {
            s->checkpoint = i;
            say(s, "checkpoint");
        }
    /* the goal */
    if (!s->done && fabsf(c[0] - 22.0f) < 2.5f && fabsf(c[2] - 94.0f) < 3.0f && c[1] < 0.0f) {
        s->done = true;
        s->done_time = s->time;
        say(s, "the flag!");
    }
    /* crates that fall off the course are put in the yard with the balls */
    for (int i = 0; i < N_CRATES; i++) {
        float cp[3];
        demo_object_center(s->crate[i], cp);
        if (cp[1] > 20.0f) {
            const ivp_quatr_t q = ivp_quatr_identity();
            const ivp_vec3r_t p = ivp_vec3r((ivp_real_t)(-66.0 + 1.0 * i), (ivp_real_t)-0.42, IVP_REAL(3.0));
            ivp_environment_ensure_core_in_simulation(s->env, s->crate[i]);
            ivp_object_beam_to_new_position(s->crate[i], &q, &p, false);
            s->crate[i]->physical_core->speed = ivp_vec3r_zero();
            s->crate[i]->physical_core->rot_speed = ivp_vec3f(0.0f, 0.0f, 0.0f);
        }
    }
    /* after the flag the ball that rolls off comes back to the goal */
    if (s->done && c[1] > FALL_Y) {
        const double pos[3] = { 22.0, -kinds[s->active].radius - 0.02, 94.0 };
        put_ball(s, s->active, pos, NULL);
    }
    /* falling */
    if (c[1] > FALL_Y && !s->done) {
        s->falls++;
        if (--s->lives <= 0) {
            s->lives = LIVES;
            s->checkpoint = 0;
            say(s, "out of lives: from the start");
        } else {
            say(s, "you fell");
        }
        respawn(s);
    }
    if (v) chase_camera(s, v, c);
}

static void ballance_reset(demo_app_t *app)
{
    restart_game((ballance_demo_t *)app->user);
}

static void ballance_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    ballance_demo_t *s = (ballance_demo_t *)app->user;
    s->drive.throttle_time = 0.12f;
    s->drive.steer_time = 0.12f;
    s->drive.center_time = 0.10f;
    demo_drive_panel(&v->panel, &s->drive);
}

/* ── drawing ─────────────────────────────────────────────────────────────── */

static void ballance_draw(demo_app_t *app, demo_viewer_t *v)
{
    ballance_demo_t *s = (ballance_demo_t *)app->user;
    for (int i = 0; i < s->n_pieces; i++) demo_draw_object(v, s->piece[i].obj, s->piece[i].col);
    const float c_crate[3] = { 0.92f, 0.60f, 0.25f }, c_mover[3] = { 0.40f, 0.65f, 0.95f };
    for (int i = 0; i < N_CRATES; i++) demo_draw_object(v, s->crate[i], c_crate);
    demo_draw_object(v, s->mover, c_mover);

    /* pads: a coloured plate and a light above it */
    for (int i = 0; i < N_PADS; i++) {
        const pad_t *p = &s->pad[i];
        const float pos[3] = { (float)p->x, (float)p->y_top - 0.02f, (float)p->z }, half[3] = { 0.9f, 0.025f, 0.9f };
        demo_solid_box(v, pos, half, NULL, kinds[p->kind].col);
        const float top[3] = { pos[0], pos[1] - 2.2f, pos[2] };
        demo_set_alpha(v, 0.35f);
        const float beam_half[3] = { 0.5f, 1.1f, 0.5f }, beam_pos[3] = { pos[0], pos[1] - 1.1f, pos[2] };
        demo_solid_box(v, beam_pos, beam_half, NULL, kinds[p->kind].col);
        demo_set_alpha(v, 1.0f);
        (void)top;
    }
    for (int i = 0; i < N_CHECK; i++) {
        if (i == 0) continue;
        const spot_t *c = &s->check[i];
        const float pos[3] = { (float)c->x, (float)c->y_top - 0.02f, (float)c->z }, half[3] = { 0.9f, 0.025f, 0.9f };
        const float col[3] = { i <= s->checkpoint ? 0.30f : 0.20f, i <= s->checkpoint ? 0.95f : 0.55f, 0.35f };
        demo_solid_box(v, pos, half, NULL, col);
    }

    /* the ball; three darker spots show how it turns */
    const kind_t *k = &kinds[s->active];
    float pos[3], rot[9];
    demo_object_pose(s->ball[s->active], pos, rot);
    demo_solid_sphere(v, pos, (float)k->radius, k->col);
    for (int a = 0; a < 3; a++) {
        const float d = (float)k->radius * 0.86f, sp[3] = { pos[0] + rot[a * 3] * d, pos[1] + rot[a * 3 + 1] * d, pos[2] + rot[a * 3 + 2] * d };
        demo_solid_sphere(v, sp, (float)k->radius * 0.28f, k->spot);
        const float sq[3] = { pos[0] - rot[a * 3] * d, pos[1] - rot[a * 3 + 1] * d, pos[2] - rot[a * 3 + 2] * d };
        demo_solid_sphere(v, sq, (float)k->radius * 0.28f, k->spot);
    }

    char lives[16] = "";
    for (int i = 0; i < s->lives && i < LIVES; i++) strcat(lives, "o ");
    demo_hud(v, DEMO_COL_TITLE, "%s ball   lives %s  time %.1f s   speed %.1f m/s", k->name, lives, s->time,
             (double)demo_object_speed(s->ball[s->active]));
    demo_hud(v, DEMO_COL_INFO, "checkpoint %d of %d   pads: paper -> stone -> wood", s->checkpoint, N_CHECK - 1);
    if (s->done) demo_hud(v, DEMO_COL_HIGHLIGHT, "COURSE COMPLETE in %.1f s, %d falls", s->done_time, s->falls);
    else if (s->time < s->message_until) demo_hud(v, DEMO_COL_HIGHLIGHT, "%s", s->message);
}

static void ballance_stats(demo_app_t *app, char *buf, size_t size)
{
    ballance_demo_t *s = (ballance_demo_t *)app->user;
    float c[3];
    ball_pos(s, c);
    snprintf(buf, size, "%s ball (%.1f %.1f %.1f) stage %d/%d cp %d lives %d%s", kinds[s->active].name, (double)c[0], (double)c[1],
             (double)c[2], s->bot_stage, N_ROUTE, s->checkpoint, s->lives, s->done ? " DONE" : "");
}

static bool ballance_check(demo_app_t *app)
{
    ballance_demo_t *s = (ballance_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, s->done, "the autopilot reached the flag (stage %d of %d)", s->bot_stage, N_ROUTE);
    ok &= demo_check(tag, s->done && s->done_time < 80.0, "in %.1f s", s->done_time);
    ok &= demo_check(tag, s->falls == 0, "without falling (%d falls)", s->falls);
    ok &= demo_check(tag, s->kinds_used == 7u, "and it used all three materials (mask %u)", s->kinds_used);
    ok &= demo_check(tag, s->checkpoint == N_CHECK - 1, "and the last checkpoint (%d)", s->checkpoint);
    int moved = 0;
    for (int i = 0; i < N_CRATES; i++) {
        float p[3];
        demo_object_center(s->crate[i], p);
        if (demo_distance(p, s->crate_start[i]) > 0.6f) moved++;
    }
    ok &= demo_check(tag, moved >= 3, "the stone ball pushed %d of %d crates aside", moved, N_CRATES);
    ok &= demo_check(tag, s->mover_err < 0.4f, "the moving platform kept to its path (worst %.2f m)", (double)s->mover_err);
    {
        const float fz[2] = { 0.0f, 1.0f }, fx[2] = { 1.0f, 0.0f };
        float a[2], b[2], c[2];
        keys_to_world(fz, 1.0f, 0.0f, a);              /* up while looking along +z rolls along +z */
        keys_to_world(fz, 0.0f, 1.0f, b);              /* right rolls along +x (the flag side) */
        keys_to_world(fx, 0.0f, 1.0f, c);              /* looking along +x, right is -z */
        ok &= demo_check(tag, a[1] > 0.99f && fabsf(a[0]) < 1e-5f && b[0] > 0.99f && fabsf(b[1]) < 1e-5f && c[1] < -0.99f,
                         "the arrow keys roll relative to the camera");
    }
    ok &= demo_check(tag, s->max_speed > 4.0f, "top speed %.1f m/s", (double)s->max_speed);
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Ballance -- Roll the Ball";
    app.cfg.tag         = "ballance";
    app.cfg.friction    = 0.9;
    app.cfg.elasticity  = 0.2;
    app.cfg.orbit_dist  = 9.0f;
    app.cfg.orbit_pitch = -28.0f;
    app.cfg.orbit_yaw   = 180.0f;
    app.cfg.target[0]   = 0.0f;
    app.cfg.target[1]   = -0.5f;
    app.cfg.target[2]   = 2.5f;
    app.cfg.grid_extent = -1.0f;
    app.cfg.headless_frames = 3600;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    ballance_demo_t *s = (ballance_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->env = w->env;
    demo_drive_defaults(&s->drive);
    s->drive.throttle_time = 0.12f;
    s->drive.steer_time = 0.12f;
    s->drive.center_time = 0.10f;

    build_course(s, w);

    /* a place to keep the balls that are not in play */
    const float grey[3] = { 0.3f, 0.3f, 0.3f };
    (void)grey;
    s->park = demo_create_box(w, NULL, 16.0, 0.5, 4.0, 0.0, NULL, -54.0, 0.5, 0.0);
    /* with a wall round it: a parked ball that creeps stays on the floor */
    demo_create_box(w, NULL, 0.3, 1.0, 4.3, 0.0, NULL, -70.3, -1.0, 0.0);
    demo_create_box(w, NULL, 0.3, 1.0, 4.3, 0.0, NULL, -37.7, -1.0, 0.0);
    demo_create_box(w, NULL, 16.3, 1.0, 0.3, 0.0, NULL, -54.0, -1.0, 4.3);
    demo_create_box(w, NULL, 16.3, 1.0, 0.3, 0.0, NULL, -54.0, -1.0, -4.3);
    for (int k = 0; k < N_KINDS; k++) {
        const ivp_material_t m = demo_material(kinds[k].mu, kinds[k].elasticity);
        s->ball[k] = demo_create_ball(w, &m, kinds[k].radius, kinds[k].mass, NULL, -60.0 + 6.0 * k, -kinds[k].radius - 0.02, 0.0);
        if (!s->ball[k]) return 1;
        ivp_collision_listener_init(&s->lst[k], IVP_COLLISION_CB_FRICTION_CREATED | IVP_COLLISION_CB_FRICTION_DELETED, &s->contacts[k]);
        s->lst[k].friction_created_fn = ball_friction_created;
        s->lst[k].friction_deleted_fn = ball_friction_deleted;
        ivp_object_add_collision_listener(s->ball[k], &s->lst[k]);
    }
    restart_game(s);
    respawn(s);
    s->autopilot = getenv("IVP_BALLANCE_AUTOPILOT") != NULL;

    app.controls    = "Arrows: roll   Backspace: last checkpoint   P: autopilot   RMB: look around";
    app.viewer_init = ballance_viewer_init;
    app.update      = ballance_update;
    app.on_reset    = ballance_reset;
    app.draw        = ballance_draw;
    app.stats       = ballance_stats;
    app.check       = ballance_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
