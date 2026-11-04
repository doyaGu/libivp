/* example_rolling_ball.c -- Roll a ball along a course of planks
 *
 * After the game of that name: you roll a ball over wooden planks, ramps, a
 * moving platform, a narrow beam and a wall of crates, and the ball changes its
 * material on the way. All of it is rigid body physics; there is no jump.
 *
 *   wood    the ball you start with: the gap and the beam are for it
 *   stone   heavy: slower to start and to stop than the others, the only one that can push the crates aside
 *   paper   a feather: starts and stops at once, for the run to the flag
 *
 * The balls are modelled on the classic game of the kind (see kinds[] below).
 *
 * Roll over a coloured pad to change. A red flag is a checkpoint you have not reached, a green
 * one you have: if you fall off the course you start again from the last green one,
 * with the material you had there, and lose a life; three lives and the course starts over. The goal is the platform with the flag.
 *
 *   Up / Down / Left / Right   roll, relative to where the camera looks
 *   Backspace                  back to the last checkpoint
 *   P                          the autopilot rolls for you (the one of the self-check)
 *   (gamepad)                  left stick rolls
 *   The camera follows the ball and turns with the course (not with the ball, so a
 *   swerve does not turn the controls); hold the right mouse button to look around.
 *
 * Headless, an autopilot rolls the whole course. It presses the arrow keys like a player: on or off, a
 * little late, easing off before a gap and waiting for the moving platform. The self-check requires it to
 * reach the flag without falling, and a ball knocked off at a checkpoint to come back to it.
 * IVP_ROLLING_BALL_HUMAN=<n> adds a player's slips (n picks them); nearly every n finishes.
 *
 * Demonstrates: static boxes as a level (ramps by quaternion), a golem driven
 *               platform, dynamic crates, one ball per material that is swapped
 *               by beaming (ivp_object_beam_to_new_position), a chase camera.
 */
#include "demo_app.h"
#include "demo_drive.h"

#include <ivp/controllers/ivp_controller_golem.h>

#define N_KINDS    3
#define MAX_PIECES 40
#define N_CRATES   12
#define N_PADS     2
#define N_CHECK    5
#define FALL_Y     7.0f
#define LIVES      3
#define PI_F       3.14159265f
#define MOVER_Z0   43.0        /* the moving platform swings about this z ... */
#define MOVER_AMP  1.2         /* ... by this much ... */
#define MOVER_W    0.6         /* ... at this angular speed */

enum { WOOD, STONE, PAPER };

typedef struct kind {
    const char *name;
    double      radius, mass, mu, elasticity;
    double      damp;           /* the engine's speed damping per second */
    float       top;            /* m/s the ball ends up at with a key held: push / damping */
    float       col[3], spot[3];
} kind_t;

/* The balls of the classic ball-rolling game this is modelled on (GamePhysBallData of its Unity remake, which
 * runs the original IVP) have mass 1.9 / 10 / 0.2, friction 0.8 / 0.5 / 0.5, elasticity 0.2 / 0.1 / 0.4 and
 * linear damping 0.9 / 0.3 / 1.3, and are pushed by a constant force: no speed limit, no brake; the ball
 * settles at push / damping. Mass, friction and elasticity are kept. The damping is not: with the game's
 * values a wood ball needs a second to get going and another to stop, which on an arrow key was dull, so
 * all three balls respond about two and a half times faster (damping 2.2 / 0.9 / 3.0, a time constant of 0.45 s,
 * 1.1 s and 0.33 s) with their end speeds in about the proportions of the game (1 : 1.13 : 1.1). The balls are
 * 0.9 m across. The push is the end speed times the damping. */
static const kind_t kinds[N_KINDS] = {
    { "wood",  0.45, 1.9, 0.8, 0.2, 2.2, 5.5f, { 0.80f, 0.58f, 0.34f }, { 0.55f, 0.36f, 0.18f } },
    { "stone", 0.50, 10.0, 0.5, 0.1, 0.9, 6.2f, { 0.62f, 0.63f, 0.66f }, { 0.38f, 0.39f, 0.42f } },
    { "paper", 0.40, 0.2, 0.5, 0.4, 3.0, 6.0f, { 0.95f, 0.95f, 0.90f }, { 0.70f, 0.78f, 0.90f } },
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

typedef struct rolling_ball_demo {
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
    int                      active;
    pad_t                    pad[N_PADS];
    spot_t                   check[N_CHECK];
    int                      checkpoint;
    int                      cp_kind[N_CHECK];      /* the material the ball had there */
    unsigned                 human_seed;            /* IVP_ROLLING_BALL_HUMAN: a keyboard player model */
    unsigned                 rng;
    unsigned char            key_queue[16];         /* its keys, delayed by its reaction time */
    int                      key_head;
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
    bool                     fall_test_started, fall_test_done;     /* headless: knock the ball off at a checkpoint */
    float                    fall_test_dist;
    int                      fall_test_kind, fall_test_checkpoint;
    float                    push_scale, damp_scale;    /* F5: the push and the damping of every ball */
    float                    mover_err;
    int                      mover_problems;
    char                     message[80];
    double                   message_until;
} rolling_ball_demo_t;

/* ── the course ──────────────────────────────────────────────────────────── */

static ivp_object_t *add_piece(rolling_ball_demo_t *s, demo_world_t *w, double hx, double hy, double hz, double cx, double cy,
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
static void plat(rolling_ball_demo_t *s, demo_world_t *w, double cx, double z0, double z1, double hx, double y_top, const float col[3])
{
    add_piece(s, w, hx, 0.3, 0.5 * (z1 - z0), cx, y_top + 0.3, 0.5 * (z0 + z1), NULL, col);
}

/* A ramp whose top runs from (z0, y0) to (z1, y1); rising along +z when y1 < y0 (up is -Y). */
static void ramp(rolling_ball_demo_t *s, demo_world_t *w, double cx, double z0, double y0, double z1, double y1, double hx,
                 const float col[3])
{
    const double dz = z1 - z0, dy = y1 - y0, len = sqrt(dz * dz + dy * dy), ang = atan2(-dy, dz);
    const ivp_quatr_t q = ivp_quatr_make(sin(0.5 * ang), 0.0, 0.0, cos(0.5 * ang));
    /* the top surface passes through the middle of the run; the slab is 0.3 below it, along its normal */
    const double mz = 0.5 * (z0 + z1), my = 0.5 * (y0 + y1), cy = my + 0.3 * cos(ang), cz = mz + 0.3 * sin(ang);
    add_piece(s, w, hx, 0.3, 0.5 * len, cx, cy, cz, &q, col);
}

static void build_course(rolling_ball_demo_t *s, demo_world_t *w)
{
    const float wood[3] = { 0.72f, 0.55f, 0.34f }, dark[3] = { 0.55f, 0.40f, 0.25f }, stone[3] = { 0.55f, 0.57f, 0.62f },
                beam[3] = { 0.62f, 0.45f, 0.28f };
    plat(s, w, 0.0, 0.0, 8.0, 4.0, 0.0, wood);                  /* start */
    plat(s, w, 0.0, 8.0, 22.0, 1.4, 0.0, dark);                 /* plank */
    ramp(s, w, 0.0, 22.0, 0.0, 32.0, -3.0, 1.4, wood);          /* up */
    plat(s, w, 0.0, 32.0, 40.0, 4.0, -3.0, wood);               /* platform A: checkpoint */
    plat(s, w, 0.0, 46.0, 56.0, 4.0, -3.0, wood);               /* platform B, beyond the gap: checkpoint */
    plat(s, w, 0.0, 56.0, 70.0, 0.75, -3.0, beam);               /* the beam */
    plat(s, w, 0.0, 70.0, 80.0, 3.0, -3.0, stone);              /* platform C: checkpoint, stone pad, the crates */
    ramp(s, w, 0.0, 80.0, -3.0, 90.0, 0.0, 1.4, wood);          /* down */
    plat(s, w, 0.0, 90.0, 98.0, 4.0, 0.0, wood);                /* platform D: paper pad, checkpoint */
    plat(s, w, 11.0, 92.6, 95.4, 7.0, 0.0, dark);               /* the plank to the goal */
    plat(s, w, 22.0, 90.0, 98.0, 4.0, 0.0, stone);              /* the goal */
    /* low rails along the two planks keep a ball that swerves on them */
    const float rail[3] = { 0.45f, 0.33f, 0.20f };
    add_piece(s, w, 0.1, 0.3, 7.0, -1.5, -0.3, 15.0, NULL, rail);
    add_piece(s, w, 0.1, 0.3, 7.0, 1.5, -0.3, 15.0, NULL, rail);
    add_piece(s, w, 7.0, 0.3, 0.1, 11.0, -0.3, 92.4, NULL, rail);
    add_piece(s, w, 7.0, 0.3, 0.1, 11.0, -0.3, 95.6, NULL, rail);
    /* a post and a flag on the goal */
    const float red[3] = { 0.90f, 0.25f, 0.20f };
    add_piece(s, w, 0.06, 1.6, 0.06, 24.0, -1.6, 94.0, NULL, stone);
    add_piece(s, w, 0.04, 0.4, 0.6, 24.0, -2.8, 94.6, NULL, red);

    s->pad[0] = (pad_t){ 0.0, 72.5, -3.0, STONE };
    s->pad[1] = (pad_t){ 0.0, 93.5, 0.0, PAPER };
    s->check[0] = (spot_t){ 0.0, 2.5, 0.0 };
    s->check[1] = (spot_t){ 0.0, 33.8, -3.0 };
    s->check[2] = (spot_t){ 0.0, 52.0, -3.0 };
    s->check[3] = (spot_t){ 0.0, 70.8, -3.0 };
    s->check[4] = (spot_t){ 1.8, 95.2, 0.0 };

    /* the crate wall: six wide and two high, from edge to edge of platform C */
    int n = 0;
    for (int row = 0; row < 2; row++)
        for (int col = 0; col < 6; col++) {
            const double x = (col - 2.5) * 0.84, y = -3.0 - 0.4 - row * 0.8 - 0.02, z = 76.0;
            const ivp_material_t m = demo_material(0.35, 0.15);
            s->crate[n] = demo_create_box(w, &m, 0.4, 0.4, 0.4, 0.8, NULL, x, y, z);
            s->crate_start[n][0] = (float)x;
            s->crate_start[n][1] = (float)y;
            s->crate_start[n][2] = (float)z;
            n++;
        }

    /* the moving platform between A and B */
    s->mover_z0 = MOVER_Z0;
    s->mover_y = -3.0 + 0.3;
    ivp_object_template_t t;
    demo_template_dynamic(&t, &w->mat, 800.0);
    demo_set_box_inertia(&t, 800.0, 2.4, 0.3, 1.8);
    t.speed_damp_factor = IVP_REAL(0.0);
    t.rot_speed_damp_factor = ivp_vec3r_zero();
    demo_template_pose(&t, NULL, 0.0, s->mover_y, s->mover_z0);
    s->mover = demo_create_box_templ(w, &t, 2.4, 0.3, 1.8);
    ivp_golem_state_init(&s->golem);
    s->golem.max_translation_force = ivp_vec3r(IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    s->golem.max_torque = ivp_vec3r(IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    ivp_controller_handle_t h = ivp_environment_create_core_controller(s->env, IVP_CP_MOTION, ivp_controller_golem, &s->golem);
    ivp_environment_add_controller_to_core(s->env, h, s->mover);
    ivp_golem_state_setup(&s->golem, s->mover);
}

/* The platform (3.6 m long) swings between touching A and touching B, 6 m apart. */
static double mover_z(double t) { return MOVER_Z0 + MOVER_AMP * sin(MOVER_W * t); }
static double mover_vz(double t) { return MOVER_AMP * MOVER_W * cos(MOVER_W * t); }

static void drive_mover(rolling_ball_demo_t *s)
{
    const ivp_time_t now = ivp_environment_get_time(s->env);
    const double t = (double)now;
    const ivp_vec3r_t pp = ivp_vec3r(IVP_REAL(0.0), (ivp_real_t)s->mover_y, (ivp_real_t)mover_z(t)),
                      vv = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), (ivp_real_t)mover_vz(t));
    ivp_golem_set_prime_position(&s->golem, &pp, &vv, now);
}

/* ── the balls ───────────────────────────────────────────────────────────── */

static void park_ball(rolling_ball_demo_t *s, int k)
{
    const ivp_quatr_t q = ivp_quatr_identity();
    const ivp_vec3r_t p = ivp_vec3r((ivp_real_t)(-60.0 + 6.0 * k), (ivp_real_t)(-kinds[k].radius - 0.02), IVP_REAL(0.0));
    ivp_environment_ensure_core_in_simulation(s->env, s->ball[k]);
    ivp_object_beam_to_new_position(s->ball[k], &q, &p, false);
    s->ball[k]->physical_core->speed = ivp_vec3r_zero();
    s->ball[k]->physical_core->rot_speed = ivp_vec3f(0.0f, 0.0f, 0.0f);
}

static void put_ball(rolling_ball_demo_t *s, int k, const double pos[3], const ivp_vec3r_t *vel)
{
    const ivp_quatr_t q = ivp_quatr_identity();
    const ivp_vec3r_t p = ivp_vec3r((ivp_real_t)pos[0], (ivp_real_t)pos[1], (ivp_real_t)pos[2]);
    ivp_environment_ensure_core_in_simulation(s->env, s->ball[k]);
    ivp_object_beam_to_new_position(s->ball[k], &q, &p, false);
    s->ball[k]->physical_core->speed = vel ? *vel : ivp_vec3r_zero();
    s->ball[k]->physical_core->rot_speed = ivp_vec3f(0.0f, 0.0f, 0.0f);
}

static void ball_pos(const rolling_ball_demo_t *s, float c[3])
{
    demo_object_center(s->ball[s->active], c);
}

static void say(rolling_ball_demo_t *s, const char *text)
{
    snprintf(s->message, sizeof(s->message), "%s", text);
    s->message_until = s->time + 2.5;
}

/* Change the material of the ball under the player: the new ball takes over position and velocity. */
static void change_kind(rolling_ball_demo_t *s, int k, double y_top)
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

/* Put the ball back on the last checkpoint, with the material it had there. */
static void respawn(rolling_ball_demo_t *s)
{
    static const int stage_after[N_CHECK] = { 1, 3, 6, 9, 13 };      /* the autopilot goes on from the checkpoint */
    s->bot_stage = stage_after[s->checkpoint];
    const spot_t *c = &s->check[s->checkpoint];
    const int want = s->cp_kind[s->checkpoint];
    if (want != s->active) {
        park_ball(s, s->active);
        s->active = want;
    }
    const double pos[3] = { c->x, c->y_top - kinds[s->active].radius - 0.02, c->z };
    put_ball(s, s->active, pos, NULL);
}

/* ── rolling ─────────────────────────────────────────────────────────────── */

/* A ball on the moving platform is steered relative to it: it is the platform's speed it must not leave behind. */
static float platform_speed_under(const rolling_ball_demo_t *s)
{
    float c[3];
    demo_object_center(s->ball[s->active], c);
    const float mz = (float)mover_z(s->time);
    if (fabsf(c[0]) < 2.4f && fabsf(c[2] - mz) < 1.8f && c[1] < (float)s->mover_y && c[1] > (float)s->mover_y - 1.5f)
        return (float)mover_vz(s->time);
    return 0.0f;
}

/* Push the ball along the world direction (dx, dz); its length 0..1 is the stick. This is all the control
 * there is, as in the game: a constant push, and the damping decides how fast the ball ends up. */
static void roll(rolling_ball_demo_t *s, float dx, float dz, float dt)
{
    const float mag = sqrtf(dx * dx + dz * dz);
    if (mag < 1e-4f) return;
    const kind_t *k = &kinds[s->active];
    const float m = fminf(mag, 1.0f), a = (float)(k->top * k->damp) * s->push_scale * m * dt;
    demo_async_add_speed_ws(s->env, s->ball[s->active], (double)(a * dx / mag), 0.0, (double)(a * dz / mag));
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

/* Where the course goes at this point: along +z, and round the corner to +x at the end. The camera
 * turns with the course, not with the ball, so a swerve of the ball does not turn the controls. */
static void course_forward(const float c[3], float f[2])
{
    if (c[2] < 90.0f) {
        f[0] = 0.0f;
        f[1] = 1.0f;
        return;
    }
    float a = c[0] / 6.0f;
    a = a < 0.0f ? 0.0f : (a > 1.0f ? 1.0f : a);
    f[0] = sinf(a * 0.5f * PI_F);
    f[1] = cosf(a * 0.5f * PI_F);
}

static void chase_camera(rolling_ball_demo_t *s, demo_viewer_t *v, const float c[3])
{
    (void)s;
    const float dt = v->frame_dt > 0.1f ? 0.1f : v->frame_dt;
    const float kf = 1.0f - expf(-dt * 6.0f);
    for (int i = 0; i < 3; i++) v->cam.target[i] += (c[i] - v->cam.target[i]) * kf;
    float mx, my;
    const bool looking = (SDL_GetMouseState(&mx, &my) & SDL_BUTTON_RMASK) != 0;
    if (!looking) {
        float f[2];
        course_forward(c, f);
        const float want = atan2f(-f[0], -f[1]) * 180.0f / PI_F;
        const float diff = fmodf(want - v->cam.orbit_yaw + 540.0f, 360.0f) - 180.0f;
        v->cam.orbit_yaw += diff * (1.0f - expf(-dt * 2.5f));
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
    { 0.0f, 38.9f, 0.35f, 1 }, { 0.0f, 43.0f, 0.5f, 2 }, { 0.0f, 51.0f, 0.6f, 0 },  { 0.0f, 56.5f, 0.45f, 0 },
    { 0.0f, 69.0f, 0.5f, 0 },  { 0.0f, 72.5f, 0.8f, 0 }, { 0.0f, 79.0f, 1.0f, 0 },  { 0.0f, 89.0f, 0.7f, 0 },
    { 0.0f, 93.5f, 0.45f, 0 }, { 4.0f, 94.0f, 0.55f, 0 }, { 18.0f, 94.0f, 0.9f, 0 }, { 22.0f, 94.0f, 0.5f, 0 },
};
#define N_ROUTE ((int)(sizeof(route) / sizeof(route[0])))

/* The world direction the autopilot rolls in. */
static void bot_direction(rolling_ball_demo_t *s, const float c[3], float out[2])
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
        if (to->wait == 1) go = mz < 42.0f;
        if (to->wait == 2) go = mz > 43.9f;
        if (go) {
            s->bot_stage++;
            return;
        }
        /* wait: stop the ball */
        const ivp_vec3r_t *v = &s->ball[s->active]->physical_core->speed;
        out[0] = -0.6f * (float)v->k[0];
        out[1] = -0.6f * ((float)v->k[2] - platform_speed_under(s));
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
    float al = sqrtf(ax * ax + az * az);
    if (al > 1e-4f) {
        ax /= al;
        az /= al;
    }
    /* and take the sideways speed off, as a person leans against a drift */
    if (ll > 1e-3f && to->wait != 2) {
        const ivp_vec3r_t *vel = &s->ball[s->active]->physical_core->speed;
        const float nx = -lz, nz = lx, vlat = (float)vel->k[0] * nx + (float)vel->k[2] * nz;
        const float off = (c[0] - from->x) * nx + (c[2] - from->z) * nz;           /* how far off the line */
        ax -= nx * (0.5f * vlat + 0.35f * off);
        az -= nz * (0.5f * vlat + 0.35f * off);
        al = sqrtf(ax * ax + az * az);
        if (al > 1e-4f) {
            ax /= al;
            az /= al;
        }
    }
    out[0] = ax * to->speed;
    out[1] = az * to->speed;
}

/* ── a player at the keyboard ───────────────────────────────────────────── */

static unsigned rand_u(rolling_ball_demo_t *s)
{
    s->rng ^= s->rng << 13;
    s->rng ^= s->rng >> 17;
    s->rng ^= s->rng << 5;
    return s->rng;
}

static float rand01(rolling_ball_demo_t *s) { return (float)(rand_u(s) & 0xFFFFu) / 65536.0f; }

/* The autopilot's wish turned into what a person with four arrow keys does: keys are on or off (a half
 * stick is a tap every so often), the eye sees the ball half a second late and the finger another
 * tenth, and now and then the wrong key goes down. Used to test whether the course can be played. */
static void human_keys(rolling_ball_demo_t *s, const float want[2], const float f[2], float *throttle, float *steer)
{
    const float r[2] = { f[1], -f[0] };
    const float mag = sqrtf(want[0] * want[0] + want[1] * want[1]);
    const kind_t *k = &kinds[s->active];
    const ivp_vec3r_t *v = &s->ball[s->active]->physical_core->speed;
    const float vx = (float)v->k[0], vz = (float)v->k[2] - platform_speed_under(s);
    /* the velocity the player wants and the difference to what the ball does; the keys are bang-bang on it */
    float wx = 0.0f, wz = 0.0f;
    if (mag > 1e-3f) {
        const float sp = (mag > 1.0f ? 1.0f : mag) * k->top * s->push_scale;
        wx = want[0] / mag * sp;
        wz = want[1] / mag * sp;
    }
    const float ex = wx - vx, ez = wz - vz;
    const float ef = ex * f[0] + ez * f[1], es = ex * r[0] + ez * r[1];
    unsigned keys = 0;                                         /* bit 0 up, 1 down, 2 right, 3 left */
    if (ef > 0.8f) keys |= 1u;
    if (ef < -1.2f) keys |= 2u;
    if (es > 0.6f) keys |= 4u;
    if (es < -0.6f) keys |= 8u;
    if (s->human_seed && rand01(s) < 0.01f) keys ^= (rand_u(s) & 1u) ? 4u : 8u;          /* a slip of the finger */
    /* a finger does not flicker: the decision holds for 8 frames */
    if (s->key_head % 8 != 0) keys = s->key_queue[(s->key_head + 15) % 16];
    s->key_queue[s->key_head] = (unsigned char)keys;
    s->key_head = (s->key_head + 1) % 16;
    const unsigned late = s->key_queue[(s->key_head + 16 - 9) % 16];       /* 9 frames ago: 0.15 s */
    *throttle = ((late & 1u) ? 1.0f : 0.0f) - ((late & 2u) ? 1.0f : 0.0f);
    *steer = ((late & 4u) ? 1.0f : 0.0f) - ((late & 8u) ? 1.0f : 0.0f);
}

/* ── the game ────────────────────────────────────────────────────────────── */

static void restart_game(rolling_ball_demo_t *s)
{
    s->active = WOOD;
    s->checkpoint = 0;
    for (int i = 0; i < N_CHECK; i++) s->cp_kind[i] = WOOD;
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
    ivp_golem_state_setup(&s->golem, s->mover);
}

static void rolling_ball_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    rolling_ball_demo_t *s = (rolling_ball_demo_t *)app->user;
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
    for (int k = 0; k < N_KINDS; k++) s->ball[k]->physical_core->speed_damp_factor = (ivp_real_t)(kinds[k].damp * s->damp_scale);
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
            /* the autopilot presses the arrow keys like a player does (IVP_ROLLING_BALL_HUMAN adds the slips) */
            float want[2], f[2], th, st;
            bot_direction(s, c, want);
            course_forward(c, f);
            human_keys(s, want, f, &th, &st);
            demo_drive_input_t in = { th, st, false, false };
            demo_drive_step(&s->drive, &in, dt);
            keys_to_world(f, s->drive.throttle, s->drive.steer, dir);
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
            s->cp_kind[i] = s->active;
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
    /* headless: at the third checkpoint, throw the ball off the course and see where it comes back */
    if (!v && !s->fall_test_started && s->checkpoint == 3) {
        s->fall_test_started = true;
        const double pos[3] = { c[0], 30.0, c[2] };
        put_ball(s, s->active, pos, NULL);
    }
    /* falling */
    if (c[1] > FALL_Y && !s->done) {
        s->falls++;
        if (s->human_seed || getenv("IVP_ROLLING_BALL_VERBOSE")) fprintf(stderr, "FELL at x %.1f z %.1f t %.1f stage %d kind %s\n", (double)c[0], (double)c[2], s->time, s->bot_stage, kinds[s->active].name);
        if (--s->lives <= 0) {
            s->lives = LIVES;
            s->checkpoint = 0;
            say(s, "out of lives: from the start");
        } else {
            say(s, "you fell");
        }
        respawn(s);
        if (s->fall_test_started && !s->fall_test_done) {
            float q[3];
            ball_pos(s, q);
            const float want[3] = { (float)s->check[s->checkpoint].x, q[1], (float)s->check[s->checkpoint].z };
            s->fall_test_dist = demo_distance(q, want);
            s->fall_test_kind = s->active;
            s->fall_test_checkpoint = s->checkpoint;
            s->fall_test_done = true;
        }
    }
}

static void rolling_ball_update_view(demo_app_t *app, demo_viewer_t *v)
{
    rolling_ball_demo_t *s = (rolling_ball_demo_t *)app->user;
    float c[3];
    ball_pos(s, c);
    chase_camera(s, v, c);
}

static void rolling_ball_reset(demo_app_t *app)
{
    restart_game((rolling_ball_demo_t *)app->user);
}

static void rolling_ball_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    rolling_ball_demo_t *s = (rolling_ball_demo_t *)app->user;
    s->drive.throttle_time = 0.06f;
    s->drive.steer_time = 0.08f;
    s->drive.center_time = 0.06f;
    demo_panel_add_float(&v->panel, "push", &s->push_scale, 0.4f, 2.5f, NULL, NULL);
    demo_panel_add_float(&v->panel, "damping", &s->damp_scale, 0.4f, 2.5f, NULL, NULL);
    demo_panel_add_float(&v->panel, "camera dist", &v->cam.orbit_distance, 5.0f, 20.0f, NULL, NULL);
    demo_drive_panel(&v->panel, &s->drive);
}

/* ── drawing ─────────────────────────────────────────────────────────────── */

static void rolling_ball_draw(demo_app_t *app, demo_viewer_t *v)
{
    rolling_ball_demo_t *s = (rolling_ball_demo_t *)app->user;
    for (int i = 0; i < s->n_pieces; i++) demo_draw_object(v, s->piece[i].obj, s->piece[i].col);
    const float c_crate[3] = { 0.92f, 0.60f, 0.25f }, c_mover[3] = { 0.40f, 0.65f, 0.95f };
    for (int i = 0; i < N_CRATES; i++) demo_draw_object(v, s->crate[i], c_crate);
    demo_draw_object(v, s->mover, c_mover);

    /* the ground far below, and pillars under the platforms, so that the course stands somewhere */
    {
        const float floor_col[3] = { 0.30f, 0.45f, 0.30f }, floor_pos[3] = { 20.0f, 10.5f, 50.0f }, floor_half[3] = { 120.0f, 0.5f, 120.0f };
        demo_solid_box(v, floor_pos, floor_half, NULL, floor_col);
        const float stone_col[3] = { 0.45f, 0.46f, 0.50f };
        static const float pillars[][3] = { { -3.0f, 0.0f, 1.0f }, { 3.0f, 0.0f, 1.0f }, { -3.0f, 0.0f, 7.0f }, { 3.0f, 0.0f, 7.0f },
                                            { -3.0f, -3.0f, 33.0f }, { 3.0f, -3.0f, 33.0f }, { -3.0f, -3.0f, 39.0f }, { 3.0f, -3.0f, 39.0f },
                                            { -3.0f, -3.0f, 47.0f }, { 3.0f, -3.0f, 47.0f }, { -3.0f, -3.0f, 55.0f }, { 3.0f, -3.0f, 55.0f },
                                            { 0.0f, -3.0f, 62.0f }, { 0.0f, -3.0f, 69.0f },
                                            { -2.4f, -3.0f, 71.0f }, { 2.4f, -3.0f, 71.0f }, { -2.4f, -3.0f, 79.0f }, { 2.4f, -3.0f, 79.0f },
                                            { -3.0f, 0.0f, 91.0f }, { 3.0f, 0.0f, 91.0f }, { -3.0f, 0.0f, 97.0f }, { 3.0f, 0.0f, 97.0f },
                                            { 19.0f, 0.0f, 91.0f }, { 25.0f, 0.0f, 91.0f }, { 19.0f, 0.0f, 97.0f }, { 25.0f, 0.0f, 97.0f },
                                            { 8.0f, 0.0f, 94.0f }, { 14.0f, 0.0f, 94.0f } };
        for (size_t i = 0; i < sizeof(pillars) / sizeof(pillars[0]); i++) {
            const float top = pillars[i][1] + 0.6f, h = 0.5f * (10.0f - top);
            const float pos[3] = { pillars[i][0], top + h, pillars[i][2] }, half[3] = { 0.35f, h, 0.35f };
            demo_solid_box(v, pos, half, NULL, stone_col);
        }
    }

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
    for (int i = 1; i < N_CHECK; i++) {
        const spot_t *c = &s->check[i];
        const bool on = i <= s->checkpoint;
        const float pos[3] = { (float)c->x, (float)c->y_top - 0.02f, (float)c->z }, half[3] = { 0.9f, 0.025f, 0.9f };
        const float col[3] = { on ? 0.25f : 0.85f, on ? 0.95f : 0.30f, on ? 0.35f : 0.25f };
        demo_solid_box(v, pos, half, NULL, col);
        /* a flag on a post: red until the ball has been here, then green */
        const float post_pos[3] = { pos[0] - 0.8f, pos[1] - 0.9f, pos[2] - 0.8f }, post_half[3] = { 0.04f, 0.9f, 0.04f };
        const float grey[3] = { 0.5f, 0.5f, 0.55f };
        demo_solid_box(v, post_pos, post_half, NULL, grey);
        const float flag_pos[3] = { pos[0] - 0.8f, pos[1] - 1.55f, pos[2] - 0.5f }, flag_half[3] = { 0.03f, 0.22f, 0.3f };
        demo_solid_box(v, flag_pos, flag_half, NULL, col);
    }

    /* the ball; six darker patches show how it turns: spheres sunk into it, so that they only just show */
    const kind_t *k = &kinds[s->active];
    float pos[3], rot[9];
    demo_object_pose(s->ball[s->active], pos, rot);
    demo_solid_sphere(v, pos, (float)k->radius, k->col);
    for (int a = 0; a < 3; a++) {
        const float d = (float)k->radius * 0.55f, sp[3] = { pos[0] + rot[a * 3] * d, pos[1] + rot[a * 3 + 1] * d, pos[2] + rot[a * 3 + 2] * d };
        demo_solid_sphere(v, sp, (float)k->radius * 0.50f, k->spot);
        const float sq[3] = { pos[0] - rot[a * 3] * d, pos[1] - rot[a * 3 + 1] * d, pos[2] - rot[a * 3 + 2] * d };
        demo_solid_sphere(v, sq, (float)k->radius * 0.50f, k->spot);
    }

    char lives[16] = "";
    for (int i = 0; i < s->lives && i < LIVES; i++) strcat(lives, "o ");
    demo_hud(v, DEMO_COL_TITLE, "%s ball   lives %s  time %.1f s   speed %.1f m/s", k->name, lives, s->time,
             (double)demo_object_speed(s->ball[s->active]));
    demo_hud(v, DEMO_COL_INFO, "checkpoint %d of %d   pads: stone, then paper", s->checkpoint, N_CHECK - 1);
    if (s->done) demo_hud(v, DEMO_COL_HIGHLIGHT, "COURSE COMPLETE in %.1f s, %d falls", s->done_time, s->falls);
    else if (s->time < s->message_until) demo_hud(v, DEMO_COL_HIGHLIGHT, "%s", s->message);
}

static void rolling_ball_stats(demo_app_t *app, char *buf, size_t size)
{
    rolling_ball_demo_t *s = (rolling_ball_demo_t *)app->user;
    float c[3];
    ball_pos(s, c);
    snprintf(buf, size, "%s ball (%.1f %.1f %.1f) stage %d/%d cp %d lives %d%s", kinds[s->active].name, (double)c[0], (double)c[1],
             (double)c[2], s->bot_stage, N_ROUTE, s->checkpoint, s->lives, s->done ? " DONE" : "");
}

static bool rolling_ball_check(demo_app_t *app)
{
    rolling_ball_demo_t *s = (rolling_ball_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    ok &= demo_check(tag, s->done, "the autopilot reached the flag (stage %d of %d)", s->bot_stage, N_ROUTE);
    ok &= demo_check(tag, s->done && s->done_time < 100.0, "in %.1f s", s->done_time);
    ok &= demo_check(tag, s->falls == 1, "without falling except for the one we forced (%d falls)", s->falls);
    ok &= demo_check(tag, s->fall_test_done && s->fall_test_checkpoint == 3 && s->fall_test_dist < 0.5f &&
                              s->fall_test_kind == s->cp_kind[3],
                     "a ball thrown off at checkpoint 3 came back to it (%.2f m away, a %s ball)", (double)s->fall_test_dist,
                     kinds[s->fall_test_kind].name);
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
    app.cfg.title       = "IVP Rolling Ball -- Roll the Ball";
    app.cfg.tag         = "rolling_ball";
    app.cfg.friction    = 0.9;
    app.cfg.elasticity  = 0.2;
    app.cfg.orbit_dist  = 9.0f;
    app.cfg.orbit_pitch = -28.0f;
    app.cfg.orbit_yaw   = 180.0f;
    app.cfg.target[0]   = 0.0f;
    app.cfg.target[1]   = -0.5f;
    app.cfg.target[2]   = 2.5f;
    app.cfg.grid_extent = -1.0f;
    app.cfg.headless_frames = 5400;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    rolling_ball_demo_t *s = (rolling_ball_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;
    s->env = w->env;
    demo_drive_defaults(&s->drive);
    s->drive.throttle_time = 0.06f;
    s->drive.steer_time = 0.08f;
    s->drive.center_time = 0.06f;

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
        ivp_object_template_t bt;
        demo_template_dynamic(&bt, &m, kinds[k].mass);
        demo_set_sphere_inertia(&bt, kinds[k].mass, kinds[k].radius);
        bt.speed_damp_factor = (ivp_real_t)kinds[k].damp;
        bt.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.05), IVP_REAL(0.05), IVP_REAL(0.05));
        demo_template_pose(&bt, NULL, -60.0 + 6.0 * k, -kinds[k].radius - 0.02, 0.0);
        s->ball[k] = demo_create_ball_templ(w, &bt, kinds[k].radius);
        if (!s->ball[k]) return 1;
    }
    restart_game(s);
    respawn(s);
    s->rng = 1u;
    s->push_scale = 1.0f;
    s->damp_scale = 1.0f;
    s->autopilot = getenv("IVP_ROLLING_BALL_AUTOPILOT") != NULL;
    if (getenv("IVP_ROLLING_BALL_HUMAN")) {
        s->human_seed = (unsigned)atoi(getenv("IVP_ROLLING_BALL_HUMAN")) * 2654435761u + 12345u;
        s->rng = s->human_seed ? s->human_seed : 1u;
    }

    app.controls    = "Arrows: roll   Backspace: last checkpoint   P: autopilot   RMB: look around";
    app.viewer_init = rolling_ball_viewer_init;
    app.update      = rolling_ball_update;
    app.update_view = rolling_ball_update_view;
    app.on_reset    = rolling_ball_reset;
    app.draw        = rolling_ball_draw;
    app.stats       = rolling_ball_stats;
    app.check       = rolling_ball_check;

    int rc = demo_run(&app, argc, argv);
    demo_world_destroy(&app.world);
    free(s);
    return rc;
}
