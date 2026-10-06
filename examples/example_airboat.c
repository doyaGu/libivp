/* example_airboat.c -- Valve's raycast airboat, from a beach into a lake
 *
 * ivp_airboat_init turns a light box into a four pontoon airboat: every PSI
 * the controller asks the game for one ray per pontoon (here: a ray against
 * the heightfield terrain and a plane for the water) and pushes the hull
 * with a pontoon spring where it touches the ground and with buoyancy where
 * it is in the water. A fan pushes it forward, the rudder steers.
 *
 *   Up / Down      throttle forward / reverse
 *   Left / Right   steer
 *   Space          reset the scene
 *
 * The scene is a heightfield with a lake (the terrain is a compact grid, the
 * water a translucent slab and a plane in the ray caster). The boat starts on
 * the beach and is driven down into the water.
 *
 * Demonstrates: ivp_airboat_init, the game-side ray caster callback,
 *               ivp_airboat_update_throttle / do_steering, a heightfield.
 */
#include "demo_app.h"
#include "demo_terrain.h"

#include <ivp/controllers/ivp_controller_airboat.h>

#define N          51             /* terrain points per side */
#define FIELD      2.0f           /* metres between points: 100 m square */
#define LAND       3.0f           /* height of the land, metres up */
#define WATER_UP   2.0f           /* water level, metres up */
#define WATER_Y    (-WATER_UP)    /* the same as a world y (Y is down) */
#define LAKE_R     26.0f
#define START_X    (-8.0f)

#define FLOAT_HEIGHT 0.25f        /* the afloat hull centre is this far above the water */

typedef struct airboat_demo {
    demo_terrain_t       terrain;
    ivp_object_t        *hull;
    ivp_airboat_state_t  boat;
    float                throttle, steer;     /* -1 .. 1 */
    /* tunables, also on the F5 panel */
    float                max_throttle;        /* full throttle; the reference scenario drives with 0.15 .. 0.6 */
    float                pontoon_k, pontoon_damp;
    float                steer_range;         /* rudder angle at full lock, in the controller's units */
    float                prop_angle;
    /* ray caster results of the last PSI */
    int                  n_ground, n_water;
    /* headless measurements */
    float                x_start;
    float                speed_cruise;
    int                  afloat_frames;       /* frames with at least three pontoons in the water */
    float                max_tilt;
    float                heading0, heading1;
    float                draft_error;         /* largest distance of the hull from its resting height while afloat */
} airboat_demo_t;

static float smoothstep(float t)
{
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return t * t * (3.0f - 2.0f * t);
}

/* A round lake with sloping beaches 4 m below the land, dunes, and hills that
 * rise towards the edge of the field so that the boat stays inside. */
static float terrain_height(float x, float z)
{
    float d = sqrtf(x * x + z * z) / LAKE_R;
    float basin = smoothstep((1.0f - d) / 0.8f);
    float dunes = 0.35f * sinf(x * 0.21f) * cosf(z * 0.17f);
    float edge = fmaxf(fabsf(x), fabsf(z)) / (0.5f * (float)(N - 1) * FIELD);
    float wall = 12.0f * smoothstep((edge - 0.76f) / 0.24f);
    return LAND + dunes * (1.0f - basin) - basin * (LAND + 1.0f) + wall;
}

/* Heading of the hull's forward (+Z) axis in the water plane, degrees. */
static float heading_deg(const ivp_object_t *hull)
{
    float pos[3], rot[9];
    demo_object_pose(hull, pos, rot);
    return atan2f(rot[6], rot[8]) * (180.0f / 3.14159265f);
}

/* Reference do_raycasts_gameside: one impact per pontoon ray. The ground is
 * the terrain object, the water a plane at WATER_Y. */
static void airboat_rays(void *ctx, int n_rays, const ivp_ray_solver_template_t *rays,
                         ivp_airboat_impact_t *impacts)
{
    airboat_demo_t *s = (airboat_demo_t *)ctx;
    s->n_ground = s->n_water = 0;
    for (int i = 0; i < n_rays; i++) {
        const ivp_vec3r_t *start = &rays[i].ray_start_point;
        const ivp_vec3r_t *dir = &rays[i].ray_normized_direction;
        const double len = (double)rays[i].ray_length;
        ivp_airboat_impact_t *imp = &impacts[i];
        memset(imp, 0, sizeof(*imp));

        ivp_ray_solver_min_t rs;
        ivp_ray_solver_min_init(&rs, &rays[i]);
        ivp_ray_solver_check_object(&rs.base, s->terrain.obj);
        const ivp_ray_hit_t *h = ivp_ray_solver_min_get_ray_hit(&rs);
        if (h && h->hit_distance >= IVP_REAL(0.0) && (double)h->hit_distance <= len) {
            imp->impact = true;
            for (int k = 0; k < 3; k++)
                imp->impact_point_ws.k[k] = start->k[k] + dir->k[k] * h->hit_distance;
            /* the terrain keeps the world orientation; the normal faces the ray */
            ivp_vec3f_t n = ivp_vec3f((float)h->hit_normal.k[0], (float)h->hit_normal.k[1],
                                      (float)h->hit_normal.k[2]);
            if (n.k[0] * (float)dir->k[0] + n.k[1] * (float)dir->k[1] + n.k[2] * (float)dir->k[2] > 0.0f) {
                n.k[0] = -n.k[0]; n.k[1] = -n.k[1]; n.k[2] = -n.k[2];
            }
            imp->impact_normal_ws = n;
            imp->friction = 0.8f;
            s->n_ground++;
        }
        imp->in_water = (double)start->k[1] > (double)WATER_Y;
        if (imp->in_water) imp->impact = false;           /* afloat: buoyancy only */
        if (imp->in_water) s->n_water++;
        if ((float)dir->k[1] > 1e-6f) {
            const double t = ((double)WATER_Y - (double)start->k[1]) / (double)dir->k[1];
            if (t >= 0.0 && t <= len) imp->impact_water = true;
        }
    }
}

/* Panel callback: the pontoon springs of the controller. */
static void airboat_apply(void *ctx)
{
    airboat_demo_t *s = (airboat_demo_t *)ctx;
    for (int i = 0; i < s->boat.n_wheels; i++) {
        ivp_airboat_change_spring_constant(&s->boat, i, s->pontoon_k);
        ivp_airboat_change_spring_dampening(&s->boat, i, s->pontoon_damp);
        ivp_airboat_change_spring_dampening_compression(&s->boat, i, s->pontoon_damp);
    }
}

static void airboat_viewer_init(demo_app_t *app, demo_viewer_t *v)
{
    airboat_demo_t *s = (airboat_demo_t *)app->user;
    demo_terrain_make_mesh(&s->terrain, v->renderer);
    demo_panel_t *p = &v->panel;
    demo_panel_add_float(p, "full throttle",  &s->max_throttle, 0.1f,  1.0f,    NULL, NULL);
    demo_panel_add_float(p, "rudder range",   &s->steer_range,  0.05f, 0.6f,    NULL, NULL);
    demo_panel_add_float(p, "pontoon k",      &s->pontoon_k,    2000.0f, 30000.0f, airboat_apply, s);
    demo_panel_add_float(p, "pontoon damping", &s->pontoon_damp, 100.0f, 3000.0f, airboat_apply, s);
}

static void apply_input(airboat_demo_t *s, demo_app_t *app)
{
    ivp_airboat_update_throttle(&s->boat, s->throttle);
    ivp_airboat_do_steering(&s->boat, s->steer * s->steer_range);
    /* update_throttle does not wake the body, and a sleeping core runs no controller */
    if (s->throttle != 0.0f || s->steer != 0.0f)
        ivp_environment_ensure_core_in_simulation(app->world.env, s->hull);
}

static void airboat_update(demo_app_t *app, demo_viewer_t *v, int frame)
{
    airboat_demo_t *s = (airboat_demo_t *)app->user;
    float c[3];
    demo_object_center(s->hull, c);
    float tilt = demo_object_tilt_deg(s->hull);
    if (tilt > s->max_tilt) s->max_tilt = tilt;
    if (s->n_water >= 3) {
        s->afloat_frames++;
        float err = fabsf(c[1] - (WATER_Y - FLOAT_HEIGHT));
        if (err > s->draft_error) s->draft_error = err;
    }

    if (v) {
        const bool *keys = SDL_GetKeyboardState(NULL);
        s->throttle = (keys[SDL_SCANCODE_UP] ? s->max_throttle : 0.0f) - (keys[SDL_SCANCODE_DOWN] ? 0.5f * s->max_throttle : 0.0f);
        s->steer = (keys[SDL_SCANCODE_RIGHT] ? 1.0f : 0.0f) - (keys[SDL_SCANCODE_LEFT] ? 1.0f : 0.0f);   /* + turns right */
        for (int k = 0; k < 3; k++) v->cam.target[k] += (c[k] - v->cam.target[k]) * 0.05f;
    } else {
        /* scripted: float 1 s, drive, steer left for 2 s, keep going */
        s->throttle = frame < 60 ? 0.0f : 0.35f;
        s->steer = (frame >= 240 && frame < 360) ? 1.0f : 0.0f;
        if (frame == 60) s->x_start = c[0];
        if (frame == 240) s->heading0 = heading_deg(s->hull);
        if (frame == 360) s->heading1 = heading_deg(s->hull);
        if (frame == 239) s->speed_cruise = demo_object_speed(s->hull);
    }
    s->prop_angle += s->throttle * 0.9f;
    apply_input(s, app);
}

static void airboat_reset(demo_app_t *app)
{
    airboat_demo_t *s = (airboat_demo_t *)app->user;
    s->throttle = s->steer = 0.0f;
    s->max_tilt = 0.0f;
    ivp_airboat_update_throttle(&s->boat, 0.0f);
    ivp_airboat_do_steering(&s->boat, 0.0f);
}

/* ── drawing ─────────────────────────────────────────────────────────────── */

static void body_vec(const float rot[9], float a, float b, float c, float out[3])
{
    for (int k = 0; k < 3; k++) out[k] = rot[k] * a + rot[3 + k] * b + rot[6 + k] * c;
}

static void body_point(const float pos[3], const float rot[9], float a, float b, float c, float out[3])
{
    body_vec(rot, a, b, c, out);
    for (int k = 0; k < 3; k++) out[k] += pos[k];
}

static void body_box(demo_viewer_t *v, const float pos[3], const float rot[9], float x, float y, float z,
                     float hx, float hy, float hz, const float color[3])
{
    float c[3], half[3] = { hx, hy, hz };
    body_point(pos, rot, x, y, z, c);
    demo_solid_box(v, c, half, rot, color);
}

static void body_cyl(demo_viewer_t *v, const float pos[3], const float rot[9], const float a[3],
                     const float b[3], float r, const float color[3])
{
    float pa[3], pb[3];
    body_point(pos, rot, a[0], a[1], a[2], pa);
    body_point(pos, rot, b[0], b[1], b[2], pb);
    if (v->renderer && demo_prim(v, pa, pb)) ivp_render_draw_solid_cylinder(v->renderer, pa, pb, r, color);
}

/* The boat on the hull pose: pontoons, deck, seat, fan cage with a turning
 * propeller and two rudders that follow the steering. Forward is +Z, up -Y. */
static void draw_boat(demo_viewer_t *v, const airboat_demo_t *s)
{
    if (!v->solid || !v->renderer) return;
    float pos[3], rot[9];
    demo_object_pose(s->hull, pos, rot);

    const float c_float[3] = { 0.93f, 0.93f, 0.90f }, c_deck[3] = { 0.85f, 0.22f, 0.18f };
    const float c_dark[3] = { 0.14f, 0.14f, 0.16f }, c_seat[3] = { 0.20f, 0.22f, 0.30f };
    const float c_cage[3] = { 0.70f, 0.72f, 0.76f }, c_blade[3] = { 0.10f, 0.10f, 0.12f };

    for (int side = -1; side <= 1; side += 2) {
        const float x = 0.7f * (float)side;
        const float a[3] = { x, 0.25f, -1.4f }, b[3] = { x, 0.25f, 1.25f };
        body_cyl(v, pos, rot, a, b, 0.2f, c_float);
        float nose[3];
        body_point(pos, rot, x, 0.25f, 1.25f, nose);
        demo_solid_sphere(v, nose, 0.2f, c_float);
        body_box(v, pos, rot, x * 0.5f, 0.0f, 0.0f, 0.35f, 0.03f, 1.1f, c_deck);     /* deck beside the seat */
    }
    body_box(v, pos, rot, 0.0f, -0.05f, 0.1f, 0.5f, 0.05f, 1.2f, c_deck);              /* deck */
    body_box(v, pos, rot, 0.0f, -0.38f, 0.2f, 0.20f, 0.20f, 0.18f, c_seat);            /* seat */
    body_box(v, pos, rot, 0.0f, -0.60f, 0.05f, 0.20f, 0.12f, 0.04f, c_seat);           /* backrest */
    body_box(v, pos, rot, 0.0f, -0.20f, -0.62f, 0.18f, 0.16f, 0.20f, c_dark);          /* engine */

    /* fan cage: two posts and a frame behind the engine, the propeller inside */
    const float zc = -1.05f, yc = -0.55f;
    for (int side = -1; side <= 1; side += 2)
        body_box(v, pos, rot, 0.5f * (float)side, yc, zc, 0.03f, 0.5f, 0.03f, c_cage);
    body_box(v, pos, rot, 0.0f, yc - 0.5f, zc, 0.53f, 0.03f, 0.03f, c_cage);
    body_box(v, pos, rot, 0.0f, yc + 0.5f, zc, 0.53f, 0.03f, 0.03f, c_cage);
    {
        float p[3], ca = cosf(s->prop_angle), sa = sinf(s->prop_angle);
        float bx[3], by[3], bz[3], r2[9];
        body_vec(rot, 1, 0, 0, bx); body_vec(rot, 0, 1, 0, by); body_vec(rot, 0, 0, 1, bz);
        for (int blade = 0; blade < 2; blade++) {
            float a = blade ? ca : -sa, b = blade ? sa : ca;     /* blade axis in the xy plane */
            for (int k = 0; k < 3; k++) {
                r2[k] = bx[k] * a + by[k] * b;
                r2[3 + k] = -bx[k] * b + by[k] * a;
                r2[6 + k] = bz[k];
            }
            body_point(pos, rot, 0.0f, yc, zc + 0.02f, p);
            const float half[3] = { 0.46f, 0.045f, 0.012f };
            demo_solid_box(v, p, half, r2, c_blade);
        }
    }
    /* rudders behind the fan, turned by the steering (about the hull Y axis) */
    {
        const float ang = -s->steer * (s->steer_range / 0.3f) * 0.6f;       /* trailing edge to the side of the turn */
        float ca = cosf(ang), sa = sinf(ang), bx[3], by[3], bz[3], r2[9], p[3];
        body_vec(rot, 1, 0, 0, bx); body_vec(rot, 0, 1, 0, by); body_vec(rot, 0, 0, 1, bz);
        for (int k = 0; k < 3; k++) {
            r2[k] = bx[k] * ca - bz[k] * sa;
            r2[3 + k] = by[k];
            r2[6 + k] = bx[k] * sa + bz[k] * ca;
        }
        for (int side = -1; side <= 1; side += 2) {
            body_point(pos, rot, 0.27f * (float)side, yc, zc - 0.28f, p);
            const float half[3] = { 0.02f, 0.38f, 0.17f };
            demo_solid_box(v, p, half, r2, c_deck);
        }
    }
}

static void airboat_draw(demo_app_t *app, demo_viewer_t *v)
{
    airboat_demo_t *s = (airboat_demo_t *)app->user;
    const float c_land[3] = { 0.72f, 0.66f, 0.46f }, c_wire[3] = { 0.60f, 0.55f, 0.35f };
    const float c_hull[3] = { 0.85f, 0.22f, 0.18f }, c_water[3] = { 0.02f, 0.30f, 0.78f };

    demo_terrain_draw(v, &s->terrain, v->solid ? c_land : c_wire);
    if (v->solid) {
        /* the collision box is hidden inside the model */
        draw_boat(v, s);
        demo_draw_object_wire(v, s->hull, c_hull);
    } else {
        demo_draw_object(v, s->hull, c_hull);
    }

    /* the water: a translucent slab from the surface down to the lake bed */
    if (v->solid) {
        const float half = 0.5f * (float)(N - 1) * FIELD - 0.05f;
        const float depth = 3.2f, centre[3] = { 0.0f, WATER_Y + depth * 0.5f, 0.0f };
        const float ext[3] = { half, depth * 0.5f, half };
        demo_set_alpha(v, 0.62f);
        demo_solid_box(v, centre, ext, NULL, c_water);
        demo_set_alpha(v, 1.0f);
    } else {
        const float half = 0.5f * (float)(N - 1) * FIELD;
        for (float t = -half; t <= half; t += 4.0f) {
            float a[3] = { t, WATER_Y, -half }, b[3] = { t, WATER_Y, half };
            float c[3] = { -half, WATER_Y, t }, d[3] = { half, WATER_Y, t };
            demo_line(v, a, b, c_water);
            demo_line(v, c, d, c_water);
        }
    }

    float c[3];
    demo_object_center(s->hull, c);
    demo_hud(v, DEMO_COL_INFO, "Speed %.1f m/s  heading %.0f deg  %s", (double)demo_object_speed(s->hull),
             (double)heading_deg(s->hull),
             s->n_water >= 3 ? "afloat" : (s->n_ground >= 3 ? "on the ground" : "airborne / mixed"));
    demo_hud(v, DEMO_COL_INFO, "Pontoons: %d on the ground, %d in the water   throttle %+.0f%%  rudder %+.0f%% (+ = right)",
             s->n_ground, s->n_water, (double)(s->throttle * 100.0f), (double)(s->steer * 100.0f));
}

static void airboat_stats(demo_app_t *app, char *buf, size_t size)
{
    airboat_demo_t *s = (airboat_demo_t *)app->user;
    float c[3];
    demo_object_center(s->hull, c);
    snprintf(buf, size, "hull (%.1f, %.2f, %.1f) speed %.2f ground %d water %d heading %.0f tilt %.1f", (double)c[0],
             (double)c[1], (double)c[2], (double)demo_object_speed(s->hull), s->n_ground, s->n_water,
             (double)heading_deg(s->hull), (double)demo_object_tilt_deg(s->hull));
}

static bool airboat_check(demo_app_t *app)
{
    airboat_demo_t *s = (airboat_demo_t *)app->user;
    const char *tag = app->cfg.tag;
    bool ok = true;
    float c[3];
    demo_object_center(s->hull, c);
    ok &= demo_check(tag, s->speed_cruise > 3.0f, "the fan drives the boat: %.1f m/s after 3 s", (double)s->speed_cruise);
    ok &= demo_check(tag, s->afloat_frames > 120, "the pontoons floated the boat for %d frames", s->afloat_frames);
    ok &= demo_check(tag, s->draft_error < 0.4f, "afloat the hull rides within %.2f m of its resting height",
                     (double)s->draft_error);
    ok &= demo_check(tag, s->max_tilt < 40.0f, "the hull stays upright (max tilt %.1f deg)", (double)s->max_tilt);
    float dh = fabsf(s->heading1 - s->heading0);
    if (dh > 180.0f) dh = 360.0f - dh;
    ok &= demo_check(tag, dh > 15.0f, "the rudder turns the boat: %.0f deg", (double)dh);
    ok &= demo_check(tag, c[0] > s->x_start + 10.0f, "the boat moved %.1f m", (double)(c[0] - s->x_start));
    return ok;
}

int main(int argc, char **argv)
{
    demo_app_t app;
    memset(&app, 0, sizeof(app));
    demo_config_defaults(&app.cfg);
    app.cfg.title       = "IVP Airboat -- Pontoons on Land and Water";
    app.cfg.tag         = "airboat";
    app.cfg.friction    = 0.8;
    app.cfg.elasticity  = 0.1;
    app.cfg.orbit_dist  = 11.0f;
    app.cfg.orbit_yaw   = 35.0f;
    app.cfg.target[0]   = START_X;
    app.cfg.target[1]   = -3.0f;
    app.cfg.grid_extent = -1.0f;
    if (!demo_world_init(&app.world, &app.cfg)) return 1;

    airboat_demo_t *s = (airboat_demo_t *)calloc(1, sizeof(*s));
    if (!s) return 1;
    app.user = s;
    demo_world_t *w = &app.world;

    float *heights = (float *)malloc(sizeof(float) * N * N);
    if (!heights) return 1;
    for (int r = 0; r < N; r++)
        for (int c = 0; c < N; c++)
            heights[r * N + c] = terrain_height(((float)r - 0.5f * (N - 1)) * FIELD,
                                                ((float)c - 0.5f * (N - 1)) * FIELD);
    bool built = demo_terrain_create(w, &s->terrain, N, FIELD, heights, app.cfg.friction, app.cfg.elasticity);
    free(heights);
    if (!built) {
        fprintf(stderr, "[airboat] terrain build failed\n");
        return 1;
    }

    /* hull: a 300 kg box, with the inertia the reference scenario uses; it
     * faces +X (the boat's forward axis is +Z) and starts on the beach */
    {
        const double half = 0.5 * 1.5707963267948966;
        ivp_quatr_t q = ivp_quatr_make(0.0, (ivp_real_t)sin(half), 0.0, (ivp_real_t)cos(half));
        ivp_object_template_t t;
        demo_template_dynamic(&t, &w->mat, 300.0);
        t.rot_inertia_is_factor = false;
        t.rot_inertia = ivp_vec3r(IVP_REAL(231.0), IVP_REAL(289.0), IVP_REAL(70.0));
        t.speed_damp_factor = IVP_REAL(0.0);
        t.rot_speed_damp_factor = ivp_vec3r(IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        t.material.friction = IVP_REAL(0.25);
        t.material.elasticity = IVP_REAL(0.1);
        const double y = (double)WATER_Y - 0.25;
        demo_template_pose(&t, &q, START_X, y, 0.0);
        s->hull = demo_create_box_templ(w, &t, 0.5, 0.1, 0.8);
        if (!s->hull) return 1;
    }

    ivp_car_system_template_t tcs;
    ivp_car_system_template_init(&tcs, 4, 2);
    tcs.car_body = s->hull;
    for (int i = 0; i < 4; i++) {
        const double wx = (i & 1) ? 0.7 : -0.7;     /* odd = right */
        const double wz = (i & 2) ? -1.3 : 1.3;     /* 2, 3 = rear */
        tcs.wheel_pos_Bos[i] = ivp_vec3f((float)wx, 0.25f, (float)wz);
        tcs.trace_pos_Bos[i] = ivp_vec3f((float)wx, 0.25f, (float)wz);
        tcs.wheel_radius[i] = (ivp_real_t)0.3f;
        tcs.spring_constant[i] = IVP_REAL(10000.0);
        tcs.spring_damping[i] = IVP_REAL(800.0);
        tcs.spring_damping_compression[i] = IVP_REAL(800.0);
        tcs.spring_pre_tension[i] = IVP_REAL(0.0);
    }
    tcs.stabilizer_constant[0] = IVP_REAL(1000.0);
    tcs.stabilizer_constant[1] = IVP_REAL(1000.0);
    tcs.wheel_max_rotation_speed[0] = IVP_REAL(50.0);
    tcs.wheel_max_rotation_speed[1] = IVP_REAL(50.0);
    tcs.extra_gravity_force = IVP_REAL(0.0);
    tcs.body_down_force_vertical_offset = (ivp_real_t)0.2f;
    if (!ivp_airboat_init(&s->boat, &tcs)) {
        fprintf(stderr, "ivp_airboat_init failed\n");
        return 1;
    }
    ivp_airboat_set_raycast(&s->boat, airboat_rays, s);
    s->max_throttle = 0.6f;
    s->steer_range = 0.3f;
    s->pontoon_k = 10000.0f;
    s->pontoon_damp = 800.0f;
    airboat_apply(s);

    app.controls    = "Up/Down: throttle forward / reverse   Left/Right: steer";
    app.viewer_init = airboat_viewer_init;
    app.update      = airboat_update;
    app.on_reset    = airboat_reset;
    app.draw        = airboat_draw;
    app.stats       = airboat_stats;
    app.check       = airboat_check;

    int rc = demo_run(&app, argc, argv);
    ivp_airboat_destroy(&s->boat);
    demo_world_destroy(&app.world);
    demo_terrain_destroy(&s->terrain);
    free(s);
    return rc;
}
