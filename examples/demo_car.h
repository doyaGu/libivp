/* demo_car.h -- A detailed model for the raycast car
 *
 * Drawn on the chassis pose with solid boxes and cylinders: cabin, roof,
 * bumpers, head and tail lights, a light bar, a spare wheel, fenders, and
 * four wheels (tyre, rim, hub, bolts that turn with the wheel) that steer,
 * follow the suspension and hang when airborne. The chassis box itself is
 * drawn by the caller (demo_draw_object), so picking and outlines still
 * work; this only adds the details, in the solid view.
 *
 * The model assumes the box chassis of example_vehicle: half extents
 * (1.0, 0.4, 2.0), forward +Z, up -Y (IVP worlds are Y down).
 *
 * Included after demo_app.h and <ivp/controllers/ivp_controller_raycast_car.h>.
 */
#ifndef IVP_EXAMPLE_DEMO_CAR_H
#define IVP_EXAMPLE_DEMO_CAR_H

typedef struct demo_car_frame {
    const ivp_object_t *chassis;
    float               pos[3];       /* object origin (the chassis box centre) */
    float               rot[9];       /* orientation, column major */
} demo_car_frame_t;

/* World position of a point of the body, given in the chassis object space.
 * (The wheel hard points are in the core system instead: a mass centre that
 * is not at the object origin shifts the two apart.) */
static inline void demo_car_point(const demo_car_frame_t *f, float x, float y, float z, float out[3])
{
    for (int k = 0; k < 3; k++) out[k] = f->pos[k] + f->rot[k] * x + f->rot[3 + k] * y + f->rot[6 + k] * z;
}

static inline void demo_car_core_point(const demo_car_frame_t *f, const ivp_vec3f_t *p, float out[3])
{
    ivp_vec3r_t q = ivp_vec3r(p->k[0], p->k[1], p->k[2]);
    demo_core_point_ws(f->chassis, &q, out);
}

static inline void demo_car_box(demo_viewer_t *v, const demo_car_frame_t *f,
                                float x, float y, float z, float hx, float hy, float hz,
                                const float color[3])
{
    float c[3], half[3] = { hx, hy, hz };
    demo_car_point(f, x, y, z, c);
    demo_solid_box(v, c, half, f->rot, color);
}

static inline void demo_car_cyl(demo_viewer_t *v, const float a[3], const float b[3], float r,
                                const float color[3])
{
    if (v->renderer && demo_prim(v, a, b)) ivp_render_draw_solid_cylinder(v->renderer, a, b, r, color);
}

static inline void demo_car_wheel(demo_viewer_t *v, const demo_car_frame_t *f, const ivp_car_wheel_t *wh)
{
    const float tyre[3] = { 0.07f, 0.07f, 0.08f }, rim[3] = { 0.78f, 0.80f, 0.84f };
    const float hub[3] = { 0.30f, 0.31f, 0.34f }, bolt[3] = { 0.95f, 0.95f, 0.97f };
    const float arm[3] = { 0.25f, 0.26f, 0.28f };

    /* hard point and wheel centre: the ray reaches the ground one radius
     * beyond the wheel centre; in the air the spring hangs at full length */
    float hp[3], c[3], dir[3];
    demo_car_core_point(f, &wh->hp_cs, hp);
    for (int k = 0; k < 3; k++)
        dir[k] = f->rot[0 + k] * wh->spring_direction_cs.k[0] + f->rot[3 + k] * wh->spring_direction_cs.k[1]
               + f->rot[6 + k] * wh->spring_direction_cs.k[2];
    float reach = wh->grounded ? wh->raycast_dist - wh->wheel_radius : wh->spring_len;
    if (reach < 0.0f) reach = 0.0f;
    for (int k = 0; k < 3; k++) c[k] = hp[k] + dir[k] * reach;

    /* axle direction (steered), pointing out of the car */
    float axis[3];
    for (int k = 0; k < 3; k++)
        axis[k] = f->rot[0 + k] * wh->axis_direction_cs.k[0] + f->rot[3 + k] * wh->axis_direction_cs.k[1]
                + f->rot[6 + k] * wh->axis_direction_cs.k[2];
    float side = wh->hp_cs.k[0] >= 0.0f ? 1.0f : -1.0f;
    float al = sqrtf(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    if (al < 1e-6f) return;
    for (int k = 0; k < 3; k++) axis[k] /= al;
    /* make it point out of the car: along +X on the right side, -X on the left */
    if (side * (axis[0] * f->rot[0] + axis[1] * f->rot[1] + axis[2] * f->rot[2]) < 0.0f)
        for (int k = 0; k < 3; k++) axis[k] = -axis[k];
    float r = wh->wheel_radius;

    float a[3], b[3];
    #define AXIS_PT(out, t) do { for (int k = 0; k < 3; k++) (out)[k] = c[k] + axis[k] * (t); } while (0)
    AXIS_PT(a, -0.15f); AXIS_PT(b, 0.15f);
    demo_car_cyl(v, a, b, r, tyre);
    AXIS_PT(a, -0.12f); AXIS_PT(b, 0.17f);
    demo_car_cyl(v, a, b, r * 0.62f, rim);
    AXIS_PT(a, 0.10f); AXIS_PT(b, 0.20f);
    demo_car_cyl(v, a, b, r * 0.22f, hub);

    /* five bolts on the outer face, turning with the wheel */
    float u[3], w[3];
    const float up[3] = { f->rot[3], f->rot[4], f->rot[5] };
    u[0] = axis[1] * up[2] - axis[2] * up[1];
    u[1] = axis[2] * up[0] - axis[0] * up[2];
    u[2] = axis[0] * up[1] - axis[1] * up[0];
    float ul = sqrtf(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
    if (ul > 1e-6f) {
        for (int k = 0; k < 3; k++) u[k] /= ul;
        w[0] = axis[1] * u[2] - axis[2] * u[1];
        w[1] = axis[2] * u[0] - axis[0] * u[2];
        w[2] = axis[0] * u[1] - axis[1] * u[0];
        for (int i = 0; i < 5; i++) {
            float phi = wh->angle_wheel + 1.2566371f * (float)i;
            float cs = cosf(phi), sn = sinf(phi), p[3];
            for (int k = 0; k < 3; k++)
                p[k] = c[k] + axis[k] * 0.185f + (u[k] * cs + w[k] * sn) * r * 0.38f;
            const float half[3] = { 0.035f, 0.035f, 0.035f };
            demo_solid_box(v, p, half, f->rot, bolt);
        }
    }

    /* suspension arm from the hard point to the wheel */
    demo_car_cyl(v, hp, c, 0.045f, arm);
    #undef AXIS_PT
}

/* Add the details to the car; body is the paint colour. */
static inline void demo_draw_car(demo_viewer_t *v, const ivp_object_t *chassis, const ivp_car_state_t *car,
                                 const float body[3])
{
    if (!v->solid || !v->renderer) return;
    demo_car_frame_t f;
    f.chassis = chassis;
    demo_object_pose(chassis, f.pos, f.rot);

    const float glass[3]  = { 0.10f, 0.15f, 0.22f }, trim[3] = { 0.12f, 0.12f, 0.14f };
    const float roof[3]   = { body[0] * 0.85f, body[1] * 0.85f, body[2] * 0.85f };
    const float head[3]   = { 1.00f, 0.95f, 0.70f }, tail[3] = { 0.90f, 0.10f, 0.10f };
    const float lamp[3]   = { 1.00f, 0.85f, 0.30f };

    /* cabin: glass between four pillars of paint, a roof on top (up is -Y) */
    demo_car_box(v, &f, 0.0f, -0.74f, -0.25f, 0.86f, 0.34f, 0.94f, glass);
    demo_car_box(v, &f, 0.0f, -1.13f, -0.25f, 0.92f, 0.05f, 1.00f, roof);
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sz = -1; sz <= 1; sz += 2)
            demo_car_box(v, &f, 0.86f * (float)sx, -0.74f, -0.25f + 0.94f * (float)sz, 0.045f, 0.36f, 0.045f, roof);
    /* hood scoop and fender flares */
    demo_car_box(v, &f, 0.0f, -0.47f, 1.15f, 0.34f, 0.07f, 0.45f, roof);
    for (int i = 0; i < car->n_wheels; i++) {
        float sx = car->wheels[i].hp_cs.k[0] >= 0.0f ? 1.0f : -1.0f;
        demo_car_box(v, &f, 1.04f * sx, -0.18f, car->wheels[i].hp_cs.k[2], 0.08f, 0.13f, 0.52f, trim);
    }
    /* bumpers, head and tail lights */
    demo_car_box(v, &f, 0.0f, 0.24f, 2.07f, 1.05f, 0.13f, 0.08f, trim);
    demo_car_box(v, &f, 0.0f, 0.24f, -2.07f, 1.05f, 0.13f, 0.08f, trim);
    demo_car_box(v, &f, 0.72f, -0.12f, 2.03f, 0.22f, 0.10f, 0.05f, head);
    demo_car_box(v, &f, -0.72f, -0.12f, 2.03f, 0.22f, 0.10f, 0.05f, head);
    demo_car_box(v, &f, 0.76f, -0.12f, -2.03f, 0.20f, 0.10f, 0.05f, tail);
    demo_car_box(v, &f, -0.76f, -0.12f, -2.03f, 0.20f, 0.10f, 0.05f, tail);
    /* light bar on the roof */
    demo_car_box(v, &f, 0.0f, -1.22f, 0.45f, 0.62f, 0.05f, 0.07f, trim);
    for (int i = 0; i < 4; i++)
        demo_car_box(v, &f, -0.45f + 0.30f * (float)i, -1.29f, 0.45f, 0.08f, 0.04f, 0.05f, lamp);
    /* spare wheel on the tail */
    {
        float a[3], b[3];
        demo_car_point(&f, 0.0f, -0.12f, -2.14f, a);
        demo_car_point(&f, 0.0f, -0.12f, -2.34f, b);
        demo_car_cyl(v, a, b, 0.38f, (const float[3]){ 0.07f, 0.07f, 0.08f });
        demo_car_point(&f, 0.0f, -0.12f, -2.30f, a);
        demo_car_point(&f, 0.0f, -0.12f, -2.37f, b);
        demo_car_cyl(v, a, b, 0.22f, (const float[3]){ 0.78f, 0.80f, 0.84f });
    }

    for (int i = 0; i < car->n_wheels; i++) demo_car_wheel(v, &f, &car->wheels[i]);
}

#endif /* IVP_EXAMPLE_DEMO_CAR_H */
