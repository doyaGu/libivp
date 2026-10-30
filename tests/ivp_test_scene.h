/* ivp_test_scene.h -- geometry and bodies shared by the world tests.
 *
 * The ground is an 80 x 1 x 80 static box centred at y = 2, so its top is
 * at y = IVP_TEST_GROUND_TOP (IVP worlds are Y down: "above" is smaller y).
 * Bodies are 1 m cubes or balls of radius 0.4 with lightly damped motion.
 * The geometry is built outside any world and outlives the worlds using
 * it. */
#ifndef IVP_TEST_SCENE_H
#define IVP_TEST_SCENE_H

#include <ivp/builder/ivp_build_surface.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/geometry/ivp_surman.h>

#include <stdbool.h>
#include <string.h>

#define IVP_TEST_GROUND_TOP 1.5

typedef struct ivp_test_scene_assets {
    ivp_compact_surface_t *ground_surface;
    ivp_compact_surface_t *cube_surface;
    ivp_surman_polygon_t ground_surman;
    ivp_surman_polygon_t cube_surman;
} ivp_test_scene_assets_t;

static inline bool ivp_test_scene_assets_init(ivp_test_scene_assets_t *assets)
{
    memset(assets, 0, sizeof(*assets));
    assets->ground_surface = ivp_build_box_surface(NULL, 40.0f, 0.5f, 40.0f);
    assets->cube_surface = ivp_build_box_surface(NULL, 0.5f, 0.5f, 0.5f);
    if (!assets->ground_surface || !assets->cube_surface) return false;
    ivp_surman_polygon_init(&assets->ground_surman, assets->ground_surface);
    ivp_surman_polygon_init(&assets->cube_surman, assets->cube_surface);
    return true;
}

static inline void ivp_test_scene_assets_destroy(ivp_test_scene_assets_t *assets)
{
    if (assets->cube_surface) ivp_compact_surface_free(NULL, assets->cube_surface);
    if (assets->ground_surface) ivp_compact_surface_free(NULL, assets->ground_surface);
    memset(assets, 0, sizeof(*assets));
}

static inline ivp_object_t *ivp_test_scene_ground(ivp_environment_t *env,
                                                  const ivp_test_scene_assets_t *assets)
{
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_POLYGON;
    t.physical_unmoveable = true;
    t.mass = IVP_REAL(1.0e6);
    ivp_vec3r_set(&t.rot_inertia, IVP_REAL(1.0e6), IVP_REAL(1.0e6), IVP_REAL(1.0e6));
    t.rot_inertia_is_factor = false;
    t.material.friction = IVP_REAL(0.6);
    t.material.elasticity = IVP_REAL(0.3);
    ivp_vec3r_set(&t.position, IVP_REAL(0.0), IVP_REAL(2.0), IVP_REAL(0.0));
    return ivp_environment_create_object(env, &t, &assets->ground_surman.base);
}

/* A body at (x, y, z): a ball when surman is NULL, otherwise of that
 * surface (with a fixed inertia for the shared cube).  simulated false
 * leaves it out of the simulation until something wakes it. */
static inline ivp_object_t *ivp_test_scene_body(ivp_environment_t *env,
                                                const ivp_test_scene_assets_t *assets,
                                                const ivp_surman_t *surman,
                                                double x, double y, double z,
                                                bool simulated)
{
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.start_in_simulation = simulated;
    t.material.friction = IVP_REAL(0.6);
    t.material.elasticity = IVP_REAL(0.3);
    t.speed_damp_factor = IVP_REAL(0.02);
    ivp_vec3r_set(&t.rot_speed_damp_factor, IVP_REAL(0.02), IVP_REAL(0.02),
                  IVP_REAL(0.02));
    ivp_vec3r_set(&t.position, (ivp_real_t)x, (ivp_real_t)y, (ivp_real_t)z);
    if (!surman) {
        t.type = IVP_OBJECT_BALL;
        t.ball_radius = IVP_REAL(0.4);
        t.mass = IVP_REAL(0.8);
        return ivp_environment_create_object(env, &t, NULL);
    }
    t.type = IVP_OBJECT_POLYGON;
    t.mass = IVP_REAL(0.6);
    if (surman == &assets->cube_surman.base) {
        ivp_vec3r_set(&t.rot_inertia, IVP_REAL(0.1), IVP_REAL(0.1), IVP_REAL(0.1));
        t.rot_inertia_is_factor = false;
    }
    return ivp_environment_create_object(env, &t, surman);
}

#endif /* IVP_TEST_SCENE_H */
