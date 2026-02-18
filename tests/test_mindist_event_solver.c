#include "ivp_test.h"

#include <ivp/collision/ivp_mindist_event.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/solver/ivp_contact.h>

#include <string.h>

static ivp_environment_t *make_env(void)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t x, ivp_real_t vx)
{
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.position, x, IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    if (obj) {
        ivp_vec3r_set(&obj->core.speed, vx, IVP_REAL(0.0), IVP_REAL(0.0));
    }
    return obj;
}

static void seed_entry_from_contact(ivp_mindist_entry_t *entry,
                                    ivp_object_t *a,
                                    ivp_object_t *b)
{
    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = a;
    cp.obj_b = b;
    ivp_contact_compute_sphere_sphere(&cp);

    entry->last_distance = cp.gap_length;
    entry->last_normal = cp.normal;
    entry->closest_point_a = cp.point_a;
    entry->closest_point_b = cp.point_b;
    entry->state = IVP_MDIST_EXACT;
    entry->synapse[0].status = IVP_ST_BALL;
    entry->synapse[1].status = IVP_ST_BALL;
}

IVP_TEST(mindist_event_predicts_impact)
{
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(3.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(2.2), IVP_REAL(-3.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_mindist_entry_t *entry =
        ivp_mindist_manager_get_or_create(&env->mindist_manager, a, b);
    IVP_ASSERT_NOT_NULL(entry);

    seed_entry_from_contact(entry, a, b);

    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);

    ivp_mindist_event_prediction_t prediction;
    bool ok = ivp_mindist_event_predict(entry, &settings,
                                        IVP_REAL(0.0), IVP_REAL(0.1),
                                        &prediction);
    IVP_ASSERT_TRUE(ok);
    IVP_ASSERT_EQ(prediction.kind, IVP_MINDIST_EVENT_IMPACT);
    IVP_ASSERT_TRUE(prediction.event_time >= IVP_REAL(0.0));
    IVP_ASSERT_TRUE(prediction.event_time <= IVP_REAL(0.1));

    ivp_environment_destroy(env);
}

IVP_TEST(mindist_event_ignores_separating_pair)
{
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0), IVP_REAL(-2.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(2.2), IVP_REAL(2.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_mindist_entry_t *entry =
        ivp_mindist_manager_get_or_create(&env->mindist_manager, a, b);
    IVP_ASSERT_NOT_NULL(entry);

    seed_entry_from_contact(entry, a, b);

    ivp_collision_settings_t settings;
    ivp_collision_settings_init_default(&settings);

    ivp_mindist_event_prediction_t prediction;
    bool ok = ivp_mindist_event_predict(entry, &settings,
                                        IVP_REAL(0.0), IVP_REAL(0.1),
                                        &prediction);
    IVP_ASSERT_FALSE(ok);

    ivp_environment_destroy(env);
}

int main(void)
{
    IVP_REGISTER(mindist_event_predicts_impact);
    IVP_REGISTER(mindist_event_ignores_separating_pair);
    IVP_RUN_ALL();
}
