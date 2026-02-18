#include "ivp_test.h"

#include <ivp/collision/ivp_mindist_persistent.h>
#include <ivp/collision/ivp_synapse.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/solver/ivp_contact.h>
#include <ivp/solver/ivp_friction_system.h>

#include <string.h>

static ivp_environment_t *make_env(void)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create(&cfg);
}

static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t x)
{
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.position, x, IVP_REAL(0.0), IVP_REAL(0.0));
    return ivp_environment_create_object(env, &tmpl, NULL);
}

IVP_TEST(synapse_mindist_attach_and_reverse_lookup)
{
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(2.2));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_mindist_entry_t *entry =
        ivp_mindist_manager_get_or_create(&env->mindist_manager, a, b);
    IVP_ASSERT_NOT_NULL(entry);

    IVP_ASSERT_NOT_NULL(a->mindist_synapses);
    IVP_ASSERT_NOT_NULL(b->mindist_synapses);

    ivp_mindist_entry_t *back_a = ivp_synapse_link_get_mindist(a->mindist_synapses);
    ivp_mindist_entry_t *back_b = ivp_synapse_link_get_mindist(b->mindist_synapses);
    IVP_ASSERT_EQ(back_a, entry);
    IVP_ASSERT_EQ(back_b, entry);

    ivp_environment_destroy(env);
}

IVP_TEST(synapse_friction_attach_and_reverse_lookup)
{
    ivp_environment_t *env = make_env();
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_t *a = make_ball(env, IVP_REAL(0.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.9));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_contact_point_t cp;
    memset(&cp, 0, sizeof(cp));
    cp.obj_a = a;
    cp.obj_b = b;
    ivp_contact_compute_sphere_sphere(&cp);
    ivp_contact_combine_materials(&cp);
    ivp_contact_compute_virtual_mass(&cp);

    ivp_friction_manager_begin_psi(&env->friction_manager);
    ivp_persistent_contact_t *pc =
        ivp_friction_manager_match_or_create(&env->friction_manager, &cp);
    IVP_ASSERT_NOT_NULL(pc);

    IVP_ASSERT_NOT_NULL(a->friction_synapses);
    IVP_ASSERT_NOT_NULL(b->friction_synapses);

    ivp_persistent_contact_t *back_a =
        ivp_synapse_link_get_friction_contact(a->friction_synapses);
    ivp_persistent_contact_t *back_b =
        ivp_synapse_link_get_friction_contact(b->friction_synapses);
    IVP_ASSERT_EQ(back_a, pc);
    IVP_ASSERT_EQ(back_b, pc);

    ivp_environment_destroy(env);
}

int main(void)
{
    IVP_REGISTER(synapse_mindist_attach_and_reverse_lookup);
    IVP_REGISTER(synapse_friction_attach_and_reverse_lookup);
    IVP_RUN_ALL();
}
