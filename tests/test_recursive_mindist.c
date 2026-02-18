#include "ivp_test.h"

#include <ivp/collision/ivp_mindist_recursive.h>
#include <ivp/collision/ivp_oo_watcher.h>
#include <ivp/dynamics/ivp_environment.h>

#include <string.h>

IVP_TEST(recursive_mindist_add_remove)
{
    ivp_mindist_recursive_t recursive;
    ivp_mindist_recursive_init(&recursive, NULL);

    ivp_mindist_entry_t entry_a;
    ivp_mindist_entry_t entry_b;
    memset(&entry_a, 0, sizeof(entry_a));
    memset(&entry_b, 0, sizeof(entry_b));
    entry_a.state = IVP_MDIST_EXACT;
    entry_b.state = IVP_MDIST_EXACT;

    IVP_ASSERT_TRUE(ivp_mindist_recursive_add_child(&recursive, &entry_a));
    IVP_ASSERT_TRUE(ivp_mindist_recursive_add_child(&recursive, &entry_b));
    IVP_ASSERT_EQ(recursive.child_count, 2);
    IVP_ASSERT_EQ(entry_a.state, IVP_MDIST_HULL_RECURSIVE);
    IVP_ASSERT_EQ(entry_b.state, IVP_MDIST_HULL_RECURSIVE);

    ivp_mindist_recursive_remove_child(&recursive, &entry_a);
    IVP_ASSERT_EQ(recursive.child_count, 1);
    IVP_ASSERT_EQ(entry_a.state, IVP_MDIST_INVALID);

    ivp_mindist_recursive_destroy(&recursive);
}

IVP_TEST(oo_watcher_wakeup_invalidates_children)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_environment_t *env = ivp_environment_create(&cfg);
    IVP_ASSERT_NOT_NULL(env);

    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.mass = IVP_REAL(1.0);

    ivp_object_t *a = ivp_environment_create_object(env, &tmpl, NULL);
    ivp_object_t *b = ivp_environment_create_object(env, &tmpl, NULL);
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_oo_watcher_t watcher;
    ivp_oo_watcher_init(&watcher, a, b, env->allocator);

    ivp_mindist_entry_t entry;
    memset(&entry, 0, sizeof(entry));
    entry.state = IVP_MDIST_EXACT;
    IVP_ASSERT_TRUE(ivp_mindist_recursive_add_child(&watcher.recursive, &entry));

    ivp_oo_watcher_hull_wakeup(&watcher);
    IVP_ASSERT_EQ(entry.state, IVP_MDIST_INVALID);

    ivp_oo_watcher_destroy(&watcher);
    ivp_environment_destroy(env);
}

int main(void)
{
    IVP_REGISTER(recursive_mindist_add_remove);
    IVP_REGISTER(oo_watcher_wakeup_invalidates_children);
    IVP_RUN_ALL();
}
