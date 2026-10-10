/* ivp_worker_group.c -- shared persistent worker lifecycle and rendezvous. */
#include "base/ivp_worker_group.h"

#include <ivp/base/ivp_assert.h>

#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#if defined(_WIN32)
#include <windows.h>
typedef CRITICAL_SECTION ivp_worker_mutex_t;
typedef CONDITION_VARIABLE ivp_worker_cond_t;
#else
#include <pthread.h>
#include <sched.h>
typedef pthread_mutex_t ivp_worker_mutex_t;
typedef pthread_cond_t ivp_worker_cond_t;
#endif

typedef struct ivp_worker_lane {
    struct ivp_worker_group *group;
    int index;
} ivp_worker_lane_t;

struct ivp_worker_group {
    ivp_allocator_t allocator;
    const ivp_worker_runtime_t *runtime;
    ivp_worker_thread_t *threads;
    ivp_worker_lane_t *lanes;
    int thread_count;

    ivp_worker_mutex_t mutex;
    ivp_worker_cond_t work_ready;
    ivp_worker_cond_t state_changed;
    bool stopping;
    bool running;
    uint64_t generation;
    atomic_int setup_ready_count;
    atomic_bool setup_failed;
    int done_count;

    ivp_worker_group_fn function;
    void *context;
    fenv_t floating_point_environment;
};

#if defined(_WIN32)
static bool group_sync_init(ivp_worker_group_t *group)
{
    InitializeCriticalSection(&group->mutex);
    InitializeConditionVariable(&group->work_ready);
    InitializeConditionVariable(&group->state_changed);
    return true;
}
static void group_sync_destroy(ivp_worker_group_t *group)
{
    DeleteCriticalSection(&group->mutex);
}
static void group_lock(ivp_worker_group_t *group)
{
    EnterCriticalSection(&group->mutex);
}
static void group_unlock(ivp_worker_group_t *group)
{
    LeaveCriticalSection(&group->mutex);
}
static void group_wait(ivp_worker_cond_t *condition,
                       ivp_worker_group_t *group)
{
    (void)SleepConditionVariableCS(condition, &group->mutex, INFINITE);
}
static void group_broadcast(ivp_worker_cond_t *condition)
{
    WakeAllConditionVariable(condition);
}
static void group_yield(void) { (void)SwitchToThread(); }
#else
static bool group_sync_init(ivp_worker_group_t *group)
{
    if (pthread_mutex_init(&group->mutex, NULL) != 0) return false;
    if (pthread_cond_init(&group->work_ready, NULL) != 0) {
        (void)pthread_mutex_destroy(&group->mutex);
        return false;
    }
    if (pthread_cond_init(&group->state_changed, NULL) != 0) {
        (void)pthread_cond_destroy(&group->work_ready);
        (void)pthread_mutex_destroy(&group->mutex);
        return false;
    }
    return true;
}
static void group_sync_destroy(ivp_worker_group_t *group)
{
    (void)pthread_cond_destroy(&group->state_changed);
    (void)pthread_cond_destroy(&group->work_ready);
    (void)pthread_mutex_destroy(&group->mutex);
}
static void group_lock(ivp_worker_group_t *group)
{
    (void)pthread_mutex_lock(&group->mutex);
}
static void group_unlock(ivp_worker_group_t *group)
{
    (void)pthread_mutex_unlock(&group->mutex);
}
static void group_wait(ivp_worker_cond_t *condition,
                       ivp_worker_group_t *group)
{
    (void)pthread_cond_wait(condition, &group->mutex);
}
static void group_broadcast(ivp_worker_cond_t *condition)
{
    (void)pthread_cond_broadcast(condition);
}
static void group_yield(void) { (void)sched_yield(); }
#endif

static void group_worker_loop(ivp_worker_lane_t *lane)
{
    ivp_worker_group_t *group = lane->group;
    uint64_t observed_generation = 0;
    group_lock(group);
    for (;;) {
        while (!group->stopping &&
               observed_generation == group->generation) {
            group_wait(&group->work_ready, group);
        }
        if (group->stopping) break;
        observed_generation = group->generation;

        ivp_worker_group_fn function = group->function;
        void *context = group->context;
        const fenv_t floating_point_environment =
            group->floating_point_environment;
        group_unlock(group);

        if (!group->runtime->floating_point_apply(
                group->runtime->context, &floating_point_environment)) {
            atomic_store_explicit(&group->setup_failed, true,
                                  memory_order_release);
        }
        int ready = atomic_fetch_add_explicit(
                        &group->setup_ready_count, 1,
                        memory_order_acq_rel) + 1;
        if (ready == group->thread_count) {
            group_lock(group);
            group_broadcast(&group->state_changed);
            group_unlock(group);
        } else {
            unsigned int spins = 0;
            while (atomic_load_explicit(&group->setup_ready_count,
                                        memory_order_acquire) !=
                   group->thread_count) {
                if ((++spins & 63u) == 0u) group_yield();
            }
        }

        if (!atomic_load_explicit(&group->setup_failed,
                                  memory_order_acquire)) {
            function(context, lane->index);
        }

        group_lock(group);
        group->done_count++;
        if (group->done_count == group->thread_count) {
            group_broadcast(&group->state_changed);
        }
    }
    group_unlock(group);
}

static void group_worker_main(void *opaque)
{
    group_worker_loop((ivp_worker_lane_t *)opaque);
}

static void group_join_threads(ivp_worker_group_t *group, int count)
{
    for (int i = 0; i < count; i++) {
        group->runtime->thread_join(group->runtime->context,
                                    &group->threads[i]);
    }
}

ivp_worker_group_t *ivp_worker_group_create(
    int background_thread_count, const ivp_allocator_t *allocator,
    const ivp_worker_runtime_t *runtime)
{
    if (background_thread_count < 0 ||
        !ivp_worker_runtime_is_valid(runtime)) {
        return NULL;
    }
    if ((size_t)background_thread_count >
            SIZE_MAX / sizeof(ivp_worker_thread_t) ||
        (size_t)background_thread_count >
            SIZE_MAX / sizeof(ivp_worker_lane_t)) {
        return NULL;
    }
    if (!allocator) allocator = ivp_default_allocator();
    ivp_worker_group_t *group =
        (ivp_worker_group_t *)ivp_alloc(allocator, sizeof(*group));
    if (!group) return NULL;
    memset(group, 0, sizeof(*group));
    group->allocator = *allocator;
    group->runtime = runtime;
    group->thread_count = background_thread_count;
    atomic_init(&group->setup_ready_count, 0);
    atomic_init(&group->setup_failed, false);

    if (background_thread_count > 0) {
        group->threads = (ivp_worker_thread_t *)ivp_alloc(
            &group->allocator,
            (size_t)background_thread_count * sizeof(*group->threads));
        group->lanes = (ivp_worker_lane_t *)ivp_alloc(
            &group->allocator,
            (size_t)background_thread_count * sizeof(*group->lanes));
    }
    if ((background_thread_count > 0 &&
         (!group->threads || !group->lanes)) ||
        !group_sync_init(group)) {
        if (group->lanes) {
            ivp_free(&group->allocator, group->lanes,
                     (size_t)background_thread_count *
                         sizeof(*group->lanes));
        }
        if (group->threads) {
            ivp_free(&group->allocator, group->threads,
                     (size_t)background_thread_count *
                         sizeof(*group->threads));
        }
        ivp_free(&group->allocator, group, sizeof(*group));
        return NULL;
    }

    int created = 0;
    for (; created < background_thread_count; created++) {
        group->lanes[created].group = group;
        group->lanes[created].index = created;
        if (!group->runtime->thread_start(
                group->runtime->context, &group->threads[created],
                group_worker_main, &group->lanes[created])) {
            break;
        }
    }
    if (created != background_thread_count) {
        group_lock(group);
        group->stopping = true;
        group_broadcast(&group->work_ready);
        group_unlock(group);
        group_join_threads(group, created);
        group_sync_destroy(group);
        ivp_free(&group->allocator, group->lanes,
                 (size_t)background_thread_count * sizeof(*group->lanes));
        ivp_free(&group->allocator, group->threads,
                 (size_t)background_thread_count * sizeof(*group->threads));
        ivp_free(&group->allocator, group, sizeof(*group));
        return NULL;
    }
    return group;
}

void ivp_worker_group_destroy(ivp_worker_group_t *group)
{
    if (!group) return;
    group_lock(group);
    IVP_ASSERT(!group->running);
    while (group->running) group_wait(&group->state_changed, group);
    group->stopping = true;
    group_broadcast(&group->work_ready);
    group_unlock(group);
    group_join_threads(group, group->thread_count);
    group_sync_destroy(group);
    if (group->lanes) {
        ivp_free(&group->allocator, group->lanes,
                 (size_t)group->thread_count * sizeof(*group->lanes));
    }
    if (group->threads) {
        ivp_free(&group->allocator, group->threads,
                 (size_t)group->thread_count * sizeof(*group->threads));
    }
    ivp_free(&group->allocator, group, sizeof(*group));
}

int ivp_worker_group_background_thread_count(const ivp_worker_group_t *group)
{
    return group ? group->thread_count : 0;
}

bool ivp_worker_group_dispatch(ivp_worker_group_t *group,
                               bool caller_participates,
                               ivp_worker_group_fn function, void *context)
{
    if (!group || !function) return false;
    group_lock(group);
    if (group->running || group->stopping) {
        group_unlock(group);
        return false;
    }
    if (group->thread_count > 0 &&
        !group->runtime->floating_point_capture(
            group->runtime->context,
            &group->floating_point_environment)) {
        group_unlock(group);
        return false;
    }

    group->function = function;
    group->context = context;
    atomic_store_explicit(&group->setup_ready_count, 0,
                          memory_order_relaxed);
    atomic_store_explicit(&group->setup_failed, false,
                          memory_order_relaxed);
    group->done_count = 0;
    group->running = true;
    if (group->thread_count > 0) {
        group->generation++;
        group_broadcast(&group->work_ready);
        while (atomic_load_explicit(&group->setup_ready_count,
                                    memory_order_acquire) !=
               group->thread_count) {
            group_wait(&group->state_changed, group);
        }
    }
    bool setup_ok = !atomic_load_explicit(&group->setup_failed,
                                          memory_order_acquire);
    group_unlock(group);

    if (setup_ok && caller_participates) {
        function(context, group->thread_count);
    }

    group_lock(group);
    while (group->done_count != group->thread_count) {
        group_wait(&group->state_changed, group);
    }
    group->running = false;
    group->function = NULL;
    group->context = NULL;
    group_broadcast(&group->state_changed);
    group_unlock(group);
    return setup_ok;
}
