/* test_qhull_overlap.c -- prove that reentrant hull builds can overlap. */
#include "ivp_test.h"
#include "ivp_qhull_api.h"

#include <string.h>
#include <time.h>

#ifdef _WIN32
#include <process.h>
#include <windows.h>
#else
#include <pthread.h>
#endif

#define QHULL_OVERLAP_THREADS 2
#define QHULL_OVERLAP_TIMEOUT_MS 5000

static const double qhull_overlap_points[] = {
    0.0,  0.0,  0.0,
    1.0,  0.0,  0.0,
    0.0,  1.0,  0.0,
    0.0,  0.0,  1.0,
    1.0,  1.0,  1.0,
    0.2,  0.2,  0.2,
    0.9,  0.1,  0.4,
   -0.3,  0.5,  0.6,
    0.4, -0.2,  0.8,
    0.6,  0.7, -0.1,
};

typedef struct qhull_overlap_gate {
#ifdef _WIN32
    CRITICAL_SECTION mutex;
    CONDITION_VARIABLE changed;
#else
    pthread_mutex_t mutex;
    pthread_cond_t changed;
#endif
    int entered;
    int release;
} qhull_overlap_gate_t;

typedef struct qhull_overlap_worker {
    double points[sizeof(qhull_overlap_points) / sizeof(double)];
    int result;
} qhull_overlap_worker_t;

static int qhull_overlap_gate_init(qhull_overlap_gate_t *gate)
{
    memset(gate, 0, sizeof(*gate));
#ifdef _WIN32
    InitializeCriticalSection(&gate->mutex);
    InitializeConditionVariable(&gate->changed);
    return 1;
#else
    if (pthread_mutex_init(&gate->mutex, NULL) != 0) return 0;
    if (pthread_cond_init(&gate->changed, NULL) != 0) {
        (void)pthread_mutex_destroy(&gate->mutex);
        return 0;
    }
    return 1;
#endif
}

static void qhull_overlap_gate_destroy(qhull_overlap_gate_t *gate)
{
#ifdef _WIN32
    DeleteCriticalSection(&gate->mutex);
#else
    (void)pthread_cond_destroy(&gate->changed);
    (void)pthread_mutex_destroy(&gate->mutex);
#endif
}

static void qhull_overlap_gate_enter(void *opaque)
{
    qhull_overlap_gate_t *gate = (qhull_overlap_gate_t *)opaque;
#ifdef _WIN32
    EnterCriticalSection(&gate->mutex);
    gate->entered++;
    WakeAllConditionVariable(&gate->changed);
    while (!gate->release)
        (void)SleepConditionVariableCS(&gate->changed, &gate->mutex, INFINITE);
    LeaveCriticalSection(&gate->mutex);
#else
    (void)pthread_mutex_lock(&gate->mutex);
    gate->entered++;
    (void)pthread_cond_broadcast(&gate->changed);
    while (!gate->release)
        (void)pthread_cond_wait(&gate->changed, &gate->mutex);
    (void)pthread_mutex_unlock(&gate->mutex);
#endif
}

static int qhull_overlap_gate_release_when_ready(qhull_overlap_gate_t *gate)
{
    int ready;
#ifdef _WIN32
    ULONGLONG deadline = GetTickCount64() + QHULL_OVERLAP_TIMEOUT_MS;
    EnterCriticalSection(&gate->mutex);
    while (gate->entered < QHULL_OVERLAP_THREADS) {
        ULONGLONG now = GetTickCount64();
        DWORD remaining = now < deadline ? (DWORD)(deadline - now) : 0u;
        if (remaining == 0u) break;
        if (!SleepConditionVariableCS(&gate->changed, &gate->mutex,
                                      remaining))
            break;
    }
    ready = gate->entered == QHULL_OVERLAP_THREADS;
    gate->release = 1;
    WakeAllConditionVariable(&gate->changed);
    LeaveCriticalSection(&gate->mutex);
#else
    struct timespec deadline = {0};
    (void)timespec_get(&deadline, TIME_UTC);
    deadline.tv_sec += QHULL_OVERLAP_TIMEOUT_MS / 1000;
    deadline.tv_nsec +=
        (long)(QHULL_OVERLAP_TIMEOUT_MS % 1000) * 1000000L;
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec++;
        deadline.tv_nsec -= 1000000000L;
    }

    (void)pthread_mutex_lock(&gate->mutex);
    while (gate->entered < QHULL_OVERLAP_THREADS) {
        int result = pthread_cond_timedwait(&gate->changed, &gate->mutex,
                                            &deadline);
        if (result != 0) break;
    }
    ready = gate->entered == QHULL_OVERLAP_THREADS;
    gate->release = 1;
    (void)pthread_cond_broadcast(&gate->changed);
    (void)pthread_mutex_unlock(&gate->mutex);
#endif
    return ready;
}

static void qhull_overlap_gate_release(qhull_overlap_gate_t *gate)
{
#ifdef _WIN32
    EnterCriticalSection(&gate->mutex);
    gate->release = 1;
    WakeAllConditionVariable(&gate->changed);
    LeaveCriticalSection(&gate->mutex);
#else
    (void)pthread_mutex_lock(&gate->mutex);
    gate->release = 1;
    (void)pthread_cond_broadcast(&gate->changed);
    (void)pthread_mutex_unlock(&gate->mutex);
#endif
}

static void qhull_overlap_build(qhull_overlap_worker_t *worker)
{
    ivp_qhull_context_t *context;

    memcpy(worker->points, qhull_overlap_points,
           sizeof(qhull_overlap_points));
    context = ivp_qhull_context_create();
    if (!context) {
        worker->result = -1;
        return;
    }
    worker->result = ivp_qhull_new_hull(
        context, worker->points, 10,
        "qhull Qs Pp C-0 W1e-14 E1.0e-18");
    ivp_qhull_context_destroy(context);
}

#ifdef _WIN32
static unsigned __stdcall qhull_overlap_thread_main(void *opaque)
{
    qhull_overlap_build((qhull_overlap_worker_t *)opaque);
    return 0;
}
#else
static void *qhull_overlap_thread_main(void *opaque)
{
    qhull_overlap_build((qhull_overlap_worker_t *)opaque);
    return NULL;
}
#endif

IVP_TEST(reentrant_builds_enter_backend_together)
{
    qhull_overlap_gate_t gate;
    qhull_overlap_worker_t workers[QHULL_OVERLAP_THREADS] = {0};
#ifdef _WIN32
    HANDLE threads[QHULL_OVERLAP_THREADS] = {0};
#else
    pthread_t threads[QHULL_OVERLAP_THREADS];
#endif
    int gate_ready = qhull_overlap_gate_init(&gate);
    int started = 0;
    int overlapped = 0;

    IVP_ASSERT_TRUE(gate_ready);
    ivp_qhull_set_test_before_build_hook(qhull_overlap_gate_enter, &gate);
    for (int i = 0; i < QHULL_OVERLAP_THREADS; i++) {
#ifdef _WIN32
        uintptr_t thread = _beginthreadex(NULL, 0, qhull_overlap_thread_main,
                                         &workers[i], 0, NULL);
        threads[i] = (HANDLE)thread;
        if (!threads[i]) break;
#else
        if (pthread_create(&threads[i], NULL, qhull_overlap_thread_main,
                           &workers[i]) != 0)
            break;
#endif
        started++;
    }

    if (started == QHULL_OVERLAP_THREADS)
        overlapped = qhull_overlap_gate_release_when_ready(&gate);
    else
        qhull_overlap_gate_release(&gate);

    for (int i = 0; i < started; i++) {
#ifdef _WIN32
        (void)WaitForSingleObject(threads[i], INFINITE);
        CloseHandle(threads[i]);
#else
        (void)pthread_join(threads[i], NULL);
#endif
    }
    ivp_qhull_set_test_before_build_hook(NULL, NULL);
    qhull_overlap_gate_destroy(&gate);

    IVP_ASSERT_EQ(started, QHULL_OVERLAP_THREADS);
    IVP_ASSERT_TRUE(overlapped);
    for (int i = 0; i < QHULL_OVERLAP_THREADS; i++)
        IVP_ASSERT_EQ(workers[i].result, 0);
}

int main(void)
{
    IVP_REGISTER(reentrant_builds_enter_backend_together);
    IVP_RUN_ALL();
}
