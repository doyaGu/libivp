/* test_qhull_concurrency.c -- concurrent callers receive isolated results.
 *
 * The mainline backend achieves this with independent qhT state.  The
 * reference backend serializes its classic process-global Qhull state.
 */
#include "ivp_test.h"

#include <ivp/builder/ivp_build_surface.h>
#include <ivp/geometry/ivp_compact_surface.h>

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef _WIN32
#include <process.h>
#include <windows.h>
#else
#include <pthread.h>
#endif

#define QHULL_THREAD_COUNT 8
#define QHULL_BUILDS_PER_THREAD 64

static const double qhull_test_points[] = {
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

typedef struct qhull_thread_args {
    const ivp_compact_surface_t *reference;
    int reference_size;
    int failed;
} qhull_thread_args_t;

static void qhull_build_worker(qhull_thread_args_t *args)
{
    for (int iteration = 0; iteration < QHULL_BUILDS_PER_THREAD; iteration++) {
        ivp_compact_surface_t *surface =
            ivp_build_convex_surface_d(NULL, qhull_test_points, 10);
        if (!surface ||
            ivp_surface_get_byte_size(surface) != args->reference_size ||
            memcmp(surface, args->reference, (size_t)args->reference_size) != 0) {
            args->failed = 1;
        }
        if (surface) ivp_compact_surface_free(NULL, surface);
        if (args->failed) return;
    }
}

#ifdef _WIN32
static unsigned __stdcall qhull_thread_main(void *opaque)
{
    qhull_build_worker((qhull_thread_args_t *)opaque);
    return 0;
}
#else
static void *qhull_thread_main(void *opaque)
{
    qhull_build_worker((qhull_thread_args_t *)opaque);
    return NULL;
}
#endif

IVP_TEST(concurrent_convex_builds_match_serial_result)
{
    ivp_compact_surface_t *reference =
        ivp_build_convex_surface_d(NULL, qhull_test_points, 10);
    IVP_ASSERT_NOT_NULL(reference);

    qhull_thread_args_t args[QHULL_THREAD_COUNT];
#ifdef _WIN32
    HANDLE threads[QHULL_THREAD_COUNT];
#else
    pthread_t threads[QHULL_THREAD_COUNT];
#endif
    int started = 0;

    for (int i = 0; i < QHULL_THREAD_COUNT; i++) {
        args[i].reference = reference;
        args[i].reference_size = ivp_surface_get_byte_size(reference);
        args[i].failed = 0;
#ifdef _WIN32
        uintptr_t thread = _beginthreadex(NULL, 0, qhull_thread_main,
                                         &args[i], 0, NULL);
        threads[i] = (HANDLE)thread;
        if (!threads[i]) break;
#else
        if (pthread_create(&threads[i], NULL, qhull_thread_main, &args[i]) != 0) break;
#endif
        started++;
    }

    IVP_ASSERT_EQ(started, QHULL_THREAD_COUNT);
    for (int i = 0; i < started; i++) {
#ifdef _WIN32
        WaitForSingleObject(threads[i], INFINITE);
        CloseHandle(threads[i]);
#else
        (void)pthread_join(threads[i], NULL);
#endif
        IVP_ASSERT_EQ(args[i].failed, 0);
    }

    ivp_compact_surface_free(NULL, reference);
}

int main(void)
{
    IVP_REGISTER(concurrent_convex_builds_match_serial_result);
    IVP_RUN_ALL();
}
