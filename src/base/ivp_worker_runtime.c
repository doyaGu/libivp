/* ivp_worker_runtime.c -- production adapter for worker platform services. */
#include "base/ivp_worker_runtime.h"

#if defined(_WIN32)
#include <process.h>
#include <stdint.h>
#include <windows.h>

static unsigned __stdcall worker_thread_main(void *opaque)
{
    ivp_worker_thread_t *thread = (ivp_worker_thread_t *)opaque;
    thread->entry(thread->entry_context);
    return 0;
}
#else
static void *worker_thread_main(void *opaque)
{
    ivp_worker_thread_t *thread = (ivp_worker_thread_t *)opaque;
    thread->entry(thread->entry_context);
    return NULL;
}
#endif

static bool worker_thread_start(void *context, ivp_worker_thread_t *thread,
                                ivp_worker_entry_fn entry,
                                void *entry_context)
{
    (void)context;
    thread->entry = entry;
    thread->entry_context = entry_context;
#if defined(_WIN32)
    uintptr_t handle = _beginthreadex(NULL, 0, worker_thread_main,
                                      thread, 0, NULL);
    thread->handle = (void *)handle;
    return thread->handle != NULL;
#else
    return pthread_create(&thread->handle, NULL, worker_thread_main,
                          thread) == 0;
#endif
}

static void worker_thread_join(void *context, ivp_worker_thread_t *thread)
{
    (void)context;
#if defined(_WIN32)
    (void)WaitForSingleObject((HANDLE)thread->handle, INFINITE);
    (void)CloseHandle((HANDLE)thread->handle);
    thread->handle = NULL;
#else
    (void)pthread_join(thread->handle, NULL);
#endif
}

static bool worker_floating_point_capture(void *context,
                                          fenv_t *environment_out)
{
    (void)context;
    return fegetenv(environment_out) == 0;
}

static bool worker_floating_point_apply(void *context,
                                        const fenv_t *environment)
{
    (void)context;
    return fesetenv(environment) == 0;
}

static const ivp_worker_runtime_t s_default_runtime = {
    .context = NULL,
    .thread_start = worker_thread_start,
    .thread_join = worker_thread_join,
    .floating_point_capture = worker_floating_point_capture,
    .floating_point_apply = worker_floating_point_apply
};

const ivp_worker_runtime_t *ivp_worker_runtime_default(void)
{
    return &s_default_runtime;
}

bool ivp_worker_runtime_is_valid(const ivp_worker_runtime_t *runtime)
{
    return runtime && runtime->thread_start && runtime->thread_join &&
           runtime->floating_point_capture && runtime->floating_point_apply;
}
