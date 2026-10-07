/* ivp_worker_runtime.h -- private OS-thread and floating-point adapter. */
#ifndef IVP_WORKER_RUNTIME_H
#define IVP_WORKER_RUNTIME_H

#include <fenv.h>
#include <stdbool.h>

#if !defined(_WIN32)
#include <pthread.h>
#endif

typedef void (*ivp_worker_entry_fn)(void *context);

typedef struct ivp_worker_thread {
#if defined(_WIN32)
    void *handle;
#else
    pthread_t handle;
#endif
    ivp_worker_entry_fn entry;
    void *entry_context;
} ivp_worker_thread_t;

/* This is an internal seam.  Production uses the default adapter; tests may
 * provide a delegating adapter that injects thread or floating-point setup
 * failures without changing the public scheduler/environment interfaces. */
typedef struct ivp_worker_runtime {
    void *context;
    bool (*thread_start)(void *context, ivp_worker_thread_t *thread,
                         ivp_worker_entry_fn entry, void *entry_context);
    void (*thread_join)(void *context, ivp_worker_thread_t *thread);
    bool (*floating_point_capture)(void *context, fenv_t *environment_out);
    bool (*floating_point_apply)(void *context,
                                 const fenv_t *environment);
} ivp_worker_runtime_t;

const ivp_worker_runtime_t *ivp_worker_runtime_default(void);
bool ivp_worker_runtime_is_valid(const ivp_worker_runtime_t *runtime);

#endif /* IVP_WORKER_RUNTIME_H */
