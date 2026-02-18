/* ivp_listener_psi.c — Per-PSI step listener implementation */

#include <ivp/events/ivp_listener_psi.h>
#include <ivp/dynamics/ivp_environment.h>
#include <string.h>

#define IVP_PSI_LISTENER_MAGIC 0x5053494Cu /* "PSIL" */

static void ivp_psi_listener_reset(ivp_psi_listener_t *listener)
{
    memset(listener, 0, sizeof(*listener));
    listener->event_handle = ivp_event_listener_handle_invalid();
    listener->registered_env = NULL;
    listener->active = false;
    listener->_magic = IVP_PSI_LISTENER_MAGIC;
}

static bool ivp_psi_listener_is_initialized(const ivp_psi_listener_t *listener)
{
    return listener && listener->_magic == IVP_PSI_LISTENER_MAGIC;
}

/* ── Internal event callback ───────────────────────────────────────────── */

static void psi_event_adapter(void *ctx,
                              struct ivp_environment *env,
                              const ivp_event_t *event_data)
{
    ivp_psi_listener_t *listener = (ivp_psi_listener_t *)ctx;
    if (!listener || !listener->active || !listener->fn) return;

    /* Only fire on SIM_UNIT_BEGIN events */
    if (event_data->type != IVP_EVENT_SIM_UNIT_BEGIN) return;

    listener->fn(listener->ctx, env, event_data->time, event_data->dt);
}

/* ── API ───────────────────────────────────────────────────────────────── */

int ivp_psi_listener_register(ivp_psi_listener_t *listener,
                              ivp_environment_t *env,
                              ivp_psi_listener_fn_t fn,
                              void *ctx)
{
    if (!listener || !env || !fn) return -1;

    if (!ivp_psi_listener_is_initialized(listener)) {
        ivp_psi_listener_reset(listener);
    }

    /* Prevent orphaned registrations: caller must unregister first. */
    if (listener->active && ivp_event_listener_handle_is_valid(listener->event_handle)) {
        return -1;
    }

    listener->fn = fn;
    listener->ctx = ctx;
    listener->active = true;
    listener->registered_env = NULL;
    listener->event_handle = ivp_event_listener_handle_invalid();

    /* Register as a generic event listener, filtering in the adapter */
    ivp_event_listener_handle_t h =
        ivp_environment_add_event_listener(env, psi_event_adapter, listener);

    if (!ivp_event_listener_handle_is_valid(h)) {
        listener->active = false;
        listener->fn = NULL;
        listener->ctx = NULL;
        return -1;
    }

    listener->event_handle = h;
    listener->registered_env = env;
    return 0;
}

int ivp_psi_listener_unregister(ivp_psi_listener_t *listener,
                                ivp_environment_t *env)
{
    if (!listener || !env) return -1;
    if (!ivp_psi_listener_is_initialized(listener)) return -1;
    if (!ivp_event_listener_handle_is_valid(listener->event_handle)) return -1;
    if (listener->registered_env && listener->registered_env != env) return -1;

    bool removed = ivp_environment_remove_event_listener(env, listener->event_handle);
    if (!removed) return -1;

    listener->active = false;
    listener->event_handle = ivp_event_listener_handle_invalid();
    listener->registered_env = NULL;
    listener->fn = NULL;
    listener->ctx = NULL;
    return 0;
}
