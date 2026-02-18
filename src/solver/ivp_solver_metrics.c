/* ivp_solver_metrics.c -- Per-PSI solver metrics implementation */

#include <ivp/solver/ivp_solver_metrics.h>
#include <string.h>

/* ── File-scope active-metrics pointer ───────────────────────────────── */

static ivp_solver_metrics_t *s_active_metrics = NULL;

void ivp_solver_metrics_set_active(ivp_solver_metrics_t *m)
{
    s_active_metrics = m;
}

ivp_solver_metrics_t *ivp_solver_metrics_get_active(void)
{
    return s_active_metrics;
}

/* ── Lifecycle ────────────────────────────────────────────────────────── */

void ivp_solver_metrics_init(ivp_solver_metrics_t *m)
{
    if (!m) return;
    memset(m, 0, sizeof(*m));
}
