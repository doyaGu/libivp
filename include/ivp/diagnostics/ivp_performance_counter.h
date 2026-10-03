/* ivp_performance_counter.h — IVP_PerformanceCounter / IVP_PerformanceCounter_Simple
 *
 * Port of ivp_physics/ivp_performancecounter.{hxx,cxx}.
 *
 * The environment owns a counter (ivp_environment_get_performance_counter):
 * the one given in ivp_environment_config_t::performance_counter, else a
 * heap allocated ivp_performance_counter_simple_t.  The engine calls it at the
 * reference places:
 *   - start_pcount / stop_pcount around every time manager run
 *     (ivp_environment_simulate_until / _dtime / _one_psi and
 *     ivp_environment_simulate_variable_time_step);
 *   - pcount(IVP_PE_PSI_UNIVERSE ... IVP_PE_PSI_END) between the phases of
 *     every PSI;
 *   - pcount(IVP_PE_AT_INIT) / pcount(IVP_PE_AT_END) around every mindist
 *     collision time event;
 *   - environment_is_going_to_be_deleted when the environment is destroyed.
 * pcount(el) means "the time since the previous pcount belongs to the element
 * passed to the previous pcount; from now on time is counted for el".
 * Counters only measure time: they never influence the simulation.
 */
#ifndef IVP_PERFORMANCE_COUNTER_H
#define IVP_PERFORMANCE_COUNTER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_time.h>
#include <ivp/base/ivp_allocator.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_environment;

/* The reference compiles the engine call sites only when this is defined
 * (always, ivp_performancecounter.hxx). */
#define IVP_ENABLE_PERFORMANCE_COUNTER

/* IVP_PERFORMANCE_ELEMENT */
typedef enum ivp_performance_element {
    IVP_PE_PSI_START = 0, /* not used by pcount */

    IVP_PE_PSI_UNIVERSE,
    IVP_PE_PSI_CONTROLLERS,
    IVP_PE_PSI_INTEGRATORS,
    IVP_PE_PSI_HULL,
    IVP_PE_PSI_SHORT_MINDISTS,
    IVP_PE_PSI_CRITICAL_MINDISTS,
    IVP_PE_PSI_END,

    IVP_PE_AT_INIT,
    IVP_PE_AT_COLLISION,
    IVP_PE_AT_INTEGRATORS,
    IVP_PE_AT_HULL,
    IVP_PE_AT_SHORT_MINDISTS,
    IVP_PE_AT_CRITICAL_MINDISTS,
    IVP_PE_AT_END,

    IVP_PE_USR1,
    IVP_PE_USR2,

    IVP_PE_MAX
} ivp_performance_element_t;

/* ── IVP_PerformanceCounter (abstract) ─────────────────────────────────── */

typedef struct ivp_performance_counter ivp_performance_counter_t;

typedef struct ivp_performance_counter_vtable {
    void (*start_pcount)(ivp_performance_counter_t *pc);
    void (*pcount)(ivp_performance_counter_t *pc, ivp_performance_element_t el);
    void (*stop_pcount)(ivp_performance_counter_t *pc);
    /* Called once from ivp_environment_destroy (after the objects and the
     * managers are gone).  The counter may free itself here. */
    void (*environment_is_going_to_be_deleted)(ivp_performance_counter_t *pc, struct ivp_environment *env);
    void (*reset_and_print_performance_counters)(ivp_performance_counter_t *pc, ivp_time_t current_time);
} ivp_performance_counter_vtable_t;

/* Base of every counter: embed it as the first member and point vt at a
 * table with all five functions set. */
struct ivp_performance_counter {
    const ivp_performance_counter_vtable_t *vt;
};

static inline void ivp_performance_counter_start_pcount(ivp_performance_counter_t *pc) {
    pc->vt->start_pcount(pc);
}
static inline void ivp_performance_counter_pcount(ivp_performance_counter_t *pc, ivp_performance_element_t el) {
    pc->vt->pcount(pc, el);
}
static inline void ivp_performance_counter_stop_pcount(ivp_performance_counter_t *pc) {
    pc->vt->stop_pcount(pc);
}
static inline void ivp_performance_counter_environment_is_going_to_be_deleted(ivp_performance_counter_t *pc,
                                                                              struct ivp_environment *env) {
    pc->vt->environment_is_going_to_be_deleted(pc, env);
}
static inline void ivp_performance_counter_reset_and_print_performance_counters(ivp_performance_counter_t *pc,
                                                                                ivp_time_t current_time) {
    pc->vt->reset_and_print_performance_counters(pc, current_time);
}

/* ── IVP_PerformanceCounter_Simple ─────────────────────────────────────── *
 * Wall clock timing (QueryPerformanceCounter on Windows, CLOCK_MONOTONIC
 * elsewhere; the reference only times on WIN32 and is empty elsewhere).
 *   pcount(el):   if el == IVP_PE_PSI_UNIVERSE count_PSIs++;
 *                 counter[counting] += 1e6 * seconds since the last pcount;
 *                 counting = el.
 *   start_pcount: counting = IVP_PE_PSI_START (time until the first pcount
 *                 is booked to IVP_PE_PSI_START).
 *   stop_pcount:  nothing.
 *   reset_and_print_performance_counters: prints the averages per PSI (see
 *                 below) with ivp_message (engine output, "[havok] "
 *                 prefix, like the reference) or to `out`, then clears the
 *                 counters (nothing is printed or reset when no PSI was
 *                 counted).
 *   environment_is_going_to_be_deleted: frees the counter when it was made
 *                 by ivp_performance_counter_simple_create (P_DELETE_THIS);
 *                 an in-place initialised counter is left alone. */
typedef struct ivp_performance_counter_simple {
    ivp_performance_counter_t base;

    double   inv_counter_freq;          /* seconds per tick                   */
    int64_t  ref_counter64;             /* ticks at the last pcount / reset   */

    unsigned int              count_PSIs;
    ivp_performance_element_t counting;

    ivp_real_t counter[IVP_PE_MAX];     /* microseconds (seconds * 1e6)       */
    ivp_time_t time_of_last_reset;

    /* libivp: report destination.  stdout (after init) or NULL: the engine
     * output through ivp_message, like the reference; any other FILE gets the
     * report text without ivp_message's "[havok] " prefix. */
    FILE *out;

    /* internal: allocator of ivp_performance_counter_simple_create, NULL for
     * an in-place initialised counter */
    const ivp_allocator_t *owner_allocator;
} ivp_performance_counter_simple_t;

/* new IVP_PerformanceCounter_Simple(): heap counter freed by
 * environment_is_going_to_be_deleted.  alloc NULL = default allocator. */
ivp_performance_counter_simple_t *ivp_performance_counter_simple_create(const ivp_allocator_t *alloc);

/* Free a counter made by ivp_performance_counter_simple_create that is not
 * handed to an environment (no-op for in-place counters). */
void ivp_performance_counter_simple_destroy(ivp_performance_counter_simple_t *pc);

/* In-place constructor (caller owned storage, never freed by the engine). */
void ivp_performance_counter_simple_init(ivp_performance_counter_simple_t *pc);

/* The report printed by reset_and_print_performance_counters, as in the
 * reference (sums of counter[] divided by count_PSIs, in seconds):
 *   "[performance] TOTAL %2.1f%% %2.2fs\nCOLLISION %2.2fs\nDYNAMIC %2.2fs\n\n"
 *   "UNIVERSE: %2.2fs\nCONTROLLERS: %2.2fs\nINTEGRATORS: %2.2fs\n"
 *   "HULL: %2.2fs\nSHORT MINDISTS: %2.2fs\nCRITICAL MINDISTS: %2.2fs\nSIMULATE %2.2fs\n"
 * COLLISION = UNIVERSE + SHORT + CRITICAL + HULL + AT_INIT,
 * DYNAMIC = CONTROLLERS + INTEGRATORS, TOTAL = COLLISION + DYNAMIC. */

#ifdef __cplusplus
}
#endif

#endif /* IVP_PERFORMANCE_COUNTER_H */
