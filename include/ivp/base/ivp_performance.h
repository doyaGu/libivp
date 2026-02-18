/* ivp_performance.h -- Runtime performance counters
 *
 * Tracks simulation statistics:
 *   - PSI step count
 *   - Object count (peak and current)
 *   - Collision detection counts
 *   - Contact resolution counts
 *   - Controller execution counts
 *   - Total kinetic energy
 *
 * All counters are plain integers / doubles for thread safety
 * (no locks needed for single-writer / single-reader).
 *
 * Ported from ivp_performancecounter.hxx + ivp_betterstatisticsmanager.hxx.
 */
#ifndef IVP_PERFORMANCE_H
#define IVP_PERFORMANCE_H

#include <ivp/base/ivp_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Performance counters ──────────────────────────────────────────────── */

typedef struct ivp_perf_counters {
    /* Cumulative */
    uint64_t psi_count;            /* total PSI steps executed */
    uint64_t broadphase_pairs;     /* total broadphase pair tests */
    uint64_t narrowphase_tests;    /* total narrowphase tests */
    uint64_t contacts_resolved;    /* total contacts resolved */
    uint64_t controller_calls;     /* total controller callback invocations */

    /* Per-PSI (reset each step) */
    int      objects_active;       /* active objects this PSI */
    int      objects_sleeping;     /* sleeping objects this PSI */
    int      contacts_this_psi;    /* contacts found this PSI */
    int      controllers_this_psi; /* controllers run this PSI */

    /* Peaks */
    int      peak_objects;         /* peak object count */
    int      peak_contacts;        /* peak contacts per PSI */

    /* Energy */
    double   total_kinetic_energy; /* total KE at end of last PSI */
    double   peak_kinetic_energy;  /* peak KE observed */
} ivp_perf_counters_t;

/* ── API ───────────────────────────────────────────────────────────────── */

/* Reset all counters to zero. */
static inline void ivp_perf_reset(ivp_perf_counters_t *p)
{
    p->psi_count = 0;
    p->broadphase_pairs = 0;
    p->narrowphase_tests = 0;
    p->contacts_resolved = 0;
    p->controller_calls = 0;
    p->objects_active = 0;
    p->objects_sleeping = 0;
    p->contacts_this_psi = 0;
    p->controllers_this_psi = 0;
    p->peak_objects = 0;
    p->peak_contacts = 0;
    p->total_kinetic_energy = 0.0;
    p->peak_kinetic_energy = 0.0;
}

/* Initialize counters (same as reset). */
static inline void ivp_perf_init(ivp_perf_counters_t *p) {
    ivp_perf_reset(p);
}

/* Begin a new PSI step: reset per-PSI counters. */
static inline void ivp_perf_begin_psi(ivp_perf_counters_t *p) {
    p->psi_count++;
    p->objects_active = 0;
    p->objects_sleeping = 0;
    p->contacts_this_psi = 0;
    p->controllers_this_psi = 0;
}

/* Record broadphase pair test. */
static inline void ivp_perf_add_broadphase_pair(ivp_perf_counters_t *p) {
    p->broadphase_pairs++;
}

/* Record narrowphase test. */
static inline void ivp_perf_add_narrowphase_test(ivp_perf_counters_t *p) {
    p->narrowphase_tests++;
}

/* Record a resolved contact. */
static inline void ivp_perf_add_contact(ivp_perf_counters_t *p) {
    p->contacts_resolved++;
    p->contacts_this_psi++;
    if (p->contacts_this_psi > p->peak_contacts) {
        p->peak_contacts = p->contacts_this_psi;
    }
}

/* Record a controller call. */
static inline void ivp_perf_add_controller_call(ivp_perf_counters_t *p) {
    p->controller_calls++;
    p->controllers_this_psi++;
}

/* Record object counts and update peaks. */
static inline void ivp_perf_set_object_counts(ivp_perf_counters_t *p,
                                                int active, int sleeping) {
    p->objects_active = active;
    p->objects_sleeping = sleeping;
    int total = active + sleeping;
    if (total > p->peak_objects) {
        p->peak_objects = total;
    }
}

/* Record total kinetic energy and update peak. */
static inline void ivp_perf_set_energy(ivp_perf_counters_t *p, double ke) {
    p->total_kinetic_energy = ke;
    if (ke > p->peak_kinetic_energy) {
        p->peak_kinetic_energy = ke;
    }
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_PERFORMANCE_H */
