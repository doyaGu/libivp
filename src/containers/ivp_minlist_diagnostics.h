/* Private, optional benchmark probes. Absent from normal engine builds. */
#ifndef IVP_MINLIST_DIAGNOSTICS_H
#define IVP_MINLIST_DIAGNOSTICS_H
#include <stdint.h>
#ifdef IVP_BENCHMARK_DIAGNOSTICS
typedef struct ivp_minlist_diagnostics {
    uint64_t adds;
    uint64_t removes;
    uint64_t tree_adds;
    uint64_t skip_adds;
    uint64_t fallback_adds;
    uint64_t comparisons;
    uint64_t max_comparisons;
    uint32_t max_population;
    uint64_t population_buckets[6];
} ivp_minlist_diagnostics_t;
void ivp_minlist_diagnostics_reset(void);
ivp_minlist_diagnostics_t ivp_minlist_diagnostics_get(void);
#endif
#endif
