/* Private intrusive workload probes; absent from normal engine builds. */
#ifndef IVP_RECHECK_DIAGNOSTICS_H
#define IVP_RECHECK_DIAGNOSTICS_H
#include <stdint.h>
#ifdef IVP_BENCHMARK_DIAGNOSTICS
typedef struct ivp_recheck_diagnostics {
    uint64_t rechecks;
    uint64_t recursive_rechecks;
    uint64_t max_depth;
    uint64_t candidates;
    uint64_t old_collisions;
    uint64_t max_candidates;
    uint64_t max_old_collisions;
    uint64_t potential_search_pairs;
    uint64_t searches;
    uint64_t found;
    uint64_t misses;
    uint64_t entries_compared;
    uint64_t max_scan;
    uint64_t swaps;
    uint64_t indexed_rechecks;
    uint64_t indexed_searches;
    uint64_t indexed_found;
    uint64_t index_entries;
    uint64_t index_probes;
    uint64_t max_index_probes;
    uint64_t duplicate_fallbacks;
    uint64_t unsafe_fallbacks;
    uint64_t oom_fallbacks;
} ivp_recheck_diagnostics_t;
void ivp_recheck_diagnostics_reset(void);
ivp_recheck_diagnostics_t ivp_recheck_diagnostics_get(void);
ivp_recheck_diagnostics_t *ivp_recheck_diagnostics_mutable(void);
#endif
#endif
