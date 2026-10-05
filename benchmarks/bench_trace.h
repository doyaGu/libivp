/* Canonical, address-independent verification records shared by both runners. */
#ifndef BENCH_TRACE_H
#define BENCH_TRACE_H
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BENCH_TRACE_CONTRACT "native-trajectory-v1"
#define BENCH_HASH_INIT UINT64_C(1469598103934665603)

typedef struct bench_step {
    uint64_t objects;
    uint64_t constraints;
    uint64_t events;
    uint64_t statistics;
    uint64_t event_count;
    uint64_t trajectory;
    int object_count;
} bench_step_t;

static uint64_t bench_hash_u64(uint64_t hash, uint64_t value)
{
    for (int i = 0; i < 8; ++i) {
        hash ^= value & UINT64_C(255);
        hash *= UINT64_C(1099511628211);
        value >>= 8;
    }
    return hash;
}

static uint64_t bench_hash_double(uint64_t hash, double value)
{
    uint64_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return bench_hash_u64(hash, bits);
}

static uint64_t bench_hash_step(uint64_t hash, const bench_step_t *step)
{
    hash = bench_hash_u64(hash, (uint64_t)step->object_count);
    hash = bench_hash_u64(hash, step->objects);
    hash = bench_hash_u64(hash, step->constraints);
    hash = bench_hash_u64(hash, step->events);
    hash = bench_hash_u64(hash, step->statistics);
    return bench_hash_u64(hash, step->event_count);
}

static void bench_print_step(FILE *out, const char *implementation,
                            const char *name, int scale, int index,
                            const bench_step_t *step)
{
    fprintf(out, "{\"schema\":\"libivp-benchmark-v1\",\"kind\":\"native-step\","
            "\"verification_contract\":\"%s\",\"implementation\":\"%s\","
            "\"case\":\"%s\",\"scale\":%d,\"step\":%d,\"objects\":%d,"
            "\"object_hash\":\"%016llx\",\"constraint_hash\":\"%016llx\","
            "\"event_hash\":\"%016llx\",\"statistics_hash\":\"%016llx\","
            "\"event_count\":%llu,\"trajectory_hash\":\"%016llx\"}\n",
            BENCH_TRACE_CONTRACT, implementation, name, scale, index,
            step->object_count, (unsigned long long)step->objects,
            (unsigned long long)step->constraints, (unsigned long long)step->events,
            (unsigned long long)step->statistics, (unsigned long long)step->event_count,
            (unsigned long long)step->trajectory);
}
#endif
