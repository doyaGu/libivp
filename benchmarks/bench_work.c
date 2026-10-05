#include "bench_work.h"
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SLOTS 65536
#define MAX_DIM 4096
#define MAX_STEPS 1024
typedef struct queue_slot { const void *key; int kind; } queue_slot_t;
typedef struct queue_counts {
    uint64_t adds, removes, maximum, populations[6];
} queue_counts_t;
static queue_slot_t slots[SLOTS];
static queue_counts_t queues[3];
static uint64_t lcs[MAX_DIM + 1], actives[MAX_DIM + 1];
static uint64_t steps[MAX_STEPS][3];
static unsigned step_count;
static uint64_t lcs_calls;
static int running;

static queue_slot_t *lookup(const void *key)
{
    uintptr_t hash = ((uintptr_t)key >> 3) * (uintptr_t)UINT64_C(11400714819323198485);
    unsigned slot = (unsigned)(hash ^ (hash >> 16)) & (SLOTS - 1);
    for (unsigned i = 0; i < SLOTS; ++i) {
        if (!slots[slot].key || slots[slot].key == key) return &slots[slot];
        slot = (slot + 1) & (SLOTS - 1);
    }
    fprintf(stderr, "work probe registry overflow\n");
    abort();
}

void bench_work_reset(void)
{
    running = 0;
    memset(slots, 0, sizeof(slots));
    memset(queues, 0, sizeof(queues));
    memset(lcs, 0, sizeof(lcs));
    memset(actives, 0, sizeof(actives));
    step_count = 0;
    lcs_calls = 0;
}
void bench_work_register(const void *queue, int kind)
{
    if (!queue || kind < 1 || kind > 2) abort();
    queue_slot_t *slot = lookup(queue);
    slot->key = queue;
    slot->kind = kind;
}
void bench_work_start(void) { running = 1; }
void bench_work_add(const void *queue, unsigned population)
{
    if (!running) return;
    queue_counts_t *q = &queues[lookup(queue)->kind];
    q->adds++;
    if (population > q->maximum) q->maximum = population;
    int bucket = population < 16 ? 0 : population < 64 ? 1 : population < 256 ? 2 :
                 population < 1024 ? 3 : population < 4096 ? 4 : 5;
    q->populations[bucket]++;
}
void bench_work_remove(const void *queue)
{
    if (running) queues[lookup(queue)->kind].removes++;
}
void bench_work_lcs(int dimension, int initial_actives)
{
    if (!running) return;
    if (dimension < 0 || dimension > MAX_DIM || initial_actives < 0) abort();
    lcs[dimension]++;
    actives[dimension] += (unsigned)initial_actives;
    lcs_calls++;
}
void bench_work_step(void)
{
    if (step_count == MAX_STEPS) abort();
    steps[step_count][0] = queues[1].adds;
    steps[step_count][1] = queues[2].adds;
    steps[step_count++][2] = lcs_calls;
}
void bench_work_finish(void)
{
    running = 0;
    const char *names[] = {"other", "hull", "time"};
    fprintf(stderr, "{\"schema\":\"libivp-benchmark-v1\",\"kind\":\"native-work\","
                    "\"scope\":\"simulation-only\",\"queues\":{");
    for (int i = 0; i < 3; ++i) {
        queue_counts_t *q = &queues[i];
        fprintf(stderr, "%s\"%s\":{\"adds\":%" PRIu64 ",\"removes\":%" PRIu64
                        ",\"max_population\":%" PRIu64 ",\"population_buckets\":[",
                i ? "," : "", names[i], q->adds, q->removes, q->maximum);
        for (int j = 0; j < 6; ++j) fprintf(stderr, "%s%" PRIu64, j ? "," : "", q->populations[j]);
        fprintf(stderr, "]}");
    }
    fprintf(stderr, "},\"lcs\":[");
    int first = 1;
    for (int n = 0; n <= MAX_DIM; ++n) if (lcs[n]) {
        fprintf(stderr, "%s{\"n\":%d,\"calls\":%" PRIu64 ",\"initial_actives\":%" PRIu64 "}",
                first ? "" : ",", n, lcs[n], actives[n]);
        first = 0;
    }
    fprintf(stderr, "],\"step_fields\":[\"hull_adds\",\"time_adds\",\"lcs_calls\"],\"cumulative_steps\":[");
    for (unsigned i = 0; i < step_count; ++i)
        fprintf(stderr, "%s[%" PRIu64 ",%" PRIu64 ",%" PRIu64 "]",
                i ? "," : "", steps[i][0], steps[i][1], steps[i][2]);
    fprintf(stderr, "]}\n");
}
