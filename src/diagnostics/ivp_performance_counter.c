/* ivp_performance_counter.c — port of ivp_physics/ivp_performancecounter.cxx
 *
 * IVP_PerformanceCounter_Simple with the WIN32 implementation of the
 * reference made portable: QueryPerformanceCounter on Windows,
 * clock_gettime(CLOCK_MONOTONIC) elsewhere.
 */
#if !defined(_WIN32) && !defined(__APPLE__) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 199309L /* clock_gettime with -std=c17 */
#endif

#include <ivp/diagnostics/ivp_performance_counter.h>
#include <ivp/base/ivp_message.h>

#include <stdio.h>
#include <string.h>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <time.h>
#endif

static const double kScaleFactorForReducingPrecisionLoss = 1e6;
static const double kInvertedScaleFactorForReducingPrecisionLoss = 1.0 / 1e6;

/* ── clock ─────────────────────────────────────────────────────────────── */

#if defined(_WIN32)
static double get_inverted_performance_frequency(void)
{
    LARGE_INTEGER frequency;
    (void)QueryPerformanceFrequency(&frequency); /* always succeeds since XP */
    return 1.0 / (double)(frequency.QuadPart > 0 ? frequency.QuadPart : 1);
}

static int64_t get_performance_counter(void)
{
    LARGE_INTEGER profile_counter;
    (void)QueryPerformanceCounter(&profile_counter); /* always succeeds since XP */
    return (int64_t)profile_counter.QuadPart;
}
#else
static double get_inverted_performance_frequency(void)
{
    return 1e-9; /* nanosecond ticks */
}

static int64_t get_performance_counter(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
    return (int64_t)ts.tv_sec * INT64_C(1000000000) + (int64_t)ts.tv_nsec;
}
#endif

/* ── IVP_PerformanceCounter_Simple ─────────────────────────────────────── */

static void simple_reset_and_print(ivp_performance_counter_t *base, ivp_time_t current_time)
{
    ivp_performance_counter_simple_t *pc = (ivp_performance_counter_simple_t *)base;
    const double universe_samples_count = (double)pc->count_PSIs;
    if (universe_samples_count == 0.0) return;

    const ivp_real_t *counter = pc->counter;
    double collision = (double)counter[IVP_PE_PSI_UNIVERSE] + (double)counter[IVP_PE_PSI_SHORT_MINDISTS] +
                       (double)counter[IVP_PE_PSI_CRITICAL_MINDISTS] + (double)counter[IVP_PE_PSI_HULL] +
                       (double)counter[IVP_PE_AT_INIT];
    double dynamics = (double)counter[IVP_PE_PSI_CONTROLLERS] + (double)counter[IVP_PE_PSI_INTEGRATORS];
    double sum = collision + dynamics;

    const double factor = kInvertedScaleFactorForReducingPrecisionLoss / universe_samples_count;

    /* reference: ivp_message (engine output, "[havok] " prefix) */
    char report[4096];
    snprintf(report, sizeof(report),
             "[performance] TOTAL %2.1f%% %2.2fs\nCOLLISION %2.2fs\nDYNAMIC %2.2fs\n\n"
             "UNIVERSE: %2.2fs\nCONTROLLERS: %2.2fs\nINTEGRATORS: %2.2fs\n"
             "HULL: %2.2fs\nSHORT MINDISTS: %2.2fs\nCRITICAL MINDISTS: %2.2fs\nSIMULATE %2.2fs\n",
             sum * factor * 100.0, sum * factor, collision * factor, dynamics * factor,
             (double)counter[IVP_PE_PSI_UNIVERSE] * factor, (double)counter[IVP_PE_PSI_CONTROLLERS] * factor,
             (double)counter[IVP_PE_PSI_INTEGRATORS] * factor, (double)counter[IVP_PE_PSI_HULL] * factor,
             (double)counter[IVP_PE_PSI_SHORT_MINDISTS] * factor,
             (double)counter[IVP_PE_PSI_CRITICAL_MINDISTS] * factor, (double)counter[IVP_PE_AT_INIT] * factor);
    if (!pc->out || pc->out == stdout) {
        ivp_message("%s", report);
    } else {
        fputs(report, pc->out);   /* libivp: user destination, text without the prefix */
    }

    pc->ref_counter64 = get_performance_counter();

    pc->counting = IVP_PE_PSI_START;
    memset(pc->counter, 0, sizeof(pc->counter));
    pc->count_PSIs = 0;
    pc->time_of_last_reset = current_time;
}

static void simple_environment_is_going_to_be_deleted(ivp_performance_counter_t *base, struct ivp_environment *env)
{
    (void)env;
    /* P_DELETE_THIS(this) */
    ivp_performance_counter_simple_destroy((ivp_performance_counter_simple_t *)base);
}

static void simple_pcount(ivp_performance_counter_t *base, ivp_performance_element_t el)
{
    ivp_performance_counter_simple_t *pc = (ivp_performance_counter_simple_t *)base;
    if (el == IVP_PE_PSI_UNIVERSE) pc->count_PSIs++;

    const int64_t now = get_performance_counter();
    const int64_t diff0 = now - pc->ref_counter64;
    pc->ref_counter64 = now;

    if ((int)pc->counting >= 0 && pc->counting < IVP_PE_MAX) {
        pc->counter[pc->counting] +=
            (ivp_real_t)(kScaleFactorForReducingPrecisionLoss * (double)diff0 * pc->inv_counter_freq);
    }
    pc->counting = el;
}

static void simple_start_pcount(ivp_performance_counter_t *base)
{
    ivp_performance_counter_simple_t *pc = (ivp_performance_counter_simple_t *)base;
    pc->counting = IVP_PE_PSI_START;
}

static void simple_stop_pcount(ivp_performance_counter_t *base)
{
    (void)base;
}

static const ivp_performance_counter_vtable_t s_simple_vtable = {
    .start_pcount = simple_start_pcount,
    .pcount = simple_pcount,
    .stop_pcount = simple_stop_pcount,
    .environment_is_going_to_be_deleted = simple_environment_is_going_to_be_deleted,
    .reset_and_print_performance_counters = simple_reset_and_print,
};

void ivp_performance_counter_simple_init(ivp_performance_counter_simple_t *pc)
{
    if (!pc) return;
    memset(pc, 0, sizeof(*pc));
    pc->base.vt = &s_simple_vtable;
    pc->inv_counter_freq = get_inverted_performance_frequency();
    pc->ref_counter64 = get_performance_counter();
    pc->count_PSIs = 0;
    pc->counting = IVP_PE_PSI_START;
    pc->time_of_last_reset = 0.0;
    pc->out = stdout;
    pc->owner_allocator = NULL;
}

ivp_performance_counter_simple_t *ivp_performance_counter_simple_create(const ivp_allocator_t *alloc)
{
    const ivp_allocator_t *a = alloc ? alloc : ivp_default_allocator();
    ivp_performance_counter_simple_t *pc =
        (ivp_performance_counter_simple_t *)ivp_alloc(a, sizeof(ivp_performance_counter_simple_t));
    if (!pc) return NULL;
    ivp_performance_counter_simple_init(pc);
    pc->owner_allocator = a;
    return pc;
}

void ivp_performance_counter_simple_destroy(ivp_performance_counter_simple_t *pc)
{
    if (!pc || !pc->owner_allocator) return;
    const ivp_allocator_t *a = pc->owner_allocator;
    ivp_free(a, pc, sizeof(ivp_performance_counter_simple_t));
}
