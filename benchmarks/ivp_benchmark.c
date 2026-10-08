/* Deterministic libivp performance and allocation benchmark. */
#if !defined(_WIN32) && !defined(__APPLE__) && !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 199309L
#endif
#include <ivp/base/ivp_allocator.h>
#include <ivp/base/ivp_message.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/controllers/ivp_constraint.h>
#include <ivp/diagnostics/ivp_performance_counter.h>
#include <ivp/diagnostics/ivp_statistic_manager.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/geometry/ivp_compact_surface.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/solver/ivp_solver_constants.h>

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef IVP_BENCHMARK_DIAGNOSTICS
#include "ivp_minlist_diagnostics.h"
#include "ivp_recheck_diagnostics.h"
#endif
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <time.h>
#endif

#define BENCH_SCHEMA "libivp-benchmark-v1"
#define MAX_SAMPLES 64
#define PHASE_COUNT IVP_PE_MAX
#define MAX_STEPS 600
#define WINDOW_COUNT 12
#include "bench_verify.h"
#include "bench_scene.h"
#ifdef _MSC_VER
#define BENCH_NOINLINE __declspec(noinline)
#else
#define BENCH_NOINLINE __attribute__((noinline))
#endif

typedef enum bench_kind {
    BENCH_ACTIVE,
    BENCH_CONTACT,
    BENCH_DENSE_CONTACTS,
    BENCH_CONSTRAINT,
    BENCH_ISLAND_CONSTRAINTS,
    BENCH_MIXED_ISLAND_CONSTRAINTS,
    BENCH_HETEROGENEOUS_ISLANDS,
    BENCH_IMPACTS,
    BENCH_CHURN,
    BENCH_SLEEPING,
    BENCH_CONVEX
} bench_kind_t;

typedef struct bench_config {
    const char *name;
    bench_kind_t kind;
    int scale;
    int steps;
    double dt;
    int contact_height;
} bench_config_t;

typedef struct bench_measurement {
    int64_t operation_ns;
    int64_t total_ns;
    int64_t validation_ns;
    int64_t operations;
    int64_t psis;
    double simulated_seconds;
    int64_t phase_ns[PHASE_COUNT];
    uint64_t state_hash;
    int mindists_generated;
    int mindists_deleted;
    int range_intra_exceeded;
    int range_world_exceeded;
    int impacts;
    ivp_parallel_stats_t parallel;
    ivp_parallel_stats_t parallel_before_operation;
    int parallel_lifetime_max_workers_used;
    int valid;
    bench_verification_t verification;
    int64_t step_ns[MAX_STEPS];
    int step_count;
    int64_t window_ns[WINDOW_COUNT];
    int window_steps[WINDOW_COUNT];
    int window_impacts[WINDOW_COUNT];
#ifdef IVP_BENCHMARK_DIAGNOSTICS
    ivp_minlist_diagnostics_t minlist;
    ivp_recheck_diagnostics_t broadphase;
#endif
} bench_measurement_t;

typedef struct options {
    const char *profile;
    const char *case_filter;
    const char *format;
    const char *output_path;
    int samples;
    double min_sample_ms;
    int check;
    int list;
    int scale;
    double hotspot_ms;
    int verify;
    int contact_height;
    int noop_counter;
    int workers;
} options_t;

static int noop_counter_requested;
static int benchmark_worker_count = 1;
#ifdef BENCH_WORK_DIAGNOSTICS
#include "bench_work.h"
#include <ivp/collision/ivp_hull_mgr.h>
#include <ivp/dynamics/ivp_time_manager.h>
#endif

static int64_t now_ns(void)
{
#ifdef _WIN32
    LARGE_INTEGER frequency, counter;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&counter);
    return (int64_t)((double)counter.QuadPart * 1000000000.0 /
                     (double)frequency.QuadPart);
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * INT64_C(1000000000) + (int64_t)ts.tv_nsec;
#endif
}

/* The reference counter deliberately leaves its clock running at stop_pcount.
 * Use a benchmark-owned counter so setup, warmup, hashing and teardown cannot
 * be attributed to simulation phases. No engine timing semantics are changed. */
typedef struct bench_counter {
    ivp_performance_counter_t base;
    const ivp_allocator_t *allocator;
    int64_t ns[PHASE_COUNT];
    int64_t last;
    ivp_performance_element_t phase;
    int running;
} bench_counter_t;

static void counter_start(ivp_performance_counter_t *base)
{
    bench_counter_t *pc = (bench_counter_t *)base;
    pc->phase = IVP_PE_PSI_START;
    pc->last = now_ns();
    pc->running = 1;
}

static void counter_phase(ivp_performance_counter_t *base, ivp_performance_element_t phase)
{
    bench_counter_t *pc = (bench_counter_t *)base;
    if (!pc->running) return;
    int64_t now = now_ns();
    pc->ns[pc->phase] += now - pc->last;
    pc->last = now;
    pc->phase = phase;
}

static void counter_stop(ivp_performance_counter_t *base)
{
    bench_counter_t *pc = (bench_counter_t *)base;
    if (!pc->running) return;
    pc->ns[pc->phase] += now_ns() - pc->last;
    pc->running = 0;
}

static void counter_destroy(ivp_performance_counter_t *base, ivp_environment_t *env)
{
    (void)env;
    bench_counter_t *pc = (bench_counter_t *)base;
    ivp_free(pc->allocator, pc, sizeof(*pc));
}

static void counter_reset(ivp_performance_counter_t *base, ivp_time_t time)
{
    (void)time;
    bench_counter_t *pc = (bench_counter_t *)base;
    memset(pc->ns, 0, sizeof(pc->ns));
    pc->running = 0;
}

static const ivp_performance_counter_vtable_t counter_vtable = {
    counter_start, counter_phase, counter_stop, counter_destroy, counter_reset
};

static void counter_noop(ivp_performance_counter_t *base) { (void)base; }
static void counter_noop_phase(ivp_performance_counter_t *base, ivp_performance_element_t phase)
{ (void)base; (void)phase; }
static const ivp_performance_counter_vtable_t noop_counter_vtable = {
    counter_noop, counter_noop_phase, counter_noop, counter_destroy, counter_reset
};

static void begin_operation(ivp_environment_t *env, bench_measurement_t *m)
{
    ivp_environment_get_parallel_stats(env, &m->parallel_before_operation);
#ifdef IVP_BENCHMARK_DIAGNOSTICS
    ivp_minlist_diagnostics_reset();
    ivp_recheck_diagnostics_reset();
#endif
    ivp_performance_counter_t *pc = ivp_environment_get_performance_counter(env);
    if (pc) counter_reset(pc, 0.0);
}

/* Stable stack boundaries for external sampling, also in optimized builds. */
static BENCH_NOINLINE void bench_simulate(ivp_environment_t *env, int steps,
                                         const bench_config_t *cfg,
                                         bench_measurement_t *m, int instrumented)
{
    if (instrumented == 3) {
        verify_begin(&m->verification, env);
#ifdef BENCH_WORK_DIAGNOSTICS
        bench_work_reset();
        bench_work_register(&ivp_environment_get_time_manager(env)->min_hash, 2);
        for (int i = 0; i < ivp_environment_get_object_count(env); ++i) {
            ivp_object_t *o = ivp_environment_get_object(env, i);
            if (o->hull_manager) bench_work_register(&o->hull_manager->sorted_synapses, 1);
        }
        bench_work_start();
        bench_work_step();
#endif
        for (int i = 0; i < steps; ++i) {
            ivp_environment_simulate_one_psi(env);
            verify_step(&m->verification);
#ifdef BENCH_WORK_DIAGNOSTICS
            bench_work_step();
#endif
        }
#ifdef BENCH_WORK_DIAGNOSTICS
        bench_work_finish();
#endif
        m->verification.running = 0;
        return;
    }
    if (!instrumented) {
        for (int i = 0; i < steps; ++i) ivp_environment_simulate_one_psi(env);
        return;
    }
    ivp_statistic_manager_t *sm = ivp_environment_get_statistic_manager(env);
    for (int i = 0; i < steps; ++i) {
        int previous_impacts = sm ? sm->impact_counter : 0;
        int64_t start = now_ns();
        ivp_environment_simulate_one_psi(env);
        int64_t ns = now_ns() - start;
        int index = m->step_count++;
        if (index < MAX_STEPS) m->step_ns[index] = ns;
        int window = index * WINDOW_COUNT / cfg->steps;
        if (window >= WINDOW_COUNT) window = WINDOW_COUNT - 1;
        m->window_ns[window] += ns;
        m->window_steps[window]++;
        m->window_impacts[window] += sm ? sm->impact_counter - previous_impacts : 0;
    }
}

typedef void (*bench_before_step_fn)(void *context);

static BENCH_NOINLINE void bench_simulate_driven(
    ivp_environment_t *env, int steps, const bench_config_t *cfg,
    bench_measurement_t *m, int instrumented,
    bench_before_step_fn before_step, void *context)
{
    if (instrumented == 3) {
        verify_begin(&m->verification, env);
        for (int i = 0; i < steps; ++i) {
            before_step(context);
            ivp_environment_simulate_one_psi(env);
            verify_step(&m->verification);
        }
        m->verification.running = 0;
        return;
    }
    if (!instrumented) {
        for (int i = 0; i < steps; ++i) {
            before_step(context);
            ivp_environment_simulate_one_psi(env);
        }
        return;
    }
    ivp_statistic_manager_t *sm = ivp_environment_get_statistic_manager(env);
    for (int i = 0; i < steps; ++i) {
        int previous_impacts = sm ? sm->impact_counter : 0;
        before_step(context);
        int64_t start = now_ns();
        ivp_environment_simulate_one_psi(env);
        int64_t ns = now_ns() - start;
        int index = m->step_count++;
        if (index < MAX_STEPS) m->step_ns[index] = ns;
        int window = index * WINDOW_COUNT / cfg->steps;
        if (window >= WINDOW_COUNT) window = WINDOW_COUNT - 1;
        m->window_ns[window] += ns;
        m->window_steps[window]++;
        m->window_impacts[window] +=
            sm ? sm->impact_counter - previous_impacts : 0;
    }
}

static uint64_t hash_bytes(uint64_t hash, const void *data, size_t size)
{
    const unsigned char *bytes = (const unsigned char *)data;
    for (size_t i = 0; i < size; ++i) {
        hash ^= bytes[i];
        hash *= UINT64_C(1099511628211);
    }
    return hash;
}

static uint64_t hash_object(uint64_t hash, const ivp_object_t *obj)
{
    const ivp_core_t *core = obj->physical_core;
    hash = hash_bytes(hash, &obj->type, sizeof(obj->type));
    hash = hash_bytes(hash, &obj->object_movement_state, sizeof(obj->object_movement_state));
    hash = hash_bytes(hash, &obj->collision_detection_enabled,
                      sizeof(obj->collision_detection_enabled));
    if (!core) return hash;
    hash = hash_bytes(hash, &core->movement_state, sizeof(core->movement_state));
    hash = hash_bytes(hash, core->pos_world_f_core_last_psi.k,
                      sizeof(core->pos_world_f_core_last_psi.k));
    hash = hash_bytes(hash, &core->q_world_f_core_last_psi,
                      sizeof(core->q_world_f_core_last_psi));
    hash = hash_bytes(hash, core->speed.k, sizeof(core->speed.k));
    hash = hash_bytes(hash, core->rot_speed.k, sizeof(core->rot_speed.k));
    return hash;
}

static BENCH_NOINLINE uint64_t hash_environment(const ivp_environment_t *env)
{
    uint64_t hash = UINT64_C(1469598103934665603);
    const int count = ivp_environment_get_object_count(env);
    hash = hash_bytes(hash, &count, sizeof(count));
    for (int i = 0; i < count; ++i) {
        hash = hash_object(hash, ivp_environment_get_object(env, i));
    }
    return hash;
}

static BENCH_NOINLINE void bench_destroy(ivp_environment_t *env)
{
    ivp_environment_destroy(env);
}

static int parse_positive_int(const char *s, int *out)
{
    char *end = NULL;
    errno = 0;
    long value = strtol(s, &end, 10);
    if (errno || !end || *end || value <= 0 || value > 1000000) return 0;
    *out = (int)value;
    return 1;
}

static int parse_positive_double(const char *s, double *out)
{
    char *end = NULL;
    errno = 0;
    double value = strtod(s, &end);
    if (errno || !end || *end || !(value > 0.0) || !isfinite(value)) return 0;
    *out = value;
    return 1;
}

static void usage(const char *exe)
{
    fprintf(stderr,
            "Usage: %s [--profile quick|standard|large] [--case NAME] [--samples N]\n"
            "          [--min-sample-ms MS] [--format text|jsonl] [--output PATH]\n"
            "          [--scale N] [--hotspot-ms MS] [--verify] [--check] [--list]\n"
            "          [--contact-height N] [--workers N] [--noop-counter]\n", exe);
}

static void benchmark_message(const char *text, void *userdata)
{
    (void)userdata;
    fputs(text, stderr);
}

static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t radius,
                               ivp_real_t x, ivp_real_t y, ivp_real_t z)
{
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.ball_radius = radius;
    t.mass = IVP_REAL(1.0);
    ivp_real_t inertia = IVP_REAL(0.4) * radius * radius;
    ivp_vec3r_set(&t.rot_inertia, inertia, inertia, inertia);
    t.rot_inertia_is_factor = false;
    t.material.friction = IVP_REAL(0.7);
    t.material.elasticity = IVP_REAL(0.1);
    t.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set_zero(&t.rot_speed_damp_factor);
    ivp_vec3r_set(&t.position, x, y, z);
    t.orientation = ivp_quatr_identity();
    return ivp_environment_create_object(env, &t, NULL);
}

static int object_finite(const ivp_object_t *obj)
{
    const ivp_vec3r_t *p = &obj->physical_core->pos_world_f_core_last_psi;
    return isfinite((double)p->k[0]) && isfinite((double)p->k[1]) &&
           isfinite((double)p->k[2]);
}

static void collect_phases(ivp_environment_t *env, bench_measurement_t *m)
{
    ivp_environment_get_parallel_stats(env, &m->parallel);
    m->parallel_lifetime_max_workers_used = m->parallel.max_workers_used;
    /* Only additive counters admit interval subtraction. Keep the lifetime
     * high-water mark separate rather than pretending it is an interval max. */
    m->parallel.island_dispatches -= m->parallel_before_operation.island_dispatches;
    m->parallel.island_tasks -= m->parallel_before_operation.island_tasks;
    m->parallel.broadphase_dispatches -= m->parallel_before_operation.broadphase_dispatches;
    m->parallel.broadphase_tasks -= m->parallel_before_operation.broadphase_tasks;
    m->parallel.solver_dispatches -= m->parallel_before_operation.solver_dispatches;
    m->parallel.solver_tasks -= m->parallel_before_operation.solver_tasks;
#ifdef IVP_BENCHMARK_DIAGNOSTICS
    m->minlist = ivp_minlist_diagnostics_get();
    m->broadphase = ivp_recheck_diagnostics_get();
#endif
    ivp_performance_counter_t *base = ivp_environment_get_performance_counter(env);
    if (!base) return;
    bench_counter_t *pc = (bench_counter_t *)base;
    for (int i = 0; i < PHASE_COUNT; ++i)
        m->phase_ns[i] += pc->ns[i];

    ivp_statistic_manager_t *sm = ivp_environment_get_statistic_manager(env);
    if (sm) {
        m->mindists_generated = sm->mindists_generated;
        m->mindists_deleted = sm->mindists_deleted;
        m->range_intra_exceeded = sm->range_intra_exceeded;
        m->range_world_exceeded = sm->range_world_exceeded;
        m->impacts = sm->impact_counter;
    }
}

static ivp_environment_t *make_env(const ivp_allocator_t *allocator, double dt,
                                   int gravity, int group_filter, int instrumented)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    cfg.allocator = (ivp_allocator_t *)allocator;
    cfg.delta_psi_time = (ivp_real_t)dt;
    cfg.parallel_worker_count = benchmark_worker_count;
    if (!gravity) ivp_vec3r_set_zero(&cfg.gravity);
    if (group_filter) cfg.collision_filter = ivp_collision_filter_group();
    cfg.disable_performance_counter = instrumented != 1;
    if (instrumented == 1 || noop_counter_requested) {
        bench_counter_t *pc = (bench_counter_t *)ivp_alloc(allocator, sizeof(*pc));
        if (!pc) return NULL;
        memset(pc, 0, sizeof(*pc));
        pc->base.vt = instrumented == 1 ? &counter_vtable : &noop_counter_vtable;
        pc->allocator = allocator;
        cfg.performance_counter = &pc->base;
        ivp_environment_t *env = ivp_environment_create(&cfg);
        if (!env) ivp_free(allocator, pc, sizeof(*pc));
        return env;
    }
    return ivp_environment_create(&cfg);
}

static int run_active(const bench_config_t *cfg, const ivp_allocator_t *allocator,
                      bench_measurement_t *m, int instrumented)
{
    int64_t total_start = now_ns();
    ivp_environment_t *env = make_env(allocator, cfg->dt, 0, 0, instrumented);
    if (!env) return 0;
    ivp_object_t *probe = NULL;
    for (int i = 0; i < cfg->scale; ++i) {
        int x = i % 16, y = (i / 16) % 16, z = i / 256;
        ivp_object_t *o = make_ball(env, IVP_REAL(0.35),
                                   (ivp_real_t)(x * 4), (ivp_real_t)(y * 4),
                                   (ivp_real_t)(z * 4));
        if (!o) { ivp_environment_destroy(env); return 0; }
        ivp_vec3r_set(&o->physical_core->speed, IVP_REAL(2.0), IVP_REAL(0.25), IVP_REAL(0.1));
        o->physical_core->delta_world_f_core_psis = o->physical_core->speed;
        probe = o;
    }
    begin_operation(env, m);
    int64_t start = now_ns();
    bench_simulate(env, cfg->steps, cfg, m, instrumented);
    m->operation_ns = now_ns() - start;
    m->operations = cfg->steps;
    m->psis = cfg->steps;
    m->simulated_seconds = cfg->steps * cfg->dt;
    m->valid = probe && object_finite(probe);
    m->state_hash = hash_environment(env);
    collect_phases(env, m);
    bench_destroy(env);
    m->total_ns = now_ns() - total_start;
    return 1;
}

static int run_contact(const bench_config_t *cfg, const ivp_allocator_t *allocator,
                       bench_measurement_t *m, int instrumented)
{
    int64_t total_start = now_ns();
    ivp_environment_t *env = make_env(allocator, cfg->dt, 1, 0, instrumented);
    if (!env) return 0;
    float extent = bench_contact_ground_extent(cfg->scale, cfg->contact_height);
    ivp_compact_surface_t *surface = ivp_build_box_surface(allocator, extent, 0.5f, extent);
    if (!surface) { ivp_environment_destroy(env); return 0; }
    ivp_surman_polygon_t surman;
    ivp_surman_polygon_init(&surman, surface);
    ivp_object_template_t ground_t;
    ivp_object_template_init(&ground_t);
    ground_t.type = IVP_OBJECT_POLYGON;
    ground_t.physical_unmoveable = true;
    ground_t.mass = IVP_REAL(1e6);
    ivp_vec3r_set(&ground_t.rot_inertia, IVP_REAL(1e6), IVP_REAL(1e6), IVP_REAL(1e6));
    ground_t.rot_inertia_is_factor = false;
    ivp_vec3r_set(&ground_t.position, IVP_REAL(0.0), IVP_REAL(3.0), IVP_REAL(0.0));
    if (!ivp_environment_create_object(env, &ground_t, &surman.base)) {
        ivp_environment_destroy(env); ivp_compact_surface_free(allocator, surface); return 0;
    }
    ivp_object_t *probe = NULL;
    for (int i = 0; i < cfg->scale; ++i) {
        double p[3];
        bench_contact_position(i, cfg->scale, cfg->contact_height, p);
        probe = make_ball(env, IVP_REAL(0.45),
                          (ivp_real_t)p[0], (ivp_real_t)p[1], (ivp_real_t)p[2]);
        if (!probe) { ivp_environment_destroy(env); ivp_compact_surface_free(allocator, surface); return 0; }
    }
    begin_operation(env, m);
    int64_t start = now_ns();
    bench_simulate(env, cfg->steps, cfg, m, instrumented);
    m->operation_ns = now_ns() - start;
    m->operations = m->psis = cfg->steps;
    m->simulated_seconds = cfg->steps * cfg->dt;
    m->valid = probe && object_finite(probe);
    m->state_hash = hash_environment(env);
    collect_phases(env, m);
    bench_destroy(env);
    ivp_compact_surface_free(allocator, surface);
    m->total_ns = now_ns() - total_start;
    return 1;
}

typedef struct dense_contact_driver {
    ivp_object_t **balls;
    ivp_vec3r_t *inward_speeds;
    int count;
} dense_contact_driver_t;

static void drive_dense_contacts(void *opaque)
{
    dense_contact_driver_t *driver = (dense_contact_driver_t *)opaque;
    for (int i = 0; i < driver->count; i++) {
        driver->balls[i]->physical_core->speed = driver->inward_speeds[i];
    }
}

static int run_dense_contacts(const bench_config_t *cfg,
                              const ivp_allocator_t *allocator,
                              bench_measurement_t *m, int instrumented)
{
    if (cfg->scale > IVP_MAX_FRICTION_MATRIX) return 0;
    int64_t total_start = now_ns();
    ivp_environment_t *env = make_env(allocator, cfg->dt, 0, 0,
                                      instrumented);
    if (!env) return 0;
    ivp_object_t **balls = (ivp_object_t **)ivp_alloc(
        allocator, (size_t)cfg->scale * sizeof(*balls));
    ivp_vec3r_t *inward_speeds = (ivp_vec3r_t *)ivp_alloc(
        allocator, (size_t)cfg->scale * sizeof(*inward_speeds));
    if (!balls || !inward_speeds) goto fail;

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(5.0);
    object_template.mass = IVP_REAL(1000.0);
    object_template.material.friction = IVP_REAL(0.8);
    object_template.material.elasticity = IVP_REAL(0.0);
    object_template.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set_zero(&object_template.rot_speed_damp_factor);
    ivp_object_t *center = ivp_environment_create_object(
        env, &object_template, NULL);
    if (!center) goto fail;

    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.25);
    object_template.mass = IVP_REAL(1.0);
    object_template.material.friction = IVP_REAL(0.8);
    object_template.material.elasticity = IVP_REAL(0.0);
    object_template.speed_damp_factor = IVP_REAL(0.0);
    ivp_vec3r_set_zero(&object_template.rot_speed_damp_factor);
    const double golden_angle = 2.39996322972865332;
    for (int i = 0; i < cfg->scale; i++) {
        double y = 1.0 - 2.0 * ((double)i + 0.5) / (double)cfg->scale;
        double radial = sqrt(1.0 - y * y);
        double angle = golden_angle * (double)i;
        double x = cos(angle) * radial;
        double z = sin(angle) * radial;
        ivp_vec3r_set(&object_template.position,
                      (ivp_real_t)(5.30 * x),
                      (ivp_real_t)(5.30 * y),
                      (ivp_real_t)(5.30 * z));
        ivp_vec3r_set(&inward_speeds[i],
                      (ivp_real_t)(-2.0 * x),
                      (ivp_real_t)(-2.0 * y),
                      (ivp_real_t)(-2.0 * z));
        balls[i] = ivp_environment_create_object(env, &object_template, NULL);
        if (!balls[i]) goto fail;
    }
    dense_contact_driver_t driver = {balls, inward_speeds, cfg->scale};
    drive_dense_contacts(&driver);
    begin_operation(env, m);
    int64_t start = now_ns();
    bench_simulate_driven(env, cfg->steps, cfg, m, instrumented,
                          drive_dense_contacts, &driver);
    m->operation_ns = now_ns() - start;
    m->operations = m->psis = cfg->steps;
    m->simulated_seconds = cfg->steps * cfg->dt;
    m->valid = object_finite(center) && object_finite(balls[cfg->scale - 1]);
    m->state_hash = hash_environment(env);
    collect_phases(env, m);
    bench_destroy(env);
    ivp_free(allocator, inward_speeds,
             (size_t)cfg->scale * sizeof(*inward_speeds));
    ivp_free(allocator, balls, (size_t)cfg->scale * sizeof(*balls));
    m->total_ns = now_ns() - total_start;
    return 1;

fail:
    if (inward_speeds) {
        ivp_free(allocator, inward_speeds,
                 (size_t)cfg->scale * sizeof(*inward_speeds));
    }
    if (balls) {
        ivp_free(allocator, balls, (size_t)cfg->scale * sizeof(*balls));
    }
    ivp_environment_destroy(env);
    return 0;
}

static int run_constraint(const bench_config_t *cfg, const ivp_allocator_t *allocator,
                          bench_measurement_t *m, int instrumented)
{
    int64_t total_start = now_ns();
    ivp_environment_t *env = make_env(allocator, cfg->dt, 1, 1, instrumented);
    if (!env) return 0;
    ivp_object_t **objects = (ivp_object_t **)ivp_alloc(allocator, (size_t)cfg->scale * sizeof(*objects));
    ivp_constraint_state_t *constraints = (ivp_constraint_state_t *)ivp_alloc(
        allocator, (size_t)cfg->scale * sizeof(*constraints));
    if (!objects || !constraints) {
        if (objects) ivp_free(allocator, objects, (size_t)cfg->scale * sizeof(*objects));
        if (constraints) ivp_free(allocator, constraints, (size_t)cfg->scale * sizeof(*constraints));
        ivp_environment_destroy(env); return 0;
    }
    for (int i = 0; i < cfg->scale; ++i) {
        objects[i] = make_ball(env, IVP_REAL(0.3), (ivp_real_t)(i * 0.7), IVP_REAL(0.0), IVP_REAL(0.0));
        if (!objects[i]) { ivp_environment_destroy(env); return 0; }
        ivp_object_set_collision_group(objects[i], 1);
        ivp_vec3r_t anchor = ivp_vec3r((ivp_real_t)(i * 0.7 - 0.35), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_constraint_init_ball_socket_ws(&constraints[i], i ? objects[i - 1] : NULL,
                                           &anchor, objects[i]);
        ivp_environment_announce_dependent_controller(env, 0, ivp_controller_constraint,
                                                      &constraints[i], ivp_constraint_discovery);
    }
    m->verification.constraints = constraints;
    m->verification.constraint_count = cfg->scale;
    begin_operation(env, m);
    int64_t start = now_ns();
    bench_simulate(env, cfg->steps, cfg, m, instrumented);
    m->operation_ns = now_ns() - start;
    m->operations = m->psis = cfg->steps;
    m->simulated_seconds = cfg->steps * cfg->dt;
    m->valid = object_finite(objects[cfg->scale - 1]);
    m->state_hash = hash_environment(env);
    collect_phases(env, m);
    bench_destroy(env);
    ivp_free(allocator, constraints, (size_t)cfg->scale * sizeof(*constraints));
    ivp_free(allocator, objects, (size_t)cfg->scale * sizeof(*objects));
    m->total_ns = now_ns() - total_start;
    return 1;
}

static int run_island_constraints(const bench_config_t *cfg,
                                  const ivp_allocator_t *allocator,
                                  bench_measurement_t *m, int instrumented)
{
    int64_t total_start = now_ns();
    ivp_environment_t *env = make_env(allocator, cfg->dt, 0, 0,
                                      instrumented);
    if (!env) return 0;
    ivp_object_t **objects = (ivp_object_t **)ivp_alloc(
        allocator, (size_t)cfg->scale * sizeof(*objects));
    ivp_constraint_state_t *constraints =
        (ivp_constraint_state_t *)ivp_alloc(
            allocator, (size_t)cfg->scale * sizeof(*constraints));
    if (!objects || !constraints) goto fail;

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.25);
    object_template.mass = IVP_REAL(1.0);
    object_template.speed_damp_factor = IVP_REAL(0.0);
    object_template.enable_collision_detection = false;
    ivp_vec3r_set_zero(&object_template.rot_speed_damp_factor);
    for (int i = 0; i < cfg->scale; i++) {
        ivp_vec3r_t anchor = ivp_vec3r(
            (ivp_real_t)(i % 64) * IVP_REAL(2.0),
            (ivp_real_t)(i / 64) * IVP_REAL(2.0), IVP_REAL(0.0));
        object_template.position = anchor;
        objects[i] = ivp_environment_create_object(
            env, &object_template, NULL);
        if (!objects[i]) goto fail;
        objects[i]->physical_core->speed.k[0] =
            (ivp_real_t)(float)(0.25f + (float)(i & 7) * 0.01f);
        ivp_constraint_init_ball_socket_ws(
            &constraints[i], NULL, &anchor, objects[i]);
        if (cfg->kind == BENCH_MIXED_ISLAND_CONSTRAINTS && i == 0) {
            constraints[i].force_exceed_action = IVP_CFE_BREAK;
            constraints[i].break_force_threshold = IVP_REAL(1.0e10);
        }
        if (!ivp_controller_handle_is_valid(
                ivp_environment_announce_dependent_controller(
                    env, IVP_CP_CONSTRAINTS, ivp_controller_constraint,
                    &constraints[i], ivp_constraint_discovery))) {
            goto fail;
        }
    }
    m->verification.constraints = constraints;
    m->verification.constraint_count = cfg->scale;
    begin_operation(env, m);
    int64_t start = now_ns();
    bench_simulate(env, cfg->steps, cfg, m, instrumented);
    m->operation_ns = now_ns() - start;
    m->operations = m->psis = cfg->steps;
    m->simulated_seconds = cfg->steps * cfg->dt;
    m->valid = object_finite(objects[cfg->scale - 1]);
    m->state_hash = hash_environment(env);
    collect_phases(env, m);
    bench_destroy(env);
    ivp_free(allocator, constraints,
             (size_t)cfg->scale * sizeof(*constraints));
    ivp_free(allocator, objects, (size_t)cfg->scale * sizeof(*objects));
    m->total_ns = now_ns() - total_start;
    return m->valid;

fail:
    ivp_environment_destroy(env);
    if (constraints) {
        ivp_free(allocator, constraints,
                 (size_t)cfg->scale * sizeof(*constraints));
    }
    if (objects) {
        ivp_free(allocator, objects,
                 (size_t)cfg->scale * sizeof(*objects));
    }
    return 0;
}

static int run_heterogeneous_constraint_islands(
    const bench_config_t *cfg, const ivp_allocator_t *allocator,
    bench_measurement_t *m, int instrumented)
{
    int64_t total_start = now_ns();
    ivp_environment_t *env = make_env(allocator, cfg->dt, 0, 0,
                                      instrumented);
    if (!env) return 0;
    ivp_object_t **objects = (ivp_object_t **)ivp_alloc(
        allocator, (size_t)cfg->scale * sizeof(*objects));
    ivp_constraint_state_t *constraints =
        (ivp_constraint_state_t *)ivp_alloc(
            allocator, (size_t)cfg->scale * sizeof(*constraints));
    if (!objects || !constraints) goto fail;

    int island_count = cfg->scale / 8;
    if (island_count < 256) island_count = cfg->scale / 2;
    if (island_count < 1) island_count = 1;
    int heavy_count = island_count / 4;
    if (heavy_count < 1) heavy_count = 1;
    int light_count = island_count - heavy_count;
    int light_objects = light_count * 2;
    int heavy_objects = cfg->scale - light_objects;

    ivp_object_template_t object_template;
    ivp_object_template_init(&object_template);
    object_template.type = IVP_OBJECT_BALL;
    object_template.ball_radius = IVP_REAL(0.2);
    object_template.mass = IVP_REAL(1.0);
    object_template.speed_damp_factor = IVP_REAL(0.0);
    object_template.enable_collision_detection = false;
    ivp_vec3r_set_zero(&object_template.rot_speed_damp_factor);

    int object_index = 0;
    for (int island = 0; island < island_count; island++) {
        int island_size;
        if (island < light_count) {
            island_size = 2;
        } else {
            /* Heavy islands are created last, so the manager's prepend order
             * clusters them at the beginning of the parallel task list. */
            int heavy_index = island - light_count;
            island_size = heavy_objects / heavy_count;
            if (heavy_index < heavy_objects % heavy_count) island_size++;
        }
        ivp_object_t *previous = NULL;
        ivp_vec3r_t previous_position = ivp_vec3r(
            IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
        for (int local = 0; local < island_size; local++) {
            ivp_vec3r_set(
                &object_template.position,
                (ivp_real_t)(island % 64) * IVP_REAL(24.0) +
                    (ivp_real_t)local * IVP_REAL(0.5),
                (ivp_real_t)(island / 64) * IVP_REAL(4.0),
                IVP_REAL(0.0));
            objects[object_index] = ivp_environment_create_object(
                env, &object_template, NULL);
            if (!objects[object_index]) goto fail;
            objects[object_index]->physical_core->speed.k[0] =
                (ivp_real_t)(float)(
                    0.2f + (float)(object_index & 7) * 0.01f);
            ivp_vec3r_t anchor = object_template.position;
            if (previous) {
                anchor.k[0] = (anchor.k[0] + previous_position.k[0]) *
                              IVP_REAL(0.5);
                anchor.k[1] = (anchor.k[1] + previous_position.k[1]) *
                              IVP_REAL(0.5);
                anchor.k[2] = (anchor.k[2] + previous_position.k[2]) *
                              IVP_REAL(0.5);
            }
            ivp_constraint_init_ball_socket_ws(
                &constraints[object_index], previous, &anchor,
                objects[object_index]);
            if (!ivp_controller_handle_is_valid(
                    ivp_environment_announce_dependent_controller(
                        env, IVP_CP_CONSTRAINTS, ivp_controller_constraint,
                        &constraints[object_index],
                        ivp_constraint_discovery))) {
                goto fail;
            }
            previous = objects[object_index];
            previous_position = object_template.position;
            object_index++;
        }
    }
    if (object_index != cfg->scale) goto fail;

    m->verification.constraints = constraints;
    m->verification.constraint_count = cfg->scale;
    begin_operation(env, m);
    int64_t start = now_ns();
    bench_simulate(env, cfg->steps, cfg, m, instrumented);
    m->operation_ns = now_ns() - start;
    m->operations = m->psis = cfg->steps;
    m->simulated_seconds = cfg->steps * cfg->dt;
    m->valid = object_finite(objects[cfg->scale - 1]);
    m->state_hash = hash_environment(env);
    collect_phases(env, m);
    bench_destroy(env);
    ivp_free(allocator, constraints,
             (size_t)cfg->scale * sizeof(*constraints));
    ivp_free(allocator, objects, (size_t)cfg->scale * sizeof(*objects));
    m->total_ns = now_ns() - total_start;
    return m->valid;

fail:
    ivp_environment_destroy(env);
    if (constraints) {
        ivp_free(allocator, constraints,
                 (size_t)cfg->scale * sizeof(*constraints));
    }
    if (objects) {
        ivp_free(allocator, objects,
                 (size_t)cfg->scale * sizeof(*objects));
    }
    return 0;
}

static int run_impacts(const bench_config_t *cfg, const ivp_allocator_t *allocator,
                       bench_measurement_t *m, int instrumented)
{
    int64_t total_start = now_ns();
    ivp_environment_t *env = make_env(allocator, cfg->dt, 0, 0, instrumented);
    if (!env) return 0;
    ivp_object_t *probe = NULL;
    for (int i = 0; i < cfg->scale; ++i) {
        ivp_real_t y = (ivp_real_t)((i % 16) * 2.0);
        ivp_real_t z = (ivp_real_t)((i / 16) * 2.0);
        ivp_object_t *a = make_ball(env, IVP_REAL(0.4), IVP_REAL(-4.0), y, z);
        ivp_object_t *b = make_ball(env, IVP_REAL(0.4), IVP_REAL(4.0), y, z);
        if (!a || !b) { ivp_environment_destroy(env); return 0; }
        a->material.elasticity = b->material.elasticity = IVP_REAL(0.9);
        ivp_vec3r_set(&a->physical_core->speed, IVP_REAL(80.0), IVP_REAL(0.0), IVP_REAL(0.0));
        ivp_vec3r_set(&b->physical_core->speed, IVP_REAL(-80.0), IVP_REAL(0.0), IVP_REAL(0.0));
        a->physical_core->delta_world_f_core_psis = a->physical_core->speed;
        b->physical_core->delta_world_f_core_psis = b->physical_core->speed;
        probe = b;
    }
    begin_operation(env, m);
    int64_t start = now_ns();
    bench_simulate(env, cfg->steps, cfg, m, instrumented);
    m->operation_ns = now_ns() - start;
    m->operations = m->psis = cfg->steps;
    m->simulated_seconds = cfg->steps * cfg->dt;
    m->valid = probe && object_finite(probe);
    m->state_hash = hash_environment(env);
    collect_phases(env, m);
    bench_destroy(env);
    m->total_ns = now_ns() - total_start;
    return 1;
}

static int run_churn(const bench_config_t *cfg, const ivp_allocator_t *allocator,
                     bench_measurement_t *m, int instrumented)
{
    int64_t total_start = now_ns();
    ivp_environment_t *env = make_env(allocator, IVP_REAL(1.0 / 180.0), 0, 0,
                                      instrumented);
    if (!env) return 0;
    ivp_object_t **objects = (ivp_object_t **)ivp_alloc(allocator, (size_t)cfg->scale * sizeof(*objects));
    if (!objects) { ivp_environment_destroy(env); return 0; }
    const int cycles = 16;
    uint64_t state_hash = UINT64_C(1469598103934665603);
    begin_operation(env, m);
    int64_t start = now_ns();
    for (int cycle = 0; cycle < cycles; ++cycle) {
        for (int i = 0; i < cfg->scale; ++i) {
            objects[i] = make_ball(env, IVP_REAL(0.25), (ivp_real_t)(i * 2),
                                   (ivp_real_t)cycle, IVP_REAL(0.0));
            if (!objects[i]) { m->valid = 0; goto done_churn; }
        }
        bench_simulate(env, 1, cfg, m, instrumented);
        int64_t validation_start = now_ns();
        for (int i = 0; i < cfg->scale; ++i)
            state_hash = hash_object(state_hash, objects[i]);
        m->validation_ns += now_ns() - validation_start;
        for (int i = 0; i < cfg->scale; ++i) ivp_environment_destroy_object(env, objects[i]);
    }
    m->valid = 1;
done_churn:
    m->operation_ns = now_ns() - start - m->validation_ns;
    m->operations = (int64_t)cfg->scale * cycles;
    m->psis = cycles;
    m->simulated_seconds = cycles / 180.0;
    m->state_hash = state_hash;
    collect_phases(env, m);
    ivp_free(allocator, objects, (size_t)cfg->scale * sizeof(*objects));
    bench_destroy(env);
    m->total_ns = now_ns() - total_start;
    return m->valid;
}

static BENCH_NOINLINE int bench_sleep_warmup(ivp_environment_t *env)
{
    int all_frozen = 0;
    for (int warmup = 0; warmup < 600 && !all_frozen; ++warmup) {
        ivp_environment_simulate_one_psi(env);
        all_frozen = 1;
        const int count = ivp_environment_get_object_count(env);
        for (int i = 0; i < count; ++i) {
            const ivp_object_t *obj = ivp_environment_get_object(env, i);
            if (!obj->physical_core->physical_unmoveable && !ivp_object_is_frozen(obj)) {
                all_frozen = 0;
                break;
            }
        }
    }
    return all_frozen;
}

static int run_sleeping(const bench_config_t *cfg, const ivp_allocator_t *allocator,
                        bench_measurement_t *m, int instrumented)
{
    int64_t total_start = now_ns();
    ivp_environment_t *env = make_env(allocator, cfg->dt, 0, 0, instrumented);
    if (!env) return 0;
    ivp_object_t *probe = NULL;
    for (int i = 0; i < cfg->scale; ++i) {
        probe = make_ball(env, IVP_REAL(0.3), (ivp_real_t)((i % 32) * 2),
                          (ivp_real_t)(((i / 32) % 32) * 2), (ivp_real_t)((i / 1024) * 2));
        if (!probe) { ivp_environment_destroy(env); return 0; }
    }
    if (!bench_sleep_warmup(env)) { ivp_environment_destroy(env); return 0; }
    begin_operation(env, m);
    int64_t start = now_ns();
    bench_simulate(env, cfg->steps, cfg, m, instrumented);
    m->operation_ns = now_ns() - start;
    m->operations = m->psis = cfg->steps;
    m->simulated_seconds = cfg->steps * cfg->dt;
    m->valid = probe && object_finite(probe);
    m->state_hash = hash_environment(env);
    collect_phases(env, m);
    bench_destroy(env);
    m->total_ns = now_ns() - total_start;
    return 1;
}

static int run_convex(const bench_config_t *cfg, const ivp_allocator_t *allocator,
                      bench_measurement_t *m, int instrumented)
{
    (void)instrumented;
    int64_t total_start = now_ns();
    double *points = (double *)ivp_alloc(allocator, (size_t)cfg->scale * 3 * sizeof(*points));
    if (!points) return 0;
    for (int i = 0; i < cfg->scale; ++i) {
        double u = ((double)i + 0.5) / (double)cfg->scale;
        double phi = acos(1.0 - 2.0 * u);
        double theta = 2.39996322972865332 * (double)i;
        double radius = 1.0 + 0.05 * sin((double)i * 1.7);
        points[i * 3 + 0] = radius * sin(phi) * cos(theta);
        points[i * 3 + 1] = radius * cos(phi);
        points[i * 3 + 2] = radius * sin(phi) * sin(theta);
    }
    int64_t start = now_ns();
    ivp_compact_surface_t *surface = ivp_build_convex_surface_d(allocator, points, cfg->scale);
    m->operation_ns = now_ns() - start;
    m->operations = 1;
    m->valid = surface != NULL;
    if (surface) {
        int byte_size = ivp_surface_get_byte_size(surface);
        m->state_hash = hash_bytes(UINT64_C(1469598103934665603), surface,
                                   (size_t)byte_size);
        ivp_compact_surface_free(allocator, surface);
    }
    ivp_free(allocator, points, (size_t)cfg->scale * 3 * sizeof(*points));
    m->total_ns = now_ns() - total_start;
    return m->valid;
}

static BENCH_NOINLINE int run_once(const bench_config_t *cfg, const ivp_allocator_t *allocator,
                    bench_measurement_t *m, int instrumented)
{
    memset(m, 0, sizeof(*m));
    switch (cfg->kind) {
        case BENCH_ACTIVE: return run_active(cfg, allocator, m, instrumented);
        case BENCH_CONTACT: return run_contact(cfg, allocator, m, instrumented);
        case BENCH_DENSE_CONTACTS:
            return run_dense_contacts(cfg, allocator, m, instrumented);
        case BENCH_CONSTRAINT: return run_constraint(cfg, allocator, m, instrumented);
        case BENCH_ISLAND_CONSTRAINTS:
        case BENCH_MIXED_ISLAND_CONSTRAINTS:
            return run_island_constraints(cfg, allocator, m, instrumented);
        case BENCH_HETEROGENEOUS_ISLANDS:
            return run_heterogeneous_constraint_islands(
                cfg, allocator, m, instrumented);
        case BENCH_IMPACTS: return run_impacts(cfg, allocator, m, instrumented);
        case BENCH_CHURN: return run_churn(cfg, allocator, m, instrumented);
        case BENCH_SLEEPING: return run_sleeping(cfg, allocator, m, instrumented);
        case BENCH_CONVEX: return run_convex(cfg, allocator, m, instrumented);
    }
    return 0;
}

static int compare_double(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

static double percentile(double *values, int count, double p)
{
    double copy[MAX_SAMPLES];
    memcpy(copy, values, (size_t)count * sizeof(*copy));
    qsort(copy, (size_t)count, sizeof(*copy), compare_double);
    int index = (int)ceil(p * count) - 1;
    if (index < 0) index = 0;
    if (index >= count) index = count - 1;
    return copy[index];
}

static const char *phase_name(int phase)
{
    static const char *names[PHASE_COUNT] = {
        "start", "universe", "controllers", "integrators", "hull",
        "short_mindists", "critical_mindists", "psi_end", "at_init",
        "at_collision", "at_integrators", "at_hull", "at_short_mindists",
        "at_critical_mindists", "at_end", "user1", "user2"
    };
    return phase >= 0 && phase < PHASE_COUNT ? names[phase] : "unknown";
}

static int compare_ns(const void *a, const void *b)
{
    int64_t x = *(const int64_t *)a, y = *(const int64_t *)b;
    return (x > y) - (x < y);
}

static void print_diagnostic(FILE *out, const bench_config_t *cfg,
                             const bench_measurement_t *m,
                             const bench_measurement_t *latency)
{
    fprintf(out, ",\"diagnostic_operation_ns\":%lld,\"diagnostic_total_ns\":%lld,"
            "\"phase_scope\":\"simulation\",\"steps_per_world\":%d,\"dt\":%.17g",
            (long long)m->operation_ns, (long long)m->total_ns, cfg->steps, cfg->dt);
    fprintf(out, ",\"latency_operation_ns\":%lld,\"latency_total_ns\":%lld",
            (long long)latency->operation_ns, (long long)latency->total_ns);
    int count = latency->step_count;
    int64_t sorted[MAX_STEPS];
    if (count > MAX_STEPS) count = MAX_STEPS;
    memcpy(sorted, latency->step_ns, (size_t)count * sizeof(*sorted));
    qsort(sorted, (size_t)count, sizeof(*sorted), compare_ns);
    if (count) {
        fprintf(out, ",\"psi_latency_ns\":{\"count\":%d,\"median\":%lld,"
                "\"p95\":%lld,\"max\":%lld,\"scope\":\"%s\"}",
                count, (long long)sorted[(count - 1) / 2],
                (long long)sorted[(int)ceil(count * 0.95) - 1],
                (long long)sorted[count - 1],
                noop_counter_requested ? "single-world-noop-counter" : "single-world-counter-disabled");
    }
    fprintf(out, ",\"windows\":[");
    int comma = 0;
    int step = 0;
    for (int i = 0; i < WINDOW_COUNT; ++i) {
        if (!latency->window_steps[i]) continue;
        fprintf(out, "%s{\"first_step\":%d,\"steps\":%d,\"simulate_ns\":%lld,\"impacts\":%d}",
                comma ? "," : "", step, latency->window_steps[i],
                (long long)latency->window_ns[i], latency->window_impacts[i]);
        comma = 1;
        step += latency->window_steps[i];
    }
    fprintf(out, "]");
#ifdef IVP_BENCHMARK_DIAGNOSTICS
    const ivp_minlist_diagnostics_t *d = &m->minlist;
    fprintf(out, ",\"minlist_operations\":{\"adds\":%llu,\"removes\":%llu,"
            "\"tree_adds\":%llu,\"skip_adds\":%llu,\"fallback_adds\":%llu,"
            "\"finger_attempts\":%llu,\"finger_hits\":%llu,"
            "\"search_comparisons\":%llu,\"max_search_comparisons\":%llu,\"max_population\":%u,"
            "\"population_bucket_upper_bounds\":[15,63,255,1023,4095,null],\"population_buckets\":[",
            (unsigned long long)d->adds, (unsigned long long)d->removes,
            (unsigned long long)d->tree_adds, (unsigned long long)d->skip_adds,
            (unsigned long long)d->fallback_adds,
            (unsigned long long)d->finger_attempts, (unsigned long long)d->finger_hits,
            (unsigned long long)d->comparisons,
            (unsigned long long)d->max_comparisons, d->max_population);
    for (int i = 0; i < 6; ++i)
        fprintf(out, "%s%llu", i ? "," : "", (unsigned long long)d->population_buckets[i]);
    fprintf(out, "]}");
    const ivp_recheck_diagnostics_t *b = &m->broadphase;
    fprintf(out, ",\"broadphase_operations\":{\"rechecks\":%llu,\"recursive_rechecks\":%llu,"
            "\"max_depth\":%llu,\"candidates\":%llu,\"old_collisions\":%llu,"
            "\"max_candidates\":%llu,\"max_old_collisions\":%llu,\"potential_search_pairs\":%llu,"
            "\"searches\":%llu,\"found\":%llu,\"misses\":%llu,\"entries_compared\":%llu,"
            "\"max_scan\":%llu,\"swaps\":%llu,\"indexed_rechecks\":%llu,"
            "\"indexed_searches\":%llu,\"indexed_found\":%llu,\"index_entries\":%llu,"
            "\"index_probes\":%llu,\"max_index_probes\":%llu,\"duplicate_fallbacks\":%llu,"
            "\"unsafe_fallbacks\":%llu,\"oom_fallbacks\":%llu}",
            (unsigned long long)b->rechecks, (unsigned long long)b->recursive_rechecks,
            (unsigned long long)b->max_depth, (unsigned long long)b->candidates,
            (unsigned long long)b->old_collisions, (unsigned long long)b->max_candidates,
            (unsigned long long)b->max_old_collisions, (unsigned long long)b->potential_search_pairs,
            (unsigned long long)b->searches, (unsigned long long)b->found,
            (unsigned long long)b->misses, (unsigned long long)b->entries_compared,
            (unsigned long long)b->max_scan, (unsigned long long)b->swaps,
            (unsigned long long)b->indexed_rechecks, (unsigned long long)b->indexed_searches,
            (unsigned long long)b->indexed_found, (unsigned long long)b->index_entries,
            (unsigned long long)b->index_probes, (unsigned long long)b->max_index_probes,
            (unsigned long long)b->duplicate_fallbacks, (unsigned long long)b->unsafe_fallbacks,
            (unsigned long long)b->oom_fallbacks);
    fprintf(out, ",\"hull_operations\":{\"manager_checks\":%llu,\"callbacks\":%llu,"
            "\"max_callbacks_per_check\":%llu,\"polygon_callbacks\":%llu,"
            "\"anchor_callbacks\":%llu,\"oo_watcher_callbacks\":%llu,"
            "\"oo_connector_callbacks\":%llu,\"synapse_inserts\":%llu,"
            "\"lazy_synapse_inserts\":%llu,\"synapse_updates\":%llu,"
            "\"lazy_synapse_updates\":%llu,\"synapse_removes\":%llu,"
            "\"halfspace_extensions\":%llu,\"exact_promotions\":%llu,"
            "\"phantom_promotions\":%llu,\"recursive_events\":%llu,"
            "\"oo_events\":%llu",
            (unsigned long long)b->hull_manager_checks,
            (unsigned long long)b->hull_callbacks,
            (unsigned long long)b->max_hull_callbacks_per_check,
            (unsigned long long)b->hull_polygon_callbacks,
            (unsigned long long)b->hull_anchor_callbacks,
            (unsigned long long)b->hull_oo_watcher_callbacks,
            (unsigned long long)b->hull_oo_connector_callbacks,
            (unsigned long long)b->hull_synapse_inserts,
            (unsigned long long)b->hull_lazy_synapse_inserts,
            (unsigned long long)b->hull_synapse_updates,
            (unsigned long long)b->hull_lazy_synapse_updates,
            (unsigned long long)b->hull_synapse_removes,
            (unsigned long long)b->hull_halfspace_extensions,
            (unsigned long long)b->hull_exact_promotions,
            (unsigned long long)b->hull_phantom_promotions,
            (unsigned long long)b->hull_recursive_events,
            (unsigned long long)b->hull_oo_events);
    fputs(",\"callback_bucket_upper_bounds\":[0,1,3,7,15,31,null],"
          "\"callback_buckets\":[", out);
    for (int i = 0; i < 7; ++i)
        fprintf(out, "%s%llu", i ? "," : "",
                (unsigned long long)b->hull_callback_buckets[i]);
    fputs("]}", out);
    fprintf(out, ",\"contact_recalc_operations\":{\"scope\":\"per-simulation-time-v1\","
            "\"fingerprint_contract\":\"contact-input-128-v1\",\"calls\":%llu,"
            "\"branch_pp\":%llu,\"branch_pk\":%llu,\"branch_pf\":%llu,\"branch_kk\":%llu,"
            "\"same_time_calls\":%llu,\"same_geometry_fingerprint\":%llu,"
            "\"same_kinematic_fingerprint\":%llu,\"same_full_input_fingerprint\":%llu,"
            "\"max_calls_per_contact_at_time\":%llu,\"side_visits\":%llu,"
            "\"object_pose_first_uses\":%llu,\"object_pose_reuses\":%llu,"
            "\"core_state_first_uses\":%llu,\"core_state_reuses\":%llu,"
            "\"point_world_uses\":%llu,\"point_world_first_uses\":%llu,"
            "\"point_world_reuses\":%llu,\"edge_world_uses\":%llu,"
            "\"edge_world_first_uses\":%llu,\"edge_world_reuses\":%llu,"
            "\"face_normal_uses\":%llu,\"face_normal_first_uses\":%llu,"
            "\"face_normal_reuses\":%llu,\"face_tangent_uses\":%llu,"
            "\"face_tangent_first_uses\":%llu,\"face_tangent_reuses\":%llu,"
            "\"tracking_overflows\":%llu}",
            (unsigned long long)b->contact_recalc_calls,
            (unsigned long long)b->contact_branch_pp,
            (unsigned long long)b->contact_branch_pk,
            (unsigned long long)b->contact_branch_pf,
            (unsigned long long)b->contact_branch_kk,
            (unsigned long long)b->contact_same_time_calls,
            (unsigned long long)b->contact_same_geometry_fingerprint,
            (unsigned long long)b->contact_same_kinematic_fingerprint,
            (unsigned long long)b->contact_same_full_input_fingerprint,
            (unsigned long long)b->contact_max_calls_per_contact_at_time,
            (unsigned long long)b->contact_side_visits,
            (unsigned long long)b->contact_object_pose_first_uses,
            (unsigned long long)b->contact_object_pose_reuses,
            (unsigned long long)b->contact_core_state_first_uses,
            (unsigned long long)b->contact_core_state_reuses,
            (unsigned long long)b->contact_point_world_uses,
            (unsigned long long)b->contact_point_world_first_uses,
            (unsigned long long)b->contact_point_world_reuses,
            (unsigned long long)b->contact_edge_world_uses,
            (unsigned long long)b->contact_edge_world_first_uses,
            (unsigned long long)b->contact_edge_world_reuses,
            (unsigned long long)b->contact_face_normal_uses,
            (unsigned long long)b->contact_face_normal_first_uses,
            (unsigned long long)b->contact_face_normal_reuses,
            (unsigned long long)b->contact_face_tangent_uses,
            (unsigned long long)b->contact_face_tangent_first_uses,
            (unsigned long long)b->contact_face_tangent_reuses,
            (unsigned long long)b->contact_tracking_overflows);
#endif
}

static int profile_config(const bench_config_t *cfg, const options_t *opt, FILE *out)
{
    int64_t start = now_ns(), operation_ns = 0;
    uint64_t hash = 0;
    int worlds = 0;
    do {
        bench_measurement_t one;
        if (!run_once(cfg, ivp_default_allocator(), &one, 0) || !one.valid ||
            (worlds && one.state_hash != hash)) return 0;
        hash = one.state_hash;
        operation_ns += one.operation_ns;
        ++worlds;
    } while (now_ns() - start < opt->hotspot_ms * 1.0e6);
    fprintf(out, "{\"schema\":\"%s\",\"kind\":\"native-profile\","
            "\"implementation\":\"libivp\",\"case\":\"%s\",\"scale\":%d,"
            "\"parallel_workers\":%d,"
            "\"worlds\":%d,\"steps_per_world\":%d,\"operation_ns\":%lld,"
            "\"total_ns\":%lld,\"state_hash\":\"%016llx\"}\n",
            BENCH_SCHEMA, cfg->name, cfg->scale, benchmark_worker_count,
            worlds, cfg->steps,
            (long long)operation_ns, (long long)(now_ns() - start),
            (unsigned long long)hash);
    return 1;
}

static int benchmark_config(const bench_config_t *cfg, const options_t *opt, FILE *out)
{
    double normalized[MAX_SAMPLES];
    uint64_t expected_hash = 0;
    for (int warmup = 0; warmup < 1; ++warmup) {
        bench_measurement_t ignored;
        if (!run_once(cfg, ivp_default_allocator(), &ignored, 0)) return 0;
        expected_hash = ignored.state_hash;
    }
    for (int sample = 0; sample < opt->samples; ++sample) {
        bench_measurement_t aggregate;
        memset(&aggregate, 0, sizeof(aggregate));
        int iterations = 0;
        do {
            bench_measurement_t one;
            if (!run_once(cfg, ivp_default_allocator(), &one, 0) || !one.valid ||
                one.state_hash != expected_hash) return 0;
            aggregate.operation_ns += one.operation_ns;
            aggregate.total_ns += one.total_ns;
            aggregate.validation_ns += one.validation_ns;
            aggregate.operations += one.operations;
            aggregate.psis += one.psis;
            aggregate.simulated_seconds += one.simulated_seconds;
            ++iterations;
        } while (aggregate.operation_ns < (int64_t)(opt->min_sample_ms * 1000000.0));
        normalized[sample] = (double)aggregate.operation_ns / (double)aggregate.operations;
        if (!strcmp(opt->format, "jsonl")) {
            fprintf(out,
                    "{\"schema\":\"%s\",\"kind\":\"native\",\"implementation\":\"libivp\","
                    "\"case\":\"%s\",\"scale\":%d,\"sample\":%d,\"iterations\":%d,"
                    "\"parallel_workers\":%d,"
                    "\"operations\":%lld,\"psis\":%lld,\"simulated_seconds\":%.17g,"
                    "\"operation_ns\":%lld,\"total_ns\":%lld,\"validation_ns\":%lld,"
                    "\"timing_contract\":\"%s\",\"p95_scope\":\"batch-average\"}\n",
                    BENCH_SCHEMA, cfg->name, cfg->scale, sample, iterations,
                    benchmark_worker_count,
                    (long long)aggregate.operations, (long long)aggregate.psis,
                    aggregate.simulated_seconds, (long long)aggregate.operation_ns,
                    (long long)aggregate.total_ns, (long long)aggregate.validation_ns,
                    cfg->kind == BENCH_CHURN ? "churn-excluding-validation-v2" :
                        noop_counter_requested ? "native-noop-v1" : "native-v1");
        }
    }

    ivp_allocator_t counting = ivp_counting_allocator_create(NULL);
    bench_measurement_t memory_run;
    if (!run_once(cfg, &counting, &memory_run, 0) ||
        memory_run.state_hash != expected_hash) {
        ivp_counting_allocator_destroy(&counting);
        return 0;
    }
    ivp_alloc_stats_t stats;
    ivp_counting_allocator_get_stats(&counting, &stats);
    ivp_counting_allocator_destroy(&counting);

    bench_measurement_t diagnostic;
    if (!run_once(cfg, ivp_default_allocator(), &diagnostic, 1) ||
        diagnostic.state_hash != expected_hash) return 0;
    /* Per-PSI tails require individual observations, not percentiles of batch
     * averages. Only the two boundary clocks remain in this separate run. */
    bench_measurement_t latency;
    if (!run_once(cfg, ivp_default_allocator(), &latency, 2) ||
        latency.state_hash != expected_hash) return 0;
    bench_measurement_t verification;
    if (!run_once(cfg, ivp_default_allocator(), &verification, 3) ||
        verification.state_hash != expected_hash) return 0;
    counting = ivp_counting_allocator_create(NULL);
    if (!run_once(cfg, &counting, &memory_run, 3) ||
        memory_run.state_hash != expected_hash ||
        memory_run.verification.count != verification.verification.count ||
        memory_run.verification.trajectory != verification.verification.trajectory) {
        ivp_counting_allocator_destroy(&counting);
        fprintf(stderr, "%s: verification differs with counting allocator\n", cfg->name);
        return 0;
    }
    ivp_alloc_stats_t verification_stats;
    ivp_counting_allocator_get_stats(&counting, &verification_stats);
    ivp_counting_allocator_destroy(&counting);
    if (verification_stats.current_bytes != 0) return 0;
    int latency_count = latency.step_count;
    if (latency_count > MAX_STEPS) latency_count = MAX_STEPS;
    int64_t latency_sorted[MAX_STEPS];
    memcpy(latency_sorted, latency.step_ns, (size_t)latency_count * sizeof(*latency_sorted));
    qsort(latency_sorted, (size_t)latency_count, sizeof(*latency_sorted), compare_ns);
    double psi_p95 = latency_count
        ? (double)latency_sorted[(int)ceil(latency_count * 0.95) - 1] : 0.0;

    double median = percentile(normalized, opt->samples, 0.5);
    double p95 = percentile(normalized, opt->samples, 0.95);
    double minimum = percentile(normalized, opt->samples, 1.0 / opt->samples);
    double maximum = percentile(normalized, opt->samples, 1.0);
    int gate_ok = 1;
    if (opt->check) {
        if (cfg->kind == BENCH_CHURN) gate_ok = p95 < 10000000.0;
        else if (cfg->kind == BENCH_CONVEX) gate_ok = p95 < 500000000.0;
        else if (!strcmp(opt->profile, "large")) gate_ok = psi_p95 <= cfg->dt * 1.0e9;
        else gate_ok = p95 < cfg->dt * 4.0e9;
        if (stats.current_bytes != 0) gate_ok = 0;
    }

    if (!strcmp(opt->format, "jsonl")) {
        fprintf(out,
                "{\"schema\":\"%s\",\"kind\":\"native-summary\","
                "\"implementation\":\"libivp\",\"case\":\"%s\",\"scale\":%d,"
                "\"parallel_workers\":%d,"
                "\"median_ns_per_op\":%.3f,\"p95_ns_per_op\":%.3f,"
                "\"min_ns_per_op\":%.3f,\"max_ns_per_op\":%.3f,"
                "\"alloc_count\":%lld,\"bytes_allocated\":%lld,"
                "\"peak_bytes\":%lld,\"residual_bytes\":%lld,"
                "\"state_hash\":\"%016llx\",\"gate_ok\":%s,"
                "\"statistics\":{\"mindists_generated\":%d,\"mindists_deleted\":%d,"
                "\"range_intra_exceeded\":%d,\"range_world_exceeded\":%d,"
                "\"impacts\":%d},"
                "\"parallel_scope\":\"operation-v1\","
                "\"parallel\":{\"island_dispatches\":%llu,"
                "\"island_tasks\":%llu,\"broadphase_dispatches\":%llu,"
                "\"broadphase_tasks\":%llu,\"solver_dispatches\":%llu,"
                "\"solver_tasks\":%llu},"
                "\"parallel_lifetime_max_workers_used\":%d,"
                "\"parallel_before_operation\":{\"island_dispatches\":%llu,"
                "\"island_tasks\":%llu,\"broadphase_dispatches\":%llu,"
                "\"broadphase_tasks\":%llu,\"solver_dispatches\":%llu,"
                "\"solver_tasks\":%llu,\"max_workers_used\":%d},"
                "\"phases_ns\":{",
                BENCH_SCHEMA, cfg->name, cfg->scale, benchmark_worker_count,
                median, p95, minimum, maximum,
                (long long)stats.alloc_count, (long long)stats.bytes_allocated,
                (long long)stats.peak_bytes, (long long)stats.current_bytes,
                (unsigned long long)expected_hash,
                gate_ok ? "true" : "false",
                diagnostic.mindists_generated, diagnostic.mindists_deleted,
                diagnostic.range_intra_exceeded, diagnostic.range_world_exceeded,
                diagnostic.impacts,
                (unsigned long long)diagnostic.parallel.island_dispatches,
                (unsigned long long)diagnostic.parallel.island_tasks,
                (unsigned long long)diagnostic.parallel.broadphase_dispatches,
                (unsigned long long)diagnostic.parallel.broadphase_tasks,
                (unsigned long long)diagnostic.parallel.solver_dispatches,
                (unsigned long long)diagnostic.parallel.solver_tasks,
                diagnostic.parallel_lifetime_max_workers_used,
                (unsigned long long)diagnostic.parallel_before_operation.island_dispatches,
                (unsigned long long)diagnostic.parallel_before_operation.island_tasks,
                (unsigned long long)diagnostic.parallel_before_operation.broadphase_dispatches,
                (unsigned long long)diagnostic.parallel_before_operation.broadphase_tasks,
                (unsigned long long)diagnostic.parallel_before_operation.solver_dispatches,
                (unsigned long long)diagnostic.parallel_before_operation.solver_tasks,
                diagnostic.parallel_before_operation.max_workers_used);
        for (int i = 0; i < PHASE_COUNT; ++i) {
            if (i) fputc(',', out);
            fprintf(out, "\"%s\":%lld", phase_name(i),
                    (long long)diagnostic.phase_ns[i]);
        }
        fprintf(out, "}");
        print_diagnostic(out, cfg, &diagnostic, &latency);
        fprintf(out, ",\"verification_contract\":\"%s\",\"verification_steps\":%d,"
                "\"trajectory_hash\":\"%016llx\",\"contact_height\":%d", BENCH_TRACE_CONTRACT,
                verification.verification.count,
                (unsigned long long)verification.verification.trajectory, cfg->contact_height);
        fprintf(out, "}\n");
    } else {
        double throughput = median > 0.0 ? 1.0e9 / median : 0.0;
        fprintf(out, "%-18s n=%-5d workers=%-2d median=%9.3f us/op p95=%9.3f us/op throughput=%10.1f/s peak=%8.1f KiB%s\n",
                cfg->name, cfg->scale, benchmark_worker_count,
                median / 1000.0, p95 / 1000.0, throughput,
                (double)stats.peak_bytes / 1024.0, gate_ok ? "" : "  FAIL");
    }
    return gate_ok;
}

static const bench_config_t quick_configs[] = {
    {"active_bodies", BENCH_ACTIVE, 256, 240, 1.0 / 180.0},
    {"contact_pile", BENCH_CONTACT, 64, 360, 1.0 / 180.0},
    {"constraint_chain", BENCH_CONSTRAINT, 64, 360, 1.0 / 180.0},
    {"fast_impacts", BENCH_IMPACTS, 32, 180, 1.0 / 180.0},
    {"object_churn", BENCH_CHURN, 32, 16, 1.0 / 180.0},
    {"sleeping_world", BENCH_SLEEPING, 1024, 600, 1.0 / 60.0},
    {"convex_build", BENCH_CONVEX, 128, 1, 0.0}
};

static const bench_config_t standard_configs[] = {
    {"active_bodies", BENCH_ACTIVE, 64, 240, 1.0 / 180.0},
    {"active_bodies", BENCH_ACTIVE, 256, 240, 1.0 / 180.0},
    {"active_bodies", BENCH_ACTIVE, 1024, 240, 1.0 / 180.0},
    {"contact_pile", BENCH_CONTACT, 16, 360, 1.0 / 180.0},
    {"contact_pile", BENCH_CONTACT, 64, 360, 1.0 / 180.0},
    {"contact_pile", BENCH_CONTACT, 256, 360, 1.0 / 180.0},
    {"constraint_chain", BENCH_CONSTRAINT, 16, 360, 1.0 / 180.0},
    {"constraint_chain", BENCH_CONSTRAINT, 64, 360, 1.0 / 180.0},
    {"constraint_chain", BENCH_CONSTRAINT, 256, 360, 1.0 / 180.0},
    {"fast_impacts", BENCH_IMPACTS, 8, 180, 1.0 / 180.0},
    {"fast_impacts", BENCH_IMPACTS, 32, 180, 1.0 / 180.0},
    {"fast_impacts", BENCH_IMPACTS, 128, 180, 1.0 / 180.0},
    {"object_churn", BENCH_CHURN, 16, 16, 1.0 / 180.0},
    {"object_churn", BENCH_CHURN, 64, 16, 1.0 / 180.0},
    {"object_churn", BENCH_CHURN, 256, 16, 1.0 / 180.0},
    {"sleeping_world", BENCH_SLEEPING, 256, 600, 1.0 / 60.0},
    {"sleeping_world", BENCH_SLEEPING, 1024, 600, 1.0 / 60.0},
    {"sleeping_world", BENCH_SLEEPING, 4096, 600, 1.0 / 60.0},
    {"convex_build", BENCH_CONVEX, 32, 1, 0.0},
    {"convex_build", BENCH_CONVEX, 128, 1, 0.0},
    {"convex_build", BENCH_CONVEX, 512, 1, 0.0}
};

static const bench_config_t large_configs[] = {
    {"active_bodies", BENCH_ACTIVE, 4096, 240, 1.0 / 180.0},
    {"contact_pile", BENCH_CONTACT, 1024, 360, 1.0 / 180.0},
    {"dense_contacts", BENCH_DENSE_CONTACTS, 128, 60, 1.0 / 180.0},
    {"constraint_chain", BENCH_CONSTRAINT, 1024, 360, 1.0 / 180.0},
    {"constraint_islands", BENCH_ISLAND_CONSTRAINTS, 4096, 32, 1.0 / 180.0},
    {"mixed_constraint_islands", BENCH_MIXED_ISLAND_CONSTRAINTS, 4096, 32,
     1.0 / 180.0},
    {"heterogeneous_constraint_islands", BENCH_HETEROGENEOUS_ISLANDS, 4096,
     32, 1.0 / 180.0},
    {"fast_impacts", BENCH_IMPACTS, 512, 180, 1.0 / 180.0},
    {"object_churn", BENCH_CHURN, 1024, 16, 1.0 / 180.0},
    {"sleeping_world", BENCH_SLEEPING, 16384, 600, 1.0 / 60.0},
    {"convex_build", BENCH_CONVEX, 2048, 1, 0.0}
};

int main(int argc, char **argv)
{
    options_t opt = {
        .profile = "quick",
        .format = "text",
        .workers = 1,
    };
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) { usage(argv[0]); return 0; }
        if (!strcmp(argv[i], "--profile") && i + 1 < argc) { opt.profile = argv[++i]; continue; }
        if (!strcmp(argv[i], "--case") && i + 1 < argc) { opt.case_filter = argv[++i]; continue; }
        if (!strcmp(argv[i], "--samples") && i + 1 < argc && parse_positive_int(argv[++i], &opt.samples)) continue;
        if (!strcmp(argv[i], "--min-sample-ms") && i + 1 < argc && parse_positive_double(argv[++i], &opt.min_sample_ms)) continue;
        if (!strcmp(argv[i], "--hotspot-ms") && i + 1 < argc && parse_positive_double(argv[++i], &opt.hotspot_ms)) continue;
        if (!strcmp(argv[i], "--scale") && i + 1 < argc && parse_positive_int(argv[++i], &opt.scale)) continue;
        if (!strcmp(argv[i], "--format") && i + 1 < argc) { opt.format = argv[++i]; continue; }
        if (!strcmp(argv[i], "--output") && i + 1 < argc) { opt.output_path = argv[++i]; continue; }
        if (!strcmp(argv[i], "--check")) { opt.check = 1; continue; }
        if (!strcmp(argv[i], "--verify")) { opt.verify = 1; continue; }
        if (!strcmp(argv[i], "--noop-counter")) { opt.noop_counter = 1; continue; }
        if (!strcmp(argv[i], "--workers") && i + 1 < argc &&
            parse_positive_int(argv[++i], &opt.workers)) continue;
        if (!strcmp(argv[i], "--contact-height") && i + 1 < argc && parse_positive_int(argv[++i], &opt.contact_height)) continue;
        if (!strcmp(argv[i], "--list")) { opt.list = 1; continue; }
        usage(argv[0]); return 2;
    }
    if (strcmp(opt.profile, "quick") && strcmp(opt.profile, "standard") &&
        strcmp(opt.profile, "large")) { usage(argv[0]); return 2; }
    if (strcmp(opt.format, "text") && strcmp(opt.format, "jsonl")) { usage(argv[0]); return 2; }
    /* Native benchmark JSONL must also be parseable in Debug builds. */
    if (!strcmp(opt.format, "jsonl") || opt.hotspot_ms || opt.verify)
        ivp_set_message_output(benchmark_message, NULL);
    noop_counter_requested = opt.noop_counter;
    benchmark_worker_count = opt.workers;
    if (opt.contact_height && (!opt.case_filter || strcmp(opt.case_filter, "contact_pile"))) {
        fprintf(stderr, "--contact-height requires --case contact_pile\n"); return 2;
    }
    if ((opt.hotspot_ms || opt.scale) && !opt.case_filter) {
        fprintf(stderr, "--hotspot-ms and --scale require --case\n"); return 2;
    }
    if (opt.verify && (!opt.case_filter || opt.hotspot_ms || opt.check || opt.list ||
                      !strcmp(opt.case_filter, "convex_build"))) {
        fprintf(stderr, "--verify requires one simulation case and cannot be combined with timing/check/list modes\n");
        return 2;
    }
    if (opt.hotspot_ms && opt.check) {
        fprintf(stderr, "--hotspot-ms cannot be used with --check\n"); return 2;
    }
    const bench_config_t *configs;
    size_t count;
    if (!strcmp(opt.profile, "quick")) {
        configs = quick_configs;
        count = sizeof(quick_configs) / sizeof(quick_configs[0]);
    } else if (!strcmp(opt.profile, "standard")) {
        configs = standard_configs;
        count = sizeof(standard_configs) / sizeof(standard_configs[0]);
    } else {
        configs = large_configs;
        count = sizeof(large_configs) / sizeof(large_configs[0]);
    }
    if (opt.list) {
        const char *previous = NULL;
        for (size_t i = 0; i < count; ++i) {
            if (!previous || strcmp(previous, configs[i].name)) {
                puts(configs[i].name);
                previous = configs[i].name;
            }
        }
        return 0;
    }
    if (!opt.samples) opt.samples = !strcmp(opt.profile, "standard") ? 7 : 5;
    if (opt.samples > MAX_SAMPLES) { fprintf(stderr, "--samples must be <= %d\n", MAX_SAMPLES); return 2; }
    if (opt.min_sample_ms == 0.0) {
        if (!strcmp(opt.profile, "quick")) opt.min_sample_ms = 200.0;
        else if (!strcmp(opt.profile, "standard")) opt.min_sample_ms = 250.0;
        else opt.min_sample_ms = 500.0;
    }
    FILE *out = stdout;
    if (opt.output_path) {
        out = fopen(opt.output_path, "w");
        if (!out) { perror(opt.output_path); return 1; }
    }
    int ok = 1, matched = 0;
    for (size_t i = 0; i < count; ++i) {
        if (opt.case_filter && strcmp(opt.case_filter, configs[i].name)) continue;
        matched = 1;
        bench_config_t cfg = configs[i];
        cfg.contact_height = opt.contact_height;
        if (opt.scale) cfg.scale = opt.scale;
        if (cfg.contact_height > cfg.scale) {
            fprintf(stderr, "--contact-height must not exceed object count\n");
            if (out != stdout) fclose(out);
            return 2;
        }
        if (opt.verify) {
            bench_measurement_t verification;
            if (!run_once(&cfg, ivp_default_allocator(), &verification, 3)) ok = 0;
            else {
                for (int j = 0; j < verification.verification.count; ++j)
                    bench_print_step(out, "libivp", cfg.name, cfg.scale, j,
                                     &verification.verification.steps[j]);
                fprintf(out, "{\"schema\":\"%s\",\"kind\":\"native-verification\","
                        "\"verification_contract\":\"%s\",\"implementation\":\"libivp\","
                        "\"case\":\"%s\",\"scale\":%d,\"steps_per_world\":%d,\"dt\":%.17g,"
                        "\"parallel_workers\":%d,"
                        "\"state_hash\":\"%016llx\",\"trajectory_hash\":\"%016llx\",\"contact_height\":%d}\n",
                        BENCH_SCHEMA, BENCH_TRACE_CONTRACT, cfg.name, cfg.scale, cfg.steps,
                        cfg.dt, benchmark_worker_count,
                        (unsigned long long)verification.state_hash,
                        (unsigned long long)verification.verification.trajectory, cfg.contact_height);
            }
        } else if (opt.hotspot_ms) {
            if (!profile_config(&cfg, &opt, out)) ok = 0;
        } else if (!benchmark_config(&cfg, &opt, out)) ok = 0;
        if (opt.scale || opt.verify) break;
    }
    if (opt.output_path) fclose(out);
    if (!matched) { fprintf(stderr, "Unknown case: %s\n", opt.case_filter ? opt.case_filter : ""); return 2; }
    return ok ? 0 : 1;
}
