/* test_diagnostics.c — IVP_PerformanceCounter, IVP_BetterStatisticsmanager
 * (+ console callback), IVP_BetterDebugmanager / IVP_Debug_Manager and
 * IVP_Clustering_Visualizer ports (ivp/diagnostics/).
 */
#include "ivp_test.h"

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_assert.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/events/ivp_listener_collision.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/geometry/ivp_ledgetree.h>
#include <ivp/builder/ivp_build_surface.h>
#include <ivp/builder/ivp_build_ledge.h>
#include <ivp/builder/ivp_build_ledge_soup.h>
#include <ivp/diagnostics/ivp_performance_counter.h>
#include <ivp/diagnostics/ivp_better_statistics_manager.h>
#include <ivp/diagnostics/ivp_debug_manager.h>
#include <ivp/diagnostics/ivp_clustering_visualizer.h>

#include <stdio.h>
#include <string.h>

/* ── helpers ───────────────────────────────────────────────────────────── */

static ivp_object_t *make_ball(ivp_environment_t *env, ivp_real_t x, ivp_real_t y, ivp_real_t vx)
{
    ivp_object_template_t tmpl;
    ivp_object_template_init(&tmpl);
    tmpl.type = IVP_OBJECT_BALL;
    tmpl.ball_radius = IVP_REAL(1.0);
    tmpl.mass = IVP_REAL(1.0);
    ivp_vec3r_set(&tmpl.position, x, y, IVP_REAL(0.0));
    ivp_object_t *obj = ivp_environment_create_object(env, &tmpl, NULL);
    if (obj) ivp_vec3r_set(&obj->physical_core->speed, vx, IVP_REAL(0.0), IVP_REAL(0.0));
    return obj;
}

static ivp_environment_t *make_env_no_gravity(ivp_performance_counter_t *pc)
{
    ivp_environment_config_t cfg;
    ivp_environment_config_init(&cfg);
    ivp_vec3r_set(&cfg.gravity, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(0.0));
    cfg.performance_counter = pc;
    return ivp_environment_create(&cfg);
}

/* read everything written to a tmpfile */
static void read_back(FILE *f, char *buf, size_t size)
{
    fflush(f);
    rewind(f);
    size_t n = fread(buf, 1, size - 1, f);
    buf[n] = '\0';
}

/* ── recording performance counter ─────────────────────────────────────── */

#define REC_START   100
#define REC_STOP    101
#define REC_IMPACT  200
#define REC_MAX     256

typedef struct rec_counter {
    ivp_performance_counter_t base;
    int   seq[REC_MAX];
    int   n;
    int   deleted_calls;
    ivp_environment_t *deleted_env;
    int   reset_calls;
    ivp_time_t reset_time;
} rec_counter_t;

static void rec_push(rec_counter_t *r, int v)
{
    if (r->n < REC_MAX) r->seq[r->n++] = v;
}
static void rec_start(ivp_performance_counter_t *pc) { rec_push((rec_counter_t *)pc, REC_START); }
static void rec_pcount(ivp_performance_counter_t *pc, ivp_performance_element_t el) { rec_push((rec_counter_t *)pc, (int)el); }
static void rec_stop(ivp_performance_counter_t *pc) { rec_push((rec_counter_t *)pc, REC_STOP); }
static void rec_deleted(ivp_performance_counter_t *pc, struct ivp_environment *env)
{
    rec_counter_t *r = (rec_counter_t *)pc;
    r->deleted_calls++;
    r->deleted_env = env;
}
static void rec_reset(ivp_performance_counter_t *pc, ivp_time_t t)
{
    rec_counter_t *r = (rec_counter_t *)pc;
    r->reset_calls++;
    r->reset_time = t;
}
static const ivp_performance_counter_vtable_t rec_vt = {
    rec_start, rec_pcount, rec_stop, rec_deleted, rec_reset
};

static void rec_init(rec_counter_t *r)
{
    memset(r, 0, sizeof(*r));
    r->base.vt = &rec_vt;
}

static void rec_on_pre_collision(void *ctx, const ivp_collision_event_t *ev)
{
    (void)ev;
    rec_push((rec_counter_t *)ctx, REC_IMPACT);
}

/* ── performance counter ───────────────────────────────────────────────── */

IVP_TEST(default_env_creates_simple_counter)
{
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);
    ivp_performance_counter_t *pc = ivp_environment_get_performance_counter(env);
    IVP_ASSERT_NOT_NULL(pc);
    ivp_performance_counter_simple_t probe;
    ivp_performance_counter_simple_init(&probe);
    IVP_ASSERT_TRUE(pc->vt == probe.base.vt);

    ivp_performance_counter_simple_t *simple = (ivp_performance_counter_simple_t *)pc;
    IVP_ASSERT_EQ(simple->count_PSIs, 0u);
    IVP_ASSERT_TRUE(simple->owner_allocator != NULL); /* freed by the environment */

    /* no collisions: every simulate run ends with the PSI phases */
    for (int i = 0; i < 5; i++) ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_EQ(simple->count_PSIs, 5u);
    IVP_ASSERT_EQ((int)simple->counting, (int)IVP_PE_PSI_END);
    for (int i = 0; i < IVP_PE_MAX; i++) IVP_ASSERT_TRUE(simple->counter[i] >= IVP_REAL(0.0));

    ivp_environment_destroy(env); /* environment_is_going_to_be_deleted frees the counter */
}

IVP_TEST(simple_counter_accumulates_and_resets)
{
    ivp_performance_counter_simple_t pc;
    ivp_performance_counter_simple_init(&pc);
    IVP_ASSERT_EQ((int)pc.counting, (int)IVP_PE_PSI_START);
    IVP_ASSERT_TRUE(pc.inv_counter_freq > 0.0);
    ivp_performance_counter_t *base = &pc.base;

    FILE *out = tmpfile();
    IVP_ASSERT_NOT_NULL(out);
    pc.out = out;

    /* nothing counted: nothing printed, nothing reset */
    ivp_performance_counter_reset_and_print_performance_counters(base, 3.0);
    char buf[2048];
    read_back(out, buf, sizeof(buf));
    IVP_ASSERT_EQ(buf[0], '\0');
    IVP_ASSERT_TRUE(pc.time_of_last_reset == 0.0);

    ivp_performance_counter_start_pcount(base);
    ivp_performance_counter_pcount(base, IVP_PE_PSI_UNIVERSE);
    volatile double sink = 0.0;
    for (int i = 0; i < 200000; i++) sink += (double)i * 1e-9;
    ivp_performance_counter_pcount(base, IVP_PE_PSI_CONTROLLERS);
    ivp_performance_counter_pcount(base, IVP_PE_PSI_END);
    ivp_performance_counter_stop_pcount(base);
    (void)sink;

    IVP_ASSERT_EQ(pc.count_PSIs, 1u);
    IVP_ASSERT_EQ((int)pc.counting, (int)IVP_PE_PSI_END);
    IVP_ASSERT_TRUE(pc.counter[IVP_PE_PSI_UNIVERSE] > IVP_REAL(0.0)); /* microseconds of the busy loop */

    ivp_performance_counter_start_pcount(base);
    IVP_ASSERT_EQ((int)pc.counting, (int)IVP_PE_PSI_START);

    ivp_performance_counter_reset_and_print_performance_counters(base, 3.0);
    read_back(out, buf, sizeof(buf));
    IVP_ASSERT_TRUE(strncmp(buf, "[performance] TOTAL ", 20) == 0);
    IVP_ASSERT_NOT_NULL(strstr(buf, "\nCOLLISION "));
    IVP_ASSERT_NOT_NULL(strstr(buf, "\nDYNAMIC "));
    IVP_ASSERT_NOT_NULL(strstr(buf, "\n\nUNIVERSE: "));
    IVP_ASSERT_NOT_NULL(strstr(buf, "\nCONTROLLERS: "));
    IVP_ASSERT_NOT_NULL(strstr(buf, "\nINTEGRATORS: "));
    IVP_ASSERT_NOT_NULL(strstr(buf, "\nHULL: "));
    IVP_ASSERT_NOT_NULL(strstr(buf, "\nSHORT MINDISTS: "));
    IVP_ASSERT_NOT_NULL(strstr(buf, "\nCRITICAL MINDISTS: "));
    IVP_ASSERT_NOT_NULL(strstr(buf, "\nSIMULATE "));

    IVP_ASSERT_EQ(pc.count_PSIs, 0u);
    IVP_ASSERT_EQ((int)pc.counting, (int)IVP_PE_PSI_START);
    for (int i = 0; i < IVP_PE_MAX; i++) IVP_ASSERT_TRUE(pc.counter[i] == IVP_REAL(0.0));
    IVP_ASSERT_TRUE(pc.time_of_last_reset == 3.0);

    /* in-place counters are not freed by environment_is_going_to_be_deleted */
    ivp_performance_counter_environment_is_going_to_be_deleted(base, NULL);
    IVP_ASSERT_EQ(pc.count_PSIs, 0u);
    fclose(out);
}

IVP_TEST(user_counter_receives_reference_sequence)
{
    rec_counter_t rec;
    rec_init(&rec);
    ivp_environment_t *env = make_env_no_gravity(&rec.base);
    IVP_ASSERT_NOT_NULL(env);
    IVP_ASSERT_TRUE(ivp_environment_get_performance_counter(env) == &rec.base);

    /* two balls (radius 1) 0.2 apart closing at 20 m/s: they collide
     * 0.01 s after the first PSI, before the second (1/66 s) */
    ivp_object_t *a = make_ball(env, IVP_REAL(-1.1), IVP_REAL(0.0), IVP_REAL(10.0));
    ivp_object_t *b = make_ball(env, IVP_REAL(1.1), IVP_REAL(0.0), IVP_REAL(-10.0));
    IVP_ASSERT_NOT_NULL(a);
    IVP_ASSERT_NOT_NULL(b);

    ivp_collision_listener_t listener;
    ivp_collision_listener_init(&listener, IVP_COLLISION_CB_PRE_COLLISION, &rec);
    listener.pre_collision_fn = rec_on_pre_collision;
    IVP_ASSERT_EQ(ivp_environment_add_collision_listener_global(env, &listener), 0);

    IVP_ASSERT_EQ(rec.n, 0); /* nothing counted outside the simulate calls */
    ivp_environment_simulate_one_psi(env);

    static const int expected[] = {
        REC_START,
        IVP_PE_PSI_UNIVERSE, IVP_PE_PSI_CONTROLLERS, IVP_PE_PSI_INTEGRATORS, IVP_PE_PSI_HULL,
        IVP_PE_PSI_SHORT_MINDISTS, IVP_PE_PSI_CRITICAL_MINDISTS, IVP_PE_PSI_END,
        IVP_PE_AT_INIT, REC_IMPACT, IVP_PE_AT_END,
        REC_STOP,
    };
    const int n_expected = (int)(sizeof(expected) / sizeof(expected[0]));
    if (rec.n != n_expected) {
        fprintf(stderr, "  recorded sequence (%d):", rec.n);
        for (int i = 0; i < rec.n; i++) fprintf(stderr, " %d", rec.seq[i]);
        fprintf(stderr, "\n");
    }
    IVP_ASSERT_EQ(rec.n, n_expected);
    for (int i = 0; i < n_expected; i++) IVP_ASSERT_EQ(rec.seq[i], expected[i]);

    /* the next step: a plain PSI (the balls separate) */
    rec.n = 0;
    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_TRUE(rec.n >= 9);
    IVP_ASSERT_EQ(rec.seq[0], REC_START);
    IVP_ASSERT_EQ(rec.seq[1], (int)IVP_PE_PSI_UNIVERSE);
    IVP_ASSERT_EQ(rec.seq[7], (int)IVP_PE_PSI_END);
    IVP_ASSERT_EQ(rec.seq[rec.n - 1], REC_STOP);

    /* variable time step: start / one PSI / stop */
    rec.n = 0;
    ivp_environment_simulate_variable_time_step(env, IVP_REAL(1.0) / IVP_REAL(60.0));
    IVP_ASSERT_TRUE(rec.n >= 9);
    IVP_ASSERT_EQ(rec.seq[0], REC_START);
    IVP_ASSERT_EQ(rec.seq[1], (int)IVP_PE_PSI_UNIVERSE);
    IVP_ASSERT_EQ(rec.seq[rec.n - 1], REC_STOP);

    /* a simulate call too short to do anything still brackets the run */
    rec.n = 0;
    ivp_environment_simulate_dtime(env, IVP_REAL(0.001));
    IVP_ASSERT_TRUE(rec.n >= 2);
    IVP_ASSERT_EQ(rec.seq[0], REC_START);
    IVP_ASSERT_EQ(rec.seq[rec.n - 1], REC_STOP);

    IVP_ASSERT_EQ(rec.deleted_calls, 0);
    IVP_ASSERT_EQ(rec.reset_calls, 0); /* the engine never resets / prints by itself */
    ivp_environment_destroy(env);
    IVP_ASSERT_EQ(rec.deleted_calls, 1);
    IVP_ASSERT_TRUE(rec.deleted_env == env);
}

/* counters measure time only: a user counter changes nothing */
static bool same_vec(const ivp_vec3r_t *a, const ivp_vec3r_t *b)
{
    return a->k[0] == b->k[0] && a->k[1] == b->k[1] && a->k[2] == b->k[2];
}

static bool same_core_state(const ivp_core_t *a, const ivp_core_t *b)
{
    const ivp_transform_r_t *ma = &a->m_world_f_core_last_psi, *mb = &b->m_world_f_core_last_psi;
    for (int r = 0; r < 3; r++) {
        if (!same_vec(&ma->rot.rows[r], &mb->rot.rows[r])) return false;
    }
    return same_vec(&ma->pos, &mb->pos) && same_vec(&a->speed, &b->speed) &&
           a->rot_speed.k[0] == b->rot_speed.k[0] && a->rot_speed.k[1] == b->rot_speed.k[1] &&
           a->rot_speed.k[2] == b->rot_speed.k[2];
}

IVP_TEST(counter_does_not_change_simulation)
{
    rec_counter_t rec;
    rec_init(&rec);
    ivp_environment_t *env0 = make_env_no_gravity(NULL);
    ivp_environment_t *env1 = make_env_no_gravity(&rec.base);
    IVP_ASSERT_NOT_NULL(env0);
    IVP_ASSERT_NOT_NULL(env1);
    ivp_object_t *a0 = make_ball(env0, IVP_REAL(-1.1), IVP_REAL(0.0), IVP_REAL(10.0));
    ivp_object_t *b0 = make_ball(env0, IVP_REAL(1.1), IVP_REAL(0.3), IVP_REAL(-10.0));
    ivp_object_t *a1 = make_ball(env1, IVP_REAL(-1.1), IVP_REAL(0.0), IVP_REAL(10.0));
    ivp_object_t *b1 = make_ball(env1, IVP_REAL(1.1), IVP_REAL(0.3), IVP_REAL(-10.0));
    IVP_ASSERT_TRUE(a0 && b0 && a1 && b1);
    for (int i = 0; i < 30; i++) {
        ivp_environment_simulate_one_psi(env0);
        ivp_environment_simulate_one_psi(env1);
    }
    IVP_ASSERT_TRUE(same_core_state(a0->physical_core, a1->physical_core));
    IVP_ASSERT_TRUE(same_core_state(b0->physical_core, b1->physical_core));
    ivp_environment_destroy(env0);
    ivp_environment_destroy(env1);
}

/* ── better statistics manager ─────────────────────────────────────────── */

IVP_TEST(statistics_manager_follows_simulation_time)
{
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);
    ivp_better_statistics_manager_t *m = ivp_environment_get_better_statistics_manager(env);
    IVP_ASSERT_NOT_NULL(m);
    IVP_ASSERT_TRUE(ivp_better_statistics_manager_get_state(m));
    IVP_ASSERT_TRUE(m->update_delayed);
    IVP_ASSERT_TRUE(m->update_interval == IVP_REAL(1.0));
    IVP_ASSERT_TRUE(ivp_better_statistics_manager_get_simulation_time(m) == IVP_REAL(0.0));

    ivp_environment_simulate_dtime(env, IVP_REAL(0.5));
    IVP_ASSERT_TRUE(m->simulation_time == (ivp_real_t)ivp_environment_get_time(env));
    IVP_ASSERT_TRUE(m->update_delayed); /* 0.5 - 0 < 1 */

    ivp_environment_simulate_dtime(env, IVP_REAL(0.6));
    IVP_ASSERT_TRUE(m->simulation_time == (ivp_real_t)ivp_environment_get_time(env));
    IVP_ASSERT_FALSE(m->update_delayed);
    IVP_ASSERT_TRUE(m->time_of_last_update == m->simulation_time);

    /* nothing to simulate: the current time */
    ivp_environment_simulate_until(env, ivp_environment_get_time(env));
    IVP_ASSERT_TRUE(m->simulation_time == (ivp_real_t)ivp_environment_get_time(env));
    IVP_ASSERT_TRUE(m->update_delayed);
    ivp_environment_simulate_dtime(env, IVP_REAL(0.0));
    IVP_ASSERT_TRUE(m->simulation_time == (ivp_real_t)ivp_environment_get_time(env));

    ivp_environment_simulate_until(env, ivp_environment_get_time(env) + 0.25);
    IVP_ASSERT_TRUE(m->simulation_time == (ivp_real_t)ivp_environment_get_time(env));

    ivp_environment_simulate_one_psi(env);
    IVP_ASSERT_TRUE(m->simulation_time == (ivp_real_t)ivp_environment_get_time(env));

    /* variable step: current time + clamped delta (set before simulating) */
    ivp_time_t before = ivp_environment_get_time(env);
    ivp_environment_simulate_variable_time_step(env, IVP_REAL(0.02));
    IVP_ASSERT_TRUE(m->simulation_time == (ivp_real_t)(before + (double)0.02f));

    ivp_environment_destroy(env);
}

static int g_cb_enable_calls;
typedef struct counting_callback {
    ivp_better_statistics_callback_t base;
    int requests;
} counting_callback_t;
static void counting_output(ivp_better_statistics_callback_t *cb, ivp_better_statistics_data_entity_t *e)
{
    (void)e;
    ((counting_callback_t *)cb)->requests++;
}
static void counting_enable(ivp_better_statistics_callback_t *cb) { (void)cb; g_cb_enable_calls++; }
static void counting_disable(ivp_better_statistics_callback_t *cb) { (void)cb; g_cb_enable_calls--; }
static const ivp_better_statistics_callback_vtable_t counting_vt = { counting_output, counting_enable, counting_disable };

IVP_TEST(statistics_manager_print_and_console_callback)
{
    ivp_better_statistics_manager_t m;
    ivp_better_statistics_manager_init(&m);

    ivp_better_statistics_data_entity_t e_int, e_double, e_iarr, e_darr, e_str, e_off;
    ivp_better_statistics_data_entity_init(&e_int, IVP_BSM_INT_VALUE);
    ivp_better_statistics_data_entity_init(&e_double, IVP_BSM_DOUBLE_VALUE);
    ivp_better_statistics_data_entity_init(&e_iarr, IVP_BSM_INT_ARRAY);
    ivp_better_statistics_data_entity_init(&e_darr, IVP_BSM_DOUBLE_ARRAY);
    ivp_better_statistics_data_entity_init(&e_str, IVP_BSM_STRING);
    ivp_better_statistics_data_entity_init(&e_off, IVP_BSM_INT_VALUE);

    /* constructor defaults */
    IVP_ASSERT_TRUE(strcmp(e_int.text, "Value: ") == 0);
    IVP_ASSERT_TRUE(ivp_better_statistics_data_entity_get_state(&e_int));
    IVP_ASSERT_EQ(e_iarr.data.int_array.size, 10);
    IVP_ASSERT_EQ(e_iarr.data.int_array.max_value, 10);
    IVP_ASSERT_EQ(e_iarr.data.int_array.border_color, 1);
    IVP_ASSERT_EQ(e_iarr.data.int_array.graph_color, 2);
    IVP_ASSERT_EQ(e_darr.data.double_array.size, 10);
    IVP_ASSERT_TRUE(e_darr.data.double_array.max_value == IVP_REAL(10.0));

    ivp_better_statistics_data_entity_set_int_value(&e_int, 42);
    ivp_better_statistics_data_entity_set_double_value(&e_double, IVP_REAL(2.5));
    ivp_better_statistics_data_entity_set_text(&e_double, "D=");
    ivp_better_statistics_data_entity_set_array_size(&e_iarr, 3);
    ivp_better_statistics_data_entity_set_text(&e_iarr, "IA");
    for (int v = 1; v <= 4; v++) ivp_better_statistics_data_entity_set_int_array_latest_value(&e_iarr, v);
    ivp_better_statistics_data_entity_set_array_size(&e_darr, 2);
    ivp_better_statistics_data_entity_set_text(&e_darr, "DA");
    ivp_better_statistics_data_entity_set_double_array_latest_value(&e_darr, IVP_REAL(0.5));
    ivp_better_statistics_data_entity_set_double_array_latest_value(&e_darr, IVP_REAL(1.25));
    ivp_better_statistics_data_entity_set_text(&e_str, "hello");
    ivp_better_statistics_data_entity_set_position(&e_str, 3, 4);
    IVP_ASSERT_EQ(e_str.xpos, 3);
    IVP_ASSERT_EQ(e_str.ypos, 4);
    ivp_better_statistics_data_entity_set_int_value(&e_off, 7);
    ivp_better_statistics_data_entity_disable(&e_off);
    IVP_ASSERT_FALSE(ivp_better_statistics_data_entity_get_state(&e_off));

    /* mismatching setters leave the entity alone */
    ivp_better_statistics_data_entity_set_int_value(&e_iarr, 99);
    IVP_ASSERT_EQ(e_iarr.data.int_array.size, 3);

    ivp_better_statistics_manager_install_data_entity(&m, &e_int);
    ivp_better_statistics_manager_install_data_entity(&m, &e_double);
    ivp_better_statistics_manager_install_data_entity(&m, &e_off);
    ivp_better_statistics_manager_install_data_entity(&m, &e_iarr);
    ivp_better_statistics_manager_install_data_entity(&m, &e_darr);
    ivp_better_statistics_manager_install_data_entity(&m, &e_str);

    ivp_statistics_console_callback_t con;
    ivp_statistics_console_callback_init(&con);
    IVP_ASSERT_TRUE(con.out == stdout);
    FILE *out = tmpfile();
    IVP_ASSERT_NOT_NULL(out);
    con.out = out;
    ivp_better_statistics_manager_install_output_callback(&m, &con.base);

    counting_callback_t cnt = { { &counting_vt }, 0 };
    ivp_better_statistics_manager_install_output_callback(&m, &cnt.base);

    char buf[1024];

    /* update delayed after construction: no output */
    ivp_better_statistics_manager_print(&m);
    read_back(out, buf, sizeof(buf));
    IVP_ASSERT_EQ(buf[0], '\0');
    IVP_ASSERT_EQ(cnt.requests, 0);

    ivp_better_statistics_manager_set_simulation_time(&m, IVP_REAL(0.75));
    IVP_ASSERT_TRUE(m.update_delayed);
    ivp_better_statistics_manager_set_simulation_time(&m, IVP_REAL(1.0));
    IVP_ASSERT_FALSE(m.update_delayed);
    IVP_ASSERT_TRUE(m.time_of_last_update == IVP_REAL(1.0));

    ivp_better_statistics_manager_print(&m);
    read_back(out, buf, sizeof(buf));
    const char *expected =
        "Value: 42\n"
        "D=2.500000\n"
        "IA\n2\n3\n4\n"
        "DA\n0.500000\n1.250000\n"
        "hello\n";
    if (strcmp(buf, expected) != 0) fprintf(stderr, "  console output:\n%s\n", buf);
    IVP_ASSERT_TRUE(strcmp(buf, expected) == 0);
    IVP_ASSERT_EQ(cnt.requests, 5); /* disabled entity skipped */

    /* disabled manager prints nothing */
    ivp_better_statistics_manager_disable(&m);
    IVP_ASSERT_FALSE(ivp_better_statistics_manager_get_state(&m));
    ivp_better_statistics_manager_print(&m);
    IVP_ASSERT_EQ(cnt.requests, 5);
    ivp_better_statistics_manager_enable(&m);

    /* removal (also of things never installed) */
    ivp_better_statistics_manager_remove_output_callback(&m, &con.base);
    ivp_better_statistics_manager_remove_output_callback(&m, &con.base);
    ivp_better_statistics_manager_remove_data_entity(&m, &e_int);
    ivp_better_statistics_manager_remove_data_entity(&m, &e_int);
    ivp_better_statistics_manager_print(&m);
    IVP_ASSERT_EQ(cnt.requests, 9);

    /* console enable / disable do nothing */
    con.base.vt->enable(&con.base);
    con.base.vt->disable(&con.base);

    /* array resize clears; negative sizes become 0 */
    ivp_better_statistics_data_entity_set_array_size(&e_iarr, -5);
    IVP_ASSERT_EQ(e_iarr.data.int_array.size, 0);
    ivp_better_statistics_data_entity_set_int_array_latest_value(&e_iarr, 1);

    ivp_better_statistics_manager_destroy(&m);
    ivp_better_statistics_data_entity_destroy(&e_int);
    ivp_better_statistics_data_entity_destroy(&e_double);
    ivp_better_statistics_data_entity_destroy(&e_iarr);
    ivp_better_statistics_data_entity_destroy(&e_darr);
    ivp_better_statistics_data_entity_destroy(&e_str);
    ivp_better_statistics_data_entity_destroy(&e_off);
    fclose(out);
}

/* ── debug managers ────────────────────────────────────────────────────── */

typedef struct debug_capture {
    char text[256];
    ivp_debug_class_t last_class;
    int calls;
} debug_capture_t;

static void capture_output(ivp_better_debug_manager_t *dm, ivp_debug_class_t class_id, const char *string)
{
    debug_capture_t *c = (debug_capture_t *)dm->output_ctx;
    snprintf(c->text, sizeof(c->text), "%s", string);
    c->last_class = class_id;
    c->calls++;
}

IVP_TEST(better_debug_manager)
{
    ivp_better_debug_manager_t dm;
    ivp_better_debug_manager_init(&dm);
    const ivp_debug_class_t user_class = (ivp_debug_class_t)1500;
    IVP_ASSERT_FALSE(ivp_better_debug_manager_is_debug_enabled(&dm, IVP_DM_QHULL));
    IVP_ASSERT_FALSE(ivp_better_debug_manager_is_debug_enabled(&dm, user_class));
    ivp_better_debug_manager_enable_debug_output(&dm, user_class);
    IVP_ASSERT_TRUE(ivp_better_debug_manager_is_debug_enabled(&dm, user_class));
    ivp_better_debug_manager_disable_debug_output(&dm, user_class);
    IVP_ASSERT_FALSE(ivp_better_debug_manager_is_debug_enabled(&dm, user_class));

    /* out of range classes are ignored */
    ivp_better_debug_manager_enable_debug_output(&dm, IVP_DEBUG_MAX_N_CLASSES);
    IVP_ASSERT_FALSE(ivp_better_debug_manager_is_debug_enabled(&dm, IVP_DEBUG_MAX_N_CLASSES));
    ivp_better_debug_manager_enable_debug_output(&dm, (ivp_debug_class_t)-1);
    IVP_ASSERT_FALSE(ivp_better_debug_manager_is_debug_enabled(&dm, (ivp_debug_class_t)-1));

    /* not initialized: nothing is enabled */
    ivp_better_debug_manager_enable_debug_output(&dm, IVP_DM_QHULL);
    dm.initialized = 0;
    IVP_ASSERT_FALSE(ivp_better_debug_manager_is_debug_enabled(&dm, IVP_DM_QHULL));
    dm.initialized = 1;

    debug_capture_t cap;
    memset(&cap, 0, sizeof(cap));
    dm.output_function = capture_output;
    dm.output_ctx = &cap;
    ivp_better_debug_manager_dprint(&dm, IVP_DM_GEOMPACK_LEVEL1, "N=%d %s\n", 5, "faces");
    IVP_ASSERT_EQ(cap.calls, 1);
    IVP_ASSERT_TRUE(strcmp(cap.text, "N=5 faces\n") == 0);
    IVP_ASSERT_EQ((int)cap.last_class, (int)IVP_DM_GEOMPACK_LEVEL1);

    /* the global instance is constructed statically */
    IVP_ASSERT_EQ(ivp_debugmanager.initialized, 1);
    int hits = 0;
    IVP_IFDEBUG(IVP_DM_SURBUILD_Q12) { hits++; }
    IVP_ASSERT_EQ(hits, 0);
    ivp_better_debug_manager_enable_debug_output(&ivp_debugmanager, IVP_DM_SURBUILD_Q12);
    IVP_IFDEBUG(IVP_DM_SURBUILD_Q12) { hits++; }
    IVP_ASSERT_EQ(hits, 1);
    ivp_better_debug_manager_disable_debug_output(&ivp_debugmanager, IVP_DM_SURBUILD_Q12);
}

IVP_TEST(environment_debug_manager_counts_psis)
{
    ivp_environment_t *env = ivp_environment_create(NULL);
    IVP_ASSERT_NOT_NULL(env);
    ivp_debug_manager_t *dm = ivp_environment_get_debug_manager(env);
    IVP_ASSERT_NOT_NULL(dm);
    IVP_ASSERT_TRUE(dm->psi_counter == IVP_REAL(0.0));
    IVP_ASSERT_FALSE(dm->check_fs);
    IVP_ASSERT_NULL(dm->out_deb_file);
    for (int i = 0; i < 3; i++) ivp_environment_simulate_one_psi(env);
#ifdef IVP_DEBUG_BUILD
    IVP_ASSERT_TRUE(dm->psi_counter == IVP_REAL(3.0));
#else
    /* the reference counts PSIs only in debug builds (IVP_IF(1)) */
    IVP_ASSERT_TRUE(dm->psi_counter == IVP_REAL(0.0));
#endif

    /* clear closes the debug file */
    dm->out_deb_file = tmpfile();
    IVP_ASSERT_NOT_NULL(dm->out_deb_file);
    ivp_debug_manager_clear(dm);
    IVP_ASSERT_NULL(dm->out_deb_file);
    ivp_environment_destroy(env);
}

/* ── clustering visualizer ─────────────────────────────────────────────── */

#define CV_MAX 16
typedef struct node_rec {
    ivp_clustering_visualizer_shortrange_callback_t base;
    int n_nodes;
    const ivp_compact_ledgetree_node_t *nodes[CV_MAX];
    int n_touches[CV_MAX];
    bool reported[CV_MAX];
    ivp_real_t box_x[CV_MAX];
    ivp_object_t *pp[CV_MAX];
    int n_intruder;
    ivp_vec3r_t intruder_pos;
    ivp_real_t intruder_radius;
} node_rec_t;

static void node_rec_node(ivp_clustering_visualizer_shortrange_callback_t *cb)
{
    node_rec_t *r = (node_rec_t *)cb;
    if (r->n_nodes >= CV_MAX) return;
    r->nodes[r->n_nodes] = cb->node;
    r->n_touches[r->n_nodes] = cb->n_touches;
    r->reported[r->n_nodes] = cb->reported_ledge;
    r->box_x[r->n_nodes] = cb->box_extents.k[0];
    r->pp[r->n_nodes] = cb->private_property;
    r->n_nodes++;
}
static void node_rec_intruder(ivp_clustering_visualizer_shortrange_callback_t *cb)
{
    node_rec_t *r = (node_rec_t *)cb;
    r->n_intruder++;
    r->intruder_pos = cb->position;
    r->intruder_radius = cb->box_extents.k[0];
}
static const ivp_clustering_visualizer_shortrange_callback_vtable_t node_rec_vt = { node_rec_node, node_rec_intruder };

static ivp_compact_ledge_t *make_box_ledge(float cx)
{
    ivp_vec3f_t pts[8];
    int i = 0;
    for (int sx = -1; sx <= 1; sx += 2)
        for (int sy = -1; sy <= 1; sy += 2)
            for (int sz = -1; sz <= 1; sz += 2) pts[i++] = ivp_vec3f(cx + (float)sx, (float)sy, (float)sz);
    return ivp_build_convex_ledge(NULL, pts, 8);
}

IVP_TEST(clustering_visualizer_shortrange)
{
    /* compound surface: two unit boxes at x = -2 and x = +2 */
    ivp_compact_ledge_t *la = make_box_ledge(-2.0f);
    ivp_compact_ledge_t *lb = make_box_ledge(2.0f);
    IVP_ASSERT_NOT_NULL(la);
    IVP_ASSERT_NOT_NULL(lb);
    ivp_ledge_soup_t soup;
    ivp_ledge_soup_init(&soup, NULL);
    IVP_ASSERT_EQ(ivp_ledge_soup_add(&soup, la), 0);
    IVP_ASSERT_EQ(ivp_ledge_soup_add(&soup, lb), 0);
    ivp_compact_surface_t *surface = ivp_ledge_soup_compile(&soup);
    IVP_ASSERT_NOT_NULL(surface);
    const ivp_compact_ledgetree_node_t *root = ivp_surface_get_ledgetree_root(surface);
    IVP_ASSERT_FALSE(ivp_ledgetree_is_terminal(root));

    ivp_environment_t *env = make_env_no_gravity(NULL);
    IVP_ASSERT_NOT_NULL(env);
    ivp_surman_polygon_t sm;
    ivp_surman_polygon_init(&sm, surface);
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_POLYGON;
    t.mass = IVP_REAL(1.0);
    t.physical_unmoveable = true;
    ivp_object_t *poly = ivp_environment_create_object(env, &t, &sm.base);
    IVP_ASSERT_NOT_NULL(poly);
    /* the ball (intruder) sits above the box at x = -2 */
    ivp_object_t *ball = make_ball(env, IVP_REAL(-2.0), IVP_REAL(1.5), IVP_REAL(0.0));
    IVP_ASSERT_NOT_NULL(ball);

    ivp_clustering_visualizer_t vis;
    ivp_clustering_visualizer_init(&vis);
    ivp_clustering_visualizer_shortrange_t *sr = &vis.shortrange;
    IVP_ASSERT_FALSE(ivp_clustering_visualizer_shortrange_get_state(sr));
    IVP_ASSERT_EQ((int)sr->boxmode, (int)IVP_CV_TRAVERSED_BOXES);

    node_rec_t rec;
    memset(&rec, 0, sizeof(rec));
    ivp_clustering_visualizer_shortrange_callback_init(&rec.base, &node_rec_vt);
    IVP_ASSERT_TRUE(rec.base.sphere_radius == IVP_REAL(-1.0));
    IVP_ASSERT_EQ(rec.base.n_touches, -1);
    ivp_clustering_visualizer_shortrange_install_visualize_callback(sr, &rec.base);

    /* disabled: nothing happens */
    ivp_clustering_visualizer_shortrange_analyze_collision(sr, ball, IVP_REAL(0.75), poly, IVP_REAL(0.0));
    IVP_ASSERT_EQ(sr->stats_n_sphere_collisiontests, 0);

    ivp_clustering_visualizer_shortrange_enable(sr);
    ivp_clustering_visualizer_shortrange_analyze_collision(sr, ball, IVP_REAL(0.75), poly, IVP_REAL(0.0));
    /* root (inner) + box A (terminal hit) pass both tests, box B fails the sphere test;
     * the ball as private property is skipped (no ledgetree) */
    IVP_ASSERT_EQ(sr->stats_n_sphere_collisiontests, 3);
    IVP_ASSERT_EQ(sr->stats_n_box_collisiontests, 2);

    ivp_clustering_visualizer_shortrange_visualize_collisions(sr);
    IVP_ASSERT_EQ(rec.n_nodes, 3);
    IVP_ASSERT_EQ(sr->stats_n_reported_ledges, 1);
    IVP_ASSERT_EQ(sr->stats_n_sphere_collisiontests, 0);
    IVP_ASSERT_EQ(sr->stats_n_box_collisiontests, 0);
    /* pipeline order root, left, right handed out newest first */
    IVP_ASSERT_TRUE(rec.nodes[2] == root);
    IVP_ASSERT_EQ(rec.n_touches[2], 0);
    IVP_ASSERT_FALSE(rec.reported[2]);
    IVP_ASSERT_TRUE(rec.box_x[2] == (ivp_real_t)root->box_sizes[0] *
                                       (ivp_real_t)(IVP_COMPACT_BOUNDINGBOX_STEP_SIZE * root->radius));
    int hit = -1, miss = -1;
    for (int i = 0; i < 2; i++) {
        IVP_ASSERT_TRUE(ivp_ledgetree_is_terminal(rec.nodes[i]));
        IVP_ASSERT_EQ(rec.n_touches[i], 1);
        IVP_ASSERT_TRUE(rec.pp[i] == poly);
        if (rec.nodes[i]->center[0] < 0.0f) hit = i; else miss = i;
    }
    IVP_ASSERT_TRUE(hit >= 0 && miss >= 0);
    IVP_ASSERT_TRUE(rec.reported[hit]);
    IVP_ASSERT_FALSE(rec.reported[miss]);
    /* the intruder sphere, in world coordinates */
    IVP_ASSERT_EQ(rec.n_intruder, 1);
    IVP_ASSERT_NEAR(rec.intruder_pos.k[0], -2.0, 1e-6);
    IVP_ASSERT_NEAR(rec.intruder_pos.k[1], 1.5, 1e-6);
    IVP_ASSERT_TRUE(rec.intruder_radius == IVP_REAL(0.75));

    /* pipeline consumed */
    rec.n_nodes = 0;
    rec.n_intruder = 0;
    ivp_clustering_visualizer_shortrange_visualize_collisions(sr);
    IVP_ASSERT_EQ(rec.n_nodes, 0);
    IVP_ASSERT_EQ(rec.n_intruder, 0);

    /* terminals only: just the touched terminal, touched twice */
    sr->boxmode = IVP_CV_TERMINALS_ONLY;
    ivp_clustering_visualizer_shortrange_analyze_collision(sr, ball, IVP_REAL(0.75), poly, IVP_REAL(0.0));
    ivp_clustering_visualizer_shortrange_analyze_collision(sr, ball, IVP_REAL(0.75), poly, IVP_REAL(0.0));
    ivp_clustering_visualizer_shortrange_visualize_collisions(sr);
    IVP_ASSERT_EQ(rec.n_nodes, 1);
    IVP_ASSERT_TRUE(rec.reported[0]);
    IVP_ASSERT_EQ(rec.n_touches[0], 2);
    IVP_ASSERT_TRUE(rec.nodes[0]->center[0] < 0.0f);

    /* intruder filter */
    rec.n_nodes = 0;
    sr->use_intruder_accept_filter = true;
    ivp_clustering_visualizer_shortrange_analyze_collision(sr, ball, IVP_REAL(0.75), poly, IVP_REAL(0.0));
    ivp_clustering_visualizer_shortrange_visualize_collisions(sr);
    IVP_ASSERT_EQ(rec.n_nodes, 0);
    ivp_clustering_visualizer_shortrange_add_object_to_intruder_filter(sr, ball, IVP_CV_FILTER_ACCEPT);
    ivp_clustering_visualizer_shortrange_add_object_to_intruder_filter(sr, ball, IVP_CV_FILTER_ACCEPT);
    ivp_clustering_visualizer_shortrange_analyze_collision(sr, ball, IVP_REAL(0.75), poly, IVP_REAL(0.0));
    ivp_clustering_visualizer_shortrange_visualize_collisions(sr);
    IVP_ASSERT_EQ(rec.n_nodes, 1);
    ivp_clustering_visualizer_shortrange_remove_object_from_intruder_filter(sr, ball, IVP_CV_FILTER_ACCEPT);
    IVP_ASSERT_NULL(ivp_clustering_visualizer_object_hash_find(&sr->accept_filter_intruder, ball));

    ivp_clustering_visualizer_shortrange_remove_visualize_callback(sr, &rec.base);
    ivp_clustering_visualizer_shortrange_disable(sr);
    IVP_ASSERT_FALSE(ivp_clustering_visualizer_shortrange_get_state(sr));

    ivp_clustering_visualizer_destroy(&vis);
    ivp_environment_destroy(env);
    ivp_ledge_soup_destroy(&soup);
    ivp_compact_surface_free(NULL, surface);
    ivp_compact_ledge_free(NULL, la);
    ivp_compact_ledge_free(NULL, lb);
}

typedef struct sphere_rec {
    ivp_clustering_visualizer_longrange_callback_t base;
    int visualized, devisualized, enables, disables;
    ivp_object_t *last;
    ivp_vec3f_t center;
    ivp_real_t radius;
} sphere_rec_t;
static void sphere_vis(ivp_clustering_visualizer_longrange_callback_t *cb)
{
    sphere_rec_t *r = (sphere_rec_t *)cb;
    r->visualized++;
    r->last = cb->real_object;
    r->center = cb->center;
    r->radius = cb->radius;
}
static void sphere_devis(ivp_clustering_visualizer_longrange_callback_t *cb)
{
    sphere_rec_t *r = (sphere_rec_t *)cb;
    r->devisualized++;
    r->last = cb->real_object;
}
static void sphere_enable(ivp_clustering_visualizer_longrange_callback_t *cb) { ((sphere_rec_t *)cb)->enables++; }
static void sphere_disable(ivp_clustering_visualizer_longrange_callback_t *cb) { ((sphere_rec_t *)cb)->disables++; }
static const ivp_clustering_visualizer_longrange_callback_vtable_t sphere_vt = {
    sphere_vis, sphere_devis, sphere_enable, sphere_disable
};

IVP_TEST(clustering_visualizer_longrange)
{
    ivp_environment_t *env = make_env_no_gravity(NULL);
    IVP_ASSERT_NOT_NULL(env);
    ivp_object_t *moving = make_ball(env, IVP_REAL(0.0), IVP_REAL(0.0), IVP_REAL(1.0));
    ivp_object_template_t t;
    ivp_object_template_init(&t);
    t.type = IVP_OBJECT_BALL;
    t.ball_radius = IVP_REAL(1.0);
    t.mass = IVP_REAL(1.0);
    t.physical_unmoveable = true;
    ivp_vec3r_set(&t.position, IVP_REAL(10.0), IVP_REAL(0.0), IVP_REAL(0.0));
    ivp_object_t *fixed = ivp_environment_create_object(env, &t, NULL);
    IVP_ASSERT_TRUE(moving && fixed);

    ivp_clustering_visualizer_longrange_t lr;
    ivp_clustering_visualizer_longrange_init(&lr);
    IVP_ASSERT_FALSE(ivp_clustering_visualizer_longrange_get_state(&lr));
    IVP_ASSERT_TRUE(lr.use_accept_filter && lr.use_reject_filter);
    IVP_ASSERT_FALSE(lr.visualize_moveable_objects || lr.visualize_unmoveable_objects);

    sphere_rec_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.base.vt = &sphere_vt;
    ivp_clustering_visualizer_longrange_install_visualize_callback(&lr, &rec.base);

    ivp_vec3f_t c0 = ivp_vec3f(1.0f, 2.0f, 3.0f);
    ivp_vec3f_t c1 = ivp_vec3f(10.0f, 0.0f, 0.0f);
    ivp_clustering_visualizer_longrange_add_object(&lr, moving, &c0, IVP_REAL(4.0)); /* queued even when disabled */
    ivp_clustering_visualizer_longrange_add_object(&lr, fixed, &c1, IVP_REAL(2.0));
    IVP_ASSERT_EQ(ivp_pvec_len(&lr.visualize_data), 2);

    ivp_clustering_visualizer_longrange_visualize(&lr); /* disabled */
    IVP_ASSERT_EQ(rec.visualized, 0);

    ivp_clustering_visualizer_longrange_enable(&lr);
    IVP_ASSERT_EQ(rec.enables, 1);
    IVP_ASSERT_TRUE(rec.base.enabled);

    ivp_clustering_visualizer_longrange_visualize(&lr); /* moveable / unmoveable both off */
    IVP_ASSERT_EQ(rec.visualized, 0);
    lr.visualize_moveable_objects = true;
    ivp_clustering_visualizer_longrange_visualize(&lr); /* empty accept filter */
    IVP_ASSERT_EQ(rec.visualized, 0);
    ivp_clustering_visualizer_longrange_add_object_to_filter(&lr, moving, IVP_CV_FILTER_ACCEPT);
    ivp_clustering_visualizer_longrange_add_object_to_filter(&lr, moving, IVP_CV_FILTER_REJECT);
    ivp_clustering_visualizer_longrange_visualize(&lr); /* rejected */
    IVP_ASSERT_EQ(rec.visualized, 0);
    ivp_clustering_visualizer_longrange_remove_object_from_filter(&lr, moving, IVP_CV_FILTER_REJECT);
    ivp_clustering_visualizer_longrange_visualize(&lr);
    IVP_ASSERT_EQ(rec.visualized, 1);
    IVP_ASSERT_TRUE(rec.last == moving);
    IVP_ASSERT_TRUE(rec.center.k[0] == 1.0f && rec.center.k[1] == 2.0f && rec.center.k[2] == 3.0f);
    IVP_ASSERT_TRUE(rec.radius == IVP_REAL(4.0));
    IVP_ASSERT_EQ(ivp_pvec_len(&lr.visualize_data), 1); /* the static one stays queued */

    ivp_clustering_visualizer_longrange_remove_object(&lr, fixed);
    IVP_ASSERT_EQ(rec.devisualized, 1);
    IVP_ASSERT_TRUE(rec.last == fixed);
    IVP_ASSERT_EQ(ivp_pvec_len(&lr.visualize_data), 0);

    ivp_clustering_visualizer_longrange_disable(&lr);
    IVP_ASSERT_EQ(rec.disables, 1);
    IVP_ASSERT_FALSE(rec.base.enabled);
    ivp_clustering_visualizer_longrange_remove_object(&lr, fixed); /* disabled: ignored */
    IVP_ASSERT_EQ(rec.devisualized, 1);

    ivp_clustering_visualizer_longrange_remove_visualize_callback(&lr, &rec.base);
    ivp_clustering_visualizer_longrange_destroy(&lr);
    ivp_environment_destroy(env);
}

static int g_cv_core_count = 0;
static void cv_count_core(const char *expr, const char *file, int line,
                          const char *func, const char *msg, void *userdata)
{
    (void)file; (void)line; (void)func; (void)msg; (void)userdata;
    if (strcmp(expr, "CORE") == 0) g_cv_core_count++;
}

IVP_TEST(clustering_visualizer_object_hash)
{
    ivp_clustering_visualizer_object_hash_t h;
    ivp_clustering_visualizer_object_hash_init(&h, 3);
    ivp_object_t objs[40];
    for (int i = 0; i < 40; i++) ivp_clustering_visualizer_object_hash_add(&h, &objs[i]);
    for (int i = 0; i < 40; i++) IVP_ASSERT_TRUE(ivp_clustering_visualizer_object_hash_find(&h, &objs[i]) == &objs[i]);
    IVP_ASSERT_TRUE(ivp_clustering_visualizer_object_hash_remove(&h, &objs[7]) == &objs[7]);
    IVP_ASSERT_NULL(ivp_clustering_visualizer_object_hash_find(&h, &objs[7]));
    /* removing an absent object: CORE in the reference (IVP_VHash::remove_elem),
     * reported to the handler, then NULL */
    g_cv_core_count = 0;
    ivp_set_assert_handler(cv_count_core, NULL);
    IVP_ASSERT_NULL(ivp_clustering_visualizer_object_hash_remove(&h, &objs[7]));
    ivp_set_assert_handler(NULL, NULL);
    IVP_ASSERT_EQ(g_cv_core_count, 1);
    ivp_clustering_visualizer_object_hash_destroy(&h);
}

int main(void)
{
    IVP_REGISTER(default_env_creates_simple_counter);
    IVP_REGISTER(simple_counter_accumulates_and_resets);
    IVP_REGISTER(user_counter_receives_reference_sequence);
    IVP_REGISTER(counter_does_not_change_simulation);
    IVP_REGISTER(statistics_manager_follows_simulation_time);
    IVP_REGISTER(statistics_manager_print_and_console_callback);
    IVP_REGISTER(better_debug_manager);
    IVP_REGISTER(environment_debug_manager_counts_psis);
    IVP_REGISTER(clustering_visualizer_shortrange);
    IVP_REGISTER(clustering_visualizer_longrange);
    IVP_REGISTER(clustering_visualizer_object_hash);
    IVP_RUN_ALL();
}
