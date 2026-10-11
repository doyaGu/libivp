/* Native workload counterpart. Uses the unmodified, external reference engine. */
#include <ivp_physics.hxx>
#include <ivp_core.hxx>
#include <ivp_templates.hxx>
#include <ivp_material.hxx>
#include <ivp_performancecounter.hxx>
#include <ivp_surman_polygon.hxx>
#include <ivp_surbuild_pointsoup.hxx>
#include <ivp_listener_collision.hxx>
#include "bench_trace.h"
#include "bench_scene.h"
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <vector>
#ifdef BENCH_WORK_DIAGNOSTICS
#include "bench_work.h"
#include <ivp_hull_manager.hxx>
#include <ivp_time.hxx>
#endif

struct NoClock : IVP_PerformanceCounter {
    void start_pcount() override {}
    void pcount(IVP_PERFORMANCE_ELEMENT) override {}
    void stop_pcount() override {}
    void reset_and_print_performance_counters(IVP_Time) override {}
    void environment_is_going_to_be_deleted(IVP_Environment *) override { delete this; }
};

static long long now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Scene : IVP_Listener_Collision {
    IVP_Environment *env;
    std::vector<IVP_Real_Object *> objects;
    IVP_Material_Simple ball_material;
    IVP_Material_Simple ground_material;
    IVP_Compact_Surface *surface = nullptr;
    IVP_SurfaceManager_Polygon *surman = nullptr;
    std::vector<bench_step_t> trace;
    uint64_t events = BENCH_HASH_INIT, event_count = 0, trajectory = BENCH_HASH_INIT;
    bool observing = false;

    Scene(bool contacts) : IVP_Listener_Collision(
        IVP_LISTENER_COLLISION_CALLBACK_PRE_COLLISION |
        IVP_LISTENER_COLLISION_CALLBACK_POST_COLLISION |
        IVP_LISTENER_COLLISION_CALLBACK_FRICTION),
        ball_material(0.7, contacts ? 0.1 : 0.9), ground_material(0.5, 0.3)
    {
        IVP_Application_Environment config;
        config.n_cache_object = 256;
        config.performancecounter = new NoClock;
        env = IVP_Environment_Manager::get_environment_manager()->create_environment(&config, "native benchmark", 0);
        env->set_delta_PSI_time(1.0 / 180.0);
        env->reset_time();
        IVP_U_Point gravity;
        gravity.set(0.0, contacts ? (double)9.83f : 0.0, 0.0);
        env->set_gravity(&gravity);
    }

    ~Scene() override
    {
        observing = false;
        delete env;
        delete surman;
        if (surface) P_FREE_ALIGNED(surface);
    }

    IVP_Real_Object *ball(double radius, double x, double y, double z)
    {
        IVP_Template_Real_Object t;
        t.mass = 1.0;
        t.rot_inertia_is_factor = IVP_FALSE;
        double inertia = 0.4 * radius * radius;
        t.rot_inertia.set(inertia, inertia, inertia);
        t.speed_damp_factor = 0.0;
        t.rot_speed_damp_factor.set_to_zero();
        t.enable_piling_optimization = IVP_FALSE;
        t.material = &ball_material;
        IVP_Template_Ball b;
        b.radius = (float)radius;
        IVP_U_Quat q;
        q.x = q.y = q.z = 0.0; q.w = 1.0;
        IVP_U_Point p;
        p.set(x, y, z);
        IVP_Real_Object *o = env->create_ball(&b, &t, &q, &p);
        o->enable_collision_detection(IVP_TRUE);
        o->ensure_in_simulation_now();
        objects.push_back(o);
        return o;
    }

    void ground(float extent)
    {
        IVP_U_Vector<IVP_U_Point> points(8);
        IVP_U_Point corners[8];
        int i = 0;
        for (int x = -1; x <= 1; x += 2)
            for (int y = -1; y <= 1; y += 2)
                for (int z = -1; z <= 1; z += 2) {
                    corners[i].set(x * (double)extent, y * 0.5, z * (double)extent);
                    points.add(&corners[i++]);
                }
        surface = IVP_SurfaceBuilder_Pointsoup::convert_pointsoup_to_compact_surface(&points);
        if (!surface) std::abort();
        surman = new IVP_SurfaceManager_Polygon(surface);
        IVP_Template_Real_Object t;
        t.physical_unmoveable = IVP_TRUE;
        t.mass = 1e6;
        t.rot_inertia_is_factor = IVP_FALSE;
        t.rot_inertia.set(1e6, 1e6, 1e6);
        t.material = &ground_material;
        IVP_U_Quat q;
        q.x = q.y = q.z = 0.0; q.w = 1.0;
        IVP_U_Point p;
        p.set(0.0, 3.0, 0.0);
        IVP_Real_Object *o = env->create_polygon(surman, &t, &q, &p);
        o->enable_collision_detection(IVP_TRUE);
        objects.push_back(o);
    }

    uint64_t id(IVP_Real_Object *o) const
    {
        for (size_t i = 0; i < objects.size(); ++i) if (objects[i] == o) return i + 1;
        return 0;
    }

    void event(int kind, IVP_Contact_Situation *s)
    {
        if (!observing) return;
        events = bench_hash_u64(events, kind);
        events = bench_hash_double(events, env->get_current_time().get_time());
        events = bench_hash_u64(events, id(s->objects[0]));
        events = bench_hash_u64(events, id(s->objects[1]));
        ++event_count;
    }
    void event_pre_collision(IVP_Event_Collision *e) override { event(1, e->contact_situation); }
    void event_post_collision(IVP_Event_Collision *e) override { event(2, e->contact_situation); }
    void event_friction_created(IVP_Event_Friction *e) override { event(3, e->contact_situation); }
    void event_friction_deleted(IVP_Event_Friction *e) override { event(4, e->contact_situation); }

    void snapshot()
    {
        bench_step_t r = {};
        r.object_count = (int)objects.size();
        uint64_t h = BENCH_HASH_INIT;
        for (IVP_Real_Object *o : objects) {
            IVP_Core *c = o->get_core();
            h = bench_hash_u64(h, o->get_type() == IVP_BALL ? 1 : 2);
            h = bench_hash_u64(h, o->get_movement_state());
            h = bench_hash_u64(h, c->movement_state);
            h = bench_hash_u64(h, o->is_collision_detection_enabled());
            for (int j = 0; j < 3; ++j) h = bench_hash_double(h, c->pos_world_f_core_last_psi.k[j]);
            h = bench_hash_double(h, c->q_world_f_core_last_psi.x);
            h = bench_hash_double(h, c->q_world_f_core_last_psi.y);
            h = bench_hash_double(h, c->q_world_f_core_last_psi.z);
            h = bench_hash_double(h, c->q_world_f_core_last_psi.w);
            for (int j = 0; j < 3; ++j) h = bench_hash_double(h, (double)c->speed.k[j]);
            for (int j = 0; j < 3; ++j) h = bench_hash_double(h, (double)c->rot_speed.k[j]);
        }
        r.objects = h;
        r.constraints = BENCH_HASH_INIT;
        r.events = events;
        r.event_count = event_count;
        IVP_Statistic_Manager *s = env->get_statistic_manager();
        h = bench_hash_u64(BENCH_HASH_INIT, s->mindists_generated);
        h = bench_hash_u64(h, s->mindists_deleted);
        h = bench_hash_u64(h, s->range_intra_exceeded);
        h = bench_hash_u64(h, s->range_world_exceeded);
        r.statistics = bench_hash_u64(h, s->impact_counter);
        trajectory = bench_hash_step(trajectory, &r);
        r.trajectory = trajectory;
        trace.push_back(r);
    }
};

static void run_world(const char *name, int scale, int height, bool verify,
                      long long &simulate_ns, long long &total_ns)
{
    bool contacts = !std::strcmp(name, "contact_pile");
    int steps = contacts ? 360 : 180;
    const double dt = 1.0 / 180.0;
    long long total_start = now_ns();
    std::vector<bench_step_t> trace;
    {
        Scene world(contacts);
        if (contacts) {
            world.ground(bench_contact_ground_extent(scale, height));
            for (int i = 0; i < scale; ++i) {
                double p[3];
                bench_contact_position(i, scale, height, p);
                world.ball(0.45, p[0], p[1], p[2]);
            }
        } else {
            for (int i = 0; i < scale; ++i) {
                double y = (i % 16) * 2.0, z = (i / 16) * 2.0;
                IVP_Real_Object *a = world.ball(0.4, -4.0, y, z);
                IVP_Real_Object *b = world.ball(0.4, 4.0, y, z);
                a->get_core()->speed.set(80.0f, 0.0f, 0.0f);
                b->get_core()->speed.set(-80.0f, 0.0f, 0.0f);
                a->get_core()->delta_world_f_core_psis = a->get_core()->speed;
                b->get_core()->delta_world_f_core_psis = b->get_core()->speed;
            }
        }
        if (verify) {
            world.env->add_listener_collision_global(&world);
            world.observing = true;
            world.snapshot();
#ifdef BENCH_WORK_DIAGNOSTICS
            bench_work_reset();
            bench_work_register(world.env->get_time_manager()->min_hash, 2);
            for (IVP_Real_Object *o : world.objects)
                bench_work_register(o->get_hull_manager()->get_sorted_synapses(), 1);
            bench_work_start();
            bench_work_step();
#endif
        }
        long long start = now_ns();
        if (verify) {
            for (int i = 0; i < steps; ++i) {
                world.env->simulate_until(world.env->get_current_time() + dt);
                world.snapshot();
#ifdef BENCH_WORK_DIAGNOSTICS
                bench_work_step();
#endif
            }
#ifdef BENCH_WORK_DIAGNOSTICS
            bench_work_finish();
#endif
        } else {
            for (int i = 0; i < steps; ++i)
                world.env->simulate_until(world.env->get_current_time() + dt);
        }
        simulate_ns += now_ns() - start;
        if (verify) trace.swap(world.trace);
    }
    total_ns += now_ns() - total_start;
    if (verify) {
        for (size_t i = 0; i < trace.size(); ++i)
            bench_print_step(stdout, "reference", name, scale, (int)i, &trace[i]);
        std::printf("{\"schema\":\"libivp-benchmark-v2\",\"kind\":\"native-verification\","
                    "\"trajectory_schema\":\"%s\",\"implementation\":\"reference\","
                    "\"case\":\"%s\",\"scale\":%d,\"steps_per_world\":%d,\"dt\":%.17g,"
                    "\"trajectory_hash\":\"%016llx\",\"contact_height\":%d}\n",
                    BENCH_TRACE_SCHEMA, name, scale, steps, dt,
                    (unsigned long long)trace.back().trajectory, height);
    }
}

static bool positive_int(const char *text, int &out)
{
    char *end = nullptr;
    errno = 0;
    long value = std::strtol(text, &end, 10);
    if (errno || end == text || *end || value < 1 || value > 1000000) return false;
    out = (int)value;
    return true;
}

static bool positive_double(const char *text, double &out)
{
    char *end = nullptr;
    errno = 0;
    double value = std::strtod(text, &end);
    if (errno || end == text || *end || !std::isfinite(value) || value <= 0) return false;
    out = value;
    return true;
}

int main(int argc, char **argv)
{
    const char *name = nullptr;
    int scale = 0, samples = 5, height = 0;
    double min_ms = 200.0;
    bool verify = false;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--verify")) { verify = true; continue; }
        if (i + 1 == argc) return 2;
        const char *key = argv[i], *value = argv[++i];
        if (!std::strcmp(key, "--case")) name = value;
        else if (!std::strcmp(key, "--scale")) { if (!positive_int(value, scale)) return 2; }
        else if (!std::strcmp(key, "--samples")) { if (!positive_int(value, samples)) return 2; }
        else if (!std::strcmp(key, "--contact-height")) { if (!positive_int(value, height)) return 2; }
        else if (!std::strcmp(key, "--min-sample-ms")) { if (!positive_double(value, min_ms)) return 2; }
        else return 2;
    }
    if (!name || (std::strcmp(name, "fast_impacts") && std::strcmp(name, "contact_pile")) ||
        scale < 1 || scale > 16384 || samples < 1 || samples > 64 ||
        !std::isfinite(min_ms) || min_ms <= 0 || height < 0 || height > scale ||
        (height && std::strcmp(name, "contact_pile"))) return 2;
    long long warmup_sim = 0, warmup_total = 0;
    run_world(name, scale, height, verify, warmup_sim, warmup_total);
    if (verify) return 0;
    for (int sample = 0; sample < samples; ++sample) {
        long long simulate_ns = 0, total_ns = 0;
        int worlds = 0, steps = !std::strcmp(name, "contact_pile") ? 360 : 180;
        do {
            run_world(name, scale, height, false, simulate_ns, total_ns);
            ++worlds;
        } while (simulate_ns < min_ms * 1e6);
        std::printf("{\"schema\":\"libivp-benchmark-v2\",\"kind\":\"native\","
                    "\"implementation\":\"reference\",\"case\":\"%s\",\"scale\":%d,"
                    "\"sample\":%d,\"iterations\":%d,\"operations\":%d,\"psis\":%d,"
                    "\"operation_ns\":%lld,\"total_ns\":%lld,\"timing_method\":\"native-noop-v1\"}\n",
                    name, scale, sample, worlds, worlds * steps, worlds * steps, simulate_ns, total_ns);
    }
}
