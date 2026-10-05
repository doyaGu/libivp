/* Verification-only observers. Never installed in timing or allocation runs. */
#ifndef BENCH_VERIFY_H
#define BENCH_VERIFY_H
#include "bench_trace.h"

typedef struct bench_verification {
    ivp_environment_t *env;
    ivp_collision_listener_t listener;
    const ivp_constraint_state_t *constraints;
    int constraint_count;
    int running;
    int count;
    uint64_t events;
    uint64_t event_count;
    uint64_t trajectory;
    bench_step_t steps[MAX_STEPS + 1];
} bench_verification_t;

static uint64_t verify_object_id(const bench_verification_t *v, const ivp_object_t *obj)
{
    for (int i = 0; i < ivp_environment_get_object_count(v->env); ++i)
        if (ivp_environment_get_object(v->env, i) == obj) return (uint64_t)i + 1;
    return 0;
}

static void verify_event(bench_verification_t *v, int kind,
                         const ivp_object_t *a, const ivp_object_t *b)
{
    if (!v->running) return;
    v->events = bench_hash_u64(v->events, (uint64_t)kind);
    v->events = bench_hash_double(v->events, ivp_environment_get_time(v->env));
    v->events = bench_hash_u64(v->events, verify_object_id(v, a));
    v->events = bench_hash_u64(v->events, verify_object_id(v, b));
    ++v->event_count;
}

static void verify_pre(void *ctx, const ivp_collision_event_t *e)
{ verify_event((bench_verification_t *)ctx, 1, e->obj_a, e->obj_b); }
static void verify_post(void *ctx, const ivp_collision_event_t *e)
{ verify_event((bench_verification_t *)ctx, 2, e->obj_a, e->obj_b); }
static void verify_contact_add(void *ctx, const ivp_friction_event_t *e)
{ verify_event((bench_verification_t *)ctx, 3, e->obj_a, e->obj_b); }
static void verify_contact_remove(void *ctx, const ivp_friction_event_t *e)
{ verify_event((bench_verification_t *)ctx, 4, e->obj_a, e->obj_b); }

static uint64_t verify_constraints(const bench_verification_t *v)
{
    uint64_t h = BENCH_HASH_INIT;
    for (int i = 0; i < v->constraint_count; ++i) {
        const ivp_constraint_state_t *s = &v->constraints[i];
        const ivp_constraint_local_core_t *c = &s->core;
        h = bench_hash_u64(h, verify_object_id(v, s->obj_a));
        h = bench_hash_u64(h, verify_object_id(v, s->obj_b));
        h = bench_hash_u64(h, s->active);
        h = bench_hash_u64(h, s->is_broken);
        h = bench_hash_u64(h, s->core_ready);
        h = bench_hash_double(h, (double)s->accumulated_impulse);
        h = bench_hash_u64(h, c->enabled);
        h = bench_hash_u64(h, c->broken);
        h = bench_hash_u64(h, c->has_rot);
        h = bench_hash_u64(h, c->use_maxforce);
        h = bench_hash_u64(h, (uint64_t)c->fixedtrans_dim);
        h = bench_hash_u64(h, (uint64_t)c->fixedrot_dim);
        h = bench_hash_u64(h, (uint64_t)c->limitedtrans_dim);
        h = bench_hash_u64(h, (uint64_t)c->limitedrot_dim);
        h = bench_hash_u64(h, (uint64_t)c->matrix_size);
        h = bench_hash_u64(h, (uint64_t)c->norm);
        h = bench_hash_double(h, (double)c->force_factor);
        h = bench_hash_double(h, (double)c->damp_factor_div_force);
        h = bench_hash_double(h, (double)c->limited_axis_stiffness);
        for (int j = 0; j < 6; ++j) {
            h = bench_hash_u64(h, (uint64_t)c->fixed[j]);
            h = bench_hash_u64(h, (uint64_t)c->maxforce_type[j]);
            h = bench_hash_double(h, (double)c->borderleft_Rfs[j]);
            h = bench_hash_double(h, (double)c->borderright_Rfs[j]);
            h = bench_hash_double(h, (double)c->halfimpulse[j]);
            h = bench_hash_double(h, (double)c->last_impulse[j]);
        }
        for (int row = 0; row < 3; ++row) {
            h = bench_hash_u64(h, (uint64_t)c->mapping_uRfs_f_Rfs[row]);
            h = bench_hash_u64(h, (uint64_t)c->mapping_uRrs_f_Rrs[row]);
            h = bench_hash_double(h, c->m_Rfs_f_Rcs.vv[row]);
            h = bench_hash_double(h, c->m_Afs_f_Acs.vv[row]);
            for (int col = 0; col < 3; ++col) {
                h = bench_hash_double(h, c->m_Rfs_f_Rcs.rows[row][col]);
                h = bench_hash_double(h, c->m_Afs_f_Acs.rows[row][col]);
                h = bench_hash_double(h, c->m_Rrs_f_Rcs[row][col]);
                h = bench_hash_double(h, c->m_Ars_f_Acs[row][col]);
            }
        }
    }
    return h;
}

static void verify_step(bench_verification_t *v)
{
    bench_step_t *r = &v->steps[v->count++];
    r->objects = BENCH_HASH_INIT;
    r->object_count = ivp_environment_get_object_count(v->env);
    for (int i = 0; i < r->object_count; ++i) {
        const ivp_object_t *o = ivp_environment_get_object(v->env, i);
        const ivp_core_t *c = o->physical_core;
        uint64_t h = r->objects;
        h = bench_hash_u64(h, o->type == IVP_OBJECT_BALL ? 1 : 2);
        h = bench_hash_u64(h, (uint64_t)o->object_movement_state);
        h = bench_hash_u64(h, (uint64_t)c->movement_state);
        h = bench_hash_u64(h, o->collision_detection_enabled);
        for (int j = 0; j < 3; ++j) h = bench_hash_double(h, (double)c->pos_world_f_core_last_psi.k[j]);
        h = bench_hash_double(h, (double)c->q_world_f_core_last_psi.x);
        h = bench_hash_double(h, (double)c->q_world_f_core_last_psi.y);
        h = bench_hash_double(h, (double)c->q_world_f_core_last_psi.z);
        h = bench_hash_double(h, (double)c->q_world_f_core_last_psi.w);
        for (int j = 0; j < 3; ++j) h = bench_hash_double(h, (double)c->speed.k[j]);
        for (int j = 0; j < 3; ++j) h = bench_hash_double(h, (double)c->rot_speed.k[j]);
        r->objects = h;
    }
    r->constraints = verify_constraints(v);
    r->events = v->events;
    r->event_count = v->event_count;
    const ivp_statistic_manager_t *s = ivp_environment_get_statistic_manager(v->env);
    uint64_t h = BENCH_HASH_INIT;
    h = bench_hash_u64(h, (uint64_t)s->mindists_generated);
    h = bench_hash_u64(h, (uint64_t)s->mindists_deleted);
    h = bench_hash_u64(h, (uint64_t)s->range_intra_exceeded);
    h = bench_hash_u64(h, (uint64_t)s->range_world_exceeded);
    r->statistics = bench_hash_u64(h, (uint64_t)s->impact_counter);
    v->trajectory = bench_hash_step(v->trajectory, r);
    r->trajectory = v->trajectory;
}

static void verify_begin(bench_verification_t *v, ivp_environment_t *env)
{
    if (!v->env) {
        v->env = env;
        v->trajectory = v->events = BENCH_HASH_INIT;
        ivp_collision_listener_init(&v->listener, IVP_COLLISION_CB_PRE_COLLISION |
                IVP_COLLISION_CB_POST_COLLISION | IVP_COLLISION_CB_FRICTION_CREATED |
                IVP_COLLISION_CB_FRICTION_DELETED, v);
        v->listener.pre_collision_fn = verify_pre;
        v->listener.post_collision_fn = verify_post;
        v->listener.friction_created_fn = verify_contact_add;
        v->listener.friction_deleted_fn = verify_contact_remove;
        ivp_environment_add_collision_listener_global(env, &v->listener);
        verify_step(v);
    }
    v->running = 1;
}
#endif
