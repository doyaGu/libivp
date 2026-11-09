/* ivp_controller_buoyancy.c -- Buoyancy controller
 *
 * Port of IVP_Controller_Buoyancy (ivp_controller/ivp_controller_buoyancy.cxx):
 * do_simulation_controller, use_buoyancy_solver, apply_buoyancy_impulse,
 * apply_dampening and the multidimensional interpolation path
 * (calculate_future_extrapolation, provide_new_input_solution_combination),
 * with the reference IVP_FLOAT / IVP_U_Float_Point member and value types.
 *
 * The reference engine defines IVP_NO_MD_INTERPOLATION in ivp_physics.hxx:
 * its buoyancy controller ignores IVP_Template_Buoyancy::use_interpolation
 * and always runs the solver.  libivp does the same unless compiled with
 * -DIVP_MD_INTERPOLATION, which enables the (ported) interpolation path.
 */

#include <ivp/controllers/ivp_controller_buoyancy.h>
#include <ivp/controllers/ivp_controller_util.h>
#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_environment.h>
#include <ivp/geometry/ivp_surman.h>
#include <ivp/utility/ivp_multidimensional_interp.h>

#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "ivp_buoyancy_solver.h"
#include "controllers/ivp_controller_ref.h"

/* reference ivp_physics.hxx */
#if !defined(IVP_MD_INTERPOLATION) && !defined(IVP_NO_MD_INTERPOLATION)
#define IVP_NO_MD_INTERPOLATION
#endif


static void buoyancy_defaults(ivp_buoyancy_state_t *state)
{
    memset(state, 0, sizeof(*state));
    /* Reference IVP_Template_Buoyancy (ivp_controller_buoyancy.hxx:83-105) */
    state->simulate_wing_behavior = false;
    state->linear_drag = IVP_REAL(0.1);           /* pressure_damp_factor */
    state->friction_damp_factor = IVP_REAL(0.05);
    state->torque_factor = IVP_REAL(1.0);
    state->fluid_density = IVP_REAL(0.3);
    state->buoyancy_eps = (ivp_real_t)1e-10;
    state->viscosity_factor = IVP_REAL(0.01);
    state->viscosity_input_factor = IVP_REAL(2.0);
    state->angular_drag = IVP_REAL(0.01);         /* ball_rot_dampening_factor */
    state->use_interpolation = false;
    state->max_interpolation_tries = 10;
    state->max_tries_nr_of_vectors_involved = 15;
    state->use_stochastic_insertion = true;
    state->insert_extrapol_only = true;
    state->mi_weight_current_speed = IVP_REAL(1.0);
    state->mi_weight_surface = IVP_REAL(1.0);
    state->mi_weight_rot_speed = IVP_REAL(1.0);
    state->max_res = IVP_REAL(1e-2);
    state->nr_future_psi_for_extrapolation = 1;
    state->active = true;
}

static const ivp_allocator_t *buoyancy_allocator(const ivp_buoyancy_state_t *state)
{
    return state->allocator_bound ? &state->allocator : ivp_default_allocator();
}

static const ivp_allocator_t *core_data_allocator(const ivp_buoyancy_core_data_t *cd)
{
    return cd->allocator ? cd->allocator : ivp_default_allocator();
}

void ivp_buoyancy_init(ivp_buoyancy_state_t *state,
                       ivp_real_t surface_y,
                       ivp_real_t fluid_density)
{
    buoyancy_defaults(state);
    ivp_vec3r_set(&state->surface_normal, IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    state->surface_height = -surface_y;
    state->fluid_density = fluid_density;
}

void ivp_buoyancy_init_plane(ivp_buoyancy_state_t *state,
                             const ivp_vec3r_t *normal,
                             ivp_real_t height,
                             ivp_real_t fluid_density)
{
    buoyancy_defaults(state);
    state->surface_normal = *normal;
    if (ivp_vec3r_normalize(&state->surface_normal) != IVP_OK) {
        ivp_vec3r_set(&state->surface_normal, IVP_REAL(0.0), IVP_REAL(-1.0), IVP_REAL(0.0));
    }
    state->surface_height = height;
    state->fluid_density = fluid_density;
}

void ivp_buoyancy_set_liquid_surface(ivp_buoyancy_state_t *state,
                                     const ivp_liquid_surface_t *surface)
{
    state->liquid_surface = surface;
}

void ivp_buoyancy_set_objects(ivp_buoyancy_state_t *state,
                              ivp_object_t *const *objects, int count)
{
    state->objects = objects;
    state->object_count = objects ? count : 0;
}

/* IVP_Controller_Buoyancy::clear_attacher_interpolator */
static void clear_attacher_interpolator(ivp_buoyancy_core_data_t *cd)
{
    const ivp_allocator_t *allocator = core_data_allocator(cd);
    for (int i = 0; i < cd->interpolator_count; i++) {
        ivp_buoyancy_object_interpolator_t *ai = &cd->interpolators[i];
        if (ai->mi) {
            ivp_mi_destroy(ai->mi);
            ivp_free(allocator, ai->mi, sizeof(ivp_mi_interpolator_t));
            ai->mi = NULL;
        }
        if (ai->last_input) ivp_mi_vector_free(ai->last_input, allocator);
        if (ai->last_solution) ivp_mi_vector_free(ai->last_solution, allocator);
        ai->last_input = ai->last_solution = NULL;
    }
    if (cd->interpolators) {
        ivp_free(allocator, cd->interpolators,
                 (size_t)cd->interpolator_count * sizeof(ivp_buoyancy_object_interpolator_t));
    }
    cd->interpolators = NULL;
    cd->interpolator_count = 0;
}

/* IVP_Controller_Buoyancy::rebuild_attacher_interpolator: one (empty)
 * interpolator per object of the core */
static void rebuild_attacher_interpolator(ivp_buoyancy_core_data_t *cd, const ivp_core_t *core)
{
    clear_attacher_interpolator(cd);
    int n = ivp_pvec_len(&core->objects);
    if (n <= 0) return;
    cd->interpolators = (ivp_buoyancy_object_interpolator_t *)ivp_alloc(
        core_data_allocator(cd), (size_t)n * sizeof(ivp_buoyancy_object_interpolator_t));
    if (!cd->interpolators) return;
    memset(cd->interpolators, 0, (size_t)n * sizeof(ivp_buoyancy_object_interpolator_t));
    cd->interpolator_count = n;
    for (int i = 0; i < n; i++) cd->interpolators[i].object = (const ivp_object_t *)ivp_pvec_at(&core->objects, i);
}

/* IVP_Controller_Buoyancy::sync_attacher_interpolator_with_core_objects */
static void sync_attacher_interpolator_with_core_objects(ivp_buoyancy_core_data_t *cd, const ivp_core_t *core)
{
    int nr_of_objects = ivp_pvec_len(&core->objects);
    if (nr_of_objects != cd->interpolator_count) {
        rebuild_attacher_interpolator(cd, core);
        return;
    }
    for (int i = 0; i < cd->interpolator_count; i++) {
        if (cd->interpolators[i].object != ivp_pvec_at(&core->objects, i)) {
            rebuild_attacher_interpolator(cd, core);
            return;
        }
    }
}

/* ~IVP_Controller_Buoyancy */
static void core_data_release(ivp_buoyancy_core_data_t *cd)
{
    clear_attacher_interpolator(cd);
}

/* IVP_Controller_Buoyancy constructor: fresh per-core state */
static void core_data_init(ivp_buoyancy_core_data_t *cd, const ivp_object_t *obj,
                           const ivp_allocator_t *allocator)
{
    memset(cd, 0, sizeof(*cd));
    cd->allocator = allocator ? allocator : ivp_default_allocator();
    cd->object = obj;
    cd->object_handle = ivp_environment_get_object_handle(obj);
}

static ivp_buoyancy_core_data_t *core_data_at(ivp_buoyancy_state_t *state, int i)
{
    return (i < IVP_BUOYANCY_INLINE_CORES) ? &state->cores[i]
                                           : &state->cores_overflow[i - IVP_BUOYANCY_INLINE_CORES];
}

void ivp_buoyancy_state_destroy(ivp_buoyancy_state_t *state)
{
    if (!state) return;
    for (int i = 0; i < state->core_count; i++) core_data_release(core_data_at(state, i));
    if (state->cores_overflow) {
        ivp_free(buoyancy_allocator(state), state->cores_overflow,
                 (size_t)state->cores_overflow_capacity * sizeof(ivp_buoyancy_core_data_t));
    }
    state->cores_overflow = NULL;
    state->cores_overflow_capacity = 0;
    state->core_count = 0;
    memset(&state->allocator, 0, sizeof(state->allocator));
    state->allocator_bound = false;
}

static bool same_handle(ivp_object_handle_t a, ivp_object_handle_t b)
{
    return a.slot == b.slot && a.generation == b.generation;
}

/* An entry whose object was deleted (no longer reachable by its handle). */
static bool core_data_is_stale(const ivp_buoyancy_core_data_t *cd, const ivp_environment_t *env)
{
    if (!env) return false;
    return ivp_environment_get_object_by_handle(env, cd->object_handle) != cd->object;
}

/* Per-core data of the all-cores registration (one entry per core, as one
 * IVP_Controller_Buoyancy per core in the reference; no core limit). */
static ivp_buoyancy_core_data_t *core_data(ivp_buoyancy_state_t *state, const ivp_object_t *obj,
                                           const ivp_environment_t *env)
{
    const ivp_object_handle_t h = ivp_environment_get_object_handle(obj);
    for (int i = 0; i < state->core_count; i++) {
        ivp_buoyancy_core_data_t *cd = core_data_at(state, i);
        if (cd->object != obj) continue;
        if (!same_handle(cd->object_handle, h)) {
            /* a new object at the address of a deleted one: new controller */
            core_data_release(cd);
            core_data_init(cd, obj, buoyancy_allocator(state));
        }
        return cd;
    }
    /* reuse the entry of a deleted object */
    for (int i = 0; i < state->core_count; i++) {
        ivp_buoyancy_core_data_t *cd = core_data_at(state, i);
        if (!core_data_is_stale(cd, env)) continue;
        core_data_release(cd);
        core_data_init(cd, obj, buoyancy_allocator(state));
        return cd;
    }
    int idx = state->core_count;
    if (idx >= IVP_BUOYANCY_INLINE_CORES) {
        int o = idx - IVP_BUOYANCY_INLINE_CORES;
        if (o >= state->cores_overflow_capacity) {
            int nc = state->cores_overflow_capacity ? state->cores_overflow_capacity * 2 : IVP_BUOYANCY_INLINE_CORES;
            ivp_buoyancy_core_data_t *n = (ivp_buoyancy_core_data_t *)ivp_realloc(
                buoyancy_allocator(state), state->cores_overflow,
                (size_t)state->cores_overflow_capacity * sizeof(ivp_buoyancy_core_data_t),
                (size_t)nc * sizeof(ivp_buoyancy_core_data_t));
            if (!n) return NULL;
            state->cores_overflow = n;
            state->cores_overflow_capacity = nc;
        }
    }
    state->core_count++;
    ivp_buoyancy_core_data_t *cd = core_data_at(state, idx);
    core_data_init(cd, obj, buoyancy_allocator(state));
    return cd;
}

static bool object_in_set(const ivp_buoyancy_state_t *state, const ivp_object_t *obj)
{
    if (!state->objects) return true;
    for (int i = 0; i < state->object_count; i++) {
        if (state->objects[i] == obj) return true;
    }
    return false;
}

/* liquid surface as reference IVP_U_Float_Hesse (n.p + hesse_val > 0 inside
 * the liquid) and IVP_U_Float_Point abs_speed_of_current_ws
 * (IVP_Liquid_Surface_Descriptor::calc_liquid_surface) */
static void calc_liquid_surface(const ivp_buoyancy_state_t *state, const ivp_core_t *core,
                                ivp_bs_hesse_t *surface_ws, ivp_vec3f_t *abs_speed_ws)
{
    ivp_vec3r_t normal = state->surface_normal, vel = state->flow_velocity;
    ivp_real_t height = state->surface_height;
    if (state->liquid_surface && state->liquid_surface->eval_fn) {
        /* the descriptor defines the plane even outside of its domain
         * (reference has no domain test) */
        (void)state->liquid_surface->eval_fn(state->liquid_surface->ctx,
                                             &core->m_world_f_core_last_psi.pos,
                                             &normal, &height, &vel);
    }
    surface_ws->n.k[0] = (float)-normal.k[0];
    surface_ws->n.k[1] = (float)-normal.k[1];
    surface_ws->n.k[2] = (float)-normal.k[2];
    surface_ws->hesse_val = (float)height;
    lvf_set_p(abs_speed_ws, &vel);
}

/* ── debug helpers (reference debug build) ─────────────────────────────── */

/* IVP_U_Float_Point::real_length: sqrtd of the float quad_length */
static double buoy_float_real_length(const ivp_vec3f_t *v)
{
    return sqrt((double)(v->k[0] * v->k[0] + v->k[1] * v->k[1] + v->k[2] * v->k[2]));
}

/* IVP_Core::core_plausible_check (ivp_core.cxx, MAX_PLAUSIBLE_LEN
 * 100000.0f); rot_speed_change is passed in because apply_dampening
 * accumulates it in a local before storing it in the core. */
#define BUOY_MAX_PLAUSIBLE_LEN 100000.0f
static void buoy_core_plausible_check_rsc(const ivp_core_t *core, const ivp_vec3f_t *rot_speed_change)
{
    const ivp_vec3f_t speed_change = ivp_ctl_f(&core->speed_change);
    const ivp_vec3f_t speed = ivp_ctl_f(&core->speed);
    double rot_change_len, trans_change_len;
    rot_change_len = buoy_float_real_length(rot_speed_change);
    trans_change_len = buoy_float_real_length(&speed_change);
    IVP_ASSERT(rot_change_len < (double)BUOY_MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(trans_change_len < (double)BUOY_MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(rot_change_len > (double)-BUOY_MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(trans_change_len > (double)-BUOY_MAX_PLAUSIBLE_LEN);

    double rot_len, trans_len;
    rot_len = buoy_float_real_length(&core->rot_speed);
    trans_len = buoy_float_real_length(&speed);
    IVP_ASSERT(rot_len < (double)BUOY_MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(trans_len < (double)BUOY_MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(rot_len > (double)-BUOY_MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(trans_len > (double)-BUOY_MAX_PLAUSIBLE_LEN);
    IVP_USE(rot_change_len);
    IVP_USE(trans_change_len);
    IVP_USE(rot_len);
    IVP_USE(trans_len);
}

static void buoy_core_plausible_check(const ivp_core_t *core)
{
    const ivp_vec3f_t rot_speed_change = ivp_ctl_f(&core->rot_speed_change);
    buoy_core_plausible_check_rsc(core, &rot_speed_change);
}

/* IVP_U_Float_Point::print / IVP_U_Point::print */
static void buoy_print_float_point(const ivp_vec3f_t *p, const char *comment)
{
    if (comment) {
        ivp_debug_printf("%s Point %f %f %f\n", comment, (double)p->k[0], (double)p->k[1], (double)p->k[2]);
    } else {
        ivp_debug_printf("Point %f %f %f\n", (double)p->k[0], (double)p->k[1], (double)p->k[2]);
    }
}

static void buoy_print_point(const ivp_vec3r_t *p, const char *comment)
{
    if (comment) {
        ivp_debug_printf("%s Point %f %f %f\n", comment, (double)p->k[0], (double)p->k[1], (double)p->k[2]);
    } else {
        ivp_debug_printf("Point %f %f %f\n", (double)p->k[0], (double)p->k[1], (double)p->k[2]);
    }
}

/* reference IVP_Controller_Buoyancy::apply_buoyancy_impulse
 * (ivp_controller_buoyancy.cxx:213-253) */
static void apply_buoyancy_impulse(const ivp_buoyancy_state_t *state, ivp_environment_t *env,
                                   ivp_object_t *object, ivp_core_t *core, D delta_time,
                                   float volume_under, const ivp_vec3f_t *volume_center_under)
{
    /* the buoyancy impulse works opposite to the gravity */
    ivp_vec3f_t dir_ws = ivp_ctl_f(&env->gravity);
    if (!(volume_under > (float)state->buoyancy_eps)) return;

    /* transform_position_to_world_coords (cache object) */
    ivp_transform_r_t tmp;
    ivp_vec3r_t volume_center_under_ws;
    lvm_vmult4_fp(ivp_ctl_cache_m_world_f_object(object, &tmp), volume_center_under, &volume_center_under_ws);

    D force_under = volume_under * (-(float)state->fluid_density) * (delta_time);
    ivp_vec3f_t impulse_ws;
    lvf_set_multiple(&impulse_ws, &dir_ws, force_under);

    IVP_IF(0) { /* ivp_controller_buoyancy.cxx:238 */
        buoy_print_float_point(&impulse_ws, "buoyancy-impulse");
        buoy_print_point(&volume_center_under_ws, "volume_center_under_ws");
        buoy_print_float_point(volume_center_under, "volume_center_under_os");
        ivp_debug_printf("volume_under = %f\n\n", (double)volume_under);
    }

    ivp_ctl_async_push_core_ws(core, &volume_center_under_ws, &impulse_ws);

    IVP_IF(1) { /* ivp_controller_buoyancy.cxx:248 */
        buoy_core_plausible_check(core);
    }
}

/* reference IVP_Controller_Buoyancy::apply_dampening
 * (ivp_controller_buoyancy.cxx:106-207, DAMPENING_WITH_MOVEVECTOR). */
static void apply_dampening(const ivp_buoyancy_state_t *state, ivp_object_t *object, ivp_core_t *core,
                            float object_visible_surface_content_under, D delta_time,
                            ivp_vec3f_t *sum_impulse_x_movevector, ivp_vec3f_t *sum_impulse_x_point,
                            ivp_vec3f_t *sum_impulse)
{
    /* torque_factor * sqrt(visible content), turns the move vector around */
    float move_factor = (float)(-(float)state->torque_factor *
                                sqrt((double)object_visible_surface_content_under) * (0.1f));
    lvf_mult(sum_impulse_x_movevector, move_factor * delta_time);

    lvf_mult(sum_impulse_x_point, delta_time);
    lvf_mult(sum_impulse, delta_time);

    /* convert sum_impulse into world coordinates (cache object matrix) */
    ivp_transform_r_t tmp;
    ivp_vec3f_t sum_impulse_ws;
    lvm_vmult3_f(ivp_ctl_cache_m_world_f_object(object, &tmp), sum_impulse, &sum_impulse_ws);

    ivp_transform_r_t m_core_f_object;
    ivp_ctl_calc_m_core_f_object(object, &m_core_f_object);

    /* push the object center with the sum of impulses */
    ivp_transform_r_t m_world_f_object;
    ivp_ctl_m_world_f_object_AT(object, &m_world_f_object);
    ivp_ctl_async_push_core_ws(core, &m_world_f_object.pos, &sum_impulse_ws);
    IVP_IF(0) { /* ivp_controller_buoyancy.cxx:150 */
        const ivp_vec3f_t speed_change = ivp_ctl_f(&core->speed_change);
        ivp_debug_printf("core->speed_change.real_length = %f\n", buoy_float_real_length(&speed_change));
        IVP_ASSERT(buoy_float_real_length(&speed_change) < (double)70000.0f);
        buoy_core_plausible_check(core);
    }

    /* rotation related values in core space */
    ivp_vec3f_t sum_impulse_x_point_cs, sum_impulse_x_movevector_cs;
    lvm_vmult3_f(&m_core_f_object, sum_impulse_x_point, &sum_impulse_x_point_cs);
    lvm_vmult3_f(&m_core_f_object, sum_impulse_x_movevector, &sum_impulse_x_movevector_cs);

    const ivp_vec3f_t iri = ivp_core_inv_rot_inertia_f(core);
    ivp_vec3f_t delta_rot_component1_cs, delta_rot_component2_cs;
    lvf_set_pairwise_mult(&delta_rot_component1_cs, &sum_impulse_x_point_cs, &iri);
    lvf_set_pairwise_mult(&delta_rot_component2_cs, &sum_impulse_x_movevector_cs, &iri);
    lvf_add(&delta_rot_component1_cs, &delta_rot_component2_cs);

    ivp_vec3f_t rot_speed_change = ivp_ctl_f(&core->rot_speed_change);
    lvf_add(&rot_speed_change, &delta_rot_component1_cs);
    IVP_IF(1) { /* ivp_controller_buoyancy.cxx:177 */
        buoy_core_plausible_check_rsc(core, &rot_speed_change);
    }

    /* balls: dampening of the rotation */
    if (object->type == IVP_OBJECT_BALL) {
        float radius = object->extra_radius;   /* IVP_Ball::get_radius */
        ivp_vec3f_t rotation_dampening;
        lvf_set_multiple(&rotation_dampening, &core->rot_speed,
                         (float)state->angular_drag * object_visible_surface_content_under /
                             (2 * IVP_PI * radius * radius));
        lvf_subtract(&rot_speed_change, &rotation_dampening);
        IVP_IF(1) { /* ivp_controller_buoyancy.cxx:193 */
            buoy_core_plausible_check_rsc(core, &rot_speed_change);
        }
    }
    core->rot_speed_change = ivp_ctl_r(&rot_speed_change);
}

/* ── IVP_Buoyancy_Input / IVP_Buoyancy_Output as IVP_MI_Vector elements ──
 * IVP_U_Float_Point / IVP_U_Float_Hesse are 4 floats (k[3] + hesse_val,
 * IVP_VECTOR_UNIT_FLOAT); INPUT_VECTOR_LENGTH 12, SOLUTION_VECTOR_LENGTH 18. */
#define BUOY_INPUT_LEN       12
#define BI_SURFACE_OS         0   /* surface_os k[3], hesse_val */
#define BI_REL_SPEED_OS       4   /* rel_speed_of_current_os k[3], hesse_val (0) */
#define BI_ROT_SPEED          8   /* rot_speed k[3], hesse_val (0) */
#define BUOY_SOLUTION_LEN    18
#define BO_VOLUME_CENTER      0   /* volume_center_under */
#define BO_SUM_IMPULSE        4   /* sum_impulse */
#define BO_SUM_X_POINT        8   /* sum_impulse_x_point */
#define BO_SUM_X_MOVEVECTOR  12   /* sum_impulse_x_movevector */
#define BO_VOLUME_UNDER      16   /* volume_under */
#define BO_VISIBLE_CONTENT   17   /* object_visible_surface_content_under */

static inline void bo_get(const float *out, int at, ivp_vec3f_t *v)
{
    v->k[0] = out[at]; v->k[1] = out[at + 1]; v->k[2] = out[at + 2];
}

static inline void bo_set(float *out, int at, const ivp_vec3f_t *v)
{
    out[at] = v->k[0]; out[at + 1] = v->k[1]; out[at + 2] = v->k[2];
}

/* reference IVP_Controller_Buoyancy::use_buoyancy_solver */
static bool use_buoyancy_solver(const ivp_buoyancy_state_t *state, ivp_core_t *core, ivp_object_t *object,
                                const ivp_bs_hesse_t *surface_os, const ivp_vec3f_t *rel_speed_of_current_os,
                                float *solution_values_out, const ivp_vec3f_t *resulting_speed_of_current_ws)
{
    ivp_buoyancy_solver_t bs;
    ivp_buoyancy_solver_init(&bs, core, state, resulting_speed_of_current_ws);
    ivp_vec3f_t rel_speed_of_current_os_aligned = *rel_speed_of_current_os;
    bool in_water = ivp_buoyancy_solver_compute_forces(&bs, &rel_speed_of_current_os_aligned, surface_os, object);
    if (in_water) {
        solution_values_out[BO_VOLUME_UNDER] = bs.volume_under;
        bo_set(solution_values_out, BO_VOLUME_CENTER, &bs.volume_center_under);
        solution_values_out[BO_VISIBLE_CONTENT] = bs.object_visible_surface_content_under;
        bo_set(solution_values_out, BO_SUM_IMPULSE, &bs.sum_impulse);
        bo_set(solution_values_out, BO_SUM_X_POINT, &bs.sum_impulse_x_point);
        bo_set(solution_values_out, BO_SUM_X_MOVEVECTOR, &bs.sum_impulse_x_movevector);
    }
    return in_water;
}

#if !defined(IVP_NO_MD_INTERPOLATION)
/* the Attacher_Interpolator's interpolator and Last_IO vectors (created on first use) */
static bool interpolator_ensure(ivp_buoyancy_object_interpolator_t *cd,
                                const ivp_allocator_t *allocator)
{
    if (cd->mi && cd->last_input && cd->last_solution) return true;
    if (!cd->mi) {
        cd->mi = (ivp_mi_interpolator_t *)ivp_alloc(allocator, sizeof(ivp_mi_interpolator_t));
        if (!cd->mi) return false;
        if (ivp_mi_init(cd->mi, 5, BUOY_INPUT_LEN, BUOY_SOLUTION_LEN, allocator) != 0) {
            ivp_free(allocator, cd->mi, sizeof(ivp_mi_interpolator_t));
            cd->mi = NULL;
            return false;
        }
    }
    if (!cd->last_input) cd->last_input = ivp_mi_vector_malloc(BUOY_INPUT_LEN, allocator);
    if (!cd->last_solution) cd->last_solution = ivp_mi_vector_malloc(BUOY_SOLUTION_LEN, allocator);
    cd->last_psi_time = 0;
    return cd->last_input && cd->last_solution;
}

/* reference IVP_Controller_Buoyancy::calculate_future_extrapolation */
static bool calculate_future_extrapolation(const ivp_buoyancy_state_t *state, const ivp_buoyancy_object_interpolator_t *cd,
                                           const ivp_mi_vector_t *new_input, const ivp_mi_vector_t *solution_values,
                                           D d_time, double current_time,
                                           ivp_mi_vector_t *future_input, ivp_mi_vector_t *future_solution)
{
    D future_time = (D)(current_time + (double)state->nr_future_psi_for_extrapolation * (double)d_time);
    /* IVP_Time::operator- (retail: no float cast) */
    D divisor = (D)(current_time - cd->last_psi_time);
    if (divisor > P_DOUBLE_RES) {
        float time_quotient = (float)((float)(future_time - cd->last_psi_time) / divisor);

        ivp_mi_vector_set(future_input, new_input);
        ivp_mi_vector_subtract(future_input, cd->last_input);
        ivp_mi_vector_mult(future_input, time_quotient);
        ivp_mi_vector_add(future_input, cd->last_input);

        ivp_mi_vector_set(future_solution, solution_values);
        ivp_mi_vector_subtract(future_solution, cd->last_solution);
        ivp_mi_vector_mult(future_solution, time_quotient);
        ivp_mi_vector_add(future_solution, cd->last_solution);
        return true;
    }

    IVP_IF(1) { /* ivp_controller_buoyancy.cxx:55 */
        ivp_debug_printf("No extrapolation possible!\n");
    }
    return false;
}

/* reference IVP_Controller_Buoyancy::provide_new_input_solution_combination */
static void provide_new_input_solution_combination(const ivp_buoyancy_state_t *state, ivp_buoyancy_object_interpolator_t *cd,
                                                   const ivp_mi_vector_t *weighted_new_input,
                                                   const ivp_mi_vector_t *solution_values,
                                                   D d_time, double current_time)
{
    float future_input_buf[BUOY_INPUT_LEN], future_solution_buf[BUOY_SOLUTION_LEN];
    ivp_mi_vector_t future_input, future_solution;
    ivp_mi_vector_init(&future_input, future_input_buf, BUOY_INPUT_LEN);
    ivp_mi_vector_init(&future_solution, future_solution_buf, BUOY_SOLUTION_LEN);

    /* stochastic insertion only once all slots are occupied */
    const bool stochastic = state->use_stochastic_insertion &&
                            !(ivp_mi_get_nr_occupied(cd->mi) < ivp_mi_get_nr_of_vectors(cd->mi));

    if (!state->insert_extrapol_only) {
        if (stochastic) ivp_mi_add_new_input_solution_combination_stochastic(cd->mi, weighted_new_input, solution_values);
        else ivp_mi_add_new_input_solution_combination_conventional(cd->mi, weighted_new_input, solution_values);
    }

    /* extrapolate into the future */
    if (calculate_future_extrapolation(state, cd, weighted_new_input, solution_values, d_time, current_time,
                                       &future_input, &future_solution)) {
        ivp_mi_vector_set_time_stamp(&future_input, current_time);
        ivp_mi_vector_set_time_stamp(&future_solution, current_time);
        if (stochastic) ivp_mi_add_new_input_solution_combination_stochastic(cd->mi, &future_input, &future_solution);
        else ivp_mi_add_new_input_solution_combination_conventional(cd->mi, &future_input, &future_solution);
    }
}

#endif /* !IVP_NO_MD_INTERPOLATION */

/* ── Controller callback ───────────────────────────────────────────────── */

/* IVP_Controller_Buoyancy::do_simulation_controller for one core: the liquid
 * and current at the core, then buoyancy and dampening of every object of
 * the core (attacher_interpolator, synced with the core's objects). */
static void buoyancy_simulate_core(ivp_buoyancy_state_t *state, ivp_buoyancy_core_data_t *cd,
                                   ivp_core_t *core, const ivp_event_sim_t *event_sim)
{
    ivp_environment_t *environment = event_sim->environment;
    const D delta_time = (D)event_sim->delta_time;

    ivp_bs_hesse_t surface_hesse_ws;
    ivp_vec3f_t abs_speed_of_current_ws;
    calc_liquid_surface(state, core, &surface_hesse_ws, &abs_speed_of_current_ws);

    /* IVP_FLOAT / IVP_U_Float_Point controller members */
    float core_visible_surface_content_under = (float)cd->core_visible_surface_content_under;
    float core_visible_surface_content_under_old = (float)cd->core_visible_surface_content_under_old;
    ivp_vec3f_t relative_speed_of_current_in_objects_vicinity_old;
    lvf_set_p(&relative_speed_of_current_in_objects_vicinity_old,
              &cd->relative_speed_of_current_in_objects_vicinity_old);

    /* if area is increased, reduce mean velocity of water */
    if (core_visible_surface_content_under > core_visible_surface_content_under_old) {
        lvf_mult(&relative_speed_of_current_in_objects_vicinity_old,
                 core_visible_surface_content_under_old / core_visible_surface_content_under);
    }
    core_visible_surface_content_under_old = core_visible_surface_content_under;
    core_visible_surface_content_under = 0.0f;

    /* resulting speed of current (DAMPENING_WITH_PARTICLE_ACCELERATION) */
    ivp_vec3f_t resulting_speed_of_current_ws = abs_speed_of_current_ws;
    lvf_add(&resulting_speed_of_current_ws, &relative_speed_of_current_in_objects_vicinity_old);
    ivp_vec3f_t core_speed = ivp_core_speed_f(core);
    ivp_vec3f_t rel_speed_of_current_ws;
    lvf_subtract2(&rel_speed_of_current_ws, &resulting_speed_of_current_ws, &core_speed);

    sync_attacher_interpolator_with_core_objects(cd, core);

    /* calc buoyancy and dampening for every object in the core */
    const int nr_of_objects = cd->interpolator_count;
    for (int i = 0; i < nr_of_objects; i++) {
        ivp_buoyancy_object_interpolator_t *ai = &cd->interpolators[i];
        ivp_object_t *obj = (ivp_object_t *)(uintptr_t)ai->object;
        ivp_transform_r_t tmp;
        const ivp_transform_r_t *m_cache = ivp_ctl_cache_m_world_f_object(obj, &tmp);
        ivp_vec3f_t rel_speed_of_current_os;
        ivp_bs_hesse_t surface_os;
        lvm_vimult3_f(m_cache, &rel_speed_of_current_ws, &rel_speed_of_current_os);
        lvm_vimult3_f(m_cache, &surface_hesse_ws.n, &surface_os.n);
        ivp_transform_r_t m_world_f_object;
        ivp_ctl_m_world_f_object_AT(obj, &m_world_f_object);
        D dist = lvp_dot_f(&m_world_f_object.pos, &surface_hesse_ws.n);
        surface_os.hesse_val = (float)(surface_hesse_ws.hesse_val + dist);

        /* new input vector with the provided weights */
        float weighted_input_buf[BUOY_INPUT_LEN];
        ivp_mi_vector_t weighted_new_input;
        ivp_mi_vector_init(&weighted_new_input, weighted_input_buf, BUOY_INPUT_LEN);
        {
            ivp_vec3f_t w;
            lvf_set_multiple(&w, &rel_speed_of_current_os, (float)state->mi_weight_current_speed);
            bo_set(weighted_input_buf, BI_REL_SPEED_OS, &w);
            lvf_set_multiple(&w, &surface_os.n, (float)state->mi_weight_surface);
            bo_set(weighted_input_buf, BI_SURFACE_OS, &w);
            weighted_input_buf[BI_SURFACE_OS + 3] = surface_os.hesse_val * (float)state->mi_weight_surface;
            lvf_set_multiple(&w, &core->rot_speed, (float)state->mi_weight_rot_speed);
            bo_set(weighted_input_buf, BI_ROT_SPEED, &w);
        }

        /* new solutions (IVP_Buoyancy_Output: zeroed) */
        float b_output[BUOY_SOLUTION_LEN];
        memset(b_output, 0, sizeof(b_output));
        ivp_mi_vector_t solution_values;
        ivp_mi_vector_init(&solution_values, b_output, BUOY_SOLUTION_LEN);

#if !defined(IVP_NO_MD_INTERPOLATION)
        if (state->use_interpolation && interpolator_ensure(ai, core_data_allocator(cd))) {
            const double current_time = environment ? (double)ivp_environment_get_time(environment) : 0.0;
            bool result;
            /* force a solver run after max_interpolation_tries */
            if (cd->interpolation_counter++ >= state->max_interpolation_tries) {
                result = false;
                cd->interpolation_counter = 0;
            } else {
                result = ivp_mi_check_interpolation(ai->mi, &weighted_new_input,
                                                    state->max_tries_nr_of_vectors_involved,
                                                    (float)state->max_res, &solution_values);
            }
            if (!result) {
                bool in_water = use_buoyancy_solver(state, core, obj, &surface_os, &rel_speed_of_current_os,
                                                    b_output, &resulting_speed_of_current_ws);
                if (in_water) {
                    provide_new_input_solution_combination(state, ai, &weighted_new_input, &solution_values,
                                                           delta_time, current_time);
                }
            }
            /* last input / solution for calculate_future_extrapolation */
            ivp_mi_vector_set(ai->last_input, &weighted_new_input);
            ivp_mi_vector_set(ai->last_solution, &solution_values);
            ai->last_psi_time = current_time;
        } else
#endif
        {
            /* don't use interpolation */
            (void)use_buoyancy_solver(state, core, obj, &surface_os, &rel_speed_of_current_os,
                                      b_output, &resulting_speed_of_current_ws);
        }

        /* apply the impulses to the current object */
        if (environment) {
            ivp_vec3f_t volume_center_under;
            bo_get(b_output, BO_VOLUME_CENTER, &volume_center_under);
            apply_buoyancy_impulse(state, environment, obj, core, delta_time,
                                   b_output[BO_VOLUME_UNDER], &volume_center_under);
        }
        ivp_vec3f_t sum_impulse_x_movevector, sum_impulse_x_point, sum_impulse;
        bo_get(b_output, BO_SUM_X_MOVEVECTOR, &sum_impulse_x_movevector);
        bo_get(b_output, BO_SUM_X_POINT, &sum_impulse_x_point);
        bo_get(b_output, BO_SUM_IMPULSE, &sum_impulse);
        apply_dampening(state, obj, core, (float)fabs((D)b_output[BO_VISIBLE_CONTENT]), delta_time,
                        &sum_impulse_x_movevector, &sum_impulse_x_point, &sum_impulse);

        /* fabs in case the interpolation's result was negative */
        core_visible_surface_content_under =
            (float)(core_visible_surface_content_under + fabs((D)b_output[BO_VISIBLE_CONTENT]));
    }

    /* DAMPENING_WITH_PARTICLE_ACCELERATION */
    float inverse_viscosity = (float)(1.0f - ((float)state->viscosity_factor * delta_time));
    if (inverse_viscosity < 0.0f) {
        inverse_viscosity = 0.0f;
    } else if (inverse_viscosity > 1.0f) {
        inverse_viscosity = 1.0f;
    }
    ivp_vec3f_t speed_of_object_relative_to_medium_ws = ivp_core_speed_f(core);
    lvf_subtract(&speed_of_object_relative_to_medium_ws, &resulting_speed_of_current_ws);

    lvf_mult(&relative_speed_of_current_in_objects_vicinity_old, inverse_viscosity);
    lvf_add_multiple(&relative_speed_of_current_in_objects_vicinity_old, &speed_of_object_relative_to_medium_ws,
                     (float)state->viscosity_input_factor * delta_time);

    lvp_set_f(&cd->relative_speed_of_current_in_objects_vicinity_old, &relative_speed_of_current_in_objects_vicinity_old);
    cd->core_visible_surface_content_under = core_visible_surface_content_under;
    cd->core_visible_surface_content_under_old = core_visible_surface_content_under_old;
}

void ivp_controller_buoyancy(const ivp_event_sim_t *event_sim,
                             ivp_object_t *obj,
                             void *ctx)
{
    ivp_buoyancy_state_t *state = (ivp_buoyancy_state_t *)ctx;
    if (!state || !state->active || !obj) return;
    if (!state->allocator_bound) {
        state->allocator = *ivp_env_get_allocator(event_sim->environment);
        state->allocator_bound = true;
    }
    ivp_core_t *core = obj->physical_core;
    if (core->physical_unmoveable) return;

    ivp_buoyancy_core_data_t *cd;
    if (state->attacher) {
        /* attached controller of this core (IVP_Attacher_To_Cores_Buoyancy) */
        cd = ivp_buoyancy_attacher_get_core_data(state->attacher, core);
    } else {
        if (!object_in_set(state, obj)) return;
        cd = core_data(state, obj, event_sim->environment);
    }
    if (!cd) return;
    buoyancy_simulate_core(state, cd, core, event_sim);
}

/* ── IVP_Attacher_To_Cores_Buoyancy ─────────────────────────────────────── */

static ivp_buoyancy_attacher_t *buoy_attacher_of(ivp_attacher_to_cores_t *a)
{
    return (ivp_buoyancy_attacher_t *)a->user_data;
}

/* IVP_Controller_Buoyancy(attacher, core): add_controller_to_core */
static void *buoy_create_attachment(ivp_attacher_to_cores_t *a, ivp_core_t *core)
{
    ivp_buoyancy_attacher_t *ba = buoy_attacher_of(a);
    ivp_object_t *obj = ivp_attacher_core_object(core);
    const ivp_allocator_t *allocator = buoyancy_allocator(&ba->template_buoyancy);
    ivp_buoyancy_core_data_t *cd =
        (ivp_buoyancy_core_data_t *)ivp_alloc(allocator, sizeof(*cd));
    if (!cd) return NULL;
    core_data_init(cd, obj, allocator);
    if (!ivp_environment_add_controller_to_core(ba->environment, ba->controller, obj)) {
        ivp_free(allocator, cd, sizeof(*cd));
        return NULL;
    }
    return cd;
}

/* ~IVP_Controller_Buoyancy: remove_controller_from_core */
static void buoy_delete_attachment(ivp_attacher_to_cores_t *a, ivp_core_t *core, void *attachment)
{
    ivp_buoyancy_attacher_t *ba = buoy_attacher_of(a);
    ivp_buoyancy_core_data_t *cd = (ivp_buoyancy_core_data_t *)attachment;
    if (a->environment) {
        (void)ivp_environment_remove_controller_from_core(ba->environment, ba->controller,
                                                          ivp_attacher_core_object(core));
    }
    core_data_release(cd);
    ivp_free(core_data_allocator(cd), cd, sizeof(*cd));
}

static void buoy_free(ivp_buoyancy_attacher_t *ba, bool environment_alive)
{
    if (environment_alive && ba->environment) {
        (void)ivp_environment_remove_controller(ba->environment, ba->controller);
    }
    ivp_free(buoyancy_allocator(&ba->template_buoyancy), ba, sizeof(*ba));
}

/* the set was deleted: P_DELETE_THIS */
static void buoy_attacher_deleted(ivp_attacher_to_cores_t *a)
{
    ivp_buoyancy_attacher_t *ba = buoy_attacher_of(a);
    if (ba->deleted_fn) ba->deleted_fn(ba->deleted_ctx, ba);
    buoy_free(ba, !a->environment_deleted);
}

static const ivp_attacher_to_cores_class_t buoy_attacher_class = {
    buoy_create_attachment,
    buoy_delete_attachment,
    buoy_attacher_deleted,
};

ivp_buoyancy_attacher_t *ivp_buoyancy_attacher_create(ivp_environment_t *env,
                                                      const ivp_buoyancy_state_t *templ,
                                                      ivp_set_active_t *set_of_cores)
{
    if (!env || !templ || !set_of_cores) return NULL;
    const ivp_allocator_t *allocator = ivp_env_get_allocator(env);
    ivp_buoyancy_attacher_t *ba =
        (ivp_buoyancy_attacher_t *)ivp_alloc(allocator, sizeof(*ba));
    if (!ba) return NULL;
    memset(ba, 0, sizeof(*ba));
    /* template_buoyancy = templ (parameters and liquid surface descriptor) */
    ba->template_buoyancy = *templ;
    ba->template_buoyancy.allocator = *allocator;
    ba->template_buoyancy.allocator_bound = true;
    ba->template_buoyancy.objects = NULL;
    ba->template_buoyancy.object_count = 0;
    memset(ba->template_buoyancy.cores, 0, sizeof(ba->template_buoyancy.cores));
    ba->template_buoyancy.cores_overflow = NULL;
    ba->template_buoyancy.cores_overflow_capacity = 0;
    ba->template_buoyancy.core_count = 0;
    ba->template_buoyancy.attacher = ba;
    ba->environment = env;
    ba->controller = ivp_environment_create_core_controller(
        env, IVP_CP_FORCEFIELDS, ivp_controller_buoyancy,
        &ba->template_buoyancy);
    if (!ivp_controller_handle_is_valid(ba->controller)) {
        ivp_free(allocator, ba, sizeof(*ba));
        return NULL;
    }
    if (!ivp_attacher_to_cores_init(&ba->attacher, env, set_of_cores, &buoy_attacher_class, ba)) {
        buoy_free(ba, true);
        return NULL;
    }
    return ba;
}

void ivp_buoyancy_attacher_destroy(ivp_buoyancy_attacher_t *ba)
{
    if (!ba) return;
    const bool env_alive = !ba->attacher.environment_deleted;
    ivp_attacher_to_cores_destroy(&ba->attacher);
    buoy_free(ba, env_alive);
}

void ivp_buoyancy_attacher_set_deleted_listener(ivp_buoyancy_attacher_t *ba,
                                                ivp_buoyancy_attacher_deleted_fn_t fn, void *ctx)
{
    if (!ba) return;
    ba->deleted_fn = fn;
    ba->deleted_ctx = ctx;
}

ivp_buoyancy_core_data_t *ivp_buoyancy_attacher_get_core_data(ivp_buoyancy_attacher_t *ba,
                                                              const ivp_core_t *core)
{
    if (!ba) return NULL;
    return (ivp_buoyancy_core_data_t *)ivp_attacher_to_cores_get_attachment(&ba->attacher, core);
}
