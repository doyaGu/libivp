/* ivp_mindist_debug.h — debug helpers of the mindist pipeline (internal)
 *
 * Reference: ivp_physics/ivp_mindist_debug.cxx, ivp_physics/ivp_debug.hxx,
 * ivp_physics/ivp_mindist.hxx (DEBUG_CHECK_LEN), the "#ifdef DEBUG" parts of
 * ivp_collision/ivp_compact_ledge_solver.cxx.  Implemented in
 * ivp_mindist_minimize.c (they need the IVP_CLS helpers kept there).
 */
#ifndef IVP_MINDIST_DEBUG_H
#define IVP_MINDIST_DEBUG_H

#include <ivp/base/ivp_assert.h>
#include <ivp/collision/ivp_collision_core.h>

/* ivp_mindist.hxx:100: DEBUG_CHECK_LEN (termination length checks of the
 * minimize walk) exists only in a debug build that also defines
 * IVP_MINDIST_BEHAVIOUR_DEBUG (opt-in: -DIVP_MINDIST_BEHAVIOUR_DEBUG; the
 * reference keeps "#define IVP_MINDIST_BEHAVIOUR_DEBUG" commented out). */
#if defined(IVP_DEBUG_BUILD) && defined(IVP_MINDIST_BEHAVIOUR_DEBUG)
#define IVP_DEBUG_CHECK_LEN
#endif

/* ivp_mindist.hxx:110-111 */
#define IVP_MIN_TERMINATION_QDLEN 1E-3f     /* generates core */
#define IVP_MIN_TERMINATION_QDLEN_EPS 1E-6f /* generates warning */

/* ivp_check_debug_mindist (ivp_mindist_debug.cxx, compiled in every build
 * like the reference): true (after printing "name0 name1 time-md: ") when
 * the mindist connects the objects named IVP_DEBUG_OBJECT0 / 1
 * (ivp/diagnostics/ivp_debug_manager.h) from IVP_DEBUG_TIME on.  The
 * reference ships both names as 0: always false.  Used as IVP_IF condition
 * of the mindist traces. */
bool ivp_check_debug_mindist(ivp_mindist_t *md);

/* IVP_Synapse_Real::print, IVP_Mindist::print,
 * IVP_Mindist_Manager::print_mindists (ivp_mindist.cxx, compiled in every
 * reference build, never called by the engine). */
void ivp_md_synapse_print(const ivp_md_synapse_t *s);
void ivp_mindist_print(ivp_mindist_t *md, const char *text);
void ivp_mindist_mgr_print_mindists(ivp_mindist_mgr_t *mm);

/* p_sqrt (ivp_mindist_debug.cxx, "for debugger"): prints sqrt(val). */
int ivp_mindist_debug_p_sqrt(double val);

#ifdef IVP_DEBUG_BUILD
/* Debugger helpers of the reference debug build (ivp_mindist_debug.cxx
 * "#ifdef DEBUG"), never called by the engine:
 *  - p_optimize_FF: brute force minimal quad distance of two triangles
 *    (A, B: first edges), min(min_qdist, ...);
 *  - proove_polypoly / proove_ballpoly: brute force check of a mindist's
 *    length (prints and CORE when it differs by 50 percent or more);
 *  - situation: prints a triangle / edge situation in the triangle plane. */
ivp_real_t ivp_mindist_debug_p_optimize_FF(const ivp_compact_edge_t *A, const ivp_compact_edge_t *B,
                                           ivp_cache_ledge_point_t *m_cache_A, ivp_cache_ledge_point_t *m_cache_B,
                                           ivp_real_t min_qdist);
/* IVP_RETURN_TYPE: true == IVP_OK, false == IVP_FAULT */
bool ivp_mindist_debug_proove_polypoly(ivp_mindist_t *mindist);
bool ivp_mindist_debug_proove_ballpoly(ivp_mindist_t *mindist);
int ivp_mindist_debug_situation(ivp_cache_ledge_point_t *tri_cache, ivp_cache_ledge_point_t *e_cache,
                                const ivp_compact_edge_t *triangle_e0, const ivp_compact_edge_t *edge);

/* IVP_Compact_Ledge_Solver debug functions ("#ifdef DEBUG",
 * ivp_compact_ledge_solver.cxx:280-391, 583) in world space. */
ivp_real_t ivp_cls_debug_calc_s_val(const ivp_compact_edge_t *edge, const ivp_vec3r_t *p_world,
                                    ivp_cache_ledge_point_t *m_cache_edge);
void ivp_cls_debug_calc_qr_vals(const ivp_compact_edge_t *e_tri, const ivp_vec3r_t *p_world, ivp_real_t *out_q,
                                ivp_real_t *out_r, ivp_cache_ledge_point_t *m_cache_e_tri);
void ivp_cls_debug_calc_qr_vals_edge(const ivp_compact_edge_t *e_tri, const ivp_compact_edge_t *p,
                                     ivp_real_t *out_q, ivp_real_t *out_r, ivp_cache_ledge_point_t *m_cache_e_tri,
                                     ivp_cache_ledge_point_t *m_cache_p);
void ivp_cls_debug_calc_hesse_normized_AT(const ivp_compact_edge_t *edge, ivp_cache_ledge_point_t *clp,
                                          ivp_vec3r_t *normal_ws, ivp_real_t *hesse_val_ws);
#endif /* IVP_DEBUG_BUILD */

#endif /* IVP_MINDIST_DEBUG_H */
