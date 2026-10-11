/* ivp_merge_core.h -- merged cores (port of IVP_Core_Merged / IVP_Merge_Core,
 * ivp_intern/ivp_merge_core.{hxx,cxx}, and IVP_Environment::merge_objects)
 *
 * ivp_environment_merge_objects gives several objects one shared core: the
 * new IVP_Core_Merged takes the orientation of the heaviest core (or of an
 * unmovable core), its matrix position is the mass center (pos_world_f_core_
 * last_psi stays at the heaviest core, exactly like the reference), mass,
 * rotation inertia, speeds and damping are combined by IVP_Merge_Core and
 * every object is placed in the new core space (set_new_m_object_f_core).
 * The old cores are deleted.  The objects' physical_core, friction_core and
 * original_core all become the merged core.
 *
 * Preconditions (reference 8ad306e; an invalid request is rejected
 * atomically: false is returned and nothing changes):
 *   - no NULL entry, every object belongs to this environment and has a
 *     core, and no object is listed twice;
 *   - every object is the only object of its core (cores of merged or
 *     attached objects are shared);
 *   - no core is simulated: freeze objects first
 *     (ivp_object_disable_simulation) or merge them before they are woken
 *     up;
 *   - no object has contact points.
 * Any number of objects can be merged (IVP_Merge_Core::core_stack is not
 * written, as in the reference).
 */
#ifndef IVP_MERGE_CORE_H
#define IVP_MERGE_CORE_H

#include <ivp/base/ivp_types.h>
#include <ivp/dynamics/ivp_core.h>
#include <ivp/dynamics/ivp_object.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IVP_MAX_CORE_STACK 6

struct ivp_environment;

/* IVP_Environment::merge_objects(objs_to_merge): the merged core is
 * created from objs_to_merge[0] (IVP_Core_Merged(real_obj)).  Returns false
 * (and changes nothing) when a precondition above is not satisfied. */
bool ivp_environment_merge_objects(struct ivp_environment *env, ivp_object_t *const *objs_to_merge, int n_objs);

/* IVP_Core::set_matrizes_and_speed (splitting): set the matrix, speed and
 * rotation speed of a core that was replaced by template_core (directly or
 * through a chain of merged cores) from template_core; *m_CORE_f_core_out is
 * the change of core space.  Not used by the reference simulation (its only
 * caller, IVP_Core_Collision::split_collision_merged_core_next_PSI, returns
 * before reaching it). */
void ivp_core_set_matrizes_and_speed(ivp_core_t *core, ivp_core_merged_t *template_core,
                                     ivp_transform_r_t *m_CORE_f_core_out);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MERGE_CORE_H */
