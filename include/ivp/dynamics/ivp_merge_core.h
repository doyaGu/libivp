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
 * Reference limits (the reference crashes, hangs or leaves dangling
 * pointers otherwise; libivp refuses such a merge and returns false
 * without changing anything):
 *   - every object must be the only object of its core (cores of objects
 *     that are already merged / attached are deleted while other objects
 *     still use them, and a core listed twice is deleted twice);
 *   - no core may be simulated: IVP_Core::init leaves the merged core's
 *     q_world_f_core_next_psi zero and the simulated branch of
 *     merge_objects normalizes it (endless loop in fast_normize_quat);
 *     freeze objects first (ivp_object_disable_simulation) or merge them
 *     before they are woken up;
 *   - no object may have contact points (friction systems keep pointers to
 *     the deleted cores).
 * More than IVP_MAX_CORE_STACK objects overflow the reference's
 * IVP_Merge_Core::core_stack; libivp does not write the (otherwise unused)
 * stack, so any number of objects can be merged.
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
 * (and changes nothing) for inputs outside the reference limits above. */
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
