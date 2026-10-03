/* ivp_controller_raycast.h -- private world ray caster for controllers
 * (reference IVP_Ray_Solver with IVP_RAY_SOLVER_ALL over the environment). */
#ifndef IVP_CONTROLLER_RAYCAST_H
#define IVP_CONTROLLER_RAYCAST_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/base/ivp_forward_decls.h>

/* Closest hit of a world-space ray (normalized direction, given length)
 * against all objects of env except `exclude`.  Returns false if nothing is
 * hit.  The normal is in world space, normalized, pointing against the ray
 * side of the surface. */
bool ivp_controller_world_raycast(ivp_environment_t *env,
                                  const ivp_object_t *exclude,
                                  const ivp_vec3r_t *start_ws,
                                  const ivp_vec3r_t *dir_ws,
                                  ivp_real_t length,
                                  ivp_real_t *out_distance,
                                  ivp_vec3r_t *out_normal_ws,
                                  ivp_object_t **out_object);

#endif
