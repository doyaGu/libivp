/* ivp_cache_object.h — object transforms at any time (AT) between PSIs
 *
 * Reference: IVP_Real_Object::calc_at_matrix / calc_at_quaternion
 * (ivp_physics/ivp_object.cxx).  The reference IVP_Cache_Object manager itself
 * is internal (ivp/collision/ivp_collision_core.h).
 */
#ifndef IVP_CACHE_OBJECT_H
#define IVP_CACHE_OBJECT_H

#include <ivp/dynamics/ivp_object.h>
#include <ivp/math/ivp_transform.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Object world transform at time t (IVP_Real_Object::calc_at_matrix). */
void ivp_object_get_at_matrix(const ivp_object_t *obj, ivp_time_t t, ivp_transform_r_t *out);

/* Object world position / orientation at time t (IVP_Real_Object::calc_at_quaternion). */
void ivp_object_get_at_position(const ivp_object_t *obj, ivp_time_t t, ivp_vec3r_t *out);
void ivp_object_get_at_quat(const ivp_object_t *obj, ivp_time_t t, ivp_quatr_t *out);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CACHE_OBJECT_H */
