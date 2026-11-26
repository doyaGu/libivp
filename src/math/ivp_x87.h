#ifndef IVP_X87_H
#define IVP_X87_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_transform.h>
#include <stdint.h>

#if IVP_X87_ARITHMETIC || IVP_X87_ARITHMETIC_TRANSCENDENTAL
#ifdef __cplusplus
extern "C" {
#endif

/* Select 64 significand bits for a simulation step. Preserve rounding and
 * exception masks, including for nested steps on the same thread. */
uint16_t ivp_x87_enter_step(void);
void ivp_x87_leave_step(uint16_t saved_precision);

#if IVP_X87_ARITHMETIC
void ivp_x87_gyroscopic_coefficients(const ivp_vec3f_t *inertia, const ivp_vec3f_t *inverse_inertia,
                                     double coefficients[3]);
void ivp_x87_add_gyroscopic_velocity(ivp_vec3f_t *angular_velocity, const double coefficients[3],
                                     double step);
void ivp_x87_damp_velocity(ivp_vec3f_t *linear, ivp_vec3f_t *angular,
                           const ivp_vec3f_t *angular_damping,
                           double linear_damping, double step);
void ivp_x87_cross3f(const ivp_vec3f_t *left, const ivp_vec3f_t *right, ivp_vec3f_t *cross);
void ivp_x87_accumulate_world_impulse(const ivp_transform_r_t *transform, const ivp_vec3r_t *point,
                                      const ivp_vec3f_t *impulse, float inverse_mass,
                                      const ivp_vec3f_t *inverse_inertia,
                                      ivp_vec3f_t *linear, ivp_vec3f_t *angular);
double ivp_x87_worst_case_effective_mass(const ivp_vec3f_t *point,
                                         const ivp_vec3f_t *inverse_inertia, float inverse_mass);
float ivp_x87_worst_case_inverse_mass(const ivp_vec3f_t *point, const ivp_vec3f_t *inverse_inertia,
                                      float inverse_mass, double epsilon);
void ivp_x87_surface_speed(const ivp_mat3r_t *rotation, const ivp_vec3f_t *point,
                           const ivp_vec3f_t *linear_speed, const ivp_vec3f_t *angular_speed,
                           ivp_vec3f_t *velocity);
void ivp_x87_test_push(const ivp_vec3f_t *point, const ivp_vec3f_t *local_impulse,
                       const ivp_vec3f_t *world_impulse, float inverse_mass,
                       const ivp_vec3f_t *inverse_inertia, ivp_vec3f_t *linear_speed,
                       ivp_vec3f_t *angular_speed);
double ivp_x87_length3f(const float point[3]);
double ivp_x87_length3d(const double point[3]);
bool ivp_x87_normalize3f(float point[3], int steps, double epsilon, double *length);
bool ivp_x87_normalize3d(double point[3], int steps, double epsilon, double *length);
#ifdef IVP_REAL_DOUBLE
#define ivp_x87_normalize3r ivp_x87_normalize3d
#else
#define ivp_x87_normalize3r ivp_x87_normalize3f
#endif

void ivp_x87_rotate3d(const double rotation[3][3], const double local[3], double world[3]);
void ivp_x87_inverse_rotate3d(const double rotation[3][3], const double world[3], double local[3]);
void ivp_x87_inverse_transform_point3d(const double rotation[3][3], const double origin[3],
                                       const double world[3], double local[3]);
void ivp_x87_multiply_rotation3d(const double left[3][3], const double right[3][3], double output[3][3]);
void ivp_x87_multiply_transposed_rotation3d(const double left[3][3], const double right[3][3],
                                            double output[3][3]);
bool ivp_x87_invert_transform3d(double matrix[3][3], double origin[3], double epsilon);
void ivp_x87_cross3r(const ivp_vec3r_t *left, const ivp_vec3r_t *right, ivp_vec3r_t *cross);
void ivp_x87_advance_position(const ivp_vec3r_t *position, const ivp_vec3r_t *velocity,
                              double elapsed, ivp_vec3r_t *out);
void ivp_x87_interpolate_point(const ivp_vec3r_t *first, const ivp_vec3r_t *last, ivp_real_t fraction,
                               ivp_vec3r_t *out);
void ivp_x87_transform_point3d(const double rotation[3][3], const double origin[3],
                               const double point[3], double out[3]);
bool ivp_x87_quat_normalize(double rotation[4]);
void ivp_x87_quat_to_mat3(const double rotation[4], ivp_mat3r_t *matrix);
void ivp_x87_quat_from_mat3(double rotation[4], const ivp_mat3r_t *matrix);
void ivp_x87_quat_set_fast_multiple_with_clip(ivp_real_t out[4], const float angles[3],
                                              double factor);
#ifdef IVP_REAL_DOUBLE
void ivp_x87_quat_interpolate(ivp_quatr_t *out, const ivp_quatr_t *from,
                              const ivp_quatr_t *to, ivp_real_t time);
#endif
void ivp_x87_quat_invert_mult(ivp_real_t out[4], const ivp_real_t left[4],
                              const ivp_real_t right[4]);
float ivp_x87_quat_rotation_axis(const ivp_vec3r_t *rotation, const ivp_mat3r_t *to_world,
                                 float inverse_step, ivp_vec3f_t *world_axis);
void ivp_x87_quat_mult(ivp_real_t out[4], const ivp_real_t left[4],
                       const ivp_real_t right[4]);
void ivp_x87_quat_set_very_fast_multiple(ivp_real_t out[4], const float angles[3],
                                        double factor);
#endif

#ifdef __cplusplus
}
#endif
#endif

#endif /* IVP_X87_H */
