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
void ivp_x87_select_extended_precision(void);
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
double ivp_x87_spring_impulse(const ivp_vec3f_t *direction, const ivp_vec3f_t *relative_speed,
                              float length, float rest_length, float stiffness, float axial_damping,
                              float lateral_damping, double step, double maximum_force, ivp_vec3f_t *impulse);
double ivp_x87_intersect_ray_edge(const double origin[3], const double direction[3], const double start[3],
                                  const double end[3], int axis, double point[3]);
float ivp_x87_reduced_inverse_mass(double first, double second, double epsilon);
double ivp_x87_worst_case_effective_mass(const ivp_vec3f_t *point,
                                         const ivp_vec3f_t *inverse_inertia, float inverse_mass);
float ivp_x87_worst_case_inverse_mass(const ivp_vec3f_t *point, const ivp_vec3f_t *inverse_inertia,
                                      float inverse_mass, double epsilon);
void ivp_x87_add_scaled_row(double *target, const double *source, double factor, int count);
double ivp_x87_dot_product_forward(const double *left, const double *right, int count);
double ivp_x87_dot_product_backward(const double *left, const double *right, int count);
double ivp_x87_back_substitute_lu_row(double value, const double *upper, const double *solution, int count);
double ivp_x87_indexed_dot_product_with_bias(const double *left, const double *right,
                                             const int *indices, int count, double bias);
double ivp_x87_dot_product_residual(const double *left, const double *right, int count, double desired);
double ivp_x87_divide_double(double numerator, double denominator);
double ivp_x87_multiply_double(double left, double right);
bool ivp_x87_back_substitute_gauss_row(const double *upper, double *solution,
                                       int pivot, int count, double epsilon);
void ivp_x87_normalize_lu_lower_row(double *lower, int count, int pivot);
void ivp_x87_normalize_lu_pivot(double *lower, double *upper, int count, int pivot);
void ivp_x87_contact_normal_jacobian(const ivp_mat3r_t *rotation, const ivp_vec3f_t *normal,
    const ivp_vec3f_t *point, bool second_body, ivp_vec3f_t *jacobian);
void ivp_x87_inverse_transform_contact_point(const ivp_transform_r_t *transform,
                                             const ivp_vec3r_t *world, ivp_vec3f_t *local);
void ivp_x87_surface_speed(const ivp_mat3r_t *rotation, const ivp_vec3f_t *point,
                           const ivp_vec3f_t *linear_speed, const ivp_vec3f_t *angular_speed,
                           ivp_vec3f_t *velocity);
void ivp_x87_test_push(const ivp_vec3f_t *point, const ivp_vec3f_t *local_impulse,
                       const ivp_vec3f_t *world_impulse, float inverse_mass,
                       const ivp_vec3f_t *inverse_inertia, ivp_vec3f_t *linear_speed,
                       ivp_vec3f_t *angular_speed);
bool ivp_x87_solve_constraint_system(int count, double matrix[6][6], double desired[6],
                                     double result[6], double epsilon);
double ivp_x87_collision_speed_bound(float first_linear, float second_linear,
                                     float first_surface, float second_surface);
double ivp_x87_collision_time(double first, double last, double target,
                              double first_distance, double last_distance, bool adjust_midpoint);
bool ivp_x87_better_edge_face_gradient(const ivp_vec3r_t *edge, const ivp_vec3r_t *face_normal,
                                       ivp_real_t minimum, ivp_real_t *gradient);
#ifdef IVP_REAL_DOUBLE
double ivp_x87_vertex_walk_projection(const ivp_vec3r_t *offset, const ivp_vec3f_t *vertex);
bool ivp_x87_better_vertex_walk_gradient(const ivp_vec3r_t *offset, const ivp_vec3f_t *vertex,
                                         const ivp_vec3f_t *neighbor, double projection,
                                         double best_gradient, double *gradient);
#endif
float ivp_x87_point_face_distance(const ivp_vec3r_t *normal, const ivp_vec3r_t *point,
                                  const ivp_vec3f_t *face_point);
double ivp_x87_hull_determinant(double *const rows[], int dimension);
int ivp_x87_hull_determinant_exceeds(double *const rows[], int dimension, double maximum);
bool ivp_x87_set_hull_plane_offset(double *const vertices[3], const double origin[3],
                                   const double normal[3], double tolerance, double *offset);
double ivp_x87_hull_plane_gauss_offset(const double *point, const double *normal, int dimension);
double ivp_x87_hull_plane_distance(const double point[3], const double normal[3], double offset);
int ivp_x87_normalize_hull_plane(double normal[3], unsigned orientation,
                                 double min_denominator, const double *minimum,
                                 unsigned *below_minimum, double *length);
double ivp_x87_length_squared3f(const float point[3]);
float ivp_x87_minimum_rotational_inertia(const ivp_vec3f_t *inertia, float factor);
double ivp_x87_length3f(const float point[3]);
double ivp_x87_length3d(const double point[3]);
bool ivp_x87_normalize3f(float point[3], int steps, double epsilon, double *length);
bool ivp_x87_normalize3d(double point[3], int steps, double epsilon, double *length);
#ifdef IVP_REAL_DOUBLE
#define ivp_x87_normalize3r ivp_x87_normalize3d
#else
#define ivp_x87_normalize3r ivp_x87_normalize3f
#endif

void ivp_x87_surface_radius_and_deviation(const float mass_center[3], const float center[3],
                                          float radius, int deviation, float step,
                                          float *out_radius, float *out_deviation);
void ivp_x87_inertia_mass_triangle(const float p0[3], const float p1[3], const float p2[3],
                                   double *sum_det, double *sum_squared_area, float sum_mass[3]);
void ivp_x87_inertia_edge_integral(float p0_x, float p0_y, float p0_z, float p1_x,
                                   double vector_x, double vector_y, double vector_z,
                                   bool triangle_area, double fzdy, double fydz, double area[3]);

void ivp_x87_rotate3d(const double rotation[3][3], const double local[3], double world[3]);
void ivp_x87_inverse_rotate3d(const double rotation[3][3], const double world[3], double local[3]);
void ivp_x87_inverse_transform_point3d(const double rotation[3][3], const double origin[3],
                                       const double world[3], double local[3]);
void ivp_x87_multiply_rotation3d(const double left[3][3], const double right[3][3], double output[3][3]);
void ivp_x87_multiply_transposed_rotation3d(const double left[3][3], const double right[3][3],
                                            double output[3][3]);
bool ivp_x87_invert_transform3d(double matrix[3][3], double origin[3], double epsilon);
void ivp_x87_constraint_rotation_error(const double relative_rotation[3][3], double error[3]);
void ivp_x87_cross3r(const ivp_vec3r_t *left, const ivp_vec3r_t *right, ivp_vec3r_t *cross);
void ivp_x87_advance_position(const ivp_vec3r_t *position, const ivp_vec3r_t *velocity,
                              double elapsed, ivp_vec3r_t *out);
void ivp_x87_interpolate_point(const ivp_vec3r_t *first, const ivp_vec3r_t *last, ivp_real_t fraction,
                               ivp_vec3r_t *out);
ivp_real_t ivp_x87_edge_contact_fraction(const ivp_vec3r_t *normal, const ivp_vec3r_t *first,
                                         const ivp_vec3r_t *last, const ivp_vec3r_t *point);
void ivp_x87_transform_point3d(const double rotation[3][3], const double origin[3],
                               const double point[3], double out[3]);
#ifdef IVP_REAL_DOUBLE
double ivp_x87_point_plane_separation(const ivp_transform_r_t *point_transform,
                                      const ivp_vec3r_t *point, const ivp_transform_r_t *plane_transform,
                                      const ivp_vec3r_t *plane_point, const ivp_vec3r_t *plane_normal);
double ivp_x87_edge_edge_separation(const ivp_transform_r_t *first_transform, const ivp_vec3r_t first_edge[2],
                                    const ivp_transform_r_t *second_transform, const ivp_vec3r_t second_edge[2],
                                    double orientation);
double ivp_x87_edge_face_projection(const ivp_mat3r_t *edge_rotation, const ivp_vec3r_t *edge,
                                    const ivp_mat3r_t *face_rotation, const ivp_vec3r_t *normal);
#endif
void ivp_x87_transform_mesh_vertex(const ivp_transform_r_t *transform, const ivp_vec3f_t *vertex,
                                   ivp_vec3r_t *out);
void ivp_x87_transform_collision_point(const ivp_vec3f_t *point,
                                       const ivp_transform_r_t *source, const ivp_transform_r_t *target,
                                       ivp_vec3r_t *out);

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
