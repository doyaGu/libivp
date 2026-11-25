#ifndef IVP_X87_H
#define IVP_X87_H

#include <ivp/base/ivp_types.h>
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
double ivp_x87_length3d(const double point[3]);
bool ivp_x87_normalize3f(float point[3], int steps, double epsilon, double *length);
bool ivp_x87_normalize3d(double point[3], int steps, double epsilon, double *length);
#ifdef IVP_REAL_DOUBLE
#define ivp_x87_normalize3r ivp_x87_normalize3d
#else
#define ivp_x87_normalize3r ivp_x87_normalize3f
#endif

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
