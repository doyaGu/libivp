#ifndef IVP_FPU_H
#define IVP_FPU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint16_t (*ivp_fpu_read_control_t)(void);
typedef void (*ivp_fpu_write_control_t)(uint16_t word);

/* Bind the current thread's host control word. A point-cloud hull selects
 * extended significand precision and leaves it active for the host, as in
 * IVP. Rounding and exception masks are preserved. Pass null callbacks to
 * detach the host. The callbacks are only used by x87 arithmetic builds. */
void ivp_set_fpu_control_word_callbacks(ivp_fpu_read_control_t read_control,
                                       ivp_fpu_write_control_t write_control);

#ifdef __cplusplus
}
#endif

#endif /* IVP_FPU_H */
