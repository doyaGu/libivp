#include <ivp/math/ivp_fpu.h>

#include "math/ivp_x87.h"

#include <stddef.h>

#if IVP_X87_ARITHMETIC
#if defined(_MSC_VER) && !defined(__clang__)
#define IVP_FPU_THREAD_LOCAL __declspec(thread)
#else
#define IVP_FPU_THREAD_LOCAL _Thread_local
#endif

static IVP_FPU_THREAD_LOCAL ivp_fpu_read_control_t read_host_control;
static IVP_FPU_THREAD_LOCAL ivp_fpu_write_control_t write_host_control;

void ivp_x87_select_extended_precision(void)
{
    if (!read_host_control)
        return;

    const uint16_t word = read_host_control();
    const uint16_t extended = (uint16_t)(word | 0x0300u);
    if (word != extended)
        write_host_control(extended);
}
#endif

void ivp_set_fpu_control_word_callbacks(ivp_fpu_read_control_t read_control,
                                       ivp_fpu_write_control_t write_control)
{
#if IVP_X87_ARITHMETIC
    read_host_control = write_control ? read_control : NULL;
    write_host_control = read_control ? write_control : NULL;
#else
    (void)read_control;
    (void)write_control;
#endif
}
