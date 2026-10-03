/* test_active_value_ext.c — Tests for oscillator active value types */
#include "ivp_test.h"
#include <ivp/utility/ivp_active_value.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── Sine oscillator tests ─────────────────────────────────────────────── */

IVP_TEST(sine_at_known_phases) {
    ivp_active_float_t *time_src = ivp_active_float_terminal_create(NULL, "t", 0.0);
    IVP_ASSERT_NOT_NULL(time_src);

    /* Reference IVP_U_Active_Sine: sin(t*freq + shift)*amp + null (no 2*PI
     * factor), so freq=2*PI, amp=2, null=0, shift=0 → 2*sin(2*PI*t) */
    ivp_active_float_t *sine = ivp_active_float_sine_create(NULL, "sine", time_src,
        2.0 * M_PI, 2.0, 0.0, 0.0);
    IVP_ASSERT_NOT_NULL(sine);

    /* t=0: sin(0) = 0 → value = 0 */
    IVP_ASSERT_NEAR(ivp_active_float_get(sine), 0.0, 1e-6);

    /* t=0.25: sin(PI/2) = 1 → value = 2 */
    ivp_active_float_set(time_src, 0.25);
    IVP_ASSERT_NEAR(ivp_active_float_get(sine), 2.0, 1e-6);

    /* t=0.5: sin(PI) = 0 → value = 0 */
    ivp_active_float_set(time_src, 0.5);
    IVP_ASSERT_NEAR(ivp_active_float_get(sine), 0.0, 1e-6);

    /* t=0.75: sin(3*PI/2) = -1 → value = -2 */
    ivp_active_float_set(time_src, 0.75);
    IVP_ASSERT_NEAR(ivp_active_float_get(sine), -2.0, 1e-6);

    ivp_active_float_remove_ref(sine);
    ivp_active_float_remove_ref(time_src);
}

IVP_TEST(sine_with_offset_and_shift) {
    ivp_active_float_t *time_src = ivp_active_float_terminal_create(NULL, "t", 0.0);
    /* reference: sin(t*freq + shift)*amp + null; freq=1, amp=1, null=5,
     * shift=PI/2 (the shift is a phase in radians, added after t*freq) */
    ivp_active_float_t *sine = ivp_active_float_sine_create(NULL, "sine", time_src,
        1.0, 1.0, 5.0, M_PI / 2.0);
    IVP_ASSERT_NOT_NULL(sine);

    /* t=0: sin(PI/2) + 5 = 1 + 5 = 6 */
    IVP_ASSERT_NEAR(ivp_active_float_get(sine), 6.0, 1e-6);

    ivp_active_float_remove_ref(sine);
    ivp_active_float_remove_ref(time_src);
}

/* ── Square wave tests ─────────────────────────────────────────────────── */

IVP_TEST(square_wave_transition) {
    ivp_active_float_t *time_src = ivp_active_float_terminal_create(NULL, "t", 0.0);
    /* Reference IVP_U_Active_Square: ((int)(t*freq) & 1) ? high : low,
     * i.e. period 2/freq starting low.  freq=2, low=0, high=10 */
    ivp_active_float_t *sq = ivp_active_float_square_create(NULL, "sq", time_src,
        2.0, 0.0, 10.0);
    IVP_ASSERT_NOT_NULL(sq);

    /* t=0.1: (int)0.2 = 0 → low=0 */
    ivp_active_float_set(time_src, 0.1);
    IVP_ASSERT_NEAR(ivp_active_float_get(sq), 0.0, 1e-6);

    /* t=0.6: (int)1.2 = 1 → high=10 */
    ivp_active_float_set(time_src, 0.6);
    IVP_ASSERT_NEAR(ivp_active_float_get(sq), 10.0, 1e-6);

    /* t=1.2: (int)2.4 = 2 → low=0 */
    ivp_active_float_set(time_src, 1.2);
    IVP_ASSERT_NEAR(ivp_active_float_get(sq), 0.0, 1e-6);

    ivp_active_float_remove_ref(sq);
    ivp_active_float_remove_ref(time_src);
}

/* ── Pulse wave tests ──────────────────────────────────────────────────── */

IVP_TEST(pulse_duty_cycle) {
    ivp_active_float_t *time_src = ivp_active_float_terminal_create(NULL, "t", 0.0);
    /* reference: ((int)(t*freq*m2) % m2 < m1) ? high : low;
     * freq=1, m1=1, m2=3, low=0, high=100 → duty = 1/3 high */
    ivp_active_float_t *pulse = ivp_active_float_pulse_create(NULL, "pulse", time_src,
        1.0, 1, 3, 0.0, 100.0);
    IVP_ASSERT_NOT_NULL(pulse);

    /* t=0.1: phase=0.1 < 0.333 → high=100 */
    ivp_active_float_set(time_src, 0.1);
    IVP_ASSERT_NEAR(ivp_active_float_get(pulse), 100.0, 1e-6);

    /* t=0.5: phase=0.5 >= 0.333 → low=0 */
    ivp_active_float_set(time_src, 0.5);
    IVP_ASSERT_NEAR(ivp_active_float_get(pulse), 0.0, 1e-6);

    ivp_active_float_remove_ref(pulse);
    ivp_active_float_remove_ref(time_src);
}

/* ── Sub tests ─────────────────────────────────────────────────────────── */

IVP_TEST(sub_basic) {
    ivp_active_float_t *a = ivp_active_float_terminal_create(NULL, "a", 10.0);
    ivp_active_float_t *b = ivp_active_float_terminal_create(NULL, "b", 3.0);
    ivp_active_float_t *sub = ivp_active_float_sub_create(NULL, "sub", a, b);
    IVP_ASSERT_NOT_NULL(sub);

    IVP_ASSERT_NEAR(ivp_active_float_get(sub), 7.0, 1e-6);

    /* Change a → propagates */
    ivp_active_float_set(a, 20.0);
    IVP_ASSERT_NEAR(ivp_active_float_get(sub), 17.0, 1e-6);

    ivp_active_float_remove_ref(sub);
    ivp_active_float_remove_ref(a);
    ivp_active_float_remove_ref(b);
}

/* ── AddMultiple tests ─────────────────────────────────────────────────── */

IVP_TEST(add_multiple_basic) {
    ivp_active_float_t *a = ivp_active_float_terminal_create(NULL, "a", 5.0);
    ivp_active_float_t *b = ivp_active_float_terminal_create(NULL, "b", 3.0);
    /* a + factor * b = 5 + 2*3 = 11 */
    ivp_active_float_t *am = ivp_active_float_add_multiple_create(NULL, "am", a, b, 2.0);
    IVP_ASSERT_NOT_NULL(am);

    IVP_ASSERT_NEAR(ivp_active_float_get(am), 11.0, 1e-6);

    /* Change b → propagates */
    ivp_active_float_set(b, 10.0);
    IVP_ASSERT_NEAR(ivp_active_float_get(am), 25.0, 1e-6);

    ivp_active_float_remove_ref(am);
    ivp_active_float_remove_ref(a);
    ivp_active_float_remove_ref(b);
}

/* ── Propagation chain test ────────────────────────────────────────────── */

IVP_TEST(sine_propagation_chain) {
    /* terminal time → sine → add (with constant) */
    ivp_active_float_t *time_src = ivp_active_float_terminal_create(NULL, "t", 0.0);
    ivp_active_float_t *sine = ivp_active_float_sine_create(NULL, "sine", time_src,
        2.0 * M_PI, 1.0, 0.0, 0.0); /* reference sin(t*freq): freq = 2*PI */
    ivp_active_float_t *offset = ivp_active_float_terminal_create(NULL, "off", 100.0);
    ivp_active_float_t *result = ivp_active_float_add_create(NULL, "result", sine, offset);

    IVP_ASSERT_NOT_NULL(result);

    /* t=0: sin(0) + 100 = 100 */
    IVP_ASSERT_NEAR(ivp_active_float_get(result), 100.0, 1e-6);

    /* t=0.25: sin(PI/2) + 100 = 101 */
    ivp_active_float_set(time_src, 0.25);
    IVP_ASSERT_NEAR(ivp_active_float_get(result), 101.0, 1e-6);

    ivp_active_float_remove_ref(result);
    ivp_active_float_remove_ref(offset);
    ivp_active_float_remove_ref(sine);
    ivp_active_float_remove_ref(time_src);
}

/* ── Registration ──────────────────────────────────────────────────────── */

#define IVP_TEST_LIST \
    IVP_REGISTER(sine_at_known_phases); \
    IVP_REGISTER(sine_with_offset_and_shift); \
    IVP_REGISTER(square_wave_transition); \
    IVP_REGISTER(pulse_duty_cycle); \
    IVP_REGISTER(sub_basic); \
    IVP_REGISTER(add_multiple_basic); \
    IVP_REGISTER(sine_propagation_chain);

int main(void) {
    IVP_TEST_LIST
    IVP_RUN_ALL();
}
