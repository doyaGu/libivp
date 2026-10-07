/* test_demo_drive.c -- The smoothing of the driving examples (examples/demo_drive_core.h) */
#include "ivp_test.h"
#include "demo_drive_core.h"

static void run(demo_drive_t *d, float throttle, float steer, float seconds)
{
    demo_drive_input_t in;
    memset(&in, 0, sizeof(in));
    in.throttle = throttle;
    in.steer = steer;
    const float dt = 1.0f / 120.0f;
    for (float t = 0.0f; t < seconds - 1e-6f; t += dt) demo_drive_step(d, &in, dt);
}

IVP_TEST(throttle_winds_on_over_its_time_and_falls_twice_as_fast) {
    demo_drive_t d;
    demo_drive_defaults(&d);
    run(&d, 1.0f, 0.0f, 0.5f * d.throttle_time);
    IVP_ASSERT_NEAR(d.throttle, 0.5f, 0.02f);
    run(&d, 1.0f, 0.0f, d.throttle_time);
    IVP_ASSERT_NEAR(d.throttle, 1.0f, 1e-6f);
    run(&d, 0.0f, 0.0f, 0.25f * d.throttle_time);
    IVP_ASSERT_NEAR(d.throttle, 0.5f, 0.07f);          /* 2 / throttle_time, to the step */
    run(&d, 0.0f, 0.0f, d.throttle_time);
    IVP_ASSERT_NEAR(d.throttle, 0.0f, 1e-6f);
}

IVP_TEST(steering_winds_on_slowly_and_centres_quickly) {
    demo_drive_t d;
    demo_drive_defaults(&d);
    run(&d, 0.0f, 1.0f, 0.5f * d.steer_time);
    IVP_ASSERT_NEAR(d.steer, 0.5f, 0.02f);
    run(&d, 0.0f, 0.0f, 0.25f * d.center_time);
    IVP_ASSERT_NEAR(d.steer, 0.25f, 0.03f);            /* 1 / center_time: from 0.5 down by 0.25 */
    IVP_ASSERT_TRUE(d.center_time < d.steer_time);
    run(&d, 0.0f, 0.0f, d.center_time);
    IVP_ASSERT_NEAR(d.steer, 0.0f, 1e-6f);
}

IVP_TEST(a_reversal_of_the_steering_is_twice_as_fast) {
    demo_drive_t d;
    demo_drive_defaults(&d);
    run(&d, 0.0f, 1.0f, d.steer_time);
    IVP_ASSERT_NEAR(d.steer, 1.0f, 1e-6f);
    run(&d, 0.0f, -1.0f, 0.25f * d.steer_time);       /* 2 / steer_time from +1 */
    IVP_ASSERT_NEAR(d.steer, 0.5f, 0.02f);
}

IVP_TEST(a_tap_is_a_small_correction) {
    demo_drive_t d;
    demo_drive_defaults(&d);
    run(&d, 0.0f, 1.0f, 0.1f);
    IVP_ASSERT_TRUE(d.steer > 0.1f && d.steer < 0.3f);
    run(&d, 0.0f, 0.0f, 1.0f);
    IVP_ASSERT_NEAR(d.steer, 0.0f, 1e-6f);
}

IVP_TEST(a_long_frame_does_not_jump) {
    demo_drive_t d;
    demo_drive_defaults(&d);
    demo_drive_input_t in = { 0.0f, 1.0f, false, false };
    demo_drive_step(&d, &in, 5.0f);                    /* clamped to 0.1 s */
    IVP_ASSERT_NEAR(d.steer, 0.1f / d.steer_time, 1e-5f);
}

IVP_TEST(the_lock_shrinks_with_the_speed) {
    IVP_ASSERT_NEAR(demo_drive_lock(0.0f, 14.0f), 1.0f, 1e-6f);
    IVP_ASSERT_NEAR(demo_drive_lock(14.0f, 14.0f), 0.5f, 1e-6f);
    IVP_ASSERT_NEAR(demo_drive_lock(-14.0f, 14.0f), 0.5f, 1e-6f);
    IVP_ASSERT_TRUE(demo_drive_lock(30.0f, 14.0f) < demo_drive_lock(10.0f, 14.0f));
}

int main(void) {
    IVP_REGISTER(throttle_winds_on_over_its_time_and_falls_twice_as_fast);
    IVP_REGISTER(steering_winds_on_slowly_and_centres_quickly);
    IVP_REGISTER(a_reversal_of_the_steering_is_twice_as_fast);
    IVP_REGISTER(a_tap_is_a_small_correction);
    IVP_REGISTER(a_long_frame_does_not_jump);
    IVP_REGISTER(the_lock_shrinks_with_the_speed);
    IVP_RUN_ALL();
}
