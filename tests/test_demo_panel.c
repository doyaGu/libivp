/* test_demo_panel.c -- Slider and checkbox logic of examples/demo_panel.h
 *
 * The panel takes the mouse state as arguments, so it can be driven
 * without a window. Layout (win_w = 1000, top = 100): the panel sits at
 * the right edge, x = 1000 - 270 - 10 = 720, y = 100; the first row starts
 * 26 px below that.
 */
#include "ivp_test.h"
#include "demo_panel.h"

static int s_changes;
static void on_change(void *ctx) { (void)ctx; s_changes++; }

#define WIN_W 1000
#define TOP   100.0f
#define PX    720.0f

/* centre of the slider track of float row 0 */
static float track_y(void) { return TOP + DEMO_PANEL_HEAD_H + 26.0f; }

IVP_TEST(panel_closed_does_not_capture) {
    demo_panel_t p;
    float v = 0.5f;
    demo_panel_init(&p);
    demo_panel_add_float(&p, "v", &v, 0.0f, 1.0f, NULL, NULL);
    IVP_ASSERT_FALSE(demo_panel_update(&p, WIN_W, TOP, PX + 20, track_y(), true));
    IVP_ASSERT_NEAR(v, 0.5f, 1e-6);
}

IVP_TEST(panel_slider_follows_the_mouse) {
    demo_panel_t p;
    float v = 0.5f;
    demo_panel_init(&p);
    demo_panel_add_float(&p, "v", &v, 10.0f, 20.0f, on_change, NULL);
    p.open = true;
    s_changes = 0;

    const float x0 = PX + DEMO_PANEL_PAD, x1 = PX + DEMO_PANEL_WIDTH - DEMO_PANEL_PAD;
    /* press in the middle of the track */
    IVP_ASSERT_TRUE(demo_panel_update(&p, WIN_W, TOP, (x0 + x1) * 0.5f, track_y(), true));
    IVP_ASSERT_NEAR(v, 15.0f, 1e-3);
    /* drag past the right end: clamped, and the mouse may leave the panel */
    IVP_ASSERT_TRUE(demo_panel_update(&p, WIN_W, TOP, x1 + 300.0f, track_y() + 200.0f, true));
    IVP_ASSERT_NEAR(v, 20.0f, 1e-6);
    /* drag to the far left */
    IVP_ASSERT_TRUE(demo_panel_update(&p, WIN_W, TOP, x0 - 50.0f, track_y(), true));
    IVP_ASSERT_NEAR(v, 10.0f, 1e-6);
    IVP_ASSERT_EQ(s_changes, 3);
    /* release: the slider lets go, the mouse outside is free again */
    IVP_ASSERT_FALSE(demo_panel_update(&p, WIN_W, TOP, 10.0f, 10.0f, false));
    IVP_ASSERT_EQ(p.active, -1);
    IVP_ASSERT_NEAR(v, 10.0f, 1e-6);
}

IVP_TEST(panel_checkbox_toggles_once_per_click) {
    demo_panel_t p;
    bool b = false;
    demo_panel_init(&p);
    demo_panel_add_bool(&p, "b", &b, on_change, NULL);
    p.open = true;
    s_changes = 0;

    float y = TOP + DEMO_PANEL_HEAD_H + 8.0f;
    demo_panel_update(&p, WIN_W, TOP, PX + 20, y, true);
    IVP_ASSERT_TRUE(b);
    /* holding the button does not toggle again */
    demo_panel_update(&p, WIN_W, TOP, PX + 20, y, true);
    IVP_ASSERT_TRUE(b);
    demo_panel_update(&p, WIN_W, TOP, PX + 20, y, false);
    demo_panel_update(&p, WIN_W, TOP, PX + 20, y, true);
    IVP_ASSERT_FALSE(b);
    IVP_ASSERT_EQ(s_changes, 2);
}

IVP_TEST(panel_rows_stack_and_hit_the_right_one) {
    demo_panel_t p;
    float f = 0.0f;
    bool b = false;
    demo_panel_init(&p);
    demo_panel_add_float(&p, "f", &f, 0.0f, 1.0f, NULL, NULL);
    demo_panel_add_bool(&p, "b", &b, NULL, NULL);
    p.open = true;

    /* the checkbox row starts after the float row */
    float by = TOP + DEMO_PANEL_HEAD_H + DEMO_PANEL_FLOAT_H + 8.0f;
    demo_panel_update(&p, WIN_W, TOP, PX + 20, by, true);
    IVP_ASSERT_TRUE(b);
    IVP_ASSERT_NEAR(f, 0.0f, 1e-6);
    IVP_ASSERT_EQ(p.active, -1);
    IVP_ASSERT_NEAR(p.h, DEMO_PANEL_HEAD_H + DEMO_PANEL_PAD + DEMO_PANEL_FLOAT_H
                         + DEMO_PANEL_BOOL_H, 1e-3);
}

IVP_TEST(panel_ignores_clicks_outside) {
    demo_panel_t p;
    float f = 0.25f;
    demo_panel_init(&p);
    demo_panel_add_float(&p, "f", &f, 0.0f, 1.0f, NULL, NULL);
    p.open = true;
    IVP_ASSERT_FALSE(demo_panel_update(&p, WIN_W, TOP, 100.0f, 100.0f, true));
    IVP_ASSERT_EQ(p.active, -1);
    IVP_ASSERT_NEAR(f, 0.25f, 1e-6);
}

IVP_TEST(panel_capacity_is_bounded) {
    demo_panel_t p;
    float f = 0.0f;
    demo_panel_init(&p);
    for (int i = 0; i < DEMO_PANEL_MAX_PARAMS; i++)
        IVP_ASSERT_TRUE(demo_panel_add_float(&p, "f", &f, 0.0f, 1.0f, NULL, NULL));
    IVP_ASSERT_FALSE(demo_panel_add_float(&p, "f", &f, 0.0f, 1.0f, NULL, NULL));
    IVP_ASSERT_FALSE(demo_panel_add_bool(&p, "b", (bool[]){ false }, NULL, NULL));
}

IVP_TEST(panel_rows_flow_into_a_second_column) {
    demo_panel_t p;
    float f[5] = { 0 };
    demo_panel_init(&p);
    for (int i = 0; i < 5; i++) demo_panel_add_float(&p, "f", &f[i], 0.0f, 1.0f, NULL, NULL);
    p.open = true;
    /* room for three float rows per column */
    p.max_h = DEMO_PANEL_HEAD_H + 3 * DEMO_PANEL_FLOAT_H + DEMO_PANEL_PAD;
    demo_panel_update(&p, WIN_W, TOP, 0.0f, 0.0f, false);
    IVP_ASSERT_EQ(p.n_cols, 2);
    IVP_ASSERT_EQ(p.params[2].col, 0);
    IVP_ASSERT_EQ(p.params[3].col, 1);
    IVP_ASSERT_NEAR(p.params[3].ry, DEMO_PANEL_HEAD_H, 1e-6);
    IVP_ASSERT_NEAR(p.w, 2 * DEMO_PANEL_WIDTH, 1e-6);

    /* a press in the middle of the second column's first track moves row 3 only */
    float x0, x1, ty;
    demo_panel_track(&p, &p.params[3], &x0, &x1, &ty);
    demo_panel_update(&p, WIN_W, TOP, (x0 + x1) * 0.5f, ty, true);
    IVP_ASSERT_NEAR(f[3], 0.5f, 1e-3);
    IVP_ASSERT_NEAR(f[0], 0.0f, 1e-6);
}

int main(void) {
    IVP_REGISTER(panel_closed_does_not_capture);
    IVP_REGISTER(panel_slider_follows_the_mouse);
    IVP_REGISTER(panel_checkbox_toggles_once_per_click);
    IVP_REGISTER(panel_rows_stack_and_hit_the_right_one);
    IVP_REGISTER(panel_ignores_clicks_outside);
    IVP_REGISTER(panel_rows_flow_into_a_second_column);
    IVP_REGISTER(panel_capacity_is_bounded);
    IVP_RUN_ALL();
}
