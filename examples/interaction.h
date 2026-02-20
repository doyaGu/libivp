/* interaction.h -- Shared interaction helpers for IVP examples */
#ifndef IVP_EXAMPLE_INTERACTION_H
#define IVP_EXAMPLE_INTERACTION_H

#include <ivp/render/ivp_render.h>

#include <ivp/dynamics/ivp_environment.h>
#include <ivp/dynamics/ivp_handle.h>
#include <ivp/dynamics/ivp_object.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ivp_example_drag_settings {
    float pick_radius_scale;
    float min_depth;
    float stiffness;
    float damping;
    float max_force;
    float ramp_rate;
} ivp_example_drag_settings_t;

typedef struct ivp_example_dragger {
    bool active;
    bool has_target_point;
    bool prev_lmb_down;
    bool lmb_seeded;

    ivp_object_t *target;
    ivp_vec3r_t   force;
    float         target_ws[3];
    float         depth;
    float         ramp;

    ivp_controller_handle_t controller_handle;
    ivp_example_drag_settings_t settings;
} ivp_example_dragger_t;

typedef struct ivp_example_focus_controller {
    bool follow_enabled;
    bool prev_tab_down;
    bool prev_c_down;
    bool prev_esc_down;

    ivp_object_t *focused;
    float follow_lerp;
    float vertical_offset;
} ivp_example_focus_controller_t;

/* ── Input tracker for scene/view reset ───────────────────────────────── */

typedef struct ivp_example_input_tracker {
    bool prev_space_down;
    bool prev_home_down;
    bool scene_reset_requested;
    bool view_reset_requested;
} ivp_example_input_tracker_t;

void ivp_example_dragger_init(ivp_example_dragger_t *dragger);
bool ivp_example_dragger_attach(ivp_example_dragger_t *dragger,
                                ivp_environment_t *env,
                                int priority);
void ivp_example_dragger_detach(ivp_example_dragger_t *dragger,
                                ivp_environment_t *env);

void ivp_example_dragger_update(ivp_example_dragger_t *dragger,
                                const ivp_camera_t *cam,
                                ivp_environment_t *env,
                                int window_width,
                                int window_height,
                                float frame_dt);

void ivp_example_focus_init(ivp_example_focus_controller_t *focus);
void ivp_example_focus_update(ivp_example_focus_controller_t *focus,
                              ivp_camera_t *cam,
                              ivp_environment_t *env,
                              float frame_dt);

void ivp_example_input_tracker_init(ivp_example_input_tracker_t *tracker);
void ivp_example_input_tracker_update(ivp_example_input_tracker_t *tracker);

void ivp_example_interaction_draw_overlays(ivp_renderer_t *renderer,
                                           const ivp_example_dragger_t *dragger,
                                           const ivp_example_focus_controller_t *focus,
                                           const float drag_color[3],
                                           const float focus_color[3]);

const char *ivp_example_interaction_hud_primary_line(void);
const char *ivp_example_interaction_hud_secondary_line(void);
const char *ivp_example_interaction_hud_tertiary_line(void);

void ivp_example_draw_interaction_hud(ivp_renderer_t *renderer,
                                      float x,
                                      float y,
                                      const float primary_color[3],
                                      const float secondary_color[3]);

#ifdef __cplusplus
}
#endif

#endif /* IVP_EXAMPLE_INTERACTION_H */
