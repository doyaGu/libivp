#include "interaction.h"

#include <SDL3/SDL.h>

#include <ivp/base/ivp_types.h>
#include <ivp/controllers/ivp_controller.h>
#include <ivp/math/ivp_vec3.h>

#include <math.h>
#include <stdlib.h>

static void drag_controller_apply(const ivp_event_sim_t *event_sim,
                                  ivp_object_t *obj,
                                  void *ctx)
{
    ivp_environment_t *env = event_sim->environment;
    const ivp_real_t dt = event_sim->delta_time;
    ivp_example_dragger_t *dragger = (ivp_example_dragger_t *)ctx;
    if (!dragger || !dragger->active || obj != dragger->target) return;
    if (obj->physical_core->physical_unmoveable) return;

    ivp_vec3r_t dv;
    ivp_vec3r_scale(&dv, &dragger->force, obj->physical_core->inv_mass * dt);
    ivp_vec3r_add(&obj->physical_core->speed_change, &obj->physical_core->speed_change, &dv);
}

static bool ray_sphere_hit(const float ro[3], const float rd[3],
                           const float c[3], float r, float *t_hit)
{
    float ocx = ro[0] - c[0];
    float ocy = ro[1] - c[1];
    float ocz = ro[2] - c[2];

    float b = ocx * rd[0] + ocy * rd[1] + ocz * rd[2];
    float cterm = ocx * ocx + ocy * ocy + ocz * ocz - r * r;
    float disc = b * b - cterm;
    if (disc < 0.0f) return false;

    float s = sqrtf(disc);
    float t0 = -b - s;
    float t1 = -b + s;
    float t = (t0 > 0.0f) ? t0 : t1;
    if (t <= 0.0f) return false;
    *t_hit = t;
    return true;
}

static bool screen_ray_from_camera(const ivp_camera_t *cam,
                                   int width,
                                   int height,
                                   float mx,
                                   float my,
                                   float ro[3],
                                   float rd[3])
{
    if (!cam || width <= 0 || height <= 0) return false;

    float fwd[3] = {
        cam->target[0] - cam->eye[0],
        cam->target[1] - cam->eye[1],
        cam->target[2] - cam->eye[2]
    };
    float fwd_len = sqrtf(fwd[0]*fwd[0] + fwd[1]*fwd[1] + fwd[2]*fwd[2]);
    if (fwd_len < 1e-6f) return false;
    fwd[0] /= fwd_len; fwd[1] /= fwd_len; fwd[2] /= fwd_len;

    float right[3] = {
        fwd[1] * cam->up[2] - fwd[2] * cam->up[1],
        fwd[2] * cam->up[0] - fwd[0] * cam->up[2],
        fwd[0] * cam->up[1] - fwd[1] * cam->up[0]
    };
    float right_len = sqrtf(right[0]*right[0] + right[1]*right[1] + right[2]*right[2]);
    if (right_len < 1e-6f) return false;
    right[0] /= right_len; right[1] /= right_len; right[2] /= right_len;

    float up[3] = {
        right[1] * fwd[2] - right[2] * fwd[1],
        right[2] * fwd[0] - right[0] * fwd[2],
        right[0] * fwd[1] - right[1] * fwd[0]
    };

    float aspect = (float)width / (float)height;
    float tan_half_fov = tanf(cam->fov_deg * (3.14159265358979323846f / 180.0f) * 0.5f);
    float nx = (2.0f * mx / (float)width) - 1.0f;
    float ny = 1.0f - (2.0f * my / (float)height);

    ro[0] = cam->eye[0];
    ro[1] = cam->eye[1];
    ro[2] = cam->eye[2];

    rd[0] = fwd[0] + right[0] * nx * aspect * tan_half_fov + up[0] * ny * tan_half_fov;
    rd[1] = fwd[1] + right[1] * nx * aspect * tan_half_fov + up[1] * ny * tan_half_fov;
    rd[2] = fwd[2] + right[2] * nx * aspect * tan_half_fov + up[2] * ny * tan_half_fov;

    float rd_len = sqrtf(rd[0]*rd[0] + rd[1]*rd[1] + rd[2]*rd[2]);
    if (rd_len < 1e-6f) return false;
    rd[0] /= rd_len; rd[1] /= rd_len; rd[2] /= rd_len;
    return true;
}

static void dragger_clear(ivp_example_dragger_t *dragger)
{
    dragger->active = false;
    dragger->target = NULL;
    dragger->has_target_point = false;
    dragger->depth = 0.0f;
    dragger->ramp = 0.0f;
    ivp_vec3r_set_zero(&dragger->force);
}

void ivp_example_dragger_init(ivp_example_dragger_t *dragger)
{
    dragger->controller_handle.slot = UINT32_MAX;
    dragger->controller_handle.generation = 0;
    dragger->prev_lmb_down = false;
    dragger->lmb_seeded = false;
    dragger->settings.pick_radius_scale = 1.15f;
    dragger->settings.min_depth = 0.5f;
    dragger->settings.stiffness = 22.0f;
    dragger->settings.damping = 8.5f;
    dragger->settings.max_force = 70.0f;
    dragger->settings.ramp_rate = 4.0f;
    dragger_clear(dragger);
}

bool ivp_example_dragger_attach(ivp_example_dragger_t *dragger,
                                ivp_environment_t *env,
                                int priority)
{
    if (!dragger || !env) return false;
    if (dragger->controller_handle.slot != UINT32_MAX &&
        dragger->controller_handle.generation != 0) {
        return true;
    }
    dragger->controller_handle =
        ivp_environment_add_controller(env, priority, drag_controller_apply, dragger);
    return dragger->controller_handle.slot != UINT32_MAX &&
           dragger->controller_handle.generation != 0;
}

void ivp_example_dragger_detach(ivp_example_dragger_t *dragger,
                                ivp_environment_t *env)
{
    if (!dragger || !env) return;
    if (dragger->controller_handle.slot != UINT32_MAX &&
        dragger->controller_handle.generation != 0) {
        ivp_environment_remove_controller(env, dragger->controller_handle);
    }
    dragger->controller_handle.slot = UINT32_MAX;
    dragger->controller_handle.generation = 0;
    dragger_clear(dragger);
}

void ivp_example_dragger_update(ivp_example_dragger_t *dragger,
                                const ivp_camera_t *cam,
                                ivp_environment_t *env,
                                int window_width,
                                int window_height,
                                float frame_dt)
{
    if (!dragger || !cam || !env) return;

    static int drag_disabled = -1;
    if (drag_disabled < 0) {
        const char *e = getenv("IVP_DISABLE_DRAG");
        drag_disabled = (e && e[0] && e[0] != '0') ? 1 : 0;
    }
    if (drag_disabled) {
        dragger_clear(dragger);
        return;
    }

    float mx = 0.0f, my = 0.0f;
    SDL_MouseButtonFlags buttons = SDL_GetMouseState(&mx, &my);
    bool lmb_down = (buttons & SDL_BUTTON_MASK(SDL_BUTTON_LEFT)) != 0;

    if (!dragger->lmb_seeded) {
        dragger->prev_lmb_down = lmb_down;
        dragger->lmb_seeded = true;
    }

    float ray_o[3], ray_d[3];
    bool ray_ok = screen_ray_from_camera(cam, window_width, window_height, mx, my, ray_o, ray_d);

    if (lmb_down && !dragger->prev_lmb_down) {
        dragger_clear(dragger);

        if (ray_ok) {
            float best_t = 1e30f;
            ivp_object_t *best_obj = NULL;

            int object_count = ivp_environment_get_object_count(env);
            for (int i = 0; i < object_count; i++) {
                ivp_object_t *obj = ivp_environment_get_object(env, i);
                if (!obj || obj->physical_core->physical_unmoveable || obj->physical_core->inv_mass <= IVP_REAL(0.0)) {
                    continue;
                }

                float center[3] = {
                    (float)obj->physical_core->pos_world_f_core_last_psi.k[0],
                    (float)obj->physical_core->pos_world_f_core_last_psi.k[1],
                    (float)obj->physical_core->pos_world_f_core_last_psi.k[2]
                };
                float radius = (float)obj->physical_core->upper_limit_radius;
                if (radius < 0.05f) radius = 0.05f;
                radius *= dragger->settings.pick_radius_scale;

                float t_hit = 0.0f;
                if (ray_sphere_hit(ray_o, ray_d, center, radius, &t_hit) && t_hit < best_t) {
                    best_t = t_hit;
                    best_obj = obj;
                }
            }

            if (best_obj) {
                dragger->active = true;
                dragger->target = best_obj;
                dragger->depth = (best_t > dragger->settings.min_depth)
                    ? best_t
                    : dragger->settings.min_depth;
                dragger->ramp = 0.0f;
            }
        }
    }

    if (!lmb_down && dragger->prev_lmb_down) {
        dragger_clear(dragger);
    }

    if (dragger->active && lmb_down && ray_ok && dragger->target) {
        dragger->target_ws[0] = ray_o[0] + ray_d[0] * dragger->depth;
        dragger->target_ws[1] = ray_o[1] + ray_d[1] * dragger->depth;
        dragger->target_ws[2] = ray_o[2] + ray_d[2] * dragger->depth;
        dragger->has_target_point = true;

        ivp_vec3r_t p, v;
        ivp_object_get_position(dragger->target, &p);
        ivp_object_get_speed(dragger->target, &v);

        float ex = dragger->target_ws[0] - (float)p.k[0];
        float ey = dragger->target_ws[1] - (float)p.k[1];
        float ez = dragger->target_ws[2] - (float)p.k[2];

        float fx = dragger->settings.stiffness * ex - dragger->settings.damping * (float)v.k[0];
        float fy = dragger->settings.stiffness * ey - dragger->settings.damping * (float)v.k[1];
        float fz = dragger->settings.stiffness * ez - dragger->settings.damping * (float)v.k[2];

        dragger->ramp += frame_dt * dragger->settings.ramp_rate;
        if (dragger->ramp > 1.0f) dragger->ramp = 1.0f;
        fx *= dragger->ramp;
        fy *= dragger->ramp;
        fz *= dragger->ramp;

        float fmag = sqrtf(fx*fx + fy*fy + fz*fz);
        if (fmag > dragger->settings.max_force && fmag > 1e-6f) {
            float s = dragger->settings.max_force / fmag;
            fx *= s; fy *= s; fz *= s;
        }

        ivp_vec3r_set(&dragger->force, (ivp_real_t)fx, (ivp_real_t)fy, (ivp_real_t)fz);
    } else {
        dragger->has_target_point = false;
        ivp_vec3r_set_zero(&dragger->force);
    }

    dragger->prev_lmb_down = lmb_down;
}

void ivp_example_focus_init(ivp_example_focus_controller_t *focus)
{
    focus->follow_enabled = true;
    focus->prev_tab_down = false;
    focus->prev_c_down = false;
    focus->prev_esc_down = false;
    focus->focused = NULL;
    focus->follow_lerp = 7.0f;
    focus->vertical_offset = 0.0f;
}

static ivp_object_t *find_first_dynamic(ivp_environment_t *env)
{
    int object_count = ivp_environment_get_object_count(env);
    for (int i = 0; i < object_count; i++) {
        ivp_object_t *obj = ivp_environment_get_object(env, i);
        if (obj && !obj->physical_core->physical_unmoveable && obj->physical_core->inv_mass > IVP_REAL(0.0)) {
            return obj;
        }
    }
    return NULL;
}

static ivp_object_t *find_next_dynamic(ivp_environment_t *env,
                                       ivp_object_t *current,
                                       int step)
{
    int object_count = ivp_environment_get_object_count(env);
    if (object_count <= 0) return NULL;

    int current_index = -1;
    for (int i = 0; i < object_count; i++) {
        if (ivp_environment_get_object(env, i) == current) {
            current_index = i;
            break;
        }
    }

    int start = (current_index >= 0) ? current_index : 0;
    for (int n = 1; n <= object_count; n++) {
        int idx = (start + step * n) % object_count;
        if (idx < 0) idx += object_count;
        ivp_object_t *obj = ivp_environment_get_object(env, idx);
        if (obj && !obj->physical_core->physical_unmoveable && obj->physical_core->inv_mass > IVP_REAL(0.0)) {
            return obj;
        }
    }

    return NULL;
}

void ivp_example_focus_update(ivp_example_focus_controller_t *focus,
                              ivp_camera_t *cam,
                              ivp_environment_t *env,
                              float frame_dt)
{
    if (!focus || !cam || !env) return;

    const bool *keys = SDL_GetKeyboardState(NULL);
    bool tab_down = keys[SDL_SCANCODE_TAB];
    bool c_down = keys[SDL_SCANCODE_C];
    bool esc_down = keys[SDL_SCANCODE_ESCAPE];
    bool shift_down = keys[SDL_SCANCODE_LSHIFT] || keys[SDL_SCANCODE_RSHIFT];

    if (tab_down && !focus->prev_tab_down) {
        int step = shift_down ? -1 : 1;
        if (!focus->focused) {
            focus->focused = find_first_dynamic(env);
        } else {
            focus->focused = find_next_dynamic(env, focus->focused, step);
        }
    }

    if (esc_down && !focus->prev_esc_down && focus->focused) {
        focus->focused = NULL;
    }

    if (c_down && !focus->prev_c_down) {
        focus->follow_enabled = !focus->follow_enabled;
    }

    if (focus->focused && focus->follow_enabled) {
        ivp_vec3r_t p;
        ivp_object_get_position(focus->focused, &p);
        float target[3] = {
            (float)p.k[0],
            (float)p.k[1] + focus->vertical_offset,
            (float)p.k[2]
        };

        float alpha = frame_dt * focus->follow_lerp;
        if (alpha > 1.0f) alpha = 1.0f;
        cam->target[0] += (target[0] - cam->target[0]) * alpha;
        cam->target[1] += (target[1] - cam->target[1]) * alpha;
        cam->target[2] += (target[2] - cam->target[2]) * alpha;
    }

    focus->prev_tab_down = tab_down;
    focus->prev_c_down = c_down;
    focus->prev_esc_down = esc_down;
}

void ivp_example_input_tracker_init(ivp_example_input_tracker_t *tracker)
{
    tracker->prev_space_down = false;
    tracker->prev_home_down = false;
    tracker->scene_reset_requested = false;
    tracker->view_reset_requested = false;
}

void ivp_example_input_tracker_update(ivp_example_input_tracker_t *tracker)
{
    const bool *keys = SDL_GetKeyboardState(NULL);
    bool space_down = keys[SDL_SCANCODE_SPACE];
    bool home_down = keys[SDL_SCANCODE_HOME];

    tracker->scene_reset_requested = (space_down && !tracker->prev_space_down);
    tracker->view_reset_requested = (home_down && !tracker->prev_home_down);

    tracker->prev_space_down = space_down;
    tracker->prev_home_down = home_down;
}

void ivp_example_interaction_draw_overlays(ivp_renderer_t *renderer,
                                           const ivp_example_dragger_t *dragger,
                                           const ivp_example_focus_controller_t *focus,
                                           const float drag_color[3],
                                           const float focus_color[3])
{
    if (!renderer) return;

    if (dragger && dragger->active && dragger->target && dragger->has_target_point) {
        ivp_vec3r_t p;
        ivp_object_get_position(dragger->target, &p);
        float from[3] = {(float)p.k[0], (float)p.k[1], (float)p.k[2]};
        ivp_render_draw_arrow(renderer, from, dragger->target_ws, drag_color, 0.25f);
    }

    if (focus && focus->focused) {
        ivp_vec3r_t p;
        ivp_object_get_position(focus->focused, &p);
        float pos[3] = {(float)p.k[0], (float)p.k[1], (float)p.k[2]};
        float hs[3] = {0.6f, 0.6f, 0.6f};
        ivp_render_draw_wire_box(renderer, pos, hs, NULL, focus_color);
    }
}

const char *ivp_example_interaction_hud_primary_line(void)
{
    return "WASD+QE/RF:pan  RMB:orbit  MMB:pan  Scroll:zoom";
}

const char *ivp_example_interaction_hud_secondary_line(void)
{
    return "LMB hold:drag  Tab/Shift+Tab:focus  C:follow  Esc:unfocus";
}

const char *ivp_example_interaction_hud_tertiary_line(void)
{
    return "Space:reset scene  Home:reset view";
}

void ivp_example_draw_interaction_hud(ivp_renderer_t *renderer,
                                      float x,
                                      float y,
                                      const float primary_color[3],
                                      const float secondary_color[3])
{
    if (!renderer) return;
    ivp_render_draw_text(renderer, x, y,
        ivp_example_interaction_hud_primary_line(), primary_color);
    ivp_render_draw_text(renderer, x, y + 18.0f,
        ivp_example_interaction_hud_secondary_line(), secondary_color);
    ivp_render_draw_text(renderer, x, y + 36.0f,
        ivp_example_interaction_hud_tertiary_line(), secondary_color);
}
