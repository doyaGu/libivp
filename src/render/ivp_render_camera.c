/* ivp_render_camera.c -- Camera orbit, pan, zoom implementation */

#include <ivp/render/ivp_render_camera.h>
#include <string.h>
#include <math.h>

void ivp_camera_config_init(ivp_camera_config_t *cfg)
{
    cfg->pitch_min        = IVP_CAMERA_PITCH_MIN;
    cfg->pitch_max        = IVP_CAMERA_PITCH_MAX;
    cfg->dist_min         = IVP_CAMERA_DIST_MIN;
    cfg->dist_max         = IVP_CAMERA_DIST_MAX;
    cfg->orbit_sensitivity = IVP_CAMERA_ORBIT_SENSITIVITY;
    cfg->pan_speed        = IVP_CAMERA_PAN_SPEED;
    cfg->key_pan_speed    = IVP_CAMERA_KEY_PAN_SPEED;
    cfg->scroll_factor    = IVP_CAMERA_SCROLL_FACTOR;
    cfg->fly_speed        = 5.0f;
    cfg->fly_sensitivity  = 0.3f;
    cfg->ortho_size       = 10.0f;
}

void ivp_camera_init_ex(ivp_camera_t *cam, const ivp_camera_config_t *config)
{
    memset(cam, 0, sizeof(*cam));
    cam->target[1]      = -3.0f;
    cam->up[1]          = -1.0f;
    cam->fov_deg        = 60.0f;
    cam->near_plane     = 0.1f;
    cam->far_plane      = 500.0f;
    cam->orbit_distance = 20.0f;
    cam->orbit_yaw      = 45.0f;
    cam->orbit_pitch    = -30.0f;

    if (config) {
        cam->config = *config;
    } else {
        ivp_camera_config_init(&cam->config);
    }

    cam->mode       = IVP_CAMERA_MODE_ORBIT;
    cam->projection = IVP_CAMERA_PROJ_PERSPECTIVE;
    cam->ortho_size = cam->config.ortho_size;
    cam->fly_speed  = cam->config.fly_speed;
    cam->fly_yaw    = 0.0f;
    cam->fly_pitch  = 0.0f;
}

void ivp_camera_init(ivp_camera_t *cam)
{
    ivp_camera_init_ex(cam, NULL);
}

void ivp_camera_input_init(ivp_camera_input_t *input)
{
    memset(input, 0, sizeof(*input));
}

void ivp_camera_update_orbit(ivp_camera_t *cam)
{
    if (cam->mode != IVP_CAMERA_MODE_ORBIT) return;

    float yr = cam->orbit_yaw   * IVP_RENDER_PI_F / 180.0f;
    float pr = cam->orbit_pitch * IVP_RENDER_PI_F / 180.0f;
    float cp = cosf(pr), sp = sinf(pr);
    float cy = cosf(yr), sy = sinf(yr);

    /* IVP uses Y-down: eye position relative to target */
    cam->eye[0] = cam->target[0] + cam->orbit_distance * cp * sy;
    cam->eye[1] = cam->target[1] + cam->orbit_distance * sp;
    cam->eye[2] = cam->target[2] + cam->orbit_distance * cp * cy;
}

void ivp_camera_update_fly(ivp_camera_t *cam)
{
    if (cam->mode != IVP_CAMERA_MODE_FLY) return;

    float yr = cam->fly_yaw   * IVP_RENDER_PI_F / 180.0f;
    float pr = cam->fly_pitch * IVP_RENDER_PI_F / 180.0f;
    float cp = cosf(pr);

    /* Forward direction from yaw/pitch */
    float fwd[3] = {
        -sinf(yr) * cp,
         sinf(pr),
        -cosf(yr) * cp
    };

    cam->target[0] = cam->eye[0] + fwd[0];
    cam->target[1] = cam->eye[1] + fwd[1];
    cam->target[2] = cam->eye[2] + fwd[2];
}

bool ivp_camera_process_event(ivp_camera_t *cam,
                              ivp_camera_input_t *input,
                              const SDL_Event *ev,
                              int *out_w, int *out_h)
{
    bool resized = false;

    /* Keyboard pan state */
    if (ev->type == SDL_EVENT_KEY_DOWN || ev->type == SDL_EVENT_KEY_UP) {
        bool down = (ev->type == SDL_EVENT_KEY_DOWN);
        switch (ev->key.key) {
        case SDLK_W: input->key_w = down; break;
        case SDLK_A: input->key_a = down; break;
        case SDLK_S: input->key_s = down; break;
        case SDLK_D: input->key_d = down; break;
        case SDLK_Q: input->key_q = down; break;
        case SDLK_E: input->key_e = down; break;
        case SDLK_R: input->key_r = down; break;
        case SDLK_F: input->key_f = down; break;
        default: break;
        }
    }

    /* Mouse orbit (right button) */
    if (ev->type == SDL_EVENT_MOUSE_BUTTON_DOWN && ev->button.button == SDL_BUTTON_RIGHT) {
        input->rmouse_down = true;
        input->last_mx = ev->button.x;
        input->last_my = ev->button.y;
    }
    if (ev->type == SDL_EVENT_MOUSE_BUTTON_UP && ev->button.button == SDL_BUTTON_RIGHT) {
        input->rmouse_down = false;
    }

    /* Mouse pan (middle button) */
    if (ev->type == SDL_EVENT_MOUSE_BUTTON_DOWN && ev->button.button == SDL_BUTTON_MIDDLE) {
        input->mouse_down = true;
        input->last_mx = ev->button.x;
        input->last_my = ev->button.y;
    }
    if (ev->type == SDL_EVENT_MOUSE_BUTTON_UP && ev->button.button == SDL_BUTTON_MIDDLE) {
        input->mouse_down = false;
    }

    if (ev->type == SDL_EVENT_MOUSE_MOTION && cam) {
        float dx = ev->motion.x - input->last_mx;
        float dy = ev->motion.y - input->last_my;
        input->last_mx = ev->motion.x;
        input->last_my = ev->motion.y;

        if (cam->mode == IVP_CAMERA_MODE_FLY) {
            if (input->rmouse_down) {
                cam->fly_yaw   += dx * cam->config.fly_sensitivity;
                cam->fly_pitch += dy * cam->config.fly_sensitivity;
                if (cam->fly_pitch > cam->config.pitch_max)
                    cam->fly_pitch = cam->config.pitch_max;
                if (cam->fly_pitch < cam->config.pitch_min)
                    cam->fly_pitch = cam->config.pitch_min;
            }
        } else {
            if (input->rmouse_down) {
                cam->orbit_yaw   += dx * cam->config.orbit_sensitivity;
                cam->orbit_pitch += dy * cam->config.orbit_sensitivity;
                if (cam->orbit_pitch > cam->config.pitch_max)
                    cam->orbit_pitch = cam->config.pitch_max;
                if (cam->orbit_pitch < cam->config.pitch_min)
                    cam->orbit_pitch = cam->config.pitch_min;
            }
            if (input->mouse_down) {
                float yr = cam->orbit_yaw * IVP_RENDER_PI_F / 180.0f;
                float right[3] = { cosf(yr), 0, -sinf(yr) };
                float pan_speed = cam->orbit_distance * cam->config.pan_speed;
                cam->target[0] -= right[0] * dx * pan_speed;
                cam->target[2] -= right[2] * dx * pan_speed;
                cam->target[1] -= dy * pan_speed;
            }
        }
    }

    if (ev->type == SDL_EVENT_MOUSE_WHEEL && cam) {
        if (cam->mode == IVP_CAMERA_MODE_FLY) {
            /* Scroll adjusts fly speed */
            cam->fly_speed += ev->wheel.y * cam->fly_speed * cam->config.scroll_factor;
            if (cam->fly_speed < 0.1f) cam->fly_speed = 0.1f;
            if (cam->fly_speed > 200.0f) cam->fly_speed = 200.0f;
        } else {
            cam->orbit_distance -= ev->wheel.y * cam->orbit_distance * cam->config.scroll_factor;
            if (cam->orbit_distance < cam->config.dist_min)
                cam->orbit_distance = cam->config.dist_min;
            if (cam->orbit_distance > cam->config.dist_max)
                cam->orbit_distance = cam->config.dist_max;
        }
    }

    if (ev->type == SDL_EVENT_WINDOW_RESIZED) {
        if (out_w) *out_w = ev->window.data1;
        if (out_h) *out_h = ev->window.data2;
        resized = true;
    }

    return resized;
}

void ivp_camera_apply_key_pan(ivp_camera_t *cam,
                              const ivp_camera_input_t *input)
{
    if (!cam) return;

    if (cam->mode == IVP_CAMERA_MODE_FLY) {
        float speed = cam->fly_speed * cam->config.key_pan_speed;
        float yr = cam->fly_yaw   * IVP_RENDER_PI_F / 180.0f;
        float pr = cam->fly_pitch * IVP_RENDER_PI_F / 180.0f;
        float cp = cosf(pr);
        float fwd[3]   = { -sinf(yr)*cp, sinf(pr), -cosf(yr)*cp };
        float right[3] = { -cosf(yr), 0, sinf(yr) };

        if (input->key_w) { cam->eye[0] += fwd[0]*speed; cam->eye[1] += fwd[1]*speed; cam->eye[2] += fwd[2]*speed; }
        if (input->key_s) { cam->eye[0] -= fwd[0]*speed; cam->eye[1] -= fwd[1]*speed; cam->eye[2] -= fwd[2]*speed; }
        if (input->key_a) { cam->eye[0] -= right[0]*speed; cam->eye[2] -= right[2]*speed; }
        if (input->key_d) { cam->eye[0] += right[0]*speed; cam->eye[2] += right[2]*speed; }
        if (input->key_q) { cam->eye[1] -= speed; }
        if (input->key_e) { cam->eye[1] += speed; }
        if (input->key_r) { cam->eye[1] -= speed; }
        if (input->key_f) { cam->eye[1] += speed; }
    } else {
        float pan_speed = cam->orbit_distance * cam->config.key_pan_speed;
        float yr = cam->orbit_yaw * IVP_RENDER_PI_F / 180.0f;
        float fwd[3]   = { -sinf(yr), 0, -cosf(yr) };
        float right[3] = { -cosf(yr), 0,  sinf(yr) };

        if (input->key_w) { cam->target[0] += fwd[0]*pan_speed; cam->target[2] += fwd[2]*pan_speed; }
        if (input->key_s) { cam->target[0] -= fwd[0]*pan_speed; cam->target[2] -= fwd[2]*pan_speed; }
        if (input->key_a) { cam->target[0] -= right[0]*pan_speed; cam->target[2] -= right[2]*pan_speed; }
        if (input->key_d) { cam->target[0] += right[0]*pan_speed; cam->target[2] += right[2]*pan_speed; }
        if (input->key_q) { cam->target[1] -= pan_speed; }
        if (input->key_e) { cam->target[1] += pan_speed; }
        if (input->key_r) { cam->target[1] -= pan_speed; }
        if (input->key_f) { cam->target[1] += pan_speed; }
    }
}

void ivp_camera_compute_matrices(const ivp_camera_t *cam,
                                 int width, int height,
                                 float view_mat[16],
                                 float proj_mat[16],
                                 float vp_mat[16])
{
    if (cam) {
        ivp_render_mat4_lookat(view_mat, cam->eye, cam->target, cam->up);
    } else {
        ivp_render_mat4_identity(view_mat);
    }

    float aspect = (height > 0) ? (float)width / (float)height : 1.0f;
    float zn  = cam ? cam->near_plane : 0.1f;
    float zf  = cam ? cam->far_plane : 500.0f;

    if (cam && cam->projection == IVP_CAMERA_PROJ_ORTHOGRAPHIC) {
        float half_h = cam->ortho_size;
        float half_w = half_h * aspect;
        ivp_render_mat4_ortho(proj_mat,
                              -half_w, half_w,
                              -half_h, half_h,
                              zn, zf);
    } else {
        float fov = cam ? cam->fov_deg : 60.0f;
        ivp_render_mat4_perspective(proj_mat, fov * IVP_RENDER_PI_F / 180.0f, aspect, zn, zf);
    }

    ivp_render_mat4_mul(vp_mat, proj_mat, view_mat);
}
