/* ivp_render_camera.h -- Camera struct, orbit controls, input processing */
#ifndef IVP_RENDER_CAMERA_H
#define IVP_RENDER_CAMERA_H

#include <ivp/render/ivp_render_math.h>
#include <stdbool.h>
#include <SDL3/SDL_events.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Camera constants ──────────────────────────────────────────────────── */

#define IVP_CAMERA_PITCH_MIN          (-89.0f)
#define IVP_CAMERA_PITCH_MAX          ( 89.0f)
#define IVP_CAMERA_DIST_MIN           0.5f
#define IVP_CAMERA_DIST_MAX           200.0f
#define IVP_CAMERA_ORBIT_SENSITIVITY  0.5f
#define IVP_CAMERA_PAN_SPEED          0.003f
#define IVP_CAMERA_KEY_PAN_SPEED      0.02f
#define IVP_CAMERA_SCROLL_FACTOR      0.1f

/* ── Camera mode / projection enums ───────────────────────────────────── */

typedef enum ivp_camera_mode {
    IVP_CAMERA_MODE_ORBIT = 0,  /* existing behavior (default) */
    IVP_CAMERA_MODE_FLY   = 1   /* WASD + mouse-look free camera */
} ivp_camera_mode_t;

typedef enum ivp_camera_projection {
    IVP_CAMERA_PROJ_PERSPECTIVE  = 0,  /* default */
    IVP_CAMERA_PROJ_ORTHOGRAPHIC = 1
} ivp_camera_projection_t;

/* ── Camera config ────────────────────────────────────────────────────── */

typedef struct ivp_camera_config {
    float pitch_min;
    float pitch_max;
    float dist_min;
    float dist_max;
    float orbit_sensitivity;
    float pan_speed;
    float key_pan_speed;
    float scroll_factor;
    float fly_speed;        /* default: 5.0 */
    float fly_sensitivity;  /* default: 0.3 */
    float ortho_size;       /* default: 10.0 */
} ivp_camera_config_t;

/* ── Camera struct ─────────────────────────────────────────────────────── */

typedef struct ivp_camera ivp_camera_t;

struct ivp_camera {
    float eye[3];
    float target[3];
    float up[3];
    float fov_deg;
    float near_plane;
    float far_plane;
    /* orbit controls */
    float orbit_distance;
    float orbit_yaw;
    float orbit_pitch;
    /* runtime config */
    ivp_camera_config_t     config;
    /* mode and projection */
    ivp_camera_mode_t       mode;         /* default: ORBIT */
    ivp_camera_projection_t projection;   /* default: PERSPECTIVE */
    float                   ortho_size;   /* half-height for ortho */
    /* fly mode state */
    float                   fly_speed;    /* units/sec */
    float                   fly_yaw;      /* horizontal look angle (degrees) */
    float                   fly_pitch;    /* vertical look angle (degrees) */
};

/* ── Camera input state ────────────────────────────────────────────────── */

typedef struct ivp_camera_input {
    bool mouse_down;    /* middle button (pan) */
    bool rmouse_down;   /* right button (orbit) */
    float last_mx, last_my;
    bool key_w, key_a, key_s, key_d;
    bool key_q, key_e, key_r, key_f;
} ivp_camera_input_t;

/* ── Functions ─────────────────────────────────────────────────────────── */

void ivp_camera_config_init(ivp_camera_config_t *cfg);
void ivp_camera_init(ivp_camera_t *cam);
void ivp_camera_init_ex(ivp_camera_t *cam, const ivp_camera_config_t *config);
void ivp_camera_input_init(ivp_camera_input_t *input);
void ivp_camera_update_orbit(ivp_camera_t *cam);
void ivp_camera_update_fly(ivp_camera_t *cam);

/* Process a single SDL event for camera orbit/pan/zoom.
 * Returns true if the event was a window resize (new size in *out_w, *out_h). */
bool ivp_camera_process_event(ivp_camera_t *cam,
                              ivp_camera_input_t *input,
                              const SDL_Event *ev,
                              int *out_w, int *out_h);

/* Apply WASD/QE/RF key pan to camera target. */
void ivp_camera_apply_key_pan(ivp_camera_t *cam,
                              const ivp_camera_input_t *input);

/* Compute view + projection matrices from camera state. */
void ivp_camera_compute_matrices(const ivp_camera_t *cam,
                                 int width, int height,
                                 float view_mat[16],
                                 float proj_mat[16],
                                 float vp_mat[16]);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_CAMERA_H */
