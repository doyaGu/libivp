/* ivp_render_stats.h -- Frame statistics and diagnostics */
#ifndef IVP_RENDER_STATS_H
#define IVP_RENDER_STATS_H

#include <ivp/render/ivp_render_types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ivp_render_stats {
    /* Draw context stats */
    int   draw_vertex_count;     /* vertices submitted this frame */
    int   draw_vertex_capacity;  /* max capacity */
    float draw_utilization;      /* count / capacity (0.0-1.0) */
    /* HUD context stats */
    int   hud_vertex_count;
    int   hud_vertex_capacity;
    float hud_utilization;
    /* Frame timing */
    float frame_dt;              /* last frame delta (seconds) */
    float fps;                   /* 1.0 / frame_dt */
} ivp_render_stats_t;

/* Snapshot current frame statistics. Call after end_frame(). */
void ivp_render_stats_get(const ivp_renderer_t *r, ivp_render_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RENDER_STATS_H */
