/* ivp_render_stats.c -- Frame statistics and diagnostics */

#include <ivp/render/ivp_render_stats.h>
#include <ivp/render/ivp_render_core.h>
#include <ivp/render/ivp_render_draw.h>
#include <ivp/render/ivp_render_hud.h>

void ivp_render_stats_get(const ivp_renderer_t *r, ivp_render_stats_t *stats)
{
    if (!r || !stats) return;

    ivp_render_draw_ctx_t *draw = ivp_renderer_get_draw_ctx((ivp_renderer_t *)r);
    ivp_render_hud_ctx_t  *hud  = ivp_renderer_get_hud_ctx((ivp_renderer_t *)r);

    stats->draw_vertex_count    = draw ? ivp_render_draw_ctx_count(draw) : 0;
    stats->draw_vertex_capacity = draw ? ivp_render_draw_ctx_capacity(draw) : 0;
    stats->draw_utilization     = (stats->draw_vertex_capacity > 0)
        ? (float)stats->draw_vertex_count / (float)stats->draw_vertex_capacity
        : 0.0f;

    stats->hud_vertex_count    = hud ? ivp_render_hud_ctx_count(hud) : 0;
    stats->hud_vertex_capacity = hud ? ivp_render_hud_ctx_capacity(hud) : 0;
    stats->hud_utilization     = (stats->hud_vertex_capacity > 0)
        ? (float)stats->hud_vertex_count / (float)stats->hud_vertex_capacity
        : 0.0f;

    stats->frame_dt = ivp_renderer_get_dt(r);
    stats->fps = (stats->frame_dt > 1e-6f) ? 1.0f / stats->frame_dt : 0.0f;
}
