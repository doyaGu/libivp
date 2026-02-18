/* ivp_build_q12bsp.h — Quake 1/2 BSP to compact surface builder
 *
 * Converts Quake 1/2 BSP data into IVP compact surfaces for physics.
 * Each solid leaf in the BSP tree is converted to a convex piece via
 * half-space intersection. All pieces are assembled into a compound surface.
 *
 * Ported from ivp_compact_builder/ivp_surbuild_q12.cxx.
 */
#ifndef IVP_BUILD_Q12BSP_H
#define IVP_BUILD_Q12BSP_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_surface.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── BSP data types (matching Quake format) ────────────────────────────── */

typedef struct ivp_q12_plane {
    float normal[3];
    float dist;
    int   type;         /* axial type hint (unused for physics) */
} ivp_q12_plane_t;

typedef struct ivp_q12_node {
    int      planenum;
    short    children[2]; /* >0: node index, -1: solid leaf, -2: empty leaf */
    short    mins[3];
    short    maxs[3];
    uint16_t firstface;
    uint16_t numfaces;
} ivp_q12_node_t;

typedef struct ivp_q12_clipnode {
    int   planenum;
    short children[2];
} ivp_q12_clipnode_t;

typedef struct ivp_q12_model {
    float mins[3];
    float maxs[3];
    float origin[3];
    int   headnode[4];  /* hull 0..3 entry points */
    int   visleafs;
    int   firstface;
    int   numfaces;
} ivp_q12_model_t;

/* ── Parsed BSP container ──────────────────────────────────────────────── */

typedef struct ivp_q12bsp {
    ivp_q12_model_t    *models;
    ivp_q12_plane_t    *planes;
    ivp_q12_node_t     *nodes;
    ivp_q12_clipnode_t *clipnodes;
    int n_models;
    int n_planes;
    int n_nodes;
    int n_clipnodes;
    bool owns_data;
    const ivp_allocator_t *allocator;
} ivp_q12bsp_t;

/* ── Build configuration ───────────────────────────────────────────────── */

typedef struct ivp_q12bsp_config {
    float scale;                 /* meters-per-unit, default: 1.0 */
    float shrink_value;          /* inward shrink for clipping, default: 0.0 */
    float pointmerge_threshold;  /* vertex merge distance, default: 0.01 */
} ivp_q12bsp_config_t;

/* Fill with defaults. */
void ivp_q12bsp_config_init(ivp_q12bsp_config_t *cfg);

/* ── Loading / initialization ──────────────────────────────────────────── */

/* Load BSP data from a file (Quake 1 format).
 * Returns IVP_OK on success. */
ivp_result_t ivp_q12bsp_load_file(const ivp_allocator_t *alloc,
                                  const char *filename,
                                  ivp_q12bsp_t *out);

/* Initialize BSP from pre-parsed arrays (does NOT take ownership).
 * The arrays must remain valid until ivp_q12bsp_free is called. */
void ivp_q12bsp_init_from_memory(ivp_q12bsp_t *bsp,
                                 ivp_q12_model_t *models, int n_models,
                                 ivp_q12_plane_t *planes, int n_planes,
                                 ivp_q12_node_t *nodes, int n_nodes,
                                 ivp_q12_clipnode_t *clipnodes, int n_clipnodes);

/* Free BSP data (only frees owned allocations). */
void ivp_q12bsp_free(ivp_q12bsp_t *bsp);

/* ── Building ──────────────────────────────────────────────────────────── */

/* Build a compact surface from a BSP model.
 * model_index selects which brush model to convert (0 = world).
 * cfg may be NULL for defaults. Returns NULL on failure. */
ivp_compact_surface_t *ivp_q12bsp_build_surface(const ivp_allocator_t *alloc,
                                                const ivp_q12bsp_t *bsp,
                                                int model_index,
                                                const ivp_q12bsp_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BUILD_Q12BSP_H */
