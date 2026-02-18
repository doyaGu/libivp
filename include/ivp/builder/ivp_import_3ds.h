/* ivp_import_3ds.h — 3DS file import and surface builder
 *
 * Parses Autodesk 3DS binary format (chunk-based) and builds compact
 * surfaces suitable for physics simulation.
 *
 * Supports loading from file or from an in-memory buffer.
 * Build step uses ivp_build_concave_surface() for mesh → compound surface.
 *
 * Ported from ivp_compact_builder/3dsimport_load.cxx and 3dsimport_co.cxx.
 */
#ifndef IVP_IMPORT_3DS_H
#define IVP_IMPORT_3DS_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/base/ivp_allocator.h>
#include <ivp/geometry/ivp_compact_surface.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Parsed mesh data ──────────────────────────────────────────────────── */

typedef struct ivp_3ds_mesh {
    float  *vertices;       /* [n_vertices * 3] */
    int    *indices;        /* [n_triangles * 3] */
    int     n_vertices;
    int     n_triangles;
    char    name[64];
} ivp_3ds_mesh_t;

typedef struct ivp_3ds_scene {
    ivp_3ds_mesh_t *meshes;
    int             n_meshes;
    const ivp_allocator_t *allocator;
} ivp_3ds_scene_t;

/* ── Import configuration ──────────────────────────────────────────────── */

typedef struct ivp_3ds_import_config {
    float scale;              /* default: 1.0 */
    float weld_epsilon;       /* default: 1e-4 */
    bool  remove_degenerate;  /* default: true */
    bool  center_meshes;      /* default: false */
} ivp_3ds_import_config_t;

/* Fill config with defaults. */
void ivp_3ds_import_config_init(ivp_3ds_import_config_t *cfg);

/* ── Loading ───────────────────────────────────────────────────────────── */

/* Load a 3DS file from disk.
 * Returns IVP_OK on success; fills *out with parsed mesh data.
 * On failure, *out is zeroed and the error code indicates what went wrong. */
ivp_result_t ivp_3ds_load_file(const ivp_allocator_t *alloc,
                               const char *filename,
                               ivp_3ds_scene_t *out);

/* Load a 3DS scene from an in-memory buffer.
 * Returns IVP_OK on success; fills *out with parsed mesh data. */
ivp_result_t ivp_3ds_load_memory(const ivp_allocator_t *alloc,
                                 const void *data, int size,
                                 ivp_3ds_scene_t *out);

/* Free all data associated with a parsed scene. */
void ivp_3ds_scene_free(ivp_3ds_scene_t *scene);

/* ── Building ──────────────────────────────────────────────────────────── */

/* Build a compact surface from a single mesh in the parsed scene.
 * cfg may be NULL for defaults. Returns NULL on failure. */
ivp_compact_surface_t *ivp_3ds_build_surface(const ivp_allocator_t *alloc,
                                             const ivp_3ds_scene_t *scene,
                                             int mesh_index,
                                             const ivp_3ds_import_config_t *cfg);

/* Convenience: load file + build surface from first mesh in one call.
 * cfg may be NULL for defaults. Returns NULL on failure. */
ivp_compact_surface_t *ivp_3ds_load_and_build(const ivp_allocator_t *alloc,
                                              const char *filename,
                                              const ivp_3ds_import_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* IVP_IMPORT_3DS_H */
