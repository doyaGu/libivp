/* ivp_mapping.h -- Coordinate axis remapping for IVP C17
 *
 * Maps between coordinate systems by permuting axes.
 * For example, mapping (X,Y,Z) -> (Y,Z,X) swaps all three axes.
 * Used by collision and constraint systems for frame transformations.
 *
 * Ported from ivu_mapping.hxx / ivu_mapping.cxx.
 */
#ifndef IVP_MAPPING_H
#define IVP_MAPPING_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_mat3.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Handedness ────────────────────────────────────────────────────────── */

typedef enum ivp_matrix_category {
    IVP_MC_UNDEFINED     = 0,
    IVP_MC_RIGHT_HANDED  = 1,
    IVP_MC_LEFT_HANDED   = 2
} ivp_matrix_category_t;

/* ── Mapping ───────────────────────────────────────────────────────────── */

typedef struct ivp_mapping {
    int8_t k[3];  /* k[dst_axis] = src_axis */
} ivp_mapping_t;

/* Initialize to identity mapping (X->X, Y->Y, Z->Z). */
static inline ivp_mapping_t ivp_mapping_identity(void) {
    ivp_mapping_t m;
    m.k[0] = IVP_INDEX_X;
    m.k[1] = IVP_INDEX_Y;
    m.k[2] = IVP_INDEX_Z;
    return m;
}

/* Apply mapping to a vector: out[i] = in[mapping[i]]. */
static inline void ivp_mapping_vapply(const ivp_mapping_t *map,
                                       const ivp_vec3r_t *in,
                                       ivp_vec3r_t *out) {
    out->k[0] = in->k[map->k[0]];
    out->k[1] = in->k[map->k[1]];
    out->k[2] = in->k[map->k[2]];
}

/* Inverse-apply mapping: out[mapping[i]] = in[i]. */
static inline void ivp_mapping_viapply(const ivp_mapping_t *map,
                                        const ivp_vec3r_t *in,
                                        ivp_vec3r_t *out) {
    out->k[map->k[0]] = in->k[0];
    out->k[map->k[1]] = in->k[1];
    out->k[map->k[2]] = in->k[2];
}

/* Apply mapping to a matrix (permute rows): out_row[i] = in_row[mapping[i]]. */
void ivp_mapping_mapply(const ivp_mapping_t *map,
                         const ivp_mat3r_t *in,
                         ivp_mat3r_t *out);

/* Inverse-apply to matrix (permute rows inversely). */
void ivp_mapping_miapply(const ivp_mapping_t *map,
                          const ivp_mat3r_t *in,
                          ivp_mat3r_t *out);

/* Apply to matrix columns: out_col[i] = in_col[mapping[i]]. */
void ivp_mapping_mi2apply(const ivp_mapping_t *map,
                           const ivp_mat3r_t *in,
                           ivp_mat3r_t *out);

/* Determine if mapping is right-handed, left-handed, or undefined. */
ivp_matrix_category_t ivp_mapping_type(const ivp_mapping_t *map);

#ifdef __cplusplus
}
#endif

#endif /* IVP_MAPPING_H */
