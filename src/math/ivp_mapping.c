/* ivp_mapping.c -- Coordinate axis remapping implementation
 *
 * Ported from ivu_mapping.cxx (Ipion Software GmbH).
 */
#include <ivp/math/ivp_mapping.h>

void ivp_mapping_mapply(const ivp_mapping_t *map,
                         const ivp_mat3r_t *in,
                         ivp_mat3r_t *out) {
    ivp_mat3r_t tmp;
    ivp_mat3r_get_row(in, map->k[0], &tmp.rows[0]);
    ivp_mat3r_get_row(in, map->k[1], &tmp.rows[1]);
    ivp_mat3r_get_row(in, map->k[2], &tmp.rows[2]);
    *out = tmp;
}

void ivp_mapping_miapply(const ivp_mapping_t *map,
                          const ivp_mat3r_t *in,
                          ivp_mat3r_t *out) {
    ivp_mat3r_t tmp;
    ivp_mat3r_get_row(in, 0, &tmp.rows[map->k[0]]);
    ivp_mat3r_get_row(in, 1, &tmp.rows[map->k[1]]);
    ivp_mat3r_get_row(in, 2, &tmp.rows[map->k[2]]);
    *out = tmp;
}

void ivp_mapping_mi2apply(const ivp_mapping_t *map,
                           const ivp_mat3r_t *in,
                           ivp_mat3r_t *out) {
    ivp_vec3r_t c0, c1, c2;
    ivp_mat3r_t tmp;
    ivp_mat3r_get_col(in, map->k[0], &c0);
    ivp_mat3r_get_col(in, map->k[1], &c1);
    ivp_mat3r_get_col(in, map->k[2], &c2);
    ivp_mat3r_set_cols(&tmp, &c0, &c1, &c2);
    *out = tmp;
}

ivp_matrix_category_t ivp_mapping_type(const ivp_mapping_t *map) {
    /* Check if it's a valid permutation first */
    int seen[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++) {
        if (map->k[i] < 0 || map->k[i] > 2) return IVP_MC_UNDEFINED;
        seen[map->k[i]]++;
    }
    if (seen[0] != 1 || seen[1] != 1 || seen[2] != 1) return IVP_MC_UNDEFINED;

    /* Even permutations: (0,1,2), (1,2,0), (2,0,1) -> right-handed
     * Odd permutations:  (0,2,1), (1,0,2), (2,1,0) -> left-handed */
    int parity = 0;
    int p[3] = {map->k[0], map->k[1], map->k[2]};
    for (int i = 0; i < 3; i++) {
        while (p[i] != i) {
            int tmp = p[p[i]];
            p[p[i]] = p[i];
            p[i] = tmp;
            parity++;
        }
    }
    return (parity % 2 == 0) ? IVP_MC_RIGHT_HANDED : IVP_MC_LEFT_HANDED;
}
