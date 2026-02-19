/* ivp_cache_object.h — AT-time object transform cache for IVP C17
 *
 * Ported from IVP_Cache_Object / IVP_Real_Object AT functions.
 *
 * The core stores the center-of-mass pose. Objects may have a shift
 * (shift_core_f_object) and a rotation offset (q_core_f_object) from
 * core space to object space. This header provides:
 *   1. ivp_cache_object_t — a cached snapshot of the object's world transform
 *   2. Object-level AT (any-time) query functions that compose core interpolation
 *      with the core-to-object offsets
 */
#ifndef IVP_CACHE_OBJECT_H
#define IVP_CACHE_OBJECT_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_quat.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/math/ivp_transform.h>
#include <ivp/dynamics/ivp_object.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Cached object world transform ──────────────────────────────────────── */

typedef struct ivp_cache_object {
    ivp_quatr_t       q_world_f_object;    /* AT-interpolated orientation (object space) */
    ivp_transform_r_t m_world_f_object;    /* AT-interpolated full 3x4 transform         */
    ivp_vec3r_t       core_pos;            /* AT-interpolated core center position        */
    ivp_time_t        cached_time;         /* time at which cache was computed            */
} ivp_cache_object_t;

/* ── Object-level AT query functions ────────────────────────────────────── */

/* Get AT-time object world position.
 * If shift_core_f_object is zero (common case), returns core AT position directly.
 * Otherwise, applies the shift in core-rotated space. */
static inline void ivp_object_get_at_position(const ivp_object_t *obj,
                                               ivp_time_t t,
                                               ivp_vec3r_t *out) {
    const ivp_core_t *core = ivp_object_get_core_const(obj);
    ivp_core_get_at_position(core, t, out);
    if (!obj->shift_core_f_object_is_zero) {
        /* Rotate shift by AT quaternion and add */
        ivp_quatr_t q;
        ivp_core_get_at_quat(core, t, &q);
        ivp_mat3r_t rot;
        ivp_quat_to_mat3(&q, &rot);
        ivp_vec3r_t ws_shift;
        ivp_mat3r_vmult(&ws_shift, &rot, &obj->shift_core_f_object);
        ivp_vec3r_add(out, out, &ws_shift);
    }
}

/* Get AT-time object world quaternion.
 * Composes core AT quaternion with q_core_f_object if present. */
static inline void ivp_object_get_at_quat(const ivp_object_t *obj,
                                           ivp_time_t t,
                                           ivp_quatr_t *out) {
    const ivp_core_t *core = ivp_object_get_core_const(obj);
    ivp_core_get_at_quat(core, t, out);
    if (obj->q_core_f_object) {
        ivp_quatr_t combined;
        ivp_quat_mult(&combined, out, obj->q_core_f_object);
        *out = combined;
    }
}

/* Get AT-time full object world transform (rotation matrix + position).
 * This is the object-level equivalent of ivp_core_calc_at_matrix(). */
static inline void ivp_object_get_at_matrix(const ivp_object_t *obj,
                                             ivp_time_t t,
                                             ivp_transform_r_t *out) {
    const ivp_core_t *core = ivp_object_get_core_const(obj);

    /* Start with core AT quaternion → rotation matrix */
    ivp_quatr_t q;
    ivp_core_get_at_quat(core, t, &q);

    /* Apply rotation offset if present */
    if (obj->q_core_f_object) {
        ivp_quatr_t combined;
        ivp_quat_mult(&combined, &q, obj->q_core_f_object);
        q = combined;
    }

    ivp_quat_to_mat3(&q, &out->rot);

    /* Core AT position */
    ivp_core_get_at_position(core, t, &out->pos);

    /* Apply shift offset if present */
    if (!obj->shift_core_f_object_is_zero) {
        ivp_vec3r_t ws_shift;
        ivp_mat3r_vmult(&ws_shift, &out->rot, &obj->shift_core_f_object);
        ivp_vec3r_add(&out->pos, &out->pos, &ws_shift);
    }
}

/* Compute and fill a cache_object snapshot at time t. */
static inline void ivp_cache_object_update(ivp_cache_object_t *cache,
                                            const ivp_object_t *obj,
                                            ivp_time_t t) {
    const ivp_core_t *core = ivp_object_get_core_const(obj);

    /* Core AT position (always needed) */
    ivp_core_get_at_position(core, t, &cache->core_pos);

    /* Object AT transform */
    ivp_object_get_at_matrix(obj, t, &cache->m_world_f_object);

    /* Object AT quaternion */
    ivp_object_get_at_quat(obj, t, &cache->q_world_f_object);

    cache->cached_time = t;
}

/* ── Core-to-object offset setters ──────────────────────────────────────── */

/* Set the translation offset from core center-of-mass to object origin (in core space). */
static inline void ivp_object_set_shift_core_f_object(ivp_object_t *obj,
                                                       const ivp_vec3r_t *shift) {
    obj->shift_core_f_object = *shift;
    obj->shift_core_f_object_is_zero =
        (shift->k[0] == IVP_REAL(0.0) &&
         shift->k[1] == IVP_REAL(0.0) &&
         shift->k[2] == IVP_REAL(0.0));
}

/* Set the rotation offset from core frame to object frame.
 * Pass NULL to clear (identity). The pointer is borrowed — caller must
 * ensure it remains valid for the object's lifetime. */
static inline void ivp_object_set_q_core_f_object(ivp_object_t *obj,
                                                    ivp_quatr_t *q) {
    obj->q_core_f_object = q;
}

#ifdef __cplusplus
}
#endif

#endif /* IVP_CACHE_OBJECT_H */
