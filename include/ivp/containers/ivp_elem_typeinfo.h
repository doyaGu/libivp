/* ivp_elem_typeinfo.h — Orthogonal element metadata for containers */
#ifndef IVP_ELEM_TYPEINFO_H
#define IVP_ELEM_TYPEINFO_H

#include <ivp/base/ivp_types.h>
#include <ivp/containers/ivp_elem_lifecycle.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Type flags for container-side policy decisions. */
enum {
    IVP_ELEM_FLAG_NONE          = 0u,
    IVP_ELEM_FLAG_POINTER       = 1u << 0,
    IVP_ELEM_FLAG_TRIVIAL_MOVE  = 1u << 1,
    IVP_ELEM_FLAG_TRIVIAL_COPY  = 1u << 2,
    IVP_ELEM_FLAG_TRIVIAL_DROP  = 1u << 3
};

typedef enum ivp_elem_type_id {
    IVP_ELEM_TYPE_ID_INVALID = 0,
    IVP_ELEM_TYPE_ID_PTR     = 1,
    IVP_ELEM_TYPE_ID_U32     = 2,
    IVP_ELEM_TYPE_ID_F32     = 3,
    IVP_ELEM_TYPE_ID_VEC3F   = 4,
    IVP_ELEM_TYPE_ID_COUNT
} ivp_elem_type_id_t;

typedef struct ivp_elem_typeinfo {
    uint32_t    type_id;
    const char *name;
    size_t      size;
    size_t      align;
    uint32_t    flags;
    const ivp_elem_lifecycle_t *lifecycle;
} ivp_elem_typeinfo_t;

enum { IVP_ELEM_TYPEINFO_REGISTRY_MAX = 256u };

/* Canonical typeinfo for pointer element containers (void* payloads). */
extern const ivp_elem_typeinfo_t IVP_ELEM_TYPEINFO_PTR;
extern const ivp_elem_typeinfo_t IVP_ELEM_TYPEINFO_U32;
extern const ivp_elem_typeinfo_t IVP_ELEM_TYPEINFO_F32;
extern const ivp_elem_typeinfo_t IVP_ELEM_TYPEINFO_VEC3F;

/* Registry APIs: builtins are preloaded; users may register extra types at runtime. */
bool ivp_elem_typeinfo_register(const ivp_elem_typeinfo_t *typeinfo);
const ivp_elem_typeinfo_t *ivp_elem_typeinfo_from_id(ivp_elem_type_id_t id);
const ivp_elem_typeinfo_t *ivp_elem_typeinfo_from_name(const char *name);
bool ivp_elem_typeinfo_is_registered(const ivp_elem_typeinfo_t *typeinfo);
uint32_t ivp_elem_typeinfo_registry_count(void);
const ivp_elem_typeinfo_t *ivp_elem_typeinfo_registry_at(uint32_t index);

#ifdef __cplusplus
}
#endif

#endif /* IVP_ELEM_TYPEINFO_H */
