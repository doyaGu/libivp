#include <ivp/containers/ivp_elem_typeinfo.h>
#include <ivp/math/ivp_vec3.h>
#include <string.h>

const ivp_elem_typeinfo_t IVP_ELEM_TYPEINFO_PTR = {
    IVP_ELEM_TYPE_ID_PTR,
    "void*",
    sizeof(void *),
    sizeof(void *),
    IVP_ELEM_FLAG_POINTER | IVP_ELEM_FLAG_TRIVIAL_MOVE |
        IVP_ELEM_FLAG_TRIVIAL_COPY | IVP_ELEM_FLAG_TRIVIAL_DROP,
    NULL
};

const ivp_elem_typeinfo_t IVP_ELEM_TYPEINFO_U32 = {
    IVP_ELEM_TYPE_ID_U32,
    "u32",
    sizeof(uint32_t),
    sizeof(uint32_t),
    IVP_ELEM_FLAG_TRIVIAL_MOVE | IVP_ELEM_FLAG_TRIVIAL_COPY | IVP_ELEM_FLAG_TRIVIAL_DROP,
    NULL
};

const ivp_elem_typeinfo_t IVP_ELEM_TYPEINFO_F32 = {
    IVP_ELEM_TYPE_ID_F32,
    "f32",
    sizeof(float),
    sizeof(float),
    IVP_ELEM_FLAG_TRIVIAL_MOVE | IVP_ELEM_FLAG_TRIVIAL_COPY | IVP_ELEM_FLAG_TRIVIAL_DROP,
    NULL
};

const ivp_elem_typeinfo_t IVP_ELEM_TYPEINFO_VEC3F = {
    IVP_ELEM_TYPE_ID_VEC3F,
    "vec3f",
    sizeof(ivp_vec3f_t),
    _Alignof(ivp_vec3f_t),
    IVP_ELEM_FLAG_TRIVIAL_MOVE | IVP_ELEM_FLAG_TRIVIAL_COPY | IVP_ELEM_FLAG_TRIVIAL_DROP,
    NULL
};

static const ivp_elem_typeinfo_t *s_elem_registry[IVP_ELEM_TYPEINFO_REGISTRY_MAX];
static uint32_t s_elem_registry_count;
static bool s_elem_registry_initialized;

static bool ivp_elem_typeinfo_is_valid(const ivp_elem_typeinfo_t *typeinfo) {
    if (!typeinfo) return false;
    if (typeinfo->type_id == (uint32_t)IVP_ELEM_TYPE_ID_INVALID) return false;
    if (!typeinfo->name || typeinfo->name[0] == '\0') return false;
    if (typeinfo->size == 0u) return false;
    if (typeinfo->align == 0u) return false;
    return true;
}

static void ivp_elem_registry_init_once(void) {
    if (s_elem_registry_initialized) return;
    s_elem_registry_initialized = true;

    s_elem_registry[s_elem_registry_count++] = &IVP_ELEM_TYPEINFO_PTR;
    s_elem_registry[s_elem_registry_count++] = &IVP_ELEM_TYPEINFO_U32;
    s_elem_registry[s_elem_registry_count++] = &IVP_ELEM_TYPEINFO_F32;
    s_elem_registry[s_elem_registry_count++] = &IVP_ELEM_TYPEINFO_VEC3F;
}

bool ivp_elem_typeinfo_register(const ivp_elem_typeinfo_t *typeinfo) {
    ivp_elem_registry_init_once();
    if (!ivp_elem_typeinfo_is_valid(typeinfo)) return false;

    for (uint32_t i = 0; i < s_elem_registry_count; i++) {
        const ivp_elem_typeinfo_t *current = s_elem_registry[i];
        if (current == typeinfo) {
            return true;
        }
        if (current->type_id == typeinfo->type_id) {
            return false;
        }
        if (strcmp(current->name, typeinfo->name) == 0) {
            return false;
        }
    }

    if (s_elem_registry_count >= (uint32_t)IVP_ELEM_TYPEINFO_REGISTRY_MAX) {
        return false;
    }

    s_elem_registry[s_elem_registry_count++] = typeinfo;
    return true;
}

const ivp_elem_typeinfo_t *ivp_elem_typeinfo_from_id(ivp_elem_type_id_t id) {
    ivp_elem_registry_init_once();
    uint32_t wanted = (uint32_t)id;
    for (uint32_t i = 0; i < s_elem_registry_count; i++) {
        const ivp_elem_typeinfo_t *current = s_elem_registry[i];
        if (current->type_id == wanted) {
            return current;
        }
    }
    return NULL;
}

const ivp_elem_typeinfo_t *ivp_elem_typeinfo_from_name(const char *name) {
    ivp_elem_registry_init_once();
    if (!name) return NULL;
    for (uint32_t i = 0; i < s_elem_registry_count; i++) {
        const ivp_elem_typeinfo_t *current = s_elem_registry[i];
        if (strcmp(current->name, name) == 0) {
            return current;
        }
    }
    return NULL;
}

bool ivp_elem_typeinfo_is_registered(const ivp_elem_typeinfo_t *typeinfo) {
    ivp_elem_registry_init_once();
    if (!typeinfo) return false;
    for (uint32_t i = 0; i < s_elem_registry_count; i++) {
        if (s_elem_registry[i] == typeinfo) {
            return true;
        }
    }
    return false;
}

uint32_t ivp_elem_typeinfo_registry_count(void) {
    ivp_elem_registry_init_once();
    return s_elem_registry_count;
}

const ivp_elem_typeinfo_t *ivp_elem_typeinfo_registry_at(uint32_t index) {
    ivp_elem_registry_init_once();
    if (index >= s_elem_registry_count) return NULL;
    return s_elem_registry[index];
}
