#include <ivp/containers/ivp_elem_typeinfo.h>
#include <ivp/math/ivp_vec3.h>
#include <stdatomic.h>
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

static const ivp_elem_typeinfo_t *s_elem_registry[IVP_ELEM_TYPEINFO_REGISTRY_MAX] = {
    &IVP_ELEM_TYPEINFO_PTR,
    &IVP_ELEM_TYPEINFO_U32,
    &IVP_ELEM_TYPEINFO_F32,
    &IVP_ELEM_TYPEINFO_VEC3F,
};
static uint32_t s_elem_registry_count = 4u;
static atomic_flag s_elem_registry_lock = ATOMIC_FLAG_INIT;

static void ivp_elem_registry_lock(void)
{
    while (atomic_flag_test_and_set_explicit(&s_elem_registry_lock,
                                              memory_order_acquire)) {
    }
}

static void ivp_elem_registry_unlock(void)
{
    atomic_flag_clear_explicit(&s_elem_registry_lock, memory_order_release);
}

static bool ivp_elem_typeinfo_is_valid(const ivp_elem_typeinfo_t *typeinfo) {
    if (!typeinfo) return false;
    if (typeinfo->type_id == (uint32_t)IVP_ELEM_TYPE_ID_INVALID) return false;
    if (!typeinfo->name || typeinfo->name[0] == '\0') return false;
    if (typeinfo->size == 0u) return false;
    if (typeinfo->align == 0u) return false;
    return true;
}

bool ivp_elem_typeinfo_register(const ivp_elem_typeinfo_t *typeinfo) {
    if (!ivp_elem_typeinfo_is_valid(typeinfo)) return false;

    ivp_elem_registry_lock();
    for (uint32_t i = 0; i < s_elem_registry_count; i++) {
        const ivp_elem_typeinfo_t *current = s_elem_registry[i];
        if (current == typeinfo) {
            ivp_elem_registry_unlock();
            return true;
        }
        if (current->type_id == typeinfo->type_id) {
            ivp_elem_registry_unlock();
            return false;
        }
        if (strcmp(current->name, typeinfo->name) == 0) {
            ivp_elem_registry_unlock();
            return false;
        }
    }

    if (s_elem_registry_count >= (uint32_t)IVP_ELEM_TYPEINFO_REGISTRY_MAX) {
        ivp_elem_registry_unlock();
        return false;
    }

    s_elem_registry[s_elem_registry_count++] = typeinfo;
    ivp_elem_registry_unlock();
    return true;
}

const ivp_elem_typeinfo_t *ivp_elem_typeinfo_from_id(ivp_elem_type_id_t id) {
    uint32_t wanted = (uint32_t)id;
    const ivp_elem_typeinfo_t *result = NULL;
    ivp_elem_registry_lock();
    for (uint32_t i = 0; i < s_elem_registry_count; i++) {
        const ivp_elem_typeinfo_t *current = s_elem_registry[i];
        if (current->type_id == wanted) {
            result = current;
            break;
        }
    }
    ivp_elem_registry_unlock();
    return result;
}

const ivp_elem_typeinfo_t *ivp_elem_typeinfo_from_name(const char *name) {
    if (!name) return NULL;
    const ivp_elem_typeinfo_t *result = NULL;
    ivp_elem_registry_lock();
    for (uint32_t i = 0; i < s_elem_registry_count; i++) {
        const ivp_elem_typeinfo_t *current = s_elem_registry[i];
        if (strcmp(current->name, name) == 0) {
            result = current;
            break;
        }
    }
    ivp_elem_registry_unlock();
    return result;
}

bool ivp_elem_typeinfo_is_registered(const ivp_elem_typeinfo_t *typeinfo) {
    if (!typeinfo) return false;
    bool registered = false;
    ivp_elem_registry_lock();
    for (uint32_t i = 0; i < s_elem_registry_count; i++) {
        if (s_elem_registry[i] == typeinfo) {
            registered = true;
            break;
        }
    }
    ivp_elem_registry_unlock();
    return registered;
}

uint32_t ivp_elem_typeinfo_registry_count(void) {
    uint32_t count;
    ivp_elem_registry_lock();
    count = s_elem_registry_count;
    ivp_elem_registry_unlock();
    return count;
}

const ivp_elem_typeinfo_t *ivp_elem_typeinfo_registry_at(uint32_t index) {
    const ivp_elem_typeinfo_t *result = NULL;
    ivp_elem_registry_lock();
    if (index < s_elem_registry_count) {
        result = s_elem_registry[index];
    }
    ivp_elem_registry_unlock();
    return result;
}
