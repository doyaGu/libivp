#ifndef IVP_BUILD_CHECKED_H
#define IVP_BUILD_CHECKED_H

#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

static inline bool ivp_build_size_mul(size_t a, size_t b, size_t *out)
{
    if (!out || (a != 0u && b > SIZE_MAX / a)) return false;
    *out = a * b;
    return true;
}

static inline bool ivp_build_size_add(size_t a, size_t b, size_t *out)
{
    if (!out || b > SIZE_MAX - a) return false;
    *out = a + b;
    return true;
}

static inline bool ivp_build_count_bytes(int count, size_t element_size,
                                         size_t *out)
{
    return count >= 0 && ivp_build_size_mul((size_t)count, element_size, out);
}

static inline bool ivp_build_int_mul(int a, int b, int *out)
{
    if (!out || a < 0 || b < 0 || (a != 0 && b > INT_MAX / a)) return false;
    *out = a * b;
    return true;
}

static inline bool ivp_build_int_add(int a, int b, int *out)
{
    if (!out || a < 0 || b < 0 || b > INT_MAX - a) return false;
    *out = a + b;
    return true;
}

static inline bool ivp_build_int_grow_pow2(int minimum, int *out)
{
    if (!out || minimum < 0) return false;
    int capacity = 16;
    while (capacity < minimum) {
        if (capacity > INT_MAX / 2) return false;
        capacity *= 2;
    }
    *out = capacity;
    return true;
}

#endif /* IVP_BUILD_CHECKED_H */
