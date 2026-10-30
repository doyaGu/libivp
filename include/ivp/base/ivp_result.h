/* ivp_result.h — Error codes for IVP C17 */
#ifndef IVP_RESULT_H
#define IVP_RESULT_H

#include <stdbool.h>

typedef enum ivp_result {
    IVP_OK                          = 0,

    /* General errors (1–99) */
    IVP_ERR_INVALID_ARG             = 1,
    IVP_ERR_OUT_OF_MEMORY           = 2,
    IVP_ERR_INVALID_STATE           = 3,
    IVP_NOT_FOUND                   = 4,   /* query produced no result (not an error) */

    /* Numerical errors (100–199) */
    IVP_ERR_SINGULAR_MATRIX         = 100,
    IVP_ERR_NORMALIZATION_FAILED    = 101,
    IVP_ERR_NUMERICAL_OVERFLOW      = 102,

    /* Geometry / collision errors (200–299) */
    IVP_ERR_DEGENERATE_GEOMETRY     = 200,
    IVP_ERR_SURFACE_CORRUPT         = 201,

    /* Capacity limits (300–399) */
    IVP_ERR_CAPACITY_EXCEEDED       = 300,

    /* Render module errors (400–499) */
    IVP_ERR_RENDER_SDL_INIT         = 400,
    IVP_ERR_RENDER_WINDOW           = 401,
    IVP_ERR_RENDER_GPU_DEVICE       = 402,
    IVP_ERR_RENDER_SWAPCHAIN        = 403,
    IVP_ERR_RENDER_SHADER           = 404,

    /* I/O errors (500–599) */
    IVP_ERR_FILE_NOT_FOUND          = 500,
    IVP_ERR_FILE_READ               = 501,
    IVP_ERR_FILE_CORRUPT            = 502,
    IVP_ERR_FILE_VERSION            = 503
} ivp_result_t;

/* Helper: check if result is OK */
static inline bool ivp_ok(ivp_result_t r) { return r == IVP_OK; }

/* Helper: store r in an optional out parameter and return it */
static inline ivp_result_t ivp_result_set(ivp_result_t *out, ivp_result_t r) {
    if (out) *out = r;
    return r;
}

/* Convert result to human-readable string */
static inline const char *ivp_result_str(ivp_result_t r) {
    switch (r) {
        case IVP_OK:                          return "OK";
        case IVP_ERR_INVALID_ARG:             return "INVALID_ARG";
        case IVP_ERR_OUT_OF_MEMORY:           return "OUT_OF_MEMORY";
        case IVP_ERR_INVALID_STATE:           return "INVALID_STATE";
        case IVP_NOT_FOUND:                    return "NOT_FOUND";
        case IVP_ERR_SINGULAR_MATRIX:         return "SINGULAR_MATRIX";
        case IVP_ERR_NORMALIZATION_FAILED:    return "NORMALIZATION_FAILED";
        case IVP_ERR_NUMERICAL_OVERFLOW:      return "NUMERICAL_OVERFLOW";
        case IVP_ERR_DEGENERATE_GEOMETRY:     return "DEGENERATE_GEOMETRY";
        case IVP_ERR_SURFACE_CORRUPT:         return "SURFACE_CORRUPT";
        case IVP_ERR_CAPACITY_EXCEEDED:       return "CAPACITY_EXCEEDED";
        case IVP_ERR_RENDER_SDL_INIT:         return "RENDER_SDL_INIT";
        case IVP_ERR_RENDER_WINDOW:           return "RENDER_WINDOW";
        case IVP_ERR_RENDER_GPU_DEVICE:       return "RENDER_GPU_DEVICE";
        case IVP_ERR_RENDER_SWAPCHAIN:        return "RENDER_SWAPCHAIN";
        case IVP_ERR_RENDER_SHADER:           return "RENDER_SHADER";
        case IVP_ERR_FILE_NOT_FOUND:          return "FILE_NOT_FOUND";
        case IVP_ERR_FILE_READ:               return "FILE_READ";
        case IVP_ERR_FILE_CORRUPT:            return "FILE_CORRUPT";
        case IVP_ERR_FILE_VERSION:            return "FILE_VERSION";
        default:                              return "UNKNOWN_ERROR";
    }
}

#endif /* IVP_RESULT_H */
