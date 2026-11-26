/* qhull_ivp.h -- what the reference engine's bundled Qhull took from
 * <ivp_physics.hxx>.
 *
 * The files in this directory are the Qhull snapshot the IVP reference
 * engine ships in ivp_compact_builder (Ipion's "ipion 99/07/10" edition of
 * The Geometry Center's Qhull, with the engine's later edits), compiled as
 * C. IVP provides message output, memory helpers, and extended-precision
 * hull predicates. THIRD_PARTY_NOTICES.md lists the changes and the license.
 */
#ifndef IVP_QHULL_IPION_IVP_H
#define IVP_QHULL_IPION_IVP_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <ivp/base/ivp_message.h>

#include "qhull_symbols.h"

#if IVP_X87_ARITHMETIC
#include "../../math/ivp_x87.h"
#endif

/* ivu_types.hxx: an unsigned integer as wide as a pointer */
typedef uintptr_t uintp;

/* ivu_memory.cxx p_malloc / p_calloc / p_free: the C allocator */
#define p_malloc(size)        malloc(size)
#define p_calloc(nelem, size) calloc((nelem), (size))

/* ivu_types.hxx P_FREE over p_free (free) */
#define P_FREE(a)          \
    {                      \
        if (a) {           \
            free((void *)(a)); \
            (a) = NULL;    \
        }                  \
    }

#endif /* IVP_QHULL_IPION_IVP_H */
