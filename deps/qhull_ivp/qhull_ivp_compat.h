/* qhull_ivp_compat.h -- glue between the vendored IVP qhull 2.6 and libivp
 *
 * Qhull, Copyright (c) 1993-1999, The Geometry Center, University of
 * Minnesota.  See COPYING.txt in this directory for the license.
 *
 * The reference IVP (ivp_compact_builder/qhull*.cxx) compiles qhull 2.6
 * (Geometry Center, 1999; modified by Ipion Software GmbH 1999-2000:
 * errors through ivp_message(), IVP allocators, no output in
 * qh_new_qhull) as C++ against <ivp_physics.hxx>.  This header provides
 * the few IVP helpers the sources use (p_malloc/p_calloc/P_FREE,
 * ivp_message) so the same code compiles as C17.
 *
 * Modification notice (qhull license condition 3): modified for libivp on
 * 2026-10-02 to reproduce the reference IVP convex hulls (facet and facet
 * vertex order) exactly.  Changes against ivp_compact_builder/qhull*.cxx:
 * renamed .cxx/.hxx to .c/.h, <ivp_physics.hxx> replaced by this header,
 * getid_() uses a C cast, qh_readfeasible() prototype matches its
 * definition, ptr_intT is uintptr_t, two unnamed parameters are named,
 * ivp_message() is qh_ivp_message() (same text, output through a sink that
 * libivp routes to its ivp_message()), and qhull_ivp_api.[ch] were added.
 * Algorithms and arithmetic are unchanged.
 */
#ifndef QHULL_IVP_COMPAT_H
#define QHULL_IVP_COMPAT_H

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define p_malloc(size) malloc(size)
#define p_calloc(nelem, size) calloc((nelem), (size))
#define p_free(ptr) free(ptr)

/* IVP P_FREE: free and reset the pointer */
#define P_FREE(a)                  \
    do {                           \
        if (a) {                   \
            free((void *)(a));     \
            (a) = NULL;            \
        }                          \
    } while (0)

/* IVP's general message sink (ivu_string.cxx printed "[havok] ..." to
 * stdout).  Renamed to stay out of libivp's namespace; the text goes to the
 * sink installed with qh_ivp_set_message_sink() (qhull_ivp_api.h), by
 * default "[havok] " + text on stdout like the reference. */
#include "qhull_ivp_api.h"
#define ivp_message qh_ivp_message
void qh_ivp_message(const char *templat, ...);

#endif /* QHULL_IVP_COMPAT_H */
