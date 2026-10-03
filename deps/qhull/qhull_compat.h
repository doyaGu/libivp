/* qhull_compat.h -- build glue for qhull 2.6
 *
 * Qhull, Copyright (c) 1993-1999, The Geometry Center, University of
 * Minnesota.  See COPYING.txt in this directory for the license.
 *
 * The sources in this directory are qhull 2.6, taken from the copy in the
 * reference source tree (ivp_compact_builder/qhull*.cxx), where they are
 * compiled as C++ and print through the host's ivp_message().  This header
 * provides the few host helpers the sources use (p_malloc/p_calloc/P_FREE,
 * ivp_message) so the same code compiles as C17.
 *
 * Modification notice (qhull license condition 3): modified by doyaGu for
 * libivp on 2026-10-02 to reproduce the reference convex hulls (facet and
 * facet vertex order) exactly.  Changes against
 * ivp_compact_builder/qhull*.cxx:
 * renamed .cxx/.hxx to .c/.h, <ivp_physics.hxx> replaced by this header,
 * getid_() uses a C cast, qh_readfeasible() prototype matches its
 * definition, ptr_intT is uintptr_t, two unnamed parameters are named,
 * ivp_message() is qh_message() (same text, output through a sink that
 * libivp routes to its ivp_message()), and qhull_api.[ch] were added.
 * Algorithms and arithmetic are unchanged.
 *
 * The original qhull source code may be obtained from http://www.qhull.org
 * (qhull license condition 4).
 */
#ifndef QHULL_COMPAT_H
#define QHULL_COMPAT_H

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define p_malloc(size) malloc(size)
#define p_calloc(nelem, size) calloc((nelem), (size))
#define p_free(ptr) free(ptr)

/* P_FREE: free and reset the pointer */
#define P_FREE(a)                  \
    do {                           \
        if (a) {                   \
            free((void *)(a));     \
            (a) = NULL;            \
        }                          \
    } while (0)

/* The host's message function (the reference ivu_string.cxx printed
 * "[havok] ..." to stdout).  Renamed to stay out of libivp's namespace;
 * the text goes to the sink installed with qh_api_set_message_sink()
 * (qhull_api.h), by default "[havok] " + text on stdout like the
 * reference. */
#include "qhull_api.h"
#define ivp_message qh_message
void qh_message(const char *templat, ...);

#endif /* QHULL_COMPAT_H */
