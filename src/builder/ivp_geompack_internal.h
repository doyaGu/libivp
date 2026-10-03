/* ivp_geompack_internal.h — GEOMPACK state (reference class IVP_Geompack)
 *
 * Internal to the builder module.  See ivp_geompack.c.
 */
#ifndef IVP_GEOMPACK_INTERNAL_H
#define IVP_GEOMPACK_INTERNAL_H

#include <ivp/base/ivp_allocator.h>
#include <stdbool.h>
#include <stddef.h>

/* struct geompack_parameters (geompack.hxx) */
typedef struct ivp_geompack_parameters {
    double tolin;   /* relative tolerance */
    double angacc;  /* min acceptable dihedral angle of cut faces, degrees */
    double rdacc;   /* min acceptable relative distance cut plane <-> vertices */

    int     nvc;    /* number of vertices in vcl */
    int     nface;  /* number of faces */
    double *vcl;    /* [3 * (nvc + 2)] vertex coordinates (ownership passes to geompack) */
    int    *facep;  /* [3 * (nface + 2)] row 1: 1-based head pointer into fvl (ownership passes) */
    size_t  vcl_cap_bytes;
    size_t  facep_cap_bytes;
    const int *fvl; /* 1-based vertex indices of the faces (CCW from outside) */

    /* outputs (allocated with the geompack allocator, caller frees with the
     * returned capacities) */
    int     nvc_out;
    int     nface_out;
    int     nvert_out;
    int     npolh_out;
    double *vcl_out;
    int    *facep_out;
    int    *fvl_out;
    size_t  vcl_out_cap;
    size_t  facep_out_cap;
    size_t  fvl_out_cap;
    int     ierr;
} ivp_geompack_parameters_t;

typedef struct ivp_geompack {
    const ivp_allocator_t *alloc;

    double tolerance;
    int    ierr;
    double angacc;
    double rdacc;

    int     size_intworkarray;
    int    *g_intworkarray;
    size_t  cap_g_intworkarray;

    int     size_doubleworkarray;
    double *g_doubleworkarray;
    size_t  cap_g_doubleworkarray;

    int     size_vcl;
    double *g_vcl;
    size_t  cap_g_vcl;

    int     size_polyhedronfirstfaceoffset;
    int    *g_polyhedronfirstfaceoffset;
    size_t  cap_g_polyhedronfirstfaceoffset;

    int     size_polyhedronfaceindices;
    int    *g_polyhedronfaceindices;
    size_t  cap_g_polyhedronfaceindices;

    int     size_facearrays;
    double *g_normals;
    size_t  cap_g_normals;
    int    *g_facesdata;
    size_t  cap_g_facesdata;
    int    *g_facestype;
    size_t  cap_g_facestype;

    int     size_facevertexarrays;
    int    *g_faceverticeslist;
    size_t  cap_g_faceverticeslist;
    double *g_edge_angles;
    size_t  cap_g_edge_angles;

    int     size_ev;
    int    *g_ev;
    size_t  cap_g_ev;

    int     hashtable_maxsize;
    int     hashtable_size;
    int    *g_hashtable;
    size_t  cap_g_hashtable;

    int n_original_vertices;
    int n_polyhedronfaces;
    int n_work_vertices;
    int nface;
    int npolh;
} ivp_geompack_t;

/* IVP_Geompack::decompose (geompack_drdec3.cxx:54) */
void ivp_geompack_decompose(const ivp_allocator_t *alloc, ivp_geompack_parameters_t *params);

#endif /* IVP_GEOMPACK_INTERNAL_H */
