/* ivp_geompack.c — GEOMPACK convex decomposition (cvdec3) for libivp
 *
 * Mechanical C port of the reference engine's f2c translation of Barry
 * Joe's GEOMPACK (ivp_compact_builder/geompack_*.cxx: initcb, prime,
 * edght, insvr3, insed3, ptpolg, dsphdc, insfac, cutfac, resedg, cvdec3,
 * drdec3) and of IVP_Convex_Decompositor
 * (ivp_compact_builder/ivp_convex_decompositor.cxx).
 *
 * GEOMPACK Version 3 (Fortran77), written and generously provided free of
 * charge by Dr. Barry Joe (bjoe@netcom.ca).
 *
 * The routines keep the f2c structure (1-based Fortran indexing through
 * shifted pointers, gotos) so that they can be compared line by line with
 * the reference; the class members live in ivp_geompack_t.  The debug
 * printing of the reference (IVP_IF(1) + IVP_IFDEBUG(IVP_DM_GEOMPACK_*))
 * is kept at the same places.  Memory growth follows increase_memory()
 * (+1024 elements), new memory is zero filled.
 */
#include "ivp_geompack_internal.h"

#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#include <ivp/diagnostics/ivp_debug_manager.h>

#include <math.h>
#include <string.h>

#define GP_PI 3.14159265358979323846

#define gp_abs(x) ((x) >= 0 ? (x) : -(x))
#define gp_min(a, b) ((a) <= (b) ? (a) : (b))
#define gp_max(a, b) ((a) >= (b) ? (a) : (b))

/* IVP_Geompack::increase_memory: grow to (count + 1024) elements */
static int gp_increase_memory(ivp_geompack_t *gp, void **mem_block, size_t *cap_bytes,
                              int count, size_t size_of_element)
{
    size_t new_bytes = ((size_t)count + 1024u) * size_of_element;
    if (new_bytes <= *cap_bytes) return 1;
    void *new_mem = ivp_realloc(gp->alloc, *mem_block, *cap_bytes, new_bytes);
    if (!new_mem) return 0;
    memset((char *)new_mem + *cap_bytes, 0, new_bytes - *cap_bytes);
    *mem_block = new_mem;
    *cap_bytes = new_bytes;
    return 1;
}

static int i_sign(ivp_geompack_t *gp, int a, int b)
{
    (void)gp;
    int x = (a >= 0 ? a : -a);
    return (b >= 0 ? x : -x);
}

static double d_sign(ivp_geompack_t *gp, double a, double b)
{
    (void)gp;
    double x = (a >= 0 ? a : -a);
    return (b >= 0 ? x : -x);
}

/* forward declarations (the f2c routines call each other freely) */
static void initcb_(ivp_geompack_t *gp, double tolin);
static int prime_(ivp_geompack_t *gp, int k);
static void edght_(ivp_geompack_t *gp, int *a, int *b, int *v, int *hdfree, int *last, int *w);
static void insvr3_(ivp_geompack_t *gp, int a);
static void insed3_(ivp_geompack_t *gp, int *a, int *b, int *hfl);
static void ptpolg_(ivp_geompack_t *gp, int dim, int ldv, int *nv, int inc, int *pgind,
                    double *vcl, double *pt, double *nrml, double *dtol, int *inout);
static void dsphdc_(ivp_geompack_t *gp);
static void insfac_(ivp_geompack_t *gp, int *p, double *nrmlc, int *nce, int *cedge, double *cdang);
static void cutfac_(ivp_geompack_t *gp, int *p, double *nrmlc, double *dtol, int *nedgc,
                    int *pedge, int *nce, int *cedge, double *cdang, long int *rflag);
static void resedg_(ivp_geompack_t *gp, int u, long int *rflag);
static void cvdec3_(ivp_geompack_t *gp);

/* ---- geompack_initcb.cxx ---- */
/* initcb.f -- translated by f2c (version 19990311).
*/

static void initcb_(ivp_geompack_t *gp, double tolin) {

    /* Local variables */
    double epsp1, eps;

/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Initialize global variables in common blocks */
/*        GERROR, GCONST, and GPRINT. The latter is used for */
/*        printing debugging information. */

/*     Input parameters: */
/* 	 TOLIN - relative tolerance used to determine TOL */

/*     Output parameters in common blocks: */
/*        IERR - error code, initialized to 0 */
/*        PI - ACOS(-1.0D0) */
/*        TOL - relative tolerance MAX(TOLIN,100.0D0*EPS) where */
/*              EPS is approximation to machine epsilon */
/*        IPRT - standard output unit 6 */
/*        MSGLVL - message level, initialized to 0 */

    gp->ierr = 0;
    eps = 1.0;
L10:
    eps /= 2.0;
    epsp1 = eps + 1.0;
    if (epsp1 > 1.0) {
	goto L10;
    }
    // Computing MAX
    gp->tolerance = gp_max(tolin, eps * 100.0);

    return;
}

/* ---- geompack_prime.cxx ---- */
/* prime.f -- translated by f2c (version 19990311).
*/

static int prime_(ivp_geompack_t *gp, int k) {

    /* Initialized data */

    static const int primes[150] = { 17,31,47,61,79,97,113,127,149,163,179,193,
	    211,227,241,257,271,293,307,331,353,379,401,431,457,479,503,541,
	    563,587,613,641,673,701,727,751,773,797,821,853,877,907,929,953,
	    977,1009,1049,1087,1123,1163,1201,1237,1277,1319,1361,1399,1433,
	    1471,1511,1543,1579,1613,1657,1699,1741,1783,1831,1873,1931,1973,
	    2017,2069,2129,2203,2267,2333,2389,2441,2503,2557,2609,2663,2719,
	    2789,2851,2917,2999,3061,3137,3209,3299,3371,3449,3527,3613,3697,
	    3779,3863,3947,4049,4211,4421,4621,4813,5011,5227,5413,5623,5813,
	    6011,6211,6421,6619,6823,7013,7211,7411,7621,7817,8011,8219,8419,
	    8623,8819,9011,9221,9413,9613,9811,10037,10211,10427,10613,10831,
	    11027,11213,11411,11617,11813,12011,12211,12413,12611,12821,13033,
	    13217,13411,13613,13829,14011 };

    /* System generated locals */
    int ret_val;

    /* Local variables */
    int l, m, u;

/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Return a prime >= K (if possible) from internal array */
/*        of primes. More primes can be added if desired. */

/*     Input parameters: */
/*        K - positive int */

/*     Returned function value: */
/*        PRIME - smallest prime >= K from internal array (or largest */
/*              in array) */

    if (k <= primes[0]) {
	ret_val = primes[0];
	return(ret_val);
    } else if (k >= primes[149]) {
	ret_val = primes[149];
	return(ret_val);
    }

/*     Use binary search to find prime >= K. */

    l = 1;
    u = 150;
L10:
    m = (l + u) / 2;
    if (k < primes[m - 1]) {
	u = m - 1;
    } else if (k > primes[m - 1]) {
	l = m + 1;
    } else {
	ret_val = primes[m - 1];
	return(ret_val);
    }
    if (l <= u) {
	goto L10;
    }
    ret_val = primes[u];

    return(ret_val);
}

/* ---- geompack_edght.cxx ---- */
/* edght.f -- translated by f2c (version 19990311).
*/

static void edght_(ivp_geompack_t *gp, int	*a,
			  int	*b,
			  int	*v,
			  int	*hdfree,
			  int	*last,
			  int	*w) {

    int bptr, newp, k, aa, bb, ptr;

    int *edge = gp->g_intworkarray;
    int max_n_edges = (int)(gp->size_intworkarray / 4)-1; // max_n_edges = size of int work array / 4 because we have to store 4 values for each edge!

/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Search in hash table HASHTABLE for record in EDGE containing */
/*        key (A,B). */

/*     Input parameters: */
/*        A,B - vertex indices, > 0, of edge (also key of hash table) */
/*        V - value associated with edge */
/*        N - upper bound on A, B */
/*        HASHTABLE_SIZE - size of hash table HASHTABLE */
/*        MAX_N_EDGES - maximum size available for EDGE array */
/*        HDFREE - head pointer to linked list of free entries of EDGE */
/*              array due to deletions */
/*        LAST - index of last entry used in EDGE array */
/*        HASHTABLE(0:HASHTABLE_SIZE-1) - hash table of head pointers (direct chaining */
/*              with ordered lists is used) */
/*        EDGE(1:4,1:MAX_N_EDGES) - entries of hash table records; */
/*              EDGE(1,I) = MIN(A,B); EDGE(2,I) = MAX(A,B); */
/*              EDGE(3,I) = V; EDGE(4,I) = link */
/*        [Note: Before first call to this routine, HDFREE, LAST, and */
/*              entries of HASHTABLE should be set to 0.] */

/*     Updated parameters: */
/*        HDFREE,LAST - at least one of these is updated */
/*        HASHTABLE,EDGE - if key with A,B is found then this record is deleted */
/*              from hash table, else record is inserted in hash table */

/*     Output parameters: */
/*        W - EDGE(3,INDEX), where INDEX is index of record, if found; */
/*              else 0 */

/*     Abnormal return: */
/*        IERR is set to 1 */

    /* Function Body */
    if (*a < *b) {
	aa = *a;
	bb = *b;
    } else {
	aa = *b;
	bb = *a;
    }
    k = (aa * gp->n_original_vertices + bb) % gp->hashtable_size;
    bptr = -1;
    ptr = gp->g_hashtable[k];
L10:
    if (ptr != 0) {
	if (edge[(ptr << 2) + 1] > aa) {
	    goto L20;
	} else if (edge[(ptr << 2) + 1] == aa) {
	    if (edge[(ptr << 2) + 2] > bb) {
		goto L20;
	    } else if (edge[(ptr << 2) + 2] == bb) {
		if (bptr == -1) {
		    gp->g_hashtable[k] = edge[(ptr << 2) + 4];
		} else {
		    edge[(bptr << 2) + 4] = edge[(ptr << 2) + 4];
		}
		edge[(ptr << 2) + 4] = *hdfree;
		*hdfree = ptr;
		*w = edge[(ptr << 2) + 3];
		return;
	    }
	}
	bptr = ptr;
	ptr = edge[(ptr << 2) + 4];
	goto L10;
    }
L20:
    if (*hdfree > 0) {
	newp = *hdfree;
	*hdfree = edge[(*hdfree << 2) + 4];
    } else {
	(*last)++;
	newp = *last;
recheck_size:
	if (newp >= max_n_edges) {
	    int res = gp_increase_memory(gp, (void **)&gp->g_intworkarray, &gp->cap_g_intworkarray, gp->size_intworkarray, sizeof(int));
	    if ( res == 0 ) {
		gp->ierr = 500;
		return;
	    }
//	    *addr_of_size_intworkarray *= 2;
	    gp->size_intworkarray += 1024;
	    edge = gp->g_intworkarray;
	    max_n_edges = (int)(gp->size_intworkarray / 4)-1;
	    goto recheck_size;
	}
    }
    if (bptr == -1) {
	gp->g_hashtable[k] = newp;
    } else {
	edge[(bptr << 2) + 4] = newp;
    }
    edge[(newp << 2) + 1] = aa;
    edge[(newp << 2) + 2] = bb;
    edge[(newp << 2) + 3] = *v;
    edge[(newp << 2) + 4] = ptr;
    *w = 0;

    return;
}

/* ---- geompack_insvr3.cxx ---- */
/* insvr3.f -- translated by f2c (version 19990311).
*/

static void insvr3_(ivp_geompack_t *gp, int a) {

    int    *faceverticeslist = gp->g_faceverticeslist;
    double *edge_angles      = gp->g_edge_angles;

    int lnxt, b, c__, d__, i__, j, k, l;
    long int bflag;
    int n, la, lb, li, lj;
    double angnxt, ang;

/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Insert a vertex on an edge of polyhedral decomposition */
/*        data structure. */

/*     Input parameters: */
/*        A - index of FACEVERTICESLIST specifying edge containing inserted vertex */
/*        ADDR_OF_N_ORIGINAL_VERTICES - number of vertex coordinates */
/*        ADDR_OF_N_WORK_VERTICES - number of positions used in FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        ADDR_OF_SIZE_FACEVERTEXARRAYS - maximum size available for FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        VCL(1:3,1:ADDR_OF_N_ORIGINAL_VERTICES+1) - vertex coordinate list; VCL(*,ADDR_OF_N_ORIGINAL_VERTICES+1) are */
/*              coordinates of vertex to be inserted */
/*        FACEVERTICESLIST(1:6,1:ADDR_OF_N_WORK_VERTICES) - face vertex list */
/*        EDGE_ANGLES(1:ADDR_OF_N_WORK_VERTICES) - edge angles */

/*     Updated parameters: */
/*        ADDR_OF_N_ORIGINAL_VERTICES,ADDR_OF_N_WORK_VERTICES,FACEVERTICESLIST,EDGE_ANGLES */

/*     Abnormal return: */
/*        IERR is set to 15 */

    /* Parameter adjustments */
    --edge_angles;
    faceverticeslist -= 7;

    /* Function Body */
    gp->n_original_vertices++;
    b = faceverticeslist[a * 6 + 3];
    la = faceverticeslist[a * 6 + 1];
    lb = faceverticeslist[b * 6 + 1];
    (void)la;
    (void)lb;
    i__ = a;

/*     Find start edge of FACEVERTICESLIST if AB lies on boundary of decomposition. */

    c__ = i__;
L10:
    d__ = faceverticeslist[c__ * 6 + 5];
    if (d__ != 0 && d__ != i__) {
	c__ = d__;
	goto L10;
    }
    bflag = d__ == 0;

/*     Insert new entry of FACEVERTICESLIST for each face containing edge AB. */

    i__ = c__;
    k = gp->n_work_vertices;
L20:
    j = faceverticeslist[i__ * 6 + 3];
    k++;

recheck_size:
    if ( (k+2) > gp->size_facevertexarrays) {
	int res = 0;
	res = res | gp_increase_memory(gp, (void **)&gp->g_faceverticeslist, &gp->cap_g_faceverticeslist, gp->size_facevertexarrays, 6*sizeof(int));
	res = res | gp_increase_memory(gp, (void **)&gp->g_edge_angles, &gp->cap_g_edge_angles, gp->size_facevertexarrays, sizeof(double));
	if ( res == 0 ) {
	    gp->ierr = 500;
	    return;
	}
//	*addr_of_size_facevertexarrays *= 2;
	gp->size_facevertexarrays += 1024;
	faceverticeslist = gp->g_faceverticeslist;
	faceverticeslist -= 7;
	edge_angles = gp->g_edge_angles;
	--edge_angles;
	goto recheck_size;
    }

    faceverticeslist[k * 6 + 1] = gp->n_original_vertices;
    faceverticeslist[k * 6 + 2] = faceverticeslist[i__ * 6 + 2];
    faceverticeslist[k * 6 + 3] = j;
    faceverticeslist[k * 6 + 4] = i__;
    faceverticeslist[k * 6 + 5] = 0;
    faceverticeslist[k * 6 + 6] = 0;
    faceverticeslist[i__ * 6 + 3] = k;
    faceverticeslist[j * 6 + 4] = k;
    edge_angles[k] = -1.;
    i__ = faceverticeslist[i__ * 6 + 6];
    if (i__ != 0 && i__ != c__) {
	goto L20;
    }
    gp->n_work_vertices = k;

/*     Set EDGA, EDGC, and EDGE_ANGLES fields. */

    i__ = c__;
    l = faceverticeslist[i__ * 6 + 6];
    ang = edge_angles[i__];
L30:
    k = faceverticeslist[i__ * 6 + 3];
    j = faceverticeslist[k * 6 + 3];
    li = faceverticeslist[i__ * 6 + 1];
    lj = faceverticeslist[j * 6 + 1];
    n = faceverticeslist[l * 6 + 3];
    lnxt = faceverticeslist[l * 6 + 6];
    angnxt = edge_angles[l];
    if (li < lj) {
	if (faceverticeslist[l * 6 + 1] == li) {
	    faceverticeslist[k * 6 + 5] = n;
	    faceverticeslist[n * 6 + 6] = k;
	    edge_angles[n] = ang;
	} else {
	    faceverticeslist[i__ * 6 + 6] = n;
	    faceverticeslist[n * 6 + 5] = i__;
	    faceverticeslist[k * 6 + 5] = l;
	    faceverticeslist[l * 6 + 6] = k;
	    edge_angles[l] = ang;
	    if (lnxt == 0) {
		faceverticeslist[l * 6 + 5] = 0;
	    }
	}
    } else {
	if (faceverticeslist[l * 6 + 1] == li) {
	    faceverticeslist[k * 6 + 6] = n;
	    faceverticeslist[n * 6 + 5] = k;
	    edge_angles[k] = ang;
	    faceverticeslist[i__ * 6 + 5] = l;
	    faceverticeslist[l * 6 + 6] = i__;
	    edge_angles[l] = ang;
	    if (lnxt == 0) {
		faceverticeslist[l * 6 + 5] = 0;
	    }
	} else {
	    faceverticeslist[k * 6 + 6] = l;
	    faceverticeslist[l * 6 + 5] = k;
	    edge_angles[k] = ang;
	    faceverticeslist[i__ * 6 + 5] = n;
	    faceverticeslist[n * 6 + 6] = i__;
	    edge_angles[n] = ang;
	}
	if (bflag && i__ == c__) {
	    faceverticeslist[i__ * 6 + 6] = 0;
	    edge_angles[i__] = -1.;
	}
    }
    i__ = l;
    l = lnxt;
    ang = angnxt;
    if (l != 0 && i__ != c__) {
	goto L30;
    }

    return;
}

/* ---- geompack_insed3.cxx ---- */
/* insed3.f -- translated by f2c (version 19990311).
*/

static void insed3_(ivp_geompack_t *gp, int	*a,
			   int	*b,
			   int	*hfl) {

    int	   *polyhedronfaceindices     = gp->g_polyhedronfaceindices;
    int    *facesdata                 = gp->g_facesdata;
    int    *facestype                 = gp->g_facestype;
    double *normals                   = gp->g_normals;
    int    *faceverticeslist          = gp->g_faceverticeslist;
    double *edge_angles               = gp->g_edge_angles;

    /* Local variables */
    int f, g, i__, j, k, sp, sq;

/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Insert an edge on a face of polyhedral decomposition data */
/*        structure. It is assumed that edge is entirely inside face. */

/*     Input parameters: */
/*        A,B - indices of FACEVERTICESLIST for nonadjacent vertices on same face */
/*        NFACE - number of faces or positions used in FACESDATA array */
/*        NVERT - number of positions used in FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        ADDR_OF_N_POLYHEDRONFACES - number of positions used in POLYHEDRONFACEINDICES array */
/*        ADDR_OF_SIZE_FACEARRAYS - maximum size available for FACESDATA, FACESTYPE, NORMALS arrays */
/*        ADDR_OF_SIZE_FACEVERTEXARRAYS - maximum size available for FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        ADDR_OF_SIZE_POLYHEDRONFACEINDICES - maximum size available for POLYHEDRONFACEINDICES array */
/*        FACESDATA(1:3,1:NFACE) - face pointer list */
/*        FACTYP(1:NFACE) - face types */
/*        NORMALS(1:3,1:NFACE) - unit normal vectors for faces; outward */
/*              normal corresponds to CCW traversal of face from polyh */
/*              with index FACESDATA(2,F) */
/*        FACEVERTICESLIST(1:6,1:NVERT) - face vertex list */
/*        EDGE_ANGLES(1:NVERT) - edge angles */
/*        HFL(1:*) - head pointer to face indices in POLYHEDRONFACEINDICES for each polyh */
/*        POLYHEDRONFACEINDICES(1:2,1:ADDR_OF_N_POLYHEDRONFACES) - list of signed face indices for each polyh */

/*     Updated parameters: */
/*        A - index in FACEVERTICESLIST of new edge; LOC field of output A same as */
/*              input A */
/*        NFACE - increased by 1 */
/*        NVERT - increased by 2 */
/*        ADDR_OF_N_POLYHEDRONFACES - increased by 1 or 2 */
/*        FACESDATA,FACTYP,NORMALS,FACEVERTICESLIST,EDGE_ANGLES,POLYHEDRONFACEINDICES - updated by new edge */
/*        NORMALS - one column added at end */

/*     Abnormal return: */
/*        IERR is set to 15, 16, or 17 */

    /* Parameter adjustments */
    normals -= 4;
    --facestype;
    facesdata -= 4;
    --edge_angles;
    faceverticeslist -= 7;
    polyhedronfaceindices -= 3;
    --hfl;

    /* Function Body */
    f = faceverticeslist[*a * 6 + 2];
    i__ = gp->n_work_vertices + 1;
    j = gp->n_work_vertices + 2;
    gp->n_work_vertices = j;

recheck_size1:
    if ( (gp->n_work_vertices+2) > gp->size_facevertexarrays) {
	int res = 0;
	res = res | gp_increase_memory(gp, (void **)&gp->g_faceverticeslist, &gp->cap_g_faceverticeslist, gp->size_facevertexarrays, 6*sizeof(int));
	res = res | gp_increase_memory(gp, (void **)&gp->g_edge_angles, &gp->cap_g_edge_angles, gp->size_facevertexarrays, sizeof(double));
	if ( res == 0 ) {
	    gp->ierr = 500;
	    return;
	}
//	*addr_of_size_facevertexarrays *= 2;
	gp->size_facevertexarrays += 1024;
	faceverticeslist = gp->g_faceverticeslist;
	faceverticeslist -= 7;
	edge_angles = gp->g_edge_angles;
	--edge_angles;
	goto recheck_size1;
    }

    faceverticeslist[i__ * 6 + 1] = faceverticeslist[*a * 6 + 1];
    faceverticeslist[i__ * 6 + 3] = *b;
    k = faceverticeslist[*a * 6 + 4];
    faceverticeslist[i__ * 6 + 4] = k;
    faceverticeslist[k * 6 + 3] = i__;
    faceverticeslist[i__ * 6 + 5] = j;
    faceverticeslist[i__ * 6 + 6] = j;
    faceverticeslist[j * 6 + 1] = faceverticeslist[*b * 6 + 1];
    faceverticeslist[j * 6 + 2] = f;
    faceverticeslist[j * 6 + 3] = *a;
    k = faceverticeslist[*b * 6 + 4];
    faceverticeslist[j * 6 + 4] = k;
    faceverticeslist[k * 6 + 3] = j;
    faceverticeslist[j * 6 + 5] = i__;
    faceverticeslist[j * 6 + 6] = i__;
    faceverticeslist[*a * 6 + 4] = j;
    faceverticeslist[*b * 6 + 4] = i__;
    edge_angles[i__] = GP_PI;
    edge_angles[j] = GP_PI;
    gp->nface++;

recheck_size2:
    if ( (gp->nface+2) > gp->size_facearrays ) {
	int res = 0;
	res = res | gp_increase_memory(gp, (void **)&gp->g_normals, &gp->cap_g_normals, gp->size_facearrays, 3*sizeof(double));
	res = res | gp_increase_memory(gp, (void **)&gp->g_facesdata, &gp->cap_g_facesdata, gp->size_facearrays, 3*sizeof(int));
	res = res | gp_increase_memory(gp, (void **)&gp->g_facestype, &gp->cap_g_facestype, gp->size_facearrays, sizeof(int));
	if ( res == 0 ) {
	    gp->ierr = 500;
	    return;
	}
//	*addr_of_size_facearrays *= 2;
	gp->size_facearrays += 1024;
	facesdata = gp->g_facesdata;
	facesdata -= 4;
	facestype = gp->g_facestype;
	--facestype;
	normals = gp->g_normals;
	normals -= 4;
	goto recheck_size2;
    }

    facesdata[f * 3 + 1] = *a;
    sp = facesdata[f * 3 + 2];
    sq = facesdata[f * 3 + 3];
    facesdata[gp->nface * 3 + 1] = *b;
    facesdata[gp->nface * 3 + 2] = sp;
    facesdata[gp->nface * 3 + 3] = sq;
    facestype[gp->nface] = facestype[f];
    normals[gp->nface * 3 + 1] = normals[f * 3 + 1];
    normals[gp->nface * 3 + 2] = normals[f * 3 + 2];
    normals[gp->nface * 3 + 3] = normals[f * 3 + 3];
    k = *b;
L10:
    faceverticeslist[k * 6 + 2] = gp->nface;
    k = faceverticeslist[k * 6 + 3];
    if (k != *b) {
	goto L10;
    }
    g = hfl[gp_abs(sp)];
    gp->n_polyhedronfaces++;

recheck_size_3:
    if ( (2+gp->n_polyhedronfaces) > gp->size_polyhedronfaceindices ) {
	int res = gp_increase_memory(gp, (void **)&gp->g_polyhedronfaceindices, &gp->cap_g_polyhedronfaceindices, gp->size_polyhedronfaceindices, 2*sizeof(int));
	if ( res == 0 ) {
	    gp->ierr = 500;
	    return;
	}
//	*addr_of_size_polyhedronfaceindices *= 2;
	gp->size_polyhedronfaceindices += 1024;
	polyhedronfaceindices = gp->g_polyhedronfaceindices;
	polyhedronfaceindices -= 3;
	goto recheck_size_3;
    }

    polyhedronfaceindices[(gp->n_polyhedronfaces << 1) + 1] = i_sign(gp, gp->nface, sp);
    polyhedronfaceindices[(gp->n_polyhedronfaces << 1) + 2] = polyhedronfaceindices[(g << 1) + 2];
    polyhedronfaceindices[(g << 1) + 2] = gp->n_polyhedronfaces;
    if (sq != 0) {
	g = hfl[gp_abs(sq)];
	gp->n_polyhedronfaces++;

recheck_size_4:
	if ( (2+gp->n_polyhedronfaces) > gp->size_polyhedronfaceindices ) {
	    int res = gp_increase_memory(gp, (void **)&gp->g_polyhedronfaceindices, &gp->cap_g_polyhedronfaceindices, gp->size_polyhedronfaceindices, 2*sizeof(int));
	    if ( res == 0 ) {
		gp->ierr = 500;
		return;
	    }
//	    *addr_of_size_polyhedronfaceindices *= 2;
	    gp->size_polyhedronfaceindices += 1024;
	    polyhedronfaceindices = gp->g_polyhedronfaceindices;
	    polyhedronfaceindices -= 3;
	    goto recheck_size_4;
	}

	polyhedronfaceindices[(gp->n_polyhedronfaces << 1) + 1] = i_sign(gp, gp->nface, sq);
	polyhedronfaceindices[(gp->n_polyhedronfaces << 1) + 2] = polyhedronfaceindices[(g << 1) + 2];
	polyhedronfaceindices[(g << 1) + 2] = gp->n_polyhedronfaces;
    } else {
	if ((faceverticeslist[*b * 6 + 1] - faceverticeslist[*a * 6 + 1]) * sp > 0) {
	    faceverticeslist[i__ * 6 + 5] = 0;
	    faceverticeslist[j * 6 + 6] = 0;
	    edge_angles[j] = -1.;
	} else {
	    faceverticeslist[j * 6 + 5] = 0;
	    faceverticeslist[i__ * 6 + 6] = 0;
	    edge_angles[i__] = -1.;
	}
    }
    *a = i__;

    return;
}

/* ---- geompack_ptpolg.cxx ---- */
/* ptpolg.f -- translated by f2c (version 19990311).
*/

static void ptpolg_(ivp_geompack_t *gp, int		dim,
			   int		ldv,
			   int		*nv,
			   int		inc,
			   int		*pgind,
			   double	*vcl,
			   double	*pt,
			   double	*nrml,
			   double	*dtol,
			   int		*inout) {

    /* System generated locals */
    int vcl_dim1, vcl_offset, i__1, i__2;
    double d__1, d__2, d__3;

    /* Local variables */
    double area, dist;
    int h__, i__, j, k, l, m, s;
    double t, armax, da[3], db[3];
    int la, lb;
    double cp[3];
    int sa;
    double ta;
    int sb;
    double nr[4], dir[3], rhs[3];

/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Determine whether a point lies inside, outside, or on */
/*        boundary of a planar polygon in 2 or 3 dimensional space. */
/*        It is assumed that point lies in plane of polygon. */

/*     Input parameters: */
/*        DIM - dimension of polygon (2 or 3) */
/*        LDV - leading dimension of VCL array in calling routine */
/*        NV - number of vertices in polygon */
/*        INC - increment for PGIND indicating indices of polygon */
/*        PGIND(0:NV*INC) - indices in VCL of polygon vertices are in */
/*              PGIND(0), PGIND(INC), ..., PGIND(NV*INC) with first and */
/*              last vertices identical */
/*        VCL(1:DIM,1:*) - vertex coordinate list */
/*        PT(1:DIM) - point for which in/out test is applied */
/*        NRML(1:3) - unit normal vector of plane containing polygon, */
/*              with vertices oriented CCW wrt normal (used iff DIM = 3); */
/*              normal is assumed to be (0,0,1) if DIM = 2 */
/*        DTOL - absolute tolerance to determine whether a point is on */
/*              a line or plane */

/*     Output parameters: */
/*        INOUT - +1, 0, or -1 depending on whether point PT is inside */
/*              polygon, on boundary of polygon, or outside polygon; */
/*              or -2 if error in input parameters */

    /* Parameter adjustments */
    --pt;
    vcl_dim1 = ldv;
    vcl_offset = 1 + vcl_dim1 * 1;
    vcl -= vcl_offset;
    --nrml;

    /* Function Body */
    *inout = -2;
    if (dim < 2 || dim > 3) {
	return;
    }

/*     Find edge subtending max area with PT as third triangle vertex. */

    armax = 0.;
    h__ = 0;
    lb = pgind[0];
    i__1 = dim;
    for (j = 1; j <= i__1; ++j) {
	db[j - 1] = vcl[j + lb * vcl_dim1] - pt[j];
/* L10: */
    }
    i__1 = *nv;
    for (i__ = 1; i__ <= i__1; ++i__) {
	la = lb;
	lb = pgind[i__ * inc];
	i__2 = dim;
	for (j = 1; j <= i__2; ++j) {
	    da[j - 1] = db[j - 1];
	    db[j - 1] = vcl[j + lb * vcl_dim1] - pt[j];
	    dir[j - 1] = vcl[j + lb * vcl_dim1] - vcl[j + la * vcl_dim1];
/* L20: */
	}
	if (dim == 2) {
	    area = (d__1 = da[0] * db[1] - db[0] * da[1], gp_abs(d__1));
	} else {
/* Computing 2nd power */
	    d__1 = da[1] * db[2] - db[1] * da[2];
/* Computing 2nd power */
	    d__2 = da[2] * db[0] - db[2] * da[0];
/* Computing 2nd power */
	    d__3 = da[0] * db[1] - db[0] * da[1];
	    area = d__1 * d__1 + d__2 * d__2 + d__3 * d__3;
	}
	if (area > armax) {
	    h__ = i__;
	    armax = area;
	}
/* L30: */
    }
    if (dim == 2) {
/* Computing 2nd power */
	d__1 = armax;
	armax = d__1 * d__1;
    }
/* Computing 2nd power */
    d__1 = *dtol;
    if (armax <= d__1 * d__1) {
	return;
    }

/*     Direction of ray is from PT through midpoint of edge subtending */
/*     max area. NR is unit normal of line or plane containing ray, */
/*     which is also orthogonal to NRML in 3-D case. */

    la = pgind[(h__ - 1) * inc];
    lb = pgind[h__ * inc];
    dir[0] = (vcl[la * vcl_dim1 + 1] + vcl[lb * vcl_dim1 + 1]) * .5 - pt[1];
    dir[1] = (vcl[la * vcl_dim1 + 2] + vcl[lb * vcl_dim1 + 2]) * .5 - pt[2];
    if (dim == 2) {
/* Computing 2nd power */
	d__1 = dir[0];
/* Computing 2nd power */
	d__2 = dir[1];
	dist = sqrt(d__1 * d__1 + d__2 * d__2);
	dir[0] /= dist;
	dir[1] /= dist;
	dir[2] = 0.;
	nr[0] = -dir[1];
	nr[1] = dir[0];
	nr[3] = nr[0] * pt[1] + nr[1] * pt[2];
	dist = nr[0] * vcl[lb * vcl_dim1 + 1] + nr[1] * vcl[lb * vcl_dim1 + 2]
		 - nr[3];
    } else {
	dir[2] = (vcl[la * vcl_dim1 + 3] + vcl[lb * vcl_dim1 + 3]) * .5 - pt[
		3];
/* Computing 2nd power */
	d__1 = dir[0];
/* Computing 2nd power */
	d__2 = dir[1];
/* Computing 2nd power */
	d__3 = dir[2];
	dist = sqrt(d__1 * d__1 + d__2 * d__2 + d__3 * d__3);
	dir[0] /= dist;
	dir[1] /= dist;
	dir[2] /= dist;
	nr[0] = nrml[2] * dir[2] - nrml[3] * dir[1];
	nr[1] = nrml[3] * dir[0] - nrml[1] * dir[2];
	nr[2] = nrml[1] * dir[1] - nrml[2] * dir[0];
	nr[3] = nr[0] * pt[1] + nr[1] * pt[2] + nr[2] * pt[3];
	dist = nr[0] * vcl[lb * vcl_dim1 + 1] + nr[1] * vcl[lb * vcl_dim1 + 2]
		 + nr[2] * vcl[lb * vcl_dim1 + 3] - nr[3];
    }
    if (dist > 0.) {
	sb = 1;
    } else {
	sb = -1;
    }
    m = 1;
    if (gp_abs(dir[1]) > gp_abs(dir[0])) {
	m = 2;
    }
    if (gp_abs(dir[2]) > (d__1 = dir[m - 1], gp_abs(d__1))) {
	m = 3;
    }
    k = 1;

/*     For remaining edges of polygon, check whether ray intersects edge. */
/*     Vertices or edges lying on ray are handled by looking at preceding */
/*     and succeeding edges not lying on ray. */

    k = 1;
    i__ = h__ + 1;
    if (i__ > *nv) {
	i__ = 1;
    }
L40:
    la = lb;
    lb = pgind[i__ * inc];
    sa = sb;
    if (dim == 2) {
	dist = nr[0] * vcl[lb * vcl_dim1 + 1] + nr[1] * vcl[lb * vcl_dim1 + 2]
		 - nr[3];
    } else {
	dist = nr[0] * vcl[lb * vcl_dim1 + 1] + nr[1] * vcl[lb * vcl_dim1 + 2]
		 + nr[2] * vcl[lb * vcl_dim1 + 3] - nr[3];
    }
    if (gp_abs(dist) <= *dtol) {
	sb = 0;
    } else if (dist > 0.) {
	sb = 1;
    } else {
	sb = -1;
    }
    s = sa * sb;
    if (s < 0) {
	if (dim == 2) {
	    da[0] = vcl[la * vcl_dim1 + 1] - vcl[lb * vcl_dim1 + 1];
	    da[1] = vcl[la * vcl_dim1 + 2] - vcl[lb * vcl_dim1 + 2];
	    rhs[0] = vcl[la * vcl_dim1 + 1] - pt[1];
	    rhs[1] = vcl[la * vcl_dim1 + 2] - pt[2];
	    t = (rhs[0] * da[1] - rhs[1] * da[0]) / (dir[0] * da[1] - dir[1] *
		     da[0]);
	} else {
	    da[0] = vcl[la * vcl_dim1 + 1] - vcl[lb * vcl_dim1 + 1];
	    da[1] = vcl[la * vcl_dim1 + 2] - vcl[lb * vcl_dim1 + 2];
	    da[2] = vcl[la * vcl_dim1 + 3] - vcl[lb * vcl_dim1 + 3];
	    rhs[0] = vcl[la * vcl_dim1 + 1] - pt[1];
	    rhs[1] = vcl[la * vcl_dim1 + 2] - pt[2];
	    rhs[2] = vcl[la * vcl_dim1 + 3] - pt[3];
	    cp[0] = dir[1] * da[2] - dir[2] * da[1];
	    cp[1] = dir[2] * da[0] - dir[0] * da[2];
	    cp[2] = dir[0] * da[1] - dir[1] * da[0];
	    l = 1;
	    if (gp_abs(cp[1]) > gp_abs(cp[0])) {
		l = 2;
	    }
	    if (gp_abs(cp[2]) > (d__1 = cp[l - 1], gp_abs(d__1))) {
		l = 3;
	    }
	    if (l == 1) {
		t = (rhs[1] * da[2] - rhs[2] * da[1]) / cp[0];
	    } else if (l == 2) {
		t = (rhs[2] * da[0] - rhs[0] * da[2]) / cp[1];
	    } else {
		t = (rhs[0] * da[1] - rhs[1] * da[0]) / cp[2];
	    }
	}
	if (t > *dtol) {
	    ++k;
	} else if (t >= -(*dtol)) {
	    *inout = 0;
	    return;
	}
    } else if (s == 0) {
	l = lb;
L50:
	++i__;
	if (i__ > *nv) {
	    i__ = 1;
	}
	if (i__ == h__) {
	    return;
	}
	la = lb;
	lb = pgind[i__ * inc];
	if (dim == 2) {
	    dist = nr[0] * vcl[lb * vcl_dim1 + 1] + nr[1] * vcl[lb * vcl_dim1 
		    + 2] - nr[3];
	} else {
	    dist = nr[0] * vcl[lb * vcl_dim1 + 1] + nr[1] * vcl[lb * vcl_dim1 
		    + 2] + nr[2] * vcl[lb * vcl_dim1 + 3] - nr[3];
	}
	if (gp_abs(dist) <= *dtol) {
	    goto L50;
	} else if (dist > 0.) {
	    sb = 1;
	} else {
	    sb = -1;
	}
	t = (vcl[m + l * vcl_dim1] - pt[m]) / dir[m - 1];
	if (gp_abs(t) <= *dtol) {
	    *inout = 0;
	    return;
	}
	if (la != l) {
	    ta = (vcl[m + la * vcl_dim1] - pt[m]) / dir[m - 1];
	    if (gp_abs(ta) <= *dtol || t * ta < 0.) {
		*inout = 0;
		return;
	    }
	}
	if (sa * sb < 0 && t > 0.) {
	    ++k;
	}
    }
    ++i__;
    if (i__ > *nv) {
	i__ = 1;
    }
    if (i__ != h__) {
	goto L40;
    }

/*     Point lies inside polygon if number of intersections K is odd. */

    if (k % 2 == 1) {
	*inout = 1;
    } else {
	*inout = -1;
    }

    return;
}

/* ---- geompack_dsphdc.cxx ---- */
/* dsphdc.f -- translated by f2c (version 19990311).
*/

static void dsphdc_(ivp_geompack_t *gp) {

    double *vcl			      = gp->g_vcl;
    int    *facesdata                 = gp->g_facesdata;
    int    *faceverticeslist          = gp->g_faceverticeslist;
    double *normals                   = gp->g_normals;
    double *edge_angles               = gp->g_edge_angles;
    int	   *polyhedronfirstfaceoffset = gp->g_polyhedronfirstfaceoffset;
    int	   *polyhedronfaceindices     = gp->g_polyhedronfaceindices;

    /* System generated locals */
    int i__1, i__2, i__3, i__4;
    double d__1, d__2, d__3;

    /* Local variables */
    double leng;
    int last;
    double dotp;
    int f, g, i__, j, k, l;
    long int fflag;
    int p;
    long int gflag;
    double ab[3], ac[3];
    int la, lb, lc;
    double en[3];
    int sf, hdfree;
    double pi2, ang;
    int ccw, nht;

/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Initialize the polyhedral decomposition data structure */
/*        where there are no holes on faces and no interior holes. It is */
/*        assumed head vertex of each face is a strictly convex vertex. */

/*     Input parameters: */
/*        NVC - number of vertex coordinates */
/*        NFACE - number of faces in polyhedral decomposition */
/*        NPOLH - number of polyhedra in decomposition */
/*        VCL(1:3,1:NVC) - vertex coordinate list */
/*        FACESDATA(1,1:NFACE+1) - head pointer to vertex indices in FACEVERTICESLIST for */
/*              each face; 1 = FACESDATA(1,1) < ... < FACESDATA(1,NFACE+1) */
/*        FACEVERTICESLIST(1,1:*) - vertex indices; those for Ith face are in FACEVERTICESLIST(1,J) */
/*              for J = FACESDATA(1,I),...,FACESDATA(1,I+1)-1 */
/*        POLYHEDRONFIRSTFACEOFFSET(1:NPOLH+1) - head pointer to face indices in POLYHEDRONFACEINDICES for each */
/*              polyhedron; 1 = POLYHEDRONFIRSTFACEOFFSET(1) < POLYHEDRONFIRSTFACEOFFSET(2) < ... < POLYHEDRONFIRSTFACEOFFSET(NPOLH+1) */
/*        POLYHEDRONFACEINDICES(1,1:*) - signed face indices; those for Ith polyh are in */
/*              POLYHEDRONFACEINDICES(1,J) for J = POLYHEDRONFIRSTFACEOFFSET(I),...,POLYHEDRONFIRSTFACEOFFSET(I+1)-1; the face index */
/*              must be negated if the ordering of vertices for the face */
/*              in FACEVERTICESLIST is in CW order when viewed from outside Ith polyh */
/*        HASHTABLE_SIZE - size of hash table HT; should be a prime number which */
/*              is >= NVC+2 */
/*        MAXEDG - maximum size available for EDGE array; should be at */
/*              least max number of edges in a polyh of decomposition */

/*     Output parameters: */
/*        FACESDATA(1:3,1:NFACE) - FACESDATA(1,F) same as input; FACESDATA(2,F) and */
/*              FACESDATA(3,F) are signed indices of 2 polyhedra sharing face */
/*              F; if F is boundary face then FACESDATA(3,F) = 0; the sign of */
/*              the polyh index indicates whether face is oriented CCW */
/*              (positive) or CW (negative) in FACEVERTICESLIST when viewed from */
/*              outside polyh; if interior face, 2 signs are different */
/*        NORMALS(1:3,1:NFACE) - normals at faces; NORMALS(*,F) is unit outward */
/*              normal of face F with its vertices oriented CCW when */
/*              viewed from outside polyhedron |FACESDATA(2,F)| */
/*        FACEVERTICESLIST(1:6,1:NVERT) - face vertex list where NVERT = FACESDATA(1, */
/*              NFACE+1)-1; 6 rows are for LOC, FACN, SUCC, PRED, EDGA, */
/*              EDGC; first 4 fields are the same as that used for the */
/*              convex polyhedron data structure (see routine DSCPH). */
/*              EDGA and EDGC give information about the edge UV where */
/*              V = FACEVERTICESLIST(SUCC,U). Let LU = FACEVERTICESLIST(LOC,U), LV = FACEVERTICESLIST(LOC,V), */
/*              and SF = +1 (-1) if face containing UV in polyh P is */
/*              oriented CCW (CW) when viewed from outside P. Let WX be */
/*              edge corresponding to UV in the adjacent face of P, where */
/*              X = FACEVERTICESLIST(SUCC,W). If (LV-LU)*SF > 0, then FACEVERTICESLIST(EDGC,U) = W, */
/*              FACEVERTICESLIST(EDGA,W) = U, and EDGE_ANGLES(U) is angle at UV between the */
/*              2 faces inside P; else FACEVERTICESLIST(EDGA,U) = W, FACEVERTICESLIST(EDGC,W) = U, */
/*              and EDGE_ANGLES(W) is the edge angle. In other words, if P is */
/*              viewed from outside with edge UV directed upwards from */
/*              vertex with smaller LOC value to other vertex, then there */
/*              is a CCW or CW rotation in P from face containing UV to */
/*              other face as indicated by EDGA or EDGC, respectively (A */
/*              for AntiCW, C for CW). If the CCW or CW rotation between */
/*              2 faces is exterior to the region, then the EDGA or EDGC */
/*              value is 0 and EDGE_ANGLES value is -1. */
/*        EDGE_ANGLES(1:NVERT) - angles at edges common to 2 faces in a polyh; */
/*              EDGE_ANGLES(J) corresponds to FACEVERTICESLIST(*,J) and is determined by */
/*              EDGC field */
/*        POLYHEDRONFACEINDICES(1:2,1:NPF) - row 1 same as input and row 2 used for link, */
/*              where NPF = POLYHEDRONFIRSTFACEOFFSET(NPOLH+1)-1 */

/*     Working parameters: */
/*        HT(0:HASHTABLE_SIZE-1),EDGE(1:4,1:MAXEDG) - hash table and edge records */
/*              used to determine matching occurrences of polyhedron */
/*              edges by calling routine EDGHT */

/*     Abnormal return: */
/*        IERR is set to 1, 321, or 322 */

/*     Routines called: */
/*        EDGHT */

    /* Parameter adjustments */
    vcl -= 4;
    normals -= 4;
    facesdata -= 4;
    --polyhedronfirstfaceoffset;
    faceverticeslist -= 7;
    --edge_angles;
    polyhedronfaceindices -= 3;

    /* Function Body */
    pi2 = GP_PI * 2.;
    hdfree = 0;
    last = 0;
    nht = 0;
    i__1 = gp->hashtable_size - 1;
    for (i__ = 0; i__ <= i__1; ++i__) {
	gp->g_hashtable[i__] = 0;
/* L10: */
    }
    i__1 = gp->nface;
    for (i__ = 1; i__ <= i__1; ++i__) {
	facesdata[i__ * 3 + 2] = 0;
	facesdata[i__ * 3 + 3] = 0;
	k = facesdata[i__ * 3 + 1];
	l = facesdata[(i__ + 1) * 3 + 1] - 1;
	i__2 = l;
	for (j = k; j <= i__2; ++j) {
	    faceverticeslist[j * 6 + 2] = i__;
	    faceverticeslist[j * 6 + 3] = j + 1;
	    faceverticeslist[j * 6 + 4] = j - 1;
	    faceverticeslist[j * 6 + 5] = 0;
	    faceverticeslist[j * 6 + 6] = 0;
	    edge_angles[j] = -1.;
/* L20: */
	}
	faceverticeslist[l * 6 + 3] = k;
	faceverticeslist[k * 6 + 4] = l;
/* L30: */
    }
    i__1 = gp->npolh;
    for (i__ = 1; i__ <= i__1; ++i__) {
	k = polyhedronfirstfaceoffset[i__];
	l = polyhedronfirstfaceoffset[i__ + 1] - 1;
	i__2 = l;
	for (j = k; j <= i__2; ++j) {
	    polyhedronfaceindices[(j << 1) + 2] = j + 1;
	    f = polyhedronfaceindices[(j << 1) + 1];
	    p = i_sign(gp, i__, f);
	    f = gp_abs(f);
	    if (facesdata[f * 3 + 2] == 0) {
		facesdata[f * 3 + 2] = p;
	    } else {
		facesdata[f * 3 + 3] = p;
	    }
/* L40: */
	}
	polyhedronfaceindices[(l << 1) + 2] = k;
/* L50: */
    }
    i__1 = gp->nface;
    for (f = 1; f <= i__1; ++f) {
	if (facesdata[f * 3 + 2] * facesdata[f * 3 + 3] > 0) {
	    gp->ierr = 321;
	    IVP_IF(1) {
		IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
		    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "*** GEOMPACK: face oriented same way twice in routine DSPHDC\n");
		}
	    }
	    return;
	}
/* L60: */
    }

/*     Compute normals for each face from orientation in FACESDATA(2,*). */

    i__1 = gp->nface;
    for (f = 1; f <= i__1; ++f) {
	if (facesdata[f * 3 + 2] > 0) {
	    ccw = 3;
	} else {
	    ccw = 4;
	}
	j = facesdata[f * 3 + 1];
	lb = faceverticeslist[j * 6 + 1];
	lc = faceverticeslist[faceverticeslist[ccw + j * 6] * 6 + 1];
	la = faceverticeslist[faceverticeslist[7 - ccw + j * 6] * 6 + 1];
	for (i__ = 1; i__ <= 3; ++i__) {
	    ab[i__ - 1] = vcl[i__ + lb * 3] - vcl[i__ + la * 3];
	    ac[i__ - 1] = vcl[i__ + lc * 3] - vcl[i__ + la * 3];
/* L70: */
	}
	normals[f * 3 + 1] = ab[1] * ac[2] - ab[2] * ac[1];
	normals[f * 3 + 2] = ab[2] * ac[0] - ab[0] * ac[2];
	normals[f * 3 + 3] = ab[0] * ac[1] - ab[1] * ac[0];
/* Computing 2nd power */
	d__1 = normals[f * 3 + 1];
/* Computing 2nd power */
	d__2 = normals[f * 3 + 2];
/* Computing 2nd power */
	d__3 = normals[f * 3 + 3];
	leng = sqrt(d__1 * d__1 + d__2 * d__2 + d__3 * d__3);
	if (leng > 0.) {
	    normals[f * 3 + 1] /= leng;
	    normals[f * 3 + 2] /= leng;
	    normals[f * 3 + 3] /= leng;
	}
/* L80: */
    }

/*     Determine EDGA, EDGC fields and compute EDGE_ANGLES values. */

    i__1 = gp->npolh;
    for (p = 1; p <= i__1; ++p) {
	nht = 0;
	i__2 = polyhedronfirstfaceoffset[p + 1] - 1;
	for (i__ = polyhedronfirstfaceoffset[p]; i__ <= i__2; ++i__) {
	    sf = polyhedronfaceindices[(i__ << 1) + 1];
	    f = gp_abs(sf);
	    i__3 = facesdata[(f + 1) * 3 + 1] - 1;
	    for (j = facesdata[f * 3 + 1]; j <= i__3; ++j) {
		la = faceverticeslist[j * 6 + 1];
		lb = faceverticeslist[faceverticeslist[j * 6 + 3] * 6 + 1];

		// ***********************************************************
		edght_(gp, &la,
		       &lb,
		       &j,
		       &hdfree,
		       &last,
		       &k
		       );

		if (gp->ierr != 0) {
		    return;
		}
		if (k <= 0) {
		    ++nht;
		} else {
		    --nht;
		    g = faceverticeslist[k * 6 + 2];
		    dotp = normals[f * 3 + 1] * normals[g * 3 + 1] + normals[f * 3 + 2]
			     * normals[g * 3 + 2] + normals[f * 3 + 3] * normals[g * 3 
			    + 3];
		    if (gp_abs(dotp) > 1. - gp->tolerance) {
			dotp = d_sign(gp, 1.0, dotp);
		    }
		    fflag = (i__4 = facesdata[f * 3 + 2], gp_abs(i__4)) == p;
		    gflag = (i__4 = facesdata[g * 3 + 2], gp_abs(i__4)) == p;
		    if (fflag != gflag) {
			dotp = -dotp;
		    }
		    ang = GP_PI - acos(dotp);
/*                 Determine whether edge angle is reflex. */
		    for (l = 1; l <= 3; ++l) {
			ab[l - 1] = vcl[l + lb * 3] - vcl[l + la * 3];
/* L90: */
		    }
		    en[0] = normals[f * 3 + 2] * ab[2] - normals[f * 3 + 3] * ab[1];
		    en[1] = normals[f * 3 + 3] * ab[0] - normals[f * 3 + 1] * ab[2];
		    en[2] = normals[f * 3 + 1] * ab[1] - normals[f * 3 + 2] * ab[0];
		    if (fflag != (sf > 0)) {
			en[0] = -en[0];
			en[1] = -en[1];
			en[2] = -en[2];
		    }
/*                 AC = (midpoint of A and B) + EN - A */
		    for (l = 1; l <= 3; ++l) {
			ac[l - 1] = (vcl[l + lb * 3] - vcl[l + la * 3]) * .5 
				+ en[l - 1];
/* L100: */
		    }
		    dotp = ac[0] * normals[g * 3 + 1] + ac[1] * normals[g * 3 + 2] 
			    + ac[2] * normals[g * 3 + 3];
		    if (! gflag) {
			dotp = -dotp;
		    }
		    if (dotp > 0.) {
			ang = pi2 - ang;
		    }
		    if ((lb - la) * sf > 0) {
			faceverticeslist[j * 6 + 6] = k;
			faceverticeslist[k * 6 + 5] = j;
			edge_angles[j] = ang;
		    } else {
			faceverticeslist[j * 6 + 5] = k;
			faceverticeslist[k * 6 + 6] = j;
			edge_angles[k] = ang;
		    }
		}
/* L110: */
	    }
/* L120: */
	}
	if (nht != 0) {
	    gp->ierr = 322;
	    IVP_IF(1) {
		IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
		    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "*** GEOMPACK: unmatched edge determined by routine DSPHDC\n");
		}
	    }
	    return;
	}
/* L130: */
    }

    return;
}

/* ---- geompack_insfac.cxx ---- */
/* insfac.f -- translated by f2c (version 19990311).
*/

static void insfac_(ivp_geompack_t *gp, int		*p,
			   double	*nrmlc,
			   int		*nce,
			   int		*cedge,
			   double	*cdang) {

    int	   *polyhedronfirstfaceoffset = gp->g_polyhedronfirstfaceoffset;
    int	   *polyhedronfaceindices     = gp->g_polyhedronfaceindices;
    int    *facesdata                 = gp->g_facesdata;
    int    *facestype                 = gp->g_facestype;
    double *normals                   = gp->g_normals;
    int    *faceverticeslist          = gp->g_faceverticeslist;
    double *edge_angles               = gp->g_edge_angles;

    /* System generated locals */
    int i__1, i__2;
    double d__1;

    /* Local variables */
    int head;
    long int docf;
    int pind, a, b, c__, f, g, i__, j, k, tailn, tailp, tailt;
    int la, lb;
    int fp, sf, nv, sp, np1;
    double ang;
    long int dof;
    int ptr;

/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Insert a new face (cut face) in polyhedral decomposition */
/*        data structure. It is assumed that interior of face does not */
/*        intersect any other faces. */

/*     Input parameters: */
/*        P - polyhedron index */
/*        NRMLC(1:3) - unit normal vector of cut plane */
/*        NCE - number of edges in cut face */
/*        CEDGE(1:2,0:NCE) - CEDGE(1,I) is an index of VCL, indices > ADDR_OF_N_ORIGINAL_VERTICES */
/*              are new points; CEDGE(2,I) = J indicates that edge of cut */
/*              face ending at CEDGE(1,I) is edge from J to FACEVERTICESLIST(SUCC,J) */
/*              if J > 0; else if J < 0 then edge of cut face ending at */
/*              CEDGE(1,I) is a new edge and CEDGE(1,I) lies on edge from */
/*              -J to FACEVERTICESLIST(SUC,-J) and new edge lies in face FACEVERTICESLIST(FACN,-J); */
/*              CEDGE(2,I) always refers to an edge in the subpolyhedron */
/*              in negative half-space; CEDGE(1,NCE) = CEDGE(1,0); */
/*              CEDGE(2,0) is not input but is used temporarily */
/*        CDANG(1:NCE) - dihedral angles created by edges of cut polygon */
/*              in pos. half-space; neg. sign for angle I indicates that */
/*              face containing edge I is oriented CW in polyhedron P */
/*        ADDR_OF_N_ORIGINAL_VERTICES - number of vertex coordinates (excluding new ones)) */
/*        NFACE - number of faces or positions used in FACESDATA array */
/*        ADDR_OF_N_WORK_VERTICES - number of positions used in FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        NPOLH - number of polyhedra or positions used in POLYHEDRONFIRSTFACEOFFSET array */
/*        ADDR_OF_N_POLYHEDRONFACES - number of positions used in POLYHEDRONFACEINDICES array */
/*        ADDR_OF_SIZE_FACEARRAYS - maximum size available for FACESDATA, FACESTYPE, NORMALS arrays */
/*        ADDR_OF_SIZE_FACEVERTEXARRAYS - maximum size available for FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        ADDR_OF_SIZE_POLYHEDRONFIRSTFACEOFFSET - maximum size available for POLYHEDRONFIRSTFACEOFFSET array */
/*        ADDR_OF_SIZE_POLYHEDRONFACEINDICES - maximum size available for POLYHEDRONFACEINDICES array */
/*        VCL(1:3,1:ADDR_OF_N_ORIGINAL_VERTICES+?) - vertex coordinate list; the new vertices to */
/*           be inserted as indicated by CEDGE are after column ADDR_OF_N_ORIGINAL_VERTICES */
/*        FACESDATA(1:3,1:NFACE) - face pointer list */
/*        FACESTYPE(1:NFACE) - face types */
/*        NORMALS(1:3,1:NFACE) - unit normal vectors for faces */
/*        FACEVERTICESLIST(1:6,1:ADDR_OF_N_WORK_VERTICES) - face vertex list */
/*        EDGE_ANGLES(1:ADDR_OF_N_WORK_VERTICES) - edge angles */
/*        POLYHEDRONFIRSTFACEOFFSET(1:NPOLH) - head ptr to face indices in POLYHEDRONFACEINDICES for each polyh */
/*        POLYHEDRONFACEINDICES(1:2,1:ADDR_OF_N_POLYHEDRONFACES) - list of signed face indices for each polyh */

/*     Updated parameters: */
/*        CEDGE(1:1,1:NCE) - updated to edges of cut face wrt positive */
/*              half-space */
/*        CEDGE(2:2,1:NCE) - neg. entries updated to index of new edge */
/*        ADDR_OF_N_ORIGINAL_VERTICES,NFACE,ADDR_OF_N_WORK_VERTICES,NPOLH,ADDR_OF_N_POLYHEDRONFACES - sizes updated due to cut face */
/*        FACESDATA,FACESTYPE,NORMALS,FACEVERTICESLIST,EDGE_ANGLES,POLYHEDRONFIRSTFACEOFFSET,POLYHEDRONFACEINDICES - updated by cut face */

/*     Routines called: */
/*        INSVR3, INSED3 */

/*     Abnormal return: */
/*        IERR is set to 15, 16, 17, or 18 */

/*     Insert new vertices and update CEDGE(2,*). */

    /* Parameter adjustments */
    --nrmlc;
    --cdang;
    normals -= 4;
    --facestype;
    facesdata -= 4;
    --edge_angles;
    faceverticeslist -= 7;
    --polyhedronfirstfaceoffset;
    polyhedronfaceindices -= 3;

    /* Function Body */
    i__1 = *nce - 1;
    for (i__ = 0; i__ <= i__1; ++i__) {
	if (cedge[(i__ << 1) + 1] <= gp->n_original_vertices ) {
	    goto L10;
	}
	if (i__ == 0) {
	    j = *nce;
	} else {
	    j = i__;
	}
	a = -cedge[(j << 1) + 2];

	// *******************************************************************
	insvr3_(gp, a);

	faceverticeslist = gp->g_faceverticeslist;
	faceverticeslist -= 7;

	edge_angles = gp->g_edge_angles;
	--edge_angles;

	if (gp->ierr != 0) {
	    return;
	}
	if (cdang[j] < 0.) {
	    cedge[(j << 1) + 2] = -faceverticeslist[a * 6 + 3];
	}
L10:
	;
    }

/*     Insert new edges and update CEDGE(2,*). */

    cedge[2] = cedge[(*nce << 1) + 2];
    i__1 = *nce;
    for (i__ = 1; i__ <= i__1; ++i__) {
	b = -cedge[(i__ << 1) + 2];
	if (b < 0) {
	    goto L30;
	}
	f = faceverticeslist[b * 6 + 2];
	la = cedge[((i__-1) << 1) + 1];
	a = cedge[((i__-1) << 1) + 2];
	if (a < 0) {
/*           This can only occur if I = 1. */
	    a = -a;
	    if (faceverticeslist[a * 6 + 1] == la) {
		a = faceverticeslist[a * 6 + 4];
	    } else {
		a = faceverticeslist[a * 6 + 3];
	    }
	}
L20:
	if (faceverticeslist[a * 6 + 1] == la) {
	    a = faceverticeslist[a * 6 + 4];
	    j = la - faceverticeslist[a * 6 + 1];
	    sf = *p;
	} else {
	    a = faceverticeslist[a * 6 + 3];
	    j = faceverticeslist[faceverticeslist[a * 6 + 3] * 6 + 1] - la;
	    sf = -(*p);
	}
	if (j * sf > 0) {
	    a = faceverticeslist[a * 6 + 6];
	} else {
	    a = faceverticeslist[a * 6 + 5];
	}
	fp = faceverticeslist[a * 6 + 2];
	if (fp != f) {
	    goto L20;
	}
	if (faceverticeslist[a * 6 + 1] == la) {
	    j = a;
	    a = faceverticeslist[b * 6 + 3];
	    b = j;
	} else {
	    a = faceverticeslist[a * 6 + 3];
	}

	// *******************************************************************
	insed3_(gp, &a,
		&b,
		&polyhedronfirstfaceoffset[1]
		);

	polyhedronfaceindices = gp->g_polyhedronfaceindices;
	polyhedronfaceindices -= 3;

	facesdata = gp->g_facesdata;
	facesdata -= 4;

	facestype = gp->g_facestype;
	--facestype;

	normals = gp->g_normals;
	normals -= 4;

	faceverticeslist = gp->g_faceverticeslist;
	faceverticeslist -= 7;

	edge_angles = gp->g_edge_angles;
	--edge_angles;

	if (gp->ierr != 0) {
	    return;
	}
	cedge[(i__ << 1) + 2] = a;
L30:
	;
    }

/*     Insert cut face into decomposition data structure. Subpolyhedron */
/*     in negative half space is numbered P, other is numbered NPOLH. */

    gp->nface++;
    gp->npolh++;
    gp->n_polyhedronfaces += 2;
recheck_size:
    if ( (gp->n_work_vertices + *nce + 2) > gp->size_facevertexarrays) {
	int res = 0;
	res = res | gp_increase_memory(gp, (void **)&gp->g_faceverticeslist, &gp->cap_g_faceverticeslist, gp->size_facevertexarrays, 6*sizeof(int));
	res = res | gp_increase_memory(gp, (void **)&gp->g_edge_angles, &gp->cap_g_edge_angles, gp->size_facevertexarrays, sizeof(double));
	if ( res == 0 ) {
	    gp->ierr = 500;
	    return;
	}
//	*addr_of_size_facevertexarrays *= 2;
	gp->size_facevertexarrays +=1024;
	faceverticeslist = gp->g_faceverticeslist;
	faceverticeslist -= 7;
	edge_angles = gp->g_edge_angles;
	--edge_angles;
	goto recheck_size;
    }

    else if ( (gp->nface+2) > gp->size_facearrays ) {
	int res = 0;
	res = res | gp_increase_memory(gp, (void **)&gp->g_normals, &gp->cap_g_normals, gp->size_facearrays, 3*sizeof(double));
	res = res | gp_increase_memory(gp, (void **)&gp->g_facesdata, &gp->cap_g_facesdata, gp->size_facearrays, 3*sizeof(int));
	res = res | gp_increase_memory(gp, (void **)&gp->g_facestype, &gp->cap_g_facestype, gp->size_facearrays, sizeof(int));
	if ( res == 0 ) {
	    gp->ierr = 500;
	    return;
	}
//	*addr_of_size_facearrays *= 2;
	gp->size_facearrays += 1024;
	facesdata = gp->g_facesdata;
	facesdata -= 4;
	facestype = gp->g_facestype;
	--facestype;
	normals = gp->g_normals;
	normals -= 4;
	goto recheck_size;
    }

    else if ( (2+gp->n_polyhedronfaces) > gp->size_polyhedronfaceindices ) {
	int res = gp_increase_memory(gp, (void **)&gp->g_polyhedronfaceindices, &gp->cap_g_polyhedronfaceindices, gp->size_polyhedronfaceindices, 2*sizeof(int));
	if ( res == 0 ) {
	    gp->ierr = 500;
	    return;
	}
//	*addr_of_size_polyhedronfaceindices *= 2;
	gp->size_polyhedronfaceindices += 1024;
	polyhedronfaceindices = gp->g_polyhedronfaceindices;
	polyhedronfaceindices -= 3;
	goto recheck_size;
    }

    else if ( (1+gp->npolh) > gp->size_polyhedronfirstfaceoffset ) {
	int res = gp_increase_memory(gp, (void **)&gp->g_polyhedronfirstfaceoffset, &gp->cap_g_polyhedronfirstfaceoffset, gp->size_polyhedronfirstfaceoffset, sizeof(int));
	if ( res == 0 ) {
	    gp->ierr = 500;
	    return;
	}
//	*addr_of_size_polyhedronfirstfaceoffset *= 2;
	gp->size_polyhedronfirstfaceoffset += 1024;
	polyhedronfirstfaceoffset = gp->g_polyhedronfirstfaceoffset;
	--polyhedronfirstfaceoffset;
	goto recheck_size;
    }

    nv = gp->n_work_vertices;
    facesdata[gp->nface * 3 + 1] = gp->n_work_vertices + 1;
    facesdata[gp->nface * 3 + 2] = *p;
    facesdata[gp->nface * 3 + 3] = -gp->npolh;
    facestype[gp->nface] = 0;
    normals[gp->nface * 3 + 1] = nrmlc[1];
    normals[gp->nface * 3 + 2] = nrmlc[2];
    normals[gp->nface * 3 + 3] = nrmlc[3];
    i__1 = *nce - 1;
    for (i__ = 0; i__ <= i__1; ++i__) {
	gp->n_work_vertices++;
	faceverticeslist[gp->n_work_vertices * 6 + 1] = cedge[(i__ << 1) + 1];
	faceverticeslist[gp->n_work_vertices * 6 + 2] = gp->nface;
	faceverticeslist[gp->n_work_vertices * 6 + 3] = gp->n_work_vertices + 1;
	faceverticeslist[gp->n_work_vertices * 6 + 4] = gp->n_work_vertices - 1;
/* L40: */
    }
    faceverticeslist[gp->n_work_vertices * 6 + 3] = facesdata[gp->nface * 3 + 1];
    faceverticeslist[facesdata[gp->nface * 3 + 1] * 6 + 4] = gp->n_work_vertices;

/*     Set CEDGE(1,*) to edges of face in polyh NPOLH (after split). New */
/*     face is CCW from outside new P which contains edges of CEDGE(2,*). */

    i__1 = *nce;
    for (i__ = 1; i__ <= i__1; ++i__) {
	a = cedge[(i__ << 1) + 2];
	la = faceverticeslist[a * 6 + 1];
	lb = faceverticeslist[faceverticeslist[a * 6 + 3] * 6 + 1];
	if ((lb - la) * cdang[i__] > 0.) {
	    cedge[(i__ << 1) + 1] = faceverticeslist[a * 6 + 6];
	} else {
	    cedge[(i__ << 1) + 1] = faceverticeslist[a * 6 + 5];
	}
/* L50: */
    }

/*     Determine which faces of old P belong to new P or other new polyh, */
/*     and update POLYHEDRONFIRSTFACEOFFSET, POLYHEDRONFACEINDICES. NP1 is used as a temporary polyhedron index. */
/*     Faces of old P are put into FACESDATA(2,*) field. */
/*     FACESTYPE(F) is set to -1 for double occurring faces. */

    dof = 0;
    np1 = gp->npolh + 1;
    j = polyhedronfirstfaceoffset[*p];
L60:
    f = (i__1 = polyhedronfaceindices[(j << 1) + 1], gp_abs(i__1));
    if ((i__1 = facesdata[f * 3 + 2], gp_abs(i__1)) == (i__2 = facesdata[f * 3 + 3], gp_abs(
	    i__2))) {
	facesdata[f * 3 + 2] = np1;
	facesdata[f * 3 + 3] = -np1;
	facestype[f] = -1;
	dof = 1;
    } else if ((i__1 = facesdata[f * 3 + 2], gp_abs(i__1)) == *p) {
	facesdata[f * 3 + 2] = i_sign(gp, np1, facesdata[f * 3 + 2]);
    } else {
	i__ = facesdata[f * 3 + 2];
	facesdata[f * 3 + 2] = i_sign(gp, np1, facesdata[f * 3 + 3]);
	facesdata[f * 3 + 3] = i__;
	normals[f * 3 + 1] = -normals[f * 3 + 1];
	normals[f * 3 + 2] = -normals[f * 3 + 2];
	normals[f * 3 + 3] = -normals[f * 3 + 3];
    }
    j = polyhedronfaceindices[(j << 1) + 2];
    if (j != polyhedronfirstfaceoffset[*p]) {
	goto L60;
    }
    i__1 = *nce;
    for (i__ = 1; i__ <= i__1; ++i__) {
	j = 2;
	f = faceverticeslist[cedge[(i__ << 1) + 1] * 6 + 2];
	if (facestype[f] == -1) {
	    if (faceverticeslist[cedge[(i__ << 1) + 1] * 6 + 1] != faceverticeslist[(nv + i__) * 6 + 1])
		     {
		j = 3;
	    }
	}
	facesdata[j + f * 3] = i_sign(gp, gp->npolh, facesdata[j + f * 3]);
	j = 2;
	f = faceverticeslist[cedge[(i__ << 1) + 2] * 6 + 2];
	if (facestype[f] == -1) {
	    if (cdang[i__] < 0.) {
		j = 3;
	    }
	}
	facesdata[j + f * 3] = i_sign(gp, *p, facesdata[j + f * 3]);
/* L70: */
    }
    polyhedronfaceindices[((gp->n_polyhedronfaces-1) << 1) + 1] = gp->nface;
    polyhedronfaceindices[(gp->n_polyhedronfaces << 1) + 1] = -(gp->nface);
    tailp = gp->n_polyhedronfaces - 1;
    tailn = gp->n_polyhedronfaces;
    tailt = polyhedronfirstfaceoffset[*p];
    ptr = polyhedronfaceindices[(tailt << 1) + 2];
    polyhedronfaceindices[(tailt << 1) + 2] = 0;
    polyhedronfirstfaceoffset[*p] = gp->n_polyhedronfaces - 1;
    polyhedronfirstfaceoffset[gp->npolh] = gp->n_polyhedronfaces;

L80:
    if (ptr == 0) {
	goto L110;
    }
    j = ptr;
    sp = polyhedronfaceindices[(ptr << 1) + 1];
    f = gp_abs(sp);
    ptr = polyhedronfaceindices[(ptr << 1) + 2];
    if (facestype[f] != -1 || sp > 0) {
	k = 2;
    } else {
	k = 3;
    }
    sf = facesdata[k + f * 3];
    if (gp_abs(sf) == *p) {
	polyhedronfaceindices[(tailp << 1) + 2] = j;
	tailp = j;
    } else if (gp_abs(sf) == gp->npolh) {
	polyhedronfaceindices[(tailn << 1) + 2] = j;
	tailn = j;
    } else {
	a = facesdata[f * 3 + 1];
	la = faceverticeslist[a * 6 + 1];
L90:
	b = faceverticeslist[a * 6 + 3];
	lb = faceverticeslist[b * 6 + 1];
	if ((lb - la) * sp > 0) {
	    c__ = faceverticeslist[a * 6 + 6];
	} else {
	    c__ = faceverticeslist[a * 6 + 5];
	}
	g = faceverticeslist[c__ * 6 + 2];
	i__ = 2;
	if (facestype[g] == -1) {
	    if (faceverticeslist[c__ * 6 + 1] == la) {
		if (sp > 0) {
		    i__ = 3;
		}
	    } else {
		if (sp < 0) {
		    i__ = 3;
		}
	    }
	}
	if ((i__1 = facesdata[i__ + g * 3], gp_abs(i__1)) == *p) {
	    polyhedronfaceindices[(tailp << 1) + 2] = j;
	    tailp = j;
	    facesdata[k + f * 3] = i_sign(gp, *p, facesdata[k + f * 3]);
	    goto L100;
	} else if ((i__1 = facesdata[i__ + g * 3], gp_abs(i__1)) == gp->npolh) {
	    polyhedronfaceindices[(tailn << 1) + 2] = j;
	    tailn = j;
	    facesdata[k + f * 3] = i_sign(gp, gp->npolh, facesdata[k + f * 3]);
	    goto L100;
	}
	a = b;
	la = lb;
	if (a != facesdata[f * 3 + 1]) {
	    goto L90;
	}
	polyhedronfaceindices[(tailt << 1) + 2] = j;
	polyhedronfaceindices[(j << 1) + 2] = 0;
	tailt = j;
L100:
	;
    }
    goto L80;
L110:
    polyhedronfaceindices[(tailp << 1) + 2] = gp->n_polyhedronfaces - 1;
    polyhedronfaceindices[(tailn << 1) + 2] = gp->n_polyhedronfaces;

/*     Check whether cut face occurs twice in same polyhedron. */
/*     Temporarily modify PRED field of edges of cut face. */

    docf = 0;
    i__1 = *nce;
    for (i__ = 1; i__ <= i__1; ++i__) {
	for (j = 1; j <= 2; ++j) {
	    a = cedge[j + (i__ << 1)];
	    faceverticeslist[a * 6 + 4] = -faceverticeslist[a * 6 + 4];
/* L120: */
	}
/* L130: */
    }
    for (i__ = 1; i__ <= 2; ++i__) {
	if (i__ == 1) {
	    head = gp->n_polyhedronfaces - 1;
	    pind = *p;
	} else {
	    head = gp->n_polyhedronfaces;
	    pind = gp->npolh;
	}
	ptr = polyhedronfaceindices[(head << 1) + 2];
L140:
	sf = polyhedronfaceindices[(ptr << 1) + 1];
	f = gp_abs(sf);
	a = facesdata[f * 3 + 1];
	la = faceverticeslist[a * 6 + 1];
L150:
	b = faceverticeslist[a * 6 + 3];
	lb = faceverticeslist[b * 6 + 1];
	if (faceverticeslist[a * 6 + 4] > 0) {
	    if ((lb - la) * sf > 0) {
		c__ = faceverticeslist[a * 6 + 6];
	    } else {
		c__ = faceverticeslist[a * 6 + 5];
	    }
	    g = faceverticeslist[c__ * 6 + 2];
	    k = 2;
	    if (facestype[g] == -1) {
		if (faceverticeslist[c__ * 6 + 1] == la) {
		    if (sf > 0) {
			k = 3;
		    }
		} else {
		    if (sf < 0) {
			k = 3;
		    }
		}
	    }
	    if ((i__1 = facesdata[k + g * 3], gp_abs(i__1)) != pind) {
		docf = 1;
		goto L170;
	    }
	}
	a = b;
	la = lb;
	if (a != facesdata[f * 3 + 1]) {
	    goto L150;
	}
	ptr = polyhedronfaceindices[(ptr << 1) + 2];
	if (ptr != head) {
	    goto L140;
	}
/* L160: */
    }

/*     Reset PRED field of edges of cut face. */

L170:
    i__1 = *nce;
    for (i__ = 1; i__ <= i__1; ++i__) {
	for (j = 1; j <= 2; ++j) {
	    a = cedge[j + (i__ << 1)];
	    faceverticeslist[a * 6 + 4] = -faceverticeslist[a * 6 + 4];
/* L180: */
	}
/* L190: */
    }

/*     Update EDGA, EDGC, and EDGE_ANGLES fields. */

    i__1 = *nce;
    for (i__ = 1; i__ <= i__1; ++i__) {
	a = cedge[(i__ << 1) + 2];
	c__ = cedge[(i__ << 1) + 1];
	la = faceverticeslist[a * 6 + 1];
	lb = faceverticeslist[faceverticeslist[a * 6 + 3] * 6 + 1];
	ang = (d__1 = cdang[i__], gp_abs(d__1));
	if ((lb - la) * cdang[i__] > 0.) {
	    faceverticeslist[a * 6 + 6] = nv + i__;
	    faceverticeslist[c__ * 6 + 5] = nv + i__;
	    faceverticeslist[(nv + i__) * 6 + 6] = c__;
	    faceverticeslist[(nv + i__) * 6 + 5] = a;
	    edge_angles[a] -= ang;
	    edge_angles[nv + i__] = ang;
	} else {
	    faceverticeslist[c__ * 6 + 6] = nv + i__;
	    faceverticeslist[a * 6 + 5] = nv + i__;
	    faceverticeslist[(nv + i__) * 6 + 6] = a;
	    faceverticeslist[(nv + i__) * 6 + 5] = c__;
	    edge_angles[nv + i__] = edge_angles[c__] - ang;
	    edge_angles[c__] = ang;
	}
/* L200: */
    }

/*     If DOF, reset FACESTYPE values of -1 to 0. */

    if (dof) {
	for (i__ = 1; i__ <= 2; ++i__) {
	    if (i__ == 1) {
		head = gp->n_polyhedronfaces - 1;
	    } else {
		head = gp->n_polyhedronfaces;
	    }
	    ptr = polyhedronfaceindices[(head << 1) + 2];
L210:
	    f = (i__1 = polyhedronfaceindices[(ptr << 1) + 1], gp_abs(i__1));
	    if (facestype[f] == -1) {
		facestype[f] = 0;
	    }
	    ptr = polyhedronfaceindices[(ptr << 1) + 2];
	    if (ptr != head) {
		goto L210;
	    }
/* L220: */
	}
    }

/*     If cut face is double occurring, set all faces to belong to */
/*     polyhedron P. */

    if (! docf) {
	goto L240;
    }
    gp->npolh--;
    facesdata[gp->nface * 3 + 3] = -(*p);
    tailp = polyhedronfaceindices[((gp->n_polyhedronfaces-1) << 1) + 2];
    polyhedronfaceindices[((gp->n_polyhedronfaces-1) << 1) + 2] = gp->n_polyhedronfaces;
    ptr = gp->n_polyhedronfaces;
L230:
    sf = polyhedronfaceindices[(ptr << 1) + 1];
    f = gp_abs(sf);
    if (sf * facesdata[f * 3 + 2] > 0) {
	facesdata[f * 3 + 2] = i_sign(gp, *p, sf);
    } else {
	facesdata[f * 3 + 3] = -(*p);
    }
    if (polyhedronfaceindices[(ptr << 1) + 2] != gp->n_polyhedronfaces) {
	ptr = polyhedronfaceindices[(ptr << 1) + 2];
	goto L230;
    } else {
	polyhedronfaceindices[(ptr << 1) + 2] = tailp;
    }

L240:
    IVP_IF(1) {
	IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL2) {
	    /* the reference prints vcl[j + la * 3] through the (unshifted)
	     * g_vcl pointer it loaded on entry; insvr3_ may have moved the
	     * array since, the current one is read here */
	    const double *vcl = gp->g_vcl;
	    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "Cut face: #edges, polyh(1:2) = %d %d %d\n", (*nce), facesdata[gp->nface * 3 + 2], facesdata[gp->nface * 3 + 3]);
	    i__1 = *nce;
	    for (i__ = 1; i__ <= i__1; ++i__) {
		la = faceverticeslist[(nv + i__) * 6 + 1];
		ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, " %d %d ", i__, la);
		for (j = 1; j <= 3; ++j) {
		    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "%f ", vcl[j + la * 3]);
		}
		ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "\n");

	    }
	    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "\n");
	}
    }

    return;
}

/* ---- geompack_cutfac.cxx ---- */
/* cutfac.f -- translated by f2c (version 19990311).
*/

static void cutfac_(ivp_geompack_t *gp, int		*p,
			   double	*nrmlc,
			   double	*dtol,
			   int		*nedgc,
			   int		*pedge,
			   int		*nce,
			   int		*cedge,
			   double	*cdang,
			   long int	*rflag) {

    double *vcl              = gp->g_vcl;
    int    *facesdata        = gp->g_facesdata;
    double *normals          = gp->g_normals;
    int    *faceverticeslist = gp->g_faceverticeslist;
    double *edge_angles      = gp->g_edge_angles;

    /* System generated locals */
    int i__1, i__2;
    double d__1, d__2, d__3, d__4;

    /* Local variables */
    double angr, cmax;
    int imin, kmax;
    double dist, nmax, dotp, tmin, ntol;
    int a, e, f, i__, j, k, l = -1, n;
    long int eflag;
    double s, t;
    int w;
    double iamin;
    int ccwfl;
    double dsave[3];
    int isave = 0;
    double dirsq;
    int estop, inout, estrt;
    double dir1sq;
    int ca, cb;
    double de[3];
    int ee, la, lb, fl;
    double cp[3];
    int fr, sf, lv, lw, sp;
    double intang;
    double pi2;
    int lw1;
    double dee[3], ang;
    long int dof;
    double dir[3];
    int nev, sgn;
    double rhs[3], dir1[3];

/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Trace out cut face in polyhedron P given a starting edge, */
/*        e.g. a reflex edge or an edge in the interior of a face. */
/*        Accept cut face if it creates no small angles, it is an outer */
/*        boundary and there are no holes (inner polygons) in it. It is */
/*        assumed starting edge does not lie on a double-occurring face. */

/*     Input parameters: */
/*        P - polyhedron index */
/*        NRMLC(1:4) - unit normal vector of cut plane plus right hand */
/*              side constant term of plane equation */
/*        ANGACC - min acceptable dihedral angle in radians produced by */
/*              a cut face */
/*        DTOL - absolute tolerance to determine if point is on cut plane */
/*        ADDR_OF_N_ORIGINAL_VERTICES - number of vertex coordinates */
/*        SIZE_VCL - maximum size available for VCL array */
/*        VCL(1:3,1:ADDR_OF_N_ORIGINAL_VERTICES) - vertex coordinate list */
/*        FACESDATA(1:3,1:*) - face pointer list */
/*        NRML(1:3,1:*) - unit normal vectors for faces */
/*        FACEVERTICESLIST(1:6,1:*) - face vertex list */
/*        EDGE_ANGLES(1:*) - edge angles */
/*        NEDGC - number of edges which intersect cut plane */
/*        PEDGE(1:3,1:NEDGC) - edges of polyh P that intersect cut plane */
/*              excluding starting edge if it is an edge of P or the edge */
/*              containing CEDGE(1,1) if CEDGE(1,1) > ADDR_OF_N_ORIGINAL_VERTICES; */
/*              PEDGE(1,I), PEDGE(2,I) are indices of FACEVERTICESLIST; if PEDGE(1,I) */
/*              = A and B = FACEVERTICESLIST(SUCC,A) then PEDGE(3,I) = 10*CA+CB where */
/*              CA = 1, 2, or 3 if vertex A in negative half-space, on */
/*              cut plane, or in positive half-space determined by cut */
/*              plane, and similarly for CB */
/*        CEDGE(1:2,0:1) - CEDGE(1,0) = LV, CEDGE(1,1) = LU where LU, LV */
/*              are indices of VCL and are vertices of starting edge; if */
/*              called by RESEDG, LU < LV <= ADDR_OF_N_ORIGINAL_VERTICES are on reflex edge and */
/*              CEDGE(2,1) = U is index of FACEVERTICESLIST indicating reflex edge; */
/*              LV may be ADDR_OF_N_ORIGINAL_VERTICES+1 and LU may be ADDR_OF_N_ORIGINAL_VERTICES+1 or ADDR_OF_N_ORIGINAL_VERTICES+2 to indicate */
/*              a point on interior of an edge; CEDGE(2,1) is specified */
/*              as described for output below; if LV > ADDR_OF_N_ORIGINAL_VERTICES, CEDGE(2,0) */
/*              takes on the value ABS(CEDGE(2,NCE)) described for output */
/*              below, else CEDGE(2,0) is not used */
/*        CDANG(1) - dihedral angle at starting edge determined by cut */
/*              plane in positive half-space */

/*     Updated parameters: */
/*        VCL - some temporary or permanent entries may be added at end */
/*              of array */
/*        NEDGC - may be decreased to indicate number of unchecked edges */
/*        PEDGE(1:3,1:NEDGC) - columns may be permuted (NEDGC=input val) */

/*     Output parameters: */
/*        NCE - number of edges in cut polygon; it is assumed there is */
/*              enough space in the following two arrays */
/*        CEDGE(1:2,0:NCE) - CEDGE(1,I) is an index of VCL, indices > ADDR_OF_N_ORIGINAL_VERTICES */
/*              are new points; CEDGE(2,I) = J indicates that edge of cut */
/*              face ending at CEDGE(1,I) is edge from J to FACEVERTICESLIST(SUCC,J) */
/*              if J > 0; else if J < 0 then edge of cut face ending at */
/*              CEDGE(1,I) is a new edge and CEDGE(1,I) lies on edge from */
/*              -J to FACEVERTICESLIST(SUC,-J) and new edge lies in face FACEVERTICESLIST(FACN,-J); */
/*              CEDGE(2,I) always refers to an edge in the subpolyhedron */
/*              in negative half-space; CEDGE(1,NCE) = CEDGE(1,0) */
/*        CDANG(1:NCE) - dihedral angles created by edges of cut polygon */
/*              in pos. half-space; neg. sign for angle I indicates that */
/*              face containing edge I is oriented CW in polyhedron P */
/*        RFLAG - .TRUE. iff reflex or starting edge is resolved */

/*     Routines called: */
/*        PTPOLG */

/*     Abnormal return: */
/*        IERR is set to 14, 325, 326, or 328 */

    /* Parameter adjustments */
    --nrmlc;
    vcl -= 4;
    facesdata -= 4;
    normals -= 4;
    faceverticeslist -= 7;
    --edge_angles;
    --cdang;

    /* Function Body */
    *rflag = 0;
    pi2 = GP_PI + GP_PI;
/* Computing MAX */
    i__1 = gp_max(gp->n_original_vertices, cedge[1]);
    n = gp_max(i__1,cedge[3]);
    *nce = 1;
    w = cedge[4];
    lv = cedge[1];
    lw = lv;
    lw1 = cedge[3];
    if (lw1 <= gp->n_original_vertices ) {
	nev = 1;
	gp->g_ev[nev - 1] = lw1;
    } else {
	nev = 0;
    }
    if (w > 0) {
	fl = faceverticeslist[w * 6 + 2];
	if (lw > lw1) {
	    fr = faceverticeslist[faceverticeslist[w * 6 + 6] * 6 + 2];
	} else {
	    fr = faceverticeslist[faceverticeslist[w * 6 + 5] * 6 + 2];
	}
    } else {
	fl = faceverticeslist[-w * 6 + 2];
	fr = fl;
    }
    dir[0] = vcl[lw1 * 3 + 1] - vcl[lw * 3 + 1];
    dir[1] = vcl[lw1 * 3 + 2] - vcl[lw * 3 + 2];
    dir[2] = vcl[lw1 * 3 + 3] - vcl[lw * 3 + 3];
    kmax = 1;
    if (gp_abs(nrmlc[2]) > gp_abs(nrmlc[1])) {
	kmax = 2;
    }
    if (gp_abs(nrmlc[3]) > (d__1 = nrmlc[kmax], gp_abs(d__1))) {
	kmax = 3;
    }
    if ((i__1 = facesdata[fl * 3 + 2], gp_abs(i__1)) == *p) {
	ccwfl = facesdata[fl * 3 + 2];
    } else {
	ccwfl = facesdata[fl * 3 + 3];
    }
    if (ccwfl < 0) {
	cdang[1] = -cdang[1];
    }

/*     LW, LW1, FL, FR, DIR are updated before each iteration of loop. */
/*     CCWFL = P (-P) if FL is CCW (CW) according to SUCC traversal. */

L10:
    if (lw1 > gp->n_original_vertices ) {

/*           LW1 is new vertex on interior of edge E. FL is used for */
/*           previous and next faces containing edges of cut polygon. */

	e = -cedge[(*nce << 1) + 2];
	la = faceverticeslist[e * 6 + 1];
	lb = faceverticeslist[faceverticeslist[e * 6 + 3] * 6 + 1];
	if ((lb - la) * ccwfl > 0) {
	    ee = faceverticeslist[e * 6 + 6];
	} else {
	    ee = faceverticeslist[e * 6 + 5];
	}
	fl = faceverticeslist[ee * 6 + 2];
	dof = (i__1 = facesdata[fl * 3 + 2], gp_abs(i__1)) == (i__2 = facesdata[fl * 3 + 
		3], gp_abs(i__2));
	if (dof) {
	    l = faceverticeslist[ee * 6 + 1];
	    if (l == la) {
		ccwfl = -ccwfl;
	    }
	} else if ((i__1 = facesdata[fl * 3 + 2], gp_abs(i__1)) == *p) {
	    ccwfl = facesdata[fl * 3 + 2];
	} else {
	    ccwfl = facesdata[fl * 3 + 3];
	}
	dir[0] = nrmlc[2] * normals[fl * 3 + 3] - nrmlc[3] * normals[fl * 3 + 2];
	dir[1] = nrmlc[3] * normals[fl * 3 + 1] - nrmlc[1] * normals[fl * 3 + 3];
	dir[2] = nrmlc[1] * normals[fl * 3 + 2] - nrmlc[2] * normals[fl * 3 + 1];
	if ((i__1 = facesdata[fl * 3 + 2], gp_abs(i__1)) != *p || (dof && ccwfl < 0)) {
	    dir[0] = -dir[0];
	    dir[1] = -dir[1];
	    dir[2] = -dir[2];
	}
	goto L70;
    } else {

/*           LW1 is existing vertex of polyhedron P. FL (FL and FR) is */
/*           previous face if edge ending at LW1 is new (already exists). */
/*           In former case, -CEDGE(2,NCE) is the edge of FL incident on */
/*           LW1 which will lie only in subpolyhedron PL. In latter case, */
/*           CEDGE(2,NCE) is an edge of FL. Cycle thru edges, faces CCW */
/*           (from outside) between edges ESTRT, ESTOP. */
/*           If LW1 lies on a doubly-occurring face, there are 2 cycles */
/*           around LW1 and the correct one is chosen based on CCWFL. */

	iamin = pi2;
	imin = 0;
/* Computing 2nd power */
	d__1 = dir[0];
/* Computing 2nd power */
	d__2 = dir[1];
/* Computing 2nd power */
	d__3 = dir[2];
	dirsq = d__1 * d__1 + d__2 * d__2 + d__3 * d__3;
	eflag = cedge[(*nce << 1) + 2] > 0;
	if (! eflag) {
	    estrt = -cedge[(*nce << 1) + 2];
	    sp = ccwfl;
	    if (ccwfl > 0) {
		estop = faceverticeslist[estrt * 6 + 3];
	    } else {
		estop = faceverticeslist[estrt * 6 + 4];
	    }
	} else {
	    w = cedge[(*nce << 1) + 2];
	    la = faceverticeslist[w * 6 + 1];
	    if (ccwfl > 0) {
		estrt = faceverticeslist[w * 6 + 4];
	    } else {
		estrt = faceverticeslist[w * 6 + 3];
	    }
	    if (la == lw) {
		l = lw1 - lw;
	    } else {
		l = lw - lw1;
	    }
	    if (l * ccwfl > 0) {
		w = faceverticeslist[w * 6 + 6];
	    } else {
		w = faceverticeslist[w * 6 + 5];
	    }
	    if ((i__1 = facesdata[fr * 3 + 2], gp_abs(i__1)) == (i__2 = facesdata[fr * 3 
		    + 3], gp_abs(i__2))) {
		lb = faceverticeslist[w * 6 + 1];
		if (la == lb) {
		    sp = -ccwfl;
		} else {
		    sp = ccwfl;
		}
	    } else if ((i__1 = facesdata[fr * 3 + 2], gp_abs(i__1)) == *p) {
		sp = facesdata[fr * 3 + 2];
	    } else {
		sp = facesdata[fr * 3 + 3];
	    }
	    if (sp > 0) {
		estop = faceverticeslist[w * 6 + 3];
	    } else {
		estop = faceverticeslist[w * 6 + 4];
	    }
	}
	la = faceverticeslist[estop * 6 + 1];
	lb = faceverticeslist[faceverticeslist[estop * 6 + 3] * 6 + 1];
	if ((lb - la) * sp > 0) {
	    estop = faceverticeslist[estop * 6 + 6];
	} else {
	    estop = faceverticeslist[estop * 6 + 5];
	}
	e = estrt;
	sf = ccwfl;
L20:
	if (eflag || (e != estrt && e != estop)) {
	    if (faceverticeslist[e * 6 + 1] == lw1) {
		l = faceverticeslist[faceverticeslist[e * 6 + 3] * 6 + 1];
	    } else {
		l = faceverticeslist[e * 6 + 1];
	    }
	    dist = nrmlc[1] * vcl[l * 3 + 1] + nrmlc[2] * vcl[l * 3 + 2] + 
		    nrmlc[3] * vcl[l * 3 + 3] - nrmlc[4];
	    if (gp_abs(dist) <= *dtol) {
		dir1[0] = vcl[l * 3 + 1] - vcl[lw1 * 3 + 1];
		dir1[1] = vcl[l * 3 + 2] - vcl[lw1 * 3 + 2];
		dir1[2] = vcl[l * 3 + 3] - vcl[lw1 * 3 + 3];
/* Computing 2nd power */
		d__1 = dir1[0];
/* Computing 2nd power */
		d__2 = dir1[1];
/* Computing 2nd power */
		d__3 = dir1[2];
		dir1sq = d__1 * d__1 + d__2 * d__2 + d__3 * d__3;
		dotp = -(dir[0] * dir1[0] + dir[1] * dir1[1] + dir[2] * dir1[
			2]) / sqrt(dirsq * dir1sq);
		if (gp_abs(dotp) > 1. - gp->tolerance) {
		    dotp = d_sign(gp, 1.0, dotp);
		}
		if (kmax == 1) {
		    cp[0] = dir[1] * dir1[2] - dir[2] * dir1[1];
		} else if (kmax == 2) {
		    cp[1] = dir[2] * dir1[0] - dir[0] * dir1[2];
		} else {
		    cp[2] = dir[0] * dir1[1] - dir[1] * dir1[0];
		}
/* Computing MAX */
		d__2 = gp_abs(dir[0]), d__3 = gp_abs(dir[1]), d__2 = gp_max(d__2,d__3),
			 d__3 = gp_abs(dir[2]), d__2 = gp_max(d__2,d__3), d__3 = 
			gp_abs(dir1[0]), d__2 = gp_max(d__2,d__3), d__3 = gp_abs(dir1[
			1]), d__2 = gp_max(d__2,d__3), d__3 = gp_abs(dir1[2]);
		if ((d__1 = cp[kmax - 1], gp_abs(d__1)) <= gp->tolerance * gp_max(
			d__2,d__3)) {
		    intang = GP_PI;
		} else if (cp[kmax - 1] * nrmlc[kmax] > 0.) {
		    intang = acos(dotp);
		} else {
		    intang = pi2 - acos(dotp);
		}
		if (intang < iamin) {
		    iamin = intang;
		    imin = e;
		    dsave[0] = dir1[0];
		    dsave[1] = dir1[1];
		    dsave[2] = dir1[2];
		}
	    }
	}

	if (e == estop) {
	    goto L40;
	}
	la = faceverticeslist[e * 6 + 1];
	lb = faceverticeslist[faceverticeslist[e * 6 + 3] * 6 + 1];
	if ((lb - la) * sf > 0) {
	    e = faceverticeslist[e * 6 + 6];
	} else {
	    e = faceverticeslist[e * 6 + 5];
	}
	f = faceverticeslist[e * 6 + 2];
	dof = (i__1 = facesdata[f * 3 + 2], gp_abs(i__1)) == (i__2 = facesdata[f * 3 + 3]
		, gp_abs(i__2));
	if (dof) {
	    l = faceverticeslist[e * 6 + 1];
	    if (l == la) {
		sf = -sf;
	    }
	} else if ((i__1 = facesdata[f * 3 + 2], gp_abs(i__1)) == *p) {
	    sf = facesdata[f * 3 + 2];
	} else {
	    sf = facesdata[f * 3 + 3];
	}
	if (sf > 0) {
	    ee = faceverticeslist[e * 6 + 4];
	    la = faceverticeslist[faceverticeslist[e * 6 + 3] * 6 + 1];
	    lb = faceverticeslist[ee * 6 + 1];
	} else {
	    ee = faceverticeslist[e * 6 + 3];
	    la = faceverticeslist[e * 6 + 1];
	    lb = faceverticeslist[faceverticeslist[ee * 6 + 3] * 6 + 1];
	}
	dir1[0] = nrmlc[2] * normals[f * 3 + 3] - nrmlc[3] * normals[f * 3 + 2];
	dir1[1] = nrmlc[3] * normals[f * 3 + 1] - nrmlc[1] * normals[f * 3 + 3];
	dir1[2] = nrmlc[1] * normals[f * 3 + 2] - nrmlc[2] * normals[f * 3 + 1];
/* Computing MAX */
	d__1 = gp_abs(dir1[0]), d__2 = gp_abs(dir1[1]), d__1 = gp_max(d__1,d__2), d__2 
		= gp_abs(dir1[2]);
	if (gp_max(d__1,d__2) <= gp->tolerance) {
	    goto L30;
	}
	sgn = 1;
	if ((i__1 = facesdata[f * 3 + 2], gp_abs(i__1)) != *p || (dof && sf < 0)) {
	    dir1[0] = -dir1[0];
	    dir1[1] = -dir1[1];
	    dir1[2] = -dir1[2];
	    sgn = -1;
	}
	k = 1;
	if ((d__1 = normals[f * 3 + 2], gp_abs(d__1)) > (d__2 = normals[f * 3 + 1], 
		gp_abs(d__2))) {
	    k = 2;
	}
	if ((d__1 = normals[f * 3 + 3], gp_abs(d__1)) > (d__2 = normals[k + f * 3], 
		gp_abs(d__2))) {
	    k = 3;
	}
	nmax = sgn * normals[k + f * 3];
	de[0] = vcl[la * 3 + 1] - vcl[lw1 * 3 + 1];
	de[1] = vcl[la * 3 + 2] - vcl[lw1 * 3 + 2];
	de[2] = vcl[la * 3 + 3] - vcl[lw1 * 3 + 3];
	dee[0] = vcl[lb * 3 + 1] - vcl[lw1 * 3 + 1];
	dee[1] = vcl[lb * 3 + 2] - vcl[lw1 * 3 + 2];
	dee[2] = vcl[lb * 3 + 3] - vcl[lw1 * 3 + 3];
/* Computing MAX */
	d__1 = gp_abs(de[0]), d__2 = gp_abs(de[1]), d__1 = gp_max(d__1,d__2), d__2 = 
		gp_abs(de[2]), d__1 = gp_max(d__1,d__2), d__2 = gp_abs(dee[0]), d__1 = 
		gp_max(d__1,d__2), d__2 = gp_abs(dee[1]), d__1 = gp_max(d__1,d__2), 
		d__2 = gp_abs(dee[2]);
	ntol = gp->tolerance * gp_max(d__1,d__2);
	if (k == 1) {
	    cp[0] = de[1] * dee[2] - de[2] * dee[1];
	} else if (k == 2) {
	    cp[1] = de[2] * dee[0] - de[0] * dee[2];
	} else {
	    cp[2] = de[0] * dee[1] - de[1] * dee[0];
	}
	if ((d__1 = cp[k - 1], gp_abs(d__1)) <= ntol || cp[k - 1] * nmax > 0.) {
	    if (k == 1) {
		cp[0] = de[1] * dir1[2] - de[2] * dir1[1];
	    }
	    if (k == 2) {
		cp[1] = de[2] * dir1[0] - de[0] * dir1[2];
	    }
	    if (k == 3) {
		cp[2] = de[0] * dir1[1] - de[1] * dir1[0];
	    }
	    if ((d__1 = cp[k - 1], gp_abs(d__1)) <= ntol || cp[k - 1] * nmax < 
		    0.) {
		goto L30;
	    }
	    if (k == 1) {
		cp[0] = dir1[1] * dee[2] - dir1[2] * dee[1];
	    }
	    if (k == 2) {
		cp[1] = dir1[2] * dee[0] - dir1[0] * dee[2];
	    }
	    if (k == 3) {
		cp[2] = dir1[0] * dee[1] - dir1[1] * dee[0];
	    }
	    if ((d__1 = cp[k - 1], gp_abs(d__1)) <= ntol || cp[k - 1] * nmax < 
		    0.) {
		goto L30;
	    }
	} else {
	    if (k == 1) {
		cp[0] = dir1[1] * de[2] - dir1[2] * de[1];
	    }
	    if (k == 2) {
		cp[1] = dir1[2] * de[0] - dir1[0] * de[2];
	    }
	    if (k == 3) {
		cp[2] = dir1[0] * de[1] - dir1[1] * de[0];
	    }
	    if ((d__1 = cp[k - 1], gp_abs(d__1)) <= ntol || cp[k - 1] * nmax > 
		    0.) {
		if (k == 1) {
		    cp[0] = dee[1] * dir1[2] - dee[2] * dir1[1];
		}
		if (k == 2) {
		    cp[1] = dee[2] * dir1[0] - dee[0] * dir1[2];
		}
		if (k == 3) {
		    cp[2] = dee[0] * dir1[1] - dee[1] * dir1[0];
		}
		if ((d__1 = cp[k - 1], gp_abs(d__1)) <= ntol || cp[k - 1] * nmax 
			> 0.) {
		    goto L30;
		}
	    }
	}
/* Computing 2nd power */
	d__1 = dir1[0];
/* Computing 2nd power */
	d__2 = dir1[1];
/* Computing 2nd power */
	d__3 = dir1[2];
	dir1sq = d__1 * d__1 + d__2 * d__2 + d__3 * d__3;
	dotp = -(dir[0] * dir1[0] + dir[1] * dir1[1] + dir[2] * dir1[2]) / 
		sqrt(dirsq * dir1sq);
	if (gp_abs(dotp) > 1. - gp->tolerance) {
	    dotp = d_sign(gp, 1.0, dotp);
	}
	if (kmax == 1) {
	    cp[0] = dir[1] * dir1[2] - dir[2] * dir1[1];
	} else if (kmax == 2) {
	    cp[1] = dir[2] * dir1[0] - dir[0] * dir1[2];
	} else {
	    cp[2] = dir[0] * dir1[1] - dir[1] * dir1[0];
	}
/* Computing MAX */
	d__2 = gp_abs(dir[0]), d__3 = gp_abs(dir[1]), d__2 = gp_max(d__2,d__3), d__3 = 
		gp_abs(dir[2]), d__2 = gp_max(d__2,d__3), d__3 = gp_abs(dir1[0]), d__2 
		= gp_max(d__2,d__3), d__3 = gp_abs(dir1[1]), d__2 = gp_max(d__2,d__3), 
		d__3 = gp_abs(dir1[2]);
	if ((d__1 = cp[kmax - 1], gp_abs(d__1)) <= gp->tolerance * gp_max(d__2,d__3))
		 {
	    intang = GP_PI;
	} else if (cp[kmax - 1] * nrmlc[kmax] > 0.) {
	    intang = acos(dotp);
	} else {
	    intang = pi2 - acos(dotp);
	}
	if (intang < iamin) {
	    iamin = intang;
	    imin = -f;
	    ccwfl = sf;
	    dsave[0] = dir1[0];
	    dsave[1] = dir1[1];
	    dsave[2] = dir1[2];
	}
L30:
	e = ee;
	goto L20;

L40:
	if (imin == 0) {
/* 	       IERR = 325 */
	    return;
	} else if (imin > 0) {
	    dir[0] = dsave[0];
	    dir[1] = dsave[1];
	    dir[2] = dsave[2];
	    lw = lw1;
	    la = faceverticeslist[imin * 6 + 1];
	    lb = faceverticeslist[faceverticeslist[imin * 6 + 3] * 6 + 1];
	    if (la == lw1) {
		lw1 = lb;
	    } else {
		lw1 = la;
	    }
	    ++(*nce);
	    cedge[(*nce << 1) + 1] = lw1;
	    cedge[(*nce << 1) + 2] = imin;
	    fl = faceverticeslist[imin * 6 + 2];
	    dof = (i__1 = facesdata[fl * 3 + 2], gp_abs(i__1)) == (i__2 = facesdata[fl * 
		    3 + 3], gp_abs(i__2));
	    if (dof) {
		if (la == lw) {
		    ccwfl = -(*p);
		} else {
		    ccwfl = *p;
		}
	    } else if ((i__1 = facesdata[fl * 3 + 2], gp_abs(i__1)) == *p) {
		ccwfl = facesdata[fl * 3 + 2];
	    } else {
		ccwfl = facesdata[fl * 3 + 3];
	    }
	    if ((lb - la) * ccwfl > 0) {
		fr = faceverticeslist[faceverticeslist[imin * 6 + 6] * 6 + 2];
	    } else {
		fr = faceverticeslist[faceverticeslist[imin * 6 + 5] * 6 + 2];
	    }
	    k = 1;
L50:
	    if (pedge[k * 3 + 1] == imin || pedge[k * 3 + 2] == imin) {
		for (i__ = 1; i__ <= 3; ++i__) {
		    j = pedge[i__ + k * 3];
		    pedge[i__ + k * 3] = pedge[i__ + *nedgc * 3];
		    pedge[i__ + *nedgc * 3] = j;
/* L60: */
		}
		--(*nedgc);
	    } else {
		++k;
		goto L50;
	    }
	    goto L110;
	} else {
	    dir[0] = dsave[0];
	    dir[1] = dsave[1];
	    dir[2] = dsave[2];
	    fl = -imin;
	    goto L70;
	}
    }

/*        Determine LW1 from direction DIR in interior of face FL. */

L70:
    lw = lw1;
    fr = 0;
    imin = 0;
    tmin = 0.;
    k = 1;
    if (gp_abs(dir[1]) > gp_abs(dir[0])) {
	k = 2;
    }
    if (gp_abs(dir[2]) > (d__1 = dir[k - 1], gp_abs(d__1))) {
	k = 3;
    }
    ntol = gp->tolerance * (d__1 = dir[k - 1], gp_abs(d__1));
    i__1 = *nedgc;
    for (i__ = 1; i__ <= i__1; ++i__) {
	e = pedge[i__ * 3 + 1];
	ee = pedge[i__ * 3 + 2];
	if (faceverticeslist[e * 6 + 2] == fl) {
	    a = e;
	} else if (faceverticeslist[ee * 6 + 2] == fl) {
	    a = ee;
	} else {
	    goto L80;
	}
	ca = pedge[i__ * 3 + 3] / 10;
	cb = pedge[i__ * 3 + 3] % 10;
	if (ca == 2) {
	    la = faceverticeslist[a * 6 + 1];
	    if (cb == 2) {
		lb = faceverticeslist[faceverticeslist[a * 6 + 3] * 6 + 1];
		s = (vcl[k + la * 3] - vcl[k + lw * 3]) / dir[k - 1];
		t = (vcl[k + lb * 3] - vcl[k + lw * 3]) / dir[k - 1];
		if (s > 0.) {
		    if (gp_min(s,t) < tmin || imin == 0) {
			if (s < t) {
			    if (ccwfl < 0) {
				imin = a;
				lw1 = la;
				tmin = s;
			    }
			} else {
			    if (ccwfl > 0) {
				imin = a;
				lw1 = lb;
				tmin = t;
			    }
			}
		    }
		}
	    } else {
		l = faceverticeslist[e * 6 + 1];
		if ((l == la && ccwfl < 0) || (l != la && ccwfl > 0)) {
		    t = (vcl[k + l * 3] - vcl[k + lw * 3]) / dir[k - 1];
		    if (t > ntol) {
			if (t < tmin || imin == 0) {
			    lw1 = l;
			    imin = a;
			    tmin = t;
			}
		    }
		}
	    }
	} else if (cb == 2) {
	    la = faceverticeslist[a * 6 + 1];
	    l = faceverticeslist[faceverticeslist[e * 6 + 3] * 6 + 1];
	    if ((l == la && ccwfl < 0) || (l != la && ccwfl > 0)) {
		t = (vcl[k + l * 3] - vcl[k + lw * 3]) / dir[k - 1];
		if (t > ntol) {
		    if (t < tmin || imin == 0) {
			lw1 = l;
			imin = a;
			tmin = t;
		    }
		}
	    }
	} else {
	    la = faceverticeslist[e * 6 + 1];
	    lb = faceverticeslist[faceverticeslist[e * 6 + 3] * 6 + 1];
	    dir1[0] = vcl[la * 3 + 1] - vcl[lb * 3 + 1];
	    dir1[1] = vcl[la * 3 + 2] - vcl[lb * 3 + 2];
	    dir1[2] = vcl[la * 3 + 3] - vcl[lb * 3 + 3];
	    rhs[0] = vcl[la * 3 + 1] - vcl[lw * 3 + 1];
	    rhs[1] = vcl[la * 3 + 2] - vcl[lw * 3 + 2];
	    rhs[2] = vcl[la * 3 + 3] - vcl[lw * 3 + 3];
	    cp[0] = dir[1] * dir1[2] - dir[2] * dir1[1];
	    cp[1] = dir[2] * dir1[0] - dir[0] * dir1[2];
	    cp[2] = dir[0] * dir1[1] - dir[1] * dir1[0];
	    l = 1;
	    if (gp_abs(cp[1]) > gp_abs(cp[0])) {
		l = 2;
	    }
	    if (gp_abs(cp[2]) > (d__1 = cp[l - 1], gp_abs(d__1))) {
		l = 3;
	    }
	    if (l == 1) {
		t = (rhs[1] * dir1[2] - rhs[2] * dir1[1]) / cp[0];
	    } else if (l == 2) {
		t = (rhs[2] * dir1[0] - rhs[0] * dir1[2]) / cp[1];
	    } else {
		t = (rhs[0] * dir1[1] - rhs[1] * dir1[0]) / cp[2];
	    }
	    if (t > ntol) {
		if (t < tmin || imin == 0) {
		    imin = -a;
		    tmin = t;
		    isave = i__;
		}
	    }
	}
L80:
	;
    }
    if (imin == 0) {
/*  	    IERR = 326 */
	return;
    }
    if (imin < 0) {
	if (lv > gp->n_original_vertices ) {
	    if (-imin == cedge[2]) {
		lw1 = lv;
		goto L90;
	    }
	}
	n++;
recheck_size_1:
	if ( (2+n)*1 > gp->size_vcl ) {
	    int res = gp_increase_memory(gp, (void **)&gp->g_vcl, &gp->cap_g_vcl, gp->size_vcl, 3*sizeof(double));
	    if ( res == 0 ) {
		gp->ierr = 500;
		return;
	    }
//	    *size_vcl *= 2;
	    gp->size_vcl += 1024;
	    vcl = gp->g_vcl;
	    vcl -= 4;
	    goto recheck_size_1;
	}
	lw1 = n;
	vcl[n * 3 + 1] = vcl[lw * 3 + 1] + tmin * dir[0];
	vcl[n * 3 + 2] = vcl[lw * 3 + 2] + tmin * dir[1];
	vcl[n * 3 + 3] = vcl[lw * 3 + 3] + tmin * dir[2];
L90:
	if ((i__1 = facesdata[fl * 3 + 2], gp_abs(i__1)) != (i__2 = facesdata[fl * 3 + 3]
		, gp_abs(i__2))) {
	    for (i__ = 1; i__ <= 3; ++i__) {
		j = pedge[i__ + isave * 3];
		pedge[i__ + isave * 3] = pedge[i__ + *nedgc * 3];
		pedge[i__ + *nedgc * 3] = j;
/* L100: */
	    }
	    --(*nedgc);
	}
    }
    ++(*nce);
    cedge[(*nce << 1) + 1] = lw1;
    cedge[(*nce << 1) + 2] = -gp_abs(imin);

/*        If vertex of cut polygon has appeared before, then cut polygon */
/*        is simply-connected (non-simple), so reject cut plane. */

L110:
    if (lw1 == lv) {
	goto L150;
    }
    if (lw1 <= gp->n_original_vertices ) {
	i__1 = nev;
	for (i__ = 1; i__ <= i__1; ++i__) {
	    if (lw1 == gp->g_ev[i__ - 1]) {
		IVP_IF(1) {
		    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
			ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL3, "Rejected due to simply-connected polygon: case 1\n");
		    }
		}
		return;
	    }
/* L120: */
	}

recheck_size_2:
	if (nev >= gp->size_ev) { // orig. 20
	    int res = gp_increase_memory(gp, (void **)&gp->g_ev, &gp->cap_g_ev, gp->size_ev, sizeof(int));
	    if ( res == 0 ) {
		gp->ierr = 500;
		return;
	    }
//	    *addr_of_size_ev *= 2;
	    gp->size_ev += 1024;
	    goto recheck_size_2;
	}

	++nev;
	gp->g_ev[nev - 1] = lw1;
    } else {
	i__1 = n - 1;
	for (i__ = gp->n_original_vertices + 1; i__ <= i__1; ++i__) {
	    for (j = 1; j <= 3; ++j) {
/* Computing MAX */
		d__3 = (d__1 = vcl[j + i__ * 3], gp_abs(d__1)), d__4 = (d__2 = 
			vcl[j + n * 3], gp_abs(d__2));
		cmax = gp_max(d__3,d__4);
		if ((d__1 = vcl[j + i__ * 3] - vcl[j + n * 3], gp_abs(d__1)) > 
			gp->tolerance * cmax && cmax > gp->tolerance) {
		    goto L140;
		}
/* L130: */
	    }
	    IVP_IF(1) {
		IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
		    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL3, "Rejected due to simply-connected polygon: case 2\n");
		}
	    }
	    return;
L140:
	    ;
	}
    }

/*        Compute dihedral angles due to cut plane at edge. If any angle */
/*        is too small, reject cut plane. */

L150:
    if (fr == 0) {
	f = fl;
	dof = (i__1 = facesdata[f * 3 + 2], gp_abs(i__1)) == (i__2 = facesdata[f * 3 + 3]
		, gp_abs(i__2));
	eflag = (i__1 = facesdata[f * 3 + 2], gp_abs(i__1)) != *p || (dof && ccwfl <
		0);
    } else {
	f = fr;
	dof = (i__1 = facesdata[f * 3 + 2], gp_abs(i__1)) == (i__2 = facesdata[f * 3 + 3]
		, gp_abs(i__2));
	sf = ccwfl;
	if (dof) {
	    e = cedge[(*nce << 1) + 2];
	    la = faceverticeslist[e * 6 + 1];
	    lb = faceverticeslist[faceverticeslist[e * 6 + 3] * 6 + 1];
	    if ((lb - la) * ccwfl > 0) {
		e = faceverticeslist[e * 6 + 6];
	    } else {
		e = faceverticeslist[e * 6 + 5];
	    }
	    l = faceverticeslist[e * 6 + 1];
	    if (l == la) {
		sf = -ccwfl;
	    }
	}
	eflag = (i__1 = facesdata[f * 3 + 2], gp_abs(i__1)) != *p || (dof && sf < 0);
    }
    dotp = -(nrmlc[1] * normals[f * 3 + 1] + nrmlc[2] * normals[f * 3 + 2] + nrmlc[
	    3] * normals[f * 3 + 3]);
    if (gp_abs(dotp) > 1. - gp->tolerance) {
	dotp = d_sign(gp, 1.0, dotp);
    }
    if (eflag) {
	dotp = -dotp;
    }
    angr = GP_PI - acos(dotp);
    dir1[0] = nrmlc[2] * dir[2] - nrmlc[3] * dir[1];
    dir1[1] = nrmlc[3] * dir[0] - nrmlc[1] * dir[2];
    dir1[2] = nrmlc[1] * dir[1] - nrmlc[2] * dir[0];
    dotp = dir1[0] * normals[f * 3 + 1] + dir1[1] * normals[f * 3 + 2] + dir1[2] * 
	    normals[f * 3 + 3];
    if (eflag) {
	dotp = -dotp;
    }
    if (dotp > 0.) {
	angr = pi2 - angr;
    }
    if (fr == 0) {
	ang = GP_PI;
    } else {
	a = cedge[(*nce << 1) + 2];
	la = faceverticeslist[a * 6 + 1];
	lb = faceverticeslist[faceverticeslist[a * 6 + 3] * 6 + 1];
	if ((lb - la) * ccwfl > 0) {
	    ang = edge_angles[a];
	} else {
	    ang = edge_angles[faceverticeslist[a * 6 + 5]];
	}
    }
    if ( (angr < gp->angacc) || ang - angr < gp->angacc ) {
	IVP_IF(1) {
	    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
		ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL3, "Rejected due to small angle\n");
	    }
	}
	return;
    }
    if (ccwfl > 0) {
	cdang[*nce] = angr;
    } else {
	cdang[*nce] = -angr;
    }
    if (lw1 != lv) {
	goto L10;
    }

/*     Determine if cut polygon is outer or inner boundary by summing */
/*     the exterior angles (which lie in range (-PI,PI)). A sum of 2*PI */
/*     (-2*PI) means that polygon is outer (inner). Cut polygon is */
/*     rejected in the latter case. */

    s = 0.;
    la = cedge[(((*nce)-1) << 1) + 1];
    lb = cedge[1];
    dir1[0] = vcl[lb * 3 + 1] - vcl[la * 3 + 1];
    dir1[1] = vcl[lb * 3 + 2] - vcl[la * 3 + 2];
    dir1[2] = vcl[lb * 3 + 3] - vcl[la * 3 + 3];
/* Computing 2nd power */
    d__1 = dir1[0];
/* Computing 2nd power */
    d__2 = dir1[1];
/* Computing 2nd power */
    d__3 = dir1[2];
    dir1sq = d__1 * d__1 + d__2 * d__2 + d__3 * d__3;
    i__1 = *nce - 1;
    for (i__ = 0; i__ <= i__1; ++i__) {
	dir[0] = dir1[0];
	dir[1] = dir1[1];
	dir[2] = dir1[2];
	dirsq = dir1sq;
	la = lb;
	lb = cedge[((i__+1) << 1) + 1];
	dir1[0] = vcl[lb * 3 + 1] - vcl[la * 3 + 1];
	dir1[1] = vcl[lb * 3 + 2] - vcl[la * 3 + 2];
	dir1[2] = vcl[lb * 3 + 3] - vcl[la * 3 + 3];
/* Computing 2nd power */
	d__1 = dir1[0];
/* Computing 2nd power */
	d__2 = dir1[1];
/* Computing 2nd power */
	d__3 = dir1[2];
	dir1sq = d__1 * d__1 + d__2 * d__2 + d__3 * d__3;
	dotp = (dir[0] * dir1[0] + dir[1] * dir1[1] + dir[2] * dir1[2]) / 
		sqrt(dirsq * dir1sq);
	if (gp_abs(dotp) > 1. - gp->tolerance) {
	    dotp = d_sign(gp, 1.0, dotp);
	}
	ang = acos(dotp);
	if (kmax == 1) {
	    cp[0] = dir[1] * dir1[2] - dir[2] * dir1[1];
	} else if (kmax == 2) {
	    cp[1] = dir[2] * dir1[0] - dir[0] * dir1[2];
	} else {
	    cp[2] = dir[0] * dir1[1] - dir[1] * dir1[0];
	}
	if (cp[kmax - 1] * nrmlc[kmax] < 0.) {
	    ang = -ang;
	}
	s += ang;
/* L160: */
    }
    if (s < 0.) {
	IVP_IF(1) {
	    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
		ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL3, "Rejected due to inner boundary\n");
	    }
	}
	return;
    }

/*     Move edges incident on LV (if <= gp->N_ORIGINAL_VERTICES), EV(1:NEV) to end of PEDGE. */

    if (lv <= gp->n_original_vertices ) {
	l = lv;
	ee = 0;
    } else {
	ee = 1;
    }
    i__1 = nev;
    for (e = ee; e <= i__1; ++e) {
	if (e > 0) {
	    l = gp->g_ev[e - 1];
	}
	k = 1;
L170:
	if (k > *nedgc) {
	    goto L190;
	}
	a = pedge[k * 3 + 1];
	la = faceverticeslist[a * 6 + 1];
	lb = faceverticeslist[faceverticeslist[a * 6 + 3] * 6 + 1];
	if (la == l || lb == l) {
	    for (i__ = 1; i__ <= 3; ++i__) {
		j = pedge[i__ + k * 3];
		pedge[i__ + k * 3] = pedge[i__ + *nedgc * 3];
		pedge[i__ + *nedgc * 3] = j;
/* L180: */
	    }
	    --(*nedgc);
	} else {
	    ++k;
	}
	goto L170;
L190:
	;
    }

/*     Determine if cut face contains any inner polygons by checking */
/*     if the remaining edges of PEDGE intersect interior of cut face. */

    i__1 = *nedgc;
    for (i__ = 1; i__ <= i__1; ++i__) {
	a = pedge[i__ * 3 + 1];
	ca = pedge[i__ * 3 + 3] / 10;
	cb = pedge[i__ * 3 + 3] % 10;
	la = faceverticeslist[a * 6 + 1];
	lb = faceverticeslist[faceverticeslist[a * 6 + 3] * 6 + 1];
	if (ca == 2) {
	    cp[0] = vcl[la * 3 + 1];
	    cp[1] = vcl[la * 3 + 2];
	    cp[2] = vcl[la * 3 + 3];
	} else if (cb == 2) {
	    cp[0] = vcl[lb * 3 + 1];
	    cp[1] = vcl[lb * 3 + 2];
	    cp[2] = vcl[lb * 3 + 3];
	} else {
	    dir[0] = vcl[lb * 3 + 1] - vcl[la * 3 + 1];
	    dir[1] = vcl[lb * 3 + 2] - vcl[la * 3 + 2];
	    dir[2] = vcl[lb * 3 + 3] - vcl[la * 3 + 3];
	    t = (nrmlc[4] - nrmlc[1] * vcl[la * 3 + 1] - nrmlc[2] * vcl[la * 
		    3 + 2] - nrmlc[3] * vcl[la * 3 + 3]) / (nrmlc[1] * dir[0] 
		    + nrmlc[2] * dir[1] + nrmlc[3] * dir[2]);
	    cp[0] = vcl[la * 3 + 1] + t * dir[0];
	    cp[1] = vcl[la * 3 + 2] + t * dir[1];
	    cp[2] = vcl[la * 3 + 3] + t * dir[2];
	}

	// *******************************************************************
	ptpolg_(gp, 3,
		3,
		nce,
		2,
		&cedge[1],
		&vcl[4],
		cp,
		&nrmlc[1], 
		dtol,
		&inout);

	if (inout == 1) {
	    IVP_IF(1) {
		IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
		    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL3, "Rejected due to hole polygon\n");
		}
	    }
	    return;
	}
/* L200: */
    }
    *rflag = 1;
    IVP_IF(1) {
	IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
	    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL3, "CEDGE(1:2), CDANG\n");
	    i__1 = *nce;
	    for (i__ = 1; i__ <= i__1; ++i__) {
		ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL3, "%d %d %d %f\n", i__, cedge[(i__ << 1) + 1], cedge[(i__ << 1) + 2], cdang[i__] * 180.0 / GP_PI);
	    }
  	}
    }

    return;
}

/* ---- geompack_resedg.cxx ---- */
/* resedg.f -- translated by f2c (version 19990311).
*/

static void resedg_(ivp_geompack_t *gp, int		u_in,
			   long int	*rflag) {

    int *   intworkarray	      = gp->g_intworkarray;
    double *doubleworkarray           = gp->g_doubleworkarray;
    double *vcl                       = gp->g_vcl;
    int	   *polyhedronfirstfaceoffset = gp->g_polyhedronfirstfaceoffset;
    int	   *polyhedronfaceindices     = gp->g_polyhedronfaceindices;
    int    *facesdata                 = gp->g_facesdata;
    double *normals                   = gp->g_normals;
    int    *faceverticeslist          = gp->g_faceverticeslist;
    double *edge_angles               = gp->g_edge_angles;

    /* System generated locals */
    int i__1, i__2;
    double d__1, d__2, d__3;

    /* Local variables */
    int n_edges, nang;
    double leng, dtol, dotp;
    int a, b, c__;
    double e[3];
    int f, i__, j, k, nedgc, p;
    double t;
    int v;
    double anghi, anglo, nrmlc[4];
    long int first;
    int ca, cb;
    double da, db, ce;
    int la, lb, lc;
    double cn;
    int fl;
    double ep[3];
    int fr, lu, lv, sp;
    double mindis;
    int nce;
    double ang[5] = { 0.0 }, enc[3];   /* zeroed only for -Wmaybe-uninitialized */
    int ccw;
    double enl[3], enr[3];
    int luo, lvo;
    double sum;
    int ptr;

/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Attempt to resolve reflex edge by a cut polygon which */
/*        does not create small dihedral angles and does not pass near */
/*        any vertices not on cut plane. */

/*     Input parameters: */
/*        U - index in FACEVERTICESLIST of reflex edge */
/*        ANGACC - min acceptable dihedral angle in radians produced by */
/*              a cut face */
/*        RDACC - min acceptable relative distance between a cut plane */
/*              and vertices not on plane */
/*        ADDR_OF_N_ORIGINAL_VERTICES - number of vertex coordinates or positions used in VCL */
/*        NFACE - number of faces or positions used in FACESDATA array */
/*        ADDR_OF_N_WORK_VERTICES - number of positions used in FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        NPOLH - number of polyhedra or positions used in POLYHEDRONFIRSTFACEOFFSET array */
/*        ADDR_OF_N_POLYHEDRONFACES - number of positions used in POLYHEDRONFACEINDICES array */
/*        ADDR_OF_SIZE_VCL - maximum size available for VCL array */
/*        ADDR_OF_SIZE_FACEARRAYS - maximum size available for FACESDATA, FACESTYPE, NORMALS arrays */
/*        ADDR_OF_SIZE_FACEVERTEXARRAYS - maximum size available for FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        ADDR_OF_SIZE_POLYHEDRONFIRSTFACEOFFSET - maximum size available for POLYHEDRONFIRSTFACEOFFSET array */
/*        ADDR_OF_SIZE_POLYHEDRONFACEINDICES - maximum size available for POLYHEDRONFACEINDICES array */
/*        ADDR_OF_SIZE_intWORKARRAY - maximum size available for intWORKARRAY array; should be about */
/*              5/3*NEDG where NEDG is number of edges in polyhedron */
/*              containing reflex edge */
/*        ADDR_OF_SIZE_DOUBLEWORKARRAY - maximum size available for DOUBLEWORKARRAY array; should be >= NEDG */
/*        VCL(1:3,1:ADDR_OF_N_ORIGINAL_VERTICES) - vertex coordinate list */
/*        FACESDATA(1:3,1:NFACE) - face pointer list */
/*        FACESTYPE(1:NFACE) - face types */
/*        NORMALS(1:3,1:NFACE) - unit normal vectors for faces */
/*        FACEVERTICESLIST(1:6,1:ADDR_OF_N_WORK_VERTICES) - face vertex list */
/*        EDGE_ANGLES(1:ADDR_OF_N_WORK_VERTICES) - edge angles */
/*        POLYHEDRONFIRSTFACEOFFSET(1:NPOLH) - head ptr to face indices in POLYHEDRONFACEINDICES for each polyh */
/*        POLYHEDRONFACEINDICES(1:2,1:ADDR_OF_N_POLYHEDRONFACES) - list of signed face indices for each polyh */

/*     Updated parameters: */
/*        ADDR_OF_N_ORIGINAL_VERTICES,NFACE,ADDR_OF_N_WORK_VERTICES,NPOLH,ADDR_OF_N_POLYHEDRONFACES - sizes updated due to cut face */
/*        VCL,FACESDATA,FACESTYPE,NORMALS,FACEVERTICESLIST,EDGE_ANGLES,POLYHEDRONFIRSTFACEOFFSET,POLYHEDRONFACEINDICES - updated by cut face */

/*     Output parameters: */
/*        RFLAG - .TRUE. iff reflex edge is resolved */

/*     Working parameters: */
/*        intWORKARRAY(1:3,1:*ADDR_OF_SIZE_intWORKARRAY) - int work array */
/*        DOUBLEWORKARRAY(1:ADDR_OF_SIZE_DOUBLEWORKARRAY) - double precision work array */

/*     Routines called: */
/*        CUTFAC, INSFAC */

/*     Abnormal return: */
/*        IERR is set to 6, 7, 14, 15, 16, 17, 18, 325, 326, or 328 */

/*     Find faces FL, FR and polyhedron P containing reflex edge UV. */
/*     Do not resolve UV if FL or FR is a double-occurring face. */

    /* Parameter adjustments */
    vcl -= 4;
    normals -= 4;
    facesdata -= 4;
    --edge_angles;
    faceverticeslist -= 7;
    --polyhedronfirstfaceoffset;
    polyhedronfaceindices -= 3;
    intworkarray -= 4; // vorerst mal drinlassen!!!

    /* Function Body */
    *rflag = 0;
    v = faceverticeslist[u_in * 6 + 3];
    lu = faceverticeslist[u_in * 6 + 1];
    lv = faceverticeslist[v * 6 + 1];
    fl = faceverticeslist[u_in * 6 + 2];
    fr = faceverticeslist[faceverticeslist[u_in * 6 + 6] * 6 + 2];
    if ((i__1 = facesdata[fl * 3 + 2], gp_abs(i__1)) == (i__2 = facesdata[fl * 3 + 3], 
	    gp_abs(i__2))) {
	return;
    }
    if ((i__1 = facesdata[fr * 3 + 2], gp_abs(i__1)) == (i__2 = facesdata[fr * 3 + 3], 
	    gp_abs(i__2))) {
	return;
    }
    if (lu < lv) {
	if (facesdata[fl * 3 + 2] > 0) {
	    p = facesdata[fl * 3 + 2];
	} else {
	    p = facesdata[fl * 3 + 3];
	}
	luo = lu;
	lvo = lv;
    } else {
	if (facesdata[fl * 3 + 2] < 0) {
	    p = -facesdata[fl * 3 + 2];
	} else {
	    p = -facesdata[fl * 3 + 3];
	}
	luo = lv;
	lvo = lu;
    }
    IVP_IF(1) {
	IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
	    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL3, "RESEDG: U,V,LU,LV,FL,FR,P = %d %d %d %d %d %d %d\n", u_in, v, lu, lv, fl, fr, p);
	}
    }

/*     Determine edges in polyhedron P and average edge length. */

    n_edges = 0;
    sum = 0.;
    ptr = polyhedronfirstfaceoffset[p];
L10:
    f = polyhedronfaceindices[(ptr << 1) + 1];
    if (f > 0) {
	ccw = 3;
    } else {
	ccw = 4;
	f = -f;
    }
    a = facesdata[f * 3 + 1];
    lb = faceverticeslist[a * 6 + 1];
L20:
    la = lb;
    b = faceverticeslist[ccw + a * 6];
    lb = faceverticeslist[b * 6 + 1];
    if (la < lb) {
/* Computing 2nd power */
	d__1 = vcl[la * 3 + 1] - vcl[lb * 3 + 1];
/* Computing 2nd power */
	d__2 = vcl[la * 3 + 2] - vcl[lb * 3 + 2];
/* Computing 2nd power */
	d__3 = vcl[la * 3 + 3] - vcl[lb * 3 + 3];
	leng = sqrt(d__1 * d__1 + d__2 * d__2 + d__3 * d__3);
	sum += leng;
	if (la != luo || lb != lvo) {
	    n_edges++;
recheck_size_1:
	    if ( (2+n_edges)*3 > (gp->size_intworkarray) ) {
		int res = gp_increase_memory(gp, (void **)&gp->g_intworkarray, &gp->cap_g_intworkarray, gp->size_intworkarray, sizeof(int));
		if ( res == 0 ) {
		    gp->ierr = 500;
		    return;
		}
//		*addr_of_size_intworkarray *= 2;
		gp->size_intworkarray += 1024;
		intworkarray = gp->g_intworkarray;
		intworkarray -= 4; // vorerst mal drinlassen!!!
		goto recheck_size_1;
	    }
	    if (ccw == 3) {
		intworkarray[n_edges * 3 + 1] = a;
		intworkarray[n_edges * 3 + 2] = faceverticeslist[a * 6 + 6];
	    } else {
		intworkarray[n_edges * 3 + 1] = b;
		intworkarray[n_edges * 3 + 2] = faceverticeslist[b * 6 + 6];
	    }
	}
    }
    a = b;
    if (a != facesdata[f * 3 + 1]) {
	goto L20;
    }
    ptr = polyhedronfaceindices[(ptr << 1) + 2];
    if (ptr != polyhedronfirstfaceoffset[p]) {
	goto L10;
    }
    sum /= (double) (n_edges + 1);
    mindis = gp->rdacc * sum;
    dtol = gp->tolerance * sum;
recheck_size_2:
    if ( ((2+n_edges)*5) > (gp->size_intworkarray) ) {
	int res = gp_increase_memory(gp, (void **)&gp->g_intworkarray, &gp->cap_g_intworkarray, gp->size_intworkarray, sizeof(int));
	if ( res == 0 ) {
	    gp->ierr = 500;
	    return;
	}
//	*addr_of_size_intworkarray *= 2;
	gp->size_intworkarray += 1024;
	intworkarray = gp->g_intworkarray;
	intworkarray -= 4; // vorerst mal drinlassen!!!
	goto recheck_size_2;
    }
    else if ( (2+n_edges) > (gp->size_doubleworkarray) ) {
	int res = gp_increase_memory(gp, (void **)&gp->g_doubleworkarray, &gp->cap_g_doubleworkarray, gp->size_doubleworkarray, sizeof(double));
	if ( res == 0 ) {
	    gp->ierr = 500;
	    return;
	}
//	*addr_of_size_doubleworkarray *= 2;
	gp->size_doubleworkarray += 1024;
	doubleworkarray = gp->g_doubleworkarray;
	goto recheck_size_2;
    }

/*     Compute unit vectors E, ENL, ENR which are directed edge UV and */
/*     edge normals to UV on faces FL and FR. E is unit normal vector */
/*     of plane containing ENL, ENR (it is normalization of ENL x ENR). */

    e[0] = vcl[lvo * 3 + 1] - vcl[luo * 3 + 1];
    e[1] = vcl[lvo * 3 + 2] - vcl[luo * 3 + 2];
    e[2] = vcl[lvo * 3 + 3] - vcl[luo * 3 + 3];
/* Computing 2nd power */
    d__1 = e[0];
/* Computing 2nd power */
    d__2 = e[1];
/* Computing 2nd power */
    d__3 = e[2];
    leng = sqrt(d__1 * d__1 + d__2 * d__2 + d__3 * d__3);
    e[0] /= leng;
    e[1] /= leng;
    e[2] /= leng;
    enl[0] = normals[fl * 3 + 2] * e[2] - normals[fl * 3 + 3] * e[1];
    enl[1] = normals[fl * 3 + 3] * e[0] - normals[fl * 3 + 1] * e[2];
    enl[2] = normals[fl * 3 + 1] * e[1] - normals[fl * 3 + 2] * e[0];
    enr[0] = e[1] * normals[fr * 3 + 3] - e[2] * normals[fr * 3 + 2];
    enr[1] = e[2] * normals[fr * 3 + 1] - e[0] * normals[fr * 3 + 3];
    enr[2] = e[0] * normals[fr * 3 + 2] - e[1] * normals[fr * 3 + 1];
    if ((i__1 = facesdata[fl * 3 + 2], gp_abs(i__1)) != p) {
	enl[0] = -enl[0];
	enl[1] = -enl[1];
	enl[2] = -enl[2];
    }
    if ((i__1 = facesdata[fr * 3 + 2], gp_abs(i__1)) != p) {
	enr[0] = -enr[0];
	enr[1] = -enr[1];
	enr[2] = -enr[2];
    }
    k = 1;
    if (gp_abs(e[1]) > gp_abs(e[0])) {
	k = 2;
    }
    if (gp_abs(e[2]) > (d__1 = e[k - 1], gp_abs(d__1))) {
	k = 3;
    }

/*     Find range of angles [ANGLO, ANGHI] that will resolve reflex edge, */
/*     and an ordered list of at most NANGMX angles determined by planes */
/*     containing reflex edge plus an adjacent edge (the bisection and */
/*     other angles may also be added to list). The list is obtained */
/*     by cycling through edges incident on U and V. CCW = 1 (-1) if */
/*     going CCW (CW) around LA (when viewed from outside polyhedron P). */

    ang[0] = edge_angles[u_in] - GP_PI;
    anglo = gp_max(gp->angacc,ang[0]);
/* Computing MIN */
    d__1 = GP_PI, d__2 = edge_angles[u_in] - gp->angacc;
    anghi = gp_min(d__1,d__2);
    if (anglo > ang[0]) {
	nang = 0;
    } else {
	nang = 2;
	ang[1] = GP_PI;
    }
    for (i__ = 1; i__ <= 2; ++i__) {
	ccw = 1;
	if (i__ == 1) {
	    la = lu;
	    b = faceverticeslist[u_in * 6 + 4];
	    if (lu > lv) {
		ccw = -1;
	    }
	} else {
	    la = lv;
	    b = v;
	    if (lu < lv) {
		ccw = -1;
	    }
	}
	first = 1;
	f = fl;
L30:
	if (! first) {
	    if (faceverticeslist[b * 6 + 1] == la) {
		b = faceverticeslist[b * 6 + 4];
	    } else {
		b = faceverticeslist[b * 6 + 3];
	    }
	}
	c__ = faceverticeslist[b * 6 + 3];
	if ((i__1 = facesdata[f * 3 + 2], gp_abs(i__1)) == (i__2 = facesdata[f * 3 + 3], 
		gp_abs(i__2))) {
	    if (faceverticeslist[b * 6 + 1] == la) {
		sp = -ccw * p;
	    } else {
		sp = ccw * p;
	    }
	} else if ((i__1 = facesdata[f * 3 + 2], gp_abs(i__1)) == p) {
	    sp = facesdata[f * 3 + 2];
	} else {
	    sp = facesdata[f * 3 + 3];
	}
	if ((faceverticeslist[c__ * 6 + 1] - faceverticeslist[b * 6 + 1]) * sp > 0) {
	    b = faceverticeslist[b * 6 + 6];
	} else {
	    b = faceverticeslist[b * 6 + 5];
	}
	f = faceverticeslist[b * 6 + 2];
	if (f == fr) {
	    goto L50;
	}
	if (first) {
	    first = 0;
	} else {
	    if (faceverticeslist[b * 6 + 1] == la) {
		c__ = faceverticeslist[b * 6 + 3];
	    } else {
		c__ = b;
	    }
	    lc = faceverticeslist[c__ * 6 + 1];
	    ep[0] = vcl[lc * 3 + 1] - vcl[la * 3 + 1];
	    ep[1] = vcl[lc * 3 + 2] - vcl[la * 3 + 2];
	    ep[2] = vcl[lc * 3 + 3] - vcl[la * 3 + 3];
	    nrmlc[0] = ep[1] * e[2] - ep[2] * e[1];
	    nrmlc[1] = ep[2] * e[0] - ep[0] * e[2];
	    nrmlc[2] = ep[0] * e[1] - ep[1] * e[0];
/* Computing 2nd power */
	    d__1 = nrmlc[0];
/* Computing 2nd power */
	    d__2 = nrmlc[1];
/* Computing 2nd power */
	    d__3 = nrmlc[2];
	    leng = sqrt(d__1 * d__1 + d__2 * d__2 + d__3 * d__3);
/* Computing MAX */
	    d__1 = gp_abs(ep[0]), d__2 = gp_abs(ep[1]), d__1 = gp_max(d__1,d__2), d__2 
		    = gp_abs(ep[2]);
	    if (leng <= gp->tolerance * gp_max(d__1,d__2)) {
		goto L30;
	    }
	    nrmlc[0] /= leng;
	    nrmlc[1] /= leng;
	    nrmlc[2] /= leng;
	    enc[0] = e[1] * nrmlc[2] - e[2] * nrmlc[1];
	    enc[1] = e[2] * nrmlc[0] - e[0] * nrmlc[2];
	    enc[2] = e[0] * nrmlc[1] - e[1] * nrmlc[0];
	    nrmlc[0] = enr[1] * enc[2] - enr[2] * enc[1];
	    nrmlc[1] = enr[2] * enc[0] - enr[0] * enc[2];
	    nrmlc[2] = enr[0] * enc[1] - enr[1] * enc[0];
/* Computing 2nd power */
	    d__1 = nrmlc[0];
/* Computing 2nd power */
	    d__2 = nrmlc[1];
/* Computing 2nd power */
	    d__3 = nrmlc[2];
	    leng = sqrt(d__1 * d__1 + d__2 * d__2 + d__3 * d__3);
	    if (leng <= gp->tolerance) {
		goto L30;
	    }
	    t = nrmlc[k - 1];
	    nrmlc[0] = enc[1] * enl[2] - enc[2] * enl[1];
	    nrmlc[1] = enc[2] * enl[0] - enc[0] * enl[2];
	    nrmlc[2] = enc[0] * enl[1] - enc[1] * enl[0];
/* Computing 2nd power */
	    d__1 = nrmlc[0];
/* Computing 2nd power */
	    d__2 = nrmlc[1];
/* Computing 2nd power */
	    d__3 = nrmlc[2];
	    leng = sqrt(d__1 * d__1 + d__2 * d__2 + d__3 * d__3);
	    if (leng <= gp->tolerance) {
		goto L30;
	    }
	    if (nrmlc[k - 1] * t < 0.) {
		goto L30;
	    }
	    dotp = enr[0] * enc[0] + enr[1] * enc[1] + enr[2] * enc[2];
	    if (e[k - 1] * t < 0.) {
		dotp = -dotp;
	    }
	    if (gp_abs(dotp) > 1. - gp->tolerance) {
		dotp = d_sign(gp, 1.0, dotp);
	    }
	    t = acos(dotp);
	    if (t < anglo || t > anghi) {
		goto L30;
	    }
	    i__1 = nang;
	    for (j = 1; j <= i__1; ++j) {
		if ((d__1 = ang[j - 1] - t, gp_abs(d__1)) <= gp->tolerance) {
		    goto L30;
		}
/* L40: */
	    }
	    ++nang;
	    ang[nang - 1] = t;
	    if (nang >= 5) {
		goto L80;
	    }
	}
	goto L30;
L50:
	;
    }
    t = edge_angles[u_in] * .5;
    i__1 = nang;
    for (j = 1; j <= i__1; ++j) {
	if ((d__1 = ang[j - 1] - t, gp_abs(d__1)) <= gp->tolerance) {
	    goto L70;
	}
/* L60: */
    }
    ++nang;
    ang[nang - 1] = t;
L70:
    if (nang <= 3 && edge_angles[u_in] <= GP_PI * 1.33) {
	nang += 2;
	ang[nang - 2] = edge_angles[u_in] * .25;
	ang[nang - 1] = edge_angles[u_in] * .75;
	if (GP_PI - ang[nang - 1] <= .1) {
	    ang[nang - 2] = edge_angles[u_in] * .375;
	    ang[nang - 1] = edge_angles[u_in] * .625;
	}
	if (ang[nang - 2] < anglo || ang[nang - 2] > anghi) {
	    ang[nang - 2] = ang[nang - 1];
	    --nang;
	}
	if (ang[nang - 1] < anglo || ang[nang - 1] > anghi) {
	    --nang;
	}
    } else if (nang < 5 && edge_angles[u_in] <= GP_PI * 1.4) {
	t = edge_angles[u_in] * 0.375;
	if (t >= anglo && t <= anghi) {
	    ++nang;
	    ang[nang - 1] = t;
	}
    }
L80:
    IVP_IF(1) {
	IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
	    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL3, "Candidate angles (dihedral angle = %f ", edge_angles[u_in] * 180.0 / GP_PI);
	    for (i__ = 1; i__ <= nang; ++i__) {
		ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL3, "%f ", ang[i__ - 1] * 180.0 / GP_PI);
	    }
	}
    }

/*     For each angle in ANG array, try to resolve reflex edge as */
/*     follows. Compute unit normal NRMLC and equation of cut plane; this */
/*     normal is outward wrt subpolyhedron containing face FL; NRMLC(4) */
/*     is right hand side constant term of plane equation. */
/*     Then determine which edges of polyhedron intersect cut plane; */
/*     reject plane if there is a vertex within distance MINDIS of plane */
/*     which does not lie on plane. */

    i__1 = nang;
    for (i__ = 1; i__ <= i__1; ++i__) {
	cn = cos(ang[i__ - 1]);
	if ((i__2 = facesdata[fr * 3 + 2], gp_abs(i__2)) != p) {
	    cn = -cn;
	}
	ce = sin(ang[i__ - 1]);
	nrmlc[0] = cn * normals[fr * 3 + 1] + ce * enr[0];
	nrmlc[1] = cn * normals[fr * 3 + 2] + ce * enr[1];
	nrmlc[2] = cn * normals[fr * 3 + 3] + ce * enr[2];
	nrmlc[3] = nrmlc[0] * vcl[lu * 3 + 1] + nrmlc[1] * vcl[lu * 3 + 2] + 
		nrmlc[2] * vcl[lu * 3 + 3];
	nedgc = n_edges;
	j = 1;
L90:
	la = faceverticeslist[intworkarray[j * 3 + 1] * 6 + 1];
	da = nrmlc[0] * vcl[la * 3 + 1] + nrmlc[1] * vcl[la * 3 + 2] + nrmlc[
		2] * vcl[la * 3 + 3] - nrmlc[3];
	if (gp_abs(da) <= dtol) {
	    ca = 0;
	} else if (gp_abs(da) <= mindis) {
	    goto L110;
	} else if (da < 0.) {
	    ca = -1;
	} else {
	    ca = 1;
	}
	b = faceverticeslist[intworkarray[j * 3 + 1] * 6 + 3];
	lb = faceverticeslist[b * 6 + 1];
	db = nrmlc[0] * vcl[lb * 3 + 1] + nrmlc[1] * vcl[lb * 3 + 2] + nrmlc[
		2] * vcl[lb * 3 + 3] - nrmlc[3];
	if (gp_abs(db) <= dtol) {
	    cb = 0;
	} else if (gp_abs(db) <= mindis) {
	    goto L110;
	} else if (db < 0.) {
	    cb = -1;
	} else {
	    cb = 1;
	}
	if (ca * cb > 0) {
	    for (k = 1; k <= 2; ++k) {
		c__ = intworkarray[k + j * 3];
		intworkarray[k + j * 3] = intworkarray[k + nedgc * 3];
		intworkarray[k + nedgc * 3] = c__;
/* L100: */
	    }
	    --nedgc;
	} else {
	    intworkarray[j * 3 + 3] = (ca + 2) * 10 + (cb + 2);
	    ++j;
	}
	if (j <= nedgc) {
	    goto L90;
	}
	intworkarray[(n_edges + 1) * 3 + 1] = lvo;
	intworkarray[(n_edges + 1) * 3 + 3] = luo;
	intworkarray[(n_edges + 2) * 3 + 1] = u_in;
	doubleworkarray[1] = ang[i__ - 1];
	IVP_IF(1) {
	    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL3) {
		ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL3, "Trying angle %d   NEDGC = %d\n", i__, nedgc);
	    }
	}

	// *******************************************************************
	cutfac_(gp, &p,
		nrmlc,
		&dtol,
		&nedgc,
		intworkarray,
		&nce,
		&intworkarray[((n_edges+1)*3)],
		&doubleworkarray[1],
		rflag
		);

	vcl = gp->g_vcl;
	vcl -= 4;

	if (gp->ierr != 0) {
	    return;
	}
	if (*rflag) {

	    // ***************************************************************
	    insfac_(gp, &p,
		    nrmlc,
		    &nce,
		    &intworkarray[(n_edges+1)*3],
		    &doubleworkarray[1]
		    );

	    polyhedronfirstfaceoffset = gp->g_polyhedronfirstfaceoffset;
	    --polyhedronfirstfaceoffset;

	    polyhedronfaceindices = gp->g_polyhedronfaceindices;
	    polyhedronfaceindices -= 3;

	    facesdata = gp->g_facesdata;
	    facesdata -= 4;

	    normals = gp->g_normals;
	    normals -= 4;

	    faceverticeslist = gp->g_faceverticeslist;
	    faceverticeslist -= 7;

	    edge_angles = gp->g_edge_angles;
	    --edge_angles;

	    return;
	}
L110:
	;
    }

    return;
}

/* ---- geompack_cvdec3.cxx ---- */
/* cvdec3.f -- translated by f2c (version 19990311).
*/

static void cvdec3_(ivp_geompack_t *gp) {

    double *edge_angles = gp->g_edge_angles;

    /* Local variables */
    int l, n, u;
    long int rflag;
    double piptol;

/*     Written and copyright by: */
/*        Barry Joe, Dept. of Computing Science, Univ. of Alberta */
/*        Edmonton, Alberta, Canada  T6G 2H1 */
/*        Phone: (403) 492-5757      Email: barry@cs.ualberta.ca */

/*     Purpose: Given one or more polyhedra in polyhedral decomposition */
/*        data structure, decompose the polyhedra into convex parts. It */
/*        is assumed all faces are simple (any faces with holes should be */
/*        decomposed into simple polygons), each face appears at most */
/*        once in a polyhedron (double-occurring faces are not allowed), */
/*        and no interior holes occur in any polyhedra. */

/*     Input parameters: */
/*        ANGACC - min acceptable dihedral angle in radians produced by */
/*              cut faces */
/*        RDACC - minimum acceptable relative distance between cut planes */
/*              and vertices not on plane */
/*        ADDR_OF_N_ORIGINAL_VERTICES - number of vertex coordinates or positions used in VCL */
/*        NFACE - number of faces or positions used in FACESDATA array */
/*        ADDR_OF_N_WORK_VERTICES - number of positions used in FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        NPOLH - number of polyhedra or positions used in POLYHEDRONFIRSTFACEOFFSET array */
/*        ADDR_OF_N_POLYHEDRONFACES - number of positions used in POLYHEDRONFACEINDICES array */
/*        SIZE_VCL - maximum size available for VCL array */
/*        ADDR_OF_SIZE_FACEARRAYS - maximum size available for FACESDATA, FACESTYPE, NORMALS arrays */
/*        ADDR_OF_SIZE_FACEVERTEXARRAYS - maximum size available for FACEVERTICESLIST, EDGE_ANGLES arrays */
/*        ADDR_OF_SIZE_POLYHEDRONFIRSTFACEOFFSET - maximum size available for POLYHEDRONFIRSTFACEOFFSET array */
/*        ADDR_OF_SIZE_POLYHEDRONFACEINDICES - maximum size available for POLYHEDRONFACEINDICES array */
/*        ADDR_OF_intWORKARRAY - maximum size available for intWORKARRAY array; should be about */
/*              5 times number of edges in any polyhedron */
/*        ADDR_OF_SIZE_DOUBLEWORKARRAY - maximum size available for DOUBLEWORKARRAY array; should be about */
/*              number of edges in any polyhedron */
/*        VCL(1:3,1:ADDR_OF_N_ORIGINAL_VERTICES) - vertex coordinate list */
/*        FACESDATA(1:3,1:NFACE) - face pointer list: row 1 is head pointer, */
/*              rows 2 and 3 are signed polyhedron indices */
/*        FACESTYPE(1:NFACE) - face types: useful for specifying types of */
/*              boundary faces; entries must be >= 0; any new interior */
/*              faces (not part of previous face) has face type set to 0 */
/*        NORMALS(1:3,1:NFACE) - unit normal vectors for faces; outward */
/*              normal corresponds to CCW traversal of face from polyh */
/*              with index |FACESDATA(2,F)| */
/*        FACEVERTICESLIST(1:6,1:ADDR_OF_N_WORK_VERTICES) - face vertex list; see routine DSPHDC */
/*        EDGE_ANGLES(1:ADDR_OF_N_WORK_VERTICES) - angles at edges common to 2 faces in a polyh; */
/*              EDGE_ANGLES(J) corresponds to FACEVERTICESLIST(*,J), determined by EDGC field */
/*        POLYHEDRONFIRSTFACEOFFSET(1:NPOLH) - head ptr to face indices in POLYHEDRONFACEINDICES for each polyh */
/*        POLYHEDRONFACEINDICES(1:2,1:ADDR_OF_N_POLYHEDRONFACES) - list of signed face indices for each polyh; */
/*              row 2 used for link */

/*     Updated parameters: */
/*        ADDR_OF_N_ORIGINAL_VERTICES,NFACE,ADDR_OF_N_WORK_VERTICES,NPOLH,ADDR_OF_N_POLYHEDRONFACES - sizes updated due to cut faces */
/*        VCL,FACESDATA,FACESTYPE,NORMALS,FACEVERTICESLIST,EDGE_ANGLES,POLYHEDRONFIRSTFACEOFFSET,POLYHEDRONFACEINDICES - updated by cut faces */
/*              to contain convex decomposition */

/*     Working parameters: */
/*        intWORKARRAY(1:*ADDR_OF_intWORKARRAY) - int work array */
/*        DOUBLEWORKARRAY(1:ADDR_OF_SIZE_DOUBLEWORKARRAY) - double precision work array */

/*     Routines called: */
/*        RESEDG */

/*     Abnormal return: */
/*        IERR is set to 6, 7, 14, 15, 16, 17, 18, 325, 326, 327, or 328 */

    /* Function Body */
    piptol = GP_PI + gp->tolerance;
L10:
    l = 0;
    n = 0;
    u = 1;
L20:
    if (edge_angles[u-1] > piptol) {

	// *******************************************************************
	resedg_(gp, u,
		&rflag
		);

	edge_angles = gp->g_edge_angles;

	if (gp->ierr != 0) {
	    return;
	}
	if (rflag) {
	    ++n;
	} else {
	    if (l == 0) {
		l = u;
	    }
	}
    }
    ++u;
    if (u <= gp->n_work_vertices) {
	goto L20;
    }
    if (l > 0) {
	if (n == 0) {
	    gp->ierr = 327;
	    IVP_IF(1) {
		IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
		    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "*** GEOMPACK: at least one reflex edge has not been resolved by routine RESEDG\n");
		}
	    }
	    return;
	} else {
	    goto L10;
	}
    } else {
	if (n > 0) {
	    goto L10;
	}
    }

    return;
}


/* ---- geompack_drdec3.cxx: IVP_Geompack::decompose ---- */

static void *gp_calloc(ivp_geompack_t *gp, size_t count, size_t elem, size_t *cap)
{
    size_t bytes = count * elem;
    if (bytes == 0) bytes = elem;
    void *p = ivp_alloc(gp->alloc, bytes);
    if (p) {
        memset(p, 0, bytes);
        *cap = bytes;
    } else {
        *cap = 0;
    }
    return p;
}

static void gp_free(ivp_geompack_t *gp, void *p, size_t cap)
{
    if (p) ivp_free(gp->alloc, p, cap);
}

/* The two identical IVP_IF(1) / IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1)
 * statistics blocks of IVP_Geompack::decompose (geompack_drdec3.cxx:202 and
 * :287), only the heading differs. */
static void gp_debug_statistics(const ivp_geompack_t *gp, const char *heading)
{
    int i;
    int n_reflex_edges;
    double minimum_angle;
    int mem_iwa;
    int mem_dwa;
    int mem_vcl;
    int mem_pffl;
    int mem_pfil;
    int mem_fl;
    int mem_fvl;
    int mem_total;

    /* calculate some statistical values */
    n_reflex_edges = 0;
    minimum_angle = GP_PI * (double)2.0f; /* set minimum angle to 2*pi */
    for (i = 0; i < gp->n_work_vertices; i++) {
        if (gp->g_edge_angles[i] > GP_PI + gp->tolerance) {
            n_reflex_edges++;
        }
        if (gp->g_edge_angles[i] > -1.0) {
            /* Computing minimum angle in convex decomposition */
            minimum_angle = gp_min(minimum_angle, gp->g_edge_angles[i]);
        }
    }
    minimum_angle = minimum_angle * (double)180.0f / GP_PI;

    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "%s", heading);
    mem_iwa = (int)((size_t)gp->size_intworkarray * sizeof(int));
    mem_dwa = (int)((size_t)gp->size_doubleworkarray * sizeof(double));
    mem_vcl = (int)((size_t)gp->size_vcl * 3 * sizeof(double));
    mem_pffl = (int)((size_t)gp->size_polyhedronfirstfaceoffset * sizeof(int));
    mem_pfil = (int)((size_t)gp->size_polyhedronfaceindices * 2 * sizeof(int));
    mem_fl = (int)((size_t)gp->size_facearrays * sizeof(int) + (size_t)gp->size_facearrays * 3 * sizeof(int) +
                   (size_t)gp->size_facearrays * 3 * sizeof(double));
    mem_fvl = (int)((size_t)gp->size_facevertexarrays * 6 * sizeof(int) +
                    (size_t)gp->size_facevertexarrays * sizeof(double));
    mem_total = mem_iwa + mem_dwa + mem_vcl + mem_pffl + mem_pfil + mem_fl + mem_fvl;
    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL2) {
        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for int WORK ARRAY: %d bytes\n", mem_iwa);
        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for DOUBLE WORK ARRAY: %d bytes\n", mem_dwa);
        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for VERTEX COORDINATE LIST: %d bytes\n", mem_vcl);
        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for POLYHEDRON FIRST FACE LIST: %d bytes\n", mem_pffl);
        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for POLYHEDRON FACE INDEX LIST: %d bytes\n", mem_pfil);
        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for FACE LISTS: %d bytes\n", mem_fl);
        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "Memory allocated for FACE VERTICES LISTS: %d bytes\n", mem_fvl);
    }
    else {
        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "Total memory allocated: %d bytes\n", mem_total);
    }
    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "N_ORIGINAL_VERTICES = %d\n", gp->n_original_vertices);
    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "NFACE               = %d\n", gp->nface);
    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "N_WORK_VERTICES     = %d\n", gp->n_work_vertices);
    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "NPOLH               = %d\n", gp->npolh);
    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "N_POLYHEDRONFACES   = %d\n", gp->n_polyhedronfaces);
    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "N_REFLEX_EDGES      = %d\n", n_reflex_edges);
    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "MINIMUM_ANGLE       = %f\n", minimum_angle);
    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "\n");
}

void ivp_geompack_decompose(const ivp_allocator_t *alloc, ivp_geompack_parameters_t *params)
{
    ivp_geompack_t gs;
    ivp_geompack_t *gp = &gs;
    memset(gp, 0, sizeof(*gp));
    gp->alloc = alloc;
    int i;
    int retry_counter;
    bool oom = false;

    gp->size_intworkarray = 12;
    gp->size_doubleworkarray = 12;
    gp->size_vcl = params->nvc + 2;
    gp->size_polyhedronfirstfaceoffset = 2;
    gp->size_polyhedronfaceindices = 1;
    gp->size_facearrays = params->nface + 2;
    gp->size_facevertexarrays = params->facep[params->nface * 3] + 2;
    gp->size_ev = 200;
    gp->hashtable_maxsize = 307;

    params->nvc_out = 0;
    params->nface_out = 0;
    params->nvert_out = 0;
    params->npolh_out = 0;

    gp->angacc = params->angacc * GP_PI / 180.0;
    gp->rdacc = params->rdacc;

    initcb_(gp, params->tolin);

    gp->g_hashtable = (int *)gp_calloc(gp, (size_t)gp->hashtable_maxsize, sizeof(int), &gp->cap_g_hashtable);
    gp->g_intworkarray = (int *)gp_calloc(gp, (size_t)gp->size_intworkarray, sizeof(int), &gp->cap_g_intworkarray);
    gp->g_doubleworkarray = (double *)gp_calloc(gp, (size_t)gp->size_doubleworkarray, sizeof(double),
                                                &gp->cap_g_doubleworkarray);
    gp->g_polyhedronfirstfaceoffset = (int *)gp_calloc(gp, (size_t)gp->size_polyhedronfirstfaceoffset,
                                                       sizeof(int), &gp->cap_g_polyhedronfirstfaceoffset);
    gp->g_polyhedronfaceindices = (int *)gp_calloc(gp, (size_t)gp->size_polyhedronfaceindices,
                                                   2 * sizeof(int), &gp->cap_g_polyhedronfaceindices);
    gp->g_normals = (double *)gp_calloc(gp, (size_t)gp->size_facearrays, 3 * sizeof(double), &gp->cap_g_normals);
    gp->g_facestype = (int *)gp_calloc(gp, (size_t)gp->size_facearrays, sizeof(int), &gp->cap_g_facestype);
    gp->g_faceverticeslist = (int *)gp_calloc(gp, (size_t)gp->size_facevertexarrays, 6 * sizeof(int),
                                              &gp->cap_g_faceverticeslist);
    gp->g_edge_angles = (double *)gp_calloc(gp, (size_t)gp->size_facevertexarrays, sizeof(double),
                                            &gp->cap_g_edge_angles);
    gp->g_ev = (int *)gp_calloc(gp, (size_t)gp->size_ev, sizeof(int), &gp->cap_g_ev);

    /* vertex coordinates and face pointers are taken over from the caller */
    gp->g_vcl = params->vcl;
    gp->cap_g_vcl = params->vcl_cap_bytes;
    gp->g_facesdata = params->facep;
    gp->cap_g_facesdata = params->facep_cap_bytes;

    if (!gp->g_facestype || !gp->g_hashtable || !gp->g_polyhedronfirstfaceoffset ||
        !gp->g_polyhedronfaceindices || !gp->g_faceverticeslist || !gp->g_intworkarray ||
        !gp->g_doubleworkarray || !gp->g_edge_angles || !gp->g_normals || !gp->g_ev) {
        IVP_IF(1) {
            IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "*** GEOMPACK: Out of memory!\n\n");
            }
        }
        oom = true;
        gp->ierr = 500;
        goto out_of_memory;
    }

    gp->n_original_vertices = params->nvc;
    gp->nface = params->nface;
    gp->npolh = 1;

    for (i = 0; i < gp->nface; i++) gp->g_facestype[i] = 0;

    gp->n_work_vertices = gp->g_facesdata[gp->nface * 3] - 1;
    for (i = 0; i < gp->n_work_vertices; i++) gp->g_faceverticeslist[i * 6] = params->fvl[i];

    gp->g_polyhedronfirstfaceoffset[0] = 1;
    gp->g_polyhedronfirstfaceoffset[1] = gp->nface + 1;
    gp->n_polyhedronfaces = gp->g_polyhedronfirstfaceoffset[gp->npolh] - 1;

    while ((2 + gp->n_polyhedronfaces) * 1 > gp->size_polyhedronfaceindices) {
        int res = gp_increase_memory(gp, (void **)&gp->g_polyhedronfaceindices,
                                     &gp->cap_g_polyhedronfaceindices,
                                     gp->size_polyhedronfaceindices, 2 * sizeof(int));
        if (res == 0) {
            gp->ierr = 500;
            goto GEOMPACK_abort;
        }
        gp->size_polyhedronfaceindices += 1024;
    }
    for (i = 1; i <= gp->n_polyhedronfaces; i++) {
        gp->g_polyhedronfaceindices[(i << 1) - 2] = i;
    }

    gp->hashtable_size = gp_min(prime_(gp, gp->n_original_vertices + 2), gp->hashtable_maxsize);

    /* Init data structure */
    dsphdc_(gp);
    if (gp->ierr != 0) {
        IVP_IF(1) {
            IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
                if (gp->ierr == 500) {
                    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "*** GEOMPACK: Out of memory!\n\n");
                }
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "Premature abort due to above error.\n\n");
            }
        }
        goto GEOMPACK_abort;
    }

    IVP_IF(1) {
        IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
            gp_debug_statistics(gp, "Statistics after initializing polyhedral decomposition data structure:\n");
        }
    }

    /* Decompose polyhedral region into convex parts. */
    retry_counter = 1;
Retry_convex_decomposition:
    cvdec3_(gp);
    if ((gp->ierr != 0) && (gp->ierr != 327)) { /* abort on error but skip "reflex edge" resolving problems */
        IVP_IF(1) {
            IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
                if (gp->ierr == 500) {
                    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "*** GEOMPACK: Out of memory!\n\n");
                }
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "Premature abort due to above error.\n\n");
            }
        }
        goto GEOMPACK_abort;
    }

    IVP_IF(1) {
        IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
            gp_debug_statistics(gp, "Intermediate decomposition statistics:\n");
        }
    }

    if (gp->ierr == 327) { /* unresolved reflex edge */
        if (retry_counter < 36) {
            gp->angacc -= GP_PI / 36.0;
            if (gp->angacc > 0.0) {
                gp->rdacc *= 0.95;
                gp->ierr = 0;
                retry_counter++;
                IVP_IF(1) {
                    IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
                        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "Retrying with <angacc=%f> and <rdacc=%f>\n\n", gp->angacc, gp->rdacc);
                    }
                }
                goto Retry_convex_decomposition;
            }
        }
        IVP_IF(1) {
            IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "Premature abort due to difficulties in resolving some reflex edge(s).\n\n");
            }
        }
        goto GEOMPACK_abort;
    }

    /* Final output. */
    IVP_IF(1) {
        IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "+++ Convex decomposition successful!\n\n");
        }

        IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL2) {
            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "VCL (vertex coordinate list)\n");
            for (i = 1; i <= gp->n_original_vertices; i++) {
                int j;
                ivp_debug_printf("#%d : \t", i);
                for (j = 1; j <= 3; j++) {
                    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "%f   ", gp->g_vcl[j + (i * 3) - 4]);
                }
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "\n");
            }
            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "\n");

            for (i = 1; i <= gp->nface; i++) {
                int j;
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "i=%d\tfacesdata={", i);
                for (j = 1; j <= 3; j++) {
                    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "%d ", gp->g_facesdata[j + (i * 3) - 4]);
                }
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "}  facestype[%d]=%d\tnormals={", i - 1, gp->g_facestype[i - 1]);
                for (j = 1; j <= 3; j++) {
                    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "%f  ", gp->g_normals[j + (i * 3) - 4]);
                }
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "\n");
            }
            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "\n\n");

            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "FaceVertexList >>FVL<<\n");
            for (i = 1; i <= gp->n_work_vertices; i++) {
                int j;
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "#%d\t", i);
                for (j = 1; j <= 6; j++) {
                    switch (j) {
                    case 1:
                        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "LOC = ");
                        break;
                    case 2:
                        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "FACN = ");
                        break;
                    case 3:
                        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "SUCC = ");
                        break;
                    case 4:
                        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "PRED = ");
                        break;
                    case 5:
                        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "EDGA = ");
                        break;
                    case 6:
                        ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "EDGC = ");
                        break;
                    }
                    ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "%d\t", gp->g_faceverticeslist[j + (i * 6) - 7]);
                }
                ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "\t(%f Grad)\n", gp->g_edge_angles[i - 1] / GP_PI * 180.0);
            }
            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL2, "\n\n");
        }
    }

GEOMPACK_abort:
    IVP_IF(1) {
        IVP_IFDEBUG(IVP_DM_GEOMPACK_LEVEL1) {
            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "Final GEOMPACK statistics:\n");
            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "N_ORIGINAL_VERTICES   \t= %d (number of vertex coordinates)\n", gp->n_original_vertices);
            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "NFACE \t= %d (number of faces in polyhedral decomposition)\n", gp->nface);
            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "N_WORK_VERTICES \t= %d (number of positions used in FVL array)\n", gp->n_work_vertices);
            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "NPOLH \t= %d (number of polyhedra in decomposition)\n", gp->npolh);
            ivp_better_debug_manager_dprint(&ivp_debugmanager, IVP_DM_GEOMPACK_LEVEL1, "\n\n");
        }
    }

    params->nvc_out = gp->n_original_vertices;
    params->nface_out = gp->nface;
    params->nvert_out = gp->n_work_vertices;
    params->npolh_out = gp->npolh;

out_of_memory:
    if (oom) {
        params->nvc_out = 0;
        params->nface_out = 0;
        params->nvert_out = 0;
        params->npolh_out = 0;
    }
    params->ierr = gp->ierr;
    params->facep_out = gp->g_facesdata;
    params->facep_out_cap = gp->cap_g_facesdata;
    params->fvl_out = gp->g_faceverticeslist;
    params->fvl_out_cap = gp->cap_g_faceverticeslist;
    params->vcl_out = gp->g_vcl;
    params->vcl_out_cap = gp->cap_g_vcl;

    gp_free(gp, gp->g_facestype, gp->cap_g_facestype);
    gp_free(gp, gp->g_hashtable, gp->cap_g_hashtable);
    gp_free(gp, gp->g_polyhedronfirstfaceoffset, gp->cap_g_polyhedronfirstfaceoffset);
    gp_free(gp, gp->g_polyhedronfaceindices, gp->cap_g_polyhedronfaceindices);
    gp_free(gp, gp->g_intworkarray, gp->cap_g_intworkarray);
    gp_free(gp, gp->g_doubleworkarray, gp->cap_g_doubleworkarray);
    gp_free(gp, gp->g_edge_angles, gp->cap_g_edge_angles);
    gp_free(gp, gp->g_normals, gp->cap_g_normals);
    gp_free(gp, gp->g_ev, gp->cap_g_ev);
}
