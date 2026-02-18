/* ivp_build_box.c — Hardcoded box compact surface builder
 *
 * Constructs a compact surface for an axis-aligned box centered at the origin.
 *
 * Box topology:
 *   8 vertices, 12 triangles (6 faces × 2), 36 half-edges (18 unique edges).
 *   All normals point outward (CCW winding from outside).
 *
 * Vertex ordering (matches original IVP reference runner):
 *   for (sx = -1,+1) for (sy = -1,+1) for (sz = -1,+1)
 *   Index: 0=(-,-,-) 1=(-,-,+) 2=(-,+,-) 3=(-,+,+)
 *          4=(+,-,-) 5=(+,-,+) 6=(+,+,-) 7=(+,+,+)
 *
 * Face triangulation (outward normals verified by cross product):
 *   Face x=−hx: T0(0,1,3) T1(0,3,2)   normal=(−1,0,0) ✓
 *   Face x=+hx: T2(4,6,7) T3(4,7,5)   normal=(+1,0,0) ✓
 *   Face y=−hy: T4(0,4,5) T5(0,5,1)   normal=(0,−1,0) ✓
 *   Face y=+hy: T6(2,3,7) T7(2,7,6)   normal=(0,+1,0) ✓
 *   Face z=−hz: T8(0,2,6) T9(0,6,4)   normal=(0,0,−1) ✓
 *   Face z=+hz: T10(1,5,7) T11(1,7,3) normal=(0,0,+1) ✓
 *
 * Memory layout:
 *   [ivp_compact_surface_t]              48 bytes
 *   [ivp_compact_ledge_t]                16 bytes  (ledge header)
 *   [ivp_compact_triangle_t × 12]       192 bytes  (triangles)
 *   [ivp_compact_poly_point_t × 8]      128 bytes  (vertices)
 *   [ivp_compact_ledgetree_node_t × 1]   28 bytes  (single terminal node)
 *   Total: 48 + 16 + 192 + 128 + 28 = 412 bytes → pad to 416 (16-aligned)
 */

#include <ivp/builder/ivp_build_surface.h>
#include <ivp/base/ivp_allocator.h>
#include <math.h>
#include <string.h>

/* ── Box topology constants ─────────────────────────────────────────────── */

#define BOX_N_VERTICES   8
#define BOX_N_TRIANGLES 12
#define BOX_N_EDGES     36  /* 12 × 3 */

/* Triangle vertex indices (3 per triangle, CCW outward) */
static const int s_box_tri_verts[BOX_N_TRIANGLES][3] = {
    /* Face x = -hx */  {0, 1, 3},  {0, 3, 2},
    /* Face x = +hx */  {4, 6, 7},  {4, 7, 5},
    /* Face y = -hy */  {0, 4, 5},  {0, 5, 1},
    /* Face y = +hy */  {2, 3, 7},  {2, 7, 6},
    /* Face z = -hz */  {0, 2, 6},  {0, 6, 4},
    /* Face z = +hz */  {1, 5, 7},  {1, 7, 3},
};

/* Opposite edge table (precomputed).
 * For each triangle t, edge e (0..2), s_box_opposite[t][e] = {opp_tri, opp_edge}.
 *
 * Each edge (t,e) connects vertex[e] → vertex[(e+1)%3] of triangle t.
 * Its opposite edge connects the same pair of vertices in reverse direction.
 *
 * opposite_index = (opp_tri * 4 + opp_edge) - (t * 4 + e)
 */
static const int s_box_opposite[BOX_N_TRIANGLES][3][2] = {
    /* T0  */ { {5,2},  {11,2}, {1,0}  },
    /* T1  */ { {0,2},  {6,0},  {8,0}  },
    /* T2  */ { {9,1},  {7,1},  {3,0}  },
    /* T3  */ { {2,2},  {10,1}, {4,1}  },
    /* T4  */ { {9,2},  {3,2},  {5,0}  },
    /* T5  */ { {4,2},  {10,0}, {0,0}  },
    /* T6  */ { {1,1},  {11,1}, {7,0}  },
    /* T7  */ { {6,2},  {2,1},  {8,1}  },
    /* T8  */ { {1,2},  {7,2},  {9,0}  },
    /* T9  */ { {8,2},  {2,0},  {4,0}  },
    /* T10 */ { {5,1},  {3,1},  {11,0} },
    /* T11 */ { {10,2}, {6,1},  {0,1}  },
};

/* ── Size computation ───────────────────────────────────────────────────── */

#define SURFACE_HEADER_SIZE  ((int)sizeof(ivp_compact_surface_t))      /* 48 */
#define LEDGE_HEADER_SIZE    ((int)sizeof(ivp_compact_ledge_t))        /* 16 */
#define TRI_SIZE             ((int)sizeof(ivp_compact_triangle_t))     /* 16 */
#define POINT_SIZE           ((int)sizeof(ivp_compact_poly_point_t))   /* 16 */
#define NODE_SIZE            ((int)sizeof(ivp_compact_ledgetree_node_t)) /* 28 */

#define LEDGE_DATA_SIZE  (LEDGE_HEADER_SIZE + BOX_N_TRIANGLES * TRI_SIZE \
                          + BOX_N_VERTICES * POINT_SIZE)
/* 16 + 192 + 128 = 336 */

#define TOTAL_UNPADDED   (SURFACE_HEADER_SIZE + LEDGE_DATA_SIZE + NODE_SIZE)
/* 48 + 336 + 28 = 412 */

/* Round up to 16-byte alignment */
#define TOTAL_PADDED     ((TOTAL_UNPADDED + 15) & ~15)
/* 416 */

/* ── Builder implementation ─────────────────────────────────────────────── */

ivp_compact_surface_t *ivp_build_box_surface(const ivp_allocator_t *alloc,
                                               float half_x,
                                               float half_y,
                                               float half_z)
{
    if (!alloc) alloc = ivp_default_allocator();

    /* Allocate 16-byte aligned block */
    void *mem = ivp_alloc_aligned(alloc, (size_t)TOTAL_PADDED, 16);
    if (!mem) return NULL;
    memset(mem, 0, (size_t)TOTAL_PADDED);

    /* ── Pointers into the block ────────────────────────────────────────── */

    ivp_compact_surface_t *surface = (ivp_compact_surface_t *)mem;

    ivp_compact_ledge_t *ledge =
        (ivp_compact_ledge_t *)((char *)mem + SURFACE_HEADER_SIZE);

    ivp_compact_triangle_t *triangles =
        (ivp_compact_triangle_t *)((char *)ledge + LEDGE_HEADER_SIZE);

    ivp_compact_poly_point_t *points =
        (ivp_compact_poly_point_t *)((char *)triangles +
                                      BOX_N_TRIANGLES * TRI_SIZE);

    ivp_compact_ledgetree_node_t *node =
        (ivp_compact_ledgetree_node_t *)((char *)ledge + LEDGE_DATA_SIZE);

    /* ── Fill vertex positions ──────────────────────────────────────────── */

    float hx = half_x, hy = half_y, hz = half_z;

    /* Order: for sx in {-1,+1} for sy in {-1,+1} for sz in {-1,+1} */
    int vi = 0;
    for (int sx = -1; sx <= 1; sx += 2) {
        for (int sy = -1; sy <= 1; sy += 2) {
            for (int sz = -1; sz <= 1; sz += 2) {
                points[vi].k[0] = (float)sx * hx;
                points[vi].k[1] = (float)sy * hy;
                points[vi].k[2] = (float)sz * hz;
                points[vi].hesse_val = 0.0f;
                vi++;
            }
        }
    }

    /* ── Fill triangles and edges ───────────────────────────────────────── */

    for (int t = 0; t < BOX_N_TRIANGLES; t++) {
        ivp_compact_triangle_t *tri = &triangles[t];

        /* Triangle header */
        tri->packed = 0;
        ivp_tri_set_tri_index(tri, t);
        ivp_tri_set_pierce_index(tri, 0);
        ivp_tri_set_material_index(tri, 0);
        ivp_tri_set_is_virtual(tri, 0);

        /* Fill 3 edges */
        for (int e = 0; e < 3; e++) {
            int start_vertex = s_box_tri_verts[t][e];

            /* Compute opposite_index:
             * Linear position of this edge:  t*4 + e
             * Linear position of opposite:   opp_tri*4 + opp_edge
             * opposite_index = opp_linear - this_linear */
            int opp_tri  = s_box_opposite[t][e][0];
            int opp_edge = s_box_opposite[t][e][1];
            int this_linear = t * 4 + e;
            int opp_linear  = opp_tri * 4 + opp_edge;
            int opposite_index = opp_linear - this_linear;

            ivp_edge_set(&tri->edges[e], start_vertex, opposite_index, 0);
        }
    }

    /* ── Fill ledge header ──────────────────────────────────────────────── */

    ivp_ledge_init(ledge);
    /* c_point_offset: byte offset from ledge to point array */
    ledge->c_point_offset = LEDGE_HEADER_SIZE + BOX_N_TRIANGLES * TRI_SIZE;
    /* node_or_client: byte offset from ledge to ledgetree node */
    ledge->node_or_client = LEDGE_DATA_SIZE;  /* relative offset to tree node */
    /* flags: terminal (has_children=0), compact (is_compact=1) */
    ivp_ledge_set_has_children(ledge, 0);
    ivp_ledge_set_is_compact(ledge, 1);
    ivp_ledge_set_size(ledge, LEDGE_DATA_SIZE);
    ledge->n_triangles = BOX_N_TRIANGLES;
    ledge->reserved = 0;

    /* ── Fill ledgetree node ────────────────────────────────────────────── */

    node->offset_right_node = 0;  /* terminal */
    /* offset_compact_ledge: byte offset from node back to ledge */
    node->offset_compact_ledge = (int32_t)((char *)ledge - (char *)node);

    /* Bounding sphere: centered at origin, radius = diagonal/2 */
    node->center[0] = 0.0f;
    node->center[1] = 0.0f;
    node->center[2] = 0.0f;
    float radius = sqrtf(hx * hx + hy * hy + hz * hz);
    node->radius = radius;

    /* Encode AABB box sizes */
    if (radius > 0.0f) {
        float inv_step = 1.0f / (radius * IVP_COMPACT_BOUNDINGBOX_STEP_SIZE);
        int bx = (int)(hx * inv_step) + 1;
        int by = (int)(hy * inv_step) + 1;
        int bz = (int)(hz * inv_step) + 1;
        node->box_sizes[0] = (uint8_t)(bx > 255 ? 255 : bx);
        node->box_sizes[1] = (uint8_t)(by > 255 ? 255 : by);
        node->box_sizes[2] = (uint8_t)(bz > 255 ? 255 : bz);
    } else {
        node->box_sizes[0] = node->box_sizes[1] = node->box_sizes[2] = 1;
    }
    node->padding = 0;

    /* ── Fill surface header ────────────────────────────────────────────── */

    surface->mass_center[0] = 0.0f;
    surface->mass_center[1] = 0.0f;
    surface->mass_center[2] = 0.0f;

    /* Rotation inertia for unit mass box:
     * Ixx = (hy² + hz²) / 3
     * Iyy = (hx² + hz²) / 3
     * Izz = (hx² + hy²) / 3 */
    surface->rotation_inertia[0] = (hy * hy + hz * hz) / 3.0f;
    surface->rotation_inertia[1] = (hx * hx + hz * hz) / 3.0f;
    surface->rotation_inertia[2] = (hx * hx + hy * hy) / 3.0f;

    surface->upper_limit_radius = radius;

    /* size_and_deviation: byte_size in bits [31:8], max_factor in bits [7:0] */
    ivp_surface_set_byte_size(surface, TOTAL_PADDED);
    ivp_surface_set_max_deviation(surface, 1);  /* minimal deviation for convex */

    /* offset_ledgetree_root: from surface start to tree node */
    surface->offset_ledgetree_root =
        (int32_t)((char *)node - (char *)surface);

    surface->dummy[0] = 0;
    surface->dummy[1] = 0;
    surface->dummy[2] = 0;

    return surface;
}
