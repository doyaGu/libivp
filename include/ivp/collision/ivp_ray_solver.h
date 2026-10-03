/* ivp_ray_solver.h — Ray casting (IVP_Ray_Solver family)
 *
 * Port of ivp_surface_manager/ivp_ray_solver.hxx and
 * ivp_collision/ivp_ray_solver.cxx:
 *
 *   ivp_ray_solver_t        IVP_Ray_Solver: world space ray, sends every hit
 *                           to a hit listener callback (IVP_Ray_Hit_Listener)
 *   ivp_ray_solver_os_t     IVP_Ray_Solver_Os: the ray in one object's space;
 *                           walks the ledgetree (node spheres) and the
 *                           convex ledges
 *   ivp_ray_solver_min_t    IVP_Ray_Solver_Min: keeps the closest hit
 *   ivp_ray_solver_min_hash_t  IVP_Ray_Solver_Min_Hash: keeps all hits
 *   ivp_ray_solver_group_t  IVP_Ray_Solver_Group: several short rays tested
 *                           against a common bounding sphere
 *
 * Reference semantics:
 *   - convex ledges: only triangles facing the ray start are entered and
 *     the hit triangle is found by walking across neighbours; a ray that
 *     starts inside a convex ledge does not hit it;
 *   - two-triangle (flat) ledges are two sided; the reported normal is
 *     flipped towards the ray start;
 *   - every reported normal (hit_surface_direction_os) is normalised and in
 *     object space;
 *   - ledgetree nodes are pruned with the reference ray-segment/sphere test
 *     (including its -0.01 tolerance), compact hulls are not tested;
 *   - an object can produce several hits (one per hit terminal ledge);
 *   - environment queries walk the OV tree (IVP_OV_Tree_Manager) like
 *     check_ray_against_node: only objects with collision detection enabled
 *     are visited, in the reference order (node elements last to first, then
 *     children last to first), which decides the order of equal distance
 *     hits;
 *   - IVP_RAY_SOLVER_IGNORE_PHANTOMS skips objects converted to phantoms
 *     (object->controller_phantom), IGNORE_MOVINGS / IGNORE_STATICS test
 *     IVP_MTIS_IS_STATIC(object->object_movement_state) (a static object
 *     converted to a phantom with the default template becomes
 *     IVP_MT_STATIC_PHANTOM, which is NOT static).
 *
 * The legacy single-surface helpers (ivp_ray_cast_surface / _ledge,
 * ivp_ray_test_triangle, ivp_ray_test_sphere) are kept; the first two now
 * follow the reference semantics above.
 */
#ifndef IVP_RAY_SOLVER_H
#define IVP_RAY_SOLVER_H

#include <ivp/base/ivp_types.h>
#include <ivp/base/ivp_result.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/geometry/ivp_compact_surface.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_object;
struct ivp_environment;
struct ivp_ov_node;
struct ivp_ov_tree;
struct ivp_min_hash;

/* ── Constants / flags ──────────────────────────────────────────────────── */

#define IVP_MAX_NUM_RAY_HITS 256
#define IVP_MAX_RAY_HITS IVP_MAX_NUM_RAY_HITS

typedef enum ivp_ray_solver_flags {
    IVP_RAY_SOLVER_ALL             = 0,
    IVP_RAY_SOLVER_IGNORE_PHANTOMS = 1, /* object->controller_phantom (+ is_phantom) */
    IVP_RAY_SOLVER_IGNORE_MOVINGS  = 2,
    IVP_RAY_SOLVER_IGNORE_STATICS  = 4
} ivp_ray_solver_flags_t;

/* ── Ray hit result (IVP_Ray_Hit) ───────────────────────────────────────── */

typedef struct ivp_ray_hit {
    ivp_real_t                       hit_distance;        /* distance along the ray */
    ivp_vec3r_t                      hit_point;           /* start + dir * distance (ray space) */
    ivp_vec3r_t                      hit_normal;          /* hit_surface_direction_os: unit, object space */
    struct ivp_object               *hit_object;          /* NULL for object space queries */
    const ivp_compact_ledge_t       *hit_ledge;           /* NULL for balls */
    const ivp_compact_triangle_t    *hit_triangle;        /* NULL for balls */
    int                              hit_material_index;  /* material index from triangle */
} ivp_ray_hit_t;

/* ── Ray solver (IVP_Ray_Solver) ────────────────────────────────────────── */

typedef struct ivp_ray_solver_template {
    ivp_vec3r_t ray_start_point;          /* world space */
    ivp_vec3r_t ray_normized_direction;   /* must be normalised */
    ivp_real_t  ray_length;
    unsigned    ray_flags;                /* ivp_ray_solver_flags_t bits */
} ivp_ray_solver_template_t;

/* IVP_Ray_Hit_Listener::add_hit_object */
typedef void (*ivp_ray_hit_listener_fn)(void *listener,
                                        struct ivp_object *object,
                                        const ivp_compact_ledge_t *ledge,
                                        const ivp_compact_triangle_t *triangle,
                                        double hit_dist,
                                        const double hit_sur_vec_os[3]);

/* Optional extra phantom classifier for IVP_RAY_SOLVER_IGNORE_PHANTOMS
 * (libivp extension).  Objects with a phantom controller
 * (ivp_object_convert_to_phantom) are always treated as phantoms; the
 * callback can classify further objects as phantoms. */
typedef bool (*ivp_ray_is_phantom_fn)(void *user, const struct ivp_object *object);

typedef struct ivp_ray_solver {
    double   ray_start_point[3];   /* IVP_U_Point */
    double   ray_end_point[3];     /* IVP_U_Point */
    float    ray_center_point[3];  /* IVP_U_Float_Point */
    float    ray_direction[3];     /* IVP_U_Float_Point */
    float    ray_length;
    unsigned ray_flags;

    ivp_ray_hit_listener_fn add_hit;
    void                   *listener;
    ivp_ray_is_phantom_fn   is_phantom;
    void                   *is_phantom_user;
} ivp_ray_solver_t;

void ivp_ray_solver_init(ivp_ray_solver_t *rs,
                         const ivp_ray_solver_template_t *templ,
                         ivp_ray_hit_listener_fn add_hit,
                         void *listener);

/* IVP_Ray_Solver::check_ray_against_sphere (world space culling test). */
bool ivp_ray_solver_check_sphere(const ivp_ray_solver_t *rs,
                                 const float sphere_center_ws[3],
                                 float sphere_radius);

/* IVP_Ray_Solver::check_ray_against_cube (axis aligned cube luf..rlb). */
bool ivp_ray_solver_check_cube(const ivp_ray_solver_t *rs,
                               const float luf_point[3],
                               const float rlb_point[3]);

/* IVP_Ray_Solver::check_ray_against_ball. */
void ivp_ray_solver_check_ball(ivp_ray_solver_t *rs, struct ivp_object *ball);

/* IVP_Ray_Solver::check_ray_against_object (flags, ball / polygon). */
void ivp_ray_solver_check_object(ivp_ray_solver_t *rs, struct ivp_object *object);

/* IVP_Ray_Solver::check_ray_against_all_objects_in_sim: walks the
 * environment's OV tree from its root with ivp_ray_solver_check_node().
 * Objects without collision detection are not in the tree and are never
 * reported. */
void ivp_ray_solver_check_all_objects(ivp_ray_solver_t *rs,
                                      const struct ivp_environment *env);

/* IVP_Ray_Solver::check_ray_against_node (recursive): returns if the ray
 * misses the node cube (ivp_ray_solver_check_cube); otherwise tests the
 * node's elements from last to first (core sphere at the last PSI with
 * upper_limit_radius, ivp_ray_solver_check_sphere, then
 * ivp_ray_solver_check_object) and recurses into the children from last
 * to first.  `node` may be NULL. */
void ivp_ray_solver_check_node(ivp_ray_solver_t *rs, const struct ivp_ov_node *node,
                               const struct ivp_ov_tree *ov_tree);

/* Used by the polygon surface manager (insert_all_ledges_hitting_ray):
 * transform the ray into `object` space and test `surface`. */
void ivp_ray_solver_insert_surface_hits(ivp_ray_solver_t *rs,
                                        struct ivp_object *object,
                                        const ivp_compact_surface_t *surface);

/* ── Object space solver (IVP_Ray_Solver_Os) ────────────────────────────── */

typedef struct ivp_ray_solver_os {
    double   ray_start_point[3];
    float    ray_center_point[3];
    double   ray_end_point[3];
    float    ray_direction[3];
    float    ray_length;
    ivp_ray_hit_listener_fn add_hit;
    void                   *listener;
    struct ivp_object      *object;
} ivp_ray_solver_os_t;

/* Ray of `rs` transformed into the object space of `object` (object
 * transform at the environment's current time). */
void ivp_ray_solver_os_init(ivp_ray_solver_os_t *os,
                            const ivp_ray_solver_t *rs,
                            struct ivp_object *object);

/* Ray given directly in object space (object may be NULL). */
void ivp_ray_solver_os_init_local(ivp_ray_solver_os_t *os,
                                  const double start_os[3],
                                  const double dir_os[3],
                                  double length,
                                  ivp_ray_hit_listener_fn add_hit,
                                  void *listener,
                                  struct ivp_object *object);

void ivp_ray_solver_os_check_compact_surface(ivp_ray_solver_os_t *os,
                                             const ivp_compact_surface_t *surface);
void ivp_ray_solver_os_check_ledgetree_node(ivp_ray_solver_os_t *os,
                                            const ivp_compact_ledgetree_node_t *node);
bool ivp_ray_solver_os_check_compact_ledge(ivp_ray_solver_os_t *os,
                                           const ivp_compact_ledge_t *ledge);

/* ── Closest hit (IVP_Ray_Solver_Min) ───────────────────────────────────── */

typedef struct ivp_ray_solver_min {
    ivp_ray_solver_t base;
    float            min_dist;   /* FLT_MAX while no hit */
    ivp_ray_hit_t    ray_hit;
} ivp_ray_solver_min_t;

void ivp_ray_solver_min_init(ivp_ray_solver_min_t *rs, const ivp_ray_solver_template_t *templ);
/* NULL if nothing was hit. */
const ivp_ray_hit_t *ivp_ray_solver_min_get_ray_hit(const ivp_ray_solver_min_t *rs);

/* ── All hits (IVP_Ray_Solver_Min_Hash) ─────────────────────────────────── */

typedef struct ivp_ray_solver_min_hash {
    ivp_ray_solver_t base;
    ivp_ray_hit_t    hit_info[IVP_MAX_NUM_RAY_HITS];
    double           hit_key[IVP_MAX_NUM_RAY_HITS]; /* min hash value: the unrounded
                                                     * (IVP_DOUBLE) hit distance */
    int              n_hits;     /* further hits are dropped (reference) */
} ivp_ray_solver_min_hash_t;

void ivp_ray_solver_min_hash_init(ivp_ray_solver_min_hash_t *rs,
                                  const ivp_ray_solver_template_t *templ);

/* The reference result container (get_result_min_hash, an IVP_U_Min_Hash
 * of size 8): adds every hit (elem = const ivp_ray_hit_t *, value =
 * hit_key) in insertion order to `out`, which the caller has initialised
 * (ivp_min_hash_init) and destroys.  Returns IVP_OK or the first add error. */
ivp_result_t ivp_ray_solver_min_hash_fill_min_hash(const ivp_ray_solver_min_hash_t *rs,
                                                   struct ivp_min_hash *out);

/* Hits in the order the reference IVP_U_Min_Hash hands them out
 * (find_min_elem / remove_min): increasing double distance (hit_key), equal
 * distances in insertion (traversal) order.  Writes up to `capacity`
 * pointers, returns the number of hits. */
int ivp_ray_solver_min_hash_get_sorted(const ivp_ray_solver_min_hash_t *rs,
                                       const ivp_ray_hit_t **out,
                                       int capacity);

/* ── Ray groups (IVP_Ray_Solver_Group) ──────────────────────────────────── */

typedef struct ivp_ray_solver_group {
    float              center_ws[3];
    float              radius;
    int                n_ray_solvers;
    ivp_ray_solver_t **ray_solvers;
} ivp_ray_solver_group_t;

void ivp_ray_solver_group_init(ivp_ray_solver_group_t *g, int n_ray_solvers,
                               ivp_ray_solver_t **ray_solvers);
/* IVP_Ray_Solver_Group::check_ray_group_against_object (filters of the
 * first ray; the rays are tested from last to first). */
void ivp_ray_solver_group_check_object(ivp_ray_solver_group_t *g, struct ivp_object *object);
/* IVP_Ray_Solver_Group::check_ray_group_against_cube: true unless the
 * group sphere misses the cube (center, edge length). */
bool ivp_ray_solver_group_check_cube(const ivp_ray_solver_group_t *g,
                                     const float cube_center_ws[3], float cube_size);
/* IVP_Ray_Solver_Group::check_ray_group_against_node (same walk as
 * ivp_ray_solver_check_node, tested against the group sphere). */
void ivp_ray_solver_group_check_node(ivp_ray_solver_group_t *g, const struct ivp_ov_node *node,
                                     const struct ivp_ov_tree *ov_tree);
/* IVP_Ray_Solver_Group::check_ray_group_against_all_objects_in_sim */
void ivp_ray_solver_group_check_all_objects(ivp_ray_solver_group_t *g,
                                            const struct ivp_environment *env);

/* ── Single surface helpers (object space) ──────────────────────────────── */

typedef struct ivp_ray_input {
    ivp_vec3r_t origin;       /* ray start point (object space) */
    ivp_vec3r_t direction;    /* normalized ray direction */
    ivp_real_t  max_distance; /* maximum ray length */
} ivp_ray_input_t;

/* Closest reference hit of the ray against a compact surface (object
 * space).  Returns IVP_OK on hit, IVP_NOT_FOUND otherwise. */
ivp_result_t ivp_ray_cast_surface(const ivp_compact_surface_t *surface,
                                   const ivp_ray_input_t      *ray,
                                   ivp_ray_hit_t              *hit_out);

/* Reference hit of the ray against one convex ledge (object space). */
ivp_result_t ivp_ray_cast_ledge(const ivp_compact_ledge_t *ledge,
                                 const ivp_ray_input_t     *ray,
                                 ivp_ray_hit_t             *hit_out);

/* Two sided plane + barycentric test against a single triangle
 * (libivp utility, not part of the reference). */
ivp_result_t ivp_ray_test_triangle(const ivp_compact_ledge_t    *ledge,
                                    const ivp_compact_triangle_t *tri,
                                    const ivp_ray_input_t        *ray,
                                    ivp_real_t                   *hit_dist);

/* Infinite ray vs sphere (libivp utility).  If hit, *t_out receives the
 * entry distance (exit distance when the origin is inside). */
bool ivp_ray_test_sphere(const ivp_vec3r_t *origin,
                         const ivp_vec3r_t *direction,
                         const ivp_vec3r_t *center,
                         ivp_real_t         radius,
                         ivp_real_t        *t_out);

#ifdef __cplusplus
}
#endif

#endif /* IVP_RAY_SOLVER_H */
