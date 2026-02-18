/* ivp_contact.h — Contact detection and contact point for IVP C17
 *
 * Provides:
 *   - Contact point structure (normal, points, penetration, material)
 *   - Range-manager-aware N² broadphase with adaptive speed-based margins
 *   - Virtual mass computation at contact point
 *   - Material combination rules (friction×, elasticity×, adhesion+)
 *   - Closing speed computation
 *   - Impulse application
 */
#ifndef IVP_CONTACT_H
#define IVP_CONTACT_H

#include <ivp/base/ivp_types.h>
#include <ivp/math/ivp_vec3.h>
#include <ivp/solver/ivp_solver_constants.h>
#include <ivp/collision/ivp_collision_tolerance.h>
#include <ivp/base/ivp_forward_decls.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Contact point ──────────────────────────────────────────────────────── */

typedef struct ivp_contact_point {
    /* Object references (not owned) */
    ivp_object_t   *obj_a;
    ivp_object_t   *obj_b;

    /* Geometry (world space) */
    ivp_vec3r_t     normal;         /* surface normal A→B, unit vector        */
    ivp_vec3r_t     point_a;        /* contact point on surface A (world)     */
    ivp_vec3r_t     point_b;        /* contact point on surface B (world)     */
    ivp_real_t      gap_length;     /* separation distance (negative = pen.)  */

    /* Offsets from center of mass (world space) */
    ivp_vec3r_t     r_a;            /* contact_point_a - pos_a                */
    ivp_vec3r_t     r_b;            /* contact_point_b - pos_b                */

    /* Cross products for angular impulse (core space) */
    ivp_vec3r_t     cross_normal_cs_a;  /* r_a × normal (in A's core space)   */
    ivp_vec3r_t     cross_normal_cs_b;  /* r_b × normal (in B's core space)   */

    /* Friction tangent directions (world space) */
    ivp_vec3r_t     span_dirs[2];   /* orthogonal tangent plane basis         */

    /* Material properties (combined) */
    ivp_real_t      friction;       /* combined Coulomb coefficient            */
    ivp_real_t      elasticity;     /* combined coefficient of restitution     */
    ivp_real_t      adhesion;       /* combined adhesion (additive)            */

    /* Precomputed virtual mass along normal */
    ivp_real_t      virtual_mass;   /* reduced mass along normal direction     */
    ivp_real_t      inv_virtual_mass;

    /* Persistent state (for friction across PSIs) */
    int             age_psi;        /* number of PSIs this contact has existed */
    bool            is_active;      /* currently in contact                    */
} ivp_contact_point_t;

/* ── Contact pair list (per-environment) ────────────────────────────────── */

typedef struct ivp_contact_list {
    ivp_contact_point_t contacts[IVP_MAX_CONTACT_PAIRS];
    int count;
} ivp_contact_list_t;

/* ── Contact geometry computation ───────────────────────────────────────── */

/* Compute full contact geometry for a sphere-sphere contact.
 * Fills normal, points, gap_length, r_a, r_b, cross products, spans.
 * obj_a and obj_b must already be set. */
void ivp_contact_compute_sphere_sphere(ivp_contact_point_t *cp);

/* Compute contact geometry for a sphere vs polygon contact.
 * The sphere object must be obj_a, the polygon must be obj_b.
 * Uses the polygon's surman to get the convex ledge, transforms
 * the sphere center into the polygon's object space, finds the closest
 * point on the ledge, then transforms results back to world space. */
void ivp_contact_compute_sphere_polygon(ivp_contact_point_t *cp);

/* Compute contact geometry for two polygon objects.
 * Uses ivp_find_closest_pair with vertices transformed to world space.
 * Both objects must have a valid surman with a single convex ledge. */
void ivp_contact_compute_polygon_polygon(ivp_contact_point_t *cp);

/* Generic contact computation: dispatches to the correct function
 * based on the object types of obj_a and obj_b.
 * Also computes virtual mass, materials, and cross products. */
void ivp_contact_compute_generic(ivp_contact_point_t *cp);

/* Refresh only geometry-related terms for an existing contact pair.
 * Keeps object ordering and recomputes normal/points/gap/r/cross/span. */
void ivp_contact_refresh_geometry(ivp_contact_point_t *cp);

/* Compute one or more contacts for an object pair.
 * For most pairs this returns 1 contact.
 * For polygon-polygon pairs this may return a manifold-style set
 * of up to 4 stable contacts.
 *
 * contacts_out must have space for max_contacts elements.
 * Returns number of contacts written (0..max_contacts). */
int ivp_contact_compute_generic_multi(ivp_object_t *obj_a,
                                      ivp_object_t *obj_b,
                                      ivp_contact_point_t *contacts_out,
                                      int max_contacts);

/* Compute virtual mass along the contact normal.
 * Must be called after contact geometry is set. */
void ivp_contact_compute_virtual_mass(ivp_contact_point_t *cp);

/* Compute combined material properties.
 * friction: multiply.  elasticity: multiply.  adhesion: add. */
void ivp_contact_combine_materials(ivp_contact_point_t *cp);

/* ── Velocities at contact ──────────────────────────────────────────────── */

/* Compute closing speed along contact normal.
 * Positive = approaching, negative = separating. */
ivp_real_t ivp_contact_closing_speed(const ivp_contact_point_t *cp);

/* Compute relative velocity at contact point (world space).
 * v_rel = v_b + ω_b×r_b - v_a - ω_a×r_a */
void ivp_contact_relative_velocity(const ivp_contact_point_t *cp,
                                    ivp_vec3r_t *v_rel_out);

/* ── Impulse application ────────────────────────────────────────────────── */

/* Apply impulse along contact normal to both objects.
 * Positive impulse pushes A away from B (along +normal for B, -normal for A).
 *   v_a -= impulse * inv_mass_a * normal
 *   ω_a -= inv_I_a * (r_a × (impulse * normal))   [core space]
 *   v_b += impulse * inv_mass_b * normal
 *   ω_b += inv_I_b * (r_b × (impulse * normal))   [core space] */
void ivp_contact_apply_impulse(ivp_contact_point_t *cp, ivp_real_t impulse);

/* Apply impulse in an arbitrary world-space direction. */
void ivp_contact_apply_impulse_dir(ivp_contact_point_t *cp,
                                    const ivp_vec3r_t *direction,
                                    ivp_real_t impulse);

/* ── Virtual mass utility ───────────────────────────────────────────────── */

/* Compute virtual mass of a single core along a given direction at offset r.
 * virt_mass = 1 / (inv_mass + ((inv_I * (r × n)) · (r × n)))
 * Returns the virtual mass (not inverse). */
ivp_real_t ivp_calc_virtual_mass(const ivp_core_t *core,
                                  const ivp_vec3r_t *r_world,
                                  const ivp_vec3r_t *normal);

#ifdef __cplusplus
}
#endif

#endif /* IVP_CONTACT_H */
