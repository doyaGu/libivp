/* ivp_contact_order.c -- Shock propagation contact ordering implementation
 *
 * Sorts contacts by gravity-projected height (bottom contacts first).
 * This ensures lower contacts are solved before upper contacts in
 * stacking scenarios, improving stability.
 */

#include <ivp/solver/ivp_contact_order.h>
#include <ivp/dynamics/ivp_object.h>

/* Compute the gravity-projected height of a contact point.
 * For Y-down gravity, higher dot product = lower in the stack. */
static ivp_real_t contact_gravity_height(const ivp_persistent_contact_t *pc,
                                          const ivp_vec3r_t *gravity_dir)
{
    /* Use the midpoint of the two contact points projected onto gravity */
    ivp_vec3r_t mid;
    mid.k[0] = (pc->cp.point_a.k[0] + pc->cp.point_b.k[0]) * IVP_REAL(0.5);
    mid.k[1] = (pc->cp.point_a.k[1] + pc->cp.point_b.k[1]) * IVP_REAL(0.5);
    mid.k[2] = (pc->cp.point_a.k[2] + pc->cp.point_b.k[2]) * IVP_REAL(0.5);
    return ivp_vec3r_dot(&mid, gravity_dir);
}

int ivp_contact_order_shock_propagation(ivp_persistent_contact_t **contacts,
                                         int count,
                                         const ivp_vec3r_t *gravity_dir)
{
    if (!contacts || count <= 1 || !gravity_dir) return count;

    /* Insertion sort (small N, stable, cache-friendly for pointers).
     * Sort by descending gravity projection: in Y-down gravity,
     * contacts with higher dot(pos, gravity) are lower in the stack
     * and should be solved first. */
    for (int i = 1; i < count; i++) {
        ivp_persistent_contact_t *key = contacts[i];
        ivp_real_t key_h = contact_gravity_height(key, gravity_dir);
        int j = i - 1;
        while (j >= 0 && contact_gravity_height(contacts[j], gravity_dir) < key_h) {
            contacts[j + 1] = contacts[j];
            j--;
        }
        contacts[j + 1] = key;
    }

    return count;
}
