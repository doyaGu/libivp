/* ivp_ov_tree.c — port of IVP_OV_Tree_Manager / IVP_OV_Element / IVP_OV_Node
 * (ivp_collision/ivp_clustering_longrange.cxx) and IVP_U_FVector<IVP_Collision>.
 */
#include <ivp/collision/ivp_collision_core.h>
#include <ivp/dynamics/ivp_object.h>
#include "dynamics/ivp_env_internal.h"
#include "dynamics/ivp_env_hot.h"
#include <ivp/base/ivp_assert.h>
#include <ivp/base/ivp_message.h>
#ifdef IVP_HOME_BUILD
#include <ivp/diagnostics/ivp_clustering_visualizer.h>
#endif
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>


/* ── IVP_U_FVector<IVP_Collision> ───────────────────────────────────────── */

static void coll_set_fvector_index(ivp_collision_t *c, int old, int i)
{
    if (c->fvector_index[0] == old) {
        c->fvector_index[0] = i;
    } else {
        IVP_ASSERT(c->fvector_index[1] == old);
        c->fvector_index[1] = i;
    }
}

void ivp_coll_fvector_init(ivp_coll_fvector_t *v) { v->elems = NULL; v->n = 0; v->cap = 0; }

void ivp_coll_fvector_clear(ivp_coll_fvector_t *v)
{
    free(v->elems);
    v->elems = NULL; v->n = 0; v->cap = 0;
}

int ivp_coll_fvector_index_of(const ivp_coll_fvector_t *v, const ivp_collision_t *c)
{
    int i0 = c->fvector_index[0];
    if (i0 >= 0 && i0 < v->n && v->elems[i0] == c) return i0;
    int i1 = c->fvector_index[1];
    IVP_ASSERT(i1 < 0 || i1 >= v->n || v->elems[i1] == c);
    if (i1 >= 0 && i1 < v->n && v->elems[i1] == c) return i1;
    return -1;
}

int ivp_coll_fvector_add(ivp_coll_fvector_t *v, ivp_collision_t *c)
{
    if (v->n >= v->cap) {
        int nc = v->cap * 2 + 1;
        if (nc < 8) nc = 8;
        ivp_collision_t **ne = (ivp_collision_t **)realloc(v->elems, (size_t)nc * sizeof(*ne));
        IVP_ASSERT(ne);
        if (!ne) return -1;
        v->elems = ne;
        v->cap = nc;
    }
    IVP_ASSERT(ivp_coll_fvector_index_of(v, c) == -1);
    v->elems[v->n] = c;
    coll_set_fvector_index(c, -1, v->n);
    return v->n++;
}

void ivp_coll_fvector_swap(ivp_coll_fvector_t *v, int index1, int index2)
{
    IVP_ASSERT((index1 >= 0) && (index1 < v->n));
    IVP_ASSERT((index2 >= 0) && (index2 < v->n));
    ivp_collision_t *a = v->elems[index1];
    ivp_collision_t *b = v->elems[index2];
    v->elems[index1] = b;
    v->elems[index2] = a;
    coll_set_fvector_index(a, index1, index2);
    coll_set_fvector_index(b, index2, index1);
}

void ivp_coll_fvector_remove_allow_resort(ivp_coll_fvector_t *v, ivp_collision_t *c)
{
    int index = ivp_coll_fvector_index_of(v, c);
    IVP_ASSERT(index >= 0);
    if (index < 0) return;
    v->n--;
    if (v->n > index) {
        ivp_collision_t *e = v->elems[v->n];
        v->elems[index] = e;
        coll_set_fvector_index(e, v->n, index);
    }
    coll_set_fvector_index(c, index, -1);
}

/* ── IVP_OV_Element ─────────────────────────────────────────────────────── */

static void ov_element_hull_limit_exceeded(ivp_listener_hull_t *lh, ivp_hull_mgr_t *hm, float v)
{
    (void)v;
    ivp_ov_element_t *el = (ivp_ov_element_t *)lh;
    IVP_ASSERT(hm == el->hull_manager);
    (void)hm;
    ivp_mindist_mgr_recheck_ov_element(ivp_env_get_mindist_mgr(el->real_object->owner),
                                       el->real_object);
}

static void ov_element_hull_manager_deleted(ivp_listener_hull_t *lh, ivp_hull_mgr_t *hm)
{
    ivp_ov_element_t *el = (ivp_ov_element_t *)lh;
    IVP_ASSERT(hm == el->hull_manager);
    (void)hm;
    ivp_ov_element_destroy(el);
}

static const ivp_listener_hull_vt_t ov_element_vt = {
    IVP_HULL_ELEM_OO_CONNECTOR,
    ov_element_hull_limit_exceeded,
    ov_element_hull_manager_deleted,
    NULL
};

ivp_ov_element_t *ivp_ov_element_create(struct ivp_object *obj)
{
    ivp_ov_element_t *el = (ivp_ov_element_t *)calloc(1, sizeof(*el));
    IVP_ASSERT(el);
    if (!el) return NULL;
    ivp_listener_hull_init(&el->lh, &ov_element_vt);
    el->node = NULL;
    el->radius = -1.0f;
    el->real_object = obj;
    el->hull_manager = NULL;
    ivp_coll_fvector_init(&el->collision_fvector);
    return el;
}

void ivp_ov_element_destroy(ivp_ov_element_t *el)
{
    if (el->hull_manager) {
        ivp_hull_mgr_remove_synapse(el->hull_manager, &el->lh);
        el->hull_manager = NULL;
    }
    struct ivp_environment *env = el->real_object->owner;
    ivp_env_fire_object_is_removed_from_collision_detection(env, el->real_object);
    IVP_ASSERT(el->collision_fvector.n == 0);
    ivp_ov_tree_remove_ov_element(ivp_env_get_ov_tree(env), el);
    ivp_coll_fvector_clear(&el->collision_fvector);
    if (el->real_object->ov_element == el) el->real_object->ov_element = NULL;
    free(el);
}

void ivp_ov_element_add_to_hull_manager(ivp_ov_element_t *el, ivp_hull_mgr_t *hm, double hull_time)
{
    ivp_time_t current_time = ivp_env_get_current_time(el->real_object->owner);
    if (el->hull_manager) {
        IVP_ASSERT(el->hull_manager == hm);
        ivp_hull_mgr_update_synapse(el->hull_manager, &el->lh, current_time, hull_time);
        return;
    }
    el->hull_manager = hm;
    ivp_hull_mgr_insert_synapse(hm, &el->lh, current_time, hull_time);
}

void ivp_ov_element_add_oo_collision(ivp_ov_element_t *el, ivp_collision_t *c)
{
    ivp_coll_fvector_add(&el->collision_fvector, c);
}

void ivp_ov_element_remove_oo_collision(ivp_ov_element_t *el, ivp_collision_t *c)
{
    ivp_coll_fvector_remove_allow_resort(&el->collision_fvector, c);
}

/* ── IVP_OV_Node ────────────────────────────────────────────────────────── */

static ivp_ov_node_t *ov_node_create(void)
{
    ivp_ov_node_t *n = (ivp_ov_node_t *)calloc(1, sizeof(*n));
    IVP_ASSERT(n);
    if (!n) return NULL;
    ivp_pvec_init(&n->children);
    ivp_pvec_init(&n->elements);
    return n;
}

static void ov_node_destroy(ivp_ov_node_t *n)
{
    if (n->parent) {
        ivp_pvec_remove(&n->parent->children, n);
    }
    while (ivp_pvec_len(&n->children)) {
        ov_node_destroy((ivp_ov_node_t *)ivp_pvec_at(&n->children, 0));
    }
    ivp_pvec_clear(&n->children);
    ivp_pvec_clear(&n->elements);
    free(n);
}

/* ── node lookup (IVP_ov_tree_hash; lookup only, order-free) ─────────── */

static unsigned ov_hash_value(const ivp_ov_node_data_t *d)
{
    return (unsigned)d->x * 73856093u ^ (unsigned)d->y * 19349663u ^
           (unsigned)d->z * 83492791u ^ (unsigned)d->rasterlevel * 2654435761u;
}

static size_t ov_hash_index(const ivp_ov_tree_t *t, const ivp_ov_node_data_t *d)
{
    return (size_t)ov_hash_value(d) & (t->hash_size - 1u);
}

static bool ov_node_equal(const ivp_ov_node_t *a, const ivp_ov_node_t *b)
{
    return a->data.rasterlevel == b->data.rasterlevel && a->data.x == b->data.x &&
           a->data.y == b->data.y && a->data.z == b->data.z;
}

static void ov_hash_grow(ivp_ov_tree_t *t)
{
    if (t->hash_size > SIZE_MAX / 2u / sizeof(*t->hash)) return;
    const size_t new_size = t->hash_size * 2u;
    ivp_ov_node_t **new_hash = (ivp_ov_node_t **)ivp_alloc(t->allocator,
                                                           new_size * sizeof(*new_hash));
    if (!new_hash) return;
    memset(new_hash, 0, new_size * sizeof(*new_hash));
    for (size_t i = 0; i < t->hash_size; i++) {
        ivp_ov_node_t *node = t->hash[i];
        while (node) {
            ivp_ov_node_t *next = node->hash_next;
            const size_t new_index = (size_t)ov_hash_value(&node->data) & (new_size - 1u);
            node->hash_next = new_hash[new_index];
            new_hash[new_index] = node;
            node = next;
        }
    }
    ivp_free(t->allocator, t->hash, t->hash_size * sizeof(*t->hash));
    t->hash = new_hash;
    t->hash_size = new_size;
}

static void ov_hash_add(ivp_ov_tree_t *t, ivp_ov_node_t *n)
{
    if ((t->node_count + 1u) * 4u > t->hash_size * 3u) ov_hash_grow(t);
    size_t i = ov_hash_index(t, &n->data);
    n->hash_next = t->hash[i];
    t->hash[i] = n;
    t->node_count++;
}

static void ov_hash_remove(ivp_ov_tree_t *t, ivp_ov_node_t *n)
{
    size_t i = ov_hash_index(t, &n->data);
    ivp_ov_node_t **pp = &t->hash[i];
    while (*pp) {
        if (*pp == n) {
            *pp = n->hash_next;
            n->hash_next = NULL;
            IVP_ASSERT(t->node_count > 0);
            t->node_count--;
            return;
        }
        pp = &(*pp)->hash_next;
    }
}

static ivp_ov_node_t *ov_hash_find(ivp_ov_tree_t *t, const ivp_ov_node_t *key)
{
    for (ivp_ov_node_t *n = t->hash[ov_hash_index(t, &key->data)]; n; n = n->hash_next) {
        if (ov_node_equal(n, key)) return n;
    }
    return NULL;
}

/* ── IVP_OV_Tree_Manager ────────────────────────────────────────────────── */

ivp_result_t ivp_ov_tree_init_with_allocator(ivp_ov_tree_t *t, const ivp_allocator_t *allocator)
{
    if (!t) return IVP_ERR_INVALID_ARG;
    memset(t, 0, sizeof(*t));
    t->allocator = allocator ? allocator : ivp_default_allocator();
    t->hash_size = IVP_OV_HASH_INITIAL_SIZE;
    t->hash = (ivp_ov_node_t **)ivp_alloc(t->allocator, t->hash_size * sizeof(*t->hash));
    if (!t->hash) {
        t->hash_size = 0;
        return IVP_ERR_OUT_OF_MEMORY;
    }
    memset(t->hash, 0, t->hash_size * sizeof(*t->hash));
    double y = 1.0, iy = 1.0;
    for (int x = 0; x <= 40; x++) {
        t->powerlist[40 + x] = y;
        t->powerlist[40 - x] = iy;
        y *= 2.0;
        iy *= 0.5;
    }
    return IVP_OK;
}

void ivp_ov_tree_init(ivp_ov_tree_t *t)
{
    ivp_result_t result = ivp_ov_tree_init_with_allocator(t, ivp_default_allocator());
    IVP_ASSERT(result == IVP_OK);
    (void)result;
}

void ivp_ov_tree_destroy(ivp_ov_tree_t *t)
{
    /* reference ~IVP_OV_Tree_Manager only deletes the hash; remaining nodes
     * are owned by elements that are destroyed with their objects. */
    if (t->root) {
        ov_node_destroy(t->root);
        t->root = NULL;
    }
    if (t->hash) {
        ivp_free(t->allocator, t->hash, t->hash_size * sizeof(*t->hash));
        t->hash = NULL;
    }
    t->hash_size = 0;
    t->node_count = 0;
}

static inline double ov_power2(const ivp_ov_tree_t *t, int x) { return t->powerlist[x + 40]; }

/* PFM_LD(double): exponent of the IEEE representation */
static int ov_log_base2(double x)
{
    /* biased IEEE exponent - 0x3ff (also for 0 / denormals, unlike frexp) */
    uint64_t bits;
    memcpy(&bits, &x, sizeof(bits));
    return (int)((bits >> 52) & 0x7ff) - 0x3ff;
}

/* reference non-LINUX builds: (int)floorf(x) / (int)ceilf(x) (float rounding) */
static inline int ivp_int_floor(double x) { return (int)floorf((float)x); }
static inline int ivp_int_ceil(double x) { return (int)ceilf((float)x); }

static double ivp_raster_scale(int rasterlevel_diff)
{
    if (rasterlevel_diff <= 0) {
        IVP_ASSERT(rasterlevel_diff >= 0);   /* ivp_clustering_longrange.cxx:151 */
        return 1.0;
    }
    double scale = 1.0;
    for (int i = 0; i < rasterlevel_diff; i++) scale *= 2.0;
    return scale;
}

static bool box_contains_box(const ivp_ov_node_data_t *master_data, const ivp_ov_node_t *sub_node,
                             int rasterlevel_diff)
{
    const double scale = ivp_raster_scale(rasterlevel_diff);
    double master_x1 = master_data->x * scale;
    double master_y1 = master_data->y * scale;
    double master_z1 = master_data->z * scale;
    double offset = 2.0 * scale - 2.0;
    if (sub_node->data.x < master_x1) return false;
    if (sub_node->data.y < master_y1) return false;
    if (sub_node->data.z < master_z1) return false;
    if (sub_node->data.x > (master_x1 + offset)) return false;
    if (sub_node->data.y > (master_y1 + offset)) return false;
    if (sub_node->data.z > (master_z1 + offset)) return false;
    return true;
}

static double calc_optimal_box(ivp_ov_tree_t *t, const ivp_ov_element_t *element,
                               double min_radius, double max_radius)
{
    double raster_size = 2.0 * min_radius;
    int raster_level = ov_log_base2(raster_size) + 1;
    int raster_points;
    if (raster_level < -40) {
        raster_level = -40;
        IVP_IF(1) {   /* ivp_clustering_longrange.cxx:218 */
            ivp_debug_printf("IVP_OV_Tree_Manager - WARNING: Object too small! Adjusting raster size to "
                             "2^-40 meters.\n");
        }
    }

    int x_min, x_max, y_min, y_max, z_min, z_max;
    for (;; raster_level++) {
        raster_points = raster_level - 1;
        double iraster_dist = ov_power2(t, -raster_points);
        x_min = ivp_int_floor(((double)element->center.k[0] - min_radius) * iraster_dist);
        x_max = ivp_int_ceil(((double)element->center.k[0] + min_radius) * iraster_dist);
        if (x_max > x_min + 2) continue;
        y_min = ivp_int_floor(((double)element->center.k[1] - min_radius) * iraster_dist);
        y_max = ivp_int_ceil(((double)element->center.k[1] + min_radius) * iraster_dist);
        if (y_max > y_min + 2) continue;
        z_min = ivp_int_floor(((double)element->center.k[2] - min_radius) * iraster_dist);
        z_max = ivp_int_ceil(((double)element->center.k[2] + min_radius) * iraster_dist);
        if (z_max > z_min + 2) continue;
        break;
    }
    t->search_node.data.x = x_min;
    t->search_node.data.y = y_min;
    t->search_node.data.z = z_min;
    t->search_node.data.rasterlevel = raster_points;
    t->search_node.data.sizelevel = raster_level;

    if (min_radius >= max_radius) return min_radius;

    raster_level++;
    raster_points = raster_level - 1;
    double iraster_dist = ov_power2(t, -raster_points);
    x_min = ivp_int_floor(((double)element->center.k[0] - max_radius) * iraster_dist);
    x_max = ivp_int_ceil(((double)element->center.k[0] + max_radius) * iraster_dist);
    if (x_max > x_min + 2) return min_radius;
    y_min = ivp_int_floor(((double)element->center.k[1] - max_radius) * iraster_dist);
    y_max = ivp_int_ceil(((double)element->center.k[1] + max_radius) * iraster_dist);
    if (y_max > y_min + 2) return min_radius;
    z_min = ivp_int_floor(((double)element->center.k[2] - max_radius) * iraster_dist);
    z_max = ivp_int_ceil(((double)element->center.k[2] + max_radius) * iraster_dist);
    if (z_max > z_min + 2) return min_radius;

    t->search_node.data.x = x_min;
    t->search_node.data.y = y_min;
    t->search_node.data.z = z_min;
    t->search_node.data.rasterlevel = raster_points;
    t->search_node.data.sizelevel = raster_level;
    return max_radius;
}

static bool box_overlaps_with_box(const ivp_ov_node_t *largenode, const ivp_ov_node_t *smallnode,
                                  int rasterlevel_diff)
{
    const double scale = ivp_raster_scale(rasterlevel_diff);
    if ((double)smallnode->data.x + 2.0 <= largenode->data.x * scale) return false;
    if ((double)smallnode->data.y + 2.0 <= largenode->data.y * scale) return false;
    if ((double)smallnode->data.z + 2.0 <= largenode->data.z * scale) return false;
    if (smallnode->data.x >= ((double)largenode->data.x + 2.0) * scale) return false;
    if (smallnode->data.y >= ((double)largenode->data.y + 2.0) * scale) return false;
    if (smallnode->data.z >= ((double)largenode->data.z + 2.0) * scale) return false;
    return true;
}

static ivp_ov_node_t *find_smallest_box(const ivp_ov_node_t *master_node, const ivp_ov_node_t *sub_node)
{
    int rasterlevel_diff = (master_node->data.rasterlevel - 1) - sub_node->data.rasterlevel;
    for (int i = 0; i < ivp_pvec_len(&master_node->children); i++) {
        const ivp_ov_node_t *child = (const ivp_ov_node_t *)ivp_pvec_at(&master_node->children, i);
        if (box_contains_box(&child->data, sub_node, rasterlevel_diff)) {
            return find_smallest_box(child, sub_node);
        }
    }
    return (ivp_ov_node_t *)master_node;
}

static void connect_boxes(ivp_ov_tree_t *t, ivp_ov_node_t *node, ivp_ov_node_t *new_node)
{
    int rasterlevel_diff = node->data.sizelevel - new_node->data.sizelevel;
    IVP_ASSERT(rasterlevel_diff > 0);
    const double scale = ivp_raster_scale(rasterlevel_diff);
    const double half_scale = scale * 0.5;
    if (rasterlevel_diff == 1) {
        new_node->parent = node;
        ivp_pvec_add(&node->children, new_node);
        /* MY_DEBUG is defined at the top of the reference file (every build) */
        if (ivp_pvec_len(&node->children) > 27)   /* ivp_clustering_longrange.cxx:424 */
            ivp_debug_printf("*** ERROR *** Excessive amount of children: %d\n", ivp_pvec_len(&node->children));
        return;
    }
    ivp_ov_node_t *new_subnode = ov_node_create();
    if (new_node->data.x >= ((double)node->data.x + 1.0) * scale) {
        new_subnode->data.x = (node->data.x + 1) * 2;
    } else if (new_node->data.x >= ((double)node->data.x * 2.0 + 1.0) * half_scale) {
        new_subnode->data.x = (node->data.x * 2) + 1;
    } else {
        new_subnode->data.x = node->data.x * 2;
    }
    if (new_node->data.y >= ((double)node->data.y + 1.0) * scale) {
        new_subnode->data.y = (node->data.y + 1) * 2;
    } else if (new_node->data.y >= ((double)node->data.y * 2.0 + 1.0) * half_scale) {
        new_subnode->data.y = (node->data.y * 2) + 1;
    } else {
        new_subnode->data.y = node->data.y * 2;
    }
    if (new_node->data.z >= ((double)node->data.z + 1.0) * scale) {
        new_subnode->data.z = (node->data.z + 1) * 2;
    } else if (new_node->data.z >= ((double)node->data.z * 2.0 + 1.0) * half_scale) {
        new_subnode->data.z = (node->data.z * 2) + 1;
    } else {
        new_subnode->data.z = node->data.z * 2;
    }
    new_subnode->data.rasterlevel = node->data.rasterlevel - 1;
    new_subnode->data.sizelevel = node->data.sizelevel - 1;
    new_subnode->parent = node;
    ivp_pvec_add(&node->children, new_subnode);
    ov_hash_add(t, new_subnode);
    if (ivp_pvec_len(&node->children) > 27)   /* ivp_clustering_longrange.cxx:499 (MY_DEBUG) */
        ivp_debug_printf("*** ERROR *** Excessive amount of children: %d\n", ivp_pvec_len(&node->children));
    connect_boxes(t, new_subnode, new_node);
}

static void expand_tree(ivp_ov_tree_t *t, const ivp_ov_node_t *new_node)
{
    /* DEBUG_ERROR_OUTPUT is defined at the top of the reference file (every build) */
    if (t->root->data.sizelevel > 40)   /* ivp_clustering_longrange.cxx:533 */
        ivp_debug_printf("*** ERROR *** Excessive sizelevel (%d) for element\n", t->root->data.sizelevel);
    IVP_ASSERT(t->root->data.sizelevel < 41);
    ivp_ov_node_t *old_root = t->root;
    ivp_ov_node_t *new_root = ov_node_create();
    int rest_x = old_root->data.x % 2;
    int rest_y = old_root->data.y % 2;
    int rest_z = old_root->data.z % 2;
    new_root->data.x = old_root->data.x / 2;   /* IVP_Inline_Math::int_div_2 */
    new_root->data.y = old_root->data.y / 2;
    new_root->data.z = old_root->data.z / 2;
    new_root->data.rasterlevel = old_root->data.rasterlevel + 1;
    new_root->data.sizelevel = old_root->data.sizelevel + 1;

    double rasterpoints = ov_power2(t, new_root->data.rasterlevel);
    double rasterpoints_newnode = ov_power2(t, new_node->data.rasterlevel);
    if (rest_x == -1) {
        new_root->data.x--;
    } else if (rest_x == 0) {
        if ((new_node->data.x * rasterpoints_newnode) < (rasterpoints * new_root->data.x)) new_root->data.x--;
    }
    if (rest_y == -1) {
        new_root->data.y--;
    } else if (rest_y == 0) {
        if ((new_node->data.y * rasterpoints_newnode) < (rasterpoints * new_root->data.y)) new_root->data.y--;
    }
    if (rest_z == -1) {
        new_root->data.z--;
    } else if (rest_z == 0) {
        if ((new_node->data.z * rasterpoints_newnode) < (rasterpoints * new_root->data.z)) new_root->data.z--;
    }
    if (ivp_pvec_len(&new_root->children) >= 27)   /* ivp_clustering_longrange.cxx:634 (MY_DEBUG) */
        ivp_debug_printf("*** ERROR *** : Mehr als 27 Kinder *********************************\n");
    ivp_pvec_add(&new_root->children, old_root);
    old_root->parent = new_root;
    ov_hash_add(t, new_root);
    t->root = new_root;
}

static double ov_qdist_f(const ivp_vec3f_t *a, const ivp_vec3f_t *b)
{
    /* IVP_U_Float_Point::quad_distance_to: float differences stored in
     * IVP_DOUBLE locals, squared and summed in double */
    double dx = (double)(a->k[0] - b->k[0]);
    double dy = (double)(a->k[1] - b->k[1]);
    double dz = (double)(a->k[2] - b->k[2]);
    dx *= dx;
    dy *= dy;
    dz *= dz;
    return dx + dy + dz;
}

static void collect_subbox_collision_partners(ivp_ov_tree_t *t, const ivp_ov_element_t *elem,
                                              const ivp_ov_node_t *node)
{
    for (int i = ivp_pvec_len(&node->elements) - 1; i >= 0; i--) {
        ivp_ov_element_t *el = (ivp_ov_element_t *)ivp_pvec_at(&node->elements, i);
        double qdist = ov_qdist_f(&el->center, &elem->center);
        double minimal_dist = (double)(elem->radius + el->radius); /* IVP_FLOAT + IVP_FLOAT */
        if (qdist > minimal_dist * minimal_dist) continue;
        ivp_pvec_add(t->collision_partners, el);
    }
    for (int i = ivp_pvec_len(&node->children) - 1; i >= 0; i--) {
        collect_subbox_collision_partners(t, elem, (const ivp_ov_node_t *)ivp_pvec_at(&node->children, i));
    }
}

static void collect_collision_partners(ivp_ov_tree_t *t, const ivp_ov_element_t *elem,
                                       const ivp_ov_node_t *masternode, const ivp_ov_node_t *new_node)
{
    for (int i = ivp_pvec_len(&masternode->elements) - 1; i >= 0; i--) {
        ivp_ov_element_t *el = (ivp_ov_element_t *)ivp_pvec_at(&masternode->elements, i);
        double qdist = ov_qdist_f(&el->center, &elem->center);
        double minimal_dist = (double)(elem->radius + el->radius); /* IVP_FLOAT + IVP_FLOAT */
        if (qdist > minimal_dist * minimal_dist) continue;
        ivp_pvec_add(t->collision_partners, el);
    }
    if ((masternode->data.sizelevel - 1) > new_node->data.sizelevel) {
        int rasterlevel_diff = (masternode->data.rasterlevel - 1) - new_node->data.rasterlevel;
        for (int i = 0; i < ivp_pvec_len(&masternode->children); i++) {
            const ivp_ov_node_t *child = (const ivp_ov_node_t *)ivp_pvec_at(&masternode->children, i);
            if (child == new_node) {
                collect_subbox_collision_partners(t, elem, child);
            } else if (box_overlaps_with_box(child, new_node, rasterlevel_diff)) {
                collect_collision_partners(t, elem, child, new_node);
            }
        }
    } else {
        int rasterlevel_diff = new_node->data.rasterlevel - (masternode->data.rasterlevel - 1);
        for (int i = 0; i < ivp_pvec_len(&masternode->children); i++) {
            const ivp_ov_node_t *child = (const ivp_ov_node_t *)ivp_pvec_at(&masternode->children, i);
            if (child == new_node) {
                collect_subbox_collision_partners(t, elem, child);
            } else if (box_overlaps_with_box(new_node, child, rasterlevel_diff)) {
                collect_collision_partners(t, elem, child, new_node);
            }
        }
    }
}

double ivp_ov_tree_insert_ov_element(ivp_ov_tree_t *t, ivp_ov_element_t *element,
                                     double min_radius, double max_radius, ivp_pvec_t *colliding_balls)
{
    if (!element) return 0.0;
    double used_radius = calc_optimal_box(t, element, min_radius, max_radius);
    element->radius = (float)used_radius;

#ifdef IVP_HOME_BUILD
    /* IVP_SUPREME_SUPPORT (ivp_clustering_longrange.cxx:755) */
    ivp_clustering_visualizer_longrange_add_object(&ivp_global_clustering_visualizer.longrange, element->real_object,
                                                   &element->center, (ivp_real_t)used_radius);
#endif

    ivp_ov_node_t *new_node = ov_hash_find(t, &t->search_node);
    if (new_node) {
        ivp_pvec_add(&new_node->elements, element);
        element->node = new_node;
    } else {
        new_node = ov_node_create();
        new_node->data = t->search_node.data;
        ivp_pvec_add(&new_node->elements, element);
        element->node = new_node;
        if (!t->root) {
            t->root = new_node;
            ov_hash_add(t, new_node);
            return used_radius;
        }
        for (;;) {
            if (t->root->data.rasterlevel >= new_node->data.rasterlevel) {
                if (!box_contains_box(&t->root->data, new_node,
                                      t->root->data.rasterlevel - new_node->data.rasterlevel)) {
                    expand_tree(t, new_node);
                    continue;
                }
            } else {
                expand_tree(t, new_node);
                continue;
            }
            break;
        }
        if (t->root->data.rasterlevel == new_node->data.rasterlevel) {
            ivp_pvec_add(&t->root->elements, element);
            element->node = t->root;
            ivp_pvec_clear(&new_node->children);
            ivp_pvec_clear(&new_node->elements);
            free(new_node);
            new_node = t->root;
        } else {
            ov_hash_add(t, new_node);
            ivp_ov_node_t *smallest_node = find_smallest_box(t->root, new_node);
            connect_boxes(t, smallest_node, new_node);
        }
    }
    t->collision_partners = colliding_balls;
    if (t->collision_partners) {
        collect_collision_partners(t, element, t->root, new_node);
    }
    return used_radius;
}

static ivp_ov_node_t *cleanup_node(ivp_ov_tree_t *t, ivp_ov_node_t *node)
{
    if (ivp_pvec_len(&node->elements) != 0) return NULL;
    if (ivp_pvec_len(&node->children) != 0) return NULL;
    if (node->parent == NULL) t->root = NULL;
    ov_hash_remove(t, node);
    ivp_ov_node_t *parent = node->parent;
    ov_node_destroy(node);
    return parent;
}

void ivp_ov_tree_remove_ov_element(ivp_ov_tree_t *t, ivp_ov_element_t *element)
{
    ivp_ov_node_t *node = element->node;
    if (!node) return;
#ifdef IVP_HOME_BUILD
    /* IVP_SUPREME_SUPPORT (ivp_clustering_longrange.cxx:890) */
    ivp_clustering_visualizer_longrange_remove_object(&ivp_global_clustering_visualizer.longrange, element->real_object);
#endif
    element->node = NULL;
    ivp_pvec_remove(&node->elements, element);
    while ((node = cleanup_node(t, node)) != NULL) { }
}

/* IVP_OV_Tree_Manager::get_luf_coordinates_ws: (IVP_FLOAT)(power2(r) * x) */
void ivp_ov_tree_get_luf_coordinates_ws(const ivp_ov_tree_t *t, const ivp_ov_node_t *node,
                                        float p[3], float *cubesize)
{
    p[0] = (float)(ov_power2(t, node->data.rasterlevel) * node->data.x);
    p[1] = (float)(ov_power2(t, node->data.rasterlevel) * node->data.y);
    p[2] = (float)(ov_power2(t, node->data.rasterlevel) * node->data.z);
    *cubesize = (float)ov_power2(t, node->data.sizelevel);
}

/* IVP_OV_Tree_Manager::get_center_coordinates_ws: (IVP_FLOAT)(power2(r) * (x + 1)) */
void ivp_ov_tree_get_center_coordinates_ws(const ivp_ov_tree_t *t, const ivp_ov_node_t *node,
                                           float p[3], float *cubesize)
{
    p[0] = (float)(ov_power2(t, node->data.rasterlevel) * (node->data.x + 1));
    p[1] = (float)(ov_power2(t, node->data.rasterlevel) * (node->data.y + 1));
    p[2] = (float)(ov_power2(t, node->data.rasterlevel) * (node->data.z + 1));
    *cubesize = (float)ov_power2(t, node->data.sizelevel);
}
