/* ivp_debug_manager.h — IVP_BetterDebugmanager and IVP_Debug_Manager
 *
 * IVP_BetterDebugmanager (ivp_physics/ivp_betterdebugmanager.{hxx,cxx}):
 * per class debug output switches and a printf style dprint whose output
 * goes through an overridable output function.  The reference keeps one
 * global instance, ivp_debugmanager, used through IVP_IFDEBUG(class) by the
 * builders (pointsoup, ledge soup, convex decompositor, geompack).
 *
 * IVP_Debug_Manager (ivp_physics/ivp_debug_manager.hxx, implemented in
 * ivp_environment.cxx): the per environment debug switches
 * (ivp_environment_get_debug_manager).  In debug builds the environment
 * counts PSIs in psi_counter (IVP_IF(1), like the reference); the other
 * switches are read by the reference only in debug builds (IVP_IF) for file
 * dumps (out_deb_file) and checks.  The reference creates the manager only in
 * debug builds; libivp always creates it (nothing reads it in release).
 */
#ifndef IVP_DEBUG_MANAGER_H
#define IVP_DEBUG_MANAGER_H

#include <ivp/base/ivp_types.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── IVP_BetterDebugmanager ────────────────────────────────────────────── */

/* IVP_DEBUG_CLASS.  The values from 0-1023 are reserved for internal Ipion
 * use; define your own debug identifiers using values from 1024-2047. */
typedef enum ivp_debug_class {
    IVP_DM_DUMMY,
    IVP_DM_SURBUILD_POINTSOUP,
    IVP_DM_SURBUILD_HALFSPACESOUP,
    IVP_DM_SURBUILD_LEDGESOUP,
    IVP_DM_SURBUILD_Q12,
    IVP_DM_CONVEX_DECOMPOSITOR,
    IVP_DM_QHULL,
    IVP_DM_GEOMPACK_LEVEL1,
    IVP_DM_GEOMPACK_LEVEL2,
    IVP_DM_GEOMPACK_LEVEL3,
    IVP_DM_CLUSTERING_SHORTRANGE_VISUALIZER,

    IVP_DEBUG_IPION_ERROR_MSG = 1024,

    IVP_DEBUG_MAX_N_CLASSES = 2048 /* DO NOT CHANGE THIS VALUE! */
} ivp_debug_class_t;

typedef struct ivp_better_debug_manager ivp_better_debug_manager_t;

/* IVP_BetterDebugmanager::output_function (virtual): receives the formatted
 * dprint text. */
typedef void (*ivp_better_debug_output_fn_t)(ivp_better_debug_manager_t *dm, ivp_debug_class_t class_id,
                                             const char *string);

struct ivp_better_debug_manager {
    int flag_list[IVP_DEBUG_MAX_N_CLASSES];
    int initialized;
    /* NULL: the reference output_function, printf("%s", string) */
    ivp_better_debug_output_fn_t output_function;
    void *output_ctx; /* user data for output_function */
};

/* Constructor: all classes off.  Debug builds (IVP_ENABLE_ASSERTS, the
 * reference "DEBUG") enable IVP_DM_CLUSTERING_SHORTRANGE_VISUALIZER and
 * IVP_DEBUG_IPION_ERROR_MSG. */
void ivp_better_debug_manager_init(ivp_better_debug_manager_t *dm);

/* Out of range classes are ignored. */
void ivp_better_debug_manager_enable_debug_output(ivp_better_debug_manager_t *dm, ivp_debug_class_t class_id);
void ivp_better_debug_manager_disable_debug_output(ivp_better_debug_manager_t *dm, ivp_debug_class_t class_id);
bool ivp_better_debug_manager_is_debug_enabled(const ivp_better_debug_manager_t *dm, ivp_debug_class_t class_id);

/* vsnprintf into a 4096 byte buffer, then output_function(class_id, buffer). */
void ivp_better_debug_manager_dprint(ivp_better_debug_manager_t *dm, ivp_debug_class_t class_id,
                                     const char *formatstring, ...)
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((format(printf, 3, 4)))
#endif
    ;

/* The default output function: printf("%s", string). */
void ivp_better_debug_manager_default_output(ivp_better_debug_manager_t *dm, ivp_debug_class_t class_id,
                                             const char *string);

/* The global instance ("use global variable only in emergency"), statically
 * initialised like the reference constructor. */
extern ivp_better_debug_manager_t ivp_debugmanager;

#define IVP_IFDEBUG(dci) if (ivp_better_debug_manager_is_debug_enabled(&ivp_debugmanager, (dci)))

/* ── ivp_debug.hxx ─────────────────────────────────────────────────────── */

/* Names (ivp_object_t name) of the two objects whose mindist / core events
 * the debug build traces (ivp_check_debug_mindist, physic_private), and the
 * simulation time from which on.  The reference ships them disabled (0);
 * edit these defines (like the reference's "#if 0" block) to trace a pair. */
#define IVP_DEBUG_OBJECT0 0
#define IVP_DEBUG_OBJECT1 0
#define IVP_DEBUG_TIME 0

/* ── IVP_Debug_Manager ─────────────────────────────────────────────────── */

typedef struct ivp_debug_manager {
    bool psi_synchrone;          /* display every psi */
    bool disable_mindists;
    bool disable_friction;
    bool debug_impact;
    bool debug_mindist;
    bool debug_friction;
    bool disable_core_dampening;
    bool display_statistic;
    bool display_debug_vectors;

    ivp_real_t all_time_impacts; /* number of impacts so far      */
    ivp_real_t psi_counter;      /* number of PSI processed so far */

    float last_impact_translation_speed_normal;
    float last_impact_rot_speed_normal;
    float last_impact_rot_speed_whole;

    FILE *out_deb_file;          /* closed by clear (unless stdout) */
    int   file_nr;               /* revolve to next filename (10 pieces) */
    bool  file_out_impacts;
    bool  debug_imp_sys;
    bool  revolve_deb_file;

    bool  check_fs;
    int   arbitrary_counter;
    int   arbitrary_flag;
} ivp_debug_manager_t;

/* IVP_Debug_Manager::init_debug_manager (also the constructor): all switches
 * off, counters 0, out_deb_file NULL. */
void ivp_debug_manager_init(ivp_debug_manager_t *dm);
/* IVP_Debug_Manager::clear_debug_manager (also the destructor): closes
 * out_deb_file unless it is NULL or stdout. */
void ivp_debug_manager_clear(ivp_debug_manager_t *dm);

#ifdef __cplusplus
}
#endif

#endif /* IVP_DEBUG_MANAGER_H */
