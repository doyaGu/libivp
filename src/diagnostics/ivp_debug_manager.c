/* ivp_debug_manager.c — port of ivp_physics/ivp_betterdebugmanager.cxx and
 * the IVP_Debug_Manager methods of ivp_physics/ivp_environment.cxx
 */
#include <ivp/diagnostics/ivp_debug_manager.h>
#include <ivp/base/ivp_message.h>

#include <stdarg.h>
#include <string.h>

/* ── IVP_BetterDebugmanager ────────────────────────────────────────────── */

void ivp_better_debug_manager_enable_debug_output(ivp_better_debug_manager_t *dm, ivp_debug_class_t class_id)
{
    if (!dm) return;
    if ((int)class_id < (int)IVP_DM_DUMMY || (int)class_id >= (int)IVP_DEBUG_MAX_N_CLASSES) return;
    dm->flag_list[class_id] = 1;
}

void ivp_better_debug_manager_disable_debug_output(ivp_better_debug_manager_t *dm, ivp_debug_class_t class_id)
{
    if (!dm) return;
    if ((int)class_id < (int)IVP_DM_DUMMY || (int)class_id >= (int)IVP_DEBUG_MAX_N_CLASSES) return;
    dm->flag_list[class_id] = 0;
}

bool ivp_better_debug_manager_is_debug_enabled(const ivp_better_debug_manager_t *dm, ivp_debug_class_t class_id)
{
    if (!dm) return false;
    if ((int)class_id >= (int)IVP_DEBUG_MAX_N_CLASSES) return false;
    if ((int)class_id < (int)IVP_DM_DUMMY || dm->flag_list[class_id] == 0 || !dm->initialized) return false;
    return true;
}

void ivp_better_debug_manager_default_output(ivp_better_debug_manager_t *dm, ivp_debug_class_t class_id,
                                             const char *string)
{
    (void)dm;
    (void)class_id;
    ivp_debug_printf("%s", string);
}

void ivp_better_debug_manager_dprint(ivp_better_debug_manager_t *dm, ivp_debug_class_t class_id,
                                     const char *formatstring, ...)
{
    if (!dm || !formatstring) return;
    char buffer[4096];
    va_list args;

    va_start(args, formatstring);
    vsnprintf(buffer, sizeof(buffer), formatstring, args);
    va_end(args);

    if (dm->output_function) {
        dm->output_function(dm, class_id, buffer);
    } else {
        ivp_better_debug_manager_default_output(dm, class_id, buffer);
    }
}

void ivp_better_debug_manager_init(ivp_better_debug_manager_t *dm)
{
    if (!dm) return;
    for (int i = 0; i < (int)IVP_DEBUG_MAX_N_CLASSES; i++) dm->flag_list[i] = 0;
    dm->initialized = 1;
    dm->output_function = NULL;
    dm->output_ctx = NULL;
#ifdef IVP_ENABLE_ASSERTS /* reference: #ifdef DEBUG */
    ivp_better_debug_manager_enable_debug_output(dm, IVP_DM_CLUSTERING_SHORTRANGE_VISUALIZER);
    ivp_better_debug_manager_enable_debug_output(dm, IVP_DEBUG_IPION_ERROR_MSG);
#endif
}

/* IVP_BetterDebugmanager ivp_debugmanager (global constructor) */
ivp_better_debug_manager_t ivp_debugmanager = {
    .flag_list = {
#ifdef IVP_ENABLE_ASSERTS
        [IVP_DM_CLUSTERING_SHORTRANGE_VISUALIZER] = 1,
        [IVP_DEBUG_IPION_ERROR_MSG] = 1,
#else
        0
#endif
    },
    .initialized = 1,
    .output_function = NULL,
    .output_ctx = NULL,
};

/* ── IVP_Debug_Manager ─────────────────────────────────────────────────── */

void ivp_debug_manager_clear(ivp_debug_manager_t *dm)
{
    if (!dm) return;
    if (dm->out_deb_file && dm->out_deb_file != stdout) {
        fclose(dm->out_deb_file);
    }
    dm->out_deb_file = NULL;
}

void ivp_debug_manager_init(ivp_debug_manager_t *dm)
{
    if (!dm) return;
    dm->psi_synchrone = false;
    dm->disable_mindists = false;
    dm->disable_friction = false;
    dm->debug_impact = false;
    dm->debug_mindist = false;
    dm->debug_friction = false;
    dm->disable_core_dampening = false;
    dm->display_statistic = false;
    dm->display_debug_vectors = false;
    dm->all_time_impacts = IVP_REAL(0.0);
    dm->psi_counter = IVP_REAL(0.0);
    dm->last_impact_translation_speed_normal = 0.0f;
    dm->last_impact_rot_speed_normal = 0.0f;
    dm->last_impact_rot_speed_whole = 0.0f;
    dm->out_deb_file = NULL;
    dm->file_nr = 0;
    dm->file_out_impacts = false;
    dm->debug_imp_sys = false;
    dm->revolve_deb_file = false;
    dm->check_fs = false;
    dm->arbitrary_counter = 0;
    dm->arbitrary_flag = 0;
}
