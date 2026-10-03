/* ivp_sim_unit.h — simulation units (port of IVP_Simulation_Unit)
 *
 * A simulation unit groups the cores that are connected through controllers
 * (friction systems, constraints, springs ...).  Each unit runs its own
 * controller list during a PSI and freezes/revives as a whole.  The full
 * definition is internal (src/dynamics/ivp_sim_internal.h).
 */
#ifndef IVP_SIM_UNIT_H
#define IVP_SIM_UNIT_H

#ifdef __cplusplus
extern "C" {
#endif

struct ivp_sim_unit;
typedef struct ivp_sim_unit ivp_sim_unit_t;
struct ivp_sim_units_manager;
typedef struct ivp_sim_units_manager ivp_sim_units_manager_t;

#ifdef __cplusplus
}
#endif

#endif /* IVP_SIM_UNIT_H */
