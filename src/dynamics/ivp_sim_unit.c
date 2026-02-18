/* ivp_sim_unit.c -- Simulation unit implementation */

#include <ivp/dynamics/ivp_object.h>
#include <ivp/dynamics/ivp_sim_unit.h>

int ivp_object_get_sim_unit_id(const struct ivp_object *obj)
{
    if (!obj) return IVP_SIM_UNIT_DEFAULT;
    return obj->sim_unit_id;
}

ivp_result_t ivp_object_set_sim_unit(struct ivp_object *obj, int unit_id)
{
    if (!obj) return IVP_ERR_INVALID_ARG;
    if (unit_id < 0 || unit_id > IVP_SIM_UNIT_MAX) return IVP_ERR_INVALID_ARG;

    obj->sim_unit_id = unit_id;
    return IVP_OK;
}
