/* ivp_better_statistics_manager.h — IVP_BetterStatisticsmanager
 *
 * Port of ivp_physics/ivp_betterstatisticsmanager.{hxx,cxx} and
 * ivp_physics/ivp_stat_manager_cback_con.{hxx,cxx}
 * (IVP_Statisticsmanager_Console_Callback).
 *
 * A list of user data entities (int / double values and arrays, strings)
 * that print() hands to every installed output callback.  Every environment
 * owns one (ivp_environment_get_better_statistics_manager); the simulate
 * functions keep its simulation time up to date (set_simulation_time), which
 * throttles print() to one output per update_interval of simulation time.
 *
 * "IVP_DOUBLE" members are ivp_real_t (double unless IVP_REAL_DOUBLE=OFF).
 */
#ifndef IVP_BETTER_STATISTICS_MANAGER_H
#define IVP_BETTER_STATISTICS_MANAGER_H

#include <ivp/base/ivp_types.h>
#include <ivp/containers/ivp_pvec.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* IVP_BETTERSTATISTICSMANAGER_DATA_ENTITY_TYPE */
typedef enum ivp_better_statistics_data_entity_type {
    IVP_BSM_INT_VALUE = 1,    /* one single 'int' value      */
    IVP_BSM_DOUBLE_VALUE = 2, /* one single 'double' value   */
    IVP_BSM_INT_ARRAY = 3,    /* an array of 'int' values    */
    IVP_BSM_DOUBLE_ARRAY = 4, /* an array of 'double' values */
    IVP_BSM_STRING = 5
} ivp_better_statistics_data_entity_type_t;

/* IVP_BetterStatisticsmanager_Data_Int_Array */
typedef struct ivp_better_statistics_data_int_array {
    int  size;
    int *array;
    int  max_value;
    int  xpos, ypos;     /* upper, left corner         */
    int  width, height;  /* graphical size of array    */
    int  bg_color, border_color, graph_color;
} ivp_better_statistics_data_int_array_t;

/* IVP_BetterStatisticsmanager_Data_Double_Array */
typedef struct ivp_better_statistics_data_double_array {
    int         size;
    ivp_real_t *array;
    ivp_real_t  max_value;
    int         xpos, ypos;
    int         width, height;
    int         bg_color, border_color, graph_color;
} ivp_better_statistics_data_double_array_t;

/* IVP_BetterStatisticsmanager_Data_Entity */
typedef struct ivp_better_statistics_data_entity {
    bool enabled; /* use enable / disable / get_state */
    ivp_better_statistics_data_entity_type_t type;
    union {
        int        int_value;
        ivp_real_t double_value;
        ivp_better_statistics_data_int_array_t    int_array;
        ivp_better_statistics_data_double_array_t double_array;
    } data;
    char *text;       /* owned (set_text copies) */
    int   text_color;
    int   xpos, ypos;
} ivp_better_statistics_data_entity_t;

/* Constructor: enabled, text "Value: ", value 0; arrays get 10 zeroed
 * values, max_value 10, width = height = 10, colors bg 0 / border 1 /
 * graph 2. */
void ivp_better_statistics_data_entity_init(ivp_better_statistics_data_entity_t *e,
                                            ivp_better_statistics_data_entity_type_t type);
/* Destructor: frees the array and the text. */
void ivp_better_statistics_data_entity_destroy(ivp_better_statistics_data_entity_t *e);

void ivp_better_statistics_data_entity_enable(ivp_better_statistics_data_entity_t *e);
void ivp_better_statistics_data_entity_disable(ivp_better_statistics_data_entity_t *e);
bool ivp_better_statistics_data_entity_get_state(const ivp_better_statistics_data_entity_t *e);

/* The setters only act on entities of the matching type (the reference
 * writes the union regardless). */
void ivp_better_statistics_data_entity_set_int_value(ivp_better_statistics_data_entity_t *e, int value);
void ivp_better_statistics_data_entity_set_double_value(ivp_better_statistics_data_entity_t *e, ivp_real_t value);
/* Changing the array size clears all values (negative sizes become 0). */
void ivp_better_statistics_data_entity_set_array_size(ivp_better_statistics_data_entity_t *e, int size);
/* Shift the array one to the front and store value as the last element. */
void ivp_better_statistics_data_entity_set_int_array_latest_value(ivp_better_statistics_data_entity_t *e, int value);
void ivp_better_statistics_data_entity_set_double_array_latest_value(ivp_better_statistics_data_entity_t *e,
                                                                     ivp_real_t value);
/* Copies text (keeps the old text when the copy fails). */
void ivp_better_statistics_data_entity_set_text(ivp_better_statistics_data_entity_t *e, const char *text);
void ivp_better_statistics_data_entity_set_position(ivp_better_statistics_data_entity_t *e, int x, int y);

/* ── IVP_BetterStatisticsmanager_Callback_Interface ────────────────────── */

typedef struct ivp_better_statistics_callback ivp_better_statistics_callback_t;

typedef struct ivp_better_statistics_callback_vtable {
    void (*output_request)(ivp_better_statistics_callback_t *cb, ivp_better_statistics_data_entity_t *entity);
    void (*enable)(ivp_better_statistics_callback_t *cb);
    void (*disable)(ivp_better_statistics_callback_t *cb);
} ivp_better_statistics_callback_vtable_t;

/* Embed as the first member of the implementation. */
struct ivp_better_statistics_callback {
    const ivp_better_statistics_callback_vtable_t *vt;
};

/* ── IVP_BetterStatisticsmanager ───────────────────────────────────────── */

typedef struct ivp_better_statistics_manager {
    bool       enabled;
    ivp_pvec_t output_callbacks;  /* ivp_better_statistics_callback_t* (not owned)    */
    ivp_pvec_t data_entities;     /* ivp_better_statistics_data_entity_t* (not owned) */
    ivp_real_t simulation_time;
    ivp_real_t time_of_last_update;

    bool       update_delayed;    /* true: print() does nothing      */
    ivp_real_t update_interval;   /* simulation seconds, default 1.0 */
} ivp_better_statistics_manager_t;

/* Constructor: enabled, update_interval 1, update_delayed, times 0. */
void ivp_better_statistics_manager_init(ivp_better_statistics_manager_t *m);
/* Destructor (entities and callbacks are not owned). */
void ivp_better_statistics_manager_destroy(ivp_better_statistics_manager_t *m);

/* Output every enabled entity to every callback (callbacks outer loop,
 * entities in install order), unless disabled or update_delayed. */
void ivp_better_statistics_manager_print(ivp_better_statistics_manager_t *m);

/* Update the simulation time: update_delayed becomes
 * (time - time_of_last_update < update_interval); when not delayed,
 * time_of_last_update = time. */
void ivp_better_statistics_manager_set_simulation_time(ivp_better_statistics_manager_t *m, ivp_real_t time);
ivp_real_t ivp_better_statistics_manager_get_simulation_time(const ivp_better_statistics_manager_t *m);

void ivp_better_statistics_manager_install_data_entity(ivp_better_statistics_manager_t *m,
                                                       ivp_better_statistics_data_entity_t *entity);
void ivp_better_statistics_manager_remove_data_entity(ivp_better_statistics_manager_t *m,
                                                      ivp_better_statistics_data_entity_t *entity);
void ivp_better_statistics_manager_install_output_callback(ivp_better_statistics_manager_t *m,
                                                           ivp_better_statistics_callback_t *callback);
void ivp_better_statistics_manager_remove_output_callback(ivp_better_statistics_manager_t *m,
                                                          ivp_better_statistics_callback_t *callback);

void ivp_better_statistics_manager_enable(ivp_better_statistics_manager_t *m);
void ivp_better_statistics_manager_disable(ivp_better_statistics_manager_t *m);
bool ivp_better_statistics_manager_get_state(const ivp_better_statistics_manager_t *m);

/* ── IVP_Statisticsmanager_Console_Callback ────────────────────────────── *
 * printf output of an entity:
 *   INT_VALUE    "<text><value %d>\n"
 *   DOUBLE_VALUE "<text><value %f>\n"
 *   INT_ARRAY    "<text>\n" then "%d\n" per element
 *   DOUBLE_ARRAY "<text>\n" then "%f\n" per element
 *   STRING       "<text>\n"
 * enable / disable do nothing. */
typedef struct ivp_statistics_console_callback {
    ivp_better_statistics_callback_t base;
    /* libivp: destination.  stdout (after init) or NULL: the engine output
     * (ivp_debug_printf, like the reference printf); any other FILE: fprintf */
    FILE *out;
} ivp_statistics_console_callback_t;

void ivp_statistics_console_callback_init(ivp_statistics_console_callback_t *cb);

#ifdef __cplusplus
}
#endif

#endif /* IVP_BETTER_STATISTICS_MANAGER_H */
