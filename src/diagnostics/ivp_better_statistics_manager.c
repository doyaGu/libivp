/* ivp_better_statistics_manager.c — port of ivp_physics/ivp_betterstatisticsmanager.cxx */
#include <ivp/diagnostics/ivp_better_statistics_manager.h>
#include <ivp/utility/ivp_string.h>

#include <string.h>

/* ── IVP_BetterStatisticsmanager_Data_Entity ───────────────────────────── */

void ivp_better_statistics_data_entity_set_int_value(ivp_better_statistics_data_entity_t *e, int value)
{
    if (!e || e->type != IVP_BSM_INT_VALUE) return;
    e->data.int_value = value;
}

void ivp_better_statistics_data_entity_set_double_value(ivp_better_statistics_data_entity_t *e, ivp_real_t value)
{
    if (!e || e->type != IVP_BSM_DOUBLE_VALUE) return;
    e->data.double_value = value;
}

void ivp_better_statistics_data_entity_set_array_size(ivp_better_statistics_data_entity_t *e, int array_size)
{
    if (!e) return;
    if (array_size < 0) array_size = 0;
    switch (e->type) {
    case IVP_BSM_INT_ARRAY:
        if (e->data.int_array.array) {
            ivp_free(&e->allocator, e->data.int_array.array,
                     ((size_t)e->data.int_array.size + 1) * sizeof(int));
        }
        e->data.int_array.array = (int *)ivp_alloc(
            &e->allocator, ((size_t)array_size + 1) * sizeof(int));
        if (e->data.int_array.array) {
            memset(e->data.int_array.array, 0, ((size_t)array_size + 1) * sizeof(int));
        }
        e->data.int_array.size = e->data.int_array.array ? array_size : 0;
        break;
    case IVP_BSM_DOUBLE_ARRAY:
        if (e->data.double_array.array) {
            ivp_free(&e->allocator, e->data.double_array.array,
                     ((size_t)e->data.double_array.size + 1) * sizeof(ivp_real_t));
        }
        e->data.double_array.array = (ivp_real_t *)ivp_alloc(
            &e->allocator, ((size_t)array_size + 1) * sizeof(ivp_real_t));
        if (e->data.double_array.array) {
            memset(e->data.double_array.array, 0,
                   ((size_t)array_size + 1) * sizeof(ivp_real_t));
        }
        e->data.double_array.size = e->data.double_array.array ? array_size : 0;
        break;
    default:
        break;
    }
}

void ivp_better_statistics_data_entity_set_int_array_latest_value(ivp_better_statistics_data_entity_t *e, int value)
{
    if (!e || e->type != IVP_BSM_INT_ARRAY) return;
    if (!e->data.int_array.array || e->data.int_array.size <= 0) return;
    int x;
    for (x = 0; x < e->data.int_array.size - 1; x++) {
        e->data.int_array.array[x] = e->data.int_array.array[x + 1];
    }
    e->data.int_array.array[x] = value;
}

void ivp_better_statistics_data_entity_set_double_array_latest_value(ivp_better_statistics_data_entity_t *e,
                                                                     ivp_real_t value)
{
    if (!e || e->type != IVP_BSM_DOUBLE_ARRAY) return;
    if (!e->data.double_array.array || e->data.double_array.size <= 0) return;
    int x;
    for (x = 0; x < e->data.double_array.size - 1; x++) {
        e->data.double_array.array[x] = e->data.double_array.array[x + 1];
    }
    e->data.double_array.array[x] = value;
}

void ivp_better_statistics_data_entity_enable(ivp_better_statistics_data_entity_t *e)
{
    if (e) e->enabled = true;
}

void ivp_better_statistics_data_entity_disable(ivp_better_statistics_data_entity_t *e)
{
    if (e) e->enabled = false;
}

bool ivp_better_statistics_data_entity_get_state(const ivp_better_statistics_data_entity_t *e)
{
    return e ? e->enabled : false;
}

void ivp_better_statistics_data_entity_set_text(ivp_better_statistics_data_entity_t *e, const char *text_in)
{
    if (!e) return;
    char *new_text = ivp_string_dup(&e->allocator, text_in);
    if (!new_text) return;
    ivp_string_free(&e->allocator, e->text);
    e->text = new_text;
}

void ivp_better_statistics_data_entity_set_position(ivp_better_statistics_data_entity_t *e, int x, int y)
{
    if (!e) return;
    e->xpos = x;
    e->ypos = y;
}

void ivp_better_statistics_data_entity_init_with_allocator(
    ivp_better_statistics_data_entity_t *e,
    ivp_better_statistics_data_entity_type_t type_in,
    const ivp_allocator_t *allocator)
{
    if (!e) return;
    memset(e, 0, sizeof(*e));
    e->allocator = *(allocator ? allocator : ivp_default_allocator());
    e->enabled = true;
    e->type = type_in;
    e->text = ivp_string_dup(&e->allocator, "Value: ");
    e->text_color = 0;
    e->xpos = 0;
    e->ypos = 0;

    switch (e->type) {
    case IVP_BSM_INT_VALUE:
        e->data.int_value = 0;
        break;
    case IVP_BSM_DOUBLE_VALUE:
        e->data.double_value = IVP_REAL(0.0);
        break;
    case IVP_BSM_INT_ARRAY:
        e->data.int_array.array = (int *)ivp_alloc(&e->allocator, 11 * sizeof(int));
        if (e->data.int_array.array) memset(e->data.int_array.array, 0, 11 * sizeof(int));
        e->data.int_array.size = e->data.int_array.array ? 10 : 0;
        e->data.int_array.max_value = 10;
        e->data.int_array.width = 10;
        e->data.int_array.height = 10;
        e->data.int_array.xpos = 0;
        e->data.int_array.ypos = 0;
        e->data.int_array.bg_color = 0;
        e->data.int_array.border_color = 1;
        e->data.int_array.graph_color = 2;
        break;
    case IVP_BSM_DOUBLE_ARRAY:
        e->data.double_array.array =
            (ivp_real_t *)ivp_alloc(&e->allocator, 11 * sizeof(ivp_real_t));
        if (e->data.double_array.array) {
            memset(e->data.double_array.array, 0, 11 * sizeof(ivp_real_t));
        }
        e->data.double_array.size = e->data.double_array.array ? 10 : 0;
        e->data.double_array.max_value = IVP_REAL(10.0);
        e->data.double_array.width = 10;
        e->data.double_array.height = 10;
        e->data.double_array.xpos = 0;
        e->data.double_array.ypos = 0;
        e->data.double_array.bg_color = 0;
        e->data.double_array.border_color = 1;
        e->data.double_array.graph_color = 2;
        break;
    case IVP_BSM_STRING:
        break;
    }
}

void ivp_better_statistics_data_entity_init(ivp_better_statistics_data_entity_t *e,
                                            ivp_better_statistics_data_entity_type_t type_in)
{
    ivp_better_statistics_data_entity_init_with_allocator(e, type_in, NULL);
}

void ivp_better_statistics_data_entity_destroy(ivp_better_statistics_data_entity_t *e)
{
    if (!e) return;
    switch (e->type) {
    case IVP_BSM_INT_ARRAY:
        if (e->data.int_array.array) {
            ivp_free(&e->allocator, e->data.int_array.array,
                     ((size_t)e->data.int_array.size + 1) * sizeof(int));
        }
        e->data.int_array.array = NULL;
        e->data.int_array.size = 0;
        break;
    case IVP_BSM_DOUBLE_ARRAY:
        if (e->data.double_array.array) {
            ivp_free(&e->allocator, e->data.double_array.array,
                     ((size_t)e->data.double_array.size + 1) * sizeof(ivp_real_t));
        }
        e->data.double_array.array = NULL;
        e->data.double_array.size = 0;
        break;
    default:
        break;
    }
    ivp_string_free(&e->allocator, e->text);
    e->text = NULL;
}

/* ── IVP_BetterStatisticsmanager ───────────────────────────────────────── */

void ivp_better_statistics_manager_print(ivp_better_statistics_manager_t *m)
{
    if (!m) return;
    if (!m->enabled) return;
    if (m->update_delayed) return;

    for (int i = 0; i < ivp_pvec_len(&m->output_callbacks); i++) {
        ivp_better_statistics_callback_t *callback =
            (ivp_better_statistics_callback_t *)ivp_pvec_at(&m->output_callbacks, i);
        for (int k = 0; k < ivp_pvec_len(&m->data_entities); k++) {
            ivp_better_statistics_data_entity_t *entity =
                (ivp_better_statistics_data_entity_t *)ivp_pvec_at(&m->data_entities, k);
            if (ivp_better_statistics_data_entity_get_state(entity)) {
                callback->vt->output_request(callback, entity);
            }
        }
    }
}

void ivp_better_statistics_manager_set_simulation_time(ivp_better_statistics_manager_t *m, ivp_real_t time)
{
    if (!m) return;
    m->simulation_time = time;

    if ((m->simulation_time - m->time_of_last_update) < m->update_interval) {
        m->update_delayed = true;
        return;
    }
    m->time_of_last_update = m->simulation_time;
    m->update_delayed = false;
}

ivp_real_t ivp_better_statistics_manager_get_simulation_time(const ivp_better_statistics_manager_t *m)
{
    return m ? m->simulation_time : IVP_REAL(0.0);
}

void ivp_better_statistics_manager_install_data_entity(ivp_better_statistics_manager_t *m,
                                                       ivp_better_statistics_data_entity_t *entity)
{
    if (!m || !entity) return;
    ivp_pvec_add(&m->data_entities, entity);
}

void ivp_better_statistics_manager_remove_data_entity(ivp_better_statistics_manager_t *m,
                                                      ivp_better_statistics_data_entity_t *entity)
{
    if (!m) return;
    (void)ivp_pvec_remove_if_present(&m->data_entities, entity);
}

void ivp_better_statistics_manager_install_output_callback(ivp_better_statistics_manager_t *m,
                                                           ivp_better_statistics_callback_t *callback)
{
    if (!m || !callback) return;
    ivp_pvec_add(&m->output_callbacks, callback);
}

void ivp_better_statistics_manager_remove_output_callback(ivp_better_statistics_manager_t *m,
                                                          ivp_better_statistics_callback_t *callback)
{
    if (!m) return;
    (void)ivp_pvec_remove_if_present(&m->output_callbacks, callback);
}

void ivp_better_statistics_manager_enable(ivp_better_statistics_manager_t *m)
{
    if (m) m->enabled = true;
}

void ivp_better_statistics_manager_disable(ivp_better_statistics_manager_t *m)
{
    if (m) m->enabled = false;
}

bool ivp_better_statistics_manager_get_state(const ivp_better_statistics_manager_t *m)
{
    return m ? m->enabled : false;
}

void ivp_better_statistics_manager_init_with_allocator(
    ivp_better_statistics_manager_t *m, const ivp_allocator_t *allocator)
{
    if (!m) return;
    memset(m, 0, sizeof(*m));
    m->allocator = *(allocator ? allocator : ivp_default_allocator());
    m->enabled = true;
    ivp_pvec_init_with_allocator(&m->output_callbacks, &m->allocator);
    ivp_pvec_init_with_allocator(&m->data_entities, &m->allocator);
    m->update_interval = IVP_REAL(1.0);
    m->update_delayed = true;
    m->simulation_time = IVP_REAL(0.0);
    m->time_of_last_update = IVP_REAL(0.0);
}

void ivp_better_statistics_manager_init(ivp_better_statistics_manager_t *m)
{
    ivp_better_statistics_manager_init_with_allocator(m, NULL);
}

void ivp_better_statistics_manager_destroy(ivp_better_statistics_manager_t *m)
{
    if (!m) return;
    ivp_pvec_clear(&m->output_callbacks);
    ivp_pvec_clear(&m->data_entities);
}
