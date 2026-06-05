#include "sensor_battery.h"
#include <math.h>
#include <string.h>

void battery_init(battery_model_t *m, float full_v, float capacity_mah)
{
    memset(m, 0, sizeof(*m));
    m->full_voltage_v         = full_v;
    m->empty_voltage_v        = full_v * (9.9f / 12.6f); /* scale for cells */
    m->current_voltage_v      = full_v;
    m->capacity_mah           = capacity_mah;
    m->internal_resistance_ohm = 0.05f;
    m->current_draw_a         = 5.0f;   /* idle draw */
}

void battery_set_load(battery_model_t *m, float current_a)
{
    m->current_draw_a = current_a;
}

void battery_tick(battery_model_t *m, float dt_s,
                  float *out_voltage, float *out_current, uint8_t *out_pct)
{
    /* Discharge: mAh used this tick */
    float mah_this_tick = m->current_draw_a * (dt_s / 3.6f);
    m->used_mah += mah_this_tick;
    if (m->used_mah > m->capacity_mah) m->used_mah = m->capacity_mah;

    float soc = 1.0f - (m->used_mah / m->capacity_mah);   /* 1.0 → 0.0 */

    /* Nonlinear discharge curve: flat middle, steep at ends */
    float ocv = m->empty_voltage_v
              + (m->full_voltage_v - m->empty_voltage_v)
              * (0.1f + 0.8f * soc + 0.1f * soc * soc);

    /* Internal resistance voltage sag under load */
    float v_under_load = ocv - (m->current_draw_a * m->internal_resistance_ohm);

    /* Apply forced sag fault */
    if (m->sag_fault) {
        if (v_under_load > m->sag_target_v)
            v_under_load -= (v_under_load - m->sag_target_v) * 0.05f;
        if (v_under_load < m->sag_target_v)
            v_under_load = m->sag_target_v;
    }

    m->current_voltage_v = v_under_load;

    *out_voltage = v_under_load;
    *out_current = m->current_draw_a;
    *out_pct     = (uint8_t)(soc * 100.0f + 0.5f);
    if (*out_pct > 100) *out_pct = 100;
}

void battery_set_sag(battery_model_t *m, bool active, float target_v)
{
    m->sag_fault   = active;
    m->sag_target_v = target_v;
}
