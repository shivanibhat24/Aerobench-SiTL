#ifndef SENSOR_BATTERY_H
#define SENSOR_BATTERY_H

#include "sim_types.h"

typedef struct {
    float  full_voltage_v;     /* e.g. 12.6V for 3S LiPo   */
    float  empty_voltage_v;    /* e.g.  9.9V                */
    float  current_voltage_v;  /* simulated state           */
    float  current_draw_a;     /* load current              */
    float  capacity_mah;
    float  used_mah;
    float  internal_resistance_ohm;

    bool   sag_fault;          /* forced voltage sag        */
    float  sag_target_v;
} battery_model_t;

void    battery_init(battery_model_t *m, float full_v, float capacity_mah);
void    battery_set_load(battery_model_t *m, float current_a);
void    battery_tick(battery_model_t *m, float dt_s,
                     float *out_voltage, float *out_current, uint8_t *out_pct);
void    battery_set_sag(battery_model_t *m, bool active, float target_v);

#endif /* SENSOR_BATTERY_H */
