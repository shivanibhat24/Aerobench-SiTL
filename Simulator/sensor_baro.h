#ifndef SENSOR_BARO_H
#define SENSOR_BARO_H

#include "sim_types.h"

typedef struct {
    float true_altitude_m;
    float base_pressure_hpa;  /* sea-level: 1013.25 hPa */
    float base_temp_c;        /* sea-level temperature   */
    float noise_hpa;
    float noise_temp_c;
} baro_model_t;

void baro_init(baro_model_t *m);
void baro_set_altitude(baro_model_t *m, float alt_m);
void baro_tick(baro_model_t *m, float *out_pressure_hpa, float *out_temp_c);

#endif /* SENSOR_BARO_H */
