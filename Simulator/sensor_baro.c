#include "sensor_baro.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static float randn_baro(float stddev)
{
    float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159265f * u2) * stddev;
}

void baro_init(baro_model_t *m)
{
    memset(m, 0, sizeof(*m));
    m->base_pressure_hpa = 1013.25f;
    m->base_temp_c       = 15.0f;
    m->noise_hpa         = 0.1f;
    m->noise_temp_c      = 0.05f;
    m->true_altitude_m   = 0.0f;
}

void baro_set_altitude(baro_model_t *m, float alt_m)
{
    m->true_altitude_m = alt_m;
}

void baro_tick(baro_model_t *m, float *out_pressure_hpa, float *out_temp_c)
{
    /*
     * International Standard Atmosphere (troposphere, 0–11 km):
     *   T(h) = T0 - L*h         (L = 0.0065 K/m)
     *   P(h) = P0 * (T(h)/T0)^(g/(R*L))
     *   exponent = 9.80665 / (287.05 * 0.0065) = 5.2561
     */
    float h  = m->true_altitude_m;
    float T0 = m->base_temp_c + 273.15f;   /* Kelvin */
    float L  = 0.0065f;                     /* K/m lapse rate */
    float Th = T0 - L * h;
    float p  = m->base_pressure_hpa * powf(Th / T0, 5.2561f);
    float t  = Th - 273.15f;               /* back to Celsius */

    *out_pressure_hpa = p + randn_baro(m->noise_hpa);
    *out_temp_c       = t + randn_baro(m->noise_temp_c);
}
