#include "sensor_gps.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

static float randf_range(float lo, float hi)
{
    return lo + (hi - lo) * ((float)rand() / (float)RAND_MAX);
}

static float randn_gps(float stddev)
{
    float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159265f * u2) * stddev;
}

void gps_init(gps_model_t *m, double lat0, double lon0)
{
    memset(m, 0, sizeof(*m));
    m->true_lat         = lat0;
    m->true_lon         = lon0;
    m->true_alt_m       = 0.0f;
    m->true_speed_mps   = 0.0f;
    m->fix_quality      = 1;
    m->satellites       = 8;
    m->position_noise_m = 0.5f;   /* ±0.5 m  */
    m->speed_noise_mps  = 0.05f;
}

void gps_set_position(gps_model_t *m, double lat, double lon,
                      float alt_m, float speed_mps)
{
    m->true_lat       = lat;
    m->true_lon       = lon;
    m->true_alt_m     = alt_m;
    m->true_speed_mps = speed_mps;
}

void gps_tick(gps_model_t *m,
              double *out_lat, double *out_lon,
              float  *out_alt, float  *out_speed,
              uint8_t *out_fix, uint8_t *out_sats)
{
    if (m->signal_lost) {
        *out_fix  = 0;
        *out_sats = 0;
        *out_lat  = 0.0;
        *out_lon  = 0.0;
        *out_alt  = 0.0f;
        *out_speed = 0.0f;
        return;
    }

    /* 1° lat ≈ 111 km, 1° lon ≈ 111 km × cos(lat) */
    double meters_per_deg_lat = 111000.0;
    double meters_per_deg_lon = 111000.0 * cos(m->true_lat * 3.14159265 / 180.0);

    *out_lat   = m->true_lat + randn_gps(m->position_noise_m) / meters_per_deg_lat;
    *out_lon   = m->true_lon + randn_gps(m->position_noise_m) / meters_per_deg_lon;
    *out_alt   = m->true_alt_m   + randn_gps(m->position_noise_m * 1.5f);
    *out_speed = m->true_speed_mps + randn_gps(m->speed_noise_mps);
    if (*out_speed < 0.0f) *out_speed = 0.0f;

    /* Satellites vary slightly */
    int sat_jitter = (rand() % 3) - 1;
    int sats = (int)m->satellites + sat_jitter;
    if (sats < 4) sats = 4;
    if (sats > 12) sats = 12;
    *out_sats = (uint8_t)sats;
    *out_fix  = m->fix_quality;
}

void gps_set_signal_lost(gps_model_t *m, bool lost)
{
    m->signal_lost = lost;
    if (lost) {
        m->fix_quality = 0;
        m->satellites  = 0;
    } else {
        m->fix_quality = 1;
        m->satellites  = 8;
    }
}
