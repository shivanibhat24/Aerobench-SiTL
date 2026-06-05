#ifndef SENSOR_GPS_H
#define SENSOR_GPS_H

#include "sim_types.h"

typedef struct {
    double  true_lat, true_lon;
    float   true_alt_m;
    float   true_speed_mps;
    uint8_t fix_quality;       /* 0=none, 1=GPS, 2=DGPS */
    uint8_t satellites;

    float   position_noise_m;  /* meters of position jitter */
    float   speed_noise_mps;

    bool    signal_lost;       /* fault: no fix             */
} gps_model_t;

void gps_init(gps_model_t *m, double lat0, double lon0);
void gps_set_position(gps_model_t *m, double lat, double lon,
                      float alt_m, float speed_mps);
void gps_tick(gps_model_t *m,
              double *out_lat, double *out_lon,
              float  *out_alt, float  *out_speed,
              uint8_t *out_fix, uint8_t *out_sats);
void gps_set_signal_lost(gps_model_t *m, bool lost);

#endif /* SENSOR_GPS_H */
