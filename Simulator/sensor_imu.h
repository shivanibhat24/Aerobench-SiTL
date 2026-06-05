#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include "sim_types.h"

typedef struct {
    /* True physical state (set by scenario engine) */
    float true_accel_x, true_accel_y, true_accel_z; /* m/s² */
    float true_gyro_x,  true_gyro_y,  true_gyro_z;  /* deg/s */

    /* Noise parameters */
    float accel_noise_stddev;   /* m/s²  typical: 0.05 */
    float gyro_noise_stddev;    /* deg/s typical: 0.02 */

    /* Gyro drift (slow random walk) */
    float gyro_drift_x, gyro_drift_y, gyro_drift_z;
    float drift_rate;           /* deg/s per second drift rate */

    /* Fault state */
    bool  frozen;               /* sensor outputs locked at last value */
    float frozen_ax, frozen_ay, frozen_az;
    float frozen_gx, frozen_gy, frozen_gz;

    /* Gravity offset — 1g on Z when level */
    float gravity_z;            /* m/s²: ~9.81 */
} imu_model_t;

void imu_init(imu_model_t *m);
void imu_set_motion(imu_model_t *m,
                    float ax, float ay, float az,
                    float gx, float gy, float gz);
void imu_tick(imu_model_t *m, float dt_s,
              float *out_ax, float *out_ay, float *out_az,
              float *out_gx, float *out_gy, float *out_gz);
void imu_freeze(imu_model_t *m, bool freeze);

#endif /* SENSOR_IMU_H */
