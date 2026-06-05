#include "sensor_imu.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Box-Muller transform: returns a normally-distributed random float */
static float randn(float stddev)
{
    if (stddev == 0.0f) return 0.0f;
    float u1 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    float u2 = ((float)rand() + 1.0f) / ((float)RAND_MAX + 1.0f);
    float z  = sqrtf(-2.0f * logf(u1)) * cosf(2.0f * 3.14159265f * u2);
    return z * stddev;
}

void imu_init(imu_model_t *m)
{
    memset(m, 0, sizeof(*m));
    m->accel_noise_stddev = 0.05f;
    m->gyro_noise_stddev  = 0.02f;
    m->drift_rate         = 0.001f;
    m->gravity_z          = 9.81f;
    /* Default: hover level — gravity on Z */
    m->true_accel_z       = 9.81f;
}

void imu_set_motion(imu_model_t *m,
                    float ax, float ay, float az,
                    float gx, float gy, float gz)
{
    m->true_accel_x = ax;
    m->true_accel_y = ay;
    m->true_accel_z = az;
    m->true_gyro_x  = gx;
    m->true_gyro_y  = gy;
    m->true_gyro_z  = gz;
}

void imu_tick(imu_model_t *m, float dt_s,
              float *out_ax, float *out_ay, float *out_az,
              float *out_gx, float *out_gy, float *out_gz)
{
    if (m->frozen) {
        *out_ax = m->frozen_ax; *out_ay = m->frozen_ay; *out_az = m->frozen_az;
        *out_gx = m->frozen_gx; *out_gy = m->frozen_gy; *out_gz = m->frozen_gz;
        return;
    }

    /* Accumulate gyro drift random walk */
    m->gyro_drift_x += randn(m->drift_rate * dt_s);
    m->gyro_drift_y += randn(m->drift_rate * dt_s);
    m->gyro_drift_z += randn(m->drift_rate * dt_s);

    *out_ax = m->true_accel_x + randn(m->accel_noise_stddev);
    *out_ay = m->true_accel_y + randn(m->accel_noise_stddev);
    *out_az = m->true_accel_z + randn(m->accel_noise_stddev);

    *out_gx = m->true_gyro_x + m->gyro_drift_x + randn(m->gyro_noise_stddev);
    *out_gy = m->true_gyro_y + m->gyro_drift_y + randn(m->gyro_noise_stddev);
    *out_gz = m->true_gyro_z + m->gyro_drift_z + randn(m->gyro_noise_stddev);
}

void imu_freeze(imu_model_t *m, bool freeze)
{
    if (freeze && !m->frozen) {
        /* Latch current output values */
        m->frozen_ax = m->true_accel_x;
        m->frozen_ay = m->true_accel_y;
        m->frozen_az = m->true_accel_z;
        m->frozen_gx = m->true_gyro_x;
        m->frozen_gy = m->true_gyro_y;
        m->frozen_gz = m->true_gyro_z;
    }
    m->frozen = freeze;
}
