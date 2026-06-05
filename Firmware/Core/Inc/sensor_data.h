#ifndef SENSOR_DATA_H
#define SENSOR_DATA_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/*  Raw sensor frame — populated by task_sensor from inbound UART      */
/* ------------------------------------------------------------------ */

typedef struct {
    /* Accelerometer (m/s²) */
    float accel_x;
    float accel_y;
    float accel_z;

    /* Gyroscope (deg/s) */
    float gyro_x;
    float gyro_y;
    float gyro_z;

    /* GPS */
    double latitude;
    double longitude;
    float  altitude_m;
    float  ground_speed_mps;
    uint8_t gps_fix_quality;   /* 0=no fix, 1=GPS, 2=DGPS */
    uint8_t gps_satellites;

    /* Battery */
    float battery_voltage_v;
    float battery_current_a;
    uint8_t battery_pct;

    /* Environmental */
    float baro_pressure_hpa;
    float temperature_c;

    /* Metadata */
    uint32_t timestamp_ms;     /* simulator wall-clock ms   */
    uint8_t  sensor_status;    /* bitmask: see SENSOR_FLAG_* */
} sensor_data_t;

/* sensor_status bitmask */
#define SENSOR_FLAG_ACCEL_OK   (1u << 0)
#define SENSOR_FLAG_GYRO_OK    (1u << 1)
#define SENSOR_FLAG_GPS_OK     (1u << 2)
#define SENSOR_FLAG_BATT_OK    (1u << 3)
#define SENSOR_FLAG_BARO_OK    (1u << 4)
#define SENSOR_FLAG_ALL_OK     (0x1Fu)

/* ------------------------------------------------------------------ */
/*  Flight state machine states                                         */
/* ------------------------------------------------------------------ */

typedef enum {
    FLIGHT_STATE_INIT      = 0,
    FLIGHT_STATE_PREFLIGHT = 1,
    FLIGHT_STATE_ARMED     = 2,
    FLIGHT_STATE_FLYING    = 3,
    FLIGHT_STATE_LANDING   = 4,
    FLIGHT_STATE_FAULT     = 5,
    FLIGHT_STATE_COUNT
} flight_state_t;

/* Human-readable names (useful for telemetry / black-box) */
static const char * const FLIGHT_STATE_NAMES[FLIGHT_STATE_COUNT] = {
    "INIT", "PREFLIGHT", "ARMED", "FLYING", "LANDING", "FAULT"
};

/* ------------------------------------------------------------------ */
/*  Fault flags — set in the FreeRTOS EventGroup, one bit each         */
/* ------------------------------------------------------------------ */

#define FAULT_FLAG_GPS_LOSS        (1u << 0)
#define FAULT_FLAG_SENSOR_FREEZE   (1u << 1)
#define FAULT_FLAG_BATT_LOW        (1u << 2)
#define FAULT_FLAG_COMMS_TIMEOUT   (1u << 3)
#define FAULT_FLAG_WATCHDOG_NEAR   (1u << 4)
#define FAULT_FLAG_STACK_OVERFLOW  (1u << 5)

/* Thresholds */
#define BATT_LOW_VOLTAGE_V         (10.5f)
#define BATT_CRITICAL_VOLTAGE_V    (9.8f)
#define SENSOR_FREEZE_TIMEOUT_MS   (500u)
#define COMMS_TIMEOUT_MS           (1000u)
#define GPS_MIN_SATELLITES         (4u)

#endif /* SENSOR_DATA_H */
