#ifndef SIM_TYPES_H
#define SIM_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ------------------------------------------------------------------ */
/*  Mirror of firmware sensor_data_t                                    */
/*  Must stay byte-for-byte compatible with the STM32 struct           */
/* ------------------------------------------------------------------ */

typedef struct {
    float    accel_x, accel_y, accel_z;     /* m/s²          */
    float    gyro_x,  gyro_y,  gyro_z;      /* deg/s         */
    double   latitude, longitude;
    float    altitude_m;
    float    ground_speed_mps;
    uint8_t  gps_fix_quality;
    uint8_t  gps_satellites;
    float    battery_voltage_v;
    float    battery_current_a;
    uint8_t  battery_pct;
    float    baro_pressure_hpa;
    float    temperature_c;
    uint32_t timestamp_ms;
    uint8_t  sensor_status;
} sim_sensor_data_t;

/* sensor_status bitmask — mirrors firmware SENSOR_FLAG_* */
#define SIM_SENSOR_ACCEL_OK   (1u << 0)
#define SIM_SENSOR_GYRO_OK    (1u << 1)
#define SIM_SENSOR_GPS_OK     (1u << 2)
#define SIM_SENSOR_BATT_OK    (1u << 3)
#define SIM_SENSOR_BARO_OK    (1u << 4)
#define SIM_SENSOR_ALL_OK     (0x1Fu)

/* ------------------------------------------------------------------ */
/*  Protocol constants (mirror firmware protocol.h)                    */
/* ------------------------------------------------------------------ */

#define PROTO_SOF              (0xABu)
#define PROTO_HEADER_SIZE      (4u)
#define PROTO_CRC_SIZE         (2u)
#define PROTO_MAX_PAYLOAD      (128u)

#define PKT_TYPE_SENSOR_FRAME  (0x10u)
#define PKT_TYPE_FAULT_INJECT  (0x11u)
#define PKT_TYPE_HEARTBEAT_REQ (0x12u)
#define PKT_TYPE_HEARTBEAT_ACK (0x21u)

/* ------------------------------------------------------------------ */
/*  Fault flags — mirror firmware FAULT_FLAG_*                         */
/* ------------------------------------------------------------------ */

#define FAULT_GPS_LOSS         (1u << 0)
#define FAULT_SENSOR_FREEZE    (1u << 1)
#define FAULT_BATT_LOW         (1u << 2)
#define FAULT_COMMS_TIMEOUT    (1u << 3)

/* ------------------------------------------------------------------ */
/*  Simulator configuration                                             */
/* ------------------------------------------------------------------ */

#define SIM_TICK_RATE_HZ       (100u)
#define SIM_TICK_MS            (1000u / SIM_TICK_RATE_HZ)
#define SIM_UART_TX_RATE_HZ    (50u)   /* sensor frames per second    */
#define SIM_TCP_PORT           (5760u) /* GCS command/fault-inject port*/

typedef struct {
    char   uart_device[64];   /* e.g. "/dev/ttyS0" or "/dev/pts/3"  */
    int    uart_baud;         /* 115200                               */
    char   scenario_file[256];
    bool   tcp_enabled;
    int    tcp_port;
    bool   verbose;
} sim_config_t;

#endif /* SIM_TYPES_H */
