#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include "sensor_data.h"

/* ------------------------------------------------------------------ */
/*  Frame format                                                        */
/*                                                                      */
/*   [SOF 1B][TYPE 1B][LEN 2B][PAYLOAD nB][CRC16 2B]                  */
/*                                                                      */
/*  SOF  = 0xAB  (start-of-frame magic byte)                           */
/*  TYPE = packet type ID (see PKT_TYPE_*)                             */
/*  LEN  = payload length in bytes, little-endian                      */
/*  CRC  = CRC-16/CCITT over TYPE+LEN+PAYLOAD, little-endian          */
/* ------------------------------------------------------------------ */

#define PROTO_SOF           (0xABu)
#define PROTO_HEADER_SIZE   (4u)    /* SOF + TYPE + LEN(2) */
#define PROTO_CRC_SIZE      (2u)
#define PROTO_MAX_PAYLOAD   (128u)
#define PROTO_MAX_FRAME     (PROTO_HEADER_SIZE + PROTO_MAX_PAYLOAD + PROTO_CRC_SIZE)

/* ------------------------------------------------------------------ */
/*  Packet type IDs                                                     */
/* ------------------------------------------------------------------ */

typedef enum {
    /* Inbound (QEMU → STM32) */
    PKT_TYPE_SENSOR_FRAME    = 0x10,   /* full sensor_data_t snapshot   */
    PKT_TYPE_FAULT_INJECT    = 0x11,   /* fault injection command        */
    PKT_TYPE_HEARTBEAT_REQ   = 0x12,   /* ping from simulator            */

    /* Outbound (STM32 → GCS) */
    PKT_TYPE_TELEMETRY       = 0x20,   /* flight telemetry frame         */
    PKT_TYPE_HEARTBEAT_ACK   = 0x21,   /* response to heartbeat request  */
    PKT_TYPE_BLACKBOX_ENTRY  = 0x22,   /* streamed black-box log record  */
    PKT_TYPE_HEALTH_REPORT   = 0x23,   /* RTOS task health snapshot      */
} pkt_type_t;

/* ------------------------------------------------------------------ */
/*  Inbound payload structs                                             */
/* ------------------------------------------------------------------ */

/* PKT_TYPE_SENSOR_FRAME: payload is a packed sensor_data_t */
/* (serialised field by field — see protocol.c for pack/unpack)        */

typedef struct __attribute__((packed)) {
    uint8_t fault_id;      /* FAULT_FLAG_* bitmask to inject   */
    uint8_t active;        /* 1 = inject, 0 = clear            */
    uint32_t duration_ms;  /* 0 = latched, >0 = auto-clear     */
} pkt_fault_inject_t;

/* ------------------------------------------------------------------ */
/*  Outbound payload structs                                            */
/* ------------------------------------------------------------------ */

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint8_t  flight_state;       /* flight_state_t          */
    uint8_t  fault_flags;        /* active FAULT_FLAG_* bits */
    float    altitude_m;
    float    ground_speed_mps;
    float    battery_voltage_v;
    uint8_t  battery_pct;
    float    accel_x, accel_y, accel_z;
    float    gyro_x,  gyro_y,  gyro_z;
    double   latitude, longitude;
    uint8_t  gps_fix_quality;
    uint8_t  gps_satellites;
    float    temperature_c;
    uint8_t  sensor_status;
} pkt_telemetry_t;

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint8_t  event_type;         /* BLACKBOX_EVT_* */
    uint8_t  flight_state;
    uint8_t  fault_flags;
    uint8_t  sensor_status;
    float    battery_voltage_v;
    float    altitude_m;
    char     message[32];
} pkt_blackbox_entry_t;

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint8_t  cpu_load_pct;
    /* Per-task stack high-water marks (words remaining) */
    uint16_t hwm_sensor;
    uint16_t hwm_fsm;
    uint16_t hwm_telemetry;
    uint16_t hwm_health;
    uint32_t free_heap_bytes;
} pkt_health_report_t;

/* ------------------------------------------------------------------ */
/*  Raw frame buffer with header + payload + CRC                        */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t  type;
    uint16_t len;                         /* payload length */
    uint8_t  payload[PROTO_MAX_PAYLOAD];
} proto_frame_t;

/* ------------------------------------------------------------------ */
/*  RX state machine states (used by task_sensor DMA parser)           */
/* ------------------------------------------------------------------ */

typedef enum {
    RX_STATE_WAIT_SOF = 0,
    RX_STATE_WAIT_TYPE,
    RX_STATE_WAIT_LEN_LO,
    RX_STATE_WAIT_LEN_HI,
    RX_STATE_WAIT_PAYLOAD,
    RX_STATE_WAIT_CRC_LO,
    RX_STATE_WAIT_CRC_HI,
} rx_state_t;

typedef struct {
    rx_state_t state;
    proto_frame_t frame;
    uint16_t   payload_idx;
    uint8_t    crc_lo;
} proto_rx_ctx_t;

/* ------------------------------------------------------------------ */
/*  API                                                                 */
/* ------------------------------------------------------------------ */

/* CRC-16/CCITT (poly 0x1021, init 0xFFFF) */
uint16_t proto_crc16(const uint8_t *data, size_t len);

/* Serialise a sensor_data_t into a payload buffer.
 * Returns number of bytes written. */
size_t proto_pack_sensor(const sensor_data_t *s, uint8_t *buf, size_t buf_size);

/* Deserialise payload buffer into sensor_data_t.
 * Returns true on success. */
bool proto_unpack_sensor(const uint8_t *buf, size_t len, sensor_data_t *out);

/* Serialise a telemetry frame from sensor + state into output buffer.
 * Writes full wire frame (header + payload + CRC).
 * Returns total bytes written (ready to UART-TX). */
size_t proto_build_telemetry(const sensor_data_t *s,
                              flight_state_t state,
                              uint8_t fault_flags,
                              uint8_t *out_buf,
                              size_t   out_size);

/* Serialise a health report into output buffer (full wire frame).
 * Returns total bytes written. */
size_t proto_build_health(const pkt_health_report_t *h,
                           uint8_t *out_buf,
                           size_t   out_size);

/* Feed one byte into the RX state machine.
 * Returns true when a complete, CRC-valid frame is ready in ctx->frame. */
bool proto_rx_feed(proto_rx_ctx_t *ctx, uint8_t byte);

/* Reset RX state machine (call after frame consumed or on timeout) */
void proto_rx_reset(proto_rx_ctx_t *ctx);

#endif /* PROTOCOL_H */
