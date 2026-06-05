#ifndef BLACKBOX_H
#define BLACKBOX_H

#include <stdint.h>
#include <stddef.h>
#include "sensor_data.h"

/* ------------------------------------------------------------------ */
/*  Ring buffer configuration                                           */
/*  512 entries × 64 bytes = 32 KB SRAM (fits STM32G071's 36 KB)      */
/* ------------------------------------------------------------------ */

#define BLACKBOX_CAPACITY   (512u)
#define BLACKBOX_MSG_LEN    (32u)

/* ------------------------------------------------------------------ */
/*  Event type codes                                                    */
/* ------------------------------------------------------------------ */

typedef enum {
    BB_EVT_BOOT           = 0x01,
    BB_EVT_STATE_CHANGE   = 0x02,
    BB_EVT_FAULT_SET      = 0x03,
    BB_EVT_FAULT_CLEAR    = 0x04,
    BB_EVT_SENSOR_UPDATE  = 0x05,
    BB_EVT_COMMS_TIMEOUT  = 0x06,
    BB_EVT_WATCHDOG_FEED  = 0x07,
    BB_EVT_HEALTH_SNAPSHOT= 0x08,
    BB_EVT_USER           = 0xFF,
} bb_event_type_t;

/* ------------------------------------------------------------------ */
/*  Single log record (64 bytes)                                        */
/* ------------------------------------------------------------------ */

typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint8_t  event_type;        /* bb_event_type_t */
    uint8_t  flight_state;      /* flight_state_t  */
    uint8_t  fault_flags;
    uint8_t  sensor_status;
    float    battery_voltage_v;
    float    altitude_m;
    float    accel_magnitude;   /* |accel| m/s²    */
    char     message[BLACKBOX_MSG_LEN];
    uint16_t sequence;          /* monotonic record counter */
    uint8_t  _pad[2];
} bb_record_t;
/* static_assert below in blackbox.c verifies sizeof == 64 */

/* ------------------------------------------------------------------ */
/*  Ring buffer handle                                                  */
/* ------------------------------------------------------------------ */

typedef struct {
    bb_record_t  entries[BLACKBOX_CAPACITY];
    uint16_t     write_idx;      /* next slot to write */
    uint16_t     read_idx;       /* next slot to read (for streaming) */
    uint16_t     count;          /* records currently held */
    uint16_t     sequence;       /* global monotonic counter */
    bool         overflow;       /* set when we wrap and overwrite */
} blackbox_t;

/* ------------------------------------------------------------------ */
/*  API                                                                 */
/* ------------------------------------------------------------------ */

/* Initialise the black-box buffer — call once in main before tasks    */
void blackbox_init(blackbox_t *bb);

/* Write a log record. Thread-safe — uses a FreeRTOS mutex internally. */
void blackbox_log(blackbox_t *bb,
                  bb_event_type_t type,
                  flight_state_t  state,
                  uint8_t         fault_flags,
                  uint8_t         sensor_status,
                  float           voltage,
                  float           altitude,
                  float           accel_mag,
                  const char     *msg);

/* Read the oldest unread record into *out.
 * Returns true if a record was available. */
bool blackbox_read(blackbox_t *bb, bb_record_t *out);

/* Peek at a record by absolute index without advancing read pointer. */
bool blackbox_peek(const blackbox_t *bb, uint16_t idx, bb_record_t *out);

/* How many records are currently stored. */
uint16_t blackbox_count(const blackbox_t *bb);

/* Reset read pointer to oldest entry (for full replay). */
void blackbox_rewind(blackbox_t *bb);

#endif /* BLACKBOX_H */
